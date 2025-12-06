"""PySpark transformations for the medallion layers.

bronze  raw frame records as ingested (+ source file, ingest time)
silver  typed, de-duplicated frames with derived columns and per-frame event flags
gold    fleet-level tables: per-vehicle daily summary, latency SLO, event log, sensor health

Every function here is a pure DataFrame -> DataFrame transformation so it can be unit
tested on a local SparkSession and reused unchanged in Databricks jobs.
"""

from __future__ import annotations

from pyspark.sql import DataFrame, Window
from pyspark.sql import functions as F

from .rules import DEFAULT, Thresholds

FRAME_PERIOD_S = 0.1  # LiDAR-driven frames at 10 Hz


# ---------------------------------------------------------------------------------------
# Silver
# ---------------------------------------------------------------------------------------

def to_silver(bronze: DataFrame, t: Thresholds = DEFAULT) -> DataFrame:
    """Clean, de-duplicate and enrich raw frames."""
    df = (
        bronze
        .where(F.col("vehicle_id").isNotNull() & F.col("sensor_ts_ns").isNotNull())
        # Re-uploads or replayed batches can duplicate frames; keep one per key.
        .dropDuplicates(["vehicle_id", "run_id", "seq"])
        .withColumn("event_ts", F.timestamp_micros((F.col("sensor_ts_ns") / 1000).cast("long")))
        .withColumn("event_date", F.to_date("event_ts"))
        .withColumn("speed_kph", F.col("speed_mps") * 3.6)
        .withColumn("ground_ratio",
                    F.when(F.col("filtered_points") > 0,
                           F.col("ground_points") / F.col("filtered_points")))
        .withColumn("over_budget",
                    F.coalesce(F.col("over_budget"), F.col("e2e_latency_ms") > F.lit(t.latency_budget_ms)))
    )

    # CAN decode errors are a running counter per run; a frame raises "can_errors" when it grew.
    w = Window.partitionBy("vehicle_id", "run_id").orderBy("seq")
    df = df.withColumn("can_errors_delta",
                       F.col("can_decode_errors") - F.lag("can_decode_errors").over(w))

    flags = {
        "harsh_brake": F.col("long_accel_mps2") <= t.harsh_brake_mps2,
        "harsh_accel": F.col("long_accel_mps2") >= t.harsh_accel_mps2,
        "harsh_lateral": F.abs(F.col("lat_accel_mps2")) >= t.harsh_lateral_mps2,
        "forward_collision_warning": (F.col("ttc_s") >= 0) & (F.col("ttc_s") < t.ttc_warning_s),
        "imu_anomaly": (F.col("accel_norm_mps2") < t.imu_norm_low) | (F.col("accel_norm_mps2") > t.imu_norm_high),
        "excessive_roll": F.abs(F.col("roll_rad")) > t.max_roll_rad,
        "latency_breach": F.col("over_budget"),
        "lidar_degraded": F.col("raw_points") < t.min_lidar_points,
        "can_errors": F.col("can_errors_delta") > 0,
    }
    for name, cond in flags.items():
        df = df.withColumn(f"is_{name}", F.coalesce(cond, F.lit(False)))

    # Harsh braking is the event; harsh acceleration should not fire on the same frame.
    df = df.withColumn("is_harsh_accel", F.col("is_harsh_accel") & ~F.col("is_harsh_brake"))
    return df


EVENT_COLUMNS = [
    "harsh_brake",
    "harsh_accel",
    "harsh_lateral",
    "forward_collision_warning",
    "imu_anomaly",
    "excessive_roll",
    "latency_breach",
    "lidar_degraded",
    "can_errors",
]


# ---------------------------------------------------------------------------------------
# Gold
# ---------------------------------------------------------------------------------------

def vehicle_daily_summary(silver: DataFrame) -> DataFrame:
    """One row per vehicle per day: utilisation, driving behaviour and pipeline health."""
    aggs = [
        F.count("*").alias("frames"),
        F.countDistinct("run_id").alias("runs"),
        (F.sum(F.coalesce("speed_mps", F.lit(0.0))) * FRAME_PERIOD_S / 1000.0).alias("distance_km"),
        F.max("speed_kph").alias("max_speed_kph"),
        F.avg("speed_kph").alias("avg_speed_kph"),
        F.expr("percentile(e2e_latency_ms, 0.5)").alias("p50_latency_ms"),
        F.expr("percentile(e2e_latency_ms, 0.99)").alias("p99_latency_ms"),
        F.max("e2e_latency_ms").alias("max_latency_ms"),
        F.avg(F.when(F.col("is_latency_breach"), 0.0).otherwise(1.0)).alias("slo_compliance"),
        F.avg("raw_points").alias("avg_lidar_points"),
        F.avg("num_obstacles").alias("avg_obstacles"),
        F.min("ttc_s").alias("min_ttc_s"),
    ]
    aggs += [F.sum(F.col(f"is_{e}").cast("int")).alias(f"{e}_events") for e in EVENT_COLUMNS]
    return silver.groupBy("vehicle_id", "event_date").agg(*aggs)


def latency_slo(silver: DataFrame, budget_ms: float = DEFAULT.latency_budget_ms) -> DataFrame:
    """Per vehicle and hour: latency percentiles against the per-frame budget."""
    return (
        silver
        .withColumn("hour", F.date_trunc("hour", "event_ts"))
        .groupBy("vehicle_id", "hour")
        .agg(
            F.count("*").alias("frames"),
            F.expr("percentile(e2e_latency_ms, 0.5)").alias("p50_ms"),
            F.expr("percentile(e2e_latency_ms, 0.95)").alias("p95_ms"),
            F.expr("percentile(e2e_latency_ms, 0.99)").alias("p99_ms"),
            F.max("e2e_latency_ms").alias("max_ms"),
            F.expr("percentile(lidar_proc_ms, 0.99)").alias("lidar_p99_ms"),
            F.sum(F.col("is_latency_breach").cast("int")).alias("breaches"),
        )
        .withColumn("budget_ms", F.lit(budget_ms))
        .withColumn("p99_within_budget", F.col("p99_ms") <= F.col("budget_ms"))
    )


def event_log(silver: DataFrame, min_gap_s: float = 2.0) -> DataFrame:
    """Collapse consecutive flagged frames into discrete events with start/end/peak.

    Frames of the same event type closer than `min_gap_s` belong to one event, so a single
    2-second braking manoeuvre yields one row rather than twenty.
    """
    stacked = None
    for e in EVENT_COLUMNS:
        part = (
            silver.where(F.col(f"is_{e}"))
            .select(
                "vehicle_id", "run_id", "seq", "event_ts",
                F.lit(e).alias("event_type"),
                "speed_kph", "long_accel_mps2", "lat_accel_mps2", "ttc_s", "e2e_latency_ms",
                "nearest_obstacle_m",
            )
        )
        stacked = part if stacked is None else stacked.unionByName(part)

    w = Window.partitionBy("vehicle_id", "run_id", "event_type").orderBy("event_ts")
    gap = F.col("event_ts").cast("double") - F.lag(F.col("event_ts").cast("double")).over(w)
    grouped = (
        stacked
        .withColumn("new_event", F.when(gap.isNull() | (gap > min_gap_s), 1).otherwise(0))
        .withColumn("event_no", F.sum("new_event").over(w.rowsBetween(Window.unboundedPreceding, 0)))
    )
    return (
        grouped.groupBy("vehicle_id", "run_id", "event_type", "event_no")
        .agg(
            F.min("event_ts").alias("start_ts"),
            F.max("event_ts").alias("end_ts"),
            F.count("*").alias("frames"),
            F.max("speed_kph").alias("max_speed_kph"),
            F.min("long_accel_mps2").alias("min_long_accel_mps2"),
            F.max(F.abs("lat_accel_mps2")).alias("max_abs_lat_accel_mps2"),
            F.min("ttc_s").alias("min_ttc_s"),
            F.max("e2e_latency_ms").alias("max_latency_ms"),
            F.min("nearest_obstacle_m").alias("min_obstacle_m"),
        )
        .withColumn("duration_s", F.col("end_ts").cast("double") - F.col("start_ts").cast("double"))
        .drop("event_no")
    )


def sensor_health(silver: DataFrame) -> DataFrame:
    """Per vehicle and day: indicators of degrading sensors for maintenance triage."""
    return (
        silver.groupBy("vehicle_id", "event_date")
        .agg(
            F.avg("raw_points").alias("avg_lidar_points"),
            F.expr("percentile(raw_points, 0.05)").alias("p05_lidar_points"),
            F.avg("ground_ratio").alias("avg_ground_ratio"),
            F.avg(F.col("imu_valid").cast("double")).alias("imu_match_rate"),
            F.avg(F.col("vehicle_valid").cast("double")).alias("can_match_rate"),
            F.stddev("accel_norm_mps2").alias("accel_norm_stddev"),
            F.sum(F.col("is_imu_anomaly").cast("int")).alias("imu_anomalies"),
            F.max("can_decode_errors").alias("can_decode_errors"),
        )
        .withColumn(
            "health_flag",
            F.when(F.col("p05_lidar_points") < DEFAULT.min_lidar_points, "lidar_check")
            .when(F.col("imu_match_rate") < 0.95, "imu_link_check")
            .when(F.col("can_match_rate") < 0.95, "can_link_check")
            .when(F.col("imu_anomalies") > 0, "imu_calibration")
            .otherwise("ok"),
        )
    )


def fleet_overview(daily: DataFrame) -> DataFrame:
    """Fleet-wide roll-up per day, ranking vehicles by risk events per 100 km."""
    risk = (F.col("harsh_brake_events") + F.col("harsh_lateral_events") +
            F.col("forward_collision_warning_events"))
    per_vehicle = daily.withColumn(
        "risk_per_100km", F.when(F.col("distance_km") > 0, risk / F.col("distance_km") * 100.0))
    w = Window.partitionBy("event_date").orderBy(F.col("risk_per_100km").desc_nulls_last())
    return per_vehicle.withColumn("risk_rank", F.rank().over(w))
