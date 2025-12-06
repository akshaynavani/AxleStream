"""Run the bronze -> silver -> gold flow on a local SparkSession (no Databricks needed).

    python analytics/jobs/local_run.py --src telemetry_out --out analytics_out

Reads the exporter's JSONL files directly, applies the same transformations as the
Databricks jobs, writes each gold table as Parquet (or Delta if delta-spark is installed)
under --out, and prints the per-vehicle daily summary.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pyspark.sql import SparkSession  # noqa: E402
from pyspark.sql import functions as F  # noqa: E402

from axle_analytics.diagnostics import (  # noqa: E402
    event_log,
    fleet_overview,
    latency_slo,
    sensor_health,
    to_silver,
    vehicle_daily_summary,
)
from axle_analytics.rules import Thresholds  # noqa: E402
from axle_analytics.schema import FRAME_SCHEMA  # noqa: E402


def build_spark(use_delta: bool) -> SparkSession:
    builder = (SparkSession.builder.appName("axlestream-local").master("local[*]")
               .config("spark.sql.session.timeZone", "UTC")
               .config("spark.sql.shuffle.partitions", "8"))
    if use_delta:
        from delta import configure_spark_with_delta_pip

        builder = (builder
                   .config("spark.sql.extensions", "io.delta.sql.DeltaSparkSessionExtension")
                   .config("spark.sql.catalog.spark_catalog", "org.apache.spark.sql.delta.catalog.DeltaCatalog"))
        return configure_spark_with_delta_pip(builder).getOrCreate()
    return builder.getOrCreate()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="telemetry_out")
    ap.add_argument("--out", default="analytics_out")
    ap.add_argument("--budget-ms", type=float, default=10.0)
    ap.add_argument("--delta", action="store_true", help="write Delta instead of Parquet")
    args = ap.parse_args(argv)

    spark = build_spark(args.delta)
    fmt = "delta" if args.delta else "parquet"

    bronze = (spark.read.schema(FRAME_SCHEMA)
              .option("recursiveFileLookup", "true")
              .option("pathGlobFilter", "*.jsonl")
              .json(os.path.join(args.src, "frames"))
              .withColumn("_ingested_at", F.current_timestamp()))
    silver = to_silver(bronze, Thresholds(latency_budget_ms=args.budget_ms)).cache()
    print(f"frames: {silver.count()}  vehicles: {silver.select('vehicle_id').distinct().count()}")

    daily = vehicle_daily_summary(silver)
    tables = {
        "silver_frames": silver,
        "gold_vehicle_daily": daily,
        "gold_latency_slo": latency_slo(silver, args.budget_ms),
        "gold_events": event_log(silver),
        "gold_sensor_health": sensor_health(silver),
        "gold_fleet_overview": fleet_overview(daily),
    }
    for name, df in tables.items():
        df.write.format(fmt).mode("overwrite").save(os.path.join(args.out, name))

    (daily.select("vehicle_id", "event_date", "frames", "distance_km", "p50_latency_ms", "p99_latency_ms",
                  "slo_compliance", "harsh_brake_events", "forward_collision_warning_events", "can_errors_events")
     .orderBy("vehicle_id").show(truncate=False))
    tables["gold_events"].groupBy("event_type").count().orderBy("event_type").show()
    spark.stop()


if __name__ == "__main__":
    main()
