"""Spark transformation tests; skipped automatically when pyspark (or Java) is unavailable."""

import pytest

pyspark = pytest.importorskip("pyspark")

from pyspark.sql import SparkSession  # noqa: E402

from axle_analytics.diagnostics import (  # noqa: E402
    event_log,
    fleet_overview,
    latency_slo,
    sensor_health,
    to_silver,
    vehicle_daily_summary,
)
from axle_analytics.schema import FRAME_SCHEMA  # noqa: E402

BASE_TS = 1_714_521_600_000_000_000  # 2024-05-01T00:00:00Z


@pytest.fixture(scope="module")
def spark():
    try:
        s = (SparkSession.builder.master("local[2]").appName("axle-tests")
             .config("spark.sql.session.timeZone", "UTC")
             .config("spark.sql.shuffle.partitions", "2")
             .getOrCreate())
    except Exception as exc:  # e.g. no JVM on the machine
        pytest.skip(f"Spark unavailable: {exc}")
    yield s
    s.stop()


def row(vid, seq, **kw):
    r = {f.name: None for f in FRAME_SCHEMA.fields}
    r.update({
        "vehicle_id": vid, "run_id": "run1", "seq": seq,
        "sensor_ts_ns": BASE_TS + seq * 100_000_000,
        "e2e_latency_ms": 3.0, "lidar_proc_ms": 2.5, "over_budget": False,
        "raw_points": 9000, "filtered_points": 4000, "ground_points": 3000, "num_obstacles": 2,
        "imu_valid": True, "vehicle_valid": True, "accel_norm_mps2": 9.81,
        "long_accel_mps2": 0.0, "lat_accel_mps2": 0.0, "roll_rad": 0.0,
        "speed_mps": 10.0, "can_decode_errors": 0, "obstacles": [],
    })
    r.update(kw)
    return r


@pytest.fixture(scope="module")
def silver(spark):
    rows = [row("veh-001", i) for i in range(100)]
    # veh-002: a 1-second harsh braking manoeuvre, one latency breach, CAN errors growing
    rows += [row("veh-002", i,
                 long_accel_mps2=-5.0 if 20 <= i < 30 else 0.0,
                 e2e_latency_ms=14.0 if i == 50 else 3.0,
                 over_budget=(i == 50),
                 can_decode_errors=0 if i < 60 else 2)
             for i in range(100)]
    rows.append(row("veh-001", 5))  # duplicate frame (re-upload)
    bronze = spark.createDataFrame(rows, FRAME_SCHEMA)
    return to_silver(bronze).cache()


def test_silver_deduplicates(silver):
    assert silver.count() == 200


def test_silver_flags(silver):
    v2 = silver.where("vehicle_id = 'veh-002'")
    assert v2.where("is_harsh_brake").count() == 10
    assert v2.where("is_latency_breach").count() == 1
    assert v2.where("is_can_errors").count() == 1  # only the frame where the counter grew
    assert silver.where("vehicle_id = 'veh-001' AND (is_harsh_brake OR is_latency_breach)").count() == 0


def test_daily_summary(silver):
    daily = {r["vehicle_id"]: r for r in vehicle_daily_summary(silver).collect()}
    assert daily["veh-001"]["frames"] == 100
    assert abs(daily["veh-001"]["distance_km"] - 0.1) < 1e-9  # 10 m/s * 10 s
    assert daily["veh-002"]["harsh_brake_events"] == 10
    assert daily["veh-002"]["slo_compliance"] == pytest.approx(0.99)


def test_event_log_merges_consecutive_frames(silver):
    events = event_log(silver).where("event_type = 'harsh_brake'").collect()
    assert len(events) == 1
    assert events[0]["frames"] == 10
    assert events[0]["duration_s"] == pytest.approx(0.9)


def test_latency_slo_and_health(silver):
    slo = latency_slo(silver, 10.0).collect()
    assert all(r["p99_within_budget"] for r in slo if r["vehicle_id"] == "veh-001")
    health = {r["vehicle_id"]: r["health_flag"] for r in sensor_health(silver).collect()}
    assert health["veh-001"] == "ok"


def test_fleet_overview_ranks_riskier_vehicle_first(silver):
    ranked = fleet_overview(vehicle_daily_summary(silver)).orderBy("risk_rank").collect()
    assert ranked[0]["vehicle_id"] == "veh-002"
