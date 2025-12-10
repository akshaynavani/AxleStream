import json
import math

from axle_analytics.rules import Thresholds, classify_frame, percentile, read_jsonl, summarise_local


def frame(**kw):
    base = {
        "vehicle_id": "veh-001",
        "run_id": "r1",
        "seq": 0,
        "e2e_latency_ms": 3.0,
        "over_budget": False,
        "raw_points": 9000,
        "accel_norm_mps2": 9.81,
        "long_accel_mps2": 0.0,
        "lat_accel_mps2": 0.0,
        "roll_rad": 0.0,
        "ttc_s": None,
        "speed_mps": 10.0,
        "can_decode_errors": 0,
    }
    base.update(kw)
    return base


def test_nominal_frame_raises_nothing():
    assert classify_frame(frame()) == []


def test_harsh_brake_and_accel_are_exclusive():
    assert classify_frame(frame(long_accel_mps2=-4.2)) == ["harsh_brake"]
    assert classify_frame(frame(long_accel_mps2=3.5)) == ["harsh_accel"]


def test_lateral_uses_absolute_value():
    assert "harsh_lateral" in classify_frame(frame(lat_accel_mps2=-4.5))


def test_forward_collision_warning_needs_valid_ttc():
    assert "forward_collision_warning" in classify_frame(frame(ttc_s=1.2))
    assert "forward_collision_warning" not in classify_frame(frame(ttc_s=None))
    assert "forward_collision_warning" not in classify_frame(frame(ttc_s=5.0))


def test_imu_anomaly_outside_band():
    assert "imu_anomaly" in classify_frame(frame(accel_norm_mps2=14.0))
    assert "imu_anomaly" in classify_frame(frame(accel_norm_mps2=2.0))


def test_latency_breach_from_flag_or_value():
    assert "latency_breach" in classify_frame(frame(over_budget=True, e2e_latency_ms=1.0))
    assert "latency_breach" in classify_frame(frame(e2e_latency_ms=12.0))
    t = Thresholds(latency_budget_ms=20.0)
    assert "latency_breach" not in classify_frame(frame(e2e_latency_ms=12.0), t)


def test_can_errors_only_when_counter_grows():
    assert "can_errors" not in classify_frame(frame(can_decode_errors=3))
    assert "can_errors" in classify_frame(frame(can_decode_errors=3), prev_can_errors=2)
    assert "can_errors" not in classify_frame(frame(can_decode_errors=3), prev_can_errors=3)


def test_lidar_degraded():
    assert "lidar_degraded" in classify_frame(frame(raw_points=120))


def test_percentile_interpolates():
    assert percentile([1, 2, 3, 4], 50) == 2.5
    assert percentile([5], 99) == 5
    assert math.isnan(percentile([], 50))


def test_summarise_local_per_vehicle(tmp_path):
    rows = [frame(seq=i, e2e_latency_ms=2.0 + i * 0.05) for i in range(100)]
    rows += [frame(vehicle_id="veh-002", seq=i, long_accel_mps2=-5.0, e2e_latency_ms=15.0) for i in range(10)]
    path = tmp_path / "part.jsonl"
    path.write_text("\n".join(json.dumps(r) for r in rows) + "\n")

    summary = summarise_local(read_jsonl([str(path)]))
    v1, v2 = summary["veh-001"], summary["veh-002"]
    assert v1["frames"] == 100
    assert v1["slo_compliance"] == 1.0
    assert abs(v1["distance_m"] - 100.0) < 1e-6  # 10 m/s for 10 s
    assert v2["events"]["harsh_brake"] == 10
    assert v2["events"]["latency_breach"] == 10
    assert v2["slo_compliance"] == 0.0
