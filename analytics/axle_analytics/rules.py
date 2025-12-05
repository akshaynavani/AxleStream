"""Diagnostic thresholds and pure-Python event classification.

The Spark jobs build column expressions from the same constants, so the definitions here
are the single source of truth. `classify_frame` mirrors the Spark logic for unit tests
and for quick local analysis of a JSONL file without a Spark session.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Iterable, Iterator, Mapping

GRAVITY = 9.80665


@dataclass(frozen=True)
class Thresholds:
    harsh_brake_mps2: float = -3.5       # longitudinal decel beyond this is "harsh braking"
    harsh_accel_mps2: float = 3.0
    harsh_lateral_mps2: float = 4.0      # aggressive cornering
    ttc_warning_s: float = 2.0           # forward collision warning
    imu_norm_low: float = 8.0            # |f| far from 1 g suggests IMU fault / impact
    imu_norm_high: float = 11.8
    max_roll_rad: float = 0.25           # ~14 deg: rollover risk / sensor mount issue
    latency_budget_ms: float = 10.0
    min_lidar_points: int = 1000         # fewer points per sweep => blocked/dirty sensor


DEFAULT = Thresholds()

EVENT_TYPES = (
    "harsh_brake",
    "harsh_accel",
    "harsh_lateral",
    "forward_collision_warning",
    "imu_anomaly",
    "excessive_roll",
    "latency_breach",
    "lidar_degraded",
    "can_errors",
)


def _num(rec: Mapping, key: str):
    v = rec.get(key)
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def classify_frame(rec: Mapping, t: Thresholds = DEFAULT, prev_can_errors: int | None = None) -> list[str]:
    """Return the diagnostic event types raised by one exported frame record."""
    events: list[str] = []

    long_a = _num(rec, "long_accel_mps2")
    if long_a is not None:
        if long_a <= t.harsh_brake_mps2:
            events.append("harsh_brake")
        elif long_a >= t.harsh_accel_mps2:
            events.append("harsh_accel")

    lat_a = _num(rec, "lat_accel_mps2")
    if lat_a is not None and abs(lat_a) >= t.harsh_lateral_mps2:
        events.append("harsh_lateral")

    ttc = _num(rec, "ttc_s")
    if ttc is not None and 0 <= ttc < t.ttc_warning_s:
        events.append("forward_collision_warning")

    norm = _num(rec, "accel_norm_mps2")
    if norm is not None and not (t.imu_norm_low <= norm <= t.imu_norm_high):
        events.append("imu_anomaly")

    roll = _num(rec, "roll_rad")
    if roll is not None and abs(roll) > t.max_roll_rad:
        events.append("excessive_roll")

    lat_ms = _num(rec, "e2e_latency_ms")
    if rec.get("over_budget") is True or (lat_ms is not None and lat_ms > t.latency_budget_ms):
        events.append("latency_breach")

    raw = _num(rec, "raw_points")
    if raw is not None and raw < t.min_lidar_points:
        events.append("lidar_degraded")

    errs = _num(rec, "can_decode_errors")
    if errs is not None and prev_can_errors is not None and errs > prev_can_errors:
        events.append("can_errors")

    return events


def read_jsonl(paths: Iterable[str]) -> Iterator[dict]:
    for p in paths:
        with open(p, "r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if line:
                    yield json.loads(line)


def percentile(values: list[float], p: float) -> float:
    """Linear-interpolated percentile (matches Spark's percentile, not percentile_approx)."""
    if not values:
        return float("nan")
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return s[int(k)]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def summarise_local(records: Iterable[Mapping], t: Thresholds = DEFAULT) -> dict:
    """Small local equivalent of the gold per-vehicle summary (no Spark required)."""
    per_vehicle: dict[str, dict] = {}
    prev_errs: dict[str, int] = {}
    for r in records:
        vid = r.get("vehicle_id", "unknown")
        v = per_vehicle.setdefault(vid, {"frames": 0, "latencies": [], "events": {e: 0 for e in EVENT_TYPES},
                                         "distance_m": 0.0})
        v["frames"] += 1
        lat = _num(r, "e2e_latency_ms")
        if lat is not None:
            v["latencies"].append(lat)
        speed = _num(r, "speed_mps")
        if speed is not None:
            v["distance_m"] += speed * 0.1  # 10 Hz frames
        for e in classify_frame(r, t, prev_errs.get(vid)):
            v["events"][e] += 1
        errs = _num(r, "can_decode_errors")
        if errs is not None:
            prev_errs[vid] = int(errs)

    out = {}
    for vid, v in per_vehicle.items():
        lats = v.pop("latencies")
        v["p50_latency_ms"] = percentile(lats, 50)
        v["p99_latency_ms"] = percentile(lats, 99)
        v["slo_compliance"] = (sum(1 for x in lats if x <= t.latency_budget_ms) / len(lats)) if lats else float("nan")
        out[vid] = v
    return out
