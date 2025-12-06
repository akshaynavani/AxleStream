"""Gold: fleet-level diagnostics tables rebuilt from silver.

Tables written (all under <catalog>.<schema>):
    gold_vehicle_daily     per vehicle/day utilisation, behaviour events, latency SLO
    gold_latency_slo       per vehicle/hour latency percentiles vs the per-frame budget
    gold_events            discrete diagnostic events (braking, FCW, IMU anomalies, ...)
    gold_sensor_health     per vehicle/day sensor degradation indicators + triage flag
    gold_fleet_overview    daily vehicle risk ranking (events per 100 km)
    gold_run_summaries     on-vehicle run summaries uploaded next to the frames
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pyspark.sql import SparkSession  # noqa: E402

from axle_analytics.config import load_settings  # noqa: E402
from axle_analytics.diagnostics import (  # noqa: E402
    event_log,
    fleet_overview,
    latency_slo,
    sensor_health,
    vehicle_daily_summary,
)


def main(argv=None) -> None:
    settings = load_settings()
    ap = argparse.ArgumentParser()
    ap.add_argument("--silver", default=settings.table("silver_frames"))
    ap.add_argument("--runs-path", default=settings.runs_path)
    ap.add_argument("--prefix", default=f"{settings.catalog}.{settings.schema}.gold_")
    args = ap.parse_args(argv)

    spark = SparkSession.builder.appName("axlestream-gold").getOrCreate()
    silver = spark.table(args.silver).cache()

    daily = vehicle_daily_summary(silver)
    outputs = {
        "vehicle_daily": daily,
        "latency_slo": latency_slo(silver, settings.latency_budget_ms),
        "events": event_log(silver),
        "sensor_health": sensor_health(silver),
        "fleet_overview": fleet_overview(daily),
    }
    for name, df in outputs.items():
        table = args.prefix + name
        df.write.format("delta").mode("overwrite").option("overwriteSchema", "true").saveAsTable(table)
        print(f"gold: wrote {table}")

    try:
        runs = spark.read.option("recursiveFileLookup", "true").json(args.runs_path)
        runs.write.format("delta").mode("overwrite").option("overwriteSchema", "true") \
            .saveAsTable(args.prefix + "run_summaries")
        print(f"gold: wrote {args.prefix}run_summaries")
    except Exception as exc:  # run summaries are optional
        print(f"gold: skipped run summaries ({exc})")

    silver.unpersist()


if __name__ == "__main__":
    main()
