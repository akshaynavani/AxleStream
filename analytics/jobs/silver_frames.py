"""Silver: clean, de-duplicate and flag frames; MERGE into the silver table.

The MERGE on (vehicle_id, run_id, seq) makes the job idempotent: re-processing the same
bronze rows never creates duplicates. Only bronze rows ingested since the last successful
run (tracked via the max `_ingested_at` already in silver) are processed.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pyspark.sql import SparkSession  # noqa: E402
from pyspark.sql import functions as F  # noqa: E402

from axle_analytics.config import load_settings  # noqa: E402
from axle_analytics.diagnostics import to_silver  # noqa: E402
from axle_analytics.rules import Thresholds  # noqa: E402


def main(argv=None) -> None:
    settings = load_settings()
    ap = argparse.ArgumentParser()
    ap.add_argument("--bronze", default=settings.table("bronze_frames"))
    ap.add_argument("--silver", default=settings.table("silver_frames"))
    ap.add_argument("--full-refresh", action="store_true")
    args = ap.parse_args(argv)

    spark = SparkSession.builder.appName("axlestream-silver").getOrCreate()
    thresholds = Thresholds(latency_budget_ms=settings.latency_budget_ms)

    bronze = spark.table(args.bronze)
    silver_exists = spark.catalog.tableExists(args.silver)

    if silver_exists and not args.full_refresh:
        watermark = spark.table(args.silver).agg(F.max("_ingested_at")).first()[0]
        if watermark is not None:
            bronze = bronze.where(F.col("_ingested_at") > F.lit(watermark))

    silver = to_silver(bronze, thresholds)

    if not silver_exists or args.full_refresh:
        (silver.write.format("delta").mode("overwrite")
         .option("overwriteSchema", "true")
         .partitionBy("event_date")
         .saveAsTable(args.silver))
        print(f"silver: wrote {args.silver}")
        return

    from delta.tables import DeltaTable

    target = DeltaTable.forName(spark, args.silver)
    (
        target.alias("t")
        .merge(silver.alias("s"), "t.vehicle_id = s.vehicle_id AND t.run_id = s.run_id AND t.seq = s.seq")
        .whenNotMatchedInsertAll()
        .execute()
    )
    print(f"silver: merged new frames into {args.silver}")


if __name__ == "__main__":
    main()
