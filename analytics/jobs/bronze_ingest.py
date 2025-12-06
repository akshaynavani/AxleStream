"""Bronze: incrementally ingest uploaded frame batches into a Delta table.

On Databricks this uses Auto Loader (`cloudFiles`) with an availableNow trigger, so each
scheduled run picks up exactly the files that arrived since the previous one. With
`--batch` it falls back to a plain batch read (useful for local Spark without Auto Loader).

Parameters (Databricks spark_python_task or CLI):
    --source PATH   frame files root (default: <volume>/frames)
    --table NAME    target table (default: <catalog>.<schema>.bronze_frames)
    --batch         batch-read every file instead of Auto Loader
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pyspark.sql import SparkSession  # noqa: E402
from pyspark.sql import functions as F  # noqa: E402

from axle_analytics.config import load_settings  # noqa: E402
from axle_analytics.schema import FRAME_SCHEMA  # noqa: E402


def main(argv=None) -> None:
    settings = load_settings()
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default=settings.frames_path)
    ap.add_argument("--table", default=settings.table("bronze_frames"))
    ap.add_argument("--checkpoint", default=settings.checkpoint("bronze_frames"))
    ap.add_argument("--batch", action="store_true")
    args = ap.parse_args(argv)

    spark = SparkSession.builder.appName("axlestream-bronze").getOrCreate()

    if args.batch:
        df = (
            spark.read.schema(FRAME_SCHEMA)
            .option("recursiveFileLookup", "true")
            .json(args.source)
            .withColumn("_source_file", F.input_file_name())
            .withColumn("_ingested_at", F.current_timestamp())
        )
        df.write.format("delta").mode("append").saveAsTable(args.table)
        print(f"bronze: appended {df.count()} rows to {args.table}")
        return

    stream = (
        spark.readStream.format("cloudFiles")
        .option("cloudFiles.format", "json")
        .option("cloudFiles.schemaLocation", args.checkpoint + "/schema")
        # vehicle_id is already inside every record; don't infer it from the path too.
        .option("cloudFiles.partitionColumns", "")
        .option("pathGlobFilter", "*.jsonl")
        .schema(FRAME_SCHEMA)
        .load(args.source)
        .withColumn("_source_file", F.col("_metadata.file_path"))
        .withColumn("_ingested_at", F.current_timestamp())
    )
    query = (
        stream.writeStream.format("delta")
        .option("checkpointLocation", args.checkpoint)
        .trigger(availableNow=True)
        .toTable(args.table)
    )
    query.awaitTermination()
    print(f"bronze: ingest into {args.table} complete")


if __name__ == "__main__":
    main()
