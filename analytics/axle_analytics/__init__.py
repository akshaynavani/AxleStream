"""Offline fleet diagnostics for AxleStream telemetry on Databricks.

Modules:
    config       -- environment-driven settings (workspace host, token, volume paths)
    rules        -- diagnostic thresholds and pure-Python event classification
    schema       -- Spark schema for exported frame records
    diagnostics  -- PySpark transformations (bronze -> silver -> gold)
    uploader     -- pushes finished JSONL batches to a Unity Catalog volume
"""

__version__ = "0.1.0"
