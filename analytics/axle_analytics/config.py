"""Settings loaded from environment variables (optionally via a local .env file).

Nothing secret is ever hard-coded: the Databricks token only comes from the environment.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

try:  # python-dotenv is optional; plain environment variables work without it
    from dotenv import load_dotenv
except ImportError:  # pragma: no cover
    load_dotenv = None


def _load_env_file() -> None:
    if load_dotenv is None:
        return
    # Look for .env in the working directory, then at the repository root.
    for candidate in (Path.cwd() / ".env", Path(__file__).resolve().parents[2] / ".env"):
        if candidate.is_file():
            load_dotenv(candidate, override=False)
            return


@dataclass(frozen=True)
class Settings:
    databricks_host: str = ""
    databricks_token: str = field(default="", repr=False)
    catalog: str = "main"
    schema: str = "axlestream"
    volume: str = "raw_telemetry"
    checkpoint_root: str = ""
    latency_budget_ms: float = 10.0

    @property
    def volume_path(self) -> str:
        """Unity Catalog volume root that the uploader writes to and Auto Loader reads from."""
        return f"/Volumes/{self.catalog}/{self.schema}/{self.volume}"

    @property
    def frames_path(self) -> str:
        return f"{self.volume_path}/frames"

    @property
    def runs_path(self) -> str:
        return f"{self.volume_path}/runs"

    def table(self, name: str) -> str:
        return f"{self.catalog}.{self.schema}.{name}"

    def checkpoint(self, name: str) -> str:
        root = self.checkpoint_root or f"{self.volume_path}/_checkpoints"
        return f"{root}/{name}"

    def require_workspace(self) -> None:
        missing = [n for n, v in (("DATABRICKS_HOST", self.databricks_host),
                                  ("DATABRICKS_TOKEN", self.databricks_token)) if not v]
        if missing:
            raise RuntimeError(
                "missing Databricks credentials: set " + ", ".join(missing) +
                " in the environment or in .env (see .env.example)")


def load_settings() -> Settings:
    _load_env_file()
    host = os.environ.get("DATABRICKS_HOST", "").rstrip("/")
    if host and not host.startswith("http"):
        host = "https://" + host
    return Settings(
        databricks_host=host,
        databricks_token=os.environ.get("DATABRICKS_TOKEN", ""),
        catalog=os.environ.get("AXLE_CATALOG", "main"),
        schema=os.environ.get("AXLE_SCHEMA", "axlestream"),
        volume=os.environ.get("AXLE_VOLUME", "raw_telemetry"),
        checkpoint_root=os.environ.get("AXLE_CHECKPOINT_ROOT", ""),
        latency_budget_ms=float(os.environ.get("AXLE_LATENCY_BUDGET_MS", "10")),
    )
