"""Upload finished AxleStream telemetry batches to a Unity Catalog volume.

Walks the exporter output directory, picks up completed `*.jsonl` frame files and run
summaries (`*.inprogress` files are still being written and are skipped), and PUTs them to
the Databricks Files API preserving the partition layout:

    telemetry_out/frames/vehicle_id=X/date=D/part-*.jsonl
      -> /Volumes/<catalog>/<schema>/<volume>/frames/vehicle_id=X/date=D/part-*.jsonl

A manifest (`.uploaded.json` in the source dir) records what has been sent, so the
uploader is safe to run repeatedly (e.g. from cron on the vehicle gateway).

    python -m axle_analytics.uploader --src ../telemetry_out [--dry-run] [--watch 30]
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.parse
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

from .config import Settings, load_settings

MANIFEST = ".uploaded.json"
UPLOAD_SUFFIXES = (".jsonl", ".json")


@dataclass
class UploadResult:
    uploaded: list[str]
    skipped: int
    failed: list[tuple[str, str]]


def find_ready_files(src: Path) -> list[Path]:
    """Completed telemetry files under `src`, in deterministic order."""
    out = []
    for sub in ("frames", "runs"):
        root = src / sub
        if not root.is_dir():
            continue
        for p in sorted(root.rglob("*")):
            if p.is_file() and p.suffix in UPLOAD_SUFFIXES and not p.name.endswith(".inprogress"):
                out.append(p)
    return out


def load_manifest(src: Path) -> dict:
    path = src / MANIFEST
    if path.is_file():
        try:
            return json.loads(path.read_text())
        except json.JSONDecodeError:
            pass
    return {}


def save_manifest(src: Path, manifest: dict) -> None:
    tmp = src / (MANIFEST + ".tmp")
    tmp.write_text(json.dumps(manifest, indent=1, sort_keys=True))
    tmp.replace(src / MANIFEST)


def remote_path(settings: Settings, src: Path, file: Path) -> str:
    rel = file.relative_to(src).as_posix()
    return f"{settings.volume_path}/{rel}"


def files_api_url(settings: Settings, path: str) -> str:
    return f"{settings.databricks_host}/api/2.0/fs/files{urllib.parse.quote(path)}?overwrite=true"


def make_http_put(settings: Settings, timeout_s: float = 60.0) -> Callable[[str, bytes], None]:
    import requests  # imported lazily so dry runs/tests work without it

    session = requests.Session()
    session.headers.update({
        "Authorization": f"Bearer {settings.databricks_token}",
        "Content-Type": "application/octet-stream",
    })

    def put(url: str, body: bytes) -> None:
        for attempt in range(4):
            resp = session.put(url, data=body, timeout=timeout_s)
            if resp.status_code < 300:
                return
            if resp.status_code in (429, 500, 502, 503, 504) and attempt < 3:
                time.sleep(2 ** attempt)
                continue
            raise RuntimeError(f"HTTP {resp.status_code}: {resp.text[:200]}")

    return put


def upload(src: Path, settings: Settings, put: Callable[[str, bytes], None] | None = None,
           dry_run: bool = False, files: Iterable[Path] | None = None) -> UploadResult:
    manifest = load_manifest(src)
    candidates = list(files) if files is not None else find_ready_files(src)
    if put is None and not dry_run:
        settings.require_workspace()
        put = make_http_put(settings)

    uploaded, failed, skipped = [], [], 0
    for f in candidates:
        key = f.relative_to(src).as_posix()
        stat = f.stat()
        fingerprint = f"{stat.st_size}:{int(stat.st_mtime)}"
        if manifest.get(key) == fingerprint:
            skipped += 1
            continue
        dest = remote_path(settings, src, f)
        if dry_run:
            print(f"would upload {key} -> {dest}")
            uploaded.append(key)
            continue
        try:
            put(files_api_url(settings, dest), f.read_bytes())
        except Exception as exc:  # keep going; failures are retried on the next run
            failed.append((key, str(exc)))
            continue
        manifest[key] = fingerprint
        uploaded.append(key)

    if not dry_run:
        save_manifest(src, manifest)
    return UploadResult(uploaded=uploaded, skipped=skipped, failed=failed)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default="telemetry_out", help="exporter output directory")
    ap.add_argument("--dry-run", action="store_true", help="list what would be uploaded")
    ap.add_argument("--watch", type=float, default=0.0, help="re-scan every N seconds (0 = once)")
    args = ap.parse_args(argv)

    settings = load_settings()
    src = Path(args.src).resolve()
    if not src.is_dir():
        print(f"source directory not found: {src}", file=sys.stderr)
        return 1

    while True:
        res = upload(src, settings, dry_run=args.dry_run)
        print(f"uploaded={len(res.uploaded)} skipped={res.skipped} failed={len(res.failed)}")
        for key, err in res.failed:
            print(f"  FAILED {key}: {err}", file=sys.stderr)
        if args.watch <= 0:
            return 1 if res.failed else 0
        time.sleep(args.watch)


if __name__ == "__main__":
    sys.exit(main())
