from pathlib import Path

from axle_analytics.config import Settings
from axle_analytics.uploader import files_api_url, find_ready_files, upload

SETTINGS = Settings(databricks_host="https://example.cloud.databricks.com", databricks_token="dummy",
                    catalog="main", schema="axle", volume="raw")


def make_tree(root: Path) -> None:
    part = root / "frames" / "vehicle_id=veh-001" / "date=2024-05-01"
    part.mkdir(parents=True)
    (part / "part-run1-00000.jsonl").write_text('{"seq":0}\n')
    (part / "part-run1-00001.jsonl").write_text('{"seq":1}\n')
    (part / "part-run1-00002.jsonl.inprogress").write_text('{"seq":2}\n')
    runs = root / "runs" / "vehicle_id=veh-001"
    runs.mkdir(parents=True)
    (runs / "run1.json").write_text('{"frames_fused":2}\n')
    (root / "veh-001.log").write_text("not uploaded")


def test_find_ready_files_skips_in_progress(tmp_path):
    make_tree(tmp_path)
    names = [p.name for p in find_ready_files(tmp_path)]
    assert names == ["part-run1-00000.jsonl", "part-run1-00001.jsonl", "run1.json"]


def test_upload_preserves_layout_and_is_idempotent(tmp_path):
    make_tree(tmp_path)
    calls = []

    def fake_put(url, body):
        calls.append((url, body))

    res = upload(tmp_path, SETTINGS, put=fake_put)
    assert len(res.uploaded) == 3 and not res.failed
    urls = [u for u, _ in calls]
    assert urls[0] == (
        "https://example.cloud.databricks.com/api/2.0/fs/files/Volumes/main/axle/raw/frames/"
        "vehicle_id%3Dveh-001/date%3D2024-05-01/part-run1-00000.jsonl?overwrite=true"
    )
    assert calls[0][1] == b'{"seq":0}\n'

    calls.clear()
    res2 = upload(tmp_path, SETTINGS, put=fake_put)
    assert res2.uploaded == [] and res2.skipped == 3
    assert calls == []


def test_failed_uploads_are_retried_next_time(tmp_path):
    make_tree(tmp_path)

    def flaky_put(url, body):
        if "00001" in url:
            raise RuntimeError("HTTP 503")

    res = upload(tmp_path, SETTINGS, put=flaky_put)
    assert len(res.failed) == 1
    res2 = upload(tmp_path, SETTINGS, put=lambda u, b: None)
    assert [k.split("/")[-1] for k in res2.uploaded] == ["part-run1-00001.jsonl"]


def test_dry_run_does_not_touch_manifest(tmp_path):
    make_tree(tmp_path)
    res = upload(tmp_path, SETTINGS, dry_run=True)
    assert len(res.uploaded) == 3
    assert not (tmp_path / ".uploaded.json").exists()


def test_files_api_url_quotes_path():
    url = files_api_url(SETTINGS, "/Volumes/main/axle/raw/a b.jsonl")
    assert url.endswith("/api/2.0/fs/files/Volumes/main/axle/raw/a%20b.jsonl?overwrite=true")
