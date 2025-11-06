# AxleStream — Implementation Plan

Real-time vehicle telemetry pipeline in C++17 targeting ARM (aarch64) Linux, processing
LiDAR point clouds, IMU samples and CAN bus frames through multithreaded lock-free queues
with a sub-10 ms per-frame latency budget. Processed telemetry is exported as batched
files that Databricks (PySpark + Delta Lake) ingests for offline diagnostics and
fleet-level analysis.

## Key design decisions

- **Language/build:** C++17, CMake >= 3.16, GoogleTest via `FetchContent`. Builds natively on
  x86_64 or aarch64; a CMake toolchain file is provided for cross-compiling to ARM64.
- **No hardware assumed.** Every sensor has a *simulated* source (deterministic, seeded) and a
  *replay* source reading a recorded binary log (`.axl`). Real SocketCAN input is optional
  (`-DAXLE_WITH_SOCKETCAN=ON`, Linux only).
- **Concurrency:** one thread per source, one thread per processing stage, one exporter thread.
  Stages are connected by bounded single-producer/single-consumer lock-free ring buffers
  (cache-line padded atomics, acquire/release ordering). The fusion stage merges the three
  processed streams. Optional CPU pinning via `pthread_setaffinity_np`.
- **Latency:** each message carries a monotonic `ingest_ns` stamp; the exporter records
  end-to-end latency into a log-bucketed histogram (p50/p95/p99/max) and flags frames that
  exceed the 10 ms budget.
- **Databricks:** C++ side writes rotating JSON Lines batches (one record per fused frame plus
  a CAN signal stream). Python tooling uploads them to a DBFS/Volume path using env-var
  credentials and PySpark jobs build bronze -> silver -> gold Delta tables for diagnostics.

## Phases

### Phase 1 — Core primitives
Deliverables: CMake project skeleton, message types, lock-free SPSC ring buffer, monotonic
clock helpers, latency histogram, unit tests.
Files: `CMakeLists.txt`, `cmake/aarch64-toolchain.cmake`, `include/axle/types.hpp`,
`include/axle/spsc_queue.hpp`, `include/axle/clock.hpp`, `include/axle/latency.hpp`,
`src/core/latency.cpp`, `tests/CMakeLists.txt`, `tests/test_spsc_queue.cpp`,
`tests/test_latency.cpp`.

### Phase 2 — Sensor sources (simulated, replay, SocketCAN)
Deliverables: source interface, simulated rotating LiDAR, IMU and CAN generators driven by a
shared vehicle motion model, `.axl` binary log writer/reader, replay source honouring original
timing (or as-fast-as-possible), optional SocketCAN source.
Files: `include/axle/source.hpp`, `include/axle/vehicle_model.hpp`, `src/sources/*.cpp`,
`include/axle/log_format.hpp`, `src/io/log_writer.cpp`, `src/io/log_reader.cpp`,
`tests/test_log_format.cpp`, `tests/test_sim_sources.cpp`.

### Phase 3 — Processing stages
Deliverables: LiDAR processor (range crop, voxel-grid downsample, RANSAC-free height-based
ground removal, grid clustering into obstacles), IMU complementary filter (roll/pitch) plus
yaw integration, CAN decoder with a small DBC-like signal table (wheel speed, steering angle,
RPM, brake pressure), fusion stage that joins latest IMU/CAN state to each LiDAR frame.
Files: `include/axle/lidar_processor.hpp`, `src/processing/lidar_processor.cpp`,
`include/axle/imu_filter.hpp`, `src/processing/imu_filter.cpp`, `include/axle/can_decoder.hpp`,
`src/processing/can_decoder.cpp`, `include/axle/fusion.hpp`, `src/processing/fusion.cpp`,
tests for each.

### Phase 4 — Pipeline runtime, exporter and CLI
Deliverables: pipeline orchestrator wiring threads + queues, CPU affinity, graceful shutdown on
SIGINT, JSON Lines batch exporter with file rotation, latency report, `axlestream` CLI
(`run` with sim/replay/socketcan sources, `record` to `.axl`), `axle_bench` latency benchmark.
Files: `include/axle/pipeline.hpp`, `src/pipeline/pipeline.cpp`, `include/axle/exporter.hpp`,
`src/io/jsonl_exporter.cpp`, `include/axle/config.hpp`, `src/app/main.cpp`,
`src/app/bench.cpp`, `tests/test_pipeline.cpp`.

### Phase 5 — Databricks fleet analytics
Deliverables: Python uploader (Databricks Files API, env-var credentials), PySpark jobs for
bronze ingest, silver cleaning, gold fleet diagnostics (latency SLO breaches, harsh braking,
IMU anomalies, per-vehicle summaries), a fleet simulation script that produces multi-vehicle
data, Databricks job definition, `.env.example`, pytest tests for pure-Python helpers.
Files: `analytics/axle_analytics/*.py`, `analytics/jobs/*.py`, `analytics/databricks_job.json`,
`analytics/requirements.txt`, `analytics/tests/*.py`, `scripts/simulate_fleet.sh`,
`.env.example`.

Finally: `.gitignore` and `README.md` with architecture diagram, run and test instructions.
