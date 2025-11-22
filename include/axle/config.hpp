#pragma once

#include <cstddef>
#include <string>

#include "axle/clock.hpp"

namespace axle {

/// Pipeline stages that can be pinned to a CPU core.
enum class Stage { Lidar = 0, Imu, Can, Fusion, Export, Count };

struct PipelineConfig {
    // Queue capacities (rounded up to powers of two).
    std::size_t lidar_queue = 8;      // full sweeps are big; keep this shallow
    std::size_t imu_queue = 2048;
    std::size_t can_queue = 4096;
    std::size_t state_queue = 4096;   // processed IMU/vehicle states into fusion
    std::size_t result_queue = 8;     // processed LiDAR results into fusion
    std::size_t export_queue = 64;

    /// true: producers block when a queue is full (offline replay / benchmarks, no loss).
    /// false: producers drop and count (live operation, never stall a sensor thread).
    bool backpressure = false;

    /// Per-frame end-to-end budget (ingest -> fused) used for SLO accounting.
    Nanos latency_budget_ns = ms_to_ns(10.0);

    /// CPU to pin each stage to; -1 leaves scheduling to the OS. Indexed by Stage.
    int cpu[static_cast<int>(Stage::Count)] = {-1, -1, -1, -1, -1};

    /// SCHED_FIFO priority for processing stages (1..99); 0 keeps the default policy.
    /// Requires CAP_SYS_NICE or root; failure is reported and ignored.
    int rt_priority = 0;

    /// Spin iterations before an idle stage starts yielding/sleeping.
    int spin_before_yield = 2000;
};

struct ExportConfig {
    bool enabled = true;
    std::string out_dir = "telemetry_out";
    std::string vehicle_id = "sim-001";
    std::string run_id;                   // generated when empty
    std::size_t records_per_file = 600;   // ~1 minute of 10 Hz frames
    double max_file_age_s = 60.0;
    std::size_t max_obstacles = 8;        // obstacles serialised per frame
};

}  // namespace axle
