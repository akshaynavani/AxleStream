#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "axle/can_decoder.hpp"
#include "axle/config.hpp"
#include "axle/exporter.hpp"
#include "axle/fusion.hpp"
#include "axle/latency.hpp"
#include "axle/log_format.hpp"
#include "axle/source.hpp"
#include "axle/spsc_queue.hpp"

namespace axle {

/// Point-in-time copy of pipeline counters (plain values, safe to print or serialise).
struct PipelineSnapshot {
    std::uint64_t lidar_in{0}, imu_in{0}, can_in{0};
    std::uint64_t lidar_dropped{0}, imu_dropped{0}, can_dropped{0};
    std::uint64_t state_dropped{0}, result_dropped{0}, export_dropped{0};
    std::uint64_t frames_fused{0}, frames_exported{0}, over_budget{0};
    std::uint64_t can_bad_checksum{0}, can_counter_jumps{0}, can_unknown{0};
    std::uint64_t imu_unmatched{0}, vehicle_unmatched{0};
};

/// Multithreaded processing graph:
///
///   sources ──► [lidar q] ──► LiDAR stage ──► [result q] ──┐
///           ──► [imu q]   ──► IMU stage   ──► [imu st q] ──┼──► Fusion ──► [export q] ──► Exporter
///           ──► [can q]   ──► CAN stage   ──► [veh st q] ──┘
///
/// Every queue is a lock-free SPSC ring with exactly one producer and one consumer thread.
/// Shutdown cascades: once all sources finish (or stop is requested), each stage drains its
/// input and then marks itself done, which lets the next stage finish in turn.
class Pipeline {
public:
    Pipeline(PipelineConfig cfg, std::vector<SourcePtr> sources, std::unique_ptr<FrameSink> sink,
             LogWriter* recorder = nullptr);
    ~Pipeline();

    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    void start();
    /// Ask sources to stop; stages then drain and exit. Non-blocking.
    void request_stop() noexcept { stop_.store(true, std::memory_order_relaxed); }
    /// Block until every thread has exited.
    void wait();
    bool finished() const noexcept { return export_done_.load(std::memory_order_acquire); }

    PipelineSnapshot snapshot() const;
    const LatencyHistogram& e2e_latency() const noexcept { return lat_e2e_; }
    const LatencyHistogram& lidar_latency() const noexcept { return lat_lidar_; }
    const LatencyHistogram& imu_latency() const noexcept { return lat_imu_; }
    const LatencyHistogram& can_latency() const noexcept { return lat_can_; }
    const LatencyHistogram& export_latency() const noexcept { return lat_export_; }
    const std::vector<SourcePtr>& sources() const noexcept { return sources_; }
    const PipelineConfig& config() const noexcept { return cfg_; }

    /// Multi-line human readable report.
    std::string report() const;
    /// Machine-readable run summary (JSON object).
    std::string summary_json(const std::string& vehicle_id, const std::string& run_id) const;

private:
    struct ExportItem {
        FusedFrame frame;
        Nanos e2e_ns{0};
        bool over_budget{false};
    };

    struct Counters {
        std::atomic<std::uint64_t> lidar_in{0}, imu_in{0}, can_in{0};
        std::atomic<std::uint64_t> lidar_dropped{0}, imu_dropped{0}, can_dropped{0};
        std::atomic<std::uint64_t> state_dropped{0}, result_dropped{0}, export_dropped{0};
        std::atomic<std::uint64_t> frames_fused{0}, frames_exported{0}, over_budget{0};
        std::atomic<std::uint64_t> can_bad_checksum{0}, can_counter_jumps{0}, can_unknown{0};
        std::atomic<std::uint64_t> imu_unmatched{0}, vehicle_unmatched{0};
    };

    template <typename T>
    bool push(SpscQueue<T>& q, T&& v, std::atomic<std::uint64_t>& drops);

    SensorSinks make_sinks();
    void stage_setup(Stage stage, const char* name, bool realtime);

    void run_sources();
    void lidar_stage();
    void imu_stage();
    void can_stage();
    void fusion_stage();
    void export_stage();

    PipelineConfig cfg_;
    std::vector<SourcePtr> sources_;
    std::unique_ptr<FrameSink> sink_;
    LogWriter* recorder_;

    SpscQueue<LidarFrame> lidar_q_;
    SpscQueue<ImuSample> imu_q_;
    SpscQueue<CanFrame> can_q_;
    SpscQueue<LidarResult> result_q_;
    SpscQueue<ImuState> imu_state_q_;
    SpscQueue<VehicleState> veh_state_q_;
    SpscQueue<ExportItem> export_q_;

    std::atomic<bool> stop_{false};
    std::atomic<bool> abort_{false};
    std::atomic<int> sources_running_{0};
    std::atomic<bool> sources_done_{false};
    std::atomic<bool> lidar_done_{false}, imu_done_{false}, can_done_{false};
    std::atomic<bool> fusion_done_{false}, export_done_{false};

    Counters c_;
    LatencyHistogram lat_lidar_, lat_imu_, lat_can_, lat_e2e_, lat_export_;

    std::vector<std::thread> threads_;
    bool started_{false};
};

}  // namespace axle
