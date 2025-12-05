#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "axle/exporter.hpp"
#include "axle/log_format.hpp"
#include "axle/pipeline.hpp"
#include "axle/sim_sources.hpp"

using namespace axle;
namespace fs = std::filesystem;

namespace {

/// Captures fused frames in memory.
class CaptureSink final : public FrameSink {
public:
    struct Shared {
        std::mutex mu;
        std::vector<FusedFrame> frames;
        bool closed{false};
    };
    explicit CaptureSink(std::shared_ptr<Shared> s) : s_(std::move(s)) {}
    void write(const FusedFrame& f, Nanos, bool) override {
        std::lock_guard<std::mutex> lock(s_->mu);
        s_->frames.push_back(f);
    }
    void close() override { s_->closed = true; }

private:
    std::shared_ptr<Shared> s_;
};

SimConfig fast_sim(double seconds) {
    SimConfig cfg;
    cfg.realtime = false;
    cfg.duration_s = seconds;
    cfg.lidar_h_res_deg = 1.0;  // keep tests quick
    return cfg;
}

PipelineConfig lossless() {
    PipelineConfig p;
    p.backpressure = true;
    p.spin_before_yield = 100;
    return p;
}

}  // namespace

TEST(Pipeline, ProcessesEverySimulatedFrameLosslessly) {
    auto shared = std::make_shared<CaptureSink::Shared>();
    Pipeline p(lossless(), make_sim_sources(fast_sim(2.0)), std::make_unique<CaptureSink>(shared));
    p.start();
    p.wait();

    const auto s = p.snapshot();
    EXPECT_EQ(s.frames_fused, 20u);
    EXPECT_EQ(s.frames_exported, 20u);
    EXPECT_EQ(s.imu_in, 400u);
    EXPECT_EQ(s.can_in, 2u * (100 + 100 + 50 + 50 + 10));
    EXPECT_EQ(s.lidar_dropped + s.imu_dropped + s.can_dropped + s.state_dropped + s.export_dropped, 0u);
    EXPECT_TRUE(shared->closed);

    ASSERT_EQ(shared->frames.size(), 20u);
    for (std::size_t i = 0; i < shared->frames.size(); ++i) {
        const auto& f = shared->frames[i];
        EXPECT_EQ(f.seq, i);  // ordering preserved end to end
        EXPECT_GT(f.lidar.raw_points, 0u);
        EXPECT_TRUE(f.imu_valid) << "frame " << i;
        EXPECT_TRUE(f.vehicle_valid) << "frame " << i;
        EXPECT_GE(f.fused_ns, f.ingest_ns);
    }
    EXPECT_EQ(p.e2e_latency().count(), 20u);
}

TEST(Pipeline, FusedStateIsTimeAligned) {
    auto shared = std::make_shared<CaptureSink::Shared>();
    Pipeline p(lossless(), make_sim_sources(fast_sim(3.0)), std::make_unique<CaptureSink>(shared));
    p.start();
    p.wait();
    for (const auto& f : shared->frames) {
        const Nanos imu_skew = f.imu.sensor_ts_ns - f.sensor_ts_ns;
        EXPECT_LE(std::llabs(imu_skew), ms_to_ns(5.1)) << "frame " << f.seq;
    }
}

TEST(Pipeline, RejectsTwoProducersForSameKind) {
    std::vector<SourcePtr> sources = make_sim_sources(fast_sim(0.1));
    auto extra = make_sim_sources(fast_sim(0.1));
    sources.push_back(std::move(extra.front()));  // second LiDAR producer
    EXPECT_THROW({ Pipeline p(lossless(), std::move(sources), nullptr); }, std::invalid_argument);
}

TEST(Pipeline, ReplayMatchesRecording) {
    const auto path = (fs::temp_directory_path() / "axle_pipeline_replay.axl").string();
    {
        LogWriter writer(path);
        Pipeline rec(lossless(), make_sim_sources(fast_sim(1.0)), nullptr, &writer);
        rec.start();
        rec.wait();
        EXPECT_EQ(rec.snapshot().frames_fused, 10u);
    }

    auto shared = std::make_shared<CaptureSink::Shared>();
    std::vector<SourcePtr> sources;
    sources.push_back(std::make_unique<ReplaySource>(path, 0.0));
    Pipeline replay(lossless(), std::move(sources), std::make_unique<CaptureSink>(shared));
    replay.start();
    replay.wait();
    EXPECT_EQ(replay.snapshot().frames_fused, 10u);
    EXPECT_EQ(replay.snapshot().imu_in, 200u);
    fs::remove(path);
}

TEST(Pipeline, CountsInjectedCanFaults) {
    auto cfg = fast_sim(1.0);
    cfg.can_corrupt_prob = 0.2;
    Pipeline p(lossless(), make_sim_sources(cfg), nullptr);
    p.start();
    p.wait();
    const auto s = p.snapshot();
    EXPECT_GT(s.can_bad_checksum, 20u);
    EXPECT_LT(s.can_bad_checksum, 150u);
    EXPECT_GT(s.can_counter_jumps, 0u);  // rejected frames leave gaps in the counters
}

TEST(Pipeline, StopRequestEndsUnboundedRun) {
    SimConfig cfg;  // realtime, no duration
    cfg.lidar_h_res_deg = 2.0;
    Pipeline p(PipelineConfig{}, make_sim_sources(cfg), nullptr);
    p.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    p.request_stop();
    p.wait();
    EXPECT_TRUE(p.finished());
    EXPECT_GE(p.snapshot().frames_fused, 1u);
}

TEST(JsonlExporter, WritesPartitionedFilesAndRotates) {
    const auto dir = fs::temp_directory_path() / "axle_export_test";
    fs::remove_all(dir);
    ExportConfig cfg;
    cfg.out_dir = dir.string();
    cfg.vehicle_id = "veh-42";
    cfg.run_id = "testrun";
    cfg.records_per_file = 3;
    {
        JsonlExporter ex(cfg);
        for (int i = 0; i < 7; ++i) {
            FusedFrame f;
            f.seq = static_cast<std::uint64_t>(i);
            f.sensor_ts_ns = 1'700'000'000'000'000'000LL;  // 2023-11-14 UTC
            f.vehicle_valid = true;
            f.vehicle.speed_mps = 12.5f;
            ex.write(f, ms_to_ns(2.5), false);
        }
        ex.close();
        EXPECT_EQ(ex.files_written(), 3u);
        EXPECT_EQ(ex.records_written(), 7u);
    }
    const auto part_dir = dir / "frames" / "vehicle_id=veh-42" / "date=2023-11-14";
    ASSERT_TRUE(fs::exists(part_dir));
    int files = 0, lines = 0;
    for (const auto& e : fs::directory_iterator(part_dir)) {
        EXPECT_EQ(e.path().extension(), ".jsonl");
        ++files;
        std::ifstream in(e.path());
        std::string line;
        while (std::getline(in, line)) {
            ++lines;
            EXPECT_EQ(line.front(), '{');
            EXPECT_EQ(line.back(), '}');
            EXPECT_NE(line.find("\"vehicle_id\":\"veh-42\""), std::string::npos);
            EXPECT_NE(line.find("\"speed_mps\":12.500"), std::string::npos);
            EXPECT_NE(line.find("\"ttc_s\":null"), std::string::npos);
        }
    }
    EXPECT_EQ(files, 3);
    EXPECT_EQ(lines, 7);
    fs::remove_all(dir);
}

TEST(JsonlExporter, EscapesStringsAndOmitsInvalidBlocks) {
    ExportConfig cfg;
    cfg.vehicle_id = "a\"b";
    cfg.run_id = "r";
    FusedFrame f;
    const auto json = JsonlExporter::to_json(f, cfg, 0, true);
    EXPECT_NE(json.find("\"vehicle_id\":\"a\\\"b\""), std::string::npos);
    EXPECT_NE(json.find("\"over_budget\":true"), std::string::npos);
    EXPECT_EQ(json.find("roll_rad"), std::string::npos);
    EXPECT_EQ(json.find("speed_mps"), std::string::npos);
    EXPECT_NE(json.find("\"obstacles\":[]"), std::string::npos);
}
