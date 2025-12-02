// axle_bench — micro-benchmarks for the latency-critical pieces.
//
//   1. LiDAR processing time per sweep on pre-generated simulated frames
//   2. SPSC queue one-way hand-off latency between two threads (ping-pong / 2)
//   3. Full pipeline in fast mode (backpressure, no export) for end-to-end latency
//
// Usage: axle_bench [--frames N] [--lidar-res DEG] [--budget-ms MS] [--pin-a CPU --pin-b CPU]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "axle/latency.hpp"
#include "axle/lidar_processor.hpp"
#include "axle/pipeline.hpp"
#include "axle/sim_sources.hpp"
#include "axle/spsc_queue.hpp"
#include "axle/thread_util.hpp"

using namespace axle;

namespace {

struct BenchArgs {
    int frames = 200;
    double lidar_res = 0.4;
    double budget_ms = 10.0;
    int pin_a = -1;
    int pin_b = -1;
};

BenchArgs parse(int argc, char** argv) {
    BenchArgs b;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string k = argv[i];
        const char* v = argv[i + 1];
        if (k == "--frames") b.frames = std::atoi(v);
        else if (k == "--lidar-res") b.lidar_res = std::atof(v);
        else if (k == "--budget-ms") b.budget_ms = std::atof(v);
        else if (k == "--pin-a") b.pin_a = std::atoi(v);
        else if (k == "--pin-b") b.pin_b = std::atoi(v);
    }
    return b;
}

bool bench_lidar(const BenchArgs& args) {
    SimConfig cfg;
    cfg.lidar_h_res_deg = args.lidar_res;
    auto model = std::make_shared<const VehicleModel>();
    SimLidarSource sim(model, cfg);

    std::printf("generating %d sweeps (%.2f deg resolution)...\n", args.frames, args.lidar_res);
    std::vector<LidarFrame> frames;
    frames.reserve(static_cast<std::size_t>(args.frames));
    for (int i = 0; i < args.frames; ++i) frames.push_back(sim.generate(i * 0.1, static_cast<std::uint64_t>(i)));

    LidarProcessor proc;
    for (int i = 0; i < 5 && i < args.frames; ++i) proc.process(frames[static_cast<std::size_t>(i)]);  // warm-up

    LatencyHistogram h;
    std::size_t points = 0;
    for (auto& f : frames) {
        f.ingest_ns = now_ns();
        const auto r = proc.process(f);
        h.record(r.processed_ns - f.ingest_ns);
        points += f.points.size();
    }
    std::printf("[lidar] avg %.0f pts/sweep\n", static_cast<double>(points) / static_cast<double>(frames.size()));
    std::printf("%s\n", h.summary("lidar-process").c_str());
    const bool ok = h.percentile(99) <= ms_to_ns(args.budget_ms);
    std::printf("[lidar] p99 %s %.1f ms budget\n", ok ? "within" : "EXCEEDS", args.budget_ms);
    return ok;
}

void bench_queue(const BenchArgs& args) {
    constexpr int kRounds = 200000;
    SpscQueue<Nanos> ping(1024), pong(1024);
    std::thread echo([&] {
        if (args.pin_b >= 0) pin_current_thread(args.pin_b);
        for (int i = 0; i < kRounds; ++i) {
            std::optional<Nanos> v;
            while (!(v = ping.try_pop())) cpu_relax();
            while (!pong.try_push(*v)) cpu_relax();
        }
    });
    if (args.pin_a >= 0) pin_current_thread(args.pin_a);
    LatencyHistogram h;
    for (int i = 0; i < kRounds; ++i) {
        const Nanos t0 = now_ns();
        while (!ping.try_push(t0)) cpu_relax();
        std::optional<Nanos> v;
        while (!(v = pong.try_pop())) cpu_relax();
        h.record((now_ns() - t0) / 2);
    }
    echo.join();
    std::printf("%s\n", h.summary("spsc-one-way").c_str());
}

void bench_pipeline(const BenchArgs& args) {
    SimConfig scfg;
    scfg.realtime = false;
    scfg.duration_s = args.frames / 10.0;
    scfg.lidar_h_res_deg = args.lidar_res;
    PipelineConfig pcfg;
    pcfg.backpressure = true;
    pcfg.latency_budget_ns = ms_to_ns(args.budget_ms);

    Pipeline p(pcfg, make_sim_sources(scfg), std::make_unique<NullSink>());
    const Nanos t0 = now_ns();
    p.start();
    p.wait();
    const double secs = static_cast<double>(now_ns() - t0) / 1e9;
    const auto s = p.snapshot();
    std::printf("[pipeline] %llu frames in %.2fs (%.1f frames/s), over budget: %llu\n",
                static_cast<unsigned long long>(s.frames_fused), secs, static_cast<double>(s.frames_fused) / secs,
                static_cast<unsigned long long>(s.over_budget));
    std::printf("%s\n", p.lidar_latency().summary("lidar-stage").c_str());
    std::printf("%s\n", p.e2e_latency().summary("end-to-end").c_str());
    std::printf("note: fast mode queues frames behind each other, so end-to-end includes queueing;\n"
                "      use `axlestream run` (realtime) for steady-state latency.\n");
}

}  // namespace

int main(int argc, char** argv) {
    const auto args = parse(argc, argv);
    const bool ok = bench_lidar(args);
    bench_queue(args);
    bench_pipeline(args);
    return ok ? 0 : 1;
}
