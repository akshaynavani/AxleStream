// axlestream — command-line front end for the telemetry pipeline.
//
//   axlestream run     [options]        process live/simulated/replayed sensor data
//   axlestream record  --out FILE.axl   capture simulated sensors to a recording
//   axlestream inspect FILE.axl         summarise a recording
//
// Run `axlestream --help` for the full option list.

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <variant>
#include <vector>

#include "axle/exporter.hpp"
#include "axle/log_format.hpp"
#include "axle/pipeline.hpp"
#include "axle/sim_sources.hpp"
#include "axle/socketcan_source.hpp"

using namespace axle;

namespace {

std::atomic<bool> g_interrupted{false};

void on_signal(int) { g_interrupted.store(true); }

void usage() {
    std::puts(R"(AxleStream - real-time LiDAR / IMU / CAN telemetry pipeline

Usage:
  axlestream run [options]
  axlestream record --out FILE.axl [--duration S] [sim options]
  axlestream inspect FILE.axl

Sources (run):
  --source sim               simulated LiDAR + IMU + CAN (default)
  --source replay:FILE.axl   replay a recording
  --can-iface IFACE          take CAN from SocketCAN instead of the simulator
                             (requires a build with -DAXLE_WITH_SOCKETCAN=ON)
  --replay-speed X           replay rate multiplier, 0 = as fast as possible (default 1)
  --loop                     loop the replay until interrupted

Simulation:
  --duration S               stop after S seconds of sensor time (0 = until Ctrl-C)
  --fast                     generate data as fast as the pipeline accepts (implies backpressure)
  --seed N                   scenario seed (default 7)
  --lidar-res DEG            horizontal LiDAR resolution in degrees (default 0.4)
  --can-fault-rate P         probability of injecting a corrupted CAN checksum

Pipeline:
  --budget-ms MS             per-frame latency budget (default 10)
  --pin L,I,C,F,E            pin lidar/imu/can/fusion/export stages to CPUs (-1 = no pin)
  --rt-priority N            SCHED_FIFO priority for processing stages (needs CAP_SYS_NICE)
  --backpressure             block instead of dropping when queues are full
  --record FILE.axl          also record raw inputs while running

Export:
  --out DIR                  telemetry output directory (default ./telemetry_out)
  --vehicle-id ID            vehicle identifier written to every record (default sim-001)
  --batch N                  records per JSONL file (default 600)
  --no-export                discard fused frames (benchmarking)
  --quiet                    no periodic progress lines
)");
}

struct Args {
    std::string command;
    std::vector<std::string> positional;
    std::map<std::string, std::string> opts;

    bool has(const std::string& k) const { return opts.count(k) != 0; }
    std::string get(const std::string& k, const std::string& def = "") const {
        auto it = opts.find(k);
        return it == opts.end() ? def : it->second;
    }
    double num(const std::string& k, double def) const {
        auto it = opts.find(k);
        return it == opts.end() ? def : std::atof(it->second.c_str());
    }
};

Args parse(int argc, char** argv) {
    static const char* kFlags[] = {"--fast", "--loop", "--backpressure", "--no-export", "--quiet", "--help"};
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string tok = argv[i];
        if (tok.rfind("--", 0) == 0) {
            bool is_flag = false;
            for (const char* f : kFlags) is_flag |= tok == f;
            if (is_flag) {
                a.opts[tok] = "1";
            } else if (i + 1 < argc) {
                a.opts[tok] = argv[++i];
            } else {
                throw std::invalid_argument("missing value for " + tok);
            }
        } else if (tok == "-h") {
            a.opts["--help"] = "1";
        } else if (a.command.empty()) {
            a.command = tok;
        } else {
            a.positional.push_back(tok);
        }
    }
    return a;
}

SimConfig sim_config_from(const Args& a) {
    SimConfig cfg;
    cfg.duration_s = a.num("--duration", 0.0);
    cfg.realtime = !a.has("--fast");
    cfg.seed = static_cast<std::uint64_t>(a.num("--seed", 7));
    cfg.lidar_h_res_deg = a.num("--lidar-res", cfg.lidar_h_res_deg);
    cfg.can_corrupt_prob = a.num("--can-fault-rate", 0.0);
    return cfg;
}

void parse_pins(const std::string& spec, PipelineConfig& cfg) {
    std::stringstream ss(spec);
    std::string item;
    int idx = 0;
    while (std::getline(ss, item, ',') && idx < static_cast<int>(Stage::Count)) {
        cfg.cpu[idx++] = std::atoi(item.c_str());
    }
}

std::vector<SourcePtr> build_sources(const Args& a) {
    std::vector<SourcePtr> sources;
    const std::string spec = a.get("--source", "sim");

    if (spec.rfind("replay:", 0) == 0) {
        const std::string path = spec.substr(7);
        sources.push_back(std::make_unique<ReplaySource>(path, a.num("--replay-speed", 1.0), a.has("--loop")));
        if (a.has("--can-iface")) {
            throw std::invalid_argument("--can-iface cannot be combined with replay (replay already carries CAN)");
        }
        return sources;
    }
    if (spec != "sim") throw std::invalid_argument("unknown --source " + spec);

    sources = make_sim_sources(sim_config_from(a));
    if (a.has("--can-iface")) {
        // Replace the simulated CAN producer with the real bus.
        for (auto it = sources.begin(); it != sources.end(); ++it) {
            if ((*it)->kinds() == kCanMask) {
                sources.erase(it);
                break;
            }
        }
        sources.push_back(std::make_unique<SocketCanSource>(a.get("--can-iface")));
    }
    return sources;
}

void print_progress(const Pipeline& p, Nanos started) {
    const auto s = p.snapshot();
    const auto& e2e = p.e2e_latency();
    std::fprintf(stderr,
                 "[%6.1fs] frames=%llu  e2e p50=%.2fms p99=%.2fms max=%.2fms  over_budget=%llu  drops=%llu\n",
                 static_cast<double>(now_ns() - started) / 1e9, static_cast<unsigned long long>(s.frames_fused),
                 ns_to_ms(e2e.percentile(50)), ns_to_ms(e2e.percentile(99)), ns_to_ms(e2e.max()),
                 static_cast<unsigned long long>(s.over_budget),
                 static_cast<unsigned long long>(s.lidar_dropped + s.imu_dropped + s.can_dropped + s.state_dropped +
                                                 s.result_dropped + s.export_dropped));
}

int cmd_run(const Args& a) {
    PipelineConfig pcfg;
    pcfg.latency_budget_ns = ms_to_ns(a.num("--budget-ms", 10.0));
    pcfg.backpressure = a.has("--backpressure") || a.has("--fast") ||
                        (a.get("--source").rfind("replay:", 0) == 0 && a.num("--replay-speed", 1.0) == 0.0);
    pcfg.rt_priority = static_cast<int>(a.num("--rt-priority", 0));
    if (a.has("--pin")) parse_pins(a.get("--pin"), pcfg);

    ExportConfig ecfg;
    ecfg.enabled = !a.has("--no-export");
    ecfg.out_dir = a.get("--out", "telemetry_out");
    ecfg.vehicle_id = a.get("--vehicle-id", "sim-001");
    ecfg.records_per_file = static_cast<std::size_t>(a.num("--batch", 600));
    ecfg.run_id = make_run_id();

    std::unique_ptr<FrameSink> sink;
    if (ecfg.enabled) {
        sink = std::make_unique<JsonlExporter>(ecfg);
    } else {
        sink = std::make_unique<NullSink>();
    }

    std::unique_ptr<LogWriter> recorder;
    if (a.has("--record")) recorder = std::make_unique<LogWriter>(a.get("--record"));

    Pipeline pipeline(pcfg, build_sources(a), std::move(sink), recorder.get());
    std::fprintf(stderr, "[axle] run %s vehicle=%s sources:", ecfg.run_id.c_str(), ecfg.vehicle_id.c_str());
    for (const auto& s : pipeline.sources()) std::fprintf(stderr, " %s", s->name().c_str());
    std::fprintf(stderr, "%s\n", pcfg.backpressure ? " (backpressure)" : "");

    const Nanos started = now_ns();
    pipeline.start();

    Nanos last_print = started;
    while (!pipeline.finished()) {
        if (g_interrupted.load()) {
            pipeline.request_stop();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!a.has("--quiet") && now_ns() - last_print > ms_to_ns(1000)) {
            print_progress(pipeline, started);
            last_print = now_ns();
        }
    }
    pipeline.wait();

    std::fputs(pipeline.report().c_str(), stdout);
    if (ecfg.enabled) {
        const auto path = (std::filesystem::path(ecfg.out_dir) / "runs" / ("vehicle_id=" + ecfg.vehicle_id) /
                           (ecfg.run_id + ".json"))
                              .string();
        write_run_summary(path, pipeline.summary_json(ecfg.vehicle_id, ecfg.run_id));
        std::printf("telemetry written to %s (run summary %s)\n", ecfg.out_dir.c_str(), path.c_str());
    }
    if (recorder) {
        std::printf("recorded %llu records (%.1f MiB) to %s\n", static_cast<unsigned long long>(recorder->records()),
                    static_cast<double>(recorder->bytes()) / (1024.0 * 1024.0), a.get("--record").c_str());
    }

    const auto snap = pipeline.snapshot();
    return snap.frames_fused > 0 ? 0 : 2;
}

int cmd_record(const Args& a) {
    if (!a.has("--out")) throw std::invalid_argument("record needs --out FILE.axl");
    LogWriter writer(a.get("--out"));
    auto sources = make_sim_sources(sim_config_from(a));
    SensorSinks sinks = tee_to_log(SensorSinks{}, writer);

    std::atomic<bool> stop{false};
    std::vector<std::thread> threads;
    for (auto& s : sources) {
        Source* src = s.get();
        threads.emplace_back([src, &stop, &sinks] { src->run(stop, sinks); });
    }
    if (a.num("--duration", 0.0) <= 0) {
        std::fprintf(stderr, "[axle] recording until Ctrl-C\n");
    }
    // Wait for natural completion (duration) or Ctrl-C.
    std::thread watcher([&] {
        while (!stop.load()) {
            if (g_interrupted.load()) stop.store(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
    for (auto& t : threads) t.join();
    stop.store(true);
    watcher.join();
    writer.flush();
    std::printf("recorded %llu records (%.1f MiB) to %s\n", static_cast<unsigned long long>(writer.records()),
                static_cast<double>(writer.bytes()) / (1024.0 * 1024.0), a.get("--out").c_str());
    return 0;
}

int cmd_inspect(const Args& a) {
    if (a.positional.empty()) throw std::invalid_argument("inspect needs a FILE.axl argument");
    LogReader reader(a.positional.front());
    std::uint64_t lidar = 0, imu = 0, can = 0, points = 0;
    Nanos first = -1, last = -1;
    std::map<std::uint32_t, std::uint64_t> can_ids;
    while (auto rec = reader.next()) {
        std::visit(
            [&](const auto& m) {
                using T = std::decay_t<decltype(m)>;
                if (first < 0 || m.sensor_ts_ns < first) first = m.sensor_ts_ns;
                if (m.sensor_ts_ns > last) last = m.sensor_ts_ns;
                if constexpr (std::is_same_v<T, LidarFrame>) {
                    ++lidar;
                    points += m.points.size();
                } else if constexpr (std::is_same_v<T, ImuSample>) {
                    ++imu;
                } else {
                    ++can;
                    ++can_ids[m.id];
                }
            },
            *rec);
    }
    const double span = first >= 0 ? static_cast<double>(last - first) / 1e9 : 0.0;
    std::printf("file: %s\n", a.positional.front().c_str());
    std::printf("span: %.2f s%s\n", span, reader.truncated() ? "  (WARNING: truncated/corrupt tail)" : "");
    std::printf("lidar frames: %llu (avg %.0f points, %.1f Hz)\n", static_cast<unsigned long long>(lidar),
                lidar ? static_cast<double>(points) / static_cast<double>(lidar) : 0.0,
                span > 0 ? static_cast<double>(lidar) / span : 0.0);
    std::printf("imu samples:  %llu (%.1f Hz)\n", static_cast<unsigned long long>(imu),
                span > 0 ? static_cast<double>(imu) / span : 0.0);
    std::printf("can frames:   %llu\n", static_cast<unsigned long long>(can));
    for (const auto& [id, n] : can_ids) {
        std::printf("  0x%03X: %llu (%.1f Hz)\n", id, static_cast<unsigned long long>(n),
                    span > 0 ? static_cast<double>(n) / span : 0.0);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    try {
        const Args args = parse(argc, argv);
        if (args.has("--help") || args.command.empty()) {
            usage();
            return args.command.empty() && !args.has("--help") ? 1 : 0;
        }
        if (args.command == "run") return cmd_run(args);
        if (args.command == "record") return cmd_record(args);
        if (args.command == "inspect") return cmd_inspect(args);
        std::fprintf(stderr, "unknown command '%s'\n", args.command.c_str());
        usage();
        return 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
