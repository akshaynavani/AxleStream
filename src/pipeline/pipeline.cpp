#include "axle/pipeline.hpp"

#include <cstdio>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "axle/imu_filter.hpp"
#include "axle/lidar_processor.hpp"
#include "axle/thread_util.hpp"

namespace axle {

namespace {
// States up to this far beyond a LiDAR timestamp are admitted before fusing it, so the
// "closest" match can be slightly newer than the sweep.
constexpr Nanos kFusionLookahead = ms_to_ns(5.0);
// In lossless (backpressure) mode the fusion stage waits at most this long for the IMU/CAN
// streams to catch up with a LiDAR sweep before fusing with whatever it has.
constexpr Nanos kMaxAlignWait = ms_to_ns(200.0);

inline std::uint64_t load(const std::atomic<std::uint64_t>& a) { return a.load(std::memory_order_relaxed); }
inline void bump(std::atomic<std::uint64_t>& a, std::uint64_t n = 1) { a.fetch_add(n, std::memory_order_relaxed); }
}  // namespace

Pipeline::Pipeline(PipelineConfig cfg, std::vector<SourcePtr> sources, std::unique_ptr<FrameSink> sink,
                   LogWriter* recorder)
    : cfg_(cfg),
      sources_(std::move(sources)),
      sink_(sink ? std::move(sink) : std::make_unique<NullSink>()),
      recorder_(recorder),
      lidar_q_(cfg.lidar_queue),
      imu_q_(cfg.imu_queue),
      can_q_(cfg.can_queue),
      result_q_(cfg.result_queue),
      imu_state_q_(cfg.state_queue),
      veh_state_q_(cfg.state_queue),
      export_q_(cfg.export_queue) {
    // Each per-kind queue is single-producer: refuse configurations where two source
    // threads would feed the same queue.
    std::uint8_t claimed = 0;
    for (const auto& s : sources_) {
        if (claimed & s->kinds()) {
            throw std::invalid_argument("two sources produce the same sensor kind (" + s->name() +
                                        "); each kind needs exactly one producer");
        }
        claimed = static_cast<std::uint8_t>(claimed | s->kinds());
    }
    has_lidar_ = (claimed & kLidarMask) != 0;
    has_imu_ = (claimed & kImuMask) != 0;
    has_can_ = (claimed & kCanMask) != 0;
}

Pipeline::~Pipeline() {
    if (started_) {
        request_stop();
        abort_.store(true, std::memory_order_relaxed);
        wait();
    }
}

template <typename T>
bool Pipeline::push(SpscQueue<T>& q, T&& v, std::atomic<std::uint64_t>& drops) {
    if (cfg_.backpressure) {
        while (!q.try_push(std::move(v))) {
            if (abort_.load(std::memory_order_relaxed)) {
                bump(drops);
                return false;
            }
            cpu_relax();
        }
        return true;
    }
    if (!q.try_push(std::move(v))) {  // try_push leaves `v` intact on failure
        bump(drops);
        return false;
    }
    return true;
}

SensorSinks Pipeline::make_sinks() {
    SensorSinks s;
    s.lidar = [this](LidarFrame&& f) { return push(lidar_q_, std::move(f), c_.lidar_dropped); };
    s.imu = [this](ImuSample&& m) { return push(imu_q_, std::move(m), c_.imu_dropped); };
    s.can = [this](CanFrame&& f) { return push(can_q_, std::move(f), c_.can_dropped); };
    return recorder_ ? tee_to_log(s, *recorder_) : s;
}

void Pipeline::stage_setup(Stage stage, const char* name, bool realtime) {
    set_current_thread_name(name);
    const int cpu = cfg_.cpu[static_cast<int>(stage)];
    if (cpu >= 0 && !pin_current_thread(cpu)) {
        std::fprintf(stderr, "[axle] could not pin %s to cpu %d\n", name, cpu);
    }
    if (realtime && cfg_.rt_priority > 0) set_realtime_priority(cfg_.rt_priority);
}

void Pipeline::start() {
    if (started_) return;
    started_ = true;

    threads_.emplace_back([this] { lidar_stage(); });
    threads_.emplace_back([this] { imu_stage(); });
    threads_.emplace_back([this] { can_stage(); });
    threads_.emplace_back([this] { fusion_stage(); });
    threads_.emplace_back([this] { export_stage(); });
    run_sources();
}

void Pipeline::run_sources() {
    const SensorSinks sinks = make_sinks();
    sources_running_.store(static_cast<int>(sources_.size()), std::memory_order_relaxed);
    if (sources_.empty()) sources_done_.store(true, std::memory_order_release);

    for (auto& src : sources_) {
        Source* s = src.get();
        threads_.emplace_back([this, s, sinks] {
            set_current_thread_name("src-" + s->name());
            try {
                s->run(stop_, sinks);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "[axle] source %s failed: %s\n", s->name().c_str(), e.what());
            }
            if (sources_running_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                sources_done_.store(true, std::memory_order_release);
            }
        });
    }
}

void Pipeline::wait() {
    for (auto& t : threads_) {
        if (t.joinable()) t.join();
    }
    threads_.clear();
    if (recorder_) recorder_->flush();
}

// ---------------------------------------------------------------------------------------
// Stages
// ---------------------------------------------------------------------------------------

void Pipeline::lidar_stage() {
    stage_setup(Stage::Lidar, "axle-lidar", true);
    LidarProcessor proc;
    Backoff backoff(cfg_.spin_before_yield);
    for (;;) {
        if (auto frame = lidar_q_.try_pop()) {
            backoff.reset();
            bump(c_.lidar_in);
            LidarResult r = proc.process(*frame);
            lat_lidar_.record(r.processed_ns - r.ingest_ns);
            push(result_q_, std::move(r), c_.result_dropped);
            continue;
        }
        if (sources_done_.load(std::memory_order_acquire) && lidar_q_.empty_approx()) break;
        backoff.idle();
    }
    lidar_done_.store(true, std::memory_order_release);
}

void Pipeline::imu_stage() {
    stage_setup(Stage::Imu, "axle-imu", true);
    ImuFilter filter;
    Backoff backoff(cfg_.spin_before_yield);
    for (;;) {
        if (auto s = imu_q_.try_pop()) {
            backoff.reset();
            bump(c_.imu_in);
            ImuState st = filter.update(*s);
            lat_imu_.record(now_ns() - st.ingest_ns);
            push(imu_state_q_, std::move(st), c_.state_dropped);
            continue;
        }
        if (sources_done_.load(std::memory_order_acquire) && imu_q_.empty_approx()) break;
        backoff.idle();
    }
    imu_done_.store(true, std::memory_order_release);
}

void Pipeline::can_stage() {
    stage_setup(Stage::Can, "axle-can", true);
    CanDecoder decoder;
    Backoff backoff(cfg_.spin_before_yield);
    for (;;) {
        if (auto f = can_q_.try_pop()) {
            backoff.reset();
            bump(c_.can_in);
            const auto before = decoder.stats();
            if (decoder.on_frame(*f)) {
                VehicleState st = decoder.state();
                lat_can_.record(now_ns() - st.ingest_ns);
                push(veh_state_q_, std::move(st), c_.state_dropped);
            }
            const auto& after = decoder.stats();
            bump(c_.can_bad_checksum, after.bad_checksum - before.bad_checksum);
            bump(c_.can_counter_jumps, after.counter_jumps - before.counter_jumps);
            bump(c_.can_unknown, after.unknown_id - before.unknown_id);
            continue;
        }
        if (sources_done_.load(std::memory_order_acquire) && can_q_.empty_approx()) break;
        backoff.idle();
    }
    can_done_.store(true, std::memory_order_release);
}

void Pipeline::fusion_stage() {
    stage_setup(Stage::Fusion, "axle-fusion", true);
    Fusion fusion;
    Backoff backoff(cfg_.spin_before_yield);
    std::optional<LidarResult> pending;
    Nanos last_imu_ts = std::numeric_limits<Nanos>::min();
    Nanos last_veh_ts = std::numeric_limits<Nanos>::min();

    // Feed states into the fusion history. With a LiDAR result pending only states up to
    // its timestamp (+ lookahead) are taken, keeping the join time-aligned even when the
    // IMU/CAN stages run far ahead (e.g. fast replay). Without LiDAR input everything is
    // drained so producers never stall.
    auto drain_states = [&](bool bounded, Nanos horizon) {
        bool any = false;
        while (ImuState* s = imu_state_q_.front()) {
            if (bounded && s->sensor_ts_ns > horizon) break;
            fusion.on_imu(*s);
            last_imu_ts = s->sensor_ts_ns;
            imu_state_q_.try_pop();
            any = true;
        }
        while (VehicleState* v = veh_state_q_.front()) {
            if (bounded && v->sensor_ts_ns > horizon) break;
            fusion.on_vehicle(*v);
            last_veh_ts = v->sensor_ts_ns;
            veh_state_q_.try_pop();
            any = true;
        }
        return any;
    };

    for (;;) {
        if (!pending) pending = result_q_.try_pop();

        if (pending) {
            backoff.reset();
            const Nanos target = pending->sensor_ts_ns;
            drain_states(true, target + kFusionLookahead);
            if (cfg_.backpressure) {
                // Offline runs: the state streams may lag the LiDAR stream, so wait until each
                // has reached the sweep time (or finished) to make the join deterministic.
                // A state still queued after a bounded drain is newer than the horizon, so
                // its presence also means the stream has caught up.
                const Nanos wait_start = now_ns();
                for (;;) {
                    const bool imu_ok = !has_imu_ || last_imu_ts >= target || imu_state_q_.front() ||
                                        imu_done_.load(std::memory_order_acquire);
                    const bool veh_ok = !has_can_ || last_veh_ts >= target || veh_state_q_.front() ||
                                        can_done_.load(std::memory_order_acquire);
                    if ((imu_ok && veh_ok) || now_ns() - wait_start > kMaxAlignWait) break;
                    cpu_relax();
                    drain_states(true, target + kFusionLookahead);
                }
            }
            FusedFrame fused = fusion.fuse(std::move(*pending));
            pending.reset();

            const Nanos e2e = fused.fused_ns - fused.ingest_ns;
            const bool over = e2e > cfg_.latency_budget_ns;
            lat_e2e_.record(e2e);
            bump(c_.frames_fused);
            if (over) bump(c_.over_budget);
            if (!fused.imu_valid) bump(c_.imu_unmatched);
            if (!fused.vehicle_valid) bump(c_.vehicle_unmatched);

            ExportItem item{std::move(fused), e2e, over};
            push(export_q_, std::move(item), c_.export_dropped);
            continue;
        }

        // While LiDAR results may still arrive, states wait in their queues for the next
        // sweep. Once the LiDAR stage has finished, drain them freely so upstream can exit.
        const bool lidar_finished =
            !has_lidar_ || (lidar_done_.load(std::memory_order_acquire) && result_q_.empty_approx());
        if (lidar_finished && drain_states(false, 0)) {
            backoff.reset();
            continue;
        }
        if (lidar_finished && imu_done_.load(std::memory_order_acquire) &&
            can_done_.load(std::memory_order_acquire) && imu_state_q_.empty_approx() &&
            veh_state_q_.empty_approx()) {
            break;
        }
        backoff.idle();
    }
    fusion_done_.store(true, std::memory_order_release);
}

void Pipeline::export_stage() {
    stage_setup(Stage::Export, "axle-export", false);
    Backoff backoff(cfg_.spin_before_yield);
    Nanos last_flush = now_ns();
    for (;;) {
        if (auto item = export_q_.try_pop()) {
            backoff.reset();
            try {
                sink_->write(item->frame, item->e2e_ns, item->over_budget);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "[axle] export failed: %s\n", e.what());
                bump(c_.export_dropped);
                continue;
            }
            lat_export_.record(now_ns() - item->frame.ingest_ns);
            bump(c_.frames_exported);
            continue;
        }
        const Nanos now = now_ns();
        if (now - last_flush > ms_to_ns(1000)) {
            sink_->flush();
            last_flush = now;
        }
        if (fusion_done_.load(std::memory_order_acquire) && export_q_.empty_approx()) break;
        backoff.idle();
    }
    sink_->close();
    export_done_.store(true, std::memory_order_release);
}

// ---------------------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------------------

PipelineSnapshot Pipeline::snapshot() const {
    PipelineSnapshot s;
    s.lidar_in = load(c_.lidar_in);
    s.imu_in = load(c_.imu_in);
    s.can_in = load(c_.can_in);
    s.lidar_dropped = load(c_.lidar_dropped);
    s.imu_dropped = load(c_.imu_dropped);
    s.can_dropped = load(c_.can_dropped);
    s.state_dropped = load(c_.state_dropped);
    s.result_dropped = load(c_.result_dropped);
    s.export_dropped = load(c_.export_dropped);
    s.frames_fused = load(c_.frames_fused);
    s.frames_exported = load(c_.frames_exported);
    s.over_budget = load(c_.over_budget);
    s.can_bad_checksum = load(c_.can_bad_checksum);
    s.can_counter_jumps = load(c_.can_counter_jumps);
    s.can_unknown = load(c_.can_unknown);
    s.imu_unmatched = load(c_.imu_unmatched);
    s.vehicle_unmatched = load(c_.vehicle_unmatched);
    return s;
}

std::string Pipeline::report() const {
    const auto s = snapshot();
    std::ostringstream os;
    os << "frames fused=" << s.frames_fused << " exported=" << s.frames_exported
       << " over_budget=" << s.over_budget << " (budget " << ns_to_ms(cfg_.latency_budget_ns) << " ms)\n";
    os << "inputs lidar=" << s.lidar_in << " imu=" << s.imu_in << " can=" << s.can_in << '\n';
    os << "drops lidar=" << s.lidar_dropped << " imu=" << s.imu_dropped << " can=" << s.can_dropped
       << " state=" << s.state_dropped << " result=" << s.result_dropped << " export=" << s.export_dropped
       << '\n';
    os << "can bad_checksum=" << s.can_bad_checksum << " counter_jumps=" << s.can_counter_jumps
       << " unknown_id=" << s.can_unknown << '\n';
    os << lat_lidar_.summary("lidar-stage") << '\n';
    os << lat_imu_.summary("imu-stage") << '\n';
    os << lat_can_.summary("can-stage") << '\n';
    os << lat_e2e_.summary("end-to-end") << '\n';
    os << lat_export_.summary("exported") << '\n';
    return os.str();
}

std::string Pipeline::summary_json(const std::string& vehicle_id, const std::string& run_id) const {
    const auto s = snapshot();
    auto hist = [](const LatencyHistogram& h) {
        char buf[256];
        std::snprintf(buf, sizeof buf,
                      "{\"count\":%llu,\"mean_ms\":%.4f,\"p50_ms\":%.4f,\"p95_ms\":%.4f,\"p99_ms\":%.4f,"
                      "\"max_ms\":%.4f}",
                      static_cast<unsigned long long>(h.count()), h.mean() / 1e6, ns_to_ms(h.percentile(50)),
                      ns_to_ms(h.percentile(95)), ns_to_ms(h.percentile(99)), ns_to_ms(h.max()));
        return std::string(buf);
    };
    std::ostringstream os;
    os << "{\"vehicle_id\":\"" << vehicle_id << "\",\"run_id\":\"" << run_id << "\""
       << ",\"budget_ms\":" << ns_to_ms(cfg_.latency_budget_ns) << ",\"frames_fused\":" << s.frames_fused
       << ",\"frames_exported\":" << s.frames_exported << ",\"over_budget\":" << s.over_budget
       << ",\"inputs\":{\"lidar\":" << s.lidar_in << ",\"imu\":" << s.imu_in << ",\"can\":" << s.can_in << "}"
       << ",\"drops\":{\"lidar\":" << s.lidar_dropped << ",\"imu\":" << s.imu_dropped
       << ",\"can\":" << s.can_dropped << ",\"state\":" << s.state_dropped << ",\"result\":" << s.result_dropped
       << ",\"export\":" << s.export_dropped << "}"
       << ",\"can\":{\"bad_checksum\":" << s.can_bad_checksum << ",\"counter_jumps\":" << s.can_counter_jumps
       << ",\"unknown_id\":" << s.can_unknown << "}"
       << ",\"latency\":{\"lidar_stage\":" << hist(lat_lidar_) << ",\"imu_stage\":" << hist(lat_imu_)
       << ",\"can_stage\":" << hist(lat_can_) << ",\"end_to_end\":" << hist(lat_e2e_)
       << ",\"exported\":" << hist(lat_export_) << "}}";
    return os.str();
}

}  // namespace axle
