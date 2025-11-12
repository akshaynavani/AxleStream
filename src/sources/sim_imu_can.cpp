#include <cmath>
#include <random>

#include "axle/can_db.hpp"
#include "axle/sim_sources.hpp"

namespace axle {

namespace {
constexpr double kGravity = 9.80665;

/// Shared tick loop: invokes `body(tick, offset_ns)` at `hz`, honouring realtime pacing,
/// the configured duration and the stop flag.
template <typename Body>
void tick_loop(const SimConfig& cfg, double hz, const std::atomic<bool>& stop, Body&& body) {
    const Nanos period = static_cast<Nanos>(1e9 / hz);
    const std::uint64_t max_ticks = cfg.duration_s > 0 ? static_cast<std::uint64_t>(cfg.duration_s * hz) : 0;
    for (std::uint64_t tick = 0; !stop.load(std::memory_order_relaxed); ++tick) {
        if (max_ticks && tick >= max_ticks) break;
        const Nanos offset = static_cast<Nanos>(tick) * period;
        if (cfg.realtime) sleep_until_ns(cfg.start_mono_ns + offset);
        body(tick, offset);
    }
}
}  // namespace

// ---------------------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------------------

SimImuSource::SimImuSource(std::shared_ptr<const VehicleModel> model, SimConfig cfg)
    : model_(std::move(model)), cfg_(cfg) {}

void SimImuSource::run(const std::atomic<bool>& stop, const SensorSinks& sinks) {
    std::mt19937_64 rng(cfg_.seed + 1);
    std::normal_distribution<double> accel_noise(0.0, cfg_.imu_accel_noise);
    std::normal_distribution<double> gyro_noise(0.0, cfg_.imu_gyro_noise);

    tick_loop(cfg_, cfg_.imu_hz, stop, [&](std::uint64_t tick, Nanos offset) {
        const auto k = model_->at(static_cast<double>(offset) / 1e9);
        ImuSample s;
        s.seq = tick;
        s.sensor_ts_ns = cfg_.start_wall_ns + offset;
        // Body frame: x forward, y left, z up. A level, stationary IMU reads +g on z.
        s.accel.x = static_cast<float>(k.long_accel_mps2 + accel_noise(rng));
        s.accel.y = static_cast<float>(k.speed_mps * k.yaw_rate_rps + accel_noise(rng));
        s.accel.z = static_cast<float>(kGravity + accel_noise(rng));
        s.gyro.x = static_cast<float>(gyro_noise(rng));
        s.gyro.y = static_cast<float>(gyro_noise(rng));
        s.gyro.z = static_cast<float>(k.yaw_rate_rps + cfg_.imu_gyro_bias_z + gyro_noise(rng));
        s.ingest_ns = now_ns();
        emit(sinks.imu, std::move(s));
    });
}

// ---------------------------------------------------------------------------------------
// CAN
// ---------------------------------------------------------------------------------------

SimCanSource::SimCanSource(std::shared_ptr<const VehicleModel> model, SimConfig cfg)
    : model_(std::move(model)), cfg_(cfg) {}

void SimCanSource::run(const std::atomic<bool>& stop, const SensorSinks& sinks) {
    std::mt19937_64 rng(cfg_.seed + 2);
    std::bernoulli_distribution corrupt(cfg_.can_corrupt_prob);
    std::uint8_t counters[5] = {0, 0, 0, 0, 0};
    std::uint64_t seq = 0;

    tick_loop(cfg_, cfg_.can_hz, stop, [&](std::uint64_t tick, Nanos offset) {
        const auto k = model_->at(static_cast<double>(offset) / 1e9);
        const Nanos ts = cfg_.start_wall_ns + offset;

        auto send = [&](CanFrame f, int counter_slot) {
            ++counters[counter_slot];
            if (corrupt(rng)) f.data[7] ^= 0x5A;  // injected bus fault
            f.seq = seq++;
            f.sensor_ts_ns = ts;
            f.ingest_ns = now_ns();
            emit(sinks.can, std::move(f));
        };

        // 100 Hz messages
        send(can_db::encode_wheel_speed(k.speed_mps, counters[0]), 0);
        send(can_db::encode_steering(k.steering_deg, counters[1]), 1);
        // 50 Hz messages
        if (tick % 2 == 0) {
            send(can_db::encode_engine(k.engine_rpm, k.throttle_pct, counters[2]), 2);
            send(can_db::encode_brake(k.brake_bar, k.brake_bar > 6.0, counters[3]), 3);
        }
        // 10 Hz messages
        if (tick % 10 == 0) {
            send(can_db::encode_gear(k.gear, counters[4]), 4);
        }
    });
}

// ---------------------------------------------------------------------------------------

std::vector<SourcePtr> make_sim_sources(SimConfig cfg) {
    // Build the scenario first: precomputing the trajectory takes a few milliseconds and
    // must not eat into the realtime schedule.
    VehicleModel::Params p;
    p.seed = cfg.seed;
    if (cfg.duration_s > p.duration_s) p.duration_s = cfg.duration_s;
    auto model = std::make_shared<const VehicleModel>(p);

    if (cfg.start_mono_ns == 0) cfg.start_mono_ns = now_ns() + ms_to_ns(50);  // small warm-up
    if (cfg.start_wall_ns == 0) cfg.start_wall_ns = wall_ns() + ms_to_ns(50);

    std::vector<SourcePtr> out;
    out.push_back(std::make_unique<SimLidarSource>(model, cfg));
    out.push_back(std::make_unique<SimImuSource>(model, cfg));
    out.push_back(std::make_unique<SimCanSource>(model, cfg));
    return out;
}

}  // namespace axle
