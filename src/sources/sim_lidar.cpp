#include <algorithm>
#include <cmath>
#include <limits>

#include "axle/sim_sources.hpp"

namespace axle {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

// xorshift64* — tiny, fast PRNG; plenty for sensor noise and keeps the hot loop cheap.
inline std::uint64_t next_u64(std::uint64_t& s) {
    s ^= s >> 12;
    s ^= s << 25;
    s ^= s >> 27;
    return s * 0x2545F4914F6CDD1DULL;
}
inline double uniform01(std::uint64_t& s) {
    return static_cast<double>(next_u64(s) >> 11) * (1.0 / 9007199254740992.0);
}
// Irwin-Hall approximation of a standard normal (sum of 4 uniforms), cheap and bounded.
inline double approx_normal(std::uint64_t& s) {
    const double sum = uniform01(s) + uniform01(s) + uniform01(s) + uniform01(s);
    return (sum - 2.0) * 1.7320508075688772;  // sqrt(12/4)
}

/// Slab test: distance along a unit ray to an AABB, or +inf if missed.
inline double ray_box(double ox, double oy, double oz, double dx, double dy, double dz, const WorldBox& b) {
    double tmin = 0.0;
    double tmax = std::numeric_limits<double>::infinity();
    const double o[3] = {ox, oy, oz};
    const double d[3] = {dx, dy, dz};
    const double lo[3] = {b.cx - b.hx, b.cy - b.hy, b.cz - b.hz};
    const double hi[3] = {b.cx + b.hx, b.cy + b.hy, b.cz + b.hz};
    for (int a = 0; a < 3; ++a) {
        if (std::abs(d[a]) < 1e-12) {
            if (o[a] < lo[a] || o[a] > hi[a]) return std::numeric_limits<double>::infinity();
            continue;
        }
        const double inv = 1.0 / d[a];
        double t0 = (lo[a] - o[a]) * inv;
        double t1 = (hi[a] - o[a]) * inv;
        if (t0 > t1) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmin > tmax) return std::numeric_limits<double>::infinity();
    }
    return tmin;
}

}  // namespace

SimLidarSource::SimLidarSource(std::shared_ptr<const VehicleModel> model, SimConfig cfg)
    : model_(std::move(model)), cfg_(cfg), rng_state_(cfg.seed * 0x9E3779B97F4A7C15ULL + 1) {}

LidarFrame SimLidarSource::generate(double t_s, std::uint64_t seq) {
    const auto k = model_->at(t_s);
    const double ox = k.x_m, oy = k.y_m, oz = cfg_.lidar_mount_height_m;
    const double cy = std::cos(k.yaw_rad), sy = std::sin(k.yaw_rad);
    const double max_r = cfg_.lidar_max_range_m;

    // Cull obstacles to those within range of the sensor for this sweep.
    std::vector<const WorldBox*> nearby;
    for (const auto& b : model_->obstacles()) {
        const double ddx = b.cx - ox, ddy = b.cy - oy;
        const double reach = max_r + std::max(b.hx, b.hy);
        if (ddx * ddx + ddy * ddy < reach * reach) nearby.push_back(&b);
    }

    const int az_steps = static_cast<int>(std::lround(360.0 / cfg_.lidar_h_res_deg));
    const int beams = std::max(1, cfg_.lidar_beams);
    const double elev_step =
        beams > 1 ? (cfg_.lidar_max_elev_deg - cfg_.lidar_min_elev_deg) / (beams - 1) : 0.0;

    LidarFrame frame;
    frame.seq = seq;
    frame.points.reserve(static_cast<std::size_t>(az_steps * beams));

    for (int a = 0; a < az_steps; ++a) {
        const double az = a * cfg_.lidar_h_res_deg * kDeg;  // sensor frame azimuth
        const double ca = std::cos(az), sa = std::sin(az);
        for (int b = 0; b < beams; ++b) {
            if (uniform01(rng_state_) < cfg_.lidar_dropout) continue;
            const double el = (cfg_.lidar_min_elev_deg + b * elev_step) * kDeg;
            const double ce = std::cos(el), se = std::sin(el);

            // Direction in vehicle frame, then rotated into world frame.
            const double vx = ce * ca, vy = ce * sa, vz = se;
            const double wx = vx * cy - vy * sy;
            const double wy = vx * sy + vy * cy;
            const double wz = vz;

            double range = std::numeric_limits<double>::infinity();
            bool hit_obstacle = false;
            if (wz < -1e-6) {
                range = -oz / wz;  // ground plane z = 0
            }
            for (const WorldBox* box : nearby) {
                const double r = ray_box(ox, oy, oz, wx, wy, wz, *box);
                if (r < range) {
                    range = r;
                    hit_obstacle = true;
                }
            }
            if (!(range < max_r) || range < 0.5) continue;

            range += approx_normal(rng_state_) * cfg_.lidar_range_noise_m;
            LidarPoint p;
            p.x = static_cast<float>(range * vx);
            p.y = static_cast<float>(range * vy);
            p.z = static_cast<float>(range * vz);
            const double base = hit_obstacle ? 140.0 : 30.0;
            p.intensity = static_cast<float>(std::clamp(base + approx_normal(rng_state_) * 15.0, 0.0, 255.0));
            frame.points.push_back(p);
        }
    }
    return frame;
}

void SimLidarSource::run(const std::atomic<bool>& stop, const SensorSinks& sinks) {
    const Nanos period = static_cast<Nanos>(1e9 / cfg_.lidar_hz);
    const std::uint64_t max_frames =
        cfg_.duration_s > 0 ? static_cast<std::uint64_t>(cfg_.duration_s * cfg_.lidar_hz) : 0;

    for (std::uint64_t seq = 0; !stop.load(std::memory_order_relaxed); ++seq) {
        if (max_frames && seq >= max_frames) break;
        const Nanos offset = static_cast<Nanos>(seq) * period;
        if (cfg_.realtime) {
            sleep_until_ns(cfg_.start_mono_ns + offset);
        }
        // A real scanner timestamps the frame at the end of the sweep; mimic that.
        LidarFrame frame = generate(static_cast<double>(offset) / 1e9, seq);
        frame.sensor_ts_ns = cfg_.start_wall_ns + offset;
        frame.ingest_ns = now_ns();
        emit(sinks.lidar, std::move(frame));
    }
}

}  // namespace axle
