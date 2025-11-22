#include "axle/fusion.hpp"

#include <algorithm>
#include <iterator>
#include <limits>

namespace axle {

Fusion::Fusion() : Fusion(Config{}) {}
Fusion::Fusion(Config cfg) : cfg_(cfg) {}

void Fusion::on_imu(const ImuState& s) {
    imu_.push_back(s);
    while (imu_.size() > cfg_.history) imu_.pop_front();
}

void Fusion::on_vehicle(const VehicleState& s) {
    vehicle_.push_back(s);
    while (vehicle_.size() > cfg_.history) vehicle_.pop_front();
}

template <typename T>
const T* Fusion::closest(const std::deque<T>& hist, Nanos ts, Nanos max_skew) {
    if (hist.empty()) return nullptr;
    // History is (nearly) sorted by sensor time; binary search for the insertion point
    // and compare the neighbours on either side.
    auto it = std::lower_bound(hist.begin(), hist.end(), ts,
                               [](const T& s, Nanos t) { return s.sensor_ts_ns < t; });
    const T* best = nullptr;
    Nanos best_d = std::numeric_limits<Nanos>::max();
    auto consider = [&](typename std::deque<T>::const_iterator c) {
        if (c == hist.end()) return;
        const Nanos diff = c->sensor_ts_ns - ts;
        const Nanos d = diff < 0 ? -diff : diff;
        if (d < best_d) {
            best_d = d;
            best = &*c;
        }
    };
    consider(it);
    if (it != hist.begin()) consider(std::prev(it));
    return best_d <= max_skew ? best : nullptr;
}

float Fusion::corridor_distance(const LidarResult& r, float half_width, float max_len) {
    float best = std::numeric_limits<float>::max();
    for (const auto& ob : r.obstacles) {
        if (ob.max.x <= 0.f || ob.min.x > max_len) continue;
        if (ob.min.y > half_width || ob.max.y < -half_width) continue;
        best = std::min(best, std::max(0.f, ob.min.x));
    }
    return best == std::numeric_limits<float>::max() ? -1.f : best;
}

FusedFrame Fusion::fuse(LidarResult&& lidar) {
    FusedFrame out;
    out.seq = lidar.seq;
    out.sensor_ts_ns = lidar.sensor_ts_ns;
    out.ingest_ns = lidar.ingest_ns;

    if (const auto* imu = closest(imu_, lidar.sensor_ts_ns, cfg_.max_skew_ns)) {
        out.imu = *imu;
        out.imu_valid = true;
    }
    if (const auto* veh = closest(vehicle_, lidar.sensor_ts_ns, cfg_.max_skew_ns)) {
        out.vehicle = *veh;
        out.vehicle_valid = true;
    }

    const float dist = corridor_distance(lidar, cfg_.corridor_half_width_m, cfg_.corridor_length_m);
    if (out.vehicle_valid && dist >= 0.f && out.vehicle.speed_mps > cfg_.min_speed_for_ttc_mps) {
        out.time_to_collision_s = dist / out.vehicle.speed_mps;
    }

    out.lidar = std::move(lidar);
    out.fused_ns = now_ns();
    return out;
}

}  // namespace axle
