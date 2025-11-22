#pragma once

#include <deque>

#include "axle/types.hpp"

namespace axle {

/// Joins each processed LiDAR sweep with the IMU and vehicle-bus states closest to it in
/// sensor time, and derives frame-level indicators (time-to-collision on the ego corridor).
///
/// Not thread-safe: owned by the single fusion stage thread.
class Fusion {
public:
    struct Config {
        Nanos max_skew_ns = ms_to_ns(100);  // state older/newer than this is "invalid"
        std::size_t history = 256;          // per-stream ring of recent states
        float corridor_half_width_m = 1.4f;
        float corridor_length_m = 60.f;
        float min_speed_for_ttc_mps = 0.5f;
    };

    Fusion();
    explicit Fusion(Config cfg);

    void on_imu(const ImuState& s);
    void on_vehicle(const VehicleState& s);
    FusedFrame fuse(LidarResult&& lidar);

    /// Nearest in-corridor obstacle distance ahead (x > 0), or -1 if none.
    static float corridor_distance(const LidarResult& r, float half_width, float max_len);

private:
    template <typename T>
    static const T* closest(const std::deque<T>& hist, Nanos ts, Nanos max_skew);

    Config cfg_;
    std::deque<ImuState> imu_;
    std::deque<VehicleState> vehicle_;
};

}  // namespace axle
