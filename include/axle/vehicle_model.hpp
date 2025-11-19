#pragma once

#include <cstdint>
#include <vector>

#include "axle/types.hpp"

namespace axle {

/// Kinematic state of the simulated ego vehicle at a point in time.
struct VehicleKinematics {
    double t_s{0};
    double x_m{0}, y_m{0};  // world position
    double yaw_rad{0};
    double speed_mps{0};
    double long_accel_mps2{0};
    double yaw_rate_rps{0};
    double steering_deg{0};  // steering-wheel angle
    double throttle_pct{0};
    double brake_bar{0};
    double engine_rpm{0};
    int gear{0};
};

/// Axis-aligned box obstacle in the world frame.
struct WorldBox {
    double cx, cy, cz;
    double hx, hy, hz;  // half extents
};

/// Deterministic drive scenario shared by all simulated sensors.
///
/// The speed profile is piecewise (accelerate, cruise, brake, stop, repeat) and steering is
/// a slow sinusoid, so the trajectory is a gentle S-curve. Position is integrated at a fixed
/// internal step and cached, so `at(t)` is a pure function of t and safe to call from
/// multiple sensor threads concurrently after construction.
class VehicleModel {
public:
    struct Params {
        double cycle_s = 40.0;         // length of one accelerate/cruise/brake/stop cycle
        double cruise_mps = 15.0;      // ~54 km/h
        double wheelbase_m = 2.8;
        double steering_ratio = 15.0;
        double max_steer_wheel_deg = 30.0;
        double steer_period_s = 20.0;
        double duration_s = 600.0;     // precomputed horizon; queries beyond it wrap
        double step_s = 0.001;
        std::uint64_t seed = 42;
        int num_obstacles = 60;
    };

    VehicleModel();
    explicit VehicleModel(Params p);

    VehicleKinematics at(double t_s) const;
    const std::vector<WorldBox>& obstacles() const noexcept { return obstacles_; }
    const Params& params() const noexcept { return params_; }

private:
    double speed_profile(double t_s, double* accel, double* throttle, double* brake) const;
    double steering_profile(double t_s) const;
    void precompute();
    void place_obstacles();

    Params params_;
    std::vector<VehicleKinematics> table_;  // sampled every 10 ms
    std::vector<WorldBox> obstacles_;
};

}  // namespace axle
