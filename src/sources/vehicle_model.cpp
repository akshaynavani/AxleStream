#include "axle/vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace axle {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kSampleDt = 0.01;  // table resolution
constexpr double kWheelRadius = 0.33;
constexpr double kFinalDrive = 3.7;
constexpr double kGearRatios[] = {3.5, 2.1, 1.4, 1.0, 0.8};
constexpr double kIdleRpm = 800.0;
}  // namespace

VehicleModel::VehicleModel() : VehicleModel(Params{}) {}

VehicleModel::VehicleModel(Params p) : params_(p) {
    precompute();
    place_obstacles();
}

double VehicleModel::speed_profile(double t_s, double* accel, double* throttle, double* brake) const {
    const double cycle = params_.cycle_s;
    const double tc = std::fmod(t_s, cycle);
    const double accel_end = cycle * 0.20;
    const double cruise_end = cycle * 0.70;
    const double brake_end = cycle * 0.85;
    const double v = params_.cruise_mps;

    if (tc < accel_end) {
        *accel = v / accel_end;
        *throttle = 60.0;
        *brake = 0.0;
        return *accel * tc;
    }
    if (tc < cruise_end) {
        *accel = 0.0;
        *throttle = 20.0;
        *brake = 0.0;
        return v;
    }
    if (tc < brake_end) {
        const double decel = v / (brake_end - cruise_end);
        *accel = -decel;
        *throttle = 0.0;
        *brake = 8.0 + decel * 6.0;  // rough pressure for the requested deceleration
        return v - decel * (tc - cruise_end);
    }
    *accel = 0.0;
    *throttle = 0.0;
    *brake = 5.0;  // holding pressure at standstill
    return 0.0;
}

double VehicleModel::steering_profile(double t_s) const {
    return params_.max_steer_wheel_deg * std::sin(2.0 * kPi * t_s / params_.steer_period_s);
}

void VehicleModel::precompute() {
    const auto samples = static_cast<std::size_t>(params_.duration_s / kSampleDt) + 1;
    table_.clear();
    table_.reserve(samples);

    double x = 0.0, y = 0.0, yaw = 0.0;
    double t = 0.0;
    double next_sample = 0.0;
    const double dt = params_.step_s;

    while (table_.size() < samples) {
        double accel = 0, throttle = 0, brake = 0;
        const double v = speed_profile(t, &accel, &throttle, &brake);
        const double steer_wheel = steering_profile(t);
        const double road_wheel_rad = (steer_wheel / params_.steering_ratio) * kPi / 180.0;
        const double yaw_rate = v * std::tan(road_wheel_rad) / params_.wheelbase_m;

        if (t + 1e-9 >= next_sample) {
            VehicleKinematics k;
            k.t_s = t;
            k.x_m = x;
            k.y_m = y;
            k.yaw_rad = yaw;
            k.speed_mps = v;
            k.long_accel_mps2 = accel;
            k.yaw_rate_rps = yaw_rate;
            k.steering_deg = steer_wheel;
            k.throttle_pct = throttle;
            k.brake_bar = brake;

            int gear = 0;
            if (v > 0.05) {
                gear = v < 5 ? 1 : v < 9 ? 2 : v < 13 ? 3 : v < 17 ? 4 : 5;
            }
            k.gear = gear;
            if (gear == 0) {
                k.engine_rpm = kIdleRpm;
            } else {
                const double wheel_rps = v / (2.0 * kPi * kWheelRadius);
                const double rpm = wheel_rps * 60.0 * kGearRatios[gear - 1] * kFinalDrive;
                k.engine_rpm = std::max(kIdleRpm, rpm);
            }
            table_.push_back(k);
            next_sample += kSampleDt;
        }

        x += v * std::cos(yaw) * dt;
        y += v * std::sin(yaw) * dt;
        yaw += yaw_rate * dt;
        t += dt;
    }
}

VehicleKinematics VehicleModel::at(double t_s) const {
    if (t_s < 0) t_s = 0;
    const double horizon = static_cast<double>(table_.size() - 1) * kSampleDt;
    const double t = std::fmod(t_s, horizon);
    const double pos = t / kSampleDt;
    const auto i = std::min(static_cast<std::size_t>(pos), table_.size() - 2);
    const double f = pos - static_cast<double>(i);
    const auto& a = table_[i];
    const auto& b = table_[i + 1];
    auto lerp = [f](double u, double w) { return u + (w - u) * f; };

    VehicleKinematics k = f < 0.5 ? a : b;  // discrete fields (gear) from nearest sample
    k.t_s = t_s;
    k.x_m = lerp(a.x_m, b.x_m);
    k.y_m = lerp(a.y_m, b.y_m);
    k.yaw_rad = lerp(a.yaw_rad, b.yaw_rad);
    k.speed_mps = lerp(a.speed_mps, b.speed_mps);
    k.long_accel_mps2 = lerp(a.long_accel_mps2, b.long_accel_mps2);
    k.yaw_rate_rps = lerp(a.yaw_rate_rps, b.yaw_rate_rps);
    k.steering_deg = lerp(a.steering_deg, b.steering_deg);
    k.throttle_pct = lerp(a.throttle_pct, b.throttle_pct);
    k.brake_bar = lerp(a.brake_bar, b.brake_bar);
    k.engine_rpm = lerp(a.engine_rpm, b.engine_rpm);
    return k;
}

void VehicleModel::place_obstacles() {
    std::mt19937_64 rng(params_.seed);
    std::uniform_real_distribution<double> when(2.0, params_.duration_s - 1.0);
    std::uniform_real_distribution<double> lateral(2.5, 10.0);
    std::uniform_real_distribution<double> ahead(15.0, 40.0);
    std::bernoulli_distribution side(0.5);
    std::uniform_int_distribution<int> kind(0, 2);

    obstacles_.clear();
    for (int i = 0; i < params_.num_obstacles; ++i) {
        const auto k = at(when(rng));
        const double sign = side(rng) ? 1.0 : -1.0;
        const double fwd = ahead(rng);
        const double lat = sign * lateral(rng);
        const double c = std::cos(k.yaw_rad), s = std::sin(k.yaw_rad);
        WorldBox box{};
        box.cx = k.x_m + fwd * c - lat * s;
        box.cy = k.y_m + fwd * s + lat * c;
        switch (kind(rng)) {
            case 0:  // parked car
                box.hx = 2.2; box.hy = 0.95; box.hz = 0.75;
                break;
            case 1:  // pedestrian / pole
                box.hx = 0.3; box.hy = 0.3; box.hz = 0.9;
                break;
            default:  // truck / wall segment
                box.hx = 4.0; box.hy = 1.25; box.hz = 1.6;
                break;
        }
        box.cz = box.hz;  // resting on the ground plane (z = 0)
        obstacles_.push_back(box);
    }
}

}  // namespace axle
