#include "axle/imu_filter.hpp"

#include <algorithm>
#include <cmath>

namespace axle {

namespace {
constexpr float kG = 9.80665f;
constexpr float kPi = 3.14159265358979f;

inline float wrap_pi(float a) {
    while (a > kPi) a -= 2.f * kPi;
    while (a < -kPi) a += 2.f * kPi;
    return a;
}
}  // namespace

ImuFilter::ImuFilter() : ImuFilter(Config{}) {}
ImuFilter::ImuFilter(Config cfg) : cfg_(cfg) {}

void ImuFilter::reset() {
    initialised_ = false;
    roll_ = pitch_ = yaw_ = 0.f;
    still_count_ = 0;
}

ImuState ImuFilter::update(const ImuSample& s) {
    const float ax = s.accel.x, ay = s.accel.y, az = s.accel.z;
    const float norm = std::sqrt(ax * ax + ay * ay + az * az);
    const float acc_roll = std::atan2(ay, az);
    const float acc_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    const float dev = std::abs(norm - kG);
    float dt = 0.f;
    if (initialised_) dt = static_cast<float>(s.sensor_ts_ns - last_ts_) * 1e-9f;

    if (!initialised_ || dt <= 0.f || dt > cfg_.max_dt_s) {
        // (Re)initialise attitude from gravity; keep the yaw estimate across dropouts.
        roll_ = acc_roll;
        pitch_ = acc_pitch;
        initialised_ = true;
        dt = 0.f;
    } else {
        // Stationary detection drives z-gyro bias learning.
        const float gyro_mag = std::sqrt(s.gyro.x * s.gyro.x + s.gyro.y * s.gyro.y + s.gyro.z * s.gyro.z);
        if (gyro_mag < cfg_.still_gyro_rps + std::abs(bias_z_) && dev < cfg_.still_accel_mps2) {
            if (++still_count_ >= cfg_.still_samples) {
                bias_z_ += cfg_.bias_lr * (s.gyro.z - bias_z_);
            }
        } else {
            still_count_ = 0;
        }

        // Gyro propagation (small-angle Euler rates are fine for road-vehicle attitudes).
        const float gyro_roll = roll_ + s.gyro.x * dt;
        const float gyro_pitch = pitch_ + s.gyro.y * dt;

        // Adaptive gain: full correction at 1 g, none beyond the gate.
        const float trust = std::clamp(1.f - dev / cfg_.gate_mps2, 0.f, 1.f);
        const float k = (1.f - cfg_.alpha) * trust;
        roll_ = (1.f - k) * gyro_roll + k * acc_roll;
        pitch_ = (1.f - k) * gyro_pitch + k * acc_pitch;
        yaw_ = wrap_pi(yaw_ + (s.gyro.z - bias_z_) * dt);
    }
    last_ts_ = s.sensor_ts_ns;

    ImuState out;
    out.seq = s.seq;
    out.sensor_ts_ns = s.sensor_ts_ns;
    out.ingest_ns = s.ingest_ns;
    out.roll_rad = roll_;
    out.pitch_rad = pitch_;
    out.yaw_rad = yaw_;
    out.yaw_rate_rps = s.gyro.z - bias_z_;
    // Remove the gravity component projected onto the body axes.
    out.long_accel_mps2 = ax + kG * std::sin(pitch_);
    out.lat_accel_mps2 = ay - kG * std::sin(roll_) * std::cos(pitch_);
    out.accel_norm_mps2 = norm;
    return out;
}

}  // namespace axle
