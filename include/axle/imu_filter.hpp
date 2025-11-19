#pragma once

#include "axle/types.hpp"

namespace axle {

/// Complementary attitude filter.
///
/// Roll and pitch integrate the gyro and are pulled towards the gravity direction measured
/// by the accelerometer. The accelerometer correction gain is reduced when the measured
/// specific-force magnitude deviates from 1 g (vehicle accelerating/turning), which stops
/// longitudinal acceleration from being misread as pitch. Yaw is integrated from the bias-
/// corrected z gyro; the z bias is learned while the vehicle is detected as stationary.
class ImuFilter {
public:
    struct Config {
        float alpha = 0.995f;            // gyro weight per update (at nominal rate)
        float gate_mps2 = 0.6f;          // |norm - g| at which accel correction reaches zero
        float still_gyro_rps = 0.01f;    // max |gyro| to count as stationary
        float still_accel_mps2 = 0.15f;  // max |norm - g| to count as stationary
        int still_samples = 100;         // consecutive still samples before bias learning
        float bias_lr = 0.01f;           // EMA rate for bias learning
        float max_dt_s = 0.1f;           // larger gaps reset integration (dropouts)
    };

    ImuFilter();
    explicit ImuFilter(Config cfg);

    ImuState update(const ImuSample& s);
    void reset();

    float gyro_bias_z() const noexcept { return bias_z_; }
    bool initialised() const noexcept { return initialised_; }

private:
    Config cfg_;
    bool initialised_{false};
    Nanos last_ts_{0};
    float roll_{0}, pitch_{0}, yaw_{0};
    float bias_z_{0};
    int still_count_{0};
};

}  // namespace axle
