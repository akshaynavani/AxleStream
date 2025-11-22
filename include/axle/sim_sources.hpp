#pragma once

#include <memory>

#include "axle/source.hpp"
#include "axle/vehicle_model.hpp"

namespace axle {

/// Settings shared by the simulated sensors. All simulated sources created from the same
/// SimConfig share a start time so their timestamps line up.
struct SimConfig {
    double duration_s = 0.0;  // 0 = run until stopped
    bool realtime = true;     // false = generate as fast as downstream accepts
    Nanos start_wall_ns = 0;  // filled in by make_sim_sources() if zero
    Nanos start_mono_ns = 0;
    std::uint64_t seed = 7;

    // LiDAR: VLP-16 style spinning scanner
    double lidar_hz = 10.0;
    int lidar_beams = 16;
    double lidar_min_elev_deg = -15.0;
    double lidar_max_elev_deg = 15.0;
    double lidar_h_res_deg = 0.4;
    double lidar_max_range_m = 100.0;
    double lidar_mount_height_m = 1.8;
    double lidar_range_noise_m = 0.02;
    double lidar_dropout = 0.02;

    // IMU
    double imu_hz = 200.0;
    double imu_accel_noise = 0.05;   // m/s^2 (1 sigma)
    double imu_gyro_noise = 0.002;   // rad/s
    double imu_gyro_bias_z = 0.001;  // rad/s constant bias

    // CAN: base tick rate; individual messages are emitted at divisors of it
    double can_hz = 100.0;
    double can_corrupt_prob = 0.0;   // probability of a deliberately bad checksum (fault injection)
};

class SimLidarSource final : public Source {
public:
    SimLidarSource(std::shared_ptr<const VehicleModel> model, SimConfig cfg);
    std::string name() const override { return "sim-lidar"; }
    std::uint8_t kinds() const override { return kLidarMask; }
    void run(const std::atomic<bool>& stop, const SensorSinks& sinks) override;

    /// Generate the sweep for a given simulation time (exposed for tests/benchmarks).
    LidarFrame generate(double t_s, std::uint64_t seq);

private:
    std::shared_ptr<const VehicleModel> model_;
    SimConfig cfg_;
    std::uint64_t rng_state_;
};

class SimImuSource final : public Source {
public:
    SimImuSource(std::shared_ptr<const VehicleModel> model, SimConfig cfg);
    std::string name() const override { return "sim-imu"; }
    std::uint8_t kinds() const override { return kImuMask; }
    void run(const std::atomic<bool>& stop, const SensorSinks& sinks) override;

private:
    std::shared_ptr<const VehicleModel> model_;
    SimConfig cfg_;
};

class SimCanSource final : public Source {
public:
    SimCanSource(std::shared_ptr<const VehicleModel> model, SimConfig cfg);
    std::string name() const override { return "sim-can"; }
    std::uint8_t kinds() const override { return kCanMask; }
    void run(const std::atomic<bool>& stop, const SensorSinks& sinks) override;

private:
    std::shared_ptr<const VehicleModel> model_;
    SimConfig cfg_;
};

/// Convenience: build LiDAR + IMU + CAN simulators over one shared VehicleModel.
std::vector<SourcePtr> make_sim_sources(SimConfig cfg);

}  // namespace axle
