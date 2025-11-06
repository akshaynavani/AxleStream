#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "axle/clock.hpp"

namespace axle {

/// Which sensor produced a raw message.
enum class SensorKind : std::uint8_t { Lidar = 1, Imu = 2, Can = 3 };

struct Vec3 {
    float x{0.f}, y{0.f}, z{0.f};
};

/// One LiDAR return in the sensor frame (metres) with intensity [0, 255].
struct LidarPoint {
    float x{0.f}, y{0.f}, z{0.f};
    float intensity{0.f};
};

/// A full LiDAR sweep. Point clouds are large, so frames are moved, never copied, through
/// the pipeline (queues hold the frame by value; the vector buffer is transferred).
struct LidarFrame {
    std::uint64_t seq{0};
    Nanos sensor_ts_ns{0};  // timestamp assigned by the sensor/simulator (wall clock)
    Nanos ingest_ns{0};     // monotonic time the frame entered the pipeline
    std::vector<LidarPoint> points;
};

/// One IMU sample: specific force (m/s^2) and angular rate (rad/s) in the body frame.
struct ImuSample {
    std::uint64_t seq{0};
    Nanos sensor_ts_ns{0};
    Nanos ingest_ns{0};
    Vec3 accel;
    Vec3 gyro;
};

/// Classic CAN 2.0 frame (up to 8 data bytes). Extended IDs flagged via `extended`.
struct CanFrame {
    std::uint64_t seq{0};
    Nanos sensor_ts_ns{0};
    Nanos ingest_ns{0};
    std::uint32_t id{0};
    std::uint8_t dlc{0};
    bool extended{false};
    std::array<std::uint8_t, 8> data{};
};

// ---------------------------------------------------------------------------------------
// Processed outputs
// ---------------------------------------------------------------------------------------

/// Axis-aligned obstacle cluster extracted from a LiDAR sweep.
struct Obstacle {
    Vec3 centroid;
    Vec3 min;
    Vec3 max;
    std::uint32_t num_points{0};
    float range_m{0.f};
};

struct LidarResult {
    std::uint64_t seq{0};
    Nanos sensor_ts_ns{0};
    Nanos ingest_ns{0};
    Nanos processed_ns{0};
    std::uint32_t raw_points{0};
    std::uint32_t filtered_points{0};
    std::uint32_t ground_points{0};
    float nearest_obstacle_m{-1.f};  // -1 when no obstacles
    std::vector<Obstacle> obstacles;
};

/// Orientation estimate from the IMU filter.
struct ImuState {
    std::uint64_t seq{0};
    Nanos sensor_ts_ns{0};
    Nanos ingest_ns{0};
    float roll_rad{0.f};
    float pitch_rad{0.f};
    float yaw_rad{0.f};
    float yaw_rate_rps{0.f};
    float long_accel_mps2{0.f};
    float lat_accel_mps2{0.f};
    float accel_norm_mps2{0.f};
};

/// Decoded vehicle bus state (latest value of each signal seen on the bus).
struct VehicleState {
    std::uint64_t seq{0};
    Nanos sensor_ts_ns{0};
    Nanos ingest_ns{0};
    float speed_mps{0.f};
    float steering_deg{0.f};
    float engine_rpm{0.f};
    float brake_bar{0.f};
    float throttle_pct{0.f};
    std::uint8_t gear{0};
    std::uint32_t decode_errors{0};
};

/// One output record per LiDAR frame, joined with the most recent IMU and CAN state.
struct FusedFrame {
    std::uint64_t seq{0};
    Nanos sensor_ts_ns{0};
    Nanos ingest_ns{0};
    Nanos fused_ns{0};
    LidarResult lidar;
    ImuState imu;
    VehicleState vehicle;
    bool imu_valid{false};
    bool vehicle_valid{false};
    float time_to_collision_s{-1.f};  // -1 when not applicable
};

}  // namespace axle
