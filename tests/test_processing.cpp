#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "axle/can_db.hpp"
#include "axle/can_decoder.hpp"
#include "axle/fusion.hpp"
#include "axle/imu_filter.hpp"
#include "axle/lidar_processor.hpp"
#include "axle/sim_sources.hpp"

using namespace axle;

namespace {

/// Flat ground at z = -1.8 plus one box obstacle centred at (cx, cy).
LidarFrame synthetic_scene(float cx, float cy) {
    LidarFrame f;
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> u(-40.f, 40.f);
    for (int i = 0; i < 20000; ++i) {
        f.points.push_back({u(rng), u(rng), -1.8f, 20.f});
    }
    for (float x = cx - 1.f; x <= cx + 1.f; x += 0.1f) {
        for (float y = cy - 0.5f; y <= cy + 0.5f; y += 0.1f) {
            for (float z = -1.4f; z <= 0.f; z += 0.2f) f.points.push_back({x, y, z, 150.f});
        }
    }
    return f;
}

ImuSample imu_at(Nanos ts, float ax, float ay, float az, float gz) {
    ImuSample s;
    s.sensor_ts_ns = ts;
    s.accel = {ax, ay, az};
    s.gyro = {0.f, 0.f, gz};
    return s;
}

}  // namespace

// ---- LiDAR ------------------------------------------------------------------------------

TEST(LidarProcessor, SeparatesGroundAndFindsObstacle) {
    LidarProcessor proc;
    const auto r = proc.process(synthetic_scene(12.f, 0.f));
    EXPECT_GT(r.ground_points, 1000u);
    EXPECT_LT(r.filtered_points, r.raw_points);
    ASSERT_EQ(r.obstacles.size(), 1u);
    const auto& ob = r.obstacles.front();
    EXPECT_NEAR(ob.centroid.x, 12.f, 0.3f);
    EXPECT_NEAR(ob.centroid.y, 0.f, 0.3f);
    EXPECT_NEAR(r.nearest_obstacle_m, 11.f, 0.3f);
}

TEST(LidarProcessor, RangeCropDropsEgoAndFarPoints) {
    LidarProcessor proc;
    LidarFrame f;
    f.points.push_back({0.5f, 0.2f, 0.f, 1.f});    // ego body
    f.points.push_back({200.f, 0.f, 0.f, 1.f});    // beyond max range
    f.points.push_back({10.f, 0.f, 5.f, 1.f});     // overhead
    const auto r = proc.process(f);
    EXPECT_EQ(r.filtered_points, 0u);
    EXPECT_TRUE(r.obstacles.empty());
    EXPECT_FLOAT_EQ(r.nearest_obstacle_m, -1.f);
}

TEST(LidarProcessor, ReusableAcrossFrames) {
    LidarProcessor proc;
    const auto a = proc.process(synthetic_scene(12.f, 0.f));
    const auto b = proc.process(synthetic_scene(-20.f, 8.f));
    ASSERT_EQ(b.obstacles.size(), 1u);
    EXPECT_NEAR(b.obstacles[0].centroid.x, -20.f, 0.3f);
    EXPECT_EQ(a.obstacles.size(), 1u);
}

TEST(LidarProcessor, HandlesSimulatedSweep) {
    auto model = std::make_shared<const VehicleModel>();
    SimConfig cfg;
    SimLidarSource sim(model, cfg);
    LidarProcessor proc;
    for (int i = 0; i < 20; ++i) {
        const auto r = proc.process(sim.generate(5.0 + i, static_cast<std::uint64_t>(i)));
        EXPECT_GT(r.ground_points, 0u);
        EXPECT_LE(r.ground_points, r.filtered_points);
    }
}

// ---- IMU --------------------------------------------------------------------------------

TEST(ImuFilter, LevelAtRest) {
    ImuFilter f;
    ImuState s;
    for (int i = 0; i < 400; ++i) s = f.update(imu_at(i * 5'000'000LL, 0.f, 0.f, 9.80665f, 0.f));
    EXPECT_NEAR(s.roll_rad, 0.f, 1e-3);
    EXPECT_NEAR(s.pitch_rad, 0.f, 1e-3);
    EXPECT_NEAR(s.yaw_rad, 0.f, 1e-3);
}

TEST(ImuFilter, ConvergesToStaticTilt) {
    ImuFilter f;
    const float tilt = 0.1f;  // nose-up pitch
    const float ax = -9.80665f * std::sin(tilt), az = 9.80665f * std::cos(tilt);
    ImuState s;
    for (int i = 0; i < 2000; ++i) s = f.update(imu_at(i * 5'000'000LL, ax, 0.f, az, 0.f));
    EXPECT_NEAR(s.pitch_rad, tilt, 0.01);
    EXPECT_NEAR(s.long_accel_mps2, 0.f, 0.1);
}

TEST(ImuFilter, IntegratesYawRate) {
    ImuFilter::Config cfg;
    cfg.still_samples = 1'000'000;  // disable bias learning for this test
    ImuFilter f(cfg);
    ImuState s;
    for (int i = 0; i <= 200; ++i) s = f.update(imu_at(i * 5'000'000LL, 0.f, 0.f, 9.80665f, 0.5f));
    EXPECT_NEAR(s.yaw_rad, 0.5f, 0.01f);  // 0.5 rad/s for 1 s
}

TEST(ImuFilter, LearnsGyroBiasWhenStill) {
    ImuFilter f;
    for (int i = 0; i < 2000; ++i) f.update(imu_at(i * 5'000'000LL, 0.f, 0.f, 9.80665f, 0.004f));
    EXPECT_NEAR(f.gyro_bias_z(), 0.004f, 0.0005f);
}

TEST(ImuFilter, AccelerationDoesNotMasqueradeAsPitch) {
    ImuFilter f;
    ImuState s;
    int i = 0;
    for (; i < 100; ++i) f.update(imu_at(i * 5'000'000LL, 0.f, 0.f, 9.80665f, 0.f));
    // 4 m/s^2 of forward acceleration pushes |f| ~0.8 m/s^2 above 1 g, past the gate, so
    // the accelerometer correction is suppressed and pitch stays where it was.
    for (; i < 500; ++i) s = f.update(imu_at(i * 5'000'000LL, 4.0f, 0.f, 9.80665f, 0.f));
    EXPECT_NEAR(s.pitch_rad, 0.f, 0.02f);
    EXPECT_NEAR(s.long_accel_mps2, 4.0f, 0.2f);
}

// ---- CAN --------------------------------------------------------------------------------

TEST(CanDecoder, DecodesAllSignals) {
    CanDecoder d;
    d.on_frame(can_db::encode_wheel_speed(12.5, 0));
    d.on_frame(can_db::encode_steering(-15.0, 0));
    d.on_frame(can_db::encode_engine(2100.0, 25.0, 0));
    d.on_frame(can_db::encode_brake(3.5, false, 0));
    EXPECT_FALSE(d.complete());
    d.on_frame(can_db::encode_gear(3, 0));
    EXPECT_TRUE(d.complete());
    const auto& s = d.state();
    EXPECT_NEAR(s.speed_mps, 12.5f, 0.01f);
    EXPECT_NEAR(s.steering_deg, -15.f, 0.05f);
    EXPECT_NEAR(s.engine_rpm, 2100.f, 0.3f);
    EXPECT_NEAR(s.throttle_pct, 25.f, 0.5f);
    EXPECT_NEAR(s.brake_bar, 3.5f, 0.01f);
    EXPECT_EQ(s.gear, 3);
    EXPECT_EQ(d.stats().decoded, 5u);
}

TEST(CanDecoder, RejectsBadChecksumAndUnknownIds) {
    CanDecoder d;
    auto f = can_db::encode_wheel_speed(10.0, 0);
    f.data[0] ^= 0xFF;
    EXPECT_FALSE(d.on_frame(f));
    EXPECT_EQ(d.stats().bad_checksum, 1u);
    EXPECT_EQ(d.state().decode_errors, 1u);

    CanFrame unknown;
    unknown.id = 0x7FF;
    unknown.dlc = 8;
    EXPECT_FALSE(d.on_frame(unknown));
    EXPECT_EQ(d.stats().unknown_id, 1u);

    auto short_frame = can_db::encode_gear(2, 0);
    short_frame.dlc = 4;
    EXPECT_FALSE(d.on_frame(short_frame));
    EXPECT_EQ(d.stats().bad_dlc, 1u);
}

TEST(CanDecoder, CountsCounterJumpsWithWraparound) {
    CanDecoder d;
    for (int c = 0; c < 20; ++c) d.on_frame(can_db::encode_wheel_speed(5.0, static_cast<std::uint8_t>(c & 0x0F)));
    EXPECT_EQ(d.stats().counter_jumps, 0u);
    d.on_frame(can_db::encode_wheel_speed(5.0, 9));  // expected 4
    EXPECT_EQ(d.stats().counter_jumps, 1u);
}

// ---- Fusion -----------------------------------------------------------------------------

TEST(Fusion, PicksClosestStateInTime) {
    Fusion fu;
    for (int i = 0; i < 50; ++i) {
        ImuState s;
        s.sensor_ts_ns = i * 5'000'000LL;
        s.seq = static_cast<std::uint64_t>(i);
        fu.on_imu(s);
    }
    VehicleState v;
    v.sensor_ts_ns = 100'000'000LL;
    v.speed_mps = 10.f;
    fu.on_vehicle(v);

    LidarResult r;
    r.sensor_ts_ns = 101'000'000LL;  // closest IMU sample is seq 20 (100 ms)
    const auto out = fu.fuse(std::move(r));
    ASSERT_TRUE(out.imu_valid);
    EXPECT_EQ(out.imu.seq, 20u);
    EXPECT_TRUE(out.vehicle_valid);
}

TEST(Fusion, MarksStaleStateInvalid) {
    Fusion fu;
    ImuState s;
    s.sensor_ts_ns = 0;
    fu.on_imu(s);
    LidarResult r;
    r.sensor_ts_ns = ms_to_ns(500);
    const auto out = fu.fuse(std::move(r));
    EXPECT_FALSE(out.imu_valid);
    EXPECT_FALSE(out.vehicle_valid);
    EXPECT_FLOAT_EQ(out.time_to_collision_s, -1.f);
}

TEST(Fusion, ComputesTimeToCollisionForCorridorObstacle) {
    Fusion fu;
    VehicleState v;
    v.sensor_ts_ns = 0;
    v.speed_mps = 10.f;
    fu.on_vehicle(v);

    LidarResult r;
    Obstacle in_lane;
    in_lane.min = {20.f, -0.5f, -1.f};
    in_lane.max = {22.f, 0.5f, 0.f};
    Obstacle beside;
    beside.min = {5.f, 3.f, -1.f};
    beside.max = {7.f, 4.f, 0.f};
    r.obstacles = {beside, in_lane};
    const auto out = fu.fuse(std::move(r));
    EXPECT_NEAR(out.time_to_collision_s, 2.0f, 1e-4);
}
