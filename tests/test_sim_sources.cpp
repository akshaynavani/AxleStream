#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <vector>

#include "axle/can_db.hpp"
#include "axle/sim_sources.hpp"
#include "axle/vehicle_model.hpp"

using namespace axle;

TEST(VehicleModel, SpeedProfileCyclesThroughPhases) {
    VehicleModel m;
    EXPECT_NEAR(m.at(0.0).speed_mps, 0.0, 1e-6);
    EXPECT_NEAR(m.at(15.0).speed_mps, m.params().cruise_mps, 1e-6);  // cruising
    EXPECT_LT(m.at(31.0).long_accel_mps2, 0.0);                      // braking
    EXPECT_GT(m.at(31.0).brake_bar, 6.0);
    EXPECT_NEAR(m.at(36.0).speed_mps, 0.0, 1e-6);  // stopped
    EXPECT_EQ(m.at(36.0).gear, 0);
}

TEST(VehicleModel, IsDeterministicForSeed) {
    VehicleModel a, b;
    const auto ka = a.at(123.4), kb = b.at(123.4);
    EXPECT_DOUBLE_EQ(ka.x_m, kb.x_m);
    EXPECT_DOUBLE_EQ(ka.yaw_rad, kb.yaw_rad);
    ASSERT_EQ(a.obstacles().size(), b.obstacles().size());
    EXPECT_DOUBLE_EQ(a.obstacles()[5].cx, b.obstacles()[5].cx);
}

TEST(VehicleModel, VehicleActuallyMoves) {
    VehicleModel m;
    const auto k = m.at(30.0);
    EXPECT_GT(std::hypot(k.x_m, k.y_m), 100.0);
}

TEST(SimLidar, ProducesGroundAndObstacleReturns) {
    auto model = std::make_shared<const VehicleModel>();
    SimConfig cfg;
    cfg.lidar_dropout = 0.0;
    SimLidarSource lidar(model, cfg);
    const auto frame = lidar.generate(10.0, 0);

    // 900 azimuth steps x 16 beams; upward beams mostly miss, downward beams hit ground.
    EXPECT_GT(frame.points.size(), 5000u);
    EXPECT_LE(frame.points.size(), 900u * 16u);

    int ground = 0;
    for (const auto& p : frame.points) {
        const float r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        EXPECT_LE(r, cfg.lidar_max_range_m + 0.5f);
        if (std::abs(p.z + cfg.lidar_mount_height_m) < 0.15) ++ground;
    }
    EXPECT_GT(ground, 1000);
}

TEST(SimLidar, StopsAfterDuration) {
    SimConfig cfg;
    cfg.realtime = false;
    cfg.duration_s = 0.5;  // 5 frames at 10 Hz
    cfg.lidar_h_res_deg = 2.0;
    auto model = std::make_shared<const VehicleModel>();
    SimLidarSource lidar(model, cfg);
    int frames = 0;
    SensorSinks sinks;
    sinks.lidar = [&](LidarFrame&&) { ++frames; return true; };
    std::atomic<bool> stop{false};
    lidar.run(stop, sinks);
    EXPECT_EQ(frames, 5);
}

TEST(SimImu, ReportsGravityAndYawRate) {
    SimConfig cfg;
    cfg.realtime = false;
    cfg.duration_s = 1.0;
    auto model = std::make_shared<const VehicleModel>();
    SimImuSource imu(model, cfg);
    std::vector<ImuSample> samples;
    SensorSinks sinks;
    sinks.imu = [&](ImuSample&& s) { samples.push_back(s); return true; };
    std::atomic<bool> stop{false};
    imu.run(stop, sinks);
    ASSERT_EQ(samples.size(), 200u);
    double az = 0;
    for (const auto& s : samples) az += s.accel.z;
    EXPECT_NEAR(az / samples.size(), 9.80665, 0.05);
    EXPECT_GT(samples.back().sensor_ts_ns, samples.front().sensor_ts_ns);
}

TEST(SimCan, EmitsMessagesAtConfiguredRatesWithValidChecksums) {
    SimConfig cfg;
    cfg.realtime = false;
    cfg.duration_s = 1.0;
    auto model = std::make_shared<const VehicleModel>();
    SimCanSource can(model, cfg);
    int wheel = 0, steer = 0, engine = 0, brake = 0, gear = 0, bad = 0;
    SensorSinks sinks;
    sinks.can = [&](CanFrame&& f) {
        if (!can_db::checksum_ok(f)) ++bad;
        switch (f.id) {
            case can_db::kWheelSpeedId: ++wheel; break;
            case can_db::kSteeringId: ++steer; break;
            case can_db::kEngineId: ++engine; break;
            case can_db::kBrakeId: ++brake; break;
            case can_db::kGearId: ++gear; break;
        }
        return true;
    };
    std::atomic<bool> stop{false};
    can.run(stop, sinks);
    EXPECT_EQ(wheel, 100);
    EXPECT_EQ(steer, 100);
    EXPECT_EQ(engine, 50);
    EXPECT_EQ(brake, 50);
    EXPECT_EQ(gear, 10);
    EXPECT_EQ(bad, 0);
}

TEST(CanDb, EncodeDecodeRoundTrip) {
    EXPECT_NEAR(can_db::decode_speed_mps(can_db::encode_wheel_speed(13.7, 0)), 13.7, 0.01);
    EXPECT_NEAR(can_db::decode_steering_deg(can_db::encode_steering(-87.3, 0)), -87.3, 0.051);
    const auto eng = can_db::encode_engine(2450.0, 37.5, 3);
    EXPECT_NEAR(can_db::decode_rpm(eng), 2450.0, 0.25);
    EXPECT_NEAR(can_db::decode_throttle_pct(eng), 37.5, 0.5);
    EXPECT_EQ(can_db::counter_of(eng), 3);
    EXPECT_TRUE(can_db::checksum_ok(eng));
    EXPECT_EQ(can_db::decode_gear(can_db::encode_gear(4, 0)), 4);
}
