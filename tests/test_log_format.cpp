#include <gtest/gtest.h>

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <vector>

#include "axle/log_format.hpp"

using namespace axle;

namespace {

std::string temp_path(const std::string& name) {
    return (std::filesystem::temp_directory_path() / ("axle_test_" + name)).string();
}

LidarFrame sample_lidar() {
    LidarFrame f;
    f.seq = 3;
    f.sensor_ts_ns = 1'700'000'000'000'000'000LL;
    for (int i = 0; i < 100; ++i) f.points.push_back({1.0f * i, -2.5f * i, 0.25f, 77.f});
    return f;
}

ImuSample sample_imu() {
    ImuSample s;
    s.seq = 9;
    s.sensor_ts_ns = 1'700'000'000'005'000'000LL;
    s.accel = {0.1f, -0.2f, 9.81f};
    s.gyro = {0.001f, 0.002f, 0.05f};
    return s;
}

CanFrame sample_can() {
    CanFrame c;
    c.seq = 11;
    c.sensor_ts_ns = 1'700'000'000'010'000'000LL;
    c.id = 0x1C4;
    c.dlc = 8;
    c.data = {1, 2, 3, 4, 5, 6, 7, 8};
    return c;
}

}  // namespace

TEST(LogCodec, LidarRoundTrip) {
    const auto f = sample_lidar();
    const auto bytes = logfmt::encode(f);
    auto back = logfmt::decode_lidar(bytes.data(), bytes.size());
    ASSERT_TRUE(back);
    EXPECT_EQ(back->seq, f.seq);
    EXPECT_EQ(back->sensor_ts_ns, f.sensor_ts_ns);
    ASSERT_EQ(back->points.size(), f.points.size());
    EXPECT_FLOAT_EQ(back->points[42].y, f.points[42].y);
}

TEST(LogCodec, RejectsTruncatedPayload) {
    auto bytes = logfmt::encode(sample_lidar());
    bytes.resize(bytes.size() - 3);
    EXPECT_FALSE(logfmt::decode_lidar(bytes.data(), bytes.size()));
    auto imu = logfmt::encode(sample_imu());
    imu.pop_back();
    EXPECT_FALSE(logfmt::decode_imu(imu.data(), imu.size()));
}

TEST(LogFile, WriteThenReadPreservesOrderAndContent) {
    const auto path = temp_path("roundtrip.axl");
    {
        LogWriter w(path);
        w.write(sample_lidar());
        w.write(sample_imu());
        w.write(sample_can());
        EXPECT_EQ(w.records(), 3u);
    }
    LogReader r(path);
    auto a = r.next();
    auto b = r.next();
    auto c = r.next();
    ASSERT_TRUE(a && b && c);
    EXPECT_TRUE(std::holds_alternative<LidarFrame>(*a));
    EXPECT_TRUE(std::holds_alternative<ImuSample>(*b));
    ASSERT_TRUE(std::holds_alternative<CanFrame>(*c));
    EXPECT_EQ(std::get<CanFrame>(*c).id, 0x1C4u);
    EXPECT_EQ(std::get<CanFrame>(*c).data[7], 8);
    EXPECT_FLOAT_EQ(std::get<ImuSample>(*b).accel.z, 9.81f);
    EXPECT_FALSE(r.next());
    EXPECT_FALSE(r.truncated());
    std::filesystem::remove(path);
}

TEST(LogFile, DetectsTruncatedTail) {
    const auto path = temp_path("truncated.axl");
    {
        LogWriter w(path);
        w.write(sample_imu());
        w.write(sample_lidar());
    }
    std::filesystem::resize_file(path, std::filesystem::file_size(path) - 10);
    LogReader r(path);
    EXPECT_TRUE(r.next());
    EXPECT_FALSE(r.next());
    EXPECT_TRUE(r.truncated());
    std::filesystem::remove(path);
}

TEST(LogFile, RejectsForeignFile) {
    const auto path = temp_path("foreign.bin");
    {
        std::ofstream out(path, std::ios::binary);
        out << "definitely not a log file";
    }
    EXPECT_THROW(LogReader r(path), std::runtime_error);
    std::filesystem::remove(path);
}

TEST(ReplaySource, ReplaysAllRecordsAsFastAsPossible) {
    const auto path = temp_path("replay.axl");
    {
        LogWriter w(path);
        for (int i = 0; i < 5; ++i) w.write(sample_imu());
        for (int i = 0; i < 3; ++i) w.write(sample_can());
        w.write(sample_lidar());
    }
    int lidar = 0, imu = 0, can = 0;
    SensorSinks sinks;
    sinks.lidar = [&](LidarFrame&& f) { ++lidar; EXPECT_GT(f.ingest_ns, 0); return true; };
    sinks.imu = [&](ImuSample&&) { ++imu; return true; };
    sinks.can = [&](CanFrame&&) { ++can; return true; };

    ReplaySource src(path, 0.0);
    std::atomic<bool> stop{false};
    src.run(stop, sinks);
    EXPECT_EQ(lidar, 1);
    EXPECT_EQ(imu, 5);
    EXPECT_EQ(can, 3);
    EXPECT_EQ(src.stats().produced.load(), 9u);
    EXPECT_EQ(src.stats().dropped.load(), 0u);
    std::filesystem::remove(path);
}

TEST(TeeToLog, RecordsAndForwards) {
    const auto path = temp_path("tee.axl");
    int forwarded = 0;
    {
        LogWriter w(path);
        SensorSinks down;
        down.imu = [&](ImuSample&&) { ++forwarded; return true; };
        auto tee = tee_to_log(down, w);
        tee.imu(sample_imu());
        tee.can(sample_can());  // no downstream: recorded only
    }
    EXPECT_EQ(forwarded, 1);
    LogReader r(path);
    int n = 0;
    while (r.next()) ++n;
    EXPECT_EQ(n, 2);
    std::filesystem::remove(path);
}
