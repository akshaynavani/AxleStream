#pragma once

#include <cstdint>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "axle/source.hpp"
#include "axle/types.hpp"

namespace axle {

/// `.axl` recording format (all integers little-endian, floats IEEE-754 binary32):
///
///   File header (16 bytes):  "AXLS" | u16 version | u16 reserved | i64 created_wall_ns
///   Record header (8 bytes): u8 kind | u8[3] reserved | u32 payload_len
///   Payload by kind:
///     Lidar: u64 seq | i64 sensor_ts | u32 n | n x (f32 x, f32 y, f32 z, f32 intensity)
///     Imu:   u64 seq | i64 sensor_ts | f32 ax ay az gx gy gz
///     Can:   u64 seq | i64 sensor_ts | u32 id | u8 dlc | u8 extended | u8[8] data
///
/// Records are appended in arrival order, so replay reproduces cross-sensor interleaving.
namespace logfmt {
constexpr char kMagic[4] = {'A', 'X', 'L', 'S'};
constexpr std::uint16_t kVersion = 1;
constexpr std::size_t kFileHeaderSize = 16;
constexpr std::size_t kRecordHeaderSize = 8;
constexpr std::uint32_t kMaxPayload = 64u * 1024u * 1024u;  // sanity bound for corrupt files

std::vector<std::uint8_t> encode(const LidarFrame& f);
std::vector<std::uint8_t> encode(const ImuSample& s);
std::vector<std::uint8_t> encode(const CanFrame& c);

std::optional<LidarFrame> decode_lidar(const std::uint8_t* p, std::size_t n);
std::optional<ImuSample> decode_imu(const std::uint8_t* p, std::size_t n);
std::optional<CanFrame> decode_can(const std::uint8_t* p, std::size_t n);
}  // namespace logfmt

using LogRecord = std::variant<LidarFrame, ImuSample, CanFrame>;

/// Thread-safe appender; multiple source threads may record concurrently.
class LogWriter {
public:
    explicit LogWriter(const std::string& path);
    ~LogWriter();

    void write(const LidarFrame& f);
    void write(const ImuSample& s);
    void write(const CanFrame& c);
    void flush();

    std::uint64_t records() const noexcept { return records_; }
    std::uint64_t bytes() const noexcept { return bytes_; }

private:
    void write_record(SensorKind kind, const std::vector<std::uint8_t>& payload);

    std::mutex mu_;
    std::ofstream out_;
    std::uint64_t records_{0};
    std::uint64_t bytes_{0};
};

/// Sequential reader. Stops (returns nullopt) at EOF or on the first malformed record.
class LogReader {
public:
    explicit LogReader(const std::string& path);

    std::optional<LogRecord> next();
    Nanos created_wall_ns() const noexcept { return created_wall_ns_; }
    bool truncated() const noexcept { return truncated_; }

private:
    std::ifstream in_;
    Nanos created_wall_ns_{0};
    bool truncated_{false};
    std::vector<std::uint8_t> buf_;
};

/// Replays a recording into the pipeline. With `speed > 0` inter-message gaps are reproduced
/// (scaled by 1/speed); with `speed == 0` messages are pushed as fast as possible. Ingest
/// timestamps are re-stamped at replay time so latency figures stay meaningful.
class ReplaySource final : public Source {
public:
    ReplaySource(std::string path, double speed = 1.0, bool loop = false);
    std::string name() const override { return "replay:" + path_; }
    void run(const std::atomic<bool>& stop, const SensorSinks& sinks) override;

private:
    std::string path_;
    double speed_;
    bool loop_;
};

/// Wrap sinks so every message is also appended to `writer` before being forwarded.
SensorSinks tee_to_log(const SensorSinks& downstream, LogWriter& writer);

}  // namespace axle
