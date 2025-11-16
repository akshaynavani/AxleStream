#include <cstring>
#include <stdexcept>
#include <type_traits>

#include "axle/log_format.hpp"

namespace axle {

namespace {
std::uint64_t get_le(const unsigned char* b, int bytes) {
    std::uint64_t v = 0;
    for (int i = 0; i < bytes; ++i) v |= static_cast<std::uint64_t>(b[i]) << (8 * i);
    return v;
}
}  // namespace

LogReader::LogReader(const std::string& path) : in_(path, std::ios::binary) {
    if (!in_) throw std::runtime_error("cannot open log: " + path);
    unsigned char hdr[logfmt::kFileHeaderSize];
    in_.read(reinterpret_cast<char*>(hdr), sizeof hdr);
    if (in_.gcount() != static_cast<std::streamsize>(sizeof hdr) || std::memcmp(hdr, logfmt::kMagic, 4) != 0) {
        throw std::runtime_error("not an AxleStream log: " + path);
    }
    const auto version = static_cast<std::uint16_t>(get_le(hdr + 4, 2));
    if (version != logfmt::kVersion) {
        throw std::runtime_error("unsupported log version " + std::to_string(version));
    }
    created_wall_ns_ = static_cast<Nanos>(get_le(hdr + 8, 8));
}

std::optional<LogRecord> LogReader::next() {
    unsigned char hdr[logfmt::kRecordHeaderSize];
    in_.read(reinterpret_cast<char*>(hdr), sizeof hdr);
    if (in_.gcount() == 0) return std::nullopt;  // clean EOF
    if (in_.gcount() != static_cast<std::streamsize>(sizeof hdr)) {
        truncated_ = true;
        return std::nullopt;
    }
    const auto kind = static_cast<SensorKind>(hdr[0]);
    const auto len = static_cast<std::uint32_t>(get_le(hdr + 4, 4));
    if (len > logfmt::kMaxPayload) {
        truncated_ = true;
        return std::nullopt;
    }
    buf_.resize(len);
    in_.read(reinterpret_cast<char*>(buf_.data()), len);
    if (in_.gcount() != static_cast<std::streamsize>(len)) {
        truncated_ = true;
        return std::nullopt;
    }

    switch (kind) {
        case SensorKind::Lidar:
            if (auto f = logfmt::decode_lidar(buf_.data(), len)) return LogRecord{std::move(*f)};
            break;
        case SensorKind::Imu:
            if (auto s = logfmt::decode_imu(buf_.data(), len)) return LogRecord{*s};
            break;
        case SensorKind::Can:
            if (auto c = logfmt::decode_can(buf_.data(), len)) return LogRecord{*c};
            break;
    }
    truncated_ = true;  // unknown kind or malformed payload
    return std::nullopt;
}

// ---------------------------------------------------------------------------------------

ReplaySource::ReplaySource(std::string path, double speed, bool loop)
    : path_(std::move(path)), speed_(speed), loop_(loop) {}

void ReplaySource::run(const std::atomic<bool>& stop, const SensorSinks& sinks) {
    do {
        LogReader reader(path_);
        Nanos first_ts = -1;
        const Nanos start_mono = now_ns();

        while (!stop.load(std::memory_order_relaxed)) {
            auto rec = reader.next();
            if (!rec) break;

            const Nanos ts = std::visit([](const auto& m) { return m.sensor_ts_ns; }, *rec);
            if (first_ts < 0) first_ts = ts;
            if (speed_ > 0) {
                const auto offset = static_cast<Nanos>(static_cast<double>(ts - first_ts) / speed_);
                sleep_until_ns(start_mono + offset);
            }

            std::visit(
                [&](auto& m) {
                    m.ingest_ns = now_ns();
                    using T = std::decay_t<decltype(m)>;
                    if constexpr (std::is_same_v<T, LidarFrame>) {
                        emit(sinks.lidar, std::move(m));
                    } else if constexpr (std::is_same_v<T, ImuSample>) {
                        emit(sinks.imu, std::move(m));
                    } else {
                        emit(sinks.can, std::move(m));
                    }
                },
                *rec);
        }
    } while (loop_ && !stop.load(std::memory_order_relaxed));
}

}  // namespace axle
