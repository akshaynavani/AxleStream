#include <stdexcept>

#include "axle/log_format.hpp"

namespace axle {

namespace {
void put_le(std::ofstream& out, std::uint64_t v, int bytes) {
    char b[8];
    for (int i = 0; i < bytes; ++i) b[i] = static_cast<char>((v >> (8 * i)) & 0xFF);
    out.write(b, bytes);
}
}  // namespace

LogWriter::LogWriter(const std::string& path) : out_(path, std::ios::binary | std::ios::trunc) {
    if (!out_) throw std::runtime_error("cannot open log for writing: " + path);
    out_.write(logfmt::kMagic, 4);
    put_le(out_, logfmt::kVersion, 2);
    put_le(out_, 0, 2);
    put_le(out_, static_cast<std::uint64_t>(wall_ns()), 8);
    bytes_ = logfmt::kFileHeaderSize;
}

LogWriter::~LogWriter() { flush(); }

void LogWriter::write_record(SensorKind kind, const std::vector<std::uint8_t>& payload) {
    std::lock_guard<std::mutex> lock(mu_);
    put_le(out_, static_cast<std::uint8_t>(kind), 1);
    put_le(out_, 0, 3);
    put_le(out_, static_cast<std::uint32_t>(payload.size()), 4);
    out_.write(reinterpret_cast<const char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
    ++records_;
    bytes_ += logfmt::kRecordHeaderSize + payload.size();
}

// Encoding happens outside the lock so concurrent writers only serialise on the file write.
void LogWriter::write(const LidarFrame& f) { write_record(SensorKind::Lidar, logfmt::encode(f)); }
void LogWriter::write(const ImuSample& s) { write_record(SensorKind::Imu, logfmt::encode(s)); }
void LogWriter::write(const CanFrame& c) { write_record(SensorKind::Can, logfmt::encode(c)); }

void LogWriter::flush() {
    std::lock_guard<std::mutex> lock(mu_);
    out_.flush();
}

SensorSinks tee_to_log(const SensorSinks& downstream, LogWriter& writer) {
    SensorSinks s;
    s.lidar = [down = downstream.lidar, &writer](LidarFrame&& f) {
        writer.write(f);
        return down ? down(std::move(f)) : true;
    };
    s.imu = [down = downstream.imu, &writer](ImuSample&& m) {
        writer.write(m);
        return down ? down(std::move(m)) : true;
    };
    s.can = [down = downstream.can, &writer](CanFrame&& c) {
        writer.write(c);
        return down ? down(std::move(c)) : true;
    };
    return s;
}

}  // namespace axle
