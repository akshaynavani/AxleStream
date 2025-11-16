#include <cstring>

#include "axle/log_format.hpp"

namespace axle::logfmt {

namespace {

class ByteWriter {
public:
    explicit ByteWriter(std::size_t reserve) { buf_.reserve(reserve); }

    void u8(std::uint8_t v) { buf_.push_back(v); }
    void u32(std::uint32_t v) {
        for (int i = 0; i < 4; ++i) buf_.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
    void u64(std::uint64_t v) {
        for (int i = 0; i < 8; ++i) buf_.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
    void i64(std::int64_t v) { u64(static_cast<std::uint64_t>(v)); }
    void f32(float v) {
        std::uint32_t bits;
        std::memcpy(&bits, &v, sizeof bits);
        u32(bits);
    }
    std::vector<std::uint8_t> take() { return std::move(buf_); }

private:
    std::vector<std::uint8_t> buf_;
};

class ByteReader {
public:
    ByteReader(const std::uint8_t* p, std::size_t n) : p_(p), n_(n) {}

    bool ok() const { return ok_; }
    std::size_t remaining() const { return n_ - pos_; }

    std::uint8_t u8() {
        if (!need(1)) return 0;
        return p_[pos_++];
    }
    std::uint32_t u32() {
        if (!need(4)) return 0;
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(p_[pos_++]) << (8 * i);
        return v;
    }
    std::uint64_t u64() {
        if (!need(8)) return 0;
        std::uint64_t v = 0;
        for (int i = 0; i < 8; ++i) v |= static_cast<std::uint64_t>(p_[pos_++]) << (8 * i);
        return v;
    }
    std::int64_t i64() { return static_cast<std::int64_t>(u64()); }
    float f32() {
        const std::uint32_t bits = u32();
        float v;
        std::memcpy(&v, &bits, sizeof v);
        return v;
    }

private:
    bool need(std::size_t k) {
        if (!ok_ || n_ - pos_ < k) {
            ok_ = false;
            return false;
        }
        return true;
    }

    const std::uint8_t* p_;
    std::size_t n_;
    std::size_t pos_{0};
    bool ok_{true};
};

}  // namespace

std::vector<std::uint8_t> encode(const LidarFrame& f) {
    ByteWriter w(20 + f.points.size() * 16);
    w.u64(f.seq);
    w.i64(f.sensor_ts_ns);
    w.u32(static_cast<std::uint32_t>(f.points.size()));
    for (const auto& p : f.points) {
        w.f32(p.x);
        w.f32(p.y);
        w.f32(p.z);
        w.f32(p.intensity);
    }
    return w.take();
}

std::vector<std::uint8_t> encode(const ImuSample& s) {
    ByteWriter w(40);
    w.u64(s.seq);
    w.i64(s.sensor_ts_ns);
    w.f32(s.accel.x);
    w.f32(s.accel.y);
    w.f32(s.accel.z);
    w.f32(s.gyro.x);
    w.f32(s.gyro.y);
    w.f32(s.gyro.z);
    return w.take();
}

std::vector<std::uint8_t> encode(const CanFrame& c) {
    ByteWriter w(30);
    w.u64(c.seq);
    w.i64(c.sensor_ts_ns);
    w.u32(c.id);
    w.u8(c.dlc);
    w.u8(c.extended ? 1 : 0);
    for (auto b : c.data) w.u8(b);
    return w.take();
}

std::optional<LidarFrame> decode_lidar(const std::uint8_t* p, std::size_t n) {
    ByteReader r(p, n);
    LidarFrame f;
    f.seq = r.u64();
    f.sensor_ts_ns = r.i64();
    const std::uint32_t count = r.u32();
    if (!r.ok() || r.remaining() != static_cast<std::size_t>(count) * 16) return std::nullopt;
    f.points.resize(count);
    for (auto& pt : f.points) {
        pt.x = r.f32();
        pt.y = r.f32();
        pt.z = r.f32();
        pt.intensity = r.f32();
    }
    if (!r.ok()) return std::nullopt;
    return f;
}

std::optional<ImuSample> decode_imu(const std::uint8_t* p, std::size_t n) {
    ByteReader r(p, n);
    ImuSample s;
    s.seq = r.u64();
    s.sensor_ts_ns = r.i64();
    s.accel.x = r.f32();
    s.accel.y = r.f32();
    s.accel.z = r.f32();
    s.gyro.x = r.f32();
    s.gyro.y = r.f32();
    s.gyro.z = r.f32();
    if (!r.ok() || r.remaining() != 0) return std::nullopt;
    return s;
}

std::optional<CanFrame> decode_can(const std::uint8_t* p, std::size_t n) {
    ByteReader r(p, n);
    CanFrame c;
    c.seq = r.u64();
    c.sensor_ts_ns = r.i64();
    c.id = r.u32();
    c.dlc = r.u8();
    c.extended = r.u8() != 0;
    for (auto& b : c.data) b = r.u8();
    if (!r.ok() || r.remaining() != 0 || c.dlc > 8) return std::nullopt;
    return c;
}

}  // namespace axle::logfmt
