#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include "axle/types.hpp"

/// Minimal DBC-equivalent signal database for the simulated vehicle bus.
///
/// Every message carries a 4-bit rolling counter in byte 6 (low nibble) and an additive
/// checksum in byte 7 (sum of bytes 0..6, mod 256), mirroring common OEM conventions.
/// Multi-byte signals are little-endian (Intel byte order).
namespace axle::can_db {

constexpr std::uint32_t kWheelSpeedId = 0x0B4;  // 100 Hz
constexpr std::uint32_t kSteeringId = 0x025;    // 100 Hz
constexpr std::uint32_t kEngineId = 0x1C4;      // 50 Hz
constexpr std::uint32_t kBrakeId = 0x224;       // 50 Hz
constexpr std::uint32_t kGearId = 0x3BC;        // 10 Hz

// Signal scaling factors
constexpr double kSpeedScaleKph = 0.01;   // u16
constexpr double kSteerScaleDeg = 0.1;    // i16
constexpr double kRpmScale = 0.25;        // u16
constexpr double kThrottleScale = 0.5;    // u8, percent
constexpr double kBrakeScaleBar = 0.01;   // u16

inline std::uint8_t checksum(const CanFrame& f) noexcept {
    unsigned sum = 0;
    for (int i = 0; i < 7; ++i) sum += f.data[static_cast<std::size_t>(i)];
    return static_cast<std::uint8_t>(sum & 0xFF);
}

inline void seal(CanFrame& f, std::uint8_t counter) noexcept {
    f.dlc = 8;
    f.data[6] = static_cast<std::uint8_t>(counter & 0x0F);
    f.data[7] = checksum(f);
}

inline bool checksum_ok(const CanFrame& f) noexcept { return f.dlc == 8 && f.data[7] == checksum(f); }
inline std::uint8_t counter_of(const CanFrame& f) noexcept { return f.data[6] & 0x0F; }

inline void put_u16(CanFrame& f, int byte, std::uint16_t v) noexcept {
    f.data[static_cast<std::size_t>(byte)] = static_cast<std::uint8_t>(v & 0xFF);
    f.data[static_cast<std::size_t>(byte + 1)] = static_cast<std::uint8_t>(v >> 8);
}
inline std::uint16_t get_u16(const CanFrame& f, int byte) noexcept {
    return static_cast<std::uint16_t>(f.data[static_cast<std::size_t>(byte)] |
                                      (f.data[static_cast<std::size_t>(byte + 1)] << 8));
}

template <typename T>
inline T clamp_round(double v, double lo, double hi) noexcept {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return static_cast<T>(std::lround(v));
}

// ---- Encoders (used by the simulator) -------------------------------------------------

inline CanFrame encode_wheel_speed(double speed_mps, std::uint8_t counter) {
    CanFrame f;
    f.id = kWheelSpeedId;
    put_u16(f, 0, clamp_round<std::uint16_t>(speed_mps * 3.6 / kSpeedScaleKph, 0, 65535));
    seal(f, counter);
    return f;
}

inline CanFrame encode_steering(double wheel_deg, std::uint8_t counter) {
    CanFrame f;
    f.id = kSteeringId;
    const auto raw = clamp_round<std::int16_t>(wheel_deg / kSteerScaleDeg, -32768, 32767);
    put_u16(f, 0, static_cast<std::uint16_t>(raw));
    seal(f, counter);
    return f;
}

inline CanFrame encode_engine(double rpm, double throttle_pct, std::uint8_t counter) {
    CanFrame f;
    f.id = kEngineId;
    put_u16(f, 0, clamp_round<std::uint16_t>(rpm / kRpmScale, 0, 65535));
    f.data[2] = clamp_round<std::uint8_t>(throttle_pct / kThrottleScale, 0, 200);
    seal(f, counter);
    return f;
}

inline CanFrame encode_brake(double pressure_bar, bool pedal_pressed, std::uint8_t counter) {
    CanFrame f;
    f.id = kBrakeId;
    put_u16(f, 0, clamp_round<std::uint16_t>(pressure_bar / kBrakeScaleBar, 0, 65535));
    f.data[2] = pedal_pressed ? 1 : 0;
    seal(f, counter);
    return f;
}

inline CanFrame encode_gear(int gear, std::uint8_t counter) {
    CanFrame f;
    f.id = kGearId;
    f.data[0] = static_cast<std::uint8_t>(gear < 0 ? 0 : gear > 15 ? 15 : gear);
    seal(f, counter);
    return f;
}

// ---- Decoders (physical values; no validation here, see CanDecoder) --------------------

inline double decode_speed_mps(const CanFrame& f) { return get_u16(f, 0) * kSpeedScaleKph / 3.6; }
inline double decode_steering_deg(const CanFrame& f) {
    return static_cast<std::int16_t>(get_u16(f, 0)) * kSteerScaleDeg;
}
inline double decode_rpm(const CanFrame& f) { return get_u16(f, 0) * kRpmScale; }
inline double decode_throttle_pct(const CanFrame& f) { return f.data[2] * kThrottleScale; }
inline double decode_brake_bar(const CanFrame& f) { return get_u16(f, 0) * kBrakeScaleBar; }
inline int decode_gear(const CanFrame& f) { return f.data[0]; }

}  // namespace axle::can_db
