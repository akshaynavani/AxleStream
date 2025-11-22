#pragma once

#include <array>
#include <cstdint>

#include "axle/types.hpp"

namespace axle {

/// Validates and decodes vehicle bus frames into a running VehicleState.
///
/// Validation per frame: DLC must be 8, checksum must match, and the 4-bit rolling counter
/// must advance by exactly one from the previous frame with the same ID (a skipped counter
/// indicates lost frames; a repeated one indicates a stuck transmitter). Frames failing the
/// checksum are rejected; counter discontinuities are counted but the frame is still used.
class CanDecoder {
public:
    struct Stats {
        std::uint64_t frames{0};
        std::uint64_t decoded{0};
        std::uint64_t unknown_id{0};
        std::uint64_t bad_dlc{0};
        std::uint64_t bad_checksum{0};
        std::uint64_t counter_jumps{0};
    };

    /// Returns true if `state()` changed as a result of this frame.
    bool on_frame(const CanFrame& f);

    const VehicleState& state() const noexcept { return state_; }
    const Stats& stats() const noexcept { return stats_; }

    /// True once every core signal (speed, steering, engine, brake, gear) has been seen.
    bool complete() const noexcept { return seen_mask_ == kAllSeen; }

private:
    static constexpr std::uint8_t kAllSeen = 0x1F;
    int slot_for(std::uint32_t id) const noexcept;

    VehicleState state_;
    Stats stats_;
    std::array<int, 5> last_counter_{{-1, -1, -1, -1, -1}};
    std::uint8_t seen_mask_{0};
};

}  // namespace axle
