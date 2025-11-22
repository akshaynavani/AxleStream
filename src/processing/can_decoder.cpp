#include "axle/can_decoder.hpp"

#include "axle/can_db.hpp"

namespace axle {

int CanDecoder::slot_for(std::uint32_t id) const noexcept {
    switch (id) {
        case can_db::kWheelSpeedId: return 0;
        case can_db::kSteeringId: return 1;
        case can_db::kEngineId: return 2;
        case can_db::kBrakeId: return 3;
        case can_db::kGearId: return 4;
        default: return -1;
    }
}

bool CanDecoder::on_frame(const CanFrame& f) {
    ++stats_.frames;
    const int slot = f.extended ? -1 : slot_for(f.id);
    if (slot < 0) {
        ++stats_.unknown_id;
        return false;
    }
    if (f.dlc != 8) {
        ++stats_.bad_dlc;
        ++state_.decode_errors;
        return false;
    }
    if (!can_db::checksum_ok(f)) {
        ++stats_.bad_checksum;
        ++state_.decode_errors;
        return false;
    }

    const int counter = can_db::counter_of(f);
    auto& last = last_counter_[static_cast<std::size_t>(slot)];
    if (last >= 0 && counter != ((last + 1) & 0x0F)) {
        ++stats_.counter_jumps;
    }
    last = counter;

    switch (slot) {
        case 0:
            state_.speed_mps = static_cast<float>(can_db::decode_speed_mps(f));
            break;
        case 1:
            state_.steering_deg = static_cast<float>(can_db::decode_steering_deg(f));
            break;
        case 2:
            state_.engine_rpm = static_cast<float>(can_db::decode_rpm(f));
            state_.throttle_pct = static_cast<float>(can_db::decode_throttle_pct(f));
            break;
        case 3:
            state_.brake_bar = static_cast<float>(can_db::decode_brake_bar(f));
            break;
        case 4:
            state_.gear = static_cast<std::uint8_t>(can_db::decode_gear(f));
            break;
        default:
            break;
    }
    seen_mask_ = static_cast<std::uint8_t>(seen_mask_ | (1u << slot));
    state_.seq = f.seq;
    state_.sensor_ts_ns = f.sensor_ts_ns;
    state_.ingest_ns = f.ingest_ns;
    ++stats_.decoded;
    return true;
}

}  // namespace axle
