#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "axle/types.hpp"

namespace axle {

/// Destinations a source hands its raw messages to. Each callback returns false if the
/// message was dropped (e.g. downstream queue full) so sources can count drops.
/// Any callback may be empty, in which case messages of that kind are discarded.
struct SensorSinks {
    std::function<bool(LidarFrame&&)> lidar;
    std::function<bool(ImuSample&&)> imu;
    std::function<bool(CanFrame&&)> can;
};

/// Bitmask of sensor kinds a source produces. The pipeline uses it to guarantee that every
/// per-kind SPSC queue has exactly one producer thread.
enum KindMask : std::uint8_t {
    kLidarMask = 1u << 0,
    kImuMask = 1u << 1,
    kCanMask = 1u << 2,
    kAllKinds = kLidarMask | kImuMask | kCanMask,
};

struct SourceStats {
    std::atomic<std::uint64_t> produced{0};
    std::atomic<std::uint64_t> dropped{0};
};

/// A producer of raw sensor messages. `run` executes on a dedicated thread and must return
/// promptly once `stop` becomes true (or when the source is exhausted, e.g. end of a log).
class Source {
public:
    virtual ~Source() = default;
    virtual std::string name() const = 0;
    virtual void run(const std::atomic<bool>& stop, const SensorSinks& sinks) = 0;
    virtual std::uint8_t kinds() const { return kAllKinds; }

    const SourceStats& stats() const noexcept { return stats_; }

protected:
    template <typename Msg, typename Fn>
    void emit(const Fn& sink, Msg&& msg) {
        stats_.produced.fetch_add(1, std::memory_order_relaxed);
        if (!sink || !sink(std::forward<Msg>(msg))) {
            stats_.dropped.fetch_add(1, std::memory_order_relaxed);
        }
    }

    SourceStats stats_;
};

using SourcePtr = std::unique_ptr<Source>;

}  // namespace axle
