#pragma once

#include <chrono>
#include <cstdint>
#include <thread>

namespace axle {

using Nanos = std::int64_t;

/// Monotonic nanoseconds since an arbitrary epoch. Used for all latency measurement.
inline Nanos now_ns() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

/// Wall-clock nanoseconds since the Unix epoch. Used for exported timestamps.
inline Nanos wall_ns() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

constexpr Nanos ms_to_ns(double ms) noexcept { return static_cast<Nanos>(ms * 1'000'000.0); }
constexpr double ns_to_ms(Nanos ns) noexcept { return static_cast<double>(ns) / 1'000'000.0; }

/// Sleep until an absolute monotonic deadline (coarse sleep, then short spin for precision).
inline void sleep_until_ns(Nanos deadline) {
    constexpr Nanos kSpinWindow = 200'000;  // 200 us
    Nanos remaining = deadline - now_ns();
    if (remaining > kSpinWindow) {
        std::this_thread::sleep_for(std::chrono::nanoseconds(remaining - kSpinWindow));
    }
    while (now_ns() < deadline) {
        std::this_thread::yield();
    }
}

/// CPU relax hint for spin loops; maps to YIELD on ARM and PAUSE on x86.
inline void cpu_relax() noexcept {
#if defined(__aarch64__) || defined(__arm__)
    asm volatile("yield" ::: "memory");
#elif defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#else
    std::this_thread::yield();
#endif
}

}  // namespace axle
