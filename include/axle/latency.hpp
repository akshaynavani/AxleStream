#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <string>

#include "axle/clock.hpp"

namespace axle {

/// Fixed-memory latency histogram with logarithmic buckets (HDR-style, ~3% resolution).
///
/// Recording is wait-free and can happen from one writer thread while another thread takes
/// snapshots; counts are relaxed atomics so a snapshot is eventually consistent, which is
/// fine for monitoring.
class LatencyHistogram {
public:
    // Sub-buckets per power of two; 32 gives ~3% relative error.
    static constexpr int kSubBits = 5;
    static constexpr int kSubBuckets = 1 << kSubBits;
    // Covers 1 ns .. ~2^40 ns (~18 minutes), more than enough.
    static constexpr int kMaxExp = 40;
    static constexpr int kNumBuckets = (kMaxExp + 1) * kSubBuckets;

    LatencyHistogram();

    void record(Nanos value) noexcept;
    void reset() noexcept;

    std::uint64_t count() const noexcept { return count_.load(std::memory_order_relaxed); }
    Nanos min() const noexcept;
    Nanos max() const noexcept { return max_.load(std::memory_order_relaxed); }
    double mean() const noexcept;

    /// Value at the given percentile (0..100). Returns the upper bound of the bucket.
    Nanos percentile(double p) const noexcept;

    /// Number of samples strictly greater than `threshold` (bucket-resolution estimate).
    std::uint64_t count_above(Nanos threshold) const noexcept;

    /// One-line human summary in milliseconds.
    std::string summary(const std::string& name) const;

    static int bucket_index(Nanos value) noexcept;
    static Nanos bucket_upper(int index) noexcept;

private:
    std::array<std::atomic<std::uint64_t>, kNumBuckets> buckets_;
    std::atomic<std::uint64_t> count_{0};
    std::atomic<std::int64_t> sum_{0};
    std::atomic<std::int64_t> min_{INT64_MAX};
    std::atomic<std::int64_t> max_{0};
};

}  // namespace axle
