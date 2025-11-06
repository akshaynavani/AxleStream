#include "axle/latency.hpp"

#include <algorithm>
#include <cstdio>

namespace axle {

namespace {
int floor_log2(std::uint64_t v) noexcept {
    int r = 0;
    while (v >>= 1) ++r;
    return r;
}
}  // namespace

LatencyHistogram::LatencyHistogram() { reset(); }

int LatencyHistogram::bucket_index(Nanos value) noexcept {
    if (value < kSubBuckets) {
        return static_cast<int>(value < 0 ? 0 : value);  // exact for tiny values
    }
    const auto v = static_cast<std::uint64_t>(value);
    const int exp = floor_log2(v);
    if (exp > kMaxExp) {
        return kNumBuckets - 1;
    }
    // Top kSubBits bits below the leading one select the sub-bucket.
    const int shift = exp - kSubBits;
    const int sub = static_cast<int>((v >> shift) & (kSubBuckets - 1));
    return (exp - kSubBits + 1) * kSubBuckets + sub;
}

Nanos LatencyHistogram::bucket_upper(int index) noexcept {
    if (index < kSubBuckets) {
        return index;
    }
    const int group = index / kSubBuckets;  // >= 1
    const int sub = index % kSubBuckets;
    const int exp = group + kSubBits - 1;
    const int shift = exp - kSubBits;
    const std::uint64_t base = (std::uint64_t{1} << exp) | (static_cast<std::uint64_t>(sub) << shift);
    return static_cast<Nanos>(base + ((std::uint64_t{1} << shift) - 1));
}

void LatencyHistogram::record(Nanos value) noexcept {
    if (value < 0) value = 0;
    buckets_[static_cast<std::size_t>(bucket_index(value))].fetch_add(1, std::memory_order_relaxed);
    count_.fetch_add(1, std::memory_order_relaxed);
    sum_.fetch_add(value, std::memory_order_relaxed);

    std::int64_t cur = min_.load(std::memory_order_relaxed);
    while (value < cur && !min_.compare_exchange_weak(cur, value, std::memory_order_relaxed)) {
    }
    cur = max_.load(std::memory_order_relaxed);
    while (value > cur && !max_.compare_exchange_weak(cur, value, std::memory_order_relaxed)) {
    }
}

void LatencyHistogram::reset() noexcept {
    for (auto& b : buckets_) b.store(0, std::memory_order_relaxed);
    count_.store(0, std::memory_order_relaxed);
    sum_.store(0, std::memory_order_relaxed);
    min_.store(INT64_MAX, std::memory_order_relaxed);
    max_.store(0, std::memory_order_relaxed);
}

Nanos LatencyHistogram::min() const noexcept {
    const auto m = min_.load(std::memory_order_relaxed);
    return m == INT64_MAX ? 0 : m;
}

double LatencyHistogram::mean() const noexcept {
    const auto n = count();
    return n == 0 ? 0.0 : static_cast<double>(sum_.load(std::memory_order_relaxed)) / static_cast<double>(n);
}

Nanos LatencyHistogram::percentile(double p) const noexcept {
    const auto n = count();
    if (n == 0) return 0;
    p = std::clamp(p, 0.0, 100.0);
    auto target = static_cast<std::uint64_t>(p / 100.0 * static_cast<double>(n) + 0.5);
    if (target == 0) target = 1;
    std::uint64_t seen = 0;
    for (int i = 0; i < kNumBuckets; ++i) {
        seen += buckets_[static_cast<std::size_t>(i)].load(std::memory_order_relaxed);
        if (seen >= target) {
            return std::min(bucket_upper(i), max());
        }
    }
    return max();
}

std::uint64_t LatencyHistogram::count_above(Nanos threshold) const noexcept {
    const int idx = bucket_index(threshold);
    std::uint64_t total = 0;
    for (int i = idx + 1; i < kNumBuckets; ++i) {
        total += buckets_[static_cast<std::size_t>(i)].load(std::memory_order_relaxed);
    }
    return total;
}

std::string LatencyHistogram::summary(const std::string& name) const {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "%-14s n=%-8llu mean=%.3fms p50=%.3fms p95=%.3fms p99=%.3fms max=%.3fms",
                  name.c_str(), static_cast<unsigned long long>(count()), mean() / 1e6,
                  ns_to_ms(percentile(50)), ns_to_ms(percentile(95)), ns_to_ms(percentile(99)),
                  ns_to_ms(max()));
    return buf;
}

}  // namespace axle
