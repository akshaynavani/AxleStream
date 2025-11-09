#include <gtest/gtest.h>

#include <thread>

#include "axle/latency.hpp"

using axle::LatencyHistogram;
using axle::ms_to_ns;

TEST(LatencyHistogram, EmptyHistogramIsZero) {
    LatencyHistogram h;
    EXPECT_EQ(h.count(), 0u);
    EXPECT_EQ(h.percentile(99), 0);
    EXPECT_EQ(h.min(), 0);
    EXPECT_DOUBLE_EQ(h.mean(), 0.0);
}

TEST(LatencyHistogram, BucketBoundsContainValue) {
    for (axle::Nanos v : {0LL, 1LL, 31LL, 32LL, 33LL, 1000LL, 123456LL, 9'999'999LL, 5'000'000'000LL}) {
        const int idx = LatencyHistogram::bucket_index(v);
        EXPECT_GE(LatencyHistogram::bucket_upper(idx), v) << v;
        if (idx > 0) {
            EXPECT_LT(LatencyHistogram::bucket_upper(idx - 1), v) << v;
        }
    }
}

TEST(LatencyHistogram, RelativeErrorWithinFourPercent) {
    for (axle::Nanos v = 100; v < 100'000'000; v = v * 3 / 2 + 7) {
        const auto upper = LatencyHistogram::bucket_upper(LatencyHistogram::bucket_index(v));
        EXPECT_LE(static_cast<double>(upper - v) / static_cast<double>(v), 0.04) << v;
    }
}

TEST(LatencyHistogram, PercentilesOnUniformData) {
    LatencyHistogram h;
    for (int i = 1; i <= 1000; ++i) h.record(ms_to_ns(i * 0.01));  // 0.01ms .. 10ms
    EXPECT_EQ(h.count(), 1000u);
    EXPECT_NEAR(axle::ns_to_ms(h.percentile(50)), 5.0, 0.2);
    EXPECT_NEAR(axle::ns_to_ms(h.percentile(99)), 9.9, 0.4);
    EXPECT_EQ(h.max(), ms_to_ns(10.0));
    EXPECT_EQ(h.min(), ms_to_ns(0.01));
    EXPECT_NEAR(h.mean() / 1e6, 5.005, 0.01);
}

TEST(LatencyHistogram, CountAboveBudget) {
    LatencyHistogram h;
    for (int i = 0; i < 95; ++i) h.record(ms_to_ns(2));
    for (int i = 0; i < 5; ++i) h.record(ms_to_ns(15));
    EXPECT_EQ(h.count_above(ms_to_ns(10)), 5u);
}

TEST(LatencyHistogram, ResetClears) {
    LatencyHistogram h;
    h.record(1000);
    h.reset();
    EXPECT_EQ(h.count(), 0u);
    EXPECT_EQ(h.max(), 0);
}

TEST(LatencyHistogram, ConcurrentRecordersCountEverything) {
    LatencyHistogram h;
    std::thread a([&] { for (int i = 0; i < 100000; ++i) h.record(1000 + i); });
    std::thread b([&] { for (int i = 0; i < 100000; ++i) h.record(5000 + i); });
    a.join();
    b.join();
    EXPECT_EQ(h.count(), 200000u);
}
