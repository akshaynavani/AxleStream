#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "axle/spsc_queue.hpp"

using axle::SpscQueue;

TEST(SpscQueue, CapacityRoundsUpToPowerOfTwo) {
    SpscQueue<int> q(5);
    EXPECT_EQ(q.capacity(), 8u);
    SpscQueue<int> q2(1);
    EXPECT_EQ(q2.capacity(), 2u);
}

TEST(SpscQueue, PushPopFifoOrder) {
    SpscQueue<int> q(4);
    EXPECT_TRUE(q.try_push(1));
    EXPECT_TRUE(q.try_push(2));
    EXPECT_TRUE(q.try_push(3));
    EXPECT_EQ(q.size_approx(), 3u);
    EXPECT_EQ(*q.try_pop(), 1);
    EXPECT_EQ(*q.try_pop(), 2);
    EXPECT_EQ(*q.try_pop(), 3);
    EXPECT_FALSE(q.try_pop().has_value());
}

TEST(SpscQueue, RejectsWhenFull) {
    SpscQueue<int> q(4);
    for (int i = 0; i < 4; ++i) EXPECT_TRUE(q.try_push(i));
    EXPECT_FALSE(q.try_push(99));
    EXPECT_EQ(*q.try_pop(), 0);
    EXPECT_TRUE(q.try_push(99));
}

TEST(SpscQueue, FrontPeeksWithoutRemoving) {
    SpscQueue<int> q(4);
    EXPECT_EQ(q.front(), nullptr);
    q.try_push(7);
    ASSERT_NE(q.front(), nullptr);
    EXPECT_EQ(*q.front(), 7);
    EXPECT_EQ(q.size_approx(), 1u);
}

TEST(SpscQueue, MoveOnlyTypes) {
    SpscQueue<std::unique_ptr<std::string>> q(2);
    EXPECT_TRUE(q.try_push(std::make_unique<std::string>("lidar")));
    auto v = q.try_pop();
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(**v, "lidar");
}

TEST(SpscQueue, DestroysRemainingElements) {
    auto tracker = std::make_shared<int>(0);
    {
        SpscQueue<std::shared_ptr<int>> q(4);
        q.try_push(tracker);
        q.try_push(tracker);
        EXPECT_EQ(tracker.use_count(), 3);
    }
    EXPECT_EQ(tracker.use_count(), 1);
}

TEST(SpscQueue, ConcurrentProducerConsumerPreservesSequence) {
    constexpr std::uint64_t kCount = 1'000'000;
    SpscQueue<std::uint64_t> q(1024);

    std::thread producer([&] {
        for (std::uint64_t i = 0; i < kCount; ++i) q.push(i);
    });

    std::uint64_t expected = 0;
    std::uint64_t sum = 0;
    while (expected < kCount) {
        if (auto v = q.try_pop()) {
            ASSERT_EQ(*v, expected);
            sum += *v;
            ++expected;
        }
    }
    producer.join();
    EXPECT_EQ(sum, kCount * (kCount - 1) / 2);
    EXPECT_TRUE(q.empty_approx());
}

TEST(SpscQueue, ConcurrentLargePayloadsAreIntact) {
    constexpr int kFrames = 2000;
    SpscQueue<std::vector<int>> q(16);
    std::thread producer([&] {
        for (int i = 0; i < kFrames; ++i) q.push(std::vector<int>(256, i));
    });
    for (int i = 0; i < kFrames;) {
        if (auto v = q.try_pop()) {
            ASSERT_EQ(v->size(), 256u);
            ASSERT_EQ(v->front(), i);
            ASSERT_EQ(v->back(), i);
            ++i;
        }
    }
    producer.join();
}
