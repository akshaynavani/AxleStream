#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <new>
#include <optional>
#include <type_traits>
#include <utility>

#include "axle/clock.hpp"

namespace axle {

// Fixed rather than std::hardware_destructive_interference_size, which GCC flags as
// ABI-unstable. 64 bytes matches Cortex-A7x/Neoverse cores and x86.
inline constexpr std::size_t kCacheLine = 64;

/// Bounded single-producer / single-consumer lock-free ring buffer.
///
/// - Capacity is rounded up to a power of two so index wrap is a mask.
/// - Head and tail live on separate cache lines to avoid false sharing between the
///   producer and consumer cores.
/// - Each side keeps a cached copy of the other side's index and only reloads the shared
///   atomic when the cache says the queue looks full/empty, which keeps cross-core traffic low.
/// - Acquire/release ordering is sufficient on both ARMv8 and x86.
template <typename T>
class SpscQueue {
public:
    explicit SpscQueue(std::size_t capacity)
        : capacity_(round_up_pow2(capacity < 2 ? 2 : capacity)),
          mask_(capacity_ - 1),
          slots_(static_cast<Slot*>(::operator new[](sizeof(Slot) * capacity_, std::align_val_t{alignof(Slot)}))) {}

    ~SpscQueue() {
        // Destroy any elements still in flight.
        std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        while (head != tail) {
            ptr(head)->~T();
            ++head;
        }
        ::operator delete[](slots_, std::align_val_t{alignof(Slot)});
    }

    SpscQueue(const SpscQueue&) = delete;
    SpscQueue& operator=(const SpscQueue&) = delete;

    /// Producer side. Returns false if the queue is full.
    template <typename... Args>
    bool try_emplace(Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>) {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail - head_cache_ == capacity_) {
            head_cache_ = head_.load(std::memory_order_acquire);
            if (tail - head_cache_ == capacity_) {
                return false;
            }
        }
        new (ptr(tail)) T(std::forward<Args>(args)...);
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    bool try_push(const T& v) { return try_emplace(v); }
    bool try_push(T&& v) { return try_emplace(std::move(v)); }

    /// Producer side. Spins (with CPU relax) until space is available.
    void push(T v) {
        while (!try_emplace(std::move(v))) {
            cpu_relax();
        }
    }

    /// Consumer side. Returns std::nullopt if the queue is empty.
    std::optional<T> try_pop() noexcept(std::is_nothrow_move_constructible_v<T>) {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        if (head == tail_cache_) {
            tail_cache_ = tail_.load(std::memory_order_acquire);
            if (head == tail_cache_) {
                return std::nullopt;
            }
        }
        T* p = ptr(head);
        std::optional<T> out(std::move(*p));
        p->~T();
        head_.store(head + 1, std::memory_order_release);
        return out;
    }

    /// Consumer side. Peek at the front element without removing it (nullptr if empty).
    T* front() noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        if (head == tail_cache_) {
            tail_cache_ = tail_.load(std::memory_order_acquire);
            if (head == tail_cache_) {
                return nullptr;
            }
        }
        return ptr(head);
    }

    /// Approximate size; exact only when called from a quiescent state.
    std::size_t size_approx() const noexcept {
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        const std::size_t head = head_.load(std::memory_order_acquire);
        return tail - head;
    }

    bool empty_approx() const noexcept { return size_approx() == 0; }
    std::size_t capacity() const noexcept { return capacity_; }

private:
    struct alignas(T) Slot {
        unsigned char storage[sizeof(T)];
    };

    static std::size_t round_up_pow2(std::size_t v) {
        std::size_t p = 1;
        while (p < v) p <<= 1;
        return p;
    }

    T* ptr(std::size_t idx) noexcept {
        return std::launder(reinterpret_cast<T*>(slots_[idx & mask_].storage));
    }

    const std::size_t capacity_;
    const std::size_t mask_;
    Slot* const slots_;

    alignas(kCacheLine) std::atomic<std::size_t> head_{0};  // written by consumer
    alignas(kCacheLine) std::size_t tail_cache_{0};          // consumer-local
    alignas(kCacheLine) std::atomic<std::size_t> tail_{0};  // written by producer
    alignas(kCacheLine) std::size_t head_cache_{0};          // producer-local
};

}  // namespace axle
