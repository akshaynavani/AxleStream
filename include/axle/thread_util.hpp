#pragma once

#include <string>
#include <thread>

#include "axle/clock.hpp"

namespace axle {

/// Pin the calling thread to one CPU core. Returns false (and does nothing) when the
/// platform does not support affinity or the core does not exist.
bool pin_current_thread(int cpu);

/// Best-effort thread name (visible in htop / perf / gdb). Truncated to 15 chars on Linux.
void set_current_thread_name(const std::string& name);

/// Switch the calling thread to SCHED_FIFO at `priority`. Needs CAP_SYS_NICE.
bool set_realtime_priority(int priority);

/// Idle strategy for consumer loops: spin briefly (lowest wake-up latency), then yield,
/// then sleep in short naps so an idle pipeline does not burn whole cores.
class Backoff {
public:
    explicit Backoff(int spin_limit = 2000) : spin_limit_(spin_limit) {}

    void idle() {
        if (count_ < spin_limit_) {
            cpu_relax();
        } else if (count_ < spin_limit_ + 100) {
            std::this_thread::yield();
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
        ++count_;
    }
    void reset() noexcept { count_ = 0; }

private:
    int spin_limit_;
    int count_{0};
};

}  // namespace axle
