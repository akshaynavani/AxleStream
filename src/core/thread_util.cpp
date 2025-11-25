#include "axle/thread_util.hpp"

#include <cstdio>
#include <cstring>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

namespace axle {

bool pin_current_thread(int cpu) {
    if (cpu < 0) return false;
#if defined(__linux__)
    if (cpu >= static_cast<int>(std::thread::hardware_concurrency())) return false;
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<unsigned>(cpu), &set);
    return pthread_setaffinity_np(pthread_self(), sizeof set, &set) == 0;
#else
    // macOS has no hard affinity API; treat as unsupported.
    return false;
#endif
}

void set_current_thread_name(const std::string& name) {
#if defined(__linux__)
    char buf[16];
    std::strncpy(buf, name.c_str(), sizeof buf - 1);
    buf[sizeof buf - 1] = '\0';
    pthread_setname_np(pthread_self(), buf);
#elif defined(__APPLE__)
    pthread_setname_np(name.c_str());
#else
    (void)name;
#endif
}

bool set_realtime_priority(int priority) {
    if (priority <= 0) return false;
#if defined(__linux__)
    sched_param sp{};
    sp.sched_priority = priority;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) {
        std::fprintf(stderr, "[axle] SCHED_FIFO(%d) unavailable: %s\n", priority, std::strerror(rc));
        return false;
    }
    return true;
#else
    return false;
#endif
}

}  // namespace axle
