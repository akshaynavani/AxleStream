#include "axle/socketcan_source.hpp"

#include <stdexcept>

#if defined(AXLE_WITH_SOCKETCAN) && defined(__linux__)
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#endif

namespace axle {

#if defined(AXLE_WITH_SOCKETCAN) && defined(__linux__)

bool SocketCanSource::available() noexcept { return true; }

SocketCanSource::SocketCanSource(std::string ifname) : ifname_(std::move(ifname)) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("socket(PF_CAN) failed: " + std::string(std::strerror(errno)));

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd_);
        throw std::runtime_error("unknown CAN interface: " + ifname_);
    }

    const int enable = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof enable);

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
        ::close(fd_);
        throw std::runtime_error("bind(" + ifname_ + ") failed: " + std::string(std::strerror(errno)));
    }
}

SocketCanSource::~SocketCanSource() {
    if (fd_ >= 0) ::close(fd_);
}

void SocketCanSource::run(const std::atomic<bool>& stop, const SensorSinks& sinks) {
    std::uint64_t seq = 0;
    pollfd pfd{fd_, POLLIN, 0};

    while (!stop.load(std::memory_order_relaxed)) {
        // Short poll timeout so the stop flag is observed promptly.
        const int ready = ::poll(&pfd, 1, 50);
        if (ready <= 0) continue;

        can_frame raw{};
        iovec iov{&raw, sizeof raw};
        alignas(cmsghdr) char ctrl[CMSG_SPACE(sizeof(timeval))];
        msghdr msg{};
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = ctrl;
        msg.msg_controllen = sizeof ctrl;

        const ssize_t n = ::recvmsg(fd_, &msg, 0);
        if (n != static_cast<ssize_t>(sizeof raw)) continue;
        if (raw.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) continue;  // skip error/remote frames

        Nanos ts = wall_ns();
        for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c; c = CMSG_NXTHDR(&msg, c)) {
            if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMP) {
                timeval tv;
                std::memcpy(&tv, CMSG_DATA(c), sizeof tv);
                ts = static_cast<Nanos>(tv.tv_sec) * 1'000'000'000 + static_cast<Nanos>(tv.tv_usec) * 1000;
            }
        }

        CanFrame f;
        f.seq = seq++;
        f.sensor_ts_ns = ts;
        f.extended = (raw.can_id & CAN_EFF_FLAG) != 0;
        f.id = raw.can_id & (f.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
        f.dlc = raw.can_dlc > 8 ? 8 : raw.can_dlc;
        std::memcpy(f.data.data(), raw.data, f.dlc);
        f.ingest_ns = now_ns();
        emit(sinks.can, std::move(f));
    }
}

#else  // SocketCAN disabled or non-Linux platform

bool SocketCanSource::available() noexcept { return false; }

SocketCanSource::SocketCanSource(std::string ifname) : ifname_(std::move(ifname)) {
    throw std::runtime_error(
        "SocketCAN support not compiled in; rebuild on Linux with -DAXLE_WITH_SOCKETCAN=ON");
}

SocketCanSource::~SocketCanSource() = default;

void SocketCanSource::run(const std::atomic<bool>&, const SensorSinks&) {}

#endif

}  // namespace axle
