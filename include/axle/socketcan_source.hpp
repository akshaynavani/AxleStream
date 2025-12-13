#pragma once

#include <string>

#include "axle/source.hpp"

namespace axle {

/// Reads raw CAN frames from a Linux SocketCAN interface (e.g. `can0`, or `vcan0` for a
/// virtual bus fed by `cangen`/`canplayer`). Only built with -DAXLE_WITH_SOCKETCAN=ON.
/// Kernel receive timestamps (SO_TIMESTAMP) are used as sensor timestamps when available.
class SocketCanSource final : public Source {
public:
    explicit SocketCanSource(std::string ifname);
    ~SocketCanSource() override;

    std::string name() const override { return "socketcan:" + ifname_; }
    std::uint8_t kinds() const override { return kCanMask; }
    void run(const std::atomic<bool>& stop, const SensorSinks& sinks) override;

    static bool available() noexcept;

private:
    std::string ifname_;
    [[maybe_unused]] int fd_{-1};  // unused when SocketCAN is compiled out
};

}  // namespace axle
