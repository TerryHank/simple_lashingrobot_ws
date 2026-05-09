#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "tie_robot_hw/driver/cabin_protocol.hpp"
#include "tie_robot_hw/driver/cabin_tcp_transport.hpp"

namespace tie_robot_hw {
namespace driver {

class CabinDriver {
public:
    CabinDriver();
    explicit CabinDriver(std::unique_ptr<CabinTcpTransport> transport);
    ~CabinDriver();

    bool start(DriverError* error = nullptr);
    void stop();
    bool moveToPose(const CabinPoseCommand& command, DriverError* error);
    bool moveByOffset(const CabinPoseCommand& command, DriverError* error);
    bool sendStop(DriverError* error);
    void markExternalIoSuccess();
    CabinStateSnapshot readState() const;
    ConnectionState connectionState() const;
    std::string lastErrorText() const;
    int socketFd() const;

private:
    std::unique_ptr<CabinTcpTransport> transport_;
    mutable std::mutex state_mutex_;
    CabinStateSnapshot last_state_;
};

}  // namespace driver
}  // namespace tie_robot_hw
