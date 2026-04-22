#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "tie_robot_hw/driver/driver_types.hpp"

namespace tie_robot_hw {
namespace driver {

class CabinTcpTransport {
public:
    CabinTcpTransport(
        const std::string& host = "192.168.6.62",
        uint16_t port = 2001,
        int timeout_sec = 5);
    ~CabinTcpTransport();

    bool connect(DriverError* error);
    void disconnect();
    bool ensureConnected(DriverError* error);
    bool sendAndReceive(
        const std::vector<uint8_t>& request,
        std::vector<uint8_t>* response,
        DriverError* error,
        int expected_response_bytes = 0);

    ConnectionState connectionState() const;
    std::string lastErrorText() const;
    int socketFd() const;

private:
    bool connectLocked(DriverError* error);
    void disconnectLocked();
    bool setError(
        const std::string& code,
        const std::string& message,
        const std::string& detail,
        bool retryable,
        DriverError* error);

    std::string host_;
    uint16_t port_;
    int timeout_sec_;
    int socket_fd_;
    mutable std::mutex io_mutex_;
    mutable std::mutex error_mutex_;
    std::atomic<ConnectionState> connection_state_;
    std::string last_error_text_;
};

}  // namespace driver
}  // namespace tie_robot_hw
