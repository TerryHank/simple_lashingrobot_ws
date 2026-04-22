#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <modbus.h>

#include "tie_robot_hw/driver/driver_types.hpp"

namespace tie_robot_hw {
namespace driver {

class ModbusTransport {
public:
    ModbusTransport(
        const std::string& host = "192.168.6.167",
        int port = 502);
    ~ModbusTransport();

    bool connect(DriverError* error);
    void disconnect();
    bool ensureConnected(DriverError* error);
    bool writeSingleRegister(int address, uint16_t value, DriverError* error);
    bool writeRegisters(int address, const std::vector<uint16_t>& values, DriverError* error);
    bool readRegisters(
        int address,
        int count,
        std::vector<uint16_t>* values,
        DriverError* error);

    ConnectionState connectionState() const;
    std::string lastErrorText() const;
    modbus_t* rawContext() const;

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
    int port_;
    modbus_t* context_;
    mutable std::mutex io_mutex_;
    mutable std::mutex error_mutex_;
    std::atomic<ConnectionState> connection_state_;
    std::string last_error_text_;
};

}  // namespace driver
}  // namespace tie_robot_hw
