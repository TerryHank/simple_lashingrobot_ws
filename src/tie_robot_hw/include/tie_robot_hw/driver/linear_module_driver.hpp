#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "tie_robot_hw/driver/driver_types.hpp"
#include "tie_robot_hw/driver/linear_module_protocol.hpp"
#include "tie_robot_hw/driver/modbus_transport.hpp"

namespace tie_robot_hw {
namespace driver {

class LinearModuleDriver {
public:
    LinearModuleDriver();
    explicit LinearModuleDriver(std::unique_ptr<ModbusTransport> transport);
    ~LinearModuleDriver();

    bool start(DriverError* error = nullptr);
    void stop();
    bool clearFinishAll(DriverError* error);
    bool writeQueuedPoints(
        const std::vector<LinearModulePoint>& points,
        DriverError* error);
    bool pulseExecutionEnable(DriverError* error);
    bool setZeroRequest(bool requested, DriverError* error);
    LinearModuleStateSnapshot readState() const;
    ConnectionState connectionState() const;
    std::string lastErrorText() const;

private:
    std::unique_ptr<ModbusTransport> transport_;
    mutable std::mutex state_mutex_;
    LinearModuleStateSnapshot last_state_;
};

}  // namespace driver
}  // namespace tie_robot_hw
