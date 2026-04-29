#include "tie_robot_hw/driver/linear_module_driver.hpp"

namespace tie_robot_hw {
namespace driver {

namespace {

constexpr int kRegisterFinishAll = 5210;
constexpr int kRegisterEnableDisable = 5076;
constexpr int kRegisterIsZero = 5072;

}  // namespace

LinearModuleDriver::LinearModuleDriver()
    : transport_(new ModbusTransport())
{
}

LinearModuleDriver::LinearModuleDriver(std::unique_ptr<ModbusTransport> transport)
    : transport_(std::move(transport))
{
}

LinearModuleDriver::~LinearModuleDriver() = default;

bool LinearModuleDriver::start(DriverError* error)
{
    if (!transport_) {
        if (error != nullptr) {
            error->code = "transport_missing";
            error->message = "线性模组驱动缺少 transport";
            error->retryable = false;
        }
        return false;
    }
    return transport_->ensureConnected(error);
}

void LinearModuleDriver::stop()
{
    if (transport_) {
        transport_->disconnect();
    }
}

bool LinearModuleDriver::clearFinishAll(DriverError* error)
{
    return start(error) && transport_->writeSingleRegister(kRegisterFinishAll, 0, error);
}

bool LinearModuleDriver::writeQueuedPoints(
    const std::vector<LinearModulePoint>& points,
    DriverError* error)
{
    if (!start(error)) {
        return false;
    }

    for (std::size_t index = 0; index < points.size(); ++index) {
        if (!transport_->writeRegisters(
                LinearModuleProtocol::pointBaseRegister(static_cast<int>(index)),
                LinearModuleProtocol::buildPointRegisterPayload(points[index]),
                error)) {
            return false;
        }
    }

    return true;
}

bool LinearModuleDriver::pulseExecutionEnable(DriverError* error)
{
    if (!start(error)) {
        return false;
    }
    if (!transport_->writeSingleRegister(kRegisterEnableDisable, 0, error)) {
        return false;
    }
    if (!transport_->writeSingleRegister(kRegisterEnableDisable, 1, error)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    last_state_.connected = true;
    return true;
}

bool LinearModuleDriver::setZeroRequest(bool requested, DriverError* error)
{
    if (!start(error)) {
        return false;
    }
    return transport_->writeSingleRegister(kRegisterIsZero, requested ? 1 : 0, error);
}

LinearModuleStateSnapshot LinearModuleDriver::readState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    LinearModuleStateSnapshot snapshot = last_state_;
    snapshot.connected = connectionState() == ConnectionState::kReady;
    return snapshot;
}

ConnectionState LinearModuleDriver::connectionState() const
{
    return transport_ ? transport_->connectionState() : ConnectionState::kDisconnected;
}

std::string LinearModuleDriver::lastErrorText() const
{
    return transport_ ? transport_->lastErrorText() : std::string();
}

}  // namespace driver
}  // namespace tie_robot_hw
