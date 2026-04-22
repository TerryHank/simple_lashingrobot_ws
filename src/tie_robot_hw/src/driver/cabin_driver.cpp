#include "tie_robot_hw/driver/cabin_driver.hpp"

namespace tie_robot_hw {
namespace driver {

CabinDriver::CabinDriver()
    : transport_(new CabinTcpTransport())
{
}

CabinDriver::CabinDriver(std::unique_ptr<CabinTcpTransport> transport)
    : transport_(std::move(transport))
{
}

CabinDriver::~CabinDriver() = default;

bool CabinDriver::start(DriverError* error)
{
    if (!transport_) {
        if (error != nullptr) {
            error->code = "transport_missing";
            error->message = "索驱驱动缺少 transport";
            error->retryable = false;
        }
        return false;
    }
    return transport_->ensureConnected(error);
}

void CabinDriver::stop()
{
    if (transport_) {
        transport_->disconnect();
    }
}

bool CabinDriver::moveToPose(const CabinPoseCommand& command, DriverError* error)
{
    if (!start(error)) {
        return false;
    }

    std::vector<uint8_t> response;
    if (!transport_->sendAndReceive(
            CabinProtocol::buildMoveToPoseFrame(command),
            &response,
            error,
            8)) {
        return false;
    }

    DriverError protocol_error = CabinProtocol::decodeStatus(0x0012, response);
    if (!protocol_error.code.empty()) {
        if (error != nullptr) {
            *error = protocol_error;
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    last_state_.x_mm = command.x_mm;
    last_state_.y_mm = command.y_mm;
    last_state_.z_mm = command.z_mm;
    last_state_.speed_mm_per_sec = command.speed_mm_per_sec;
    last_state_.connected = true;
    return true;
}

bool CabinDriver::sendStop(DriverError* error)
{
    if (!start(error)) {
        return false;
    }
    std::vector<uint8_t> response;
    return transport_->sendAndReceive(CabinProtocol::buildStopFrame(), &response, error, 8);
}

CabinStateSnapshot CabinDriver::readState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    CabinStateSnapshot snapshot = last_state_;
    snapshot.connected = connectionState() == ConnectionState::kReady;
    return snapshot;
}

ConnectionState CabinDriver::connectionState() const
{
    return transport_ ? transport_->connectionState() : ConnectionState::kDisconnected;
}

std::string CabinDriver::lastErrorText() const
{
    return transport_ ? transport_->lastErrorText() : std::string();
}

int CabinDriver::socketFd() const
{
    return transport_ ? transport_->socketFd() : -1;
}

}  // namespace driver
}  // namespace tie_robot_hw
