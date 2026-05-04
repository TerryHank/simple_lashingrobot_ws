#include "tie_robot_hw/driver/cabin_driver.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace tie_robot_hw {
namespace driver {
namespace {

constexpr float kIncrementalAxisEpsilonMm = 0.001f;

DriverError validateIncrementalMoveCommand(const CabinPoseCommand& command)
{
    DriverError error;
    const int nonzero_axis_count =
        (std::fabs(command.x_mm) > kIncrementalAxisEpsilonMm ? 1 : 0) +
        (std::fabs(command.y_mm) > kIncrementalAxisEpsilonMm ? 1 : 0) +
        (std::fabs(command.z_mm) > kIncrementalAxisEpsilonMm ? 1 : 0);

    if (nonzero_axis_count != 1) {
        std::ostringstream detail;
        detail << "x_delta=" << command.x_mm
               << "，y_delta=" << command.y_mm
               << "，z_delta=" << command.z_mm
               << "；遥控步距一次只允许选择一个非零轴";
        error.code = "invalid_incremental_move_axis";
        error.message = "索驱增量运动指令轴选择非法";
        error.detail = detail.str();
        error.retryable = false;
    }
    return error;
}

void resetTransportAfterProtocolDesync(CabinTcpTransport& transport, DriverError& protocol_error)
{
    if (protocol_error.code != "protocol_response_desynchronized") {
        return;
    }

    transport.disconnect();
    if (!protocol_error.detail.empty()) {
        protocol_error.detail += "，已断开当前TCP连接以清理残留回包；为避免TCP相对运动重复执行，本次指令不会自动重发，请确认设备状态后重试";
    }
}

uint16_t requestCommandWord(const std::vector<uint8_t>& request)
{
    if (request.size() < 4) {
        return 0;
    }
    return static_cast<uint16_t>((static_cast<uint16_t>(request[2]) << 8) | request[3]);
}

std::string formatRequestContext(const std::vector<uint8_t>& request)
{
    std::ostringstream stream;
    stream << "request_command=";
    if (request.size() >= 4) {
        stream << "0x"
               << std::uppercase
               << std::hex
               << std::setw(4)
               << std::setfill('0')
               << requestCommandWord(request)
               << std::dec;
    } else {
        stream << "unknown";
    }

    stream << "，request_frame=[";
    for (std::size_t index = 0; index < request.size(); ++index) {
        if (index > 0) {
            stream << ' ';
        }
        stream << std::uppercase
               << std::hex
               << std::setw(2)
               << std::setfill('0')
               << static_cast<unsigned int>(request[index]);
    }
    stream << "]";
    return stream.str();
}

void appendRequestContext(DriverError& protocol_error, const std::vector<uint8_t>& request)
{
    if (protocol_error.detail.find("request_frame=[") != std::string::npos) {
        return;
    }
    const std::string request_context = formatRequestContext(request);
    if (protocol_error.detail.empty()) {
        protocol_error.detail = request_context;
        return;
    }
    protocol_error.detail += "，" + request_context;
}

}  // namespace

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

    const std::vector<uint8_t> request = CabinProtocol::buildMoveToPoseFrame(command);
    std::vector<uint8_t> response;
    if (!transport_->sendAndReceive(
            request,
            &response,
            error,
            8)) {
        return false;
    }

    DriverError protocol_error = CabinProtocol::decodeStatus(0x0012, response);
    if (!protocol_error.code.empty()) {
        appendRequestContext(protocol_error, request);
        resetTransportAfterProtocolDesync(*transport_, protocol_error);
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

bool CabinDriver::moveByOffset(const CabinPoseCommand& command, DriverError* error)
{
    DriverError validation_error = validateIncrementalMoveCommand(command);
    if (!validation_error.code.empty()) {
        if (error != nullptr) {
            *error = validation_error;
        }
        return false;
    }

    if (!start(error)) {
        return false;
    }

    const std::vector<uint8_t> request = CabinProtocol::buildRelativeMoveFrame(command);
    std::vector<uint8_t> response;
    if (!transport_->sendAndReceive(
            request,
            &response,
            error,
            8)) {
        return false;
    }

    DriverError protocol_error = CabinProtocol::decodeStatus(0x0012, response);
    if (!protocol_error.code.empty()) {
        appendRequestContext(protocol_error, request);
        resetTransportAfterProtocolDesync(*transport_, protocol_error);
        if (error != nullptr) {
            *error = protocol_error;
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (last_state_.connected) {
        last_state_.x_mm += command.x_mm;
        last_state_.y_mm += command.y_mm;
        last_state_.z_mm += command.z_mm;
    }
    last_state_.speed_mm_per_sec = command.speed_mm_per_sec;
    last_state_.connected = true;
    return true;
}

bool CabinDriver::sendStop(DriverError* error)
{
    if (!start(error)) {
        return false;
    }

    DriverError driver_error;
    const std::vector<uint8_t> request = CabinProtocol::buildStopFrame();
    std::vector<uint8_t> response;
    if (transport_->sendAndReceive(request, &response, &driver_error, 8)) {
        DriverError protocol_error = CabinProtocol::decodeStatus(0x0013, response);
        if (!protocol_error.code.empty()) {
            appendRequestContext(protocol_error, request);
            resetTransportAfterProtocolDesync(*transport_, protocol_error);
            if (error != nullptr) {
                *error = protocol_error;
            }
            return false;
        }
        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    if (driver_error.code == "tcp_recv_failed" &&
        driver_error.detail.find("connection closed by peer") != std::string::npos) {
        transport_->disconnect();
        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    if (error != nullptr) {
        *error = driver_error;
    }
    return false;
}

void CabinDriver::markExternalIoSuccess()
{
    if (transport_) {
        transport_->markExternalIoSuccess();
    }
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
