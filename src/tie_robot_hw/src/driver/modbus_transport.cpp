#include "tie_robot_hw/driver/modbus_transport.hpp"

#include <cerrno>
#include <cstring>

namespace tie_robot_hw {
namespace driver {

ModbusTransport::ModbusTransport(const std::string& host, int port)
    : host_(host),
      port_(port),
      context_(nullptr),
      connection_state_(ConnectionState::kDisconnected)
{
}

ModbusTransport::~ModbusTransport()
{
    disconnect();
}

bool ModbusTransport::setError(
    const std::string& code,
    const std::string& message,
    const std::string& detail,
    bool retryable,
    DriverError* error)
{
    if (error != nullptr) {
        error->code = code;
        error->message = message;
        error->detail = detail;
        error->retryable = retryable;
    }
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_text_ = message;
        if (!detail.empty()) {
            last_error_text_ += ": " + detail;
        }
    }
    return false;
}

bool ModbusTransport::connect(DriverError* error)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    return connectLocked(error);
}

void ModbusTransport::disconnect()
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    disconnectLocked();
}

bool ModbusTransport::ensureConnected(DriverError* error)
{
    if (connectionState() == ConnectionState::kReady && context_ != nullptr) {
        if (error != nullptr) {
            error->clear();
        }
        return true;
    }
    return connect(error);
}

bool ModbusTransport::writeSingleRegister(int address, uint16_t value, DriverError* error)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (context_ == nullptr && !connectLocked(error)) {
        return false;
    }
    if (modbus_write_register(context_, address, value) == -1) {
        connection_state_.store(ConnectionState::kReconnecting);
        return setError("modbus_write_single_failed", "Modbus 单寄存器写入失败", modbus_strerror(errno), true, error);
    }
    connection_state_.store(ConnectionState::kReady);
    return true;
}

bool ModbusTransport::writeRegisters(int address, const std::vector<uint16_t>& values, DriverError* error)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (context_ == nullptr && !connectLocked(error)) {
        return false;
    }
    if (modbus_write_registers(context_, address, static_cast<int>(values.size()), values.data()) == -1) {
        connection_state_.store(ConnectionState::kReconnecting);
        return setError("modbus_write_multi_failed", "Modbus 多寄存器写入失败", modbus_strerror(errno), true, error);
    }
    connection_state_.store(ConnectionState::kReady);
    return true;
}

bool ModbusTransport::readRegisters(
    int address,
    int count,
    std::vector<uint16_t>* values,
    DriverError* error)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (context_ == nullptr && !connectLocked(error)) {
        return false;
    }
    std::vector<uint16_t> local_values(static_cast<std::size_t>(count), 0);
    if (modbus_read_registers(context_, address, count, local_values.data()) == -1) {
        connection_state_.store(ConnectionState::kReconnecting);
        return setError("modbus_read_failed", "Modbus 寄存器读取失败", modbus_strerror(errno), true, error);
    }
    if (values != nullptr) {
        *values = local_values;
    }
    connection_state_.store(ConnectionState::kReady);
    return true;
}

ConnectionState ModbusTransport::connectionState() const
{
    return connection_state_.load();
}

std::string ModbusTransport::lastErrorText() const
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_text_;
}

modbus_t* ModbusTransport::rawContext() const
{
    return context_;
}

bool ModbusTransport::connectLocked(DriverError* error)
{
    disconnectLocked();
    connection_state_.store(ConnectionState::kConnecting);

    context_ = modbus_new_tcp(host_.c_str(), port_);
    if (context_ == nullptr) {
        connection_state_.store(ConnectionState::kFault);
        return setError("modbus_new_tcp_failed", "创建 Modbus TCP 上下文失败", std::strerror(errno), false, error);
    }

    if (modbus_connect(context_) == -1) {
        connection_state_.store(ConnectionState::kReconnecting);
        const bool result = setError(
            "modbus_connect_failed",
            "Modbus TCP 连接失败",
            modbus_strerror(errno),
            true,
            error
        );
        disconnectLocked();
        return result;
    }

    connection_state_.store(ConnectionState::kReady);
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

void ModbusTransport::disconnectLocked()
{
    if (context_ != nullptr) {
        modbus_close(context_);
        modbus_free(context_);
        context_ = nullptr;
    }
    connection_state_.store(ConnectionState::kDisconnected);
}

}  // namespace driver
}  // namespace tie_robot_hw
