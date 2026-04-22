#include "tie_robot_hw/driver/cabin_tcp_transport.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace tie_robot_hw {
namespace driver {

CabinTcpTransport::CabinTcpTransport(const std::string& host, uint16_t port, int timeout_sec)
    : host_(host),
      port_(port),
      timeout_sec_(timeout_sec),
      socket_fd_(-1),
      connection_state_(ConnectionState::kDisconnected)
{
}

CabinTcpTransport::~CabinTcpTransport()
{
    disconnect();
}

bool CabinTcpTransport::setError(
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

bool CabinTcpTransport::connect(DriverError* error)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    return connectLocked(error);
}

void CabinTcpTransport::disconnect()
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    disconnectLocked();
}

bool CabinTcpTransport::ensureConnected(DriverError* error)
{
    if (connectionState() == ConnectionState::kReady && socket_fd_ >= 0) {
        if (error != nullptr) {
            error->clear();
        }
        return true;
    }
    return connect(error);
}

bool CabinTcpTransport::sendAndReceive(
    const std::vector<uint8_t>& request,
    std::vector<uint8_t>* response,
    DriverError* error,
    int expected_response_bytes)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (socket_fd_ < 0 && !connectLocked(error)) {
        return false;
    }

    std::size_t total_sent = 0;
    while (total_sent < request.size()) {
        const ssize_t sent = ::send(
            socket_fd_,
            request.data() + total_sent,
            request.size() - total_sent,
            0
        );
        if (sent <= 0) {
            connection_state_.store(ConnectionState::kReconnecting);
            return setError(
                "tcp_send_failed",
                "索驱 TCP 发送失败",
                std::strerror(errno),
                true,
                error
            );
        }
        total_sent += static_cast<std::size_t>(sent);
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);
    timeval timeout;
    timeout.tv_sec = timeout_sec_;
    timeout.tv_usec = 0;
    const int select_result = ::select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (select_result <= 0) {
        connection_state_.store(ConnectionState::kReconnecting);
        const std::string detail = select_result == 0 ? "read timeout" : std::strerror(errno);
        return setError("tcp_read_wait_failed", "索驱 TCP 等待回包失败", detail, true, error);
    }

    std::vector<uint8_t> local_response(static_cast<std::size_t>(
        expected_response_bytes > 0 ? expected_response_bytes : 256
    ));
    const ssize_t recv_len = ::recv(socket_fd_, local_response.data(), local_response.size(), 0);
    if (recv_len <= 0) {
        connection_state_.store(ConnectionState::kReconnecting);
        const std::string detail =
            recv_len == 0 ? "connection closed by peer" : std::strerror(errno);
        return setError("tcp_recv_failed", "索驱 TCP 接收失败", detail, true, error);
    }

    local_response.resize(static_cast<std::size_t>(recv_len));
    if (response != nullptr) {
        *response = local_response;
    }
    connection_state_.store(ConnectionState::kReady);
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

ConnectionState CabinTcpTransport::connectionState() const
{
    return connection_state_.load();
}

std::string CabinTcpTransport::lastErrorText() const
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_text_;
}

int CabinTcpTransport::socketFd() const
{
    return socket_fd_;
}

bool CabinTcpTransport::connectLocked(DriverError* error)
{
    disconnectLocked();
    connection_state_.store(ConnectionState::kConnecting);

    socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        connection_state_.store(ConnectionState::kFault);
        return setError("socket_create_failed", "创建索驱 TCP socket 失败", std::strerror(errno), true, error);
    }

    int reuse_opt = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_opt, sizeof(reuse_opt));
    int no_delay = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));

    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    if (::inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) != 1) {
        connection_state_.store(ConnectionState::kFault);
        const bool result = setError(
            "invalid_ip_address",
            "索驱 TCP 目标地址非法",
            host_,
            false,
            error
        );
        disconnectLocked();
        return result;
    }

    if (::connect(socket_fd_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        connection_state_.store(ConnectionState::kReconnecting);
        const bool result = setError(
            "tcp_connect_failed",
            "索驱 TCP 连接失败",
            std::strerror(errno),
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

void CabinTcpTransport::disconnectLocked()
{
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    connection_state_.store(ConnectionState::kDisconnected);
}

}  // namespace driver
}  // namespace tie_robot_hw
