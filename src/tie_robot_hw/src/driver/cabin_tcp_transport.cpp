#include "tie_robot_hw/driver/cabin_tcp_transport.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <netinet/tcp.h>
#include <sstream>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace tie_robot_hw {
namespace driver {
namespace {

constexpr std::size_t kCabinMotionResponseBytes = 8;
constexpr std::size_t kCabinStateResponseBytes = 144;
constexpr int kMaxSkippedStateResponsesBeforeMotionStatus = 3;

uint16_t requestCommandWord(const std::vector<uint8_t>& request)
{
    if (request.size() < 4) {
        return 0;
    }
    return static_cast<uint16_t>((static_cast<uint16_t>(request[2]) << 8) | request[3]);
}

uint32_t readUInt32LittleEndian(const std::vector<uint8_t>& bytes, std::size_t offset)
{
    return static_cast<uint32_t>(bytes[offset]) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
           (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}

uint16_t readUInt16LittleEndian(const std::vector<uint8_t>& bytes, std::size_t offset)
{
    return static_cast<uint16_t>(bytes[offset]) |
           (static_cast<uint16_t>(bytes[offset + 1]) << 8);
}

uint32_t readUInt32BigEndian(const std::vector<uint8_t>& bytes, std::size_t offset)
{
    return (static_cast<uint32_t>(bytes[offset]) << 24) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 16) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 8) |
           static_cast<uint32_t>(bytes[offset + 3]);
}

float readFloatLittleEndian(const std::vector<uint8_t>& bytes, std::size_t offset)
{
    const uint32_t raw = readUInt32LittleEndian(bytes, offset);
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

bool isTcpMotionStatusRequest(const std::vector<uint8_t>& request, int expected_response_bytes)
{
    const uint16_t command_word = requestCommandWord(request);
    return expected_response_bytes == static_cast<int>(kCabinMotionResponseBytes) &&
           (command_word == 0x0011 || command_word == 0x0012 || command_word == 0x0013);
}

uint16_t computeChecksum(const std::vector<uint8_t>& frame, std::size_t payload_size)
{
    uint16_t checksum = 0;
    for (std::size_t index = 0; index < payload_size && index < frame.size(); ++index) {
        checksum = static_cast<uint16_t>(checksum + frame[index]);
    }
    return checksum;
}

uint32_t knownTcpMotionStatusMask(uint16_t command_word)
{
    if (command_word == 0x0013) {
        return 0x0000000F;
    }
    return 0x0001FFFF;
}

uint32_t extractTcpMotionStatusWord(uint16_t command_word, const std::vector<uint8_t>& response)
{
    if (command_word == 0x0012) {
        return readUInt32BigEndian(response, 2);
    }
    return readUInt32LittleEndian(response, 2);
}

bool isValidTcpMotionStatusResponse(uint16_t command_word, const std::vector<uint8_t>& response)
{
    if (response.size() != kCabinMotionResponseBytes ||
        response[1] != 0x90 ||
        (response[0] != 0xEB && !(command_word == 0x0013 && response[0] == 0xEF))) {
        return false;
    }

    const uint16_t actual_checksum = readUInt16LittleEndian(response, response.size() - 2);
    const uint16_t expected_checksum = computeChecksum(response, response.size() - 2);
    if (actual_checksum != expected_checksum) {
        return false;
    }

    const uint32_t status_word = extractTcpMotionStatusWord(command_word, response);
    return (status_word & ~knownTcpMotionStatusMask(command_word)) == 0U;
}

bool looksLikeCabinStateResponsePrefix(const std::vector<uint8_t>& response)
{
    if (response.size() != kCabinMotionResponseBytes ||
        response[0] != 0xEB ||
        response[1] != 0x90) {
        return false;
    }

    const uint32_t raw_status_like_value = readUInt32LittleEndian(response, 2);
    if ((raw_status_like_value & ~0x0001FFFFU) == 0U) {
        return false;
    }

    const float x_mm = readFloatLittleEndian(response, 2);
    return std::isfinite(x_mm) && std::fabs(x_mm) < 100000.0f;
}

std::string formatRequestContext(const std::vector<uint8_t>& request)
{
    std::ostringstream stream;
    stream << "request_command=";
    if (request.size() >= 4) {
        const uint16_t command_word = requestCommandWord(request);
        stream << "0x"
               << std::uppercase
               << std::hex
               << std::setw(4)
               << std::setfill('0')
               << command_word
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

std::string appendRequestContext(const std::string& detail, const std::vector<uint8_t>& request)
{
    if (detail.empty()) {
        return formatRequestContext(request);
    }
    return detail + "，" + formatRequestContext(request);
}

}  // namespace

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

    if (isTcpMotionStatusRequest(request, expected_response_bytes)) {
        drainPendingInputLocked();
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
                appendRequestContext(std::strerror(errno), request),
                true,
                error
            );
        }
        total_sent += static_cast<std::size_t>(sent);
    }

    const std::size_t expected_bytes = static_cast<std::size_t>(
        expected_response_bytes > 0 ? expected_response_bytes : 256
    );

    for (int skipped_state_response_count = 0;
         skipped_state_response_count <= kMaxSkippedStateResponsesBeforeMotionStatus;
         ++skipped_state_response_count) {
        std::vector<uint8_t> local_response;
        if (!receiveExactLocked(expected_bytes, &local_response, request, "read response", error)) {
            return false;
        }

        const uint16_t command_word = requestCommandWord(request);
        if (isTcpMotionStatusRequest(request, expected_response_bytes) &&
            !isValidTcpMotionStatusResponse(command_word, local_response) &&
            looksLikeCabinStateResponsePrefix(local_response)) {
            std::vector<uint8_t> discarded_state_tail;
            if (!receiveExactLocked(
                    kCabinStateResponseBytes - kCabinMotionResponseBytes,
                    &discarded_state_tail,
                    request,
                    "discard stale cabin state response tail",
                    error)) {
                return false;
            }
            continue;
        }

        if (response != nullptr) {
            *response = local_response;
        }
        connection_state_.store(ConnectionState::kReady);
        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    connection_state_.store(ConnectionState::kReconnecting);
    return setError(
        "tcp_stale_state_response_loop",
        "索驱 TCP 运动回包连续读到状态包",
        appendRequestContext("连续跳过状态查询回包后仍未读到运动状态回包", request),
        true,
        error
    );
}

void CabinTcpTransport::markExternalIoSuccess()
{
    connection_state_.store(ConnectionState::kReady);
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_text_.clear();
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

std::size_t CabinTcpTransport::drainPendingInputLocked()
{
    if (socket_fd_ < 0) {
        return 0;
    }

    std::size_t drained_bytes = 0;
    uint8_t buffer[256];
    while (true) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_fd_, &read_fds);
        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        const int select_result = ::select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (select_result <= 0) {
            break;
        }

        const ssize_t recv_len = ::recv(socket_fd_, buffer, sizeof(buffer), MSG_DONTWAIT);
        if (recv_len <= 0) {
            break;
        }
        drained_bytes += static_cast<std::size_t>(recv_len);
    }
    return drained_bytes;
}

bool CabinTcpTransport::receiveExactLocked(
    std::size_t expected_bytes,
    std::vector<uint8_t>* response,
    const std::vector<uint8_t>& request,
    const std::string& phase,
    DriverError* error)
{
    std::vector<uint8_t> local_response(expected_bytes);
    std::size_t total_recv = 0;
    while (total_recv < expected_bytes) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_fd_, &read_fds);
        timeval timeout;
        timeout.tv_sec = timeout_sec_;
        timeout.tv_usec = 0;
        const int select_result = ::select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (select_result <= 0) {
            connection_state_.store(ConnectionState::kReconnecting);
            const std::string detail = phase + ": " +
                (select_result == 0 ? "read timeout" : std::strerror(errno)) +
                "，received=" + std::to_string(total_recv) +
                "/" + std::to_string(expected_bytes);
            return setError(
                "tcp_read_wait_failed",
                "索驱 TCP 等待回包失败",
                appendRequestContext(detail, request),
                true,
                error
            );
        }

        const ssize_t recv_len = ::recv(
            socket_fd_,
            local_response.data() + total_recv,
            expected_bytes - total_recv,
            0
        );
        if (recv_len <= 0) {
            connection_state_.store(ConnectionState::kReconnecting);
            const std::string detail = phase + ": " +
                (recv_len == 0 ? "connection closed by peer" : std::strerror(errno)) +
                "，received=" + std::to_string(total_recv) +
                "/" + std::to_string(expected_bytes);
            return setError(
                "tcp_recv_failed",
                "索驱 TCP 接收失败",
                appendRequestContext(detail, request),
                true,
                error
            );
        }
        total_recv += static_cast<std::size_t>(recv_len);
    }

    if (response != nullptr) {
        *response = local_response;
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
