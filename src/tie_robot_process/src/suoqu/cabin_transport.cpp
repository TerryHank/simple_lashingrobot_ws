#include "tie_robot_process/suoqu/cabin_transport.hpp"

#include "common.hpp"

#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <sstream>

namespace tie_robot_process {
namespace suoqu {

void cache_pending_tcp_status_error(uint16_t status_word)
{
    ::pending_tcp_status_word.store(status_word, std::memory_order_relaxed);
    ::pending_tcp_status_word_valid.store(true, std::memory_order_release);
}

bool consume_pending_tcp_status_error(uint16_t& status_word)
{
    const bool had_pending_error =
        ::pending_tcp_status_word_valid.exchange(false, std::memory_order_acq_rel);
    if (!had_pending_error) {
        status_word = 0;
        return false;
    }
    status_word = ::pending_tcp_status_word.exchange(0, std::memory_order_acq_rel);
    return status_word != 0;
}

bool is_motion_move_command_frame(const uint8_t* control_word, int tlen)
{
    if (control_word == nullptr || tlen != 36) {
        return false;
    }
    return control_word[0] == 0xEB &&
           control_word[1] == 0x90 &&
           control_word[2] == 0x00 &&
           control_word[3] == 0x12;
}

const char* tcp_protocol_command_name(uint16_t command_word)
{
    switch (command_word) {
        case 0x0002: return "电机使能控制";
        case 0x0003: return "电机复位控制";
        case 0x0004: return "运动到关节零点启动";
        case 0x0005: return "运动到关节零点停止";
        case 0x0006: return "逆解计算激活";
        case 0x0007: return "TCP复位";
        case 0x0010: return "TCP连续运动";
        case 0x0011: return "TCP增量运动";
        case 0x0012: return "TCP位置运动";
        case 0x0013: return "TCP位置运动停止";
        default: return "未知索驱指令";
    }
}

namespace {

void append_tcp_status_reason_if_set(
    std::vector<std::string>& reasons,
    uint16_t status_word,
    int bit_index,
    const char* reason)
{
    if ((status_word & (static_cast<uint16_t>(1u) << bit_index)) != 0) {
        reasons.emplace_back(reason);
    }
}

}  // namespace

std::vector<std::string> decode_tcp_protocol_status_reasons(uint16_t command_word, uint16_t status_word)
{
    std::vector<std::string> reasons;
    switch (command_word) {
        case 0x0004:
            append_tcp_status_reason_if_set(reasons, status_word, 0, "逆解激活");
            append_tcp_status_reason_if_set(reasons, status_word, 1, "电机未全部使能");
            append_tcp_status_reason_if_set(reasons, status_word, 2, "设备运动中");
            append_tcp_status_reason_if_set(reasons, status_word, 3, "设备报警");
            break;
        case 0x0005:
            append_tcp_status_reason_if_set(reasons, status_word, 0, "逆解激活");
            append_tcp_status_reason_if_set(reasons, status_word, 1, "电机未全部使能");
            append_tcp_status_reason_if_set(reasons, status_word, 2, "设备未运动");
            append_tcp_status_reason_if_set(reasons, status_word, 3, "设备报警");
            break;
        case 0x0006:
            append_tcp_status_reason_if_set(reasons, status_word, 0, "逆解激活");
            append_tcp_status_reason_if_set(reasons, status_word, 1, "电机轴不在零点");
            append_tcp_status_reason_if_set(reasons, status_word, 2, "TCP不在零点");
            break;
        case 0x0007:
            append_tcp_status_reason_if_set(reasons, status_word, 0, "逆解激活");
            append_tcp_status_reason_if_set(reasons, status_word, 1, "设备运动中");
            break;
        case 0x0010:
        case 0x0011:
        case 0x0012:
            append_tcp_status_reason_if_set(reasons, status_word, 0, "逆解未激活");
            append_tcp_status_reason_if_set(reasons, status_word, 1, "电机未全部使能");
            append_tcp_status_reason_if_set(reasons, status_word, 2, "设备运动中");
            append_tcp_status_reason_if_set(reasons, status_word, 3, "设备报警");
            append_tcp_status_reason_if_set(reasons, status_word, 4, "速度错误");
            append_tcp_status_reason_if_set(reasons, status_word, 5, "X超正限位");
            append_tcp_status_reason_if_set(reasons, status_word, 6, "X超负限位");
            append_tcp_status_reason_if_set(reasons, status_word, 7, "Y超正限位");
            append_tcp_status_reason_if_set(reasons, status_word, 8, "Y超负限位");
            append_tcp_status_reason_if_set(reasons, status_word, 9, "Z超正限位");
            append_tcp_status_reason_if_set(reasons, status_word, 10, "Z超负限位");
            append_tcp_status_reason_if_set(reasons, status_word, 11, "A超正限位");
            append_tcp_status_reason_if_set(reasons, status_word, 12, "A超负限位");
            append_tcp_status_reason_if_set(reasons, status_word, 13, "B超正限位");
            append_tcp_status_reason_if_set(reasons, status_word, 14, "B超负限位");
            append_tcp_status_reason_if_set(reasons, status_word, 15, "C超正限位");
            break;
        case 0x0013:
            append_tcp_status_reason_if_set(reasons, status_word, 0, "逆解未激活");
            append_tcp_status_reason_if_set(reasons, status_word, 1, "电机未全部使能");
            append_tcp_status_reason_if_set(reasons, status_word, 2, "设备未运动");
            append_tcp_status_reason_if_set(reasons, status_word, 3, "设备报警");
            break;
        default:
            break;
    }
    return reasons;
}

uint16_t extract_tcp_protocol_status_word(
    uint16_t command_word,
    const uint8_t* buffer,
    ssize_t recv_len)
{
    if (buffer == nullptr || recv_len < 4) {
        return 0;
    }

    switch (command_word) {
        case 0x0004:
        case 0x0005:
        case 0x0006:
        case 0x0007:
            return static_cast<uint16_t>(buffer[2]) |
                   (static_cast<uint16_t>(buffer[3]) << 8);
        case 0x0010:
        case 0x0011:
        case 0x0012:
        case 0x0013:
            if (recv_len >= 6) {
                return static_cast<uint16_t>(buffer[4]) |
                       (static_cast<uint16_t>(buffer[5]) << 8);
            }
            return 0;
        default:
            if (recv_len >= 6) {
                return static_cast<uint16_t>(buffer[4]) |
                       (static_cast<uint16_t>(buffer[5]) << 8);
            }
            return static_cast<uint16_t>(buffer[2]) |
                   (static_cast<uint16_t>(buffer[3]) << 8);
    }
}

TcpProtocolStatusDecode decode_tcp_protocol_status(
    uint16_t command_word,
    const uint8_t* buffer,
    ssize_t recv_len)
{
    TcpProtocolStatusDecode decoded_status;
    decoded_status.command_word = command_word;
    decoded_status.command_name = tcp_protocol_command_name(command_word);

    if (buffer == nullptr || recv_len < 4) {
        return decoded_status;
    }

    if ((command_word == 0x0002 || command_word == 0x0003) && recv_len >= 18) {
        static const char* const kMotorStatusReasons[] = {
            "设备运动中",
            "电机运动中",
            "电机初始化未完成",
            "电机报警",
        };
        for (int motor_index = 0; motor_index < 8; ++motor_index) {
            const int base = 2 + motor_index * 2;
            const uint16_t motor_status =
                static_cast<uint16_t>(buffer[base]) |
                (static_cast<uint16_t>(buffer[base + 1]) << 8);
            if (motor_status == 0) {
                continue;
            }
            if (decoded_status.status_word == 0) {
                decoded_status.status_word = motor_status;
            }
            decoded_status.has_error = true;
            std::vector<std::string> motor_reasons;
            for (int bit_index = 0; bit_index < 4; ++bit_index) {
                if ((motor_status & (static_cast<uint16_t>(1u) << bit_index)) != 0) {
                    motor_reasons.emplace_back(kMotorStatusReasons[bit_index]);
                }
            }
            if (motor_reasons.empty()) {
                std::ostringstream reason_stream;
                reason_stream << "电机" << (motor_index + 1) << ": 未知状态0x"
                              << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
                              << motor_status;
                decoded_status.reasons.push_back(reason_stream.str());
            } else {
                std::ostringstream reason_stream;
                reason_stream << "电机" << (motor_index + 1) << ": ";
                for (size_t reason_index = 0; reason_index < motor_reasons.size(); ++reason_index) {
                    if (reason_index > 0) {
                        reason_stream << "、";
                    }
                    reason_stream << motor_reasons[reason_index];
                }
                decoded_status.reasons.push_back(reason_stream.str());
            }
        }
        return decoded_status;
    }

    decoded_status.status_word = extract_tcp_protocol_status_word(command_word, buffer, recv_len);
    decoded_status.has_error = decoded_status.status_word != 0;
    if (decoded_status.has_error) {
        decoded_status.reasons = decode_tcp_protocol_status_reasons(command_word, decoded_status.status_word);
        if (decoded_status.reasons.empty()) {
            std::ostringstream reason_stream;
            reason_stream << "未在协议表中找到状态字0x"
                          << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
                          << decoded_status.status_word << "的中文映射";
            decoded_status.reasons.push_back(reason_stream.str());
        }
    }
    return decoded_status;
}

std::string format_tcp_protocol_status_message(const TcpProtocolStatusDecode& decoded_status)
{
    std::ostringstream message_stream;
    message_stream << "指令=" << decoded_status.command_name << "(0x"
                   << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
                   << decoded_status.command_word << ")"
                   << "，状态字=0x" << std::setw(4) << decoded_status.status_word;
    if (!decoded_status.reasons.empty()) {
        message_stream << "，含义=";
        for (size_t reason_index = 0; reason_index < decoded_status.reasons.size(); ++reason_index) {
            if (reason_index > 0) {
                message_stream << "；";
            }
            message_stream << decoded_status.reasons[reason_index];
        }
    }
    return message_stream.str();
}

std::string build_tcp_frame_hex_string(const uint8_t* data, size_t len)
{
    if (data == nullptr || len == 0) {
        return "";
    }
    std::ostringstream oss;
    for (size_t i = 0; i < len; ++i) {
        if (i > 0) {
            oss << ' ';
        }
        oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(data[i]);
    }
    return oss.str();
}

std::string build_tcp_command_debug_context(uint16_t command_word, const uint8_t* control_word, int tlen)
{
    std::ostringstream oss;
    oss << "指令=" << tcp_protocol_command_name(command_word)
        << "(0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
        << command_word << std::dec << ")";
    if (control_word != nullptr && tlen > 0) {
        oss << "，frame=[" << build_tcp_frame_hex_string(control_word, static_cast<size_t>(tlen)) << "]";
    }
    return oss.str();
}

void update_last_cabin_transport_error_detail(const std::string& detail)
{
    std::lock_guard<std::mutex> lock(cabin_last_error_detail_mutex);
    ::last_cabin_transport_error_detail = detail;
}

void clear_last_cabin_transport_error_detail()
{
    std::lock_guard<std::mutex> lock(cabin_last_error_detail_mutex);
    ::last_cabin_transport_error_detail.clear();
}

void set_last_execution_wait_error_detail(const std::string& detail)
{
    std::lock_guard<std::mutex> lock(cabin_last_error_detail_mutex);
    ::last_cabin_execution_wait_error_detail = detail;
}

void clear_last_execution_wait_error_detail()
{
    std::lock_guard<std::mutex> lock(cabin_last_error_detail_mutex);
    ::last_cabin_execution_wait_error_detail.clear();
}

std::string get_last_cabin_failure_detail()
{
    std::lock_guard<std::mutex> lock(cabin_last_error_detail_mutex);
    if (!::last_cabin_transport_error_detail.empty()) {
        return ::last_cabin_transport_error_detail;
    }
    return ::last_cabin_execution_wait_error_detail;
}

void persist_last_cabin_fatal_error_detail(const std::string& detail)
{
    {
        std::lock_guard<std::mutex> lock(cabin_last_error_detail_mutex);
        ::last_cabin_transport_error_detail = detail;
    }
    std::ofstream outfile(::kCabinLastFatalErrorDetailFile, std::ios::trunc);
    if (outfile.is_open()) {
        outfile << detail;
    }
}

void clear_last_cabin_fatal_error_detail()
{
    clear_last_cabin_transport_error_detail();
    clear_last_execution_wait_error_detail();
    std::ofstream outfile(::kCabinLastFatalErrorDetailFile, std::ios::trunc);
}

std::string compose_cabin_failure_message(const std::string& prefix)
{
    const std::string failure_detail = get_last_cabin_failure_detail();
    if (failure_detail.empty()) {
        return prefix;
    }
    return prefix + "。最近一次底层错误：" + failure_detail;
}

void log_cabin_error_ros(const std::string& detail)
{
    if (!detail.empty()) {
        ROS_ERROR("Cabin_Error: %s", detail.c_str());
    }
}

void log_cabin_warn_ros(const std::string& detail)
{
    if (!detail.empty()) {
        ROS_WARN("Cabin_Warn: %s", detail.c_str());
    }
}

std::string compose_cabin_driver_error_message(
    const std::string& prefix,
    const tie_robot_hw::driver::DriverError& driver_error)
{
    std::string detail = driver_error.message;
    if (!driver_error.detail.empty()) {
        detail += "，detail=" + driver_error.detail;
    }
    if (detail.empty() && ::g_cabin_driver) {
        detail = ::g_cabin_driver->lastErrorText();
    }
    if (detail.empty()) {
        return prefix;
    }
    return prefix + "，" + detail;
}

void sync_global_socket_fd_from_cabin_driver()
{
    ::sockfd = ::g_cabin_driver ? ::g_cabin_driver->socketFd() : -1;
}

bool stop_cabin_motion_via_driver(std::string* error_message)
{
    if (!::g_cabin_driver) {
        const std::string detail = "索驱驱动未初始化，无法下发停止指令";
        update_last_cabin_transport_error_detail(detail);
        log_cabin_error_ros(detail);
        if (error_message != nullptr) {
            *error_message = detail;
        }
        return false;
    }

    tie_robot_hw::driver::DriverError driver_error;
    {
        std::lock_guard<std::mutex> lock2(::socket_mutex);
        const bool stop_ok = ::g_cabin_driver->sendStop(&driver_error);
        sync_global_socket_fd_from_cabin_driver();
        if (!stop_ok) {
            const std::string detail = compose_cabin_driver_error_message("索驱停止指令下发失败", driver_error);
            update_last_cabin_transport_error_detail(detail);
            printCurrentTime();
            std::printf("Cabin_Error: %s\n", detail.c_str());
            log_cabin_error_ros(detail);
            if (error_message != nullptr) {
                *error_message = detail;
            }
            return false;
        }
    }

    clear_last_cabin_transport_error_detail();
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool move_cabin_pose_via_driver(
    float speed_mm_per_sec,
    float x_mm,
    float y_mm,
    float z_mm,
    std::string* error_message)
{
    if (!::g_cabin_driver) {
        const std::string detail = "索驱驱动未初始化，无法下发位姿运动指令";
        update_last_cabin_transport_error_detail(detail);
        log_cabin_error_ros(detail);
        if (error_message != nullptr) {
            *error_message = detail;
        }
        return false;
    }

    tie_robot_hw::driver::DriverError driver_error;
    tie_robot_hw::driver::CabinPoseCommand command;
    command.speed_mm_per_sec = speed_mm_per_sec;
    command.x_mm = x_mm;
    command.y_mm = y_mm;
    command.z_mm = z_mm;
    ::TCP_Move[0] = speed_mm_per_sec;
    ::TCP_Move[1] = x_mm;
    ::TCP_Move[2] = y_mm;
    ::TCP_Move[3] = z_mm;

    {
        std::lock_guard<std::mutex> lock2(::socket_mutex);
        const bool move_ok = ::g_cabin_driver->moveToPose(command, &driver_error);
        sync_global_socket_fd_from_cabin_driver();
        if (!move_ok) {
            const std::string detail = compose_cabin_driver_error_message("索驱位姿运动驱动下发失败", driver_error);
            update_last_cabin_transport_error_detail(detail);
            printCurrentTime();
            std::printf("Cabin_Error: %s\n", detail.c_str());
            log_cabin_error_ros(detail);
            if (error_message != nullptr) {
                *error_message = detail;
            }
            return false;
        }
    }

    clear_last_cabin_transport_error_detail();
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

}  // namespace suoqu
}  // namespace tie_robot_process
