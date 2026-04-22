#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <tie_robot_hw/driver/cabin_driver.hpp>

extern std::unique_ptr<tie_robot_hw::driver::CabinDriver> g_cabin_driver;
extern int sockfd;
extern std::mutex socket_mutex;
extern std::mutex cabin_last_error_detail_mutex;
extern std::string last_cabin_transport_error_detail;
extern std::string last_cabin_execution_wait_error_detail;
extern const std::string kCabinLastFatalErrorDetailFile;
extern std::atomic<uint16_t> pending_tcp_status_word;
extern std::atomic<bool> pending_tcp_status_word_valid;
extern float TCP_Move[7];

namespace tie_robot_process {
namespace suoqu {

struct TcpProtocolStatusDecode
{
    uint16_t command_word = 0;
    uint16_t status_word = 0;
    std::string command_name;
    std::vector<std::string> reasons;
    bool has_error = false;
};

void cache_pending_tcp_status_error(uint16_t status_word);
bool consume_pending_tcp_status_error(uint16_t& status_word);
bool is_motion_move_command_frame(const uint8_t* control_word, int tlen);
const char* tcp_protocol_command_name(uint16_t command_word);
std::vector<std::string> decode_tcp_protocol_status_reasons(uint16_t command_word, uint16_t status_word);
uint16_t extract_tcp_protocol_status_word(uint16_t command_word, const uint8_t* buffer, ssize_t recv_len);
TcpProtocolStatusDecode decode_tcp_protocol_status(
    uint16_t command_word,
    const uint8_t* buffer,
    ssize_t recv_len);
std::string format_tcp_protocol_status_message(const TcpProtocolStatusDecode& decoded_status);
std::string build_tcp_frame_hex_string(const uint8_t* data, size_t len);
std::string build_tcp_command_debug_context(uint16_t command_word, const uint8_t* control_word, int tlen);
void update_last_cabin_transport_error_detail(const std::string& detail);
void clear_last_cabin_transport_error_detail();
void set_last_execution_wait_error_detail(const std::string& detail);
void clear_last_execution_wait_error_detail();
std::string get_last_cabin_failure_detail();
void persist_last_cabin_fatal_error_detail(const std::string& detail);
void clear_last_cabin_fatal_error_detail();
std::string compose_cabin_failure_message(const std::string& prefix);
void log_cabin_error_ros(const std::string& detail);
void log_cabin_warn_ros(const std::string& detail);
std::string compose_cabin_driver_error_message(
    const std::string& prefix,
    const tie_robot_hw::driver::DriverError& driver_error);
void sync_global_socket_fd_from_cabin_driver();
bool stop_cabin_motion_via_driver(std::string* error_message);
bool move_cabin_pose_via_driver(
    float speed_mm_per_sec,
    float x_mm,
    float y_mm,
    float z_mm,
    std::string* error_message);

}  // namespace suoqu
}  // namespace tie_robot_process
