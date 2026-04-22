#pragma once

#include <string>
#include <vector>

namespace tie_robot_hw {
namespace driver {

enum class ConnectionState {
    kDisconnected = 0,
    kConnecting = 1,
    kReady = 2,
    kReconnecting = 3,
    kFault = 4,
};

struct DriverError {
    std::string code;
    std::string message;
    std::string detail;
    bool retryable = false;

    void clear()
    {
        code.clear();
        message.clear();
        detail.clear();
        retryable = false;
    }
};

struct CabinPoseCommand {
    float speed_mm_per_sec = 0.0f;
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
};

struct CabinStateSnapshot {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    float yaw_deg = 0.0f;
    float speed_mm_per_sec = 0.0f;
    int motion_status = 0;
    int internal_calc_error = 0;
    int device_alarm = 0;
    bool connected = false;
};

struct LinearModulePoint {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    float angle_deg = 0.0f;
};

struct LinearModuleStateSnapshot {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    float speed_x_mm_per_sec = 0.0f;
    float speed_y_mm_per_sec = 0.0f;
    float speed_z_mm_per_sec = 0.0f;
    float motor_angle_deg = 0.0f;
    float motor_speed_deg_per_sec = 0.0f;
    int finish_all = 0;
    int arrive_z = 0;
    int error_x = 0;
    int error_y = 0;
    int error_z = 0;
    int error_lashing = 0;
    int error_motor = 0;
    bool connected = false;
};

}  // namespace driver
}  // namespace tie_robot_hw
