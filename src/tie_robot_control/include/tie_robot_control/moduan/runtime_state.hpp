#ifndef TIE_ROBOT_CONTROL_MODUAN_RUNTIME_STATE_HPP
#define TIE_ROBOT_CONTROL_MODUAN_RUNTIME_STATE_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <modbus.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>

#include "tie_robot_hw/driver/linear_module_driver.hpp"
#include "tie_robot_msgs/ProcessImage.h"
#include "tie_robot_msgs/linear_module_upload.h"
#include "tie_robot_msgs/motion.h"

struct Module_State {
    double X;
    double Y;
    double Z;
    double X_SPEED;
    double Y_SPEED;
    double Z_SPEED;
    uint16_t ERROR_FLAG_X;
    uint16_t ERROR_FLAG_Y;
    uint16_t ERROR_FLAG_Z;
    uint16_t ERROR_FLAG_LASHING;
    float x_gesture;
    float y_gesture;
    uint16_t JULI;
    uint16_t ARRIVEZ_FLAG;
    uint16_t FINISH_ALL_FLAG;
};

struct Motor_State {
    double MOTOR_ANGLE;
    double MOTOR_SPEED;
    uint16_t ERROR_FLAG_MOTOR;
};

extern modbus_t* plc;
extern bool light_state;
extern std::atomic<bool> error_detected;
extern std::mutex error_msg_mutex;
extern std::string last_error_msg;

extern tie_robot_msgs::motion transform_msg;
extern tie_robot_msgs::linear_module_upload linear_module_data_upload;
extern tie_robot_msgs::ProcessImage srv;
extern ros::ServiceClient AI_client;
extern ros::ServiceClient linear_client;
extern std_srvs::Trigger Trigger_srv;
extern ros::ServiceClient trigger_client;

extern double left_motor_speed;
extern double right_motor_speed;
extern double gain_speed;
extern int channel1_percent;
extern int channel2_percent;
extern int channel3_percent;
extern int state;
extern int send_odd_points;
extern int fd;
extern uint8_t data;
extern ros::Publisher pub_ctl_vel;
extern ros::Publisher pub_wheel_direction;
extern std_msgs::Int32 msg_state;
extern std_msgs::Float32 msg_vel;
extern uint16_t UtoF_Register[2];
extern uint16_t DtoU_Register[2];
extern uint16_t UtoD_Register[2];
extern uint16_t last_JULI;
extern double module_speed;
extern double motor_speed;
extern double current_z;
extern bool current_z_flag;
extern std_msgs::Int32 msg;
extern double reset_angle;
extern double last_left_motor_speed;
extern double last_right_motor_speed;
extern ros::Publisher pub_motor_theata;
extern ros::Publisher pub_linear_module_gb_origin;
extern ros::Publisher pub_forced_stop;
extern ros::Publisher pub_moduan_warning;
extern ros::Publisher pub_lashing_warning;
extern ros::Publisher pub_moduan_work;
extern ros::Publisher pub_pause;
extern ros::Publisher pub_linear_module_data_upload;
extern int pause_interrupt;
extern std::mutex module_state_mutex;
extern std::mutex plc_mutex;
extern uint16_t MODULE;
extern Module_State module_state;
extern Motor_State motor_state;
extern bool enable_lashing;
extern std::mutex lashing_mutex;
extern ros::Subscriber enb_las_sub;
extern int pause_interrupt_count;
extern bool handle_pause_interrupt;
extern std::pair<std::vector<int>, std::vector<float>> bind_data;
extern std::vector<std::pair<std::vector<int>, std::vector<float>>> bind_all_data;
extern std::unique_ptr<tie_robot_hw::driver::LinearModuleDriver> g_linear_module_driver;
extern float last_x;
extern float last_y;

constexpr uint8_t kProcessImageModeAdaptiveHeight = 1;
constexpr uint8_t kProcessImageModeBindCheck = 2;
constexpr double kBindMaxHeightMm = 95.0;
constexpr double kTcpTravelMinZMm = 0.0;
constexpr double kTcpTravelMaxZMm = 140.0;
constexpr double kTravelMaxXMm = 360.0;
constexpr double kTravelMaxYMm = 320.0;
constexpr double kPrecomputedFastModuleSpeedMmPerSec = 400.0;
constexpr const char* kBindHeightExcessMessageKey = "BIND_HEIGHT_EXCESS_MM=";
constexpr int kFinishAllTimeoutSec = 30;
constexpr int kFinishAllLogIntervalSec = 2;

#endif
