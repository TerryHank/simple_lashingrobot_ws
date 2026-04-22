#ifndef TIE_ROBOT_CONTROL_MODUAN_ROS_CALLBACKS_HPP
#define TIE_ROBOT_CONTROL_MODUAN_ROS_CALLBACKS_HPP

#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

#include "tie_robot_msgs/ExecuteBindPoints.h"
#include "tie_robot_msgs/linear_module_move.h"
#include "tie_robot_control/moduan/runtime_state.hpp"

void pub_moduan_state(
    ros::Publisher& pub_linear_module_data_upload,
    Module_State* state,
    Motor_State* mot_state,
    float robot_battery_voltage,
    float robot_temperature);
void pub_moduan_work_state(bool moduan_work_flag);
void enable_lashing_callback(const std_msgs::Float32::ConstPtr& msg);
void pause_interrupt_Callback(const std_msgs::Float32& debug_mes);
bool moduan_bind_service(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
bool moduan_bind_points_service(
    tie_robot_msgs::ExecuteBindPoints::Request& req,
    tie_robot_msgs::ExecuteBindPoints::Response& res);
bool moduan_bind_points_fast_service(
    tie_robot_msgs::ExecuteBindPoints::Request& req,
    tie_robot_msgs::ExecuteBindPoints::Response& res);
void forced_stop_nodeCallback(const std_msgs::Float32& debug_mes);
void request_moduan_zero(const char* reason);
void moduan_move_zero_forthread(double x, double y, double z, double angle);
void moduan_move_zero_callback(const std_msgs::Float32::ConstPtr& msg);
bool moduan_move_service(
    tie_robot_msgs::linear_module_move::Request& req,
    tie_robot_msgs::linear_module_move::Response& res);
void light_switch(const std_msgs::Bool& debug_mes);
void send_odd_points_callback(const std_msgs::Bool& debug_mes);
void change_speed_callback(const std_msgs::Float32& debug_mes);
void handSolveWarnCallback(const std_msgs::Float32& warn_msg);
void read_module_motor_state(Module_State* state, Motor_State* mot_state);
std::string getBeijingTimeString();
void robotSaveBindingDataCallback(const std_msgs::Float32::ConstPtr& msg);
void initPLC();
void auto_zero_on_startup(ros::NodeHandle& private_nh);
int RunModuanNode(int argc, char** argv);

#endif
