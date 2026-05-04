#include "tie_robot_control/moduan/moduan_ros_callbacks.hpp"

#include <chrono>
#include <clocale>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tie_robot_control/common.hpp>
#include "tie_robot_msgs/ExecuteBindPointsTaskAction.h"
#include "tie_robot_msgs/ModuanState.h"
#include "tie_robot_control/moduan/error_handling.hpp"
#include "tie_robot_control/moduan/linear_module_executor.hpp"
#include "tie_robot_control/moduan/numeric_codec.hpp"
#include "tie_robot_control/moduan/register_map.hpp"
#include "tie_robot_control/moduan/runtime_state.hpp"

using namespace tie_robot_control::moduan_registers;

namespace {

constexpr const char* kModuanDiagnosticHardwareId = "tie_robot/moduan_driver";
constexpr const char* kScepterDepthFrame = "Scepter_depth_frame";
constexpr const char* kGripperFrame = "gripper_frame";
std::unique_ptr<diagnostic_updater::Updater> g_moduan_diagnostic_updater;
using ExecuteBindPointsActionServer =
    actionlib::SimpleActionServer<tie_robot_msgs::ExecuteBindPointsTaskAction>;
std::unique_ptr<ExecuteBindPointsActionServer> g_execute_bind_points_action_server;
std::unique_ptr<tf2_ros::Buffer> g_tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> g_tf_listener;

const char* connection_state_label(tie_robot_hw::driver::ConnectionState state)
{
    switch (state) {
        case tie_robot_hw::driver::ConnectionState::kConnecting:
            return "connecting";
        case tie_robot_hw::driver::ConnectionState::kReady:
            return "ready";
        case tie_robot_hw::driver::ConnectionState::kReconnecting:
            return "reconnecting";
        case tie_robot_hw::driver::ConnectionState::kFault:
            return "fault";
        case tie_robot_hw::driver::ConnectionState::kDisconnected:
        default:
            return "disconnected";
    }
}

bool start_moduan_driver_connection(std::string* detail)
{
    g_moduan_driver_enabled.store(true);
    if (!g_linear_module_driver) {
        g_linear_module_driver = std::make_unique<tie_robot_hw::driver::LinearModuleDriver>();
    }

    tie_robot_hw::driver::DriverError driver_error;
    const bool wrapper_ok = g_linear_module_driver->start(&driver_error);

    {
        std::lock_guard<std::mutex> lock(plc_mutex);
        if (plc == nullptr) {
            plc = PLC_Connection();
        }
    }

    if (!wrapper_ok || plc == nullptr) {
        std::ostringstream oss;
        oss << "末端驱动启动失败";
        if (!driver_error.message.empty()) {
            oss << "，" << driver_error.message;
        }
        if (!driver_error.detail.empty()) {
            oss << "，detail=" << driver_error.detail;
        }
        if (plc == nullptr) {
            oss << "，PLC上下文不可用";
        }
        const std::string message = oss.str();
        {
            std::lock_guard<std::mutex> lock(error_msg_mutex);
            last_error_msg = message;
        }
        if (detail != nullptr) {
            *detail = message;
        }
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(error_msg_mutex);
        last_error_msg.clear();
    }
    if (detail != nullptr) {
        detail->clear();
    }
    return true;
}

void stop_moduan_driver_connection()
{
    g_moduan_driver_enabled.store(false);
    if (g_linear_module_driver) {
        g_linear_module_driver->stop();
    }
    std::lock_guard<std::mutex> lock(plc_mutex);
    if (plc != nullptr) {
        modbus_close(plc);
        modbus_free(plc);
        plc = nullptr;
    }
}

void produce_moduan_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    const bool enabled = g_moduan_driver_enabled.load();
    const bool plc_connected = plc != nullptr;
    const auto transport_state = g_linear_module_driver
        ? g_linear_module_driver->connectionState()
        : tie_robot_hw::driver::ConnectionState::kDisconnected;
    const std::string transport_error = g_linear_module_driver ? g_linear_module_driver->lastErrorText() : std::string();
    std::string runtime_error;
    {
        std::lock_guard<std::mutex> lock(error_msg_mutex);
        runtime_error = last_error_msg;
    }

    if (!enabled) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "末端驱动已关闭");
    } else if (transport_state == tie_robot_hw::driver::ConnectionState::kReady || plc_connected) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "末端驱动已连接");
    } else if (!runtime_error.empty() || !transport_error.empty()) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "末端驱动通信异常");
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "末端驱动未连接");
    }

    stat.hardware_id = kModuanDiagnosticHardwareId;
    stat.add("enabled", enabled ? "true" : "false");
    stat.add("plc_connected", plc_connected ? "true" : "false");
    stat.add("transport_state", connection_state_label(transport_state));
    stat.add("transport_error", transport_error);
    stat.add("runtime_error", runtime_error);
    stat.add("x_mm", module_state.X);
    stat.add("y_mm", module_state.Y);
    stat.add("z_mm", module_state.Z);
    stat.add("motor_angle_deg", motor_state.MOTOR_ANGLE);
    stat.add("finish_all", module_state.FINISH_ALL_FLAG);
    stat.add("arrive_z", module_state.ARRIVEZ_FLAG);
    stat.add("error_x", module_state.ERROR_FLAG_X);
    stat.add("error_y", module_state.ERROR_FLAG_Y);
    stat.add("error_z", module_state.ERROR_FLAG_Z);
    stat.add("error_lashing", module_state.ERROR_FLAG_LASHING);
    stat.add("error_motor", motor_state.ERROR_FLAG_MOTOR);
    stat.add("battery_voltage", linear_module_data_upload.robot_battery_voltage);
    stat.add("temperature", linear_module_data_upload.robot_temperature);
}

void moduan_diagnostic_timer_callback(const ros::TimerEvent&)
{
    if (g_moduan_diagnostic_updater) {
        g_moduan_diagnostic_updater->force_update();
    }
}

tie_robot_msgs::ExecuteBindPointsTaskFeedback make_execute_bind_points_feedback(
    const std::string& phase,
    uint32_t selected_count,
    const std::chrono::steady_clock::time_point& start_time)
{
    tie_robot_msgs::ExecuteBindPointsTaskFeedback feedback;
    feedback.phase = phase;
    feedback.selected_count = selected_count;
    {
        std::lock_guard<std::mutex> lock(module_state_mutex);
        feedback.current_x = module_state.X;
        feedback.current_y = module_state.Y;
        feedback.current_z = module_state.Z;
    }
    feedback.elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    return feedback;
}

void publish_execute_bind_points_feedback(
    const std::string& phase,
    uint32_t selected_count,
    const std::chrono::steady_clock::time_point& start_time)
{
    if (!g_execute_bind_points_action_server || !g_execute_bind_points_action_server->isActive()) {
        return;
    }
    g_execute_bind_points_action_server->publishFeedback(
        make_execute_bind_points_feedback(phase, selected_count, start_time));
}

void execute_bind_points_action_callback(
    const tie_robot_msgs::ExecuteBindPointsTaskGoalConstPtr& goal)
{
    const auto start_time = std::chrono::steady_clock::now();
    const uint32_t requested_count = static_cast<uint32_t>(goal->points.size());
    tie_robot_msgs::ExecuteBindPointsTaskResult result;

    if (!g_execute_bind_points_action_server || !g_execute_bind_points_action_server->isActive()) {
        return;
    }
    if (g_execute_bind_points_action_server->isPreemptRequested()) {
        result.success = false;
        result.message = "线性模组执行任务在启动前被取消";
        result.error_code = "preempted_before_start";
        g_execute_bind_points_action_server->setPreempted(result, result.message);
        return;
    }

    publish_execute_bind_points_feedback("accepted", requested_count, start_time);
    std::string message;
    bool executed = false;
    {
        std::lock_guard<std::mutex> lashing_lock(lashing_mutex);
        publish_execute_bind_points_feedback("executing", requested_count, start_time);
        std::vector<tie_robot_msgs::PointCoords> points(goal->points.begin(), goal->points.end());
        executed = execute_bind_points(points, message, goal->apply_jump_bind_filter);
    }

    result.success = executed;
    result.message = message;
    result.error_code = executed ? "" : "execute_bind_points_failed";
    publish_execute_bind_points_feedback(executed ? "succeeded" : "aborted", requested_count, start_time);
    if (executed) {
        g_execute_bind_points_action_server->setSucceeded(result, result.message);
    } else {
        g_execute_bind_points_action_server->setAborted(result, result.message);
    }
}

bool transform_scepter_camera_point_to_gripper_point(
    const tie_robot_msgs::PointCoords& camera_point,
    tie_robot_msgs::PointCoords& gripper_point,
    std::string* error_message)
{
    if (!g_tf_buffer) {
        if (error_message != nullptr) {
            *error_message = "TF buffer未初始化";
        }
        return false;
    }

    geometry_msgs::PointStamped stamped_camera_point;
    stamped_camera_point.header.stamp = ros::Time(0);
    stamped_camera_point.header.frame_id = kScepterDepthFrame;
    stamped_camera_point.point.x = static_cast<double>(camera_point.World_coord[0]) / 1000.0;
    stamped_camera_point.point.y = static_cast<double>(camera_point.World_coord[1]) / 1000.0;
    stamped_camera_point.point.z = static_cast<double>(camera_point.World_coord[2]) / 1000.0;

    try {
        const geometry_msgs::PointStamped stamped_gripper_point = g_tf_buffer->transform(
            stamped_camera_point,
            kGripperFrame,
            ros::Duration(0.2));
        gripper_point = camera_point;
        gripper_point.World_coord[0] = static_cast<float>(stamped_gripper_point.point.x * 1000.0);
        gripper_point.World_coord[1] = static_cast<float>(stamped_gripper_point.point.y * 1000.0);
        gripper_point.World_coord[2] = static_cast<float>(stamped_gripper_point.point.z * 1000.0);
        return true;
    } catch (const tf2::TransformException& ex) {
        if (error_message != nullptr) {
            *error_message = ex.what();
        }
        return false;
    }
}

bool transform_scepter_camera_points_to_gripper_points(
    const std::vector<tie_robot_msgs::PointCoords>& camera_points,
    std::vector<tie_robot_msgs::PointCoords>& gripper_points,
    std::string& error_message)
{
    gripper_points.clear();
    gripper_points.reserve(camera_points.size());
    for (const auto& camera_point : camera_points) {
        tie_robot_msgs::PointCoords gripper_point;
        std::string point_error;
        if (!transform_scepter_camera_point_to_gripper_point(camera_point, gripper_point, &point_error)) {
            std::ostringstream oss;
            oss << kScepterDepthFrame << "->" << kGripperFrame
                << "点坐标变换失败，idx=" << camera_point.idx;
            if (!point_error.empty()) {
                oss << "，" << point_error;
            }
            error_message = oss.str();
            return false;
        }
        gripper_points.push_back(gripper_point);
    }
    error_message.clear();
    return true;
}

}  // namespace

void pub_moduan_state(
    ros::Publisher& pub_linear_module_data_upload,
    Module_State* state,
    Motor_State* mot_state,
    float robot_battery_voltage,
    float robot_temperature)
{
    tie_robot_msgs::linear_module_upload linear_module_data_upload_msg;
    linear_module_data_upload_msg.linear_module_position_X = state->X;
    linear_module_data_upload_msg.linear_module_position_Y = state->Y;
    linear_module_data_upload_msg.linear_module_position_Z = state->Z;
    linear_module_data_upload_msg.linear_module_speed_X = state->X_SPEED;
    linear_module_data_upload_msg.linear_module_speed_Y = state->Y_SPEED;
    linear_module_data_upload_msg.linear_module_speed_Z = state->Z_SPEED;
    linear_module_data_upload_msg.linear_module_error_flag_X = state->ERROR_FLAG_X;
    linear_module_data_upload_msg.linear_module_error_flag_Y = state->ERROR_FLAG_Y;
    linear_module_data_upload_msg.linear_module_error_flag_Z = state->ERROR_FLAG_Z;
    linear_module_data_upload_msg.motor_angle = mot_state->MOTOR_ANGLE;
    linear_module_data_upload_msg.motor_speed = mot_state->MOTOR_SPEED;
    linear_module_data_upload_msg.motor_error_flag = mot_state->ERROR_FLAG_MOTOR;
    linear_module_data_upload_msg.robot_battery_voltage =
        (std::isfinite(robot_battery_voltage) && robot_battery_voltage > 0.0f && robot_battery_voltage <= 100.0f)
            ? robot_battery_voltage
            : 0.0f;
    linear_module_data_upload_msg.robot_temperature = robot_temperature;
    linear_module_data_upload_msg.x_gesture = state->y_gesture;
    linear_module_data_upload_msg.y_gesture = state->x_gesture;
    linear_module_data_upload = linear_module_data_upload_msg;
    pub_linear_module_data_upload.publish(linear_module_data_upload_msg);
}

void publish_moduan_state_topic(
    Module_State* state,
    Motor_State* mot_state)
{
    tie_robot_msgs::ModuanState state_msg;
    const bool executing = moduan_plc_execution_state.load(std::memory_order_acquire);
    const bool error =
        state->ERROR_FLAG_X != 0 ||
        state->ERROR_FLAG_Y != 0 ||
        state->ERROR_FLAG_Z != 0 ||
        state->ERROR_FLAG_LASHING != 0 ||
        mot_state->ERROR_FLAG_MOTOR != 0 ||
        error_detected.load(std::memory_order_acquire);

    std::string runtime_error;
    {
        std::lock_guard<std::mutex> lock(error_msg_mutex);
        runtime_error = last_error_msg;
    }

    state_msg.connected = plc != nullptr;
    state_msg.ready = state_msg.connected && !error;
    state_msg.executing = executing;
    state_msg.finish_all = state->FINISH_ALL_FLAG != 0;
    state_msg.error = error;
    state_msg.x = state->X;
    state_msg.y = state->Y;
    state_msg.z = state->Z;
    state_msg.motor_angle = mot_state->MOTOR_ANGLE;
    state_msg.phase = executing ? "executing" : (state_msg.finish_all ? "finished" : "idle");
    state_msg.last_error = runtime_error;
    pub_moduan_state_topic.publish(state_msg);
}

void pub_moduan_work_state(bool moduan_work_flag)
{
    std_msgs::Bool msg_;
    msg_.data = moduan_work_flag;
    pub_moduan_work.publish(msg_);
}

void enable_lashing_callback(const std_msgs::Float32::ConstPtr& msg)
{
    enable_lashing = bool(msg->data);
    printCurrentTime();
    ros_log_printf("Moduan_log: 绑扎使能状态已更新为：%s\n", enable_lashing ? "允许绑扎" : "禁止绑扎");
    std::lock_guard<std::mutex> lock(plc_mutex);
    PLC_Order_Write(IS_LASHING, enable_lashing ? 1 : 0, plc);
}

void pause_interrupt_Callback(const std_msgs::Float32& debug_mes)
{
    if (debug_mes.data == 1.0)
    {
        std::lock_guard<std::mutex> lock(plc_mutex);
        PLC_Order_Write(IS_STOP, 1, plc);
        handle_pause_interrupt = true;
        printCurrentTime();
        ros_log_printf("Moduan_log:手动暂停，正在暂停末端运动。\n");
    }
}

bool moduan_bind_service(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    (void)req;
    std::lock_guard<std::mutex> lashing_lock(lashing_mutex);
    {
        int finishall_flag = 0;
        {
            std::lock_guard<std::mutex> lock2(module_state_mutex);
            finishall_flag = static_cast<int>(module_state.FINISH_ALL_FLAG);
        }
        if (finishall_flag)
        {
            std::lock_guard<std::mutex> lock2(plc_mutex);
            PLC_Order_Write(FINISHALL, 0, plc);
        }
    }
    srv.request.request_mode = kProcessImageModeExecutionRefine;
    if (!AI_client.call(srv)) {
        res.success = false;
        res.message = "调用视觉服务失败";
        return true;
    }

    if (!srv.response.success) {
        printCurrentTime();
        ros_log_printf("Moduan_Warn: 单点绑扎执行微调视觉失败：%s\n", srv.response.message.c_str());
        if (srv.response.out_of_height_count > 0) {
            for (size_t i = 0; i < srv.response.out_of_height_point_indices.size() &&
                               i < srv.response.out_of_height_z_values.size(); ++i) {
                ros_log_printf(
                    "Moduan_Warn: 超高点 idx=%d, 实际z=%.2fmm\n",
                    srv.response.out_of_height_point_indices[i],
                    srv.response.out_of_height_z_values[i]
                );
            }
        }
        res.success = false;
        res.message = srv.response.message;
        return true;
    }

    if (srv.response.out_of_height_count > 0) {
        printCurrentTime();
        ros_log_printf("Moduan_Warn: 绑扎视觉检测到超高点，本次仅记录日志，不拦截下游执行。\n");
        for (size_t i = 0; i < srv.response.out_of_height_point_indices.size() &&
                           i < srv.response.out_of_height_z_values.size(); ++i) {
            ros_log_printf(
                "Moduan_Warn: 超高点 idx=%d, 实际z=%.2fmm\n",
                srv.response.out_of_height_point_indices[i],
                srv.response.out_of_height_z_values[i]
            );
        }
    }

    ros_log_printf("Moduan_log: 子区域内钢筋绑扎点数量:%zu.\n", srv.response.PointCoordinatesArray.size());
    auto sortedArray = srv.response.PointCoordinatesArray;
    std::vector<tie_robot_msgs::PointCoords> cameraPoints(sortedArray.begin(), sortedArray.end());
    std::vector<tie_robot_msgs::PointCoords> filteredPoints;
    std::string transform_error;
    if (!transform_scepter_camera_points_to_gripper_points(cameraPoints, filteredPoints, transform_error)) {
        printCurrentTime();
        ros_log_printf("Moduan_Warn: 单点绑扎视觉点转换TCP局部坐标失败：%s\n", transform_error.c_str());
        res.success = false;
        res.message = "单点绑扎视觉点转换TCP局部坐标失败：" + transform_error;
        return true;
    }
    ros_log_printf(
        "Moduan_log: 单点绑扎视觉点已由%s转换为%s，原始点数量:%zu，转换后点数量:%zu。\n",
        kScepterDepthFrame,
        kGripperFrame,
        cameraPoints.size(),
        filteredPoints.size()
    );
    if (filteredPoints.empty()) {
        printCurrentTime();
        ros_log_printf(
            "Moduan_Warn: 视觉无可用绑扎点，跳过当前区域。原始点数量:%zu，过滤后点数量:%zu。\n",
            sortedArray.size(),
            filteredPoints.size()
        );
        res.success = false;
        res.message = "视觉无可用绑扎点，跳过当前区域";
        return true;
    }
    const bool executed = execute_bind_points(filteredPoints, res.message);
    res.success = executed;
    if (res.message.empty()) {
        res.message = executed ? "区域绑扎作业完成" : "视觉无可用绑扎点，跳过当前区域";
    }
    return true;
}

bool moduan_bind_points_service(
    tie_robot_msgs::ExecuteBindPoints::Request &req,
    tie_robot_msgs::ExecuteBindPoints::Response &res)
{
    std::lock_guard<std::mutex> lashing_lock(lashing_mutex);
    clear_finishall_flag_if_needed();
    std::vector<tie_robot_msgs::PointCoords> points(req.points.begin(), req.points.end());
    const bool executed = execute_bind_points(points, res.message, false);
    res.success = executed;
    if (res.message.empty()) {
        res.message = executed ? "区域绑扎作业完成" : "预生成绑扎点为空，跳过当前区域";
    }
    return true;
}

bool moduan_bind_points_fast_service(
    tie_robot_msgs::ExecuteBindPoints::Request &req,
    tie_robot_msgs::ExecuteBindPoints::Response &res)
{
    std::lock_guard<std::mutex> lashing_lock(lashing_mutex);
    ScopedModuleSpeedOverride speed_override(kPrecomputedFastModuleSpeedMmPerSec);
    clear_finishall_flag_if_needed();
    std::vector<tie_robot_msgs::PointCoords> points(req.points.begin(), req.points.end());
    const bool executed = execute_bind_points(points, res.message, false);
    res.success = executed;
    if (res.message.empty()) {
        res.message = executed ? "区域绑扎作业完成" : "预生成绑扎点为空，跳过当前区域";
    }
    return true;
}

void forced_stop_nodeCallback(const std_msgs::Float32 &debug_mes)
{
    if(debug_mes.data == 3.0)
    {
        printCurrentTime();
        ros_log_printf("Moduan_log:急停信号已被触发，强制关闭末端节点。\n");
        ros::shutdown();
    }
}

void request_moduan_zero(const char* reason)
{
    printCurrentTime();
    ros_log_printf("Moduan_log:%s，末端返回零点，旋转电机回零至 %.1f 度。\n", reason, reset_angle);
    std::string driver_error_message;
    if (request_linear_module_zero_via_driver(&driver_error_message)) {
        return;
    }
    ROS_WARN_STREAM("Moduan_Warn: " << driver_error_message << "，回退旧回零链。");
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(EN_DISABLE, 1, plc);
        Set_Motor_Speed(&motor_speed, plc);
        Set_Motor_Angle(&reset_angle, plc);
    }
    move_linear_module_to_origin();
}

void moduan_move_zero_forthread(double x, double y, double z, double angle)
{
    (void)x;
    (void)y;
    (void)z;
    (void)angle;
    printCurrentTime();
    ros_log_printf("Moduan_log:末端返回零点。\n");
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(EN_DISABLE, 1, plc);
        PLC_Order_Write(IS_ZERO, 1, plc);
    }
}

void moduan_move_zero_callback(const std_msgs::Float32::ConstPtr& msg)
{
    (void)msg;
    printCurrentTime();
    ros_log_printf("Moduan_log:末端返回零点。\n");
    std::string driver_error_message;
    if (request_linear_module_zero_via_driver(&driver_error_message)) {
        return;
    }
    ROS_WARN_STREAM("Moduan_Warn: " << driver_error_message << "，回退旧回零链。");
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(EN_DISABLE, 1, plc);
        PLC_Order_Write(IS_ZERO, 1, plc);
    }
}

bool moduan_move_service(
    tie_robot_msgs::linear_module_move::Request &req,
    tie_robot_msgs::linear_module_move::Response &res)
{
    std::lock_guard<std::mutex> lashing_lock(lashing_mutex);
    double x = req.pos_x;
    double y = req.pos_y;
    double z = req.pos_z;
    double angle = req.angle;

    printCurrentTime();
    ros_log_printf("Moduan_log:正在使用三轴运动模式，目标点(%lf,%lf,%lf)。\n", x, y, z);
    if(x < 0 || x > kTravelMaxXMm || y < 0 || y > kTravelMaxYMm ||
       z < kTcpTravelMinZMm || z > kTcpTravelMaxZMm)
    {
        printCurrentTime();
        ros_log_printf("Moduan_log:目标点超出范围。\n");
        res.success = false;
        res.message = "目标点超出范围，TCP z轴行程仅支持0~140mm";
        return true;
    }

    tie_robot_msgs::PointCoords single_point;
    single_point.idx = 1;
    single_point.World_coord[0] = static_cast<float>(x);
    single_point.World_coord[1] = static_cast<float>(y);
    single_point.World_coord[2] = static_cast<float>(z);
    single_point.Angle = static_cast<float>(angle);

    std::vector<tie_robot_msgs::PointCoords> single_points{single_point};
    const bool executed = execute_bind_points(single_points, res.message, false);
    if (!executed) {
        res.success = false;
        if (res.message.empty()) {
            res.message = "等待FINISHALL标志超时，线性模组未确认完成";
        }
        return true;
    }

    ros_log_printf("Moduan_log:线性模组现在已经运行至(%lf,%lf,%lf)mm处。\n", x, y, z);
    res.success = true;
    res.message = "运动完成";
    return true;
}

void light_switch(const std_msgs::Bool &debug_mes)
{
    if (!debug_mes.data) {
        printCurrentTime();
        ros_log_printf("Moduan_log:灯已关闭。\n");
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(LIGHT, 0x00, plc);
    } else {
        printCurrentTime();
        ros_log_printf("Moduan_log:灯已打开。\n");
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(LIGHT, 0x01, plc);
    }
}

void send_odd_points_callback(const std_msgs::Bool &debug_mes)
{
    if(debug_mes.data)
    {
        printCurrentTime();
        ros_log_printf("Moduan_log:跳绑2/4已开启；视觉直绑仅绑第1和第4个点，全局预计算路径由上游棋盘格过滤。\n");
        send_odd_points  = 1;
    }
    else
    {
        printCurrentTime();
        ros_log_printf("Moduan_log:跳绑已关闭，恢复全绑。\n");
        send_odd_points  = 3;
    }
}

void change_speed_callback(const std_msgs::Float32 &debug_mes)
{
    printCurrentTime();
    ros_log_printf("Moduan_log:设置速度为%lf。\n",debug_mes.data);
    apply_module_speed_mm_per_sec(static_cast<double>(debug_mes.data));
}

void handSolveWarnCallback(const std_msgs::Float32 &warn_msg)
{
    if (warn_msg.data == 1.0 && !handle_pause_interrupt)
    {
        printCurrentTime();
        ROS_WARN("手动恢复报警复位\n");
        {
            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(WARNING_RESET, 1, plc);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            {
                std::lock_guard<std::mutex> lock2(plc_mutex);
                PLC_Order_Write(WARNING_RESET, 0, plc);
            }
        }

        for(int n = 0; n < 6; n++)
        {
            std_msgs::Float32 error_flag;
            error_flag.data = 0.0;
            pub_lashing_warning.publish(error_flag);
            pub_moduan_warning.publish(error_flag);
        }
    }
    else {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(IS_STOP, 0, plc);
        handle_pause_interrupt = false;
        printCurrentTime();
        ros_log_printf("Moduan_log:手动恢复，正在恢复末端运动。\n");
    }
    {
        std::lock_guard<std::mutex> lock2(plc_mutex);
        PLC_Order_Write(FINISHALL, 0, plc);
    }
}

void read_module_motor_state(Module_State *state, Motor_State *mot_state)
{
    while (true)
    {
        float robot_battery_voltage = 0.;
        float robot_temperature = 0.;
        {
            std::lock_guard<std::mutex> lock1(module_state_mutex);
            std::lock_guard<std::mutex> lock2(plc_mutex);
            state->X = Read_Module_Coordinate(RX_COORDINATE, plc);
            state->Y = Read_Module_Coordinate(RY_COORDINATE, plc);
            state->Z = Read_Module_Coordinate(RZ_COORDINATE, plc);
            state->X_SPEED = Read_Module_Speed(RX_SPEED, plc);
            state->Y_SPEED = Read_Module_Speed(RY_SPEED, plc);
            state->Z_SPEED = Read_Module_Speed(RZ_SPEED, plc);
            uint16_t status_moudle_error_all = Read_Module_Status(ERROR_INQUIRE, plc);
            uint16_t status_moudle_error_ceju = Read_Module_Status(CEJU, plc);
            uint16_t status_moudle_arrive_z = Read_Module_Status(ARRIVEZ, plc);
            uint16_t status_moudle_finish = Read_Module_Status(FINISHALL, plc);
            state->JULI = (status_moudle_error_ceju & 0x02) != 0;
            state->ARRIVEZ_FLAG = (status_moudle_arrive_z & 0x02) != 0;
            state->FINISH_ALL_FLAG = (status_moudle_finish & 0x01) != 0;

            state->ERROR_FLAG_X = (status_moudle_error_all & 0x01) != 0;
            state->ERROR_FLAG_Y = (status_moudle_error_all & 0x02) != 0;
            state->ERROR_FLAG_Z = (status_moudle_error_all & 0x04) != 0;
            mot_state->ERROR_FLAG_MOTOR = (status_moudle_error_all & 0x08) != 0;
            state->ERROR_FLAG_LASHING = (status_moudle_error_all & 0x10) != 0;
            mot_state->MOTOR_SPEED = Read_Motor_Speed(plc);
            mot_state->MOTOR_ANGLE = Read_Motor_Angle(plc);
            robot_battery_voltage = Read_Module_Float_RangedScaled(BATTERY_VOLTAGE, plc, 0.0f, 100.0f, 0.01f);
            robot_temperature = Read_Module_Float_RangedScaled(INNER_TEM, plc, -40.0f, 120.0f, 0.01f);
        }
        {
            if(state->ERROR_FLAG_LASHING == 1)
            {
                ros_log_printf("state->ERROR_FLAG_LASHING:%d\n",state->ERROR_FLAG_LASHING);
                std_msgs::Float32 error_flag;
                error_flag.data = 1.0;
                pub_lashing_warning.publish(error_flag);
                handle_system_error("检测绑扎枪报警");
            }
            if(state->ERROR_FLAG_X == 1 || state->ERROR_FLAG_Y == 1 ||
               state->ERROR_FLAG_Z == 1 || mot_state->ERROR_FLAG_MOTOR == 1 )
            {
                printCurrentTime();
                ros_log_printf("Moduan_error:");
                std_msgs::Float32 error_flag;
                error_flag.data = 1.0;

                if(state->ERROR_FLAG_X == 1)
                {
                    ros_log_printf("X轴异常！！！\n");
                    handle_system_error("X轴异常");
                    pub_moduan_warning.publish(error_flag);
                }
                if(state->ERROR_FLAG_Y == 1)
                {
                    ros_log_printf("Y轴异常！！！\n");
                    handle_system_error("Y轴异常");
                    pub_moduan_warning.publish(error_flag);
                }
                if(state->ERROR_FLAG_Z == 1)
                {
                    ros_log_printf("Z轴异常！！！\n");
                    handle_system_error("Z轴异常");
                    pub_moduan_warning.publish(error_flag);
                }
                if(mot_state->ERROR_FLAG_MOTOR == 1)
                {
                    ros_log_printf("旋转电机异常！！！\n");
                    handle_system_error("旋转电机异常");
                    pub_moduan_warning.publish(error_flag);
                }
                ros_log_printf("正在急停末端运动和旋转电机转动。\n");
                ros::Duration(2.0).sleep();
            }
            if (!current_z_flag && state->JULI == 1 && last_JULI == 0) {
                printCurrentTime();
                ros_log_printf("Moduan_log: 检测到JULI上升沿信号！当前Z轴位置: %f mm\n", state->Z);
                current_z = state->Z;
                current_z_flag = true;
            } else if (state->JULI == 0) {
                current_z = state->Z;
                current_z_flag = false;
            }

            last_JULI = state->JULI;
        }
        pub_moduan_state(pub_linear_module_data_upload, state, mot_state, robot_battery_voltage, robot_temperature);
        publish_moduan_state_topic(state, mot_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::string getBeijingTimeString()
{
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    std::time_t now_time = clock::to_time_t(now);
    std::tm local_time{};
#ifdef __linux__
    localtime_r(&now_time, &local_time);
#else
    localtime_s(&local_time, &now_time);
#endif
    std::ostringstream oss;
    oss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

void robotSaveBindingDataCallback(const std_msgs::Float32::ConstPtr& msg) {
    (void)msg;
    std::string filename = "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_control/data/bindData.txt";
    int floats_per_int = 4;
    std::ofstream ofs(filename, std::ios::app);
    if (!ofs.is_open())
    {
        ROS_ERROR("Cannot open file: %s", filename.c_str());
        return;
    }
    ofs << "===== World Time: " << getBeijingTimeString() << " =====\n";

    int count = 0;
    for (const auto& block : bind_all_data)
    {
        count++;
        const auto& ints = block.first;
        const auto& floats = block.second;

        if (count == static_cast<int>(bind_all_data.size()))
        {
            ofs << "Ints:";
            for (int v : ints)
            {
                ofs << " " << v;
            }
            ofs << "\n";
            ofs << "Floats:\n";
            for (size_t i = 0; i < floats.size(); ++i)
            {
                ofs << floats[i];
                if ((i + 1) % floats_per_int == 0)
                    ofs << "\n";
                else
                    ofs << " ";
            }
            ofs << "\n";
        }
    }
}

void initPLC()
{
    g_linear_module_driver = std::make_unique<tie_robot_hw::driver::LinearModuleDriver>();
    if (!g_moduan_driver_enabled.load()) {
        return;
    }
    plc = PLC_Connection();
    if (plc == nullptr) {
        std::lock_guard<std::mutex> lock(error_msg_mutex);
        last_error_msg = "末端驱动初始化失败，PLC连接未建立";
        return;
    }
    PLC_Order_Write(WARNING_RESET, 1, plc);
    PLC_Order_Write(WARNING_RESET, 0, plc);
    PLC_Order_Write(EN_DISABLE, 1, plc);
    PLC_Order_Write(FINISHALL, 0, plc);
    PLC_Order_Write(IS_STOP, 0, plc);
    Set_Module_Speed(WX_SPEED, &module_speed, plc);
    Set_Module_Speed(WY_SPEED, &module_speed, plc);
    Set_Module_Speed(WZ_SPEED, &module_speed, plc);
    Set_Motor_Speed(&motor_speed, plc);
    Set_Motor_Angle(&reset_angle, plc);
}

void auto_zero_on_startup(ros::NodeHandle& private_nh)
{
    bool auto_zero_on_start = true;
    private_nh.param("auto_zero_on_start", auto_zero_on_start, true);
    if (!auto_zero_on_start) {
        printCurrentTime();
        ros_log_printf("Moduan_log:节点启动自动回零已关闭。\n");
        return;
    }

    ros::Duration(0.2).sleep();
    request_moduan_zero("节点启动自动回零");
}

bool moduan_driver_start_service(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    std::string detail;
    res.success = start_moduan_driver_connection(&detail);
    res.message = res.success ? "末端驱动已启动并建立连接。" : detail;
    return true;
}

bool moduan_driver_stop_service(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    stop_moduan_driver_connection();
    res.success = true;
    res.message = "末端驱动已关闭。";
    return true;
}

bool moduan_driver_restart_service(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    stop_moduan_driver_connection();
    std::string detail;
    res.success = start_moduan_driver_connection(&detail);
    res.message = res.success ? "末端驱动已重启并恢复连接。" : detail;
    return true;
}

bool moduan_driver_raw_execute_points_service(
    tie_robot_msgs::ExecuteBindPoints::Request& req,
    tie_robot_msgs::ExecuteBindPoints::Response& res)
{
    const bool previous_remote_mode = g_use_remote_moduan_driver.exchange(false);
    res.success = execute_bind_points(req.points, res.message, true);
    g_use_remote_moduan_driver.store(previous_remote_mode);
    return true;
}

int RunModuanNodeWithDefaultRole(int argc, char** argv, const std::string& default_role) {
    (void)argc;
    (void)argv;
    setlocale(LC_ALL, "");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");
    std::string node_role = default_role;
    private_nh.param<std::string>("node_role", node_role, node_role);
    const bool driver_role = node_role == "driver" || node_role == "compat_all";
    const bool motion_controller_role = node_role == "motion_controller" || node_role == "compat_all";
    g_use_remote_moduan_driver.store(!driver_role);
    printCurrentTime();
    ros_log_printf(
        "<---%s Started, role=%s.--->\n",
        ros::this_node::getName().c_str(),
        node_role.c_str()
    );
    signal(SIGINT, signalHandler);
    if (!driver_role && !motion_controller_role) {
        ros_log_printf("Moduan_Error: 未知线性模组节点角色 node_role=%s，节点退出。\n", node_role.c_str());
        return 1;
    }
    if (driver_role) {
        initPLC();
    }

    pub_moduan_work = nh_.advertise<std_msgs::Bool>("/moduan_work", 5);
    if (driver_role) {
        pub_moduan_warning = nh_.advertise<std_msgs::Float32>("/robot/moduan_status", 5);
        pub_lashing_warning = nh_.advertise<std_msgs::Float32>("/robot/binding_gun_status", 5);
        pub_forced_stop = nh_.advertise<std_msgs::Float32>("/cabin/forced_stop", 5);
        pub_pause = nh_.advertise<std_msgs::Bool>("/cabin/pause_interrupt", 5);
        pub_linear_module_data_upload =
            nh_.advertise<tie_robot_msgs::linear_module_upload>("/moduan/moduan_gesture_data", 5);
        pub_moduan_state_topic =
            nh_.advertise<tie_robot_msgs::ModuanState>("/moduan/state", 5);
    }

    ros::ServiceServer linear_service;
    ros::ServiceServer lashing_service;
    ros::ServiceServer pseudo_slam_lashing_service;
    ros::ServiceServer pseudo_slam_lashing_fast_service;
    ros::ServiceServer moduan_driver_start_srv;
    ros::ServiceServer moduan_driver_stop_srv;
    ros::ServiceServer moduan_driver_restart_srv;
    ros::ServiceServer moduan_driver_raw_execute_srv;

    if (motion_controller_role) {
        if (!g_tf_buffer) {
            g_tf_buffer = std::make_unique<tf2_ros::Buffer>();
            g_tf_listener = std::make_unique<tf2_ros::TransformListener>(*g_tf_buffer);
        }
        AI_client = nh_.serviceClient<tie_robot_msgs::ProcessImage>("/pointAI/process_image");
        linear_service = nh_.advertiseService("/moduan/single_move", moduan_move_service);
        lashing_service = nh_.advertiseService("/moduan/sg", moduan_bind_service);
        pseudo_slam_lashing_service =
            nh_.advertiseService("/moduan/sg_precomputed", moduan_bind_points_service);
        pseudo_slam_lashing_fast_service =
            nh_.advertiseService("/moduan/sg_precomputed_fast", moduan_bind_points_fast_service);
        g_execute_bind_points_action_server = std::make_unique<ExecuteBindPointsActionServer>(
            nh_,
            "/moduan/execute_bind_points",
            execute_bind_points_action_callback,
            false
        );
        g_execute_bind_points_action_server->start();
    }

    ros::Timer moduan_diagnostic_timer;
    if (driver_role) {
        moduan_driver_start_srv =
            nh_.advertiseService("/moduan/driver/start", moduan_driver_start_service);
        moduan_driver_stop_srv =
            nh_.advertiseService("/moduan/driver/stop", moduan_driver_stop_service);
        moduan_driver_restart_srv =
            nh_.advertiseService("/moduan/driver/restart", moduan_driver_restart_service);
        moduan_driver_raw_execute_srv =
            nh_.advertiseService("/moduan/driver/raw_execute_points", moduan_driver_raw_execute_points_service);

        g_moduan_diagnostic_updater = std::make_unique<diagnostic_updater::Updater>(nh_);
        g_moduan_diagnostic_updater->setHardwareID(kModuanDiagnosticHardwareId);
        g_moduan_diagnostic_updater->add("末端驱动", produce_moduan_diagnostics);
        moduan_diagnostic_timer =
            nh_.createTimer(ros::Duration(1.0), moduan_diagnostic_timer_callback);

        std::thread get_module_state_thread(read_module_motor_state, &module_state, &motor_state);
        get_module_state_thread.detach();
    }

    ros::Subscriber moduan_zero_sub;
    ros::Subscriber enb_las_sub_local;
    ros::Subscriber forced_stop;
    ros::Subscriber hand_solve_warn;
    ros::Subscriber interrupt0;
    ros::Subscriber send_odd;
    ros::Subscriber change_speed;
    ros::Subscriber light_order;
    ros::Subscriber save_bind_data_sub;

    if (driver_role) {
        moduan_zero_sub = nh_.subscribe("/web/moduan/moduan_move_zero", 5, &moduan_move_zero_callback);
        enb_las_sub_local = nh_.subscribe("/web/moduan/enb_las", 5, &enable_lashing_callback);
        forced_stop = nh_.subscribe("/web/moduan/forced_stop", 5, &forced_stop_nodeCallback);
        hand_solve_warn = nh_.subscribe("/web/moduan/hand_sovle_warn", 5, &handSolveWarnCallback);
        interrupt0 = nh_.subscribe("/web/moduan/interrupt_stop", 5, &pause_interrupt_Callback);
        send_odd = nh_.subscribe("/web/moduan/send_odd_points", 5, &send_odd_points_callback);
        change_speed = nh_.subscribe("/web/moduan/set_moduan_speed", 5, &change_speed_callback);
        light_order = nh_.subscribe("/web/moduan/light", 5, &light_switch);
        save_bind_data_sub = nh_.subscribe("/web/moduan/save_binding_data", 5, robotSaveBindingDataCallback);
    }

    (void)private_nh;
    (void)linear_service;
    (void)lashing_service;
    (void)pseudo_slam_lashing_service;
    (void)pseudo_slam_lashing_fast_service;
    (void)moduan_driver_start_srv;
    (void)moduan_driver_stop_srv;
    (void)moduan_driver_restart_srv;
    (void)moduan_driver_raw_execute_srv;
    (void)moduan_diagnostic_timer;
    (void)moduan_zero_sub;
    (void)enb_las_sub_local;
    (void)forced_stop;
    (void)hand_solve_warn;
    (void)interrupt0;
    (void)send_odd;
    (void)change_speed;
    (void)light_order;
    (void)save_bind_data_sub;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}

int RunModuanNode(int argc, char** argv)
{
    return RunModuanNodeWithDefaultRole(argc, argv, "compat_all");
}
