#include "tie_robot_web/web_bridge/topics_transfer_runtime.hpp"

#include <thread>

namespace tie_robot_web {
namespace web_bridge {
namespace {

tie_robot_msgs::ProcessImage g_image_srv;
tie_robot_msgs::SingleMove g_single_chassis_move_srv;
tie_robot_msgs::Pathguihua g_global_chassis_path_srv;
tie_robot_msgs::linear_module_move g_moduan_move_srv;

void publish_plan_path_service(const tie_robot_msgs::Pathguihua& path_srv)
{
    (void)path_srv;
    if (g_service_clients.chassis_plan_path_client.call(g_global_chassis_path_srv)) {
        ROS_INFO("Call succeeded: %s", g_global_chassis_path_srv.response.message.c_str());
    } else {
        ROS_ERROR("Call failed");
    }
}

void publish_moduan_move_debug_service(const tie_robot_msgs::linear_module_move& move_srv)
{
    tie_robot_msgs::linear_module_move request = move_srv;
    g_service_clients.moduan_client.call(request);
}

void publish_chassis_move_debug_service(const tie_robot_msgs::SingleMove& move_srv)
{
    tie_robot_msgs::SingleMove request = move_srv;
    if (g_service_clients.chassis_single_move_client.call(request)) {
        if (request.response.success) {
            ROS_INFO("Call succeeded: %s", request.response.message.c_str());
        } else {
            ROS_ERROR("Call failed: %s", request.response.message.c_str());
        }
    } else {
        ROS_ERROR("Call transport failed：%s", request.response.message.c_str());
    }
}

void publish_pointai_debug_service(const tie_robot_msgs::ProcessImage& image_srv)
{
    tie_robot_msgs::ProcessImage request = image_srv;
    if (g_service_clients.image_solve_client.call(request)) {
        ROS_INFO("Call succeeded: %d", request.response.count);
    } else {
        ROS_ERROR("Call failed");
    }
}

void publish_single_bind_service(bool use_precomputed_current_area)
{
    std_srvs::Trigger single_bind_srv;
    ros::ServiceClient& target_client = use_precomputed_current_area
        ? g_service_clients.current_area_bind_from_scan_client
        : g_service_clients.lashing_client;

    if (target_client.call(single_bind_srv)) {
        ROS_INFO("Call succeeded: %s", single_bind_srv.response.message.c_str());
    } else {
        ROS_ERROR("Call failed");
    }
}

}  // namespace

void robotStartCallback(const std_msgs::Float32::ConstPtr& msg)
{
    printCurrentTime();
    logMessage("/web/cabin/start", "收到启动机器人命令，值: " + std::to_string(msg->data));
    std::thread(startRobotService).detach();
}

void robotPlanPathCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    printCurrentTime();
    logMessage("/web/cabin/plan_path", "收到规划作业路径命令");
    g_global_chassis_path_srv.request.marking_x = msg->position.x;
    g_global_chassis_path_srv.request.marking_y = msg->position.y;
    g_global_chassis_path_srv.request.height = msg->position.z;
    g_global_chassis_path_srv.request.zone_x = msg->orientation.x;
    g_global_chassis_path_srv.request.zone_y = msg->orientation.y;
    g_global_chassis_path_srv.request.robot_x_step = msg->orientation.z;
    g_global_chassis_path_srv.request.robot_y_step = msg->orientation.w;
    g_global_chassis_path_srv.request.speed = 0.0;

    std::cout << COLOR_GREEN << "作业参数(mm):["
              << g_global_chassis_path_srv.request.marking_x << ","
              << g_global_chassis_path_srv.request.marking_y << ","
              << g_global_chassis_path_srv.request.height << ","
              << g_global_chassis_path_srv.request.zone_x << ","
              << g_global_chassis_path_srv.request.zone_y << ","
              << g_global_chassis_path_srv.request.robot_x_step << ","
              << g_global_chassis_path_srv.request.robot_y_step << ","
              << g_global_chassis_path_srv.request.speed << "]" << COLOR_RESET << std::endl;

    std::thread(publish_plan_path_service, g_global_chassis_path_srv).detach();
}

void robotClearPathCallback(const std_msgs::Float32::ConstPtr& msg)
{
    printCurrentTime();
    logMessage("/web/cabin/clear_path", "收到清除作业路径命令，值: " + std::to_string(msg->data));
    clearJsonFile("/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/path_points.json");
}

void robotRestartCallback(const std_msgs::Float32::ConstPtr& msg)
{
    printCurrentTime();
    logMessage("/web/cabin/restart", "收到重启机器人命令，值: " + std::to_string(msg->data));
    std::thread(restartRobotService).detach();
}

void moduanLinearModuleMoveCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    printCurrentTime();
    logMessage("/web/moduan/linear_module_move", "收到末端运动调试命令");
    g_moduan_move_srv.request.pos_x = msg->position.x;
    g_moduan_move_srv.request.pos_y = msg->position.y;
    g_moduan_move_srv.request.pos_z = msg->position.z;
    g_moduan_move_srv.request.angle = msg->orientation.x;
    std::cout << COLOR_GREEN << "末端调试:["
              << g_moduan_move_srv.request.pos_x << ","
              << g_moduan_move_srv.request.pos_y << ","
              << g_moduan_move_srv.request.pos_z << ","
              << g_moduan_move_srv.request.angle << "]" << COLOR_RESET << std::endl;
    std::thread(publish_moduan_move_debug_service, g_moduan_move_srv).detach();
}

void chassisModuleMoveCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    printCurrentTime();
    logMessage("/web/cabin/single_move", "收到索驱运动调试命令");
    g_single_chassis_move_srv.request.command = "单点运动请求";
    g_single_chassis_move_srv.request.x = static_cast<float>(msg->position.x);
    g_single_chassis_move_srv.request.y = static_cast<float>(msg->position.y);
    g_single_chassis_move_srv.request.z = static_cast<float>(msg->position.z);
    g_single_chassis_move_srv.request.speed = static_cast<float>(msg->orientation.x);
    std::thread(publish_chassis_move_debug_service, g_single_chassis_move_srv).detach();
}

void fastImageSolveProcessImageCallback(const std_msgs::Float32::ConstPtr& msg)
{
    printCurrentTime();
    logMessage("/web/pointAI/process_image", "收到视觉识别调试命令，值: " + std::to_string(msg->data));
    std::thread(publish_pointai_debug_service, g_image_srv).detach();
}

void moduanSingleBindCallback(const std_msgs::Float32::ConstPtr& msg)
{
    printCurrentTime();
    const bool use_precomputed_current_area = msg->data >= 1.0f;
    if (use_precomputed_current_area) {
        logMessage(
            "/web/moduan/single_bind",
            "收到定点绑扎调试命令，single_bind模式=1：使用扫描后的当前区域预计算点直执行");
    } else {
        logMessage(
            "/web/moduan/single_bind",
            "收到定点绑扎调试命令，single_bind模式=0：使用原逻辑本区域识别+执行");
    }
    std::thread(publish_single_bind_service, use_precomputed_current_area).detach();
}

void robotStopCallback(const std_msgs::Float32::ConstPtr& msg)
{
    printCurrentTime();
    logMessage("/web/cabin/shutdown", "收到关闭机器人命令，值: " + std::to_string(msg->data));
    stopRobotService();
}

}  // namespace web_bridge
}  // namespace tie_robot_web
