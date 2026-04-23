#pragma once

#include "common.hpp"

#include <actionlib/server/simple_action_server.h>
#include <memory>
#include <string>

#include "tie_robot_msgs/RunBindPathDirectTestTaskAction.h"
#include "tie_robot_msgs/StartGlobalWorkTaskAction.h"
#include "tie_robot_msgs/StartPseudoSlamScanTaskAction.h"

namespace tie_robot_web {
namespace web_bridge {

using StartPseudoSlamScanActionServer =
    actionlib::SimpleActionServer<tie_robot_msgs::StartPseudoSlamScanTaskAction>;
using StartGlobalWorkActionServer =
    actionlib::SimpleActionServer<tie_robot_msgs::StartGlobalWorkTaskAction>;
using RunBindPathDirectTestActionServer =
    actionlib::SimpleActionServer<tie_robot_msgs::RunBindPathDirectTestTaskAction>;

struct ServiceClients {
    ros::ServiceClient image_solve_client;
    ros::ServiceClient moduan_client;
    ros::ServiceClient lashing_client;
    ros::ServiceClient current_area_bind_from_scan_client;
    ros::ServiceClient direct_bind_path_test_client;
    ros::ServiceClient chassis_single_move_client;
    ros::ServiceClient chassis_plan_path_client;
    ros::ServiceClient chassis_set_execution_mode_client;
    ros::ServiceClient chassis_start_work_with_options_client;
    ros::ServiceClient chassis_scan_with_options_client;
};

extern ServiceClients g_service_clients;
extern std::unique_ptr<StartPseudoSlamScanActionServer> g_start_pseudo_slam_scan_action_server;
extern std::unique_ptr<StartGlobalWorkActionServer> g_start_global_work_action_server;
extern std::unique_ptr<RunBindPathDirectTestActionServer> g_run_bind_path_direct_test_action_server;
extern const std::string kCabinLastFatalErrorDetailFile;

void logMessage(const std::string& topic, const std::string& message);
std::string read_latest_cabin_fatal_error_detail();
void log_trigger_service_transport_failure(const std::string& service_name);

void publish_scan_action_feedback(
    StartPseudoSlamScanActionServer& server,
    const std::string& stage,
    const std::string& detail,
    float progress);
void publish_start_work_action_feedback(
    StartGlobalWorkActionServer& server,
    const std::string& stage,
    const std::string& detail,
    float progress);
void publish_direct_bind_action_feedback(
    RunBindPathDirectTestActionServer& server,
    const std::string& stage,
    const std::string& detail,
    float progress);
bool call_set_execution_mode_service(uint8_t execution_mode, std::string& message);

void executeStartPseudoSlamScanAction(
    const tie_robot_msgs::StartPseudoSlamScanTaskGoalConstPtr& goal);
void executeStartGlobalWorkAction(
    const tie_robot_msgs::StartGlobalWorkTaskGoalConstPtr& goal);
void executeRunBindPathDirectTestAction(
    const tie_robot_msgs::RunBindPathDirectTestTaskGoalConstPtr& goal);

void startRobotService();
void startDriverStackService();
void restartDriverStackService();
void startAlgorithmStackService();
void restartAlgorithmStackService();
void restartRosStackService();
void restartRobotService();
void stopRobotService();

bool startDriverStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res);
bool restartDriverStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res);
bool startAlgorithmStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res);
bool restartAlgorithmStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res);
bool restartRosStackServiceCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res);

void robotStartCallback(const std_msgs::Float32::ConstPtr& msg);
void robotPlanPathCallback(const geometry_msgs::Pose::ConstPtr& msg);
void robotClearPathCallback(const std_msgs::Float32::ConstPtr& msg);
void robotRestartCallback(const std_msgs::Float32::ConstPtr& msg);
void moduanLinearModuleMoveCallback(const geometry_msgs::Pose::ConstPtr& msg);
void chassisModuleMoveCallback(const geometry_msgs::Pose::ConstPtr& msg);
void fastImageSolveProcessImageCallback(const std_msgs::Float32::ConstPtr& msg);
void moduanSingleBindCallback(const std_msgs::Float32::ConstPtr& msg);
void robotStopCallback(const std_msgs::Float32::ConstPtr& msg);

int RunTopicsTransferNode(int argc, char** argv);

}  // namespace web_bridge
}  // namespace tie_robot_web
