#include "tie_robot_web/web_bridge/web_action_bridge_runtime.hpp"

#include <fstream>
#include <sstream>

namespace tie_robot_web {
namespace web_bridge {

ServiceClients g_service_clients;
std::unique_ptr<StartPseudoSlamScanActionServer> g_start_pseudo_slam_scan_action_server;
std::unique_ptr<StartGlobalWorkActionServer> g_start_global_work_action_server;
std::unique_ptr<RunBindPathDirectTestActionServer> g_run_bind_path_direct_test_action_server;
const std::string kCabinLastFatalErrorDetailFile =
    "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/cabin_last_fatal_error.txt";

void logMessage(const std::string& topic, const std::string& message)
{
    ROS_INFO_STREAM("[web_action_bridge] " << topic << ": " << message);
}

std::string read_latest_cabin_fatal_error_detail()
{
    std::ifstream infile(kCabinLastFatalErrorDetailFile);
    if (!infile.is_open()) {
        return "";
    }

    std::ostringstream buffer;
    buffer << infile.rdbuf();
    return buffer.str();
}

void log_trigger_service_transport_failure(const std::string& service_name)
{
    const std::string fatal_detail = read_latest_cabin_fatal_error_detail();
    if (!fatal_detail.empty()) {
        ROS_ERROR(
            "Call failed: %s。最近一次索驱致命错误: %s",
            service_name.c_str(),
            fatal_detail.c_str());
        return;
    }
    ROS_ERROR("Call failed: %s", service_name.c_str());
}

void publish_scan_action_feedback(
    StartPseudoSlamScanActionServer& server,
    const std::string& stage,
    const std::string& detail,
    float progress)
{
    tie_robot_msgs::StartPseudoSlamScanTaskFeedback feedback;
    feedback.stage = stage;
    feedback.detail = detail;
    feedback.progress = progress;
    server.publishFeedback(feedback);
}

void publish_start_work_action_feedback(
    StartGlobalWorkActionServer& server,
    const std::string& stage,
    const std::string& detail,
    float progress)
{
    tie_robot_msgs::StartGlobalWorkTaskFeedback feedback;
    feedback.stage = stage;
    feedback.detail = detail;
    feedback.progress = progress;
    server.publishFeedback(feedback);
}

void publish_direct_bind_action_feedback(
    RunBindPathDirectTestActionServer& server,
    const std::string& stage,
    const std::string& detail,
    float progress)
{
    tie_robot_msgs::RunBindPathDirectTestTaskFeedback feedback;
    feedback.stage = stage;
    feedback.detail = detail;
    feedback.progress = progress;
    server.publishFeedback(feedback);
}

bool call_set_execution_mode_service(uint8_t execution_mode, std::string& message)
{
    tie_robot_msgs::SetExecutionMode mode_srv;
    mode_srv.request.execution_mode = execution_mode;
    if (!g_service_clients.chassis_set_execution_mode_client.call(mode_srv)) {
        message = "调用/cabin/set_execution_mode失败";
        return false;
    }
    message = mode_srv.response.message;
    return mode_srv.response.success;
}

}  // namespace web_bridge
}  // namespace tie_robot_web
