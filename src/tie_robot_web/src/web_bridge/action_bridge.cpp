#include "tie_robot_web/web_bridge/web_action_bridge_runtime.hpp"

namespace tie_robot_web {
namespace web_bridge {

std::string start_global_work_execution_mode_name(uint8_t execution_mode)
{
    switch (execution_mode) {
        case 0:
            return "slam_precomputed";
        case 1:
            return "ledger_with_refine";
        case 2:
            return "planned_path_refine_only";
        default:
            return "unknown";
    }
}

void executeStartPseudoSlamScanAction(
    const tie_robot_msgs::StartPseudoSlamScanTaskGoalConstPtr& goal)
{
    auto& server = *g_start_pseudo_slam_scan_action_server;
    tie_robot_msgs::StartPseudoSlamScanTaskResult result;
    const std::string scan_mode =
        goal->scan_strategy == 3
            ? "当前画面无运动视觉记录"
            : (goal->scan_strategy == 2 ? "固定工作区单次扫描" : (goal->scan_strategy == 1 ? "多扫描位" : "单次中心位"));

    printCurrentTime();
    logMessage(
        "/web/cabin/start_pseudo_slam_scan",
        "收到扫描建图Action目标，模式="
            + scan_mode
            + "，"
            + std::string(goal->enable_capture_gate ? "开启最终采集门" : "关闭最终采集门"));

    if (server.isPreemptRequested()) {
        result.success = false;
        result.message = "扫描建图在派发前被取消";
        server.setPreempted(result, result.message);
        return;
    }

    publish_scan_action_feedback(server, "dispatching", "正在调用/cabin/start_pseudo_slam_scan_with_options", 0.2f);

    tie_robot_msgs::StartPseudoSlamScan scan_srv;
    scan_srv.request.enable_capture_gate = goal->enable_capture_gate;
    scan_srv.request.scan_strategy = goal->scan_strategy;
    scan_srv.request.use_fixed_scan_pose_override = goal->use_fixed_scan_pose_override;
    scan_srv.request.fixed_scan_pose_x_mm = goal->fixed_scan_pose_x_mm;
    scan_srv.request.fixed_scan_pose_y_mm = goal->fixed_scan_pose_y_mm;
    scan_srv.request.fixed_scan_pose_z_mm = goal->fixed_scan_pose_z_mm;
    scan_srv.request.bind_group_point_count = goal->bind_group_point_count;
    if (!g_service_clients.chassis_scan_with_options_client.call(scan_srv)) {
        result.success = false;
        result.message = "扫描建图服务调用失败";
        server.setAborted(result, result.message);
        return;
    }

    result.success = scan_srv.response.success;
    result.message = scan_srv.response.message;
    publish_scan_action_feedback(server, "completed", result.message, 1.0f);
    if (!result.success) {
        server.setAborted(result, result.message);
        return;
    }
    server.setSucceeded(result, result.message);
}

void executeStartGlobalWorkAction(
    const tie_robot_msgs::StartGlobalWorkTaskGoalConstPtr& goal)
{
    auto& server = *g_start_global_work_action_server;
    tie_robot_msgs::StartGlobalWorkTaskResult result;
    const bool clear_execution_memory = goal->clear_execution_memory;
    const bool use_execution_memory = goal->use_execution_memory;
    const std::string execution_mode_name =
        start_global_work_execution_mode_name(goal->execution_mode);
    const std::string execution_memory_mode_name = !use_execution_memory
        ? "执行记忆关闭直接开始"
        : clear_execution_memory
            ? "清空记忆后开始"
            : "按执行记忆续跑";

    printCurrentTime();
    logMessage(
        "/web/cabin/start_global_work",
        "收到开始全局作业Action目标，模式="
            + execution_memory_mode_name
            + "，执行模式="
            + execution_mode_name
            + "，执行记忆="
            + std::string(use_execution_memory ? "开启" : "关闭"));

    if (server.isPreemptRequested()) {
        result.success = false;
        result.message = "开始执行层在派发前被取消";
        server.setPreempted(result, result.message);
        return;
    }

    publish_start_work_action_feedback(server, "set_mode", "正在调用/cabin/set_execution_mode", 0.1f);
    std::string mode_message;
    if (!call_set_execution_mode_service(goal->execution_mode, mode_message)) {
        result.success = false;
        result.message = mode_message;
        server.setAborted(result, result.message);
        return;
    }

    if (server.isPreemptRequested()) {
        result.success = false;
        result.message = "开始执行层在设置模式后被取消";
        server.setPreempted(result, result.message);
        return;
    }

    publish_start_work_action_feedback(server, "dispatching", "正在调用/cabin/start_work_with_options", 0.25f);
    tie_robot_msgs::StartGlobalWork start_work_srv;
    start_work_srv.request.command = "全局运动请求";
    start_work_srv.request.clear_execution_memory = goal->clear_execution_memory;
    start_work_srv.request.use_execution_memory = goal->use_execution_memory;
    if (!g_service_clients.chassis_start_work_with_options_client.call(start_work_srv)) {
        log_trigger_service_transport_failure("/cabin/start_work_with_options");
        result.success = false;
        result.message = "调用/cabin/start_work_with_options失败";
        server.setAborted(result, result.message);
        return;
    }

    result.success = start_work_srv.response.success;
    result.message = start_work_srv.response.message;
    publish_start_work_action_feedback(server, "completed", result.message, 1.0f);
    if (!result.success) {
        server.setAborted(result, result.message);
        return;
    }
    server.setSucceeded(result, result.message);
}

void executeRunBindPathDirectTestAction(
    const tie_robot_msgs::RunBindPathDirectTestTaskGoalConstPtr&)
{
    auto& server = *g_run_bind_path_direct_test_action_server;
    tie_robot_msgs::RunBindPathDirectTestTaskResult result;

    printCurrentTime();
    logMessage(
        "/web/cabin/run_bind_path_direct_test",
        "收到直接执行账本测试Action目标：只读取pseudo_slam_bind_path.json直执行，不走相机live、不清记忆、不写执行记忆");

    if (server.isPreemptRequested()) {
        result.success = false;
        result.message = "直接执行账本测试在派发前被取消";
        server.setPreempted(result, result.message);
        return;
    }

    publish_direct_bind_action_feedback(server, "dispatching", "正在调用/cabin/run_bind_path_direct_test", 0.2f);
    std_srvs::Trigger direct_bind_path_test_srv;
    if (!g_service_clients.direct_bind_path_test_client.call(direct_bind_path_test_srv)) {
        log_trigger_service_transport_failure("/cabin/run_bind_path_direct_test");
        result.success = false;
        result.message = "调用/cabin/run_bind_path_direct_test失败";
        server.setAborted(result, result.message);
        return;
    }

    result.success = direct_bind_path_test_srv.response.success;
    result.message = direct_bind_path_test_srv.response.message;
    publish_direct_bind_action_feedback(server, "completed", result.message, 1.0f);
    if (!result.success) {
        server.setAborted(result, result.message);
        return;
    }
    server.setSucceeded(result, result.message);
}

}  // namespace web_bridge
}  // namespace tie_robot_web
