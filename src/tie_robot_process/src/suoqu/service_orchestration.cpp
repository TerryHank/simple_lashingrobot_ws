#include "suoqu_runtime_internal.hpp"
#include "tie_robot_process/suoqu/cabin_transport.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

bool cabinDriverStartService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    cabin_driver_enabled.store(true);
    if (!g_cabin_driver) {
        g_cabin_driver = std::make_unique<tie_robot_hw::driver::CabinDriver>();
    }
    res.success = connectToServer();
    res.message = res.success
        ? "索驱驱动已启动并建立连接。"
        : tie_robot_process::suoqu::get_last_cabin_failure_detail();
    return true;
}

bool cabinDriverStopService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    cabin_driver_enabled.store(false);
    if (g_cabin_driver) {
        g_cabin_driver->stop();
    }
    tie_robot_process::suoqu::sync_global_socket_fd_from_cabin_driver();
    tie_robot_process::suoqu::clear_last_cabin_transport_error_detail();
    cabin_driver_last_state_stamp_sec.store(0.0);
    res.success = true;
    res.message = "索驱驱动已关闭。";
    return true;
}

bool cabinDriverRestartService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    std_srvs::Trigger::Request dummy_req;
    std_srvs::Trigger::Response dummy_res;
    cabinDriverStopService(dummy_req, dummy_res);
    return cabinDriverStartService(dummy_req, res);
}

bool cabinMotionStopService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    std::string driver_error_message;
    res.success = tie_robot_process::suoqu::stop_cabin_motion_via_driver(&driver_error_message);
    res.message = res.success
        ? "索驱运动已停止。"
        : driver_error_message;
    return true;
}

bool startPseudoSlamScan(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    std::vector<Cabin_Point> con_path;
    float cabin_height = 0.0f;
    float cabin_speed = 0.0f;
    try {
        load_configured_path(con_path, cabin_height, cabin_speed);
        cabin_speed = get_global_cabin_move_speed_mm_per_sec();
        res.success = run_pseudo_slam_scan(
            con_path,
            cabin_height,
            cabin_speed,
            PseudoSlamScanStrategy::kFixedManualWorkspace,
            false,
            res.message
        );
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
    }
    return true;
}

bool clearPseudoSlamMarkersService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    clear_pseudo_slam_markers();
    res.success = true;
    res.message = "已清除/cabin/pseudo_slam_markers历史绑扎点。";
    return true;
}

bool startPseudoSlamScanWithOptions(
    tie_robot_msgs::StartPseudoSlamScan::Request& req,
    tie_robot_msgs::StartPseudoSlamScan::Response& res)
{
    std::vector<Cabin_Point> con_path;
    float cabin_height = 0.0f;
    float cabin_speed = 0.0f;
    try {
        load_configured_path(con_path, cabin_height, cabin_speed);
        cabin_speed = get_global_cabin_move_speed_mm_per_sec();
        PseudoSlamFixedScanPoseOverride fixed_scan_pose_override;
        if (req.use_fixed_scan_pose_override) {
            if (!std::isfinite(req.fixed_scan_pose_x_mm) ||
                !std::isfinite(req.fixed_scan_pose_y_mm) ||
                !std::isfinite(req.fixed_scan_pose_z_mm)) {
                res.success = false;
                res.message = "固定识别位姿覆盖参数无效";
                return true;
            }
            fixed_scan_pose_override.enabled = true;
            fixed_scan_pose_override.x_mm = req.fixed_scan_pose_x_mm;
            fixed_scan_pose_override.y_mm = req.fixed_scan_pose_y_mm;
            fixed_scan_pose_override.z_mm = req.fixed_scan_pose_z_mm;
        }
        res.success = run_pseudo_slam_scan(
            con_path,
            cabin_height,
            cabin_speed,
            normalize_pseudo_slam_scan_strategy(req.scan_strategy),
            req.enable_capture_gate,
            res.message,
            req.bind_group_point_count,
            req.bind_group_row_threshold_mm,
            req.bind_group_column_threshold_mm,
            req.bind_execution_cabin_min_z_mm,
            req.bind_execution_cabin_z_mode,
            fixed_scan_pose_override,
            req.recognition_pose_index
        );
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
    }
    return true;
}

bool replanPseudoSlamBindPath(
    tie_robot_msgs::ReplanPseudoSlamBindPath::Request& req,
    tie_robot_msgs::ReplanPseudoSlamBindPath::Response& res)
{
    printCurrentTime();
    ros_log_printf(
        "Cabin_log: 收到阈值重规划请求：识别位姿=%u，成行阈值=%.2fmm，成列阈值=%.2fmm，每组点数=%u，规划Z=%.2fmm，Z模式=%u。\n",
        static_cast<unsigned int>(req.recognition_pose_index),
        static_cast<double>(req.bind_group_row_threshold_mm),
        static_cast<double>(req.bind_group_column_threshold_mm),
        static_cast<unsigned int>(req.bind_group_point_count),
        static_cast<double>(req.bind_execution_cabin_min_z_mm),
        static_cast<unsigned int>(req.bind_execution_cabin_z_mode)
    );

    int area_count = 0;
    int group_count = 0;
    int point_count = 0;
    try {
        res.success = replan_pseudo_slam_bind_path_from_current_points(
            res.message,
            req.bind_group_point_count,
            req.bind_group_row_threshold_mm,
            req.bind_group_column_threshold_mm,
            req.bind_execution_cabin_min_z_mm,
            req.bind_execution_cabin_z_mode,
            req.recognition_pose_index,
            &area_count,
            &group_count,
            &point_count
        );
        res.area_count = static_cast<uint32_t>(std::max(area_count, 0));
        res.group_count = static_cast<uint32_t>(std::max(group_count, 0));
        res.point_count = static_cast<uint32_t>(std::max(point_count, 0));
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
        res.area_count = 0;
        res.group_count = 0;
        res.point_count = 0;
    }
    return true;
}

bool bind_current_area_from_scan_service(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    try {
        res.success = run_current_area_bind_from_scan_test(res.message);
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
    }
    return true;
}

bool bind_path_direct_test_service(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    try {
        res.success = run_bind_path_direct_test(res.message);
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
    }
    return true;
}

bool run_global_work_with_execution_memory_mode(
    const std::string& command,
    bool use_execution_memory,
    tie_robot_msgs::MotionControl::Response& res)
{
    (void)command;
    try {
        const GlobalExecutionMode execution_mode = get_global_execution_mode();
        printCurrentTime();
        ros_log_printf(
            "Cabin_log: 当前全局执行模式为%s，执行记忆=%s。\n",
            global_execution_mode_name(execution_mode),
            use_execution_memory ? "开启" : "关闭"
        );

        std::ifstream scan_file(pseudo_slam_bind_path_json_file);
        if (!scan_file.good()) {
            res.success = false;
            res.message =
                "未找到pseudo_slam_bind_path.json，请先完成扫描建图后再开始执行层";
            return true;
        }

        switch (execution_mode) {
            case GlobalExecutionMode::kSlamPrecomputed:
                res.success = run_bind_from_scan(res.message, use_execution_memory);
                return true;
            case GlobalExecutionMode::kLedgerWithRefine:
                res.success = run_live_visual_global_work(res.message, use_execution_memory);
                return true;
            case GlobalExecutionMode::kPlannedPathRefineOnly:
                res.success = run_planned_path_refine_only_global_work(res.message, use_execution_memory);
                return true;
        }

        res.success = false;
        res.message = "未知的全局执行模式";
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
    }
    return true;
}

bool startGlobalWork(
    tie_robot_msgs::MotionControl::Request& req,
    tie_robot_msgs::MotionControl::Response& res)
{
    printCurrentTime();
    ros_log_printf("Cabin_log: 收到%s\n", req.command.c_str());
    return run_global_work_with_execution_memory_mode(req.command, true, res);
}

bool startGlobalWorkWithOptions(
    tie_robot_msgs::StartGlobalWork::Request& req,
    tie_robot_msgs::StartGlobalWork::Response& res)
{
    printCurrentTime();
    ros_log_printf(
        "Cabin_log: 收到%s，clear_execution_memory=%s，use_execution_memory=%s（默认执行记忆关闭）。\n",
        req.command.c_str(),
        req.clear_execution_memory ? "true" : "false",
        req.use_execution_memory ? "true" : "false"
    );

    try {
        std::lock_guard<std::mutex> pseudo_slam_workflow_lock(pseudo_slam_workflow_mutex);
        if (req.clear_execution_memory && req.use_execution_memory) {
            std::string current_path_signature;
            if (!load_current_path_signature_for_execution(current_path_signature, res.message)) {
                res.success = false;
                return true;
            }
            if (!reset_bind_execution_memory_from_current_scan_artifacts(
                    current_path_signature,
                    res.message
                )) {
                res.success = false;
                return true;
            }
        }
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
        return true;
    }

    tie_robot_msgs::MotionControl::Response legacy_res;
    const bool handled = run_global_work_with_execution_memory_mode(
        req.command,
        req.use_execution_memory,
        legacy_res
    );
    res.success = legacy_res.success;
    res.message = legacy_res.message;
    return handled;
}
