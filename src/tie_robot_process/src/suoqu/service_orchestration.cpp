#include "suoqu_runtime_internal.hpp"
#include "tie_robot_process/suoqu/cabin_transport.hpp"

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
            fixed_scan_pose_override
        );
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
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

bool startGlobalWork(
    tie_robot_msgs::MotionControl::Request& req,
    tie_robot_msgs::MotionControl::Response& res)
{
    printCurrentTime();
    ros_log_printf("Cabin_log: 收到%s\n", req.command.c_str());
    try {
        const GlobalExecutionMode execution_mode = get_global_execution_mode();
        printCurrentTime();
        ros_log_printf(
            "Cabin_log: 当前全局执行模式为%s。\n",
            global_execution_mode_name(execution_mode)
        );

        std::ifstream scan_file(pseudo_slam_bind_path_json_file);
        if (scan_file.good()) {
            printCurrentTime();
            ros_log_printf(
                "Cabin_log: 开始执行层检测到pseudo_slam_bind_path.json，优先按预生成路径执行。\n"
            );
            res.success = run_bind_from_scan(res.message);
            return true;
        }

        if (execution_mode == GlobalExecutionMode::kLiveVisual) {
            res.success = run_live_visual_global_work(res.message);
            return true;
        }

        res.success = false;
        res.message =
            "当前全局执行模式为slam_precomputed，未找到pseudo_slam_bind_path.json，请先完成扫描建图或切换到live_visual模式";
    } catch (const std::exception& ex) {
        res.success = false;
        res.message = ex.what();
    }
    return true;
}

bool startGlobalWorkWithOptions(
    tie_robot_msgs::StartGlobalWork::Request& req,
    tie_robot_msgs::StartGlobalWork::Response& res)
{
    printCurrentTime();
    ros_log_printf(
        "Cabin_log: 收到%s，clear_execution_memory=%s（clear_execution_memory=true表示先清记忆再执行）。\n",
        req.command.c_str(),
        req.clear_execution_memory ? "true" : "false"
    );

    try {
        std::lock_guard<std::mutex> pseudo_slam_workflow_lock(pseudo_slam_workflow_mutex);
        if (req.clear_execution_memory) {
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

    tie_robot_msgs::MotionControl::Request legacy_req;
    tie_robot_msgs::MotionControl::Response legacy_res;
    legacy_req.command = req.command;
    const bool handled = startGlobalWork(legacy_req, legacy_res);
    res.success = legacy_res.success;
    res.message = legacy_res.message;
    return handled;
}
