#include "suoqu_runtime_internal.hpp"

#include <fstream>

bool startPseudoSlamScan(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
    std::vector<Cabin_Point> con_path;
    float cabin_height = 0.0f;
    float cabin_speed = 0.0f;
    try {
        load_configured_path(con_path, cabin_height, cabin_speed);
        res.success = run_pseudo_slam_scan(
            con_path,
            cabin_height,
            cabin_speed,
            PseudoSlamScanStrategy::kSingleCenter,
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
        res.success = run_pseudo_slam_scan(
            con_path,
            cabin_height,
            cabin_speed,
            normalize_pseudo_slam_scan_strategy(req.scan_strategy),
            req.enable_capture_gate,
            res.message
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
    printf("Cabin_log: 收到%s\n", req.command.c_str());
    try {
        const GlobalExecutionMode execution_mode = get_global_execution_mode();
        printCurrentTime();
        printf(
            "Cabin_log: 当前全局执行模式为%s。\n",
            global_execution_mode_name(execution_mode)
        );

        std::ifstream scan_file(pseudo_slam_bind_path_json_file);
        if (scan_file.good()) {
            printCurrentTime();
            printf(
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
    printf(
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
