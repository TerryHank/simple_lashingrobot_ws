#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROCESS_DIR = WORKSPACE_ROOT / "tie_robot_process"
CONTROL_DIR = WORKSPACE_ROOT / "tie_robot_control"
HW_DIR = WORKSPACE_ROOT / "tie_robot_hw"
MSGS_DIR = WORKSPACE_ROOT / "tie_robot_msgs"
WEB_DIR = WORKSPACE_ROOT / "tie_robot_web"


class MotionChainSignalGuardTest(unittest.TestCase):
    def test_moduan_state_topic_and_action_interfaces_are_declared(self):
        cmake = (MSGS_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        state_msg = (MSGS_DIR / "msg" / "ModuanState.msg").read_text(encoding="utf-8")
        action = (
            MSGS_DIR / "action" / "ExecuteBindPointsTask.action"
        ).read_text(encoding="utf-8")

        self.assertIn("ModuanState.msg", cmake)
        self.assertIn("ExecuteBindPointsTask.action", cmake)
        self.assertIn("bool executing", state_msg)
        self.assertIn("bool finish_all", state_msg)
        self.assertIn("PointCoords[] points", action)
        self.assertIn("string phase", action)
        self.assertIn("bool success", action)

    def test_cabin_motion_driver_rejects_only_real_moduan_motion_not_task_latch(self):
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        transport = (
            PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("extern std::atomic<bool> moduan_work_flag;", runtime_header)
        self.assertIn("std::atomic<bool> moduan_work_flag{false};", node)
        self.assertIn("std::atomic<bool> moduan_work_topic_flag{false};", node)
        self.assertIn("std::atomic<bool> moduan_state_executing_flag{false};", node)
        self.assertIn("void refresh_moduan_motion_guard_flag()", node)
        self.assertIn("moduan_work_topic_flag.store(debug_mes.data", node)
        self.assertIn("void moduan_state_Callback(const tie_robot_msgs::ModuanState& state_msg)", node)
        self.assertIn("moduan_state_executing_flag.store(state_msg.executing", node)
        self.assertIn('nh.subscribe("/moduan/state"', node)
        refresh_start = node.index("void refresh_moduan_motion_guard_flag()")
        refresh_end = node.index("\nvoid moduan_work_Callback", refresh_start)
        refresh_body = node[refresh_start:refresh_end]
        self.assertIn("moduan_state_executing_flag.load", refresh_body)
        self.assertNotIn("moduan_work_topic_flag.load", refresh_body)
        run_start = node.index("int RunSuoquNodeWithDefaultRole(")
        run_body = node[run_start:]
        guard_sub_index = run_body.index('moduan_work_sub = nh.subscribe("/moduan_work"')
        driver_role_index = run_body.index("if (is_suoqu_driver_role())")
        bind_executor_role_index = run_body.index("if (is_suoqu_bind_task_executor_role())")
        self.assertLess(guard_sub_index, driver_role_index)
        self.assertLess(guard_sub_index, bind_executor_role_index)

        move_start = transport.index("bool move_cabin_pose_via_driver(")
        move_end = transport.index("\nbool move_cabin_incremental_via_driver", move_start)
        move_body = transport[move_start:move_end]

        self.assertIn("reject_cabin_move_if_moduan_not_safe", move_body)
        guard_index = move_body.index("reject_cabin_move_if_moduan_not_safe")
        remote_call_index = transport.index(
            '"/cabin/driver/raw_move"',
            move_start + guard_index,
        )
        helper_call_index = transport.index("call_remote_cabin_single_move_service", move_start + guard_index)
        driver_call_index = transport.index("::g_cabin_driver->moveToPose", move_start + guard_index)
        self.assertLess(move_start + guard_index, helper_call_index)
        self.assertLess(helper_call_index, remote_call_index)
        self.assertLess(move_start + guard_index, driver_call_index)
        helper_start = transport.index("bool reject_cabin_move_if_moduan_not_safe(")
        helper_end = transport.index("\nstd::string compose_cabin_driver_error_message", helper_start)
        helper_body = transport[helper_start:helper_end]
        self.assertIn("moduan_work_flag.load(", helper_body)
        self.assertIn("末端绑扎/线性模组正在运动", helper_body)

    def test_cabin_motion_driver_requires_moduan_z_axis_at_zero(self):
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        transport = (
            PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("extern std::atomic<bool> moduan_state_received_flag;", runtime_header)
        self.assertIn("extern std::atomic<bool> moduan_state_connected_flag;", runtime_header)
        self.assertIn("extern std::atomic<double> moduan_state_last_stamp_sec;", runtime_header)
        self.assertIn("extern std::atomic<double> moduan_state_z_mm;", runtime_header)
        self.assertIn("std::atomic<bool> moduan_state_received_flag{false};", node)
        self.assertIn("std::atomic<bool> moduan_state_connected_flag{false};", node)
        self.assertIn("std::atomic<double> moduan_state_last_stamp_sec", node)
        self.assertIn("std::atomic<double> moduan_state_z_mm", node)

        callback_start = node.index("void moduan_state_Callback(")
        callback_end = node.index("\n/*\n    函数功能：暂停中断", callback_start)
        callback_body = node[callback_start:callback_end]
        self.assertIn("moduan_state_z_mm.store(state_msg.z", callback_body)
        self.assertIn("moduan_state_connected_flag.store(state_msg.connected", callback_body)
        self.assertIn("moduan_state_last_stamp_sec.store(ros::Time::now().toSec()", callback_body)
        self.assertIn("moduan_state_received_flag.store(true", callback_body)

        self.assertIn("kModuanSafeZZeroToleranceMm", transport)
        self.assertIn("constexpr double kModuanSafeZZeroToleranceMm = 10.0;", transport)
        self.assertIn("kModuanStateFreshMaxAgeSec", transport)
        self.assertIn("reject_cabin_move_if_moduan_not_safe", transport)

        move_start = transport.index("bool move_cabin_pose_via_driver(")
        move_end = transport.index("\nbool move_cabin_incremental_via_driver", move_start)
        move_body = transport[move_start:move_end]
        absolute_guard_index = move_body.index("reject_cabin_move_if_moduan_not_safe")
        remote_call_index = move_body.index('"/cabin/driver/raw_move"')
        driver_call_index = move_body.index("::g_cabin_driver->moveToPose")
        self.assertLess(absolute_guard_index, remote_call_index)
        self.assertLess(absolute_guard_index, driver_call_index)
        self.assertIn("末端Z轴未回到0", transport)
        self.assertIn("尚未收到末端状态", transport)
        self.assertIn("末端状态已过期", transport)
        self.assertIn("末端状态显示未连接", transport)

        incremental_start = transport.index("bool move_cabin_incremental_via_driver(")
        incremental_end = transport.index("\n}  // namespace suoqu", incremental_start)
        incremental_body = transport[incremental_start:incremental_end]
        incremental_guard_index = incremental_body.index("reject_cabin_move_if_moduan_not_safe")
        remote_incremental_index = incremental_body.index('"/cabin/driver/incremental_move"')
        driver_incremental_index = incremental_body.index("::g_cabin_driver->moveByOffset")
        self.assertLess(incremental_guard_index, remote_incremental_index)
        self.assertLess(incremental_guard_index, driver_incremental_index)

    def test_remote_raw_move_failure_detail_is_cached_for_frontend_action_result(self):
        transport = (
            PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("kRemoteCabinRawMoveCallTimeoutSec", transport)
        self.assertIn("RemoteCabinSingleMoveCallResult", transport)
        self.assertIn("call_remote_cabin_single_move_service", transport)

        move_start = transport.index("bool move_cabin_pose_via_driver(")
        move_end = transport.index("\nbool move_cabin_incremental_via_driver", move_start)
        move_body = transport[move_start:move_end]
        remote_start = move_body.index('if (::use_remote_cabin_driver.load(std::memory_order_relaxed)) {')
        remote_end = move_body.index("\n    if (!::cabin_driver_enabled.load())", remote_start)
        remote_body = move_body[remote_start:remote_end]

        self.assertIn('"/cabin/driver/raw_move"', remote_body)
        self.assertIn("raw_move_call_result", remote_body)
        self.assertIn("raw_move_call_result.message", remote_body)
        self.assertIn("update_last_cabin_transport_error_detail(detail)", remote_body)
        self.assertIn("log_cabin_error_ros(detail)", remote_body)
        self.assertIn("目标=(", remote_body)
        self.assertIn("raw_move_call_result.success", remote_body)

        helper_start = transport.index("RemoteCabinSingleMoveCallResult call_remote_cabin_single_move_service")
        helper_end = transport.index("\nbool stop_cabin_motion_via_driver", helper_start)
        helper_body = transport[helper_start:helper_end]
        self.assertIn("std::promise<RemoteCabinSingleMoveCallResult>", helper_body)
        self.assertIn("wait_for(std::chrono::duration<double>(timeout_sec))", helper_body)
        self.assertIn("调用索驱驱动层服务超时", helper_body)

    def test_bind_from_scan_moves_directly_to_first_area_pose_without_origin_z_premove(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("bool run_bind_from_scan(")
        end = node.index("\n// service 编排已抽到", start)
        body = node[start:end]

        self.assertNotIn("bind_from_scan先回到规划原点", body)
        self.assertNotIn("bind_from_scan回到规划原点", body)
        self.assertNotIn("bind_from_scan回原点时", body)
        self.assertNotIn("move_path_origin_z", body)

        area_pose_index = body.index('const auto cabin_pose = area_json["cabin_pose"];')
        area_move_log_index = body.index("bind_from_scan区域%d移动到")
        self.assertLess(area_pose_index, area_move_log_index)

    def test_bind_from_scan_waits_for_moduan_idle_guard_after_successful_group(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool wait_for_moduan_post_bind_idle_guard(", node)
        helper_start = node.index("bool wait_for_moduan_post_bind_idle_guard(")
        helper_end = node.index("\nbool find_nearest_bind_area_for_current_cabin_pose", helper_start)
        helper_body = node[helper_start:helper_end]
        self.assertIn("kModuanPostBindIdleStableSamples", helper_body)
        self.assertIn("kModuanPostBindIdlePollMs", helper_body)
        self.assertIn("moduan_work_flag.load(std::memory_order_acquire)", helper_body)
        self.assertIn("std::this_thread::sleep_for", helper_body)

        start = node.index("bool run_bind_from_scan(")
        end = node.index("\n// service 编排已抽到", start)
        body = node[start:end]
        execute_index = body.index("if (!execute_moduan_bind_points_via_action")
        success_guard_index = body.index("wait_for_moduan_post_bind_idle_guard", execute_index)
        memory_write_index = body.index("if (use_execution_memory)", execute_index)
        self.assertLess(success_guard_index, memory_write_index)

    def test_frontend_pause_suspends_then_short_resume_continues_current_workflow(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("std::atomic<bool> execution_pause_requested{false};", node)
        self.assertIn("std::atomic<bool> execution_return_to_start_requested{false};", node)
        self.assertIn("bool is_execution_pause_requested()", node)
        self.assertIn("void request_execution_pause(", node)
        self.assertIn("void resume_execution_pause(", node)
        self.assertIn("bool wait_while_execution_paused(", node)

        callback_start = node.index("void pause_interrupt_Callback(")
        callback_end = node.index("\nvoid solve_stop_Callback", callback_start)
        callback_body = node[callback_start:callback_end]
        self.assertIn("request_execution_pause", callback_body)
        self.assertIn("stop_cabin_motion_via_driver", callback_body)
        self.assertNotIn("不可暂停索驱", callback_body)
        self.assertNotIn("&& !moduan_work_flag.load", callback_body)

        resume_start = node.index("void solve_stop_Callback(")
        resume_end = node.index("\n/*\n    函数功能：急停", resume_start)
        resume_body = node[resume_start:resume_end]
        self.assertIn("debug_mes.data == 1.0", resume_body)
        self.assertIn("resume_execution_pause", resume_body)
        self.assertNotIn("自动任务保持人工暂停锁定", resume_body)

        wait_start = node.index("bool wait_cabin_axis_stable_arrival(")
        wait_end = node.index("\n// 规划路径前下发速度", wait_start)
        wait_body = node[wait_start:wait_end]
        self.assertIn("wait_while_execution_paused", wait_body)
        self.assertIn("执行层等待轴", wait_body)
        self.assertIn("收到人工暂停", wait_body)
        pause_block = wait_body[wait_body.index("wait_while_execution_paused"):]
        self.assertIn("return false;", pause_block)
        pause_helper_start = node.index("bool wait_while_execution_paused(")
        pause_helper_end = node.index("\nstd::vector<uint8_t> build_pseudo_slam_ir_roi_frame", pause_helper_start)
        pause_helper_body = node[pause_helper_start:pause_helper_end]
        self.assertIn("move_cabin_pose_via_driver", pause_helper_body)

    def test_bind_from_scan_waits_on_short_pause_but_aborts_on_return_to_start(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("bool run_bind_from_scan(")
        end = node.index("\n// service 编排已抽到", start)
        body = node[start:end]

        self.assertIn("clear_execution_pause_request();", body)
        self.assertIn("record_execution_pause_return_pose", body)
        self.assertIn("wait_while_execution_paused", body)
        self.assertIn("fail_if_execution_return_to_start_requested", body)

        first_pause_guard = body.index("wait_while_execution_paused")
        first_area_move = body.index("bind_from_scan区域%d移动到")
        self.assertLess(first_pause_guard, first_area_move)

        failure_branch = body[body.index("if (!execute_moduan_bind_points_via_action"):]
        self.assertIn("if (is_execution_return_to_start_requested())", failure_branch)
        self.assertIn("return false;", failure_branch[:failure_branch.index("continue;")])

    def test_direct_bind_path_test_also_stops_on_long_press_return_to_start(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("bool run_bind_path_direct_test(")
        end = node.index("\nbool run_live_visual_global_work", start)
        body = node[start:end]

        self.assertIn("clear_execution_pause_request();", body)
        self.assertIn("record_execution_pause_return_pose", body)
        self.assertIn("wait_while_execution_paused", body)
        self.assertIn("fail_if_execution_return_to_start_requested", body)

        first_pause_guard = body.index("wait_while_execution_paused")
        first_area_move = body.index("bind_path_direct_test区域%d移动到")
        self.assertLess(first_pause_guard, first_area_move)

        failure_branch = body[body.index("if (!execute_moduan_bind_points_via_action"):]
        self.assertIn("if (is_execution_return_to_start_requested())", failure_branch)
        self.assertIn("return false;", failure_branch[:failure_branch.index("continue;")])
        self.assertIn("停止并回起点请求", body)

    def test_precomputed_execution_groups_are_sorted_as_tcp_snake_rows_after_filtering(self):
        area_execution = (
            PROCESS_DIR / "src" / "suoqu" / "area_execution.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("sort_precomputed_group_points_by_tcp_snake_rows", area_execution)
        helper_start = area_execution.index("void sort_precomputed_group_points_by_tcp_snake_rows(")
        helper_end = area_execution.index("\nstd::unordered_set<int> collect_blocked_execution_global_indices_from_points_json", helper_start)
        helper_body = area_execution[helper_start:helper_end]
        self.assertIn("kPrecomputedGroupSnakeRowToleranceMm", helper_body)
        self.assertIn('point_json.value("x"', helper_body)
        self.assertIn('"y"', helper_body)
        self.assertIn("row_index % 2U", helper_body)
        self.assertIn("ascending_y", helper_body)

        filter_start = area_execution.index("nlohmann::json filter_precomputed_group_points_for_execution(")
        filter_end = area_execution.index("\nstd::unordered_set<int> collect_blocked_execution_global_indices_from_points_json", filter_start)
        filter_body = area_execution[filter_start:filter_end]
        sort_index = filter_body.index("sort_precomputed_group_points_by_tcp_snake_rows(filtered_points);")
        return_index = filter_body.rindex("return filtered_points;")
        self.assertLess(sort_index, return_index)

    def test_live_visual_also_stops_on_long_press_return_to_start(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("bool run_live_visual_global_work(")
        end = node.index("\nbool run_planned_path_refine_only_global_work", start)
        body = node[start:end]

        self.assertIn("clear_execution_pause_request();", body)
        self.assertIn("record_execution_pause_return_pose", body)
        self.assertIn("wait_while_execution_paused", body)
        self.assertIn("fail_if_execution_return_to_start_requested", body)

        first_pause_guard = body.index("wait_while_execution_paused")
        first_area_move = body.index("live_visual区域%d移动到")
        self.assertLess(first_pause_guard, first_area_move)

        failure_branch = body[body.index("if (!execute_moduan_bind_points_via_action"):]
        self.assertIn("if (is_execution_return_to_start_requested())", failure_branch)
        self.assertIn("return false;", failure_branch[:failure_branch.index("continue;")])
        self.assertIn("停止并回起点请求", body)

    def test_live_visual_waits_for_moduan_idle_before_next_cabin_move(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("bool run_live_visual_global_work(")
        end = node.index("\nbool run_planned_path_refine_only_global_work", start)
        body = node[start:end]

        execution_index = body.index("execute_moduan_bind_points_via_action")
        wait_index = body.index("wait_for_moduan_post_bind_idle_guard", execution_index)
        success_count_index = body.index("executed_area_count++;", execution_index)
        self.assertLess(wait_index, success_count_index)

    def test_recover_pause_command_zeroes_moduan_before_returning_cabin_to_start(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool recover_paused_execution_to_start(", node)
        helper_start = node.index("bool recover_paused_execution_to_start(")
        helper_end = node.index("\nbool wait_for_moduan_post_bind_idle_guard", helper_start)
        helper_body = node[helper_start:helper_end]
        zero_index = helper_body.index('"/moduan/return_zero_ordered"')
        cabin_move_index = helper_body.index("move_cabin_pose_via_driver", zero_index)
        wait_x_index = helper_body.index("wait_cabin_axis_stable_arrival(AXIS_X", cabin_move_index)
        self.assertLess(zero_index, cabin_move_index)
        self.assertLess(cabin_move_index, wait_x_index)
        self.assertIn("record_execution_pause_return_pose", node)

        callback_start = node.index("void solve_stop_Callback(")
        callback_end = node.index("\n/*\n    函数功能：急停", callback_start)
        callback_body = node[callback_start:callback_end]
        self.assertIn("debug_mes.data == 2.0", callback_body)
        self.assertIn("前端长按停止当前作业并回起点", callback_body)
        self.assertIn("stop_cabin_motion_via_driver", callback_body)
        self.assertIn("recover_paused_execution_to_start", callback_body)
        stop_index = callback_body.index("stop_cabin_motion_via_driver")
        recover_index = callback_body.index("recover_paused_execution_to_start")
        self.assertLess(stop_index, recover_index)
        stop_failure_body = callback_body[callback_body.index("if (!stop_cabin_motion_via_driver"):]
        self.assertIn("return;", stop_failure_body[:stop_failure_body.index("recover_paused_execution_to_start")])

        control = (CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp").read_text(encoding="utf-8")
        zero_start = control.index("bool move_linear_module_to_origin()")
        zero_end = control.index("\ndouble max_bind_height_excess_mm", zero_start)
        zero_body = control[zero_start:zero_end]
        z_set_index = zero_body.index("Set_Module_Coordinate(WZ_COORDINATE")
        z_wait_index = zero_body.index("wait_linear_module_axis_arrival(AXIS_Z")
        x_set_index = zero_body.index("Set_Module_Coordinate(WX_COORDINATE")
        y_set_index = zero_body.index("Set_Module_Coordinate(WY_COORDINATE")
        self.assertLess(z_set_index, z_wait_index)
        self.assertLess(z_wait_index, x_set_index)
        self.assertLess(z_wait_index, y_set_index)

        callbacks = (CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp").read_text(encoding="utf-8")
        self.assertIn("先抬升Z轴回0，再回X/Y到0", callbacks)

    def test_cabin_stop_and_move_do_not_trigger_moduan_is_zero(self):
        transport = (PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp").read_text(encoding="utf-8")

        self.assertNotIn("trigger_moduan_is_zero_before_cabin_motion", transport)
        self.assertNotIn('"/moduan/request_is_zero"', transport)
        self.assertNotIn("末端IS_ZERO前置服务", transport)

        stop_start = transport.index("bool stop_cabin_motion_via_driver(")
        stop_end = transport.index("\nbool move_cabin_pose_via_driver", stop_start)
        stop_body = transport[stop_start:stop_end]
        stop_remote_index = stop_body.index("if (::use_remote_cabin_driver")
        stop_enabled_index = stop_body.index("if (!::cabin_driver_enabled")
        self.assertLess(stop_remote_index, stop_enabled_index)

        move_start = transport.index("bool move_cabin_pose_via_driver(")
        move_end = transport.index("\nbool move_cabin_incremental_via_driver", move_start)
        move_body = transport[move_start:move_end]
        safe_index = move_body.index("reject_cabin_move_if_moduan_not_safe")
        raw_move_index = move_body.index('"/cabin/driver/raw_move"')
        direct_move_index = move_body.index("::g_cabin_driver->moveToPose")
        self.assertLess(safe_index, raw_move_index)
        self.assertLess(safe_index, direct_move_index)

        incremental_start = transport.index("bool move_cabin_incremental_via_driver(")
        incremental_body = transport[incremental_start:]
        incremental_safe_index = incremental_body.index("reject_cabin_move_if_moduan_not_safe")
        incremental_remote_index = incremental_body.index('"/cabin/driver/incremental_move"')
        incremental_direct_index = incremental_body.index("::g_cabin_driver->moveByOffset")
        self.assertLess(incremental_safe_index, incremental_remote_index)
        self.assertLess(incremental_safe_index, incremental_direct_index)

    def test_recover_pause_command_falls_back_to_bind_path_origin(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool load_execution_pause_return_pose_from_bind_path_json(", node)
        helper_start = node.index("bool recover_paused_execution_to_start(")
        helper_end = node.index("\nbool wait_for_moduan_post_bind_idle_guard", helper_start)
        helper_body = node[helper_start:helper_end]

        missing_record_index = helper_body.index("if (!get_execution_pause_return_pose(return_pose))")
        fallback_index = helper_body.index("load_execution_pause_return_pose_from_bind_path_json", missing_record_index)
        zero_index = helper_body.index('"/moduan/return_zero_ordered"')
        self.assertLess(missing_record_index, fallback_index)
        self.assertLess(fallback_index, zero_index)
        self.assertIn("也无法从pseudo_slam_bind_path.json恢复", helper_body)
        self.assertIn("record_execution_pause_return_pose(", helper_body[fallback_index:zero_index])

    def test_live_visual_records_planning_origin_for_pause_return(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("bool run_live_visual_global_work(")
        end = node.index("\nbool run_planned_path_refine_only_global_work", start)
        body = node[start:end]

        self.assertIn("clear_execution_pause_request();", body)
        self.assertIn("record_execution_pause_return_pose(", body)

        clear_index = body.index("clear_execution_pause_request();")
        workflow_lock_index = body.index("std::lock_guard<std::mutex> pseudo_slam_workflow_lock")
        self.assertLess(clear_index, workflow_lock_index)

        record_index = body.index("record_execution_pause_return_pose(")
        origin_move_index = body.index("if (!move_cabin_pose_for_automatic_execution(")
        self.assertLess(record_index, origin_move_index)
        record_body = body[record_index:origin_move_index]
        self.assertIn("path_origin_x", record_body)
        self.assertIn("path_origin_y", record_body)
        self.assertIn("move_path_origin_z", record_body)

    def test_planned_path_refine_only_waits_and_retries_single_bind_after_arrival(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool wait_for_planned_path_pre_bind_settle(", node)
        self.assertIn("bool call_sg_live_visual_with_no_points_retry(", node)
        self.assertIn("constexpr int kPlannedPathLargeZPreBindSettleMs = 300;", node)
        self.assertIn("constexpr int kPlannedPathNoPointsRetrySettleMs = 300;", node)

        start = node.index("bool run_planned_path_refine_only_global_work(")
        end = node.index("\nbool run_bind_from_scan(", start)
        body = node[start:end]

        arrival_index = body.index("if (!wait_cabin_axis_stable_arrival")
        settle_index = body.index("wait_for_planned_path_pre_bind_settle", arrival_index)
        jump_snapshot_index = body.index("const bool jump_bind_enabled_snapshot", arrival_index)
        self.assertLess(settle_index, jump_snapshot_index)

        self.assertNotIn("sg_live_visual_client.call(bind_srv)", body)
        self.assertIn("call_sg_live_visual_with_no_points_retry", body)

        helper_start = node.index("bool call_sg_live_visual_with_no_points_retry(")
        helper_end = node.index("\nstd::vector<uint8_t> build_pseudo_slam_ir_roi_frame", helper_start)
        helper_body = node[helper_start:helper_end]
        self.assertIn("EXECUTION_REFINE_NO_POINTS", helper_body)
        self.assertIn("kPlannedPathNoPointsRetrySettleMs", helper_body)
        self.assertIn("sg_live_visual_client.call", helper_body)
        self.assertGreaterEqual(helper_body.count("sg_live_visual_client.call"), 2)

    def test_execution_chain_logs_missing_vision_service_dependency(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool call_execution_refine_vision_service(", node)
        helper_start = node.index("bool call_execution_refine_vision_service(")
        helper_end = node.index("\nbool is_execution_refine_no_points_response", helper_start)
        helper_body = node[helper_start:helper_end]
        self.assertIn("/pointAI/process_image", helper_body)
        self.assertIn("pointAINode", helper_body)
        self.assertIn("AI_client.exists()", helper_body)
        self.assertIn("Cabin_Error", helper_body)

        live_start = node.index("bool run_live_visual_global_work(")
        live_end = node.index("\nbool run_planned_path_refine_only_global_work", live_start)
        live_body = node[live_start:live_end]
        self.assertIn("call_execution_refine_vision_service", live_body)
        self.assertNotIn("AI_client.call(scan_srv)", live_body)

        planned_start = node.index("bool run_planned_path_refine_only_global_work(")
        planned_end = node.index("\nbool run_bind_from_scan(", planned_start)
        planned_body = node[planned_start:planned_end]
        self.assertIn("call_execution_refine_vision_service", planned_body)
        self.assertNotIn("AI_client.call(scan_srv)", planned_body)

    def test_planned_path_refine_only_logs_moduan_sg_vision_dependency_when_service_missing(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        helper_start = node.index("bool call_sg_live_visual_with_no_points_retry(")
        helper_end = node.index("\nstd::vector<uint8_t> build_pseudo_slam_ir_roi_frame", helper_start)
        helper_body = node[helper_start:helper_end]
        self.assertIn("/moduan/sg", helper_body)
        self.assertIn("/pointAI/process_image", helper_body)
        self.assertIn("pointAINode", helper_body)
        self.assertIn("sg_live_visual_client.exists()", helper_body)
        self.assertIn("Cabin_Error", helper_body)

    def test_execution_chain_keeps_progress_when_vision_service_dependency_is_missing(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        live_start = node.index("bool run_live_visual_global_work(")
        live_end = node.index("\nbool run_planned_path_refine_only_global_work", live_start)
        live_body = node[live_start:live_end]
        live_call_failure_start = live_body.index("if (!call_execution_refine_vision_service")
        live_call_failure_end = live_body.index(
            "\n        if (!scan_srv.response.success)",
            live_call_failure_start,
        )
        live_call_failure_block = live_body[live_call_failure_start:live_call_failure_end]
        self.assertIn("Cabin_Error", live_call_failure_block)
        self.assertIn("视觉依赖不可用", live_call_failure_block)
        self.assertIn("skipped_area_count++", live_call_failure_block)
        self.assertIn("continue;", live_call_failure_block)
        live_error_log_index = live_call_failure_block.index("Cabin_Error")
        live_continue_index = live_call_failure_block.index("continue;", live_error_log_index)
        self.assertLess(live_error_log_index, live_continue_index)

        planned_start = node.index("bool run_planned_path_refine_only_global_work(")
        planned_end = node.index("\nbool run_bind_from_scan(", planned_start)
        planned_body = node[planned_start:planned_end]

        sg_call_failure_start = planned_body.index("if (!call_sg_live_visual_with_no_points_retry")
        sg_call_failure_end = planned_body.index(
            "\n            if (!bind_srv.response.success)",
            sg_call_failure_start,
        )
        sg_call_failure_block = planned_body[sg_call_failure_start:sg_call_failure_end]
        self.assertIn("Cabin_Error", sg_call_failure_block)
        self.assertIn("单点绑扎/视觉依赖不可用", sg_call_failure_block)
        self.assertIn("skipped_area_count++", sg_call_failure_block)
        self.assertIn("continue;", sg_call_failure_block)
        sg_error_log_index = sg_call_failure_block.index("Cabin_Error")
        sg_continue_index = sg_call_failure_block.index("continue;", sg_error_log_index)
        self.assertLess(sg_error_log_index, sg_continue_index)

        jump_call_failure_start = planned_body.index("if (!call_execution_refine_vision_service")
        jump_call_failure_end = planned_body.index(
            "\n        } else if (!scan_srv.response.success)",
            jump_call_failure_start,
        )
        jump_call_failure_block = planned_body[jump_call_failure_start:jump_call_failure_end]
        self.assertIn("refine_failure_reason = vision_call_message;", jump_call_failure_block)
        self.assertNotIn("return false;", jump_call_failure_block)

    def test_ledger_refine_axis_threshold_is_hot_configurable(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        scan_processing = (
            PROCESS_DIR / "src" / "suoqu" / "pseudo_slam_scan_processing.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "std::atomic<float> live_visual_refine_axis_threshold_mm",
            runtime_header,
        )
        self.assertIn(
            "std::atomic<float> live_visual_refine_axis_threshold_mm{kPseudoSlamCheckerboardAxisThresholdMm};",
            node,
        )
        self.assertIn(
            'nh.subscribe("/web/cabin/set_ledger_refine_axis_threshold_mm"',
            node,
        )
        self.assertIn("void ledger_refine_axis_threshold_callback", node)

        classify_start = scan_processing.index("bool classify_live_visual_point_into_checkerboard(")
        classify_end = scan_processing.index(
            "\nstd::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_non_checkerboard_points",
            classify_start,
        )
        classify_body = scan_processing[classify_start:classify_end]
        self.assertIn("live_visual_refine_axis_threshold_mm.load", classify_body)
        self.assertIn("axis_threshold_mm", classify_body)
        self.assertNotIn("> kPseudoSlamCheckerboardAxisThresholdMm", classify_body)

    def test_planned_path_refine_only_stops_after_unsafe_moduan_failure(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool is_unsafe_moduan_execution_failure(", node)
        helper_start = node.index("bool is_unsafe_moduan_execution_failure(")
        helper_end = node.index("\nbool wait_for_planned_path_settle_duration", helper_start)
        helper_body = node[helper_start:helper_end]
        self.assertIn("FINISHALL", helper_body)
        self.assertIn("线性模组", helper_body)
        self.assertIn("未确认完成", helper_body)

        start = node.index("bool run_planned_path_refine_only_global_work(")
        end = node.index("\nbool run_bind_from_scan(", start)
        body = node[start:end]

        single_failure_start = body.index("if (!bind_srv.response.success)")
        single_failure_end = body.index("std::string post_bind_idle_guard_message", single_failure_start)
        single_failure_body = body[single_failure_start:single_failure_end]
        self.assertIn(
            "is_unsafe_moduan_execution_failure(bind_srv.response.message)",
            single_failure_body,
        )
        single_unsafe_index = single_failure_body.index(
            "is_unsafe_moduan_execution_failure(bind_srv.response.message)"
        )
        single_return_index = single_failure_body.index("return false;", single_unsafe_index)
        single_continue_index = single_failure_body.index("continue;", single_unsafe_index)
        self.assertLess(single_return_index, single_continue_index)
        self.assertIn("阻止后续索驱移动", single_failure_body)

        jump_failure_start = body.index("if (!execute_moduan_bind_points_via_action")
        jump_failure_end = body.index("std::string post_bind_idle_guard_message", jump_failure_start)
        jump_failure_body = body[jump_failure_start:jump_failure_end]
        self.assertIn(
            "is_unsafe_moduan_execution_failure(bind_action_message)",
            jump_failure_body,
        )
        jump_unsafe_index = jump_failure_body.index(
            "is_unsafe_moduan_execution_failure(bind_action_message)"
        )
        jump_return_index = jump_failure_body.index("return false;", jump_unsafe_index)
        jump_continue_index = jump_failure_body.index("continue;", jump_unsafe_index)
        self.assertLess(jump_return_index, jump_continue_index)
        self.assertIn("阻止后续索驱移动", jump_failure_body)

    def test_live_visual_checkerboard_matching_uses_inferred_grid_world_axes(self):
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        scan_processing = (
            PROCESS_DIR / "src" / "suoqu" / "pseudo_slam_scan_processing.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("DynamicBindWorldAxis row_world_axis", runtime_header)
        self.assertIn("DynamicBindWorldAxis col_world_axis", runtime_header)
        self.assertIn("infer_dynamic_bind_grid_axis_mapping", scan_processing)

        load_start = scan_processing.index("bool load_live_visual_checkerboard_grid(")
        load_end = scan_processing.index("\nbool classify_live_visual_point_into_checkerboard", load_start)
        load_body = scan_processing[load_start:load_end]
        self.assertIn("axis_mapping.row_axis", load_body)
        self.assertIn("axis_mapping.col_axis", load_body)

        classify_start = scan_processing.index("bool classify_live_visual_point_into_checkerboard(")
        classify_end = scan_processing.index("\nstd::vector<tie_robot_msgs::PointCoords> filter_pseudo_slam_non_checkerboard_points", classify_start)
        classify_body = scan_processing[classify_start:classify_end]
        self.assertIn("checkerboard_grid.row_world_axis", classify_body)
        self.assertIn("checkerboard_grid.col_world_axis", classify_body)
        self.assertIn("get_dynamic_bind_world_axis_value", classify_body)

    def test_live_visual_micro_adjust_acceptance_has_no_xy_gate_and_keeps_refined_z(self):
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertNotIn("kLiveVisualMicroAdjustXYToleranceMm", runtime_header)
        self.assertNotIn("kLiveVisualMicroAdjustZToleranceMm", runtime_header)

        start = suoqu_node.index("nlohmann::json build_live_visual_execution_points_from_planned_area(")
        end = suoqu_node.index("\nbool load_precomputed_local_points_from_group_json", start)
        helper_body = suoqu_node[start:end]

        self.assertIn("const double refine_score = refine_dx_mm + refine_dy_mm;", helper_body)
        self.assertIn('execution_point_json["world_z"] = live_world_z;', helper_body)
        self.assertNotIn("kLiveVisualMicroAdjustXYToleranceMm", helper_body)
        self.assertNotIn("xy阈值", helper_body)
        self.assertNotIn("refine_dz_mm", helper_body)
        self.assertNotIn("kLiveVisualMicroAdjustZToleranceMm", helper_body)
        self.assertNotIn("z阈值", helper_body)
        self.assertNotIn("超出xy微调范围", helper_body)
        self.assertNotIn("超出xyz微调范围", helper_body)

    def test_frontend_pause_resume_uses_long_press_for_return_to_start(self):
        catalog = (
            WEB_DIR / "frontend" / "src" / "config" / "controlPanelCatalog.js"
        ).read_text(encoding="utf-8")
        legacy = (
            WEB_DIR / "frontend" / "src" / "config" / "legacyCommandCatalog.js"
        ).read_text(encoding="utf-8")
        controller = (
            WEB_DIR / "frontend" / "src" / "controllers" / "LegacyCommandController.js"
        ).read_text(encoding="utf-8")
        ui = (WEB_DIR / "frontend" / "src" / "ui" / "UIController.js").read_text(encoding="utf-8")

        pause_start = catalog.index("pauseResume:")
        pause_end = catalog.index("\n  lashingEnabled:", pause_start)
        pause_body = catalog[pause_start:pause_end]
        self.assertIn("inactiveRequiresLongPress", pause_body)
        self.assertIn("inactiveLongPressCommandId: 25", pause_body)
        self.assertIn("activeRequiresLongPress", pause_body)
        self.assertIn("longPressCommandId: 25", pause_body)

        self.assertIn('{ id: 25, name: "停止并回起点"', legacy)
        self.assertIn("command.id === 25", controller)
        self.assertIn("return { data: 2 };", controller)
        self.assertIn("handleToggleLongPress", controller)
        self.assertIn("inactiveRequiresLongPress", controller)
        self.assertIn("inactiveLongPressCommandId", controller)
        self.assertIn("activeRequiresLongPress", controller)
        self.assertIn("const longPressCommandId = currentValue", controller)
        self.assertIn("definition.inactiveLongPressCommandId", controller)
        self.assertIn("当前工作将停止，线性模组按Z优先回到(0,0,0)，索驱回到执行起点", controller)
        self.assertIn("状态=停止并回起点", controller)
        self.assertNotIn("短按保持人工暂停锁定", controller)
        self.assertNotIn("状态=恢复回起点", controller)
        short_press_body = controller[controller.index("handleToggle(toggleId, parameters)"):]
        short_press_body = short_press_body[:short_press_body.index("handleToggleLongPress")]
        self.assertIn("resolveToggleCommand", short_press_body)
        self.assertNotIn("activeRequiresLongPress", short_press_body)
        self.assertIn("onControlToggle(callback)", ui)
        self.assertIn("longPressCommandId", ui)
        self.assertIn("is-long-press-charging", ui)

    def test_manual_area_takeover_abandons_paused_auto_chain_without_returning_origin(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        moduan_callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        moduan_executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")
        runtime_header = (
            CONTROL_DIR
            / "include"
            / "tie_robot_control"
            / "moduan"
            / "runtime_state.hpp"
        ).read_text(encoding="utf-8")
        runtime_state = (
            CONTROL_DIR / "src" / "moduan" / "runtime_state.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("std::atomic<bool> execution_manual_takeover_requested{false};", suoqu_node)
        self.assertIn("void request_execution_manual_takeover", suoqu_node)
        self.assertIn("void manual_area_takeover_Callback", suoqu_node)
        manual_callback = suoqu_node[
            suoqu_node.index("void manual_area_takeover_Callback"):
            suoqu_node.index("\nvoid forced_stop_nodeCallback", suoqu_node.index("void manual_area_takeover_Callback"))
        ]
        self.assertIn("request_execution_manual_takeover(\"前端手动切换工作区域\")", manual_callback)
        self.assertIn("stop_cabin_motion_via_driver", manual_callback)
        self.assertNotIn("recover_paused_execution_to_start", manual_callback)
        self.assertIn('nh.subscribe("/web/cabin/manual_area_takeover"', suoqu_node)
        self.assertIn("is_execution_manual_takeover_requested()", suoqu_node)

        self.assertIn("moduan_manual_takeover_requested", runtime_header)
        self.assertIn("std::atomic<bool> moduan_manual_takeover_requested(false);", runtime_state)
        self.assertIn("void manual_area_takeover_callback", moduan_callbacks)
        self.assertIn('subscribe("/web/cabin/manual_area_takeover"', moduan_callbacks)
        self.assertIn("moduan_manual_takeover_requested.store(true", moduan_callbacks)
        self.assertIn("moduan_manual_takeover_requested.load", moduan_executor)

    def test_cabin_driver_retries_absolute_move_once_after_retryable_transport_close(self):
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")

        self.assertIn("kMoveToPoseTransportAttemptCount", driver)
        self.assertIn("kMoveToPoseReconnectRetryDelay", driver)
        self.assertIn("bool isRetryableAbsoluteMoveTransportError(", driver)
        helper_start = driver.index("bool isRetryableAbsoluteMoveTransportError(")
        helper_end = driver.index("\nvoid resetTransportAfterProtocolDesync", helper_start)
        helper_body = driver[helper_start:helper_end]
        self.assertIn("error.retryable", helper_body)
        self.assertIn('error.code == "tcp_recv_failed"', helper_body)
        self.assertIn('error.code == "tcp_read_wait_failed"', helper_body)
        self.assertIn('error.code == "tcp_send_failed"', helper_body)

        start = driver.index("bool CabinDriver::moveToPose(")
        end = driver.index("\nbool CabinDriver::moveByOffset", start)
        body = driver[start:end]
        self.assertIn("for (int attempt = 0; attempt < kMoveToPoseTransportAttemptCount; ++attempt)", body)
        self.assertIn("isRetryableAbsoluteMoveTransportError(transport_error)", body)
        self.assertIn("transport_->disconnect();", body)
        self.assertIn("std::this_thread::sleep_for(kMoveToPoseReconnectRetryDelay)", body)
        self.assertIn("continue;", body)
        send_index = body.index("transport_->sendAndReceive")
        retry_index = body.index("isRetryableAbsoluteMoveTransportError(transport_error)")
        self.assertLess(send_index, retry_index)

        offset_start = driver.index("bool CabinDriver::moveByOffset(")
        offset_end = driver.index("\nbool CabinDriver::sendStop", offset_start)
        offset_body = driver[offset_start:offset_end]
        self.assertNotIn("kMoveToPoseTransportAttemptCount", offset_body)
        self.assertNotIn("isRetryableAbsoluteMoveTransportError", offset_body)

    def test_cabin_stop_treats_peer_close_after_stop_frame_as_delivered(self):
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")
        protocol = (HW_DIR / "src" / "driver" / "cabin_protocol.cpp").read_text(encoding="utf-8")

        self.assertIn("std::vector<uint8_t> CabinProtocol::buildStopFrame()", protocol)
        self.assertIn("0x13", protocol)

        start = driver.index("bool CabinDriver::sendStop(")
        end = driver.index("\nCabinStateSnapshot CabinDriver::readState", start)
        body = driver[start:end]
        self.assertIn("CabinProtocol::buildStopFrame()", body)
        self.assertIn('driver_error.code == "tcp_recv_failed"', body)
        self.assertIn('driver_error.detail.find("connection closed by peer")', body)
        self.assertNotIn('driver_error.detail == "connection closed by peer"', body)
        self.assertIn("transport_->disconnect();", body)
        self.assertIn("error->clear();", body)

    def test_cabin_stop_treats_device_not_moving_status_as_delivered(self):
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")

        self.assertIn("bool isStopAlreadyIdleStatus(", driver)
        helper_start = driver.index("bool isStopAlreadyIdleStatus(")
        helper_end = driver.index("\nvoid resetTransportAfterProtocolDesync", helper_start)
        helper_body = driver[helper_start:helper_end]
        self.assertIn('error.code == "motion_command_rejected"', helper_body)
        self.assertIn('error.message == "索驱上位机拒绝停止指令"', helper_body)
        self.assertIn('error.detail.find("status_word=0x00000004")', helper_body)
        self.assertIn('error.detail.find("设备未运动")', helper_body)

        start = driver.index("bool CabinDriver::sendStop(")
        end = driver.index("\nCabinStateSnapshot CabinDriver::readState", start)
        body = driver[start:end]
        decode_index = body.index("DriverError protocol_error = CabinProtocol::decodeStatus(0x0013, response);")
        idle_index = body.index("isStopAlreadyIdleStatus(protocol_error)", decode_index)
        failure_index = body.index("if (!protocol_error.code.empty())", idle_index)
        self.assertLess(idle_index, failure_index)
        self.assertIn("error->clear();", body[idle_index:failure_index])
        self.assertIn("return true;", body[idle_index:failure_index])

    def test_cabin_driver_keeps_tcp_position_relative_frame_for_optional_remote_mode(self):
        protocol_header = (HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_protocol.hpp").read_text(encoding="utf-8")
        protocol = (HW_DIR / "src" / "driver" / "cabin_protocol.cpp").read_text(encoding="utf-8")
        driver_header = (HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_driver.hpp").read_text(encoding="utf-8")
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        topic_registry = (
            WORKSPACE_ROOT / "tie_robot_web" / "frontend" / "src" / "config" / "topicRegistry.js"
        ).read_text(encoding="utf-8")

        self.assertIn("buildRelativeMoveFrame", protocol_header)
        self.assertIn("std::vector<uint8_t> CabinProtocol::buildIncrementalMoveFrame", protocol)
        self.assertIn("std::vector<uint8_t> CabinProtocol::buildRelativeMoveFrame", protocol)
        self.assertIn("return buildPositionMoveFrame(command, 0x0002);", protocol)
        self.assertIn("frame.push_back(0x11);", protocol)
        self.assertIn("bool moveByOffset(const CabinPoseCommand& command, DriverError* error);", driver_header)
        start = driver.index("bool CabinDriver::moveByOffset(")
        end = driver.index("\nbool CabinDriver::sendStop", start)
        body = driver[start:end]
        self.assertIn("CabinProtocol::buildRelativeMoveFrame(command)", body)
        self.assertIn("decodeStatus(0x0012", body)
        self.assertNotIn("CabinProtocol::buildIncrementalMoveFrame(command)", body)
        self.assertNotIn("decodeStatus(0x0011", body)
        self.assertIn('nh.advertiseService("/cabin/driver/incremental_move"', node)
        self.assertIn('incrementalMove: "/cabin/driver/incremental_move"', topic_registry)

    def test_cabin_driver_resets_socket_after_desynchronized_motion_response(self):
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")

        self.assertIn("resetTransportAfterProtocolDesync", driver)
        helper_start = driver.index("void resetTransportAfterProtocolDesync(")
        helper_end = driver.index("\nCabinDriver::CabinDriver", helper_start)
        helper_body = driver[helper_start:helper_end]
        self.assertIn('protocol_error.code != "protocol_response_desynchronized"', helper_body)
        self.assertIn("transport.disconnect();", helper_body)
        self.assertIn("避免TCP相对运动重复执行", helper_body)

        start = driver.index("bool CabinDriver::moveByOffset(")
        end = driver.index("\nbool CabinDriver::sendStop", start)
        body = driver[start:end]
        self.assertIn("resetTransportAfterProtocolDesync(*transport_, protocol_error);", body)
        self.assertNotIn("for (int attempt", body)

    def test_cabin_tcp_transport_errors_include_request_frame_context(self):
        transport = (HW_DIR / "src" / "driver" / "cabin_tcp_transport.cpp").read_text(encoding="utf-8")

        self.assertIn("std::string formatRequestContext(const std::vector<uint8_t>& request)", transport)
        self.assertIn("request_command=", transport)
        self.assertIn("request_frame=[", transport)
        send_start = transport.index('"tcp_send_failed"')
        recv_start = transport.index('"tcp_recv_failed"')
        wait_start = transport.index('"tcp_read_wait_failed"')
        self.assertIn("appendRequestContext(", transport[send_start:send_start + 500])
        self.assertIn("appendRequestContext(", transport[wait_start:wait_start + 500])
        self.assertIn("appendRequestContext(", transport[recv_start:recv_start + 500])

    def test_cabin_tcp_transport_drains_stale_state_bytes_before_motion_send(self):
        transport_header = (
            HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_tcp_transport.hpp"
        ).read_text(encoding="utf-8")
        transport = (HW_DIR / "src" / "driver" / "cabin_tcp_transport.cpp").read_text(encoding="utf-8")

        self.assertIn("std::size_t drainPendingInputLocked();", transport_header)
        self.assertIn("std::size_t CabinTcpTransport::drainPendingInputLocked()", transport)
        self.assertIn("bool receiveExactLocked(", transport_header)
        self.assertIn("bool CabinTcpTransport::receiveExactLocked(", transport)
        self.assertIn("MSG_DONTWAIT", transport)
        self.assertIn("looksLikeCabinStateResponsePrefix", transport)
        self.assertIn("kCabinStateResponseBytes - kCabinMotionResponseBytes", transport)

        send_start = transport.index("bool CabinTcpTransport::sendAndReceive(")
        send_body = transport[send_start:transport.index("\nvoid CabinTcpTransport::markExternalIoSuccess", send_start)]
        drain_index = send_body.index("drainPendingInputLocked();")
        send_index = send_body.index("::send(")
        read_index = send_body.index("receiveExactLocked(expected_bytes")
        skip_index = send_body.index("looksLikeCabinStateResponsePrefix")
        self.assertLess(drain_index, send_index)
        self.assertLess(send_index, read_index)
        self.assertLess(read_index, skip_index)

    def test_legacy_suoqu_tcp_position_status_decoder_uses_32_bit_dictionary(self):
        header = (
            PROCESS_DIR / "include" / "tie_robot_process" / "suoqu" / "cabin_transport.hpp"
        ).read_text(encoding="utf-8")
        transport = (
            PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("std::atomic<uint32_t> pending_tcp_status_word", header)
        self.assertIn("uint32_t status_word = 0;", header)
        self.assertIn("uint32_t extract_tcp_protocol_status_word", header)
        self.assertIn(
            'append_tcp_status_reason_if_set(reasons, status_word, 16, "C超负限位")',
            transport,
        )

        extract_start = transport.index("uint32_t extract_tcp_protocol_status_word(")
        extract_end = transport.index("\nTcpProtocolStatusDecode decode_tcp_protocol_status", extract_start)
        extract_body = transport[extract_start:extract_end]
        position_case = extract_body[extract_body.index("case 0x0012:"):]
        self.assertIn("(static_cast<uint32_t>(buffer[2]) << 24)", position_case)
        self.assertIn("(static_cast<uint32_t>(buffer[3]) << 16)", position_case)
        self.assertIn("(static_cast<uint32_t>(buffer[4]) << 8)", position_case)
        self.assertIn("static_cast<uint32_t>(buffer[5])", position_case)
        self.assertNotIn("static_cast<uint16_t>(buffer[4])", position_case)

        self.assertIn("std::atomic<uint32_t> pending_tcp_status_word{0};", node)
        self.assertIn("status_word=0x%08X", node)
        self.assertIn("状态字异常0x%08X", node)

    def test_cabin_protocol_retry_refreshes_socket_after_reconnect(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("int Frame_Generate_With_Retry(")
        end = node.index("\nvoid solve_stop", start)
        body = node[start:end]
        invalid_socket_guard_index = body.index("if (socket < 0)")
        first_frame_generate_index = body.index("Frame_Generate(Control_Word")
        reconnect_index = body.index("connectToServer()")
        refresh_index = body.index("socket = sockfd;", reconnect_index)
        resend_index = body.index("正在尝试重新发送指令", reconnect_index)
        self.assertLess(invalid_socket_guard_index, first_frame_generate_index)
        self.assertLess(reconnect_index, refresh_index)
        self.assertLess(refresh_index, resend_index)

    def test_cabin_state_command_has_explicit_debug_name(self):
        transport = (
            PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")

        command_name_start = transport.index("const char* tcp_protocol_command_name(")
        command_name_end = transport.index("\nnamespace {", command_name_start)
        command_name_body = transport[command_name_start:command_name_end]

        self.assertIn('case 0x0001: return "索驱状态查询";', command_name_body)

    def test_cabin_protocol_retry_drops_stale_transport_before_reconnect(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("int Frame_Generate_With_Retry(")
        end = node.index("\nvoid solve_stop", start)
        body = node[start:end]

        reconnect_log_index = body.index("正在尝试与索驱上位机重新创建TCP连接")
        reconnect_call_index = body.index("connectToServer()", reconnect_log_index)
        stale_drop_index = body.index("drop_stale_cabin_transport_before_retry", reconnect_log_index)
        stop_index = body.index("g_cabin_driver->stop();", stale_drop_index)
        sync_index = body.index("sync_global_socket_fd_from_cabin_driver();", stale_drop_index)
        socket_refresh_index = body.index("socket = sockfd;", stale_drop_index)

        self.assertLess(stale_drop_index, reconnect_call_index)
        self.assertLess(stop_index, reconnect_call_index)
        self.assertLess(sync_index, reconnect_call_index)
        self.assertLess(socket_refresh_index, reconnect_call_index)

    def test_cabin_tcp_sends_do_not_raise_sigpipe_on_peer_close(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        transport = (HW_DIR / "src" / "driver" / "cabin_tcp_transport.cpp").read_text(encoding="utf-8")

        frame_start = node.index("\nint Frame_Generate(uint8_t* Control_Word")
        frame_end = node.index("\nint Frame_Generate_With_Retry", frame_start)
        frame_body = node[frame_start:frame_end]
        self.assertIn("MSG_NOSIGNAL", frame_body)
        self.assertNotIn("send(socket, Control_Word + total_sent, Tlen - total_sent, 0)", frame_body)

        send_start = transport.index("bool CabinTcpTransport::sendAndReceive(")
        send_end = transport.index("\nvoid CabinTcpTransport::markExternalIoSuccess", send_start)
        send_body = transport[send_start:send_end]
        self.assertIn("MSG_NOSIGNAL", send_body)
        self.assertNotIn("request.size() - total_sent,\n            0", send_body)

    def test_cabin_protocol_retry_keeps_requesting_instead_of_emergency_exit(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("int Frame_Generate_With_Retry(")
        end = node.index("\nvoid solve_stop", start)
        body = node[start:end]

        self.assertIn("while (ros::ok())", body)
        self.assertIn("持续请求索驱", body)
        self.assertIn("继续请求索驱", body)
        self.assertNotIn("重新发送命令失败超过5次", body)
        self.assertNotIn("重新连接失败超过5次", body)
        self.assertNotIn("emergency_exit_with_flush(4)", body)

    def test_legacy_frame_retry_only_waits_on_pure_motion_busy_status(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("int Frame_Generate_With_Retry(")
        end = node.index("\nvoid solve_stop", start)
        body = node[start:end]

        self.assertNotIn("keep_retrying_on_status_reject", body)
        self.assertIn("const uint16_t status_command_word =", body)
        self.assertIn("pending_tcp_status_command_word.load(std::memory_order_relaxed)", body)
        self.assertIn(
            "if (is_transient_cabin_motion_status(status_command_word, status_word))",
            body,
        )

        transient_index = body.index("if (is_transient_cabin_motion_status")
        continue_index = body.index("continue;", transient_index)
        hard_failure_index = body.index("return -2;", continue_index)
        self.assertLess(transient_index, continue_index)
        self.assertLess(continue_index, hard_failure_index)
        self.assertIn("索驱上位机拒绝当前运动指令", body[continue_index:hard_failure_index])

    def test_cabin_state_poll_uses_legacy_100ms_socket_loop(self):
        transport_header = (
            HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_tcp_transport.hpp"
        ).read_text(encoding="utf-8")
        transport = (HW_DIR / "src" / "driver" / "cabin_tcp_transport.cpp").read_text(encoding="utf-8")
        driver_header = (
            HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_driver.hpp"
        ).read_text(encoding="utf-8")
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")
        protocol_header = (
            HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_protocol.hpp"
        ).read_text(encoding="utf-8")
        protocol = (HW_DIR / "src" / "driver" / "cabin_protocol.cpp").read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool sendAndReceive(", transport_header)
        self.assertNotIn("bool pollState(", driver_header)
        self.assertNotIn("CabinDriver::pollState(", driver)
        self.assertIn("decodeHeartbeatState(", protocol_header)
        self.assertIn("DriverError CabinProtocol::decodeHeartbeatState(", protocol)
        self.assertNotIn("constexpr int kCabinStatePollIntervalMs = 20;", node)

        read_state_start = node.index("void read_cabin_state(")
        read_state_end = node.index("\nint RunSuoquNodeWithDefaultRole(", read_state_start)
        read_state_body = node[read_state_start:read_state_end]
        self.assertIn("std::this_thread::sleep_for(std::chrono::milliseconds(100));", read_state_body)
        frame_index = read_state_body.index(
            "Frame_Generate_With_Retry(TCP_Normal_Connection, 14, CABIN_STATE_RESPONSE_BYTES)"
        )
        mark_ready_index = read_state_body.index("g_cabin_driver->markExternalIoSuccess();", frame_index)
        self.assertLess(frame_index, mark_ready_index)
        self.assertNotIn("g_cabin_driver->pollState(", read_state_body)

    def test_cabin_tcp_legacy_frame_reader_consumes_exact_protocol_response_length(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr int CABIN_STATE_RESPONSE_BYTES = 144;", node)

        frame_start = node.index("\nint Frame_Generate(uint8_t* Control_Word, int Tlen")
        frame_end = node.index("\nint Frame_Generate_With_Retry", frame_start)
        frame_body = node[frame_start:frame_end]
        self.assertIn("const int expected_recv_len = Rlen;", frame_body)
        self.assertIn("while (total_recv < expected_recv_len)", frame_body)
        self.assertIn("recv(", frame_body)
        self.assertIn("buffer + total_recv", frame_body)
        self.assertIn("total_recv += recv_len;", frame_body)
        self.assertIn("memcpy(cabin_state_buffer, buffer, static_cast<size_t>(total_recv));", frame_body)
        self.assertNotIn("recv(socket, buffer, sizeof(buffer) - 1", frame_body)

        read_state_start = node.index("void read_cabin_state(")
        read_state_end = node.index("\nint RunSuoquNodeWithDefaultRole(", read_state_start)
        read_state_body = node[read_state_start:read_state_end]
        self.assertIn("Frame_Generate_With_Retry(TCP_Normal_Connection, 14, CABIN_STATE_RESPONSE_BYTES)", read_state_body)

    def test_moduan_services_do_not_set_busy_before_plc_execution(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertNotIn("class ScopedModuanWorkState", callbacks)
        self.assertNotIn("ScopedModuanWorkState work_state;", callbacks)
        self.assertNotIn("reject_if_cabin_motion_active", callbacks)

        for function_name in (
            "moduan_bind_service",
            "moduan_bind_points_service",
            "moduan_bind_points_fast_service",
            "moduan_move_service",
        ):
            start = callbacks.index(f"bool {function_name}(")
            next_function = callbacks.find("\nbool ", start + 5)
            if next_function == -1:
                next_function = len(callbacks)
            body = callbacks[start:next_function]
            self.assertIn("std::lock_guard<std::mutex> lashing_lock(lashing_mutex);", body)
            self.assertNotIn("pub_moduan_work_state(true);", body)
            self.assertNotIn("pub_moduan_work_state(false);", body)

        single_bind_start = callbacks.index("bool moduan_bind_service(")
        single_bind_end = callbacks.index("\nbool moduan_bind_points_service", single_bind_start)
        single_bind_body = callbacks[single_bind_start:single_bind_end]
        self.assertLess(single_bind_body.index("AI_client.call(srv)"), single_bind_body.index("execute_bind_points("))

    def test_moduan_work_topic_mirrors_plc_execution_until_finishall(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("wait_for_plc_finish_all", executor)
        self.assertIn("kFinishAllPollInterval", executor)
        self.assertNotIn("finish_all(150)", executor)
        self.assertIn("ScopedPlcExecutionState", executor)
        self.assertIn("pub_moduan_work_state(true);", executor)
        self.assertIn("pub_moduan_work_state(false);", executor)

        execute_start = executor.index("bool execute_bind_points(")
        execute_body = executor[execute_start:]
        write_points_index = execute_body.index("writeQueuedPoints")
        work_state_index = execute_body.index("ScopedPlcExecutionState plc_execution_state;", write_points_index)
        trigger_index = execute_body.index("pulseExecutionEnable")
        finishall_index = execute_body.index("wait_for_plc_finish_all(", work_state_index)
        self.assertLess(write_points_index, work_state_index)
        self.assertLess(work_state_index, trigger_index)
        self.assertLess(trigger_index, finishall_index)
        self.assertLess(work_state_index, finishall_index)

        finish_all_start = executor.index("bool wait_for_plc_finish_all(")
        finish_all_end = executor.index("\nvoid moveLinearModule", finish_all_start)
        finish_all_body = executor[finish_all_start:finish_all_end]
        self.assertIn("FINISH_ALL_FLAG", finish_all_body)
        self.assertIn("PLC_Order_Write(FINISHALL, 0, plc);", finish_all_body)

    def test_moduan_work_stays_busy_until_motion_is_confirmed_safe(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")

        class_start = executor.index("class ScopedPlcExecutionState")
        class_end = executor.index("struct LinearModuleAxisSnapshot", class_start)
        class_body = executor[class_start:class_end]
        self.assertIn("void mark_safe_to_clear()", class_body)
        destructor_start = class_body.index("~ScopedPlcExecutionState()")
        destructor_body = class_body[destructor_start:class_body.index("\n    }\n", destructor_start) + 7]
        self.assertNotIn("pub_moduan_work_state(false);", destructor_body)
        self.assertIn("保持/moduan_work=true", destructor_body)

        execute_start = executor.index("bool execute_bind_points(")
        execute_body = executor[execute_start:]
        finish_failure_index = execute_body.index("if (!wait_for_plc_finish_all(")
        finish_failure_end = execute_body.index(
            "        }\n        plc_execution_state.mark_safe_to_clear();",
            finish_failure_index,
        )
        finish_failure_body = execute_body[
            finish_failure_index:
            finish_failure_end
        ]
        self.assertNotIn("mark_safe_to_clear", finish_failure_body)
        release_index = execute_body.index("plc_execution_state.mark_safe_to_clear()", finish_failure_index)
        bind_data_index = execute_body.index("bind_all_data.push_back", finish_failure_index)
        self.assertLess(release_index, bind_data_index)

        direct_move_start = executor.index("bool move_linear_module_to_target(")
        direct_move_end = executor.index("\nvoid moveLinearModule", direct_move_start)
        direct_move_body = executor[direct_move_start:direct_move_end]
        scope_index = direct_move_body.index("ScopedPlcExecutionState linear_move_state;")
        first_motion_index = direct_move_body.index("Set_Module_Coordinate(WX_COORDINATE")
        direct_release_index = direct_move_body.index("linear_move_state.mark_safe_to_clear();")
        success_message_index = direct_move_body.index('response_message = "线性模组原子移动完成";')
        self.assertLess(scope_index, first_motion_index)
        self.assertLess(direct_release_index, success_message_index)

        origin_start = executor.index("bool move_linear_module_to_origin()")
        origin_end = executor.index("\ndouble max_bind_height_excess_mm", origin_start)
        origin_body = executor[origin_start:origin_end]
        origin_scope_index = origin_body.index("ScopedPlcExecutionState return_zero_state;")
        origin_first_motion_index = origin_body.index("Set_Module_Coordinate(WZ_COORDINATE")
        origin_release_index = origin_body.index("return_zero_state.mark_safe_to_clear();")
        origin_return_index = origin_body.index("return arrived_x && arrived_y;")
        self.assertLess(origin_scope_index, origin_first_motion_index)
        self.assertLess(origin_release_index, origin_return_index)

    def test_moduan_driver_publishes_standard_state_topic(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("#include \"tie_robot_msgs/ModuanState.h\"", callbacks)
        self.assertIn("publish_moduan_state_topic", callbacks)
        self.assertIn(
            'nh_.advertise<tie_robot_msgs::ModuanState>("/moduan/state"',
            callbacks,
        )
        self.assertIn("state_msg.executing", callbacks)
        self.assertIn("state_msg.finish_all", callbacks)
        self.assertIn("state->FINISH_ALL_FLAG", callbacks)
        self.assertIn("constexpr double kModuanStateMovingSpeedEpsilon = 10.0;", callbacks)
        state_start = callbacks.index("void publish_moduan_state_topic(")
        state_end = callbacks.index("\nvoid pub_moduan_work_state", state_start)
        state_body = callbacks[state_start:state_end]
        self.assertIn("const bool axis_motion", state_body)
        self.assertIn("state_msg.executing = axis_motion;", state_body)
        self.assertNotIn("moduan_plc_execution_state.load", state_body)
        self.assertIn("std::fabs(state->X_SPEED)", callbacks)
        self.assertIn("std::fabs(state->Y_SPEED)", callbacks)
        self.assertIn("std::fabs(state->Z_SPEED)", callbacks)
        self.assertIn("pub_moduan_state_topic.publish(state_msg);", callbacks)

    def test_moduan_execute_bind_points_action_is_advertised(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        cmake = (CONTROL_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        package_xml = (CONTROL_DIR / "package.xml").read_text(encoding="utf-8")

        self.assertIn("#include <actionlib/server/simple_action_server.h>", callbacks)
        self.assertIn("ExecuteBindPointsTaskAction", callbacks)
        self.assertIn("execute_bind_points_action_callback", callbacks)
        self.assertIn('"/moduan/execute_bind_points"', callbacks)
        self.assertIn("g_execute_bind_points_action_server->start();", callbacks)
        self.assertIn("setSucceeded", callbacks)
        self.assertIn("setAborted", callbacks)
        self.assertIn("actionlib", cmake)
        self.assertIn("<build_depend>actionlib</build_depend>", package_xml)

    def test_moduan_raw_execute_keeps_process_layer_jump_bind_selection(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        service_start = callbacks.index("bool moduan_driver_raw_execute_points_service(")
        service_end = callbacks.index("\nint RunModuanNodeWithDefaultRole", service_start)
        service_body = callbacks[service_start:service_end]

        self.assertIn("execute_bind_points(req.points, res.message)", service_body)
        self.assertNotIn("execute_bind_points(req.points, res.message,", service_body)

    def test_live_visual_refine_filters_bound_classification_points_before_ledger_matching(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        refine_start = node.index("if (!call_execution_refine_vision_service(")
        refine_end = node.index("area_world_points = dedupe_world_points(area_world_points);", refine_start)
        refine_block = node[refine_start:refine_end]

        self.assertIn("if (point.is_shuiguan)", refine_block)
        self.assertIn("跳过已绑扎视觉点", refine_block)

    def test_moduan_control_layer_has_no_legacy_idx_based_jump_bind_filter(self):
        action = (
            MSGS_DIR / "action" / "ExecuteBindPointsTask.action"
        ).read_text(encoding="utf-8")
        runtime_header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "runtime_state.hpp"
        ).read_text(encoding="utf-8")
        runtime_state = (
            CONTROL_DIR / "src" / "moduan" / "runtime_state.cpp"
        ).read_text(encoding="utf-8")
        callbacks_header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "moduan_ros_callbacks.hpp"
        ).read_text(encoding="utf-8")
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        executor_header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "linear_module_executor.hpp"
        ).read_text(encoding="utf-8")
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")
        process_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        area_execution = (
            PROCESS_DIR / "src" / "suoqu" / "area_execution.cpp"
        ).read_text(encoding="utf-8")
        suoqu_runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        topic_registry = (
            WEB_DIR / "frontend" / "src" / "config" / "topicRegistry.js"
        ).read_text(encoding="utf-8")
        legacy_catalog = (
            WEB_DIR / "frontend" / "src" / "config" / "legacyCommandCatalog.js"
        ).read_text(encoding="utf-8")

        for source in (
            action,
            runtime_header,
            runtime_state,
            callbacks_header,
            callbacks,
            executor_header,
            executor,
            process_node,
            area_execution,
            suoqu_runtime_header,
            topic_registry,
            legacy_catalog,
        ):
            self.assertNotIn("send_odd_points", source)
            self.assertNotIn("send_odd", source)
            self.assertNotIn("sendOddPoints", source)
            self.assertNotIn("should_keep_jump_bind_point", source)
            self.assertNotIn("apply_jump_bind_filter", source)
            self.assertNotIn("point.idx == 1", source)
            self.assertNotIn("point.idx == 4", source)
            self.assertNotIn("第1和第4个点", source)

        self.assertIn("jumpBindEnabled: \"/web/moduan/jump_bind_enabled\"", topic_registry)
        self.assertIn('nh.subscribe("/web/moduan/jump_bind_enabled"', process_node)

        service_start = callbacks.index("bool moduan_driver_raw_execute_points_service(")
        service_end = callbacks.index("\nint RunModuanNodeWithDefaultRole", service_start)
        service_body = callbacks[service_start:service_end]
        self.assertIn("execute_bind_points(req.points, res.message)", service_body)
        self.assertNotIn("execute_bind_points(req.points, res.message,", service_body)

        action_start = callbacks.index("void execute_bind_points_action_callback(")
        action_end = callbacks.index("\nvoid forced_stop_nodeCallback", action_start)
        action_body = callbacks[action_start:action_end]
        self.assertIn("execute_bind_points(points, message)", action_body)
        self.assertNotIn("goal->apply_jump_bind_filter", action_body)

    def test_process_layer_dispatches_precomputed_bind_points_through_action(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        cmake = (PROCESS_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        package_xml = (PROCESS_DIR / "package.xml").read_text(encoding="utf-8")

        self.assertIn("#include <actionlib/client/simple_action_client.h>", node)
        self.assertIn("ExecuteBindPointsTaskAction", node)
        self.assertIn("execute_moduan_bind_points_via_action", node)
        self.assertIn('"/moduan/execute_bind_points"', node)
        self.assertNotIn("sg_precomputed_client.call(", node)
        self.assertNotIn("sg_precomputed_fast_client.call(", node)
        self.assertIn("actionlib", cmake)
        self.assertIn("<build_depend>actionlib</build_depend>", package_xml)

    def test_execution_axis_wait_is_arrival_gated_instead_of_time_gated(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("kExecutionArrivalSoftTimeoutSec", node)
        self.assertIn("kExecutionArrivalSoftTimeoutLogIntervalSec", node)
        self.assertNotIn("kExecutionArrivalHardTimeoutSec", node)

        wait_start = node.index("bool wait_cabin_axis_stable_arrival(")
        wait_end = node.index("\n// 规划路径前下发速度", wait_start)
        wait_body = node[wait_start:wait_end]
        soft_timeout_index = wait_body.index("elapsed_sec >= kExecutionArrivalSoftTimeoutSec")

        self.assertIn("cur_motion_status != 0", wait_body[soft_timeout_index:])
        self.assertIn("axis_error_mm < normalized_tolerance_mm", wait_body[soft_timeout_index:])
        self.assertIn("pose_delta_mm > kExecutionArrivalPoseDeltaToleranceMm", wait_body[soft_timeout_index:])
        self.assertNotIn("elapsed_sec < kExecutionArrivalHardTimeoutSec", wait_body[soft_timeout_index:])
        self.assertIn("继续等待", wait_body[soft_timeout_index:])
        self.assertIn("索驱已停止但未到位", wait_body[soft_timeout_index:])

        keep_waiting_index = wait_body.index("continue_waiting_after_soft_timeout", soft_timeout_index)
        fail_detail_index = wait_body.index("set_last_execution_wait_error_detail", soft_timeout_index)
        self.assertLess(keep_waiting_index, fail_detail_index)

    def test_execution_axis_wait_tolerates_cabin_state_dropouts_and_reissues_target(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr double kCabinDriverStateFreshMaxAgeSec", node)
        self.assertIn("double get_cabin_driver_state_age_sec()", node)
        self.assertIn("bool is_cabin_driver_state_fresh(", node)

        wait_start = node.index("bool wait_cabin_axis_stable_arrival(")
        wait_end = node.index("\n// 规划路径前下发速度", wait_start)
        wait_body = node[wait_start:wait_end]

        self.assertIn("saw_stale_cabin_state", wait_body)
        self.assertIn("is_cabin_driver_state_fresh(&state_age_sec)", wait_body)
        self.assertIn("索驱状态断流", wait_body)
        self.assertIn("保持当前任务等待驱动恢复", wait_body)
        self.assertIn("active_wait_start_time = state_check_time;", wait_body)
        self.assertIn("stable_sample_count = 0;", wait_body)
        self.assertIn("reissue_cabin_target_after_state_recovery", wait_body)

        stale_index = wait_body.index("if (!is_cabin_driver_state_fresh(&state_age_sec))")
        soft_timeout_index = wait_body.index("elapsed_sec >= kExecutionArrivalSoftTimeoutSec")
        reissue_index = wait_body.index("reissue_cabin_target_after_state_recovery", stale_index)
        self.assertLess(stale_index, soft_timeout_index)
        self.assertLess(stale_index, reissue_index)

    def test_execution_axis_wait_treats_only_move_device_busy_status_as_transient(self):
        header = (
            PROCESS_DIR / "include" / "tie_robot_process" / "suoqu" / "cabin_transport.hpp"
        ).read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        transport = (
            PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("std::atomic<uint16_t> pending_tcp_status_command_word", header)
        self.assertIn("std::atomic<uint16_t> pending_tcp_status_command_word{0};", node)
        self.assertIn(
            "cache_pending_tcp_status_error(decoded_status.command_word, decoded_status.status_word)",
            node,
        )
        self.assertIn("bool is_transient_cabin_motion_status(", node)

        helper_start = node.index("bool is_transient_cabin_motion_status(")
        helper_end = node.index("\nbool is_retryable_cabin_driver_recovery_error", helper_start)
        helper_body = node[helper_start:helper_end]
        self.assertIn("status_word == (1u << 2)", helper_body)
        self.assertIn("command_word == 0x0012", helper_body)
        self.assertIn("command_word == 0x0011", helper_body)
        self.assertNotIn("Z超正限位", helper_body)
        self.assertNotIn("速度错误", helper_body)

        consume_start = transport.index("bool consume_pending_tcp_status_error(")
        consume_end = transport.index("\nbool is_motion_move_command_frame", consume_start)
        consume_body = transport[consume_start:consume_end]
        self.assertIn("pending_tcp_status_command_word.exchange(0", consume_body)

        wait_start = node.index("bool wait_cabin_axis_stable_arrival(")
        wait_end = node.index("\n// 规划路径前下发速度", wait_start)
        wait_body = node[wait_start:wait_end]
        pending_index = wait_body.index("consume_pending_tcp_status_error(")
        transient_index = wait_body.index("is_transient_cabin_motion_status", pending_index)
        fatal_index = wait_body.index("立即停止等待", pending_index)
        self.assertLess(transient_index, fatal_index)
        self.assertIn("索驱暂未接受运动指令", wait_body[transient_index:fatal_index])
        self.assertIn("continue;", wait_body[transient_index:fatal_index])

    def test_automatic_execution_retries_device_busy_without_retrying_limit_rejections(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        command_parser_start = node.index("bool has_cabin_motion_command_word_text(")
        command_parser_end = node.index("\nbool is_cabin_motion_busy_status_message", command_parser_start)
        command_parser_body = node[command_parser_start:command_parser_end]
        self.assertIn("request_command=0x0012", command_parser_body)
        self.assertIn("(0x0012)", command_parser_body)

        parser_start = node.index("bool is_cabin_motion_busy_status_message(")
        parser_end = node.index("\nbool is_retryable_cabin_driver_recovery_error", parser_start)
        parser_body = node[parser_start:parser_end]
        self.assertIn("status_word=0x00000004", parser_body)
        self.assertIn("状态字=0x00000004", parser_body)
        self.assertIn("设备运动中", parser_body)

        helper_start = node.index("bool is_retryable_cabin_driver_recovery_error(")
        helper_end = node.index("\nbool move_cabin_pose_for_automatic_execution", helper_start)
        helper_body = node[helper_start:helper_end]

        self.assertIn("is_cabin_motion_busy_status_message(error_message)", helper_body)
        self.assertNotIn('error_message.find("设备运动中")', helper_body)
        self.assertNotIn('lower_message.find("status_word=0x00000004")', helper_body)
        self.assertNotIn('error_message.find("Z超正限位")', helper_body)
        self.assertNotIn('error_message.find("速度错误")', helper_body)

    def test_automatic_execution_retries_cabin_moves_across_driver_reconnect(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool is_retryable_cabin_driver_recovery_error(", node)
        self.assertIn("bool move_cabin_pose_for_automatic_execution(", node)

        helper_start = node.index("bool move_cabin_pose_for_automatic_execution(")
        helper_end = node.index("\nbool fail_if_execution_return_to_start_requested", helper_start)
        helper_body = node[helper_start:helper_end]
        self.assertIn("move_cabin_pose_via_driver", helper_body)
        self.assertIn("is_retryable_cabin_driver_recovery_error(driver_error_message)", helper_body)
        self.assertIn("等待索驱驱动恢复后继续当前任务", helper_body)
        self.assertIn("is_execution_return_to_start_requested()", helper_body)
        self.assertIn("return false;", helper_body)

        for function_name, next_marker in (
            ("bool run_bind_path_direct_test(", "\nbool run_live_visual_global_work"),
            ("bool run_live_visual_global_work(", "\nbool run_planned_path_refine_only_global_work"),
            ("bool run_planned_path_refine_only_global_work(", "\nbool run_bind_from_scan"),
            ("bool run_bind_from_scan(", "\n// service 编排已抽到"),
        ):
            start = node.index(function_name)
            end = node.index(next_marker, start)
            body = node[start:end]
            self.assertIn("move_cabin_pose_for_automatic_execution", body, function_name)

    def test_single_cabin_move_retries_and_reissues_across_driver_reconnect(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("bool cabin_single_move(")
        end = node.index("\nbool cabin_driver_raw_move_service", start)
        body = node[start:end]

        self.assertIn("move_cabin_pose_for_automatic_execution", body)
        self.assertIn("wait_cabin_axis_stable_arrival(AXIS_X", body)
        self.assertIn("wait_cabin_axis_stable_arrival(AXIS_Y", body)
        self.assertIn("wait_cabin_axis_stable_arrival(AXIS_Z", body)
        self.assertNotIn("move_cabin_pose_via_driver(cabin_speed", body)
        self.assertNotIn("wait_cabin_axis_arrival(", body)

    def test_pseudo_slam_scan_moves_retry_and_reissue_across_driver_reconnect(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("bool run_pseudo_slam_scan(")
        end = node.index("\nstd::vector<tie_robot_msgs::PointCoords> load_bind_points_from_group_json", start)
        body = node[start:end]

        for case_marker, next_marker in (
            ("case PseudoSlamScanStrategy::kFixedManualWorkspace:", "case PseudoSlamScanStrategy::kSingleCenter:"),
            ("case PseudoSlamScanStrategy::kSingleCenter:", "case PseudoSlamScanStrategy::kMultiPose:"),
            ("case PseudoSlamScanStrategy::kMultiPose:", "if (!should_persist_pseudo_slam_bind_artifacts"),
        ):
            case_start = body.index(case_marker)
            case_end = body.index(next_marker, case_start)
            case_body = body[case_start:case_end]

            self.assertIn("move_cabin_pose_for_automatic_execution", case_body, case_marker)
            self.assertIn("wait_cabin_axis_stable_arrival(AXIS_X", case_body, case_marker)
            self.assertIn("wait_cabin_axis_stable_arrival(AXIS_Y", case_body, case_marker)
            self.assertIn("wait_cabin_axis_stable_arrival(AXIS_Z", case_body, case_marker)
            self.assertNotIn("move_cabin_pose_via_driver(", case_body, case_marker)
            self.assertNotIn("wait_cabin_axis_arrival(", case_body, case_marker)


if __name__ == "__main__":
    unittest.main()
