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

    def test_cabin_motion_driver_rejects_moves_while_plc_execution_signal_is_active(self):
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        transport = (
            PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("extern std::atomic<bool> moduan_work_flag;", runtime_header)
        self.assertIn("std::atomic<bool> moduan_work_flag{false};", node)
        self.assertIn("moduan_work_flag.store(debug_mes.data", node)

        move_start = transport.index("bool move_cabin_pose_via_driver(")
        move_end = transport.index("\nbool move_cabin_incremental_via_driver", move_start)
        move_body = transport[move_start:move_end]

        self.assertIn("moduan_work_flag.load(", move_body)
        guard_index = move_body.index("moduan_work_flag.load(")
        remote_call_index = transport.index(
            '"/cabin/driver/raw_move"',
            move_start + guard_index,
        )
        helper_call_index = transport.index("call_remote_cabin_single_move_service", move_start + guard_index)
        driver_call_index = transport.index("::g_cabin_driver->moveToPose", move_start + guard_index)
        self.assertLess(move_start + guard_index, helper_call_index)
        self.assertLess(helper_call_index, remote_call_index)
        self.assertLess(move_start + guard_index, driver_call_index)
        self.assertIn("末端绑扎/线性模组正在运动", move_body[guard_index:])

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
        self.assertIn("前端长按暂停/恢复作业", callback_body)
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
        origin_move_index = body.index("if (!move_cabin_pose_via_driver(")
        self.assertLess(record_index, origin_move_index)
        record_body = body[record_index:origin_move_index]
        self.assertIn("path_origin_x", record_body)
        self.assertIn("path_origin_y", record_body)
        self.assertIn("move_path_origin_z", record_body)

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

        self.assertIn('{ id: 25, name: "恢复回起点"', legacy)
        self.assertIn("command.id === 25", controller)
        self.assertIn("return { data: 2 };", controller)
        self.assertIn("handleToggleLongPress", controller)
        self.assertIn("inactiveRequiresLongPress", controller)
        self.assertIn("inactiveLongPressCommandId", controller)
        self.assertIn("activeRequiresLongPress", controller)
        self.assertIn("const longPressCommandId = currentValue", controller)
        self.assertIn("definition.inactiveLongPressCommandId", controller)
        self.assertIn("当前工作停止后，等待线性模组Z轴先归零，再让索驱回到执行起点", controller)
        self.assertNotIn("短按保持人工暂停锁定", controller)
        self.assertNotIn("回原点", controller)
        short_press_body = controller[controller.index("handleToggle(toggleId, parameters)"):]
        short_press_body = short_press_body[:short_press_body.index("handleToggleLongPress")]
        self.assertIn("resolveToggleCommand", short_press_body)
        self.assertNotIn("activeRequiresLongPress", short_press_body)
        self.assertIn("onControlToggle(callback)", ui)
        self.assertIn("longPressCommandId", ui)
        self.assertIn("is-long-press-charging", ui)

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

    def test_cabin_state_poll_clears_transport_timeout_after_successful_heartbeat(self):
        transport_header = (
            HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_tcp_transport.hpp"
        ).read_text(encoding="utf-8")
        transport = (HW_DIR / "src" / "driver" / "cabin_tcp_transport.cpp").read_text(encoding="utf-8")
        driver_header = (
            HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_driver.hpp"
        ).read_text(encoding="utf-8")
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("void markExternalIoSuccess();", transport_header)
        self.assertIn("void CabinTcpTransport::markExternalIoSuccess()", transport)
        mark_start = transport.index("void CabinTcpTransport::markExternalIoSuccess()")
        mark_end = transport.index("\nConnectionState CabinTcpTransport::connectionState", mark_start)
        mark_body = transport[mark_start:mark_end]
        self.assertIn("ConnectionState::kReady", mark_body)
        self.assertIn("last_error_text_.clear();", mark_body)

        self.assertIn("void markExternalIoSuccess();", driver_header)
        self.assertIn("void CabinDriver::markExternalIoSuccess()", driver)
        self.assertIn("transport_->markExternalIoSuccess();", driver)

        read_state_start = node.index("void read_cabin_state(")
        read_state_end = node.index("\nint RunSuoquNodeWithDefaultRole(", read_state_start)
        read_state_body = node[read_state_start:read_state_end]
        frame_call_index = read_state_body.index("const int state_frame_result = Frame_Generate_With_Retry(")
        mark_index = read_state_body.index("g_cabin_driver->markExternalIoSuccess();", frame_call_index)
        self.assertIn("if (state_frame_result == 0 && g_cabin_driver)", read_state_body)
        self.assertLess(frame_call_index, mark_index)

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
        self.assertIn(
            "Frame_Generate_With_Retry(TCP_Normal_Connection, 14, CABIN_STATE_RESPONSE_BYTES)",
            read_state_body,
        )

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


if __name__ == "__main__":
    unittest.main()
