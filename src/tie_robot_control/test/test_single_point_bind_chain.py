#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
CONTROL_DIR = WORKSPACE_ROOT / "tie_robot_control"
WEB_DIR = WORKSPACE_ROOT / "tie_robot_web"
MSGS_DIR = WORKSPACE_ROOT / "tie_robot_msgs"


class SinglePointBindChainTest(unittest.TestCase):
    def test_single_point_bind_uses_execution_refine_hough_and_dispatches_all_points(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        runtime_header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "runtime_state.hpp"
        ).read_text(encoding="utf-8")
        process_image_srv = (MSGS_DIR / "srv" / "ProcessImage.srv").read_text(encoding="utf-8")

        body_start = callbacks.index("bool moduan_bind_service(")
        body_end = callbacks.index("\nbool moduan_bind_points_service", body_start)
        body = callbacks[body_start:body_end]

        self.assertIn("uint8 MODE_EXECUTION_REFINE=4", process_image_srv)
        self.assertIn("constexpr uint8_t kProcessImageModeExecutionRefine = 4;", runtime_header)
        self.assertIn("srv.request.request_mode = kProcessImageModeExecutionRefine;", body)
        self.assertNotIn("kProcessImageModeBindCheck", body)
        self.assertIn("std::vector<tie_robot_msgs::PointCoords> filteredPoints", body)
        self.assertIn("execute_bind_points(filteredPoints, res.message", body)
        self.assertNotIn("resize(1)", body)
        self.assertNotIn(".front()", body)

    def test_single_point_bind_transforms_camera_points_to_gripper_before_execution(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        body_start = callbacks.index("bool moduan_bind_service(")
        body_end = callbacks.index("\nbool moduan_bind_points_service", body_start)
        body = callbacks[body_start:body_end]

        self.assertIn("transform_scepter_camera_points_to_gripper_points", callbacks)
        self.assertIn('"Scepter_depth_frame"', callbacks)
        self.assertIn('"gripper_frame"', callbacks)
        self.assertIn("transform_scepter_camera_points_to_gripper_points(", body)
        self.assertLess(
            body.index("transform_scepter_camera_points_to_gripper_points("),
            body.index("execute_bind_points("),
        )
        self.assertNotIn(
            "std::vector<tie_robot_msgs::PointCoords> filteredPoints(sortedArray.begin(), sortedArray.end());",
            body,
        )

    def test_single_point_bind_sorts_gripper_points_as_snake_rows_before_execution(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        body_start = callbacks.index("bool moduan_bind_service(")
        body_end = callbacks.index("\nbool moduan_bind_points_service", body_start)
        body = callbacks[body_start:body_end]

        self.assertIn("void sort_gripper_points_by_snake_rows(", callbacks)
        self.assertIn("kSinglePointBindSnakeRowToleranceMm", callbacks)
        self.assertIn("point.World_coord[0]", callbacks)
        self.assertIn("point.World_coord[1]", callbacks)
        self.assertIn("row_index % 2U", callbacks)
        self.assertIn("sort_gripper_points_by_snake_rows(filteredPoints);", body)
        self.assertLess(
            body.index("transform_scepter_camera_points_to_gripper_points("),
            body.index("sort_gripper_points_by_snake_rows(filteredPoints);"),
        )
        self.assertLess(
            body.index("sort_gripper_points_by_snake_rows(filteredPoints);"),
            body.index("execute_bind_points("),
        )

    def test_linear_module_executor_trusts_visual_selected_points_without_hard_travel_reject(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")
        runtime_header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "runtime_state.hpp"
        ).read_text(encoding="utf-8")
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        execute_start = executor.index("bool execute_bind_points(")
        execute_body = executor[execute_start:]
        move_start = callbacks.index("bool moduan_move_service(")
        move_end = callbacks.index("\nvoid light_switch", move_start)
        move_body = callbacks[move_start:move_end]

        for stale_symbol in (
            "is_valid_precomputed_tcp_travel_point",
            "is_valid_precomputed_tcp_travel_z",
            "kTcpTravelMinZMm",
            "kTcpTravelMaxZMm",
            "kTravelMaxXMm",
            "kTravelMaxYMm",
        ):
            with self.subTest(stale_symbol=stale_symbol):
                self.assertNotIn(stale_symbol, executor)
                self.assertNotIn(stale_symbol, runtime_header)
                self.assertNotIn(stale_symbol, callbacks)

        self.assertNotIn("不是合法TCP行程", execute_body)
        self.assertNotIn("超出行程", execute_body)
        self.assertNotIn("rejected_invalid_tcp_travel_count", execute_body)
        self.assertNotIn("目标点超出范围", move_body)
        self.assertIn("selected_bind_points.push_back(point);", execute_body)

    def test_tcp_linear_remote_single_move_uses_driver_raw_atomic_move(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")
        executor_header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "linear_module_executor.hpp"
        ).read_text(encoding="utf-8")
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        callbacks_header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "moduan_ros_callbacks.hpp"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "bool move_linear_module_to_target(double x, double y, double z, double angle, std::string& response_message);",
            executor_header,
        )
        direct_move_start = executor.index("bool move_linear_module_to_target(")
        direct_move_end = executor.index("\nvoid moveLinearModule", direct_move_start)
        direct_move_body = executor[direct_move_start:direct_move_end]
        self.assertIn("Set_Module_Coordinate(WX_COORDINATE", direct_move_body)
        self.assertIn("Set_Module_Coordinate(WY_COORDINATE", direct_move_body)
        self.assertIn("Set_Module_Coordinate(WZ_COORDINATE", direct_move_body)
        self.assertIn("trigger_linear_module_motion_execution(\"X/Y轴\"", direct_move_body)
        self.assertIn("trigger_linear_module_motion_execution(\"Z轴\"", direct_move_body)
        self.assertIn("wait_linear_module_axis_arrival(AXIS_X", direct_move_body)
        self.assertIn("线性模组当前处于软件错误状态", direct_move_body)
        error_guard_index = direct_move_body.index("error_detected.load")
        first_move_index = direct_move_body.index("Set_Module_Coordinate(WX_COORDINATE")
        xy_trigger_index = direct_move_body.index('trigger_linear_module_motion_execution("X/Y轴"')
        xy_wait_index = direct_move_body.index("wait_linear_module_axis_arrival(AXIS_X")
        z_write_index = direct_move_body.index("Set_Module_Coordinate(WZ_COORDINATE")
        z_trigger_index = direct_move_body.index('trigger_linear_module_motion_execution("Z轴"')
        z_wait_index = direct_move_body.index("wait_linear_module_axis_arrival(AXIS_Z")
        self.assertLess(error_guard_index, first_move_index)
        self.assertLess(first_move_index, xy_trigger_index)
        self.assertLess(xy_trigger_index, xy_wait_index)
        self.assertLess(z_write_index, z_trigger_index)
        self.assertLess(z_trigger_index, z_wait_index)
        self.assertNotIn("waiting on current target pt", direct_move_body)
        self.assertNotIn("while (is_error)", direct_move_body)
        self.assertNotIn("execute_bind_points(", direct_move_body)
        self.assertNotIn("wait_for_plc_finish_all(", direct_move_body)
        self.assertNotIn("FINISHALL", direct_move_body)

        move_start = callbacks.index("bool moduan_move_service(")
        move_end = callbacks.index("\nvoid light_switch", move_start)
        move_body = callbacks[move_start:move_end]
        self.assertIn('"/moduan/driver/raw_single_move"', move_body)
        self.assertIn("ros::service::call", move_body)
        self.assertIn("move_linear_module_to_target(x, y, z, angle, res.message)", move_body)
        self.assertNotIn("tie_robot_msgs::PointCoords", move_body)
        self.assertNotIn("execute_bind_points(", move_body)

        self.assertIn("bool moduan_driver_raw_single_move_service(", callbacks_header)
        raw_move_start = callbacks.index("bool moduan_driver_raw_single_move_service(")
        raw_move_end = callbacks.index("\nbool moduan_driver_raw_execute_points_service", raw_move_start)
        raw_move_body = callbacks[raw_move_start:raw_move_end]
        self.assertIn("move_linear_module_to_target(x, y, z, angle, res.message)", raw_move_body)
        self.assertNotIn("execute_bind_points(", raw_move_body)
        self.assertIn('advertiseService("/moduan/driver/raw_single_move"', callbacks)

    def test_driver_raw_execute_points_serializes_with_ordered_return_zero(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        raw_execute_start = callbacks.index("bool moduan_driver_raw_execute_points_service(")
        raw_execute_end = callbacks.index("\nint RunModuanNodeWithDefaultRole", raw_execute_start)
        raw_execute_body = callbacks[raw_execute_start:raw_execute_end]

        self.assertIn("std::lock_guard<std::mutex> lashing_lock(lashing_mutex);", raw_execute_body)
        self.assertLess(
            raw_execute_body.index("std::lock_guard<std::mutex> lashing_lock(lashing_mutex);"),
            raw_execute_body.index("execute_bind_points("),
        )

        return_zero_start = callbacks.index("bool return_zero_ordered_service(")
        return_zero_end = callbacks.index("\nvoid moduan_move_zero_forthread", return_zero_start)
        return_zero_body = callbacks[return_zero_start:return_zero_end]
        self.assertIn("std::unique_lock<std::mutex> lashing_lock(lashing_mutex, std::defer_lock);", return_zero_body)
        self.assertIn("wait_for_lashing_mutex_for_ordered_return_zero(lashing_lock", return_zero_body)
        self.assertLess(
            return_zero_body.index("wait_for_lashing_mutex_for_ordered_return_zero(lashing_lock"),
            return_zero_body.index("move_linear_module_to_origin()"),
        )

    def test_ordered_return_zero_polls_for_current_execution_and_motion_release(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("kOrderedReturnZeroLockTimeout", callbacks)
        self.assertIn("kOrderedReturnZeroMotionReleaseTimeout", callbacks)
        self.assertIn("kOrderedReturnZeroLockPollInterval", callbacks)
        self.assertIn("bool wait_for_lashing_mutex_for_ordered_return_zero(", callbacks)
        wait_lock_start = callbacks.index("bool wait_for_lashing_mutex_for_ordered_return_zero(")
        wait_lock_end = callbacks.index("\n}\n\nbool wait_for_ordered_return_zero_motion_release", wait_lock_start) + 3
        wait_lock_body = callbacks[wait_lock_start:wait_lock_end]
        self.assertIn("lashing_lock.try_lock()", wait_lock_body)
        self.assertIn("std::this_thread::sleep_for(kOrderedReturnZeroLockPollInterval)", wait_lock_body)
        self.assertIn("kOrderedReturnZeroLockTimeout", wait_lock_body)
        self.assertIn("等待当前末端执行链释放", wait_lock_body)
        self.assertIn("超时", wait_lock_body)

        wait_motion_start = callbacks.index("bool wait_for_ordered_return_zero_motion_release(")
        wait_motion_end = callbacks.index("\n}\n\nvoid request_legacy_moduan_zero", wait_motion_start) + 3
        wait_motion_body = callbacks[wait_motion_start:wait_motion_end]
        self.assertIn("module_state.X_SPEED", wait_motion_body)
        self.assertIn("module_state.Y_SPEED", wait_motion_body)
        self.assertIn("module_state.Z_SPEED", wait_motion_body)
        self.assertIn("kModuanStateMovingSpeedEpsilon", wait_motion_body)
        self.assertIn("std::this_thread::sleep_for(kOrderedReturnZeroLockPollInterval)", wait_motion_body)
        self.assertIn("kOrderedReturnZeroMotionReleaseTimeout", wait_motion_body)

        service_start = callbacks.index("bool return_zero_ordered_service(")
        service_end = callbacks.index("\nvoid moduan_move_zero_forthread", service_start)
        service_body = callbacks[service_start:service_end]
        request_index = service_body.index("moduan_return_zero_ordered_requested.store(true")
        zero_index = service_body.index("PLC_Order_Write(IS_ZERO, 1")
        zero_release_index = service_body.index("PLC_Order_Write(IS_ZERO, 0", zero_index)
        stop_index = service_body.index("PLC_Order_Write(IS_STOP, 1")
        lock_wait_index = service_body.index("wait_for_lashing_mutex_for_ordered_return_zero")
        motion_wait_index = service_body.index("wait_for_ordered_return_zero_motion_release")
        clear_index = service_body.index("moduan_return_zero_ordered_requested.store(false")
        move_index = service_body.index("move_linear_module_to_origin()")
        self.assertLess(zero_index, stop_index)
        self.assertLess(stop_index, zero_release_index)
        self.assertLess(zero_release_index, lock_wait_index)
        self.assertLess(request_index, lock_wait_index)
        self.assertLess(lock_wait_index, motion_wait_index)
        self.assertLess(motion_wait_index, clear_index)
        self.assertLess(clear_index, move_index)

    def test_linear_module_axis_wait_aborts_when_ordered_return_zero_requested(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")

        wait_start = executor.index("bool wait_linear_module_axis_arrival(")
        wait_end = executor.index("\nbool arrive_z", wait_start)
        wait_body = executor[wait_start:wait_end]
        self.assertIn("moduan_return_zero_ordered_requested.load(std::memory_order_acquire)", wait_body)
        request_index = wait_body.index("moduan_return_zero_ordered_requested.load(std::memory_order_acquire)")
        timeout_index = wait_body.index("if (elapsed_sec >= kLinearModuleAxisArrivalTimeoutSec)")
        self.assertLess(request_index, timeout_index)
        self.assertIn("收到长按停止并回起点请求", wait_body[request_index:timeout_index])

    def test_pause_return_zero_service_moves_z_to_zero_before_xy(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "linear_module_executor.hpp"
        ).read_text(encoding="utf-8")

        self.assertIn("bool move_linear_module_to_origin();", header)
        move_start = executor.index("bool move_linear_module_to_origin()")
        move_end = executor.index("\ndouble max_bind_height_excess_mm", move_start)
        move_body = executor[move_start:move_end]
        z_write_index = move_body.index("Set_Module_Coordinate(WZ_COORDINATE")
        z_wait_index = move_body.index("wait_linear_module_axis_arrival(AXIS_Z")
        xy_write_index = move_body.index("Set_Module_Coordinate(WX_COORDINATE", z_wait_index)
        self.assertLess(z_write_index, z_wait_index)
        self.assertLess(z_wait_index, xy_write_index)

        self.assertIn("bool return_zero_ordered_service(", callbacks)
        service_start = callbacks.index("bool return_zero_ordered_service(")
        service_end = callbacks.index("\nvoid moduan_move_zero_forthread", service_start)
        service_body = callbacks[service_start:service_end]
        self.assertIn("move_linear_module_to_origin()", service_body)
        self.assertIn('"/moduan/return_zero_ordered"', callbacks)

    def test_direct_moduan_zero_uses_legacy_plc_zero_request_without_waiting(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")
        executor_header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "linear_module_executor.hpp"
        ).read_text(encoding="utf-8")

        callback_start = callbacks.index("void moduan_move_zero_callback(")
        callback_end = callbacks.index("\nbool moduan_move_service(", callback_start)
        callback_body = callbacks[callback_start:callback_end]
        self.assertIn("request_legacy_moduan_zero(", callback_body)
        helper_start = callbacks.index("void request_legacy_moduan_zero(")
        helper_end = callbacks.index("\n}\n\nvoid request_moduan_zero", helper_start) + 3
        helper_body = callbacks[helper_start:helper_end]
        enable_index = helper_body.index("PLC_Order_Write(EN_DISABLE, 1")
        zero_index = helper_body.index("PLC_Order_Write(IS_ZERO, 1")
        zero_release_index = helper_body.index("PLC_Order_Write(IS_ZERO, 0", zero_index)
        self.assertLess(enable_index, zero_index)
        self.assertLess(zero_index, zero_release_index)
        self.assertNotIn("request_linear_module_zero_via_driver", callback_body)
        self.assertNotIn("move_linear_module_to_origin()", callback_body)
        self.assertNotIn("wait_linear_module_axis_arrival", callback_body)
        self.assertNotIn("request_linear_module_zero_via_driver", executor)
        self.assertNotIn("request_linear_module_zero_via_driver", executor_header)

    def test_legacy_moduan_zero_request_is_shared_and_releases_pause_state(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        helper_start = callbacks.index("void request_legacy_moduan_zero(")
        helper_end = callbacks.index("\n}\n\nvoid request_moduan_zero", helper_start) + 3
        helper_body = callbacks[helper_start:helper_end]
        self.assertIn("moduan_return_zero_ordered_requested.store(false", helper_body)
        self.assertIn("handle_pause_interrupt = false;", helper_body)
        stop_index = helper_body.index("PLC_Order_Write(IS_STOP, 0")
        finish_index = helper_body.index("PLC_Order_Write(FINISHALL, 0")
        enable_index = helper_body.index("PLC_Order_Write(EN_DISABLE, 1")
        zero_index = helper_body.index("PLC_Order_Write(IS_ZERO, 1")
        zero_release_index = helper_body.index("PLC_Order_Write(IS_ZERO, 0", zero_index)
        self.assertLess(stop_index, enable_index)
        self.assertLess(finish_index, enable_index)
        self.assertLess(enable_index, zero_index)
        self.assertLess(zero_index, zero_release_index)
        self.assertNotIn("request_linear_module_zero_via_driver", helper_body)
        self.assertNotIn("move_linear_module_to_origin()", helper_body)
        self.assertNotIn("wait_linear_module_axis_arrival", helper_body)

        for signature in (
            "void request_moduan_zero(",
            "void moduan_move_zero_forthread(",
            "void moduan_move_zero_callback(",
        ):
            with self.subTest(signature=signature):
                body_start = callbacks.index(signature)
                if signature == "void request_moduan_zero(":
                    body_end = callbacks.index("\nbool return_zero_ordered_service", body_start)
                elif signature == "void moduan_move_zero_forthread(":
                    body_end = callbacks.index("\nvoid moduan_move_zero_callback", body_start)
                else:
                    body_end = callbacks.index("\nbool moduan_move_service", body_start)
                body = callbacks[body_start:body_end]
                self.assertIn("request_legacy_moduan_zero(", body)

    def test_linear_module_speed_setting_is_global_and_not_overridden_by_fast_service(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("void apply_module_speed_mm_per_sec(double new_speed)", executor)
        apply_start = executor.index("void apply_module_speed_mm_per_sec(double new_speed)")
        apply_end = executor.index("\nstd::string compose_linear_module_driver_error_message", apply_start)
        apply_body = executor[apply_start:apply_end]
        self.assertIn("module_speed = new_speed;", apply_body)
        self.assertIn("Set_Module_Speed(WX_SPEED, &module_speed, plc);", apply_body)
        self.assertIn("Set_Module_Speed(WY_SPEED, &module_speed, plc);", apply_body)
        self.assertIn("Set_Module_Speed(WZ_SPEED, &module_speed, plc);", apply_body)

        speed_callback_start = callbacks.index("void change_speed_callback(")
        speed_callback_end = callbacks.index("\nvoid handSolveWarnCallback", speed_callback_start)
        speed_callback_body = callbacks[speed_callback_start:speed_callback_end]
        self.assertIn("apply_module_speed_mm_per_sec(static_cast<double>(debug_mes.data));", speed_callback_body)
        self.assertIn('nh_.subscribe("/web/moduan/set_moduan_speed"', callbacks)

        init_start = callbacks.index("void initPLC()")
        init_end = callbacks.index("\nvoid auto_zero_on_startup", init_start)
        init_body = callbacks[init_start:init_end]
        self.assertIn("Set_Module_Speed(WX_SPEED, &module_speed, plc);", init_body)
        self.assertIn("Set_Module_Speed(WY_SPEED, &module_speed, plc);", init_body)
        self.assertIn("Set_Module_Speed(WZ_SPEED, &module_speed, plc);", init_body)

        fast_start = callbacks.index("bool moduan_bind_points_fast_service(")
        fast_end = callbacks.index("\nvoid forced_stop_nodeCallback", fast_start)
        fast_body = callbacks[fast_start:fast_end]
        self.assertNotIn("ScopedModuleSpeedOverride", fast_body)
        self.assertNotIn("kPrecomputedFastModuleSpeedMmPerSec", fast_body)

    def test_forced_stop_does_not_touch_is_zero_before_shutdown(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        stop_start = callbacks.index("void forced_stop_nodeCallback(")
        stop_end = callbacks.index("\nbool wait_for_lashing_mutex_for_ordered_return_zero", stop_start)
        stop_body = callbacks[stop_start:stop_end]
        self.assertNotIn("IS_ZERO", stop_body)
        self.assertIn("ros::shutdown()", stop_body)

    def test_signal_handler_does_not_touch_is_zero_before_shutdown(self):
        error_handling = (
            CONTROL_DIR / "src" / "moduan" / "error_handling.cpp"
        ).read_text(encoding="utf-8")

        handler_start = error_handling.index("void signalHandler(")
        handler_body = error_handling[handler_start:]
        self.assertNotIn("IS_ZERO", handler_body)
        self.assertIn("ros::shutdown()", handler_body)

    def test_short_pause_waits_without_is_zero_and_return_to_start_aborts_finishall_wait(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        runtime_header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "runtime_state.hpp"
        ).read_text(encoding="utf-8")

        self.assertIn("moduan_return_zero_ordered_requested", runtime_header)
        wait_start = executor.index("bool wait_for_plc_finish_all(")
        wait_end = executor.index("\nvoid moveLinearModule", wait_start)
        wait_body = executor[wait_start:wait_end]
        self.assertIn("handle_pause_interrupt", wait_body)
        self.assertIn("moduan_return_zero_ordered_requested", wait_body)
        return_index = wait_body.index("moduan_return_zero_ordered_requested")
        pause_index = wait_body.index("handle_pause_interrupt", return_index)
        finish_index = wait_body.index("if (finishall_flag) break;")
        self.assertLess(pause_index, finish_index)
        self.assertIn("人工暂停", wait_body)
        pause_wait_body = wait_body[pause_index:finish_index]
        self.assertIn("while (", pause_wait_body)
        self.assertIn("handle_pause_interrupt &&", pause_wait_body)
        self.assertIn("恢复当前末端执行等待", pause_wait_body)
        self.assertIn("return false;", wait_body[return_index:pause_index])

        pause_start = callbacks.index("void pause_interrupt_Callback(")
        pause_end = callbacks.index("\nvoid manual_area_takeover_callback", pause_start)
        pause_body = callbacks[pause_start:pause_end]
        self.assertNotIn("IS_ZERO", pause_body)
        self.assertIn("PLC_Order_Write(IS_STOP, 1", pause_body)

        hand_start = callbacks.index("void handSolveWarnCallback(")
        hand_end = callbacks.index("\nvoid read_module_motor_state", hand_start)
        hand_body = callbacks[hand_start:hand_end]
        self.assertIn("warn_msg.data == 2.0", hand_body)
        self.assertIn("moduan_return_zero_ordered_requested.store(true", hand_body)
        long_press_body = hand_body[hand_body.index("warn_msg.data == 2.0"):]
        zero_index = long_press_body.index("PLC_Order_Write(IS_ZERO, 1")
        zero_release_index = long_press_body.index("PLC_Order_Write(IS_ZERO, 0", zero_index)
        stop_index = long_press_body.index("PLC_Order_Write(IS_STOP, 1")
        self.assertLess(zero_index, stop_index)
        self.assertLess(stop_index, zero_release_index)

    def test_precomputed_bind_does_not_touch_is_zero_and_waits_for_finishall_clear_before_trigger(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("bool wait_for_plc_finish_all_clear(", executor)
        execute_start = executor.index("bool execute_bind_points(")
        execute_body = executor[execute_start:]
        clear_index = execute_body.index("clearFinishAll")
        clear_wait_index = execute_body.index("wait_for_plc_finish_all_clear(")
        write_points_index = execute_body.index("writeQueuedPoints")
        trigger_index = execute_body.index("pulseExecutionEnable")
        finish_wait_index = execute_body.index("wait_for_plc_finish_all(")
        self.assertNotIn("setZeroRequest", execute_body)
        self.assertNotIn("wait_for_plc_register_value", execute_body)
        self.assertNotIn("IS_ZERO", execute_body)
        self.assertLess(clear_index, clear_wait_index)
        self.assertLess(clear_wait_index, write_points_index)
        self.assertLess(write_points_index, trigger_index)
        self.assertLess(trigger_index, finish_wait_index)

    def test_moduan_driver_startup_keeps_default_lashing_disabled_in_plc(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        runtime_state = (
            CONTROL_DIR / "src" / "moduan" / "runtime_state.cpp"
        ).read_text(encoding="utf-8")

        init_start = callbacks.index("void initPLC()")
        init_end = callbacks.index("\nvoid auto_zero_on_startup", init_start)
        init_body = callbacks[init_start:init_end]

        self.assertIn("bool enable_lashing = false;", runtime_state)
        self.assertIn("enable_lashing = false;", init_body)
        enable_index = init_body.index("PLC_Order_Write(IS_LASHING, 0")
        speed_index = init_body.index("Set_Module_Speed(WX_SPEED")
        self.assertLess(enable_index, speed_index)

    def test_split_moduan_nodes_receive_return_to_start_in_motion_controller(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("void moduan_motion_controller_return_to_start_callback(", callbacks)
        motion_callback_start = callbacks.index("void moduan_motion_controller_return_to_start_callback(")
        motion_callback_end = callbacks.index("\nvoid read_module_motor_state", motion_callback_start)
        motion_callback_body = callbacks[motion_callback_start:motion_callback_end]
        self.assertIn("warn_msg.data == 2.0", motion_callback_body)
        self.assertIn("moduan_return_zero_ordered_requested.store(true", motion_callback_body)
        self.assertNotIn("PLC_Order_Write", motion_callback_body)

        hand_start = callbacks.index("void handSolveWarnCallback(")
        hand_end = callbacks.index("\nvoid moduan_motion_controller_return_to_start_callback", hand_start)
        hand_body = callbacks[hand_start:hand_end]
        long_press_index = hand_body.index("warn_msg.data == 2.0")
        zero_index = hand_body.index("PLC_Order_Write(IS_ZERO, 1", long_press_index)
        zero_release_index = hand_body.index("PLC_Order_Write(IS_ZERO, 0", zero_index)
        stop_index = hand_body.index("PLC_Order_Write(IS_STOP, 1", long_press_index)
        flag_index = hand_body.index("moduan_return_zero_ordered_requested.store(true", long_press_index)
        self.assertLess(flag_index, stop_index)
        self.assertLess(zero_index, stop_index)
        self.assertLess(stop_index, zero_release_index)

        run_start = callbacks.index("int RunModuanNodeWithDefaultRole(")
        run_body = callbacks[run_start:]
        motion_role_start = run_body.index("if (motion_controller_role)")
        driver_role_start = run_body.index("if (driver_role)", motion_role_start)
        motion_role_body = run_body[motion_role_start:driver_role_start]
        self.assertIn('subscribe("/web/moduan/hand_sovle_warn"', motion_role_body)
        self.assertIn("&moduan_motion_controller_return_to_start_callback", motion_role_body)

    def test_frontend_single_point_bind_remains_backend_atomic_trigger(self):
        task_action_controller = (
            WEB_DIR / "frontend" / "src" / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")

        body_start = task_action_controller.index("async triggerSinglePointBind()")
        body_end = task_action_controller.index("\n  async triggerExecutionRefineVisionOnly()", body_start)
        body = task_action_controller[body_start:body_end]

        self.assertIn("await this.rosConnection.callSinglePointBindService()", body)
        self.assertNotIn("callProcessImageService", body)
        self.assertNotIn("triggerSurfaceDpRecognition", body)


if __name__ == "__main__":
    unittest.main()
