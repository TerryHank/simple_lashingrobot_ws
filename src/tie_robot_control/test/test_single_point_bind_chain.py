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
        self.assertIn("wait_linear_module_axis_arrival(AXIS_X", direct_move_body)
        self.assertIn("线性模组当前处于软件错误状态", direct_move_body)
        error_guard_index = direct_move_body.index("error_detected.load")
        first_move_index = direct_move_body.index("Set_Module_Coordinate(WX_COORDINATE")
        self.assertLess(error_guard_index, first_move_index)
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

    def test_short_pause_waits_and_return_to_start_aborts_finishall_wait(self):
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
        self.assertIn("while (handle_pause_interrupt", wait_body[pause_index:finish_index])
        self.assertIn("恢复当前末端执行等待", wait_body[pause_index:finish_index])
        self.assertIn("return false;", wait_body[return_index:pause_index])

        hand_start = callbacks.index("void handSolveWarnCallback(")
        hand_end = callbacks.index("\nvoid read_module_motor_state", hand_start)
        hand_body = callbacks[hand_start:hand_end]
        self.assertIn("warn_msg.data == 2.0", hand_body)
        self.assertIn("moduan_return_zero_ordered_requested.store(true", hand_body)

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
        stop_index = hand_body.index("PLC_Order_Write(IS_STOP, 1", long_press_index)
        flag_index = hand_body.index("moduan_return_zero_ordered_requested.store(true", long_press_index)
        self.assertLess(flag_index, stop_index)

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
