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

    def test_linear_module_executor_rejects_full_tcp_travel_range_not_only_z(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")

        execute_start = executor.index("bool execute_bind_points(")
        execute_body = executor[execute_start:]

        self.assertIn("is_valid_precomputed_tcp_travel_point", executor)
        self.assertIn("kTravelMaxXMm", executor)
        self.assertIn("kTravelMaxYMm", executor)
        self.assertIn("kTcpTravelMaxZMm", executor)
        self.assertIn("!is_valid_precomputed_tcp_travel_point(", execute_body)

    def test_frontend_single_point_bind_remains_backend_atomic_trigger(self):
        task_action_controller = (
            WEB_DIR / "frontend" / "src" / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")

        body_start = task_action_controller.index("async triggerSinglePointBind()")
        body_end = task_action_controller.index("\n  handleSavedWorkspacePayload", body_start)
        body = task_action_controller[body_start:body_end]

        self.assertIn("await this.rosConnection.callSinglePointBindService()", body)
        self.assertNotIn("callProcessImageService", body)
        self.assertNotIn("triggerSurfaceDpRecognition", body)


if __name__ == "__main__":
    unittest.main()
