#!/usr/bin/env python3

import os
import sys
import unittest
import inspect
from pathlib import Path


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts")
)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import pointAI  # noqa: E402


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
CHASSIS_CTRL_DIR = WORKSPACE_ROOT / "chassis_ctrl"
FAST_IMAGE_SOLVE_DIR = WORKSPACE_ROOT / "fast_image_solve"


def make_points_array(z_values, idx_values=None):
    msg = pointAI.PointsArray()
    msg.PointCoordinatesArray = []
    idx_values = idx_values or list(range(1, len(z_values) + 1))
    for idx, z_value in zip(idx_values, z_values):
        point = pointAI.PointCoords()
        point.idx = idx
        point.Pix_coord = [0, 0]
        point.World_coord = [0.0, 0.0, float(z_value)]
        point.Angle = 0.0
        point.is_shuiguan = False
        msg.PointCoordinatesArray.append(point)
    msg.count = len(msg.PointCoordinatesArray)
    return msg


class PointAIOrderTest(unittest.TestCase):
    def test_process_image_service_supports_request_modes_and_failure_details(self):
        service_path = FAST_IMAGE_SOLVE_DIR / "srv" / "ProcessImage.srv"
        service_text = service_path.read_text(encoding="utf-8")
        self.assertIn("uint8 MODE_DEFAULT=0", service_text)
        self.assertIn("uint8 MODE_ADAPTIVE_HEIGHT=1", service_text)
        self.assertIn("uint8 MODE_BIND_CHECK=2", service_text)
        self.assertIn("uint8 request_mode", service_text)
        self.assertIn("bool success", service_text)
        self.assertIn("string message", service_text)
        self.assertIn("int32 out_of_height_count", service_text)
        self.assertIn("int32[] out_of_height_point_indices", service_text)
        self.assertIn("float32[] out_of_height_z_values", service_text)

    def test_area_progress_message_exists_with_expected_fields(self):
        message_path = CHASSIS_CTRL_DIR / "msg" / "AreaProgress.msg"
        self.assertTrue(message_path.exists(), "AreaProgress.msg should exist")
        message_text = message_path.read_text(encoding="utf-8")
        self.assertIn("int32 current_area_index", message_text)
        self.assertIn("int32 total_area_count", message_text)
        self.assertIn("int32 just_finished_area_index", message_text)
        self.assertIn("bool ready_for_next_area", message_text)
        self.assertIn("bool all_done", message_text)

    def test_chassis_ctrl_cmakelists_registers_area_progress_message(self):
        cmake_path = CHASSIS_CTRL_DIR / "CMakeLists.txt"
        cmake_text = cmake_path.read_text(encoding="utf-8")
        self.assertIn("AreaProgress.msg", cmake_text)

    def test_suoqu_node_advertises_area_progress_topic(self):
        suoqu_path = CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp"
        suoqu_text = suoqu_path.read_text(encoding="utf-8")
        self.assertIn("/cabin/area_progress", suoqu_text)
        self.assertIn("pub_area_progress", suoqu_text)

    def test_suoqu_node_publishes_area_progress_updates(self):
        suoqu_path = CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp"
        suoqu_text = suoqu_path.read_text(encoding="utf-8")
        self.assertIn("publish_area_progress(", suoqu_text)

    def test_suoqu_skips_area_when_adaptive_height_has_no_usable_vision_points(self):
        suoqu_path = CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp"
        suoqu_text = suoqu_path.read_text(encoding="utf-8")
        self.assertIn("bool adaptive_height(", suoqu_text)
        self.assertIn("if (!adaptive_height(", suoqu_text)
        self.assertIn("自适应高度视觉无可用坐标，跳过当前区域", suoqu_text)

    def test_moduan_reports_no_bindable_points_as_skipped_not_completed(self):
        moduan_path = CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp"
        moduan_text = moduan_path.read_text(encoding="utf-8")
        self.assertIn("snakeSortedPoints.empty()", moduan_text)
        self.assertIn("res.success = false", moduan_text)
        self.assertIn("视觉无可用绑扎点，跳过当前区域", moduan_text)

    def test_moduan_reports_bind_height_excess_for_cabin_adjustment(self):
        moduan_path = CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp"
        moduan_text = moduan_path.read_text(encoding="utf-8")
        self.assertIn("BIND_HEIGHT_EXCESS_MM=", moduan_text)
        self.assertIn("max_bind_height_excess_mm", moduan_text)
        self.assertIn("srv.response.out_of_height_z_values", moduan_text)
        self.assertIn("kBindMaxHeightMm", moduan_text)

    def test_moduan_jump_bind_only_keeps_points_one_and_four(self):
        moduan_path = CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp"
        moduan_text = moduan_path.read_text(encoding="utf-8")
        self.assertIn("should_keep_jump_bind_point", moduan_text)
        self.assertIn("point_index == 0 || point_index == 3", moduan_text)
        self.assertIn("selected_bind_point_count", moduan_text)
        self.assertNotIn("send_odd_points == 1 && i % 2 == 0", moduan_text)
        self.assertIn("跳绑2/4已开启", moduan_text)

    def test_suoqu_retries_bind_after_height_excess_adjustment(self):
        suoqu_path = CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp"
        suoqu_text = suoqu_path.read_text(encoding="utf-8")
        self.assertIn("parse_bind_height_excess_mm", suoqu_text)
        self.assertIn("adjust_cabin_height_for_bind_excess", suoqu_text)
        self.assertIn("max_bind_height_adjust_retries", suoqu_text)
        self.assertIn("TCP_Move[3] = adjusted_height", suoqu_text)
        self.assertIn("adjusted_height = old_height - height_excess_mm", suoqu_text)
        self.assertIn("continue", suoqu_text)

    def test_moduan_node_auto_zeroes_on_start_by_default(self):
        moduan_path = CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp"
        moduan_text = moduan_path.read_text(encoding="utf-8")
        self.assertIn('private_nh.param("auto_zero_on_start"', moduan_text)
        self.assertIn("request_moduan_zero(\"节点启动自动回零\")", moduan_text)
        self.assertIn("auto_zero_on_startup(private_nh)", moduan_text)

    def test_moduan_zero_also_returns_rotation_motor_to_zero_angle(self):
        moduan_path = CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp"
        moduan_text = moduan_path.read_text(encoding="utf-8")
        zero_func_start = moduan_text.index("void request_moduan_zero")
        zero_func_end = moduan_text.index(
            "void moduan_move_zero_forthread", zero_func_start
        )
        zero_func_text = moduan_text[zero_func_start:zero_func_end]
        self.assertIn("旋转电机回零", zero_func_text)
        self.assertIn("Set_Motor_Speed(&motor_speed, plc)", zero_func_text)
        self.assertIn("Set_Motor_Angle(&reset_angle, plc)", zero_func_text)

    def test_moduan_zero_moves_to_origin_instead_of_marking_current_position_zero(self):
        moduan_path = CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp"
        moduan_text = moduan_path.read_text(encoding="utf-8")
        zero_func_start = moduan_text.index("void request_moduan_zero")
        zero_func_end = moduan_text.index(
            "void moduan_move_zero_forthread", zero_func_start
        )
        zero_func_text = moduan_text[zero_func_start:zero_func_end]
        self.assertIn("void move_linear_module_to_origin()\n{", moduan_text)
        origin_func_start = moduan_text.index("void move_linear_module_to_origin()\n{")
        origin_func_end = moduan_text.index("void request_moduan_zero", origin_func_start)
        origin_func_text = moduan_text[origin_func_start:origin_func_end]

        self.assertNotIn("PLC_Order_Write(IS_ZERO", zero_func_text)
        self.assertIn("move_linear_module_to_origin()", zero_func_text)
        self.assertIn("Set_Module_Coordinate(WZ_COORDINATE,&zero_target,plc)", origin_func_text)
        self.assertIn("Set_Module_Coordinate(WX_COORDINATE,&zero_target,plc)", origin_func_text)
        self.assertIn("Set_Module_Coordinate(WY_COORDINATE,&zero_target,plc)", origin_func_text)
        self.assertLess(
            origin_func_text.index("Set_Module_Coordinate(WZ_COORDINATE"),
            origin_func_text.index("Set_Module_Coordinate(WX_COORDINATE"),
        )

    def test_suoqu_and_moduan_launch_exposes_auto_zero_switch(self):
        launch_path = CHASSIS_CTRL_DIR / "launch" / "suoquAndmoduan.launch"
        launch_text = launch_path.read_text(encoding="utf-8")
        self.assertIn('arg name="auto_zero_on_start" default="true"', launch_text)
        self.assertIn('param name="auto_zero_on_start" value="$(arg auto_zero_on_start)"', launch_text)

    def test_sort_matrix_points_numbers_from_calibrated_origin(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        matrix_points = [
            (1, [10, 10, [0, 0, 100]], [71.0, 291.0, 83.0]),
            (2, [110, 10, [0, 0, 100]], [68.0, 148.0, 82.0]),
            (3, [10, 90, [0, 0, 100]], [187.0, 292.0, 78.0]),
            (4, [110, 90, [0, 0, 100]], [190.0, 148.0, 80.0]),
        ]

        sorted_points = processor.sort_matrix_points(matrix_points)

        self.assertEqual([item[0] for item in sorted_points], [2, 1, 4, 3])

    def test_selected_point_numbers_start_at_one_inside_selected_range(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        selected_points = [
            (8, [10, 10, [0, 0, 100]], [10.0, 10.0, 0.0]),
            (4, [110, 10, [0, 0, 100]], [20.0, 10.0, 0.0]),
            (9, [10, 90, [0, 0, 100]], [10.0, 20.0, 0.0]),
            (7, [110, 90, [0, 0, 100]], [20.0, 20.0, 0.0]),
        ]

        self.assertEqual(
            processor.get_selected_point_numbers(selected_points),
            {8: 1, 4: 2, 9: 3, 7: 4}
        )

    def test_filter_close_points_keeps_origin_nearer_point(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        points = [
            (1, [10, 10, [0, 0, 100]], [62.0, -152.0, 0.0]),
            (2, [20, 20, [0, 0, 100]], [63.0, -197.0, 0.0]),
            (3, [120, 120, [0, 0, 100]], [180.0, 180.0, 0.0]),
        ]

        filtered_points = processor.filter_close_points_by_origin(points, min_distance_mm=100.0)

        self.assertEqual([item[0] for item in filtered_points], [1, 3])

    def test_unselected_labels_are_hidden_by_default(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.show_candidate_labels = False

        self.assertTrue(processor.should_draw_display_label("selected"))
        self.assertFalse(processor.should_draw_display_label("unselected"))

    def test_unselected_labels_can_be_enabled_for_debug(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.show_candidate_labels = True

        self.assertTrue(processor.should_draw_display_label("unselected"))

    def test_default_stable_frame_count_is_three(self):
        init_source = inspect.getsource(pointAI.ImageProcessor.__init__)
        self.assertIn('rospy.get_param("~stable_frame_count", 3)', init_source)

    def test_adaptive_mode_accepts_stable_points_above_bind_height_limit(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        point_coords = make_points_array([101.0, 97.0], idx_values=[1, 2])

        result = processor.evaluate_point_coords_for_mode(
            point_coords,
            request_mode=1
        )

        self.assertTrue(result["success"])
        self.assertEqual(result["out_of_height_count"], 0)
        self.assertEqual(result["out_of_height_point_indices"], [])
        self.assertEqual(result["out_of_height_z_values"], [])

    def test_bind_mode_rejects_points_above_ninety_four_mm_with_details(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        point_coords = make_points_array([90.0, 95.5, 101.0], idx_values=[1, 2, 4])

        result = processor.evaluate_point_coords_for_mode(
            point_coords,
            request_mode=2
        )

        self.assertFalse(result["success"])
        self.assertEqual(result["out_of_height_count"], 2)
        self.assertEqual(result["out_of_height_point_indices"], [2, 4])
        self.assertEqual(result["out_of_height_z_values"], [95.5, 101.0])
        self.assertIn("94mm", result["message"])

    def test_process_image_timing_message_includes_mode_and_elapsed_ms(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)

        class DummyResponse:
            success = True
            count = 4
            message = "stable"

        message = processor.build_process_image_timing_message(
            request_mode=2,
            response=DummyResponse(),
            elapsed_sec=0.1234,
        )

        self.assertIn("bind_check", message)
        self.assertIn("success=True", message)
        self.assertIn("count=4", message)
        self.assertIn("elapsed_ms=123.4", message)
        self.assertIn("message=stable", message)

    def test_z_window_accepts_three_frames_inside_five_mm_tolerance(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.stable_frame_count = 3
        processor.stable_z_tolerance_mm = 5.0
        snapshots = [
            processor.build_z_snapshot(make_points_array([100.0])),
            processor.build_z_snapshot(make_points_array([104.0])),
            processor.build_z_snapshot(make_points_array([96.0])),
        ]

        self.assertTrue(processor.is_stable_z_window(snapshots))

    def test_z_window_rejects_three_frames_outside_five_mm_tolerance(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.stable_frame_count = 3
        processor.stable_z_tolerance_mm = 5.0
        snapshots = [
            processor.build_z_snapshot(make_points_array([100.0])),
            processor.build_z_snapshot(make_points_array([104.0])),
            processor.build_z_snapshot(make_points_array([111.0])),
        ]

        self.assertFalse(processor.is_stable_z_window(snapshots))


if __name__ == "__main__":
    unittest.main()
