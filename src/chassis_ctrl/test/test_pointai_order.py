#!/usr/bin/env python3

import os
import sys
import unittest
import inspect
import tempfile
from pathlib import Path

import yaml


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


def make_points_array_with_world_coords(world_coords, idx_values=None):
    msg = pointAI.PointsArray()
    msg.PointCoordinatesArray = []
    idx_values = idx_values or list(range(1, len(world_coords) + 1))
    for idx, (x_value, y_value, z_value) in zip(idx_values, world_coords):
        point = pointAI.PointCoords()
        point.idx = idx
        point.Pix_coord = [0, 0]
        point.World_coord = [float(x_value), float(y_value), float(z_value)]
        point.Angle = 0.0
        point.is_shuiguan = False
        msg.PointCoordinatesArray.append(point)
    msg.count = len(msg.PointCoordinatesArray)
    return msg


def make_center_record(source_idx, calibrated_x, calibrated_y, calibrated_z=80.0):
    return (
        source_idx,
        [int(calibrated_x), int(calibrated_y), [0, 0, int(calibrated_z)]],
        [float(calibrated_x), float(calibrated_y), float(calibrated_z)],
    )


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
        self.assertIn("filteredPoints.empty()", moduan_text)
        self.assertIn("res.success = false", moduan_text)
        self.assertIn("视觉无可用绑扎点，跳过当前区域", moduan_text)

    def test_moduan_reports_bind_height_excess_for_cabin_adjustment(self):
        moduan_path = CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp"
        moduan_text = moduan_path.read_text(encoding="utf-8")
        self.assertIn("srv.response.out_of_height_z_values", moduan_text)
        self.assertIn("Moduan_Warn: 超高点 idx=%d, 实际z=%.2fmm", moduan_text)
        self.assertNotIn("res.message = append_bind_height_excess_message", moduan_text)
        self.assertNotIn("(double)point.World_coord[2] <= kBindMaxHeightMm", moduan_text)

    def test_moduan_jump_bind_only_keeps_points_one_and_four(self):
        moduan_path = CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp"
        moduan_text = moduan_path.read_text(encoding="utf-8")
        self.assertIn("should_keep_jump_bind_point", moduan_text)
        self.assertIn("point.idx == 1 || point.idx == 4", moduan_text)
        self.assertIn("selected_bind_point_count", moduan_text)
        self.assertNotIn("send_odd_points == 1 && i % 2 == 0", moduan_text)
        self.assertIn("跳绑2/4已开启", moduan_text)

    def test_moduan_nodes_use_visual_order_without_snake_sort(self):
        moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")
        moduan_show_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode_show.cpp").read_text(encoding="utf-8")

        self.assertNotIn("snake_sort(", moduan_text)
        self.assertNotIn("snake_sort(", moduan_show_text)
        self.assertIn("for (size_t i = 0; i < filteredPoints.size(); i++)", moduan_show_text)
        self.assertIn("for (int i = 0; i < filteredPoints.size(); i++)", moduan_text)

    def test_visual_scripts_only_keep_matrix_sort_logic(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

        self.assertNotIn("def snake_sort(", pointai_text)
        self.assertNotIn("def snake_sort(", vision_text)
        self.assertIn("def sort_matrix_points(", pointai_text)
        self.assertIn("def sort_matrix_points(", vision_text)

    def test_suoqu_retries_bind_after_height_excess_adjustment(self):
        suoqu_path = CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp"
        suoqu_text = suoqu_path.read_text(encoding="utf-8")
        self.assertIn("parse_bind_height_excess_mm", suoqu_text)
        self.assertIn("adjust_cabin_height_for_bind_excess", suoqu_text)
        self.assertIn("max_bind_height_adjust_retries", suoqu_text)
        self.assertIn("TCP_Move[3] = adjusted_height", suoqu_text)
        self.assertIn("adjusted_height = old_height - height_excess_mm", suoqu_text)
        self.assertIn("continue", suoqu_text)

    def test_suoqu_frame_generate_logs_non_heartbeat_frames_and_detects_peer_close(self):
        suoqu_path = CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp"
        suoqu_text = suoqu_path.read_text(encoding="utf-8")
        self.assertIn("void printFrameBytes(", suoqu_text)
        self.assertIn("recv_len == 0", suoqu_text)
        self.assertIn("TCP connection closed by peer", suoqu_text)
        self.assertIn("SO_SNDTIMEO", suoqu_text)
        self.assertIn("SO_RCVTIMEO", suoqu_text)

    def test_suoqu_delay_time_has_progress_and_timeout_logging(self):
        suoqu_path = CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp"
        suoqu_text = suoqu_path.read_text(encoding="utf-8")
        self.assertIn("DELAY_TIME_TIMEOUT_SEC", suoqu_text)
        self.assertIn("DELAY_TIME_LOG_INTERVAL_SEC", suoqu_text)
        self.assertIn("等待轴%d到位中", suoqu_text)
        self.assertIn("等待轴%d到位超时", suoqu_text)

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
        self.assertTrue(processor.should_draw_display_label("jump_skipped"))
        self.assertFalse(processor.should_draw_display_label("unselected"))

    def test_unselected_labels_can_be_enabled_for_debug(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.show_candidate_labels = True

        self.assertTrue(processor.should_draw_display_label("unselected"))

    def test_non_downstream_candidate_labels_are_visible_by_default(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.show_candidate_labels = False

        self.assertTrue(processor.should_draw_display_label("in_range"))
        self.assertTrue(processor.should_draw_display_label("out_of_range"))
        self.assertTrue(processor.should_draw_display_label("zero_world"))

    def test_travel_range_reject_reasons_report_axis_and_limit(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.travel_range_max_x_mm = 320.0
        processor.travel_range_max_y_mm = 320.0

        reasons = processor.get_travel_range_reject_reasons(321.0, -1.0)

        self.assertEqual(reasons, ["X超过320", "Y小于0"])

    def test_pointai_logs_chinese_range_filter_details(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")

        self.assertIn("可执行范围过滤", pointai_text)
        self.assertIn("范围外原因统计", pointai_text)
        self.assertIn("无法组成2x2矩阵", pointai_text)

    def test_build_detection_summary_log_uses_multiline_block(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.travel_range_max_x_mm = 320.0
        processor.travel_range_max_y_mm = 360.0

        message = processor.build_detection_summary_log(
            request_mode=pointAI.PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT,
            raw_candidate_count=24,
            duplicate_removed_count=0,
            in_range_candidate_count=0,
            out_of_range_point_count=24,
            selected_count=0,
            output_count=0,
            out_of_range_reason_counts={"X小于0": 24, "Y小于0": 24},
            out_of_range_samples=[
                "idx=8,pix=(23,434),coord=(-301.0,-67.0,255.0),原因=X小于0+Y小于0",
                "idx=9,pix=(100,437),coord=(-296.0,-205.0,254.0),原因=X小于0+Y小于0",
            ],
        )

        self.assertIn("pointAI调试:\n", message)
        self.assertIn("  模式: adaptive_height", message)
        self.assertIn("  可执行范围过滤: 原始候选=24, 去重移除=0, 范围内=0, 范围外=24, 2x2选中=0, 本次输出=0", message)
        self.assertIn("  范围限制: 0<=x<=320, 0<=y<=360", message)
        self.assertIn("  范围外原因统计: X小于0=24, Y小于0=24", message)
        self.assertIn("  样例:\n    - idx=8,pix=(23,434),coord=(-301.0,-67.0,255.0),原因=X小于0+Y小于0", message)
        self.assertIn("  结论: 自适应高度模式当前没有可用于高度平均的范围内点", message)
        self.assertTrue(message.endswith("\n"))

    def test_jump_bind_visual_selection_keeps_only_points_one_and_four_executing(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.jump_bind_enabled = True

        self.assertTrue(processor.should_execute_selected_point_number(1))
        self.assertFalse(processor.should_execute_selected_point_number(2))
        self.assertFalse(processor.should_execute_selected_point_number(3))
        self.assertTrue(processor.should_execute_selected_point_number(4))

    def test_without_jump_bind_all_selected_points_remain_executable(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.jump_bind_enabled = False

        self.assertTrue(processor.should_execute_selected_point_number(2))
        self.assertTrue(processor.should_execute_selected_point_number(3))

    def test_visual_scripts_subscribe_to_jump_bind_topic_and_mark_skipped_points(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

        self.assertIn("/web/moduan/send_odd_points", pointai_text)
        self.assertIn("/web/moduan/send_odd_points", vision_text)
        self.assertIn('"jump_skipped"', pointai_text)
        self.assertIn('"jump_skipped"', vision_text)

    def test_axis_aligned_line_filter_keeps_only_lines_within_fifteen_degrees(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.axis_alignment_tolerance_deg = 15.0

        self.assertTrue(processor.is_near_axis_aligned_line(0, 0, 100, 10))
        self.assertTrue(processor.is_near_axis_aligned_line(0, 0, 10, 100))
        self.assertFalse(processor.is_near_axis_aligned_line(0, 0, 100, 100))

    def test_visual_scripts_use_single_axis_alignment_helper(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

        self.assertIn("def axis_angle_diff_deg", pointai_text)
        self.assertIn("def axis_angle_diff_deg", vision_text)
        self.assertIn("def is_near_axis_aligned_line", pointai_text)
        self.assertIn("def is_near_axis_aligned_line", vision_text)
        self.assertNotIn("angle_diff <= 60", pointai_text)
        self.assertNotIn("angle_diff <= 60", vision_text)

    def test_visual_travel_range_caps_y_at_three_hundred_twenty(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)

        self.assertTrue(processor.is_point_in_travel_range(320, 320))
        self.assertFalse(processor.is_point_in_travel_range(320, 321))

    def test_visual_scripts_use_three_hundred_twenty_as_y_limit(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

        self.assertIn('travel_range_max_y_mm", 320.0', pointai_text)
        self.assertIn('travel_range_max_y_mm", 320.0', vision_text)
        self.assertNotIn("calibrated_y <= 360", pointai_text)
        self.assertNotIn("calibrated_y <= 360", vision_text)

    def test_moduan_uses_three_hundred_twenty_as_y_limit(self):
        moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")

        self.assertIn("kTravelMaxYMm = 320.0", moduan_text)
        self.assertIn("y < 0 || y > kTravelMaxYMm", moduan_text)
        self.assertNotIn("point.World_coord[1] < 360", moduan_text)
        self.assertNotIn("y < 0 || y > 360", moduan_text)

    def test_adaptive_height_outputs_all_in_range_points_without_matrix_count_limit(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        in_range_centers = [
            make_center_record(1, 10.0, 10.0),
            make_center_record(2, 20.0, 20.0),
            make_center_record(3, 30.0, 30.0),
        ]
        selected_centers = [in_range_centers[0], in_range_centers[2]]

        output_centers = processor.select_output_centers_for_mode(
            pointAI.PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT,
            in_range_centers,
            selected_centers=selected_centers,
        )

        self.assertEqual(output_centers, in_range_centers)

    def test_bind_check_still_outputs_only_selected_matrix_points(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        in_range_centers = [
            make_center_record(1, 10.0, 10.0),
            make_center_record(2, 20.0, 20.0),
            make_center_record(3, 30.0, 30.0),
        ]
        selected_centers = [in_range_centers[0], in_range_centers[2]]

        output_centers = processor.select_output_centers_for_mode(
            pointAI.PROCESS_IMAGE_MODE_BIND_CHECK,
            in_range_centers,
            selected_centers=selected_centers,
        )

        self.assertEqual(output_centers, selected_centers)

    def test_display_points_match_downstream_selected_matrix_points(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.travel_range_max_x_mm = 320.0
        processor.travel_range_max_y_mm = 320.0
        processor.jump_bind_enabled = False
        selected_centers = [
            make_center_record(5, 10.0, 10.0),
            make_center_record(6, 120.0, 10.0),
            make_center_record(7, 10.0, 120.0),
            make_center_record(8, 120.0, 120.0),
        ]

        display_points = processor.build_matrix_display_points(selected_centers)

        self.assertEqual(len(display_points), 4)
        self.assertEqual([item[0] for item in display_points], [1, 2, 3, 4])
        self.assertEqual([item[2] for item in display_points], [item[2] for item in selected_centers])
        self.assertEqual([item[3] for item in display_points], ["selected"] * 4)

    def test_display_matrix_prefers_downstream_points_and_falls_back_to_bind_display_range(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.display_bind_range_max_x_mm = 360.0
        processor.display_bind_range_max_y_mm = 360.0
        candidate_centers = [
            make_center_record(1, 10.0, 10.0),
            make_center_record(2, 332.0, 12.0),
            make_center_record(3, 10.0, 100.0),
            make_center_record(4, 329.0, 103.0),
            make_center_record(5, 334.0, -90.0),
        ]
        selected_centers = [
            make_center_record(5, 40.0, 40.0),
            make_center_record(6, 140.0, 40.0),
            make_center_record(7, 40.0, 140.0),
            make_center_record(8, 140.0, 140.0),
        ]

        self.assertEqual(
            processor.select_display_matrix_centers(selected_centers, [], candidate_centers),
            selected_centers
        )
        fallback_centers = processor.select_display_matrix_centers([], [], candidate_centers)

        self.assertEqual(len(fallback_centers), 4)
        self.assertNotIn(5, [item[0] for item in fallback_centers])

    def test_display_fallback_uses_bind_display_range_without_out_labels(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.travel_range_max_x_mm = 320.0
        processor.travel_range_max_y_mm = 320.0
        processor.display_bind_range_max_x_mm = 360.0
        processor.display_bind_range_max_y_mm = 360.0
        processor.jump_bind_enabled = False
        candidate_centers = [
            make_center_record(1, 10.0, 10.0),
            make_center_record(2, 332.0, 12.0),
            make_center_record(3, 10.0, 100.0),
            make_center_record(4, 329.0, 103.0),
            make_center_record(9, 240.0, 240.0),
        ]

        fallback_centers = processor.select_display_matrix_centers([], [], candidate_centers)
        display_points = processor.build_matrix_display_points(
            fallback_centers,
            mark_travel_out_of_range=False
        )

        self.assertEqual(len(display_points), 4)
        self.assertNotIn([240.0, 240.0, 80.0], [item[2] for item in display_points])
        self.assertEqual([item[3] for item in display_points], ["selected"] * 4)

    def test_display_fallback_still_shows_four_in_range_points_when_matrix_match_fails(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        in_range_centers = [
            make_center_record(1, 10.0, 10.0),
            make_center_record(2, 10.0, 100.0),
            make_center_record(3, 110.0, 200.0),
            make_center_record(4, 110.0, 300.0),
        ]

        self.assertEqual(processor.select_nearest_origin_matrix_points(in_range_centers), [])

        fallback_centers = processor.select_display_matrix_centers([], in_range_centers, in_range_centers)

        self.assertEqual(len(fallback_centers), 4)
        self.assertEqual({item[0] for item in fallback_centers}, {1, 2, 3, 4})

    def test_visual_display_prefers_downstream_centers_with_bind_display_fallback(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

        self.assertIn("display_centers = self.select_display_matrix_centers(self.sorted_centers, in_range_centers, candidate_centers)", pointai_text)
        self.assertIn("display_centers = self.select_display_matrix_centers(self.sorted_centers, in_range_centers, candidate_centers)", vision_text)
        self.assertIn("self.result_display_points = self.build_matrix_display_points(display_centers)", pointai_text)
        self.assertIn("self.result_display_points = self.build_matrix_display_points(display_centers)", vision_text)
        self.assertNotIn("select_display_matrix_centers(self.sorted_centers, candidate_centers)", pointai_text)
        self.assertNotIn("select_display_matrix_centers(self.sorted_centers, candidate_centers)", vision_text)
        self.assertNotIn("self.result_display_points = self.build_matrix_display_points(candidate_centers)", pointai_text)
        self.assertNotIn("self.result_display_points = self.build_matrix_display_points(candidate_centers)", vision_text)

    def test_visual_services_pass_request_mode_to_pre_img(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

        self.assertIn("def pre_img(self, request_mode=PROCESS_IMAGE_MODE_DEFAULT)", pointai_text)
        self.assertIn("def pre_img(self, request_mode=PROCESS_IMAGE_MODE_DEFAULT)", vision_text)
        self.assertIn("self.pre_img(request_mode=request_mode)", pointai_text)
        self.assertIn("self.pre_img(request_mode=request_mode)", vision_text)
        self.assertIn("request_mode == PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT", pointai_text)
        self.assertIn("request_mode == PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT", vision_text)
        self.assertNotIn("自适应高度已满足2x2矩阵要求", pointai_text)
        self.assertNotIn("自适应高度已满足2x2矩阵要求", vision_text)
        self.assertNotIn("自适应高度不检查绑扎高度和2x2数量", pointai_text)
        self.assertNotIn("不要求2x2矩阵或4点数量", pointai_text)
        self.assertNotIn("不要求2x2矩阵或4点数量", vision_text)

    def test_apply_spatial_calibration_uses_tf_result_without_offset_addition(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.fixed_z_value = 0.0
        processor.calibration_offset_x = 9999.0
        processor.calibration_offset_y = -9999.0
        processor.calibration_offset_z = 5555.0
        processor.transform_to_gripper_frame = lambda x, y, z, idx: (11, 22, 33)

        calibrated = processor.apply_spatial_calibration(1.0, 2.0, 3.0, 7)

        self.assertEqual(calibrated, [11.0, 22.0, 33.0])

    def test_apply_spatial_calibration_keeps_fixed_z_override_after_tf(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.fixed_z_value = 88.0
        processor.transform_to_gripper_frame = lambda x, y, z, idx: (11, 22, 33)

        calibrated = processor.apply_spatial_calibration(1.0, 2.0, 3.0, 7)

        self.assertEqual(calibrated, [11.0, 22.0, 88.0])

    def test_visual_scripts_use_tf_as_unique_spatial_calibration_source(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

        self.assertIn("def apply_spatial_calibration(", pointai_text)
        self.assertIn("def apply_spatial_calibration(", vision_text)
        self.assertIn(
            "calibrated_world_coord = self.apply_spatial_calibration(",
            pointai_text,
        )
        self.assertIn(
            "calibrated_world_coord = self.apply_spatial_calibration(",
            vision_text,
        )
        self.assertNotIn("x_value + self.calibration_offset_x", pointai_text)
        self.assertNotIn("y_value + self.calibration_offset_y", pointai_text)
        self.assertNotIn("z_value + self.calibration_offset_z", pointai_text)
        self.assertNotIn("x_value + self.calibration_offset_x", vision_text)
        self.assertNotIn("y_value + self.calibration_offset_y", vision_text)
        self.assertNotIn("z_value + self.calibration_offset_z", vision_text)

    def test_calibration_offset_callback_updates_gripper_tf_translation_mm(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        with tempfile.TemporaryDirectory() as temp_dir:
            tf_config_path = Path(temp_dir) / "gripper_tf.yaml"
            tf_config_path.write_text(
                yaml.safe_dump(
                    {
                        "parent_frame": "Scepter_depth_frame",
                        "child_frame": "gripper_frame",
                        "translation_mm": {"x": 1.0, "y": 2.0, "z": 3.0},
                        "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                    },
                    sort_keys=False,
                    allow_unicode=True,
                ),
                encoding="utf-8",
            )
            processor.gripper_tf_config_file = str(tf_config_path)
            pose = pointAI.Pose()
            pose.position.x = 123.0
            pose.position.y = 456.0
            pose.position.z = 789.0

            processor.calibration_offset_callback(pose)

            config = yaml.safe_load(tf_config_path.read_text(encoding="utf-8"))
            self.assertEqual(
                config["translation_mm"],
                {"x": 123.0, "y": 456.0, "z": 789.0},
            )

    def test_lashing_config_only_keeps_height_threshold(self):
        config_text = (CHASSIS_CTRL_DIR / "data" / "lashing_config.json").read_text(
            encoding="utf-8"
        )
        self.assertIn('"height_threshold"', config_text)
        self.assertNotIn('"cal_x"', config_text)
        self.assertNotIn('"cal_y"', config_text)
        self.assertNotIn('"cal_z"', config_text)

    def test_visual_scripts_write_tf_translation_mm_from_legacy_topic(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

        self.assertIn("gripper_tf.yaml", pointai_text)
        self.assertIn("translation_mm", pointai_text)
        self.assertIn("gripper_tf.yaml", vision_text)
        self.assertIn("translation_mm", vision_text)
        self.assertNotIn("已忽略旧的set_pointAI_offset请求", pointai_text)
        self.assertNotIn("已忽略旧的set_pointAI_offset请求", vision_text)

    def test_debug_and_transfer_logs_reference_tf_translation_calibration(self):
        debug_button_text = (CHASSIS_CTRL_DIR / "scripts" / "debug_button_node.py").read_text(
            encoding="utf-8"
        )
        topics_transfer_text = (CHASSIS_CTRL_DIR / "src" / "topics_transfer.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("设置TF平移标定", debug_button_text)
        self.assertIn("gripper_tf.yaml.translation_mm", topics_transfer_text)

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

    def test_bind_mode_accepts_point_at_ninety_five_mm_limit(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        point_coords = make_points_array([95.0], idx_values=[1])

        result = processor.evaluate_point_coords_for_mode(
            point_coords,
            request_mode=2
        )

        self.assertTrue(result["success"])
        self.assertEqual(result["out_of_height_count"], 0)
        self.assertEqual(result["out_of_height_point_indices"], [])
        self.assertEqual(result["out_of_height_z_values"], [])

    def test_bind_mode_reports_points_above_ninety_five_mm_with_details(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        point_coords = make_points_array([90.0, 95.5, 101.0], idx_values=[1, 2, 4])

        result = processor.evaluate_point_coords_for_mode(
            point_coords,
            request_mode=2
        )

        self.assertTrue(result["success"])
        self.assertEqual(result["out_of_height_count"], 2)
        self.assertEqual(result["out_of_height_point_indices"], [2, 4])
        self.assertEqual(result["out_of_height_z_values"], [95.5, 101.0])
        self.assertIn("95mm", result["message"])
        self.assertIn("仅作日志", result["message"])

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

    def test_bind_mode_reports_points_above_ninety_five_mm_without_rejecting(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        point_coords = make_points_array_with_world_coords(
            [(10.0, 20.0, 90.0), (30.0, 40.0, 95.5), (50.0, 60.0, 101.0)],
            idx_values=[1, 2, 4]
        )

        result = processor.evaluate_point_coords_for_mode(
            point_coords,
            request_mode=2
        )

        self.assertTrue(result["success"])
        self.assertEqual(result["out_of_height_count"], 2)
        self.assertEqual(result["out_of_height_point_indices"], [2, 4])
        self.assertEqual(result["out_of_height_z_values"], [95.5, 101.0])
        self.assertIn("仅作日志", result["message"])

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

    def test_coordinate_window_accepts_three_frames_inside_five_mm_tolerance(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.stable_frame_count = 3
        processor.stable_z_tolerance_mm = 5.0
        snapshots = [
            processor.build_coordinate_snapshot(
                make_points_array_with_world_coords([(100.0, 200.0, 100.0)])
            ),
            processor.build_coordinate_snapshot(
                make_points_array_with_world_coords([(103.0, 197.0, 104.0)])
            ),
            processor.build_coordinate_snapshot(
                make_points_array_with_world_coords([(96.0, 204.0, 96.0)])
            ),
        ]

        self.assertTrue(processor.is_stable_coordinate_window(snapshots))

    def test_coordinate_window_rejects_three_frames_when_xy_exceeds_tolerance(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.stable_frame_count = 3
        processor.stable_z_tolerance_mm = 5.0
        snapshots = [
            processor.build_coordinate_snapshot(
                make_points_array_with_world_coords([(100.0, 200.0, 100.0)])
            ),
            processor.build_coordinate_snapshot(
                make_points_array_with_world_coords([(111.0, 200.0, 102.0)])
            ),
            processor.build_coordinate_snapshot(
                make_points_array_with_world_coords([(100.0, 200.0, 101.0)])
            ),
        ]

        self.assertFalse(processor.is_stable_coordinate_window(snapshots))

    def test_bind_mode_uses_coordinate_stability_without_height_rejection_in_visual_scripts(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

        self.assertIn("def build_coordinate_snapshot", pointai_text)
        self.assertIn("def build_coordinate_snapshot", vision_text)
        self.assertIn("def is_stable_coordinate_window", pointai_text)
        self.assertIn("def is_stable_coordinate_window", vision_text)
        self.assertIn("self.is_stable_coordinate_window(stable_snapshots)", pointai_text)
        self.assertIn("self.is_stable_coordinate_window(stable_snapshots)", vision_text)
        self.assertNotIn('if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK and result["out_of_height_count"] > 0', pointai_text)
        self.assertNotIn('if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK and result["out_of_height_count"] > 0', vision_text)


if __name__ == "__main__":
    unittest.main()
