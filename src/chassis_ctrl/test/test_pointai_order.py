#!/usr/bin/env python3

import os
import sys
import unittest
import inspect
import tempfile
import re
import json
from pathlib import Path

import yaml
import numpy as np


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts")
)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import pointAI  # noqa: E402


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = Path(__file__).resolve().parents[3]
CHASSIS_CTRL_DIR = WORKSPACE_ROOT / "chassis_ctrl"


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


def extract_cpp_block(source_text, anchor):
    start = source_text.index(anchor)
    brace_start = source_text.index("{", start)
    depth = 0
    for idx in range(brace_start, len(source_text)):
        char = source_text[idx]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source_text[start:idx + 1]
    raise ValueError(f"could not find matching brace for {anchor!r}")


class PointAIOrderTest(unittest.TestCase):
    def test_bind_execution_memory_load_rejects_semantically_corrupted_ledger(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        load_function = extract_cpp_block(
            suoqu_text,
            "bool load_bind_execution_memory_json(",
        )

        self.assertIn("if (!memory_json.is_object())", load_function)
        self.assertIn('if (!memory_json.contains("executed_points"))', load_function)
        self.assertIn('!memory_json["executed_points"].is_array()', load_function)
        self.assertIn(
            'if (memory_json.contains("path_origin") && !memory_json["path_origin"].is_object())',
            load_function,
        )
        self.assertIn("if (!point_json.is_object())", load_function)
        self.assertIn('!point_json.contains("global_row")', load_function)
        self.assertIn('!point_json.contains("global_col")', load_function)
        self.assertRegex(
            load_function,
            re.compile(r'error_message\s*=\s*kBindExecutionMemoryUnreadableError;'),
        )
        self.assertRegex(load_function, re.compile(r"return false;"))

    def test_bind_execution_memory_load_is_fail_closed_when_existing_file_is_unreadable(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn(
            "load_bind_execution_memory_json(",
            suoqu_text,
        )
        self.assertIn(
            "kBindExecutionMemoryUnreadableError",
            suoqu_text,
        )

        for anchor in [
            "bool run_current_area_bind_from_scan_test(std::string& message)",
            "bool run_live_visual_global_work(std::string& message)",
            "bool run_bind_from_scan(std::string& message)",
        ]:
            function_text = extract_cpp_block(suoqu_text, anchor)
            self.assertIn(
                "if (!load_bind_execution_memory_json(bind_execution_memory, bind_execution_memory_error))",
                function_text,
            )
            self.assertRegex(
                function_text,
                re.compile(r"message\s*=\s*bind_execution_memory_error;"),
            )
            self.assertRegex(function_text, re.compile(r"return false;"))

    def test_scan_rebuild_only_resets_execution_memory_after_both_artifacts_write(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn("bool write_pseudo_slam_points_json(", suoqu_text)
        self.assertIn("bool write_pseudo_slam_bind_path_json(", suoqu_text)
        self.assertIn("const bool pseudo_slam_points_written =", scan_function)
        self.assertIn("const bool pseudo_slam_bind_path_written =", scan_function)
        self.assertIn(
            "if (!pseudo_slam_points_written || !pseudo_slam_bind_path_written)",
            scan_function,
        )
        self.assertIn(
            "扫描完成后未重置bind_execution_memory.json",
            scan_function,
        )
        self.assertGreater(
            scan_function.index("reset_bind_execution_memory_for_scan_session"),
            scan_function.index("if (!pseudo_slam_points_written || !pseudo_slam_bind_path_written)"),
        )

    def test_scan_rebuild_reports_failure_when_execution_memory_reset_fails(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn(
            "if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error))",
            scan_function,
        )
        self.assertIn("bind_execution_memory.json重置失败", scan_function)
        self.assertRegex(
            scan_function,
            re.compile(r'message\s*=\s*".*bind_execution_memory\.json重置失败.*";'),
        )
        failure_block = scan_function[
            scan_function.index(
                "if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error))"
            ):
        ]
        self.assertRegex(failure_block, re.compile(r"return false;"))

    def test_scan_rebuild_writes_scan_session_id_into_both_scan_artifacts(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        write_points_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_points_json(",
        )
        write_bind_path_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_bind_path_json(",
        )
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn("const std::string& scan_session_id", write_points_text)
        self.assertIn('points_json["scan_session_id"] = scan_session_id;', write_points_text)
        self.assertIn("const std::string& scan_session_id", write_bind_path_text)
        self.assertIn('bind_path_json["scan_session_id"] = scan_session_id;', write_bind_path_text)
        self.assertLess(
            scan_function.index("const std::string scan_session_id ="),
            scan_function.index("const bool pseudo_slam_points_written ="),
        )
        self.assertIn("write_pseudo_slam_points_json(", scan_function)
        self.assertIn("scan_session_id,", scan_function)
        self.assertIn("write_pseudo_slam_bind_path_json(", scan_function)

    def test_precomputed_bind_entrypoints_fail_closed_on_scan_session_mismatch(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        validate_function = extract_cpp_block(
            suoqu_text,
            "bool validate_scan_session_alignment(",
        )
        load_artifacts_function = extract_cpp_block(
            suoqu_text,
            "bool load_scan_artifacts_for_execution(",
        )

        self.assertIn('artifact_json.value("scan_session_id", std::string())', validate_function)
        self.assertIn("bind_execution_memory.scan_session_id.empty()", validate_function)
        self.assertIn("artifact_scan_session_id != bind_execution_memory.scan_session_id", validate_function)
        self.assertIn("scan_session_id不一致", validate_function)
        self.assertIn("fail-closed", validate_function)
        self.assertIn("pseudo_slam_points_json_file", load_artifacts_function)
        self.assertIn("pseudo_slam_bind_path_json_file", load_artifacts_function)
        self.assertRegex(
            load_artifacts_function,
            re.compile(
                r'validate_scan_session_alignment\(\s*points_json,\s*"pseudo_slam_points\.json",\s*bind_execution_memory,\s*error_message',
                re.S,
            ),
        )
        self.assertRegex(
            load_artifacts_function,
            re.compile(
                r'validate_scan_session_alignment\(\s*bind_path_json,\s*"pseudo_slam_bind_path\.json",\s*bind_execution_memory,\s*error_message',
                re.S,
            ),
        )

        for anchor in [
            "bool run_current_area_bind_from_scan_test(std::string& message)",
            "bool run_bind_from_scan(std::string& message)",
        ]:
            function_text = extract_cpp_block(suoqu_text, anchor)
            load_stmt = (
                "if (!load_bind_execution_memory_json(bind_execution_memory, bind_execution_memory_error))"
            )
            load_artifacts_stmt = (
                "if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"
            )

            self.assertIn(load_stmt, function_text)
            self.assertIn(
                load_artifacts_stmt,
                function_text,
            )
            validation_block = function_text[
                function_text.index(
                    load_artifacts_stmt
                ):
            ]
            self.assertRegex(validation_block, re.compile(r"return false;"))
            self.assertLess(function_text.index(load_stmt), function_text.index(load_artifacts_stmt))

        current_area_text = extract_cpp_block(
            suoqu_text,
            "bool run_current_area_bind_from_scan_test(std::string& message)",
        )
        self.assertLess(
            current_area_text.index(
                "if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"
            ),
            current_area_text.index("float current_cabin_x = 0.0f;"),
        )
        self.assertLess(
            current_area_text.index(
                "if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"
            ),
            current_area_text.index("find_nearest_bind_area_for_current_cabin_pose("),
        )
        self.assertLess(
            current_area_text.index(
                "if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"
            ),
            current_area_text.index("TCP_Move[0] = fast_cabin_speed;"),
        )

        global_bind_text = extract_cpp_block(
            suoqu_text,
            "bool run_bind_from_scan(std::string& message)",
        )
        self.assertLess(
            global_bind_text.index(
                "if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"
            ),
            global_bind_text.index('const auto& areas_json = bind_path_json["areas"];'),
        )
        self.assertLess(
            global_bind_text.index(
                "if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"
            ),
            global_bind_text.index("float path_origin_x = 0.0f;"),
        )
        self.assertLess(
            global_bind_text.index(
                "if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"
            ),
            global_bind_text.index("TCP_Move[0] = cabin_speed;"),
        )

    def test_execution_filters_duplicate_checkerboard_cells_within_current_batch(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        filter_function = extract_cpp_block(
            suoqu_text,
            "nlohmann::json filter_precomputed_group_points_for_execution(",
        )

        self.assertIn("std::unordered_set<long long> current_batch_checkerboard_cells;", filter_function)
        self.assertIn("encode_checkerboard_cell_key(global_row, global_col)", filter_function)
        self.assertIn(
            "const bool inserted = current_batch_checkerboard_cells.insert(checkerboard_cell_key).second;",
            filter_function,
        )
        self.assertIn("if (!inserted) {", filter_function)
        self.assertLess(
            filter_function.index("current_batch_checkerboard_cells.insert(checkerboard_cell_key).second"),
            filter_function.index("filtered_points.push_back(point_json);"),
        )

    def test_execution_filters_blocked_outlier_global_indices_from_points_json(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        filter_function = extract_cpp_block(
            suoqu_text,
            "nlohmann::json filter_precomputed_group_points_for_execution(",
        )

        self.assertIn("const std::unordered_set<int>& blocked_global_indices", filter_function)
        self.assertIn('const int global_idx = point_json.value("global_idx", point_json.value("idx", -1));', filter_function)
        self.assertIn("if (global_idx > 0 && blocked_global_indices.count(global_idx) > 0) {", filter_function)

        self.assertIn("collect_blocked_execution_global_indices_from_points_json", suoqu_text)
        self.assertIn('point_json.value("is_planning_outlier", false)', suoqu_text)
        self.assertIn('point_json.value("is_planning_outlier_line_member", false)', suoqu_text)
        self.assertIn('point_json.value("is_outlier_secondary_plane_member", false)', suoqu_text)
        self.assertIn('point_json.value("is_outlier_column_neighbor_blocked", false)', suoqu_text)
        self.assertIn('!point_json.value("is_planning_checkerboard_member", false)', suoqu_text)

    def test_execution_entrypoints_use_points_json_outlier_flags_to_block_bind_path_points(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn(
            "const std::unordered_set<int> blocked_global_indices =\n        collect_blocked_execution_global_indices_from_points_json(points_json);",
            suoqu_text,
        )
        self.assertIn(
            "execution_group_json[\"points\"] = filter_precomputed_group_points_for_execution(\n            group_json,\n            bind_execution_memory,\n            blocked_global_indices,",
            suoqu_text,
        )
        self.assertIn(
            "execution_group_json[\"points\"] = filter_precomputed_group_points_for_execution(\n            execution_group_json,\n            bind_execution_memory,\n            blocked_global_indices,",
            suoqu_text,
        )

    def test_scan_artifacts_and_execution_memory_persist_path_signature(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        load_memory_text = extract_cpp_block(
            suoqu_text,
            "bool load_bind_execution_memory_json(",
        )
        write_memory_text = extract_cpp_block(
            suoqu_text,
            "bool write_bind_execution_memory_json(",
        )
        reset_memory_text = extract_cpp_block(
            suoqu_text,
            "BindExecutionMemory reset_bind_execution_memory_for_scan_session(",
        )
        write_points_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_points_json(",
        )
        write_bind_path_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_bind_path_json(",
        )
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn("std::string path_signature;", suoqu_text)
        self.assertIn('memory.path_signature = memory_json.value("path_signature", "");', load_memory_text)
        self.assertIn('{"path_signature", memory.path_signature}', write_memory_text)
        self.assertIn("const std::string& path_signature", reset_memory_text)
        self.assertIn("memory.path_signature = path_signature;", reset_memory_text)
        self.assertIn("const std::string& path_signature", write_points_text)
        self.assertIn('points_json["path_signature"] = path_signature;', write_points_text)
        self.assertIn("const std::string& path_signature", write_bind_path_text)
        self.assertIn('bind_path_json["path_signature"] = path_signature;', write_bind_path_text)
        self.assertIn("const std::string path_signature =", scan_function)
        self.assertIn("build_path_signature(con_path, cabin_height, cabin_speed)", scan_function)
        self.assertRegex(
            scan_function,
            re.compile(r"write_pseudo_slam_points_json\([\s\S]*scan_session_id,\s*path_signature,"),
        )
        self.assertRegex(
            scan_function,
            re.compile(r"write_pseudo_slam_bind_path_json\([\s\S]*scan_session_id,\s*path_signature,"),
        )
        self.assertIn(
            "reset_bind_execution_memory_for_scan_session(scan_session_id, path_signature, path_origin)",
            scan_function,
        )

    def test_execution_entrypoints_fail_closed_on_path_signature_mismatch(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        validate_function = extract_cpp_block(
            suoqu_text,
            "bool validate_path_signature_alignment(",
        )
        load_artifacts_function = extract_cpp_block(
            suoqu_text,
            "bool load_scan_artifacts_for_execution(",
        )

        self.assertIn("const std::string& current_path_signature", validate_function)
        self.assertIn('artifact_json.value("path_signature", std::string())', validate_function)
        self.assertIn("bind_execution_memory.path_signature", validate_function)
        self.assertIn("artifact_path_signature != current_path_signature", validate_function)
        self.assertIn("bind_execution_memory.path_signature != current_path_signature", validate_function)
        self.assertIn("path_signature不一致", validate_function)
        self.assertIn("请先重新扫描", validate_function)
        self.assertIn("const std::string& current_path_signature", load_artifacts_function)
        self.assertRegex(
            load_artifacts_function,
            re.compile(
                r'validate_path_signature_alignment\(\s*points_json,\s*"pseudo_slam_points\.json",\s*bind_execution_memory,\s*current_path_signature,\s*error_message',
                re.S,
            ),
        )
        self.assertRegex(
            load_artifacts_function,
            re.compile(
                r'validate_path_signature_alignment\(\s*bind_path_json,\s*"pseudo_slam_bind_path\.json",\s*bind_execution_memory,\s*current_path_signature,\s*error_message',
                re.S,
            ),
        )

        for anchor in [
            "bool run_current_area_bind_from_scan_test(std::string& message)",
            "bool run_live_visual_global_work(std::string& message)",
            "bool run_bind_from_scan(std::string& message)",
        ]:
            function_text = extract_cpp_block(suoqu_text, anchor)
            self.assertIn(
                "std::string current_path_signature;",
                function_text,
            )
            self.assertIn(
                "if (!load_current_path_signature_for_execution(current_path_signature, message))",
                function_text,
            )
            self.assertIn(
                "if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))",
                function_text,
            )
            self.assertLess(
                function_text.index("if (!load_current_path_signature_for_execution(current_path_signature, message))"),
                function_text.index(
                    "if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"
                ),
            )

    def test_quickstart_requires_rescan_after_path_change(self):
        quickstart_text = (REPO_ROOT / "SLAM_V11_QUICKSTART.md").read_text(encoding="utf-8")

        self.assertIn("path_signature", quickstart_text)
        self.assertIn("改路径后必须重新扫描", quickstart_text)
        self.assertIn("关键运动参数", quickstart_text)

    def test_live_visual_entrypoint_fail_closed_on_scan_session_mismatch(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        load_grid_text = extract_cpp_block(
            suoqu_text,
            "bool load_live_visual_checkerboard_grid(",
        )
        live_visual_text = extract_cpp_block(
            suoqu_text,
            "bool run_live_visual_global_work(std::string& message)",
        )

        self.assertIn("const nlohmann::json& points_json", load_grid_text)
        self.assertIn(
            'if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))',
            live_visual_text,
        )
        self.assertIn(
            "if (!load_live_visual_checkerboard_grid(points_json, checkerboard_grid, checkerboard_grid_error))",
            live_visual_text,
        )
        self.assertNotIn("std::ifstream infile(pseudo_slam_points_json_file);", load_grid_text)
        self.assertLess(
            live_visual_text.index("if (!load_bind_execution_memory_json(bind_execution_memory, bind_execution_memory_error))"),
            live_visual_text.index("if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"),
        )
        self.assertLess(
            live_visual_text.index("if (!load_scan_artifacts_for_execution(points_json, bind_path_json, bind_execution_memory, current_path_signature, message))"),
            live_visual_text.index("if (!load_live_visual_checkerboard_grid(points_json, checkerboard_grid, checkerboard_grid_error))"),
        )

    def test_execution_memory_write_failure_invalidates_current_scan_artifacts(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        invalidate_file_helper_text = extract_cpp_block(
            suoqu_text,
            "bool invalidate_scan_session_in_artifact_file(",
        )
        invalidate_helper_text = extract_cpp_block(
            suoqu_text,
            "bool invalidate_current_scan_artifacts_after_execution_memory_write_failure(",
        )

        self.assertIn("pseudo_slam_points_json_file", invalidate_helper_text)
        self.assertIn("pseudo_slam_bind_path_json_file", invalidate_helper_text)
        self.assertIn('artifact_json["scan_session_id"] = "";', invalidate_file_helper_text)
        self.assertIn('"scan_session_invalid_reason"', invalidate_file_helper_text)
        self.assertIn('"requires_rescan"', invalidate_file_helper_text)
        self.assertIn("需要重新扫描/重新建图", invalidate_helper_text)
        self.assertRegex(invalidate_helper_text, re.compile(r"return false;"))

    def test_execution_entrypoints_fail_closed_when_execution_memory_persist_fails(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        failure_expectations = {
            "bool run_current_area_bind_from_scan_test(std::string& message)": (
                "if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error))",
                "当前区域预计算直执行",
            ),
            "bool run_live_visual_global_work(std::string& message)": (
                "if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error))",
                "live_visual",
            ),
            "bool run_bind_from_scan(std::string& message)": (
                "if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error))",
                "bind_from_scan",
            ),
        }

        for anchor, (write_failure_stmt, path_label) in failure_expectations.items():
            function_text = extract_cpp_block(suoqu_text, anchor)
            failure_block = function_text[function_text.index(write_failure_stmt):]

            self.assertIn(
                "invalidate_current_scan_artifacts_after_execution_memory_write_failure(",
                failure_block,
            )
            self.assertIn("bind_execution_memory.json写入失败", failure_block)
            self.assertIn("需要重新扫描/重新建图", failure_block)
            self.assertIn(path_label, failure_block)
            self.assertRegex(
                failure_block,
                re.compile(r'message\s*=\s*".*重新扫描/重新建图.*";', re.S),
            )
            self.assertRegex(failure_block, re.compile(r"return false;"))

    def test_live_visual_uses_planning_authoritative_checkerboard_fields(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        write_points_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_points_json(",
        )
        load_grid_text = extract_cpp_block(
            suoqu_text,
            "bool load_live_visual_checkerboard_grid(",
        )

        self.assertIn("planning_checkerboard_info_by_idx", write_points_text)
        self.assertIn('"is_planning_checkerboard_member"', write_points_text)
        self.assertIn('"planning_global_row"', write_points_text)
        self.assertIn('"planning_global_col"', write_points_text)
        self.assertIn('"planning_checkerboard_parity"', write_points_text)
        self.assertIn(
            'if (!point_json.value("is_planning_checkerboard_member", false))',
            load_grid_text,
        )
        self.assertIn(
            'point_json.value("planning_global_row", -1)',
            load_grid_text,
        )
        self.assertIn(
            'point_json.value("planning_global_col", -1)',
            load_grid_text,
        )
        self.assertIn(
            'point_json.value("planning_checkerboard_parity", -1)',
            load_grid_text,
        )

    def test_quickstart_documents_joint_scan_artifact_validation(self):
        quickstart_text = (REPO_ROOT / "SLAM_V11_QUICKSTART.md").read_text(encoding="utf-8")

        self.assertIn("三个执行入口现在都会联合校验", quickstart_text)
        self.assertIn("pseudo_slam_points.json", quickstart_text)
        self.assertIn("pseudo_slam_bind_path.json", quickstart_text)
        self.assertIn("不是只校验各自直接消费的那一份", quickstart_text)
        self.assertIn("任意一份扫描产物已失效或 session 不对齐", quickstart_text)

    def test_pseudo_slam_entrypoints_share_single_serial_workflow_lock(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        self.assertIn("std::mutex pseudo_slam_workflow_mutex;", suoqu_text)

        lock_stmt = "std::lock_guard<std::mutex> pseudo_slam_workflow_lock(pseudo_slam_workflow_mutex);"
        anchors_and_followups = [
            (
                "bool run_pseudo_slam_scan(",
                "set_pseudo_slam_tf_points({});",
            ),
            (
                "bool run_current_area_bind_from_scan_test(std::string& message)",
                "if (!load_bind_execution_memory_json(bind_execution_memory, bind_execution_memory_error))",
            ),
            (
                "bool run_live_visual_global_work(std::string& message)",
                "if (!load_bind_execution_memory_json(bind_execution_memory, bind_execution_memory_error))",
            ),
            (
                "bool run_bind_from_scan(std::string& message)",
                "if (!load_bind_execution_memory_json(bind_execution_memory, bind_execution_memory_error))",
            ),
        ]

        for anchor, followup in anchors_and_followups:
            function_text = extract_cpp_block(suoqu_text, anchor)
            self.assertIn(lock_stmt, function_text)
            self.assertLess(function_text.index(lock_stmt), function_text.index(followup))

    def test_scan_only_requests_use_fresh_process_image_objects(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )
        live_visual_function = extract_cpp_block(
            suoqu_text,
            "bool run_live_visual_global_work(std::string& message)",
        )

        self.assertNotIn("chassis_ctrl::ProcessImage srv;", suoqu_text)
        self.assertNotIn("AI_client.call(srv)", suoqu_text)
        self.assertNotRegex(scan_function, re.compile(r"(?<![A-Za-z0-9_])srv\.response"))
        self.assertNotRegex(live_visual_function, re.compile(r"(?<![A-Za-z0-9_])srv\.response"))
        self.assertRegex(
            scan_function,
            re.compile(
                r"chassis_ctrl::ProcessImage scan_srv;[\s\S]*?"
                r"scan_srv.request.request_mode = kProcessImageModeScanOnly;[\s\S]*?"
                r"AI_client.call\(scan_srv\)",
                re.S,
            ),
        )
        self.assertNotIn("collected_scan_frame_count", scan_function)
        self.assertRegex(
            live_visual_function,
            re.compile(
                r"for\s*\(int area_order_index = 0; area_order_index < total_area_count; \+\+area_order_index\)\s*\{[\s\S]*?"
                r"chassis_ctrl::ProcessImage scan_srv;[\s\S]*?"
                r"scan_srv.request.request_mode = kProcessImageModeExecutionRefine;[\s\S]*?"
                r"AI_client.call\(scan_srv\)",
                re.S,
            ),
        )

    def test_process_image_service_supports_request_modes_and_failure_details(self):
        service_path = CHASSIS_CTRL_DIR / "srv" / "ProcessImage.srv"
        service_text = service_path.read_text(encoding="utf-8")
        self.assertIn("uint8 MODE_DEFAULT=0", service_text)
        self.assertIn("uint8 MODE_ADAPTIVE_HEIGHT=1", service_text)
        self.assertIn("uint8 MODE_BIND_CHECK=2", service_text)
        self.assertIn("uint8 MODE_SCAN_ONLY=3", service_text)
        self.assertIn("uint8 MODE_EXECUTION_REFINE=4", service_text)
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

    def test_suoqu_slam_execution_path_no_longer_uses_live_visual_calls(self):
        suoqu_path = CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp"
        suoqu_text = suoqu_path.read_text(encoding="utf-8")
        self.assertNotIn("bool adaptive_height(", suoqu_text)
        self.assertNotIn("if (!adaptive_height(", suoqu_text)
        self.assertNotIn("开始视觉识别", suoqu_text)
        self.assertNotIn("parse_bind_height_excess_mm", suoqu_text)
        self.assertNotIn("adjust_cabin_height_for_bind_excess", suoqu_text)

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
        vision_text = pointai_text

        self.assertNotIn("def snake_sort(", pointai_text)
        self.assertNotIn("def snake_sort(", vision_text)
        self.assertIn("def sort_matrix_points(", pointai_text)
        self.assertIn("def sort_matrix_points(", vision_text)

    def test_suoqu_no_longer_retries_bind_after_height_excess_adjustment(self):
        suoqu_path = CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp"
        suoqu_text = suoqu_path.read_text(encoding="utf-8")
        self.assertNotIn("parse_bind_height_excess_mm", suoqu_text)
        self.assertNotIn("adjust_cabin_height_for_bind_excess", suoqu_text)
        self.assertNotIn("max_bind_height_adjust_retries", suoqu_text)
        self.assertNotIn("TCP_Move[3] = adjusted_height", suoqu_text)
        self.assertNotIn("adjusted_height = old_height - height_excess_mm", suoqu_text)

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

    def test_filter_close_points_default_excludes_all_points_connected_within_one_hundred_mm(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        points = [
            (1, [10, 10, [0, 0, 100]], [0.0, 0.0, 0.0]),
            (2, [20, 20, [0, 0, 100]], [60.0, 0.0, 0.0]),
            (3, [30, 30, [0, 0, 100]], [120.0, 0.0, 0.0]),
            (4, [120, 120, [0, 0, 100]], [260.0, 260.0, 0.0]),
        ]

        filtered_points = processor.filter_close_points_by_origin(points)

        self.assertEqual([item[0] for item in filtered_points], [4])

    def test_scan_only_keeps_single_frame_close_points_until_global_scan_filter(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        points = [
            (1, [10, 10, [0, 0, 100]], [0.0, 0.0, 0.0]),
            (2, [20, 20, [0, 0, 100]], [60.0, 0.0, 0.0]),
            (3, [30, 30, [0, 0, 100]], [120.0, 0.0, 0.0]),
            (4, [120, 120, [0, 0, 100]], [260.0, 260.0, 0.0]),
        ]

        scan_points, scan_removed_count = processor.filter_candidate_centers_for_request_mode(
            points,
            pointAI.PROCESS_IMAGE_MODE_SCAN_ONLY,
        )
        default_points, default_removed_count = processor.filter_candidate_centers_for_request_mode(
            points,
            pointAI.PROCESS_IMAGE_MODE_DEFAULT,
        )

        self.assertEqual([item[0] for item in scan_points], [1, 2, 3, 4])
        self.assertEqual(scan_removed_count, 0)
        self.assertEqual([item[0] for item in default_points], [4])
        self.assertEqual(default_removed_count, 3)

    def test_close_point_filter_defaults_to_one_hundred_mm_in_visual_scripts(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = pointai_text

        self.assertIn("def filter_close_points_by_origin(self, centers, min_distance_mm=100.0):", pointai_text)
        self.assertIn("def filter_close_points_by_origin(self, centers, min_distance_mm=100.0):", vision_text)

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
        processor.travel_range_max_x_mm = 360.0
        processor.travel_range_max_y_mm = 320.0

        reasons = processor.get_travel_range_reject_reasons(361.0, -1.0)

        self.assertEqual(reasons, ["X超过360", "Y小于0"])

    def test_pointai_logs_chinese_range_filter_details(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")

        self.assertIn("范围外原因统计", pointai_text)
        self.assertIn("可执行范围内点数不足4个", pointai_text)
        self.assertIn("范围外原因统计", pointai_text)
        self.assertIn("无法组成2x2矩阵", pointai_text)

    def test_build_detection_summary_log_uses_multiline_block(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.travel_range_max_x_mm = 320.0
        processor.travel_range_max_y_mm = 360.0
        processor.matrix_selection_max_x_mm = 500.0
        processor.matrix_selection_max_y_mm = 500.0

        message = processor.build_detection_summary_log(
            request_mode=pointAI.PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT,
            raw_candidate_count=24,
            duplicate_removed_count=0,
            in_range_candidate_count=0,
            out_of_range_point_count=24,
            selected_count=0,
            output_count=0,
            out_of_range_reason_counts={"超出自适应采集框": 24},
            out_of_range_samples=[
                "idx=8,pix=(23,434),coord=(-301.0,-67.0,255.0),原因=超出自适应采集框",
                "idx=9,pix=(100,437),coord=(-296.0,-205.0,254.0),原因=超出自适应采集框",
            ],
        )

        self.assertIn("pointAI调试:\n", message)
        self.assertIn("  模式: adaptive_height", message)
        self.assertIn("  可执行范围过滤: 原始候选=24, 去重移除=0, 范围内=0, 范围外=24, 2x2选中=0, 本次输出=0", message)
        self.assertIn("  范围限制: 白框ROI，并在边缘按全局工作区自适应裁剪", message)
        self.assertIn("  范围外原因统计: 超出自适应采集框=24", message)
        self.assertIn("  样例:\n    - idx=8,pix=(23,434),coord=(-301.0,-67.0,255.0),原因=超出自适应采集框", message)
        self.assertIn("  结论: 自适应高度模式当前没有可用于高度平均的范围内点", message)
        self.assertTrue(message.endswith("\n"))

    def test_scan_workspace_uses_machine_center_anchor_and_frontend_workspace_geometry(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")

        self.assertIn('marking_x = float(path_json.get("marking_x", point_x_values[0]))', pointai_text)
        self.assertIn('marking_y = float(path_json.get("marking_y", point_y_values[0]))', pointai_text)
        self.assertIn('workspace_min_x = marking_x - robot_x_step / 2.0', pointai_text)
        self.assertIn('workspace_min_y = marking_y - robot_y_step / 2.0', pointai_text)
        self.assertIn('workspace_max_x = workspace_min_x + zone_x', pointai_text)
        self.assertIn('workspace_max_y = workspace_min_y + zone_y', pointai_text)
        self.assertNotIn('workspace_max_x = max(point_x_values) + robot_x_step / 2.0', pointai_text)
        self.assertNotIn('workspace_max_y = max(point_y_values) + robot_y_step / 2.0', pointai_text)
        self.assertNotIn("scan_workspace_padding_mm", pointai_text)

    def test_matrix_selection_range_has_independent_x_limit(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")

        self.assertIn('self.matrix_selection_max_x_mm = float(rospy.get_param("~matrix_selection_max_x_mm", 500.0))', pointai_text)
        self.assertIn('self.matrix_selection_max_y_mm = float(rospy.get_param("~matrix_selection_max_y_mm", 500.0))', pointai_text)
        self.assertIn("def is_point_in_matrix_selection_range(self, calibrated_x, calibrated_y):", pointai_text)
        self.assertIn('0 <= calibrated_x <= getattr(self, "matrix_selection_max_x_mm", 500.0)', pointai_text)
        self.assertIn('0 <= calibrated_y <= getattr(self, "matrix_selection_max_y_mm", 500.0)', pointai_text)

    def test_non_scan_modes_use_adaptive_roi_clipped_by_scan_workspace_for_matrix_selection(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")

        self.assertIn("def get_roi_pixel_mask(self):", pointai_text)
        self.assertIn("def is_point_in_matrix_selection_pixel_mask(self, pixel_x, pixel_y, pixel_mask=None):", pointai_text)
        self.assertIn(
            "matrix_selection_pixel_mask = None if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY else self.get_travel_range_pixel_mask()",
            pointai_text,
        )
        self.assertIn("point_is_allowed = self.is_point_in_matrix_selection_pixel_mask(", pointai_text)

    def test_scan_workspace_pixel_mask_uses_world_coordinate_channels(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.x_channel = np.array(
            [
                [0, 100, 200, 300],
                [0, 100, 200, 300],
                [0, 100, 200, 300],
                [0, 100, 200, 300],
            ],
            dtype=np.int32,
        )
        processor.y_channel = np.array(
            [
                [0, 0, 0, 0],
                [100, 100, 100, 100],
                [200, 200, 200, 200],
                [300, 300, 300, 300],
            ],
            dtype=np.int32,
        )
        processor.depth_v = np.full((4, 4), 100, dtype=np.int32)
        processor.lookup_transform_matrix_mm = lambda target_frame, source_frame="Scepter_depth_frame": np.eye(4)
        processor.load_scan_planning_workspace = lambda: {
            "min_x": 50.0,
            "max_x": 250.0,
            "min_y": 50.0,
            "max_y": 250.0,
        }

        mask = processor.get_scan_workspace_pixel_mask()

        expected_mask = np.array(
            [
                [0, 0, 0, 0],
                [0, 1, 1, 0],
                [0, 1, 1, 0],
                [0, 0, 0, 0],
            ],
            dtype=np.uint8,
        )
        np.testing.assert_array_equal(mask, expected_mask)

    def test_manual_workspace_quad_pixel_mask_overrides_rectangular_workspace_projection(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.x_channel = np.zeros((6, 6), dtype=np.int32)
        processor.y_channel = np.zeros((6, 6), dtype=np.int32)
        processor.depth_v = np.ones((6, 6), dtype=np.int32)
        processor.load_manual_workspace_quad = lambda: {
            "corner_pixels": [[1, 1], [4, 1], [4, 4], [1, 4]],
            "corner_world_cabin_frame": [
                [100.0, 100.0, 300.0],
                [400.0, 100.0, 300.0],
                [400.0, 400.0, 300.0],
                [100.0, 400.0, 300.0],
            ],
        }
        processor.load_scan_planning_workspace = lambda: {
            "min_x": 999.0,
            "max_x": 1000.0,
            "min_y": 999.0,
            "max_y": 1000.0,
        }

        mask = processor.get_scan_workspace_pixel_mask()

        expected_mask = np.array(
            [
                [0, 0, 0, 0, 0, 0],
                [0, 1, 1, 1, 1, 0],
                [0, 1, 1, 1, 1, 0],
                [0, 1, 1, 1, 1, 0],
                [0, 1, 1, 1, 1, 0],
                [0, 0, 0, 0, 0, 0],
            ],
            dtype=np.uint8,
        )
        np.testing.assert_array_equal(mask, expected_mask)

    def test_manual_workspace_quad_world_polygon_overrides_rectangular_workspace_filter(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.load_manual_workspace_quad = lambda: {
            "corner_pixels": [[10, 10], [20, 10], [20, 20], [10, 20]],
            "corner_world_cabin_frame": [
                [100.0, 100.0, 300.0],
                [300.0, 100.0, 300.0],
                [300.0, 300.0, 300.0],
                [100.0, 300.0, 300.0],
            ],
        }
        processor.load_scan_planning_workspace = lambda: {
            "min_x": 0.0,
            "max_x": 50.0,
            "min_y": 0.0,
            "max_y": 50.0,
        }

        self.assertTrue(processor.is_point_in_scan_workspace(200.0, 200.0))
        self.assertFalse(processor.is_point_in_scan_workspace(80.0, 200.0))

    def test_workspace_quad_callback_saves_pixel_and_cabin_frame_world_corners(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        with tempfile.TemporaryDirectory() as temp_dir:
            processor.manual_workspace_quad_file = os.path.join(temp_dir, "manual_workspace_quad.json")
            published_messages = []
            processor.manual_workspace_quad_pixels_pub = type(
                "PublisherStub",
                (),
                {"publish": lambda self, message: published_messages.append(message)},
            )()
            processor.get_valid_world_coord_near_pixel = lambda pixel_x, pixel_y, search_radius=6: (
                [int(pixel_x), int(pixel_y), 100],
                [int(pixel_x), int(pixel_y)],
                False,
            )
            processor.apply_spatial_calibration = lambda x_value, y_value, z_value, idx, target_frame="gripper_frame": [
                float(x_value + 100.0),
                float(y_value + 200.0),
                float(z_value + 300.0),
            ]

            msg = pointAI.Float32MultiArray(data=[30.0, 40.0, 50.0, 40.0, 50.0, 70.0, 30.0, 70.0])
            processor.manual_workspace_quad_callback(msg)

            with open(processor.manual_workspace_quad_file, "r", encoding="utf-8") as file_obj:
                manual_workspace_json = pointAI.json.load(file_obj)

        self.assertEqual(
            manual_workspace_json["corner_pixels"],
            [[30, 40], [50, 40], [50, 70], [30, 70]],
        )
        self.assertEqual(
            manual_workspace_json["corner_world_cabin_frame"],
            [
                [130.0, 240.0, 400.0],
                [150.0, 240.0, 400.0],
                [150.0, 270.0, 400.0],
                [130.0, 270.0, 400.0],
            ],
        )
        self.assertEqual(len(published_messages), 1)
        self.assertEqual(list(published_messages[0].data), [30.0, 40.0, 50.0, 40.0, 50.0, 70.0, 30.0, 70.0])

    def test_publish_current_manual_workspace_quad_pixels_uses_saved_corner_pixels(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        published_messages = []
        processor.manual_workspace_quad_pixels_pub = type(
            "PublisherStub",
            (),
            {"publish": lambda self, message: published_messages.append(message)},
        )()
        processor.load_manual_workspace_quad = lambda: {
            "corner_pixels": [[11, 22], [33, 44], [55, 66], [77, 88]],
            "corner_world_cabin_frame": [
                [100.0, 100.0, 300.0],
                [300.0, 100.0, 300.0],
                [300.0, 300.0, 300.0],
                [100.0, 300.0, 300.0],
            ],
        }

        processor.publish_current_manual_workspace_quad_pixels()

        self.assertEqual(len(published_messages), 1)
        self.assertEqual(list(published_messages[0].data), [11.0, 22.0, 33.0, 44.0, 55.0, 66.0, 77.0, 88.0])

    def test_workspace_s2_period_and_phase_estimation_prefers_periodic_peaks(self):
        profile = np.zeros(180, dtype=np.float32)
        for center_index in range(5, profile.size, 18):
            for offset, value in ((-1, 0.5), (0, 1.0), (1, 0.5)):
                sample_index = center_index + offset
                if 0 <= sample_index < profile.size:
                    profile[sample_index] = value

        estimated = pointAI.ImageProcessor.estimate_workspace_s2_period_and_phase(
            profile,
            min_period=10,
            max_period=30,
        )

        self.assertIsNotNone(estimated)
        self.assertEqual(estimated["period"], 18)
        self.assertEqual(estimated["phase"], 5)
        self.assertGreater(estimated["score"], 0.8)

    def test_workspace_s2_period_and_phase_estimation_prefers_true_fundamental_over_nearby_distractor(self):
        sample_x = np.arange(360, dtype=np.float32)
        rng = np.random.default_rng(0)
        profile = (
            1.0 * np.cos((2.0 * np.pi * sample_x) / 18.0)
            + 0.35 * np.cos((2.0 * np.pi * sample_x) / 14.0)
            + 0.3 * rng.normal(size=sample_x.shape)
        ).astype(np.float32)

        estimated = pointAI.ImageProcessor.estimate_workspace_s2_period_and_phase(
            profile,
            min_period=10,
            max_period=30,
        )

        self.assertIsNotNone(estimated)
        self.assertEqual(estimated["period"], 18)

    def test_workspace_s2_line_positions_follow_period_and_phase_inside_workspace(self):
        positions = pointAI.ImageProcessor.build_workspace_s2_line_positions(
            start_pixel=148,
            end_pixel=220,
            period_px=18,
            phase_px=4,
        )

        self.assertEqual(positions, [152, 170, 188, 206])

    def test_workspace_s2_projective_line_segments_follow_quad_geometry(self):
        quad_points = [[10, 10], [120, 28], [102, 102], [24, 86]]
        rectified_width = 100
        rectified_height = 80

        segments = pointAI.ImageProcessor.build_workspace_s2_projective_line_segments(
            quad_points,
            rectified_width,
            rectified_height,
            vertical_lines=[50],
            horizontal_lines=[40],
        )

        self.assertEqual(len(segments["vertical"]), 1)
        self.assertEqual(len(segments["horizontal"]), 1)

        vertical_segment = segments["vertical"][0]
        horizontal_segment = segments["horizontal"][0]

        self.assertNotEqual(vertical_segment[0][0], vertical_segment[1][0])
        self.assertNotEqual(horizontal_segment[0][1], horizontal_segment[1][1])

    def test_workspace_s2_rectified_geometry_prefers_world_scale_when_available(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)

        geometry = processor.build_workspace_s2_rectified_geometry(
            corner_pixels=[[154, 74], [493, 74], [491, 460], [145, 451]],
            corner_world_cabin_frame=[
                [-1465.0, 2877.0, 6356.0],
                [-1473.0, 490.0, 6378.0],
                [1082.0, 519.0, 6336.0],
                [1019.0, 2929.0, 6326.0],
            ],
        )

        self.assertEqual(geometry["rectified_width"], 480)
        self.assertEqual(geometry["rectified_height"], 504)

    def test_pointai_exposes_dedicated_manual_workspace_s2_topics(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")

        self.assertIn("/web/pointAI/run_workspace_s2", pointai_text)
        self.assertIn("/pointAI/manual_workspace_s2_result_raw", pointai_text)
        self.assertIn("/pointAI/manual_workspace_s2_points", pointai_text)
        self.assertIn("/web/pointAI/move_to_workspace_center_scan_pose", pointai_text)

    def test_workspace_center_scan_pose_callback_runs_only_for_true_trigger(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        callback_invocations = []
        processor.run_workspace_center_scan_pose_move = (
            lambda: callback_invocations.append("run") or {"success": True}
        )

        processor.workspace_center_scan_pose_callback(pointAI.Bool(data=False))
        processor.workspace_center_scan_pose_callback(pointAI.Bool(data=True))

        self.assertEqual(callback_invocations, ["run"])

    def test_load_workspace_center_scan_pose_target_returns_fixed_scan_pose(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        target = processor.load_workspace_center_scan_pose_target()

        self.assertEqual(
            target,
            {
                "x": -260.0,
                "y": 1700.0,
                "z": 2997.0,
                "speed": 100.0,
            },
        )

    def test_run_workspace_center_scan_pose_move_calls_single_move_service(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.load_workspace_center_scan_pose_target = lambda: {
            "x": -260.0,
            "y": 1700.0,
            "z": 2997.0,
            "speed": 100.0,
        }

        service_calls = []
        original_wait_for_service = pointAI.rospy.wait_for_service
        original_service_proxy = pointAI.rospy.ServiceProxy
        pointAI.rospy.wait_for_service = lambda name, timeout=None: service_calls.append(
            ("wait", name, timeout)
        )
        pointAI.rospy.ServiceProxy = lambda name, srv_type: (
            lambda request: service_calls.append(
                (
                    "call",
                    name,
                    request.command,
                    request.x,
                    request.y,
                    request.z,
                    request.speed,
                )
            )
            or type("ResponseStub", (), {"success": True, "message": "ok"})()
        )

        try:
            result = processor.run_workspace_center_scan_pose_move()
        finally:
            pointAI.rospy.wait_for_service = original_wait_for_service
            pointAI.rospy.ServiceProxy = original_service_proxy

        self.assertTrue(result["success"])
        self.assertIn(("wait", "/cabin/single_move", 1.0), service_calls)
        self.assertIn(
            ("call", "/cabin/single_move", "单点运动请求", -260.0, 1700.0, 2997.0, 100.0),
            service_calls,
        )

    def test_manual_workspace_s2_callback_runs_only_for_true_trigger(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        callback_invocations = []
        processor.run_manual_workspace_s2 = lambda: callback_invocations.append("run") or {"success": True}

        processor.manual_workspace_s2_callback(pointAI.Bool(data=False))
        processor.manual_workspace_s2_callback(pointAI.Bool(data=True))

        self.assertEqual(callback_invocations, ["run"])

    def test_run_manual_workspace_s2_publishes_result_image_and_points(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.cv2 = pointAI.cv2
        height, width = 120, 140
        base_depth = np.full((height, width), 2000.0, dtype=np.float32)

        for x_value in range(24, 121, 18):
            base_depth[:, max(0, x_value - 1):min(width, x_value + 2)] -= 40.0
        for y_value in range(14, 91, 18):
            base_depth[max(0, y_value - 1):min(height, y_value + 2), :] -= 40.0

        image_raw_world = np.zeros((height, width, 3), dtype=np.float32)
        x_grid = np.tile(np.arange(width, dtype=np.float32), (height, 1))
        y_grid = np.tile(np.arange(height, dtype=np.float32).reshape(-1, 1), (1, width))
        image_raw_world[:, :, 0] = x_grid + 1.0
        image_raw_world[:, :, 1] = y_grid + 1.0
        image_raw_world[:, :, 2] = base_depth

        processor.image_raw_world = image_raw_world
        processor.image_infrared_copy = np.zeros((height, width), dtype=np.uint8)
        processor.load_manual_workspace_quad = lambda: {
            "corner_pixels": [[20, 10], [120, 10], [120, 90], [20, 90]],
            "corner_world_cabin_frame": [
                [20.0, 10.0, 2000.0],
                [120.0, 10.0, 2000.0],
                [120.0, 90.0, 2000.0],
                [20.0, 90.0, 2000.0],
            ],
        }
        processor.apply_spatial_calibration = lambda x_value, y_value, z_value, idx, target_frame="gripper_frame": [
            float(x_value),
            float(y_value),
            float(z_value),
        ]

        published_images = []
        published_points = []
        processor.manual_workspace_s2_result_raw_pub = type(
            "PublisherStub",
            (),
            {"publish": lambda self, message: published_images.append(message)},
        )()
        processor.manual_workspace_s2_points_pub = type(
            "PublisherStub",
            (),
            {"publish": lambda self, message: published_points.append(message)},
        )()

        def stub_cv2_to_imgmsg(image, encoding='mono8'):
            return type(
                "ImageMsgStub",
                (),
                {"image": image, "encoding": encoding, "header": type("HeaderStub", (), {})()},
            )()

        processor.bridge = type(
            "BridgeStub",
            (),
            {"cv2_to_imgmsg": staticmethod(stub_cv2_to_imgmsg)},
        )()
        original_time_now = pointAI.rospy.Time.now
        pointAI.rospy.Time.now = staticmethod(lambda: 0)

        try:
            result = processor.run_manual_workspace_s2()
        finally:
            pointAI.rospy.Time.now = original_time_now

        self.assertTrue(result["success"])
        self.assertGreater(result["point_count"], 0)
        self.assertEqual(len(published_images), 1)
        self.assertEqual(len(published_points), 1)
        self.assertGreater(published_points[0].count, 0)

    def test_run_manual_workspace_s2_pipeline_can_skip_publication(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.prepare_manual_workspace_s2_inputs = lambda: {
            "manual_workspace": {"corner_pixels": [[10, 10], [30, 10], [30, 30], [10, 30]]},
            "workspace_mask": np.ones((40, 40), dtype=np.uint8),
            "workspace_bbox": (10, 10, 30, 30),
            "rectified_geometry": {"rectified_width": 20, "rectified_height": 20, "inverse_h": np.eye(3, dtype=np.float32)},
            "vertical_estimate": {"period": 10, "phase": 5},
            "horizontal_estimate": {"period": 10, "phase": 5},
        }
        processor.build_workspace_s2_line_positions = lambda start_pixel, end_pixel, period_px, phase_px: [5, 15]
        processor.map_workspace_s2_rectified_points_to_image = lambda points, inverse_h: points
        processor.build_workspace_s2_projective_line_segments = lambda *args, **kwargs: {"vertical": [], "horizontal": []}
        expected_points = make_points_array_with_world_coords([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
        processor.build_manual_workspace_s2_points_array = lambda intersection_pixels, workspace_mask: (
            expected_points,
            [(1, [5, 5], [1.0, 2.0, 3.0], "selected", "S2")],
        )
        processor.render_manual_workspace_s2_result_image = lambda *args, **kwargs: np.zeros((40, 40), dtype=np.uint8)
        publish_invocations = []
        processor.publish_manual_workspace_s2_result = lambda result_image, points_array_msg: publish_invocations.append(
            (result_image, points_array_msg)
        )

        result = processor.run_manual_workspace_s2_pipeline(publish=False)

        self.assertTrue(result["success"])
        self.assertEqual(result["point_coords"].count, 2)
        self.assertEqual(len(publish_invocations), 0)

    def test_scan_only_prefers_manual_workspace_s2_points_when_saved_quad_exists(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.image = object()
        processor.image_raw_world = np.zeros((2, 2, 3), dtype=np.float32)
        processor.world_image_seq = 1
        processor.process_wait_timeout_sec = 0.1
        processor.process_request_rate_hz = 1.0
        processor.stable_frame_count = 3
        processor.stable_z_tolerance_mm = 5.0
        processor.pre_img = lambda request_mode=None: make_points_array_with_world_coords([(9.0, 9.0, 9.0)])
        processor.has_detected_points = lambda point_coords: bool(
            point_coords and getattr(point_coords, "count", 0) > 0
        )
        processor.load_manual_workspace_quad = lambda: {
            "corner_pixels": [[10, 10], [20, 10], [20, 20], [10, 20]],
            "corner_world_cabin_frame": [
                [100.0, 100.0, 300.0],
                [300.0, 100.0, 300.0],
                [300.0, 300.0, 300.0],
                [100.0, 300.0, 300.0],
            ],
        }
        manual_s2_points = make_points_array_with_world_coords(
            [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]
        )
        processor.run_manual_workspace_s2_pipeline = lambda publish=False: {
            "success": True,
            "message": "manual workspace S2 finished",
            "point_count": 3,
            "point_coords": manual_s2_points,
            "result_image": np.zeros((4, 4), dtype=np.uint8),
        }
        original_rate = pointAI.rospy.Rate
        pointAI.rospy.Rate = lambda hz: type("RateStub", (), {"sleep": lambda self: None})()
        try:
            result = processor.wait_for_stable_point_coords(pointAI.PROCESS_IMAGE_MODE_SCAN_ONLY)
        finally:
            pointAI.rospy.Rate = original_rate

        self.assertTrue(result["success"])
        self.assertEqual(result["point_coords"].count, 3)
        self.assertIn("手动工作区S2", result["message"])

    def test_try_scan_only_manual_workspace_s2_publishes_result_overlay(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.load_manual_workspace_quad = lambda: {
            "corner_pixels": [[10, 10], [20, 10], [20, 20], [10, 20]],
            "corner_world_cabin_frame": [
                [100.0, 100.0, 300.0],
                [300.0, 100.0, 300.0],
                [300.0, 300.0, 300.0],
                [100.0, 300.0, 300.0],
            ],
        }
        captured_publish_flag = []
        manual_s2_points = make_points_array_with_world_coords([(1.0, 2.0, 3.0)])
        processor.run_manual_workspace_s2_pipeline = lambda publish=False: (
            captured_publish_flag.append(publish)
            or {
                "success": True,
                "message": "manual workspace S2 finished",
                "point_count": 1,
                "point_coords": manual_s2_points,
                "result_image": np.zeros((2, 2), dtype=np.uint8),
            }
        )

        result = processor.try_scan_only_manual_workspace_s2()

        self.assertTrue(result["success"])
        self.assertEqual(result["point_coords"].count, 1)
        self.assertEqual(captured_publish_flag, [True])

    def test_manual_workspace_s2_result_labels_only_show_point_indices(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.image_infrared_copy = np.zeros((40, 40), dtype=np.uint8)
        processor.load_manual_workspace_quad = lambda: {
            "corner_pixels": [[5, 5], [35, 5], [35, 35], [5, 35]],
            "corner_world_cabin_frame": [
                [5.0, 5.0, 2000.0],
                [35.0, 5.0, 2000.0],
                [35.0, 35.0, 2000.0],
                [5.0, 35.0, 2000.0],
            ],
        }

        captured_labels = []
        processor.find_non_overlapping_label_position = lambda image_shape, anchor_point, text, occupied_bboxes: (
            (int(anchor_point[0]), int(anchor_point[1])),
            (0, 0, 1, 1),
        )
        processor.draw_text_with_background = lambda image, text, position, **kwargs: captured_labels.append(text)

        processor.render_manual_workspace_s2_result_image(
            workspace_mask=np.ones((40, 40), dtype=np.uint8),
            line_segments={
                "vertical": [([10, 5], [10, 35]), ([20, 5], [20, 35])],
                "horizontal": [([5, 12], [35, 12]), ([5, 24], [35, 24])],
            },
            display_points=[
                (1, [10, 12], [100.0, 200.0, 300.0], "selected", "S2"),
                (2, [20, 24], [110.0, 210.0, 310.0], "selected", "S2"),
            ],
        )

        self.assertEqual(captured_labels, ["1", "2"])

    def test_roi_pixel_mask_uses_white_rectangle_bounds(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.x_channel = np.zeros((4, 4), dtype=np.int32)
        processor.point1 = (1, 1)
        processor.point2 = (2, 3)

        mask = processor.get_roi_pixel_mask()

        expected_mask = np.array(
            [
                [0, 0, 0, 0],
                [0, 1, 1, 0],
                [0, 1, 1, 0],
                [0, 1, 1, 0],
            ],
            dtype=np.uint8,
        )
        np.testing.assert_array_equal(mask, expected_mask)

    def test_travel_range_pixel_mask_clips_white_roi_to_scan_workspace(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.x_channel = np.zeros((4, 4), dtype=np.int32)
        processor.point1 = (0, 0)
        processor.point2 = (3, 3)
        processor.get_scan_workspace_pixel_mask = lambda: np.array(
            [
                [0, 1, 1, 0],
                [0, 1, 1, 0],
                [0, 1, 1, 0],
                [0, 0, 0, 0],
            ],
            dtype=np.uint8,
        )

        mask = processor.get_travel_range_pixel_mask()

        expected_mask = np.array(
            [
                [0, 1, 1, 0],
                [0, 1, 1, 0],
                [0, 1, 1, 0],
                [0, 0, 0, 0],
            ],
            dtype=np.uint8,
        )
        np.testing.assert_array_equal(mask, expected_mask)

    def test_draw_scan_workspace_overlay_draws_workspace_contour_on_result_layer(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.get_scan_workspace_pixel_mask = lambda: np.array(
            [
                [0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 1, 1, 1, 0, 0],
                [0, 1, 1, 1, 0, 0],
                [0, 1, 1, 1, 1, 0],
                [0, 0, 0, 0, 0, 0],
            ],
            dtype=np.uint8,
        )
        result_image = np.zeros((6, 6), dtype=np.uint8)

        processor.draw_scan_workspace_overlay(result_image)

        self.assertGreater(int(result_image[1, 2]), 0)
        self.assertGreater(int(result_image[4, 4]), 0)

    def test_result_image_draws_scan_workspace_and_travel_range_overlays(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")

        self.assertIn("def lookup_transform_matrix_mm(self, target_frame, source_frame=\"Scepter_depth_frame\"):", pointai_text)
        self.assertIn("def get_frame_space_pixel_mask(self, target_frame, min_x, max_x, min_y, max_y):", pointai_text)
        self.assertIn("def get_roi_pixel_mask(self):", pointai_text)
        self.assertIn("def get_scan_workspace_pixel_mask(self):", pointai_text)
        self.assertIn("def get_travel_range_pixel_mask(self):", pointai_text)
        self.assertIn("def draw_scan_workspace_overlay(self, result_image):", pointai_text)
        self.assertIn("def draw_travel_range_overlay(self, result_image):", pointai_text)
        self.assertIn("cv2.findContours(", pointai_text)
        self.assertIn("cv2.drawContours(", pointai_text)
        self.assertIn("self.draw_scan_workspace_overlay(result_image)", pointai_text)
        self.assertIn("self.draw_travel_range_overlay(result_image)", pointai_text)

    def test_remove_small_foreground_components_drops_isolated_binary_blobs(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        binary_image = np.zeros((8, 8), dtype=np.uint8)
        binary_image[1:6, 3] = 255
        binary_image[5, 3:6] = 255
        binary_image[6, 3] = 255
        binary_image[2:4, 6:8] = 255

        filtered = processor.remove_small_foreground_components(binary_image, min_area_px=6)

        self.assertEqual(int(filtered[2, 6]), 0)
        self.assertEqual(int(filtered[3, 7]), 0)
        self.assertEqual(int(filtered[1, 3]), 255)
        self.assertEqual(int(filtered[6, 3]), 255)

    def test_binary_pipeline_filters_small_connected_components_before_thinning(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = pointai_text

        self.assertIn("self.binary_small_blob_min_area_px = int(rospy.get_param(", pointai_text)
        self.assertIn("def remove_small_foreground_components(self, binary_image, min_area_px=None):", pointai_text)
        self.assertIn(
            "self.Depth_image_Raw_binary = self.remove_small_foreground_components(",
            pointai_text,
        )
        self.assertIn("cv2.connectedComponentsWithStats(", pointai_text)
        self.assertIn("self.binary_small_blob_min_area_px = int(rospy.get_param(", vision_text)
        self.assertIn("def remove_small_foreground_components(self, binary_image, min_area_px=None):", vision_text)
        self.assertIn(
            "self.Depth_image_Raw_binary = self.remove_small_foreground_components(",
            vision_text,
        )

    def test_pseudo_slam_filters_close_xy_clusters_after_full_scan(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn("constexpr float kPseudoSlamClosePointClusterXYToleranceMm = 100.0f;", suoqu_text)
        self.assertIn("filter_pseudo_slam_close_xy_point_clusters", suoqu_text)
        self.assertIn("rejected_indexes.insert(candidate_index);", suoqu_text)
        self.assertIn("rejected_indexes.insert(other_index);", suoqu_text)
        self.assertIn("std::fabs(candidate_point.World_coord[0] - other_point.World_coord[0]) <", suoqu_text)
        self.assertIn("std::fabs(candidate_point.World_coord[1] - other_point.World_coord[1]) <", suoqu_text)
        self.assertIn("kPseudoSlamClosePointClusterXYToleranceMm &&", suoqu_text)
        self.assertIn("kPseudoSlamClosePointClusterXYToleranceMm) {", suoqu_text)
        self.assertNotIn("dx * dx + dy * dy < min_distance_sq", suoqu_text)
        self.assertNotIn("merged_world_points = filter_pseudo_slam_close_xy_point_clusters(merged_world_points);", scan_function)
        self.assertNotIn("frame_world_points = dedupe_world_points(frame_world_points);", scan_function)
        self.assertNotIn("area_world_points = dedupe_world_points(area_world_points);", scan_function)
        self.assertNotIn("merged_world_points = dedupe_world_points(merged_world_points);", scan_function)
        self.assertNotIn("filter_pseudo_slam_close_xy_points_keep_highest_z", scan_function)

    def test_multi_pose_scan_clusters_overlap_points_during_scan_and_reassigns_representatives(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )
        multi_pose_branch = scan_function.split("case PseudoSlamScanStrategy::kMultiPose:", 1)[1]

        self.assertIn("constexpr float kPseudoSlamScanDuplicateXYToleranceMm = 10.0f;", suoqu_text)
        self.assertIn("struct PseudoSlamOverlapCluster", suoqu_text)
        self.assertIn("std::vector<chassis_ctrl::PointCoords> member_points;", suoqu_text)
        self.assertIn("build_scan_cluster_representatives(", suoqu_text)
        self.assertIn("merge_frame_points_into_overlap_clusters(", suoqu_text)
        self.assertIn("const size_t cluster_count_before_frame = clusters.size();", suoqu_text)
        self.assertIn("for (size_t cluster_index = 0; cluster_index < cluster_count_before_frame; ++cluster_index)", suoqu_text)
        self.assertIn("const double cluster_center_x =", suoqu_text)
        self.assertIn("const double cluster_center_y =", suoqu_text)
        self.assertIn("const double tolerance_sq = static_cast<double>(tolerance_mm) * static_cast<double>(tolerance_mm);", suoqu_text)
        self.assertIn("dx * dx + dy * dy <= tolerance_sq", suoqu_text)
        self.assertIn("best_member_distance_sq", suoqu_text)
        self.assertIn("merged_world_points = build_scan_cluster_representatives(scan_clusters);", scan_function)
        self.assertIn("kPseudoSlamScanDuplicateXYToleranceMm", scan_function)
        self.assertIn("assign_global_indices(merged_world_points);", scan_function)
        self.assertNotIn("filter_new_scan_points_against_existing_xy_tolerance(", multi_pose_branch)

    def test_scan_duplicate_filter_uses_circular_euclidean_distance(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("dx * dx + dy * dy <= tolerance_sq", suoqu_text)
        self.assertNotIn(
            "std::fabs(candidate_point.World_coord[0] - existing_point.World_coord[0]) <= tolerance_mm &&\n                std::fabs(candidate_point.World_coord[1] - existing_point.World_coord[1]) <= tolerance_mm",
            suoqu_text,
        )

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
        vision_text = pointai_text

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
        vision_text = pointai_text

        self.assertIn("def axis_angle_diff_deg", pointai_text)
        self.assertIn("def axis_angle_diff_deg", vision_text)
        self.assertIn("def is_near_axis_aligned_line", pointai_text)
        self.assertIn("def is_near_axis_aligned_line", vision_text)
        self.assertNotIn("angle_diff <= 60", pointai_text)
        self.assertNotIn("angle_diff <= 60", vision_text)

    def test_visual_travel_range_caps_at_three_hundred_sixty_by_three_hundred_twenty(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)

        self.assertTrue(processor.is_point_in_travel_range(360, 320))
        self.assertFalse(processor.is_point_in_travel_range(361, 320))
        self.assertFalse(processor.is_point_in_travel_range(360, 321))

    def test_visual_scripts_use_three_hundred_sixty_by_three_hundred_twenty_as_tcp_xy_limits(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = pointai_text

        self.assertIn('travel_range_max_x_mm", 360.0', pointai_text)
        self.assertIn('travel_range_max_x_mm", 360.0', vision_text)
        self.assertIn('travel_range_max_y_mm", 320.0', pointai_text)
        self.assertIn('travel_range_max_y_mm", 320.0', vision_text)
        self.assertNotIn('travel_range_max_x_mm", 320.0', pointai_text)
        self.assertNotIn('travel_range_max_x_mm", 320.0', vision_text)
        self.assertNotIn('travel_range_max_y_mm", 360.0', pointai_text)
        self.assertNotIn('travel_range_max_y_mm", 360.0', vision_text)

    def test_moduan_uses_three_hundred_sixty_by_three_hundred_twenty_as_tcp_xy_limits(self):
        moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")

        self.assertIn("kTravelMaxXMm = 360.0", moduan_text)
        self.assertIn("kTravelMaxYMm = 320.0", moduan_text)
        self.assertIn("x < 0 || x > kTravelMaxXMm", moduan_text)
        self.assertIn("y < 0 || y > kTravelMaxYMm", moduan_text)
        self.assertNotIn("x < 0 || x > 320", moduan_text)
        self.assertNotIn("y < 0 || y > 360", moduan_text)

    def test_pointai_defaults_travel_range_to_three_hundred_sixty_by_three_hundred_twenty_by_one_forty(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        self.assertIn('self.travel_range_max_x_mm = float(rospy.get_param("~travel_range_max_x_mm", 360.0))', pointai_text)
        self.assertIn('self.travel_range_max_y_mm = float(rospy.get_param("~travel_range_max_y_mm", 320.0))', pointai_text)
        self.assertIn('self.travel_range_max_z_mm = float(rospy.get_param("~travel_range_max_z_mm", 140.0))', pointai_text)

    def test_suoqu_uses_three_hundred_sixty_by_three_hundred_twenty_by_one_forty_as_local_tcp_limits(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        self.assertIn("constexpr float kTravelMaxXMm = 360.0f;", suoqu_text)
        self.assertIn("constexpr float kTravelMaxYMm = 320.0f;", suoqu_text)
        self.assertIn("constexpr float kTravelMaxZMm = 140.0f;", suoqu_text)

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
        vision_text = pointai_text

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
        vision_text = pointai_text

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
        vision_text = pointai_text

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

    def test_calibration_offset_callback_keeps_tf_yaml_unchanged_and_defers_to_broadcaster(self):
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
                {"x": 1.0, "y": 2.0, "z": 3.0},
            )

    def test_lashing_config_only_keeps_height_threshold(self):
        config_text = (CHASSIS_CTRL_DIR / "data" / "lashing_config.json").read_text(
            encoding="utf-8"
        )
        self.assertIn('"height_threshold"', config_text)
        self.assertNotIn('"cal_x"', config_text)
        self.assertNotIn('"cal_y"', config_text)
        self.assertNotIn('"cal_z"', config_text)

    def test_visual_scripts_keep_legacy_offset_topic_but_defer_live_tf_updates_to_broadcaster(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = pointai_text
        broadcaster_text = (CHASSIS_CTRL_DIR / "scripts" / "gripper_tf_broadcaster.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("/web/pointAI/set_offset", pointai_text)
        self.assertIn("/web/pointAI/set_offset", vision_text)
        self.assertIn("由gripper_tf_broadcaster实时处理", pointai_text)
        self.assertIn("由gripper_tf_broadcaster实时处理", vision_text)
        self.assertNotIn("请重启pointai_tf_verify.launch后验证", pointai_text)
        self.assertNotIn("请重启pointai_tf_verify.launch后验证", vision_text)
        self.assertIn("POINTAI_OFFSET_TOPIC", broadcaster_text)

    def test_debug_and_transfer_logs_reference_tf_translation_calibration(self):
        debug_button_text = (CHASSIS_CTRL_DIR / "scripts" / "debug_button_node.py").read_text(
            encoding="utf-8"
        )
        topics_transfer_text = (CHASSIS_CTRL_DIR / "src" / "topics_transfer.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("设置TF平移标定", debug_button_text)
        self.assertIn("gripper_tf.yaml.translation_mm", topics_transfer_text)

    def test_pointai_supports_scan_mode_for_pseudo_slam(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")

        self.assertIn("PROCESS_IMAGE_MODE_SCAN_ONLY", pointai_text)
        self.assertIn("pseudo_slam", pointai_text)
        self.assertIn("scan_only", pointai_text)
        self.assertIn("cabin_frame", pointai_text)
        self.assertIn("display_centers = list(in_range_centers)", pointai_text)

    def test_pointai_cabin_frame_transform_uses_global_upward_z_mapping(self):
        cabin_transform_text = inspect.getsource(pointAI.ImageProcessor.transform_to_cabin_frame)

        self.assertIn('self.lookup_transform_matrix_mm("cabin_frame", source_frame="Scepter_depth_frame")', cabin_transform_text)
        self.assertIn('self.lookup_transform_matrix_mm("gripper_frame", source_frame="Scepter_depth_frame")', cabin_transform_text)
        self.assertIn("float(T[0, 3]) + float(x)", cabin_transform_text)
        self.assertIn("float(T[1, 3]) + float(y)", cabin_transform_text)
        self.assertIn("float(T[2, 3]) - float(z) - float(T_gripper[2, 3])", cabin_transform_text)
        self.assertIn("z 轴大地朝上", cabin_transform_text)

    def test_scan_and_execution_refine_result_overlays_use_explicit_mode_masks(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        draw_overlay_text = inspect.getsource(pointAI.ImageProcessor.draw_scan_workspace_overlay)

        self.assertIn("def get_manual_workspace_cabin_polygon_pixel_mask(self):", pointai_text)
        self.assertIn("def build_convex_polygon_inside_mask(", pointai_text)
        self.assertIn("self.current_result_request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY", draw_overlay_text)
        self.assertIn("workspace_mask = self.get_scan_workspace_pixel_mask()", draw_overlay_text)
        self.assertIn("self.get_manual_workspace_cabin_polygon_pixel_mask()", draw_overlay_text)

    def test_scan_and_execution_refine_result_labels_use_explicit_coordinate_formats(self):
        label_text = inspect.getsource(pointAI.ImageProcessor.format_result_display_label)

        self.assertIn("self.current_result_request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY", label_text)
        self.assertIn('return f"{display_idx}, {world_coord}, {status_text}"', label_text)
        self.assertIn('return f"{display_idx}, tcp=({tcp_x:.1f},{tcp_y:.1f},{tcp_z:.1f}), {status_text}"', label_text)

    def test_execution_refine_display_filters_to_tcp_travel_volume(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.current_result_request_mode = pointAI.PROCESS_IMAGE_MODE_EXECUTION_REFINE
        processor.travel_range_max_x_mm = 360.0
        processor.travel_range_max_y_mm = 320.0
        processor.travel_range_max_z_mm = 140.0
        processor.get_selected_point_numbers = lambda matrix_centers: {1: 1, 2: 2}
        processor.get_selected_point_status = lambda display_idx: "selected"
        processor.transform_cabin_point_to_gripper_frame = lambda x, y, z: [100.0, 120.0, 80.0] if x < 50.0 else [400.0, 120.0, 80.0]

        display_points = processor.build_matrix_display_points([
            (1, [10, 20], [10.0, 20.0, 30.0]),
            (2, [30, 40], [60.0, 20.0, 30.0]),
        ])

        self.assertEqual(len(display_points), 1)
        self.assertEqual(display_points[0][0], 1)
        self.assertEqual(display_points[0][2], [100.0, 120.0, 80.0])

    def test_execution_refine_returns_without_manual_workspace_s2_override(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.current_result_request_mode = pointAI.PROCESS_IMAGE_MODE_EXECUTION_REFINE
        processor.process_wait_timeout_sec = 0.0
        processor.process_request_rate_hz = 10.0
        processor.stable_frame_count = 3
        processor.stable_z_tolerance_mm = 5.0
        processor.image = object()
        processor.image_raw_world = object()
        processor.world_image_seq = 1
        processor.has_detected_points = lambda point_coords: True
        point_coords = object()
        processor.pre_img = lambda request_mode=None: point_coords
        processor.evaluate_point_coords_for_mode = lambda latest_point_coords, request_mode: {
            "success": True,
            "message": "ok",
            "point_coords": latest_point_coords,
        }
        processor.try_scan_only_manual_workspace_s2 = lambda: self.fail("manual workspace S2 should be disabled during execution display")
        original_rate = pointAI.rospy.Rate
        original_is_shutdown = pointAI.rospy.is_shutdown
        pointAI.rospy.Rate = lambda hz: type("FakeRate", (), {"sleep": staticmethod(lambda: None)})()
        pointAI.rospy.is_shutdown = lambda: False

        try:
            result = processor.wait_for_stable_point_coords(pointAI.PROCESS_IMAGE_MODE_EXECUTION_REFINE)
        finally:
            pointAI.rospy.Rate = original_rate
            pointAI.rospy.is_shutdown = original_is_shutdown

        self.assertTrue(result["success"])
        self.assertIs(result["point_coords"], point_coords)

    def test_suoqu_live_visual_uses_execution_refine_mode(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr uint8_t kProcessImageModeExecutionRefine = 4;", suoqu_text)
        self.assertIn("scan_srv.request.request_mode = kProcessImageModeExecutionRefine;", suoqu_text)
        self.assertNotIn("scan_only_execution_display_enabled", suoqu_text)
        self.assertNotIn("/pointAI/scan_only_execution_display", suoqu_text)

    def test_execution_refine_mode_does_not_depend_on_scan_only_toggle(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")

        self.assertIn("PROCESS_IMAGE_MODE_EXECUTION_REFINE = 4", pointai_text)
        self.assertIn('return "execution_refine"', pointai_text)
        self.assertNotIn("should_use_tcp_display_for_scan_only", pointai_text)
        self.assertNotIn("scan_only_execution_display_enabled", pointai_text)
        self.assertNotIn("/pointAI/scan_only_execution_display", pointai_text)
        self.assertIn("if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:", pointai_text)

    def test_suoqu_supports_cabin_frame_and_pseudo_slam_execution(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("cabin_frame", suoqu_text)
        self.assertIn("Scepter_depth_frame", suoqu_text)
        self.assertIn("pseudo_slam_points.json", suoqu_text)
        self.assertIn("pseudo_slam_bind_path.json", suoqu_text)
        self.assertIn("scan_only", suoqu_text)
        self.assertIn("bind_from_scan", suoqu_text)
        self.assertIn("跳过该点", suoqu_text)

    def test_suoqu_uses_global_upward_z_when_mapping_between_cabin_and_local_bind_frames(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        transform_text = extract_cpp_block(
            suoqu_text,
            "bool transform_cabin_world_point_to_planned_gripper_point(",
        )
        candidate_pose_text = extract_cpp_block(
            suoqu_text,
            "DynamicBindPlanningCandidatePose build_dynamic_bind_candidate_pose_from_world_point(",
        )

        self.assertIn("static_cast<double>(cabin_height - world_point.World_coord[2]) / 1000.0 -", transform_text)
        self.assertIn("gripper_from_scepter.getOrigin().z()", transform_text)
        self.assertIn("average_world_z +", candidate_pose_text)
        self.assertIn("desired_point_in_scepter_frame.z() * 1000.0 +", candidate_pose_text)
        self.assertIn("gripper_from_scepter.getOrigin().z() * 1000.0", candidate_pose_text)

    def test_pseudo_slam_bind_path_groups_points_after_scan(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("build_bind_groups_from_scan_world", suoqu_text)
        self.assertIn('area_json["groups"]', suoqu_text)
        self.assertIn("remaining_world_points", suoqu_text)
        self.assertIn("group_index", suoqu_text)
        self.assertIn("跳过该组继续下一组", suoqu_text)
        self.assertIn('"global_idx"', suoqu_text)
        self.assertIn('"local_idx"', suoqu_text)
        self.assertIn('point_json.value("local_idx"', suoqu_text)
        self.assertIn('if (areas_json.empty())', suoqu_text)
        self.assertIn("pseudo_slam_bind_path.json没有可执行区域", suoqu_text)

    def test_pseudo_slam_bind_path_supports_edge_pair_groups(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("edge_pair", suoqu_text)
        self.assertIn("group_type", suoqu_text)
        self.assertIn("selected_indices.size() == 2", suoqu_text)

    def test_pseudo_slam_checkerboard_membership_no_longer_requires_both_axes(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("can_form_edge_pair", suoqu_text)
        self.assertNotIn("has_horizontal_neighbor && has_vertical_neighbor", suoqu_text)

    def test_suoqu_persists_bind_execution_memory_to_disk(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bind_execution_memory.json", suoqu_text)
        self.assertIn("write_bind_execution_memory_json", suoqu_text)
        self.assertIn("load_bind_execution_memory_json", suoqu_text)

    def test_bind_execution_memory_write_is_atomic_and_read_failures_are_logged(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn('const std::string temp_json_path = kBindExecutionMemoryJsonPath + ".tmp";', suoqu_text)
        self.assertIn("std::rename(temp_json_path.c_str(), kBindExecutionMemoryJsonPath.c_str())", suoqu_text)
        self.assertIn("bind_execution_memory.json读取或解析失败", suoqu_text)

    def test_rescan_rebuilds_execution_memory_after_scan_outputs_are_written(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("reset_bind_execution_memory_for_scan_session", suoqu_text)
        self.assertIn("write_pseudo_slam_points_json", suoqu_text)
        self.assertIn("write_pseudo_slam_bind_path_json", suoqu_text)

    def test_start_work_supports_precomputed_and_live_visual_execution_modes(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        topics_text = (CHASSIS_CTRL_DIR / "src" / "topics_transfer.cpp").read_text(encoding="utf-8")
        start_work_text = extract_cpp_block(
            suoqu_text,
            "bool startGlobalWork(chassis_ctrl::MotionControl::Request &req,",
        )

        self.assertIn("enum class GlobalExecutionMode", suoqu_text)
        self.assertIn("kSlamPrecomputed", suoqu_text)
        self.assertIn("kLiveVisual", suoqu_text)
        self.assertIn(
            "std::atomic<int> global_execution_mode{static_cast<int>(GlobalExecutionMode::kLiveVisual)};",
            suoqu_text,
        )
        self.assertIn("run_bind_from_scan(res.message)", suoqu_text)
        self.assertIn("run_live_visual_global_work(res.message)", suoqu_text)
        self.assertIn('sg_live_visual_client = nh.serviceClient<std_srvs::Trigger>("/moduan/sg")', suoqu_text)
        self.assertIn("/web/cabin/set_execution_mode", suoqu_text)
        self.assertIn("当前全局执行模式为slam_precomputed", suoqu_text)
        self.assertIn(
            "开始执行层检测到pseudo_slam_bind_path.json，优先按预生成路径执行",
            start_work_text,
        )
        self.assertLess(
            start_work_text.index("if (scan_file.good()) {"),
            start_work_text.index("if (execution_mode == GlobalExecutionMode::kLiveVisual) {"),
        )
        self.assertIn("/web/cabin/set_execution_mode", topics_text)
        self.assertIn("live_visual", topics_text)
        self.assertIn("slam_precomputed", topics_text)

    def test_pseudo_slam_bind_path_records_checkerboard_grid_metadata(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("build_checkerboard_info_by_global_index", suoqu_text)
        self.assertIn('"global_row"', suoqu_text)
        self.assertIn('"global_col"', suoqu_text)
        self.assertIn('"checkerboard_parity"', suoqu_text)
        self.assertIn('"is_checkerboard_member"', suoqu_text)

    def test_pseudo_slam_checkerboard_and_bind_execution_use_path_origin_anchor(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn('"path_origin"', suoqu_text)
        self.assertIn("const Cabin_Point& path_origin", suoqu_text)
        self.assertIn("world_point.World_coord[0] - path_origin.x", suoqu_text)
        self.assertIn("world_point.World_coord[1] - path_origin.y", suoqu_text)
        self.assertIn('bind_path_json["path_origin"]', suoqu_text)
        self.assertIn('bind_path_json["path_origin"]["x"]', suoqu_text)
        self.assertIn("bind_from_scan先回到规划原点", suoqu_text)
        self.assertIn("live_visual先回到规划原点", suoqu_text)

    def test_precomputed_execution_filters_by_checkerboard_and_execution_memory(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("filter_precomputed_group_points_for_execution", suoqu_text)
        self.assertIn("is_point_already_executed", suoqu_text)
        self.assertIn("record_successful_execution_point", suoqu_text)
        self.assertIn('group_json.value("group_type"', suoqu_text)

    def test_slam_precomputed_skips_points_already_recorded_in_execution_memory(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("is_point_already_executed", suoqu_text)
        self.assertIn("record_successful_execution_point", suoqu_text)
        self.assertIn('point_json.value("global_row"', suoqu_text)

    def test_slam_precomputed_records_only_dispatched_points_after_prepare_filtering(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("collect_dispatched_precomputed_point_jsons", suoqu_text)
        self.assertIn("const auto dispatched_point_jsons = collect_dispatched_precomputed_point_jsons(", suoqu_text)
        self.assertIn("for (const auto& dispatched_point_json : dispatched_point_jsons)", suoqu_text)
        self.assertNotIn('for (const auto& point_json : execution_group_json["points"])', suoqu_text)

    def test_live_visual_reprojects_points_into_global_checkerboard_before_execution(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("classify_live_visual_point_into_checkerboard", suoqu_text)
        self.assertIn("load_bind_execution_memory_json", suoqu_text)
        self.assertIn("未能归入全局棋盘格", suoqu_text)
        self.assertIn("source_mode", suoqu_text)

    def test_live_visual_uses_scan_grid_to_classify_points_before_execution(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("run_live_visual_global_work", suoqu_text)
        self.assertIn("scan_srv.request.request_mode = kProcessImageModeExecutionRefine", suoqu_text)
        self.assertIn("classify_live_visual_point_into_checkerboard", suoqu_text)
        self.assertIn("load_bind_execution_memory_json", suoqu_text)
        self.assertIn("sg_precomputed_fast_client.call", suoqu_text)

    def test_live_visual_skips_points_that_cannot_be_projected_into_global_grid(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("未能归入全局棋盘格", suoqu_text)
        self.assertIn("continue;", suoqu_text)

    def test_live_visual_sparse_checkerboard_indices_do_not_require_dense_center_vectors(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("row_centers_by_global_row", suoqu_text)
        self.assertIn("col_centers_by_global_col", suoqu_text)
        self.assertIn("find_nearest_checkerboard_center_key", suoqu_text)
        self.assertNotIn("pseudo_slam_points.json中的全局行中心不完整", suoqu_text)
        self.assertNotIn("pseudo_slam_points.json中的全局列中心不完整", suoqu_text)

    def test_live_visual_uses_scanned_area_points_as_coarse_reference(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        live_visual_section = extract_cpp_block(
            suoqu_text,
            "bool run_live_visual_global_work(std::string& message)",
        )

        self.assertIn("find_planned_bind_area_json_by_area_index", suoqu_text)
        self.assertIn("collect_planned_area_world_points_by_global_index", suoqu_text)
        self.assertIn("build_live_visual_execution_points_from_planned_area", suoqu_text)
        self.assertIn("const auto& areas_json = bind_path_json[\"areas\"];", live_visual_section)
        self.assertIn("collect_planned_area_world_points_by_global_index(", live_visual_section)
        self.assertIn("build_live_visual_execution_points_from_planned_area(", live_visual_section)
        self.assertIn("assign_planned_gripper_coords_to_bind_point_json(", suoqu_text)

    def test_live_visual_skips_points_outside_current_scanned_area_reference(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr float kLiveVisualMicroAdjustXYToleranceMm = 30.0f;", suoqu_text)
        self.assertIn("constexpr float kLiveVisualMicroAdjustZToleranceMm = 6.0f;", suoqu_text)
        self.assertIn("不在当前区域扫描参考点中", suoqu_text)
        self.assertIn("超出xyz微调范围", suoqu_text)
        self.assertIn("保留扫描参考点", suoqu_text)
        self.assertNotIn("conflicting_live_global_indices", suoqu_text)
        self.assertIn('{"global_idx", checkerboard_it->second.global_idx}', suoqu_text)
        self.assertNotIn("constexpr float kLiveVisualPlannedPointRefineMaxDistanceMm", suoqu_text)
        self.assertNotIn("constexpr float kLiveVisualPlannedPointRefineMaxZDeltaMm = 5.0f;", suoqu_text)

    def test_pseudo_slam_scan_moves_once_to_global_workspace_center_and_requests_single_frame(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn("constexpr float kPseudoSlamGlobalScanHeightOffsetMm = 1500.0f;", suoqu_text)
        self.assertIn("bool compute_pseudo_slam_global_scan_pose(", suoqu_text)
        self.assertIn('const float marking_x = path_json.value("marking_x", 0.0f);', suoqu_text)
        self.assertIn('const float zone_x = path_json.value("zone_x", 0.0f);', suoqu_text)
        self.assertIn("const float workspace_min_x = marking_x - robot_x_step / 2.0f;", suoqu_text)
        self.assertIn("scan_center.x = workspace_min_x + zone_x / 2.0f;", suoqu_text)
        self.assertIn("scan_height = planning_cabin_height + kPseudoSlamGlobalScanHeightOffsetMm;", suoqu_text)
        self.assertIn("total_scan_area_count = 1;", scan_function)
        self.assertIn("TCP_Move[1] = global_scan_center.x;", scan_function)
        self.assertIn("TCP_Move[2] = global_scan_center.y;", scan_function)
        self.assertIn("TCP_Move[3] = global_scan_height;", scan_function)
        self.assertIn("pseudo_slam全局中心扫描移动到", scan_function)
        self.assertIn("pseudo_slam全局中心单帧识别完成", scan_function)
        self.assertNotIn("collected_scan_frame_count", scan_function)
        self.assertNotIn("area_world_points", scan_function)
        self.assertNotIn("ros::Duration(config.scan_retry_interval_sec).sleep();", scan_function)
        self.assertIn("case PseudoSlamScanStrategy::kSingleCenter:", scan_function)

    def test_pseudo_slam_multi_pose_scan_keeps_path_loop_and_clusters_only_cross_pose_overlap(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn("case PseudoSlamScanStrategy::kMultiPose:", scan_function)
        self.assertIn("total_scan_area_count = static_cast<int>(con_path.size());", scan_function)
        self.assertIn("for (const auto& cabin_point : con_path)", scan_function)
        self.assertIn("publish_area_progress(area_index, total_scan_area_count, 0, false, false);", scan_function)
        self.assertIn("merge_frame_points_into_overlap_clusters(", scan_function)
        self.assertIn("scan_clusters", scan_function)
        self.assertIn("scan_pose_index", scan_function)

    def test_pseudo_slam_scan_uses_final_capture_gate_before_requesting_visual(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn('const std::string kPseudoSlamCaptureGateImageTopic = "/Scepter/ir/image_raw";', suoqu_text)
        self.assertIn("constexpr int kPseudoSlamCaptureGateStableSampleCount = 3;", suoqu_text)
        self.assertIn("constexpr double kPseudoSlamCaptureGatePollIntervalSec = 0.1;", suoqu_text)
        self.assertIn("constexpr double kPseudoSlamCaptureGateImageMeanDiffThreshold = 2.5;", suoqu_text)
        self.assertIn("void pseudo_slam_ir_image_callback(const sensor_msgs::ImageConstPtr& msg)", suoqu_text)
        self.assertIn("bool wait_for_pseudo_slam_capture_gate(", suoqu_text)
        self.assertIn("pseudo_slam scan_only区域%d等待最终采集门通过", suoqu_text)
        self.assertIn("pseudo_slam scan_only区域%d最终采集门已通过", suoqu_text)

    def test_pseudo_slam_scan_can_skip_capture_gate_when_request_disables_it(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn("if (enable_capture_gate)", scan_function)
        self.assertIn("wait_for_pseudo_slam_capture_gate(1, global_scan_center, global_scan_height)", scan_function)
        self.assertIn("已关闭最终采集门，直接请求一次视觉", scan_function)

    def test_pseudo_slam_capture_gate_waits_for_new_ir_frame_without_resetting_stability(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("const bool image_sample_updated = current_roi_stamp != previous_roi_stamp;", suoqu_text)
        self.assertIn("if (has_previous_frame && !image_sample_updated)", suoqu_text)
        self.assertIn("const PseudoSlamCaptureGateConfig config = load_pseudo_slam_capture_gate_config();", suoqu_text)
        self.assertIn("ros::Duration(config.poll_interval_sec).sleep();", suoqu_text)
        self.assertNotIn(
            "if (target_reached && motion_stopped && pose_delta_stable && image_stable) {\n"
            "            stable_sample_count++;\n"
            "        } else {\n"
            "            stable_sample_count = 0;\n"
            "        }",
            suoqu_text,
        )

    def test_pseudo_slam_capture_gate_supports_hot_rosparam_overrides(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("struct PseudoSlamCaptureGateConfig", suoqu_text)
        self.assertIn("load_pseudo_slam_capture_gate_config()", suoqu_text)
        self.assertIn('ros::param::param("~pseudo_slam_capture_gate_stable_sample_count"', suoqu_text)
        self.assertIn('ros::param::param("~pseudo_slam_capture_gate_poll_interval_sec"', suoqu_text)
        self.assertIn('ros::param::param("~pseudo_slam_capture_gate_image_mean_diff_threshold"', suoqu_text)
        self.assertIn('ros::param::param("~pseudo_slam_scan_min_point_count"', suoqu_text)
        self.assertIn('ros::param::param("~pseudo_slam_scan_retry_interval_sec"', suoqu_text)

    def test_suoqu_uses_checkerboard_jump_bind_for_precomputed_global_execution(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("checkerboard_jump_bind_enabled", suoqu_text)
        self.assertIn("checkerboard_jump_bind_callback", suoqu_text)
        self.assertIn('/web/moduan/send_odd_points', suoqu_text)
        self.assertIn('point_json.value("checkerboard_parity", 0)', suoqu_text)
        self.assertIn("全局棋盘格跳绑已开启", suoqu_text)

    def test_suoqu_publishes_pseudo_slam_points_to_rviz_in_cabin_frame(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("/cabin/pseudo_slam_markers", suoqu_text)
        self.assertIn("visualization_msgs::MarkerArray", suoqu_text)
        self.assertIn("visualization_msgs::Marker::SPHERE_LIST", suoqu_text)
        self.assertIn("visualization_msgs::Marker::TEXT_VIEW_FACING", suoqu_text)
        self.assertIn("visualization_msgs::Marker::DELETEALL", suoqu_text)
        self.assertIn('marker.header.frame_id = "cabin_frame"', suoqu_text)
        self.assertIn("global_idx", suoqu_text)

    def test_suoqu_restores_pseudo_slam_markers_from_points_json_on_startup(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        main_function = extract_cpp_block(suoqu_text, "int main(int argc, char **argv)")

        self.assertIn("bool load_pseudo_slam_marker_points_from_json(", suoqu_text)
        self.assertIn('pseudo_slam_points_json_file', suoqu_text)
        self.assertIn('if (!points_json.contains("pseudo_slam_points") ||', suoqu_text)
        self.assertIn('publish_pseudo_slam_markers(restored_marker_points);', suoqu_text)
        self.assertIn('Cabin_log: 已从pseudo_slam_points.json恢复%d个历史扫描点到/cabin/pseudo_slam_markers。', suoqu_text)
        self.assertIn('Cabin_Warn: 启动时恢复/cabin/pseudo_slam_markers失败：%s', suoqu_text)
        self.assertIn('restore_pseudo_slam_markers_from_json_on_startup();', main_function)

    def test_suoqu_rviz_markers_highlight_current_execution_area_and_dispatched_points(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("struct PseudoSlamMarkerExecutionState", suoqu_text)
        self.assertIn("std::unordered_set<int> highlighted_area_global_indices;", suoqu_text)
        self.assertIn("std::unordered_set<int> active_dispatch_global_indices;", suoqu_text)
        self.assertIn("std_msgs::ColorRGBA point_color;", suoqu_text)
        self.assertIn("points_marker.colors.push_back(point_color);", suoqu_text)
        self.assertIn("point_color.r = 1.0f;", suoqu_text)
        self.assertIn("point_color.g = 0.20f;", suoqu_text)
        self.assertIn("point_color.b = 0.20f;", suoqu_text)
        self.assertIn("point_color.g = 0.92f;", suoqu_text)
        self.assertIn("set_pseudo_slam_marker_execution_state(", suoqu_text)
        self.assertIn("clear_pseudo_slam_marker_execution_state();", suoqu_text)

    def test_suoqu_publishes_pseudo_slam_world_point_tfs(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("pseudo_slam_point_", suoqu_text)
        self.assertIn("publish_pseudo_slam_point_transforms()", suoqu_text)
        self.assertIn("pseudo_slam_tf_points_mutex", suoqu_text)
        self.assertIn("sendTransform(point_transforms)", suoqu_text)

    def test_pseudo_slam_filters_planning_outliers_but_keeps_visualized_points(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("kPseudoSlamPlanningZOutlierMm", suoqu_text)
        self.assertIn("constexpr float kPseudoSlamPlanningZOutlierMm = 8.0f;", suoqu_text)
        self.assertIn('ros::param::param("~pseudo_slam_planning_z_outlier_mm"', suoqu_text)
        self.assertIn('ros::param::param(\n        "~pseudo_slam_outlier_secondary_plane_z_threshold_mm"', suoqu_text)
        self.assertIn("fit_pseudo_slam_plane", suoqu_text)
        self.assertIn("fit_pseudo_slam_plane_least_squares", suoqu_text)
        self.assertIn("solve_pseudo_slam_plane_from_three_points", suoqu_text)
        self.assertIn("compute_pseudo_slam_plane_z_residual_mm", suoqu_text)
        self.assertIn("filter_pseudo_slam_planning_outliers", suoqu_text)
        self.assertIn("RANSAC", suoqu_text)
        self.assertIn("used_ransac", suoqu_text)
        self.assertIn("拟合平面", suoqu_text)
        self.assertNotIn("z中位数=", suoqu_text)
        self.assertIn(
            "std::vector<chassis_ctrl::PointCoords> planning_world_points = filter_pseudo_slam_planning_outliers(merged_world_points);",
            suoqu_text,
        )
        self.assertIn(
            "build_dynamic_bind_area_entries_from_scan_world(",
            suoqu_text,
        )
        self.assertIn("merged_checkerboard_info_by_idx", suoqu_text)
        self.assertIn("write_pseudo_slam_points_json(", suoqu_text)
        self.assertIn("merged_world_points,", suoqu_text)
        self.assertIn("merged_checkerboard_info_by_idx,", suoqu_text)
        self.assertIn("checkerboard_info_by_idx,", suoqu_text)
        self.assertIn("publish_pseudo_slam_markers(merged_world_points)", suoqu_text)
        self.assertIn("set_pseudo_slam_tf_points(merged_world_points)", suoqu_text)

    def test_pseudo_slam_marker_outlier_threshold_can_hot_refresh_rviz(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("refresh_pseudo_slam_marker_outlier_state_from_current_points", suoqu_text)
        self.assertIn("maybe_refresh_pseudo_slam_marker_outlier_threshold", suoqu_text)
        self.assertIn("pseudo_slam离群阈值热更新", suoqu_text)
        self.assertIn("pseudo_slam_outlier_secondary_plane_z_threshold_mm", suoqu_text)
        self.assertIn("pseudo_slam_outlier_secondary_plane_neighbor_xy_tolerance_mm", suoqu_text)
        self.assertIn("publish_pseudo_slam_markers(marker_points_snapshot)", suoqu_text)

    def test_pseudo_slam_does_not_block_neighbor_columns_around_parallel_outliers(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertNotIn("kPseudoSlamPlanningOutlierNeighborColumnRadius", suoqu_text)
        self.assertNotIn("collect_pseudo_slam_blocked_columns_from_outliers", suoqu_text)
        self.assertNotIn("filter_pseudo_slam_parallel_outlier_neighbor_columns", suoqu_text)
        self.assertNotIn("pseudo_slam平行离群列屏蔽", suoqu_text)

    def test_pseudo_slam_filters_normal_points_near_z_outlier_columns_within_ten_mm(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr float kPseudoSlamOutlierColumnAxisToleranceMm = 10.0f;", suoqu_text)
        self.assertIn("constexpr float kPseudoSlamOutlierColumnPointToleranceMm = 10.0f;", suoqu_text)
        self.assertIn("collect_pseudo_slam_planning_z_outliers", suoqu_text)
        self.assertIn("filter_pseudo_slam_points_near_outlier_columns", suoqu_text)
        self.assertIn("拟合成列的z离群点附近±10mm内正常点视为不可执行", suoqu_text)
        self.assertIn(
            "const std::vector<chassis_ctrl::PointCoords> planning_z_outlier_points =",
            suoqu_text,
        )
        self.assertIn(
            "planning_world_points = filter_pseudo_slam_points_near_outlier_columns(",
            suoqu_text,
        )

    def test_points_removed_by_outlier_column_filter_are_not_kept_as_checkerboard_members(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("sync_merged_checkerboard_membership_with_planning", suoqu_text)
        self.assertIn("merged_checkerboard_info_by_idx = sync_merged_checkerboard_membership_with_planning(", suoqu_text)
        self.assertIn("merged_info.is_checkerboard_member = false;", suoqu_text)
        self.assertIn("planning_checkerboard_info_by_idx.find(entry.first)", suoqu_text)

    def test_pseudo_slam_points_json_persists_outlier_marker_flag(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        write_points_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_points_json(",
        )

        self.assertIn('"is_planning_outlier"', write_points_text)
        self.assertIn("const bool is_planning_outlier =", write_points_text)
        self.assertIn("!is_planning_checkerboard_member", write_points_text)

    def test_pseudo_slam_points_json_persists_outlier_column_neighbor_blocked_flag(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        write_points_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_points_json(",
        )

        self.assertIn('"is_outlier_column_neighbor_blocked"', write_points_text)
        self.assertIn("const bool is_outlier_column_neighbor_blocked =", write_points_text)
        self.assertIn("outlier_column_neighbor_blocked_global_indices.count(point.idx) > 0", write_points_text)

    def test_pseudo_slam_points_json_persists_outlier_line_member_flag(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        write_points_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_points_json(",
        )

        self.assertIn('"is_planning_outlier_line_member"', write_points_text)
        self.assertIn("const bool is_planning_outlier_line_member =", write_points_text)
        self.assertIn("outlier_line_global_indices.count(point.idx) > 0", write_points_text)

    def test_pseudo_slam_points_json_persists_outlier_secondary_plane_flag(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        write_points_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_points_json(",
        )

        self.assertIn('"is_outlier_secondary_plane_member"', write_points_text)
        self.assertIn("const bool is_outlier_secondary_plane_member =", write_points_text)
        self.assertIn("outlier_secondary_plane_global_indices.count(point.idx) > 0", write_points_text)

    def test_pseudo_slam_marker_restore_loads_outlier_flags(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        restore_function = extract_cpp_block(
            suoqu_text,
            "bool load_pseudo_slam_marker_points_from_json(",
        )

        self.assertIn("restored_outlier_global_indices.clear()", restore_function)
        self.assertIn('point_json.value("is_planning_outlier", false)', restore_function)
        self.assertIn("restored_outlier_global_indices.insert(point.idx);", restore_function)

    def test_pseudo_slam_marker_restore_loads_outlier_column_neighbor_flags(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        restore_function = extract_cpp_block(
            suoqu_text,
            "bool load_pseudo_slam_marker_points_from_json(",
        )

        self.assertIn("restored_outlier_column_neighbor_global_indices.clear()", restore_function)
        self.assertIn('point_json.value("is_outlier_column_neighbor_blocked", false)', restore_function)
        self.assertIn("restored_outlier_column_neighbor_global_indices.insert(point.idx);", restore_function)

    def test_pseudo_slam_marker_restore_loads_outlier_line_flags(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        restore_function = extract_cpp_block(
            suoqu_text,
            "bool load_pseudo_slam_marker_points_from_json(",
        )

        self.assertIn("restored_outlier_line_global_indices.clear()", restore_function)
        self.assertIn('point_json.value("is_planning_outlier_line_member", false)', restore_function)
        self.assertIn("restored_outlier_line_global_indices.insert(point.idx);", restore_function)

    def test_pseudo_slam_marker_restore_loads_outlier_secondary_plane_flags(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        restore_function = extract_cpp_block(
            suoqu_text,
            "bool load_pseudo_slam_marker_points_from_json(",
        )

        self.assertIn("restored_outlier_secondary_plane_global_indices.clear()", restore_function)
        self.assertIn('point_json.value("is_outlier_secondary_plane_member", false)', restore_function)
        self.assertIn("restored_outlier_secondary_plane_global_indices.insert(point.idx);", restore_function)

    def test_pseudo_slam_markers_use_distinct_color_for_outliers(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        publish_function = extract_cpp_block(
            suoqu_text,
            "void publish_pseudo_slam_markers(",
        )

        self.assertIn("const bool is_outlier_point =", publish_function)
        self.assertIn("pseudo_slam_marker_outlier_global_indices", publish_function)
        self.assertIn("point_color.r = 0.05f;", publish_function)
        self.assertIn("point_color.g = 0.05f;", publish_function)
        self.assertIn("point_color.b = 0.05f;", publish_function)

    def test_pseudo_slam_markers_use_distinct_color_for_outlier_column_neighbor_blocked_points(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        publish_function = extract_cpp_block(
            suoqu_text,
            "void publish_pseudo_slam_markers(",
        )

        self.assertIn("const bool is_outlier_column_neighbor_blocked =", publish_function)
        self.assertIn("pseudo_slam_marker_outlier_column_neighbor_global_indices", publish_function)
        self.assertIn("is_outlier_column_neighbor_blocked || is_outlier_line_point || is_outlier_point", publish_function)
        self.assertIn("point_color.r = 0.05f;", publish_function)

    def test_pseudo_slam_markers_use_distinct_color_for_outlier_line_points(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        publish_function = extract_cpp_block(
            suoqu_text,
            "void publish_pseudo_slam_markers(",
        )

        self.assertIn("const bool is_outlier_line_point =", publish_function)
        self.assertIn("pseudo_slam_marker_outlier_line_global_indices", publish_function)
        self.assertIn("is_outlier_column_neighbor_blocked || is_outlier_line_point || is_outlier_point", publish_function)
        self.assertIn("point_color.r = 0.05f;", publish_function)

    def test_pseudo_slam_markers_use_distinct_color_for_outlier_secondary_plane_points(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        publish_function = extract_cpp_block(
            suoqu_text,
            "void publish_pseudo_slam_markers(",
        )

        self.assertIn("const bool is_outlier_secondary_plane_point =", publish_function)
        self.assertIn("pseudo_slam_marker_outlier_secondary_plane_global_indices", publish_function)
        self.assertIn("point_color.r = 0.55f;", publish_function)
        self.assertIn("point_color.g = 0.85f;", publish_function)
        self.assertIn("point_color.b = 0.30f;", publish_function)

    def test_pseudo_slam_collects_outlier_line_members_from_outlier_points(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr float kPseudoSlamOutlierLineDistanceToleranceMm = 12.0f;", suoqu_text)
        self.assertIn("constexpr int kPseudoSlamOutlierLineMinPointCount = 3;", suoqu_text)
        self.assertIn("collect_pseudo_slam_outlier_line_global_indices", suoqu_text)
        self.assertIn("compute_pseudo_slam_xy_point_to_line_distance_mm", suoqu_text)
        self.assertIn("pseudo_slam离群线拟合", suoqu_text)

    def test_pseudo_slam_collects_secondary_plane_members_from_outlier_points(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr int kPseudoSlamOutlierSecondaryPlaneMinPointCount = 6;", suoqu_text)
        self.assertIn("constexpr float kPseudoSlamOutlierSecondaryPlaneNeighborToleranceMm = 100.0f;", suoqu_text)
        self.assertIn("collect_pseudo_slam_outlier_secondary_plane_global_indices", suoqu_text)
        self.assertIn("filter_pseudo_slam_points_near_outlier_secondary_plane_members", suoqu_text)
        self.assertIn("pseudo_slam离群点二次平面拟合", suoqu_text)
        self.assertIn("load_pseudo_slam_outlier_secondary_plane_threshold_mm", suoqu_text)
        self.assertIn("load_pseudo_slam_outlier_secondary_plane_neighbor_tolerance_mm", suoqu_text)
        self.assertIn("pseudo_slam离群二次平面成员附近xy±", suoqu_text)

    def test_pseudo_slam_only_executes_points_that_belong_to_checkerboard(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("is_checkerboard_member = false", suoqu_text)
        self.assertIn("has_horizontal_neighbor", suoqu_text)
        self.assertIn("has_vertical_neighbor", suoqu_text)
        self.assertIn("filter_pseudo_slam_non_checkerboard_points", suoqu_text)
        self.assertIn("checkerboard_it->second.is_checkerboard_member", suoqu_text)
        self.assertIn(
            "planning_world_points = filter_pseudo_slam_non_checkerboard_points(",
            suoqu_text,
        )

    def test_pseudo_slam_bind_areas_are_sorted_by_snake_rows_from_corner_origin(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr float kDynamicBindSnakeRowToleranceMm = 90.0f;", suoqu_text)
        self.assertIn("sort_bind_area_entries_by_snake_rows", suoqu_text)
        self.assertIn(
            "sort_bind_area_entries_by_snake_rows(bind_area_entries, kDynamicBindSnakeRowToleranceMm);",
            suoqu_text,
        )
        self.assertIn("pseudo_slam绑扎路径已改为按索驱坐标系左下角原点蛇形排序", suoqu_text)

    def test_dynamic_bind_execution_origin_uses_first_snake_sorted_area(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )
        write_bind_path_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_bind_path_json(",
        )

        self.assertIn("BindExecutionPathOriginPose build_dynamic_bind_execution_path_origin(", suoqu_text)
        self.assertIn(
            "BindExecutionPathOriginPose execution_path_origin =\n        build_dynamic_bind_execution_path_origin(bind_area_entries, path_origin, cabin_height);",
            scan_function,
        )
        self.assertIn(
            "execution_path_origin.x = bind_area_entries.front().cabin_point.x;",
            suoqu_text,
        )
        self.assertIn(
            "execution_path_origin.y = bind_area_entries.front().cabin_point.y;",
            suoqu_text,
        )
        self.assertIn("synthetic bounding-box corner", suoqu_text)
        self.assertIn("const BindExecutionPathOriginPose& path_origin", write_bind_path_text)
        self.assertIn('{"z", path_origin.z}', write_bind_path_text)
        self.assertIn(
            'point.World_coord[0]},\n                    {"world_y", point.World_coord[1]},\n                    {"world_z", point.World_coord[2]}',
            write_bind_path_text,
        )
        self.assertIn("assign_planned_gripper_coords_to_bind_point_json(", write_bind_path_text)

    def test_bind_path_points_store_world_coordinates_and_tcp_local_coordinates_separately(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        helper_text = extract_cpp_block(
            suoqu_text,
            "bool assign_planned_gripper_coords_to_bind_point_json(",
        )
        write_bind_path_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_bind_path_json(",
        )

        self.assertIn('point_json["x"] = gripper_point.World_coord[0];', helper_text)
        self.assertIn('point_json["y"] = gripper_point.World_coord[1];', helper_text)
        self.assertIn('point_json["z"] = gripper_point.World_coord[2];', helper_text)
        self.assertIn('{"world_x", point.World_coord[0]}', write_bind_path_text)
        self.assertIn('{"world_y", point.World_coord[1]}', write_bind_path_text)
        self.assertIn('{"world_z", point.World_coord[2]}', write_bind_path_text)
        self.assertNotIn('{"x", point.World_coord[0]}', write_bind_path_text)
        self.assertNotIn('{"y", point.World_coord[1]}', write_bind_path_text)
        self.assertNotIn('{"z", point.World_coord[2]}', write_bind_path_text)

    def test_precomputed_bind_loader_reads_absolute_world_coords_from_world_fields(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        load_group_text = extract_cpp_block(
            suoqu_text,
            "std::vector<chassis_ctrl::PointCoords> load_bind_points_from_group_json(",
        )

        self.assertIn(
            'point.idx = point_json.value("local_idx", point_json.value("idx", point_index));',
            load_group_text,
        )
        self.assertIn(
            'point.World_coord[0] = point_json.value("world_x", point_json.value("x", 0.0f));',
            load_group_text,
        )
        self.assertIn(
            'point.World_coord[1] = point_json.value("world_y", point_json.value("y", 0.0f));',
            load_group_text,
        )
        self.assertIn(
            'point.World_coord[2] = point_json.value("world_z", point_json.value("z", 0.0f));',
            load_group_text,
        )

    def test_dynamic_scan_bind_planning_replaces_static_path_area_generation(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn("build_dynamic_bind_area_entries_from_scan_world", suoqu_text)
        self.assertIn(
            "std::vector<PseudoSlamGroupedAreaEntry> build_dynamic_bind_area_entries_from_scan_world(",
            suoqu_text,
        )
        self.assertIn(
            "std::vector<PseudoSlamGroupedAreaEntry> bind_area_entries =\n        build_dynamic_bind_area_entries_from_scan_world(",
            scan_function,
        )
        self.assertNotIn(
            "for (const auto& cabin_point : con_path) {\n        grouped_area_index++;",
            scan_function,
        )

    def test_dynamic_scan_bind_planning_uses_tcp_travel_range_360_320_140(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode_show.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr float kTravelMaxXMm = 360.0f;", suoqu_text)
        self.assertIn("constexpr float kTravelMaxYMm = 320.0f;", suoqu_text)
        self.assertIn("constexpr float kTravelMaxZMm = 140.0f;", suoqu_text)
        self.assertIn("point.World_coord[2] <= kTravelMaxZMm", suoqu_text)
        self.assertIn("(double)point.World_coord[0] < 360", moduan_text)
        self.assertIn("(double)point.World_coord[1] < 320", moduan_text)
        self.assertIn("(double)point.World_coord[2] < 140", moduan_text)

    def test_dynamic_scan_bind_planning_pads_last_group_to_four_template_points(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("build_dynamic_four_point_template_group", suoqu_text)
        self.assertIn("unfinished_global_indices", suoqu_text)
        self.assertIn("template_points.size() < 4", suoqu_text)
        self.assertIn("bind_group.bind_points_world.size() == 4", suoqu_text)
        self.assertIn("已绑定点仅作为模板补齐占位", suoqu_text)

    def test_execution_bind_clamps_cabin_z_floor_to_four_hundred_eighty_five(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr float kBindExecutionCabinMinZMm = 485.0f;", suoqu_text)
        self.assertIn("float clamp_bind_execution_cabin_z(float planned_cabin_z)", suoqu_text)
        self.assertIn("return std::max(planned_cabin_z, kBindExecutionCabinMinZMm);", suoqu_text)
        self.assertIn("constexpr float kExecutionArrivalToleranceMm = 40.0f;", suoqu_text)
        self.assertIn("constexpr int kExecutionArrivalStableSampleCount = 2;", suoqu_text)
        self.assertIn("constexpr float kExecutionArrivalPoseDeltaToleranceMm = 5.0f;", suoqu_text)
        self.assertIn("bool delay_time_for_execution(int Axis, double Target_position, double target_tolerance_mm = kExecutionArrivalToleranceMm)", suoqu_text)

        current_area_text = extract_cpp_block(
            suoqu_text,
            "bool run_current_area_bind_from_scan_test(std::string& message)",
        )
        self.assertIn("const float move_cabin_z = clamp_bind_execution_cabin_z(cabin_height);", current_area_text)
        self.assertIn("TCP_Move[3] = move_cabin_z;", current_area_text)
        self.assertIn("delay_time_for_execution(AXIS_Z, move_cabin_z)", current_area_text)

        live_visual_text = extract_cpp_block(
            suoqu_text,
            "bool run_live_visual_global_work(std::string& message)",
        )
        self.assertIn("float path_origin_x = 0.0f;", live_visual_text)
        self.assertIn("align_execution_path_origin_xy_to_first_area_if_needed(", suoqu_text)
        self.assertIn(
            "align_execution_path_origin_xy_to_first_area_if_needed(\n        areas_json,\n        path_origin_x,\n        path_origin_y,\n        \"live_visual\"\n    );",
            live_visual_text,
        )
        self.assertIn("const float move_path_origin_z = clamp_bind_execution_cabin_z(path_origin_z);", live_visual_text)
        self.assertIn("TCP_Move[3] = move_path_origin_z;", live_visual_text)
        self.assertIn("delay_time_for_execution(AXIS_Z, move_path_origin_z)", live_visual_text)
        self.assertIn("const float move_cabin_z = clamp_bind_execution_cabin_z(cabin_z);", live_visual_text)
        self.assertIn("TCP_Move[3] = move_cabin_z;", live_visual_text)
        self.assertIn("delay_time_for_execution(AXIS_Z, move_cabin_z)", live_visual_text)

        bind_from_scan_text = extract_cpp_block(
            suoqu_text,
            "bool run_bind_from_scan(std::string& message)",
        )
        self.assertIn(
            "align_execution_path_origin_xy_to_first_area_if_needed(\n        areas_json,\n        path_origin_x,\n        path_origin_y,\n        \"bind_from_scan\"\n    );",
            bind_from_scan_text,
        )
        self.assertIn("const float move_path_origin_z = clamp_bind_execution_cabin_z(path_origin_z);", bind_from_scan_text)
        self.assertIn("TCP_Move[3] = move_path_origin_z;", bind_from_scan_text)
        self.assertIn("delay_time_for_execution(AXIS_Z, move_path_origin_z)", bind_from_scan_text)
        self.assertIn("const float move_cabin_z = clamp_bind_execution_cabin_z(cabin_z);", bind_from_scan_text)
        self.assertIn("TCP_Move[3] = move_cabin_z;", bind_from_scan_text)
        self.assertIn("delay_time_for_execution(AXIS_Z, move_cabin_z)", bind_from_scan_text)

    def test_dynamic_bind_generator_clamps_cabin_z_floor_to_four_hundred_eighty_five(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        dynamic_planning_text = extract_cpp_block(
            suoqu_text,
            "std::vector<PseudoSlamGroupedAreaEntry> build_dynamic_bind_area_entries_from_scan_world(",
        )
        self.assertIn(
            "area_entry.cabin_z = clamp_bind_execution_cabin_z(\n            best_candidate_pose.cabin_z > 0.0f ? best_candidate_pose.cabin_z : cabin_height\n        );",
            dynamic_planning_text,
        )

        write_json_text = extract_cpp_block(
            suoqu_text,
            "bool write_pseudo_slam_bind_path_json(",
        )
        self.assertIn('{"z", area_entry.cabin_z},', write_json_text)

    def test_frontend_exposes_pseudo_slam_scan_entry(self):
        debug_button_text = (CHASSIS_CTRL_DIR / "scripts" / "debug_button_node.py").read_text(
            encoding="utf-8"
        )
        topics_transfer_text = (CHASSIS_CTRL_DIR / "src" / "topics_transfer.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("扫描建图", debug_button_text)
        self.assertIn("/web/cabin/start_pseudo_slam_scan", topics_transfer_text)
        self.assertIn("收到扫描建图命令", topics_transfer_text)

    def test_scan_entry_supports_parameterized_capture_gate_service(self):
        cmake_text = (CHASSIS_CTRL_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        topics_transfer_text = (CHASSIS_CTRL_DIR / "src" / "topics_transfer.cpp").read_text(
            encoding="utf-8"
        )
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        service_definition = (
            CHASSIS_CTRL_DIR / "srv" / "StartPseudoSlamScan.srv"
        ).read_text(encoding="utf-8")

        self.assertIn("StartPseudoSlamScan.srv", cmake_text)
        self.assertIn("uint8 SCAN_STRATEGY_SINGLE_CENTER=0", service_definition)
        self.assertIn("uint8 SCAN_STRATEGY_MULTI_POSE=1", service_definition)
        self.assertIn("uint8 SCAN_STRATEGY_FIXED_MANUAL_WORKSPACE=2", service_definition)
        self.assertIn("bool enable_capture_gate", service_definition)
        self.assertIn("uint8 scan_strategy", service_definition)
        self.assertIn("---", service_definition)
        self.assertIn("bool success", service_definition)
        self.assertIn("string message", service_definition)
        self.assertIn("ros::ServiceClient Chassis_scan_with_options_client;", topics_transfer_text)
        self.assertIn(
            'nh.serviceClient<chassis_ctrl::StartPseudoSlamScan>("/cabin/start_pseudo_slam_scan_with_options")',
            topics_transfer_text,
        )
        self.assertIn(
            'nh.advertiseService("/cabin/start_pseudo_slam_scan_with_options", startPseudoSlamScanWithOptions)',
            suoqu_text,
        )

    def test_start_work_supports_parameterized_clear_execution_memory_service(self):
        cmake_text = (CHASSIS_CTRL_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        topics_transfer_text = (CHASSIS_CTRL_DIR / "src" / "topics_transfer.cpp").read_text(
            encoding="utf-8"
        )
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        service_definition = (
            CHASSIS_CTRL_DIR / "srv" / "StartGlobalWork.srv"
        ).read_text(encoding="utf-8")

        self.assertIn("StartGlobalWork.srv", cmake_text)
        self.assertIn("string command", service_definition)
        self.assertIn("bool clear_execution_memory", service_definition)
        self.assertIn("---", service_definition)
        self.assertIn("bool success", service_definition)
        self.assertIn("string message", service_definition)
        self.assertIn("ros::ServiceClient Chassis_start_work_with_options_client;", topics_transfer_text)
        self.assertIn(
            'nh.serviceClient<chassis_ctrl::StartGlobalWork>("/cabin/start_work_with_options")',
            topics_transfer_text,
        )
        self.assertIn(
            'nh.advertiseService("/cabin/start_work_with_options", startGlobalWorkWithOptions)',
            suoqu_text,
        )

    def test_start_work_topic_uses_msg_value_to_toggle_clear_execution_memory(self):
        topics_transfer_text = (CHASSIS_CTRL_DIR / "src" / "topics_transfer.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("const bool clear_execution_memory = msg->data >= 2.0f;", topics_transfer_text)
        self.assertIn("start_work_srv.request.clear_execution_memory = clear_execution_memory;", topics_transfer_text)
        self.assertIn("全局作业命令，模式=", topics_transfer_text)

    def test_start_work_with_options_can_clear_execution_memory_before_execution(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        start_work_with_options_text = extract_cpp_block(
            suoqu_text,
            "bool startGlobalWorkWithOptions(",
        )

        self.assertIn("if (req.clear_execution_memory)", start_work_with_options_text)
        self.assertIn("reset_bind_execution_memory_from_current_scan_artifacts", start_work_with_options_text)
        self.assertIn("load_current_path_signature_for_execution", start_work_with_options_text)
        self.assertIn("clear_execution_memory=true", start_work_with_options_text)

    def test_scan_topic_uses_msg_value_to_select_scan_strategy_and_capture_gate(self):
        topics_transfer_text = (CHASSIS_CTRL_DIR / "src" / "topics_transfer.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("const bool fixed_manual_workspace_scan = msg->data >= 5.0f;", topics_transfer_text)
        self.assertIn("const bool multi_pose_scan = msg->data >= 3.0f && msg->data < 5.0f;", topics_transfer_text)
        self.assertIn("(msg->data >= 6.0f)", topics_transfer_text)
        self.assertIn("scan_srv.request.enable_capture_gate = enable_capture_gate;", topics_transfer_text)
        self.assertIn(
            "scan_srv.request.scan_strategy = fixed_manual_workspace_scan ? 2 : (multi_pose_scan ? 1 : 0);",
            topics_transfer_text,
        )
        self.assertIn("扫描建图命令，模式=", topics_transfer_text)

    def test_plain_scan_service_defaults_to_single_center_scan_without_capture_gate(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        start_scan_function = extract_cpp_block(
            suoqu_text,
            "bool startPseudoSlamScan(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)",
        )

        self.assertIn("res.success = run_pseudo_slam_scan(", start_scan_function)
        self.assertIn("PseudoSlamScanStrategy::kSingleCenter", start_scan_function)
        self.assertIn("false,", start_scan_function)

    def test_scan_options_service_passes_requested_scan_strategy(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        start_scan_with_options_function = extract_cpp_block(
            suoqu_text,
            "bool startPseudoSlamScanWithOptions(",
        )

        self.assertIn("normalize_pseudo_slam_scan_strategy(req.scan_strategy)", start_scan_with_options_function)
        self.assertIn("req.enable_capture_gate", start_scan_with_options_function)

    def test_fixed_manual_workspace_scan_strategy_uses_fixed_pose_and_speed(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("kFixedManualWorkspace = 2", suoqu_text)
        self.assertIn("kPseudoSlamFixedManualWorkspaceScanXmm = -260.0f", suoqu_text)
        self.assertIn("kPseudoSlamFixedManualWorkspaceScanYmm = 1700.0f", suoqu_text)
        self.assertIn("kPseudoSlamFixedManualWorkspaceScanZmm = 2997.0f", suoqu_text)
        self.assertIn("kPseudoSlamFixedManualWorkspaceScanSpeedMmPerSec = 100.0f", suoqu_text)
        self.assertIn("case 2:", suoqu_text)
        self.assertIn("PseudoSlamScanStrategy::kFixedManualWorkspace", suoqu_text)
        self.assertIn("pseudo_slam固定工作区扫描移动到", suoqu_text)

    def test_delay_time_returns_timeout_status_and_logs_axis_snapshot(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        delay_time_function = extract_cpp_block(
            suoqu_text,
            "bool delay_time(int Axis, double Target_position)",
        )

        self.assertIn("return false;", delay_time_function)
        self.assertIn("return true;", delay_time_function)
        self.assertIn("Cabin_Error: 等待轴%d到位超时", delay_time_function)
        self.assertIn("当前(X,Y,Z)=", delay_time_function)
        self.assertIn("motion_status=%d", delay_time_function)

    def test_execution_delay_time_allows_fast_arrival_without_motion_stopped(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        execution_delay_time_function = extract_cpp_block(
            suoqu_text,
            "bool delay_time_for_execution(int Axis, double Target_position, double target_tolerance_mm = kExecutionArrivalToleranceMm)",
        )

        self.assertIn("normalized_tolerance_mm", execution_delay_time_function)
        self.assertIn("stable_sample_count", execution_delay_time_function)
        self.assertIn("pose_delta_mm", execution_delay_time_function)
        self.assertIn("axis_error_mm", execution_delay_time_function)
        self.assertIn("axis_error_mm < normalized_tolerance_mm", execution_delay_time_function)
        self.assertIn("pose_delta_mm <= kExecutionArrivalPoseDeltaToleranceMm", execution_delay_time_function)
        self.assertIn("stable_sample_count >= kExecutionArrivalStableSampleCount", execution_delay_time_function)
        self.assertNotIn("cabin_state.motion_status == 0", execution_delay_time_function)
        self.assertIn("Cabin_log: 执行层等待轴%d到位中", execution_delay_time_function)
        self.assertIn("连续稳定样本=%d/%d", execution_delay_time_function)

    def test_heartbeat_frames_skip_extra_write_delay_to_reduce_state_latency(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        frame_generate_function = extract_cpp_block(
            suoqu_text,
            "int Frame_Generate(uint8_t* Control_Word, int Tlen, int Rlen, int socket = sockfd)",
        )

        self.assertIn("constexpr int HEARTBEAT_WRITE_DELAY_MS = 0;", suoqu_text)
        self.assertIn("const int post_send_delay_ms = is_heartbeat_frame ? HEARTBEAT_WRITE_DELAY_MS : WRITE_DELAY_MS;", frame_generate_function)
        self.assertIn("if (post_send_delay_ms > 0)", frame_generate_function)

    def test_pseudo_slam_scan_fails_fast_when_axis_move_times_out(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        scan_function = extract_cpp_block(
            suoqu_text,
            "bool run_pseudo_slam_scan(",
        )

        self.assertIn("if (!delay_time(AXIS_X, global_scan_center.x))", scan_function)
        self.assertIn('message = "扫描建图因索驱X轴到位超时而中止"', scan_function)
        self.assertIn("if (!delay_time(AXIS_Y, global_scan_center.y))", scan_function)
        self.assertIn('message = "扫描建图因索驱Y轴到位超时而中止"', scan_function)
        self.assertIn("if (!delay_time(AXIS_Z, global_scan_height))", scan_function)
        self.assertIn('message = "扫描建图因索驱Z轴到位超时而中止"', scan_function)
        self.assertIn("const int send_result = Frame_Generate_With_Retry(TCP_Move_Frame, 36, 8);", scan_function)
        self.assertIn("if (send_result < 0)", scan_function)
        self.assertIn('message = "扫描建图下发索驱移动指令失败"', scan_function)

    def test_frame_generate_logs_nonzero_status_word_from_short_reply(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        frame_generate_function = extract_cpp_block(
            suoqu_text,
            "int Frame_Generate(uint8_t* Control_Word, int Tlen, int Rlen, int socket = sockfd)",
        )

        self.assertIn("decode_tcp_protocol_status", suoqu_text)
        self.assertIn("format_tcp_protocol_status_message", suoqu_text)
        self.assertIn("const uint16_t command_word", frame_generate_function)
        self.assertIn("const auto decoded_status = decode_tcp_protocol_status", frame_generate_function)
        self.assertIn("Cabin_Warn: 索驱协议返回异常", frame_generate_function)
        self.assertIn("cache_pending_tcp_status_error(decoded_status.status_word);", frame_generate_function)
        self.assertIn("return -2;", frame_generate_function)

    def test_protocol_status_map_covers_inverse_enable_and_tcp_move(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn('case 0x0006', suoqu_text)
        self.assertIn("逆解计算激活", suoqu_text)
        self.assertIn("电机轴不在零点", suoqu_text)
        self.assertIn('case 0x0012', suoqu_text)
        self.assertIn("TCP位置运动", suoqu_text)
        self.assertIn("电机未全部使能", suoqu_text)

    def test_delay_time_fails_fast_on_pending_tcp_status_error(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        delay_time_function = extract_cpp_block(
            suoqu_text,
            "bool delay_time(int Axis, double Target_position)",
        )

        self.assertIn("bool consume_pending_tcp_status_error", suoqu_text)
        self.assertIn("consume_pending_tcp_status_error(pending_tcp_status_word)", delay_time_function)
        self.assertIn("Cabin_Error: 等待轴%d到位前检测到索驱状态字异常", delay_time_function)

    def test_motion_command_retries_until_status_word_is_accepted(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        frame_generate_with_retry_function = extract_cpp_block(
            suoqu_text,
            "int Frame_Generate_With_Retry(uint8_t* Control_Word ,int Tlen,int Rlen,int socket = sockfd)",
        )

        self.assertIn("bool is_motion_move_command_frame", suoqu_text)
        self.assertIn("const bool keep_retrying_on_status_reject", frame_generate_with_retry_function)
        self.assertIn("if (frame_result == -2)", frame_generate_with_retry_function)
        self.assertIn("继续等待索驱上位机接受当前运动指令后重试", frame_generate_with_retry_function)
        self.assertIn("continue;", frame_generate_with_retry_function)

    def test_moduan_precomputed_bind_service_bypasses_local_one_and_four_jump_bind(self):
        moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool apply_jump_bind_filter", moduan_text)
        self.assertIn("if (apply_jump_bind_filter && !should_keep_jump_bind_point(point))", moduan_text)
        self.assertIn("execute_bind_points(points, res.message, false)", moduan_text)

    def test_suoqu_flushes_logs_before_emergency_exit_and_checks_init_failure(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("void emergency_exit_with_flush", suoqu_text)
        self.assertIn("fflush(stdout);", suoqu_text)
        self.assertIn("fflush(stderr);", suoqu_text)
        self.assertIn("setvbuf(stdout, nullptr, _IONBF, 0);", suoqu_text)
        self.assertIn("setvbuf(stderr, nullptr, _IONBF, 0);", suoqu_text)
        self.assertIn("if (!suoquInit())", suoqu_text)

    def test_single_bind_topic_supports_visual_and_precomputed_current_area_modes(self):
        topics_text = (CHASSIS_CTRL_DIR / "src" / "topics_transfer.cpp").read_text(encoding="utf-8")

        self.assertIn("/web/moduan/single_bind", topics_text)
        self.assertIn("/cabin/bind_current_area_from_scan", topics_text)
        self.assertIn("single_bind模式=1", topics_text)
        self.assertIn("single_bind模式=0", topics_text)

    def test_suoqu_precomputed_bind_keeps_execution_path_without_auto_cabin_height_adjust(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("/cabin/bind_current_area_from_scan", suoqu_text)
        self.assertIn("run_current_area_bind_from_scan_test", suoqu_text)
        self.assertIn("run_bind_from_scan", suoqu_text)
        self.assertIn("find_nearest_bind_area_for_current_cabin_pose", suoqu_text)
        self.assertIn("prepare_precomputed_bind_group_for_execution", suoqu_text)
        self.assertIn("当前组转换到虎口后无x/y工作区内点", suoqu_text)
        self.assertNotIn("compute_precomputed_bind_height_shift_mm", suoqu_text)
        self.assertNotIn("adjust_cabin_height_for_precomputed_bind_shift", suoqu_text)
        self.assertNotIn("当前组点位距虎口高度超限，索驱Z自动微调", suoqu_text)
        self.assertIn("/moduan/sg_precomputed_fast", suoqu_text)

    def test_suoqu_precomputed_bind_execution_has_no_extra_sleep_delays(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        current_area_section = suoqu_text.split(
            "bool run_current_area_bind_from_scan_test(std::string& message)"
        )[1].split("bool run_bind_from_scan(std::string& message)")[0]
        global_bind_section = suoqu_text.split(
            "bool run_bind_from_scan(std::string& message)"
        )[1].split("bool startPseudoSlamScan")[0]

        self.assertNotIn("ros::Duration(0.2).sleep();", current_area_section)
        self.assertNotIn("ros::Duration(0.2).sleep();", global_bind_section)

    def test_execution_paths_update_and_clear_rviz_marker_highlight_state(self):
        suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        current_area_section = extract_cpp_block(
            suoqu_text,
            "bool run_current_area_bind_from_scan_test(std::string& message)",
        )
        live_visual_section = extract_cpp_block(
            suoqu_text,
            "bool run_live_visual_global_work(std::string& message)",
        )
        global_bind_section = extract_cpp_block(
            suoqu_text,
            "bool run_bind_from_scan(std::string& message)",
        )

        self.assertIn("set_pseudo_slam_marker_execution_state(", current_area_section)
        self.assertIn("clear_pseudo_slam_marker_execution_state();", current_area_section)
        self.assertIn("set_pseudo_slam_marker_execution_state(", live_visual_section)
        self.assertIn("clear_pseudo_slam_marker_execution_state();", live_visual_section)
        self.assertIn("set_pseudo_slam_marker_execution_state(", global_bind_section)
        self.assertIn("clear_pseudo_slam_marker_execution_state();", global_bind_section)

    def test_moduan_supports_fast_precomputed_bind_service(self):
        moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")

        self.assertIn("/moduan/sg_precomputed_fast", moduan_text)
        self.assertIn("moduan_bind_points_fast_service", moduan_text)
        self.assertIn("execute_bind_points(points, res.message, false)", moduan_text)
        self.assertIn("ScopedModuleSpeedOverride", moduan_text)
        self.assertIn("kPrecomputedFastModuleSpeedMmPerSec", moduan_text)
        self.assertIn("clear_finishall_flag_if_needed();", moduan_text)

    def test_moduan_precomputed_bind_rejects_local_z_outside_tcp_travel_range(self):
        moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")
        execute_bind_points_function = extract_cpp_block(
            moduan_text,
            "bool execute_bind_points(",
        )

        self.assertIn("constexpr double kTcpTravelMinZMm = 0.0;", moduan_text)
        self.assertIn("constexpr double kTcpTravelMaxZMm = 140.0;", moduan_text)
        self.assertIn("bool is_valid_precomputed_tcp_travel_z(double local_z_mm)", moduan_text)
        self.assertIn(
            "if (!is_valid_precomputed_tcp_travel_z(static_cast<double>(world_z)))",
            execute_bind_points_function,
        )
        self.assertIn("局部z=%.2fmm，不是合法TCP行程", execute_bind_points_function)
        self.assertIn("预生成点局部z超出TCP行程，当前组无可执行点", execute_bind_points_function)
        self.assertIn("if (!finish_all(150))", execute_bind_points_function)
        self.assertIn("等待FINISHALL标志超时，当前子区域绑扎未确认完成", execute_bind_points_function)

    def test_moduan_finish_all_wait_has_timeout_and_progress_logs(self):
        moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")
        finish_all_function = extract_cpp_block(
            moduan_text,
            "bool finish_all(int inter_time)",
        )

        self.assertIn("constexpr int kFinishAllTimeoutSec = 30;", moduan_text)
        self.assertIn("constexpr int kFinishAllLogIntervalSec = 2;", moduan_text)
        self.assertIn("Moduan_log: 等待FINISHALL标志中", finish_all_function)
        self.assertIn("Moduan_Error: 等待FINISHALL标志超时", finish_all_function)
        self.assertIn("return false;", finish_all_function)

    def test_moduan_single_move_service_uses_tcp_travel_z_limits(self):
        moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")
        move_service_function = extract_cpp_block(
            moduan_text,
            "bool moduan_move_service(chassis_ctrl::linear_module_move::Request &req,",
        )

        self.assertIn("z < kTcpTravelMinZMm || z > kTcpTravelMaxZMm", move_service_function)
        self.assertIn("TCP z轴行程仅支持0~140mm", move_service_function)
        self.assertIn("等待FINISHALL标志超时，线性模组未确认完成", move_service_function)

    def test_default_stable_frame_count_is_three(self):
        init_source = inspect.getsource(pointAI.ImageProcessor.__init__)
        self.assertIn('rospy.get_param("~stable_frame_count", 3)', init_source)

    def test_scan_only_does_not_define_separate_stability_config(self):
        init_source = inspect.getsource(pointAI.ImageProcessor.__init__)
        self.assertNotIn('rospy.get_param("~scan_stable_frame_count", 2)', init_source)
        self.assertNotIn('rospy.get_param("~scan_stable_z_tolerance_mm", 4.0)', init_source)

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
        vision_text = pointai_text

        self.assertIn("def build_coordinate_snapshot", pointai_text)
        self.assertIn("def build_coordinate_snapshot", vision_text)
        self.assertIn("def is_stable_coordinate_window", pointai_text)
        self.assertIn("def is_stable_coordinate_window", vision_text)
        self.assertIn("self.is_stable_coordinate_window(", pointai_text)
        self.assertIn("self.is_stable_coordinate_window(", vision_text)
        self.assertNotIn('if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK and result["out_of_height_count"] > 0', pointai_text)
        self.assertNotIn('if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK and result["out_of_height_count"] > 0', vision_text)

    def test_scan_only_returns_immediately_without_stability_wait_in_visual_scripts(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = pointai_text

        self.assertIn("request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY", pointai_text)
        self.assertIn("request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY", vision_text)
        self.assertIn(
            "if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:\n                manual_workspace_s2_result = self.try_scan_only_manual_workspace_s2()\n                if manual_workspace_s2_result is not None:\n                    return manual_workspace_s2_result\n                return self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)",
            pointai_text,
        )
        self.assertIn(
            "if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:\n                return self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)",
            pointai_text,
        )
        self.assertIn(
            "if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:\n                manual_workspace_s2_result = self.try_scan_only_manual_workspace_s2()\n                if manual_workspace_s2_result is not None:\n                    return manual_workspace_s2_result\n                return self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)",
            vision_text,
        )
        self.assertNotIn("def get_mode_stable_frame_count", pointai_text)
        self.assertNotIn("def get_mode_stable_frame_count", vision_text)
        self.assertNotIn("def get_mode_stable_z_tolerance_mm", pointai_text)
        self.assertNotIn("def get_mode_stable_z_tolerance_mm", vision_text)
        self.assertNotIn("pointAI等待扫描点Z稳定写入", pointai_text)
        self.assertNotIn("pointAI等待扫描点Z稳定写入", vision_text)

    def test_scan_only_skips_top_detection_occlusion_but_execute_keeps_it(self):
        processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
        processor.x1, processor.y1 = 1, 1
        processor.x2, processor.y2 = 4, 3

        processor.Depth_image_Raw = np.ones((5, 6), dtype=np.int32)
        scan_mask = processor.apply_detection_occlusions(pointAI.PROCESS_IMAGE_MODE_SCAN_ONLY)
        self.assertTrue(np.all(processor.Depth_image_Raw == 1))
        self.assertTrue(np.all(scan_mask == 1))

        processor.Depth_image_Raw = np.ones((5, 6), dtype=np.int32)
        execute_mask = processor.apply_detection_occlusions(pointAI.PROCESS_IMAGE_MODE_DEFAULT)
        expected = np.ones((5, 6), dtype=np.uint8)
        expected[1:3, 1:4] = 0
        self.assertTrue(np.array_equal(execute_mask, expected))
        expected_depth = np.ones((5, 6), dtype=np.int32)
        expected_depth[1:3, 1:4] = 0
        self.assertTrue(np.array_equal(processor.Depth_image_Raw, expected_depth))

    def test_visual_scripts_apply_top_detection_occlusion_only_outside_scan_mode(self):
        pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
        vision_text = pointai_text

        self.assertIn("def should_apply_top_detection_occlusion", pointai_text)
        self.assertIn("def should_apply_top_detection_occlusion", vision_text)
        self.assertIn("return request_mode != PROCESS_IMAGE_MODE_SCAN_ONLY", pointai_text)
        self.assertIn("return request_mode != PROCESS_IMAGE_MODE_SCAN_ONLY", vision_text)
        self.assertIn("self.detection_occlusion_mask = self.apply_detection_occlusions(request_mode)", pointai_text)
        self.assertIn("self.detection_occlusion_mask = self.apply_detection_occlusions(request_mode)", vision_text)


if __name__ == "__main__":
    unittest.main()
