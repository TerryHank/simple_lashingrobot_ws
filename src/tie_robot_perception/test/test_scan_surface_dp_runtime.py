#!/usr/bin/env python3

import ast
import re
import sys
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = REPO_ROOT / "src" / "tie_robot_perception" / "src"
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

MANUAL_WORKSPACE_S2_PATH = (
    PERCEPTION_SRC
    / "tie_robot_perception"
    / "pointai"
    / "manual_workspace_s2.py"
)
PROCESS_IMAGE_SERVICE_PATH = (
    PERCEPTION_SRC
    / "tie_robot_perception"
    / "pointai"
    / "process_image_service.py"
)
WORKSPACE_S2_PATH = (
    PERCEPTION_SRC
    / "tie_robot_perception"
    / "perception"
    / "workspace_s2.py"
)
FRONTEND_VISUAL_MODE_PATH = (
    REPO_ROOT
    / "src"
    / "tie_robot_web"
    / "frontend"
    / "src"
    / "config"
    / "visualRecognitionMode.js"
)


def _function_source(path, function_name):
    source = path.read_text(encoding="utf-8")
    module = ast.parse(source)
    for node in module.body:
        if isinstance(node, ast.FunctionDef) and node.name == function_name:
            return ast.get_source_segment(source, node)
    raise AssertionError(f"{function_name} not found in {path}")


def _draw_axis_line(response, axis, rhos, sigma_px=1.45, amplitude=1.0):
    height, width = response.shape[:2]
    y_coords, x_coords = np.indices((height, width), dtype=np.float32)
    coord_map = x_coords if axis == "x" else y_coords
    for rho in rhos:
        distance = np.abs(coord_map - float(rho))
        response += float(amplitude) * np.exp(-0.5 * (distance / float(sigma_px)) ** 2)


def _build_synthetic_rectified_grid():
    height = 96
    width = 104
    vertical_lines = [16.0, 40.0, 64.0, 88.0]
    horizontal_lines = [12.0, 36.0, 60.0, 84.0]
    ridge_signal = np.zeros((height, width), dtype=np.float32)
    _draw_axis_line(ridge_signal, "x", vertical_lines)
    _draw_axis_line(ridge_signal, "y", horizontal_lines)
    ridge_signal = np.clip(ridge_signal, 0.0, 1.0).astype(np.float32)

    valid_mask = np.ones((height, width), dtype=bool)
    rectified_depth = (1000.0 - (32.0 * ridge_signal)).astype(np.float32)
    rectified_ir = (180.0 - (85.0 * ridge_signal)).astype(np.float32)
    identity_h = np.eye(3, dtype=np.float32)
    return {
        "rectified_depth": rectified_depth,
        "filled_depth": rectified_depth.copy(),
        "rectified_ir": rectified_ir,
        "rectified_valid": valid_mask,
        "response": ridge_signal,
        "response_source": "synthetic",
        "rectified_geometry": {
            "rectified_width": width,
            "rectified_height": height,
            "inverse_h": identity_h,
            "resolution_mm_per_px": 5.0,
        },
    }


def _build_synthetic_rectified_grid_with_lines(width, height, vertical_lines, horizontal_lines, spacing_mm=140.0):
    ridge_signal = np.zeros((height, width), dtype=np.float32)
    _draw_axis_line(ridge_signal, "x", vertical_lines)
    _draw_axis_line(ridge_signal, "y", horizontal_lines)
    ridge_signal = np.clip(ridge_signal, 0.0, 1.0).astype(np.float32)

    valid_mask = np.ones((height, width), dtype=bool)
    rectified_depth = (1000.0 - (34.0 * ridge_signal)).astype(np.float32)
    rectified_ir = (185.0 - (90.0 * ridge_signal)).astype(np.float32)
    identity_h = np.eye(3, dtype=np.float32)
    resolution_mm_per_px = float(spacing_mm) / float(np.median(np.diff(vertical_lines)))
    return {
        "rectified_depth": rectified_depth,
        "filled_depth": rectified_depth.copy(),
        "rectified_ir": rectified_ir,
        "rectified_valid": valid_mask,
        "response": ridge_signal,
        "response_source": "synthetic",
        "rectified_geometry": {
            "rectified_width": width,
            "rectified_height": height,
            "inverse_h": identity_h,
            "resolution_mm_per_px": resolution_mm_per_px,
        },
    }


class ScanSurfaceDpRuntimeTest(unittest.TestCase):
    def test_surface_dp_outputs_curve_intersections_on_synthetic_grid(self):
        from tie_robot_perception.pointai import scan_surface_dp

        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid(),
            threshold_percentile=78.0,
        )

        self.assertTrue(result["success"], result.get("message"))
        self.assertEqual(result["variant_id"], "surface_dp_curve")
        self.assertEqual(result["line_counts"], [4, 4])
        self.assertEqual(len(result["rectified_intersections"]), 16)
        self.assertGreaterEqual(result["mean_completed_surface_score"], 0.65)

    def test_instance_graph_junctions_are_diagnostic_not_primary_output(self):
        from tie_robot_perception.pointai import scan_surface_dp

        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid(),
            threshold_percentile=78.0,
        )

        diagnostics = result["diagnostics"]
        self.assertGreater(diagnostics["instance_graph_junction_count"], 0)
        self.assertEqual(result["primary_point_source"], "dp_curve_intersections")
        self.assertNotEqual(
            len(result["rectified_intersections"]),
            diagnostics["instance_graph_junction_count"],
        )

    def test_manual_workspace_s2_pipeline_does_not_auto_fallback_to_depth_only(self):
        pipeline_source = _function_source(MANUAL_WORKSPACE_S2_PATH, "run_manual_workspace_s2_pipeline")

        self.assertIn("run_manual_workspace_surface_dp_pipeline", pipeline_source)
        self.assertNotIn("run_manual_workspace_s2_depth_only_pipeline", pipeline_source)
        self.assertIn('"legacy_depth_only_fallback"] = False', pipeline_source)

    def test_surface_dp_rejects_legacy_support_when_physical_prior_unresolved(self):
        from tie_robot_perception.pointai import scan_surface_dp

        width = 500
        height = 500
        legacy_lines = [50.0 + (32.0 * index) for index in range(8)]
        legacy_response = np.zeros((height, width), dtype=np.float32)
        _draw_axis_line(legacy_response, "x", legacy_lines)
        _draw_axis_line(legacy_response, "y", legacy_lines)
        legacy_response = np.clip(legacy_response, 0.0, 1.0)
        valid_mask = np.ones((height, width), dtype=bool)
        modalities = {
            "fused_instance_response": legacy_response,
            "combined_response": legacy_response,
            "depth_response": legacy_response,
            "infrared_response": legacy_response,
            "depth_gradient": legacy_response,
            "hessian_ridge": legacy_response,
            "frangi_like": legacy_response,
            "binary_candidate": legacy_response > 0.20,
        }

        surface = scan_surface_dp._build_completed_surface(
            {
                "rectified_valid": valid_mask,
                "rectified_geometry": {
                    "rectified_width": width,
                    "rectified_height": height,
                    "resolution_mm_per_px": 5.0,
                },
            },
            modalities,
            min_period=10,
            max_period=30,
        )

        self.assertEqual(surface["base_physical_source"], "physical_prior_unresolved")
        self.assertEqual(surface["completed_physical_source"], "physical_prior_unresolved")
        self.assertEqual(surface["base_line_families"], [])
        self.assertEqual(surface["completed_line_families"], [])
        self.assertEqual(int(np.count_nonzero(surface["line_support_mask"])), 0)

    def test_scan_surface_dp_runtime_labels_have_replaced_pr_fprg(self):
        process_service_text = PROCESS_IMAGE_SERVICE_PATH.read_text(encoding="utf-8")
        frontend_mode_text = FRONTEND_VISUAL_MODE_PATH.read_text(encoding="utf-8")

        for runtime_text in (process_service_text, frontend_mode_text):
            self.assertNotIn("2026-04-22", runtime_text)
            self.assertNotIn("4月22日", runtime_text)
            self.assertNotIn("PR-FPRG", runtime_text)
            self.assertIn("Surface-DP", runtime_text)

    def test_workspace_s2_old_dense_bias_is_marked_legacy_only(self):
        workspace_s2_text = WORKSPACE_S2_PATH.read_text(encoding="utf-8")

        self.assertIsNone(re.search(r"(?<!LEGACY_)PREFERRED_WORKSPACE_S2_LATTICE_LINE_COUNT", workspace_s2_text))
        self.assertIsNone(re.search(r"(?<!LEGACY_)WORKSPACE_S2_SCORE_TARGET_MIN_POINTS", workspace_s2_text))
        self.assertIn("LEGACY_WORKSPACE_S2_PREFERRED_LATTICE_LINE_COUNT", workspace_s2_text)
        self.assertIn("LEGACY_WORKSPACE_S2_SCORE_TARGET_MIN_POINTS", workspace_s2_text)

    def test_surface_dp_uses_full_workspace_physical_prior_for_scan_grid(self):
        from tie_robot_perception.pointai import scan_surface_dp

        vertical_lines = [50.0 + (28.0 * index) for index in range(16)]
        horizontal_lines = [48.0 + (28.0 * index) for index in range(16)]
        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid_with_lines(
                width=496,
                height=517,
                vertical_lines=vertical_lines,
                horizontal_lines=horizontal_lines,
            ),
            threshold_percentile=78.0,
        )

        self.assertTrue(result["success"], result.get("message"))
        self.assertEqual(result["line_counts"], [16, 16])
        self.assertEqual(len(result["rectified_intersections"]), 256)
        self.assertEqual(result["diagnostics"]["physical_prior_modes"], ["full_workspace", "full_workspace"])

    def test_surface_dp_falls_back_to_visible_local_prior_for_small_views(self):
        from tie_robot_perception.pointai import scan_surface_dp

        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid_with_lines(
                width=96,
                height=72,
                vertical_lines=[20.0, 48.0, 76.0],
                horizontal_lines=[18.0, 46.0],
            ),
            threshold_percentile=76.0,
        )

        self.assertTrue(result["success"], result.get("message"))
        self.assertEqual(result["line_counts"], [2, 3])
        self.assertEqual(len(result["rectified_intersections"]), 6)
        self.assertEqual(result["diagnostics"]["physical_prior_modes"], ["visible_local", "visible_local"])

    def test_surface_dp_uses_ridge_modalities_when_fused_response_misses_one_axis(self):
        from tie_robot_perception.pointai import scan_surface_dp

        width = 496
        height = 517
        vertical_lines = [50.0 + (28.0 * index) for index in range(16)]
        horizontal_lines = [48.0 + (28.0 * index) for index in range(16)]
        fused_response = np.zeros((height, width), dtype=np.float32)
        ridge_response = np.zeros((height, width), dtype=np.float32)
        _draw_axis_line(fused_response, "y", horizontal_lines)
        _draw_axis_line(ridge_response, "x", vertical_lines)
        _draw_axis_line(ridge_response, "y", horizontal_lines)
        fused_response = np.clip(fused_response, 0.0, 1.0)
        ridge_response = np.clip(ridge_response, 0.0, 1.0)
        valid_mask = np.ones((height, width), dtype=bool)
        modalities = {
            "fused_instance_response": fused_response,
            "combined_response": fused_response,
            "depth_response": fused_response,
            "infrared_response": ridge_response,
            "depth_gradient": ridge_response,
            "hessian_ridge": ridge_response,
            "frangi_like": ridge_response,
            "binary_candidate": ridge_response > 0.20,
        }

        surface = scan_surface_dp._build_completed_surface(
            {
                "rectified_valid": valid_mask,
                "rectified_geometry": {
                    "rectified_width": width,
                    "rectified_height": height,
                    "resolution_mm_per_px": 5.0,
                },
            },
            modalities,
            min_period=10,
            max_period=30,
        )

        self.assertEqual(
            [len(family.get("line_rhos", [])) for family in surface["completed_line_families"]],
            [16, 16],
        )
        self.assertEqual(
            [family.get("physical_prior_mode") for family in surface["completed_line_families"]],
            ["full_workspace", "full_workspace"],
        )


if __name__ == "__main__":
    unittest.main()
