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


def _build_synthetic_rectified_grid_with_beam_band(
    raised_beam=True,
    raised_start=172,
    raised_end=181,
    beam_lift_mm=44.0,
    normal_rebar_drop_mm=18.0,
):
    width = 496
    height = 517
    vertical_lines = [50.0 + (28.0 * index) for index in range(16)]
    horizontal_lines = [48.0 + (28.0 * index) for index in range(16)]
    ridge_signal = np.zeros((height, width), dtype=np.float32)
    _draw_axis_line(ridge_signal, "x", vertical_lines)
    _draw_axis_line(ridge_signal, "y", horizontal_lines)
    _draw_axis_line(ridge_signal, "x", [170.0, 182.0], sigma_px=3.2, amplitude=1.8)
    ridge_signal = np.clip(ridge_signal, 0.0, 1.0).astype(np.float32)

    valid_mask = np.ones((height, width), dtype=bool)
    rectified_depth = (1000.0 - (float(normal_rebar_drop_mm) * ridge_signal)).astype(np.float32)
    if raised_beam:
        rectified_depth[:, int(raised_start):int(raised_end)] -= float(beam_lift_mm)
    rectified_ir = (185.0 - (90.0 * ridge_signal)).astype(np.float32)
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


def _build_synthetic_rectified_grid_with_raised_regular_column():
    width = 496
    height = 517
    vertical_lines = [50.0 + (28.0 * index) for index in range(16)]
    horizontal_lines = [48.0 + (28.0 * index) for index in range(16)]
    ridge_signal = np.zeros((height, width), dtype=np.float32)
    _draw_axis_line(ridge_signal, "x", vertical_lines)
    _draw_axis_line(ridge_signal, "y", horizontal_lines)
    _draw_axis_line(ridge_signal, "x", [190.0], sigma_px=3.2, amplitude=1.8)
    ridge_signal = np.clip(ridge_signal, 0.0, 1.0).astype(np.float32)

    valid_mask = np.ones((height, width), dtype=bool)
    rectified_depth = (1000.0 - (34.0 * ridge_signal)).astype(np.float32)
    rectified_depth[:, 187:194] -= 80.0
    rectified_ir = (185.0 - (90.0 * ridge_signal)).astype(np.float32)
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


def _build_synthetic_beam_dark_gutter_response(raised_beam=True):
    width = 320
    height = 260
    vertical_lines = [30.0, 58.0, 86.0, 180.0, 208.0, 236.0, 264.0, 292.0]
    horizontal_lines = [28.0, 56.0, 84.0, 112.0, 140.0, 168.0, 196.0, 224.0, 252.0]
    response = np.zeros((height, width), dtype=np.float32)
    _draw_axis_line(response, "x", vertical_lines, sigma_px=1.2, amplitude=1.0)
    _draw_axis_line(response, "y", horizontal_lines, sigma_px=1.2, amplitude=1.0)
    response[:, 108:151] = 0.0
    _draw_axis_line(response, "x", [112.0, 146.0], sigma_px=1.2, amplitude=1.6)
    response = np.clip(response, 0.0, 1.0).astype(np.float32)
    valid_mask = np.ones((height, width), dtype=bool)
    binary_candidate = response > 0.35
    height_response = (0.42 * binary_candidate.astype(np.float32)).astype(np.float32)
    if raised_beam:
        height_response[:, 124:135] = 0.95
    return response, binary_candidate, valid_mask, height_response


def _build_synthetic_flat_height_wide_beam_response():
    width = 420
    height = 260
    vertical_lines = [32.0 + (28.0 * index) for index in range(13)]
    horizontal_lines = [28.0, 56.0, 84.0, 112.0, 140.0, 168.0, 196.0, 224.0, 252.0]
    response = np.zeros((height, width), dtype=np.float32)
    _draw_axis_line(response, "x", vertical_lines, sigma_px=1.1, amplitude=1.0)
    _draw_axis_line(response, "y", horizontal_lines, sigma_px=1.1, amplitude=0.85)
    _draw_axis_line(response, "x", [228.0], sigma_px=3.8, amplitude=1.9)
    response = np.clip(response, 0.0, 1.0).astype(np.float32)
    valid_mask = np.ones((height, width), dtype=bool)
    binary_candidate = response > 0.34
    intermittent_rows = (np.arange(height) % 8) < 3
    binary_candidate[:, 224:233] = False
    binary_candidate[intermittent_rows, 224:233] = response[intermittent_rows, 224:233] > 0.34
    height_response = (0.42 * binary_candidate.astype(np.float32)).astype(np.float32)
    height_response[:, 224:233] = 0.50
    return response, binary_candidate, valid_mask, height_response


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

    def test_surface_dp_rejects_line_support_when_physical_spacing_unresolved(self):
        from tie_robot_perception.pointai import scan_surface_dp

        width = 500
        height = 500
        out_of_spacing_lines = [50.0 + (40.0 * index) for index in range(8)]
        legacy_response = np.zeros((height, width), dtype=np.float32)
        _draw_axis_line(legacy_response, "x", out_of_spacing_lines)
        _draw_axis_line(legacy_response, "y", out_of_spacing_lines)
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

    def test_surface_dp_accepts_spacing_valid_grid_without_full_workspace_line_count_limit(self):
        from tie_robot_perception.pointai import scan_surface_dp

        vertical_lines = [42.0 + (28.0 * index) for index in range(13)]
        horizontal_lines = [40.0 + (28.0 * index) for index in range(13)]
        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid_with_lines(
                width=420,
                height=420,
                vertical_lines=vertical_lines,
                horizontal_lines=horizontal_lines,
            ),
            threshold_percentile=78.0,
        )

        self.assertTrue(result["success"], result.get("message"))
        self.assertEqual(result["line_counts"], [13, 13])
        self.assertEqual(len(result["rectified_intersections"]), 169)

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

    def test_surface_dp_rejects_unbalanced_full_workspace_line_counts(self):
        from tie_robot_perception.pointai import scan_surface_dp

        width = 496
        height = 517
        vertical_lines = [50.0 + (28.0 * index) for index in range(16)]
        sparse_horizontal_lines = [48.0, 76.0]
        response = np.zeros((height, width), dtype=np.float32)
        _draw_axis_line(response, "x", vertical_lines)
        _draw_axis_line(response, "y", sparse_horizontal_lines)
        response = np.clip(response, 0.0, 1.0)

        families, source = scan_surface_dp._build_best_physical_axis_aligned_line_families(
            [("unbalanced_response", response)],
            np.ones((height, width), dtype=bool),
            {
                "rectified_width": width,
                "rectified_height": height,
                "resolution_mm_per_px": 5.0,
            },
            peak_min_ratio=0.16,
        )

        self.assertEqual(families, [])
        self.assertIsNone(source)

    def test_surface_dp_skips_unbalanced_first_candidate_and_uses_balanced_later_candidate(self):
        from tie_robot_perception.pointai import scan_surface_dp

        width = 496
        height = 517
        vertical_lines = [50.0 + (28.0 * index) for index in range(16)]
        horizontal_lines = [48.0 + (28.0 * index) for index in range(16)]
        unbalanced_response = np.zeros((height, width), dtype=np.float32)
        balanced_response = np.zeros((height, width), dtype=np.float32)
        _draw_axis_line(unbalanced_response, "x", vertical_lines)
        _draw_axis_line(unbalanced_response, "y", horizontal_lines[:2])
        _draw_axis_line(balanced_response, "x", vertical_lines)
        _draw_axis_line(balanced_response, "y", horizontal_lines)
        unbalanced_response = np.clip(unbalanced_response, 0.0, 1.0)
        balanced_response = np.clip(balanced_response, 0.0, 1.0)

        families, source = scan_surface_dp._build_best_physical_axis_aligned_line_families(
            [
                ("unbalanced_first", unbalanced_response),
                ("balanced_later", balanced_response),
            ],
            np.ones((height, width), dtype=bool),
            {
                "rectified_width": width,
                "rectified_height": height,
                "resolution_mm_per_px": 5.0,
            },
            peak_min_ratio=0.16,
        )

        self.assertEqual(source, "balanced_later")
        self.assertEqual([len(family.get("line_rhos", [])) for family in families], [16, 16])

    def test_surface_dp_reports_wide_beam_candidate_bands_without_filtering_points(self):
        from tie_robot_perception.pointai import scan_surface_dp

        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid_with_beam_band(),
            threshold_percentile=78.0,
        )

        self.assertTrue(result["success"], result.get("message"))
        self.assertEqual(result["line_counts"], [16, 16])
        self.assertEqual(len(result["rectified_intersections"]), 256)
        self.assertGreaterEqual(result["diagnostics"]["beam_candidate_count"], 1)
        beam_bands = result["beam_candidate_bands"]
        beam_band = next(
            band
            for band in beam_bands
            if band.get("axis") == "x" and int(band["start"]) <= 176 <= int(band["end"])
        )
        self.assertLessEqual(int(beam_band["width"]), 24)
        self.assertGreater(float(beam_band["height_delta"]), 0.05)

    def test_surface_dp_rejects_beam_like_band_unless_column_is_higher_than_surrounding_rebar(self):
        from tie_robot_perception.pointai import scan_surface_dp

        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid_with_beam_band(raised_beam=False),
            threshold_percentile=78.0,
        )

        self.assertTrue(result["success"], result.get("message"))
        self.assertEqual(result["line_counts"], [16, 16])
        self.assertEqual(result["diagnostics"]["beam_candidate_count"], 0)

    def test_surface_dp_accepts_thin_raised_beam_column_after_height_gate(self):
        from tie_robot_perception.pointai import scan_surface_dp

        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid_with_beam_band(
                raised_start=174,
                raised_end=180,
                beam_lift_mm=80.0,
            ),
            threshold_percentile=78.0,
        )

        self.assertTrue(result["success"], result.get("message"))
        self.assertGreaterEqual(result["diagnostics"]["beam_candidate_count"], 1)
        self.assertTrue(
            any(
                int(band["start"]) <= 176 <= int(band["end"])
                and int(band["width"]) <= 8
                for band in result["beam_candidate_bands"]
            ),
            result["beam_candidate_bands"],
        )

    def test_surface_dp_rejects_raised_regular_grid_column_as_beam_candidate(self):
        from tie_robot_perception.pointai import scan_surface_dp

        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid_with_raised_regular_column(),
            threshold_percentile=78.0,
        )

        self.assertTrue(result["success"], result.get("message"))
        self.assertEqual(result["line_counts"], [16, 16])
        self.assertEqual(result["diagnostics"]["beam_candidate_count"], 0)

    def test_surface_dp_detects_dark_gutter_beam_between_two_narrow_bright_edges(self):
        from tie_robot_perception.pointai import scan_surface_dp

        response, binary_candidate, valid_mask, height_response = _build_synthetic_beam_dark_gutter_response()

        beam_bands = scan_surface_dp.detect_beam_candidate_bands(
            response,
            binary_candidate,
            valid_mask,
            height_response=height_response,
        )

        self.assertTrue(
            any(
                band.get("axis") == "x"
                and int(band["start"]) <= 129 <= int(band["end"])
                and int(band["width"]) < int(band["original_width"])
                and band.get("type") == "beam_candidate"
                for band in beam_bands
            ),
            beam_bands,
        )

    def test_surface_dp_rejects_dark_gutter_band_without_height_lift(self):
        from tie_robot_perception.pointai import scan_surface_dp

        response, binary_candidate, valid_mask, height_response = _build_synthetic_beam_dark_gutter_response(
            raised_beam=False,
        )

        beam_bands = scan_surface_dp.detect_beam_candidate_bands(
            response,
            binary_candidate,
            valid_mask,
            height_response=height_response,
        )

        self.assertEqual(beam_bands, [])

    def test_surface_dp_detects_flat_height_wide_beam_from_structural_continuity(self):
        from tie_robot_perception.pointai import scan_surface_dp

        response, binary_candidate, valid_mask, height_response = _build_synthetic_flat_height_wide_beam_response()

        beam_bands = scan_surface_dp.detect_beam_candidate_bands(
            response,
            binary_candidate,
            valid_mask,
            height_response=height_response,
        )

        self.assertTrue(
            any(
                band.get("axis") == "x"
                and int(band["start"]) <= 228 <= int(band["end"])
                and band.get("beam_signature") == "wide_continuous_column"
                for band in beam_bands
            ),
            beam_bands,
        )

    def test_surface_dp_lattice_gate_keeps_dark_gutter_beam_when_line_family_lands_on_it(self):
        from tie_robot_perception.pointai import scan_surface_dp

        line_families = [
            {
                "axis_orientation": "vertical",
                "line_angle_deg": 90.0,
                "line_rhos": [50.0, 78.0, 106.0, 134.0, 162.0, 176.0, 190.0, 218.0, 246.0, 274.0],
            },
            {
                "axis_orientation": "horizontal",
                "line_angle_deg": 0.0,
                "line_rhos": [48.0, 76.0, 104.0, 132.0],
            },
        ]
        beam_band = {
            "axis": "x",
            "start": 173,
            "end": 179,
            "width": 7,
            "original_start": 154,
            "original_end": 198,
            "original_width": 45,
            "height_delta": 0.18,
            "beam_signature": "dark_gutter_edge_pair",
            "height_gate": "raised_column",
            "type": "beam_candidate",
        }

        accepted_bands, rejected_count = scan_surface_dp._filter_beam_candidate_bands_by_lattice_context(
            [beam_band],
            line_families,
            image_width=320,
        )

        self.assertEqual(rejected_count, 0)
        self.assertEqual(len(accepted_bands), 1)
        self.assertEqual(accepted_bands[0]["lattice_gate"], "dark_gutter_over_vertical_rebar_line")

    def test_surface_dp_lattice_gate_rejects_wide_raised_regular_line_without_dark_gutter(self):
        from tie_robot_perception.pointai import scan_surface_dp

        line_families = [
            {
                "axis_orientation": "vertical",
                "line_angle_deg": 90.0,
                "line_rhos": [50.0, 78.0, 106.0, 134.0, 162.0, 176.0, 190.0, 218.0, 246.0, 274.0],
            }
        ]
        raised_regular_line = {
            "axis": "x",
            "start": 173,
            "end": 179,
            "width": 7,
            "original_start": 154,
            "original_end": 198,
            "original_width": 45,
            "height_delta": 0.18,
            "height_gate": "raised_column",
            "type": "beam_candidate",
        }

        accepted_bands, rejected_count = scan_surface_dp._filter_beam_candidate_bands_by_lattice_context(
            [raised_regular_line],
            line_families,
            image_width=320,
        )

        self.assertEqual(accepted_bands, [])
        self.assertEqual(rejected_count, 1)

    def test_surface_dp_lattice_gate_accepts_structural_beam_when_line_family_swallows_it(self):
        from tie_robot_perception.pointai import scan_surface_dp

        line_families = [
            {
                "axis_orientation": "vertical",
                "line_angle_deg": 90.0,
                "line_rhos": [297.0, 331.0, 357.0, 383.0, 411.0, 441.0, 474.0],
            },
            {
                "axis_orientation": "horizontal",
                "line_angle_deg": 0.0,
                "line_rhos": [48.0, 76.0, 104.0, 132.0],
            },
        ]
        swallowed_beam = {
            "axis": "x",
            "start": 333,
            "end": 341,
            "width": 9,
            "original_start": 333,
            "original_end": 341,
            "original_width": 9,
            "height_delta": 0.02,
            "structural_delta": 0.19,
            "peak": 1.46,
            "coverage": 0.358,
            "beam_signature": "wide_continuous_column",
            "height_gate": "structure_continuity_height_flat",
            "type": "beam_candidate",
        }

        accepted_bands, rejected_count = scan_surface_dp._filter_beam_candidate_bands_by_lattice_context(
            [swallowed_beam],
            line_families,
            image_width=490,
        )

        self.assertEqual(rejected_count, 0)
        self.assertEqual(len(accepted_bands), 1)
        self.assertEqual(accepted_bands[0]["lattice_gate"], "structural_beam_over_vertical_rebar_line")

    def test_surface_dp_height_gate_compares_against_nearby_rebar_context(self):
        from tie_robot_perception.pointai import scan_surface_dp

        image_width = 491
        height_profile = np.full((image_width,), 0.36, dtype=np.float32)
        for center in (319, 350):
            height_profile[center - 2:center + 3] = 0.54
        for far_column in (379, 407):
            height_profile[far_column - 2:far_column + 3] = 0.76
        height_profile[331:340] = 0.66

        refined_band = scan_surface_dp._refine_beam_band_by_height(
            331,
            339,
            height_profile,
            image_width,
        )

        self.assertIsNotNone(refined_band)
        self.assertLessEqual(int(refined_band["start"]), 335)
        self.assertGreaterEqual(int(refined_band["end"]), 335)
        self.assertGreater(float(refined_band["height_delta"]), 0.055)

    def test_surface_dp_keeps_final_points_outside_beam_candidate_thirteen_centimeter_margin(self):
        from tie_robot_perception.pointai import scan_surface_dp

        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid_with_beam_band(),
            threshold_percentile=78.0,
            enable_beam_exclusion=True,
        )

        self.assertTrue(result["success"], result.get("message"))
        self.assertEqual(result["line_counts"], [16, 16])
        self.assertLess(len(result["rectified_intersections"]), 256)
        self.assertEqual(result["diagnostics"]["beam_exclusion_enabled"], True)
        self.assertEqual(result["diagnostics"]["beam_exclusion_margin_mm"], 130.0)
        self.assertGreater(result["diagnostics"]["beam_candidate_13cm_pixels"], 0)
        beam_margin_mask = np.asarray(result["beam_candidate_13cm_mask"], dtype=bool)
        final_points_inside_beam_mask = 0
        for point in result.get("rectified_intersections", []):
            x_index = int(round(float(point[0])))
            y_index = int(round(float(point[1])))
            if x_index < 0 or y_index < 0:
                continue
            if y_index >= beam_margin_mask.shape[0] or x_index >= beam_margin_mask.shape[1]:
                continue
            if beam_margin_mask[y_index, x_index]:
                final_points_inside_beam_mask += 1
        self.assertEqual(final_points_inside_beam_mask, 0)

    def test_surface_dp_beam_exclusion_keeps_curve_tracing_outside_beam_margin_mask(self):
        from tie_robot_perception.pointai import scan_surface_dp

        result = scan_surface_dp.build_scan_surface_dp_result(
            _build_synthetic_rectified_grid_with_beam_band(),
            threshold_percentile=78.0,
            enable_beam_exclusion=True,
        )

        self.assertTrue(result["success"], result.get("message"))
        beam_margin_mask = np.asarray(result["beam_candidate_13cm_mask"], dtype=bool)
        traced_points_inside_beam_mask = 0
        traced_points_sampled = 0
        for family in result.get("curved_families", []):
            for curved_line in family.get("curved_lines", []):
                for point in curved_line.get("polyline_points", []):
                    x_index = int(round(float(point[0])))
                    y_index = int(round(float(point[1])))
                    if x_index < 0 or y_index < 0:
                        continue
                    if y_index >= beam_margin_mask.shape[0] or x_index >= beam_margin_mask.shape[1]:
                        continue
                    traced_points_sampled += 1
                    if beam_margin_mask[y_index, x_index]:
                        traced_points_inside_beam_mask += 1

        self.assertGreater(traced_points_sampled, 0)
        self.assertEqual(traced_points_inside_beam_mask, 0)

    def test_surface_dp_debug_base_images_overlay_rectified_intersections(self):
        publish_source = _function_source(MANUAL_WORKSPACE_S2_PATH, "publish_scan_surface_dp_base_images")
        debug_source = _function_source(MANUAL_WORKSPACE_S2_PATH, "publish_scan_surface_dp_debug_image")
        manual_workspace_text = MANUAL_WORKSPACE_S2_PATH.read_text(encoding="utf-8")

        self.assertIn("draw_scan_surface_dp_debug_points", manual_workspace_text)
        self.assertIn("draw_scan_surface_dp_debug_beam_bands", manual_workspace_text)
        self.assertIn('overlay_points=surface_result.get("rectified_intersections", [])', publish_source)
        self.assertIn('overlay_beam_bands=surface_result.get("beam_candidate_bands", [])', publish_source)
        self.assertIn('"scan_surface_dp_completed_surface_image_pub"', publish_source)
        self.assertIn('encoding = "bgr8" if rendered_image.ndim == 3 else "mono8"', debug_source)


if __name__ == "__main__":
    unittest.main()
