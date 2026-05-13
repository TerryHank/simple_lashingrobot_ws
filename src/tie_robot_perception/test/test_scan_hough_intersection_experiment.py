#!/usr/bin/env python3

import ast
import sys
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = REPO_ROOT / "src" / "tie_robot_perception" / "src"
TOOLS_DIR = REPO_ROOT / "src" / "tie_robot_perception" / "tools"
for import_path in (PERCEPTION_SRC, TOOLS_DIR):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

HOUGH_EXPERIMENT_PATH = TOOLS_DIR / "scan_hough_intersection_experiment.py"
HOUGH_REPORT_PATH = TOOLS_DIR / "current_scan_all_sources_hough_report.py"


def _draw_axis_line(response, axis, rhos, sigma_px=1.15, amplitude=1.0):
    height, width = response.shape[:2]
    y_coords, x_coords = np.indices((height, width), dtype=np.float32)
    coord_map = x_coords if axis == "x" else y_coords
    for rho in rhos:
        distance = np.abs(coord_map - float(rho))
        response += float(amplitude) * np.exp(-0.5 * (distance / float(sigma_px)) ** 2)


def _function_source(path, function_name):
    source = path.read_text(encoding="utf-8")
    module = ast.parse(source)
    for node in module.body:
        if isinstance(node, ast.FunctionDef) and node.name == function_name:
            return ast.get_source_segment(source, node)
    raise AssertionError(f"{function_name} not found in {path}")


class ScanHoughIntersectionExperimentTest(unittest.TestCase):
    def test_extract_hough_intersections_clusters_axis_lines_on_synthetic_response(self):
        import scan_hough_intersection_experiment as hough_experiment

        response = np.zeros((96, 128), dtype=np.float32)
        _draw_axis_line(response, "x", [22.0, 58.0, 94.0])
        _draw_axis_line(response, "y", [28.0, 68.0])
        response = np.clip(response, 0.0, 1.0)

        result = hough_experiment.extract_hough_intersections(
            response,
            threshold_percentile=82.0,
            hough_threshold=12,
            min_line_length=28,
            max_line_gap=5,
            cluster_tolerance_px=4.5,
        )

        self.assertEqual(result["line_counts"], [3, 2])
        self.assertEqual(len(result["intersections"]), 6)
        for expected_x in [22.0, 58.0, 94.0]:
            self.assertTrue(
                any(abs(actual - expected_x) <= 3.0 for actual in result["vertical_lines"]),
                result["vertical_lines"],
            )
        for expected_y in [28.0, 68.0]:
            self.assertTrue(
                any(abs(actual - expected_y) <= 3.0 for actual in result["horizontal_lines"]),
                result["horizontal_lines"],
            )

    def test_extract_hough_intersections_handles_empty_response(self):
        import scan_hough_intersection_experiment as hough_experiment

        result = hough_experiment.extract_hough_intersections(
            np.zeros((40, 60), dtype=np.float32),
            threshold_percentile=90.0,
        )

        self.assertEqual(result["line_counts"], [0, 0])
        self.assertEqual(result["intersections"], [])
        self.assertEqual(result["raw_line_count"], 0)

    def test_hough_report_reuses_all_scan_sources_and_writes_summary(self):
        report_text = HOUGH_REPORT_PATH.read_text(encoding="utf-8")
        self.assertIn("from current_scan_all_sources_report import", report_text)
        self.assertIn("BASE_SOURCE_ORDER", report_text)
        self.assertIn("build_hough_response_maps", report_text)
        self.assertIn("extract_hough_intersections", report_text)
        self.assertIn('"summary.json"', report_text)
        self.assertIn("_hough_rectified.png", report_text)

        write_report_source = _function_source(HOUGH_REPORT_PATH, "write_report")
        self.assertIn("summary", write_report_source)
        self.assertIn("hough_rows", write_report_source)
        self.assertIn("index.html", write_report_source)

    def test_hough_report_writes_step_images_for_each_source(self):
        report_text = HOUGH_REPORT_PATH.read_text(encoding="utf-8")
        for suffix in (
            "_01_response.png",
            "_02_binary.png",
            "_03_skeleton.png",
            "_04_hough_segments.png",
            "_05_clustered_lines.png",
            "_06_intersections.png",
        ):
            self.assertIn(suffix, report_text)

        write_report_source = _function_source(HOUGH_REPORT_PATH, "write_report")
        self.assertIn("stage_images", write_report_source)
        self.assertIn("render_hough_stage_image", write_report_source)
        self.assertIn("流程效果图", write_report_source)

    def test_hough_report_uses_header_switcher_for_source_panels(self):
        report_text = HOUGH_REPORT_PATH.read_text(encoding="utf-8")
        write_report_source = _function_source(HOUGH_REPORT_PATH, "write_report")

        self.assertIn("source-switcher", report_text)
        self.assertIn("source-tab", report_text)
        self.assertIn("source-panel", report_text)
        self.assertIn("data-source-id", report_text)
        self.assertIn("activateSource", report_text)
        self.assertIn("querySelectorAll('.source-tab')", report_text)

        self.assertIn("tab_html", write_report_source)
        self.assertIn("panel_html", write_report_source)
        self.assertIn("active", write_report_source)
        self.assertNotIn("card_html.append(", write_report_source)

    def test_hough_stage_renderer_outputs_nonempty_images(self):
        import scan_hough_intersection_experiment as hough_experiment

        response = np.zeros((96, 128), dtype=np.float32)
        _draw_axis_line(response, "x", [22.0, 58.0, 94.0])
        _draw_axis_line(response, "y", [28.0, 68.0])
        response = np.clip(response, 0.0, 1.0)
        result = hough_experiment.extract_hough_intersections(
            response,
            threshold_percentile=82.0,
            hough_threshold=12,
            min_line_length=28,
            max_line_gap=5,
            cluster_tolerance_px=4.5,
        )

        for stage_name in ("response", "binary", "skeleton", "segments", "clustered_lines", "intersections"):
            image = hough_experiment.render_hough_stage_image(
                response,
                None,
                result,
                stage_name,
                label=stage_name,
            )
            self.assertEqual(image.ndim, 3)
            self.assertEqual(image.shape[:2], response.shape)
            self.assertGreater(int(np.count_nonzero(image)), 0, stage_name)

    def test_hough_report_can_fallback_to_visual_snapshot(self):
        report_text = HOUGH_REPORT_PATH.read_text(encoding="utf-8")
        self.assertIn("load_latest_snapshot_frame", report_text)
        self.assertIn("--snapshot-dir", report_text)
        self.assertIn("--no-snapshot-fallback", report_text)

        capture_source = _function_source(HOUGH_REPORT_PATH, "capture_frame_for_hough_report")
        self.assertIn("snapshot_dir", capture_source)
        self.assertIn("allow_snapshot_fallback", capture_source)
        self.assertIn("capture_current_frame", capture_source)
        self.assertIn("load_latest_snapshot_frame", capture_source)

    def test_hough_evaluation_builds_response_maps_without_stale_surface_dp_private_api(self):
        import current_scan_all_sources_hough_report as report

        response = np.zeros((96, 128), dtype=np.float32)
        _draw_axis_line(response, "x", [22.0, 58.0, 94.0])
        _draw_axis_line(response, "y", [28.0, 68.0])
        response = np.clip(response, 0.0, 1.0)
        rectified_depth = (1000.0 - (35.0 * response)).astype(np.float32)
        rectified_ir = (190.0 - (80.0 * response)).astype(np.float32)
        runtime_input = {
            "rectified_depth": rectified_depth,
            "filled_depth": rectified_depth.copy(),
            "rectified_ir": rectified_ir,
            "rectified_valid": np.ones(response.shape, dtype=bool),
            "response": response,
            "response_source": "synthetic",
            "rectified_geometry": {
                "rectified_width": response.shape[1],
                "rectified_height": response.shape[0],
                "inverse_h": np.eye(3, dtype=np.float32),
                "resolution_mm_per_px": 5.0,
            },
        }

        evaluation = report.build_hough_evaluation(
            runtime_input,
            threshold_percentile=82.0,
            hough_threshold_percentile=82.0,
        )

        self.assertIn("completed_surface_response", evaluation["response_maps"])
        self.assertIn("depth_gradient", evaluation["source_order"])
        self.assertIn("completed_surface_response", evaluation["source_order"])
        self.assertTrue(evaluation["hough_rows"])


if __name__ == "__main__":
    unittest.main()
