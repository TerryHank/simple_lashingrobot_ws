#!/usr/bin/env python3

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
TOOL_PATH = REPO_ROOT / "src" / "tie_robot_perception" / "tools" / "bind_point_modality_sweep_report.py"


class BindPointModalitySweepReportTest(unittest.TestCase):
    def test_report_tool_exists_and_is_report_only(self):
        self.assertTrue(TOOL_PATH.exists(), "缺少绑扎点模态扫图报告脚本。")

        source = TOOL_PATH.read_text(encoding="utf-8")
        self.assertIn("camera_channels_to_tcp_jaw_channels", source)
        self.assertIn("build_execution_refine_candidate_points", source)
        self.assertIn("执行层使用 TCP 工具坐标/线性模组工作范围", source)
        self.assertIn("不使用手动画的像素工作区作为执行 ROI", source)
        self.assertIn("Hough", source)
        self.assertIn("pseudo_slam_points.json", source)
        self.assertIn("pseudo_slam_bind_path.json", source)
        self.assertIn("READ_ONLY_GUARD_PATHS", source)
        self.assertIn("assert_read_only_inputs_unchanged", source)
        self.assertIn("真实准确率", source)
        self.assertIn("无标注代理评分", source)
        self.assertIn("最佳模态", source)

    def test_build_report_from_snapshot_writes_ranked_summary_without_touching_inputs(self):
        import importlib.util

        spec = importlib.util.spec_from_file_location("bind_point_modality_sweep_report", TOOL_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory() as tmp_dir:
            output_dir = Path(tmp_dir) / "report"
            summary = module.build_report(
                output_dir=output_dir,
                snapshot_dir=REPO_ROOT / "docs" / "releases" / "slam_v30" / "visual_modalities",
                max_points=12,
            )

            self.assertEqual(summary["evaluation_mode"], "unlabeled_proxy")
            self.assertEqual(summary["point_source"], "execution_refine_hough_tcp_roi")
            self.assertNotIn("points_path", summary)
            self.assertIn("execution_roi", summary)
            self.assertEqual(summary["execution_roi"]["roi_frame"], "gripper_frame_tcp_tool_workspace")
            self.assertIn("hough_diagnostics", summary)
            self.assertGreaterEqual(summary["hough_diagnostics"]["tcp_range_mask_pixels"], 0)
            self.assertGreaterEqual(summary["point_count"], 0)
            self.assertGreaterEqual(len(summary["modalities"]), 8)
            modality_ids = [item["id"] for item in summary["modalities"]]
            self.assertIn("white_box_multimodal_rule", modality_ids)
            self.assertIn("best_modality", summary)
            self.assertTrue(summary["best_modality"]["id"])
            self.assertGreaterEqual(summary["best_modality"]["proxy_score"], 0.0)
            self.assertLessEqual(summary["best_modality"]["proxy_score"], 1.0)
            white_box = next(item for item in summary["modalities"] if item["id"] == "white_box_multimodal_rule")
            self.assertIn("samples", white_box)
            if white_box["samples"]:
                self.assertIn("white_box_center_blob_score", white_box["samples"][0]["metrics"])
                self.assertIn("white_box_center_blob_hull_fill", white_box["samples"][0]["metrics"])

            summary_path = output_dir / "summary.json"
            self.assertTrue(summary_path.exists())
            persisted = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["best_modality"]["id"], persisted["best_modality"]["id"])

            index_html = (output_dir / "index.html").read_text(encoding="utf-8")
            self.assertIn("绑扎点分类模态扫图报告", index_html)
            self.assertIn("执行层使用 TCP 工具坐标/线性模组工作范围", index_html)
            self.assertIn("不使用手动画的像素工作区作为执行 ROI", index_html)
            self.assertIn("无标注代理评分", index_html)
            self.assertIn("最佳模态", index_html)
            self.assertIn("真实准确率", index_html)

    def test_image_label_renderer_uses_real_chinese_glyphs(self):
        import importlib.util

        spec = importlib.util.spec_from_file_location("bind_point_modality_sweep_report", TOOL_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        canvas = np.zeros((80, 360, 3), dtype=np.uint8)
        chinese_label = module.draw_label(canvas, "深度高度差")
        question_mark_label = module.draw_label(canvas, "?" * len("深度高度差".encode("utf-8")))

        self.assertGreater(
            np.count_nonzero(chinese_label != question_mark_label),
            0,
            "中文标签不应被 OpenCV Hershey 字体退化渲染成一串问号。",
        )


if __name__ == "__main__":
    unittest.main()
