#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROCESS_IMAGE_SERVICE_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "src"
    / "tie_robot_perception"
    / "pointai"
    / "process_image_service.py"
)
CHANGELOG_PATH = WORKSPACE_ROOT.parent / "CHANGELOG.md"
PR_FPRG_KNOWLEDGE_PATH = WORKSPACE_ROOT.parent / "docs" / "handoff" / "2026-04-23_pr_fprg_knowledge.md"


class PointAIScanOnlyPrFrpgTest(unittest.TestCase):
    def test_scan_only_wait_loop_bypasses_pre_img_gate(self):
        service_text = PROCESS_IMAGE_SERVICE_PATH.read_text(encoding="utf-8")
        start_index = service_text.index("def wait_for_stable_point_coords(self, request_mode):")
        end_index = service_text.index("def handle_process_image(self, req):", start_index)
        wait_loop_text = service_text[start_index:end_index]

        scan_only_branch_index = wait_loop_text.index("if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:")
        pre_img_index = wait_loop_text.index("point_coords = self.pre_img(request_mode=request_mode)")
        self.assertLess(
            scan_only_branch_index,
            pre_img_index,
            "扫描模式应先直接尝试 PR-FPRG 手动工作区 S2，而不是先走 pre_img() 门控",
        )
        self.assertIn(
            "manual_workspace_s2_result = self.try_scan_only_manual_workspace_s2()",
            wait_loop_text,
        )
        self.assertIn(
            "扫描模式当前只走 PR-FPRG 手动工作区 S2，请先提交已保存工作区四边形。",
            wait_loop_text,
        )
        self.assertIn(
            "pointAI process_image timed out while waiting for PR-FPRG manual workspace S2",
            wait_loop_text,
        )
        self.assertNotIn(
            "return self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)\n        if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:",
            wait_loop_text,
        )

    def test_project_logs_record_scan_only_pr_fprg_rule(self):
        changelog_text = CHANGELOG_PATH.read_text(encoding="utf-8")
        knowledge_text = PR_FPRG_KNOWLEDGE_PATH.read_text(encoding="utf-8")

        self.assertIn(
            "PROCESS_IMAGE_MODE_SCAN_ONLY` 下的 S2 现在直接轮询 `PR-FPRG` 手动工作区链路",
            changelog_text,
        )
        self.assertIn(
            "在 `PROCESS_IMAGE_MODE_SCAN_ONLY` 里先等 `pre_img()` 旧检测出点，再把 `S2` 当成附加步骤",
            knowledge_text,
        )

    def test_manual_workspace_s2_result_also_bridges_to_coordinate_point(self):
        rendering_path = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "rendering.py"
        )
        rendering_text = rendering_path.read_text(encoding="utf-8")
        start_index = rendering_text.index("def publish_manual_workspace_s2_result(self, result_image, points_array_msg):")
        end_index = rendering_text.index("def draw_mask_contours(self, result_image, workspace_mask, color, thickness):", start_index)
        publish_text = rendering_text[start_index:end_index]

        self.assertIn("self.coordinate_publisher.publish(points_array_msg)", publish_text)
        self.assertIn("self.manual_workspace_s2_points_pub.publish(points_array_msg)", publish_text)


if __name__ == "__main__":
    unittest.main()
