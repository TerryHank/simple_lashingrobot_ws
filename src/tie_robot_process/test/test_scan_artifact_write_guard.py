#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROCESS_DIR = WORKSPACE_ROOT / "tie_robot_process"
WEB_DIR = WORKSPACE_ROOT / "tie_robot_web"


class ScanArtifactWriteGuardTest(unittest.TestCase):
    def test_scan_artifacts_only_persist_from_fixed_recognition_pose_strategy(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("should_persist_pseudo_slam_bind_artifacts(", suoqu_node)
        helper_start = suoqu_node.index("bool should_persist_pseudo_slam_bind_artifacts(")
        helper_end = suoqu_node.index("\nbool run_pseudo_slam_scan(", helper_start)
        helper_body = suoqu_node[helper_start:helper_end]
        self.assertIn("scan_strategy == PseudoSlamScanStrategy::kFixedManualWorkspace", helper_body)
        self.assertNotIn("kSingleCenter", helper_body)
        self.assertNotIn("kMultiPose", helper_body)

        write_guard_index = suoqu_node.index("should_persist_pseudo_slam_bind_artifacts(scan_strategy)")
        points_write_index = suoqu_node.index("write_pseudo_slam_points_json(", write_guard_index)
        bind_path_write_index = suoqu_node.index("write_pseudo_slam_bind_path_json(", write_guard_index)
        memory_write_index = suoqu_node.index("write_bind_execution_memory_json(", write_guard_index)
        self.assertLess(write_guard_index, points_write_index)
        self.assertLess(write_guard_index, bind_path_write_index)
        self.assertLess(write_guard_index, memory_write_index)
        self.assertIn("当前不是固定识别位姿扫描", suoqu_node[write_guard_index:points_write_index])

    def test_legacy_scan_service_defaults_to_fixed_recognition_pose(self):
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")

        start_index = service_orchestration.index("bool startPseudoSlamScan(")
        end_index = service_orchestration.index("\nbool startPseudoSlamScanWithOptions", start_index)
        start_service_body = service_orchestration[start_index:end_index]
        self.assertIn("PseudoSlamScanStrategy::kFixedManualWorkspace", start_service_body)
        self.assertNotIn("PseudoSlamScanStrategy::kSingleCenter", start_service_body)

    def test_single_point_bind_calls_atomic_backend_service_without_frontend_visual_split(self):
        task_action_controller = (
            WEB_DIR / "frontend" / "src" / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")

        single_bind_start = task_action_controller.index("async triggerSinglePointBind()")
        single_bind_end = task_action_controller.index("\n  handleSavedWorkspacePayload", single_bind_start)
        single_bind_body = task_action_controller[single_bind_start:single_bind_end]

        self.assertIn("await this.rosConnection.callSinglePointBindService()", single_bind_body)
        self.assertNotIn("triggerSurfaceDpRecognition({", single_bind_body)
        self.assertNotIn("callLashingRecognizeOnceService", single_bind_body)
        self.assertNotIn("startPseudoSlamScanActionClient", single_bind_body)
        self.assertNotIn("triggerPseudoSlamScan", single_bind_body)
        self.assertNotIn("pseudo_slam_bind_path", single_bind_body)


if __name__ == "__main__":
    unittest.main()
