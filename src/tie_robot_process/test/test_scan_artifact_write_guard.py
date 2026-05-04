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
        self.assertIn("scan_strategy == PseudoSlamScanStrategy::kCurrentFrameNoMotion", helper_body)
        self.assertNotIn("kSingleCenter", helper_body)
        self.assertNotIn("kMultiPose", helper_body)

        write_guard_index = suoqu_node.index("should_persist_pseudo_slam_bind_artifacts(scan_strategy)")
        points_write_index = suoqu_node.index("write_pseudo_slam_points_json(", write_guard_index)
        bind_path_write_index = suoqu_node.index("write_pseudo_slam_bind_path_json(", write_guard_index)
        memory_write_index = suoqu_node.index("write_bind_execution_memory_json(", write_guard_index)
        self.assertLess(write_guard_index, points_write_index)
        self.assertLess(write_guard_index, bind_path_write_index)
        self.assertLess(write_guard_index, memory_write_index)
        self.assertIn("当前不是可持久化扫描", suoqu_node[write_guard_index:points_write_index])

    def test_legacy_scan_service_defaults_to_fixed_recognition_pose(self):
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")

        start_index = service_orchestration.index("bool startPseudoSlamScan(")
        end_index = service_orchestration.index("\nbool startPseudoSlamScanWithOptions", start_index)
        start_service_body = service_orchestration[start_index:end_index]
        self.assertIn("PseudoSlamScanStrategy::kFixedManualWorkspace", start_service_body)
        self.assertNotIn("PseudoSlamScanStrategy::kSingleCenter", start_service_body)

    def test_fixed_recognition_pose_defaults_to_490_and_accepts_frontend_override(self):
        header = (PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp").read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")
        srv = (WORKSPACE_ROOT / "tie_robot_msgs" / "srv" / "StartPseudoSlamScan.srv").read_text(encoding="utf-8")
        action = (
            WORKSPACE_ROOT / "tie_robot_msgs" / "action" / "StartPseudoSlamScanTask.action"
        ).read_text(encoding="utf-8")
        action_bridge = (
            WORKSPACE_ROOT / "tie_robot_web" / "src" / "web_bridge" / "action_bridge.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("kPseudoSlamFixedManualWorkspaceScanXmm = 490.0f", header)
        self.assertNotIn("kPseudoSlamFixedManualWorkspaceScanXmm = -260.0f", header)
        self.assertIn("bool use_fixed_scan_pose_override", srv)
        self.assertIn("float32 fixed_scan_pose_x_mm", srv)
        self.assertIn("bool use_fixed_scan_pose_override", action)
        self.assertIn("float32 fixed_scan_pose_x_mm", action)
        self.assertIn("goal->use_fixed_scan_pose_override", action_bridge)
        self.assertIn("scan_srv.request.fixed_scan_pose_x_mm = goal->fixed_scan_pose_x_mm", action_bridge)
        self.assertIn("req.use_fixed_scan_pose_override", service_orchestration)
        self.assertIn("fixed_scan_pose_override.enabled", suoqu_node)
        self.assertIn("fixed_scan_pose_override.x_mm", suoqu_node)

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

    def test_frontend_visual_trigger_runs_current_frame_no_motion_action_to_overwrite_bind_artifacts(self):
        task_action_controller = (
            WEB_DIR / "frontend" / "src" / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            WEB_DIR / "frontend" / "src" / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")

        trigger_start = task_action_controller.index("async triggerSurfaceDpRecognition")
        trigger_end = task_action_controller.index("\n  async triggerSinglePointBind", trigger_start)
        trigger_body = task_action_controller[trigger_start:trigger_end]
        self.assertIn("startPseudoSlamScanActionClient", trigger_body)
        self.assertIn("enable_capture_gate: false", trigger_body)
        self.assertIn("scan_strategy: 3", trigger_body)
        self.assertIn("pseudo_slam_points.json", trigger_body)
        self.assertIn("pseudo_slam_bind_path.json", trigger_body)
        self.assertNotIn("固定识别位姿", trigger_body)
        self.assertNotIn("callProcessImageService", trigger_body)

        run_saved_start = app_logic.index("runSavedS2:")
        run_saved_end = app_logic.index("\n      triggerSingleBind:", run_saved_start)
        run_saved_body = app_logic[run_saved_start:run_saved_end]
        self.assertIn("startPseudoSlamScanActionClient", run_saved_body)
        self.assertNotIn("processImageService", run_saved_body)

    def test_current_frame_no_motion_strategy_does_not_move_cabin(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        header = (PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp").read_text(encoding="utf-8")

        self.assertIn("kCurrentFrameNoMotion = 3", header)
        self.assertIn("case 3:", suoqu_node)
        self.assertIn("PseudoSlamScanStrategy::kCurrentFrameNoMotion", suoqu_node)

        current_frame_start = suoqu_node.index("case PseudoSlamScanStrategy::kCurrentFrameNoMotion")
        current_frame_end = suoqu_node.index("case PseudoSlamScanStrategy::kFixedManualWorkspace", current_frame_start)
        current_frame_body = suoqu_node[current_frame_start:current_frame_end]
        self.assertIn("当前画面无运动视觉记录", current_frame_body)
        self.assertIn("kProcessImageModeScanOnly", current_frame_body)
        self.assertIn("write_pseudo_slam_points_json", suoqu_node)
        self.assertNotIn("move_cabin_pose_via_driver", current_frame_body)
        self.assertNotIn("wait_cabin_axis_arrival", current_frame_body)
        self.assertNotIn("TCP_Move[1]", current_frame_body)
        self.assertNotIn("cabin_height = current_state.Z", current_frame_body)

    def test_current_frame_no_motion_triggers_scan_vision_before_cabin_state_guard(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        current_frame_start = suoqu_node.index("case PseudoSlamScanStrategy::kCurrentFrameNoMotion")
        current_frame_end = suoqu_node.index("case PseudoSlamScanStrategy::kFixedManualWorkspace", current_frame_start)
        current_frame_body = suoqu_node[current_frame_start:current_frame_end]

        scan_request_index = current_frame_body.index("AI_client.call(scan_srv)")
        stale_state_guard_index = current_frame_body.index("state_age_sec < 0.0 || state_age_sec > 2.0")
        moving_guard_index = current_frame_body.index("current_state.motion_status != 0")
        invalid_position_guard_index = current_frame_body.index("!std::isfinite(current_state.X)")

        self.assertLess(scan_request_index, stale_state_guard_index)
        self.assertLess(scan_request_index, moving_guard_index)
        self.assertLess(scan_request_index, invalid_position_guard_index)

    def test_bind_path_uses_raw_scan_checkerboard_membership_before_planning_filters(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        initial_checkerboard_index = suoqu_node.index(
            "std::unordered_map<int, PseudoSlamCheckerboardInfo> merged_checkerboard_info_by_idx"
        )
        bind_path_info_index = suoqu_node.index(
            "const std::unordered_map<int, PseudoSlamCheckerboardInfo> bind_path_checkerboard_info_by_idx",
            initial_checkerboard_index,
        )
        sync_index = suoqu_node.index(
            "merged_checkerboard_info_by_idx = sync_merged_checkerboard_membership_with_planning",
            bind_path_info_index,
        )
        bind_path_points_index = suoqu_node.index(
            "std::vector<tie_robot_msgs::PointCoords> bind_path_world_points",
            bind_path_info_index,
        )

        self.assertLess(bind_path_info_index, bind_path_points_index)
        self.assertLess(bind_path_info_index, sync_index)
        self.assertIn(
            "const std::unordered_map<int, PseudoSlamCheckerboardInfo> bind_path_checkerboard_info_by_idx =\n"
            "        merged_checkerboard_info_by_idx;",
            suoqu_node[initial_checkerboard_index:sync_index],
        )

    def test_bind_path_falls_back_to_merged_points_when_checkerboard_membership_is_empty(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        empty_guard_index = suoqu_node.index("if (bind_path_world_points.empty()) {")
        fallback_index = suoqu_node.index(
            "bind_path_world_points = merged_world_points;",
            empty_guard_index,
        )
        planner_call_index = suoqu_node.index(
            "build_dynamic_bind_area_entries_from_scan_world(",
            fallback_index,
        )

        self.assertLess(empty_guard_index, fallback_index)
        self.assertLess(fallback_index, planner_call_index)


if __name__ == "__main__":
    unittest.main()
