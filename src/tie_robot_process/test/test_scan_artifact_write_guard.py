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

    def test_scan_bind_path_uses_raw_surface_dp_grid_without_planning_outlier_gate(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        bind_store = (PROCESS_DIR / "src" / "suoqu" / "bind_path_store.cpp").read_text(encoding="utf-8")

        start = suoqu_node.index("std::vector<tie_robot_msgs::PointCoords> bind_path_world_points;")
        end = suoqu_node.index("if (bind_path_world_points.empty())", start)
        bind_path_source_block = suoqu_node[start:end]
        self.assertIn("world_point.has_grid_index", bind_path_source_block)
        self.assertIn("bind_path_info.global_row = world_point.global_row;", bind_path_source_block)
        self.assertIn("bind_path_info.global_col = world_point.global_col;", bind_path_source_block)
        self.assertIn("bind_path_info.is_checkerboard_member = true;", bind_path_source_block)
        self.assertNotIn("checkerboard_info_by_idx.find(world_point.idx)", bind_path_source_block)
        self.assertNotIn("is_planning_outlier", bind_path_source_block)

        self.assertNotIn('"is_planning_checkerboard_member"', bind_store)
        self.assertNotIn('"is_planning_outlier"', bind_store)

    def test_legacy_scan_service_defaults_to_fixed_recognition_pose(self):
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")

        start_index = service_orchestration.index("bool startPseudoSlamScan(")
        end_index = service_orchestration.index("\nbool startPseudoSlamScanWithOptions", start_index)
        start_service_body = service_orchestration[start_index:end_index]
        self.assertIn("PseudoSlamScanStrategy::kFixedManualWorkspace", start_service_body)
        self.assertNotIn("PseudoSlamScanStrategy::kSingleCenter", start_service_body)

    def test_clear_bind_points_exposes_marker_deleteall_service(self):
        header = (PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp").read_text(encoding="utf-8")
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        web_server = (
            WEB_DIR / "scripts" / "workspace_picker_web_server.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "bool clearPseudoSlamMarkersService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);",
            header,
        )
        self.assertIn("bool clearPseudoSlamMarkersService", service_orchestration)
        self.assertIn("clear_pseudo_slam_markers();", service_orchestration)
        self.assertIn(
            'nh.advertiseService("/cabin/clear_pseudo_slam_markers", clearPseudoSlamMarkersService)',
            suoqu_node,
        )
        self.assertIn('CLEAR_PSEUDO_SLAM_MARKERS_SERVICE = "/cabin/clear_pseudo_slam_markers"', web_server)
        self.assertIn("clear_pseudo_slam_markers_service()", web_server)

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

    def test_scan_action_carries_visual_debug_bind_group_point_count(self):
        srv = (WORKSPACE_ROOT / "tie_robot_msgs" / "srv" / "StartPseudoSlamScan.srv").read_text(encoding="utf-8")
        action = (
            WORKSPACE_ROOT / "tie_robot_msgs" / "action" / "StartPseudoSlamScanTask.action"
        ).read_text(encoding="utf-8")
        action_bridge = (
            WORKSPACE_ROOT / "tie_robot_web" / "src" / "web_bridge" / "action_bridge.cpp"
        ).read_text(encoding="utf-8")
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("uint16 bind_group_point_count", srv)
        self.assertIn("float32 bind_execution_cabin_min_z_mm", srv)
        self.assertIn("uint8 BIND_EXECUTION_CABIN_Z_MODE_FIXED=0", srv)
        self.assertIn("uint8 BIND_EXECUTION_CABIN_Z_MODE_MIN=1", srv)
        self.assertIn("uint8 bind_execution_cabin_z_mode", srv)
        self.assertIn("uint16 bind_group_point_count", action)
        self.assertIn("float32 bind_execution_cabin_min_z_mm", action)
        self.assertIn("uint8 BIND_EXECUTION_CABIN_Z_MODE_FIXED=0", action)
        self.assertIn("uint8 BIND_EXECUTION_CABIN_Z_MODE_MIN=1", action)
        self.assertIn("uint8 bind_execution_cabin_z_mode", action)
        self.assertIn("scan_srv.request.bind_group_point_count = goal->bind_group_point_count", action_bridge)
        self.assertIn("scan_srv.request.bind_execution_cabin_min_z_mm = goal->bind_execution_cabin_min_z_mm", action_bridge)
        self.assertIn("scan_srv.request.bind_execution_cabin_z_mode = goal->bind_execution_cabin_z_mode", action_bridge)
        self.assertIn("req.bind_group_point_count", service_orchestration)
        self.assertIn("req.bind_execution_cabin_min_z_mm", service_orchestration)
        self.assertIn("req.bind_execution_cabin_z_mode", service_orchestration)
        self.assertIn("requested_group_point_count", suoqu_node)
        self.assertIn("requested_bind_execution_cabin_min_z_mm", suoqu_node)
        self.assertIn("requested_bind_execution_cabin_z_mode", suoqu_node)
        self.assertIn("normalize_bind_execution_cabin_min_z_mm", suoqu_node)
        self.assertIn("normalize_bind_execution_cabin_z_mode", suoqu_node)
        self.assertIn("is_adaptive_bind_grouping_requested", suoqu_node)
        self.assertIn("config.adaptive_grouping_enabled = adaptive_grouping_enabled;", suoqu_node)
        self.assertIn("adaptive_bind_grouping", suoqu_node)
        self.assertIn("自适应", suoqu_node)
        self.assertIn("无法规划", suoqu_node)

    def test_scan_action_carries_visual_debug_grouping_axis_thresholds(self):
        srv = (WORKSPACE_ROOT / "tie_robot_msgs" / "srv" / "StartPseudoSlamScan.srv").read_text(encoding="utf-8")
        action = (
            WORKSPACE_ROOT / "tie_robot_msgs" / "action" / "StartPseudoSlamScanTask.action"
        ).read_text(encoding="utf-8")
        action_bridge = (
            WORKSPACE_ROOT / "tie_robot_web" / "src" / "web_bridge" / "action_bridge.cpp"
        ).read_text(encoding="utf-8")
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")
        header = (PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp").read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("float32 bind_group_row_threshold_mm", srv)
        self.assertIn("float32 bind_group_column_threshold_mm", srv)
        self.assertIn("float32 bind_group_row_threshold_mm", action)
        self.assertIn("float32 bind_group_column_threshold_mm", action)
        self.assertIn("scan_srv.request.bind_group_row_threshold_mm = goal->bind_group_row_threshold_mm", action_bridge)
        self.assertIn("scan_srv.request.bind_group_column_threshold_mm = goal->bind_group_column_threshold_mm", action_bridge)
        self.assertIn("req.bind_group_row_threshold_mm", service_orchestration)
        self.assertIn("req.bind_group_column_threshold_mm", service_orchestration)
        self.assertIn("kDynamicBindMatrixRowThresholdMm = 40.0f", header)
        self.assertIn("kDynamicBindMatrixColumnThresholdMm = 45.0f", header)
        self.assertIn("normalize_bind_group_axis_threshold_mm", suoqu_node)
        self.assertIn("requested_bind_group_row_threshold_mm", suoqu_node)
        self.assertIn("requested_bind_group_column_threshold_mm", suoqu_node)
        self.assertIn("config.matrix_row_threshold_mm =", suoqu_node)
        self.assertIn("config.matrix_column_threshold_mm =", suoqu_node)

    def test_replan_service_logs_received_axis_thresholds(self):
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")

        start = service_orchestration.index("bool replanPseudoSlamBindPath(")
        end = service_orchestration.index("\nbool bind_current_area_from_scan_service", start)
        body = service_orchestration[start:end]
        self.assertIn("Cabin_log: 收到阈值重规划请求", body)
        self.assertIn("req.bind_group_row_threshold_mm", body)
        self.assertIn("req.bind_group_column_threshold_mm", body)
        self.assertIn("req.recognition_pose_index", body)

    def test_scan_action_carries_recognition_pose_index_and_artifacts_group_by_pose(self):
        srv = (WORKSPACE_ROOT / "tie_robot_msgs" / "srv" / "StartPseudoSlamScan.srv").read_text(encoding="utf-8")
        action = (
            WORKSPACE_ROOT / "tie_robot_msgs" / "action" / "StartPseudoSlamScanTask.action"
        ).read_text(encoding="utf-8")
        action_bridge = (
            WORKSPACE_ROOT / "tie_robot_web" / "src" / "web_bridge" / "action_bridge.cpp"
        ).read_text(encoding="utf-8")
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")
        header = (PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp").read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        bind_store = (PROCESS_DIR / "src" / "suoqu" / "bind_path_store.cpp").read_text(encoding="utf-8")
        memory_store = (PROCESS_DIR / "src" / "suoqu" / "execution_memory_store.cpp").read_text(encoding="utf-8")

        self.assertIn("uint16 recognition_pose_index", srv)
        self.assertIn("uint16 recognition_pose_index", action)
        self.assertIn("scan_srv.request.recognition_pose_index = goal->recognition_pose_index", action_bridge)
        self.assertIn("req.recognition_pose_index", service_orchestration)
        self.assertIn("int recognition_pose_index = 1", header)
        self.assertIn("normalize_recognition_pose_index", suoqu_node)
        self.assertIn("recognition_pose_index", suoqu_node)
        self.assertIn("recognition_pose_index", bind_store)
        self.assertIn('"scan_pose_groups"', bind_store)
        self.assertIn('"scan_pose_groups_by_pose_index"', bind_store)
        self.assertIn('"pose_index"', bind_store)
        self.assertIn('"pose_local_global_idx"', bind_store)
        self.assertIn("replace_scan_pose_group_by_index", bind_store)
        self.assertIn("flatten_scan_pose_groups", bind_store)
        self.assertIn("point_record.recognition_pose_index", memory_store)
        self.assertIn('"recognition_pose_index"', memory_store)

    def test_scan_artifacts_and_execution_memory_carry_jump_bind_color_metadata(self):
        bind_store = (PROCESS_DIR / "src" / "suoqu" / "bind_path_store.cpp").read_text(encoding="utf-8")
        area_execution = (PROCESS_DIR / "src" / "suoqu" / "area_execution.cpp").read_text(encoding="utf-8")
        memory_store = (PROCESS_DIR / "src" / "suoqu" / "execution_memory_store.cpp").read_text(encoding="utf-8")
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        scan_processing = (
            PROCESS_DIR / "src" / "suoqu" / "pseudo_slam_scan_processing.cpp"
        ).read_text(encoding="utf-8")
        web_server = (
            WEB_DIR / "scripts" / "workspace_picker_web_server.py"
        ).read_text(encoding="utf-8")

        self.assertIn("bool jump_bind", runtime_header)
        self.assertIn("std::string checkerboard_color", runtime_header)
        self.assertIn("point_json_is_jump_bind_target", runtime_header)
        self.assertIn("point_json_matches_jump_bind_parity", runtime_header)
        self.assertIn('"jump_bind"', bind_store)
        self.assertIn('"checkerboard_color"', bind_store)
        self.assertNotIn('"planning_jump_bind"', bind_store)
        self.assertNotIn('"planning_checkerboard_color"', bind_store)
        self.assertNotIn('"is_planning_outlier"', bind_store)
        self.assertIn("point_json_matches_jump_bind_parity(point_json, selected_jump_bind_parity)", area_execution)
        self.assertNotIn('point_json.value("checkerboard_parity", 0) != 0', area_execution)
        self.assertIn('point_record.jump_bind = point_json.value("jump_bind"', memory_store)
        self.assertIn('point_record.checkerboard_color = point_json.value("checkerboard_color"', memory_store)
        self.assertIn('"jump_bind"', memory_store)
        self.assertIn('"checkerboard_color"', memory_store)
        self.assertIn('"jump_bind"', scan_processing)
        self.assertIn('"checkerboard_color"', scan_processing)
        self.assertIn('"jump_bind"', web_server)
        self.assertIn('"checkerboard_color"', web_server)

    def test_jump_bind_frontend_long_press_toggles_and_click_selects_checkerboard_color(self):
        control_catalog = (
            WEB_DIR / "frontend" / "src" / "config" / "controlPanelCatalog.js"
        ).read_text(encoding="utf-8")
        legacy_catalog = (
            WEB_DIR / "frontend" / "src" / "config" / "legacyCommandCatalog.js"
        ).read_text(encoding="utf-8")
        topic_registry = (
            WEB_DIR / "frontend" / "src" / "config" / "topicRegistry.js"
        ).read_text(encoding="utf-8")
        controller = (
            WEB_DIR / "frontend" / "src" / "controllers" / "LegacyCommandController.js"
        ).read_text(encoding="utf-8")
        app = (WEB_DIR / "frontend" / "src" / "app" / "TieRobotFrontApp.js").read_text(encoding="utf-8")
        scene3d = (WEB_DIR / "frontend" / "src" / "views" / "Scene3DView.js").read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        area_execution = (PROCESS_DIR / "src" / "suoqu" / "area_execution.cpp").read_text(encoding="utf-8")

        jump_start = control_catalog.index("jumpBindEnabled:")
        jump_end = control_catalog.index("\n  lightEnabled:", jump_start)
        jump_body = control_catalog[jump_start:jump_end]
        self.assertIn('singleClickAction: "cycleSelectedParity"', jump_body)
        self.assertIn("selectedParityCommandId: 26", jump_body)
        self.assertIn("longPressTogglesState: true", jump_body)
        self.assertIn("inactiveLongPressCommandId: 12", jump_body)
        self.assertIn("longPressCommandId: 12", jump_body)
        self.assertIn("jumpBindParity", topic_registry)
        self.assertIn("jumpBindEnabled", topic_registry)
        self.assertIn("/web/moduan/jump_bind_parity", topic_registry)
        self.assertIn("/web/moduan/jump_bind_enabled", topic_registry)
        self.assertIn('{ id: 26, name: "切换跳绑黑白棋"', legacy_catalog)
        self.assertIn("handleSelectedParityToggle", controller)
        self.assertIn("publishSelectedParityIfConfigured", controller)
        self.assertIn("setJumpBindVisualizationState", app)
        self.assertIn("buildJumpBindPointPositions", scene3d)
        self.assertIn("this.jumpBindPoints", scene3d)
        self.assertIn("checkerboard_jump_bind_parity_callback", suoqu_node)
        self.assertIn("/web/moduan/jump_bind_parity", suoqu_node)
        self.assertIn("/web/moduan/jump_bind_enabled", suoqu_node)
        self.assertIn("checkerboard_jump_bind_selected_parity.load", suoqu_node)
        self.assertIn("point_json_matches_jump_bind_parity", area_execution)

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

    def test_start_execution_defaults_to_execution_memory_disabled(self):
        task_action_controller = (
            WEB_DIR / "frontend" / "src" / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")
        control_panel_catalog = (
            WEB_DIR / "frontend" / "src" / "config" / "controlPanelCatalog.js"
        ).read_text(encoding="utf-8")
        legacy_ir_picker = (WEB_DIR / "web" / "ir_workspace_picker.mjs").read_text(encoding="utf-8")
        legacy_execution_actions = (
            WEB_DIR / "web" / "modules" / "execution_actions.mjs"
        ).read_text(encoding="utf-8")
        action = (
            WORKSPACE_ROOT / "tie_robot_msgs" / "action" / "StartGlobalWorkTask.action"
        ).read_text(encoding="utf-8")
        srv = (
            WORKSPACE_ROOT / "tie_robot_msgs" / "srv" / "StartGlobalWork.srv"
        ).read_text(encoding="utf-8")
        action_bridge = (
            WEB_DIR / "src" / "web_bridge" / "action_bridge.cpp"
        ).read_text(encoding="utf-8")
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("bool use_execution_memory", action)
        self.assertIn("bool use_execution_memory", srv)
        self.assertIn('case "startExecution":', task_action_controller)
        self.assertIn(
            "return this.triggerExecutionLayer({ useExecutionMemory: false, clearExecutionMemory: false });",
            task_action_controller,
        )
        self.assertIn(
            "return this.triggerExecutionLayer({ useExecutionMemory: true, clearExecutionMemory: false });",
            task_action_controller,
        )
        self.assertIn("use_execution_memory: useExecutionMemory", task_action_controller)
        self.assertIn('{ id: "startExecutionKeepMemory", label: "记忆续跑\\n开始", tone: "amber" }', control_panel_catalog)
        self.assertIn(
            "startExecutionBtn.addEventListener(\"click\", () => triggerExecutionLayer({ useExecutionMemory: false, clearExecutionMemory: false }));",
            legacy_ir_picker,
        )
        self.assertIn(
            "startExecutionClearMemoryBtn.addEventListener(\"click\", () => triggerExecutionLayer({ useExecutionMemory: true, clearExecutionMemory: false }));",
            legacy_ir_picker,
        )
        self.assertIn("use_execution_memory: useExecutionMemory", legacy_execution_actions)
        self.assertIn("start_work_srv.request.use_execution_memory = goal->use_execution_memory", action_bridge)
        self.assertIn("if (req.clear_execution_memory && req.use_execution_memory)", service_orchestration)
        self.assertIn("run_global_work_with_execution_memory_mode(", service_orchestration)
        self.assertIn("bool run_bind_from_scan(std::string& message, bool use_execution_memory)", suoqu_node)
        self.assertIn("执行记忆关闭", suoqu_node)

    def test_global_execution_mode_selects_ledger_refine_or_pure_refine_without_bind_path_short_circuit(self):
        service_orchestration = (
            PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")
        header = (PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp").read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        set_mode_srv = (
            WORKSPACE_ROOT / "tie_robot_msgs" / "srv" / "SetExecutionMode.srv"
        ).read_text(encoding="utf-8")
        start_work_action = (
            WORKSPACE_ROOT / "tie_robot_msgs" / "action" / "StartGlobalWorkTask.action"
        ).read_text(encoding="utf-8")

        self.assertIn("uint8 MODE_SLAM_PRECOMPUTED=0", set_mode_srv)
        self.assertIn("uint8 MODE_LEDGER_WITH_REFINE=1", set_mode_srv)
        self.assertIn("uint8 MODE_PLANNED_PATH_REFINE_ONLY=2", set_mode_srv)
        self.assertIn("uint8 MODE_LEDGER_WITH_REFINE=1", start_work_action)
        self.assertIn("uint8 MODE_PLANNED_PATH_REFINE_ONLY=2", start_work_action)
        self.assertIn("kLedgerWithRefine = 1", header)
        self.assertIn("kPlannedPathRefineOnly = 2", header)
        self.assertIn("bool run_planned_path_refine_only_global_work(std::string& message", header)

        start = service_orchestration.index("bool run_global_work_with_execution_memory_mode(")
        end = service_orchestration.index("\nbool startGlobalWork(", start)
        body = service_orchestration[start:end]
        self.assertIn("case GlobalExecutionMode::kSlamPrecomputed", body)
        self.assertIn("run_bind_from_scan(res.message, use_execution_memory)", body)
        self.assertIn("case GlobalExecutionMode::kLedgerWithRefine", body)
        self.assertIn("run_live_visual_global_work(res.message, use_execution_memory)", body)
        self.assertIn("case GlobalExecutionMode::kPlannedPathRefineOnly", body)
        self.assertIn("run_planned_path_refine_only_global_work(res.message, use_execution_memory)", body)
        bind_path_check_index = body.index("std::ifstream scan_file(pseudo_slam_bind_path_json_file)")
        mode_switch_index = body.index("switch (execution_mode)")
        self.assertLess(bind_path_check_index, mode_switch_index)
        self.assertNotIn("优先按预生成路径执行", body)

        pure_start = suoqu_node.index("bool run_planned_path_refine_only_global_work(")
        pure_end = suoqu_node.index("\nbool run_bind_from_scan(", pure_start)
        pure_body = suoqu_node[pure_start:pure_end]
        self.assertIn("call_sg_live_visual_with_no_points_retry", pure_body)
        self.assertIn("sg_live_visual_client.call", suoqu_node)
        self.assertIn("/moduan/sg", pure_body)
        self.assertIn("planned_path_refine_only", pure_body)
        self.assertIn("jump_bind_enabled_snapshot", pure_body)
        self.assertIn("checkerboard_jump_bind_enabled.load", pure_body)
        self.assertIn("build_refined_execution_points_from_nearest_area_ledger_points", pure_body)
        self.assertNotIn("build_refined_execution_points_from_area_quadrants", pure_body)
        self.assertIn("transform_scepter_camera_points_to_gripper_points", pure_body)
        self.assertIn("filter_precomputed_group_points_for_execution", pure_body)
        self.assertIn("load_precomputed_local_points_from_group_json", pure_body)

    def test_planned_path_refine_only_jump_bind_uses_nearest_area_ledger_correction(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn(
            "nlohmann::json build_refined_execution_points_from_nearest_area_ledger_points(",
            suoqu_node,
        )
        helper_start = suoqu_node.index(
            "nlohmann::json build_refined_execution_points_from_nearest_area_ledger_points("
        )
        helper_end = suoqu_node.index("\nbool load_precomputed_local_points_from_group_json", helper_start)
        helper_body = suoqu_node[helper_start:helper_end]

        self.assertIn("PlannedAreaNearestPointReference", helper_body)
        self.assertIn("point_distance_mm(live_world_point, planned_point.world_point)", helper_body)
        self.assertIn("used_planned_global_indices", helper_body)
        self.assertIn("used_live_indices", helper_body)
        self.assertIn('execution_point_json["nearest_ledger_distance_mm"]', helper_body)
        self.assertIn('execution_point_json["world_x"] = live_world_point.World_coord[0];', helper_body)
        self.assertIn('execution_point_json["world_y"] = live_world_point.World_coord[1];', helper_body)
        self.assertIn('execution_point_json["world_z"] = live_world_point.World_coord[2];', helper_body)
        self.assertIn('execution_point_json["x"] = live_gripper_point.World_coord[0];', helper_body)
        self.assertIn('execution_point_json["y"] = live_gripper_point.World_coord[1];', helper_body)
        self.assertIn('execution_point_json["z"] = live_gripper_point.World_coord[2];', helper_body)
        self.assertNotIn("classify_live_visual_point_into_checkerboard", helper_body)
        self.assertNotIn("quadrant_key_for_world_point", helper_body)
        self.assertNotIn("live_point_by_quadrant", helper_body)
        self.assertNotIn("planned_point_by_quadrant", helper_body)

        pure_start = suoqu_node.index("bool run_planned_path_refine_only_global_work(")
        pure_end = suoqu_node.index("\nbool run_bind_from_scan(", pure_start)
        pure_body = suoqu_node[pure_start:pure_end]
        self.assertIn("kProcessImageModeExecutionRefine", pure_body)
        self.assertIn("build_world_point_from_scan_response", pure_body)
        self.assertIn("transform_scepter_camera_points_to_gripper_points", pure_body)
        self.assertIn("jump_bind_enabled_snapshot", pure_body)
        self.assertIn("selected_jump_bind_parity_snapshot", pure_body)

    def test_planned_path_refine_only_jump_bind_does_not_require_area_quadrants(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertNotIn("build_refined_execution_points_from_area_quadrants", suoqu_node)
        self.assertNotIn("quadrant_key_for_world_point", suoqu_node)
        self.assertNotIn("同区域四宫格", suoqu_node)
        self.assertNotIn("当前账本区域边界外", suoqu_node)

        helper_start = suoqu_node.index(
            "nlohmann::json build_refined_execution_points_from_nearest_area_ledger_points("
        )
        helper_end = suoqu_node.index("\nbool load_precomputed_local_points_from_group_json", helper_start)
        helper_body = suoqu_node[helper_start:helper_end]

        self.assertIn("当前账本区域欧式最近点", helper_body)
        self.assertIn("账本点", helper_body)
        self.assertNotIn("kAreaQuadrantCount", helper_body)
        self.assertNotIn("planned_area_margin_mm", helper_body)
        self.assertNotIn("is_world_point_inside_expanded_area_bounds", helper_body)

    def test_planned_path_refine_only_jump_bind_uses_current_area_ledger_points_only(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn(
            "nlohmann::json build_refined_execution_points_from_nearest_area_ledger_points(",
            suoqu_node,
        )
        helper_start = suoqu_node.index(
            "nlohmann::json build_refined_execution_points_from_nearest_area_ledger_points("
        )
        helper_end = suoqu_node.index("\nbool load_precomputed_local_points_from_group_json", helper_start)
        helper_body = suoqu_node[helper_start:helper_end]

        self.assertIn('planned_area_json["groups"]', helper_body)
        self.assertIn('point_json.value("global_idx", -1)', helper_body)
        self.assertIn("planned_points.empty()", helper_body)
        self.assertIn("当前账本区域没有可用于最近点匹配的账本点", helper_body)
        self.assertIn("match_candidates", helper_body)
        self.assertIn("used_planned_global_indices", helper_body)
        self.assertNotIn("areas_json", helper_body)
        self.assertNotIn("build_area_quadrant_bounds", helper_body)

    def test_planned_path_refine_only_skips_current_area_when_execution_refine_two_by_two_not_ready(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        pure_start = suoqu_node.index("bool run_planned_path_refine_only_global_work(")
        pure_end = suoqu_node.index("\nbool run_bind_from_scan(", pure_start)
        pure_body = suoqu_node[pure_start:pure_end]
        jump_start = pure_body.index("nlohmann::json execution_group_json;")
        jump_body = pure_body[jump_start:pure_body.index("std::string bind_action_message;", jump_start)]

        self.assertIn("kProcessImageModeExecutionRefine", jump_body)
        self.assertNotIn("while (ros::ok())", jump_body)
        self.assertNotIn("跳绑微调2x2未就绪", jump_body)
        self.assertNotIn("继续停留当前区域重试", jump_body)
        self.assertNotIn("wait_for_planned_path_settle_duration", jump_body)
        self.assertIn("skipped_area_count++", jump_body)
        self.assertIn("跳过当前区域", jump_body)
        self.assertIn("publish_area_progress", jump_body)

    def test_bind_path_direct_test_uses_bind_path_only_without_outlier_blocking(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = suoqu_node.index("bool run_bind_path_direct_test(")
        end = suoqu_node.index("\nbool run_live_visual_global_work(", start)
        body = suoqu_node[start:end]
        self.assertNotIn("nlohmann::json points_json", body)
        self.assertNotIn("pseudo_slam_points_json_file", body)
        self.assertNotIn("collect_blocked_execution_global_indices_from_points_json", body)
        self.assertIn("no_blocked_global_indices", body)
        self.assertIn("filter_precomputed_group_points_for_execution(", body)
        self.assertIn("checkerboard_jump_bind_enabled", body)

    def test_execution_blocked_indices_do_not_use_outlier_or_planning_flags(self):
        area_execution = (PROCESS_DIR / "src" / "suoqu" / "area_execution.cpp").read_text(encoding="utf-8")

        helper_start = area_execution.index("std::unordered_set<int> collect_blocked_execution_global_indices_from_points_json(")
        helper_body = area_execution[helper_start:]
        self.assertIn("(void)points_json;", helper_body)
        self.assertNotIn("is_planning_outlier", helper_body)
        self.assertNotIn("is_planning_outlier_line_member", helper_body)
        self.assertNotIn("is_outlier_secondary_plane_member", helper_body)
        self.assertNotIn("is_outlier_column_neighbor_blocked", helper_body)
        self.assertNotIn("is_planning_checkerboard_member", helper_body)

    def test_frontend_visual_trigger_runs_current_frame_no_motion_action_to_update_selected_pose_group(self):
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
        self.assertIn("recognition_pose_index: recognitionPoseIndex", trigger_body)
        self.assertIn("识别位姿", trigger_body)
        self.assertIn("保留其他识别位姿数据", trigger_body)
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

    def test_bind_path_prefers_full_surface_dp_grid_before_raw_fallback(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        planning_filter_index = suoqu_node.index(
            "planning_world_points = filter_pseudo_slam_non_checkerboard_points("
        )
        planning_rebuild_index = suoqu_node.index(
            "checkerboard_info_by_idx = build_checkerboard_info_by_global_index(planning_world_points, path_origin);",
            planning_filter_index,
        )
        bind_path_info_index = suoqu_node.index(
            "std::unordered_map<int, PseudoSlamCheckerboardInfo> bind_path_checkerboard_info_by_idx",
            planning_rebuild_index,
        )
        sync_index = suoqu_node.index(
            "merged_checkerboard_info_by_idx = sync_merged_checkerboard_membership_with_planning",
            bind_path_info_index,
        )
        bind_path_points_index = suoqu_node.index(
            "std::vector<tie_robot_msgs::PointCoords> bind_path_world_points",
            bind_path_info_index,
        )

        self.assertLess(planning_rebuild_index, bind_path_info_index)
        self.assertLess(bind_path_info_index, bind_path_points_index)
        self.assertLess(bind_path_info_index, sync_index)
        bind_path_grid_body = suoqu_node[
            bind_path_points_index:suoqu_node.index("if (bind_path_world_points.empty()) {", bind_path_points_index)
        ]
        self.assertIn(
            "bind_path_checkerboard_info_by_idx =\n"
            "        merged_checkerboard_info_by_idx;",
            suoqu_node[planning_rebuild_index:sync_index],
        )
        self.assertIn("world_point.has_grid_index", bind_path_grid_body)
        self.assertIn("bind_path_info.global_row = world_point.global_row;", bind_path_grid_body)
        self.assertIn("bind_path_info.global_col = world_point.global_col;", bind_path_grid_body)
        self.assertIn("bind_path_info.is_checkerboard_member = true;", bind_path_grid_body)
        self.assertIn("grid_index.global_row = world_point.global_row;", bind_path_grid_body)
        self.assertIn("grid_index.global_col = world_point.global_col;", bind_path_grid_body)
        self.assertNotIn("checkerboard_info_by_idx.find(world_point.idx)", bind_path_grid_body)

    def test_surface_dp_grid_indices_are_carried_into_planning_checkerboard(self):
        point_coords_msg = (WORKSPACE_ROOT / "tie_robot_msgs" / "msg" / "PointCoords.msg").read_text(encoding="utf-8")
        self.assertIn("bool has_grid_index", point_coords_msg)
        self.assertIn("int32 global_row", point_coords_msg)
        self.assertIn("int32 global_col", point_coords_msg)

        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        build_world_point_index = suoqu_node.index("bool build_world_point_from_scan_response(")
        build_world_point_body = suoqu_node[
            build_world_point_index:suoqu_node.index("void assign_global_indices", build_world_point_index)
        ]
        self.assertIn("world_point.has_grid_index = point.has_grid_index;", build_world_point_body)
        self.assertIn("world_point.global_row = point.global_row;", build_world_point_body)
        self.assertIn("world_point.global_col = point.global_col;", build_world_point_body)

        scan_processing = (PROCESS_DIR / "src" / "suoqu" / "pseudo_slam_scan_processing.cpp").read_text(encoding="utf-8")
        checkerboard_builder_index = scan_processing.index("build_checkerboard_info_by_global_index(")
        checkerboard_builder_body = scan_processing[
            checkerboard_builder_index:scan_processing.index("sync_merged_checkerboard_membership_with_planning", checkerboard_builder_index)
        ]
        self.assertIn("has_explicit_grid_index", checkerboard_builder_body)
        self.assertLess(
            checkerboard_builder_body.index("has_explicit_grid_index"),
            checkerboard_builder_body.index("cluster_checkerboard_axis_centers("),
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
