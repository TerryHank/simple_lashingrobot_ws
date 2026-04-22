#!/usr/bin/env python3

import importlib.util
import errno
import socket
import unittest
from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
TIE_ROBOT_BRINGUP_DIR = WORKSPACE_SRC / "tie_robot_bringup"
TIE_ROBOT_WEB_DIR = WORKSPACE_SRC / "tie_robot_web"
TIE_ROBOT_PERCEPTION_DIR = WORKSPACE_SRC / "tie_robot_perception"
FRONTEND_DIR = TIE_ROBOT_WEB_DIR / "web"
SERVER_SCRIPT = TIE_ROBOT_WEB_DIR / "scripts" / "workspace_picker_web_server.py"
OPEN_SCRIPT = TIE_ROBOT_WEB_DIR / "scripts" / "workspace_picker_web_open.py"
FRONTEND_SRC_DIR = TIE_ROBOT_WEB_DIR / "frontend" / "src"
LEGACY_COMMAND_CATALOG = FRONTEND_SRC_DIR / "config" / "legacyCommandCatalog.js"
CONTROL_PANEL_CATALOG = FRONTEND_SRC_DIR / "config" / "controlPanelCatalog.js"
VIEWER_STORE = FRONTEND_SRC_DIR / "state" / "ViewerStore.js"
SCENE_ADAPTER = FRONTEND_SRC_DIR / "data" / "SceneAdapter.js"
PANEL_REGISTRY = FRONTEND_SRC_DIR / "panels" / "panelRegistry.js"
DEFAULT_LAYOUTS = FRONTEND_SRC_DIR / "layout" / "defaultLayouts.js"
LAYOUT_MANAGER = FRONTEND_SRC_DIR / "layout" / "LayoutManager.js"


def load_module(module_name, script_path):
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class WorkspacePickerWebTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server_module = load_module(
            "tie_robot_web_server_runtime_test",
            SERVER_SCRIPT,
        )
        cls.open_module = load_module(
            "tie_robot_web_open_runtime_test",
            OPEN_SCRIPT,
        )

    def test_frontend_and_bridge_assets_exist(self):
        self.assertTrue((FRONTEND_DIR / "index.html").exists())
        self.assertTrue((FRONTEND_DIR / "ir_workspace_picker.mjs").exists())
        self.assertTrue((FRONTEND_DIR / "ir_workspace_picker_helpers.mjs").exists())
        self.assertTrue(any(FRONTEND_DIR.glob("vendor/roslib-*.js")))
        self.assertTrue(any(FRONTEND_DIR.glob("vendor/rolldown-runtime-*.js")))

    def test_run_launch_starts_api_stack_and_workspace_picker_web(self):
        launch_text = (TIE_ROBOT_BRINGUP_DIR / "launch" / "run.launch").read_text(
            encoding="utf-8"
        )

        self.assertIn('<arg name="include_api_stack" default="true"', launch_text)
        self.assertIn('<arg name="workspace_picker_port" default="8080"', launch_text)
        self.assertIn('<include file="$(find tie_robot_bringup)/launch/api.launch"', launch_text)
        self.assertIn('name="workspace_picker_web_server"', launch_text)
        self.assertIn('type="workspace_picker_web_server.py"', launch_text)
        self.assertIn('name="workspace_picker_web_open"', launch_text)
        self.assertIn('type="workspace_picker_web_open.py"', launch_text)

    def test_workspace_picker_web_scripts_reference_new_package_layout(self):
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        open_script = OPEN_SCRIPT.read_text(encoding="utf-8")
        picker_script = (FRONTEND_DIR / "ir_workspace_picker.mjs").read_text(
            encoding="utf-8"
        )
        ros_connection_controller = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        ros_connection_script = (FRONTEND_DIR / "modules" / "ros_connection.mjs").read_text(
            encoding="utf-8"
        )
        execution_actions_script = (FRONTEND_DIR / "modules" / "execution_actions.mjs").read_text(
            encoding="utf-8"
        )
        picker_html = (FRONTEND_DIR / "index.html").read_text(encoding="utf-8")

        self.assertIn('Path(__file__).resolve().parents[1] / "web"', server_script)
        self.assertIn("DEFAULT_WORKSPACE_PICKER_PORT = 8080", server_script)
        self.assertIn("SimpleHTTPRequestHandler", server_script)
        self.assertIn("DEFAULT_WORKSPACE_PICKER_PORT = 8080", open_script)
        self.assertIn("http://127.0.0.1", open_script)
        self.assertIn("tie_robot_web/web/index.html", open_script)
        self.assertIn('connectRosbridge()', picker_script)
        self.assertIn("new ROSLIB.ActionClient(", ros_connection_controller)
        self.assertIn("new ROSLIB.Service(", ros_connection_controller)
        self.assertIn("START_PSEUDO_SLAM_SCAN_ACTION_TYPE", ros_connection_controller)
        self.assertIn("START_GLOBAL_WORK_ACTION_TYPE", ros_connection_controller)
        self.assertIn("RUN_DIRECT_BIND_PATH_TEST_ACTION_TYPE", ros_connection_controller)
        self.assertIn("SET_EXECUTION_MODE_SERVICE_TYPE", ros_connection_controller)
        self.assertIn('const SYSTEM_LOG_TOPIC = "/system_log/all";', ros_connection_controller)
        self.assertIn('messageType: "rosgraph_msgs/Log"', ros_connection_controller)
        self.assertIn("triggerWorkspaceCenterScanPoseMove", execution_actions_script)
        self.assertIn("triggerExecutionLayer", execution_actions_script)
        self.assertIn("triggerDirectBindPathTest", execution_actions_script)
        self.assertIn("工作区覆盖层已更新，当前统一显示 /pointAI/result_image_raw。", app_logic)
        self.assertIn("setExecutionOverlayMessage", task_action_controller)
        self.assertIn('<div id="app"></div>', picker_html)
        self.assertIn("绑扎机器人控制台", picker_html)
        self.assertIn("./assets/app/", picker_html)

    def test_viewer_architecture_scaffold_exists(self):
        self.assertTrue(VIEWER_STORE.exists())
        self.assertTrue(SCENE_ADAPTER.exists())
        self.assertTrue(PANEL_REGISTRY.exists())
        self.assertTrue(DEFAULT_LAYOUTS.exists())
        self.assertTrue(LAYOUT_MANAGER.exists())

        viewer_store = VIEWER_STORE.read_text(encoding="utf-8")
        scene_adapter = SCENE_ADAPTER.read_text(encoding="utf-8")
        panel_registry = PANEL_REGISTRY.read_text(encoding="utf-8")
        default_layouts = DEFAULT_LAYOUTS.read_text(encoding="utf-8")
        layout_manager = LAYOUT_MANAGER.read_text(encoding="utf-8")
        storage_utils = (FRONTEND_SRC_DIR / "utils" / "storage.js").read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")

        self.assertIn("export class ViewerStore", viewer_store)
        self.assertIn("getState()", viewer_store)
        self.assertIn("updateIn(key, nextValue)", viewer_store)
        self.assertIn("export class SceneAdapter", scene_adapter)
        self.assertIn("normalizePointCloud(message, context = {})", scene_adapter)
        self.assertIn("normalizeTiePoints(message, context = {})", scene_adapter)
        self.assertIn("normalizePlanningMarkers(message, context = {})", scene_adapter)
        self.assertIn("export const PANEL_REGISTRY = [", panel_registry)
        self.assertIn('id: "controlPanel"', panel_registry)
        self.assertIn('id: "workspacePanel"', panel_registry)
        self.assertIn("executionDebug", default_layouts)
        self.assertIn("visionDebug", default_layouts)
        self.assertIn("export class LayoutManager", layout_manager)
        self.assertIn("getLayout(layoutId)", layout_manager)
        self.assertIn("persistLayout(layoutId, layout)", layout_manager)
        self.assertIn("export const VIEWER_LAYOUT_PREFIX", storage_utils)
        self.assertIn("loadViewerLayout(layoutId)", storage_utils)
        self.assertIn("saveViewerLayout(layoutId, value)", storage_utils)
        self.assertIn("new ViewerStore()", app_logic)
        self.assertIn("new SceneAdapter()", app_logic)
        self.assertIn("new LayoutManager({", app_logic)
        self.assertIn("PANEL_REGISTRY", app_logic)
        self.assertIn("DEFAULT_LAYOUTS", app_logic)

    def test_workspace_overlay_chain_uses_result_img_only(self):
        ros_connection_controller = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        workspace_view = (
            FRONTEND_SRC_DIR / "views" / "WorkspaceCanvasView.js"
        ).read_text(encoding="utf-8")
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        rendering_py = (
            TIE_ROBOT_PERCEPTION_DIR
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "rendering.py"
        ).read_text(encoding="utf-8")

        self.assertNotIn("WORKSPACE_S2_RESULT_TOPIC", ros_connection_controller)
        self.assertIn('const EXECUTION_RESULT_TOPIC = "/pointAI/result_image_raw";', ros_connection_controller)
        self.assertNotIn("lastS2ResultMessage", workspace_view)
        self.assertNotIn('this.overlaySource = "s2";', workspace_view)
        self.assertIn("this.setExecutionOverlayMessage(message);", workspace_view)
        self.assertIn("覆盖层统一等待 result_img", task_action_controller)
        self.assertIn("工作区覆盖层已更新，当前统一显示 /pointAI/result_image_raw。", app_logic)
        self.assertIn("self.result_image_raw_pub.publish(result_image_msg)", rendering_py)

    def test_frontend_defaults_camera_follow_to_disabled(self):
        topic_layer_catalog = (
            FRONTEND_SRC_DIR / "config" / "topicLayerCatalog.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")
        legacy_command_catalog = LEGACY_COMMAND_CATALOG.read_text(encoding="utf-8")

        self.assertIn("followCamera: false", topic_layer_catalog)
        self.assertNotIn('data-toolbar-action="toggle-follow-camera"', ui_controller)
        self.assertIn('<input id="followCameraToggle" type="checkbox" />', ui_controller)
        self.assertIn("this.followCamera = false;", scene_view)

    def test_frontend_visible_text_and_scene_placeholders_are_localized_and_sized(self):
        topic_layer_catalog = (
            FRONTEND_SRC_DIR / "config" / "topicLayerCatalog.js"
        ).read_text(encoding="utf-8")
        control_panel_catalog = CONTROL_PANEL_CATALOG.read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")
        legacy_command_catalog = LEGACY_COMMAND_CATALOG.read_text(encoding="utf-8")

        self.assertIn('{ id: "filteredWorldCoord", label: "滤波世界点云" }', topic_layer_catalog)
        self.assertIn('{ id: "rawWorldCoord", label: "原始世界点云" }', topic_layer_catalog)
        self.assertIn('<div class="toolbar-brand">绑扎机器人</div>', ui_controller)
        self.assertNotIn("toolbar-brand-subtitle", ui_controller)
        self.assertIn('{ id: "controlPanel", label: "控制面板" }', ui_controller)
        self.assertIn('{ id: "workspacePanel", label: "工作区" }', ui_controller)
        self.assertIn('{ id: "topicLayersPanel", label: "话题图层" }', ui_controller)
        self.assertIn('{ id: "logPanel", label: "日志" }', ui_controller)
        self.assertNotIn('{ id: "processPanel", label: "任务" }', ui_controller)
        self.assertNotIn('{ id: "legacyPanel", label: "命令" }', ui_controller)
        self.assertIn('id="controlPanel" class="floating-panel panel-control"', ui_controller)
        self.assertIn('id="controlPanelTaskGrid" class="control-button-grid"', ui_controller)
        self.assertIn('id="controlPanelGroups" class="control-group-stack"', ui_controller)
        self.assertIn('this.refs.taskButtons = [...this.refs.controlPanelTaskGrid.querySelectorAll("[data-task-action]")];', ui_controller)
        self.assertIn('data-control-toggle="${control.id}"', ui_controller)
        self.assertIn("onControlToggle(callback)", ui_controller)
        self.assertIn("setControlToggleState(toggleId, state)", ui_controller)
        self.assertNotIn('id="parameterFields"', ui_controller)
        self.assertNotIn("renderParameterFields()", ui_controller)
        self.assertIn("提交四边形\\n触发 S2", control_panel_catalog)
        self.assertIn("开始\\n执行层", control_panel_catalog)
        self.assertIn("账本\\n测试", control_panel_catalog)
        self.assertIn('id: "pauseResume"', control_panel_catalog)
        self.assertIn('id: "lashingEnabled"', control_panel_catalog)
        self.assertIn('id: "jumpBindEnabled"', control_panel_catalog)
        self.assertIn('id: "lightEnabled"', control_panel_catalog)
        self.assertNotIn('name: "暂停作业"', ui_controller)
        self.assertIn('class="toolbar-group toolbar-status-group"', ui_controller)
        self.assertIn('id="connectionBadge" class="toolbar-connection-badge info"', ui_controller)
        self.assertIn('id="voltageBadge" class="toolbar-voltage-badge"', ui_controller)
        self.assertIn('id="statusChips" class="toolbar-status-chips"', ui_controller)
        self.assertNotIn('id="resultMessage"', ui_controller)
        self.assertNotIn('id="statusPanel"', ui_controller)
        self.assertNotIn('id="connectionText"', ui_controller)
        self.assertNotIn('id="connectionStatus"', ui_controller)
        self.assertIn('href="/help/"', ui_controller)
        self.assertNotIn('href="/help/camera-sdk/index"', ui_controller)
        self.assertIn('globalX: "全局 X"', legacy_command_catalog)
        self.assertIn("const ROBOT_BODY_SIZE_METERS = 0.7;", scene_view)
        self.assertIn("x: 0.2,", scene_view)
        self.assertIn("y: 0.1,", scene_view)
        self.assertIn("z: 0.2,", scene_view)
        self.assertNotIn("this.worldRoot.scale.set(1, 1, -1);", scene_view)
        self.assertIn("this.scene.up.set(0, 0, 1);", scene_view)
        self.assertIn("const INITIAL_CAMERA_POSITION = new THREE.Vector3(2.6, -3.0, 2.0);", scene_view)
        self.assertIn("const GLOBAL_VIEW_POSITION = new THREE.Vector3(3.6, -4.1, 2.8);", scene_view)
        self.assertIn("this.controls.maxPolarAngle = Math.PI * 0.48;", scene_view)
        self.assertIn("this.controls.minPolarAngle = 0.08;", scene_view)
        self.assertIn("function getPointAiStyleLocalPoint(localPoint) {", scene_view)
        self.assertIn("localPoint.y,", scene_view)
        self.assertIn("-localPoint.x,", scene_view)
        self.assertIn("getGripperOffsetRelativeToScepter()", scene_view)
        self.assertIn("convertScepterPointCloudPointToCabinPoint(localPoint)", scene_view)
        self.assertIn("scepterTransform.position.z - pointAiStyleLocalPoint.z - gripperOffsetZ", scene_view)
        self.assertNotIn("point.applyQuaternion(scepterTransform.quaternion).add(scepterTransform.position);", scene_view)

    def test_toolbar_is_single_line_and_does_not_duplicate_scene_controls(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        status_logic = (
            FRONTEND_SRC_DIR / "controllers" / "StatusMonitorController.js"
        ).read_text(encoding="utf-8")

        self.assertNotIn('data-toolbar-action="view:camera"', ui_controller)
        self.assertNotIn('data-toolbar-action="view:global"', ui_controller)
        self.assertNotIn('data-toolbar-action="toggle-follow-camera"', ui_controller)
        self.assertNotIn('data-toolbar-action="toggle-layer:pointCloud"', ui_controller)
        self.assertNotIn('data-toolbar-action="toggle-layer:tiePoints"', ui_controller)
        self.assertIn("flex-wrap: nowrap;", app_css)
        self.assertIn("overflow-x: auto;", app_css)
        self.assertIn("justify-content: space-between;", app_css)
        self.assertIn("width: calc(100vw - 32px);", app_css)
        self.assertIn("left: 16px;", app_css)
        self.assertIn("right: 16px;", app_css)
        self.assertNotIn('case "view:camera":', app_logic)
        self.assertNotIn('case "view:global":', app_logic)
        self.assertNotIn('case "toggle-follow-camera":', app_logic)
        self.assertIn(".panel-control {", app_css)
        self.assertIn(".control-button-grid {", app_css)
        self.assertIn('grid-template-columns: repeat(2, minmax(0, 1fr));', app_css)
        self.assertNotIn("--control-board:", app_css)
        self.assertIn('.control-action-btn[data-tone="green"] {', app_css)
        self.assertIn('.control-action-btn[data-tone="red"] {', app_css)
        self.assertIn('.control-action-btn[data-tone="blue"] {', app_css)
        self.assertIn(".control-command-group {", app_css)
        self.assertIn(".control-action-btn::before {", app_css)
        self.assertIn(".control-action-btn.is-active {", app_css)

    def test_floating_panels_are_resizable_on_desktop(self):
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        status_logic = (
            FRONTEND_SRC_DIR / "controllers" / "StatusMonitorController.js"
        ).read_text(encoding="utf-8")

        self.assertIn("resize: both;", app_css)
        self.assertIn(".floating-panel::after {", app_css)
        self.assertIn("right: 10px;", app_css)
        self.assertIn("bottom: 10px;", app_css)
        self.assertIn("resize: none;", app_css)
        self.assertIn(".toolbar-status-group {", app_css)
        self.assertIn(".toolbar-connection-badge {", app_css)
        self.assertIn(".toolbar-voltage-badge {", app_css)
        self.assertNotIn(".toolbar-status-message {", app_css)
        self.assertIn("height: calc(100vh - 108px);", app_css)
        self.assertIn("left: 360px;", app_css)
        self.assertIn(".status-chip.success { color: var(--success); }", app_css)
        self.assertIn('const MODUAN_TELEMETRY_TOPIC = "/moduan/moduan_gesture_data";', status_logic)
        self.assertIn('const MODUAN_TELEMETRY_TYPE = "tie_robot_msgs/linear_module_upload";', status_logic)
        self.assertIn("voltage > 100", ui_controller)

    def test_panel_card_fonts_are_compact(self):
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn(".panel-title {\n  font-size: 13px;", app_css)
        self.assertIn(".panel-subtitle {\n  color: var(--text-tertiary);\n  font-size: 10px;", app_css)
        self.assertIn(".section-title {\n  margin: 0 0 10px;\n  color: var(--text-secondary);\n  font-size: 11px;", app_css)
        self.assertIn(".control-group-title {\n  font-size: 11px;", app_css)
        self.assertIn(".control-action-btn {\n  position: relative;", app_css)
        self.assertIn("  font-size: 11px;", app_css)
        self.assertIn(".info-block {\n  display: grid;", app_css)
        self.assertIn("  font-size: 11px;", app_css)
        self.assertIn(".stats-card strong {\n  font-size: 16px;", app_css)

    def test_terminal_log_panel_subscribes_stdout_stream(self):
        ros_connection_controller = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")

        self.assertIn('const SYSTEM_LOG_TOPIC = "/system_log/all";', ros_connection_controller)
        self.assertIn('messageType: "rosgraph_msgs/Log"', ros_connection_controller)
        self.assertIn("onSystemLog", ros_connection_controller)
        self.assertIn('if (!rawText.startsWith("[stdout]")) {', app_logic)
        self.assertIn('message: `[${nodeName}] ${content}`', app_logic)
        self.assertIn("终端日志订阅", ui_controller)
        self.assertIn("暂无终端日志，等待节点 stdout 输出。", ui_controller)
        self.assertNotIn("this.ui.renderLogs(this.frontendLogs)", app_logic)

    def test_bind_http_server_falls_forward_when_preferred_port_is_busy(self):
        busy_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        busy_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        busy_socket.bind(("127.0.0.1", 0))
        busy_socket.listen(1)
        preferred_port = busy_socket.getsockname()[1]

        class DummyHandler:
            def __init__(self, *args, **kwargs):
                pass

        try:
            self.assertTrue(hasattr(self.server_module, "bind_http_server"))
            server, actual_port = self.server_module.bind_http_server(
                "127.0.0.1", preferred_port, DummyHandler
            )
            self.assertNotEqual(actual_port, preferred_port)
            self.assertGreater(actual_port, preferred_port)
            server.server_close()
        finally:
            busy_socket.close()

    def test_bind_http_server_falls_back_to_non_privileged_port_when_http_80_is_forbidden(self):
        attempted_ports = []

        class DummyHandler:
            def __init__(self, *args, **kwargs):
                pass

        class FakeServer:
            daemon_threads = False

            def server_close(self):
                pass

        def fake_http_server(server_address, _handler):
            _host, port = server_address
            attempted_ports.append(port)
            if port == 80:
                raise OSError(errno.EACCES, "Permission denied")
            if port == 1024:
                return FakeServer()
            raise AssertionError(f"unexpected fallback port attempted: {port}")

        original_http_server = self.server_module.ThreadingHTTPServer
        try:
            self.server_module.ThreadingHTTPServer = fake_http_server
            server, actual_port = self.server_module.bind_http_server(
                "127.0.0.1", 80, DummyHandler
            )
        finally:
            self.server_module.ThreadingHTTPServer = original_http_server

        self.assertEqual(actual_port, 1024)
        self.assertEqual(attempted_ports[:2], [80, 1024])
        server.server_close()

    def test_wait_for_picker_url_prefers_actual_port_param_when_available(self):
        checked_urls = []
        params = {"/workspace_picker_web/port": 8768}

        def fake_get_param(name, default=None):
            return params.get(name, default)

        def fake_url_ready(url):
            checked_urls.append(url)
            return url.endswith(":8768/index.html")

        self.assertTrue(hasattr(self.open_module, "wait_for_picker_url"))
        target_url = self.open_module.wait_for_picker_url(
            browser_host="127.0.0.1",
            fallback_port=8080,
            timeout_sec=0.1,
            get_param=fake_get_param,
            url_ready=fake_url_ready,
            sleep_fn=lambda _seconds: None,
            is_shutdown=lambda: False,
        )
        self.assertEqual(target_url, "http://127.0.0.1:8768/index.html")
        self.assertIn("http://127.0.0.1:8768/index.html", checked_urls)

    def test_build_target_url_omits_explicit_port_for_http_80(self):
        self.assertEqual(
            self.open_module.build_target_url("127.0.0.1", 80),
            "http://127.0.0.1/index.html",
        )
        self.assertEqual(
            self.server_module.build_browser_url("127.0.0.1", 80),
            "http://127.0.0.1/index.html",
        )

    def test_clean_url_request_path_maps_help_pages_to_html_files(self):
        frontend_dir = TIE_ROBOT_WEB_DIR / "web"

        self.assertEqual(
            self.server_module.resolve_clean_url_request_path(
                "/help/camera-sdk/index",
                frontend_dir,
            ),
            "/help/camera-sdk/index.html",
        )
        self.assertEqual(
            self.server_module.resolve_clean_url_request_path(
                "/help/camera-sdk/vendor-vzense/zh-cn/README",
                frontend_dir,
            ),
            "/help/camera-sdk/vendor-vzense/zh-cn/README.html",
        )
        self.assertEqual(
            self.server_module.resolve_clean_url_request_path("/index.html", frontend_dir),
            "/index.html",
        )


if __name__ == "__main__":
    unittest.main()
