#!/usr/bin/env python3

import errno
import importlib.util
import socket
import unittest
from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
TIE_ROBOT_BRINGUP_DIR = WORKSPACE_SRC / "tie_robot_bringup"
TIE_ROBOT_WEB_DIR = WORKSPACE_SRC / "tie_robot_web"
TIE_ROBOT_PERCEPTION_DIR = WORKSPACE_SRC / "tie_robot_perception"
TIE_ROBOT_BRINGUP_DRIVER_STACK = TIE_ROBOT_BRINGUP_DIR / "launch" / "driver_stack.launch"
TIE_ROBOT_BRINGUP_ALGORITHM_STACK = TIE_ROBOT_BRINGUP_DIR / "launch" / "algorithm_stack.launch"
FRONTEND_DIR = TIE_ROBOT_WEB_DIR / "web"
SERVER_SCRIPT = TIE_ROBOT_WEB_DIR / "scripts" / "workspace_picker_web_server.py"
OPEN_SCRIPT = TIE_ROBOT_WEB_DIR / "scripts" / "workspace_picker_web_open.py"
FRONTEND_SRC_DIR = TIE_ROBOT_WEB_DIR / "frontend" / "src"
CONTROL_PANEL_CATALOG = FRONTEND_SRC_DIR / "config" / "controlPanelCatalog.js"
IMAGE_TOPIC_CATALOG = FRONTEND_SRC_DIR / "config" / "imageTopicCatalog.js"
LOG_TOPIC_CATALOG = FRONTEND_SRC_DIR / "config" / "logTopicCatalog.js"
SYSTEM_CONTROL_CATALOG = FRONTEND_SRC_DIR / "config" / "systemControlCatalog.js"
STATUS_MONITOR_CATALOG = FRONTEND_SRC_DIR / "config" / "statusMonitorCatalog.js"
PANEL_REGISTRY = FRONTEND_SRC_DIR / "panels" / "panelRegistry.js"
DEFAULT_LAYOUTS = FRONTEND_SRC_DIR / "layout" / "defaultLayouts.js"


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

    def test_frontend_assets_exist(self):
        self.assertTrue((FRONTEND_DIR / "index.html").exists())
        self.assertTrue((FRONTEND_DIR / "help" / "index.html").exists())
        self.assertTrue(any(FRONTEND_DIR.glob("assets/app/index-*.js")))
        self.assertTrue(any(FRONTEND_DIR.glob("assets/app/index-*.css")))

    def test_run_launch_keeps_frontend_and_core_nodes_guarded(self):
        run_launch = (TIE_ROBOT_BRINGUP_DIR / "launch" / "run.launch").read_text(
            encoding="utf-8"
        )
        api_launch = (TIE_ROBOT_BRINGUP_DIR / "launch" / "api.launch").read_text(
            encoding="utf-8"
        )
        driver_stack_launch = TIE_ROBOT_BRINGUP_DRIVER_STACK.read_text(encoding="utf-8")
        algorithm_stack_launch = TIE_ROBOT_BRINGUP_ALGORITHM_STACK.read_text(encoding="utf-8")
        camera_launch = (
            TIE_ROBOT_PERCEPTION_DIR / "launch" / "scepter_camera.launch"
        ).read_text(encoding="utf-8")

        self.assertIn('<arg name="workspace_picker_port" default="8080"', run_launch)
        self.assertIn('name="workspace_picker_web_server"', run_launch)
        self.assertIn('respawn="true"', run_launch)
        self.assertIn('driver_stack.launch', run_launch)
        self.assertIn('algorithm_stack.launch', run_launch)
        self.assertIn('name="suoquNode"', driver_stack_launch)
        self.assertIn('name="moduanNode"', driver_stack_launch)
        self.assertIn('name="gripper_tf_broadcaster"', driver_stack_launch)
        self.assertIn('name="pointAINode"', algorithm_stack_launch)
        self.assertIn('name="stable_point_tf_broadcaster"', algorithm_stack_launch)
        self.assertIn('name="topicTransNode"', api_launch)
        self.assertIn('name="system_log_mux"', api_launch)
        self.assertIn('respawn="true"', api_launch)
        self.assertIn('name="scepter_manager"', camera_launch)
        self.assertIn('name="scepter_world_coord_processor"', camera_launch)
        self.assertIn('respawn="true"', camera_launch)
        self.assertNotIn('required="true"', camera_launch)

    def test_frontend_shell_uses_task_only_left_panel_top_status_and_bottom_quick_controls(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")
        control_panel_catalog = CONTROL_PANEL_CATALOG.read_text(encoding="utf-8")
        panel_registry = PANEL_REGISTRY.read_text(encoding="utf-8")
        default_layouts = DEFAULT_LAYOUTS.read_text(encoding="utf-8")

        self.assertNotIn("主链" "任务", ui_controller)
        self.assertNotIn("关键开关", ui_controller)
        self.assertIn('id="statusCapsuleGrid" class="toolbar-status-capsules"', ui_controller)
        self.assertIn('id="quickControlGrid" class="quick-control-grid"', ui_controller)
        self.assertIn("quick-control-dock", ui_controller)
        self.assertNotIn("快捷开关", ui_controller)
        self.assertIn("toolbar-status-strip", ui_controller)
        self.assertIn("toolbar-status-capsules", app_css)
        self.assertIn("quick-control-dock", app_css)
        self.assertIn("quick-control-grid", app_css)
        self.assertIn("quick-control-light", app_css)
        self.assertIn("z-index: 12;", app_css)
        self.assertIn("display: flex;", app_css)
        self.assertIn("flex-wrap: wrap;", app_css)
        self.assertIn("border-radius: 999px;", app_css)
        self.assertNotIn("controlPanelGroups", ui_controller)
        self.assertIn('aria-pressed="${control.active ? "true" : "false"}"', ui_controller)
        self.assertIn('class="quick-control-light ${control.active ? "is-active" : ""}"', ui_controller)
        self.assertIn('class="quick-control-label"', ui_controller)
        self.assertIn('data-panel-maximize="controlPanel"', ui_controller)
        self.assertIn('data-panel-maximize="imagePanel"', ui_controller)
        self.assertIn('data-panel-maximize="settingsPanel"', ui_controller)
        self.assertIn('data-panel-maximize="terminalPanel"', ui_controller)
        self.assertIn('data-panel-maximize="logPanel"', ui_controller)
        self.assertIn('id="imagePanel" class="floating-panel panel-image"', ui_controller)
        self.assertIn('id="terminalPanel" class="floating-panel panel-terminal"', ui_controller)
        self.assertIn('id="imageTopicSelect"', ui_controller)
        self.assertIn('IMAGE_TOPIC_OPTIONS.map((option) =>', ui_controller)
        self.assertIn('option.label', ui_controller)
        self.assertIn('DEFAULT_IMAGE_TOPIC', ui_controller)
        self.assertIn('id="createTerminalSession"', ui_controller)
        self.assertIn('id="terminalTabStrip"', ui_controller)
        self.assertIn('id="terminalViewport"', ui_controller)
        self.assertIn('id="createTerminalSessionEmpty"', ui_controller)
        self.assertIn('id="logTopicSelect"', ui_controller)
        self.assertIn('LOG_TOPIC_OPTIONS.map((option) =>', ui_controller)
        self.assertIn('DEFAULT_LOG_TOPIC', ui_controller)
        self.assertIn('data-image-panel-resize="true"', ui_controller)
        self.assertIn('id="settingsPanel" class="floating-panel panel-settings"', ui_controller)
        self.assertIn("切换本工程图像话题预览", ui_controller)
        self.assertIn("话题、工作区、图层与外参配置", ui_controller)
        self.assertIn('id="settingsPageSelect"', ui_controller)
        self.assertNotIn('id="controlFeedback"', ui_controller)
        self.assertIn('<option value="topics">话题总览</option>', ui_controller)
        self.assertIn('<option value="workspace">工作区选点</option>', ui_controller)
        self.assertIn('<option value="scene">显示与视角</option>', ui_controller)
        self.assertIn('<option value="layers">图层与数据</option>', ui_controller)
        self.assertIn('<option value="calibration">相机-TCP外参</option>', ui_controller)
        self.assertIn('data-settings-page="topics"', ui_controller)
        self.assertIn('data-settings-page="workspace"', ui_controller)
        self.assertIn('data-settings-page="scene"', ui_controller)
        self.assertIn('data-settings-page="layers"', ui_controller)
        self.assertIn('data-settings-page="calibration"', ui_controller)
        self.assertIn('id="topicInventoryList"', ui_controller)
        self.assertIn("topic-inventory-group", ui_controller)
        self.assertIn('const rootNamespace = normalizedSegments[0] ? `/${normalizedSegments[0]}` : "(根级话题)";', ui_controller)
        self.assertIn('data-topic-group-toggle="${groupKey}"', ui_controller)
        self.assertIn("bindTopicInventoryGroupToggles()", ui_controller)
        self.assertIn("相机-TCP外参", ui_controller)
        self.assertIn('id="gripperTfCurrent"', ui_controller)
        self.assertIn('id="gripperTfX"', ui_controller)
        self.assertIn('id="gripperTfY"', ui_controller)
        self.assertIn('id="gripperTfZ"', ui_controller)
        self.assertIn('id="applyGripperTfCalibration"', ui_controller)
        self.assertIn("工作区选点", ui_controller)
        self.assertIn('id: "moveToPosition", label: "移动到\\n索驱位置"', control_panel_catalog)
        self.assertIn("图层设置", ui_controller)
        self.assertIn("显示与视角", ui_controller)
        self.assertIn("图层与数据", ui_controller)
        self.assertIn('this.setSettingsPage("topics");', ui_controller)
        self.assertNotIn("系统控制", ui_controller)
        self.assertNotIn('data-settings-action="${action.id}"', ui_controller)
        self.assertIn('<label class="checkbox-field"><input id="showPointCloudToggle" type="checkbox" /><span>点云</span></label>', ui_controller)
        self.assertNotIn('id="topicsPanel"', ui_controller)
        self.assertNotIn('id="tfPanel"', ui_controller)
        self.assertNotIn('id="problemsPanel"', ui_controller)
        self.assertIn("imagePanel", panel_registry)
        self.assertIn("settingsPanel", panel_registry)
        self.assertIn("terminalPanel", panel_registry)
        self.assertNotIn("topicsPanel", panel_registry)
        self.assertNotIn("tfPanel", panel_registry)
        self.assertNotIn("problemsPanel", panel_registry)
        self.assertIn("imagePanel", default_layouts)
        self.assertIn("settingsPanel", default_layouts)
        self.assertIn("terminalPanel", default_layouts)
        self.assertNotIn("topicsPanel", default_layouts)
        self.assertNotIn("tfPanel", default_layouts)
        self.assertNotIn("problemsPanel", default_layouts)
        self.assertIn('items: ["pauseResume", "lashingEnabled", "jumpBindEnabled", "lightEnabled"]', control_panel_catalog)
        self.assertNotIn("视觉调试", control_panel_catalog)
        self.assertNotIn("急停作业", control_panel_catalog)
        self.assertNotIn("清除作业路径", control_panel_catalog)

    def test_toolbar_uses_theme_toggle_without_system_control_panel(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        frontend_index = (
            TIE_ROBOT_WEB_DIR / "frontend" / "index.html"
        ).read_text(encoding="utf-8")
        ros_connection = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        image_topic_catalog = IMAGE_TOPIC_CATALOG.read_text(encoding="utf-8")
        log_topic_catalog = LOG_TOPIC_CATALOG.read_text(encoding="utf-8")
        storage = (
            FRONTEND_SRC_DIR / "utils" / "storage.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertNotIn('id="settingsButton"', ui_controller)
        self.assertNotIn('data-toolbar-action="toggle-settings-menu"', ui_controller)
        self.assertNotIn('id="settingsModal"', ui_controller)
        self.assertIn('id="imagePanel" class="floating-panel panel-image"', ui_controller)
        self.assertIn('id="settingsPanel" class="floating-panel panel-settings"', ui_controller)
        self.assertIn('id="terminalPanel" class="floating-panel panel-terminal"', ui_controller)
        self.assertIn('id="imageTopicSelect"', ui_controller)
        self.assertIn('id="logTopicSelect"', ui_controller)
        self.assertIn('切换本工程图像话题预览', ui_controller)
        self.assertIn('切换日志来源', ui_controller)
        self.assertIn('本机 Ubuntu Shell 与 SSH 会话', ui_controller)
        self.assertIn('工作区选点', ui_controller)
        self.assertIn('id="themeToggle"', ui_controller)
        self.assertIn('data-toolbar-action="toggle-theme"', ui_controller)
        self.assertIn('class="toolbar-theme-icon" aria-hidden="true">🌙</span>', ui_controller)
        self.assertIn('id="connectionBadge"', ui_controller)
        self.assertIn('data-connection-action=""', ui_controller)
        self.assertIn('class="toolbar-connection-action-label"></span>', ui_controller)
        self.assertNotIn("SYSTEM_CONTROL_ACTIONS.map((action)", ui_controller)
        self.assertNotIn('data-settings-action="${action.id}"', ui_controller)
        self.assertIn('if (action === "toggle-theme")', app_logic)
        self.assertIn("onConnectionAction((actionId) => {", app_logic)
        self.assertIn("applyTheme(theme)", app_logic)
        self.assertIn("loadThemePreference()", app_logic)
        self.assertIn("saveThemePreference(this.theme)", app_logic)
        self.assertIn("this.systemLogs = [];", app_logic)
        self.assertIn("this.selectedLogTopicId = DEFAULT_LOG_TOPIC;", app_logic)
        self.assertIn("new TerminalController({", app_logic)
        self.assertIn("this.terminalController.init();", app_logic)
        self.assertIn("this.ui.renderTopicInventory([]);", app_logic)
        self.assertIn("this.ui.setGripperTfCalibration(null);", app_logic)
        self.assertIn("onTopicInventory: (topics) => {", app_logic)
        self.assertIn("this.ui.renderTopicInventory(topics);", app_logic)
        self.assertIn("this.ui.onCalibrationApply((payload) => {", app_logic)
        self.assertIn("applyGripperTfCalibration(payload)", app_logic)
        self.assertIn("async handleMoveToPosition()", app_logic)
        self.assertIn("DIRECT_CABIN_MOVE_TARGET", app_logic)
        self.assertIn("x: -260", app_logic)
        self.assertIn("y: 1700", app_logic)
        self.assertIn("z: 3197", app_logic)
        self.assertIn("speed: this.getGlobalCabinMoveSpeed()", app_logic)
        self.assertIn("callCabinSingleMoveService(payload)", app_logic)
        self.assertIn("this.ui.setGripperTfCalibration(this.sceneView.getCameraToTcpCalibration());", app_logic)
        self.assertIn("async applyGripperTfCalibration(payload)", app_logic)
        self.assertIn("await this.rosConnectionController.callGripperTfCalibrationService(payload)", app_logic)
        self.assertIn("{ forceInputs: true }", app_logic)
        self.assertIn("this.workspaceView.getSavedWorkspacePoints()", app_logic)
        self.assertIn("this.rosConnectionController.triggerWorkspaceS2Refresh()", app_logic)
        self.assertIn("已按新外参自动重跑当前工作区识别", app_logic)
        self.assertIn("this.ui.onLogTopicChange(() => {", app_logic)
        self.assertIn("this.ui.onTerminalAction((action, sessionId) => {", app_logic)
        self.assertIn("syncLogSubscription({ suppressLog = false } = {})", app_logic)
        self.assertIn("renderLogView()", app_logic)
        self.assertIn("matchesLogTopicFilter(entry, this.selectedLogTopicId)", app_logic)
        self.assertIn('icon.textContent = theme === "dark" ? "🌙" : "☀️";', ui_controller)
        self.assertIn("this.terminalController.setTheme(this.theme);", app_logic)
        self.assertIn("tie_robot_frontend_theme", storage)
        self.assertIn('document.documentElement.setAttribute("data-theme", theme);', frontend_index)
        self.assertIn("restartDriverStackService", ros_connection)
        self.assertIn("startDriverStackService", ros_connection)
        self.assertIn("startAlgorithmStackService", ros_connection)
        self.assertIn("restartAlgorithmStackService", ros_connection)
        self.assertIn("restartRosStackService", ros_connection)
        self.assertIn("cabinSingleMoveService", ros_connection)
        self.assertIn("callCabinSingleMoveService({ x, y, z, speed })", ros_connection)
        self.assertIn("displayedImageTopicSubscriber", ros_connection)
        self.assertIn("this.logTopicSubscriber = null;", ros_connection)
        self.assertIn("this.topicInventoryTimer = null;", ros_connection)
        self.assertIn("this.desiredLogTopicId = DEFAULT_LOG_TOPIC;", ros_connection)
        self.assertIn("applyDisplayedImageSubscription({ suppressLog = false } = {})", ros_connection)
        self.assertIn("updateDisplayedImageSubscription(topicName)", ros_connection)
        self.assertIn("applyLogSubscription({ suppressLog = false } = {})", ros_connection)
        self.assertIn("updateLogSubscription(topicId)", ros_connection)
        self.assertIn("gripperTfOffsetPublisher", ros_connection)
        self.assertIn("POINTAI_OFFSET_TOPIC", ros_connection)
        self.assertIn("SET_GRIPPER_TF_CALIBRATION_SERVICE", ros_connection)
        self.assertIn("SET_GRIPPER_TF_CALIBRATION_SERVICE_TYPE", ros_connection)
        self.assertIn("startTopicInventoryPolling()", ros_connection)
        self.assertIn("stopTopicInventoryPolling()", ros_connection)
        self.assertIn("fetchTopicInventory({ suppressLog = false } = {})", ros_connection)
        self.assertIn("this.ros.getTopicsAndRawTypes((result) => {", ros_connection)
        self.assertIn("publishGripperTfOffsetMm({ x, y, z })", ros_connection)
        self.assertIn("setGripperTfCalibrationService: new ROSLIB.Service", ros_connection)
        self.assertIn("callGripperTfCalibrationService({ x, y, z })", ros_connection)
        self.assertIn("triggerWorkspaceS2Refresh()", ros_connection)
        self.assertIn("getLogTopicOption(this.desiredLogTopicId)", ros_connection)
        self.assertIn("this.callbacks.onSystemLog?.(message, option)", ros_connection)
        self.assertIn('queue_length: 30,', ros_connection)
        self.assertIn("const COLOR_TOPIC = \"/Scepter/color/image_raw\";", ros_connection)
        self.assertIn("const DEPTH_TOPIC = \"/Scepter/depth/image_raw\";", ros_connection)
        self.assertIn('id: "/Scepter/ir/image_raw"', image_topic_catalog)
        self.assertIn('id: "/pointAI/result_image_raw"', image_topic_catalog)
        self.assertIn('id: "/Scepter/color/image_raw"', image_topic_catalog)
        self.assertIn('id: "/Scepter/depth/image_raw"', image_topic_catalog)
        self.assertIn('id: "/Scepter/worldCoord/world_coord"', image_topic_catalog)
        self.assertIn('id: "/Scepter/worldCoord/raw_world_coord"', image_topic_catalog)
        self.assertIn('id: "all"', log_topic_catalog)
        self.assertIn('id: "algorithm"', log_topic_catalog)
        self.assertIn('id: "cabin"', log_topic_catalog)
        self.assertIn('id: "moduan"', log_topic_catalog)
        self.assertIn('id: "camera"', log_topic_catalog)
        self.assertIn('id: "bridge"', log_topic_catalog)
        self.assertIn('topic: "/system_log/all"', log_topic_catalog)
        self.assertIn('topic: "/system_log/suoquNode"', log_topic_catalog)
        self.assertIn('topic: "/system_log/moduanNode"', log_topic_catalog)
        self.assertIn('DEFAULT_LOG_TOPIC = "all"', log_topic_catalog)
        self.assertIn("matchesLogTopicFilter(entry, topicId)", log_topic_catalog)
        self.assertIn(".toolbar-theme-toggle", app_css)
        self.assertIn(".toolbar-theme-icon", app_css)
        self.assertIn('[data-theme="light"]', app_css)

        self.assertIn('[data-theme="light"] .toolbar-pill.active {', app_css)
        self.assertIn("color: #0f2f63;", app_css)
        self.assertIn('.toolbar-connection-badge[data-has-action="true"]', app_css)
        self.assertIn(".toolbar-connection-action-label", app_css)
        self.assertIn('[data-theme="dark"] {', app_css)
        self.assertIn('color-scheme: dark;', app_css)
        self.assertIn(".panel-control {\n  top: 88px;", app_css)
        self.assertIn("background: var(--glass-bg-strong);", app_css)
        self.assertNotIn(".control-feedback {", app_css)
        self.assertIn('[data-theme="light"] .panel-control {', app_css)
        self.assertIn(".panel-image {", app_css)
        self.assertIn(".panel-settings {", app_css)
        self.assertIn("overflow: visible;", app_css)
        self.assertIn(".panel-log {", app_css)
        self.assertIn(".panel-log-topic-field {", app_css)
        self.assertIn("left: calc(50vw - 350px);", app_css)
        self.assertIn("top: calc(100vh - 446px);", app_css)
        self.assertIn("width: 700px;", app_css)
        self.assertIn("height: 320px;", app_css)
        self.assertIn("width: 256px;", app_css)
        self.assertIn("min-width: 256px;", app_css)
        self.assertIn(".image-panel-content,", app_css)
        self.assertIn(".settings-panel-content {", app_css)
        self.assertIn(".settings-page-switcher {", app_css)
        self.assertIn(".panel-settings .panel-content {", app_css)
        self.assertIn(".settings-page-field {", app_css)
        self.assertIn(".settings-page-stack {", app_css)
        self.assertIn(".panel-settings .ui-select-shell[data-open=\"true\"] {", app_css)
        self.assertIn(".panel-settings .ui-select-menu {", app_css)
        self.assertIn(".settings-page.is-active {", app_css)
        self.assertIn(".topic-inventory-list {", app_css)
        self.assertIn(".topic-inventory-item {", app_css)
        self.assertIn(".topic-inventory-group-chevron {", app_css)
        self.assertIn(".topic-inventory-group.is-collapsed .topic-inventory-group-chevron {", app_css)
        self.assertIn(".panel-image .panel-content {", app_css)
        self.assertIn("padding: 0;", app_css)
        self.assertIn(".image-panel-content {\n  display: flex;", app_css)
        self.assertIn("flex: 1;", app_css)
        self.assertIn(".settings-section {", app_css)
        self.assertIn(".field select option,", app_css)
        self.assertIn(".field select option:checked {", app_css)
        self.assertIn('[data-theme="light"] .field select option,', app_css)
        self.assertIn("appearance: none;", app_css)
        self.assertIn("border-radius: 18px;", app_css)
        self.assertIn("padding-right: 46px;", app_css)
        self.assertIn("transform: translateY(-1px);", app_css)
        self.assertIn("color-scheme: dark;", app_css)
        self.assertIn(".field select option,", app_css)
        self.assertIn("background: #10161f;", app_css)
        self.assertIn('[data-theme="light"] .field select {', app_css)
        self.assertIn("color-scheme: light;", app_css)
        self.assertIn('warn: { id: "startRosStack", label: "启动ROS" }', ui_controller)
        self.assertIn('error: { id: "startRosStack", label: "启动ROS" }', ui_controller)
        self.assertIn('this.setSettingsPage("topics");', ui_controller)
        self.assertIn('this.refs.settingsPageSelect = this.rootElement.querySelector("#settingsPageSelect");', ui_controller)
        self.assertIn('const nextVisible = this.ui.togglePanelVisible(panelId);', app_logic)
        self.assertIn('if (panelId === "logPanel" && nextVisible) {', app_logic)
        self.assertIn('this.panelManager.applyDefaultPanelRect(panelId);', app_logic)

    def test_settings_panel_supports_home_page_preference_and_control_panel_customization(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        storage = (
            FRONTEND_SRC_DIR / "utils" / "storage.js"
        ).read_text(encoding="utf-8")
        control_panel_catalog = CONTROL_PANEL_CATALOG.read_text(encoding="utf-8")
        ros_connection = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertNotIn('id="settingsHomePageSelect"', ui_controller)
        self.assertIn('id="controlPanelCustomizeButton"', ui_controller)
        self.assertIn('id="controlPanelCustomizeMenu"', ui_controller)
        self.assertIn('id="controlPanelCustomButtonForm"', ui_controller)
        self.assertIn('id="controlPanelCustomButtonName"', ui_controller)
        self.assertIn('id="controlPanelCustomButtonService"', ui_controller)
        self.assertIn('id="controlPanelTrashBin"', ui_controller)
        self.assertIn("renderControlPanelTasks(taskIds = null, customButtons = [])", ui_controller)
        self.assertIn("renderControlPanelCustomizeMenu(customButtons = [])", ui_controller)
        self.assertIn('homeButton.dataset.settingsHomePage = option.value;', ui_controller)
        self.assertIn('homeButton.className = "settings-home-page-button";', ui_controller)
        self.assertIn('this.refs.controlPanelCustomizeButton = this.rootElement.querySelector("#controlPanelCustomizeButton");', ui_controller)
        self.assertIn('this.refs.controlPanelCustomizeMenu = this.rootElement.querySelector("#controlPanelCustomizeMenu");', ui_controller)
        self.assertIn('this.refs.controlPanelCustomButtonForm = this.rootElement.querySelector("#controlPanelCustomButtonForm");', ui_controller)
        self.assertIn('this.refs.controlPanelTrashBin = this.rootElement.querySelector("#controlPanelTrashBin");', ui_controller)
        self.assertIn('data-custom-service-action="${button.id}"', ui_controller)
        self.assertIn("onSettingsHomePageChange(callback)", ui_controller)
        self.assertIn("this.handleSettingsHomePageChange = callback;", ui_controller)
        self.assertIn("onControlPanelCustomButtonCreate(callback)", ui_controller)
        self.assertIn("onCustomControlPanelAction(callback)", ui_controller)
        self.assertIn("onCustomControlPanelDelete(callback)", ui_controller)
        self.assertIn("startCustomControlPanelLongPress(", ui_controller)
        self.assertIn("beginCustomControlPanelDrag(", ui_controller)
        self.assertIn("finishCustomControlPanelDrag(", ui_controller)
        self.assertIn("toggleControlPanelCustomizeMenu(forceOpen = null)", ui_controller)
        self.assertIn(
            "this.renderControlPanelTasks(this.visibleControlPanelTaskIds, this.customControlPanelButtons);",
            app_logic,
        )
        self.assertIn(
            "this.renderControlPanelCustomizeMenu(this.customControlPanelButtons);",
            app_logic,
        )
        self.assertIn("this.ui.setSettingsHomePage(this.settingsHomePage);", app_logic)
        self.assertIn("this.ui.setSettingsPage(this.settingsHomePage);", app_logic)
        self.assertIn("loadCustomControlPanelButtons()", app_logic)
        self.assertIn("saveCustomControlPanelButtons(this.customControlPanelButtons)", app_logic)
        self.assertIn("normalizeCustomControlPanelButtons(loadCustomControlPanelButtons())", app_logic)
        self.assertIn("this.ui.onControlPanelCustomButtonCreate((draft) => {", app_logic)
        self.assertIn("this.ui.onCustomControlPanelAction((buttonId) => {", app_logic)
        self.assertIn("this.ui.onCustomControlPanelDelete((buttonId) => {", app_logic)
        self.assertIn("this.handleCustomControlPanelButton(buttonId);", app_logic)
        self.assertIn("loadSettingsHomePagePreference()", app_logic)
        self.assertIn("saveSettingsHomePagePreference(pageId)", app_logic)
        self.assertIn("loadControlPanelVisibleTasks()", app_logic)
        self.assertIn("SETTINGS_HOME_PAGE_KEY", storage)
        self.assertIn("CONTROL_PANEL_VISIBLE_TASKS_KEY", storage)
        self.assertIn("CUSTOM_CONTROL_PANEL_BUTTONS_KEY", storage)
        self.assertIn("loadSettingsHomePagePreference()", storage)
        self.assertIn("saveSettingsHomePagePreference(pageId)", storage)
        self.assertIn("loadControlPanelVisibleTasks()", storage)
        self.assertIn("saveControlPanelVisibleTasks(taskIds)", storage)
        self.assertIn("loadCustomControlPanelButtons()", storage)
        self.assertIn("saveCustomControlPanelButtons(buttons)", storage)
        self.assertIn("normalizeControlPanelVisibleTaskIds(taskIds)", control_panel_catalog)
        self.assertIn("getControlPanelTaskIds()", control_panel_catalog)
        self.assertIn("normalizeCustomControlPanelButtons(buttons)", control_panel_catalog)
        self.assertIn('serviceType: "std_srvs/Trigger"', control_panel_catalog)
        self.assertIn("callTriggerService(serviceName, serviceType = TRIGGER_SERVICE_TYPE)", ros_connection)
        self.assertIn(".settings-home-page-button {", app_css)
        self.assertIn(".settings-home-page-button.is-active {", app_css)
        self.assertIn(".control-panel-customize-menu {", app_css)
        self.assertIn(".control-panel-empty-state {", app_css)
        self.assertIn(".control-panel-custom-form {", app_css)
        self.assertIn(".control-panel-trash-bin {", app_css)
        self.assertIn(".control-action-btn-custom {", app_css)

    def test_move_to_position_uses_fixed_cabin_target(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")

        self.assertIn("const DIRECT_CABIN_MOVE_TARGET = Object.freeze({", app_logic)
        self.assertIn("x: -260,", app_logic)
        self.assertIn("y: 1700,", app_logic)
        self.assertIn("z: 3197,", app_logic)
        self.assertIn("const payload = { ...DIRECT_CABIN_MOVE_TARGET, speed: this.getGlobalCabinMoveSpeed() };", app_logic)
        self.assertIn("准备直接移动到索驱位置", app_logic)

    def test_fixed_scan_messages_use_updated_cabin_target(self):
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")
        execution_actions = (
            FRONTEND_DIR / "modules" / "execution_actions.mjs"
        ).read_text(encoding="utf-8")

        self.assertIn("x=-260, y=1700, z=3197", task_action_controller)
        self.assertIn("全局索驱速度", task_action_controller)
        self.assertNotIn("speed=100", task_action_controller)
        self.assertIn("x=-260, y=1700, z=3197", execution_actions)
        self.assertIn("全局索驱速度", execution_actions)
        self.assertNotIn("speed=100", execution_actions)

    def test_panel_manager_supports_header_drag_and_native_resize(self):
        panel_manager = (
            FRONTEND_SRC_DIR / "ui" / "PanelManager.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn("toggleMaximize(panelId)", panel_manager)
        self.assertIn("setPanelMaximized(panelId, maximized)", panel_manager)
        self.assertIn("getDefaultPanelRect(panelId)", panel_manager)
        self.assertIn('if (panelId === "logPanel") {', panel_manager)
        self.assertIn('const width = Math.min(700, Math.max(520, window.innerWidth - 720));', panel_manager)
        self.assertIn('const height = Math.min(320, Math.max(240, window.innerHeight - 860));', panel_manager)
        self.assertIn('const left = Math.max(360, Math.round((window.innerWidth - width) / 2));', panel_manager)
        self.assertIn('const top = Math.max(88, window.innerHeight - height - 126);', panel_manager)
        self.assertIn('if (panelId === "terminalPanel") {', panel_manager)
        self.assertIn('const width = Math.min(920, Math.max(620, window.innerWidth - 320));', panel_manager)
        self.assertIn("applyDefaultPanelRect(panelId)", panel_manager)
        self.assertIn('panel.style.left = `${defaultRect.left}px`;', panel_manager)
        self.assertIn('panel.style.top = `${defaultRect.top}px`;', panel_manager)
        self.assertIn('panel.style.width = `${defaultRect.width}px`;', panel_manager)
        self.assertIn('panel.style.height = `${defaultRect.height}px`;', panel_manager)
        self.assertIn("if (!panel || this.panels.has(panel.id)) {", panel_manager)
        self.assertIn('panel.querySelector("[data-panel-maximize]")', panel_manager)
        self.assertIn('const panelHeader = panel.querySelector(".panel-header");', panel_manager)
        self.assertIn('panel.style.left = `${rect.left}px`;', panel_manager)
        self.assertIn('panel.style.top = `${rect.top}px`;', panel_manager)
        self.assertIn('panel.style.right = "auto";', panel_manager)
        self.assertIn('panel.style.bottom = "auto";', panel_manager)
        self.assertIn('panel.style.transform = "";', panel_manager)
        self.assertIn('panel.addEventListener("mousedown", () => {', panel_manager)
        self.assertIn('panelHeader.addEventListener("mousedown", (event) => {', panel_manager)
        self.assertIn('if (event.target.closest(".panel-action-btn") || event.target.closest(".panel-header-field")) {', panel_manager)
        self.assertIn('event.target.closest(".panel-header-field")', panel_manager)
        self.assertIn('window.addEventListener("mousemove", (event) => {', panel_manager)
        self.assertIn('window.addEventListener("mouseup", () => {', panel_manager)
        self.assertIn('document.body.style.userSelect = "none";', panel_manager)
        self.assertIn('if (panelId === "imagePanel") {', panel_manager)
        self.assertIn('const imageResizeHandle = panel.querySelector("[data-image-panel-resize]");', panel_manager)
        self.assertIn("const getImageAspectRatio = () => {", panel_manager)
        self.assertIn("const applyImagePanelAspectRatio = () => {", panel_manager)
        self.assertIn("new ResizeObserver(() => {", panel_manager)
        self.assertIn("panel.style.height = `${headerHeight + nextContentHeight}px`;", panel_manager)
        self.assertIn("imageResizeHandle.addEventListener(\"mousedown\"", panel_manager)
        self.assertIn("const deltaX = event.clientX - resizeStartX;", panel_manager)
        self.assertIn("const minWidth = 320;", panel_manager)
        self.assertIn('this.refs.settingsPageSelect?.addEventListener("change", () => this.setSettingsPage(this.refs.settingsPageSelect.value));', ui_controller)
        self.assertNotIn('root.querySelectorAll(".floating-panel").forEach((panel) => {', app_logic)
        self.assertIn('if (panelId === "terminalPanel" && nextVisible) {', app_logic)
        self.assertIn('this.terminalController.handle("ensure")', app_logic)
        self.assertIn("resize: both;", app_css)
        self.assertIn(".panel-image {", app_css)
        self.assertIn(".panel-terminal {", app_css)
        self.assertIn("resize: none;", app_css)
        self.assertIn(".image-panel-resize-handle {", app_css)
        self.assertIn("cursor: nwse-resize;", app_css)
        self.assertIn(".terminal-tab-strip {", app_css)
        self.assertIn(".terminal-viewport {", app_css)
        self.assertIn(".floating-panel.maximized {", app_css)
        self.assertIn("resize: none;", app_css)
        self.assertIn("cursor: move;", app_css)
        self.assertIn(".canvas-stage {", app_css)
        self.assertIn("height: 100%;", app_css)
        self.assertIn("aspect-ratio: auto;", app_css)
        self.assertIn("object-fit: contain;", app_css)
        self.assertIn("object-position: center;", app_css)
        self.assertNotIn(".panel-resize-handle", app_css)
        self.assertNotIn("touch-action: none;", app_css)

    def test_status_capsule_tracks_only_connection_and_hardware(self):
        status_catalog = STATUS_MONITOR_CATALOG.read_text(encoding="utf-8")
        status_controller = (
            FRONTEND_SRC_DIR / "controllers" / "StatusMonitorController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")

        self.assertIn('id: "ros"', status_catalog)
        self.assertIn('id: "chassis"', status_catalog)
        self.assertIn('id: "moduan"', status_catalog)
        self.assertIn('id: "visual"', status_catalog)
        self.assertIn('diagnosticHardwareId: "tie_robot/chassis_driver"', status_catalog)
        self.assertIn('diagnosticHardwareId: "tie_robot/moduan_driver"', status_catalog)
        self.assertIn('diagnosticHardwareId: "tie_robot/visual_algorithm"', status_catalog)
        self.assertIn('name: "/diagnostics"', status_controller)
        self.assertIn("STATUS_MONITORS.filter((item) => item.diagnosticHardwareId)", status_controller)
        self.assertIn("this.diagnosticCache = new Map();", status_controller)
        self.assertIn("this.diagnosticCache.set(hardwareId, {", status_controller)
        self.assertIn("const cached = this.diagnosticCache.get(monitor.diagnosticHardwareId);", status_controller)
        self.assertIn("const stale = now - cached.receivedAt > DIAGNOSTIC_STALE_MS;", status_controller)
        self.assertIn('this.callbacks.onStatusChip?.("moduan"', status_controller)
        self.assertIn('this.callbacks.onStatusChip?.("visual"', status_controller)
        self.assertIn("状态胶囊当前只保留硬件状态", app_logic)
        self.assertIn('STATUS_MONITORS\n      .filter((item) => item.id !== "ros")', ui_controller)
        self.assertIn('data-status-id="${item.id}"', ui_controller)
        self.assertIn('data-status-action=""', ui_controller)
        self.assertIn('class="system-status-action-label"', ui_controller)
        self.assertIn("onStatusChipAction(callback)", ui_controller)
        self.assertIn("setStatusChipState(statusId, level, detail)", ui_controller)
        self.assertIn('ros: level === "success" ? "restartRosStack" : "startRosStack"', ui_controller)
        self.assertIn('chassis: level === "success" ? "restartCabinDriver" : "startCabinDriver"', ui_controller)
        self.assertIn('moduan: level === "success" ? "restartModuanDriver" : "startModuanDriver"', ui_controller)
        self.assertIn('visual: level === "success" ? "restartAlgorithmStack" : "startAlgorithmStack"', ui_controller)
        self.assertIn('ros: level === "success" ? "重启ROS" : "启动ROS"', ui_controller)
        self.assertIn('chassis: level === "success" ? "重启" : "启动"', ui_controller)
        self.assertIn('visual: level === "success" ? "重启" : "启动"', ui_controller)
        self.assertIn('chip.dataset.statusAction = nextAction;', ui_controller)
        self.assertIn('button.setAttribute("aria-pressed", active ? "true" : "false");', ui_controller)

    def test_workspace_overlay_chain_uses_result_img_only(self):
        ros_connection_controller = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")

        self.assertIn('const EXECUTION_RESULT_TOPIC = "/pointAI/result_image_raw";', ros_connection_controller)
        self.assertIn("覆盖层统一等待 result_img", task_action_controller)
        self.assertIn("工作区覆盖层已更新，当前统一显示 /pointAI/result_image_raw。", app_logic)

    def test_image_panel_supports_project_image_topics_and_overlay_switching(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        workspace_canvas_view = (
            FRONTEND_SRC_DIR / "views" / "WorkspaceCanvasView.js"
        ).read_text(encoding="utf-8")
        image_utils = (
            FRONTEND_SRC_DIR / "utils" / "irImageUtils.js"
        ).read_text(encoding="utf-8")

        self.assertIn("syncDisplayedImageSubscription({ suppressLog = false } = {})", app_logic)
        self.assertIn('this.workspaceView.setSavedWorkspaceGuideVisible(false);', app_logic)
        self.assertIn('this.ui.onSettingsPageChange((pageId) => {', app_logic)
        self.assertIn('this.workspaceView.setSavedWorkspaceGuideVisible(pageId === "workspace");', app_logic)
        self.assertIn("this.ui.onImageTopicChange(() => {", app_logic)
        self.assertIn("this.workspaceView.setOverlayEnabled(isOverlayCompatibleImageTopic(selectedTopic));", app_logic)
        self.assertIn("图像卡片已切换到", app_logic)
        self.assertIn("setSavedWorkspaceGuideVisible(enabled)", workspace_canvas_view)
        self.assertIn("if (this.savedWorkspaceGuideVisible && this.savedWorkspacePoints.length >= 2)", workspace_canvas_view)
        self.assertIn("setOverlayEnabled(enabled)", workspace_canvas_view)
        self.assertIn("if (!this.overlayEnabled || !this.lastExecutionResultMessage)", workspace_canvas_view)
        self.assertIn('encoding.includes("mono16") || encoding.includes("16uc1")', image_utils)
        self.assertIn('encoding.includes("32fc1")', image_utils)
        self.assertIn('encoding.includes("32fc3")', image_utils)

    def test_saved_workspace_ack_replaces_current_edit_quad_with_backend_saved_quad(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        workspace_canvas_view = (
            FRONTEND_SRC_DIR / "views" / "WorkspaceCanvasView.js"
        ).read_text(encoding="utf-8")
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")

        self.assertIn("setSelectedWorkspacePayload(payload)", workspace_canvas_view)
        self.assertIn("this.selectedPoints = parseWorkspaceQuadPayload(payload);", workspace_canvas_view)
        self.assertIn("this.notifySelectionChanged();", workspace_canvas_view)
        self.assertIn("return this.confirmPendingWorkspaceQuadSubmission(payload);", task_action_controller)
        self.assertIn("const confirmed = this.taskActionController.handleSavedWorkspacePayload(payload);", app_logic)
        self.assertIn("this.workspaceView.setSelectedWorkspacePayload(payload);", app_logic)

    def test_point_cloud_subscription_is_lazy_single_source_and_throttled(self):
        ros_connection_controller = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        topic_layer_catalog = (
            FRONTEND_SRC_DIR / "config" / "topicLayerCatalog.js"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")

        self.assertIn("this.fixedTopicSubscribers = [];", ros_connection_controller)
        self.assertIn("this.pointCloudTopicSubscriber = null;", ros_connection_controller)
        self.assertIn("this.desiredPointCloudSubscription = {", ros_connection_controller)
        self.assertIn("applyPointCloudSubscription({ suppressLog = false } = {})", ros_connection_controller)
        self.assertIn("updatePointCloudSubscription({ enabled, source })", ros_connection_controller)
        self.assertIn("const nextTopicName = this.getPointCloudTopicName(nextState.source);", ros_connection_controller)
        self.assertIn("throttle_rate: 350,", ros_connection_controller)
        self.assertIn("queue_length: 1,", ros_connection_controller)
        self.assertNotIn('subscriptions[3].subscribe((message) => this.callbacks.onPointCloudImage?.("filteredWorldCoord", message));', ros_connection_controller)
        self.assertNotIn('subscriptions[4].subscribe((message) => this.callbacks.onPointCloudImage?.("rawWorldCoord", message));', ros_connection_controller)
        self.assertIn("syncPointCloudSubscription({ suppressLog = false } = {})", app_logic)
        self.assertIn("this.syncPointCloudSubscription({ suppressLog: true });", app_logic)
        self.assertIn("this.syncPointCloudSubscription();", app_logic)
        self.assertIn("this.clearInactivePointClouds({ clearSelectedSource: !layerState.showPointCloud });", app_logic)
        self.assertIn('showPointCloud: false,', topic_layer_catalog)
        self.assertIn("clearPointCloudSource(source)", scene_view)
        self.assertIn("clearAllPointCloudSources()", scene_view)

    def test_viewer_app_no_longer_renders_topics_tf_problems_panels(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")

        self.assertNotIn("renderViewerPanels(state)", app_logic)
        self.assertNotIn("recordTopicActivity(topicId, entry)", app_logic)
        self.assertNotIn("refreshViewerProblems()", app_logic)
        self.assertIn("new ViewerStore()", app_logic)
        self.assertIn("new SceneAdapter()", app_logic)
        self.assertIn("cache.set(frameId, null);", scene_view)
        self.assertIn("if (!parentWorld) {", scene_view)
        self.assertIn("return null;", scene_view)
        self.assertNotIn("|| {\n    position: new THREE.Vector3(0, 0, 0),", scene_view)
        self.assertIn("this.grid.visible = true;", scene_view)
        self.assertIn("this.robotGroup.visible = false;", scene_view)
        self.assertIn("if (scepterTransform)", scene_view)
        self.assertIn("this.applyCustomDisplayPose(this.robotGroup, scepterTransform.position);", scene_view)
        self.assertNotIn("ROBOT_BODY_CLEARANCE_ABOVE_TCP_METERS", scene_view)
        self.assertIn("if (gripperTransform)", scene_view)
        self.assertIn("this.applyCustomDisplayPose(this.tcpToolGroup, gripperTransform.position);", scene_view)
        self.assertNotIn("this.robotGroup.position.z += 0.58;", scene_view)
        self.assertIn("applyCustomDisplayPose(group, position)", scene_view)
        self.assertIn("group.quaternion.identity();", scene_view)
        self.assertIn("group.scale.set(1, 1, -1);", scene_view)
        self.assertNotIn("this.robotGroup.quaternion.copy(scepterTransform.quaternion);", scene_view)
        self.assertNotIn("this.tcpToolGroup.quaternion.copy(gripperTransform.quaternion);", scene_view)
        self.assertIn("const gripperOffsetZ = this.getGripperOffsetRelativeToScepter();", scene_view)
        self.assertIn("z: directRecord.position.z * 1000.0,", scene_view)
        self.assertIn("z: relative.z * 1000.0,", scene_view)
        self.assertIn("scepterTransform.position.z - pointAiStyleLocalPoint.z + gripperOffsetZ,", scene_view)
        self.assertNotIn("z: -directRecord.position.z * 1000.0,", scene_view)
        self.assertNotIn("z: -relative.z * 1000.0,", scene_view)
        self.assertNotIn("scepterTransform.position.z - pointAiStyleLocalPoint.z - gripperOffsetZ,", scene_view)
        self.assertNotIn("Math.abs(this.getGripperOffsetRelativeToScepter())", scene_view)
        self.assertIn("materials[0].color.setHex(0x7d8da4);", scene_view)
        self.assertIn("materials[1].color.setHex(0xa6b2c2);", scene_view)
        self.assertIn("materials[0].color.setHex(0xffffff);", scene_view)
        self.assertIn("materials[1].color.setHex(0xe8eef7);", scene_view)

    def test_camera_depth_frame_tf_publishes_downward_z_and_frontend_view_stays_stable(self):
        suoqu_node = (
            WORKSPACE_SRC / "tie_robot_process" / "src" / "suoquNode.cpp"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")

        self.assertIn("tf2::Quaternion scepter_depth_orientation;", suoqu_node)
        self.assertIn("scepter_depth_orientation.setRPY(M_PI, 0.0, 0.0);", suoqu_node)
        self.assertIn("transform.transform.rotation.x = scepter_depth_orientation.x();", suoqu_node)
        self.assertIn("transform.transform.rotation.y = scepter_depth_orientation.y();", suoqu_node)
        self.assertIn("transform.transform.rotation.z = scepter_depth_orientation.z();", suoqu_node)
        self.assertIn("transform.transform.rotation.w = scepter_depth_orientation.w();", suoqu_node)
        self.assertIn("const nextPosition = scepterTransform.position.clone().add(CAMERA_VIEW_OFFSET);", scene_view)
        self.assertIn("const nextTarget = scepterTransform.position.clone().add(CAMERA_VIEW_TARGET_OFFSET);", scene_view)
        self.assertNotIn("CAMERA_VIEW_OFFSET.clone().applyQuaternion(scepterTransform.quaternion)", scene_view)
        self.assertNotIn("CAMERA_VIEW_TARGET_OFFSET.clone().applyQuaternion(scepterTransform.quaternion)", scene_view)

    def test_bind_http_server_falls_forward_when_preferred_port_is_busy(self):
        try:
            busy_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except PermissionError as exc:
            self.skipTest(f"当前环境不允许创建测试 socket: {exc}")
        busy_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        busy_socket.bind(("127.0.0.1", 0))
        busy_socket.listen(1)
        preferred_port = busy_socket.getsockname()[1]

        class DummyHandler:
            def __init__(self, *args, **kwargs):
                pass

        try:
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

    def test_system_control_http_endpoints_cover_start_and_restart_actions(self):
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        system_control_catalog = SYSTEM_CONTROL_CATALOG.read_text(encoding="utf-8")
        system_control_controller = (
            FRONTEND_SRC_DIR / "controllers" / "SystemControlController.js"
        ).read_text(encoding="utf-8")

        self.assertIn('"/api/system/start_ros_stack": WORKSPACE_ROOT / "start_.sh"', server_script)
        self.assertIn('"/api/system/start_driver_stack": WORKSPACE_ROOT / "start_driver_stack.sh"', server_script)
        self.assertIn('"/api/system/restart_driver_stack": WORKSPACE_ROOT / "restart.sh"', server_script)
        self.assertIn('"/api/system/start_algorithm_stack": WORKSPACE_ROOT / "start_algorithm_stack.sh"', server_script)
        self.assertIn('"/api/system/restart_algorithm_stack": WORKSPACE_ROOT / "restart_algorithm_stack.sh"', server_script)
        self.assertIn('"/api/system/restart_ros_stack": WORKSPACE_ROOT / "restart_ros_stack.sh"', server_script)
        self.assertIn('id: "startRosStack"', system_control_catalog)
        self.assertIn('id: "restartAlgorithmStack"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/start_ros_stack"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/restart_algorithm_stack"', system_control_catalog)
        self.assertIn("handleViaHttp(action)", system_control_controller)
        self.assertIn("fetch(action.httpEndpoint", system_control_controller)

    def test_terminal_tool_uses_backend_session_api_and_multi_session_panel(self):
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        terminal_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TerminalController.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")

        self.assertIn("DEFAULT_TERMINAL_PORT = 8081", server_script)
        self.assertIn('WORKSPACE_PICKER_TERMINAL_PORT_PARAM = "/workspace_picker_web/terminal_port"', server_script)
        self.assertIn('if parsed.path == "/api/terminal/config":', server_script)
        self.assertIn('if parsed.path == "/api/terminal/sessions":', server_script)
        self.assertIn('parsed.path.startswith("/api/terminal/sessions/")', server_script)
        self.assertIn("class TerminalSessionManager", server_script)
        self.assertIn("class TerminalWebSocketHandler", server_script)
        self.assertIn('"/ws/terminal/(?P<session_id>[^/]+)"', server_script)
        self.assertIn('import { Terminal } from "@xterm/xterm";', terminal_controller)
        self.assertIn('import { FitAddon } from "@xterm/addon-fit";', terminal_controller)
        self.assertIn('import "@xterm/xterm/css/xterm.css";', terminal_controller)
        self.assertIn('await fetch("/api/terminal/config"', terminal_controller)
        self.assertIn('await fetch("/api/terminal/sessions"', terminal_controller)
        self.assertIn("new WebSocket(buildTerminalWsUrl(session))", terminal_controller)
        self.assertIn('this.refs.terminalTabStrip?.addEventListener("click"', ui_controller)
        self.assertIn("renderTerminalSessions(sessions, activeSessionId)", ui_controller)

    def test_submit_workspace_quad_waits_for_saved_workspace_ack_before_triggering_s2(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")

        self.assertIn("handleSavedWorkspacePayload(payload)", task_action_controller)
        self.assertIn("setPendingWorkspaceQuadSubmission(payload)", task_action_controller)
        self.assertIn("confirmPendingWorkspaceQuadSubmission(payload)", task_action_controller)
        self.assertIn("clearPendingWorkspaceQuadSubmission()", task_action_controller)
        self.assertIn("const confirmed = this.taskActionController.handleSavedWorkspacePayload(payload);", app_logic)
        self.assertIn("if (confirmed) {", app_logic)
        self.assertIn("this.workspaceView.setSelectedWorkspacePayload(payload);", app_logic)
        self.assertIn("this.taskActionController.handleSavedWorkspacePayload(payload);", app_logic)
        self.assertNotIn("}, 180);", task_action_controller)
        self.assertNotIn("resources.runWorkspaceS2Publisher.publish(new ROSLIB.Message({ data: true }));", task_action_controller)

    def test_ui_controller_removes_control_feedback_and_flashes_task_buttons(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")

        self.assertNotIn('id="controlFeedback"', ui_controller)
        self.assertNotIn('this.refs.controlFeedback = this.rootElement.querySelector("#controlFeedback");', ui_controller)
        self.assertIn("this.taskButtonFeedbackTimers = new Map();", ui_controller)
        self.assertIn("flashTaskButton(taskAction, { durationMs = 700 } = {})", ui_controller)
        self.assertIn("this.flashTaskButton(button.dataset.taskAction);", ui_controller)
        self.assertIn("setControlFeedback(_message) {}", ui_controller)

    def test_s2_trigger_auto_prepares_overlay_view_and_timeout_feedback(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")

        self.assertIn("onWorkspaceS2Triggered: () => this.handleWorkspaceS2Triggered(),", app_logic)
        self.assertIn("handleWorkspaceS2Triggered()", app_logic)
        self.assertIn("ensureS2OverlayDisplayTopic()", app_logic)
        self.assertIn("scheduleS2ResultTimeout()", app_logic)
        self.assertIn("clearS2ResultTimeout()", app_logic)
        self.assertIn('this.ui.setSelectedImageTopic(DEFAULT_IMAGE_TOPIC);', app_logic)
        self.assertIn("图像卡片已自动切回红外图像，以便显示 S2 覆盖层。", app_logic)
        self.assertIn("S2 已触发，但等待结果图超时", app_logic)
        self.assertIn("this.clearS2ResultTimeout();", app_logic)
        self.assertIn("this.callbacks.onWorkspaceS2Triggered?.();", task_action_controller)

    def test_settings_panel_renders_cabin_remote_page(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn('<option value="cabinRemote">索驱遥控</option>', ui_controller)
        self.assertIn('data-settings-page="cabinRemote"', ui_controller)
        self.assertIn('id="cabinKeyboardRemoteToggle"', ui_controller)
        self.assertIn('id="cabinRemoteStep"', ui_controller)
        self.assertIn('id="cabinRemoteSpeed"', ui_controller)
        self.assertIn('id="cabinRemoteCurrentPosition"', ui_controller)
        self.assertIn('id="cabinRemoteStatus"', ui_controller)
        self.assertIn('data-cabin-remote-stop="true"', ui_controller)
        self.assertIn('data-cabin-remote-direction="xNegative"', ui_controller)
        self.assertIn('data-cabin-remote-direction="xPositive"', ui_controller)
        self.assertIn('data-cabin-remote-direction="yPositive"', ui_controller)
        self.assertIn('data-cabin-remote-direction="yNegative"', ui_controller)
        self.assertIn('data-cabin-remote-direction="zPositive"', ui_controller)
        self.assertIn('data-cabin-remote-direction="zNegative"', ui_controller)
        self.assertIn("索驱停止移动", ui_controller)
        self.assertIn("this.refs.cabinKeyboardRemoteToggle =", ui_controller)
        self.assertIn("this.refs.cabinRemoteStep =", ui_controller)
        self.assertIn("this.refs.cabinRemoteSpeed =", ui_controller)
        self.assertIn("this.refs.cabinRemoteCurrentPosition =", ui_controller)
        self.assertIn("this.refs.cabinRemoteStatus =", ui_controller)
        self.assertIn("this.refs.cabinRemoteButtons =", ui_controller)
        self.assertIn("this.refs.cabinRemoteStopButton =", ui_controller)
        self.assertIn("getCabinRemoteSettings()", ui_controller)
        self.assertIn("onCabinRemoteAction(callback)", ui_controller)
        self.assertIn("setCabinRemoteCurrentPosition(position)", ui_controller)
        self.assertIn("setCabinRemoteButtonActive(directionId)", ui_controller)
        self.assertIn("clearCabinRemoteButtonActive()", ui_controller)
        self.assertIn("setCabinRemoteStatus(message)", ui_controller)
        self.assertIn("setCabinRemoteButtonsEnabled(enabled)", ui_controller)
        self.assertIn(".cabin-remote-stop-btn {", app_css)
        self.assertIn('grid-template-areas:', app_css)
        self.assertIn('"zPositive xPositive zNegative"', app_css)
        self.assertIn('"yPositive stop yNegative"', app_css)
        self.assertIn('"emptyLeft xNegative emptyRight"', app_css)
        self.assertIn(".cabin-remote-position-grid {", app_css)
        self.assertIn(".cabin-remote-position-value {", app_css)
        self.assertIn(".cabin-remote-pad {", app_css)
        self.assertIn(".cabin-remote-btn {", app_css)
        self.assertIn(".cabin-remote-btn.is-active {", app_css)
        self.assertIn(".cabin-remote-status-active {", app_css)

    def test_cabin_remote_keyboard_and_tf_flow_exist(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")
        cabin_remote_controller = (
            FRONTEND_SRC_DIR / "controllers" / "CabinRemoteController.js"
        ).read_text(encoding="utf-8")
        ros_connection = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        service_orchestration = (
            WORKSPACE_SRC / "tie_robot_process" / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")
        suoqu_node = (
            WORKSPACE_SRC / "tie_robot_process" / "src" / "suoquNode.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn('import { CabinRemoteController } from "../controllers/CabinRemoteController.js";', app_logic)
        self.assertIn("this.handleCabinRemoteKeyDown = this.handleCabinRemoteKeyDown.bind(this);", app_logic)
        self.assertIn("this.cabinRemoteController = new CabinRemoteController({", app_logic)
        self.assertIn('document.addEventListener("keydown", this.handleCabinRemoteKeyDown, true);', app_logic)
        self.assertIn("this.ui.onCabinRemoteAction((directionId, event) => {", app_logic)
        self.assertIn("handleCabinRemoteKeyDown(event)", app_logic)
        self.assertIn("resolveCabinRemoteDirectionFromKey(key)", app_logic)
        self.assertIn('q: "zPositive"', app_logic)
        self.assertIn('w: "xPositive"', app_logic)
        self.assertIn('e: "zNegative"', app_logic)
        self.assertIn('a: "yPositive"', app_logic)
        self.assertIn('s: "xNegative"', app_logic)
        self.assertIn('d: "yNegative"', app_logic)
        self.assertIn("handleCabinRemoteStopAction(source)", app_logic)
        self.assertIn('event.type === "stop"', app_logic)
        self.assertIn("await this.rosConnectionController.callCabinMotionStopService()", app_logic)
        self.assertIn("this.stopCabinRemoteRepeat();", app_logic)
        self.assertIn("shouldIgnoreCabinRemoteKeyboard(target)", app_logic)
        self.assertIn("event.repeat", app_logic)
        self.assertIn("keyboardEnabled", app_logic)
        self.assertIn("this.cabinRemoteController.setLastKnownCabinPosition(payload);", app_logic)
        self.assertIn("this.ui.setCabinRemoteButtonsEnabled(", app_logic)
        self.assertIn("this.ui.setCabinRemoteCurrentPosition(currentCabinPosition);", app_logic)
        self.assertIn("refreshCabinRemoteCurrentPosition()", app_logic)
        self.assertIn("startCabinRemoteRepeat(directionId)", app_logic)
        self.assertIn("stopCabinRemoteRepeat()", app_logic)
        self.assertIn("scheduleCabinRemoteRepeatTick()", app_logic)
        self.assertIn("window.addEventListener(\"pointerup\", this.handleCabinRemoteGlobalPointerUp, true);", app_logic)
        self.assertIn("window.addEventListener(\"blur\", this.handleCabinRemoteWindowBlur, true);", app_logic)
        self.assertIn("document.addEventListener(\"visibilitychange\", this.handleCabinRemoteVisibilityChange, true);", app_logic)
        self.assertIn("event.type === \"pressstart\"", app_logic)
        self.assertIn("event.type === \"pressend\"", app_logic)
        self.assertIn("this.ui.setCabinRemoteButtonActive(directionId);", app_logic)
        self.assertIn("this.ui.clearCabinRemoteButtonActive();", app_logic)
        self.assertIn("this.cabinRemoteRepeatTimerId = window.setTimeout(", app_logic)
        self.assertIn("if (this.cabinRemoteMoveInFlight) {", app_logic)
        self.assertIn("moveToPosition: ready && Boolean(resources?.cabinSingleMoveService),", app_logic)
        self.assertIn("getCurrentCabinPositionMm()", scene_view)
        self.assertIn("const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);", scene_view)
        self.assertIn("x: scepterTransform.position.x * 1000.0,", scene_view)
        self.assertIn("y: scepterTransform.position.y * 1000.0,", scene_view)
        self.assertIn("z: scepterTransform.position.z * 1000.0,", scene_view)
        self.assertIn("const CABIN_REMOTE_DIRECTION_DEFINITIONS = {", cabin_remote_controller)
        self.assertIn('xNegative: { axis: "x", delta: -1, label: "X-" }', cabin_remote_controller)
        self.assertIn('yPositive: { axis: "y", delta: 1, label: "Y+" }', cabin_remote_controller)
        self.assertIn('zPositive: { axis: "z", delta: 1, label: "Z+" }', cabin_remote_controller)
        self.assertIn("this.sceneView.getCurrentCabinPositionMm()", cabin_remote_controller)
        self.assertIn("this.lastKnownCabinPositionMm", cabin_remote_controller)
        self.assertIn("target[definition.axis] += definition.delta * sanitizedStep;", cabin_remote_controller)
        self.assertIn("callCabinSingleMoveService(target)", cabin_remote_controller)
        self.assertIn("syncGlobalCabinMoveSpeed({ suppressLog = false } = {})", app_logic)
        self.assertIn("this.rosConnectionController.publishCabinSpeed(speed)", app_logic)
        self.assertIn('const SET_CABIN_SPEED_TOPIC = "/web/cabin/set_cabin_speed";', ros_connection)
        self.assertIn("cabinSpeedPublisher: new ROSLIB.Topic({", ros_connection)
        self.assertIn("publishCabinSpeed(speed)", ros_connection)
        self.assertIn("this.resources.cabinSpeedPublisher.publish(", ros_connection)
        self.assertIn('const STOP_CABIN_MOTION_SERVICE = "/cabin/motion/stop";', ros_connection)
        self.assertIn("stopCabinMotionService: new ROSLIB.Service({", ros_connection)
        self.assertIn("callCabinMotionStopService()", ros_connection)
        self.assertIn("this.resources.stopCabinMotionService.callService(", ros_connection)
        self.assertIn("bool cabinMotionStopService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)", service_orchestration)
        self.assertIn("stop_cabin_motion_via_driver(&driver_error_message)", service_orchestration)
        self.assertIn("索驱运动已停止。", service_orchestration)
        self.assertIn("cabin_speed = get_global_cabin_move_speed_mm_per_sec();", service_orchestration)
        self.assertIn("float get_global_cabin_move_speed_mm_per_sec()", suoqu_node)
        self.assertIn("const float fixed_scan_speed = get_global_cabin_move_speed_mm_per_sec();", suoqu_node)
        self.assertIn("const float cabin_speed = get_global_cabin_move_speed_mm_per_sec();", suoqu_node)
        self.assertIn('nh.advertiseService("/cabin/motion/stop", cabinMotionStopService);', suoqu_node)


if __name__ == "__main__":
    unittest.main()
