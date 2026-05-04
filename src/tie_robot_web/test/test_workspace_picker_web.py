#!/usr/bin/env python3

import errno
import importlib.util
import socket
import sys
import unittest
from pathlib import Path


sys.dont_write_bytecode = True

WORKSPACE_SRC = Path(__file__).resolve().parents[2]
TIE_ROBOT_BRINGUP_DIR = WORKSPACE_SRC / "tie_robot_bringup"
TIE_ROBOT_WEB_DIR = WORKSPACE_SRC / "tie_robot_web"
TIE_ROBOT_PERCEPTION_DIR = WORKSPACE_SRC / "tie_robot_perception"
TIE_ROBOT_PROCESS_DIR = WORKSPACE_SRC / "tie_robot_process"
TIE_ROBOT_BRINGUP_DRIVER_STACK = TIE_ROBOT_BRINGUP_DIR / "launch" / "driver_stack.launch"
TIE_ROBOT_BRINGUP_ALGORITHM_STACK = TIE_ROBOT_BRINGUP_DIR / "launch" / "algorithm_stack.launch"
TIE_ROBOT_BRINGUP_FRONTEND_SERVICE = TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-frontend.service.in"
TIE_ROBOT_BRINGUP_FRONTEND_AUTOSTART_INSTALLER = TIE_ROBOT_BRINGUP_DIR / "scripts" / "install_frontend_autostart.sh"
TIE_ROBOT_BRINGUP_BACKEND_SERVICE = TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-backend.service.in"
TIE_ROBOT_BRINGUP_BACKEND_SUDOERS = TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-backend-control.sudoers.in"
TIE_ROBOT_BRINGUP_BACKEND_INSTALLER = TIE_ROBOT_BRINGUP_DIR / "scripts" / "install_backend_service.sh"
TIE_ROBOT_BRINGUP_DEMO_MODE_SERVICE = TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-demo-show-full.service.in"
TIE_ROBOT_BRINGUP_DEMO_ROSBRIDGE_SERVICE = TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-demo-rosbridge.service.in"
TIE_ROBOT_BRINGUP_DEMO_MODE_INSTALLER = TIE_ROBOT_BRINGUP_DIR / "scripts" / "install_demo_mode_service.sh"
TIE_ROBOT_BRINGUP_ROSBRIDGE_SERVICE = TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-rosbridge.service.in"
TIE_ROBOT_BRINGUP_ROSBRIDGE_INSTALLER = TIE_ROBOT_BRINGUP_DIR / "scripts" / "install_rosbridge_service.sh"
TIE_ROBOT_BRINGUP_ROSBRIDGE_STACK = TIE_ROBOT_BRINGUP_DIR / "launch" / "rosbridge_stack.launch"
TIE_ROBOT_BRINGUP_DEMO_ROSBRIDGE_LAUNCH = TIE_ROBOT_BRINGUP_DIR / "launch" / "demo_rosbridge_light.launch"
TIE_ROBOT_BRINGUP_TF_STACK = TIE_ROBOT_BRINGUP_DIR / "launch" / "tf_stack.launch"
TIE_ROBOT_BRINGUP_DRIVER_INSTALLER = TIE_ROBOT_BRINGUP_DIR / "scripts" / "install_driver_services.sh"
TIE_ROBOT_BRINGUP_DRIVER_SUDOERS = TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-driver-control.sudoers.in"
TIE_ROBOT_DRIVER_SERVICE_NAMES = (
    "tie-robot-driver-suoqu.service",
    "tie-robot-driver-moduan.service",
    "tie-robot-driver-camera.service",
)
FRONTEND_DIR = TIE_ROBOT_WEB_DIR / "web"
SERVER_SCRIPT = TIE_ROBOT_WEB_DIR / "scripts" / "workspace_picker_web_server.py"
OPEN_SCRIPT = TIE_ROBOT_WEB_DIR / "scripts" / "workspace_picker_web_open.py"
FRONTEND_SRC_DIR = TIE_ROBOT_WEB_DIR / "frontend" / "src"
FRONTEND_PROJECT_GRAPH_HTML = TIE_ROBOT_WEB_DIR / "frontend" / "project-graph.html"
FRONTEND_PROJECT_GRAPH_DIR = FRONTEND_SRC_DIR / "projectGraph"
CONTROL_PANEL_CATALOG = FRONTEND_SRC_DIR / "config" / "controlPanelCatalog.js"
IMAGE_TOPIC_CATALOG = FRONTEND_SRC_DIR / "config" / "imageTopicCatalog.js"
LOG_TOPIC_CATALOG = FRONTEND_SRC_DIR / "config" / "logTopicCatalog.js"
TOPIC_REGISTRY = FRONTEND_SRC_DIR / "config" / "topicRegistry.js"
SYSTEM_CONTROL_CATALOG = FRONTEND_SRC_DIR / "config" / "systemControlCatalog.js"
STATUS_MONITOR_CATALOG = FRONTEND_SRC_DIR / "config" / "statusMonitorCatalog.js"
PANEL_REGISTRY = FRONTEND_SRC_DIR / "panels" / "panelRegistry.js"
DEFAULT_LAYOUTS = FRONTEND_SRC_DIR / "layout" / "defaultLayouts.js"
LEGACY_COMMAND_CATALOG = FRONTEND_SRC_DIR / "config" / "legacyCommandCatalog.js"
STORAGE_UTILS = FRONTEND_SRC_DIR / "utils" / "storage.js"


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
        self.assertTrue((FRONTEND_DIR / "project-graph.html").exists())
        self.assertTrue((FRONTEND_DIR / "help" / "index.html").exists())
        self.assertTrue(any(FRONTEND_DIR.glob("assets/app/index-*.js")))
        self.assertTrue(any(FRONTEND_DIR.glob("assets/app/index-*.css")))

    def test_gitnexus_project_graph_page_exists(self):
        project_graph_html = FRONTEND_PROJECT_GRAPH_HTML.read_text(encoding="utf-8")
        project_graph_main = (
            FRONTEND_PROJECT_GRAPH_DIR / "main.js"
        ).read_text(encoding="utf-8")
        project_graph_data = (
            FRONTEND_PROJECT_GRAPH_DIR / "graphData.js"
        ).read_text(encoding="utf-8")
        project_graph_style = (
            FRONTEND_PROJECT_GRAPH_DIR / "style.css"
        ).read_text(encoding="utf-8")
        vite_config = (
            TIE_ROBOT_WEB_DIR / "frontend" / "vite.config.js"
        ).read_text(encoding="utf-8")

        self.assertIn("<title>GitNexus 工程关系图</title>", project_graph_html)
        self.assertIn("./src/projectGraph/main.js", project_graph_html)
        self.assertIn("projectGraph: resolve(__dirname, \"project-graph.html\")", vite_config)
        self.assertIn("gitnexusSummary", project_graph_data)
        self.assertIn("29351", project_graph_data)
        self.assertIn("46164", project_graph_data)
        for package_name in (
            "tie_robot_msgs",
            "tie_robot_hw",
            "tie_robot_perception",
            "tie_robot_process",
            "tie_robot_control",
            "tie_robot_web",
            "tie_robot_bringup",
            "tie_robot_description",
        ):
            self.assertIn(package_name, project_graph_data)
        self.assertIn("renderPackageGraph", project_graph_main)
        self.assertIn("data-edge-from", project_graph_main)
        self.assertIn("关系强度", project_graph_main)
        self.assertIn("font-family: \"Iosevka\"", project_graph_style)
        self.assertIn("radial-gradient", project_graph_style)

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
        frontend_launch = (
            TIE_ROBOT_BRINGUP_DIR / "launch" / "frontend.launch"
        ).read_text(encoding="utf-8")

        self.assertNotIn("workspace_picker_web_server", run_launch)
        self.assertNotIn("workspace_picker_web_open", run_launch)
        self.assertNotIn("workspace_picker_port", run_launch)
        self.assertNotIn("auto_open_workspace_picker", run_launch)
        self.assertNotIn('driver_stack.launch', run_launch)
        self.assertIn('algorithm_stack.launch', run_launch)
        self.assertIn('driver_suoqu.launch', driver_stack_launch)
        self.assertIn('driver_moduan.launch', driver_stack_launch)
        self.assertIn('driver_camera.launch', driver_stack_launch)
        self.assertIn('<arg name="workspace_picker_port" default="8080"', frontend_launch)
        self.assertIn('name="workspace_picker_web_server"', frontend_launch)
        self.assertIn('name="workspace_picker_web_open"', frontend_launch)
        self.assertIn('respawn="true"', frontend_launch)
        self.assertIn('driver_suoqu.launch', driver_stack_launch)
        self.assertNotIn('name="camera_origin_z_offset_mm"', driver_stack_launch)
        self.assertNotIn('name="base_link_z_offset_mm"', driver_stack_launch)
        self.assertIn('driver_moduan.launch', driver_stack_launch)
        self.assertIn('driver_camera.launch', driver_stack_launch)
        self.assertIn('name="pointAINode"', algorithm_stack_launch)
        self.assertNotIn('name="stable_point_tf_broadcaster"', algorithm_stack_launch)
        self.assertIn('name="web_action_bridge_node"', api_launch)
        self.assertIn('type="web_action_bridge_node"', api_launch)
        self.assertNotIn('name="webActionBridgeNode"', api_launch)
        self.assertNotIn('type="webActionBridgeNode"', api_launch)
        self.assertNotIn('topicTransNode', api_launch)
        self.assertNotIn('topictransNode', api_launch)
        self.assertIn('name="system_log_mux"', api_launch)
        self.assertIn('respawn="true"', api_launch)
        self.assertIn('name="scepter_manager"', camera_launch)
        self.assertIn('name="scepter_world_coord_processor"', camera_launch)
        self.assertIn('<arg name="camera_respawn" default="false"', camera_launch)
        self.assertIn('respawn="$(arg camera_respawn)"', camera_launch)
        self.assertNotIn('required="true"', camera_launch)

    def test_frontend_has_systemd_boot_autostart_guard(self):
        bringup_cmake = (TIE_ROBOT_BRINGUP_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        service_template = TIE_ROBOT_BRINGUP_FRONTEND_SERVICE.read_text(encoding="utf-8")
        installer = TIE_ROBOT_BRINGUP_FRONTEND_AUTOSTART_INSTALLER.read_text(encoding="utf-8")

        self.assertIn("Restart=always", service_template)
        self.assertIn("RestartSec=3", service_template)
        self.assertIn("workspace_picker_web_server.py --no-ros", service_template)
        self.assertIn("--port 8080", service_template)
        self.assertIn('$$(hostname -I)', service_template)
        self.assertIn('$${FRONTEND_HOST%%%% *}', service_template)
        self.assertIn('$${FRONTEND_HOST:-127.0.0.1}', service_template)
        self.assertNotIn("roslaunch tie_robot_bringup run.launch", service_template)
        self.assertNotIn("roslaunch tie_robot_bringup frontend.launch", service_template)
        self.assertIn("@WORKSPACE@", service_template)
        self.assertIn("@USER@", service_template)
        self.assertIn("systemctl daemon-reload", installer)
        self.assertIn("systemctl enable tie-robot-frontend.service", installer)
        self.assertIn("systemctl restart tie-robot-frontend.service", installer)
        self.assertIn("systemd", bringup_cmake)
        self.assertIn("scripts", bringup_cmake)
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('parser.add_argument("--no-ros", action="store_true"', server_script)
        self.assertIn("run_without_ros=", server_script)

    def test_ros_backend_is_systemd_managed_from_frontend(self):
        backend_service = TIE_ROBOT_BRINGUP_BACKEND_SERVICE.read_text(encoding="utf-8")
        backend_sudoers = TIE_ROBOT_BRINGUP_BACKEND_SUDOERS.read_text(encoding="utf-8")
        backend_installer = TIE_ROBOT_BRINGUP_BACKEND_INSTALLER.read_text(encoding="utf-8")
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        system_control_catalog = SYSTEM_CONTROL_CATALOG.read_text(encoding="utf-8")
        app_logic = (FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js").read_text(encoding="utf-8")
        ui_controller = (FRONTEND_SRC_DIR / "ui" / "UIController.js").read_text(encoding="utf-8")
        app_css = (FRONTEND_SRC_DIR / "styles" / "app.css").read_text(encoding="utf-8")

        self.assertIn("Description=Tie Robot ROS backend stack", backend_service)
        self.assertIn("roslaunch tie_robot_bringup run.launch", backend_service)
        self.assertIn("KillSignal=SIGINT", backend_service)
        self.assertIn("KillMode=control-group", backend_service)
        self.assertIn("TimeoutStopSec=5", backend_service)
        self.assertIn("Restart=on-failure", backend_service)
        self.assertNotIn("workspace_picker_web_server", backend_service)

        self.assertIn("tie-robot-backend.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl start tie-robot-backend.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl stop tie-robot-backend.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl restart tie-robot-backend.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl start tie-robot-rosbridge.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl stop tie-robot-rosbridge.service", backend_sudoers)
        for service_name in TIE_ROBOT_DRIVER_SERVICE_NAMES:
            self.assertIn(f"/usr/bin/systemctl restart {service_name}", backend_sudoers)
        self.assertIn(
            "/usr/bin/systemctl stop tie-robot-backend.service "
            "tie-robot-driver-suoqu.service tie-robot-driver-moduan.service "
            "tie-robot-driver-camera.service tie-robot-rosbridge.service",
            backend_sudoers,
        )
        self.assertIn("visudo -cf", backend_installer)
        self.assertIn("tie-robot-backend.service", backend_installer)
        self.assertIn("tie-robot-backend-control", backend_installer)
        self.assertNotIn("systemctl enable tie-robot-backend.service", backend_installer)

        self.assertIn('ROS_BACKEND_SERVICE = "tie-robot-backend.service"', server_script)
        self.assertIn('ROSBRIDGE_SERVICE = "tie-robot-rosbridge.service"', server_script)
        self.assertIn("FULL_ROS_STACK_STOP_ORDER = (", server_script)
        self.assertIn("FULL_ROS_STACK_START_ORDER = (", server_script)
        self.assertIn('"/api/system/start_ros_stack": "start"', server_script)
        self.assertIn('"/api/system/stop_ros_stack": "stop"', server_script)
        self.assertIn('"/api/system/restart_ros_stack": FULL_ROS_STACK_RESTART_ACTION', server_script)
        self.assertIn("handle_full_ros_stack_restart", server_script)
        self.assertIn('"/api/system/ros_stack_status"', server_script)
        self.assertIn("sudo", server_script)
        self.assertIn("systemctl", server_script)
        self.assertNotIn('"/api/system/start_ros_stack": WORKSPACE_ROOT / "start_.sh"', server_script)
        self.assertNotIn('"/api/system/restart_ros_stack": WORKSPACE_ROOT / "restart_ros_stack.sh"', server_script)

        self.assertIn('id: "stopRosStack"', system_control_catalog)
        self.assertIn('description: "快速停止 rosbridge、驱动层和 ROS 后端，再按依赖顺序重新启动"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/stop_ros_stack"', system_control_catalog)
        self.assertIn('serviceKey: null', system_control_catalog)
        self.assertNotIn('serviceKey: "startAlgorithmStackService"', system_control_catalog)
        self.assertNotIn('serviceKey: "restartAlgorithmStackService"', system_control_catalog)
        self.assertIn('onPendingChange: (actionId, pending) => this.ui.setSystemActionPending(actionId, pending)', app_logic)
        self.assertIn('data-voltage-action="startRosStack"', ui_controller)
        self.assertIn("setSystemActionPending(actionId, pending)", ui_controller)
        self.assertIn("toolbar-voltage-badge is-booting", ui_controller)
        self.assertIn(".toolbar-voltage-badge.is-booting", app_css)

    def test_rosbridge_has_independent_systemd_boot_autostart_guard(self):
        run_launch = (TIE_ROBOT_BRINGUP_DIR / "launch" / "run.launch").read_text(
            encoding="utf-8"
        )
        api_launch = (TIE_ROBOT_BRINGUP_DIR / "launch" / "api.launch").read_text(
            encoding="utf-8"
        )
        rosbridge_launch = TIE_ROBOT_BRINGUP_ROSBRIDGE_STACK.read_text(encoding="utf-8")
        rosbridge_service = TIE_ROBOT_BRINGUP_ROSBRIDGE_SERVICE.read_text(encoding="utf-8")
        rosbridge_installer = TIE_ROBOT_BRINGUP_ROSBRIDGE_INSTALLER.read_text(encoding="utf-8")
        frontend_installer = TIE_ROBOT_BRINGUP_FRONTEND_AUTOSTART_INSTALLER.read_text(
            encoding="utf-8"
        )
        frontend_service = TIE_ROBOT_BRINGUP_FRONTEND_SERVICE.read_text(encoding="utf-8")
        bringup_cmake = (TIE_ROBOT_BRINGUP_DIR / "CMakeLists.txt").read_text(
            encoding="utf-8"
        )

        self.assertNotIn("rosbridge_websocket", run_launch)
        self.assertNotIn("rosbridge_websocket", api_launch)
        self.assertNotIn("rosapi_node", api_launch)
        self.assertNotIn("tf2_web_republisher", api_launch)

        self.assertIn('name="rosbridge_websocket"', rosbridge_launch)
        self.assertIn('pkg="rosbridge_server"', rosbridge_launch)
        self.assertIn('type="rosbridge_websocket"', rosbridge_launch)
        self.assertIn('name="rosapi"', rosbridge_launch)
        self.assertIn('type="rosapi_node"', rosbridge_launch)
        self.assertIn('respawn="true"', rosbridge_launch)
        self.assertIn('$(find tie_robot_bringup)/launch/tf_stack.launch', rosbridge_launch)
        self.assertNotIn('name="tf2_web_republisher"', rosbridge_launch)
        self.assertNotIn('name="gripper_tf_broadcaster"', rosbridge_launch)

        self.assertIn("Description=Tie Robot ROS bridge websocket stack", rosbridge_service)
        self.assertIn("roslaunch tie_robot_bringup rosbridge_stack.launch", rosbridge_service)
        self.assertIn("Restart=always", rosbridge_service)
        self.assertIn("KillSignal=SIGINT", rosbridge_service)
        self.assertIn("TimeoutStopSec=5", rosbridge_service)
        self.assertNotIn("roslaunch tie_robot_bringup run.launch", rosbridge_service)
        self.assertNotIn("workspace_picker_web_server", rosbridge_service)

        self.assertIn("tie-robot-rosbridge.service", rosbridge_installer)
        self.assertIn("systemctl enable tie-robot-rosbridge.service", rosbridge_installer)
        self.assertIn("systemctl restart tie-robot-rosbridge.service", rosbridge_installer)
        self.assertIn("install_rosbridge_service.sh", frontend_installer)
        self.assertIn("tie-robot-rosbridge.service", frontend_service)
        self.assertIn("scripts/install_rosbridge_service.sh", bringup_cmake)

    def test_tf_layer_is_guarded_with_rosbridge_stack(self):
        rosbridge_launch = TIE_ROBOT_BRINGUP_ROSBRIDGE_STACK.read_text(encoding="utf-8")
        tf_stack_launch = TIE_ROBOT_BRINGUP_TF_STACK.read_text(encoding="utf-8")
        driver_suoqu_launch = (
            TIE_ROBOT_BRINGUP_DIR / "launch" / "driver_suoqu.launch"
        ).read_text(encoding="utf-8")
        driver_camera_launch = (
            TIE_ROBOT_BRINGUP_DIR / "launch" / "driver_camera.launch"
        ).read_text(encoding="utf-8")
        perception_cmake = (
            TIE_ROBOT_PERCEPTION_DIR / "CMakeLists.txt"
        ).read_text(encoding="utf-8")
        robot_tf_broadcaster = (
            TIE_ROBOT_PERCEPTION_DIR / "scripts" / "robot_tf_broadcaster.py"
        ).read_text(encoding="utf-8")

        self.assertIn('$(find tie_robot_bringup)/launch/tf_stack.launch', rosbridge_launch)
        self.assertNotIn('name="tf2_web_republisher"', rosbridge_launch)
        self.assertIn('name="tf2_web_republisher"', tf_stack_launch)
        self.assertIn('type="tf2_web_republisher"', tf_stack_launch)
        self.assertNotIn('name="robot_tf_broadcaster"', driver_suoqu_launch)
        self.assertIn('name="robot_tf_broadcaster"', tf_stack_launch)
        self.assertIn('type="robot_tf_broadcaster.py"', tf_stack_launch)
        self.assertNotIn('name="gripper_tf_broadcaster"', rosbridge_launch)
        self.assertNotIn('name="gripper_tf_broadcaster"', driver_camera_launch)
        self.assertIn('name="gripper_tf_broadcaster"', tf_stack_launch)
        self.assertIn('type="gripper_tf_broadcaster.py"', tf_stack_launch)
        self.assertIn('respawn="true"', tf_stack_launch)
        self.assertIn('cabin_state_topic', tf_stack_launch)
        self.assertIn('/cabin/cabin_data_upload', tf_stack_launch)
        self.assertIn('$(find tie_robot_perception)/config/gripper_tf.yaml', tf_stack_launch)
        self.assertIn("scripts/gripper_tf_broadcaster.py", perception_cmake)
        self.assertIn("scripts/robot_tf_broadcaster.py", perception_cmake)
        self.assertIn("rospy.Subscriber(cabin_state_topic", robot_tf_broadcaster)
        self.assertIn("base_transform.header.frame_id = map_frame", robot_tf_broadcaster)
        self.assertIn("scepter_transform.header.frame_id = base_link_frame", robot_tf_broadcaster)
        self.assertIn("quaternion_from_euler(math.pi, 0.0, 0.0)", robot_tf_broadcaster)
        self.assertIn("broadcaster.sendTransform([base_transform, scepter_transform])", robot_tf_broadcaster)

    def test_drivers_have_independent_systemd_guard_and_frontend_controls(self):
        run_launch = (TIE_ROBOT_BRINGUP_DIR / "launch" / "run.launch").read_text(
            encoding="utf-8"
        )
        rosbridge_stack_launch = TIE_ROBOT_BRINGUP_ROSBRIDGE_STACK.read_text(
            encoding="utf-8"
        )
        driver_stack_launch = TIE_ROBOT_BRINGUP_DRIVER_STACK.read_text(encoding="utf-8")
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        system_control_catalog = SYSTEM_CONTROL_CATALOG.read_text(encoding="utf-8")
        frontend_installer = TIE_ROBOT_BRINGUP_FRONTEND_AUTOSTART_INSTALLER.read_text(
            encoding="utf-8"
        )
        driver_installer = TIE_ROBOT_BRINGUP_DRIVER_INSTALLER.read_text(encoding="utf-8")
        driver_sudoers = TIE_ROBOT_BRINGUP_DRIVER_SUDOERS.read_text(encoding="utf-8")
        bringup_cmake = (TIE_ROBOT_BRINGUP_DIR / "CMakeLists.txt").read_text(
            encoding="utf-8"
        )

        self.assertNotIn("driver_stack.launch", run_launch)
        self.assertIn("algorithm_stack.launch", run_launch)
        self.assertNotIn("api.launch", run_launch)
        self.assertIn("api.launch", rosbridge_stack_launch)
        self.assertIn('driver_suoqu.launch', driver_stack_launch)
        self.assertIn('driver_moduan.launch', driver_stack_launch)
        self.assertIn('driver_camera.launch', driver_stack_launch)

        for service_name in TIE_ROBOT_DRIVER_SERVICE_NAMES:
            service_template = (
                TIE_ROBOT_BRINGUP_DIR / "systemd" / f"{service_name}.in"
            ).read_text(encoding="utf-8")
            self.assertIn(f"Description=Tie Robot {service_name}", service_template)
            self.assertIn("Restart=always", service_template)
            self.assertIn("RestartSec=3", service_template)
            self.assertIn("StartLimitIntervalSec=0", service_template)
            self.assertIn("KillSignal=SIGINT", service_template)
            self.assertIn("KillMode=control-group", service_template)
            self.assertIn("TimeoutStopSec=5", service_template)
            self.assertNotIn("run.launch", service_template)
            self.assertIn(service_name, driver_installer)
            self.assertIn(f"/usr/bin/systemctl start {service_name}", driver_sudoers)
            self.assertIn(f"/usr/bin/systemctl stop {service_name}", driver_sudoers)
            self.assertIn(f"/usr/bin/systemctl restart {service_name}", driver_sudoers)

        self.assertIn("roslaunch tie_robot_bringup driver_suoqu.launch", (
            TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-driver-suoqu.service.in"
        ).read_text(encoding="utf-8"))
        self.assertIn("roslaunch tie_robot_bringup driver_moduan.launch", (
            TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-driver-moduan.service.in"
        ).read_text(encoding="utf-8"))
        self.assertIn("roslaunch tie_robot_bringup driver_camera.launch", (
            TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-driver-camera.service.in"
        ).read_text(encoding="utf-8"))

        self.assertIn("install_driver_services.sh", frontend_installer)
        self.assertIn("scripts/install_driver_services.sh", bringup_cmake)
        self.assertIn("tie-robot-driver-control", driver_installer)
        self.assertIn("systemctl enable tie-robot-driver-suoqu.service", driver_installer)
        self.assertIn("systemctl enable tie-robot-driver-moduan.service", driver_installer)
        self.assertIn("systemctl enable tie-robot-driver-camera.service", driver_installer)

        self.assertIn('DRIVER_SYSTEMD_SERVICES = {', server_script)
        self.assertIn('"cabin": "tie-robot-driver-suoqu.service"', server_script)
        self.assertIn('"moduan": "tie-robot-driver-moduan.service"', server_script)
        self.assertIn('"camera": "tie-robot-driver-camera.service"', server_script)
        self.assertIn('"/api/system/start_driver_stack": ("start", "all")', server_script)
        self.assertIn('"/api/system/stop_driver_stack": ("stop", "all")', server_script)
        self.assertIn('"/api/system/restart_camera_driver": ("restart", "camera")', server_script)
        self.assertNotIn('"/api/system/start_driver_stack": WORKSPACE_ROOT / "start_driver_stack.sh"', server_script)
        self.assertNotIn('"/api/system/restart_driver_stack": WORKSPACE_ROOT / "restart.sh"', server_script)

        self.assertIn('id: "stopDriverStack"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/start_cabin_driver"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/stop_moduan_driver"', system_control_catalog)
        self.assertIn('id: "startCameraDriver"', system_control_catalog)
        self.assertIn('id: "stopCameraDriver"', system_control_catalog)
        self.assertIn('id: "restartCameraDriver"', system_control_catalog)
        self.assertNotIn('serviceKey: "startCabinDriverService"', system_control_catalog)
        self.assertNotIn('serviceKey: "restartModuanDriverService"', system_control_catalog)

        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertNotIn('{ id: "driverGuard", label: "驱动守护" }', ui_controller)
        self.assertNotIn('<option value="driverGuard">驱动守护</option>', ui_controller)
        self.assertNotIn('data-settings-page="driverGuard"', ui_controller)
        self.assertNotIn('data-system-action="startDriverStack"', ui_controller)
        self.assertNotIn('data-system-action="stopCabinDriver"', ui_controller)
        self.assertNotIn('data-system-action="restartModuanDriver"', ui_controller)
        self.assertNotIn('data-system-action="startCameraDriver"', ui_controller)
        self.assertNotIn('data-system-action="stopCameraDriver"', ui_controller)
        self.assertIn("onSystemAction(callback)", ui_controller)
        self.assertIn("this.refs.systemActionButtons", ui_controller)
        self.assertIn("this.ui.onSystemAction((actionId) => {", app_logic)
        self.assertIn("this.systemControlController.handle(actionId);", app_logic)
        self.assertNotIn(".driver-guard-grid {", app_css)
        self.assertNotIn(".driver-guard-row {", app_css)

    def test_gb28181_settings_writes_local_config_as_operator_action(self):
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        panel_manager = (
            FRONTEND_SRC_DIR / "ui" / "PanelManager.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn('"/api/gb28181/config"', server_script)
        self.assertIn("handle_gb28181_config_post", server_script)
        self.assertIn('id="gb28181WriteConfig"', ui_controller)
        self.assertIn("writeGb28181Config()", ui_controller)
        self.assertIn('fetch("/api/gb28181/config"', ui_controller)
        self.assertIn(".gb28181-local-status", app_css)
        self.assertNotIn("gb28181ConfigPreview", ui_controller)
        self.assertNotIn("gb28181-local-command", ui_controller)
        self.assertNotIn("source /opt/ros", ui_controller)
        self.assertNotIn("roslaunch tie_robot_bringup api.launch start_gb28181", ui_controller)
        self.assertNotIn("src/tie_robot_gb28181/config/gb28181_device.yaml", ui_controller)

    def test_network_ping_api_uses_safe_ping_command(self):
        module = self.server_module
        self.assertEqual(module.normalize_ping_host(" 192.168.6.62 "), "192.168.6.62")
        self.assertEqual(module.normalize_ping_host("linear-module.local"), "linear-module.local")

        for bad_host in ("", "192.168.6.62;reboot", "bad host", "-c 99 192.168.6.62", "x" * 254):
            with self.assertRaises(ValueError):
                module.normalize_ping_host(bad_host)

        captured = {}
        original_run = module.subprocess.run
        original_ping_bin = module.PING_BIN

        def fake_run(command, **kwargs):
            captured["command"] = command
            captured["kwargs"] = kwargs
            return module.subprocess.CompletedProcess(
                command,
                0,
                stdout=(
                    "3 packets transmitted, 3 received, 0% packet loss\n"
                    "rtt min/avg/max/mdev = 0.100/0.200/0.300/0.010 ms\n"
                ),
                stderr="",
            )

        try:
            module.PING_BIN = "/bin/ping"
            module.subprocess.run = fake_run
            result = module.run_network_ping("192.168.6.62")
        finally:
            module.subprocess.run = original_run
            module.PING_BIN = original_ping_bin

        self.assertEqual(captured["command"], ["/bin/ping", "-c", "3", "-W", "1", "192.168.6.62"])
        self.assertNotIn("shell", captured["kwargs"])
        self.assertEqual(captured["kwargs"]["timeout"], 6)
        self.assertTrue(result["success"])
        self.assertTrue(result["reachable"])
        self.assertIn("0% packet loss", result["summary"])

    def test_settings_page_has_manual_network_ping_panel(self):
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        storage_utils = STORAGE_UTILS.read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn('"/api/network/ping"', server_script)
        self.assertIn("handle_network_ping_post", server_script)
        self.assertIn("run_network_ping", server_script)
        self.assertIn('{ id: "networkPing", label: "网络配置" }', ui_controller)
        self.assertIn('data-settings-page="networkPing"', ui_controller)
        self.assertIn('id="networkPingCabinHost"', ui_controller)
        self.assertIn('id="networkPingModuanHost"', ui_controller)
        self.assertIn("上位机 IP", ui_controller)
        self.assertIn('buttonLabel: "保存并测试"', ui_controller)
        self.assertIn('data-state="idle"', ui_controller)
        self.assertIn('data-network-ping-target="cabin"', ui_controller)
        self.assertIn('data-network-ping-target="moduan"', ui_controller)
        self.assertIn("getNetworkPingSettings()", ui_controller)
        self.assertIn("setNetworkPingResult(targetId, result)", ui_controller)
        self.assertIn("button.dataset.state = state;", ui_controller)
        self.assertIn(
            "settings[target.settingsKey] = value == null ? target.defaultHost : String(value).trim();",
            ui_controller,
        )
        self.assertIn("连接成功", ui_controller)
        self.assertIn("连接失败", ui_controller)
        self.assertNotIn('class="network-ping-result mono"', ui_controller)
        self.assertNotIn("result?.stdout, result?.stderr", ui_controller)
        self.assertIn("onNetworkPingTest(callback)", ui_controller)
        self.assertIn("loadNetworkPingSettings", storage_utils)
        self.assertIn("saveNetworkPingSettings", storage_utils)
        self.assertIn('cabinHost: "192.168.6.62"', storage_utils)
        self.assertIn('moduanHost: "192.168.6.167"', storage_utils)
        self.assertIn("this.networkPingSettings = loadNetworkPingSettings();", app_logic)
        self.assertIn('fetch("/api/network/ping"', app_logic)
        self.assertIn("handleNetworkPingTest(targetId)", app_logic)
        self.assertIn(".network-ping-grid", app_css)
        self.assertIn('.network-ping-button[data-state="success"]', app_css)
        self.assertIn('.network-ping-button[data-state="error"]', app_css)
        self.assertIn(".network-ping-result", app_css)

    def test_scene_view_modes_are_free_camera_top_and_follow_origin(self):
        topic_layer_catalog = (
            FRONTEND_SRC_DIR / "config" / "topicLayerCatalog.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")
        topic_layer_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TopicLayerController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn('{ id: "free", label: "自由视角" }', topic_layer_catalog)
        self.assertIn('{ id: "camera", label: "相机视角" }', topic_layer_catalog)
        self.assertIn('{ id: "top", label: "俯视视角" }', topic_layer_catalog)
        self.assertIn('viewMode: "free"', topic_layer_catalog)
        self.assertIn("followOrigin: false", topic_layer_catalog)
        self.assertNotIn("followCamera", topic_layer_catalog)

        self.assertIn('class="settings-grid scene-view-grid"', ui_controller)
        self.assertIn('class="settings-section scene-view-card"', ui_controller)
        self.assertIn('class="scene-view-mode-group"', ui_controller)
        self.assertIn('data-scene-view-mode="${mode.id}"', ui_controller)
        self.assertIn('id="followOriginToggle"', ui_controller)
        self.assertIn("跟随原点", ui_controller)
        self.assertNotIn("跟随相机", ui_controller)
        self.assertIn(".scene-view-grid", app_css)
        self.assertIn(".scene-view-mode-button", app_css)

        self.assertIn('this.viewMode = "free";', scene_view)
        self.assertIn("setFollowOrigin(enabled)", scene_view)
        self.assertIn("resolveSceneViewOrigin(viewMode = this.viewMode)", scene_view)
        self.assertIn("CAMERA_VIEW_FORWARD_AXIS.clone().applyQuaternion(scepterTransform.quaternion)", scene_view)
        self.assertIn("TOP_VIEW_FORWARD_AXIS", scene_view)
        self.assertIn("new THREE.Vector3(0, 0, -1)", scene_view)
        self.assertIn("this.controls.enabled = this.viewMode === \"free\";", scene_view)
        self.assertIn("this.camera.position.copy(position);", scene_view)
        self.assertIn("this.controls.target.copy(target);", scene_view)
        self.assertNotIn("setFollowCamera", scene_view)

        self.assertIn("this.sceneView.setFollowOrigin(this.state.followOrigin);", topic_layer_controller)
        self.assertNotIn("setFollowCamera", topic_layer_controller)

    def test_settings_page_dropdown_supports_drag_order_persistence(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")
        storage_utils = STORAGE_UTILS.read_text(encoding="utf-8")

        self.assertIn("SETTINGS_PAGE_ORDER_KEY", storage_utils)
        self.assertIn("loadSettingsPageOrderPreference", storage_utils)
        self.assertIn("saveSettingsPageOrderPreference", storage_utils)
        self.assertIn("settingsPageOrder", ui_controller)
        self.assertIn('layers: "scene"', ui_controller)
        self.assertIn('calibration: "visualDebug"', ui_controller)
        self.assertIn("renderSettingsPageOptionMarkup()", ui_controller)
        self.assertIn("handleSettingsPagePointerDown", ui_controller)
        self.assertIn("handleSettingsPagePointerMove", ui_controller)
        self.assertIn("createSettingsPageDragGhost", ui_controller)
        self.assertIn("updateSettingsPageDragGhost", ui_controller)
        self.assertIn("cleanupSettingsPageDragGhost", ui_controller)
        self.assertIn("animateSettingsPageOrderChange", ui_controller)
        self.assertIn("flushSettingsPagePointerDragFrame", ui_controller)
        self.assertIn("handleSettingsPageGlobalPointerUp", ui_controller)
        self.assertIn("handleSettingsPageGlobalDragAbort", ui_controller)
        self.assertIn('window.addEventListener("pointerup", this.handleSettingsPageGlobalPointerUp, true);', ui_controller)
        self.assertIn('window.addEventListener("blur", this.handleSettingsPageGlobalDragAbort, true);', ui_controller)
        self.assertIn('document.addEventListener("visibilitychange", this.handleSettingsPageGlobalDragAbort, true);', ui_controller)
        self.assertIn("requestAnimationFrame(() => this.flushSettingsPagePointerDragFrame())", ui_controller)
        self.assertIn("notify: false", ui_controller)
        self.assertIn("drag.orderChanged = true;", ui_controller)
        self.assertIn("moveSettingsPageOption", ui_controller)
        self.assertIn("onSettingsPageOrderChange", ui_controller)
        self.assertIn("getSettingsPageHomeFirstOrder", ui_controller)
        self.assertIn("notifySettingsPageOrderCommitted", ui_controller)
        self.assertIn("this.handleSettingsHomePageChange?.(normalizedHomePageId);", ui_controller)
        self.assertIn("notifyOrder: true", ui_controller)
        self.assertIn("reorder: false", ui_controller)
        self.assertIn("settings-page-drag-handle", ui_controller)
        self.assertIn("optionRow.draggable = false", ui_controller)
        self.assertIn("loadSettingsPageOrderPreference", app_logic)
        self.assertIn("saveSettingsPageOrderPreference", app_logic)
        self.assertIn("const homeFirstPageOrder = this.ui.getSettingsPageOrder();", app_logic)
        self.assertIn("saveSettingsPageOrderPreference(homeFirstPageOrder);", app_logic)
        self.assertIn(".settings-page-drag-handle", app_css)
        self.assertIn(".settings-page-option-row.is-dragging", app_css)
        self.assertIn(".settings-page-option-row.is-sorting", app_css)
        self.assertIn(".settings-page-drag-ghost", app_css)

    def test_rosbridge_reconnects_three_times_then_prompts_manual_retry(self):
        ros_connection = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
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

        self.assertIn("AUTO_RECONNECT_MAX_ATTEMPTS = 3", ros_connection)
        self.assertIn("AUTO_RECONNECT_DELAY_MS = 1500", ros_connection)
        self.assertIn("scheduleReconnect(rosbridgeUrl)", ros_connection)
        self.assertIn("showManualReconnectRequired(rosbridgeUrl)", ros_connection)
        self.assertIn("connect({ manual = false, autoReconnect = false } = {})", ros_connection)
        self.assertIn('this.connect({ autoReconnect: true });', ros_connection)
        self.assertIn('manual: "手动重连"', ui_controller)
        self.assertIn('reconnecting: "重连中"', ui_controller)
        self.assertIn('manual: { id: "manualRosReconnect", label: "手动重连" }', ui_controller)
        self.assertIn('reconnecting: { id: "manualRosReconnect", label: "立即重连" }', ui_controller)
        self.assertIn('if (actionId === "manualRosReconnect") {', app_logic)
        self.assertIn('this.rosConnectionController.connect({ manual: true });', app_logic)
        self.assertIn(".toolbar-connection-badge.reconnecting", app_css)
        self.assertIn(".toolbar-connection-badge.manual", app_css)

    def test_frontend_shell_uses_task_only_left_panel_top_status_and_bottom_quick_controls(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        panel_manager = (
            FRONTEND_SRC_DIR / "ui" / "PanelManager.js"
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
        self.assertIn('id="bottomCabinPosition"', ui_controller)
        self.assertIn('class="bottom-cabin-position mono"', ui_controller)
        self.assertIn('data-bottom-cabin-axis="x"', ui_controller)
        self.assertIn('data-bottom-cabin-axis="y"', ui_controller)
        self.assertIn('data-bottom-cabin-axis="z"', ui_controller)
        self.assertIn('id="bottomLinearModulePosition"', ui_controller)
        self.assertIn('class="bottom-linear-module-position mono"', ui_controller)
        self.assertIn('data-bottom-linear-axis="local:x"', ui_controller)
        self.assertIn('data-bottom-linear-axis="local:y"', ui_controller)
        self.assertIn('data-bottom-linear-axis="local:z"', ui_controller)
        self.assertIn('data-bottom-linear-axis="global:x"', ui_controller)
        self.assertIn('data-bottom-linear-axis="global:y"', ui_controller)
        self.assertIn('data-bottom-linear-axis="global:z"', ui_controller)
        self.assertIn('id="quickControlGrid" class="quick-control-grid"', ui_controller)
        self.assertIn("quick-control-dock", ui_controller)
        self.assertLess(ui_controller.index('class="quick-control-dock"'), ui_controller.index('id="bottomCabinPosition"'))
        self.assertLess(ui_controller.index('id="bottomCabinPosition"'), ui_controller.index('id="quickControlGrid" class="quick-control-grid"'))
        self.assertLess(ui_controller.index('id="bottomCabinPosition"'), ui_controller.index('id="bottomLinearModulePosition"'))
        self.assertLess(ui_controller.index('id="bottomLinearModulePosition"'), ui_controller.index('id="quickControlGrid" class="quick-control-grid"'))
        self.assertNotIn('id="toolbarCabinPosition"', ui_controller)
        self.assertNotIn("toolbar-cabin-position", ui_controller)
        self.assertNotIn("快捷开关", ui_controller)
        self.assertIn("toolbar-status-strip", ui_controller)
        self.assertIn(".bottom-cabin-position {", app_css)
        self.assertIn("left: 50%;", app_css)
        self.assertIn("transform: translateX(-50%);", app_css)
        self.assertIn(".bottom-cabin-position[data-state=\"live\"]", app_css)
        self.assertIn(".bottom-linear-module-position {", app_css)
        self.assertIn(".bottom-linear-module-position[data-state=\"live\"]", app_css)
        self.assertIn(".bottom-linear-module-row {", app_css)
        self.assertIn(".bottom-linear-module-axis {", app_css)
        self.assertIn("toolbar-status-capsules", app_css)
        self.assertIn("quick-control-dock", app_css)
        self.assertIn("quick-control-grid", app_css)
        self.assertIn("flex-direction: column;", app_css)
        self.assertIn("pointer-events: none;", app_css)
        self.assertIn("pointer-events: auto;", app_css)
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
        self.assertIn("话题、工作区、图层、外参与国标接入", ui_controller)
        self.assertIn('id="settingsPageSelect"', ui_controller)
        self.assertNotIn('id="controlFeedback"', ui_controller)
        self.assertIn('{ id: "topics", label: "话题总览" }', ui_controller)
        self.assertIn('{ id: "workspace", label: "工作区选点" }', ui_controller)
        self.assertIn('{ id: "scene", label: "显示与视角" }', ui_controller)
        self.assertNotIn('{ id: "layers", label: "图层与数据" }', ui_controller)
        self.assertIn('layers: "scene"', ui_controller)
        self.assertNotIn('{ id: "calibration", label: "相机-TCP外参" }', ui_controller)
        self.assertIn('calibration: "visualDebug"', ui_controller)
        self.assertIn("renderSettingsPageOptionMarkup()", ui_controller)
        self.assertIn('data-settings-page="topics"', ui_controller)
        self.assertIn('data-settings-page="workspace"', ui_controller)
        self.assertIn('data-settings-page="scene"', ui_controller)
        self.assertNotIn('data-settings-page="layers"', ui_controller)
        self.assertNotIn('data-settings-page="calibration"', ui_controller)
        self.assertLess(ui_controller.index('data-settings-page="visualDebug"'), ui_controller.index('id="gripperTfCurrent"'))
        self.assertLess(ui_controller.index('id="gripperTfCurrent"'), ui_controller.index('id="visualDebugLogList"'))
        self.assertLess(ui_controller.index('data-settings-page="scene"'), ui_controller.index('id="topicLayerMode"'))
        self.assertLess(ui_controller.index('id="topicLayerStats"'), ui_controller.index('data-settings-page="homeCalibration"'))
        self.assertIn('id="topicInventoryList"', ui_controller)
        self.assertIn("topic-inventory-group", ui_controller)
        self.assertIn('const rootNamespace = normalizedSegments[0] ? `/${normalizedSegments[0]}` : "(根级话题)";', ui_controller)
        self.assertIn('data-topic-group-toggle="${groupKey}"', ui_controller)
        self.assertIn("bindTopicInventoryGroupToggles()", ui_controller)
        self.assertIn("相机-TCP外参", ui_controller)
        self.assertIn('settings-section gripper-tf-calibration-card', ui_controller)
        self.assertIn('gripper-tf-current', ui_controller)
        self.assertIn('gripper-tf-grid', ui_controller)
        self.assertIn('gripper-tf-actions', ui_controller)
        self.assertIn('id="gripperTfCurrent"', ui_controller)
        self.assertIn('id="gripperTfX"', ui_controller)
        self.assertIn('id="gripperTfY"', ui_controller)
        self.assertIn('id="gripperTfZ"', ui_controller)
        self.assertIn('id="applyGripperTfCalibration"', ui_controller)
        self.assertIn(".gripper-tf-calibration-card {", app_css)
        self.assertIn(".gripper-tf-current {", app_css)
        self.assertIn(".gripper-tf-grid {", app_css)
        self.assertIn(".gripper-tf-actions {", app_css)
        self.assertIn(".gripper-tf-actions .primary-btn {", app_css)
        self.assertIn("工作区选点", ui_controller)
        self.assertIn('id: "setRecognitionPose", label: "设为\\n识别位姿"', control_panel_catalog)
        self.assertIn('id: "moveToPosition", label: "移动到\\n位姿"', control_panel_catalog)
        self.assertIn("图层设置", ui_controller)
        self.assertIn("显示与视角", ui_controller)
        self.assertNotIn("图层与数据", ui_controller)
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
        legacy_command_catalog = LEGACY_COMMAND_CATALOG.read_text(encoding="utf-8")
        topic_registry = TOPIC_REGISTRY.read_text(encoding="utf-8")
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
        self.assertNotIn("this.rosConnectionController.triggerWorkspaceS2Refresh()", app_logic)
        self.assertNotIn("已按新外参自动重跑当前工作区识别", app_logic)
        self.assertIn("外参已热更新；视觉识别不会自动重跑，请手动点击“触发视觉识别”。", app_logic)
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
        self.assertIn('} from "../config/topicRegistry.js";', ros_connection)
        self.assertIn("applyDisplayedImageSubscription({ suppressLog = false } = {})", ros_connection)
        self.assertIn("updateDisplayedImageSubscription(topicName)", ros_connection)
        self.assertIn("applyLogSubscription({ suppressLog = false } = {})", ros_connection)
        self.assertIn("updateLogSubscription(topicId)", ros_connection)
        self.assertNotIn("gripperTfOffsetPublisher", ros_connection)
        self.assertNotIn("POINTAI_OFFSET_TOPIC", ros_connection)
        self.assertNotIn("/web/pointAI/set_offset", ros_connection)
        self.assertNotIn("/web/pointAI/set_gripper_tf_calibration", ros_connection)
        self.assertIn("SERVICES.tf.setGripperTfCalibration", ros_connection)
        self.assertIn("SERVICE_TYPES.tf.setGripperTfCalibration", ros_connection)
        self.assertIn("startTopicInventoryPolling()", ros_connection)
        self.assertIn("stopTopicInventoryPolling()", ros_connection)
        self.assertIn("fetchTopicInventory({ suppressLog = false } = {})", ros_connection)
        self.assertIn("this.ros.getTopicsAndRawTypes((result) => {", ros_connection)
        self.assertIn("registry: getTopicRegistryEntry(name),", ros_connection)
        self.assertNotIn("publishGripperTfOffsetMm({ x, y, z })", ros_connection)
        self.assertIn("setGripperTfCalibrationService: new ROSLIB.Service", ros_connection)
        self.assertIn("callGripperTfCalibrationService({ x, y, z })", ros_connection)
        self.assertIn("triggerWorkspaceS2Refresh()", ros_connection)
        self.assertIn("getLogTopicOption(this.desiredLogTopicId)", ros_connection)
        self.assertIn("this.callbacks.onSystemLog?.(message, option)", ros_connection)
        self.assertIn('queue_length: 30,', ros_connection)
        self.assertNotIn("const COLOR_TOPIC =", ros_connection)
        self.assertNotIn("const DEPTH_TOPIC =", ros_connection)
        self.assertIn("TOPICS.camera.colorImage", topic_registry)
        self.assertIn("TOPICS.camera.depthImage", topic_registry)
        self.assertIn("TOPICS.camera.irImage", image_topic_catalog)
        self.assertIn("TOPICS.algorithm.resultImageRaw", image_topic_catalog)
        self.assertIn("TOPICS.camera.colorImage", image_topic_catalog)
        self.assertIn("TOPICS.camera.depthImage", image_topic_catalog)
        self.assertIn("TOPICS.camera.filteredWorldCoord", image_topic_catalog)
        self.assertIn("TOPICS.camera.rawWorldCoord", image_topic_catalog)
        self.assertIn('id: "all"', log_topic_catalog)
        self.assertIn('id: "algorithm"', log_topic_catalog)
        self.assertIn('id: "cabin"', log_topic_catalog)
        self.assertIn('id: "moduan"', log_topic_catalog)
        self.assertIn('id: "camera"', log_topic_catalog)
        self.assertIn('id: "bridge"', log_topic_catalog)
        self.assertIn("topic: TOPICS.logs.all", log_topic_catalog)
        self.assertIn("topic: TOPICS.logs.cabin", log_topic_catalog)
        self.assertIn("topic: TOPICS.logs.moduan", log_topic_catalog)
        self.assertIn('"suoqu_driver_node"', log_topic_catalog)
        self.assertIn('"moduan_driver_node"', log_topic_catalog)
        self.assertIn('cabin: "/system_log/suoqu_driver_node"', topic_registry)
        self.assertIn('moduan: "/system_log/moduan_driver_node"', topic_registry)
        self.assertIn("web_action_bridge_node", log_topic_catalog)
        self.assertNotIn("topicTransNode", log_topic_catalog)
        self.assertNotIn("/web/pointAI/set_offset", legacy_command_catalog)
        self.assertIn("topic: TOPICS.tf.setOffset", legacy_command_catalog)
        self.assertIn('name: "修正TF外参"', legacy_command_catalog)
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
        self.assertIn("overflow: hidden;", app_css)
        self.assertIn(".panel-log {", app_css)
        self.assertIn(".panel-log-topic-field {", app_css)
        self.assertIn("left: calc(50vw - 350px);", app_css)
        self.assertIn("top: calc(100vh - 446px);", app_css)
        self.assertIn("width: 700px;", app_css)
        self.assertIn("height: 320px;", app_css)
        self.assertIn("width: 340px;", app_css)
        self.assertIn("min-width: 320px;", app_css)
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
        self.assertIn('if (nextVisible) {', app_logic)
        self.assertNotIn('if (panelId === "logPanel" && nextVisible) {', app_logic)
        self.assertIn('this.panelManager.applyDefaultPanelRect(panelId);', app_logic)

    def test_frontend_topic_registry_documents_direct_topic_ownership(self):
        topic_registry = TOPIC_REGISTRY.read_text(encoding="utf-8")
        ros_connection = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        image_topic_catalog = IMAGE_TOPIC_CATALOG.read_text(encoding="utf-8")
        topic_layer_catalog = (
            FRONTEND_SRC_DIR / "config" / "topicLayerCatalog.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn("export const TOPICS = Object.freeze({", topic_registry)
        self.assertIn("export const FRONTEND_DIRECT_TOPIC_REGISTRY =", topic_registry)
        self.assertIn('sourceLayer: "driver"', topic_registry)
        self.assertIn('sourceLayer: "algorithm"', topic_registry)
        self.assertIn('sourceLayer: "process"', topic_registry)
        self.assertIn('sourceLayer: "control"', topic_registry)
        self.assertIn('ownerNode: "scepter_manager"', topic_registry)
        self.assertIn('ownerNode: "pointAINode"', topic_registry)
        self.assertIn('ownerNode: "suoquNode"', topic_registry)
        self.assertIn('ownerNode: "moduanNode"', topic_registry)
        self.assertIn('direction: "subscribe"', topic_registry)
        self.assertIn('direction: "publish"', topic_registry)
        self.assertIn("getTopicRegistryEntry(topicName)", topic_registry)
        self.assertIn("getPointCloudTopicName(source)", topic_registry)
        self.assertIn("getTopicLayerSourceTopic(source)", topic_registry)
        self.assertIn("FRONTEND_DIRECT_TOPIC_REGISTRY", image_topic_catalog)
        self.assertIn("getTopicLayerSourceTopic", topic_layer_catalog)
        self.assertIn("getTopicRegistryEntry(name)", ros_connection)
        self.assertIn("topic-inventory-source", ui_controller)
        self.assertIn("topic-inventory-owner", ui_controller)
        self.assertIn("entry.registry?.sourceLabel", ui_controller)
        self.assertIn(".topic-inventory-source {", app_css)
        self.assertIn(".topic-inventory-owner {", app_css)

    def test_web_bridge_no_longer_runs_legacy_topic_conversion_node(self):
        api_launch = (TIE_ROBOT_BRINGUP_DIR / "launch" / "api.launch").read_text(
            encoding="utf-8"
        )
        web_cmake = (TIE_ROBOT_WEB_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        bridge_main = (TIE_ROBOT_WEB_DIR / "src" / "web_action_bridge.cpp").read_text(
            encoding="utf-8"
        )
        bridge_runtime_header = (
            TIE_ROBOT_WEB_DIR / "include" / "tie_robot_web" / "web_bridge" / "web_action_bridge_runtime.hpp"
        ).read_text(encoding="utf-8")
        bridge_node_app = (
            TIE_ROBOT_WEB_DIR / "src" / "web_bridge" / "node_app.cpp"
        ).read_text(encoding="utf-8")
        bridge_runtime = (
            TIE_ROBOT_WEB_DIR / "src" / "web_bridge" / "runtime.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("add_executable(${WEB_ACTION_BRIDGE_TARGET}", web_cmake)
        self.assertIn("WEB_ACTION_BRIDGE_NODE_SOURCES", web_cmake)
        self.assertIn("src/web_action_bridge.cpp", web_cmake)
        self.assertIn("install(TARGETS", web_cmake)
        self.assertIn("web_action_bridge_node", web_cmake)
        self.assertIn("webActionBridgeNode", web_cmake)
        self.assertIn('name="web_action_bridge_node"', api_launch)
        self.assertIn('type="web_action_bridge_node"', api_launch)
        self.assertIn("RunWebActionBridgeNode", bridge_main)
        self.assertIn("RunWebActionBridgeNode", bridge_runtime_header)
        self.assertIn('ros::init(argc, argv, "web_action_bridge_node");', bridge_node_app)
        self.assertIn("[web_action_bridge]", bridge_runtime)
        self.assertNotIn("topictransNode", web_cmake + api_launch)
        self.assertNotIn("topicTransNode", web_cmake + api_launch)
        self.assertNotIn("topics_transfer", web_cmake + api_launch + bridge_main + bridge_runtime_header + bridge_node_app + bridge_runtime)
        self.assertNotIn("subscriber_callbacks.cpp", web_cmake)
        self.assertNotIn("nh.subscribe(\"/web/pointAI/process_image\"", bridge_node_app)
        self.assertNotIn("nh.subscribe(\"/web/cabin/cabin_move_debug\"", bridge_node_app)
        self.assertNotIn("nh.subscribe(\"/web/moduan/moduan_move_debug\"", bridge_node_app)

    def test_settings_panel_remains_resizable_and_scrollable(self):
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn("resize: both;", app_css)
        self.assertIn(".panel-settings {\n  top: 88px;", app_css)
        self.assertIn("width: 340px;", app_css)
        self.assertIn("min-width: 320px;", app_css)
        self.assertIn("overflow: hidden;", app_css)
        self.assertIn(".panel-content {\n  flex: 1;", app_css)
        self.assertIn(".panel-settings .panel-content {\n  overflow: hidden;", app_css)
        self.assertIn(".settings-panel-content {\n  display: grid;", app_css)
        self.assertIn("overflow: hidden;", app_css)
        self.assertIn(".settings-page-stack {", app_css)
        self.assertIn("min-height: 0;", app_css)
        self.assertIn("overflow: auto;", app_css)
        self.assertNotIn("width: 128px;\n    min-width: 128px;", app_css)

    def test_settings_panel_has_layered_ros_log_page(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        ros_connection = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn('{ id: "logs", label: "节点日志" }', ui_controller)
        self.assertIn('{ id: "logs", label: "节点日志" }', ui_controller)
        self.assertIn('data-settings-page="logs"', ui_controller)
        self.assertIn('id="settingsLayerLogList"', ui_controller)
        self.assertIn("renderSettingsLayerLogs", ui_controller)
        self.assertIn("toggleSettingsLayerLogNode", ui_controller)
        self.assertIn('data-layer-log-node="${node.nodeName}"', ui_controller)
        self.assertIn('label: "驱动层"', app_logic)
        self.assertIn('label: "算法层"', app_logic)

        self.assertIn("this.layerSystemLogs = [];", app_logic)
        self.assertIn("handleLayerSystemLog(message)", app_logic)
        self.assertIn("buildSettingsLayerLogViewModel()", app_logic)
        self.assertIn("getLayerLogCategory(nodeName)", app_logic)
        self.assertIn('return "driver";', app_logic)
        self.assertIn('return "algorithm";', app_logic)
        self.assertIn("this.ui.renderSettingsLayerLogs(this.buildSettingsLayerLogViewModel());", app_logic)

        self.assertIn("this.settingsLogTopicSubscriber = null;", ros_connection)
        self.assertIn("this.buildTopic(TOPICS.logs.all, MESSAGE_TYPES.log", ros_connection)
        self.assertIn("this.callbacks.onLayerSystemLog?.(message)", ros_connection)
        self.assertIn("settingsLogTopicSubscriber.unsubscribe()", ros_connection)

        self.assertIn(".settings-layer-log-list {", app_css)
        self.assertIn(".settings-layer-log-card {", app_css)
        self.assertIn(".settings-layer-log-card.is-expanded", app_css)
        self.assertIn(".settings-layer-log-history {", app_css)

    def test_topic_layers_unify_planning_and_tie_points_for_operator_display(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        topic_layer_catalog = (
            FRONTEND_SRC_DIR / "config" / "topicLayerCatalog.js"
        ).read_text(encoding="utf-8")

        self.assertIn('label: "只显示规划点/绑扎点"', topic_layer_catalog)
        self.assertIn('label: "点云 + 规划点/绑扎点"', topic_layer_catalog)
        self.assertIn("showTiePoints: false", topic_layer_catalog)
        self.assertNotIn('id: "onlyTiePoints"', topic_layer_catalog)
        self.assertNotIn('id: "pointCloudAndTiePoints"', topic_layer_catalog)
        self.assertNotIn("<span>绑扎点</span>", ui_controller)
        self.assertNotIn("<span>规划点</span>", ui_controller)
        self.assertIn("<span>规划点/绑扎点</span>", ui_controller)
        self.assertNotIn("<span>绑扎点</span><strong>${stats.tiePointCount}</strong>", ui_controller)
        self.assertIn("<span>规划点/绑扎点</span><strong>${unifiedPlanningPointCount}</strong>", ui_controller)

    def test_settings_panel_supports_home_page_preference_without_control_panel_customization(self):
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
        self.assertNotIn('id="controlPanelCustomizeButton"', ui_controller)
        self.assertNotIn('id="controlPanelCustomizeMenu"', ui_controller)
        self.assertNotIn('id="controlPanelCustomButtonForm"', ui_controller)
        self.assertNotIn('id="controlPanelCustomButtonName"', ui_controller)
        self.assertNotIn('id="controlPanelCustomButtonService"', ui_controller)
        self.assertNotIn('id="controlPanelTrashBin"', ui_controller)
        self.assertIn("renderControlPanelTasks()", ui_controller)
        self.assertNotIn("renderControlPanelCustomizeMenu", ui_controller)
        self.assertIn('homeButton.dataset.settingsHomePage = option.value;', ui_controller)
        self.assertIn('homeButton.className = "settings-home-page-button";', ui_controller)
        self.assertNotIn("controlPanelCustomizeButton", ui_controller)
        self.assertNotIn("controlPanelCustomizeMenu", ui_controller)
        self.assertNotIn("controlPanelCustomButton", ui_controller)
        self.assertNotIn("controlPanelTrashBin", ui_controller)
        self.assertNotIn("data-custom-service-action", ui_controller)
        self.assertIn("onSettingsHomePageChange(callback)", ui_controller)
        self.assertIn("this.handleSettingsHomePageChange = callback;", ui_controller)
        self.assertNotIn("onControlPanelCustomButtonCreate(callback)", ui_controller)
        self.assertNotIn("onCustomControlPanelAction(callback)", ui_controller)
        self.assertNotIn("onCustomControlPanelDelete(callback)", ui_controller)
        self.assertNotIn("startCustomControlPanelLongPress(", ui_controller)
        self.assertNotIn("beginCustomControlPanelDrag(", ui_controller)
        self.assertNotIn("finishCustomControlPanelDrag(", ui_controller)
        self.assertNotIn("toggleControlPanelCustomizeMenu(forceOpen = null)", ui_controller)
        self.assertIn("this.renderControlPanelTasks();", app_logic)
        self.assertNotIn("customControlPanelButtons", app_logic)
        self.assertIn("this.ui.setSettingsHomePage(this.settingsHomePage);", app_logic)
        self.assertIn("this.ui.setSettingsPage(this.settingsHomePage);", app_logic)
        self.assertNotIn("loadCustomControlPanelButtons()", app_logic)
        self.assertNotIn("saveCustomControlPanelButtons", app_logic)
        self.assertNotIn("normalizeCustomControlPanelButtons", app_logic)
        self.assertNotIn("onControlPanelCustomButtonCreate", app_logic)
        self.assertNotIn("onCustomControlPanelAction", app_logic)
        self.assertNotIn("onCustomControlPanelDelete", app_logic)
        self.assertNotIn("handleCustomControlPanelButton", app_logic)
        self.assertIn("loadSettingsHomePagePreference()", app_logic)
        self.assertIn("saveSettingsHomePagePreference(pageId)", app_logic)
        self.assertIn("SETTINGS_HOME_PAGE_KEY", storage)
        self.assertIn("loadSettingsHomePagePreference()", storage)
        self.assertIn("saveSettingsHomePagePreference(pageId)", storage)
        self.assertNotIn("CONTROL_PANEL_VISIBLE_TASKS_KEY", storage)
        self.assertNotIn("CUSTOM_CONTROL_PANEL_BUTTONS_KEY", storage)
        self.assertNotIn("loadControlPanelVisibleTasks()", storage)
        self.assertNotIn("saveControlPanelVisibleTasks(taskIds)", storage)
        self.assertNotIn("loadCustomControlPanelButtons()", storage)
        self.assertNotIn("saveCustomControlPanelButtons(buttons)", storage)
        self.assertNotIn("normalizeControlPanelVisibleTaskIds(taskIds)", control_panel_catalog)
        self.assertNotIn("normalizeCustomControlPanelButtons(buttons)", control_panel_catalog)
        self.assertNotIn('serviceType: "std_srvs/Trigger"', control_panel_catalog)
        self.assertNotIn("callTriggerService(serviceName, serviceType = TRIGGER_SERVICE_TYPE)", ros_connection)
        self.assertIn(".settings-home-page-button {", app_css)
        self.assertIn(".settings-home-page-button.is-active {", app_css)
        self.assertNotIn(".control-panel-customize-menu {", app_css)
        self.assertNotIn(".control-panel-empty-state {", app_css)
        self.assertNotIn(".control-panel-custom-form {", app_css)
        self.assertNotIn(".control-panel-trash-bin {", app_css)
        self.assertNotIn(".control-action-btn-custom {", app_css)

    def test_move_to_position_uses_persisted_recognition_pose(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        control_panel_catalog = CONTROL_PANEL_CATALOG.read_text(encoding="utf-8")
        storage = (
            FRONTEND_SRC_DIR / "utils" / "storage.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")

        self.assertIn("const DIRECT_CABIN_MOVE_TARGET = Object.freeze({", app_logic)
        self.assertIn("x: -260,", app_logic)
        self.assertIn("y: 1700,", app_logic)
        self.assertIn("z: 3197,", app_logic)
        self.assertIn('export const RECOGNITION_POSE_KEY = "tie_robot_frontend_recognition_pose";', storage)
        self.assertIn("loadRecognitionPose(defaultPose = null)", storage)
        self.assertIn("saveRecognitionPose(value)", storage)
        self.assertIn("this.recognitionPose = loadRecognitionPose(DIRECT_CABIN_MOVE_TARGET);", app_logic)
        self.assertIn('if (taskAction === "setRecognitionPose")', app_logic)
        self.assertIn("handleSetRecognitionPose()", app_logic)
        self.assertIn("const pose = this.sceneView.getCurrentCabinPositionMm();", app_logic)
        self.assertIn("saveRecognitionPose(pose);", app_logic)
        self.assertIn("const payload = { ...this.recognitionPose, speed: this.getGlobalCabinMoveSpeed() };", app_logic)
        self.assertIn("准备直接移动到位姿", app_logic)
        self.assertNotIn("准备直接移动到索驱位置", app_logic)
        self.assertIn('{ id: "setRecognitionPose", label: "设为\\n识别位姿", tone: "amber" }', control_panel_catalog)
        self.assertIn('{ id: "moveToPosition", label: "移动到\\n位姿", tone: "blue" }', control_panel_catalog)
        self.assertIn("setTaskButtonEnabled(taskAction, enabled)", ui_controller)

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
        self.assertIn("getCenteredPanelRect(width, height, verticalOffset = 0)", panel_manager)
        self.assertIn('if (panelId === "controlPanel") {', panel_manager)
        self.assertIn('if (panelId === "imagePanel") {', panel_manager)
        self.assertIn('if (panelId === "settingsPanel") {', panel_manager)
        self.assertIn("isUsablePanelRect(rect)", panel_manager)
        self.assertIn("if (!this.isUsablePanelRect(rect)) {", panel_manager)
        self.assertIn("panel.style.width = `${target.width}px`;", panel_manager)
        self.assertIn("panel.style.height = `${target.height}px`;", panel_manager)
        self.assertNotIn('target.left + Math.max(0, (target.width - width) / 2)', panel_manager)
        self.assertNotIn('target.left + Math.max(0, (target.width - panelWidth) / 2)', panel_manager)
        self.assertIn('if (panelId === "logPanel") {', panel_manager)
        self.assertIn('const width = Math.min(700, Math.max(520, window.innerWidth - 720));', panel_manager)
        self.assertIn('const height = Math.min(320, Math.max(240, window.innerHeight - 860));', panel_manager)
        self.assertIn("return this.getCenteredPanelRect(width, height);", panel_manager)
        self.assertIn('if (panelId === "terminalPanel") {', panel_manager)
        self.assertIn('const width = Math.min(920, Math.max(620, window.innerWidth - 320));', panel_manager)
        self.assertIn("return this.getCenteredPanelRect(width, height);", panel_manager)
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
        self.assertIn('if (nextVisible) {', app_logic)
        self.assertIn('if (!this.panelManager.hasPanelLayout(panelId)) {', app_logic)
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
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn('id: "ros"', status_catalog)
        self.assertIn('id: "chassis"', status_catalog)
        self.assertIn('id: "moduan"', status_catalog)
        self.assertIn('id: "visual"', status_catalog)
        self.assertIn('diagnosticHardwareId: "tie_robot/chassis_driver"', status_catalog)
        self.assertIn('diagnosticHardwareId: "tie_robot/moduan_driver"', status_catalog)
        self.assertIn('diagnosticHardwareId: "tie_robot/visual_algorithm"', status_catalog)
        self.assertIn("name: TOPICS.process.diagnostics", status_controller)
        self.assertIn("messageType: MESSAGE_TYPES.diagnosticArray", status_controller)
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
        self.assertIn('chassis: level === "success" ? "stopCabinSubsystem" : "restartCabinSubsystem"', ui_controller)
        self.assertIn('moduan: level === "success" ? "stopModuanSubsystem" : "startModuanSubsystem"', ui_controller)
        self.assertIn('visual: level === "success" ? "stopVisualSubsystem" : "startVisualSubsystem"', ui_controller)
        self.assertIn('ros: level === "success" ? "重启ROS" : "启动ROS"', ui_controller)
        self.assertIn('chassis: level === "success" ? "关闭" : "重启"', ui_controller)
        self.assertIn('visual: level === "success" ? "关闭" : "启动"', ui_controller)
        self.assertIn('chip.dataset.statusAction = nextAction;', ui_controller)
        self.assertIn('button.dataset.statusAction === actionId', ui_controller)
        self.assertIn('button.classList.contains("is-pending")', ui_controller)
        self.assertIn('actionLabel.textContent = pendingLabel;', ui_controller)
        self.assertIn('aria-busy', ui_controller)
        self.assertIn('.system-status-item.is-pending .system-status-label', app_css)
        self.assertIn('.system-status-item.is-pending .system-status-action-label', app_css)
        self.assertIn('@keyframes status-spin', app_css)
        self.assertIn('button.setAttribute("aria-pressed", active ? "true" : "false");', ui_controller)

    def test_workspace_overlay_chain_uses_dedicated_pr_fprg_overlay(self):
        ros_connection_controller = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")

        self.assertIn('this.buildTopicFromRegistry("algorithm.manualWorkspaceS2ResultRaw")', ros_connection_controller)
        self.assertIn('this.buildTopicFromRegistry("algorithm.manualWorkspaceS2Points")', ros_connection_controller)
        self.assertIn("this.callbacks.onWorkspaceS2Overlay?.(message)", ros_connection_controller)
        self.assertIn("this.callbacks.onVisualRecognitionPoints?.(message)", ros_connection_controller)
        self.assertIn("当前图像图层会等待视觉识别点位覆盖层", task_action_controller)
        self.assertIn("handleVisualRecognitionPointsMessage(message)", app_logic)
        self.assertIn("showVisualRecognitionOverlayMessage(message)", app_logic)
        self.assertIn("if (this.surfaceDpOverlayRequested) {", app_logic)
        self.assertIn("this.showVisualRecognitionOverlayMessage(message);", app_logic)
        self.assertIn("if (this.surfaceDpOverlayActive) {", app_logic)
        self.assertIn("this.surfaceDpOverlayRequested = false;", app_logic)
        self.assertIn("if (!this.surfaceDpOverlayRequested) {", app_logic)
        self.assertIn("this.workspaceView.setS2OverlayMessage(message);", app_logic)
        self.assertIn("this.workspaceView.setVisualRecognitionOverlaySourceSize", app_logic)
        self.assertIn("后端视觉识别结果图已叠加在当前图像图层", app_logic)

        execution_overlay_body = app_logic[
            app_logic.index("onExecutionOverlay: (message) => {"):
            app_logic.index("onWorkspaceS2Overlay: (message) => {")
        ]
        self.assertIn("if (this.surfaceDpOverlayRequested) {\n          return;\n        }", execution_overlay_body)
        self.assertNotIn("this.showVisualRecognitionOverlayMessage(message);", execution_overlay_body)

    def test_visual_recognition_uses_canonical_service_and_result_image_topic(self):
        topic_registry = TOPIC_REGISTRY.read_text(encoding="utf-8")
        ros_connection_controller = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")

        self.assertIn('recognizeOnce: "/perception/lashing/recognize_once"', topic_registry)
        self.assertIn('manualWorkspaceS2ResultRaw: "/perception/lashing/result_image"', topic_registry)
        self.assertIn('manualWorkspaceS2Points: "/perception/lashing/points_camera"', topic_registry)
        self.assertIn('coordinatePoint: "/perception/lashing/points_camera"', topic_registry)
        self.assertIn("lashingRecognizeOnceService: new ROSLIB.Service({", ros_connection_controller)
        self.assertIn("name: SERVICES.algorithm.recognizeOnce", ros_connection_controller)
        self.assertIn("serviceType: SERVICE_TYPES.trigger", ros_connection_controller)
        self.assertIn("callLashingRecognizeOnceService()", ros_connection_controller)
        self.assertIn("this.resources.lashingRecognizeOnceService.callService(", ros_connection_controller)
        self.assertIn("const result = await this.rosConnection.callLashingRecognizeOnceService();", task_action_controller)
        self.assertIn("Boolean(resources?.lashingRecognizeOnceService)", app_logic)
        self.assertNotIn("this.resources.runWorkspaceS2Publisher.advertise();", ros_connection_controller)
        self.assertNotIn("runWorkspaceS2Publisher.publish", task_action_controller)
        self.assertNotIn("resources?.runWorkspaceS2Publisher", task_action_controller)

    def test_image_panel_supports_project_image_topics_and_overlay_switching(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        workspace_canvas_view = (
            FRONTEND_SRC_DIR / "views" / "WorkspaceCanvasView.js"
        ).read_text(encoding="utf-8")
        image_topic_catalog = IMAGE_TOPIC_CATALOG.read_text(encoding="utf-8")
        storage_utils = STORAGE_UTILS.read_text(encoding="utf-8")
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
        self.assertIn("TOPICS.algorithm.resultImageRaw", image_topic_catalog)
        self.assertIn('const defaults = { mode: "raw", gamma: 1.0, overlayOpacity: 0.88 };', storage_utils)
        self.assertIn('manualWorkspaceS2Points: "/perception/lashing/points_camera"', TOPIC_REGISTRY.read_text(encoding="utf-8"))
        self.assertIn("setSavedWorkspaceGuideVisible(enabled)", workspace_canvas_view)
        self.assertIn("setS2OverlayMessage(message)", workspace_canvas_view)
        self.assertIn("setVisualRecognitionPointsMessage(message", workspace_canvas_view)
        self.assertIn("drawVisualRecognitionPoints()", workspace_canvas_view)
        self.assertIn("const imageMessage = this.lastImageMessage;", workspace_canvas_view)
        self.assertIn("if (this.savedWorkspaceGuideVisible && this.savedWorkspacePoints.length >= 2)", workspace_canvas_view)
        self.assertIn("setOverlayEnabled(enabled)", workspace_canvas_view)
        self.assertIn("if (this.overlayEnabled && this.lastExecutionResultMessage)", workspace_canvas_view)
        self.assertNotIn("pinnedResultImageMessage", workspace_canvas_view)
        self.assertIn('encoding.includes("mono16") || encoding.includes("16uc1")', image_utils)
        self.assertIn('encoding.includes("32fc1")', image_utils)
        self.assertIn('encoding.includes("32fc3")', image_utils)

    def test_ir_image_draws_live_tcp_workspace_boundary_overlay(self):
        topic_registry = TOPIC_REGISTRY.read_text(encoding="utf-8")
        ros_connection_controller = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")
        workspace_canvas_view = (
            FRONTEND_SRC_DIR / "views" / "WorkspaceCanvasView.js"
        ).read_text(encoding="utf-8")
        overlay_utils = (
            FRONTEND_SRC_DIR / "utils" / "tcpWorkspaceOverlay.js"
        ).read_text(encoding="utf-8")

        self.assertIn('cameraInfo: "sensor_msgs/CameraInfo"', topic_registry)
        self.assertIn('irCameraInfo: "/Scepter/ir/camera_info"', topic_registry)
        self.assertIn('key: "camera.irCameraInfo"', topic_registry)
        self.assertIn('this.buildTopicFromRegistry("camera.irCameraInfo")', ros_connection_controller)
        self.assertIn("this.callbacks.onIrCameraInfo?.(message)", ros_connection_controller)
        self.assertIn("this.irCameraInfo = null;", app_logic)
        self.assertIn("onIrCameraInfo: (message) => {", app_logic)
        self.assertIn("syncTcpWorkspaceBoundaryOverlay()", app_logic)
        self.assertIn("this.workspaceView.setTcpWorkspaceBoundary(boundary);", app_logic)
        self.assertIn("projectTcpWorkspaceBoundaryToImage(cameraInfo)", scene_view)
        self.assertIn("GRIPPER_FRAME", scene_view)
        self.assertNotIn("applyDistortion", scene_view)
        self.assertIn("setTcpWorkspaceBoundary(boundary)", workspace_canvas_view)
        self.assertIn("drawTcpWorkspaceBoundary()", workspace_canvas_view)
        self.assertIn("TCP_WORKSPACE_BOUNDARY_MM", overlay_utils)
        self.assertIn("max: 380", overlay_utils)
        self.assertIn("max: 330", overlay_utils)
        self.assertIn("projectCameraPointMetersToImagePixel", overlay_utils)

    def test_ir_camera_info_distortion_is_not_used_by_tf_or_visual_layers(self):
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")
        overlay_utils = (
            FRONTEND_SRC_DIR / "utils" / "tcpWorkspaceOverlay.js"
        ).read_text(encoding="utf-8")
        pointai_image_buffers = (
            TIE_ROBOT_PERCEPTION_DIR
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "image_buffers.py"
        ).read_text(encoding="utf-8")
        robot_tf_broadcaster = (
            TIE_ROBOT_PERCEPTION_DIR / "scripts" / "robot_tf_broadcaster.py"
        ).read_text(encoding="utf-8")
        gripper_tf_broadcaster = (
            TIE_ROBOT_PERCEPTION_DIR / "scripts" / "gripper_tf_broadcaster.py"
        ).read_text(encoding="utf-8")

        self.assertNotIn("cameraInfo?.D", overlay_utils)
        self.assertNotIn(".D", overlay_utils)
        self.assertNotIn("applyPlumbBobDistortion", overlay_utils)
        self.assertNotIn("applyDistortion", overlay_utils)
        self.assertNotIn("undistort", overlay_utils)
        self.assertNotIn("applyDistortion", scene_view)
        self.assertNotIn("msg.D", pointai_image_buffers)
        self.assertNotIn("dist_coeffs = np.array", pointai_image_buffers)
        self.assertNotIn("CameraInfo", robot_tf_broadcaster)
        self.assertNotIn("CameraInfo", gripper_tf_broadcaster)

    def test_scene_view_renders_area_planning_from_bind_path_json(self):
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")

        self.assertIn('"/api/planning/bind-path"', server_script)
        self.assertIn("fetch(\"/api/planning/bind-path\", { cache: \"no-store\" })", app_logic)
        self.assertIn("schedulePlanningAreaRefresh()", app_logic)
        self.assertIn("refreshPlanningAreaOverlay()", app_logic)
        self.assertIn("this.sceneView.setPlanningAreaPayload(payload.bind_path || null);", app_logic)
        self.assertIn("this.sceneView.setPlanningAreaPayload(null);", app_logic)
        self.assertIn("setPlanningAreaPayload(payload)", scene_view)
        self.assertIn("this.planningAreaOutlines = new THREE.LineSegments(", scene_view)
        self.assertIn("this.planningAreaPath = new THREE.LineSegments(", scene_view)
        self.assertIn("this.planningAreaCenters = buildPointsObject(", scene_view)
        self.assertIn("buildAreaOutlineSegmentPositions(areas)", scene_view)
        self.assertIn("function buildAreaCenterPathPositions(areas, pathOrigin = null)", scene_view)
        self.assertIn("buildAreaCenterPathPositions(areas, payload?.path_origin || null)", scene_view)
        self.assertIn("buildAreaCenterPositions(areas)", scene_view)
        self.assertNotIn("planningPathPositions.length ? planningPathPositions : buildSequentialLineSegmentPositions(positions)", scene_view)

    def test_scene_view_projects_tie_points_from_camera_frame_using_tf(self):
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")

        self.assertIn("convertScepterPointCloudPointToMapPoint(point)", scene_view)
        self.assertIn("const mapPoint = this.convertScepterPointCloudPointToMapPoint(vector);", scene_view)
        self.assertIn("worldPositions.push(mapPoint.x, mapPoint.y, mapPoint.z);", scene_view)
        self.assertNotIn("positions.push(vector.x, vector.y, vector.z);", scene_view)

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
        self.assertIn("const nextTopicName = getPointCloudTopicName(nextState.source);", ros_connection_controller)
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
        self.assertIn("const baseLinkTransform = this.getWorldTransform(BASE_LINK_FRAME);", scene_view)
        self.assertIn("this.applyGroupTfTransform(this.baseLinkFrame, baseLinkTransform, BASE_LINK_FRAME);", scene_view)
        self.assertIn("const robotTransform = baseLinkTransform || scepterTransform;", scene_view)
        self.assertIn("if (robotTransform)", scene_view)
        self.assertIn("this.applyWorldAlignedDisplayPose(this.robotGroup, robotTransform.position);", scene_view)
        self.assertIn("robotMesh.position.z = ROBOT_BODY_SIZE_METERS / 2.0;", scene_view)
        self.assertNotIn("ROBOT_BODY_CLEARANCE_ABOVE_TCP_METERS", scene_view)
        self.assertIn("if (gripperTransform)", scene_view)
        self.assertIn("this.applyCustomDisplayPose(this.tcpToolGroup, gripperTransform.position);", scene_view)
        self.assertIn("tcpToolMesh.position.z = -TCP_TOOL_SIZE_METERS.z / 2.0;", scene_view)
        self.assertNotIn("this.robotGroup.position.z += 0.58;", scene_view)
        self.assertIn("applyCustomDisplayPose(group, position)", scene_view)
        self.assertIn("applyWorldAlignedDisplayPose(group, position)", scene_view)
        self.assertIn("applyTfFramePose(group, transform)", scene_view)
        self.assertIn("group.quaternion.copy(transform.quaternion);", scene_view)
        self.assertIn("group.quaternion.identity();", scene_view)
        self.assertIn("group.scale.set(1, 1, -1);", scene_view)
        self.assertNotIn("this.robotGroup.quaternion.copy(scepterTransform.quaternion);", scene_view)
        self.assertNotIn("this.tcpToolGroup.quaternion.copy(gripperTransform.quaternion);", scene_view)
        self.assertIn("z: directRecord.position.z * 1000.0,", scene_view)
        self.assertIn("z: relative.z * 1000.0,", scene_view)
        self.assertIn("const pointInMap = localPoint.clone().applyQuaternion(scepterTransform.quaternion);", scene_view)
        self.assertIn("return scepterTransform.position.clone().add(pointInMap);", scene_view)
        self.assertNotIn("pointAiStyleLocalPoint", scene_view)
        self.assertNotIn("z: -directRecord.position.z * 1000.0,", scene_view)
        self.assertNotIn("z: -relative.z * 1000.0,", scene_view)
        self.assertNotIn("scepterTransform.position.z - localPoint.z - gripperOffsetZ,", scene_view)
        self.assertNotIn("scepterTransform.position.z - localPoint.z + gripperOffsetZ,", scene_view)
        self.assertNotIn("Math.abs(this.getGripperOffsetRelativeToScepter())", scene_view)
        self.assertIn("materials[0].color.setHex(0x7d8da4);", scene_view)
        self.assertIn("materials[1].color.setHex(0xa6b2c2);", scene_view)
        self.assertIn("materials[0].color.setHex(0xffffff);", scene_view)
        self.assertIn("materials[1].color.setHex(0xe8eef7);", scene_view)

    def test_base_link_splits_machine_pose_from_downward_camera_frame(self):
        suoqu_node = (
            WORKSPACE_SRC / "tie_robot_process" / "src" / "suoquNode.cpp"
        ).read_text(encoding="utf-8")
        robot_tf_broadcaster = (
            WORKSPACE_SRC / "tie_robot_perception" / "scripts" / "robot_tf_broadcaster.py"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")

        self.assertIn('const BASE_LINK_FRAME = "base_link";', scene_view)
        self.assertIn("this.baseLinkFrame = new THREE.Group();", scene_view)
        self.assertIn("this.baseLinkFrame.add(new THREE.AxesHelper(0.32));", scene_view)
        self.assertIn("this.baseLinkFrame.visible = Boolean(baseLinkTransform) && this.isTfAxisFrameVisible(BASE_LINK_FRAME);", scene_view)
        self.assertIn("robotMesh.position.z = ROBOT_BODY_SIZE_METERS / 2.0;", scene_view)
        self.assertIn("this.applyGroupTfTransform(this.baseLinkFrame, baseLinkTransform, BASE_LINK_FRAME);", scene_view)
        self.assertIn("this.applyGroupTfTransform(this.scepterFrame, scepterTransform, SCEPTER_FRAME);", scene_view)
        self.assertIn("this.applyGroupTfTransform(this.gripperFrame, gripperTransform, GRIPPER_FRAME);", scene_view)
        self.assertIn("this.applyTfFramePose(group, transform);", scene_view)
        self.assertIn("const poseTransform = this.getWorldTransform(BASE_LINK_FRAME) || this.getWorldTransform(SCEPTER_FRAME);", scene_view)

        self.assertIn("base_transform = TransformStamped()", robot_tf_broadcaster)
        self.assertIn("base_transform.header.frame_id = map_frame", robot_tf_broadcaster)
        self.assertIn("base_transform.child_frame_id = base_link_frame", robot_tf_broadcaster)
        self.assertIn("base_transform.transform.rotation.w = 1.0", robot_tf_broadcaster)
        self.assertIn("cabin_to_map_sign", robot_tf_broadcaster)
        self.assertIn("apply_cabin_to_map_sign_mm(latest_pose_mm, config)", robot_tf_broadcaster)
        self.assertNotIn('latest_pose_mm = {"x": 0.0, "y": 0.0, "z": 0.0}', robot_tf_broadcaster)
        self.assertNotIn("publishing zero pose TF", robot_tf_broadcaster)
        self.assertIn('latest_pose_mm = {"value": None}', robot_tf_broadcaster)
        self.assertIn("pose_snapshot is None", robot_tf_broadcaster)
        self.assertIn("continue", robot_tf_broadcaster)
        self.assertNotIn("camera_origin_z_offset_mm", suoqu_node)
        self.assertNotIn("g_camera_origin_z_offset_mm", suoqu_node)
        self.assertNotIn("g_base_link_z_offset_mm", suoqu_node)
        self.assertNotIn("base_link_z_offset_mm", suoqu_node)
        self.assertNotIn("base_link_transform.transform.translation.z = (static_cast<double>(cabin_state.Z) +", suoqu_node)
        self.assertNotIn("base_link_transform.transform.translation.z = static_cast<double>(cabin_state.Z) / 1000.0 + camera_z_offset_m;", suoqu_node)
        self.assertIn("scepter_transform = TransformStamped()", robot_tf_broadcaster)
        self.assertIn("scepter_transform.header.frame_id = base_link_frame", robot_tf_broadcaster)
        self.assertIn("scepter_transform.child_frame_id = scepter_frame", robot_tf_broadcaster)
        self.assertIn('apply_cabin_to_map_sign_mm(config["base_to_camera_mm"], config)', robot_tf_broadcaster)
        self.assertIn('_set_quaternion_from_rpy(scepter_transform, config["base_to_camera_rpy"])', robot_tf_broadcaster)
        self.assertIn("broadcaster.sendTransform([base_transform, scepter_transform])", robot_tf_broadcaster)
        self.assertNotIn("cabin_tf_broadcaster->sendTransform(transforms);", suoqu_node)
        self.assertNotIn("double camera_z_offset_m = 0.0;", suoqu_node)
        self.assertNotIn("camera_z_offset_m = -gripper_from_scepter.getOrigin().z();", suoqu_node)
        self.assertIn("const position = scepterTransform.position.clone();", scene_view)
        self.assertIn("CAMERA_VIEW_FORWARD_AXIS.clone().applyQuaternion(scepterTransform.quaternion)", scene_view)
        self.assertIn("CAMERA_VIEW_UP_AXIS.clone().applyQuaternion(scepterTransform.quaternion)", scene_view)
        self.assertIn("TOP_VIEW_FORWARD_AXIS", scene_view)

    def test_tf_axes_can_be_toggled_per_frame(self):
        topic_layer_catalog = (
            FRONTEND_SRC_DIR / "config" / "topicLayerCatalog.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")

        self.assertIn("export const TF_AXIS_FRAMES = [", topic_layer_catalog)
        self.assertIn('{ id: "map", label: "索驱世界 map" }', topic_layer_catalog)
        self.assertIn('{ id: "base_link", label: "机器 base_link" }', topic_layer_catalog)
        self.assertIn('{ id: "Scepter_depth_frame", label: "相机 Scepter_depth_frame" }', topic_layer_catalog)
        self.assertIn('{ id: "gripper_frame", label: "TCP gripper_frame" }', topic_layer_catalog)
        self.assertIn("tfAxisFrameVisibility: DEFAULT_TF_AXIS_FRAME_VISIBILITY,", topic_layer_catalog)

        self.assertIn('data-tf-axis-frame="${frame.id}"', ui_controller)
        self.assertIn("this.refs.tfAxisFrameToggles =", ui_controller)
        self.assertIn("tfAxisFrameVisibility: this.getTfAxisFrameVisibility(),", ui_controller)
        self.assertIn("setTfAxisFrameVisibility(state.tfAxisFrameVisibility)", ui_controller)
        self.assertIn("坐标轴明细", ui_controller)

        self.assertIn("getTfAxisFrameVisibility(state)", scene_view)
        self.assertIn("isTfAxisFrameVisible(frameId)", scene_view)
        self.assertIn("this.mapAxes.visible = this.isTfAxisFrameVisible(MAP_FRAME);", scene_view)
        self.assertIn("this.baseLinkFrame.visible = Boolean(baseLinkTransform) && this.isTfAxisFrameVisible(BASE_LINK_FRAME);", scene_view)
        self.assertIn("this.scepterFrame.visible = Boolean(scepterTransform) && this.isTfAxisFrameVisible(SCEPTER_FRAME);", scene_view)
        self.assertIn("this.gripperFrame.visible = Boolean(gripperTransform) && this.isTfAxisFrameVisible(GRIPPER_FRAME);", scene_view)
        self.assertIn("this.applyGroupTfTransform(this.scepterFrame, scepterTransform, SCEPTER_FRAME);", scene_view)
        self.assertIn("this.applyGroupTfTransform(this.gripperFrame, gripperTransform, GRIPPER_FRAME);", scene_view)
        self.assertIn("this.applyTfFramePose(group, transform);", scene_view)
        self.assertIn("group.visible = this.isTfAxisFrameVisible(frameId);", scene_view)
        self.assertNotIn("if (this.layerState.showAxes !== false) {\n      group.visible = true;", scene_view)

    def test_help_station_embeds_local_gitnexus_webui_bridge_graph(self):
        help_index = (WORKSPACE_SRC / "tie_robot_web" / "help" / "index.md").read_text(encoding="utf-8")
        help_config = (
            WORKSPACE_SRC / "tie_robot_web" / "help" / ".vitepress" / "config.mjs"
        ).read_text(encoding="utf-8")
        graph_page = (
            WORKSPACE_SRC / "tie_robot_web" / "help" / "reference" / "gitnexus-graph.md"
        ).read_text(encoding="utf-8")
        launcher = (
            WORKSPACE_SRC / "tie_robot_web" / "tools" / "run_gitnexus_local_webui.py"
        ).read_text(encoding="utf-8")

        self.assertIn("[查看本地 GitNexus WebUI 图谱](./reference/gitnexus-graph)", help_index)
        self.assertIn('{ text: "GitNexus 图谱", link: "/reference/gitnexus-graph" }', help_config)
        self.assertIn('text: "GitNexus 图谱"', help_config)
        self.assertIn('{ text: "本地 WebUI", link: "/reference/gitnexus-graph" }', help_config)
        self.assertIn("window.location.hostname", graph_page)
        self.assertIn("gitnexusWebuiUrl", graph_page)
        self.assertIn("bridgeReposUrl", graph_page)
        self.assertIn(":5173", graph_page)
        self.assertIn(":4747", graph_page)
        self.assertIn("new URLSearchParams", graph_page)
        self.assertIn(".vp-doc .gitnexus-open-link", graph_page)
        self.assertIn("color: #071018 !important", graph_page)
        self.assertNotIn("server=http://127.0.0.1:4747", graph_page)
        self.assertNotIn("gitnexus.vercel.app", graph_page)
        self.assertIn("<iframe", graph_page)
        self.assertIn(":src=\"gitnexusWebuiUrl\"", graph_page)
        self.assertIn("src/tie_robot_web/tools/run_gitnexus_local_webui.py", graph_page)
        self.assertIn("gitnexus serve --host 0.0.0.0 --port 4747", graph_page)
        self.assertIn("npm run dev -- --host 0.0.0.0 --port 5173", graph_page)
        self.assertIn("3713 files", graph_page)
        self.assertIn("29351 symbols", graph_page)
        self.assertIn("46164 edges", graph_page)
        self.assertIn("https://github.com/abhigyanpatwari/GitNexus.git", launcher)
        self.assertIn('GITNEXUS_BRIDGE_HOST", "0.0.0.0"', launcher)
        self.assertIn('GITNEXUS_WEBUI_HOST", "0.0.0.0"', launcher)
        self.assertIn("GITNEXUS_PUBLIC_HOST", launcher)
        self.assertIn("GITNEXUS_INTERNAL_BRIDGE_PORT", launcher)
        self.assertIn("run_cors_proxy", launcher)
        self.assertIn("Access-Control-Allow-Origin", launcher)
        self.assertIn("/api/heartbeat", launcher)
        self.assertIn("text/event-stream", launcher)
        self.assertIn("patch_webui_backend_url_race", launcher)
        self.assertIn("new URLSearchParams(window.location.search).get('server')", launcher)
        self.assertIn('"--cors-proxy"', launcher)
        self.assertIn('["gitnexus", "serve", "--host", INTERNAL_BRIDGE_HOST, "--port", str(internal_bridge_port)]', launcher)
        self.assertIn("[sys.executable, __file__, \"--cors-proxy\", BRIDGE_HOST, str(BRIDGE_PORT), INTERNAL_BRIDGE_HOST, str(internal_bridge_port)]", launcher)
        self.assertIn("npm\", \"run\", \"dev\"", launcher)
        self.assertIn("GITNEXUS_WEBUI_DIR", launcher)

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
        start_algorithm_script = (WORKSPACE_SRC.parent / "start_algorithm_stack.sh").read_text(encoding="utf-8")
        restart_algorithm_script = (WORKSPACE_SRC.parent / "restart_algorithm_stack.sh").read_text(encoding="utf-8")
        stop_algorithm_script = (WORKSPACE_SRC.parent / "stop_algorithm_stack.sh").read_text(encoding="utf-8")

        self.assertIn('ROS_BACKEND_SERVICE = "tie-robot-backend.service"', server_script)
        self.assertIn('ROSBRIDGE_SERVICE = "tie-robot-rosbridge.service"', server_script)
        self.assertIn('"/api/system/start_ros_stack": "start"', server_script)
        self.assertIn('"/api/system/stop_ros_stack": "stop"', server_script)
        self.assertIn('"/api/system/restart_ros_stack": FULL_ROS_STACK_RESTART_ACTION', server_script)
        self.assertIn('"/api/system/ros_stack_status"', server_script)
        self.assertIn('command = ["sudo", "-n", SYSTEMCTL_BIN, action, *service_names]', server_script)
        self.assertIn("FULL_ROS_STACK_STOP_ORDER", server_script)
        self.assertIn("FULL_ROS_STACK_START_ORDER", server_script)
        self.assertNotIn('"/api/system/start_ros_stack": WORKSPACE_ROOT / "start_.sh"', server_script)
        self.assertNotIn('"/api/system/restart_ros_stack": WORKSPACE_ROOT / "restart_ros_stack.sh"', server_script)
        self.assertIn('"/api/system/start_driver_stack": ("start", "all")', server_script)
        self.assertIn('"/api/system/stop_driver_stack": ("stop", "all")', server_script)
        self.assertIn('"/api/system/restart_driver_stack": ("restart", "all")', server_script)
        self.assertIn('"cabin": "tie-robot-driver-suoqu.service"', server_script)
        self.assertIn('"moduan": "tie-robot-driver-moduan.service"', server_script)
        self.assertIn('"camera": "tie-robot-driver-camera.service"', server_script)
        self.assertNotIn('"/api/system/start_driver_stack": WORKSPACE_ROOT / "start_driver_stack.sh"', server_script)
        self.assertNotIn('"/api/system/restart_driver_stack": WORKSPACE_ROOT / "restart.sh"', server_script)
        self.assertIn('"/api/system/start_algorithm_stack": WORKSPACE_ROOT / "start_algorithm_stack.sh"', server_script)
        self.assertIn('"/api/system/stop_algorithm_stack": WORKSPACE_ROOT / "stop_algorithm_stack.sh"', server_script)
        self.assertIn('"/api/system/restart_algorithm_stack": WORKSPACE_ROOT / "restart_algorithm_stack.sh"', server_script)
        self.assertIn('id: "startRosStack"', system_control_catalog)
        self.assertIn('id: "stopRosStack"', system_control_catalog)
        self.assertIn('id: "stopDriverStack"', system_control_catalog)
        self.assertIn('id: "stopAlgorithmStack"', system_control_catalog)
        self.assertIn('id: "startCabinSubsystem"', system_control_catalog)
        self.assertIn('steps: ["startCabinDriver"]', system_control_catalog)
        self.assertIn('id: "restartCabinSubsystem"', system_control_catalog)
        self.assertIn('steps: ["restartCabinDriver"]', system_control_catalog)
        self.assertIn('id: "stopCabinSubsystem"', system_control_catalog)
        self.assertIn('steps: ["stopCabinDriver"]', system_control_catalog)
        self.assertIn('id: "startModuanSubsystem"', system_control_catalog)
        self.assertIn('steps: ["startModuanDriver"]', system_control_catalog)
        self.assertIn('id: "stopModuanSubsystem"', system_control_catalog)
        self.assertIn('steps: ["stopModuanDriver"]', system_control_catalog)
        self.assertIn('id: "startVisualSubsystem"', system_control_catalog)
        self.assertIn('steps: ["startCameraDriver", "startAlgorithmStack"]', system_control_catalog)
        self.assertIn('id: "stopVisualSubsystem"', system_control_catalog)
        self.assertIn('steps: ["stopAlgorithmStack", "stopCameraDriver"]', system_control_catalog)
        self.assertIn('id: "restartAlgorithmStack"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/start_ros_stack"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/stop_ros_stack"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/stop_algorithm_stack"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/start_camera_driver"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/restart_algorithm_stack"', system_control_catalog)
        self.assertIn("executeActionSequence(action)", system_control_controller)
        self.assertIn("for (const stepId of action.steps)", system_control_controller)
        self.assertIn("handleViaHttp(action)", system_control_controller)
        self.assertIn("fetch(action.httpEndpoint", system_control_controller)
        self.assertNotIn("/stable_point_tf_broadcaster", start_algorithm_script)
        self.assertNotIn("/stable_point_tf_broadcaster", restart_algorithm_script)
        self.assertNotIn("rosnode ping -c 1", start_algorithm_script)
        self.assertNotIn("rosnode ping -c 1", restart_algorithm_script)
        self.assertIn("rosnode list", start_algorithm_script)
        self.assertIn("/bind_map_builder", start_algorithm_script)
        self.assertIn("/bind_map_builder", restart_algorithm_script)
        self.assertIn("/bind_map_builder", stop_algorithm_script)

    def test_demo_mode_uses_old_show_full_without_translation_layer(self):
        demo_service = TIE_ROBOT_BRINGUP_DEMO_MODE_SERVICE.read_text(encoding="utf-8")
        demo_rosbridge_service = TIE_ROBOT_BRINGUP_DEMO_ROSBRIDGE_SERVICE.read_text(encoding="utf-8")
        demo_rosbridge_launch = TIE_ROBOT_BRINGUP_DEMO_ROSBRIDGE_LAUNCH.read_text(encoding="utf-8")
        demo_installer = TIE_ROBOT_BRINGUP_DEMO_MODE_INSTALLER.read_text(encoding="utf-8")
        legacy_frontend_service = (
            TIE_ROBOT_BRINGUP_DIR / "systemd" / "tie-robot-show-legacy-frontend.service.in"
        ).read_text(encoding="utf-8")
        frontend_installer = TIE_ROBOT_BRINGUP_FRONTEND_AUTOSTART_INSTALLER.read_text(
            encoding="utf-8"
        )
        backend_sudoers = TIE_ROBOT_BRINGUP_BACKEND_SUDOERS.read_text(encoding="utf-8")
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        system_control_catalog = SYSTEM_CONTROL_CATALOG.read_text(encoding="utf-8")
        system_control_controller = (
            FRONTEND_SRC_DIR / "controllers" / "SystemControlController.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")

        self.assertIn("Description=Tie Robot legacy show_full demo mode", demo_service)
        self.assertIn("WorkingDirectory=@LEGACY_WORKSPACE@", demo_service)
        self.assertIn("ROS_MASTER_URI=http://127.0.0.1:11311", demo_service)
        self.assertIn("tie-robot-demo-rosbridge.service", demo_service)
        self.assertNotIn("tie-robot-rosbridge.service", demo_service)
        self.assertIn("ScepterSDK/3rd-PartyPlugin/ROS", demo_service)
        self.assertIn("roslaunch chassis_ctrl show_full.launch", demo_service)
        self.assertIn("KillSignal=SIGINT", demo_service)
        self.assertIn("TimeoutStopSec=15", demo_service)
        self.assertNotIn("rosbridge_websocket.launch", demo_service)
        self.assertNotIn("show_legacy_shared_driver_stack.launch", demo_service)
        self.assertNotIn("show_legacy_driver_bridge", demo_service)

        self.assertIn("Description=Tie Robot demo lightweight rosbridge", demo_rosbridge_service)
        self.assertIn("roslaunch tie_robot_bringup demo_rosbridge_light.launch", demo_rosbridge_service)
        self.assertIn('name="rosbridge_websocket"', demo_rosbridge_launch)
        self.assertIn('name="rosapi"', demo_rosbridge_launch)
        self.assertIn("/pointAI/result_image", demo_rosbridge_launch)
        self.assertIn("/Scepter/ir/image_raw/compressed", demo_rosbridge_launch)
        self.assertIn("/Scepter/depth/image_raw/compressed", demo_rosbridge_launch)
        self.assertNotIn("/Scepter/ir/image_raw,", demo_rosbridge_launch)
        self.assertNotIn("/Scepter/depth/image_raw,", demo_rosbridge_launch)
        self.assertNotIn("tf_stack.launch", demo_rosbridge_launch)
        self.assertNotIn("api.launch", demo_rosbridge_launch)
        self.assertNotIn("robot_tf_broadcaster", demo_rosbridge_launch)
        self.assertNotIn("web_action_bridge_node", demo_rosbridge_launch)

        self.assertIn("tie-robot-demo-show-full.service", demo_installer)
        self.assertIn("tie-robot-demo-rosbridge.service", demo_installer)
        self.assertIn("systemctl disable tie-robot-demo-show-full.service", demo_installer)
        self.assertIn("systemctl disable tie-robot-demo-rosbridge.service", demo_installer)
        self.assertIn(
            "systemctl disable --now tie-robot-show-legacy-shared-driver-stack.service",
            demo_installer,
        )
        self.assertNotIn("systemctl enable tie-robot-demo-show-full.service", demo_installer)
        self.assertIn("Wants=network-online.target", legacy_frontend_service)
        self.assertNotIn("tie-robot-rosbridge.service", legacy_frontend_service)
        self.assertIn("install_show_legacy_frontend_service.sh", frontend_installer)
        self.assertIn("install_demo_mode_service.sh", frontend_installer)

        self.assertIn("/usr/bin/systemctl start tie-robot-demo-show-full.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl stop tie-robot-demo-show-full.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl start tie-robot-demo-rosbridge.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl stop tie-robot-demo-rosbridge.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl start tie-robot-show-legacy-frontend.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl stop tie-robot-show-legacy-shared-driver-stack.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl reset-failed tie-robot-demo-show-full.service", backend_sudoers)
        self.assertIn("/usr/bin/systemctl reset-failed tie-robot-demo-rosbridge.service", backend_sudoers)

        self.assertIn('DEMO_MODE_SERVICE = "tie-robot-demo-show-full.service"', server_script)
        self.assertIn('DEMO_ROSBRIDGE_SERVICE = "tie-robot-demo-rosbridge.service"', server_script)
        self.assertIn('LEGACY_FRONTEND_SERVICE = "tie-robot-show-legacy-frontend.service"', server_script)
        self.assertIn('LEGACY_SHARED_DRIVER_STACK_SERVICE = "tie-robot-show-legacy-shared-driver-stack.service"', server_script)
        self.assertIn('"/api/system/demo_mode_status"', server_script)
        self.assertIn('"/api/system/toggle_demo_mode"', server_script)
        self.assertIn("DEMO_MODE_STOP_ORDER = (", server_script)
        self.assertIn("DEMO_MODE_RESTORE_START_ORDER = (", server_script)
        self.assertIn("DEMO_MODE_CLEANUP_PROCESS_PATTERNS", server_script)
        self.assertIn("_cleanup_demo_mode_ros_artifacts", server_script)
        self.assertIn("_run_demo_mode_enter", server_script)
        self.assertIn("_run_demo_mode_exit", server_script)

        self.assertIn('id: "toggleDemoMode"', system_control_catalog)
        self.assertIn('httpEndpoint: "/api/system/toggle_demo_mode"', system_control_catalog)
        self.assertIn('statusEndpoint: "/api/system/demo_mode_status"', system_control_catalog)
        self.assertIn("refreshDemoModeStatus", system_control_controller)
        self.assertIn("legacyFrontendUrl", system_control_controller)
        self.assertIn("onOpenUrl", system_control_controller)
        self.assertIn('data-status-id="demoMode"', ui_controller)
        self.assertIn("setDemoModeState(active", ui_controller)
        self.assertIn("startDemoModeStatusPolling", app_logic)
        self.assertIn("refreshDemoModeStatus", app_logic)

    def test_demo_mode_toggle_stops_current_services_and_restores_without_bridge(self):
        handler = object.__new__(self.server_module.NoCacheStaticHandler)
        calls = []

        def fake_run_systemctl(action, *service_names, timeout=30):
            calls.append((action, service_names, timeout))

            class Completed:
                returncode = 0
                stdout = ""
                stderr = ""

            return Completed()

        def fake_query_systemd_status(service_name):
            return {
                "service": service_name,
                "loadState": "loaded",
                "activeState": "active",
                "subState": "running",
                "mainPid": 42,
            }

        handler._run_systemctl = fake_run_systemctl
        handler._query_systemd_status = fake_query_systemd_status
        handler._cleanup_demo_mode_ros_artifacts = lambda: self.server_module.subprocess.CompletedProcess(
            ["cleanup-demo-mode-ros"],
            0,
            "清理完成",
            "",
        )
        enter_results = handler._run_demo_mode_enter()
        exit_results = handler._run_demo_mode_exit()

        self.assertEqual(
            calls,
            [
                ("start", (self.server_module.LEGACY_FRONTEND_SERVICE,), 30),
                ("stop", (self.server_module.ROS_BACKEND_SERVICE,), self.server_module.DEMO_MODE_STOP_TIMEOUT_SEC),
                ("stop", (self.server_module.DRIVER_SYSTEMD_SERVICES["cabin"],), self.server_module.DEMO_MODE_STOP_TIMEOUT_SEC),
                ("stop", (self.server_module.DRIVER_SYSTEMD_SERVICES["moduan"],), self.server_module.DEMO_MODE_STOP_TIMEOUT_SEC),
                ("stop", (self.server_module.DRIVER_SYSTEMD_SERVICES["camera"],), self.server_module.DEMO_MODE_STOP_TIMEOUT_SEC),
                ("stop", (self.server_module.ROSBRIDGE_SERVICE,), self.server_module.DEMO_MODE_STOP_TIMEOUT_SEC),
                ("stop", (self.server_module.LEGACY_SHARED_DRIVER_STACK_SERVICE,), self.server_module.DEMO_MODE_STOP_TIMEOUT_SEC),
                ("start", (self.server_module.DEMO_ROSBRIDGE_SERVICE,), 30),
                ("start", (self.server_module.DEMO_MODE_SERVICE,), 30),
                ("stop", (self.server_module.DEMO_MODE_SERVICE,), self.server_module.DEMO_MODE_STOP_TIMEOUT_SEC),
                ("stop", (self.server_module.DEMO_ROSBRIDGE_SERVICE,), self.server_module.DEMO_MODE_STOP_TIMEOUT_SEC),
                ("reset-failed", (self.server_module.DEMO_MODE_SERVICE,), 30),
                ("reset-failed", (self.server_module.DEMO_ROSBRIDGE_SERVICE,), 30),
                ("start", (self.server_module.ROSBRIDGE_SERVICE,), 30),
                ("start", (self.server_module.DRIVER_SYSTEMD_SERVICES["cabin"],), 30),
                ("start", (self.server_module.DRIVER_SYSTEMD_SERVICES["moduan"],), 30),
                ("start", (self.server_module.DRIVER_SYSTEMD_SERVICES["camera"],), 30),
                ("start", (self.server_module.ROS_BACKEND_SERVICE,), 30),
            ],
        )
        self.assertEqual([item["action"] for item in enter_results], [
            "start",
            "stop",
            "stop",
            "stop",
            "stop",
            "stop",
            "stop",
            "start",
            "cleanup_demo_mode_ros",
            "start",
        ])
        self.assertEqual(exit_results[0]["services"], (self.server_module.DEMO_MODE_SERVICE,))
        self.assertEqual(exit_results[1]["action"], "cleanup_demo_mode_ros")
        self.assertEqual(exit_results[2]["services"], (self.server_module.DEMO_ROSBRIDGE_SERVICE,))
        self.assertEqual(exit_results[3]["action"], "reset-failed")
        self.assertEqual(exit_results[4]["services"], (self.server_module.DEMO_ROSBRIDGE_SERVICE,))

    def test_restart_ros_stack_stops_everything_before_restarting_dependencies(self):
        handler = object.__new__(self.server_module.NoCacheStaticHandler)
        calls = []

        def fake_run_systemctl(action, *service_names, timeout=30):
            calls.append((action, service_names, timeout))

            class Completed:
                returncode = 0
                stdout = ""
                stderr = ""

            return Completed()

        handler._run_systemctl = fake_run_systemctl
        handler._fast_kill_stale_ros_processes = lambda: self.server_module.subprocess.CompletedProcess(
            ["fast-kill-stale-ros"],
            0,
            "清理完成",
            "",
        )
        results = handler._run_full_ros_stack_restart()

        self.assertEqual(
            calls,
            [
                ("stop", self.server_module.FULL_ROS_STACK_STOP_ORDER, 8),
                ("start", (self.server_module.ROSBRIDGE_SERVICE,), 30),
                ("start", (self.server_module.DRIVER_SYSTEMD_SERVICES["cabin"],), 30),
                ("start", (self.server_module.DRIVER_SYSTEMD_SERVICES["moduan"],), 30),
                ("start", (self.server_module.DRIVER_SYSTEMD_SERVICES["camera"],), 30),
                ("start", (self.server_module.ROS_BACKEND_SERVICE,), 30),
            ],
        )
        self.assertEqual(
            [item["phase"] for item in results],
            ["stop", "cleanup", "start", "start", "start", "start", "start"],
        )
        self.assertEqual(results[0]["services"], self.server_module.FULL_ROS_STACK_STOP_ORDER)
        self.assertEqual(results[1]["action"], "fast_kill_stale_ros")

    def test_systemctl_timeout_returns_failed_completed_process(self):
        handler = object.__new__(self.server_module.NoCacheStaticHandler)
        original_run = self.server_module.subprocess.run

        def fake_run(command, **_kwargs):
            raise self.server_module.subprocess.TimeoutExpired(
                cmd=command,
                timeout=8,
                output="",
                stderr="slow stop",
            )

        self.server_module.subprocess.run = fake_run
        try:
            result = handler._run_systemctl("stop", "tie-robot-backend.service", timeout=8)
        finally:
            self.server_module.subprocess.run = original_run

        self.assertEqual(result.returncode, 124)
        self.assertIn("systemctl stop tie-robot-backend.service 超时", result.stderr)

    def test_fast_ros_cleanup_terminates_then_kills_leftover_processes(self):
        handler = object.__new__(self.server_module.NoCacheStaticHandler)
        snapshots = [
            [
                (101, "python3 /opt/ros/noetic/bin/rosmaster --core -p 11311"),
                (102, "/home/hyq-/simple_lashingrobot_ws/devel/lib/tie_robot_perception/scepter_camera"),
            ],
            [
                (102, "/home/hyq-/simple_lashingrobot_ws/devel/lib/tie_robot_perception/scepter_camera"),
            ],
            [],
        ]
        signals = []
        sleeps = []

        def fake_find_processes():
            return snapshots.pop(0) if snapshots else []

        def fake_signal_processes(processes, signal_number):
            signals.append((signal_number, tuple(pid for pid, _command in processes)))
            return []

        handler._find_stale_ros_processes = fake_find_processes
        handler._signal_ros_processes = fake_signal_processes
        handler._sleep = lambda seconds: sleeps.append(seconds)

        result = handler._fast_kill_stale_ros_processes()

        self.assertEqual(result.returncode, 0)
        self.assertEqual(
            signals,
            [
                (self.server_module.signal.SIGTERM, (101, 102)),
                (self.server_module.signal.SIGKILL, (102,)),
            ],
        )
        self.assertEqual(
            sleeps,
            [
                self.server_module.ROS_FAST_KILL_TERM_GRACE_SEC,
                self.server_module.ROS_FAST_KILL_KILL_GRACE_SEC,
            ],
        )
        self.assertIn("2 个旧 ROS 进程", result.stdout)

    def test_terminal_tool_uses_backend_session_api_and_multi_session_panel(self):
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        terminal_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TerminalController.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
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
        self.assertIn('this.ui.setTerminalNotice("正在创建终端会话…", "info");', terminal_controller)
        self.assertIn("this.ui.setTerminalNotice(null);", terminal_controller)
        self.assertIn("setTerminalNotice(message, level = \"info\")", ui_controller)
        self.assertIn("this.refs.terminalEmptyState.dataset.state = level;", ui_controller)
        self.assertIn('class="terminal-empty-state" data-state="idle"', ui_controller)
        self.assertIn('this.ui.setTerminalNotice(`终端操作失败：${error?.message || String(error)}`, "error");', app_logic)
        self.assertIn('this.refs.terminalTabStrip?.addEventListener("click"', ui_controller)
        self.assertIn("renderTerminalSessions(sessions, activeSessionId)", ui_controller)

    def test_terminal_backend_uses_default_pty_sessions_without_tmux(self):
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        terminal_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TerminalController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")

        self.assertIn('TERMINAL_SHELL_RC_DIR = Path("/tmp/tie_robot_web_terminal_rc")', server_script)
        self.assertIn("def _spawn_shell_process(self):", server_script)
        self.assertIn("pty.openpty()", server_script)
        self.assertIn("build_terminal_shell_command(self.rcfile_path)", server_script)
        self.assertIn('env["WORKSPACE_PICKER_TERMINAL_SESSION_ID"] = self.session_id', server_script)
        self.assertIn("def is_alive(self):", server_script)
        self.assertIn('label = self.build_session_label()', server_script)
        self.assertIn("session.close(terminate_process=True)", server_script)
        self.assertIn("创建终端失败", server_script)
        self.assertNotIn("TERMINAL_TMUX_PREFIX", server_script)
        self.assertNotIn("GENERIC_TMUX_WINDOW_NAMES", server_script)
        self.assertNotIn('shutil.which("tmux")', server_script)
        self.assertNotIn("discover_existing_sessions", server_script)
        self.assertNotIn('"new-session"', server_script)
        self.assertNotIn('"rename-window"', server_script)
        self.assertNotIn('"attach-session"', server_script)
        self.assertNotIn('"kill-session"', server_script)
        self.assertNotIn("refresh_label_from_tmux_window", server_script)
        self.assertNotIn('"display-message"', server_script)
        self.assertNotIn("tmuxSessionName", server_script)
        self.assertNotIn('label = f"终端 {self.sequence}"', server_script)
        self.assertNotIn('f"终端 {len(self.sessions) + 1}"', server_script)
        self.assertIn("label: session.label || sessionId", terminal_controller)
        self.assertNotIn("tmuxSessionName", terminal_controller)
        self.assertNotIn('label: session.label || `终端 ${this.sessions.size + 1}`', terminal_controller)
        self.assertIn("hydrateExistingSessions", terminal_controller)
        self.assertIn("startSessionLabelSync", terminal_controller)
        self.assertIn("refreshSessionSummaries", terminal_controller)
        self.assertIn("normalizeSessionPayload", terminal_controller)
        self.assertIn("config?.sessions", terminal_controller)
        self.assertIn("this.connectSessionSocket(session.sessionId);", terminal_controller)
        self.assertIn('this.restoreTerminalPanelIfVisible();', app_logic)

    def test_frontend_persists_floating_panel_layout_across_refresh(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        panel_manager = (
            FRONTEND_SRC_DIR / "ui" / "PanelManager.js"
        ).read_text(encoding="utf-8")

        self.assertIn("persistActiveLayout()", app_logic)
        self.assertIn("createLayoutSnapshot()", app_logic)
        self.assertIn("this.layoutManager.persistLayout(this.activeLayout.id", app_logic)
        self.assertIn("window.addEventListener(\"beforeunload\", this.handleWindowBeforeUnload, true);", app_logic)
        self.assertIn('this.panelManager.applyPanelLayout(this.activeLayout);', app_logic)
        self.assertIn('if (!this.panelManager.hasPanelLayout(panelId)) {', app_logic)
        self.assertIn("getPanelVisibilitySnapshot()", ui_controller)
        self.assertIn("applyPanelLayout(layout)", panel_manager)
        self.assertIn("getPanelLayoutSnapshot()", panel_manager)
        self.assertIn("hasPanelLayout(panelId)", panel_manager)
        self.assertIn("this.onLayoutChange?.(this.getPanelLayoutSnapshot());", panel_manager)

    def test_logs_strip_legacy_source_prefixes_and_do_not_promote_unable_text_to_error(self):
        process_common = (
            TIE_ROBOT_PROCESS_DIR
            / "include"
            / "tie_robot_process"
            / "common.hpp"
        ).read_text(encoding="utf-8")
        log_text_utils = (
            FRONTEND_SRC_DIR / "utils" / "logText.js"
        ).read_text(encoding="utf-8")

        self.assertIn("LEGACY_LOG_SOURCE_PREFIX_PATTERN", log_text_utils)
        self.assertIn("ROS_LOG_HEADER_PATTERN", log_text_utils)
        self.assertIn("ros_log_extract_legacy_prefix(message, &legacy_level_tag)", process_common)
        self.assertIn('legacy_level_tag == "warn"', process_common)
        self.assertIn('legacy_level_tag == "error"', process_common)
        self.assertNotIn('failure_tokens[] = {"失败", "无法"}', process_common)

    def test_terminal_graphical_commands_open_frontend_cards(self):
        server_script = SERVER_SCRIPT.read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        terminal_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TerminalController.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        panel_manager = (
            FRONTEND_SRC_DIR / "ui" / "PanelManager.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn("GRAPHICAL_COMMAND_ALIASES", server_script)
        self.assertIn('"rviz"', server_script)
        self.assertIn('"rqt"', server_script)
        self.assertIn("class GraphicalAppSessionManager", server_script)
        self.assertIn("GUI_PROXY_PREFIX", server_script)
        self.assertIn("def get_session(self, session_id):", server_script)
        self.assertIn("def forget_session(self, session_id):", server_script)
        self.assertIn("if not session.closed and session.state != \"closed\"", server_script)
        self.assertIn("self.manager.forget_session(self.session_id)", server_script)
        self.assertIn("def handle_graphical_app_proxy_get(self, parsed):", server_script)
        self.assertIn("def proxy_graphical_app_websocket(self, session, target_path):", server_script)
        self.assertIn("def wait_for_graphical_app_port(self, session, timeout=12.0):", server_script)
        self.assertIn("def fit_to_viewport(self, width, height):", server_script)
        self.assertIn("def should_skip_backend_geometry_fit(self):", server_script)
        self.assertIn('return command_name in {"rviz", "rviz2"}', server_script)
        self.assertIn("跳过后端强制缩放", server_script)
        self.assertIn("def _find_primary_window_id(self):", server_script)
        self.assertIn("wmctrl", server_script)
        self.assertIn("xdotool", server_script)
        self.assertIn("ready_deadline = time.time() + 30.0", server_script)
        self.assertIn('"图形界面端口启动超时，请关闭后重试。"', server_script)
        self.assertIn("socket.create_connection((\"127.0.0.1\", int(session.web_port))", server_script)
        self.assertIn("def should_suppress_duplicate_index_request", server_script)
        self.assertIn("duplicate_index_request_window_sec", server_script)
        self.assertIn('if "embed_instance=" in str(target_path or ""):', server_script)
        self.assertIn("self.send_response(204)", server_script)
        self.assertIn("def _is_web_http_ready(self, timeout=0.8):", server_script)
        self.assertIn('connection.request("GET", "/", headers={"Host": f"127.0.0.1:{int(self.web_port)}"})', server_script)
        self.assertIn("if session._is_web_http_ready(timeout=0.6):", server_script)
        self.assertIn("def handle_graphical_app_session_unhealthy(self, session, message):", server_script)
        self.assertIn("def send_graphical_app_session_closed_page(self, session_id, message, status_code=410):", server_script)
        self.assertIn('"tie-robot-gui-session-closed"', server_script)
        self.assertIn('if parsed.path == "/api/gui/sessions":', server_script)
        self.assertIn("if parsed.path.startswith(GUI_PROXY_PREFIX):", server_script)
        self.assertIn('parsed.path.startswith("/api/gui/sessions/")', server_script)
        self.assertIn('parsed.path.endswith("/geometry")', server_script)
        self.assertIn("handle_graphical_app_geometry_post", server_script)
        self.assertIn("build_terminal_shell_command", server_script)
        self.assertIn("WORKSPACE_PICKER_GUI_API", server_script)
        self.assertIn("WORKSPACE_PICKER_TERMINAL_SESSION_ID", server_script)
        self.assertIn("import signal", server_script)
        self.assertIn("from urllib.parse import quote", server_script)
        self.assertIn('session_id = str(session.get("sessionId") or result.get("session_id") or "")', server_script)
        self.assertIn("def close_graphical_session(exit_code=130):", server_script)
        self.assertIn("urllib.request.Request(session_url, method=\"DELETE\")", server_script)
        self.assertIn("signal.signal(signal.SIGINT, handle_interrupt)", server_script)
        self.assertIn("signal.signal(signal.SIGHUP, handle_interrupt)", server_script)
        self.assertIn("while True:", server_script)
        self.assertIn("time.sleep(0.8)", server_script)
        self.assertIn("rbufsize = 0", server_script)
        self.assertIn("except http.client.IncompleteRead as exc:", server_script)
        self.assertIn('"accept-encoding"', server_script)
        self.assertIn("def _patch_xpra_html5_resource(target_path, body):", server_script)
        self.assertIn('b"var proto_flags = 0x1;"', server_script)
        self.assertIn('b\'"rencode": true,\'', server_script)
        self.assertIn('b\'      "encodings": this.supported_encodings,\\n\'', server_script)
        self.assertIn('b\'      "encodings.core": this.supported_encodings,\\n\'', server_script)
        self.assertIn("QT_X11_NO_MITSHM=1", server_script)
        self.assertIn("LIBGL_ALWAYS_SOFTWARE=1", server_script)
        self.assertIn("DISABLE_ROS1_EOL_WARNINGS=1", server_script)
        self.assertIn('"--sharing=yes"', server_script)
        self.assertIn('"--lock=no"', server_script)
        self.assertIn('headers["Connection"] = "close"', server_script)
        self.assertNotIn('if path_only.endswith("/index.html") or path_only == "/":', server_script)
        self.assertNotIn("tie-robot-xpra-native-style", server_script)
        self.assertIn('if path_only.endswith("/js/Window.js"):', server_script)
        self.assertIn("is_tie_robot_embedded_main_window", server_script)
        self.assertIn("tie-robot-embedded-main-window", server_script)
        self.assertIn('if path_only.endswith("/css/client.css"):', server_script)
        self.assertNotIn("decorations = false", server_script)
        self.assertNotIn("set_maximized(true)", server_script)
        self.assertIn("client_frame_buffer.extend(data)", server_script)
        self.assertIn("def _forward_complete_websocket_frames(self, frame_buffer, destination):", server_script)
        self.assertIn("def _next_websocket_frame_size(frame_buffer):", server_script)
        self.assertIn('f"--bind-ws=0.0.0.0:{self.web_port}"', server_script)
        self.assertIn('"--exit-with-children=yes"', server_script)
        self.assertIn('f"--start-child={command_text}"', server_script)
        self.assertNotIn('f"--start={command_text}"', server_script)
        self.assertNotIn('f"--bind-tcp=0.0.0.0:{self.web_port}"', server_script)
        self.assertNotIn('f"--bind-tcp=127.0.0.1:{self.web_port}"', server_script)
        self.assertIn('type": "gui_session"', server_script)
        self.assertIn('type": "gui_sessions"', server_script)
        self.assertIn('type": "gui_session_closed"', server_script)
        self.assertIn('this.callbacks.onGraphicalSession?.(payload.session);', terminal_controller)
        self.assertIn('this.callbacks.onGraphicalSessions?.(payload.sessions || []);', terminal_controller)
        self.assertIn("graphicalAppPanelDock", ui_controller)
        self.assertIn("renderGraphicalAppSessions(sessions)", ui_controller)
        self.assertIn("upsertGraphicalAppPanel(session, index)", ui_controller)
        self.assertNotIn("graphicalAppPanelDock.innerHTML", ui_controller)
        self.assertIn("getGraphicalAppSessionSignature(sessions)", app_logic)
        self.assertIn("this.graphicalAppSessionSignature", app_logic)
        self.assertIn("if (nextSignature === this.graphicalAppSessionSignature) {", app_logic)
        self.assertIn('const canRenderFrame = Boolean(frameUrl) && session.state === "ready";', ui_controller)
        self.assertIn("activateGraphicalAppPanelDrag(panel)", ui_controller)
        self.assertIn('const header = panel.querySelector(".graphical-app-header");', ui_controller)
        self.assertIn('header.addEventListener("mousedown", (event) => {', ui_controller)
        self.assertIn('new URLSearchParams({', ui_controller)
        self.assertIn('const proxyPath = `/api/gui/proxy/${encodeURIComponent(session.sessionId)}/`;', ui_controller)
        self.assertIn('submit: "true"', ui_controller)
        self.assertIn("path: proxyPath", ui_controller)
        self.assertIn("embed_instance: this.graphicalAppEmbedInstanceId", ui_controller)
        self.assertIn('floating_menu: "0"', ui_controller)
        self.assertIn('sharing: "1"', ui_controller)
        self.assertIn('offscreen: "0"', ui_controller)
        self.assertIn('frame.style.width = "100%";', ui_controller)
        self.assertIn('frame.style.height = "100%";', ui_controller)
        self.assertNotIn("requestGraphicalAppFrameRedraw(frame);", ui_controller)
        self.assertNotIn('frame.contentWindow.dispatchEvent(new Event("resize"));', ui_controller)
        self.assertNotIn("resetGraphicalAppWindowSurface(frame);", ui_controller)
        self.assertNotIn("panel.__graphicalAppRedrawTimer", ui_controller)
        self.assertNotIn("resetGraphicalAppWindowSurface(frame) {", ui_controller)
        self.assertNotIn("windowElement.style.transform = \"\";", ui_controller)
        self.assertNotIn("canvas.style.transform = \"\";", ui_controller)
        self.assertNotIn("screen.style.background = \"#071b33\";", ui_controller)
        self.assertNotIn("scaleGraphicalAppWindowSurface(frame);", ui_controller)
        self.assertNotIn("const scale = Math.min(scaleX, scaleY);", ui_controller)
        self.assertNotIn("const scale = Math.max(scaleX, scaleY);", ui_controller)
        self.assertNotIn("windowElement.style.transform = `translate(${offsetX}px, ${offsetY}px) scale(${scale})`;", ui_controller)
        self.assertNotIn("windowElement.style.transformOrigin = \"0 0\";", ui_controller)
        self.assertNotIn("scale(${scaleX}, ${scaleY})", ui_controller)
        self.assertNotIn("this.fitGraphicalAppWindowsToFrame(frame);", ui_controller)
        self.assertNotIn("syncGraphicalAppHeaderIcon(panel, windowRef);", ui_controller)
        self.assertNotIn("applyGraphicalAppEmbeddedWindowChrome", ui_controller)
        self.assertNotIn("windowRef.w =", ui_controller)
        self.assertNotIn("windowRef.h =", ui_controller)
        self.assertNotIn("windowRef.handle_resized", ui_controller)
        self.assertNotIn("windowRef._set_decorated?.(false);", ui_controller)
        self.assertNotIn("windowRef.leftoffset = 0;", ui_controller)
        self.assertNotIn("windowRef.topoffset = 0;", ui_controller)
        self.assertNotIn("this.requestGraphicalAppBackendGeometry(panel, frame);", ui_controller)
        self.assertIn("new ResizeObserver", ui_controller)
        self.assertNotIn('fetch(`/api/gui/sessions/${encodeURIComponent(sessionId)}/geometry`', ui_controller)
        self.assertNotIn("requestXpraRedraw();", ui_controller)
        self.assertNotIn("xpraClient.redraw_windows?.();", ui_controller)
        self.assertNotIn("xpraClient.request_refresh?.(windowRef.wid);", ui_controller)
        self.assertIn("getGraphicalAppPanelZIndex(sessionId)", ui_controller)
        self.assertIn("bringGraphicalAppPanelToFront(panel)", ui_controller)
        self.assertIn("this.bringGraphicalAppPanelToFront(panel);", ui_controller)
        self.assertIn('focusCatcher.addEventListener("pointerdown"', ui_controller)
        self.assertIn("event.stopPropagation();", ui_controller)
        self.assertIn('frameWindow.addEventListener("pointerdown", bringToFront, true);', ui_controller)
        self.assertNotIn("server: window.location.hostname", ui_controller)
        self.assertNotIn("server: host", ui_controller)
        self.assertNotIn("port: window.location.port ||", ui_controller)
        self.assertNotIn('ssl: "0"', ui_controller)
        self.assertIn("graphical-app-focus-catcher", ui_controller)
        self.assertIn("data-gui-session-maximize", ui_controller)
        self.assertIn("graphical-app-icon", ui_controller)
        self.assertIn("toggleGraphicalAppMaximized(sessionId)", ui_controller)
        self.assertIn('panel.classList.toggle("maximized", Boolean(state?.maximized));', ui_controller)
        self.assertIn('panel.classList.add("is-window-active");', ui_controller)
        self.assertIn('panel.classList.add("is-window-active");', panel_manager)
        self.assertIn('document.querySelectorAll(".floating-panel, .graphical-app-panel")', ui_controller)
        self.assertIn('document.querySelectorAll(".floating-panel, .graphical-app-panel")', panel_manager)
        self.assertNotIn('panel.addEventListener("pointerenter", bringToFront);', ui_controller)
        self.assertIn("onGraphicalAppAction(callback)", ui_controller)
        self.assertIn("data-gui-session-close", ui_controller)
        self.assertIn("handleGraphicalAppSession", app_logic)
        self.assertIn('this.ui.renderGraphicalAppSessions(this.graphicalAppSessions);', app_logic)
        self.assertIn("this.handleGraphicalAppFrameMessage = this.handleGraphicalAppFrameMessage.bind(this);", app_logic)
        self.assertIn('window.addEventListener("message", this.handleGraphicalAppFrameMessage);', app_logic)
        self.assertIn("startGraphicalAppSessionPolling()", app_logic)
        self.assertIn("this.graphicalAppSessionPollTimer = window.setInterval", app_logic)
        self.assertIn('event.data?.type !== "tie-robot-gui-session-closed"', app_logic)
        self.assertIn("this.refreshGraphicalAppSessions();", app_logic)
        self.assertIn("async refreshGraphicalAppSessions(options = {})", app_logic)
        self.assertIn('await fetch("/api/gui/sessions")', app_logic)
        self.assertIn('await fetch(`/api/gui/sessions/${sessionId}`', app_logic)
        self.assertIn(".graphical-app-panel", app_css)
        self.assertIn(".graphical-app-panel.maximized", app_css)
        self.assertIn(".graphical-app-frame", app_css)
        self.assertIn(".graphical-app-focus-catcher", app_css)
        self.assertIn(".graphical-app-panel:not(.is-window-active) .graphical-app-focus-catcher", app_css)
        self.assertIn(".graphical-app-icon", app_css)
        self.assertIn(".graphical-app-panel.is-window-active .graphical-app-focus-catcher", app_css)
        self.assertIn("contain: strict;", app_css)
        self.assertNotIn(".graphical-app-panel-dock {\n  position: fixed;\n  inset: 0;\n  z-index:", app_css)
        self.assertIn("cursor: move;", app_css)

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
        control_panel_catalog = (
            FRONTEND_SRC_DIR / "config" / "controlPanelCatalog.js"
        ).read_text(encoding="utf-8")

        self.assertIn('{ id: "runSavedS2", label: "触发\\n视觉识别", tone: "green" }', control_panel_catalog)
        self.assertIn("onWorkspaceS2Triggered: () => this.handleWorkspaceS2Triggered(),", app_logic)
        self.assertIn("handleWorkspaceS2Triggered()", app_logic)
        self.assertNotIn("ensureS2OverlayDisplayTopic()", app_logic)
        self.assertIn("scheduleS2ResultTimeout()", app_logic)
        self.assertIn("clearS2ResultTimeout()", app_logic)
        self.assertIn("this.surfaceDpOverlayRequested = true;", app_logic)
        self.assertNotIn('this.ui.setSelectedImageTopic(DEFAULT_IMAGE_TOPIC);', app_logic)
        self.assertIn("视觉识别会叠加在当前图像图层，不切换图像话题。", app_logic)
        self.assertIn("后端视觉识别结果图已叠加在当前图像图层", app_logic)
        self.assertIn("视觉识别已触发，但等待后端结果图超时", app_logic)
        self.assertIn("this.clearS2ResultTimeout();", app_logic)
        self.assertIn("this.callbacks.onWorkspaceS2Triggered?.();", task_action_controller)
        self.assertLess(
            task_action_controller.index("this.callbacks.onWorkspaceS2Triggered?.();"),
            task_action_controller.index("await this.rosConnection.callLashingRecognizeOnceService();"),
        )

    def test_s2_trigger_caches_latched_visual_points_without_replacing_backend_overlay(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")

        self.assertIn("this.latestVisualRecognitionPointsMessage = null;", app_logic)
        self.assertIn("cacheLatestVisualRecognitionPointsMessage(message)", app_logic)
        self.assertIn("this.latestVisualRecognitionPointsMessage = message;", app_logic)
        self.assertNotIn("applyCachedVisualRecognitionPointsOverlay()", app_logic)
        self.assertNotIn("const cachedPointCount = this.applyCachedVisualRecognitionPointsOverlay();", app_logic)
        self.assertIn("return pointsLength;", app_logic)
        self.assertLess(
            app_logic.index("this.cacheLatestVisualRecognitionPointsMessage(message);"),
            app_logic.index("const requestActive = this.surfaceDpOverlayActive || this.surfaceDpOverlayRequested;"),
        )

    def test_visual_recognition_backend_overlay_can_be_cleared(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        control_panel_catalog = (
            FRONTEND_SRC_DIR / "config" / "controlPanelCatalog.js"
        ).read_text(encoding="utf-8")

        self.assertIn('{ id: "clearVisualRecognition", label: "清除\\n识别结果", tone: "blue" }', control_panel_catalog)
        self.assertLess(
            control_panel_catalog.index('id: "runSavedS2"'),
            control_panel_catalog.index('id: "clearVisualRecognition"'),
        )
        self.assertLess(
            control_panel_catalog.index('id: "clearVisualRecognition"'),
            control_panel_catalog.index('id: "triggerSingleBind"'),
        )

        self.assertIn('if (taskAction === "clearVisualRecognition")', app_logic)
        self.assertIn("handleClearVisualRecognitionOverlay()", app_logic)
        self.assertIn("this.clearS2ResultTimeout();", app_logic)
        self.assertIn("this.surfaceDpOverlayActive = false;", app_logic)
        self.assertIn("this.surfaceDpOverlayRequested = false;", app_logic)
        self.assertIn("this.visualRecognitionOverlayCleared = true;", app_logic)
        self.assertIn("this.visualRecognitionOverlayCompleted = false;", app_logic)
        self.assertIn("this.workspaceView.setS2OverlayMessage(null);", app_logic)
        self.assertIn("this.workspaceView.setVisualRecognitionPointsMessage(null);", app_logic)
        self.assertIn("this.workspaceView.setVisualRecognitionOverlaySourceSize(null);", app_logic)
        self.assertIn("图像卡片显示原图", app_logic)
        self.assertIn("clearVisualRecognition: true", app_logic)
        self.assertIn("if (this.visualRecognitionOverlayCleared) {\n          return;\n        }", app_logic)
        self.assertIn("this.visualRecognitionOverlayCleared = false;", app_logic)

    def test_single_point_bind_button_calls_atomic_backend_service(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        task_action_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TaskActionController.js"
        ).read_text(encoding="utf-8")
        ros_connection_controller = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        control_panel_catalog = (
            FRONTEND_SRC_DIR / "config" / "controlPanelCatalog.js"
        ).read_text(encoding="utf-8")
        topic_registry = (
            FRONTEND_SRC_DIR / "config" / "topicRegistry.js"
        ).read_text(encoding="utf-8")

        self.assertIn('{ id: "runSavedS2", label: "触发\\n视觉识别", tone: "green" }', control_panel_catalog)
        self.assertIn('{ id: "triggerSingleBind", label: "触发单点\\n绑扎", tone: "red" }', control_panel_catalog)
        self.assertIn('case "triggerSingleBind":', task_action_controller)
        self.assertIn("return this.triggerSinglePointBind();", task_action_controller)
        self.assertIn("async triggerSinglePointBind()", task_action_controller)
        single_bind_start = task_action_controller.index("async triggerSinglePointBind()")
        single_bind_body = task_action_controller[
            single_bind_start:
            task_action_controller.index("\n  handleSavedWorkspacePayload", single_bind_start)
        ]
        self.assertIn("await this.rosConnection.callSinglePointBindService()", single_bind_body)
        self.assertNotIn("triggerSurfaceDpRecognition({", single_bind_body)
        self.assertNotIn("callLashingRecognizeOnceService", single_bind_body)
        trigger_only_body = task_action_controller[
            task_action_controller.index("\n  triggerSavedWorkspaceS2()"):
            task_action_controller.index("\n  async triggerSurfaceDpRecognition")
        ]
        self.assertNotIn("callSinglePointBindService", trigger_only_body)

        self.assertIn('singleBind: "/moduan/sg"', topic_registry)
        self.assertIn("singlePointBindService: new ROSLIB.Service({", ros_connection_controller)
        self.assertIn("name: SERVICES.moduan.singleBind", ros_connection_controller)
        self.assertIn("callSinglePointBindService()", ros_connection_controller)
        self.assertIn("triggerSingleBind:", app_logic)
        self.assertIn("resources?.singlePointBindService", app_logic)

    def test_settings_panel_renders_cabin_remote_page(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn('{ id: "cabinRemote", label: "索驱遥控" }', ui_controller)
        self.assertIn('data-settings-page="cabinRemote"', ui_controller)
        self.assertIn('id="cabinKeyboardRemoteToggle"', ui_controller)
        self.assertIn('id="cabinRemoteMoveMode"', ui_controller)
        self.assertIn('{ id: "absolute", label: "绝对点动" }', ui_controller)
        self.assertIn('{ id: "relative", label: "相对点动" }', ui_controller)
        self.assertIn('data-cabin-remote-move-mode="${mode.id}"', ui_controller)
        self.assertIn("绝对点动", ui_controller)
        self.assertIn("相对点动", ui_controller)
        self.assertIn('id="cabinRemoteStep"', ui_controller)
        self.assertIn('id="cabinRemoteSpeed"', ui_controller)
        self.assertIn('id="cabinRemoteAbsoluteX"', ui_controller)
        self.assertIn('id="cabinRemoteAbsoluteY"', ui_controller)
        self.assertIn('id="cabinRemoteAbsoluteZ"', ui_controller)
        self.assertIn('id="cabinRemoteAbsoluteMoveButton"', ui_controller)
        self.assertNotIn('id="cabinRemoteCurrentPosition"', ui_controller)
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
        self.assertIn("this.refs.cabinRemoteMoveMode =", ui_controller)
        self.assertIn("this.refs.cabinRemoteMoveModeButtons =", ui_controller)
        self.assertIn("this.refs.cabinRemoteStep =", ui_controller)
        self.assertIn("this.refs.cabinRemoteSpeed =", ui_controller)
        self.assertIn("this.refs.cabinRemoteAbsoluteX =", ui_controller)
        self.assertIn("this.refs.cabinRemoteAbsoluteMoveButton =", ui_controller)
        self.assertIn("this.refs.bottomCabinPosition =", ui_controller)
        self.assertIn("this.refs.bottomCabinPositionAxes =", ui_controller)
        self.assertIn("this.refs.bottomLinearModulePosition =", ui_controller)
        self.assertIn("this.refs.bottomLinearModuleAxes =", ui_controller)
        self.assertIn("this.refs.cabinRemoteAbsoluteY =", ui_controller)
        self.assertIn("this.refs.cabinRemoteAbsoluteZ =", ui_controller)
        self.assertIn("this.refs.cabinRemoteStatus =", ui_controller)
        self.assertIn("this.refs.cabinRemoteButtons =", ui_controller)
        self.assertIn("this.refs.cabinRemoteStopButton =", ui_controller)
        self.assertIn("getCabinRemoteSettings()", ui_controller)
        self.assertIn("setCabinRemoteMoveMode(moveMode)", ui_controller)
        self.assertIn("getCabinRemoteAbsoluteTarget()", ui_controller)
        self.assertIn("setCabinRemoteAbsoluteTarget(position)", ui_controller)
        self.assertIn("const isEditing = [", ui_controller)
        self.assertIn("onCabinRemoteAbsoluteMoveAction(callback)", ui_controller)
        self.assertIn("onCabinRemoteAction(callback)", ui_controller)
        self.assertIn("setCabinRemoteCurrentPosition(position)", ui_controller)
        self.assertIn("this.refs.bottomCabinPosition.dataset.state = hasPosition ? \"live\" : \"waiting\";", ui_controller)
        self.assertIn("axisNode.textContent = `${label} ${value}`;", ui_controller)
        self.assertIn("this.refs.bottomCabinPosition.setAttribute(\"aria-label\", title);", ui_controller)
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
        self.assertIn(".cabin-remote-mode-group {", app_css)
        self.assertIn(".cabin-remote-mode-button.is-active {", app_css)
        self.assertIn(".cabin-remote-pad {", app_css)
        self.assertIn(".cabin-remote-btn {", app_css)
        self.assertIn(".cabin-remote-btn.is-active {", app_css)
        self.assertIn(".cabin-remote-status-active {", app_css)
        self.assertIn(".cabin-remote-absolute-actions {", app_css)

    def test_cabin_remote_step_and_speed_are_persisted(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        storage = (
            FRONTEND_SRC_DIR / "utils" / "storage.js"
        ).read_text(encoding="utf-8")

        self.assertIn('export const CABIN_REMOTE_SETTINGS_KEY = "tie_robot_frontend_cabin_remote_settings";', storage)
        self.assertIn("loadCabinRemoteSettings()", storage)
        self.assertIn("saveCabinRemoteSettings(value)", storage)
        self.assertIn("step: normalizePositiveNumber(parsed?.step, defaults.step)", storage)
        self.assertIn("speed: normalizePositiveNumber(parsed?.speed, defaults.speed)", storage)
        self.assertIn('moveMode: parsed?.moveMode === "relative" ? "relative" : defaults.moveMode', storage)

        self.assertIn("setCabinRemoteSettings(settings)", ui_controller)
        self.assertIn("this.refs.cabinRemoteStep.value = String(step);", ui_controller)
        self.assertIn("this.refs.cabinRemoteSpeed.value = String(speed);", ui_controller)
        self.assertIn("this.setCabinRemoteMoveMode(settings?.moveMode);", ui_controller)

        self.assertIn("loadCabinRemoteSettings", app_logic)
        self.assertIn("saveCabinRemoteSettings", app_logic)
        self.assertIn("this.cabinRemoteSettings = loadCabinRemoteSettings();", app_logic)
        self.assertIn("this.ui.setCabinRemoteSettings(this.cabinRemoteSettings);", app_logic)
        self.assertIn("this.cabinRemoteSettings = settings;", app_logic)
        self.assertIn("saveCabinRemoteSettings(settings);", app_logic)

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
        cabin_remote_keyboard = (
            FRONTEND_SRC_DIR / "utils" / "cabinRemoteKeyboard.js"
        ).read_text(encoding="utf-8")
        ros_connection = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        topic_registry = (
            FRONTEND_SRC_DIR / "config" / "topicRegistry.js"
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
        self.assertIn('q: "zPositive"', cabin_remote_keyboard)
        self.assertIn('w: "xPositive"', cabin_remote_keyboard)
        self.assertIn('e: "zNegative"', cabin_remote_keyboard)
        self.assertIn('a: "yPositive"', cabin_remote_keyboard)
        self.assertIn('s: "xNegative"', cabin_remote_keyboard)
        self.assertIn('d: "yNegative"', cabin_remote_keyboard)
        self.assertIn('return { type: "stop" };', cabin_remote_keyboard)
        self.assertIn("handleCabinRemoteStopAction(source)", app_logic)
        self.assertIn('event.type === "stop"', app_logic)
        self.assertIn("await this.rosConnectionController.callCabinMotionStopService()", app_logic)
        self.assertIn("shouldIgnoreCabinRemoteKeyboardTarget(target, document.activeElement)", app_logic)
        self.assertIn("event.repeat", app_logic)
        self.assertIn("keyboardEnabled", app_logic)
        self.assertIn("this.cabinRemoteController.setLastKnownCabinPosition(payload);", app_logic)
        self.assertIn("this.ui.setCabinRemoteButtonsEnabled(", app_logic)
        self.assertIn("this.ui.setCabinRemoteCurrentPosition(currentCabinPosition);", app_logic)
        self.assertIn("this.ui.setCabinRemoteAbsoluteTarget(currentCabinPosition);", app_logic)
        self.assertIn("refreshCabinRemoteCurrentPosition()", app_logic)
        self.assertIn('callback(button.dataset.cabinRemoteDirection, { type: "click" });', ui_controller)
        self.assertNotIn("startCabinRemoteRepeat(directionId)", app_logic)
        self.assertNotIn("stopCabinRemoteRepeat()", app_logic)
        self.assertNotIn("scheduleCabinRemoteRepeatTick()", app_logic)
        self.assertNotIn("event.type === \"pressstart\"", app_logic)
        self.assertNotIn("event.type === \"pressend\"", app_logic)
        self.assertNotIn("this.cabinRemoteRepeatTimerId = window.setTimeout(", app_logic)
        self.assertIn("moveInFlight: this.cabinRemoteMoveInFlight,", app_logic)
        self.assertIn("if (!operationState.canOperate) {", app_logic)
        self.assertIn("moveToPosition: ready && Boolean(resources?.cabinSingleMoveService),", app_logic)
        self.assertIn("getCurrentCabinPositionMm()", scene_view)
        self.assertIn("const poseTransform = this.getWorldTransform(BASE_LINK_FRAME) || this.getWorldTransform(SCEPTER_FRAME);", scene_view)
        self.assertIn("x: poseTransform.position.x * 1000.0,", scene_view)
        self.assertIn("y: poseTransform.position.y * 1000.0,", scene_view)
        self.assertIn("z: poseTransform.position.z * 1000.0,", scene_view)
        self.assertIn("const CABIN_REMOTE_DIRECTION_DEFINITIONS = {", cabin_remote_controller)
        self.assertIn('xNegative: { axis: "x", delta: -1, label: "X-" }', cabin_remote_controller)
        self.assertIn('yPositive: { axis: "y", delta: 1, label: "Y+" }', cabin_remote_controller)
        self.assertIn('zPositive: { axis: "z", delta: 1, label: "Z+" }', cabin_remote_controller)
        self.assertIn("this.sceneView.getCurrentCabinPositionMm()", cabin_remote_controller)
        self.assertIn("this.lastKnownCabinPositionMm", cabin_remote_controller)
        self.assertIn("delta[definition.axis] = definition.delta * sanitizedStep;", cabin_remote_controller)
        self.assertIn("normalizeMoveMode(moveMode)", cabin_remote_controller)
        self.assertIn("callCabinSingleMoveService(target)", cabin_remote_controller)
        self.assertIn("callCabinIncrementalMoveService(target)", cabin_remote_controller)
        self.assertIn("syncGlobalCabinMoveSpeed({ suppressLog = false } = {})", app_logic)
        self.assertIn("this.rosConnectionController.publishCabinSpeed(speed)", app_logic)
        self.assertIn("TOPICS.process.setCabinSpeed", ros_connection)
        self.assertIn("cabinSpeedPublisher: new ROSLIB.Topic({", ros_connection)
        self.assertIn("publishCabinSpeed(speed)", ros_connection)
        self.assertIn("this.resources.cabinSpeedPublisher.publish(", ros_connection)
        self.assertIn("SERVICES.cabin.motionStop", ros_connection)
        self.assertIn("cabinIncrementalMoveService: new ROSLIB.Service({", ros_connection)
        self.assertIn("callCabinIncrementalMoveService({ x, y, z, speed })", ros_connection)
        self.assertIn('singleMove: "/cabin/driver/raw_move"', topic_registry)
        self.assertIn('incrementalMove: "/cabin/driver/incremental_move"', topic_registry)
        self.assertNotIn('singleMove: "/cabin/single_move"', topic_registry)
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

    def test_tcp_linear_module_remote_page_and_ros_flow_exist(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        tcp_remote_controller_path = (
            FRONTEND_SRC_DIR / "controllers" / "TcpLinearRemoteController.js"
        )
        ros_connection = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            FRONTEND_SRC_DIR / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertTrue(tcp_remote_controller_path.exists())
        tcp_remote_controller = tcp_remote_controller_path.read_text(encoding="utf-8")

        self.assertIn('{ id: "tcpLinearRemote", label: "TCP线模遥控" }', ui_controller)
        self.assertIn('{ id: "tcpLinearRemote", label: "TCP线模遥控" }', ui_controller)
        self.assertIn('data-settings-page="tcpLinearRemote"', ui_controller)
        self.assertIn('id="tcpLinearRemoteCurrentPosition"', ui_controller)
        self.assertIn('id="tcpLinearRemoteStatus"', ui_controller)
        self.assertIn('id="tcpLinearRemoteStep"', ui_controller)
        self.assertIn('id="tcpLinearRemoteAngleStep"', ui_controller)
        self.assertIn('data-tcp-linear-remote-axis="xPositive"', ui_controller)
        self.assertIn('data-tcp-linear-remote-axis="xNegative"', ui_controller)
        self.assertIn('data-tcp-linear-remote-axis="yPositive"', ui_controller)
        self.assertIn('data-tcp-linear-remote-axis="yNegative"', ui_controller)
        self.assertIn('data-tcp-linear-remote-axis="zPositive"', ui_controller)
        self.assertIn('data-tcp-linear-remote-axis="zNegative"', ui_controller)
        self.assertIn('data-tcp-linear-remote-axis="anglePositive"', ui_controller)
        self.assertIn('data-tcp-linear-remote-axis="angleNegative"', ui_controller)
        self.assertIn('data-tcp-linear-remote-stop="true"', ui_controller)
        self.assertLess(
            ui_controller.index('data-tcp-linear-remote-axis="anglePositive"'),
            ui_controller.index('data-tcp-linear-remote-axis="xNegative"'),
        )
        self.assertIn("callback(\"stopMotion\", { type: \"stop\" });", ui_controller)
        self.assertIn('getTcpLinearRemoteSettings()', ui_controller)
        self.assertIn('onTcpLinearRemoteAction(callback)', ui_controller)
        self.assertIn('onTcpLinearRemoteSettingsChange(callback)', ui_controller)
        self.assertIn('setTcpLinearRemoteState(message)', ui_controller)
        self.assertIn('setBottomLinearModulePosition(localPosition, globalPosition)', ui_controller)
        self.assertIn('setTcpLinearRemoteStatus(message)', ui_controller)
        self.assertIn('setTcpLinearRemoteButtonsEnabled(enabled)', ui_controller)

        self.assertIn('import { TcpLinearRemoteController } from "../controllers/TcpLinearRemoteController.js";', app_logic)
        self.assertIn("this.tcpLinearRemoteController = new TcpLinearRemoteController({", app_logic)
        self.assertIn("onLinearModuleState: (message) => {", app_logic)
        self.assertIn("this.tcpLinearRemoteController.setCurrentState(message);", app_logic)
        self.assertIn("this.ui.setTcpLinearRemoteState(message);", app_logic)
        self.assertIn("this.refreshBottomLinearModulePosition();", app_logic)
        self.assertIn("refreshBottomLinearModulePosition()", app_logic)
        self.assertIn("this.sceneView.getLinearModuleGlobalPositionMm(localPosition)", app_logic)
        self.assertIn("this.ui.onTcpLinearRemoteAction((directionId, event = {}) => {", app_logic)
        self.assertIn("handleTcpLinearRemoteDirection(directionId)", app_logic)
        self.assertIn('event.type === "stop"', app_logic)
        self.assertIn("handleTcpLinearRemoteStopAction()", app_logic)
        self.assertIn("await this.rosConnectionController.publishLinearModuleInterruptStop()", app_logic)
        self.assertIn("this.ui.setTcpLinearRemoteButtonsEnabled(", app_logic)
        self.assertIn("Boolean(resources?.linearModuleInterruptStopPublisher)", app_logic)

        self.assertIn('"control.linearModuleState"', ros_connection)
        self.assertIn("linearModuleInterruptStopPublisher: new ROSLIB.Topic({", ros_connection)
        self.assertIn("name: TOPICS.control.interruptStop", ros_connection)
        self.assertIn("messageType: MESSAGE_TYPES.float32", ros_connection)
        self.assertIn("publishLinearModuleInterruptStop()", ros_connection)
        self.assertIn("linearModuleInterruptStopPublisher.publish(new ROSLIB.Message({ data: 1 }))", ros_connection)
        self.assertIn("SERVICES.moduan.singleMove", ros_connection)
        self.assertIn("SERVICE_TYPES.moduan.linearModuleMove", ros_connection)
        self.assertIn("linearModuleSingleMoveService: new ROSLIB.Service({", ros_connection)
        self.assertIn("callLinearModuleSingleMoveService({ x, y, z, angle })", ros_connection)
        self.assertIn('this.buildTopicFromRegistry("control.linearModuleState")', ros_connection)
        self.assertIn("this.callbacks.onLinearModuleState?.(message)", ros_connection)

        self.assertIn("const TCP_LINEAR_REMOTE_DIRECTION_DEFINITIONS = {", tcp_remote_controller)
        self.assertIn('xPositive: { axis: "x", delta: 1, label: "X+" }', tcp_remote_controller)
        self.assertIn('zNegative: { axis: "z", delta: -1, label: "Z-" }', tcp_remote_controller)
        self.assertIn('anglePositive: { axis: "angle", delta: 1, label: "角度+" }', tcp_remote_controller)
        self.assertIn("linear_module_position_X", tcp_remote_controller)
        self.assertIn("callLinearModuleSingleMoveService(target)", tcp_remote_controller)
        self.assertIn("clampTcpLinearTarget(target)", tcp_remote_controller)

        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")
        self.assertIn("getLinearModuleGlobalPositionMm(localPosition)", scene_view)
        self.assertIn("this.getWorldTransform(GRIPPER_FRAME)", scene_view)
        self.assertIn("applyQuaternion(gripperTransform.quaternion)", scene_view)

        self.assertIn(".tcp-linear-remote-card {", app_css)
        self.assertIn(".tcp-linear-remote-pad {", app_css)
        self.assertIn('"zPositive xPositive zNegative"', app_css)
        self.assertIn('"yPositive pause yNegative"', app_css)
        self.assertIn('"anglePositive xNegative angleNegative"', app_css)
        self.assertIn(".tcp-linear-remote-stop-btn {", app_css)
        self.assertIn(".tcp-linear-remote-btn.is-active {", app_css)
        self.assertIn(".tcp-linear-remote-position-grid {", app_css)


if __name__ == "__main__":
    unittest.main()
