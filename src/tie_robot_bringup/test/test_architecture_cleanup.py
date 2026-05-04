#!/usr/bin/env python3

import json
import re
import unittest
from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
REPO_ROOT = Path(__file__).resolve().parents[3]

TIE_ROBOT_MSGS_DIR = WORKSPACE_SRC / "tie_robot_msgs"
TIE_ROBOT_DESCRIPTION_DIR = WORKSPACE_SRC / "tie_robot_description"
TIE_ROBOT_HW_DIR = WORKSPACE_SRC / "tie_robot_hw"
TIE_ROBOT_CONTROL_DIR = WORKSPACE_SRC / "tie_robot_control"
TIE_ROBOT_PERCEPTION_DIR = WORKSPACE_SRC / "tie_robot_perception"
TIE_ROBOT_PROCESS_DIR = WORKSPACE_SRC / "tie_robot_process"
TIE_ROBOT_BRINGUP_DIR = WORKSPACE_SRC / "tie_robot_bringup"
TIE_ROBOT_WEB_DIR = WORKSPACE_SRC / "tie_robot_web"

DIRECT_PRINTF_RE = re.compile(r"(?<![A-Za-z0-9_])printf\s*\(")
STDERR_FPRINTF_RE = re.compile(r"fprintf\s*\(\s*stderr\s*,")
BRINGUP_INCLUDE_RE = re.compile(
    r'<include\s+file="\$\(find tie_robot_bringup\)/launch/(?P<name>[^"]+)"'
)


def iter_runtime_source_files():
    roots = [
        TIE_ROBOT_PROCESS_DIR / "src",
        TIE_ROBOT_PROCESS_DIR / "include",
        TIE_ROBOT_CONTROL_DIR / "src",
        TIE_ROBOT_CONTROL_DIR / "include",
        TIE_ROBOT_PERCEPTION_DIR / "scripts",
        TIE_ROBOT_PERCEPTION_DIR / "src" / "tie_robot_perception" / "pointai",
        TIE_ROBOT_WEB_DIR / "src",
        TIE_ROBOT_WEB_DIR / "include",
        TIE_ROBOT_WEB_DIR / "scripts",
    ]
    suffixes = {".cpp", ".hpp", ".h", ".py"}
    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.suffix in suffixes and "json.hpp" not in path.name:
                yield path


def expand_bringup_launch_text(launch_text, seen=None):
    seen = set(seen or [])
    expanded = [launch_text]
    for match in BRINGUP_INCLUDE_RE.finditer(launch_text):
        include_name = match.group("name")
        if include_name in seen:
            continue
        include_path = TIE_ROBOT_BRINGUP_DIR / "launch" / include_name
        if not include_path.exists():
            continue
        seen.add(include_name)
        expanded.append(expand_bringup_launch_text(include_path.read_text(encoding="utf-8"), seen))
    return "\n".join(expanded)


class TieRobotArchitectureCleanupTest(unittest.TestCase):
    def test_target_packages_exist_with_expected_names(self):
        package_dirs = {
            "tie_robot_msgs": TIE_ROBOT_MSGS_DIR,
            "tie_robot_description": TIE_ROBOT_DESCRIPTION_DIR,
            "tie_robot_hw": TIE_ROBOT_HW_DIR,
            "tie_robot_control": TIE_ROBOT_CONTROL_DIR,
            "tie_robot_perception": TIE_ROBOT_PERCEPTION_DIR,
            "tie_robot_process": TIE_ROBOT_PROCESS_DIR,
            "tie_robot_bringup": TIE_ROBOT_BRINGUP_DIR,
            "tie_robot_web": TIE_ROBOT_WEB_DIR,
        }

        for package_name, package_dir in package_dirs.items():
            with self.subTest(package=package_name):
                self.assertTrue((package_dir / "package.xml").exists())
                package_text = (package_dir / "package.xml").read_text(encoding="utf-8")
                self.assertIn(f"<name>{package_name}</name>", package_text)

    def test_tie_robot_msgs_owns_ros_interfaces(self):
        self.assertTrue((TIE_ROBOT_MSGS_DIR / "msg").exists())
        self.assertTrue((TIE_ROBOT_MSGS_DIR / "srv").exists())
        self.assertTrue((TIE_ROBOT_MSGS_DIR / "action").exists())

        msgs_cmake = (TIE_ROBOT_MSGS_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("add_message_files(", msgs_cmake)
        self.assertIn("add_service_files(", msgs_cmake)
        self.assertIn("add_action_files(", msgs_cmake)
        self.assertIn("PointCoords.msg", msgs_cmake)
        self.assertIn("ProcessImage.srv", msgs_cmake)
        self.assertIn("StartGlobalWorkTask.action", msgs_cmake)

        self.assertFalse((TIE_ROBOT_WEB_DIR / "msg").exists())
        self.assertFalse((TIE_ROBOT_WEB_DIR / "srv").exists())
        self.assertFalse((TIE_ROBOT_WEB_DIR / "action").exists())

    def test_bringup_launch_references_new_package_boundaries(self):
        run_launch = (TIE_ROBOT_BRINGUP_DIR / "launch" / "run.launch").read_text(
            encoding="utf-8"
        )
        api_launch = (TIE_ROBOT_BRINGUP_DIR / "launch" / "api.launch").read_text(
            encoding="utf-8"
        )
        driver_stack_launch = (
            TIE_ROBOT_BRINGUP_DIR / "launch" / "driver_stack.launch"
        ).read_text(encoding="utf-8")
        expanded_driver_stack_launch = expand_bringup_launch_text(driver_stack_launch)
        rosbridge_stack_launch = (
            TIE_ROBOT_BRINGUP_DIR / "launch" / "rosbridge_stack.launch"
        ).read_text(encoding="utf-8")
        tf_stack_launch = (
            TIE_ROBOT_BRINGUP_DIR / "launch" / "tf_stack.launch"
        ).read_text(encoding="utf-8")
        algorithm_stack_launch = (
            TIE_ROBOT_BRINGUP_DIR / "launch" / "algorithm_stack.launch"
        ).read_text(encoding="utf-8")

        self.assertNotIn('$(find tie_robot_bringup)/launch/api.launch', run_launch)
        self.assertNotIn('$(find tie_robot_bringup)/launch/driver_stack.launch', run_launch)
        self.assertIn('$(find tie_robot_bringup)/launch/algorithm_stack.launch', run_launch)
        self.assertNotIn('workspace_picker_web_server', run_launch)
        self.assertNotIn('workspace_picker_web_open', run_launch)
        self.assertIn('pkg="tie_robot_process"', expanded_driver_stack_launch)
        self.assertIn('pkg="tie_robot_control"', expanded_driver_stack_launch)
        self.assertNotIn('name="robot_tf_broadcaster"', expanded_driver_stack_launch)
        self.assertNotIn('name="gripper_tf_broadcaster"', expanded_driver_stack_launch)
        self.assertIn('$(find tie_robot_bringup)/launch/tf_stack.launch', rosbridge_stack_launch)
        self.assertIn('$(find tie_robot_bringup)/launch/api.launch', rosbridge_stack_launch)
        self.assertIn('name="tf2_web_republisher"', tf_stack_launch)
        self.assertIn('name="robot_tf_broadcaster"', tf_stack_launch)
        self.assertIn('name="gripper_tf_broadcaster"', tf_stack_launch)
        self.assertIn('$(find tie_robot_perception)/config/gripper_tf.yaml', tf_stack_launch)
        self.assertIn('pkg="tie_robot_perception"', algorithm_stack_launch)

        self.assertIn('$(find tie_robot_perception)/launch/scepter_camera.launch', expanded_driver_stack_launch)
        self.assertIn('pkg="tie_robot_web"', api_launch)

    def test_legacy_app_frontend_directory_has_been_removed(self):
        self.assertFalse(
            (WORKSPACE_SRC / "APP").exists(),
            "旧前端 src/APP 已迁移到 tie_robot_web/frontend，不应再保留打包产物。",
        )

    def test_rviz_configs_use_map_as_fixed_frame(self):
        rviz_dir = TIE_ROBOT_DESCRIPTION_DIR / "rviz"
        rviz_files = sorted(rviz_dir.glob("*.rviz"))
        self.assertGreater(len(rviz_files), 0)
        for rviz_path in rviz_files:
            with self.subTest(rviz=str(rviz_path.relative_to(REPO_ROOT))):
                content = rviz_path.read_text(encoding="utf-8")
                self.assertIn("Fixed Frame: map", content)
                self.assertNotIn("Fixed Frame: odom", content)
                self.assertNotIn("Fixed Frame: " + "cabin" + "_frame", content)

    def test_runtime_packages_own_current_nodes_and_assets(self):
        self.assertTrue((TIE_ROBOT_PERCEPTION_DIR / "scripts" / "pointai_node.py").exists())
        self.assertFalse((TIE_ROBOT_PERCEPTION_DIR / "scripts" / "pointAI.py").exists())
        self.assertTrue(
            (TIE_ROBOT_PERCEPTION_DIR / "scripts" / "gripper_tf_broadcaster.py").exists()
        )
        self.assertTrue((TIE_ROBOT_PERCEPTION_DIR / "scripts" / "robot_tf_broadcaster.py").exists())
        self.assertFalse(
            (TIE_ROBOT_PERCEPTION_DIR / "scripts" / "stable_point_tf_broadcaster.py").exists()
        )
        self.assertTrue((TIE_ROBOT_PERCEPTION_DIR / "src" / "camera" / "scepter_driver.cpp").exists())
        self.assertTrue((TIE_ROBOT_CONTROL_DIR / "src" / "moduanNode.cpp").exists())
        self.assertTrue((TIE_ROBOT_PROCESS_DIR / "src" / "suoquNode.cpp").exists())
        self.assertTrue((TIE_ROBOT_PROCESS_DIR / "data" / "pseudo_slam_bind_path.json").exists())
        self.assertTrue(
            (
                TIE_ROBOT_PROCESS_DIR
                / "include"
                / "tie_robot_process"
                / "planning"
                / "dynamic_bind_planning.hpp"
            ).exists()
        )
        self.assertTrue(
            (
                TIE_ROBOT_PROCESS_DIR
                / "src"
                / "planning"
                / "dynamic_bind_planning.cpp"
            ).exists()
        )
        self.assertTrue(
            (
                TIE_ROBOT_PERCEPTION_DIR
                / "src"
                / "tie_robot_perception"
                / "perception"
                / "workspace_s2.py"
            ).exists()
        )
        self.assertTrue((TIE_ROBOT_DESCRIPTION_DIR / "URDF" / "model.urdf").exists())
        self.assertTrue((TIE_ROBOT_DESCRIPTION_DIR / "rviz" / "chassis_visual.rviz").exists())

    def test_linear_module_execution_chain_uses_driver_atomic_functions(self):
        driver_header = (
            TIE_ROBOT_HW_DIR / "include" / "tie_robot_hw" / "driver" / "linear_module_driver.hpp"
        ).read_text(encoding="utf-8")
        driver_source = (
            TIE_ROBOT_HW_DIR / "src" / "driver" / "linear_module_driver.cpp"
        ).read_text(encoding="utf-8")
        linear_module_executor = (
            TIE_ROBOT_CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("bool writeQueuedPoints(", driver_header)
        self.assertIn("bool pulseExecutionEnable(", driver_header)
        self.assertIn("bool setZeroRequest(", driver_header)
        self.assertNotIn("executeQueuedPoints(", driver_header)
        self.assertNotIn("requestZero(", driver_header)
        self.assertIn("bool LinearModuleDriver::writeQueuedPoints(", driver_source)
        self.assertIn("bool LinearModuleDriver::pulseExecutionEnable(", driver_source)
        self.assertIn("bool LinearModuleDriver::setZeroRequest(", driver_source)
        self.assertNotIn("bool LinearModuleDriver::executeQueuedPoints(", driver_source)
        self.assertNotIn("bool LinearModuleDriver::requestZero(", driver_source)

        pulse_body = driver_source[
            driver_source.index("bool LinearModuleDriver::pulseExecutionEnable("):
            driver_source.index("bool LinearModuleDriver::setZeroRequest(")
        ]
        clear_enable_index = pulse_body.index(
            "transport_->writeSingleRegister(kRegisterEnableDisable, 0, error)"
        )
        set_enable_index = pulse_body.index(
            "transport_->writeSingleRegister(kRegisterEnableDisable, 1, error)"
        )
        self.assertLess(clear_enable_index, set_enable_index)

        self.assertNotIn("executeQueuedPoints(", linear_module_executor)
        self.assertNotIn("requestZero(", linear_module_executor)
        execute_bind_points_body = linear_module_executor[
            linear_module_executor.index("bool execute_bind_points("):
            linear_module_executor.index('response_message = "区域绑扎作业完成";')
        ]
        write_points_index = execute_bind_points_body.index("writeQueuedPoints(")
        pulse_enable_index = execute_bind_points_body.index("pulseExecutionEnable(")
        self.assertLess(write_points_index, pulse_enable_index)

    def test_ros_package_scripts_and_tools_follow_catkin_conventions(self):
        self.assertTrue(
            (
                TIE_ROBOT_PERCEPTION_DIR
                / "tools"
                / "pr_fprg_peak_supported_probe.py"
            ).exists()
        )
        self.assertFalse(
            (
                TIE_ROBOT_PERCEPTION_DIR
                / "scripts"
                / "pr_fprg_peak_supported_probe.py"
            ).exists()
        )
        self.assertTrue((TIE_ROBOT_WEB_DIR / "tools" / "run_gitnexus_local_webui.py").exists())
        self.assertFalse((TIE_ROBOT_WEB_DIR / "scripts" / "run_gitnexus_local_webui.py").exists())

        allowed_script_files = {
            "tie_robot_bringup": {
                "install_backend_service.sh",
                "install_driver_services.sh",
                "install_frontend_autostart.sh",
                "install_rosbridge_service.sh",
            },
            "tie_robot_perception": {
                "gripper_tf_broadcaster.py",
                "pointai_node.py",
                "robot_tf_broadcaster.py",
            },
            "tie_robot_process": {
                "architecture_role_node.py",
                "global_bind_planner_node.py",
            },
            "tie_robot_web": {
                "system_log_mux.py",
                "workspace_picker_web_open.py",
                "workspace_picker_web_server.py",
            },
        }
        for package_name, expected_files in allowed_script_files.items():
            scripts_dir = WORKSPACE_SRC / package_name / "scripts"
            if not scripts_dir.exists():
                continue
            actual_files = {path.name for path in scripts_dir.iterdir() if path.is_file()}
            self.assertEqual(expected_files, actual_files)

    def test_cpp_public_headers_are_namespaced_and_installed(self):
        packages = [
            TIE_ROBOT_CONTROL_DIR,
            TIE_ROBOT_HW_DIR,
            TIE_ROBOT_PERCEPTION_DIR,
            TIE_ROBOT_PROCESS_DIR,
            TIE_ROBOT_WEB_DIR,
        ]

        for package_dir in packages:
            with self.subTest(package=package_dir.name):
                include_dir = package_dir / "include"
                if not include_dir.exists():
                    continue

                loose_headers = sorted(
                    str(path.relative_to(package_dir))
                    for path in include_dir.iterdir()
                    if path.is_file() and path.suffix in {".h", ".hpp"}
                )
                self.assertEqual([], loose_headers)

                namespaced_include = include_dir / package_dir.name
                if not namespaced_include.exists():
                    continue

                cmake = (package_dir / "CMakeLists.txt").read_text(encoding="utf-8")
                self.assertIn("INCLUDE_DIRS include", cmake)
                self.assertIn("install(DIRECTORY include/${PROJECT_NAME}/", cmake)

        control_cmake = (TIE_ROBOT_CONTROL_DIR / "CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        self.assertIn("install(TARGETS", control_cmake)
        self.assertIn("moduanNode", control_cmake)
        self.assertIn("moduan_driver_node", control_cmake)
        self.assertIn("moduan_motion_controller_node", control_cmake)

    def test_runtime_data_json_files_are_not_single_scalar_parameter_files(self):
        runtime_data_dirs = [
            TIE_ROBOT_PERCEPTION_DIR / "data",
            TIE_ROBOT_PROCESS_DIR / "data",
            TIE_ROBOT_CONTROL_DIR / "data",
        ]
        violations = []

        for data_dir in runtime_data_dirs:
            if not data_dir.exists():
                continue
            for path in sorted(data_dir.glob("*.json")):
                payload = json.loads(path.read_text(encoding="utf-8"))
                if not isinstance(payload, dict) or len(payload) != 1:
                    continue
                only_value = next(iter(payload.values()))
                if not isinstance(only_value, (dict, list)):
                    violations.append(str(path.relative_to(REPO_ROOT)))

        self.assertEqual(
            violations,
            [],
            "单个标量参数不应落成独立 JSON；请放到 launch/rosparam 或参数 YAML 中。",
        )

    def test_hw_package_owns_transport_and_driver_libraries(self):
        self.assertTrue(
            (TIE_ROBOT_HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_driver.hpp").exists()
        )
        self.assertTrue((TIE_ROBOT_HW_DIR / "src" / "driver" / "cabin_driver.cpp").exists())
        self.assertTrue(
            (TIE_ROBOT_HW_DIR / "src" / "driver" / "linear_module_driver.cpp").exists()
        )

        hw_cmake = (TIE_ROBOT_HW_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("add_library(tie_robot_hw_driver_core", hw_cmake)
        self.assertNotIn("scepter_camera", hw_cmake)

    def test_moduan_telemetry_reads_battery_and_temperature_with_scaled_float_registers(self):
        numeric_codec_header = (
            TIE_ROBOT_CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "numeric_codec.hpp"
        ).read_text(encoding="utf-8")
        numeric_codec_impl = (
            TIE_ROBOT_CONTROL_DIR / "src" / "moduan" / "numeric_codec.cpp"
        ).read_text(encoding="utf-8")
        moduan_callbacks = (
            TIE_ROBOT_CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("float Read_Module_Float(int axis, modbus_t* ctx);", numeric_codec_header)
        self.assertIn(
            "float Read_Module_Float_Ranged(int axis, modbus_t* ctx, float preferred_min, float preferred_max);",
            numeric_codec_header,
        )
        self.assertIn(
            "float Read_Module_Float_RangedScaled(int axis, modbus_t* ctx, float preferred_min, float preferred_max, float scale);",
            numeric_codec_header,
        )
        self.assertIn("float Read_Module_Float(int axis, modbus_t* ctx)", numeric_codec_impl)
        self.assertIn("float Read_Module_Float_Ranged(int axis, modbus_t* ctx, float preferred_min, float preferred_max)", numeric_codec_impl)
        self.assertIn("float Read_Module_Float_RangedScaled(int axis, modbus_t* ctx, float preferred_min, float preferred_max, float scale)", numeric_codec_impl)
        self.assertIn("float choose_preferred_scaled_float(", numeric_codec_impl)
        self.assertIn("const float scaled_low_word_first = low_word_first * scale;", numeric_codec_impl)
        self.assertIn("const float scaled_high_word_first = high_word_first * scale;", numeric_codec_impl)
        self.assertIn("const float preferred_candidates[] = {", numeric_codec_impl)
        self.assertIn("scaled_low_word_first,", numeric_codec_impl)
        self.assertIn("scaled_high_word_first,", numeric_codec_impl)
        self.assertIn("return choose_preferred_scaled_float(", numeric_codec_impl)
        self.assertIn(
            "robot_battery_voltage = Read_Module_Float_RangedScaled(BATTERY_VOLTAGE, plc, 0.0f, 100.0f, 0.01f);",
            moduan_callbacks,
        )
        self.assertIn(
            "robot_temperature = Read_Module_Float_RangedScaled(INNER_TEM, plc, -40.0f, 120.0f, 0.01f);",
            moduan_callbacks,
        )
        self.assertIn("modbus_t* ensure_active_ctx(modbus_t* ctx)", numeric_codec_impl)
        self.assertIn("void invalidate_ctx(modbus_t* ctx)", numeric_codec_impl)
        self.assertIn("if (ctx == plc) {\n        plc = nullptr;\n    }", numeric_codec_impl)
        self.assertIn("plc = PLC_Connection();", numeric_codec_impl)
        self.assertIn("ros_log_printf(\"Moduan_log:PLC连接已自动重建。\\n\");", numeric_codec_impl)
        self.assertNotIn("Read_Module_Speed(BATTERY_VOLTAGE", moduan_callbacks)
        self.assertNotIn("Read_Module_Speed(INNER_TEM", moduan_callbacks)

    def test_runtime_logging_uses_ros_log_api(self):
        violations = []
        for path in iter_runtime_source_files():
            text = path.read_text(encoding="utf-8")
            if path.name == "common.hpp":
                text = text.replace("ros_log_printf(", "")
            for line_number, line in enumerate(text.splitlines(), start=1):
                stripped = line.strip()
                if stripped.startswith("//") or stripped.startswith("#"):
                    continue
                if DIRECT_PRINTF_RE.search(line) and "ros_log_printf(" not in line:
                    violations.append(f"{path.relative_to(WORKSPACE_SRC)}:{line_number}: {stripped}")
                if STDERR_FPRINTF_RE.search(line) or "std::cout" in line or "std::cerr" in line:
                    violations.append(f"{path.relative_to(WORKSPACE_SRC)}:{line_number}: {stripped}")
                if path.suffix == ".py" and stripped.startswith("print("):
                    violations.append(f"{path.relative_to(WORKSPACE_SRC)}:{line_number}: {stripped}")
        self.assertEqual([], violations)

    def test_driver_nodes_publish_standard_diagnostics_and_support_independent_start_stop(self):
        moduan_cmake = (TIE_ROBOT_CONTROL_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        moduan_package = (TIE_ROBOT_CONTROL_DIR / "package.xml").read_text(encoding="utf-8")
        moduan_callbacks = (
            TIE_ROBOT_CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        moduan_executor = (
            TIE_ROBOT_CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")
        suoqu_cmake = (TIE_ROBOT_PROCESS_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        suoqu_package = (TIE_ROBOT_PROCESS_DIR / "package.xml").read_text(encoding="utf-8")
        suoqu_node = (TIE_ROBOT_PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        suoqu_services = (
            TIE_ROBOT_PROCESS_DIR / "src" / "suoqu" / "service_orchestration.cpp"
        ).read_text(encoding="utf-8")
        suoqu_transport = (
            TIE_ROBOT_PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")
        perception_cmake = (TIE_ROBOT_PERCEPTION_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        perception_package = (TIE_ROBOT_PERCEPTION_DIR / "package.xml").read_text(encoding="utf-8")
        pointai_node = (
            TIE_ROBOT_PERCEPTION_DIR / "src" / "tie_robot_perception" / "pointai" / "diagnostics.py"
        ).read_text(encoding="utf-8")
        web_status_catalog = (
            TIE_ROBOT_WEB_DIR / "frontend" / "src" / "config" / "statusMonitorCatalog.js"
        ).read_text(encoding="utf-8")
        web_status_controller = (
            TIE_ROBOT_WEB_DIR / "frontend" / "src" / "controllers" / "StatusMonitorController.js"
        ).read_text(encoding="utf-8")
        web_system_control = (
            TIE_ROBOT_WEB_DIR / "frontend" / "src" / "config" / "systemControlCatalog.js"
        ).read_text(encoding="utf-8")
        web_ros_connection = (
            TIE_ROBOT_WEB_DIR / "frontend" / "src" / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        web_ui = (
            TIE_ROBOT_WEB_DIR / "frontend" / "src" / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        process_control = (
            TIE_ROBOT_WEB_DIR / "src" / "web_bridge" / "process_control.cpp"
        ).read_text(encoding="utf-8")
        node_app = (
            TIE_ROBOT_WEB_DIR / "src" / "web_bridge" / "node_app.cpp"
        ).read_text(encoding="utf-8")
        web_server = (
            TIE_ROBOT_WEB_DIR / "scripts" / "workspace_picker_web_server.py"
        ).read_text(encoding="utf-8")

        self.assertIn("diagnostic_updater", moduan_cmake)
        self.assertIn("<build_depend>diagnostic_updater</build_depend>", moduan_package)
        self.assertIn('"/moduan/driver/start"', moduan_callbacks)
        self.assertIn('"/moduan/driver/stop"', moduan_callbacks)
        self.assertIn('"/moduan/driver/restart"', moduan_callbacks)
        self.assertIn('setHardwareID(kModuanDiagnosticHardwareId);', moduan_callbacks)
        self.assertIn('stat.hardware_id = kModuanDiagnosticHardwareId;', moduan_callbacks)
        self.assertIn('g_moduan_diagnostic_updater->add("末端驱动", produce_moduan_diagnostics);', moduan_callbacks)
        self.assertIn('if (!g_moduan_driver_enabled.load()) {', moduan_executor)
        self.assertIn('error->message = "末端驱动已关闭";', moduan_executor)

        self.assertIn("diagnostic_updater", suoqu_cmake)
        self.assertIn("<build_depend>diagnostic_updater</build_depend>", suoqu_package)
        self.assertIn('"/cabin/driver/start"', suoqu_node)
        self.assertIn('"/cabin/driver/stop"', suoqu_node)
        self.assertIn('"/cabin/driver/restart"', suoqu_node)
        self.assertIn('setHardwareID(kCabinDiagnosticHardwareId);', suoqu_node)
        self.assertIn('stat.hardware_id = kCabinDiagnosticHardwareId;', suoqu_node)
        self.assertIn('g_cabin_diagnostic_updater->add("索驱驱动", produce_cabin_driver_diagnostics);', suoqu_node)
        self.assertIn("索驱驱动已关闭。", suoqu_services)
        self.assertIn("索驱驱动已关闭，拒绝下发运动指令", suoqu_transport)

        self.assertIn("diagnostic_updater", perception_cmake)
        self.assertIn("<build_depend>diagnostic_updater</build_depend>", perception_package)
        self.assertIn('VISUAL_DIAGNOSTIC_HARDWARE_ID = "tie_robot/visual_algorithm"', pointai_node)
        self.assertIn('self.visual_diagnostic_updater.add("视觉算法", self.produce_visual_algorithm_diagnostics)', pointai_node)
        self.assertIn('stat.hardware_id = VISUAL_DIAGNOSTIC_HARDWARE_ID', pointai_node)
        self.assertIn('stat.summary(DiagnosticStatus.OK, "视觉算法运行中")', pointai_node)

        self.assertIn('diagnosticHardwareId: "tie_robot/chassis_driver"', web_status_catalog)
        self.assertIn('diagnosticHardwareId: "tie_robot/moduan_driver"', web_status_catalog)
        self.assertIn('diagnosticHardwareId: "tie_robot/visual_algorithm"', web_status_catalog)
        self.assertIn("name: TOPICS.process.diagnostics", web_status_controller)
        self.assertIn("messageType: MESSAGE_TYPES.diagnosticArray", web_status_controller)
        self.assertIn('this.callbacks.onStatusChip?.("visual", "warn", "视觉状态未上报");', web_status_controller)
        self.assertIn('chassis: level === "success" ? "stopCabinSubsystem" : "restartCabinSubsystem"', web_ui)
        self.assertIn('moduan: level === "success" ? "stopModuanSubsystem" : "startModuanSubsystem"', web_ui)
        self.assertIn('visual: level === "success" ? "stopVisualSubsystem" : "startVisualSubsystem"', web_ui)
        self.assertIn('id: "stopAlgorithmStack"', web_system_control)
        self.assertIn('id: "startCabinSubsystem"', web_system_control)
        self.assertIn('id: "restartCabinSubsystem"', web_system_control)
        self.assertIn('id: "stopCabinSubsystem"', web_system_control)
        self.assertIn('id: "restartAlgorithmStack"', web_system_control)
        self.assertIn("restartAlgorithmStackService", web_ros_connection)
        self.assertIn('kRestartAlgorithmStackScript', process_control)
        self.assertIn('"/web/system/restart_algorithm_stack"', node_app)
        self.assertIn('"/api/system/stop_algorithm_stack"', web_server)
        self.assertIn('"/api/system/restart_algorithm_stack"', web_server)

    def test_legacy_packages_and_frontend_dirs_are_removed(self):
        self.assertFalse((WORKSPACE_SRC / "APP").exists())
        self.assertFalse((WORKSPACE_SRC / "chassis_ctrl").exists())
        self.assertFalse((WORKSPACE_SRC / "robot_interface_hub").exists())
        self.assertFalse((WORKSPACE_SRC / "robot_hw_drivers").exists())
        self.assertFalse((WORKSPACE_SRC / "robot_algorithm_layer").exists())
        self.assertFalse((WORKSPACE_SRC / "ir_workspace_picker_web").exists())
        self.assertFalse((WORKSPACE_SRC / "ScepterROS").exists())

    def test_new_web_package_owns_frontend_entry(self):
        web_index = (TIE_ROBOT_WEB_DIR / "web" / "index.html").read_text(
            encoding="utf-8"
        )
        self.assertIn('<div id="app"></div>', web_index)
        self.assertIn("./assets/app/", web_index)


if __name__ == "__main__":
    unittest.main()
