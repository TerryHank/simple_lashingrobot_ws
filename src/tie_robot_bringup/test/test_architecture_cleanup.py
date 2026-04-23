#!/usr/bin/env python3

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
        algorithm_stack_launch = (
            TIE_ROBOT_BRINGUP_DIR / "launch" / "algorithm_stack.launch"
        ).read_text(encoding="utf-8")

        self.assertIn('$(find tie_robot_bringup)/launch/api.launch', run_launch)
        self.assertIn('$(find tie_robot_bringup)/launch/driver_stack.launch', run_launch)
        self.assertIn('$(find tie_robot_bringup)/launch/algorithm_stack.launch', run_launch)
        self.assertIn('pkg="tie_robot_web"', run_launch)
        self.assertIn('pkg="tie_robot_process"', driver_stack_launch)
        self.assertIn('pkg="tie_robot_control"', driver_stack_launch)
        self.assertIn('pkg="tie_robot_perception"', driver_stack_launch)
        self.assertIn('$(find tie_robot_perception)/config/gripper_tf.yaml', driver_stack_launch)
        self.assertIn('pkg="tie_robot_perception"', algorithm_stack_launch)

        self.assertIn('$(find tie_robot_perception)/launch/scepter_camera.launch', api_launch)
        self.assertIn('pkg="tie_robot_web"', api_launch)

    def test_runtime_packages_own_current_nodes_and_assets(self):
        self.assertTrue((TIE_ROBOT_PERCEPTION_DIR / "scripts" / "pointAI.py").exists())
        self.assertTrue(
            (TIE_ROBOT_PERCEPTION_DIR / "scripts" / "gripper_tf_broadcaster.py").exists()
        )
        self.assertTrue(
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
        self.assertIn("printf(\"Moduan_log:PLC连接已自动重建。\\n\");", numeric_codec_impl)
        self.assertNotIn("Read_Module_Speed(BATTERY_VOLTAGE", moduan_callbacks)
        self.assertNotIn("Read_Module_Speed(INNER_TEM", moduan_callbacks)

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
            TIE_ROBOT_PERCEPTION_DIR / "scripts" / "pointAI.py"
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
        self.assertIn('name: "/diagnostics"', web_status_controller)
        self.assertIn('this.callbacks.onStatusChip?.("visual", "warn", "视觉状态未上报");', web_status_controller)
        self.assertIn('chassis: level === "success" ? "restartCabinDriver" : "startCabinDriver"', web_ui)
        self.assertIn('moduan: level === "success" ? "restartModuanDriver" : "startModuanDriver"', web_ui)
        self.assertIn('visual: level === "success" ? "restartAlgorithmStack" : "startAlgorithmStack"', web_ui)
        self.assertIn('id: "restartAlgorithmStack"', web_system_control)
        self.assertIn("restartAlgorithmStackService", web_ros_connection)
        self.assertIn('kRestartAlgorithmStackScript', process_control)
        self.assertIn('"/web/system/restart_algorithm_stack"', node_app)
        self.assertIn('"/api/system/restart_algorithm_stack"', web_server)

    def test_legacy_packages_and_frontend_dirs_are_removed(self):
        self.assertFalse((WORKSPACE_SRC / "chassis_ctrl").exists())
        self.assertFalse((WORKSPACE_SRC / "robot_interface_hub").exists())
        self.assertFalse((WORKSPACE_SRC / "robot_hw_drivers").exists())
        self.assertFalse((WORKSPACE_SRC / "robot_algorithm_layer").exists())
        self.assertFalse((WORKSPACE_SRC / "ir_workspace_picker_web").exists())
        self.assertFalse((WORKSPACE_SRC / "ScepterROS").exists())

    def test_app_entry_points_to_new_web_package(self):
        app_index = (REPO_ROOT / "src" / "APP" / "dist" / "index.html").read_text(
            encoding="utf-8"
        )
        self.assertIn("../../tie_robot_web/web/index.html", app_index)


if __name__ == "__main__":
    unittest.main()
