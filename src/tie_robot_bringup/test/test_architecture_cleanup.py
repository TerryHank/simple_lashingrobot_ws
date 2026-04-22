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

        self.assertIn('$(find tie_robot_bringup)/launch/api.launch', run_launch)
        self.assertIn('pkg="tie_robot_process"', run_launch)
        self.assertIn('pkg="tie_robot_control"', run_launch)
        self.assertIn('pkg="tie_robot_perception"', run_launch)
        self.assertIn('pkg="tie_robot_web"', run_launch)
        self.assertIn('$(find tie_robot_perception)/config/gripper_tf.yaml', run_launch)

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
        self.assertIn("const float scaled_low_word_first = low_word_first * scale;", numeric_codec_impl)
        self.assertIn("const float scaled_high_word_first = high_word_first * scale;", numeric_codec_impl)
        self.assertIn(
            "robot_battery_voltage = Read_Module_Float_RangedScaled(BATTERY_VOLTAGE, plc, 0.0f, 100.0f, 0.01f);",
            moduan_callbacks,
        )
        self.assertIn(
            "robot_temperature = Read_Module_Float_RangedScaled(INNER_TEM, plc, -40.0f, 120.0f, 0.01f);",
            moduan_callbacks,
        )
        self.assertNotIn("Read_Module_Speed(BATTERY_VOLTAGE", moduan_callbacks)
        self.assertNotIn("Read_Module_Speed(INNER_TEM", moduan_callbacks)

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
