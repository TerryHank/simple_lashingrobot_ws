#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = Path(__file__).resolve().parents[3]
PROCESS_DIR = WORKSPACE_ROOT / "tie_robot_process"
PERCEPTION_DIR = WORKSPACE_ROOT / "tie_robot_perception"


class TfCoordinateContractTest(unittest.TestCase):
    def test_legacy_world_frame_name_is_not_left_in_repository_text(self):
        legacy_frame = "cabin" + "_frame"
        ignored_dirs = {
            ".git",
            "build",
            "devel",
            "install",
            "logs",
            "node_modules",
            "__pycache__",
        }
        violations = []

        for path in sorted(REPO_ROOT.rglob("*")):
            if not path.is_file():
                continue
            if any(part in ignored_dirs for part in path.parts):
                continue
            if any(part.startswith(".") for part in path.relative_to(REPO_ROOT).parts):
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            if legacy_frame in text:
                violations.append(str(path.relative_to(REPO_ROOT)))

        self.assertEqual(
            violations,
            [],
            "旧世界坐标系名不应残留；索驱世界坐标系统一命名为 map。",
        )

    def test_pointai_algorithm_layer_has_no_tf_listener_or_projection_helpers(self):
        pointai_entry = (PERCEPTION_DIR / "scripts" / "pointai_node.py").read_text(encoding="utf-8")
        pointai_package = PERCEPTION_DIR / "src" / "tie_robot_perception" / "pointai"
        pointai_ros_interfaces = (pointai_package / "ros_interfaces.py").read_text(
            encoding="utf-8"
        )
        pointai_runtime_text = "\n".join(
            path.read_text(encoding="utf-8")
            for path in sorted(pointai_package.glob("*.py"))
        )
        pointai_projection_runtime_text = "\n".join(
            path.read_text(encoding="utf-8")
            for path in sorted(pointai_package.glob("*.py"))
            if path.name != "bind_point_tf.py"
        )

        self.assertIn("tf2_ros.TransformBroadcaster()", pointai_ros_interfaces)
        self.assertNotIn("tf2_ros.TransformListener", pointai_entry)
        self.assertNotIn("tf2_ros.Buffer", pointai_entry)
        self.assertNotIn("tf_listener", pointai_entry)
        self.assertNotIn("tf_buffer", pointai_entry)
        self.assertNotIn("T_camera_ee", pointai_entry)
        self.assertNotIn("spatial_calibration_source", pointai_entry)
        self.assertNotIn("gripper_tf", pointai_entry)
        self.assertNotIn("/web/pointAI/set_offset", pointai_entry)
        self.assertNotIn("downstream_transforms", pointai_entry)
        self.assertNotIn("tf_only", pointai_entry)
        self.assertNotIn("apply_spatial_calibration", pointai_runtime_text)
        self.assertNotIn("to_cabin_world_coord", pointai_runtime_text)
        self.assertNotIn("transform_point_to_frame", pointai_runtime_text)
        self.assertNotIn("update_gripper_tf_translation_mm", pointai_runtime_text)
        self.assertNotIn("calibration_offset_callback", pointai_runtime_text)
        self.assertNotIn("gripper_tf_config_file", pointai_runtime_text)
        self.assertNotIn("parent_frame", pointai_projection_runtime_text)
        self.assertNotIn("child_frame", pointai_projection_runtime_text)

    def test_process_layer_no_longer_publishes_pseudo_slam_point_tf_frames(self):
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertNotIn("pseudo_slam_point_", suoqu_node)
        self.assertNotIn("publish_pseudo_slam_point_transforms();", suoqu_node)
        self.assertIn(
            "pseudo_slam点TF停发",
            suoqu_node,
        )

    def test_process_planning_uses_tf_chain_not_manual_camera_or_gripper_math(self):
        dynamic_geometry = (
            PROCESS_DIR / "src" / "planning" / "dynamic_bind_geometry.cpp"
        ).read_text(encoding="utf-8")
        scan_processing = (
            PROCESS_DIR / "src" / "suoqu" / "pseudo_slam_scan_processing.cpp"
        ).read_text(encoding="utf-8")
        suoqu_node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        combined = "\n".join([dynamic_geometry, scan_processing, suoqu_node])
        for legacy_formula in (
            "world_point.World_coord[0] - cabin_point.x",
            "world_point.World_coord[1] - cabin_point.y",
            "cabin_height - world_point.World_coord[2]",
            "current_cabin_state.Z - world_point.World_coord[2]",
            "gripper_from_scepter.getOrigin().z()",
            "point_in_scepter_frame",
        ):
            self.assertNotIn(legacy_formula, combined)

        self.assertIn("lookup_gripper_from_base_link_transform", suoqu_node)
        self.assertIn("lookup_current_gripper_from_cabin_transform", suoqu_node)
        self.assertIn("build_gripper_from_cabin_transform", dynamic_geometry)
        self.assertIn("gripper_from_base_link", dynamic_geometry)


if __name__ == "__main__":
    unittest.main()
