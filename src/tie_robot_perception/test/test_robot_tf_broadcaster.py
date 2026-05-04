#!/usr/bin/env python3

import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = Path(__file__).resolve().parents[2]
TIE_ROBOT_PERCEPTION_DIR = WORKSPACE_SRC / "tie_robot_perception"
TIE_ROBOT_BRINGUP_DIR = WORKSPACE_SRC / "tie_robot_bringup"
SCRIPT_DIR = PACKAGE_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import robot_tf_broadcaster  # noqa: E402


def make_depth_image(values, width=3, height=2):
    flat_values = [int(value) for row in values for value in row]
    return SimpleNamespace(
        height=height,
        width=width,
        encoding="16UC1",
        is_bigendian=False,
        step=width * 2,
        data=struct.pack("<" + ("H" * len(flat_values)), *flat_values),
    )


class RobotTFBroadcasterTest(unittest.TestCase):
    def test_robot_home_tf_yaml_exists_with_home_and_base_camera_structure(self):
        config_path = TIE_ROBOT_PERCEPTION_DIR / "config" / "robot_home_tf.yaml"
        srv_path = WORKSPACE_SRC / "tie_robot_msgs" / "srv" / "RobotHomeCalibration.srv"
        self.assertTrue(config_path.exists(), "robot_home_tf.yaml should exist")
        config_text = config_path.read_text(encoding="utf-8")
        request_text = srv_path.read_text(encoding="utf-8").split("---", 1)[0]
        self.assertIn("home_cabin_mm:", config_text)
        self.assertIn("base_to_camera_mm:", config_text)
        self.assertIn("base_to_camera_rpy:", config_text)
        self.assertIn("cabin_to_map_sign:", config_text)
        self.assertIn("z: 460.0", config_text)
        self.assertIn("x: 1.0", config_text)
        self.assertNotIn("base_to_camera", request_text)

    def test_tf_stack_passes_robot_home_config_to_robot_tf_broadcaster(self):
        launch_text = (TIE_ROBOT_BRINGUP_DIR / "launch" / "tf_stack.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('name="robot_tf_broadcaster"', launch_text)
        self.assertIn("robot_home_tf.yaml", launch_text)
        self.assertIn("/Scepter/depth/image_raw", launch_text)

    def test_load_and_save_robot_home_config_round_trip(self):
        config = {
            "home_cabin_mm": {"x": 100.0, "y": 200.0, "z": 523.0},
            "base_to_camera_mm": {"x": 12.0, "y": -34.0, "z": 56.0},
            "base_to_camera_rpy": {"roll": 3.141592653589793, "pitch": 0.0, "yaw": 0.0},
            "cabin_to_map_sign": {"x": 1.0, "y": 1.0, "z": 1.0},
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "robot_home_tf.yaml"
            robot_tf_broadcaster.save_robot_home_config(str(config_path), config)
            reloaded = robot_tf_broadcaster.load_robot_home_config(str(config_path))

        self.assertEqual(reloaded["home_cabin_mm"], config["home_cabin_mm"])
        self.assertEqual(reloaded["base_to_camera_mm"], config["base_to_camera_mm"])
        self.assertEqual(reloaded["base_to_camera_rpy"], config["base_to_camera_rpy"])
        self.assertEqual(reloaded["cabin_to_map_sign"], config["cabin_to_map_sign"])

    def test_build_transforms_keeps_cabin_x_same_sign_in_map(self):
        config = {
            "home_cabin_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
            "base_to_camera_mm": {"x": 10.0, "y": 20.0, "z": 30.0},
            "base_to_camera_rpy": {"roll": 3.141592653589793, "pitch": 0.0, "yaw": 0.0},
            "cabin_to_map_sign": {"x": 1.0, "y": 1.0, "z": 1.0},
        }
        base_transform, scepter_transform = robot_tf_broadcaster.build_transforms(
            "map",
            "base_link",
            "Scepter_depth_frame",
            {"x": 1000.0, "y": -2000.0, "z": 523.0},
            config,
            stamp=None,
        )

        self.assertEqual(base_transform.header.frame_id, "map")
        self.assertEqual(base_transform.child_frame_id, "base_link")
        self.assertAlmostEqual(base_transform.transform.translation.x, 1.0)
        self.assertAlmostEqual(base_transform.transform.translation.y, -2.0)
        self.assertAlmostEqual(base_transform.transform.translation.z, 0.523)
        self.assertEqual(scepter_transform.header.frame_id, "base_link")
        self.assertEqual(scepter_transform.child_frame_id, "Scepter_depth_frame")
        self.assertAlmostEqual(scepter_transform.transform.translation.x, 0.01)
        self.assertAlmostEqual(scepter_transform.transform.translation.y, 0.02)
        self.assertAlmostEqual(scepter_transform.transform.translation.z, 0.03)

    def test_depth_ground_probe_uses_rolling_median_of_frame_max_depth(self):
        probe = robot_tf_broadcaster.DepthGroundProbe(window_size=3, max_valid_depth_mm=5000.0)

        self.assertEqual(
            probe.update(make_depth_image([[0, 1200, 65535], [800, 900, 1000]])),
            1200.0,
        )
        self.assertEqual(
            probe.update(make_depth_image([[0, 1600, 0], [700, 1300, 1400]])),
            1400.0,
        )
        self.assertEqual(
            probe.update(make_depth_image([[0, 2000, 0], [700, 1300, 1800]])),
            1600.0,
        )

    def test_camera_and_ground_probe_points_are_projected_through_configured_tf(self):
        config = {
            "home_cabin_mm": {"x": 1000.0, "y": 2000.0, "z": 523.0},
            "base_to_camera_mm": {"x": 100.0, "y": 20.0, "z": 300.0},
            "base_to_camera_rpy": {"roll": 3.141592653589793, "pitch": 0.0, "yaw": 0.0},
            "cabin_to_map_sign": {"x": 1.0, "y": 1.0, "z": 1.0},
        }
        current_pose = {"x": 1500.0, "y": 2500.0, "z": 700.0}

        camera_map = robot_tf_broadcaster.transform_camera_point_to_map_mm(
            current_pose,
            config,
            {"x": 0.0, "y": 0.0, "z": 0.0},
        )
        ground_probe = robot_tf_broadcaster.transform_camera_point_to_map_mm(
            current_pose,
            config,
            {"x": 0.0, "y": 0.0, "z": 894.0},
        )

        self.assertEqual(camera_map, {"x": 1600.0, "y": 2520.0, "z": 1000.0})
        self.assertAlmostEqual(ground_probe["x"], 1600.0)
        self.assertAlmostEqual(ground_probe["y"], 2520.0)
        self.assertAlmostEqual(ground_probe["z"], 106.0)


if __name__ == "__main__":
    unittest.main()
