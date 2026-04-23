#!/usr/bin/env python3

import sys
import tempfile
import unittest
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = Path(__file__).resolve().parents[2]
TIE_ROBOT_PERCEPTION_DIR = WORKSPACE_SRC / "tie_robot_perception"
TIE_ROBOT_BRINGUP_DIR = WORKSPACE_SRC / "tie_robot_bringup"
SCRIPT_DIR = PACKAGE_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import gripper_tf_broadcaster  # noqa: E402


class GripperTFBroadcasterTest(unittest.TestCase):
    def test_gripper_tf_yaml_exists_with_pose_structure(self):
        config_path = TIE_ROBOT_PERCEPTION_DIR / "config" / "gripper_tf.yaml"
        self.assertTrue(config_path.exists(), "gripper_tf.yaml should exist")
        config_text = config_path.read_text(encoding="utf-8")
        self.assertIn("parent_frame: Scepter_depth_frame", config_text)
        self.assertIn("child_frame: gripper_frame", config_text)
        self.assertIn("translation_mm:", config_text)
        self.assertIn("rotation_rpy:", config_text)

    def test_api_launch_does_not_start_gripper_tf_broadcaster(self):
        launch_text = (TIE_ROBOT_BRINGUP_DIR / "launch" / "api.launch").read_text(
            encoding="utf-8"
        )
        self.assertNotIn('name="gripper_tf_broadcaster"', launch_text)
        self.assertNotIn('type="gripper_tf_broadcaster.py"', launch_text)

    def test_pointai_tf_verify_launch_starts_only_pointai_and_gripper_tf(self):
        launch_text = (
            TIE_ROBOT_BRINGUP_DIR / "launch" / "pointai_tf_verify.launch"
        ).read_text(encoding="utf-8")
        self.assertNotIn('<include file="$(find tie_robot_bringup)/launch/api.launch"', launch_text)
        self.assertIn('name="pointAINode"', launch_text)
        self.assertIn('type="pointAI.py"', launch_text)
        self.assertIn('name="gripper_tf_broadcaster"', launch_text)
        self.assertIn('type="gripper_tf_broadcaster.py"', launch_text)
        self.assertNotIn('name="stable_point_tf_broadcaster"', launch_text)

    def test_run_launch_starts_pointai_and_tf_broadcasters(self):
        launch_text = (TIE_ROBOT_BRINGUP_DIR / "launch" / "run.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            '<include file="$(find tie_robot_bringup)/launch/driver_stack.launch" />',
            launch_text,
        )
        self.assertIn(
            '<include file="$(find tie_robot_bringup)/launch/algorithm_stack.launch" />',
            launch_text,
        )

    def test_load_gripper_tf_config_reads_pose_fields(self):
        config_path = TIE_ROBOT_PERCEPTION_DIR / "config" / "gripper_tf.yaml"
        raw_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        self.assertGreater(
            float(raw_config["translation_mm"]["z"]),
            0.0,
            "用户标定值应使用“TCP 在相机 z+ 方向”为正的录入口径",
        )
        config = gripper_tf_broadcaster.load_gripper_tf_config(
            str(config_path),
            allow_identity_config=True,
        )
        self.assertEqual(config["parent_frame"], "Scepter_depth_frame")
        self.assertEqual(config["child_frame"], "gripper_frame")
        self.assertEqual(config["translation_mm"], raw_config["translation_mm"])
        self.assertAlmostEqual(
            config["translation"]["x"],
            -float(raw_config["translation_mm"]["x"]) / 1000.0,
        )
        self.assertAlmostEqual(
            config["translation"]["y"],
            -float(raw_config["translation_mm"]["y"]) / 1000.0,
        )
        self.assertAlmostEqual(
            config["translation"]["z"],
            float(raw_config["translation_mm"]["z"]) / 1000.0,
        )

    def test_with_updated_translation_mm_refreshes_runtime_and_publish_values(self):
        config = {
            "parent_frame": "Scepter_depth_frame",
            "child_frame": "gripper_frame",
            "translation_mm": {"x": 100.0, "y": 200.0, "z": 300.0},
            "translation": {"x": -0.1, "y": -0.2, "z": -0.3},
            "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }

        updated = gripper_tf_broadcaster.with_updated_translation_mm(
            config,
            275.0,
            84.0,
            735.0,
        )

        self.assertEqual(
            updated["translation_mm"],
            {"x": 275.0, "y": 84.0, "z": 735.0},
        )
        self.assertEqual(
            updated["translation"],
            {"x": -0.275, "y": -0.084, "z": 0.735},
        )

    def test_save_gripper_tf_config_persists_updated_translation_mm(self):
        config = {
            "parent_frame": "Scepter_depth_frame",
            "child_frame": "gripper_frame",
            "translation_mm": {"x": 275.0, "y": 84.0, "z": 735.0},
            "translation": {"x": -0.275, "y": -0.084, "z": -0.735},
            "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }

        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "gripper_tf.yaml"
            gripper_tf_broadcaster.save_gripper_tf_config(str(config_path), config)
            reloaded = yaml.safe_load(config_path.read_text(encoding="utf-8"))

        self.assertEqual(reloaded["parent_frame"], "Scepter_depth_frame")
        self.assertEqual(reloaded["child_frame"], "gripper_frame")
        self.assertEqual(
            reloaded["translation_mm"],
            {"x": 275.0, "y": 84.0, "z": 735.0},
        )
        self.assertNotIn("translation", reloaded)

    def test_broadcaster_subscribes_to_live_offset_topic(self):
        script_text = (
            TIE_ROBOT_PERCEPTION_DIR / "scripts" / "gripper_tf_broadcaster.py"
        ).read_text(encoding="utf-8")

        self.assertIn('POINTAI_OFFSET_TOPIC = "/web/pointAI/set_offset"', script_text)
        self.assertIn('SET_GRIPPER_TF_CALIBRATION_SERVICE = "/web/pointAI/set_gripper_tf_calibration"', script_text)
        self.assertIn("rospy.Subscriber(POINTAI_OFFSET_TOPIC, Pose", script_text)
        self.assertIn("rospy.Service(", script_text)
        self.assertIn("SetGripperTfCalibration", script_text)
        self.assertIn("无需重启节点", script_text)

    def test_pointai_cabin_z_formula_uses_direct_gripper_tf_z_sign(self):
        tf_transform_text = (
            TIE_ROBOT_PERCEPTION_DIR
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "tf_transform.py"
        ).read_text(encoding="utf-8")

        self.assertIn("cabin_z = int(round(float(T[2, 3]) - float(z) + float(T_gripper[2, 3])))", tf_transform_text)
        self.assertNotIn("cabin_z = int(round(float(T[2, 3]) - float(z) - float(T_gripper[2, 3])))", tf_transform_text)

    def test_build_transform_uses_parent_child_and_translation(self):
        config = {
            "parent_frame": "Scepter_depth_frame",
            "child_frame": "gripper_frame",
            "translation_mm": {"x": 100.0, "y": 200.0, "z": 300.0},
            "translation": {"x": 0.1, "y": 0.2, "z": 0.3},
            "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }
        transform = gripper_tf_broadcaster.build_transform(config)
        self.assertEqual(transform.header.frame_id, "Scepter_depth_frame")
        self.assertEqual(transform.child_frame_id, "gripper_frame")
        self.assertAlmostEqual(transform.transform.translation.x, 0.1)
        self.assertAlmostEqual(transform.transform.translation.y, 0.2)
        self.assertAlmostEqual(transform.transform.translation.z, 0.3)
        self.assertAlmostEqual(transform.transform.rotation.w, 1.0)

    def test_pointai_no_longer_contains_legacy_workspace_tf_publishers(self):
        pointai_text = (TIE_ROBOT_PERCEPTION_DIR / "scripts" / "pointAI.py").read_text(
            encoding="utf-8"
        )
        for marker in (
            "sendTransform(",
            "publish_aruco_frame_transform",
            "publish_tf_transform",
            "publish_gripper_tf_transform",
        ):
            self.assertNotIn(marker, pointai_text)

    def test_stable_point_tf_broadcaster_exists_and_tracks_bind_points(self):
        broadcaster_path = (
            TIE_ROBOT_PERCEPTION_DIR / "scripts" / "stable_point_tf_broadcaster.py"
        )
        self.assertTrue(broadcaster_path.exists(), "stable_point_tf_broadcaster.py should exist")
        content = broadcaster_path.read_text(encoding="utf-8")

        self.assertIn("/coordinate_point", content)
        self.assertIn("/cabin/area_progress", content)
        self.assertIn("bind_point_", content)
        self.assertIn("stable_frame_count", content)
        self.assertIn("stable_z_tolerance_mm", content)
        self.assertIn("cabin_frame", content)

    def test_stable_point_tracker_promotes_point_after_two_close_z_frames(self):
        from stable_point_tf_broadcaster import StablePointTracker  # noqa: E402

        tracker = StablePointTracker(stable_frame_count=2, stable_z_tolerance_mm=4.0)

        first_frame = [
            {"idx": 1, "World_coord": [100.0, 200.0, 1000.0]},
        ]
        second_frame = [
            {"idx": 1, "World_coord": [101.0, 201.0, 1002.5]},
        ]

        self.assertEqual(tracker.ingest_points(first_frame), [])
        published = tracker.ingest_points(second_frame)

        self.assertEqual(len(published), 1)
        self.assertEqual(published[0]["child_frame_id"], "bind_point_1")
        self.assertEqual(
            published[0]["translation_m"],
            [0.101, 0.201, 1.0025],
        )


if __name__ == "__main__":
    unittest.main()
