#!/usr/bin/env python3

import sys
import tempfile
import unittest
from pathlib import Path

import yaml


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import gripper_tf_broadcaster  # noqa: E402


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
CHASSIS_CTRL_DIR = WORKSPACE_ROOT / "chassis_ctrl"


class GripperTFBroadcasterTest(unittest.TestCase):
    def test_gripper_tf_yaml_exists_with_pose_structure(self):
        config_path = CHASSIS_CTRL_DIR / "config" / "gripper_tf.yaml"
        self.assertTrue(config_path.exists(), "gripper_tf.yaml should exist")
        config_text = config_path.read_text(encoding="utf-8")
        self.assertIn("parent_frame: Scepter_depth_frame", config_text)
        self.assertIn("child_frame: gripper_frame", config_text)
        self.assertIn("translation_mm:", config_text)
        self.assertIn("rotation_rpy:", config_text)

    def test_api_launch_no_longer_starts_gripper_tf_broadcaster(self):
        launch_text = (CHASSIS_CTRL_DIR / "launch" / "api.launch").read_text(encoding="utf-8")
        self.assertNotIn('name="gripper_tf_broadcaster"', launch_text)
        self.assertNotIn('type="gripper_tf_broadcaster.py"', launch_text)

    def test_pointai_tf_verify_launch_starts_only_pointai_and_gripper_tf(self):
        launch_text = (
            CHASSIS_CTRL_DIR / "launch" / "pointai_tf_verify.launch"
        ).read_text(encoding="utf-8")
        self.assertNotIn('include file="$(find chassis_ctrl)/launch/api.launch"', launch_text)
        self.assertIn('name="pointAINode"', launch_text)
        self.assertIn('type="pointAI.py"', launch_text)
        self.assertIn('env name="ROSCONSOLE_FORMAT" value="[${severity}] ${message}"', launch_text)
        self.assertIn('name="gripper_tf_broadcaster"', launch_text)
        self.assertIn('type="gripper_tf_broadcaster.py"', launch_text)
        self.assertNotIn('name="stable_point_tf_broadcaster"', launch_text)

    def test_run_launch_sets_pointai_console_format_without_timestamp(self):
        launch_text = (CHASSIS_CTRL_DIR / "launch" / "run.launch").read_text(encoding="utf-8")
        self.assertIn('name="pointAINode"', launch_text)
        self.assertIn('type="pointAI.py"', launch_text)
        self.assertIn('env name="ROSCONSOLE_FORMAT" value="[${severity}] ${message}"', launch_text)
        self.assertIn('name="gripper_tf_broadcaster"', launch_text)
        self.assertIn('type="gripper_tf_broadcaster.py"', launch_text)
        self.assertIn('name="stable_point_tf_broadcaster"', launch_text)
        self.assertIn('type="stable_point_tf_broadcaster.py"', launch_text)

    def test_load_gripper_tf_config_reads_pose_fields(self):
        config_path = CHASSIS_CTRL_DIR / "config" / "gripper_tf.yaml"
        raw_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        config = gripper_tf_broadcaster.load_gripper_tf_config(
            str(config_path),
            allow_identity_config=True,
        )
        self.assertEqual(config["parent_frame"], "Scepter_depth_frame")
        self.assertEqual(config["child_frame"], "gripper_frame")
        self.assertIn("translation_mm", config)
        self.assertIn("translation", config)
        self.assertIn("rotation_rpy", config)
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
            -float(raw_config["translation_mm"]["z"]) / 1000.0,
        )

    def test_load_gripper_tf_config_inverts_user_translation_for_tf_publish(self):
        config = {
            "parent_frame": "Scepter_depth_frame",
            "child_frame": "gripper_frame",
            "translation_mm": {"x": 100.0, "y": 200.0, "z": -300.0},
            "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle, sort_keys=False)
            handle.flush()
            loaded = gripper_tf_broadcaster.load_gripper_tf_config(
                handle.name,
                allow_identity_config=True,
            )

        self.assertEqual(loaded["translation_mm"], {"x": 100.0, "y": 200.0, "z": -300.0})
        self.assertEqual(loaded["translation"], {"x": -0.1, "y": -0.2, "z": 0.3})

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

    def test_load_gripper_tf_config_rejects_identity_placeholder_by_default(self):
        config = {
            "parent_frame": "Scepter_depth_frame",
            "child_frame": "gripper_frame",
            "translation_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
            "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle)
            handle.flush()
            with self.assertRaisesRegex(ValueError, "placeholder pose"):
                gripper_tf_broadcaster.load_gripper_tf_config(handle.name)

    def test_load_gripper_tf_config_rejects_degrees_like_rotation(self):
        config = {
            "parent_frame": "Scepter_depth_frame",
            "child_frame": "gripper_frame",
            "translation_mm": {"x": 100.0, "y": 0.0, "z": 0.0},
            "rotation_rpy": {"roll": 180.0, "pitch": 0.0, "yaw": 0.0},
        }
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle)
            handle.flush()
            with self.assertRaisesRegex(ValueError, "expected radians"):
                gripper_tf_broadcaster.load_gripper_tf_config(handle.name)

    def test_load_gripper_tf_config_rejects_legacy_translation_field(self):
        config = {
            "parent_frame": "Scepter_depth_frame",
            "child_frame": "gripper_frame",
            "translation": {"x": 0.6, "y": 0.6, "z": 0.6},
            "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", encoding="utf-8") as handle:
            yaml.safe_dump(config, handle)
            handle.flush()
            with self.assertRaisesRegex(ValueError, "translation_mm"):
                gripper_tf_broadcaster.load_gripper_tf_config(handle.name)

    def test_old_workspace_business_tf_publishers_are_removed(self):
        file_expectations = {
            CHASSIS_CTRL_DIR / "scripts" / "pointAI.py": [
                "sendTransform(",
                "publish_aruco_frame_transform",
                "publish_tf_transform",
                "publish_gripper_tf_transform",
            ],
            WORKSPACE_ROOT / "fast_image_solve" / "scripts" / "vision.py": [
                "sendTransform(",
                "publish_aruco_frame_transform",
                "publish_tf_transform",
                "publish_gripper_tf_transform",
            ],
            WORKSPACE_ROOT / "fast_image_solve" / "src" / "5.18auto.cpp": [
                "sendTransform(",
                "publishTfTransform",
                "publishGripperTfTransform",
                "pubCamGripperTf",
            ],
            WORKSPACE_ROOT
            / "fast_image_solve"
            / "include"
            / "fast_image_solve"
            / "5.18auto.hpp": [
                "publishTfTransform",
                "publishGripperTfTransform",
                "pubCamGripperTf",
            ],
        }

        for path, forbidden_markers in file_expectations.items():
            content = path.read_text(encoding="utf-8")
            for marker in forbidden_markers:
                self.assertNotIn(marker, content, f"{path} should not contain {marker}")

    def test_stable_point_tf_broadcaster_exists_and_tracks_bind_points(self):
        broadcaster_path = CHASSIS_CTRL_DIR / "scripts" / "stable_point_tf_broadcaster.py"
        self.assertTrue(broadcaster_path.exists(), "stable_point_tf_broadcaster.py should exist")
        content = broadcaster_path.read_text(encoding="utf-8")

        self.assertIn("/coordinate_point", content)
        self.assertIn("/cabin/area_progress", content)
        self.assertIn("bind_point_", content)
        self.assertIn("area_", content)
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
        self.assertEqual(published[0]["header_frame_id"], "cabin_frame")
        self.assertAlmostEqual(published[0]["translation_m"][2], 1.0025)

    def test_stable_point_tracker_archives_active_points_on_area_switch(self):
        from stable_point_tf_broadcaster import StablePointTracker  # noqa: E402

        tracker = StablePointTracker(stable_frame_count=2, stable_z_tolerance_mm=4.0)
        tracker.ingest_area_progress({"current_area_index": 1, "all_done": False})
        tracker.ingest_points([
            {"idx": 1, "World_coord": [100.0, 200.0, 1000.0]},
        ])
        tracker.ingest_points([
            {"idx": 1, "World_coord": [100.0, 200.0, 1003.0]},
        ])

        archived = tracker.ingest_area_progress({"current_area_index": 2, "all_done": False})

        self.assertEqual(len(archived), 1)
        self.assertEqual(archived[0]["child_frame_id"], "area_1_point_1")
        self.assertEqual(tracker.active_area_index, 2)


if __name__ == "__main__":
    unittest.main()
