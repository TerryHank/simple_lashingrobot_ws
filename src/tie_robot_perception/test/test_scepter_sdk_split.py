#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
PACKAGE_DIR = WORKSPACE_SRC / "tie_robot_perception"


class ScepterSdkSplitTest(unittest.TestCase):
    def test_scepter_static_tf_aliases_are_children_of_depth_frame(self):
        content = (PACKAGE_DIR / "src" / "camera" / "intrinsics.cpp").read_text(
            encoding="utf-8"
        )

        for child_frame in (
            "camera_frame",
            "color_frame",
            "points_frame",
            "depth2colorpoints_frame",
            "alignedcolor_frame",
            "aligneddepth_frame",
        ):
            with self.subTest(child_frame=child_frame):
                self.assertIn(
                    f"publish_static_identity_tf(tf_broadcaster, now, depth_frame, {child_frame});",
                    content,
                )

        self.assertNotIn("msg.child_frame_id = depth_frame;", content)

    def test_camera_manager_no_longer_contains_world_coord_or_convert_service(self):
        content = (PACKAGE_DIR / "src" / "camera" / "scepter_manager.cpp").read_text(
            encoding="utf-8"
        )
        for marker in (
            "worldCoord_pub_",
            "raw_worldCoord_pub_",
            "convertDepthToPointCloud(",
            "convert_depth_to_point_cloud",
            "SACSegmentation",
            "ExtractIndices",
            "pcl::",
        ):
            self.assertNotIn(marker, content)

    def test_camera_header_no_longer_exposes_algorithm_members(self):
        content = (
            PACKAGE_DIR / "include" / "tie_robot_perception" / "camera" / "scepter_manager.hpp"
        ).read_text(encoding="utf-8")
        for marker in (
            "ConvertDepthToPointCloud",
            "worldCoord_nh_",
            "worldCoord_pub_",
            "raw_worldCoord_pub_",
            "convertDepthToPointCloud(",
        ):
            self.assertNotIn(marker, content)

    def test_launch_starts_world_coord_processor(self):
        content = (PACKAGE_DIR / "launch" / "scepter_camera.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('<arg name="camera_respawn" default="false"', content)
        self.assertIn('type="scepter_world_coord_processor"', content)
        self.assertIn('name="scepter_world_coord_processor"', content)
        self.assertIn('respawn="$(arg camera_respawn)"', content)

    def test_camera_driver_exits_after_consecutive_frame_failures_for_guard_restart(self):
        content = (PACKAGE_DIR / "src" / "camera" / "device_session.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("kMaxConsecutiveFrameReadyFailures", content)
        self.assertIn("++missed_frames", content)
        self.assertIn("missed_frames = 0;", content)
        self.assertIn("相机连续取帧失败", content)
        self.assertIn("ros::shutdown();", content)

    def test_cmake_splits_driver_and_algorithm_targets(self):
        content = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("add_executable(scepter_world_coord_processor", content)
        camera_block = re.search(
            r"target_link_libraries\(scepter_camera(?P<body>.*?)\n\)",
            content,
            re.S,
        )
        self.assertIsNotNone(camera_block)
        self.assertNotIn("${PCL_LIBRARIES}", camera_block.group("body"))
        processor_block = re.search(
            r"target_link_libraries\(scepter_world_coord_processor(?P<body>.*?)\n\)",
            content,
            re.S,
        )
        self.assertIsNotNone(processor_block)
        self.assertIn("${PCL_LIBRARIES}", processor_block.group("body"))


if __name__ == "__main__":
    unittest.main()
