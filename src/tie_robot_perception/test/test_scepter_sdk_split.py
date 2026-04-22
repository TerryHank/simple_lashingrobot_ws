#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
PACKAGE_DIR = WORKSPACE_SRC / "tie_robot_perception"


class ScepterSdkSplitTest(unittest.TestCase):
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
        self.assertIn('type="scepter_world_coord_processor"', content)
        self.assertIn('name="scepter_world_coord_processor"', content)

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
