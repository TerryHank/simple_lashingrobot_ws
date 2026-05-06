#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROCESS_DIR = WORKSPACE_ROOT / "tie_robot_process"


class TcpTravelRangeConfigTest(unittest.TestCase):
    def test_dynamic_planning_and_suoqu_runtime_use_160mm_z_travel(self):
        planning_header = (
            PROCESS_DIR / "include" / "tie_robot_process" / "planning" / "dynamic_bind_planning.hpp"
        ).read_text(encoding="utf-8")
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")

        self.assertIn("float tcp_max_z_mm = 160.0f;", planning_header)
        self.assertIn("constexpr float kTravelMaxZMm = 160.0f;", runtime_header)

    def test_precomputed_bind_group_loader_does_not_apply_duplicate_runtime_range_gate(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        scan_processing = (
            PROCESS_DIR / "src" / "suoqu" / "pseudo_slam_scan_processing.cpp"
        ).read_text(encoding="utf-8")

        loader_start = node.index("bool load_precomputed_local_points_from_group_json(")
        loader_end = node.index("\nnlohmann::json collect_dispatched_precomputed_point_jsons", loader_start)
        loader_body = node[loader_start:loader_end]

        self.assertNotIn("is_local_bind_point_in_range", loader_body)
        self.assertNotIn("out_of_range_local_coord_count", loader_body)
        self.assertNotIn("超出虎口范围", loader_body)
        self.assertNotIn("当前组JSON局部点全部超出虎口范围", loader_body)
        self.assertNotIn("bool is_local_bind_point_in_range(const tie_robot_msgs::PointCoords& point);", runtime_header)
        self.assertNotIn("bool is_local_bind_point_in_range(const tie_robot_msgs::PointCoords& point)", scan_processing)


if __name__ == "__main__":
    unittest.main()
