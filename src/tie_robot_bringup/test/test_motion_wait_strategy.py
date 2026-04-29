#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
CONTROL_ROOT = WORKSPACE_ROOT / "src" / "tie_robot_control"
PROCESS_ROOT = WORKSPACE_ROOT / "src" / "tie_robot_process"


def read(path):
    return path.read_text(encoding="utf-8")


class MotionWaitStrategyTest(unittest.TestCase):
    def test_legacy_delay_time_api_is_removed(self):
        checked_files = [
            CONTROL_ROOT / "include" / "tie_robot_control" / "moduan" / "linear_module_executor.hpp",
            CONTROL_ROOT / "src" / "moduan" / "linear_module_executor.cpp",
            PROCESS_ROOT / "src" / "suoquNode.cpp",
        ]
        forbidden_patterns = [
            re.compile(r"\bdelay_time\b"),
            re.compile(r"\bdelay_time_for_execution\b"),
            re.compile(r"\bDELAY_TIME_"),
            re.compile(r"\btraget_coordinate\b"),
        ]

        for path in checked_files:
            text = read(path)
            with self.subTest(path=str(path.relative_to(WORKSPACE_ROOT))):
                for pattern in forbidden_patterns:
                    self.assertIsNone(pattern.search(text), pattern.pattern)

    def test_motion_waits_are_named_as_arrival_confirmation(self):
        control_header = read(
            CONTROL_ROOT
            / "include"
            / "tie_robot_control"
            / "moduan"
            / "linear_module_executor.hpp"
        )
        control_cpp = read(CONTROL_ROOT / "src" / "moduan" / "linear_module_executor.cpp")
        process_cpp = read(PROCESS_ROOT / "src" / "suoquNode.cpp")

        self.assertIn("wait_linear_module_axis_arrival", control_header)
        self.assertIn("wait_linear_module_axis_arrival", control_cpp)
        self.assertIn("kLinearModuleAxisArrivalTimeoutSec", control_cpp)
        self.assertIn("kMotionWaitTimeoutSec", process_cpp)
        self.assertIn("wait_cabin_axis_arrival", process_cpp)
        self.assertIn("wait_cabin_axis_stable_arrival", process_cpp)


if __name__ == "__main__":
    unittest.main()
