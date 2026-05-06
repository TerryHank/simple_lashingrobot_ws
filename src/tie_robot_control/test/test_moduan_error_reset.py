#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
CONTROL_DIR = WORKSPACE_ROOT / "tie_robot_control"


class ModuanErrorResetTest(unittest.TestCase):
    def test_manual_warning_reset_clears_software_global_error(self):
        header = (
            CONTROL_DIR / "include" / "tie_robot_control" / "moduan" / "error_handling.hpp"
        ).read_text(encoding="utf-8")
        error_handling = (
            CONTROL_DIR / "src" / "moduan" / "error_handling.cpp"
        ).read_text(encoding="utf-8")
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("void clear_system_error();", header)
        self.assertIn("void clear_system_error()", error_handling)
        self.assertIn("error_detected.store(false", error_handling)
        self.assertIn("last_error_msg.clear();", error_handling)

        hand_start = callbacks.index("void handSolveWarnCallback(")
        hand_end = callbacks.index("\nvoid read_module_motor_state", hand_start)
        hand_body = callbacks[hand_start:hand_end]
        warning_reset_low_index = hand_body.index("PLC_Order_Write(WARNING_RESET, 0")
        clear_index = hand_body.index("clear_system_error();")
        publish_clear_index = hand_body.index("error_flag.data = 0.0;", clear_index)
        self.assertLess(warning_reset_low_index, clear_index)
        self.assertLess(clear_index, publish_clear_index)


if __name__ == "__main__":
    unittest.main()
