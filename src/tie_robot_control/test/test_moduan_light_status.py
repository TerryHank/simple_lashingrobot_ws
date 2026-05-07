#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
CONTROL_DIR = WORKSPACE_ROOT / "tie_robot_control"
MSG_DIR = WORKSPACE_ROOT / "tie_robot_msgs" / "msg"


class ModuanLightStatusTest(unittest.TestCase):
    def test_linear_module_status_message_carries_light_state(self):
        msg_text = (MSG_DIR / "linear_module_upload.msg").read_text(encoding="utf-8")
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("bool light_state", msg_text)

        pub_start = callbacks.index("void pub_moduan_state(")
        pub_end = callbacks.index("\nvoid publish_moduan_state_topic", pub_start)
        pub_body = callbacks[pub_start:pub_end]
        self.assertIn("linear_module_data_upload_msg.light_state = light_state;", pub_body)

        read_start = callbacks.index("void read_module_motor_state(")
        read_end = callbacks.index("\nstd::string getBeijingTimeString", read_start)
        read_body = callbacks[read_start:read_end]
        self.assertIn("Read_Module_Status(LIGHT, plc)", read_body)
        self.assertIn("light_state =", read_body)

        switch_start = callbacks.index("void light_switch(")
        switch_end = callbacks.index("\nvoid change_speed_callback", switch_start)
        switch_body = callbacks[switch_start:switch_end]
        self.assertIn("light_state = false;", switch_body)
        self.assertIn("light_state = true;", switch_body)


if __name__ == "__main__":
    unittest.main()
