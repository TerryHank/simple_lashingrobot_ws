#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SYSTEMD_DIR = WORKSPACE_ROOT / "src" / "tie_robot_bringup" / "systemd"
TOOLS_DIR = WORKSPACE_ROOT / "src" / "tie_robot_bringup" / "tools"


def read_template(name):
    return (SYSTEMD_DIR / name).read_text(encoding="utf-8")


class SystemdRosMasterOwnershipTest(unittest.TestCase):
    def test_driver_services_wait_for_rosbridge_owned_master(self):
        for service_name in (
            "tie-robot-driver-suoqu.service.in",
            "tie-robot-driver-moduan.service.in",
            "tie-robot-driver-camera.service.in",
        ):
            with self.subTest(service=service_name):
                template = read_template(service_name)

                self.assertIn("Wants=network-online.target tie-robot-rosbridge.service", template)
                self.assertIn("After=network-online.target tie-robot-rosbridge.service", template)
                self.assertIn("Environment=ROS_MASTER_URI=http://127.0.0.1:11311", template)
                self.assertIn("wait_for_ros_master.py", template)
                self.assertIn("--timeout-sec 30", template)

    def test_backend_service_waits_for_rosbridge_owned_master(self):
        template = read_template("tie-robot-backend.service.in")

        self.assertIn("Wants=network-online.target tie-robot-rosbridge.service", template)
        self.assertIn("After=network-online.target tie-robot-rosbridge.service", template)
        self.assertIn("Environment=ROS_MASTER_URI=http://127.0.0.1:11311", template)
        self.assertIn("wait_for_ros_master.py", template)
        self.assertIn("--timeout-sec 30", template)

    def test_rosbridge_service_is_the_only_master_owner(self):
        template = read_template("tie-robot-rosbridge.service.in")

        self.assertIn("Environment=ROS_MASTER_URI=http://127.0.0.1:11311", template)
        self.assertNotIn("wait_for_ros_master.py", template)

    def test_wait_for_ros_master_tool_exists(self):
        tool = TOOLS_DIR / "wait_for_ros_master.py"

        self.assertTrue(tool.exists())
        text = tool.read_text(encoding="utf-8")
        self.assertIn("xmlrpc.client.ServerProxy", text)
        self.assertIn("getSystemState", text)


if __name__ == "__main__":
    unittest.main()
