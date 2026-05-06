#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
FRONTEND_SRC_DIR = WORKSPACE_SRC / "tie_robot_web" / "frontend" / "src"


class TcpLinearRemoteRawMoveTest(unittest.TestCase):
    def test_tcp_linear_remote_calls_driver_raw_single_move_service(self):
        topic_registry = (FRONTEND_SRC_DIR / "config" / "topicRegistry.js").read_text(encoding="utf-8")
        ros_connection = (
            FRONTEND_SRC_DIR / "controllers" / "RosConnectionController.js"
        ).read_text(encoding="utf-8")
        tcp_remote_controller = (
            FRONTEND_SRC_DIR / "controllers" / "TcpLinearRemoteController.js"
        ).read_text(encoding="utf-8")

        self.assertIn('singleMove: "/moduan/driver/raw_single_move"', topic_registry)
        self.assertNotIn('singleMove: "/moduan/single_move"', topic_registry)
        self.assertIn("name: SERVICES.moduan.singleMove", ros_connection)
        self.assertIn("SERVICE_TYPES.moduan.linearModuleMove", ros_connection)
        self.assertIn("callLinearModuleSingleMoveService(target)", tcp_remote_controller)


if __name__ == "__main__":
    unittest.main()
