#!/usr/bin/env python3

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path


sys.dont_write_bytecode = True

WORKSPACE_SRC = Path(__file__).resolve().parents[2]
TIE_ROBOT_BRINGUP_DIR = WORKSPACE_SRC / "tie_robot_bringup"
TIE_ROBOT_WEB_DIR = WORKSPACE_SRC / "tie_robot_web"
SCRIPT_DIR = TIE_ROBOT_WEB_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import system_log_mux  # noqa: E402


class SystemLogMuxTest(unittest.TestCase):
    def test_system_log_mux_script_exists_and_api_launch_starts_it(self):
        script_path = TIE_ROBOT_WEB_DIR / "scripts" / "system_log_mux.py"
        self.assertTrue(script_path.exists(), "system_log_mux.py should exist")

        launch_text = (TIE_ROBOT_BRINGUP_DIR / "launch" / "api.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('name="system_log_mux"', launch_text)
        self.assertIn('type="system_log_mux.py"', launch_text)

    def test_system_log_mux_declares_per_node_and_total_topics(self):
        script_text = (TIE_ROBOT_WEB_DIR / "scripts" / "system_log_mux.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("/system_log/all", script_text)
        self.assertIn('"/system_log/"', script_text)
        self.assertIn("/rosout_agg", script_text)
        self.assertIn("[stdout]", script_text)

    def test_sanitize_node_name_and_topic_generation(self):
        self.assertEqual(system_log_mux.sanitize_node_name("/suoquNode"), "suoquNode")
        self.assertEqual(
            system_log_mux.sanitize_node_name("/foo/ns/moduanNode"),
            "moduanNode",
        )
        self.assertEqual(system_log_mux.sanitize_node_name(""), "unknown")
        self.assertEqual(
            system_log_mux.make_node_topic("/foo/ns/moduanNode"),
            "/system_log/moduanNode",
        )

    def test_discover_latest_stdout_logs_prefers_latest_file_per_node(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            older_dir = root / "old"
            newer_dir = root / "new"
            older_dir.mkdir()
            newer_dir.mkdir()

            older_file = older_dir / "suoquNode-1-stdout.log"
            newer_file = newer_dir / "suoquNode-2-stdout.log"
            other_file = newer_dir / "moduanNode-3-stdout.log"

            older_file.write_text("older\n", encoding="utf-8")
            time.sleep(0.01)
            newer_file.write_text("newer\n", encoding="utf-8")
            other_file.write_text("other\n", encoding="utf-8")

            discovered = system_log_mux.discover_latest_stdout_logs(root)

            self.assertEqual(discovered["suoquNode"], newer_file)
            self.assertEqual(discovered["moduanNode"], other_file)

    def test_read_recent_lines_returns_tail_lines_without_empty_rows(self):
        with tempfile.NamedTemporaryFile("w+", suffix=".log", encoding="utf-8") as handle:
            handle.write("line1\n\nline2\nline3\n")
            handle.flush()

            lines, position, partial = system_log_mux.read_recent_lines(
                Path(handle.name), tail_bytes=4096
            )

            self.assertEqual(lines, ["line1", "line2", "line3"])
            self.assertEqual(position, os.path.getsize(handle.name))
            self.assertEqual(partial, "")

    def test_read_appended_lines_tracks_partial_line_between_polls(self):
        with tempfile.NamedTemporaryFile("w+", suffix=".log", encoding="utf-8") as handle:
            handle.write("prefix")
            handle.flush()

            first_lines, first_position, first_partial = system_log_mux.read_appended_lines(
                Path(handle.name), 0, ""
            )
            self.assertEqual(first_lines, [])
            self.assertEqual(first_partial, "prefix")

            handle.write(" line\nnext\n")
            handle.flush()

            second_lines, _, second_partial = system_log_mux.read_appended_lines(
                Path(handle.name), first_position, first_partial
            )
            self.assertEqual(second_lines, ["prefix line", "next"])
            self.assertEqual(second_partial, "")

    def test_extract_recent_failure_reason_prefers_latest_error_or_disconnect_line(self):
        lines = [
            "\x1b[0m[INFO] [1.0]: TCP连接成功。\x1b[0m",
            "[WARN] [2.0]: PLC未连接",
            "Error in XmlRpcClient::writeRequest: write error (拒绝连接).",
        ]

        reason = system_log_mux.extract_recent_failure_reason(lines)

        self.assertEqual(
            reason,
            "Error in XmlRpcClient::writeRequest: write error (拒绝连接).",
        )

    def test_build_driver_link_status_reports_missing_publisher_node_and_reason(self):
        monitor = {
            "label": "索驱",
            "node_name": "suoqu_driver_node",
            "status_topic": "/cabin/cabin_data_upload",
        }

        status = system_log_mux.build_driver_link_status(
            monitor,
            topic_publishers={},
            registered_nodes={"/rosbridge_websocket"},
            recent_reason="索驱TCP驱动连接失败，detail=拒绝连接",
        )

        self.assertFalse(status["connected"])
        self.assertEqual(status["level"], system_log_mux.Log.ERROR)
        self.assertIn("索驱断链", status["message"])
        self.assertIn("/cabin/cabin_data_upload 无发布者", status["message"])
        self.assertIn("/suoqu_driver_node 未注册到当前 ROS master", status["message"])
        self.assertIn("最近错误：索驱TCP驱动连接失败，detail=拒绝连接", status["message"])

    def test_build_driver_link_status_reports_recovery_when_topic_and_node_are_registered(self):
        monitor = {
            "label": "末端",
            "node_name": "moduan_driver_node",
            "status_topic": "/moduan/moduan_gesture_data",
        }

        status = system_log_mux.build_driver_link_status(
            monitor,
            topic_publishers={"/moduan/moduan_gesture_data": {"/moduan_driver_node"}},
            registered_nodes={"/moduan_driver_node"},
            recent_reason=None,
        )

        self.assertTrue(status["connected"])
        self.assertEqual(status["level"], system_log_mux.Log.INFO)
        self.assertIn("末端链路恢复", status["message"])


if __name__ == "__main__":
    unittest.main()
