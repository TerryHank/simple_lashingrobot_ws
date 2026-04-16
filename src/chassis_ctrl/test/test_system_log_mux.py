#!/usr/bin/env python3

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import system_log_mux  # noqa: E402


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
CHASSIS_CTRL_DIR = WORKSPACE_ROOT / "chassis_ctrl"


class SystemLogMuxTest(unittest.TestCase):
    def test_system_log_mux_script_exists_and_api_launch_starts_it(self):
        script_path = CHASSIS_CTRL_DIR / "scripts" / "system_log_mux.py"
        self.assertTrue(script_path.exists(), "system_log_mux.py should exist")

        launch_text = (CHASSIS_CTRL_DIR / "launch" / "api.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('name="system_log_mux"', launch_text)
        self.assertIn('type="system_log_mux.py"', launch_text)

    def test_system_log_mux_declares_per_node_and_total_topics(self):
        script_text = (CHASSIS_CTRL_DIR / "scripts" / "system_log_mux.py").read_text(
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


if __name__ == "__main__":
    unittest.main()
