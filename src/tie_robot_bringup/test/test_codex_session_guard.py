#!/usr/bin/env python3

import importlib.util
import os
import tempfile
import time
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "codex_session_guard.py"
INSTALLER_PATH = REPO_ROOT / "scripts" / "install_codex_session_guard_timer.sh"


def load_guard_module():
    spec = importlib.util.spec_from_file_location("codex_session_guard", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CodexSessionGuardTest(unittest.TestCase):
    def setUp(self):
        self.guard = load_guard_module()
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.codex_home = Path(self.temp_dir.name) / ".codex"
        self.sessions_dir = self.codex_home / "sessions" / "2026" / "04" / "22"
        self.sessions_dir.mkdir(parents=True)

    def write_session(self, name, body):
        path = self.sessions_dir / name
        path.write_text(body, encoding="utf-8")
        return path

    def test_scan_finds_only_oversized_active_sessions(self):
        small = self.write_session("small.jsonl", "{}\n")
        large = self.write_session("large.jsonl", "{}\n" + ("x" * 64))
        archived = self.codex_home / "archived_sessions" / "oversized" / "old.jsonl"
        archived.parent.mkdir(parents=True)
        archived.write_text("x" * 128, encoding="utf-8")

        matches = self.guard.find_oversized_sessions(
            self.codex_home,
            threshold_bytes=32,
        )

        self.assertEqual([item.path for item in matches], [large])
        self.assertNotIn(small, [item.path for item in matches])
        self.assertNotIn(archived, [item.path for item in matches])

    def test_scan_can_skip_recent_sessions(self):
        fresh = self.write_session("fresh.jsonl", "{}\n" + ("x" * 64))
        old = self.write_session("old.jsonl", "{}\n" + ("x" * 64))
        old_timestamp = time.time() - 7200
        os.utime(old, (old_timestamp, old_timestamp))

        matches = self.guard.find_oversized_sessions(
            self.codex_home,
            threshold_bytes=32,
            min_age_seconds=3600,
        )

        self.assertEqual([item.path for item in matches], [old])
        self.assertNotIn(fresh, [item.path for item in matches])

    def test_scan_can_skip_open_sessions(self):
        active = self.write_session("active.jsonl", "{}\n" + ("x" * 64))
        with active.open("r", encoding="utf-8"):
            matches = self.guard.find_oversized_sessions(
                self.codex_home,
                threshold_bytes=32,
                skip_open=True,
            )

        self.assertEqual(matches, [])

    def test_archive_moves_oversized_sessions_and_writes_manifest(self):
        large = self.write_session(
            "rollout-large.jsonl",
            '{"type":"session_meta","payload":{"id":"unit-test"}}\n' + ("x" * 64),
        )

        archived = self.guard.archive_oversized_sessions(
            self.codex_home,
            threshold_bytes=32,
            apply=True,
        )

        expected = (
            self.codex_home
            / "archived_sessions"
            / "oversized"
            / "2026"
            / "04"
            / "22"
            / "rollout-large.jsonl"
        )
        manifest = self.codex_home / "archived_sessions" / "oversized" / "INDEX.md"

        self.assertEqual(len(archived), 1)
        self.assertFalse(large.exists())
        self.assertTrue(expected.exists())
        self.assertTrue(manifest.exists())
        manifest_text = manifest.read_text(encoding="utf-8")
        self.assertIn("rollout-large.jsonl", manifest_text)
        self.assertIn("unit-test", manifest_text)

    def test_timer_installer_uses_safe_archive_options(self):
        installer_text = INSTALLER_PATH.read_text(encoding="utf-8")

        self.assertIn("tie-codex-session-guard.timer", installer_text)
        self.assertIn("codex_session_guard.py archive", installer_text)
        self.assertIn("--threshold-mb $THRESHOLD_MB", installer_text)
        self.assertIn("--min-age-minutes $MIN_AGE_MINUTES", installer_text)
        self.assertIn("--skip-open", installer_text)
        self.assertIn("--apply", installer_text)


if __name__ == "__main__":
    unittest.main()
