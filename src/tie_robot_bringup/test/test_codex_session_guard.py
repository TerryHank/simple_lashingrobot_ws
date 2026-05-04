#!/usr/bin/env python3

import importlib.util
import json
import os
import tempfile
import time
import unittest
from unittest import mock
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "codex_session_guard.py"
INSTALLER_PATH = REPO_ROOT / "scripts" / "install_codex_session_guard_timer.sh"
SUMMARY_INSTALLER_PATH = REPO_ROOT / "scripts" / "install_codex_session_summary_timer.sh"


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
        self.assertIn("ALLOW_CODEX_SESSION_ARCHIVE_TIMER", installer_text)
        self.assertIn("--threshold-mb $THRESHOLD_MB", installer_text)
        self.assertIn("--min-age-minutes $MIN_AGE_MINUTES", installer_text)
        self.assertIn("--skip-open", installer_text)
        self.assertIn("--apply", installer_text)

    def test_summarize_archives_original_and_replaces_active_session_with_summary_jsonl(self):
        large = self.write_session(
            "rollout-large.jsonl",
            "\n".join(
                [
                    '{"type":"session_meta","payload":{"id":"unit-test","timestamp":"2026-04-30T00:00:00Z"}}',
                    '{"timestamp":"2026-04-30T00:01:00Z","payload":{"type":"message","role":"user","content":[{"type":"input_text","text":"帮我修复 PR-FPRG 报告"}]}}',
                    '{"timestamp":"2026-04-30T00:02:00Z","payload":{"type":"message","role":"assistant","content":[{"type":"output_text","text":"我会检查报告生成脚本并补峰值图。"}]}}',
                    '{"timestamp":"2026-04-30T00:03:00Z","payload":{"type":"function_call","name":"exec_command","arguments":"{\\"cmd\\": \\"python3 scripts/report.py\\"}"}}',
                    "x" * 4096,
                ]
            )
            + "\n",
        )

        summaries = self.guard.summarize_oversized_sessions(
            self.codex_home,
            threshold_bytes=32,
            apply=True,
        )

        summary_path = (
            self.codex_home
            / "session_summaries"
            / "oversized"
            / "2026"
            / "04"
            / "22"
            / "rollout-large.summary.md"
        )
        archived_path = (
            self.codex_home
            / "archived_sessions"
            / "oversized"
            / "2026"
            / "04"
            / "22"
            / "rollout-large.jsonl"
        )
        manifest = self.codex_home / "archived_sessions" / "oversized" / "INDEX.md"

        self.assertEqual(len(summaries), 1)
        self.assertTrue(archived_path.exists())
        self.assertTrue(large.exists(), "summarize must leave a compact JSONL at the active path")
        self.assertTrue(summary_path.exists())
        self.assertTrue(manifest.exists())

        archived_text = archived_path.read_text(encoding="utf-8")
        active_text = large.read_text(encoding="utf-8")
        summary_text = summary_path.read_text(encoding="utf-8")

        self.assertIn("x" * 4096, archived_text)
        self.assertNotIn("x" * 4096, active_text)
        self.assertLess(large.stat().st_size, archived_path.stat().st_size)
        self.assertIn('"summary_replacement": true', active_text)
        self.assertIn("Original full JSONL content copied to archive at", active_text)
        self.assertIn("archived_sessions/oversized/2026/04/22/rollout-large.jsonl", active_text)
        self.assertIn("帮我修复 PR-FPRG 报告", active_text)
        self.assertIn("我会检查报告生成脚本并补峰值图。", active_text)
        self.assertIn("unit-test", summary_text)
        self.assertIn("帮我修复 PR-FPRG 报告", summary_text)
        self.assertIn("我会检查报告生成脚本并补峰值图。", summary_text)
        self.assertIn("Original full JSONL content copied to archive at", summary_text)

    def test_summarize_preserves_original_title_anchor_before_summary_message(self):
        large = self.write_session(
            "title-anchor.jsonl",
            "\n".join(
                [
                    '{"type":"session_meta","payload":{"id":"title-test","timestamp":"2026-04-30T00:00:00Z","cwd":"/repo"}}',
                    '{"timestamp":"2026-04-30T00:00:01Z","type":"response_item","payload":{"type":"message","role":"user","content":[{"type":"input_text","text":"# AGENTS.md instructions for /repo\\n\\n<environment_context>...</environment_context>"}]}}',
                    '{"timestamp":"2026-04-30T00:01:00Z","type":"response_item","payload":{"type":"message","role":"user","content":[{"type":"input_text","text":"# Context from my IDE setup:\\n\\n## My request for Codex:\\n保留这个原始标题"}]}}',
                    '{"timestamp":"2026-04-30T00:02:00Z","type":"response_item","payload":{"type":"message","role":"assistant","content":[{"type":"output_text","text":"我会处理。"}]}}',
                    "x" * 4096,
                ]
            )
            + "\n",
        )

        self.guard.summarize_oversized_sessions(
            self.codex_home,
            threshold_bytes=32,
            apply=True,
        )

        records = [json.loads(line) for line in large.read_text(encoding="utf-8").splitlines()]
        self.assertTrue(records[0]["payload"]["summary_replacement"])
        self.assertEqual(records[0]["payload"]["cwd"], "/repo")
        self.assertEqual(records[1]["payload"]["role"], "user")
        self.assertIn("保留这个原始标题", str(records[1]["payload"]["content"]))
        self.assertNotIn("AGENTS.md instructions", str(records[1]["payload"]["content"]))
        self.assertEqual(records[2]["payload"]["role"], "assistant")
        self.assertIn("This active Codex session was compacted", str(records[2]["payload"]["content"]))

    def test_summarize_copies_original_content_without_moving_session_file(self):
        large = self.write_session(
            "copy-not-move.jsonl",
            '{"type":"session_meta","payload":{"id":"copy-test"}}\n' + ("x" * 4096),
        )

        with mock.patch.object(
            self.guard.shutil,
            "move",
            side_effect=AssertionError("summarize must copy original content, not move the session file"),
        ):
            summaries = self.guard.summarize_oversized_sessions(
                self.codex_home,
                threshold_bytes=32,
                apply=True,
            )

        archived_path = (
            self.codex_home
            / "archived_sessions"
            / "oversized"
            / "2026"
            / "04"
            / "22"
            / "copy-not-move.jsonl"
        )

        self.assertEqual(len(summaries), 1)
        self.assertTrue(large.exists())
        self.assertTrue(archived_path.exists())
        self.assertIn("x" * 4096, archived_path.read_text(encoding="utf-8"))
        self.assertIn('"summary_replacement": true', large.read_text(encoding="utf-8"))

    def test_summarize_can_skip_open_sessions(self):
        active = self.write_session(
            "active-large.jsonl",
            '{"type":"session_meta","payload":{"id":"open-session"}}\n' + ("x" * 4096),
        )

        with active.open("r", encoding="utf-8"):
            summaries = self.guard.summarize_oversized_sessions(
                self.codex_home,
                threshold_bytes=32,
                skip_open=True,
                apply=True,
            )

        self.assertEqual(summaries, [])
        self.assertTrue(active.exists())
        self.assertIn("x" * 4096, active.read_text(encoding="utf-8"))
        self.assertFalse((self.codex_home / "archived_sessions" / "oversized" / "2026").exists())

    def test_repair_summary_replacements_restores_title_anchor_from_archive(self):
        active = self.write_session(
            "needs-repair.jsonl",
            "\n".join(
                [
                    '{"type":"session_meta","payload":{"id":"repair-test","timestamp":"2026-04-30T00:00:00Z","summary_replacement":true,"original_size_bytes":4096,"archived_original":"archived_sessions/oversized/2026/04/22/needs-repair.jsonl"}}',
                    '{"timestamp":"2026-04-30T00:01:00Z","payload":{"type":"message","role":"assistant","content":[{"type":"output_text","text":"old summary only"}]}}',
                ]
            )
            + "\n",
        )
        archived = (
            self.codex_home
            / "archived_sessions"
            / "oversized"
            / "2026"
            / "04"
            / "22"
            / "needs-repair.jsonl"
        )
        archived.parent.mkdir(parents=True)
        archived.write_text(
            "\n".join(
                [
                    '{"type":"session_meta","payload":{"id":"repair-test","timestamp":"2026-04-30T00:00:00Z","cwd":"/repo"}}',
                    '{"timestamp":"2026-04-30T00:00:01Z","type":"response_item","payload":{"type":"message","role":"user","content":[{"type":"input_text","text":"# AGENTS.md instructions for /repo\\n\\n<environment_context>...</environment_context>"}]}}',
                    '{"timestamp":"2026-04-30T00:01:00Z","type":"response_item","payload":{"type":"message","role":"user","content":[{"type":"input_text","text":"## My request for Codex:\\n恢复这个旧标题"}]}}',
                    "x" * 4096,
                ]
            )
            + "\n",
            encoding="utf-8",
        )

        repaired = self.guard.repair_summary_replacements(self.codex_home, apply=True)

        records = [json.loads(line) for line in active.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(len(repaired), 1)
        self.assertTrue(records[0]["payload"]["summary_replacement"])
        self.assertEqual(records[0]["payload"]["cwd"], "/repo")
        self.assertEqual(records[1]["payload"]["role"], "user")
        self.assertIn("恢复这个旧标题", str(records[1]["payload"]["content"]))
        self.assertEqual(records[2]["payload"]["role"], "assistant")

    def test_summary_timer_installer_compacts_active_sessions(self):
        installer_text = SUMMARY_INSTALLER_PATH.read_text(encoding="utf-8")

        self.assertIn("tie-codex-session-summary.timer", installer_text)
        self.assertIn("codex_session_guard.py summarize", installer_text)
        self.assertIn("--threshold-mb $THRESHOLD_MB", installer_text)
        self.assertIn("--min-age-minutes $MIN_AGE_MINUTES", installer_text)
        self.assertIn("--skip-open", installer_text)
        self.assertIn("--apply", installer_text)
        self.assertIn("copy original content", installer_text.lower())
        self.assertIn("summary JSONL", installer_text)


if __name__ == "__main__":
    unittest.main()
