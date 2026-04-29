#!/usr/bin/env python3

import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
AGENTS_MD = REPO_ROOT / "AGENTS.md"
MEMORY_DIR = REPO_ROOT / "docs" / "agent_memory"
MEMORY_README = MEMORY_DIR / "README.md"
MEMORY_CURRENT = MEMORY_DIR / "current.md"
MEMORY_LOG = MEMORY_DIR / "session_log.md"
MEMORY_TEMPLATE = MEMORY_DIR / "session_template.md"
MEMORY_ORGANISM = MEMORY_DIR / "organism.md"
CODEX_LOCAL_SETUP = MEMORY_DIR / "codex_local_setup.md"
POWER_LOSS_RECOVERY = MEMORY_DIR / "power_loss_recovery.md"
POWER_LOSS_CHECKPOINT = MEMORY_DIR / "checkpoint.md"
MEMORY_SCRIPT = REPO_ROOT / "scripts" / "agent_memory.py"


class AgentMemoryContractTest(unittest.TestCase):
    def test_agents_bootstrap_points_to_shared_memory(self):
        agents_text = AGENTS_MD.read_text(encoding="utf-8")

        self.assertIn("Codex 会话启动协议", agents_text)
        self.assertIn("Codex 工程有机体协议", agents_text)
        self.assertIn("每次开启 Codex 会话", agents_text)
        self.assertIn("codex debug prompt-input", agents_text)
        self.assertIn("docs/agent_memory/README.md", agents_text)
        self.assertIn("docs/agent_memory/current.md", agents_text)
        self.assertIn("docs/agent_memory/organism.md", agents_text)
        self.assertIn("docs/agent_memory/codex_local_setup.md", agents_text)
        self.assertIn("docs/agent_memory/power_loss_recovery.md", agents_text)
        self.assertIn("checkpoint", agents_text)
        self.assertIn("recover", agents_text)
        self.assertIn("scripts/agent_memory.py", agents_text)
        self.assertIn("示例泛化原则", agents_text)
        self.assertIn("比如", agents_text)
        self.assertIn("例如", agents_text)
        self.assertIn("机械照搬", agents_text)
        self.assertIn("轻启动", agents_text)
        self.assertIn("按需扩展", agents_text)
        self.assertIn("长上下文瘦身原则", agents_text)
        self.assertIn("不要默认展开所有历史文档", agents_text)
        self.assertIn("codex_session_guard.py", agents_text)

    def test_memory_documents_form_a_complete_agent_handoff_loop(self):
        for path in (
            MEMORY_README,
            MEMORY_CURRENT,
            MEMORY_LOG,
            MEMORY_TEMPLATE,
            MEMORY_ORGANISM,
            CODEX_LOCAL_SETUP,
            POWER_LOSS_RECOVERY,
            POWER_LOSS_CHECKPOINT,
        ):
            with self.subTest(path=str(path.relative_to(REPO_ROOT))):
                self.assertTrue(path.exists())
                self.assertGreater(len(path.read_text(encoding="utf-8").strip()), 0)

        readme_text = MEMORY_README.read_text(encoding="utf-8")
        current_text = MEMORY_CURRENT.read_text(encoding="utf-8")
        log_text = MEMORY_LOG.read_text(encoding="utf-8")
        template_text = MEMORY_TEMPLATE.read_text(encoding="utf-8")
        organism_text = MEMORY_ORGANISM.read_text(encoding="utf-8")
        local_setup_text = CODEX_LOCAL_SETUP.read_text(encoding="utf-8")
        recovery_text = POWER_LOSS_RECOVERY.read_text(encoding="utf-8")
        checkpoint_text = POWER_LOSS_CHECKPOINT.read_text(encoding="utf-8")

        self.assertIn("current.md", readme_text)
        self.assertIn("session_log.md", readme_text)
        self.assertIn("session_template.md", readme_text)
        self.assertIn("organism.md", readme_text)
        self.assertIn("轻启动", readme_text)
        self.assertIn("按需扩展", readme_text)
        self.assertIn("长上下文瘦身", readme_text)
        self.assertIn("codex_session_guard.py", readme_text)
        self.assertIn("codex_local_setup.md", readme_text)
        self.assertIn("power_loss_recovery.md", readme_text)
        self.assertIn("checkpoint.md", readme_text)
        self.assertIn("scripts/agent_memory.py", readme_text)
        self.assertIn("CHANGELOG.md", current_text)
        self.assertIn("docs/handoff", current_text)
        self.assertIn("Codex 工程有机体协议", current_text)
        self.assertIn("示例泛化原则", current_text)
        self.assertIn("轻启动原则", current_text)
        self.assertIn("长上下文瘦身原则", current_text)
        self.assertIn("按需扩展", current_text)
        self.assertIn("codex_session_guard.py", current_text)
        self.assertIn("比如", current_text)
        self.assertIn("例如", current_text)
        self.assertIn("AGENT-MEMORY:", log_text)
        self.assertIn("影响范围", template_text)
        self.assertIn("验证证据", template_text)
        self.assertIn("感知层", organism_text)
        self.assertIn("记忆层", organism_text)
        self.assertIn("免疫层", organism_text)
        self.assertIn("生长层", organism_text)
        self.assertIn("上下文瘦身层", organism_text)
        self.assertIn("示例泛化层", organism_text)
        self.assertIn("示例不是需求边界", organism_text)
        self.assertIn("机械照搬", organism_text)
        self.assertIn("codex -C /home/hyq-/simple_lashingrobot_ws", local_setup_text)
        self.assertIn("codex debug prompt-input", local_setup_text)
        self.assertIn("断电恢复层", recovery_text)
        self.assertIn("checkpoint", recovery_text)
        self.assertIn("recover", recovery_text)
        self.assertIn("Agent Power-Loss Checkpoint", checkpoint_text)

    def test_memory_cli_check_and_refresh_are_available(self):
        check_result = subprocess.run(
            ["python3", str(MEMORY_SCRIPT), "check"],
            cwd=REPO_ROOT,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(
            check_result.returncode,
            0,
            check_result.stdout + check_result.stderr,
        )
        self.assertIn("agent memory contract ok", check_result.stdout)

        refresh_result = subprocess.run(
            ["python3", str(MEMORY_SCRIPT), "refresh", "--dry-run"],
            cwd=REPO_ROOT,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(
            refresh_result.returncode,
            0,
            refresh_result.stdout + refresh_result.stderr,
        )
        self.assertIn("# Agent Memory Current Snapshot", refresh_result.stdout)
        self.assertIn("Recent Session Memory", refresh_result.stdout)
        self.assertIn("示例泛化原则", refresh_result.stdout)
        self.assertIn("长上下文瘦身原则", refresh_result.stdout)
        self.assertIn("codex_session_guard.py", refresh_result.stdout)

    def test_memory_cli_checkpoint_and_recover_are_available(self):
        checkpoint_result = subprocess.run(
            [
                "python3",
                str(MEMORY_SCRIPT),
                "checkpoint",
                "--dry-run",
                "--task",
                "unit-test checkpoint task",
                "--next",
                "unit-test recovery step",
                "--validation",
                "python3 -m unittest unit-test",
            ],
            cwd=REPO_ROOT,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(
            checkpoint_result.returncode,
            0,
            checkpoint_result.stdout + checkpoint_result.stderr,
        )
        self.assertIn("# Agent Power-Loss Checkpoint", checkpoint_result.stdout)
        self.assertIn("unit-test checkpoint task", checkpoint_result.stdout)
        self.assertIn("unit-test recovery step", checkpoint_result.stdout)
        self.assertIn("python3 -m unittest unit-test", checkpoint_result.stdout)

        recover_result = subprocess.run(
            ["python3", str(MEMORY_SCRIPT), "recover"],
            cwd=REPO_ROOT,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(
            recover_result.returncode,
            0,
            recover_result.stdout + recover_result.stderr,
        )
        self.assertIn("Agent Recovery Report", recover_result.stdout)
        self.assertIn("checkpoint.md", recover_result.stdout)


if __name__ == "__main__":
    unittest.main()
