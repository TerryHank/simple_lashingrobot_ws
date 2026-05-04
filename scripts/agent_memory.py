#!/usr/bin/env python3

import argparse
import os
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
AGENTS_MD = REPO_ROOT / "AGENTS.md"
CHANGELOG_MD = REPO_ROOT / "CHANGELOG.md"
MEMORY_DIR = REPO_ROOT / "docs" / "agent_memory"
MEMORY_README = MEMORY_DIR / "README.md"
MEMORY_CURRENT = MEMORY_DIR / "current.md"
MEMORY_LOG = MEMORY_DIR / "session_log.md"
MEMORY_TEMPLATE = MEMORY_DIR / "session_template.md"
MEMORY_ORGANISM = MEMORY_DIR / "organism.md"
CODEX_LOCAL_SETUP = MEMORY_DIR / "codex_local_setup.md"
POWER_LOSS_RECOVERY = MEMORY_DIR / "power_loss_recovery.md"
POWER_LOSS_CHECKPOINT = MEMORY_DIR / "checkpoint.md"
HANDOFF_DIR = REPO_ROOT / "docs" / "handoff"


REQUIRED_FILES = (
    AGENTS_MD,
    CHANGELOG_MD,
    MEMORY_README,
    MEMORY_CURRENT,
    MEMORY_LOG,
    MEMORY_TEMPLATE,
    MEMORY_ORGANISM,
    CODEX_LOCAL_SETUP,
    POWER_LOSS_RECOVERY,
    POWER_LOSS_CHECKPOINT,
)


def rel(path):
    return str(path.relative_to(REPO_ROOT))


def read_text(path):
    return path.read_text(encoding="utf-8")


def write_text(path, text):
    path.write_text(text, encoding="utf-8")


def atomic_write_text(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(f".{path.name}.tmp")
    with tmp_path.open("w", encoding="utf-8") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp_path, path)
    try:
        dir_fd = os.open(str(path.parent), os.O_DIRECTORY)
    except OSError:
        return
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


def run_git(args):
    try:
        result = subprocess.run(
            ["git"] + args,
            cwd=REPO_ROOT,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        return ""
    if result.returncode != 0:
        return ""
    return result.stdout.strip()


def split_csv(values):
    items = []
    for value in values or []:
        for part in value.split(","):
            item = part.strip()
            if item:
                items.append(item)
    return items


def extract_latest_changelog():
    if not CHANGELOG_MD.exists():
        return []
    text = read_text(CHANGELOG_MD)
    match = re.search(r"^##\s+(.+?)\s*$", text, re.MULTILINE)
    if not match:
        return []
    start = match.start()
    next_match = re.search(r"^##\s+.+?\s*$", text[match.end() :], re.MULTILINE)
    end = match.end() + next_match.start() if next_match else len(text)
    section = text[start:end].strip()
    lines = []
    for line in section.splitlines():
        stripped = line.strip()
        if stripped.startswith("- "):
            lines.append(stripped)
        if len(lines) >= 8:
            break
    return lines


def extract_recent_memory(limit=6):
    if not MEMORY_LOG.exists():
        return []
    text = read_text(MEMORY_LOG)
    sections = re.split(r"(?m)^##\s+", text)
    entries = []
    for section in sections[1:]:
        title, _, body = section.partition("\n")
        if "AGENT-MEMORY:" not in body:
            continue
        summary_match = re.search(
            r"(?ms)^###\s+摘要\s*(.+?)(?:^###\s+|\Z)",
            body,
        )
        summary_lines = []
        if summary_match:
            for line in summary_match.group(1).splitlines():
                stripped = line.strip()
                if stripped.startswith("- "):
                    summary_lines.append(stripped[2:].strip())
        summary = " ".join(summary_lines) if summary_lines else "见 session_log.md。"
        entries.append((title.strip(), summary))
        if len(entries) >= limit:
            break
    return entries


def list_handoff_docs():
    if not HANDOFF_DIR.exists():
        return []
    return sorted(HANDOFF_DIR.glob("*.md"), reverse=True)


def limit_lines(text, limit=80):
    lines = text.splitlines()
    if len(lines) <= limit:
        return lines
    omitted = len(lines) - limit
    return lines[:limit] + [f"... omitted {omitted} more lines ..."]


def build_snapshot():
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    commit = run_git(["rev-parse", "--short", "HEAD"]) or "unknown"
    changelog_lines = extract_latest_changelog()
    memory_entries = extract_recent_memory()
    handoff_docs = list_handoff_docs()

    lines = [
        "# Agent Memory Current Snapshot",
        "",
        f"> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：{timestamp}，当前 HEAD：`{commit}`。",
        "",
        "## Bootstrap Files",
        "",
        "轻启动必读：",
        "",
        "- `README.md`：项目总览、包边界、启动方式和主链路。",
        "- `CHANGELOG.md`：近期项目级约定和关键调整。",
        "- `docs/agent_memory/current.md`：当前共享记忆快照。",
        "",
        "按需扩展读取：",
        "",
        "- `docs/agent_memory/README.md`：维护共享记忆、写入账本或校验契约时读取。",
        "- `docs/agent_memory/organism.md`：调整 agent 行为、工程协作协议或有机体口径时读取。",
        "- `docs/agent_memory/codex_local_setup.md`：排查 Codex 启动目录或模型输入注入时读取。",
        "- `docs/agent_memory/power_loss_recovery.md`：突然断电后的恢复协议。",
        "- `docs/agent_memory/checkpoint.md`：最近一次可恢复现场。",
        "- `docs/handoff`：只在当前任务命中对应专题时读取。",
        "",
        "## Current High-Signal Memory",
        "",
        "- 当前工程已经收口到 `tie_robot_msgs`、`tie_robot_hw`、`tie_robot_perception`、`tie_robot_control`、`tie_robot_process`、`tie_robot_web`、`tie_robot_bringup`、`tie_robot_description` 这 8 个主包。",
        "- 新前端源码在 `src/tie_robot_web/frontend`，静态产物在 `src/tie_robot_web/web`。如果修改前端且影响静态页面，需要重新构建产物。",
        "- 不要恢复控制面板按钮上方的任务提示框，除非用户明确要求。",
        "- 本工程里的 Codex 按 `docs/agent_memory/organism.md`（Codex 工程有机体协议）运行：先感知、再行动、持续记忆、完成前自检。",
        "- 轻启动原则：新会话默认先读 `README.md`、`CHANGELOG.md`、`docs/agent_memory/current.md`；其他记忆文档按需扩展读取，避免长上下文卡顿。",
        "- 示例泛化原则：用户说“比如”“例如”“类似”时，示例不是需求边界；先识别真实目标和工程约束，再举一反三，避免机械照搬。",
        "- 长上下文瘦身原则：优先读摘要和相关片段，不把整仓状态、大段日志、构建产物、`.debug_frames` 或无关历史全文塞入上下文；卡顿时写 checkpoint 并开新会话续接。",
        "- Codex 会话超过 100MB 时使用 `python3 scripts/codex_session_guard.py summarize --threshold-mb 100 --skip-open --apply` 压缩活跃会话：原始完整 JSONL 内容复制到 `~/.codex/archived_sessions/oversized/`，`~/.codex/sessions` 里的会话文件不移动出原路径，只把原路径内容改写为摘要替身 JSONL；本机启用 `tie-codex-session-summary.timer` 自动执行该流程。",
        "- Codex 摘要替身必须保留原始 `session_meta` 关键字段和首条真实用户请求作为标题锚点；已有摘要替身可用 `python3 scripts/codex_session_guard.py repair-summaries --apply` 从 archive 原文修复标题锚点。",
        "- 长任务或高风险修改前后运行 `python3 scripts/agent_memory.py checkpoint ...`，断电后用 `python3 scripts/agent_memory.py recover` 读取恢复报告。",
        "- 当前认可的手动工作区 S2 是 `PR-FPRG 透视展开频相回归网格方案`；不要退回原图 bbox 估周期、像素尺度 rectified 或 `pre_img()` 前置门控。",
        "- 涉及 TF、点云或 3D Scene 时，先读 `CHANGELOG.md` 中关于相机、TCP 和前端显示坐标系的约定。",
        "",
        "## Latest CHANGELOG Signals",
        "",
    ]
    if changelog_lines:
        lines.extend(changelog_lines)
    else:
        lines.append("- 未找到可摘取的 `CHANGELOG.md` 条目。")

    lines.extend(["", "## Recent Session Memory", ""])
    if memory_entries:
        for title, summary in memory_entries:
            lines.append(f"- `{title}`：{summary}")
    else:
        lines.append("- 暂无带 `AGENT-MEMORY:` 标记的会话记忆。")

    lines.extend(["", "## Handoff Documents", ""])
    if handoff_docs:
        for path in handoff_docs[:10]:
            lines.append(f"- `{rel(path)}`")
    else:
        lines.append("- 暂无 `docs/handoff` 文档。")

    lines.extend(
        [
            "",
            "## Update Protocol",
            "",
            "1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。",
            "2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。",
            "3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。",
            "",
        ]
    )
    return "\n".join(lines)


def add_entry(args):
    MEMORY_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = args.when or datetime.now().strftime("%Y-%m-%d %H:%M")
    files = split_csv(args.files)
    tags = split_csv(args.tags)
    validations = split_csv(args.validation)
    decisions = args.decision or []
    notes = args.note or []

    entry = [
        f"## {timestamp} - {args.title}",
        "",
        "<!-- AGENT-MEMORY: entry -->",
        "",
        "### 摘要",
        "",
        f"- {args.summary}",
        "",
        "### 影响范围",
        "",
    ]
    if files:
        entry.extend(f"- `{item}`" for item in files)
    else:
        entry.append("- 未指定。")

    entry.extend(["", "### 关键决策", ""])
    if decisions:
        entry.extend(f"- {item}" for item in decisions)
    else:
        entry.append("- 见摘要。")

    if tags:
        entry.extend(["", "### 标签", ""])
        entry.extend(f"- `{item}`" for item in tags)

    entry.extend(["", "### 验证证据", ""])
    if validations:
        entry.extend(f"- `{item}`" for item in validations)
    else:
        entry.append("- 未记录验证命令。")

    entry.extend(["", "### 后续注意", ""])
    if notes:
        entry.extend(f"- {item}" for item in notes)
    else:
        entry.append("- 暂无。")
    entry.append("")

    existing = read_text(MEMORY_LOG) if MEMORY_LOG.exists() else "# Agent Session Memory Log\n\n"
    marker = "本文件按时间倒序记录跨会话共享记忆。新条目写在最上方，并保留 `AGENT-MEMORY:` 标记，方便脚本识别。\n\n"
    if "AGENT-MEMORY:" in existing:
        header, _, rest = existing.partition("## ")
        new_text = header.rstrip() + "\n\n" + "\n".join(entry) + "\n## " + rest.lstrip()
    else:
        header = existing.rstrip()
        if marker not in header:
            header = "# Agent Session Memory Log\n\n" + marker.rstrip()
        new_text = header.rstrip() + "\n\n" + "\n".join(entry)
    write_text(MEMORY_LOG, new_text)
    write_text(MEMORY_CURRENT, build_snapshot())
    print(f"added memory entry: {args.title}")
    print(f"refreshed {rel(MEMORY_CURRENT)}")


def refresh(args):
    snapshot = build_snapshot()
    if args.dry_run:
        print(snapshot)
    else:
        write_text(MEMORY_CURRENT, snapshot)
        print(f"refreshed {rel(MEMORY_CURRENT)}")


def build_checkpoint(args):
    timestamp = args.when or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    head = run_git(["rev-parse", "--short", "HEAD"]) or "unknown"
    branch = run_git(["branch", "--show-current"]) or "unknown"
    status = run_git(["status", "--short"])
    files = split_csv(args.files)
    validations = split_csv(args.validation)
    next_steps = args.next or []
    notes = args.note or []
    task = args.task or "未指定当前任务。"

    lines = [
        "# Agent Power-Loss Checkpoint",
        "",
        f"- 更新时间：{timestamp}",
        f"- HEAD：`{head}`",
        f"- 分支：`{branch}`",
        "",
        "## 当前任务",
        "",
        f"- {task}",
        "",
        "## 下一步",
        "",
    ]
    if next_steps:
        lines.extend(f"- {item}" for item in next_steps)
    else:
        lines.append("- 先运行 `python3 scripts/agent_memory.py recover`，再读取 `AGENTS.md`、`docs/agent_memory/current.md` 和本 checkpoint。")

    lines.extend(["", "## 影响范围", ""])
    if files:
        lines.extend(f"- `{item}`" for item in files)
    else:
        lines.append("- 未手工指定；请结合下方 Git 状态判断。")

    lines.extend(["", "## 最近验证", ""])
    if validations:
        lines.extend(f"- `{item}`" for item in validations)
    else:
        lines.append("- 暂无记录。")

    lines.extend(["", "## 注意事项", ""])
    if notes:
        lines.extend(f"- {item}" for item in notes)
    else:
        lines.append("- 断电恢复只能恢复已落盘内容；未保存的 IDE buffer、运行中进程和未写入文件的思路无法保证。")

    lines.extend(["", "## Git 状态摘要", ""])
    if status:
        lines.append("```text")
        lines.extend(limit_lines(status))
        lines.append("```")
    else:
        lines.append("- Git 工作区没有状态输出。")

    lines.extend(
        [
            "",
            "## 恢复命令",
            "",
            "```bash",
            "python3 scripts/agent_memory.py recover",
            "python3 scripts/agent_memory.py check",
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def checkpoint(args):
    text = build_checkpoint(args)
    if args.dry_run:
        print(text)
    else:
        atomic_write_text(POWER_LOSS_CHECKPOINT, text)
        print(f"wrote {rel(POWER_LOSS_CHECKPOINT)}")


def recover(_args):
    checkpoint_text = read_text(POWER_LOSS_CHECKPOINT) if POWER_LOSS_CHECKPOINT.exists() else ""
    current_text = read_text(MEMORY_CURRENT) if MEMORY_CURRENT.exists() else ""
    report = [
        "# Agent Recovery Report",
        "",
        "## 先读这些文件",
        "",
        "- `AGENTS.md`",
        "- `docs/agent_memory/current.md`",
        "- `docs/agent_memory/organism.md`",
        "- `docs/agent_memory/checkpoint.md`",
        "",
        "## 最近 checkpoint.md",
        "",
    ]
    if checkpoint_text:
        report.append(checkpoint_text)
    else:
        report.append("- 未找到 checkpoint；只能根据 Git 状态和共享记忆恢复。")

    report.extend(["", "## 当前记忆快照摘要", ""])
    if current_text:
        report.extend(limit_lines(current_text, limit=60))
    else:
        report.append("- 未找到 current.md。")

    report.extend(
        [
            "",
            "## 建议恢复步骤",
            "",
            "1. 确认工作区状态：`git status --short`。",
            "2. 对照 checkpoint 的下一步和影响范围。",
            "3. 重新运行相关测试或服务状态检查。",
            "4. 继续前先补一条新的 checkpoint。",
            "",
        ]
    )
    print("\n".join(report))


def check(_args):
    missing = [rel(path) for path in REQUIRED_FILES if not path.exists()]
    if missing:
        for path in missing:
            print(f"missing required file: {path}", file=sys.stderr)
        return 1

    agents_text = read_text(AGENTS_MD)
    readme_text = read_text(MEMORY_README)
    current_text = read_text(MEMORY_CURRENT)
    log_text = read_text(MEMORY_LOG)
    template_text = read_text(MEMORY_TEMPLATE)
    organism_text = read_text(MEMORY_ORGANISM)
    local_setup_text = read_text(CODEX_LOCAL_SETUP)
    recovery_text = read_text(POWER_LOSS_RECOVERY)
    checkpoint_text = read_text(POWER_LOSS_CHECKPOINT)

    checks = [
        ("AGENTS links memory README", "docs/agent_memory/README.md" in agents_text),
        ("AGENTS links current snapshot", "docs/agent_memory/current.md" in agents_text),
        ("AGENTS mentions memory CLI", "scripts/agent_memory.py" in agents_text),
        ("AGENTS has Codex bootstrap protocol", "Codex 会话启动协议" in agents_text),
        ("AGENTS mentions Codex session start", "每次开启 Codex 会话" in agents_text),
        ("AGENTS mentions prompt-input verification", "codex debug prompt-input" in agents_text),
        ("AGENTS has organism protocol", "Codex 工程有机体协议" in agents_text),
        ("AGENTS links organism doc", "docs/agent_memory/organism.md" in agents_text),
        ("AGENTS links local setup doc", "docs/agent_memory/codex_local_setup.md" in agents_text),
        ("AGENTS links recovery doc", "docs/agent_memory/power_loss_recovery.md" in agents_text),
        ("AGENTS mentions checkpoint", "checkpoint" in agents_text),
        ("AGENTS mentions recover", "recover" in agents_text),
        ("AGENTS has example generalization", "示例泛化原则" in agents_text),
        ("AGENTS treats examples as hints", "比如" in agents_text and "例如" in agents_text),
        ("AGENTS avoids mechanical copying", "机械照搬" in agents_text),
        ("AGENTS has light bootstrap", "轻启动" in agents_text),
        ("AGENTS has on-demand expansion", "按需扩展" in agents_text),
        ("AGENTS has context slimming", "长上下文瘦身原则" in agents_text),
        ("AGENTS avoids default history expansion", "不要默认展开所有历史文档" in agents_text),
        ("memory README links current", "current.md" in readme_text),
        ("memory README links log", "session_log.md" in readme_text),
        ("memory README links template", "session_template.md" in readme_text),
        ("memory README links organism", "organism.md" in readme_text),
        ("memory README links local setup", "codex_local_setup.md" in readme_text),
        ("memory README links recovery", "power_loss_recovery.md" in readme_text),
        ("memory README links checkpoint", "checkpoint.md" in readme_text),
        ("memory README links CLI", "scripts/agent_memory.py" in readme_text),
        ("memory README captures example preference", "比如" in readme_text and "机械照搬" in readme_text),
        ("memory README has light bootstrap", "轻启动" in readme_text),
        ("memory README has on-demand expansion", "按需扩展" in readme_text),
        ("memory README has context slimming", "长上下文瘦身" in readme_text),
        ("current links CHANGELOG", "CHANGELOG.md" in current_text),
        ("current links handoff docs", "docs/handoff" in current_text),
        ("current mentions organism", "Codex 工程有机体协议" in current_text),
        ("current has example generalization", "示例泛化原则" in current_text),
        ("current has light bootstrap", "轻启动原则" in current_text),
        ("current has context slimming", "长上下文瘦身原则" in current_text),
        ("current has on-demand expansion", "按需扩展" in current_text),
        ("session log has marker", "AGENT-MEMORY:" in log_text),
        ("template asks impact scope", "影响范围" in template_text),
        ("template asks validation evidence", "验证证据" in template_text),
        ("organism has perception layer", "感知层" in organism_text),
        ("organism has memory layer", "记忆层" in organism_text),
        ("organism has immune layer", "免疫层" in organism_text),
        ("organism has growth layer", "生长层" in organism_text),
        ("organism has context slimming layer", "上下文瘦身层" in organism_text),
        ("organism has example generalization layer", "示例泛化层" in organism_text),
        ("organism treats examples as hints", "示例不是需求边界" in organism_text),
        ("organism avoids mechanical copying", "机械照搬" in organism_text),
        ("local setup has codex -C", "codex -C /home/hyq-/simple_lashingrobot_ws" in local_setup_text),
        ("local setup has prompt-input", "codex debug prompt-input" in local_setup_text),
        ("recovery has power loss layer", "断电恢复层" in recovery_text),
        ("recovery mentions checkpoint", "checkpoint" in recovery_text),
        ("recovery mentions recover", "recover" in recovery_text),
        ("checkpoint has title", "Agent Power-Loss Checkpoint" in checkpoint_text),
    ]
    failed = [name for name, ok in checks if not ok]
    if failed:
        for name in failed:
            print(f"failed: {name}", file=sys.stderr)
        return 1
    print("agent memory contract ok")
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="Maintain shared agent memory for this repository.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    add_parser = subparsers.add_parser("add", help="append a memory entry and refresh current.md")
    add_parser.add_argument("--title", required=True)
    add_parser.add_argument("--summary", required=True)
    add_parser.add_argument("--files", action="append", default=[])
    add_parser.add_argument("--validation", action="append", default=[])
    add_parser.add_argument("--decision", action="append", default=[])
    add_parser.add_argument("--note", action="append", default=[])
    add_parser.add_argument("--tags", action="append", default=[])
    add_parser.add_argument("--when")
    add_parser.set_defaults(func=add_entry)

    refresh_parser = subparsers.add_parser("refresh", help="regenerate current.md")
    refresh_parser.add_argument("--dry-run", action="store_true")
    refresh_parser.set_defaults(func=refresh)

    checkpoint_parser = subparsers.add_parser("checkpoint", help="write a recoverable power-loss checkpoint")
    checkpoint_parser.add_argument("--task")
    checkpoint_parser.add_argument("--next", action="append", default=[])
    checkpoint_parser.add_argument("--files", action="append", default=[])
    checkpoint_parser.add_argument("--validation", action="append", default=[])
    checkpoint_parser.add_argument("--note", action="append", default=[])
    checkpoint_parser.add_argument("--when")
    checkpoint_parser.add_argument("--dry-run", action="store_true")
    checkpoint_parser.set_defaults(func=checkpoint)

    recover_parser = subparsers.add_parser("recover", help="print recovery context from checkpoint and memory")
    recover_parser.set_defaults(func=recover)

    check_parser = subparsers.add_parser("check", help="validate memory system links and files")
    check_parser.set_defaults(func=check)

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
