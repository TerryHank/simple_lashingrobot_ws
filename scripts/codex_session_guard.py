#!/usr/bin/env python3

import argparse
import json
import os
import re
import shutil
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


DEFAULT_CODEX_HOME = Path.home() / ".codex"
DEFAULT_THRESHOLD_MB = 50
DEFAULT_SUMMARY_THRESHOLD_MB = 100
DEFAULT_MIN_AGE_MINUTES = 0
ARCHIVE_SUBDIR = Path("archived_sessions") / "oversized"
SUMMARY_SUBDIR = Path("session_summaries") / "oversized"


@dataclass(frozen=True)
class SessionFile:
    path: Path
    relative_path: Path
    size_bytes: int
    session_id: str
    timestamp: str


def read_session_meta(path):
    try:
        with path.open("r", encoding="utf-8") as handle:
            first_line = handle.readline()
    except OSError:
        return "", ""
    try:
        record = json.loads(first_line)
    except json.JSONDecodeError:
        return "", ""
    payload = record.get("payload", {}) if isinstance(record, dict) else {}
    return str(payload.get("id", "")), str(payload.get("timestamp", ""))


def is_file_open(path):
    target = Path(path).resolve()
    proc_dir = Path("/proc")
    if not proc_dir.exists():
        return False

    for fd_dir in proc_dir.glob("[0-9]*/fd"):
        try:
            for fd_path in fd_dir.iterdir():
                try:
                    if Path(os.readlink(fd_path)).resolve() == target:
                        return True
                except (FileNotFoundError, OSError, RuntimeError):
                    continue
        except (FileNotFoundError, PermissionError, OSError):
            continue
    return False


def find_oversized_sessions(codex_home, threshold_bytes, min_age_seconds=0, skip_open=False):
    codex_home = Path(codex_home).expanduser()
    sessions_dir = codex_home / "sessions"
    if not sessions_dir.exists():
        return []

    matches = []
    now = time.time()
    for path in sorted(sessions_dir.rglob("*.jsonl")):
        if not path.is_file():
            continue
        stat = path.stat()
        size_bytes = stat.st_size
        if size_bytes <= threshold_bytes:
            continue
        if min_age_seconds and now - stat.st_mtime < min_age_seconds:
            continue
        if skip_open and is_file_open(path):
            continue
        session_id, timestamp = read_session_meta(path)
        matches.append(
            SessionFile(
                path=path,
                relative_path=path.relative_to(sessions_dir),
                size_bytes=size_bytes,
                session_id=session_id,
                timestamp=timestamp,
            )
        )
    return matches


def append_manifest(codex_home, archived_items):
    if not archived_items:
        return
    archive_root = Path(codex_home).expanduser() / ARCHIVE_SUBDIR
    manifest = archive_root / "INDEX.md"
    archive_root.mkdir(parents=True, exist_ok=True)

    header = "# Oversized Codex Sessions\n\n"
    if manifest.exists():
        existing = manifest.read_text(encoding="utf-8")
    else:
        existing = header
    if not existing.startswith(header):
        existing = header + existing

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = [existing.rstrip(), "", f"## {timestamp}", ""]
    for item, destination in archived_items:
        size_mb = item.size_bytes / (1024 * 1024)
        session_id = item.session_id or "unknown"
        session_time = item.timestamp or "unknown"
        lines.append(
            f"- `{item.relative_path}` -> `{destination.relative_to(archive_root)}` "
            f"({size_mb:.1f} MB, id `{session_id}`, timestamp `{session_time}`)"
        )
    lines.append("")
    manifest.write_text("\n".join(lines), encoding="utf-8")


def unique_destination(path):
    path = Path(path)
    if not path.exists():
        return path

    suffix = datetime.now().strftime("%Y%m%d%H%M%S")
    candidate = path.with_name(f"{path.stem}-{suffix}{path.suffix}")
    counter = 1
    while candidate.exists():
        candidate = path.with_name(f"{path.stem}-{suffix}-{counter}{path.suffix}")
        counter += 1
    return candidate


def text_from_content(content):
    if isinstance(content, str):
        return content.strip()
    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, str):
                parts.append(item)
            elif isinstance(item, dict):
                value = item.get("text") or item.get("content") or item.get("value")
                if isinstance(value, str):
                    parts.append(value)
        return "\n".join(part.strip() for part in parts if part and part.strip())
    if isinstance(content, dict):
        value = content.get("text") or content.get("content") or content.get("value")
        if isinstance(value, str):
            return value.strip()
    return ""


def read_json_record_line(path, line_number=1):
    try:
        with Path(path).open("r", encoding="utf-8", errors="replace") as handle:
            for current_line_number, line in enumerate(handle, start=1):
                if current_line_number == line_number:
                    return json.loads(line)
    except (OSError, json.JSONDecodeError):
        return None
    return None


def payload_message_text(payload):
    if not isinstance(payload, dict):
        return ""
    if payload.get("type") == "message":
        return text_from_content(payload.get("content"))
    if payload.get("type") in ("user_message", "agent_message"):
        return str(payload.get("message") or payload.get("text") or "").strip()
    return ""


def is_user_message_payload(payload):
    if not isinstance(payload, dict):
        return False
    return (
        payload.get("type") == "message"
        and payload.get("role") == "user"
    ) or payload.get("type") == "user_message"


def is_bootstrap_user_message(text):
    normalized = re.sub(r"\s+", " ", str(text)).strip()
    if not normalized:
        return True
    if normalized.startswith("# AGENTS.md instructions"):
        return True
    if normalized.startswith("<environment_context"):
        return True
    if "<environment_context>" in normalized and "## My request" not in normalized:
        return True
    return False


def extract_title_anchor_record(path):
    first_user_record = None
    try:
        handle = Path(path).open("r", encoding="utf-8", errors="replace")
    except OSError:
        return None

    with handle:
        for line in handle:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            payload = record.get("payload", {}) if isinstance(record, dict) else {}
            if not is_user_message_payload(payload):
                continue
            if first_user_record is None:
                first_user_record = record
            text = payload_message_text(payload)
            if not is_bootstrap_user_message(text):
                return record
    return first_user_record


def build_summary_meta_record(item, archived_destination, codex_home, generated_at, source_path):
    record = read_json_record_line(source_path) or {}
    if not isinstance(record, dict):
        record = {}
    payload = record.get("payload")
    if not isinstance(payload, dict):
        payload = {}
        record["payload"] = payload
    record.setdefault("type", "session_meta")
    payload.setdefault("id", item.session_id or "")
    payload.setdefault("timestamp", item.timestamp or generated_at)
    payload["summary_replacement"] = True
    payload["original_size_bytes"] = item.size_bytes
    payload["archived_original"] = archive_label(codex_home, archived_destination)
    payload["generated_at"] = generated_at
    return record


def shorten_text(text, limit=480):
    text = re.sub(r"\s+", " ", str(text)).strip()
    if len(text) <= limit:
        return text
    return text[: limit - 3].rstrip() + "..."


def extract_session_highlights(path, max_messages=16, max_commands=16):
    highlights = {
        "user_messages": [],
        "assistant_messages": [],
        "commands": [],
        "task_completions": [],
        "line_count": 0,
    }
    try:
        handle = path.open("r", encoding="utf-8", errors="replace")
    except OSError:
        return highlights

    with handle:
        for line in handle:
            highlights["line_count"] += 1
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            payload = record.get("payload", {}) if isinstance(record, dict) else {}
            if not isinstance(payload, dict):
                continue
            payload_type = payload.get("type")
            role = payload.get("role")
            timestamp = str(record.get("timestamp", "") or payload.get("timestamp", ""))

            if payload_type == "message" and role in ("user", "assistant"):
                text = text_from_content(payload.get("content"))
                if not text:
                    continue
                key = "user_messages" if role == "user" else "assistant_messages"
                highlights[key].append((timestamp, shorten_text(text)))
                highlights[key] = highlights[key][-max_messages:]
                continue

            if payload_type in ("user_message", "agent_message"):
                text = payload.get("message") or payload.get("text") or ""
                if not text:
                    continue
                key = "user_messages" if payload_type == "user_message" else "assistant_messages"
                highlights[key].append((timestamp, shorten_text(text)))
                highlights[key] = highlights[key][-max_messages:]
                continue

            if payload_type == "function_call":
                command = payload.get("command") or payload.get("arguments") or payload.get("name") or ""
                if command:
                    highlights["commands"].append((timestamp, shorten_text(command, limit=360)))
                    highlights["commands"] = highlights["commands"][-max_commands:]
                continue

            if payload_type == "task_complete":
                text = payload.get("last_agent_message") or payload.get("message") or ""
                if text:
                    highlights["task_completions"].append((timestamp, shorten_text(text)))
                    highlights["task_completions"] = highlights["task_completions"][-max_messages:]
    return highlights


def format_bullet_items(items, empty_text):
    if not items:
        return [f"- {empty_text}"]
    lines = []
    for timestamp, text in items:
        prefix = f"{timestamp} " if timestamp else ""
        lines.append(f"- {prefix}{text}")
    return lines


def archive_label(codex_home, destination):
    codex_home = Path(codex_home).expanduser()
    destination = Path(destination).expanduser()
    try:
        return str(destination.relative_to(codex_home))
    except ValueError:
        return str(destination)


def build_session_summary(item, generated_at=None, archived_destination=None, codex_home=None):
    generated_at = generated_at or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    size_mb = item.size_bytes / (1024 * 1024)
    highlights = extract_session_highlights(item.path)
    archived_text = "not archived by this dry run"
    if archived_destination is not None:
        archived_text = archive_label(codex_home or DEFAULT_CODEX_HOME, archived_destination)
    lines = [
        "# Codex Session Context Summary",
        "",
        f"- Source: `{item.relative_path}`",
        f"- Original full JSONL content copied to archive at `{archived_text}`.",
        "- Active sessions keep a compact summary JSONL at the original path for fast reopening.",
        f"- Generated: {generated_at}",
        f"- Size: {size_mb:.1f} MB",
        f"- Session id: `{item.session_id or 'unknown'}`",
        f"- Session timestamp: `{item.timestamp or 'unknown'}`",
        f"- Parsed JSONL lines: {highlights['line_count']}",
        "",
        "## Recent User Requests",
        "",
    ]
    lines.extend(format_bullet_items(highlights["user_messages"], "no user messages parsed"))
    lines.extend(["", "## Recent Assistant Context", ""])
    lines.extend(format_bullet_items(highlights["assistant_messages"], "no assistant messages parsed"))
    lines.extend(["", "## Recent Tool Commands", ""])
    lines.extend(format_bullet_items(highlights["commands"], "no tool commands parsed"))
    lines.extend(["", "## Task Completion Notes", ""])
    lines.extend(format_bullet_items(highlights["task_completions"], "no task completion notes parsed"))
    lines.append("")
    return "\n".join(lines)


def summary_path_for(codex_home, item):
    summary_root = Path(codex_home).expanduser() / SUMMARY_SUBDIR
    return (summary_root / item.relative_path).with_suffix(".summary.md")


def build_summary_replacement_jsonl(
    item,
    archived_destination,
    summary_text,
    codex_home,
    source_path=None,
):
    generated_at = datetime.now().isoformat(timespec="seconds")
    archived_text = archive_label(codex_home, archived_destination)
    source_path = source_path or item.path
    meta = build_summary_meta_record(
        item,
        archived_destination,
        codex_home,
        generated_at,
        source_path,
    )
    title_anchor = extract_title_anchor_record(source_path)
    message_text = (
        "This active Codex session was compacted to keep the session list responsive.\n\n"
        f"Original full JSONL content copied to archive at `{archived_text}`.\n\n"
        f"{summary_text}"
    )
    message = {
        "timestamp": generated_at,
        "payload": {
            "type": "message",
            "role": "assistant",
            "content": [{"type": "output_text", "text": message_text}],
        },
    }
    records = [meta]
    if title_anchor:
        records.append(title_anchor)
    records.append(message)
    return "\n".join(json.dumps(record, ensure_ascii=False) for record in records) + "\n"


def archive_oversized_sessions(
    codex_home,
    threshold_bytes,
    apply=False,
    min_age_seconds=0,
    skip_open=False,
):
    codex_home = Path(codex_home).expanduser()
    archive_root = codex_home / ARCHIVE_SUBDIR
    matches = find_oversized_sessions(
        codex_home,
        threshold_bytes,
        min_age_seconds=min_age_seconds,
        skip_open=skip_open,
    )
    archived = []

    for item in matches:
        destination = archive_root / item.relative_path
        if not apply:
            archived.append((item, destination))
            continue
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination = unique_destination(destination)
        shutil.move(str(item.path), str(destination))
        archived.append((item, destination))

    if apply:
        append_manifest(codex_home, archived)
    return archived


def summarize_oversized_sessions(
    codex_home,
    threshold_bytes,
    apply=False,
    min_age_seconds=0,
    skip_open=False,
):
    codex_home = Path(codex_home).expanduser()
    archive_root = codex_home / ARCHIVE_SUBDIR
    matches = find_oversized_sessions(
        codex_home,
        threshold_bytes,
        min_age_seconds=min_age_seconds,
        skip_open=skip_open,
    )
    summaries = []
    archived = []

    for item in matches:
        summary_destination = summary_path_for(codex_home, item)
        archive_destination = unique_destination(archive_root / item.relative_path)
        summaries.append((item, summary_destination))
        if not apply:
            continue
        summary_text = build_session_summary(
            item,
            archived_destination=archive_destination,
            codex_home=codex_home,
        )
        replacement_text = build_summary_replacement_jsonl(
            item,
            archive_destination,
            summary_text,
            codex_home,
        )
        summary_destination.parent.mkdir(parents=True, exist_ok=True)
        archive_destination.parent.mkdir(parents=True, exist_ok=True)
        summary_destination.write_text(summary_text, encoding="utf-8")
        shutil.copy2(str(item.path), str(archive_destination))
        item.path.write_text(replacement_text, encoding="utf-8")
        archived.append((item, archive_destination))

    if apply:
        append_manifest(codex_home, archived)
    return summaries


def find_summary_replacements(codex_home):
    codex_home = Path(codex_home).expanduser()
    sessions_dir = codex_home / "sessions"
    if not sessions_dir.exists():
        return []

    replacements = []
    for path in sorted(sessions_dir.rglob("*.jsonl")):
        record = read_json_record_line(path)
        payload = record.get("payload", {}) if isinstance(record, dict) else {}
        if not isinstance(payload, dict) or not payload.get("summary_replacement"):
            continue
        archived_original = payload.get("archived_original")
        if not archived_original:
            continue
        archived_path = codex_home / str(archived_original)
        if not archived_path.exists():
            continue
        original_size = int(payload.get("original_size_bytes") or archived_path.stat().st_size)
        replacements.append(
            (
                path,
                SessionFile(
                    path=archived_path,
                    relative_path=path.relative_to(sessions_dir),
                    size_bytes=original_size,
                    session_id=str(payload.get("id", "")),
                    timestamp=str(payload.get("timestamp", "")),
                ),
                archived_path,
            )
        )
    return replacements


def repair_summary_replacements(codex_home, apply=False):
    codex_home = Path(codex_home).expanduser()
    repaired = []
    for active_path, item, archived_path in find_summary_replacements(codex_home):
        repaired.append((item, active_path))
        if not apply:
            continue
        summary_destination = summary_path_for(codex_home, item)
        summary_text = build_session_summary(
            item,
            archived_destination=archived_path,
            codex_home=codex_home,
        )
        replacement_text = build_summary_replacement_jsonl(
            item,
            archived_path,
            summary_text,
            codex_home,
            source_path=archived_path,
        )
        summary_destination.parent.mkdir(parents=True, exist_ok=True)
        summary_destination.write_text(summary_text, encoding="utf-8")
        active_path.write_text(replacement_text, encoding="utf-8")
    return repaired


def print_items(items, action):
    if not items:
        print("no oversized active Codex sessions found")
        return
    for item, destination in items:
        size_mb = item.size_bytes / (1024 * 1024)
        print(
            f"{action}: {item.path} -> {destination} "
            f"({size_mb:.1f} MB, id={item.session_id or 'unknown'})"
        )


def build_parser():
    def add_common_options(
        target,
        default_codex_home=argparse.SUPPRESS,
        default_threshold=argparse.SUPPRESS,
        default_min_age=argparse.SUPPRESS,
    ):
        target.add_argument(
            "--codex-home",
            default=default_codex_home,
            help="Codex home directory, default: ~/.codex",
        )
        target.add_argument(
            "--threshold-mb",
            type=float,
            default=default_threshold,
            help="Process active sessions larger than this size.",
        )
        target.add_argument(
            "--min-age-minutes",
            type=float,
            default=default_min_age,
            help="Only process sessions that have not been modified for this many minutes.",
        )
        target.add_argument(
            "--skip-open",
            action="store_true",
            help="Skip JSONL files that are currently open by any process.",
        )

    parser = argparse.ArgumentParser(
        description="Scan, summarize, or archive oversized Codex JSONL sessions."
    )
    add_common_options(
        parser,
        default_codex_home=str(DEFAULT_CODEX_HOME),
        default_threshold=DEFAULT_THRESHOLD_MB,
        default_min_age=DEFAULT_MIN_AGE_MINUTES,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    scan = subparsers.add_parser("scan", help="show oversized active sessions")
    add_common_options(scan)
    scan.set_defaults(apply=False)

    archive = subparsers.add_parser("archive", help="archive oversized active sessions")
    add_common_options(archive)
    archive.add_argument("--apply", action="store_true", help="actually move files")
    archive.set_defaults(apply=None)

    summarize = subparsers.add_parser(
        "summarize",
        help="copy original oversized session content to archive and leave compact summary JSONL in place",
    )
    add_common_options(summarize, default_threshold=DEFAULT_SUMMARY_THRESHOLD_MB)
    summarize.add_argument("--apply", action="store_true", help="actually compact matching sessions")
    summarize.set_defaults(apply=None)

    repair = subparsers.add_parser(
        "repair-summaries",
        help="rewrite existing compact summary JSONL files from archived originals",
    )
    add_common_options(repair, default_threshold=DEFAULT_SUMMARY_THRESHOLD_MB)
    repair.add_argument("--apply", action="store_true", help="actually rewrite summary replacements")
    repair.set_defaults(apply=None)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    threshold_bytes = int(args.threshold_mb * 1024 * 1024)
    min_age_seconds = int(args.min_age_minutes * 60)
    if args.command == "scan":
        items = archive_oversized_sessions(
            args.codex_home,
            threshold_bytes,
            apply=False,
            min_age_seconds=min_age_seconds,
            skip_open=args.skip_open,
        )
        print_items(items, "would archive")
        return 0
    if args.command == "archive":
        items = archive_oversized_sessions(
            args.codex_home,
            threshold_bytes,
            apply=args.apply,
            min_age_seconds=min_age_seconds,
            skip_open=args.skip_open,
        )
        print_items(items, "archived" if args.apply else "would archive")
        if not args.apply and items:
            print("rerun with --apply to move these files")
        return 0
    if args.command == "summarize":
        items = summarize_oversized_sessions(
            args.codex_home,
            threshold_bytes,
            apply=args.apply,
            min_age_seconds=min_age_seconds,
            skip_open=args.skip_open,
        )
        print_items(items, "summarized" if args.apply else "would summarize")
        if not args.apply and items:
            print("rerun with --apply to copy original content into archive and write compact summary JSONL files")
        return 0
    if args.command == "repair-summaries":
        items = repair_summary_replacements(args.codex_home, apply=args.apply)
        print_items(items, "repaired" if args.apply else "would repair")
        if not args.apply and items:
            print("rerun with --apply to rewrite these summary replacement files")
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
