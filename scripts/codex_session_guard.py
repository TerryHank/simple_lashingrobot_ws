#!/usr/bin/env python3

import argparse
import json
import os
import shutil
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


DEFAULT_CODEX_HOME = Path.home() / ".codex"
DEFAULT_THRESHOLD_MB = 50
DEFAULT_MIN_AGE_MINUTES = 0
ARCHIVE_SUBDIR = Path("archived_sessions") / "oversized"


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
        if destination.exists():
            suffix = datetime.now().strftime("%Y%m%d%H%M%S")
            destination = destination.with_name(f"{destination.stem}-{suffix}{destination.suffix}")
        shutil.move(str(item.path), str(destination))
        archived.append((item, destination))

    if apply:
        append_manifest(codex_home, archived)
    return archived


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
            help="Archive active sessions larger than this size.",
        )
        target.add_argument(
            "--min-age-minutes",
            type=float,
            default=default_min_age,
            help="Only archive sessions that have not been modified for this many minutes.",
        )
        target.add_argument(
            "--skip-open",
            action="store_true",
            help="Skip JSONL files that are currently open by any process.",
        )

    parser = argparse.ArgumentParser(
        description="Move oversized Codex JSONL sessions out of the active history."
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
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
