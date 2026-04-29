#!/usr/bin/env python3
"""Check ROS interface string literals against the canonical migration map."""
import argparse
import pathlib
import re
import sys

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_MAP = REPO_ROOT / "docs" / "architecture" / "ros_interface_migration_map.yaml"
DEFAULT_ROOTS = [
    REPO_ROOT / "src" / "tie_robot_perception",
    REPO_ROOT / "src" / "tie_robot_control",
    REPO_ROOT / "src" / "tie_robot_process",
    REPO_ROOT / "src" / "tie_robot_web" / "frontend" / "src",
    REPO_ROOT / "src" / "tie_robot_web" / "src",
    REPO_ROOT / "src" / "tie_robot_bringup" / "launch",
]
TEXT_EXTENSIONS = {
    ".cpp",
    ".h",
    ".hpp",
    ".js",
    ".jsx",
    ".launch",
    ".mjs",
    ".py",
    ".ts",
    ".tsx",
    ".xml",
}
SKIP_DIRS = {"node_modules", "web", "build", "devel", ".git"}
INTERFACE_LITERAL_RE = re.compile(
    r"""["'](/(?:web|pointAI|coordinate_point|Scepter|cabin|moduan|robot|system_log|tf|diagnostics|rosout)[^"']*)["']"""
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default=str(DEFAULT_MAP), help="Path to ros_interface_migration_map.yaml")
    parser.add_argument(
        "--roots",
        nargs="+",
        default=[str(path) for path in DEFAULT_ROOTS],
        help="Source roots to scan",
    )
    parser.add_argument("--quiet", action="store_true", help="Only print failures")
    return parser.parse_args()


def load_known_names(map_path):
    data = yaml.safe_load(pathlib.Path(map_path).read_text()) or {}
    known = set(data.get("standard_keep", {}).get("topics", []))
    for group in data.get("interfaces", {}).values():
        for item in group:
            legacy = item.get("legacy")
            canonical = item.get("canonical")
            if legacy:
                known.add(legacy)
            if canonical:
                known.add(canonical)
    return known


def iter_source_files(roots):
    for root_name in roots:
        root = pathlib.Path(root_name)
        if not root.exists():
            continue
        if root.is_file():
            if root.suffix in TEXT_EXTENSIONS:
                yield root
            continue
        for path in root.rglob("*"):
            if not path.is_file() or path.suffix not in TEXT_EXTENSIONS:
                continue
            if any(part in SKIP_DIRS for part in path.parts):
                continue
            yield path


def find_interface_literals(paths):
    found = {}
    for path in paths:
        text = path.read_text(errors="ignore")
        for match in INTERFACE_LITERAL_RE.finditer(text):
            name = match.group(1)
            if name.startswith("/home/"):
                continue
            found.setdefault(name, set()).add(path)
    return found


def is_allowed_without_map(name):
    return name.startswith("/tf2_")


def main():
    args = parse_args()
    known = load_known_names(args.map)
    found = find_interface_literals(iter_source_files(args.roots))
    missing = sorted(name for name in found if name not in known and not is_allowed_without_map(name))

    if missing:
        print("Unmapped ROS interface literals:")
        for name in missing:
            locations = ", ".join(str(path) for path in sorted(found[name])[:3])
            print(f"  {name} :: {locations}")
        return 1

    if not args.quiet:
        print(f"ROS interface names ok: found={len(found)} known={len(known)} missing=0")
    return 0


if __name__ == "__main__":
    sys.exit(main())
