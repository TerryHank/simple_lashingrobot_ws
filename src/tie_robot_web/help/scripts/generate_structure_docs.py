#!/usr/bin/env python3

from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
SRC_ROOT = WORKSPACE_ROOT / "src"
OUTPUT = WORKSPACE_ROOT / "src" / "tie_robot_web" / "help" / "reference" / "file-tree.md"
PACKAGE_PREFIX = "tie_robot_"


def render_tree() -> str:
    lines = ["# 文件结构树", "", "当前 `src/` 主包如下：", ""]
    for package_dir in sorted(
        p for p in SRC_ROOT.iterdir() if p.is_dir() and p.name.startswith(PACKAGE_PREFIX)
    ):
        lines.append(f"## {package_dir.name}")
        lines.append("")
        for child in sorted(package_dir.iterdir()):
            lines.append(f"- `{package_dir.name}/{child.name}`")
        lines.append("")
    return "\n".join(lines).strip() + "\n"


def main() -> None:
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(render_tree(), encoding="utf-8")


if __name__ == "__main__":
    main()
