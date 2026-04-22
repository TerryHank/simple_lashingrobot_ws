#!/usr/bin/env python3

from pathlib import Path
import re
import shutil


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
SOURCE_ROOT = WORKSPACE_ROOT / "vzense_wiki"
TARGET_ROOT = (
    WORKSPACE_ROOT
    / "src"
    / "tie_robot_web"
    / "help"
    / "camera-sdk"
    / "vendor-vzense"
)
SKIP_NAMES = {"_sidebar.md", "_navbar.md", "_all_md_files.txt"}


def copy_tree(source: Path, target: Path) -> None:
    for item in source.iterdir():
        if item.name in SKIP_NAMES:
            continue
        destination = target / item.name
        if item.is_dir():
            destination.mkdir(parents=True, exist_ok=True)
            copy_tree(item, destination)
            continue
        shutil.copy2(item, destination)


def sanitize_markdown(markdown_text: str) -> str:
    sanitized = re.sub(r"<!--.*?-->", "", markdown_text, flags=re.S)
    sanitized = re.sub(r"</?(?:div|center)\b[^>]*>", "", sanitized, flags=re.I)
    sanitized = re.sub(r"</?(?:span|font)\b[^>]*>", "", sanitized, flags=re.I)
    sanitized = sanitized.replace("</br>", "<br/>").replace("<br>", "<br/>")
    sanitized = sanitized.replace("](./zh-cn/", "](/camera-sdk/vendor-vzense/zh-cn/")
    sanitized = sanitized.replace("](./en/", "](/camera-sdk/vendor-vzense/en/")
    sanitized = sanitized.replace("](zh-cn/", "](/camera-sdk/vendor-vzense/zh-cn/")
    sanitized = sanitized.replace("](en/", "](/camera-sdk/vendor-vzense/en/")
    sanitized = sanitized.replace("](/zh-cn/", "](/camera-sdk/vendor-vzense/zh-cn/")
    sanitized = sanitized.replace("](/en/", "](/camera-sdk/vendor-vzense/en/")
    return sanitized


def sanitize_vendor_markdown_tree(root: Path) -> None:
    for markdown_path in root.rglob("*.md"):
        raw_text = markdown_path.read_text(encoding="utf-8", errors="ignore")
        markdown_path.write_text(sanitize_markdown(raw_text), encoding="utf-8")


def main() -> None:
    if not SOURCE_ROOT.exists():
        raise SystemExit(f"missing source wiki: {SOURCE_ROOT}")

    if TARGET_ROOT.exists():
        shutil.rmtree(TARGET_ROOT)
    TARGET_ROOT.mkdir(parents=True, exist_ok=True)
    copy_tree(SOURCE_ROOT, TARGET_ROOT)
    sanitize_vendor_markdown_tree(TARGET_ROOT)


if __name__ == "__main__":
    main()
