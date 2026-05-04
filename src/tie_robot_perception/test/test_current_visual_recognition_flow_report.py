#!/usr/bin/env python3

import unittest
from html.parser import HTMLParser
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
REPORT_DIR = REPO_ROOT / "docs" / "reports" / "current_visual_recognition_flow"
REPORT_PATH = REPORT_DIR / "index.html"
GENERATOR_PATH = (
    REPO_ROOT
    / "src"
    / "tie_robot_perception"
    / "tools"
    / "build_current_visual_recognition_flow_page.py"
)


class _ImageSourceParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.image_sources = []

    def handle_starttag(self, tag, attrs):
        if tag != "img":
            return
        attrs_by_name = dict(attrs)
        source = attrs_by_name.get("src")
        if source:
            self.image_sources.append(source)


class CurrentVisualRecognitionFlowReportTest(unittest.TestCase):
    def test_report_page_documents_current_visual_steps_with_existing_images(self):
        self.assertTrue(GENERATOR_PATH.exists(), "缺少当前视觉流程效果页生成脚本。")
        self.assertTrue(REPORT_PATH.exists(), "缺少当前视觉流程效果页 index.html。")

        report_html = REPORT_PATH.read_text(encoding="utf-8")
        for required_text in (
            "当前视觉识别流程效果图",
            "扫描识别：2026-04-22 PR-FPRG",
            "执行微调：平面分割 + Hough",
            "depth-only 背景差分",
            "纵横 profile 周期相位",
            "透视网格反投影",
            "不包含 depth+IR 组合响应、方向线族或梁筋 ±13 cm 过滤",
        ):
            self.assertIn(required_text, report_html)

        parser = _ImageSourceParser()
        parser.feed(report_html)
        self.assertGreaterEqual(len(parser.image_sources), 12)
        for image_source in parser.image_sources:
            self.assertFalse(image_source.startswith(("http://", "https://")))
            image_path = (REPORT_DIR / image_source).resolve()
            self.assertTrue(
                str(image_path).startswith(str(REPORT_DIR.resolve())),
                f"图片路径越界：{image_source}",
            )
            self.assertTrue(image_path.exists(), f"报告引用的图片不存在：{image_source}")


if __name__ == "__main__":
    unittest.main()
