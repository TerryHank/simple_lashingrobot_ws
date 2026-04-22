#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
HELP_ROOT = WORKSPACE_ROOT / "src" / "tie_robot_web" / "help"
WEB_ROOT = WORKSPACE_ROOT / "src" / "tie_robot_web" / "web"
README_ROOT = WORKSPACE_ROOT / "README.md"


class GiantBusinessStructuringTest(unittest.TestCase):
    def test_root_readme_covers_project_homepage_entry(self):
        self.assertTrue(README_ROOT.exists(), str(README_ROOT))
        readme = README_ROOT.read_text(encoding="utf-8")
        self.assertIn("Tie Robot Workspace", readme)
        self.assertIn("当前主包", readme)
        self.assertIn("新前端", readme)
        self.assertIn("3D Scene", readme)
        self.assertIn("Topic Layers", readme)
        self.assertIn("tie_robot_bringup", readme)
        self.assertIn("tie_robot_web", readme)
        self.assertIn("pseudo_slam_bind_path.json", readme)

    def test_help_site_source_scaffold_exists(self):
        expected = [
            HELP_ROOT / "package.json",
            HELP_ROOT / ".vitepress" / "config.mjs",
            HELP_ROOT / "index.md",
            HELP_ROOT / "guide" / "overview.md",
            HELP_ROOT / "guide" / "runtime-flows.md",
            HELP_ROOT / "guide" / "dev-entrypoints.md",
            HELP_ROOT / "reference" / "refactor-map.md",
            HELP_ROOT / "scripts" / "generate_structure_docs.py",
            HELP_ROOT / "scripts" / "sync_vzense_wiki.py",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

    def test_frontend_contains_help_entry(self):
        index_html = (WEB_ROOT / "index.html").read_text(encoding="utf-8")
        self.assertIn("/help/", index_html)

    def test_camera_sdk_help_is_integrated(self):
        expected = [
            HELP_ROOT / "camera-sdk" / "index.md",
            HELP_ROOT / "camera-sdk" / "vendor-vzense" / "README.md",
            HELP_ROOT / "camera-sdk" / "vendor-vzense" / "zh-cn" / "README.md",
            HELP_ROOT / "camera-sdk" / "vendor-vzense" / "zh-cn" / "ScepterSDK" / "BaseSDK.md",
            HELP_ROOT / "camera-sdk" / "vendor-vzense" / "zh-cn" / "ScepterSDK" / "3rd-Party-Plugin" / "ROS.md",
            HELP_ROOT / "camera-sdk" / "vendor-vzense" / "en" / "README.md",
            HELP_ROOT / "camera-sdk" / "vendor-vzense" / "en" / "ScepterSDK" / "BaseSDK.md",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        vitepress_config = (HELP_ROOT / ".vitepress" / "config.mjs").read_text(encoding="utf-8")
        self.assertIn("/camera-sdk/index", vitepress_config)
        self.assertIn("/camera-sdk/vendor-vzense/zh-cn/README", vitepress_config)

    def test_moduan_node_is_split_into_structured_modules(self):
        expected = [
            WORKSPACE_ROOT / "src" / "tie_robot_control" / "include" / "tie_robot_control" / "moduan" / "register_map.hpp",
            WORKSPACE_ROOT / "src" / "tie_robot_control" / "include" / "tie_robot_control" / "moduan" / "runtime_state.hpp",
            WORKSPACE_ROOT / "src" / "tie_robot_control" / "src" / "moduan" / "numeric_codec.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_control" / "src" / "moduan" / "linear_module_executor.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_control" / "src" / "moduan" / "moduan_ros_callbacks.cpp",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        moduan_entry = WORKSPACE_ROOT / "src" / "tie_robot_control" / "src" / "moduanNode.cpp"
        with moduan_entry.open(encoding="utf-8") as handle:
            line_count = sum(1 for _ in handle)
        self.assertLess(line_count, 40)

    def test_pointai_is_split_into_structured_modules(self):
        expected = [
            WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "tie_robot_perception" / "pointai" / "__init__.py",
            WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "tie_robot_perception" / "pointai" / "processor.py",
            WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "tie_robot_perception" / "pointai" / "runtime_config.py",
            WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "tie_robot_perception" / "pointai" / "world_coord.py",
            WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "tie_robot_perception" / "pointai" / "matrix_selection.py",
            WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "tie_robot_perception" / "pointai" / "process_image_service.py",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        pointai_entry = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "scripts" / "pointAI.py"
        with pointai_entry.open(encoding="utf-8") as handle:
            line_count = sum(1 for _ in handle)
        self.assertLess(line_count, 900)

    def test_scepter_manager_is_split_into_camera_modules(self):
        expected = [
            WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "camera" / "device_session.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "camera" / "frame_publish.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "camera" / "intrinsics.cpp",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        manager_cpp = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src" / "camera" / "scepter_manager.cpp"
        with manager_cpp.open(encoding="utf-8") as handle:
            line_count = sum(1 for _ in handle)
        self.assertLess(line_count, 400)

    def test_frontend_entry_is_split_into_web_modules(self):
        expected = [
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "web" / "modules" / "dom_refs.mjs",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "web" / "modules" / "ui_state.mjs",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "web" / "modules" / "canvas_renderer.mjs",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "web" / "modules" / "ros_connection.mjs",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "web" / "modules" / "execution_actions.mjs",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        frontend_entry = WORKSPACE_ROOT / "src" / "tie_robot_web" / "web" / "ir_workspace_picker.mjs"
        with frontend_entry.open(encoding="utf-8") as handle:
            line_count = sum(1 for _ in handle)
        self.assertLess(line_count, 320)

    def test_robot_viewer_style_frontend_scaffold_exists(self):
        expected = [
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "package.json",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "vite.config.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "index.html",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "main.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "app" / "TieRobotFrontApp.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "controllers" / "RosConnectionController.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "controllers" / "TaskActionController.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "controllers" / "LegacyCommandController.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "controllers" / "StatusMonitorController.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "ui" / "UIController.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "ui" / "PanelManager.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "views" / "WorkspaceCanvasView.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "config" / "legacyCommandCatalog.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "styles" / "app.css",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        built_entry = WORKSPACE_ROOT / "src" / "tie_robot_web" / "web" / "index.html"
        built_html = built_entry.read_text(encoding="utf-8")
        self.assertIn("Tie Robot Console", built_html)

    def test_frontend_3d_scene_and_topic_layers_scaffold_exists(self):
        expected = [
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "views" / "Scene3DView.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "controllers" / "TopicLayerController.js",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "config" / "topicLayerCatalog.js",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        package_json = (
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "package.json"
        ).read_text(encoding="utf-8")
        self.assertIn('"three"', package_json)

        ui_controller = (
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        self.assertIn("3D Scene", ui_controller)
        self.assertIn("Topic Layers", ui_controller)

        overview = (HELP_ROOT / "guide" / "overview.md").read_text(encoding="utf-8")
        dev_entrypoints = (HELP_ROOT / "guide" / "dev-entrypoints.md").read_text(encoding="utf-8")
        self.assertIn("3D Scene", overview)
        self.assertIn("Topic Layers", overview)
        self.assertIn("Scene3DView.js", dev_entrypoints)
        self.assertIn("/coordinate_point", dev_entrypoints)

    def test_frontend_uses_robot_viewer_style_background_layout(self):
        ui_controller = (
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")
        app_css = (
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "frontend" / "src" / "styles" / "app.css"
        ).read_text(encoding="utf-8")

        self.assertIn("scene-background", ui_controller)
        self.assertIn("top-toolbar", ui_controller)
        self.assertIn("data-toolbar-action", ui_controller)
        self.assertIn("workspacePanel", ui_controller)
        self.assertIn("topicLayersPanel", ui_controller)

        self.assertIn(".scene-background", app_css)
        self.assertIn(".top-toolbar", app_css)
        self.assertIn(".toolbar-pill", app_css)
        self.assertIn(".scene-overlay", app_css)

    def test_topics_transfer_is_split_into_bridge_modules(self):
        expected = [
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "include" / "tie_robot_web" / "web_bridge" / "topics_transfer_runtime.hpp",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "src" / "web_bridge" / "runtime.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "src" / "web_bridge" / "action_bridge.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "src" / "web_bridge" / "process_control.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "src" / "web_bridge" / "subscriber_callbacks.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_web" / "src" / "web_bridge" / "node_app.cpp",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        entry = WORKSPACE_ROOT / "src" / "tie_robot_web" / "src" / "topics_transfer.cpp"
        with entry.open(encoding="utf-8") as handle:
            line_count = sum(1 for _ in handle)
        self.assertLess(line_count, 40)

    def test_dynamic_bind_planning_is_split_into_planning_modules(self):
        expected = [
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "planning" / "dynamic_bind_planning_internal.hpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "planning" / "dynamic_bind_geometry.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "planning" / "dynamic_bind_grouping.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "planning" / "dynamic_bind_builder.cpp",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        entry = WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "planning" / "dynamic_bind_planning.cpp"
        with entry.open(encoding="utf-8") as handle:
            line_count = sum(1 for _ in handle)
        self.assertLess(line_count, 320)

    def test_suoqu_node_is_partially_structured_with_extracted_runtime_modules(self):
        expected = [
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "include" / "tie_robot_process" / "suoqu" / "cabin_transport.hpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoqu" / "cabin_transport.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoqu" / "planned_bind_and_memory.inc",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoqu" / "pseudo_slam_markers.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoqu" / "pseudo_slam_scan_processing.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoqu" / "service_orchestration.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoqu" / "area_execution.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoqu" / "execution_memory_store.cpp",
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoqu" / "bind_path_store.cpp",
        ]
        for path in expected:
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        entry = WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoquNode.cpp"
        with entry.open(encoding="utf-8") as handle:
            line_count = sum(1 for _ in handle)
        self.assertLess(line_count, 5000)

        planned_bind_include = (
            WORKSPACE_ROOT / "src" / "tie_robot_process" / "src" / "suoqu" / "planned_bind_and_memory.inc"
        )
        with planned_bind_include.open(encoding="utf-8") as handle:
            include_line_count = sum(1 for _ in handle)
        self.assertLess(include_line_count, 120)


if __name__ == "__main__":
    unittest.main()
