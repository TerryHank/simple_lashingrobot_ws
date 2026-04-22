#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
CHASSIS_CTRL_DIR = WORKSPACE_ROOT / "chassis_ctrl"
FRONTEND_DIR = WORKSPACE_ROOT / "ir_workspace_picker_web"


class WorkspacePickerWebLaunchTest(unittest.TestCase):
    def test_workspace_picker_frontend_lives_outside_app_directory(self):
        self.assertTrue((FRONTEND_DIR / "index.html").exists())
        self.assertTrue((FRONTEND_DIR / "ir_workspace_picker.mjs").exists())
        self.assertTrue((FRONTEND_DIR / "ir_workspace_picker_helpers.mjs").exists())
        self.assertTrue((FRONTEND_DIR / "vendor" / "roslib-CFvqKDwv.js").exists())
        self.assertTrue((FRONTEND_DIR / "vendor" / "rolldown-runtime-CeubTPu9.js").exists())
        self.assertFalse((WORKSPACE_ROOT / "APP" / "dist" / "ir_workspace_picker.html").exists())

    def test_run_launch_starts_api_stack_and_workspace_picker_web(self):
        launch_text = (CHASSIS_CTRL_DIR / "launch" / "run.launch").read_text(encoding="utf-8")

        self.assertIn('<arg name="include_api_stack" default="true"', launch_text)
        self.assertIn('<include file="$(find chassis_ctrl)/launch/api.launch"', launch_text)
        self.assertIn('name="workspace_picker_web_server"', launch_text)
        self.assertIn('type="workspace_picker_web_server.py"', launch_text)
        self.assertIn('name="workspace_picker_web_open"', launch_text)
        self.assertIn('type="workspace_picker_web_open.py"', launch_text)

    def test_workspace_picker_web_scripts_reference_independent_frontend_directory(self):
        server_script = (CHASSIS_CTRL_DIR / "scripts" / "workspace_picker_web_server.py").read_text(encoding="utf-8")
        open_script = (CHASSIS_CTRL_DIR / "scripts" / "workspace_picker_web_open.py").read_text(encoding="utf-8")
        picker_script = (FRONTEND_DIR / "ir_workspace_picker.mjs").read_text(encoding="utf-8")
        picker_html = (FRONTEND_DIR / "index.html").read_text(encoding="utf-8")

        self.assertIn('ir_workspace_picker_web', server_script)
        self.assertIn('SimpleHTTPRequestHandler', server_script)
        self.assertIn('http://127.0.0.1', open_script)
        self.assertIn('ir_workspace_picker_web/index.html', open_script)
        self.assertIn("./vendor/roslib-CFvqKDwv.js", picker_script)
        self.assertNotIn("../APP/dist/assets/roslib-CFvqKDwv.js", picker_script)
        self.assertIn("/web/pointAI/run_workspace_s2", picker_script)
        self.assertIn("/web/cabin/start_pseudo_slam_scan", picker_script)
        self.assertIn("/web/cabin/set_execution_mode", picker_script)
        self.assertIn("/web/cabin/start_global_work", picker_script)
        self.assertIn("/pointAI/manual_workspace_s2_result_raw", picker_script)
        self.assertIn("/pointAI/result_image_raw", picker_script)
        self.assertIn("runS2Publisher.publish", picker_script)
        self.assertIn('id="runSavedS2"', picker_html)
        self.assertIn('id="moveToWorkspaceCenterScanPose"', picker_html)
        self.assertIn('id="startExecution"', picker_html)
        self.assertIn('id="startExecutionClearMemory"', picker_html)
        self.assertIn("直接识别绑扎点", picker_html)
        self.assertIn("移动到固定扫描位姿并扫描规划", picker_html)
        self.assertIn("开始执行层", picker_html)
        self.assertIn("清记忆并开始执行层", picker_html)
        self.assertIn("const runSavedS2Btn = document.getElementById('runSavedS2');", picker_script)
        self.assertIn("const moveToWorkspaceCenterScanPoseBtn = document.getElementById('moveToWorkspaceCenterScanPose');", picker_script)
        self.assertIn("const startExecutionBtn = document.getElementById('startExecution');", picker_script)
        self.assertIn("const startExecutionClearMemoryBtn = document.getElementById('startExecutionClearMemory');", picker_script)
        self.assertIn("function triggerSavedWorkspaceS2()", picker_script)
        self.assertIn("function triggerWorkspaceCenterScanPoseMove()", picker_script)
        self.assertIn("pseudoSlamScanPublisher.publish(new ROSLIB.Message({ data: 5.0 }));", picker_script)
        self.assertIn("function triggerExecutionLayer(clearExecutionMemory = false)", picker_script)
        self.assertIn("overlaySource = 'execution';", picker_script)
        self.assertIn("executionModePublisher.publish(new ROSLIB.Message({ data: 1.0 }));", picker_script)
        self.assertIn("startGlobalWorkPublisher.publish(", picker_script)
        self.assertIn("function handleCanvasPointerDown(", picker_script)
        self.assertIn("function handleCanvasPointerMove(", picker_script)
        self.assertIn("function handleCanvasPointerUp(", picker_script)
        self.assertIn("canvas.addEventListener('pointerdown', handleCanvasPointerDown);", picker_script)
        self.assertIn("canvas.addEventListener('pointermove', handleCanvasPointerMove);", picker_script)
        self.assertIn("canvas.addEventListener('pointerup', handleCanvasPointerUp);", picker_script)
        self.assertIn("savedWorkspacePoints.length === 4", picker_script)
        self.assertNotIn('id="resultCanvas"', picker_html)
        self.assertIn('id="s2OverlayCanvas"', picker_html)
        self.assertIn('id="overlayOpacityRange"', picker_html)
        self.assertIn('IR 选点图', picker_html)
        self.assertIn('drawS2Overlay', picker_script)
        self.assertIn('applyOverlayOpacity', picker_script)
        self.assertIn('mix-blend-mode: screen', picker_html)
        self.assertIn("overlaySource === 'execution' ? lastExecutionResultImageMessage : lastResultImageMessage", picker_script)


if __name__ == "__main__":
    unittest.main()
