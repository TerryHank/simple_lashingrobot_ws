import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "tie_robot_vision"
TARGET_WS = Path(
    "/home/hyq-/simple_lashingrobot_show/"
    "simple_lashingrobot_ws20260403/simple_lashingrobot_ws"
)


class TieRobotVisionPackageTest(unittest.TestCase):
    def test_portable_vision_package_has_required_surface(self):
        required = [
            PKG / "package.xml",
            PKG / "CMakeLists.txt",
            PKG / "msg" / "PointCoords.msg",
            PKG / "msg" / "PointsArray.msg",
            PKG / "srv" / "ProcessImage.srv",
            PKG / "srv" / "SetGripperTfCalibration.srv",
            PKG / "srv" / "RobotHomeCalibration.srv",
            PKG / "scripts" / "pointai_node.py",
            PKG / "scripts" / "gripper_tf_broadcaster.py",
            PKG / "scripts" / "robot_tf_broadcaster.py",
            PKG / "launch" / "vision_stack.launch",
            PKG / "launch" / "legacy_20260403_vision.launch",
            PKG / "docs" / "移植交接说明.md",
        ]
        missing = [str(path.relative_to(ROOT)) for path in required if not path.exists()]
        self.assertEqual([], missing)

    def test_legacy_launch_documents_old_workspace_path_and_compat_service(self):
        launch_text = (PKG / "launch" / "legacy_20260403_vision.launch").read_text(encoding="utf-8")
        self.assertIn("/pointAI/process_image", launch_text)
        self.assertIn("/Moduan/process_image", launch_text)
        self.assertIn("legacy_chassis_state_relay.py", launch_text)
        self.assertIn("legacy_moduan_state_relay.py", launch_text)
        doc_text = (PKG / "docs" / "移植交接说明.md").read_text(encoding="utf-8")
        self.assertIn(str(TARGET_WS), doc_text)
        self.assertIn("20260403", doc_text)

    def test_legacy_state_relay_scripts_are_installed(self):
        cmake_text = (PKG / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/legacy_chassis_state_relay.py", cmake_text)
        self.assertIn("scripts/legacy_moduan_state_relay.py", cmake_text)

    def test_target_workspace_receives_portable_package_after_adaptation(self):
        target_pkg = TARGET_WS / "src" / "tie_robot_vision"
        self.assertTrue((target_pkg / "package.xml").exists())
        self.assertTrue((target_pkg / "launch" / "legacy_20260403_vision.launch").exists())
        self.assertTrue((target_pkg / "docs" / "移植交接说明.md").exists())


if __name__ == "__main__":
    unittest.main()
