#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
BRINGUP_LAUNCH_DIR = WORKSPACE_ROOT / "src" / "tie_robot_bringup" / "launch"
CONTROL_ROOT = WORKSPACE_ROOT / "src" / "tie_robot_control"
PROCESS_ROOT = WORKSPACE_ROOT / "src" / "tie_robot_process"
BRINGUP_INCLUDE_RE = re.compile(
    r'<include\s+file="\$\(find tie_robot_bringup\)/launch/(?P<name>[^"]+)"'
)


def read(path):
    return path.read_text(encoding="utf-8")


def expand_bringup_launch_text(launch_text, seen=None):
    seen = set(seen or [])
    expanded = [launch_text]
    for match in BRINGUP_INCLUDE_RE.finditer(launch_text):
        include_name = match.group("name")
        if include_name in seen:
            continue
        include_path = BRINGUP_LAUNCH_DIR / include_name
        if not include_path.exists():
            continue
        seen.add(include_name)
        expanded.append(expand_bringup_launch_text(read(include_path), seen))
    return "\n".join(expanded)


def node_attrs(launch_text):
    return {
        match.group("name"): {
            "pkg": match.group("pkg"),
            "type": match.group("type"),
            "body": "",
        }
        for match in re.finditer(
            r'<node\s+name="(?P<name>[^"]+)"\s+pkg="(?P<pkg>[^"]+)"\s+type="(?P<type>[^"]+)"',
            launch_text,
        )
    }


class DriverAlgorithmNodeBoundariesTest(unittest.TestCase):
    def test_launch_uses_dedicated_executables_instead_of_node_role_multiplexing(self):
        driver_launch = expand_bringup_launch_text(read(BRINGUP_LAUNCH_DIR / "driver_stack.launch"))
        algorithm_launch = read(BRINGUP_LAUNCH_DIR / "algorithm_stack.launch")
        launch_text = driver_launch + "\n" + algorithm_launch
        nodes = node_attrs(launch_text)

        expected_types = {
            "suoqu_driver_node": "suoqu_driver_node",
            "moduan_driver_node": "moduan_driver_node",
            "cabin_motion_controller": "cabin_motion_controller_node",
            "moduan_motion_controller": "moduan_motion_controller_node",
            "bind_task_executor": "bind_task_executor_node",
            "global_bind_planner": "global_bind_planner_node.py",
        }
        for node_name, expected_type in expected_types.items():
            with self.subTest(node=node_name):
                self.assertIn(node_name, nodes)
                self.assertEqual(expected_type, nodes[node_name]["type"])

        self.assertNotIn('name="node_role"', launch_text)
        self.assertNotIn('type="suoquNode"', launch_text)
        self.assertNotIn('type="moduanNode"', launch_text)
        self.assertNotIn('type="architecture_role_node.py"', nodes["global_bind_planner"]["type"])

    def test_bringup_launches_do_not_start_compat_combined_nodes(self):
        launch_text = "\n".join(read(path) for path in BRINGUP_LAUNCH_DIR.glob("*.launch"))

        self.assertNotIn('type="suoquNode"', launch_text)
        self.assertNotIn('type="moduanNode"', launch_text)
        self.assertNotIn('name="node_role"', launch_text)

    def test_cmake_builds_separate_driver_motion_and_executor_binaries(self):
        control_cmake = read(CONTROL_ROOT / "CMakeLists.txt")
        process_cmake = read(PROCESS_ROOT / "CMakeLists.txt")

        for executable_name in [
            "moduanNode",
            "moduan_driver_node",
            "moduan_motion_controller_node",
        ]:
            with self.subTest(executable=executable_name):
                self.assertRegex(control_cmake, rf"add_executable\({executable_name}\b")

        for executable_name in [
            "suoquNode",
            "suoqu_driver_node",
            "cabin_motion_controller_node",
            "bind_task_executor_node",
        ]:
            with self.subTest(executable=executable_name):
                self.assertRegex(process_cmake, rf"add_executable\({executable_name}\b")

    def test_global_bind_planner_publishes_standard_nav_path(self):
        process_cmake = read(PROCESS_ROOT / "CMakeLists.txt")
        package_xml = read(PROCESS_ROOT / "package.xml")
        planner_script = PROCESS_ROOT / "scripts" / "global_bind_planner_node.py"

        self.assertIn("nav_msgs", process_cmake)
        self.assertIn("<build_depend>nav_msgs</build_depend>", package_xml)
        self.assertTrue(planner_script.exists(), str(planner_script))

        planner_text = read(planner_script)
        self.assertIn("from nav_msgs.msg import Path", planner_text)
        self.assertIn("/cabin/global_bind_path", planner_text)
        self.assertIn("frame_id = \"map\"", planner_text)


if __name__ == "__main__":
    unittest.main()
