#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
BRINGUP_LAUNCH_DIR = WORKSPACE_ROOT / "src" / "tie_robot_bringup" / "launch"
BRINGUP_INCLUDE_RE = re.compile(
    r'<include\s+file="\$\(find tie_robot_bringup\)/launch(?P<name>/[^"]+)"'
)


def read_launch(name):
    return (BRINGUP_LAUNCH_DIR / name).read_text(encoding="utf-8")


def expand_bringup_launch_text(launch_text, seen=None):
    seen = set(seen or [])
    expanded = [launch_text]
    for match in BRINGUP_INCLUDE_RE.finditer(launch_text):
        include_name = match.group("name").lstrip("/")
        if include_name in seen:
            continue
        include_path = BRINGUP_LAUNCH_DIR / include_name
        if not include_path.exists():
            continue
        seen.add(include_name)
        expanded.append(expand_bringup_launch_text(include_path.read_text(encoding="utf-8"), seen))
    return "\n".join(expanded)


def node_names(launch_text):
    return set(re.findall(r'<node\s+name="([^"]+)"', launch_text))


def node_types(launch_text):
    return {
        match.group(1): match.group(2)
        for match in re.finditer(r'<node\s+name="([^"]+)"\s+pkg="[^"]+"\s+type="([^"]+)"', launch_text)
    }


class NavigationStackSplitTest(unittest.TestCase):
    def test_driver_stack_contains_only_driver_guard_nodes(self):
        launch_text = expand_bringup_launch_text(read_launch("driver_stack.launch"))
        names = node_names(launch_text)
        types = node_types(launch_text)

        self.assertIn("suoqu_driver_node", names)
        self.assertIn("moduan_driver_node", names)
        self.assertIn("scepter_camera.launch", launch_text)
        self.assertEqual("suoqu_driver_node", types["suoqu_driver_node"])
        self.assertEqual("moduan_driver_node", types["moduan_driver_node"])
        self.assertNotIn("robot_tf_broadcaster", names)
        self.assertNotIn("gripper_tf_broadcaster", names)
        self.assertNotIn("suoquNode", names)
        self.assertNotIn("moduanNode", names)
        self.assertNotIn("cabin_motion_controller", names)
        self.assertNotIn("moduan_motion_controller", names)
        self.assertNotIn('name="node_role"', launch_text)

    def test_rosbridge_stack_includes_guarded_tf_stack(self):
        launch_text = read_launch("rosbridge_stack.launch")
        expanded_launch_text = expand_bringup_launch_text(launch_text)
        names = node_names(expanded_launch_text)
        types = node_types(expanded_launch_text)

        self.assertIn('$(find tie_robot_bringup)/launch/tf_stack.launch', launch_text)
        self.assertIn('$(find tie_robot_bringup)/launch/api.launch', launch_text)
        self.assertIn("tf2_web_republisher", names)
        self.assertEqual("tf2_web_republisher", types["tf2_web_republisher"])
        self.assertIn("robot_tf_broadcaster", names)
        self.assertEqual("robot_tf_broadcaster.py", types["robot_tf_broadcaster"])
        self.assertIn("gripper_tf_broadcaster", names)
        self.assertEqual("gripper_tf_broadcaster.py", types["gripper_tf_broadcaster"])
        self.assertIn("web_action_bridge_node", names)
        self.assertIn("system_log_mux", names)
        self.assertIn("/cabin/cabin_data_upload", expanded_launch_text)
        self.assertIn("config/gripper_tf.yaml", expanded_launch_text)
        self.assertIn('respawn="true"', expanded_launch_text)

    def test_run_launch_is_algorithm_backend_only(self):
        launch_text = read_launch("run.launch")

        self.assertIn('$(find tie_robot_bringup)/launch/algorithm_stack.launch', launch_text)
        self.assertNotIn('$(find tie_robot_bringup)/launch/api.launch', launch_text)
        self.assertNotIn('$(find tie_robot_bringup)/launch/driver_stack.launch', launch_text)
        self.assertNotIn('$(find tie_robot_bringup)/launch/tf_stack.launch', launch_text)

    def test_tf_stack_contains_guarded_tf_nodes(self):
        launch_text = read_launch("tf_stack.launch")
        names = node_names(launch_text)
        types = node_types(launch_text)

        self.assertIn("tf2_web_republisher", names)
        self.assertEqual("tf2_web_republisher", types["tf2_web_republisher"])
        self.assertIn("robot_tf_broadcaster", names)
        self.assertEqual("robot_tf_broadcaster.py", types["robot_tf_broadcaster"])
        self.assertIn("gripper_tf_broadcaster", names)
        self.assertEqual("gripper_tf_broadcaster.py", types["gripper_tf_broadcaster"])
        self.assertIn("/cabin/cabin_data_upload", launch_text)
        self.assertIn("config/gripper_tf.yaml", launch_text)
        self.assertIn('respawn="true"', launch_text)

    def test_algorithm_stack_contains_motion_and_planning_nodes(self):
        launch_text = read_launch("algorithm_stack.launch")
        names = node_names(launch_text)
        types = node_types(launch_text)

        expected_nodes = {
            "pointAINode",
            "bind_map_builder",
            "global_bind_planner",
            "cabin_motion_controller",
            "moduan_motion_controller",
            "bind_task_executor",
        }
        self.assertTrue(expected_nodes.issubset(names))
        self.assertNotIn("suoquNode", names)
        self.assertNotIn("moduanNode", names)
        self.assertEqual("global_bind_planner_node.py", types["global_bind_planner"])
        self.assertEqual("cabin_motion_controller_node", types["cabin_motion_controller"])
        self.assertEqual("moduan_motion_controller_node", types["moduan_motion_controller"])
        self.assertEqual("bind_task_executor_node", types["bind_task_executor"])
        self.assertNotIn('name="node_role"', launch_text)


if __name__ == "__main__":
    unittest.main()
