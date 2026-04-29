#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROCESS_DIR = WORKSPACE_ROOT / "tie_robot_process"
CONTROL_DIR = WORKSPACE_ROOT / "tie_robot_control"
MSGS_DIR = WORKSPACE_ROOT / "tie_robot_msgs"


class MotionChainSignalGuardTest(unittest.TestCase):
    def test_moduan_state_topic_and_action_interfaces_are_declared(self):
        cmake = (MSGS_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        state_msg = (MSGS_DIR / "msg" / "ModuanState.msg").read_text(encoding="utf-8")
        action = (
            MSGS_DIR / "action" / "ExecuteBindPointsTask.action"
        ).read_text(encoding="utf-8")

        self.assertIn("ModuanState.msg", cmake)
        self.assertIn("ExecuteBindPointsTask.action", cmake)
        self.assertIn("bool executing", state_msg)
        self.assertIn("bool finish_all", state_msg)
        self.assertIn("PointCoords[] points", action)
        self.assertIn("string phase", action)
        self.assertIn("bool success", action)

    def test_cabin_motion_driver_rejects_moves_while_plc_execution_signal_is_active(self):
        runtime_header = (
            PROCESS_DIR / "src" / "suoqu" / "suoqu_runtime_internal.hpp"
        ).read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        transport = (
            PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("extern std::atomic<bool> moduan_work_flag;", runtime_header)
        self.assertIn("std::atomic<bool> moduan_work_flag{false};", node)
        self.assertIn("moduan_work_flag.store(debug_mes.data", node)

        self.assertIn("moduan_work_flag.load(", transport)
        guard_index = transport.index("moduan_work_flag.load(")
        remote_call_index = transport.index('ros::service::call("/cabin/driver/raw_move"', guard_index)
        driver_call_index = transport.index("::g_cabin_driver->moveToPose", guard_index)
        self.assertLess(guard_index, remote_call_index)
        self.assertLess(guard_index, driver_call_index)
        self.assertIn("末端绑扎/线性模组正在运动", transport[guard_index:driver_call_index])

    def test_moduan_services_do_not_set_busy_before_plc_execution(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertNotIn("class ScopedModuanWorkState", callbacks)
        self.assertNotIn("ScopedModuanWorkState work_state;", callbacks)
        self.assertNotIn("reject_if_cabin_motion_active", callbacks)

        for function_name in (
            "moduan_bind_service",
            "moduan_bind_points_service",
            "moduan_bind_points_fast_service",
            "moduan_move_service",
        ):
            start = callbacks.index(f"bool {function_name}(")
            next_function = callbacks.find("\nbool ", start + 5)
            if next_function == -1:
                next_function = len(callbacks)
            body = callbacks[start:next_function]
            self.assertIn("std::lock_guard<std::mutex> lashing_lock(lashing_mutex);", body)
            self.assertNotIn("pub_moduan_work_state(true);", body)
            self.assertNotIn("pub_moduan_work_state(false);", body)

        single_bind_start = callbacks.index("bool moduan_bind_service(")
        single_bind_end = callbacks.index("\nbool moduan_bind_points_service", single_bind_start)
        single_bind_body = callbacks[single_bind_start:single_bind_end]
        self.assertLess(single_bind_body.index("AI_client.call(srv)"), single_bind_body.index("execute_bind_points("))

    def test_moduan_work_topic_mirrors_plc_execution_until_finishall(self):
        executor = (
            CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("wait_for_plc_finish_all", executor)
        self.assertIn("kFinishAllPollInterval", executor)
        self.assertNotIn("finish_all(150)", executor)
        self.assertIn("ScopedPlcExecutionState", executor)
        self.assertIn("pub_moduan_work_state(true);", executor)
        self.assertIn("pub_moduan_work_state(false);", executor)

        execute_start = executor.index("bool execute_bind_points(")
        execute_body = executor[execute_start:]
        write_points_index = execute_body.index("writeQueuedPoints")
        work_state_index = execute_body.index("ScopedPlcExecutionState plc_execution_state;", write_points_index)
        trigger_index = execute_body.index("pulseExecutionEnable")
        finishall_index = execute_body.index("wait_for_plc_finish_all(", work_state_index)
        self.assertLess(write_points_index, work_state_index)
        self.assertLess(work_state_index, trigger_index)
        self.assertLess(trigger_index, finishall_index)
        self.assertLess(work_state_index, finishall_index)

        finish_all_start = executor.index("bool wait_for_plc_finish_all(")
        finish_all_end = executor.index("\nvoid moveLinearModule", finish_all_start)
        finish_all_body = executor[finish_all_start:finish_all_end]
        self.assertIn("FINISH_ALL_FLAG", finish_all_body)
        self.assertIn("PLC_Order_Write(FINISHALL, 0, plc);", finish_all_body)

    def test_moduan_driver_publishes_standard_state_topic(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("#include \"tie_robot_msgs/ModuanState.h\"", callbacks)
        self.assertIn("publish_moduan_state_topic", callbacks)
        self.assertIn(
            'nh_.advertise<tie_robot_msgs::ModuanState>("/moduan/state"',
            callbacks,
        )
        self.assertIn("state_msg.executing", callbacks)
        self.assertIn("state_msg.finish_all", callbacks)
        self.assertIn("state->FINISH_ALL_FLAG", callbacks)
        self.assertIn("pub_moduan_state_topic.publish(state_msg);", callbacks)

    def test_moduan_execute_bind_points_action_is_advertised(self):
        callbacks = (
            CONTROL_DIR / "src" / "moduan" / "moduan_ros_callbacks.cpp"
        ).read_text(encoding="utf-8")
        cmake = (CONTROL_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        package_xml = (CONTROL_DIR / "package.xml").read_text(encoding="utf-8")

        self.assertIn("#include <actionlib/server/simple_action_server.h>", callbacks)
        self.assertIn("ExecuteBindPointsTaskAction", callbacks)
        self.assertIn("execute_bind_points_action_callback", callbacks)
        self.assertIn('"/moduan/execute_bind_points"', callbacks)
        self.assertIn("g_execute_bind_points_action_server->start();", callbacks)
        self.assertIn("setSucceeded", callbacks)
        self.assertIn("setAborted", callbacks)
        self.assertIn("actionlib", cmake)
        self.assertIn("<build_depend>actionlib</build_depend>", package_xml)

    def test_process_layer_dispatches_precomputed_bind_points_through_action(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        cmake = (PROCESS_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        package_xml = (PROCESS_DIR / "package.xml").read_text(encoding="utf-8")

        self.assertIn("#include <actionlib/client/simple_action_client.h>", node)
        self.assertIn("ExecuteBindPointsTaskAction", node)
        self.assertIn("execute_moduan_bind_points_via_action", node)
        self.assertIn('"/moduan/execute_bind_points"', node)
        self.assertNotIn("sg_precomputed_client.call(", node)
        self.assertNotIn("sg_precomputed_fast_client.call(", node)
        self.assertIn("actionlib", cmake)
        self.assertIn("<build_depend>actionlib</build_depend>", package_xml)


if __name__ == "__main__":
    unittest.main()
