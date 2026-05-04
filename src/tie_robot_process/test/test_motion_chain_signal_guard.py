#!/usr/bin/env python3

import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
PROCESS_DIR = WORKSPACE_ROOT / "tie_robot_process"
CONTROL_DIR = WORKSPACE_ROOT / "tie_robot_control"
HW_DIR = WORKSPACE_ROOT / "tie_robot_hw"
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

    def test_cabin_stop_treats_peer_close_after_stop_frame_as_delivered(self):
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")
        protocol = (HW_DIR / "src" / "driver" / "cabin_protocol.cpp").read_text(encoding="utf-8")

        self.assertIn("std::vector<uint8_t> CabinProtocol::buildStopFrame()", protocol)
        self.assertIn("0x13", protocol)

        start = driver.index("bool CabinDriver::sendStop(")
        end = driver.index("\nCabinStateSnapshot CabinDriver::readState", start)
        body = driver[start:end]
        self.assertIn("CabinProtocol::buildStopFrame()", body)
        self.assertIn('driver_error.code == "tcp_recv_failed"', body)
        self.assertIn('driver_error.detail.find("connection closed by peer")', body)
        self.assertNotIn('driver_error.detail == "connection closed by peer"', body)
        self.assertIn("transport_->disconnect();", body)
        self.assertIn("error->clear();", body)

    def test_cabin_driver_keeps_tcp_position_relative_frame_for_optional_remote_mode(self):
        protocol_header = (HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_protocol.hpp").read_text(encoding="utf-8")
        protocol = (HW_DIR / "src" / "driver" / "cabin_protocol.cpp").read_text(encoding="utf-8")
        driver_header = (HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_driver.hpp").read_text(encoding="utf-8")
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
        topic_registry = (
            WORKSPACE_ROOT / "tie_robot_web" / "frontend" / "src" / "config" / "topicRegistry.js"
        ).read_text(encoding="utf-8")

        self.assertIn("buildRelativeMoveFrame", protocol_header)
        self.assertIn("std::vector<uint8_t> CabinProtocol::buildIncrementalMoveFrame", protocol)
        self.assertIn("std::vector<uint8_t> CabinProtocol::buildRelativeMoveFrame", protocol)
        self.assertIn("return buildPositionMoveFrame(command, 0x0002);", protocol)
        self.assertIn("frame.push_back(0x11);", protocol)
        self.assertIn("bool moveByOffset(const CabinPoseCommand& command, DriverError* error);", driver_header)
        start = driver.index("bool CabinDriver::moveByOffset(")
        end = driver.index("\nbool CabinDriver::sendStop", start)
        body = driver[start:end]
        self.assertIn("CabinProtocol::buildRelativeMoveFrame(command)", body)
        self.assertIn("decodeStatus(0x0012", body)
        self.assertNotIn("CabinProtocol::buildIncrementalMoveFrame(command)", body)
        self.assertNotIn("decodeStatus(0x0011", body)
        self.assertIn('nh.advertiseService("/cabin/driver/incremental_move"', node)
        self.assertIn('incrementalMove: "/cabin/driver/incremental_move"', topic_registry)

    def test_cabin_driver_resets_socket_after_desynchronized_motion_response(self):
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")

        self.assertIn("resetTransportAfterProtocolDesync", driver)
        helper_start = driver.index("void resetTransportAfterProtocolDesync(")
        helper_end = driver.index("\nCabinDriver::CabinDriver", helper_start)
        helper_body = driver[helper_start:helper_end]
        self.assertIn('protocol_error.code != "protocol_response_desynchronized"', helper_body)
        self.assertIn("transport.disconnect();", helper_body)
        self.assertIn("避免TCP相对运动重复执行", helper_body)

        start = driver.index("bool CabinDriver::moveByOffset(")
        end = driver.index("\nbool CabinDriver::sendStop", start)
        body = driver[start:end]
        self.assertIn("resetTransportAfterProtocolDesync(*transport_, protocol_error);", body)
        self.assertNotIn("for (int attempt", body)

    def test_cabin_tcp_transport_errors_include_request_frame_context(self):
        transport = (HW_DIR / "src" / "driver" / "cabin_tcp_transport.cpp").read_text(encoding="utf-8")

        self.assertIn("std::string formatRequestContext(const std::vector<uint8_t>& request)", transport)
        self.assertIn("request_command=", transport)
        self.assertIn("request_frame=[", transport)
        send_start = transport.index('"tcp_send_failed"')
        recv_start = transport.index('"tcp_recv_failed"')
        wait_start = transport.index('"tcp_read_wait_failed"')
        self.assertIn("appendRequestContext(", transport[send_start:send_start + 500])
        self.assertIn("appendRequestContext(", transport[wait_start:wait_start + 500])
        self.assertIn("appendRequestContext(", transport[recv_start:recv_start + 500])

    def test_cabin_tcp_transport_drains_stale_state_bytes_before_motion_send(self):
        transport_header = (
            HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_tcp_transport.hpp"
        ).read_text(encoding="utf-8")
        transport = (HW_DIR / "src" / "driver" / "cabin_tcp_transport.cpp").read_text(encoding="utf-8")

        self.assertIn("std::size_t drainPendingInputLocked();", transport_header)
        self.assertIn("std::size_t CabinTcpTransport::drainPendingInputLocked()", transport)
        self.assertIn("bool receiveExactLocked(", transport_header)
        self.assertIn("bool CabinTcpTransport::receiveExactLocked(", transport)
        self.assertIn("MSG_DONTWAIT", transport)
        self.assertIn("looksLikeCabinStateResponsePrefix", transport)
        self.assertIn("kCabinStateResponseBytes - kCabinMotionResponseBytes", transport)

        send_start = transport.index("bool CabinTcpTransport::sendAndReceive(")
        send_body = transport[send_start:transport.index("\nvoid CabinTcpTransport::markExternalIoSuccess", send_start)]
        drain_index = send_body.index("drainPendingInputLocked();")
        send_index = send_body.index("::send(")
        read_index = send_body.index("receiveExactLocked(expected_bytes")
        skip_index = send_body.index("looksLikeCabinStateResponsePrefix")
        self.assertLess(drain_index, send_index)
        self.assertLess(send_index, read_index)
        self.assertLess(read_index, skip_index)

    def test_legacy_suoqu_tcp_position_status_decoder_uses_32_bit_dictionary(self):
        header = (
            PROCESS_DIR / "include" / "tie_robot_process" / "suoqu" / "cabin_transport.hpp"
        ).read_text(encoding="utf-8")
        transport = (
            PROCESS_DIR / "src" / "suoqu" / "cabin_transport.cpp"
        ).read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("std::atomic<uint32_t> pending_tcp_status_word", header)
        self.assertIn("uint32_t status_word = 0;", header)
        self.assertIn("uint32_t extract_tcp_protocol_status_word", header)
        self.assertIn(
            'append_tcp_status_reason_if_set(reasons, status_word, 16, "C超负限位")',
            transport,
        )

        extract_start = transport.index("uint32_t extract_tcp_protocol_status_word(")
        extract_end = transport.index("\nTcpProtocolStatusDecode decode_tcp_protocol_status", extract_start)
        extract_body = transport[extract_start:extract_end]
        position_case = extract_body[extract_body.index("case 0x0012:"):]
        self.assertIn("(static_cast<uint32_t>(buffer[2]) << 24)", position_case)
        self.assertIn("(static_cast<uint32_t>(buffer[3]) << 16)", position_case)
        self.assertIn("(static_cast<uint32_t>(buffer[4]) << 8)", position_case)
        self.assertIn("static_cast<uint32_t>(buffer[5])", position_case)
        self.assertNotIn("static_cast<uint16_t>(buffer[4])", position_case)

        self.assertIn("std::atomic<uint32_t> pending_tcp_status_word{0};", node)
        self.assertIn("status_word=0x%08X", node)
        self.assertIn("状态字异常0x%08X", node)

    def test_cabin_protocol_retry_refreshes_socket_after_reconnect(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        start = node.index("int Frame_Generate_With_Retry(")
        end = node.index("\nvoid solve_stop", start)
        body = node[start:end]
        invalid_socket_guard_index = body.index("if (socket < 0)")
        first_frame_generate_index = body.index("Frame_Generate(Control_Word")
        reconnect_index = body.index("connectToServer()")
        refresh_index = body.index("socket = sockfd;", reconnect_index)
        resend_index = body.index("正在尝试重新发送指令", reconnect_index)
        self.assertLess(invalid_socket_guard_index, first_frame_generate_index)
        self.assertLess(reconnect_index, refresh_index)
        self.assertLess(refresh_index, resend_index)

    def test_cabin_state_poll_clears_transport_timeout_after_successful_heartbeat(self):
        transport_header = (
            HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_tcp_transport.hpp"
        ).read_text(encoding="utf-8")
        transport = (HW_DIR / "src" / "driver" / "cabin_tcp_transport.cpp").read_text(encoding="utf-8")
        driver_header = (
            HW_DIR / "include" / "tie_robot_hw" / "driver" / "cabin_driver.hpp"
        ).read_text(encoding="utf-8")
        driver = (HW_DIR / "src" / "driver" / "cabin_driver.cpp").read_text(encoding="utf-8")
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("void markExternalIoSuccess();", transport_header)
        self.assertIn("void CabinTcpTransport::markExternalIoSuccess()", transport)
        mark_start = transport.index("void CabinTcpTransport::markExternalIoSuccess()")
        mark_end = transport.index("\nConnectionState CabinTcpTransport::connectionState", mark_start)
        mark_body = transport[mark_start:mark_end]
        self.assertIn("ConnectionState::kReady", mark_body)
        self.assertIn("last_error_text_.clear();", mark_body)

        self.assertIn("void markExternalIoSuccess();", driver_header)
        self.assertIn("void CabinDriver::markExternalIoSuccess()", driver)
        self.assertIn("transport_->markExternalIoSuccess();", driver)

        read_state_start = node.index("void read_cabin_state(")
        read_state_end = node.index("\nint RunSuoquNodeWithDefaultRole(", read_state_start)
        read_state_body = node[read_state_start:read_state_end]
        frame_call_index = read_state_body.index("const int state_frame_result = Frame_Generate_With_Retry(")
        mark_index = read_state_body.index("g_cabin_driver->markExternalIoSuccess();", frame_call_index)
        self.assertIn("if (state_frame_result == 0 && g_cabin_driver)", read_state_body)
        self.assertLess(frame_call_index, mark_index)

    def test_cabin_tcp_legacy_frame_reader_consumes_exact_protocol_response_length(self):
        node = (PROCESS_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

        self.assertIn("constexpr int CABIN_STATE_RESPONSE_BYTES = 144;", node)

        frame_start = node.index("\nint Frame_Generate(uint8_t* Control_Word, int Tlen")
        frame_end = node.index("\nint Frame_Generate_With_Retry", frame_start)
        frame_body = node[frame_start:frame_end]
        self.assertIn("const int expected_recv_len = Rlen;", frame_body)
        self.assertIn("while (total_recv < expected_recv_len)", frame_body)
        self.assertIn("recv(", frame_body)
        self.assertIn("buffer + total_recv", frame_body)
        self.assertIn("total_recv += recv_len;", frame_body)
        self.assertIn("memcpy(cabin_state_buffer, buffer, static_cast<size_t>(total_recv));", frame_body)
        self.assertNotIn("recv(socket, buffer, sizeof(buffer) - 1", frame_body)

        read_state_start = node.index("void read_cabin_state(")
        read_state_end = node.index("\nint RunSuoquNodeWithDefaultRole(", read_state_start)
        read_state_body = node[read_state_start:read_state_end]
        self.assertIn(
            "Frame_Generate_With_Retry(TCP_Normal_Connection, 14, CABIN_STATE_RESPONSE_BYTES)",
            read_state_body,
        )

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
