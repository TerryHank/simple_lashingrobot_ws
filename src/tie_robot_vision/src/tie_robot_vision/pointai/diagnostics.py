"""pointAI 视觉节点诊断状态。"""
import time

import diagnostic_updater
import rospy
from diagnostic_msgs.msg import DiagnosticStatus


VISUAL_DIAGNOSTIC_HARDWARE_ID = "tie_robot/visual_algorithm"
VISUAL_INPUT_STALE_SEC = 5.0
VISUAL_ERROR_HOLD_SEC = 8.0


def setup_visual_diagnostics(self):
    self.visual_diagnostic_updater = diagnostic_updater.Updater()
    self.visual_diagnostic_updater.setHardwareID(VISUAL_DIAGNOSTIC_HARDWARE_ID)
    self.visual_diagnostic_updater.add("视觉算法", self.produce_visual_algorithm_diagnostics)
    self.visual_diagnostic_timer = rospy.Timer(rospy.Duration(1.0), self.publish_visual_diagnostics)
    self.force_visual_diagnostics()


def mark_visual_input(self, input_key):
    self.visual_input_timestamps[input_key] = time.time()
    self.force_visual_diagnostics()


def mark_visual_process_request(self):
    self.visual_last_process_request_time = time.time()
    self.force_visual_diagnostics()


def mark_visual_process_result(self, success, message=""):
    now = time.time()
    self.visual_last_process_request_time = now
    if success:
        self.visual_last_process_success_time = now
        self.visual_last_error_message = ""
        self.visual_last_error_time = 0.0
    else:
        self.visual_last_error_message = message or "视觉算法处理失败"
        self.visual_last_error_time = now
    self.force_visual_diagnostics()


def mark_visual_error(self, message):
    self.visual_last_error_message = str(message)
    self.visual_last_error_time = time.time()
    self.force_visual_diagnostics()


def force_visual_diagnostics(self):
    if getattr(self, "visual_diagnostic_updater", None) is not None:
        self.visual_diagnostic_updater.force_update()


def publish_visual_diagnostics(self, _event):
    if getattr(self, "visual_diagnostic_updater", None) is not None:
        self.visual_diagnostic_updater.update()


def produce_visual_algorithm_diagnostics(self, stat):
    now = time.time()
    stat.hardware_id = VISUAL_DIAGNOSTIC_HARDWARE_ID
    required_inputs = [
        ("红外图像", "infrared"),
        ("世界点云", "world_coord"),
        ("原始世界点云", "raw_world_coord"),
        ("相机内参", "camera_info"),
    ]
    missing_inputs = []
    stale_inputs = []

    for label, input_key in required_inputs:
        timestamp = self.visual_input_timestamps.get(input_key)
        if not timestamp:
            missing_inputs.append(label)
            stat.add(f"{input_key}_age_sec", "never")
            continue

        age_sec = max(0.0, now - timestamp)
        stat.add(f"{input_key}_age_sec", f"{age_sec:.2f}")
        if age_sec > VISUAL_INPUT_STALE_SEC:
            stale_inputs.append(f"{label}{age_sec:.1f}s")

    if self.visual_last_process_request_time > 0.0:
        stat.add("process_request_age_sec", f"{max(0.0, now - self.visual_last_process_request_time):.2f}")
    else:
        stat.add("process_request_age_sec", "never")

    if self.visual_last_process_success_time > 0.0:
        stat.add("process_success_age_sec", f"{max(0.0, now - self.visual_last_process_success_time):.2f}")
    else:
        stat.add("process_success_age_sec", "never")

    recent_error = (
        bool(self.visual_last_error_message)
        and self.visual_last_error_time > 0.0
        and (now - self.visual_last_error_time) <= VISUAL_ERROR_HOLD_SEC
    )

    if recent_error:
        stat.summary(DiagnosticStatus.ERROR, "视觉算法异常")
        stat.add("transport_state", "algorithm_error")
        stat.add("failure_detail", self.visual_last_error_message)
    elif missing_inputs:
        stat.summary(DiagnosticStatus.WARN, "视觉算法等待输入")
        stat.add("transport_state", "waiting_inputs")
        stat.add("failure_detail", "缺少输入: " + "、".join(missing_inputs))
    elif stale_inputs:
        stat.summary(DiagnosticStatus.WARN, "视觉算法输入超时")
        stat.add("transport_state", "stale_inputs")
        stat.add("failure_detail", "输入超时: " + "、".join(stale_inputs))
    else:
        stat.summary(DiagnosticStatus.OK, "视觉算法运行中")
        stat.add("transport_state", "running")
        stat.add("failure_detail", "")


def bind_visual_diagnostic_methods(cls):
    cls.setup_visual_diagnostics = setup_visual_diagnostics
    cls.mark_visual_input = mark_visual_input
    cls.mark_visual_process_request = mark_visual_process_request
    cls.mark_visual_process_result = mark_visual_process_result
    cls.mark_visual_error = mark_visual_error
    cls.force_visual_diagnostics = force_visual_diagnostics
    cls.publish_visual_diagnostics = publish_visual_diagnostics
    cls.produce_visual_algorithm_diagnostics = produce_visual_algorithm_diagnostics
