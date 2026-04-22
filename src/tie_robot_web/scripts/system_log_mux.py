#!/usr/bin/env python3

import math
import os
import re
from pathlib import Path

import rospy
from rosgraph_msgs.msg import Log


SYSTEM_LOG_TOPIC_PREFIX = "/system_log/"
SYSTEM_LOG_TOTAL_TOPIC = "/system_log/all"
ROSOUT_AGG_TOPIC = "/rosout_agg"
STDOUT_PREFIX = "[stdout]"
STDOUT_FILE_PATTERN = re.compile(r"^(?P<node>.+)-\d+-stdout\.log$")
INVALID_TOPIC_CHARS_PATTERN = re.compile(r"[^A-Za-z0-9_]")


def sanitize_node_name(node_name):
    raw_name = str(node_name or "").strip().strip("/")
    if not raw_name:
        return "unknown"

    basename = raw_name.split("/")[-1].strip()
    if not basename:
        return "unknown"

    sanitized = INVALID_TOPIC_CHARS_PATTERN.sub("_", basename)
    return sanitized or "unknown"


def make_node_topic(node_name):
    return f"{SYSTEM_LOG_TOPIC_PREFIX}{sanitize_node_name(node_name)}"


def resolve_log_root():
    ros_log_dir = os.environ.get("ROS_LOG_DIR")
    if ros_log_dir:
        return Path(ros_log_dir).expanduser().resolve()

    return (Path.home() / ".ros" / "log").resolve()


def discover_latest_stdout_logs(log_root):
    latest_by_node = {}
    root = Path(log_root)
    if not root.exists():
        return latest_by_node

    for path in root.rglob("*-stdout.log"):
        match = STDOUT_FILE_PATTERN.match(path.name)
        if not match:
            continue

        node_name = sanitize_node_name(match.group("node"))
        previous = latest_by_node.get(node_name)
        if previous is None:
            latest_by_node[node_name] = path
            continue

        try:
            if path.stat().st_mtime > previous.stat().st_mtime:
                latest_by_node[node_name] = path
        except FileNotFoundError:
            continue

    return latest_by_node


def _split_text_lines(text):
    if not text:
        return [], ""

    lines = text.splitlines(keepends=True)
    complete = []
    partial = ""
    for line in lines:
        if line.endswith("\n") or line.endswith("\r"):
            stripped = line.rstrip("\r\n")
            if stripped:
                complete.append(stripped)
        else:
            partial = line

    return complete, partial


def read_recent_lines(path, tail_bytes):
    file_path = Path(path)
    if not file_path.exists():
        return [], 0, ""

    file_size = file_path.stat().st_size
    start = max(0, file_size - max(0, int(tail_bytes)))
    with file_path.open("rb") as handle:
        handle.seek(start)
        data = handle.read()

    text = data.decode("utf-8", errors="replace")
    if start > 0 and "\n" in text:
        text = text.split("\n", 1)[1]

    lines = [line for line in text.splitlines() if line.strip()]
    return lines, file_size, ""


def read_appended_lines(path, position, partial):
    file_path = Path(path)
    if not file_path.exists():
        return [], 0, ""

    file_size = file_path.stat().st_size
    start_position = max(0, int(position))
    carry = partial or ""

    if file_size < start_position:
        start_position = 0
        carry = ""

    with file_path.open("rb") as handle:
        handle.seek(start_position)
        data = handle.read()

    text = carry + data.decode("utf-8", errors="replace")
    lines, next_partial = _split_text_lines(text)
    return lines, file_size, next_partial


def infer_log_level(line_text):
    text = str(line_text or "")
    upper = text.upper()
    if "[FATAL]" in upper or " FATAL" in upper:
        return Log.FATAL
    if "[ERROR]" in upper or " ERROR" in upper or "_ERROR:" in upper or "ERROR:" in upper:
        return Log.ERROR
    if "[WARN]" in upper or " WARN" in upper or "_WARN:" in upper or "WARNING" in upper:
        return Log.WARN
    if "[DEBUG]" in upper or " DEBUG" in upper:
        return Log.DEBUG
    return Log.INFO


def build_stdout_log_message(node_name, line_text, source_path):
    msg = Log()
    msg.header.stamp = rospy.Time.now()
    msg.level = infer_log_level(line_text)
    msg.name = f"/{sanitize_node_name(node_name)}"
    msg.file = str(source_path)
    msg.function = "stdout_log_tail"
    msg.line = 0
    msg.topics = []
    msg.msg = f"{STDOUT_PREFIX} {line_text}"
    return msg


class StdoutLogTailer:
    def __init__(self, log_root, recent_tail_bytes=4096, self_node_name="system_log_mux"):
        self.log_root = Path(log_root)
        self.recent_tail_bytes = max(0, int(recent_tail_bytes))
        self.self_node_name = sanitize_node_name(self_node_name)
        self._states = {}

    def poll(self):
        events = []
        latest_files = discover_latest_stdout_logs(self.log_root)
        for node_name, path in latest_files.items():
            if node_name == self.self_node_name:
                continue

            state = self._states.get(node_name)
            if state is None or state["path"] != path:
                lines, position, partial = read_recent_lines(path, self.recent_tail_bytes)
                self._states[node_name] = {
                    "path": path,
                    "position": position,
                    "partial": partial,
                }
                for line in lines:
                    events.append((node_name, path, line))
                continue

            lines, position, partial = read_appended_lines(
                path, state["position"], state["partial"]
            )
            state["position"] = position
            state["partial"] = partial
            for line in lines:
                events.append((node_name, path, line))

        return events


class SystemLogMux:
    def __init__(self):
        ros_log_topic = rospy.get_param("~ros_log_topic", ROSOUT_AGG_TOPIC)
        stdout_poll_hz = float(rospy.get_param("~stdout_poll_hz", 2.0))
        stdout_tail_bytes = int(rospy.get_param("~stdout_tail_bytes", 4096))
        if not math.isfinite(stdout_poll_hz) or stdout_poll_hz <= 0.0:
            raise ValueError("stdout_poll_hz must be a positive finite number")

        self.total_topic = rospy.get_param("~total_topic", SYSTEM_LOG_TOTAL_TOPIC)
        self.log_root = Path(rospy.get_param("~log_root", str(resolve_log_root())))
        self.ros_log_topic = ros_log_topic
        self.stdout_publisher = rospy.Publisher(self.total_topic, Log, queue_size=500)
        self.node_publishers = {}
        self.stdout_tailer = StdoutLogTailer(
            log_root=self.log_root,
            recent_tail_bytes=stdout_tail_bytes,
            self_node_name=rospy.get_name(),
        )
        self.stdout_poll_hz = stdout_poll_hz
        self.stdout_timer = rospy.Timer(
            rospy.Duration(1.0 / stdout_poll_hz), self._poll_stdout_logs
        )
        self.ros_log_subscriber = rospy.Subscriber(
            ros_log_topic, Log, self._handle_ros_log, queue_size=500
        )

    def _publisher_for_node(self, node_name):
        node_topic = make_node_topic(node_name)
        publisher = self.node_publishers.get(node_topic)
        if publisher is None:
            publisher = rospy.Publisher(node_topic, Log, queue_size=200)
            self.node_publishers[node_topic] = publisher
        return publisher

    def _handle_ros_log(self, msg):
        self._publisher_for_node(msg.name).publish(msg)
        self.stdout_publisher.publish(msg)

    def _poll_stdout_logs(self, _event):
        for node_name, source_path, line_text in self.stdout_tailer.poll():
            stdout_msg = build_stdout_log_message(node_name, line_text, source_path)
            self.stdout_publisher.publish(stdout_msg)


def main():
    rospy.init_node("system_log_mux")
    try:
        node = SystemLogMux()
    except Exception as exc:
        rospy.logfatal("Failed to start system_log_mux: %s", exc)
        raise SystemExit(1) from exc

    rospy.loginfo(
        "system_log_mux started: ros_log_topic=%s total_topic=%s log_root=%s stdout_poll_hz=%.3f",
        node.ros_log_topic,
        node.total_topic,
        node.log_root,
        node.stdout_poll_hz,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
