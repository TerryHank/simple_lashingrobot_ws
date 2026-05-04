#!/usr/bin/env python3

import argparse
import asyncio
import collections
import errno
import fcntl
import html
import http.client
import ipaddress
import json
import logging
import os
import pty
import re
import shlex
import shutil
import select
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time
import uuid
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit, urlunsplit

import rospy
import termios
import tornado.httpserver
import tornado.ioloop
import tornado.netutil
import tornado.web
import tornado.websocket


WORKSPACE_PICKER_PORT_PARAM = "/workspace_picker_web/port"
WORKSPACE_PICKER_URL_PARAM = "/workspace_picker_web/url"
WORKSPACE_PICKER_TERMINAL_PORT_PARAM = "/workspace_picker_web/terminal_port"
DEFAULT_BROWSER_HOST = "127.0.0.1"
DEFAULT_WORKSPACE_PICKER_PORT = 8080
DEFAULT_TERMINAL_PORT = 8081
DEFAULT_NON_PRIVILEGED_FALLBACK_PORT = 1024
DEFAULT_PORT_SEARCH_COUNT = 20
DEFAULT_TERMINAL_COLS = 120
DEFAULT_TERMINAL_ROWS = 32
TERMINAL_BACKLOG_MAX_CHUNKS = 256
TERMINAL_SHELL_RC_DIR = Path("/tmp/tie_robot_web_terminal_rc")
DEFAULT_GUI_PORT = 6080
DEFAULT_GUI_DISPLAY = 120
GUI_PROXY_PREFIX = "/api/gui/proxy/"
FRONTEND_LOGGER = logging.getLogger("workspace_picker_web_server")
GRAPHICAL_COMMAND_ALIASES = (
    "rviz",
    "rviz2",
    "rqt",
    "rqt_graph",
    "rqt_plot",
    "rqt_reconfigure",
)
WORKSPACE_ROOT = Path("/home/hyq-/simple_lashingrobot_ws")
LEGACY_SHOW_WORKSPACE = Path(
    "/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws"
)
SCEPTER_ROS_ROOT = Path("/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS")
PLANNING_BIND_PATH_FILE = (
    WORKSPACE_ROOT / "src" / "tie_robot_process" / "data" / "pseudo_slam_bind_path.json"
)
GB28181_CONFIG_FILE = (
    WORKSPACE_ROOT / "src" / "tie_robot_gb28181" / "config" / "gb28181_device.yaml"
)
PING_BIN = shutil.which("ping") or "/bin/ping"
PING_COUNT = 3
PING_TIMEOUT_SECONDS = 1
PING_COMMAND_TIMEOUT_SECONDS = 6
SYSTEMCTL_BIN = shutil.which("systemctl") or "/usr/bin/systemctl"
ROS_BACKEND_SERVICE = "tie-robot-backend.service"
ROSBRIDGE_SERVICE = "tie-robot-rosbridge.service"
DEMO_ROSBRIDGE_SERVICE = "tie-robot-demo-rosbridge.service"
DEMO_MODE_SERVICE = "tie-robot-demo-show-full.service"
LEGACY_FRONTEND_SERVICE = "tie-robot-show-legacy-frontend.service"
LEGACY_SHARED_DRIVER_STACK_SERVICE = "tie-robot-show-legacy-shared-driver-stack.service"
DEMO_MODE_FRONTEND_PORT = 5173
DRIVER_SYSTEMD_SERVICES = {
    "cabin": "tie-robot-driver-suoqu.service",
    "moduan": "tie-robot-driver-moduan.service",
    "camera": "tie-robot-driver-camera.service",
}
FULL_ROS_STACK_RESTART_ACTION = "full_restart"
FULL_ROS_STACK_STOP_ORDER = (
    ROS_BACKEND_SERVICE,
    DRIVER_SYSTEMD_SERVICES["cabin"],
    DRIVER_SYSTEMD_SERVICES["moduan"],
    DRIVER_SYSTEMD_SERVICES["camera"],
    ROSBRIDGE_SERVICE,
)
FULL_ROS_STACK_START_ORDER = (
    ROSBRIDGE_SERVICE,
    DRIVER_SYSTEMD_SERVICES["cabin"],
    DRIVER_SYSTEMD_SERVICES["moduan"],
    DRIVER_SYSTEMD_SERVICES["camera"],
    ROS_BACKEND_SERVICE,
)
FULL_ROS_STACK_STOP_TIMEOUT_SEC = 8
DEMO_MODE_STOP_TIMEOUT_SEC = 20
DEMO_MODE_STOP_ORDER = (
    ROS_BACKEND_SERVICE,
    DRIVER_SYSTEMD_SERVICES["cabin"],
    DRIVER_SYSTEMD_SERVICES["moduan"],
    DRIVER_SYSTEMD_SERVICES["camera"],
    ROSBRIDGE_SERVICE,
    LEGACY_SHARED_DRIVER_STACK_SERVICE,
)
DEMO_MODE_RESTORE_START_ORDER = (
    ROSBRIDGE_SERVICE,
    DRIVER_SYSTEMD_SERVICES["cabin"],
    DRIVER_SYSTEMD_SERVICES["moduan"],
    DRIVER_SYSTEMD_SERVICES["camera"],
    ROS_BACKEND_SERVICE,
)
DEMO_MODE_OPTIONAL_STOP_SERVICES = (
    LEGACY_SHARED_DRIVER_STACK_SERVICE,
)
DEMO_MODE_CLEANUP_PROCESS_PATTERNS = (
    f"{LEGACY_SHOW_WORKSPACE}/devel/lib/chassis_ctrl/",
    f"{LEGACY_SHOW_WORKSPACE}/src/chassis_ctrl/scripts/pointAI.py",
    f"{SCEPTER_ROS_ROOT}/devel/lib/ScepterROS/",
    "roslaunch chassis_ctrl show_full.launch",
    "roslaunch chassis_ctrl api.launch",
    "roslaunch chassis_ctrl run.launch",
)
ROS_FAST_KILL_TERM_GRACE_SEC = 0.8
ROS_FAST_KILL_KILL_GRACE_SEC = 0.3
ROS_FAST_KILL_PROCESS_PATTERNS = (
    f"{WORKSPACE_ROOT}/devel/lib/tie_robot_",
    "/opt/ros/noetic/bin/roslaunch tie_robot_bringup",
    "/opt/ros/noetic/bin/rosmaster --core -p 11311",
    "/opt/ros/noetic/lib/rosout/rosout",
    "/opt/ros/noetic/lib/rosbridge_server/rosbridge_websocket",
    "/opt/ros/noetic/lib/rosapi/rosapi_node",
    "/opt/ros/noetic/lib/tf2_web_republisher/tf2_web_republisher",
)
SYSTEMD_ROS_BACKEND_ACTIONS = {
    "/api/system/start_ros_stack": "start",
    "/api/system/stop_ros_stack": "stop",
    "/api/system/restart_ros_stack": FULL_ROS_STACK_RESTART_ACTION,
}
SYSTEMD_DRIVER_ACTIONS = {
    "/api/system/start_driver_stack": ("start", "all"),
    "/api/system/stop_driver_stack": ("stop", "all"),
    "/api/system/restart_driver_stack": ("restart", "all"),
    "/api/system/start_cabin_driver": ("start", "cabin"),
    "/api/system/stop_cabin_driver": ("stop", "cabin"),
    "/api/system/restart_cabin_driver": ("restart", "cabin"),
    "/api/system/start_moduan_driver": ("start", "moduan"),
    "/api/system/stop_moduan_driver": ("stop", "moduan"),
    "/api/system/restart_moduan_driver": ("restart", "moduan"),
    "/api/system/start_camera_driver": ("start", "camera"),
    "/api/system/stop_camera_driver": ("stop", "camera"),
    "/api/system/restart_camera_driver": ("restart", "camera"),
}
SYSTEM_CONTROL_SCRIPTS = {
    "/api/system/start_algorithm_stack": WORKSPACE_ROOT / "start_algorithm_stack.sh",
    "/api/system/stop_algorithm_stack": WORKSPACE_ROOT / "stop_algorithm_stack.sh",
    "/api/system/restart_algorithm_stack": WORKSPACE_ROOT / "restart_algorithm_stack.sh",
}
PING_HOSTNAME_RE = re.compile(
    r"^(?=.{1,253}$)(?!-)(?:[A-Za-z0-9-]{1,63}\.)*[A-Za-z0-9-]{1,63}\.?$"
)


def yaml_quote(value):
    return json.dumps(str(value), ensure_ascii=False)


def normalize_ping_host(value):
    host = str(value or "").strip()
    if not host:
        raise ValueError("请先填写要测试的 IP。")
    if len(host) > 253 or any(character.isspace() for character in host):
        raise ValueError("IP 或主机名格式无效。")

    try:
        ipaddress.ip_address(host)
        return host
    except ValueError:
        pass

    if host.startswith("-") or not PING_HOSTNAME_RE.match(host):
        raise ValueError("IP 或主机名格式无效。")
    return host


def extract_ping_summary(stdout, stderr=""):
    summary_lines = []
    for line in str(stdout or "").splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if (
            "packets transmitted" in stripped
            or "packet loss" in stripped
            or "已发送" in stripped
            or "包丢失" in stripped
            or stripped.startswith("rtt ")
            or stripped.startswith("round-trip ")
            or "min/avg/max" in stripped
        ):
            summary_lines.append(stripped)
    if summary_lines:
        return " | ".join(summary_lines[-2:])
    fallback = str(stderr or "").strip() or str(stdout or "").strip()
    return fallback.splitlines()[-1].strip() if fallback else ""


def run_network_ping(host):
    target = normalize_ping_host(host)
    command = [
        PING_BIN,
        "-c",
        str(PING_COUNT),
        "-W",
        str(PING_TIMEOUT_SECONDS),
        target,
    ]
    start_time = time.monotonic()
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=PING_COMMAND_TIMEOUT_SECONDS,
        )
        duration_ms = round((time.monotonic() - start_time) * 1000)
        summary = extract_ping_summary(completed.stdout, completed.stderr)
        reachable = completed.returncode == 0
        return {
            "success": reachable,
            "reachable": reachable,
            "target": target,
            "command": shlex.join(command),
            "returnCode": completed.returncode,
            "durationMs": duration_ms,
            "summary": summary,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
        }
    except subprocess.TimeoutExpired as exc:
        duration_ms = round((time.monotonic() - start_time) * 1000)
        stdout = exc.stdout.decode("utf-8", errors="ignore") if isinstance(exc.stdout, bytes) else str(exc.stdout or "")
        stderr = exc.stderr.decode("utf-8", errors="ignore") if isinstance(exc.stderr, bytes) else str(exc.stderr or "")
        return {
            "success": False,
            "reachable": False,
            "target": target,
            "command": shlex.join(command),
            "returnCode": None,
            "durationMs": duration_ms,
            "summary": extract_ping_summary(stdout, stderr) or "Ping 超时。",
            "stdout": stdout.strip(),
            "stderr": stderr.strip(),
            "message": "Ping 超时。",
        }
    except Exception as exc:
        duration_ms = round((time.monotonic() - start_time) * 1000)
        return {
            "success": False,
            "reachable": False,
            "target": target,
            "command": shlex.join(command),
            "returnCode": None,
            "durationMs": duration_ms,
            "summary": str(exc),
            "stdout": "",
            "stderr": str(exc),
            "message": f"Ping 执行失败：{exc}",
        }


def normalize_gb28181_port(value, label):
    try:
        port = int(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label}必须是 1-65535 之间的数字")
    if port < 1 or port > 65535:
        raise ValueError(f"{label}必须是 1-65535 之间的数字")
    return port


def normalize_gb28181_sip_payload(payload):
    sip_payload = payload.get("sip") if isinstance(payload, dict) else None
    if not isinstance(sip_payload, dict):
        raise ValueError("配置内容不完整，请重新填写后再写入。")

    required_fields = (
        ("server_ip", "上级平台 SIP IP"),
        ("server_id", "平台国标 ID"),
        ("domain", "国标域"),
        ("password", "设备接入密码"),
        ("local_ip", "本机 SIP IP"),
    )
    normalized = {}
    missing_labels = []
    for key, label in required_fields:
        value = str(sip_payload.get(key, "")).strip()
        if not value:
            missing_labels.append(label)
        normalized[key] = value
    if missing_labels:
        raise ValueError(f"请先填写：{'、'.join(missing_labels)}。")

    normalized["device_id"] = str(
        sip_payload.get("device_id") or "34020000001320000001"
    ).strip()
    if not normalized["device_id"]:
        raise ValueError("本机设备 ID 不能为空。")

    normalized["server_port"] = normalize_gb28181_port(
        sip_payload.get("server_port", 5060),
        "上级平台 SIP 端口",
    )
    normalized["local_port"] = normalize_gb28181_port(
        sip_payload.get("local_port", 5060),
        "本机 SIP 端口",
    )
    return normalized


def render_gb28181_sip_block(sip_config):
    return "\n".join([
        "sip:",
        f"  server_ip: {yaml_quote(sip_config['server_ip'])}",
        f"  server_port: {sip_config['server_port']}",
        f"  server_id: {yaml_quote(sip_config['server_id'])}",
        f"  domain: {yaml_quote(sip_config['domain'])}",
        f"  device_id: {yaml_quote(sip_config['device_id'])}",
        f"  password: {yaml_quote(sip_config['password'])}",
        f"  local_ip: {yaml_quote(sip_config['local_ip'])}",
        f"  local_port: {sip_config['local_port']}",
    ])


def replace_top_level_yaml_block(existing_text, block_name, replacement_block):
    lines = existing_text.splitlines()
    block_header = f"{block_name}:"
    start_index = None
    for index, line in enumerate(lines):
        if line.strip() == block_header and line == line.lstrip():
            start_index = index
            break

    if start_index is None:
        prefix = existing_text.rstrip()
        separator = "\n\n" if prefix else ""
        return f"{prefix}{separator}{replacement_block}\n"

    end_index = start_index + 1
    while end_index < len(lines):
        line = lines[end_index]
        is_top_level = line.strip() and line == line.lstrip()
        if is_top_level and not line.lstrip().startswith("#"):
            break
        end_index += 1

    merged_lines = (
        lines[:start_index]
        + replacement_block.splitlines()
        + lines[end_index:]
    )
    return "\n".join(merged_lines).rstrip() + "\n"


def merge_gb28181_sip_config(config_path, sip_config):
    sip_block = render_gb28181_sip_block(sip_config)
    existing_text = config_path.read_text(encoding="utf-8") if config_path.exists() else ""
    return replace_top_level_yaml_block(existing_text, "sip", sip_block)


def get_frontend_dir():
    return Path(__file__).resolve().parents[1] / "web"


def build_browser_url(browser_host, port):
    if int(port) == 80:
        return f"http://{browser_host}/index.html"
    return f"http://{browser_host}:{int(port)}/index.html"


class RosStyleLogFormatter(logging.Formatter):
    def format(self, record):
        level_name = "WARN" if record.levelname == "WARNING" else record.levelname
        return f"[{level_name}] [{record.created:.9f}]: {record.getMessage()}"


def configure_standalone_logging():
    if FRONTEND_LOGGER.handlers:
        return
    handler = logging.StreamHandler()
    handler.setFormatter(RosStyleLogFormatter())
    FRONTEND_LOGGER.addHandler(handler)
    FRONTEND_LOGGER.setLevel(logging.INFO)
    FRONTEND_LOGGER.propagate = False


def resolve_clean_url_request_path(request_path, frontend_dir):
    parsed = urlsplit(request_path)
    raw_path = parsed.path or "/"
    if raw_path in ("", "/"):
        return request_path

    requested_file = frontend_dir / raw_path.lstrip("/")
    if requested_file.is_file():
        return request_path

    if Path(raw_path).suffix:
        return request_path

    html_candidate = requested_file.with_suffix(".html")
    if html_candidate.exists():
        return urlunsplit((parsed.scheme, parsed.netloc, f"{raw_path}.html", parsed.query, parsed.fragment))

    return request_path


def set_pty_window_size(fd, cols, rows):
    cols = max(40, int(cols or DEFAULT_TERMINAL_COLS))
    rows = max(12, int(rows or DEFAULT_TERMINAL_ROWS))
    fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))


def normalize_graphical_command(raw_command):
    if isinstance(raw_command, str):
        command = shlex.split(raw_command)
    elif isinstance(raw_command, (list, tuple)):
        command = [str(part) for part in raw_command if str(part)]
    else:
        command = []
    return command


def build_terminal_shell_command(rcfile_path):
    return ["/bin/bash", "--rcfile", str(rcfile_path), "-i"]


def build_terminal_rcfile(session_id, gui_api_url):
    alias_functions = "\n".join(
        f'{alias}() {{ __workspace_picker_launch_gui "{alias}" "$@"; }}'
        for alias in GRAPHICAL_COMMAND_ALIASES
    )
    return f"""
if [ -f ~/.bash_profile ]; then
  . ~/.bash_profile
elif [ -f ~/.bash_login ]; then
  . ~/.bash_login
elif [ -f ~/.profile ]; then
  . ~/.profile
elif [ -f ~/.bashrc ]; then
  . ~/.bashrc
fi

export WORKSPACE_PICKER_GUI_API={shlex.quote(gui_api_url)}
export WORKSPACE_PICKER_TERMINAL_SESSION_ID={shlex.quote(session_id)}

__workspace_picker_launch_gui() {{
  if [ "$#" -lt 1 ]; then
    echo "用法: gui <command> [args...]"
    return 2
  fi
  python3 - "$WORKSPACE_PICKER_GUI_API" "$WORKSPACE_PICKER_TERMINAL_SESSION_ID" "$@" <<'PY'
import json
import signal
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import quote

api_url = sys.argv[1]
terminal_session_id = sys.argv[2]
command = sys.argv[3:]
payload = json.dumps({{
    "command": command,
    "terminal_session_id": terminal_session_id,
}}, ensure_ascii=False).encode("utf-8")
request = urllib.request.Request(
    api_url,
    data=payload,
    headers={{"Content-Type": "application/json; charset=utf-8"}},
    method="POST",
)
try:
    with urllib.request.urlopen(request, timeout=6.0) as response:
        body = response.read().decode("utf-8", errors="ignore")
        result = json.loads(body or "{{}}")
except urllib.error.HTTPError as exc:
    body = exc.read().decode("utf-8", errors="ignore")
    try:
        result = json.loads(body or "{{}}")
    except Exception:
        result = {{"success": False, "message": body or str(exc)}}
except Exception as exc:
    result = {{"success": False, "message": str(exc)}}

message = result.get("message") or "图形界面请求已发送"
sys.stdout.write(message + "\\n")
sys.stdout.flush()

session = result.get("session") or {{}}
session_id = str(session.get("sessionId") or result.get("session_id") or "")
session_label = str(session.get("label") or " ".join(command) or "图形界面")
if not result.get("success") or not session_id:
    sys.exit(0 if result.get("success") else 1)

session_url = api_url.rstrip("/") + "/" + quote(session_id, safe="")
session_closed = False

def close_graphical_session(exit_code=130):
    global session_closed
    if not session_closed:
        session_closed = True
        try:
            close_request = urllib.request.Request(session_url, method="DELETE")
            with urllib.request.urlopen(close_request, timeout=3.0):
                pass
        except Exception:
            pass
    sys.stdout.write("\\n已关闭图形界面：" + session_label + "\\n")
    sys.stdout.flush()
    sys.exit(exit_code)

def handle_interrupt(signum, _frame):
    close_graphical_session(130 if signum == signal.SIGINT else 143)

def find_current_session(sessions):
    for item in sessions:
        item_id = str(item.get("sessionId") or item.get("session_id") or "")
        if item_id == session_id:
            return item
    return None

signal.signal(signal.SIGINT, handle_interrupt)
signal.signal(signal.SIGTERM, handle_interrupt)
if hasattr(signal, "SIGHUP"):
    signal.signal(signal.SIGHUP, handle_interrupt)

while True:
    try:
        status_request = urllib.request.Request(api_url, method="GET")
        with urllib.request.urlopen(status_request, timeout=3.0) as response:
            body = response.read().decode("utf-8", errors="ignore")
            status = json.loads(body or "{{}}")
        current_session = find_current_session(status.get("sessions") or [])
        if not current_session:
            sys.exit(0)
        if current_session.get("state") == "error":
            sys.exit(1)
        time.sleep(0.8)
    except KeyboardInterrupt:
        close_graphical_session(130)
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            sys.exit(0)
        time.sleep(0.8)
    except Exception:
        time.sleep(0.8)
PY
}}

gui() {{
  __workspace_picker_launch_gui "$@"
}}

{alias_functions}
"""


class TerminalSession:
    def __init__(
        self,
        session_id,
        label,
        manager,
        cols=DEFAULT_TERMINAL_COLS,
        rows=DEFAULT_TERMINAL_ROWS,
        gui_api_url=None,
    ):
        self.session_id = session_id
        self.label = label or ""
        self.manager = manager
        self.cols = max(40, int(cols or DEFAULT_TERMINAL_COLS))
        self.rows = max(12, int(rows or DEFAULT_TERMINAL_ROWS))
        self.gui_api_url = gui_api_url or f"http://127.0.0.1:{DEFAULT_WORKSPACE_PICKER_PORT}/api/gui/sessions"
        self.clients = set()
        self.buffer = collections.deque(maxlen=TERMINAL_BACKLOG_MAX_CHUNKS)
        self.buffer_chars = 0
        self.lock = threading.RLock()
        self.closed = False
        self.exit_code = None
        self.master_fd = None
        self.process = None
        self.reader_thread = None
        self.rcfile_path = None
        self.manager_closing = False
        self._spawn_shell_process()

    def _spawn_shell_process(self):
        TERMINAL_SHELL_RC_DIR.mkdir(parents=True, exist_ok=True)
        self.rcfile_path = str(TERMINAL_SHELL_RC_DIR / f"{self.session_id}.bashrc")
        Path(self.rcfile_path).write_text(
            build_terminal_rcfile(self.session_id, self.gui_api_url),
            encoding="utf-8",
        )

        master_fd = None
        slave_fd = None
        try:
            master_fd, slave_fd = pty.openpty()
            set_pty_window_size(master_fd, self.cols, self.rows)
            flags = fcntl.fcntl(master_fd, fcntl.F_GETFL)
            fcntl.fcntl(master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

            env = os.environ.copy()
            env.setdefault("TERM", "xterm-256color")
            env.setdefault("COLORTERM", "truecolor")
            env["WORKSPACE_PICKER_GUI_API"] = self.gui_api_url
            env["WORKSPACE_PICKER_TERMINAL_SESSION_ID"] = self.session_id

            self.process = subprocess.Popen(
                build_terminal_shell_command(self.rcfile_path),
                stdin=slave_fd,
                stdout=slave_fd,
                stderr=slave_fd,
                cwd=str(WORKSPACE_ROOT),
                env=env,
                close_fds=True,
                start_new_session=True,
            )
            os.close(slave_fd)
            slave_fd = None
            self.master_fd = master_fd
            master_fd = None
            self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.reader_thread.start()
        except Exception:
            if slave_fd is not None:
                try:
                    os.close(slave_fd)
                except OSError:
                    pass
            if master_fd is not None:
                try:
                    os.close(master_fd)
                except OSError:
                    pass
            self._cleanup_rcfile()
            raise

    def is_alive(self):
        return bool(
            not self.closed
            and self.process is not None
            and self.process.poll() is None
        )

    def snapshot(self):
        return {
            "session_id": self.session_id,
            "sessionId": self.session_id,
            "label": self.label,
            "cols": self.cols,
            "rows": self.rows,
        }

    def get_history_text(self):
        with self.lock:
            return "".join(self.buffer)

    def attach_client(self, client):
        with self.lock:
            self.clients.add(client)

    def detach_client(self, client):
        with self.lock:
            self.clients.discard(client)

    def write(self, data):
        if self.closed or self.master_fd is None or not self.is_alive() or not data:
            return
        if isinstance(data, str):
            data = data.encode("utf-8", errors="ignore")
        try:
            os.write(self.master_fd, data)
        except OSError as exc:
            rospy.logwarn("terminal session write failed: %s (%s)", self.session_id, exc)

    def resize(self, cols, rows):
        self.cols = max(40, int(cols or self.cols))
        self.rows = max(12, int(rows or self.rows))
        if self.master_fd is None or self.closed:
            return
        try:
            set_pty_window_size(self.master_fd, self.cols, self.rows)
        except OSError as exc:
            rospy.logwarn("terminal session resize failed: %s (%s)", self.session_id, exc)

    def close(self, terminate_process=False):
        self.closed = True
        self.manager_closing = bool(terminate_process)
        process = self.process
        if terminate_process and process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception:
                pass
            try:
                process.wait(timeout=0.8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass
                try:
                    process.wait(timeout=0.5)
                except Exception:
                    pass
        try:
            if self.master_fd is not None:
                os.close(self.master_fd)
        except OSError:
            pass
        self.master_fd = None
        self._cleanup_rcfile()

    def _cleanup_rcfile(self):
        if self.rcfile_path:
            try:
                os.unlink(self.rcfile_path)
            except OSError:
                pass
            self.rcfile_path = None

    def _append_output(self, text):
        if not text:
            return
        with self.lock:
            self.buffer.append(text)
            self.buffer_chars += len(text)
            while self.buffer and self.buffer_chars > 65536:
                removed = self.buffer.popleft()
                self.buffer_chars -= len(removed)

    def _read_loop(self):
        while not self.closed:
            if self.process and self.process.poll() is not None:
                break
            fd = self.master_fd
            if fd is None:
                break
            try:
                ready, _, _ = select.select([fd], [], [], 0.2)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                continue
            except OSError:
                break
            if not chunk:
                break
            text = chunk.decode("utf-8", errors="ignore")
            self._append_output(text)
            self.manager.broadcast(self.session_id, {
                "type": "output",
                "data": text,
            })

        process = self.process
        if process and self.exit_code is None:
            try:
                self.exit_code = process.wait(timeout=0.5)
            except Exception:
                self.exit_code = -1
        try:
            if self.master_fd is not None:
                os.close(self.master_fd)
        except OSError:
            pass
        self.process = None
        self.master_fd = None
        closed_by_manager = self.manager_closing
        self.manager_closing = False
        self.closed = True
        self._cleanup_rcfile()
        if closed_by_manager:
            return
        self.manager.broadcast(self.session_id, {
            "type": "exit",
            "code": int(self.exit_code or 0),
        })


class TerminalSessionManager:
    def __init__(self):
        self.sessions = {}
        self.lock = threading.RLock()
        self.io_loop = None
        self.sequence = 0
        self.gui_api_url = f"http://127.0.0.1:{DEFAULT_WORKSPACE_PICKER_PORT}/api/gui/sessions"

    def set_io_loop(self, io_loop):
        self.io_loop = io_loop

    def set_gui_api_url(self, gui_api_url):
        self.gui_api_url = gui_api_url
        with self.lock:
            sessions = list(self.sessions.values())
        for session in sessions:
            session.gui_api_url = gui_api_url

    def build_session_label(self):
        shell_name = Path(build_terminal_shell_command("/tmp/tie_robot_empty_rc")[0]).name or "shell"
        existing_labels = {session.label for session in self.sessions.values()}
        if shell_name not in existing_labels:
            return shell_name
        suffix = 2
        while f"{shell_name} {suffix}" in existing_labels:
            suffix += 1
        return f"{shell_name} {suffix}"

    def create_session(self, cols=DEFAULT_TERMINAL_COLS, rows=DEFAULT_TERMINAL_ROWS):
        with self.lock:
            self.sequence += 1
            session_id = uuid.uuid4().hex[:10]
            label = self.build_session_label()
            session = TerminalSession(
                session_id,
                label,
                self,
                cols=cols,
                rows=rows,
                gui_api_url=self.gui_api_url,
            )
            self.sessions[session_id] = session
            return session

    def list_sessions(self):
        with self.lock:
            stale_session_ids = [
                session_id
                for session_id, session in self.sessions.items()
                if session.closed
            ]
            for session_id in stale_session_ids:
                self.sessions.pop(session_id, None)
            return [session.snapshot() for session in self.sessions.values()]

    def get_session(self, session_id):
        with self.lock:
            return self.sessions.get(session_id)

    def attach_client(self, session_id, client):
        session = self.get_session(session_id)
        if not session:
            return None
        session.attach_client(client)
        return session

    def detach_client(self, session_id, client):
        session = self.get_session(session_id)
        if not session:
            return
        session.detach_client(client)
        if session.closed:
            with self.lock:
                self.sessions.pop(session_id, None)

    def resize_session(self, session_id, cols, rows):
        session = self.get_session(session_id)
        if session:
            session.resize(cols, rows)

    def write_session(self, session_id, data):
        session = self.get_session(session_id)
        if session:
            session.write(data)

    def close_session(self, session_id):
        session = None
        with self.lock:
            session = self.sessions.pop(session_id, None)
        if session:
            session.close(terminate_process=True)
            self.broadcast(session_id, {"type": "closed"})
        return session

    def shutdown(self):
        with self.lock:
            sessions = list(self.sessions.values())
            self.sessions.clear()
        for session in sessions:
            session.close(terminate_process=True)

    def _broadcast_on_loop(self, session_id, payload):
        session = self.get_session(session_id)
        if not session:
            return
        payload_text = json.dumps(payload, ensure_ascii=False)
        stale_clients = []
        for client in list(session.clients):
            try:
                client.write_message(payload_text)
            except Exception:
                stale_clients.append(client)
        for client in stale_clients:
            session.detach_client(client)

    def broadcast(self, session_id, payload):
        if self.io_loop is None:
            return
        self.io_loop.add_callback(self._broadcast_on_loop, session_id, payload)


class GraphicalAppSession:
    def __init__(
        self,
        session_id,
        label,
        command,
        manager,
        terminal_session_id=None,
        display_number=None,
        web_port=None,
    ):
        self.session_id = session_id
        self.label = label
        self.command = command
        self.manager = manager
        self.terminal_session_id = terminal_session_id
        self.display_number = display_number
        self.web_port = web_port
        self.web_path = "/"
        self.state = "starting"
        self.message = "正在启动图形界面。"
        self.created_at = time.time()
        self.process = None
        self.monitor_thread = None
        self.closed = False
        self.primary_window_id = None
        self.duplicate_index_request_window_sec = 8.0
        self.index_request_times = {}
        self._spawn()

    def snapshot(self):
        return {
            "sessionId": self.session_id,
            "label": self.label,
            "command": self.command,
            "terminalSessionId": self.terminal_session_id,
            "display": f":{self.display_number}" if self.display_number is not None else "",
            "webPort": self.web_port,
            "webPath": self.web_path,
            "state": self.state,
            "message": self.message,
            "createdAt": self.created_at,
        }

    def _spawn(self):
        xpra_path = shutil.which("xpra")
        if not xpra_path:
            self.state = "error"
            self.message = "未找到 xpra，无法把 rviz/rqt 这类原生图形窗口嵌入前端卡片。"
            return

        command_text = shlex.join(self.command)
        xpra_command = [
            xpra_path,
            "start",
            f":{self.display_number}",
            "--daemon=no",
            f"--bind-ws=0.0.0.0:{self.web_port}",
            "--html=on",
            "--mdns=no",
            "--sharing=yes",
            "--lock=no",
            "--notifications=no",
            "--pulseaudio=no",
            "--exit-with-children=yes",
            "--start-env=QT_X11_NO_MITSHM=1",
            "--start-env=LIBGL_ALWAYS_SOFTWARE=1",
            "--start-env=DISABLE_ROS1_EOL_WARNINGS=1",
            f"--start-child={command_text}",
        ]
        env = os.environ.copy()
        env.setdefault("XPRA_HTML", "1")
        env.setdefault("QT_X11_NO_MITSHM", "1")
        env.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
        env.setdefault("DISABLE_ROS1_EOL_WARNINGS", "1")
        try:
            self.process = subprocess.Popen(
                xpra_command,
                cwd=str(WORKSPACE_ROOT),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                close_fds=True,
                start_new_session=True,
            )
        except Exception as exc:
            self.state = "error"
            self.message = f"启动图形界面失败：{exc}"
            return

        self.monitor_thread = threading.Thread(target=self._monitor, daemon=True)
        self.monitor_thread.start()

    def _monitor(self):
        ready_deadline = time.time() + 30.0
        while not self.closed and time.time() < ready_deadline:
            if self.process and self.process.poll() is not None:
                self.state = "error"
                self.message = f"图形界面进程已退出，退出码 {self.process.returncode}。"
                self.manager.notify_session_updated(self)
                return
            if self._is_web_port_ready():
                self.state = "ready"
                self.message = "图形界面已在前端卡片中打开。"
                self.manager.notify_session_updated(self)
                break
            time.sleep(0.2)

        if self.state == "starting" and not self.closed:
            self.state = "error"
            self.message = "图形界面端口启动超时，请关闭后重试。"
            self.manager.notify_session_updated(self)

        if not self.process:
            return
        exit_code = self.process.wait()
        if self.closed:
            return
        self.closed = True
        self.state = "closed"
        self.message = f"图形界面已退出，退出码 {exit_code}。"
        self.manager.forget_session(self.session_id)
        self.manager.notify_session_closed(self)

    def _is_web_port_ready(self):
        return self._is_web_http_ready(timeout=0.8)

    def should_suppress_duplicate_index_request(self, client_key, target_path):
        path_only = str(target_path or "/").split("?", 1)[0] or "/"
        if path_only not in ("/", "/index.html"):
            return False
        if "embed_instance=" in str(target_path or ""):
            return False

        now = time.time()
        client_text = str(client_key or "unknown")
        last_request_time = self.index_request_times.get(client_text)
        self.index_request_times[client_text] = now

        stale_before = now - (self.duplicate_index_request_window_sec * 4)
        for key, request_time in list(self.index_request_times.items()):
            if request_time < stale_before:
                self.index_request_times.pop(key, None)

        return (
            last_request_time is not None
            and now - last_request_time < self.duplicate_index_request_window_sec
        )

    def _is_web_http_ready(self, timeout=0.8):
        if not self.web_port:
            return False
        connection = None
        try:
            connection = http.client.HTTPConnection("127.0.0.1", int(self.web_port), timeout=float(timeout))
            connection.request("GET", "/", headers={"Host": f"127.0.0.1:{int(self.web_port)}"})
            response = connection.getresponse()
            return 200 <= int(response.status) < 500
        except Exception:
            return False
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass

    def fit_to_viewport(self, width, height):
        if self.should_skip_backend_geometry_fit():
            return True, "RViz 使用 Xpra 原生窗口尺寸，跳过后端强制缩放以避免 OpenGL 黑屏。"

        width = max(320, min(4096, int(width or 0)))
        height = max(240, min(4096, int(height or 0)))
        window_id = self._find_primary_window_id()
        if not window_id:
            return False, "暂未找到图形程序主窗口。"

        env = self._display_env()
        wmctrl_path = shutil.which("wmctrl")
        xdotool_path = shutil.which("xdotool")
        if not wmctrl_path and not xdotool_path:
            return False, "未找到 wmctrl/xdotool，无法调整图形窗口尺寸。"

        commands = []
        if xdotool_path:
            commands.append([xdotool_path, "windowsize", str(window_id), str(width), str(height)])
            commands.append([xdotool_path, "windowmove", str(window_id), "0", "0"])
        if wmctrl_path:
            window_ref = hex(int(str(window_id), 0))
            commands.append([wmctrl_path, "-ir", window_ref, "-e", f"0,0,0,{width},{height}"])
        elif xdotool_path:
            commands.insert(
                0,
                [xdotool_path, "windowmove", str(window_id), "0", "0"],
            )

        for command in commands:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                env=env,
                timeout=2,
            )
            if completed.returncode != 0:
                self.primary_window_id = None
                message = completed.stderr.strip() or completed.stdout.strip() or "xdotool 调整图形窗口失败。"
                return False, message
        if xdotool_path:
            subprocess.run(
                [xdotool_path, "windowraise", str(window_id)],
                check=False,
                capture_output=True,
                text=True,
                env=env,
                timeout=2,
            )
        return True, f"图形窗口已调整为 {width}x{height}。"

    def should_skip_backend_geometry_fit(self):
        command_name = Path(str(self.command[0] if self.command else "")).name
        return command_name in {"rviz", "rviz2"}

    def _display_env(self):
        env = os.environ.copy()
        env["DISPLAY"] = f":{self.display_number}"
        return env

    def _find_primary_window_id(self):
        if self.primary_window_id and self._get_window_geometry(self.primary_window_id):
            return self.primary_window_id

        xdotool_path = shutil.which("xdotool")
        if not xdotool_path:
            return None

        search_terms = []
        command_name = Path(str(self.command[0] if self.command else "")).name
        if command_name:
            search_terms.append(("class", command_name))
            search_terms.append(("name", command_name))
        if self.label:
            search_terms.append(("name", self.label.split()[0]))

        candidate_ids = []
        for search_type, term in search_terms:
            completed = subprocess.run(
                [xdotool_path, "search", f"--{search_type}", term],
                check=False,
                capture_output=True,
                text=True,
                env=self._display_env(),
                timeout=2,
            )
            if completed.returncode != 0:
                continue
            for raw_window_id in completed.stdout.splitlines():
                window_id = raw_window_id.strip()
                if window_id and window_id not in candidate_ids:
                    candidate_ids.append(window_id)

        best_window_id = None
        best_area = 0
        for window_id in candidate_ids:
            geometry = self._get_window_geometry(window_id)
            if not geometry:
                continue
            area = geometry["width"] * geometry["height"]
            if area > best_area:
                best_area = area
                best_window_id = window_id

        self.primary_window_id = best_window_id
        return best_window_id

    def _get_window_geometry(self, window_id):
        xwininfo_path = shutil.which("xwininfo")
        if not xwininfo_path:
            return None
        completed = subprocess.run(
            [xwininfo_path, "-id", str(window_id)],
            check=False,
            capture_output=True,
            text=True,
            env=self._display_env(),
            timeout=2,
        )
        if completed.returncode != 0:
            return None
        width_match = re.search(r"^\s*Width:\s*(\d+)", completed.stdout, re.MULTILINE)
        height_match = re.search(r"^\s*Height:\s*(\d+)", completed.stdout, re.MULTILINE)
        if not width_match or not height_match:
            return None
        width = int(width_match.group(1))
        height = int(height_match.group(1))
        if width < 80 or height < 60:
            return None
        return {"width": width, "height": height}

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.process and self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception:
                pass
        self.state = "closed"
        self.message = "图形界面已关闭。"


class GraphicalAppSessionManager:
    def __init__(
        self,
        terminal_manager=None,
        preferred_port=DEFAULT_GUI_PORT,
        search_count=DEFAULT_PORT_SEARCH_COUNT,
    ):
        self.sessions = {}
        self.lock = threading.RLock()
        self.terminal_manager = terminal_manager
        self.preferred_port = preferred_port
        self.search_count = search_count
        self.sequence = 0
        self.next_display_number = DEFAULT_GUI_DISPLAY

    def set_terminal_manager(self, terminal_manager):
        self.terminal_manager = terminal_manager

    def list_sessions(self):
        with self.lock:
            return [
                session.snapshot()
                for session in self.sessions.values()
                if not session.closed and session.state != "closed"
            ]

    def get_session(self, session_id):
        with self.lock:
            return self.sessions.get(session_id)

    def forget_session(self, session_id):
        with self.lock:
            return self.sessions.pop(session_id, None)

    def create_session(self, raw_command, terminal_session_id=None):
        command = normalize_graphical_command(raw_command)
        if not command:
            raise ValueError("图形界面命令不能为空")

        with self.lock:
            self.sequence += 1
            session_id = uuid.uuid4().hex[:10]
            label = f"{command[0]} {self.sequence}"
            display_number = self.next_display_number
            self.next_display_number += 1
            web_port = self._allocate_web_port()
            session = GraphicalAppSession(
                session_id,
                label,
                command,
                self,
                terminal_session_id=terminal_session_id,
                display_number=display_number,
                web_port=web_port,
            )
            self.sessions[session_id] = session

        self.notify_session_updated(session)
        return session

    def close_session(self, session_id):
        with self.lock:
            session = self.sessions.pop(session_id, None)
        if not session:
            return None
        session.close()
        self.notify_session_closed(session)
        return session

    def shutdown(self):
        with self.lock:
            sessions = list(self.sessions.values())
            self.sessions.clear()
        for session in sessions:
            session.close()

    def notify_session_updated(self, session):
        if not self.terminal_manager or not session.terminal_session_id:
            return
        self.terminal_manager.broadcast(session.terminal_session_id, {
            "type": "gui_session",
            "session": session.snapshot(),
        })
        self.terminal_manager.broadcast(session.terminal_session_id, {
            "type": "gui_sessions",
            "sessions": self.list_sessions(),
        })

    def notify_session_closed(self, session):
        if not self.terminal_manager or not session.terminal_session_id:
            return
        self.terminal_manager.broadcast(session.terminal_session_id, {
            "type": "gui_session_closed",
            "sessionId": session.session_id,
        })
        self.terminal_manager.broadcast(session.terminal_session_id, {
            "type": "gui_sessions",
            "sessions": self.list_sessions(),
        })

    def _allocate_web_port(self):
        occupied = {
            int(session.web_port)
            for session in self.sessions.values()
            if session.web_port is not None
        }
        for candidate_port in iter_candidate_ports(self.preferred_port, self.search_count * 4):
            if candidate_port in occupied:
                continue
            if self._is_port_available(candidate_port):
                return candidate_port
        raise OSError(errno.EADDRINUSE, "未找到可用图形界面端口")

    @staticmethod
    def _is_port_available(port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", int(port)))
            return True
        except OSError:
            return False
        finally:
            sock.close()


class TerminalWebSocketHandler(tornado.websocket.WebSocketHandler):
    def initialize(self, manager):
        self.manager = manager
        self.session = None
        self.session_id = None

    def check_origin(self, origin):
        return True

    def open(self, session_id):
        session = self.manager.attach_client(session_id, self)
        if session is None:
            self.close(code=4404, reason="terminal session not found")
            return
        self.session = session
        self.session_id = session_id
        self.write_message(json.dumps({
            "type": "ready",
            "sessionId": session.session_id,
            "label": session.label,
            "cols": session.cols,
            "rows": session.rows,
        }, ensure_ascii=False))
        history = session.get_history_text()
        if history:
            self.write_message(json.dumps({
                "type": "history",
                "data": history,
            }, ensure_ascii=False))
        if session.closed:
            self.write_message(json.dumps({
                "type": "exit",
                "code": int(session.exit_code or 0),
            }, ensure_ascii=False))

    def on_message(self, message):
        if self.session is None:
            return
        try:
            payload = json.loads(message)
        except json.JSONDecodeError:
            return
        message_type = payload.get("type")
        if message_type == "input":
            self.manager.write_session(self.session_id, payload.get("data", ""))
            return
        if message_type == "resize":
            self.manager.resize_session(
                self.session_id,
                payload.get("cols", DEFAULT_TERMINAL_COLS),
                payload.get("rows", DEFAULT_TERMINAL_ROWS),
            )
            return

    def on_close(self):
        if self.session_id:
            self.manager.detach_client(self.session_id, self)


class TerminalServerThread:
    def __init__(self, host, preferred_port, search_count, manager):
        self.host = host
        self.preferred_port = preferred_port
        self.search_count = search_count
        self.manager = manager
        self.actual_port = None
        self.thread = None
        self.io_loop = None
        self.http_server = None
        self.ready = threading.Event()

    def start(self):
        sockets, actual_port = bind_tornado_sockets(self.host, self.preferred_port, self.search_count)
        self.actual_port = actual_port

        def run():
            self.io_loop = tornado.ioloop.IOLoop()
            asyncio.set_event_loop(self.io_loop.asyncio_loop)
            self.manager.set_io_loop(self.io_loop)
            app = tornado.web.Application([
                (r"/ws/terminal/(?P<session_id>[^/]+)", TerminalWebSocketHandler, {"manager": self.manager}),
            ])
            self.http_server = tornado.httpserver.HTTPServer(app)
            self.http_server.add_sockets(sockets)
            self.ready.set()
            self.io_loop.start()

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        self.ready.wait(timeout=5.0)
        return self.actual_port

    def stop(self):
        self.manager.shutdown()
        if self.io_loop is not None:
            self.io_loop.add_callback(self._stop_on_loop)
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def _stop_on_loop(self):
        if self.http_server is not None:
            self.http_server.stop()
        self.io_loop.stop()


def bind_tornado_sockets(host, preferred_port, search_count=DEFAULT_PORT_SEARCH_COUNT):
    last_exc = None
    for candidate_port in iter_candidate_ports(preferred_port, search_count=search_count):
        try:
            sockets = tornado.netutil.bind_sockets(candidate_port, address=host)
            return sockets, candidate_port
        except OSError as exc:
            if exc.errno not in (errno.EADDRINUSE, errno.EACCES, getattr(errno, "EPERM", errno.EACCES)):
                raise
            last_exc = exc
    if last_exc is not None:
        raise last_exc
    raise OSError(errno.EADDRINUSE, "未找到可用终端端口")


class NoCacheStaticHandler(SimpleHTTPRequestHandler):
    rbufsize = 0
    terminal_manager = None
    graphical_app_manager = None
    terminal_port = DEFAULT_TERMINAL_PORT

    def do_GET(self):
        parsed = urlsplit(self.path)
        if parsed.path.startswith(GUI_PROXY_PREFIX):
            self.handle_graphical_app_proxy_get(parsed)
            return
        if parsed.path == "/api/terminal/config":
            self.send_json({
                "success": True,
                "terminal_port": int(self.terminal_port),
                "ws_path_prefix": "/ws/terminal",
                "sessions": self.terminal_manager.list_sessions() if self.terminal_manager else [],
            })
            return
        if parsed.path == "/api/gui/sessions":
            self.send_json({
                "success": True,
                "sessions": self.graphical_app_manager.list_sessions() if self.graphical_app_manager else [],
            })
            return
        if parsed.path == "/api/system/ros_stack_status":
            self.handle_ros_backend_status_get()
            return
        if parsed.path == "/api/system/demo_mode_status":
            self.handle_demo_mode_status_get()
            return
        if parsed.path == "/api/planning/bind-path":
            self.handle_planning_bind_path_get()
            return
        return super().do_GET()

    def handle_graphical_app_proxy_get(self, parsed):
        if self.graphical_app_manager is None:
            self.send_error(503, "graphical app manager unavailable")
            return
        proxy_tail = parsed.path[len(GUI_PROXY_PREFIX):]
        session_id, separator, resource_path = proxy_tail.partition("/")
        if not session_id:
            self.send_error(404, "graphical app session not found")
            return
        session = self.graphical_app_manager.get_session(session_id)
        if session is None or session.state in ("closed", "error"):
            message = session.message if session is not None else "图形界面会话已结束，请重新打开。"
            self.send_graphical_app_session_closed_page(session_id, message)
            return
        target_path = f"/{resource_path}" if separator else "/"
        if not target_path or target_path == "/":
            target_path = "/"
        if parsed.query:
            target_path = f"{target_path}?{parsed.query}"
        if self.headers.get("Upgrade", "").lower() == "websocket":
            self.proxy_graphical_app_websocket(session, target_path.split("?", 1)[0] or "/")
            return
        if session.should_suppress_duplicate_index_request(self.client_address[0], target_path):
            self.send_response(204)
            self.end_headers()
            return
        self.proxy_graphical_app_http(session, target_path)

    def proxy_graphical_app_http(self, session, target_path):
        connection = None
        try:
            self.wait_for_graphical_app_port(session)
            connection = http.client.HTTPConnection(
                "127.0.0.1",
                int(session.web_port),
                timeout=8,
            )
            headers = {
                key: value
                for key, value in self.headers.items()
                if key.lower() not in {"host", "connection", "upgrade", "proxy-connection", "accept-encoding"}
            }
            headers["Host"] = f"127.0.0.1:{int(session.web_port)}"
            headers["Connection"] = "close"
            connection.request("GET", target_path, headers=headers)
            response = connection.getresponse()
            try:
                body = response.read()
            except http.client.IncompleteRead as exc:
                body = exc.partial
            body, patched = self._patch_xpra_html5_resource(target_path, body)
        except Exception as exc:
            message = f"图形界面连接已失效：{exc}"
            self.handle_graphical_app_session_unhealthy(session, message)
            self.send_graphical_app_session_closed_page(session.session_id, message, status_code=502)
            return
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass

        self.send_response(response.status, response.reason)
        hop_by_hop_headers = {
            "connection",
            "keep-alive",
            "proxy-authenticate",
            "proxy-authorization",
            "te",
            "trailers",
            "transfer-encoding",
            "upgrade",
        }
        for key, value in response.getheaders():
            if key.lower() in hop_by_hop_headers:
                continue
            if patched and key.lower() in {"content-length", "content-encoding"}:
                continue
            self.send_header(key, value)
        if patched:
            self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    @staticmethod
    def _patch_xpra_html5_resource(target_path, body):
        path_only = target_path.split("?", 1)[0]
        if path_only.endswith("/js/Protocol.js"):
            patched_body = body.replace(
                b"var proto_flags = 0x10;",
                b"var proto_flags = 0x1;",
            )
            return patched_body, patched_body != body
        if path_only.endswith("/js/Client.js"):
            patched_body = body.replace(
                b'"rencodeplus": true,',
                b'"rencode": true,',
            ).replace(
                b'      "encodings": {\n'
                b'        "": this.supported_encodings,\n'
                b'        "core": this.supported_encodings,\n'
                b'        "rgb_formats": this.RGB_FORMATS,\n'
                b'        "window-icon": ["png"],\n'
                b'        "cursor": ["png"],\n'
                b'        "packet": true,\n'
                b"      },",
                b'      "encodings": this.supported_encodings,\n'
                b'      "encodings.core": this.supported_encodings,\n'
                b'      "encodings.rgb_formats": this.RGB_FORMATS,\n'
                b'      "encodings.window-icon": ["png"],\n'
                b'      "encodings.cursor": ["png"],\n'
                b'      "encodings.packet": true,',
            ).replace(
                b'if (!hello["rencodeplus"]) {\n'
                b'      throw "no common packet encoders, \'rencodeplus\' is required by this client";\n'
                b"    }",
                b'if (!hello["rencode"] && !hello["rencodeplus"]) {\n'
                b'      throw "no common packet encoders, \'rencode\' is required by this client";\n'
                b"    }",
            )
            return patched_body, patched_body != body
        if path_only.endswith("/js/Window.js"):
            patched_body = body.replace(
                b"    this.configure_border_class();\n"
                b"    this.add_headerbar();\n"
                b"    this.make_draggable()\n",
                b"    if (this.is_tie_robot_embedded_main_window()) {\n"
                b"      this.resizable = false;\n"
                b"      this.maximized = true;\n"
                b"      jQuery(this.div).addClass(\"tie-robot-embedded-main-window\");\n"
                b"    }\n"
                b"    this.configure_border_class();\n"
                b"    this.add_headerbar();\n"
                b"    this.make_draggable()\n",
            ).replace(
                b"      }\n"
                b"      this._set_decorated(decorated);\n",
                b"      }\n"
                b"      if (this.is_tie_robot_embedded_main_window()) {\n"
                b"        decorated = false;\n"
                b"      }\n"
                b"      this._set_decorated(decorated);\n",
                1,
            ).replace(
                b"      this.decorations = Boolean(metadata[\"decorations\"]);\n"
                b"      this._set_decorated(this.decorations);\n",
                b"      this.decorations = this.is_tie_robot_embedded_main_window() ? false : Boolean(metadata[\"decorations\"]);\n"
                b"      this._set_decorated(this.decorations);\n",
            ).replace(
                b"  is_desktop() {\n",
                b"  is_tie_robot_embedded_main_window() {\n"
                b"    if (this.tray || this.override_redirect || this.client.server_is_desktop || this.client.server_is_shadow) {\n"
                b"      return false;\n"
                b"    }\n"
                b"    if (this.has_windowtype([\"DIALOG\", \"UTILITY\", \"DROPDOWN\", \"TOOLTIP\", \"POPUP_MENU\", \"MENU\", \"COMBO\"])) {\n"
                b"      return false;\n"
                b"    }\n"
                b"    return this.windowtype.length === 0 || this.has_windowtype([\"NORMAL\"]);\n"
                b"  }\n\n"
                b"  is_desktop() {\n",
            )
            return patched_body, patched_body != body
        if path_only.endswith("/css/client.css"):
            style_patch = (
                b"\n"
                b"/* Tie Robot embedded graphical cards: native app content should fill the card. */\n"
                b".tie-robot-embedded-main-window {\n"
                b"  border: 0 !important;\n"
                b"  box-shadow: none !important;\n"
                b"  border-radius: 0 !important;\n"
                b"}\n"
                b".tie-robot-embedded-main-window > .windowhead {\n"
                b"  display: none !important;\n"
                b"}\n"
                b".tie-robot-embedded-main-window > canvas {\n"
                b"  border-radius: 0 !important;\n"
                b"}\n"
            )
            if style_patch.strip() in body:
                return body, False
            return body + style_patch, True
        return body, False

    def proxy_graphical_app_websocket(self, session, target_path):
        target_socket = None
        try:
            self.wait_for_graphical_app_port(session)
            target_socket = socket.create_connection(("127.0.0.1", int(session.web_port)), timeout=8)
            target_socket.settimeout(None)
            request_lines = [
                f"GET {target_path or '/'} HTTP/1.1\r\n",
            ]
            saw_host = False
            for key, value in self.headers.items():
                lower_key = key.lower()
                if lower_key == "host":
                    request_lines.append(f"Host: 127.0.0.1:{int(session.web_port)}\r\n")
                    saw_host = True
                    continue
                if lower_key == "proxy-connection":
                    continue
                request_lines.append(f"{key}: {value}\r\n")
            if not saw_host:
                request_lines.append(f"Host: 127.0.0.1:{int(session.web_port)}\r\n")
            request_lines.append("\r\n")
            target_socket.sendall("".join(request_lines).encode("iso-8859-1"))

            response = bytearray()
            while b"\r\n\r\n" not in response:
                chunk = target_socket.recv(4096)
                if not chunk:
                    break
                response.extend(chunk)
                if len(response) > 65536:
                    raise RuntimeError("websocket upgrade response is too large")
            if not response:
                raise RuntimeError("empty websocket upgrade response")
            self.connection.sendall(response)
            self._bridge_websocket_sockets(self.connection, target_socket)
        except Exception as exc:
            self.handle_graphical_app_session_unhealthy(session, f"图形界面 WebSocket 已断开：{exc}")
            if not self.wfile.closed:
                try:
                    self.send_error(502, f"graphical app websocket proxy failed: {exc}")
                except Exception:
                    pass
        finally:
            self.close_connection = True
            if target_socket is not None:
                try:
                    target_socket.close()
                except Exception:
                    pass

    def wait_for_graphical_app_port(self, session, timeout=12.0):
        deadline = time.time() + float(timeout)
        last_error = None
        while time.time() < deadline:
            if session.closed:
                raise RuntimeError("graphical app session is closed")
            if session.process and session.process.poll() is not None:
                raise RuntimeError(f"graphical app process exited with code {session.process.returncode}")
            if session._is_web_http_ready(timeout=0.6):
                return
            try:
                with socket.create_connection(("127.0.0.1", int(session.web_port)), timeout=0.3):
                    pass
            except OSError as exc:
                last_error = exc
            time.sleep(0.2)
        if last_error is not None:
            raise last_error
        raise TimeoutError("graphical app web service did not respond")

    def handle_graphical_app_session_unhealthy(self, session, message):
        if not session:
            return
        session.message = message
        if self.graphical_app_manager:
            closed_session = self.graphical_app_manager.close_session(session.session_id)
            if closed_session is not None:
                return
        session.close()

    def send_graphical_app_session_closed_page(self, session_id, message, status_code=410):
        session_id_text = str(session_id or "")
        message_text = str(message or "图形界面会话已结束，请重新打开。")
        script_payload = json.dumps({
            "type": "tie-robot-gui-session-closed",
            "sessionId": session_id_text,
            "message": message_text,
        }, ensure_ascii=False)
        body = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <title>图形界面已结束</title>
  <style>
    html, body {{
      margin: 0;
      width: 100%;
      height: 100%;
      display: grid;
      place-items: center;
      background: #05080c;
      color: #ebebf5;
      font-family: -apple-system, BlinkMacSystemFont, "Noto Sans SC", sans-serif;
    }}
    main {{
      max-width: 520px;
      padding: 28px;
      line-height: 1.7;
      text-align: center;
    }}
    h1 {{
      margin: 0 0 10px;
      font-size: 18px;
    }}
    p {{
      margin: 0;
      color: #a7aab3;
      font-size: 13px;
    }}
  </style>
</head>
<body>
  <main>
    <h1>图形界面已结束</h1>
    <p>{html.escape(message_text)}</p>
  </main>
  <script>
    try {{
      window.parent.postMessage({script_payload}, window.location.origin);
    }} catch (_error) {{}}
  </script>
</body>
</html>""".encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _bridge_websocket_sockets(self, client_socket, target_socket):
        client_socket.settimeout(None)
        sockets = [client_socket, target_socket]
        client_frame_buffer = bytearray()
        while True:
            readable, _, exceptional = select.select(sockets, [], sockets, 0.5)
            if exceptional:
                break
            if not readable:
                continue
            for source in readable:
                destination = target_socket if source is client_socket else client_socket
                try:
                    data = source.recv(65536)
                except OSError:
                    return
                if not data:
                    return
                if source is client_socket:
                    client_frame_buffer.extend(data)
                    if not self._forward_complete_websocket_frames(client_frame_buffer, destination):
                        return
                    continue
                try:
                    destination.sendall(data)
                except OSError:
                    return

    def _forward_complete_websocket_frames(self, frame_buffer, destination):
        while True:
            parsed = self._next_websocket_frame_size(frame_buffer)
            if parsed is None:
                return True
            frame_size, opcode = parsed
            frame = bytes(frame_buffer[:frame_size])
            del frame_buffer[:frame_size]
            try:
                destination.sendall(frame)
            except OSError:
                return False
            if opcode == 0x8:
                return False

    @staticmethod
    def _next_websocket_frame_size(frame_buffer):
        if len(frame_buffer) < 2:
            return None
        second = frame_buffer[1]
        payload_length = second & 0x7F
        header_size = 2
        if payload_length == 126:
            if len(frame_buffer) < 4:
                return None
            payload_length = int.from_bytes(frame_buffer[2:4], "big")
            header_size = 4
        elif payload_length == 127:
            if len(frame_buffer) < 10:
                return None
            payload_length = int.from_bytes(frame_buffer[2:10], "big")
            header_size = 10
        if second & 0x80:
            header_size += 4
        frame_size = header_size + payload_length
        if len(frame_buffer) < frame_size:
            return None
        opcode = frame_buffer[0] & 0x0F
        return frame_size, opcode

    def do_POST(self):
        parsed = urlsplit(self.path)
        if parsed.path == "/api/terminal/sessions":
            self.handle_terminal_session_create()
            return
        if parsed.path == "/api/gui/sessions":
            self.handle_graphical_app_session_create()
            return
        if parsed.path.startswith("/api/gui/sessions/") and parsed.path.endswith("/geometry"):
            self.handle_graphical_app_geometry_post(parsed)
            return
        if parsed.path == "/api/gb28181/config":
            self.handle_gb28181_config_post()
            return
        if parsed.path == "/api/network/ping":
            self.handle_network_ping_post()
            return
        if parsed.path == "/api/system/toggle_demo_mode":
            self.handle_demo_mode_toggle(parsed.path)
            return
        if parsed.path == "/api/system/enter_demo_mode":
            self.handle_demo_mode_enter(parsed.path)
            return
        if parsed.path == "/api/system/exit_demo_mode":
            self.handle_demo_mode_exit(parsed.path)
            return
        systemd_action = SYSTEMD_ROS_BACKEND_ACTIONS.get(parsed.path)
        if systemd_action is not None:
            self.handle_ros_backend_systemd_action(systemd_action, parsed.path)
            return
        driver_action = SYSTEMD_DRIVER_ACTIONS.get(parsed.path)
        if driver_action is not None:
            action, target = driver_action
            self.handle_driver_systemd_action(action, target, parsed.path)
            return
        script_path = SYSTEM_CONTROL_SCRIPTS.get(parsed.path)
        if script_path is None:
            self.send_error(404, "unknown api endpoint")
            return

        threading.Thread(
            target=self._run_control_script,
            args=(str(script_path), parsed.path),
            daemon=True,
        ).start()
        payload = json.dumps({
            "success": True,
            "message": f"已受理：{parsed.path}",
        }).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def handle_ros_backend_status_get(self):
        status_payload = self._query_ros_backend_status()
        self.send_json({
            "success": True,
            "message": f"{ROS_BACKEND_SERVICE} 状态：{status_payload.get('activeState', 'unknown')}/{status_payload.get('subState', 'unknown')}",
            "service": ROS_BACKEND_SERVICE,
            "status": status_payload,
        })

    def handle_demo_mode_status_get(self):
        payload = self._build_demo_mode_status_payload()
        self.send_json({
            "success": True,
            **payload,
        })

    def handle_demo_mode_toggle(self, api_path):
        demo_status = self._query_systemd_status(DEMO_MODE_SERVICE)
        if self._systemd_status_is_active(demo_status):
            self.handle_demo_mode_exit(api_path)
            return
        self.handle_demo_mode_enter(api_path)

    def handle_demo_mode_enter(self, api_path):
        completed_results = self._run_demo_mode_enter()
        self._send_demo_mode_transition_response(
            api_path,
            "enter",
            completed_results,
            "已进入演示模式：5173 旧前端、轻量 rosbridge 与 chassis_ctrl show_full.launch 已启动，当前后端、驱动、完整 rosbridge 和旧转义层已停止。",
        )

    def handle_demo_mode_exit(self, api_path):
        completed_results = self._run_demo_mode_exit()
        self._send_demo_mode_transition_response(
            api_path,
            "exit",
            completed_results,
            "已退出演示模式：show_full.launch 与轻量 rosbridge 已停止，本程序完整 rosbridge、驱动层和 ROS 后端已恢复。",
        )

    def _send_demo_mode_transition_response(self, api_path, action, completed_results, success_message):
        success = all(
            item["result"].returncode == 0
            for item in completed_results
        )
        if success:
            message = success_message
        else:
            failure_texts = [
                item["result"].stderr.strip() or item["result"].stdout.strip()
                for item in completed_results
                if item["result"].returncode != 0
            ]
            message = failure_texts[0] if failure_texts else "演示模式切换失败"

        status_payload = self._build_demo_mode_status_payload()
        self._log_demo_mode_control_result(api_path, action, completed_results, success)
        self.send_json({
            "success": success,
            "message": message,
            "action": action,
            "steps": [
                {
                    "phase": item["phase"],
                    "action": item["action"],
                    "services": list(item["services"]),
                    "returnCode": item["result"].returncode,
                }
                for item in completed_results
            ],
            **status_payload,
        }, status_code=200 if success else 503)

    def _build_demo_mode_status_payload(self):
        demo_status = self._query_systemd_status(DEMO_MODE_SERVICE)
        related_services = {
            service_name: self._query_systemd_status(service_name)
            for service_name in (
                DEMO_MODE_SERVICE,
                DEMO_ROSBRIDGE_SERVICE,
                LEGACY_FRONTEND_SERVICE,
                ROSBRIDGE_SERVICE,
                ROS_BACKEND_SERVICE,
                DRIVER_SYSTEMD_SERVICES["cabin"],
                DRIVER_SYSTEMD_SERVICES["moduan"],
                DRIVER_SYSTEMD_SERVICES["camera"],
                LEGACY_SHARED_DRIVER_STACK_SERVICE,
            )
        }
        active = self._systemd_status_is_active(demo_status)
        return {
            "active": active,
            "demoModeActive": active,
            "service": DEMO_MODE_SERVICE,
            "legacyFrontendUrl": self._build_legacy_frontend_url(),
            "status": demo_status,
            "services": related_services,
        }

    def _build_legacy_frontend_url(self):
        host = self._request_browser_host()
        return f"http://{host}:{DEMO_MODE_FRONTEND_PORT}/"

    def _request_browser_host(self):
        host_header = str(self.headers.get("Host") or "").strip()
        if not host_header:
            return DEFAULT_BROWSER_HOST
        if host_header.startswith("["):
            host = host_header[1:].split("]", 1)[0]
        else:
            host = host_header.split(":", 1)[0]
        host = host.strip()
        if not host or host == "0.0.0.0":
            return DEFAULT_BROWSER_HOST
        return host

    def _systemd_status_is_active(self, status_payload):
        return str((status_payload or {}).get("activeState") or "").lower() == "active"

    def _run_demo_mode_enter(self):
        results = []
        for service_name in (LEGACY_FRONTEND_SERVICE,):
            start_result = self._run_systemctl("start", service_name)
            results.append({
                "phase": "prepare",
                "action": "start",
                "services": (service_name,),
                "result": start_result,
            })
            if start_result.returncode != 0:
                return results

        for service_name in DEMO_MODE_STOP_ORDER:
            stop_result = self._run_demo_mode_stop_service(service_name)
            results.append({
                "phase": "stop-current",
                "action": "stop",
                "services": (service_name,),
                "result": stop_result,
            })
            if stop_result.returncode != 0:
                return results

        start_demo_bridge_result = self._run_systemctl("start", DEMO_ROSBRIDGE_SERVICE)
        results.append({
            "phase": "start-demo-rosbridge",
            "action": "start",
            "services": (DEMO_ROSBRIDGE_SERVICE,),
            "result": start_demo_bridge_result,
        })
        if start_demo_bridge_result.returncode != 0:
            return results

        cleanup_result = self._cleanup_demo_mode_ros_artifacts()
        results.append({
            "phase": "cleanup-demo",
            "action": "cleanup_demo_mode_ros",
            "services": (),
            "result": cleanup_result,
        })
        if cleanup_result.returncode != 0:
            return results

        start_result = self._run_systemctl("start", DEMO_MODE_SERVICE)
        results.append({
            "phase": "start-demo",
            "action": "start",
            "services": (DEMO_MODE_SERVICE,),
            "result": start_result,
        })
        return results

    def _run_demo_mode_exit(self):
        results = []
        stop_result = self._run_systemctl(
            "stop",
            DEMO_MODE_SERVICE,
            timeout=DEMO_MODE_STOP_TIMEOUT_SEC,
        )
        results.append({
            "phase": "stop-demo",
            "action": "stop",
            "services": (DEMO_MODE_SERVICE,),
            "result": stop_result,
        })
        if stop_result.returncode != 0:
            return results

        cleanup_result = self._cleanup_demo_mode_ros_artifacts()
        results.append({
            "phase": "cleanup-demo",
            "action": "cleanup_demo_mode_ros",
            "services": (),
            "result": cleanup_result,
        })
        if cleanup_result.returncode != 0:
            return results

        stop_demo_bridge_result = self._run_systemctl(
            "stop",
            DEMO_ROSBRIDGE_SERVICE,
            timeout=DEMO_MODE_STOP_TIMEOUT_SEC,
        )
        results.append({
            "phase": "stop-demo-rosbridge",
            "action": "stop",
            "services": (DEMO_ROSBRIDGE_SERVICE,),
            "result": stop_demo_bridge_result,
        })
        if stop_demo_bridge_result.returncode != 0:
            return results

        reset_result = self._run_demo_mode_reset_failed()
        results.append({
            "phase": "reset-demo",
            "action": "reset-failed",
            "services": (DEMO_MODE_SERVICE,),
            "result": reset_result,
        })
        if reset_result.returncode != 0:
            return results

        reset_demo_bridge_result = self._run_demo_mode_reset_failed(DEMO_ROSBRIDGE_SERVICE)
        results.append({
            "phase": "reset-demo-rosbridge",
            "action": "reset-failed",
            "services": (DEMO_ROSBRIDGE_SERVICE,),
            "result": reset_demo_bridge_result,
        })
        if reset_demo_bridge_result.returncode != 0:
            return results

        for service_name in DEMO_MODE_RESTORE_START_ORDER:
            start_result = self._run_systemctl("start", service_name)
            results.append({
                "phase": "restore-current",
                "action": "start",
                "services": (service_name,),
                "result": start_result,
            })
            if start_result.returncode != 0:
                break
        return results

    def _run_demo_mode_stop_service(self, service_name):
        if service_name in DEMO_MODE_OPTIONAL_STOP_SERVICES:
            status_payload = self._query_systemd_status(service_name)
            if str(status_payload.get("loadState") or "").lower() == "not-found":
                return subprocess.CompletedProcess(
                    ["systemctl", "stop", service_name],
                    0,
                    f"{service_name} 未安装，已跳过。",
                    "",
                )
        return self._run_systemctl(
            "stop",
            service_name,
            timeout=DEMO_MODE_STOP_TIMEOUT_SEC,
        )

    def _run_demo_mode_reset_failed(self, service_name=DEMO_MODE_SERVICE):
        reset_result = self._run_systemctl("reset-failed", service_name)
        if reset_result.returncode == 0:
            return reset_result
        status_payload = self._query_systemd_status(service_name)
        if (
            not self._systemd_status_is_active(status_payload)
            and str(status_payload.get("result") or "").lower() == "success"
        ):
            return subprocess.CompletedProcess(
                ["systemctl", "reset-failed", service_name],
                0,
                reset_result.stdout,
                reset_result.stderr,
            )
        return reset_result

    def _cleanup_demo_mode_ros_artifacts(self):
        initial_processes = self._find_demo_mode_processes()
        failures = []
        if initial_processes:
            failures.extend(self._signal_ros_processes(initial_processes, signal.SIGTERM))
            self._sleep(ROS_FAST_KILL_TERM_GRACE_SEC)
            remaining_processes = self._find_demo_mode_processes()
            if remaining_processes:
                failures.extend(self._signal_ros_processes(remaining_processes, signal.SIGKILL))
                self._sleep(ROS_FAST_KILL_KILL_GRACE_SEC)

        stubborn_processes = self._find_demo_mode_processes()
        cleanup_completed = self._run_rosnode_cleanup()
        if failures or stubborn_processes or cleanup_completed.returncode != 0:
            details = []
            details.extend(failures)
            if stubborn_processes:
                details.append(
                    "仍有演示模式残留进程: "
                    + ", ".join(f"{pid}:{command[:120]}" for pid, command in stubborn_processes)
                )
            if cleanup_completed.returncode != 0:
                details.append(cleanup_completed.stderr.strip() or cleanup_completed.stdout.strip())
            return subprocess.CompletedProcess(
                ["cleanup-demo-mode-ros"],
                1,
                cleanup_completed.stdout,
                "\n".join(text for text in details if text),
            )

        stdout_parts = []
        if initial_processes:
            stdout_parts.append(f"已清理 {len(initial_processes)} 个演示模式残留进程。")
        if cleanup_completed.stdout.strip():
            stdout_parts.append(cleanup_completed.stdout.strip())
        return subprocess.CompletedProcess(
            ["cleanup-demo-mode-ros"],
            0,
            "\n".join(stdout_parts) or "演示模式 ROS 残留清理完成。",
            "",
        )

    def _find_demo_mode_processes(self):
        try:
            completed = subprocess.run(
                ["ps", "-eo", "pid=,args="],
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            )
        except Exception as exc:
            FRONTEND_LOGGER.warning("failed to list demo mode ROS processes: %s", exc)
            return []

        if completed.returncode != 0:
            return []

        current_pid = os.getpid()
        candidates = []
        for line in completed.stdout.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            try:
                pid_text, command = stripped.split(None, 1)
                pid = int(pid_text)
            except ValueError:
                continue
            if pid == current_pid or "workspace_picker_web_server.py" in command:
                continue
            if any(pattern in command for pattern in DEMO_MODE_CLEANUP_PROCESS_PATTERNS):
                candidates.append((pid, command))
        return candidates

    def _run_rosnode_cleanup(self):
        try:
            return subprocess.run(
                ["rosnode", "cleanup"],
                input="y\n",
                check=False,
                capture_output=True,
                text=True,
                timeout=8,
            )
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout.decode("utf-8", errors="replace") if isinstance(exc.stdout, bytes) else str(exc.stdout or "")
            stderr = exc.stderr.decode("utf-8", errors="replace") if isinstance(exc.stderr, bytes) else str(exc.stderr or "")
            return subprocess.CompletedProcess(
                ["rosnode", "cleanup"],
                124,
                stdout,
                "\n".join(text for text in ("rosnode cleanup 超时。", stderr.strip()) if text),
            )
        except Exception as exc:
            return subprocess.CompletedProcess(
                ["rosnode", "cleanup"],
                1,
                "",
                str(exc),
            )

    def handle_ros_backend_systemd_action(self, action, api_path):
        if action == FULL_ROS_STACK_RESTART_ACTION:
            self.handle_full_ros_stack_restart(api_path)
            return

        completed = self._run_ros_backend_systemctl(action)
        status_payload = self._query_ros_backend_status()
        success = completed.returncode == 0
        action_label = {
            "start": "启动",
            "stop": "停止",
            "restart": "重启",
        }.get(action, action)
        message = f"已通过 systemd {action_label} ROS 后端：{ROS_BACKEND_SERVICE}"
        if not success:
            message = (
                completed.stderr.strip()
                or completed.stdout.strip()
                or f"systemctl {action} {ROS_BACKEND_SERVICE} 失败，退出码 {completed.returncode}"
            )
        self._log_systemd_control_result(api_path, action, completed, success)
        self.send_json({
            "success": success,
            "message": message,
            "service": ROS_BACKEND_SERVICE,
            "action": action,
            "status": status_payload,
        }, status_code=200 if success else 503)

    def _run_ros_backend_systemctl(self, action):
        return self._run_systemctl(action, ROS_BACKEND_SERVICE)

    def handle_full_ros_stack_restart(self, api_path):
        completed_results = self._run_full_ros_stack_restart()
        success = all(
            item["result"].returncode == 0
            for item in completed_results
        )
        status_payload = {
            service_name: self._query_systemd_status(service_name)
            for service_name in FULL_ROS_STACK_START_ORDER
        }
        if success:
            message = "已快速停止并重启 ROS 全栈：rosbridge、驱动层、ROS 后端。"
        else:
            failure_texts = [
                item["result"].stderr.strip() or item["result"].stdout.strip()
                for item in completed_results
                if item["result"].returncode != 0
            ]
            message = failure_texts[0] if failure_texts else "systemctl 重启 ROS 全栈失败"

        self._log_full_ros_stack_restart_result(api_path, completed_results, success)
        self.send_json({
            "success": success,
            "message": message,
            "services": list(FULL_ROS_STACK_START_ORDER),
            "action": FULL_ROS_STACK_RESTART_ACTION,
            "steps": [
                {
                    "phase": item["phase"],
                    "action": item["action"],
                    "services": list(item["services"]),
                    "returnCode": item["result"].returncode,
                }
                for item in completed_results
            ],
            "status": status_payload,
        }, status_code=200 if success else 503)

    def _run_full_ros_stack_restart(self):
        results = []
        stop_result = self._run_systemctl(
            "stop",
            *FULL_ROS_STACK_STOP_ORDER,
            timeout=FULL_ROS_STACK_STOP_TIMEOUT_SEC,
        )
        results.append({
            "phase": "stop",
            "action": "stop",
            "services": FULL_ROS_STACK_STOP_ORDER,
            "result": stop_result,
        })

        cleanup_result = self._fast_kill_stale_ros_processes()
        results.append({
            "phase": "cleanup",
            "action": "fast_kill_stale_ros",
            "services": (),
            "result": cleanup_result,
        })
        if stop_result.returncode != 0 or cleanup_result.returncode != 0:
            return results

        for service_name in FULL_ROS_STACK_START_ORDER:
            start_result = self._run_systemctl("start", service_name)
            results.append({
                "phase": "start",
                "action": "start",
                "services": (service_name,),
                "result": start_result,
            })
            if start_result.returncode != 0:
                break
        return results

    def _fast_kill_stale_ros_processes(self):
        initial_processes = self._find_stale_ros_processes()
        if not initial_processes:
            return subprocess.CompletedProcess(
                ["fast-kill-stale-ros"],
                0,
                "未发现残留 ROS 进程。",
                "",
            )

        failures = self._signal_ros_processes(initial_processes, signal.SIGTERM)
        self._sleep(ROS_FAST_KILL_TERM_GRACE_SEC)
        remaining_processes = self._find_stale_ros_processes()
        if remaining_processes:
            failures.extend(self._signal_ros_processes(remaining_processes, signal.SIGKILL))
            self._sleep(ROS_FAST_KILL_KILL_GRACE_SEC)

        stubborn_processes = self._find_stale_ros_processes()
        if failures or stubborn_processes:
            details = []
            if failures:
                details.extend(failures)
            if stubborn_processes:
                details.append(
                    "仍有残留 ROS 进程: "
                    + ", ".join(f"{pid}:{command[:120]}" for pid, command in stubborn_processes)
                )
            return subprocess.CompletedProcess(
                ["fast-kill-stale-ros"],
                1,
                "",
                "\n".join(details),
            )

        return subprocess.CompletedProcess(
            ["fast-kill-stale-ros"],
            0,
            f"已清理 {len(initial_processes)} 个旧 ROS 进程。",
            "",
        )

    def _find_stale_ros_processes(self):
        try:
            completed = subprocess.run(
                ["ps", "-eo", "pid=,args="],
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            )
        except Exception as exc:
            FRONTEND_LOGGER.warning("failed to list ROS processes before restart: %s", exc)
            return []

        if completed.returncode != 0:
            return []

        current_pid = os.getpid()
        candidates = []
        for line in completed.stdout.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            try:
                pid_text, command = stripped.split(None, 1)
                pid = int(pid_text)
            except ValueError:
                continue
            if pid == current_pid or "workspace_picker_web_server.py" in command:
                continue
            if any(pattern in command for pattern in ROS_FAST_KILL_PROCESS_PATTERNS):
                candidates.append((pid, command))
        return candidates

    def _signal_ros_processes(self, processes, signal_number):
        failures = []
        for pid, command in processes:
            try:
                os.kill(pid, signal_number)
            except ProcessLookupError:
                continue
            except PermissionError as exc:
                failures.append(f"无法终止 ROS 进程 {pid}: {exc} ({command[:120]})")
            except OSError as exc:
                if exc.errno != errno.ESRCH:
                    failures.append(f"终止 ROS 进程 {pid} 失败: {exc} ({command[:120]})")
        return failures

    def _sleep(self, seconds):
        time.sleep(seconds)

    def handle_driver_systemd_action(self, action, target, api_path):
        service_names = self._driver_service_names_for_target(target)
        completed_results = [
            self._run_driver_systemctl(action, service_name)
            for service_name in service_names
        ]
        success = all(result.returncode == 0 for result in completed_results)
        status_payload = {
            service_name: self._query_systemd_status(service_name)
            for service_name in service_names
        }
        action_label = {
            "start": "启动",
            "stop": "停止",
            "restart": "重启",
        }.get(action, action)
        target_label = {
            "all": "全部驱动",
            "cabin": "索驱驱动",
            "moduan": "线性模组驱动",
            "camera": "相机驱动",
        }.get(target, target)
        if success:
            message = f"已通过 systemd {action_label}{target_label}。"
        else:
            failure_texts = [
                result.stderr.strip() or result.stdout.strip()
                for result in completed_results
                if result.returncode != 0
            ]
            message = failure_texts[0] if failure_texts else f"systemctl {action} {target_label}失败"
        self._log_driver_control_result(api_path, action, service_names, completed_results, success)
        self.send_json({
            "success": success,
            "message": message,
            "services": service_names,
            "action": action,
            "target": target,
            "status": status_payload,
        }, status_code=200 if success else 503)

    def _driver_service_names_for_target(self, target):
        if target == "all":
            return [
                DRIVER_SYSTEMD_SERVICES["cabin"],
                DRIVER_SYSTEMD_SERVICES["moduan"],
                DRIVER_SYSTEMD_SERVICES["camera"],
            ]
        service_name = DRIVER_SYSTEMD_SERVICES.get(target)
        return [service_name] if service_name else []

    def _run_driver_systemctl(self, action, service_name):
        return self._run_systemctl(action, service_name)

    def _run_systemctl(self, action, *service_names, timeout=30):
        command = ["sudo", "-n", SYSTEMCTL_BIN, action, *service_names]
        try:
            return subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout.decode("utf-8", errors="replace") if isinstance(exc.stdout, bytes) else str(exc.stdout or "")
            stderr = exc.stderr.decode("utf-8", errors="replace") if isinstance(exc.stderr, bytes) else str(exc.stderr or "")
            timeout_message = f"systemctl {action} {' '.join(service_names)} 超时（>{timeout}s）。"
            return subprocess.CompletedProcess(
                command,
                124,
                stdout,
                "\n".join(text for text in (timeout_message, stderr.strip()) if text),
            )

    def _query_ros_backend_status(self):
        return self._query_systemd_status(ROS_BACKEND_SERVICE)

    def _query_systemd_status(self, service_name):
        command = [
            SYSTEMCTL_BIN,
            "show",
            service_name,
            "--property=LoadState",
            "--property=ActiveState",
            "--property=SubState",
            "--property=MainPID",
            "--property=Result",
            "--property=ExecMainStatus",
        ]
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
        except Exception as exc:
            return {
                "service": service_name,
                "loadState": "unknown",
                "activeState": "unknown",
                "subState": "unknown",
                "mainPid": 0,
                "result": "unknown",
                "execMainStatus": 0,
                "error": str(exc),
            }

        parsed = {}
        for line in completed.stdout.splitlines():
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            parsed[key] = value
        return {
            "service": service_name,
            "loadState": parsed.get("LoadState", "not-found" if completed.returncode else "unknown"),
            "activeState": parsed.get("ActiveState", "unknown"),
            "subState": parsed.get("SubState", "unknown"),
            "mainPid": int(parsed.get("MainPID") or 0),
            "result": parsed.get("Result", "unknown"),
            "execMainStatus": int(parsed.get("ExecMainStatus") or 0),
            "returnCode": completed.returncode,
        }

    def _log_systemd_control_result(self, api_path, action, completed, success):
        configure_standalone_logging()
        if success:
            FRONTEND_LOGGER.info(
                "workspace picker systemd control succeeded: %s -> systemctl %s %s",
                api_path,
                action,
                ROS_BACKEND_SERVICE,
            )
            return
        FRONTEND_LOGGER.error(
            "workspace picker systemd control failed: %s -> systemctl %s %s (code=%s, stderr=%s)",
            api_path,
            action,
            ROS_BACKEND_SERVICE,
            completed.returncode,
            completed.stderr.strip(),
        )

    def _log_full_ros_stack_restart_result(self, api_path, completed_results, success):
        configure_standalone_logging()
        step_text = " -> ".join(
            f"{item['action']}:{','.join(item['services'])}"
            for item in completed_results
        )
        if success:
            FRONTEND_LOGGER.info(
                "workspace picker full ROS restart succeeded: %s -> %s",
                api_path,
                step_text,
            )
            return
        errors = [
            item["result"].stderr.strip() or item["result"].stdout.strip()
            for item in completed_results
            if item["result"].returncode != 0
        ]
        FRONTEND_LOGGER.error(
            "workspace picker full ROS restart failed: %s -> %s (errors=%s)",
            api_path,
            step_text,
            " | ".join(errors),
        )

    def _log_driver_control_result(self, api_path, action, service_names, completed_results, success):
        configure_standalone_logging()
        service_text = ",".join(service_names)
        if success:
            FRONTEND_LOGGER.info(
                "workspace picker driver control succeeded: %s -> systemctl %s %s",
                api_path,
                action,
                service_text,
            )
            return
        errors = [
            result.stderr.strip() or result.stdout.strip()
            for result in completed_results
            if result.returncode != 0
        ]
        FRONTEND_LOGGER.error(
            "workspace picker driver control failed: %s -> systemctl %s %s (errors=%s)",
            api_path,
            action,
            service_text,
            " | ".join(errors),
        )

    def _log_demo_mode_control_result(self, api_path, action, completed_results, success):
        configure_standalone_logging()
        step_text = " -> ".join(
            f"{item['action']}:{','.join(item['services'])}"
            for item in completed_results
        )
        if success:
            FRONTEND_LOGGER.info(
                "workspace picker demo mode control succeeded: %s -> %s:%s",
                api_path,
                action,
                step_text,
            )
            return
        errors = [
            item["result"].stderr.strip() or item["result"].stdout.strip()
            for item in completed_results
            if item["result"].returncode != 0
        ]
        FRONTEND_LOGGER.error(
            "workspace picker demo mode control failed: %s -> %s:%s (errors=%s)",
            api_path,
            action,
            step_text,
            " | ".join(errors),
        )

    def do_DELETE(self):
        parsed = urlsplit(self.path)
        if parsed.path.startswith("/api/terminal/sessions/"):
            session_id = parsed.path.rsplit("/", 1)[-1]
            closed_session = self.terminal_manager.close_session(session_id) if self.terminal_manager else None
            if closed_session is None:
                self.send_error(404, "terminal session not found")
                return
            self.send_json({
                "success": True,
                "message": f"已关闭终端：{closed_session.label}",
                "session_id": closed_session.session_id,
            })
            return
        if parsed.path.startswith("/api/gui/sessions/"):
            session_id = parsed.path.rsplit("/", 1)[-1]
            closed_session = self.graphical_app_manager.close_session(session_id) if self.graphical_app_manager else None
            if closed_session is None:
                self.send_error(404, "graphical app session not found")
                return
            self.send_json({
                "success": True,
                "message": f"已关闭图形界面：{closed_session.label}",
                "session_id": closed_session.session_id,
            })
            return
        self.send_error(404, "unknown api endpoint")

    def _run_control_script(self, script_path, api_path):
        try:
            completed = subprocess.run(
                ["bash", script_path],
                check=False,
                capture_output=True,
                text=True,
            )
            if completed.returncode == 0:
                rospy.loginfo("workspace picker system control succeeded: %s -> %s", api_path, script_path)
                return
            rospy.logerr(
                "workspace picker system control failed: %s -> %s (code=%s, stderr=%s)",
                api_path,
                script_path,
                completed.returncode,
                completed.stderr.strip(),
            )
        except Exception as exc:
            rospy.logerr("workspace picker system control exception: %s -> %s (%s)", api_path, script_path, exc)

    def send_head(self):
        if getattr(self, "directory", None):
            self.path = resolve_clean_url_request_path(self.path, Path(self.directory))
        return super().send_head()

    def handle_terminal_session_create(self):
        if self.terminal_manager is None:
            self.send_error(503, "terminal manager unavailable")
            return
        payload = self.read_json_body()
        cols = payload.get("cols", DEFAULT_TERMINAL_COLS)
        rows = payload.get("rows", DEFAULT_TERMINAL_ROWS)
        try:
            session = self.terminal_manager.create_session(cols=cols, rows=rows)
        except Exception as exc:
            self.send_json({
                "success": False,
                "message": f"创建终端失败：{exc}",
            }, status_code=503)
            return
        self.send_json({
            "success": True,
            "message": f"已创建终端：{session.label}",
            "session": {
                "sessionId": session.session_id,
                "label": session.label,
                "cols": session.cols,
                "rows": session.rows,
                "terminalPort": int(self.terminal_port),
                "wsPath": f"/ws/terminal/{session.session_id}",
            },
        })

    def handle_graphical_app_session_create(self):
        if self.graphical_app_manager is None:
            self.send_error(503, "graphical app manager unavailable")
            return
        payload = self.read_json_body()
        command = payload.get("command", [])
        terminal_session_id = payload.get("terminal_session_id") or payload.get("terminalSessionId")
        try:
            session = self.graphical_app_manager.create_session(
                command,
                terminal_session_id=terminal_session_id,
            )
        except Exception as exc:
            self.send_json({
                "success": False,
                "message": f"创建图形界面失败：{exc}",
            }, status_code=400)
            return

        success = session.state != "error"
        self.send_json({
            "success": success,
            "message": (
                f"已在前端打开图形界面卡片：{session.label}"
                if success else session.message
            ),
            "session": session.snapshot(),
        }, status_code=200 if success else 503)

    def handle_graphical_app_geometry_post(self, parsed):
        if self.graphical_app_manager is None:
            self.send_error(503, "graphical app manager unavailable")
            return
        prefix = "/api/gui/sessions/"
        suffix = "/geometry"
        session_id = unquote(parsed.path[len(prefix):-len(suffix)].strip("/"))
        session = self.graphical_app_manager.get_session(session_id)
        if not session:
            self.send_error(404, "graphical app session not found")
            return

        payload = self.read_json_body()
        try:
            width = int(payload.get("width") or 0)
            height = int(payload.get("height") or 0)
        except (TypeError, ValueError):
            self.send_json({
                "success": False,
                "message": "图形窗口尺寸参数无效。",
            }, status_code=400)
            return

        success, message = session.fit_to_viewport(width, height)
        self.send_json({
            "success": success,
            "message": message,
            "session": session.snapshot(),
            "width": width,
            "height": height,
        }, status_code=200 if success else 503)

    def handle_planning_bind_path_get(self):
        if not PLANNING_BIND_PATH_FILE.exists():
            self.send_json({
                "success": False,
                "message": "pseudo_slam_bind_path.json 不存在。",
                "bind_path": None,
            }, status_code=404)
            return

        try:
            bind_path = json.loads(PLANNING_BIND_PATH_FILE.read_text(encoding="utf-8"))
        except Exception as exc:
            self.send_json({
                "success": False,
                "message": f"读取 pseudo_slam_bind_path.json 失败：{exc}",
                "bind_path": None,
            }, status_code=500)
            return

        self.send_json({
            "success": True,
            "bind_path": bind_path,
        })

    def handle_gb28181_config_post(self):
        if not GB28181_CONFIG_FILE.parent.exists():
            self.send_json({
                "success": False,
                "message": "本机还没有安装国标视频网关，暂时无法写入配置。",
                "config_path": str(GB28181_CONFIG_FILE),
            }, status_code=404)
            return

        try:
            sip_config = normalize_gb28181_sip_payload(self.read_json_body())
        except ValueError as exc:
            self.send_json({
                "success": False,
                "message": str(exc),
                "config_path": str(GB28181_CONFIG_FILE),
            }, status_code=400)
            return

        temp_path = None
        try:
            merged_text = merge_gb28181_sip_config(GB28181_CONFIG_FILE, sip_config)
            with tempfile.NamedTemporaryFile(
                "w",
                encoding="utf-8",
                dir=str(GB28181_CONFIG_FILE.parent),
                delete=False,
            ) as temp_file:
                temp_file.write(merged_text)
                temp_path = Path(temp_file.name)
            os.replace(temp_path, GB28181_CONFIG_FILE)
        except Exception as exc:
            if temp_path is not None:
                try:
                    temp_path.unlink(missing_ok=True)
                except Exception:
                    pass
            self.send_json({
                "success": False,
                "message": f"写入本机配置失败：{exc}",
                "config_path": str(GB28181_CONFIG_FILE),
            }, status_code=500)
            return

        self.send_json({
            "success": True,
            "message": "已写入本机 GB28181 接入配置。",
            "config_path": str(GB28181_CONFIG_FILE),
        })

    def handle_network_ping_post(self):
        payload = self.read_json_body()
        raw_host = payload.get("host") or payload.get("ip") or payload.get("target")
        try:
            result = run_network_ping(raw_host)
        except ValueError as exc:
            self.send_json({
                "success": False,
                "reachable": False,
                "message": str(exc),
                "target": str(raw_host or "").strip(),
            }, status_code=400)
            return

        target_label = str(payload.get("label") or payload.get("targetId") or "").strip()
        result["label"] = target_label
        if not result.get("message"):
            state_text = "可达" if result.get("reachable") else "不可达"
            result["message"] = f"{result['target']} {state_text}。"
        self.send_json(result)

    def read_json_body(self):
        try:
            content_length = int(self.headers.get("Content-Length", "0") or "0")
        except ValueError:
            return {}
        if content_length <= 0:
            return {}
        raw_body = self.rfile.read(content_length)
        if not raw_body:
            return {}
        try:
            return json.loads(raw_body.decode("utf-8"))
        except Exception:
            return {}

    def send_json(self, payload, status_code=200):
        response = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


def iter_candidate_ports(preferred_port, search_count=DEFAULT_PORT_SEARCH_COUNT):
    yield preferred_port
    if preferred_port < DEFAULT_NON_PRIVILEGED_FALLBACK_PORT:
        start_port = DEFAULT_NON_PRIVILEGED_FALLBACK_PORT
    else:
        start_port = preferred_port + 1
    for candidate_port in range(start_port, start_port + search_count):
        if candidate_port != preferred_port:
            yield candidate_port


def bind_http_server(host, preferred_port, handler, search_count=DEFAULT_PORT_SEARCH_COUNT):
    last_exc = None
    for candidate_port in iter_candidate_ports(preferred_port, search_count=search_count):
        try:
            server = ThreadingHTTPServer((host, candidate_port), handler)
            server.daemon_threads = True
            return server, candidate_port
        except OSError as exc:
            if exc.errno not in (errno.EADDRINUSE, errno.EACCES, getattr(errno, "EPERM", errno.EACCES)):
                raise
            last_exc = exc
    if last_exc is not None:
        raise last_exc
    raise OSError(errno.EADDRINUSE, "未找到可用端口")


def parse_args():
    parser = argparse.ArgumentParser(description="Tie Robot frontend static web server")
    parser.add_argument("--no-ros", action="store_true", help="run without registering a ROS node")
    parser.add_argument("--host", default=None, help="HTTP bind host")
    parser.add_argument("--port", type=int, default=None, help="preferred HTTP port")
    parser.add_argument("--terminal-port", type=int, default=None, help="preferred terminal websocket port")
    parser.add_argument("--browser-host", default=None, help="host name or IP shown in generated frontend URL")
    parser.add_argument("--port-search-count", type=int, default=None, help="number of fallback ports to try")
    return parser.parse_args()


def log_frontend(run_without_ros, level, message, *args):
    if run_without_ros:
        configure_standalone_logging()
        level_number = logging.WARNING if level == "WARN" else getattr(logging, level, logging.INFO)
        FRONTEND_LOGGER.log(level_number, message, *args)
        return
    if level == "WARN":
        rospy.logwarn(message, *args)
    elif level == "ERROR":
        rospy.logerr(message, *args)
    else:
        rospy.loginfo(message, *args)


def get_frontend_config(args, run_without_ros):
    if run_without_ros:
        return {
            "host": args.host or "0.0.0.0",
            "port": args.port if args.port is not None else DEFAULT_WORKSPACE_PICKER_PORT,
            "terminal_port": args.terminal_port if args.terminal_port is not None else DEFAULT_TERMINAL_PORT,
            "browser_host": args.browser_host or DEFAULT_BROWSER_HOST,
            "search_count": args.port_search_count if args.port_search_count is not None else DEFAULT_PORT_SEARCH_COUNT,
        }
    return {
        "host": args.host or rospy.get_param("~host", "0.0.0.0"),
        "port": args.port if args.port is not None else int(rospy.get_param("~port", DEFAULT_WORKSPACE_PICKER_PORT)),
        "terminal_port": args.terminal_port if args.terminal_port is not None else int(rospy.get_param("~terminal_port", DEFAULT_TERMINAL_PORT)),
        "browser_host": args.browser_host or rospy.get_param("~browser_host", DEFAULT_BROWSER_HOST),
        "search_count": args.port_search_count if args.port_search_count is not None else int(rospy.get_param("~port_search_count", DEFAULT_PORT_SEARCH_COUNT)),
    }


def set_frontend_ros_params(run_without_ros, actual_port, actual_url, actual_terminal_port):
    if run_without_ros:
        return
    rospy.set_param(WORKSPACE_PICKER_PORT_PARAM, actual_port)
    rospy.set_param(WORKSPACE_PICKER_URL_PARAM, actual_url)
    rospy.set_param(WORKSPACE_PICKER_TERMINAL_PORT_PARAM, actual_terminal_port)


def clear_frontend_ros_params(run_without_ros):
    if run_without_ros:
        return
    for param_name in (WORKSPACE_PICKER_PORT_PARAM, WORKSPACE_PICKER_URL_PARAM, WORKSPACE_PICKER_TERMINAL_PORT_PARAM):
        try:
            rospy.delete_param(param_name)
        except Exception:
            pass


def wait_without_ros_shutdown(shutdown_server):
    shutdown_event = threading.Event()

    def handle_shutdown(_signum, _frame):
        shutdown_event.set()

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)
    try:
        while not shutdown_event.wait(0.5):
            pass
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        shutdown_server()


def main():
    args = parse_args()
    run_without_ros = args.no_ros
    if not run_without_ros:
        rospy.init_node("workspace_picker_web_server", anonymous=False)

    frontend_dir = get_frontend_dir()
    if not frontend_dir.exists():
        log_frontend(run_without_ros, "ERROR", "工作区选点前端目录不存在: %s", frontend_dir)
        return

    config = get_frontend_config(args, run_without_ros=run_without_ros)
    host = config["host"]
    port = int(config["port"])
    terminal_port = int(config["terminal_port"])
    browser_host = config["browser_host"]
    search_count = int(config["search_count"])

    terminal_manager = TerminalSessionManager()
    graphical_app_manager = GraphicalAppSessionManager(terminal_manager=terminal_manager)
    terminal_server = TerminalServerThread(
        host=host,
        preferred_port=terminal_port,
        search_count=search_count,
        manager=terminal_manager,
    )
    actual_terminal_port = terminal_server.start()
    NoCacheStaticHandler.terminal_manager = terminal_manager
    NoCacheStaticHandler.graphical_app_manager = graphical_app_manager
    NoCacheStaticHandler.terminal_port = actual_terminal_port

    handler = partial(NoCacheStaticHandler, directory=str(frontend_dir))
    server, actual_port = bind_http_server(host, port, handler, search_count=search_count)
    actual_url = build_browser_url(browser_host, actual_port)
    terminal_manager.set_gui_api_url(f"http://127.0.0.1:{actual_port}/api/gui/sessions")

    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    if actual_port != port:
        log_frontend(run_without_ros, "WARN", "工作区选点前端默认端口 %d 不可用，自动切换到 %d", port, actual_port)

    set_frontend_ros_params(run_without_ros, actual_port, actual_url, actual_terminal_port)

    log_frontend(
        run_without_ros,
        "INFO",
        "workspace picker web server started: serving %s at %s (terminal ws port=%s)",
        frontend_dir,
        actual_url,
        actual_terminal_port,
    )

    def shutdown_server():
        try:
            server.shutdown()
            server.server_close()
        except Exception:
            pass
        try:
            graphical_app_manager.shutdown()
        except Exception:
            pass
        try:
            terminal_server.stop()
        except Exception:
            pass
        clear_frontend_ros_params(run_without_ros)

    if run_without_ros:
        wait_without_ros_shutdown(shutdown_server)
    else:
        rospy.on_shutdown(shutdown_server)
        rospy.spin()


if __name__ == "__main__":
    main()
