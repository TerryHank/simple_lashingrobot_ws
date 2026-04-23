#!/usr/bin/env python3

import collections
import errno
import fcntl
import json
import os
import pty
import select
import signal
import struct
import subprocess
import threading
import time
import uuid
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

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
WORKSPACE_ROOT = Path("/home/hyq-/simple_lashingrobot_ws")
SYSTEM_CONTROL_SCRIPTS = {
    "/api/system/start_ros_stack": WORKSPACE_ROOT / "start_.sh",
    "/api/system/start_driver_stack": WORKSPACE_ROOT / "start_driver_stack.sh",
    "/api/system/restart_driver_stack": WORKSPACE_ROOT / "restart.sh",
    "/api/system/start_algorithm_stack": WORKSPACE_ROOT / "start_algorithm_stack.sh",
    "/api/system/restart_algorithm_stack": WORKSPACE_ROOT / "restart_algorithm_stack.sh",
    "/api/system/restart_ros_stack": WORKSPACE_ROOT / "restart_ros_stack.sh",
}


def get_frontend_dir():
    return Path(__file__).resolve().parents[1] / "web"


def build_browser_url(browser_host, port):
    if int(port) == 80:
        return f"http://{browser_host}/index.html"
    return f"http://{browser_host}:{int(port)}/index.html"


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


class TerminalSession:
    def __init__(self, session_id, label, manager, cols=DEFAULT_TERMINAL_COLS, rows=DEFAULT_TERMINAL_ROWS):
        self.session_id = session_id
        self.label = label
        self.manager = manager
        self.cols = max(40, int(cols or DEFAULT_TERMINAL_COLS))
        self.rows = max(12, int(rows or DEFAULT_TERMINAL_ROWS))
        self.clients = set()
        self.buffer = collections.deque(maxlen=TERMINAL_BACKLOG_MAX_CHUNKS)
        self.buffer_chars = 0
        self.lock = threading.RLock()
        self.closed = False
        self.exit_code = None
        self.master_fd = None
        self.process = None
        self.reader_thread = None
        self._spawn()

    def _spawn(self):
        master_fd, slave_fd = pty.openpty()
        set_pty_window_size(master_fd, self.cols, self.rows)
        flags = fcntl.fcntl(master_fd, fcntl.F_GETFL)
        fcntl.fcntl(master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        env = os.environ.copy()
        env.setdefault("TERM", "xterm-256color")
        env.setdefault("COLORTERM", "truecolor")

        self.process = subprocess.Popen(
            ["/bin/bash", "-l"],
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            cwd=str(WORKSPACE_ROOT),
            env=env,
            close_fds=True,
            start_new_session=True,
        )
        os.close(slave_fd)
        self.master_fd = master_fd
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()

    def snapshot(self):
        return {
            "session_id": self.session_id,
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
        if self.closed or self.master_fd is None or not data:
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

    def close(self, terminate_process=True):
        if self.closed:
            return
        self.closed = True
        if terminate_process and self.process and self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception:
                pass
        try:
            if self.master_fd is not None:
                os.close(self.master_fd)
        except OSError:
            pass
        self.master_fd = None

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
            try:
                ready, _, _ = select.select([self.master_fd], [], [], 0.2)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            try:
                chunk = os.read(self.master_fd, 65536)
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

        if self.process and self.exit_code is None:
            try:
                self.exit_code = self.process.wait(timeout=0.5)
            except Exception:
                self.exit_code = -1
        self.closed = True
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

    def set_io_loop(self, io_loop):
        self.io_loop = io_loop

    def create_session(self, cols=DEFAULT_TERMINAL_COLS, rows=DEFAULT_TERMINAL_ROWS):
        with self.lock:
            self.sequence += 1
            session_id = uuid.uuid4().hex[:10]
            label = f"终端 {self.sequence}"
            session = TerminalSession(session_id, label, self, cols=cols, rows=rows)
            self.sessions[session_id] = session
            return session

    def list_sessions(self):
        with self.lock:
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
        if session.closed and not session.clients:
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
            session.close()
            self.broadcast(session_id, {"type": "closed"})
        return session

    def shutdown(self):
        with self.lock:
            sessions = list(self.sessions.values())
            self.sessions.clear()
        for session in sessions:
            session.close()

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
            self.io_loop.make_current()
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
    terminal_manager = None
    terminal_port = DEFAULT_TERMINAL_PORT

    def do_GET(self):
        parsed = urlsplit(self.path)
        if parsed.path == "/api/terminal/config":
            self.send_json({
                "success": True,
                "terminal_port": int(self.terminal_port),
                "ws_path_prefix": "/ws/terminal",
                "sessions": self.terminal_manager.list_sessions() if self.terminal_manager else [],
            })
            return
        return super().do_GET()

    def do_POST(self):
        parsed = urlsplit(self.path)
        if parsed.path == "/api/terminal/sessions":
            self.handle_terminal_session_create()
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
        session = self.terminal_manager.create_session(cols=cols, rows=rows)
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


def main():
    rospy.init_node("workspace_picker_web_server", anonymous=False)

    frontend_dir = get_frontend_dir()
    if not frontend_dir.exists():
        rospy.logerr("工作区选点前端目录不存在: %s", frontend_dir)
        return

    host = rospy.get_param("~host", "0.0.0.0")
    port = int(rospy.get_param("~port", DEFAULT_WORKSPACE_PICKER_PORT))
    terminal_port = int(rospy.get_param("~terminal_port", DEFAULT_TERMINAL_PORT))
    browser_host = rospy.get_param("~browser_host", DEFAULT_BROWSER_HOST)
    search_count = int(rospy.get_param("~port_search_count", DEFAULT_PORT_SEARCH_COUNT))

    terminal_manager = TerminalSessionManager()
    terminal_server = TerminalServerThread(
        host=host,
        preferred_port=terminal_port,
        search_count=search_count,
        manager=terminal_manager,
    )
    actual_terminal_port = terminal_server.start()
    NoCacheStaticHandler.terminal_manager = terminal_manager
    NoCacheStaticHandler.terminal_port = actual_terminal_port

    handler = partial(NoCacheStaticHandler, directory=str(frontend_dir))
    server, actual_port = bind_http_server(host, port, handler, search_count=search_count)
    actual_url = build_browser_url(browser_host, actual_port)

    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    if actual_port != port:
        rospy.logwarn("工作区选点前端默认端口 %d 不可用，自动切换到 %d", port, actual_port)

    rospy.set_param(WORKSPACE_PICKER_PORT_PARAM, actual_port)
    rospy.set_param(WORKSPACE_PICKER_URL_PARAM, actual_url)
    rospy.set_param(WORKSPACE_PICKER_TERMINAL_PORT_PARAM, actual_terminal_port)

    rospy.loginfo(
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
            terminal_server.stop()
        except Exception:
            pass
        for param_name in (WORKSPACE_PICKER_PORT_PARAM, WORKSPACE_PICKER_URL_PARAM):
            try:
                rospy.delete_param(param_name)
            except Exception:
                pass
        try:
            rospy.delete_param(WORKSPACE_PICKER_TERMINAL_PORT_PARAM)
        except Exception:
            pass

    rospy.on_shutdown(shutdown_server)
    rospy.spin()


if __name__ == "__main__":
    main()
