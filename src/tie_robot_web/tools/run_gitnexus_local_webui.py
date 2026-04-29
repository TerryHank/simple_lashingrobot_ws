#!/usr/bin/env python3

import os
import http.server
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

try:
    import rospy
except ImportError:
    rospy = None


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
GITNEXUS_REPO_URL = "https://github.com/abhigyanpatwari/GitNexus.git"
DEFAULT_WEBUI_ROOT = Path.home() / ".cache" / "tie_robot_web" / "GitNexus"
WEBUI_ROOT = Path(os.environ.get("GITNEXUS_WEBUI_DIR", DEFAULT_WEBUI_ROOT)).expanduser()
WEBUI_DIR = WEBUI_ROOT / "gitnexus-web"
BRIDGE_HOST = os.environ.get("GITNEXUS_BRIDGE_HOST", "0.0.0.0")
BRIDGE_PORT = int(os.environ.get("GITNEXUS_BRIDGE_PORT", "4747"))
INTERNAL_BRIDGE_HOST = os.environ.get("GITNEXUS_INTERNAL_BRIDGE_HOST", "127.0.0.1")
INTERNAL_BRIDGE_PORT = int(os.environ.get("GITNEXUS_INTERNAL_BRIDGE_PORT", str(BRIDGE_PORT + 1)))
WEBUI_HOST = os.environ.get("GITNEXUS_WEBUI_HOST", "0.0.0.0")
WEBUI_PORT = int(os.environ.get("GITNEXUS_WEBUI_PORT", "5173"))
PROJECT_NAME = os.environ.get("GITNEXUS_PROJECT", "simple_lashingrobot_ws")
HOP_BY_HOP_HEADERS = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailers",
    "transfer-encoding",
    "upgrade",
    "host",
    "origin",
    "access-control-request-method",
    "access-control-request-headers",
    "access-control-request-private-network",
}
WEBUI_BACKEND_URL_OLD = """const [backendUrl] = useState<string>(() => {
    try {
      return localStorage.getItem(LS_URL_KEY) ?? DEFAULT_BACKEND_URL;
    } catch {
      return DEFAULT_BACKEND_URL;
    }
  });"""
WEBUI_BACKEND_URL_PATCH = """const [backendUrl] = useState<string>(() => {
    try {
      const serverParam = new URLSearchParams(window.location.search).get('server');
      if (serverParam) {
        localStorage.setItem(LS_URL_KEY, serverParam);
        return serverParam;
      }
      return localStorage.getItem(LS_URL_KEY) ?? DEFAULT_BACKEND_URL;
    } catch {
      return DEFAULT_BACKEND_URL;
    }
  });"""


def detect_lan_host():
    try:
        output = subprocess.check_output(
            ["hostname", "-I"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=0.5,
        )
        for token in output.split():
            if token and ":" not in token and not token.startswith("127."):
                return token
    except (OSError, subprocess.SubprocessError):
        pass

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("8.8.8.8", 80))
            return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"


PUBLIC_HOST = os.environ.get("GITNEXUS_PUBLIC_HOST", detect_lan_host())


def is_rospy_ready():
    return rospy is not None and rospy.core.is_initialized()


def write_ros_style_log(level, message, stream):
    stream.write(f"[{level}] [{time.time():.9f}]: [gitnexus-webui] {message}\n")
    stream.flush()


def log_info(message):
    if is_rospy_ready():
        rospy.loginfo("[gitnexus-webui] %s", message)
        return
    write_ros_style_log("INFO", message, sys.stdout)


def log_error(message):
    if is_rospy_ready():
        rospy.logerr("[gitnexus-webui] %s", message)
        return
    write_ros_style_log("ERROR", message, sys.stderr)


def get_connect_host(host):
    if host in {"0.0.0.0", "::"}:
        return "127.0.0.1"
    return host


def get_public_host(host):
    if host in {"0.0.0.0", "::"}:
        return PUBLIC_HOST
    return host


def is_port_open(host, port):
    try:
        with socket.create_connection((get_connect_host(host), port), timeout=0.35):
            return True
    except OSError:
        return False


def find_available_port(host, preferred_port):
    port = preferred_port
    while is_port_open(host, port):
        port += 1
    return port


class ReusableThreadingHTTPServer(http.server.ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class GitNexusCorsProxyHandler(http.server.BaseHTTPRequestHandler):
    target_host = "127.0.0.1"
    target_port = 4748
    protocol_version = "HTTP/1.0"

    def log_message(self, fmt, *args):
        log_info(f"proxy {self.address_string()} - {fmt % args}")

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_cors_headers()
        self.end_headers()

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/api/heartbeat":
            self.stream_heartbeat()
            return
        self.forward_request()

    def do_POST(self):
        self.forward_request()

    def do_PUT(self):
        self.forward_request()

    def do_PATCH(self):
        self.forward_request()

    def do_DELETE(self):
        self.forward_request()

    def send_cors_headers(self):
        origin = self.headers.get("Origin")
        self.send_header("Access-Control-Allow-Origin", origin or "*")
        self.send_header("Vary", "Origin")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, PUT, PATCH, DELETE, OPTIONS")
        requested_headers = self.headers.get("Access-Control-Request-Headers")
        self.send_header("Access-Control-Allow-Headers", requested_headers or "Content-Type, Authorization")

    def stream_heartbeat(self):
        self.send_response(200)
        self.send_cors_headers()
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        try:
            while True:
                self.wfile.write(b": gitnexus-webui bridge heartbeat\n\n")
                self.wfile.flush()
                time.sleep(15)
        except (BrokenPipeError, ConnectionResetError):
            return

    def forward_request(self):
        content_length = int(self.headers.get("Content-Length") or "0")
        body = self.rfile.read(content_length) if content_length > 0 else None
        target_url = f"http://{self.target_host}:{self.target_port}{self.path}"
        headers = {
            key: value
            for key, value in self.headers.items()
            if key.lower() not in HOP_BY_HOP_HEADERS
        }
        headers["Host"] = f"{self.target_host}:{self.target_port}"
        request = urllib.request.Request(
            target_url,
            data=body,
            headers=headers,
            method=self.command,
        )

        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            response = opener.open(request, timeout=300)
        except urllib.error.HTTPError as error:
            response = error
        except OSError as error:
            message = f"GitNexus backend proxy failed: {error}".encode("utf-8")
            self.send_response(502)
            self.send_cors_headers()
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(message)))
            self.end_headers()
            self.wfile.write(message)
            return

        with response:
            self.send_response(response.status)
            self.send_cors_headers()
            for key, value in response.headers.items():
                if key.lower() in HOP_BY_HOP_HEADERS or key.lower().startswith("access-control-"):
                    continue
                self.send_header(key, value)
            self.end_headers()
            shutil.copyfileobj(response, self.wfile)


def run_cors_proxy(bind_host, bind_port, target_host, target_port):
    handler_class = type(
        "ConfiguredGitNexusCorsProxyHandler",
        (GitNexusCorsProxyHandler,),
        {
            "target_host": target_host,
            "target_port": int(target_port),
        },
    )
    server = ReusableThreadingHTTPServer((bind_host, int(bind_port)), handler_class)
    log_info(
        f"CORS proxy listening at http://{get_public_host(bind_host)}:{bind_port} "
        f"-> http://{target_host}:{target_port}"
    )
    try:
        server.serve_forever()
    finally:
        server.server_close()


def run_checked(command, cwd=None):
    log_info(f"$ {' '.join(command)}")
    subprocess.run(command, cwd=cwd, check=True)


def ensure_webui_source():
    if WEBUI_DIR.exists():
        return
    WEBUI_ROOT.parent.mkdir(parents=True, exist_ok=True)
    if WEBUI_ROOT.exists() and not WEBUI_DIR.exists():
        raise RuntimeError(f"{WEBUI_ROOT} exists but does not look like a GitNexus checkout")
    run_checked(["git", "clone", "--depth", "1", GITNEXUS_REPO_URL, str(WEBUI_ROOT)])


def ensure_webui_dependencies():
    if (WEBUI_DIR / "node_modules").exists():
        return
    run_checked(["npm", "install"], cwd=str(WEBUI_DIR))


def patch_webui_backend_url_race():
    use_backend_path = WEBUI_DIR / "src" / "hooks" / "useBackend.ts"
    if not use_backend_path.exists():
        raise RuntimeError(f"GitNexus WebUI hook not found: {use_backend_path}")

    content = use_backend_path.read_text(encoding="utf-8")
    if "new URLSearchParams(window.location.search).get('server')" in content:
        return
    if WEBUI_BACKEND_URL_OLD not in content:
        raise RuntimeError(
            "GitNexus WebUI useBackend.ts changed; cannot apply LAN server parameter patch"
        )

    use_backend_path.write_text(
        content.replace(WEBUI_BACKEND_URL_OLD, WEBUI_BACKEND_URL_PATCH),
        encoding="utf-8",
    )
    log_info("patched GitNexus WebUI to prefer ?server= before localhost/localStorage")


def start_process(command, cwd=None):
    log_info(f"starting: {' '.join(command)}")
    return subprocess.Popen(command, cwd=cwd, start_new_session=True)


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)


def main():
    if len(sys.argv) >= 6 and sys.argv[1] == "--cors-proxy":
        run_cors_proxy(sys.argv[2], int(sys.argv[3]), sys.argv[4], int(sys.argv[5]))
        return 0

    if shutil.which("gitnexus") is None:
        log_error("gitnexus CLI not found. Install it with: npm install -g gitnexus")
        return 1
    if shutil.which("npm") is None:
        log_error("npm not found. GitNexus WebUI needs Node/npm.")
        return 1

    ensure_webui_source()
    patch_webui_backend_url_race()
    ensure_webui_dependencies()

    bridge_process = None
    bridge_proxy_process = None
    webui_process = None
    internal_bridge_port = find_available_port(INTERNAL_BRIDGE_HOST, INTERNAL_BRIDGE_PORT)
    public_bridge_host = get_public_host(BRIDGE_HOST)
    public_webui_host = get_public_host(WEBUI_HOST)
    local_webui_url = (
        f"http://{public_webui_host}:{WEBUI_PORT}/"
        f"?server=http://{public_bridge_host}:{BRIDGE_PORT}&project={PROJECT_NAME}"
    )

    try:
        if is_port_open(BRIDGE_HOST, BRIDGE_PORT):
            log_info(f"public bridge already running at http://{public_bridge_host}:{BRIDGE_PORT}")
        else:
            bridge_process = start_process(
                ["gitnexus", "serve", "--host", INTERNAL_BRIDGE_HOST, "--port", str(internal_bridge_port)],
                cwd=str(WORKSPACE_ROOT),
            )
            bridge_proxy_process = start_process(
                [sys.executable, __file__, "--cors-proxy", BRIDGE_HOST, str(BRIDGE_PORT), INTERNAL_BRIDGE_HOST, str(internal_bridge_port)]
            )

        if is_port_open(WEBUI_HOST, WEBUI_PORT):
            log_info(f"WebUI already running at http://{public_webui_host}:{WEBUI_PORT}")
        else:
            webui_process = start_process(
                ["npm", "run", "dev", "--", "--host", WEBUI_HOST, "--port", str(WEBUI_PORT)],
                cwd=str(WEBUI_DIR),
            )

        log_info("")
        log_info(f"open: {local_webui_url}")
        log_info("press Ctrl+C to stop processes started by this script")

        while True:
            time.sleep(0.5)
            if bridge_process is not None and bridge_process.poll() is not None:
                return bridge_process.returncode or 1
            if bridge_proxy_process is not None and bridge_proxy_process.poll() is not None:
                return bridge_proxy_process.returncode or 1
            if webui_process is not None and webui_process.poll() is not None:
                return webui_process.returncode or 1
    except KeyboardInterrupt:
        return 0
    finally:
        stop_process(webui_process)
        stop_process(bridge_proxy_process)
        stop_process(bridge_process)


if __name__ == "__main__":
    raise SystemExit(main())
