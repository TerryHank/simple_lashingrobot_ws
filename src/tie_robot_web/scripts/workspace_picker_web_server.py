#!/usr/bin/env python3

import errno
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

import rospy


WORKSPACE_PICKER_PORT_PARAM = "/workspace_picker_web/port"
WORKSPACE_PICKER_URL_PARAM = "/workspace_picker_web/url"
DEFAULT_BROWSER_HOST = "127.0.0.1"
DEFAULT_WORKSPACE_PICKER_PORT = 8080
DEFAULT_NON_PRIVILEGED_FALLBACK_PORT = 1024
DEFAULT_PORT_SEARCH_COUNT = 20


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
    if requested_file.exists():
        return request_path

    if Path(raw_path).suffix:
        return request_path

    html_candidate = requested_file.with_suffix(".html")
    if html_candidate.exists():
        return urlunsplit((parsed.scheme, parsed.netloc, f"{raw_path}.html", parsed.query, parsed.fragment))

    return request_path


class NoCacheStaticHandler(SimpleHTTPRequestHandler):
    def send_head(self):
        if getattr(self, "directory", None):
            self.path = resolve_clean_url_request_path(self.path, Path(self.directory))
        return super().send_head()

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
    browser_host = rospy.get_param("~browser_host", DEFAULT_BROWSER_HOST)
    search_count = int(rospy.get_param("~port_search_count", DEFAULT_PORT_SEARCH_COUNT))

    handler = partial(NoCacheStaticHandler, directory=str(frontend_dir))
    server, actual_port = bind_http_server(host, port, handler, search_count=search_count)
    actual_url = build_browser_url(browser_host, actual_port)

    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    if actual_port != port:
        rospy.logwarn("工作区选点前端默认端口 %d 不可用，自动切换到 %d", port, actual_port)

    rospy.set_param(WORKSPACE_PICKER_PORT_PARAM, actual_port)
    rospy.set_param(WORKSPACE_PICKER_URL_PARAM, actual_url)

    rospy.loginfo(
        "workspace picker web server started: serving %s at %s",
        frontend_dir,
        actual_url,
    )

    def shutdown_server():
        try:
            server.shutdown()
            server.server_close()
        except Exception:
            pass
        for param_name in (WORKSPACE_PICKER_PORT_PARAM, WORKSPACE_PICKER_URL_PARAM):
            try:
                rospy.delete_param(param_name)
            except Exception:
                pass

    rospy.on_shutdown(shutdown_server)
    rospy.spin()


if __name__ == "__main__":
    main()
