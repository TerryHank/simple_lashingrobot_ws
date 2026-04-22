#!/usr/bin/env python3

import errno
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rospy


WORKSPACE_PICKER_PORT_PARAM = "/workspace_picker_web/port"
WORKSPACE_PICKER_URL_PARAM = "/workspace_picker_web/url"
DEFAULT_BROWSER_HOST = "127.0.0.1"
DEFAULT_PORT_SEARCH_COUNT = 20


def get_frontend_dir():
    return Path(__file__).resolve().parents[2] / "ir_workspace_picker_web"


def build_browser_url(browser_host, port):
    return f"http://{browser_host}:{port}/index.html"


class NoCacheStaticHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


def bind_http_server(host, preferred_port, handler, search_count=DEFAULT_PORT_SEARCH_COUNT):
    last_exc = None
    for candidate_port in range(preferred_port, preferred_port + search_count + 1):
        try:
            server = ThreadingHTTPServer((host, candidate_port), handler)
            server.daemon_threads = True
            return server, candidate_port
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
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
    port = int(rospy.get_param("~port", 8765))
    browser_host = rospy.get_param("~browser_host", DEFAULT_BROWSER_HOST)
    search_count = int(rospy.get_param("~port_search_count", DEFAULT_PORT_SEARCH_COUNT))

    handler = partial(NoCacheStaticHandler, directory=str(frontend_dir))
    server, actual_port = bind_http_server(host, port, handler, search_count=search_count)
    actual_url = build_browser_url(browser_host, actual_port)

    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    if actual_port != port:
        rospy.logwarn("工作区选点前端端口 %d 已被占用，自动切换到 %d", port, actual_port)

    rospy.set_param(WORKSPACE_PICKER_PORT_PARAM, actual_port)
    rospy.set_param(WORKSPACE_PICKER_URL_PARAM, actual_url)

    rospy.loginfo(
        "workspace picker web server started: serving %s at http://%s:%d/index.html",
        frontend_dir,
        browser_host,
        actual_port,
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
