#!/usr/bin/env python3

import os
import shutil
import subprocess
import time
import urllib.request

import rospy


FRONTEND_ENTRY_RELATIVE_HINT = "ir_workspace_picker_web/index.html"
DEFAULT_BROWSER_HOST = "127.0.0.1"
DEFAULT_BROWSER_BASE_URL = "http://127.0.0.1"
WORKSPACE_PICKER_PORT_PARAM = "/workspace_picker_web/port"
WORKSPACE_PICKER_URL_PARAM = "/workspace_picker_web/url"


def can_open_browser():
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def build_target_url(browser_host, port):
    return f"http://{browser_host}:{int(port)}/index.html"


def url_is_ready(url):
    try:
        with urllib.request.urlopen(url, timeout=1.0):
            return True
    except Exception:
        return False


def wait_for_picker_url(
    browser_host,
    fallback_port,
    timeout_sec,
    get_param=None,
    url_ready=None,
    sleep_fn=None,
    is_shutdown=None,
):
    get_param = rospy.get_param if get_param is None else get_param
    url_ready = url_is_ready if url_ready is None else url_ready
    sleep_fn = time.sleep if sleep_fn is None else sleep_fn
    is_shutdown = rospy.is_shutdown if is_shutdown is None else is_shutdown

    deadline = time.time() + timeout_sec
    while time.time() < deadline and not is_shutdown():
        target_url = get_param(WORKSPACE_PICKER_URL_PARAM, "")
        if not target_url:
            actual_port = int(get_param(WORKSPACE_PICKER_PORT_PARAM, fallback_port))
            target_url = build_target_url(browser_host, actual_port)
        if url_ready(target_url):
            return target_url
        sleep_fn(0.5)
    return None


def main():
    rospy.init_node("workspace_picker_web_open", anonymous=False)

    port = int(rospy.get_param("~port", 8765))
    browser_host = rospy.get_param("~browser_host", DEFAULT_BROWSER_HOST)
    wait_timeout_sec = float(rospy.get_param("~wait_timeout_sec", 8.0))
    open_delay_sec = float(rospy.get_param("~open_delay_sec", 1.0))
    target_url = build_target_url(browser_host, port)

    if not can_open_browser():
        rospy.logwarn("未检测到桌面显示环境，跳过自动打开前端: %s", target_url)
        return

    xdg_open = shutil.which("xdg-open")
    if not xdg_open:
        rospy.logwarn("系统缺少 xdg-open，跳过自动打开前端: %s", target_url)
        return

    if open_delay_sec > 0.0:
        time.sleep(open_delay_sec)

    target_url = wait_for_picker_url(browser_host, port, wait_timeout_sec)
    if not target_url:
        target_url = build_target_url(browser_host, port)
        rospy.logwarn("前端页面未在超时内就绪，跳过自动打开: %s", target_url)
        return

    try:
        subprocess.Popen([xdg_open, target_url])
        rospy.loginfo("已请求打开工作区选点前端: %s", target_url)
    except Exception as exc:
        rospy.logwarn("自动打开工作区选点前端失败: %s", exc)


if __name__ == "__main__":
    main()
