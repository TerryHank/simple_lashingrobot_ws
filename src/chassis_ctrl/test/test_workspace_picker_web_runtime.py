#!/usr/bin/env python3

import importlib.util
import socket
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
SERVER_SCRIPT = WORKSPACE_ROOT / "chassis_ctrl" / "scripts" / "workspace_picker_web_server.py"
OPEN_SCRIPT = WORKSPACE_ROOT / "chassis_ctrl" / "scripts" / "workspace_picker_web_open.py"


def load_module(module_name, script_path):
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class WorkspacePickerWebRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server_module = load_module("workspace_picker_web_server_runtime_test", SERVER_SCRIPT)
        cls.open_module = load_module("workspace_picker_web_open_runtime_test", OPEN_SCRIPT)

    def test_bind_http_server_falls_forward_when_preferred_port_is_busy(self):
        busy_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        busy_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        busy_socket.bind(("127.0.0.1", 0))
        busy_socket.listen(1)
        preferred_port = busy_socket.getsockname()[1]

        class DummyHandler:
            def __init__(self, *args, **kwargs):
                pass

        try:
            self.assertTrue(hasattr(self.server_module, "bind_http_server"))
            server, actual_port = self.server_module.bind_http_server("127.0.0.1", preferred_port, DummyHandler)
            self.assertNotEqual(actual_port, preferred_port)
            self.assertGreater(actual_port, preferred_port)
            server.server_close()
        finally:
            busy_socket.close()

    def test_wait_for_picker_url_prefers_actual_port_param_when_available(self):
        checked_urls = []
        params = {"/workspace_picker_web/port": 8768}

        def fake_get_param(name, default=None):
            return params.get(name, default)

        def fake_url_ready(url):
            checked_urls.append(url)
            return url.endswith(":8768/index.html")

        self.assertTrue(hasattr(self.open_module, "wait_for_picker_url"))
        target_url = self.open_module.wait_for_picker_url(
            browser_host="127.0.0.1",
            fallback_port=8765,
            timeout_sec=0.1,
            get_param=fake_get_param,
            url_ready=fake_url_ready,
            sleep_fn=lambda _seconds: None,
            is_shutdown=lambda: False,
        )
        self.assertEqual(target_url, "http://127.0.0.1:8768/index.html")
        self.assertIn("http://127.0.0.1:8768/index.html", checked_urls)


if __name__ == "__main__":
    unittest.main()
