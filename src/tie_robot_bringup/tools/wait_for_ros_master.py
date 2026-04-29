#!/usr/bin/env python3
"""Wait until an existing ROS master answers XML-RPC calls."""

import argparse
import os
import sys
import time
import urllib.parse
import xmlrpc.client


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--master-uri",
        default=os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311"),
        help="ROS master URI to wait for.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=30.0,
        help="Maximum time to wait before failing.",
    )
    parser.add_argument(
        "--poll-sec",
        type=float,
        default=0.5,
        help="Polling interval.",
    )
    return parser.parse_args()


def validate_master_uri(master_uri):
    parsed = urllib.parse.urlparse(master_uri)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ValueError(f"invalid ROS_MASTER_URI: {master_uri!r}")


def master_is_ready(master_uri):
    proxy = xmlrpc.client.ServerProxy(master_uri)
    code, _message, _state = proxy.getSystemState("/tie_robot_wait_for_ros_master")
    return int(code) == 1


def main():
    args = parse_args()
    validate_master_uri(args.master_uri)
    deadline = time.monotonic() + max(0.1, args.timeout_sec)
    poll_sec = max(0.1, args.poll_sec)
    last_error = None

    while time.monotonic() < deadline:
        try:
            if master_is_ready(args.master_uri):
                print(f"ROS master ready: {args.master_uri}")
                return 0
        except Exception as exc:
            last_error = exc
        time.sleep(poll_sec)

    print(
        f"Timed out waiting for ROS master {args.master_uri}: {last_error}",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
