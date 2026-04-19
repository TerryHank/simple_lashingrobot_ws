#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import time

import rospy
from std_msgs.msg import String


def read_plan_payload(plan_path: str) -> str:
    with open(plan_path, "r", encoding="utf-8") as handle:
        parsed = json.load(handle)
    return json.dumps(parsed, ensure_ascii=False)


def main():
    rospy.init_node("acceptance_plan_publisher", anonymous=False)

    plan_path = rospy.get_param(
        "~plan_path",
        "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json",
    )
    topic_name = rospy.get_param("~topic", "/acceptance/plan")
    poll_interval = float(rospy.get_param("~poll_interval", 1.0))

    publisher = rospy.Publisher(topic_name, String, queue_size=1, latch=True)
    last_mtime = None
    last_payload = None

    rospy.loginfo("acceptance_plan_publisher watching %s -> %s", plan_path, topic_name)

    while not rospy.is_shutdown():
        try:
            if not os.path.exists(plan_path):
                if last_payload is not None:
                    rospy.logwarn_throttle(30.0, "acceptance plan file missing: %s", plan_path)
                last_payload = None
                last_mtime = None
                time.sleep(poll_interval)
                continue

            mtime = os.path.getmtime(plan_path)
            if last_mtime == mtime and last_payload is not None:
                time.sleep(poll_interval)
                continue

            payload = read_plan_payload(plan_path)
            publisher.publish(String(data=payload))
            last_payload = payload
            last_mtime = mtime
            rospy.loginfo("published acceptance plan (%s bytes)", len(payload))
        except Exception as error:
            rospy.logwarn_throttle(10.0, "acceptance plan publish failed: %s", error)

        time.sleep(poll_interval)


if __name__ == "__main__":
    main()
