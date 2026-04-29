#!/usr/bin/env python3

import json
import os
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath


DEFAULT_BIND_PATH_JSON = (
    "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/pseudo_slam_bind_path.json"
)


def _pose_from_xyz(x, y, z, stamp, frame_id):
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.w = 1.0
    return pose


def build_nav_path_from_bind_path(bind_path, stamp=None, frame_id="map"):
    if stamp is None:
        try:
            stamp = rospy.Time.now()
        except rospy.exceptions.ROSInitException:
            stamp = rospy.Time(0)
    path = NavPath()
    path.header.stamp = stamp
    path.header.frame_id = frame_id

    default_z = float(bind_path.get("cabin_height", 0.0))
    path_origin = bind_path.get("path_origin")
    if isinstance(path_origin, dict):
        path.poses.append(
            _pose_from_xyz(
                path_origin.get("x", 0.0),
                path_origin.get("y", 0.0),
                path_origin.get("z", default_z),
                stamp,
                frame_id,
            )
        )

    for area in bind_path.get("areas", []):
        if not isinstance(area, dict):
            continue
        cabin_pose = area.get("cabin_pose")
        if not isinstance(cabin_pose, dict):
            continue
        path.poses.append(
            _pose_from_xyz(
                cabin_pose.get("x", 0.0),
                cabin_pose.get("y", 0.0),
                cabin_pose.get("z", default_z),
                stamp,
                frame_id,
            )
        )
    return path


def load_bind_path_json(path):
    with Path(path).open(encoding="utf-8") as handle:
        return json.load(handle)


def main():
    rospy.init_node("global_bind_planner_node")
    bind_path_json = rospy.get_param("~bind_path_json", DEFAULT_BIND_PATH_JSON)
    publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 1.0))
    frame_id = "map"
    publisher = rospy.Publisher("/cabin/global_bind_path", NavPath, queue_size=1, latch=True)

    rate = rospy.Rate(max(publish_rate_hz, 0.1))
    last_mtime_ns = None
    while not rospy.is_shutdown():
        try:
            current_mtime_ns = os.stat(bind_path_json).st_mtime_ns
        except FileNotFoundError:
            rospy.logwarn_throttle(
                5.0,
                "GlobalBindPlanner_Warn: pseudo_slam_bind_path.json 不存在，等待扫描规划结果生成。",
            )
            rate.sleep()
            continue

        if current_mtime_ns != last_mtime_ns:
            try:
                bind_path = load_bind_path_json(bind_path_json)
                nav_path = build_nav_path_from_bind_path(bind_path, rospy.Time.now(), frame_id)
                publisher.publish(nav_path)
                last_mtime_ns = current_mtime_ns
                rospy.loginfo(
                    "GlobalBindPlanner_log: 已发布标准路径 /cabin/global_bind_path，poses=%d。",
                    len(nav_path.poses),
                )
            except Exception as exc:
                rospy.logerr(
                    "GlobalBindPlanner_Error: 发布标准路径 /cabin/global_bind_path 失败：%s",
                    exc,
                )

        rate.sleep()


if __name__ == "__main__":
    main()
