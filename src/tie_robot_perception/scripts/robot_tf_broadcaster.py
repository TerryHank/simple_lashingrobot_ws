#!/usr/bin/env python3
"""Continuously publish the robot TF chain owned by the TF layer."""

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from tie_robot_msgs.msg import cabin_upload


def finite_or_none(value):
    try:
        numeric_value = float(value)
    except (TypeError, ValueError):
        return None
    return numeric_value if math.isfinite(numeric_value) else None


def extract_pose_mm(msg):
    pose_mm = {
        "x": finite_or_none(msg.cabin_state_X),
        "y": finite_or_none(msg.cabin_state_Y),
        "z": finite_or_none(msg.cabin_state_Z),
    }
    if any(value is None for value in pose_mm.values()):
        return None
    return pose_mm


def build_transforms(map_frame, base_link_frame, scepter_frame, latest_pose_mm, stamp):
    base_transform = TransformStamped()
    base_transform.header.stamp = stamp
    base_transform.header.frame_id = map_frame
    base_transform.child_frame_id = base_link_frame
    base_transform.transform.translation.x = latest_pose_mm["x"] / 1000.0
    base_transform.transform.translation.y = latest_pose_mm["y"] / 1000.0
    base_transform.transform.translation.z = latest_pose_mm["z"] / 1000.0
    base_transform.transform.rotation.w = 1.0

    scepter_transform = TransformStamped()
    scepter_transform.header.stamp = stamp
    scepter_transform.header.frame_id = base_link_frame
    scepter_transform.child_frame_id = scepter_frame
    scepter_transform.transform.translation.x = 0.0
    scepter_transform.transform.translation.y = 0.0
    scepter_transform.transform.translation.z = 0.0

    scepter_quat = quaternion_from_euler(math.pi, 0.0, 0.0)
    scepter_transform.transform.rotation.x = scepter_quat[0]
    scepter_transform.transform.rotation.y = scepter_quat[1]
    scepter_transform.transform.rotation.z = scepter_quat[2]
    scepter_transform.transform.rotation.w = scepter_quat[3]

    return base_transform, scepter_transform


def main():
    rospy.init_node("robot_tf_broadcaster")

    map_frame = rospy.get_param("~map_frame", "map")
    base_link_frame = rospy.get_param("~base_link_frame", "base_link")
    scepter_frame = rospy.get_param("~scepter_frame", "Scepter_depth_frame")
    cabin_state_topic = rospy.get_param("~cabin_state_topic", "/cabin/cabin_data_upload")
    publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
    stale_warn_sec = float(rospy.get_param("~stale_warn_sec", 3.0))
    if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
        raise ValueError("publish_rate_hz must be positive")
    if not math.isfinite(stale_warn_sec) or stale_warn_sec <= 0.0:
        stale_warn_sec = 3.0

    latest_pose_mm = {"value": None}
    last_cabin_state_stamp = {"value": None}
    pose_lock = threading.Lock()

    def handle_cabin_state(msg):
        pose_mm = extract_pose_mm(msg)
        if pose_mm is None:
            rospy.logwarn_throttle(
                stale_warn_sec,
                "robot_tf_broadcaster: ignored invalid %s pose; keeping last valid TF pose.",
                cabin_state_topic,
            )
            return
        with pose_lock:
            latest_pose_mm["value"] = pose_mm
            last_cabin_state_stamp["value"] = rospy.Time.now()

    rospy.Subscriber(cabin_state_topic, cabin_upload, handle_cabin_state, queue_size=20)
    broadcaster = tf2_ros.TransformBroadcaster()
    rospy.loginfo(
        "robot_tf_broadcaster started: %s -> %s -> %s from %s",
        map_frame,
        base_link_frame,
        scepter_frame,
        cabin_state_topic,
    )

    rate = rospy.Rate(max(1.0, publish_rate_hz))
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        with pose_lock:
            pose_snapshot = dict(latest_pose_mm["value"]) if latest_pose_mm["value"] is not None else None
            last_stamp = last_cabin_state_stamp["value"]

        if pose_snapshot is None:
            rospy.logwarn_throttle(
                stale_warn_sec,
                "robot_tf_broadcaster: waiting for %s; not publishing robot pose TF yet.",
                cabin_state_topic,
            )
            rate.sleep()
            continue
        if last_stamp is None:
            rate.sleep()
            continue
        elif (now - last_stamp).to_sec() > stale_warn_sec:
            rospy.logwarn_throttle(
                stale_warn_sec,
                "robot_tf_broadcaster: %s is stale, keeping last TF pose.",
                cabin_state_topic,
            )

        base_transform, scepter_transform = build_transforms(
            map_frame,
            base_link_frame,
            scepter_frame,
            pose_snapshot,
            now,
        )
        broadcaster.sendTransform([base_transform, scepter_transform])
        rate.sleep()


if __name__ == "__main__":
    main()
