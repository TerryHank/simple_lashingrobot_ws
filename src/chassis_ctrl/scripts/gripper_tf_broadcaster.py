#!/usr/bin/env python3

import math
import os

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


def _read_string(data, key):
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return value.strip()


def _read_float(data, key_path):
    node = data
    for key in key_path:
        if key not in node:
            raise ValueError(f"missing {'.'.join(key_path)}")
        node = node[key]

    try:
        value = float(node)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{'.'.join(key_path)} must be numeric") from exc

    if not math.isfinite(value):
        raise ValueError(f"{'.'.join(key_path)} must be finite")
    return value


def _is_identity_pose(translation, rotation_rpy):
    values = [
        translation["x"],
        translation["y"],
        translation["z"],
        rotation_rpy["roll"],
        rotation_rpy["pitch"],
        rotation_rpy["yaw"],
    ]
    return all(abs(value) <= 1e-9 for value in values)


def _validate_radians(rotation_rpy):
    limit = (2.0 * math.pi) + 1e-6
    for axis, value in rotation_rpy.items():
        if abs(value) > limit:
            raise ValueError(
                f"rotation_rpy.{axis}={value} exceeds 2*pi radians; expected radians"
            )


def load_gripper_tf_config(config_path, allow_identity_config=False):
    with open(config_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    if "translation" in data:
        raise ValueError(
            "legacy field 'translation' is no longer supported; use 'translation_mm'"
        )

    for key in ("parent_frame", "child_frame", "translation_mm", "rotation_rpy"):
        if key not in data:
            raise ValueError(f"missing required key: {key}")

    translation_mm = {
        "x": _read_float(data, ("translation_mm", "x")),
        "y": _read_float(data, ("translation_mm", "y")),
        "z": _read_float(data, ("translation_mm", "z")),
    }
    config = {
        "parent_frame": _read_string(data, "parent_frame"),
        "child_frame": _read_string(data, "child_frame"),
        "translation_mm": translation_mm,
        "translation": {
            "x": -translation_mm["x"] / 1000.0,
            "y": -translation_mm["y"] / 1000.0,
            "z": -translation_mm["z"] / 1000.0,
        },
        "rotation_rpy": {
            "roll": _read_float(data, ("rotation_rpy", "roll")),
            "pitch": _read_float(data, ("rotation_rpy", "pitch")),
            "yaw": _read_float(data, ("rotation_rpy", "yaw")),
        },
    }

    if config["parent_frame"] == config["child_frame"]:
        raise ValueError("parent_frame and child_frame must be different")

    _validate_radians(config["rotation_rpy"])

    if not allow_identity_config and _is_identity_pose(
        config["translation"], config["rotation_rpy"]
    ):
        raise ValueError(
            "gripper_tf.yaml still uses the all-zero placeholder pose; calibrate the TF "
            "before launch or explicitly set ~allow_identity_config:=true"
        )

    return config


def build_transform(config):
    transform = TransformStamped()
    transform.header.frame_id = config["parent_frame"]
    transform.child_frame_id = config["child_frame"]
    transform.transform.translation.x = config["translation"]["x"]
    transform.transform.translation.y = config["translation"]["y"]
    transform.transform.translation.z = config["translation"]["z"]

    quat = quaternion_from_euler(
        config["rotation_rpy"]["roll"],
        config["rotation_rpy"]["pitch"],
        config["rotation_rpy"]["yaw"],
    )
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]
    return transform


def resolve_default_config_path():
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "gripper_tf.yaml")
    )


def main():
    rospy.init_node("gripper_tf_broadcaster")
    config_path = rospy.get_param("~config_path", resolve_default_config_path())
    allow_identity_config = bool(rospy.get_param("~allow_identity_config", False))

    try:
        publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be a positive finite number")
        config = load_gripper_tf_config(
            config_path, allow_identity_config=allow_identity_config
        )
    except Exception as exc:
        rospy.logfatal(
            "Failed to start gripper_tf_broadcaster with config %s: %s",
            config_path,
            exc,
        )
        raise SystemExit(1) from exc

    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(max(1.0, publish_rate_hz))

    rospy.loginfo(
        "gripper_tf_broadcaster started: %s -> %s from %s | user_translation_mm=(%.3f, %.3f, %.3f) | published_tf_translation_m=(%.6f, %.6f, %.6f) | rotation_rpy=(%.6f, %.6f, %.6f) rad",
        config["parent_frame"],
        config["child_frame"],
        config_path,
        config["translation_mm"]["x"],
        config["translation_mm"]["y"],
        config["translation_mm"]["z"],
        config["translation"]["x"],
        config["translation"]["y"],
        config["translation"]["z"],
        config["rotation_rpy"]["roll"],
        config["rotation_rpy"]["pitch"],
        config["rotation_rpy"]["yaw"],
    )

    while not rospy.is_shutdown():
        transform = build_transform(config)
        transform.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(transform)
        rate.sleep()


if __name__ == "__main__":
    main()
