#!/usr/bin/env python3
"""Continuously publish the robot TF chain owned by the TF layer."""

import copy
import math
import os
import tempfile
import threading
from collections import deque
from types import SimpleNamespace

import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_matrix
from tie_robot_msgs.msg import cabin_upload

try:
    from tie_robot_msgs.srv import (
        RobotHomeCalibration,
        RobotHomeCalibrationResponse,
    )
except ImportError:  # Allows source-tree unit tests before catkin regenerates srv code.
    RobotHomeCalibration = None

    class RobotHomeCalibrationResponse(SimpleNamespace):
        pass


ROBOT_HOME_CALIBRATION_SERVICE = "/web/tf/robot_home_calibration"

DEFAULT_ROBOT_HOME_CONFIG = {
    "home_cabin_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
    "base_to_camera_mm": {"x": 0.0, "y": 0.0, "z": 460.0},
    "base_to_camera_rpy": {"roll": math.pi, "pitch": 0.0, "yaw": 0.0},
    "cabin_to_map_sign": {"x": 1.0, "y": 1.0, "z": 1.0},
}


def finite_or_none(value):
    try:
        numeric_value = float(value)
    except (TypeError, ValueError):
        return None
    return numeric_value if math.isfinite(numeric_value) else None


def _read_float(data, key_path, default=None):
    node = data
    for key in key_path:
        if not isinstance(node, dict) or key not in node:
            if default is not None:
                return float(default)
            raise ValueError(f"missing {'.'.join(key_path)}")
        node = node[key]

    value = finite_or_none(node)
    if value is None:
        raise ValueError(f"{'.'.join(key_path)} must be a finite number")
    return value


def _axis_dict(data, root_key, defaults):
    source = data.get(root_key, {}) if isinstance(data, dict) else {}
    return {
        "x": _read_float(source, ("x",), defaults["x"]),
        "y": _read_float(source, ("y",), defaults["y"]),
        "z": _read_float(source, ("z",), defaults["z"]),
    }


def _axis_sign_dict(data, root_key, defaults):
    raw_values = _axis_dict(data, root_key, defaults)
    return {
        axis: -1.0 if raw_values[axis] < 0.0 else 1.0
        for axis in ("x", "y", "z")
    }


def _rpy_dict(data, root_key, defaults):
    source = data.get(root_key, {}) if isinstance(data, dict) else {}
    return {
        "roll": _read_float(source, ("roll",), defaults["roll"]),
        "pitch": _read_float(source, ("pitch",), defaults["pitch"]),
        "yaw": _read_float(source, ("yaw",), defaults["yaw"]),
    }


def normalize_robot_home_config(data):
    data = data or {}
    return {
        "home_cabin_mm": _axis_dict(
            data,
            "home_cabin_mm",
            DEFAULT_ROBOT_HOME_CONFIG["home_cabin_mm"],
        ),
        "base_to_camera_mm": _axis_dict(
            data,
            "base_to_camera_mm",
            DEFAULT_ROBOT_HOME_CONFIG["base_to_camera_mm"],
        ),
        "base_to_camera_rpy": _rpy_dict(
            data,
            "base_to_camera_rpy",
            DEFAULT_ROBOT_HOME_CONFIG["base_to_camera_rpy"],
        ),
        "cabin_to_map_sign": _axis_sign_dict(
            data,
            "cabin_to_map_sign",
            DEFAULT_ROBOT_HOME_CONFIG["cabin_to_map_sign"],
        ),
    }


def load_robot_home_config(config_path):
    if not os.path.exists(config_path):
        return copy.deepcopy(DEFAULT_ROBOT_HOME_CONFIG)
    with open(config_path, "r", encoding="utf-8") as handle:
        return normalize_robot_home_config(yaml.safe_load(handle) or {})


def save_robot_home_config(config_path, config):
    serialized = normalize_robot_home_config(config)
    target_dir = os.path.dirname(config_path) or "."
    os.makedirs(target_dir, exist_ok=True)
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            suffix=".yaml",
            encoding="utf-8",
            dir=target_dir,
            delete=False,
        ) as handle:
            yaml.safe_dump(serialized, handle, sort_keys=False, allow_unicode=True)
            temp_path = handle.name
        os.replace(temp_path, config_path)
    finally:
        if temp_path and os.path.exists(temp_path):
            os.unlink(temp_path)


def with_home_cabin_mm(config, x_mm, y_mm, z_mm):
    updated = copy.deepcopy(normalize_robot_home_config(config))
    updated["home_cabin_mm"] = {
        "x": float(x_mm),
        "y": float(y_mm),
        "z": float(z_mm),
    }
    return normalize_robot_home_config(updated)


def extract_pose_mm(msg):
    pose_mm = {
        "x": finite_or_none(msg.cabin_state_X),
        "y": finite_or_none(msg.cabin_state_Y),
        "z": finite_or_none(msg.cabin_state_Z),
    }
    if any(value is None for value in pose_mm.values()):
        return None
    return pose_mm


def _set_translation_m(transform, translation_mm):
    transform.transform.translation.x = translation_mm["x"] / 1000.0
    transform.transform.translation.y = translation_mm["y"] / 1000.0
    transform.transform.translation.z = translation_mm["z"] / 1000.0


def apply_cabin_to_map_sign_mm(values_mm, config):
    sign = normalize_robot_home_config(config)["cabin_to_map_sign"]
    return {
        "x": float(values_mm["x"]) * sign["x"],
        "y": float(values_mm["y"]) * sign["y"],
        "z": float(values_mm["z"]) * sign["z"],
    }


def _set_quaternion_from_rpy(transform, rotation_rpy):
    quat = quaternion_from_euler(
        rotation_rpy["roll"],
        rotation_rpy["pitch"],
        rotation_rpy["yaw"],
    )
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]


def build_transforms(map_frame, base_link_frame, scepter_frame, latest_pose_mm, config, stamp):
    config = normalize_robot_home_config(config)
    base_transform = TransformStamped()
    base_transform.header.stamp = stamp
    base_transform.header.frame_id = map_frame
    base_transform.child_frame_id = base_link_frame
    _set_translation_m(base_transform, apply_cabin_to_map_sign_mm(latest_pose_mm, config))
    base_transform.transform.rotation.w = 1.0

    scepter_transform = TransformStamped()
    scepter_transform.header.stamp = stamp
    scepter_transform.header.frame_id = base_link_frame
    scepter_transform.child_frame_id = scepter_frame
    _set_translation_m(scepter_transform, apply_cabin_to_map_sign_mm(config["base_to_camera_mm"], config))
    _set_quaternion_from_rpy(scepter_transform, config["base_to_camera_rpy"])

    return base_transform, scepter_transform


def transform_camera_point_to_map_mm(latest_pose_mm, config, camera_point_mm):
    config = normalize_robot_home_config(config)
    base_vector = np.array([
        float(latest_pose_mm["x"]) * config["cabin_to_map_sign"]["x"],
        float(latest_pose_mm["y"]) * config["cabin_to_map_sign"]["y"],
        float(latest_pose_mm["z"]) * config["cabin_to_map_sign"]["z"],
    ], dtype=np.float64)
    base_to_camera_vector = np.array([
        config["base_to_camera_mm"]["x"] * config["cabin_to_map_sign"]["x"],
        config["base_to_camera_mm"]["y"] * config["cabin_to_map_sign"]["y"],
        config["base_to_camera_mm"]["z"] * config["cabin_to_map_sign"]["z"],
    ], dtype=np.float64)
    camera_point_vector = np.array([
        float(camera_point_mm["x"]),
        float(camera_point_mm["y"]),
        float(camera_point_mm["z"]),
    ], dtype=np.float64)
    quat = quaternion_from_euler(
        config["base_to_camera_rpy"]["roll"],
        config["base_to_camera_rpy"]["pitch"],
        config["base_to_camera_rpy"]["yaw"],
    )
    rotation_matrix = quaternion_matrix(quat)[:3, :3]
    point_in_base = rotation_matrix.dot(camera_point_vector)
    point_in_map = base_vector + base_to_camera_vector + np.array([
        point_in_base[0] * config["cabin_to_map_sign"]["x"],
        point_in_base[1] * config["cabin_to_map_sign"]["y"],
        point_in_base[2] * config["cabin_to_map_sign"]["z"],
    ], dtype=np.float64)
    return {
        "x": round(float(point_in_map[0]), 6),
        "y": round(float(point_in_map[1]), 6),
        "z": round(float(point_in_map[2]), 6),
    }


def extract_depth_frame_max_mm(msg, max_valid_depth_mm=20000.0):
    if msg is None or int(getattr(msg, "height", 0)) <= 0 or int(getattr(msg, "width", 0)) <= 0:
        return None

    encoding = str(getattr(msg, "encoding", "")).upper()
    if encoding not in {"16UC1", "MONO16"}:
        return None

    width = int(msg.width)
    height = int(msg.height)
    step = int(getattr(msg, "step", width * 2))
    row_stride = step // 2
    if row_stride < width:
        return None

    raw_data = getattr(msg, "data", b"")
    if not isinstance(raw_data, (bytes, bytearray, memoryview)):
        raw_data = bytes(raw_data)
    dtype = ">u2" if bool(getattr(msg, "is_bigendian", False)) else "<u2"
    depth_values = np.frombuffer(raw_data, dtype=np.dtype(dtype))
    required_values = height * row_stride
    if depth_values.size < required_values:
        return None

    depth_image = depth_values[:required_values].reshape((height, row_stride))[:, :width]
    valid_mask = (
        (depth_image > 0)
        & (depth_image != 65535)
        & (depth_image <= float(max_valid_depth_mm))
    )
    if not np.any(valid_mask):
        return None
    return float(np.max(depth_image[valid_mask]))


class DepthGroundProbe:
    def __init__(self, window_size=15, max_valid_depth_mm=20000.0):
        self.window_size = max(1, int(window_size))
        self.max_valid_depth_mm = float(max_valid_depth_mm)
        self.samples_mm = deque(maxlen=self.window_size)
        self.latest_frame_max_mm = None
        self.latest_distance_mm = None

    def update(self, msg):
        frame_max_mm = extract_depth_frame_max_mm(
            msg,
            max_valid_depth_mm=self.max_valid_depth_mm,
        )
        if frame_max_mm is None:
            return self.latest_distance_mm
        self.latest_frame_max_mm = frame_max_mm
        self.samples_mm.append(frame_max_mm)
        sorted_samples = sorted(self.samples_mm)
        mid_index = len(sorted_samples) // 2
        if len(sorted_samples) % 2:
            self.latest_distance_mm = float(sorted_samples[mid_index])
        else:
            self.latest_distance_mm = float(
                (sorted_samples[mid_index - 1] + sorted_samples[mid_index]) / 2.0
            )
        return self.latest_distance_mm


def _set_response_fields(response, values):
    for key, value in values.items():
        setattr(response, key, value)
    return response


def build_robot_home_response(success, message, current_pose_mm, config, depth_probe):
    config = normalize_robot_home_config(config)
    has_current_pose = current_pose_mm is not None
    camera_map = None
    ground_probe = None
    camera_ground_distance_mm = finite_or_none(getattr(depth_probe, "latest_distance_mm", None))
    if has_current_pose:
        camera_map = transform_camera_point_to_map_mm(
            current_pose_mm,
            config,
            {"x": 0.0, "y": 0.0, "z": 0.0},
        )
        if camera_ground_distance_mm is not None:
            ground_probe = transform_camera_point_to_map_mm(
                current_pose_mm,
                config,
                {"x": 0.0, "y": 0.0, "z": camera_ground_distance_mm},
            )

    return _set_response_fields(
        RobotHomeCalibrationResponse(),
        {
            "success": bool(success),
            "message": str(message),
            "has_current_pose": bool(has_current_pose),
            "current_x_mm": float(current_pose_mm["x"]) if has_current_pose else 0.0,
            "current_y_mm": float(current_pose_mm["y"]) if has_current_pose else 0.0,
            "current_z_mm": float(current_pose_mm["z"]) if has_current_pose else 0.0,
            "home_x_mm": float(config["home_cabin_mm"]["x"]),
            "home_y_mm": float(config["home_cabin_mm"]["y"]),
            "home_z_mm": float(config["home_cabin_mm"]["z"]),
            "base_to_camera_x_mm": float(config["base_to_camera_mm"]["x"]),
            "base_to_camera_y_mm": float(config["base_to_camera_mm"]["y"]),
            "base_to_camera_z_mm": float(config["base_to_camera_mm"]["z"]),
            "base_to_camera_roll_rad": float(config["base_to_camera_rpy"]["roll"]),
            "base_to_camera_pitch_rad": float(config["base_to_camera_rpy"]["pitch"]),
            "base_to_camera_yaw_rad": float(config["base_to_camera_rpy"]["yaw"]),
            "has_camera_pose": bool(camera_map is not None),
            "camera_x_mm": float(camera_map["x"]) if camera_map is not None else 0.0,
            "camera_y_mm": float(camera_map["y"]) if camera_map is not None else 0.0,
            "camera_z_mm": float(camera_map["z"]) if camera_map is not None else 0.0,
            "has_ground_probe": bool(ground_probe is not None),
            "camera_ground_distance_mm": float(camera_ground_distance_mm or 0.0),
            "ground_x_mm": float(ground_probe["x"]) if ground_probe is not None else 0.0,
            "ground_y_mm": float(ground_probe["y"]) if ground_probe is not None else 0.0,
            "ground_z_mm": float(ground_probe["z"]) if ground_probe is not None else 0.0,
        },
    )


def resolve_default_config_path():
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "robot_home_tf.yaml")
    )


def main():
    rospy.init_node("robot_tf_broadcaster")

    map_frame = rospy.get_param("~map_frame", "map")
    base_link_frame = rospy.get_param("~base_link_frame", "base_link")
    scepter_frame = rospy.get_param("~scepter_frame", "Scepter_depth_frame")
    cabin_state_topic = rospy.get_param("~cabin_state_topic", "/cabin/cabin_data_upload")
    depth_topic = rospy.get_param("~depth_topic", "/Scepter/depth/image_raw")
    config_path = rospy.get_param("~config_path", resolve_default_config_path())
    publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
    stale_warn_sec = float(rospy.get_param("~stale_warn_sec", 3.0))
    depth_probe_window_size = int(rospy.get_param("~depth_probe_window_size", 15))
    max_valid_depth_mm = float(rospy.get_param("~max_valid_depth_mm", 20000.0))
    if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
        raise ValueError("publish_rate_hz must be positive")
    if not math.isfinite(stale_warn_sec) or stale_warn_sec <= 0.0:
        stale_warn_sec = 3.0

    latest_pose_mm = {"value": None}
    last_cabin_state_stamp = {"value": None}
    runtime_config = {"value": load_robot_home_config(config_path)}
    depth_probe = DepthGroundProbe(
        window_size=depth_probe_window_size,
        max_valid_depth_mm=max_valid_depth_mm,
    )
    pose_lock = threading.Lock()
    config_lock = threading.Lock()

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

    def handle_depth_image(msg):
        depth_probe.update(msg)

    rospy.Subscriber(depth_topic, Image, handle_depth_image, queue_size=2)

    def handle_robot_home_calibration(request):
        command = (getattr(request, "command", "") or "get").strip() or "get"
        try:
            with pose_lock:
                pose_snapshot = (
                    dict(latest_pose_mm["value"])
                    if latest_pose_mm["value"] is not None
                    else None
                )
            with config_lock:
                updated_config = copy.deepcopy(runtime_config["value"])
                if command in {"capture_current", "set_current", "capture_current_home"}:
                    if pose_snapshot is None:
                        return build_robot_home_response(
                            False,
                            "尚未收到索驱当前坐标，无法设置 Home。",
                            pose_snapshot,
                            updated_config,
                            depth_probe,
                        )
                    updated_config = with_home_cabin_mm(
                        updated_config,
                        pose_snapshot["x"],
                        pose_snapshot["y"],
                        pose_snapshot["z"],
                    )
                elif command == "set_home":
                    updated_config = with_home_cabin_mm(
                        updated_config,
                        request.home_x_mm,
                        request.home_y_mm,
                        request.home_z_mm,
                    )
                elif command != "get":
                    return build_robot_home_response(
                        False,
                        f"未知 Home 标定命令: {command}",
                        pose_snapshot,
                        updated_config,
                        depth_probe,
                    )

                if command != "get":
                    save_robot_home_config(config_path, updated_config)
                    runtime_config["value"] = updated_config

            message = "Home 点位与 TF 状态已读取。" if command == "get" else "Home 点位已写入并热更新。"
            return build_robot_home_response(
                True,
                message,
                pose_snapshot,
                updated_config,
                depth_probe,
            )
        except Exception as exc:
            rospy.logerr("robot_tf_broadcaster: 处理%s失败: %s", ROBOT_HOME_CALIBRATION_SERVICE, exc)
            with config_lock:
                config_snapshot = copy.deepcopy(runtime_config["value"])
            with pose_lock:
                pose_snapshot = (
                    dict(latest_pose_mm["value"])
                    if latest_pose_mm["value"] is not None
                    else None
                )
            return build_robot_home_response(
                False,
                f"Home 点位处理失败: {exc}",
                pose_snapshot,
                config_snapshot,
                depth_probe,
            )

    if RobotHomeCalibration is not None:
        rospy.Service(
            ROBOT_HOME_CALIBRATION_SERVICE,
            RobotHomeCalibration,
            handle_robot_home_calibration,
        )
    else:
        rospy.logwarn(
            "RobotHomeCalibration service type is unavailable; rebuild tie_robot_msgs to expose %s.",
            ROBOT_HOME_CALIBRATION_SERVICE,
        )

    broadcaster = tf2_ros.TransformBroadcaster()
    rospy.loginfo(
        "robot_tf_broadcaster started: %s -> %s -> %s from %s | depth_topic=%s | config=%s",
        map_frame,
        base_link_frame,
        scepter_frame,
        cabin_state_topic,
        depth_topic,
        config_path,
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

        with config_lock:
            config_snapshot = copy.deepcopy(runtime_config["value"])

        base_transform, scepter_transform = build_transforms(
            map_frame,
            base_link_frame,
            scepter_frame,
            pose_snapshot,
            config_snapshot,
            now,
        )
        broadcaster.sendTransform([base_transform, scepter_transform])
        rate.sleep()


if __name__ == "__main__":
    main()
