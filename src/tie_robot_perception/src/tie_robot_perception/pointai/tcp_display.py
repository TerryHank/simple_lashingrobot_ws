"""Coordinate conversion from raw camera coordinates to TCP jaw coordinates."""

import math
from pathlib import Path

import numpy as np
import yaml


PERCEPTION_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_TCP_DISPLAY_CONFIG_PATH = PERCEPTION_PACKAGE_ROOT / "config" / "gripper_tf.yaml"
_CACHED_TCP_DISPLAY_CONFIG = None
_CACHED_TCP_DISPLAY_CONFIG_MTIME = None


def _load_tcp_display_config(config_path=DEFAULT_TCP_DISPLAY_CONFIG_PATH):
    global _CACHED_TCP_DISPLAY_CONFIG
    global _CACHED_TCP_DISPLAY_CONFIG_MTIME

    config_path = Path(config_path)
    config_mtime = config_path.stat().st_mtime
    if (
        _CACHED_TCP_DISPLAY_CONFIG is not None
        and _CACHED_TCP_DISPLAY_CONFIG_MTIME == config_mtime
    ):
        return _CACHED_TCP_DISPLAY_CONFIG

    with open(config_path, "r", encoding="utf-8") as file_obj:
        data = yaml.safe_load(file_obj) or {}

    translation = data.get("translation_mm") or {}
    rotation = data.get("rotation_rpy") or {}
    config = {
        "translation_mm": {
            "x": float(translation.get("x", 0.0)),
            "y": float(translation.get("y", 0.0)),
            "z": float(translation.get("z", 0.0)),
        },
        "rotation_rpy": {
            "roll": float(rotation.get("roll", 0.0)),
            "pitch": float(rotation.get("pitch", 0.0)),
            "yaw": float(rotation.get("yaw", 0.0)),
        },
    }
    _CACHED_TCP_DISPLAY_CONFIG = config
    _CACHED_TCP_DISPLAY_CONFIG_MTIME = config_mtime
    return config


def _rotation_matrix_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _apply_transposed_rotation(rotation_matrix, vector):
    return [
        rotation_matrix[0][0] * vector[0] + rotation_matrix[1][0] * vector[1] + rotation_matrix[2][0] * vector[2],
        rotation_matrix[0][1] * vector[0] + rotation_matrix[1][1] * vector[1] + rotation_matrix[2][1] * vector[2],
        rotation_matrix[0][2] * vector[0] + rotation_matrix[1][2] * vector[1] + rotation_matrix[2][2] * vector[2],
    ]


def camera_coord_to_tcp_jaw_coord(camera_coord, config=None, current_tcp_mm=None):
    """Convert Scepter camera millimeter coordinates into gripper/TCP local coordinates."""
    if config is None:
        config = _load_tcp_display_config()

    camera_x, camera_y, camera_z = [float(value) for value in camera_coord[:3]]
    translation = config["translation_mm"]
    rotation = config["rotation_rpy"]
    parent_delta = [
        camera_x - translation["x"],
        camera_y - translation["y"],
        camera_z - translation["z"],
    ]
    rotation_matrix = _rotation_matrix_from_rpy(
        rotation["roll"],
        rotation["pitch"],
        rotation["yaw"],
    )
    gripper_coord = _apply_transposed_rotation(rotation_matrix, parent_delta)
    if not current_tcp_mm:
        return gripper_coord

    return [
        gripper_coord[0] - float(current_tcp_mm.get("x", 0.0)),
        gripper_coord[1] - float(current_tcp_mm.get("y", 0.0)),
        gripper_coord[2] - float(current_tcp_mm.get("z", 0.0)),
    ]


def camera_channels_to_tcp_jaw_channels(x_channel, y_channel, z_channel, config=None, current_tcp_mm=None):
    """Vectorized camera-channel conversion into gripper/TCP coordinates."""
    if config is None:
        config = _load_tcp_display_config()

    translation = config["translation_mm"]
    rotation = config["rotation_rpy"]
    rotation_matrix = _rotation_matrix_from_rpy(
        rotation["roll"],
        rotation["pitch"],
        rotation["yaw"],
    )

    camera_x = np.asarray(x_channel, dtype=np.float32)
    camera_y = np.asarray(y_channel, dtype=np.float32)
    camera_z = np.asarray(z_channel, dtype=np.float32)
    parent_delta_x = camera_x - float(translation["x"])
    parent_delta_y = camera_y - float(translation["y"])
    parent_delta_z = camera_z - float(translation["z"])

    gripper_x = (
        (rotation_matrix[0][0] * parent_delta_x)
        + (rotation_matrix[1][0] * parent_delta_y)
        + (rotation_matrix[2][0] * parent_delta_z)
    )
    gripper_y = (
        (rotation_matrix[0][1] * parent_delta_x)
        + (rotation_matrix[1][1] * parent_delta_y)
        + (rotation_matrix[2][1] * parent_delta_z)
    )
    gripper_z = (
        (rotation_matrix[0][2] * parent_delta_x)
        + (rotation_matrix[1][2] * parent_delta_y)
        + (rotation_matrix[2][2] * parent_delta_z)
    )

    if current_tcp_mm:
        gripper_x = gripper_x - float(current_tcp_mm.get("x", 0.0))
        gripper_y = gripper_y - float(current_tcp_mm.get("y", 0.0))
        gripper_z = gripper_z - float(current_tcp_mm.get("z", 0.0))

    return gripper_x, gripper_y, gripper_z
