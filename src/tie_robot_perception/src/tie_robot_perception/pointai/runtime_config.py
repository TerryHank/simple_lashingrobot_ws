"""pointAI 拆分后的职责模块。"""
import json
import math
import os
import time
import yaml

import cv2
import numpy as np
import rospy
import torch
import tf2_ros
from cv2 import ximgproc
from cv2.ppf_match_3d import Pose3D
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Vector3
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from sklearn.cluster import DBSCAN
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_matrix

from tie_robot_msgs.msg import PointCoords, PointsArray, motion
from tie_robot_msgs.srv import (
    PlaneDetection,
    PlaneDetectionResponse,
    ProcessImage,
    ProcessImageResponse,
    SingleMove,
    SingleMoveRequest,
    linear_module_move,
    linear_module_moveRequest,
    linear_module_moveResponse,
)
from tie_robot_perception.perception.workspace_s2 import (
    build_workspace_s2_axis_profile,
    build_workspace_s2_bbox,
    build_workspace_s2_line_positions,
    build_workspace_s2_projective_line_segments,
    build_workspace_s2_rectified_geometry,
    estimate_workspace_s2_period_and_phase,
    map_workspace_s2_rectified_points_to_image,
    normalize_workspace_s2_response,
    smooth_workspace_s2_profile,
    sort_polygon_indices_clockwise,
    sort_polygon_points_clockwise,
)
from .constants import *

def load_runtime_config(self):
    if not os.path.exists(self.cali_offset_file):
        return

    try:
        with open(self.cali_offset_file, 'r') as f:
            data = json.load(f)
    except Exception as exc:
        rospy.logwarn("读取%s失败，仅使用默认高度阈值: %s", self.cali_offset_file, exc)
        return

    self.height_threshold = data.get('height_threshold', self.height_threshold)
    if any(key in data for key in ('cal_x', 'cal_y', 'cal_z')):
        rospy.logwarn(
            "pointAI: lashing_config.json中的cal_x/cal_y/cal_z已弃用，"
            "请改%s里的translation_mm。",
            self.gripper_tf_config_file,
        )


def save_runtime_config(self):
    data = {}
    if os.path.exists(self.cali_offset_file):
        try:
            with open(self.cali_offset_file, 'r') as f:
                data = json.load(f)
        except Exception:
            data = {}

    data.pop('cal_x', None)
    data.pop('cal_y', None)
    data.pop('cal_z', None)
    data['height_threshold'] = self.height_threshold

    with open(self.cali_offset_file, 'w') as f:
        json.dump(data, f, indent=4)


def update_gripper_tf_translation_mm(self, x_mm, y_mm, z_mm):
    if not os.path.exists(self.gripper_tf_config_file):
        raise FileNotFoundError(self.gripper_tf_config_file)

    with open(self.gripper_tf_config_file, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f) or {}

    config.setdefault('parent_frame', 'Scepter_depth_frame')
    config.setdefault('child_frame', 'gripper_frame')
    config.setdefault(
        'rotation_rpy',
        {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    )
    config['translation_mm'] = {
        'x': float(x_mm),
        'y': float(y_mm),
        'z': float(z_mm),
    }

    with open(self.gripper_tf_config_file, 'w', encoding='utf-8') as f:
        yaml.safe_dump(config, f, sort_keys=False, allow_unicode=True)


def fixed_z_value_callback(self, msg):
    """
    /fixed_z_value 回调: 若值不为0将在计算点坐标时强制替换z
    """
    if msg.data < 20:
        self.height_threshold = msg.data
        self.save_runtime_config()
        rospy.loginfo(f"设置高度阈值为: {self.height_threshold}")
    else:
        self.fixed_z_value = msg.data
        rospy.loginfo(f"接收到固定下探Z值: {self.fixed_z_value}") 


def calibration_offset_callback(self, msg):
    """
    兼容旧前端的回调函数。
    旧的视觉偏差入口现在由 gripper_tf_broadcaster 实时处理。
    """
    rospy.loginfo(
        "pointAI: /web/pointAI/set_offset 现由gripper_tf_broadcaster实时处理，"
        "translation_mm=(%.3f, %.3f, %.3f)，无需重启节点。",
        msg.position.x,
        msg.position.y,
        msg.position.z,
    )


def load_scan_planning_workspace(self):
    default_bounds = {
        "min_x": 0.0,
        "max_x": 0.0,
        "min_y": 0.0,
        "max_y": 0.0,
    }
    try:
        if not os.path.exists(self.path_points_file):
            return default_bounds

        with open(self.path_points_file, "r", encoding="utf-8") as file_obj:
            path_json = json.load(file_obj)

        path_points = path_json.get("path_points", [])
        if not path_points:
            return default_bounds

        point_x_values = [float(point["x"]) for point in path_points]
        point_y_values = [float(point["y"]) for point in path_points]
        marking_x = float(path_json.get("marking_x", point_x_values[0]))
        marking_y = float(path_json.get("marking_y", point_y_values[0]))
        robot_x_step = float(path_json.get("robot_x_step", 0.0))
        robot_y_step = float(path_json.get("robot_y_step", 0.0))
        zone_x = float(path_json.get("zone_x", 0.0))
        zone_y = float(path_json.get("zone_y", 0.0))

        if robot_x_step > 0.0 and robot_y_step > 0.0 and zone_x > 0.0 and zone_y > 0.0:
            workspace_min_x = marking_x - robot_x_step / 2.0
            workspace_min_y = marking_y - robot_y_step / 2.0
            workspace_max_x = workspace_min_x + zone_x
            workspace_max_y = workspace_min_y + zone_y
            return {
                "min_x": workspace_min_x,
                "max_x": workspace_max_x,
                "min_y": workspace_min_y,
                "max_y": workspace_max_y,
            }

        half_x_step = robot_x_step / 2.0 if robot_x_step > 0.0 else 0.0
        half_y_step = robot_y_step / 2.0 if robot_y_step > 0.0 else 0.0
        return {
            "min_x": min(point_x_values) - half_x_step,
            "max_x": max(point_x_values) + half_x_step,
            "min_y": min(point_y_values) - half_y_step,
            "max_y": max(point_y_values) + half_y_step,
        }
    except Exception as exc:
        rospy.logwarn_throttle(5.0, f"pointAI扫描工作区读取失败: {exc}")
        return default_bounds


def load_workspace_center_scan_pose_target(self):
    return {
        "x": -260.0,
        "y": 1700.0,
        "z": 2997.0,
        "speed": 100.0,
    }


def run_workspace_center_scan_pose_move(self):
    target = self.load_workspace_center_scan_pose_target()
    service_name = "/cabin/single_move"
    try:
        rospy.wait_for_service(service_name, timeout=1.0)
        single_move_service = rospy.ServiceProxy(service_name, SingleMove)
        request = SingleMoveRequest()
        request.command = "单点运动请求"
        request.x = float(target["x"])
        request.y = float(target["y"])
        request.z = float(target["z"])
        request.speed = float(target["speed"])
        response = single_move_service(request)
    except Exception as exc:
        rospy.logwarn("pointAI调用%s失败: %s", service_name, exc)
        return {"success": False, "message": str(exc), "target": target}

    success = bool(getattr(response, "success", False))
    message = getattr(response, "message", "")
    if success:
        rospy.loginfo(
            "pointAI已下发固定扫描位姿: x=%.1f, y=%.1f, z=%.1f, speed=%.1f",
            target["x"],
            target["y"],
            target["z"],
            target["speed"],
        )
    else:
        rospy.logwarn("pointAI固定扫描位姿下发失败: %s", message)

    return {
        "success": success,
        "message": message,
        "target": target,
    }


def workspace_center_scan_pose_callback(self, msg):
    if not bool(getattr(msg, "data", False)):
        return

    result = self.run_workspace_center_scan_pose_move()
    if not result.get("success", False):
        rospy.logwarn(
            "pointAI workspace center scan pose move failed: %s",
            result.get("message", "unknown error"),
        )
