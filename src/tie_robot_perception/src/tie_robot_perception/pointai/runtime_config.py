"""pointAI 拆分后的职责模块。"""
import json
import math
import os
import time

import cv2
import numpy as np
import rospy
import torch
from cv2 import ximgproc
from cv2.ppf_match_3d import Pose3D
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Vector3
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from sklearn.cluster import DBSCAN
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32
from std_srvs.srv import Trigger, TriggerResponse

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
    try:
        self.height_threshold = float(rospy.get_param("~height_threshold", self.height_threshold))
        self.scan_linear_compensation_enabled = bool(
            rospy.get_param("~scan_linear_compensation_enabled", self.scan_linear_compensation_enabled)
        )
        self.scan_linear_compensation_reference_z_mm = float(
            rospy.get_param(
                "~scan_linear_compensation_reference_z_mm",
                self.scan_linear_compensation_reference_z_mm,
            )
        )
        self.scan_linear_compensation_x_per_mm = float(
            rospy.get_param("~scan_linear_compensation_x_per_mm", self.scan_linear_compensation_x_per_mm)
        )
        self.scan_linear_compensation_y_per_mm = float(
            rospy.get_param("~scan_linear_compensation_y_per_mm", self.scan_linear_compensation_y_per_mm)
        )
        self.scan_linear_compensation_min_z_mm = float(
            rospy.get_param("~scan_linear_compensation_min_z_mm", self.scan_linear_compensation_min_z_mm)
        )
        self.scan_linear_compensation_max_abs_scale_delta = float(
            rospy.get_param(
                "~scan_linear_compensation_max_abs_scale_delta",
                self.scan_linear_compensation_max_abs_scale_delta,
            )
        )
    except Exception as exc:
        rospy.logwarn("pointAI: 读取运行参数失败，仅使用默认视觉参数: %s", exc)


def save_runtime_config(self):
    rospy.set_param("~height_threshold", float(self.height_threshold))
    rospy.set_param("~scan_linear_compensation_enabled", bool(self.scan_linear_compensation_enabled))
    rospy.set_param(
        "~scan_linear_compensation_reference_z_mm",
        float(self.scan_linear_compensation_reference_z_mm),
    )
    rospy.set_param("~scan_linear_compensation_x_per_mm", float(self.scan_linear_compensation_x_per_mm))
    rospy.set_param("~scan_linear_compensation_y_per_mm", float(self.scan_linear_compensation_y_per_mm))
    rospy.set_param("~scan_linear_compensation_min_z_mm", float(self.scan_linear_compensation_min_z_mm))
    rospy.set_param(
        "~scan_linear_compensation_max_abs_scale_delta",
        float(self.scan_linear_compensation_max_abs_scale_delta),
    )


def fixed_z_value_callback(self, msg):
    """
    /fixed_z_value 回调: 若值不为0将在计算点坐标时强制替换z
    """
    if msg.data < 20:
        self.height_threshold = msg.data
        self.save_runtime_config()
        rospy.loginfo("pointAI: 设置高度阈值为: %s", self.height_threshold)
    else:
        self.fixed_z_value = msg.data
        rospy.loginfo("pointAI: 接收到固定下探Z值: %s", self.fixed_z_value)


def set_stable_frame_count_callback(self, msg):
    self.stable_frame_count = max(1, int(getattr(msg, "data", 1)))
    rospy.set_param("~stable_frame_count", int(self.stable_frame_count))
    rospy.loginfo("pointAI: 视觉服务最终放行帧数已设置为: %d", self.stable_frame_count)


def set_scan_beam_exclusion_callback(self, msg):
    self.scan_beam_exclusion_enabled = bool(getattr(msg, "data", False))
    rospy.set_param("~scan_beam_exclusion_enabled", bool(self.scan_beam_exclusion_enabled))
    rospy.loginfo(
        "pointAI: 扫描梁筋±13cm过滤已%s。",
        "启用" if self.scan_beam_exclusion_enabled else "关闭",
    )


def set_scan_response_source_callback(self, msg):
    requested_source = str(getattr(msg, "data", "") or "").strip()
    allowed_sources = {
        "fused_instance_response",
        "frangi_like",
        "hessian_ridge",
        "depth_gradient",
        "infrared_response",
        "combined_response",
        "depth_response",
    }
    if requested_source not in allowed_sources:
        rospy.logwarn("pointAI: 扫描底图%s无效，回退为depth_gradient。", requested_source)
        requested_source = "depth_gradient"
    self.scan_response_source = requested_source
    rospy.set_param("~scan_response_source", str(self.scan_response_source))
    rospy.loginfo("pointAI: 扫描底图已设置为: %s", self.scan_response_source)


def _normalize_execution_refine_tcp_roi_axis(min_value, max_value):
    min_mm = float(min_value)
    max_mm = float(max_value)
    if not math.isfinite(min_mm) or not math.isfinite(max_mm):
        raise ValueError("execution_refine_tcp_roi values must be finite")
    if min_mm <= max_mm:
        return min_mm, max_mm
    return max_mm, min_mm


def set_execution_refine_tcp_roi_bounds(self, bounds):
    x_min, x_max = _normalize_execution_refine_tcp_roi_axis(bounds["min_x"], bounds["max_x"])
    y_min, y_max = _normalize_execution_refine_tcp_roi_axis(bounds["min_y"], bounds["max_y"])
    z_min, z_max = _normalize_execution_refine_tcp_roi_axis(bounds["min_z"], bounds["max_z"])
    self.execution_refine_tcp_roi_min_x_mm = x_min
    self.execution_refine_tcp_roi_max_x_mm = x_max
    self.execution_refine_tcp_roi_min_y_mm = y_min
    self.execution_refine_tcp_roi_max_y_mm = y_max
    self.execution_refine_tcp_roi_min_z_mm = z_min
    self.execution_refine_tcp_roi_max_z_mm = z_max
    rospy.set_param("~execution_refine_tcp_roi_min_x_mm", float(x_min))
    rospy.set_param("~execution_refine_tcp_roi_max_x_mm", float(x_max))
    rospy.set_param("~execution_refine_tcp_roi_min_y_mm", float(y_min))
    rospy.set_param("~execution_refine_tcp_roi_max_y_mm", float(y_max))
    rospy.set_param("~execution_refine_tcp_roi_min_z_mm", float(z_min))
    rospy.set_param("~execution_refine_tcp_roi_max_z_mm", float(z_max))
    return {
        "min_x": x_min,
        "max_x": x_max,
        "min_y": y_min,
        "max_y": y_max,
        "min_z": z_min,
        "max_z": z_max,
    }


def set_execution_refine_tcp_roi_callback(self, msg):
    raw_data = list(getattr(msg, "data", []))
    if len(raw_data) != 6:
        rospy.logwarn("pointAI: 线性模组绑扎范围需要6个值[min_x,max_x,min_y,max_y,min_z,max_z]，实际收到%d个", len(raw_data))
        return
    try:
        bounds = self.set_execution_refine_tcp_roi_bounds({
            "min_x": raw_data[0],
            "max_x": raw_data[1],
            "min_y": raw_data[2],
            "max_y": raw_data[3],
            "min_z": raw_data[4],
            "max_z": raw_data[5],
        })
    except (TypeError, ValueError) as exc:
        rospy.logwarn("pointAI: 线性模组绑扎范围参数无效: %s", exc)
        return
    rospy.loginfo(
        "pointAI: 线性模组绑扎范围已更新 x[%.1f, %.1f] y[%.1f, %.1f] z[%.1f, %.1f]mm",
        bounds["min_x"],
        bounds["max_x"],
        bounds["min_y"],
        bounds["max_y"],
        bounds["min_z"],
        bounds["max_z"],
    )


def linear_module_state_callback(self, msg):
    self.current_linear_module_position_mm = {
        "x": float(getattr(msg, "linear_module_position_X", 0.0)),
        "y": float(getattr(msg, "linear_module_position_Y", 0.0)),
        "z": float(getattr(msg, "linear_module_position_Z", 0.0)),
    }


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
