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

def lookup_transform_matrix_mm(self, target_frame, source_frame="Scepter_depth_frame"):
    try:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rospy.Time(0),
        )

        T = quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        T[:3, 3] = [transform.transform.translation.x * 1000,
                    transform.transform.translation.y * 1000,
                    transform.transform.translation.z * 1000]
        return T
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn_throttle(5, f"坐标转换失败: {str(e)}")
    except Exception as e:
        rospy.logerr_throttle(5, f"坐标转换未知错误: {str(e)}")
    return None


def transform_point_to_frame(self, x, y, z, target_frame, source_frame="Scepter_depth_frame"):
    try:
        T = self.lookup_transform_matrix_mm(target_frame, source_frame=source_frame)
        if T is None:
            return None

        point_source = np.array([x, y, z, 1])
        point_target = np.dot(T, point_source)

        return (
            int(round(point_target[0])),
            int(round(point_target[1])),
            int(round(point_target[2])),
        )
    except Exception as e:
        rospy.logerr_throttle(5, f"坐标转换未知错误: {str(e)}")


def transform_to_gripper_frame(self, x, y, z, idx):
    del idx
    return self.transform_point_to_frame(x, y, z, "gripper_frame")


def transform_to_cabin_frame(self, x, y, z, idx):
    del idx
    try:
        T = self.lookup_transform_matrix_mm("cabin_frame", source_frame="Scepter_depth_frame")
        if T is None:
            return None
        T_gripper = self.lookup_transform_matrix_mm("gripper_frame", source_frame="Scepter_depth_frame")
        if T_gripper is None:
            return None

        cabin_x = int(round(float(T[0, 3]) + float(x)))
        cabin_y = int(round(float(T[1, 3]) + float(y)))
        # raw_world_coord 在当前链路里 z 仍表示相机朝下方向的距离。
        # cabin_frame 定义为索驱全局坐标系，z 轴大地朝上；
        # 另外当前索驱高度记录的是工具/索驱位姿，而不是相机光心，所以还需要补回
        # Scepter_depth_frame -> gripper_frame 的竖向外参。项目当前约定 z 不再取反：
        # 标定文件里写多少，TF z 就发布多少。
        cabin_z = int(round(float(T[2, 3]) - float(z) + float(T_gripper[2, 3])))
        return (cabin_x, cabin_y, cabin_z)
    except Exception as e:
        rospy.logerr_throttle(5, f"cabin_frame坐标转换未知错误: {str(e)}")
        return None


def transform_cabin_point_to_gripper_frame(self, x, y, z):
    transformed_point = self.transform_point_to_frame(
        x,
        y,
        z,
        "gripper_frame",
        source_frame="cabin_frame",
    )
    if transformed_point is None:
        return None
    return [float(value) for value in transformed_point]


def is_point_in_tcp_display_range(self, tcp_x, tcp_y, tcp_z):
    return (
        0 <= tcp_x <= getattr(self, "travel_range_max_x_mm", 360.0)
        and 0 <= tcp_y <= getattr(self, "travel_range_max_y_mm", 320.0)
        and 0 <= tcp_z <= getattr(self, "travel_range_max_z_mm", 140.0)
    )


def apply_spatial_calibration(self, x_value, y_value, z_value, idx, target_frame="gripper_frame"):
    if target_frame == "cabin_frame":
        transformed_point = self.transform_to_cabin_frame(x_value, y_value, z_value, idx)
    else:
        transformed_point = self.transform_to_gripper_frame(x_value, y_value, z_value, idx)
    if transformed_point is None:
        return None

    calibrated_x, calibrated_y, calibrated_z = [float(value) for value in transformed_point]
    if self.fixed_z_value != 0:
        calibrated_z = float(self.fixed_z_value)
    return [calibrated_x, calibrated_y, calibrated_z]


def transform_to_end_effector(self, x_obj,y_obj,z_obj,theta_obj):
    """
    ROS服务回调：将物体在相机坐标系下的位置和旋转角度转换到末端执行器坐标系
    :param req: TransformToEndEffectorRequest，包含x_obj, y_obj, z_obj, theta_obj
    :return: TransformToEndEffectorResponse，包含position_ee_obj, rotation_ee_obj
    """
    pose_matrix_path = self.cali_matrix_file
    if not os.path.exists(pose_matrix_path):
        position_ee_obj=[]
        rotation_ee_obj=[]
        return position_ee_obj, rotation_ee_obj
    T_camera_ee = np.load(pose_matrix_path)
    # 将平移部分从米转换为毫米
    # print("T_camera_ee",T_camera_ee)
    T_camera_ee[:3, 3] = T_camera_ee[:3, 3] * 1000
    R_camera_obj = self.create_rotation_matrix(theta_obj)
    T_camera_obj = np.eye(4)
    T_camera_obj[:3, :3] = R_camera_obj
    T_camera_obj[:3, 3] = [x_obj, y_obj, z_obj]
    T_ee_obj = np.dot(T_camera_ee, T_camera_obj)
    position_ee_obj = np.round(T_ee_obj[:3, 3]).astype(int).tolist()

    rotation_ee_obj = T_ee_obj[:3, :3].flatten().tolist()

    return position_ee_obj,rotation_ee_obj
