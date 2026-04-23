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

def get_cabin_frame_xy_channels(self):
    if not self.ensure_raw_world_channels():
        return None

    cached_seq = getattr(self, "_cached_cabin_frame_xy_seq", None)
    current_seq = getattr(self, "world_image_seq", 0)
    if cached_seq == current_seq:
        return getattr(self, "_cached_cabin_frame_xy_channels", None)

    transform_matrix = self.lookup_transform_matrix_mm("cabin_frame", source_frame="Scepter_depth_frame")
    if transform_matrix is None:
        return None

    source_x = self.x_channel.astype(np.float32)
    source_y = self.y_channel.astype(np.float32)
    source_z = self.depth_v.astype(np.float32)
    valid_mask = source_z != 0.0

    target_x = (
        transform_matrix[0, 0] * source_x
        + transform_matrix[0, 1] * source_y
        + transform_matrix[0, 2] * source_z
        + transform_matrix[0, 3]
    )
    target_y = (
        transform_matrix[1, 0] * source_x
        + transform_matrix[1, 1] * source_y
        + transform_matrix[1, 2] * source_z
        + transform_matrix[1, 3]
    )

    cached_channels = {
        "x": target_x.astype(np.float32),
        "y": target_y.astype(np.float32),
        "valid_mask": valid_mask.astype(bool),
    }
    self._cached_cabin_frame_xy_seq = current_seq
    self._cached_cabin_frame_xy_channels = cached_channels
    return cached_channels


def image_raw_world_callback(self, msg):
    img = self.bridge.imgmsg_to_cv2(msg)          # 先取图
    img = np.array(img, copy=True)                # 保证可写
    # ① 通道对调  X↔Y
    img[:, :, 0], img[:, :, 1] = img[:, :, 1], img[:, :, 0].copy()
    # ② 新 Y 翻符号 → 正向朝上
    img[:, :, 1] *= -1
    self.image_raw_world = img
    self.mark_visual_input("raw_world_coord")
    self.ensure_raw_world_channels()


def ensure_raw_world_channels(self):
    if getattr(self, "image_raw_world", None) is None:
        return False

    image_raw_world_channels = self.cv2.split(self.image_raw_world)
    self.x_channel = (image_raw_world_channels[0]).astype(np.int32)
    self.y_channel = (image_raw_world_channels[1]).astype(np.int32)
    self.depth_v = (image_raw_world_channels[2]).astype(np.int32)
    return True


def get_valid_world_coord_near_pixel(self, pixel_x, pixel_y, search_radius=6):
    height, width = self.x_channel.shape
    pixel_x = int(np.clip(pixel_x, 0, width - 1))
    pixel_y = int(np.clip(pixel_y, 0, height - 1))

    raw_world_coord = [
        int(self.x_channel[pixel_y, pixel_x]),
        int(self.y_channel[pixel_y, pixel_x]),
        int(self.depth_v[pixel_y, pixel_x]),
    ]
    if raw_world_coord[0] != 0 and raw_world_coord[1] != 0 and raw_world_coord[2] != 0:
        return raw_world_coord, [pixel_x, pixel_y], False

    best_world_coord = None
    best_sample_pixel = None
    best_distance = None

    for radius in range(1, search_radius + 1):
        min_y = max(0, pixel_y - radius)
        max_y = min(height - 1, pixel_y + radius)
        min_x = max(0, pixel_x - radius)
        max_x = min(width - 1, pixel_x + radius)

        for sample_y in range(min_y, max_y + 1):
            for sample_x in range(min_x, max_x + 1):
                sample_world_coord = [
                    int(self.x_channel[sample_y, sample_x]),
                    int(self.y_channel[sample_y, sample_x]),
                    int(self.depth_v[sample_y, sample_x]),
                ]
                if sample_world_coord[0] == 0 or sample_world_coord[1] == 0 or sample_world_coord[2] == 0:
                    continue

                distance = (sample_x - pixel_x) ** 2 + (sample_y - pixel_y) ** 2
                if best_distance is None or distance < best_distance:
                    best_distance = distance
                    best_world_coord = sample_world_coord
                    best_sample_pixel = [sample_x, sample_y]

        if best_world_coord is not None:
            return best_world_coord, best_sample_pixel, True

    return raw_world_coord, [pixel_x, pixel_y], False
