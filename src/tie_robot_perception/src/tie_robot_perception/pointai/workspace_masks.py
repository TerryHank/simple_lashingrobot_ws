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

def save_manual_workspace_quad(self, corner_pixels, corner_world_cabin_frame, corner_sample_pixels=None):
    manual_workspace_json = {
        "corner_pixels": [[int(point[0]), int(point[1])] for point in corner_pixels],
        "corner_world_cabin_frame": [
            [float(point[0]), float(point[1]), float(point[2])]
            for point in corner_world_cabin_frame
        ],
    }
    if corner_sample_pixels is not None:
        manual_workspace_json["corner_sample_pixels"] = [
            [int(point[0]), int(point[1])] for point in corner_sample_pixels
        ]

    with open(self.manual_workspace_quad_file, "w", encoding="utf-8") as file_obj:
        json.dump(manual_workspace_json, file_obj, indent=2, ensure_ascii=False)


def build_manual_workspace_quad_pixels_message(self, manual_workspace=None):
    manual_workspace = self.load_manual_workspace_quad() if manual_workspace is None else manual_workspace
    if manual_workspace is None:
        return None

    corner_pixels = manual_workspace.get("corner_pixels")
    if not isinstance(corner_pixels, list) or len(corner_pixels) != 4:
        return None

    payload = []
    for point in corner_pixels:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            return None
        payload.extend([float(point[0]), float(point[1])])

    return Float32MultiArray(data=payload)


def publish_current_manual_workspace_quad_pixels(self):
    publisher = getattr(self, "manual_workspace_quad_pixels_pub", None)
    if publisher is None:
        return

    message = self.build_manual_workspace_quad_pixels_message()
    if message is None:
        return
    publisher.publish(message)


def load_manual_workspace_quad(self):
    manual_workspace_file = getattr(self, "manual_workspace_quad_file", None)
    if not manual_workspace_file or not os.path.exists(manual_workspace_file):
        return None

    try:
        with open(manual_workspace_file, "r", encoding="utf-8") as file_obj:
            manual_workspace_json = json.load(file_obj)
    except Exception:
        return None

    corner_pixels = manual_workspace_json.get("corner_pixels")
    corner_world_cabin_frame = manual_workspace_json.get("corner_world_cabin_frame")
    if (
        not isinstance(corner_pixels, list)
        or not isinstance(corner_world_cabin_frame, list)
        or len(corner_pixels) != 4
        or len(corner_world_cabin_frame) != 4
    ):
        return None

    normalized_corner_pixels = []
    normalized_corner_world = []
    for pixel_point, world_point in zip(corner_pixels, corner_world_cabin_frame):
        if (
            not isinstance(pixel_point, (list, tuple))
            or not isinstance(world_point, (list, tuple))
            or len(pixel_point) != 2
            or len(world_point) != 3
        ):
            return None

        normalized_corner_pixels.append([int(round(pixel_point[0])), int(round(pixel_point[1]))])
        normalized_corner_world.append([
            float(world_point[0]),
            float(world_point[1]),
            float(world_point[2]),
        ])

    return {
        "corner_pixels": normalized_corner_pixels,
        "corner_world_cabin_frame": normalized_corner_world,
    }


def manual_workspace_quad_callback(self, msg):
    raw_data = list(getattr(msg, "data", []))
    if len(raw_data) != 8:
        return

    clicked_corner_pixels = []
    corner_sample_pixels = []
    corner_world_cabin_frame = []
    for corner_index in range(0, len(raw_data), 2):
        pixel_x = int(round(raw_data[corner_index]))
        pixel_y = int(round(raw_data[corner_index + 1]))
        raw_world_coord, sample_pixel, _ = self.get_valid_world_coord_near_pixel(pixel_x, pixel_y)
        if raw_world_coord[0] == 0 or raw_world_coord[1] == 0 or raw_world_coord[2] == 0:
            return

        calibrated_world_coord = self.apply_spatial_calibration(
            raw_world_coord[0],
            raw_world_coord[1],
            raw_world_coord[2],
            corner_index // 2,
            target_frame="cabin_frame",
        )
        if calibrated_world_coord is None:
            return

        clicked_corner_pixels.append([pixel_x, pixel_y])
        corner_sample_pixels.append([int(sample_pixel[0]), int(sample_pixel[1])])
        corner_world_cabin_frame.append([
            float(calibrated_world_coord[0]),
            float(calibrated_world_coord[1]),
            float(calibrated_world_coord[2]),
        ])

    point_records = list(zip(clicked_corner_pixels, corner_sample_pixels, corner_world_cabin_frame))
    ordered_corner_pixels = self.sort_polygon_points_clockwise(clicked_corner_pixels)
    ordered_records = []
    remaining_records = point_records[:]
    for ordered_pixel in ordered_corner_pixels:
        for record_index, record in enumerate(remaining_records):
            if record[0][0] == ordered_pixel[0] and record[0][1] == ordered_pixel[1]:
                ordered_records.append(record)
                remaining_records.pop(record_index)
                break

    if len(ordered_records) != 4:
        ordered_records = point_records

    self.save_manual_workspace_quad(
        [record[0] for record in ordered_records],
        [record[2] for record in ordered_records],
        corner_sample_pixels=[record[1] for record in ordered_records],
    )
    self.publish_current_manual_workspace_quad_pixels()


def get_manual_workspace_pixel_mask(self):
    manual_workspace = self.load_manual_workspace_quad()
    if manual_workspace is None:
        return None

    image_shape = None
    for attr_name in ("x_channel", "y_channel", "depth_v", "image_infrared_copy", "image_infrared"):
        image_data = getattr(self, attr_name, None)
        if image_data is not None:
            image_shape = image_data.shape[:2]
            break

    if image_shape is None:
        return None

    image_height, image_width = image_shape
    polygon_points = np.array(manual_workspace["corner_pixels"], dtype=np.int32)
    polygon_points[:, 0] = np.clip(polygon_points[:, 0], 0, image_width - 1)
    polygon_points[:, 1] = np.clip(polygon_points[:, 1], 0, image_height - 1)

    polygon_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    cv2.fillPoly(polygon_mask, [polygon_points.reshape((-1, 1, 2))], 1)
    if not np.any(polygon_mask):
        return None
    return polygon_mask


def build_convex_polygon_inside_mask(target_x, target_y, polygon_xy, valid_mask):
    polygon_xy = np.asarray(polygon_xy, dtype=np.float32)
    if polygon_xy.shape[0] < 3:
        return None

    inside_mask = np.asarray(valid_mask, dtype=bool).copy()
    if not np.any(inside_mask):
        return None

    polygon_center = np.mean(polygon_xy, axis=0)
    cross_sign = None
    for edge_index in range(polygon_xy.shape[0]):
        point_a = polygon_xy[edge_index]
        point_b = polygon_xy[(edge_index + 1) % polygon_xy.shape[0]]
        edge_cross = (
            (point_b[0] - point_a[0]) * (polygon_center[1] - point_a[1])
            - (point_b[1] - point_a[1]) * (polygon_center[0] - point_a[0])
        )
        if abs(float(edge_cross)) > 1e-3:
            cross_sign = 1.0 if edge_cross > 0.0 else -1.0
            break

    if cross_sign is None:
        return None

    for edge_index in range(polygon_xy.shape[0]):
        point_a = polygon_xy[edge_index]
        point_b = polygon_xy[(edge_index + 1) % polygon_xy.shape[0]]
        cross_value = (
            (point_b[0] - point_a[0]) * (target_y - point_a[1])
            - (point_b[1] - point_a[1]) * (target_x - point_a[0])
        )
        inside_mask &= (cross_sign * cross_value) >= -1e-3

    if not np.any(inside_mask):
        return None
    return inside_mask.astype(np.uint8)


def get_manual_workspace_cabin_polygon_pixel_mask(self):
    manual_workspace = self.load_manual_workspace_quad()
    if manual_workspace is None:
        return None

    cabin_channels = self.get_cabin_frame_xy_channels()
    if cabin_channels is None:
        return None

    polygon_xy = np.array(
        [[float(point[0]), float(point[1])] for point in manual_workspace["corner_world_cabin_frame"]],
        dtype=np.float32,
    )
    if polygon_xy.shape != (4, 2):
        return None

    return self.build_convex_polygon_inside_mask(
        cabin_channels["x"],
        cabin_channels["y"],
        polygon_xy,
        cabin_channels["valid_mask"],
    )


def is_point_in_manual_workspace_polygon(self, world_x, world_y):
    manual_workspace = self.load_manual_workspace_quad()
    if manual_workspace is None:
        return None

    polygon_xy = np.array(
        [[point[0], point[1]] for point in manual_workspace["corner_world_cabin_frame"]],
        dtype=np.float32,
    )
    if polygon_xy.shape != (4, 2):
        return None

    return cv2.pointPolygonTest(
        polygon_xy.reshape((-1, 1, 2)),
        (float(world_x), float(world_y)),
        False,
    ) >= 0.0


def is_point_in_scan_workspace(self, world_x, world_y):
    manual_workspace_result = self.is_point_in_manual_workspace_polygon(world_x, world_y)
    if manual_workspace_result is not None:
        return manual_workspace_result

    workspace = self.load_scan_planning_workspace()
    if workspace["min_x"] == workspace["max_x"] and workspace["min_y"] == workspace["max_y"]:
        return True
    return (
        workspace["min_x"] <= world_x <= workspace["max_x"]
        and workspace["min_y"] <= world_y <= workspace["max_y"]
    )


def get_frame_space_pixel_mask(self, target_frame, min_x, max_x, min_y, max_y):
    if (not hasattr(self, 'x_channel')) or (not hasattr(self, 'y_channel')) or (not hasattr(self, 'depth_v')):
        return None
    if self.x_channel is None or self.y_channel is None or self.depth_v is None:
        return None

    transform_matrix = self.lookup_transform_matrix_mm(target_frame)
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

    mask = (
        valid_mask
        & (target_x >= min_x)
        & (target_x <= max_x)
        & (target_y >= min_y)
        & (target_y <= max_y)
    )
    if not np.any(mask):
        return None
    return mask.astype(np.uint8)


def get_roi_pixel_mask(self):
    image_shape = None
    for attr_name in ("x_channel", "y_channel", "depth_v", "image_infrared_copy", "image_infrared"):
        image_data = getattr(self, attr_name, None)
        if image_data is not None:
            image_shape = image_data.shape[:2]
            break

    if image_shape is None:
        return None

    image_height, image_width = image_shape
    left = max(0, min(self.point1[0], self.point2[0]))
    right = min(image_width - 1, max(self.point1[0], self.point2[0]))
    top = max(0, min(self.point1[1], self.point2[1]))
    bottom = min(image_height - 1, max(self.point1[1], self.point2[1]))
    if left > right or top > bottom:
        return None

    roi_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    roi_mask[top:bottom + 1, left:right + 1] = 1
    return roi_mask


def get_scan_workspace_pixel_mask(self):
    manual_workspace_mask = self.get_manual_workspace_pixel_mask()
    if manual_workspace_mask is not None:
        return manual_workspace_mask

    workspace = self.load_scan_planning_workspace()
    if workspace["min_x"] == workspace["max_x"] and workspace["min_y"] == workspace["max_y"]:
        return None
    return self.get_frame_space_pixel_mask(
        "cabin_frame",
        workspace["min_x"],
        workspace["max_x"],
        workspace["min_y"],
        workspace["max_y"],
    )


def should_apply_top_detection_occlusion(self, request_mode):
    return request_mode != PROCESS_IMAGE_MODE_SCAN_ONLY


def apply_detection_occlusions(self, request_mode):
    detection_mask = np.ones(self.Depth_image_Raw.shape[:2], dtype=np.uint8)
    if self.should_apply_top_detection_occlusion(request_mode):
        self.Depth_image_Raw[self.y1:self.y2, self.x1:self.x2] = 0
        detection_mask[self.y1:self.y2, self.x1:self.x2] = 0
    return detection_mask


def get_travel_range_pixel_mask(self):
    roi_mask = self.get_roi_pixel_mask()
    if roi_mask is None:
        return None

    workspace_mask = self.get_scan_workspace_pixel_mask()
    if workspace_mask is None:
        return roi_mask

    return (roi_mask & workspace_mask.astype(np.uint8)).astype(np.uint8)


def is_point_in_matrix_selection_pixel_mask(self, pixel_x, pixel_y, pixel_mask=None):
    matrix_selection_pixel_mask = self.get_travel_range_pixel_mask() if pixel_mask is None else pixel_mask
    if matrix_selection_pixel_mask is None:
        return self.is_point_in_roi(pixel_x, pixel_y)

    pixel_x = int(round(pixel_x))
    pixel_y = int(round(pixel_y))
    if (
        pixel_x < 0 or pixel_y < 0
        or pixel_y >= matrix_selection_pixel_mask.shape[0]
        or pixel_x >= matrix_selection_pixel_mask.shape[1]
    ):
        return False
    return bool(matrix_selection_pixel_mask[pixel_y, pixel_x])


def is_point_in_roi(self, x, y):
    left = min(self.point1[0], self.point2[0])
    right = max(self.point1[0], self.point2[0])
    top = min(self.point1[1], self.point2[1])
    bottom = max(self.point1[1], self.point2[1])
    return left <= x <= right and top <= y <= bottom


def is_point_in_travel_range(self, calibrated_x, calibrated_y):
    return (
        0 <= calibrated_x <= getattr(self, "travel_range_max_x_mm", 360.0)
        and 0 <= calibrated_y <= getattr(self, "travel_range_max_y_mm", 320.0)
    )


def is_point_in_matrix_selection_range(self, calibrated_x, calibrated_y):
    return (
        0 <= calibrated_x <= getattr(self, "matrix_selection_max_x_mm", 500.0)
        and 0 <= calibrated_y <= getattr(self, "matrix_selection_max_y_mm", 500.0)
    )


def is_point_in_display_bind_range(self, calibrated_x, calibrated_y):
    return (
        0 <= calibrated_x <= getattr(self, "display_bind_range_max_x_mm", 500.0)
        and 0 <= calibrated_y <= getattr(self, "display_bind_range_max_y_mm", 360.0)
    )
