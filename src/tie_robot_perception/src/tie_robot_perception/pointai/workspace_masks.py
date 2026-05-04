"""pointAI 拆分后的职责模块。"""
import json
import os
import time
import yaml

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
from .tcp_display import camera_channels_to_tcp_jaw_channels, camera_coord_to_tcp_jaw_coord

def save_manual_workspace_quad(self, corner_pixels, corner_world_camera_frame, corner_sample_pixels=None):
    manual_workspace_json = {
        "corner_pixels": [[int(point[0]), int(point[1])] for point in corner_pixels],
        "corner_world_camera_frame": [
            [float(point[0]), float(point[1]), float(point[2])]
            for point in corner_world_camera_frame
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
    self.lashing_workspace_quad_pixels_pub.publish(message)


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
    corner_world_camera_frame = manual_workspace_json.get("corner_world_camera_frame")
    if not isinstance(corner_pixels, list) or len(corner_pixels) != 4:
        return None

    normalized_corner_pixels = []
    normalized_corner_world = []
    for pixel_point in corner_pixels:
        if not isinstance(pixel_point, (list, tuple)) or len(pixel_point) != 2:
            return None

        normalized_corner_pixels.append([int(round(pixel_point[0])), int(round(pixel_point[1]))])

    normalized_workspace = {
        "corner_pixels": normalized_corner_pixels,
    }
    if isinstance(corner_world_camera_frame, list) and len(corner_world_camera_frame) == 4:
        for world_point in corner_world_camera_frame:
            if not isinstance(world_point, (list, tuple)) or len(world_point) != 3:
                normalized_corner_world = []
                break
            normalized_corner_world.append([
                float(world_point[0]),
                float(world_point[1]),
                float(world_point[2]),
            ])
        if len(normalized_corner_world) == 4:
            normalized_workspace["corner_world_camera_frame"] = normalized_corner_world

    return normalized_workspace


def manual_workspace_quad_callback(self, msg):
    raw_data = list(getattr(msg, "data", []))
    if len(raw_data) != 8:
        return

    clicked_corner_pixels = []
    corner_sample_pixels = []
    corner_world_camera_frame = []
    for corner_index in range(0, len(raw_data), 2):
        pixel_x = int(round(raw_data[corner_index]))
        pixel_y = int(round(raw_data[corner_index + 1]))
        raw_world_coord, sample_pixel, _ = self.get_valid_world_coord_near_pixel(pixel_x, pixel_y)
        if raw_world_coord[0] == 0 or raw_world_coord[1] == 0 or raw_world_coord[2] == 0:
            return

        clicked_corner_pixels.append([pixel_x, pixel_y])
        corner_sample_pixels.append([int(sample_pixel[0]), int(sample_pixel[1])])
        corner_world_camera_frame.append([
            float(raw_world_coord[0]),
            float(raw_world_coord[1]),
            float(raw_world_coord[2]),
        ])

    point_records = list(zip(clicked_corner_pixels, corner_sample_pixels, corner_world_camera_frame))
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


def get_manual_workspace_camera_polygon_pixel_mask(self):
    return self.get_manual_workspace_pixel_mask()


def get_manual_workspace_cabin_polygon_pixel_mask(self):
    return self.get_manual_workspace_camera_polygon_pixel_mask()


def is_point_in_manual_workspace_polygon(self, world_x, world_y):
    manual_workspace = self.load_manual_workspace_quad()
    if manual_workspace is None:
        return None

    corner_world_camera_frame = manual_workspace.get("corner_world_camera_frame")
    if not isinstance(corner_world_camera_frame, list) or len(corner_world_camera_frame) != 4:
        return None

    polygon_xy = np.array(
        [[point[0], point[1]] for point in corner_world_camera_frame],
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
    del target_frame, min_x, max_x, min_y, max_y
    return None


def get_scan_workspace_pixel_mask(self):
    manual_workspace_mask = self.get_manual_workspace_pixel_mask()
    if manual_workspace_mask is not None:
        return manual_workspace_mask

    workspace = self.load_scan_planning_workspace()
    if workspace["min_x"] == workspace["max_x"] and workspace["min_y"] == workspace["max_y"]:
        return None
    return self.get_frame_space_pixel_mask(
        "map",
        workspace["min_x"],
        workspace["max_x"],
        workspace["min_y"],
        workspace["max_y"],
    )


def get_tcp_occlusion_pixel_mask(self):
    image_shape = None
    for attr_name in ("x_channel", "y_channel", "depth_v", "image_infrared_copy", "image_infrared"):
        image_data = getattr(self, attr_name, None)
        if image_data is not None:
            image_shape = image_data.shape[:2]
            break

    if image_shape is None:
        return None

    image_height, image_width = image_shape
    left, top, right, bottom = getattr(self, "tcp_occlusion_mask_rect", (160, 0, 523, 80))
    left = max(0, min(image_width - 1, int(round(left))))
    right = max(0, min(image_width - 1, int(round(right))))
    top = max(0, min(image_height - 1, int(round(top))))
    bottom = max(0, min(image_height - 1, int(round(bottom))))
    if left > right or top > bottom:
        return None

    tcp_occlusion_mask = np.zeros((image_height, image_width), dtype=np.uint8)
    tcp_occlusion_mask[top:bottom + 1, left:right + 1] = 1
    return tcp_occlusion_mask


def should_apply_tcp_occlusion_mask(self, request_mode):
    return request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE


def apply_detection_occlusions(self, request_mode):
    detection_mask = np.ones(self.Depth_image_Raw.shape[:2], dtype=np.uint8)
    if not self.should_apply_tcp_occlusion_mask(request_mode):
        return detection_mask

    tcp_occlusion_mask = self.get_tcp_occlusion_pixel_mask()
    if tcp_occlusion_mask is None:
        return detection_mask

    self.Depth_image_Raw[tcp_occlusion_mask > 0] = 0
    detection_mask[tcp_occlusion_mask > 0] = 0
    return detection_mask


def get_execution_refine_tcp_roi_bounds(self):
    return {
        "min_x": float(getattr(self, "execution_refine_tcp_roi_min_x_mm", 0.0)),
        "max_x": float(getattr(self, "execution_refine_tcp_roi_max_x_mm", 380.0)),
        "min_y": float(getattr(self, "execution_refine_tcp_roi_min_y_mm", 0.0)),
        "max_y": float(getattr(self, "execution_refine_tcp_roi_max_y_mm", 3330.0)),
        "min_z": float(getattr(self, "execution_refine_tcp_roi_min_z_mm", 0.0)),
        "max_z": float(getattr(self, "execution_refine_tcp_roi_max_z_mm", 3160.0)),
    }


def _is_valid_camera_world_coord(camera_world_coord):
    if not isinstance(camera_world_coord, (list, tuple)) or len(camera_world_coord) < 3:
        return False

    try:
        coord_values = [float(value) for value in camera_world_coord[:3]]
    except (TypeError, ValueError):
        return False

    return (
        all(np.isfinite(coord_values))
        and coord_values[0] != 0.0
        and coord_values[1] != 0.0
        and coord_values[2] != 0.0
    )


def is_camera_world_coord_in_execution_refine_tcp_range(self, camera_world_coord):
    if not _is_valid_camera_world_coord(camera_world_coord):
        return False

    tcp_x, tcp_y, tcp_z = camera_coord_to_tcp_jaw_coord(camera_world_coord)
    bounds = self.get_execution_refine_tcp_roi_bounds()
    return (
        bounds["min_x"] <= tcp_x <= bounds["max_x"]
        and bounds["min_y"] <= tcp_y <= bounds["max_y"]
        and bounds["min_z"] <= tcp_z <= bounds["max_z"]
    )


def get_execution_refine_tcp_range_pixel_mask(self):
    x_channel = getattr(self, "x_channel", None)
    y_channel = getattr(self, "y_channel", None)
    z_channel = getattr(self, "depth_v", None)
    if x_channel is None or y_channel is None or z_channel is None:
        return None

    tcp_x, tcp_y, tcp_z = camera_channels_to_tcp_jaw_channels(
        x_channel,
        y_channel,
        z_channel,
    )
    valid_camera_coord_mask = (
        np.isfinite(x_channel)
        & np.isfinite(y_channel)
        & np.isfinite(z_channel)
        & (x_channel != 0.0)
        & (y_channel != 0.0)
        & (z_channel != 0.0)
    )
    bounds = self.get_execution_refine_tcp_roi_bounds()
    tcp_range_mask = (
        valid_camera_coord_mask
        & (tcp_x >= bounds["min_x"])
        & (tcp_x <= bounds["max_x"])
        & (tcp_y >= bounds["min_y"])
        & (tcp_y <= bounds["max_y"])
        & (tcp_z >= bounds["min_z"])
        & (tcp_z <= bounds["max_z"])
    )
    return tcp_range_mask.astype(np.uint8)


def get_travel_range_pixel_mask(self):
    return self.get_scan_workspace_pixel_mask()


def is_point_in_matrix_selection_pixel_mask(self, pixel_x, pixel_y, pixel_mask=None):
    matrix_selection_pixel_mask = self.get_travel_range_pixel_mask() if pixel_mask is None else pixel_mask
    if matrix_selection_pixel_mask is None:
        return True

    pixel_x = int(round(pixel_x))
    pixel_y = int(round(pixel_y))
    if (
        pixel_x < 0 or pixel_y < 0
        or pixel_y >= matrix_selection_pixel_mask.shape[0]
        or pixel_x >= matrix_selection_pixel_mask.shape[1]
    ):
        return False
    return bool(matrix_selection_pixel_mask[pixel_y, pixel_x])


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
