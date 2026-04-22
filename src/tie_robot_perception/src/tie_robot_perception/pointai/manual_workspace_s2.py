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

def prepare_manual_workspace_s2_inputs(self):
    manual_workspace = self.load_manual_workspace_quad()
    workspace_mask = self.get_manual_workspace_pixel_mask()
    if manual_workspace is None or workspace_mask is None:
        return None

    if not self.ensure_raw_world_channels():
        return None

    rectified_geometry = self.build_workspace_s2_rectified_geometry(
        manual_workspace["corner_pixels"],
        manual_workspace.get("corner_world_cabin_frame"),
    )
    if rectified_geometry is None:
        return None

    raw_depth = self.depth_v.astype(np.float32)
    valid_mask = np.isfinite(raw_depth) & (raw_depth != 0.0)
    if np.count_nonzero(valid_mask & workspace_mask.astype(bool)) < 100:
        return None

    rectified_size = (
        rectified_geometry["rectified_width"],
        rectified_geometry["rectified_height"],
    )
    rectified_depth = cv2.warpPerspective(
        raw_depth,
        rectified_geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_LINEAR,
    ).astype(np.float32)
    rectified_valid_mask = cv2.warpPerspective(
        valid_mask.astype(np.uint8),
        rectified_geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_NEAREST,
    ).astype(bool)
    if np.count_nonzero(rectified_valid_mask) < 100:
        return None

    median_depth = float(np.median(rectified_depth[rectified_valid_mask]))
    filled_depth = np.where(rectified_valid_mask, rectified_depth, median_depth).astype(np.float32)
    background_depth = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=11.0, sigmaY=11.0)

    response_variants = [
        background_depth - filled_depth,
        filled_depth - background_depth,
    ]

    best_variant = None
    for response_variant in response_variants:
        response_variant = response_variant.astype(np.float32)
        normalized_response = self.normalize_workspace_s2_response(response_variant, rectified_valid_mask)
        rectified_mask_uint8 = rectified_valid_mask.astype(np.uint8)
        vertical_profile = self.build_workspace_s2_axis_profile(normalized_response, rectified_mask_uint8, axis=0)
        horizontal_profile = self.build_workspace_s2_axis_profile(normalized_response, rectified_mask_uint8, axis=1)
        vertical_estimate = self.estimate_workspace_s2_period_and_phase(vertical_profile, min_period=10, max_period=30)
        horizontal_estimate = self.estimate_workspace_s2_period_and_phase(horizontal_profile, min_period=10, max_period=30)
        if vertical_estimate is None or horizontal_estimate is None:
            continue

        combined_score = float(vertical_estimate["score"] + horizontal_estimate["score"])
        if best_variant is None or combined_score > best_variant["combined_score"]:
            best_variant = {
                "response": normalized_response,
                "vertical_estimate": vertical_estimate,
                "horizontal_estimate": horizontal_estimate,
                "combined_score": combined_score,
            }

    if best_variant is None:
        return None

    return {
        "manual_workspace": manual_workspace,
        "workspace_mask": workspace_mask.astype(np.uint8),
        "workspace_bbox": self.build_workspace_s2_bbox(workspace_mask),
        "rectified_geometry": rectified_geometry,
        "response_crop": best_variant["response"],
        "workspace_mask_crop": rectified_valid_mask.astype(np.uint8),
        "vertical_estimate": best_variant["vertical_estimate"],
        "horizontal_estimate": best_variant["horizontal_estimate"],
    }


def build_manual_workspace_s2_points_array(self, intersection_pixels, workspace_mask):
    points_array_msg = PointsArray()
    points_array_msg.PointCoordinatesArray = []
    display_points = []

    if workspace_mask is None:
        points_array_msg.count = 0
        return points_array_msg, display_points

    point_index = 1
    for intersection_pixel in intersection_pixels:
        pixel_x = int(intersection_pixel[0])
        pixel_y = int(intersection_pixel[1])
        if (
            pixel_y < 0 or pixel_x < 0
            or pixel_y >= workspace_mask.shape[0]
            or pixel_x >= workspace_mask.shape[1]
            or not workspace_mask[pixel_y, pixel_x]
        ):
            continue

        raw_world_coord, _, _ = self.get_valid_world_coord_near_pixel(pixel_x, pixel_y)
        if raw_world_coord[0] == 0 or raw_world_coord[1] == 0 or raw_world_coord[2] == 0:
            continue

        calibrated_world_coord = self.apply_spatial_calibration(
            raw_world_coord[0],
            raw_world_coord[1],
            raw_world_coord[2],
            point_index - 1,
            target_frame="cabin_frame",
        )
        if calibrated_world_coord is None:
            continue

        point_msg = PointCoords()
        point_msg.is_shuiguan = False
        point_msg.Angle = -45
        point_msg.idx = point_index
        point_msg.Pix_coord = [pixel_x, pixel_y]
        point_msg.World_coord = [
            float(calibrated_world_coord[0]),
            float(calibrated_world_coord[1]),
            float(calibrated_world_coord[2]),
        ]
        points_array_msg.PointCoordinatesArray.append(point_msg)
        display_points.append(
            (
                point_index,
                [pixel_x, pixel_y],
                [
                    float(calibrated_world_coord[0]),
                    float(calibrated_world_coord[1]),
                    float(calibrated_world_coord[2]),
                ],
                "selected",
                "S2",
            )
        )
        point_index += 1

    points_array_msg.count = len(points_array_msg.PointCoordinatesArray)
    return points_array_msg, display_points


def run_manual_workspace_s2_pipeline(self, publish=False):
    s2_inputs = self.prepare_manual_workspace_s2_inputs()
    if s2_inputs is None:
        return {
            "success": False,
            "message": "当前工作区或原始世界坐标不足，无法运行S2",
            "point_coords": None,
            "result_image": None,
        }

    rectified_width = s2_inputs["rectified_geometry"]["rectified_width"]
    rectified_height = s2_inputs["rectified_geometry"]["rectified_height"]
    vertical_lines = self.build_workspace_s2_line_positions(
        start_pixel=0,
        end_pixel=rectified_width - 1,
        period_px=s2_inputs["vertical_estimate"]["period"],
        phase_px=s2_inputs["vertical_estimate"]["phase"],
    )
    horizontal_lines = self.build_workspace_s2_line_positions(
        start_pixel=0,
        end_pixel=rectified_height - 1,
        period_px=s2_inputs["horizontal_estimate"]["period"],
        phase_px=s2_inputs["horizontal_estimate"]["phase"],
    )

    rectified_intersections = [
        [float(vertical_x), float(horizontal_y)]
        for horizontal_y in horizontal_lines
        for vertical_x in vertical_lines
    ]
    image_intersections = self.map_workspace_s2_rectified_points_to_image(
        rectified_intersections,
        s2_inputs["rectified_geometry"]["inverse_h"],
    )
    line_segments = self.build_workspace_s2_projective_line_segments(
        s2_inputs["manual_workspace"]["corner_pixels"],
        rectified_width,
        rectified_height,
        vertical_lines,
        horizontal_lines,
    )

    points_array_msg, display_points = self.build_manual_workspace_s2_points_array(
        image_intersections,
        s2_inputs["workspace_mask"],
    )
    result_image = self.render_manual_workspace_s2_result_image(
        s2_inputs["workspace_mask"],
        line_segments,
        display_points,
    )
    if publish:
        self.publish_manual_workspace_s2_result(result_image, points_array_msg)

    workspace_bbox = s2_inputs["workspace_bbox"] or (0, 0, 0, 0)
    min_x, min_y, max_x, max_y = workspace_bbox
    rospy.loginfo(
        "pointAI manual workspace S2: bbox=(%d,%d,%d,%d), rectified=(%d,%d), v_period=%d, h_period=%d, points=%d",
        min_x,
        min_y,
        max_x,
        max_y,
        rectified_width,
        rectified_height,
        s2_inputs["vertical_estimate"]["period"],
        s2_inputs["horizontal_estimate"]["period"],
        points_array_msg.count,
    )
    return {
        "success": True,
        "message": "manual workspace S2 finished",
        "point_count": points_array_msg.count,
        "point_coords": points_array_msg,
        "result_image": result_image,
    }


def run_manual_workspace_s2(self):
    return self.run_manual_workspace_s2_pipeline(publish=True)


def try_scan_only_manual_workspace_s2(self):
    if self.load_manual_workspace_quad() is None:
        return None

    s2_result = self.run_manual_workspace_s2_pipeline(publish=True)
    if not s2_result.get("success", False):
        return {
            "success": False,
            "message": (
                "扫描模式检测到已保存工作区四边形，但手动工作区S2失败："
                f"{s2_result.get('message', 'unknown error')}"
            ),
            "point_coords": None,
            "out_of_height_count": 0,
            "out_of_height_point_indices": [],
            "out_of_height_z_values": [],
        }

    point_coords = s2_result.get("point_coords")
    return {
        "success": True,
        "message": (
            f"扫描模式已直接输出手动工作区S2世界坐标点，共{getattr(point_coords, 'count', 0)}个"
        ),
        "point_coords": point_coords,
        "out_of_height_count": 0,
        "out_of_height_point_indices": [],
        "out_of_height_z_values": [],
    }


def manual_workspace_s2_callback(self, msg):
    if not bool(getattr(msg, "data", False)):
        return

    result = self.run_manual_workspace_s2()
    if not result.get("success", False):
        rospy.logwarn("pointAI manual workspace S2 failed: %s", result.get("message", "unknown error"))


def smooth_workspace_s2_profile(profile):
    return smooth_workspace_s2_profile(profile)


def estimate_workspace_s2_period_and_phase(cls, profile, min_period=10, max_period=30):
    return estimate_workspace_s2_period_and_phase(
        profile,
        min_period=min_period,
        max_period=max_period,
    )


def build_workspace_s2_line_positions(start_pixel, end_pixel, period_px, phase_px):
    return build_workspace_s2_line_positions(
        start_pixel,
        end_pixel,
        period_px,
        phase_px,
    )


def build_workspace_s2_bbox(workspace_mask):
    return build_workspace_s2_bbox(workspace_mask)


def build_workspace_s2_axis_profile(response_map, workspace_mask, axis):
    return build_workspace_s2_axis_profile(response_map, workspace_mask, axis)


def normalize_workspace_s2_response(response_map, valid_mask, lower_percentile=5.0, upper_percentile=95.0):
    return normalize_workspace_s2_response(
        response_map,
        valid_mask,
        lower_percentile=lower_percentile,
        upper_percentile=upper_percentile,
    )


def build_workspace_s2_rectified_geometry(
    self,
    corner_pixels,
    corner_world_cabin_frame=None,
    resolution_mm_per_px=5.0,
):
    return build_workspace_s2_rectified_geometry(
        corner_pixels,
        corner_world_cabin_frame=corner_world_cabin_frame,
        resolution_mm_per_px=resolution_mm_per_px,
    )


def map_workspace_s2_rectified_points_to_image(rectified_points, inverse_h):
    return map_workspace_s2_rectified_points_to_image(rectified_points, inverse_h)


def build_workspace_s2_projective_line_segments(
    cls,
    corner_pixels,
    rectified_width,
    rectified_height,
    vertical_lines,
    horizontal_lines,
):
    return build_workspace_s2_projective_line_segments(
        corner_pixels,
        rectified_width,
        rectified_height,
        vertical_lines,
        horizontal_lines,
    )


def sort_polygon_points_clockwise(points):
    return sort_polygon_points_clockwise(points)


def sort_polygon_indices_clockwise(points):
    return sort_polygon_indices_clockwise(points)
