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

def get_text_bbox(self, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.23, thickness=1):
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = position
    top_left = (x, y - text_height - baseline)
    bottom_right = (x + text_width, y + baseline)
    return top_left, bottom_right, text_width, text_height, baseline


def bboxes_overlap(bbox1, bbox2, padding=4):
    left1, top1, right1, bottom1 = bbox1
    left2, top2, right2, bottom2 = bbox2
    return not (
        right1 + padding < left2 or
        right2 + padding < left1 or
        bottom1 + padding < top2 or
        bottom2 + padding < top1
    )


def find_non_overlapping_label_position(self, image_shape, anchor_point, text, occupied_bboxes, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.23, thickness=1):
    image_height, image_width = image_shape[:2]
    (_, _), (_, _), text_width, text_height, baseline = self.get_text_bbox(
        text, (0, 0), font=font, font_scale=font_scale, thickness=thickness
    )
    min_baseline_y = text_height + baseline + 1
    max_baseline_y = max(min_baseline_y, image_height - baseline - 1)
    max_text_x = max(0, image_width - text_width - 1)
    anchor_x, anchor_y = int(anchor_point[0]), int(anchor_point[1])

    offset_levels = [10, 22, 34, 46, 58, 70]
    for offset in offset_levels:
        candidate_positions = [
            (anchor_x + offset, anchor_y - offset),
            (anchor_x - text_width - offset, anchor_y - offset),
            (anchor_x + offset, anchor_y + text_height + offset),
            (anchor_x - text_width - offset, anchor_y + text_height + offset),
            (anchor_x - (text_width // 2), anchor_y - offset),
            (anchor_x - (text_width // 2), anchor_y + text_height + offset),
        ]

        for candidate_x, candidate_y in candidate_positions:
            clamped_x = int(np.clip(candidate_x, 0, max_text_x))
            clamped_y = int(np.clip(candidate_y, min_baseline_y, max_baseline_y))
            top_left, bottom_right, _, _, _ = self.get_text_bbox(
                text, (clamped_x, clamped_y), font=font, font_scale=font_scale, thickness=thickness
            )
            candidate_bbox = (top_left[0], top_left[1], bottom_right[0], bottom_right[1])
            if not any(self.bboxes_overlap(candidate_bbox, used_bbox) for used_bbox in occupied_bboxes):
                return (clamped_x, clamped_y), candidate_bbox

    scan_step_y = max(12, text_height + baseline + 4)
    scan_step_x = max(24, text_width // 3)
    for scan_y in range(min_baseline_y, max_baseline_y + 1, scan_step_y):
        for scan_x in range(0, max_text_x + 1, scan_step_x):
            top_left, bottom_right, _, _, _ = self.get_text_bbox(
                text, (scan_x, scan_y), font=font, font_scale=font_scale, thickness=thickness
            )
            candidate_bbox = (top_left[0], top_left[1], bottom_right[0], bottom_right[1])
            if not any(self.bboxes_overlap(candidate_bbox, used_bbox) for used_bbox in occupied_bboxes):
                return (scan_x, scan_y), candidate_bbox

    fallback_x = int(np.clip(anchor_x + 10, 0, max_text_x))
    fallback_y = int(np.clip(anchor_y - 10, min_baseline_y, max_baseline_y))
    top_left, bottom_right, _, _, _ = self.get_text_bbox(
        text, (fallback_x, fallback_y), font=font, font_scale=font_scale, thickness=thickness
    )
    fallback_bbox = (top_left[0], top_left[1], bottom_right[0], bottom_right[1])
    return (fallback_x, fallback_y), fallback_bbox


def draw_text_with_background(self,image, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.23, text_color=(255, 255, 255), bg_color=(0, 0, 0), thickness=1):
    top_left, bottom_right, _, _, _ = self.get_text_bbox(
        text, position, font=font, font_scale=font_scale, thickness=thickness
    )
    # 绘制背景矩形
    cv2.rectangle(image, top_left, bottom_right, bg_color, cv2.FILLED)
    # 绘制文本
    cv2.putText(image, text, position, font, font_scale, text_color, thickness, cv2.LINE_AA)


def render_manual_workspace_s2_result_image(self, workspace_mask, line_segments, display_points):
    if getattr(self, "image_infrared_copy", None) is None:
        result_image = np.zeros(workspace_mask.shape[:2], dtype=np.uint8)
    else:
        result_image = np.array(self.image_infrared_copy, copy=True)

    manual_workspace = self.load_manual_workspace_quad()
    if manual_workspace is not None:
        polygon_points = np.array(manual_workspace["corner_pixels"], dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(result_image, [polygon_points], True, 180, 2)

    for segment_start, segment_end in line_segments.get("vertical", []):
        cv2.line(
            result_image,
            (int(segment_start[0]), int(segment_start[1])),
            (int(segment_end[0]), int(segment_end[1])),
            110,
            1,
        )
    for segment_start, segment_end in line_segments.get("horizontal", []):
        cv2.line(
            result_image,
            (int(segment_start[0]), int(segment_start[1])),
            (int(segment_end[0]), int(segment_end[1])),
            110,
            1,
        )

    occupied_label_bboxes = []
    for display_idx, pix_coord, world_coord, _, status_detail in display_points:
        cv2.circle(result_image, (int(pix_coord[0]), int(pix_coord[1])), 3, 255, -1)
        label_text = f"{display_idx}"
        label_position, label_bbox = self.find_non_overlapping_label_position(
            result_image.shape,
            pix_coord,
            label_text,
            occupied_label_bboxes,
        )
        occupied_label_bboxes.append(label_bbox)
        self.draw_text_with_background(result_image, label_text, label_position)

    return result_image


def format_result_display_label(self, display_idx, world_coord, status_text):
    if self.current_result_request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
        return f"{display_idx}, {world_coord}, {status_text}"

    tcp_x = float(world_coord[0])
    tcp_y = float(world_coord[1])
    tcp_z = float(world_coord[2])
    return f"{display_idx}, tcp=({tcp_x:.1f},{tcp_y:.1f},{tcp_z:.1f}), {status_text}"


def publish_manual_workspace_s2_result(self, result_image, points_array_msg):
    result_image_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='mono8')
    result_image_msg.header.stamp = rospy.Time.now()
    result_image_msg.header.frame_id = "infrared_camera"
    self.manual_workspace_s2_result_raw_pub.publish(result_image_msg)
    self.result_image_raw_pub.publish(result_image_msg)

    compress_msg = CompressedImage()
    compress_msg.header.stamp = result_image_msg.header.stamp
    compress_msg.header.frame_id = result_image_msg.header.frame_id
    compress_msg.format = "jpeg"

    _, jpeg_data = cv2.imencode(
        '.jpg', result_image, [cv2.IMWRITE_JPEG_QUALITY, 85]
    )

    compress_msg.data = jpeg_data.tobytes()
    self.image_pub.publish(compress_msg)
    self.coordinate_publisher.publish(points_array_msg)
    self.manual_workspace_s2_points_pub.publish(points_array_msg)


def draw_mask_contours(self, result_image, workspace_mask, color, thickness):
    if workspace_mask is None:
        return

    contours, _ = cv2.findContours(
        workspace_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return

    significant_contours = [
        contour for contour in contours
        if cv2.contourArea(contour) >= 25.0
    ]
    if not significant_contours:
        significant_contours = [max(contours, key=cv2.contourArea)]

    cv2.drawContours(result_image, significant_contours, -1, color, thickness)


def draw_scan_workspace_overlay(self, result_image):
    if self.current_result_request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
        workspace_mask = self.get_scan_workspace_pixel_mask()
    else:
        workspace_mask = self.get_manual_workspace_cabin_polygon_pixel_mask()
        if workspace_mask is None:
            workspace_mask = self.get_scan_workspace_pixel_mask()
    self.draw_mask_contours(result_image, workspace_mask, 180, 2)


def draw_travel_range_overlay(self, result_image):
    workspace_mask = self.get_travel_range_pixel_mask()
    self.draw_mask_contours(result_image, workspace_mask, 120, 1)
