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

def calculate_intersections(lines):
    intersections = []
    angles = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            x1, y1, x2, y2 = lines[i][0]
            x3, y3, x4, y4 = lines[j][0]

            # 计算每条线段的方向向量
            v1 = (x2 - x1, y2 - y1)
            v2 = (x4 - x3, y4 - y3)

            # 计算两条线的夹角
            dot_product = v1[0] * v2[0] + v1[1] * v2[1]
            cross_product = v1[0] * v2[1] - v1[1] * v2[0]
            angle_between_lines = np.arctan2(cross_product, dot_product)
            angle_between_lines = np.abs(np.degrees(angle_between_lines))

            if 35 <= angle_between_lines <= 150:
                denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
                if denom == 0:
                    continue  # 平行,没有交点

                px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
                py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

                if (min(x1, x2) <= px <= max(x1, x2) and min(y1, y2) <= py <= max(y1, y2) and
                    min(x3, x4) <= px <= max(x3, x4) and min(y3, y4) <= py <= max(y3, y4)):
                    intersections.append((px, py))
                    angles.append(angle_between_lines)
    return intersections, angles


def jump_bind_callback(self, msg):
    self.jump_bind_enabled = bool(msg.data)


def should_execute_selected_point_number(self, point_number):
    if point_number in (None, "-"):
        return False
    return not bool(getattr(self, "jump_bind_enabled", False)) or int(point_number) in (1, 4)


def get_selected_point_status(self, point_number):
    return "selected" if self.should_execute_selected_point_number(point_number) else "jump_skipped"


def get_travel_range_reject_reasons(self, calibrated_x, calibrated_y):
    max_x = getattr(self, "travel_range_max_x_mm", 360.0)
    max_y = getattr(self, "travel_range_max_y_mm", 320.0)
    reasons = []
    if calibrated_x < 0:
        reasons.append("X小于0")
    elif calibrated_x > max_x:
        reasons.append(f"X超过{max_x:.0f}")
    if calibrated_y < 0:
        reasons.append("Y小于0")
    elif calibrated_y > max_y:
        reasons.append(f"Y超过{max_y:.0f}")
    return reasons


def get_matrix_selection_reject_reasons(self, calibrated_x, calibrated_y):
    max_x = getattr(self, "matrix_selection_max_x_mm", 500.0)
    max_y = getattr(self, "matrix_selection_max_y_mm", 500.0)
    reasons = []
    if calibrated_x < 0:
        reasons.append("X小于0")
    elif calibrated_x > max_x:
        reasons.append(f"X超过{max_x:.0f}")
    if calibrated_y < 0:
        reasons.append("Y小于0")
    elif calibrated_y > max_y:
        reasons.append(f"Y超过{max_y:.0f}")
    return reasons


def get_display_status_text(self, status, detail=""):
    if status == "selected":
        return "SEL"
    if status == "jump_skipped":
        return "SKIP"
    if status == "in_range":
        return "IN"
    if status == "out_of_range":
        return detail or "OUT"
    if status == "zero_world":
        return "ZERO"
    return "CAND"


def axis_angle_diff_deg(x1, y1, x2, y2):
    angle_deg = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1))) % 180.0
    return min(angle_deg, abs(90.0 - angle_deg), abs(180.0 - angle_deg))


def is_near_axis_aligned_line(self, x1, y1, x2, y2):
    return self.axis_angle_diff_deg(x1, y1, x2, y2) <= getattr(
        self,
        "axis_alignment_tolerance_deg",
        15.0
    )


def should_draw_display_label(self, status):
    return status in ("selected", "jump_skipped", "in_range", "out_of_range", "zero_world") or bool(getattr(self, "show_candidate_labels", False))


def angle_between(self, line1, line2):
    x1, y1, x2, y2 = line1[0]
    x3, y3, x4, y4 = line2[0]

    # 计算每条线段的方向向量
    v1 = (x2 - x1, y2 - y1)
    v2 = (x4 - x3, y4 - y3)

    # 计算两条线的夹角
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    angle_between_lines = np.arctan2(cross_product, dot_product)
    angle_between_lines = np.abs(np.degrees(angle_between_lines))

    # 计算每条线段的单位方向向量
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)
    if v1_norm != 0:
        v1_unit = (v1[0] / v1_norm, v1[1] / v1_norm)
    else:
        v1_unit = (0, 0)  # 或者其他处理方式

    if v2_norm != 0:
        v2_unit = (v2[0] / v2_norm, v2[1] / v2_norm)
    else:
        v2_unit = (0, 0)  # 或者其他处理方式

    # 计算角平分线的方向向量
    bisector = (v1_unit[0] + v2_unit[0], v1_unit[1] + v2_unit[1])
    bisector_norm = np.linalg.norm(bisector)
    if bisector_norm != 0:
        bisector_unit = (bisector[0] / bisector_norm, bisector[1] / bisector_norm)
    else:
        bisector_unit = (0, 0)

    # 计算角平分线与水平线的夹角
    angle_with_horizontal = np.arctan2(bisector_unit[1], bisector_unit[0])
    angle_with_horizontal = np.degrees(angle_with_horizontal)
    return angle_between_lines, angle_with_horizontal


def remove_small_foreground_components(self, binary_image, min_area_px=None):
    if binary_image is None:
        return binary_image

    min_area_px = (
        getattr(self, "binary_small_blob_min_area_px", 20)
        if min_area_px is None else int(min_area_px)
    )
    if min_area_px <= 1:
        return binary_image

    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(binary_image, connectivity=8)
    filtered_binary = np.zeros_like(binary_image)
    for component_idx in range(1, component_count):
        component_area = stats[component_idx, cv2.CC_STAT_AREA]
        if component_area >= min_area_px:
            filtered_binary[labels == component_idx] = 255
    return filtered_binary


def mark_sign_points(self, step=4, max_points=8000):
    """
    黑色: x>0 且 y>0
    白色: x<0 且 y<0
    只处理部分像素(步长/抽样)避免过慢
    """
    if (not hasattr(self, 'x_channel')) or (not hasattr(self, 'y_channel')):
        return
    if self.x_channel is None or self.y_channel is None:
        return
    if self.image_infrared_copy is None:
        return

    x_ch = self.x_channel
    y_ch = self.y_channel

    # 条件掩码
    pos_mask = (x_ch > 0) & (y_ch > 0)
    neg_mask = (x_ch < 0) & (y_ch < 0)

    # 取坐标
    pos_idx = np.column_stack(np.where(pos_mask))
    neg_idx = np.column_stack(np.where(neg_mask))

    # 降采样
    pos_idx = pos_idx[::step]
    neg_idx = neg_idx[::step]

    # 限制最大数量
    if pos_idx.shape[0] > max_points:
        pos_idx = pos_idx[np.linspace(0, pos_idx.shape[0]-1, max_points, dtype=int)]
    if neg_idx.shape[0] > max_points:
        neg_idx = neg_idx[np.linspace(0, neg_idx.shape[0]-1, max_points, dtype=int)]

    # 绘制（注意图像是 (H,W)，坐标 (y,x)）
    for y, x in pos_idx:
        # 黑色
        self.image_infrared_copy[y, x] = 0
    for y, x in neg_idx:
        # 白色
        self.image_infrared_copy[y, x] = 255


def group_points_by_axis(self, centers, axis_index=0, threshold=40):
    if not centers:
        return []

    grouped_points = []
    sorted_centers = sorted(
        centers,
        key=lambda item: (item[2][axis_index], item[2][1 - axis_index], item[0])
    )

    current_group = [sorted_centers[0]]
    current_mean = float(sorted_centers[0][2][axis_index])
    for item in sorted_centers[1:]:
        axis_value = float(item[2][axis_index])
        if abs(axis_value - current_mean) <= threshold:
            current_group.append(item)
            current_mean = sum(point[2][axis_index] for point in current_group) / len(current_group)
        else:
            grouped_points.append(current_group)
            current_group = [item]
            current_mean = axis_value

    if current_group:
        grouped_points.append(current_group)

    return grouped_points


def match_points_between_rows(self, upper_row, lower_row, column_threshold=45):
    matched_pairs = []
    used_lower_indices = set()

    upper_sorted = sorted(upper_row, key=lambda item: (item[2][1], item[2][0], item[0]))
    lower_sorted = sorted(lower_row, key=lambda item: (item[2][1], item[2][0], item[0]))

    for upper_item in upper_sorted:
        upper_y = upper_item[2][1]
        best_lower_index = None
        best_gap = None

        for lower_index, lower_item in enumerate(lower_sorted):
            if lower_index in used_lower_indices:
                continue

            gap = abs(upper_y - lower_item[2][1])
            if gap > column_threshold:
                continue

            if best_gap is None or gap < best_gap:
                best_gap = gap
                best_lower_index = lower_index

        if best_lower_index is None:
            continue

        used_lower_indices.add(best_lower_index)
        matched_pairs.append((upper_item, lower_sorted[best_lower_index], best_gap))

    matched_pairs.sort(
        key=lambda pair: (
            min(pair[0][2][1], pair[1][2][1]),
            pair[2],
            pair[0][2][0],
            pair[1][2][0]
        )
    )
    return matched_pairs


def score_matrix_candidate(self, matrix_points, column_gaps):
    distance_scores = sorted(
        (
            point[2][0] * point[2][0] + point[2][1] * point[2][1],
            point[2][0],
            point[2][1],
            point[0]
        )
        for point in matrix_points
    )
    x_values = sorted(point[2][0] for point in matrix_points)
    y_values = sorted(point[2][1] for point in matrix_points)
    return tuple(distance_scores + [(tuple(x_values), tuple(y_values), tuple(sorted(column_gaps)))])


def sort_matrix_points(self, matrix_points):
    if len(matrix_points) != 4:
        return sorted(
            matrix_points,
            key=lambda item: (
                item[2][0] * item[2][0] + item[2][1] * item[2][1],
                item[2][0],
                item[2][1],
                item[0]
            )
        )

    def pixel_xy(point):
        return float(point[1][0]), float(point[1][1])

    def calibrated_xy(point):
        return float(point[2][0]), float(point[2][1])

    # Start numbering from the point nearest the calibrated world origin.
    point1 = min(
        matrix_points,
        key=lambda item: (
            calibrated_xy(item)[0] * calibrated_xy(item)[0] + calibrated_xy(item)[1] * calibrated_xy(item)[1],
            calibrated_xy(item)[0],
            calibrated_xy(item)[1],
            item[0]
        )
    )
    point1_x, point1_y = pixel_xy(point1)
    remaining_points = [point for point in matrix_points if point is not point1]

    point2 = min(
        remaining_points,
        key=lambda item: (
            abs(pixel_xy(item)[1] - point1_y),
            -abs(pixel_xy(item)[0] - point1_x),
            pixel_xy(item)[0],
            item[0]
        )
    )
    remaining_points = [point for point in remaining_points if point is not point2]

    point3 = min(
        remaining_points,
        key=lambda item: (
            abs(pixel_xy(item)[0] - point1_x),
            -abs(pixel_xy(item)[1] - point1_y),
            pixel_xy(item)[1],
            item[0]
        )
    )
    point4 = next(point for point in remaining_points if point is not point3)

    return [point1, point2, point3, point4]


def get_selected_point_numbers(self, selected_points):
    return {
        source_idx: selected_idx + 1
        for selected_idx, (source_idx, _, _) in enumerate(selected_points)
    }


def select_output_centers_for_mode(self, request_mode, in_range_centers, selected_centers):
    if request_mode in (PROCESS_IMAGE_MODE_SCAN_ONLY, PROCESS_IMAGE_MODE_EXECUTION_REFINE):
        return list(in_range_centers)
    if request_mode == PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT:
        return self.sort_matrix_points(in_range_centers)
    return list(selected_centers)


def select_display_matrix_centers(self, downstream_matrix_centers, in_range_centers, candidate_centers):
    if downstream_matrix_centers:
        return list(downstream_matrix_centers)

    display_range_centers = [
        center_record
        for center_record in candidate_centers
        if self.is_point_in_display_bind_range(center_record[2][0], center_record[2][1])
    ]
    fallback_matrix = self.select_nearest_origin_matrix_points(display_range_centers)
    if fallback_matrix:
        return fallback_matrix
    nearest_in_range_centers = sorted(
        display_range_centers,
        key=lambda item: (
            item[2][0] * item[2][0] + item[2][1] * item[2][1],
            item[2][0],
            item[2][1],
            item[0]
        )
    )[:4]
    return self.sort_matrix_points(nearest_in_range_centers)


def build_matrix_display_points(self, matrix_centers, mark_travel_out_of_range=False):
    display_point_numbers = self.get_selected_point_numbers(matrix_centers)
    display_points = []

    for source_idx, center, calibrated_world_coord in matrix_centers:
        display_idx = display_point_numbers.get(source_idx, "-")
        pix_coord = [int(center[0]), int(center[1])]
        display_coord = calibrated_world_coord
        status = self.get_selected_point_status(display_idx)
        status_detail = ""

        if mark_travel_out_of_range and not self.is_point_in_travel_range(
            calibrated_world_coord[0],
            calibrated_world_coord[1]
        ):
            status = "out_of_range"
            reject_reasons = self.get_travel_range_reject_reasons(
                calibrated_world_coord[0],
                calibrated_world_coord[1]
            )
            status_detail = "OUT_" + "+".join(reason[0] for reason in reject_reasons) if reject_reasons else "OUT"

        display_points.append(
            (
                display_idx,
                pix_coord,
                display_coord,
                status,
                status_detail
            )
        )

    return display_points


def filter_close_points_by_origin(self, centers, min_distance_mm=100.0):
    if not centers:
        return []

    min_distance_sq = min_distance_mm * min_distance_mm
    sorted_centers = sorted(
        centers,
        key=lambda item: (
            item[2][0] * item[2][0] + item[2][1] * item[2][1],
            item[2][0],
            item[2][1],
            item[0]
        )
    )

    rejected_indexes = set()
    for candidate_index, candidate in enumerate(sorted_centers):
        candidate_x, candidate_y = candidate[2][0], candidate[2][1]
        for other_index in range(candidate_index + 1, len(sorted_centers)):
            other_x, other_y = sorted_centers[other_index][2][0], sorted_centers[other_index][2][1]
            dx = candidate_x - other_x
            dy = candidate_y - other_y
            if dx * dx + dy * dy < min_distance_sq:
                rejected_indexes.add(candidate_index)
                rejected_indexes.add(other_index)

    return [
        candidate
        for candidate_index, candidate in enumerate(sorted_centers)
        if candidate_index not in rejected_indexes
    ]


def filter_candidate_centers_for_request_mode(self, candidate_centers, request_mode):
    if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
        return list(candidate_centers), 0

    raw_candidate_count = len(candidate_centers)
    filtered_candidate_centers = self.filter_close_points_by_origin(candidate_centers)
    return filtered_candidate_centers, raw_candidate_count - len(filtered_candidate_centers)


def select_nearest_origin_matrix_points(self, centers, max_points=4, row_threshold=40, column_threshold=45):
    if len(centers) < max_points:
        return []

    rows = self.group_points_by_axis(centers, axis_index=0, threshold=row_threshold)
    if len(rows) < 2:
        return []

    best_points = []
    best_score = None
    for upper_index in range(len(rows) - 1):
        upper_row = rows[upper_index]
        if len(upper_row) < 2:
            continue

        for lower_index in range(upper_index + 1, len(rows)):
            lower_row = rows[lower_index]
            if len(lower_row) < 2:
                continue

            matched_pairs = self.match_points_between_rows(
                upper_row,
                lower_row,
                column_threshold=column_threshold
            )
            if len(matched_pairs) < 2:
                continue

            for first_pair_index in range(len(matched_pairs) - 1):
                for second_pair_index in range(first_pair_index + 1, len(matched_pairs)):
                    first_pair = matched_pairs[first_pair_index]
                    second_pair = matched_pairs[second_pair_index]
                    selected_points = [
                        first_pair[0],
                        first_pair[1],
                        second_pair[0],
                        second_pair[1],
                    ]
                    sorted_points = self.sort_matrix_points(selected_points)
                    candidate_score = self.score_matrix_candidate(
                        sorted_points,
                        [first_pair[2], second_pair[2]]
                    )
                    if best_score is None or candidate_score < best_score:
                        best_score = candidate_score
                        best_points = sorted_points

    return best_points
