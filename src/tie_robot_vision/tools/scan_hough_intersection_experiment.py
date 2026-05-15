#!/usr/bin/env python3

"""Report-only Hough line/intersection extraction for scan response maps."""

from __future__ import annotations

import math

import cv2
import numpy as np


def _finite_valid_mask(response_map, valid_mask=None):
    response = np.asarray(response_map, dtype=np.float32)
    valid = np.isfinite(response)
    if valid_mask is not None:
        candidate = np.asarray(valid_mask, dtype=bool)
        if candidate.shape == response.shape:
            valid &= candidate
    return valid


def normalize01(response_map, valid_mask=None, low=2.0, high=98.0):
    response = np.asarray(response_map, dtype=np.float32)
    valid = _finite_valid_mask(response, valid_mask)
    output = np.zeros(response.shape[:2], dtype=np.float32)
    if response.ndim != 2 or response.size == 0 or not np.any(valid):
        return output

    values = response[valid]
    lower = float(np.percentile(values, float(low)))
    upper = float(np.percentile(values, float(high)))
    if upper <= lower + 1e-6:
        lower = float(np.min(values))
        upper = float(np.max(values))
    if upper <= lower + 1e-6:
        if upper > 0.0:
            output[valid] = 1.0
        return output

    output = np.clip((response - lower) / (upper - lower), 0.0, 1.0).astype(np.float32)
    output[~valid] = 0.0
    return output


def _build_hough_binary(response_map, valid_mask=None, threshold_percentile=86.0):
    response = np.asarray(response_map, dtype=np.float32)
    valid = _finite_valid_mask(response, valid_mask)
    if response.ndim != 2 or response.size == 0 or not np.any(valid):
        return np.zeros(response.shape[:2], dtype=np.uint8), 0.0

    normalized = normalize01(response, valid)
    valid_values = normalized[valid]
    if valid_values.size == 0 or float(np.max(valid_values)) <= 1e-6:
        return np.zeros(response.shape[:2], dtype=np.uint8), 0.0

    threshold = float(np.percentile(valid_values, float(threshold_percentile)))
    if threshold <= 1e-6:
        threshold = max(0.05, float(np.mean(valid_values) + np.std(valid_values)))
    binary = ((normalized >= threshold) & valid).astype(np.uint8) * 255
    if int(np.count_nonzero(binary)) == 0:
        return binary, threshold

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
    binary = cv2.medianBlur(binary, 3)
    return binary, threshold


def _skeletonize(binary):
    binary = np.asarray(binary, dtype=np.uint8)
    if binary.size == 0 or int(np.count_nonzero(binary)) == 0:
        return binary
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        return cv2.ximgproc.thinning(binary, cv2.ximgproc.THINNING_ZHANGSUEN)

    skeleton = np.zeros_like(binary)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    working = binary.copy()
    while True:
        eroded = cv2.erode(working, element)
        opened = cv2.dilate(eroded, element)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(working, opened))
        working = eroded
        if cv2.countNonZero(working) == 0:
            break
    return skeleton


def _line_orientation_and_rho(line, angle_tolerance_deg):
    x1, y1, x2, y2 = [float(value) for value in line]
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        return None, None, 0.0

    angle = abs(math.degrees(math.atan2(dy, dx)))
    angle = min(angle, abs(180.0 - angle))
    tolerance = abs(float(angle_tolerance_deg))
    if angle <= tolerance:
        return "horizontal", (y1 + y2) * 0.5, length
    if abs(90.0 - angle) <= tolerance:
        return "vertical", (x1 + x2) * 0.5, length
    return None, None, length


def _cluster_rhos(weighted_rhos, cluster_tolerance_px):
    if not weighted_rhos:
        return []
    sorted_items = sorted((float(rho), float(max(weight, 1e-6))) for rho, weight in weighted_rhos)
    clusters = []
    current = [sorted_items[0]]
    for rho, weight in sorted_items[1:]:
        current_center = sum(item_rho * item_weight for item_rho, item_weight in current) / sum(
            item_weight for _item_rho, item_weight in current
        )
        if abs(rho - current_center) <= float(cluster_tolerance_px):
            current.append((rho, weight))
        else:
            clusters.append(current)
            current = [(rho, weight)]
    clusters.append(current)

    centers = []
    for cluster in clusters:
        weight_sum = sum(weight for _rho, weight in cluster)
        centers.append(sum(rho * weight for rho, weight in cluster) / weight_sum)
    return [float(center) for center in centers]


def extract_hough_intersections(
    response_map,
    valid_mask=None,
    threshold_percentile=86.0,
    hough_threshold=18,
    min_line_length=24,
    max_line_gap=8,
    angle_tolerance_deg=14.0,
    cluster_tolerance_px=6.0,
):
    response = np.asarray(response_map, dtype=np.float32)
    if response.ndim != 2 or response.size == 0:
        return {
            "vertical_lines": [],
            "horizontal_lines": [],
            "line_counts": [0, 0],
            "intersections": [],
            "raw_line_count": 0,
            "accepted_line_count": 0,
            "binary_pixels": 0,
            "threshold": 0.0,
            "binary_image": np.zeros(response.shape[:2], dtype=np.uint8),
            "skeleton_image": np.zeros(response.shape[:2], dtype=np.uint8),
            "line_segments": [],
        }

    binary, threshold = _build_hough_binary(
        response,
        valid_mask=valid_mask,
        threshold_percentile=threshold_percentile,
    )
    skeleton = _skeletonize(binary)
    if int(np.count_nonzero(skeleton)) == 0:
        return {
            "vertical_lines": [],
            "horizontal_lines": [],
            "line_counts": [0, 0],
            "intersections": [],
            "raw_line_count": 0,
            "accepted_line_count": 0,
            "binary_pixels": int(np.count_nonzero(binary)),
            "threshold": float(threshold),
            "binary_image": binary,
            "skeleton_image": skeleton,
            "line_segments": [],
        }

    raw_lines = cv2.HoughLinesP(
        skeleton,
        rho=1,
        theta=np.pi / 180.0,
        threshold=int(hough_threshold),
        minLineLength=int(min_line_length),
        maxLineGap=int(max_line_gap),
    )
    raw_line_count = 0 if raw_lines is None else int(len(raw_lines))
    vertical_rhos = []
    horizontal_rhos = []
    line_segments = []
    if raw_lines is not None:
        for line in raw_lines:
            x1, y1, x2, y2 = [int(value) for value in line[0]]
            orientation, rho, length = _line_orientation_and_rho(
                (x1, y1, x2, y2),
                angle_tolerance_deg,
            )
            if orientation is None:
                continue
            segment = {
                "orientation": orientation,
                "rho": float(rho),
                "length": float(length),
                "points": [int(x1), int(y1), int(x2), int(y2)],
            }
            line_segments.append(segment)
            if orientation == "vertical":
                vertical_rhos.append((rho, length))
            else:
                horizontal_rhos.append((rho, length))

    vertical_lines = _cluster_rhos(vertical_rhos, cluster_tolerance_px)
    horizontal_lines = _cluster_rhos(horizontal_rhos, cluster_tolerance_px)
    height, width = response.shape[:2]
    intersections = []
    valid = _finite_valid_mask(response, valid_mask)
    for x_value in vertical_lines:
        for y_value in horizontal_lines:
            x_index = int(round(float(x_value)))
            y_index = int(round(float(y_value)))
            if x_index < 0 or y_index < 0 or x_index >= width or y_index >= height:
                continue
            if valid.shape == response.shape and not bool(valid[y_index, x_index]):
                continue
            intersections.append([float(x_value), float(y_value)])

    return {
        "vertical_lines": vertical_lines,
        "horizontal_lines": horizontal_lines,
        "line_counts": [int(len(vertical_lines)), int(len(horizontal_lines))],
        "intersections": intersections,
        "raw_line_count": raw_line_count,
        "accepted_line_count": int(len(line_segments)),
        "binary_pixels": int(np.count_nonzero(binary)),
        "threshold": float(threshold),
        "binary_image": binary,
        "skeleton_image": skeleton,
        "line_segments": line_segments,
    }


def render_heatmap(response_map, valid_mask=None):
    image_u8 = (normalize01(response_map, valid_mask) * 255.0).astype(np.uint8)
    return cv2.applyColorMap(image_u8, cv2.COLORMAP_TURBO)


def _draw_label(image, label):
    output = image.copy()
    text = str(label)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    cv2.rectangle(output, (10, 10), (26 + text_width, 26 + text_height + baseline), (9, 13, 18), -1)
    cv2.putText(output, text, (18, 29), font, scale, (245, 248, 246), thickness, cv2.LINE_AA)
    return output


def _gray_to_bgr(gray_image):
    gray = np.asarray(gray_image, dtype=np.uint8)
    if gray.ndim == 3:
        return gray
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def render_hough_stage_image(response_map, valid_mask, hough_result, stage_name, label=""):
    stage_name = str(stage_name or "intersections")
    label = str(label or stage_name)
    if stage_name == "response":
        return _draw_label(render_heatmap(response_map, valid_mask), label)
    if stage_name == "binary":
        return _draw_label(_gray_to_bgr(hough_result.get("binary_image", np.zeros_like(response_map))), label)
    if stage_name == "skeleton":
        skeleton = np.asarray(hough_result.get("skeleton_image", np.zeros_like(response_map)), dtype=np.uint8)
        image = np.zeros((*skeleton.shape[:2], 3), dtype=np.uint8)
        image[skeleton > 0] = (0, 255, 90)
        return _draw_label(image, label)

    image = render_heatmap(response_map, valid_mask)
    height, width = image.shape[:2]
    if stage_name in ("segments", "hough_segments"):
        for segment in hough_result.get("line_segments", []):
            x1, y1, x2, y2 = [int(value) for value in segment.get("points", [0, 0, 0, 0])]
            color = (35, 245, 150) if segment.get("orientation") == "vertical" else (30, 190, 255)
            cv2.line(image, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        return _draw_label(image, label)

    if stage_name in ("clustered_lines", "intersections"):
        for x_value in hough_result.get("vertical_lines", []):
            x = int(round(float(x_value)))
            if 0 <= x < width:
                cv2.line(image, (x, 0), (x, height - 1), (0, 255, 80), 1, cv2.LINE_AA)
        for y_value in hough_result.get("horizontal_lines", []):
            y = int(round(float(y_value)))
            if 0 <= y < height:
                cv2.line(image, (0, y), (width - 1, y), (0, 220, 255), 1, cv2.LINE_AA)

    if stage_name == "intersections":
        for point in hough_result.get("intersections", [])[:800]:
            x = int(round(float(point[0])))
            y = int(round(float(point[1])))
            if 0 <= x < width and 0 <= y < height:
                cv2.circle(image, (x, y), 3, (0, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(image, (x, y), 4, (15, 20, 20), 1, cv2.LINE_AA)

    return _draw_label(image, label)


def render_hough_overlay(response_map, valid_mask, hough_result, label="hough"):
    return render_hough_stage_image(response_map, valid_mask, hough_result, "intersections", label=label)


def json_safe_hough_result(hough_result):
    return {
        "line_counts": [int(value) for value in hough_result.get("line_counts", [0, 0])],
        "point_count": int(len(hough_result.get("intersections", []) or [])),
        "vertical_lines": [float(value) for value in hough_result.get("vertical_lines", [])],
        "horizontal_lines": [float(value) for value in hough_result.get("horizontal_lines", [])],
        "intersections": [
            [float(point[0]), float(point[1])]
            for point in hough_result.get("intersections", [])
        ],
        "raw_line_count": int(hough_result.get("raw_line_count", 0)),
        "accepted_line_count": int(hough_result.get("accepted_line_count", 0)),
        "binary_pixels": int(hough_result.get("binary_pixels", 0)),
        "threshold": float(hough_result.get("threshold", 0.0)),
    }
