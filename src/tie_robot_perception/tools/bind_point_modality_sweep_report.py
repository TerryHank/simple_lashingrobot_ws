#!/usr/bin/env python3
"""Build a report-only modality sweep for bind-point classification."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import re
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from sklearn.cluster import DBSCAN


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src"
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from tie_robot_perception.pointai import scan_surface_dp  # noqa: E402
from tie_robot_perception.pointai.bind_point_classification import (  # noqa: E402
    ClassificationConfig,
    classify_pre_bind_white_box,
    extract_evidence_bundle,
)
from tie_robot_perception.pointai.tcp_display import (  # noqa: E402
    DEFAULT_TCP_DISPLAY_CONFIG_PATH,
    camera_channels_to_tcp_jaw_channels,
    camera_coord_to_tcp_jaw_coord,
)


DEFAULT_SNAPSHOT_DIR = WORKSPACE_ROOT / "docs" / "releases" / "slam_v30" / "visual_modalities"
DEFAULT_REPORT_DIR = (
    WORKSPACE_ROOT
    / "src"
    / "tie_robot_web"
    / "web"
    / "reports"
    / "bind_point_classification_modalities_current"
)

READ_ONLY_GUARD_PATHS = [
    WORKSPACE_ROOT / "src" / "tie_robot_process" / "data" / "pseudo_slam_points.json",
    WORKSPACE_ROOT / "src" / "tie_robot_process" / "data" / "pseudo_slam_bind_path.json",
    WORKSPACE_ROOT / "src" / "tie_robot_process" / "data" / "bind_execution_memory.json",
    WORKSPACE_ROOT / "src" / "tie_robot_perception" / "data" / "bind_classification_events.jsonl",
]

LOW_SCORE_THRESHOLD = 0.40
HIGH_SCORE_THRESHOLD = 0.64
EXECUTION_TCP_ROI_BOUNDS = {
    "min_x": 0.0,
    "max_x": 380.0,
    "min_y": 0.0,
    "max_y": 330.0,
    "min_z": 0.0,
    "max_z": 160.0,
}
TCP_OCCLUSION_MASK_RECT = (160, 0, 523, 80)
HOUGH_THRESHOLD = 45
HOUGH_MIN_LINE_LENGTH = 60
HOUGH_MAX_LINE_GAP = 150
AXIS_ALIGNMENT_TOLERANCE_DEG = 15.0
SMALL_BLOB_MIN_AREA_PX = 20
EXECUTION_POINT_SOURCE = "execution_refine_hough_tcp_roi"
EXECUTION_ROI_NOTE = "执行层使用 TCP 工具坐标/线性模组工作范围，不使用手动画的像素工作区作为执行 ROI"

LABEL_META = {
    "bound": {
        "title": "已绑扎",
        "tone": "red",
        "color": (70, 70, 245),
        "rgb": "#ff5a5f",
    },
    "unbound": {
        "title": "未绑扎",
        "tone": "green",
        "color": (58, 205, 94),
        "rgb": "#6dd77a",
    },
    "uncertain": {
        "title": "不确定",
        "tone": "amber",
        "color": (42, 184, 235),
        "rgb": "#f2c14e",
    },
}

CJK_FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/arphic/ukai.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]
_LABEL_FONT_CACHE = {}


def json_safe(value):
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    return value


def file_digest(path):
    path = Path(path)
    if not path.exists():
        return {"exists": False, "size": 0, "sha256": None}
    digest = hashlib.sha256()
    with path.open("rb") as file_obj:
        for chunk in iter(lambda: file_obj.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"exists": True, "size": int(path.stat().st_size), "sha256": digest.hexdigest()}


def snapshot_read_only_inputs(paths=READ_ONLY_GUARD_PATHS):
    return {str(Path(path)): file_digest(path) for path in paths}


def assert_read_only_inputs_unchanged(before_state, paths=READ_ONLY_GUARD_PATHS):
    after_state = snapshot_read_only_inputs(paths)
    if before_state != after_state:
        raise RuntimeError(
            "report-only guard failed: pseudo_slam_points.json, "
            "pseudo_slam_bind_path.json or bind_classification_events.jsonl changed"
        )


def ensure_clean_dir(path):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    for child in path.iterdir():
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()


def parse_number_list(text):
    return [float(item) for item in re.findall(r"[-+]?\d+(?:\.\d+)?", str(text))]


def load_arrays(snapshot_dir):
    snapshot_dir = Path(snapshot_dir)
    arrays_dir = snapshot_dir / "arrays" if (snapshot_dir / "arrays").is_dir() else snapshot_dir
    arrays = {}
    for path in sorted(arrays_dir.glob("*.npy")):
        arrays[path.stem] = np.load(path, allow_pickle=False)
    if "Scepter_ir_image_raw" not in arrays:
        raise RuntimeError(f"snapshot missing Scepter_ir_image_raw.npy: {arrays_dir}")
    return arrays, arrays_dir


def to_gray_float(image):
    array = np.asarray(image)
    if array.ndim == 3 and array.shape[2] >= 3:
        array = cv2.cvtColor(array[:, :, :3].astype(np.uint8), cv2.COLOR_BGR2GRAY)
    return np.asarray(array, dtype=np.float32)


def normalize01(image, valid_mask=None, low=2.0, high=98.0):
    image = np.asarray(image, dtype=np.float32)
    if image.ndim != 2 or image.size == 0:
        return np.zeros(image.shape[:2], dtype=np.float32)
    valid = np.isfinite(image)
    if valid_mask is not None:
        candidate = np.asarray(valid_mask, dtype=bool)
        if candidate.shape == image.shape:
            valid &= candidate
    output = np.zeros(image.shape[:2], dtype=np.float32)
    if not np.any(valid):
        return output
    values = image[valid]
    lower = float(np.percentile(values, float(low)))
    upper = float(np.percentile(values, float(high)))
    if upper <= lower + 1e-6:
        lower = float(np.min(values))
        upper = float(np.max(values))
    if upper <= lower + 1e-6:
        output[valid] = 1.0 if upper > 0.0 else 0.0
        return output
    output = np.clip((image - lower) / (upper - lower), 0.0, 1.0).astype(np.float32)
    output[~valid] = 0.0
    return output


def valid_depth_mask(depth):
    depth = np.asarray(depth, dtype=np.float32)
    return np.isfinite(depth) & (depth > 0.0) & (depth < 60000.0)


def fill_depth(depth, valid_mask):
    depth = np.asarray(depth, dtype=np.float32)
    valid = np.asarray(valid_mask, dtype=bool) & np.isfinite(depth)
    if not np.any(valid):
        return np.zeros(depth.shape[:2], dtype=np.float32)
    median = float(np.median(depth[valid]))
    return np.where(valid, depth, median).astype(np.float32)


def local_height_response(depth, valid_mask, sigma=11.0):
    filled = fill_depth(depth, valid_mask)
    background = cv2.GaussianBlur(filled, (0, 0), sigmaX=float(sigma), sigmaY=float(sigma))
    return normalize01(background - filled, valid_mask)


def gradient_response(image, valid_mask):
    image = np.asarray(image, dtype=np.float32)
    smoothed = cv2.GaussianBlur(image, (0, 0), sigmaX=1.2, sigmaY=1.2)
    grad_x = cv2.Sobel(smoothed, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(smoothed, cv2.CV_32F, 0, 1, ksize=3)
    return normalize01(cv2.magnitude(grad_x, grad_y), valid_mask)


def dark_line_response(gray, valid_mask, sigma=7.0):
    gray = np.asarray(gray, dtype=np.float32)
    if not np.any(valid_mask):
        return np.zeros(gray.shape[:2], dtype=np.float32)
    median = float(np.median(gray[np.asarray(valid_mask, dtype=bool)]))
    filled = np.where(valid_mask, gray, median).astype(np.float32)
    background = cv2.GaussianBlur(filled, (0, 0), sigmaX=float(sigma), sigmaY=float(sigma))
    return normalize01(background - filled, valid_mask)


def axis_angle_diff_deg(x1, y1, x2, y2):
    angle_deg = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1))) % 180.0
    return min(angle_deg, abs(90.0 - angle_deg), abs(180.0 - angle_deg))


def is_near_axis_aligned_line(x1, y1, x2, y2, tolerance_deg=AXIS_ALIGNMENT_TOLERANCE_DEG):
    return axis_angle_diff_deg(x1, y1, x2, y2) <= float(tolerance_deg)


def calculate_intersections(lines):
    intersections = []
    angles = []
    for first_index in range(len(lines)):
        for second_index in range(first_index + 1, len(lines)):
            x1, y1, x2, y2 = lines[first_index][0]
            x3, y3, x4, y4 = lines[second_index][0]
            v1 = (x2 - x1, y2 - y1)
            v2 = (x4 - x3, y4 - y3)
            dot_product = v1[0] * v2[0] + v1[1] * v2[1]
            cross_product = v1[0] * v2[1] - v1[1] * v2[0]
            angle_between_lines = abs(np.degrees(np.arctan2(cross_product, dot_product)))

            if not 35 <= angle_between_lines <= 150:
                continue

            denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if denom == 0:
                continue

            px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
            py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

            if (
                min(x1, x2) <= px <= max(x1, x2)
                and min(y1, y2) <= py <= max(y1, y2)
                and min(x3, x4) <= px <= max(x3, x4)
                and min(y3, y4) <= py <= max(y3, y4)
            ):
                intersections.append((float(px), float(py)))
                angles.append(float(angle_between_lines))
    return intersections, angles


def remove_small_foreground_components(binary_image, min_area_px=SMALL_BLOB_MIN_AREA_PX):
    if binary_image is None or int(min_area_px) <= 1:
        return binary_image
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(binary_image, connectivity=8)
    filtered_binary = np.zeros_like(binary_image)
    for component_idx in range(1, component_count):
        component_area = stats[component_idx, cv2.CC_STAT_AREA]
        if component_area >= int(min_area_px):
            filtered_binary[labels == component_idx] = 255
    return filtered_binary


def get_valid_world_coord_near_pixel(raw_world_coord, pixel_x, pixel_y, search_radius=6):
    height, width = raw_world_coord.shape[:2]
    pixel_x = int(np.clip(pixel_x, 0, width - 1))
    pixel_y = int(np.clip(pixel_y, 0, height - 1))
    channels = np.asarray(raw_world_coord, dtype=np.float32)

    def read_coord(sample_x, sample_y):
        coord = [
            float(channels[sample_y, sample_x, 0]),
            float(channels[sample_y, sample_x, 1]),
            float(channels[sample_y, sample_x, 2]),
        ]
        if not all(np.isfinite(coord)) or coord[0] == 0.0 or coord[1] == 0.0 or coord[2] == 0.0:
            return None
        return coord

    direct = read_coord(pixel_x, pixel_y)
    if direct is not None:
        return direct, [pixel_x, pixel_y], False

    best_world_coord = None
    best_sample_pixel = None
    best_distance = None
    for radius in range(1, int(search_radius) + 1):
        min_y = max(0, pixel_y - radius)
        max_y = min(height - 1, pixel_y + radius)
        min_x = max(0, pixel_x - radius)
        max_x = min(width - 1, pixel_x + radius)
        for sample_y in range(min_y, max_y + 1):
            for sample_x in range(min_x, max_x + 1):
                sample_world_coord = read_coord(sample_x, sample_y)
                if sample_world_coord is None:
                    continue
                distance = (sample_x - pixel_x) ** 2 + (sample_y - pixel_y) ** 2
                if best_distance is None or distance < best_distance:
                    best_distance = distance
                    best_world_coord = sample_world_coord
                    best_sample_pixel = [sample_x, sample_y]
        if best_world_coord is not None:
            return best_world_coord, best_sample_pixel, True

    return [0.0, 0.0, 0.0], [pixel_x, pixel_y], False


def build_execution_tcp_range_pixel_mask(raw_world_coord, bounds=EXECUTION_TCP_ROI_BOUNDS):
    if raw_world_coord is None or raw_world_coord.ndim != 3 or raw_world_coord.shape[2] < 3:
        return None, {}
    x_channel = np.asarray(raw_world_coord[:, :, 0], dtype=np.float32)
    y_channel = np.asarray(raw_world_coord[:, :, 1], dtype=np.float32)
    z_channel = np.asarray(raw_world_coord[:, :, 2], dtype=np.float32)
    tcp_x, tcp_y, tcp_z = camera_channels_to_tcp_jaw_channels(x_channel, y_channel, z_channel)
    valid_camera_coord_mask = (
        np.isfinite(x_channel)
        & np.isfinite(y_channel)
        & np.isfinite(z_channel)
        & (x_channel != 0.0)
        & (y_channel != 0.0)
        & (z_channel != 0.0)
    )
    tcp_range_mask = (
        valid_camera_coord_mask
        & (tcp_x >= bounds["min_x"])
        & (tcp_x <= bounds["max_x"])
        & (tcp_y >= bounds["min_y"])
        & (tcp_y <= bounds["max_y"])
        & (tcp_z >= bounds["min_z"])
        & (tcp_z <= bounds["max_z"])
    )
    diagnostics = {
        "valid_camera_coord_pixels": int(np.count_nonzero(valid_camera_coord_mask)),
        "tcp_range_mask_pixels": int(np.count_nonzero(tcp_range_mask)),
    }
    if np.any(tcp_range_mask):
        ys, xs = np.where(tcp_range_mask)
        diagnostics["tcp_range_mask_bbox_px"] = [
            int(xs.min()),
            int(ys.min()),
            int(xs.max()),
            int(ys.max()),
        ]
    else:
        diagnostics["tcp_range_mask_bbox_px"] = None
    return tcp_range_mask.astype(np.uint8), diagnostics


def build_execution_refine_binary(arrays, tcp_range_mask):
    world_coord = arrays.get("Scepter_worldCoord_world_coord")
    if world_coord is None or world_coord.ndim != 3 or world_coord.shape[2] < 3:
        return None, {"binary_error": "snapshot missing Scepter_worldCoord_world_coord"}
    if tcp_range_mask is None:
        return None, {"binary_error": "missing TCP range mask"}

    depth_image = np.asarray(world_coord[:, :, 2], dtype=np.float32)
    depth_image = np.where(np.isfinite(depth_image), depth_image, 0.0).astype(np.float32)
    left, top, right, bottom = TCP_OCCLUSION_MASK_RECT
    left = max(0, min(depth_image.shape[1] - 1, int(left)))
    right = max(0, min(depth_image.shape[1] - 1, int(right)))
    top = max(0, min(depth_image.shape[0] - 1, int(top)))
    bottom = max(0, min(depth_image.shape[0] - 1, int(bottom)))
    if left <= right and top <= bottom:
        depth_image[top : bottom + 1, left : right + 1] = 0.0

    positive_depth = depth_image[depth_image > 0.0]
    if positive_depth.size == 0:
        return None, {"binary_error": "平面分割后没有可用于 Hough 的非平面深度像素"}
    max_depth = float(np.max(positive_depth) - 5.0)
    if max_depth <= 11.0:
        return None, {"binary_error": "平面分割后非平面深度范围过窄，无法生成 Hough 二值图"}

    binary = np.zeros(depth_image.shape[:2], dtype=np.uint8)
    binary[(depth_image >= 11.0) & (depth_image <= max_depth)] = 255
    binary[tcp_range_mask <= 0] = 0
    binary = cv2.medianBlur(binary, 3)
    binary = remove_small_foreground_components(binary)
    binary[tcp_range_mask <= 0] = 0
    return binary, {
        "binary_error": "",
        "binary_foreground_pixels": int(np.count_nonzero(binary)),
        "binary_depth_min_threshold": 11.0,
        "binary_depth_max_threshold": max_depth,
    }


def is_camera_world_coord_in_execution_tcp_range(camera_world_coord, bounds=EXECUTION_TCP_ROI_BOUNDS):
    if not isinstance(camera_world_coord, (list, tuple, np.ndarray)) or len(camera_world_coord) < 3:
        return False
    try:
        coord = [float(value) for value in camera_world_coord[:3]]
    except (TypeError, ValueError):
        return False
    if not all(np.isfinite(coord)) or coord[0] == 0.0 or coord[1] == 0.0 or coord[2] == 0.0:
        return False
    tcp_x, tcp_y, tcp_z = camera_coord_to_tcp_jaw_coord(coord)
    return (
        bounds["min_x"] <= tcp_x <= bounds["max_x"]
        and bounds["min_y"] <= tcp_y <= bounds["max_y"]
        and bounds["min_z"] <= tcp_z <= bounds["max_z"]
    )


def is_point_in_pixel_mask(pixel_x, pixel_y, pixel_mask):
    if pixel_mask is None:
        return True
    pixel_x = int(round(float(pixel_x)))
    pixel_y = int(round(float(pixel_y)))
    if pixel_x < 0 or pixel_y < 0 or pixel_y >= pixel_mask.shape[0] or pixel_x >= pixel_mask.shape[1]:
        return False
    return bool(pixel_mask[pixel_y, pixel_x])


def build_tcp_snake_sort_key(point):
    try:
        tcp_x, tcp_y, tcp_z = camera_coord_to_tcp_jaw_coord(point["world"])
        del tcp_z
        return (0, float(tcp_x), float(tcp_y), int(point["source_idx"]))
    except Exception:
        return (1, float(point["pix"][1]), -float(point["pix"][0]), int(point["source_idx"]))


def sort_execution_points_by_tcp_snake_rows(points, row_tolerance_mm=40.0):
    if len(points) < 2:
        return list(points)
    sorted_points = sorted(points, key=build_tcp_snake_sort_key)
    rows = []
    row_means = []
    row_buckets = []
    for point in sorted_points:
        sort_key = build_tcp_snake_sort_key(point)
        bucket = sort_key[0]
        row_value = float(sort_key[1])
        if not rows or bucket != row_buckets[-1] or abs(row_value - row_means[-1]) > float(row_tolerance_mm):
            rows.append([point])
            row_means.append(row_value)
            row_buckets.append(bucket)
            continue
        rows[-1].append(point)
        row_means[-1] = (row_means[-1] * (len(rows[-1]) - 1) + row_value) / len(rows[-1])

    ordered_points = []
    for row_index, row_points in enumerate(rows):
        ascending_column = (row_index % 2) == 0
        ordered_points.extend(
            sorted(
                row_points,
                key=lambda point: (
                    build_tcp_snake_sort_key(point)[2]
                    if ascending_column
                    else -build_tcp_snake_sort_key(point)[2],
                    build_tcp_snake_sort_key(point)[1],
                    build_tcp_snake_sort_key(point)[3],
                ),
            )
        )
    return ordered_points


def build_execution_refine_candidate_points(arrays, max_points=None):
    """离线复刻 MODE_EXECUTION_REFINE：TCP 执行盒 mask -> Hough -> 交点候选。

    执行层使用 TCP 工具坐标/线性模组工作范围，不使用手动画的像素工作区作为执行 ROI。
    """
    raw_world_coord = arrays.get("Scepter_worldCoord_raw_world_coord")
    tcp_range_mask, mask_diagnostics = build_execution_tcp_range_pixel_mask(raw_world_coord)
    binary, binary_diagnostics = build_execution_refine_binary(arrays, tcp_range_mask)
    diagnostics = {
        **mask_diagnostics,
        **binary_diagnostics,
        "hough_lines": 0,
        "axis_aligned_lines": 0,
        "intersections": 0,
        "centers": 0,
        "world_fallback": 0,
        "zero_world": 0,
        "candidate_points": 0,
        "in_tcp_range_candidates": 0,
        "selected_points": 0,
    }
    if binary is None:
        return [], diagnostics, {"tcp_mask": tcp_range_mask, "binary": None}

    skeleton = cv2.ximgproc.thinning(binary, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN) if hasattr(cv2, "ximgproc") else binary
    lines = cv2.HoughLinesP(
        skeleton,
        rho=1,
        theta=np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LINE_LENGTH,
        maxLineGap=HOUGH_MAX_LINE_GAP,
    )
    diagnostics["hough_lines"] = 0 if lines is None else int(len(lines))
    axis_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if is_near_axis_aligned_line(x1, y1, x2, y2):
                axis_lines.append(line)
    diagnostics["axis_aligned_lines"] = int(len(axis_lines))
    if not axis_lines:
        return [], diagnostics, {"tcp_mask": tcp_range_mask, "binary": binary, "skeleton": skeleton}

    intersections, _ = calculate_intersections(axis_lines)
    diagnostics["intersections"] = int(len(intersections))
    if not intersections:
        return [], diagnostics, {"tcp_mask": tcp_range_mask, "binary": binary, "skeleton": skeleton}

    clustering = DBSCAN(eps=15, min_samples=1).fit(np.asarray(intersections, dtype=np.float32))
    clusters = {}
    for index, label in enumerate(clustering.labels_):
        clusters.setdefault(int(label), []).append(intersections[index])

    centers = []
    world_fallback_count = 0
    for cluster_points in clusters.values():
        center_x = int(sum(point[0] for point in cluster_points) / len(cluster_points))
        center_y = int(sum(point[1] for point in cluster_points) / len(cluster_points))
        world_coord, sample_pixel, used_fallback = get_valid_world_coord_near_pixel(raw_world_coord, center_x, center_y)
        if used_fallback:
            world_fallback_count += 1
        centers.append(
            {
                "source_idx": len(centers),
                "pix": [float(center_x), float(center_y)],
                "sample_pix": [int(sample_pixel[0]), int(sample_pixel[1])],
                "world": [float(world_coord[0]), float(world_coord[1]), float(world_coord[2])],
                "used_world_fallback": bool(used_fallback),
            }
        )

    diagnostics["centers"] = int(len(centers))
    diagnostics["world_fallback"] = int(world_fallback_count)
    in_range_points = []
    zero_world_count = 0
    for center in centers:
        world = center["world"]
        if world[0] == 0.0 or world[1] == 0.0 or world[2] == 0.0:
            zero_world_count += 1
            continue
        if is_camera_world_coord_in_execution_tcp_range(world) and is_point_in_pixel_mask(center["pix"][0], center["pix"][1], tcp_range_mask):
            in_range_points.append(center)

    diagnostics["zero_world"] = int(zero_world_count)
    diagnostics["candidate_points"] = int(len(centers) - zero_world_count)
    diagnostics["in_tcp_range_candidates"] = int(len(in_range_points))
    selected_points = sort_execution_points_by_tcp_snake_rows(in_range_points)
    if max_points is not None:
        selected_points = selected_points[: int(max_points)]
    for idx, point in enumerate(selected_points, start=1):
        point["idx"] = int(idx)
        try:
            tcp_coord = camera_coord_to_tcp_jaw_coord(point["world"])
        except Exception:
            tcp_coord = [0.0, 0.0, 0.0]
        point["tcp"] = [float(tcp_coord[0]), float(tcp_coord[1]), float(tcp_coord[2])]
    diagnostics["selected_points"] = int(len(selected_points))
    return selected_points, diagnostics, {"tcp_mask": tcp_range_mask, "binary": binary, "skeleton": skeleton}


def make_modality(modalities, modality_id, label, response_map, valid_mask, note):
    response_map = normalize01(response_map, valid_mask)
    modalities.append(
        {
            "id": modality_id,
            "label": label,
            "map": response_map.astype(np.float32),
            "valid": np.asarray(valid_mask, dtype=bool),
            "note": note,
        }
    )


def build_modalities(arrays, workspace_mask):
    ir = to_gray_float(arrays["Scepter_ir_image_raw"])
    valid = np.asarray(workspace_mask, dtype=bool)
    if "Scepter_worldCoord_raw_world_coord" in arrays and arrays["Scepter_worldCoord_raw_world_coord"].ndim == 3:
        raw_z = arrays["Scepter_worldCoord_raw_world_coord"][:, :, 2].astype(np.float32)
    elif "Scepter_depth_image_raw" in arrays:
        raw_z = arrays["Scepter_depth_image_raw"].astype(np.float32)
    else:
        raw_z = np.zeros(ir.shape[:2], dtype=np.float32)
    depth_valid = valid & valid_depth_mask(raw_z)
    ir_valid = valid & np.isfinite(ir)

    depth_height = local_height_response(raw_z, depth_valid, sigma=11.0)
    depth_gradient = gradient_response(fill_depth(raw_z, depth_valid), depth_valid)
    ir_dark = dark_line_response(ir, ir_valid, sigma=7.0)
    ir_u8 = np.clip(normalize01(ir, ir_valid) * 255.0, 0, 255).astype(np.uint8)
    ir_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(ir_u8).astype(np.float32)
    ir_clahe_dark = dark_line_response(ir_clahe, ir_valid, sigma=7.0)
    combined = normalize01((0.62 * depth_height) + (0.38 * ir_dark), valid & depth_valid)
    hessian = scan_surface_dp.hessian_ridge_response(combined, valid & depth_valid)
    frangi = scan_surface_dp.multiscale_frangi_like_response(combined, valid & depth_valid)
    gradient_ridge_fusion = normalize01((0.40 * depth_gradient) + (0.30 * hessian) + (0.30 * frangi), valid)

    modalities = []
    make_modality(modalities, "raw_ir_response", "红外原图归一化", ir, ir_valid, "直接使用 IR 灰度，观察材质纹理。")
    make_modality(modalities, "ir_dark_response", "红外暗线响应", ir_dark, ir_valid, "局部背景减 IR，凸显钢筋暗线和结节阴影。")
    make_modality(modalities, "ir_clahe_dark_response", "CLAHE 红外暗线", ir_clahe_dark, ir_valid, "先做局部对比度增强，再提暗线响应。")
    make_modality(modalities, "depth_height_response", "深度高度差", depth_height, depth_valid, "局部背景深度减当前深度，凸显高出平面的结构。")
    make_modality(modalities, "depth_gradient_response", "深度梯度边缘", depth_gradient, depth_valid, "Sobel 深度梯度，凸显高度突变边界。")
    make_modality(modalities, "combined_ir_depth_response", "红外 + 深度组合", combined, valid & depth_valid, "深度高度差和红外暗线加权融合。")
    make_modality(modalities, "hessian_ridge_response", "Hessian 脊线", hessian, valid & depth_valid, "在组合响应上提二阶亮脊。")
    make_modality(modalities, "frangi_like_response", "Frangi-like 脊线", frangi, valid & depth_valid, "多尺度线状结构增强。")
    make_modality(
        modalities,
        "gradient_ridge_fusion",
        "梯度 + 脊线融合",
        gradient_ridge_fusion,
        valid & depth_valid,
        "深度梯度、Hessian 和 Frangi 的保守融合。",
    )

    if "Scepter_depth_image_raw" in arrays:
        depth_image = arrays["Scepter_depth_image_raw"].astype(np.float32)
        depth_image_valid = valid & valid_depth_mask(depth_image)
        make_modality(
            modalities,
            "depth_image_height_response",
            "原始 depth 高度差",
            local_height_response(depth_image, depth_image_valid, sigma=11.0),
            depth_image_valid,
            "直接使用 /Scepter/depth/image_raw 的局部高度差。",
        )
        make_modality(
            modalities,
            "depth_image_gradient_response",
            "原始 depth 梯度",
            gradient_response(fill_depth(depth_image, depth_image_valid), depth_image_valid),
            depth_image_valid,
            "直接使用 /Scepter/depth/image_raw 的深度梯度。",
        )

    if "pointAI_result_image_raw" in arrays:
        pointai_gray = to_gray_float(arrays["pointAI_result_image_raw"])
        pointai_valid = valid & np.isfinite(pointai_gray)
        make_modality(
            modalities,
            "pointai_result_dark_response",
            "pointAI 底图暗线",
            dark_line_response(pointai_gray, pointai_valid, sigma=7.0),
            pointai_valid,
            "使用 pointAI/result_image_raw 的暗线响应，通常接近 IR 输入。",
        )

    if "Scepter_color_image_raw" in arrays:
        color_luma = to_gray_float(arrays["Scepter_color_image_raw"])
        color_valid = valid & np.isfinite(color_luma)
        make_modality(
            modalities,
            "color_luma_dark_response",
            "彩色亮度暗线",
            dark_line_response(color_luma, color_valid, sigma=7.0),
            color_valid,
            "从彩色图亮度提暗线，作为可见光对照。",
        )

    if "Scepter_transformedColor_image_raw" in arrays:
        transformed_color_luma = to_gray_float(arrays["Scepter_transformedColor_image_raw"])
        transformed_color_valid = valid & np.isfinite(transformed_color_luma)
        make_modality(
            modalities,
            "transformed_color_luma_dark_response",
            "对齐彩色亮度暗线",
            dark_line_response(transformed_color_luma, transformed_color_valid, sigma=7.0),
            transformed_color_valid,
            "从 transformedColor 亮度提暗线，作为对齐可见光对照。",
        )

    if "Scepter_transformedDepth_image_raw" in arrays:
        transformed_depth = arrays["Scepter_transformedDepth_image_raw"].astype(np.float32)
        transformed_valid = valid & valid_depth_mask(transformed_depth)
        make_modality(
            modalities,
            "transformed_depth_height_response",
            "对齐深度高度差",
            local_height_response(transformed_depth, transformed_valid, sigma=11.0),
            transformed_valid,
            "使用 transformedDepth 的局部高度差。",
        )

    if "Scepter_worldCoord_world_coord" in arrays and arrays["Scepter_worldCoord_world_coord"].ndim == 3:
        world_z = arrays["Scepter_worldCoord_world_coord"][:, :, 2].astype(np.float32)
        world_valid = valid & valid_depth_mask(world_z)
        make_modality(
            modalities,
            "world_coord_z_height_response",
            "world_coord Z 高度差",
            local_height_response(world_z, world_valid, sigma=11.0),
            world_valid,
            "使用去平面后的 world_coord Z 做高度差对照。",
        )

    return modalities


def disc_mask(shape, radius, center=None):
    height, width = shape[:2]
    if center is None:
        center = ((width - 1) * 0.5, (height - 1) * 0.5)
    yy, xx = np.indices((height, width), dtype=np.float32)
    return ((xx - float(center[0])) ** 2 + (yy - float(center[1])) ** 2) <= float(radius) ** 2


def ring_mask(shape, inner_radius, outer_radius):
    return disc_mask(shape, outer_radius) & ~disc_mask(shape, inner_radius)


def crop_with_padding(image, center_xy, half_size, fill_value=0.0):
    image = np.asarray(image)
    size = int(half_size) * 2 + 1
    output = np.full((size, size), fill_value, dtype=image.dtype)
    center_x = int(round(float(center_xy[0])))
    center_y = int(round(float(center_xy[1])))
    x0 = center_x - int(half_size)
    y0 = center_y - int(half_size)
    x1 = center_x + int(half_size) + 1
    y1 = center_y + int(half_size) + 1
    src_x0 = max(0, x0)
    src_y0 = max(0, y0)
    src_x1 = min(image.shape[1], x1)
    src_y1 = min(image.shape[0], y1)
    if src_x0 >= src_x1 or src_y0 >= src_y1:
        return output
    dst_x0 = src_x0 - x0
    dst_y0 = src_y0 - y0
    output[dst_y0 : dst_y0 + (src_y1 - src_y0), dst_x0 : dst_x0 + (src_x1 - src_x0)] = image[
        src_y0:src_y1, src_x0:src_x1
    ]
    return output


def score01(value, scale):
    if float(scale) <= 1e-6:
        return 0.0
    return float(np.clip(float(value) / float(scale), 0.0, 1.0))


def classify_modality_point(modality, point, patch_half_size=24):
    response_map = np.asarray(modality["map"], dtype=np.float32)
    valid_map = np.asarray(modality["valid"], dtype=bool)
    patch = crop_with_padding(response_map, point["pix"], patch_half_size, fill_value=0.0).astype(np.float32)
    valid_patch = crop_with_padding(valid_map.astype(np.uint8), point["pix"], patch_half_size, fill_value=0).astype(bool)
    valid_ratio = float(np.mean(valid_patch)) if valid_patch.size else 0.0
    if valid_ratio < 0.35:
        return {
            "idx": int(point["idx"]),
            "pix": point["pix"],
            "label": "uncertain",
            "score": 0.0,
            "quality": score01(valid_ratio, 0.35),
            "reason": "valid_ratio_too_low",
            "metrics": {"valid_ratio": valid_ratio},
        }

    center = disc_mask(patch.shape, 5) & valid_patch
    ring = ring_mask(patch.shape, 8, 20) & valid_patch
    if not np.any(ring):
        ring = valid_patch & ~center
    center_values = patch[center] if np.any(center) else np.asarray([], dtype=np.float32)
    ring_values = patch[ring] if np.any(ring) else np.asarray([], dtype=np.float32)
    center_mean = float(np.mean(center_values)) if center_values.size else 0.0
    ring_mean = float(np.mean(ring_values)) if ring_values.size else 0.0
    center_std = float(np.std(center_values)) if center_values.size else 0.0
    patch_std = float(np.std(patch[valid_patch])) if np.any(valid_patch) else 0.0
    center_delta = max(0.0, center_mean - ring_mean)

    patch_u8 = np.clip(patch * 255.0, 0, 255).astype(np.uint8)
    edges = cv2.Canny(patch_u8, 30, 95) > 0
    cross = np.zeros_like(valid_patch, dtype=bool)
    mid_y = cross.shape[0] // 2
    mid_x = cross.shape[1] // 2
    cross[mid_y, :] = True
    cross[:, mid_x] = True
    center_cross = cross & disc_mask(patch.shape, 8) & valid_patch
    outer_cross = cross & ~disc_mask(patch.shape, 8) & valid_patch
    outer_density = float(np.mean(edges[outer_cross])) if np.any(outer_cross) else 0.0
    center_density = float(np.mean(edges[center_cross])) if np.any(center_cross) else 0.0
    ridge_break_ratio = 0.0
    if outer_density > 1e-6:
        ridge_break_ratio = float(np.clip(1.0 - (center_density / outer_density), 0.0, 1.0))

    delta_score = score01(center_delta, 0.18)
    texture_score = score01(max(center_std, patch_std * 0.65), 0.16)
    peak_score = score01(center_mean, 0.58)
    ridge_score = score01(ridge_break_ratio, 0.35)
    score = float(
        np.clip(
            (0.42 * delta_score) + (0.27 * texture_score) + (0.21 * peak_score) + (0.10 * ridge_score),
            0.0,
            1.0,
        )
    )
    if score >= HIGH_SCORE_THRESHOLD:
        label = "bound"
        reason = "center_response_texture_or_ridge_break"
    elif score <= LOW_SCORE_THRESHOLD:
        label = "unbound"
        reason = "flat_clean_low_response"
    else:
        label = "uncertain"
        reason = "score_between_thresholds"

    return {
        "idx": int(point["idx"]),
        "pix": [float(point["pix"][0]), float(point["pix"][1])],
        "world": [float(value) for value in point.get("world", [0.0, 0.0, 0.0])[:3]],
        "tcp": [float(value) for value in point.get("tcp", [0.0, 0.0, 0.0])[:3]],
        "label": label,
        "score": score,
        "quality": score01(valid_ratio, 0.35),
        "reason": reason,
        "metrics": {
            "valid_ratio": valid_ratio,
            "center_mean": center_mean,
            "ring_mean": ring_mean,
            "center_delta": center_delta,
            "center_std": center_std,
            "patch_std": patch_std,
            "ridge_break_ratio": ridge_break_ratio,
            "delta_score": delta_score,
            "texture_score": texture_score,
            "peak_score": peak_score,
            "ridge_score": ridge_score,
        },
    }


def _white_box_config_for_report():
    return ClassificationConfig(
        mode="blocking",
        method="white_box",
        patch_half_size_px=32,
        height_band_mm=15.0,
        min_valid_depth_ratio=0.35,
        min_ir_stddev=5.0,
        center_radius_px=6,
        ring_inner_radius_px=8,
        ring_outer_radius_px=24,
        bound_height_delta_mm=4.0,
        bound_texture_stddev=16.0,
        ridge_break_ratio_threshold=0.35,
        pre_bind_bound_score_low=LOW_SCORE_THRESHOLD,
        pre_bind_bound_score_high=HIGH_SCORE_THRESHOLD,
    )


def classify_white_box_point(arrays, point, config):
    ir_image = arrays.get("Scepter_ir_image_raw")
    raw_world_image = arrays.get("Scepter_worldCoord_raw_world_coord")
    if raw_world_image is not None and getattr(raw_world_image, "ndim", 0) == 3 and raw_world_image.shape[2] >= 3:
        depth_image = raw_world_image[:, :, 2]
    else:
        depth_image = arrays.get("Scepter_depth_image_raw")
    bundle = extract_evidence_bundle(
        ir_image=ir_image,
        depth_image=depth_image,
        raw_world_image=raw_world_image,
        point_idx=int(point["idx"]),
        pix_coord=point["pix"],
        world_coord=point.get("world", [0.0, 0.0, 0.0]),
        phase="execution_refine",
        config=config,
    )
    decision = classify_pre_bind_white_box(bundle, config)
    metrics = dict(decision.metrics)
    metrics.setdefault("center_delta", float(metrics.get("white_box_off_axis_height_delta", 0.0)))
    metrics.setdefault("ridge_break_ratio", float(metrics.get("ridge_break_ratio", 0.0)))
    return {
        "idx": int(point["idx"]),
        "pix": [float(point["pix"][0]), float(point["pix"][1])],
        "world": [float(value) for value in point.get("world", [0.0, 0.0, 0.0])[:3]],
        "tcp": [float(value) for value in point.get("tcp", [0.0, 0.0, 0.0])[:3]],
        "label": decision.label,
        "score": float(decision.score),
        "quality": float(decision.quality),
        "reason": decision.reason,
        "metrics": metrics,
    }


def evaluate_white_box_rule(arrays, points, labels=None):
    config = _white_box_config_for_report()
    samples = [classify_white_box_point(arrays, point, config) for point in points]
    scores = np.asarray([float(sample["score"]) for sample in samples], dtype=np.float32)
    qualities = np.asarray([float(sample["quality"]) for sample in samples], dtype=np.float32)
    counts = label_counts(samples)
    labels_for_samples = [sample.get("label", "uncertain") for sample in samples]
    confident_mask = np.asarray(
        [label in {"bound", "unbound"} for label in labels_for_samples],
        dtype=bool,
    )
    if scores.size:
        decided_scores = scores[confident_mask] if np.any(confident_mask) else np.asarray([], dtype=np.float32)
        margin_score = (
            float(np.mean(np.abs(decided_scores - 0.5) * 2.0))
            if decided_scores.size
            else 0.0
        )
        spread_score = float(np.percentile(scores, 90.0) - np.percentile(scores, 10.0))
        mean_evidence = float(
            np.mean([sample["metrics"].get("white_box_off_axis_height_delta", 0.0) for sample in samples])
        )
    else:
        margin_score = 0.0
        spread_score = 0.0
        mean_evidence = 0.0
    confident_ratio = float(np.mean(confident_mask)) if confident_mask.size else 0.0
    quality_mean = float(np.mean(qualities)) if qualities.size else 0.0
    blob_mean = (
        float(np.mean([sample["metrics"].get("white_box_center_blob_score", 0.0) for sample in samples]))
        if samples
        else 0.0
    )
    proxy_score = float(
        np.clip(
            (0.30 * margin_score)
            + (0.25 * confident_ratio)
            + (0.18 * quality_mean)
            + (0.12 * np.clip(spread_score, 0.0, 1.0))
            + (0.08 * score01(mean_evidence, 0.18))
            + (0.07 * score01(blob_mean, 0.62)),
            0.0,
            1.0,
        )
    )
    result = {
        "id": "white_box_multimodal_rule",
        "label": "白盒融合算法",
        "note": "执行层分类同款：raw_world 高度、IR 暗线、CLAHE、梯度、斜丝残差和中心凸包紧致块联合判定。",
        "point_count": len(samples),
        "counts": counts,
        "mean_score": float(np.mean(scores)) if scores.size else 0.0,
        "median_score": float(np.median(scores)) if scores.size else 0.0,
        "score_spread_p90_p10": spread_score,
        "mean_quality": quality_mean,
        "confident_ratio": confident_ratio,
        "uncertain_ratio": float(counts.get("uncertain", 0) / float(len(samples))) if samples else 0.0,
        "mean_center_delta": mean_evidence,
        "mean_center_blob_score": blob_mean,
        "margin_score": margin_score,
        "proxy_score": proxy_score,
        "samples": samples,
    }
    if labels:
        result["label_metrics"] = compute_label_metrics(samples, labels)
    return result


def label_counts(samples):
    counts = {"bound": 0, "unbound": 0, "uncertain": 0}
    for sample in samples:
        label = sample.get("label", "uncertain")
        counts[label] = counts.get(label, 0) + 1
    return counts


def load_manual_labels(labels_json):
    if labels_json is None:
        return {}
    path = Path(labels_json)
    if not path.exists():
        raise RuntimeError(f"labels json does not exist: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, dict) and "labels" in payload:
        payload = payload["labels"]
    labels = {}
    if isinstance(payload, dict):
        for key, value in payload.items():
            label = str(value).strip().lower()
            if label in {"bound", "unbound"}:
                labels[int(key)] = label
    elif isinstance(payload, list):
        for item in payload:
            if not isinstance(item, dict):
                continue
            label = str(item.get("label", item.get("bind_state", ""))).strip().lower()
            if label in {"bound", "unbound"} and item.get("idx") is not None:
                labels[int(item["idx"])] = label
    return labels


def infer_overlay_labels_from_result_image(arrays, points):
    if "perception_lashing_result_image" not in arrays:
        return {}
    image = np.asarray(arrays["perception_lashing_result_image"], dtype=np.uint8)
    if image.ndim != 3 or image.shape[2] < 3:
        return {}
    inferred = {}
    for point in points:
        x = int(round(float(point["pix"][0])))
        y = int(round(float(point["pix"][1])))
        patch = image[max(0, y - 14) : min(image.shape[0], y + 15), max(0, x - 14) : min(image.shape[1], x + 15)]
        if patch.size == 0:
            continue
        b = patch[:, :, 0].astype(np.int16)
        g = patch[:, :, 1].astype(np.int16)
        r = patch[:, :, 2].astype(np.int16)
        red_count = int(np.count_nonzero((r > 155) & (g < 125) & (b < 125)))
        green_count = int(np.count_nonzero((g > 155) & (r < 145) & (b < 145)))
        if red_count >= 18 and red_count > green_count * 1.8:
            inferred[int(point["idx"])] = "bound"
        elif green_count >= 18 and green_count > red_count * 1.8:
            inferred[int(point["idx"])] = "unbound"

    counts = {"bound": 0, "unbound": 0}
    for label in inferred.values():
        counts[label] += 1
    if counts["bound"] >= 2 and counts["unbound"] >= 2:
        return inferred
    return {}


def compute_label_metrics(samples, labels):
    labeled = [sample for sample in samples if int(sample["idx"]) in labels]
    if not labeled:
        return {}
    total = len(labeled)
    decided = [sample for sample in labeled if sample["label"] in {"bound", "unbound"}]
    correct = sum(1 for sample in labeled if sample["label"] == labels[int(sample["idx"])])
    decided_correct = sum(1 for sample in decided if sample["label"] == labels[int(sample["idx"])])
    tp = sum(1 for sample in labeled if sample["label"] == "bound" and labels[int(sample["idx"])] == "bound")
    fp = sum(1 for sample in labeled if sample["label"] == "bound" and labels[int(sample["idx"])] == "unbound")
    fn = sum(1 for sample in labeled if sample["label"] != "bound" and labels[int(sample["idx"])] == "bound")
    precision = tp / float(tp + fp) if (tp + fp) else 0.0
    recall = tp / float(tp + fn) if (tp + fn) else 0.0
    f1 = (2.0 * precision * recall / (precision + recall)) if (precision + recall) else 0.0
    return {
        "labeled_count": total,
        "accuracy": correct / float(total),
        "decided_accuracy": decided_correct / float(len(decided)) if decided else 0.0,
        "decided_count": len(decided),
        "bound_precision": precision,
        "bound_recall": recall,
        "bound_f1": f1,
    }


def evaluate_modality(modality, points, labels=None):
    samples = [classify_modality_point(modality, point) for point in points]
    scores = np.asarray([float(sample["score"]) for sample in samples], dtype=np.float32)
    qualities = np.asarray([float(sample["quality"]) for sample in samples], dtype=np.float32)
    counts = label_counts(samples)
    labels_for_samples = [sample.get("label", "uncertain") for sample in samples]
    confident_mask = np.asarray(
        [label in {"bound", "unbound"} for label in labels_for_samples],
        dtype=bool,
    )
    if scores.size:
        decided_scores = scores[confident_mask] if np.any(confident_mask) else np.asarray([], dtype=np.float32)
        margin_score = (
            float(np.mean(np.abs(decided_scores - 0.5) * 2.0))
            if decided_scores.size
            else 0.0
        )
        spread_score = float(np.percentile(scores, 90.0) - np.percentile(scores, 10.0))
        mean_evidence = float(np.mean([sample["metrics"].get("center_delta", 0.0) for sample in samples]))
    else:
        margin_score = 0.0
        spread_score = 0.0
        mean_evidence = 0.0
    confident_ratio = float(np.mean(confident_mask)) if confident_mask.size else 0.0
    quality_mean = float(np.mean(qualities)) if qualities.size else 0.0
    proxy_score = float(
        np.clip(
            (0.34 * margin_score)
            + (0.28 * confident_ratio)
            + (0.18 * quality_mean)
            + (0.12 * np.clip(spread_score, 0.0, 1.0))
            + (0.08 * score01(mean_evidence, 0.18)),
            0.0,
            1.0,
        )
    )
    result = {
        "id": modality["id"],
        "label": modality["label"],
        "note": modality["note"],
        "point_count": len(samples),
        "counts": counts,
        "mean_score": float(np.mean(scores)) if scores.size else 0.0,
        "median_score": float(np.median(scores)) if scores.size else 0.0,
        "score_spread_p90_p10": spread_score,
        "mean_quality": quality_mean,
        "confident_ratio": confident_ratio,
        "uncertain_ratio": float(counts.get("uncertain", 0) / float(len(samples))) if samples else 0.0,
        "mean_center_delta": mean_evidence,
        "margin_score": margin_score,
        "proxy_score": proxy_score,
        "samples": samples,
    }
    if labels:
        result["label_metrics"] = compute_label_metrics(samples, labels)
    return result


def rank_results(results, labels=None):
    if labels:
        return sorted(
            results,
            key=lambda row: (
                row.get("label_metrics", {}).get("accuracy", 0.0),
                row.get("label_metrics", {}).get("bound_f1", 0.0),
                row["proxy_score"],
            ),
            reverse=True,
        )
    return sorted(results, key=lambda row: row["proxy_score"], reverse=True)


def empty_evaluation_result(modality):
    return {
        "id": modality["id"],
        "label": modality["label"],
        "note": modality["note"],
        "point_count": 0,
        "counts": {"bound": 0, "unbound": 0, "uncertain": 0},
        "mean_score": 0.0,
        "median_score": 0.0,
        "score_spread_p90_p10": 0.0,
        "mean_quality": 0.0,
        "confident_ratio": 0.0,
        "uncertain_ratio": 0.0,
        "mean_center_delta": 0.0,
        "mean_center_blob_score": 0.0,
        "margin_score": 0.0,
        "proxy_score": 0.0,
        "samples": [],
    }


def render_heatmap(response_map):
    image_u8 = np.clip(np.asarray(response_map, dtype=np.float32) * 255.0, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(image_u8, cv2.COLORMAP_TURBO)


def draw_workspace_polygon(image, corners):
    if corners is None:
        return image
    polygon = np.asarray(corners, dtype=np.int32).reshape((-1, 1, 2))
    overlay = image.copy()
    cv2.fillPoly(overlay, [polygon], (40, 145, 110))
    image = cv2.addWeighted(overlay, 0.16, image, 0.84, 0.0)
    cv2.polylines(image, [polygon], True, (80, 255, 190), 2, cv2.LINE_AA)
    return image


def draw_execution_mask_overlay(image, execution_artifacts):
    output = image.copy()
    tcp_mask = None if execution_artifacts is None else execution_artifacts.get("tcp_mask")
    binary = None if execution_artifacts is None else execution_artifacts.get("binary")
    if tcp_mask is not None and np.any(tcp_mask):
        overlay = output.copy()
        overlay[np.asarray(tcp_mask, dtype=bool)] = (46, 137, 255)
        output = cv2.addWeighted(overlay, 0.22, output, 0.78, 0.0)
        contours, _ = cv2.findContours(np.asarray(tcp_mask, dtype=np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(output, contours, -1, (70, 170, 255), 1, cv2.LINE_AA)
    if binary is not None and np.any(binary):
        overlay = output.copy()
        overlay[np.asarray(binary, dtype=np.uint8) > 0] = (70, 255, 225)
        output = cv2.addWeighted(overlay, 0.35, output, 0.65, 0.0)
    return output


def draw_points(image, samples, show_idx=False):
    output = image.copy()
    for sample in samples:
        x = int(round(float(sample["pix"][0])))
        y = int(round(float(sample["pix"][1])))
        label = sample.get("label", "uncertain")
        color = LABEL_META.get(label, LABEL_META["uncertain"])["color"]
        cv2.circle(output, (x, y), 7, (10, 12, 15), -1, cv2.LINE_AA)
        cv2.circle(output, (x, y), 5, color, -1, cv2.LINE_AA)
        cv2.circle(output, (x, y), 8, color, 1, cv2.LINE_AA)
        if show_idx:
            cv2.putText(
                output,
                str(sample["idx"]),
                (x + 7, y - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.34,
                (245, 245, 245),
                1,
                cv2.LINE_AA,
            )
    return output


def load_label_font(size=20):
    size = int(size)
    if size in _LABEL_FONT_CACHE:
        return _LABEL_FONT_CACHE[size]
    for font_path in CJK_FONT_CANDIDATES:
        path = Path(font_path)
        if not path.exists():
            continue
        try:
            font = ImageFont.truetype(str(path), size=size)
        except OSError:
            continue
        _LABEL_FONT_CACHE[size] = font
        return font
    font = ImageFont.load_default()
    _LABEL_FONT_CACHE[size] = font
    return font


def draw_label(image, label):
    output = image.copy()
    text = str(label)
    font = load_label_font(20)
    pil_image = Image.fromarray(cv2.cvtColor(output, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_image)
    text_box = draw.textbbox((0, 0), text, font=font)
    text_width = max(1, int(text_box[2] - text_box[0]))
    text_height = max(1, int(text_box[3] - text_box[1]))
    left, top = 10, 10
    padding_x, padding_y = 8, 6
    right = min(output.shape[1] - 1, left + text_width + padding_x * 2)
    bottom = min(output.shape[0] - 1, top + text_height + padding_y * 2)
    draw.rectangle((left, top, right, bottom), fill=(8, 12, 16))
    draw.text(
        (left + padding_x, top + padding_y - text_box[1]),
        text,
        font=font,
        fill=(245, 248, 246),
    )
    return cv2.cvtColor(np.asarray(pil_image), cv2.COLOR_RGB2BGR)


def render_overview(arrays, execution_artifacts, best_result):
    ir = np.asarray(arrays["Scepter_ir_image_raw"], dtype=np.uint8)
    if ir.ndim == 2:
        image = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
    else:
        image = ir[:, :, :3].copy()
    image = draw_execution_mask_overlay(image, execution_artifacts)
    image = draw_points(image, best_result["samples"], show_idx=True)
    return draw_label(image, f"best: {best_result['label']}")


def render_modality_image(modality, execution_artifacts, result):
    image = render_heatmap(modality["map"])
    image = draw_execution_mask_overlay(image, execution_artifacts)
    image = draw_points(image, result["samples"], show_idx=False)
    return draw_label(image, f"{result['label']} proxy={result['proxy_score']:.3f}")


def render_white_box_result_image(arrays, execution_artifacts, result):
    ir = np.asarray(arrays["Scepter_ir_image_raw"], dtype=np.uint8)
    if ir.ndim == 2:
        image = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
    else:
        image = ir[:, :, :3].copy()
    image = draw_execution_mask_overlay(image, execution_artifacts)
    image = draw_points(image, result["samples"], show_idx=True)
    return draw_label(image, f"白盒融合 proxy={result['proxy_score']:.3f}")


def pct(value):
    return f"{100.0 * float(value):.1f}%"


def write_images(output_dir, arrays, execution_artifacts, modalities, ranked_results):
    images_dir = Path(output_dir) / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    modality_by_id = {modality["id"]: modality for modality in modalities}
    best_result = ranked_results[0]
    if best_result["id"] == "white_box_multimodal_rule":
        best_overview = render_white_box_result_image(arrays, execution_artifacts, best_result)
    else:
        best_overview = render_overview(arrays, execution_artifacts, best_result)
    cv2.imwrite(str(images_dir / "00_best_overview.png"), best_overview)
    image_map = {"best_overview": "images/00_best_overview.png"}
    for rank, result in enumerate(ranked_results, start=1):
        filename = f"{rank:02d}_{result['id']}.png"
        if result["id"] == "white_box_multimodal_rule":
            rendered = render_white_box_result_image(arrays, execution_artifacts, result)
        else:
            modality = modality_by_id[result["id"]]
            rendered = render_modality_image(modality, execution_artifacts, result)
        cv2.imwrite(str(images_dir / filename), rendered)
        result["image"] = f"images/{filename}"
        image_map[result["id"]] = result["image"]
    return image_map


def format_metric(result, key):
    value = result.get("label_metrics", {}).get(key)
    if value is None:
        return "无标注"
    return pct(value)


def write_html(output_dir, summary, ranked_results):
    best = ranked_results[0]
    generated_at = html.escape(summary["generated_at"])
    evaluation_mode_title = "真实准确率" if summary["evaluation_mode"] != "unlabeled_proxy" else "无标注代理评分"
    diagnostics = summary.get("hough_diagnostics", {})
    roi_note = summary.get("execution_roi", {}).get("note", EXECUTION_ROI_NOTE)
    notice = (
        "当前没有可用的红 / 绿人工标注或可置信覆盖图标注，因此不能计算真实准确率。"
        "本页的最佳模态来自无标注代理评分：高置信比例、中心 / 环形响应差、局部纹理、脊线破坏和证据质量。"
        if summary["evaluation_mode"] == "unlabeled_proxy"
        else "当前检测到红 / 绿标注，可计算真实准确率；不确定样本会在总准确率中按未命中处理。"
    )
    if summary.get("point_count", 0) <= 0:
        notice = (
            "当前快照按 TCP 工具坐标执行盒做 Hough 后没有形成可分类点。"
            "这通常表示当前视觉快照、相机-TCP 外参或线性模组所在区域没有对上；本页只保留底图和门控诊断，不给出有效分类精度结论。"
        )
    rows = []
    for rank, result in enumerate(ranked_results, start=1):
        counts = result["counts"]
        rows.append(
            f"""
            <tr>
              <td>{rank}</td>
              <td><strong>{html.escape(result['label'])}</strong><br><span>{html.escape(result['id'])}</span></td>
              <td>{result['proxy_score']:.3f}</td>
              <td>{format_metric(result, 'accuracy')}</td>
              <td>{pct(result['confident_ratio'])}</td>
              <td><span class="pill red">{counts.get('bound', 0)}</span></td>
              <td><span class="pill green">{counts.get('unbound', 0)}</span></td>
              <td><span class="pill amber">{counts.get('uncertain', 0)}</span></td>
              <td>{result['mean_center_delta']:.3f}</td>
              <td>{html.escape(result['note'])}</td>
            </tr>
            """
        )

    cards = []
    for result in ranked_results:
        cards.append(
            f"""
            <article class="modality-card">
              <div class="card-head">
                <strong>{html.escape(result['label'])}</strong>
                <span>{result['proxy_score']:.3f}</span>
              </div>
              <img src="{html.escape(result['image'])}" alt="{html.escape(result['label'])}">
              <p>{html.escape(result['note'])}</p>
            </article>
            """
        )

    point_rows = []
    for sample in best["samples"][:120]:
        meta = LABEL_META.get(sample["label"], LABEL_META["uncertain"])
        tcp = sample.get("tcp", [0.0, 0.0, 0.0])
        point_rows.append(
            f"""
            <tr>
              <td>{sample['idx']}</td>
              <td><span class="pill {meta['tone']}">{meta['title']}</span></td>
              <td>{sample['score']:.3f}</td>
              <td>{sample['quality']:.3f}</td>
              <td>({tcp[0]:.0f}, {tcp[1]:.0f}, {tcp[2]:.0f})</td>
              <td>{sample['metrics'].get('center_delta', 0.0):.3f}</td>
              <td>{sample['metrics'].get('white_box_center_blob_score', sample['metrics'].get('center_blob_score', 0.0)):.3f}</td>
              <td>{sample['metrics'].get('ridge_break_ratio', 0.0):.3f}</td>
              <td>{html.escape(sample['reason'])}</td>
            </tr>
            """
        )

    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>绑扎点分类模态扫图报告</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #101316;
      --panel: #181c20;
      --panel-2: #20262b;
      --line: #33404a;
      --text: #edf3f2;
      --muted: #a7b1b7;
      --red: #ff5a5f;
      --green: #6dd77a;
      --amber: #f2c14e;
      --blue: #78c7ff;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: #101316;
      color: var(--text);
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", "Segoe UI", sans-serif;
    }}
    main {{ max-width: 1440px; margin: 0 auto; padding: 28px; }}
    header {{ display: flex; justify-content: space-between; gap: 24px; align-items: flex-end; border-bottom: 1px solid var(--line); padding-bottom: 18px; }}
    h1 {{ margin: 0; font-size: 30px; letter-spacing: 0; }}
    h2 {{ margin: 30px 0 14px; font-size: 18px; letter-spacing: 0; }}
    p {{ color: var(--muted); line-height: 1.62; }}
    code {{ color: var(--blue); }}
    .stamp {{ color: var(--muted); text-align: right; font-size: 13px; }}
    .stats {{ display: grid; grid-template-columns: repeat(5, minmax(160px, 1fr)); gap: 12px; margin-top: 20px; }}
    .stat {{ background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 16px; }}
    .stat span {{ display: block; color: var(--muted); font-size: 12px; }}
    .stat strong {{ display: block; margin-top: 8px; font-size: 27px; }}
    .notice {{ border: 1px solid #705a24; background: #211d12; color: #f6da87; border-radius: 8px; padding: 14px 16px; }}
    .hero-img {{ width: 100%; border: 1px solid var(--line); border-radius: 8px; background: #06090b; }}
    table {{ width: 100%; border-collapse: collapse; background: var(--panel); border: 1px solid var(--line); border-radius: 8px; overflow: hidden; }}
    th, td {{ text-align: left; padding: 10px 12px; border-bottom: 1px solid var(--line); font-size: 13px; vertical-align: top; }}
    th {{ color: var(--muted); background: var(--panel-2); font-weight: 600; }}
    td span {{ color: var(--muted); font-size: 12px; }}
    .pill {{ display: inline-flex; min-width: 34px; justify-content: center; padding: 3px 8px; border-radius: 999px; font-weight: 700; font-size: 12px; }}
    .pill.red {{ color: #ffecee; background: rgba(255, 90, 95, .18); border: 1px solid rgba(255, 90, 95, .55); }}
    .pill.green {{ color: #ecfff0; background: rgba(109, 215, 122, .18); border: 1px solid rgba(109, 215, 122, .55); }}
    .pill.amber {{ color: #fff5d3; background: rgba(242, 193, 78, .18); border: 1px solid rgba(242, 193, 78, .55); }}
    .grid {{ display: grid; grid-template-columns: repeat(3, minmax(260px, 1fr)); gap: 14px; }}
    .modality-card {{ background: var(--panel); border: 1px solid var(--line); border-radius: 8px; overflow: hidden; }}
    .card-head {{ display: flex; justify-content: space-between; gap: 12px; padding: 12px 14px; background: var(--panel-2); border-bottom: 1px solid var(--line); }}
    .card-head span {{ color: var(--blue); font-weight: 700; }}
    .modality-card img {{ display: block; width: 100%; background: #050607; }}
    .modality-card p {{ margin: 0; padding: 12px 14px; min-height: 72px; }}
    @media (max-width: 980px) {{
      header {{ display: block; }}
      .stamp {{ text-align: left; margin-top: 12px; }}
      .stats, .grid {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>绑扎点分类模态扫图报告</h1>
      <p>固定当前区域快照，只读复刻执行微调的 TCP ROI + Hough 找点，再测试所有可用模态底图；不写入真实点位、扫描账本或分类事件日志。</p>
    </div>
    <div class="stamp">生成时间<br>{generated_at}</div>
  </header>

  <section class="stats">
    <div class="stat"><span>最佳模态</span><strong>{html.escape(best['label'])}</strong></div>
    <div class="stat"><span>{evaluation_mode_title}</span><strong>{best['proxy_score']:.3f}</strong></div>
    <div class="stat"><span>测试点数</span><strong>{summary['point_count']}</strong></div>
    <div class="stat"><span>TCP mask 像素</span><strong>{diagnostics.get('tcp_range_mask_pixels', 0)}</strong></div>
    <div class="stat"><span>Hough 交点</span><strong>{diagnostics.get('intersections', 0)}</strong></div>
  </section>

  <h2>当前结论</h2>
  <p class="notice">{html.escape(notice)}</p>
  <p class="notice">{html.escape(roi_note)}</p>
  <img class="hero-img" src="images/00_best_overview.png" alt="最佳模态分类覆盖图">

  <h2>模态排名</h2>
  <table>
    <thead>
      <tr><th>#</th><th>模态</th><th>无标注代理评分</th><th>真实准确率</th><th>高置信</th><th>红</th><th>绿</th><th>不确定</th><th>中心差</th><th>说明</th></tr>
    </thead>
    <tbody>{''.join(rows)}</tbody>
  </table>

  <h2>所有模态覆盖图</h2>
  <section class="grid">{''.join(cards)}</section>

  <h2>最佳模态点级结果</h2>
  <table>
    <thead><tr><th>点位</th><th>分类</th><th>score</th><th>quality</th><th>TCP(mm)</th><th>中心差</th><th>中心块</th><th>脊线破坏</th><th>reason</th></tr></thead>
    <tbody>{''.join(point_rows)}</tbody>
  </table>
</main>
</body>
</html>
"""
    (Path(output_dir) / "index.html").write_text(html_text, encoding="utf-8")


def build_report(
    output_dir=DEFAULT_REPORT_DIR,
    snapshot_dir=DEFAULT_SNAPSHOT_DIR,
    labels_json=None,
    max_points=None,
):
    before_state = snapshot_read_only_inputs()
    output_dir = Path(output_dir)
    ensure_clean_dir(output_dir)

    arrays, arrays_dir = load_arrays(snapshot_dir)
    points, hough_diagnostics, execution_artifacts = build_execution_refine_candidate_points(
        arrays,
        max_points=max_points,
    )
    tcp_mask = execution_artifacts.get("tcp_mask")
    workspace_mask = (
        np.asarray(tcp_mask, dtype=bool)
        if tcp_mask is not None
        else np.ones(arrays["Scepter_ir_image_raw"].shape[:2], dtype=bool)
    )
    modalities = build_modalities(arrays, workspace_mask)

    labels = load_manual_labels(labels_json)
    label_source = "manual_labels_json" if labels else None
    if not labels:
        labels = infer_overlay_labels_from_result_image(arrays, points)
        label_source = "overlay_red_green" if labels else None
    evaluation_mode = "labeled" if labels else "unlabeled_proxy"

    results = (
        [evaluate_modality(modality, points, labels=labels) for modality in modalities]
        if points
        else [empty_evaluation_result(modality) for modality in modalities]
    )
    results.append(evaluate_white_box_rule(arrays, points, labels=labels))
    ranked_results = rank_results(results, labels=labels)
    image_map = write_images(output_dir, arrays, execution_artifacts, modalities, ranked_results)

    public_results = []
    for result in ranked_results:
        copied = dict(result)
        copied["samples"] = [json_safe(sample) for sample in result["samples"]]
        public_results.append(json_safe(copied))
    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "evaluation_mode": evaluation_mode,
        "label_source": label_source,
        "point_source": EXECUTION_POINT_SOURCE,
        "snapshot_dir": str(Path(snapshot_dir)),
        "arrays_dir": str(arrays_dir),
        "point_count": len(points),
        "modality_count": len(ranked_results),
        "execution_roi": {
            "roi_frame": "gripper_frame_tcp_tool_workspace",
            "bounds_mm": dict(EXECUTION_TCP_ROI_BOUNDS),
            "tcp_config_path": str(DEFAULT_TCP_DISPLAY_CONFIG_PATH),
            "note": EXECUTION_ROI_NOTE,
        },
        "hough_diagnostics": json_safe(hough_diagnostics),
        "best_modality": {
            key: value
            for key, value in ranked_results[0].items()
            if key not in {"samples"}
        },
        "modalities": public_results,
        "images": image_map,
        "read_only_guard_paths": [str(path) for path in READ_ONLY_GUARD_PATHS],
        "report_url_path": "/reports/bind_point_classification_modalities_current/index.html",
        "note": (
            "无标注代理评分不是红 / 绿框真实准确率；点源来自执行层 TCP ROI + Hough 交点，"
            "不再使用 perception_lashing_points_camera.txt 扫描候选点。perception_lashing_result_image 是渲染覆盖图，"
            "为避免把输出颜色泄漏进分类输入，不作为候选模态参与排名。拿到人工标注 JSON 后可用 --labels-json 复跑。"
            if evaluation_mode == "unlabeled_proxy"
            else "已使用红 / 绿标签计算真实准确率。"
        ),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(json_safe(summary), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    write_html(output_dir, summary, ranked_results)
    assert_read_only_inputs_unchanged(before_state)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-dir", default=str(DEFAULT_SNAPSHOT_DIR))
    parser.add_argument("--labels-json", default=None)
    parser.add_argument("--output-dir", default=str(DEFAULT_REPORT_DIR))
    parser.add_argument("--max-points", type=int, default=None)
    args = parser.parse_args()

    summary = build_report(
        output_dir=Path(args.output_dir),
        snapshot_dir=Path(args.snapshot_dir),
        labels_json=Path(args.labels_json) if args.labels_json else None,
        max_points=args.max_points,
    )
    print(Path(args.output_dir) / "index.html")
    print(json.dumps({"best_modality": summary["best_modality"]["id"], "mode": summary["evaluation_mode"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
