"""Rule-based bound/unbound classification for lashing points.

The module is deliberately ROS-free so the visual evidence and scoring logic
can be unit-tested with synthetic and replayed camera frames.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import cv2
import numpy as np
import yaml


CLASSIFICATION_MODES = {"off", "shadow", "advisory", "blocking"}
CLASSIFICATION_METHODS = {"deep_learning", "white_box"}


@dataclass
class ClassificationConfig:
    mode: str = "off"
    method: str = "deep_learning"
    event_log_path: Path = Path("src/tie_robot_perception/data/bind_classification_events.jsonl")
    evidence_root: Path = Path("src/tie_robot_perception/data/bind_evidence")
    deep_learning_model_path: Path = Path("/home/hyq-/simple_lashingrobot_ws/best.pt")
    deep_learning_confidence: float = 0.2
    deep_learning_depth_tolerance_mm: float = 75.0
    deep_learning_bound_class_id: int = 0
    deep_learning_unbound_class_id: int = 1
    patch_half_size_px: int = 32
    height_band_mm: float = 15.0
    min_valid_depth_ratio: float = 0.35
    min_ir_stddev: float = 5.0
    center_radius_px: int = 6
    ring_inner_radius_px: int = 8
    ring_outer_radius_px: int = 24
    bound_height_delta_mm: float = 4.0
    bound_texture_stddev: float = 16.0
    ridge_break_ratio_threshold: float = 0.35
    pre_bind_bound_score_low: float = 0.42
    pre_bind_bound_score_high: float = 0.68
    post_bind_success_score_low: float = 0.38
    post_bind_success_score_high: float = 0.62

    def __post_init__(self):
        self.mode = normalize_classification_mode(self.mode)
        self.method = normalize_classification_method(self.method)
        self.event_log_path = Path(self.event_log_path)
        self.evidence_root = Path(self.evidence_root)
        self.deep_learning_model_path = Path(self.deep_learning_model_path)
        self.patch_half_size_px = max(2, int(self.patch_half_size_px))
        self.center_radius_px = max(1, int(self.center_radius_px))
        self.ring_inner_radius_px = max(self.center_radius_px + 1, int(self.ring_inner_radius_px))
        self.ring_outer_radius_px = max(self.ring_inner_radius_px + 1, int(self.ring_outer_radius_px))


@dataclass
class EvidenceBundle:
    point_idx: int
    phase: str
    pix_coord: Tuple[int, int]
    world_coord: Tuple[float, float, float]
    ir_patch: np.ndarray
    depth_patch: np.ndarray
    raw_world_patch: np.ndarray
    height_patch: np.ndarray
    valid_depth_mask: np.ndarray
    ridge_patch: np.ndarray
    evidence_quality: float
    metrics: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class RuleDecision:
    label: str
    score: float
    quality: float
    reason: str
    metrics: Dict[str, float]


def normalize_classification_mode(mode):
    normalized = str(mode or "off").strip().lower()
    return normalized if normalized in CLASSIFICATION_MODES else "off"


def normalize_classification_method(method):
    normalized = str(method or "deep_learning").strip().lower()
    return normalized if normalized in CLASSIFICATION_METHODS else "deep_learning"


def _as_path(value, fallback):
    if value is None or str(value).strip() == "":
        return Path(fallback)
    return Path(value)


def load_classification_config(config_path=None, overrides=None):
    data = {}
    if config_path:
        path = Path(config_path)
        if path.exists():
            loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            if isinstance(loaded, dict):
                data.update(loaded)
    if overrides:
        data.update({key: value for key, value in dict(overrides).items() if value is not None})

    return ClassificationConfig(
        mode=data.get("mode", "off"),
        method=data.get("method", "deep_learning"),
        event_log_path=_as_path(
            data.get("event_log_path"),
            "src/tie_robot_perception/data/bind_classification_events.jsonl",
        ),
        evidence_root=_as_path(
            data.get("evidence_root"),
            "src/tie_robot_perception/data/bind_evidence",
        ),
        deep_learning_model_path=_as_path(
            data.get("deep_learning_model_path"),
            "/home/hyq-/simple_lashingrobot_ws/best.pt",
        ),
        deep_learning_confidence=float(data.get("deep_learning_confidence", 0.2)),
        deep_learning_depth_tolerance_mm=float(data.get("deep_learning_depth_tolerance_mm", 75.0)),
        deep_learning_bound_class_id=int(data.get("deep_learning_bound_class_id", 0)),
        deep_learning_unbound_class_id=int(data.get("deep_learning_unbound_class_id", 1)),
        patch_half_size_px=int(data.get("patch_half_size_px", 32)),
        height_band_mm=float(data.get("height_band_mm", 15.0)),
        min_valid_depth_ratio=float(data.get("min_valid_depth_ratio", 0.35)),
        min_ir_stddev=float(data.get("min_ir_stddev", 5.0)),
        center_radius_px=int(data.get("center_radius_px", 6)),
        ring_inner_radius_px=int(data.get("ring_inner_radius_px", 8)),
        ring_outer_radius_px=int(data.get("ring_outer_radius_px", 24)),
        bound_height_delta_mm=float(data.get("bound_height_delta_mm", 4.0)),
        bound_texture_stddev=float(data.get("bound_texture_stddev", 16.0)),
        ridge_break_ratio_threshold=float(data.get("ridge_break_ratio_threshold", 0.35)),
        pre_bind_bound_score_low=float(data.get("pre_bind_bound_score_low", 0.42)),
        pre_bind_bound_score_high=float(data.get("pre_bind_bound_score_high", 0.68)),
        post_bind_success_score_low=float(data.get("post_bind_success_score_low", 0.38)),
        post_bind_success_score_high=float(data.get("post_bind_success_score_high", 0.62)),
    )


def _safe_array(image, dtype=None):
    if image is None:
        return None
    array = np.asarray(image)
    return array.astype(dtype, copy=False) if dtype is not None else array


def _crop_with_padding(image, center_xy, half_size, fill_value=0):
    array = _safe_array(image)
    size = (int(half_size) * 2) + 1
    if array is None or array.size == 0:
        return np.full((size, size), fill_value, dtype=np.float32)

    center_x = int(round(float(center_xy[0])))
    center_y = int(round(float(center_xy[1])))
    y0 = center_y - int(half_size)
    y1 = center_y + int(half_size) + 1
    x0 = center_x - int(half_size)
    x1 = center_x + int(half_size) + 1

    if array.ndim == 2:
        output_shape = (size, size)
    else:
        output_shape = (size, size, array.shape[2])
    output = np.full(output_shape, fill_value, dtype=array.dtype)

    src_y0 = max(0, y0)
    src_y1 = min(array.shape[0], y1)
    src_x0 = max(0, x0)
    src_x1 = min(array.shape[1], x1)
    if src_y0 >= src_y1 or src_x0 >= src_x1:
        return output

    dst_y0 = src_y0 - y0
    dst_y1 = dst_y0 + (src_y1 - src_y0)
    dst_x0 = src_x0 - x0
    dst_x1 = dst_x0 + (src_x1 - src_x0)
    output[dst_y0:dst_y1, dst_x0:dst_x1] = array[src_y0:src_y1, src_x0:src_x1]
    return output


def _disc_mask(shape, radius, center=None):
    height, width = shape[:2]
    if center is None:
        center = ((width - 1) / 2.0, (height - 1) / 2.0)
    yy, xx = np.indices((height, width), dtype=np.float32)
    return ((xx - float(center[0])) ** 2 + (yy - float(center[1])) ** 2) <= float(radius) ** 2


def _ring_mask(shape, inner_radius, outer_radius):
    outer = _disc_mask(shape, outer_radius)
    inner = _disc_mask(shape, inner_radius)
    return outer & ~inner


def _finite_values(values):
    array = np.asarray(values, dtype=np.float32)
    return array[np.isfinite(array)]


def _mean_or(values, fallback=0.0):
    finite = _finite_values(values)
    return float(np.mean(finite)) if finite.size else float(fallback)


def _std_or(values, fallback=0.0):
    finite = _finite_values(values)
    return float(np.std(finite)) if finite.size else float(fallback)


def _score01(value, full_scale):
    full_scale = float(full_scale)
    if full_scale <= 1e-6:
        return 0.0
    return float(np.clip(float(value) / full_scale, 0.0, 1.0))


def _build_height_patch(depth_patch, valid_mask, config):
    depth = np.asarray(depth_patch, dtype=np.float32)
    valid = np.asarray(valid_mask, dtype=bool) & np.isfinite(depth) & (depth > 0.0)
    if not np.any(valid):
        return np.zeros_like(depth, dtype=np.float32), 0.0

    ring = _ring_mask(depth.shape, config.ring_inner_radius_px, config.ring_outer_radius_px) & valid
    if not np.any(ring):
        ring = valid
    local_reference_depth = float(np.median(depth[ring]))
    height = np.where(valid, np.clip(local_reference_depth - depth, 0.0, config.height_band_mm), 0.0)
    return height.astype(np.float32), local_reference_depth


def _local_height_response(depth_patch, valid_mask, sigma=11.0):
    depth = np.asarray(depth_patch, dtype=np.float32)
    valid = np.asarray(valid_mask, dtype=bool) & np.isfinite(depth) & (depth > 0.0)
    if not np.any(valid):
        return np.zeros(depth.shape[:2], dtype=np.float32)

    filled = depth.copy()
    filled[~valid] = float(np.median(depth[valid]))
    background = cv2.GaussianBlur(filled, (0, 0), sigmaX=float(sigma), sigmaY=float(sigma))
    response = background - filled
    response[~valid] = 0.0
    valid_response = response[valid]
    lower = float(np.percentile(valid_response, 2.0))
    upper = float(np.percentile(valid_response, 98.0))
    if upper <= lower + 1e-6:
        lower = float(np.min(valid_response))
        upper = float(np.max(valid_response))
    output = np.zeros(depth.shape[:2], dtype=np.float32)
    if upper > lower + 1e-6:
        output[valid] = np.clip((response[valid] - lower) / (upper - lower), 0.0, 1.0)
    elif upper > 0.0:
        output[valid] = 1.0
    return output.astype(np.float32)


def _local_height_mm_response(depth_patch, valid_mask, config):
    depth = np.asarray(depth_patch, dtype=np.float32)
    valid = np.asarray(valid_mask, dtype=bool) & np.isfinite(depth) & (depth > 0.0)
    if not np.any(valid):
        return np.zeros(depth.shape[:2], dtype=np.float32)

    ring = _ring_mask(depth.shape, config.ring_inner_radius_px, config.ring_outer_radius_px) & valid
    if not np.any(ring):
        ring = valid
    local_reference_depth = float(np.median(depth[ring]))
    return np.where(
        valid,
        np.clip(local_reference_depth - depth, 0.0, float(config.height_band_mm)),
        0.0,
    ).astype(np.float32)


def _normalize_response(image, valid_mask=None, low=2.0, high=98.0):
    image = np.asarray(image, dtype=np.float32)
    output = np.zeros(image.shape[:2], dtype=np.float32)
    if image.ndim != 2 or image.size == 0:
        return output
    valid = np.isfinite(image)
    if valid_mask is not None:
        mask = np.asarray(valid_mask, dtype=bool)
        if mask.shape == image.shape:
            valid &= mask
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


def _local_dark_response(gray_patch, valid_mask, sigma=7.0):
    gray = np.asarray(gray_patch, dtype=np.float32)
    valid = np.asarray(valid_mask, dtype=bool) & np.isfinite(gray)
    if not np.any(valid):
        return np.zeros(gray.shape[:2], dtype=np.float32)
    filled = gray.copy()
    filled[~valid] = float(np.median(gray[valid]))
    background = cv2.GaussianBlur(filled, (0, 0), sigmaX=float(sigma), sigmaY=float(sigma))
    response = background - filled
    response[~valid] = 0.0
    return _normalize_response(response, valid)


def _gradient_response(image, valid_mask):
    image = np.asarray(image, dtype=np.float32)
    valid = np.asarray(valid_mask, dtype=bool) & np.isfinite(image)
    if not np.any(valid):
        return np.zeros(image.shape[:2], dtype=np.float32)
    filled = image.copy()
    filled[~valid] = float(np.median(image[valid]))
    smoothed = cv2.GaussianBlur(filled, (0, 0), sigmaX=1.2, sigmaY=1.2)
    grad_x = cv2.Sobel(smoothed, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(smoothed, cv2.CV_32F, 0, 1, ksize=3)
    return _normalize_response(cv2.magnitude(grad_x, grad_y), valid)


def _mean_mask(image, mask, fallback=0.0):
    mask = np.asarray(mask, dtype=bool)
    if not np.any(mask):
        return float(fallback)
    return _mean_or(np.asarray(image)[mask], fallback)


def _std_mask(image, mask, fallback=0.0):
    mask = np.asarray(mask, dtype=bool)
    if not np.any(mask):
        return float(fallback)
    return _std_or(np.asarray(image)[mask], fallback)


def _profile_axis_center(response, valid, axis, center_index, search_radius, band_half_width):
    response = np.asarray(response, dtype=np.float32)
    valid = np.asarray(valid, dtype=bool)
    if response.ndim != 2 or not np.any(valid):
        return int(center_index)
    length = response.shape[0 if axis == "row" else 1]
    start = max(0, int(center_index) - int(search_radius))
    stop = min(length - 1, int(center_index) + int(search_radius))
    best_index = int(center_index)
    best_score = -1.0
    for index in range(start, stop + 1):
        if axis == "row":
            band = np.zeros(response.shape, dtype=bool)
            band[max(0, index - band_half_width) : min(response.shape[0], index + band_half_width + 1), :] = True
        else:
            band = np.zeros(response.shape, dtype=bool)
            band[:, max(0, index - band_half_width) : min(response.shape[1], index + band_half_width + 1)] = True
        mask = band & valid
        if not np.any(mask):
            continue
        score = float(np.mean(response[mask]))
        if score > best_score:
            best_score = score
            best_index = index
    return int(best_index)


def _axis_profile_half_width(response, valid, axis, center_index, default_width):
    response = np.asarray(response, dtype=np.float32)
    valid = np.asarray(valid, dtype=bool)
    if response.ndim != 2 or not np.any(valid):
        return int(default_width)
    if axis == "row":
        profile = np.asarray(
            [
                _mean_mask(response, valid & (np.indices(response.shape)[0] == row), 0.0)
                for row in range(response.shape[0])
            ],
            dtype=np.float32,
        )
    else:
        profile = np.asarray(
            [
                _mean_mask(response, valid & (np.indices(response.shape)[1] == col), 0.0)
                for col in range(response.shape[1])
            ],
            dtype=np.float32,
        )
    peak = float(profile[int(center_index)]) if 0 <= int(center_index) < profile.size else 0.0
    if peak <= 1e-6:
        return int(default_width)
    threshold = max(peak * 0.42, float(np.percentile(profile, 70.0)))
    left = int(center_index)
    while left > 0 and profile[left - 1] >= threshold:
        left -= 1
    right = int(center_index)
    while right + 1 < profile.size and profile[right + 1] >= threshold:
        right += 1
    return int(np.clip(((right - left + 1) // 2) + 1, default_width, max(default_width, 8)))


def _axis_angle_diff_deg(x1, y1, x2, y2):
    angle = abs(np.degrees(np.arctan2(float(y2) - float(y1), float(x2) - float(x1)))) % 180.0
    return min(angle, abs(90.0 - angle), abs(180.0 - angle))


def _diagonal_residual_score(residual, valid, axis_mask, center_mask):
    residual = np.asarray(residual, dtype=np.float32)
    valid = np.asarray(valid, dtype=bool)
    local_mask = valid & np.asarray(center_mask, dtype=bool) & ~np.asarray(axis_mask, dtype=bool)
    if not np.any(local_mask):
        return 0.0, 0, 0.0
    local_values = residual[local_mask]
    if float(np.percentile(local_values, 95.0) - np.percentile(local_values, 5.0)) < 0.06:
        return 0.0, 0, 0.0
    threshold = max(0.28, float(np.percentile(local_values, 82.0)))
    binary = ((residual >= threshold) & local_mask).astype(np.uint8) * 255
    if int(np.count_nonzero(binary)) < 3:
        return 0.0, 0, 0.0
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((2, 2), dtype=np.uint8))

    min_dim = max(1, min(residual.shape[:2]))
    diagonal_count = 0
    diagonal_length = 0.0
    lines = cv2.HoughLinesP(
        binary,
        rho=1,
        theta=np.pi / 180,
        threshold=max(4, int(round(min_dim * 0.08))),
        minLineLength=max(5, int(round(min_dim * 0.14))),
        maxLineGap=max(2, int(round(min_dim * 0.05))),
    )
    if lines is not None:
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = [float(value) for value in line]
            length = math.hypot(x2 - x1, y2 - y1)
            if length <= 0.0:
                continue
            if _axis_angle_diff_deg(x1, y1, x2, y2) > 18.0 and length <= min_dim * 0.85:
                diagonal_count += 1
                diagonal_length += length

    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    for component_idx in range(1, component_count):
        area = int(stats[component_idx, cv2.CC_STAT_AREA])
        if area < max(3, int(round(min_dim * 0.04))):
            continue
        ys, xs = np.where(labels == component_idx)
        if xs.size < 3:
            continue
        coords = np.column_stack([xs.astype(np.float32), ys.astype(np.float32)])
        coords -= np.mean(coords, axis=0, keepdims=True)
        covariance = np.cov(coords, rowvar=False)
        if not np.all(np.isfinite(covariance)):
            continue
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)
        major_value = float(max(eigenvalues[order[-1]], 0.0))
        minor_value = float(max(eigenvalues[order[0]], 0.0))
        length = 4.0 * math.sqrt(major_value)
        if length < max(4.0, min_dim * 0.10):
            continue
        major_vector = eigenvectors[:, order[-1]]
        angle_diff = _axis_angle_diff_deg(0.0, 0.0, float(major_vector[0]), float(major_vector[1]))
        elongation = math.sqrt((major_value + 1e-6) / (minor_value + 1e-6))
        if angle_diff > 18.0 and elongation >= 1.8:
            diagonal_count += 1
            diagonal_length += length * min(1.0, elongation / 4.0)

    diagonal_score = float(np.clip(diagonal_length / max(min_dim * 0.42, 1.0), 0.0, 1.0))
    area_ratio = float(np.mean(binary[local_mask] > 0)) if np.any(local_mask) else 0.0
    return diagonal_score, int(diagonal_count), area_ratio


def _off_axis_diagonal_score(response, valid, axis_mask, search_mask):
    """Find short diagonal evidence that remains after the rebar axes are masked.

    A clean unbound crossing is mostly explained by the two near-axis bars. A
    tied crossing usually leaves short off-axis wire or knot fragments. The
    mask intentionally spans center and near ring pixels instead of only the
    tiny center disc, because the wire often crosses just outside the exact
    Hough intersection.
    """
    response = np.asarray(response, dtype=np.float32)
    valid = np.asarray(valid, dtype=bool)
    axis_mask = np.asarray(axis_mask, dtype=bool)
    axis_u8 = (axis_mask.astype(np.uint8) * 255)
    if np.any(axis_u8):
        axis_u8 = cv2.dilate(axis_u8, np.ones((3, 3), dtype=np.uint8), iterations=1)
    local_mask = valid & np.asarray(search_mask, dtype=bool) & ~(axis_u8 > 0)
    if not np.any(local_mask):
        return 0.0, 0, 0.0

    values = response[local_mask]
    if float(np.percentile(values, 95.0) - np.percentile(values, 5.0)) < 0.08:
        return 0.0, 0, 0.0
    threshold = max(0.24, float(np.percentile(values, 78.0)))
    binary = ((response >= threshold) & local_mask).astype(np.uint8) * 255
    if int(np.count_nonzero(binary)) < 3:
        return 0.0, 0, 0.0
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((2, 2), dtype=np.uint8))

    min_dim = max(1, min(response.shape[:2]))
    diagonal_count = 0
    diagonal_length = 0.0
    lines = cv2.HoughLinesP(
        binary,
        rho=1,
        theta=np.pi / 180,
        threshold=max(4, int(round(min_dim * 0.08))),
        minLineLength=max(5, int(round(min_dim * 0.14))),
        maxLineGap=max(2, int(round(min_dim * 0.05))),
    )
    if lines is not None:
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = [float(value) for value in line]
            length = math.hypot(x2 - x1, y2 - y1)
            if length <= 0.0:
                continue
            if _axis_angle_diff_deg(x1, y1, x2, y2) > 18.0 and length <= min_dim * 0.90:
                diagonal_count += 1
                diagonal_length += length

    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    for component_idx in range(1, component_count):
        area = int(stats[component_idx, cv2.CC_STAT_AREA])
        if area < max(3, int(round(min_dim * 0.04))):
            continue
        ys, xs = np.where(labels == component_idx)
        if xs.size < 3:
            continue
        coords = np.column_stack([xs.astype(np.float32), ys.astype(np.float32)])
        coords -= np.mean(coords, axis=0, keepdims=True)
        covariance = np.cov(coords, rowvar=False)
        if not np.all(np.isfinite(covariance)):
            continue
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)
        major_value = float(max(eigenvalues[order[-1]], 0.0))
        minor_value = float(max(eigenvalues[order[0]], 0.0))
        length = 4.0 * math.sqrt(major_value)
        if length < max(4.0, min_dim * 0.10):
            continue
        major_vector = eigenvectors[:, order[-1]]
        angle_diff = _axis_angle_diff_deg(0.0, 0.0, float(major_vector[0]), float(major_vector[1]))
        elongation = math.sqrt((major_value + 1e-6) / (minor_value + 1e-6))
        if angle_diff > 18.0 and elongation >= 1.8:
            diagonal_count += 1
            diagonal_length += length * min(1.0, elongation / 4.0)

    diagonal_score = float(np.clip(diagonal_length / max(min_dim * 0.72, 1.0), 0.0, 1.0))
    area_ratio = float(np.mean(binary[local_mask] > 0)) if np.any(local_mask) else 0.0
    return diagonal_score, int(diagonal_count), area_ratio


def _center_blob_shape_score(response, valid, center_mask, search_mask=None):
    response = np.asarray(response, dtype=np.float32)
    valid = np.asarray(valid, dtype=bool)
    if response.ndim != 2 or not np.any(valid):
        return {
            "score": 0.0,
            "area": 0,
            "extent": 0.0,
            "elongation": 0.0,
            "hull_fill": 0.0,
            "span_ratio": 0.0,
            "center_fill": 0.0,
        }

    min_dim = max(1, min(response.shape[:2]))
    if search_mask is None:
        search_mask = _disc_mask(response.shape, max(2, int(round(min_dim * 0.42))))
    local_mask = valid & np.asarray(search_mask, dtype=bool)
    center_mask = valid & np.asarray(center_mask, dtype=bool)
    if not np.any(local_mask) or not np.any(center_mask):
        return {
            "score": 0.0,
            "area": 0,
            "extent": 0.0,
            "elongation": 0.0,
            "hull_fill": 0.0,
            "span_ratio": 0.0,
            "center_fill": 0.0,
        }

    values = response[local_mask]
    if float(np.percentile(values, 95.0) - np.percentile(values, 5.0)) < 0.08:
        return {
            "score": 0.0,
            "area": 0,
            "extent": 0.0,
            "elongation": 0.0,
            "hull_fill": 0.0,
            "span_ratio": 0.0,
            "center_fill": 0.0,
        }

    threshold = max(0.34, float(np.percentile(values, 76.0)))
    binary = ((response >= threshold) & local_mask).astype(np.uint8) * 255
    if int(np.count_nonzero(binary)) < max(5, int(round(min_dim * 0.18))):
        return {
            "score": 0.0,
            "area": 0,
            "extent": 0.0,
            "elongation": 0.0,
            "hull_fill": 0.0,
            "span_ratio": 0.0,
            "center_fill": 0.0,
        }
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((2, 2), dtype=np.uint8))

    best = {
        "score": 0.0,
        "area": 0,
        "extent": 0.0,
        "elongation": 0.0,
        "hull_fill": 0.0,
        "span_ratio": 0.0,
        "center_fill": 0.0,
    }
    min_area = max(6, int(round((min_dim * min_dim) * 0.025)))
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    for component_idx in range(1, component_count):
        component = labels == component_idx
        if not np.any(component & center_mask):
            continue
        area = int(stats[component_idx, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        bbox_w = int(stats[component_idx, cv2.CC_STAT_WIDTH])
        bbox_h = int(stats[component_idx, cv2.CC_STAT_HEIGHT])
        bbox_area = max(1, bbox_w * bbox_h)
        span_ratio = float(max(bbox_w, bbox_h) / max(float(min_dim), 1.0))
        extent = float(np.clip(area / float(bbox_area), 0.0, 1.0))
        center_fill = float(np.mean(component[center_mask])) if np.any(center_mask) else 0.0

        ys, xs = np.where(component)
        coords = np.column_stack([xs.astype(np.float32), ys.astype(np.float32)])
        if coords.shape[0] >= 3:
            centered = coords - np.mean(coords, axis=0, keepdims=True)
            covariance = np.cov(centered, rowvar=False)
            if np.all(np.isfinite(covariance)):
                eigenvalues = np.linalg.eigvalsh(covariance)
                major_value = float(max(np.max(eigenvalues), 0.0))
                minor_value = float(max(np.min(eigenvalues), 0.0))
                elongation = math.sqrt((major_value + 1e-6) / (minor_value + 1e-6))
            else:
                elongation = 99.0
            hull = cv2.convexHull(coords.astype(np.float32).reshape(-1, 1, 2))
            hull_area = max(float(cv2.contourArea(hull)), float(area), 1.0)
            hull_fill = float(np.clip(area / hull_area, 0.0, 1.0))
        else:
            elongation = 99.0
            hull_fill = 0.0

        compact_extent = _score01(extent - 0.34, 0.46)
        compact_hull = _score01(hull_fill - 0.58, 0.34)
        compact_center = _score01(center_fill, 0.48)
        compact_round = float(np.clip((2.6 - elongation) / 1.6, 0.0, 1.0))
        compact_span = float(np.clip((0.76 - span_ratio) / 0.28, 0.0, 1.0))
        area_score = _score01(area, max(10, int(round((min_dim * min_dim) * 0.07))))
        score = float(
            np.clip(
                (0.24 * compact_extent)
                + (0.22 * compact_hull)
                + (0.22 * compact_center)
                + (0.18 * compact_round)
                + (0.10 * compact_span)
                + (0.04 * area_score),
                0.0,
                1.0,
            )
        )
        if span_ratio > 0.82 or extent < 0.35 or center_fill < 0.22:
            score *= 0.25
        if elongation > 3.2:
            score *= 0.35
        if score > best["score"]:
            best = {
                "score": score,
                "area": area,
                "extent": extent,
                "elongation": elongation,
                "hull_fill": hull_fill,
                "span_ratio": span_ratio,
                "center_fill": center_fill,
            }
    return best


def _build_ridge_patch(ir_patch, depth_patch, valid_mask):
    valid = np.asarray(valid_mask, dtype=bool)
    if not np.any(valid):
        return np.zeros_like(valid, dtype=np.uint8)

    ir = np.asarray(ir_patch, dtype=np.uint8)
    depth = np.asarray(depth_patch, dtype=np.float32)
    if ir.size > 0:
        ir_blur = cv2.GaussianBlur(ir, (3, 3), 0)
        edges = cv2.Canny(ir_blur, 24, 72)
    else:
        edges = np.zeros_like(valid, dtype=np.uint8)

    finite_depth = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
    grad_x = cv2.Sobel(finite_depth, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(finite_depth, cv2.CV_32F, 0, 1, ksize=3)
    gradient = cv2.magnitude(grad_x, grad_y)
    gradient[~valid] = 0.0
    if np.any(gradient[valid] > 0):
        threshold = float(np.percentile(gradient[valid], 75.0))
        depth_edges = (gradient >= threshold).astype(np.uint8) * 255
    else:
        depth_edges = np.zeros_like(edges, dtype=np.uint8)

    ridge = cv2.bitwise_or(edges, depth_edges)
    ridge[~valid] = 0
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        ridge = cv2.ximgproc.thinning(ridge)
    return ridge.astype(np.uint8)


def _compute_bundle_metrics(bundle, config):
    height = np.asarray(bundle.height_patch, dtype=np.float32)
    valid = np.asarray(bundle.valid_depth_mask, dtype=bool)
    ir = np.asarray(bundle.ir_patch, dtype=np.uint8)
    ridge = np.asarray(bundle.ridge_patch, dtype=np.uint8) > 0

    center = _disc_mask(height.shape, config.center_radius_px) & valid
    ring = _ring_mask(height.shape, config.ring_inner_radius_px, config.ring_outer_radius_px) & valid
    if not np.any(ring):
        ring = valid & ~center
    center_height = _mean_or(height[center], 0.0)
    ring_height = _mean_or(height[ring], 0.0)
    center_ir_stddev = _std_or(ir[center], 0.0)
    ring_ir_stddev = _std_or(ir[ring], 0.0)
    valid_depth_ratio = float(np.mean(valid)) if valid.size else 0.0

    cross_mask = np.zeros_like(valid, dtype=bool)
    mid_y = cross_mask.shape[0] // 2
    mid_x = cross_mask.shape[1] // 2
    cross_mask[mid_y, :] = True
    cross_mask[:, mid_x] = True
    center_cross = cross_mask & _disc_mask(height.shape, config.ring_inner_radius_px)
    outer_cross = cross_mask & ~_disc_mask(height.shape, config.ring_inner_radius_px)
    ridge_outer_density = float(np.mean(ridge[outer_cross])) if np.any(outer_cross) else 0.0
    ridge_center_density = float(np.mean(ridge[center_cross])) if np.any(center_cross) else 0.0
    if ridge_outer_density <= 1e-6:
        ridge_break_ratio = 0.0
    else:
        ridge_break_ratio = float(np.clip(1.0 - (ridge_center_density / ridge_outer_density), 0.0, 1.0))

    metrics = dict(bundle.metrics)
    metrics.update({
        "valid_depth_ratio": valid_depth_ratio,
        "center_height_mm": center_height,
        "ring_height_mm": ring_height,
        "center_height_delta_mm": max(0.0, center_height - ring_height),
        "center_ir_stddev": center_ir_stddev,
        "ring_ir_stddev": ring_ir_stddev,
        "ridge_center_density": ridge_center_density,
        "ridge_outer_density": ridge_outer_density,
        "ridge_break_ratio": ridge_break_ratio,
    })
    return metrics


def _quality_from_metrics(metrics, config):
    valid_depth_ratio = float(metrics.get("valid_depth_ratio", 0.0))
    if valid_depth_ratio <= 0.0:
        return 0.0
    return float(np.clip(valid_depth_ratio / max(float(config.min_valid_depth_ratio), 1e-6), 0.0, 1.0))


def extract_evidence_bundle(
    ir_image,
    depth_image,
    raw_world_image,
    point_idx,
    pix_coord,
    world_coord,
    phase,
    config,
):
    half_size = int(config.patch_half_size_px)
    ir_source = _safe_array(ir_image, np.uint8)
    if ir_source is None:
        ir_source = np.zeros((1, 1), dtype=np.uint8)

    raw_world_source = _safe_array(raw_world_image, np.float32)
    depth_source = _safe_array(depth_image, np.float32)
    if depth_source is None and raw_world_source is not None and raw_world_source.ndim == 3 and raw_world_source.shape[2] >= 3:
        depth_source = raw_world_source[:, :, 2]
    if depth_source is None:
        depth_source = np.zeros(ir_source.shape[:2], dtype=np.float32)

    ir_patch = _crop_with_padding(ir_source, pix_coord, half_size, fill_value=0).astype(np.uint8)
    depth_patch = _crop_with_padding(depth_source, pix_coord, half_size, fill_value=0).astype(np.float32)
    if raw_world_source is None:
        raw_world_patch = np.zeros((*depth_patch.shape, 3), dtype=np.float32)
        raw_world_patch[:, :, 2] = depth_patch
    else:
        raw_world_patch = _crop_with_padding(raw_world_source, pix_coord, half_size, fill_value=0).astype(np.float32)

    valid_depth_mask = np.isfinite(depth_patch) & (depth_patch > 0.0)
    height_patch, local_reference_depth = _build_height_patch(depth_patch, valid_depth_mask, config)
    ridge_patch = _build_ridge_patch(ir_patch, depth_patch, valid_depth_mask)
    bundle = EvidenceBundle(
        point_idx=int(point_idx),
        phase=str(phase),
        pix_coord=(int(round(float(pix_coord[0]))), int(round(float(pix_coord[1])))),
        world_coord=tuple(float(value) for value in world_coord[:3]),
        ir_patch=ir_patch,
        depth_patch=depth_patch,
        raw_world_patch=raw_world_patch,
        height_patch=height_patch,
        valid_depth_mask=valid_depth_mask,
        ridge_patch=ridge_patch,
        evidence_quality=0.0,
        metrics={"local_reference_depth_mm": local_reference_depth},
    )
    bundle.metrics = _compute_bundle_metrics(bundle, config)
    bundle.evidence_quality = _quality_from_metrics(bundle.metrics, config)
    return bundle


def classify_pre_bind_rule(bundle, config):
    metrics = _compute_bundle_metrics(bundle, config)
    quality = _quality_from_metrics(metrics, config)
    if metrics["valid_depth_ratio"] < float(config.min_valid_depth_ratio):
        return RuleDecision(
            label="uncertain",
            score=0.0,
            quality=quality,
            reason="valid_depth_ratio_too_low",
            metrics=metrics,
        )

    height_score = _score01(metrics["center_height_delta_mm"], config.bound_height_delta_mm)
    texture_score = _score01(metrics["center_ir_stddev"], config.bound_texture_stddev)
    ridge_score = _score01(metrics["ridge_break_ratio"], config.ridge_break_ratio_threshold)
    score = float(np.clip((0.48 * height_score) + (0.34 * texture_score) + (0.18 * ridge_score), 0.0, 1.0))

    if score >= float(config.pre_bind_bound_score_high):
        label = "bound"
        reason = "center_bump_texture_or_ridge_break"
    elif score <= float(config.pre_bind_bound_score_low):
        label = "unbound"
        reason = "clean_flat_crossing"
    else:
        label = "uncertain"
        reason = "score_between_thresholds"

    metrics.update({
        "height_score": height_score,
        "texture_score": texture_score,
        "ridge_score": ridge_score,
    })
    return RuleDecision(label=label, score=score, quality=quality, reason=reason, metrics=metrics)


def classify_pre_bind_white_box(bundle, config):
    raw_world = np.asarray(bundle.raw_world_patch, dtype=np.float32)
    if raw_world.ndim == 3 and raw_world.shape[2] >= 3:
        depth = raw_world[:, :, 2]
        response_source = "multimodal_cross_residual"
    else:
        depth = np.asarray(bundle.depth_patch, dtype=np.float32)
        response_source = "depth_patch_cross_residual"
    valid = np.isfinite(depth) & (depth > 0.0)
    valid_ratio = float(np.mean(valid)) if valid.size else 0.0
    quality = float(np.clip(valid_ratio / max(float(config.min_valid_depth_ratio), 1e-6), 0.0, 1.0))
    if valid_ratio < float(config.min_valid_depth_ratio):
        return RuleDecision(
            label="uncertain",
            score=0.0,
            quality=quality,
            reason="valid_ratio_too_low",
            metrics={
                "valid_ratio": valid_ratio,
                "white_box_response_source": response_source,
            },
        )

    ir = np.asarray(bundle.ir_patch, dtype=np.uint8)
    height_response = _local_height_response(depth, valid, sigma=11.0)
    height_mm_response = _local_height_mm_response(depth, valid, config)
    normalized_height_mm_response = _normalize_response(height_mm_response, valid, low=0.0, high=98.0)
    dark_response = _local_dark_response(ir, valid, sigma=7.0)
    ir_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(np.clip(ir, 0, 255).astype(np.uint8))
    clahe_dark_response = _local_dark_response(ir_clahe, valid, sigma=7.0)
    gradient_response = _gradient_response(depth, valid)

    fused_response = _normalize_response(
        (0.46 * height_response)
        + (0.24 * dark_response)
        + (0.18 * clahe_dark_response)
        + (0.12 * gradient_response),
        valid,
    )

    min_dim = max(1, min(fused_response.shape[:2]))
    center_radius = max(int(config.center_radius_px), int(round(min_dim * 0.12)))
    ring_inner = max(int(config.ring_inner_radius_px), center_radius + 2)
    ring_outer = max(int(config.ring_outer_radius_px), ring_inner + 2)
    ring_outer = min(ring_outer, max(1, (min_dim // 2) - 1))
    if ring_outer <= ring_inner:
        ring_outer = min(max(ring_inner + 2, ring_inner + 1), max(1, (min_dim // 2) - 1))
    center = _disc_mask(fused_response.shape, center_radius) & valid
    ring = _ring_mask(fused_response.shape, ring_inner, ring_outer) & valid
    if not np.any(ring):
        ring = valid & ~center

    center_index_y = fused_response.shape[0] // 2
    center_index_x = fused_response.shape[1] // 2
    search_radius = max(2, min_dim // 4)
    axis_half_width = max(2, int(round(min_dim * 0.06)))
    row_y = _profile_axis_center(fused_response, valid, "row", center_index_y, search_radius, axis_half_width)
    col_x = _profile_axis_center(fused_response, valid, "col", center_index_x, search_radius, axis_half_width)
    axis_mask = np.zeros_like(valid, dtype=bool)
    axis_mask[max(0, row_y - axis_half_width) : min(axis_mask.shape[0], row_y + axis_half_width + 1), :] = True
    axis_mask[:, max(0, col_x - axis_half_width) : min(axis_mask.shape[1], col_x + axis_half_width + 1)] = True
    axis_mask &= valid
    adaptive_row_half_width = _axis_profile_half_width(fused_response, valid, "row", row_y, axis_half_width)
    adaptive_col_half_width = _axis_profile_half_width(fused_response, valid, "col", col_x, axis_half_width)
    diagonal_axis_mask = np.zeros_like(valid, dtype=bool)
    diagonal_axis_mask[
        max(0, row_y - adaptive_row_half_width) : min(diagonal_axis_mask.shape[0], row_y + adaptive_row_half_width + 1),
        :,
    ] = True
    diagonal_axis_mask[
        :,
        max(0, col_x - adaptive_col_half_width) : min(diagonal_axis_mask.shape[1], col_x + adaptive_col_half_width + 1),
    ] = True
    diagonal_axis_mask &= valid

    off_center = center & ~axis_mask
    off_ring = ring & ~axis_mask
    axis_outer = axis_mask & ~center
    axis_center = axis_mask & center

    fused_center_mean = _mean_mask(fused_response, center, 0.0)
    fused_ring_mean = _mean_mask(fused_response, ring, 0.0)
    fused_off_center_mean = _mean_mask(fused_response, off_center, 0.0)
    fused_off_ring_mean = _mean_mask(fused_response, off_ring, 0.0)
    fused_axis_outer_mean = _mean_mask(fused_response, axis_outer, 0.0)
    fused_axis_center_mean = _mean_mask(fused_response, axis_center, 0.0)

    dark_off_center_mean = _mean_mask(dark_response, off_center, 0.0)
    dark_off_ring_mean = _mean_mask(dark_response, off_ring, 0.0)
    height_off_center_mean = _mean_mask(height_response, off_center, 0.0)
    height_off_ring_mean = _mean_mask(height_response, off_ring, 0.0)

    off_axis_delta = max(0.0, fused_off_center_mean - fused_off_ring_mean)
    off_axis_dark_delta = max(0.0, dark_off_center_mean - dark_off_ring_mean)
    off_axis_height_delta = max(0.0, height_off_center_mean - height_off_ring_mean)
    axis_explained_ratio = (
        (fused_axis_outer_mean + (0.75 * fused_axis_center_mean))
        / max(fused_off_center_mean + 0.05, 0.05)
    )

    near_cross_radius = min(
        max(1, (min_dim // 2) - 1),
        max(ring_outer, int(round(min_dim * 0.46))),
    )
    near_cross_mask = _disc_mask(fused_response.shape, near_cross_radius) & valid
    residual = np.clip(fused_response - (0.85 * axis_mask.astype(np.float32)), 0.0, 1.0)
    center_diagonal_scores = []
    center_diagonal_counts = []
    center_diagonal_area_ratios = []
    off_axis_diagonal_scores = []
    off_axis_diagonal_counts = []
    off_axis_diagonal_area_ratios = []
    for diagonal_source in (
        fused_response,
        clahe_dark_response,
        height_response,
    ):
        center_score, center_count, center_area_ratio = _diagonal_residual_score(
            np.clip(diagonal_source - (0.85 * axis_mask.astype(np.float32)), 0.0, 1.0),
            valid,
            axis_mask,
            center,
        )
        off_score, off_count, off_area_ratio = _off_axis_diagonal_score(
            diagonal_source,
            valid,
            diagonal_axis_mask,
            near_cross_mask,
        )
        center_diagonal_scores.append(center_score)
        center_diagonal_counts.append(int(center_count))
        center_diagonal_area_ratios.append(center_area_ratio)
        off_axis_diagonal_scores.append(off_score)
        off_axis_diagonal_counts.append(int(off_count))
        off_axis_diagonal_area_ratios.append(off_area_ratio)

    diagonal_score = max([0.0] + center_diagonal_scores + off_axis_diagonal_scores)
    diagonal_count = int(sum(center_diagonal_counts) + sum(off_axis_diagonal_counts))
    diagonal_area_ratio = max([0.0] + center_diagonal_area_ratios + off_axis_diagonal_area_ratios)
    center_diagonal_score = max(center_diagonal_scores) if center_diagonal_scores else 0.0
    center_diagonal_count = int(sum(center_diagonal_counts))
    center_diagonal_area_ratio = max(center_diagonal_area_ratios) if center_diagonal_area_ratios else 0.0
    off_axis_diagonal_score = max(off_axis_diagonal_scores) if off_axis_diagonal_scores else 0.0
    off_axis_diagonal_count = int(sum(off_axis_diagonal_counts))
    off_axis_diagonal_area_ratio = max(off_axis_diagonal_area_ratios) if off_axis_diagonal_area_ratios else 0.0
    center_blob_shape = _center_blob_shape_score(
        np.maximum(height_response, normalized_height_mm_response),
        valid,
        center,
        near_cross_mask,
    )
    center_blob_score = float(center_blob_shape["score"])
    filament_blob_score = 0.0
    if (
        off_axis_diagonal_area_ratio >= 0.08
        and float(center_blob_shape["elongation"]) >= 3.0
        and float(center_blob_shape["extent"]) < 0.48
        and float(center_blob_shape["center_fill"]) >= 0.28
    ):
        filament_blob_score = float(
            np.clip(
                0.42
                + (0.36 * _score01(off_axis_diagonal_area_ratio, 0.20))
                + (0.22 * _score01(float(center_blob_shape["elongation"]) - 3.0, 3.0)),
                0.0,
                1.0,
            )
        )
        diagonal_score = max(diagonal_score, filament_blob_score)
        off_axis_diagonal_score = max(off_axis_diagonal_score, filament_blob_score)
        diagonal_count = max(int(diagonal_count), 1)
        off_axis_diagonal_count = max(int(off_axis_diagonal_count), 1)

    ridge_break_ratio = float(bundle.metrics.get("ridge_break_ratio", 0.0))
    diagonal_component = _score01(diagonal_score, 0.62)
    off_axis_component = _score01(off_axis_delta, 0.16)
    dark_component = _score01(off_axis_dark_delta, 0.15)
    height_component = _score01(off_axis_height_delta, 0.16)
    center_blob_component = _score01(center_blob_score, 0.72)
    ridge_component = _score01(ridge_break_ratio, 0.35)

    strong_clean_crossing_prior = (
        axis_explained_ratio >= 1.30
        and float(bundle.metrics.get("center_ir_stddev", 0.0)) < 1.0
        and center_blob_score < 0.48
    )
    has_diagonal_wire = (
        diagonal_score >= 0.34
        and diagonal_count >= 1
        and (
            not strong_clean_crossing_prior
            or center_diagonal_count > 0
            or diagonal_score >= 1.20
        )
    )
    has_center_knot = (
        axis_explained_ratio < 1.36
        and off_axis_height_delta >= 0.24
        and (
            off_axis_dark_delta >= 0.10
            or float(bundle.metrics.get("center_ir_stddev", 0.0)) >= float(config.bound_texture_stddev)
            or center_blob_score >= 0.62
        )
    )
    has_center_blob_knot = (
        axis_explained_ratio < 1.38
        and off_axis_height_delta >= 0.42
        and center_blob_score >= 0.62
    )
    has_ridge_break_knot = (
        axis_explained_ratio < 1.45
        and ridge_break_ratio >= 0.35
        and off_axis_height_delta >= 0.18
    )
    clean_crossing = (
        axis_explained_ratio >= 1.30
        and not has_diagonal_wire
        and not has_center_blob_knot
    )
    positive_votes = int(has_diagonal_wire) * 2
    positive_votes += int(has_center_knot)
    positive_votes += int(has_center_blob_knot)
    positive_votes += int(has_ridge_break_knot)
    score = float(
        np.clip(
            (0.55 * diagonal_component)
            + (0.16 * height_component)
            + (0.14 * dark_component)
            + (0.07 * off_axis_component)
            + (0.03 * center_blob_component)
            + (0.05 * ridge_component),
            0.0,
            1.0,
        )
    )

    if has_diagonal_wire or has_center_knot or has_center_blob_knot or has_ridge_break_knot:
        label = "bound"
        reason = "white_box_multimodal_tie_residual"
        score = max(score, float(config.pre_bind_bound_score_high))
    elif clean_crossing:
        label = "unbound"
        reason = "white_box_clean_rebar_crossing"
    elif score <= float(config.pre_bind_bound_score_low):
        label = "unbound"
        reason = "white_box_flat_clean_low_response"
    else:
        label = "uncertain"
        reason = "white_box_score_between_thresholds"

    metrics = dict(bundle.metrics)
    metrics.update({
        "valid_ratio": valid_ratio,
        "white_box_response_source": response_source,
        "white_box_fused_center_mean": fused_center_mean,
        "white_box_fused_ring_mean": fused_ring_mean,
        "white_box_fused_off_center_mean": fused_off_center_mean,
        "white_box_fused_off_ring_mean": fused_off_ring_mean,
        "white_box_axis_outer_mean": fused_axis_outer_mean,
        "white_box_axis_center_mean": fused_axis_center_mean,
        "white_box_axis_explained_ratio": axis_explained_ratio,
        "white_box_axis_row_half_width": int(adaptive_row_half_width),
        "white_box_axis_col_half_width": int(adaptive_col_half_width),
        "white_box_off_axis_delta": off_axis_delta,
        "white_box_off_axis_dark_delta": off_axis_dark_delta,
        "white_box_off_axis_height_delta": off_axis_height_delta,
        "white_box_diagonal_score": diagonal_score,
        "white_box_diagonal_count": int(diagonal_count),
        "white_box_diagonal_area_ratio": diagonal_area_ratio,
        "white_box_center_diagonal_score": center_diagonal_score,
        "white_box_center_diagonal_count": int(center_diagonal_count),
        "white_box_center_diagonal_area_ratio": center_diagonal_area_ratio,
        "white_box_off_axis_diagonal_score": off_axis_diagonal_score,
        "white_box_off_axis_diagonal_count": int(off_axis_diagonal_count),
        "white_box_off_axis_diagonal_area_ratio": off_axis_diagonal_area_ratio,
        "white_box_positive_votes": int(positive_votes),
        "white_box_score": score,
        "white_box_diagonal_component": diagonal_component,
        "white_box_off_axis_component": off_axis_component,
        "white_box_dark_component": dark_component,
        "white_box_height_component": height_component,
        "white_box_center_blob_score": center_blob_score,
        "white_box_center_blob_area": int(center_blob_shape["area"]),
        "white_box_center_blob_extent": float(center_blob_shape["extent"]),
        "white_box_center_blob_elongation": float(center_blob_shape["elongation"]),
        "white_box_center_blob_hull_fill": float(center_blob_shape["hull_fill"]),
        "white_box_center_blob_span_ratio": float(center_blob_shape["span_ratio"]),
        "white_box_center_blob_center_fill": float(center_blob_shape["center_fill"]),
        "white_box_center_blob_component": center_blob_component,
        "white_box_filament_blob_score": filament_blob_score,
        "white_box_ridge_component": ridge_component,
        "white_box_has_diagonal_wire": bool(has_diagonal_wire),
        "white_box_has_center_knot": bool(has_center_knot),
        "white_box_has_center_blob_knot": bool(has_center_blob_knot),
        "white_box_has_ridge_break_knot": bool(has_ridge_break_knot),
        "white_box_clean_crossing": bool(clean_crossing),
    })
    return RuleDecision(label=label, score=score, quality=quality, reason=reason, metrics=metrics)


def build_deep_learning_patch(ir_image, depth_image, pix_coord, raw_z, half_size=30, tolerance_mm=75.0):
    ir_source = _safe_array(ir_image)
    if ir_source is None:
        ir_source = np.zeros((1, 1), dtype=np.uint8)
    depth_source = _safe_array(depth_image, np.float32)
    if depth_source is None:
        depth_source = np.zeros(ir_source.shape[:2], dtype=np.float32)

    half_size = max(1, int(half_size))
    ir_patch = _crop_with_padding(ir_source, pix_coord, half_size, fill_value=0)
    depth_patch = _crop_with_padding(depth_source, pix_coord, half_size, fill_value=0).astype(np.float32)
    if ir_patch.ndim == 3:
        ir_patch = cv2.cvtColor(ir_patch, cv2.COLOR_BGR2GRAY)

    raw_z_value = float(raw_z)
    if not math.isfinite(raw_z_value) or raw_z_value <= 0.0:
        valid_depth = depth_patch[np.isfinite(depth_patch) & (depth_patch > 0.0)]
        raw_z_value = float(np.median(valid_depth)) if valid_depth.size else 0.0

    masked_patch = np.zeros(ir_patch.shape[:2], dtype=ir_patch.dtype)
    if raw_z_value > 0.0:
        depth_mask = (
            np.isfinite(depth_patch)
            & (depth_patch >= raw_z_value - float(tolerance_mm))
            & (depth_patch <= raw_z_value + float(tolerance_mm))
        )
        masked_patch[depth_mask] = ir_patch[depth_mask]
    else:
        masked_patch[...] = ir_patch

    resized = cv2.resize(masked_patch, (128, 128))
    if resized.dtype != np.uint8:
        resized = cv2.normalize(resized, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(resized)


def _load_ultralytics_model(model_path):
    from ultralytics import YOLO

    model = YOLO(str(model_path))
    try:
        import torch

        model.to("cuda" if torch.cuda.is_available() else "cpu")
    except Exception:
        pass
    return model


def get_deep_learning_model(config, cache=None):
    if cache is not None:
        cached_path = cache.get("path")
        cached_model = cache.get("model")
        if cached_model is not None and cached_path == str(config.deep_learning_model_path):
            return cached_model
    model = _load_ultralytics_model(config.deep_learning_model_path)
    if cache is not None:
        cache["path"] = str(config.deep_learning_model_path)
        cache["model"] = model
    return model


def _extract_yolo_top1(result):
    probs = getattr(result, "probs", None)
    if probs is None:
        return None, 0.0
    top1 = getattr(probs, "top1", None)
    if top1 is None:
        return None, 0.0
    confidence = getattr(probs, "top1conf", 0.0)
    try:
        if hasattr(confidence, "item"):
            confidence = confidence.item()
        confidence = float(confidence)
    except (TypeError, ValueError):
        confidence = 0.0
    return int(top1), confidence


def classify_pre_bind_deep_learning(bundle, config, model=None):
    if model is None:
        model = get_deep_learning_model(config)
    raw_z = float(bundle.world_coord[2]) if len(bundle.world_coord) >= 3 else 0.0
    if np.any(bundle.depth_patch > 0.0):
        center_y = bundle.depth_patch.shape[0] // 2
        center_x = bundle.depth_patch.shape[1] // 2
        raw_z = float(bundle.depth_patch[center_y, center_x])
    patch = build_deep_learning_patch(
        bundle.ir_patch,
        bundle.depth_patch,
        (bundle.ir_patch.shape[1] // 2, bundle.ir_patch.shape[0] // 2),
        raw_z,
        half_size=max(1, min(bundle.ir_patch.shape[:2]) // 2),
        tolerance_mm=config.deep_learning_depth_tolerance_mm,
    )
    try:
        results = model(patch, conf=float(config.deep_learning_confidence), verbose=False)
    except Exception as exc:
        return RuleDecision(
            label="uncertain",
            score=0.0,
            quality=float(bundle.evidence_quality),
            reason=f"deep_learning_inference_failed:{exc}",
            metrics=dict(bundle.metrics),
        )
    if not results:
        return RuleDecision(
            label="uncertain",
            score=0.0,
            quality=float(bundle.evidence_quality),
            reason="deep_learning_no_result",
            metrics=dict(bundle.metrics),
        )
    top1_class, confidence = _extract_yolo_top1(results[0])
    metrics = dict(bundle.metrics)
    metrics.update({
        "deep_learning_top1_class": top1_class,
        "deep_learning_confidence": confidence,
    })
    if top1_class == int(config.deep_learning_bound_class_id):
        return RuleDecision(
            "bound",
            max(confidence, float(config.pre_bind_bound_score_high)),
            float(bundle.evidence_quality),
            "deep_learning_bound_class",
            metrics,
        )
    if top1_class == int(config.deep_learning_unbound_class_id):
        return RuleDecision("unbound", confidence, float(bundle.evidence_quality), "deep_learning_unbound_class", metrics)
    return RuleDecision("uncertain", confidence, float(bundle.evidence_quality), "deep_learning_unknown_class", metrics)


def classify_pre_bind_point(bundle, config, model_cache=None):
    if normalize_classification_method(getattr(config, "method", "deep_learning")) == "white_box":
        return classify_pre_bind_white_box(bundle, config)
    model = get_deep_learning_model(config, model_cache)
    return classify_pre_bind_deep_learning(bundle, config, model=model)


def verify_post_bind_rule(before_bundle, after_bundle, config):
    before_metrics = _compute_bundle_metrics(before_bundle, config)
    after_metrics = _compute_bundle_metrics(after_bundle, config)
    before_quality = _quality_from_metrics(before_metrics, config)
    after_quality = _quality_from_metrics(after_metrics, config)
    quality = min(before_quality, after_quality)
    if quality < 1.0:
        return RuleDecision(
            label="uncertain",
            score=0.0,
            quality=quality,
            reason="before_or_after_quality_too_low",
            metrics={"before_quality": before_quality, "after_quality": after_quality},
        )

    height_delta = max(
        0.0,
        after_metrics["center_height_delta_mm"] - before_metrics["center_height_delta_mm"],
    )
    texture_delta = max(
        0.0,
        after_metrics["center_ir_stddev"] - before_metrics["center_ir_stddev"],
    )
    ridge_delta = max(
        0.0,
        after_metrics["ridge_break_ratio"] - before_metrics["ridge_break_ratio"],
    )
    height_score = _score01(height_delta, config.bound_height_delta_mm)
    texture_score = _score01(texture_delta, config.bound_texture_stddev)
    ridge_score = _score01(ridge_delta, config.ridge_break_ratio_threshold)
    score = float(np.clip((0.55 * height_score) + (0.30 * texture_score) + (0.15 * ridge_score), 0.0, 1.0))

    if score >= float(config.post_bind_success_score_high):
        label = "success"
        reason = "after_has_new_bound_evidence"
    elif score <= float(config.post_bind_success_score_low):
        label = "failed"
        reason = "after_before_delta_too_small"
    else:
        label = "uncertain"
        reason = "post_score_between_thresholds"

    return RuleDecision(
        label=label,
        score=score,
        quality=quality,
        reason=reason,
        metrics={
            "height_delta_mm": height_delta,
            "texture_delta": texture_delta,
            "ridge_break_delta": ridge_delta,
            "height_score": height_score,
            "texture_score": texture_score,
            "ridge_score": ridge_score,
            "before_quality": before_quality,
            "after_quality": after_quality,
        },
    )


def should_mark_point_as_bound(decision, config):
    mode = normalize_classification_mode(getattr(config, "mode", "shadow"))
    return mode in {"advisory", "blocking"} and getattr(decision, "label", "") == "bound"


def filter_unbound_points_for_execution(points, config):
    if normalize_classification_mode(getattr(config, "mode", "off")) != "blocking":
        return points
    return [point for point in points if not bool(getattr(point, "is_shuiguan", False))]


def _json_safe(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    return value


def append_classification_event(event_log_path, payload):
    path = Path(event_log_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    event = dict(payload)
    event.setdefault("timestamp", time.time())
    with path.open("a", encoding="utf-8") as file_obj:
        file_obj.write(json.dumps(_json_safe(event), ensure_ascii=False, sort_keys=True) + "\n")


def _bundle_manifest(bundle):
    return {
        "point_idx": int(bundle.point_idx),
        "phase": bundle.phase,
        "pix_coord": list(bundle.pix_coord),
        "world_coord": list(bundle.world_coord),
        "evidence_quality": float(bundle.evidence_quality),
        "metrics": _json_safe(bundle.metrics),
    }


def persist_evidence_bundle(bundle, config, session_id=None):
    session = str(session_id or time.strftime("%Y%m%d_%H%M%S"))
    point_dir = Path(config.evidence_root) / session / f"point_{int(bundle.point_idx):03d}" / str(bundle.phase)
    point_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(point_dir / "ir.png"), np.asarray(bundle.ir_patch, dtype=np.uint8))
    cv2.imwrite(
        str(point_dir / "height.png"),
        np.clip(np.asarray(bundle.height_patch, dtype=np.float32) * 16.0, 0, 255).astype(np.uint8),
    )
    np.save(point_dir / "depth.npy", np.asarray(bundle.depth_patch, dtype=np.float32))
    np.save(point_dir / "height.npy", np.asarray(bundle.height_patch, dtype=np.float32))
    np.save(point_dir / "raw_world.npy", np.asarray(bundle.raw_world_patch, dtype=np.float32))
    (point_dir / "manifest.json").write_text(
        json.dumps(_bundle_manifest(bundle), ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return point_dir


def decisions_to_summary(decisions):
    counts = {"bound": 0, "unbound": 0, "uncertain": 0}
    for decision in decisions:
        label = getattr(decision, "label", "uncertain")
        counts[label] = counts.get(label, 0) + 1
    return counts


def iter_point_messages(point_coords):
    for point in getattr(point_coords, "PointCoordinatesArray", []) or []:
        pix_coord = getattr(point, "Pix_coord", [])
        world_coord = getattr(point, "World_coord", [])
        if len(pix_coord) < 2 or len(world_coord) < 3:
            continue
        yield point
