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


@dataclass
class ClassificationConfig:
    mode: str = "shadow"
    event_log_path: Path = Path("src/tie_robot_vision/data/bind_classification_events.jsonl")
    evidence_root: Path = Path("src/tie_robot_vision/data/bind_evidence")
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
        self.event_log_path = Path(self.event_log_path)
        self.evidence_root = Path(self.evidence_root)
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
    normalized = str(mode or "shadow").strip().lower()
    return normalized if normalized in CLASSIFICATION_MODES else "shadow"


def _as_path(value, fallback):
    if value is None or str(value).strip() == "":
        return Path(fallback)
    return Path(value)


def _resolve_portable_path(path, base_dir):
    path = Path(path)
    if path.is_absolute():
        return path
    return Path(base_dir) / path


def load_classification_config(config_path=None, overrides=None):
    data = {}
    base_dir = Path.cwd()
    if config_path:
        path = Path(config_path)
        base_dir = path.resolve().parent.parent
        if path.exists():
            loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            if isinstance(loaded, dict):
                data.update(loaded)
    if overrides:
        data.update({key: value for key, value in dict(overrides).items() if value is not None})

    return ClassificationConfig(
        mode=data.get("mode", "shadow"),
        event_log_path=_resolve_portable_path(
            _as_path(data.get("event_log_path"), "data/bind_classification_events.jsonl"),
            base_dir,
        ),
        evidence_root=_resolve_portable_path(
            _as_path(data.get("evidence_root"), "data/bind_evidence"),
            base_dir,
        ),
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
