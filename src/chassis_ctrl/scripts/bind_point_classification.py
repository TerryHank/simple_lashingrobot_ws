#!/usr/bin/env python3

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

import cv2
import numpy as np
import yaml


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "bind_point_classification.yaml"


@dataclass(frozen=True)
class ClassificationConfig:
    mode: str
    event_log_path: Path
    evidence_root: Path
    patch_half_size_px: int
    height_band_mm: float
    alignment_search_px: int
    min_valid_depth_ratio: float
    min_ir_stddev: float
    pre_bind_bound_score_low: float
    pre_bind_bound_score_high: float
    post_bind_success_score_low: float
    post_bind_success_score_high: float


@dataclass(frozen=True)
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
    evidence_quality: float = 1.0


@dataclass(frozen=True)
class RuleDecision:
    label: str
    score: float
    quality: float
    reason: str
    metrics: Dict[str, Any] = field(default_factory=dict)


def _as_path(value: Any) -> Path:
    return value if isinstance(value, Path) else Path(str(value))


def _read_attr(source: Any, key: str, default: Any = None) -> Any:
    if source is None:
        return default
    if isinstance(source, Mapping):
        return source.get(key, default)
    return getattr(source, key, default)


def _ensure_array(value: Any, dtype: Optional[np.dtype] = None) -> np.ndarray:
    array = np.asarray(value)
    if dtype is not None:
        array = array.astype(dtype, copy=False)
    return array


def _clip01(value: float) -> float:
    return float(max(0.0, min(1.0, value)))


def _masked_values(array: np.ndarray, mask: Optional[np.ndarray]) -> np.ndarray:
    flat = np.asarray(array).reshape(-1)
    if mask is None:
        return flat
    mask_flat = np.asarray(mask, dtype=bool).reshape(-1)
    if mask_flat.size != flat.size:
        return flat
    selected = flat[mask_flat]
    return selected if selected.size else flat


def _center_outer_regions(array: np.ndarray, search_px: int) -> Tuple[np.ndarray, np.ndarray]:
    array = np.asarray(array)
    if array.ndim < 2:
        flat = array.reshape(-1)
        return flat, flat

    height, width = array.shape[:2]
    center_half = max(1, min(search_px, height // 2, width // 2))
    cy = height // 2
    cx = width // 2
    y0 = max(0, cy - center_half)
    y1 = min(height, cy + center_half)
    x0 = max(0, cx - center_half)
    x1 = min(width, cx + center_half)

    center = array[y0:y1, x0:x1]
    outer_mask = np.ones((height, width), dtype=bool)
    outer_mask[y0:y1, x0:x1] = False
    outer = array[outer_mask]
    if outer.size == 0:
        outer = center.reshape(-1)
    return center.reshape(-1), outer.reshape(-1)


def _mean_and_std(values: np.ndarray) -> Tuple[float, float]:
    if values.size == 0:
        return 0.0, 0.0
    return float(np.mean(values)), float(np.std(values))


def _quality_reason(valid_ratio: float, min_valid_ratio: float) -> Optional[str]:
    if valid_ratio < min_valid_ratio:
        return (
            f"valid_depth_ratio={valid_ratio:.2f} below "
            f"min_valid_depth_ratio={min_valid_ratio:.2f}"
        )
    return None


def load_classification_config(config_path: Optional[os.PathLike] = None) -> ClassificationConfig:
    path = _as_path(config_path) if config_path is not None else DEFAULT_CONFIG_PATH
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    return ClassificationConfig(
        mode=str(raw["mode"]),
        event_log_path=_as_path(raw["event_log_path"]),
        evidence_root=_as_path(raw["evidence_root"]),
        patch_half_size_px=int(raw["patch_half_size_px"]),
        height_band_mm=float(raw["height_band_mm"]),
        alignment_search_px=int(raw["alignment_search_px"]),
        min_valid_depth_ratio=float(raw["min_valid_depth_ratio"]),
        min_ir_stddev=float(raw["min_ir_stddev"]),
        pre_bind_bound_score_low=float(raw["pre_bind_bound_score_low"]),
        pre_bind_bound_score_high=float(raw["pre_bind_bound_score_high"]),
        post_bind_success_score_low=float(raw["post_bind_success_score_low"]),
        post_bind_success_score_high=float(raw["post_bind_success_score_high"]),
    )


def extract_evidence_bundle(source: Any = None, **kwargs: Any) -> EvidenceBundle:
    fields = {
        "point_idx": kwargs.get("point_idx", _read_attr(source, "point_idx")),
        "phase": kwargs.get("phase", _read_attr(source, "phase")),
        "pix_coord": kwargs.get("pix_coord", _read_attr(source, "pix_coord")),
        "world_coord": kwargs.get("world_coord", _read_attr(source, "world_coord")),
        "ir_patch": kwargs.get("ir_patch", _read_attr(source, "ir_patch")),
        "depth_patch": kwargs.get("depth_patch", _read_attr(source, "depth_patch")),
        "raw_world_patch": kwargs.get("raw_world_patch", _read_attr(source, "raw_world_patch")),
        "height_patch": kwargs.get("height_patch", _read_attr(source, "height_patch")),
        "valid_depth_mask": kwargs.get("valid_depth_mask", _read_attr(source, "valid_depth_mask")),
        "evidence_quality": kwargs.get("evidence_quality", _read_attr(source, "evidence_quality", 1.0)),
    }

    missing = [name for name, value in fields.items() if value is None]
    if missing:
        raise ValueError("missing required evidence fields: {}".format(", ".join(missing)))

    return EvidenceBundle(
        point_idx=int(fields["point_idx"]),
        phase=str(fields["phase"]),
        pix_coord=tuple(fields["pix_coord"]),
        world_coord=tuple(fields["world_coord"]),
        ir_patch=_ensure_array(fields["ir_patch"], dtype=np.uint8),
        depth_patch=_ensure_array(fields["depth_patch"], dtype=np.float32),
        raw_world_patch=_ensure_array(fields["raw_world_patch"], dtype=np.float32),
        height_patch=_ensure_array(fields["height_patch"], dtype=np.float32),
        valid_depth_mask=_ensure_array(fields["valid_depth_mask"], dtype=bool),
        evidence_quality=float(fields["evidence_quality"]),
    )


def _score_pre_bind(bundle: EvidenceBundle, config: ClassificationConfig) -> Tuple[float, Dict[str, float]]:
    valid_ratio = float(np.mean(bundle.valid_depth_mask)) if bundle.valid_depth_mask.size else 0.0
    center_height, outer_height = _center_outer_regions(bundle.height_patch, config.alignment_search_px)
    center_ir, outer_ir = _center_outer_regions(bundle.ir_patch, config.alignment_search_px)
    center_height_mean, center_height_std = _mean_and_std(_masked_values(center_height, None))
    outer_height_mean, outer_height_std = _mean_and_std(_masked_values(outer_height, None))
    center_ir_mean, center_ir_std = _mean_and_std(_masked_values(center_ir, None))
    outer_ir_mean, outer_ir_std = _mean_and_std(_masked_values(outer_ir, None))
    center_height_peak = float(np.max(center_height)) if center_height.size else 0.0
    center_ir_peak = float(np.max(center_ir)) if center_ir.size else 0.0

    height_delta = max(0.0, center_height_mean - outer_height_mean)
    height_peak = max(0.0, float(np.max(center_height)) - outer_height_mean) if center_height.size else 0.0
    ir_delta = max(0.0, center_ir_mean - outer_ir_mean)
    ir_std = float(np.std(_masked_values(bundle.ir_patch, bundle.valid_depth_mask)))

    height_delta_score = _clip01(center_height_peak / max(config.height_band_mm / 2.0, 1.0))
    height_peak_score = _clip01(height_peak / max(config.height_band_mm / 1.5, 1.0))
    ir_delta_score = _clip01((center_ir_peak - outer_ir_mean) / max(config.min_ir_stddev * 8.0, 1.0))
    ir_std_score = _clip01(ir_std / max(config.min_ir_stddev * 3.0, 1.0))

    quality_scale = _clip01((valid_ratio - config.min_valid_depth_ratio) / max(1.0 - config.min_valid_depth_ratio, 1e-6))
    raw_score = (
        0.50 * height_delta_score
        + 0.25 * height_peak_score
        + 0.15 * ir_delta_score
        + 0.10 * ir_std_score
    )
    score = raw_score * (0.6 + 0.4 * quality_scale)
    metrics = {
        "valid_depth_ratio": valid_ratio,
        "height_delta_mm": height_delta,
        "height_peak_mm": height_peak,
        "ir_delta": ir_delta,
        "ir_stddev": ir_std,
        "center_height_peak": center_height_peak,
        "center_ir_peak": center_ir_peak,
        "center_height_mean": center_height_mean,
        "outer_height_mean": outer_height_mean,
        "center_ir_mean": center_ir_mean,
        "outer_ir_mean": outer_ir_mean,
        "center_height_std": center_height_std,
        "outer_height_std": outer_height_std,
        "center_ir_std": center_ir_std,
        "outer_ir_std": outer_ir_std,
        "quality_scale": quality_scale,
        "raw_score": raw_score,
    }
    return score, metrics


def classify_pre_bind_rule(bundle: EvidenceBundle, config: ClassificationConfig) -> RuleDecision:
    valid_ratio = float(np.mean(bundle.valid_depth_mask)) if bundle.valid_depth_mask.size else 0.0
    quality_reason = _quality_reason(valid_ratio, config.min_valid_depth_ratio)
    score, metrics = _score_pre_bind(bundle, config)
    quality = float(bundle.evidence_quality)

    if quality_reason is not None:
        metrics["quality_reason"] = quality_reason
        return RuleDecision(label="uncertain", score=score, quality=quality, reason=quality_reason, metrics=metrics)

    if score >= config.pre_bind_bound_score_high:
        label = "bound"
        reason = "strong center bump and infrared variation"
    elif score <= config.pre_bind_bound_score_low:
        label = "unbound"
        reason = "flat patch with weak height and infrared contrast"
    else:
        label = "uncertain"
        reason = "score between pre-bind thresholds"

    metrics["rule"] = "pre_bind"
    return RuleDecision(label=label, score=score, quality=quality, reason=reason, metrics=metrics)


def _score_post_bind(before_bundle: EvidenceBundle, after_bundle: EvidenceBundle, config: ClassificationConfig) -> Tuple[float, Dict[str, float]]:
    before_valid_ratio = float(np.mean(before_bundle.valid_depth_mask)) if before_bundle.valid_depth_mask.size else 0.0
    after_valid_ratio = float(np.mean(after_bundle.valid_depth_mask)) if after_bundle.valid_depth_mask.size else 0.0

    before_center_height, before_outer_height = _center_outer_regions(before_bundle.height_patch, config.alignment_search_px)
    after_center_height, after_outer_height = _center_outer_regions(after_bundle.height_patch, config.alignment_search_px)
    before_center_ir, before_outer_ir = _center_outer_regions(before_bundle.ir_patch, config.alignment_search_px)
    after_center_ir, after_outer_ir = _center_outer_regions(after_bundle.ir_patch, config.alignment_search_px)

    before_center_height_mean, _ = _mean_and_std(before_center_height)
    after_center_height_mean, _ = _mean_and_std(after_center_height)
    before_outer_height_mean, _ = _mean_and_std(before_outer_height)
    after_outer_height_mean, _ = _mean_and_std(after_outer_height)
    before_center_ir_mean, _ = _mean_and_std(before_center_ir)
    after_center_ir_mean, _ = _mean_and_std(after_center_ir)
    before_outer_ir_mean, _ = _mean_and_std(before_outer_ir)
    after_outer_ir_mean, _ = _mean_and_std(after_outer_ir)
    before_center_height_peak = float(np.max(before_center_height)) if before_center_height.size else 0.0
    after_center_height_peak = float(np.max(after_center_height)) if after_center_height.size else 0.0
    before_center_ir_peak = float(np.max(before_center_ir)) if before_center_ir.size else 0.0
    after_center_ir_peak = float(np.max(after_center_ir)) if after_center_ir.size else 0.0

    height_gain = max(0.0, after_center_height_mean - before_center_height_mean)
    peak_gain = max(0.0, after_center_height_peak - before_center_height_peak)
    ir_gain = max(0.0, after_center_ir_peak - before_center_ir_peak)
    ir_contrast_gain = max(0.0, (after_center_ir_mean - after_outer_ir_mean) - (before_center_ir_mean - before_outer_ir_mean))

    height_gain_score = _clip01(peak_gain / max(config.height_band_mm / 2.0, 1.0))
    peak_gain_score = _clip01(height_gain / max(config.height_band_mm / 2.5, 1.0))
    ir_gain_score = _clip01(ir_gain / max(config.min_ir_stddev * 10.0, 1.0))
    contrast_gain_score = _clip01(ir_contrast_gain / max(config.min_ir_stddev * 8.0, 1.0))
    quality_scale = _clip01(min(before_valid_ratio, after_valid_ratio) / max(config.min_valid_depth_ratio, 1e-6))

    raw_score = (
        0.50 * height_gain_score
        + 0.20 * peak_gain_score
        + 0.20 * ir_gain_score
        + 0.10 * contrast_gain_score
    )
    score = raw_score * (0.7 + 0.3 * quality_scale)
    metrics = {
        "before_valid_depth_ratio": before_valid_ratio,
        "after_valid_depth_ratio": after_valid_ratio,
        "height_gain_mm": height_gain,
        "peak_gain_mm": peak_gain,
        "ir_gain": ir_gain,
        "ir_contrast_gain": ir_contrast_gain,
        "before_center_height_peak": before_center_height_peak,
        "after_center_height_peak": after_center_height_peak,
        "before_center_ir_peak": before_center_ir_peak,
        "after_center_ir_peak": after_center_ir_peak,
        "before_center_height_mean": before_center_height_mean,
        "after_center_height_mean": after_center_height_mean,
        "before_outer_height_mean": before_outer_height_mean,
        "after_outer_height_mean": after_outer_height_mean,
        "before_center_ir_mean": before_center_ir_mean,
        "after_center_ir_mean": after_center_ir_mean,
        "before_outer_ir_mean": before_outer_ir_mean,
        "after_outer_ir_mean": after_outer_ir_mean,
        "quality_scale": quality_scale,
        "raw_score": raw_score,
    }
    return score, metrics


def verify_post_bind_rule(before_bundle: EvidenceBundle, after_bundle: EvidenceBundle, config: ClassificationConfig) -> RuleDecision:
    score, metrics = _score_post_bind(before_bundle, after_bundle, config)
    quality = float(min(before_bundle.evidence_quality, after_bundle.evidence_quality))
    if min(metrics["before_valid_depth_ratio"], metrics["after_valid_depth_ratio"]) < config.min_valid_depth_ratio:
        reason = (
            f"valid_depth_ratio below threshold: "
            f"before={metrics['before_valid_depth_ratio']:.2f}, after={metrics['after_valid_depth_ratio']:.2f}"
        )
        metrics["quality_reason"] = reason
        return RuleDecision(label="uncertain", score=score, quality=quality, reason=reason, metrics=metrics)

    if score >= config.post_bind_success_score_high:
        label = "success"
        reason = "post-bind height and infrared gains are strong"
    elif score <= config.post_bind_success_score_low:
        label = "failed"
        reason = "post-bind gains are too weak"
    else:
        label = "uncertain"
        reason = "score between post-bind thresholds"

    metrics["rule"] = "post_bind"
    return RuleDecision(label=label, score=score, quality=quality, reason=reason, metrics=metrics)


def append_classification_event(event_log_path: os.PathLike, event: Mapping[str, Any]) -> None:
    path = _as_path(event_log_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = dict(event)
    with path.open("a", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=True, sort_keys=True, default=str)
        handle.write("\n")


def persist_evidence_bundle(
    bundle: EvidenceBundle,
    config: ClassificationConfig,
    session_id: str,
) -> Path:
    bundle_dir = _as_path(config.evidence_root) / str(session_id) / f"point_{bundle.point_idx}"
    bundle_dir.mkdir(parents=True, exist_ok=True)

    ir_path = bundle_dir / "ir.png"
    depth_path = bundle_dir / "depth.npy"
    height_path = bundle_dir / "height.npy"
    raw_world_path = bundle_dir / "raw_world.npy"
    valid_mask_path = bundle_dir / "valid_depth_mask.npy"
    manifest_path = bundle_dir / "manifest.json"

    ir_image = np.asarray(bundle.ir_patch)
    if ir_image.dtype != np.uint8:
        ir_image = np.clip(ir_image, 0, 255).astype(np.uint8)
    cv2.imwrite(str(ir_path), ir_image)

    np.save(str(depth_path), np.asarray(bundle.depth_patch))
    np.save(str(height_path), np.asarray(bundle.height_patch))
    np.save(str(raw_world_path), np.asarray(bundle.raw_world_patch))
    np.save(str(valid_mask_path), np.asarray(bundle.valid_depth_mask))

    manifest = {
        "point_idx": bundle.point_idx,
        "phase": bundle.phase,
        "pix_coord": list(bundle.pix_coord),
        "world_coord": list(bundle.world_coord),
        "evidence_quality": bundle.evidence_quality,
        "files": {
            "ir_png": ir_path.name,
            "depth_npy": depth_path.name,
            "height_npy": height_path.name,
            "raw_world_npy": raw_world_path.name,
            "valid_depth_mask_npy": valid_mask_path.name,
        },
        "shapes": {
            "ir_patch": list(np.asarray(bundle.ir_patch).shape),
            "depth_patch": list(np.asarray(bundle.depth_patch).shape),
            "height_patch": list(np.asarray(bundle.height_patch).shape),
            "raw_world_patch": list(np.asarray(bundle.raw_world_patch).shape),
            "valid_depth_mask": list(np.asarray(bundle.valid_depth_mask).shape),
        },
    }
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=True, indent=2, sort_keys=True, default=str)
        handle.write("\n")

    return bundle_dir


__all__ = [
    "ClassificationConfig",
    "EvidenceBundle",
    "RuleDecision",
    "append_classification_event",
    "classify_pre_bind_rule",
    "extract_evidence_bundle",
    "load_classification_config",
    "persist_evidence_bundle",
    "verify_post_bind_rule",
]
