#!/usr/bin/env python3

"""Probe derived response modalities for rebar instance graph segmentation.

This tool is deliberately outside the pointAI runtime mainline. It archives the
current PR-FPRG result, then derives extra modalities from combined/depth
responses so we can judge whether they are useful for future rebar instance
segmentation and beam exclusion.
"""

from __future__ import annotations

import argparse
import html
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rospy


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src"
TOOL_DIR = Path(__file__).resolve().parent
for import_path in (PERCEPTION_SRC, TOOL_DIR):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from pr_fprg_peak_supported_probe import (  # noqa: E402
    capture_depth_ir_frame,
    capture_synced_frame,
    normalize_probe_raw_world,
    render_result,
    render_source_workspace,
    run_peak_supported_pr_fprg,
    to_bgr,
    to_u8,
)
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_structural_edge_suppression_mask,
    detect_workspace_s2_structural_edge_bands,
    expand_workspace_s2_exclusion_mask_by_metric_margin,
    normalize_workspace_s2_response,
)


def json_safe(value):
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, (float, int, str, bool)) or value is None:
        return value
    return str(value)


def normalize01(image, valid_mask=None, lower_percentile=2.0, upper_percentile=98.0):
    image = np.asarray(image, dtype=np.float32)
    normalized = np.zeros_like(image, dtype=np.float32)
    if image.size == 0:
        return normalized
    finite_mask = np.isfinite(image)
    if valid_mask is not None:
        finite_mask &= np.asarray(valid_mask).astype(bool)
    if not np.any(finite_mask):
        return normalized
    values = image[finite_mask]
    lower = float(np.percentile(values, lower_percentile))
    upper = float(np.percentile(values, upper_percentile))
    if upper <= lower + 1e-6:
        lower = float(np.min(values))
        upper = float(np.max(values))
    if upper <= lower + 1e-6:
        normalized[finite_mask] = 1.0
        return normalized
    normalized = np.clip((image - lower) / (upper - lower), 0.0, 1.0).astype(np.float32)
    normalized[~finite_mask] = 0.0
    return normalized


def apply_gamma_u8(image_u8, display_gamma):
    display_gamma = max(0.05, float(display_gamma or 1.0))
    image_u8 = np.asarray(image_u8, dtype=np.uint8)
    if abs(display_gamma - 1.0) <= 1e-6:
        return image_u8.copy()
    lookup = np.asarray(
        [
            np.clip(((value / 255.0) ** (1.0 / display_gamma)) * 255.0, 0.0, 255.0)
            for value in range(256)
        ],
        dtype=np.uint8,
    )
    return cv2.LUT(image_u8, lookup)


def render_heatmap(response, valid_mask=None, display_gamma=1.0):
    response_u8 = (normalize01(response, valid_mask) * 255.0).astype(np.uint8)
    response_u8 = apply_gamma_u8(response_u8, display_gamma)
    return cv2.applyColorMap(response_u8, cv2.COLORMAP_TURBO)


def render_binary(mask, label=None):
    mask = np.asarray(mask).astype(bool)
    canvas = np.zeros((*mask.shape[:2], 3), dtype=np.uint8)
    canvas[mask] = (255, 255, 255)
    contours, _hierarchy = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(canvas, contours, -1, (0, 180, 255), 1)
    if label:
        cv2.putText(canvas, label, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2, cv2.LINE_AA)
    return canvas


def build_depth_response(result):
    valid_mask = np.asarray(result["rectified_valid"]).astype(bool)
    filled_depth = np.asarray(result["filled_depth"], dtype=np.float32)
    background_depth = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=11.0, sigmaY=11.0)
    depth_response = background_depth - filled_depth
    return normalize_workspace_s2_response(depth_response, valid_mask)


def build_infrared_response(result):
    valid_mask = np.asarray(result["rectified_valid"]).astype(bool)
    rectified_ir = np.asarray(result["rectified_ir"], dtype=np.float32)
    background = cv2.GaussianBlur(rectified_ir, (0, 0), sigmaX=7.0, sigmaY=7.0)
    return normalize_workspace_s2_response(background - rectified_ir, valid_mask)


def build_combined_response(result, depth_weight=0.68):
    valid_mask = np.asarray(result["rectified_valid"]).astype(bool)
    if str(result.get("response_source", "")) == "depth_ir":
        return normalize_workspace_s2_response(np.asarray(result["response"], dtype=np.float32), valid_mask)
    depth_response = build_depth_response(result)
    infrared_response = build_infrared_response(result)
    depth_weight = float(np.clip(depth_weight, 0.0, 1.0))
    combined_response = (depth_weight * depth_response) + ((1.0 - depth_weight) * infrared_response)
    return normalize_workspace_s2_response(combined_response, valid_mask)


def build_worldCoord_height_response(result):
    # The Scepter worldCoord/raw_world_coord stream enters PR-FPRG as rectified depth.
    valid_mask = np.asarray(result["rectified_valid"]).astype(bool)
    rectified_depth = np.asarray(result["rectified_depth"], dtype=np.float32)
    if rectified_depth.ndim != 2:
        return np.zeros_like(valid_mask, dtype=np.float32)
    median_depth = float(np.median(rectified_depth[valid_mask])) if np.any(valid_mask) else 0.0
    filled = np.where(valid_mask, rectified_depth, median_depth).astype(np.float32)
    local_background = cv2.GaussianBlur(filled, (0, 0), sigmaX=23.0, sigmaY=23.0)
    return normalize_workspace_s2_response(local_background - filled, valid_mask)


def build_depth_gradient_response(result):
    valid_mask = np.asarray(result["rectified_valid"]).astype(bool)
    filled_depth = np.asarray(result["filled_depth"], dtype=np.float32)
    smoothed = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=1.2, sigmaY=1.2)
    grad_x = cv2.Sobel(smoothed, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(smoothed, cv2.CV_32F, 0, 1, ksize=3)
    gradient = cv2.magnitude(grad_x, grad_y)
    return normalize_workspace_s2_response(gradient, valid_mask)


def hessian_ridge_response(response_map, valid_mask, sigma=1.6):
    response_map = np.asarray(response_map, dtype=np.float32)
    valid_mask = np.asarray(valid_mask).astype(bool)
    blurred = cv2.GaussianBlur(response_map, (0, 0), sigmaX=float(sigma), sigmaY=float(sigma))
    dxx = cv2.Sobel(blurred, cv2.CV_32F, 2, 0, ksize=3)
    dyy = cv2.Sobel(blurred, cv2.CV_32F, 0, 2, ksize=3)
    dxy = cv2.Sobel(blurred, cv2.CV_32F, 1, 1, ksize=3)
    trace = dxx + dyy
    discriminant = np.sqrt(np.maximum(((dxx - dyy) ** 2) + (4.0 * dxy * dxy), 0.0))
    lambda_a = 0.5 * (trace + discriminant)
    lambda_b = 0.5 * (trace - discriminant)
    lambda_min = np.minimum(lambda_a, lambda_b)
    bright_ridge = np.maximum(-lambda_min, 0.0)
    ridge_response = bright_ridge * np.maximum(response_map, 0.0)
    return normalize_workspace_s2_response(ridge_response, valid_mask)


def multiscale_frangi_like_response(response_map, valid_mask, sigmas=(0.9, 1.6, 2.8, 4.2)):
    ridge_layers = [hessian_ridge_response(response_map, valid_mask, sigma=sigma) for sigma in sigmas]
    if not ridge_layers:
        return np.zeros_like(response_map, dtype=np.float32)
    ridge_stack = np.stack(ridge_layers, axis=0)
    frangi_like = np.max(ridge_stack, axis=0)
    return normalize_workspace_s2_response(frangi_like, valid_mask)


def threshold_response(response_map, valid_mask, percentile=83.0):
    response_map = np.asarray(response_map, dtype=np.float32)
    valid_mask = np.asarray(valid_mask).astype(bool)
    if not np.any(valid_mask):
        return np.zeros_like(response_map, dtype=bool), 0.0
    values = response_map[valid_mask]
    threshold = float(np.percentile(values, float(percentile)))
    binary = (response_map >= threshold) & valid_mask
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary_u8 = cv2.morphologyEx(binary.astype(np.uint8), cv2.MORPH_CLOSE, kernel, iterations=1)
    binary_u8 = cv2.morphologyEx(binary_u8, cv2.MORPH_OPEN, kernel, iterations=1)
    return binary_u8.astype(bool), threshold


def skeletonize_binary(binary_mask):
    binary_u8 = (np.asarray(binary_mask).astype(np.uint8) * 255)
    if binary_u8.size == 0:
        return binary_u8.astype(bool)
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        skeleton = cv2.ximgproc.thinning(binary_u8, cv2.ximgproc.THINNING_ZHANGSUEN)
        return skeleton > 0

    skeleton = np.zeros_like(binary_u8)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    working = binary_u8.copy()
    while True:
        eroded = cv2.erode(working, element)
        opened = cv2.dilate(eroded, element)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(working, opened))
        working = eroded
        if cv2.countNonZero(working) == 0:
            break
    return skeleton > 0


def count_skeleton_nodes(skeleton):
    skeleton = np.asarray(skeleton).astype(bool)
    if skeleton.size == 0:
        return 0, 0, np.zeros_like(skeleton, dtype=bool), np.zeros_like(skeleton, dtype=bool)
    neighbor_count = cv2.filter2D(
        skeleton.astype(np.uint8),
        cv2.CV_16S,
        np.ones((3, 3), dtype=np.uint8),
        borderType=cv2.BORDER_CONSTANT,
    )
    neighbor_count = neighbor_count - skeleton.astype(np.int16)
    endpoints = skeleton & (neighbor_count == 1)
    junctions = skeleton & (neighbor_count >= 3)
    return int(np.count_nonzero(endpoints)), int(np.count_nonzero(junctions)), endpoints, junctions


def build_instance_graph_overlay(base_response, skeleton, min_component_px=18):
    base = render_heatmap(base_response)
    skeleton = np.asarray(skeleton).astype(bool)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(skeleton.astype(np.uint8), 8)
    rng = np.random.default_rng(20260430)
    overlay = base.copy()
    component_summaries = []
    for label_index in range(1, num_labels):
        area = int(stats[label_index, cv2.CC_STAT_AREA])
        if area < int(min_component_px):
            continue
        color = rng.integers(80, 255, size=3, dtype=np.uint8).tolist()
        component_mask = labels == label_index
        visible_mask = cv2.dilate(component_mask.astype(np.uint8), np.ones((3, 3), dtype=np.uint8), iterations=1).astype(bool)
        overlay[visible_mask] = color
        centroid_x, centroid_y = centroids[label_index]
        cv2.putText(
            overlay,
            str(len(component_summaries) + 1),
            (int(round(centroid_x)), int(round(centroid_y))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        component_summaries.append(
            {
                "label": int(label_index),
                "area_px": area,
                "bbox": [
                    int(stats[label_index, cv2.CC_STAT_LEFT]),
                    int(stats[label_index, cv2.CC_STAT_TOP]),
                    int(stats[label_index, cv2.CC_STAT_WIDTH]),
                    int(stats[label_index, cv2.CC_STAT_HEIGHT]),
                ],
                "centroid": [float(centroid_x), float(centroid_y)],
            }
        )

    endpoint_count, junction_count, endpoints, junctions = count_skeleton_nodes(skeleton)
    overlay[endpoints] = (0, 255, 255)
    overlay[junctions] = (0, 0, 255)
    return overlay, {
        "component_count": len(component_summaries),
        "endpoint_count": endpoint_count,
        "junction_count": junction_count,
        "components": component_summaries,
    }


def profile_components(profile, threshold):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    active = profile >= float(threshold)
    components = []
    start = None
    for index, is_active in enumerate(active.tolist() + [False]):
        if is_active and start is None:
            start = index
        elif not is_active and start is not None:
            end = index - 1
            local = profile[start:end + 1]
            peak_index = int(start + int(np.argmax(local))) if local.size else int(start)
            components.append(
                {
                    "start": int(start),
                    "end": int(end),
                    "width": int(end - start + 1),
                    "peak": float(profile[peak_index]),
                    "peak_index": int(peak_index),
                }
            )
            start = None
    return components


def detect_beam_candidate_bands(candidate_response, binary_candidate, valid_mask):
    candidate_response = np.asarray(candidate_response, dtype=np.float32)
    binary_candidate = np.asarray(binary_candidate).astype(bool)
    valid_mask = np.asarray(valid_mask).astype(bool)
    bands = []
    if candidate_response.ndim != 2 or not np.any(valid_mask):
        return bands

    height, width = candidate_response.shape[:2]
    vertical_profile = np.mean(np.where(valid_mask, candidate_response, 0.0), axis=0)
    vertical_coverage = np.mean(binary_candidate & valid_mask, axis=0)
    structural_profile = normalize01(vertical_profile) + normalize01(vertical_coverage)
    structural_profile = cv2.GaussianBlur(structural_profile.reshape(1, -1), (0, 0), sigmaX=2.5).reshape(-1)
    threshold = max(float(np.percentile(structural_profile, 88.0)), 0.32)
    for component in profile_components(structural_profile, threshold):
        coverage = float(np.mean(binary_candidate[:, component["start"]:component["end"] + 1]))
        wide_enough = component["width"] >= max(9, int(round(width * 0.025)))
        tall_enough = coverage >= 0.42
        strong_enough = float(component["peak"]) >= 0.75
        if not (wide_enough and tall_enough and strong_enough):
            continue
        bands.append(
            {
                "axis": "x",
                "start": int(component["start"]),
                "end": int(component["end"]),
                "width": int(component["width"]),
                "peak": float(component["peak"]),
                "coverage": coverage,
                "type": "beam_candidate",
            }
        )

    return bands


def render_band_overlay(response_map, bands, title, color=(0, 0, 255)):
    image = render_heatmap(response_map)
    overlay = image.copy()
    height, width = image.shape[:2]
    for band in bands:
        if band.get("axis") == "y":
            y0 = int(np.clip(band["start"], 0, height - 1))
            y1 = int(np.clip(band["end"], 0, height - 1))
            cv2.rectangle(overlay, (0, y0), (width - 1, y1), color, -1)
            cv2.rectangle(image, (0, y0), (width - 1, y1), (255, 255, 255), 2)
        else:
            x0 = int(np.clip(band["start"], 0, width - 1))
            x1 = int(np.clip(band["end"], 0, width - 1))
            cv2.rectangle(overlay, (x0, 0), (x1, height - 1), color, -1)
            cv2.rectangle(image, (x0, 0), (x1, height - 1), (255, 255, 255), 2)
    image = cv2.addWeighted(overlay, 0.30, image, 0.70, 0.0)
    cv2.putText(image, title, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
    return image


def render_skeleton_overlay(response_map, skeleton):
    image = render_heatmap(response_map)
    skeleton = np.asarray(skeleton).astype(bool)
    thick = cv2.dilate(skeleton.astype(np.uint8), np.ones((3, 3), dtype=np.uint8), iterations=1).astype(bool)
    image[thick] = (0, 255, 80)
    _endpoint_count, _junction_count, endpoints, junctions = count_skeleton_nodes(skeleton)
    image[endpoints] = (0, 255, 255)
    image[junctions] = (0, 0, 255)
    return image


def load_latest_snapshot_frame(snapshot_dir=None):
    if snapshot_dir is None:
        candidates = sorted((WORKSPACE_ROOT / ".debug_frames").glob("rebar_instance_segmentation_modalities_*"))
        if not candidates:
            raise RuntimeError("no rebar_instance_segmentation_modalities_* snapshot available")
        snapshot_dir = candidates[-1]
    snapshot_dir = Path(snapshot_dir)
    arrays_dir = snapshot_dir / "arrays"
    raw_path = arrays_dir / "Scepter_worldCoord_raw_world_coord.npy"
    depth_path = arrays_dir / "Scepter_depth_image_raw.npy"
    ir_path = arrays_dir / "Scepter_ir_image_raw.npy"
    if not ir_path.exists():
        raise RuntimeError(f"snapshot missing IR array: {ir_path}")
    raw = np.load(raw_path) if raw_path.exists() else np.load(depth_path)
    ir = np.load(ir_path)
    world_path = arrays_dir / "Scepter_worldCoord_world_coord.npy"
    world = np.load(world_path) if world_path.exists() else None
    return {
        "world": world,
        "raw": raw,
        "ir": ir,
        "world_seq": None,
        "raw_seq": None,
        "ir_seq": None,
        "stamp": time.time(),
        "frame_source": f"snapshot:{snapshot_dir.name}",
    }


def capture_frame(timeout, snapshot_dir=None, allow_snapshot_fallback=True):
    try:
        return capture_synced_frame(timeout)
    except RuntimeError as exc:
        rospy.logwarn("PointAI_log: worldCoord 同步帧不可用，尝试 depth+IR：%s", exc)
    try:
        return capture_depth_ir_frame(timeout)
    except RuntimeError as exc:
        rospy.logwarn("PointAI_log: depth+IR 当前帧不可用：%s", exc)
        if not allow_snapshot_fallback:
            raise
    return load_latest_snapshot_frame(snapshot_dir)


def derive_modalities(result, threshold_percentile=83.0):
    valid_mask = np.asarray(result["rectified_valid"]).astype(bool)
    depth_response = build_depth_response(result)
    infrared_response = build_infrared_response(result)
    combined_response = build_combined_response(result)
    worldcoord_height_response = build_worldCoord_height_response(result)
    depth_gradient = build_depth_gradient_response(result)
    hessian_ridge = hessian_ridge_response(combined_response, valid_mask, sigma=1.6)
    frangi_like = multiscale_frangi_like_response(combined_response, valid_mask)
    fused_instance_response = normalize_workspace_s2_response(
        (0.50 * combined_response)
        + (0.22 * depth_response)
        + (0.16 * frangi_like)
        + (0.08 * depth_gradient)
        + (0.04 * worldcoord_height_response),
        valid_mask,
    )
    binary_candidate, binary_threshold = threshold_response(
        fused_instance_response,
        valid_mask,
        percentile=threshold_percentile,
    )
    skeleton = skeletonize_binary(binary_candidate)
    instance_overlay, instance_graph = build_instance_graph_overlay(fused_instance_response, skeleton)

    beam_candidate_bands = detect_beam_candidate_bands(fused_instance_response, binary_candidate, valid_mask)
    beam_candidate_mask = np.zeros_like(valid_mask, dtype=bool)
    for band in beam_candidate_bands:
        beam_candidate_mask[:, int(band["start"]):int(band["end"]) + 1] = True
    beam_candidate_mask_13cm = expand_workspace_s2_exclusion_mask_by_metric_margin(
        beam_candidate_mask,
        result.get("rectified_geometry"),
        margin_mm=130.0,
    )

    legacy_edge_bands = detect_workspace_s2_structural_edge_bands(combined_response, valid_mask)
    legacy_edge_mask = build_workspace_s2_structural_edge_suppression_mask(combined_response, valid_mask)
    legacy_edge_mask_13cm = expand_workspace_s2_exclusion_mask_by_metric_margin(
        legacy_edge_mask,
        result.get("rectified_geometry"),
        margin_mm=130.0,
    )

    return {
        "selected_response": np.asarray(result["response"], dtype=np.float32),
        "combined_response": combined_response,
        "depth_response": depth_response,
        "infrared_response": infrared_response,
        "worldCoord_height_response": worldcoord_height_response,
        "depth_gradient": depth_gradient,
        "hessian_ridge": hessian_ridge,
        "frangi_like": frangi_like,
        "fused_instance_response": fused_instance_response,
        "binary_candidate": binary_candidate,
        "binary_threshold": binary_threshold,
        "skeleton": skeleton,
        "instance_graph_overlay": instance_overlay,
        "instance_graph": instance_graph,
        "beam_candidate_bands": beam_candidate_bands,
        "beam_candidate_mask": beam_candidate_mask,
        "beam_candidate_mask_13cm": beam_candidate_mask_13cm,
        "legacy_edge_bands": legacy_edge_bands,
        "legacy_edge_mask": legacy_edge_mask,
        "legacy_edge_mask_13cm": legacy_edge_mask_13cm,
    }


def write_report(output_dir, frame, result, modalities, images, display_gamma):
    output_dir = Path(output_dir)
    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source", "raw_world"),
        "response_name": result.get("response_name"),
        "response_source": result.get("response_source"),
        "point_count": len(result.get("points", [])),
        "line_counts": [len(family.get("line_rhos", [])) for family in result.get("line_families", [])],
        "binary_threshold": float(modalities["binary_threshold"]),
        "instance_graph": modalities["instance_graph"],
        "beam_candidate_bands": modalities["beam_candidate_bands"],
        "beam_candidate_mask_13cm_pixels": int(np.count_nonzero(modalities["beam_candidate_mask_13cm"])),
        "legacy_edge_bands": modalities["legacy_edge_bands"],
        "legacy_edge_mask_13cm_pixels": int(np.count_nonzero(modalities["legacy_edge_mask_13cm"])),
        "display_gamma": float(display_gamma),
        "images": images,
        "note": "worldCoord enters this probe through rectified_depth/raw_world_coord when available.",
    }
    (output_dir / "summary.json").write_text(json.dumps(json_safe(summary), indent=2, ensure_ascii=False), encoding="utf-8")

    image_cards = [
        ("00_source_workspace.png", "当前帧工作区"),
        ("01_pr_fprg_result.png", "方案 1 PR-FPRG 当前输出"),
        ("02_selected_response.png", "主链选中的底层响应"),
        ("03_combined_response.png", "组合响应：深度 + 红外"),
        ("04_depth_response.png", "深度响应"),
        ("05_worldcoord_height_response.png", "worldCoord / 深度高度响应"),
        ("06_depth_gradient.png", "深度梯度"),
        ("07_hessian_ridge.png", "Hessian ridge"),
        ("08_frangi_like.png", "多尺度 Frangi-like 脊线"),
        ("09_fused_instance_response.png", "实例候选融合响应"),
        ("10_binary_candidate.png", "二值候选"),
        ("11_skeleton.png", "骨架 skeleton"),
        ("12_instance_graph_overlay.png", "instance_graph 叠加"),
        ("13_beam_candidate_overlay.png", "新梁筋候选 beam_candidate"),
        ("16_beam_candidate_13cm.png", "新梁筋候选 ±13 cm 排除区"),
        ("14_legacy_edge_band_overlay.png", "旧梁筋 edge-band mask 对照"),
        ("15_legacy_edge_band_13cm.png", "旧梁筋 ±13 cm 排除区"),
    ]
    cards_html = "\n".join(
        f"""
        <figure class="image-card">
          <img src="images/{html.escape(filename)}" alt="{html.escape(caption)}">
          <figcaption>{html.escape(caption)}</figcaption>
        </figure>
        """
        for filename, caption in image_cards
        if filename in images
    )
    beam_text = "，".join(
        f"x={band['start']}..{band['end']} width={band['width']} coverage={band['coverage']:.2f}"
        for band in modalities["beam_candidate_bands"]
    ) or "本帧未形成稳定 beam_candidate。"
    legacy_text = "，".join(
        f"{band['axis']}:{band['start']}..{band['end']} side={band.get('side', '-')}"
        for band in modalities["legacy_edge_bands"]
    ) or "旧 edge-band mask 本帧未检出。"

    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>钢筋实例图响应衍生探针</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: #18211d;
      background: #f4f1ea;
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
      line-height: 1.55;
      overflow-x: hidden;
    }}
    header {{
      padding: 22px clamp(16px, 4vw, 44px);
      border-bottom: 1px solid #d8d0c0;
      background: #fffaf0;
    }}
    h1 {{ margin: 0 0 6px; font-size: clamp(22px, 3vw, 36px); letter-spacing: 0; }}
    .subline {{ margin: 0; color: #5f675f; }}
    main {{ max-width: 1480px; margin: 0 auto; padding: 18px clamp(12px, 3vw, 34px) 42px; }}
    .facts {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 190px), 1fr));
      gap: 10px;
      margin: 16px 0 18px;
    }}
    .fact {{
      border: 1px solid #d8d0c0;
      border-radius: 8px;
      padding: 10px 12px;
      background: #fffdf7;
      min-width: 0;
    }}
    .fact b {{ display: block; font-size: 18px; }}
    section {{
      margin-top: 18px;
      padding-top: 18px;
      border-top: 1px solid #d8d0c0;
    }}
    h2 {{ margin: 0 0 10px; font-size: 20px; letter-spacing: 0; }}
    .note {{
      border-left: 4px solid #c4672d;
      background: #fff7e7;
      padding: 10px 12px;
      border-radius: 0 8px 8px 0;
      margin: 10px 0;
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 360px), 1fr));
      gap: 14px;
      align-items: start;
    }}
    .image-card {{
      margin: 0;
      border: 1px solid #d8d0c0;
      border-radius: 8px;
      background: #fffdf7;
      overflow: hidden;
      min-width: 0;
    }}
    .image-card img {{
      display: block;
      width: 100%;
      max-height: min(72vh, 900px);
      object-fit: contain;
      background: #10140f;
    }}
    figcaption {{
      padding: 9px 10px;
      font-size: 14px;
      color: #384239;
      border-top: 1px solid #e6dfd1;
    }}
    code {{
      background: #ebe3d3;
      border-radius: 4px;
      padding: 1px 4px;
    }}
    @media (max-width: 720px) {{
      header {{ padding: 16px; }}
      main {{ padding: 12px; }}
      .image-card img {{ max-height: 70vh; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>钢筋实例图响应衍生探针</h1>
    <p class="subline">组合响应和深度响应继续往下衍射：Hessian、Frangi-like、骨架、实例图、梁筋候选。</p>
  </header>
  <main>
    <div class="facts">
      <div class="fact"><span>帧来源</span><b>{html.escape(str(summary["frame_source"]))}</b></div>
      <div class="fact"><span>响应</span><b>{html.escape(str(summary["response_name"]))}</b></div>
      <div class="fact"><span>当前点数</span><b>{summary["point_count"]}</b></div>
      <div class="fact"><span>线族数</span><b>{html.escape(str(summary["line_counts"]))}</b></div>
      <div class="fact"><span>骨架组件</span><b>{summary["instance_graph"]["component_count"]}</b></div>
      <div class="fact"><span>梁筋候选</span><b>{len(summary["beam_candidate_bands"])}</b></div>
    </div>
    <section>
      <h2>判断</h2>
      <p class="note">这不是替换主链的结果图。它是实例分割前的多模态探针：看哪些派生图能稳定把普通钢筋、梁筋、地板缝和 TCP 侵入拆开。</p>
      <p>新 beam_candidate：{html.escape(beam_text)}</p>
      <p>旧梁筋 edge-band mask 对照：{html.escape(legacy_text)}。注意：旧方法不是实例分割，只能作为本报告的对照层。</p>
    </section>
    <section>
      <h2>图像</h2>
      <div class="grid">
        {cards_html}
      </div>
    </section>
  </main>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--output-dir", default=str(WORKSPACE_ROOT / ".debug_frames" / f"rebar_instance_graph_probe_{time.strftime('%Y%m%d_%H%M%S')}"))
    parser.add_argument("--snapshot-dir", default=None)
    parser.add_argument("--no-snapshot-fallback", action="store_true")
    parser.add_argument("--threshold-percentile", type=float, default=83.0)
    parser.add_argument("--display-gamma", type=float, default=1.35)
    parser.add_argument("--response-name-filter", default="combined_depth_ir_darkline")
    args = parser.parse_args()

    rospy.init_node("rebar_instance_graph_probe", anonymous=True, disable_signals=True)
    frame = capture_frame(
        args.timeout,
        snapshot_dir=args.snapshot_dir,
        allow_snapshot_fallback=not args.no_snapshot_fallback,
    )
    raw_world, used_depth_fallback_raw_world = normalize_probe_raw_world(frame["raw"])
    frame["raw"] = raw_world
    frame["used_depth_fallback_raw_world"] = bool(used_depth_fallback_raw_world)

    result = run_peak_supported_pr_fprg(frame, response_name_filter=args.response_name_filter)
    modalities = derive_modalities(result, threshold_percentile=args.threshold_percentile)

    output_dir = Path(args.output_dir)
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)
    images = {}

    image_builders = {
        "00_source_workspace.png": render_source_workspace(frame, result),
        "01_pr_fprg_result.png": render_result(frame, result),
        "02_selected_response.png": render_heatmap(modalities["selected_response"], result["rectified_valid"], args.display_gamma),
        "03_combined_response.png": render_heatmap(modalities["combined_response"], result["rectified_valid"], args.display_gamma),
        "04_depth_response.png": render_heatmap(modalities["depth_response"], result["rectified_valid"], args.display_gamma),
        "05_worldcoord_height_response.png": render_heatmap(modalities["worldCoord_height_response"], result["rectified_valid"], args.display_gamma),
        "06_depth_gradient.png": render_heatmap(modalities["depth_gradient"], result["rectified_valid"], args.display_gamma),
        "07_hessian_ridge.png": render_heatmap(modalities["hessian_ridge"], result["rectified_valid"], args.display_gamma),
        "08_frangi_like.png": render_heatmap(modalities["frangi_like"], result["rectified_valid"], args.display_gamma),
        "09_fused_instance_response.png": render_heatmap(modalities["fused_instance_response"], result["rectified_valid"], args.display_gamma),
        "10_binary_candidate.png": render_binary(modalities["binary_candidate"], "binary candidate"),
        "11_skeleton.png": render_skeleton_overlay(modalities["fused_instance_response"], modalities["skeleton"]),
        "12_instance_graph_overlay.png": modalities["instance_graph_overlay"],
        "07_skeleton.png": render_skeleton_overlay(modalities["fused_instance_response"], modalities["skeleton"]),
        "08_instance_graph_overlay.png": modalities["instance_graph_overlay"],
        "13_beam_candidate_overlay.png": render_band_overlay(
            modalities["fused_instance_response"],
            modalities["beam_candidate_bands"],
            "beam_candidate",
            color=(0, 0, 255),
        ),
        "16_beam_candidate_13cm.png": render_binary(modalities["beam_candidate_mask_13cm"], "beam_candidate +/-13cm"),
        "14_legacy_edge_band_overlay.png": render_band_overlay(
            modalities["combined_response"],
            modalities["legacy_edge_bands"],
            "legacy edge-band mask",
            color=(0, 128, 255),
        ),
        "15_legacy_edge_band_13cm.png": render_binary(modalities["legacy_edge_mask_13cm"], "legacy edge-band +/-13cm"),
    }

    for filename, image in image_builders.items():
        path = image_dir / filename
        cv2.imwrite(str(path), image)
        images[filename] = str(path)

    write_report(output_dir, frame, result, modalities, images, args.display_gamma)
    rospy.loginfo("PointAI_log: 钢筋实例图响应衍生探针输出目录：%s", output_dir)


if __name__ == "__main__":
    main()
