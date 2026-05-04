#!/usr/bin/env python3

"""Compare rebar-surface segmentation and bind-point detection variants.

The comparison is intentionally report-only. It reuses the current PR-FPRG
scheme-1 grid, then applies derived multimodal surface segmentation masks and
beam_candidate exclusion masks to show what should, and should not, move into
the runtime mainline later.
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
    normalize_probe_raw_world,
    render_result,
    run_peak_supported_pr_fprg,
    to_bgr,
    to_u8,
)
from rebar_instance_graph_probe import (  # noqa: E402
    capture_frame,
    count_skeleton_nodes,
    derive_modalities,
    json_safe,
    render_band_overlay,
    render_binary,
    render_heatmap,
    render_skeleton_overlay,
)
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_axis_aligned_line_families,
    build_workspace_s2_curved_line_families,
    intersect_workspace_s2_curved_line_families,
    intersect_workspace_s2_oriented_line_families,
    normalize_workspace_s2_response,
)


def mask_value_at(mask, point):
    mask = np.asarray(mask).astype(bool)
    if mask.ndim != 2 or mask.size == 0:
        return False
    x_value, y_value = float(point[0]), float(point[1])
    x_index = int(round(x_value))
    y_index = int(round(y_value))
    if x_index < 0 or y_index < 0 or y_index >= mask.shape[0] or x_index >= mask.shape[1]:
        return False
    return bool(mask[y_index, x_index])


def response_value_at(response_map, point):
    response_map = np.asarray(response_map, dtype=np.float32)
    if response_map.ndim != 2 or response_map.size == 0:
        return 0.0
    x_index = int(round(float(point[0])))
    y_index = int(round(float(point[1])))
    if x_index < 0 or y_index < 0 or y_index >= response_map.shape[0] or x_index >= response_map.shape[1]:
        return 0.0
    return float(response_map[y_index, x_index])


def project_rectified_points_to_image(rectified_points, inverse_h):
    rectified_points = np.asarray(rectified_points, dtype=np.float32).reshape(-1, 1, 2)
    if rectified_points.size == 0:
        return []
    inverse_h = np.asarray(inverse_h, dtype=np.float32)
    image_points = cv2.perspectiveTransform(rectified_points, inverse_h).reshape(-1, 2)
    return [[float(x_value), float(y_value)] for x_value, y_value in image_points]


def apply_gamma_u8(image_u8, gamma):
    gamma = max(0.05, float(gamma or 1.0))
    image_u8 = np.asarray(image_u8, dtype=np.uint8)
    if abs(gamma - 1.0) <= 1e-6:
        return image_u8.copy()
    lookup = np.asarray(
        [np.clip(((value / 255.0) ** (1.0 / gamma)) * 255.0, 0.0, 255.0) for value in range(256)],
        dtype=np.uint8,
    )
    return cv2.LUT(image_u8, lookup)


def render_ir_high_gamma(frame, result=None, ir_display_gamma=1.85, draw_label=True):
    image = to_bgr(apply_gamma_u8(to_u8(frame["ir"]), ir_display_gamma))
    if result is not None:
        polygon = np.asarray(result["manual_workspace"]["corner_pixels"], dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(image, [polygon], True, (235, 235, 235), 2)
    if draw_label:
        cv2.putText(
            image,
            f"ir_high_gamma gamma={float(ir_display_gamma):.2f}",
            (18, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return image


def clip_line_segment_to_rect(family, line_rho, width, height):
    normal = np.asarray(family.get("normal"), dtype=np.float32).reshape(2)
    line_angle_rad = np.deg2rad(float(family.get("line_angle_deg", 0.0)))
    direction = np.asarray([np.cos(line_angle_rad), np.sin(line_angle_rad)], dtype=np.float32)
    point_on_line = normal * float(line_rho)
    t_min = -np.inf
    t_max = np.inf
    for axis_index, (lower_bound, upper_bound) in enumerate(((0.0, float(width - 1)), (0.0, float(height - 1)))):
        origin_value = float(point_on_line[axis_index])
        direction_value = float(direction[axis_index])
        if abs(direction_value) <= 1e-6:
            if origin_value < lower_bound or origin_value > upper_bound:
                return None
            continue
        t0 = (lower_bound - origin_value) / direction_value
        t1 = (upper_bound - origin_value) / direction_value
        t_min = max(t_min, min(t0, t1))
        t_max = min(t_max, max(t0, t1))
        if t_max < t_min:
            return None
    if not np.isfinite(t_min) or not np.isfinite(t_max):
        return None
    start = point_on_line + (direction * float(t_min))
    end = point_on_line + (direction * float(t_max))
    return (
        (int(round(float(start[0]))), int(round(float(start[1])))),
        (int(round(float(end[0]))), int(round(float(end[1])))),
    )


def draw_line_family_mask(shape, line_families, thickness_px=5):
    height, width = shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    for family in line_families or []:
        for line_rho in family.get("line_rhos", []):
            segment = clip_line_segment_to_rect(family, line_rho, width, height)
            if segment is None:
                continue
            cv2.line(mask, segment[0], segment[1], 1, max(1, int(thickness_px)), cv2.LINE_AA)
    return mask.astype(bool)


def points_from_rectified_points(result, rectified_points, response_map, variant_id, exclusion_mask=None):
    filtered_rectified = []
    for rectified in rectified_points:
        if mask_value_at(exclusion_mask, rectified):
            continue
        filtered_rectified.append([float(rectified[0]), float(rectified[1])])
    image_points = project_rectified_points_to_image(filtered_rectified, result["rectified_geometry"]["inverse_h"])
    points = []
    for rectified, image_point in zip(filtered_rectified, image_points):
        points.append(
            {
                "idx": len(points) + 1,
                "rectified": rectified,
                "pix": image_point,
                "score": response_value_at(response_map, rectified),
                "source": variant_id,
            }
        )
    return points


def pr_fprg_points(result, modalities):
    response = modalities["fused_instance_response"]
    points = []
    for point in result.get("points", []):
        rectified = [float(point["rectified"][0]), float(point["rectified"][1])]
        points.append(
            {
                "idx": int(point["idx"]),
                "rectified": rectified,
                "pix": [float(point["pix"][0]), float(point["pix"][1])],
                "score": response_value_at(response, rectified),
                "source": "pr_fprg_all",
            }
        )
    return points


def filter_points_outside_mask(points, exclusion_mask, variant_id):
    filtered = []
    for point in points:
        if mask_value_at(exclusion_mask, point["rectified"]):
            continue
        copied = dict(point)
        copied["idx"] = len(filtered) + 1
        copied["source"] = variant_id
        filtered.append(copied)
    return filtered


def cluster_instance_graph_junctions(result, modalities, exclusion_mask=None, min_area_px=2):
    skeleton = np.asarray(modalities["skeleton"]).astype(bool)
    _endpoint_count, _junction_count, _endpoints, junctions = count_skeleton_nodes(skeleton)
    if exclusion_mask is not None:
        junctions = junctions & ~np.asarray(exclusion_mask).astype(bool)
    if not np.any(junctions):
        return []

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    clustered = cv2.dilate(junctions.astype(np.uint8), kernel, iterations=1)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(clustered, 8)
    rectified_points = []
    response = modalities["fused_instance_response"]
    for label_index in range(1, num_labels):
        area = int(stats[label_index, cv2.CC_STAT_AREA])
        if area < int(min_area_px):
            continue
        center_x, center_y = centroids[label_index]
        rectified = [float(center_x), float(center_y)]
        if mask_value_at(exclusion_mask, rectified):
            continue
        rectified_points.append(
            {
                "idx": len(rectified_points) + 1,
                "rectified": rectified,
                "score": response_value_at(response, rectified),
                "source": "instance_graph_junctions",
            }
        )

    image_points = project_rectified_points_to_image(
        [point["rectified"] for point in rectified_points],
        result["rectified_geometry"]["inverse_h"],
    )
    for point, image_point in zip(rectified_points, image_points):
        point["pix"] = image_point
    return rectified_points


def build_completed_surface_intersections(result, surface_segmentation, modalities, exclusion_mask=None):
    line_families = surface_segmentation.get("completed_line_families") or result.get("line_families", [])
    if len(line_families) < 2:
        return []
    rectified_points = intersect_workspace_s2_oriented_line_families(
        line_families[0],
        line_families[1],
        result["rectified_geometry"]["rectified_width"],
        result["rectified_geometry"]["rectified_height"],
    )
    return points_from_rectified_points(
        result,
        rectified_points,
        surface_segmentation["completed_surface_response"],
        "completed_surface_intersections",
        exclusion_mask=exclusion_mask,
    )


def build_curve3456_on_completed_surface(result, modalities, surface_segmentation, exclusion_mask=None):
    curve3456_on_completed_surface = []
    base_line_families = surface_segmentation.get("completed_line_families") or result.get("line_families", [])
    if len(base_line_families) < 2:
        return curve3456_on_completed_surface

    curve_specs = [
        (
            "03_surface_greedy_curve",
            "03 补全面 greedy 曲线",
            "以补全钢筋面作为底图，沿补全后的线族拓扑做局部 ridge 贪心曲线，再求曲线交点。",
            surface_segmentation["completed_surface_response"],
            "greedy",
            "response",
            0.08,
        ),
        (
            "04_surface_dp_curve",
            "04 补全面 DP 曲线",
            "以补全钢筋面作为底图，动态规划约束相邻采样点偏移连续。",
            surface_segmentation["completed_surface_response"],
            "dynamic_programming",
            "response",
            0.12,
        ),
        (
            "05_surface_ridge_curve",
            "05 补全面 ridge 曲线",
            "以补全钢筋面作为底图，动态规划分数加入中间强、两侧弱的 ridge 判断。",
            surface_segmentation["completed_surface_response"],
            "dynamic_programming",
            "response_ridge",
            0.12,
        ),
        (
            "06_surface_ir_assisted_curve",
            "06 组合响应辅助曲线",
            "以组合响应、Frangi-like、补全面共同形成的多模态底图做曲线收束。",
            normalize_workspace_s2_response(
                (0.45 * modalities["combined_response"])
                + (0.30 * surface_segmentation["completed_surface_response"])
                + (0.25 * modalities["frangi_like"]),
                result["rectified_valid"].astype(bool),
            ),
            "dynamic_programming",
            "response_ridge",
            0.12,
        ),
    ]
    for variant_id, title, note, response_map, trace_method, score_mode, smoothness_weight in curve_specs:
        curved_families = build_workspace_s2_curved_line_families(
            response_map,
            result["rectified_valid"].astype(np.uint8),
            base_line_families,
            trace_method=trace_method,
            score_mode=score_mode,
            search_radius_px=9,
            sample_step_px=4,
            min_response_ratio=0.15,
            smoothing_window_samples=5,
            smoothness_weight=smoothness_weight,
        )
        rectified_points = intersect_workspace_s2_curved_line_families(
            curved_families[0],
            curved_families[1],
            result["rectified_geometry"]["rectified_width"],
            result["rectified_geometry"]["rectified_height"],
        )
        if not rectified_points:
            rectified_points = intersect_workspace_s2_oriented_line_families(
                base_line_families[0],
                base_line_families[1],
                result["rectified_geometry"]["rectified_width"],
                result["rectified_geometry"]["rectified_height"],
            )
            note = f"{note}；本帧曲线交点不足时使用补全直线交点兜底。"
        points = points_from_rectified_points(
            result,
            rectified_points,
            response_map,
            variant_id,
            exclusion_mask=exclusion_mask,
        )
        curve3456_on_completed_surface.append(
            {
                "id": variant_id,
                "title": title,
                "points": points,
                "point_count": len(points),
                "mean_score": float(np.mean([point["score"] for point in points])) if points else 0.0,
                "note": note,
                "curved_families": curved_families,
            }
        )
    return curve3456_on_completed_surface


def build_bind_point_variants(result, modalities, surface_segmentation):
    started = time.perf_counter()
    base_points = pr_fprg_points(result, modalities)
    variant_timings_ms = {}
    bind_point_variants = []

    def add_variant(variant_id, title, points, note):
        bind_point_variants.append(
            {
                "id": variant_id,
                "title": title,
                "points": points,
                "point_count": len(points),
                "mean_score": float(np.mean([point["score"] for point in points])) if points else 0.0,
                "note": note,
            }
        )

    direct_beam_exclusion_mask = surface_segmentation.get("beam_candidate_body_mask", modalities["beam_candidate_mask"])

    add_variant(
        "pr_fprg_all",
        "PR-FPRG 全部交点",
        base_points,
        "不做梁筋过滤，用作当前方案 1 对照。",
    )
    variant_timings_ms["pr_fprg_all"] = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    add_variant(
        "legacy_edge_band_13cm",
        "旧 edge-band ±13cm",
        filter_points_outside_mask(base_points, modalities["legacy_edge_mask_13cm"], "legacy_edge_band_13cm"),
        "旧梁筋 edge-band mask 的保守排除；本帧只抓到左侧梁筋。",
    )
    variant_timings_ms["legacy_edge_band_13cm"] = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    add_variant(
        "beam_candidate_direct",
        "新 beam_candidate 本体",
        filter_points_outside_mask(base_points, direct_beam_exclusion_mask, "beam_candidate_direct"),
        "只去掉落在梁筋本体膨胀 mask 上的绑扎点，保留梁筋附近普通钢筋交点。",
    )
    variant_timings_ms["beam_candidate_direct"] = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    add_variant(
        "beam_candidate_13cm",
        "新 beam_candidate ±13cm",
        filter_points_outside_mask(base_points, modalities["beam_candidate_mask_13cm"], "beam_candidate_13cm"),
        "按梁筋左右 ±13cm 做图谱排除，更保守，但会少保留梁筋附近交点。",
    )
    variant_timings_ms["beam_candidate_13cm"] = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    add_variant(
        "completed_surface_intersections",
        "补全钢筋面交点",
        build_completed_surface_intersections(
            result,
            surface_segmentation,
            modalities,
            exclusion_mask=direct_beam_exclusion_mask,
        ),
        "先补全钢筋面，再从补全面重建行/列线族并直接求交点；这是本轮推荐技术路径。",
    )
    variant_timings_ms["completed_surface_intersections"] = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    add_variant(
        "instance_graph_junctions",
        "instance_graph 骨架交叉点",
        cluster_instance_graph_junctions(result, modalities, exclusion_mask=direct_beam_exclusion_mask, min_area_px=2),
        "不依赖理论网格交点，直接从骨架 junction 聚类取点；当前作为探索对照。",
    )
    variant_timings_ms["instance_graph_junctions"] = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    curve_variants = build_curve3456_on_completed_surface(
        result,
        modalities,
        surface_segmentation,
        exclusion_mask=direct_beam_exclusion_mask,
    )
    bind_point_variants.extend(curve_variants)
    variant_timings_ms["curve3456_on_completed_surface"] = (time.perf_counter() - started) * 1000.0

    return bind_point_variants, variant_timings_ms


def build_surface_segmentation(result, modalities):
    surface_segmentation = {}
    binary_candidate = np.asarray(modalities["binary_candidate"]).astype(bool)
    beam_candidate_mask = np.asarray(modalities["beam_candidate_mask"]).astype(bool)
    beam_candidate_mask_13cm = np.asarray(modalities["beam_candidate_mask_13cm"]).astype(bool)
    valid_mask = np.asarray(result["rectified_valid"]).astype(bool)
    body_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (19, 19))
    beam_candidate_body_mask = cv2.dilate(beam_candidate_mask.astype(np.uint8), body_kernel, iterations=1).astype(bool) & valid_mask
    line_support_mask = draw_line_family_mask(
        binary_candidate.shape,
        result.get("line_families", []),
        thickness_px=5,
    )
    completed_surface_mask = (binary_candidate | line_support_mask) & valid_mask
    completed_surface_response = normalize_workspace_s2_response(
        (0.56 * modalities["fused_instance_response"])
        + (0.26 * modalities["frangi_like"])
        + (0.18 * line_support_mask.astype(np.float32)),
        valid_mask,
    )
    completed_line_families = build_workspace_s2_axis_aligned_line_families(
        completed_surface_response,
        valid_mask.astype(np.uint8),
        min_period=10,
        max_period=30,
        enable_local_peak_refine=True,
        enable_peak_support=True,
        enable_continuous_validation=False,
        enable_spacing_prune=True,
        enable_structural_edge_suppression=False,
        peak_min_ratio=0.22,
    )
    if len(completed_line_families) < 2:
        completed_line_families = result.get("line_families", [])
    ordinary_rebar_mask = binary_candidate & ~beam_candidate_body_mask
    ordinary_completed_rebar_mask = completed_surface_mask & ~beam_candidate_body_mask
    margin_only_mask = beam_candidate_mask_13cm & ~beam_candidate_body_mask
    surface_segmentation["ordinary_rebar_mask"] = ordinary_rebar_mask
    surface_segmentation["ordinary_completed_rebar_mask"] = ordinary_completed_rebar_mask
    surface_segmentation["line_support_mask"] = line_support_mask
    surface_segmentation["completed_surface_mask"] = completed_surface_mask
    surface_segmentation["completed_surface_response"] = completed_surface_response
    surface_segmentation["completed_line_families"] = completed_line_families[:2]
    surface_segmentation["beam_candidate_mask"] = beam_candidate_mask
    surface_segmentation["beam_candidate_body_mask"] = beam_candidate_body_mask
    surface_segmentation["beam_candidate_mask_13cm"] = beam_candidate_mask_13cm
    surface_segmentation["beam_margin_only_mask"] = margin_only_mask
    surface_segmentation["stats"] = {
        "ordinary_rebar_pixels": int(np.count_nonzero(ordinary_rebar_mask)),
        "ordinary_completed_rebar_pixels": int(np.count_nonzero(ordinary_completed_rebar_mask)),
        "completed_surface_pixels": int(np.count_nonzero(completed_surface_mask)),
        "line_support_pixels": int(np.count_nonzero(line_support_mask)),
        "completed_line_counts": [
            len(family.get("line_rhos", []))
            for family in surface_segmentation["completed_line_families"]
        ],
        "beam_candidate_pixels": int(np.count_nonzero(beam_candidate_mask)),
        "beam_candidate_body_pixels": int(np.count_nonzero(beam_candidate_body_mask)),
        "beam_candidate_13cm_pixels": int(np.count_nonzero(beam_candidate_mask_13cm)),
        "surface_candidate_pixels": int(np.count_nonzero(binary_candidate)),
    }
    return surface_segmentation


def render_surface_segmentation_overlay(modalities, surface_segmentation):
    base = render_heatmap(modalities["combined_response"])
    overlay = base.copy()
    ordinary = surface_segmentation["ordinary_rebar_mask"]
    beam = surface_segmentation.get("beam_candidate_body_mask", surface_segmentation["beam_candidate_mask"])
    margin = surface_segmentation["beam_margin_only_mask"]

    overlay[ordinary] = (40, 220, 150)
    overlay[margin] = (0, 210, 255)
    overlay[beam] = (40, 40, 230)
    image = cv2.addWeighted(overlay, 0.58, base, 0.42, 0.0)

    skeleton = np.asarray(modalities["skeleton"]).astype(bool)
    thick = cv2.dilate(skeleton.astype(np.uint8), np.ones((3, 3), dtype=np.uint8), iterations=1).astype(bool)
    image[thick & ordinary] = (255, 255, 255)
    contours, _hierarchy = cv2.findContours(beam.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(image, contours, -1, (255, 255, 255), 2)
    cv2.putText(image, "surface_segmentation", (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "green=ordinary rebar  red=beam  yellow=+/-13cm", (16, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def render_surface_completion_overlay(modalities, surface_segmentation):
    base = render_heatmap(surface_segmentation["completed_surface_response"])
    overlay = base.copy()
    completed = surface_segmentation["ordinary_completed_rebar_mask"]
    original = surface_segmentation["ordinary_rebar_mask"]
    support = surface_segmentation["line_support_mask"] & ~surface_segmentation["ordinary_rebar_mask"]
    beam = surface_segmentation.get("beam_candidate_body_mask", surface_segmentation["beam_candidate_mask"])
    overlay[completed] = (70, 220, 150)
    overlay[support] = (255, 210, 70)
    overlay[beam] = (50, 50, 230)
    image = cv2.addWeighted(overlay, 0.56, base, 0.44, 0.0)
    contours, _hierarchy = cv2.findContours(original.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(image, contours, -1, (255, 255, 255), 1)
    cv2.putText(image, "completed_surface_mask", (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "green=completed  yellow=line-fill  red=beam", (16, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def draw_curved_families_on_rectified(image, variant):
    curved_families = variant.get("curved_families") or []
    colors = [(255, 90, 0), (0, 180, 255)]
    for family_index, family in enumerate(curved_families):
        color = colors[family_index % len(colors)]
        for curved_line in family.get("curved_lines", []):
            polyline = np.asarray(curved_line.get("polyline_points", []), dtype=np.float32).reshape(-1, 2)
            if polyline.shape[0] < 2:
                continue
            cv2.polylines(image, [np.rint(polyline).astype(np.int32)], False, color, 1, cv2.LINE_AA)
    return image


def render_rectified_variant(modalities, surface_segmentation, variant):
    image = render_surface_completion_overlay(modalities, surface_segmentation)
    image = draw_curved_families_on_rectified(image, variant)
    for point in variant["points"]:
        x_value, y_value = point["rectified"]
        center = (int(round(x_value)), int(round(y_value)))
        cv2.circle(image, center, 5, (0, 255, 255), -1)
        cv2.circle(image, center, 7, (20, 40, 40), 1)
        cv2.putText(image, str(point["idx"]), (center[0] + 6, center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, variant["id"], (16, image.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return image


def render_original_variant(frame, result, variant, ir_display_gamma=1.85):
    image = render_ir_high_gamma(frame, result, ir_display_gamma=ir_display_gamma, draw_label=False)
    polygon = np.asarray(result["manual_workspace"]["corner_pixels"], dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [polygon], True, (235, 235, 235), 2)
    for point in variant["points"]:
        x_value, y_value = point["pix"]
        center = (int(round(x_value)), int(round(y_value)))
        cv2.circle(image, center, 5, (0, 255, 255), -1)
        cv2.circle(image, center, 8, (0, 115, 255), 2)
        cv2.putText(image, str(point["idx"]), (center[0] + 7, center[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)
    label = f"{variant['id']}  points={variant['point_count']}"
    cv2.putText(image, label[:54], (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, label[:54], (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def build_montage(image_paths, output_path, captions, tile_width=430, tile_height=360):
    tiles = []
    for path, caption in zip(image_paths, captions):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        height, width = image.shape[:2]
        scale = min((tile_width - 20) / max(1, width), (tile_height - 58) / max(1, height))
        new_size = (max(1, int(round(width * scale))), max(1, int(round(height * scale))))
        resized = cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)
        tile = np.full((tile_height, tile_width, 3), (246, 241, 231), dtype=np.uint8)
        x0 = (tile_width - new_size[0]) // 2
        tile[10:10 + new_size[1], x0:x0 + new_size[0]] = resized
        cv2.putText(tile, caption[:62], (14, tile_height - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (38, 45, 40), 1, cv2.LINE_AA)
        tiles.append(tile)
    if not tiles:
        return None
    cols = 2
    rows = int(np.ceil(len(tiles) / float(cols)))
    montage = np.full((rows * tile_height, cols * tile_width, 3), (246, 241, 231), dtype=np.uint8)
    for index, tile in enumerate(tiles):
        row = index // cols
        col = index % cols
        montage[row * tile_height:(row + 1) * tile_height, col * tile_width:(col + 1) * tile_width] = tile
    cv2.imwrite(str(output_path), montage)
    return output_path


def write_report(output_dir, frame, result, modalities, surface_segmentation, bind_point_variants, variant_timings_ms, timings_ms, images, ir_display_gamma):
    output_dir = Path(output_dir)
    recommended = next(
        (variant for variant in bind_point_variants if variant["id"] == "completed_surface_intersections" and variant["point_count"] >= 40),
        next((variant for variant in bind_point_variants if variant["id"] == "beam_candidate_direct"), bind_point_variants[0]),
    )
    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source", "raw_world"),
        "response_name": result.get("response_name"),
        "line_counts": [len(family.get("line_rhos", [])) for family in result.get("line_families", [])],
        "ir_display_gamma": float(ir_display_gamma),
        "surface_segmentation": surface_segmentation["stats"],
        "beam_candidate_bands": modalities["beam_candidate_bands"],
        "legacy_edge_bands": modalities["legacy_edge_bands"],
        "bind_point_variants": [
            {
                "id": variant["id"],
                "title": variant["title"],
                "point_count": variant["point_count"],
                "mean_score": variant["mean_score"],
                "note": variant["note"],
            }
            for variant in bind_point_variants
        ],
        "recommended_variant": {
            "id": recommended["id"],
            "point_count": recommended["point_count"],
            "reason": "本轮优先走“补全钢筋面后直接检测交点”的技术路径；梁筋本体点仍用 beam_candidate 排除。",
        },
        "variant_timings_ms": variant_timings_ms,
        "timings_ms": timings_ms,
        "images": images,
    }
    (output_dir / "summary.json").write_text(json.dumps(json_safe(summary), indent=2, ensure_ascii=False), encoding="utf-8")

    rows_html = "\n".join(
        f"""
        <tr>
          <td><code>{html.escape(variant['id'])}</code></td>
          <td>{html.escape(variant['title'])}</td>
          <td>{variant['point_count']}</td>
          <td>{variant['mean_score']:.3f}</td>
          <td>{html.escape(variant['note'])}</td>
        </tr>
        """
        for variant in bind_point_variants
    )
    image_cards = [
        ("ir_high_gamma_workspace.png", "高伽马 IR 原图：展示用，便于看清原图钢筋"),
        ("surface_segmentation_overlay.png", "钢筋面分割：普通钢筋 / 梁筋 / 梁筋 ±13cm"),
        ("surface_completion_overlay.png", "补全钢筋面：completed_surface_mask"),
        ("completed_surface_response.png", "补全面响应：多模态 + 线族补全"),
        ("completed_surface_mask.png", "补全面二值 mask"),
        ("beam_candidate_overlay.png", "新梁筋候选 beam_candidate"),
        ("beam_candidate_body_mask.png", "梁筋本体膨胀 mask：direct 过滤实际使用"),
        ("legacy_edge_band_overlay.png", "旧 edge-band mask 对照"),
        ("instance_graph_overlay.png", "instance_graph 骨架实例图"),
        ("bindpoint_comparison_matrix.png", "绑扎点检测方案矩阵"),
    ]
    for variant in bind_point_variants:
        image_cards.append((f"{variant['id']}_original.png", f"{variant['title']} 原图点位"))
        image_cards.append((f"{variant['id']}_rectified.png", f"{variant['title']} rectified 点位"))

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
    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>钢筋面分割与绑扎点检测对比</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: #17211d;
      background: #f5f1e8;
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
      line-height: 1.55;
      overflow-x: hidden;
    }}
    header {{
      padding: 22px clamp(16px, 4vw, 42px);
      background: #fbf7ee;
      border-bottom: 1px solid #d9d0c2;
    }}
    h1 {{ margin: 0 0 6px; font-size: clamp(22px, 3vw, 34px); letter-spacing: 0; }}
    .subline {{ margin: 0; color: #5b655d; }}
    main {{ max-width: 1520px; margin: 0 auto; padding: 16px clamp(12px, 3vw, 34px) 42px; }}
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
    section {{ margin-top: 18px; padding-top: 18px; border-top: 1px solid #d8d0c0; }}
    h2 {{ margin: 0 0 10px; font-size: 20px; letter-spacing: 0; }}
    table {{ width: 100%; border-collapse: collapse; background: #fffdf7; border: 1px solid #d8d0c0; border-radius: 8px; overflow: hidden; }}
    th, td {{ padding: 9px 10px; border-bottom: 1px solid #e4dccd; vertical-align: top; text-align: left; }}
    th {{ background: #eee6d5; }}
    code {{ background: #ece3d2; border-radius: 4px; padding: 1px 4px; }}
    .note {{ border-left: 4px solid #c4672d; background: #fff8e8; padding: 10px 12px; border-radius: 0 8px 8px 0; }}
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
      max-height: min(74vh, 940px);
      object-fit: contain;
      background: #10140f;
    }}
    figcaption {{ padding: 9px 10px; color: #384239; border-top: 1px solid #e6dfd1; }}
    @media (max-width: 720px) {{
      header {{ padding: 16px; }}
      main {{ padding: 12px; }}
      table {{ font-size: 13px; }}
      .image-card img {{ max-height: 70vh; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>钢筋面分割与绑扎点检测对比</h1>
    <p class="subline">用组合响应 + 深度响应 + beam_candidate 做分割，再比较不同绑扎点过滤口径。</p>
  </header>
  <main>
    <div class="facts">
      <div class="fact"><span>帧来源</span><b>{html.escape(str(summary['frame_source']))}</b></div>
      <div class="fact"><span>响应</span><b>{html.escape(str(summary['response_name']))}</b></div>
      <div class="fact"><span>线族</span><b>{html.escape(str(summary['line_counts']))}</b></div>
      <div class="fact"><span>推荐方案</span><b>{html.escape(summary['recommended_variant']['id'])}</b></div>
      <div class="fact"><span>推荐点数</span><b>{summary['recommended_variant']['point_count']}</b></div>
      <div class="fact"><span>全流程耗时</span><b>{summary['timings_ms']['total']:.1f} ms</b></div>
    </div>
    <section>
      <h2>结论</h2>
      <p class="note">{html.escape(summary['recommended_variant']['reason'])}</p>
      <p>展示图的 IR 底图已提高伽马：<code>ir_display_gamma={summary['ir_display_gamma']:.2f}</code>。红色是 beam_candidate 梁筋本体，黄色是梁筋 ±13cm 排除带，绿色/白色是普通钢筋候选和骨架。</p>
      <p>补全面路径：原始 surface_segmentation 可能缺线，所以先用 line support 补成 <code>completed_surface_mask</code>，再从补全面重建行列线族并直接求交点。</p>
    </section>
    <section>
      <h2>绑扎点方案对比</h2>
      <table>
        <thead><tr><th>方案</th><th>名称</th><th>点数</th><th>均值响应</th><th>说明</th></tr></thead>
        <tbody>{rows_html}</tbody>
      </table>
    </section>
    <section>
      <h2>效果图</h2>
      <div class="grid">{cards_html}</div>
    </section>
  </main>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--output-dir", default=str(WORKSPACE_ROOT / ".debug_frames" / f"rebar_surface_bindpoint_comparison_{time.strftime('%Y%m%d_%H%M%S')}"))
    parser.add_argument("--snapshot-dir", default=None)
    parser.add_argument("--no-snapshot-fallback", action="store_true")
    parser.add_argument("--threshold-percentile", type=float, default=83.0)
    parser.add_argument("--ir-display-gamma", type=float, default=1.85)
    parser.add_argument("--response-name-filter", default="combined_depth_ir_darkline")
    args = parser.parse_args()

    rospy.init_node("rebar_surface_bindpoint_comparison", anonymous=True, disable_signals=True)
    total_start = time.perf_counter()

    started = time.perf_counter()
    frame = capture_frame(
        args.timeout,
        snapshot_dir=args.snapshot_dir,
        allow_snapshot_fallback=not args.no_snapshot_fallback,
    )
    raw_world, used_depth_fallback_raw_world = normalize_probe_raw_world(frame["raw"])
    frame["raw"] = raw_world
    frame["used_depth_fallback_raw_world"] = bool(used_depth_fallback_raw_world)
    capture_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    result = run_peak_supported_pr_fprg(frame, response_name_filter=args.response_name_filter)
    pr_fprg_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    modalities = derive_modalities(result, threshold_percentile=args.threshold_percentile)
    surface_segmentation = build_surface_segmentation(result, modalities)
    segmentation_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    bind_point_variants, variant_timings_ms = build_bind_point_variants(result, modalities, surface_segmentation)
    variants_ms = (time.perf_counter() - started) * 1000.0

    output_dir = Path(args.output_dir)
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)
    images = {}

    image_builders = {
        "ir_high_gamma_workspace.png": render_ir_high_gamma(frame, result, ir_display_gamma=args.ir_display_gamma),
        "surface_segmentation_overlay.png": render_surface_segmentation_overlay(modalities, surface_segmentation),
        "surface_completion_overlay.png": render_surface_completion_overlay(modalities, surface_segmentation),
        "completed_surface_response.png": render_heatmap(surface_segmentation["completed_surface_response"]),
        "completed_surface_mask.png": render_binary(surface_segmentation["completed_surface_mask"], "completed_surface_mask"),
        "ordinary_rebar_mask.png": render_binary(surface_segmentation["ordinary_rebar_mask"], "ordinary_rebar_mask"),
        "beam_candidate_mask.png": render_binary(surface_segmentation["beam_candidate_mask"], "beam_candidate_mask"),
        "beam_candidate_body_mask.png": render_binary(surface_segmentation["beam_candidate_body_mask"], "beam_candidate_body_mask"),
        "beam_candidate_mask_13cm.png": render_binary(surface_segmentation["beam_candidate_mask_13cm"], "beam_candidate_mask_13cm"),
        "beam_candidate_overlay.png": render_band_overlay(modalities["fused_instance_response"], modalities["beam_candidate_bands"], "beam_candidate"),
        "legacy_edge_band_overlay.png": render_band_overlay(modalities["combined_response"], modalities["legacy_edge_bands"], "legacy edge-band mask", color=(0, 128, 255)),
        "instance_graph_overlay.png": render_skeleton_overlay(modalities["fused_instance_response"], modalities["skeleton"]),
        "pr_fprg_reference.png": render_result(frame, result),
    }
    for variant in bind_point_variants:
        image_builders[f"{variant['id']}_original.png"] = render_original_variant(
            frame,
            result,
            variant,
            ir_display_gamma=args.ir_display_gamma,
        )
        image_builders[f"{variant['id']}_rectified.png"] = render_rectified_variant(modalities, surface_segmentation, variant)

    for filename, image in image_builders.items():
        path = image_dir / filename
        cv2.imwrite(str(path), image)
        images[filename] = str(path)

    matrix_path = image_dir / "bindpoint_comparison_matrix.png"
    build_montage(
        [image_dir / f"{variant['id']}_original.png" for variant in bind_point_variants],
        matrix_path,
        [f"{variant['id']} points={variant['point_count']}" for variant in bind_point_variants],
    )
    images["bindpoint_comparison_matrix.png"] = str(matrix_path)

    timings_ms = {
        "capture": capture_ms,
        "pr_fprg": pr_fprg_ms,
        "segmentation": segmentation_ms,
        "variants": variants_ms,
        "total": (time.perf_counter() - total_start) * 1000.0,
    }
    write_report(
        output_dir,
        frame,
        result,
        modalities,
        surface_segmentation,
        bind_point_variants,
        variant_timings_ms,
        timings_ms,
        images,
        args.ir_display_gamma,
    )
    rospy.loginfo("PointAI_log: 钢筋面分割与绑扎点检测对比报告输出目录：%s", output_dir)


if __name__ == "__main__":
    main()
