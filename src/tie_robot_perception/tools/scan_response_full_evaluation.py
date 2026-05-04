#!/usr/bin/env python3

"""Offline full evaluation for scan response maps and bind-point variants.

This is a report-only tool. It loads a fixed visual snapshot and compares the
current scan-style depth response against combined response, Hessian ridge,
Frangi-like, fused instance response, completed rebar surface and skeleton
junction variants. Runtime nodes are not changed by this script.
"""

from __future__ import annotations

import argparse
import html
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src"
TOOL_DIR = Path(__file__).resolve().parent
for import_path in (PERCEPTION_SRC, TOOL_DIR):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from pr_fprg_peak_supported_probe import (  # noqa: E402
    get_valid_world_coord_near_pixel,
    load_manual_workspace_quad,
    map_workspace_s2_rectified_points_to_image,
    normalize_probe_raw_world,
    render_result,
    run_peak_supported_pr_fprg,
    to_bgr,
    to_u8,
)
from rebar_instance_graph_probe import (  # noqa: E402
    derive_modalities,
    json_safe,
    load_latest_snapshot_frame,
    render_binary,
    render_heatmap,
    render_skeleton_overlay,
)
from rebar_surface_bindpoint_comparison import (  # noqa: E402
    build_completed_surface_intersections,
    build_curve3456_on_completed_surface,
    build_montage,
    build_surface_segmentation,
    cluster_instance_graph_junctions,
    points_from_rectified_points,
    render_ir_high_gamma,
    render_original_variant,
    render_rectified_variant,
    render_surface_completion_overlay,
    render_surface_segmentation_overlay,
    response_value_at,
)
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_axis_aligned_line_families,
    build_workspace_s2_axis_profile,
    build_workspace_s2_line_positions,
    build_workspace_s2_projective_line_segments,
    build_workspace_s2_rectified_geometry,
    estimate_workspace_s2_period_and_phase,
    intersect_workspace_s2_oriented_line_families,
    normalize_workspace_s2_response,
)


def load_snapshot_frame(snapshot_dir: str | Path):
    frame = load_latest_snapshot_frame(Path(snapshot_dir))
    raw_world, used_depth_fallback_raw_world = normalize_probe_raw_world(frame["raw"])
    frame["raw"] = raw_world
    frame["used_depth_fallback_raw_world"] = bool(used_depth_fallback_raw_world)
    return frame


def build_workspace_mask(shape, corner_pixels):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [np.asarray(corner_pixels, dtype=np.int32).reshape((-1, 1, 2))], 1)
    return mask


def build_depth_runtime_result(frame):
    """Mimic the current MODE_SCAN_ONLY depth-only S2 runtime path offline."""
    manual_workspace = load_manual_workspace_quad()
    corner_pixels = manual_workspace["corner_pixels"]
    corner_world = manual_workspace.get("corner_world_cabin_frame")
    if corner_world is None:
        corner_world = manual_workspace.get("corner_world_camera_frame")
    raw_world = np.asarray(frame["raw"], dtype=np.float32)
    ir = to_u8(frame["ir"])
    workspace_mask = build_workspace_mask(ir.shape[:2], corner_pixels)
    geometry = build_workspace_s2_rectified_geometry(corner_pixels, corner_world)
    rectified_size = (geometry["rectified_width"], geometry["rectified_height"])

    raw_depth = raw_world[:, :, 2].astype(np.float32)
    valid_mask = np.isfinite(raw_depth) & (raw_depth != 0.0)
    rectified_depth = cv2.warpPerspective(
        raw_depth,
        geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_LINEAR,
    ).astype(np.float32)
    rectified_valid = cv2.warpPerspective(
        valid_mask.astype(np.uint8),
        geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_NEAREST,
    ).astype(bool)
    median_depth = float(np.median(rectified_depth[rectified_valid]))
    filled_depth = np.where(rectified_valid, rectified_depth, median_depth).astype(np.float32)
    background_depth = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=11.0, sigmaY=11.0)

    best_variant = None
    for response_name, response_map in (
        ("current_depth_background_minus_filled", background_depth - filled_depth),
        ("current_depth_filled_minus_background", filled_depth - background_depth),
    ):
        normalized = normalize_workspace_s2_response(response_map.astype(np.float32), rectified_valid)
        vertical_profile = build_workspace_s2_axis_profile(normalized, rectified_valid.astype(np.uint8), axis=0)
        horizontal_profile = build_workspace_s2_axis_profile(normalized, rectified_valid.astype(np.uint8), axis=1)
        vertical_estimate = estimate_workspace_s2_period_and_phase(vertical_profile, min_period=10, max_period=30)
        horizontal_estimate = estimate_workspace_s2_period_and_phase(horizontal_profile, min_period=10, max_period=30)
        if vertical_estimate is None or horizontal_estimate is None:
            continue
        combined_score = float(vertical_estimate["score"] + horizontal_estimate["score"])
        if best_variant is None or combined_score > best_variant["combined_score"]:
            best_variant = {
                "response_name": response_name,
                "response": normalized,
                "vertical_estimate": vertical_estimate,
                "horizontal_estimate": horizontal_estimate,
                "combined_score": combined_score,
            }

    if best_variant is None:
        raise RuntimeError("current depth-only runtime estimate failed")

    vertical_lines = build_workspace_s2_line_positions(
        0,
        geometry["rectified_width"] - 1,
        best_variant["vertical_estimate"]["period"],
        best_variant["vertical_estimate"]["phase"],
    )
    horizontal_lines = build_workspace_s2_line_positions(
        0,
        geometry["rectified_height"] - 1,
        best_variant["horizontal_estimate"]["period"],
        best_variant["horizontal_estimate"]["phase"],
    )
    rectified_points = [
        [float(x_value), float(y_value)]
        for y_value in horizontal_lines
        for x_value in vertical_lines
    ]
    image_points = map_workspace_s2_rectified_points_to_image(rectified_points, geometry["inverse_h"])

    points = []
    for image_point, rectified_point in zip(image_points, rectified_points):
        pixel_x, pixel_y = int(image_point[0]), int(image_point[1])
        if (
            pixel_x < 0
            or pixel_y < 0
            or pixel_y >= workspace_mask.shape[0]
            or pixel_x >= workspace_mask.shape[1]
            or not workspace_mask[pixel_y, pixel_x]
        ):
            continue
        world_coord, sample_pixel, used_fallback = get_valid_world_coord_near_pixel(raw_world, pixel_x, pixel_y)
        if any(float(value) == 0.0 for value in world_coord):
            continue
        points.append(
            {
                "idx": len(points) + 1,
                "pix": [int(pixel_x), int(pixel_y)],
                "rectified": [float(rectified_point[0]), float(rectified_point[1])],
                "world": world_coord,
                "sample_pix": sample_pixel,
                "used_fallback": bool(used_fallback),
                "score": response_value_at(best_variant["response"], rectified_point),
                "source": "current_depth_only_runtime",
            }
        )

    line_families = [
        {
            "line_angle_deg": 90.0,
            "normal": [1.0, 0.0],
            "line_rhos": [float(value) for value in vertical_lines],
        },
        {
            "line_angle_deg": 0.0,
            "normal": [0.0, 1.0],
            "line_rhos": [float(value) for value in horizontal_lines],
        },
    ]
    return {
        "manual_workspace": manual_workspace,
        "workspace_mask": workspace_mask,
        "rectified_geometry": geometry,
        "rectified_valid": rectified_valid,
        "rectified_depth": rectified_depth,
        "filled_depth": filled_depth,
        "response": best_variant["response"],
        "response_name": best_variant["response_name"],
        "response_source": "depth_runtime",
        "line_families": line_families,
        "line_segments": build_workspace_s2_projective_line_segments(
            corner_pixels,
            geometry["rectified_width"],
            geometry["rectified_height"],
            vertical_lines,
            horizontal_lines,
        ),
        "line_stages": {},
        "points": points,
        "vertical_estimate": best_variant["vertical_estimate"],
        "horizontal_estimate": best_variant["horizontal_estimate"],
    }


def build_axis_family_points(result, response_map, variant_id):
    line_families = build_workspace_s2_axis_aligned_line_families(
        response_map,
        result["rectified_valid"].astype(np.uint8),
        min_period=10,
        max_period=30,
        enable_local_peak_refine=True,
        enable_peak_support=True,
        enable_continuous_validation=False,
        enable_spacing_prune=True,
        enable_structural_edge_suppression=False,
        peak_min_ratio=0.22,
    )
    if len(line_families) < 2:
        return [], []
    line_families = line_families[:2]
    rectified_points = intersect_workspace_s2_oriented_line_families(
        line_families[0],
        line_families[1],
        result["rectified_geometry"]["rectified_width"],
        result["rectified_geometry"]["rectified_height"],
    )
    points = points_from_rectified_points(result, rectified_points, response_map, variant_id)
    return points, line_families


def nearest_distance_stats(points):
    if len(points) < 2:
        return {"min": None, "median": None, "close_pairs_lt_8px": 0}
    coords = [np.asarray(point["rectified"], dtype=np.float32) for point in points]
    distances = []
    close_pairs = 0
    for index, coord in enumerate(coords):
        nearest = None
        for other_index, other in enumerate(coords):
            if index == other_index:
                continue
            distance = float(np.linalg.norm(coord - other))
            if nearest is None or distance < nearest:
                nearest = distance
            if other_index > index and distance < 8.0:
                close_pairs += 1
        if nearest is not None:
            distances.append(nearest)
    return {
        "min": float(min(distances)) if distances else None,
        "median": float(np.median(distances)) if distances else None,
        "close_pairs_lt_8px": int(close_pairs),
    }


def point_mean_score(points, response_map):
    if not points:
        return 0.0
    return float(np.mean([response_value_at(response_map, point["rectified"]) for point in points]))


def mask_support_ratio(points, mask):
    if not points:
        return 0.0
    mask = np.asarray(mask).astype(bool)
    supported = 0
    for point in points:
        x_value, y_value = point["rectified"]
        x_index = int(round(float(x_value)))
        y_index = int(round(float(y_value)))
        if 0 <= y_index < mask.shape[0] and 0 <= x_index < mask.shape[1] and mask[y_index, x_index]:
            supported += 1
    return float(supported) / float(len(points))


def quality_score(row):
    count = float(row["point_count"])
    count_penalty = min(1.0, abs(count - 64.0) / 64.0)
    close_penalty = min(1.0, float(row["close_pairs_lt_8px"]) / max(1.0, count))
    dense_penalty = 0.0 if count <= 96.0 else min(1.0, (count - 96.0) / 128.0)
    score = (
        0.30 * row["mean_fused"]
        + 0.20 * row["mean_hessian"]
        + 0.16 * row["mean_frangi"]
        + 0.16 * row["mean_completed_surface"]
        + 0.10 * row["binary_support_ratio"]
        + 0.08 * (1.0 - count_penalty)
        - 0.10 * close_penalty
        - 0.12 * dense_penalty
    )
    return float(score)


def summarize_variant(variant, modalities, surface_segmentation):
    points = variant["points"]
    distance_stats = nearest_distance_stats(points)
    line_counts = [len(family.get("line_rhos", [])) for family in variant.get("line_families", [])[:2]]
    row = {
        "id": variant["id"],
        "title": variant["title"],
        "point_count": len(points),
        "line_counts": line_counts,
        "mean_combined": point_mean_score(points, modalities["combined_response"]),
        "mean_hessian": point_mean_score(points, modalities["hessian_ridge"]),
        "mean_frangi": point_mean_score(points, modalities["frangi_like"]),
        "mean_fused": point_mean_score(points, modalities["fused_instance_response"]),
        "mean_completed_surface": point_mean_score(points, surface_segmentation["completed_surface_response"]),
        "binary_support_ratio": mask_support_ratio(points, modalities["binary_candidate"]),
        "nearest_median_px": distance_stats["median"],
        "nearest_min_px": distance_stats["min"],
        "close_pairs_lt_8px": distance_stats["close_pairs_lt_8px"],
        "note": variant.get("note", ""),
    }
    row["quality_score"] = quality_score(row)
    return row


def build_evaluation(frame, threshold_percentile):
    current_result = build_depth_runtime_result(frame)
    combined_result = run_peak_supported_pr_fprg(frame, response_name_filter="combined_depth_ir_darkline")
    depth_result = run_peak_supported_pr_fprg(frame, response_name_filter="depth_background_minus_filled")
    modalities = derive_modalities(combined_result, threshold_percentile=threshold_percentile)
    surface_segmentation = build_surface_segmentation(combined_result, modalities)

    variants = [
        {
            "id": "current_depth_only_runtime",
            "title": "当前扫描 depth-only 运行逻辑",
            "points": current_result["points"],
            "line_families": current_result["line_families"],
            "response_map": current_result["response"],
            "note": "复刻 MODE_SCAN_ONLY 当前 depth-only 周期/相位铺网格逻辑。",
        },
        {
            "id": "combined_pr_fprg_peak_supported",
            "title": "组合响应 PR-FPRG 线族",
            "points": [
                {
                    **point,
                    "score": response_value_at(modalities["combined_response"], point["rectified"]),
                    "source": "combined_pr_fprg_peak_supported",
                }
                for point in combined_result["points"]
            ],
            "line_families": combined_result["line_families"],
            "response_map": modalities["combined_response"],
            "note": "以 combined_depth_ir_darkline 作为底图的峰值支撑线族。",
        },
        {
            "id": "depth_pr_fprg_peak_supported",
            "title": "深度响应 PR-FPRG 线族",
            "points": [
                {
                    **point,
                    "score": response_value_at(modalities["depth_response"], point["rectified"]),
                    "source": "depth_pr_fprg_peak_supported",
                }
                for point in depth_result["points"]
            ],
            "line_families": depth_result["line_families"],
            "response_map": modalities["depth_response"],
            "note": "以 depth_background_minus_filled 作为底图的峰值支撑线族。",
        },
    ]

    for variant_id, title, response_key, note in (
        ("hessian_axis_family", "Hessian ridge 线族", "hessian_ridge", "直接在 Hessian ridge 底图上估行列线族。"),
        ("frangi_axis_family", "Frangi-like 线族", "frangi_like", "直接在多尺度 Frangi-like 脊线底图上估行列线族。"),
        ("fused_instance_axis_family", "融合实例响应线族", "fused_instance_response", "直接在组合响应 + 深度 + Frangi 融合底图上估行列线族。"),
    ):
        points, line_families = build_axis_family_points(combined_result, modalities[response_key], variant_id)
        variants.append(
            {
                "id": variant_id,
                "title": title,
                "points": points,
                "line_families": line_families,
                "response_map": modalities[response_key],
                "note": note,
            }
        )

    completed_points = build_completed_surface_intersections(
        combined_result,
        surface_segmentation,
        modalities,
        exclusion_mask=surface_segmentation.get("beam_candidate_body_mask"),
    )
    variants.append(
        {
            "id": "completed_surface_intersections",
            "title": "补全钢筋面交点",
            "points": completed_points,
            "line_families": surface_segmentation["completed_line_families"],
            "response_map": surface_segmentation["completed_surface_response"],
            "note": "先做实例候选分割，再用线族支撑补全钢筋面后求交点。",
        }
    )

    instance_points = cluster_instance_graph_junctions(
        combined_result,
        modalities,
        exclusion_mask=surface_segmentation.get("beam_candidate_body_mask"),
        min_area_px=2,
    )
    variants.append(
        {
            "id": "instance_graph_junctions_raw",
            "title": "实例骨架 junction 原始聚类",
            "points": instance_points,
            "line_families": [],
            "response_map": modalities["fused_instance_response"],
            "note": "直接聚类骨架 junction；召回强但容易过检，建议只作为候选源。",
        }
    )

    curve_variants = build_curve3456_on_completed_surface(
        combined_result,
        modalities,
        surface_segmentation,
        exclusion_mask=surface_segmentation.get("beam_candidate_body_mask"),
    )
    for curve_variant in curve_variants:
        variants.append(
            {
                "id": curve_variant["id"],
                "title": curve_variant["title"],
                "points": curve_variant["points"],
                "line_families": curve_variant.get("curved_families", []),
                "response_map": surface_segmentation["completed_surface_response"],
                "note": curve_variant["note"],
            }
        )

    summary_rows = [summarize_variant(variant, modalities, surface_segmentation) for variant in variants]
    summary_rows.sort(key=lambda item: item["quality_score"], reverse=True)
    return {
        "current_result": current_result,
        "combined_result": combined_result,
        "depth_result": depth_result,
        "modalities": modalities,
        "surface_segmentation": surface_segmentation,
        "variants": variants,
        "summary_rows": summary_rows,
    }


def draw_points_rectified(base_response, points, title):
    image = render_heatmap(base_response)
    for point in points:
        x_value, y_value = point["rectified"]
        center = (int(round(float(x_value))), int(round(float(y_value))))
        cv2.circle(image, center, 5, (0, 255, 255), -1)
        cv2.circle(image, center, 7, (30, 40, 40), 1)
        cv2.putText(image, str(point["idx"]), (center[0] + 6, center[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, title[:72], (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return image


def safe_filename(name):
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in name)


def write_report(output_dir, frame, evaluation, timings_ms, ir_display_gamma):
    output_dir = Path(output_dir)
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)

    combined_result = evaluation["combined_result"]
    current_result = evaluation["current_result"]
    modalities = evaluation["modalities"]
    surface_segmentation = evaluation["surface_segmentation"]
    variants = evaluation["variants"]
    summary_rows = evaluation["summary_rows"]
    variant_by_id = {variant["id"]: variant for variant in variants}

    image_builders = {
        "00_input_high_gamma_workspace.png": render_ir_high_gamma(frame, combined_result, ir_display_gamma=ir_display_gamma),
        "01_current_depth_only_reference.png": render_result(frame, current_result),
        "02_combined_pr_fprg_reference.png": render_result(frame, combined_result),
        "03_combined_response.png": render_heatmap(modalities["combined_response"]),
        "04_hessian_ridge.png": render_heatmap(modalities["hessian_ridge"]),
        "05_frangi_like.png": render_heatmap(modalities["frangi_like"]),
        "06_fused_instance_response.png": render_heatmap(modalities["fused_instance_response"]),
        "07_binary_candidate.png": render_binary(modalities["binary_candidate"], "binary_candidate"),
        "08_instance_graph_overlay.png": render_skeleton_overlay(modalities["fused_instance_response"], modalities["skeleton"]),
        "09_surface_segmentation_overlay.png": render_surface_segmentation_overlay(modalities, surface_segmentation),
        "10_surface_completion_overlay.png": render_surface_completion_overlay(modalities, surface_segmentation),
        "11_completed_surface_response.png": render_heatmap(surface_segmentation["completed_surface_response"]),
    }

    for variant in variants:
        filename_prefix = safe_filename(variant["id"])
        if variant["id"] == "current_depth_only_runtime":
            image_builders[f"variant_{filename_prefix}_original.png"] = render_result(frame, current_result)
        else:
            point_variant = {
                "id": variant["id"],
                "title": variant["title"],
                "points": variant["points"],
                "point_count": len(variant["points"]),
            }
            image_builders[f"variant_{filename_prefix}_original.png"] = render_original_variant(
                frame,
                combined_result,
                point_variant,
                ir_display_gamma=ir_display_gamma,
            )
        image_builders[f"variant_{filename_prefix}_rectified.png"] = draw_points_rectified(
            variant["response_map"],
            variant["points"],
            f"{variant['id']} points={len(variant['points'])}",
        )

    image_paths = {}
    for filename, image in image_builders.items():
        path = image_dir / filename
        cv2.imwrite(str(path), image)
        image_paths[filename] = str(path)

    top_variant_ids = [row["id"] for row in summary_rows[:8]]
    matrix_path = image_dir / "90_top_variant_matrix.png"
    build_montage(
        [image_dir / f"variant_{safe_filename(variant_id)}_original.png" for variant_id in top_variant_ids],
        matrix_path,
        [
            f"{variant_id} points={len(variant_by_id[variant_id]['points'])}"
            for variant_id in top_variant_ids
        ],
    )
    image_paths["90_top_variant_matrix.png"] = str(matrix_path)

    recommended = next(
        (
            row
            for row in summary_rows
            if row["id"] in {
                "completed_surface_intersections",
                "04_surface_dp_curve",
                "05_surface_ridge_curve",
                "06_surface_ir_assisted_curve",
                "combined_pr_fprg_peak_supported",
            }
        ),
        summary_rows[0],
    )
    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "snapshot_frame_source": frame.get("frame_source"),
        "snapshot_uses_depth_fallback_raw_world": bool(frame.get("used_depth_fallback_raw_world", False)),
        "threshold_percentile": float(evaluation["modalities"]["binary_threshold"]),
        "current_depth_runtime": {
            "response_name": current_result["response_name"],
            "line_counts": [len(family.get("line_rhos", [])) for family in current_result["line_families"]],
            "point_count": len(current_result["points"]),
            "vertical_period_px": int(current_result["vertical_estimate"]["period"]),
            "horizontal_period_px": int(current_result["horizontal_estimate"]["period"]),
        },
        "combined_reference": {
            "response_name": combined_result["response_name"],
            "line_counts": [len(family.get("line_rhos", [])) for family in combined_result["line_families"]],
            "point_count": len(combined_result["points"]),
        },
        "surface_segmentation": surface_segmentation["stats"],
        "instance_graph": modalities["instance_graph"],
        "beam_candidate_bands": modalities["beam_candidate_bands"],
        "recommended_variant": {
            "id": recommended["id"],
            "title": recommended["title"],
            "point_count": recommended["point_count"],
            "quality_score": recommended["quality_score"],
            "reason": "离线代理指标下响应支撑强、点数不膨胀、重复近邻少；运行建议作为几何主候选，实例骨架只做验证/补召回。",
        },
        "variant_rows": summary_rows,
        "timings_ms": timings_ms,
        "images": image_paths,
        "metric_note": "No manual ground truth labels are available in this snapshot. Scores are proxy metrics using response support, skeleton support, count stability and close-duplicate penalty.",
    }
    (output_dir / "summary.json").write_text(json.dumps(json_safe(summary), indent=2, ensure_ascii=False), encoding="utf-8")

    table_rows = "\n".join(
        f"""
        <tr>
          <td><code>{html.escape(row['id'])}</code></td>
          <td>{html.escape(row['title'])}</td>
          <td>{row['point_count']}</td>
          <td>{html.escape(str(row['line_counts']))}</td>
          <td>{row['mean_fused']:.3f}</td>
          <td>{row['mean_hessian']:.3f}</td>
          <td>{row['mean_completed_surface']:.3f}</td>
          <td>{row['binary_support_ratio']:.2f}</td>
          <td>{row['close_pairs_lt_8px']}</td>
          <td>{row['quality_score']:.3f}</td>
          <td>{html.escape(row['note'])}</td>
        </tr>
        """
        for row in summary_rows
    )
    image_cards = [
        ("00_input_high_gamma_workspace.png", "输入帧：高伽马 IR + 工作区"),
        ("01_current_depth_only_reference.png", "当前扫描 depth-only 参考"),
        ("02_combined_pr_fprg_reference.png", "组合响应 PR-FPRG 参考"),
        ("03_combined_response.png", "组合响应底图"),
        ("04_hessian_ridge.png", "Hessian ridge 底图"),
        ("05_frangi_like.png", "Frangi-like 底图"),
        ("06_fused_instance_response.png", "融合实例响应底图"),
        ("08_instance_graph_overlay.png", "实例骨架图谱"),
        ("10_surface_completion_overlay.png", "补全钢筋面"),
        ("90_top_variant_matrix.png", "Top 候选原图矩阵"),
    ]
    for row in summary_rows[:8]:
        prefix = safe_filename(row["id"])
        image_cards.append((f"variant_{prefix}_original.png", f"{row['id']} 原图点位"))
        image_cards.append((f"variant_{prefix}_rectified.png", f"{row['id']} rectified 点位"))
    cards_html = "\n".join(
        f"""
        <figure class="image-card">
          <img src="images/{html.escape(filename)}" alt="{html.escape(caption)}">
          <figcaption>{html.escape(caption)}</figcaption>
        </figure>
        """
        for filename, caption in image_cards
        if filename in image_paths
    )

    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>扫描层响应底图全量实验</title>
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
    main {{ max-width: 1640px; margin: 0 auto; padding: 16px clamp(12px, 3vw, 34px) 42px; }}
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
    .fact span {{ display: block; color: #607067; font-size: 12px; }}
    .fact b {{ display: block; font-size: 18px; overflow-wrap: anywhere; }}
    section {{ margin-top: 18px; padding-top: 18px; border-top: 1px solid #d8d0c0; }}
    h2 {{ margin: 0 0 10px; font-size: 20px; letter-spacing: 0; }}
    table {{ width: 100%; border-collapse: collapse; background: #fffdf7; border: 1px solid #d8d0c0; border-radius: 8px; overflow: hidden; font-size: 13px; }}
    th, td {{ padding: 8px 9px; border-bottom: 1px solid #e4dccd; vertical-align: top; text-align: left; }}
    th {{ background: #eee6d5; }}
    code {{ background: #ece3d2; border-radius: 4px; padding: 1px 4px; }}
    .note {{ border-left: 4px solid #c4672d; background: #fff8e8; padding: 10px 12px; border-radius: 0 8px 8px 0; }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 390px), 1fr));
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
      max-height: min(78vh, 980px);
      object-fit: contain;
      background: #10140f;
    }}
    figcaption {{ padding: 9px 10px; color: #384239; border-top: 1px solid #e6dfd1; }}
    @media (max-width: 720px) {{
      header {{ padding: 16px; }}
      main {{ padding: 12px; }}
      table {{ font-size: 12px; }}
      .image-card img {{ max-height: 72vh; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>扫描层响应底图全量实验</h1>
    <p class="subline">固定 snapshot：{html.escape(str(frame.get('frame_source')))}。该报告只评估离线候选，不修改运行主链。</p>
  </header>
  <main>
    <div class="facts">
      <div class="fact"><span>当前 depth-only 点数</span><b>{len(current_result['points'])}</b></div>
      <div class="fact"><span>当前 depth-only 周期</span><b>{summary['current_depth_runtime']['vertical_period_px']} / {summary['current_depth_runtime']['horizontal_period_px']} px</b></div>
      <div class="fact"><span>组合响应参考点数</span><b>{len(combined_result['points'])}</b></div>
      <div class="fact"><span>实例骨架 junction</span><b>{summary['instance_graph']['junction_count']}</b></div>
      <div class="fact"><span>推荐方案</span><b>{html.escape(summary['recommended_variant']['id'])}</b></div>
      <div class="fact"><span>总耗时</span><b>{timings_ms['total']:.1f} ms</b></div>
    </div>
    <section>
      <h2>结论</h2>
      <p class="note">推荐：<code>{html.escape(summary['recommended_variant']['id'])}</code>。{html.escape(summary['recommended_variant']['reason'])}</p>
      <p>注意：本 snapshot 没有人工标注真值，因此表格分数是代理指标，不等同于真实精度。它适合判断哪条路线更稳、更少过检，以及是否值得进入现场标注验证。</p>
    </section>
    <section>
      <h2>全量对比</h2>
      <table>
        <thead>
          <tr>
            <th>方案</th><th>名称</th><th>点数</th><th>线数</th><th>融合响应</th><th>Hessian</th><th>补全面</th><th>二值支撑</th><th>近重复</th><th>代理分</th><th>说明</th>
          </tr>
        </thead>
        <tbody>{table_rows}</tbody>
      </table>
    </section>
    <section>
      <h2>图像证据</h2>
      <div class="grid">{cards_html}</div>
    </section>
  </main>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--snapshot-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / "rebar_instance_segmentation_modalities_20260430_112028"),
    )
    parser.add_argument(
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"scan_response_full_evaluation_{time.strftime('%Y%m%d_%H%M%S')}"),
    )
    parser.add_argument("--threshold-percentile", type=float, default=83.0)
    parser.add_argument("--ir-display-gamma", type=float, default=1.95)
    args = parser.parse_args()

    total_start = time.perf_counter()
    started = time.perf_counter()
    frame = load_snapshot_frame(args.snapshot_dir)
    load_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    evaluation = build_evaluation(frame, args.threshold_percentile)
    evaluation_ms = (time.perf_counter() - started) * 1000.0

    timings_ms = {
        "load_snapshot": load_ms,
        "evaluate": evaluation_ms,
        "total": (time.perf_counter() - total_start) * 1000.0,
    }
    write_report(args.output_dir, frame, evaluation, timings_ms, args.ir_display_gamma)
    print(f"scan response full evaluation report: {args.output_dir}")


if __name__ == "__main__":
    main()
