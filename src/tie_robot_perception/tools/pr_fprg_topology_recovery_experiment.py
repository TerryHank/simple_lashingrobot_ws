#!/usr/bin/env python3

"""Run a step-by-step PR-FPRG topology recovery experiment report.

This report is intentionally experimental and does not change the runtime
mainline. It restores the 2026-04-22 style frequency/phase full-lattice
topology as a candidate generator, then applies the current response and
beam-candidate evidence as point-level filtering and diagnostics.
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
    draw_oriented_rectified_lines,
    normalize_probe_raw_world,
    render_result,
    run_peak_supported_pr_fprg,
    to_bgr,
    to_u8,
)
from rebar_instance_graph_probe import (  # noqa: E402
    capture_frame,
    derive_modalities,
    json_safe,
    render_binary,
    render_heatmap,
)
from rebar_surface_bindpoint_comparison import (  # noqa: E402
    build_surface_segmentation,
    points_from_rectified_points,
    render_ir_high_gamma,
)
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_axis_aligned_line_families,
    build_workspace_s2_axis_profile,
    build_workspace_s2_curved_line_families,
    build_workspace_s2_oriented_projective_line_segments,
    estimate_workspace_s2_fft_period_and_phase,
    intersect_workspace_s2_curved_line_families,
    intersect_workspace_s2_oriented_line_families,
    normalize_workspace_s2_response,
)


def build_recovered_frequency_phase_grid(result):
    """Build full row/column lattice from FFT period + phase, without peak pruning."""
    response = np.asarray(result["response"], dtype=np.float32)
    valid_mask = np.asarray(result["rectified_valid"]).astype(np.uint8)
    line_families = build_workspace_s2_axis_aligned_line_families(
        response,
        valid_mask,
        min_period=10,
        max_period=30,
        enable_local_peak_refine=False,
        enable_peak_support=False,
        enable_continuous_validation=False,
        enable_spacing_prune=False,
        enable_structural_edge_suppression=False,
        peak_min_ratio=0.0,
        period_estimator="fft",
    )
    if len(line_families) < 2:
        raise RuntimeError("recovered FFT/phase topology did not produce two line families")
    line_families = line_families[:2]
    rectified_points = intersect_workspace_s2_oriented_line_families(
        line_families[0],
        line_families[1],
        result["rectified_geometry"]["rectified_width"],
        result["rectified_geometry"]["rectified_height"],
    )
    return line_families, rectified_points


def render_profile_fft_plot(profile, estimate, line_positions, title):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    width = max(760, int(profile.size * 2))
    height = 360
    canvas = np.full((height, width, 3), (252, 250, 244), dtype=np.uint8)
    cv2.putText(canvas, title, (24, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (25, 33, 29), 2, cv2.LINE_AA)

    margin_left = 48
    margin_right = 24
    top_a = 52
    plot_h = 122
    gap = 42
    top_b = top_a + plot_h + gap
    plot_w = width - margin_left - margin_right

    def normalize(values):
        values = np.asarray(values, dtype=np.float32).reshape(-1)
        if values.size == 0:
            return values
        min_value = float(np.min(values))
        max_value = float(np.max(values))
        scale = max(max_value - min_value, 1e-6)
        return (values - min_value) / scale

    cv2.rectangle(canvas, (margin_left, top_a), (margin_left + plot_w, top_a + plot_h), (214, 205, 190), 1)
    profile_norm = normalize(profile)
    if profile_norm.size > 1:
        polyline = []
        for index, value in enumerate(profile_norm):
            x_pixel = margin_left + int(round(index / float(profile_norm.size - 1) * plot_w))
            y_pixel = top_a + plot_h - int(round(float(value) * plot_h))
            polyline.append((x_pixel, y_pixel))
        cv2.polylines(canvas, [np.asarray(polyline, dtype=np.int32)], False, (37, 99, 235), 2, cv2.LINE_AA)
        for position in line_positions:
            x_pixel = margin_left + int(round(float(position) / max(1.0, profile_norm.size - 1) * plot_w))
            cv2.line(canvas, (x_pixel, top_a), (x_pixel, top_a + plot_h), (22, 163, 74), 1)
    cv2.putText(canvas, "profile + recovered line positions", (margin_left, top_a - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (75, 85, 78), 1, cv2.LINE_AA)

    period_scores = []
    smoothed = profile.copy()
    if profile.size > 5:
        smoothed = cv2.GaussianBlur(profile.reshape(1, -1), (0, 0), sigmaX=1.8).reshape(-1)
    for period in range(10, 31):
        phase_scores = []
        for phase in range(period):
            samples = smoothed[phase::period]
            if samples.size:
                phase_scores.append(float(np.mean(samples)))
        period_scores.append(max(phase_scores) if phase_scores else 0.0)
    period_scores = np.asarray(period_scores, dtype=np.float32)
    score_norm = normalize(period_scores)
    cv2.rectangle(canvas, (margin_left, top_b), (margin_left + plot_w, top_b + plot_h), (214, 205, 190), 1)
    if score_norm.size:
        bar_w = max(4, int(plot_w / float(score_norm.size * 1.4)))
        best_period = int(round(float((estimate or {}).get("period", 0))))
        for offset, value in enumerate(score_norm):
            period = 10 + offset
            x_center = margin_left + int(round(offset / max(1, score_norm.size - 1) * plot_w))
            y_top = top_b + plot_h - int(round(float(value) * plot_h))
            color = (37, 99, 235) if period != best_period else (220, 80, 40)
            cv2.rectangle(canvas, (x_center - bar_w // 2, y_top), (x_center + bar_w // 2, top_b + plot_h), color, -1)
    period = (estimate or {}).get("period", "-")
    phase = (estimate or {}).get("phase", "-")
    score = float((estimate or {}).get("score", 0.0))
    cv2.putText(
        canvas,
        f"FFT/autocorr hump  period={period}  phase={phase}  score={score:.3f}",
        (margin_left, top_b - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (75, 85, 78),
        1,
        cv2.LINE_AA,
    )
    return canvas


def render_rectified_lattice(result, line_families, title):
    image = render_heatmap(result["response"])
    image = draw_oriented_rectified_lines(image, line_families, "spacing_pruned", colors=[(0, 180, 255), (255, 120, 40)])
    cv2.putText(image, title, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return image


def render_original_points(frame, result, line_families, points, title, ir_display_gamma=1.95):
    image = render_ir_high_gamma(frame, result, ir_display_gamma=ir_display_gamma, draw_label=False)
    segments = build_workspace_s2_oriented_projective_line_segments(
        result["manual_workspace"]["corner_pixels"],
        result["rectified_geometry"]["rectified_width"],
        result["rectified_geometry"]["rectified_height"],
        line_families,
    )
    for family_index, family_segments in enumerate(segments.values()):
        color = (0, 140, 255) if family_index == 0 else (255, 120, 40)
        for segment_start, segment_end in family_segments:
            cv2.line(image, tuple(segment_start), tuple(segment_end), color, 1, cv2.LINE_AA)
    for point in points:
        x_value, y_value = point["pix"]
        center = (int(round(x_value)), int(round(y_value)))
        cv2.circle(image, center, 4, (0, 255, 255), -1)
        cv2.circle(image, center, 7, (0, 110, 255), 1)
        if int(point["idx"]) <= 120:
            cv2.putText(image, str(point["idx"]), (center[0] + 5, center[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, title[:70], (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, title[:70], (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def render_current_peak_supported_original(frame, result, ir_display_gamma=1.95):
    image = render_ir_high_gamma(frame, result, ir_display_gamma=ir_display_gamma, draw_label=False)
    segments = build_workspace_s2_oriented_projective_line_segments(
        result["manual_workspace"]["corner_pixels"],
        result["rectified_geometry"]["rectified_width"],
        result["rectified_geometry"]["rectified_height"],
        result["line_families"],
    )
    for family_segments in segments.values():
        for segment_start, segment_end in family_segments:
            cv2.line(image, tuple(segment_start), tuple(segment_end), (0, 255, 0), 1, cv2.LINE_AA)
    for point in result.get("points", []):
        center = tuple(int(v) for v in point["pix"])
        cv2.circle(image, center, 4, (0, 255, 255), -1)
        cv2.circle(image, center, 7, (0, 100, 255), 1)
    cv2.putText(image, f"current peak-supported  points={len(result.get('points', []))}", (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, f"current peak-supported  points={len(result.get('points', []))}", (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def project_rectified_polyline_to_image(result, polyline_points):
    polyline = np.asarray(polyline_points, dtype=np.float32).reshape(-1, 1, 2)
    if polyline.size == 0:
        return np.empty((0, 2), dtype=np.int32)
    mapped = cv2.perspectiveTransform(
        polyline,
        np.asarray(result["rectified_geometry"]["inverse_h"], dtype=np.float32),
    ).reshape(-1, 2)
    return np.rint(mapped).astype(np.int32)


def draw_curved_families_on_rectified(image, curved_families):
    colors = [(0, 180, 255), (255, 120, 40)]
    for family_index, family in enumerate(curved_families or []):
        color = colors[family_index % len(colors)]
        for curved_line in family.get("curved_lines", []):
            polyline = np.asarray(curved_line.get("polyline_points", []), dtype=np.float32).reshape(-1, 2)
            if polyline.shape[0] < 2:
                continue
            cv2.polylines(image, [np.rint(polyline).astype(np.int32)], False, color, 1, cv2.LINE_AA)
    return image


def draw_curved_families_on_original(image, result, curved_families):
    colors = [(0, 180, 255), (255, 120, 40)]
    for family_index, family in enumerate(curved_families or []):
        color = colors[family_index % len(colors)]
        for curved_line in family.get("curved_lines", []):
            mapped = project_rectified_polyline_to_image(result, curved_line.get("polyline_points", []))
            if mapped.shape[0] < 2:
                continue
            cv2.polylines(image, [mapped], False, color, 1, cv2.LINE_AA)
    return image


def render_curve_variant_original(frame, result, variant, ir_display_gamma=1.95):
    image = render_ir_high_gamma(frame, result, ir_display_gamma=ir_display_gamma, draw_label=False)
    draw_curved_families_on_original(image, result, variant.get("curved_families", []))
    for point in variant.get("points", []):
        x_value, y_value = point["pix"]
        center = (int(round(x_value)), int(round(y_value)))
        cv2.circle(image, center, 4, (0, 255, 255), -1)
        cv2.circle(image, center, 7, (0, 110, 255), 1)
        if int(point["idx"]) <= 120:
            cv2.putText(image, str(point["idx"]), (center[0] + 5, center[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (255, 255, 255), 1, cv2.LINE_AA)
    title = f"{variant['id']}  points={variant['point_count']}"
    cv2.putText(image, title[:70], (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, title[:70], (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def render_curve_variant_rectified(response_map, variant):
    image = render_heatmap(response_map)
    draw_curved_families_on_rectified(image, variant.get("curved_families", []))
    for point in variant.get("points", []):
        x_value, y_value = point["rectified"]
        center = (int(round(x_value)), int(round(y_value)))
        cv2.circle(image, center, 4, (0, 255, 255), -1)
        cv2.circle(image, center, 7, (20, 40, 40), 1)
    cv2.putText(image, variant["id"][:70], (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return image


def build_curve3456_on_recovered_mainline(result, modalities, surface_segmentation, recovered_line_families):
    """Run 3/4/5/6 curve algorithms using recovered frequency-phase topology."""
    valid_mask = np.asarray(result["rectified_valid"]).astype(np.uint8)
    completed_response = surface_segmentation["completed_surface_response"]
    multimodal_response = normalize_workspace_s2_response(
        (0.45 * modalities["combined_response"])
        + (0.30 * completed_response)
        + (0.25 * modalities["frangi_like"]),
        result["rectified_valid"].astype(bool),
    )
    curve_specs = [
        (
            "03_recovered_greedy_curve",
            "03 频相主线 + greedy 曲线",
            result["response"],
            "greedy",
            "response",
            0.08,
        ),
        (
            "04_recovered_dp_curve",
            "04 频相主线 + DP 曲线",
            result["response"],
            "dynamic_programming",
            "response",
            0.12,
        ),
        (
            "05_recovered_ridge_curve",
            "05 频相主线 + ridge 约束曲线",
            completed_response,
            "dynamic_programming",
            "response_ridge",
            0.12,
        ),
        (
            "06_recovered_ir_assisted_curve",
            "06 频相主线 + 红外/多模态辅助曲线",
            multimodal_response,
            "dynamic_programming",
            "response_ridge",
            0.12,
        ),
    ]
    variants = []
    for variant_id, title, response_map, trace_method, score_mode, smoothness_weight in curve_specs:
        curved_families = build_workspace_s2_curved_line_families(
            response_map,
            valid_mask,
            recovered_line_families,
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
        fallback_to_recovered_grid = False
        if not rectified_points:
            fallback_to_recovered_grid = True
            rectified_points = intersect_workspace_s2_oriented_line_families(
                recovered_line_families[0],
                recovered_line_families[1],
                result["rectified_geometry"]["rectified_width"],
                result["rectified_geometry"]["rectified_height"],
            )
        points = points_from_rectified_points(
            result,
            rectified_points,
            response_map,
            variant_id,
            exclusion_mask=modalities["beam_candidate_mask_13cm"],
        )
        variants.append(
            {
                "id": variant_id,
                "title": title,
                "points": points,
                "point_count": len(points),
                "mean_score": float(np.mean([point["score"] for point in points])) if points else 0.0,
                "curved_families": curved_families,
                "response_map": response_map,
                "fallback_to_recovered_grid": bool(fallback_to_recovered_grid),
            }
        )
    return variants


def build_experiment(frame, threshold_percentile=83.0, ir_display_gamma=1.95):
    result = run_peak_supported_pr_fprg(frame, response_name_filter="combined_depth_ir_darkline")
    modalities = derive_modalities(result, threshold_percentile=threshold_percentile)
    surface_segmentation = build_surface_segmentation(result, modalities)
    recovered_line_families, recovered_rectified_points = build_recovered_frequency_phase_grid(result)

    recovered_topology_before_filter = points_from_rectified_points(
        result,
        recovered_rectified_points,
        result["response"],
        "recovered_topology_before_filter",
        exclusion_mask=None,
    )
    recovered_topology_after_beam_filter = points_from_rectified_points(
        result,
        recovered_rectified_points,
        result["response"],
        "recovered_topology_after_beam_filter",
        exclusion_mask=modalities["beam_candidate_mask_13cm"],
    )

    vertical_profile = build_workspace_s2_axis_profile(result["response"], result["rectified_valid"].astype(np.uint8), axis=0)
    horizontal_profile = build_workspace_s2_axis_profile(result["response"], result["rectified_valid"].astype(np.uint8), axis=1)
    vertical_fft_estimate = estimate_workspace_s2_fft_period_and_phase(vertical_profile, min_period=10, max_period=30)
    horizontal_fft_estimate = estimate_workspace_s2_fft_period_and_phase(horizontal_profile, min_period=10, max_period=30)
    recovered_curve3456 = build_curve3456_on_recovered_mainline(
        result,
        modalities,
        surface_segmentation,
        recovered_line_families,
    )

    return {
        "result": result,
        "modalities": modalities,
        "surface_segmentation": surface_segmentation,
        "recovered_line_families": recovered_line_families,
        "recovered_rectified_points": recovered_rectified_points,
        "recovered_topology_before_filter": recovered_topology_before_filter,
        "recovered_topology_after_beam_filter": recovered_topology_after_beam_filter,
        "vertical_profile": vertical_profile,
        "horizontal_profile": horizontal_profile,
        "vertical_fft_estimate": vertical_fft_estimate,
        "horizontal_fft_estimate": horizontal_fft_estimate,
        "recovered_curve3456": recovered_curve3456,
        "ir_display_gamma": float(ir_display_gamma),
    }


def summarize_fft_estimate(estimate):
    estimate = estimate or {}
    keys = ("period", "phase", "score", "fft_score", "correlation_score", "phase_score", "frequency_bin", "method")
    return {key: estimate[key] for key in keys if key in estimate}


def write_report(output_dir, frame, experiment, timings_ms):
    output_dir = Path(output_dir)
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)
    result = experiment["result"]
    modalities = experiment["modalities"]
    recovered_line_families = experiment["recovered_line_families"]
    before_points = experiment["recovered_topology_before_filter"]
    after_points = experiment["recovered_topology_after_beam_filter"]
    ir_display_gamma = experiment["ir_display_gamma"]
    curve_variants = experiment["recovered_curve3456"]

    images = {
        "01_input_workspace.png": render_ir_high_gamma(frame, result, ir_display_gamma=ir_display_gamma),
        "02_rectified_ir.png": to_bgr(to_u8(result["rectified_ir"])),
        "03_combined_response.png": render_heatmap(result["response"]),
        "04_vertical_profile_fft.png": render_profile_fft_plot(
            experiment["vertical_profile"],
            experiment["vertical_fft_estimate"],
            recovered_line_families[0].get("line_rhos", []),
            "vertical / column profile FFT",
        ),
        "05_horizontal_profile_fft.png": render_profile_fft_plot(
            experiment["horizontal_profile"],
            experiment["horizontal_fft_estimate"],
            recovered_line_families[1].get("line_rhos", []),
            "horizontal / row profile FFT",
        ),
        "06_frequency_phase_lattice_rectified.png": render_rectified_lattice(
            result,
            recovered_line_families,
            "frequency-phase full lattice",
        ),
        "07_beam_candidate_13cm_mask.png": render_binary(modalities["beam_candidate_mask_13cm"], "beam_candidate_mask_13cm"),
        "08_before_filter_original.png": render_original_points(
            frame,
            result,
            recovered_line_families,
            before_points,
            f"recovered topology before filter  points={len(before_points)}",
            ir_display_gamma=ir_display_gamma,
        ),
        "09_after_filter_original.png": render_original_points(
            frame,
            result,
            recovered_line_families,
            after_points,
            f"recovered topology after beam +/-13cm  points={len(after_points)}",
            ir_display_gamma=ir_display_gamma,
        ),
        "10_current_peak_supported_original.png": render_current_peak_supported_original(
            frame,
            result,
            ir_display_gamma=ir_display_gamma,
        ),
        "11_current_peak_supported_result.png": render_result(frame, result),
    }
    curve_image_filenames = [
        ("12_03_recovered_greedy_curve_original.png", "12r_03_recovered_greedy_curve_rectified.png"),
        ("13_04_recovered_dp_curve_original.png", "13r_04_recovered_dp_curve_rectified.png"),
        ("14_05_recovered_ridge_curve_original.png", "14r_05_recovered_ridge_curve_rectified.png"),
        ("15_06_recovered_ir_assisted_curve_original.png", "15r_06_recovered_ir_assisted_curve_rectified.png"),
    ]
    for variant, (original_filename, rectified_filename) in zip(curve_variants, curve_image_filenames):
        images[original_filename] = render_curve_variant_original(
            frame,
            result,
            variant,
            ir_display_gamma=ir_display_gamma,
        )
        images[rectified_filename] = render_curve_variant_rectified(
            variant["response_map"],
            variant,
        )
    image_paths = {}
    for filename, image in images.items():
        path = image_dir / filename
        cv2.imwrite(str(path), image)
        image_paths[filename] = str(path)

    current_line_counts = [len(family.get("line_rhos", [])) for family in result.get("line_families", [])]
    recovered_line_counts = [len(family.get("line_rhos", [])) for family in recovered_line_families]
    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source", "raw_world"),
        "response_name": result.get("response_name"),
        "current_peak_supported": {
            "line_counts": current_line_counts,
            "point_count": len(result.get("points", [])),
        },
        "recovered_topology_before_filter": {
            "line_counts": recovered_line_counts,
            "point_count": len(before_points),
            "vertical_fft_estimate": summarize_fft_estimate(experiment["vertical_fft_estimate"]),
            "horizontal_fft_estimate": summarize_fft_estimate(experiment["horizontal_fft_estimate"]),
        },
        "recovered_topology_after_beam_filter": {
            "point_count": len(after_points),
            "beam_candidate_13cm_pixels": int(np.count_nonzero(modalities["beam_candidate_mask_13cm"])),
        },
        "recovered_curve3456": [
            {
                "id": variant["id"],
                "title": variant["title"],
                "point_count": variant["point_count"],
                "mean_score": variant["mean_score"],
                "fallback_to_recovered_grid": variant["fallback_to_recovered_grid"],
            }
            for variant in curve_variants
        ],
        "beam_candidate_bands": modalities["beam_candidate_bands"],
        "legacy_edge_bands": modalities["legacy_edge_bands"],
        "surface_segmentation": experiment["surface_segmentation"]["stats"],
        "timings_ms": timings_ms,
        "images": image_paths,
    }
    (output_dir / "summary.json").write_text(json.dumps(json_safe(summary), indent=2, ensure_ascii=False), encoding="utf-8")

    cards = [
        ("01_input_workspace.png", "1. 当前输入工作区（高伽马 IR + 工作区框）"),
        ("02_rectified_ir.png", "2. 透视展开后的 IR 平面"),
        ("03_combined_response.png", "3. 组合响应底图（深度暗线 + IR 暗线）"),
        ("04_vertical_profile_fft.png", "4. 列方向 profile + FFT 周期驼峰"),
        ("05_horizontal_profile_fft.png", "5. 行方向 profile + FFT 周期驼峰"),
        ("06_frequency_phase_lattice_rectified.png", "6. period + phase 生成的完整 rectified 网格"),
        ("07_beam_candidate_13cm_mask.png", "7. 梁筋 beam_candidate ±13 cm mask"),
        ("08_before_filter_original.png", "8. 过滤前：完整频相网格候选点"),
        ("09_after_filter_original.png", "9. 过滤后：梁筋 ±13 cm 点级过滤"),
        ("10_current_peak_supported_original.png", "10. 当前 peak-supported 主链对照"),
        ("12_03_recovered_greedy_curve_original.png", "12. 以频相主线为基底：03 greedy 曲线"),
        ("13_04_recovered_dp_curve_original.png", "13. 以频相主线为基底：04 DP 曲线"),
        ("14_05_recovered_ridge_curve_original.png", "14. 以频相主线为基底：05 ridge 约束曲线"),
        ("15_06_recovered_ir_assisted_curve_original.png", "15. 以频相主线为基底：06 红外/多模态辅助曲线"),
    ]
    cards_html = "\n".join(
        f"""
        <figure class="image-card">
          <img src="images/{html.escape(filename)}" alt="{html.escape(caption)}">
          <figcaption>{html.escape(caption)}</figcaption>
        </figure>
        """
        for filename, caption in cards
    )
    curve_rows_html = "\n".join(
        f"""
          <tr>
            <td><code>{html.escape(variant['id'])}</code></td>
            <td>{variant['point_count']}</td>
            <td>{variant['mean_score']:.3f}</td>
            <td>{'是' if variant['fallback_to_recovered_grid'] else '否'}</td>
          </tr>
        """
        for variant in curve_variants
    )
    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PR-FPRG 频相主拓扑恢复实验报告</title>
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
    header {{ padding: 22px clamp(16px, 4vw, 42px); background: #fbf7ee; border-bottom: 1px solid #d9d0c2; }}
    h1 {{ margin: 0 0 6px; font-size: clamp(22px, 3vw, 34px); letter-spacing: 0; }}
    .subline {{ margin: 0; color: #5b655d; }}
    main {{ max-width: 1560px; margin: 0 auto; padding: 16px clamp(12px, 3vw, 34px) 42px; }}
    .facts {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(min(100%, 210px), 1fr)); gap: 10px; margin: 16px 0 18px; }}
    .fact {{ border: 1px solid #d8d0c0; border-radius: 8px; padding: 10px 12px; background: #fffdf7; min-width: 0; }}
    .fact span {{ display: block; color: #607067; font-size: 12px; }}
    .fact b {{ display: block; font-size: 19px; overflow-wrap: anywhere; }}
    section {{ margin-top: 18px; padding-top: 18px; border-top: 1px solid #d8d0c0; }}
    h2 {{ margin: 0 0 10px; font-size: 20px; letter-spacing: 0; }}
    .note {{ border-left: 4px solid #c4672d; background: #fff8e8; padding: 10px 12px; border-radius: 0 8px 8px 0; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(min(100%, 390px), 1fr)); gap: 14px; align-items: start; }}
    .image-card {{ margin: 0; border: 1px solid #d8d0c0; border-radius: 8px; background: #fffdf7; overflow: hidden; min-width: 0; }}
    .image-card img {{ display: block; width: 100%; max-height: min(78vh, 980px); object-fit: contain; background: #10140f; }}
    figcaption {{ padding: 9px 10px; color: #384239; border-top: 1px solid #e6dfd1; }}
    code {{ background: #ece3d2; border-radius: 4px; padding: 1px 4px; }}
    table {{ width: 100%; border-collapse: collapse; background: #fffdf7; border: 1px solid #d8d0c0; border-radius: 8px; overflow: hidden; }}
    th, td {{ padding: 9px 10px; border-bottom: 1px solid #e4dccd; vertical-align: top; text-align: left; }}
    th {{ background: #eee6d5; }}
    @media (max-width: 720px) {{ header {{ padding: 16px; }} main {{ padding: 12px; }} .image-card img {{ max-height: 72vh; }} }}
  </style>
</head>
<body>
  <header>
    <h1>PR-FPRG 频相主拓扑恢复实验报告</h1>
    <p class="subline">当前帧真实跑实验：组合响应为底图，FFT / phase 生成完整网格，再做梁筋 ±13 cm 点级过滤，并与当前 peak-supported 主链对照。</p>
  </header>
  <main>
    <div class="facts">
      <div class="fact"><span>响应</span><b>{html.escape(str(summary['response_name']))}</b></div>
      <div class="fact"><span>当前主链</span><b>{current_line_counts} / {len(result.get('points', []))} 点</b></div>
      <div class="fact"><span>频相网格</span><b>{recovered_line_counts} / {len(before_points)} 点</b></div>
      <div class="fact"><span>梁筋过滤后</span><b>{len(after_points)} 点</b></div>
      <div class="fact"><span>beam mask 像素</span><b>{int(np.count_nonzero(modalities['beam_candidate_mask_13cm']))}</b></div>
      <div class="fact"><span>总耗时</span><b>{timings_ms['total']:.1f} ms</b></div>
    </div>
    <section>
      <h2>实验结论</h2>
      <p class="note">这份报告不是只画流程，而是对当前画面逐步跑出中间结果。重点看第 4/5 步的 FFT 周期驼峰、第 6 步的完整频相网格，以及第 8/9 步过滤前后的原图点位。</p>
    </section>
    <section>
      <h2>关键数值</h2>
      <table>
        <thead><tr><th>项目</th><th>结果</th></tr></thead>
        <tbody>
          <tr><td>当前 peak-supported 线族</td><td><code>{current_line_counts}</code></td></tr>
          <tr><td>恢复频相网格线族</td><td><code>{recovered_line_counts}</code></td></tr>
          <tr><td>纵向 FFT estimate</td><td><code>{html.escape(str(summarize_fft_estimate(experiment['vertical_fft_estimate'])))}</code></td></tr>
          <tr><td>横向 FFT estimate</td><td><code>{html.escape(str(summarize_fft_estimate(experiment['horizontal_fft_estimate'])))}</code></td></tr>
          <tr><td>beam_candidate bands</td><td><code>{html.escape(str(modalities['beam_candidate_bands']))}</code></td></tr>
        </tbody>
      </table>
    </section>
    <section>
      <h2>3/4/5/6：以频相主线为基底</h2>
      <table>
        <thead><tr><th>算法</th><th>过滤后点数</th><th>均值响应</th><th>是否兜底直线交点</th></tr></thead>
        <tbody>{curve_rows_html}</tbody>
      </table>
    </section>
    <section>
      <h2>每一步效果图</h2>
      <div class="grid">{cards_html}</div>
    </section>
  </main>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--output-dir", default=str(WORKSPACE_ROOT / ".debug_frames" / f"pr_fprg_topology_recovery_experiment_{time.strftime('%Y%m%d_%H%M%S')}"))
    parser.add_argument("--snapshot-dir", default=None)
    parser.add_argument("--no-snapshot-fallback", action="store_true")
    parser.add_argument("--threshold-percentile", type=float, default=83.0)
    parser.add_argument("--ir-display-gamma", type=float, default=1.95)
    args = parser.parse_args()

    rospy.init_node("pr_fprg_topology_recovery_experiment", anonymous=True, disable_signals=True)
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
    experiment = build_experiment(
        frame,
        threshold_percentile=args.threshold_percentile,
        ir_display_gamma=args.ir_display_gamma,
    )
    experiment_ms = (time.perf_counter() - started) * 1000.0
    timings_ms = {
        "capture": capture_ms,
        "experiment": experiment_ms,
        "total": (time.perf_counter() - total_start) * 1000.0,
    }
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    write_report(output_dir, frame, experiment, timings_ms)
    rospy.loginfo("PointAI_log: PR-FPRG 频相主拓扑恢复实验报告输出目录：%s", output_dir)


if __name__ == "__main__":
    main()
