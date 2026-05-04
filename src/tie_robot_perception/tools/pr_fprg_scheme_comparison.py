#!/usr/bin/env python3

"""Generate a browser report that compares independent PR-FPRG follow-up schemes."""

from __future__ import annotations

import argparse
import copy
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
    build_initial_rhos_for_families,
    capture_depth_ir_frame,
    capture_synced_frame,
    clip_oriented_rectified_line,
    get_valid_world_coord_near_pixel,
    normalize_probe_raw_world,
    render_response_heatmap,
    render_rectified_lines,
    render_removed_lines,
    run_peak_supported_pr_fprg,
    to_bgr,
    to_u8,
)
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_axis_aligned_line_families,
    build_workspace_s2_curved_line_families,
    build_workspace_s2_oriented_axis_profile,
    build_workspace_s2_structural_edge_suppression_mask,
    detect_workspace_s2_structural_edge_bands,
    expand_workspace_s2_exclusion_mask_by_metric_margin,
    filter_workspace_s2_rectified_points_outside_mask,
    intersect_workspace_s2_curved_line_families,
    intersect_workspace_s2_oriented_line_families,
    map_workspace_s2_rectified_points_to_image,
    normalize_workspace_s2_response,
    prune_workspace_s2_line_rhos_by_spacing,
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


def clone_line_families(line_families):
    cloned = copy.deepcopy(line_families)
    for family_index, family in enumerate(cloned):
        family["family_index"] = family_index
    return cloned


def build_infrared_dark_response(result):
    rectified_ir = np.asarray(result["rectified_ir"], dtype=np.float32)
    valid_mask = np.asarray(result["rectified_valid"]).astype(bool)
    background = cv2.GaussianBlur(rectified_ir, (0, 0), sigmaX=7.0, sigmaY=7.0)
    dark_response = background - rectified_ir
    return normalize_workspace_s2_response(dark_response, valid_mask)


def build_combined_depth_ir_response(result, depth_weight=0.68):
    if str(result.get("response_source", "")) == "depth_ir":
        return normalize_workspace_s2_response(
            np.asarray(result["response"], dtype=np.float32),
            np.asarray(result["rectified_valid"]).astype(bool),
        )

    depth_response = normalize_workspace_s2_response(
        np.asarray(result["response"], dtype=np.float32),
        np.asarray(result["rectified_valid"]).astype(bool),
    )
    infrared_response = build_infrared_dark_response(result)
    depth_weight = float(np.clip(depth_weight, 0.0, 1.0))
    combined = (depth_weight * depth_response) + ((1.0 - depth_weight) * infrared_response)
    return normalize_workspace_s2_response(combined, np.asarray(result["rectified_valid"]).astype(bool))


def selected_response_caption(result):
    response_source = str(result.get("response_source", ""))
    if response_source == "depth_ir":
        return "底层组合响应"
    if response_source == "depth":
        return "底层深度响应"
    if response_source.startswith("ir"):
        return "底层红外响应"
    return "底层响应"


def apply_display_gamma_u8(image_u8, display_gamma=1.0):
    display_gamma = max(0.05, float(display_gamma or 1.0))
    image_u8 = np.asarray(image_u8, dtype=np.uint8)
    if abs(display_gamma - 1.0) <= 1e-6:
        return image_u8.copy()
    lookup = np.asarray(
        [np.clip(((value / 255.0) ** (1.0 / display_gamma)) * 255.0, 0.0, 255.0) for value in range(256)],
        dtype=np.uint8,
    )
    return cv2.LUT(image_u8, lookup)


def render_gray_gamma(image, display_gamma=1.0):
    return to_bgr(apply_display_gamma_u8(to_u8(image), display_gamma))


def render_response_heatmap_gamma(response, display_gamma=1.0):
    response_u8 = apply_display_gamma_u8(to_u8(response), display_gamma)
    return cv2.applyColorMap(response_u8, cv2.COLORMAP_TURBO)


def render_response_with_beam_mask(response_map, beam_mask):
    image = render_response_heatmap(response_map)
    if beam_mask is None:
        return image
    beam_mask = np.asarray(beam_mask).astype(bool)
    if beam_mask.shape[:2] != image.shape[:2] or not np.any(beam_mask):
        return image

    red_layer = np.zeros_like(image)
    red_layer[:, :] = (0, 0, 255)
    image[beam_mask] = cv2.addWeighted(image[beam_mask], 0.18, red_layer[beam_mask], 0.82, 0.0)

    hatch_layer = image.copy()
    height, width = image.shape[:2]
    for offset in range(-height, width, 18):
        cv2.line(hatch_layer, (offset, height - 1), (offset + height, 0), (255, 255, 255), 1, cv2.LINE_AA)
    image[beam_mask] = cv2.addWeighted(image[beam_mask], 0.72, hatch_layer[beam_mask], 0.28, 0.0)

    contours, _hierarchy = cv2.findContours(beam_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(image, contours, -1, (255, 255, 255), 4)
        cv2.drawContours(image, contours, -1, (0, 0, 255), 2)
    return image


def render_beam_mask_binary(beam_mask):
    beam_mask = np.asarray(beam_mask).astype(bool) if beam_mask is not None else np.zeros((1, 1), dtype=bool)
    height, width = beam_mask.shape[:2]
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    if np.any(beam_mask):
        canvas[beam_mask] = (255, 255, 255)
        contours, _hierarchy = cv2.findContours(beam_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            cv2.drawContours(canvas, contours, -1, (0, 0, 255), 2)
    else:
        cv2.putText(
            canvas,
            "NO BEAM MASK DETECTED",
            (max(8, width // 16), max(24, height // 2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.45, min(width, height) / 760.0),
            (190, 190, 190),
            1,
            cv2.LINE_AA,
        )
    return canvas


def render_fft_frequency_spectrum(line_families):
    plot_width = 760
    plot_height = 260
    margin_left = 62
    margin_right = 34
    margin_top = 42
    margin_bottom = 46
    panels = []

    for family_index, family in enumerate(line_families or []):
        estimate = family.get("estimate") or {}
        spectrum = np.asarray(estimate.get("spectrum", []), dtype=np.float32).reshape(-1)
        profile = np.asarray(family.get("profile", []), dtype=np.float32).reshape(-1)
        canvas = np.full((plot_height, plot_width, 3), 255, dtype=np.uint8)
        plot_x0 = margin_left
        plot_y0 = margin_top
        plot_w = plot_width - margin_left - margin_right
        plot_h = plot_height - margin_top - margin_bottom
        cv2.line(canvas, (plot_x0, plot_y0 + plot_h), (plot_x0 + plot_w, plot_y0 + plot_h), (31, 41, 55), 2)
        cv2.line(canvas, (plot_x0, plot_y0), (plot_x0, plot_y0 + plot_h), (31, 41, 55), 2)
        for grid_index in range(1, 4):
            grid_y = plot_y0 + int(round((grid_index / 4.0) * plot_h))
            cv2.line(canvas, (plot_x0, grid_y), (plot_x0 + plot_w, grid_y), (226, 232, 240), 1)

        candidate_periods = np.arange(10, 31, dtype=np.float32)
        period_scores = []
        profile_size = int(profile.size)
        if spectrum.size > 2 and profile_size > 0:
            for candidate_period in candidate_periods:
                frequency_bin = int(round(float(profile_size) / float(candidate_period)))
                if frequency_bin <= 0 or frequency_bin >= spectrum.size:
                    period_scores.append(0.0)
                    continue
                band_start = max(1, frequency_bin - 1)
                band_end = min(spectrum.size - 1, frequency_bin + 1)
                period_scores.append(float(np.max(spectrum[band_start:band_end + 1])))
            values = np.log1p(np.asarray(period_scores, dtype=np.float32))
            max_value = max(float(np.max(values)), 1e-6)
            points = []
            for period_index, value in enumerate(values):
                x_pixel = plot_x0 + int(round((period_index / max(1, values.size - 1)) * plot_w))
                y_pixel = plot_y0 + plot_h - int(round((float(value) / max_value) * plot_h))
                points.append((x_pixel, y_pixel))
            if len(points) >= 2:
                cv2.polylines(canvas, [np.asarray(points, dtype=np.int32)], False, (37, 99, 235), 3)

            selected_period = float(estimate.get("period", 0.0) or 0.0)
            if float(candidate_periods[0]) <= selected_period <= float(candidate_periods[-1]):
                x_ratio = (selected_period - float(candidate_periods[0])) / max(1.0, float(candidate_periods[-1] - candidate_periods[0]))
                x_pixel = plot_x0 + int(round(x_ratio * plot_w))
                nearest_index = int(np.argmin(np.abs(candidate_periods - selected_period)))
                selected_value = float(values[nearest_index]) if values.size else 0.0
                y_pixel = plot_y0 + plot_h - int(round((selected_value / max_value) * plot_h))
                cv2.line(canvas, (x_pixel, plot_y0), (x_pixel, plot_y0 + plot_h), (0, 128, 255), 2)
                cv2.circle(canvas, (x_pixel, y_pixel), 5, (0, 128, 255), -1)
                cv2.putText(
                    canvas,
                    "peak period_px=%s" % estimate.get("period", "-"),
                    (min(x_pixel + 8, plot_width - 230), max(plot_y0 + 18, y_pixel - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    (17, 24, 39),
                    1,
                    cv2.LINE_AA,
                )

        for tick_period in (10, 15, 20, 25, 30):
            x_ratio = (float(tick_period) - 10.0) / 20.0
            x_pixel = plot_x0 + int(round(x_ratio * plot_w))
            cv2.line(canvas, (x_pixel, plot_y0 + plot_h), (x_pixel, plot_y0 + plot_h + 5), (31, 41, 55), 1)
            cv2.putText(
                canvas,
                str(tick_period),
                (x_pixel - 9, plot_y0 + plot_h + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (71, 85, 105),
                1,
                cv2.LINE_AA,
            )

        cv2.putText(
            canvas,
            "family %d %s FFT peak curve" % (family_index, family.get("axis_orientation", "axis")),
            (margin_left, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (17, 24, 39),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "period_px",
            (plot_x0 + plot_w - 72, plot_y0 + plot_h + 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (71, 85, 105),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "FFT power",
            (8, plot_y0 + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (71, 85, 105),
            1,
            cv2.LINE_AA,
        )
        panels.append(canvas)

    if not panels:
        canvas = np.full((plot_height, plot_width, 3), 255, dtype=np.uint8)
        cv2.putText(canvas, "FFT spectrum unavailable", (margin_left, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (17, 24, 39), 1, cv2.LINE_AA)
        return canvas
    return np.vstack(panels)


def refine_line_families_by_auxiliary_profile(line_families, auxiliary_response, workspace_mask, search_radius_px=8):
    refined_families = clone_line_families(line_families)
    search_radius_px = max(1, int(round(search_radius_px)))
    for family in refined_families:
        profile_data = build_workspace_s2_oriented_axis_profile(
            auxiliary_response,
            workspace_mask,
            family["line_angle_deg"],
        )
        profile = np.asarray(profile_data.get("profile", []), dtype=np.float32).reshape(-1)
        rho_min = float(profile_data.get("rho_min", family.get("rho_min", 0.0)))
        if profile.size == 0:
            continue

        smoothed_profile = np.convolve(
            profile,
            np.asarray([1.0, 2.0, 3.0, 2.0, 1.0], dtype=np.float32) / 9.0,
            mode="same",
        )
        refined_rhos = []
        for line_rho in family.get("line_rhos", []):
            center_position = int(round(float(line_rho) - rho_min))
            window_start = max(0, center_position - search_radius_px)
            window_end = min(profile.size - 1, center_position + search_radius_px)
            if window_end < window_start:
                refined_rhos.append(float(line_rho))
                continue
            window = smoothed_profile[window_start:window_end + 1]
            if window.size == 0 or float(np.max(window)) <= 1e-6:
                refined_rhos.append(float(line_rho))
                continue
            refined_rhos.append(float(rho_min + window_start + int(np.argmax(window))))

        family["line_rhos"] = prune_workspace_s2_line_rhos_by_spacing(sorted(refined_rhos))
        family["auxiliary_profile"] = profile
    return refined_families


def points_from_rectified_intersections(frame, result, rectified_points):
    raw_world, _used_depth_fallback_raw_world = normalize_probe_raw_world(frame["raw"])
    workspace_mask = result["workspace_mask"]
    image_intersections = map_workspace_s2_rectified_points_to_image(
        rectified_points,
        result["rectified_geometry"]["inverse_h"],
    )
    points = []
    for image_point, rectified_point in zip(image_intersections, rectified_points):
        pixel_x, pixel_y = int(round(float(image_point[0]))), int(round(float(image_point[1])))
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
                "pix": [pixel_x, pixel_y],
                "rectified": [float(rectified_point[0]), float(rectified_point[1])],
                "world": world_coord,
                "sample_pix": sample_pixel,
                "used_fallback": bool(used_fallback),
            }
        )
    return points


def filter_beam_points(result, rectified_points):
    beam_mask = result.get("beam_mask")
    rectified_points = list(rectified_points or [])
    if beam_mask is None:
        return rectified_points, 0
    filtered_points = filter_workspace_s2_rectified_points_outside_mask(
        rectified_points,
        beam_mask,
        margin_px=2,
    )
    return filtered_points, len(rectified_points) - len(filtered_points)


def summarize_curve_metrics(line_families):
    coverage_values = []
    score_means = []
    absolute_offsets = []
    wiggles = []
    for family in line_families or []:
        for curved_line in family.get("curved_lines") or []:
            coverage_values.append(float(curved_line.get("coverage", 0.0)))
            offsets = np.asarray(curved_line.get("offsets") or [], dtype=np.float32)
            scores = np.asarray(curved_line.get("scores") or [], dtype=np.float32)
            if offsets.size:
                absolute_offsets.extend(np.abs(offsets).tolist())
                if offsets.size > 2:
                    wiggles.append(float(np.mean(np.abs(np.diff(offsets, n=2)))))
            if scores.size:
                score_means.append(float(np.mean(scores)))

    return {
        "coverage_mean": round(float(np.mean(coverage_values)), 3) if coverage_values else None,
        "score_mean": round(float(np.mean(score_means)), 3) if score_means else None,
        "abs_offset_mean": round(float(np.mean(absolute_offsets)), 3) if absolute_offsets else None,
        "abs_offset_p95": round(float(np.percentile(absolute_offsets, 95.0)), 3) if absolute_offsets else None,
        "wiggle_mean": round(float(np.mean(wiggles)), 3) if wiggles else None,
    }


def draw_workspace_and_points(frame, result, points, line_color=(0, 255, 0)):
    image = cv2.cvtColor(to_u8(frame["ir"]), cv2.COLOR_GRAY2BGR)
    polygon = np.asarray(result["manual_workspace"]["corner_pixels"], dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [polygon], True, (220, 220, 220), 2)
    for point in points:
        cv2.circle(image, tuple(point["pix"]), 4, (0, 255, 255), -1)
        cv2.circle(image, tuple(point["pix"]), 5, line_color, 1)
        cv2.putText(
            image,
            str(point["idx"]),
            (point["pix"][0] + 4, point["pix"][1] - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return image


def draw_straight_line_families_on_original(image, result, line_families, color):
    for family in line_families:
        for line_rho in family.get("line_rhos", []):
            segment = clip_oriented_rectified_line(
                family["line_angle_deg"],
                family["normal"],
                line_rho,
                result["rectified_geometry"]["rectified_width"],
                result["rectified_geometry"]["rectified_height"],
            )
            if segment is None:
                continue
            mapped_points = map_workspace_s2_rectified_points_to_image(segment, result["rectified_geometry"]["inverse_h"])
            if len(mapped_points) == 2:
                cv2.line(image, tuple(mapped_points[0]), tuple(mapped_points[1]), color, 1)
    return image


def draw_straight_line_families_on_rectified(result, line_families, color):
    image = to_bgr(result["rectified_ir"])
    for family in line_families:
        for line_rho in family.get("line_rhos", []):
            segment = clip_oriented_rectified_line(
                family["line_angle_deg"],
                family["normal"],
                line_rho,
                image.shape[1],
                image.shape[0],
            )
            if segment is None:
                continue
            cv2.line(image, tuple(segment[0]), tuple(segment[1]), color, 1)
    return image


def draw_curved_line_families_on_original(image, result, line_families, color):
    inverse_h = result["rectified_geometry"]["inverse_h"]
    for family in line_families:
        for curved_line in family.get("curved_lines", []):
            polyline = curved_line.get("polyline_points", [])
            if len(polyline) < 2:
                continue
            mapped_points = map_workspace_s2_rectified_points_to_image(polyline, inverse_h)
            if len(mapped_points) >= 2:
                cv2.polylines(image, [np.asarray(mapped_points, dtype=np.int32)], False, color, 1)
    return image


def draw_curved_line_families_on_rectified(result, line_families, color):
    image = to_bgr(result["rectified_ir"])
    for family in line_families:
        for curved_line in family.get("curved_lines", []):
            polyline = curved_line.get("polyline_points", [])
            if len(polyline) >= 2:
                cv2.polylines(image, [np.asarray(polyline, dtype=np.int32)], False, color, 1)
    return image


def summarize_line_families(line_families):
    summary = []
    for family in line_families:
        curved_lines = family.get("curved_lines") or []
        coverage_values = [
            float(curved_line.get("coverage", 0.0))
            for curved_line in curved_lines
            if curved_line.get("polyline_points")
        ]
        summary.append(
            {
                "angle_deg": round(float(family.get("line_angle_deg", 0.0)), 2),
                "axis_orientation": family.get("axis_orientation"),
                "period_estimator": family.get("period_estimator"),
                "estimate_method": family.get("estimate", {}).get("method"),
                "period": family.get("estimate", {}).get("period"),
                "phase": family.get("estimate", {}).get("phase"),
                "rhos": [round(float(line_rho), 2) for line_rho in family.get("line_rhos", [])],
                "line_count": len(family.get("line_rhos", [])),
                "curve_coverage": (
                    round(float(np.mean(coverage_values)), 3)
                    if coverage_values
                    else None
                ),
            }
        )
    return summary


def build_timed_variant(builder, *args, **kwargs):
    started_at = time.perf_counter()
    variant = builder(*args, **kwargs)
    variant["elapsed_ms"] = round((time.perf_counter() - started_at) * 1000.0, 2)
    return variant


def build_straight_variant(frame, result, variant_id, title, note, line_families, color):
    if len(line_families or []) < 2:
        original_image = draw_workspace_and_points(frame, result, [], line_color=color)
        rectified_image = to_bgr(result["rectified_ir"])
        note = f"{note}；该方案本帧未产出两组有效线族。"
        return {
            "id": variant_id,
            "title": title,
            "note": note,
            "type": "straight",
            "topology_source": "main_axis_rowcol" if variant_id != "01b_fft_axis_peaks" else "fft_axis_peaks",
            "line_families": line_families or [],
            "points": [],
            "rectified_points": [],
            "beam_filtered_point_count": 0,
            "curve_metrics": {},
            "original_image": original_image,
            "rectified_image": rectified_image,
            "summary": summarize_line_families(line_families or []),
        }

    raw_rectified_points = intersect_workspace_s2_oriented_line_families(
        line_families[0],
        line_families[1],
        result["rectified_geometry"]["rectified_width"],
        result["rectified_geometry"]["rectified_height"],
    )
    rectified_points, beam_filtered_point_count = filter_beam_points(result, raw_rectified_points)
    points = points_from_rectified_intersections(frame, result, rectified_points)
    original_image = draw_workspace_and_points(frame, result, points, line_color=color)
    original_image = draw_straight_line_families_on_original(original_image, result, line_families, color)
    rectified_image = draw_straight_line_families_on_rectified(result, line_families, color)
    return {
        "id": variant_id,
        "title": title,
        "note": note,
        "type": "straight",
        "topology_source": "main_axis_rowcol" if variant_id != "01b_fft_axis_peaks" else "fft_axis_peaks",
        "line_families": line_families,
        "points": points,
        "rectified_points": rectified_points,
        "beam_filtered_point_count": beam_filtered_point_count,
        "curve_metrics": {},
        "original_image": original_image,
        "rectified_image": rectified_image,
        "summary": summarize_line_families(line_families),
    }


def build_curved_variant(
    frame,
    result,
    variant_id,
    title,
    note,
    response_map,
    trace_method,
    score_mode,
    color,
    smoothness_weight=0.08,
    base_line_families=None,
    topology_source="main_axis_rowcol",
):
    base_line_families = base_line_families or result["line_families"]
    if topology_source == "fft_axis_peaks":
        note = f"{note.rstrip('。')}；本卡使用 FFT 线族拓扑骨架。"
    else:
        note = f"{note.rstrip('。')}；方案3/4/5/6 当前使用主链行/列峰值线族作为曲线拓扑骨架，不使用 FFT 线族。"
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
    raw_rectified_points = intersect_workspace_s2_curved_line_families(
        curved_families[0],
        curved_families[1],
        result["rectified_geometry"]["rectified_width"],
        result["rectified_geometry"]["rectified_height"],
    )
    if not raw_rectified_points:
        raw_rectified_points = intersect_workspace_s2_oriented_line_families(
            base_line_families[0],
            base_line_families[1],
            result["rectified_geometry"]["rectified_width"],
            result["rectified_geometry"]["rectified_height"],
        )
        note = f"{note}；当前曲线无交点时已用直线交点兜底。"
    rectified_points, beam_filtered_point_count = filter_beam_points(result, raw_rectified_points)
    points = points_from_rectified_intersections(frame, result, rectified_points)
    original_image = draw_workspace_and_points(frame, result, points, line_color=color)
    original_image = draw_curved_line_families_on_original(original_image, result, curved_families, color)
    rectified_image = draw_curved_line_families_on_rectified(result, curved_families, color)
    return {
        "id": variant_id,
        "title": title,
        "note": note,
        "type": "curved",
        "topology_source": topology_source,
        "line_families": curved_families,
        "points": points,
        "rectified_points": rectified_points,
        "beam_filtered_point_count": beam_filtered_point_count,
        "curve_metrics": summarize_curve_metrics(curved_families),
        "original_image": original_image,
        "rectified_image": rectified_image,
        "summary": summarize_line_families(curved_families),
    }


def build_variants(frame, result):
    valid_mask = result["rectified_valid"].astype(np.uint8)
    baseline_families = clone_line_families(result["line_families"])
    infrared_response = build_infrared_dark_response(result)
    combined_response = build_combined_depth_ir_response(result)
    fft_axis_families = build_workspace_s2_axis_aligned_line_families(
        result["response"],
        valid_mask,
        min_period=10,
        max_period=30,
        enable_local_peak_refine=False,
        enable_peak_support=True,
        enable_continuous_validation=False,
        enable_spacing_prune=True,
        period_estimator="fft",
    )
    fft_axis_families = clone_line_families(fft_axis_families[:2])
    build_initial_rhos_for_families(fft_axis_families)
    beam_exclusion_margin_mm = 130.0
    beam_bands = detect_workspace_s2_structural_edge_bands(
        result["response"],
        valid_mask,
    )
    raw_beam_mask = build_workspace_s2_structural_edge_suppression_mask(
        result["response"],
        valid_mask,
    )
    beam_mask = expand_workspace_s2_exclusion_mask_by_metric_margin(
        raw_beam_mask,
        result.get("rectified_geometry"),
        margin_mm=beam_exclusion_margin_mm,
    )
    result["beam_bands"] = beam_bands
    result["raw_beam_mask"] = raw_beam_mask
    result["beam_mask"] = beam_mask
    result["beam_exclusion_margin_mm"] = beam_exclusion_margin_mm

    variants = [
        build_timed_variant(
            build_straight_variant,
            frame,
            result,
            "01_current_theta_rho",
            "01 当前主链：行/列峰值正交网格",
            "当前 active PR-FPRG 输出，固定透视展开图行/列 profile 峰值，保持 spacing_pruned 后的两组正交直线族。",
            baseline_families,
            (0, 255, 0),
        ),
        build_timed_variant(
            build_straight_variant,
            frame,
            result,
            "01b_fft_axis_peaks",
            "01B FFT 行/列峰值正交网格",
            "在同一组合响应图上先用 FFT 频域主峰估计当前尺度行/列周期，再做 phase/peak support/spacing 收束；它是方案1的频域对照，不是已废弃的方案2。",
            fft_axis_families,
            (0, 180, 255),
        ),
        build_timed_variant(
            build_curved_variant,
            frame,
            result,
            "03_greedy_depth_curve",
            "03 局部 ridge 贪心曲线",
            "以当前行/列峰值正交网格为拓扑骨架，每个采样点沿法线独立找局部响应中心。",
            result["response"],
            trace_method="greedy",
            score_mode="response",
            color=(255, 90, 0),
        ),
        build_timed_variant(
            build_curved_variant,
            frame,
            result,
            "04_dp_depth_curve",
            "04 最小代价路径曲线",
            "仍沿法线找 ridge，但用动态规划约束相邻采样点偏移连续。",
            result["response"],
            trace_method="dynamic_programming",
            score_mode="response",
            color=(255, 0, 220),
            smoothness_weight=0.12,
        ),
        build_timed_variant(
            build_curved_variant,
            frame,
            result,
            "05_dp_ridge_curve",
            "05 ridge 约束曲线",
            "动态规划分数加入“中间强、两侧弱”的局部 ridge 判断。",
            result["response"],
            trace_method="dynamic_programming",
            score_mode="response_ridge",
            color=(0, 120, 255),
            smoothness_weight=0.12,
        ),
        build_timed_variant(
            build_curved_variant,
            frame,
            result,
            "06_ir_assisted_curve",
            "06 红外辅助曲线",
            "沿当前底层组合响应做同拓扑曲线收束，保留为对照候选。",
            combined_response,
            trace_method="dynamic_programming",
            score_mode="response_ridge",
            color=(180, 255, 60),
            smoothness_weight=0.12,
        ),
        build_timed_variant(
            build_curved_variant,
            frame,
            result,
            "03f_fft_greedy_depth_curve",
            "03F FFT拓扑：局部 ridge 贪心曲线",
            "以 FFT 行/列峰值正交网格为拓扑骨架，每个采样点沿法线独立找局部响应中心。",
            result["response"],
            trace_method="greedy",
            score_mode="response",
            color=(0, 200, 255),
            base_line_families=fft_axis_families,
            topology_source="fft_axis_peaks",
        ),
        build_timed_variant(
            build_curved_variant,
            frame,
            result,
            "04f_fft_dp_depth_curve",
            "04F FFT拓扑：最小代价路径曲线",
            "以 FFT 行/列峰值正交网格为拓扑骨架，沿法线找 ridge 并用动态规划约束相邻采样点偏移连续。",
            result["response"],
            trace_method="dynamic_programming",
            score_mode="response",
            color=(0, 150, 255),
            smoothness_weight=0.12,
            base_line_families=fft_axis_families,
            topology_source="fft_axis_peaks",
        ),
        build_timed_variant(
            build_curved_variant,
            frame,
            result,
            "05f_fft_dp_ridge_curve",
            "05F FFT拓扑：ridge 约束曲线",
            "以 FFT 行/列峰值正交网格为拓扑骨架，动态规划分数加入“中间强、两侧弱”的局部 ridge 判断。",
            result["response"],
            trace_method="dynamic_programming",
            score_mode="response_ridge",
            color=(0, 105, 255),
            smoothness_weight=0.12,
            base_line_families=fft_axis_families,
            topology_source="fft_axis_peaks",
        ),
        build_timed_variant(
            build_curved_variant,
            frame,
            result,
            "06f_fft_ir_assisted_curve",
            "06F FFT拓扑：红外辅助曲线",
            "以 FFT 行/列峰值正交网格为拓扑骨架，沿当前底层组合响应做同拓扑曲线收束。",
            combined_response,
            trace_method="dynamic_programming",
            score_mode="response_ridge",
            color=(0, 70, 220),
            smoothness_weight=0.12,
            base_line_families=fft_axis_families,
            topology_source="fft_axis_peaks",
        ),
    ]
    return variants, {
        "infrared_response": infrared_response,
        "combined_response": combined_response,
        "beam_response": result["response"],
        "beam_bands": beam_bands,
        "raw_beam_mask": raw_beam_mask,
        "beam_mask": beam_mask,
        "fft_axis_families": fft_axis_families,
    }


def write_report(output_dir, frame, result, variants, extra_images, display_gamma=1.0):
    display_gamma = max(0.05, float(display_gamma or 1.0))
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    image_dir = output_dir / "images"
    image_dir.mkdir(exist_ok=True)

    cv2.imwrite(str(image_dir / "00_source_workspace.png"), to_bgr(frame["ir"]))
    cv2.imwrite(str(image_dir / "00_selected_response.png"), render_response_heatmap(result["response"]))
    cv2.imwrite(str(image_dir / "00_depth_response.png"), render_response_heatmap(result["response"]))
    cv2.imwrite(str(image_dir / "00_infrared_response.png"), render_response_heatmap(extra_images["infrared_response"]))
    cv2.imwrite(str(image_dir / "00_combined_response.png"), render_response_heatmap(extra_images["combined_response"]))
    cv2.imwrite(str(image_dir / "00_source_workspace_gamma.png"), render_gray_gamma(frame["ir"], display_gamma))
    cv2.imwrite(str(image_dir / "00_selected_response_gamma.png"), render_response_heatmap_gamma(result["response"], display_gamma))
    cv2.imwrite(str(image_dir / "00_combined_response_gamma.png"), render_response_heatmap_gamma(extra_images["combined_response"], display_gamma))
    cv2.imwrite(
        str(image_dir / "00_combined_response_beam_mask_overlay.png"),
        render_response_with_beam_mask(extra_images["combined_response"], extra_images.get("beam_mask")),
    )
    cv2.imwrite(str(image_dir / "00_peak_supported_lines.png"), render_rectified_lines(result, "peak_supported"))
    cv2.imwrite(str(image_dir / "00_peak_spacing_pruned_lines.png"), render_removed_lines(result))
    fft_result = dict(result)
    fft_result["line_families"] = extra_images.get("fft_axis_families", [])
    cv2.imwrite(str(image_dir / "00_fft_peak_supported_lines.png"), render_rectified_lines(fft_result, "peak_supported"))
    cv2.imwrite(str(image_dir / "00_fft_peak_spacing_pruned_lines.png"), render_removed_lines(fft_result))
    cv2.imwrite(str(image_dir / "00_fft_frequency_spectrum.png"), render_fft_frequency_spectrum(extra_images.get("fft_axis_families", [])))
    beam_overlay = render_response_with_beam_mask(extra_images["beam_response"], extra_images.get("beam_mask"))
    cv2.imwrite(str(image_dir / "00_beam_mask_overlay.png"), beam_overlay)
    cv2.imwrite(str(image_dir / "00_beam_mask_binary.png"), render_beam_mask_binary(extra_images.get("beam_mask")))

    for variant in variants:
        original_name = f"{variant['id']}_original.png"
        rectified_name = f"{variant['id']}_rectified.png"
        cv2.imwrite(str(image_dir / original_name), variant["original_image"])
        cv2.imwrite(str(image_dir / rectified_name), variant["rectified_image"])
        variant["original_image_path"] = f"images/{original_name}"
        variant["rectified_image_path"] = f"images/{rectified_name}"

    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "stamp": frame.get("stamp"),
        "frame_source": frame.get("frame_source", "raw_world"),
        "response_name": result.get("response_name"),
        "response_source": result.get("response_source"),
        "selected_response_caption": selected_response_caption(result),
        "single_frame_elapsed_ms": result.get("single_frame_elapsed_ms"),
        "display_gamma": display_gamma,
        "line_stages": result.get("line_stages"),
        "beam_bands": result.get("beam_bands", []),
        "beam_exclusion_margin_mm": result.get("beam_exclusion_margin_mm", 0.0),
        "variants": [
            {
                "id": variant["id"],
                "title": variant["title"],
                "note": variant["note"],
                "type": variant["type"],
                "topology_source": variant.get("topology_source"),
                "point_count": len(variant["points"]),
                "elapsed_ms": variant.get("elapsed_ms"),
                "beam_filtered_point_count": variant["beam_filtered_point_count"],
                "curve_metrics": variant["curve_metrics"],
                "summary": variant["summary"],
                "first_20_points": variant["points"][:20],
            }
            for variant in variants
        ],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(json_safe(summary), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    cards_html = []
    for variant in variants:
        family_text = html.escape(
            json.dumps(json_safe(variant["summary"]), ensure_ascii=False)
        )
        metric_text = html.escape(
            json.dumps(
                json_safe(
                    {
                        "elapsed_ms": variant.get("elapsed_ms"),
                        "topology_source": variant.get("topology_source"),
                        "beam_bands": result.get("beam_bands", []),
                        "beam_exclusion_margin_mm": result.get("beam_exclusion_margin_mm", 0.0),
                        "beam_filtered_point_count": variant["beam_filtered_point_count"],
                        "curve_metrics": variant["curve_metrics"],
                    }
                ),
                ensure_ascii=False,
            )
        )
        cards_html.append(
            f"""
            <section class="variant">
              <div class="variant-header">
                <h2>{html.escape(variant["title"])}</h2>
                <span>{len(variant["points"])} points / {html.escape(str(variant.get("elapsed_ms", "-")))} ms</span>
              </div>
              <p>{html.escape(variant["note"])}</p>
              <div class="images">
                <figure>
                  <img src="{html.escape(variant['original_image_path'])}" alt="{html.escape(variant['title'])} original overlay">
                  <figcaption>原图叠加</figcaption>
                </figure>
                <figure>
                  <img src="{html.escape(variant['rectified_image_path'])}" alt="{html.escape(variant['title'])} rectified overlay">
                  <figcaption>透视展开后叠加</figcaption>
                </figure>
              </div>
              <pre>{metric_text}</pre>
              <pre>{family_text}</pre>
            </section>
            """.strip()
        )

    cards_html_text = "\n".join(cards_html)
    beam_bands_text = html.escape(
        json.dumps(json_safe(summary.get("beam_bands", [])), ensure_ascii=False)
    )
    beam_detected = bool(summary.get("beam_bands"))
    beam_status_text = (
        "当前帧已检测到梁筋，beam mask 会在组合响应叠加图和二值图中显示，并只用于最终绑扎点排除。"
        if beam_detected
        else "当前帧没有检测到梁筋，beam_bands=[]；因此 mask 二值图显示 NO BEAM MASK DETECTED，本帧没有发生梁筋过滤。"
    )
    point_counts = [int(variant.get("point_count", 0)) for variant in summary["variants"]]
    max_point_count = max(point_counts) if point_counts else 0
    min_point_count = min(point_counts) if point_counts else 0
    gamma_text = html.escape(str(round(float(display_gamma), 3)))
    diagnostic_html = f"""
    <section class="diagnostics">
      <h2>诊断结论</h2>
      <div class="diagnostic-grid">
        <article class="finding">
          <span class="tag">mask</span>
          <h3>梁筋 mask 当前状态</h3>
          <p>{html.escape(beam_status_text)}</p>
          <p>当前梁筋排除半径为 {html.escape(str(summary.get('beam_exclusion_margin_mm', 0.0)))} mm。钢筋线允许穿过梁筋；只有落入 mask 的最终绑扎点会被过滤。</p>
        </article>
        <article class="finding">
          <span class="tag">curve</span>
          <h3>曲线为什么没有完全贴钢筋</h3>
          <p>方案3/4/5/6 的曲线不是先语义分割“钢筋实体”，而是在已有行/列峰值线族骨架上沿响应图追局部 ridge。当地板缝、梁筋边缘、TCP 入侵或纹理在响应图上比真实钢筋更强时，贪心或动态规划会被更强响应牵引，曲线就会贴到假 ridge 上。</p>
          <p>因此曲线方案现在只作为收束对照，不直接替代主链；报告中的 3F/4F/5F/6F 是把拓扑骨架换成 FFT 峰值后的对照。</p>
        </article>
        <article class="finding">
          <span class="tag">scale</span>
          <h3>尺度变化为什么影响精度</h3>
          <p>离钢筋近时，单根钢筋宽度、交点凸起和线间距占更多像素，profile 峰值分离清楚，深度/红外信噪比也更高。离钢筋远时，钢筋宽度接近少量像素，透视展开插值会把 ridge 变窄或断裂，深度噪声和地板缝强度占比上升，多个峰更容易合并或被假峰替代。</p>
          <p>当前帧各方案点数范围为 {min_point_count} - {max_point_count}，单帧主链耗时 {html.escape(str(summary.get('single_frame_elapsed_ms', '-')))} ms。</p>
        </article>
      </div>
    </section>
    <section class="diagnostics">
      <h2>全尺度鲁棒方案</h2>
      <ol class="roadmap">
        <li>先按相机标定和工作区透视，把 rectified 图重采样到固定物理分辨率，例如每像素固定毫米数，而不是让同一算法直接吃任意像素尺度。</li>
        <li>用多尺度 ridge 响应生成钢筋概率图：小尺度保细钢筋，大尺度抗远距噪声；FFT/profile 只估周期和相位，不再单独决定最终线。</li>
        <li>梁筋、TCP、地板缝作为独立 mask/负样本通道进入评分，梁筋只过滤点，不切断普通钢筋；地板缝通过宽度、连续方向、深度突变和时序稳定性降权。</li>
        <li>线族用全局网格优化：周期、相位、局部曲率、交点置信度一起求解；曲线只能在全局拓扑约束范围内微调，不能被单条强假纹理带跑。</li>
        <li>对远距画面增加时序多帧中值/投票，必要时引导相机进入“识别尺度窗口”；如果钢筋在图上已经低于可分辨像素宽度，单帧算法无法物理保证恢复。</li>
        <li>最终工程化时把响应、FFT/profile、多尺度 ridge 和图优化热点下沉到 C++/SIMD 或二进制程序，目标保持 100ms 以内。</li>
      </ol>
    </section>
    <section class="diagnostics">
      <h2>历史技术路径</h2>
      <ul class="history-list">
        <li>旧链路：RANSAC + Hough + pre_img，已归档，不再作为运行门控。</li>
        <li>PR-FPRG 初始链：透视展开工作区，基于深度响应做周期/相位回归和峰值找线。</li>
        <li>方向自适应 theta/rho 链：估两组真实方向、沿法线 profile 找 rho 峰、连续钢筋条验证、ridge 判断、斜线求交；后来发现现场希望行列正交输出，转为对照。</li>
        <li>红外最终 rho 微调：尝试用红外 ridge 覆盖最终 rho，但现场对齐不如早期方案，已归档为后续参考。</li>
        <li>方案3/4/5/6 曲线链：局部 ridge 贪心、动态规划、ridge 约束、红外辅助，用于解决钢筋非直线问题，但目前容易被地板缝/强边缘牵引。</li>
        <li>当前主链：组合响应 combined_depth_ir_darkline + rectified 行/列峰值正交网格；方案2废弃，曲线只做报告对照。</li>
        <li>最新对照：01B 和 3F/4F/5F/6F 使用 FFT 频域线族拓扑，并固定输出直角坐标系驼峰峰值图。</li>
      </ul>
    </section>
    """
    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PR-FPRG 方案效果对比</title>
  <style>
    *,
    *::before,
    *::after {{
      box-sizing: border-box;
    }}
    :root {{
      color-scheme: light;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f4f6f8;
      color: #17202a;
    }}
    html {{
      max-width: 100%;
      overflow-x: hidden;
    }}
    body {{
      margin: 0;
      max-width: 100%;
      overflow-x: hidden;
    }}
    header {{
      padding: 24px 28px 18px;
      background: #ffffff;
      border-bottom: 1px solid #d8dee6;
      max-width: 100vw;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 26px;
      letter-spacing: 0;
    }}
    .meta {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      font-size: 13px;
      color: #52606d;
    }}
    .meta span {{
      padding: 4px 8px;
      border: 1px solid #d8dee6;
      border-radius: 6px;
      background: #f8fafc;
    }}
    main {{
      padding: 20px 28px 36px;
      display: grid;
      gap: 18px;
      width: 100%;
      max-width: 100vw;
      min-width: 0;
    }}
    .source-strip,
    .diagnostics,
    .variant {{
      background: #ffffff;
      border: 1px solid #d8dee6;
      border-radius: 8px;
      padding: 16px;
      max-width: 100%;
      min-width: 0;
    }}
    .source-strip h2,
    .diagnostics h2,
    .variant h2 {{
      margin: 0;
      font-size: 18px;
      letter-spacing: 0;
    }}
    .diagnostic-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 300px), 1fr));
      gap: 12px;
      margin-top: 12px;
    }}
    .finding {{
      border: 1px solid #dbe4ee;
      border-radius: 8px;
      padding: 12px;
      background: #fbfcfe;
      min-width: 0;
    }}
    .finding h3 {{
      margin: 6px 0 8px;
      font-size: 16px;
      letter-spacing: 0;
    }}
    .tag {{
      display: inline-flex;
      align-items: center;
      min-height: 22px;
      padding: 2px 8px;
      border-radius: 999px;
      background: #eef6ff;
      border: 1px solid #bfd7ff;
      color: #1d4ed8;
      font-size: 12px;
      font-weight: 700;
    }}
    .roadmap,
    .history-list {{
      margin: 12px 0 0;
      padding-left: 22px;
      color: #334155;
      line-height: 1.62;
      font-size: 14px;
    }}
    .variant-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      flex-wrap: wrap;
      min-width: 0;
    }}
    .variant-header span {{
      font-size: 13px;
      color: #0f766e;
      background: #e6fffb;
      border: 1px solid #99f6e4;
      border-radius: 6px;
      padding: 4px 8px;
      white-space: nowrap;
    }}
    p {{
      margin: 8px 0 12px;
      color: #52606d;
      line-height: 1.5;
      font-size: 14px;
    }}
    code {{
      display: inline-block;
      max-width: 100%;
      overflow-x: auto;
      vertical-align: bottom;
      white-space: nowrap;
    }}
    .images {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 320px), 1fr));
      gap: 14px;
      width: 100%;
      min-width: 0;
    }}
    figure {{
      margin: 0;
      border: 1px solid #e1e7ef;
      border-radius: 8px;
      overflow: hidden;
      background: #0f172a;
      max-width: 100%;
      min-width: 0;
    }}
    img {{
      display: block;
      width: 100%;
      max-width: 100%;
      height: auto;
      max-height: min(76vh, 920px);
      object-fit: contain;
      background: #020617;
      image-rendering: auto;
    }}
    figcaption {{
      padding: 8px 10px;
      font-size: 13px;
      color: #334155;
      background: #f8fafc;
      border-top: 1px solid #e1e7ef;
    }}
    pre {{
      margin: 12px 0 0;
      padding: 10px;
      max-width: 100%;
      overflow-x: auto;
      border: 1px solid #e1e7ef;
      border-radius: 6px;
      background: #f8fafc;
      font-size: 12px;
      line-height: 1.45;
    }}
    @media (max-width: 720px) {{
      header,
      main {{
        padding-left: 14px;
        padding-right: 14px;
      }}
      .images {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>PR-FPRG 方案效果对比</h1>
    <div class="meta">
      <span>生成时间：{html.escape(summary['generated_at'])}</span>
      <span>帧源：{html.escape(str(summary['frame_source']))}</span>
      <span>响应图：{html.escape(str(summary['response_name']))}</span>
      <span>主链单帧：{html.escape(str(summary.get('single_frame_elapsed_ms', '-')))} ms</span>
      <span>display_gamma：{gamma_text}</span>
    </div>
  </header>
  <main>
    <section class="source-strip">
      <h2>输入与响应图</h2>
      <p>红色叠加区域是梁筋±13cm图谱排除掩膜，只用于最终绑扎点排除，不删除整条钢筋线。</p>
      <p>当前帧梁筋检测 beam_bands：<code>{beam_bands_text}</code>；为空时说明本帧未触发梁筋过滤，mask 图会显示 NO BEAM MASK DETECTED。</p>
      <p>方案3/4/5/6 当前使用主链行/列峰值线族作为曲线拓扑骨架，不使用 FFT 线族；方案3F/4F/5F/6F 是同算法换 FFT 线族拓扑骨架的对照。</p>
      <div class="images">
        <figure><img src="images/00_source_workspace.png" alt="source"><figcaption>红外输入</figcaption></figure>
        <figure><img src="images/00_selected_response.png" alt="selected response"><figcaption>{html.escape(summary['selected_response_caption'])}</figcaption></figure>
        <figure><img src="images/00_source_workspace_gamma.png" alt="source high gamma"><figcaption>高伽马红外输入 display_gamma={gamma_text}</figcaption></figure>
        <figure><img src="images/00_selected_response_gamma.png" alt="selected response high gamma"><figcaption>高伽马底层响应 display_gamma={gamma_text}</figcaption></figure>
        <figure><img src="images/00_combined_response_gamma.png" alt="combined response high gamma"><figcaption>高伽马组合响应 display_gamma={gamma_text}</figcaption></figure>
        <figure><img src="images/00_peak_supported_lines.png" alt="peak supported lines"><figcaption>峰值图：peak-supported 候选线</figcaption></figure>
        <figure><img src="images/00_peak_spacing_pruned_lines.png" alt="peak spacing pruned lines"><figcaption>峰值图：绿色保留 / 红色剔除</figcaption></figure>
        <figure><img src="images/00_fft_peak_supported_lines.png" alt="fft peak supported lines"><figcaption>FFT 峰值图：peak-supported 候选线</figcaption></figure>
        <figure><img src="images/00_fft_peak_spacing_pruned_lines.png" alt="fft peak spacing pruned lines"><figcaption>FFT 峰值图：绿色保留 / 红色剔除</figcaption></figure>
        <figure><img src="images/00_fft_frequency_spectrum.png" alt="fft frequency spectrum"><figcaption>FFT 频域谱图：直角坐标系驼峰峰值图</figcaption></figure>
        <figure><img src="images/00_infrared_response.png" alt="infrared response"><figcaption>红外暗线响应</figcaption></figure>
        <figure><img src="images/00_combined_response.png" alt="combined response"><figcaption>组合响应</figcaption></figure>
        <figure><img src="images/00_combined_response_beam_mask_overlay.png" alt="combined response beam mask"><figcaption>组合响应 + 梁筋±13cm过滤叠加</figcaption></figure>
        <figure><img src="images/00_beam_mask_binary.png" alt="beam mask binary"><figcaption>梁筋mask二值图</figcaption></figure>
        <figure><img src="images/00_beam_mask_overlay.png" alt="beam mask"><figcaption>梁筋±13cm图谱排除 mask</figcaption></figure>
      </div>
    </section>
    {diagnostic_html}
    {cards_html_text}
  </main>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")
    return output_dir / "index.html"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--response-name-filter",
        default=None,
        help="逗号分隔的响应名称过滤；例如 depth_background_minus_filled 强制生成纯深度响应报告。",
    )
    parser.add_argument(
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"pr_fprg_scheme_comparison_{time.strftime('%Y%m%d_%H%M%S')}"),
    )
    parser.add_argument(
        "--display-gamma",
        type=float,
        default=1.0,
        help="仅用于报告可视化的显示伽马；不改变识别算法。",
    )
    args = parser.parse_args()

    rospy.init_node("pr_fprg_scheme_comparison", anonymous=True, disable_signals=True)
    try:
        frame = capture_synced_frame(args.timeout)
    except RuntimeError as exc:
        rospy.logwarn(
            "PointAI_log: raw_world同步帧不可用，改用 depth+IR 兜底生成方案对比：%s",
            exc,
        )
        frame = capture_depth_ir_frame(args.timeout)

    pipeline_started_at = time.perf_counter()
    result = run_peak_supported_pr_fprg(frame, response_name_filter=args.response_name_filter)
    result["single_frame_elapsed_ms"] = round((time.perf_counter() - pipeline_started_at) * 1000.0, 2)
    variants, extra_images = build_variants(frame, result)
    report_path = write_report(args.output_dir, frame, result, variants, extra_images, display_gamma=args.display_gamma)
    rospy.loginfo("PointAI_log: PR-FPRG方案对比报告：%s", report_path)
    print(str(report_path))


if __name__ == "__main__":
    main()
