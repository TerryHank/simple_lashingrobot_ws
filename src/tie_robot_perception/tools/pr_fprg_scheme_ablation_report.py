#!/usr/bin/env python3

"""Generate an HTML report that ablates PR-FPRG stages for scheme 1."""

from __future__ import annotations

import argparse
import html
import json
import statistics
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
    render_response_heatmap,
    render_rectified_lines,
    render_removed_lines,
    to_bgr,
    to_u8,
)
from pr_fprg_scheme_comparison import (  # noqa: E402
    build_combined_depth_ir_response,
    build_infrared_dark_response,
    clone_line_families,
    draw_curved_line_families_on_original,
    draw_curved_line_families_on_rectified,
    draw_straight_line_families_on_original,
    draw_straight_line_families_on_rectified,
    points_from_rectified_intersections,
    refine_line_families_by_auxiliary_profile,
)
from pr_fprg_stage_ablation import (  # noqa: E402
    point_drift_stats,
    prepare_frame_inputs,
    run_line_family_variant,
    summarize_families,
)
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_structural_edge_suppression_mask,
    build_workspace_s2_curved_line_families,
    detect_workspace_s2_structural_edge_bands,
    expand_workspace_s2_exclusion_mask_by_metric_margin,
    filter_workspace_s2_rectified_points_outside_mask,
    intersect_workspace_s2_curved_line_families,
    intersect_workspace_s2_oriented_line_families,
)


REALTIME_TARGET_SCHEME_ID = "01_current_theta_rho"


SCHEMES = [
    {
        "id": "01_current_theta_rho",
        "label": "01 当前主链：行/列峰值正交网格",
        "short_label": "方案 1",
        "kind": "straight",
        "description": "透视展开后使用组合响应，只对固定行/列 profile 做峰值识别，以正交网格求交；梁筋±13cm范围只在最终点级过滤。",
        "color": (0, 255, 0),
        "tone": "baseline",
    },
    {
        "id": "03_greedy_depth_curve",
        "label": "03 局部 ridge 贪心曲线",
        "short_label": "方案 3",
        "kind": "curve",
        "description": "以当前方案1正交网格为拓扑骨架，每个采样点沿法线独立找局部响应中心；用于观察曲线追踪是否会被地板缝牵引。",
        "trace_method": "greedy",
        "score_mode": "response",
        "response_mode": "selected",
        "color": (255, 90, 0),
        "tone": "warn",
    },
    {
        "id": "04_dp_depth_curve",
        "label": "04 最小代价路径曲线",
        "short_label": "方案 4",
        "kind": "curve",
        "description": "沿法线找 ridge，并用动态规划约束相邻采样点偏移连续；保留为曲线收束候选。",
        "trace_method": "dynamic_programming",
        "score_mode": "response",
        "response_mode": "selected",
        "smoothness_weight": 0.12,
        "color": (255, 0, 220),
        "tone": "warn",
    },
    {
        "id": "05_dp_ridge_curve",
        "label": "05 ridge 约束曲线",
        "short_label": "方案 5",
        "kind": "curve",
        "description": "动态规划分数加入“中间强、两侧弱”的 ridge 判断，用于抑制曲线贴地板缝。",
        "trace_method": "dynamic_programming",
        "score_mode": "response_ridge",
        "response_mode": "selected",
        "smoothness_weight": 0.12,
        "color": (0, 120, 255),
        "tone": "baseline",
    },
    {
        "id": "06_ir_assisted_curve",
        "label": "06 红外辅助曲线",
        "short_label": "方案 6",
        "kind": "curve",
        "description": "沿当前组合响应做同拓扑曲线收束，保留为红外辅助对照候选。",
        "trace_method": "dynamic_programming",
        "score_mode": "response_ridge",
        "response_mode": "combined",
        "smoothness_weight": 0.12,
        "color": (180, 255, 60),
        "tone": "warn",
    },
]


STAGE_VARIANTS = [
    {
        "id": "full",
        "label": "全流程",
        "stage": "基准",
        "description": "当前主链：组合响应、固定行/列 profile 峰值、spacing 收束；梁筋±13cm范围只在最终点级过滤。",
        "conclusion": "方案内部基准。",
        "tone": "baseline",
        "enable_local_peak_refine": False,
        "enable_continuous_validation": False,
        "enable_spacing_prune": True,
    },
    {
        "id": "skip_continuous",
        "label": "保持关闭连续/ridge 验证",
        "stage": "连续验证",
        "description": "行/列峰值主链默认不沿钢筋方向做连续条验证和 ridge 检查，本项作为历史对照保留。",
        "conclusion": "当前主链默认关闭。",
        "tone": "ok",
        "enable_continuous_validation": False,
        "enable_spacing_prune": True,
    },
    {
        "id": "skip_spacing",
        "label": "关闭 spacing prune",
        "stage": "间距筛选",
        "description": "保留行/列 profile 峰值，但不做间距收束。",
        "conclusion": "用于观察连续验证后是否仍有密集重复线。",
        "tone": "warn",
        "enable_continuous_validation": True,
        "enable_spacing_prune": False,
    },
    {
        "id": "skip_peak_support",
        "label": "去掉 peak 支撑过滤",
        "stage": "peak 支撑",
        "description": "周期相位给出的候选线不再要求 profile 局部峰值支撑。",
        "conclusion": "可能无法稳定产出两组线族。",
        "tone": "danger",
        "enable_peak_support": False,
    },
    {
        "id": "profile_only_spacing",
        "label": "只保留 profile 周期 + spacing",
        "stage": "profile-only",
        "description": "关闭局部 refine、peak 支撑和连续验证，只保留周期线与 spacing prune。",
        "conclusion": "通常速度快但假点过多。",
        "tone": "danger",
        "enable_local_peak_refine": False,
        "enable_peak_support": False,
        "enable_continuous_validation": False,
        "enable_spacing_prune": True,
    },
    {
        "id": "profile_only_raw",
        "label": "只保留 profile 周期原始线",
        "stage": "profile-only",
        "description": "只根据固定行/列 profile 周期相位生成线，所有后续筛选都关闭。",
        "conclusion": "最快但最容易产生大量假点。",
        "tone": "danger",
        "enable_local_peak_refine": False,
        "enable_peak_support": False,
        "enable_continuous_validation": False,
        "enable_spacing_prune": False,
    },
]


def safe_json(value):
    if isinstance(value, dict):
        return {str(key): safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return safe_json(value.tolist())
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def build_result_like(prepared, response=None):
    result_like = {
        "manual_workspace": prepared["manual_workspace"],
        "workspace_mask": prepared["workspace_mask"],
        "rectified_geometry": prepared["rectified_geometry"],
        "rectified_ir": prepared["rectified_ir"],
        "rectified_valid": prepared["rectified_valid"],
    }
    if response is not None:
        result_like["response"] = response
    return result_like


def draw_points(image, points, color):
    for point in points:
        pixel = tuple(int(value) for value in point["pix"])
        cv2.circle(image, pixel, 4, (0, 255, 255), -1)
        cv2.circle(image, pixel, 6, color, 1)
        cv2.putText(
            image,
            str(point["idx"]),
            (pixel[0] + 5, pixel[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return image


def render_original_overlay(frame, prepared, scheme_result):
    result_like = build_result_like(prepared, scheme_result.get("response"))
    color = tuple(int(value) for value in scheme_result["color"])
    image = cv2.cvtColor(to_u8(frame["ir"]), cv2.COLOR_GRAY2BGR)
    polygon = np.asarray(prepared["manual_workspace"]["corner_pixels"], dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [polygon], True, (232, 232, 232), 2)
    if scheme_result["geometry_type"] == "curved":
        image = draw_curved_line_families_on_original(image, result_like, scheme_result["line_families"], color)
    else:
        image = draw_straight_line_families_on_original(image, result_like, scheme_result["line_families"], color)
    return draw_points(image, scheme_result["points"], color)


def render_rectified_overlay(prepared, scheme_result):
    result_like = build_result_like(prepared, scheme_result.get("response"))
    color = tuple(int(value) for value in scheme_result["color"])
    if scheme_result["geometry_type"] == "curved":
        image = draw_curved_line_families_on_rectified(result_like, scheme_result["line_families"], color)
    else:
        image = draw_straight_line_families_on_rectified(result_like, scheme_result["line_families"], color)
    for point_index, point in enumerate(scheme_result["rectified_points"], start=1):
        pixel = (int(round(float(point[0]))), int(round(float(point[1]))))
        if pixel[0] < 0 or pixel[1] < 0 or pixel[0] >= image.shape[1] or pixel[1] >= image.shape[0]:
            continue
        cv2.circle(image, pixel, 4, (0, 255, 255), -1)
        cv2.circle(image, pixel, 6, color, 1)
        cv2.putText(
            image,
            str(point_index),
            (pixel[0] + 5, pixel[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return image


def build_straight_scheme_payload(frame, prepared, line_families, response_map, scheme):
    result_like = build_result_like(prepared, response_map)
    scheme_families = clone_line_families(line_families)
    rectified_points = intersect_workspace_s2_oriented_line_families(
        scheme_families[0],
        scheme_families[1],
        prepared["rectified_geometry"]["rectified_width"],
        prepared["rectified_geometry"]["rectified_height"],
    )
    structural_edge_response = prepared.get("structural_edge_response", response_map)
    beam_mask = build_workspace_s2_structural_edge_suppression_mask(
        structural_edge_response,
        prepared["rectified_valid"].astype(np.uint8),
    )
    beam_mask = expand_workspace_s2_exclusion_mask_by_metric_margin(
        beam_mask,
        prepared["rectified_geometry"],
        margin_mm=130.0,
    )
    rectified_points = filter_workspace_s2_rectified_points_outside_mask(
        rectified_points,
        beam_mask,
    )
    points = points_from_rectified_intersections(frame, result_like, rectified_points)
    return {
        "geometry_type": "straight",
        "line_families": scheme_families,
        "rectified_points": rectified_points,
        "points": points,
        "families": summarize_families(scheme_families),
        "response": response_map,
        "color": scheme["color"],
    }


def build_ir_refined_scheme_payload(frame, prepared, line_families, response_map, scheme):
    result_like = build_result_like(prepared, response_map)
    infrared_response = build_infrared_dark_response(result_like)
    scheme_families = refine_line_families_by_auxiliary_profile(
        clone_line_families(line_families),
        infrared_response,
        prepared["rectified_valid"].astype(np.uint8),
        search_radius_px=8,
    )
    build_initial_rhos_for_families(scheme_families)
    rectified_points = intersect_workspace_s2_oriented_line_families(
        scheme_families[0],
        scheme_families[1],
        prepared["rectified_geometry"]["rectified_width"],
        prepared["rectified_geometry"]["rectified_height"],
    )
    structural_edge_response = prepared.get("structural_edge_response", response_map)
    beam_mask = build_workspace_s2_structural_edge_suppression_mask(
        structural_edge_response,
        prepared["rectified_valid"].astype(np.uint8),
    )
    beam_mask = expand_workspace_s2_exclusion_mask_by_metric_margin(
        beam_mask,
        prepared["rectified_geometry"],
        margin_mm=130.0,
    )
    rectified_points = filter_workspace_s2_rectified_points_outside_mask(
        rectified_points,
        beam_mask,
    )
    points = points_from_rectified_intersections(frame, result_like, rectified_points)
    return {
        "geometry_type": "straight",
        "line_families": scheme_families,
        "rectified_points": rectified_points,
        "points": points,
        "families": summarize_families(scheme_families),
        "response": response_map,
        "color": scheme["color"],
    }


def build_curved_scheme_payload(frame, prepared, line_families, response_map, scheme):
    result_like = build_result_like(prepared, response_map)
    trace_response = response_map
    if scheme.get("response_mode") == "combined":
        trace_response = build_combined_depth_ir_response(result_like)

    curved_families = build_workspace_s2_curved_line_families(
        trace_response,
        prepared["rectified_valid"].astype(np.uint8),
        clone_line_families(line_families),
        trace_method=scheme["trace_method"],
        score_mode=scheme["score_mode"],
        search_radius_px=9,
        sample_step_px=scheme.get("sample_step_px", 4),
        min_response_ratio=0.15,
        smoothing_window_samples=5,
        smoothness_weight=scheme.get("smoothness_weight", 0.12),
    )
    rectified_points = intersect_workspace_s2_curved_line_families(
        curved_families[0],
        curved_families[1],
        prepared["rectified_geometry"]["rectified_width"],
        prepared["rectified_geometry"]["rectified_height"],
    )
    fallback_to_straight = False
    if not rectified_points:
        fallback_to_straight = True
        rectified_points = intersect_workspace_s2_oriented_line_families(
            line_families[0],
            line_families[1],
            prepared["rectified_geometry"]["rectified_width"],
            prepared["rectified_geometry"]["rectified_height"],
        )
    structural_edge_response = prepared.get("structural_edge_response", response_map)
    beam_mask = build_workspace_s2_structural_edge_suppression_mask(
        structural_edge_response,
        prepared["rectified_valid"].astype(np.uint8),
    )
    beam_mask = expand_workspace_s2_exclusion_mask_by_metric_margin(
        beam_mask,
        prepared["rectified_geometry"],
        margin_mm=130.0,
    )
    rectified_points = filter_workspace_s2_rectified_points_outside_mask(
        rectified_points,
        beam_mask,
    )
    points = points_from_rectified_intersections(frame, result_like, rectified_points)
    families = [
        {
            "angle_deg": round(float(family.get("line_angle_deg", 0.0)), 3),
            "line_rhos": [round(float(rho), 3) for rho in family.get("line_rhos", [])],
            "curve_count": len(family.get("curved_lines", [])),
            "mean_coverage": (
                round(
                    float(
                        np.mean(
                            [
                                float(curved_line.get("coverage", 0.0))
                                for curved_line in family.get("curved_lines", [])
                                if curved_line.get("polyline_points")
                            ]
                        )
                    ),
                    3,
                )
                if any(curved_line.get("polyline_points") for curved_line in family.get("curved_lines", []))
                else None
            ),
        }
        for family in curved_families
    ]
    return {
        "geometry_type": "curved",
        "line_families": curved_families,
        "rectified_points": rectified_points,
        "points": points,
        "families": families,
        "response": trace_response,
        "color": scheme["color"],
        "fallback_to_straight": fallback_to_straight,
    }


def build_scheme_payload(frame, prepared, line_families, response_map, scheme):
    if scheme["kind"] == "straight":
        return build_straight_scheme_payload(frame, prepared, line_families, response_map, scheme)
    if scheme["kind"] == "ir_refined_straight":
        return build_ir_refined_scheme_payload(frame, prepared, line_families, response_map, scheme)
    if scheme["kind"] == "curve":
        return build_curved_scheme_payload(frame, prepared, line_families, response_map, scheme)
    raise ValueError(f"unknown scheme kind: {scheme['kind']}")


def time_scheme_stage_variant(frame, prepared, scheme, stage_variant, locked_response_name, repeat_count):
    timings = []
    payload = None
    for _ in range(repeat_count):
        started = time.perf_counter()
        family_result, _rectified_intersections, _runtime_result_like = run_line_family_variant(
            prepared,
            stage_variant,
            response_name_filter=locked_response_name,
        )
        scheme_payload = build_scheme_payload(
            frame,
            prepared,
            family_result["line_families"],
            family_result["response"],
            scheme,
        )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        timings.append(elapsed_ms)
        payload = {
            "family_result": family_result,
            "scheme_result": scheme_payload,
        }
    return timings, payload


def verdict_for_result(result):
    if result.get("failure"):
        return "失败"
    if result["stage_variant_id"] == "full":
        return "方案基准"
    drift = result["drift_vs_scheme_full"]
    if drift["mean_px"] == 0.0 and drift["point_count_delta"] == 0:
        return "可跳过"
    if abs(drift["point_count_delta"]) > 0:
        return "不能删"
    if drift["max_px"] is not None and drift["max_px"] > 8.0:
        return "不能删"
    return "需复核"


def mean_or_none(values):
    return None if not values else float(statistics.mean(values))


def median_or_none(values):
    return None if not values else float(statistics.median(values))


def number_text(value, digits=2):
    return "-" if value is None else f"{float(value):.{digits}f}"


def render_report(output_dir, frame, prepared, results, metadata):
    output_dir = Path(output_dir)
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(image_dir / "00_ir_input.png"), to_bgr(frame["ir"]))
    cv2.imwrite(str(image_dir / "00_rectified_ir.png"), to_bgr(prepared["rectified_ir"]))

    baseline_response = None
    baseline_full_result = None
    for result in results:
        if result["scheme_id"] == REALTIME_TARGET_SCHEME_ID and result["stage_variant_id"] == "full":
            baseline_response = result.get("response")
            baseline_full_result = result
            break
    if baseline_full_result is None:
        for result in results:
            if result["stage_variant_id"] == "full":
                baseline_response = result.get("response")
                baseline_full_result = result
                break
    if baseline_response is not None:
        cv2.imwrite(str(image_dir / "00_response.png"), render_response_heatmap(baseline_response))
        structural_edge_response = prepared.get("structural_edge_response", baseline_response)
        beam_mask = build_workspace_s2_structural_edge_suppression_mask(
            structural_edge_response,
            prepared["rectified_valid"].astype(np.uint8),
        )
        beam_mask = expand_workspace_s2_exclusion_mask_by_metric_margin(
            beam_mask,
            prepared["rectified_geometry"],
            margin_mm=130.0,
        )
        beam_overlay = render_response_heatmap(structural_edge_response)
        if np.any(beam_mask):
            red_layer = np.zeros_like(beam_overlay)
            red_layer[:, :] = (30, 30, 255)
            beam_overlay[beam_mask] = cv2.addWeighted(beam_overlay[beam_mask], 0.35, red_layer[beam_mask], 0.65, 0.0)
        beam_bands = detect_workspace_s2_structural_edge_bands(
            structural_edge_response,
            prepared["rectified_valid"].astype(np.uint8),
        )
        for band in beam_bands:
            if band["axis"] == "x":
                cv2.rectangle(
                    beam_overlay,
                    (int(band["start"]), 0),
                    (int(band["end"]), beam_overlay.shape[0] - 1),
                    (255, 255, 255),
                    1,
                )
            else:
                cv2.rectangle(
                    beam_overlay,
                    (0, int(band["start"])),
                    (beam_overlay.shape[1] - 1, int(band["end"])),
                    (255, 255, 255),
                    1,
                )
        cv2.imwrite(str(image_dir / "00_beam_mask_overlay.png"), beam_overlay)
    if baseline_full_result is not None:
        peak_result = {
            "rectified_ir": prepared["rectified_ir"],
            "line_families": baseline_full_result.get("line_families", []),
        }
        cv2.imwrite(str(image_dir / "00_peak_supported_lines.png"), render_rectified_lines(peak_result, "peak_supported"))
        cv2.imwrite(str(image_dir / "00_peak_spacing_pruned_lines.png"), render_removed_lines(peak_result))

    for result in results:
        image_prefix = f"{result['scheme_id']}__{result['stage_variant_id']}"
        original_name = f"{image_prefix}_original.png"
        rectified_name = f"{image_prefix}_rectified.png"
        if result.get("failure"):
            result["original_image_path"] = None
            result["rectified_image_path"] = None
        else:
            original = render_original_overlay(frame, prepared, result)
            rectified = render_rectified_overlay(prepared, result)
            cv2.imwrite(str(image_dir / original_name), original)
            cv2.imwrite(str(image_dir / rectified_name), rectified)
            result["original_image_path"] = f"images/{original_name}"
            result["rectified_image_path"] = f"images/{rectified_name}"

    compact_results = [
        {
            "scheme_id": result["scheme_id"],
            "scheme_label": result["scheme_label"],
            "stage_variant_id": result["stage_variant_id"],
            "stage_variant_label": result["stage_variant_label"],
            "mean_ms": result.get("mean_ms"),
            "median_ms": result.get("median_ms"),
            "min_ms": result.get("min_ms"),
            "max_ms": result.get("max_ms"),
            "point_count": result.get("point_count"),
            "drift_vs_scheme_full": result.get("drift_vs_scheme_full"),
            "drift_vs_reference_full": result.get("drift_vs_reference_full"),
            "verdict": result.get("verdict"),
            "families": result.get("families", []),
            "failure": result.get("failure"),
            "fallback_to_straight": result.get("fallback_to_straight", False),
            "first_20_points": result.get("points", [])[:20],
        }
        for result in results
    ]
    (output_dir / "summary.json").write_text(
        json.dumps(
            safe_json(
                {
                    "metadata": metadata,
                    "schemes": SCHEMES,
                    "stage_variants": STAGE_VARIANTS,
                    "results": compact_results,
                }
            ),
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    scheme_full_rows = []
    for scheme in SCHEMES:
        result = next(
            item
            for item in results
            if item["scheme_id"] == scheme["id"] and item["stage_variant_id"] == "full"
        )
        drift = result["drift_vs_reference_full"]
        scheme_full_rows.append(
            f"""
            <tr>
              <td>{html.escape(scheme['short_label'])}</td>
              <td>{html.escape(scheme['label'])}</td>
              <td>{number_text(result.get('mean_ms'))}</td>
              <td>{number_text(result.get('median_ms'))}</td>
              <td>{result.get('point_count', 0)}</td>
              <td>{number_text(drift.get('mean_px'))}</td>
              <td>{number_text(drift.get('max_px'))}</td>
              <td>{drift.get('point_count_delta')}</td>
            </tr>
            """
        )

    scheme_sections = []
    for scheme in SCHEMES:
        scheme_results = [item for item in results if item["scheme_id"] == scheme["id"]]
        rows = []
        cards = []
        for result in scheme_results:
            scheme_drift = result["drift_vs_scheme_full"]
            global_drift = result["drift_vs_reference_full"]
            mean_ms = number_text(result.get("mean_ms"))
            median_ms = number_text(result.get("median_ms"))
            max_ms = number_text(result.get("max_ms"))
            scheme_max_drift = number_text(scheme_drift.get("max_px"))
            global_max_drift = number_text(global_drift.get("max_px"))
            tone = result.get("tone", "warn")
            rows.append(
                f"""
                <tr class="{html.escape(tone)}">
                  <td>{html.escape(result['stage'])}</td>
                  <td>{html.escape(result['stage_variant_label'])}</td>
                  <td>{mean_ms}</td>
                  <td>{median_ms}</td>
                  <td>{max_ms}</td>
                  <td>{result.get('point_count', 0)}</td>
                  <td>{scheme_max_drift}</td>
                  <td>{scheme_drift.get('point_count_delta')}</td>
                  <td>{global_max_drift}</td>
                  <td><span class="pill {html.escape(tone)}">{html.escape(result['verdict'])}</span></td>
                </tr>
                """
            )
            family_text = html.escape(json.dumps(safe_json(result.get("families", [])), ensure_ascii=False))
            failure_html = (
                f"<p class=\"failure\">{html.escape(result['failure'])}</p>"
                if result.get("failure")
                else ""
            )
            fallback_html = (
                "<p class=\"failure\">曲线交点为空，本项已回退为直线交点，仅用于定位退化。</p>"
                if result.get("fallback_to_straight")
                else ""
            )
            if result.get("failure"):
                media_html = f"""
                  <div class="failure-panel">
                    <strong>本消融项没有识别效果图</strong>
                    <span>{html.escape(result['failure'])}</span>
                    <span>这表示删除该工序后未能产出可求交的两组线族，不是新的钢筋识别结果。</span>
                  </div>
                """
            else:
                media_html = f"""
                  <div class="images">
                    <figure>
                      <img loading="lazy" src="{html.escape(result['original_image_path'])}" alt="{html.escape(result['label'])} original overlay">
                      <figcaption>原图识别效果</figcaption>
                    </figure>
                    <figure>
                      <img loading="lazy" src="{html.escape(result['rectified_image_path'])}" alt="{html.escape(result['label'])} rectified overlay">
                      <figcaption>透视展开识别效果</figcaption>
                    </figure>
                  </div>
                """
            cards.append(
                f"""
                <article class="result-card {html.escape(tone)}">
                  <div class="card-head">
                    <div>
                      <p class="stage">{html.escape(result['stage'])}</p>
                      <h3>{html.escape(result['stage_variant_label'])}</h3>
                    </div>
                    <span class="pill {html.escape(tone)}">{html.escape(result['verdict'])}</span>
                  </div>
                  <p>{html.escape(result['description'])}</p>
                  <p class="conclusion">{html.escape(result['conclusion'])}</p>
                  {failure_html}
                  {fallback_html}
                  <div class="metrics">
                    <span>mean {mean_ms} ms</span>
                    <span>median {median_ms} ms</span>
                    <span>{result.get('point_count', 0)} points</span>
                    <span>scheme drift max {scheme_max_drift} px</span>
                    <span>reference drift max {global_max_drift} px</span>
                  </div>
                  {media_html}
                  <details>
                    <summary>线族明细</summary>
                    <pre>{family_text}</pre>
                  </details>
                </article>
                """
            )
        scheme_sections.append(
            f"""
            <section class="scheme-section" id="{html.escape(scheme['id'])}">
              <div class="scheme-title">
                <div>
                  <p>{html.escape(scheme['short_label'])}</p>
                  <h2>{html.escape(scheme['label'])}</h2>
                </div>
                <span>{html.escape(scheme['kind'])}</span>
              </div>
              <p class="scheme-desc">{html.escape(scheme['description'])}</p>
              <div class="table-panel compact">
                <table>
                  <thead>
                    <tr>
                      <th>工序</th>
                      <th>消融项</th>
                      <th>均值 ms</th>
                      <th>中位 ms</th>
                      <th>最大 ms</th>
                      <th>点数</th>
                      <th>相对本方案最大漂移 px</th>
                      <th>本方案点数变化</th>
                      <th>相对参考最大漂移 px</th>
                      <th>判断</th>
                    </tr>
                  </thead>
                  <tbody>{''.join(rows)}</tbody>
                </table>
              </div>
              <div class="result-grid">{''.join(cards)}</div>
            </section>
            """
        )

    scheme_nav_buttons = [
        '<button type="button" class="active" data-scheme-filter="all" onclick="showScheme(\'all\', this)">全部方案</button>'
    ]
    for scheme in SCHEMES:
        scheme_nav_buttons.append(
            f"""
            <button type="button" data-scheme-filter="{html.escape(scheme['id'])}" onclick="showScheme('{html.escape(scheme['id'])}', this)">
              <span>{html.escape(scheme['short_label'])}</span>
              <strong>{html.escape(scheme['label'].split(' ', 1)[-1])}</strong>
            </button>
            """
        )

    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PR-FPRG 方案1工序消融报告</title>
  <style>
    :root {{
      --bg: #eef1f4;
      --panel: #ffffff;
      --ink: #1c252e;
      --muted: #5b6876;
      --line: #d7dee7;
      --green: #15803d;
      --amber: #b45309;
      --red: #b91c1c;
      --cyan: #0e7490;
      --charcoal: #18212b;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--ink);
      font-family: "Noto Sans CJK SC", "Source Han Sans SC", "Microsoft YaHei", sans-serif;
    }}
    header {{
      padding: 24px 28px 18px;
      background: var(--charcoal);
      color: #f8fafc;
      border-bottom: 4px solid #26a269;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 27px;
      letter-spacing: 0;
    }}
    header p {{
      margin: 0;
      color: #cbd5e1;
      line-height: 1.55;
    }}
    main {{
      padding: 18px 28px 40px;
      display: grid;
      gap: 18px;
    }}
    .summary-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(190px, 1fr));
      gap: 12px;
    }}
    .stat, .table-panel, .scheme-section, .source-strip, .result-card {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      box-shadow: 0 1px 2px rgba(15, 23, 42, 0.06);
    }}
    .stat {{
      padding: 14px 16px;
    }}
    .stat span {{
      display: block;
      color: var(--muted);
      font-size: 13px;
    }}
    .stat strong {{
      display: block;
      margin-top: 5px;
      font-size: 22px;
      letter-spacing: 0;
    }}
    .source-strip, .scheme-section {{
      padding: 16px;
    }}
    .scheme-nav {{
      position: sticky;
      top: 0;
      z-index: 5;
      display: grid;
      gap: 10px;
      padding: 12px;
      background: rgba(238, 241, 244, 0.96);
      border: 1px solid var(--line);
      border-radius: 8px;
      backdrop-filter: blur(8px);
    }}
    .scheme-nav p {{
      margin: 0;
      color: var(--muted);
      font-size: 13px;
      font-weight: 700;
    }}
    .scheme-filter-row {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      gap: 8px;
    }}
    .scheme-filter-row button {{
      min-height: 48px;
      border: 1px solid #cbd5e1;
      border-radius: 7px;
      background: #ffffff;
      color: #243241;
      text-align: left;
      cursor: pointer;
      padding: 8px 10px;
      font: inherit;
    }}
    .scheme-filter-row button.active {{
      border-color: #15803d;
      box-shadow: inset 0 0 0 1px #15803d;
      background: #f0fdf4;
    }}
    .scheme-filter-row button span {{
      display: block;
      color: var(--muted);
      font-size: 12px;
      font-weight: 700;
    }}
    .scheme-filter-row button strong {{
      display: block;
      margin-top: 3px;
      font-size: 13px;
      letter-spacing: 0;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }}
    .source-strip h2 {{
      margin: 0 0 10px;
      font-size: 18px;
    }}
    .table-panel {{
      padding: 14px;
      overflow-x: auto;
    }}
    .table-panel.compact {{
      box-shadow: none;
      margin-top: 12px;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      min-width: 1080px;
      font-size: 13px;
    }}
    th, td {{
      padding: 10px 9px;
      border-bottom: 1px solid #e5ebf2;
      text-align: left;
      vertical-align: top;
    }}
    th {{
      color: #334155;
      background: #f8fafc;
      font-weight: 700;
    }}
    .scheme-title, .card-head {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: flex-start;
    }}
    .scheme-title p, .stage {{
      margin: 0 0 4px;
      color: var(--muted);
      font-size: 13px;
      font-weight: 700;
    }}
    .scheme-title h2, .result-card h3 {{
      margin: 0;
      letter-spacing: 0;
    }}
    .scheme-title h2 {{
      font-size: 20px;
    }}
    .result-card h3 {{
      font-size: 17px;
    }}
    .scheme-title > span {{
      border: 1px solid #cbd5e1;
      border-radius: 6px;
      padding: 5px 8px;
      color: #334155;
      background: #f8fafc;
      font-size: 12px;
      font-weight: 700;
      white-space: nowrap;
    }}
    .scheme-desc, .result-card p, .source-strip p {{
      margin: 9px 0 0;
      color: var(--muted);
      line-height: 1.55;
      font-size: 14px;
    }}
    .pill {{
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 4px 9px;
      font-size: 12px;
      font-weight: 700;
      white-space: nowrap;
    }}
    .pill.baseline {{ color: #075985; background: #e0f2fe; }}
    .pill.ok {{ color: #166534; background: #dcfce7; }}
    .pill.warn {{ color: #92400e; background: #fef3c7; }}
    .pill.danger {{ color: #991b1b; background: #fee2e2; }}
    .pill.curve {{ color: #155e75; background: #cffafe; }}
    .result-grid {{
      display: grid;
      grid-template-columns: 1fr;
      gap: 14px;
      margin-top: 14px;
    }}
    .result-card {{
      padding: 15px;
      box-shadow: none;
    }}
    .conclusion {{
      color: var(--ink) !important;
      font-weight: 700;
    }}
    .failure {{
      color: var(--red) !important;
    }}
    .failure-panel {{
      margin-top: 12px;
      display: grid;
      gap: 6px;
      border: 1px solid #fecaca;
      border-radius: 8px;
      background: #fff7f7;
      color: #7f1d1d;
      padding: 14px;
    }}
    .failure-panel strong {{
      font-size: 15px;
      letter-spacing: 0;
    }}
    .failure-panel span {{
      font-size: 13px;
      line-height: 1.45;
    }}
    .metrics {{
      margin: 12px 0;
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
    }}
    .metrics span {{
      border: 1px solid #d9e2ec;
      border-radius: 6px;
      padding: 5px 8px;
      background: #f8fafc;
      color: #334155;
      font-size: 12px;
      font-weight: 700;
    }}
    .images {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 12px;
    }}
    figure {{
      margin: 0;
      border: 1px solid #d9e2ec;
      border-radius: 8px;
      overflow: hidden;
      background: #0f172a;
    }}
    img {{
      display: block;
      width: 100%;
      height: auto;
    }}
    figcaption {{
      padding: 8px 10px;
      background: #f8fafc;
      color: #334155;
      border-top: 1px solid #d9e2ec;
      font-size: 13px;
    }}
    details {{
      margin-top: 12px;
    }}
    summary {{
      cursor: pointer;
      color: #334155;
      font-size: 13px;
      font-weight: 700;
    }}
    pre {{
      overflow-x: auto;
      padding: 10px;
      border: 1px solid #d9e2ec;
      border-radius: 6px;
      background: #f8fafc;
      font-size: 12px;
      line-height: 1.45;
    }}
    @media (max-width: 720px) {{
      header, main {{ padding-left: 14px; padding-right: 14px; }}
      .images {{ grid-template-columns: 1fr; }}
      table {{ min-width: 940px; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>PR-FPRG 方案1工序消融报告</h1>
      <p>同一帧输入、锁定同一组合响应图。当前只保留方案1直线族识别：钢筋线允许穿过梁筋，梁筋±13cm范围只在最终绑扎点级排除。</p>
  </header>
  <main>
    <section class="summary-grid">
      <div class="stat"><span>生成时间</span><strong>{html.escape(metadata['generated_at'])}</strong></div>
      <div class="stat"><span>帧源</span><strong>{html.escape(str(metadata['frame_source']))}</strong></div>
      <div class="stat"><span>重复次数</span><strong>{metadata['repeat']}</strong></div>
      <div class="stat"><span>响应图</span><strong>{html.escape(str(metadata['locked_response_name']))}</strong></div>
      <div class="stat"><span>方案数</span><strong>{len(SCHEMES)}</strong></div>
      <div class="stat"><span>消融项/方案</span><strong>{len(STAGE_VARIANTS)}</strong></div>
    </section>
    <section class="scheme-nav" aria-label="方案筛选">
      <p>方案入口：点击后只显示对应方案的 {len(STAGE_VARIANTS)} 个消融项；选“全部方案”可恢复完整列表。</p>
      <div class="scheme-filter-row">{''.join(scheme_nav_buttons)}</div>
    </section>
    <section class="source-strip">
      <h2>输入、响应与峰值图</h2>
      <p>用于所有方案和消融项的同一帧输入；峰值图固定显示方案1全流程的候选线与最终保留/剔除情况。</p>
      <div class="images">
        <figure><img src="images/00_ir_input.png" alt="ir input"><figcaption>红外输入</figcaption></figure>
        <figure><img src="images/00_rectified_ir.png" alt="rectified ir"><figcaption>透视展开红外</figcaption></figure>
        <figure><img src="images/00_response.png" alt="response"><figcaption>锁定响应图</figcaption></figure>
        <figure><img src="images/00_peak_supported_lines.png" alt="peak supported lines"><figcaption>峰值图：peak-supported 候选线</figcaption></figure>
        <figure><img src="images/00_peak_spacing_pruned_lines.png" alt="peak spacing pruned lines"><figcaption>峰值图：绿色保留 / 红色剔除</figcaption></figure>
      </div>
    </section>
    <section class="table-panel">
      <table>
        <thead>
          <tr>
            <th>方案</th>
            <th>全流程基准</th>
            <th>均值 ms</th>
            <th>中位 ms</th>
            <th>点数</th>
            <th>相对参考平均漂移 px</th>
            <th>相对参考最大漂移 px</th>
            <th>点数变化</th>
          </tr>
        </thead>
        <tbody>{''.join(scheme_full_rows)}</tbody>
      </table>
    </section>
    {''.join(scheme_sections)}
  </main>
  <script>
    function showScheme(schemeId, button) {{
      document.querySelectorAll('.scheme-section').forEach(function(section) {{
        section.hidden = schemeId !== 'all' && section.id !== schemeId;
      }});
      document.querySelectorAll('[data-scheme-filter]').forEach(function(item) {{
        item.classList.remove('active');
      }});
      if (button) {{
        button.classList.add('active');
      }}
      if (schemeId !== 'all') {{
        var target = document.getElementById(schemeId);
        if (target) {{
          target.scrollIntoView({{ behavior: 'smooth', block: 'start' }});
        }}
      }}
    }}
  </script>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")
    return output_dir / "index.html"


def build_results(frame, prepared, locked_response_name, repeat_count):
    results = []
    for scheme in SCHEMES:
        for stage_variant in STAGE_VARIANTS:
            scheme_id = scheme["id"]
            stage_variant_id = stage_variant["id"]
            failure = None
            timings = []
            payload = None
            try:
                timings, payload = time_scheme_stage_variant(
                    frame,
                    prepared,
                    scheme,
                    stage_variant,
                    locked_response_name,
                    repeat_count,
                )
            except Exception as exc:
                failure = str(exc)

            common = {
                "scheme_id": scheme_id,
                "scheme_label": scheme["label"],
                "scheme_short_label": scheme["short_label"],
                "stage_variant_id": stage_variant_id,
                "stage_variant_label": stage_variant["label"],
                "stage": stage_variant["stage"],
                "label": f"{scheme['short_label']} / {stage_variant['label']}",
                "description": stage_variant["description"],
                "conclusion": stage_variant["conclusion"],
                "tone": stage_variant.get("tone", "warn"),
                "failure": failure,
            }

            if failure is not None or payload is None:
                result = {
                    **common,
                    "mean_ms": None,
                    "median_ms": None,
                    "min_ms": None,
                    "max_ms": None,
                    "point_count": 0,
                    "points": [],
                    "rectified_points": [],
                    "line_families": [],
                    "families": [],
                    "response": None,
                    "geometry_type": "straight",
                    "color": scheme["color"],
                    "fallback_to_straight": False,
                }
            else:
                scheme_result = payload["scheme_result"]
                result = {
                    **common,
                    "mean_ms": mean_or_none(timings),
                    "median_ms": median_or_none(timings),
                    "min_ms": float(min(timings)),
                    "max_ms": float(max(timings)),
                    "point_count": len(scheme_result["points"]),
                    "points": scheme_result["points"],
                    "rectified_points": scheme_result["rectified_points"],
                    "line_families": scheme_result["line_families"],
                    "families": scheme_result["families"],
                    "response": scheme_result["response"],
                    "geometry_type": scheme_result["geometry_type"],
                    "color": scheme_result["color"],
                    "fallback_to_straight": scheme_result.get("fallback_to_straight", False),
                }
            results.append(result)

    scheme_full_points = {}
    for result in results:
        if result["stage_variant_id"] == "full":
            scheme_full_points[result["scheme_id"]] = result.get("points", [])
    global_reference = scheme_full_points.get(REALTIME_TARGET_SCHEME_ID)
    if global_reference is None and SCHEMES:
        global_reference = scheme_full_points.get(SCHEMES[0]["id"], [])
    if global_reference is None:
        global_reference = []

    for result in results:
        scheme_reference = scheme_full_points.get(result["scheme_id"], [])
        result["drift_vs_scheme_full"] = point_drift_stats(scheme_reference, result.get("points", []))
        result["drift_vs_reference_full"] = point_drift_stats(global_reference, result.get("points", []))
        result["verdict"] = verdict_for_result(result)
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument(
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"pr_fprg_scheme_ablation_report_{time.strftime('%Y%m%d_%H%M%S')}"),
    )
    args = parser.parse_args()

    rospy.init_node("pr_fprg_scheme_ablation_report", anonymous=True, disable_signals=True)
    try:
        frame = capture_synced_frame(args.timeout)
    except RuntimeError as exc:
        rospy.logwarn("PointAI_log: raw_world同步帧不可用，改用 depth+IR 兜底做方案1消融报告：%s", exc)
        frame = capture_depth_ir_frame(args.timeout)

    prepared = prepare_frame_inputs(frame)
    repeat_count = max(1, int(args.repeat))

    baseline_timings, baseline_payload = time_scheme_stage_variant(
        frame,
        prepared,
        SCHEMES[0],
        STAGE_VARIANTS[0],
        None,
        1,
    )
    _ = baseline_timings
    locked_response_name = baseline_payload["family_result"]["response_name"]

    results = build_results(frame, prepared, locked_response_name, repeat_count)
    metadata = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source", "raw_world"),
        "repeat": repeat_count,
        "locked_response_name": locked_response_name,
    }
    report_path = render_report(args.output_dir, frame, prepared, results, metadata)
    print(str(report_path))


if __name__ == "__main__":
    main()
