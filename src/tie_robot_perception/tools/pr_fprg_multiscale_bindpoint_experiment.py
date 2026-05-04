#!/usr/bin/env python3

"""Run a PR-FPRG-MS multiscale bind-point experiment report.

This tool is report-only. It keeps the current PR-FPRG runtime untouched, then
combines the best lessons from the recent experiments:

PR-FPRG-MS =
  combined response
  + 2026-04-22 frequency/phase full topology
  + rebar surface segmentation and completion
  + instance-graph junctions for near-scale fallback
  + beam_candidate +/-13cm point exclusion
  + 3/4/5/6 local curve convergence.
"""

from __future__ import annotations

import argparse
import html
import json
import shutil
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
from pr_fprg_topology_recovery_experiment import (  # noqa: E402
    build_curve3456_on_recovered_mainline,
    build_recovered_frequency_phase_grid,
    render_curve_variant_original,
    render_curve_variant_rectified,
    render_original_points,
    render_profile_fft_plot,
    render_rectified_lattice,
    summarize_fft_estimate,
)
from rebar_instance_graph_probe import (  # noqa: E402
    capture_frame,
    derive_modalities,
    json_safe,
    render_binary,
    render_heatmap,
    render_skeleton_overlay,
)
from rebar_surface_bindpoint_comparison import (  # noqa: E402
    build_completed_surface_intersections,
    build_montage,
    build_surface_segmentation,
    cluster_instance_graph_junctions,
    points_from_rectified_points,
    project_rectified_points_to_image,
    render_ir_high_gamma,
    render_original_variant,
    render_rectified_variant,
    render_surface_completion_overlay,
    render_surface_segmentation_overlay,
    response_value_at,
)
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_axis_profile,
    estimate_workspace_s2_fft_period_and_phase,
)


REPORT_RELATIVE_URL = "reports/pr_fprg_multiscale_bindpoint_experiment"


def compute_multiscale_scale_metrics(result, recovered_line_families, vertical_fft_estimate, horizontal_fft_estimate):
    """Classify the current view scale from visible topology and FFT stability."""
    current_line_counts = [len(family.get("line_rhos", [])) for family in result.get("line_families", [])[:2]]
    recovered_line_counts = [len(family.get("line_rhos", [])) for family in recovered_line_families[:2]]
    while len(current_line_counts) < 2:
        current_line_counts.append(0)
    while len(recovered_line_counts) < 2:
        recovered_line_counts.append(0)

    rectified_geometry = result.get("rectified_geometry") or {}
    resolution_mm_per_px = float(rectified_geometry.get("resolution_mm_per_px", 5.0) or 5.0)
    rectified_width = int(rectified_geometry.get("rectified_width", 0) or 0)
    rectified_height = int(rectified_geometry.get("rectified_height", 0) or 0)
    visible_cells = max(0, recovered_line_counts[0] - 1) * max(0, recovered_line_counts[1] - 1)

    fft_scores = [
        float((vertical_fft_estimate or {}).get("score", 0.0) or 0.0),
        float((horizontal_fft_estimate or {}).get("score", 0.0) or 0.0),
    ]
    min_recovered_lines = min(recovered_line_counts) if recovered_line_counts else 0
    if min_recovered_lines >= 6 and visible_cells >= 42:
        scale_mode = "frequency_primary"
        scale_reason = "可见周期充足，优先使用 2026-04-22 频相主拓扑恢复，再用实例图谱校验。"
        strategy_weights = {
            "frequency_phase_after_beam": 1.00,
            "curve04_frequency_dp_13cm": 0.90,
            "completed_surface_intersections_13cm": 0.70,
            "instance_graph_junctions_13cm": 0.55,
        }
        anchor_sources = ["frequency_phase_after_beam"]
        merge_radius_px = 7.0
    elif min_recovered_lines >= 4 and visible_cells >= 16:
        scale_mode = "hybrid"
        scale_reason = "可见周期中等，频相和补全钢筋面并重，曲线算法只做局部收束。"
        strategy_weights = {
            "frequency_phase_after_beam": 0.85,
            "curve04_frequency_dp_13cm": 0.90,
            "completed_surface_intersections_13cm": 0.90,
            "instance_graph_junctions_13cm": 0.65,
        }
        anchor_sources = ["frequency_phase_after_beam", "completed_surface_intersections_13cm"]
        merge_radius_px = 9.0
    else:
        scale_mode = "instance_graph_primary"
        scale_reason = "近距离可见周期不足，FFT 只做弱先验，主要依赖钢筋面补全和骨架交叉。"
        strategy_weights = {
            "completed_surface_intersections_13cm": 1.00,
            "instance_graph_junctions_13cm": 0.90,
            "curve04_frequency_dp_13cm": 0.75,
            "frequency_phase_after_beam": 0.55,
        }
        anchor_sources = ["completed_surface_intersections_13cm", "instance_graph_junctions_13cm"]
        merge_radius_px = 11.0

    return {
        "scale_mode": scale_mode,
        "scale_reason": scale_reason,
        "current_line_counts": current_line_counts,
        "recovered_line_counts": recovered_line_counts,
        "visible_cells": int(visible_cells),
        "rectified_width": rectified_width,
        "rectified_height": rectified_height,
        "resolution_mm_per_px": resolution_mm_per_px,
        "fft_scores": fft_scores,
        "strategy_weights": strategy_weights,
        "anchor_sources": anchor_sources,
        "merge_radius_px": merge_radius_px,
    }


def clone_points(points, source_id):
    cloned = []
    for point in points or []:
        copied = dict(point)
        copied["idx"] = len(cloned) + 1
        copied["source"] = source_id
        cloned.append(copied)
    return cloned


def rectified_distance(left, right):
    left = np.asarray(left, dtype=np.float32)
    right = np.asarray(right, dtype=np.float32)
    return float(np.linalg.norm(left - right))


def merge_multiscale_bindpoint_candidates(result, candidate_variants, scale_metrics, response_map):
    """Merge frequency, surface, instance-graph and curve candidates by rectified distance.

    The merge is anchor-based. On a frequency-primary frame, frequency/phase
    points define the graph topology and other sources can only validate nearby
    anchors. On near-scale frames, the anchor set switches to completed surface
    and instance graph points.
    """
    strategy_weights = scale_metrics["strategy_weights"]
    anchor_sources = set(scale_metrics.get("anchor_sources") or [])
    merge_radius_px = float(scale_metrics["merge_radius_px"])
    ordered_candidates = []
    for variant in candidate_variants:
        source_id = variant["id"]
        source_weight = float(strategy_weights.get(source_id, 0.5))
        for point in variant.get("points", []):
            ordered_candidates.append((source_weight, source_id, point))
    ordered_candidates.sort(key=lambda item: (-item[0], -float(item[2].get("score", 0.0))))

    def add_new_cluster(source_weight, source_id, point):
        rectified = [float(point["rectified"][0]), float(point["rectified"][1])]
        clusters.append(
            {
                "rectified": rectified,
                "pix": [float(point["pix"][0]), float(point["pix"][1])],
                "score": float(point.get("score", 0.0)),
                "sources": [source_id],
                "source_weight_sum": source_weight,
                "source": "multiscale_fused",
            }
        )

    clusters = []
    for source_weight, source_id, point in ordered_candidates:
        if source_id not in anchor_sources:
            continue
        add_new_cluster(source_weight, source_id, point)

    if not clusters:
        for source_weight, source_id, point in ordered_candidates:
            add_new_cluster(source_weight, source_id, point)

    for source_weight, source_id, point in ordered_candidates:
        if source_id in anchor_sources:
            continue
        rectified = [float(point["rectified"][0]), float(point["rectified"][1])]
        matched = None
        for cluster in clusters:
            if rectified_distance(rectified, cluster["rectified"]) <= merge_radius_px:
                matched = cluster
                break
        if matched is None:
            continue
        matched["sources"].append(source_id)
        matched["source_weight_sum"] += source_weight
        if float(point.get("score", 0.0)) > float(matched.get("score", 0.0)):
            matched["score"] = float(point.get("score", 0.0))

    fused = []
    for cluster in clusters:
        unique_sources = sorted(set(cluster["sources"]))
        confidence = min(
            1.0,
            0.30
            + (0.18 * len(unique_sources))
            + (0.22 * float(cluster.get("score", 0.0)))
            + (0.08 * min(3.0, float(cluster.get("source_weight_sum", 0.0)))),
        )
        fused.append(
            {
                "idx": 0,
                "rectified": cluster["rectified"],
                "pix": cluster["pix"],
                "score": response_value_at(response_map, cluster["rectified"]),
                "confidence": float(confidence),
                "sources": unique_sources,
                "source": "multiscale_fused",
            }
        )

    fused.sort(key=lambda point: (float(point["rectified"][1]), float(point["rectified"][0])))
    for index, point in enumerate(fused, start=1):
        point["idx"] = index
    return fused


def render_scale_decision_flow(scale_metrics):
    canvas = np.full((520, 1100, 3), (247, 243, 234), dtype=np.uint8)
    title_color = (28, 38, 32)
    text_color = (62, 72, 66)
    edge_color = (160, 142, 113)
    active_color = (36, 120, 95)
    muted_color = (210, 202, 185)
    cv2.putText(canvas, "PR-FPRG-MS multiscale decision", (34, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.9, title_color, 2, cv2.LINE_AA)
    cv2.putText(
        canvas,
        f"scale_mode={scale_metrics['scale_mode']}  recovered={scale_metrics['recovered_line_counts']}  cells={scale_metrics['visible_cells']}",
        (34, 76),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        text_color,
        1,
        cv2.LINE_AA,
    )
    nodes = [
        ("combined response", (58, 140)),
        ("scale metrics", (305, 140)),
        ("frequency topology", (552, 96)),
        ("surface completion", (552, 184)),
        ("instance graph", (552, 272)),
        ("beam +/-13cm", (780, 184)),
        ("curve 3/4/5/6", (780, 272)),
        ("fused bind points", (780, 382)),
    ]
    active = {
        "frequency topology",
        "surface completion",
        "instance graph",
        "beam +/-13cm",
        "curve 3/4/5/6",
        "fused bind points",
    }
    box_size = (210, 52)
    centers = {}
    for label, (x_value, y_value) in nodes:
        color = active_color if label in active else muted_color
        cv2.rectangle(canvas, (x_value, y_value), (x_value + box_size[0], y_value + box_size[1]), color, -1)
        cv2.rectangle(canvas, (x_value, y_value), (x_value + box_size[0], y_value + box_size[1]), edge_color, 1)
        cv2.putText(canvas, label, (x_value + 12, y_value + 33), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255) if label in active else text_color, 1, cv2.LINE_AA)
        centers[label] = (x_value + box_size[0], y_value + box_size[1] // 2)

    def arrow(left_label, right_label):
        start = centers[left_label]
        right_x, right_y = next(position for label, position in nodes if label == right_label)
        end = (right_x, right_y + box_size[1] // 2)
        cv2.arrowedLine(canvas, start, end, edge_color, 2, cv2.LINE_AA, tipLength=0.04)

    arrow("combined response", "scale metrics")
    arrow("scale metrics", "frequency topology")
    arrow("scale metrics", "surface completion")
    arrow("scale metrics", "instance graph")
    arrow("frequency topology", "beam +/-13cm")
    arrow("surface completion", "beam +/-13cm")
    arrow("instance graph", "curve 3/4/5/6")
    arrow("beam +/-13cm", "fused bind points")
    arrow("curve 3/4/5/6", "fused bind points")

    y0 = 468
    cv2.putText(canvas, f"reason: {scale_metrics['scale_reason'][:100]}", (34, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.48, text_color, 1, cv2.LINE_AA)
    return canvas


def render_fused_original(frame, result, recovered_line_families, fused_points, title, ir_display_gamma):
    image = render_original_points(
        frame,
        result,
        recovered_line_families,
        fused_points,
        title,
        ir_display_gamma=ir_display_gamma,
    )
    for point in fused_points:
        x_value, y_value = point["pix"]
        center = (int(round(x_value)), int(round(y_value)))
        confidence = float(point.get("confidence", 0.0))
        color = (0, 255, 255) if confidence >= 0.72 else (0, 190, 255)
        cv2.circle(image, center, 9, color, 1, cv2.LINE_AA)
    return image


def build_multiscale_bindpoint_experiment(frame, threshold_percentile=83.0, ir_display_gamma=2.05):
    result = run_peak_supported_pr_fprg(frame, response_name_filter="combined_depth_ir_darkline")
    modalities = derive_modalities(result, threshold_percentile=threshold_percentile)
    surface_segmentation = build_surface_segmentation(result, modalities)
    recovered_line_families, recovered_rectified_points = build_recovered_frequency_phase_grid(result)

    vertical_profile = build_workspace_s2_axis_profile(result["response"], result["rectified_valid"].astype(np.uint8), axis=0)
    horizontal_profile = build_workspace_s2_axis_profile(result["response"], result["rectified_valid"].astype(np.uint8), axis=1)
    vertical_fft_estimate = estimate_workspace_s2_fft_period_and_phase(vertical_profile, min_period=10, max_period=30)
    horizontal_fft_estimate = estimate_workspace_s2_fft_period_and_phase(horizontal_profile, min_period=10, max_period=30)
    scale_metrics = compute_multiscale_scale_metrics(
        result,
        recovered_line_families,
        vertical_fft_estimate,
        horizontal_fft_estimate,
    )

    beam_13cm_mask = modalities["beam_candidate_mask_13cm"]
    frequency_points = points_from_rectified_points(
        result,
        recovered_rectified_points,
        result["response"],
        "frequency_phase_after_beam",
        exclusion_mask=beam_13cm_mask,
    )
    completed_surface_points = clone_points(
        build_completed_surface_intersections(
            result,
            surface_segmentation,
            modalities,
            exclusion_mask=beam_13cm_mask,
        ),
        "completed_surface_intersections_13cm",
    )
    instance_graph_points = clone_points(
        cluster_instance_graph_junctions(result, modalities, exclusion_mask=beam_13cm_mask, min_area_px=2),
        "instance_graph_junctions_13cm",
    )
    curve_variants = build_curve3456_on_recovered_mainline(
        result,
        modalities,
        surface_segmentation,
        recovered_line_families,
    )
    curve04 = next((variant for variant in curve_variants if variant["id"] == "04_recovered_dp_curve"), curve_variants[0])
    curve04_points = clone_points(curve04["points"], "curve04_frequency_dp_13cm")

    candidate_variants = [
        {
            "id": "frequency_phase_after_beam",
            "title": "频相主拓扑 + 梁筋 ±13cm",
            "points": frequency_points,
            "note": "22 日频相恢复负责远/中尺度高召回。",
        },
        {
            "id": "completed_surface_intersections_13cm",
            "title": "补全钢筋面交点 + 梁筋 ±13cm",
            "points": completed_surface_points,
            "note": "近尺度补偿：分割缺线后用 line support 补全再求交点。",
        },
        {
            "id": "instance_graph_junctions_13cm",
            "title": "实例骨架交叉点 + 梁筋 ±13cm",
            "points": instance_graph_points,
            "note": "不依赖理论周期，作为近距离和遮挡下的召回补充。",
        },
        {
            "id": "curve04_frequency_dp_13cm",
            "title": "方案4 DP 曲线收束 + 梁筋 ±13cm",
            "points": curve04_points,
            "note": "以频相主线为基底，只在局部把线贴回真实钢筋。",
        },
    ]
    fused_points = merge_multiscale_bindpoint_candidates(
        result,
        candidate_variants,
        scale_metrics,
        result["response"],
    )
    candidate_variants.append(
        {
            "id": "multiscale_fused",
            "title": "PR-FPRG-MS 多尺度融合",
            "points": fused_points,
            "note": "按尺度权重合并频相、补全面、实例骨架和曲线收束候选。",
        }
    )
    for variant in candidate_variants:
        points = variant.get("points", [])
        variant["point_count"] = len(points)
        variant["mean_score"] = float(np.mean([point.get("score", 0.0) for point in points])) if points else 0.0

    return {
        "result": result,
        "modalities": modalities,
        "surface_segmentation": surface_segmentation,
        "recovered_line_families": recovered_line_families,
        "recovered_rectified_points": recovered_rectified_points,
        "vertical_profile": vertical_profile,
        "horizontal_profile": horizontal_profile,
        "vertical_fft_estimate": vertical_fft_estimate,
        "horizontal_fft_estimate": horizontal_fft_estimate,
        "scale_metrics": scale_metrics,
        "candidate_variants": candidate_variants,
        "curve_variants": curve_variants,
        "fused_points": fused_points,
        "ir_display_gamma": float(ir_display_gamma),
    }


def write_variant_rows(candidate_variants):
    rows = []
    for variant in candidate_variants:
        points = variant.get("points", [])
        mean_score = float(np.mean([point.get("score", 0.0) for point in points])) if points else 0.0
        rows.append(
            f"""
            <tr>
              <td><code>{html.escape(variant['id'])}</code></td>
              <td>{html.escape(variant['title'])}</td>
              <td>{len(points)}</td>
              <td>{mean_score:.3f}</td>
              <td>{html.escape(variant.get('note', ''))}</td>
            </tr>
            """
        )
    return "\n".join(rows)


def write_report(output_dir, frame, experiment, timings_ms):
    output_dir = Path(output_dir)
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)

    result = experiment["result"]
    modalities = experiment["modalities"]
    surface_segmentation = experiment["surface_segmentation"]
    recovered_line_families = experiment["recovered_line_families"]
    scale_metrics = experiment["scale_metrics"]
    candidate_variants = experiment["candidate_variants"]
    curve_variants = experiment["curve_variants"]
    fused_points = experiment["fused_points"]
    ir_display_gamma = experiment["ir_display_gamma"]

    frequency_variant = next(variant for variant in candidate_variants if variant["id"] == "frequency_phase_after_beam")
    completed_variant = next(variant for variant in candidate_variants if variant["id"] == "completed_surface_intersections_13cm")
    instance_variant = next(variant for variant in candidate_variants if variant["id"] == "instance_graph_junctions_13cm")
    curve04_variant = next(variant for variant in candidate_variants if variant["id"] == "curve04_frequency_dp_13cm")

    images = {
        "01_input_high_gamma_workspace.png": render_ir_high_gamma(frame, result, ir_display_gamma=ir_display_gamma),
        "02_rectified_ir.png": to_bgr(to_u8(result["rectified_ir"])),
        "03_combined_response.png": render_heatmap(result["response"]),
        "04_scale_decision_flow.png": render_scale_decision_flow(scale_metrics),
        "05_vertical_profile_fft.png": render_profile_fft_plot(
            experiment["vertical_profile"],
            experiment["vertical_fft_estimate"],
            recovered_line_families[0].get("line_rhos", []),
            "vertical profile FFT",
        ),
        "06_horizontal_profile_fft.png": render_profile_fft_plot(
            experiment["horizontal_profile"],
            experiment["horizontal_fft_estimate"],
            recovered_line_families[1].get("line_rhos", []),
            "horizontal profile FFT",
        ),
        "07_frequency_phase_lattice_rectified.png": render_rectified_lattice(
            result,
            recovered_line_families,
            "frequency-phase lattice",
        ),
        "08_surface_segmentation_overlay.png": render_surface_segmentation_overlay(modalities, surface_segmentation),
        "09_surface_completion_overlay.png": render_surface_completion_overlay(modalities, surface_segmentation),
        "10_beam_candidate_13cm_mask.png": render_binary(modalities["beam_candidate_mask_13cm"], "beam_candidate_mask_13cm"),
        "11_frequency_phase_after_beam_original.png": render_original_points(
            frame,
            result,
            recovered_line_families,
            frequency_variant["points"],
            f"frequency_phase_after_beam points={len(frequency_variant['points'])}",
            ir_display_gamma=ir_display_gamma,
        ),
        "12_completed_surface_intersections_original.png": render_original_variant(
            frame,
            result,
            completed_variant,
            ir_display_gamma=ir_display_gamma,
        ),
        "13_instance_graph_junctions_original.png": render_original_variant(
            frame,
            result,
            instance_variant,
            ir_display_gamma=ir_display_gamma,
        ),
        "14_curve04_frequency_dp_original.png": render_original_variant(
            frame,
            result,
            curve04_variant,
            ir_display_gamma=ir_display_gamma,
        ),
        "15_multiscale_fused_original.png": render_fused_original(
            frame,
            result,
            recovered_line_families,
            fused_points,
            f"PR-FPRG-MS fused points={len(fused_points)}",
            ir_display_gamma,
        ),
        "16_current_runtime_reference.png": render_result(frame, result),
    }

    for index, variant in enumerate(curve_variants, start=17):
        images[f"{index:02d}_{variant['id']}_original.png"] = render_curve_variant_original(
            frame,
            result,
            variant,
            ir_display_gamma=ir_display_gamma,
        )
        images[f"{index:02d}r_{variant['id']}_rectified.png"] = render_curve_variant_rectified(
            variant["response_map"],
            variant,
        )

    image_paths = {}
    for filename, image in images.items():
        path = image_dir / filename
        cv2.imwrite(str(path), image)
        image_paths[filename] = str(path)

    matrix_path = image_dir / "21_candidate_matrix.png"
    build_montage(
        [
            image_dir / "11_frequency_phase_after_beam_original.png",
            image_dir / "12_completed_surface_intersections_original.png",
            image_dir / "13_instance_graph_junctions_original.png",
            image_dir / "14_curve04_frequency_dp_original.png",
            image_dir / "15_multiscale_fused_original.png",
        ],
        matrix_path,
        [
            f"frequency points={len(frequency_variant['points'])}",
            f"surface points={len(completed_variant['points'])}",
            f"instance points={len(instance_variant['points'])}",
            f"curve04 points={len(curve04_variant['points'])}",
            f"fused points={len(fused_points)}",
        ],
    )
    image_paths["21_candidate_matrix.png"] = str(matrix_path)

    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source", "raw_world"),
        "report_url": f"/{REPORT_RELATIVE_URL}/index.html",
        "response_name": result.get("response_name"),
        "ir_display_gamma": ir_display_gamma,
        "scale_metrics": scale_metrics,
        "current_runtime": {
            "line_counts": [len(family.get("line_rhos", [])) for family in result.get("line_families", [])[:2]],
            "point_count": len(result.get("points", [])),
        },
        "frequency_phase": {
            "line_counts": scale_metrics["recovered_line_counts"],
            "point_count_before_filter": int(len(experiment["recovered_rectified_points"])),
            "point_count_after_beam_13cm": int(len(frequency_variant["points"])),
            "vertical_fft_estimate": summarize_fft_estimate(experiment["vertical_fft_estimate"]),
            "horizontal_fft_estimate": summarize_fft_estimate(experiment["horizontal_fft_estimate"]),
        },
        "surface_segmentation": surface_segmentation["stats"],
        "beam_candidate_bands": modalities["beam_candidate_bands"],
        "candidate_variants": [
            {
                "id": variant["id"],
                "title": variant["title"],
                "point_count": len(variant.get("points", [])),
                "mean_score": float(np.mean([point.get("score", 0.0) for point in variant.get("points", [])])) if variant.get("points") else 0.0,
                "note": variant.get("note", ""),
            }
            for variant in candidate_variants
        ],
        "curve3456": [
            {
                "id": variant["id"],
                "point_count": variant["point_count"],
                "mean_score": variant["mean_score"],
                "fallback_to_recovered_grid": variant["fallback_to_recovered_grid"],
            }
            for variant in curve_variants
        ],
        "multiscale_fused": {
            "point_count": len(fused_points),
            "mean_confidence": float(np.mean([point["confidence"] for point in fused_points])) if fused_points else 0.0,
            "source_histogram": {
                source_id: sum(1 for point in fused_points if source_id in point.get("sources", []))
                for source_id in scale_metrics["strategy_weights"].keys()
            },
        },
        "timings_ms": timings_ms,
        "images": image_paths,
    }
    (output_dir / "summary.json").write_text(json.dumps(json_safe(summary), indent=2, ensure_ascii=False), encoding="utf-8")

    cards = [
        ("01_input_high_gamma_workspace.png", "1. 当前画面：高伽马 IR + 工作区"),
        ("02_rectified_ir.png", "2. 透视展开 IR"),
        ("03_combined_response.png", "3. 组合响应底图"),
        ("04_scale_decision_flow.png", "4. 多尺度路线判别"),
        ("05_vertical_profile_fft.png", "5. 纵向 profile + FFT 驼峰"),
        ("06_horizontal_profile_fft.png", "6. 横向 profile + FFT 驼峰"),
        ("07_frequency_phase_lattice_rectified.png", "7. 频相完整网格"),
        ("08_surface_segmentation_overlay.png", "8. 钢筋面 / 梁筋实例分割"),
        ("09_surface_completion_overlay.png", "9. 钢筋面补全"),
        ("10_beam_candidate_13cm_mask.png", "10. 梁筋 ±13cm 排除区"),
        ("11_frequency_phase_after_beam_original.png", "11. 频相拓扑过滤后点位"),
        ("12_completed_surface_intersections_original.png", "12. 补全钢筋面交点"),
        ("13_instance_graph_junctions_original.png", "13. 实例骨架交叉点"),
        ("14_curve04_frequency_dp_original.png", "14. 方案4 DP 曲线局部收束"),
        ("15_multiscale_fused_original.png", "15. PR-FPRG-MS 最终融合点"),
        ("21_candidate_matrix.png", "21. 多源候选矩阵对照"),
    ]
    for index, variant in enumerate(curve_variants, start=17):
        cards.append((f"{index:02d}_{variant['id']}_original.png", f"{index}. 3/4/5/6 对照：{variant['title']}"))

    cards_html = "\n".join(
        f"""
        <figure class="image-card">
          <img src="images/{html.escape(filename)}" alt="{html.escape(caption)}">
          <figcaption>{html.escape(caption)}</figcaption>
        </figure>
        """
        for filename, caption in cards
    )
    rows_html = write_variant_rows(candidate_variants)
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
  <title>PR-FPRG-MS 多尺度绑扎点实验报告</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: #17211d;
      background: #f4f0e7;
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
      line-height: 1.56;
      overflow-x: hidden;
    }}
    header {{
      padding: 22px clamp(16px, 4vw, 44px);
      background: #fbf7ed;
      border-bottom: 1px solid #d7cebe;
    }}
    h1 {{ margin: 0 0 6px; font-size: clamp(23px, 3vw, 34px); letter-spacing: 0; }}
    .subline {{ margin: 0; color: #5b655d; }}
    main {{ max-width: 1580px; margin: 0 auto; padding: 16px clamp(12px, 3vw, 34px) 44px; }}
    .facts {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 196px), 1fr));
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
    .note {{ border-left: 4px solid #b76632; background: #fff8e8; padding: 10px 12px; border-radius: 0 8px 8px 0; }}
    table {{ width: 100%; border-collapse: collapse; background: #fffdf7; border: 1px solid #d8d0c0; border-radius: 8px; overflow: hidden; }}
    th, td {{ padding: 9px 10px; border-bottom: 1px solid #e4dccd; vertical-align: top; text-align: left; }}
    th {{ background: #eee6d5; }}
    code {{ background: #ece3d2; border-radius: 4px; padding: 1px 4px; }}
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
      table {{ font-size: 13px; }}
      .image-card img {{ max-height: 72vh; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>PR-FPRG-MS 多尺度绑扎点实验报告</h1>
    <p class="subline">当前画面真实实验：频相主拓扑、钢筋面补全、实例骨架和 3/4/5/6 曲线收束融合。</p>
  </header>
  <main>
    <div class="facts">
      <div class="fact"><span>帧来源</span><b>{html.escape(str(summary['frame_source']))}</b></div>
      <div class="fact"><span>尺度模式</span><b>{html.escape(scale_metrics['scale_mode'])}</b></div>
      <div class="fact"><span>当前主链</span><b>{summary['current_runtime']['line_counts']} / {summary['current_runtime']['point_count']} 点</b></div>
      <div class="fact"><span>频相滤后</span><b>{len(frequency_variant['points'])} 点</b></div>
      <div class="fact"><span>最终融合</span><b>{len(fused_points)} 点</b></div>
      <div class="fact"><span>总耗时</span><b>{timings_ms['total']:.1f} ms</b></div>
    </div>
    <section>
      <h2>路线结论</h2>
      <p class="note">{html.escape(scale_metrics['scale_reason'])}</p>
      <p>本报告实现的技术路线是：<code>组合响应 -> 尺度判别 -> 频相拓扑 / 钢筋面补全 / 实例骨架 -> 梁筋 ±13cm 点级排除 -> 3/4/5/6 局部收束 -> 多源融合</code>。</p>
      <p>注意：梁筋 mask 只过滤最终绑扎点，不把普通钢筋线从图谱里直接删掉；这样钢筋穿过梁筋的情况仍能参与拓扑恢复。</p>
    </section>
    <section>
      <h2>候选来源对比</h2>
      <table>
        <thead><tr><th>来源</th><th>名称</th><th>点数</th><th>均值响应</th><th>说明</th></tr></thead>
        <tbody>{rows_html}</tbody>
      </table>
    </section>
    <section>
      <h2>3/4/5/6 曲线收束对照</h2>
      <table>
        <thead><tr><th>算法</th><th>点数</th><th>均值响应</th><th>是否兜底直线</th></tr></thead>
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


def publish_report(output_dir, publish_dir):
    output_dir = Path(output_dir)
    publish_dir = Path(publish_dir)
    if output_dir.resolve() == publish_dir.resolve():
        return publish_dir
    if publish_dir.exists():
        shutil.rmtree(publish_dir)
    shutil.copytree(output_dir, publish_dir)
    return publish_dir


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"pr_fprg_multiscale_bindpoint_experiment_{time.strftime('%Y%m%d_%H%M%S')}"),
    )
    parser.add_argument(
        "--publish-dir",
        default=str(WORKSPACE_ROOT / "src" / "tie_robot_web" / "web" / REPORT_RELATIVE_URL),
    )
    parser.add_argument("--snapshot-dir", default=None)
    parser.add_argument("--no-snapshot-fallback", action="store_true")
    parser.add_argument("--threshold-percentile", type=float, default=83.0)
    parser.add_argument("--ir-display-gamma", type=float, default=2.05)
    args = parser.parse_args()

    rospy.init_node("pr_fprg_multiscale_bindpoint_experiment", anonymous=True, disable_signals=True)
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
    experiment = build_multiscale_bindpoint_experiment(
        frame,
        threshold_percentile=args.threshold_percentile,
        ir_display_gamma=args.ir_display_gamma,
    )
    experiment_ms = (time.perf_counter() - started) * 1000.0

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timings_ms = {
        "capture": capture_ms,
        "experiment": experiment_ms,
        "total": (time.perf_counter() - total_start) * 1000.0,
    }
    summary = write_report(output_dir, frame, experiment, timings_ms)
    publish_dir = publish_report(output_dir, args.publish_dir)
    rospy.loginfo("PointAI_log: PR-FPRG-MS 多尺度绑扎点实验报告输出目录：%s", output_dir)
    rospy.loginfo("PointAI_log: PR-FPRG-MS 多尺度绑扎点实验报告发布目录：%s", publish_dir)
    rospy.loginfo("PointAI_log: PR-FPRG-MS report URL path: /%s/index.html", REPORT_RELATIVE_URL)
    return summary


if __name__ == "__main__":
    main()
