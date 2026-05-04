#!/usr/bin/env python3

"""Visual report for depth-gradient + Hessian/Frangi scan candidates.

The report is offline-only. It loads a fixed snapshot, builds a focused
gradient/ridge response stack, and writes effect images that can be opened in a
browser.
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


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src"
TOOL_DIR = Path(__file__).resolve().parent
for import_path in (PERCEPTION_SRC, TOOL_DIR):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from pr_fprg_peak_supported_probe import normalize_probe_raw_world, run_peak_supported_pr_fprg  # noqa: E402
from rebar_instance_graph_probe import load_latest_snapshot_frame, render_binary, render_heatmap, render_skeleton_overlay  # noqa: E402
from rebar_surface_bindpoint_comparison import (  # noqa: E402
    build_montage,
    points_from_rectified_points,
    render_ir_high_gamma,
    render_original_variant,
    render_rectified_variant,
    response_value_at,
)
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_axis_aligned_line_families,
    build_workspace_s2_curved_line_families,
    intersect_workspace_s2_curved_line_families,
    intersect_workspace_s2_oriented_line_families,
    normalize_workspace_s2_response,
)
from tie_robot_perception.pointai.scan_surface_dp import (  # noqa: E402
    build_combined_response,
    build_depth_gradient_response,
    count_skeleton_nodes,
    draw_line_family_mask,
    hessian_ridge_response,
    multiscale_frangi_like_response,
    skeletonize_binary,
    threshold_response,
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


def load_snapshot_frame(snapshot_dir):
    frame = load_latest_snapshot_frame(Path(snapshot_dir))
    raw_world, used_depth_fallback_raw_world = normalize_probe_raw_world(frame["raw"])
    frame["raw"] = raw_world
    frame["used_depth_fallback_raw_world"] = bool(used_depth_fallback_raw_world)
    return frame


def build_gradient_ridge_scheme(result, threshold_percentile=83.0):
    valid_mask = np.asarray(result["rectified_valid"]).astype(bool)
    combined_response = build_combined_response(result)
    depth_gradient = build_depth_gradient_response(result)
    hessian_ridge = hessian_ridge_response(combined_response, valid_mask, sigma=1.6)
    frangi_like = multiscale_frangi_like_response(combined_response, valid_mask)

    gradient_ridge_response = normalize_workspace_s2_response(
        (0.34 * combined_response)
        + (0.22 * depth_gradient)
        + (0.22 * hessian_ridge)
        + (0.22 * frangi_like),
        valid_mask,
    )
    binary_candidate, binary_threshold = threshold_response(
        gradient_ridge_response,
        valid_mask,
        percentile=threshold_percentile,
    )
    skeleton = skeletonize_binary(binary_candidate)
    endpoint_count, junction_count = count_skeleton_nodes(skeleton)

    base_line_families = build_workspace_s2_axis_aligned_line_families(
        gradient_ridge_response,
        valid_mask.astype(np.uint8),
        min_period=10,
        max_period=30,
        enable_local_peak_refine=True,
        enable_peak_support=True,
        enable_continuous_validation=False,
        enable_spacing_prune=True,
        enable_structural_edge_suppression=False,
        peak_min_ratio=0.20,
    )
    line_support_mask = draw_line_family_mask(binary_candidate.shape, base_line_families, thickness_px=5)
    completed_surface_mask = (binary_candidate | line_support_mask) & valid_mask
    completed_surface_response = normalize_workspace_s2_response(
        (0.44 * gradient_ridge_response)
        + (0.16 * depth_gradient)
        + (0.16 * hessian_ridge)
        + (0.16 * frangi_like)
        + (0.08 * line_support_mask.astype(np.float32)),
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
        peak_min_ratio=0.20,
    )
    if len(completed_line_families) < 2:
        completed_line_families = base_line_families
    completed_line_families = completed_line_families[:2]

    curved_families = []
    rectified_points = []
    if len(completed_line_families) >= 2:
        curved_families = build_workspace_s2_curved_line_families(
            completed_surface_response,
            valid_mask.astype(np.uint8),
            completed_line_families,
            trace_method="dynamic_programming",
            score_mode="response_ridge",
            search_radius_px=9,
            sample_step_px=4,
            min_response_ratio=0.14,
            smoothing_window_samples=5,
            smoothness_weight=0.12,
        )
        rectified_points = intersect_workspace_s2_curved_line_families(
            curved_families[0],
            curved_families[1],
            result["rectified_geometry"]["rectified_width"],
            result["rectified_geometry"]["rectified_height"],
        )
        if not rectified_points:
            rectified_points = intersect_workspace_s2_oriented_line_families(
                completed_line_families[0],
                completed_line_families[1],
                result["rectified_geometry"]["rectified_width"],
                result["rectified_geometry"]["rectified_height"],
            )
    points = points_from_rectified_points(
        result,
        rectified_points,
        completed_surface_response,
        "depth_gradient_hessian_frangi_dp",
    )
    response_scores = [response_value_at(completed_surface_response, point["rectified"]) for point in points]
    modalities = {
        "combined_response": combined_response,
        "depth_gradient": depth_gradient,
        "hessian_ridge": hessian_ridge,
        "frangi_like": frangi_like,
        "gradient_ridge_response": gradient_ridge_response,
        "binary_candidate": binary_candidate,
        "skeleton": skeleton,
    }
    surface_segmentation = {
        "ordinary_rebar_mask": binary_candidate,
        "ordinary_completed_rebar_mask": completed_surface_mask,
        "line_support_mask": line_support_mask,
        "completed_surface_mask": completed_surface_mask,
        "completed_surface_response": completed_surface_response,
        "beam_candidate_mask": np.zeros_like(valid_mask, dtype=bool),
        "beam_candidate_body_mask": np.zeros_like(valid_mask, dtype=bool),
        "beam_margin_only_mask": np.zeros_like(valid_mask, dtype=bool),
    }
    variant = {
        "id": "depth_gradient_hessian_frangi_dp",
        "title": "depth_gradient + Hessian + Frangi DP 曲线",
        "points": points,
        "point_count": len(points),
        "line_families": curved_families or completed_line_families,
        "curved_families": curved_families,
    }
    summary = {
        "id": variant["id"],
        "point_count": len(points),
        "line_counts": [len(family.get("line_rhos", [])) for family in completed_line_families],
        "binary_threshold": float(binary_threshold),
        "binary_candidate_pixels": int(np.count_nonzero(binary_candidate)),
        "completed_surface_pixels": int(np.count_nonzero(completed_surface_mask)),
        "instance_graph_endpoint_count": int(endpoint_count),
        "instance_graph_junction_count": int(junction_count),
        "mean_completed_surface_score": float(np.mean(response_scores)) if response_scores else 0.0,
    }
    return modalities, surface_segmentation, variant, summary


def write_report(output_dir, frame, result, modalities, surface_segmentation, variant, scheme_summary):
    output_dir = Path(output_dir)
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)

    images = {
        "00_input_workspace.png": render_ir_high_gamma(frame, result, ir_display_gamma=1.95),
        "01_combined_response.png": render_heatmap(modalities["combined_response"]),
        "02_depth_gradient.png": render_heatmap(modalities["depth_gradient"]),
        "03_hessian_ridge.png": render_heatmap(modalities["hessian_ridge"]),
        "04_frangi_like.png": render_heatmap(modalities["frangi_like"]),
        "05_gradient_hessian_frangi_fusion.png": render_heatmap(modalities["gradient_ridge_response"]),
        "06_binary_candidate.png": render_binary(modalities["binary_candidate"], "binary_candidate"),
        "07_skeleton_on_fusion.png": render_skeleton_overlay(modalities["gradient_ridge_response"], modalities["skeleton"]),
        "08_completed_surface_response.png": render_heatmap(surface_segmentation["completed_surface_response"]),
        "09_dp_curve_rectified_points.png": render_rectified_variant(modalities, surface_segmentation, variant),
        "10_dp_curve_original_points.png": render_original_variant(frame, result, variant, ir_display_gamma=1.95),
    }
    image_paths = {}
    for filename, image in images.items():
        path = image_dir / filename
        cv2.imwrite(str(path), image)
        image_paths[filename] = str(path)

    montage_path = image_dir / "90_effect_montage.png"
    build_montage(
        [image_dir / name for name in images.keys()],
        montage_path,
        [name.replace(".png", "") for name in images.keys()],
        tile_width=500,
        tile_height=390,
    )
    image_paths["90_effect_montage.png"] = str(montage_path)

    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source"),
        "base_response": result.get("response_name"),
        "base_line_counts": [len(family.get("line_rhos", [])) for family in result.get("line_families", [])],
        "scheme": scheme_summary,
        "images": image_paths,
        "note": "This is an offline visual report. It does not change the runtime mainline.",
    }
    (output_dir / "summary.json").write_text(json.dumps(json_safe(summary), ensure_ascii=False, indent=2), encoding="utf-8")

    cards = "\n".join(
        f"""
        <figure>
          <img src="images/{html.escape(filename)}" alt="{html.escape(filename)}">
          <figcaption>{html.escape(filename)}</figcaption>
        </figure>
        """
        for filename in [
            "90_effect_montage.png",
            "00_input_workspace.png",
            "01_combined_response.png",
            "02_depth_gradient.png",
            "03_hessian_ridge.png",
            "04_frangi_like.png",
            "05_gradient_hessian_frangi_fusion.png",
            "06_binary_candidate.png",
            "07_skeleton_on_fusion.png",
            "08_completed_surface_response.png",
            "09_dp_curve_rectified_points.png",
            "10_dp_curve_original_points.png",
        ]
    )
    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>depth_gradient + Hessian + Frangi 效果图</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: #17211d;
      background: #f6f1e8;
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
      line-height: 1.55;
    }}
    header {{ padding: 22px clamp(16px, 4vw, 42px); background: #fffaf0; border-bottom: 1px solid #d8cfbf; }}
    h1 {{ margin: 0 0 6px; font-size: clamp(22px, 3vw, 34px); letter-spacing: 0; }}
    main {{ max-width: 1660px; margin: 0 auto; padding: 18px clamp(12px, 3vw, 34px) 42px; }}
    .facts {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(min(100%, 210px), 1fr)); gap: 10px; margin-bottom: 18px; }}
    .fact {{ background: #fffdf7; border: 1px solid #d8cfbf; border-radius: 8px; padding: 10px 12px; }}
    .fact span {{ display: block; color: #657064; font-size: 12px; }}
    .fact b {{ display: block; font-size: 18px; overflow-wrap: anywhere; }}
    .pipeline {{ background: #fff8e8; border-left: 4px solid #b55d2d; border-radius: 0 8px 8px 0; padding: 10px 12px; margin: 0 0 18px; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(min(100%, 430px), 1fr)); gap: 14px; align-items: start; }}
    figure {{ margin: 0; background: #fffdf7; border: 1px solid #d8cfbf; border-radius: 8px; overflow: hidden; }}
    img {{ display: block; width: 100%; max-height: min(82vh, 980px); object-fit: contain; background: #10140f; }}
    figcaption {{ padding: 9px 10px; border-top: 1px solid #e4dccd; color: #3d453f; }}
    code {{ background: #ece3d2; border-radius: 4px; padding: 1px 4px; }}
  </style>
</head>
<body>
  <header>
    <h1>depth_gradient + Hessian + Frangi 脊线增强效果图</h1>
    <div>固定快照：<code>{html.escape(str(frame.get("frame_source")))}</code></div>
  </header>
  <main>
    <div class="facts">
      <div class="fact"><span>基础响应</span><b>{html.escape(str(result.get("response_name")))}</b></div>
      <div class="fact"><span>基础线族</span><b>{html.escape(str(summary["base_line_counts"]))}</b></div>
      <div class="fact"><span>新方案点数</span><b>{scheme_summary["point_count"]}</b></div>
      <div class="fact"><span>新方案线族</span><b>{html.escape(str(scheme_summary["line_counts"]))}</b></div>
      <div class="fact"><span>补全面均分</span><b>{scheme_summary["mean_completed_surface_score"]:.3f}</b></div>
      <div class="fact"><span>骨架 junction</span><b>{scheme_summary["instance_graph_junction_count"]}</b></div>
    </div>
    <p class="pipeline">
      新方案：<code>combined_response + depth_gradient + Hessian ridge + Frangi-like</code>
      -> 二值候选 -> skeleton 诊断 -> completed_surface_mask -> DP 曲线 -> 曲线交点。
    </p>
    <div class="grid">
      {cards}
    </div>
  </main>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")
    return summary


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--snapshot-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / "rebar_instance_segmentation_modalities_20260430_112028"),
    )
    parser.add_argument(
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / "depth_gradient_hessian_frangi_scheme_20260504"),
    )
    parser.add_argument("--threshold-percentile", type=float, default=83.0)
    return parser.parse_args()


def main():
    args = parse_args()
    frame = load_snapshot_frame(args.snapshot_dir)
    result = run_peak_supported_pr_fprg(frame, response_name_filter="combined_depth_ir_darkline")
    modalities, surface_segmentation, variant, scheme_summary = build_gradient_ridge_scheme(
        result,
        threshold_percentile=args.threshold_percentile,
    )
    summary = write_report(args.output_dir, frame, result, modalities, surface_segmentation, variant, scheme_summary)
    print(f"depth gradient ridge scheme report: {args.output_dir}")
    print(json.dumps(json_safe(summary["scheme"]), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
