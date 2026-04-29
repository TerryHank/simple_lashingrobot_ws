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
    run_peak_supported_pr_fprg,
    to_bgr,
    to_u8,
)
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_curved_line_families,
    build_workspace_s2_oriented_axis_profile,
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
    depth_response = normalize_workspace_s2_response(
        np.asarray(result["response"], dtype=np.float32),
        np.asarray(result["rectified_valid"]).astype(bool),
    )
    infrared_response = build_infrared_dark_response(result)
    depth_weight = float(np.clip(depth_weight, 0.0, 1.0))
    combined = (depth_weight * depth_response) + ((1.0 - depth_weight) * infrared_response)
    return normalize_workspace_s2_response(combined, np.asarray(result["rectified_valid"]).astype(bool))


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


def build_straight_variant(frame, result, variant_id, title, note, line_families, color):
    rectified_points = intersect_workspace_s2_oriented_line_families(
        line_families[0],
        line_families[1],
        result["rectified_geometry"]["rectified_width"],
        result["rectified_geometry"]["rectified_height"],
    )
    points = points_from_rectified_intersections(frame, result, rectified_points)
    original_image = draw_workspace_and_points(frame, result, points, line_color=color)
    original_image = draw_straight_line_families_on_original(original_image, result, line_families, color)
    rectified_image = draw_straight_line_families_on_rectified(result, line_families, color)
    return {
        "id": variant_id,
        "title": title,
        "note": note,
        "type": "straight",
        "line_families": line_families,
        "points": points,
        "rectified_points": rectified_points,
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
):
    curved_families = build_workspace_s2_curved_line_families(
        response_map,
        result["rectified_valid"].astype(np.uint8),
        result["line_families"],
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
            result["line_families"][0],
            result["line_families"][1],
            result["rectified_geometry"]["rectified_width"],
            result["rectified_geometry"]["rectified_height"],
        )
        note = f"{note}；当前曲线无交点时已用直线交点兜底。"
    points = points_from_rectified_intersections(frame, result, rectified_points)
    original_image = draw_workspace_and_points(frame, result, points, line_color=color)
    original_image = draw_curved_line_families_on_original(original_image, result, curved_families, color)
    rectified_image = draw_curved_line_families_on_rectified(result, curved_families, color)
    return {
        "id": variant_id,
        "title": title,
        "note": note,
        "type": "curved",
        "line_families": curved_families,
        "points": points,
        "rectified_points": rectified_points,
        "original_image": original_image,
        "rectified_image": rectified_image,
        "summary": summarize_line_families(curved_families),
    }


def build_variants(frame, result):
    valid_mask = result["rectified_valid"].astype(np.uint8)
    baseline_families = clone_line_families(result["line_families"])
    infrared_response = build_infrared_dark_response(result)
    combined_response = build_combined_depth_ir_response(result)
    infrared_refined_families = refine_line_families_by_auxiliary_profile(
        baseline_families,
        infrared_response,
        valid_mask,
        search_radius_px=8,
    )
    build_initial_rhos_for_families(infrared_refined_families)

    variants = [
        build_straight_variant(
            frame,
            result,
            "01_current_theta_rho",
            "01 当前主链：theta/rho 直线族",
            "当前 active PR-FPRG 输出，保持 spacing_pruned 后的两组直线族。",
            baseline_families,
            (0, 255, 0),
        ),
        build_straight_variant(
            frame,
            result,
            "02_archived_ir_rho",
            "02 归档方案：红外最终 rho 微调",
            "只做每条线的全局 rho 平移，复现已归档的红外贴线思路，便于对比。",
            infrared_refined_families,
            (0, 220, 255),
        ),
        build_curved_variant(
            frame,
            result,
            "03_greedy_depth_curve",
            "03 局部 ridge 贪心曲线",
            "以当前 theta/rho 为拓扑骨架，每个采样点沿法线独立找局部响应中心。",
            result["response"],
            trace_method="greedy",
            score_mode="response",
            color=(255, 90, 0),
        ),
        build_curved_variant(
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
        build_curved_variant(
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
        build_curved_variant(
            frame,
            result,
            "06_ir_assisted_curve",
            "06 红外辅助曲线",
            "深度响应为主，叠加红外暗线响应，只作为后续改进候选。",
            combined_response,
            trace_method="dynamic_programming",
            score_mode="response_ridge",
            color=(180, 255, 60),
            smoothness_weight=0.12,
        ),
    ]
    return variants, {
        "infrared_response": infrared_response,
        "combined_response": combined_response,
    }


def write_report(output_dir, frame, result, variants, extra_images):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    image_dir = output_dir / "images"
    image_dir.mkdir(exist_ok=True)

    cv2.imwrite(str(image_dir / "00_source_workspace.png"), to_bgr(frame["ir"]))
    cv2.imwrite(str(image_dir / "00_depth_response.png"), render_response_heatmap(result["response"]))
    cv2.imwrite(str(image_dir / "00_infrared_response.png"), render_response_heatmap(extra_images["infrared_response"]))
    cv2.imwrite(str(image_dir / "00_combined_response.png"), render_response_heatmap(extra_images["combined_response"]))

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
        "line_stages": result.get("line_stages"),
        "variants": [
            {
                "id": variant["id"],
                "title": variant["title"],
                "note": variant["note"],
                "type": variant["type"],
                "point_count": len(variant["points"]),
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
        cards_html.append(
            f"""
            <section class="variant">
              <div class="variant-header">
                <h2>{html.escape(variant["title"])}</h2>
                <span>{len(variant["points"])} points</span>
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
              <pre>{family_text}</pre>
            </section>
            """
        )

    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PR-FPRG 方案效果对比</title>
  <style>
    :root {{
      color-scheme: light;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f4f6f8;
      color: #17202a;
    }}
    body {{
      margin: 0;
    }}
    header {{
      padding: 24px 28px 18px;
      background: #ffffff;
      border-bottom: 1px solid #d8dee6;
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
    }}
    .source-strip,
    .variant {{
      background: #ffffff;
      border: 1px solid #d8dee6;
      border-radius: 8px;
      padding: 16px;
    }}
    .source-strip h2,
    .variant h2 {{
      margin: 0;
      font-size: 18px;
      letter-spacing: 0;
    }}
    .variant-header {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
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
    .images {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 14px;
    }}
    figure {{
      margin: 0;
      border: 1px solid #e1e7ef;
      border-radius: 8px;
      overflow: hidden;
      background: #0f172a;
    }}
    img {{
      display: block;
      width: 100%;
      height: auto;
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
    </div>
  </header>
  <main>
    <section class="source-strip">
      <h2>输入与响应图</h2>
      <p>左到右分别是红外原图、深度响应、红外暗线响应和深度 + 红外组合响应。</p>
      <div class="images">
        <figure><img src="images/00_source_workspace.png" alt="source"><figcaption>红外输入</figcaption></figure>
        <figure><img src="images/00_depth_response.png" alt="depth response"><figcaption>深度响应</figcaption></figure>
        <figure><img src="images/00_infrared_response.png" alt="infrared response"><figcaption>红外暗线响应</figcaption></figure>
        <figure><img src="images/00_combined_response.png" alt="combined response"><figcaption>组合响应</figcaption></figure>
      </div>
    </section>
    {''.join(cards_html)}
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
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"pr_fprg_scheme_comparison_{time.strftime('%Y%m%d_%H%M%S')}"),
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

    result = run_peak_supported_pr_fprg(frame)
    variants, extra_images = build_variants(frame, result)
    report_path = write_report(args.output_dir, frame, result, variants, extra_images)
    rospy.loginfo("PointAI_log: PR-FPRG方案对比报告：%s", report_path)
    print(str(report_path))


if __name__ == "__main__":
    main()
