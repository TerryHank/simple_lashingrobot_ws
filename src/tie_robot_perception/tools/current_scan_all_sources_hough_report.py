#!/usr/bin/env python3

"""Generate a report-only Hough line/intersection view for all scan sources."""

from __future__ import annotations

import argparse
import html
import json
import sys
import time
from pathlib import Path

import cv2
import rospy


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src"
TOOL_DIR = Path(__file__).resolve().parent
for import_path in (PERCEPTION_SRC, TOOL_DIR):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from current_scan_all_sources_report import (  # noqa: E402
    BASE_SOURCE_ORDER,
    COMPLETED_SOURCE_ORDER,
    SOURCE_LABELS,
    build_rectified_runtime_input,
    capture_current_frame,
    json_safe,
    render_ir_workspace,
    safe_filename,
)
from pr_fprg_peak_supported_probe import normalize_probe_raw_world  # noqa: E402
from rebar_instance_graph_probe import load_latest_snapshot_frame  # noqa: E402
from scan_hough_intersection_experiment import (  # noqa: E402
    extract_hough_intersections,
    json_safe_hough_result,
    render_hough_overlay,
    render_hough_stage_image,
)
from tie_robot_perception.pointai import scan_surface_dp  # noqa: E402


def build_hough_response_maps(runtime_input, threshold_percentile=83.0):
    valid_mask = runtime_input["rectified_valid"]
    depth_response = scan_surface_dp.build_depth_response(runtime_input)
    infrared_response = scan_surface_dp.build_infrared_response(runtime_input)
    combined_response = scan_surface_dp.build_combined_response(runtime_input)
    depth_gradient = scan_surface_dp.build_depth_gradient_response(runtime_input)
    hessian_ridge = scan_surface_dp.hessian_ridge_response(combined_response, valid_mask)
    frangi_like = scan_surface_dp.multiscale_frangi_like_response(combined_response, valid_mask)
    fused_instance_response = scan_surface_dp._normalize_response(
        (0.50 * combined_response)
        + (0.22 * depth_response)
        + (0.16 * frangi_like)
        + (0.08 * depth_gradient)
        + (0.04 * hessian_ridge),
        valid_mask,
    )
    binary_candidate, _binary_threshold = scan_surface_dp.threshold_response(
        fused_instance_response,
        valid_mask,
        percentile=threshold_percentile,
    )
    line_families, _physical_source = scan_surface_dp._build_best_physical_axis_aligned_line_families(
        [(source_name, response_map) for source_name, response_map in (
            ("depth_gradient", depth_gradient),
            ("frangi_like", frangi_like),
            ("hessian_ridge", hessian_ridge),
            ("infrared_response", infrared_response),
            ("combined_response", combined_response),
            ("depth_response", depth_response),
            ("fused_instance_response", fused_instance_response),
        )],
        valid_mask,
        runtime_input.get("rectified_geometry") or {},
        peak_min_ratio=0.18,
    )
    line_support_mask = scan_surface_dp.draw_line_family_mask(
        binary_candidate.shape,
        line_families[:2],
        thickness_px=5,
    )
    completed_surface_mask = (binary_candidate | line_support_mask) & valid_mask
    completed_surface_response = scan_surface_dp._normalize_response(
        (0.70 * fused_instance_response)
        + (0.20 * line_support_mask.astype("float32"))
        + (0.10 * completed_surface_mask.astype("float32")),
        valid_mask,
    )
    return {
        "depth_response": depth_response,
        "infrared_response": infrared_response,
        "combined_response": combined_response,
        "depth_gradient": depth_gradient,
        "hessian_ridge": hessian_ridge,
        "frangi_like": frangi_like,
        "fused_instance_response": fused_instance_response,
        "completed_surface_response": completed_surface_response,
    }


def evaluate_hough_source(
    runtime_input,
    source_name,
    response_map,
    threshold_percentile=86.0,
):
    valid_mask = runtime_input["rectified_valid"]
    hough_result = extract_hough_intersections(
        response_map,
        valid_mask=valid_mask,
        threshold_percentile=threshold_percentile,
    )
    intersections = hough_result.get("intersections", [])
    mean_response = 0.0
    if intersections:
        mean_response = float(
            sum(scan_surface_dp._response_value_at(response_map, point) for point in intersections)
            / float(len(intersections))
        )
    return {
        "id": source_name,
        "label": SOURCE_LABELS.get(source_name, source_name),
        "response_map": response_map,
        "hough": hough_result,
        "line_counts": hough_result.get("line_counts", [0, 0]),
        "point_count": int(len(intersections)),
        "mean_response_at_points": float(mean_response),
        "accepted": bool(intersections),
    }


def build_hough_evaluation(runtime_input, threshold_percentile=83.0, hough_threshold_percentile=86.0):
    response_maps = build_hough_response_maps(runtime_input, threshold_percentile)
    source_order = []
    for source_name in list(BASE_SOURCE_ORDER) + list(COMPLETED_SOURCE_ORDER):
        if source_name in response_maps and source_name not in source_order:
            source_order.append(source_name)

    hough_rows = [
        evaluate_hough_source(
            runtime_input,
            source_name,
            response_maps[source_name],
            threshold_percentile=hough_threshold_percentile,
        )
        for source_name in source_order
    ]
    return {
        "response_maps": response_maps,
        "source_order": source_order,
        "hough_rows": hough_rows,
    }


def capture_frame_for_hough_report(
    timeout_sec,
    allow_depth_ir_fallback=True,
    snapshot_dir=None,
    allow_snapshot_fallback=True,
):
    try:
        return capture_current_frame(
            timeout_sec,
            allow_depth_ir_fallback=allow_depth_ir_fallback,
        )
    except RuntimeError as exc:
        rospy.logwarn("current Hough all-source report: live frame unavailable: %s", exc)
        if not allow_snapshot_fallback:
            raise

    frame = load_latest_snapshot_frame(snapshot_dir)
    raw_world, used_depth_fallback_raw_world = normalize_probe_raw_world(frame["raw"])
    frame["raw"] = raw_world
    frame["used_depth_fallback_raw_world"] = bool(used_depth_fallback_raw_world)
    return frame


def write_report(output_dir, frame, runtime_input, evaluation, timings_ms, hough_threshold_percentile):
    output_dir = Path(output_dir)
    images_dir = output_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    valid_mask = runtime_input["rectified_valid"]

    def write_image(filename, image):
        path = images_dir / filename
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f"failed to write image: {path}")
        return f"images/{filename}"

    image_paths = {
        "input_workspace": write_image("00_input_workspace.png", render_ir_workspace(runtime_input)),
    }

    stage_specs = [
        ("response", "_01_response.png", "响应底图"),
        ("binary", "_02_binary.png", "二值化"),
        ("skeleton", "_03_skeleton.png", "骨架"),
        ("segments", "_04_hough_segments.png", "Hough 线段"),
        ("clustered_lines", "_05_clustered_lines.png", "聚类横纵线"),
        ("intersections", "_06_intersections.png", "最终交点"),
    ]
    row_summaries = []
    tab_html = []
    panel_html = []
    for row_index, row in enumerate(evaluation["hough_rows"]):
        prefix = safe_filename(row["id"])
        filename = f"{prefix}_hough_rectified.png"
        image_paths[f"{prefix}_hough_rectified"] = write_image(
            filename,
            render_hough_overlay(
                row["response_map"],
                valid_mask,
                row["hough"],
                f"hough {row['id']} lines={row['line_counts']} pts={row['point_count']}",
            ),
        )
        stage_images = {}
        stage_cards = []
        for stage_name, stage_suffix, stage_label in stage_specs:
            stage_filename = f"{prefix}{stage_suffix}"
            stage_key = f"{prefix}{stage_suffix[:-4]}"
            stage_images[stage_name] = write_image(
                stage_filename,
                render_hough_stage_image(
                    row["response_map"],
                    valid_mask,
                    row["hough"],
                    stage_name,
                    label=f"{row['id']} {stage_label}",
                ),
            )
            stage_cards.append(
                f"""
                <figure class="image-card">
                  <img src="images/{html.escape(stage_filename)}" alt="{html.escape(row['label'] + ' ' + stage_label)}">
                  <figcaption>{html.escape(stage_label)}</figcaption>
                </figure>
                """
            )
            image_paths[stage_key] = stage_images[stage_name]
        row_summary = {
            "id": row["id"],
            "label": row["label"],
            "accepted": bool(row["accepted"]),
            "mean_response_at_points": float(row["mean_response_at_points"]),
            "stage_images": stage_images,
            **json_safe_hough_result(row["hough"]),
        }
        row_summaries.append(row_summary)

        active_class = " active" if row_index == 0 else ""
        active_attr = "true" if row_index == 0 else "false"
        panel_hidden_attr = "" if row_index == 0 else " hidden"
        tab_html.append(
            f"""
            <button class="source-tab{active_class}" type="button" data-source-id="{html.escape(row['id'])}" aria-pressed="{active_attr}">
              <span>{html.escape(row['label'])}</span>
              <b>{html.escape(str(row['line_counts']))} / {row['point_count']}点</b>
            </button>
            """
        )
        panel_html.append(
            f"""
            <section class="source-panel{active_class}" data-source-id="{html.escape(row['id'])}"{panel_hidden_attr}>
              <div class="source-head">
                <div>
                  <h2>{html.escape(row['label'])}</h2>
                  <p>当前底图独立完成二值化、骨架、Hough 线段、横纵线聚类和交点生成；下方是该底图流程效果图。</p>
                </div>
                <div class="source-metrics">
                  <div><span>线数</span><b>{html.escape(str(row_summary['line_counts']))}</b></div>
                  <div><span>交点</span><b>{row_summary['point_count']}</b></div>
                  <div><span>原始线段</span><b>{row_summary['raw_line_count']}</b></div>
                  <div><span>保留线段</span><b>{row_summary['accepted_line_count']}</b></div>
                  <div><span>二值像素</span><b>{row_summary['binary_pixels']}</b></div>
                  <div><span>阈值</span><b>{row_summary['threshold']:.3f}</b></div>
                </div>
              </div>
              <div class="flow-grid">{''.join(stage_cards)}</div>
            </section>
            """
        )

    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source"),
        "used_depth_fallback_raw_world": bool(runtime_input.get("used_depth_fallback_raw_world", False)),
        "rectified_size": [
            int(runtime_input["rectified_geometry"]["rectified_width"]),
            int(runtime_input["rectified_geometry"]["rectified_height"]),
        ],
        "source_order": evaluation["source_order"],
        "hough_threshold_percentile": float(hough_threshold_percentile),
        "hough_rows": row_summaries,
        "timings_ms": timings_ms,
        "images": image_paths,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(json_safe(summary), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>扫描全底图 Hough 交点实验</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: #d9e1dc;
      background: #111613;
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
      line-height: 1.5;
    }}
    header {{
      position: sticky;
      top: 0;
      z-index: 10;
      padding: 14px clamp(12px, 2vw, 26px) 12px;
      background: #151b18;
      border-bottom: 1px solid #2a342f;
      box-shadow: 0 10px 28px rgba(0, 0, 0, 0.24);
    }}
    .topline {{ display: flex; align-items: baseline; justify-content: space-between; gap: 16px; }}
    h1 {{ margin: 0; font-size: clamp(20px, 2.4vw, 30px); letter-spacing: 0; }}
    .subline {{ margin: 4px 0 0; color: #8fa098; font-size: 13px; }}
    main {{ max-width: 1780px; margin: 0 auto; padding: 14px clamp(10px, 2vw, 26px) 34px; }}
    .source-switcher {{
      display: flex;
      gap: 8px;
      overflow-x: auto;
      padding: 12px 0 2px;
      scrollbar-width: thin;
    }}
    .source-tab {{
      flex: 0 0 auto;
      min-width: 176px;
      border: 1px solid #35423c;
      border-radius: 8px;
      padding: 8px 10px;
      background: #1d2521;
      color: #d9e1dc;
      text-align: left;
      cursor: pointer;
    }}
    .source-tab span {{ display: block; font-size: 13px; white-space: nowrap; }}
    .source-tab b {{ display: block; margin-top: 3px; color: #98b8aa; font-size: 12px; font-weight: 600; }}
    .source-tab.active {{
      background: #23382f;
      border-color: #6fb395;
      color: #f2faf6;
    }}
    .facts {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 190px), 1fr));
      gap: 10px;
      margin: 0 0 14px;
    }}
    .fact {{
      border: 1px solid #2d3933;
      border-radius: 8px;
      padding: 10px 12px;
      background: #161d1a;
      min-width: 0;
    }}
    .fact span {{ display: block; color: #899990; font-size: 12px; }}
    .fact b {{ display: block; font-size: 18px; overflow-wrap: anywhere; }}
    h2 {{ margin: 0 0 4px; font-size: 22px; letter-spacing: 0; }}
    .note {{
      border-left: 4px solid #6fb395;
      background: #18221e;
      padding: 10px 12px;
      border-radius: 0 8px 8px 0;
      color: #b8c7c0;
    }}
    .overview {{
      display: grid;
      grid-template-columns: minmax(240px, 360px) 1fr;
      gap: 14px;
      align-items: stretch;
      margin-bottom: 14px;
    }}
    .source-panel {{
      display: block;
      border: 1px solid #2d3933;
      border-radius: 8px;
      background: #151b18;
      padding: 14px;
    }}
    .source-panel[hidden] {{ display: none; }}
    .source-head {{
      display: grid;
      grid-template-columns: minmax(260px, 1fr) minmax(360px, 520px);
      gap: 14px;
      align-items: start;
      margin-bottom: 14px;
    }}
    .source-head p {{ margin: 0; color: #8fa098; }}
    .source-metrics {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }}
    .source-metrics div {{
      border: 1px solid #2d3933;
      border-radius: 8px;
      padding: 8px 9px;
      background: #111613;
      min-width: 0;
    }}
    .source-metrics span {{ display: block; color: #7f9187; font-size: 12px; }}
    .source-metrics b {{ display: block; margin-top: 2px; overflow-wrap: anywhere; }}
    .flow-grid {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }}
    .image-card {{
      margin: 0;
      border: 1px solid #2d3933;
      border-radius: 8px;
      background: #111613;
      overflow: hidden;
      min-width: 0;
    }}
    .image-card img {{
      display: block;
      width: 100%;
      max-height: min(44vh, 520px);
      object-fit: contain;
      background: #10140f;
    }}
    figcaption {{ padding: 8px 9px; color: #b7c6bf; border-top: 1px solid #2d3933; font-size: 13px; }}
    @media (max-width: 760px) {{
      header {{ padding: 12px; }}
      main {{ padding: 10px; }}
      .topline, .overview, .source-head {{ display: block; }}
      .overview .image-card {{ margin-bottom: 10px; }}
      .source-metrics {{ grid-template-columns: repeat(2, minmax(0, 1fr)); margin-top: 10px; }}
      .flow-grid {{ grid-template-columns: 1fr; }}
      .image-card img {{ max-height: 70vh; }}
    }}
  </style>
</head>
<body>
  <header>
    <div class="topline">
      <div>
        <h1>扫描全底图 Hough 交点实验</h1>
        <p class="subline">帧来源：{html.escape(str(frame.get("frame_source")))}；生成时间：{html.escape(summary["generated_at"])}。本报告只做离线诊断，不改 Surface-DP 主链。</p>
      </div>
      <div class="fact"><span>rectified</span><b>{summary["rectified_size"][0]} x {summary["rectified_size"][1]}</b></div>
    </div>
    <nav class="source-switcher" aria-label="底图切换">
      {''.join(tab_html)}
    </nav>
  </header>
  <main>
    <div class="overview">
      <figure class="image-card">
        <img src="images/00_input_workspace.png" alt="当前输入工作区">
        <figcaption>当前输入工作区</figcaption>
      </figure>
      <div>
        <div class="facts">
          <div class="fact"><span>底图数量</span><b>{len(summary["hough_rows"])}</b></div>
          <div class="fact"><span>Hough 阈值百分位</span><b>{summary["hough_threshold_percentile"]:.1f}</b></div>
          <div class="fact"><span>总耗时</span><b>{timings_ms["total"]:.1f} ms</b></div>
        </div>
        <p class="note">点击顶部底图标签切换。每页只显示当前底图的「响应底图 → 二值化 → 骨架 → Hough 线段 → 聚类横纵线 → 最终交点」。黄色点是 Hough 横纵线交点；绿线偏竖向，蓝黄线偏横向。</p>
      </div>
    </div>
    {''.join(panel_html)}
  </main>
  <script>
    const tabs = Array.from(document.querySelectorAll('.source-tab'));
    const panels = Array.from(document.querySelectorAll('.source-panel'));
    function activateSource(sourceId) {{
      tabs.forEach((tab) => {{
        const active = tab.dataset.sourceId === sourceId;
        tab.classList.toggle('active', active);
        tab.setAttribute('aria-pressed', active ? 'true' : 'false');
      }});
      panels.forEach((panel) => {{
        const active = panel.dataset.sourceId === sourceId;
        panel.classList.toggle('active', active);
        panel.hidden = !active;
      }});
    }}
    tabs.forEach((tab) => {{
      tab.addEventListener('click', () => activateSource(tab.dataset.sourceId));
    }});
  </script>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"current_scan_all_sources_hough_{time.strftime('%Y%m%d_%H%M%S')}"),
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--threshold-percentile", type=float, default=83.0)
    parser.add_argument("--hough-threshold-percentile", type=float, default=86.0)
    parser.add_argument("--response-source", default="depth_gradient")
    parser.add_argument("--no-depth-ir-fallback", action="store_true")
    parser.add_argument("--snapshot-dir", default=None)
    parser.add_argument("--no-snapshot-fallback", action="store_true")
    args = parser.parse_args()

    total_start = time.perf_counter()
    rospy.init_node("current_scan_all_sources_hough_report", anonymous=True)

    started = time.perf_counter()
    frame = capture_frame_for_hough_report(
        args.timeout,
        allow_depth_ir_fallback=not args.no_depth_ir_fallback,
        snapshot_dir=args.snapshot_dir,
        allow_snapshot_fallback=not args.no_snapshot_fallback,
    )
    capture_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    runtime_input = build_rectified_runtime_input(frame)
    input_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    evaluation = build_hough_evaluation(
        runtime_input,
        threshold_percentile=args.threshold_percentile,
        hough_threshold_percentile=args.hough_threshold_percentile,
    )
    evaluation_ms = (time.perf_counter() - started) * 1000.0

    timings_ms = {
        "capture": capture_ms,
        "build_rectified_input": input_ms,
        "evaluate": evaluation_ms,
        "total": (time.perf_counter() - total_start) * 1000.0,
    }
    write_report(
        args.output_dir,
        frame,
        runtime_input,
        evaluation,
        timings_ms,
        hough_threshold_percentile=args.hough_threshold_percentile,
    )
    print(f"current scan all-source Hough report: {args.output_dir}")


if __name__ == "__main__":
    main()
