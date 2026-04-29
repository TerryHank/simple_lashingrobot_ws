#!/usr/bin/env python3

"""Generate an HTML report for PR-FPRG stage-ablation speed and visual effects."""

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
    capture_depth_ir_frame,
    capture_synced_frame,
    render_response_heatmap,
    to_bgr,
    to_u8,
)
from pr_fprg_scheme_comparison import (  # noqa: E402
    draw_straight_line_families_on_original,
    draw_straight_line_families_on_rectified,
    points_from_rectified_intersections,
)
from pr_fprg_stage_ablation import (  # noqa: E402
    point_drift_stats,
    prepare_frame_inputs,
    run_line_family_variant,
    summarize_families,
)


VARIANTS = [
    {
        "id": "full",
        "label": "全流程",
        "stage": "基准",
        "description": "保留 peak 支撑、连续钢筋条验证和 spacing prune，是当前直线 PR-FPRG 主链。",
        "conclusion": "基准",
        "tone": "baseline",
    },
    {
        "id": "skip_continuous",
        "label": "去掉连续钢筋条验证",
        "stage": "连续验证",
        "description": "rho profile 出线后，不再沿钢筋方向采样检查是否是一整条连续 ridge。",
        "conclusion": "不能删除：速度提升明显，但会多出假线/假点。",
        "tone": "danger",
        "enable_continuous_validation": False,
    },
    {
        "id": "skip_spacing",
        "label": "去掉 spacing prune",
        "stage": "间距筛选",
        "description": "保留连续验证结果，不再删除与主间距过近的边缘候选线。",
        "conclusion": "不建议删除：本帧不省时，且可能多线。",
        "tone": "warn",
        "enable_spacing_prune": False,
    },
    {
        "id": "skip_peak_support",
        "label": "去掉 peak 支撑过滤",
        "stage": "peak 支撑",
        "description": "周期相位给出的候选线不再要求 profile 局部峰值支撑。",
        "conclusion": "不能删除：可能无法稳定产出两组线族。",
        "tone": "danger",
        "enable_peak_support": False,
    },
    {
        "id": "skip_local_refine",
        "label": "去掉局部峰值 refine",
        "stage": "局部 refine",
        "description": "不把周期线整体平移到附近局部峰值；当前主链默认已跳过此阶段。",
        "conclusion": "可跳过：现场消融 0 漂移，节省少量耗时。",
        "tone": "ok",
        "enable_local_peak_refine": False,
    },
    {
        "id": "profile_only_spacing",
        "label": "只保留 profile 周期 + spacing",
        "stage": "profile-only",
        "description": "关闭局部 refine、peak 支撑和连续验证，只保留周期线与 spacing prune。",
        "conclusion": "不能用：速度快，但会产生大量假点。",
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
        "description": "只根据 theta/rho profile 周期相位生成线，所有后续筛选都关闭。",
        "conclusion": "不能用：最快但假点最多。",
        "tone": "danger",
        "enable_local_peak_refine": False,
        "enable_peak_support": False,
        "enable_continuous_validation": False,
        "enable_spacing_prune": False,
    },
    {
        "id": "full_angle_sweep",
        "label": "全角度扫描全流程",
        "stage": "theta 候选池",
        "description": "不用梯度方向先验收窄 theta 候选，而是回到全角度扫描。",
        "conclusion": "不建议恢复：更慢，且现场可能不更稳。",
        "tone": "warn",
        "use_orientation_prior_angle_pool": False,
    },
]


COLORS = {
    "baseline": (0, 255, 0),
    "ok": (22, 163, 74),
    "warn": (0, 180, 255),
    "danger": (0, 0, 255),
}


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


def build_result_like(prepared):
    return {
        "manual_workspace": prepared["manual_workspace"],
        "workspace_mask": prepared["workspace_mask"],
        "rectified_geometry": prepared["rectified_geometry"],
        "rectified_ir": prepared["rectified_ir"],
    }


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


def render_original_overlay(frame, prepared, line_families, points, color):
    result_like = build_result_like(prepared)
    image = cv2.cvtColor(to_u8(frame["ir"]), cv2.COLOR_GRAY2BGR)
    polygon = np.asarray(prepared["manual_workspace"]["corner_pixels"], dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [polygon], True, (232, 232, 232), 2)
    image = draw_straight_line_families_on_original(image, result_like, line_families, color)
    return draw_points(image, points, color)


def render_rectified_overlay(prepared, line_families, rectified_points, color):
    result_like = build_result_like(prepared)
    image = draw_straight_line_families_on_rectified(result_like, line_families, color)
    for point_index, point in enumerate(rectified_points, start=1):
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


def render_failure_image(shape, title, message):
    height, width = shape[:2]
    image = np.full((height, width, 3), (245, 247, 250), dtype=np.uint8)
    cv2.rectangle(image, (16, 16), (width - 16, height - 16), (210, 214, 222), 1)
    cv2.putText(image, title[:36], (28, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (30, 41, 59), 2, cv2.LINE_AA)
    y = 92
    for chunk in [message[index:index + 44] for index in range(0, len(message), 44)]:
        cv2.putText(image, chunk, (28, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (100, 116, 139), 1, cv2.LINE_AA)
        y += 26
    return image


def time_variant(frame, prepared, variant, locked_response_name, repeat_count):
    result_like = build_result_like(prepared)
    timings = []
    payload = None
    for _ in range(repeat_count):
        started = time.perf_counter()
        family_result, rectified_intersections, _runtime_result_like = run_line_family_variant(
            prepared,
            variant,
            response_name_filter=locked_response_name,
        )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        points = points_from_rectified_intersections(frame, result_like, rectified_intersections)
        timings.append(elapsed_ms)
        payload = {
            "family_result": family_result,
            "rectified_intersections": rectified_intersections,
            "points": points,
        }
    return timings, payload


def verdict_for_result(item):
    if item.get("failure"):
        return "失败"
    drift = item["drift_vs_full"]
    if item["id"] == "full":
        return "基准"
    if drift["mean_px"] == 0.0 and drift["point_count_delta"] == 0:
        return "可跳过"
    if abs(drift["point_count_delta"]) > 0 or (drift["max_px"] is not None and drift["max_px"] > 8.0):
        return "不能删"
    return "需复核"


def write_report(output_dir, frame, prepared, results, metadata):
    output_dir = Path(output_dir)
    image_dir = output_dir / "images"
    image_dir.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(image_dir / "00_ir_input.png"), to_bgr(frame["ir"]))
    cv2.imwrite(str(image_dir / "00_rectified_ir.png"), to_bgr(prepared["rectified_ir"]))
    baseline_response = None
    for result in results:
        if result["id"] == "full" and result.get("response") is not None:
            baseline_response = result["response"]
            break
    if baseline_response is not None:
        cv2.imwrite(str(image_dir / "00_response.png"), render_response_heatmap(baseline_response))

    for result in results:
        color = COLORS.get(result.get("tone", "warn"), (0, 255, 0))
        if result.get("failure"):
            original = render_failure_image(to_bgr(frame["ir"]).shape, result["label"], result["failure"])
            rectified = render_failure_image(to_bgr(prepared["rectified_ir"]).shape, result["label"], result["failure"])
        else:
            original = render_original_overlay(
                frame,
                prepared,
                result["line_families"],
                result["points"],
                color,
            )
            rectified = render_rectified_overlay(
                prepared,
                result["line_families"],
                result["rectified_intersections"],
                color,
            )
        original_name = f"{result['id']}_original.png"
        rectified_name = f"{result['id']}_rectified.png"
        cv2.imwrite(str(image_dir / original_name), original)
        cv2.imwrite(str(image_dir / rectified_name), rectified)
        result["original_image_path"] = f"images/{original_name}"
        result["rectified_image_path"] = f"images/{rectified_name}"

    compact_results = []
    for result in results:
        compact_results.append(
            {
                "id": result["id"],
                "label": result["label"],
                "stage": result["stage"],
                "description": result["description"],
                "conclusion": result["conclusion"],
                "tone": result["tone"],
                "mean_ms": result.get("mean_ms"),
                "median_ms": result.get("median_ms"),
                "min_ms": result.get("min_ms"),
                "max_ms": result.get("max_ms"),
                "point_count": result.get("point_count"),
                "drift_vs_full": result.get("drift_vs_full"),
                "families": result.get("families", []),
                "first_20_points": result.get("points", [])[:20],
                "failure": result.get("failure"),
                "verdict": result.get("verdict"),
            }
        )

    (output_dir / "summary.json").write_text(
        json.dumps(
            safe_json(
                {
                    "metadata": metadata,
                    "results": compact_results,
                }
            ),
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    rows = []
    cards = []
    for result in results:
        drift = result["drift_vs_full"]
        mean_drift = "-" if drift["mean_px"] is None else f"{drift['mean_px']:.2f}"
        max_drift = "-" if drift["max_px"] is None else f"{drift['max_px']:.2f}"
        mean_ms = "-" if result.get("mean_ms") is None else f"{result['mean_ms']:.2f}"
        median_ms = "-" if result.get("median_ms") is None else f"{result['median_ms']:.2f}"
        verdict = result["verdict"]
        rows.append(
            f"""
            <tr class="{html.escape(result.get('tone', ''))}">
              <td>{html.escape(result['stage'])}</td>
              <td>{html.escape(result['label'])}</td>
              <td>{mean_ms}</td>
              <td>{median_ms}</td>
              <td>{result['point_count']}</td>
              <td>{mean_drift}</td>
              <td>{max_drift}</td>
              <td>{drift['point_count_delta']}</td>
              <td><span class="pill {html.escape(result.get('tone', 'warn'))}">{html.escape(verdict)}</span></td>
            </tr>
            """
        )
        family_text = html.escape(json.dumps(safe_json(result.get("families", [])), ensure_ascii=False))
        failure_html = (
            f"<p class=\"failure\">{html.escape(result['failure'])}</p>"
            if result.get("failure")
            else ""
        )
        cards.append(
            f"""
            <section class="variant-card {html.escape(result.get('tone', ''))}">
              <div class="card-head">
                <div>
                  <p class="stage">{html.escape(result['stage'])}</p>
                  <h2>{html.escape(result['label'])}</h2>
                </div>
                <span class="pill {html.escape(result.get('tone', 'warn'))}">{html.escape(verdict)}</span>
              </div>
              <p>{html.escape(result['description'])}</p>
              <p class="conclusion">{html.escape(result['conclusion'])}</p>
              {failure_html}
              <div class="metrics">
                <span>mean {mean_ms} ms</span>
                <span>median {median_ms} ms</span>
                <span>{result['point_count']} points</span>
                <span>drift {mean_drift} / {max_drift} px</span>
              </div>
              <div class="images">
                <figure>
                  <img src="{html.escape(result['original_image_path'])}" alt="{html.escape(result['label'])} original overlay">
                  <figcaption>原图识别效果</figcaption>
                </figure>
                <figure>
                  <img src="{html.escape(result['rectified_image_path'])}" alt="{html.escape(result['label'])} rectified overlay">
                  <figcaption>透视展开识别效果</figcaption>
                </figure>
              </div>
              <details>
                <summary>线族明细</summary>
                <pre>{family_text}</pre>
              </details>
            </section>
            """
        )

    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PR-FPRG 工序消融报告</title>
  <style>
    :root {{
      --bg: #eef2f6;
      --panel: #ffffff;
      --ink: #17202a;
      --muted: #5d6b7a;
      --line: #d8e0e8;
      --green: #15803d;
      --amber: #b45309;
      --red: #b91c1c;
      --blue: #0f5f9f;
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
      background: #15202b;
      color: #f8fafc;
      border-bottom: 4px solid #26a269;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 26px;
      letter-spacing: 0;
    }}
    header p {{
      margin: 0;
      color: #cbd5e1;
      line-height: 1.5;
    }}
    main {{
      padding: 18px 28px 36px;
      display: grid;
      gap: 18px;
    }}
    .summary-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(190px, 1fr));
      gap: 12px;
    }}
    .stat, .table-panel, .variant-card {{
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
      font-size: 23px;
      letter-spacing: 0;
    }}
    .table-panel {{
      padding: 14px;
      overflow-x: auto;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      min-width: 920px;
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
    .variant-card {{
      padding: 16px;
    }}
    .card-head {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: flex-start;
    }}
    .stage {{
      margin: 0 0 4px;
      color: var(--muted);
      font-size: 13px;
      font-weight: 700;
    }}
    h2 {{
      margin: 0;
      font-size: 18px;
      letter-spacing: 0;
    }}
    .variant-card p {{
      margin: 9px 0 0;
      line-height: 1.55;
      color: var(--muted);
      font-size: 14px;
    }}
    .conclusion {{
      color: var(--ink) !important;
      font-weight: 700;
    }}
    .failure {{
      color: var(--red) !important;
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
    }}
  </style>
</head>
<body>
  <header>
    <h1>PR-FPRG 工序消融报告</h1>
    <p>同一帧输入，锁定同一响应图，逐个关闭工序后对比识别效果、速度和相对全流程点位漂移。</p>
  </header>
  <main>
    <section class="summary-grid">
      <div class="stat"><span>生成时间</span><strong>{html.escape(metadata['generated_at'])}</strong></div>
      <div class="stat"><span>帧源</span><strong>{html.escape(str(metadata['frame_source']))}</strong></div>
      <div class="stat"><span>重复次数</span><strong>{metadata['repeat']}</strong></div>
      <div class="stat"><span>锁定响应图</span><strong>{html.escape(str(metadata['locked_response_name']))}</strong></div>
    </section>
    <section class="table-panel">
      <table>
        <thead>
          <tr>
            <th>工序</th>
            <th>消融项</th>
            <th>均值 ms</th>
            <th>中位 ms</th>
            <th>点数</th>
            <th>平均漂移 px</th>
            <th>最大漂移 px</th>
            <th>点数变化</th>
            <th>结论</th>
          </tr>
        </thead>
        <tbody>{''.join(rows)}</tbody>
      </table>
    </section>
    {''.join(cards)}
  </main>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")
    return output_dir / "index.html"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument(
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"pr_fprg_stage_ablation_report_{time.strftime('%Y%m%d_%H%M%S')}"),
    )
    args = parser.parse_args()

    rospy.init_node("pr_fprg_stage_ablation_report", anonymous=True, disable_signals=True)
    try:
        frame = capture_synced_frame(args.timeout)
    except RuntimeError as exc:
        rospy.logwarn("PointAI_log: raw_world同步帧不可用，改用 depth+IR 兜底做消融报告：%s", exc)
        frame = capture_depth_ir_frame(args.timeout)

    prepared = prepare_frame_inputs(frame)
    result_like = build_result_like(prepared)
    repeat_count = max(1, int(args.repeat))

    baseline_timings, baseline_payload = time_variant(frame, prepared, VARIANTS[0], None, repeat_count)
    locked_response_name = baseline_payload["family_result"]["response_name"]
    baseline_points = baseline_payload["points"]

    results = []
    for variant in VARIANTS:
        failure = None
        timings = []
        payload = None
        try:
            timings, payload = time_variant(frame, prepared, variant, locked_response_name, repeat_count)
        except Exception as exc:
            failure = str(exc)

        if failure is not None or payload is None:
            result = {
                "id": variant["id"],
                "label": variant["label"],
                "stage": variant["stage"],
                "description": variant["description"],
                "conclusion": variant["conclusion"],
                "tone": variant["tone"],
                "mean_ms": None,
                "median_ms": None,
                "min_ms": None,
                "max_ms": None,
                "point_count": 0,
                "drift_vs_full": {
                    "matched": 0,
                    "mean_px": None,
                    "max_px": None,
                    "point_count_delta": -len(baseline_points),
                },
                "families": [],
                "points": [],
                "rectified_intersections": [],
                "line_families": [],
                "response": None,
                "failure": failure,
            }
        else:
            drift = point_drift_stats(baseline_points, payload["points"])
            result = {
                "id": variant["id"],
                "label": variant["label"],
                "stage": variant["stage"],
                "description": variant["description"],
                "conclusion": variant["conclusion"],
                "tone": variant["tone"],
                "mean_ms": float(statistics.mean(timings)),
                "median_ms": float(statistics.median(timings)),
                "min_ms": float(min(timings)),
                "max_ms": float(max(timings)),
                "point_count": len(payload["points"]),
                "drift_vs_full": drift,
                "families": summarize_families(payload["family_result"]["line_families"]),
                "points": payload["points"],
                "rectified_intersections": payload["rectified_intersections"],
                "line_families": payload["family_result"]["line_families"],
                "response": payload["family_result"]["response"],
                "failure": None,
            }
        result["verdict"] = verdict_for_result(result)
        results.append(result)

    metadata = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source", "raw_world"),
        "repeat": repeat_count,
        "locked_response_name": locked_response_name,
    }
    report_path = write_report(args.output_dir, frame, prepared, results, metadata)
    print(str(report_path))


if __name__ == "__main__":
    main()
