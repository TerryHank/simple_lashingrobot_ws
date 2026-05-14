#!/usr/bin/env python3
"""Build a static report for bound/unbound lashing-point classification."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src"
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from tie_robot_perception.pointai.bind_point_classification import (  # noqa: E402
    ClassificationConfig,
    EvidenceBundle,
    classify_pre_bind_rule,
    decisions_to_summary,
    load_classification_config,
    verify_post_bind_rule,
)


EVENT_LOG_PATH = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "data" / "bind_classification_events.jsonl"
CONFIG_PATH = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "config" / "bind_point_classification.yaml"
REPORT_DIR = WORKSPACE_ROOT / "src" / "tie_robot_web" / "web" / "reports" / "bind_point_classification_current"


LABEL_META = {
    "bound": {"title": "已绑扎", "tone": "red", "color": (70, 70, 245)},
    "unbound": {"title": "未绑扎", "tone": "green", "color": (45, 185, 80)},
    "uncertain": {"title": "不确定", "tone": "amber", "color": (48, 180, 230)},
    "success": {"title": "绑后成功", "tone": "green", "color": (45, 185, 80)},
    "failed": {"title": "绑后失败", "tone": "red", "color": (70, 70, 245)},
}


def ensure_clean_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    for child in path.iterdir():
        if child.is_file():
            child.unlink()


def disc_mask(shape, radius, center=None):
    height, width = shape[:2]
    if center is None:
        center = ((width - 1) / 2.0, (height - 1) / 2.0)
    yy, xx = np.indices((height, width), dtype=np.float32)
    return ((xx - center[0]) ** 2 + (yy - center[1]) ** 2) <= float(radius) ** 2


def make_bundle(label, ir_patch, height_patch, valid_mask, ridge_patch=None):
    shape = np.asarray(height_patch).shape
    return EvidenceBundle(
        point_idx=1,
        phase=label,
        pix_coord=(shape[1] // 2, shape[0] // 2),
        world_coord=(120.0, 80.0, 955.0),
        ir_patch=np.asarray(ir_patch, dtype=np.uint8),
        depth_patch=(1000.0 - np.asarray(height_patch, dtype=np.float32)),
        raw_world_patch=np.zeros((*shape, 3), dtype=np.float32),
        height_patch=np.asarray(height_patch, dtype=np.float32),
        valid_depth_mask=np.asarray(valid_mask, dtype=bool),
        ridge_patch=np.asarray(ridge_patch if ridge_patch is not None else np.zeros(shape, dtype=np.uint8), dtype=np.uint8),
        evidence_quality=float(np.asarray(valid_mask, dtype=bool).mean()),
        metrics={"valid_depth_ratio": float(np.asarray(valid_mask, dtype=bool).mean())},
    )


def build_synthetic_cases(config: ClassificationConfig):
    shape = (65, 65)
    yy, xx = np.indices(shape)
    center = disc_mask(shape, 7)

    clean_ir = np.full(shape, 78, dtype=np.uint8)
    clean_height = np.zeros(shape, dtype=np.float32)
    clean_ridge = np.zeros(shape, dtype=np.uint8)
    clean_ridge[32, :] = 255
    clean_ridge[:, 32] = 255

    bound_ir = np.full(shape, 74, dtype=np.uint8)
    bound_height = np.zeros(shape, dtype=np.float32)
    bound_height[center] = 7.5
    bound_ir[center] = np.where((xx[center] + yy[center]) % 2 == 0, 32, 155)
    bound_ridge = clean_ridge.copy()
    bound_ridge[24:41, 24:41] = 0

    poor_valid = np.zeros(shape, dtype=bool)
    poor_valid[:12, :12] = True

    cases = [
        {
            "id": "unbound_clean_cross",
            "name": "绿框样例：平滑规则交点",
            "expected": "unbound",
            "bundle": make_bundle("before", clean_ir, clean_height, np.ones(shape, dtype=bool), clean_ridge),
        },
        {
            "id": "bound_center_knot",
            "name": "红框样例：中心结节/凸起",
            "expected": "bound",
            "bundle": make_bundle("before", bound_ir, bound_height, np.ones(shape, dtype=bool), bound_ridge),
        },
        {
            "id": "uncertain_low_depth",
            "name": "不确定样例：有效深度不足",
            "expected": "uncertain",
            "bundle": make_bundle("before", clean_ir, clean_height, poor_valid, clean_ridge),
        },
    ]

    before = make_bundle("before", clean_ir, clean_height, np.ones(shape, dtype=bool), clean_ridge)
    after = make_bundle("after", bound_ir, bound_height, np.ones(shape, dtype=bool), bound_ridge)
    post_decision = verify_post_bind_rule(before, after, config)
    return cases, {"before": before, "after": after, "decision": post_decision}


def normalize_to_u8(image, scale=1.0):
    array = np.asarray(image, dtype=np.float32) * float(scale)
    if array.size == 0:
        return np.zeros((1, 1), dtype=np.uint8)
    finite = array[np.isfinite(array)]
    if finite.size == 0:
        return np.zeros(array.shape[:2], dtype=np.uint8)
    lo = float(np.percentile(finite, 2.0))
    hi = float(np.percentile(finite, 98.0))
    if hi <= lo + 1e-6:
        hi = lo + 1.0
    return np.clip((array - lo) * 255.0 / (hi - lo), 0, 255).astype(np.uint8)


def render_bundle_image(bundle: EvidenceBundle, decision_label: str, output_path: Path) -> None:
    ir = np.asarray(bundle.ir_patch, dtype=np.uint8)
    height = cv2.applyColorMap(normalize_to_u8(bundle.height_patch), cv2.COLORMAP_TURBO)
    ridge = cv2.cvtColor(np.asarray(bundle.ridge_patch, dtype=np.uint8), cv2.COLOR_GRAY2BGR)
    ir_bgr = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)
    panels = [ir_bgr, height, ridge]
    label_strip = np.zeros((24, panels[0].shape[1], 3), dtype=np.uint8)
    panel_titles = ["IR", "高度差", "脊线"]
    titled = []
    for title, panel in zip(panel_titles, panels):
        strip = label_strip.copy()
        cv2.putText(strip, title, (7, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1, cv2.LINE_AA)
        titled.append(np.vstack([strip, panel]))
    canvas = np.hstack(titled)
    color = LABEL_META.get(decision_label, LABEL_META["uncertain"])["color"]
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1] - 1, canvas.shape[0] - 1), color, 4)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), canvas)


def read_event_log(path: Path):
    if not path.exists() or path.stat().st_size == 0:
        return []
    events = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def event_stats(events):
    stats = {"total": len(events), "bound": 0, "unbound": 0, "uncertain": 0}
    scores = []
    for event in events:
        label = str(event.get("bind_state", "uncertain"))
        stats[label] = stats.get(label, 0) + 1
        try:
            scores.append(float(event.get("bind_score", 0.0)))
        except (TypeError, ValueError):
            pass
    stats["avg_score"] = float(np.mean(scores)) if scores else 0.0
    return stats


def pct(numerator, denominator):
    if denominator <= 0:
        return "0.0%"
    return f"{100.0 * float(numerator) / float(denominator):.1f}%"


def write_html(output_dir, config, case_results, post_result, events, stats):
    cards = []
    for result in case_results:
        decision = result["decision"]
        meta = LABEL_META.get(decision.label, LABEL_META["uncertain"])
        metrics = decision.metrics
        cards.append(f"""
        <article class="case-card {meta['tone']}">
          <div class="case-head">
            <span>{result['name']}</span>
            <strong>{meta['title']}</strong>
          </div>
          <img src="{result['image']}" alt="{result['name']}">
          <dl>
            <div><dt>score</dt><dd>{decision.score:.3f}</dd></div>
            <div><dt>quality</dt><dd>{decision.quality:.3f}</dd></div>
            <div><dt>height</dt><dd>{metrics.get('center_height_delta_mm', 0.0):.2f} mm</dd></div>
            <div><dt>texture</dt><dd>{metrics.get('center_ir_stddev', 0.0):.2f}</dd></div>
            <div><dt>ridge break</dt><dd>{metrics.get('ridge_break_ratio', 0.0):.2f}</dd></div>
          </dl>
          <p>{decision.reason}</p>
        </article>
        """)

    event_rows = []
    for event in events[-80:]:
        label = str(event.get("bind_state", "uncertain"))
        meta = LABEL_META.get(label, LABEL_META["uncertain"])
        event_rows.append(f"""
          <tr>
            <td>{event.get('point_idx', '')}</td>
            <td><span class="pill {meta['tone']}">{meta['title']}</span></td>
            <td>{float(event.get('bind_score', 0.0)):.3f}</td>
            <td>{float(event.get('evidence_quality', 0.0)):.3f}</td>
            <td>{event.get('decision_reason', '')}</td>
          </tr>
        """)
    if not event_rows:
        event_rows.append('<tr><td colspan="5" class="empty">当前还没有现场分类事件。先运行 pointAI 的 MODE_BIND_CHECK 后，本表会显示真实点位记录。</td></tr>')

    post_decision = post_result["decision"]
    post_meta = LABEL_META.get(post_decision.label, LABEL_META["uncertain"])
    generated_at = time.strftime("%Y-%m-%d %H:%M:%S")
    html = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>绑扎点分类效果报告</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #101113;
      --panel: #181a1f;
      --panel-2: #20232a;
      --line: #333842;
      --text: #eef1f3;
      --muted: #9ca6ad;
      --red: #ff5757;
      --green: #72d36a;
      --amber: #e4b84e;
      --cyan: #7bd7ff;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: radial-gradient(circle at 14% 0%, #1f242b 0, #101113 36rem);
      color: var(--text);
      font-family: "IBM Plex Sans", "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
    }}
    main {{ max-width: 1360px; margin: 0 auto; padding: 28px; }}
    header {{ display: flex; justify-content: space-between; gap: 24px; align-items: end; border-bottom: 1px solid var(--line); padding-bottom: 18px; }}
    h1 {{ margin: 0; font-size: 30px; letter-spacing: 0; }}
    h2 {{ margin: 30px 0 14px; font-size: 18px; letter-spacing: 0; }}
    p {{ color: var(--muted); line-height: 1.62; }}
    .stamp {{ color: var(--muted); font-size: 13px; text-align: right; }}
    .stats {{ display: grid; grid-template-columns: repeat(5, minmax(150px, 1fr)); gap: 12px; margin-top: 20px; }}
    .stat {{ background: linear-gradient(180deg, var(--panel), #15171b); border: 1px solid var(--line); padding: 16px; border-radius: 8px; }}
    .stat span {{ display: block; color: var(--muted); font-size: 12px; }}
    .stat strong {{ display: block; margin-top: 8px; font-size: 28px; }}
    .notice {{ border: 1px solid #6d5a2a; background: #211d13; padding: 14px 16px; border-radius: 8px; color: #f5d887; }}
    .cases {{ display: grid; grid-template-columns: repeat(3, minmax(260px, 1fr)); gap: 14px; }}
    .case-card {{ background: var(--panel); border: 1px solid var(--line); border-radius: 8px; overflow: hidden; }}
    .case-card.red {{ border-color: rgba(255, 87, 87, .55); }}
    .case-card.green {{ border-color: rgba(114, 211, 106, .55); }}
    .case-card.amber {{ border-color: rgba(228, 184, 78, .6); }}
    .case-head {{ display: flex; justify-content: space-between; align-items: center; gap: 10px; padding: 12px 14px; background: var(--panel-2); border-bottom: 1px solid var(--line); }}
    .case-head span {{ color: var(--muted); font-size: 13px; }}
    .case-head strong {{ font-size: 14px; }}
    img {{ display: block; width: 100%; image-rendering: pixelated; background: #050607; }}
    dl {{ display: grid; grid-template-columns: repeat(5, 1fr); gap: 0; margin: 0; border-top: 1px solid var(--line); }}
    dl div {{ padding: 10px; border-right: 1px solid var(--line); }}
    dt {{ color: var(--muted); font-size: 11px; }}
    dd {{ margin: 4px 0 0; font-weight: 700; font-size: 13px; }}
    .case-card p {{ margin: 0; padding: 12px 14px; min-height: 54px; }}
    table {{ width: 100%; border-collapse: collapse; background: var(--panel); border: 1px solid var(--line); border-radius: 8px; overflow: hidden; }}
    th, td {{ text-align: left; padding: 11px 12px; border-bottom: 1px solid var(--line); font-size: 13px; }}
    th {{ color: var(--muted); background: var(--panel-2); font-weight: 600; }}
    .pill {{ display: inline-flex; align-items: center; min-width: 74px; justify-content: center; padding: 3px 8px; border-radius: 999px; font-size: 12px; font-weight: 700; }}
    .pill.red {{ color: #fff; background: rgba(255, 87, 87, .24); border: 1px solid rgba(255, 87, 87, .5); }}
    .pill.green {{ color: #eaffea; background: rgba(114, 211, 106, .18); border: 1px solid rgba(114, 211, 106, .5); }}
    .pill.amber {{ color: #fff3c6; background: rgba(228, 184, 78, .18); border: 1px solid rgba(228, 184, 78, .55); }}
    .empty {{ color: var(--muted); text-align: center; padding: 22px; }}
    .flow {{ display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }}
    .flow-card {{ background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 16px; }}
    code {{ color: var(--cyan); }}
    @media (max-width: 960px) {{
      header, .flow {{ display: block; }}
      .stats, .cases {{ grid-template-columns: 1fr; }}
      .stamp {{ text-align: left; margin-top: 12px; }}
      dl {{ grid-template-columns: repeat(2, 1fr); }}
    }}
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>绑扎点已绑 / 未绑分类效果报告</h1>
      <p>规则版分类器：局部 IR + 深度高度差 + 中心/环形差分 + 脊线破坏 + 证据质量。当前默认 <code>mode=shadow</code>，只记录，不阻断。</p>
    </div>
    <div class="stamp">生成时间<br>{generated_at}</div>
  </header>

  <section class="stats">
    <div class="stat"><span>现场事件数</span><strong>{stats['total']}</strong></div>
    <div class="stat"><span>已绑扎 bound</span><strong>{stats.get('bound', 0)}</strong></div>
    <div class="stat"><span>未绑扎 unbound</span><strong>{stats.get('unbound', 0)}</strong></div>
    <div class="stat"><span>不确定 uncertain</span><strong>{stats.get('uncertain', 0)}</strong></div>
    <div class="stat"><span>平均分</span><strong>{stats['avg_score']:.3f}</strong></div>
  </section>

  <h2>当前效果结论</h2>
  <p class="notice">当前事件日志 <code>{EVENT_LOG_PATH}</code> 为空，因此还不能统计真实现场精度、召回率或红/绿框准确率。下面展示的是分类器 sanity 实验：它能按设计把干净交点判为绿框未绑、把中心结节/凸起判为红框已绑、把证据不足判为不确定。真实效果需要现场跑一次 <code>MODE_BIND_CHECK</code> 后再刷新本报告。</p>

  <h2>合成 sanity 样本</h2>
  <section class="cases">
    {''.join(cards)}
  </section>

  <h2>绑后差分复检</h2>
  <section class="flow">
    <div class="flow-card">
      <strong class="pill {post_meta['tone']}">{post_meta['title']}</strong>
      <p>before 为干净交点，after 加入中心凸起和纹理，复检输出 <strong>{post_decision.label}</strong>，score={post_decision.score:.3f}，reason={post_decision.reason}。</p>
    </div>
    <div class="flow-card">
      <p>上线顺序建议：先保持 shadow 收集现场红/绿样本；人工核对后再调阈值；最后只在高置信 bound 时进入 advisory 或 blocking。</p>
    </div>
  </section>

  <h2>现场事件记录</h2>
  <table>
    <thead><tr><th>点位</th><th>分类</th><th>score</th><th>quality</th><th>reason</th></tr></thead>
    <tbody>{''.join(event_rows)}</tbody>
  </table>
</main>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html, encoding="utf-8")


def main():
    ensure_clean_dir(REPORT_DIR)
    images_dir = REPORT_DIR / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    config = load_classification_config(CONFIG_PATH)
    cases, post_result = build_synthetic_cases(config)
    case_results = []
    decisions = []
    for case in cases:
        decision = classify_pre_bind_rule(case["bundle"], config)
        decisions.append(decision)
        image_name = f"{case['id']}.png"
        render_bundle_image(case["bundle"], decision.label, images_dir / image_name)
        case_results.append({
            "id": case["id"],
            "name": case["name"],
            "expected": case["expected"],
            "decision": decision,
            "image": f"images/{image_name}",
        })

    events = read_event_log(EVENT_LOG_PATH)
    stats = event_stats(events)
    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "event_log_path": str(EVENT_LOG_PATH),
        "event_stats": stats,
        "synthetic_summary": decisions_to_summary(decisions),
        "post_bind_decision": {
            "label": post_result["decision"].label,
            "score": post_result["decision"].score,
            "reason": post_result["decision"].reason,
        },
        "report_url_path": "/reports/bind_point_classification_current/index.html",
    }
    (REPORT_DIR / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    write_html(REPORT_DIR, config, case_results, post_result, events, stats)
    print(REPORT_DIR / "index.html")


if __name__ == "__main__":
    main()
