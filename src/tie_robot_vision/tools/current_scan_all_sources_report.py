#!/usr/bin/env python3

"""Generate a live HTML report for Surface-DP response sources.

This tool is report-only. It captures the current Scepter frame, builds the
same rectified manual workspace input used by the runtime Surface-DP path, runs
the selected single-source scan chain once, then builds the other response
sources offline as hidden comparison material.
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
import rospy


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = WORKSPACE_ROOT / "src" / "tie_robot_vision" / "src"
TOOL_DIR = Path(__file__).resolve().parent
for import_path in (PERCEPTION_SRC, TOOL_DIR):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from pr_fprg_peak_supported_probe import (  # noqa: E402
    capture_depth_ir_frame,
    capture_synced_frame,
    normalize_probe_raw_world,
    to_u8,
)
from tie_robot_vision.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_projective_line_segments,
    build_workspace_s2_rectified_geometry,
    sort_polygon_points_clockwise,
)
from tie_robot_vision.pointai import scan_surface_dp  # noqa: E402


BASE_SOURCE_ORDER = [
    "depth_gradient",
    "frangi_like",
    "hessian_ridge",
    "infrared_response",
    "combined_response",
    "depth_response",
    "fused_instance_response",
]
COMPLETED_SOURCE_ORDER = [
    "depth_gradient",
    "completed_surface_response",
    "frangi_like",
    "hessian_ridge",
    "infrared_response",
    "fused_instance_response",
    "combined_response",
    "depth_response",
]
SOURCE_LABELS = {
    "depth_response": "深度暗线响应",
    "infrared_response": "红外暗线响应",
    "combined_response": "深度 + 红外组合响应",
    "depth_gradient": "深度梯度边缘",
    "hessian_ridge": "Hessian ridge 脊线",
    "frangi_like": "多尺度 Frangi-like 脊线",
    "fused_instance_response": "融合实例响应",
    "completed_surface_response": "补全钢筋面响应",
}
SOURCE_NOTES = {
    "depth_response": "高于局部背景的深度暗线/凸起更亮，直接来自深度高度变化。",
    "infrared_response": "红外强度相对背景变暗的结构更亮，受反光和材质影响较大。",
    "combined_response": "隐藏离线对照源：把深度和红外归一化后组合。",
    "depth_gradient": "默认运行响应源：关注深度变化边缘，适合看钢筋边界和高度突变。",
    "hessian_ridge": "隐藏离线对照源：在组合响应上提二阶亮脊，偏向连续细线。",
    "frangi_like": "隐藏离线对照源：多尺度 ridge 提取，偏向不同宽度的连续线状结构。",
    "fused_instance_response": "隐藏离线对照源：旧融合图，包含组合、深度、Frangi、深度梯度、Hessian。",
    "completed_surface_response": "隐藏离线对照源：旧补全面，把旧融合响应和物理线族支撑叠加后观察。",
}


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


def load_manual_workspace():
    path = WORKSPACE_ROOT / "src" / "tie_robot_vision" / "data" / "manual_workspace_quad.json"
    return json.loads(path.read_text(encoding="utf-8"))


def capture_current_frame(timeout_sec, allow_depth_ir_fallback=True):
    try:
        return capture_synced_frame(timeout_sec)
    except RuntimeError as exc:
        rospy.logwarn("current all-source report: synced worldCoord frame unavailable: %s", exc)
    if not allow_depth_ir_fallback:
        raise RuntimeError("synced worldCoord frame unavailable and depth/IR fallback disabled")
    return capture_depth_ir_frame(timeout_sec)


def build_workspace_mask(shape, corner_pixels):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    polygon = np.asarray(sort_polygon_points_clockwise(corner_pixels), dtype=np.int32).reshape((-1, 1, 2))
    cv2.fillPoly(mask, [polygon], 1)
    return mask.astype(bool)


def build_rectified_runtime_input(frame):
    manual_workspace = load_manual_workspace()
    corner_pixels = manual_workspace["corner_pixels"]
    corner_world = manual_workspace.get("corner_world_map_frame")
    if corner_world is None:
        corner_world = manual_workspace.get("corner_world_" + "cabin" + "_frame")
    if corner_world is None:
        corner_world = manual_workspace.get("corner_world_camera_frame")
    if corner_world is None:
        raise RuntimeError("manual workspace is missing corner world coordinates")

    raw_world, used_depth_fallback_raw_world = normalize_probe_raw_world(frame["raw"])
    ir = to_u8(frame["ir"])
    geometry = build_workspace_s2_rectified_geometry(corner_pixels, corner_world)
    if geometry is None:
        raise RuntimeError("failed to build rectified manual workspace geometry")

    rectified_size = (int(geometry["rectified_width"]), int(geometry["rectified_height"]))
    raw_depth = raw_world[:, :, 2].astype(np.float32)
    valid_mask = np.isfinite(raw_depth) & (raw_depth != 0.0)
    rectified_depth = cv2.warpPerspective(
        raw_depth,
        np.asarray(geometry["forward_h"], dtype=np.float32),
        rectified_size,
        flags=cv2.INTER_LINEAR,
    ).astype(np.float32)
    rectified_valid = cv2.warpPerspective(
        valid_mask.astype(np.uint8),
        np.asarray(geometry["forward_h"], dtype=np.float32),
        rectified_size,
        flags=cv2.INTER_NEAREST,
    ).astype(bool)
    rectified_ir = cv2.warpPerspective(
        ir.astype(np.float32),
        np.asarray(geometry["forward_h"], dtype=np.float32),
        rectified_size,
        flags=cv2.INTER_LINEAR,
    ).astype(np.float32)
    if not np.any(rectified_valid):
        raise RuntimeError("rectified workspace has no valid depth pixels")
    median_depth = float(np.median(rectified_depth[rectified_valid]))
    filled_depth = np.where(rectified_valid, rectified_depth, median_depth).astype(np.float32)
    background_depth = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=11.0, sigmaY=11.0)
    selected_response = scan_surface_dp._normalize_response(background_depth - filled_depth, rectified_valid)
    return {
        "manual_workspace": manual_workspace,
        "workspace_mask": build_workspace_mask(ir.shape[:2], corner_pixels),
        "raw_world": raw_world,
        "ir": ir,
        "rectified_depth": rectified_depth,
        "filled_depth": filled_depth,
        "rectified_ir": rectified_ir,
        "rectified_valid": rectified_valid,
        "response": selected_response,
        "response_source": "manual_workspace_s2_depth_selected",
        "rectified_geometry": geometry,
        "used_depth_fallback_raw_world": bool(used_depth_fallback_raw_world),
    }


def normalize01(image, valid_mask=None, low=2.0, high=98.0):
    image = np.asarray(image, dtype=np.float32)
    valid = np.isfinite(image)
    if valid_mask is not None:
        valid &= np.asarray(valid_mask, dtype=bool)
    if not np.any(valid):
        return np.zeros(image.shape[:2], dtype=np.float32)
    values = image[valid]
    lower = float(np.percentile(values, low))
    upper = float(np.percentile(values, high))
    if upper <= lower + 1e-6:
        lower = float(np.min(values))
        upper = float(np.max(values))
    if upper <= lower + 1e-6:
        output = np.zeros_like(image, dtype=np.float32)
        output[valid] = 1.0
        return output
    output = np.clip((image - lower) / (upper - lower), 0.0, 1.0).astype(np.float32)
    output[~valid] = 0.0
    return output


def render_heatmap(response, valid_mask=None):
    image_u8 = (normalize01(response, valid_mask) * 255.0).astype(np.uint8)
    return cv2.applyColorMap(image_u8, cv2.COLORMAP_TURBO)


def render_ir_workspace(runtime_input):
    image = cv2.cvtColor(runtime_input["ir"], cv2.COLOR_GRAY2BGR)
    polygon = np.asarray(
        sort_polygon_points_clockwise(runtime_input["manual_workspace"]["corner_pixels"]),
        dtype=np.int32,
    ).reshape((-1, 1, 2))
    fill = image.copy()
    cv2.fillPoly(fill, [polygon], (40, 155, 110))
    image = cv2.addWeighted(fill, 0.20, image, 0.80, 0.0)
    cv2.polylines(image, [polygon], True, (60, 255, 180), 2, cv2.LINE_AA)
    return image


def draw_label(image, label):
    output = image.copy()
    text = str(label)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    cv2.rectangle(output, (10, 10), (26 + text_width, 26 + text_height + baseline), (9, 13, 18), -1)
    cv2.putText(output, text, (18, 29), font, scale, (245, 248, 246), thickness, cv2.LINE_AA)
    return output


def draw_rectified_overlay(response_map, valid_mask, line_families, points, label):
    image = render_heatmap(response_map, valid_mask)
    vertical_lines, horizontal_lines = scan_surface_dp._axis_positions_from_families(line_families)
    for x_value in vertical_lines:
        x = int(round(float(x_value)))
        cv2.line(image, (x, 0), (x, image.shape[0] - 1), (35, 245, 150), 1, cv2.LINE_AA)
    for y_value in horizontal_lines:
        y = int(round(float(y_value)))
        cv2.line(image, (0, y), (image.shape[1] - 1, y), (30, 190, 255), 1, cv2.LINE_AA)
    for point in points[:600]:
        x = int(round(float(point[0])))
        y = int(round(float(point[1])))
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            cv2.circle(image, (x, y), 3, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(image, (x, y), 4, (20, 25, 25), 1, cv2.LINE_AA)
    return draw_label(image, label)


def draw_original_overlay(runtime_input, line_families, image_points, label):
    image = render_ir_workspace(runtime_input)
    geometry = runtime_input["rectified_geometry"]
    corner_pixels = runtime_input["manual_workspace"]["corner_pixels"]
    vertical_lines, horizontal_lines = scan_surface_dp._axis_positions_from_families(line_families)
    segments = build_workspace_s2_projective_line_segments(
        corner_pixels,
        int(geometry["rectified_width"]),
        int(geometry["rectified_height"]),
        vertical_lines,
        horizontal_lines,
    )
    for start, end in segments.get("vertical", []):
        cv2.line(image, tuple(start), tuple(end), (35, 245, 150), 1, cv2.LINE_AA)
    for start, end in segments.get("horizontal", []):
        cv2.line(image, tuple(start), tuple(end), (30, 190, 255), 1, cv2.LINE_AA)
    workspace_mask = runtime_input["workspace_mask"]
    for point in image_points[:600]:
        x = int(round(float(point[0])))
        y = int(round(float(point[1])))
        if 0 <= x < workspace_mask.shape[1] and 0 <= y < workspace_mask.shape[0] and workspace_mask[y, x]:
            cv2.circle(image, (x, y), 3, (0, 255, 255), -1, cv2.LINE_AA)
    return draw_label(image, label)


def point_support_mean(response_map, points):
    if not points:
        return 0.0
    return float(np.mean([scan_surface_dp._response_value_at(response_map, point) for point in points]))


def in_workspace_count(runtime_input, image_points):
    mask = runtime_input["workspace_mask"]
    count = 0
    for point in image_points:
        x = int(round(float(point[0])))
        y = int(round(float(point[1])))
        if 0 <= x < mask.shape[1] and 0 <= y < mask.shape[0] and mask[y, x]:
            count += 1
    return int(count)


def evaluate_source(runtime_input, source_name, response_map, peak_min_ratio):
    valid_mask = runtime_input["rectified_valid"]
    rectified_geometry = runtime_input["rectified_geometry"]
    attempts = []
    accepted_families = []
    accepted_ratio = None
    accepted_score = -float("inf")
    for ratio in (peak_min_ratio, max(0.12, peak_min_ratio * 0.78), 0.10):
        line_families = scan_surface_dp._build_physical_axis_aligned_line_families(
            response_map,
            valid_mask,
            rectified_geometry,
            peak_min_ratio=ratio,
        )
        score = scan_surface_dp._score_physical_line_families(line_families)
        counts = [len(family.get("line_rhos", [])) for family in line_families[:2]]
        modes = [str(family.get("physical_prior_mode", "")) for family in line_families[:2]]
        attempts.append(
            {
                "ratio": float(ratio),
                "line_counts": counts,
                "physical_modes": modes,
                "score": None if not math.isfinite(float(score)) else float(score),
                "accepted": bool(score > -float("inf")),
            }
        )
        if score > -float("inf"):
            accepted_families = line_families[:2]
            accepted_ratio = float(ratio)
            accepted_score = float(score)
            break

    rectified_points = []
    image_points = []
    if len(accepted_families) >= 2:
        rectified_points = scan_surface_dp.intersect_workspace_s2_oriented_line_families(
            accepted_families[0],
            accepted_families[1],
            int(rectified_geometry["rectified_width"]),
            int(rectified_geometry["rectified_height"]),
        )
        image_points = scan_surface_dp._project_rectified_points_to_image(
            rectified_points,
            rectified_geometry.get("inverse_h"),
        )
    line_counts = [len(family.get("line_rhos", [])) for family in accepted_families[:2]]
    return {
        "id": source_name,
        "label": SOURCE_LABELS.get(source_name, source_name),
        "note": SOURCE_NOTES.get(source_name, ""),
        "response_map": response_map,
        "line_families": accepted_families,
        "line_counts": line_counts,
        "accepted_ratio": accepted_ratio,
        "accepted_score": None if not math.isfinite(accepted_score) else accepted_score,
        "rectified_point_count": len(rectified_points),
        "image_workspace_point_count": in_workspace_count(runtime_input, image_points),
        "mean_response_at_points": point_support_mean(response_map, rectified_points),
        "attempts": attempts,
        "rectified_points": rectified_points,
        "image_points": image_points,
        "accepted": bool(len(accepted_families) >= 2),
    }


def build_legacy_completed_surface_response(runtime_input, response_maps, threshold_percentile):
    valid_mask = runtime_input["rectified_valid"]
    rectified_geometry = runtime_input["rectified_geometry"]
    fused_instance_response = response_maps["fused_instance_response"]
    binary_candidate, binary_threshold = scan_surface_dp.threshold_response(
        fused_instance_response,
        valid_mask,
        percentile=threshold_percentile,
    )
    line_families, physical_source = scan_surface_dp._build_best_physical_axis_aligned_line_families(
        [(source_name, response_maps.get(source_name)) for source_name in BASE_SOURCE_ORDER],
        valid_mask,
        rectified_geometry,
        peak_min_ratio=0.18,
    )
    if len(line_families) < 2:
        line_families = []
        physical_source = "hidden_offline_physical_prior_unresolved"
    if (
        line_families
        and scan_surface_dp._full_workspace_expected(valid_mask, rectified_geometry)
        and any(family.get("physical_prior_mode") != "full_workspace" for family in line_families[:2])
    ):
        line_families = []
        physical_source = "hidden_offline_physical_prior_unresolved"
    line_support_mask = scan_surface_dp.draw_line_family_mask(
        binary_candidate.shape,
        line_families,
        thickness_px=5,
    )
    completed_surface_mask = (binary_candidate | line_support_mask) & valid_mask
    completed_surface_response = scan_surface_dp._normalize_response(
        (0.70 * fused_instance_response)
        + (0.20 * line_support_mask.astype(np.float32))
        + (0.10 * completed_surface_mask.astype(np.float32)),
        valid_mask,
    )
    return {
        "completed_surface_response": completed_surface_response,
        "completed_surface_mask": completed_surface_mask,
        "line_support_mask": line_support_mask,
        "base_line_families": line_families[:2],
        "completed_line_families": line_families[:2],
        "base_physical_source": physical_source,
        "completed_physical_source": physical_source,
        "binary_threshold": float(binary_threshold),
        "response_policy": "hidden_offline_legacy_completed_surface",
    }


def build_offline_response_maps(runtime_input, threshold_percentile):
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
    response_maps = {
        "depth_response": depth_response,
        "infrared_response": infrared_response,
        "combined_response": combined_response,
        "depth_gradient": depth_gradient,
        "hessian_ridge": hessian_ridge,
        "frangi_like": frangi_like,
        "fused_instance_response": fused_instance_response,
    }
    legacy_surface = build_legacy_completed_surface_response(
        runtime_input,
        response_maps,
        threshold_percentile,
    )
    response_maps["completed_surface_response"] = legacy_surface["completed_surface_response"]
    return response_maps, legacy_surface


def build_evaluation(runtime_input, threshold_percentile, response_source="depth_gradient"):
    selected_source = scan_surface_dp.normalize_scan_response_source(response_source)
    runtime_result = scan_surface_dp.build_scan_surface_dp_result(
        runtime_input,
        threshold_percentile=threshold_percentile,
        response_source=selected_source,
    )
    response_maps, legacy_surface = build_offline_response_maps(runtime_input, threshold_percentile)
    base_rows = [
        evaluate_source(runtime_input, source_name, response_maps[source_name], peak_min_ratio=0.18)
        for source_name in BASE_SOURCE_ORDER
    ]
    completed_rows = [
        evaluate_source(runtime_input, source_name, response_maps[source_name], peak_min_ratio=0.16)
        for source_name in COMPLETED_SOURCE_ORDER
    ]
    return {
        "hidden_offline_response_maps": response_maps,
        "legacy_surface": legacy_surface,
        "response_maps": response_maps,
        "base_rows": base_rows,
        "completed_rows": completed_rows,
        "runtime_result": runtime_result,
        "selected_runtime_response_source": selected_source,
    }


def safe_filename(name):
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in str(name))


def write_report(output_dir, frame, runtime_input, evaluation, timings_ms):
    output_dir = Path(output_dir)
    images_dir = output_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    valid_mask = runtime_input["rectified_valid"]
    runtime_result = evaluation["runtime_result"]
    selected_runtime_source = evaluation.get("selected_runtime_response_source", "depth_gradient")

    def write_image(filename, image):
        path = images_dir / filename
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f"failed to write image: {path}")
        return f"images/{filename}"

    image_paths = {
        "input_workspace": write_image("00_input_workspace.png", render_ir_workspace(runtime_input)),
        "rectified_ir": write_image(
            "01_rectified_ir.png",
            draw_label(cv2.cvtColor(to_u8(runtime_input["rectified_ir"]), cv2.COLOR_GRAY2BGR), "rectified IR"),
        ),
        "rectified_depth": write_image(
            "02_rectified_depth.png",
            draw_label(render_heatmap(runtime_input["filled_depth"], valid_mask), "filled depth"),
        ),
    }

    all_rows = []
    for pass_name, rows in (("base", evaluation["base_rows"]), ("completed", evaluation["completed_rows"])):
        for row in rows:
            prefix = f"{pass_name}_{safe_filename(row['id'])}"
            label = f"{pass_name} {row['id']} lines={row['line_counts']} pts={row['rectified_point_count']}"
            image_paths[f"{prefix}_rectified"] = write_image(
                f"{prefix}_rectified.png",
                draw_rectified_overlay(
                    row["response_map"],
                    valid_mask,
                    row["line_families"],
                    row["rectified_points"],
                    label,
                ),
            )
            image_paths[f"{prefix}_original"] = write_image(
                f"{prefix}_original.png",
                draw_original_overlay(runtime_input, row["line_families"], row["image_points"], label),
            )
            all_rows.append({key: value for key, value in row.items() if key not in {"response_map", "line_families", "rectified_points", "image_points"}})

    runtime_line_families = runtime_result.get("line_families", []) if runtime_result.get("success") else []
    runtime_points = runtime_result.get("rectified_intersections", []) if runtime_result.get("success") else []
    runtime_image_points = runtime_result.get("image_intersections", []) if runtime_result.get("success") else []
    image_paths["runtime_rectified"] = write_image(
        "90_runtime_selected_rectified.png",
        draw_rectified_overlay(
            runtime_result.get("completed_surface_response", evaluation["response_maps"]["depth_gradient"]),
            valid_mask,
            runtime_line_families,
            runtime_points,
            f"runtime {selected_runtime_source} policy={runtime_result.get('diagnostics', {}).get('scan_runtime_response_policy')}",
        ),
    )
    image_paths["runtime_original"] = write_image(
        "91_runtime_selected_original.png",
        draw_original_overlay(runtime_input, runtime_line_families, runtime_image_points, f"runtime {selected_runtime_source} output"),
    )

    def row_to_summary(row):
        return {
            "id": row["id"],
            "label": row["label"],
            "line_counts": row["line_counts"],
            "rectified_point_count": row["rectified_point_count"],
            "image_workspace_point_count": row["image_workspace_point_count"],
            "accepted_ratio": row["accepted_ratio"],
            "accepted_score": row["accepted_score"],
            "mean_response_at_points": row["mean_response_at_points"],
            "accepted": row["accepted"],
            "attempts": row["attempts"],
            "note": row["note"],
        }

    summary = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source"),
        "used_depth_fallback_raw_world": bool(runtime_input.get("used_depth_fallback_raw_world", False)),
        "rectified_size": [
            int(runtime_input["rectified_geometry"]["rectified_width"]),
            int(runtime_input["rectified_geometry"]["rectified_height"]),
        ],
        "base_order": BASE_SOURCE_ORDER,
        "completed_order": COMPLETED_SOURCE_ORDER,
        "runtime_policy": runtime_result.get("diagnostics", {}).get("scan_runtime_response_policy"),
        "runtime_response_source": runtime_result.get("diagnostics", {}).get("scan_runtime_response_source"),
        "hidden_offline_response_maps": sorted(evaluation["hidden_offline_response_maps"].keys()),
        "base_rows": [row_to_summary(row) for row in evaluation["base_rows"]],
        "completed_rows": [row_to_summary(row) for row in evaluation["completed_rows"]],
        "runtime_result": {
            "success": bool(runtime_result.get("success", False)),
            "message": runtime_result.get("message"),
            "line_counts": runtime_result.get("line_counts", []),
            "point_count": len(runtime_result.get("rectified_intersections", []) or []),
            "base_physical_source": runtime_result.get("diagnostics", {}).get("base_physical_source"),
            "completed_physical_source": runtime_result.get("diagnostics", {}).get("completed_physical_source"),
            "diagnostics": runtime_result.get("diagnostics", {}),
        },
        "timings_ms": timings_ms,
        "images": image_paths,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(json_safe(summary), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    def render_table(rows, pass_name):
        table_rows = []
        for row in rows:
            attempts = " / ".join(
                (
                    f"{attempt['ratio']:.2f}:"
                    f"{attempt['line_counts'] or '-'}:"
                    f"{'过' if attempt['accepted'] else '拒'}"
                )
                for attempt in row["attempts"]
            )
            prefix = f"{pass_name}_{safe_filename(row['id'])}"
            table_rows.append(
                f"""
                <tr>
                  <td><code>{html.escape(row['id'])}</code><span>{html.escape(row['label'])}</span></td>
                  <td>{'通过' if row['accepted'] else '未通过'}</td>
                  <td>{html.escape(str(row['line_counts']))}</td>
                  <td>{row['rectified_point_count']}</td>
                  <td>{row['image_workspace_point_count']}</td>
                  <td>{'' if row['accepted_ratio'] is None else f"{row['accepted_ratio']:.3f}"}</td>
                  <td>{'' if row['accepted_score'] is None else f"{row['accepted_score']:.3f}"}</td>
                  <td>{row['mean_response_at_points']:.3f}</td>
                  <td>{html.escape(attempts)}</td>
                  <td class="links"><a href="images/{prefix}_rectified.png">rectified</a><a href="images/{prefix}_original.png">原图</a></td>
                </tr>
                """
            )
        return "\n".join(table_rows)

    card_html = []
    front_cards = [
        ("00_input_workspace.png", "当前输入工作区"),
        ("90_runtime_selected_rectified.png", f"主链单源 {selected_runtime_source} 结果 rectified"),
        ("91_runtime_selected_original.png", f"主链单源 {selected_runtime_source} 结果原图"),
    ]
    for filename, caption in front_cards:
        card_html.append(
            f"""
            <figure class="image-card">
              <img src="images/{html.escape(filename)}" alt="{html.escape(caption)}">
              <figcaption>{html.escape(caption)}</figcaption>
            </figure>
            """
        )
    for row in evaluation["completed_rows"]:
        prefix = f"completed_{safe_filename(row['id'])}"
        card_html.append(
            f"""
            <figure class="image-card">
              <img src="images/{prefix}_rectified.png" alt="{html.escape(row['label'])}">
              <figcaption>{html.escape(row['label'])}：线数 {html.escape(str(row['line_counts']))}，点数 {row['rectified_point_count']}</figcaption>
            </figure>
            """
        )

    runtime_diag = runtime_result.get("diagnostics", {})
    base_rows_html = render_table(evaluation["base_rows"], "base")
    completed_rows_html = render_table(evaluation["completed_rows"], "completed")
    html_text = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>当前画面 Surface-DP 所有响应源识别报告</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: #18231f;
      background: #f6f4ee;
      font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
      line-height: 1.5;
    }}
    header {{
      padding: 18px clamp(14px, 3vw, 36px);
      background: #fbfaf5;
      border-bottom: 1px solid #d8d3c6;
    }}
    h1 {{ margin: 0 0 6px; font-size: clamp(22px, 3vw, 34px); letter-spacing: 0; }}
    .subline {{ margin: 0; color: #5e6b63; }}
    main {{ max-width: 1720px; margin: 0 auto; padding: 16px clamp(12px, 3vw, 34px) 42px; }}
    .facts {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 190px), 1fr));
      gap: 10px;
      margin: 14px 0 18px;
    }}
    .fact {{
      border: 1px solid #d8d3c6;
      border-radius: 8px;
      padding: 10px 12px;
      background: #fffefa;
      min-width: 0;
    }}
    .fact span {{ display: block; color: #637168; font-size: 12px; }}
    .fact b {{ display: block; font-size: 18px; overflow-wrap: anywhere; }}
    section {{ margin-top: 20px; padding-top: 18px; border-top: 1px solid #d8d3c6; }}
    h2 {{ margin: 0 0 10px; font-size: 20px; letter-spacing: 0; }}
    .note {{
      border-left: 4px solid #2d8f6f;
      background: #eef9f1;
      padding: 10px 12px;
      border-radius: 0 8px 8px 0;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      background: #fffefa;
      border: 1px solid #d8d3c6;
      border-radius: 8px;
      overflow: hidden;
      font-size: 13px;
    }}
    th, td {{ padding: 8px 9px; border-bottom: 1px solid #e4dfd3; vertical-align: top; text-align: left; }}
    th {{ background: #ebe6d8; color: #29332e; }}
    td span {{ display: block; color: #5d6a61; margin-top: 3px; }}
    code {{ background: #ece6d8; border-radius: 4px; padding: 1px 4px; }}
    .links a {{ display: inline-block; margin-right: 8px; color: #176d59; }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(min(100%, 390px), 1fr));
      gap: 14px;
      align-items: start;
    }}
    .image-card {{
      margin: 0;
      border: 1px solid #d8d3c6;
      border-radius: 8px;
      background: #fffefa;
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
    figcaption {{ padding: 9px 10px; color: #39453f; border-top: 1px solid #e4dfd3; }}
    @media (max-width: 760px) {{
      header {{ padding: 14px; }}
      main {{ padding: 12px; }}
      table {{ font-size: 12px; display: block; overflow-x: auto; }}
      .image-card img {{ max-height: 72vh; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>当前画面 Surface-DP 所有响应源识别报告</h1>
    <p class="subline">帧来源：{html.escape(str(frame.get("frame_source")))}；生成时间：{html.escape(summary["generated_at"])}。本报告只做离线诊断，不改运行节点。</p>
  </header>
  <main>
    <div class="facts">
      <div class="fact"><span>rectified 尺寸</span><b>{summary["rectified_size"][0]} x {summary["rectified_size"][1]}</b></div>
      <div class="fact"><span>最终运行线数</span><b>{html.escape(str(summary["runtime_result"]["line_counts"]))}</b></div>
      <div class="fact"><span>最终运行点数</span><b>{summary["runtime_result"]["point_count"]}</b></div>
      <div class="fact"><span>主链策略</span><b>{html.escape(str(runtime_diag.get("scan_runtime_response_policy")))}</b></div>
      <div class="fact"><span>主链响应源</span><b>{html.escape(str(runtime_diag.get("scan_runtime_response_source")))}</b></div>
      <div class="fact"><span>总耗时</span><b>{timings_ms["total"]:.1f} ms</b></div>
    </div>
    <section>
      <h2>读法</h2>
      <p class="note">当前运行主链只生成并使用一次 <code>{html.escape(str(selected_runtime_source))}</code>。下方其它源都是本报告离线生成的隐藏对照源，不参与 ROS 扫描节点运行；每个源单独跑同一套物理线族检测，表里的尝试列显示不同峰值阈值下的线数和是否通过横纵均衡门。</p>
    </section>
    <section>
      <h2>主链源与隐藏基础对照源</h2>
      <table>
        <thead><tr><th>源</th><th>状态</th><th>线数</th><th>rectified 点</th><th>原图内点</th><th>阈值</th><th>得分</th><th>点均值</th><th>尝试</th><th>图</th></tr></thead>
        <tbody>{base_rows_html}</tbody>
      </table>
    </section>
    <section>
      <h2>隐藏旧补全阶段对照源</h2>
      <table>
        <thead><tr><th>源</th><th>状态</th><th>线数</th><th>rectified 点</th><th>原图内点</th><th>阈值</th><th>得分</th><th>点均值</th><th>尝试</th><th>图</th></tr></thead>
        <tbody>{completed_rows_html}</tbody>
      </table>
    </section>
    <section>
      <h2>图像证据</h2>
      <div class="grid">{''.join(card_html)}</div>
    </section>
  </main>
</body>
</html>
"""
    (output_dir / "index.html").write_text(html_text, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"current_scan_all_sources_{time.strftime('%Y%m%d_%H%M%S')}"),
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--threshold-percentile", type=float, default=83.0)
    parser.add_argument("--response-source", default="depth_gradient")
    parser.add_argument("--no-depth-ir-fallback", action="store_true")
    args = parser.parse_args()

    total_start = time.perf_counter()
    rospy.init_node("current_scan_all_sources_report", anonymous=True)
    started = time.perf_counter()
    frame = capture_current_frame(args.timeout, allow_depth_ir_fallback=not args.no_depth_ir_fallback)
    capture_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    runtime_input = build_rectified_runtime_input(frame)
    input_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    evaluation = build_evaluation(runtime_input, args.threshold_percentile, response_source=args.response_source)
    evaluation_ms = (time.perf_counter() - started) * 1000.0

    timings_ms = {
        "capture": capture_ms,
        "build_rectified_input": input_ms,
        "evaluate": evaluation_ms,
        "total": (time.perf_counter() - total_start) * 1000.0,
    }
    write_report(args.output_dir, frame, runtime_input, evaluation, timings_ms)
    print(f"current scan all-source report: {args.output_dir}")


if __name__ == "__main__":
    main()
