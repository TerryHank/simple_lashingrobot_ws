#!/usr/bin/env python3

"""Ablate expensive PR-FPRG stages on one captured frame and compare timing/point drift."""

from __future__ import annotations

import argparse
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
    load_manual_workspace_quad,
    normalize_probe_raw_world,
    select_best_response_variant,
    to_u8,
)
from pr_fprg_scheme_comparison import points_from_rectified_intersections  # noqa: E402
from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_oriented_line_families,
    build_workspace_s2_rectified_geometry,
    build_workspace_s2_structural_edge_suppression_mask,
    filter_workspace_s2_rectified_points_outside_mask,
    intersect_workspace_s2_oriented_line_families,
    normalize_workspace_s2_response,
    score_workspace_s2_oriented_line_family_result,
)


def prepare_frame_inputs(frame):
    manual_workspace = load_manual_workspace_quad()
    corner_pixels = manual_workspace["corner_pixels"]
    corner_world = manual_workspace.get("corner_world_camera_frame")
    raw_world, used_depth_fallback_raw_world = normalize_probe_raw_world(frame["raw"])
    ir = to_u8(frame["ir"])

    workspace_mask = np.zeros(ir.shape[:2], dtype=np.uint8)
    cv2.fillPoly(workspace_mask, [np.asarray(corner_pixels, dtype=np.int32).reshape((-1, 1, 2))], 1)

    geometry = build_workspace_s2_rectified_geometry(corner_pixels, corner_world)
    rectified_size = (geometry["rectified_width"], geometry["rectified_height"])
    rectified_ir = cv2.warpPerspective(
        ir.astype(np.float32),
        geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_LINEAR,
    ).astype(np.float32)
    raw_depth = raw_world[:, :, 2].astype(np.float32)
    valid_mask = np.isfinite(raw_depth) & (raw_depth != 0.0)
    rectified_depth = cv2.warpPerspective(raw_depth, geometry["forward_h"], rectified_size, flags=cv2.INTER_LINEAR)
    rectified_valid = cv2.warpPerspective(
        valid_mask.astype(np.uint8),
        geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_NEAREST,
    ).astype(bool)
    median_depth = float(np.median(rectified_depth[rectified_valid]))
    filled_depth = np.where(rectified_valid, rectified_depth, median_depth).astype(np.float32)
    background_depth = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=11.0, sigmaY=11.0)
    infrared_background = cv2.GaussianBlur(rectified_ir, (0, 0), sigmaX=7.0, sigmaY=7.0)
    infrared_dark_line_response = infrared_background - rectified_ir
    structural_edge_response = normalize_workspace_s2_response(
        infrared_dark_line_response.astype(np.float32),
        rectified_valid,
    )

    response_variants = [
        {
            "source": "depth",
            "name": "depth_background_minus_filled",
            "image": background_depth - filled_depth,
        },
        {
            "source": "depth",
            "name": "depth_filled_minus_background",
            "image": filled_depth - background_depth,
        },
        {
            "source": "ir_fallback",
            "name": "ir_background_minus_intensity",
            "image": infrared_dark_line_response,
        },
        {
            "source": "ir_fallback",
            "name": "ir_intensity_minus_background",
            "image": rectified_ir - infrared_background,
        },
    ]

    return {
        "manual_workspace": manual_workspace,
        "workspace_mask": workspace_mask,
        "rectified_geometry": geometry,
        "rectified_ir": rectified_ir,
        "rectified_depth": rectified_depth,
        "rectified_valid": rectified_valid,
        "filled_depth": filled_depth,
        "used_depth_fallback_raw_world": used_depth_fallback_raw_world,
        "response_variants": response_variants,
        "structural_edge_response": structural_edge_response,
    }


def run_line_family_variant(prepared, variant, response_name_filter=None):
    rectified_valid = prepared["rectified_valid"]
    rectified_mask = rectified_valid.astype(np.uint8)
    best_result = None
    response_candidates = [
        response_candidate
        for response_candidate in prepared["response_variants"]
        if response_name_filter is None or response_candidate["name"] == response_name_filter
    ]
    if response_name_filter is None:
        candidate_groups = [
            [candidate for candidate in response_candidates if candidate["source"] == "depth"],
            [candidate for candidate in response_candidates if candidate["source"] != "depth"],
        ]
    else:
        candidate_groups = [response_candidates]

    for candidate_group in candidate_groups:
        selected_group_result = None
        for response_candidate in candidate_group:
            response = normalize_workspace_s2_response(
                response_candidate["image"].astype(np.float32),
                rectified_valid,
            )
            started = time.perf_counter()
            line_families = build_workspace_s2_oriented_line_families(
                response,
                rectified_mask,
                min_period=10,
                max_period=30,
                use_orientation_prior_angle_pool=variant.get("use_orientation_prior_angle_pool", True),
                enable_local_peak_refine=variant.get("enable_local_peak_refine", True),
                enable_peak_support=variant.get("enable_peak_support", True),
                enable_continuous_validation=variant.get("enable_continuous_validation", True),
                enable_spacing_prune=variant.get("enable_spacing_prune", True),
            )
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            if len(line_families) < 2 or any(len(family.get("line_rhos", [])) < 2 for family in line_families[:2]):
                continue

            line_families = line_families[:2]
            selected_group_result = {
                "elapsed_ms": elapsed_ms,
                "response_name": response_candidate["name"],
                "response_source": response_candidate["source"],
                "response": response,
                "line_families": line_families,
                "score": score_workspace_s2_oriented_line_family_result(line_families),
            }
            if selected_group_result is not None:
                break
        if selected_group_result is not None:
            best_result = selected_group_result
            break
    if best_result is None:
        raise RuntimeError(f"variant {variant['id']} did not produce two supported line families")

    rectified_intersections = intersect_workspace_s2_oriented_line_families(
        best_result["line_families"][0],
        best_result["line_families"][1],
        prepared["rectified_geometry"]["rectified_width"],
        prepared["rectified_geometry"]["rectified_height"],
    )
    beam_mask = build_workspace_s2_structural_edge_suppression_mask(
        prepared.get("structural_edge_response", best_result["response"]),
        rectified_mask,
    )
    rectified_intersections = filter_workspace_s2_rectified_points_outside_mask(
        rectified_intersections,
        beam_mask,
        margin_px=2,
    )
    result_like = {
        "workspace_mask": prepared["workspace_mask"],
        "rectified_geometry": prepared["rectified_geometry"],
    }
    return best_result, rectified_intersections, result_like


def sorted_points_xy(points):
    coords = np.asarray([point["pix"] for point in points], dtype=np.float32)
    if coords.size == 0:
        return coords.reshape(0, 2)
    order = np.lexsort((coords[:, 0], coords[:, 1]))
    return coords[order]


def point_drift_stats(reference_points, points):
    reference = sorted_points_xy(reference_points)
    current = sorted_points_xy(points)
    if reference.shape[0] == 0 or current.shape[0] == 0:
        return {
            "matched": 0,
            "mean_px": None,
            "max_px": None,
            "point_count_delta": int(current.shape[0] - reference.shape[0]),
        }

    matched = min(reference.shape[0], current.shape[0])
    distances = np.linalg.norm(reference[:matched] - current[:matched], axis=1)
    return {
        "matched": int(matched),
        "mean_px": float(np.mean(distances)),
        "max_px": float(np.max(distances)),
        "point_count_delta": int(current.shape[0] - reference.shape[0]),
    }


def summarize_families(line_families):
    return [
        {
            "angle_deg": round(float(family.get("line_angle_deg", 0.0)), 3),
            "line_rhos": [round(float(rho), 3) for rho in family.get("line_rhos", [])],
            "peak_rhos": [round(float(rho), 3) for rho in family.get("peak_rhos", [])],
            "continuous_rhos": [round(float(rho), 3) for rho in family.get("continuous_rhos", [])],
        }
        for family in line_families
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument(
        "--allow-response-switch",
        action="store_true",
        help="let every ablation variant choose its own best response map; default locks all variants to full pipeline response",
    )
    parser.add_argument(
        "--output-json",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"pr_fprg_stage_ablation_{time.strftime('%Y%m%d_%H%M%S')}.json"),
    )
    args = parser.parse_args()

    rospy.init_node("pr_fprg_stage_ablation", anonymous=True, disable_signals=True)
    try:
        frame = capture_synced_frame(args.timeout)
    except RuntimeError as exc:
        rospy.logwarn("PointAI_log: raw_world同步帧不可用，改用 depth+IR 兜底做阶段消融：%s", exc)
        frame = capture_depth_ir_frame(args.timeout)

    prepared = prepare_frame_inputs(frame)
    variants = [
        {
            "id": "full",
            "label": "全流程：08:21 连续/ridge 主链 + 梁筋点级过滤",
            "enable_local_peak_refine": True,
            "use_orientation_prior_angle_pool": True,
            "enable_continuous_validation": True,
            "enable_spacing_prune": True,
        },
        {
            "id": "skip_continuous",
            "label": "关闭连续/ridge 验证",
            "enable_continuous_validation": False,
            "enable_spacing_prune": True,
        },
        {
            "id": "skip_spacing",
            "label": "关闭 spacing prune",
            "enable_continuous_validation": True,
            "enable_spacing_prune": False,
        },
        {
            "id": "skip_peak_support",
            "label": "去掉 peak 支撑过滤",
            "enable_peak_support": False,
        },
        {
            "id": "profile_only_spacing",
            "label": "只保留 profile 周期 + spacing",
            "enable_local_peak_refine": False,
            "enable_peak_support": False,
            "enable_continuous_validation": False,
            "enable_spacing_prune": True,
        },
        {
            "id": "profile_only_raw",
            "label": "只保留 profile 周期原始线",
            "enable_local_peak_refine": False,
            "enable_peak_support": False,
            "enable_continuous_validation": False,
            "enable_spacing_prune": False,
        },
        {
            "id": "full_angle_sweep",
            "label": "全角度候选池对照",
            "use_orientation_prior_angle_pool": False,
        },
    ]

    repeat_count = max(1, int(args.repeat))
    baseline_family_result, baseline_rectified_intersections, baseline_result_like = run_line_family_variant(
        prepared,
        variants[0],
    )
    locked_response_name = None if args.allow_response_switch else baseline_family_result["response_name"]
    baseline_points = points_from_rectified_intersections(
        frame,
        baseline_result_like,
        baseline_rectified_intersections,
    )

    results = []
    for variant in variants:
        timings = []
        last_payload = None
        failure = None
        for _ in range(repeat_count):
            try:
                started = time.perf_counter()
                family_result, rectified_intersections, result_like = run_line_family_variant(
                    prepared,
                    variant,
                    response_name_filter=locked_response_name,
                )
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                points = points_from_rectified_intersections(frame, result_like, rectified_intersections)
                timings.append(elapsed_ms)
                last_payload = {
                    "family_result": family_result,
                    "rectified_intersections": rectified_intersections,
                    "points": points,
                }
            except Exception as exc:
                failure = str(exc)
                break

        if failure is not None or last_payload is None:
            results.append(
                {
                    "id": variant["id"],
                    "label": variant["label"],
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
                    "response_name": locked_response_name,
                    "families": [],
                    "first_20_points": [],
                    "failure": failure,
                }
            )
            continue

        drift = point_drift_stats(baseline_points, last_payload["points"])
        results.append(
            {
                "id": variant["id"],
                "label": variant["label"],
                "mean_ms": float(statistics.mean(timings)),
                "median_ms": float(statistics.median(timings)),
                "min_ms": float(min(timings)),
                "max_ms": float(max(timings)),
                "point_count": len(last_payload["points"]),
                "drift_vs_full": drift,
                "response_name": last_payload["family_result"]["response_name"],
                "families": summarize_families(last_payload["family_result"]["line_families"]),
                "first_20_points": last_payload["points"][:20],
            }
        )

    output = {
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frame_source": frame.get("frame_source", "raw_world"),
        "repeat": repeat_count,
        "locked_response_name": locked_response_name,
        "allow_response_switch": bool(args.allow_response_switch),
        "results": results,
    }
    output_path = Path(args.output_json)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")

    print("frame_source:", output["frame_source"])
    print("repeat:", repeat_count)
    print("locked_response_name:", locked_response_name)
    print("id,label,mean_ms,median_ms,point_count,mean_drift_px,max_drift_px,point_count_delta,response")
    for item in results:
        if item.get("failure"):
            print(
                f"{item['id']},{item['label']},FAILED,FAILED,0,,,,"
                f"{item.get('response_name')}: {item['failure']}"
            )
            continue
        drift = item["drift_vs_full"]
        mean_drift = "" if drift["mean_px"] is None else f"{drift['mean_px']:.3f}"
        max_drift = "" if drift["max_px"] is None else f"{drift['max_px']:.3f}"
        print(
            f"{item['id']},{item['label']},{item['mean_ms']:.3f},{item['median_ms']:.3f},"
            f"{item['point_count']},{mean_drift},{max_drift},{drift['point_count_delta']},"
            f"{item['response_name']}"
        )
    print("output_json:", output_path)


if __name__ == "__main__":
    main()
