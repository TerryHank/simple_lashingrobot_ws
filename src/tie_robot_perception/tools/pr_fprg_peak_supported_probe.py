#!/usr/bin/env python3

"""独立探针：抓取当前相机帧并可视化 PR-FPRG 峰值支撑线/点。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "src"
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from tie_robot_perception.perception.workspace_s2 import (  # noqa: E402
    build_workspace_s2_line_positions,
    build_workspace_s2_oriented_line_families,
    build_workspace_s2_oriented_projective_line_segments,
    build_workspace_s2_rectified_geometry,
    intersect_workspace_s2_oriented_line_families,
    map_workspace_s2_rectified_points_to_image,
    normalize_workspace_s2_response,
    score_workspace_s2_oriented_line_family_result,
    workspace_s2_line_direction_from_angle,
)


class SyncedFrameCapture:
    def __init__(self):
        self.bridge = CvBridge()
        self.data = None

    def callback(self, world_msg, raw_msg, ir_msg):
        if self.data is not None:
            return
        self.data = {
            "world": np.array(self.bridge.imgmsg_to_cv2(world_msg, desired_encoding="passthrough"), copy=True),
            "raw": np.array(self.bridge.imgmsg_to_cv2(raw_msg, desired_encoding="passthrough"), copy=True),
            "ir": np.array(self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding="passthrough"), copy=True),
            "world_seq": int(world_msg.header.seq),
            "raw_seq": int(raw_msg.header.seq),
            "ir_seq": int(ir_msg.header.seq),
            "stamp": float(raw_msg.header.stamp.secs) + float(raw_msg.header.stamp.nsecs) * 1e-9,
        }


class DepthIrFrameCapture:
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_ir = None
        self.data = None

    def depth_callback(self, depth_msg):
        self.latest_depth = {
            "image": np.array(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough"), copy=True),
            "seq": int(depth_msg.header.seq),
            "stamp": float(depth_msg.header.stamp.secs) + float(depth_msg.header.stamp.nsecs) * 1e-9,
        }
        self.try_build_frame()

    def ir_callback(self, ir_msg):
        self.latest_ir = {
            "image": np.array(self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding="passthrough"), copy=True),
            "seq": int(ir_msg.header.seq),
            "stamp": float(ir_msg.header.stamp.secs) + float(ir_msg.header.stamp.nsecs) * 1e-9,
        }
        self.try_build_frame()

    def try_build_frame(self):
        if self.data is not None or self.latest_depth is None or self.latest_ir is None:
            return
        if abs(float(self.latest_depth["stamp"]) - float(self.latest_ir["stamp"])) > 0.8:
            return
        self.data = {
            "world": None,
            "raw": self.latest_depth["image"],
            "ir": self.latest_ir["image"],
            "world_seq": None,
            "raw_seq": self.latest_depth["seq"],
            "ir_seq": self.latest_ir["seq"],
            "stamp": self.latest_depth["stamp"],
            "frame_source": "depth_ir_fallback",
        }


def to_u8(image):
    image = np.asarray(image)
    if image.dtype == np.uint8:
        return image.copy()
    return cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


def to_bgr(image):
    image_u8 = to_u8(image)
    if image_u8.ndim == 2:
        return cv2.cvtColor(image_u8, cv2.COLOR_GRAY2BGR)
    return image_u8.copy()


def get_valid_world_coord_near_pixel(raw_world, pixel_x, pixel_y, search_radius=6):
    height, width = raw_world.shape[:2]
    pixel_x = int(np.clip(pixel_x, 0, width - 1))
    pixel_y = int(np.clip(pixel_y, 0, height - 1))
    value = raw_world[pixel_y, pixel_x, :3].astype(float)
    if np.all(np.isfinite(value)) and not np.any(value == 0.0):
        return value.tolist(), [pixel_x, pixel_y], False

    for radius in range(1, search_radius + 1):
        best_coord = None
        best_pixel = None
        best_distance = None
        for sample_y in range(max(0, pixel_y - radius), min(height - 1, pixel_y + radius) + 1):
            for sample_x in range(max(0, pixel_x - radius), min(width - 1, pixel_x + radius) + 1):
                sample_value = raw_world[sample_y, sample_x, :3].astype(float)
                if not np.all(np.isfinite(sample_value)) or np.any(sample_value == 0.0):
                    continue
                distance = (sample_x - pixel_x) ** 2 + (sample_y - pixel_y) ** 2
                if best_distance is None or distance < best_distance:
                    best_distance = distance
                    best_coord = sample_value.tolist()
                    best_pixel = [sample_x, sample_y]
        if best_coord is not None:
            return best_coord, best_pixel, True
    return value.tolist(), [pixel_x, pixel_y], False


def load_manual_workspace_quad():
    path = WORKSPACE_ROOT / "src" / "tie_robot_perception" / "data" / "manual_workspace_quad.json"
    return json.loads(path.read_text(encoding="utf-8"))


def capture_synced_frame(timeout_sec):
    capture = SyncedFrameCapture()
    subscribers = [
        message_filters.Subscriber("/Scepter/worldCoord/world_coord", Image),
        message_filters.Subscriber("/Scepter/worldCoord/raw_world_coord", Image),
        message_filters.Subscriber("/Scepter/ir/image_raw", Image),
    ]
    sync = message_filters.ApproximateTimeSynchronizer(
        subscribers,
        queue_size=20,
        slop=0.35,
        allow_headerless=False,
    )
    sync.registerCallback(capture.callback)

    start_time = time.time()
    rate = rospy.Rate(50)
    while capture.data is None and time.time() - start_time < timeout_sec and not rospy.is_shutdown():
        rate.sleep()
    if capture.data is None:
        raise RuntimeError("timeout waiting for synced Scepter frames")
    capture.data["frame_source"] = "raw_world"
    return capture.data


def capture_depth_ir_frame(timeout_sec):
    capture = DepthIrFrameCapture()
    rospy.Subscriber("/Scepter/depth/image_raw", Image, capture.depth_callback, queue_size=3)
    rospy.Subscriber("/Scepter/ir/image_raw", Image, capture.ir_callback, queue_size=3)

    start_time = time.time()
    rate = rospy.Rate(50)
    while capture.data is None and time.time() - start_time < timeout_sec and not rospy.is_shutdown():
        rate.sleep()
    if capture.data is None:
        raise RuntimeError("timeout waiting for Scepter depth/IR fallback frames")
    return capture.data


def normalize_probe_raw_world(raw_image):
    raw_image = np.asarray(raw_image, dtype=np.float32)
    if raw_image.ndim == 3 and raw_image.shape[2] >= 3:
        return raw_image[:, :, :3].astype(np.float32), False
    if raw_image.ndim == 2:
        height, width = raw_image.shape[:2]
        y_coords, x_coords = np.indices((height, width), dtype=np.float32)
        return np.dstack([x_coords, y_coords, raw_image]).astype(np.float32), True
    raise RuntimeError(f"unsupported raw/depth image shape for PR-FPRG probe: {raw_image.shape}")


def build_initial_rhos_for_families(line_families):
    for family in line_families:
        estimate = family.get("estimate") or {}
        profile = family.get("profile")
        family["initial_rhos"] = [
            float(family.get("rho_min", 0.0)) + float(position)
            for position in build_workspace_s2_line_positions(
                0,
                max(0, len(profile) - 1),
                estimate.get("period", 0),
                estimate.get("phase", 0),
            )
        ]


def select_best_response_variant(response_candidates, rectified_valid):
    rectified_mask = rectified_valid.astype(np.uint8)
    best_variant = None
    for variant_index, response_candidate in enumerate(response_candidates):
        response_variant = response_candidate["image"]
        normalized_response = normalize_workspace_s2_response(
            response_variant.astype(np.float32),
            rectified_valid,
        )
        line_families = build_workspace_s2_oriented_line_families(
            normalized_response,
            rectified_mask,
            min_period=10,
            max_period=30,
        )
        if len(line_families) < 2 or any(len(family.get("line_rhos", [])) < 2 for family in line_families[:2]):
            continue
        line_families = line_families[:2]
        build_initial_rhos_for_families(line_families)
        combined_score = score_workspace_s2_oriented_line_family_result(line_families)
        if best_variant is None or combined_score > best_variant["combined_score"]:
            best_variant = {
                "variant_index": variant_index,
                "response_source": response_candidate["source"],
                "response_name": response_candidate["name"],
                "response": normalized_response,
                "line_families": line_families,
                "combined_score": combined_score,
            }
    return best_variant


def run_peak_supported_pr_fprg(frame):
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
    best_variant = select_best_response_variant(
        [
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
        ],
        rectified_valid,
    )
    if best_variant is None:
        best_variant = select_best_response_variant(
            [
                {
                    "source": "ir_fallback",
                    "name": "ir_background_minus_intensity",
                    "image": infrared_background - rectified_ir,
                },
                {
                    "source": "ir_fallback",
                    "name": "ir_intensity_minus_background",
                    "image": rectified_ir - infrared_background,
                },
            ],
            rectified_valid,
        )
    if best_variant is None:
        raise RuntimeError("oriented PR-FPRG line family estimation failed")
    response = best_variant["response"]
    rectified_mask = rectified_valid.astype(np.uint8)
    line_families = best_variant["line_families"]
    if any(len(family.get("line_rhos", [])) < 2 for family in line_families):
        raise RuntimeError("oriented PR-FPRG continuous line families insufficient")

    rectified_intersections = intersect_workspace_s2_oriented_line_families(
        line_families[0],
        line_families[1],
        geometry["rectified_width"],
        geometry["rectified_height"],
    )
    image_intersections = map_workspace_s2_rectified_points_to_image(rectified_intersections, geometry["inverse_h"])
    points = []
    for image_point, rectified_point in zip(image_intersections, rectified_intersections):
        pixel_x, pixel_y = int(image_point[0]), int(image_point[1])
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
                "rectified": rectified_point,
                "world": world_coord,
                "sample_pix": sample_pixel,
                "used_fallback": bool(used_fallback),
            }
        )

    return {
        "manual_workspace": manual_workspace,
        "workspace_mask": workspace_mask,
        "rectified_geometry": geometry,
        "rectified_ir": rectified_ir,
        "rectified_depth": rectified_depth,
        "rectified_valid": rectified_valid,
        "filled_depth": filled_depth,
        "response": response,
        "response_variant_index": best_variant["variant_index"],
        "response_source": best_variant["response_source"],
        "response_name": best_variant["response_name"],
        "used_depth_fallback_raw_world": used_depth_fallback_raw_world,
        "line_families": line_families,
        "line_stages": {
            "initial": {
                "family_0": line_families[0].get("initial_rhos", []),
                "family_1": line_families[1].get("initial_rhos", []),
            },
            "peak_supported": {
                "family_0": line_families[0].get("peak_rhos", []),
                "family_1": line_families[1].get("peak_rhos", []),
            },
            "continuous": {
                "family_0": line_families[0].get("continuous_rhos", []),
                "family_1": line_families[1].get("continuous_rhos", []),
            },
            "spacing_pruned": {
                "family_0": line_families[0].get("line_rhos", []),
                "family_1": line_families[1].get("line_rhos", []),
            },
        },
        "points": points,
    }


def render_source_workspace(frame, result):
    image = to_bgr(frame["ir"])
    polygon = np.asarray(result["manual_workspace"]["corner_pixels"], dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [polygon], True, (220, 220, 220), 2)
    overlay = image.copy()
    cv2.fillPoly(overlay, [polygon], (64, 180, 255))
    return cv2.addWeighted(overlay, 0.18, image, 0.82, 0.0)


def render_response_heatmap(response):
    response_u8 = to_u8(response)
    heatmap = cv2.applyColorMap(response_u8, cv2.COLORMAP_TURBO)
    return heatmap


def clip_oriented_rectified_line(line_angle_deg, normal, line_rho, width, height):
    normal = np.asarray(normal, dtype=np.float32).reshape(2)
    direction = workspace_s2_line_direction_from_angle(line_angle_deg)
    point_on_line = normal * float(line_rho)
    t_min = -np.inf
    t_max = np.inf
    bounds = ((0.0, float(width - 1)), (0.0, float(height - 1)))
    for axis_index, (lower_bound, upper_bound) in enumerate(bounds):
        origin_value = float(point_on_line[axis_index])
        direction_value = float(direction[axis_index])
        if abs(direction_value) <= 1e-6:
            if origin_value < lower_bound or origin_value > upper_bound:
                return None
            continue

        t0 = (lower_bound - origin_value) / direction_value
        t1 = (upper_bound - origin_value) / direction_value
        t_min = max(t_min, min(t0, t1))
        t_max = min(t_max, max(t0, t1))
        if t_max < t_min:
            return None

    if not np.isfinite(t_min) or not np.isfinite(t_max):
        return None
    start = point_on_line + (direction * float(t_min))
    end = point_on_line + (direction * float(t_max))
    return (
        [int(round(float(start[0]))), int(round(float(start[1])))],
        [int(round(float(end[0]))), int(round(float(end[1])))],
    )


def draw_oriented_rectified_lines(image, line_families, stage_name, colors=None):
    colors = colors or [(0, 255, 0), (255, 180, 0)]
    for family_index, family in enumerate(line_families):
        stage_key = {
            "initial": "initial_rhos",
            "peak_supported": "peak_rhos",
            "continuous": "continuous_rhos",
            "spacing_pruned": "line_rhos",
        }.get(stage_name, "line_rhos")
        color = colors[family_index % len(colors)]
        for line_rho in family.get(stage_key, []):
            segment = clip_oriented_rectified_line(
                family["line_angle_deg"],
                family["normal"],
                line_rho,
                image.shape[1],
                image.shape[0],
            )
            if segment is None:
                continue
            segment_start, segment_end = segment
            cv2.line(image, tuple(segment_start), tuple(segment_end), color, 1)
    return image


def render_rectified_lines(result, stage_name):
    image = to_bgr(result["rectified_ir"])
    return draw_oriented_rectified_lines(image, result["line_families"], stage_name)


def render_profile_plot(profile, line_positions, axis_name):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    width = max(640, int(profile.size * 2))
    height = 220
    margin_left = 38
    margin_right = 18
    margin_top = 28
    margin_bottom = 34
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.rectangle(
        canvas,
        (margin_left, margin_top),
        (margin_left + plot_width, margin_top + plot_height),
        (220, 220, 220),
        1,
    )
    if profile.size > 1:
        min_value = float(np.min(profile))
        max_value = float(np.max(profile))
        scale = max_value - min_value
        if scale <= 1e-6:
            scale = 1.0
        points = []
        for index, value in enumerate(profile):
            x_pixel = margin_left + int(round((index / float(profile.size - 1)) * plot_width))
            y_pixel = margin_top + plot_height - int(round(((float(value) - min_value) / scale) * plot_height))
            points.append((x_pixel, y_pixel))
        cv2.polylines(canvas, [np.asarray(points, dtype=np.int32)], False, (37, 99, 235), 2)
        for line_position in line_positions:
            x_pixel = margin_left + int(round((float(line_position) / float(profile.size - 1)) * plot_width))
            cv2.line(canvas, (x_pixel, margin_top), (x_pixel, margin_top + plot_height), (22, 163, 74), 1)
    cv2.putText(
        canvas,
        f"{axis_name} profile + selected peaks",
        (margin_left, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (17, 24, 39),
        1,
        cv2.LINE_AA,
    )
    return canvas


def family_profile_positions(family, stage_key="line_rhos"):
    rho_min = float(family.get("rho_min", 0.0))
    return [float(line_rho) - rho_min for line_rho in family.get(stage_key, [])]


def render_removed_lines(result):
    image = to_bgr(result["rectified_ir"])
    for family in result["line_families"]:
        final_rhos = [round(float(line_rho)) for line_rho in family.get("line_rhos", [])]
        for line_rho in family.get("continuous_rhos", []):
            color = (0, 255, 0) if round(float(line_rho)) in final_rhos else (0, 0, 255)
            segment = clip_oriented_rectified_line(
                family["line_angle_deg"],
                family["normal"],
                line_rho,
                image.shape[1],
                image.shape[0],
            )
            if segment is None:
                continue
            segment_start, segment_end = segment
            cv2.line(image, tuple(segment_start), tuple(segment_end), color, 1)
    return image


def render_result(frame, result):
    image = cv2.cvtColor(to_u8(frame["ir"]), cv2.COLOR_GRAY2BGR)
    polygon = np.asarray(result["manual_workspace"]["corner_pixels"], dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(image, [polygon], True, (220, 220, 220), 2)
    segments = build_workspace_s2_oriented_projective_line_segments(
        result["manual_workspace"]["corner_pixels"],
        result["rectified_geometry"]["rectified_width"],
        result["rectified_geometry"]["rectified_height"],
        result["line_families"],
    )
    for family_segments in segments.values():
        for segment_start, segment_end in family_segments:
            cv2.line(image, tuple(segment_start), tuple(segment_end), (0, 255, 0), 1)
    for point in result["points"]:
        cv2.circle(image, tuple(point["pix"]), 4, (0, 255, 255), -1)
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


def save_pr_fprg_step_visualizations(frame, result, output_dir, docs_assets_dir=None):
    output_dir = Path(output_dir)
    steps_dir = output_dir / "steps"
    steps_dir.mkdir(parents=True, exist_ok=True)
    docs_assets_dir = Path(docs_assets_dir) if docs_assets_dir else None
    if docs_assets_dir is not None:
        docs_assets_dir.mkdir(parents=True, exist_ok=True)

    step_images = [
        ("01_input_workspace", render_source_workspace(frame, result)),
        ("02_rectified_ir", to_bgr(result["rectified_ir"])),
        ("03_rectified_depth", render_response_heatmap(result["filled_depth"])),
        ("04_depth_response", render_response_heatmap(result["response"])),
        (
            "05_vertical_profile",
            render_profile_plot(
                result["line_families"][0]["profile"],
                family_profile_positions(result["line_families"][0]),
                "family 0 theta=%.1f" % float(result["line_families"][0]["line_angle_deg"]),
            ),
        ),
        (
            "06_horizontal_profile",
            render_profile_plot(
                result["line_families"][1]["profile"],
                family_profile_positions(result["line_families"][1]),
                "family 1 theta=%.1f" % float(result["line_families"][1]["line_angle_deg"]),
            ),
        ),
        ("07_initial_frequency_phase_grid", render_rectified_lines(result, "initial")),
        ("08_peak_supported_lines", render_rectified_lines(result, "peak_supported")),
        ("09_continuous_line_check", render_rectified_lines(result, "continuous")),
        ("10_spacing_pruned_lines", render_removed_lines(result)),
        ("11_final_result", render_result(frame, result)),
    ]

    artifacts = {}
    for name, image in step_images:
        debug_path = steps_dir / f"{name}.png"
        cv2.imwrite(str(debug_path), image)
        artifacts[name] = str(debug_path)
        if docs_assets_dir is not None:
            docs_path = docs_assets_dir / f"pr-fprg-step-{name}.png"
            cv2.imwrite(str(docs_path), image)
            if name == "11_final_result":
                cv2.imwrite(str(docs_assets_dir.parent / "pr-fprg-result.png"), image)

    return artifacts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--output-dir",
        default=str(WORKSPACE_ROOT / ".debug_frames" / f"pr_fprg_peak_supported_probe_{time.strftime('%Y%m%d_%H%M%S')}"),
    )
    parser.add_argument(
        "--docs-assets-dir",
        default=None,
        help="optional VitePress public asset directory for fixed step image filenames",
    )
    args = parser.parse_args()

    rospy.init_node("pr_fprg_peak_supported_probe", anonymous=True, disable_signals=True)
    try:
        frame = capture_synced_frame(args.timeout)
    except RuntimeError as exc:
        rospy.logwarn(
            "PointAI_log: raw_world同步帧不可用，改用仅供帮助站截图的 depth+IR 兜底：%s",
            exc,
        )
        frame = capture_depth_ir_frame(args.timeout)
    result = run_peak_supported_pr_fprg(frame)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / "peak_supported_result.png"), render_result(frame, result))
    step_artifacts = save_pr_fprg_step_visualizations(
        frame,
        result,
        output_dir,
        docs_assets_dir=args.docs_assets_dir,
    )
    (output_dir / "summary.json").write_text(
        json.dumps(
            {
                "stamp": frame["stamp"],
                "frame_source": frame.get("frame_source", "raw_world"),
                "used_depth_fallback_raw_world": bool(result.get("used_depth_fallback_raw_world", False)),
                "response_variant_index": int(result.get("response_variant_index", 0)),
                "response_source": result.get("response_source"),
                "response_name": result.get("response_name"),
                "seq": {
                    "world": frame.get("world_seq"),
                    "raw": frame.get("raw_seq"),
                    "ir": frame.get("ir_seq"),
                },
                "line_families": [
                    {
                        "line_angle_deg": float(family.get("line_angle_deg", 0.0)),
                        "period": int(family.get("estimate", {}).get("period", 0)),
                        "line_rhos": [float(line_rho) for line_rho in family.get("line_rhos", [])],
                    }
                    for family in result["line_families"]
                ],
                "line_stages": result["line_stages"],
                "point_count": len(result["points"]),
                "first_20_points": result["points"][:20],
                "step_artifacts": step_artifacts,
            },
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    rospy.loginfo("PointAI_log: PR-FPRG峰值支撑探针输出目录：%s", output_dir)


if __name__ == "__main__":
    main()
