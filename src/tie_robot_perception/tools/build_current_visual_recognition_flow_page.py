#!/usr/bin/env python3
"""Build a static report page for the current visual recognition pipeline."""

from __future__ import annotations

import html
import json
import math
import re
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_SRC = REPO_ROOT / "src" / "tie_robot_perception" / "src"
if str(PERCEPTION_SRC) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_SRC))

from tie_robot_perception.perception import workspace_s2  # noqa: E402


SNAPSHOT_DIR = REPO_ROOT / "docs" / "releases" / "slam_v30" / "visual_modalities"
REPORT_DIR = REPO_ROOT / "docs" / "reports" / "current_visual_recognition_flow"
IMAGES_DIR = REPORT_DIR / "images"


def load_array(name: str) -> np.ndarray:
    return np.load(SNAPSHOT_DIR / "arrays" / name)


def read_workspace_quad_pixels() -> list[list[int]]:
    message_path = SNAPSHOT_DIR / "messages" / "perception_lashing_workspace_quad_pixels.txt"
    if message_path.exists():
        match = re.search(r"data:\s*\[([^\]]+)\]", message_path.read_text(encoding="utf-8"))
        if match:
            values = [float(item.strip()) for item in match.group(1).split(",") if item.strip()]
            if len(values) >= 8:
                return [
                    [int(round(values[index])), int(round(values[index + 1]))]
                    for index in range(0, 8, 2)
                ]

    fallback_path = REPO_ROOT / "src" / "tie_robot_perception" / "data" / "manual_workspace_quad.json"
    with fallback_path.open("r", encoding="utf-8") as file_obj:
        return json.load(file_obj)["corner_pixels"]


def normalize_gray(image: np.ndarray, low: float = 2.0, high: float = 98.0) -> np.ndarray:
    image = np.asarray(image, dtype=np.float32)
    finite_mask = np.isfinite(image)
    if not np.any(finite_mask):
        return np.zeros(image.shape[:2], dtype=np.uint8)

    valid_values = image[finite_mask]
    lower = float(np.percentile(valid_values, low))
    upper = float(np.percentile(valid_values, high))
    if upper <= lower + 1e-6:
        lower = float(np.min(valid_values))
        upper = float(np.max(valid_values))
    if upper <= lower + 1e-6:
        return np.zeros(image.shape[:2], dtype=np.uint8)

    normalized = (image - lower) / (upper - lower)
    normalized = np.clip(normalized, 0.0, 1.0)
    normalized[~finite_mask] = 0.0
    return (normalized * 255.0).astype(np.uint8)


def colorize_gray(image: np.ndarray, colormap: int = cv2.COLORMAP_TURBO) -> np.ndarray:
    return cv2.applyColorMap(normalize_gray(image), colormap)


def ensure_bgr(image: np.ndarray) -> np.ndarray:
    image = np.asarray(image)
    if image.ndim == 2:
        return cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 4:
        return cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_BGRA2BGR)
    return image[:, :, :3].astype(np.uint8)


def draw_label(image: np.ndarray, text: str, origin: tuple[int, int] = (14, 28)) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.58
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = origin
    cv2.rectangle(
        image,
        (x - 8, y - text_height - baseline - 8),
        (x + text_width + 8, y + baseline + 8),
        (10, 16, 22),
        -1,
    )
    cv2.putText(image, text, (x, y), font, scale, (242, 246, 249), thickness, cv2.LINE_AA)


def save_image(name: str, image: np.ndarray, label: str | None = None) -> str:
    output_path = IMAGES_DIR / name
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image_to_write = ensure_bgr(image)
    if label:
        image_to_write = image_to_write.copy()
        draw_label(image_to_write, label)
    if not cv2.imwrite(str(output_path), image_to_write):
        raise RuntimeError(f"failed to write image: {output_path}")
    return f"images/{name}"


def get_valid_world_coord_near_pixel(raw_world: np.ndarray, pixel_x: int, pixel_y: int, search_radius: int = 6):
    height, width = raw_world.shape[:2]
    pixel_x = int(np.clip(pixel_x, 0, width - 1))
    pixel_y = int(np.clip(pixel_y, 0, height - 1))

    def is_valid(coord: np.ndarray) -> bool:
        return bool(
            np.all(np.isfinite(coord))
            and float(coord[0]) != 0.0
            and float(coord[1]) != 0.0
            and float(coord[2]) != 0.0
        )

    raw_coord = raw_world[pixel_y, pixel_x, :3].astype(np.float32)
    if is_valid(raw_coord):
        return raw_coord.tolist(), [pixel_x, pixel_y], False

    best_coord = None
    best_pixel = None
    best_distance = None
    for radius in range(1, search_radius + 1):
        min_y = max(0, pixel_y - radius)
        max_y = min(height - 1, pixel_y + radius)
        min_x = max(0, pixel_x - radius)
        max_x = min(width - 1, pixel_x + radius)
        for sample_y in range(min_y, max_y + 1):
            for sample_x in range(min_x, max_x + 1):
                candidate = raw_world[sample_y, sample_x, :3].astype(np.float32)
                if not is_valid(candidate):
                    continue
                distance = (sample_x - pixel_x) ** 2 + (sample_y - pixel_y) ** 2
                if best_distance is None or distance < best_distance:
                    best_distance = distance
                    best_coord = candidate.tolist()
                    best_pixel = [sample_x, sample_y]
        if best_coord is not None:
            return best_coord, best_pixel, True

    return [0.0, 0.0, 0.0], [pixel_x, pixel_y], False


def build_workspace_mask(shape: tuple[int, int], corner_pixels: list[list[int]]) -> np.ndarray:
    mask = np.zeros(shape, dtype=np.uint8)
    polygon = np.asarray(
        workspace_s2.sort_polygon_points_clockwise(corner_pixels),
        dtype=np.int32,
    )
    cv2.fillPoly(mask, [polygon], 1)
    return mask


def draw_workspace_overlay(ir_image: np.ndarray, corner_pixels: list[list[int]]) -> np.ndarray:
    overlay = cv2.cvtColor(ir_image.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    polygon = np.asarray(workspace_s2.sort_polygon_points_clockwise(corner_pixels), dtype=np.int32)
    fill = overlay.copy()
    cv2.fillPoly(fill, [polygon], (38, 168, 122))
    overlay = cv2.addWeighted(fill, 0.22, overlay, 0.78, 0)
    cv2.polylines(overlay, [polygon], True, (36, 255, 172), 2)
    for index, point in enumerate(polygon.tolist(), start=1):
        cv2.circle(overlay, tuple(point), 5, (0, 220, 255), -1)
        cv2.putText(
            overlay,
            str(index),
            (int(point[0]) + 7, int(point[1]) - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 220, 255),
            1,
            cv2.LINE_AA,
        )
    return overlay


def draw_profile_plot(
    vertical_profile: np.ndarray,
    horizontal_profile: np.ndarray,
    vertical_lines: list[int],
    horizontal_lines: list[int],
) -> np.ndarray:
    canvas = np.full((560, 1060, 3), (248, 250, 249), dtype=np.uint8)
    panels = [
        (vertical_profile, vertical_lines, (50, 56, 1010, 260), "vertical profile / X lines", (20, 126, 86)),
        (horizontal_profile, horizontal_lines, (50, 320, 1010, 524), "horizontal profile / Y lines", (18, 106, 168)),
    ]
    font = cv2.FONT_HERSHEY_SIMPLEX
    for profile, line_positions, (left, top, right, bottom), title, color in panels:
        cv2.rectangle(canvas, (left, top), (right, bottom), (218, 224, 223), 1)
        cv2.putText(canvas, title, (left, top - 14), font, 0.55, (36, 42, 46), 1, cv2.LINE_AA)
        profile = np.asarray(profile, dtype=np.float32).reshape(-1)
        if profile.size <= 1:
            continue
        profile_min = float(np.min(profile))
        profile_max = float(np.max(profile))
        if profile_max <= profile_min + 1e-6:
            scaled = np.zeros_like(profile)
        else:
            scaled = (profile - profile_min) / (profile_max - profile_min)
        plot_width = right - left
        plot_height = bottom - top
        points = []
        for index, value in enumerate(scaled):
            x = left + int(round((index / max(1, profile.size - 1)) * plot_width))
            y = bottom - int(round(float(value) * plot_height))
            points.append([x, y])
        cv2.polylines(canvas, [np.asarray(points, dtype=np.int32)], False, color, 2, cv2.LINE_AA)
        for position in line_positions:
            x = left + int(round((float(position) / max(1, profile.size - 1)) * plot_width))
            cv2.line(canvas, (x, top), (x, bottom), (54, 64, 73), 1, cv2.LINE_AA)
    return canvas


def draw_rectified_grid(response: np.ndarray, vertical_lines: list[int], horizontal_lines: list[int]) -> np.ndarray:
    grid_image = colorize_gray(response)
    for vertical_x in vertical_lines:
        cv2.line(grid_image, (int(vertical_x), 0), (int(vertical_x), grid_image.shape[0] - 1), (38, 235, 154), 1)
    for horizontal_y in horizontal_lines:
        cv2.line(grid_image, (0, int(horizontal_y)), (grid_image.shape[1] - 1, int(horizontal_y)), (35, 190, 255), 1)
    return grid_image


def draw_projective_overlay(
    ir_image: np.ndarray,
    line_segments: dict[str, list[tuple[list[int], list[int]]]],
    points: list[tuple[int, int, list[float]]],
    number_points: bool = False,
) -> np.ndarray:
    output = cv2.cvtColor(ir_image.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    for segment_start, segment_end in line_segments.get("vertical", []):
        cv2.line(output, tuple(segment_start), tuple(segment_end), (32, 235, 151), 1, cv2.LINE_AA)
    for segment_start, segment_end in line_segments.get("horizontal", []):
        cv2.line(output, tuple(segment_start), tuple(segment_end), (30, 182, 255), 1, cv2.LINE_AA)
    for index, (point_x, point_y, _) in enumerate(points, start=1):
        cv2.circle(output, (int(point_x), int(point_y)), 4, (0, 235, 255), -1, cv2.LINE_AA)
        if number_points:
            cv2.putText(
                output,
                str(index),
                (int(point_x) + 5, int(point_y) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.32,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
    return output


def run_scan_pipeline() -> tuple[list[dict[str, str]], dict[str, float | int | str]]:
    ir_image = load_array("Scepter_ir_image_raw.npy").astype(np.uint8)
    color_image = load_array("Scepter_color_image_raw.npy").astype(np.uint8)
    raw_world = load_array("Scepter_worldCoord_raw_world_coord.npy").astype(np.float32)
    depth_image = raw_world[:, :, 2].astype(np.float32)
    corner_pixels = read_workspace_quad_pixels()
    workspace_mask = build_workspace_mask(depth_image.shape[:2], corner_pixels)
    corner_world = [
        get_valid_world_coord_near_pixel(raw_world, int(pixel[0]), int(pixel[1]))[0]
        for pixel in corner_pixels
    ]

    geometry = workspace_s2.build_workspace_s2_rectified_geometry(corner_pixels, corner_world)
    if geometry is None:
        raise RuntimeError("failed to build rectified geometry")

    valid_mask = np.isfinite(depth_image) & (depth_image != 0.0)
    rectified_size = (geometry["rectified_width"], geometry["rectified_height"])
    rectified_depth = cv2.warpPerspective(
        depth_image,
        geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_LINEAR,
    ).astype(np.float32)
    rectified_valid_mask = cv2.warpPerspective(
        valid_mask.astype(np.uint8),
        geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_NEAREST,
    ).astype(bool)

    median_depth = float(np.median(rectified_depth[rectified_valid_mask]))
    filled_depth = np.where(rectified_valid_mask, rectified_depth, median_depth).astype(np.float32)
    background_depth = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=11.0, sigmaY=11.0)
    depth_response_variants = [
        background_depth - filled_depth,
        filled_depth - background_depth,
    ]

    best_variant = None
    for response_variant in depth_response_variants:
        normalized_response = workspace_s2.normalize_workspace_s2_response(response_variant, rectified_valid_mask)
        mask_uint8 = rectified_valid_mask.astype(np.uint8)
        vertical_profile = workspace_s2.build_workspace_s2_axis_profile(normalized_response, mask_uint8, axis=0)
        horizontal_profile = workspace_s2.build_workspace_s2_axis_profile(normalized_response, mask_uint8, axis=1)
        vertical_estimate = workspace_s2.estimate_workspace_s2_period_and_phase(
            vertical_profile,
            min_period=10,
            max_period=30,
        )
        horizontal_estimate = workspace_s2.estimate_workspace_s2_period_and_phase(
            horizontal_profile,
            min_period=10,
            max_period=30,
        )
        if vertical_estimate is None or horizontal_estimate is None:
            continue
        best_variant = {
            "response": normalized_response,
            "vertical_profile": vertical_profile,
            "horizontal_profile": horizontal_profile,
            "vertical_estimate": vertical_estimate,
            "horizontal_estimate": horizontal_estimate,
        }
        break

    if best_variant is None:
        raise RuntimeError("failed to estimate current scan period and phase")

    vertical_lines = workspace_s2.build_workspace_s2_line_positions(
        0,
        geometry["rectified_width"] - 1,
        best_variant["vertical_estimate"]["period"],
        best_variant["vertical_estimate"]["phase"],
    )
    horizontal_lines = workspace_s2.build_workspace_s2_line_positions(
        0,
        geometry["rectified_height"] - 1,
        best_variant["horizontal_estimate"]["period"],
        best_variant["horizontal_estimate"]["phase"],
    )
    rectified_intersections = [
        [float(vertical_x), float(horizontal_y)]
        for horizontal_y in horizontal_lines
        for vertical_x in vertical_lines
    ]
    image_intersections = workspace_s2.map_workspace_s2_rectified_points_to_image(
        rectified_intersections,
        geometry["inverse_h"],
    )
    valid_points = []
    for point_x, point_y in image_intersections:
        if (
            0 <= point_y < workspace_mask.shape[0]
            and 0 <= point_x < workspace_mask.shape[1]
            and workspace_mask[point_y, point_x]
        ):
            camera_coord, _, _ = get_valid_world_coord_near_pixel(raw_world, point_x, point_y)
            if all(float(value) != 0.0 and math.isfinite(float(value)) for value in camera_coord):
                valid_points.append((int(point_x), int(point_y), camera_coord))

    line_segments = workspace_s2.build_workspace_s2_projective_line_segments(
        corner_pixels,
        geometry["rectified_width"],
        geometry["rectified_height"],
        vertical_lines,
        horizontal_lines,
    )

    scan_steps = [
        {
            "title": "01 原始彩色输入",
            "detail": "/Scepter/color/image_raw；只作现场背景参照。",
            "src": save_image("01_scan_color_input.png", color_image, "color input"),
        },
        {
            "title": "02 红外输入",
            "detail": "/Scepter/ir/image_raw；当前扫描最终叠加图的底图。",
            "src": save_image("02_scan_ir_input.png", ir_image, "IR input"),
        },
        {
            "title": "03 原始相机深度",
            "detail": "/Scepter/worldCoord/raw_world_coord 的 Z 通道，扫描算法只用这一路做响应。",
            "src": save_image("03_scan_raw_depth.png", colorize_gray(depth_image), "raw world Z"),
        },
        {
            "title": "04 手动工作区",
            "detail": "前端保存的四边形工作区，后续只在这个区域内透视展开和取点。",
            "src": save_image("04_scan_workspace_overlay.png", draw_workspace_overlay(ir_image, corner_pixels), "workspace"),
        },
        {
            "title": "05 透视展开深度",
            "detail": "四边形工作区被展开到 rectified 平面，宽高由四角相机坐标尺度估计。",
            "src": save_image("05_scan_rectified_depth.png", colorize_gray(rectified_depth), "rectified depth"),
        },
        {
            "title": "06 depth-only 背景差分",
            "detail": "当前版本只取 depth 背景差分响应，不混入 IR 响应。",
            "src": save_image("06_scan_depth_response.png", colorize_gray(best_variant["response"]), "depth-only response"),
        },
        {
            "title": "07 纵横 profile 周期相位",
            "detail": "分别沿 X/Y 方向投影响应，估计 period 与 phase。",
            "src": save_image(
                "07_scan_axis_profiles.png",
                draw_profile_plot(
                    best_variant["vertical_profile"],
                    best_variant["horizontal_profile"],
                    vertical_lines,
                    horizontal_lines,
                ),
            ),
        },
        {
            "title": "08 rectified 网格",
            "detail": "按估计出的纵横线位置构造规则网格，交点仍在 rectified 平面。",
            "src": save_image(
                "08_scan_rectified_grid.png",
                draw_rectified_grid(best_variant["response"], vertical_lines, horizontal_lines),
                "rectified grid",
            ),
        },
        {
            "title": "09 透视网格反投影",
            "detail": "将 rectified 网格线和交点通过 inverse H 投回原图。",
            "src": save_image(
                "09_scan_projective_overlay.png",
                draw_projective_overlay(ir_image, line_segments, valid_points, number_points=False),
                "projective overlay",
            ),
        },
        {
            "title": "10 扫描识别输出",
            "detail": "最终输出有效相机坐标点，同步用于 /coordinate_point 与扫描账本。",
            "src": save_image(
                "10_scan_final_overlay.png",
                draw_projective_overlay(ir_image, line_segments, valid_points, number_points=True),
                "scan output",
            ),
        },
    ]

    scan_metrics = {
        "rectified_width": int(geometry["rectified_width"]),
        "rectified_height": int(geometry["rectified_height"]),
        "vertical_period": int(best_variant["vertical_estimate"]["period"]),
        "vertical_phase": int(best_variant["vertical_estimate"]["phase"]),
        "horizontal_period": int(best_variant["horizontal_estimate"]["period"]),
        "horizontal_phase": int(best_variant["horizontal_estimate"]["phase"]),
        "vertical_line_count": int(len(vertical_lines)),
        "horizontal_line_count": int(len(horizontal_lines)),
        "point_count": int(len(valid_points)),
        "source": "docs/releases/slam_v30/visual_modalities",
    }
    return scan_steps, scan_metrics


def remove_small_foreground_components(binary_image: np.ndarray, min_area_px: int = 20) -> np.ndarray:
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(binary_image, connectivity=8)
    filtered = np.zeros_like(binary_image)
    for component_idx in range(1, component_count):
        if stats[component_idx, cv2.CC_STAT_AREA] >= min_area_px:
            filtered[labels == component_idx] = 255
    return filtered


def axis_angle_diff_deg(x1: float, y1: float, x2: float, y2: float) -> float:
    angle_deg = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1))) % 180.0
    return min(angle_deg, abs(90.0 - angle_deg), abs(180.0 - angle_deg))


def filter_axis_aligned_lines(lines, tolerance_deg: float = 15.0):
    if lines is None:
        return None
    filtered = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if axis_angle_diff_deg(x1, y1, x2, y2) <= tolerance_deg:
            filtered.append(line)
    if not filtered:
        return None
    return np.asarray(filtered, dtype=np.int32)


def calculate_intersections(lines) -> list[tuple[float, float]]:
    if lines is None:
        return []
    intersections = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            x1, y1, x2, y2 = lines[i][0]
            x3, y3, x4, y4 = lines[j][0]
            v1 = (x2 - x1, y2 - y1)
            v2 = (x4 - x3, y4 - y3)
            dot_product = v1[0] * v2[0] + v1[1] * v2[1]
            cross_product = v1[0] * v2[1] - v1[1] * v2[0]
            angle = abs(np.degrees(np.arctan2(cross_product, dot_product)))
            if not (35 <= angle <= 150):
                continue
            denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if denom == 0:
                continue
            px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
            py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
            if (
                min(x1, x2) <= px <= max(x1, x2)
                and min(y1, y2) <= py <= max(y1, y2)
                and min(x3, x4) <= px <= max(x3, x4)
                and min(y3, y4) <= py <= max(y3, y4)
            ):
                intersections.append((float(px), float(py)))
    return intersections


def draw_hough_overlay(base: np.ndarray, lines, intersections: list[tuple[float, float]]) -> np.ndarray:
    output = ensure_bgr(base)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(output, (int(x1), int(y1)), (int(x2), int(y2)), (32, 235, 151), 2, cv2.LINE_AA)
    for point_x, point_y in intersections:
        cv2.circle(output, (int(round(point_x)), int(round(point_y))), 4, (0, 215, 255), -1, cv2.LINE_AA)
    return output


def run_execution_pipeline() -> tuple[list[dict[str, str]], dict[str, int | str]]:
    ir_image = load_array("Scepter_ir_image_raw.npy").astype(np.uint8)
    world_coord = load_array("Scepter_worldCoord_world_coord.npy").astype(np.float32)
    pointai_result = load_array("pointAI_result_image_raw.npy")
    perception_result = load_array("perception_lashing_result_image.npy")

    z_channel = world_coord[:, :, 2].astype(np.float32)
    positive_depth = z_channel[z_channel > 0.0]
    if positive_depth.size == 0:
        raise RuntimeError("execution sample has no positive world_coord Z values")
    max_depth = float(np.max(positive_depth) - 5.0)
    binary = np.zeros(z_channel.shape[:2], dtype=np.uint8)
    binary[(z_channel >= 11.0) & (z_channel <= max_depth)] = 255
    binary = cv2.medianBlur(binary, 3)
    binary = remove_small_foreground_components(binary, min_area_px=20)

    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        skeleton = cv2.ximgproc.thinning(binary, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
    else:
        skeleton = binary

    lines = cv2.HoughLinesP(
        skeleton,
        rho=1,
        theta=np.pi / 180,
        threshold=45,
        minLineLength=60,
        maxLineGap=150,
    )
    lines = filter_axis_aligned_lines(lines, tolerance_deg=15.0)
    intersections = calculate_intersections(lines)

    line_canvas = np.zeros_like(binary)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_canvas, (int(x1), int(y1)), (int(x2), int(y2)), 255, 2, cv2.LINE_AA)

    runtime_montage = np.full((480, 1320, 3), (236, 240, 239), dtype=np.uint8)
    left = cv2.resize(ensure_bgr(pointai_result), (640, 480), interpolation=cv2.INTER_AREA)
    right = cv2.resize(ensure_bgr(perception_result), (640, 480), interpolation=cv2.INTER_AREA)
    runtime_montage[:, :640] = left
    runtime_montage[:, 680:1320] = right
    cv2.putText(runtime_montage, "/pointAI/result_image_raw", (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240, 250, 246), 2, cv2.LINE_AA)
    cv2.putText(runtime_montage, "/perception/lashing/result_image", (700, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240, 250, 246), 2, cv2.LINE_AA)

    execution_steps = [
        {
            "title": "11 平面分割世界坐标输入",
            "detail": "执行微调读取 /Scepter/worldCoord/world_coord，使用 Z 通道构造非平面区域。",
            "src": save_image("11_execution_world_coord_z.png", colorize_gray(z_channel), "world_coord Z"),
        },
        {
            "title": "12 Hough 二值图",
            "detail": "按当前阈值逻辑保留非平面深度像素，再做小连通域清理。",
            "src": save_image("12_execution_binary.png", binary, "binary"),
        },
        {
            "title": "13 骨架与轴向线段",
            "detail": "二值图细化后运行 HoughLinesP，只保留近水平或近垂直线段。",
            "src": save_image("13_execution_hough_lines.png", line_canvas, "Hough lines"),
        },
        {
            "title": "14 Hough 交点叠加",
            "detail": "线段交点聚类后进入执行范围筛选，输出局部执行候选点。",
            "src": save_image(
                "14_execution_intersections.png",
                draw_hough_overlay(cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR), lines, intersections),
                "Hough intersections",
            ),
        },
        {
            "title": "15 运行话题结果参考",
            "detail": "离线快照中的结果话题预览；现场运行时由上面的当前算法实时发布。",
            "src": save_image("15_runtime_result_topics.png", runtime_montage),
        },
    ]

    execution_metrics = {
        "hough_line_count": 0 if lines is None else int(len(lines)),
        "hough_intersection_count": int(len(intersections)),
        "binary_pixels": int(np.count_nonzero(binary)),
        "source": "docs/releases/slam_v30/visual_modalities",
    }
    return execution_steps, execution_metrics


def render_step_cards(steps: list[dict[str, str]]) -> str:
    cards = []
    for step in steps:
        cards.append(
            f"""
            <article class="flow-card">
              <figure>
                <img loading="lazy" src="{html.escape(step['src'])}" alt="{html.escape(step['title'])}">
              </figure>
              <div class="card-copy">
                <h3>{html.escape(step['title'])}</h3>
                <p>{html.escape(step['detail'])}</p>
              </div>
            </article>
            """.strip()
        )
    return "\n".join(cards)


def render_html(scan_steps, scan_metrics, execution_steps, execution_metrics) -> str:
    scan_metric_rows = [
        ("rectified", f"{scan_metrics['rectified_width']} x {scan_metrics['rectified_height']} px"),
        ("v period / phase", f"{scan_metrics['vertical_period']} / {scan_metrics['vertical_phase']}"),
        ("h period / phase", f"{scan_metrics['horizontal_period']} / {scan_metrics['horizontal_phase']}"),
        ("grid lines", f"{scan_metrics['vertical_line_count']} x {scan_metrics['horizontal_line_count']}"),
        ("valid points", str(scan_metrics["point_count"])),
    ]
    execution_metric_rows = [
        ("binary pixels", str(execution_metrics["binary_pixels"])),
        ("Hough lines", str(execution_metrics["hough_line_count"])),
        ("intersections", str(execution_metrics["hough_intersection_count"])),
    ]

    def metric_markup(rows):
        return "\n".join(
            f"<div class=\"metric\"><span>{html.escape(label)}</span><strong>{html.escape(value)}</strong></div>"
            for label, value in rows
        )

    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>当前视觉识别流程效果图</title>
  <style>
    :root {{
      --paper: #f5f3ee;
      --ink: #17201d;
      --muted: #65716c;
      --line: #cbd3ce;
      --panel: #fffdf8;
      --accent: #127c55;
      --accent-2: #0d6e9c;
      --warn: #b66a18;
      --shadow: 0 18px 42px rgba(32, 42, 38, 0.12);
    }}

    * {{ box-sizing: border-box; }}

    body {{
      margin: 0;
      color: var(--ink);
      background:
        linear-gradient(90deg, rgba(23, 32, 29, 0.04) 1px, transparent 1px),
        linear-gradient(180deg, rgba(23, 32, 29, 0.04) 1px, transparent 1px),
        var(--paper);
      background-size: 28px 28px;
      font-family: "Noto Sans CJK SC", "Source Han Sans SC", "Microsoft YaHei", sans-serif;
      letter-spacing: 0;
    }}

    header {{
      padding: 28px clamp(18px, 4vw, 48px) 20px;
      border-bottom: 1px solid var(--line);
      background: rgba(245, 243, 238, 0.94);
      position: sticky;
      top: 0;
      z-index: 10;
      backdrop-filter: blur(14px);
    }}

    .topline {{
      display: flex;
      gap: 14px;
      justify-content: space-between;
      align-items: flex-end;
      flex-wrap: wrap;
    }}

    h1 {{
      margin: 0;
      font-size: clamp(30px, 5vw, 58px);
      line-height: 1;
      font-weight: 900;
    }}

    .subtitle {{
      margin: 10px 0 0;
      max-width: 980px;
      color: var(--muted);
      font-size: 15px;
      line-height: 1.7;
    }}

    .badges {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-top: 16px;
    }}

    .badge {{
      border: 1px solid var(--line);
      background: #ffffff;
      padding: 7px 10px;
      font-size: 12px;
      font-weight: 800;
      text-transform: uppercase;
    }}

    main {{
      padding: 26px clamp(18px, 4vw, 48px) 54px;
    }}

    .notice {{
      max-width: 1160px;
      border-left: 4px solid var(--accent);
      padding: 14px 18px;
      background: rgba(255, 253, 248, 0.78);
      box-shadow: var(--shadow);
      line-height: 1.7;
      font-size: 14px;
    }}

    section {{
      margin-top: 34px;
    }}

    .section-head {{
      display: grid;
      grid-template-columns: minmax(0, 1fr) minmax(260px, 380px);
      gap: 18px;
      align-items: end;
      margin-bottom: 18px;
    }}

    h2 {{
      margin: 0;
      font-size: clamp(24px, 3.4vw, 38px);
      line-height: 1.08;
      font-weight: 900;
    }}

    .section-head p {{
      margin: 9px 0 0;
      color: var(--muted);
      line-height: 1.7;
      font-size: 14px;
      max-width: 860px;
    }}

    .metrics {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }}

    .metric {{
      min-height: 58px;
      padding: 10px 12px;
      background: #17201d;
      color: #f8fbf7;
      border: 1px solid rgba(255, 255, 255, 0.16);
    }}

    .metric span {{
      display: block;
      color: #b7c4bd;
      font-size: 11px;
      text-transform: uppercase;
    }}

    .metric strong {{
      display: block;
      margin-top: 5px;
      font-size: 18px;
      white-space: nowrap;
    }}

    .flow-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 16px;
    }}

    .flow-card {{
      display: grid;
      grid-template-rows: auto 1fr;
      background: var(--panel);
      border: 1px solid var(--line);
      box-shadow: 0 12px 28px rgba(32, 42, 38, 0.08);
      overflow: hidden;
    }}

    .flow-card figure {{
      margin: 0;
      background: #17201d;
      border-bottom: 1px solid var(--line);
    }}

    .flow-card img {{
      display: block;
      width: 100%;
      aspect-ratio: 4 / 3;
      object-fit: contain;
      background: #17201d;
    }}

    .card-copy {{
      padding: 14px 16px 16px;
    }}

    h3 {{
      margin: 0;
      font-size: 17px;
      line-height: 1.35;
    }}

    .card-copy p {{
      margin: 8px 0 0;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.65;
    }}

    .runtime-note {{
      margin-top: 24px;
      padding: 16px 18px;
      background: #fffdf8;
      border: 1px solid var(--line);
      line-height: 1.7;
      color: var(--muted);
    }}

    code {{
      font-family: "Cascadia Mono", "Fira Code", Consolas, monospace;
      color: #153f31;
      background: rgba(18, 124, 85, 0.12);
      padding: 2px 5px;
      border-radius: 4px;
    }}

    @media (max-width: 980px) {{
      .section-head {{
        grid-template-columns: 1fr;
      }}

      .flow-grid {{
        grid-template-columns: 1fr;
      }}
    }}

    @media (max-width: 560px) {{
      header {{
        position: static;
      }}

      .metrics {{
        grid-template-columns: 1fr;
      }}

      .flow-card img {{
        aspect-ratio: 1 / 1;
      }}
    }}
  </style>
</head>
<body>
  <header>
    <div class="topline">
      <div>
        <h1>当前视觉识别流程效果图</h1>
        <p class="subtitle">
          基于仓库内 <code>slam_v30</code> 视觉模态快照离线生成，扫描分支按当前代码的 2026-04-22
          <code>manual workspace S2</code> 口径重跑：depth-only 背景差分、纵横 profile 周期相位、透视网格反投影。
        </p>
      </div>
    </div>
    <div class="badges">
      <span class="badge">/pointAI/process_image request_mode=3</span>
      <span class="badge">MODE_SCAN_ONLY</span>
      <span class="badge">MODE_EXECUTION_REFINE</span>
      <span class="badge">2026-05-03</span>
    </div>
  </header>

  <main>
    <div class="notice">
      当前扫描识别链路不包含 depth+IR 组合响应、方向线族或梁筋 ±13 cm 过滤；这些后续实验链路只保留在研究工具和报告中，不进入扫描运行路径。
      执行微调仍走平面分割 + Hough，用于逐区到位后的局部视觉。
    </div>

    <section>
      <div class="section-head">
        <div>
          <h2>扫描识别：2026-04-22 PR-FPRG</h2>
          <p>
            固定识别位姿下，前端触发仍进入 <code>/pointAI/process_image</code> 的
            <code>request_mode=3</code>。算法只看手动工作区内的原始相机深度，展开到 rectified 平面后
            做周期相位估计，再把规则网格映射回原图并输出相机坐标点。
          </p>
        </div>
        <div class="metrics">
          {metric_markup(scan_metric_rows)}
        </div>
      </div>
      <div class="flow-grid">
        {render_step_cards(scan_steps)}
      </div>
    </section>

    <section>
      <div class="section-head">
        <div>
          <h2>执行微调：平面分割 + Hough</h2>
          <p>
            执行层逐区到位后不复用扫描网格主链，而是读取平面分割后的
            <code>/Scepter/worldCoord/world_coord</code>，构造二值图、提取近水平/垂直 Hough 线段，
            再聚类交点并进入执行范围筛选。
          </p>
        </div>
        <div class="metrics">
          {metric_markup(execution_metric_rows)}
        </div>
      </div>
      <div class="flow-grid">
        {render_step_cards(execution_steps)}
      </div>
    </section>

    <div class="runtime-note">
      本页是离线可打开的流程效果页，生成脚本为
      <code>src/tie_robot_perception/tools/build_current_visual_recognition_flow_page.py</code>。
      重新采集现场样例后，可替换 <code>docs/releases/slam_v30/visual_modalities</code> 下的快照并重新运行脚本生成新图。
    </div>
  </main>
</body>
</html>
"""


def main() -> int:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    if IMAGES_DIR.exists():
        shutil.rmtree(IMAGES_DIR)
    IMAGES_DIR.mkdir(parents=True, exist_ok=True)

    scan_steps, scan_metrics = run_scan_pipeline()
    execution_steps, execution_metrics = run_execution_pipeline()
    REPORT_PATH = REPORT_DIR / "index.html"
    REPORT_PATH.write_text(
        render_html(scan_steps, scan_metrics, execution_steps, execution_metrics),
        encoding="utf-8",
    )
    manifest = {
        "report": str(REPORT_PATH.relative_to(REPO_ROOT)),
        "scan_metrics": scan_metrics,
        "execution_metrics": execution_metrics,
        "image_count": len(scan_steps) + len(execution_steps),
    }
    (REPORT_DIR / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"wrote {REPORT_PATH}")
    print(f"wrote {manifest['image_count']} images")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
