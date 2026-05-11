"""Runtime scan-layer Surface-DP rebar intersection detector.

This module is intentionally ROS-free.  It keeps the experimental
combined/fused response pipeline usable from the pointAI runtime without
importing report-only scripts from ``tools/``.
"""

import cv2
import numpy as np

from tie_robot_perception.perception.workspace_s2 import (
    build_workspace_s2_axis_profile,
    build_workspace_s2_curved_line_families,
    expand_workspace_s2_exclusion_mask_by_metric_margin,
    filter_workspace_s2_rectified_points_outside_mask,
    intersect_workspace_s2_curved_line_families,
    intersect_workspace_s2_oriented_line_families,
    normalize_workspace_s2_profile_for_support,
    normalize_workspace_s2_response,
    select_workspace_s2_peak_supported_line_positions,
)


FULL_SCAN_REBAR_SPACING_MM_RANGE = (120.0, 160.0)
MIN_PHYSICAL_LATTICE_LINE_COUNT = 2
FULL_WORKSPACE_MODE_MIN_VISIBLE_LINE_COUNT = 15
FULL_WORKSPACE_MIN_LINE_COUNT_FRACTION = 0.50
FULL_WORKSPACE_MIN_AXIS_COVERAGE_RATIO = 0.50
MIN_PHYSICAL_AXIS_PROFILE_CONTRAST = 1e-4
DEFAULT_RESOLUTION_MM_PER_PX = 5.0


def _valid_mask_from_result(result):
    valid_mask = np.asarray(result.get("rectified_valid"), dtype=bool)
    if valid_mask.ndim != 2:
        raise ValueError("rectified_valid must be a 2-D mask")
    return valid_mask


def _response_value_at(response_map, point):
    response_map = np.asarray(response_map, dtype=np.float32)
    if response_map.ndim != 2 or response_map.size == 0:
        return 0.0
    x_index = int(round(float(point[0])))
    y_index = int(round(float(point[1])))
    if x_index < 0 or y_index < 0 or y_index >= response_map.shape[0] or x_index >= response_map.shape[1]:
        return 0.0
    return float(response_map[y_index, x_index])


def _normalize_response(response_map, valid_mask, lower_percentile=5.0, upper_percentile=95.0):
    response_map = np.asarray(response_map, dtype=np.float32)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    if response_map.size == 0 or not np.any(valid_mask):
        return np.zeros_like(response_map, dtype=np.float32)
    valid_values = response_map[valid_mask]
    lower = float(np.percentile(valid_values, lower_percentile))
    upper = float(np.percentile(valid_values, upper_percentile))
    if upper <= lower + 1e-6:
        lower = float(np.min(valid_values))
        upper = float(np.max(valid_values))
    if upper <= lower + 1e-6:
        return np.zeros_like(response_map, dtype=np.float32)
    return normalize_workspace_s2_response(
        response_map,
        valid_mask,
        lower_percentile=lower_percentile,
        upper_percentile=upper_percentile,
    )


def _extract_depth_image(result, valid_mask):
    if result.get("filled_depth") is not None:
        depth = np.asarray(result["filled_depth"], dtype=np.float32)
    else:
        depth = np.asarray(result.get("rectified_depth"), dtype=np.float32)
    if depth.shape != valid_mask.shape:
        return np.zeros_like(valid_mask, dtype=np.float32)
    if not np.any(valid_mask):
        return depth.astype(np.float32)
    median_depth = float(np.median(depth[valid_mask]))
    return np.where(valid_mask, depth, median_depth).astype(np.float32)


def build_depth_response(result):
    valid_mask = _valid_mask_from_result(result)
    filled_depth = _extract_depth_image(result, valid_mask)
    background_depth = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=11.0, sigmaY=11.0)
    depth_response = background_depth - filled_depth
    return _normalize_response(depth_response, valid_mask)


def build_infrared_response(result):
    valid_mask = _valid_mask_from_result(result)
    rectified_ir = result.get("rectified_ir")
    if rectified_ir is None:
        return np.zeros_like(valid_mask, dtype=np.float32)
    rectified_ir = np.asarray(rectified_ir, dtype=np.float32)
    if rectified_ir.shape != valid_mask.shape:
        return np.zeros_like(valid_mask, dtype=np.float32)
    if not np.any(valid_mask):
        return np.zeros_like(valid_mask, dtype=np.float32)
    fill_value = float(np.median(rectified_ir[valid_mask]))
    filled_ir = np.where(valid_mask, rectified_ir, fill_value).astype(np.float32)
    background = cv2.GaussianBlur(filled_ir, (0, 0), sigmaX=7.0, sigmaY=7.0)
    return _normalize_response(background - filled_ir, valid_mask)


def build_combined_response(result, depth_weight=0.68):
    valid_mask = _valid_mask_from_result(result)
    selected_response = result.get("response")
    if str(result.get("response_source", "")) == "depth_ir" and selected_response is not None:
        return _normalize_response(np.asarray(selected_response, dtype=np.float32), valid_mask)
    depth_response = build_depth_response(result)
    infrared_response = build_infrared_response(result)
    if np.count_nonzero(infrared_response) == 0:
        return depth_response
    depth_weight = float(np.clip(depth_weight, 0.0, 1.0))
    combined_response = (depth_weight * depth_response) + ((1.0 - depth_weight) * infrared_response)
    return _normalize_response(combined_response, valid_mask)


def build_depth_gradient_response(result):
    valid_mask = _valid_mask_from_result(result)
    filled_depth = _extract_depth_image(result, valid_mask)
    smoothed = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=1.2, sigmaY=1.2)
    grad_x = cv2.Sobel(smoothed, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(smoothed, cv2.CV_32F, 0, 1, ksize=3)
    gradient = cv2.magnitude(grad_x, grad_y)
    return _normalize_response(gradient, valid_mask)


def hessian_ridge_response(response_map, valid_mask, sigma=1.6):
    response_map = np.asarray(response_map, dtype=np.float32)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    blurred = cv2.GaussianBlur(response_map, (0, 0), sigmaX=float(sigma), sigmaY=float(sigma))
    dxx = cv2.Sobel(blurred, cv2.CV_32F, 2, 0, ksize=3)
    dyy = cv2.Sobel(blurred, cv2.CV_32F, 0, 2, ksize=3)
    dxy = cv2.Sobel(blurred, cv2.CV_32F, 1, 1, ksize=3)
    trace = dxx + dyy
    discriminant = np.sqrt(np.maximum(((dxx - dyy) ** 2) + (4.0 * dxy * dxy), 0.0))
    lambda_a = 0.5 * (trace + discriminant)
    lambda_b = 0.5 * (trace - discriminant)
    lambda_min = np.minimum(lambda_a, lambda_b)
    bright_ridge = np.maximum(-lambda_min, 0.0)
    ridge_response = bright_ridge * np.maximum(response_map, 0.0)
    return _normalize_response(ridge_response, valid_mask)


def multiscale_frangi_like_response(response_map, valid_mask, sigmas=(0.9, 1.6, 2.8, 4.2)):
    ridge_layers = [hessian_ridge_response(response_map, valid_mask, sigma=sigma) for sigma in sigmas]
    if not ridge_layers:
        return np.zeros_like(response_map, dtype=np.float32)
    return _normalize_response(np.max(np.stack(ridge_layers, axis=0), axis=0), valid_mask)


def threshold_response(response_map, valid_mask, percentile=83.0):
    response_map = np.asarray(response_map, dtype=np.float32)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    if not np.any(valid_mask):
        return np.zeros_like(response_map, dtype=bool), 0.0
    threshold = float(np.percentile(response_map[valid_mask], float(percentile)))
    binary = (response_map >= threshold) & valid_mask
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary_u8 = cv2.morphologyEx(binary.astype(np.uint8), cv2.MORPH_CLOSE, kernel, iterations=1)
    binary_u8 = cv2.morphologyEx(binary_u8, cv2.MORPH_OPEN, kernel, iterations=1)
    return binary_u8.astype(bool), threshold


def skeletonize_binary(binary_mask):
    binary_u8 = np.asarray(binary_mask, dtype=np.uint8) * 255
    if binary_u8.size == 0:
        return binary_u8.astype(bool)
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        skeleton = cv2.ximgproc.thinning(binary_u8, cv2.ximgproc.THINNING_ZHANGSUEN)
        return skeleton > 0

    skeleton = np.zeros_like(binary_u8)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    working = binary_u8.copy()
    while True:
        eroded = cv2.erode(working, element)
        opened = cv2.dilate(eroded, element)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(working, opened))
        working = eroded
        if cv2.countNonZero(working) == 0:
            break
    return skeleton > 0


def count_skeleton_nodes(skeleton):
    skeleton = np.asarray(skeleton, dtype=bool)
    if skeleton.size == 0:
        return 0, 0
    neighbor_count = cv2.filter2D(
        skeleton.astype(np.uint8),
        cv2.CV_16S,
        np.ones((3, 3), dtype=np.uint8),
        borderType=cv2.BORDER_CONSTANT,
    )
    neighbor_count = neighbor_count - skeleton.astype(np.int16)
    endpoints = skeleton & (neighbor_count == 1)
    junctions = skeleton & (neighbor_count >= 3)
    return int(np.count_nonzero(endpoints)), int(np.count_nonzero(junctions))


def _normalize01(image, valid_mask=None, lower_percentile=2.0, upper_percentile=98.0):
    image = np.asarray(image, dtype=np.float32)
    normalized = np.zeros_like(image, dtype=np.float32)
    if image.size == 0:
        return normalized
    finite_mask = np.isfinite(image)
    if valid_mask is not None:
        finite_mask &= np.asarray(valid_mask, dtype=bool)
    if not np.any(finite_mask):
        return normalized
    values = image[finite_mask]
    lower = float(np.percentile(values, lower_percentile))
    upper = float(np.percentile(values, upper_percentile))
    if upper <= lower + 1e-6:
        lower = float(np.min(values))
        upper = float(np.max(values))
    if upper <= lower + 1e-6:
        normalized[finite_mask] = 1.0
        return normalized
    normalized = np.clip((image - lower) / (upper - lower), 0.0, 1.0).astype(np.float32)
    normalized[~finite_mask] = 0.0
    return normalized


def _profile_components_from_active(profile, active):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    active = np.asarray(active, dtype=bool).reshape(-1)
    components = []
    start = None
    for index, is_active in enumerate(active.tolist() + [False]):
        if is_active and start is None:
            start = index
        elif not is_active and start is not None:
            end = index - 1
            local = profile[start:end + 1]
            peak_index = int(start + int(np.argmax(local))) if local.size else int(start)
            components.append(
                {
                    "start": int(start),
                    "end": int(end),
                    "width": int(end - start + 1),
                    "peak": float(profile[peak_index]),
                    "peak_index": int(peak_index),
                }
            )
            start = None
    return components


def _top_fraction_mean(values, fraction=0.12):
    values = np.asarray(values, dtype=np.float32).reshape(-1)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return 0.0
    count = max(1, int(np.ceil(float(values.size) * float(fraction))))
    if count >= values.size:
        return float(np.mean(values))
    top_values = np.partition(values, values.size - count)[values.size - count:]
    return float(np.mean(top_values))


def _column_height_profile(height_response, valid_mask):
    if height_response is None:
        return None
    height_response = np.asarray(height_response, dtype=np.float32)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    if height_response.ndim != 2 or height_response.shape != valid_mask.shape or not np.any(valid_mask):
        return None

    valid_counts = np.sum(valid_mask, axis=0).astype(np.float32)
    safe_counts = np.maximum(valid_counts, 1.0)
    profile = np.sum(np.where(valid_mask, height_response, 0.0), axis=0) / safe_counts
    profile[valid_counts <= 0.0] = 0.0
    return profile.astype(np.float32)


def _refine_beam_band_by_height(start, end, height_profile, image_width):
    if height_profile is None:
        return None
    height_profile = np.asarray(height_profile, dtype=np.float32).reshape(-1)
    image_width = int(max(1, image_width))
    if height_profile.size != image_width:
        return None

    original_start = int(np.clip(int(start), 0, image_width - 1))
    original_end = int(np.clip(int(end), original_start, image_width - 1))
    original_width = int(original_end - original_start + 1)
    band_values = height_profile[original_start:original_end + 1]
    if band_values.size == 0:
        return None

    guard_width = max(4, min(10, int(round(image_width * 0.012))))
    context_width = max(20, min(image_width, int(round(image_width * 0.065))))
    left_start = max(0, original_start - guard_width - context_width)
    left_end = max(left_start, original_start - guard_width)
    right_start = min(image_width, original_end + guard_width + 1)
    right_end = min(image_width, original_end + guard_width + 1 + context_width)
    context_parts = []
    if left_end > left_start:
        context_parts.append(height_profile[left_start:left_end])
    if right_end > right_start:
        context_parts.append(height_profile[right_start:right_end])
    if context_parts:
        context_values = np.concatenate(context_parts)
    else:
        context_values = np.concatenate((height_profile[:original_start], height_profile[original_end + 1:]))
    if context_values.size == 0:
        return None

    band_height_score = _top_fraction_mean(band_values, fraction=0.18)
    surrounding_height_score = _top_fraction_mean(context_values, fraction=0.12)
    peak_height_score = float(np.max(band_values))
    height_delta = float(band_height_score - surrounding_height_score)
    peak_delta = float(peak_height_score - surrounding_height_score)
    raised_enough = (
        peak_height_score >= 0.62
        and (
            (height_delta >= 0.055 and peak_delta >= 0.055)
            or (height_delta >= 0.040 and peak_delta >= 0.140)
        )
    )
    if not raised_enough:
        return None

    active_threshold = max(
        surrounding_height_score + 0.035,
        surrounding_height_score + max(0.045, peak_delta * 0.42),
    )
    active = band_values >= active_threshold
    if active.size > 2:
        active = cv2.morphologyEx(
            active.reshape(1, -1).astype(np.uint8),
            cv2.MORPH_CLOSE,
            np.ones((1, 3), dtype=np.uint8),
            iterations=1,
        ).reshape(-1).astype(bool)

    peak_local_index = int(np.argmax(band_values))
    active[peak_local_index] = True
    components = []
    component_start = None
    for index, is_active in enumerate(active.tolist() + [False]):
        if is_active and component_start is None:
            component_start = index
        elif not is_active and component_start is not None:
            components.append((int(component_start), int(index - 1)))
            component_start = None
    component_start, component_end = min(
        components,
        key=lambda bounds: 0 if bounds[0] <= peak_local_index <= bounds[1] else min(
            abs(bounds[0] - peak_local_index),
            abs(bounds[1] - peak_local_index),
        ),
    )

    pad = max(1, min(3, int(round(image_width * 0.006))))
    refined_start = max(original_start, original_start + component_start - pad)
    refined_end = min(original_end, original_start + component_end + pad)
    min_width = min(original_width, max(5, int(round(image_width * 0.012))))
    if (refined_end - refined_start + 1) < min_width:
        peak_x = original_start + peak_local_index
        half_width = int(np.ceil((min_width - 1) / 2.0))
        refined_start = max(original_start, peak_x - half_width)
        refined_end = min(original_end, refined_start + min_width - 1)
        refined_start = max(original_start, refined_end - min_width + 1)

    return {
        "start": int(refined_start),
        "end": int(refined_end),
        "width": int(refined_end - refined_start + 1),
        "original_start": int(original_start),
        "original_end": int(original_end),
        "original_width": int(original_width),
        "height_score": float(band_height_score),
        "surrounding_height_score": float(surrounding_height_score),
        "height_peak": float(peak_height_score),
        "height_delta": float(height_delta),
    }


def _refine_beam_band_by_structural_continuity(
    start,
    end,
    structural_profile,
    vertical_coverage,
    height_profile,
    image_width,
):
    structural_profile = np.asarray(structural_profile, dtype=np.float32).reshape(-1)
    vertical_coverage = np.asarray(vertical_coverage, dtype=np.float32).reshape(-1)
    image_width = int(max(1, image_width))
    if structural_profile.size != image_width or vertical_coverage.size != image_width:
        return None

    original_start = int(np.clip(int(start), 0, image_width - 1))
    original_end = int(np.clip(int(end), original_start, image_width - 1))
    original_width = int(original_end - original_start + 1)
    min_width = max(8, int(round(image_width * 0.016)))
    if original_width < min_width:
        return None

    band_values = structural_profile[original_start:original_end + 1]
    coverage_values = vertical_coverage[original_start:original_end + 1]
    if band_values.size == 0 or coverage_values.size == 0:
        return None

    guard_width = max(4, min(10, int(round(image_width * 0.012))))
    context_width = max(20, min(image_width, int(round(image_width * 0.065))))
    left_start = max(0, original_start - guard_width - context_width)
    left_end = max(left_start, original_start - guard_width)
    right_start = min(image_width, original_end + guard_width + 1)
    right_end = min(image_width, original_end + guard_width + 1 + context_width)
    context_parts = []
    if left_end > left_start:
        context_parts.append(structural_profile[left_start:left_end])
    if right_end > right_start:
        context_parts.append(structural_profile[right_start:right_end])
    if context_parts:
        context_values = np.concatenate(context_parts)
    else:
        context_values = np.concatenate(
            (structural_profile[:original_start], structural_profile[original_end + 1:])
        )
    if context_values.size == 0:
        return None

    peak_score = float(np.max(band_values))
    structural_score = _top_fraction_mean(band_values, fraction=0.25)
    surrounding_structural_score = _top_fraction_mean(context_values, fraction=0.15)
    structural_delta = float(structural_score - surrounding_structural_score)
    coverage_score = float(np.mean(coverage_values))
    coverage_peak = float(np.max(coverage_values))
    if not (
        peak_score >= 1.32
        and structural_delta >= 0.18
        and coverage_score >= 0.34
        and coverage_peak >= 0.36
    ):
        return None

    height_peak = None
    height_score = None
    height_delta = None
    if height_profile is not None:
        height_profile = np.asarray(height_profile, dtype=np.float32).reshape(-1)
        if height_profile.size == image_width:
            height_band = height_profile[original_start:original_end + 1]
            height_context = []
            if left_end > left_start:
                height_context.append(height_profile[left_start:left_end])
            if right_end > right_start:
                height_context.append(height_profile[right_start:right_end])
            height_peak = float(np.max(height_band)) if height_band.size else 0.0
            height_score = _top_fraction_mean(height_band, fraction=0.18)
            if height_context:
                surrounding_height = _top_fraction_mean(np.concatenate(height_context), fraction=0.12)
                height_delta = float(height_score - surrounding_height)

    return {
        "start": int(original_start),
        "end": int(original_end),
        "width": int(original_width),
        "original_start": int(original_start),
        "original_end": int(original_end),
        "original_width": int(original_width),
        "height_score": float(height_score) if height_score is not None else 0.0,
        "height_peak": float(height_peak) if height_peak is not None else 0.0,
        "height_delta": float(height_delta) if height_delta is not None else 0.0,
        "structural_score": float(structural_score),
        "surrounding_structural_score": float(surrounding_structural_score),
        "structural_delta": float(structural_delta),
        "coverage_score": float(coverage_score),
        "coverage_peak": float(coverage_peak),
    }


def detect_beam_candidate_bands(candidate_response, binary_candidate, valid_mask, height_response=None):
    candidate_response = np.asarray(candidate_response, dtype=np.float32)
    binary_candidate = np.asarray(binary_candidate, dtype=bool)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    bands = []
    if candidate_response.ndim != 2 or candidate_response.shape != valid_mask.shape or not np.any(valid_mask):
        return bands

    height, width = candidate_response.shape[:2]
    del height
    height_profile = _column_height_profile(height_response, valid_mask)
    vertical_profile = np.mean(np.where(valid_mask, candidate_response, 0.0), axis=0)
    vertical_coverage = np.mean(binary_candidate & valid_mask, axis=0)
    structural_profile = _normalize01(vertical_profile) + _normalize01(vertical_coverage)
    structural_profile = cv2.GaussianBlur(structural_profile.reshape(1, -1), (0, 0), sigmaX=2.5).reshape(-1)
    threshold = max(float(np.percentile(structural_profile, 88.0)), 0.32)
    active_profile = structural_profile >= threshold
    close_kernel_width = max(7, min(15, int(round(width * 0.03))))
    close_kernel = np.ones((1, close_kernel_width), dtype=np.uint8)
    active_profile = cv2.morphologyEx(
        active_profile.reshape(1, -1).astype(np.uint8),
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=1,
    ).reshape(-1).astype(bool)
    components = _profile_components_from_active(structural_profile, active_profile)

    def overlaps_existing_band(start, end):
        for existing_band in bands:
            if existing_band.get("axis") != "x":
                continue
            existing_start = int(existing_band.get("start", 0))
            existing_end = int(existing_band.get("end", existing_start))
            if min(int(end), existing_end) >= max(int(start), existing_start):
                return True
        return False

    for component in components:
        band_slice = slice(int(component["start"]), int(component["end"]) + 1)
        band_valid = valid_mask[:, band_slice]
        if not np.any(band_valid):
            continue
        coverage = float(np.count_nonzero(binary_candidate[:, band_slice] & band_valid) / np.count_nonzero(band_valid))
        wide_enough = component["width"] >= max(5, int(round(width * 0.012)))
        tall_enough = coverage >= 0.38
        strong_enough = float(component["peak"]) >= 0.75
        if not (wide_enough and strong_enough):
            continue
        height_band = None
        if tall_enough:
            height_band = _refine_beam_band_by_height(
                int(component["start"]),
                int(component["end"]),
                height_profile,
                width,
            )
        if height_band is None:
            height_band = _refine_beam_band_by_structural_continuity(
                int(component["start"]),
                int(component["end"]),
                structural_profile,
                vertical_coverage,
                height_profile,
                width,
            )
            if height_band is None:
                continue
            bands.append(
                {
                    "axis": "x",
                    "start": int(height_band["start"]),
                    "end": int(height_band["end"]),
                    "width": int(height_band["width"]),
                    "peak": float(component["peak"]),
                    "coverage": coverage,
                    "type": "beam_candidate",
                    "beam_signature": "wide_continuous_column",
                    "height_gate": "structure_continuity_height_flat",
                    **height_band,
                }
            )
            continue
        bands.append(
            {
                "axis": "x",
                "start": int(height_band["start"]),
                "end": int(height_band["end"]),
                "width": int(height_band["width"]),
                "peak": float(component["peak"]),
                "coverage": coverage,
                "type": "beam_candidate",
                "height_gate": "raised_column",
                **height_band,
            }
        )

    min_pair_band_width = max(9, int(round(width * 0.025)))
    max_pair_band_width = max(36, int(round(width * 0.14)))
    min_pair_gap_width = max(5, int(round(width * 0.012)))
    max_pair_gap_width = max(28, int(round(width * 0.12)))
    for left, right in zip(components, components[1:]):
        start = int(left["start"])
        end = int(right["end"])
        gap_start = int(left["end"]) + 1
        gap_end = int(right["start"]) - 1
        gap_width = gap_end - gap_start + 1
        band_width = end - start + 1
        if gap_width < min_pair_gap_width or gap_width > max_pair_gap_width:
            continue
        if band_width < min_pair_band_width or band_width > max_pair_band_width:
            continue
        if overlaps_existing_band(start, end):
            continue

        left_slice = slice(int(left["start"]), int(left["end"]) + 1)
        right_slice = slice(int(right["start"]), int(right["end"]) + 1)
        gap_slice = slice(gap_start, gap_end + 1)
        left_coverage = float(np.mean(vertical_coverage[left_slice])) if left["width"] > 0 else 0.0
        right_coverage = float(np.mean(vertical_coverage[right_slice])) if right["width"] > 0 else 0.0
        edge_coverage = min(left_coverage, right_coverage)
        interior_coverage = float(np.mean(vertical_coverage[gap_slice])) if gap_width > 0 else 1.0
        edge_peak = min(float(left["peak"]), float(right["peak"]))
        interior_valley = float(np.min(structural_profile[gap_slice])) if gap_width > 0 else float("inf")

        strong_edges = edge_peak >= 0.70 and edge_coverage >= 0.30
        dark_gutter = (
            interior_coverage <= min(0.085, max(0.045, edge_coverage * 0.18))
            and interior_valley <= (edge_peak * 0.45)
        )
        if not (strong_edges and dark_gutter):
            continue
        height_band = _refine_beam_band_by_height(start, end, height_profile, width)
        if height_band is None:
            continue

        bands.append(
            {
                "axis": "x",
                "start": int(height_band["start"]),
                "end": int(height_band["end"]),
                "width": int(height_band["width"]),
                "peak": float(max(float(left["peak"]), float(right["peak"]))),
                "coverage": float(edge_coverage),
                "interior_coverage": float(interior_coverage),
                "type": "beam_candidate",
                "beam_signature": "dark_gutter_edge_pair",
                "height_gate": "raised_column",
                **height_band,
            }
        )
    bands = sorted(bands, key=lambda band: (str(band.get("axis", "x")), int(band.get("start", 0))))
    return bands


def build_beam_candidate_mask(shape, beam_candidate_bands, valid_mask=None):
    mask = np.zeros(shape[:2], dtype=bool)
    height, width = mask.shape[:2]
    for band in beam_candidate_bands or []:
        if band.get("axis") == "y":
            start = int(np.clip(int(band.get("start", 0)), 0, height - 1))
            end = int(np.clip(int(band.get("end", start)), start, height - 1))
            mask[start:end + 1, :] = True
        else:
            start = int(np.clip(int(band.get("start", 0)), 0, width - 1))
            end = int(np.clip(int(band.get("end", start)), start, width - 1))
            mask[:, start:end + 1] = True
    if valid_mask is not None:
        mask &= np.asarray(valid_mask, dtype=bool)
    return mask


def draw_line_family_mask(shape, line_families, thickness_px=5):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    height, width = mask.shape[:2]
    for family in line_families or []:
        line_angle = float(family.get("line_angle_deg", 0.0))
        normal = np.asarray(family.get("normal", [0.0, 1.0]), dtype=np.float32).reshape(2)
        direction = np.asarray([normal[1], -normal[0]], dtype=np.float32)
        line_extent = float(max(width, height) * 2)
        for rho in family.get("line_rhos", []):
            base_point = normal * float(rho)
            start = base_point - (direction * line_extent)
            end = base_point + (direction * line_extent)
            cv2.line(
                mask,
                (int(round(start[0])), int(round(start[1]))),
                (int(round(end[0])), int(round(end[1]))),
                255,
                max(1, int(thickness_px)),
                cv2.LINE_AA,
            )
        del line_angle
    return mask.astype(bool)


def _resolution_mm_per_px(rectified_geometry):
    try:
        resolution_mm_per_px = float(
            (rectified_geometry or {}).get("resolution_mm_per_px", DEFAULT_RESOLUTION_MM_PER_PX)
        )
    except (AttributeError, TypeError, ValueError):
        resolution_mm_per_px = DEFAULT_RESOLUTION_MM_PER_PX
    if not np.isfinite(resolution_mm_per_px) or resolution_mm_per_px <= 1e-6:
        resolution_mm_per_px = DEFAULT_RESOLUTION_MM_PER_PX
    return float(resolution_mm_per_px)


def _physical_spacing_px_range(rectified_geometry):
    resolution_mm_per_px = _resolution_mm_per_px(rectified_geometry)
    min_spacing_px = float(FULL_SCAN_REBAR_SPACING_MM_RANGE[0]) / resolution_mm_per_px
    max_spacing_px = float(FULL_SCAN_REBAR_SPACING_MM_RANGE[1]) / resolution_mm_per_px
    if max_spacing_px < min_spacing_px:
        min_spacing_px, max_spacing_px = max_spacing_px, min_spacing_px
    min_spacing_px = max(2.0, min_spacing_px)
    max_spacing_px = max(min_spacing_px + 1.0, max_spacing_px)
    return min_spacing_px, max_spacing_px


def _axis_physical_prior_for_length(axis_length, spacing_px_range):
    axis_length = max(0, int(round(axis_length)))
    min_spacing_px, _max_spacing_px = spacing_px_range
    max_visible_count = int(np.floor(max(0.0, float(axis_length - 1)) / max(min_spacing_px, 1.0))) + 1
    min_count = int(MIN_PHYSICAL_LATTICE_LINE_COUNT)
    max_count = max(min_count, int(max_visible_count))
    mode = "full_workspace" if max_visible_count >= int(FULL_WORKSPACE_MODE_MIN_VISIBLE_LINE_COUNT) else "visible_local"
    return {
        "mode": mode,
        "line_count_range": [int(min_count), int(max_count)],
        "max_visible_count": int(max_visible_count),
    }


def _score_profile_positions(profile, positions):
    support_profile = normalize_workspace_s2_profile_for_support(profile)
    scores = {}
    if support_profile.size == 0:
        return {float(position): 0.0 for position in positions}
    for position in positions:
        index = int(np.clip(int(round(float(position))), 0, support_profile.size - 1))
        scores[float(position)] = float(support_profile[index])
    return scores


def _select_physical_lattice_positions(
    profile,
    spacing_px_range,
    line_count_range,
    mode,
    min_peak_ratio=0.18,
):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    if profile.size == 0:
        return [], {}
    finite_profile = profile[np.isfinite(profile)]
    if finite_profile.size == 0:
        return [], {}
    profile_contrast = float(np.percentile(finite_profile, 95.0) - np.percentile(finite_profile, 5.0))
    profile_range = float(np.max(finite_profile) - np.min(finite_profile))
    if max(profile_contrast, profile_range) <= float(MIN_PHYSICAL_AXIS_PROFILE_CONTRAST):
        return [], {}

    min_spacing_px, max_spacing_px = spacing_px_range
    min_count, max_count = [int(value) for value in line_count_range]
    if max_count < min_count or max_count < 2:
        return [], {}

    support_profile = normalize_workspace_s2_profile_for_support(profile)
    if support_profile.size == 0 or float(np.max(support_profile)) <= 1e-6:
        return [], {}

    candidate_positions = []
    for peak_ratio in (min_peak_ratio, max(0.10, min_peak_ratio * 0.70), 0.08):
        candidate_positions = select_workspace_s2_peak_supported_line_positions(
            support_profile,
            list(range(profile.size)),
            search_radius_px=max(2, min(18, int(round(min_spacing_px * 0.45)))),
            min_spacing_px=max(2, int(round(min_spacing_px * 0.55))),
            min_peak_ratio=peak_ratio,
            duplicate_spacing_px=max(3, int(round(min_spacing_px * 0.38))),
        )
        if len(candidate_positions) >= min_count:
            break
    if len(candidate_positions) < min_count:
        return [], {}

    candidate_positions = sorted(float(position) for position in candidate_positions)
    max_count = min(int(max_count), int(len(candidate_positions)))
    if max_count < min_count:
        return [], {}
    candidate_scores = _score_profile_positions(support_profile, candidate_positions)
    candidate_array = np.asarray(candidate_positions, dtype=np.float32)
    spacing_candidates = set()
    for left_index, left_position in enumerate(candidate_positions):
        for right_position in candidate_positions[left_index + 1:]:
            spacing = float(right_position - left_position)
            if min_spacing_px <= spacing <= max_spacing_px:
                spacing_candidates.add(round(spacing, 3))
    for spacing in np.linspace(min_spacing_px, max_spacing_px, 18):
        spacing_candidates.add(round(float(spacing), 3))

    best_positions = []
    best_metadata = {}
    best_score = None

    for count in range(min_count, max_count + 1):
        for spacing in sorted(spacing_candidates):
            spacing = float(spacing)
            if spacing <= 1e-6:
                continue
            match_tolerance = max(3.0, min(10.0, spacing * 0.22))
            for anchor_position in candidate_positions:
                for anchor_slot in range(count):
                    first_position = float(anchor_position) - (float(anchor_slot) * spacing)
                    expected_positions = [first_position + (float(index) * spacing) for index in range(count)]
                    if expected_positions[0] < -match_tolerance or expected_positions[-1] > (profile.size - 1 + match_tolerance):
                        continue

                    selected_positions = []
                    used_candidate_indices = set()
                    errors = []
                    total_support = 0.0
                    for expected_position in expected_positions:
                        candidate_index = int(np.argmin(np.abs(candidate_array - float(expected_position))))
                        if candidate_index in used_candidate_indices:
                            break
                        candidate_position = float(candidate_array[candidate_index])
                        error = abs(candidate_position - float(expected_position))
                        if error > match_tolerance:
                            break
                        used_candidate_indices.add(candidate_index)
                        selected_positions.append(candidate_position)
                        errors.append(float(error))
                        total_support += float(candidate_scores.get(candidate_position, 0.0))
                    if len(selected_positions) != count:
                        continue

                    selected_positions = sorted(selected_positions)
                    diffs = np.diff(np.asarray(selected_positions, dtype=np.float32))
                    if diffs.size > 0:
                        if float(np.min(diffs)) < (min_spacing_px * 0.82):
                            continue
                        if float(np.max(diffs)) > (max_spacing_px * 1.18):
                            continue
                        spacing_cv = float(np.std(diffs) / max(float(np.mean(diffs)), 1e-6)) if diffs.size > 1 else 0.0
                    else:
                        spacing_cv = 0.0
                    mean_support = total_support / float(count)
                    mean_error_ratio = float(np.mean(errors)) / max(spacing, 1.0) if errors else 0.0
                    span_coverage = (
                        float(selected_positions[-1] - selected_positions[0]) / max(float(profile.size - 1), 1.0)
                        if len(selected_positions) > 1
                        else 0.0
                    )
                    count_fraction = float(count) / max(float(max_count), 1.0)
                    candidate_score = (
                        (mean_support * 4.0)
                        + (span_coverage * 0.85)
                        + (count_fraction * 0.20)
                        - (spacing_cv * 3.0)
                        - (mean_error_ratio * 2.0)
                    )
                    if (
                        best_score is None
                        or candidate_score > best_score
                        or (
                            abs(candidate_score - best_score) <= 1e-6
                            and (
                                -float(span_coverage),
                                -float(mean_support),
                                selected_positions,
                            )
                            < (
                                -float(best_metadata.get("span_coverage", 0.0)),
                                -float(best_metadata.get("mean_support", 0.0)),
                                best_positions,
                            )
                        )
                    ):
                        best_score = candidate_score
                        best_positions = [float(position) for position in selected_positions]
                        best_metadata = {
                            "physical_prior_mode": str(mode),
                            "selected_spacing_px": float(np.mean(diffs)) if diffs.size else float(spacing),
                            "line_count_range": [int(min_count), int(max_count)],
                            "candidate_count": int(len(candidate_positions)),
                            "mean_support": float(mean_support),
                            "mean_error_ratio": float(mean_error_ratio),
                            "span_coverage": float(span_coverage),
                        }

    return best_positions, best_metadata


def _build_physical_axis_family(
    response_map,
    valid_mask,
    axis,
    rectified_geometry,
    peak_min_ratio=0.18,
):
    response_map = np.asarray(response_map, dtype=np.float32)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    profile = build_workspace_s2_axis_profile(response_map, valid_mask.astype(np.uint8), axis=axis)
    spacing_px_range = _physical_spacing_px_range(rectified_geometry)
    axis_length = profile.size
    axis_prior = _axis_physical_prior_for_length(axis_length, spacing_px_range)
    line_positions, metadata = _select_physical_lattice_positions(
        profile,
        spacing_px_range=spacing_px_range,
        line_count_range=axis_prior["line_count_range"],
        mode=axis_prior["mode"],
        min_peak_ratio=peak_min_ratio,
    )
    if len(line_positions) < axis_prior["line_count_range"][0]:
        return None

    if axis == 1:
        line_angle_deg = 0.0
        normal = [0.0, 1.0]
        axis_orientation = "horizontal"
    else:
        line_angle_deg = 90.0
        normal = [1.0, 0.0]
        axis_orientation = "vertical"

    line_scores = _score_profile_positions(profile, line_positions)
    return {
        "axis_orientation": axis_orientation,
        "line_angle_deg": line_angle_deg,
        "normal": normal,
        "direction": [normal[1], -normal[0]],
        "rho_min": 0.0,
        "profile": np.asarray(profile, dtype=np.float32),
        "estimate": {
            "method": "physical_prior",
            "spacing_px_range": [float(spacing_px_range[0]), float(spacing_px_range[1])],
            "resolution_mm_per_px": _resolution_mm_per_px(rectified_geometry),
            "max_visible_count": int(axis_prior["max_visible_count"]),
        },
        "period_estimator": "physical_prior",
        "periodic_score": float(metadata.get("mean_support", 0.0)),
        "orientation_score": 1.0,
        "initial_positions": [int(round(position)) for position in line_positions],
        "peak_positions": [int(round(position)) for position in line_positions],
        "continuous_positions": [int(round(position)) for position in line_positions],
        "line_positions": [int(round(position)) for position in line_positions],
        "initial_rhos": [float(position) for position in line_positions],
        "peak_rhos": [float(position) for position in line_positions],
        "continuous_rhos": [float(position) for position in line_positions],
        "continuous_scores": line_scores,
        "line_rhos": [float(position) for position in line_positions],
        "physical_prior_mode": metadata.get("physical_prior_mode", axis_prior["mode"]),
        "physical_prior": metadata,
    }


def _build_physical_axis_aligned_line_families(
    response_map,
    valid_mask,
    rectified_geometry,
    peak_min_ratio=0.18,
):
    families = []
    for axis in (1, 0):
        family = _build_physical_axis_family(
            response_map,
            valid_mask,
            axis=axis,
            rectified_geometry=rectified_geometry,
            peak_min_ratio=peak_min_ratio,
        )
        if family is None:
            return []
        families.append(family)
    for family_index, family in enumerate(families):
        family["family_index"] = family_index
    return families


def _line_family_span_coverage(family):
    line_rhos = []
    for rho in (family or {}).get("line_rhos", []):
        try:
            rho_value = float(rho)
        except (TypeError, ValueError):
            continue
        if np.isfinite(rho_value):
            line_rhos.append(rho_value)
    if len(line_rhos) < 2:
        return 0.0
    profile = np.asarray((family or {}).get("profile", []), dtype=np.float32).reshape(-1)
    axis_length = int(profile.size)
    if axis_length <= 1:
        return 0.0
    return float((max(line_rhos) - min(line_rhos)) / max(float(axis_length - 1), 1.0))


def _line_family_count_fraction(family):
    line_count = len((family or {}).get("line_rhos", []))
    try:
        max_visible_count = int(((family or {}).get("estimate") or {}).get("max_visible_count", line_count))
    except (TypeError, ValueError):
        max_visible_count = line_count
    if max_visible_count <= 0:
        return 0.0
    return float(min(float(line_count) / float(max_visible_count), 1.0))


def _full_workspace_family_has_enough_coverage(family):
    if str((family or {}).get("physical_prior_mode", "")) != "full_workspace":
        return True
    line_count = len((family or {}).get("line_rhos", []))
    try:
        max_visible_count = int(((family or {}).get("estimate") or {}).get("max_visible_count", line_count))
    except (TypeError, ValueError):
        max_visible_count = line_count
    min_required_count = max(
        int(MIN_PHYSICAL_LATTICE_LINE_COUNT),
        int(np.ceil(float(max_visible_count) * float(FULL_WORKSPACE_MIN_LINE_COUNT_FRACTION))),
    )
    if line_count < min_required_count:
        return False
    return _line_family_span_coverage(family) >= float(FULL_WORKSPACE_MIN_AXIS_COVERAGE_RATIO)


def _score_physical_line_families(line_families):
    if len(line_families or []) < 2:
        return -float("inf")
    counts = [len(family.get("line_rhos", [])) for family in line_families[:2]]
    if min(counts) < 2:
        return -float("inf")
    if any(not _full_workspace_family_has_enough_coverage(family) for family in line_families[:2]):
        return -float("inf")
    support_scores = [
        float((family.get("physical_prior") or {}).get("mean_support", 0.0))
        for family in line_families[:2]
    ]
    span_scores = [_line_family_span_coverage(family) for family in line_families[:2]]
    count_fraction_scores = [_line_family_count_fraction(family) for family in line_families[:2]]
    balance_penalty = (
        abs(float(counts[0]) - float(counts[1])) /
        max(float(max(counts)), 1.0)
    ) * 0.60
    return float(
        (sum(support_scores) * 3.0)
        + (sum(span_scores) * 1.2)
        + (sum(count_fraction_scores) * 0.8)
        - balance_penalty
    )


def _build_best_physical_axis_aligned_line_families(
    response_candidates,
    valid_mask,
    rectified_geometry,
    peak_min_ratio=0.18,
):
    best_line_families = []
    best_source = None
    best_score = -float("inf")
    peak_ratios = []
    for ratio in (peak_min_ratio, max(0.12, peak_min_ratio * 0.78), 0.10):
        ratio = float(ratio)
        if not any(abs(ratio - existing_ratio) <= 1e-6 for existing_ratio in peak_ratios):
            peak_ratios.append(ratio)
    for ratio in peak_ratios:
        ratio_best_line_families = []
        ratio_best_source = None
        ratio_best_score = -float("inf")
        seen_response_ids = set()
        for source_name, response_map in response_candidates:
            if response_map is None:
                continue
            response_id = id(response_map)
            if response_id in seen_response_ids:
                continue
            seen_response_ids.add(response_id)
            line_families = _build_physical_axis_aligned_line_families(
                response_map,
                valid_mask,
                rectified_geometry,
                peak_min_ratio=ratio,
            )
            score = _score_physical_line_families(line_families)
            if score > ratio_best_score:
                ratio_best_score = score
                ratio_best_line_families = line_families[:2]
                ratio_best_source = str(source_name)
        if ratio_best_score > -float("inf"):
            best_score = ratio_best_score
            best_line_families = ratio_best_line_families
            best_source = ratio_best_source
            break
    if best_score <= -float("inf") or len(best_line_families) < 2:
        return [], None
    for family in best_line_families:
        family["physical_prior_source"] = best_source
        family["physical_prior_score"] = float(best_score)
    return best_line_families, best_source


def _full_workspace_expected(valid_mask, rectified_geometry):
    height, width = np.asarray(valid_mask).shape[:2]
    spacing_px_range = _physical_spacing_px_range(rectified_geometry)
    horizontal_prior = _axis_physical_prior_for_length(height, spacing_px_range)
    vertical_prior = _axis_physical_prior_for_length(width, spacing_px_range)
    return (
        horizontal_prior.get("mode") == "full_workspace"
        and vertical_prior.get("mode") == "full_workspace"
    )


def _build_modalities(result, threshold_percentile):
    valid_mask = _valid_mask_from_result(result)
    depth_response = build_depth_response(result)
    infrared_response = build_infrared_response(result)
    combined_response = build_combined_response(result)
    depth_gradient = build_depth_gradient_response(result)
    hessian_ridge = hessian_ridge_response(combined_response, valid_mask, sigma=1.6)
    frangi_like = multiscale_frangi_like_response(combined_response, valid_mask)
    fused_instance_response = _normalize_response(
        (0.50 * combined_response)
        + (0.22 * depth_response)
        + (0.16 * frangi_like)
        + (0.08 * depth_gradient)
        + (0.04 * hessian_ridge),
        valid_mask,
    )
    binary_candidate, binary_threshold = threshold_response(
        fused_instance_response,
        valid_mask,
        percentile=threshold_percentile,
    )
    skeleton = skeletonize_binary(binary_candidate)
    endpoint_count, junction_count = count_skeleton_nodes(skeleton)
    beam_candidate_bands = detect_beam_candidate_bands(
        fused_instance_response,
        binary_candidate,
        valid_mask,
        height_response=depth_response,
    )
    beam_candidate_mask = build_beam_candidate_mask(
        fused_instance_response.shape,
        beam_candidate_bands,
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
        "binary_candidate": binary_candidate,
        "binary_threshold": float(binary_threshold),
        "skeleton": skeleton,
        "instance_graph_endpoint_count": int(endpoint_count),
        "instance_graph_junction_count": int(junction_count),
        "beam_candidate_bands": beam_candidate_bands,
        "beam_candidate_mask": beam_candidate_mask,
        "beam_candidate_pixels": int(np.count_nonzero(beam_candidate_mask)),
    }


def _build_completed_surface(result, modalities, min_period, max_period):
    valid_mask = _valid_mask_from_result(result)
    rectified_geometry = result.get("rectified_geometry") or {}
    physical_candidates = [
        ("fused_instance_response", modalities.get("fused_instance_response")),
        ("frangi_like", modalities.get("frangi_like")),
        ("hessian_ridge", modalities.get("hessian_ridge")),
        ("depth_gradient", modalities.get("depth_gradient")),
        ("infrared_response", modalities.get("infrared_response")),
        ("combined_response", modalities.get("combined_response")),
        ("depth_response", modalities.get("depth_response")),
    ]
    base_line_families, base_physical_source = _build_best_physical_axis_aligned_line_families(
        physical_candidates,
        valid_mask,
        rectified_geometry,
        peak_min_ratio=0.18,
    )
    if len(base_line_families) < 2:
        base_line_families = []
        base_physical_source = "physical_prior_unresolved"
    line_support_mask = draw_line_family_mask(
        modalities["binary_candidate"].shape,
        base_line_families,
        thickness_px=5,
    )
    completed_surface_mask = (modalities["binary_candidate"] | line_support_mask) & valid_mask
    completed_surface_response = _normalize_response(
        (0.56 * modalities["fused_instance_response"])
        + (0.26 * modalities["frangi_like"])
        + (0.18 * line_support_mask.astype(np.float32)),
        valid_mask,
    )
    completed_candidates = [
        ("completed_surface_response", completed_surface_response),
        ("frangi_like", modalities.get("frangi_like")),
        ("hessian_ridge", modalities.get("hessian_ridge")),
        ("depth_gradient", modalities.get("depth_gradient")),
        ("infrared_response", modalities.get("infrared_response")),
        ("fused_instance_response", modalities.get("fused_instance_response")),
        ("combined_response", modalities.get("combined_response")),
        ("depth_response", modalities.get("depth_response")),
    ]
    completed_line_families, completed_physical_source = _build_best_physical_axis_aligned_line_families(
        completed_candidates,
        valid_mask,
        rectified_geometry,
        peak_min_ratio=0.16,
    )
    if len(completed_line_families) < 2:
        completed_line_families = []
        completed_physical_source = "physical_prior_unresolved"
    if (
        _full_workspace_expected(valid_mask, rectified_geometry)
        and any(family.get("physical_prior_mode") != "full_workspace" for family in completed_line_families[:2])
    ):
        completed_line_families = []
        completed_physical_source = "physical_prior_unresolved"
    return {
        "base_line_families": base_line_families[:2],
        "completed_line_families": completed_line_families[:2],
        "line_support_mask": line_support_mask,
        "completed_surface_mask": completed_surface_mask,
        "completed_surface_response": completed_surface_response,
        "physical_spacing_px_range": _physical_spacing_px_range(rectified_geometry),
        "physical_resolution_mm_per_px": _resolution_mm_per_px(rectified_geometry),
        "base_physical_source": base_physical_source,
        "completed_physical_source": completed_physical_source,
    }


def _axis_positions_from_families(line_families):
    horizontal_lines = []
    vertical_lines = []
    for family in line_families or []:
        orientation = family.get("axis_orientation")
        line_rhos = [float(rho) for rho in family.get("line_rhos", [])]
        if orientation == "vertical" or abs(float(family.get("line_angle_deg", 0.0)) - 90.0) < 20.0:
            vertical_lines = line_rhos
        else:
            horizontal_lines = line_rhos
    return vertical_lines, horizontal_lines


def _filter_beam_candidate_bands_by_lattice_context(beam_candidate_bands, line_families, image_width):
    vertical_lines, _horizontal_lines = _axis_positions_from_families(line_families)
    vertical_lines = sorted(
        float(line)
        for line in vertical_lines
        if np.isfinite(float(line)) and 0.0 <= float(line) <= float(max(0, int(image_width) - 1))
    )
    if len(vertical_lines) < 3:
        return list(beam_candidate_bands or []), 0

    spacings = np.diff(np.asarray(vertical_lines, dtype=np.float32))
    spacings = spacings[spacings > 1.0]
    if spacings.size == 0:
        return list(beam_candidate_bands or []), 0
    median_spacing = float(np.median(spacings))
    if not np.isfinite(median_spacing) or median_spacing <= 1.0:
        return list(beam_candidate_bands or []), 0

    line_tolerance_px = max(4.0, median_spacing * 0.22)
    midpoint_tolerance_px = max(5.0, median_spacing * 0.30)
    min_context_gap_px = median_spacing * 0.55
    max_context_gap_px = median_spacing * 1.65

    accepted_bands = []
    rejected_count = 0
    vertical_array = np.asarray(vertical_lines, dtype=np.float32)
    for band in beam_candidate_bands or []:
        if band.get("axis", "x") != "x":
            accepted_bands.append(dict(band))
            continue

        start = float(band.get("start", 0.0))
        end = float(band.get("end", start))
        center = (start + end) * 0.5
        nearest_line_index = int(np.argmin(np.abs(vertical_array - center)))
        nearest_vertical_line = float(vertical_array[nearest_line_index])
        nearest_line_gap = float(abs(nearest_vertical_line - center))
        original_width = float(band.get("original_width", max(1.0, end - start + 1.0)))
        height_delta = float(band.get("height_delta", 0.0))
        interior_coverage = band.get("interior_coverage")
        interior_dark_enough = True
        if interior_coverage is not None:
            interior_dark_enough = float(interior_coverage) <= 0.12
        dark_gutter_line_signature = (
            band.get("beam_signature") == "dark_gutter_edge_pair"
            and original_width >= max(18.0, median_spacing * 0.55)
            and height_delta >= 0.055
            and interior_dark_enough
        )
        wide_continuous_line_signature = (
            band.get("beam_signature") == "wide_continuous_column"
            and original_width >= max(8.0, median_spacing * 0.24)
            and float(band.get("peak", 0.0)) >= 1.30
            and float(band.get("coverage", band.get("coverage_score", 0.0))) >= 0.34
            and float(band.get("structural_delta", 0.0)) >= 0.18
        )
        if nearest_line_gap <= line_tolerance_px:
            if dark_gutter_line_signature:
                accepted_band = dict(band)
                accepted_band.update(
                    {
                        "lattice_gate": "dark_gutter_over_vertical_rebar_line",
                        "nearest_vertical_line": float(nearest_vertical_line),
                        "nearest_vertical_line_gap": float(nearest_line_gap),
                        "lattice_median_spacing": float(median_spacing),
                    }
                )
                accepted_bands.append(accepted_band)
                continue
            if wide_continuous_line_signature:
                left_gap = None
                right_gap = None
                if nearest_line_index > 0:
                    left_gap = nearest_vertical_line - float(vertical_array[nearest_line_index - 1])
                if nearest_line_index + 1 < len(vertical_array):
                    right_gap = float(vertical_array[nearest_line_index + 1]) - nearest_vertical_line
                neighbor_gaps = [
                    float(gap)
                    for gap in (left_gap, right_gap)
                    if gap is not None and np.isfinite(float(gap)) and float(gap) > 1.0
                ]
                spacing_distorted = any(
                    gap <= (median_spacing * 0.90) or gap >= (median_spacing * 1.10)
                    for gap in neighbor_gaps
                )
                if spacing_distorted:
                    accepted_band = dict(band)
                    accepted_band.update(
                        {
                            "lattice_gate": "structural_beam_over_vertical_rebar_line",
                            "nearest_vertical_line": float(nearest_vertical_line),
                            "nearest_vertical_line_gap": float(nearest_line_gap),
                            "lattice_median_spacing": float(median_spacing),
                            "lattice_left_gap": float(left_gap) if left_gap is not None else None,
                            "lattice_right_gap": float(right_gap) if right_gap is not None else None,
                        }
                    )
                    accepted_bands.append(accepted_band)
                    continue
            rejected_count += 1
            continue

        right_index = int(np.searchsorted(vertical_array, center, side="right"))
        left_index = right_index - 1
        if left_index < 0 or right_index >= len(vertical_array):
            rejected_count += 1
            continue
        left_line = float(vertical_array[left_index])
        right_line = float(vertical_array[right_index])
        lattice_gap = right_line - left_line
        if lattice_gap < min_context_gap_px or lattice_gap > max_context_gap_px:
            rejected_count += 1
            continue

        midpoint_error = abs(center - ((left_line + right_line) * 0.5))
        if midpoint_error > max(midpoint_tolerance_px, lattice_gap * 0.30):
            rejected_count += 1
            continue

        accepted_band = dict(band)
        accepted_band.update(
            {
                "lattice_gate": "between_vertical_rebar_columns",
                "lattice_left_line": float(left_line),
                "lattice_right_line": float(right_line),
                "lattice_gap": float(lattice_gap),
                "lattice_midpoint_error": float(midpoint_error),
                "nearest_vertical_line_gap": float(nearest_line_gap),
            }
        )
        accepted_bands.append(accepted_band)

    return accepted_bands, int(rejected_count)


def _project_rectified_points_to_image(rectified_points, inverse_h):
    if not rectified_points:
        return []
    points = np.asarray(rectified_points, dtype=np.float32).reshape(-1, 1, 2)
    image_points = cv2.perspectiveTransform(points, np.asarray(inverse_h, dtype=np.float32)).reshape(-1, 2)
    return [[float(x_value), float(y_value)] for x_value, y_value in image_points]


def build_scan_surface_dp_result(
    result,
    threshold_percentile=83.0,
    min_period=10,
    max_period=30,
    enable_beam_exclusion=False,
    beam_exclusion_margin_mm=130.0,
):
    valid_mask = _valid_mask_from_result(result)
    rectified_geometry = result.get("rectified_geometry") or {}
    rectified_width = int(rectified_geometry.get("rectified_width", valid_mask.shape[1]))
    rectified_height = int(rectified_geometry.get("rectified_height", valid_mask.shape[0]))
    if rectified_width <= 0 or rectified_height <= 0 or np.count_nonzero(valid_mask) < 100:
        return {"success": False, "message": "rectified workspace is too small"}

    modalities = _build_modalities(result, threshold_percentile)
    surface = _build_completed_surface(result, modalities, min_period, max_period)
    line_families = surface["completed_line_families"]
    if len(line_families) < 2:
        return {
            "success": False,
            "message": "completed surface line families are insufficient",
            "diagnostics": {
                "instance_graph_endpoint_count": modalities["instance_graph_endpoint_count"],
                "instance_graph_junction_count": modalities["instance_graph_junction_count"],
                "beam_candidate_count": len(modalities.get("beam_candidate_bands", [])),
                "beam_candidate_pixels": int(modalities.get("beam_candidate_pixels", 0)),
            },
        }
    beam_candidate_bands, beam_candidate_lattice_rejected_count = _filter_beam_candidate_bands_by_lattice_context(
        modalities.get("beam_candidate_bands", []),
        line_families,
        rectified_width,
    )
    modalities["beam_candidate_bands"] = beam_candidate_bands
    modalities["beam_candidate_mask"] = build_beam_candidate_mask(
        modalities["fused_instance_response"].shape,
        beam_candidate_bands,
        valid_mask,
    )
    modalities["beam_candidate_pixels"] = int(np.count_nonzero(modalities["beam_candidate_mask"]))
    modalities["beam_candidate_lattice_rejected_count"] = int(beam_candidate_lattice_rejected_count)

    beam_candidate_margin_mask = np.zeros_like(valid_mask, dtype=bool)
    curve_trace_mask = valid_mask
    if bool(enable_beam_exclusion):
        beam_candidate_margin_mask = expand_workspace_s2_exclusion_mask_by_metric_margin(
            modalities.get("beam_candidate_mask", np.zeros_like(valid_mask, dtype=bool)),
            rectified_geometry,
            margin_mm=float(beam_exclusion_margin_mm),
        )
        candidate_curve_trace_mask = valid_mask & (~beam_candidate_margin_mask)
        if np.any(candidate_curve_trace_mask):
            curve_trace_mask = candidate_curve_trace_mask

    curved_families = build_workspace_s2_curved_line_families(
        surface["completed_surface_response"],
        curve_trace_mask,
        line_families,
        trace_method="dynamic_programming",
        score_mode="response",
        search_radius_px=9,
        sample_step_px=4,
        min_response_ratio=0.15,
        smoothing_window_samples=5,
        smoothness_weight=0.12,
    )
    rectified_intersections = intersect_workspace_s2_curved_line_families(
        curved_families[0],
        curved_families[1],
        rectified_width,
        rectified_height,
    )
    if not rectified_intersections:
        rectified_intersections = intersect_workspace_s2_oriented_line_families(
            line_families[0],
            line_families[1],
            rectified_width,
            rectified_height,
        )
    if not rectified_intersections:
        return {
            "success": False,
            "message": "surface DP produced no intersections",
            "diagnostics": {
                "instance_graph_endpoint_count": modalities["instance_graph_endpoint_count"],
                "instance_graph_junction_count": modalities["instance_graph_junction_count"],
                "beam_candidate_count": len(modalities.get("beam_candidate_bands", [])),
                "beam_candidate_pixels": int(modalities.get("beam_candidate_pixels", 0)),
                "beam_candidate_lattice_rejected_count": int(
                    modalities.get("beam_candidate_lattice_rejected_count", 0)
                ),
            },
        }

    raw_rectified_intersection_count = len(rectified_intersections)
    beam_filtered_point_count = 0
    if bool(enable_beam_exclusion):
        rectified_intersections = filter_workspace_s2_rectified_points_outside_mask(
            rectified_intersections,
            beam_candidate_margin_mask,
            margin_px=0,
        )
        beam_filtered_point_count = raw_rectified_intersection_count - len(rectified_intersections)
        if not rectified_intersections:
            return {
                "success": False,
                "message": "surface DP beam exclusion removed all intersections",
                "diagnostics": {
                    "instance_graph_endpoint_count": modalities["instance_graph_endpoint_count"],
                    "instance_graph_junction_count": modalities["instance_graph_junction_count"],
                    "beam_candidate_count": len(modalities.get("beam_candidate_bands", [])),
                    "beam_candidate_pixels": int(modalities.get("beam_candidate_pixels", 0)),
                    "beam_candidate_lattice_rejected_count": int(
                        modalities.get("beam_candidate_lattice_rejected_count", 0)
                    ),
                    "beam_exclusion_enabled": True,
                    "beam_exclusion_margin_mm": float(beam_exclusion_margin_mm),
                    "beam_filtered_point_count": int(beam_filtered_point_count),
                    "beam_candidate_13cm_pixels": int(np.count_nonzero(beam_candidate_margin_mask)),
                },
            }

    inverse_h = rectified_geometry.get("inverse_h")
    image_intersections = _project_rectified_points_to_image(rectified_intersections, inverse_h) if inverse_h is not None else []
    vertical_lines, horizontal_lines = _axis_positions_from_families(line_families)
    completed_scores = [
        _response_value_at(surface["completed_surface_response"], point)
        for point in rectified_intersections
    ]
    physical_prior_modes = [
        str(family.get("physical_prior_mode", "physical_prior_unset"))
        for family in line_families
    ]
    return {
        "success": True,
        "message": "surface DP finished",
        "variant_id": "surface_dp_curve",
        "primary_point_source": "dp_curve_intersections",
        "rectified_intersections": rectified_intersections,
        "image_intersections": image_intersections,
        "line_counts": [len(family.get("line_rhos", [])) for family in line_families],
        "vertical_lines": vertical_lines,
        "horizontal_lines": horizontal_lines,
        "line_families": line_families,
        "curved_families": curved_families,
        "modalities": modalities,
        "surface": surface,
        "beam_candidate_bands": modalities.get("beam_candidate_bands", []),
        "beam_candidate_mask": modalities.get("beam_candidate_mask"),
        "beam_candidate_13cm_mask": beam_candidate_margin_mask,
        "completed_surface_response": surface["completed_surface_response"],
        "completed_surface_mask": surface["completed_surface_mask"],
        "mean_completed_surface_score": float(np.mean(completed_scores)) if completed_scores else 0.0,
        "diagnostics": {
            "binary_threshold": modalities["binary_threshold"],
            "binary_candidate_pixels": int(np.count_nonzero(modalities["binary_candidate"])),
            "completed_surface_pixels": int(np.count_nonzero(surface["completed_surface_mask"])),
            "instance_graph_endpoint_count": modalities["instance_graph_endpoint_count"],
            "instance_graph_junction_count": modalities["instance_graph_junction_count"],
            "beam_candidate_count": len(modalities.get("beam_candidate_bands", [])),
            "beam_candidate_pixels": int(modalities.get("beam_candidate_pixels", 0)),
            "beam_candidate_lattice_rejected_count": int(
                modalities.get("beam_candidate_lattice_rejected_count", 0)
            ),
            "beam_exclusion_enabled": bool(enable_beam_exclusion),
            "beam_exclusion_margin_mm": float(beam_exclusion_margin_mm),
            "beam_filtered_point_count": int(beam_filtered_point_count),
            "beam_candidate_13cm_pixels": int(np.count_nonzero(beam_candidate_margin_mask)),
            "physical_prior_modes": physical_prior_modes,
            "physical_spacing_mm_range": [
                float(FULL_SCAN_REBAR_SPACING_MM_RANGE[0]),
                float(FULL_SCAN_REBAR_SPACING_MM_RANGE[1]),
            ],
            "physical_spacing_px_range": [
                float(surface["physical_spacing_px_range"][0]),
                float(surface["physical_spacing_px_range"][1]),
            ],
            "physical_resolution_mm_per_px": float(surface["physical_resolution_mm_per_px"]),
            "base_physical_source": surface.get("base_physical_source"),
            "completed_physical_source": surface.get("completed_physical_source"),
        },
    }
