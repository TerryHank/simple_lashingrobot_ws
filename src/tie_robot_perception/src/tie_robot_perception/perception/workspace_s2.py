import cv2
import numpy as np


MIN_WORKSPACE_S2_SUPPORTED_LINE_COUNT = 3
LEGACY_WORKSPACE_S2_PREFERRED_LATTICE_LINE_COUNT = 8
LEGACY_WORKSPACE_S2_SCORE_TARGET_MIN_POINTS = 42


def sort_polygon_points_clockwise(points):
    if len(points) != 4:
        return points

    points_array = np.asarray(points, dtype=np.float32)
    center = np.mean(points_array, axis=0)
    point_angles = np.arctan2(points_array[:, 1] - center[1], points_array[:, 0] - center[0])
    sorted_indices = np.argsort(point_angles)
    sorted_points = [points[int(index)] for index in sorted_indices]
    start_index = min(
        range(len(sorted_points)),
        key=lambda index: (
            sorted_points[index][0] + sorted_points[index][1],
            sorted_points[index][1],
            sorted_points[index][0],
        ),
    )
    return sorted_points[start_index:] + sorted_points[:start_index]


def sort_polygon_indices_clockwise(points):
    if len(points) != 4:
        return list(range(len(points)))

    points_array = np.asarray(points, dtype=np.float32)
    center = np.mean(points_array, axis=0)
    point_angles = np.arctan2(points_array[:, 1] - center[1], points_array[:, 0] - center[0])
    sorted_indices = list(np.argsort(point_angles))
    start_index = min(
        range(len(sorted_indices)),
        key=lambda index: (
            points[sorted_indices[index]][0] + points[sorted_indices[index]][1],
            points[sorted_indices[index]][1],
            points[sorted_indices[index]][0],
        ),
    )
    return sorted_indices[start_index:] + sorted_indices[:start_index]


def smooth_workspace_s2_profile(profile):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    if profile.size == 0:
        return profile

    kernel = np.array([1.0, 2.0, 3.0, 2.0, 1.0], dtype=np.float32)
    kernel /= np.sum(kernel)
    return np.convolve(profile, kernel, mode="same")


def estimate_workspace_s2_period_and_phase(profile, min_period=10, max_period=30):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    if profile.size == 0:
        return None

    finite_mask = np.isfinite(profile)
    if not np.any(finite_mask):
        return None

    if not np.all(finite_mask):
        fill_value = float(np.median(profile[finite_mask]))
        profile = np.where(finite_mask, profile, fill_value)

    smoothed_profile = smooth_workspace_s2_profile(profile)
    centered_profile = smoothed_profile - np.mean(smoothed_profile)
    if np.linalg.norm(centered_profile) <= 1e-6:
        return None

    lower_bound = max(2, int(min_period))
    upper_bound = min(int(max_period), max(profile.size // 2, lower_bound))
    if upper_bound < lower_bound:
        return None

    best_period = None
    best_score = None
    for candidate_period in range(lower_bound, upper_bound + 1):
        base_segment = centered_profile[:-candidate_period]
        shifted_segment = centered_profile[candidate_period:]
        denominator = np.linalg.norm(base_segment) * np.linalg.norm(shifted_segment)
        if denominator <= 1e-6:
            continue

        correlation_score = float(np.dot(base_segment, shifted_segment) / denominator)
        if (
            best_score is None
            or correlation_score > best_score
            or (
                abs(correlation_score - best_score) <= 1e-6
                and candidate_period < best_period
            )
        ):
            best_period = candidate_period
            best_score = correlation_score

    if best_period is None:
        return None

    best_phase = 0
    best_phase_score = None
    for candidate_phase in range(best_period):
        phase_samples = smoothed_profile[candidate_phase::best_period]
        if phase_samples.size == 0:
            continue

        phase_score = float(np.mean(phase_samples))
        if best_phase_score is None or phase_score > best_phase_score:
            best_phase = candidate_phase
            best_phase_score = phase_score

    return {
        "period": int(best_period),
        "phase": int(best_phase),
        "score": float(best_score if best_score is not None else 0.0),
        "phase_score": float(best_phase_score if best_phase_score is not None else 0.0),
        "profile": smoothed_profile,
    }


def estimate_workspace_s2_fft_period_and_phase(profile, min_period=10, max_period=30):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    if profile.size == 0:
        return None

    finite_mask = np.isfinite(profile)
    if not np.any(finite_mask):
        return None

    if not np.all(finite_mask):
        fill_value = float(np.median(profile[finite_mask]))
        profile = np.where(finite_mask, profile, fill_value)

    smoothed_profile = smooth_workspace_s2_profile(profile)
    centered_profile = smoothed_profile - np.mean(smoothed_profile)
    if np.linalg.norm(centered_profile) <= 1e-6:
        return None

    lower_bound = max(2, int(min_period))
    upper_bound = min(int(max_period), max(profile.size // 2, lower_bound))
    if upper_bound < lower_bound:
        return None

    window = np.hanning(profile.size).astype(np.float32)
    if not np.any(window > 0.0):
        window = np.ones_like(centered_profile, dtype=np.float32)
    spectrum = np.abs(np.fft.rfft(centered_profile * window)) ** 2
    if spectrum.size <= 1 or float(np.max(spectrum[1:])) <= 1e-9:
        return None

    baseline = float(np.median(smoothed_profile))
    response_scale = max(
        1e-6,
        float(np.percentile(smoothed_profile, 95.0) - np.percentile(smoothed_profile, 5.0)),
    )
    peak_power = float(np.max(spectrum[1:]))

    best = None
    for candidate_period in range(lower_bound, upper_bound + 1):
        frequency_bin_float = float(profile.size) / float(candidate_period)
        frequency_bin = int(round(frequency_bin_float))
        if frequency_bin <= 0 or frequency_bin >= spectrum.size:
            continue

        band_start = max(1, frequency_bin - 1)
        band_end = min(spectrum.size - 1, frequency_bin + 1)
        fft_score = float(np.max(spectrum[band_start:band_end + 1]) / max(peak_power, 1e-9))

        base_segment = centered_profile[:-candidate_period]
        shifted_segment = centered_profile[candidate_period:]
        denominator = np.linalg.norm(base_segment) * np.linalg.norm(shifted_segment)
        correlation_score = (
            float(np.dot(base_segment, shifted_segment) / denominator)
            if denominator > 1e-6
            else 0.0
        )

        best_phase = 0
        best_phase_score = None
        for candidate_phase in range(candidate_period):
            phase_samples = smoothed_profile[candidate_phase::candidate_period]
            if phase_samples.size == 0:
                continue

            phase_score = float(np.mean(phase_samples))
            if best_phase_score is None or phase_score > best_phase_score:
                best_phase = candidate_phase
                best_phase_score = phase_score

        phase_contrast = (
            (float(best_phase_score) - baseline) / response_scale
            if best_phase_score is not None
            else 0.0
        )
        candidate_score = (
            (0.55 * fft_score)
            + (0.35 * phase_contrast)
            + (0.10 * max(0.0, correlation_score))
        )
        if (
            best is None
            or candidate_score > best["score"]
            or (
                abs(candidate_score - best["score"]) <= 1e-6
                and candidate_period > best["period"]
            )
        ):
            best = {
                "period": int(candidate_period),
                "phase": int(best_phase),
                "score": float(candidate_score),
                "fft_score": float(fft_score),
                "correlation_score": float(correlation_score),
                "phase_score": float(best_phase_score if best_phase_score is not None else 0.0),
                "frequency_bin": int(frequency_bin),
                "frequency_bin_float": float(frequency_bin_float),
                "method": "fft",
                "profile": smoothed_profile,
                "spectrum": spectrum.astype(np.float32),
            }

    return best


def build_workspace_s2_line_positions(start_pixel, end_pixel, period_px, phase_px):
    start_pixel = int(round(start_pixel))
    end_pixel = int(round(end_pixel))
    period_px = int(round(period_px))
    phase_px = int(round(phase_px))
    if period_px <= 0 or end_pixel < start_pixel:
        return []

    first_position = start_pixel + (phase_px % period_px)
    return list(range(first_position, end_pixel + 1, period_px))


def refine_workspace_s2_line_positions_to_local_peaks(
    profile,
    line_positions,
    search_radius_px=4,
    min_spacing_px=8,
    target_edge_margin_ratio=0.55,
    edge_anchor_weight=0.25,
    auxiliary_profile=None,
    auxiliary_weight=0.0,
):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    if profile.size == 0 or not line_positions:
        return list(line_positions or [])

    finite_mask = np.isfinite(profile)
    if not np.any(finite_mask):
        return [int(round(position)) for position in line_positions]
    if not np.all(finite_mask):
        fill_value = float(np.median(profile[finite_mask]))
        profile = np.where(finite_mask, profile, fill_value).astype(np.float32)

    smoothed_profile = smooth_workspace_s2_profile(profile)
    auxiliary_smoothed_profile = None
    auxiliary_weight = max(0.0, float(auxiliary_weight))
    if auxiliary_profile is not None and auxiliary_weight > 0.0:
        auxiliary_profile = np.asarray(auxiliary_profile, dtype=np.float32).reshape(-1)
        auxiliary_finite_mask = np.isfinite(auxiliary_profile)
        if auxiliary_profile.size == profile.size and np.any(auxiliary_finite_mask):
            if not np.all(auxiliary_finite_mask):
                fill_value = float(np.median(auxiliary_profile[auxiliary_finite_mask]))
                auxiliary_profile = np.where(auxiliary_finite_mask, auxiliary_profile, fill_value).astype(np.float32)
            auxiliary_smoothed_profile = smooth_workspace_s2_profile(auxiliary_profile)

    search_radius_px = max(0, int(round(search_radius_px)))
    initial_positions = [int(round(position)) for position in line_positions]
    min_spacing_px = max(1, int(round(min_spacing_px)))
    if any(
        (right_position - left_position) < min_spacing_px
        for left_position, right_position in zip(initial_positions, initial_positions[1:])
    ):
        return [int(np.clip(position, 0, profile.size - 1)) for position in initial_positions]
    if len(initial_positions) >= 2:
        estimated_period = float(np.median(np.diff(initial_positions)))
    else:
        estimated_period = float(min_spacing_px)
    target_edge_margin = max(0.0, float(target_edge_margin_ratio) * max(1.0, estimated_period))
    edge_anchor_weight = max(0.0, float(edge_anchor_weight))

    best_offset = 0
    best_score = None
    for candidate_offset in range(-search_radius_px, search_radius_px + 1):
        shifted_positions = [
            position + candidate_offset
            for position in initial_positions
            if 0 <= position + candidate_offset < profile.size
        ]
        if not shifted_positions:
            continue

        shifted_scores = smoothed_profile[np.asarray(shifted_positions, dtype=np.int32)]
        response_score = float(np.mean(shifted_scores))
        auxiliary_score = 0.0
        if auxiliary_smoothed_profile is not None:
            auxiliary_scores = auxiliary_smoothed_profile[np.asarray(shifted_positions, dtype=np.int32)]
            auxiliary_score = float(np.mean(auxiliary_scores))
        left_margin = float(shifted_positions[0])
        right_margin = float((profile.size - 1) - shifted_positions[-1])
        edge_error = (
            abs(left_margin - target_edge_margin)
            + abs(right_margin - target_edge_margin)
        ) / max(1.0, estimated_period)
        candidate_score = response_score + (auxiliary_weight * auxiliary_score) - (edge_anchor_weight * edge_error)
        if (
            best_score is None
            or candidate_score > best_score
            or (
                abs(candidate_score - best_score) <= 1e-6
                and abs(candidate_offset) < abs(best_offset)
            )
        ):
            best_offset = candidate_offset
            best_score = candidate_score

    return [
        int(np.clip(position + best_offset, 0, profile.size - 1))
        for position in initial_positions
    ]


def normalize_workspace_s2_profile_for_support(profile):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    if profile.size == 0:
        return profile

    finite_mask = np.isfinite(profile)
    if not np.any(finite_mask):
        return np.zeros_like(profile, dtype=np.float32)
    if not np.all(finite_mask):
        fill_value = float(np.median(profile[finite_mask]))
        profile = np.where(finite_mask, profile, fill_value).astype(np.float32)

    smoothed_profile = smooth_workspace_s2_profile(profile)
    min_value = float(np.min(smoothed_profile))
    max_value = float(np.max(smoothed_profile))
    if max_value <= min_value + 1e-6:
        return np.zeros_like(smoothed_profile, dtype=np.float32)
    return ((smoothed_profile - min_value) / (max_value - min_value)).astype(np.float32)


def select_workspace_s2_peak_supported_line_positions(
    profile,
    line_positions,
    search_radius_px=4,
    min_spacing_px=8,
    min_peak_ratio=0.45,
    auxiliary_profile=None,
    auxiliary_weight=0.0,
    duplicate_spacing_px=None,
):
    support_profile = normalize_workspace_s2_profile_for_support(profile)
    if support_profile.size == 0 or not line_positions:
        return []

    auxiliary_weight = max(0.0, float(auxiliary_weight))
    if auxiliary_profile is not None and auxiliary_weight > 0.0:
        auxiliary_support = normalize_workspace_s2_profile_for_support(auxiliary_profile)
        if auxiliary_support.size == support_profile.size:
            support_profile = np.maximum(
                support_profile,
                np.clip(auxiliary_support * auxiliary_weight, 0.0, 1.0),
            ).astype(np.float32)

    search_radius_px = max(0, int(round(search_radius_px)))
    min_spacing_px = max(1, int(round(min_spacing_px)))
    if duplicate_spacing_px is None:
        duplicate_spacing_px = min_spacing_px
    duplicate_spacing_px = max(0.0, float(duplicate_spacing_px))
    min_peak_ratio = float(np.clip(min_peak_ratio, 0.0, 1.0))
    supported_candidates = []

    for line_position in line_positions:
        center_position = int(round(float(line_position)))
        window_start = max(0, center_position - search_radius_px)
        window_end = min(support_profile.size - 1, center_position + search_radius_px)
        if window_end < window_start:
            continue

        window = support_profile[window_start:window_end + 1]
        if window.size == 0:
            continue

        local_index = int(np.argmax(window))
        peak_position = int(window_start + local_index)
        peak_score = float(support_profile[peak_position])
        if peak_score < min_peak_ratio:
            continue
        supported_candidates.append((peak_position, peak_score))

    supported_candidates.sort(key=lambda item: (-item[1], item[0]))
    selected_positions = []
    for peak_position, _peak_score in supported_candidates:
        if any(
            abs(float(peak_position) - float(selected_position)) < duplicate_spacing_px
            for selected_position in selected_positions
        ):
            continue
        selected_positions.append(peak_position)

    return sorted(selected_positions)


def normalize_workspace_s2_response_for_line_support(response_map, workspace_mask=None):
    response_map = np.asarray(response_map, dtype=np.float32)
    if response_map.size == 0:
        return response_map

    finite_mask = np.isfinite(response_map)
    if workspace_mask is not None:
        workspace_mask = np.asarray(workspace_mask).astype(bool)
        if workspace_mask.shape == response_map.shape:
            finite_mask &= workspace_mask

    if not np.any(finite_mask):
        return np.zeros_like(response_map, dtype=np.float32)

    response_values = response_map[finite_mask]
    lower_bound = float(np.percentile(response_values, 5.0))
    upper_bound = float(np.percentile(response_values, 95.0))
    if upper_bound <= lower_bound + 1e-6:
        upper_bound = float(np.max(response_values))
        lower_bound = float(np.min(response_values))
    if upper_bound <= lower_bound + 1e-6:
        normalized = np.zeros_like(response_map, dtype=np.float32)
        normalized[finite_mask] = 1.0
        return normalized

    normalized = (response_map - lower_bound) / (upper_bound - lower_bound)
    normalized = np.clip(normalized, 0.0, 1.0).astype(np.float32)
    normalized[~finite_mask] = 0.0
    return normalized


def _workspace_s2_profile_components(profile, threshold):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    if profile.size == 0:
        return []

    threshold = float(threshold)
    components = []
    component_start = None
    for index, is_active in enumerate((profile >= threshold).tolist() + [False]):
        if is_active and component_start is None:
            component_start = index
        if not is_active and component_start is not None:
            component_end = index - 1
            component_values = profile[component_start:component_end + 1]
            peak_value = float(np.max(component_values)) if component_values.size else 0.0
            components.append(
                {
                    "start": int(component_start),
                    "end": int(component_end),
                    "width": int(component_end - component_start + 1),
                    "peak": peak_value,
                }
            )
            component_start = None
    return components


def detect_workspace_s2_structural_edge_bands(
    response_map,
    workspace_mask=None,
    edge_margin_ratio=0.18,
    min_band_width_px=8,
    high_response_ratio=0.40,
    abnormal_peak_ratio=1.0,
    abnormal_width_ratio=1.1,
    allowed_axes=("x",),
):
    response_map = np.asarray(response_map, dtype=np.float32)
    if response_map.ndim != 2 or response_map.size == 0:
        return []

    if workspace_mask is None:
        valid_mask = np.ones_like(response_map, dtype=bool)
    else:
        valid_mask = np.asarray(workspace_mask).astype(bool)
        if valid_mask.shape != response_map.shape:
            valid_mask = np.ones_like(response_map, dtype=bool)
    valid_mask &= np.isfinite(response_map)
    if not np.any(valid_mask):
        return []

    support_map = normalize_workspace_s2_response_for_line_support(response_map, valid_mask)
    high_response_ratio = float(np.clip(high_response_ratio, 0.0, 1.0))
    edge_margin_ratio = float(np.clip(edge_margin_ratio, 0.02, 0.45))
    min_band_width_px = max(1, int(round(min_band_width_px)))
    abnormal_peak_ratio = max(1.0, float(abnormal_peak_ratio))
    abnormal_width_ratio = max(1.0, float(abnormal_width_ratio))
    allowed_axes = {str(axis_name).lower() for axis_name in (allowed_axes or ("x",))}
    beam_bands = []

    for axis in (0, 1):
        axis_name = "y" if axis == 0 else "x"
        if axis_name not in allowed_axes:
            continue
        if axis == 0:
            valid_counts = np.sum(valid_mask, axis=1).astype(np.float32)
            response_sums = np.sum(np.where(valid_mask, support_map, 0.0), axis=1).astype(np.float32)
            high_counts = np.sum((support_map >= high_response_ratio) & valid_mask, axis=1).astype(np.float32)
            length = response_map.shape[0]
        else:
            valid_counts = np.sum(valid_mask, axis=0).astype(np.float32)
            response_sums = np.sum(np.where(valid_mask, support_map, 0.0), axis=0).astype(np.float32)
            high_counts = np.sum((support_map >= high_response_ratio) & valid_mask, axis=0).astype(np.float32)
            length = response_map.shape[1]

        axis_valid = valid_counts > 0.0
        if length <= 0 or not np.any(axis_valid):
            continue

        response_profile = np.divide(
            response_sums,
            np.maximum(valid_counts, 1.0),
            out=np.zeros_like(response_sums, dtype=np.float32),
            where=valid_counts > 0.0,
        )
        high_coverage_profile = np.divide(
            high_counts,
            np.maximum(valid_counts, 1.0),
            out=np.zeros_like(high_counts, dtype=np.float32),
            where=valid_counts > 0.0,
        )
        structural_profile = (response_profile + high_coverage_profile).astype(np.float32)
        structural_profile = cv2.GaussianBlur(
            structural_profile.reshape(1, -1),
            (0, 0),
            sigmaX=2.0,
        ).reshape(-1)

        profile_values = structural_profile[axis_valid]
        if profile_values.size == 0:
            continue
        component_threshold = max(float(np.percentile(profile_values, 75.0)), 0.18)
        components = _workspace_s2_profile_components(structural_profile, component_threshold)
        if len(components) < 2:
            continue

        component_peaks = np.asarray([component["peak"] for component in components], dtype=np.float32)
        component_widths = np.asarray([component["width"] for component in components], dtype=np.float32)
        median_peak = float(np.median(component_peaks)) if component_peaks.size else 0.0
        median_width = float(np.median(component_widths)) if component_widths.size else 1.0
        edge_margin_px = max(min_band_width_px * 2, int(round(float(length) * edge_margin_ratio)))

        for component in components:
            touches_edge_area = component["start"] < edge_margin_px or component["end"] >= (length - edge_margin_px)
            if not touches_edge_area:
                continue

            required_width = max(float(min_band_width_px), median_width * abnormal_width_ratio)
            required_peak = max(0.45, median_peak * abnormal_peak_ratio)
            if float(component["width"]) < required_width or float(component["peak"]) < required_peak:
                continue

            expand_px = max(2, min(8, int(round(float(component["width"]) * 0.20))))
            start_index = max(0, int(component["start"]) - expand_px)
            end_index = min(length - 1, int(component["end"]) + expand_px)
            beam_bands.append(
                {
                    "axis": axis_name,
                    "side": "min" if component["start"] < edge_margin_px else "max",
                    "start": int(start_index),
                    "end": int(end_index),
                    "width": int(end_index - start_index + 1),
                    "peak": float(component["peak"]),
                    "component_start": int(component["start"]),
                    "component_end": int(component["end"]),
                }
            )

    return beam_bands


def build_workspace_s2_structural_edge_suppression_mask(
    response_map,
    workspace_mask=None,
    edge_margin_ratio=0.18,
    min_band_width_px=8,
    high_response_ratio=0.40,
    abnormal_peak_ratio=1.0,
    abnormal_width_ratio=1.1,
    allowed_axes=("x",),
):
    response_map = np.asarray(response_map, dtype=np.float32)
    if response_map.ndim != 2 or response_map.size == 0:
        return np.zeros(response_map.shape[:2], dtype=bool)

    if workspace_mask is None:
        valid_mask = np.ones_like(response_map, dtype=bool)
    else:
        valid_mask = np.asarray(workspace_mask).astype(bool)
        if valid_mask.shape != response_map.shape:
            valid_mask = np.ones_like(response_map, dtype=bool)
    valid_mask &= np.isfinite(response_map)
    if not np.any(valid_mask):
        return np.zeros_like(response_map, dtype=bool)

    suppression_mask = np.zeros_like(valid_mask, dtype=bool)
    beam_bands = detect_workspace_s2_structural_edge_bands(
        response_map,
        valid_mask,
        edge_margin_ratio=edge_margin_ratio,
        min_band_width_px=min_band_width_px,
        high_response_ratio=high_response_ratio,
        abnormal_peak_ratio=abnormal_peak_ratio,
        abnormal_width_ratio=abnormal_width_ratio,
        allowed_axes=allowed_axes,
    )
    for band in beam_bands:
        start_index = int(band["start"])
        end_index = int(band["end"])
        if band["axis"] == "y":
            suppression_mask[start_index:end_index + 1, :] = True
        else:
            suppression_mask[:, start_index:end_index + 1] = True

    return suppression_mask & valid_mask


def suppress_workspace_s2_structural_edge_response(
    response_map,
    workspace_mask=None,
    suppression_weight=0.06,
):
    response_map = np.asarray(response_map, dtype=np.float32)
    if response_map.ndim != 2 or response_map.size == 0:
        return response_map.copy()

    suppression_mask = build_workspace_s2_structural_edge_suppression_mask(response_map, workspace_mask)
    if not np.any(suppression_mask):
        return response_map.copy()

    suppressed_response = response_map.copy()
    suppressed_response[suppression_mask] *= float(np.clip(suppression_weight, 0.0, 1.0))
    return suppressed_response


def filter_workspace_s2_rectified_points_outside_mask(rectified_points, exclusion_mask, margin_px=0):
    exclusion_mask = np.asarray(exclusion_mask).astype(bool)
    if exclusion_mask.ndim != 2 or exclusion_mask.size == 0:
        return [tuple(float(value) for value in point) for point in rectified_points]

    height, width = exclusion_mask.shape[:2]
    margin_px = max(0, int(round(margin_px)))
    filtered_points = []
    for point in rectified_points:
        x_value = float(point[0])
        y_value = float(point[1])
        x_index = int(round(x_value))
        y_index = int(round(y_value))
        if x_index < 0 or x_index >= width or y_index < 0 or y_index >= height:
            continue

        x_start = max(0, x_index - margin_px)
        x_end = min(width - 1, x_index + margin_px)
        y_start = max(0, y_index - margin_px)
        y_end = min(height - 1, y_index + margin_px)
        if np.any(exclusion_mask[y_start:y_end + 1, x_start:x_end + 1]):
            continue
        filtered_points.append((x_value, y_value))

    return filtered_points


def workspace_s2_metric_margin_to_pixels(
    rectified_geometry=None,
    margin_mm=0.0,
    fallback_resolution_mm_per_px=5.0,
):
    margin_mm = max(0.0, float(margin_mm))
    if margin_mm <= 0.0:
        return 0

    resolution_mm_per_px = fallback_resolution_mm_per_px
    if rectified_geometry is not None:
        try:
            resolution_mm_per_px = float(rectified_geometry.get("resolution_mm_per_px", fallback_resolution_mm_per_px))
        except (AttributeError, TypeError, ValueError):
            resolution_mm_per_px = fallback_resolution_mm_per_px
    resolution_mm_per_px = max(1e-6, float(resolution_mm_per_px))
    return int(round(margin_mm / resolution_mm_per_px))


def expand_workspace_s2_exclusion_mask_by_metric_margin(
    exclusion_mask,
    rectified_geometry=None,
    margin_mm=0.0,
    fallback_resolution_mm_per_px=5.0,
):
    exclusion_mask = np.asarray(exclusion_mask).astype(bool)
    if exclusion_mask.ndim != 2 or exclusion_mask.size == 0:
        return exclusion_mask
    if not np.any(exclusion_mask):
        return exclusion_mask.copy()

    margin_px = workspace_s2_metric_margin_to_pixels(
        rectified_geometry,
        margin_mm=margin_mm,
        fallback_resolution_mm_per_px=fallback_resolution_mm_per_px,
    )
    if margin_px <= 0:
        return exclusion_mask.copy()

    kernel_size = (margin_px * 2) + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    expanded_mask = cv2.dilate(exclusion_mask.astype(np.uint8), kernel, iterations=1)
    return expanded_mask.astype(bool)


def filter_workspace_s2_line_rhos_by_mask_overlap(
    line_rhos,
    line_angle_deg,
    normal,
    exclusion_mask,
    max_overlap_fraction=0.35,
    sample_step_px=2.0,
):
    exclusion_mask = np.asarray(exclusion_mask).astype(bool)
    if exclusion_mask.ndim != 2 or exclusion_mask.size == 0:
        return [float(rho) for rho in line_rhos]

    height, width = exclusion_mask.shape[:2]
    normal = np.asarray(normal, dtype=np.float32).reshape(2)
    direction = workspace_s2_line_direction_from_angle(line_angle_deg)
    max_overlap_fraction = float(np.clip(max_overlap_fraction, 0.0, 1.0))
    sample_step_px = max(1.0, float(sample_step_px))
    filtered_rhos = []

    for line_rho in line_rhos:
        point_on_line = normal * float(line_rho)
        t_range = _workspace_s2_line_rect_t_range(point_on_line, direction, width, height)
        if t_range is None:
            filtered_rhos.append(float(line_rho))
            continue

        t_min, t_max = t_range
        t_values = np.arange(np.ceil(t_min), np.floor(t_max) + 1.0, sample_step_px, dtype=np.float32)
        if t_values.size == 0:
            filtered_rhos.append(float(line_rho))
            continue

        sample_points = point_on_line.reshape(1, 2) + (t_values.reshape(-1, 1) * direction.reshape(1, 2))
        sample_x = np.rint(sample_points[:, 0]).astype(np.int32)
        sample_y = np.rint(sample_points[:, 1]).astype(np.int32)
        inside_mask = (
            (sample_x >= 0)
            & (sample_x < width)
            & (sample_y >= 0)
            & (sample_y < height)
        )
        if not np.any(inside_mask):
            filtered_rhos.append(float(line_rho))
            continue

        sample_x = sample_x[inside_mask]
        sample_y = sample_y[inside_mask]
        overlap_fraction = float(np.count_nonzero(exclusion_mask[sample_y, sample_x])) / float(sample_x.size)
        if overlap_fraction <= max_overlap_fraction:
            filtered_rhos.append(float(line_rho))

    return filtered_rhos


def _workspace_s2_centered_ridge_contrast(
    response_window,
    mask_window,
    coordinate_values,
    best_position,
    search_radius_px,
):
    valid_mask = np.asarray(mask_window).astype(bool)
    if response_window.size == 0 or not np.any(valid_mask):
        return 0.0

    coordinate_values = np.asarray(coordinate_values, dtype=np.float32)
    response_window = np.asarray(response_window, dtype=np.float32)
    distances = np.abs(coordinate_values - float(best_position))
    core_radius = max(1.0, min(3.0, float(search_radius_px) * 0.25))
    side_width = max(1.0, min(3.0, float(search_radius_px) * 0.25))
    side_inner_radius = core_radius + 0.5
    side_outer_radius = side_inner_radius + side_width

    core_mask = valid_mask & (distances <= core_radius)
    if not np.any(core_mask):
        return 0.0

    left_side_mask = (
        valid_mask
        & (coordinate_values < float(best_position))
        & (distances >= side_inner_radius)
        & (distances <= side_outer_radius)
    )
    right_side_mask = (
        valid_mask
        & (coordinate_values > float(best_position))
        & (distances >= side_inner_radius)
        & (distances <= side_outer_radius)
    )
    if not np.any(left_side_mask) or not np.any(right_side_mask):
        return 0.0

    core_score = float(np.max(response_window[core_mask]))
    if core_score <= 1e-6:
        return 0.0

    left_side_score = float(np.max(response_window[left_side_mask]))
    right_side_score = float(np.max(response_window[right_side_mask]))
    shoulder_score = max(left_side_score, right_side_score)
    return float(np.clip((core_score - shoulder_score) / core_score, 0.0, 1.0))


def _workspace_s2_line_support_samples(
    support_map,
    support_mask,
    center_position,
    orientation,
    search_radius_px,
    min_response_ratio,
):
    height, width = support_map.shape[:2]
    if orientation == "vertical":
        coordinate_size = width
        length_size = height
    elif orientation == "horizontal":
        coordinate_size = height
        length_size = width
    else:
        raise ValueError("orientation must be 'vertical' or 'horizontal'")

    center_position = int(round(float(center_position)))
    window_start = max(0, center_position - search_radius_px)
    window_end = min(coordinate_size - 1, center_position + search_radius_px)
    if window_end < window_start:
        return None

    sample_indices = []
    sample_scores = []
    sample_positions = []
    sample_ridge_contrasts = []
    coordinate_values = np.arange(window_start, window_end + 1, dtype=np.float32)
    for length_index in range(length_size):
        if orientation == "vertical":
            response_window = support_map[length_index, window_start:window_end + 1]
            mask_window = support_mask[length_index, window_start:window_end + 1]
        else:
            response_window = support_map[window_start:window_end + 1, length_index]
            mask_window = support_mask[window_start:window_end + 1, length_index]

        if response_window.size == 0 or not np.any(mask_window):
            continue

        masked_response = np.where(mask_window, response_window, 0.0)
        max_score = float(np.max(masked_response))
        if max_score <= 0.0:
            best_position = float(center_position)
        else:
            weights = np.where(
                mask_window,
                np.maximum(masked_response - (float(min_response_ratio) * 0.5), 0.0),
                0.0,
            )
            weight_sum = float(np.sum(weights))
            if weight_sum > 1e-6:
                best_position = float(np.sum(coordinate_values * weights) / weight_sum)
            else:
                best_position = float(coordinate_values[int(np.argmax(masked_response))])

        ridge_contrast = _workspace_s2_centered_ridge_contrast(
            response_window,
            mask_window,
            coordinate_values,
            best_position,
            search_radius_px,
        )
        sample_indices.append(length_index)
        sample_scores.append(max_score)
        sample_positions.append(best_position)
        sample_ridge_contrasts.append(ridge_contrast)

    if not sample_indices:
        return None
    return (
        np.asarray(sample_indices, dtype=np.int32),
        np.asarray(sample_scores, dtype=np.float32),
        np.asarray(sample_positions, dtype=np.float32),
        np.asarray(sample_ridge_contrasts, dtype=np.float32),
        int(length_size),
    )


def select_workspace_s2_continuous_line_positions(
    response_map,
    workspace_mask,
    line_positions,
    orientation,
    search_radius_px=4,
    min_spacing_px=8,
    min_response_ratio=0.35,
    segment_count=12,
    min_segment_coverage=0.55,
    min_segment_sample_fraction=0.15,
    min_ridge_contrast_ratio=0.18,
    max_position_std_px=None,
    duplicate_spacing_px=None,
):
    if not line_positions:
        return []

    support_map = normalize_workspace_s2_response_for_line_support(response_map, workspace_mask)
    if support_map.ndim != 2 or support_map.size == 0:
        return []

    if workspace_mask is None:
        support_mask = np.ones_like(support_map, dtype=bool)
    else:
        support_mask = np.asarray(workspace_mask).astype(bool)
        if support_mask.shape != support_map.shape:
            support_mask = np.ones_like(support_map, dtype=bool)

    search_radius_px = max(0, int(round(search_radius_px)))
    min_spacing_px = max(1, int(round(min_spacing_px)))
    if duplicate_spacing_px is None:
        duplicate_spacing_px = min_spacing_px
    duplicate_spacing_px = max(0.0, float(duplicate_spacing_px))
    min_response_ratio = float(np.clip(min_response_ratio, 0.0, 1.0))
    segment_count = max(1, int(round(segment_count)))
    min_segment_coverage = float(np.clip(min_segment_coverage, 0.0, 1.0))
    min_segment_sample_fraction = float(np.clip(min_segment_sample_fraction, 0.0, 1.0))
    min_ridge_contrast_ratio = float(np.clip(min_ridge_contrast_ratio, 0.0, 1.0))
    if max_position_std_px is None:
        max_position_std_px = max(1.5, float(search_radius_px) * 0.9)
    max_position_std_px = max(0.0, float(max_position_std_px))

    supported_candidates = []
    for line_position in line_positions:
        samples = _workspace_s2_line_support_samples(
            support_map,
            support_mask,
            line_position,
            orientation,
            search_radius_px,
            min_response_ratio,
        )
        if samples is None:
            continue

        sample_indices, sample_scores, sample_positions, sample_ridge_contrasts, length_size = samples
        strong_sample_mask = sample_scores >= min_response_ratio
        ridge_sample_mask = strong_sample_mask & (sample_ridge_contrasts >= min_ridge_contrast_ratio)
        if not np.any(ridge_sample_mask):
            continue

        segment_edges = np.linspace(0, length_size, segment_count + 1)
        supported_segment_count = 0
        valid_segment_count = 0
        for segment_index in range(segment_count):
            segment_start = segment_edges[segment_index]
            segment_end = segment_edges[segment_index + 1]
            if segment_index + 1 == segment_count:
                segment_mask = (sample_indices >= segment_start) & (sample_indices <= segment_end)
            else:
                segment_mask = (sample_indices >= segment_start) & (sample_indices < segment_end)
            if not np.any(segment_mask):
                continue

            valid_segment_count += 1
            segment_scores = sample_scores[segment_mask]
            segment_ridge_contrasts = sample_ridge_contrasts[segment_mask]
            ridge_like_mask = (
                (segment_scores >= min_response_ratio)
                & (segment_ridge_contrasts >= min_ridge_contrast_ratio)
            )
            strong_fraction = float(np.count_nonzero(ridge_like_mask)) / float(segment_scores.size)
            if strong_fraction >= min_segment_sample_fraction:
                supported_segment_count += 1

        if valid_segment_count == 0:
            continue

        segment_coverage = float(supported_segment_count) / float(valid_segment_count)
        if segment_coverage < min_segment_coverage:
            continue

        strong_positions = sample_positions[ridge_sample_mask]
        position_std = float(np.std(strong_positions)) if strong_positions.size > 1 else 0.0
        if position_std > max_position_std_px:
            continue

        snapped_position = int(round(float(np.median(strong_positions))))
        mean_score = float(np.mean(sample_scores[ridge_sample_mask]))
        continuity_score = segment_coverage + (0.25 * mean_score) - (0.03 * position_std)
        supported_candidates.append((snapped_position, continuity_score))

    supported_candidates.sort(key=lambda item: (-item[1], item[0]))
    selected_positions = []
    for snapped_position, _score in supported_candidates:
        if any(
            abs(float(snapped_position) - float(selected_position)) < duplicate_spacing_px
            for selected_position in selected_positions
        ):
            continue
        selected_positions.append(snapped_position)

    return sorted(selected_positions)


def estimate_workspace_s2_line_spacing(line_positions):
    positions = sorted(int(round(float(position))) for position in line_positions)
    if len(positions) < 2:
        return None

    differences = np.diff(np.asarray(positions, dtype=np.float32))
    differences = differences[differences > 0.0]
    if differences.size == 0:
        return None

    sorted_differences = np.sort(differences)
    upper_half = sorted_differences[sorted_differences.size // 2:]
    if upper_half.size == 0:
        upper_half = sorted_differences
    return float(np.median(upper_half))


def prune_workspace_s2_line_positions_by_spacing(line_positions, min_spacing_ratio=0.65):
    positions = sorted(int(round(float(position))) for position in line_positions)
    if len(positions) < 3:
        return positions

    min_spacing_ratio = max(0.0, float(min_spacing_ratio))
    while len(positions) >= 3:
        reference_spacing = estimate_workspace_s2_line_spacing(positions)
        if reference_spacing is None or reference_spacing <= 1e-6:
            break

        min_allowed_spacing = reference_spacing * min_spacing_ratio
        differences = np.diff(np.asarray(positions, dtype=np.float32))
        close_pair_indices = np.where(differences < min_allowed_spacing)[0]
        if close_pair_indices.size == 0:
            break

        pair_index = int(close_pair_indices[0])
        left_index = pair_index
        right_index = pair_index + 1
        if left_index == 0:
            remove_index = left_index
        elif right_index == len(positions) - 1:
            remove_index = right_index
        else:
            remove_left_gap = positions[right_index] - positions[left_index - 1]
            remove_right_gap = positions[right_index + 1] - positions[left_index]
            remove_left_error = abs(float(remove_left_gap) - reference_spacing)
            remove_right_error = abs(float(remove_right_gap) - reference_spacing)
            remove_index = left_index if remove_left_error <= remove_right_error else right_index

        positions.pop(remove_index)

    return positions


def prune_workspace_s2_line_positions_by_near_duplicate_spacing(line_positions, duplicate_spacing_px=4):
    positions = sorted(int(round(float(position))) for position in line_positions)
    if len(positions) < 2:
        return positions

    duplicate_spacing_px = max(0.0, float(duplicate_spacing_px))
    selected_positions = []
    for position in positions:
        if selected_positions and abs(float(position) - float(selected_positions[-1])) < duplicate_spacing_px:
            continue
        selected_positions.append(position)

    return selected_positions


def prune_workspace_s2_line_positions_by_scored_spacing(line_positions, line_scores=None, min_spacing_ratio=0.60):
    positions = sorted(int(round(float(position))) for position in line_positions)
    if len(positions) < 3:
        return positions

    normalized_scores = {}
    for key, value in (line_scores or {}).items():
        normalized_scores[int(round(float(key)))] = float(value)

    min_spacing_ratio = max(0.0, float(min_spacing_ratio))
    while len(positions) >= 3:
        reference_spacing = estimate_workspace_s2_line_spacing(positions)
        if reference_spacing is None or reference_spacing <= 1e-6:
            break

        min_allowed_spacing = reference_spacing * min_spacing_ratio
        differences = np.diff(np.asarray(positions, dtype=np.float32))
        close_pair_indices = np.where(differences < min_allowed_spacing)[0]
        if close_pair_indices.size == 0:
            break

        pair_index = int(close_pair_indices[0])
        left_index = pair_index
        right_index = pair_index + 1
        left_position = positions[left_index]
        right_position = positions[right_index]
        left_score = normalized_scores.get(left_position, 0.0)
        right_score = normalized_scores.get(right_position, 0.0)
        if abs(left_score - right_score) > 1e-6:
            remove_index = left_index if left_score < right_score else right_index
        elif left_index == 0:
            remove_index = left_index
        elif right_index == len(positions) - 1:
            remove_index = right_index
        else:
            remove_left_gap = positions[right_index] - positions[left_index - 1]
            remove_right_gap = positions[right_index + 1] - positions[left_index]
            remove_left_error = abs(float(remove_left_gap) - reference_spacing)
            remove_right_error = abs(float(remove_right_gap) - reference_spacing)
            remove_index = left_index if remove_left_error <= remove_right_error else right_index

        positions.pop(remove_index)

    return positions


def normalize_workspace_s2_angle_deg(angle_deg):
    return float(angle_deg) % 180.0


def workspace_s2_angle_delta_deg(left_angle, right_angle):
    delta = abs((normalize_workspace_s2_angle_deg(left_angle) - normalize_workspace_s2_angle_deg(right_angle)) % 180.0)
    return min(delta, 180.0 - delta)


def workspace_s2_line_direction_from_angle(line_angle_deg):
    line_angle_rad = np.deg2rad(normalize_workspace_s2_angle_deg(line_angle_deg))
    return np.asarray(
        [float(np.cos(line_angle_rad)), float(np.sin(line_angle_rad))],
        dtype=np.float32,
    )


def workspace_s2_line_normal_from_angle(line_angle_deg):
    line_angle_rad = np.deg2rad(normalize_workspace_s2_angle_deg(line_angle_deg))
    return np.asarray(
        [-float(np.sin(line_angle_rad)), float(np.cos(line_angle_rad))],
        dtype=np.float32,
    )


def build_workspace_s2_oriented_axis_profile(response_map, workspace_mask, line_angle_deg):
    response_map = np.asarray(response_map, dtype=np.float32)
    if response_map.ndim != 2 or response_map.size == 0:
        return {
            "profile": np.zeros(0, dtype=np.float32),
            "rho_min": 0.0,
            "rho_max": -1.0,
            "line_angle_deg": normalize_workspace_s2_angle_deg(line_angle_deg),
            "normal": workspace_s2_line_normal_from_angle(line_angle_deg).tolist(),
        }

    if workspace_mask is None:
        valid_mask = np.ones_like(response_map, dtype=bool)
    else:
        valid_mask = np.asarray(workspace_mask).astype(bool)
        if valid_mask.shape != response_map.shape:
            valid_mask = np.ones_like(response_map, dtype=bool)
    valid_mask &= np.isfinite(response_map)
    if not np.any(valid_mask):
        return {
            "profile": np.zeros(0, dtype=np.float32),
            "rho_min": 0.0,
            "rho_max": -1.0,
            "line_angle_deg": normalize_workspace_s2_angle_deg(line_angle_deg),
            "normal": workspace_s2_line_normal_from_angle(line_angle_deg).tolist(),
        }

    normal = workspace_s2_line_normal_from_angle(line_angle_deg)
    y_coords, x_coords = np.indices(response_map.shape, dtype=np.float32)
    rho_map = (normal[0] * x_coords) + (normal[1] * y_coords)
    valid_rhos = rho_map[valid_mask]
    rho_min = float(np.floor(np.min(valid_rhos)))
    rho_max = float(np.ceil(np.max(valid_rhos)))
    bin_count = max(0, int(round(rho_max - rho_min)) + 1)
    if bin_count <= 0:
        return {
            "profile": np.zeros(0, dtype=np.float32),
            "rho_min": rho_min,
            "rho_max": rho_max,
            "line_angle_deg": normalize_workspace_s2_angle_deg(line_angle_deg),
            "normal": normal.tolist(),
        }

    bin_indices = np.rint(valid_rhos - rho_min).astype(np.int32)
    bin_indices = np.clip(bin_indices, 0, bin_count - 1)
    weighted_sum = np.bincount(
        bin_indices,
        weights=response_map[valid_mask].astype(np.float64),
        minlength=bin_count,
    )
    valid_count = np.bincount(bin_indices, minlength=bin_count)
    profile = np.divide(
        weighted_sum,
        valid_count,
        out=np.zeros(bin_count, dtype=np.float64),
        where=valid_count > 0,
    ).astype(np.float32)
    return {
        "profile": profile,
        "rho_min": rho_min,
        "rho_max": rho_max,
        "line_angle_deg": normalize_workspace_s2_angle_deg(line_angle_deg),
        "normal": normal.tolist(),
    }


def build_workspace_s2_orientation_prior_scores(response_map, workspace_mask, angle_step_deg=2.0):
    response_map = np.asarray(response_map, dtype=np.float32)
    if response_map.ndim != 2 or response_map.size == 0:
        return {}

    if workspace_mask is None:
        valid_mask = np.ones_like(response_map, dtype=bool)
    else:
        valid_mask = np.asarray(workspace_mask).astype(bool)
        if valid_mask.shape != response_map.shape:
            valid_mask = np.ones_like(response_map, dtype=bool)
    valid_mask &= np.isfinite(response_map)
    if not np.any(valid_mask):
        return {}

    angle_step_deg = max(0.5, float(angle_step_deg))
    bin_count = max(1, int(round(180.0 / angle_step_deg)))
    gradient_x = cv2.Sobel(response_map, cv2.CV_32F, 1, 0, ksize=3)
    gradient_y = cv2.Sobel(response_map, cv2.CV_32F, 0, 1, ksize=3)
    gradient_magnitude = np.sqrt((gradient_x * gradient_x) + (gradient_y * gradient_y))
    finite_magnitudes = gradient_magnitude[valid_mask]
    if finite_magnitudes.size == 0 or float(np.max(finite_magnitudes)) <= 1e-6:
        return {}

    magnitude_threshold = float(np.percentile(finite_magnitudes, 75.0))
    response_values = response_map[valid_mask]
    response_threshold = float(np.percentile(response_values, 55.0)) if response_values.size else 0.0
    support_mask = valid_mask & (gradient_magnitude >= magnitude_threshold) & (response_map >= response_threshold)
    if not np.any(support_mask):
        support_mask = valid_mask & (gradient_magnitude >= magnitude_threshold)
    if not np.any(support_mask):
        return {}

    line_angles = (np.degrees(np.arctan2(gradient_y, gradient_x)) + 90.0) % 180.0
    bin_indices = np.floor(line_angles[support_mask] / angle_step_deg).astype(np.int32) % bin_count
    weights = gradient_magnitude[support_mask] * np.maximum(response_map[support_mask], 0.05)
    histogram = np.zeros(bin_count, dtype=np.float64)
    np.add.at(histogram, bin_indices, weights.astype(np.float64))
    if histogram.size > 2:
        histogram = (
            np.roll(histogram, 1)
            + (2.0 * histogram)
            + np.roll(histogram, -1)
        ) / 4.0
    max_value = float(np.max(histogram))
    if max_value <= 1e-6:
        return {}
    histogram /= max_value
    return {
        round(float(index) * angle_step_deg, 3): float(value)
        for index, value in enumerate(histogram)
    }


def workspace_s2_lookup_orientation_prior_score(orientation_scores, line_angle_deg, angle_step_deg):
    if not orientation_scores:
        return 0.0
    angle_step_deg = max(0.5, float(angle_step_deg))
    angle_key = round(
        normalize_workspace_s2_angle_deg(round(float(line_angle_deg) / angle_step_deg) * angle_step_deg),
        3,
    )
    return float(orientation_scores.get(angle_key, 0.0))


def select_workspace_s2_orientation_candidate_angles(
    orientation_scores,
    angle_step_deg=2.0,
    seed_count=3,
    window_steps=2,
):
    if not orientation_scores:
        return None

    angle_step_deg = max(0.5, float(angle_step_deg))
    seed_count = max(1, int(round(seed_count)))
    window_steps = max(0, int(round(window_steps)))
    ordered_scores = sorted(
        ((float(angle), float(score)) for angle, score in orientation_scores.items()),
        key=lambda item: (-item[1], item[0]),
    )
    selected_seeds = []
    seed_suppression = max(angle_step_deg * 2.0, 6.0)
    for angle, score in ordered_scores:
        if score <= 0.0:
            continue
        if any(workspace_s2_angle_delta_deg(angle, selected_angle) <= seed_suppression for selected_angle in selected_seeds):
            continue
        selected_seeds.append(float(angle))
        if len(selected_seeds) >= seed_count:
            break

    if not selected_seeds:
        return None

    candidate_angles = set()
    for seed_angle in selected_seeds:
        for base_angle in (seed_angle, seed_angle + 90.0):
            for step_offset in range(-window_steps, window_steps + 1):
                candidate_angle = normalize_workspace_s2_angle_deg(base_angle + (float(step_offset) * angle_step_deg))
                candidate_angle = round(
                    normalize_workspace_s2_angle_deg(round(candidate_angle / angle_step_deg) * angle_step_deg),
                    3,
                )
                candidate_angles.add(candidate_angle)

    return sorted(candidate_angles)


def estimate_workspace_s2_oriented_line_family_candidates(
    response_map,
    workspace_mask,
    min_period=10,
    max_period=30,
    angle_step_deg=2.0,
    angle_candidates_deg=None,
    use_orientation_prior_angle_pool=True,
):
    angle_step_deg = max(0.5, float(angle_step_deg))
    orientation_scores = build_workspace_s2_orientation_prior_scores(
        response_map,
        workspace_mask,
        angle_step_deg=angle_step_deg,
    )
    if angle_candidates_deg is None and use_orientation_prior_angle_pool:
        angle_candidates_deg = select_workspace_s2_orientation_candidate_angles(
            orientation_scores,
            angle_step_deg=angle_step_deg,
        )
    if angle_candidates_deg is None:
        angle_candidates_deg = np.arange(0.0, 180.0, angle_step_deg, dtype=np.float32)

    candidates = []
    for line_angle_deg in angle_candidates_deg:
        profile_data = build_workspace_s2_oriented_axis_profile(
            response_map,
            workspace_mask,
            float(line_angle_deg),
        )
        estimate = estimate_workspace_s2_period_and_phase(
            profile_data["profile"],
            min_period=min_period,
            max_period=max_period,
        )
        if estimate is None:
            continue

        periodic_score = float(estimate.get("score", 0.0)) + (0.02 * float(estimate.get("phase_score", 0.0)))
        orientation_score = workspace_s2_lookup_orientation_prior_score(
            orientation_scores,
            float(line_angle_deg),
            angle_step_deg,
        )
        direction_score = periodic_score + (0.45 * orientation_score)
        candidate = dict(profile_data)
        candidate["estimate"] = estimate
        candidate["periodic_score"] = periodic_score
        candidate["orientation_score"] = orientation_score
        candidate["direction_score"] = direction_score
        candidates.append(candidate)

    if not candidates:
        return []

    candidates.sort(key=lambda candidate: (-candidate["direction_score"], candidate["line_angle_deg"]))
    return candidates


def estimate_workspace_s2_oriented_line_families(
    response_map,
    workspace_mask,
    min_period=10,
    max_period=30,
    angle_step_deg=2.0,
    min_family_angle_delta_deg=45.0,
    max_family_angle_delta_deg=135.0,
):
    candidates = estimate_workspace_s2_oriented_line_family_candidates(
        response_map,
        workspace_mask,
        min_period=min_period,
        max_period=max_period,
        angle_step_deg=angle_step_deg,
    )
    if not candidates:
        return []

    first_family = candidates[0]
    second_candidates = [
        candidate for candidate in candidates[1:]
        if min_family_angle_delta_deg
        <= workspace_s2_angle_delta_deg(candidate["line_angle_deg"], first_family["line_angle_deg"])
        <= max_family_angle_delta_deg
    ]
    if not second_candidates:
        return [first_family]

    def second_rank(candidate):
        angle_delta = workspace_s2_angle_delta_deg(candidate["line_angle_deg"], first_family["line_angle_deg"])
        orthogonal_penalty = abs(angle_delta - 90.0) * 0.001
        return (-(candidate["direction_score"] - orthogonal_penalty), candidate["line_angle_deg"])

    second_family = min(second_candidates, key=second_rank)
    return sorted([first_family, second_family], key=lambda family: family["line_angle_deg"])


def _workspace_s2_line_rect_t_range(point_on_line, direction, width, height):
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

        axis_t0 = (lower_bound - origin_value) / direction_value
        axis_t1 = (upper_bound - origin_value) / direction_value
        axis_min = min(axis_t0, axis_t1)
        axis_max = max(axis_t0, axis_t1)
        t_min = max(t_min, axis_min)
        t_max = min(t_max, axis_max)
        if t_max < t_min:
            return None

    if not np.isfinite(t_min) or not np.isfinite(t_max):
        return None
    return t_min, t_max


def _workspace_s2_oriented_line_support_samples(
    support_map,
    support_mask,
    line_rho,
    line_angle_deg,
    normal,
    search_radius_px,
    min_response_ratio,
    sample_step_px=1.0,
):
    height, width = support_map.shape[:2]
    normal = np.asarray(normal, dtype=np.float32).reshape(2)
    direction = workspace_s2_line_direction_from_angle(line_angle_deg)
    point_on_line = normal * float(line_rho)
    t_range = _workspace_s2_line_rect_t_range(point_on_line, direction, width, height)
    if t_range is None:
        return None

    t_min, t_max = t_range
    if t_max < t_min:
        return None

    offset_values = np.arange(-search_radius_px, search_radius_px + 1, dtype=np.float32)
    if offset_values.size == 0:
        offset_values = np.asarray([0.0], dtype=np.float32)
    sample_step_px = max(1.0, float(sample_step_px))
    t_values = np.arange(np.ceil(t_min), np.floor(t_max) + 1.0, sample_step_px, dtype=np.float32)
    if t_values.size == 0:
        return None

    base_points = point_on_line.reshape(1, 2) + (t_values.reshape(-1, 1) * direction.reshape(1, 2))
    sample_points = base_points.reshape(-1, 1, 2) + (offset_values.reshape(1, -1, 1) * normal.reshape(1, 1, 2))
    sample_x = np.rint(sample_points[:, :, 0]).astype(np.int32)
    sample_y = np.rint(sample_points[:, :, 1]).astype(np.int32)
    inside_mask = (
        (sample_x >= 0)
        & (sample_x < width)
        & (sample_y >= 0)
        & (sample_y < height)
    )
    clipped_x = np.clip(sample_x, 0, width - 1)
    clipped_y = np.clip(sample_y, 0, height - 1)
    mask_window = inside_mask & support_mask[clipped_y, clipped_x].astype(bool)
    response_window = np.where(mask_window, support_map[clipped_y, clipped_x], 0.0).astype(np.float32)
    row_has_support = np.any(mask_window, axis=1)
    if not np.any(row_has_support):
        return None

    sample_scores_all = np.max(response_window, axis=1)
    weights = np.where(
        mask_window,
        np.maximum(response_window - (float(min_response_ratio) * 0.5), 0.0),
        0.0,
    ).astype(np.float32)
    weight_sum = np.sum(weights, axis=1)
    weighted_offsets = np.sum(weights * offset_values.reshape(1, -1), axis=1)
    argmax_offsets = offset_values[np.argmax(response_window, axis=1)]
    sample_offsets_all = np.where(
        weight_sum > 1e-6,
        weighted_offsets / np.maximum(weight_sum, 1e-6),
        argmax_offsets,
    ).astype(np.float32)
    sample_offsets_all = np.where(sample_scores_all <= 0.0, 0.0, sample_offsets_all).astype(np.float32)

    distances = np.abs(offset_values.reshape(1, -1) - sample_offsets_all.reshape(-1, 1))
    core_radius = max(1.0, min(3.0, float(search_radius_px) * 0.25))
    side_width = max(1.0, min(3.0, float(search_radius_px) * 0.25))
    side_inner_radius = core_radius + 0.5
    side_outer_radius = side_inner_radius + side_width
    core_mask = mask_window & (distances <= core_radius)
    left_side_mask = (
        mask_window
        & (offset_values.reshape(1, -1) < sample_offsets_all.reshape(-1, 1))
        & (distances >= side_inner_radius)
        & (distances <= side_outer_radius)
    )
    right_side_mask = (
        mask_window
        & (offset_values.reshape(1, -1) > sample_offsets_all.reshape(-1, 1))
        & (distances >= side_inner_radius)
        & (distances <= side_outer_radius)
    )
    core_score = np.max(np.where(core_mask, response_window, 0.0), axis=1)
    left_side_score = np.max(np.where(left_side_mask, response_window, 0.0), axis=1)
    right_side_score = np.max(np.where(right_side_mask, response_window, 0.0), axis=1)
    shoulder_score = np.maximum(left_side_score, right_side_score)
    side_ok = np.any(left_side_mask, axis=1) & np.any(right_side_mask, axis=1)
    ridge_contrast_all = np.where(
        side_ok & (core_score > 1e-6),
        np.clip((core_score - shoulder_score) / np.maximum(core_score, 1e-6), 0.0, 1.0),
        0.0,
    ).astype(np.float32)

    sample_indices = np.where(row_has_support)[0].astype(np.int32)
    return (
        sample_indices,
        sample_scores_all[row_has_support].astype(np.float32),
        sample_offsets_all[row_has_support].astype(np.float32),
        ridge_contrast_all[row_has_support].astype(np.float32),
        int(t_values.size),
    )


def select_workspace_s2_continuous_line_rhos(
    response_map,
    workspace_mask,
    line_rhos,
    line_angle_deg,
    normal=None,
    search_radius_px=4,
    min_spacing_px=8,
    min_response_ratio=0.35,
    segment_count=12,
    min_segment_coverage=0.55,
    min_segment_sample_fraction=0.15,
    min_ridge_contrast_ratio=0.18,
    max_center_offset_px=None,
    max_offset_std_px=None,
    sample_step_px=1.0,
    precomputed_support_map=None,
    precomputed_support_mask=None,
    duplicate_spacing_px=None,
    return_scores=False,
):
    if not line_rhos:
        return ([], {}) if return_scores else []

    if precomputed_support_map is not None and np.asarray(precomputed_support_map).shape == np.asarray(response_map).shape:
        support_map = np.asarray(precomputed_support_map, dtype=np.float32)
    else:
        support_map = normalize_workspace_s2_response_for_line_support(response_map, workspace_mask)
    if support_map.ndim != 2 or support_map.size == 0:
        return ([], {}) if return_scores else []

    if precomputed_support_mask is not None and np.asarray(precomputed_support_mask).shape == support_map.shape:
        support_mask = np.asarray(precomputed_support_mask).astype(bool)
    elif workspace_mask is None:
        support_mask = np.ones_like(support_map, dtype=bool)
    else:
        support_mask = np.asarray(workspace_mask).astype(bool)
        if support_mask.shape != support_map.shape:
            support_mask = np.ones_like(support_map, dtype=bool)

    search_radius_px = max(0, int(round(search_radius_px)))
    min_spacing_px = max(1, int(round(min_spacing_px)))
    if duplicate_spacing_px is None:
        duplicate_spacing_px = min_spacing_px
    duplicate_spacing_px = max(0.0, float(duplicate_spacing_px))
    min_response_ratio = float(np.clip(min_response_ratio, 0.0, 1.0))
    segment_count = max(1, int(round(segment_count)))
    min_segment_coverage = float(np.clip(min_segment_coverage, 0.0, 1.0))
    min_segment_sample_fraction = float(np.clip(min_segment_sample_fraction, 0.0, 1.0))
    min_ridge_contrast_ratio = float(np.clip(min_ridge_contrast_ratio, 0.0, 1.0))
    sample_step_px = max(1.0, float(sample_step_px))
    if max_center_offset_px is None:
        max_center_offset_px = max(1.5, min(4.0, float(search_radius_px) * 0.35))
    max_center_offset_px = max(0.0, float(max_center_offset_px))
    if max_offset_std_px is None:
        max_offset_std_px = max(1.5, float(search_radius_px) * 0.9)
    max_offset_std_px = max(0.0, float(max_offset_std_px))
    normal = workspace_s2_line_normal_from_angle(line_angle_deg) if normal is None else np.asarray(normal, dtype=np.float32)

    supported_candidates = []
    for line_rho in line_rhos:
        samples = _workspace_s2_oriented_line_support_samples(
            support_map,
            support_mask,
            line_rho,
            line_angle_deg,
            normal,
            search_radius_px,
            min_response_ratio,
            sample_step_px=sample_step_px,
        )
        if samples is None:
            continue

        sample_indices, sample_scores, sample_offsets, sample_ridge_contrasts, length_size = samples
        strong_sample_mask = sample_scores >= min_response_ratio
        centered_sample_mask = np.abs(sample_offsets) <= max_center_offset_px
        ridge_sample_mask = (
            strong_sample_mask
            & centered_sample_mask
            & (sample_ridge_contrasts >= min_ridge_contrast_ratio)
        )
        if not np.any(ridge_sample_mask):
            continue

        segment_edges = np.linspace(0, length_size, segment_count + 1)
        supported_segment_count = 0
        valid_segment_count = 0
        for segment_index in range(segment_count):
            segment_start = segment_edges[segment_index]
            segment_end = segment_edges[segment_index + 1]
            if segment_index + 1 == segment_count:
                segment_mask = (sample_indices >= segment_start) & (sample_indices <= segment_end)
            else:
                segment_mask = (sample_indices >= segment_start) & (sample_indices < segment_end)
            if not np.any(segment_mask):
                continue

            valid_segment_count += 1
            segment_scores = sample_scores[segment_mask]
            segment_offsets = sample_offsets[segment_mask]
            segment_ridge_contrasts = sample_ridge_contrasts[segment_mask]
            ridge_like_mask = (
                (segment_scores >= min_response_ratio)
                & (np.abs(segment_offsets) <= max_center_offset_px)
                & (segment_ridge_contrasts >= min_ridge_contrast_ratio)
            )
            strong_fraction = float(np.count_nonzero(ridge_like_mask)) / float(segment_scores.size)
            if strong_fraction >= min_segment_sample_fraction:
                supported_segment_count += 1

        if valid_segment_count == 0:
            continue

        segment_coverage = float(supported_segment_count) / float(valid_segment_count)
        if segment_coverage < min_segment_coverage:
            continue

        strong_offsets = sample_offsets[ridge_sample_mask]
        offset_std = float(np.std(strong_offsets)) if strong_offsets.size > 1 else 0.0
        if offset_std > max_offset_std_px:
            continue

        snapped_rho = float(line_rho) + float(np.median(strong_offsets))
        mean_score = float(np.mean(sample_scores[ridge_sample_mask]))
        continuity_score = segment_coverage + (0.25 * mean_score) - (0.03 * offset_std)
        supported_candidates.append((snapped_rho, continuity_score))

    supported_candidates.sort(key=lambda item: (-item[1], item[0]))
    selected_rhos = []
    selected_scores = {}
    for snapped_rho, _score in supported_candidates:
        if any(abs(float(snapped_rho) - float(selected_rho)) < duplicate_spacing_px for selected_rho in selected_rhos):
            continue
        selected_rho = float(snapped_rho)
        selected_rhos.append(selected_rho)
        selected_scores[selected_rho] = float(_score)

    selected_rhos = sorted(selected_rhos)
    if return_scores:
        return selected_rhos, {rho: selected_scores[rho] for rho in selected_rhos}
    return selected_rhos


def build_workspace_s2_oriented_line_rhos(
    family,
    enable_local_peak_refine=True,
    enable_peak_support=True,
    duplicate_spacing_px=None,
):
    profile = np.asarray(family.get("profile", []), dtype=np.float32).reshape(-1)
    estimate = family.get("estimate") or {}
    if profile.size == 0 or not estimate:
        return []

    line_positions = build_workspace_s2_line_positions(
        0,
        profile.size - 1,
        estimate.get("period", 0),
        estimate.get("phase", 0),
    )
    period = max(1, int(round(float(estimate.get("period", 1)))))
    if duplicate_spacing_px is None:
        duplicate_spacing_px = max(2, int(round(period * 0.55)))
    duplicate_spacing_px = max(0.0, float(duplicate_spacing_px))
    if enable_local_peak_refine:
        refined_positions = refine_workspace_s2_line_positions_to_local_peaks(
            profile,
            line_positions,
            search_radius_px=max(2, min(14, int(round(period * 0.55)))),
            min_spacing_px=max(2, int(round(period * 0.55))),
            target_edge_margin_ratio=0.55,
            edge_anchor_weight=0.25,
        )
    else:
        refined_positions = [int(round(position)) for position in line_positions]

    if enable_peak_support:
        peak_positions = select_workspace_s2_peak_supported_line_positions(
            profile,
            refined_positions,
            search_radius_px=max(2, min(14, int(round(period * 0.55)))),
            min_spacing_px=max(2, int(round(period * 0.55))),
            duplicate_spacing_px=duplicate_spacing_px,
        )
    else:
        peak_positions = refined_positions
    rho_min = float(family.get("rho_min", 0.0))
    return [rho_min + float(position) for position in peak_positions]


def prune_workspace_s2_line_rhos_by_spacing(line_rhos, min_spacing_ratio=0.65):
    return [float(position) for position in prune_workspace_s2_line_positions_by_spacing(line_rhos, min_spacing_ratio)]


def prune_workspace_s2_line_rhos_by_near_duplicate_spacing(line_rhos, duplicate_spacing_px=4):
    rhos = sorted(float(rho) for rho in line_rhos)
    if len(rhos) < 2:
        return rhos

    duplicate_spacing_px = max(0.0, float(duplicate_spacing_px))
    selected_rhos = []
    for rho in rhos:
        if selected_rhos and abs(float(rho) - float(selected_rhos[-1])) < duplicate_spacing_px:
            continue
        selected_rhos.append(float(rho))

    return selected_rhos


def prune_workspace_s2_line_rhos_by_scored_spacing(line_rhos, line_scores=None, min_spacing_ratio=0.60):
    rhos = sorted(float(rho) for rho in line_rhos)
    if len(rhos) < 3:
        return rhos

    normalized_scores = {float(key): float(value) for key, value in (line_scores or {}).items()}
    regular_lattice_rhos = select_workspace_s2_regular_lattice_line_rhos(
        rhos,
        line_scores=normalized_scores,
    )
    if regular_lattice_rhos is not None:
        return regular_lattice_rhos

    positions = list(rhos)
    min_spacing_ratio = max(0.0, float(min_spacing_ratio))
    while len(positions) >= 3:
        reference_spacing = estimate_workspace_s2_line_spacing(positions)
        if reference_spacing is None or reference_spacing <= 1e-6:
            break

        min_allowed_spacing = reference_spacing * min_spacing_ratio
        differences = np.diff(np.asarray(positions, dtype=np.float32))
        close_pair_indices = np.where(differences < min_allowed_spacing)[0]
        if close_pair_indices.size == 0:
            break

        pair_index = int(close_pair_indices[0])
        left_index = pair_index
        right_index = pair_index + 1
        left_rho = float(positions[left_index])
        right_rho = float(positions[right_index])
        left_score = normalized_scores.get(left_rho, 0.0)
        right_score = normalized_scores.get(right_rho, 0.0)
        if abs(left_score - right_score) > 1e-6:
            remove_index = left_index if left_score < right_score else right_index
        elif left_index == 0:
            remove_index = left_index
        elif right_index == len(positions) - 1:
            remove_index = right_index
        else:
            remove_left_gap = positions[right_index] - positions[left_index - 1]
            remove_right_gap = positions[right_index + 1] - positions[left_index]
            remove_left_error = abs(float(remove_left_gap) - reference_spacing)
            remove_right_error = abs(float(remove_right_gap) - reference_spacing)
            remove_index = left_index if remove_left_error <= remove_right_error else right_index

        positions.pop(remove_index)

    return [float(position) for position in positions]


def select_workspace_s2_regular_lattice_line_rhos(
    line_rhos,
    line_scores=None,
    min_line_count=3,
    max_line_count=10,
):
    rhos = sorted(float(rho) for rho in line_rhos)
    if len(rhos) <= max_line_count:
        return None

    min_line_count = max(2, int(round(min_line_count)))
    max_line_count = max(min_line_count, int(round(max_line_count)))
    rho_span = float(rhos[-1] - rhos[0])
    if rho_span <= 1e-6:
        return None

    scores = {float(key): float(value) for key, value in (line_scores or {}).items()}
    rho_array = np.asarray(rhos, dtype=np.float32)

    def nearest_candidate_rho(expected_rho):
        insert_index = int(np.searchsorted(rho_array, float(expected_rho)))
        candidate_indices = []
        if insert_index < rho_array.size:
            candidate_indices.append(insert_index)
        if insert_index > 0:
            candidate_indices.append(insert_index - 1)
        if not candidate_indices:
            return None
        best_index = min(
            candidate_indices,
            key=lambda index: (abs(float(rho_array[index]) - float(expected_rho)), float(rho_array[index])),
        )
        return float(rho_array[best_index])

    min_spacing = max(24.0, rho_span / float(max_line_count))
    max_spacing = max(min_spacing + 1.0, rho_span / float(max(1, min_line_count - 1)))
    candidate_spacings = set()
    for left_index, left_rho in enumerate(rhos):
        for right_rho in rhos[left_index + 1:]:
            spacing = float(right_rho - left_rho)
            if min_spacing <= spacing <= max_spacing:
                candidate_spacings.add(round(spacing, 3))
    if not candidate_spacings:
        for spacing in np.linspace(min_spacing, max_spacing, 32):
            candidate_spacings.add(round(float(spacing), 3))

    best_candidate = None
    best_score = None
    for spacing in sorted(candidate_spacings):
        spacing = float(spacing)
        if spacing <= 1e-6:
            continue
        match_tolerance = max(4.0, min(14.0, spacing * 0.18))
        for anchor_rho in rhos:
            min_k = int(np.floor((rhos[0] - anchor_rho) / spacing)) - 1
            max_k = int(np.ceil((rhos[-1] - anchor_rho) / spacing)) + 1
            selected = []
            errors = []
            total_score = 0.0
            for step_index in range(min_k, max_k + 1):
                expected_rho = float(anchor_rho) + (float(step_index) * spacing)
                nearest_rho = nearest_candidate_rho(expected_rho)
                if nearest_rho is None:
                    continue
                error = abs(float(nearest_rho) - expected_rho)
                if error > match_tolerance:
                    continue
                if any(abs(float(nearest_rho) - float(existing_rho)) <= match_tolerance for existing_rho in selected):
                    continue
                selected.append(float(nearest_rho))
                errors.append(float(error))
                total_score += scores.get(float(nearest_rho), 0.0)

            selected = sorted(set(selected))
            if len(selected) < min_line_count or len(selected) > max_line_count:
                continue

            diffs = np.diff(np.asarray(selected, dtype=np.float32))
            if diffs.size == 0 or float(np.min(diffs)) < min_spacing:
                continue
            spacing_cv = float(np.std(diffs) / max(float(np.mean(diffs)), 1e-6)) if diffs.size > 1 else 0.0
            mean_score = float(total_score) / float(len(selected))
            mean_error_ratio = float(np.mean(errors)) / max(spacing, 1.0) if errors else 0.0
            span_coverage = float(selected[-1] - selected[0]) / max(rho_span, 1.0)
            line_count = len(selected)
            preferred_line_count = max(
                float(min_line_count),
                min(float(max_line_count), float(LEGACY_WORKSPACE_S2_PREFERRED_LATTICE_LINE_COUNT)),
            )
            count_score = (
                min(float(line_count), preferred_line_count) * 1.2
                - max(0.0, float(line_count) - preferred_line_count) * 1.35
            )
            candidate_score = (
                count_score
                + (mean_score * 3.0)
                + (span_coverage * 0.8)
                - (spacing_cv * 4.0)
                - (mean_error_ratio * 2.0)
            )
            candidate = [float(rho) for rho in selected]
            if (
                best_score is None
                or candidate_score > best_score
                or (
                    abs(candidate_score - best_score) <= 1e-6
                    and (len(candidate), candidate) < (len(best_candidate or []), best_candidate or [])
                )
            ):
                best_candidate = candidate
                best_score = candidate_score

    return best_candidate


def score_workspace_s2_family_profile_rhos(family, line_rhos):
    profile = normalize_workspace_s2_profile_for_support(family.get("profile", []))
    if profile.size == 0:
        return {float(rho): 0.0 for rho in line_rhos}
    rho_min = float(family.get("rho_min", 0.0))
    scores = {}
    for rho in line_rhos:
        profile_index = int(np.clip(int(round(float(rho) - rho_min)), 0, profile.size - 1))
        scores[float(rho)] = float(profile[profile_index])
    return scores


def _workspace_s2_mean_selected_line_support_score(family):
    line_rhos = [float(rho) for rho in family.get("line_rhos", [])]
    line_scores = {
        float(key): float(value)
        for key, value in (family.get("continuous_scores") or {}).items()
    }
    if not line_rhos or not line_scores:
        return 0.0

    selected_scores = []
    score_keys = list(line_scores.keys())
    for line_rho in line_rhos:
        if line_rho in line_scores:
            selected_scores.append(line_scores[line_rho])
            continue
        nearest_key = min(score_keys, key=lambda key: abs(float(key) - line_rho))
        if abs(float(nearest_key) - line_rho) <= 2.0:
            selected_scores.append(line_scores[nearest_key])

    if not selected_scores:
        return 0.0
    return float(np.mean(selected_scores))


def _workspace_s2_supported_family_score(family):
    line_count = len(family.get("line_rhos", []))
    capped_line_count = min(float(line_count), float(LEGACY_WORKSPACE_S2_PREFERRED_LATTICE_LINE_COUNT))
    extra_line_penalty = max(
        0.0,
        float(line_count) - float(LEGACY_WORKSPACE_S2_PREFERRED_LATTICE_LINE_COUNT),
    ) * 1.1
    too_many_penalty = max(0.0, float(line_count) - 8.0) * 0.9
    return (
        capped_line_count
        - extra_line_penalty
        - too_many_penalty
        + (3.0 * _workspace_s2_mean_selected_line_support_score(family))
        + (0.1 * float(family.get("periodic_score", 0.0)))
        + (1.2 * float(family.get("orientation_score", 0.0)))
    )


def _workspace_s2_oriented_family_pair_score(left_family, right_family):
    left_count = len(left_family.get("line_rhos", []))
    right_count = len(right_family.get("line_rhos", []))
    angle_delta = workspace_s2_angle_delta_deg(left_family["line_angle_deg"], right_family["line_angle_deg"])
    orthogonal_penalty = abs(angle_delta - 90.0) * 0.15
    balance_penalty = abs(float(left_count) - float(right_count)) * 0.35
    point_count = left_count * right_count
    density_score = min(float(point_count), 16.0) * 0.06
    sparse_penalty = max(0.0, float(LEGACY_WORKSPACE_S2_SCORE_TARGET_MIN_POINTS) - float(point_count)) * 0.12
    explosion_penalty = max(0.0, float(point_count) - 64.0) * 0.25
    return (
        _workspace_s2_supported_family_score(left_family)
        + _workspace_s2_supported_family_score(right_family)
        + density_score
        - sparse_penalty
        - explosion_penalty
        - orthogonal_penalty
        - balance_penalty
    )


def score_workspace_s2_oriented_line_family_result(
    line_families,
    target_min_points=LEGACY_WORKSPACE_S2_SCORE_TARGET_MIN_POINTS,
):
    line_families = list(line_families or [])
    if len(line_families) < 2:
        return -float("inf")

    left_family, right_family = line_families[:2]
    left_count = len(left_family.get("line_rhos", []))
    right_count = len(right_family.get("line_rhos", []))
    if left_count < MIN_WORKSPACE_S2_SUPPORTED_LINE_COUNT or right_count < MIN_WORKSPACE_S2_SUPPORTED_LINE_COUNT:
        return -float("inf")

    point_count = left_count * right_count
    angle_delta = workspace_s2_angle_delta_deg(
        left_family.get("line_angle_deg", 0.0),
        right_family.get("line_angle_deg", 0.0),
    )
    family_score = (
        _workspace_s2_supported_family_score(left_family)
        + _workspace_s2_supported_family_score(right_family)
    )
    density_score = min(float(point_count), 16.0) * 0.06
    sparse_penalty = max(0.0, float(target_min_points) - float(point_count)) * 0.20
    explosion_penalty = (
        max(0.0, float(point_count) - 64.0) * 0.35
        + max(0.0, float(left_count) - 8.0) * 0.75
        + max(0.0, float(right_count) - 8.0) * 0.75
    )
    orthogonal_penalty = abs(angle_delta - 90.0) * 0.10
    balance_penalty = abs(float(left_count) - float(right_count)) * 0.20
    return (
        family_score
        + density_score
        - sparse_penalty
        - explosion_penalty
        - orthogonal_penalty
        - balance_penalty
    )


def _select_workspace_s2_oriented_candidate_pool(candidates, angle_step_deg, seed_count=2):
    if not candidates:
        return []

    seed_count = max(1, int(round(seed_count)))
    angle_window = max(2.0, float(angle_step_deg))
    selected_by_angle = {}

    def add_candidate(candidate):
        angle_key = round(float(candidate["line_angle_deg"]), 3)
        current = selected_by_angle.get(angle_key)
        if current is None or float(candidate.get("direction_score", 0.0)) > float(current.get("direction_score", 0.0)):
            selected_by_angle[angle_key] = candidate

    def add_candidates_near(target_angle):
        for candidate in candidates:
            if workspace_s2_angle_delta_deg(candidate["line_angle_deg"], target_angle) <= angle_window:
                add_candidate(candidate)

    for seed in candidates[:seed_count]:
        add_candidates_near(seed["line_angle_deg"])
        target_angle = normalize_workspace_s2_angle_deg(float(seed["line_angle_deg"]) + 90.0)
        add_candidates_near(target_angle)

    return sorted(selected_by_angle.values(), key=lambda candidate: (-candidate["direction_score"], candidate["line_angle_deg"]))


def build_workspace_s2_oriented_line_families(
    response_map,
    workspace_mask,
    min_period=10,
    max_period=30,
    angle_step_deg=2.0,
    use_orientation_prior_angle_pool=True,
    enable_local_peak_refine=False,
    enable_peak_support=True,
    enable_continuous_validation=True,
    enable_spacing_prune=True,
    continuous_sample_step_px=5.0,
    enable_structural_edge_suppression=True,
):
    line_response_map = (
        suppress_workspace_s2_structural_edge_response(response_map, workspace_mask)
        if enable_structural_edge_suppression
        else np.asarray(response_map, dtype=np.float32)
    )
    families = estimate_workspace_s2_oriented_line_family_candidates(
        line_response_map,
        workspace_mask,
        min_period=min_period,
        max_period=max_period,
        angle_step_deg=angle_step_deg,
        use_orientation_prior_angle_pool=use_orientation_prior_angle_pool,
    )
    candidate_pool = _select_workspace_s2_oriented_candidate_pool(families, angle_step_deg)
    continuous_support_map = None
    continuous_support_mask = None
    if enable_continuous_validation:
        continuous_support_map = normalize_workspace_s2_response_for_line_support(line_response_map, workspace_mask)
        if workspace_mask is None:
            continuous_support_mask = np.ones_like(continuous_support_map, dtype=bool)
        else:
            continuous_support_mask = np.asarray(workspace_mask).astype(bool)
            if continuous_support_mask.shape != continuous_support_map.shape:
                continuous_support_mask = np.ones_like(continuous_support_map, dtype=bool)
    built_families = []
    for family in candidate_pool:
        family = dict(family)
        estimate = family.get("estimate") or {}
        period = max(1, int(round(float(estimate.get("period", 1)))))
        min_spacing_px = max(2, int(round(period * 0.55)))
        duplicate_spacing_px = max(2, min(6, int(round(period * 0.20))))
        peak_rhos = build_workspace_s2_oriented_line_rhos(
            family,
            enable_local_peak_refine=enable_local_peak_refine,
            enable_peak_support=enable_peak_support,
            duplicate_spacing_px=duplicate_spacing_px,
        )
        peak_scores = score_workspace_s2_family_profile_rhos(family, peak_rhos)
        if enable_continuous_validation:
            continuous_rhos, continuous_scores = select_workspace_s2_continuous_line_rhos(
                line_response_map,
                workspace_mask,
                peak_rhos,
                family["line_angle_deg"],
                normal=family["normal"],
                search_radius_px=max(2, min(14, int(round(period * 0.55)))),
                min_spacing_px=min_spacing_px,
                duplicate_spacing_px=duplicate_spacing_px,
                sample_step_px=continuous_sample_step_px,
                precomputed_support_map=continuous_support_map,
                precomputed_support_mask=continuous_support_mask,
                return_scores=True,
            )
            if len(continuous_rhos) < MIN_WORKSPACE_S2_SUPPORTED_LINE_COUNT:
                relaxed_rhos, relaxed_scores = select_workspace_s2_continuous_line_rhos(
                    line_response_map,
                    workspace_mask,
                    peak_rhos,
                    family["line_angle_deg"],
                    normal=family["normal"],
                    search_radius_px=max(2, min(14, int(round(period * 0.55)))),
                    min_spacing_px=min_spacing_px,
                    duplicate_spacing_px=duplicate_spacing_px,
                    sample_step_px=continuous_sample_step_px,
                    min_segment_coverage=0.25,
                    min_ridge_contrast_ratio=0.10,
                    precomputed_support_map=continuous_support_map,
                    precomputed_support_mask=continuous_support_mask,
                    return_scores=True,
                )
                if len(relaxed_rhos) >= len(continuous_rhos):
                    continuous_rhos = relaxed_rhos
                    continuous_scores = relaxed_scores
            if len(continuous_rhos) < MIN_WORKSPACE_S2_SUPPORTED_LINE_COUNT:
                continuous_rhos = list(peak_rhos)
                continuous_scores = dict(peak_scores)
        else:
            continuous_rhos = list(peak_rhos)
            continuous_scores = dict(peak_scores)

        if enable_spacing_prune:
            spacing_rhos = prune_workspace_s2_line_rhos_by_scored_spacing(
                continuous_rhos,
                line_scores=continuous_scores,
                min_spacing_ratio=0.60,
            )
        else:
            spacing_rhos = list(continuous_rhos)
        family["peak_rhos"] = peak_rhos
        family["continuous_rhos"] = continuous_rhos
        family["continuous_scores"] = continuous_scores
        family["line_rhos"] = spacing_rhos
        built_families.append(family)

    supported_families = [
        family
        for family in built_families
        if len(family.get("line_rhos", [])) >= MIN_WORKSPACE_S2_SUPPORTED_LINE_COUNT
    ]
    if len(supported_families) < 2:
        selected_families = supported_families
    else:
        best_pair = None
        best_pair_score = None
        for left_index, left_family in enumerate(supported_families):
            for right_family in supported_families[left_index + 1:]:
                angle_delta = workspace_s2_angle_delta_deg(
                    left_family["line_angle_deg"],
                    right_family["line_angle_deg"],
                )
                if angle_delta < 45.0 or angle_delta > 135.0:
                    continue
                pair_score = _workspace_s2_oriented_family_pair_score(left_family, right_family)
                if best_pair_score is None or pair_score > best_pair_score:
                    best_pair = (left_family, right_family)
                    best_pair_score = pair_score
        selected_families = list(best_pair) if best_pair is not None else supported_families[:2]

    selected_families = sorted(selected_families, key=lambda family: family["line_angle_deg"])
    for family_index, family in enumerate(selected_families):
        family["family_index"] = family_index
    return selected_families


def _workspace_s2_axis_family_line_positions(
    profile,
    min_period=10,
    max_period=30,
    enable_local_peak_refine=False,
    enable_peak_support=True,
    peak_min_ratio=0.35,
    period_estimator="profile",
):
    profile = np.asarray(profile, dtype=np.float32).reshape(-1)
    if profile.size == 0:
        return [], [], None

    if str(period_estimator).lower() in ("fft", "rfft", "frequency"):
        estimate = estimate_workspace_s2_fft_period_and_phase(
            profile,
            min_period=min_period,
            max_period=max_period,
        )
    else:
        estimate = estimate_workspace_s2_period_and_phase(
            profile,
            min_period=min_period,
            max_period=max_period,
        )
    initial_positions = []
    if estimate is not None:
        initial_positions = build_workspace_s2_line_positions(
            0,
            profile.size - 1,
            estimate.get("period", 0),
            estimate.get("phase", 0),
        )

    period = int(round(float(estimate.get("period", min_period)))) if estimate else int(round(min_period))
    period = max(1, period)
    search_radius_px = max(2, min(14, int(round(period * 0.55))))
    min_spacing_px = max(2, int(round(period * 0.55)))
    duplicate_spacing_px = max(4, min(8, int(round(period * 0.35))))

    if enable_peak_support:
        candidate_positions = list(range(profile.size))
    elif enable_local_peak_refine and initial_positions:
        candidate_positions = refine_workspace_s2_line_positions_to_local_peaks(
            profile,
            initial_positions,
            search_radius_px=search_radius_px,
            min_spacing_px=min_spacing_px,
            target_edge_margin_ratio=0.55,
            edge_anchor_weight=0.25,
        )
    else:
        candidate_positions = [int(round(position)) for position in initial_positions]

    if enable_peak_support:
        selected_positions = select_workspace_s2_peak_supported_line_positions(
            profile,
            candidate_positions,
            search_radius_px=search_radius_px,
            min_spacing_px=min_spacing_px,
            min_peak_ratio=peak_min_ratio,
            duplicate_spacing_px=duplicate_spacing_px,
        )
    else:
        selected_positions = candidate_positions

    return initial_positions, selected_positions, estimate


def _workspace_s2_build_axis_aligned_family(
    response_map,
    workspace_mask,
    axis,
    line_angle_deg,
    normal,
    axis_orientation,
    min_period=10,
    max_period=30,
    enable_local_peak_refine=False,
    enable_peak_support=True,
    enable_continuous_validation=False,
    enable_spacing_prune=True,
    peak_min_ratio=0.35,
    period_estimator="profile",
):
    profile = build_workspace_s2_axis_profile(response_map, workspace_mask, axis=axis)
    initial_positions, peak_positions, estimate = _workspace_s2_axis_family_line_positions(
        profile,
        min_period=min_period,
        max_period=max_period,
        enable_local_peak_refine=enable_local_peak_refine,
        enable_peak_support=enable_peak_support,
        peak_min_ratio=peak_min_ratio,
        period_estimator=period_estimator,
    )
    period = int(round(float(estimate.get("period", min_period)))) if estimate else int(round(min_period))
    period = max(1, period)
    min_spacing_px = max(2, int(round(period * 0.55)))
    duplicate_spacing_px = max(4, min(8, int(round(period * 0.35))))
    peak_scores = {
        float(position): float(score)
        for position, score in score_workspace_s2_family_profile_rhos(
            {"profile": profile, "rho_min": 0.0},
            [float(position) for position in peak_positions],
        ).items()
    }

    if enable_continuous_validation:
        continuous_positions = select_workspace_s2_continuous_line_positions(
            response_map,
            workspace_mask,
            peak_positions,
            axis_orientation,
            search_radius_px=max(2, min(14, int(round(period * 0.55)))),
            min_spacing_px=min_spacing_px,
            duplicate_spacing_px=duplicate_spacing_px,
        )
        continuous_scores = {
            float(position): peak_scores.get(float(position), 0.0)
            for position in continuous_positions
        }
    else:
        continuous_positions = list(peak_positions)
        continuous_scores = dict(peak_scores)

    if enable_spacing_prune:
        line_positions = prune_workspace_s2_line_rhos_by_scored_spacing(
            continuous_positions,
            line_scores=continuous_scores,
            min_spacing_ratio=0.60,
        )
    else:
        line_positions = [float(position) for position in continuous_positions]

    return {
        "axis_orientation": axis_orientation,
        "line_angle_deg": normalize_workspace_s2_angle_deg(line_angle_deg),
        "normal": [float(normal[0]), float(normal[1])],
        "direction": workspace_s2_line_direction_from_angle(line_angle_deg).tolist(),
        "rho_min": 0.0,
        "profile": np.asarray(profile, dtype=np.float32),
        "estimate": estimate or {},
        "period_estimator": str(period_estimator),
        "periodic_score": float((estimate or {}).get("score", 0.0)),
        "orientation_score": 1.0,
        "initial_positions": [int(round(position)) for position in initial_positions],
        "peak_positions": [int(round(position)) for position in peak_positions],
        "continuous_positions": [int(round(position)) for position in continuous_positions],
        "line_positions": [int(round(position)) for position in line_positions],
        "initial_rhos": [float(position) for position in initial_positions],
        "peak_rhos": [float(position) for position in peak_positions],
        "continuous_rhos": [float(position) for position in continuous_positions],
        "continuous_scores": continuous_scores,
        "line_rhos": [float(position) for position in line_positions],
    }


def build_workspace_s2_axis_aligned_line_families(
    response_map,
    workspace_mask,
    min_period=10,
    max_period=30,
    enable_local_peak_refine=False,
    enable_peak_support=True,
    enable_continuous_validation=False,
    enable_spacing_prune=True,
    enable_structural_edge_suppression=True,
    peak_min_ratio=0.35,
    period_estimator="profile",
):
    line_response_map = (
        suppress_workspace_s2_structural_edge_response(response_map, workspace_mask)
        if enable_structural_edge_suppression
        else np.asarray(response_map, dtype=np.float32)
    )
    family_specs = (
        {
            "axis": 1,
            "line_angle_deg": 0.0,
            "normal": (0.0, 1.0),
            "axis_orientation": "horizontal",
        },
        {
            "axis": 0,
            "line_angle_deg": 90.0,
            "normal": (1.0, 0.0),
            "axis_orientation": "vertical",
        },
    )

    families = [
        _workspace_s2_build_axis_aligned_family(
            line_response_map,
            workspace_mask,
            min_period=min_period,
            max_period=max_period,
            enable_local_peak_refine=enable_local_peak_refine,
            enable_peak_support=enable_peak_support,
            enable_continuous_validation=enable_continuous_validation,
            enable_spacing_prune=enable_spacing_prune,
            peak_min_ratio=peak_min_ratio,
            period_estimator=period_estimator,
            **family_spec,
        )
        for family_spec in family_specs
    ]
    supported_families = [
        family
        for family in families
        if len(family.get("line_rhos", [])) >= MIN_WORKSPACE_S2_SUPPORTED_LINE_COUNT
    ]
    for family_index, family in enumerate(supported_families):
        family["family_index"] = family_index
    return supported_families


def intersect_workspace_s2_oriented_line_families(
    first_family,
    second_family,
    rectified_width,
    rectified_height,
    margin_px=0.0,
):
    first_normal = np.asarray(first_family.get("normal"), dtype=np.float32).reshape(2)
    second_normal = np.asarray(second_family.get("normal"), dtype=np.float32).reshape(2)
    matrix = np.asarray([first_normal, second_normal], dtype=np.float32)
    determinant = float(np.linalg.det(matrix))
    if abs(determinant) <= 1e-6:
        return []

    intersections = []
    margin_px = max(0.0, float(margin_px))
    min_x = -margin_px
    max_x = float(rectified_width - 1) + margin_px
    min_y = -margin_px
    max_y = float(rectified_height - 1) + margin_px
    for second_rho in second_family.get("line_rhos", []):
        for first_rho in first_family.get("line_rhos", []):
            point = np.linalg.solve(matrix, np.asarray([float(first_rho), float(second_rho)], dtype=np.float32))
            point_x = float(point[0])
            point_y = float(point[1])
            if min_x <= point_x <= max_x and min_y <= point_y <= max_y:
                intersections.append([
                    float(np.clip(point_x, 0.0, float(rectified_width - 1))),
                    float(np.clip(point_y, 0.0, float(rectified_height - 1))),
                ])
    return intersections


def _workspace_s2_sample_line_offset_scores(
    support_map,
    support_mask,
    base_point,
    normal,
    offset_values,
    search_radius_px,
    score_mode,
):
    height, width = support_map.shape[:2]
    sample_points = base_point.reshape(1, 2) + (offset_values.reshape(-1, 1) * normal.reshape(1, 2))
    sample_x = np.rint(sample_points[:, 0]).astype(np.int32)
    sample_y = np.rint(sample_points[:, 1]).astype(np.int32)
    inside_mask = (
        (sample_x >= 0)
        & (sample_x < width)
        & (sample_y >= 0)
        & (sample_y < height)
    )
    mask_window = np.zeros(offset_values.shape, dtype=bool)
    response_window = np.zeros(offset_values.shape, dtype=np.float32)
    if np.any(inside_mask):
        inside_indices = np.where(inside_mask)[0]
        inside_y = sample_y[inside_indices]
        inside_x = sample_x[inside_indices]
        valid_inside = support_mask[inside_y, inside_x].astype(bool)
        valid_indices = inside_indices[valid_inside]
        if valid_indices.size > 0:
            mask_window[valid_indices] = True
            response_window[valid_indices] = support_map[sample_y[valid_indices], sample_x[valid_indices]]

    score_values = np.zeros(offset_values.shape, dtype=np.float32)
    if not np.any(mask_window):
        return score_values, response_window, mask_window

    if score_mode == "response":
        score_values = response_window.copy()
    else:
        ridge_scores = np.zeros(offset_values.shape, dtype=np.float32)
        for offset_index, offset_value in enumerate(offset_values):
            if not mask_window[offset_index]:
                continue
            ridge_scores[offset_index] = _workspace_s2_centered_ridge_contrast(
                response_window,
                mask_window,
                offset_values,
                float(offset_value),
                search_radius_px,
            )
        if score_mode == "ridge_contrast":
            score_values = ridge_scores
        elif score_mode == "response_ridge":
            score_values = (0.7 * response_window) + (0.3 * ridge_scores)
        else:
            raise ValueError("score_mode must be 'response', 'ridge_contrast', or 'response_ridge'")

    score_values = np.where(mask_window, score_values, 0.0).astype(np.float32)
    return score_values, response_window, mask_window


def _workspace_s2_sample_line_offset_score_matrix(
    support_map,
    support_mask,
    point_on_line,
    direction,
    normal,
    t_values,
    offset_values,
    search_radius_px,
    score_mode,
):
    height, width = support_map.shape[:2]
    base_points = point_on_line.reshape(1, 2) + (t_values.reshape(-1, 1) * direction.reshape(1, 2))
    sample_points = base_points.reshape(-1, 1, 2) + (offset_values.reshape(1, -1, 1) * normal.reshape(1, 1, 2))
    sample_x = np.rint(sample_points[:, :, 0]).astype(np.int32)
    sample_y = np.rint(sample_points[:, :, 1]).astype(np.int32)
    inside_mask = (
        (sample_x >= 0)
        & (sample_x < width)
        & (sample_y >= 0)
        & (sample_y < height)
    )
    clipped_x = np.clip(sample_x, 0, width - 1)
    clipped_y = np.clip(sample_y, 0, height - 1)
    valid_matrix = inside_mask & support_mask[clipped_y, clipped_x].astype(bool)
    response_matrix = np.where(valid_matrix, support_map[clipped_y, clipped_x], 0.0).astype(np.float32)

    if score_mode == "response":
        score_matrix = response_matrix.copy()
    else:
        coordinate_values = offset_values.astype(np.float32)
        candidate_positions = offset_values.astype(np.float32)
        distances = np.abs(coordinate_values.reshape(1, -1) - candidate_positions.reshape(-1, 1))
        core_radius = max(1.0, min(3.0, float(search_radius_px) * 0.25))
        side_width = max(1.0, min(3.0, float(search_radius_px) * 0.25))
        side_inner_radius = core_radius + 0.5
        side_outer_radius = side_inner_radius + side_width
        core_template = distances <= core_radius
        left_template = (
            (coordinate_values.reshape(1, -1) < candidate_positions.reshape(-1, 1))
            & (distances >= side_inner_radius)
            & (distances <= side_outer_radius)
        )
        right_template = (
            (coordinate_values.reshape(1, -1) > candidate_positions.reshape(-1, 1))
            & (distances >= side_inner_radius)
            & (distances <= side_outer_radius)
        )

        response_cube = response_matrix[:, None, :]
        valid_cube = valid_matrix[:, None, :]
        core_mask = valid_cube & core_template.reshape(1, *core_template.shape)
        left_mask = valid_cube & left_template.reshape(1, *left_template.shape)
        right_mask = valid_cube & right_template.reshape(1, *right_template.shape)
        core_score = np.max(np.where(core_mask, response_cube, 0.0), axis=2)
        left_score = np.max(np.where(left_mask, response_cube, 0.0), axis=2)
        right_score = np.max(np.where(right_mask, response_cube, 0.0), axis=2)
        side_ok = np.any(left_mask, axis=2) & np.any(right_mask, axis=2)
        ridge_scores = np.where(
            side_ok & (core_score > 1e-6),
            np.clip((core_score - np.maximum(left_score, right_score)) / np.maximum(core_score, 1e-6), 0.0, 1.0),
            0.0,
        ).astype(np.float32)
        if score_mode == "ridge_contrast":
            score_matrix = ridge_scores
        elif score_mode == "response_ridge":
            score_matrix = (0.7 * response_matrix) + (0.3 * ridge_scores)
        else:
            raise ValueError("score_mode must be 'response', 'ridge_contrast', or 'response_ridge'")

    return (
        np.where(valid_matrix, score_matrix, 0.0).astype(np.float32),
        response_matrix,
        valid_matrix,
    )


def _workspace_s2_smooth_offset_series(offsets, valid_mask, smoothing_window_samples):
    offsets = np.asarray(offsets, dtype=np.float32).reshape(-1)
    valid_mask = np.asarray(valid_mask).astype(bool).reshape(-1)
    if offsets.size == 0:
        return offsets

    if not np.any(valid_mask):
        return np.zeros_like(offsets, dtype=np.float32)

    if not np.all(valid_mask):
        sample_indices = np.arange(offsets.size, dtype=np.float32)
        offsets = np.interp(
            sample_indices,
            sample_indices[valid_mask],
            offsets[valid_mask],
        ).astype(np.float32)

    smoothing_window_samples = max(1, int(round(smoothing_window_samples)))
    if smoothing_window_samples <= 1 or offsets.size < 3:
        return offsets.astype(np.float32)
    if smoothing_window_samples % 2 == 0:
        smoothing_window_samples += 1
    smoothing_window_samples = min(smoothing_window_samples, offsets.size if offsets.size % 2 == 1 else offsets.size - 1)
    if smoothing_window_samples <= 1:
        return offsets.astype(np.float32)

    pad_width = smoothing_window_samples // 2
    padded_offsets = np.pad(offsets, pad_width, mode="edge")
    kernel = np.ones(smoothing_window_samples, dtype=np.float32) / float(smoothing_window_samples)
    return np.convolve(padded_offsets, kernel, mode="valid").astype(np.float32)


def _normalize_workspace_s2_response_for_curve_trace(response_map, support_mask):
    response_map = np.asarray(response_map, dtype=np.float32)
    finite_mask = np.isfinite(response_map)
    if support_mask is not None and np.asarray(support_mask).shape == response_map.shape:
        finite_mask &= np.asarray(support_mask).astype(bool)
    if not np.any(finite_mask):
        return np.zeros_like(response_map, dtype=np.float32)

    response_values = response_map[finite_mask]
    lower_bound = float(np.percentile(response_values, 5.0))
    upper_bound = float(np.max(response_values))
    if upper_bound <= lower_bound + 1e-6:
        return normalize_workspace_s2_response_for_line_support(response_map, support_mask)

    normalized = (response_map - lower_bound) / (upper_bound - lower_bound)
    normalized = np.clip(normalized, 0.0, 1.0).astype(np.float32)
    normalized[~finite_mask] = 0.0
    return normalized


def trace_workspace_s2_curved_line_centerline(
    response_map,
    workspace_mask,
    line_angle_deg,
    line_rho,
    normal=None,
    search_radius_px=8,
    sample_step_px=4,
    min_response_ratio=0.18,
    smoothing_window_samples=5,
    trace_method="greedy",
    score_mode="response",
    smoothness_weight=0.08,
    center_bias_weight=0.0,
    precomputed_support_map=None,
    precomputed_support_mask=None,
):
    response_map = np.asarray(response_map, dtype=np.float32)
    if response_map.ndim != 2 or response_map.size == 0:
        return {
            "line_angle_deg": normalize_workspace_s2_angle_deg(line_angle_deg),
            "base_rho": float(line_rho),
            "normal": (
                workspace_s2_line_normal_from_angle(line_angle_deg)
                if normal is None
                else np.asarray(normal, dtype=np.float32).reshape(2)
            ).tolist(),
            "direction": workspace_s2_line_direction_from_angle(line_angle_deg).tolist(),
            "polyline_points": [],
            "offsets": [],
            "scores": [],
            "coverage": 0.0,
            "trace_method": trace_method,
            "score_mode": score_mode,
        }

    if precomputed_support_mask is not None and np.asarray(precomputed_support_mask).shape == response_map.shape:
        support_mask = np.asarray(precomputed_support_mask).astype(bool)
    elif workspace_mask is None:
        support_mask = np.ones_like(response_map, dtype=bool)
    else:
        support_mask = np.asarray(workspace_mask).astype(bool)
        if support_mask.shape != response_map.shape:
            support_mask = np.ones_like(response_map, dtype=bool)

    if precomputed_support_map is not None and np.asarray(precomputed_support_map).shape == response_map.shape:
        support_map = np.asarray(precomputed_support_map, dtype=np.float32)
    else:
        support_map = _normalize_workspace_s2_response_for_curve_trace(response_map, support_mask)
    normal = (
        workspace_s2_line_normal_from_angle(line_angle_deg)
        if normal is None
        else np.asarray(normal, dtype=np.float32).reshape(2)
    )
    direction = workspace_s2_line_direction_from_angle(line_angle_deg)
    point_on_line = normal * float(line_rho)
    height, width = response_map.shape[:2]
    t_range = _workspace_s2_line_rect_t_range(point_on_line, direction, width, height)
    if t_range is None:
        return {
            "line_angle_deg": normalize_workspace_s2_angle_deg(line_angle_deg),
            "base_rho": float(line_rho),
            "normal": normal.tolist(),
            "direction": direction.tolist(),
            "polyline_points": [],
            "offsets": [],
            "scores": [],
            "coverage": 0.0,
            "trace_method": trace_method,
            "score_mode": score_mode,
        }

    t_min, t_max = t_range
    sample_step_px = max(1.0, float(sample_step_px))
    t_values = np.arange(np.ceil(t_min), np.floor(t_max) + 1.0, sample_step_px, dtype=np.float32)
    if t_values.size == 0:
        t_values = np.asarray([float((t_min + t_max) * 0.5)], dtype=np.float32)

    search_radius_px = max(0, int(round(search_radius_px)))
    offset_values = np.arange(-search_radius_px, search_radius_px + 1, dtype=np.float32)
    if offset_values.size == 0:
        offset_values = np.asarray([0.0], dtype=np.float32)

    score_matrix, response_matrix, valid_matrix = _workspace_s2_sample_line_offset_score_matrix(
        support_map,
        support_mask,
        point_on_line,
        direction,
        normal,
        t_values,
        offset_values,
        search_radius_px,
        score_mode,
    )
    if search_radius_px > 0 and center_bias_weight > 0.0:
        center_penalty = (np.abs(offset_values) / float(search_radius_px)) ** 2
        score_matrix = score_matrix - (float(center_bias_weight) * center_penalty.reshape(1, -1).astype(np.float32))
        score_matrix = np.where(valid_matrix, score_matrix, 0.0).astype(np.float32)
    if score_matrix.size == 0:
        return {
            "line_angle_deg": normalize_workspace_s2_angle_deg(line_angle_deg),
            "base_rho": float(line_rho),
            "normal": normal.tolist(),
            "direction": direction.tolist(),
            "polyline_points": [],
            "offsets": [],
            "scores": [],
            "coverage": 0.0,
            "trace_method": trace_method,
            "score_mode": score_mode,
        }

    trace_method = str(trace_method or "greedy")
    if trace_method == "dynamic_programming":
        invalid_penalty = -1.0e6
        dp_scores = np.where(valid_matrix, score_matrix, invalid_penalty).astype(np.float32)
        backtrack = np.zeros(dp_scores.shape, dtype=np.int32)
        transition_penalty = float(smoothness_weight) * (
            (offset_values.reshape(-1, 1) - offset_values.reshape(1, -1)) ** 2
        )
        for sample_index in range(1, dp_scores.shape[0]):
            previous_scores = dp_scores[sample_index - 1].reshape(-1, 1) - transition_penalty
            best_previous = np.argmax(previous_scores, axis=0)
            dp_scores[sample_index] += previous_scores[best_previous, np.arange(offset_values.size)]
            backtrack[sample_index] = best_previous.astype(np.int32)

        best_offset_indices = np.zeros(dp_scores.shape[0], dtype=np.int32)
        best_offset_indices[-1] = int(np.argmax(dp_scores[-1]))
        for sample_index in range(dp_scores.shape[0] - 1, 0, -1):
            best_offset_indices[sample_index - 1] = backtrack[sample_index, best_offset_indices[sample_index]]
    elif trace_method == "greedy":
        best_offset_indices = np.argmax(score_matrix, axis=1).astype(np.int32)
    else:
        raise ValueError("trace_method must be 'greedy' or 'dynamic_programming'")

    selected_offsets = offset_values[best_offset_indices].astype(np.float32)
    selected_scores = response_matrix[np.arange(response_matrix.shape[0]), best_offset_indices].astype(np.float32)
    selected_valid = valid_matrix[np.arange(valid_matrix.shape[0]), best_offset_indices]
    selected_strong = selected_valid & (selected_scores >= float(min_response_ratio))
    smoothed_offsets = _workspace_s2_smooth_offset_series(
        selected_offsets,
        selected_valid,
        smoothing_window_samples,
    )
    polyline_array = (
        point_on_line.reshape(1, 2)
        + (t_values.reshape(-1, 1) * direction.reshape(1, 2))
        + (smoothed_offsets.reshape(-1, 1) * normal.reshape(1, 2))
    ).astype(np.float32)
    polyline_array[:, 0] = np.clip(polyline_array[:, 0], 0.0, float(width - 1))
    polyline_array[:, 1] = np.clip(polyline_array[:, 1], 0.0, float(height - 1))
    polyline_points = [[float(point[0]), float(point[1])] for point in polyline_array]

    coverage = (
        float(np.count_nonzero(selected_strong)) / float(selected_strong.size)
        if selected_strong.size
        else 0.0
    )
    return {
        "line_angle_deg": normalize_workspace_s2_angle_deg(line_angle_deg),
        "base_rho": float(line_rho),
        "normal": normal.tolist(),
        "direction": direction.tolist(),
        "polyline_points": polyline_points,
        "t_values": [float(value) for value in t_values],
        "offsets": [float(value) for value in smoothed_offsets],
        "raw_offsets": [float(value) for value in selected_offsets],
        "scores": [float(value) for value in selected_scores],
        "coverage": float(coverage),
        "trace_method": trace_method,
        "score_mode": score_mode,
    }


def build_workspace_s2_curved_line_families(
    response_map,
    workspace_mask,
    line_families,
    stage_key="line_rhos",
    trace_method="greedy",
    score_mode="response",
    search_radius_px=8,
    sample_step_px=4,
    min_response_ratio=0.18,
    smoothing_window_samples=5,
    smoothness_weight=0.08,
):
    response_map = np.asarray(response_map, dtype=np.float32)
    if workspace_mask is None:
        support_mask = np.ones_like(response_map, dtype=bool)
    else:
        support_mask = np.asarray(workspace_mask).astype(bool)
        if support_mask.shape != response_map.shape:
            support_mask = np.ones_like(response_map, dtype=bool)
    support_map = _normalize_workspace_s2_response_for_curve_trace(response_map, support_mask)

    curved_families = []
    for family in line_families or []:
        curved_family = dict(family)
        curved_lines = []
        for line_rho in family.get(stage_key, []):
            curved_lines.append(
                trace_workspace_s2_curved_line_centerline(
                    response_map,
                    workspace_mask,
                    family["line_angle_deg"],
                    line_rho,
                    normal=family.get("normal"),
                    search_radius_px=search_radius_px,
                    sample_step_px=sample_step_px,
                    min_response_ratio=min_response_ratio,
                    smoothing_window_samples=smoothing_window_samples,
                    trace_method=trace_method,
                    score_mode=score_mode,
                    smoothness_weight=smoothness_weight,
                    precomputed_support_map=support_map,
                    precomputed_support_mask=support_mask,
                )
            )
        curved_family["curved_lines"] = curved_lines
        curved_families.append(curved_family)
    return curved_families


def _workspace_s2_segment_intersection(first_start, first_end, second_start, second_end):
    first_start = np.asarray(first_start, dtype=np.float32).reshape(2)
    first_end = np.asarray(first_end, dtype=np.float32).reshape(2)
    second_start = np.asarray(second_start, dtype=np.float32).reshape(2)
    second_end = np.asarray(second_end, dtype=np.float32).reshape(2)
    first_delta = first_end - first_start
    second_delta = second_end - second_start
    matrix = np.asarray(
        [
            [float(first_delta[0]), -float(second_delta[0])],
            [float(first_delta[1]), -float(second_delta[1])],
        ],
        dtype=np.float32,
    )
    determinant = float(np.linalg.det(matrix))
    if abs(determinant) <= 1e-6:
        return None

    right_hand_side = second_start - first_start
    segment_params = np.linalg.solve(matrix, right_hand_side)
    first_t = float(segment_params[0])
    second_t = float(segment_params[1])
    tolerance = 1e-5
    if -tolerance <= first_t <= 1.0 + tolerance and -tolerance <= second_t <= 1.0 + tolerance:
        point = first_start + (first_delta * first_t)
        return [float(point[0]), float(point[1])]
    return None


def _workspace_s2_polyline_intersection(first_points, second_points):
    first_points = np.asarray(first_points, dtype=np.float32).reshape(-1, 2)
    second_points = np.asarray(second_points, dtype=np.float32).reshape(-1, 2)
    if first_points.shape[0] < 2 or second_points.shape[0] < 2:
        return None

    first_start = first_points[:-1]
    first_delta = first_points[1:] - first_start
    second_start = second_points[:-1]
    second_delta = second_points[1:] - second_start
    first_start_grid = first_start[:, None, :]
    first_delta_grid = first_delta[:, None, :]
    second_start_grid = second_start[None, :, :]
    second_delta_grid = second_delta[None, :, :]
    relative_start = second_start_grid - first_start_grid

    denominator = (
        first_delta_grid[:, :, 0] * second_delta_grid[:, :, 1]
        - first_delta_grid[:, :, 1] * second_delta_grid[:, :, 0]
    )
    valid_denominator = np.abs(denominator) > 1e-6
    if not np.any(valid_denominator):
        return None

    first_t = (
        relative_start[:, :, 0] * second_delta_grid[:, :, 1]
        - relative_start[:, :, 1] * second_delta_grid[:, :, 0]
    ) / np.where(valid_denominator, denominator, 1.0)
    second_t = (
        relative_start[:, :, 0] * first_delta_grid[:, :, 1]
        - relative_start[:, :, 1] * first_delta_grid[:, :, 0]
    ) / np.where(valid_denominator, denominator, 1.0)
    tolerance = 1e-5
    intersection_mask = (
        valid_denominator
        & (first_t >= -tolerance)
        & (first_t <= 1.0 + tolerance)
        & (second_t >= -tolerance)
        & (second_t <= 1.0 + tolerance)
    )
    if not np.any(intersection_mask):
        return None

    first_indices, second_indices = np.where(intersection_mask)
    best_index = int(np.argmin(first_indices + second_indices))
    first_index = int(first_indices[best_index])
    second_index = int(second_indices[best_index])
    point = first_start[first_index] + (first_delta[first_index] * float(first_t[first_index, second_index]))
    return [float(point[0]), float(point[1])]


def intersect_workspace_s2_curved_line_families(
    first_family,
    second_family,
    rectified_width,
    rectified_height,
    margin_px=0.0,
):
    first_lines = first_family.get("curved_lines") or []
    second_lines = second_family.get("curved_lines") or []
    if not first_lines or not second_lines:
        return []

    margin_px = max(0.0, float(margin_px))
    min_x = -margin_px
    max_x = float(rectified_width - 1) + margin_px
    min_y = -margin_px
    max_y = float(rectified_height - 1) + margin_px

    intersections = []
    for second_line in second_lines:
        for first_line in first_lines:
            point = _workspace_s2_polyline_intersection(
                first_line.get("polyline_points", []),
                second_line.get("polyline_points", []),
            )
            if point is None:
                continue
            point_x = float(point[0])
            point_y = float(point[1])
            if min_x <= point_x <= max_x and min_y <= point_y <= max_y:
                intersections.append([
                    float(np.clip(point_x, 0.0, float(rectified_width - 1))),
                    float(np.clip(point_y, 0.0, float(rectified_height - 1))),
                ])
    return intersections


def _workspace_s2_clip_oriented_line_to_rect(line_angle_deg, normal, line_rho, rectified_width, rectified_height):
    normal = np.asarray(normal, dtype=np.float32).reshape(2)
    direction = workspace_s2_line_direction_from_angle(line_angle_deg)
    point_on_line = normal * float(line_rho)
    t_range = _workspace_s2_line_rect_t_range(
        point_on_line,
        direction,
        int(rectified_width),
        int(rectified_height),
    )
    if t_range is None:
        return None

    t_min, t_max = t_range
    start_point = point_on_line + (direction * float(t_min))
    end_point = point_on_line + (direction * float(t_max))
    return [
        [float(start_point[0]), float(start_point[1])],
        [float(end_point[0]), float(end_point[1])],
    ]


def build_workspace_s2_oriented_projective_line_segments(
    corner_pixels,
    rectified_width,
    rectified_height,
    line_families,
):
    if corner_pixels is None or len(corner_pixels) != 4:
        return {}

    source_points = np.asarray(
        sort_polygon_points_clockwise(
            [[float(point[0]), float(point[1])] for point in corner_pixels]
        ),
        dtype=np.float32,
    )
    destination_points = np.asarray(
        [
            [0.0, 0.0],
            [float(rectified_width - 1), 0.0],
            [float(rectified_width - 1), float(rectified_height - 1)],
            [0.0, float(rectified_height - 1)],
        ],
        dtype=np.float32,
    )
    inverse_h = cv2.getPerspectiveTransform(destination_points, source_points)

    segments_by_family = {}
    for family_index, family in enumerate(line_families or []):
        family_key = f"family_{family_index}"
        segments_by_family[family_key] = []
        for line_rho in family.get("line_rhos", []):
            clipped_segment = _workspace_s2_clip_oriented_line_to_rect(
                family["line_angle_deg"],
                family["normal"],
                line_rho,
                rectified_width,
                rectified_height,
            )
            if clipped_segment is None:
                continue

            mapped_points = map_workspace_s2_rectified_points_to_image(clipped_segment, inverse_h)
            if len(mapped_points) == 2:
                segments_by_family[family_key].append((mapped_points[0], mapped_points[1]))

    return segments_by_family


def build_workspace_s2_bbox(workspace_mask):
    if workspace_mask is None or not np.any(workspace_mask):
        return None
    y_coords, x_coords = np.where(workspace_mask > 0)
    if x_coords.size == 0 or y_coords.size == 0:
        return None
    return (
        int(x_coords.min()),
        int(y_coords.min()),
        int(x_coords.max()),
        int(y_coords.max()),
    )


def build_workspace_s2_axis_profile(response_map, workspace_mask, axis):
    response_map = np.asarray(response_map, dtype=np.float32)
    workspace_mask = np.asarray(workspace_mask, dtype=np.uint8)
    weighted_sum = np.sum(response_map * workspace_mask, axis=axis)
    valid_count = np.sum(workspace_mask, axis=axis)
    return np.divide(
        weighted_sum,
        valid_count,
        out=np.zeros_like(weighted_sum, dtype=np.float32),
        where=valid_count > 0,
    )


def normalize_workspace_s2_response(response_map, valid_mask, lower_percentile=5.0, upper_percentile=95.0):
    response_map = np.asarray(response_map, dtype=np.float32)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    normalized = np.zeros_like(response_map, dtype=np.float32)
    if response_map.size == 0 or not np.any(valid_mask):
        return normalized

    valid_values = response_map[valid_mask]
    lower_bound = float(np.percentile(valid_values, lower_percentile))
    upper_bound = float(np.percentile(valid_values, upper_percentile))
    if upper_bound <= lower_bound + 1e-6:
        upper_bound = float(np.max(valid_values))
        lower_bound = float(np.min(valid_values))
    if upper_bound <= lower_bound + 1e-6:
        normalized[valid_mask] = 1.0
        return normalized

    normalized = (response_map - lower_bound) / (upper_bound - lower_bound)
    normalized = np.clip(normalized, 0.0, 1.0).astype(np.float32)
    normalized[~valid_mask] = 0.0
    return normalized


def build_workspace_s2_rectified_geometry(corner_pixels, corner_world_camera_frame=None, resolution_mm_per_px=5.0):
    if corner_pixels is None or len(corner_pixels) != 4:
        return None

    pixel_points = [[float(point[0]), float(point[1])] for point in corner_pixels]
    ordered_indices = sort_polygon_indices_clockwise(pixel_points)
    ordered_points = [pixel_points[index] for index in ordered_indices]
    source_points = np.asarray(ordered_points, dtype=np.float32)
    rectified_width = None
    rectified_height = None

    if corner_world_camera_frame is not None and len(corner_world_camera_frame) == 4:
        ordered_world_points = [
            [
                float(corner_world_camera_frame[index][0]),
                float(corner_world_camera_frame[index][1]),
                float(corner_world_camera_frame[index][2]),
            ]
            for index in ordered_indices
        ]
        world_points = np.asarray(ordered_world_points, dtype=np.float32)
        top_width_mm = float(np.linalg.norm(world_points[1] - world_points[0]))
        bottom_width_mm = float(np.linalg.norm(world_points[2] - world_points[3]))
        left_height_mm = float(np.linalg.norm(world_points[3] - world_points[0]))
        right_height_mm = float(np.linalg.norm(world_points[2] - world_points[1]))
        if resolution_mm_per_px > 0:
            rectified_width = max(
                int(round(((top_width_mm + bottom_width_mm) / 2.0) / resolution_mm_per_px)),
                2,
            )
            rectified_height = max(
                int(round(((left_height_mm + right_height_mm) / 2.0) / resolution_mm_per_px)),
                2,
            )

    if rectified_width is None or rectified_height is None:
        top_width = float(np.linalg.norm(source_points[1] - source_points[0]))
        bottom_width = float(np.linalg.norm(source_points[2] - source_points[3]))
        left_height = float(np.linalg.norm(source_points[3] - source_points[0]))
        right_height = float(np.linalg.norm(source_points[2] - source_points[1]))
        rectified_width = max(int(round((top_width + bottom_width) / 2.0)), 2)
        rectified_height = max(int(round((left_height + right_height) / 2.0)), 2)

    destination_points = np.asarray(
        [
            [0.0, 0.0],
            [float(rectified_width - 1), 0.0],
            [float(rectified_width - 1), float(rectified_height - 1)],
            [0.0, float(rectified_height - 1)],
        ],
        dtype=np.float32,
    )

    forward_h = cv2.getPerspectiveTransform(source_points, destination_points)
    inverse_h = cv2.getPerspectiveTransform(destination_points, source_points)
    return {
        "corner_pixels": ordered_points,
        "rectified_width": rectified_width,
        "rectified_height": rectified_height,
        "resolution_mm_per_px": float(resolution_mm_per_px),
        "forward_h": forward_h,
        "inverse_h": inverse_h,
    }


def map_workspace_s2_rectified_points_to_image(rectified_points, inverse_h):
    if not rectified_points:
        return []

    points_array = np.asarray(rectified_points, dtype=np.float32).reshape((-1, 1, 2))
    mapped_points = cv2.perspectiveTransform(points_array, inverse_h).reshape((-1, 2))
    return [
        [int(round(float(point[0]))), int(round(float(point[1])))]
        for point in mapped_points
    ]


def build_workspace_s2_projective_line_segments(
    corner_pixels,
    rectified_width,
    rectified_height,
    vertical_lines,
    horizontal_lines,
):
    if corner_pixels is None or len(corner_pixels) != 4:
        return {"vertical": [], "horizontal": []}

    source_points = np.asarray(
        sort_polygon_points_clockwise(
            [[float(point[0]), float(point[1])] for point in corner_pixels]
        ),
        dtype=np.float32,
    )
    destination_points = np.asarray(
        [
            [0.0, 0.0],
            [float(rectified_width - 1), 0.0],
            [float(rectified_width - 1), float(rectified_height - 1)],
            [0.0, float(rectified_height - 1)],
        ],
        dtype=np.float32,
    )
    inverse_h = cv2.getPerspectiveTransform(destination_points, source_points)

    def map_segment(segment_points):
        mapped_points = map_workspace_s2_rectified_points_to_image(segment_points, inverse_h)
        if len(mapped_points) != 2:
            return None
        return (mapped_points[0], mapped_points[1])

    vertical_segments = []
    for vertical_x in vertical_lines:
        segment = map_segment(
            [
                [float(vertical_x), 0.0],
                [float(vertical_x), float(rectified_height - 1)],
            ]
        )
        if segment is not None:
            vertical_segments.append(segment)

    horizontal_segments = []
    for horizontal_y in horizontal_lines:
        segment = map_segment(
            [
                [0.0, float(horizontal_y)],
                [float(rectified_width - 1), float(horizontal_y)],
            ]
        )
        if segment is not None:
            horizontal_segments.append(segment)

    return {
        "vertical": vertical_segments,
        "horizontal": horizontal_segments,
    }
