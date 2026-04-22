import cv2
import numpy as np


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


def build_workspace_s2_line_positions(start_pixel, end_pixel, period_px, phase_px):
    start_pixel = int(round(start_pixel))
    end_pixel = int(round(end_pixel))
    period_px = int(round(period_px))
    phase_px = int(round(phase_px))
    if period_px <= 0 or end_pixel < start_pixel:
        return []

    first_position = start_pixel + (phase_px % period_px)
    return list(range(first_position, end_pixel + 1, period_px))


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


def build_workspace_s2_rectified_geometry(corner_pixels, corner_world_cabin_frame=None, resolution_mm_per_px=5.0):
    if corner_pixels is None or len(corner_pixels) != 4:
        return None

    pixel_points = [[float(point[0]), float(point[1])] for point in corner_pixels]
    ordered_indices = sort_polygon_indices_clockwise(pixel_points)
    ordered_points = [pixel_points[index] for index in ordered_indices]
    source_points = np.asarray(ordered_points, dtype=np.float32)
    rectified_width = None
    rectified_height = None

    if corner_world_cabin_frame is not None and len(corner_world_cabin_frame) == 4:
        ordered_world_points = [
            [
                float(corner_world_cabin_frame[index][0]),
                float(corner_world_cabin_frame[index][1]),
                float(corner_world_cabin_frame[index][2]),
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
