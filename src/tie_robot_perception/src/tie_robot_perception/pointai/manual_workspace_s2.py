"""pointAI 拆分后的职责模块。"""
import json
import math
import os
import time
import yaml

import cv2
import numpy as np
import rospy
import torch
from cv2 import ximgproc
from cv2.ppf_match_3d import Pose3D
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Vector3
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from sklearn.cluster import DBSCAN
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32
from std_srvs.srv import Trigger, TriggerResponse

from tie_robot_msgs.msg import PointCoords, PointsArray, motion
from tie_robot_msgs.srv import (
    PlaneDetection,
    PlaneDetectionResponse,
    ProcessImage,
    ProcessImageResponse,
    SingleMove,
    SingleMoveRequest,
    linear_module_move,
    linear_module_moveRequest,
    linear_module_moveResponse,
)
from tie_robot_perception.perception.workspace_s2 import (
    build_workspace_s2_axis_profile as workspace_s2_build_axis_profile,
    build_workspace_s2_bbox as workspace_s2_build_bbox,
    build_workspace_s2_line_positions as workspace_s2_build_line_positions,
    build_workspace_s2_oriented_line_families as workspace_s2_build_oriented_line_families,
    build_workspace_s2_oriented_projective_line_segments as workspace_s2_build_oriented_projective_line_segments,
    build_workspace_s2_projective_line_segments as workspace_s2_build_projective_line_segments,
    build_workspace_s2_rectified_geometry as workspace_s2_build_rectified_geometry,
    build_workspace_s2_structural_edge_suppression_mask as workspace_s2_build_structural_edge_suppression_mask,
    estimate_workspace_s2_period_and_phase as workspace_s2_estimate_period_and_phase,
    filter_workspace_s2_rectified_points_outside_mask as workspace_s2_filter_rectified_points_outside_mask,
    intersect_workspace_s2_oriented_line_families as workspace_s2_intersect_oriented_line_families,
    map_workspace_s2_rectified_points_to_image as workspace_s2_map_rectified_points_to_image,
    normalize_workspace_s2_response as workspace_s2_normalize_response,
    prune_workspace_s2_line_positions_by_spacing as workspace_s2_prune_line_positions_by_spacing,
    refine_workspace_s2_line_positions_to_local_peaks as workspace_s2_refine_line_positions_to_local_peaks,
    score_workspace_s2_oriented_line_family_result as workspace_s2_score_oriented_line_family_result,
    select_workspace_s2_continuous_line_positions as workspace_s2_select_continuous_line_positions,
    select_workspace_s2_peak_supported_line_positions as workspace_s2_select_peak_supported_line_positions,
    smooth_workspace_s2_profile as workspace_s2_smooth_profile,
    sort_polygon_indices_clockwise as workspace_s2_sort_polygon_indices_clockwise,
    sort_polygon_points_clockwise as workspace_s2_sort_polygon_points_clockwise,
)
from .constants import *


def prepare_manual_workspace_s2_inputs(self):
    manual_workspace = self.load_manual_workspace_quad()
    workspace_mask = self.get_manual_workspace_pixel_mask()
    if manual_workspace is None or workspace_mask is None:
        return None

    if not self.ensure_raw_world_channels():
        return None

    rectified_geometry = self.build_workspace_s2_rectified_geometry(
        manual_workspace["corner_pixels"],
        manual_workspace.get("corner_world_camera_frame"),
    )
    if rectified_geometry is None:
        return None

    raw_depth = self.depth_v.astype(np.float32)
    valid_mask = np.isfinite(raw_depth) & (raw_depth != 0.0)
    if np.count_nonzero(valid_mask & workspace_mask.astype(bool)) < 100:
        return None

    rectified_size = (
        rectified_geometry["rectified_width"],
        rectified_geometry["rectified_height"],
    )
    rectified_depth = cv2.warpPerspective(
        raw_depth,
        rectified_geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_LINEAR,
    ).astype(np.float32)
    rectified_valid_mask = cv2.warpPerspective(
        valid_mask.astype(np.uint8),
        rectified_geometry["forward_h"],
        rectified_size,
        flags=cv2.INTER_NEAREST,
    ).astype(bool)
    if np.count_nonzero(rectified_valid_mask) < 100:
        return None

    median_depth = float(np.median(rectified_depth[rectified_valid_mask]))
    filled_depth = np.where(rectified_valid_mask, rectified_depth, median_depth).astype(np.float32)
    background_depth = cv2.GaussianBlur(filled_depth, (0, 0), sigmaX=11.0, sigmaY=11.0)

    depth_response_variants = [
        background_depth - filled_depth,
        filled_depth - background_depth,
    ]
    infrared_response_variants = []
    rectified_mask_uint8 = rectified_valid_mask.astype(np.uint8)

    infrared_vertical_profile = None
    infrared_horizontal_profile = None
    infrared_response = None
    infrared_image = getattr(self, "image_infrared_copy", None)
    if infrared_image is not None:
        infrared_image = np.asarray(infrared_image)
        if infrared_image.ndim == 3:
            infrared_image = cv2.cvtColor(infrared_image, cv2.COLOR_BGR2GRAY)
        if infrared_image.shape[:2] == raw_depth.shape[:2]:
            rectified_infrared = cv2.warpPerspective(
                infrared_image.astype(np.float32),
                rectified_geometry["forward_h"],
                rectified_size,
                flags=cv2.INTER_LINEAR,
            ).astype(np.float32)
            infrared_background = cv2.GaussianBlur(rectified_infrared, (0, 0), sigmaX=7.0, sigmaY=7.0)
            infrared_dark_line_response = infrared_background - rectified_infrared
            infrared_response_variants.append(infrared_dark_line_response)
            infrared_response_variants.append(rectified_infrared - infrared_background)
            infrared_response = self.normalize_workspace_s2_response(
                infrared_dark_line_response,
                rectified_valid_mask,
            )
            infrared_vertical_profile = self.build_workspace_s2_axis_profile(
                infrared_response,
                rectified_mask_uint8,
                axis=0,
            )
            infrared_horizontal_profile = self.build_workspace_s2_axis_profile(
                infrared_response,
                rectified_mask_uint8,
                axis=1,
            )

    best_variant = None

    def select_best_supported_variant(candidate_variants):
        selected_variant = None
        for response_variant in candidate_variants:
            response_variant = response_variant.astype(np.float32)
            normalized_response = self.normalize_workspace_s2_response(response_variant, rectified_valid_mask)
            line_families = self.build_workspace_s2_oriented_line_families(
                normalized_response,
                rectified_mask_uint8,
                min_period=10,
                max_period=30,
            )
            if len(line_families) < 2 or any(len(family.get("line_rhos", [])) < 2 for family in line_families[:2]):
                continue

            vertical_profile = self.build_workspace_s2_axis_profile(normalized_response, rectified_mask_uint8, axis=0)
            horizontal_profile = self.build_workspace_s2_axis_profile(normalized_response, rectified_mask_uint8, axis=1)
            vertical_estimate = self.estimate_workspace_s2_period_and_phase(vertical_profile, min_period=10, max_period=30)
            horizontal_estimate = self.estimate_workspace_s2_period_and_phase(horizontal_profile, min_period=10, max_period=30)

            combined_score = self.score_workspace_s2_oriented_line_family_result(line_families[:2])
            if selected_variant is None or combined_score > selected_variant["combined_score"]:
                selected_variant = {
                    "response": normalized_response,
                    "line_families": line_families,
                    "vertical_profile": vertical_profile,
                    "horizontal_profile": horizontal_profile,
                    "vertical_estimate": vertical_estimate,
                    "horizontal_estimate": horizontal_estimate,
                    "combined_score": combined_score,
                }
        return selected_variant

    for response_variant in depth_response_variants:
        best_variant = select_best_supported_variant([response_variant])
        if best_variant is not None:
            break
    if best_variant is None:
        for response_variant in infrared_response_variants:
            best_variant = select_best_supported_variant([response_variant])
            if best_variant is not None:
                break

    if best_variant is None:
        return None

    structural_edge_response = infrared_response if infrared_response is not None else best_variant["response"]
    return {
        "manual_workspace": manual_workspace,
        "workspace_mask": workspace_mask.astype(np.uint8),
        "workspace_bbox": self.build_workspace_s2_bbox(workspace_mask),
        "rectified_geometry": rectified_geometry,
        "response_crop": best_variant["response"],
        "structural_edge_response_crop": structural_edge_response,
        "workspace_mask_crop": rectified_valid_mask.astype(np.uint8),
        "line_families": best_variant["line_families"],
        "vertical_profile": best_variant["vertical_profile"],
        "horizontal_profile": best_variant["horizontal_profile"],
        "vertical_infrared_profile": infrared_vertical_profile,
        "horizontal_infrared_profile": infrared_horizontal_profile,
        "vertical_estimate": best_variant["vertical_estimate"],
        "horizontal_estimate": best_variant["horizontal_estimate"],
    }


def manual_workspace_s2_axis_distance(left_estimate, right_estimate):
    left_period = max(1, int(round(float(left_estimate.get("period", 1)))))
    right_period = max(1, int(round(float(right_estimate.get("period", 1)))))
    period_delta = abs(left_period - right_period)
    reference_period = max(1, int(round((left_period + right_period) / 2.0)))
    left_phase = int(round(float(left_estimate.get("phase", 0)))) % reference_period
    right_phase = int(round(float(right_estimate.get("phase", 0)))) % reference_period
    raw_phase_delta = abs(left_phase - right_phase)
    phase_delta = min(raw_phase_delta, reference_period - raw_phase_delta)
    return float((period_delta * 3.0) + phase_delta)


def manual_workspace_s2_candidate_distance(left_candidate, right_candidate):
    return (
        manual_workspace_s2_axis_distance(left_candidate["vertical_estimate"], right_candidate["vertical_estimate"])
        + manual_workspace_s2_axis_distance(left_candidate["horizontal_estimate"], right_candidate["horizontal_estimate"])
    )


def manual_workspace_s2_candidate_score(candidate):
    line_families = candidate.get("line_families") or []
    if len(line_families) >= 2:
        return workspace_s2_score_oriented_line_family_result(line_families[:2])
    return float(candidate["vertical_estimate"].get("score", 0.0)) + float(candidate["horizontal_estimate"].get("score", 0.0))


def merge_manual_workspace_s2_profile_estimate(candidates, profile_key, estimate_key):
    profiles = []
    for candidate in candidates:
        profile = candidate.get(profile_key)
        if profile is None:
            profile = candidate.get(estimate_key, {}).get("profile")
        if profile is None:
            continue
        profile = np.asarray(profile, dtype=np.float32).reshape(-1)
        if profile.size == 0:
            continue
        profiles.append(profile)

    if len(profiles) < 2:
        return None

    profile_size = profiles[0].size
    same_size_profiles = [profile for profile in profiles if profile.size == profile_size]
    if len(same_size_profiles) < 2:
        return None

    merged_profile = np.median(np.vstack(same_size_profiles), axis=0).astype(np.float32)
    return estimate_workspace_s2_period_and_phase(
        merged_profile,
        min_period=10,
        max_period=30,
    )


def merge_manual_workspace_s2_raw_profile(candidates, profile_key):
    profiles = []
    for candidate in candidates:
        profile = candidate.get(profile_key)
        if profile is None:
            continue
        profile = np.asarray(profile, dtype=np.float32).reshape(-1)
        if profile.size == 0:
            continue
        profiles.append(profile)

    if len(profiles) < 2:
        return profiles[0] if profiles else None

    profile_size = profiles[0].size
    same_size_profiles = [profile for profile in profiles if profile.size == profile_size]
    if len(same_size_profiles) < 2:
        return same_size_profiles[0] if same_size_profiles else None

    return np.median(np.vstack(same_size_profiles), axis=0).astype(np.float32)


def merge_manual_workspace_s2_candidate_profiles(base_candidate, candidates):
    merged_candidate = dict(base_candidate)
    vertical_estimate = merge_manual_workspace_s2_profile_estimate(
        candidates,
        "vertical_profile",
        "vertical_estimate",
    )
    horizontal_estimate = merge_manual_workspace_s2_profile_estimate(
        candidates,
        "horizontal_profile",
        "horizontal_estimate",
    )
    if vertical_estimate is not None:
        merged_candidate["vertical_estimate"] = vertical_estimate
        merged_candidate["vertical_profile"] = vertical_estimate.get("profile")
    if horizontal_estimate is not None:
        merged_candidate["horizontal_estimate"] = horizontal_estimate
        merged_candidate["horizontal_profile"] = horizontal_estimate.get("profile")
    vertical_infrared_profile = merge_manual_workspace_s2_raw_profile(candidates, "vertical_infrared_profile")
    horizontal_infrared_profile = merge_manual_workspace_s2_raw_profile(candidates, "horizontal_infrared_profile")
    if vertical_infrared_profile is not None:
        merged_candidate["vertical_infrared_profile"] = vertical_infrared_profile
    if horizontal_infrared_profile is not None:
        merged_candidate["horizontal_infrared_profile"] = horizontal_infrared_profile
    return merged_candidate


def select_stable_manual_workspace_s2_inputs(candidates):
    oriented_candidates = [
        candidate for candidate in candidates
        if candidate is not None
        and len(candidate.get("line_families") or []) >= 2
        and all(len(family.get("line_rhos", [])) >= 2 for family in (candidate.get("line_families") or [])[:2])
    ]
    if oriented_candidates:
        return max(oriented_candidates, key=manual_workspace_s2_candidate_score)

    valid_candidates = [
        candidate for candidate in candidates
        if candidate is not None
        and candidate.get("vertical_estimate") is not None
        and candidate.get("horizontal_estimate") is not None
    ]
    if not valid_candidates:
        return None
    if len(valid_candidates) == 1:
        return valid_candidates[0]

    def candidate_rank(candidate):
        consensus_distance = sum(
            manual_workspace_s2_candidate_distance(candidate, other_candidate)
            for other_candidate in valid_candidates
        )
        return (consensus_distance, -manual_workspace_s2_candidate_score(candidate))

    base_candidate = min(valid_candidates, key=candidate_rank)
    return merge_manual_workspace_s2_candidate_profiles(base_candidate, valid_candidates)


def collect_stable_manual_workspace_s2_inputs(self):
    sample_count = max(1, int(getattr(self, "manual_workspace_s2_stability_samples", 5)))
    sample_interval_sec = max(0.0, float(getattr(self, "manual_workspace_s2_stability_interval_sec", 0.06)))
    candidates = []
    for sample_index in range(sample_count):
        candidate = self.prepare_manual_workspace_s2_inputs()
        if candidate is not None:
            candidates.append(candidate)
        if sample_index + 1 < sample_count and sample_interval_sec > 0.0 and not rospy.is_shutdown():
            rospy.sleep(sample_interval_sec)
    return select_stable_manual_workspace_s2_inputs(candidates)


def is_valid_manual_workspace_s2_camera_coord(camera_coord):
    if camera_coord is None or len(camera_coord) < 3:
        return False
    try:
        x_value = float(camera_coord[0])
        y_value = float(camera_coord[1])
        z_value = float(camera_coord[2])
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(x_value)
        and math.isfinite(y_value)
        and math.isfinite(z_value)
        and x_value != 0.0
        and y_value != 0.0
        and z_value != 0.0
    )


def log_manual_workspace_s2_camera_distance(
    self,
    point_index,
    pixel_x,
    pixel_y,
    camera_coord,
):
    del self
    if not is_valid_manual_workspace_s2_camera_coord(camera_coord):
        return

    camera_x = float(camera_coord[0])
    camera_y = float(camera_coord[1])
    camera_z = float(camera_coord[2])
    camera_distance = math.sqrt((camera_x * camera_x) + (camera_y * camera_y) + (camera_z * camera_z))
    rospy.loginfo(
        "pointAI tie point camera distance: idx=%d source_frame=Scepter_depth_frame "
        "pixel=(%d,%d) camera_xyz_mm=(%.1f,%.1f,%.1f) camera_depth_z_mm=%.1f camera_distance_mm=%.1f",
        int(point_index),
        int(pixel_x),
        int(pixel_y),
        camera_x,
        camera_y,
        camera_z,
        camera_z,
        camera_distance,
    )


def build_manual_workspace_s2_points_array(
    self,
    intersection_pixels,
    workspace_mask,
    rectified_intersections=None,
    rectified_geometry=None,
    manual_workspace=None,
):
    points_array_msg = PointsArray()
    points_array_msg.PointCoordinatesArray = []
    display_points = []

    if workspace_mask is None:
        points_array_msg.count = 0
        return points_array_msg, display_points

    point_index = 1
    del rectified_intersections, rectified_geometry, manual_workspace
    for intersection_pixel in intersection_pixels:
        pixel_x = int(intersection_pixel[0])
        pixel_y = int(intersection_pixel[1])
        if (
            pixel_y < 0 or pixel_x < 0
            or pixel_y >= workspace_mask.shape[0]
            or pixel_x >= workspace_mask.shape[1]
            or not workspace_mask[pixel_y, pixel_x]
        ):
            continue

        camera_coord, _, _ = self.get_valid_world_coord_near_pixel(pixel_x, pixel_y)
        if not is_valid_manual_workspace_s2_camera_coord(camera_coord):
            continue
        log_manual_workspace_s2_camera_distance(
            self,
            point_index,
            pixel_x,
            pixel_y,
            camera_coord,
        )

        point_msg = PointCoords()
        point_msg.is_shuiguan = False
        point_msg.Angle = -45
        point_msg.idx = point_index
        point_msg.Pix_coord = [pixel_x, pixel_y]
        point_msg.World_coord = [
            float(camera_coord[0]),
            float(camera_coord[1]),
            float(camera_coord[2]),
        ]
        points_array_msg.PointCoordinatesArray.append(point_msg)
        display_points.append(
            (
                point_index,
                [pixel_x, pixel_y],
                [
                    float(camera_coord[0]),
                    float(camera_coord[1]),
                    float(camera_coord[2]),
                ],
                "selected",
                "S2",
            )
        )
        point_index += 1

    points_array_msg.count = len(points_array_msg.PointCoordinatesArray)
    return points_array_msg, display_points


def build_manual_workspace_s2_phase_lock_key(s2_inputs, vertical_lines, horizontal_lines):
    manual_workspace = s2_inputs.get("manual_workspace") or {}
    rectified_geometry = s2_inputs.get("rectified_geometry") or {}
    corner_pixels = manual_workspace.get("corner_pixels") or []
    rounded_corners = tuple(
        (int(round(float(point[0]))), int(round(float(point[1]))))
        for point in corner_pixels
    )
    return (
        rounded_corners,
        int(rectified_geometry.get("rectified_width", 0)),
        int(rectified_geometry.get("rectified_height", 0)),
    )


def apply_manual_workspace_s2_phase_lock(self, s2_inputs, vertical_lines, horizontal_lines):
    vertical_lines = [int(round(line)) for line in vertical_lines]
    horizontal_lines = [int(round(line)) for line in horizontal_lines]
    lock_key = build_manual_workspace_s2_phase_lock_key(s2_inputs, vertical_lines, horizontal_lines)
    existing_lock = getattr(self, "manual_workspace_s2_phase_lock", None)
    if existing_lock is not None and existing_lock.get("key") == lock_key:
        return (
            list(existing_lock.get("vertical_lines", vertical_lines)),
            list(existing_lock.get("horizontal_lines", horizontal_lines)),
        )

    self.manual_workspace_s2_phase_lock = {
        "key": lock_key,
        "vertical_lines": list(vertical_lines),
        "horizontal_lines": list(horizontal_lines),
    }
    return vertical_lines, horizontal_lines


def run_manual_workspace_s2_pipeline(self, publish=False):
    start_time = time.perf_counter()
    s2_inputs = self.collect_stable_manual_workspace_s2_inputs()
    if s2_inputs is None:
        return {
            "success": False,
            "message": "当前工作区或原始世界坐标不足，无法运行S2",
            "point_coords": None,
            "result_image": None,
        }

    rectified_width = s2_inputs["rectified_geometry"]["rectified_width"]
    rectified_height = s2_inputs["rectified_geometry"]["rectified_height"]
    line_families = self.build_workspace_s2_oriented_line_families(
        s2_inputs["response_crop"],
        s2_inputs["workspace_mask_crop"],
        min_period=10,
        max_period=30,
    )
    if len(line_families) < 2:
        line_families = list(s2_inputs.get("line_families") or [])
    if len(line_families) < 2 or any(len(family.get("line_rhos", [])) < 2 for family in line_families[:2]):
        return {
            "success": False,
            "message": "manual workspace S2 oriented continuous bar-supported line families insufficient",
            "point_count": 0,
            "point_coords": None,
            "result_image": None,
        }

    rectified_intersections = self.intersect_workspace_s2_oriented_line_families(
        line_families[0],
        line_families[1],
        rectified_width,
        rectified_height,
    )
    beam_mask = self.build_workspace_s2_structural_edge_suppression_mask(
        s2_inputs.get("structural_edge_response_crop", s2_inputs["response_crop"]),
        s2_inputs["workspace_mask_crop"],
    )
    rectified_intersections = self.filter_workspace_s2_rectified_points_outside_mask(
        rectified_intersections,
        beam_mask,
        margin_px=2,
    )
    if not rectified_intersections:
        return {
            "success": False,
            "message": "manual workspace S2 oriented line families did not intersect inside workspace",
            "point_count": 0,
            "point_coords": None,
            "result_image": None,
        }
    image_intersections = self.map_workspace_s2_rectified_points_to_image(
        rectified_intersections,
        s2_inputs["rectified_geometry"]["inverse_h"],
    )
    line_segments = self.build_workspace_s2_oriented_projective_line_segments(
        s2_inputs["manual_workspace"]["corner_pixels"],
        rectified_width,
        rectified_height,
        line_families[:2],
    )

    points_array_msg, display_points = self.build_manual_workspace_s2_points_array(
        image_intersections,
        s2_inputs["workspace_mask"],
        rectified_intersections=rectified_intersections,
        rectified_geometry=s2_inputs["rectified_geometry"],
        manual_workspace=s2_inputs["manual_workspace"],
    )
    result_image = self.render_manual_workspace_s2_result_image(
        s2_inputs["workspace_mask"],
        line_segments,
        display_points,
    )
    if publish:
        self.publish_manual_workspace_s2_result(result_image, points_array_msg)

    workspace_bbox = s2_inputs["workspace_bbox"] or (0, 0, 0, 0)
    min_x, min_y, max_x, max_y = workspace_bbox
    family_summaries = ", ".join(
        "theta=%.1f period=%s lines=%d" % (
            float(family.get("line_angle_deg", 0.0)),
            family.get("estimate", {}).get("period", "-"),
            len(family.get("line_rhos", [])),
        )
        for family in line_families[:2]
    )
    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    rospy.loginfo(
        "pointAI manual workspace S2: bbox=(%d,%d,%d,%d), rectified=(%d,%d), oriented_families=[%s], points=%d, elapsed_ms=%.1f",
        min_x,
        min_y,
        max_x,
        max_y,
        rectified_width,
        rectified_height,
        family_summaries,
        points_array_msg.count,
        elapsed_ms,
    )
    return {
        "success": True,
        "message": "manual workspace S2 finished",
        "point_count": points_array_msg.count,
        "point_coords": points_array_msg,
        "result_image": result_image,
        "line_families": line_families[:2],
        "single_frame_elapsed_ms": elapsed_ms,
    }


def run_manual_workspace_s2(self):
    return self.run_manual_workspace_s2_pipeline(publish=True)


def try_scan_only_manual_workspace_s2(self):
    if self.load_manual_workspace_quad() is None:
        return None

    s2_result = self.run_manual_workspace_s2_pipeline(publish=True)
    if not s2_result.get("success", False):
        return {
            "success": False,
            "message": (
                "扫描模式检测到已保存工作区四边形，但手动工作区S2失败："
                f"{s2_result.get('message', 'unknown error')}"
            ),
            "point_coords": None,
            "out_of_height_count": 0,
            "out_of_height_point_indices": [],
            "out_of_height_z_values": [],
        }

    point_coords = s2_result.get("point_coords")
    return {
        "success": True,
        "message": (
            f"扫描模式已直接输出手动工作区S2相机原始坐标点，共{getattr(point_coords, 'count', 0)}个"
        ),
        "point_coords": point_coords,
        "out_of_height_count": 0,
        "out_of_height_point_indices": [],
        "out_of_height_z_values": [],
    }


def manual_workspace_s2_callback(self, msg):
    if not bool(getattr(msg, "data", False)):
        return

    result = self.run_manual_workspace_s2()
    if not result.get("success", False):
        rospy.logwarn("pointAI manual workspace S2 failed: %s", result.get("message", "unknown error"))


def handle_lashing_recognize_once(self, _req):
    result = self.run_manual_workspace_s2()
    success = bool(result.get("success", False))
    message = str(result.get("message", "视觉识别完成" if success else "视觉识别失败"))
    return TriggerResponse(success=success, message=message)


def smooth_workspace_s2_profile(profile):
    return workspace_s2_smooth_profile(profile)


def estimate_workspace_s2_period_and_phase(profile, min_period=10, max_period=30):
    return workspace_s2_estimate_period_and_phase(
        profile,
        min_period=min_period,
        max_period=max_period,
    )


def build_workspace_s2_line_positions(start_pixel, end_pixel, period_px, phase_px):
    return workspace_s2_build_line_positions(
        start_pixel,
        end_pixel,
        period_px,
        phase_px,
    )


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
    return workspace_s2_refine_line_positions_to_local_peaks(
        profile,
        line_positions,
        search_radius_px=search_radius_px,
        min_spacing_px=min_spacing_px,
        target_edge_margin_ratio=target_edge_margin_ratio,
        edge_anchor_weight=edge_anchor_weight,
        auxiliary_profile=auxiliary_profile,
        auxiliary_weight=auxiliary_weight,
    )


def select_workspace_s2_peak_supported_line_positions(
    profile,
    line_positions,
    search_radius_px=4,
    min_spacing_px=8,
    min_peak_ratio=0.45,
    auxiliary_profile=None,
    auxiliary_weight=0.0,
):
    return workspace_s2_select_peak_supported_line_positions(
        profile,
        line_positions,
        search_radius_px=search_radius_px,
        min_spacing_px=min_spacing_px,
        min_peak_ratio=min_peak_ratio,
        auxiliary_profile=auxiliary_profile,
        auxiliary_weight=auxiliary_weight,
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
):
    return workspace_s2_select_continuous_line_positions(
        response_map,
        workspace_mask,
        line_positions,
        orientation=orientation,
        search_radius_px=search_radius_px,
        min_spacing_px=min_spacing_px,
        min_response_ratio=min_response_ratio,
        segment_count=segment_count,
        min_segment_coverage=min_segment_coverage,
    )


def prune_workspace_s2_line_positions_by_spacing(line_positions, min_spacing_ratio=0.65):
    return workspace_s2_prune_line_positions_by_spacing(
        line_positions,
        min_spacing_ratio=min_spacing_ratio,
    )


def build_workspace_s2_bbox(workspace_mask):
    return workspace_s2_build_bbox(workspace_mask)


def build_workspace_s2_axis_profile(response_map, workspace_mask, axis):
    return workspace_s2_build_axis_profile(response_map, workspace_mask, axis)


def normalize_workspace_s2_response(response_map, valid_mask, lower_percentile=5.0, upper_percentile=95.0):
    return workspace_s2_normalize_response(
        response_map,
        valid_mask,
        lower_percentile=lower_percentile,
        upper_percentile=upper_percentile,
    )


def build_workspace_s2_rectified_geometry(
    corner_pixels,
    corner_world_camera_frame=None,
    resolution_mm_per_px=5.0,
):
    return workspace_s2_build_rectified_geometry(
        corner_pixels,
        corner_world_camera_frame=corner_world_camera_frame,
        resolution_mm_per_px=resolution_mm_per_px,
    )


def map_workspace_s2_rectified_points_to_image(rectified_points, inverse_h):
    return workspace_s2_map_rectified_points_to_image(rectified_points, inverse_h)


def build_workspace_s2_structural_edge_suppression_mask(response_map, workspace_mask):
    return workspace_s2_build_structural_edge_suppression_mask(response_map, workspace_mask)


def filter_workspace_s2_rectified_points_outside_mask(rectified_points, exclusion_mask, margin_px=0):
    return workspace_s2_filter_rectified_points_outside_mask(
        rectified_points,
        exclusion_mask,
        margin_px=margin_px,
    )


def build_workspace_s2_projective_line_segments(
    corner_pixels,
    rectified_width,
    rectified_height,
    vertical_lines,
    horizontal_lines,
):
    return workspace_s2_build_projective_line_segments(
        corner_pixels,
        rectified_width,
        rectified_height,
        vertical_lines,
        horizontal_lines,
    )


def build_workspace_s2_oriented_line_families(
    response_map,
    workspace_mask,
    min_period=10,
    max_period=30,
    angle_step_deg=2.0,
):
    return workspace_s2_build_oriented_line_families(
        response_map,
        workspace_mask,
        min_period=min_period,
        max_period=max_period,
        angle_step_deg=angle_step_deg,
        use_orientation_prior_angle_pool=True,
        enable_local_peak_refine=True,
        enable_continuous_validation=True,
        enable_spacing_prune=True,
    )


def score_workspace_s2_oriented_line_family_result(line_families):
    return workspace_s2_score_oriented_line_family_result(line_families)


def intersect_workspace_s2_oriented_line_families(
    first_family,
    second_family,
    rectified_width,
    rectified_height,
    margin_px=0.0,
):
    return workspace_s2_intersect_oriented_line_families(
        first_family,
        second_family,
        rectified_width,
        rectified_height,
        margin_px=margin_px,
    )


def build_workspace_s2_oriented_projective_line_segments(
    corner_pixels,
    rectified_width,
    rectified_height,
    line_families,
):
    return workspace_s2_build_oriented_projective_line_segments(
        corner_pixels,
        rectified_width,
        rectified_height,
        line_families,
    )


def sort_polygon_points_clockwise(points):
    return workspace_s2_sort_polygon_points_clockwise(points)


def sort_polygon_indices_clockwise(points):
    return workspace_s2_sort_polygon_indices_clockwise(points)
