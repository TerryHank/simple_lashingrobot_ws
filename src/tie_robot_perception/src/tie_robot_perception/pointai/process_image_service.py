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
    build_workspace_s2_axis_profile,
    build_workspace_s2_bbox,
    build_workspace_s2_line_positions,
    build_workspace_s2_projective_line_segments,
    build_workspace_s2_rectified_geometry,
    estimate_workspace_s2_period_and_phase,
    map_workspace_s2_rectified_points_to_image,
    normalize_workspace_s2_response,
    smooth_workspace_s2_profile,
    sort_polygon_indices_clockwise,
    sort_polygon_points_clockwise,
)
from .constants import *

def has_detected_points(self, point_coords):
    return (
        point_coords is not None
        and getattr(point_coords, "count", 0) > 0
        and len(getattr(point_coords, "PointCoordinatesArray", [])) > 0
    )


def build_z_snapshot(self, point_coords):
    return tuple(
        (int(point.idx), float(point.World_coord[2]))
        for point in point_coords.PointCoordinatesArray
    )


def build_coordinate_snapshot(self, point_coords):
    return tuple(
        (
            int(point.idx),
            float(point.World_coord[0]),
            float(point.World_coord[1]),
            float(point.World_coord[2]),
        )
        for point in point_coords.PointCoordinatesArray
    )


def is_stable_z_window(self, z_snapshots, frame_count=None, tolerance_mm=None):
    frame_count = getattr(self, "stable_frame_count", 3) if frame_count is None else int(frame_count)
    tolerance_mm = getattr(self, "stable_z_tolerance_mm", 5.0) if tolerance_mm is None else float(tolerance_mm)
    if len(z_snapshots) < frame_count:
        return False
    if not z_snapshots[-frame_count:]:
        return False

    recent_snapshots = z_snapshots[-frame_count:]
    expected_indices = tuple(idx for idx, _ in recent_snapshots[0])
    if not expected_indices:
        return False

    for snapshot in recent_snapshots:
        if tuple(idx for idx, _ in snapshot) != expected_indices:
            return False

    max_allowed_range = tolerance_mm * 2.0
    for point_index in range(len(expected_indices)):
        z_values = [snapshot[point_index][1] for snapshot in recent_snapshots]
        if max(z_values) - min(z_values) > max_allowed_range:
            return False
    return True


def is_stable_coordinate_window(self, coordinate_snapshots, frame_count=None, tolerance_mm=None):
    frame_count = getattr(self, "stable_frame_count", 3) if frame_count is None else int(frame_count)
    tolerance_mm = getattr(self, "stable_z_tolerance_mm", 5.0) if tolerance_mm is None else float(tolerance_mm)
    if len(coordinate_snapshots) < frame_count:
        return False
    if not coordinate_snapshots[-frame_count:]:
        return False

    recent_snapshots = coordinate_snapshots[-frame_count:]
    expected_indices = tuple(item[0] for item in recent_snapshots[0])
    if not expected_indices:
        return False

    for snapshot in recent_snapshots:
        if tuple(item[0] for item in snapshot) != expected_indices:
            return False

    max_allowed_range = tolerance_mm * 2.0
    for point_index in range(len(expected_indices)):
        x_values = [snapshot[point_index][1] for snapshot in recent_snapshots]
        y_values = [snapshot[point_index][2] for snapshot in recent_snapshots]
        z_values = [snapshot[point_index][3] for snapshot in recent_snapshots]
        if max(x_values) - min(x_values) > max_allowed_range:
            return False
        if max(y_values) - min(y_values) > max_allowed_range:
            return False
        if max(z_values) - min(z_values) > max_allowed_range:
            return False
    return True


def get_request_mode(self, req):
    request_mode = getattr(req, "request_mode", PROCESS_IMAGE_MODE_DEFAULT)
    if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
        return PROCESS_IMAGE_MODE_BIND_CHECK
    if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
        return PROCESS_IMAGE_MODE_SCAN_ONLY
    if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
        return PROCESS_IMAGE_MODE_EXECUTION_REFINE
    return PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT


def get_request_mode_name(self, request_mode):
    if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
        return "bind_check"
    if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
        return "scan_only"
    if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
        return "execution_refine"
    if request_mode == PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT:
        return "adaptive_height"
    return "default"


def build_process_image_timing_message(self, request_mode, response, elapsed_sec, single_frame_elapsed_ms=None):
    single_frame_part = (
        f"single_frame_elapsed_ms={float(single_frame_elapsed_ms):.1f}, "
        if single_frame_elapsed_ms is not None
        else ""
    )
    return (
        "pointAI process_image response: "
        f"mode={self.get_request_mode_name(request_mode)}, "
        f"success={getattr(response, 'success', False)}, "
        f"count={getattr(response, 'count', 0)}, "
        f"{single_frame_part}"
        f"elapsed_ms={elapsed_sec * 1000.0:.1f}, "
        f"message={getattr(response, 'message', '')}"
    )


def log_process_image_timing(self, request_mode, response, elapsed_sec, single_frame_elapsed_ms=None):
    log_message = self.build_process_image_timing_message(
        request_mode,
        response,
        elapsed_sec,
        single_frame_elapsed_ms=single_frame_elapsed_ms,
    )
    if getattr(response, "success", False):
        rospy.loginfo(log_message)
    else:
        rospy.logwarn(log_message)


def build_detection_summary_log(
    self,
    request_mode,
    raw_candidate_count,
    duplicate_removed_count,
    in_range_candidate_count,
    out_of_range_point_count,
    selected_count,
    output_count,
    out_of_range_reason_counts,
    out_of_range_samples,
):
    lines = [
        "pointAI调试:",
        f"  模式: {self.get_request_mode_name(request_mode)}",
        (
            ("  规划工作区过滤: " if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY else "  可执行范围过滤: ")
            +
            f"原始候选={raw_candidate_count}, "
            f"去重移除={duplicate_removed_count}, "
            f"范围内={in_range_candidate_count}, "
            f"范围外={out_of_range_point_count}, "
            f"2x2选中={selected_count}, "
            f"本次输出={output_count}"
        ),
        (
            "  范围限制: "
            + (
                "按path_points.json规划工作区边界"
                if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY
                else "白框ROI，并在边缘按全局工作区自适应裁剪"
            )
        ),
    ]

    if out_of_range_point_count > 0:
        reason_summary = ", ".join(
            f"{reason}={count}" for reason, count in sorted(out_of_range_reason_counts.items())
        ) if out_of_range_reason_counts else "未知"
        lines.append(f"  范围外原因统计: {reason_summary}")
        if out_of_range_samples:
            lines.append("  样例:")
            lines.extend(f"    - {sample}" for sample in out_of_range_samples)

    if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
        if output_count > 0:
            conclusion = f"扫描模式输出{output_count}个相机原始坐标点，不做2x2限制"
        else:
            conclusion = "扫描模式当前没有规划工作区内可用点"
    elif request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
        if output_count > 0:
            conclusion = f"执行微调模式输出{output_count}个相机原始坐标点，不做2x2限制"
        else:
            conclusion = "执行微调模式当前没有可用于局部视觉微调的范围内点"
    elif request_mode == PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT:
        if output_count > 0:
            conclusion = (
                f"自适应高度模式输出{output_count}个范围内点用于高度平均，"
                "不做2x2数量限制，不检查绑扎高度"
            )
        else:
            conclusion = "自适应高度模式当前没有可用于高度平均的范围内点"
    elif in_range_candidate_count < 4:
        conclusion = f"可执行范围内点数不足4个，当前只有{in_range_candidate_count}个，无法放给下游"
    elif selected_count == 0:
        conclusion = f"可执行范围内有{in_range_candidate_count}个点，但暂时无法组成2x2矩阵"
    else:
        conclusion = f"已选出{selected_count}个2x2矩阵点，等待下游处理"

    lines.append(f"  结论: {conclusion}")
    return "\n".join(lines) + "\n"


def find_out_of_height_points(self, point_coords, max_height_mm=None):
    max_height_mm = getattr(self, "bind_check_max_height_mm", 95.0) if max_height_mm is None else max_height_mm
    out_of_height_points = []
    if point_coords is None:
        return out_of_height_points

    for point in getattr(point_coords, "PointCoordinatesArray", []):
        point_idx = int(point.idx)
        world_z = float(point.World_coord[2])
        if world_z > max_height_mm:
            out_of_height_points.append((point_idx, world_z))
    return out_of_height_points


def format_out_of_height_message(self, out_of_height_points, max_height_mm=None):
    max_height_mm = getattr(self, "bind_check_max_height_mm", 95.0) if max_height_mm is None else max_height_mm
    if not out_of_height_points:
        return ""

    point_details = ", ".join(
        f"点{point_idx}: z={world_z:.1f}mm"
        for point_idx, world_z in out_of_height_points
    )
    return (
        f"不在{max_height_mm:.0f}mm之内，超过的点有{len(out_of_height_points)}个，"
        f"实际高度为[{point_details}]"
    )


def evaluate_point_coords_for_mode(self, point_coords, request_mode):
    result = {
        "success": False,
        "message": "",
        "point_coords": point_coords,
        "out_of_height_count": 0,
        "out_of_height_point_indices": [],
        "out_of_height_z_values": [],
    }

    if not self.has_detected_points(point_coords):
        result["message"] = "未检测到有效点"
        return result

    if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
        result["success"] = True
        result["message"] = (
            "扫描模式已直接输出整个矩形画幅内、规划工作区内的相机原始坐标点"
        )
        return result

    if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
        result["success"] = True
        result["message"] = (
            "执行微调模式已直接输出当前局部可执行范围内的相机原始坐标点"
        )
        return result

    if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
        out_of_height_points = self.find_out_of_height_points(point_coords)
        result["out_of_height_count"] = len(out_of_height_points)
        result["out_of_height_point_indices"] = [point_idx for point_idx, _ in out_of_height_points]
        result["out_of_height_z_values"] = [world_z for _, world_z in out_of_height_points]
        result["success"] = True
        if out_of_height_points:
            result["message"] = (
                f"绑扎点已满足{getattr(self, 'stable_frame_count', 3)}帧坐标稳定并完成2x2排序；"
                f"{self.format_out_of_height_message(out_of_height_points)}，仅作日志，不拦截下游"
            )
            return result

        result["message"] = (
            f"绑扎点已满足{getattr(self, 'stable_frame_count', 3)}帧坐标稳定，"
            "已选出2x2矩阵并完成排序"
        )
        return result

    result["success"] = True
    result["message"] = (
        f"点位已满足{getattr(self, 'stable_frame_count', 3)}帧稳定，"
        f"z轴精度在+-{getattr(self, 'stable_z_tolerance_mm', 5.0):.1f}mm内，"
        "自适应高度按范围内点放行，不检查绑扎高度和点数量"
    )
    return result


def build_process_image_response(self, success, point_coords=None, message="", out_of_height_points=None):
    out_of_height_points = out_of_height_points or []
    point_array = []
    point_count = 0
    if success and self.has_detected_points(point_coords):
        point_array = point_coords.PointCoordinatesArray
        point_count = point_coords.count

    return ProcessImageResponse(
        success=success,
        message=message,
        out_of_height_count=len(out_of_height_points),
        out_of_height_point_indices=[point_idx for point_idx, _ in out_of_height_points],
        out_of_height_z_values=[world_z for _, world_z in out_of_height_points],
        count=point_count,
        PointCoordinatesArray=point_array,
    )


def wait_for_stable_point_coords(self, request_mode):
    stable_snapshots = []
    latest_point_coords = None
    last_processed_frame_seq = -1
    start_time = time.time()
    rate = rospy.Rate(self.process_request_rate_hz)
    mode_frame_count = getattr(self, "stable_frame_count", 3)
    mode_tolerance_mm = getattr(self, "stable_z_tolerance_mm", 5.0)

    while not rospy.is_shutdown():
        if self.process_wait_timeout_sec > 0 and time.time() - start_time > self.process_wait_timeout_sec:
            if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
                message = "pointAI process_image timed out while waiting for execution refine plane-segmentation + Hough vision"
            elif request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
                message = "pointAI process_image timed out while waiting for Surface-DP physical-prior scan vision"
            else:
                message = "pointAI process_image timed out while waiting for Surface-DP main vision"
            rospy.logwarn(message)
            return {
                "success": False,
                "message": message,
                "point_coords": None,
                "out_of_height_count": 0,
                "out_of_height_point_indices": [],
                "out_of_height_z_values": [],
            }

        if self.image is None or not hasattr(self, "image_raw_world") or self.image_raw_world is None:
            rospy.logwarn_throttle(2.0, "pointAI waiting for image and raw world coordinate frames")
            rate.sleep()
            continue

        current_frame_seq = getattr(self, "world_image_seq", 0)
        if current_frame_seq == last_processed_frame_seq:
            rate.sleep()
            continue
        last_processed_frame_seq = current_frame_seq

        if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
            execution_refine_result = self.run_execution_refine_hough_pipeline(publish=True)
            single_frame_elapsed_ms = execution_refine_result.get("single_frame_elapsed_ms")
            point_coords = execution_refine_result.get("point_coords")
            if not self.has_detected_points(point_coords):
                stable_snapshots = []
                latest_point_coords = None
                rospy.logwarn_throttle(
                    2.0,
                    "pointAI等待执行微调平面分割+Hough有效点: %s",
                    execution_refine_result.get("message", "unknown error"),
                )
                rate.sleep()
                continue

            latest_point_coords = point_coords
            result = self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)
            result["single_frame_elapsed_ms"] = single_frame_elapsed_ms
            return result

        if self.load_manual_workspace_quad() is None:
            missing_workspace_message = (
                "当前扫描触发方案为Surface-DP物理先验扫描，请先提交并保存工作区四边形。"
                if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY
                else "当前主视觉方案为Surface-DP，请先提交并保存工作区四边形。"
            )
            return {
                "success": False,
                "message": missing_workspace_message,
                "point_coords": None,
                "out_of_height_count": 0,
                "out_of_height_point_indices": [],
                "out_of_height_z_values": [],
            }

        main_visual_result = self.run_manual_workspace_s2_pipeline(publish=True)
        single_frame_elapsed_ms = main_visual_result.get("single_frame_elapsed_ms")
        point_coords = main_visual_result.get("point_coords")
        if not self.has_detected_points(point_coords):
            stable_snapshots = []
            latest_point_coords = None
            rospy.logwarn_throttle(
                2.0,
                (
                    "pointAI等待Surface-DP物理先验扫描有效点: %s"
                    if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY
                    else "pointAI等待Surface-DP主视觉有效点: %s"
                ),
                main_visual_result.get("message", "unknown error"),
            )
            rate.sleep()
            continue

        latest_point_coords = point_coords
        if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
            result = self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)
            result["single_frame_elapsed_ms"] = single_frame_elapsed_ms
            return result

        if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
            snapshot = self.build_coordinate_snapshot(point_coords)
        else:
            snapshot = self.build_z_snapshot(point_coords)

        if stable_snapshots and tuple(item[0] for item in snapshot) != tuple(item[0] for item in stable_snapshots[-1]):
            stable_snapshots = []
        stable_snapshots.append(snapshot)
        stable_snapshots = stable_snapshots[-mode_frame_count:]

        if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
            is_stable = self.is_stable_coordinate_window(
                stable_snapshots,
                frame_count=mode_frame_count,
                tolerance_mm=mode_tolerance_mm,
            )
        else:
            is_stable = self.is_stable_z_window(
                stable_snapshots,
                frame_count=mode_frame_count,
                tolerance_mm=mode_tolerance_mm,
            )

        if is_stable:
            result = self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)
            if result["success"]:
                result["single_frame_elapsed_ms"] = single_frame_elapsed_ms
                return result

        if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
            rospy.loginfo_throttle(
                2.0,
                "pointAI等待绑扎点坐标稳定: %d/%d帧，坐标容差在+-%.1fmm内",
                len(stable_snapshots),
                mode_frame_count,
                mode_tolerance_mm
            )
        else:
            rospy.loginfo_throttle(
                2.0,
                "pointAI waiting for stable Z: %d/%d frames within +/-%.1f mm",
                len(stable_snapshots),
                mode_frame_count,
                mode_tolerance_mm
            )
        rate.sleep()

    return {
        "success": False,
        "message": "pointAI process_image interrupted before stable result was available",
        "point_coords": latest_point_coords if self.has_detected_points(latest_point_coords) else None,
        "out_of_height_count": 0,
        "out_of_height_point_indices": [],
        "out_of_height_z_values": [],
    }


def handle_process_image(self, req):
    request_mode = self.get_request_mode(req)
    self.current_result_request_mode = request_mode
    start_time = time.perf_counter()
    self.mark_visual_process_request()
    try:
        result = self.wait_for_stable_point_coords(request_mode)
        out_of_height_points = list(
            zip(
                result.get("out_of_height_point_indices", []),
                result.get("out_of_height_z_values", []),
            )
        )
        response = self.build_process_image_response(
            success=result.get("success", False),
            point_coords=result.get("point_coords"),
            message=result.get("message", ""),
            out_of_height_points=out_of_height_points,
        )
        elapsed_sec = time.perf_counter() - start_time
        single_frame_elapsed_ms = result.get("single_frame_elapsed_ms")
        timing_parts = []
        if single_frame_elapsed_ms is not None:
            timing_parts.append(f"单帧视觉耗时={float(single_frame_elapsed_ms):.1f}ms")
        timing_parts.append(f"视觉服务请求耗时={elapsed_sec * 1000.0:.1f}ms")
        timing_suffix = "；" + "；".join(timing_parts)
        response.message = f"{response.message}{timing_suffix}" if response.message else timing_suffix.lstrip("；")
        self.log_process_image_timing(
            request_mode,
            response,
            elapsed_sec,
            single_frame_elapsed_ms=single_frame_elapsed_ms,
        )
        self.mark_visual_process_result(response.success, response.message)
        return response

    except Exception as e:
        rospy.logerr(f"Error in handle_process_image: {str(e)}")
        self.mark_visual_error(f"Error in handle_process_image: {str(e)}")
        response = self.build_process_image_response(
            success=False,
            message=f"Error in handle_process_image: {str(e)}",
        )
        self.log_process_image_timing(request_mode, response, time.perf_counter() - start_time)
        return response
