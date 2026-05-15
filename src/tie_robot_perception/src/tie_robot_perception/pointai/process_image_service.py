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
from .bind_point_classification import (
    append_classification_event,
    classify_pre_bind_point,
    decisions_to_summary,
    extract_evidence_bundle,
    filter_unbound_points_for_execution,
    iter_point_messages,
    should_mark_point_as_bound,
)

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


def classify_points_for_phase(self, point_coords, phase):
    config = getattr(self, "bind_classification_config", None)
    if config is None or getattr(config, "mode", "shadow") == "off":
        self.latest_bind_classification_decisions = []
        if phase == "execution_refine":
            self.execution_refine_classification_diagnostic_points = []
        return point_coords
    if not self.has_detected_points(point_coords):
        self.latest_bind_classification_decisions = []
        if phase == "execution_refine":
            self.execution_refine_classification_diagnostic_points = []
        return point_coords

    ir_image = getattr(self, "image_infrared", None)
    if ir_image is None:
        ir_image = getattr(self, "image_infrared_copy", None)
    depth_image = getattr(self, "Depth_image_Raw", None)
    raw_world_image = getattr(self, "image_raw_world", None)
    if depth_image is None and raw_world_image is not None:
        raw_world_channels = self.cv2.split(raw_world_image)
        if len(raw_world_channels) >= 3:
            depth_image = raw_world_channels[2]

    decisions = []
    model_cache = getattr(self, "bind_classification_model_cache", None)
    for point in iter_point_messages(point_coords):
        bundle = extract_evidence_bundle(
            ir_image=ir_image,
            depth_image=depth_image,
            raw_world_image=raw_world_image,
            point_idx=int(point.idx),
            pix_coord=point.Pix_coord,
            world_coord=point.World_coord,
            phase=phase,
            config=config,
        )
        decision = classify_pre_bind_point(bundle, config, model_cache=model_cache)
        decisions.append(decision)
        point.is_shuiguan = bool(should_mark_point_as_bound(decision, config))
        append_classification_event(
            config.event_log_path,
            {
                "phase": phase,
                "point_idx": int(point.idx),
                "pix_coord": [int(point.Pix_coord[0]), int(point.Pix_coord[1])],
                "world_coord": [float(value) for value in point.World_coord[:3]],
                "mode": config.mode,
                "method": getattr(config, "method", "deep_learning"),
                "bind_state": decision.label,
                "bind_score": float(decision.score),
                "evidence_quality": float(decision.quality),
                "decision_reason": decision.reason,
                "metrics": decision.metrics,
                "wire_is_shuiguan": bool(point.is_shuiguan),
            },
        )

    self.latest_bind_classification_decisions = decisions
    summary = decisions_to_summary(decisions)
    rospy.loginfo(
        "pointAI绑扎点分类: mode=%s method=%s phase=%s bound=%d unbound=%d uncertain=%d",
        config.mode,
        getattr(config, "method", "deep_learning"),
        phase,
        int(summary.get("bound", 0)),
        int(summary.get("unbound", 0)),
        int(summary.get("uncertain", 0)),
    )
    return point_coords


def classify_bind_check_points(self, point_coords):
    return self.classify_points_for_phase(point_coords, "before")


def build_execution_refine_classification_diagnostic_points(self, point_coords):
    decisions = list(getattr(self, "latest_bind_classification_decisions", []) or [])
    points = list(getattr(point_coords, "PointCoordinatesArray", []) or [])
    diagnostic_points = []
    for point, decision in zip(points, decisions):
        label = str(getattr(decision, "label", "uncertain") or "uncertain").strip().lower()
        if label == "bound":
            status = "classification_bound"
            text = "BND"
        elif label == "unbound":
            status = "classification_unbound"
            text = "UNB"
        else:
            status = "classification_uncertain"
            text = "UNC"
        score = float(getattr(decision, "score", 0.0) or 0.0)
        diagnostic_points.append({
            "status": status,
            "pixel": [int(point.Pix_coord[0]), int(point.Pix_coord[1])],
            "label": f"{text}:{score:.2f}",
        })
    return diagnostic_points


def classify_execution_refine_points(self, point_coords):
    classified_points = self.classify_points_for_phase(point_coords, "execution_refine")
    self.execution_refine_classification_diagnostic_points = (
        self.build_execution_refine_classification_diagnostic_points(classified_points)
    )
    config = getattr(self, "bind_classification_config", None)
    if config is not None and getattr(config, "mode", "off") == "blocking" and self.has_detected_points(classified_points):
        kept_points = filter_unbound_points_for_execution(classified_points.PointCoordinatesArray, config)
        classified_points.PointCoordinatesArray = kept_points
        classified_points.count = len(kept_points)
    return classified_points


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


def get_execution_refine_algorithm(self):
    algorithm = str(getattr(self, "execution_refine_algorithm", "hough") or "hough").strip()
    if algorithm in {"hough", "surface_dp"}:
        return algorithm
    return "hough"


def get_execution_refine_algorithm_label(self):
    if self.get_execution_refine_algorithm() == "surface_dp":
        return "扫描同款Surface-DP"
    return "平面分割+Hough"


def build_process_image_timing_message(self, request_mode, response, elapsed_sec, single_frame_elapsed_ms=None):
    single_frame_part = (
        f"单帧耗时={float(single_frame_elapsed_ms):.1f}ms，"
        if single_frame_elapsed_ms is not None
        else ""
    )
    return (
        "pointAI视觉服务响应："
        f"模式={self.get_request_mode_name(request_mode)}，"
        f"成功={getattr(response, 'success', False)}，"
        f"点数={getattr(response, 'count', 0)}，"
        f"{single_frame_part}"
        f"总耗时={elapsed_sec * 1000.0:.1f}ms，"
        f"消息={getattr(response, 'message', '')}"
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
    in_range_candidate_count,
    out_of_range_point_count,
    selected_count,
    output_count,
    out_of_range_reason_counts,
    out_of_range_samples,
):
    if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
        range_label = "规划工作区过滤"
        selected_label = "输出候选"
        range_limit = "按path_points.json规划工作区边界"
    elif request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
        range_label = "TCP执行盒+全局工作区过滤"
        selected_label = "区域组点"
        range_limit = "按TCP执行盒和手动确认全局工作区边界"
    else:
        range_label = "可执行范围过滤"
        selected_label = "2x2选中"
        range_limit = "按手动工作区或path_points.json规划工作区边界"

    lines = [
        "pointAI调试:",
        f"  模式: {self.get_request_mode_name(request_mode)}",
        (
            f"  {range_label}: "
            f"原始候选={raw_candidate_count}, "
            f"范围内={in_range_candidate_count}, "
            f"范围外={out_of_range_point_count}, "
            f"{selected_label}={selected_count}, "
            f"本次输出={output_count}"
        ),
        f"  范围限制: {range_limit}",
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
            conclusion = f"执行微调模式输出{output_count}个全局工作区内相机原始坐标点，作为当前区域一组"
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
            f"扫描模式已满足{getattr(self, 'stable_frame_count', 3)}帧释放，"
            "输出整个矩形画幅内、规划工作区内的相机原始坐标点"
        )
        return result

    if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
        point_coords = self.classify_execution_refine_points(point_coords)
        result["point_coords"] = point_coords
        self.publish_execution_refine_classified_base_image(
            point_coords,
            getattr(self, "execution_refine_classification_diagnostic_points", []),
        )
        if not self.has_detected_points(point_coords):
            result["message"] = "执行微调分类后未发现未绑扎点"
            return result
        result["success"] = True
        result["message"] = (
            f"执行微调模式已满足{getattr(self, 'stable_frame_count', 3)}帧释放，"
            "输出当前局部可执行范围内的相机原始坐标点"
        )
        return result

    if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
        point_coords = self.classify_bind_check_points(point_coords)
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


def run_execution_refine_visual_pipeline(self, publish=True):
    if self.get_execution_refine_algorithm() == "surface_dp":
        return self.run_execution_refine_surface_dp_pipeline(publish=publish)
    return self.run_execution_refine_hough_pipeline(publish=publish)


def wait_for_stable_point_coords(self, request_mode):
    stable_snapshots = []
    latest_point_coords = None
    execution_refine_no_points_start_time = None
    last_processed_frame_seq = -1
    start_time = time.time()
    rate = rospy.Rate(self.process_request_rate_hz)
    mode_frame_count = getattr(self, "stable_frame_count", 3)
    mode_tolerance_mm = getattr(self, "stable_z_tolerance_mm", 5.0)
    release_frame_only_modes = {
        PROCESS_IMAGE_MODE_SCAN_ONLY,
        PROCESS_IMAGE_MODE_EXECUTION_REFINE,
    }

    while not rospy.is_shutdown():
        if self.process_wait_timeout_sec > 0 and time.time() - start_time > self.process_wait_timeout_sec:
            if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
                message = f"pointAI视觉服务等待执行微调{self.get_execution_refine_algorithm_label()}超时"
            elif request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
                message = "pointAI视觉服务等待Surface-DP物理先验扫描超时"
            else:
                message = "pointAI视觉服务等待Surface-DP主视觉超时"
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
            rospy.logwarn_throttle(2.0, "pointAI等待图像帧和原始世界坐标帧")
            rate.sleep()
            continue

        current_frame_seq = getattr(self, "world_image_seq", 0)
        if current_frame_seq == last_processed_frame_seq:
            rate.sleep()
            continue
        last_processed_frame_seq = current_frame_seq

        if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
            execution_refine_result = self.run_execution_refine_visual_pipeline(publish=True)
            single_frame_elapsed_ms = execution_refine_result.get("single_frame_elapsed_ms")
            point_coords = execution_refine_result.get("point_coords")
            if not self.has_detected_points(point_coords):
                stable_snapshots = []
                latest_point_coords = None
                now = time.time()
                if execution_refine_no_points_start_time is None:
                    execution_refine_no_points_start_time = now
                no_points_timeout_sec = float(
                    getattr(self, "execution_refine_no_points_timeout_sec", 3.0)
                )
                no_points_elapsed_sec = now - execution_refine_no_points_start_time
                if no_points_timeout_sec > 0 and no_points_elapsed_sec >= no_points_timeout_sec:
                    message = "EXECUTION_REFINE_NO_POINTS: 执行微调在当前区域未返回可执行点，跳过当前区域"
                    rospy.logwarn(
                        "%s；最近视觉消息：%s",
                        message,
                        execution_refine_result.get("message", "未知错误"),
                    )
                    return {
                        "success": False,
                        "message": message,
                        "point_coords": point_coords,
                        "out_of_height_count": 0,
                        "out_of_height_point_indices": [],
                        "out_of_height_z_values": [],
                        "single_frame_elapsed_ms": single_frame_elapsed_ms,
                    }
                rospy.logwarn_throttle(
                    2.0,
                    "pointAI等待执行微调%s有效点: %s（无点等待%.1fs/%.1fs）",
                    self.get_execution_refine_algorithm_label(),
                    execution_refine_result.get("message", "未知错误"),
                    no_points_elapsed_sec,
                    no_points_timeout_sec,
                )
                rate.sleep()
                continue

            latest_point_coords = point_coords
            execution_refine_no_points_start_time = None
            snapshot = self.build_coordinate_snapshot(point_coords)
            stable_snapshots.append(snapshot)
            stable_snapshots = stable_snapshots[-mode_frame_count:]
            if len(stable_snapshots) >= mode_frame_count:
                result = self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)
                result["single_frame_elapsed_ms"] = single_frame_elapsed_ms
                return result

            rospy.loginfo_throttle(
                2.0,
                "pointAI等待执行微调视觉释放帧: %d/%d帧",
                len(stable_snapshots),
                mode_frame_count,
            )
            rate.sleep()
            continue

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
            if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
                message = main_visual_result.get("message", "未知错误")
                rospy.logwarn("pointAI扫描当前帧未返回有效点: %s", message)
                return {
                    "success": False,
                    "message": message,
                    "point_coords": point_coords,
                    "out_of_height_count": 0,
                    "out_of_height_point_indices": [],
                    "out_of_height_z_values": [],
                    "single_frame_elapsed_ms": single_frame_elapsed_ms,
                }
            rospy.logwarn_throttle(
                2.0,
                (
                    "pointAI等待Surface-DP主视觉有效点: %s"
                ),
                main_visual_result.get("message", "未知错误"),
            )
            rate.sleep()
            continue

        latest_point_coords = point_coords
        if request_mode in release_frame_only_modes:
            snapshot = self.build_coordinate_snapshot(point_coords)
        elif request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
            snapshot = self.build_coordinate_snapshot(point_coords)
        else:
            snapshot = self.build_z_snapshot(point_coords)

        if (
            request_mode not in release_frame_only_modes
            and stable_snapshots
            and tuple(item[0] for item in snapshot) != tuple(item[0] for item in stable_snapshots[-1])
        ):
            stable_snapshots = []
        stable_snapshots.append(snapshot)
        stable_snapshots = stable_snapshots[-mode_frame_count:]

        if request_mode in release_frame_only_modes:
            is_stable = len(stable_snapshots) >= mode_frame_count
        elif request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
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

        if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
            rospy.loginfo_throttle(
                2.0,
                "pointAI等待扫描视觉释放帧: %d/%d帧",
                len(stable_snapshots),
                mode_frame_count,
            )
        elif request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
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
                "pointAI等待Z轴稳定: %d/%d帧，容差在+-%.1fmm内",
                len(stable_snapshots),
                mode_frame_count,
                mode_tolerance_mm
            )
        rate.sleep()

    return {
        "success": False,
        "message": "pointAI视觉服务在获得稳定结果前被中断",
        "point_coords": latest_point_coords if self.has_detected_points(latest_point_coords) else None,
        "out_of_height_count": 0,
        "out_of_height_point_indices": [],
        "out_of_height_z_values": [],
    }


def run_visual_detection_with_release_frames(self, request_mode):
    self.current_result_request_mode = request_mode
    self.mark_visual_process_request()
    result = self.wait_for_stable_point_coords(request_mode)
    self.mark_visual_process_result(
        bool(result.get("success", False)),
        str(result.get("message", "")),
    )
    return result


def handle_process_image(self, req):
    request_mode = self.get_request_mode(req)
    start_time = time.perf_counter()
    try:
        result = self.run_visual_detection_with_release_frames(request_mode)
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
        return response

    except Exception as e:
        rospy.logerr(f"处理pointAI视觉服务请求异常: {str(e)}")
        self.mark_visual_error(f"处理pointAI视觉服务请求异常: {str(e)}")
        response = self.build_process_image_response(
            success=False,
            message=f"处理pointAI视觉服务请求异常: {str(e)}",
        )
        self.log_process_image_timing(request_mode, response, time.perf_counter() - start_time)
        return response
