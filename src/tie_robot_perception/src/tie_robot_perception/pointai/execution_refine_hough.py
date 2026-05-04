"""Execution-layer plane-segmentation + Hough visual refinement."""
import time

import cv2
import numpy as np
import rospy
from cv2 import ximgproc
from sklearn.cluster import DBSCAN

from tie_robot_msgs.msg import PointCoords, PointsArray

from .constants import PROCESS_IMAGE_MODE_EXECUTION_REFINE


def _publish_mono_image(self, publisher_name, image, frame_id):
    publisher = getattr(self, publisher_name, None)
    if publisher is None:
        return

    image_msg = self.bridge.cv2_to_imgmsg(image, encoding="mono8")
    image_msg.header.stamp = rospy.Time.now()
    image_msg.header.frame_id = frame_id
    publisher.publish(image_msg)


def _build_execution_refine_binary(self):
    channels = self.cv2.split(self.image)
    if len(channels) < 3:
        return None, "平面分割世界坐标图缺少 Z 通道"

    self.Depth_image_Raw = np.array(channels[2], dtype=np.float32, copy=True)
    self.Depth_image_Raw[~np.isfinite(self.Depth_image_Raw)] = 0.0
    self.detection_occlusion_mask = self.apply_detection_occlusions(PROCESS_IMAGE_MODE_EXECUTION_REFINE)

    positive_depth = self.Depth_image_Raw[self.Depth_image_Raw > 0.0]
    if positive_depth.size == 0:
        return None, "平面分割后没有可用于 Hough 的非平面深度像素"

    max_depth = float(np.max(positive_depth) - 5.0)
    if max_depth <= 11.0:
        return None, "平面分割后非平面深度范围过窄，无法生成 Hough 二值图"

    binary = np.zeros(self.Depth_image_Raw.shape[:2], dtype=np.uint8)
    binary[(self.Depth_image_Raw >= 11.0) & (self.Depth_image_Raw <= max_depth)] = 255
    binary = cv2.medianBlur(binary, 3)
    binary = self.remove_small_foreground_components(binary)
    return binary, ""


def _build_execution_refine_line_image(self, shape):
    if getattr(self, "image_infrared", None) is not None:
        return np.zeros_like(self.image_infrared)
    return np.zeros(shape, dtype=np.uint8)


def _filter_axis_aligned_lines(self, lines):
    if lines is None:
        return None

    filtered_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if self.is_near_axis_aligned_line(x1, y1, x2, y2):
            filtered_lines.append(line)

    if not filtered_lines:
        return None
    return np.array(filtered_lines, dtype=np.int32)


def _build_execution_refine_point_array(self, output_centers):
    point_array_msg = PointsArray()
    point_array_msg.PointCoordinatesArray = []

    for idx, (_, center, calibrated_world_coord) in enumerate(output_centers, start=1):
        x, y = int(center[0]), int(center[1])
        x_value, y_value, z_value = calibrated_world_coord
        if x_value == 0 or y_value == 0 or z_value == 0:
            rospy.logwarn_throttle(
                2.0,
                "execution_refine_hough invalid raw camera coord: x=%s, y=%s, z=%s",
                x_value,
                y_value,
                z_value,
            )
            continue

        point = PointCoords()
        point.is_shuiguan = False
        point.Angle = -45
        point.idx = idx
        point.Pix_coord = [x, y]
        point.World_coord = [
            float(x_value),
            float(y_value),
            float(z_value),
        ]
        point_array_msg.PointCoordinatesArray.append(point)

    point_array_msg.count = len(point_array_msg.PointCoordinatesArray)
    return point_array_msg


def run_execution_refine_hough_pipeline(self, publish=True):
    start_time = time.perf_counter()
    if self.image is None:
        return {
            "success": False,
            "message": "执行微调平面分割+Hough等待 world_coord 图像",
            "point_coords": None,
            "single_frame_elapsed_ms": 0.0,
        }
    if not hasattr(self, "image_raw_world") or self.image_raw_world is None:
        return {
            "success": False,
            "message": "执行微调平面分割+Hough等待 raw_world_coord 图像",
            "point_coords": None,
            "single_frame_elapsed_ms": 0.0,
        }

    self.current_result_request_mode = PROCESS_IMAGE_MODE_EXECUTION_REFINE
    self.result_display_points = []
    self.last_detection_debug = {}
    self.ensure_raw_world_channels()

    binary_image, binary_error = _build_execution_refine_binary(self)
    if binary_image is None:
        return {
            "success": False,
            "message": binary_error,
            "point_coords": None,
            "single_frame_elapsed_ms": (time.perf_counter() - start_time) * 1000.0,
        }

    self.Depth_image_Raw_binary = binary_image
    if publish:
        _publish_mono_image(self, "depth_binary_image_pub", self.Depth_image_Raw_binary, "Scepter_depth_frame")

    if hasattr(ximgproc, "thinning"):
        self.skeleton = ximgproc.thinning(
            self.Depth_image_Raw_binary,
            thinningType=ximgproc.THINNING_ZHANGSUEN,
        )
    else:
        self.skeleton = self.Depth_image_Raw_binary

    self.lines = self.cv2.HoughLinesP(
        self.skeleton,
        rho=1,
        theta=np.pi / 180,
        threshold=self.threshold,
        minLineLength=self.minLineLength,
        maxLineGap=self.maxLineGap,
    )
    self.lines = _filter_axis_aligned_lines(self, self.lines)

    self.line_image = _build_execution_refine_line_image(self, self.Depth_image_Raw_binary.shape)
    if self.lines is not None:
        for line in self.lines:
            x1, y1, x2, y2 = line[0]
            self.cv2.line(self.line_image, (x1, y1), (x2, y2), 255, 2)
    if publish:
        _publish_mono_image(self, "line_image_pub", self.line_image, "Scepter_ir_frame")

    if self.lines is not None:
        self.intersections, self.angles = self.calculate_intersections(self.lines)
    else:
        self.intersections, self.angles = [], []

    lines_count = 0 if self.lines is None else len(self.lines)
    intersections_count = 0 if self.intersections is None else len(self.intersections)
    if self.intersections is None or len(self.intersections) == 0:
        self.last_detection_debug = {
            "lines": lines_count,
            "intersections": intersections_count,
            "centers": 0,
            "world_fallback": 0,
            "roi_reject": 0,
            "zero_world": 0,
            "candidate_points": 0,
            "in_range_candidates": 0,
            "selected_points": 0,
            "out_of_range_points": 0,
            "duplicate_removed_points": 0,
            "output_points": 0,
        }
        return {
            "success": False,
            "message": f"执行微调平面分割+Hough未形成交点：lines={lines_count}, intersections=0",
            "point_coords": None,
            "single_frame_elapsed_ms": (time.perf_counter() - start_time) * 1000.0,
        }

    clustering = DBSCAN(eps=15, min_samples=1).fit(self.intersections)
    clusters = {}
    for index, label in enumerate(clustering.labels_):
        clusters.setdefault(label, []).append(self.intersections[index])

    centers = []
    world_fallback_count = 0
    for points in clusters.values():
        center_x = int(sum(point[0] for point in points) / len(points)) - self.offset_x
        center_y = int(sum(point[1] for point in points) / len(points)) - self.offset_y
        world_coord, _, used_fallback = self.get_valid_world_coord_near_pixel(center_x, center_y)
        if used_fallback:
            world_fallback_count += 1
        centers.append([center_x, center_y, world_coord])

    candidate_centers = []
    roi_reject_count = 0
    zero_world_count = 0
    for source_idx, center in enumerate(centers):
        if not self.is_point_in_roi(center[0], center[1]):
            roi_reject_count += 1
            continue

        x_value, y_value, z_value = center[2]
        if x_value == 0 or y_value == 0 or z_value == 0:
            zero_world_count += 1
            continue

        candidate_centers.append((
            source_idx,
            center,
            [float(x_value), float(y_value), float(z_value)],
        ))

    raw_candidate_count = len(candidate_centers)
    candidate_centers, duplicate_removed_count = self.filter_candidate_centers_for_request_mode(
        candidate_centers,
        PROCESS_IMAGE_MODE_EXECUTION_REFINE,
    )
    matrix_selection_pixel_mask = self.get_travel_range_pixel_mask()
    in_range_centers = []
    out_of_range_count = 0
    out_of_range_reason_counts = {}
    out_of_range_samples = []
    for center_record in candidate_centers:
        if self.is_point_in_matrix_selection_pixel_mask(
            center_record[1][0],
            center_record[1][1],
            matrix_selection_pixel_mask,
        ):
            in_range_centers.append(center_record)
            continue

        out_of_range_count += 1
        out_of_range_reason_counts["超出执行微调框"] = out_of_range_reason_counts.get("超出执行微调框", 0) + 1
        if len(out_of_range_samples) < 5:
            source_idx, center, calibrated_world_coord = center_record
            out_of_range_samples.append(
                "idx={},pix=({},{})"
                ",coord=({:.1f},{:.1f},{:.1f}),原因=超出执行微调框".format(
                    source_idx,
                    int(center[0]),
                    int(center[1]),
                    calibrated_world_coord[0],
                    calibrated_world_coord[1],
                    calibrated_world_coord[2],
                )
            )

    self.sorted_centers = []
    output_centers = self.select_output_centers_for_mode(
        PROCESS_IMAGE_MODE_EXECUTION_REFINE,
        in_range_centers,
        self.sorted_centers,
    )
    self.result_display_points = self.build_matrix_display_points(output_centers)
    point_array_msg = _build_execution_refine_point_array(self, output_centers)

    self.last_detection_debug = {
        "lines": lines_count,
        "intersections": intersections_count,
        "centers": len(centers),
        "world_fallback": world_fallback_count,
        "roi_reject": roi_reject_count,
        "zero_world": zero_world_count,
        "candidate_points": len(candidate_centers),
        "in_range_candidates": len(in_range_centers),
        "selected_points": 0,
        "out_of_range_points": out_of_range_count,
        "duplicate_removed_points": duplicate_removed_count,
        "output_points": point_array_msg.count,
    }
    rospy.loginfo(
        "execution_refine_hough: lines=%d intersections=%d centers=%d "
        "world_fallback=%d roi_reject=%d zero_world=%d candidate_points=%d "
        "in_range_candidates=%d out_of_range_points=%d duplicate_removed_points=%d output_points=%d",
        self.last_detection_debug["lines"],
        self.last_detection_debug["intersections"],
        self.last_detection_debug["centers"],
        self.last_detection_debug["world_fallback"],
        self.last_detection_debug["roi_reject"],
        self.last_detection_debug["zero_world"],
        self.last_detection_debug["candidate_points"],
        self.last_detection_debug["in_range_candidates"],
        self.last_detection_debug["out_of_range_points"],
        self.last_detection_debug["duplicate_removed_points"],
        self.last_detection_debug["output_points"],
    )
    detection_summary_log = self.build_detection_summary_log(
        request_mode=PROCESS_IMAGE_MODE_EXECUTION_REFINE,
        raw_candidate_count=raw_candidate_count,
        duplicate_removed_count=duplicate_removed_count,
        in_range_candidate_count=len(in_range_centers),
        out_of_range_point_count=out_of_range_count,
        selected_count=0,
        output_count=point_array_msg.count,
        out_of_range_reason_counts=out_of_range_reason_counts,
        out_of_range_samples=out_of_range_samples,
    )
    rospy.logwarn_throttle(1.0, detection_summary_log)

    if publish:
        self.coordinate_publisher.publish(point_array_msg)
        if getattr(self, "lashing_points_camera_pub", None) is not None:
            self.lashing_points_camera_pub.publish(point_array_msg)
        previous_prefix = getattr(self, "raw_bind_point_tf_child_prefix", "surface_dp_bind_point")
        self.raw_bind_point_tf_child_prefix = "execution_hough_bind_point"
        self.publish_raw_camera_bind_point_transforms(point_array_msg)
        self.raw_bind_point_tf_child_prefix = previous_prefix

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    if point_array_msg.count <= 0:
        return {
            "success": False,
            "message": "执行微调平面分割+Hough没有输出执行范围内点",
            "point_coords": point_array_msg,
            "single_frame_elapsed_ms": elapsed_ms,
        }

    return {
        "success": True,
        "message": f"执行微调平面分割+Hough输出{point_array_msg.count}个相机原始坐标点",
        "point_coords": point_array_msg,
        "single_frame_elapsed_ms": elapsed_ms,
    }
