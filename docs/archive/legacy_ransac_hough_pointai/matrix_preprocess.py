"""pointAI 矩阵识别主流程。"""

import cv2
import numpy as np
import rospy
from cv2 import ximgproc
from sklearn.cluster import DBSCAN

from tie_robot_msgs.msg import PointCoords, PointsArray

from .constants import *
from .manual_workspace_s2 import log_manual_workspace_s2_camera_distance


def pre_img(self, request_mode=PROCESS_IMAGE_MODE_DEFAULT):
    if self.image is None:
        return None  # 返回空列表
    self.current_result_request_mode = request_mode
    self.result_display_points = []
    self.last_detection_debug = {}
    self.channels = self.cv2.split(self.image)
    # 分别获取X, Y, Z通道
    self.image_raw_world_channels = self.cv2.split(self.image_raw_world)
    # 分别获取X, Y, Z通道
    self.x_channel = (self.image_raw_world_channels[0]).astype(np.float32)
    self.y_channel = (self.image_raw_world_channels[1]).astype(np.float32)
    self.depth_v = (self.image_raw_world_channels[2]).astype(np.float32)
    self.Depth_image_Raw = np.copy((self.channels[2]).astype(np.int32))
    self.detection_occlusion_mask = self.apply_detection_occlusions(request_mode)
    # self.Depth_image_Raw[self.y3:self.y4, self.x3:self.x4] = 0
    self.Depth_image_Raw_raw = cv2.normalize(self.Depth_image_Raw, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    self.max_depth = int(np.max(self.Depth_image_Raw) - 5)
    self.Depth_image_Range = self.cv2.inRange(self.Depth_image_Raw, 11, self.max_depth )
    self.Depth_image_Raw_uni = cv2.normalize(self.Depth_image_Range, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    _, self.Depth_image_Raw_binary = cv2.threshold( self.Depth_image_Raw_uni, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    self.Depth_image_Raw_binary = cv2.medianBlur(self.Depth_image_Raw_binary, 3)
    self.Depth_image_Raw_binary = self.remove_small_foreground_components(self.Depth_image_Raw_binary)
    depth_binary_msg = self.bridge.cv2_to_imgmsg(self.Depth_image_Raw_binary, encoding='mono8')
    depth_binary_msg.header.stamp = rospy.Time.now()
    depth_binary_msg.header.frame_id = 'Scepter_depth_frame'
    self.depth_binary_image_pub.publish(depth_binary_msg)
    # # 细化处理
    self.skeleton = ximgproc.thinning(self.Depth_image_Raw_binary, thinningType=ximgproc.THINNING_ZHANGSUEN)

    # 使用滑动条参数的霍夫变换检测直线
    self.lines = self.cv2.HoughLinesP(self.skeleton, rho=1, theta=np.pi/180, threshold=self.threshold, minLineLength=self.minLineLength, maxLineGap=self.maxLineGap)

    # 新增角度过滤逻辑
    if self.lines is not None:
        filtered_lines = []
        for line in self.lines:
            x1, y1, x2, y2 = line[0]
            if self.is_near_axis_aligned_line(x1, y1, x2, y2):
                filtered_lines.append(line)

        self.lines = np.array(filtered_lines)
    self.line_image = np.zeros_like(self.image_infrared)

    # 绘制检测到的直线
    if self.lines is not None:
        for line in self.lines:
            x1, y1, x2, y2 = line[0]
            self.cv2.line(self.line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

    line_image_msg = self.bridge.cv2_to_imgmsg(self.line_image, encoding='mono8')
    line_image_msg.header.stamp = rospy.Time.now()
    line_image_msg.header.frame_id = 'Scepter_ir_frame'
    self.line_image_pub.publish(line_image_msg)



    # 计算所有线段对的交点
    if self.lines is not None:
        self.intersections, self.angles = self.calculate_intersections(self.lines)
    else:
        self.intersections, self.angles = [], []

    lines_count = 0 if self.lines is None else len(self.lines)
    intersections_count = 0 if self.intersections is None else len(self.intersections)

    # 如果没有交点或交点数组为空,直接返回
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
        }
        rospy.logwarn(
            "pointAI调试: lines=%d intersections=%d centers=0 world_fallback=0 roi_reject=0 zero_world=0 candidate_points=0 in_range_candidates=0 selected_points=0 out_of_range_points=0 duplicate_removed_points=0",
            lines_count,
            intersections_count
        )
        return None  # 返回空列表

    # 使用 DBSCAN 算法对交点进行聚类
    self.clustering = DBSCAN(eps=15, min_samples=1).fit(self.intersections)

    # 获取聚类后的标签
    self.labels = self.clustering.labels_
    # 创建一个字典来存储每个聚类的点
    self.clusters = {}
    for i, label in enumerate(self.labels):
        if label not in self.clusters:
            self.clusters[label] = []
        self.clusters[label].append(self.intersections[i])

    # 计算每个聚类的中心点
    self.centers = []
    world_fallback_count = 0
    for points in self.clusters.values():
        center_x = int(sum(p[0] for p in points) / len(points)) - self.offset_x
        center_y = int(sum(p[1] for p in points) / len(points)) - self.offset_y
        world_coord, _, used_fallback = self.get_valid_world_coord_near_pixel(center_x, center_y)
        if used_fallback:
            world_fallback_count += 1
        self.centers.append([center_x, center_y, world_coord])

    candidate_centers = []
    roi_reject_count = 0
    zero_world_count = 0
    for source_idx, center in enumerate(self.centers):
        pix_coord = [int(center[0]), int(center[1])]

        if not self.is_point_in_roi(center[0], center[1]):
            roi_reject_count += 1
            continue

        x_value, y_value, z_value = center[2]
        if x_value == 0 or y_value == 0 or z_value == 0:
            zero_world_count += 1
            continue

        calibrated_x, calibrated_y, calibrated_z = [
            float(x_value),
            float(y_value),
            float(z_value),
        ]

        center_record = (source_idx, center, [calibrated_x, calibrated_y, calibrated_z])
        candidate_centers.append(center_record)

    raw_candidate_count = len(candidate_centers)
    candidate_centers, duplicate_removed_count = self.filter_candidate_centers_for_request_mode(
        candidate_centers,
        request_mode,
    )
    matrix_selection_pixel_mask = (
        None
        if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY
        else self.get_travel_range_pixel_mask()
    )
    in_range_centers = []
    out_of_range_count = 0
    out_of_range_reason_counts = {}
    out_of_range_samples = []
    for center_record in candidate_centers:
        calibrated_x, calibrated_y, _ = center_record[2]
        if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
            point_is_allowed = self.is_point_in_scan_workspace(calibrated_x, calibrated_y)
        else:
            point_is_allowed = self.is_point_in_matrix_selection_pixel_mask(
                center_record[1][0],
                center_record[1][1],
                matrix_selection_pixel_mask,
            )

        if point_is_allowed:
            in_range_centers.append(center_record)
        else:
            out_of_range_count += 1
            if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
                reject_reasons = ["规划工作区外"]
            elif request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:
                reject_reasons = ["超出执行微调框"]
            else:
                reject_reasons = ["超出自适应采集框"]
            for reason in reject_reasons:
                out_of_range_reason_counts[reason] = out_of_range_reason_counts.get(reason, 0) + 1
            if len(out_of_range_samples) < 5:
                source_idx, center, calibrated_world_coord = center_record
                pix_coord = [int(center[0]), int(center[1])]
                out_of_range_samples.append(
                    f"idx={source_idx},pix=({pix_coord[0]},{pix_coord[1]}),coord=({calibrated_world_coord[0]:.1f},{calibrated_world_coord[1]:.1f},{calibrated_world_coord[2]:.1f}),原因={'+'.join(reject_reasons) if reject_reasons else '未知'}"
                )

    if request_mode in (PROCESS_IMAGE_MODE_SCAN_ONLY, PROCESS_IMAGE_MODE_EXECUTION_REFINE):
        self.sorted_centers = []
    else:
        self.sorted_centers = self.select_nearest_origin_matrix_points(in_range_centers)
    if (
        request_mode not in (PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT, PROCESS_IMAGE_MODE_SCAN_ONLY, PROCESS_IMAGE_MODE_EXECUTION_REFINE)
        and in_range_centers
        and not self.sorted_centers
    ):
        rospy.logwarn_throttle(
            1.0,
            "pointAI调试: 可执行范围内只有%d个点，无法组成2x2矩阵，暂不放给下游。",
            len(in_range_centers)
        )
    candidate_point_count = len(candidate_centers)
    in_range_candidate_count = len(in_range_centers)
    selected_count = len(self.sorted_centers)
    out_of_range_point_count = out_of_range_count
    output_centers = self.select_output_centers_for_mode(
        request_mode,
        in_range_centers,
        self.sorted_centers
    )
    output_count = len(output_centers)
    if request_mode in (PROCESS_IMAGE_MODE_SCAN_ONLY, PROCESS_IMAGE_MODE_EXECUTION_REFINE):
        display_centers = list(in_range_centers)
    else:
        display_centers = self.select_display_matrix_centers(self.sorted_centers, in_range_centers, candidate_centers)
    self.result_display_points = self.build_matrix_display_points(display_centers)

    self.last_detection_debug = {
        "lines": lines_count,
        "intersections": intersections_count,
        "centers": len(self.centers),
        "world_fallback": world_fallback_count,
        "roi_reject": roi_reject_count,
        "zero_world": zero_world_count,
        "candidate_points": candidate_point_count,
        "in_range_candidates": in_range_candidate_count,
        "selected_points": selected_count,
        "out_of_range_points": out_of_range_point_count,
        "duplicate_removed_points": duplicate_removed_count,
        "output_points": output_count,
    }
    rospy.loginfo(
        "pointAI调试: lines=%d intersections=%d centers=%d world_fallback=%d roi_reject=%d zero_world=%d candidate_points=%d in_range_candidates=%d selected_points=%d out_of_range_points=%d duplicate_removed_points=%d output_points=%d",
        self.last_detection_debug["lines"],
        self.last_detection_debug["intersections"],
        self.last_detection_debug["centers"],
        self.last_detection_debug["world_fallback"],
        self.last_detection_debug["roi_reject"],
        self.last_detection_debug["zero_world"],
        self.last_detection_debug["candidate_points"],
        self.last_detection_debug["in_range_candidates"],
        self.last_detection_debug["selected_points"],
        self.last_detection_debug["out_of_range_points"],
        self.last_detection_debug["duplicate_removed_points"],
        self.last_detection_debug["output_points"]
    )
    detection_summary_log = self.build_detection_summary_log(
        request_mode=request_mode,
        raw_candidate_count=raw_candidate_count,
        duplicate_removed_count=duplicate_removed_count,
        in_range_candidate_count=in_range_candidate_count,
        out_of_range_point_count=out_of_range_point_count,
        selected_count=selected_count,
        output_count=output_count,
        out_of_range_reason_counts=out_of_range_reason_counts,
        out_of_range_samples=out_of_range_samples,
    )
    rospy.logwarn_throttle(1.0, detection_summary_log)
    self.PointCoordinates_array_msg = PointsArray()
    self.PointCoordinates_array_msg.PointCoordinatesArray = []
    # 在pre_img方法中的遍历部分进行修改
    marker_point_idx = None  # 标记点的索引
    skip_next = False       # 是否跳过下一个点
    for idx, (_, center, calibrated_world_coord) in enumerate(output_centers, start=1):
        x, y = int(center[0]), int(center[1])
        self.x_value, self.y_value, self.z_value = calibrated_world_coord

        if self.x_value == 0 or self.y_value == 0 or self.z_value == 0:
            rospy.logwarn_throttle(
                2.0,
                "pointAI invalid raw camera coord: x=%s, y=%s, z=%s",
                self.x_value,
                self.y_value,
                self.z_value,
            )
            continue

        raw_z = self.depth_v[y, x]
        # 使用save_image函数判断是否已绑过
        # is_already_bound = self.save_image(x, y, raw_z)
        self.is_shuiguan = False
        # print("x_value:", self.x_value, "y_value:", self.y_value, "z_value:", self.z_value)

        raw_world_coord = center[2]
        log_manual_workspace_s2_camera_distance(
            self,
            idx,
            x,
            y,
            raw_world_coord,
        )

        # 创建点坐标对象并添加到数组
        self.PointCoordinates = PointCoords()
        self.PointCoordinates.is_shuiguan = self.is_shuiguan
        self.PointCoordinates.Angle = -45 #float((self.angles[idx] / 2) - 88) - 8
        self.PointCoordinates.idx = idx
        self.PointCoordinates.Pix_coord = [x, y]
        self.PointCoordinates.World_coord = [
            float(raw_world_coord[0]),
            float(raw_world_coord[1]),
            float(raw_world_coord[2]),
        ]
        self.PointCoordinates_array_msg.PointCoordinatesArray.append(self.PointCoordinates)

    self.PointCoordinates_array_msg.count = len(self.PointCoordinates_array_msg.PointCoordinatesArray)
    self.coordinate_publisher.publish(self.PointCoordinates_array_msg)
    self.publish_raw_camera_bind_point_transforms(self.PointCoordinates_array_msg)




    # 返回处理结果
    return self.PointCoordinates_array_msg
