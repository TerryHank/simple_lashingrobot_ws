#!/usr/bin/env python3

"""
    索驱并联式钢筋绑扎机器人-点云处理节点
"""
import warnings
warnings.filterwarnings("ignore", message="The value of the smallest subnormal for")
from cv2.ppf_match_3d import Pose3D
import rospy
import json
import yaml
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage , CameraInfo # 添加CompressedImage
import numpy as np
from sklearn.cluster import DBSCAN
from chassis_ctrl.srv import linear_module_move, linear_module_moveRequest, linear_module_moveResponse
from fast_image_solve.msg import PointsArray,PointCoords
import math
from chassis_ctrl.msg import motion
from fast_image_solve.srv import ProcessImage, ProcessImageResponse ,PlaneDetection, PlaneDetectionResponse
from std_srvs.srv import Trigger, TriggerResponse
import time
from std_msgs.msg import Int32 ,Float32, Bool
from cv2 import ximgproc
import os
# from ultralytics import YOLO
import torch
import tf2_ros  # 确保已导入
import numpy as np
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose

PROCESS_IMAGE_MODE_DEFAULT = 0
PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT = 1
PROCESS_IMAGE_MODE_BIND_CHECK = 2
PROCESS_IMAGE_MODE_SCAN_ONLY = 3  # pseudo_slam scan mode

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv2=cv2
        self.image = None
        self.vison_image  = None
        self.depth_image = None
        self.tof_intrin = None
        self.intersections = None
        # self.threshold = 60
        # self.minLineLength = 120
        # self.maxLineGap = 100
        self.threshold = 45
        self.minLineLength = 60
        self.maxLineGap = 150
        self.roi_center_x, self.roi_center_y = 362, 258
        self.roi_width, self.roi_height = 313, 277
        self.key = None
        # self.min_depth = 600
        # self.max_depth = 763
        self.min_depth = 910
        self.max_depth = 1060
        self.location_msg = motion()
        self.frame_count = 0
        self.fps = 0
        self.offset_x, self.offset_y = 0,0
        self.multiple = 0.996
        self.x1, self.y1 = 160, 0  # 区域1左上角坐标
        self.x2, self.y2 = 523, 80  # 区域1右下角坐标
        self.x3, self.y3 = 80, 406 # 区域2左上角坐标
        self.x4, self.y4 = 187, 480  # 区域2右下角坐标
        self.start_time = time.time()
        self.roi_bottom_left_x = 205
        self.roi_bottom_left_y = 358
        self.roi_top_right_x = 440
        self.roi_top_right_y = 120
        self.filter_points = True
        self.half_size = 20
        self.non_shuiguan_count = 0
        self.shuiguan_count = 0
        self.auto_cali_move_x=250
        self.auto_cali_move_y=0
        self.auto_cali_move_z=0
        self.fixed_z_value = 0.0
        self.height_threshold = 10
        self.cali_matrix_file = "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pose_matrix.npy"
        self.cali_offset_file = "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/lashing_config.json"
        self.path_points_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "data", "path_points.json")
        )
        self.gripper_tf_config_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "config", "gripper_tf.yaml")
        )
    
        # 初始化TF相关组件 - 移到这里，在订阅器之前
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 初始化图像副本
        self.image_infrared_copy = np.zeros((480, 640), dtype=np.uint8)
        self.image_infrared = None
        self.result_display_points = []
        self.last_detection_debug = {}
        self.show_candidate_labels = rospy.get_param("~show_candidate_labels", False)
        self.jump_bind_enabled = False
        self.axis_alignment_tolerance_deg = float(rospy.get_param("~axis_alignment_tolerance_deg", 15.0))
        self.process_request_rate_hz = max(1.0, float(rospy.get_param("~process_request_rate_hz", 10.0)))
        self.process_wait_timeout_sec = float(rospy.get_param("~process_wait_timeout_sec", 0.0))
        self.stable_frame_count = max(1, int(rospy.get_param("~stable_frame_count", 3)))
        self.stable_z_tolerance_mm = float(rospy.get_param("~stable_z_tolerance_mm", 5.0))
        self.bind_check_max_height_mm = float(rospy.get_param("~bind_check_max_height_mm", 95.0))
        self.travel_range_max_x_mm = float(rospy.get_param("~travel_range_max_x_mm", 320.0))
        self.travel_range_max_y_mm = float(rospy.get_param("~travel_range_max_y_mm", 320.0))
        self.display_bind_range_max_x_mm = float(rospy.get_param("~display_bind_range_max_x_mm", 360.0))
        self.display_bind_range_max_y_mm = float(rospy.get_param("~display_bind_range_max_y_mm", 360.0))
        self.world_image_seq = 0
        
        rospy.Subscriber('/Scepter/worldCoord/world_coord', Image, self.image_callback)
        rospy.Subscriber('/Scepter/worldCoord/raw_world_coord', Image, self.image_raw_world_callback)
        rospy.Subscriber('/Scepter/color/image_raw', Image, self.image_color_callback)
        rospy.Subscriber('/Scepter/ir/image_raw', Image, self.image_infrared_callback)
        rospy.Subscriber('/Scepter/depth/image_raw', Image, self.image_depth_callback)
        rospy.Subscriber('/Scepter/ir/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/web/fast_image_solve/set_pointAI_offset', Pose, self.calibration_offset_callback) # 兼容旧前端，空间标定现仅由TF提供
        rospy.Subscriber('/web/fast_image_solve/set_height_threshold', Float32, self.fixed_z_value_callback)  # 接收固定z值话题
        rospy.Subscriber('/web/moduan/send_odd_points', Bool, self.jump_bind_callback)
    
        # self.detect_and_save_pose_srv = rospy.Service('/web/fast_image_solve/detect_and_save_pose', Trigger, self.detect_and_save_pose_service)
        self.service = rospy.Service('/pointAI/process_image', ProcessImage, self.handle_process_image)
        # 新增三个Image格式发布器
        self.cropped_ir_image_pub = rospy.Publisher('/pointAI/cropped_ir_image', Image, queue_size=10)
        self.cropped_color_image_pub = rospy.Publisher('/pointAI/cropped_color_image', Image, queue_size=10)
        self.cropped_depth_image_pub = rospy.Publisher('/pointAI/cropped_depth_image', Image, queue_size=10)
        self.depth_binary_image_pub = rospy.Publisher('/pointAI/depth_binary_image', Image, queue_size=10)
        self.line_image_pub = rospy.Publisher('/pointAI/line_image', Image, queue_size=10)
        self.image_pub = rospy.Publisher('/pointAI/result_image', CompressedImage, queue_size=10)
        self.result_image_raw_pub = rospy.Publisher('/pointAI/result_image_raw', Image, queue_size=10)
        # rospy.Subscriber('/cabin/lashing_request', motion, self.printsomething) 
       
        self.plane_z = None
        self.coordinate_publisher = rospy.Publisher('/coordinate_point', PointsArray, queue_size=10)

        # device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # 加载 YOLO 模型并指定设备
        # self.yolov11 = YOLO("/home/car/lashingrobots/src/fast_image_solve/scripts/mask2best.pt")
        # self.yolov11.to(device)
        # print(f"YOLO model loaded on {device}")
        
        self.pose_matrix = None  # 添加pose_matrix初始化
        # # 发布每个中心点的坐标和对应的通道值
        self.frame_count = 0
        self.fps=0
        self.start_time = time.time()
        
        # 添加矩形框的坐标点作为类属性
        self.point1 = (20, 114)  # 点1坐标
        self.point2 = (620, 460)  # 点2坐标
        
        # 初始化时间戳
        self.stamp = rospy.Time.now()
        
        self.T_camera_ee = None
        self.camera_matrix = None
        self.dist_coeffs = None   

        self.load_runtime_config()

        print("pointAI node initialized!")
        print(
            "spatial_calibration_source=tf(Scepter_depth_frame->gripper_frame), "
            "tf_config={}, height_threshold={}".format(
                self.gripper_tf_config_file,
                self.height_threshold
            )
        )

    def load_runtime_config(self):
        if not os.path.exists(self.cali_offset_file):
            return

        try:
            with open(self.cali_offset_file, 'r') as f:
                data = json.load(f)
        except Exception as exc:
            rospy.logwarn("读取%s失败，仅使用默认高度阈值: %s", self.cali_offset_file, exc)
            return

        self.height_threshold = data.get('height_threshold', self.height_threshold)
        if any(key in data for key in ('cal_x', 'cal_y', 'cal_z')):
            rospy.logwarn(
                "pointAI: lashing_config.json中的cal_x/cal_y/cal_z已弃用，"
                "请改%s里的translation_mm。",
                self.gripper_tf_config_file,
            )

    def save_runtime_config(self):
        data = {}
        if os.path.exists(self.cali_offset_file):
            try:
                with open(self.cali_offset_file, 'r') as f:
                    data = json.load(f)
            except Exception:
                data = {}

        data.pop('cal_x', None)
        data.pop('cal_y', None)
        data.pop('cal_z', None)
        data['height_threshold'] = self.height_threshold

        with open(self.cali_offset_file, 'w') as f:
            json.dump(data, f, indent=4)

    def update_gripper_tf_translation_mm(self, x_mm, y_mm, z_mm):
        if not os.path.exists(self.gripper_tf_config_file):
            raise FileNotFoundError(self.gripper_tf_config_file)

        with open(self.gripper_tf_config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}

        config.setdefault('parent_frame', 'Scepter_depth_frame')
        config.setdefault('child_frame', 'gripper_frame')
        config.setdefault(
            'rotation_rpy',
            {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        )
        config['translation_mm'] = {
            'x': float(x_mm),
            'y': float(y_mm),
            'z': float(z_mm),
        }

        with open(self.gripper_tf_config_file, 'w', encoding='utf-8') as f:
            yaml.safe_dump(config, f, sort_keys=False, allow_unicode=True)

    def fixed_z_value_callback(self, msg):
        """
        /fixed_z_value 回调: 若值不为0将在计算点坐标时强制替换z
        """
        if msg.data < 20:
            self.height_threshold = msg.data
            self.save_runtime_config()
            rospy.loginfo(f"设置高度阈值为: {self.height_threshold}")
        else:
            self.fixed_z_value = msg.data
            rospy.loginfo(f"接收到固定下探Z值: {self.fixed_z_value}") 
        
    def calibration_offset_callback(self, msg):
        """
        兼容旧前端的回调函数。
        旧的视觉偏差入口现在直接写入 gripper_tf.yaml.translation_mm。
        """
        try:
            self.update_gripper_tf_translation_mm(
                msg.position.x,
                msg.position.y,
                msg.position.z,
            )
            rospy.loginfo(
                "pointAI: 已将旧入口/web/fast_image_solve/set_pointAI_offset映射为TF平移标定，"
                "写入%s: translation_mm=(%.3f, %.3f, %.3f)。请重启pointai_tf_verify.launch后验证。",
                self.gripper_tf_config_file,
                msg.position.x,
                msg.position.y,
                msg.position.z,
            )
        except Exception as exc:
            rospy.logerr(
                "pointAI: 写入%s失败，无法更新translation_mm: %s",
                self.gripper_tf_config_file,
                exc,
            )

    def image_raw_world_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)          # 先取图
        img = np.array(img, copy=True)                # 保证可写
        # ① 通道对调  X↔Y
        img[:, :, 0], img[:, :, 1] = img[:, :, 1], img[:, :, 0].copy()
        # ② 新 Y 翻符号 → 正向朝上
        img[:, :, 1] *= -1
        self.image_raw_world = img  
    def printsomething(self, msg):
        print("msg:", msg.data)
    @staticmethod
    # @numba.jit(nopython=True)
    def calculate_intersections(lines):
        intersections = []
        angles = []
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                x1, y1, x2, y2 = lines[i][0]
                x3, y3, x4, y4 = lines[j][0]
                
                # 计算每条线段的方向向量
                v1 = (x2 - x1, y2 - y1)
                v2 = (x4 - x3, y4 - y3)
                
                # 计算两条线的夹角
                dot_product = v1[0] * v2[0] + v1[1] * v2[1]
                cross_product = v1[0] * v2[1] - v1[1] * v2[0]
                angle_between_lines = np.arctan2(cross_product, dot_product)
                angle_between_lines = np.abs(np.degrees(angle_between_lines))
                
                if 35 <= angle_between_lines <= 150:
                    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
                    if denom == 0:
                        continue  # 平行,没有交点
                    
                    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
                    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
                    
                    if (min(x1, x2) <= px <= max(x1, x2) and min(y1, y2) <= py <= max(y1, y2) and
                        min(x3, x4) <= px <= max(x3, x4) and min(y3, y4) <= py <= max(y3, y4)):
                        intersections.append((px, py))
                        angles.append(angle_between_lines)
        return intersections, angles

    
    def get_text_bbox(self, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.23, thickness=1):
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        x, y = position
        top_left = (x, y - text_height - baseline)
        bottom_right = (x + text_width, y + baseline)
        return top_left, bottom_right, text_width, text_height, baseline

    @staticmethod
    def bboxes_overlap(bbox1, bbox2, padding=4):
        left1, top1, right1, bottom1 = bbox1
        left2, top2, right2, bottom2 = bbox2
        return not (
            right1 + padding < left2 or
            right2 + padding < left1 or
            bottom1 + padding < top2 or
            bottom2 + padding < top1
        )

    def find_non_overlapping_label_position(self, image_shape, anchor_point, text, occupied_bboxes, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.23, thickness=1):
        image_height, image_width = image_shape[:2]
        (_, _), (_, _), text_width, text_height, baseline = self.get_text_bbox(
            text, (0, 0), font=font, font_scale=font_scale, thickness=thickness
        )
        min_baseline_y = text_height + baseline + 1
        max_baseline_y = max(min_baseline_y, image_height - baseline - 1)
        max_text_x = max(0, image_width - text_width - 1)
        anchor_x, anchor_y = int(anchor_point[0]), int(anchor_point[1])

        offset_levels = [10, 22, 34, 46, 58, 70]
        for offset in offset_levels:
            candidate_positions = [
                (anchor_x + offset, anchor_y - offset),
                (anchor_x - text_width - offset, anchor_y - offset),
                (anchor_x + offset, anchor_y + text_height + offset),
                (anchor_x - text_width - offset, anchor_y + text_height + offset),
                (anchor_x - (text_width // 2), anchor_y - offset),
                (anchor_x - (text_width // 2), anchor_y + text_height + offset),
            ]

            for candidate_x, candidate_y in candidate_positions:
                clamped_x = int(np.clip(candidate_x, 0, max_text_x))
                clamped_y = int(np.clip(candidate_y, min_baseline_y, max_baseline_y))
                top_left, bottom_right, _, _, _ = self.get_text_bbox(
                    text, (clamped_x, clamped_y), font=font, font_scale=font_scale, thickness=thickness
                )
                candidate_bbox = (top_left[0], top_left[1], bottom_right[0], bottom_right[1])
                if not any(self.bboxes_overlap(candidate_bbox, used_bbox) for used_bbox in occupied_bboxes):
                    return (clamped_x, clamped_y), candidate_bbox

        scan_step_y = max(12, text_height + baseline + 4)
        scan_step_x = max(24, text_width // 3)
        for scan_y in range(min_baseline_y, max_baseline_y + 1, scan_step_y):
            for scan_x in range(0, max_text_x + 1, scan_step_x):
                top_left, bottom_right, _, _, _ = self.get_text_bbox(
                    text, (scan_x, scan_y), font=font, font_scale=font_scale, thickness=thickness
                )
                candidate_bbox = (top_left[0], top_left[1], bottom_right[0], bottom_right[1])
                if not any(self.bboxes_overlap(candidate_bbox, used_bbox) for used_bbox in occupied_bboxes):
                    return (scan_x, scan_y), candidate_bbox

        fallback_x = int(np.clip(anchor_x + 10, 0, max_text_x))
        fallback_y = int(np.clip(anchor_y - 10, min_baseline_y, max_baseline_y))
        top_left, bottom_right, _, _, _ = self.get_text_bbox(
            text, (fallback_x, fallback_y), font=font, font_scale=font_scale, thickness=thickness
        )
        fallback_bbox = (top_left[0], top_left[1], bottom_right[0], bottom_right[1])
        return (fallback_x, fallback_y), fallback_bbox

    def draw_text_with_background(self,image, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.23, text_color=(255, 255, 255), bg_color=(0, 0, 0), thickness=1):
        top_left, bottom_right, _, _, _ = self.get_text_bbox(
            text, position, font=font, font_scale=font_scale, thickness=thickness
        )
        # 绘制背景矩形
        cv2.rectangle(image, top_left, bottom_right, bg_color, cv2.FILLED)
        # 绘制文本
        cv2.putText(image, text, position, font, font_scale, text_color, thickness, cv2.LINE_AA)

    def jump_bind_callback(self, msg):
        self.jump_bind_enabled = bool(msg.data)

    def should_execute_selected_point_number(self, point_number):
        if point_number in (None, "-"):
            return False
        return not bool(getattr(self, "jump_bind_enabled", False)) or int(point_number) in (1, 4)

    def get_selected_point_status(self, point_number):
        return "selected" if self.should_execute_selected_point_number(point_number) else "jump_skipped"

    def get_travel_range_reject_reasons(self, calibrated_x, calibrated_y):
        max_x = getattr(self, "travel_range_max_x_mm", 320.0)
        max_y = getattr(self, "travel_range_max_y_mm", 320.0)
        reasons = []
        if calibrated_x < 0:
            reasons.append("X小于0")
        elif calibrated_x > max_x:
            reasons.append(f"X超过{max_x:.0f}")
        if calibrated_y < 0:
            reasons.append("Y小于0")
        elif calibrated_y > max_y:
            reasons.append(f"Y超过{max_y:.0f}")
        return reasons

    def get_display_status_text(self, status, detail=""):
        if status == "selected":
            return "SEL"
        if status == "jump_skipped":
            return "SKIP"
        if status == "in_range":
            return "IN"
        if status == "out_of_range":
            return detail or "OUT"
        if status == "zero_world":
            return "ZERO"
        return "CAND"

    @staticmethod
    def axis_angle_diff_deg(x1, y1, x2, y2):
        angle_deg = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1))) % 180.0
        return min(angle_deg, abs(90.0 - angle_deg), abs(180.0 - angle_deg))

    def is_near_axis_aligned_line(self, x1, y1, x2, y2):
        return self.axis_angle_diff_deg(x1, y1, x2, y2) <= getattr(
            self,
            "axis_alignment_tolerance_deg",
            15.0
        )

    def should_draw_display_label(self, status):
        return status in ("selected", "jump_skipped", "in_range", "out_of_range", "zero_world") or bool(getattr(self, "show_candidate_labels", False))

    def angle_between(self, line1, line2):
        x1, y1, x2, y2 = line1[0]
        x3, y3, x4, y4 = line2[0]
        
        # 计算每条线段的方向向量
        v1 = (x2 - x1, y2 - y1)
        v2 = (x4 - x3, y4 - y3)
        
        # 计算两条线的夹角
        dot_product = v1[0] * v2[0] + v1[1] * v2[1]
        cross_product = v1[0] * v2[1] - v1[1] * v2[0]
        angle_between_lines = np.arctan2(cross_product, dot_product)
        angle_between_lines = np.abs(np.degrees(angle_between_lines))
        
        # 计算每条线段的单位方向向量
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        if v1_norm != 0:
            v1_unit = (v1[0] / v1_norm, v1[1] / v1_norm)
        else:
            v1_unit = (0, 0)  # 或者其他处理方式

        if v2_norm != 0:
            v2_unit = (v2[0] / v2_norm, v2[1] / v2_norm)
        else:
            v2_unit = (0, 0)  # 或者其他处理方式
        
        # 计算角平分线的方向向量
        bisector = (v1_unit[0] + v2_unit[0], v1_unit[1] + v2_unit[1])
        bisector_norm = np.linalg.norm(bisector)
        if bisector_norm != 0:
            bisector_unit = (bisector[0] / bisector_norm, bisector[1] / bisector_norm)
        else:
            bisector_unit = (0, 0)  
        
        # 计算角平分线与水平线的夹角
        angle_with_horizontal = np.arctan2(bisector_unit[1], bisector_unit[0])
        angle_with_horizontal = np.degrees(angle_with_horizontal)
        return angle_between_lines, angle_with_horizontal

    def get_valid_world_coord_near_pixel(self, pixel_x, pixel_y, search_radius=6):
        height, width = self.x_channel.shape
        pixel_x = int(np.clip(pixel_x, 0, width - 1))
        pixel_y = int(np.clip(pixel_y, 0, height - 1))

        raw_world_coord = [
            int(self.x_channel[pixel_y, pixel_x]),
            int(self.y_channel[pixel_y, pixel_x]),
            int(self.depth_v[pixel_y, pixel_x]),
        ]
        if raw_world_coord[0] != 0 and raw_world_coord[1] != 0 and raw_world_coord[2] != 0:
            return raw_world_coord, [pixel_x, pixel_y], False

        best_world_coord = None
        best_sample_pixel = None
        best_distance = None

        for radius in range(1, search_radius + 1):
            min_y = max(0, pixel_y - radius)
            max_y = min(height - 1, pixel_y + radius)
            min_x = max(0, pixel_x - radius)
            max_x = min(width - 1, pixel_x + radius)

            for sample_y in range(min_y, max_y + 1):
                for sample_x in range(min_x, max_x + 1):
                    sample_world_coord = [
                        int(self.x_channel[sample_y, sample_x]),
                        int(self.y_channel[sample_y, sample_x]),
                        int(self.depth_v[sample_y, sample_x]),
                    ]
                    if sample_world_coord[0] == 0 or sample_world_coord[1] == 0 or sample_world_coord[2] == 0:
                        continue

                    distance = (sample_x - pixel_x) ** 2 + (sample_y - pixel_y) ** 2
                    if best_distance is None or distance < best_distance:
                        best_distance = distance
                        best_world_coord = sample_world_coord
                        best_sample_pixel = [sample_x, sample_y]

            if best_world_coord is not None:
                return best_world_coord, best_sample_pixel, True

        return raw_world_coord, [pixel_x, pixel_y], False
    
    def test_callback(self):
        self.frame_count += 1
        # 计算并打印平均帧数
        cur_time =time.time()
        elapsed_time = cur_time - self.start_time
        if elapsed_time > 0.5:
            self.fps = self.frame_count / elapsed_time
            self.frame_count=0
            self.start_time = cur_time
        print(f"Average FPS: {self.fps:.2f}")

    def image_color_callback(self, msg):
        # 缓存最新的图像
        self.image_color = self.bridge.imgmsg_to_cv2(msg)
        # # 顺时针旋转90度
        # self.image_infrared = cv2.rotate(self.image_infrared, cv2.ROTATE_90_CLOCKWISE)
        
        # 确保图像是可写的
        self.image_color_copy = np.array(self.image_color, copy=True)

    def image_depth_callback(self, msg):
        # 缓存最新的图像
        self.raw_depth = self.bridge.imgmsg_to_cv2(msg)
        # 确保图像是可写的
        self.raw_depth = np.array(self.raw_depth, copy=True)

    def image_infrared_callback(self, msg):
        # 缓存最新的图像
        self.image_infrared = self.bridge.imgmsg_to_cv2(msg)
        # # 顺时针旋转90度
        # self.image_infrared = cv2.rotate(self.image_infrared, cv2.ROTATE_90_CLOCKWISE)
        
        # 确保图像是可写的
        self.image_infrared_copy = np.array(self.image_infrared, copy=True)
        
    def transform_point_to_frame(self, x, y, z, target_frame, source_frame="Scepter_depth_frame"):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
            )

            T = quaternion_matrix([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            T[:3, 3] = [transform.transform.translation.x * 1000,
                        transform.transform.translation.y * 1000,
                        transform.transform.translation.z * 1000]

            point_source = np.array([x, y, z, 1])
            point_target = np.dot(T, point_source)

            return (
                int(round(point_target[0])),
                int(round(point_target[1])),
                int(round(point_target[2])),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"坐标转换失败: {str(e)}")
        except Exception as e:
            rospy.logerr_throttle(5, f"坐标转换未知错误: {str(e)}")

    def transform_to_gripper_frame(self, x, y, z, idx):
        del idx
        return self.transform_point_to_frame(x, y, z, "gripper_frame")

    def transform_to_cabin_frame(self, x, y, z, idx):
        del idx
        return self.transform_point_to_frame(x, y, z, "cabin_frame")

    def apply_spatial_calibration(self, x_value, y_value, z_value, idx, target_frame="gripper_frame"):
        if target_frame == "cabin_frame":
            transformed_point = self.transform_to_cabin_frame(x_value, y_value, z_value, idx)
        else:
            transformed_point = self.transform_to_gripper_frame(x_value, y_value, z_value, idx)
        if transformed_point is None:
            return None

        calibrated_x, calibrated_y, calibrated_z = [float(value) for value in transformed_point]
        if self.fixed_z_value != 0:
            calibrated_z = float(self.fixed_z_value)
        return [calibrated_x, calibrated_y, calibrated_z]

    def load_scan_planning_workspace(self):
        default_bounds = {
            "min_x": 0.0,
            "max_x": 0.0,
            "min_y": 0.0,
            "max_y": 0.0,
        }
        scan_workspace_padding_mm = 100.0
        try:
            if not os.path.exists(self.path_points_file):
                return default_bounds

            with open(self.path_points_file, "r", encoding="utf-8") as file_obj:
                path_json = json.load(file_obj)

            path_points = path_json.get("path_points", [])
            if not path_points:
                return default_bounds

            point_x_values = [float(point["x"]) for point in path_points]
            point_y_values = [float(point["y"]) for point in path_points]
            return {
                "min_x": min(point_x_values) - scan_workspace_padding_mm,
                "max_x": max(point_x_values) + float(getattr(self, "travel_range_max_x_mm", 320.0)) + scan_workspace_padding_mm,
                "min_y": min(point_y_values) - scan_workspace_padding_mm,
                "max_y": max(point_y_values) + float(getattr(self, "travel_range_max_y_mm", 320.0)) + scan_workspace_padding_mm,
            }
        except Exception as exc:
            rospy.logwarn_throttle(5.0, f"pointAI扫描工作区读取失败: {exc}")
            return default_bounds

    def is_point_in_scan_workspace(self, world_x, world_y):
        workspace = self.load_scan_planning_workspace()
        if workspace["min_x"] == workspace["max_x"] and workspace["min_y"] == workspace["max_y"]:
            return True
        return (
            workspace["min_x"] <= world_x <= workspace["max_x"]
            and workspace["min_y"] <= world_y <= workspace["max_y"]
        )

    def mark_sign_points(self, step=4, max_points=8000):
        """
        黑色: x>0 且 y>0
        白色: x<0 且 y<0
        只处理部分像素(步长/抽样)避免过慢
        """
        if (not hasattr(self, 'x_channel')) or (not hasattr(self, 'y_channel')):
            return
        if self.x_channel is None or self.y_channel is None:
            return
        if self.image_infrared_copy is None:
            return
        
        x_ch = self.x_channel
        y_ch = self.y_channel

        # 条件掩码
        pos_mask = (x_ch > 0) & (y_ch > 0)
        neg_mask = (x_ch < 0) & (y_ch < 0)

        # 取坐标
        pos_idx = np.column_stack(np.where(pos_mask))
        neg_idx = np.column_stack(np.where(neg_mask))

        # 降采样
        pos_idx = pos_idx[::step]
        neg_idx = neg_idx[::step]

        # 限制最大数量
        if pos_idx.shape[0] > max_points:
            pos_idx = pos_idx[np.linspace(0, pos_idx.shape[0]-1, max_points, dtype=int)]
        if neg_idx.shape[0] > max_points:
            neg_idx = neg_idx[np.linspace(0, neg_idx.shape[0]-1, max_points, dtype=int)]

        # 绘制（注意图像是 (H,W)，坐标 (y,x)）
        for y, x in pos_idx:
            # 黑色
            self.image_infrared_copy[y, x] = 0
        for y, x in neg_idx:
            # 白色
            self.image_infrared_copy[y, x] = 255

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)
        self.vison_image = np.array(self.image, copy=True)
        normalized = cv2.normalize(self.vison_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        self.vison_image = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    
        # self.pre_img()
        self.channels = self.cv2.split(self.image)
        self.Depth_image_Raw =(self.channels[2]).astype(np.int32)
        self.world_image_seq += 1
        # self.mark_sign_points()
        # 确保image_infrared_copy存在
        if not hasattr(self, 'image_infrared_copy') or self.image_infrared_copy is None:
            self.image_infrared_copy = np.zeros((480, 640), dtype=np.uint8)

        result_image = np.array(self.image_infrared_copy, copy=True)
        cv2.rectangle(result_image, self.point1, self.point2, 255, 2)

        occupied_label_bboxes = []
        sorted_display_points = sorted(
            self.result_display_points,
            key=lambda item: (
                0 if item[3] == "selected" else 1 if item[3] == "jump_skipped" else 2,
                item[0] if isinstance(item[0], int) else 9999,
                item[1][1],
                item[1][0]
            )
        )
        for display_item in sorted_display_points:
            if len(display_item) == 4:
                display_idx, pix_coord, world_coord, status = display_item
                status_detail = ""
            else:
                display_idx, pix_coord, world_coord, status, status_detail = display_item
            if status == "selected":
                color = 255
                text_color = 255
                bg_color = 0
            elif status == "jump_skipped":
                color = 180
                text_color = 180
                bg_color = 0
            elif status == "in_range":
                color = 220
                text_color = 220
                bg_color = 0
            elif status == "out_of_range":
                color = 0
                text_color = 0
                bg_color = 255
            elif status == "zero_world":
                color = 160
                text_color = 160
                bg_color = 0
            else:
                color = 200
                text_color = 200
                bg_color = 0
            cv2.circle(result_image, (int(pix_coord[0]), int(pix_coord[1])), 3, color, -1)

            if not self.should_draw_display_label(status):
                continue

            status_text = self.get_display_status_text(status, status_detail)
            text = f"{display_idx}, {world_coord}, {status_text}"
            label_position, label_bbox = self.find_non_overlapping_label_position(
                result_image.shape,
                pix_coord,
                text,
                occupied_label_bboxes
            )
            occupied_label_bboxes.append(label_bbox)
            self.draw_text_with_background(
                result_image,
                text,
                label_position,
                text_color=text_color,
                bg_color=bg_color
            )

        result_image_msg = self.bridge.cv2_to_imgmsg(result_image, encoding='mono8')
        result_image_msg.header.stamp = rospy.Time.now()
        result_image_msg.header.frame_id = "infrared_camera"
        self.result_image_raw_pub.publish(result_image_msg)

        compress_msg = CompressedImage()
        compress_msg.header.stamp = result_image_msg.header.stamp
        compress_msg.header.frame_id = result_image_msg.header.frame_id
        compress_msg.format = "jpeg"

        _, jpeg_data = cv2.imencode(
            '.jpg', result_image, [cv2.IMWRITE_JPEG_QUALITY, 85]
        )

        compress_msg.data = jpeg_data.tobytes()
        self.image_pub.publish(compress_msg)
        
        return

  # 更新最大深度偏移量
    
    def transform_to_end_effector(self, x_obj,y_obj,z_obj,theta_obj):
        """
        ROS服务回调：将物体在相机坐标系下的位置和旋转角度转换到末端执行器坐标系
        :param req: TransformToEndEffectorRequest，包含x_obj, y_obj, z_obj, theta_obj
        :return: TransformToEndEffectorResponse，包含position_ee_obj, rotation_ee_obj
        """
        pose_matrix_path = self.cali_matrix_file
        if not os.path.exists(pose_matrix_path):
            position_ee_obj=[]
            rotation_ee_obj=[]
            return position_ee_obj, rotation_ee_obj
        T_camera_ee = np.load(pose_matrix_path)
        # 将平移部分从米转换为毫米
        # print("T_camera_ee",T_camera_ee)
        T_camera_ee[:3, 3] = T_camera_ee[:3, 3] * 1000
        R_camera_obj = self.create_rotation_matrix(theta_obj)
        T_camera_obj = np.eye(4)
        T_camera_obj[:3, :3] = R_camera_obj
        T_camera_obj[:3, 3] = [x_obj, y_obj, z_obj]
        T_ee_obj = np.dot(T_camera_ee, T_camera_obj)
        position_ee_obj = np.round(T_ee_obj[:3, 3]).astype(int).tolist()

        rotation_ee_obj = T_ee_obj[:3, :3].flatten().tolist()

        return position_ee_obj,rotation_ee_obj

    def camera_info_callback(self,msg):

        # 从 CameraInfo 消息中提取相机内参矩阵

        # self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]])
        # 提取畸变系数
        # self.dist_coeffs = np.array(msg.D)
        # self.dist_coeffs = np.zeros_like(np.array(msg.D))
        self.dist_coeffs = np.zeros((4, 1)) 

    def save_image(self,x,y,z_value):
        # 将图像缩放到 0 到 255 的范围
        
        # 计算ROI区域的边界
        x_start = x - self.half_size
        x_end = x + self.half_size
        y_start = y - self.half_size
        y_end = y + self.half_size

        # 确保ROI不超出图像边界
        x_start = max(0, x_start)
        x_end = min(self.image_infrared.shape[1], x_end)
        y_start = max(0, y_start)
        y_end = min(self.image_infrared.shape[0], y_end)
        # # 创建CLAHE对象（自适应直方图均衡化）
        clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))

        # 应用CLAHE处理
        clahe_image_infrared = clahe.apply(self.image_infrared)
        # 创建ROI区域的掩码，而不是整个图像的掩码
        roi_depth = self.raw_depth[y_start:y_end, x_start:x_end]
        roi_infrared = clahe_image_infrared[y_start:y_end, x_start:x_end]

        # 创建一个与ROI大小相同的空图像
        sub_image = np.zeros_like(roi_infrared, dtype=roi_infrared.dtype)

        # # 仅提取特定深度范围内的像素
        tolerance = 20
        mask = (roi_depth >= 0) & (roi_depth <= z_value + tolerance)
        
        # # 应用掩码到子图像
        sub_image[mask] = roi_infrared[mask]
        # sub_image = roi_infrared
        # 调整大小为标准尺寸
        new_image = cv2.resize(sub_image, (128, 128))



        
        results = self.yolov11(new_image, verbose=False,conf=0.9)
        # save_dir = '/home/car/lashingrobots/dataset/0'
        # os.makedirs(save_dir, exist_ok=True)  # 确保目录存在
        # self.shuiguan_count += 1
        # file_name = f"Have_image_{self.shuiguan_count}.png"  # 使用有水管计数器
        # cv2.imwrite(os.path.join(save_dir, file_name), new_image)
        # return True
        if results and hasattr(results[0], 'probs'):
                probs = results[0].probs
                top1_class = probs.top1  # 获取概率最高的类别
                if top1_class == 0:
                    save_dir = '/home/car/lashingrobots/src/dataset/binded'
                    os.makedirs(save_dir, exist_ok=True)  # 确保目录存在
                    file_name = f"Have_image_{self.shuiguan_count}.png"  # 使用有水管计数器
                    cv2.imwrite(os.path.join(save_dir, file_name), new_image)
                    self.non_shuiguan_count += 1  # 有水管计数器加1
                 
                    return True
                elif top1_class == 1:
      
                    save_dir = '/home/car/lashingrobots/src/dataset/nobinded'
                    os.makedirs(save_dir, exist_ok=True)  # 确保目录存在
                    file_name = f"None_image_{self.non_shuiguan_count}.png"  # 使用没水管计数器
                    cv2.imwrite(os.path.join(save_dir, file_name), new_image)
                    self.shuiguan_count += 1  # 没水管计数器加1
                 
                    return False
        
        # # 保存图像
        # save_dir = '/home/yjy/newcam/Dataset'
        # os.makedirs(save_dir, exist_ok=True)  # 确保目录存在
        # file_name = f"image_{self.shuiguan_count}.png"
        # cv2.imwrite(os.path.join(save_dir, file_name), new_image)
        # self.shuiguan_count += 1
    def call_linear_module_move_service(self,pos_x,pos_y,pos_z):
        # 等待服务可用（超时设置可选）
        rospy.wait_for_service('linear_module_move', timeout=10)
        
        try:
            # 创建服务代理（服务名称需与C++端广告的名称一致）
            linear_module_moveser = rospy.ServiceProxy('linear_module_move', linear_module_move)
            
            # 构造请求对象（字段需与srv文件定义一致）
            req = linear_module_moveRequest()
            req.pos_x = pos_x   # X轴目标位置（mm）
            req.pos_y = pos_y   # Y轴目标位置（mm）
            req.pos_z = pos_z    # Z轴目标位置（mm）
            
            # 调用服务并获取响应
            res = linear_module_moveser (req)
            
            # 处理响应
            if res.success:
                rospy.loginfo(f"服务调用成功: {res.message}")
            else:
                rospy.logwarn(f"服务调用失败: {res.message}")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")

    def detect_and_save_pose_service(self, req):
        self.call_linear_module_move_service(self.auto_cali_move_x,self.auto_cali_move_y,0)
        time.sleep(2)  # 避免CPU占用过高
        detected = False
        saved = False
       
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        
        start_time = time.time()
        while time.time() - start_time < 5:  # 5秒超时
            if hasattr(self, 'image_color_copy') and hasattr(self, 'camera_matrix') and hasattr(self, 'dist_coeffs'):
                corners, ids, _ = cv2.aruco.detectMarkers(self.image_color_copy, aruco_dict, parameters=parameters)
                if ids is not None and len(ids) > 0:
                    detected = True
                    
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.06, self.camera_matrix, self.dist_coeffs)
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
                    pose_matrix = np.eye(4)
                    pose_matrix[:3, :3] = rotation_matrix
                    pose_matrix[:3, 3] = tvecs[0][0]
                    pose_matrix[0, 3] -= self.auto_cali_move_y/1000
                    pose_matrix[1, 3] -= self.auto_cali_move_x/1000
                    pose_matrix[2, 3] -= self.auto_cali_move_z/1000


                    try:
                        np.save(self.cali_matrix_file, pose_matrix)
                        time.sleep(0.5)  # 避免CPU占用过高
                        
                        saved = True
                        break  # 成功获取矩阵后跳出循环
                    except Exception as e:
                        saved = False
                    
                time.sleep(0.1)  # 避免CPU占用过高
        
        self.call_linear_module_move_service(0,0,0)
        return TriggerResponse(success=saved, message="Detected: {}, Saved: {}".format(detected, saved))            

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
        return PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT

    def get_request_mode_name(self, request_mode):
        if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
            return "bind_check"
        if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
            return "scan_only"
        if request_mode == PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT:
            return "adaptive_height"
        return "default"

    def build_process_image_timing_message(self, request_mode, response, elapsed_sec):
        return (
            "pointAI process_image response: "
            f"mode={self.get_request_mode_name(request_mode)}, "
            f"success={getattr(response, 'success', False)}, "
            f"count={getattr(response, 'count', 0)}, "
            f"elapsed_ms={elapsed_sec * 1000.0:.1f}, "
            f"message={getattr(response, 'message', '')}"
        )

    def log_process_image_timing(self, request_mode, response, elapsed_sec):
        log_message = self.build_process_image_timing_message(request_mode, response, elapsed_sec)
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
                    else (
                        f"0<=x<={getattr(self, 'travel_range_max_x_mm', 320.0):.0f}, "
                        f"0<=y<={getattr(self, 'travel_range_max_y_mm', 320.0):.0f}"
                    )
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
                conclusion = f"扫描模式输出{output_count}个规划工作区内世界坐标点，不做2x2限制"
            else:
                conclusion = "扫描模式当前没有规划工作区内可用点"
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
                "扫描模式已直接输出整个矩形画幅内、规划工作区内的世界坐标点"
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
                message = "pointAI process_image timed out while waiting for stable points"
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

            point_coords = self.pre_img(request_mode=request_mode)
            if not self.has_detected_points(point_coords):
                stable_snapshots = []
                latest_point_coords = None
                rospy.logwarn_throttle(2.0, "pointAI等待有效点，继续轮询视觉")
                rate.sleep()
                continue

            latest_point_coords = point_coords
            if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
                return self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)

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
        start_time = time.perf_counter()
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
            self.log_process_image_timing(request_mode, response, time.perf_counter() - start_time)
            return response
                
        except Exception as e:
            rospy.logerr(f"Error in handle_process_image: {str(e)}")
            response = self.build_process_image_response(
                success=False,
                message=f"Error in handle_process_image: {str(e)}",
            )
            self.log_process_image_timing(request_mode, response, time.perf_counter() - start_time)
            return response
    def is_point_in_roi(self, x, y):
        left = min(self.point1[0], self.point2[0])
        right = max(self.point1[0], self.point2[0])
        top = min(self.point1[1], self.point2[1])
        bottom = max(self.point1[1], self.point2[1])
        return left <= x <= right and top <= y <= bottom

    def is_point_in_travel_range(self, calibrated_x, calibrated_y):
        return (
            0 <= calibrated_x <= getattr(self, "travel_range_max_x_mm", 320.0)
            and 0 <= calibrated_y <= getattr(self, "travel_range_max_y_mm", 320.0)
        )

    def is_point_in_display_bind_range(self, calibrated_x, calibrated_y):
        return (
            0 <= calibrated_x <= getattr(self, "display_bind_range_max_x_mm", 360.0)
            and 0 <= calibrated_y <= getattr(self, "display_bind_range_max_y_mm", 360.0)
        )

    def group_points_by_axis(self, centers, axis_index=0, threshold=40):
        if not centers:
            return []

        grouped_points = []
        sorted_centers = sorted(
            centers,
            key=lambda item: (item[2][axis_index], item[2][1 - axis_index], item[0])
        )

        current_group = [sorted_centers[0]]
        current_mean = float(sorted_centers[0][2][axis_index])
        for item in sorted_centers[1:]:
            axis_value = float(item[2][axis_index])
            if abs(axis_value - current_mean) <= threshold:
                current_group.append(item)
                current_mean = sum(point[2][axis_index] for point in current_group) / len(current_group)
            else:
                grouped_points.append(current_group)
                current_group = [item]
                current_mean = axis_value

        if current_group:
            grouped_points.append(current_group)

        return grouped_points

    def match_points_between_rows(self, upper_row, lower_row, column_threshold=45):
        matched_pairs = []
        used_lower_indices = set()

        upper_sorted = sorted(upper_row, key=lambda item: (item[2][1], item[2][0], item[0]))
        lower_sorted = sorted(lower_row, key=lambda item: (item[2][1], item[2][0], item[0]))

        for upper_item in upper_sorted:
            upper_y = upper_item[2][1]
            best_lower_index = None
            best_gap = None

            for lower_index, lower_item in enumerate(lower_sorted):
                if lower_index in used_lower_indices:
                    continue

                gap = abs(upper_y - lower_item[2][1])
                if gap > column_threshold:
                    continue

                if best_gap is None or gap < best_gap:
                    best_gap = gap
                    best_lower_index = lower_index

            if best_lower_index is None:
                continue

            used_lower_indices.add(best_lower_index)
            matched_pairs.append((upper_item, lower_sorted[best_lower_index], best_gap))

        matched_pairs.sort(
            key=lambda pair: (
                min(pair[0][2][1], pair[1][2][1]),
                pair[2],
                pair[0][2][0],
                pair[1][2][0]
            )
        )
        return matched_pairs

    def score_matrix_candidate(self, matrix_points, column_gaps):
        distance_scores = sorted(
            (
                point[2][0] * point[2][0] + point[2][1] * point[2][1],
                point[2][0],
                point[2][1],
                point[0]
            )
            for point in matrix_points
        )
        x_values = sorted(point[2][0] for point in matrix_points)
        y_values = sorted(point[2][1] for point in matrix_points)
        return tuple(distance_scores + [(tuple(x_values), tuple(y_values), tuple(sorted(column_gaps)))])

    def sort_matrix_points(self, matrix_points):
        if len(matrix_points) != 4:
            return sorted(
                matrix_points,
                key=lambda item: (
                    item[2][0] * item[2][0] + item[2][1] * item[2][1],
                    item[2][0],
                    item[2][1],
                    item[0]
                )
            )

        def pixel_xy(point):
            return float(point[1][0]), float(point[1][1])

        def calibrated_xy(point):
            return float(point[2][0]), float(point[2][1])

        # Start numbering from the point nearest the calibrated world origin.
        point1 = min(
            matrix_points,
            key=lambda item: (
                calibrated_xy(item)[0] * calibrated_xy(item)[0] + calibrated_xy(item)[1] * calibrated_xy(item)[1],
                calibrated_xy(item)[0],
                calibrated_xy(item)[1],
                item[0]
            )
        )
        point1_x, point1_y = pixel_xy(point1)
        remaining_points = [point for point in matrix_points if point is not point1]

        point2 = min(
            remaining_points,
            key=lambda item: (
                abs(pixel_xy(item)[1] - point1_y),
                -abs(pixel_xy(item)[0] - point1_x),
                pixel_xy(item)[0],
                item[0]
            )
        )
        remaining_points = [point for point in remaining_points if point is not point2]

        point3 = min(
            remaining_points,
            key=lambda item: (
                abs(pixel_xy(item)[0] - point1_x),
                -abs(pixel_xy(item)[1] - point1_y),
                pixel_xy(item)[1],
                item[0]
            )
        )
        point4 = next(point for point in remaining_points if point is not point3)

        return [point1, point2, point3, point4]

    def get_selected_point_numbers(self, selected_points):
        return {
            source_idx: selected_idx + 1
            for selected_idx, (source_idx, _, _) in enumerate(selected_points)
        }

    def select_output_centers_for_mode(self, request_mode, in_range_centers, selected_centers):
        if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
            return list(in_range_centers)
        if request_mode == PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT:
            return self.sort_matrix_points(in_range_centers)
        return list(selected_centers)

    def select_display_matrix_centers(self, downstream_matrix_centers, in_range_centers, candidate_centers):
        if downstream_matrix_centers:
            return list(downstream_matrix_centers)

        display_range_centers = [
            center_record
            for center_record in candidate_centers
            if self.is_point_in_display_bind_range(center_record[2][0], center_record[2][1])
        ]
        fallback_matrix = self.select_nearest_origin_matrix_points(display_range_centers)
        if fallback_matrix:
            return fallback_matrix
        nearest_in_range_centers = sorted(
            display_range_centers,
            key=lambda item: (
                item[2][0] * item[2][0] + item[2][1] * item[2][1],
                item[2][0],
                item[2][1],
                item[0]
            )
        )[:4]
        return self.sort_matrix_points(nearest_in_range_centers)

    def build_matrix_display_points(self, matrix_centers, mark_travel_out_of_range=False):
        display_point_numbers = self.get_selected_point_numbers(matrix_centers)
        display_points = []

        for source_idx, center, calibrated_world_coord in matrix_centers:
            display_idx = display_point_numbers.get(source_idx, "-")
            pix_coord = [int(center[0]), int(center[1])]
            if (not mark_travel_out_of_range) or self.is_point_in_travel_range(
                calibrated_world_coord[0],
                calibrated_world_coord[1]
            ):
                status = self.get_selected_point_status(display_idx)
                status_detail = ""
            else:
                status = "out_of_range"
                reject_reasons = self.get_travel_range_reject_reasons(
                    calibrated_world_coord[0],
                    calibrated_world_coord[1]
                )
                status_detail = "OUT_" + "+".join(reason[0] for reason in reject_reasons) if reject_reasons else "OUT"

            display_points.append(
                (
                    display_idx,
                    pix_coord,
                    calibrated_world_coord,
                    status,
                    status_detail
                )
            )

        return display_points

    def filter_close_points_by_origin(self, centers, min_distance_mm=40.0):
        if not centers:
            return []

        min_distance_sq = min_distance_mm * min_distance_mm
        kept_points = []
        for candidate in sorted(
            centers,
            key=lambda item: (
                item[2][0] * item[2][0] + item[2][1] * item[2][1],
                item[2][0],
                item[2][1],
                item[0]
            )
        ):
            candidate_x, candidate_y = candidate[2][0], candidate[2][1]
            too_close = False
            for kept in kept_points:
                kept_x, kept_y = kept[2][0], kept[2][1]
                dx = candidate_x - kept_x
                dy = candidate_y - kept_y
                if dx * dx + dy * dy < min_distance_sq:
                    too_close = True
                    break
            if not too_close:
                kept_points.append(candidate)

        return kept_points

    def select_nearest_origin_matrix_points(self, centers, max_points=4, row_threshold=40, column_threshold=45):
        if len(centers) < max_points:
            return []

        rows = self.group_points_by_axis(centers, axis_index=0, threshold=row_threshold)
        if len(rows) < 2:
            return []

        best_points = []
        best_score = None
        for upper_index in range(len(rows) - 1):
            upper_row = rows[upper_index]
            if len(upper_row) < 2:
                continue

            for lower_index in range(upper_index + 1, len(rows)):
                lower_row = rows[lower_index]
                if len(lower_row) < 2:
                    continue

                matched_pairs = self.match_points_between_rows(
                    upper_row,
                    lower_row,
                    column_threshold=column_threshold
                )
                if len(matched_pairs) < 2:
                    continue

                for first_pair_index in range(len(matched_pairs) - 1):
                    for second_pair_index in range(first_pair_index + 1, len(matched_pairs)):
                        first_pair = matched_pairs[first_pair_index]
                        second_pair = matched_pairs[second_pair_index]
                        selected_points = [
                            first_pair[0],
                            first_pair[1],
                            second_pair[0],
                            second_pair[1],
                        ]
                        sorted_points = self.sort_matrix_points(selected_points)
                        candidate_score = self.score_matrix_candidate(
                            sorted_points,
                            [first_pair[2], second_pair[2]]
                        )
                        if best_score is None or candidate_score < best_score:
                            best_score = candidate_score
                            best_points = sorted_points

        return best_points
    
    def pre_img(self, request_mode=PROCESS_IMAGE_MODE_DEFAULT):
        if self.image is None:
            return None  # 返回空列表
        self.result_display_points = []
        self.last_detection_debug = {}
        self.channels = self.cv2.split(self.image)
        # 分别获取X, Y, Z通道
        self.image_raw_world_channels = self.cv2.split(self.image_raw_world)
        # 分别获取X, Y, Z通道
        self.x_channel = (self.image_raw_world_channels[0]).astype(np.int32)
        self.y_channel = (self.image_raw_world_channels[1]).astype(np.int32)
        self.depth_v =(self.image_raw_world_channels[2]).astype(np.int32)
        self.Depth_image_Raw = np.copy((self.channels[2]).astype(np.int32))

        self.Depth_image_Raw[self.y1:self.y2, self.x1:self.x2] = 0
        # self.Depth_image_Raw[self.y3:self.y4, self.x3:self.x4] = 0
        self.Depth_image_Raw_raw = cv2.normalize(self.Depth_image_Raw, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        self.max_depth = int(np.max(self.Depth_image_Raw) - 5)
        self.Depth_image_Range = self.cv2.inRange(self.Depth_image_Raw, 10, self.max_depth )
        self.Depth_image_Raw_uni = cv2.normalize(self.Depth_image_Range, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        _, self.Depth_image_Raw_binary = cv2.threshold( self.Depth_image_Raw_uni, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        self.Depth_image_Raw_binary = cv2.medianBlur(self.Depth_image_Raw_binary, 3) 
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
        target_frame = "cabin_frame" if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY else "gripper_frame"
        for source_idx, center in enumerate(self.centers):
            pix_coord = [int(center[0]), int(center[1])]
            raw_world_coord = [int(center[2][0]), int(center[2][1]), int(center[2][2])]

            if not self.is_point_in_roi(center[0], center[1]):
                roi_reject_count += 1
                continue

            x_value, y_value, z_value = center[2]
            if x_value == 0 or y_value == 0 or z_value == 0:
                zero_world_count += 1
                continue

            calibrated_world_coord = self.apply_spatial_calibration(
                x_value,
                y_value,
                z_value,
                source_idx,
                target_frame=target_frame
            )
            if calibrated_world_coord is None:
                continue
            calibrated_x, calibrated_y, calibrated_z = calibrated_world_coord

            center_record = (source_idx, center, [calibrated_x, calibrated_y, calibrated_z])
            candidate_centers.append(center_record)

        raw_candidate_count = len(candidate_centers)
        candidate_centers = self.filter_close_points_by_origin(candidate_centers)
        duplicate_removed_count = raw_candidate_count - len(candidate_centers)
        in_range_centers = []
        out_of_range_count = 0
        out_of_range_reason_counts = {}
        out_of_range_samples = []
        for center_record in candidate_centers:
            calibrated_x, calibrated_y, _ = center_record[2]
            if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
                point_is_allowed = self.is_point_in_scan_workspace(calibrated_x, calibrated_y)
            else:
                point_is_allowed = self.is_point_in_travel_range(calibrated_x, calibrated_y)

            if point_is_allowed:
                in_range_centers.append(center_record)
            else:
                out_of_range_count += 1
                if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
                    reject_reasons = ["规划工作区外"]
                else:
                    reject_reasons = self.get_travel_range_reject_reasons(calibrated_x, calibrated_y)
                for reason in reject_reasons:
                    out_of_range_reason_counts[reason] = out_of_range_reason_counts.get(reason, 0) + 1
                if len(out_of_range_samples) < 5:
                    source_idx, center, calibrated_world_coord = center_record
                    pix_coord = [int(center[0]), int(center[1])]
                    out_of_range_samples.append(
                        f"idx={source_idx},pix=({pix_coord[0]},{pix_coord[1]}),coord=({calibrated_world_coord[0]:.1f},{calibrated_world_coord[1]:.1f},{calibrated_world_coord[2]:.1f}),原因={'+'.join(reject_reasons) if reject_reasons else '未知'}"
                    )

        if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
            self.sorted_centers = []
        else:
            self.sorted_centers = self.select_nearest_origin_matrix_points(in_range_centers)
        if (
            request_mode not in (PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT, PROCESS_IMAGE_MODE_SCAN_ONLY)
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
        if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:
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
                print("x_value:", self.x_value, "y_value:", self.y_value, "z_value:", self.z_value)
                continue

            raw_z = self.depth_v[y, x]
            # 使用save_image函数判断是否已绑过
            # is_already_bound = self.save_image(x, y, raw_z)
            self.is_shuiguan = False
            # print("x_value:", self.x_value, "y_value:", self.y_value, "z_value:", self.z_value)

            # 创建点坐标对象并添加到数组
            self.PointCoordinates = PointCoords()
            self.PointCoordinates.is_shuiguan = self.is_shuiguan
            self.PointCoordinates.Angle = -45 #float((self.angles[idx] / 2) - 88) - 8
            self.PointCoordinates.idx = idx
            self.PointCoordinates.Pix_coord = [x, y]
            self.PointCoordinates.World_coord = [self.x_value, self.y_value, self.z_value]
            self.PointCoordinates_array_msg.PointCoordinatesArray.append(self.PointCoordinates)

        self.PointCoordinates_array_msg.count = len(self.PointCoordinates_array_msg.PointCoordinatesArray)
        self.coordinate_publisher.publish(self.PointCoordinates_array_msg)
  

    

        # 返回处理结果
        return self.PointCoordinates_array_msg
        
 

if __name__ == '__main__':
    rospy.init_node('pointAINode')
    processor = ImageProcessor()
    rospy.spin()
