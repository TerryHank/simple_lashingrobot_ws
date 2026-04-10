#!/usr/bin/env python3

"""
    索驱并联式钢筋绑扎机器人-点云处理节点
"""
import warnings
warnings.filterwarnings("ignore", message="The value of the smallest subnormal for")
from cv2.ppf_match_3d import Pose3D
import rospy
import json
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
from std_msgs.msg import Int32 ,Float32
from cv2 import ximgproc
import os
# from ultralytics import YOLO
import torch
import cv2.aruco as aruco
import tf  # 新增导入tf模块
import tf2_ros  # 确保已导入
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
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
        self.calibration_offset_x = 272
        self.calibration_offset_y = 95
        self.calibration_offset_z = -770
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
    
        # 初始化TF相关组件 - 移到这里，在订阅器之前
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 初始化图像副本
        self.image_infrared_copy = np.zeros((480, 640), dtype=np.uint8)
        self.image_infrared = None
        self.result_display_points = []
        self.last_detection_debug = {}
        
        rospy.Subscriber('/Scepter/worldCoord/world_coord', Image, self.image_callback)
        rospy.Subscriber('/Scepter/worldCoord/raw_world_coord', Image, self.image_raw_world_callback)
        rospy.Subscriber('/Scepter/color/image_raw', Image, self.image_color_callback)
        rospy.Subscriber('/Scepter/ir/image_raw', Image, self.image_infrared_callback)
        rospy.Subscriber('/Scepter/depth/image_raw', Image, self.image_depth_callback)
        rospy.Subscriber('/Scepter/ir/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/web/fast_image_solve/set_pointAI_offset', Pose, self.calibration_offset_callback) # 接收标定参数设置
        rospy.Subscriber('/web/fast_image_solve/set_height_threshold', Float32, self.fixed_z_value_callback)  # 接收固定z值话题
    
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
        
        # 在初始化最后发布坐标变换
        self.publish_gripper_tf_transform()
        
        self.T_camera_ee = None
        self.camera_matrix = None
        self.dist_coeffs = None   

        # 读取lashing_config.json文件，赋值给calibration_offset_x, calibration_offset_y, calibration_offset_z
        with open(self.cali_offset_file, 'r') as f:
            data = json.load(f)
            self.calibration_offset_x = data['cal_x']
            self.calibration_offset_y = data['cal_y']
            self.calibration_offset_z = data['cal_z']
            self.height_threshold = data['height_threshold']

        print("pointAI node initialized!")
        print("calibration_offset_x={}, calibration_offset_y={}, calibration_offset_z={}, height_threshold={}".format(self.calibration_offset_x, self.calibration_offset_y, self.calibration_offset_z, self.height_threshold))

    def fixed_z_value_callback(self, msg):
        """
        /fixed_z_value 回调: 若值不为0将在计算点坐标时强制替换z
        """
        if msg.data < 20:
            self.height_threshold = msg.data
            # 更新lashing_config.json文件中的height_threshold字段
            with open(self.cali_offset_file, 'w') as f:
                json.dump({'cal_x': self.calibration_offset_x, 'cal_y': self.calibration_offset_y, 'cal_z': self.calibration_offset_z, 'height_threshold': self.height_threshold}, f, indent=4)
            rospy.loginfo(f"设置高度阈值为: {self.height_threshold}")
        else:
            self.fixed_z_value = msg.data
            rospy.loginfo(f"接收到固定下探Z值: {self.fixed_z_value}") 
        
    def calibration_offset_callback(self, msg):
        """
        接收标定参数的回调函数
        :param msg: Pose消息，包含x, y, z标定偏移量
        """
        self.calibration_offset_x = msg.position.x
        self.calibration_offset_y = msg.position.y
        self.calibration_offset_z = msg.position.z
        # 更新lashing_config.json文件
        with open(self.cali_offset_file, 'w') as f:
            json.dump({'cal_x': self.calibration_offset_x, 'cal_y': self.calibration_offset_y, 'cal_z': self.calibration_offset_z, 'height_threshold': self.height_threshold}, f, indent=4)
        rospy.loginfo(f"标定参数已更新: calibration_offset_x={self.calibration_offset_x}, calibration_offset_y={self.calibration_offset_y}, calibration_offset_z={self.calibration_offset_z}")

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
    def publish_aruco_frame_transform(self):
        try:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()  # 使用当前时间更稳妥
            transform.header.frame_id = "aruco_raw_frame"
            transform.child_frame_id = "aruco_frame"
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            # 绕Y轴180°: 轴(0,1,0), 角π -> 四元数 (0,1,0,0)
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 1.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 0.0
            self.tf_broadcaster.sendTransform(transform)
        except Exception as e:
            rospy.logerr(f"Error in publish_aruco_frame_transform: {str(e)}")
    def publish_tf_transform(self):
        try:
            self.stamp = rospy.Time.now()
            # 从文件加载pose_matrix
            pose_matrix_path = self.cali_matrix_file
            if not os.path.exists(pose_matrix_path):
                rospy.logerr(f"Pose matrix file not found: {pose_matrix_path}")
                return
                
            pose_matrix = np.load(pose_matrix_path)
            
            # 检查矩阵形状是否正确
            if pose_matrix.shape != (4, 4):
                rospy.logerr(f"Invalid pose matrix shape: {pose_matrix.shape}, expected (4, 4)")
                return
                
            transform = TransformStamped()
            transform.header.stamp = self.stamp
            transform.header.frame_id = "Scepter_transformedColor_frame"
            transform.child_frame_id = "aruco_raw_frame"

            # 设置平移
            transform.transform.translation.x = pose_matrix[0, 3] 
            transform.transform.translation.y = pose_matrix[1, 3]
            transform.transform.translation.z = pose_matrix[2, 3]
            
            # 转换为四元数
            q = tf.transformations.quaternion_from_matrix(pose_matrix)
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(transform)
            self.publish_aruco_frame_transform()
            
        except Exception as e:
            rospy.logerr(f"Error in publish_tf_transform: {str(e)}")
            rospy.logdebug("Detailed error:", exc_info=True)


    def publish_gripper_tf_transform(self):
        try:
            # 先发布上游链
            self.publish_tf_transform()

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "aruco_frame"
            t.child_frame_id = "gripper_frame"

            # 仅平移(米)
            t.transform.translation.x = self.calibration_offset_x / 1000.0
            t.transform.translation.y = self.calibration_offset_y / 1000.0
            t.transform.translation.z = self.calibration_offset_z / 1000.0

            # 保持与 aruco_frame 完全一致的朝向（单位四元数）
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            rospy.logerr(f"Error in publish_gripper_tf_transform: {str(e)}")

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
        
    def transform_to_gripper_frame(self, x, y, z, idx):
        try:
            # # 检查gripper_frame是否存在，如果不存在先发布它
            # if not self.tf_buffer.can_transform("gripper_frame", "Scepter_frame", rospy.Time(0), rospy.Duration(1.0)):
            #     rospy.logwarn_throttle(5, "gripper_frame不存在，使用原始坐标")
            #     return x, y, z
            
            # 获取Scepter_frame到gripper_frame的变换
            transform = self.tf_buffer.lookup_transform(
                "gripper_frame",  # 目标坐标系
                "Scepter_depth_frame",  # 源坐标系
                rospy.Time(0),    # 获取最新可用变换
            )
            
            # 创建变换矩阵
            T = quaternion_matrix([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            T[:3, 3] = [transform.transform.translation.x * 1000,  # 米转毫米
                        transform.transform.translation.y * 1000,
                        transform.transform.translation.z * 1000]
            
            # 应用变换（注意单位转换）
            point_camera = np.array([x, y, z, 1])  # 原始毫米坐标
            point_gripper = np.dot(T, point_camera)
            
            # 发布TF帧
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = "gripper_frame"
            transform_stamped.child_frame_id = f"P{idx}"  # 使用idx作为区分
            
            transform_stamped.transform.translation.x = point_gripper[0] / 1000.0  # 毫米转米
            transform_stamped.transform.translation.y = point_gripper[1] / 1000.0
            transform_stamped.transform.translation.z = point_gripper[2] / 1000.0
            
            transform_stamped.transform.rotation.x = 0.0  # 无旋转
            transform_stamped.transform.rotation.y = 0.0  # 无旋转
            transform_stamped.transform.rotation.z = 0.0  # 无旋转
            transform_stamped.transform.rotation.w = 1.0  # 无旋转
            
            self.tf_broadcaster.sendTransform(transform_stamped)
            
            return int(round(point_gripper[0])), int(round(point_gripper[1])), int(round(point_gripper[2]))
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"坐标转换失败: {str(e)}")
 
        except Exception as e:
            rospy.logerr_throttle(5, f"坐标转换未知错误: {str(e)}")
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
    
        self.publish_gripper_tf_transform()
        # self.pre_img()
        self.channels = self.cv2.split(self.image)
        self.Depth_image_Raw =(self.channels[2]).astype(np.int32)
        # self.mark_sign_points()
        # 确保image_infrared_copy存在
        if not hasattr(self, 'image_infrared_copy') or self.image_infrared_copy is None:
            self.image_infrared_copy = np.zeros((480, 640), dtype=np.uint8)

        result_image = np.array(self.image_infrared_copy, copy=True)
        cv2.rectangle(result_image, self.point1, self.point2, 255, 2)

        occupied_label_bboxes = []
        sorted_display_points = sorted(self.result_display_points, key=lambda item: (item[1][1], item[1][0]))
        for display_idx, pix_coord, world_coord, status in sorted_display_points:
            status = "selected" if status == "selected" else "unselected"
            color = 255 if status == "selected" else 0
            text_color = 255 if status == "selected" else 0
            bg_color = 0 if status == "selected" else 255
            status_text = "SEL" if status == "selected" else "NO"

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
            cv2.circle(result_image, (int(pix_coord[0]), int(pix_coord[1])), 3, color, -1)

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

    def handle_process_image(self, req):
        try:
            if self.image is None:
                print("Input Image is None")
                return ProcessImageResponse(count=0, PointCoordinatesArray=[])

            # 调用pre_img处理图像
            point_coords = self.pre_img()
            
            # 确保point_coords不为None且result_finally存在
            if point_coords is not None and point_coords.count > 0:
                # 分别传入 count 和 PointCoordinatesArray
                return ProcessImageResponse(count=point_coords.count, PointCoordinatesArray=point_coords.PointCoordinatesArray)
            else:
                # 如果没有检测到有效点，返回空响应
                return ProcessImageResponse(count=0, PointCoordinatesArray=[])
                
        except Exception as e:
            rospy.logerr(f"Error in handle_process_image: {str(e)}")
            # 发生错误时返回空响应
            return ProcessImageResponse(count=0, PointCoordinatesArray=[])
    def snake_sort(self, centers, row_threshold=50):
        """
        对点进行蛇形排序，确保从左上角开始
        
        参数:
        centers: 包含点坐标的列表，每个点格式为 [x, y, [world_x, world_y, world_z]]
        row_threshold: 判定为同一行的y坐标差值阈值
        
        返回:
        排序后的点列表
        """
        if not centers:
            return []
        
        # 首先按y坐标排序，确保从上到下
        centers = sorted(centers, key=lambda p: p[1])
        
        # 将点按y坐标分组成不同的行
        rows = []
        current_row = [centers[0]]
        last_y = centers[0][1]
        
        for point in centers[1:]:
            if abs(point[1] - last_y) <= row_threshold:
                # 如果y坐标差值在阈值内，认为是同一行
                current_row.append(point)
            else:
                # 开始新的一行
                rows.append(current_row)
                current_row = [point]
                last_y = point[1]
        
        # 添加最后一行
        if current_row:
            rows.append(current_row)
        
        # 对每一行进行排序，偶数行反向
        sorted_points = []
        for i, row in enumerate(rows):
            # 按x坐标排序，确保每行都是从左到右
            sorted_row = sorted(row, key=lambda p: p[0])
            # 偶数行反向（第二行开始）
            if i % 2 == 1:
                sorted_row.reverse()
            sorted_points.extend(sorted_row)
        
        return sorted_points

    def is_point_in_roi(self, x, y):
        left = min(self.point1[0], self.point2[0])
        right = max(self.point1[0], self.point2[0])
        top = min(self.point1[1], self.point2[1])
        bottom = max(self.point1[1], self.point2[1])
        return left <= x <= right and top <= y <= bottom

    def is_point_in_travel_range(self, calibrated_x, calibrated_y):
        return 0 <= calibrated_x <= 320 and 0 <= calibrated_y <= 360

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
        return sorted(matrix_points, key=lambda item: (item[2][0], item[2][1], item[0]))

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
    
    def pre_img(self):
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
        self.max_depth = int(np.max(self.Depth_image_Raw) - 12)
        self.Depth_image_Range = self.cv2.inRange(self.Depth_image_Raw, 10, self.max_depth )
        self.Depth_image_Raw_uni = cv2.normalize(self.Depth_image_Range, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        _, self.Depth_image_Raw_binary = cv2.threshold( self.Depth_image_Raw_uni, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        self.Depth_image_Raw_binary = cv2.medianBlur(self.Depth_image_Raw_binary, 5) 
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
                dy = y2 - y1
                dx = x2 - x1
                if dx == 0:  # 垂直线
                    angle = 90.0
                else:
                    angle = np.degrees(np.arctan(dy/dx))
                
                # 计算与最近轴线的角度差
                angle_diff = min(abs(angle % 90), 90 - (angle % 90))
                
                if angle_diff <= 60:  # 保留与轴线偏差≤15度的直线
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
            }
            rospy.logwarn(
                "pointAI debug: lines=%d intersections=%d centers=0 world_fallback=0 roi_reject=0 zero_world=0 candidate_points=0 in_range_candidates=0 selected_points=0 out_of_range_points=0",
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
        in_range_centers = []
        roi_reject_count = 0
        zero_world_count = 0
        out_of_range_count = 0
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

            calibrated_x = x_value + self.calibration_offset_x
            calibrated_y = y_value + self.calibration_offset_y
            calibrated_z = z_value + self.calibration_offset_z

            if self.fixed_z_value != 0:
                calibrated_z = float(self.fixed_z_value)

            center_record = (source_idx, center, [calibrated_x, calibrated_y, calibrated_z])
            candidate_centers.append(center_record)

            if self.is_point_in_travel_range(calibrated_x, calibrated_y):
                in_range_centers.append(center_record)
            else:
                out_of_range_count += 1

        self.sorted_centers = self.select_nearest_origin_matrix_points(in_range_centers)
        if in_range_centers and not self.sorted_centers:
            rospy.logwarn(
                "pointAI debug: %d executable points detected in range but no 2x2 matrix near origin could be formed",
                len(in_range_centers)
            )
        selected_source_indices = {source_idx for source_idx, _, _ in self.sorted_centers}
        candidate_point_count = len(candidate_centers)
        in_range_candidate_count = len(in_range_centers)
        selected_count = len(self.sorted_centers)
        out_of_range_point_count = out_of_range_count

        self.result_display_points = []
        for display_idx, (source_idx, center, calibrated_world_coord) in enumerate(
            sorted(candidate_centers, key=lambda item: (item[1][1], item[1][0], item[0]))
        ):
            pix_coord = [int(center[0]), int(center[1])]
            status = "selected" if source_idx in selected_source_indices else "unselected"
            self.result_display_points.append(
                (
                    display_idx,
                    pix_coord,
                    calibrated_world_coord,
                    status
                )
            )

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
        }
        rospy.loginfo(
            "pointAI debug: lines=%d intersections=%d centers=%d world_fallback=%d roi_reject=%d zero_world=%d candidate_points=%d in_range_candidates=%d selected_points=%d out_of_range_points=%d",
            self.last_detection_debug["lines"],
            self.last_detection_debug["intersections"],
            self.last_detection_debug["centers"],
            self.last_detection_debug["world_fallback"],
            self.last_detection_debug["roi_reject"],
            self.last_detection_debug["zero_world"],
            self.last_detection_debug["candidate_points"],
            self.last_detection_debug["in_range_candidates"],
            self.last_detection_debug["selected_points"],
            self.last_detection_debug["out_of_range_points"]
        )

        self.PointCoordinates_array_msg = PointsArray()
        self.PointCoordinates_array_msg.PointCoordinatesArray = []
        # 在pre_img方法中的遍历部分进行修改
        marker_point_idx = None  # 标记点的索引
        skip_next = False       # 是否跳过下一个点
        for idx, (_, center, calibrated_world_coord) in enumerate(self.sorted_centers):
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
