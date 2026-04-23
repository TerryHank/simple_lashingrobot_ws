#!/usr/bin/env python3

"""
    索驱并联式钢筋绑扎机器人-点云处理节点
"""
import warnings
warnings.filterwarnings("ignore", message="The value of the smallest subnormal for")
from cv2.ppf_match_3d import Pose3D
import diagnostic_updater
import rospy
import json
import sys
import yaml
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage , CameraInfo # 添加CompressedImage
import numpy as np
from sklearn.cluster import DBSCAN
from tie_robot_msgs.srv import (
    linear_module_move,
    linear_module_moveRequest,
    linear_module_moveResponse,
    SingleMove,
    SingleMoveRequest,
)
from tie_robot_msgs.msg import PointsArray, PointCoords
import math
from tie_robot_msgs.msg import motion
from tie_robot_msgs.srv import ProcessImage, ProcessImageResponse, PlaneDetection, PlaneDetectionResponse
from std_srvs.srv import Trigger, TriggerResponse
import time
from std_msgs.msg import Int32 ,Float32, Bool, Float32MultiArray
from cv2 import ximgproc
import os
# from ultralytics import YOLO
import torch
import tf2_ros  # 确保已导入
import numpy as np
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from diagnostic_msgs.msg import DiagnosticStatus

PERCEPTION_PACKAGE_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "src")
)
if os.path.isdir(PERCEPTION_PACKAGE_DIR) and PERCEPTION_PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PERCEPTION_PACKAGE_DIR)

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

PROCESS_IMAGE_MODE_DEFAULT = 0
PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT = 1
PROCESS_IMAGE_MODE_BIND_CHECK = 2
PROCESS_IMAGE_MODE_SCAN_ONLY = 3  # pseudo_slam scan mode
PROCESS_IMAGE_MODE_EXECUTION_REFINE = 4  # live_visual execution refine mode
VISUAL_DIAGNOSTIC_HARDWARE_ID = "tie_robot/visual_algorithm"
VISUAL_INPUT_STALE_SEC = 5.0
VISUAL_ERROR_HOLD_SEC = 8.0


from tie_robot_perception.pointai.processor import bind_image_processor_methods

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv2=cv2
        self.image = None
        self.vison_image  = None
        self.depth_image = None
        self.tof_intrin = None
        self.visual_input_timestamps = {}
        self.visual_last_error_message = ""
        self.visual_last_error_time = 0.0
        self.visual_last_process_request_time = 0.0
        self.visual_last_process_success_time = 0.0
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
        self.cali_matrix_file = "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/data/pose_matrix.npy"
        self.cali_offset_file = "/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/data/lashing_config.json"
        self.path_points_file = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "tie_robot_process",
                "data",
                "path_points.json",
            )
        )
        self.manual_workspace_quad_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "data", "manual_workspace_quad.json")
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
        self.current_result_request_mode = PROCESS_IMAGE_MODE_DEFAULT
        self.last_detection_debug = {}
        self.show_candidate_labels = rospy.get_param("~show_candidate_labels", False)
        self.jump_bind_enabled = False
        self.axis_alignment_tolerance_deg = float(rospy.get_param("~axis_alignment_tolerance_deg", 15.0))
        self.process_request_rate_hz = max(1.0, float(rospy.get_param("~process_request_rate_hz", 10.0)))
        self.process_wait_timeout_sec = float(rospy.get_param("~process_wait_timeout_sec", 0.0))
        self.stable_frame_count = max(1, int(rospy.get_param("~stable_frame_count", 3)))
        self.stable_z_tolerance_mm = float(rospy.get_param("~stable_z_tolerance_mm", 5.0))
        self.bind_check_max_height_mm = float(rospy.get_param("~bind_check_max_height_mm", 95.0))
        self.travel_range_max_x_mm = float(rospy.get_param("~travel_range_max_x_mm", 360.0))
        self.travel_range_max_y_mm = float(rospy.get_param("~travel_range_max_y_mm", 320.0))
        self.travel_range_max_z_mm = float(rospy.get_param("~travel_range_max_z_mm", 140.0))
        self.matrix_selection_max_x_mm = float(rospy.get_param("~matrix_selection_max_x_mm", 500.0))
        self.matrix_selection_max_y_mm = float(rospy.get_param("~matrix_selection_max_y_mm", 500.0))
        self.display_bind_range_max_x_mm = float(rospy.get_param("~display_bind_range_max_x_mm", 500.0))
        self.display_bind_range_max_y_mm = float(rospy.get_param("~display_bind_range_max_y_mm", 360.0))
        self.binary_small_blob_min_area_px = int(rospy.get_param("~binary_small_blob_min_area_px", 20))
        self.world_image_seq = 0
        
        rospy.Subscriber('/Scepter/worldCoord/world_coord', Image, self.image_callback)
        rospy.Subscriber('/Scepter/worldCoord/raw_world_coord', Image, self.image_raw_world_callback)
        rospy.Subscriber('/Scepter/color/image_raw', Image, self.image_color_callback)
        rospy.Subscriber('/Scepter/ir/image_raw', Image, self.image_infrared_callback)
        rospy.Subscriber('/Scepter/depth/image_raw', Image, self.image_depth_callback)
        rospy.Subscriber('/Scepter/ir/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/web/pointAI/set_offset', Pose, self.calibration_offset_callback) # 兼容旧前端，空间标定现仅由TF提供
        rospy.Subscriber('/web/pointAI/set_workspace_quad', Float32MultiArray, self.manual_workspace_quad_callback)
        rospy.Subscriber('/web/pointAI/run_workspace_s2', Bool, self.manual_workspace_s2_callback)
        rospy.Subscriber('/web/pointAI/move_to_workspace_center_scan_pose', Bool, self.workspace_center_scan_pose_callback)
        rospy.Subscriber('/web/pointAI/set_height_threshold', Float32, self.fixed_z_value_callback)  # 接收固定z值话题
        rospy.Subscriber('/web/moduan/send_odd_points', Bool, self.jump_bind_callback)
    
        # self.detect_and_save_pose_srv = rospy.Service('/web/pointAI/detect_and_save_pose', Trigger, self.detect_and_save_pose_service)
        self.service = rospy.Service('/pointAI/process_image', ProcessImage, self.handle_process_image)
        # 新增三个Image格式发布器
        self.cropped_ir_image_pub = rospy.Publisher('/pointAI/cropped_ir_image', Image, queue_size=10)
        self.cropped_color_image_pub = rospy.Publisher('/pointAI/cropped_color_image', Image, queue_size=10)
        self.cropped_depth_image_pub = rospy.Publisher('/pointAI/cropped_depth_image', Image, queue_size=10)
        self.depth_binary_image_pub = rospy.Publisher('/pointAI/depth_binary_image', Image, queue_size=10)
        self.line_image_pub = rospy.Publisher('/pointAI/line_image', Image, queue_size=10)
        self.image_pub = rospy.Publisher('/pointAI/result_image', CompressedImage, queue_size=10)
        self.result_image_raw_pub = rospy.Publisher('/pointAI/result_image_raw', Image, queue_size=10)
        self.manual_workspace_s2_result_raw_pub = rospy.Publisher(
            '/pointAI/manual_workspace_s2_result_raw',
            Image,
            queue_size=1,
            latch=True,
        )
        self.manual_workspace_quad_pixels_pub = rospy.Publisher(
            '/pointAI/manual_workspace_quad_pixels',
            Float32MultiArray,
            queue_size=1,
            latch=True,
        )
        # rospy.Subscriber('/cabin/lashing_request', motion, self.printsomething) 
       
        self.plane_z = None
        self.coordinate_publisher = rospy.Publisher('/coordinate_point', PointsArray, queue_size=10)
        self.manual_workspace_s2_points_pub = rospy.Publisher(
            '/pointAI/manual_workspace_s2_points',
            PointsArray,
            queue_size=1,
            latch=True,
        )

        # device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # 加载 YOLO 模型并指定设备
        # self.yolov11 = YOLO("/home/car/lashingrobots/src/tie_robot_perception/scripts/mask2best.pt")
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
        self.publish_current_manual_workspace_quad_pixels()
        self.visual_diagnostic_updater = diagnostic_updater.Updater()
        self.visual_diagnostic_updater.setHardwareID(VISUAL_DIAGNOSTIC_HARDWARE_ID)
        self.visual_diagnostic_updater.add("视觉算法", self.produce_visual_algorithm_diagnostics)
        self.visual_diagnostic_timer = rospy.Timer(rospy.Duration(1.0), self.publish_visual_diagnostics)
        self.force_visual_diagnostics()

        print("pointAI node initialized!")
        print(
            "spatial_calibration_source=tf(Scepter_depth_frame->gripper_frame), "
            "tf_config={}, height_threshold={}".format(
                self.gripper_tf_config_file,
                self.height_threshold
            )
        )

    def mark_visual_input(self, input_key):
        self.visual_input_timestamps[input_key] = time.time()
        self.force_visual_diagnostics()

    def mark_visual_process_request(self):
        self.visual_last_process_request_time = time.time()
        self.force_visual_diagnostics()

    def mark_visual_process_result(self, success, message=""):
        now = time.time()
        self.visual_last_process_request_time = now
        if success:
            self.visual_last_process_success_time = now
            self.visual_last_error_message = ""
            self.visual_last_error_time = 0.0
        else:
            self.visual_last_error_message = message or "视觉算法处理失败"
            self.visual_last_error_time = now
        self.force_visual_diagnostics()

    def mark_visual_error(self, message):
        self.visual_last_error_message = str(message)
        self.visual_last_error_time = time.time()
        self.force_visual_diagnostics()

    def force_visual_diagnostics(self):
        if getattr(self, "visual_diagnostic_updater", None) is not None:
            self.visual_diagnostic_updater.force_update()

    def publish_visual_diagnostics(self, _event):
        if getattr(self, "visual_diagnostic_updater", None) is not None:
            self.visual_diagnostic_updater.update()

    def produce_visual_algorithm_diagnostics(self, stat):
        now = time.time()
        stat.hardware_id = VISUAL_DIAGNOSTIC_HARDWARE_ID
        required_inputs = [
            ("红外图像", "infrared"),
            ("世界点云", "world_coord"),
            ("原始世界点云", "raw_world_coord"),
            ("相机内参", "camera_info"),
        ]
        missing_inputs = []
        stale_inputs = []

        for label, input_key in required_inputs:
            timestamp = self.visual_input_timestamps.get(input_key)
            if not timestamp:
                missing_inputs.append(label)
                stat.add(f"{input_key}_age_sec", "never")
                continue
            age_sec = max(0.0, now - timestamp)
            stat.add(f"{input_key}_age_sec", f"{age_sec:.2f}")
            if age_sec > VISUAL_INPUT_STALE_SEC:
                stale_inputs.append(f"{label}{age_sec:.1f}s")

        if self.visual_last_process_request_time > 0.0:
            stat.add("process_request_age_sec", f"{max(0.0, now - self.visual_last_process_request_time):.2f}")
        else:
            stat.add("process_request_age_sec", "never")

        if self.visual_last_process_success_time > 0.0:
            stat.add("process_success_age_sec", f"{max(0.0, now - self.visual_last_process_success_time):.2f}")
        else:
            stat.add("process_success_age_sec", "never")

        recent_error = (
            bool(self.visual_last_error_message)
            and self.visual_last_error_time > 0.0
            and (now - self.visual_last_error_time) <= VISUAL_ERROR_HOLD_SEC
        )

        if recent_error:
            stat.summary(DiagnosticStatus.ERROR, "视觉算法异常")
            stat.add("transport_state", "algorithm_error")
            stat.add("failure_detail", self.visual_last_error_message)
        elif missing_inputs:
            stat.summary(DiagnosticStatus.WARN, "视觉算法等待输入")
            stat.add("transport_state", "waiting_inputs")
            stat.add("failure_detail", "缺少输入: " + "、".join(missing_inputs))
        elif stale_inputs:
            stat.summary(DiagnosticStatus.WARN, "视觉算法输入超时")
            stat.add("transport_state", "stale_inputs")
            stat.add("failure_detail", "输入超时: " + "、".join(stale_inputs))
        else:
            stat.summary(DiagnosticStatus.OK, "视觉算法运行中")
            stat.add("transport_state", "running")
            stat.add("failure_detail", "")


bind_image_processor_methods(ImageProcessor)

if __name__ == '__main__':
    rospy.init_node('pointAINode')
    processor = ImageProcessor()
    rospy.spin()
