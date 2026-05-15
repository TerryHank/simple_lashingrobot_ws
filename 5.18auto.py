#!/usr/bin/env python3
import warnings
warnings.filterwarnings("ignore", message="The value of the smallest subnormal for")
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage , CameraInfo # 添加CompressedImage
import numpy as np
from sklearn.cluster import DBSCAN
from chassis_ctrl.srv import linear_module_move, linear_module_moveRequest, linear_module_moveResponse
from new_camera.msg import PointsArray,PointCoords
import math
from chassis_ctrl.msg import motion
from new_camera.srv import ProcessImage, ProcessImageResponse ,PlaneDetection, PlaneDetectionResponse
from std_srvs.srv import Trigger, TriggerResponse
import time
from std_msgs.msg import Int32 ,Float32
from cv2 import ximgproc
import os
from ultralytics import YOLO
import torch
import cv2.aruco as aruco
import tf  # 新增导入tf模块
import tf2_ros  # 确保已导入
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix
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
        self.threshold = 50
        self.minLineLength = 60
        self.maxLineGap = 350
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
        self.offset_x, self.offset_y = 0,2
        # self.Calibration_offset_x,self.Calibration_offset_y,self.Calibration_offset_z = -77,85,735
        self.Calibration_offset_x,self.Calibration_offset_y,self.Calibration_offset_z = 202,98,745
        self.multiple = 0.996
        self.x1, self.y1 = 0, 0  # 区域1左上角坐标
        self.x2, self.y2 = 640, 140  # 区域1右下角坐标
        self.x3, self.y3 = 520, 140 # 区域2左上角坐标
        self.x4, self.y4 = 640, 480  # 区域2右下角坐标
        self.start_time = time.time()
        self.roi_bottom_left_x = 205
        self.roi_bottom_left_y = 358
        self.roi_top_right_x = 440
        self.roi_top_right_y = 120
        self.filter_points = True
        self.half_size = 30
        self.max_depth_offset = 90  # 新增：最大深度偏移量
        self.non_shuiguan_count = 0
        self.shuiguan_count = 0
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # 加载 YOLO 模型并指定设备
        self.yolov11 = YOLO("/home/car/lashingrobots/src/new_camera/scripts/best.pt")
        self.yolov11.to(device)
        print(f"YOLO model loaded on {device}")
        rospy.Subscriber('/Scepter/worldCoord/world_coord', Image, self.image_callback)
        rospy.Subscriber('/Scepter/color/image_raw', Image, self.image_color_callback)
        rospy.Subscriber('/Scepter/ir/image_raw', Image, self.image_infrared_callback)
        rospy.Subscriber('/max_depth_offset', Int32, self.max_depth_offset_callback)  # 新增：订阅最大深度偏移量话题
        rospy.Subscriber('/Scepter/ir/camera_info', CameraInfo, self.camera_info_callback)
        self.displacement_x=200
        self.displacement_y=0
        self.displacement_z=130
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # 新增监听器
        self.detect_and_save_pose_srv = rospy.Service('detect_and_save_pose', Trigger, self.detect_and_save_pose_service)
        self.image_pub = rospy.Publisher('/cv/show', CompressedImage, queue_size=10)  # 修改发布器类型为CompressedImage
        # rospy.Subscriber('/cabin/lashing_request', motion, self.printsomething) 
        self.service = rospy.Service('process_image', ProcessImage, self.handle_process_image)
        self.plane_z = None
        self.coordinate_publisher = rospy.Publisher('/coordinate_point', PointsArray, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pose_matrix = None  # 添加pose_matrix初始化
        # # 发布每个中心点的坐标和对应的通道值
        self.frame_count = 0
        self.fps=0
        self.start_time = time.time()
        # 创建一个空的图像作为初始值
        self.image_infrared_copy = np.zeros((480, 640), dtype=np.uint8)  # 根据你的实际图像尺寸调整
        self.image_infrared = None
        # 添加矩形框的坐标点作为类属性
        # self.point1 = (375, 205)  # 点1坐标
        # self.point2 = (580, 400)  # 点2坐标
        self.point1 = (0, 0)  # 点1坐标
        self.point2 = (640, 480)  # 点2坐标

        self.T_camera_ee = None
        self.camera_matrix = None
        self.dist_coeffs = None
       
    def get_gripper_relative_translation(self):
        """
        获取gripper_frame相对于Scepter_transformedColor_frame的平移
        """
        try:
            # 监听变换（目标坐标系: Scepter_transformedColor_frame，源坐标系: gripper_frame）
            transform = self.tf_buffer.lookup_transform(
                "Scepter_frame",  # 父坐标系（假设是用户的Scepter_frame）
                "gripper_frame",                   # 子坐标系（机械臂末端）
                rospy.Time(0),                     # 获取最新的变换
                rospy.Duration(1.0)                # 等待变换的超时时间
            )
            
            # 提取平移部分（单位：米）
            translation = {
                "x": transform.transform.translation.x,
                "y": transform.transform.translation.y,
                "z": transform.transform.translation.z
            }
            return translation
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"获取变换失败: {str(e)}")
            return None
    def publish_gripper_tf_transform(self):
        try:
            # 创建一个新的TransformStamped消息
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "aruco_frame"  # 父坐标系是aruco_frame
            transform.child_frame_id = "gripper_frame"  # 子坐标系是机械臂末端虎口
            
            # 设置平移 - 虎口相对于aruco_frame的位置(10, 0, 5)
            transform.transform.translation.x = -0.005
            transform.transform.translation.y = 0.021
            transform.transform.translation.z = -0.415
            
            # 设置旋转 - 沿y轴旋转180度
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = math.sin(math.pi/2)  # sin(180°/2) = sin(π/2) = 1.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = math.cos(math.pi/2)  # cos(180°/2) = cos(π/2) = 0.0
            
            # 发布变换
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            rospy.logerr(f"Error in publish_gripper_tf_transform: {str(e)}")

    def publish_tf_transform(self):
        try:
            # 从文件加载pose_matrix
            pose_matrix_path = "/home/car/lashingrobots/pose_matrix.npy"
            if not os.path.exists(pose_matrix_path):
                rospy.logerr(f"Pose matrix file not found: {pose_matrix_path}")
                return
                
            pose_matrix = np.load(pose_matrix_path)
            
            # 检查矩阵形状是否正确
            if pose_matrix.shape != (4, 4):
                rospy.logerr(f"Invalid pose matrix shape: {pose_matrix.shape}, expected (4, 4)")
                return
                
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "Scepter_transformedColor_frame"
            transform.child_frame_id = "aruco_frame"

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
            
            
        except Exception as e:
            rospy.logerr(f"Error in publish_tf_transform: {str(e)}")
            rospy.logdebug("Detailed error:", exc_info=True)


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

    
    def draw_text_with_background(self,image, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=0.23, text_color=(255, 255, 255), bg_color=(0, 0, 0), thickness=1):
        # 获取文本的尺寸
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

        # 计算背景矩形的左上角和右下角坐标
        x, y = position
        top_left = (x, y - text_height - baseline)
        bottom_right = (x + text_width, y + baseline)
        # 绘制背景矩形
        cv2.rectangle(image, top_left, bottom_right, bg_color, cv2.FILLED)
        # 绘制文本
        cv2.putText(image, text, (x, y), font, font_scale, text_color, thickness, cv2.LINE_AA)

        


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

    def mouse_callback(self,event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            world_point = param[y, x]  # param 是传递的图像
            # world_point = [int((-world_point[1]+self.Calibration_offset_x)/self.multiple),int((-world_point[0]+self.Calibration_offset_y)/self.multiple),int(world_point[2]-self.Calibration_offset_z)]
            print(f"({x}, {y}),World:{world_point})")
    def image_color_callback(self, msg):
        # 缓存最新的图像
        self.image_color = self.bridge.imgmsg_to_cv2(msg)
        # # 顺时针旋转90度
        # self.image_infrared = cv2.rotate(self.image_infrared, cv2.ROTATE_90_CLOCKWISE)
        
        # 确保图像是可写的
        self.image_color_copy = np.array(self.image_color, copy=True)
    def image_infrared_callback(self, msg):
        # 缓存最新的图像
        self.image_infrared = self.bridge.imgmsg_to_cv2(msg)
        # # 顺时针旋转90度
        # self.image_infrared = cv2.rotate(self.image_infrared, cv2.ROTATE_90_CLOCKWISE)
        
        # 确保图像是可写的
        self.image_infrared_copy = np.array(self.image_infrared, copy=True)
    def get_gripper_transform(self):
        try:
            # 获取gripper_frame到Scepter_frame的变换
            transform = self.tf_buffer.lookup_transform(
                "gripper_frame", 
                "Scepter_frame",
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return translation, rotation
        except Exception as e:
            rospy.logerr(f"获取坐标系变换失败: {str(e)}")
            return None, None
        
    def transform_to_gripper_frame(self, x, y, z, idx):
        try:
            # 获取Scepter_frame到gripper_frame的变换
            transform = self.tf_buffer.lookup_transform(
                "gripper_frame",  # 目标坐标系
                "Scepter_frame",  # 源坐标系
                rospy.Time(0),    # 获取最新可用变换
                rospy.Duration(1.0)
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
            transform_stamped.child_frame_id = f"point_{idx}"  # 使用idx作为区分
            
            transform_stamped.transform.translation.x = point_gripper[0] / 1000.0  # 毫米转米
            transform_stamped.transform.translation.y = point_gripper[1] / 1000.0
            transform_stamped.transform.translation.z = point_gripper[2] / 1000.0
            
            transform_stamped.transform.rotation.w = 1.0  # 无旋转
            
            self.tf_broadcaster.sendTransform(transform_stamped)
            
            return int(round(point_gripper[0])), int(round(point_gripper[1])), int(round(point_gripper[2]))
            
        except Exception as e:
            rospy.logerr(f"坐标转换失败: {str(e)}")
            return x, y, z  

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)
        self.vison_image = np.array(self.image, copy=True)
        normalized = cv2.normalize(self.vison_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        self.vison_image = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    
        # self.pre_img()
    

       

        display_image = np.array(self.image_infrared_copy, copy=True)
  
        if hasattr(self, 'PointCoordinates_array_msg'):
            for point in self.PointCoordinates_array_msg.PointCoordinatesArray:
                idx = point.idx
                world_coord = point.World_coord
                is_shuiguan = point.is_shuiguan
                pix_coord = point.Pix_coord
                angle = point.Angle  # 获取角度值
                
                # 绘制文本信息
                if is_shuiguan:
                    is_shuiguan = "Yes"
                else:
                    is_shuiguan = "No"
                text = f"{idx}, {world_coord}, {is_shuiguan}"
                self.draw_text_with_background( display_image, text, (pix_coord[0] - 10, pix_coord[1] - 10))
                
                # 绘制点
                cv2.circle( display_image, (int(pix_coord[0]), int(pix_coord[1])), 2, (255, 255, 255), -1)
                
        # if hasattr(self, 'Range_Depth_image'):
        #     cv2.imshow("Range_Depth_image", self.Range_Depth_image) 
        # if hasattr(self, 'image_color_copy'):
        #     cv2.imshow("image_color_copy",  self.image_color_copy )
        #     cv2.setMouseCallback("image_color_copy", self.mouse_callback, self.depth_image )
        # if hasattr(self, 'Depth_image_Raw_uni'):
        #     self.cv2.imshow("Depth_image_Raw_uni", self.Depth_image_Raw_uni)
        # if hasattr(self, 'Depth_image_Raw_binary'):
        #     self.cv2.imshow("Depth_image_Raw_binary", self.Depth_image_Raw_binary) 
        # if hasattr(self, 'closing'):
        #     self.cv2.imshow("closing", self.closing)
        # if hasattr(self, 'line_image'):
        #     self.cv2.imshow("line_image", self.line_image)
        # self.cv2.imshow("display_image",display_image)
        compress_msg = CompressedImage()
        compress_msg.header.stamp = rospy.Time.now()
        compress_msg.format = "png"
        _, jpeg_data = cv2.imencode('.png', display_image)
        compress_msg.data = jpeg_data.tobytes()
        self.image_pub.publish(compress_msg)  # 发布压缩图像
        self.publish_tf_transform()
        self.publish_gripper_tf_transform()
        cv2.waitKey(1) & 0xFF


    def max_depth_offset_callback(self, msg):
        self.max_depth_offset = msg.data  # 更新最大深度偏移量
    
    def create_rotation_matrix(self,theta):
        """
        根据旋转角度 theta 创建绕 Z 轴的旋转矩阵
        :param theta: 旋转角度，单位：弧度
        :return: 3x3 旋转矩阵
        """
        return np.array([
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta), math.cos(theta), 0],
            [0, 0, 1]
        ])
    
    def transform_to_end_effector(self, x_obj,y_obj,z_obj,theta_obj):
        """
        ROS服务回调：将物体在相机坐标系下的位置和旋转角度转换到末端执行器坐标系
        :param req: TransformToEndEffectorRequest，包含x_obj, y_obj, z_obj, theta_obj
        :return: TransformToEndEffectorResponse，包含position_ee_obj, rotation_ee_obj
        """
    
        pose_matrix_path = "/home/car/lashingrobots/pose_matrix.npy"

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

        # 创建ROI区域的掩码，而不是整个图像的掩码
        roi_depth = self.Depth_image_Raw[y_start:y_end, x_start:x_end]
        roi_infrared = self.image_infrared[y_start:y_end, x_start:x_end]

        # 创建一个与ROI大小相同的空图像
        sub_image = np.zeros_like(roi_infrared, dtype=roi_infrared.dtype)

        # 仅提取特定深度范围内的像素
        tolerance = 75
        mask = (roi_depth >= z_value - tolerance) & (roi_depth <= z_value + tolerance)
        
        # 应用掩码到子图像
        sub_image[mask] = roi_infrared[mask]
        
        # 调整大小为标准尺寸
        new_image = cv2.resize(sub_image, (128, 128))
        
        # 将16位图像转换为8位，以便应用直方图均衡化
        if new_image.dtype != np.uint8:
            # 归一化到 0-255 范围
            new_image_normalized = cv2.normalize(new_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        else:
            new_image_normalized = new_image

        # 创建CLAHE对象（自适应直方图均衡化）
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # 应用CLAHE处理
        new_image = clahe.apply(new_image_normalized)
        
        results = self.yolov11(new_image,conf=0.2, verbose=False)

        if results and hasattr(results[0], 'probs'):
                probs = results[0].probs
                top1_class = probs.top1  # 获取概率最高的类别
                if top1_class == 0:
                    # save_dir = '/home/yjy/newcam/src/Dataset/0'
                    # os.makedirs(save_dir, exist_ok=True)  # 确保目录存在
                    # file_name = f"Have_image_{self.shuiguan_count}.png"  # 使用有水管计数器
                    # cv2.imwrite(os.path.join(save_dir, file_name), new_image)
                    self.shuiguan_count += 1  # 有水管计数器加1
                    return False
                elif top1_class == 1:
                    # 未检测到水管，保存到 /0 目录
                    # save_dir = '/home/yjy/newcam/src/Dataset/1'
                    # os.makedirs(save_dir, exist_ok=True)  # 确保目录存在
                    # file_name = f"None_image_{self.non_shuiguan_count}.png"  # 使用没水管计数器
                    # cv2.imwrite(os.path.join(save_dir, file_name), new_image)
                    self.non_shuiguan_count += 1  # 没水管计数器加1
                    return True
        
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
        self.call_linear_module_move_service(self.displacement_x,self.displacement_y,0)
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
                    pose_matrix[0, 3] -= self.displacement_y/1000
                    pose_matrix[1, 3] -= self.displacement_x/1000
                    pose_matrix[2, 3] -= self.displacement_z/1000


                    try:
                        np.save("/home/car/lashingrobots/pose_matrix.npy", pose_matrix)
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
    def pre_img(self):
        if self.image is None:
            return None  # 返回空列表
        self.channels = self.cv2.split(self.image)
        # 分别获取X, Y, Z通道
        self.x_channel = (self.channels[0]).astype(np.int32)
        self.y_channel = (self.channels[1]).astype(np.int32)
        self.Depth_image_Raw =(self.channels[2]).astype(np.int32)
        self.Depth_image = (self.channels[2]- self.Calibration_offset_z).astype(np.int32)
        self.Depth_image_Raw[self.y1:self.y2, self.x1:self.x2] = 0
        self.Depth_image_Raw[self.y3:self.y4, self.x3:self.x4] = 0
        self.Depth_image_Raw_raw = cv2.normalize(self.Depth_image_Raw, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        self.max_depth = int(np.max(self.Depth_image_Raw) - 15)
        self.Depth_image_Range = self.cv2.inRange(self.Depth_image_Raw, 10, self.max_depth )
        self.Depth_image_Raw_uni = cv2.normalize(self.Depth_image_Range, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


        _, self.Depth_image_Raw_binary = cv2.threshold( self.Depth_image_Raw_uni, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        self.Depth_image_Raw_binary = cv2.medianBlur(self.Depth_image_Raw_binary, 5) 
        # 定义ROI区域

        # # 闭运算

        self.closing = self.cv2.morphologyEx(self.Depth_image_Raw_binary, self.cv2.MORPH_OPEN, (5,5), iterations=3)
   

        # # 细化处理
        self.skeleton = ximgproc.thinning(self.closing, thinningType=ximgproc.THINNING_ZHANGSUEN)
      
        # 使用滑动条参数的霍夫变换检测直线
        self.lines = self.cv2.HoughLinesP(self.skeleton, rho=1, theta=np.pi/180, threshold=self.threshold, 
                                        minLineLength=self.minLineLength, maxLineGap=self.maxLineGap)
        
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
                
                if angle_diff <= 25:  # 保留与轴线偏差≤15度的直线
                    filtered_lines.append(line)
            
            self.lines = np.array(filtered_lines)
        self.line_image = np.zeros_like(self.image_infrared)
        
        # 绘制检测到的直线
        if self.lines is not None:
            for line in self.lines:
                x1, y1, x2, y2 = line[0]
                self.cv2.line(self.line_image, (x1, y1), (x2, y2), (255, 255, 255), 2)

     

        # 计算所有线段对的交点
        if self.lines is not None:
            self.intersections, self.angles = self.calculate_intersections(self.lines)

        # 如果没有交点或交点数组为空,直接返回
        if self.intersections is None or len(self.intersections) == 0:
            return None  # 返回空列表

        # 使用 DBSCAN 算法对交点进行聚类
        self.clustering = DBSCAN(eps=20, min_samples=1).fit(self.intersections)

        # 获取聚类后的标签
        self.labels = self.clustering.labels_
        # 创建一个字典来存储每个聚类的点
        self.clusters = {}
        for i, label in enumerate(self.labels):
            if label not in self.clusters:
                self.clusters[label] = []
            self.clusters[label].append(self.intersections[i])

        # 计算每个聚类的中心点
        self.centers = [
            [
                int(sum(p[0] for p in points) / len(points)) - self.offset_x,
                int(sum(p[1] for p in points) / len(points)) - self.offset_y,
                [
                    self.x_channel[int(sum(p[1] for p in points) / len(points)) - self.offset_y,
                                   int(sum(p[0] for p in points) / len(points)) - self.offset_x],
                    self.y_channel[int(sum(p[1] for p in points) / len(points)) - self.offset_y,
                                   int(sum(p[0] for p in points) / len(points)) - self.offset_x],
                    self.Depth_image_Raw[int(sum(p[1] for p in points) / len(points)) - self.offset_y,
                                     int(sum(p[0] for p in points) / len(points)) - self.offset_x]
                ]
            ]
            for points in self.clusters.values()
        ]


        if self.centers is  None:
            pass
        # 过滤掉不在像素范围内的点
        # self.point1 = (375, 205)  # 点1坐标
        # self.point2 = (580, 400)  # 点2坐标
        self.point1 = (0, 0)  # 点1坐标
        self.point2 = (640, 480)  # 点2坐标

        
        # filtered_centers = [
        #     center for center in self.centers
        #      if self.point1[0] <= center[0] <= self.point2[0] and self.point1[1] <= center[1] <= self.point2[1]  # 检查像素点是否在两点确定的矩形区域内
        # ]
        
        # sorted_centers = self.snake_sort(self.centers)
        self.sorted_centers = self.centers

        self.PointCoordinates_array_msg = PointsArray()
        
        self.PointCoordinates_array_msg.PointCoordinatesArray = []
        self.result_finally = []
    
        
        for idx, center in enumerate(self.sorted_centers):
            # 根据标志变量决定发送基数点还是偶数点
            x, y = int(center[0]), int(center[1])
            self.x_value, self.y_value, self.z_value = center[2]

            if self.x_value == 0 and self.y_value == 0 and self.z_value == 0:
                min_distance = float('inf')
                nearest_world = None
                # 遍历所有其他中心点寻找最近的有效点（世界坐标非零）
                for other_center in self.sorted_centers:
                    if other_center == center:  # 跳过自身
                        continue
                    other_x_pix, other_y_pix = other_center[0], other_center[1]
                    other_wx, other_wy, other_wz = other_center[2]
                    # 跳过其他无效点（世界坐标为0的点）
                    if other_wx == 0 and other_wy == 0 and other_wz == 0:
                        continue
                    # 计算像素坐标的欧氏距离
                    distance = ((x - other_x_pix)**2 + (y - other_y_pix)**2)**0.5
                    if distance < min_distance:
                        min_distance = distance
                        nearest_world = (other_wx, other_wy, other_wz)
                # 如果找到有效点则替换
                if nearest_world is not None:
                    self.x_value, self.y_value, self.z_value = nearest_world

            raw_z = self.Depth_image_Raw[y, x] 

            self.x_value, self.y_value, self.z_value = self.transform_to_gripper_frame(self.x_value, self.y_value, self.z_value,idx)

            
            self.is_shuiguan = self.save_image(x,y,raw_z)
            self.PointCoordinates = PointCoords()
            self.PointCoordinates.is_shuiguan = self.is_shuiguan 
            self.PointCoordinates.Angle = float((self.angles[idx] / 2)-88)
            self.PointCoordinates.idx = idx + 1
            self.PointCoordinates.Pix_coord = [x, y]
            self.PointCoordinates.World_coord = [self.x_value, self.y_value, self.z_value]
            self.PointCoordinates_array_msg.PointCoordinatesArray.append(self.PointCoordinates)
    

                
        
        self.PointCoordinates_array_msg.count = len(self.PointCoordinates_array_msg.PointCoordinatesArray)
        self.coordinate_publisher.publish(self.PointCoordinates_array_msg)
  

    

        # 返回处理结果
        return self.PointCoordinates_array_msg
        
 



if __name__ == '__main__':
    rospy.init_node('ready_to_run')
    processor = ImageProcessor()
    rospy.spin()
