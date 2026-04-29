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

def printsomething(self, msg):
    rospy.loginfo("msg: %s", msg.data)


def test_callback(self):
    self.frame_count += 1
    # 计算并打印平均帧数
    cur_time =time.time()
    elapsed_time = cur_time - self.start_time
    if elapsed_time > 0.5:
        self.fps = self.frame_count / elapsed_time
        self.frame_count=0
        self.start_time = cur_time
    rospy.loginfo_throttle(1.0, "Average FPS: %.2f", self.fps)


def image_color_callback(self, msg):
    # 缓存最新的图像
    self.image_color = self.bridge.imgmsg_to_cv2(msg)
    self.mark_visual_input("color")
    # # 顺时针旋转90度
    # self.image_infrared = cv2.rotate(self.image_infrared, cv2.ROTATE_90_CLOCKWISE)

    # 确保图像是可写的
    self.image_color_copy = np.array(self.image_color, copy=True)


def image_depth_callback(self, msg):
    # 缓存最新的图像
    self.raw_depth = self.bridge.imgmsg_to_cv2(msg)
    self.mark_visual_input("depth")
    # 确保图像是可写的
    self.raw_depth = np.array(self.raw_depth, copy=True)


def image_infrared_callback(self, msg):
    # 缓存最新的图像
    self.image_infrared = self.bridge.imgmsg_to_cv2(msg)
    self.mark_visual_input("infrared")
    # # 顺时针旋转90度
    # self.image_infrared = cv2.rotate(self.image_infrared, cv2.ROTATE_90_CLOCKWISE)

    # 确保图像是可写的
    self.image_infrared_copy = np.array(self.image_infrared, copy=True)


def image_callback(self, msg):
    self.image = self.bridge.imgmsg_to_cv2(msg)
    self.mark_visual_input("world_coord")
    self.vison_image = np.array(self.image, copy=True)
    normalized = cv2.normalize(self.vison_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    self.vison_image = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

    self.channels = self.cv2.split(self.image)
    self.Depth_image_Raw =(self.channels[2]).astype(np.int32)
    self.world_image_seq += 1
    # self.mark_sign_points()
    # 确保image_infrared_copy存在
    if not hasattr(self, 'image_infrared_copy') or self.image_infrared_copy is None:
        self.image_infrared_copy = np.zeros((480, 640), dtype=np.uint8)

    result_image = np.array(self.image_infrared_copy, copy=True)
    if self.load_manual_workspace_quad() is None:
        cv2.rectangle(result_image, self.point1, self.point2, 255, 2)
    self.draw_scan_workspace_overlay(result_image)

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
        text = self.format_result_display_label(display_idx, world_coord, status_text)
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


def camera_info_callback(self,msg):

    # 从 CameraInfo 消息中提取相机内参矩阵

    # self.camera_matrix = np.array(msg.K).reshape(3, 3)
    self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]])
    # 提取畸变系数
    # self.dist_coeffs = np.array(msg.D)
    # self.dist_coeffs = np.zeros_like(np.array(msg.D))
    self.dist_coeffs = np.zeros((4, 1)) 
    self.mark_visual_input("camera_info")


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
