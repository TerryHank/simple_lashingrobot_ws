#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml  
import os

class ArucoPoseDetector:
    def __init__(self):
        rospy.init_node('aruco_pose_detector', anonymous=True)

        # --- 1. 从 YAML 文件读取相机内参和畸变系数（你的格式：list 直接定义）---
        yaml_path = rospy.get_param('~camera_info_yaml', '/home/hyq-/free-walking-bind-robot/src/free-walking-binding-robot/fast_image_solve/config/params_orbbec.yaml')  # 可通过参数指定，或使用默认 ./camera_info.yaml
        if not os.path.exists(yaml_path):
            rospy.logerr(f"❌ YAML 文件不存在: {yaml_path}")
            rospy.signal_shutdown("Camera info YAML file not found.")
            return

        self.load_camera_info_from_simple_yaml(yaml_path)


        # --- 3. CV Bridge ---
        self.bridge = CvBridge()

        # --- 4. 订阅图像话题（默认通常是 /camera/image_raw，根据你的实际话题修改）---
        self.image_sub = rospy.Subscriber("/Scepter/color/image_raw", Image, self.image_callback)

        # --- 5. 保存位姿矩阵路径 ---
        self.pose_save_path = "/home/hyq-/free-walking-bind-robot/src/free-walking-binding-robot/fast_image_solve/config/extric_matrix.txt"
        rospy.loginfo(f"✅ ArUco 位姿检测已启动，使用你提供的 YAML 文件: {yaml_path}，位姿将保存至: {self.pose_save_path}")

    def load_camera_info_from_simple_yaml(self, yaml_path):
        """从你的简单 list 格式 YAML 文件中加载 camera_matrix 和 distortion_coefficients"""
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)  # 使用 safe_load 解析 YAML

        # --- 相机内参 (3x3) ---
        camera_matrix_list = data.get('camera_matrix')
        if not camera_matrix_list or len(camera_matrix_list) != 9:
            rospy.logerr("❌ camera_matrix 格式错误，应为包含 9 个元素的列表")
            rospy.signal_shutdown("Invalid camera_matrix in YAML.")
            return

        self.camera_matrix = np.array(camera_matrix_list, dtype=np.float64).reshape(3, 3)
        
        # --- 畸变系数 (1x5) ---
        dist_coeffs_list = data.get('distortion_coefficients')
        if not dist_coeffs_list or len(dist_coeffs_list) != 5:
            rospy.logerr("❌ distortion_coefficients 格式错误，应为包含 5 个元素的列表")
            rospy.signal_shutdown("Invalid distortion_coefficients in YAML.")
            return

        self.dist_coeffs = np.array(dist_coeffs_list, dtype=np.float64)

        rospy.loginfo("📷 相机内参和畸变系数已成功从 YAML 文件加载:")
        rospy.loginfo(f"相机内参矩阵 (3x3):\n{self.camera_matrix}")
        rospy.loginfo(f"畸变系数 (1x5): {self.dist_coeffs}")

    def image_callback(self, msg):
        try:
            # x1_ = 0
            # y1_ = 0
            # x2_ = 640
            # y2_ = 72
            # x3_ = 560
            # y3_ = 72
            # x4_ = 640
            # y4_ = 480
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # # 第1个区域：(x1_, y1_) -> (x2_, y2_)
            # x1, y1 = x1_, y1_
            # w1 = x2_ - x1_
            # h1 = y2_ - y1_
            # cv_image[y1:y1+h1, x1:x1+w1] = 0

            # # 第2个区域：(x3_, y3_) -> (x4_, y4_)
            # x3, y3 = x3_, y3_
            # w2 = x4_ - x3_
            # h2 = y4_ - y3_
            # cv_image[y3:y3+h2, x3:x3+w2] = 0

            # # 第3个区域：(x1_, y3_) -> (x4_, y4_)
            # x1_3, y1_3 = x1_, y3_
            # x4_3, y4_3 = x4_, y4_
            # cv_image[y1_3:y4_3, x1_3:x4_3] = 0

            # 第4个区域：比较复杂，原C++代码是：
            # cv::Rect(x4_ - x3_ , y4_ - (x4_ - x3_), x4_ - 2 * (x4_ - x3_), x4_ - x3_)
            # x_start = x4_ - x3_
            # y_start = y4_ - (x4_ - x3_)
            # width = x4_ - 2 * (x4_ - x3_)
            # height = x4_ - x3_

            # 确保宽高不为负（可选，根据你的实际用途决定）
            # if width > 0 and height > 0:
            #     cv_image[y_start:y_start+height, x_start:x_start+width] = 0
            
            cv2.imshow("raw", cv_image)
            cv2.waitKey(1)
            
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()

            corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)



            if ids is not None and len(ids) > 0:
                rospy.loginfo(f"✅ 检测到 ArUco 标记，IDs: {ids.flatten()}")

                # --- 2. 估算位姿 ---
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.06, self.camera_matrix, self.dist_coeffs
                )
                print(corners[0].shape) 
                # cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
                for i, marker_corners in enumerate(corners):
                    # 打印调试信息（可选）
                    print(f"Marker {i} 的角点形状: {marker_corners.shape}")

                    # 提取真实的 4个角点数据
                    if marker_corners.ndim == 3 and marker_corners.shape[0] == 1:
                        real_corners = marker_corners[0]  # shape: (4, 2)
                    elif marker_corners.ndim == 2 and marker_corners.shape == (4, 2):
                        real_corners = marker_corners  # 已经是 (4, 2)，直接使用
                    else:
                        print(f"⚠️ 不支持的角点数据形状: {marker_corners.shape}，跳过此标记。")
                        continue  # 跳过这个标记，避免后续出错

                    # 获取当前 marker 的 ID
                    marker_id = ids[i][0] if ids is not None and i < len(ids) and len(ids[i]) > 0 else i

                    # 遍历四个角点
                    for j, corner in enumerate(real_corners):
                        x, y = int(corner[0]), int(corner[1])

                        # 画红色实心圆点
                        cv2.circle(cv_image, (x, y), radius=8, color=(0, 0, 255), thickness=-1)

                        # 在角点旁显示编号 (0, 1, 2, 3)
                        cv2.putText(cv_image, str(j), (x + 10, y + 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    # 在标记中心显示 ID
                    center = np.mean(real_corners, axis=0).astype(int)
                    cv2.putText(cv_image, f"ID:{marker_id}", (center[0], center[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                
                cv2.imshow("ArUco Detection", cv_image)
                cv2.waitKey(1)

                rvec = rvecs[0][0]  # 旋转向量 (3x1)
                tvec = tvecs[0][0]  # 平移向量 (3x1)

                # --- 3. 旋转向量 → 旋转矩阵 ---
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # --- 4. 构造 4x4 位姿矩阵 ---
                pose_matrix = np.eye(4, dtype=np.float64)
                pose_matrix[:3, :3] = rotation_matrix
                pose_matrix[:3, 3] = tvec

                # --- 5. 保存位姿矩阵到文件 ---
                self.save_pose_matrix_to_txt(pose_matrix)

        except Exception as e:
            rospy.logerr(f"❌ 处理图像时出错: {e}")

    def save_pose_matrix_to_txt(self, matrix):
        try:
            np.savetxt(self.pose_save_path, matrix, fmt='%.6f')
            rospy.loginfo(f"📌 4x4 位姿矩阵已保存到: {self.pose_save_path}")
        except Exception as e:
            rospy.logerr(f"❌ 保存位姿矩阵失败: {e}")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    print(cv2.__version__)
    try:
        node = ArucoPoseDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass