"""pointAI 节点运行状态初始化。"""
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge

from tie_robot_msgs.msg import motion

from .constants import PROCESS_IMAGE_MODE_DEFAULT


PERCEPTION_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
WORKSPACE_SRC_ROOT = PERCEPTION_PACKAGE_ROOT.parent


def initialize_processor_state(self):
    self.bridge = CvBridge()
    self.cv2 = cv2
    self.image = None
    self.vison_image = None
    self.depth_image = None
    self.tof_intrin = None
    self.visual_input_timestamps = {}
    self.visual_last_error_message = ""
    self.visual_last_error_time = 0.0
    self.visual_last_process_request_time = 0.0
    self.visual_last_process_success_time = 0.0
    self.intersections = None
    self.threshold = 45
    self.minLineLength = 60
    self.maxLineGap = 150
    self.roi_center_x, self.roi_center_y = 362, 258
    self.roi_width, self.roi_height = 313, 277
    self.key = None
    self.min_depth = 910
    self.max_depth = 1060
    self.location_msg = motion()
    self.frame_count = 0
    self.fps = 0
    self.offset_x, self.offset_y = 0, 0
    self.multiple = 0.996
    self.x1, self.y1 = 160, 0
    self.x2, self.y2 = 523, 80
    self.x3, self.y3 = 80, 406
    self.x4, self.y4 = 187, 480
    self.start_time = time.time()
    self.roi_bottom_left_x = 205
    self.roi_bottom_left_y = 358
    self.roi_top_right_x = 440
    self.roi_top_right_y = 120
    self.filter_points = True
    self.half_size = 20
    self.non_shuiguan_count = 0
    self.shuiguan_count = 0
    self.auto_cali_move_x = 250
    self.auto_cali_move_y = 0
    self.auto_cali_move_z = 0
    self.fixed_z_value = 0.0
    self.height_threshold = 10
    self.cali_matrix_file = os.path.join(str(PERCEPTION_PACKAGE_ROOT), "data", "pose_matrix.npy")
    self.path_points_file = os.path.join(
        str(WORKSPACE_SRC_ROOT),
        "tie_robot_process",
        "data",
        "path_points.json",
    )
    self.manual_workspace_quad_file = os.path.join(
        str(PERCEPTION_PACKAGE_ROOT),
        "data",
        "manual_workspace_quad.json",
    )

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
    self.plane_z = None
    self.pose_matrix = None
    self.point1 = (20, 114)
    self.point2 = (620, 460)
    self.stamp = rospy.Time.now()
    self.camera_matrix = None
    self.dist_coeffs = None
