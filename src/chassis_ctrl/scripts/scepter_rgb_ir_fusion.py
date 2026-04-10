#!/usr/bin/env python3
import copy

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image


class ScepterRgbIrFusionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.queue_size = rospy.get_param("~queue_size", 15)
        self.slop = rospy.get_param("~sync_slop", 0.08)
        self.ir_weight = rospy.get_param("~ir_weight", 0.72)
        self.rgb_weight = rospy.get_param("~rgb_weight", 0.28)
        self.publish_debug = rospy.get_param("~publish_debug", False)
        self.use_clahe = rospy.get_param("~use_clahe", True)
        self.clahe_clip_limit = rospy.get_param("~clahe_clip_limit", 2.0)
        self.clahe_tile_grid_size = rospy.get_param("~clahe_tile_grid_size", 8)

        color_topic = rospy.get_param("~color_topic", "/Scepter/color/image_raw")
        ir_topic = rospy.get_param("~ir_topic", "/Scepter/ir/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/Scepter/transformedDepth/image_raw")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/Scepter/color/camera_info")

        fused_topic = rospy.get_param("~fused_topic", "/Scepter/fusion/image_raw")
        fused_camera_info_topic = rospy.get_param("~fused_camera_info_topic", "/Scepter/fusion/camera_info")
        depth_relay_topic = rospy.get_param("~depth_relay_topic", "/Scepter/fusion/depth/image_raw")

        self.fused_pub = rospy.Publisher(fused_topic, Image, queue_size=2)
        self.fused_info_pub = rospy.Publisher(fused_camera_info_topic, CameraInfo, queue_size=2)
        self.depth_relay_pub = rospy.Publisher(depth_relay_topic, Image, queue_size=2)
        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=2)

        self.clahe = cv2.createCLAHE(
            clipLimit=max(0.1, float(self.clahe_clip_limit)),
            tileGridSize=(
                max(1, int(self.clahe_tile_grid_size)),
                max(1, int(self.clahe_tile_grid_size)),
            ),
        )

        color_sub = message_filters.Subscriber(color_topic, Image)
        ir_sub = message_filters.Subscriber(ir_topic, Image)
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        camera_info_sub = message_filters.Subscriber(camera_info_topic, CameraInfo)

        sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, ir_sub, depth_sub, camera_info_sub],
            queue_size=self.queue_size,
            slop=self.slop,
        )
        sync.registerCallback(self.synced_callback)

        rospy.loginfo(
            "scepter_rgb_ir_fusion initialized: color=%s ir=%s depth=%s camera_info=%s",
            color_topic,
            ir_topic,
            depth_topic,
            camera_info_topic,
        )

    def normalize_to_mono8(self, image):
        if image is None:
            return None

        if len(image.shape) == 3 and image.shape[2] == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if image.dtype == np.uint8:
            return image

        finite_mask = np.isfinite(image)
        if not finite_mask.any():
            return cv2.convertScaleAbs(image)

        valid = image[finite_mask]
        min_val = float(valid.min())
        max_val = float(valid.max())
        if max_val <= min_val:
            return cv2.convertScaleAbs(image)

        scaled = ((image.astype("float32") - min_val) * (255.0 / (max_val - min_val)))
        scaled = scaled.clip(0.0, 255.0).astype("uint8")
        scaled[~finite_mask] = 0
        return scaled

    def enhance_gray(self, gray_image):
        if not self.use_clahe:
            return gray_image
        return self.clahe.apply(gray_image)

    def build_fused_bgr(self, color_bgr, ir_gray):
        rgb_gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
        rgb_enhanced = self.enhance_gray(rgb_gray)
        ir_enhanced = self.enhance_gray(ir_gray)

        fused_luma = cv2.addWeighted(
            ir_enhanced,
            float(self.ir_weight),
            rgb_enhanced,
            float(self.rgb_weight),
            0.0,
        )

        ycrcb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2YCrCb)
        ycrcb[:, :, 0] = fused_luma
        fused_bgr = cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)
        return fused_bgr, fused_luma

    def synced_callback(self, color_msg, ir_msg, depth_msg, camera_info_msg):
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            ir_raw = self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Fusion input conversion failed: %s", exc)
            return

        ir_gray = self.normalize_to_mono8(ir_raw)
        if ir_gray is None:
            rospy.logwarn_throttle(5.0, "Fusion skipped because IR frame is empty.")
            return

        fused_bgr, fused_luma = self.build_fused_bgr(color_bgr, ir_gray)

        fused_msg = self.bridge.cv2_to_imgmsg(fused_bgr, encoding="bgr8")
        fused_msg.header = color_msg.header
        self.fused_pub.publish(fused_msg)

        fused_info = copy.deepcopy(camera_info_msg)
        fused_info.header = color_msg.header
        self.fused_info_pub.publish(fused_info)

        depth_relay_msg = copy.deepcopy(depth_msg)
        depth_relay_msg.header.stamp = color_msg.header.stamp
        self.depth_relay_pub.publish(depth_relay_msg)

        if self.debug_pub is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(fused_luma, encoding="mono8")
            debug_msg.header = color_msg.header
            self.debug_pub.publish(debug_msg)


def main():
    rospy.init_node("scepter_rgb_ir_fusion")
    ScepterRgbIrFusionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
