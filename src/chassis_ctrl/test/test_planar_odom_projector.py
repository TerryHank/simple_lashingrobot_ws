#!/usr/bin/env python3

import os
import sys
import unittest

import rospy
from nav_msgs.msg import Odometry


SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts")
)
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import planar_odom_projector  # noqa: E402


class FakePublisher:
    def __init__(self):
        self.last_msg = None

    def publish(self, msg):
        self.last_msg = msg


class PlanarOdomProjectorTest(unittest.TestCase):
    def test_planar_output_clamps_yaw_covariance(self):
        projector = planar_odom_projector.PlanarOdomProjector.__new__(
            planar_odom_projector.PlanarOdomProjector
        )
        projector.publisher = FakePublisher()
        projector.odom_frame_id = "odom"
        projector.base_frame_id = "base_link"
        projector.stamp_offset_sec = 0.0
        projector.yaw_pose_variance = 0.01
        projector.yaw_twist_variance = 0.04

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = rospy.Time(1, 0)
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[35] = 1e6
        odom.twist.covariance[35] = 1e6

        projector.odom_callback(odom)

        self.assertIsNotNone(projector.publisher.last_msg)
        self.assertLess(projector.publisher.last_msg.pose.covariance[35], 1.0)
        self.assertLess(projector.publisher.last_msg.twist.covariance[35], 1.0)


if __name__ == "__main__":
    unittest.main()
