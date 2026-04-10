#!/usr/bin/env python3

import copy
import math

import rospy
from nav_msgs.msg import Odometry


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = 0.5 * yaw
    return math.sin(half_yaw), math.cos(half_yaw)


class PlanarOdomProjector:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/vehicle/odom")
        self.output_topic = rospy.get_param("~output_topic", "/vehicle/odom_planar")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.stamp_offset_sec = rospy.get_param("~stamp_offset_sec", 0.0)
        self.yaw_pose_variance = rospy.get_param("~yaw_pose_variance", 0.01)
        self.yaw_twist_variance = rospy.get_param("~yaw_twist_variance", 0.04)

        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        self.subscriber = rospy.Subscriber(self.input_topic, Odometry, self.odom_callback, queue_size=20)

    def odom_callback(self, msg):
        planar_msg = copy.deepcopy(msg)
        planar_msg.header.frame_id = self.odom_frame_id or msg.header.frame_id
        planar_msg.child_frame_id = self.base_frame_id or msg.child_frame_id

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        if self.stamp_offset_sec != 0.0:
            stamp = stamp + rospy.Duration.from_sec(self.stamp_offset_sec)
        planar_msg.header.stamp = stamp

        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        qz, qw = quaternion_from_yaw(yaw)

        planar_msg.pose.pose.position.z = 0.0
        planar_msg.pose.pose.orientation.x = 0.0
        planar_msg.pose.pose.orientation.y = 0.0
        planar_msg.pose.pose.orientation.z = qz
        planar_msg.pose.pose.orientation.w = qw

        planar_msg.twist.twist.linear.z = 0.0
        planar_msg.twist.twist.angular.x = 0.0
        planar_msg.twist.twist.angular.y = 0.0

        pose_cov = list(planar_msg.pose.covariance)
        twist_cov = list(planar_msg.twist.covariance)
        pose_cov[14] = 1e-6
        pose_cov[21] = 1e-6
        pose_cov[28] = 1e-6
        pose_cov[35] = self.yaw_pose_variance
        twist_cov[14] = 1e-6
        twist_cov[21] = 1e-6
        twist_cov[28] = 1e-6
        twist_cov[35] = self.yaw_twist_variance
        planar_msg.pose.covariance = pose_cov
        planar_msg.twist.covariance = twist_cov

        self.publisher.publish(planar_msg)


def main():
    rospy.init_node("planar_odom_projector")
    PlanarOdomProjector()
    rospy.spin()


if __name__ == "__main__":
    main()
