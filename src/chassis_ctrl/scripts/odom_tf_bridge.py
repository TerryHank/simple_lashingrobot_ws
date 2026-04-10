#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomTfBridge:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/vehicle/odom")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=20,
        )

    def odom_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        transform.header.frame_id = msg.header.frame_id or self.odom_frame_id
        transform.child_frame_id = msg.child_frame_id or self.base_frame_id

        # Allow explicit override so the TF tree stays stable even if the source
        # odometry message uses unexpected frame names.
        transform.header.frame_id = self.odom_frame_id or transform.header.frame_id
        transform.child_frame_id = self.base_frame_id or transform.child_frame_id

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("odom_tf_bridge")
    OdomTfBridge()
    rospy.spin()
