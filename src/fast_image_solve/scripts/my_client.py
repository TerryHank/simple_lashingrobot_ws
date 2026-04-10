#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from fast_image_solve.msg import PointsArray      # 刚才自定义的消息
from geometry_msgs.msg import Point       # 内置三维点

class CoordinatePublisher:
    def __init__(self):
        rospy.init_node('coordinate_publisher_node', anonymous=True)

        # 这就是你要的 publisher
        self.coordinate_publisher = rospy.Publisher(
            '/coordinate_point', PointsArray, queue_size=10)

        self.rate = rospy.Rate(10)  # 10 Hz

    def spin(self):
        seq = 0
        while not rospy.is_shutdown():
            msg = PointsArray()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.header.seq = seq
            seq += 1

            # 随便填两个点，替换成你自己的坐标即可
            p1 = Point(x=1.0, y=2.0, z=0.5)
            p2 = Point(x=3.0, y=4.0, z=1.5)
            msg.points = [p1, p2]

            self.coordinate_publisher.publish(msg)
            rospy.loginfo("Published %d points", len(msg.points))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CoordinatePublisher()

    except rospy.ROSInterruptException:
        pass