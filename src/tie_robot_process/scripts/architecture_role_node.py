#!/usr/bin/env python3

import rospy


def main():
    rospy.init_node("architecture_role_node")
    role_name = rospy.get_param("~role_name", rospy.get_name().lstrip("/"))
    rospy.loginfo("Process_log: %s 节点已启动，当前作为架构分层占位节点运行。", role_name)
    rospy.spin()


if __name__ == "__main__":
    main()
