#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from datetime import datetime
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerResponse
from functools import partial

# 配置字典
BUTTON_CONFIG = {
    1: {'name': '启动机器人', 'topic': '/web/cabin/start', 'type': 'std_msgs/Float32'},
    2: {'name': '规划作业路径', 'topic': '/web/cabin/plan_path', 'type': 'geometry_msgs/Pose'},
    3: {'name': '清除作业路径', 'topic': '/web/cabin/clear_path', 'type': 'std_msgs/Float32'},
    4: {'name': '开始全局作业', 'topic': '/web/cabin/start_global_work', 'type': 'std_msgs/Float32'},
    5: {'name': '重启机器人', 'topic': '/web/cabin/restart', 'type': 'std_msgs/Float32'},
    6: {'name': '末端运动调试', 'topic': '/web/moduan/moduan_move_debug', 'type': 'geometry_msgs/Pose'},
    7: {'name': '索驱运动调试', 'topic': '/web/cabin/cabin_move_debug', 'type': 'geometry_msgs/Pose'},
    8: {'name': '视觉识别调试', 'topic': '/web/fast_image_solve/process_image', 'type': 'std_msgs/Float32'},
    9: {'name': '定点绑扎调试', 'topic': '/web/moduan/single_bind', 'type': 'std_msgs/Float32'},
    10: {'name': '暂停作业', 'topic': '/web/moduan/interrupt_stop', 'type': 'std_msgs/Float32'},
    11: {'name': '开启绑扎开关', 'topic': '/web/moduan/enb_las', 'type': 'std_msgs/Float32'},
    12: {'name': '开启(关闭)跳绑', 'topic': '/web/moduan/send_odd_points', 'type': 'std_msgs/Bool'},
    13: {'name': '恢复作业', 'topic': '/web/moduan/hand_sovle_warn', 'type': 'std_msgs/Float32'},
    14: {'name': '开启(关闭)灯光', 'topic': '/web/moduan/light', 'type': 'std_msgs/Bool'},
    15: {'name': '末端回零', 'topic': '/web/moduan/moduan_move_zero', 'type': 'std_msgs/Float32'},
    16: {'name': '关闭绑扎开关', 'topic': '/web/moduan/enb_las', 'type': 'std_msgs/Float32'},
    17: {'name': '关闭机器人', 'topic': '/web/cabin/shutdown', 'type': 'std_msgs/Float32'},
    18: {'name': '急停作业', 'topic': '/web/moduan/forced_stop', 'type': 'std_msgs/Float32'},
    19: {'name': '设置高度阈值', 'topic': '/web/fast_image_solve/set_height_threshold', 'type': 'std_msgs/Float32'},
    20: {'name': '保存作业路径', 'topic': '/web/cabin/save_path', 'type': 'std_msgs/Float32'},
    21: {'name': '保存绑扎数据', 'topic': '/web/moduan/save_binding_data', 'type': 'std_msgs/Float32'},
    22: {'name': '设置TF平移标定', 'topic': '/web/fast_image_solve/set_pointAI_offset', 'type': 'geometry_msgs/Pose'},
    23: {'name': '设置索驱速度', 'topic': '/web/cabin/set_cabin_speed', 'type': 'std_msgs/Float32'},
    24: {'name': '设置末端速度', 'topic': '/web/moduan/set_moduan_speed', 'type': 'std_msgs/Float32'},
    25: {'name': '扫描建图', 'topic': '/web/cabin/start_pseudo_slam_scan', 'type': 'std_msgs/Float32'}
}

# 类型映射表
TYPE_MAP = {
    'std_msgs/Float32': Float32,
    'std_msgs/Bool': Bool,
    'geometry_msgs/Pose': Pose,
    'std_srvs/Trigger': Trigger
}

class RobotDebugListener:
    def __init__(self):
        rospy.init_node('robot_button_debugger', anonymous=True)
        rospy.loginfo("机器人指令监听调试节点已启动...")

        self.subs = []
        self.services = []

        for key, config in BUTTON_CONFIG.items():
            msg_type_str = config['type']
            topic_name = config['topic']
            btn_name = config['name']

            if msg_type_str not in TYPE_MAP:
                rospy.logwarn(f"未知类型跳过: {msg_type_str}")
                continue

            msg_type = TYPE_MAP[msg_type_str]

            # 判断是 Topic 还是 Service
            if msg_type_str == 'std_srvs/Trigger':
                # 如果是 Service，我们需要扮演 Server 端来接收请求
                srv = rospy.Service(
                    topic_name, 
                    Trigger, 
                    partial(self.service_callback, btn_name=btn_name, topic=topic_name)
                )
                self.services.append(srv)
                rospy.loginfo(f"已注册服务服务端: [{btn_name}] -> {topic_name}")
            else:
                # 如果是 Topic，我们作为 Subscriber
                sub = rospy.Subscriber(
                    topic_name, 
                    msg_type, 
                    partial(self.topic_callback, btn_name=btn_name, topic=topic_name)
                )
                self.subs.append(sub)
                rospy.loginfo(f"已订阅话题: [{btn_name}] -> {topic_name}")

    def topic_callback(self, msg, btn_name, topic):
        """通用话题回调函数"""
        # 获取当前时间为世界时间格式
        time_str = rospy.Time.now().to_sec()
        time_str = datetime.fromtimestamp(time_str).strftime('%Y-%m-%d %H:%M:%S')
        
        # 根据消息类型处理不同格式的消息
        if isinstance(msg, Float32) or isinstance(msg, Bool):
            # Float32和Bool类型的消息有data属性
            data_content = msg.data
            print(f"\n>>> [话题触发] {time_str}")
            print(f"    功能名称: {btn_name}")
            print(f"    话题路径: {topic}")
            print(f"    接收数据: {data_content} (类型: {type(data_content).__name__})")
            print("-" * 40)
        elif isinstance(msg, Pose):
            # Pose类型的消息有position和orientation属性
            position = msg.position
            orientation = msg.orientation
            print(f"\n>>> [话题触发] {time_str}")
            print(f"    功能名称: {btn_name}")
            print(f"    话题路径: {topic}")
            print(f"    接收数据: Pose消息")
            print(f"      Position: x={position.x}, y={position.y}, z={position.z}")
            print(f"      Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
            print("-" * 40)
        else:
            # 其他类型的消息，尝试转换为字符串
            print(f"\n>>> [话题触发] {time_str}")
            print(f"    功能名称: {btn_name}")
            print(f"    话题路径: {topic}")
            print(f"    接收数据: {str(msg)} (类型: {type(msg).__name__})")
            print("-" * 40)

    def service_callback(self, req, btn_name, topic):
        """通用服务回调函数"""
        # 获取当前时间为世界时间格式
        time_sec = rospy.Time.now().to_sec()
        time_str = datetime.fromtimestamp(time_sec).strftime('%Y-%m-%d %H:%M:%S')
        print(f"\n>>> [服务调用] {time_str}")
        print(f"    功能名称: {btn_name}")
        print(f"    服务路径: {topic}")
        print(f"    请求内容: (Trigger服务无请求参数)")
        print("-" * 40)
        # 返回成功响应
        return TriggerResponse(success=True, message=f"Debug node handled {btn_name}")

if __name__ == '__main__':
    try:
        listener = RobotDebugListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
