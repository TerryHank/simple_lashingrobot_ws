#!/usr/bin/env python3

"""
ROS测试节点，用于测试GUI的状态显示功能
轮询发送std_msgs/Float32类型的话题，模拟各部件状态
状态映射：0=正常，1=警告，2=错误
ROS 1版本
"""

import rospy
from std_msgs.msg import Float32
import time
import random

class DebugGuiNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('debug_gui_node', anonymous=True)
        
        # 创建发布者字典，严格指定Float32类型
        self.publishers = {
            'chassis': rospy.Publisher('/robot/chassis_status', Float32, queue_size=10),
            'end_effector': rospy.Publisher('/robot/end_effector_status', Float32, queue_size=10),
            'binding_gun': rospy.Publisher('/robot/binding_gun_status', Float32, queue_size=10)
        }
        
        # 部件列表，用于轮询
        self.components = ['chassis', 'end_effector', 'binding_gun']
        
        # 当前轮询索引
        self.current_index = 0
        
        # 各部件初始状态（使用Float32类型）
        # 状态映射：0=正常，1=警告，2=错误
        self.states = {
            'chassis': 0.0,  # 索驱初始正常
            'end_effector': 0.0,  # 末端初始正常
            'binding_gun': 0.0  # 绑扎枪初始正常
        }
        
        # 状态值列表，用于随机切换
        self.status_values = [0.0, 1.0, 2.0]  # 0=正常，1=警告，2=错误
        
        # 状态变化间隔（秒）
        self.state_change_interval = 0.02
        self.last_state_change = time.time()
        
        rospy.loginfo('Debug GUI Node started')
        rospy.loginfo('Publishing status topics in polling mode:')
        for topic in ['/robot/chassis_status', '/robot/end_effector_status', '/robot/binding_gun_status']:
            rospy.loginfo('  - %s (std_msgs/Float32)' % topic)
        rospy.loginfo('轮询间隔: %d秒' % self.state_change_interval)
        rospy.loginfo('状态逻辑: 0=正常, 1=警告, 2=错误')
        rospy.loginfo('严格确保发布Float32类型数据')
    
    def timer_callback(self):
        # 定期切换状态
        current_time = time.time()
        if current_time - self.last_state_change > self.state_change_interval:
            # 按顺序轮询部件
            component = self.components[self.current_index]
            
            # 随机选择一个新状态
            new_state = random.choice(self.status_values)
            self.states[component] = new_state
            self.last_state_change = current_time
            
            # 严格验证数据类型
            state_value = self.states[component]
            if not isinstance(state_value, float):
                rospy.logerr('%s: 状态值不是浮点类型! 类型: %s, 值: %s' % (component, type(state_value), state_value))
                # 强制转换为浮点类型
                state_value = float(state_value)
                rospy.logerr('%s: 已强制转换为浮点类型: %s' % (component, state_value))
                self.states[component] = state_value
            
            # 创建Float32消息对象
            msg = Float32()
            
            # 严格设置data字段为浮点类型
            msg.data = state_value
            
            # 再次验证消息数据类型
            if not isinstance(msg.data, float):
                rospy.logerr('%s: 消息data字段不是浮点类型! 类型: %s, 值: %s' % (component, type(msg.data), msg.data))
                return
            
            # 发布状态
            self.publishers[component].publish(msg)
            
            # 日志
            status_str = '正常' if state_value == 0 else '警告' if state_value == 1 else '错误' if state_value == 2 else '未知'
            component_names = {
                'chassis': '索驱',
                'end_effector': '末端',
                'binding_gun': '绑扎枪'
            }
            rospy.loginfo('轮询 [%d/%d] - %s状态变化: %s (data: %s)' % 
                         (self.current_index + 1, len(self.components), 
                          component_names[component], status_str, 
                          str(state_value)))
            
            # 更新轮询索引
            self.current_index = (self.current_index + 1) % len(self.components)
    
    def run(self):
        # 设置循环频率
        rate = rospy.Rate(2)  # 2Hz
        
        while not rospy.is_shutdown():
            # 调用定时器回调
            self.timer_callback()
            # 休眠到指定频率
            rate.sleep()

def main():
    # 创建节点实例
    node = DebugGuiNode()
    
    try:
        # 运行节点
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down Debug GUI Node')
    except Exception as e:
        rospy.logerr('节点运行出错: %s' % str(e))

if __name__ == '__main__':
    main()