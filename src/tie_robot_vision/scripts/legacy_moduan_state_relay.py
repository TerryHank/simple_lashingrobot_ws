#!/usr/bin/env python3
"""Relay old chassis_ctrl linear-module state into tie_robot_vision message type."""

import rospy

from tie_robot_vision.msg import linear_module_upload as VisionLinearModuleUpload

try:
    from chassis_ctrl.msg import linear_module_upload as LegacyLinearModuleUpload
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "legacy_moduan_state_relay requires the old chassis_ctrl package."
    ) from exc


def copy_linear_module_state(msg):
    output = VisionLinearModuleUpload()
    for field_name in VisionLinearModuleUpload.__slots__:
        if hasattr(msg, field_name):
            setattr(output, field_name, getattr(msg, field_name))
    return output


class LegacyModuanStateRelay:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/moduan/moduan_gesture_data")
        output_topic = rospy.get_param("~output_topic", "/tie_robot_vision/moduan_gesture_data")
        self.publisher = rospy.Publisher(output_topic, VisionLinearModuleUpload, queue_size=10)
        self.subscriber = rospy.Subscriber(
            input_topic,
            LegacyLinearModuleUpload,
            self.handle_state,
            queue_size=20,
        )
        rospy.loginfo("tie_robot_vision moduan state relay: %s -> %s", input_topic, output_topic)

    def handle_state(self, msg):
        self.publisher.publish(copy_linear_module_state(msg))


def main():
    rospy.init_node("tie_robot_vision_legacy_moduan_state_relay")
    LegacyModuanStateRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
