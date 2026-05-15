#!/usr/bin/env python3
"""Relay old chassis_ctrl cabin state into tie_robot_vision message type."""

import rospy

from tie_robot_vision.msg import cabin_upload as VisionCabinUpload

try:
    from chassis_ctrl.msg import cabin_upload as LegacyCabinUpload
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "legacy_chassis_state_relay requires the old chassis_ctrl package."
    ) from exc


def copy_cabin_state(msg):
    output = VisionCabinUpload()
    for field_name in VisionCabinUpload.__slots__:
        if hasattr(msg, field_name):
            setattr(output, field_name, getattr(msg, field_name))
    return output


class LegacyChassisStateRelay:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/cabin/cabin_data_upload")
        output_topic = rospy.get_param("~output_topic", "/tie_robot_vision/cabin_data_upload")
        self.publisher = rospy.Publisher(output_topic, VisionCabinUpload, queue_size=10)
        self.subscriber = rospy.Subscriber(
            input_topic,
            LegacyCabinUpload,
            self.handle_state,
            queue_size=20,
        )
        rospy.loginfo("tie_robot_vision cabin state relay: %s -> %s", input_topic, output_topic)

    def handle_state(self, msg):
        self.publisher.publish(copy_cabin_state(msg))


def main():
    rospy.init_node("tie_robot_vision_legacy_chassis_state_relay")
    LegacyChassisStateRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
