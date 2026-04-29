"""pointAI ROS 节点装配入口。"""
import warnings

import rospy

from .diagnostics import bind_visual_diagnostic_methods
from .processor import bind_image_processor_methods
from .ros_interfaces import register_ros_interfaces
from .state import initialize_processor_state


warnings.filterwarnings("ignore", message="The value of the smallest subnormal for")


class ImageProcessor:
    def __init__(self):
        self.initialize_processor_state()
        self.register_ros_interfaces()
        self.load_runtime_config()
        self.publish_current_manual_workspace_quad_pixels()
        self.setup_visual_diagnostics()
        rospy.loginfo("pointAI node initialized!")
        rospy.loginfo(
            "pointAI camera_coord_source=/Scepter/worldCoord/raw_world_coord, "
            "source_frame=Scepter_depth_frame, downstream_coordinate_layer=external, "
            "height_threshold=%s",
            self.height_threshold,
        )


ImageProcessor.initialize_processor_state = initialize_processor_state
ImageProcessor.register_ros_interfaces = register_ros_interfaces
bind_image_processor_methods(ImageProcessor)
bind_visual_diagnostic_methods(ImageProcessor)


def main():
    rospy.init_node('pointAINode')
    processor = ImageProcessor()
    rospy.spin()
    return processor
