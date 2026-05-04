"""pointAI ROS 订阅、发布、服务和定时器装配。"""
import rospy
import tf2_ros
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32
from std_srvs.srv import Trigger

from tie_robot_msgs.msg import PointsArray, linear_module_upload
from tie_robot_msgs.srv import ProcessImage


def register_ros_interfaces(self):
    rospy.Subscriber('/Scepter/worldCoord/world_coord', Image, self.image_callback)
    rospy.Subscriber('/Scepter/worldCoord/raw_world_coord', Image, self.image_raw_world_callback)
    rospy.Subscriber('/Scepter/color/image_raw', Image, self.image_color_callback)
    rospy.Subscriber('/Scepter/ir/image_raw', Image, self.image_infrared_callback)
    rospy.Subscriber('/Scepter/depth/image_raw', Image, self.image_depth_callback)
    rospy.Subscriber('/Scepter/ir/camera_info', CameraInfo, self.camera_info_callback)
    rospy.Subscriber('/web/pointAI/set_workspace_quad', Float32MultiArray, self.manual_workspace_quad_callback)
    rospy.Subscriber('/web/pointAI/run_workspace_s2', Bool, self.manual_workspace_s2_callback)
    rospy.Subscriber('/web/pointAI/set_stable_frame_count', Int32, self.set_stable_frame_count_callback)
    rospy.Subscriber('/web/pointAI/move_to_workspace_center_scan_pose', Bool, self.workspace_center_scan_pose_callback)
    rospy.Subscriber('/web/pointAI/set_height_threshold', Float32, self.fixed_z_value_callback)
    rospy.Subscriber('/web/moduan/send_odd_points', Bool, self.jump_bind_callback)
    rospy.Subscriber('/moduan/moduan_gesture_data', linear_module_upload, self.linear_module_state_callback)

    self.service = rospy.Service('/pointAI/process_image', ProcessImage, self.handle_process_image)
    self.lashing_recognize_once_service = rospy.Service(
        '/perception/lashing/recognize_once',
        Trigger,
        self.handle_lashing_recognize_once,
    )
    self.cropped_ir_image_pub = rospy.Publisher('/pointAI/cropped_ir_image', Image, queue_size=10)
    self.cropped_color_image_pub = rospy.Publisher('/pointAI/cropped_color_image', Image, queue_size=10)
    self.cropped_depth_image_pub = rospy.Publisher('/pointAI/cropped_depth_image', Image, queue_size=10)
    self.depth_binary_image_pub = rospy.Publisher('/pointAI/depth_binary_image', Image, queue_size=10)
    self.line_image_pub = rospy.Publisher('/pointAI/line_image', Image, queue_size=10)
    self.image_pub = rospy.Publisher('/pointAI/result_image', CompressedImage, queue_size=10)
    self.scan_surface_dp_base_image_pub = rospy.Publisher(
        '/perception/lashing/scan_surface_dp_base_image',
        Image,
        queue_size=1,
        latch=True,
    )
    self.scan_surface_dp_completed_surface_image_pub = rospy.Publisher(
        '/perception/lashing/scan_surface_dp_completed_surface_image',
        Image,
        queue_size=1,
        latch=True,
    )
    self.execution_refine_base_image_pub = rospy.Publisher(
        '/perception/lashing/execution_refine_base_image',
        Image,
        queue_size=1,
        latch=True,
    )
    self.lashing_result_image_compressed_pub = rospy.Publisher(
        '/perception/lashing/result_image_compressed',
        CompressedImage,
        queue_size=10,
    )
    self.result_image_raw_pub = rospy.Publisher('/pointAI/result_image_raw', Image, queue_size=10)
    self.lashing_result_image_pub = rospy.Publisher(
        '/perception/lashing/result_image',
        Image,
        queue_size=1,
        latch=True,
    )
    self.manual_workspace_s2_result_raw_pub = rospy.Publisher(
        '/pointAI/manual_workspace_s2_result_raw',
        Image,
        queue_size=1,
        latch=True,
    )
    self.manual_workspace_quad_pixels_pub = rospy.Publisher(
        '/pointAI/manual_workspace_quad_pixels',
        Float32MultiArray,
        queue_size=1,
        latch=True,
    )
    self.lashing_workspace_quad_pixels_pub = rospy.Publisher(
        '/perception/lashing/workspace/quad_pixels',
        Float32MultiArray,
        queue_size=1,
        latch=True,
    )
    self.coordinate_publisher = rospy.Publisher('/coordinate_point', PointsArray, queue_size=10)
    self.lashing_points_camera_pub = rospy.Publisher(
        '/perception/lashing/points_camera',
        PointsArray,
        queue_size=1,
        latch=True,
    )
    self.raw_bind_point_tf_broadcaster = tf2_ros.TransformBroadcaster()
    self.raw_bind_point_tf_source_frame = "Scepter_depth_frame"
    self.raw_bind_point_tf_child_prefix = "surface_dp_bind_point"
    self.latest_raw_bind_point_transforms = []
    self.raw_bind_point_tf_publish_rate_hz = max(
        0.1,
        float(rospy.get_param("~raw_bind_point_tf_publish_rate_hz", 10.0)),
    )
    self.raw_bind_point_tf_timer = rospy.Timer(
        rospy.Duration(1.0 / self.raw_bind_point_tf_publish_rate_hz),
        self.republish_latest_raw_camera_bind_point_transforms,
    )
    self.manual_workspace_s2_points_pub = rospy.Publisher(
        '/pointAI/manual_workspace_s2_points',
        PointsArray,
        queue_size=1,
        latch=True,
    )
