#!/usr/bin/env python3

import unittest
from pathlib import Path
import sys

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

PROCESS_IMAGE_SERVICE_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "src"
    / "tie_robot_perception"
    / "pointai"
    / "process_image_service.py"
)
CHANGELOG_PATH = WORKSPACE_ROOT.parent / "CHANGELOG.md"
PR_FPRG_KNOWLEDGE_PATH = WORKSPACE_ROOT.parent / "docs" / "handoff" / "2026-04-23_pr_fprg_knowledge.md"
POINTAI_SCRIPT_PATH = WORKSPACE_ROOT / "tie_robot_perception" / "scripts" / "pointai_node.py"
LEGACY_POINTAI_SCRIPT_PATH = WORKSPACE_ROOT / "tie_robot_perception" / "scripts" / "pointAI.py"
POINTAI_RUNTIME_CONFIG_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "src"
    / "tie_robot_perception"
    / "pointai"
    / "runtime_config.py"
)
POINTAI_NODE_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "src"
    / "tie_robot_perception"
    / "pointai"
    / "node.py"
)
POINTAI_ROS_INTERFACES_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "src"
    / "tie_robot_perception"
    / "pointai"
    / "ros_interfaces.py"
)
POINTAI_DIAGNOSTICS_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "src"
    / "tie_robot_perception"
    / "pointai"
    / "diagnostics.py"
)
PR_FPRG_SCHEME_ABLATION_REPORT_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "tools"
    / "pr_fprg_scheme_ablation_report.py"
)
ALGORITHM_STACK_LAUNCH_PATH = WORKSPACE_ROOT / "tie_robot_bringup" / "launch" / "algorithm_stack.launch"
LASHING_CONFIG_PATH = WORKSPACE_ROOT / "tie_robot_perception" / "data" / "lashing_config.json"
LEGACY_RANSAC_HOUGH_ARCHIVE_DIR = (
    WORKSPACE_ROOT.parent
    / "docs"
    / "archive"
    / "legacy_ransac_hough_pointai"
)


def _draw_synthetic_line_family(response, line_angle_deg, rhos, sigma_px=1.25, amp=1.0):
    height, width = response.shape[:2]
    y_coords, x_coords = np.indices((height, width), dtype=np.float32)
    line_angle_rad = np.deg2rad(float(line_angle_deg))
    normal_x = -float(np.sin(line_angle_rad))
    normal_y = float(np.cos(line_angle_rad))
    rho_map = (normal_x * x_coords) + (normal_y * y_coords)
    for rho in rhos:
        distance = np.abs(rho_map - float(rho))
        response += (float(amp) * np.exp(-0.5 * (distance / float(sigma_px)) ** 2)).astype(np.float32)


def _line_normal_for_angle(line_angle_deg):
    line_angle_rad = np.deg2rad(float(line_angle_deg))
    return [-float(np.sin(line_angle_rad)), float(np.cos(line_angle_rad))]


def _angle_delta_deg(left_angle, right_angle):
    delta = abs((float(left_angle) - float(right_angle)) % 180.0)
    return min(delta, 180.0 - delta)


class PointAIScanOnlyPrFrpgTest(unittest.TestCase):
    def test_pointai_script_is_only_a_thin_ros_entrypoint(self):
        pointai_text = POINTAI_SCRIPT_PATH.read_text(encoding="utf-8")
        line_count = len(pointai_text.splitlines())

        self.assertLessEqual(line_count, 40)
        self.assertIn("from tie_robot_perception.pointai.node import ImageProcessor, main", pointai_text)
        self.assertFalse(LEGACY_POINTAI_SCRIPT_PATH.exists())
        self.assertNotIn("class ImageProcessor", pointai_text)
        self.assertNotIn("rospy.Subscriber", pointai_text)
        self.assertNotIn("rospy.Publisher", pointai_text)
        self.assertNotIn("diagnostic_updater.Updater", pointai_text)

    def test_pointai_node_setup_lives_in_standard_python_package_modules(self):
        for path in (POINTAI_NODE_PATH, POINTAI_ROS_INTERFACES_PATH, POINTAI_DIAGNOSTICS_PATH):
            with self.subTest(path=path):
                self.assertTrue(path.exists(), str(path))

        node_text = POINTAI_NODE_PATH.read_text(encoding="utf-8")
        ros_interfaces_text = POINTAI_ROS_INTERFACES_PATH.read_text(encoding="utf-8")
        diagnostics_text = POINTAI_DIAGNOSTICS_PATH.read_text(encoding="utf-8")

        self.assertIn("class ImageProcessor", node_text)
        self.assertIn("def main(", node_text)
        self.assertIn("def register_ros_interfaces(self):", ros_interfaces_text)
        self.assertIn("rospy.Subscriber", ros_interfaces_text)
        self.assertIn("rospy.Publisher", ros_interfaces_text)
        self.assertIn("def setup_visual_diagnostics(self):", diagnostics_text)
        self.assertIn("def produce_visual_algorithm_diagnostics(self, stat):", diagnostics_text)

    def test_height_threshold_is_ros_param_not_single_value_json(self):
        pointai_text = POINTAI_SCRIPT_PATH.read_text(encoding="utf-8")
        ros_interfaces_text = POINTAI_ROS_INTERFACES_PATH.read_text(encoding="utf-8")
        runtime_config_text = POINTAI_RUNTIME_CONFIG_PATH.read_text(encoding="utf-8")
        algorithm_stack_launch = ALGORITHM_STACK_LAUNCH_PATH.read_text(encoding="utf-8")

        self.assertFalse(
            LASHING_CONFIG_PATH.exists(),
            "height_threshold 应该走 pointAINode 私有 ROS 参数，不应再用单值 lashing_config.json",
        )
        self.assertNotIn("lashing_config.json", pointai_text + ros_interfaces_text + runtime_config_text)
        self.assertNotIn("cali_offset_file", pointai_text + ros_interfaces_text + runtime_config_text)
        self.assertIn('rospy.get_param("~height_threshold", self.height_threshold)', runtime_config_text)
        self.assertIn('rospy.set_param("~height_threshold", float(self.height_threshold))', runtime_config_text)
        self.assertIn('name="height_threshold" value="7.0"', algorithm_stack_launch)
        self.assertIn("set_height_threshold", ros_interfaces_text)

    def test_manual_workspace_s2_static_helpers_accept_runtime_arguments(self):
        from tie_robot_perception.pointai import manual_workspace_s2
        from tie_robot_perception.pointai.processor import bind_image_processor_methods

        class DummyProcessor:
            pass

        bind_image_processor_methods(DummyProcessor)

        profile = np.cos(np.arange(160, dtype=np.float32) * (2.0 * np.pi / 16.0))
        estimate = DummyProcessor().estimate_workspace_s2_period_and_phase(
            profile,
            min_period=10,
            max_period=30,
        )
        self.assertIsNotNone(estimate)
        self.assertIn("period", estimate)
        self.assertEqual(
            manual_workspace_s2.estimate_workspace_s2_period_and_phase.__code__.co_varnames[:1],
            ("profile",),
        )
        self.assertEqual(
            manual_workspace_s2.build_workspace_s2_rectified_geometry.__code__.co_varnames[:1],
            ("corner_pixels",),
        )
        self.assertEqual(
            manual_workspace_s2.build_workspace_s2_projective_line_segments.__code__.co_varnames[:1],
            ("corner_pixels",),
        )
        self.assertTrue(
            callable(DummyProcessor().score_workspace_s2_oriented_line_family_result),
            "pointAINode 运行态需要绑定 PR-FPRG 线族评分函数。",
        )
        self.assertTrue(
            callable(DummyProcessor().filter_workspace_s2_rectified_points_outside_mask),
            "pointAINode 运行态需要绑定 PR-FPRG 结构边缘过滤函数。",
        )
        self.assertTrue(
            callable(DummyProcessor().build_workspace_s2_structural_edge_suppression_mask),
            "pointAINode 运行态需要绑定 PR-FPRG 结构边缘抑制函数。",
        )

    def test_rendering_bbox_overlap_is_bound_as_static_helper(self):
        from tie_robot_perception.pointai.processor import bind_image_processor_methods

        class DummyProcessor:
            pass

        bind_image_processor_methods(DummyProcessor)
        dummy = DummyProcessor()

        self.assertTrue(dummy.bboxes_overlap((0, 0, 10, 10), (8, 8, 18, 18)))
        self.assertFalse(dummy.bboxes_overlap((0, 0, 10, 10), (40, 40, 50, 50)))

    def test_process_image_wait_loop_uses_pr_fprg_as_main_visual_path(self):
        service_text = PROCESS_IMAGE_SERVICE_PATH.read_text(encoding="utf-8")
        start_index = service_text.index("def wait_for_stable_point_coords(self, request_mode):")
        end_index = service_text.index("def handle_process_image(self, req):", start_index)
        wait_loop_text = service_text[start_index:end_index]

        self.assertIn(
            "main_visual_result = self.run_manual_workspace_s2_pipeline(publish=True)",
            wait_loop_text,
        )
        self.assertIn(
            "当前主视觉方案为方向自适应 PR-FPRG，请先提交并保存工作区四边形。",
            wait_loop_text,
        )
        self.assertIn(
            "pointAI process_image timed out while waiting for direction-adaptive PR-FPRG main vision",
            wait_loop_text,
        )
        self.assertIn(
            "if request_mode in (PROCESS_IMAGE_MODE_SCAN_ONLY, PROCESS_IMAGE_MODE_EXECUTION_REFINE):",
            wait_loop_text,
        )
        self.assertNotIn("self.pre_img(", wait_loop_text)
        self.assertNotIn("try_scan_only_manual_workspace_s2", wait_loop_text)
        self.assertNotIn(
            "return self.evaluate_point_coords_for_mode(latest_point_coords, request_mode)\n        if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:",
            wait_loop_text,
        )

    def test_project_logs_record_scan_only_pr_fprg_rule(self):
        changelog_text = CHANGELOG_PATH.read_text(encoding="utf-8")
        knowledge_text = PR_FPRG_KNOWLEDGE_PATH.read_text(encoding="utf-8")

        self.assertIn(
            "`pointAI` 的主视觉链路统一切到方向自适应 `PR-FPRG`",
            changelog_text,
        )
        self.assertIn(
            "运行主链为方向自适应版本：透视展开后在 `theta/rho` 空间估计两组钢筋线族",
            knowledge_text,
        )

    def test_manual_workspace_s2_result_also_bridges_to_coordinate_point(self):
        rendering_path = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "rendering.py"
        )
        rendering_text = rendering_path.read_text(encoding="utf-8")
        start_index = rendering_text.index("def publish_manual_workspace_s2_result(self, result_image, points_array_msg):")
        end_index = rendering_text.index("def draw_mask_contours(self, result_image, workspace_mask, color, thickness):", start_index)
        publish_text = rendering_text[start_index:end_index]

        self.assertIn("self.coordinate_publisher.publish(points_array_msg)", publish_text)
        self.assertIn("self.manual_workspace_s2_points_pub.publish(points_array_msg)", publish_text)
        self.assertIn("encoding='bgr8'", publish_text)

    def test_pointai_exposes_canonical_lashing_perception_aliases(self):
        ros_interfaces_text = POINTAI_ROS_INTERFACES_PATH.read_text(encoding="utf-8")
        rendering_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "rendering.py"
        ).read_text(encoding="utf-8")
        workspace_masks_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "workspace_masks.py"
        ).read_text(encoding="utf-8")
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        self.assertIn("'/perception/lashing/recognize_once'", ros_interfaces_text)
        self.assertIn("Trigger", ros_interfaces_text)
        self.assertIn("self.handle_lashing_recognize_once", ros_interfaces_text)
        self.assertIn("'/perception/lashing/result_image'", ros_interfaces_text)
        self.assertIn("'/perception/lashing/result_image_compressed'", ros_interfaces_text)
        self.assertIn("'/perception/lashing/points_camera'", ros_interfaces_text)
        self.assertIn("'/perception/lashing/workspace/quad_pixels'", ros_interfaces_text)
        self.assertIn("self.lashing_result_image_pub.publish(result_image_msg)", rendering_text)
        self.assertIn("self.lashing_result_image_compressed_pub.publish(compress_msg)", rendering_text)
        self.assertIn("self.lashing_points_camera_pub.publish(points_array_msg)", rendering_text)
        self.assertIn("self.lashing_workspace_quad_pixels_pub.publish(message)", workspace_masks_text)
        self.assertIn("def handle_lashing_recognize_once(self, _req):", manual_workspace_s2_text)
        self.assertIn("self.run_manual_workspace_s2()", manual_workspace_s2_text)

    def test_raw_camera_bind_point_tf_uses_scepter_depth_frame_and_meters(self):
        import rospy
        from tie_robot_msgs.msg import PointCoords, PointsArray
        from tie_robot_perception.pointai import bind_point_tf

        point = PointCoords()
        point.idx = 7
        point.World_coord = [123.0, -456.0, 789.0]

        points_array_msg = PointsArray()
        points_array_msg.PointCoordinatesArray = [point]
        points_array_msg.count = 1

        transforms = bind_point_tf.build_raw_camera_bind_point_transforms(
            points_array_msg,
            rospy.Time(42),
        )

        self.assertEqual(len(transforms), 1)
        transform = transforms[0]
        self.assertEqual(transform.header.frame_id, "Scepter_depth_frame")
        self.assertEqual(transform.header.stamp, rospy.Time(42))
        self.assertEqual(transform.child_frame_id, "pr_fprg_bind_point_7")
        self.assertAlmostEqual(transform.transform.translation.x, 0.123)
        self.assertAlmostEqual(transform.transform.translation.y, -0.456)
        self.assertAlmostEqual(transform.transform.translation.z, 0.789)
        self.assertAlmostEqual(transform.transform.rotation.w, 1.0)

    def test_raw_camera_bind_point_tf_is_cached_and_republished_with_fresh_stamp(self):
        import rospy
        from tie_robot_msgs.msg import PointCoords, PointsArray
        from tie_robot_perception.pointai import bind_point_tf

        class DummyBroadcaster:
            def __init__(self):
                self.sent_snapshots = []

            def sendTransform(self, transforms):
                self.sent_snapshots.append([
                    (
                        transform.header.stamp.to_sec(),
                        transform.header.frame_id,
                        transform.child_frame_id,
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    )
                    for transform in transforms
                ])

        class DummyProcessor:
            raw_bind_point_tf_broadcaster = DummyBroadcaster()
            raw_bind_point_tf_source_frame = "Scepter_depth_frame"
            raw_bind_point_tf_child_prefix = "pr_fprg_bind_point"

        point = PointCoords()
        point.idx = 3
        point.World_coord = [10.0, 20.0, 830.0]
        points_array_msg = PointsArray()
        points_array_msg.PointCoordinatesArray = [point]
        points_array_msg.count = 1

        timestamps = iter([rospy.Time(10), rospy.Time(11)])
        original_now = bind_point_tf.rospy.Time.now
        bind_point_tf.rospy.Time.now = lambda: next(timestamps)
        try:
            dummy = DummyProcessor()
            bind_point_tf.publish_raw_camera_bind_point_transforms(dummy, points_array_msg)
            bind_point_tf.republish_latest_raw_camera_bind_point_transforms(dummy)
        finally:
            bind_point_tf.rospy.Time.now = original_now

        self.assertEqual(len(dummy.raw_bind_point_tf_broadcaster.sent_snapshots), 2)
        first_send, second_send = dummy.raw_bind_point_tf_broadcaster.sent_snapshots
        self.assertEqual(first_send[0][0], 10.0)
        self.assertEqual(second_send[0][0], 11.0)
        self.assertEqual(first_send[0][1:], second_send[0][1:])

    def test_pointai_publishes_current_detection_points_as_raw_camera_tf(self):
        ros_interfaces_text = POINTAI_ROS_INTERFACES_PATH.read_text(encoding="utf-8")
        processor_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "processor.py"
        ).read_text(encoding="utf-8")
        rendering_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "rendering.py"
        ).read_text(encoding="utf-8")

        self.assertIn("tf2_ros.TransformBroadcaster()", ros_interfaces_text)
        self.assertIn('self.raw_bind_point_tf_source_frame = "Scepter_depth_frame"', ros_interfaces_text)
        self.assertIn('self.raw_bind_point_tf_child_prefix = "pr_fprg_bind_point"', ros_interfaces_text)
        self.assertIn("rospy.Timer(", ros_interfaces_text)
        self.assertIn("self.republish_latest_raw_camera_bind_point_transforms", ros_interfaces_text)
        self.assertIn("from . import bind_point_tf", processor_text)
        self.assertIn(
            "cls.publish_raw_camera_bind_point_transforms = bind_point_tf.publish_raw_camera_bind_point_transforms",
            processor_text,
        )
        self.assertIn(
            "cls.republish_latest_raw_camera_bind_point_transforms = bind_point_tf.republish_latest_raw_camera_bind_point_transforms",
            processor_text,
        )
        self.assertIn(
            "self.publish_raw_camera_bind_point_transforms(points_array_msg)",
            rendering_text,
        )

    def test_result_workspace_overlay_prefers_map_polygon_contour_for_display(self):
        from tie_robot_perception.pointai import rendering
        from tie_robot_perception.pointai.constants import PROCESS_IMAGE_MODE_DEFAULT

        manual_mask = np.zeros((80, 100), dtype=np.uint8)
        manual_mask[10:50, 20:70] = 1
        cabin_inside_mask = np.zeros((80, 100), dtype=np.uint8)
        cabin_inside_mask[10:50, 20:70] = 1

        class DummyProcessor:
            current_result_request_mode = PROCESS_IMAGE_MODE_DEFAULT

            def __init__(self):
                self.calls = []
                self.drawn_mask = None

            def get_manual_workspace_pixel_mask(self):
                self.calls.append("manual_pixel")
                return manual_mask

            def get_manual_workspace_cabin_polygon_pixel_mask(self):
                self.calls.append("cabin_inside")
                return cabin_inside_mask

            def get_scan_workspace_pixel_mask(self):
                self.calls.append("scan")
                return None

            def draw_mask_contours(self, result_image, workspace_mask, color, thickness):
                self.drawn_mask = np.array(workspace_mask, copy=True)

        dummy = DummyProcessor()
        rendering.draw_scan_workspace_overlay(dummy, np.zeros((80, 100), dtype=np.uint8))

        self.assertEqual(dummy.calls, ["cabin_inside"])
        self.assertTrue(np.array_equal(dummy.drawn_mask, cabin_inside_mask))

    def test_world_workspace_polygon_mask_covers_inside_area(self):
        from tie_robot_perception.pointai import workspace_masks

        grid_x, grid_y = np.meshgrid(
            np.arange(100, dtype=np.float32),
            np.arange(80, dtype=np.float32),
        )
        polygon_xy = np.array(
            [
                [20.0, 10.0],
                [70.0, 10.0],
                [70.0, 50.0],
                [20.0, 50.0],
            ],
            dtype=np.float32,
        )
        valid_mask = np.ones(grid_x.shape, dtype=bool)

        inside_mask = workspace_masks.build_convex_polygon_inside_mask(
            grid_x,
            grid_y,
            polygon_xy,
            valid_mask,
        )

        self.assertIsNotNone(inside_mask)
        self.assertEqual(int(inside_mask[10, 20]), 1)
        self.assertEqual(int(inside_mask[50, 70]), 1)
        self.assertEqual(int(inside_mask[30, 45]), 1)
        self.assertEqual(int(inside_mask[5, 5]), 0)

    def test_realtime_result_image_skips_legacy_roi_rectangle_when_manual_workspace_exists(self):
        image_buffers_path = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "image_buffers.py"
        )
        image_buffers_text = image_buffers_path.read_text(encoding="utf-8")

        self.assertIn("if self.load_manual_workspace_quad() is None:", image_buffers_text)
        self.assertIn("cv2.rectangle(result_image, self.point1, self.point2, 255, 2)", image_buffers_text)
        self.assertLess(
            image_buffers_text.index("if self.load_manual_workspace_quad() is None:"),
            image_buffers_text.index("cv2.rectangle(result_image, self.point1, self.point2, 255, 2)"),
        )

    def test_manual_workspace_s2_labels_are_drawn_directly_above_each_point(self):
        from tie_robot_perception.pointai import rendering

        class DummyProcessor:
            def __init__(self):
                self.image_infrared_copy = np.zeros((120, 160), dtype=np.uint8)
                self.draw_calls = []

            def load_manual_workspace_quad(self):
                return None

            def find_non_overlapping_label_position(self, *_args, **_kwargs):
                raise AssertionError("PR-FPRG labels must not use avoidance positioning")

            def get_text_bbox(self, text, position, font=None, font_scale=0.23, thickness=1):
                return rendering.get_text_bbox(
                    self,
                    text,
                    position,
                    font=font or rendering.cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale=font_scale,
                    thickness=thickness,
                )

            def draw_text_with_background(self, image, text, position, **kwargs):
                self.draw_calls.append((text, position))
                return rendering.draw_text_with_background(self, image, text, position, **kwargs)

        dummy = DummyProcessor()
        workspace_mask = np.ones((120, 160), dtype=np.uint8)
        display_points = [
            (1, [50, 50], [0.0, 0.0, 0.0], "selected", "S2"),
            (2, [82, 76], [0.0, 0.0, 0.0], "selected", "S2"),
        ]

        result_image = rendering.render_manual_workspace_s2_result_image(
            dummy,
            workspace_mask,
            {"vertical": [([10, 10], [140, 10])], "horizontal": [([20, 20], [20, 100])]},
            display_points,
        )

        self.assertEqual(result_image.shape, (120, 160, 3))
        self.assertEqual(result_image[10, 20].tolist(), [0, 255, 0])
        self.assertEqual(result_image[50, 50].tolist(), [0, 255, 255])
        self.assertEqual([call[0] for call in dummy.draw_calls], ["1", "2"])
        for (_text, (label_x, label_y)), (_idx, (point_x, point_y), *_rest) in zip(dummy.draw_calls, display_points):
            self.assertLess(label_y, point_y)
            self.assertLess(abs(label_x - point_x), 10)

    def test_manual_workspace_s2_publishes_raw_camera_world_coord_without_projection(self):
        from tie_robot_perception.pointai import manual_workspace_s2

        class DummyProcessor:
            fixed_z_value = 0.0

            def get_valid_world_coord_near_pixel(self, pixel_x, pixel_y):
                return [123, -456, 1789], [pixel_x, pixel_y], False

            def sort_polygon_indices_clockwise(self, points):
                return [0, 1, 2, 3]

        workspace_mask = np.ones((120, 160), dtype=np.uint8)
        manual_workspace = {
            "corner_pixels": [[0, 0], [100, 0], [100, 100], [0, 100]],
            "corner_world_camera_frame": [
                [0.0, 0.0, 100.0],
                [1000.0, 0.0, 100.0],
                [1000.0, 1000.0, 100.0],
                [0.0, 1000.0, 100.0],
            ],
        }
        rectified_geometry = {
            "rectified_width": 101,
            "rectified_height": 101,
        }

        points_array, display_points = manual_workspace_s2.build_manual_workspace_s2_points_array(
            DummyProcessor(),
            [[12, 34]],
            workspace_mask,
            rectified_intersections=[[50.0, 25.0]],
            rectified_geometry=rectified_geometry,
            manual_workspace=manual_workspace,
        )

        self.assertEqual(points_array.count, 1)
        self.assertEqual(display_points[0][2], [123.0, -456.0, 1789.0])
        self.assertEqual(
            list(points_array.PointCoordinatesArray[0].World_coord),
            [123.0, -456.0, 1789.0],
        )

    def test_manual_workspace_s2_logs_raw_camera_coord_without_cabin_projection(self):
        from tie_robot_perception.pointai import manual_workspace_s2

        class DummyProcessor:
            fixed_z_value = 0.0

            def get_valid_world_coord_near_pixel(self, pixel_x, pixel_y):
                return [30, 40, 1200], [pixel_x, pixel_y], False

        captured_logs = []
        original_loginfo = manual_workspace_s2.rospy.loginfo
        manual_workspace_s2.rospy.loginfo = lambda *args: captured_logs.append(args)
        try:
            points_array, _display_points = manual_workspace_s2.build_manual_workspace_s2_points_array(
                DummyProcessor(),
                [[12, 34]],
                np.ones((80, 80), dtype=np.uint8),
            )
        finally:
            manual_workspace_s2.rospy.loginfo = original_loginfo

        self.assertEqual(points_array.count, 1)
        self.assertTrue(captured_logs)
        log_template = captured_logs[0][0]
        log_args = captured_logs[0][1:]
        self.assertIn("pointAI tie point camera distance", log_template)
        self.assertIn("source_frame=Scepter_depth_frame", log_template)
        self.assertNotIn("target_frame=map", log_template)
        self.assertIn("camera_depth_z_mm=%.1f", log_template)
        self.assertIn("camera_distance_mm=%.1f", log_template)
        self.assertIn("camera_xyz_mm=(%.1f,%.1f,%.1f)", log_template)
        self.assertEqual(log_args[0], 1)
        self.assertAlmostEqual(log_args[5], 1200.0, places=3)
        self.assertAlmostEqual(log_args[6], 1200.0, places=3)
        self.assertAlmostEqual(log_args[7], 1201.041, places=2)
        self.assertEqual(len(log_args), 8)

    def test_pointai_publishes_raw_camera_world_coord_and_does_not_project_to_cabin(self):
        world_coord_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "world_coord.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("img[:, :, 0], img[:, :, 1]", world_coord_text)
        self.assertNotIn("img[:, :, 1] *= -1", world_coord_text)
        self.assertIn("self.x_channel = (image_raw_world_channels[0]).astype(np.float32)", world_coord_text)
        self.assertIn("self.y_channel = (image_raw_world_channels[1]).astype(np.float32)", world_coord_text)
        self.assertIn("self.depth_v = (image_raw_world_channels[2]).astype(np.float32)", world_coord_text)

        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("published_world_coord = self.to_cabin_world_coord(", manual_workspace_s2_text)
        self.assertNotIn("source_frame=target_frame", manual_workspace_s2_text)
        self.assertNotIn("self.apply_spatial_calibration(", manual_workspace_s2_text)
        self.assertIn("point_msg.World_coord = [", manual_workspace_s2_text)
        self.assertIn("float(camera_coord[0])", manual_workspace_s2_text)
        self.assertIn("float(camera_coord[1])", manual_workspace_s2_text)
        self.assertIn("float(camera_coord[2])", manual_workspace_s2_text)
        self.assertIn("log_manual_workspace_s2_camera_distance(", manual_workspace_s2_text)
        self.assertNotIn("published_world_coord", manual_workspace_s2_text)

    def test_manual_workspace_s2_stability_prefers_phase_consensus_over_single_high_score_outlier(self):
        from tie_robot_perception.pointai import manual_workspace_s2

        def candidate(vertical_phase, horizontal_phase, score):
            return {
                "vertical_estimate": {"period": 20, "phase": vertical_phase, "score": score},
                "horizontal_estimate": {"period": 22, "phase": horizontal_phase, "score": score},
            }

        selected = manual_workspace_s2.select_stable_manual_workspace_s2_inputs([
            candidate(10, 7, 1.0),
            candidate(0, 7, 1.4),
            candidate(10, 7, 1.0),
            candidate(11, 7, 0.9),
        ])

        self.assertEqual(selected["vertical_estimate"]["phase"], 10)
        self.assertEqual(selected["horizontal_estimate"]["phase"], 7)

    def test_manual_workspace_s2_stability_merges_profiles_before_final_phase(self):
        from tie_robot_perception.pointai import manual_workspace_s2

        def periodic_profile(phase, period=20, size=140):
            profile = np.zeros(size, dtype=np.float32)
            for position in range(phase, size, period):
                profile[position] = 1.0
                if position - 1 >= 0:
                    profile[position - 1] = 0.45
                if position + 1 < size:
                    profile[position + 1] = 0.45
            return profile

        def candidate(vertical_phase, horizontal_phase):
            vertical_profile = periodic_profile(vertical_phase)
            horizontal_profile = periodic_profile(horizontal_phase)
            return {
                "vertical_profile": vertical_profile,
                "horizontal_profile": horizontal_profile,
                "vertical_estimate": {
                    "period": 20,
                    "phase": 0,
                    "score": 1.0,
                    "profile": vertical_profile,
                },
                "horizontal_estimate": {
                    "period": 20,
                    "phase": 0,
                    "score": 1.0,
                    "profile": horizontal_profile,
                },
            }

        selected = manual_workspace_s2.select_stable_manual_workspace_s2_inputs([
            candidate(6, 8),
            candidate(6, 8),
            candidate(15, 8),
            candidate(6, 8),
        ])

        self.assertEqual(selected["vertical_estimate"]["phase"], 6)
        self.assertEqual(selected["horizontal_estimate"]["phase"], 8)

    def test_workspace_s2_refines_global_line_positions_to_local_response_peaks(self):
        from tie_robot_perception.perception import workspace_s2

        profile = np.zeros(80, dtype=np.float32)
        for peak_x in [12, 32, 52, 72]:
            profile[peak_x] = 1.0
            profile[peak_x - 1] = 0.55
            profile[peak_x + 1] = 0.55
        profile[24] = 1.4

        refined = workspace_s2.refine_workspace_s2_line_positions_to_local_peaks(
            profile,
            [10, 30, 50, 70],
            search_radius_px=4,
            min_spacing_px=12,
        )

        self.assertEqual(refined, [12, 32, 52, 72])
        self.assertEqual(np.diff(refined).tolist(), [20, 20, 20])

    def test_workspace_s2_refine_prefers_half_pitch_edge_margin_when_scores_tie(self):
        from tie_robot_perception.perception import workspace_s2

        profile = np.zeros(81, dtype=np.float32)
        for peak_x in [10, 30, 50, 70, 18, 38, 58, 78]:
            profile[peak_x] = 1.0

        refined = workspace_s2.refine_workspace_s2_line_positions_to_local_peaks(
            profile,
            [18, 38, 58, 78],
            search_radius_px=8,
            min_spacing_px=12,
            target_edge_margin_ratio=0.55,
            edge_anchor_weight=0.25,
        )

        self.assertEqual(refined, [10, 30, 50, 70])

    def test_workspace_s2_refine_uses_infrared_profile_to_choose_visually_aligned_phase(self):
        from tie_robot_perception.perception import workspace_s2

        depth_profile = np.zeros(81, dtype=np.float32)
        infrared_profile = np.zeros(81, dtype=np.float32)
        for peak_x in [18, 38, 58, 78]:
            depth_profile[peak_x] = 1.0
        for peak_x in [10, 30, 50, 70]:
            infrared_profile[peak_x] = 1.0

        refined = workspace_s2.refine_workspace_s2_line_positions_to_local_peaks(
            depth_profile,
            [18, 38, 58, 78],
            search_radius_px=10,
            min_spacing_px=12,
            target_edge_margin_ratio=0.55,
            edge_anchor_weight=0.0,
            auxiliary_profile=infrared_profile,
            auxiliary_weight=1.5,
        )

        self.assertEqual(refined, [10, 30, 50, 70])

    def test_workspace_s2_keeps_only_peak_supported_periodic_lines(self):
        from tie_robot_perception.perception import workspace_s2

        profile = np.zeros(101, dtype=np.float32)
        for peak_x in [20, 60]:
            profile[peak_x] = 1.0
            profile[peak_x - 1] = 0.6
            profile[peak_x + 1] = 0.6
        for weak_x in [0, 40, 80, 100]:
            profile[weak_x] = 0.08

        selected = workspace_s2.select_workspace_s2_peak_supported_line_positions(
            profile,
            [0, 20, 40, 60, 80, 100],
            search_radius_px=2,
            min_spacing_px=12,
            min_peak_ratio=0.45,
        )

        self.assertEqual(selected, [20, 60])

    def test_workspace_s2_peak_support_keeps_close_irregular_supported_lines(self):
        from tie_robot_perception.perception import workspace_s2

        profile = np.zeros(101, dtype=np.float32)
        for peak_x in [20, 30, 70]:
            profile[peak_x] = 1.0
            profile[peak_x - 1] = 0.6
            profile[peak_x + 1] = 0.6

        selected = workspace_s2.select_workspace_s2_peak_supported_line_positions(
            profile,
            [20, 30, 70],
            search_radius_px=2,
            min_spacing_px=12,
            duplicate_spacing_px=4,
            min_peak_ratio=0.45,
        )

        self.assertEqual(selected, [20, 30, 70])

    def test_workspace_s2_rejects_peak_line_without_continuous_bar_support(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((90, 120), dtype=np.float32)
        mask = np.ones_like(response, dtype=np.uint8)
        response[18:21, :] = 1.0
        response[58:61, :] = 0.95
        response[78:81, 4:22] = 1.0
        response[78:81, 80:98] = 0.9

        selected = workspace_s2.select_workspace_s2_continuous_line_positions(
            response,
            mask,
            [19, 59, 79],
            orientation="horizontal",
            search_radius_px=3,
            min_spacing_px=12,
            min_response_ratio=0.35,
            segment_count=8,
            min_segment_coverage=0.6,
        )

        self.assertEqual(selected, [19, 59])

    def test_workspace_s2_snaps_candidate_to_continuous_bar_ridge(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((100, 90), dtype=np.float32)
        mask = np.ones_like(response, dtype=np.uint8)
        response[:, 31:34] = 1.0
        response[70:84, 62:66] = 1.0

        selected = workspace_s2.select_workspace_s2_continuous_line_positions(
            response,
            mask,
            [29, 64],
            orientation="vertical",
            search_radius_px=5,
            min_spacing_px=12,
            min_response_ratio=0.35,
            segment_count=10,
            min_segment_coverage=0.6,
        )

        self.assertEqual(selected, [32])

    def test_workspace_s2_spacing_cleanup_keeps_irregular_real_rebar_lines(self):
        from tie_robot_perception.perception import workspace_s2

        selected = workspace_s2.prune_workspace_s2_line_positions_by_near_duplicate_spacing(
            [9, 115, 202, 224, 292],
            duplicate_spacing_px=4,
        )

        self.assertEqual(selected, [9, 115, 202, 224, 292])

    def test_workspace_s2_spacing_cleanup_only_merges_near_duplicate_lines(self):
        from tie_robot_perception.perception import workspace_s2

        selected = workspace_s2.prune_workspace_s2_line_positions_by_near_duplicate_spacing(
            [9, 115, 202, 203, 292],
            duplicate_spacing_px=4,
        )

        self.assertEqual(selected, [9, 115, 202, 292])

    def test_workspace_s2_scored_spacing_keeps_stronger_close_real_line(self):
        from tie_robot_perception.perception import workspace_s2

        selected = workspace_s2.prune_workspace_s2_line_positions_by_scored_spacing(
            [9, 115, 202, 224, 292],
            line_scores={202: 0.42, 224: 1.18},
            min_spacing_ratio=0.60,
        )

        self.assertEqual(selected, [9, 115, 224, 292])

    def test_workspace_s2_scored_spacing_collapses_dense_floor_seam_candidates_to_lattice(self):
        from tie_robot_perception.perception import workspace_s2

        dense_candidates = [
            -308.0,
            -256.0,
            -246.0,
            -236.0,
            -216.0,
            -214.0,
            -196.0,
            -186.0,
            -184.0,
            -166.0,
            -156.0,
            -154.0,
            -136.0,
            -128.0,
            -106.0,
            -101.0,
            -98.0,
            -75.0,
            -56.0,
            -53.0,
            -26.0,
            4.0,
            14.0,
            30.0,
            39.0,
        ]
        line_scores = {
            -308.0: 0.70,
            -236.0: 0.86,
            -166.0: 0.82,
            -98.0: 0.88,
            -26.0: 0.80,
            39.0: 0.74,
        }

        selected = workspace_s2.prune_workspace_s2_line_rhos_by_scored_spacing(
            dense_candidates,
            line_scores=line_scores,
            min_spacing_ratio=0.60,
        )

        self.assertLessEqual(len(selected), 6)
        self.assertGreaterEqual(len(selected), 4)
        self.assertEqual(selected[:4], [-308.0, -236.0, -166.0, -98.0])
        self.assertGreaterEqual(min(np.diff(selected)), 55.0)

    def test_workspace_s2_intersects_skew_line_families_inside_rectified_workspace(self):
        from tie_robot_perception.perception import workspace_s2

        family_a = {
            "line_angle_deg": 18.0,
            "normal": _line_normal_for_angle(18.0),
            "line_rhos": [-10.0, 20.0, 50.0],
        }
        family_b = {
            "line_angle_deg": 108.0,
            "normal": _line_normal_for_angle(108.0),
            "line_rhos": [-100.0, -70.0],
        }

        intersections = workspace_s2.intersect_workspace_s2_oriented_line_families(
            family_a,
            family_b,
            rectified_width=150,
            rectified_height=120,
        )

        self.assertEqual(len(intersections), 6)
        for x_value, y_value in intersections:
            self.assertGreaterEqual(x_value, 0.0)
            self.assertLessEqual(x_value, 149.0)
            self.assertGreaterEqual(y_value, 0.0)
            self.assertLessEqual(y_value, 119.0)

    def test_workspace_s2_traces_curved_line_centerline_from_local_ridges(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((90, 140), dtype=np.float32)
        y_coords, x_coords = np.indices(response.shape, dtype=np.float32)
        expected_y = 44.0 + (4.0 * np.sin((x_coords - 18.0) / 24.0))
        distance = np.abs(y_coords - expected_y)
        response += np.exp(-0.5 * (distance / 1.35) ** 2).astype(np.float32)
        mask = np.ones_like(response, dtype=np.uint8)

        curved_line = workspace_s2.trace_workspace_s2_curved_line_centerline(
            response,
            mask,
            line_angle_deg=0.0,
            line_rho=44.0,
            search_radius_px=8,
            sample_step_px=4,
            trace_method="greedy",
        )

        self.assertGreater(curved_line["coverage"], 0.85)
        sampled_points = np.asarray(curved_line["polyline_points"], dtype=np.float32)
        sampled_x = sampled_points[:, 0]
        sampled_y = sampled_points[:, 1]
        expected_sampled_y = 44.0 + (4.0 * np.sin((sampled_x - 18.0) / 24.0))
        self.assertLess(float(np.max(np.abs(sampled_y - expected_sampled_y))), 2.0)

    def test_workspace_s2_dynamic_curve_trace_resists_single_false_peak(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((90, 140), dtype=np.float32)
        y_coords, x_coords = np.indices(response.shape, dtype=np.float32)
        expected_y = 46.0 + (0.035 * (x_coords - 70.0))
        distance = np.abs(y_coords - expected_y)
        response += np.exp(-0.5 * (distance / 1.3) ** 2).astype(np.float32)
        response[58:62, 66:74] = 2.4
        mask = np.ones_like(response, dtype=np.uint8)

        curved_line = workspace_s2.trace_workspace_s2_curved_line_centerline(
            response,
            mask,
            line_angle_deg=0.0,
            line_rho=46.0,
            search_radius_px=16,
            sample_step_px=4,
            trace_method="dynamic_programming",
            smoothness_weight=0.16,
        )

        sampled_points = np.asarray(curved_line["polyline_points"], dtype=np.float32)
        sampled_x = sampled_points[:, 0]
        sampled_y = sampled_points[:, 1]
        expected_sampled_y = 46.0 + (0.035 * (sampled_x - 70.0))
        self.assertLess(float(np.max(np.abs(sampled_y - expected_sampled_y))), 3.0)

    def test_workspace_s2_intersects_curved_line_families_by_polyline_geometry(self):
        from tie_robot_perception.perception import workspace_s2

        first_family = {
            "line_angle_deg": 0.0,
            "normal": _line_normal_for_angle(0.0),
            "curved_lines": [
                {
                    "base_rho": 40.0,
                    "polyline_points": [[0.0, 40.0], [40.0, 40.0], [80.0, 42.0]],
                }
            ],
        }
        second_family = {
            "line_angle_deg": 90.0,
            "normal": _line_normal_for_angle(90.0),
            "curved_lines": [
                {
                    "base_rho": -50.0,
                    "polyline_points": [[50.0, 0.0], [50.0, 90.0]],
                }
            ],
        }

        intersections = workspace_s2.intersect_workspace_s2_curved_line_families(
            first_family,
            second_family,
            rectified_width=100,
            rectified_height=100,
        )

        self.assertEqual(len(intersections), 1)
        self.assertAlmostEqual(intersections[0][0], 50.0, places=3)
        self.assertAlmostEqual(intersections[0][1], 40.5, places=3)

    def test_workspace_s2_stage_ablation_can_skip_continuous_validation(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((120, 150), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [18.0, 48.0, 78.0], sigma_px=1.2, amp=1.0)
        _draw_synthetic_line_family(response, 90.0, [-122.0, -92.0, -62.0], sigma_px=1.2, amp=1.0)
        mask = np.ones_like(response, dtype=np.uint8)

        line_families = workspace_s2.build_workspace_s2_oriented_line_families(
            response,
            mask,
            min_period=24,
            max_period=36,
            angle_step_deg=2.0,
            enable_continuous_validation=False,
            enable_spacing_prune=False,
        )

        self.assertGreaterEqual(len(line_families), 2)
        for family in line_families[:2]:
            self.assertEqual(family["continuous_rhos"], family["peak_rhos"])
            self.assertEqual(family["line_rhos"], family["peak_rhos"])

    def test_workspace_s2_oriented_line_families_reject_too_sparse_pairs(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((120, 150), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [36.0, 86.0], sigma_px=1.2, amp=1.0)
        _draw_synthetic_line_family(response, 90.0, [-108.0, -58.0], sigma_px=1.2, amp=1.0)
        mask = np.ones_like(response, dtype=np.uint8)

        line_families = workspace_s2.build_workspace_s2_oriented_line_families(
            response,
            mask,
            min_period=40,
            max_period=60,
            angle_step_deg=2.0,
            enable_spacing_prune=False,
        )

        self.assertLess(len(line_families), 2)

    def test_workspace_s2_structural_edge_suppression_masks_wide_beam_bundle(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((160, 230), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0], sigma_px=1.1)
        _draw_synthetic_line_family(response, 90.0, [-25.0, -60.0, -95.0, -130.0, -165.0, -190.0], sigma_px=1.1)
        _draw_synthetic_line_family(response, 90.0, [-206.0, -212.0, -218.0], sigma_px=2.0, amp=1.4)
        for angle in (60.0, 120.0):
            _draw_synthetic_line_family(
                response,
                angle,
                [-90.0, -55.0, -20.0, 15.0, 50.0, 85.0, 120.0, 155.0],
                sigma_px=1.2,
                amp=0.9,
            )
        mask = np.ones_like(response, dtype=np.uint8)

        beam_mask = workspace_s2.build_workspace_s2_structural_edge_suppression_mask(response, mask)

        self.assertGreater(float(np.mean(beam_mask[:, 206:222])), 0.65)
        self.assertLess(float(np.mean(beam_mask[:, 187:194])), 0.20)
        self.assertLess(float(np.mean(beam_mask[:, 22:29])), 0.20)

    def test_workspace_s2_structural_edge_detection_reports_beam_band_location(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((160, 230), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0], sigma_px=1.1)
        _draw_synthetic_line_family(response, 90.0, [-25.0, -60.0, -95.0, -130.0, -165.0, -190.0], sigma_px=1.1)
        _draw_synthetic_line_family(response, 90.0, [-206.0, -212.0, -218.0], sigma_px=2.0, amp=1.4)
        for angle in (60.0, 120.0):
            _draw_synthetic_line_family(
                response,
                angle,
                [-90.0, -55.0, -20.0, 15.0, 50.0, 85.0, 120.0, 155.0],
                sigma_px=1.2,
                amp=0.9,
            )
        mask = np.ones_like(response, dtype=np.uint8)

        beam_bands = workspace_s2.detect_workspace_s2_structural_edge_bands(response, mask)

        right_vertical_bands = [
            band for band in beam_bands
            if band["axis"] == "x" and band["side"] == "max"
        ]
        self.assertTrue(right_vertical_bands)
        self.assertGreaterEqual(right_vertical_bands[0]["start"], 198)
        self.assertGreaterEqual(right_vertical_bands[0]["peak"], 1.4)

    def test_workspace_s2_structural_edge_detection_ignores_horizontal_tcp_intrusion(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((160, 230), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0], sigma_px=1.1)
        _draw_synthetic_line_family(response, 90.0, [-25.0, -60.0, -95.0, -130.0, -165.0, -190.0], sigma_px=1.1)
        _draw_synthetic_line_family(response, 90.0, [-206.0, -212.0, -218.0], sigma_px=2.0, amp=1.4)
        response[4:17, :] += 2.2
        mask = np.ones_like(response, dtype=np.uint8)

        beam_bands = workspace_s2.detect_workspace_s2_structural_edge_bands(response, mask)
        beam_mask = workspace_s2.build_workspace_s2_structural_edge_suppression_mask(response, mask)

        self.assertTrue([band for band in beam_bands if band["axis"] == "x"])
        self.assertFalse([band for band in beam_bands if band["axis"] == "y"])
        self.assertLess(float(np.mean(beam_mask[4:17, :])), 0.20)

    def test_workspace_s2_oriented_line_families_keep_grid_when_edge_beam_is_present(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((160, 230), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0], sigma_px=1.1)
        _draw_synthetic_line_family(response, 90.0, [-25.0, -60.0, -95.0, -130.0, -165.0, -190.0], sigma_px=1.1)
        _draw_synthetic_line_family(response, 90.0, [-206.0, -212.0, -218.0], sigma_px=2.0, amp=1.4)
        for angle in (60.0, 120.0):
            _draw_synthetic_line_family(
                response,
                angle,
                [-90.0, -55.0, -20.0, 15.0, 50.0, 85.0, 120.0, 155.0],
                sigma_px=1.2,
                amp=0.9,
            )
        mask = np.ones_like(response, dtype=np.uint8)

        line_families = workspace_s2.build_workspace_s2_oriented_line_families(
            response,
            mask,
            min_period=18,
            max_period=45,
            angle_step_deg=2.0,
            enable_local_peak_refine=False,
        )
        intersections = workspace_s2.intersect_workspace_s2_oriented_line_families(
            line_families[0],
            line_families[1],
            rectified_width=230,
            rectified_height=160,
        )

        self.assertEqual(sorted(len(family["line_rhos"]) for family in line_families[:2]), [6, 7])
        self.assertEqual(len(intersections), 42)

    def test_workspace_s2_filters_bind_points_inside_vertical_beam_mask(self):
        from tie_robot_perception.perception import workspace_s2

        beam_mask = np.zeros((100, 150), dtype=bool)
        beam_mask[:, 18:31] = True
        beam_mask[:, 122:136] = True
        rectified_points = [
            (20.0, 30.0),
            (50.0, 30.0),
            (90.0, 30.0),
            (128.0, 30.0),
            (50.0, 70.0),
        ]

        filtered_points = workspace_s2.filter_workspace_s2_rectified_points_outside_mask(
            rectified_points,
            beam_mask,
        )

        self.assertEqual(filtered_points, [(50.0, 30.0), (90.0, 30.0), (50.0, 70.0)])

    def test_workspace_s2_filters_line_rhos_that_run_inside_vertical_beam_mask(self):
        from tie_robot_perception.perception import workspace_s2

        beam_mask = np.zeros((120, 230), dtype=bool)
        beam_mask[:, 206:222] = True

        filtered_rhos = workspace_s2.filter_workspace_s2_line_rhos_by_mask_overlap(
            [-218.0, -212.0, -206.0, -190.0, -150.0],
            line_angle_deg=90.0,
            normal=_line_normal_for_angle(90.0),
            exclusion_mask=beam_mask,
            max_overlap_fraction=0.35,
        )

        self.assertEqual(filtered_rhos, [-190.0, -150.0])

    def test_workspace_s2_family_score_prefers_dense_real_grid_over_sparse_subset(self):
        from tie_robot_perception.perception import workspace_s2

        dense_family = {
            "line_rhos": [10.0, 35.0, 60.0, 85.0, 110.0, 135.0, 160.0],
            "periodic_score": 1.0,
            "orientation_score": 0.8,
        }
        sparse_family = {
            "line_rhos": [10.0, 60.0, 110.0, 160.0, 210.0],
            "periodic_score": 1.0,
            "orientation_score": 0.8,
        }

        self.assertGreater(
            workspace_s2._workspace_s2_supported_family_score(dense_family),
            workspace_s2._workspace_s2_supported_family_score(sparse_family),
        )

    def test_workspace_s2_oriented_continuous_validation_supports_precomputed_sparse_sampling(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((120, 150), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [28.0, 58.0], sigma_px=1.2, amp=1.0)
        response[88:91, 8:28] = 1.0
        response[88:91, 96:116] = 1.0
        mask = np.ones_like(response, dtype=np.uint8)
        support_map = workspace_s2.normalize_workspace_s2_response_for_line_support(response, mask)
        support_mask = mask.astype(bool)

        selected_dense = workspace_s2.select_workspace_s2_continuous_line_rhos(
            response,
            mask,
            [28.0, 58.0, 89.0],
            0.0,
            normal=_line_normal_for_angle(0.0),
            search_radius_px=4,
            min_spacing_px=12,
            min_response_ratio=0.35,
            segment_count=8,
            min_segment_coverage=0.6,
            sample_step_px=1.0,
        )
        selected_sparse = workspace_s2.select_workspace_s2_continuous_line_rhos(
            response,
            mask,
            [28.0, 58.0, 89.0],
            0.0,
            normal=_line_normal_for_angle(0.0),
            search_radius_px=4,
            min_spacing_px=12,
            min_response_ratio=0.35,
            segment_count=8,
            min_segment_coverage=0.6,
            sample_step_px=2.0,
            precomputed_support_map=support_map,
            precomputed_support_mask=support_mask,
        )

        self.assertEqual(len(selected_dense), 2)
        self.assertEqual(len(selected_sparse), 2)
        np.testing.assert_allclose(selected_sparse, selected_dense, atol=1.0)

    def test_workspace_s2_oriented_continuous_validation_keeps_close_irregular_ridges(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((120, 150), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [42.0, 52.0, 92.0], sigma_px=1.2, amp=1.0)
        mask = np.ones_like(response, dtype=np.uint8)

        selected = workspace_s2.select_workspace_s2_continuous_line_rhos(
            response,
            mask,
            [42.0, 52.0, 92.0],
            0.0,
            normal=_line_normal_for_angle(0.0),
            search_radius_px=4,
            min_spacing_px=12,
            duplicate_spacing_px=4,
            min_response_ratio=0.35,
            segment_count=8,
            min_segment_coverage=0.6,
            sample_step_px=2.0,
        )

        self.assertEqual(selected, [42.0, 52.0, 92.0])

    def test_workspace_s2_oriented_continuous_validation_can_return_rho_scores(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((120, 150), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [42.0, 52.0, 92.0], sigma_px=1.2, amp=1.0)
        mask = np.ones_like(response, dtype=np.uint8)

        selected, line_scores = workspace_s2.select_workspace_s2_continuous_line_rhos(
            response,
            mask,
            [42.0, 52.0, 92.0],
            0.0,
            normal=_line_normal_for_angle(0.0),
            search_radius_px=4,
            min_spacing_px=12,
            duplicate_spacing_px=4,
            min_response_ratio=0.35,
            segment_count=8,
            min_segment_coverage=0.6,
            sample_step_px=2.0,
            return_scores=True,
        )

        self.assertEqual(selected, [42.0, 52.0, 92.0])
        self.assertEqual(sorted(line_scores.keys()), selected)
        self.assertGreater(line_scores[42.0], 0.0)

    def test_manual_workspace_s2_response_variant_selection_uses_depth_before_ir_fallback(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        depth_block_index = manual_workspace_s2_text.index("depth_response_variants = [")
        infrared_block_index = manual_workspace_s2_text.index("infrared_response_variants = []", depth_block_index)
        fallback_index = manual_workspace_s2_text.index("if best_variant is None:", infrared_block_index)

        self.assertLess(depth_block_index, infrared_block_index)
        self.assertLess(infrared_block_index, fallback_index)
        self.assertIn("for response_variant in depth_response_variants:", manual_workspace_s2_text)
        self.assertIn("for response_variant in infrared_response_variants:", manual_workspace_s2_text[fallback_index:])
        self.assertIn("infrared_response_variants.append(infrared_dark_line_response)", manual_workspace_s2_text)
        self.assertNotIn("\n            response_variants.append(infrared_dark_line_response)", manual_workspace_s2_text)

    def test_manual_workspace_s2_current_chain_keeps_original_continuous_ridge_mainline(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        wrapper_index = manual_workspace_s2_text.index("def build_workspace_s2_oriented_line_families(")
        wrapper_body = manual_workspace_s2_text[wrapper_index:manual_workspace_s2_text.index("def score_workspace_s2_oriented_line_family_result", wrapper_index)]

        self.assertIn("enable_local_peak_refine=True", wrapper_body)
        self.assertIn("enable_continuous_validation=True", wrapper_body)
        self.assertIn("enable_spacing_prune=True", wrapper_body)
        self.assertIn("use_orientation_prior_angle_pool=True", wrapper_body)
        self.assertNotIn("enable_continuous_validation=False", wrapper_body)

    def test_manual_workspace_s2_filters_beam_points_with_structural_edge_response(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"structural_edge_response_crop": structural_edge_response', manual_workspace_s2_text)
        self.assertIn('s2_inputs.get("structural_edge_response_crop", s2_inputs["response_crop"])', manual_workspace_s2_text)

    def test_stage_ablation_filters_beam_points_with_structural_edge_response(self):
        stage_ablation_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_stage_ablation.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"structural_edge_response": structural_edge_response', stage_ablation_text)
        self.assertIn('prepared.get("structural_edge_response", best_result["response"])', stage_ablation_text)

    def test_stage_ablation_prefers_first_supported_depth_response(self):
        stage_ablation_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_stage_ablation.py"
        ).read_text(encoding="utf-8")

        self.assertIn("selected_group_result = None", stage_ablation_text)
        self.assertIn("selected_group_result = {", stage_ablation_text)
        self.assertIn("if selected_group_result is not None:\n                break", stage_ablation_text)

    def test_workspace_s2_beam_mask_does_not_delete_line_rhos(self):
        workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "perception"
            / "workspace_s2.py"
        ).read_text(encoding="utf-8")

        builder_index = workspace_s2_text.index("def build_workspace_s2_oriented_line_families(")
        builder_body = workspace_s2_text[builder_index:workspace_s2_text.index("def intersect_workspace_s2_oriented_line_families", builder_index)]

        self.assertNotIn("filter_workspace_s2_line_rhos_by_mask_overlap(", builder_body)

    def test_pr_fprg_scheme_ablation_report_focuses_only_on_scheme1(self):
        report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        schemes_block = report_text[report_text.index("SCHEMES = ["):report_text.index("STAGE_VARIANTS = [")]
        self.assertIn("01_current_theta_rho", schemes_block)
        self.assertNotIn("02_archived_ir_rho", schemes_block)
        for scheme_id in ("03_greedy_depth_curve", "04_dp_depth_curve", "05_dp_ridge_curve", "06_ir_assisted_curve"):
            self.assertNotIn(scheme_id, schemes_block)

    def test_pr_fprg_scheme_ablation_report_crosses_schemes_with_stage_variants(self):
        report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        self.assertIn("for scheme in SCHEMES:", report_text)
        self.assertIn("for stage_variant in STAGE_VARIANTS:", report_text)
        self.assertIn("scheme_id", report_text)
        self.assertIn("stage_variant_id", report_text)

    def test_pr_fprg_scheme_ablation_report_has_visible_scheme_navigation(self):
        report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        self.assertIn("scheme-nav", report_text)
        self.assertIn("data-scheme-filter", report_text)
        self.assertIn("showScheme", report_text)

    def test_pr_fprg_scheme_ablation_report_does_not_render_failure_placeholder_images(self):
        report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        self.assertNotIn("render_failure_image", report_text)
        self.assertIn("failure-panel", report_text)

    def test_pr_fprg_stage_tools_default_full_pipeline_uses_continuous_ridge_mainline(self):
        stage_ablation_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_stage_ablation.py"
        ).read_text(encoding="utf-8")
        scheme_report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        self.assertIn('"enable_local_peak_refine": True', scheme_report_text)
        self.assertIn('"enable_continuous_validation": True', scheme_report_text)
        self.assertIn('variant.get("enable_local_peak_refine", True)', stage_ablation_text)
        self.assertIn('variant.get("enable_continuous_validation", True)', stage_ablation_text)

    def test_visual_debug_runtime_controls_are_exposed_to_frontend(self):
        ros_interfaces_text = POINTAI_ROS_INTERFACES_PATH.read_text(encoding="utf-8")
        runtime_config_text = POINTAI_RUNTIME_CONFIG_PATH.read_text(encoding="utf-8")

        self.assertIn("from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32", ros_interfaces_text)
        self.assertIn("'/web/pointAI/set_stable_frame_count'", ros_interfaces_text)
        self.assertIn("self.set_stable_frame_count_callback", ros_interfaces_text)
        self.assertIn("def set_stable_frame_count_callback(self, msg):", runtime_config_text)
        self.assertIn("self.stable_frame_count = max(1, int(getattr(msg, \"data\", 1)))", runtime_config_text)
        self.assertIn('rospy.set_param("~stable_frame_count", int(self.stable_frame_count))', runtime_config_text)

    def test_manual_workspace_s2_logs_single_frame_elapsed_ms(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        self.assertIn("start_time = time.perf_counter()", manual_workspace_s2_text)
        self.assertIn("elapsed_ms = (time.perf_counter() - start_time) * 1000.0", manual_workspace_s2_text)
        self.assertIn("single_frame_elapsed_ms", manual_workspace_s2_text)
        self.assertIn("elapsed_ms=%.1f", manual_workspace_s2_text)

    def test_pr_fprg_scheme_ablation_report_marks_scheme1_as_realtime_target(self):
        report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        self.assertIn('REALTIME_TARGET_SCHEME_ID = "01_current_theta_rho"', report_text)
        self.assertNotIn('ARCHIVED_CURVE_SCHEME_IDS = ["05_dp_ridge_curve", "06_ir_assisted_curve"]', report_text)

    def test_manual_workspace_s2_pipeline_uses_oriented_line_families_as_main_path(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        self.assertIn("line_families = self.build_workspace_s2_oriented_line_families(", manual_workspace_s2_text)
        self.assertIn("rectified_intersections = self.intersect_workspace_s2_oriented_line_families(", manual_workspace_s2_text)
        self.assertNotIn("vertical_lines = self.build_workspace_s2_line_positions(", manual_workspace_s2_text)
        self.assertNotIn("horizontal_lines = self.build_workspace_s2_line_positions(", manual_workspace_s2_text)

    def test_legacy_ransac_hough_pointai_code_is_archived_and_not_runtime_bound(self):
        processor_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "processor.py"
        ).read_text(encoding="utf-8")
        process_image_service_text = PROCESS_IMAGE_SERVICE_PATH.read_text(encoding="utf-8")
        active_pointai_sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (
                WORKSPACE_ROOT
                / "tie_robot_perception"
                / "src"
                / "tie_robot_perception"
                / "pointai"
            ).glob("*.py")
        )

        self.assertTrue((LEGACY_RANSAC_HOUGH_ARCHIVE_DIR / "matrix_preprocess.py").exists())
        self.assertNotIn("cls.pre_img", processor_text)
        self.assertNotIn("self.pre_img(", process_image_service_text)
        self.assertNotIn("HoughLinesP", active_pointai_sources)
        self.assertNotIn("from .matrix_preprocess import pre_img", active_pointai_sources)

    def test_manual_workspace_s2_pipeline_uses_peak_supported_lines_before_mapping_intersections(self):
        workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "perception"
            / "workspace_s2.py"
        ).read_text(encoding="utf-8")

        refine_index = workspace_s2_text.index("refined_positions = refine_workspace_s2_line_positions_to_local_peaks(")
        peak_support_index = workspace_s2_text.index("peak_positions = select_workspace_s2_peak_supported_line_positions(")
        continuous_index = workspace_s2_text.index("continuous_rhos, continuous_scores = select_workspace_s2_continuous_line_rhos(")

        self.assertLess(refine_index, peak_support_index)
        self.assertLess(peak_support_index, continuous_index)

    def test_manual_workspace_s2_pipeline_uses_continuous_lines_before_mapping_intersections(self):
        workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "perception"
            / "workspace_s2.py"
        ).read_text(encoding="utf-8")

        peak_support_index = workspace_s2_text.index("peak_rhos = build_workspace_s2_oriented_line_rhos(")
        continuous_support_index = workspace_s2_text.index("continuous_rhos, continuous_scores = select_workspace_s2_continuous_line_rhos(")
        line_assignment_index = workspace_s2_text.index('family["line_rhos"] = spacing_rhos')

        self.assertLess(peak_support_index, continuous_support_index)
        self.assertLess(continuous_support_index, line_assignment_index)

    def test_manual_workspace_s2_pipeline_resolves_spacing_conflicts_before_mapping_intersections(self):
        workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "perception"
            / "workspace_s2.py"
        ).read_text(encoding="utf-8")

        continuous_support_index = workspace_s2_text.index("continuous_rhos, continuous_scores = select_workspace_s2_continuous_line_rhos(")
        spacing_prune_index = workspace_s2_text.index("spacing_rhos = prune_workspace_s2_line_rhos_by_scored_spacing(")
        line_assignment_index = workspace_s2_text.index('family["line_rhos"] = spacing_rhos')

        self.assertLess(continuous_support_index, spacing_prune_index)
        self.assertLess(spacing_prune_index, line_assignment_index)

    def test_manual_workspace_s2_phase_lock_keeps_same_workspace_on_same_grid_phase(self):
        from tie_robot_perception.pointai import manual_workspace_s2

        class DummyProcessor:
            manual_workspace_s2_phase_lock = None

        dummy = DummyProcessor()
        s2_inputs = {
            "manual_workspace": {
                "corner_pixels": [[10, 10], [110, 10], [110, 110], [10, 110]],
            },
            "rectified_geometry": {
                "rectified_width": 120,
                "rectified_height": 120,
            },
        }

        first_vertical, first_horizontal = manual_workspace_s2.apply_manual_workspace_s2_phase_lock(
            dummy,
            s2_inputs,
            [18, 38, 58, 78],
            [16, 36, 56, 76],
        )
        second_vertical, second_horizontal = manual_workspace_s2.apply_manual_workspace_s2_phase_lock(
            dummy,
            s2_inputs,
            [10, 30, 50, 70],
            [16, 36, 56, 76],
        )

        self.assertEqual(first_vertical, [18, 38, 58, 78])
        self.assertEqual(second_vertical, [18, 38, 58, 78])
        self.assertEqual(second_horizontal, [16, 36, 56, 76])

    def test_manual_workspace_s2_phase_lock_is_not_invalidated_by_one_extra_candidate_line(self):
        from tie_robot_perception.pointai import manual_workspace_s2

        class DummyProcessor:
            manual_workspace_s2_phase_lock = None

        dummy = DummyProcessor()
        s2_inputs = {
            "manual_workspace": {
                "corner_pixels": [[10, 10], [110, 10], [110, 110], [10, 110]],
            },
            "rectified_geometry": {
                "rectified_width": 120,
                "rectified_height": 120,
            },
        }

        manual_workspace_s2.apply_manual_workspace_s2_phase_lock(
            dummy,
            s2_inputs,
            [18, 38, 58, 78],
            [16, 36, 56, 76],
        )
        vertical, horizontal = manual_workspace_s2.apply_manual_workspace_s2_phase_lock(
            dummy,
            s2_inputs,
            [8, 28, 48, 68, 88],
            [16, 36, 56, 76],
        )

        self.assertEqual(vertical, [18, 38, 58, 78])
        self.assertEqual(horizontal, [16, 36, 56, 76])

    def test_manual_workspace_s2_pipeline_refines_lines_before_mapping_intersections(self):
        workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "perception"
            / "workspace_s2.py"
        ).read_text(encoding="utf-8")

        build_index = workspace_s2_text.index("line_positions = build_workspace_s2_line_positions(")
        refine_index = workspace_s2_text.index("refined_positions = refine_workspace_s2_line_positions_to_local_peaks(")
        peak_support_index = workspace_s2_text.index("peak_positions = select_workspace_s2_peak_supported_line_positions(")

        self.assertLess(build_index, refine_index)
        self.assertLess(refine_index, peak_support_index)


if __name__ == "__main__":
    unittest.main()
