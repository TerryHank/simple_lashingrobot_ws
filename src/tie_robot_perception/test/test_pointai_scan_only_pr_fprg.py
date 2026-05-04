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
PR_FPRG_SCHEME_COMPARISON_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "tools"
    / "pr_fprg_scheme_comparison.py"
)
REBAR_INSTANCE_GRAPH_PROBE_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "tools"
    / "rebar_instance_graph_probe.py"
)
REBAR_SURFACE_BINDPOINT_COMPARISON_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "tools"
    / "rebar_surface_bindpoint_comparison.py"
)
PR_FPRG_TOPOLOGY_RECOVERY_EXPERIMENT_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "tools"
    / "pr_fprg_topology_recovery_experiment.py"
)
PR_FPRG_MULTISCALE_BINDPOINT_EXPERIMENT_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "tools"
    / "pr_fprg_multiscale_bindpoint_experiment.py"
)
PR_FPRG_PREVIOUS_SCHEMES_ARCHIVE_PATH = (
    WORKSPACE_ROOT.parent
    / "docs"
    / "archive"
    / "pr_fprg_previous_schemes_2026-04-30.md"
)
ALGORITHM_STACK_LAUNCH_PATH = WORKSPACE_ROOT / "tie_robot_bringup" / "launch" / "algorithm_stack.launch"
LASHING_CONFIG_PATH = WORKSPACE_ROOT / "tie_robot_perception" / "data" / "lashing_config.json"
LEGACY_RANSAC_HOUGH_ARCHIVE_DIR = (
    WORKSPACE_ROOT.parent
    / "docs"
    / "archive"
    / "legacy_ransac_hough_pointai"
)
EXECUTION_REFINE_HOUGH_PATH = (
    WORKSPACE_ROOT
    / "tie_robot_perception"
    / "src"
    / "tie_robot_perception"
    / "pointai"
    / "execution_refine_hough.py"
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
            callable(DummyProcessor().build_workspace_s2_projective_line_segments),
            "pointAINode 运行态需要绑定 2026-04-22 S2 透视线段映射函数。",
        )
        self.assertFalse(hasattr(DummyProcessor, "build_workspace_s2_axis_aligned_line_families"))
        self.assertFalse(hasattr(DummyProcessor, "score_workspace_s2_oriented_line_family_result"))
        self.assertFalse(hasattr(DummyProcessor, "expand_workspace_s2_exclusion_mask_by_metric_margin"))
        self.assertFalse(hasattr(DummyProcessor, "collect_stable_manual_workspace_s2_inputs"))
        self.assertFalse(hasattr(DummyProcessor, "apply_manual_workspace_s2_phase_lock"))

    def test_rendering_bbox_overlap_is_bound_as_static_helper(self):
        from tie_robot_perception.pointai.processor import bind_image_processor_methods

        class DummyProcessor:
            pass

        bind_image_processor_methods(DummyProcessor)
        dummy = DummyProcessor()

        self.assertTrue(dummy.bboxes_overlap((0, 0, 10, 10), (8, 8, 18, 18)))
        self.assertFalse(dummy.bboxes_overlap((0, 0, 10, 10), (40, 40, 50, 50)))

    def test_process_image_wait_loop_routes_scan_to_surface_dp_and_execution_to_hough(self):
        service_text = PROCESS_IMAGE_SERVICE_PATH.read_text(encoding="utf-8")
        start_index = service_text.index("def wait_for_stable_point_coords(self, request_mode):")
        end_index = service_text.index("def handle_process_image(self, req):", start_index)
        wait_loop_text = service_text[start_index:end_index]

        self.assertIn(
            "if request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE:",
            wait_loop_text,
        )
        self.assertIn(
            "execution_refine_result = self.run_execution_refine_hough_pipeline(publish=True)",
            wait_loop_text,
        )
        self.assertIn(
            "if request_mode == PROCESS_IMAGE_MODE_SCAN_ONLY:",
            wait_loop_text,
        )
        self.assertIn(
            "main_visual_result = self.run_manual_workspace_s2_pipeline(publish=True)",
            wait_loop_text,
        )
        self.assertIn(
            "当前扫描触发方案为Surface-DP物理先验扫描",
            wait_loop_text,
        )
        self.assertIn(
            "pointAI process_image timed out while waiting for execution refine plane-segmentation + Hough vision",
            wait_loop_text,
        )
        self.assertNotIn(
            "request_mode in (PROCESS_IMAGE_MODE_SCAN_ONLY, PROCESS_IMAGE_MODE_EXECUTION_REFINE)",
            wait_loop_text,
        )
        self.assertNotIn("try_scan_only_manual_workspace_s2", wait_loop_text)

    def test_project_logs_record_scan_only_pr_fprg_rule(self):
        changelog_text = CHANGELOG_PATH.read_text(encoding="utf-8")
        knowledge_text = PR_FPRG_KNOWLEDGE_PATH.read_text(encoding="utf-8")

        self.assertIn(
            "视觉扫描算法复刻 2026-04-22 PR-FPRG",
            changelog_text,
        )
        self.assertIn(
            "扫描 S2 主链回到 depth-only 版本",
            changelog_text,
        )
        self.assertIn(
            "扫描视觉算法完全复刻 2026-04-22 版本",
            knowledge_text,
        )
        self.assertIn(
            "固定识别位姿扫描触发走 2026-04-22 `PR-FPRG` 拓扑恢复方案，执行层逐区到位后走平面分割 + Hough",
            changelog_text,
        )
        self.assertIn(
            "`MODE_EXECUTION_REFINE` 改走平面分割 + Hough 局部视觉",
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
        self.assertEqual(transform.child_frame_id, "surface_dp_bind_point_7")
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
            raw_bind_point_tf_child_prefix = "surface_dp_bind_point"

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
        self.assertIn('self.raw_bind_point_tf_child_prefix = "surface_dp_bind_point"', ros_interfaces_text)
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

    def test_realtime_result_image_omits_legacy_roi_rectangle(self):
        image_buffers_path = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "image_buffers.py"
        )
        image_buffers_text = image_buffers_path.read_text(encoding="utf-8")
        callback_start = image_buffers_text.index("def image_callback(self, msg):")
        callback_end = image_buffers_text.index("def camera_info_callback", callback_start)
        callback_text = image_buffers_text[callback_start:callback_end]

        self.assertNotIn("cv2.rectangle(result_image, self.point1, self.point2, 255, 2)", callback_text)

    def test_realtime_result_image_does_not_draw_workspace_or_legacy_roi_frames(self):
        image_buffers_path = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "image_buffers.py"
        )
        image_buffers_text = image_buffers_path.read_text(encoding="utf-8")
        callback_start = image_buffers_text.index("def image_callback(self, msg):")
        callback_end = image_buffers_text.index("def camera_info_callback", callback_start)
        callback_text = image_buffers_text[callback_start:callback_end]

        self.assertNotIn("cv2.rectangle(result_image, self.point1, self.point2, 255, 2)", callback_text)
        self.assertNotIn("self.draw_scan_workspace_overlay(result_image)", callback_text)

    def test_manual_workspace_s2_result_image_does_not_draw_manual_quad_frame(self):
        rendering_path = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "rendering.py"
        )
        rendering_text = rendering_path.read_text(encoding="utf-8")
        render_start = rendering_text.index("def render_manual_workspace_s2_result_image")
        render_end = rendering_text.index("def format_result_display_label", render_start)
        render_text = rendering_text[render_start:render_end]

        self.assertNotIn("cv2.polylines(result_image, [polygon_points], True, (220, 220, 220), 2)", render_text)

    def test_execution_refine_result_label_shows_tcp_jaw_coordinate(self):
        from tie_robot_perception.pointai import rendering
        from tie_robot_perception.pointai.constants import PROCESS_IMAGE_MODE_EXECUTION_REFINE

        class DummyProcessor:
            current_result_request_mode = PROCESS_IMAGE_MODE_EXECUTION_REFINE
            current_linear_module_position_mm = {"x": 300.0, "y": 0.0, "z": 100.0}

        original_converter = rendering.camera_coord_to_tcp_jaw_coord
        converter_calls = []

        def fake_converter(camera_coord, current_tcp_mm=None):
            converter_calls.append((camera_coord, current_tcp_mm))
            return [50.6, -2.3, 14.0]

        rendering.camera_coord_to_tcp_jaw_coord = fake_converter
        try:
            label = rendering.format_result_display_label(
                DummyProcessor(),
                3,
                [-65.6, 72.3, 854.0],
                "SEL",
            )
        finally:
            rendering.camera_coord_to_tcp_jaw_coord = original_converter

        self.assertEqual(label, "3, tcp=(50.6,-2.3,14.0), SEL")
        self.assertEqual(
            converter_calls,
            [([-65.6, 72.3, 854.0], {"x": 300.0, "y": 0.0, "z": 100.0})],
        )
        self.assertNotIn("cam=", label)

    def test_tcp_display_can_report_current_moving_tcp_relative_coordinate(self):
        from tie_robot_perception.pointai.tcp_display import camera_coord_to_tcp_jaw_coord

        config = {
            "translation_mm": {"x": 285.0, "y": 70.0, "z": 740.0},
            "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 3.141592653589793},
        }

        relative_coord = camera_coord_to_tcp_jaw_coord(
            [-65.6, 72.3, 854.0],
            config=config,
            current_tcp_mm={"x": 300.0, "y": 0.0, "z": 100.0},
        )

        self.assertEqual([round(value, 1) for value in relative_coord], [50.6, -2.3, 14.0])

    def test_tcp_display_vectorized_channels_match_scalar_coordinate_conversion(self):
        from tie_robot_perception.pointai.tcp_display import (
            camera_channels_to_tcp_jaw_channels,
            camera_coord_to_tcp_jaw_coord,
        )

        config = {
            "translation_mm": {"x": 285.0, "y": 70.0, "z": 740.0},
            "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 3.141592653589793},
        }
        x_channel = np.array([[285.0, -65.6]], dtype=np.float32)
        y_channel = np.array([[70.0, 72.3]], dtype=np.float32)
        z_channel = np.array([[740.0, 854.0]], dtype=np.float32)

        tcp_x, tcp_y, tcp_z = camera_channels_to_tcp_jaw_channels(
            x_channel,
            y_channel,
            z_channel,
            config=config,
        )

        expected_origin = camera_coord_to_tcp_jaw_coord([285.0, 70.0, 740.0], config=config)
        expected_point = camera_coord_to_tcp_jaw_coord([-65.6, 72.3, 854.0], config=config)
        self.assertEqual(
            [round(float(value), 1) for value in [tcp_x[0, 0], tcp_y[0, 0], tcp_z[0, 0]]],
            [round(value, 1) for value in expected_origin],
        )
        self.assertEqual(
            [round(float(value), 1) for value in [tcp_x[0, 1], tcp_y[0, 1], tcp_z[0, 1]]],
            [round(value, 1) for value in expected_point],
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

    def test_workspace_s2_scored_spacing_keeps_current_scale_dense_lattice(self):
        from tie_robot_perception.perception import workspace_s2

        self.assertEqual(workspace_s2.LEGACY_WORKSPACE_S2_PREFERRED_LATTICE_LINE_COUNT, 8)
        self.assertEqual(workspace_s2.LEGACY_WORKSPACE_S2_SCORE_TARGET_MIN_POINTS, 42)
        self.assertEqual(
            workspace_s2.select_workspace_s2_regular_lattice_line_rhos.__defaults__[2],
            10,
        )

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

        self.assertEqual(len(selected), 8)
        self.assertGreaterEqual(min(np.diff(selected)), 36.0)

    def test_workspace_s2_lattice_pruning_recovers_current_dense_pitch_candidates(self):
        from tie_robot_perception.perception import workspace_s2

        current_dense_candidates = [
            -282.5,
            -232.3253292143345,
            -219.22988003492355,
            -210.95329166203737,
            -198.16945579648018,
            -184.61215868592262,
            -170.8525856435299,
            -130.71272912621498,
            -99.45786380767822,
            -89.47286593914032,
            -53.171943947672844,
            -30.046466380357742,
            -8.388371169567108,
            9.421881198883057,
            27.910356298089027,
        ]

        selected = workspace_s2.prune_workspace_s2_line_rhos_by_scored_spacing(
            current_dense_candidates,
            min_spacing_ratio=0.60,
        )

        self.assertEqual(len(selected), 8)
        self.assertGreaterEqual(float(np.min(np.diff(selected))), 36.0)

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

    def test_workspace_s2_axis_aligned_line_families_use_fixed_row_column_peaks(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((120, 100), dtype=np.float32)
        for x_position in (20, 50, 80):
            response[:, x_position] = 1.0
        for y_position in (15, 45, 75, 105):
            response[y_position, :] = 1.0
        workspace_mask = np.ones_like(response, dtype=np.uint8)

        families = workspace_s2.build_workspace_s2_axis_aligned_line_families(
            response,
            workspace_mask,
            min_period=20,
            max_period=40,
            enable_structural_edge_suppression=False,
        )

        self.assertEqual([family["line_angle_deg"] for family in families], [0.0, 90.0])
        self.assertEqual([family["axis_orientation"] for family in families], ["horizontal", "vertical"])
        self.assertEqual(families[0]["normal"], [0.0, 1.0])
        self.assertEqual(families[1]["normal"], [1.0, 0.0])
        self.assertEqual([round(rho) for rho in families[0]["line_rhos"]], [15, 45, 75, 105])
        self.assertEqual([round(rho) for rho in families[1]["line_rhos"]], [20, 50, 80])

    def test_workspace_s2_fft_period_estimate_recovers_profile_peaks(self):
        from tie_robot_perception.perception import workspace_s2

        profile = np.full(180, 0.08, dtype=np.float32)
        for position in range(9, profile.size, 24):
            profile[position] = 1.0
            if position + 1 < profile.size:
                profile[position + 1] = 0.62
        profile += (0.015 * np.sin(np.arange(profile.size, dtype=np.float32) * 0.31)).astype(np.float32)

        estimate = workspace_s2.estimate_workspace_s2_fft_period_and_phase(
            profile,
            min_period=18,
            max_period=30,
        )

        self.assertIsNotNone(estimate)
        self.assertEqual(estimate["method"], "fft")
        self.assertAlmostEqual(float(estimate["period"]), 24.0, delta=1.0)
        self.assertAlmostEqual(float(estimate["phase"]), 9.0, delta=1.0)
        self.assertGreater(int(estimate["frequency_bin"]), 0)

    def test_workspace_s2_axis_aligned_line_families_can_use_fft_estimator(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((132, 126), dtype=np.float32)
        for x_position in (18, 42, 66, 90, 114):
            response[:, x_position] = 1.0
        for y_position in (9, 33, 57, 81, 105, 129):
            response[y_position, :] = 1.0
        workspace_mask = np.ones_like(response, dtype=np.uint8)

        families = workspace_s2.build_workspace_s2_axis_aligned_line_families(
            response,
            workspace_mask,
            min_period=18,
            max_period=30,
            period_estimator="fft",
            enable_structural_edge_suppression=False,
        )

        self.assertEqual([family["line_angle_deg"] for family in families], [0.0, 90.0])
        self.assertEqual([family["estimate"]["method"] for family in families], ["fft", "fft"])
        self.assertEqual([round(rho) for rho in families[0]["line_rhos"]], [9, 33, 57, 81, 105, 129])
        self.assertEqual([round(rho) for rho in families[1]["line_rhos"]], [18, 42, 66, 90, 114])

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
        _draw_synthetic_line_family(response, 90.0, [-14.0, -20.0, -26.0], sigma_px=2.0, amp=1.4)
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

        self.assertGreater(float(np.mean(beam_mask[:, 14:28])), 0.65)
        self.assertGreater(float(np.mean(beam_mask[:, 206:222])), 0.65)
        self.assertLess(float(np.mean(beam_mask[:, 187:194])), 0.20)
        self.assertLess(float(np.mean(beam_mask[:, 55:64])), 0.20)

    def test_workspace_s2_structural_edge_detection_reports_beam_band_location(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((160, 230), dtype=np.float32)
        _draw_synthetic_line_family(response, 90.0, [-14.0, -20.0, -26.0], sigma_px=2.0, amp=1.4)
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
        left_vertical_bands = [
            band for band in beam_bands
            if band["axis"] == "x" and band["side"] == "min"
        ]
        self.assertTrue(left_vertical_bands)
        self.assertLessEqual(left_vertical_bands[0]["start"], 16)
        self.assertTrue(right_vertical_bands)
        self.assertGreaterEqual(right_vertical_bands[0]["start"], 198)
        self.assertGreaterEqual(right_vertical_bands[0]["peak"], 1.4)

    def test_workspace_s2_structural_edge_detection_catches_live_like_side_beams(self):
        from tie_robot_perception.perception import workspace_s2

        response = np.zeros((371, 329), dtype=np.float32)
        _draw_synthetic_line_family(response, 0.0, [40.0, 110.0, 180.0, 250.0, 320.0], sigma_px=1.1, amp=1.0)
        _draw_synthetic_line_family(
            response,
            90.0,
            [-58.0, -100.0, -149.0, -199.0, -247.0, -284.0, -298.0],
            sigma_px=1.1,
            amp=1.0,
        )
        _draw_synthetic_line_family(response, 90.0, [-23.0], sigma_px=1.6, amp=1.4)
        _draw_synthetic_line_family(response, 90.0, [-314.0], sigma_px=1.6, amp=1.5)
        mask = np.ones_like(response, dtype=np.uint8)

        beam_bands = workspace_s2.detect_workspace_s2_structural_edge_bands(response, mask)

        self.assertTrue([
            band for band in beam_bands
            if band["axis"] == "x" and band["side"] == "min" and band["start"] <= 18
        ])
        self.assertTrue([
            band for band in beam_bands
            if band["axis"] == "x" and band["side"] == "max" and band["start"] >= 300
        ])

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

    def test_workspace_s2_expands_beam_mask_by_thirteen_centimeters_for_graph_exclusion(self):
        from tie_robot_perception.perception import workspace_s2

        beam_mask = np.zeros((80, 120), dtype=bool)
        beam_mask[:, 50:56] = True
        rectified_geometry = {
            "rectified_width": 120,
            "rectified_height": 80,
            "resolution_mm_per_px": 5.0,
        }

        graph_exclusion_mask = workspace_s2.expand_workspace_s2_exclusion_mask_by_metric_margin(
            beam_mask,
            rectified_geometry,
            margin_mm=130.0,
        )
        filtered_points = workspace_s2.filter_workspace_s2_rectified_points_outside_mask(
            [
                (23.0, 40.0),
                (24.0, 40.0),
                (81.0, 40.0),
                (82.0, 40.0),
            ],
            graph_exclusion_mask,
        )

        self.assertFalse(beam_mask[40, 24])
        self.assertTrue(graph_exclusion_mask[40, 24])
        self.assertTrue(graph_exclusion_mask[40, 81])
        self.assertFalse(graph_exclusion_mask[40, 23])
        self.assertFalse(graph_exclusion_mask[40, 82])
        self.assertEqual(filtered_points, [(23.0, 40.0), (82.0, 40.0)])

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

    def test_workspace_s2_family_score_targets_current_dense_lattice(self):
        from tie_robot_perception.perception import workspace_s2

        sparse_0600_family = {
            "line_rhos": [-263.0, -172.0, -84.0],
            "periodic_score": 1.0,
            "orientation_score": 0.8,
        }
        current_dense_family = {
            "line_rhos": [-282.5, -232.3, -184.6, -130.7, -89.5, -53.2, -30.0, 9.4],
            "periodic_score": 1.0,
            "orientation_score": 0.8,
        }

        self.assertGreater(
            workspace_s2._workspace_s2_supported_family_score(current_dense_family),
            workspace_s2._workspace_s2_supported_family_score(sparse_0600_family),
        )

    def test_workspace_s2_pair_score_prefers_current_dense_target_over_sparse_lattice(self):
        from tie_robot_perception.perception import workspace_s2

        sparse_0600_pair = [
            {
                "line_angle_deg": 88.0,
                "line_rhos": [-263.0, -172.0, -84.0],
                "periodic_score": 1.0,
                "orientation_score": 0.8,
            },
            {
                "line_angle_deg": 178.0,
                "line_rhos": [-297.0, -206.0, -121.0, -14.0],
                "periodic_score": 1.0,
                "orientation_score": 0.8,
            },
        ]
        current_dense_pair = [
            {
                "line_angle_deg": 84.0,
                "line_rhos": [-282.5, -232.3, -184.6, -130.7, -89.5, -53.2, -30.0, 9.4],
                "periodic_score": 1.0,
                "orientation_score": 0.8,
            },
            {
                "line_angle_deg": 174.0,
                "line_rhos": [-396.5, -337.5, -267.8, -210.0, -150.5, -88.9, -42.0, 14.0],
                "periodic_score": 1.0,
                "orientation_score": 0.8,
            },
        ]

        self.assertGreater(
            workspace_s2.score_workspace_s2_oriented_line_family_result(current_dense_pair),
            workspace_s2.score_workspace_s2_oriented_line_family_result(sparse_0600_pair),
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

    def test_manual_workspace_s2_response_variant_selection_replicates_april22_depth_only_mainline(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        depth_block_index = manual_workspace_s2_text.index("depth_response_variants = [")
        depth_loop_index = manual_workspace_s2_text.index("for response_variant in depth_response_variants:")

        self.assertLess(depth_block_index, depth_loop_index)
        self.assertIn("vertical_estimate = self.estimate_workspace_s2_period_and_phase(", manual_workspace_s2_text)
        self.assertIn("horizontal_estimate = self.estimate_workspace_s2_period_and_phase(", manual_workspace_s2_text)
        self.assertNotIn("combined_response_variants", manual_workspace_s2_text)
        self.assertNotIn("combined_response =", manual_workspace_s2_text)
        self.assertNotIn("infrared_response_variants", manual_workspace_s2_text)
        self.assertNotIn("infrared_dark_line_response", manual_workspace_s2_text)

    def test_manual_workspace_s2_selects_highest_scored_depth_response_variant_like_april22(self):
        from tie_robot_perception.pointai import manual_workspace_s2

        class DummyProcessor:
            def __init__(self):
                self.depth_v = np.arange(400, dtype=np.float32).reshape(20, 20) + 100.0
                self.variant_counter = 0

            def load_manual_workspace_quad(self):
                return {
                    "corner_pixels": [[0, 0], [19, 0], [19, 19], [0, 19]],
                    "corner_world_camera_frame": [
                        [0.0, 0.0, 100.0],
                        [100.0, 0.0, 100.0],
                        [100.0, 100.0, 100.0],
                        [0.0, 100.0, 100.0],
                    ],
                }

            def get_manual_workspace_pixel_mask(self):
                return np.ones((20, 20), dtype=np.uint8)

            def ensure_raw_world_channels(self):
                return True

            def build_workspace_s2_rectified_geometry(self, corner_pixels, corner_world):
                del corner_pixels, corner_world
                return {
                    "rectified_width": 20,
                    "rectified_height": 20,
                    "forward_h": np.eye(3, dtype=np.float32),
                    "inverse_h": np.eye(3, dtype=np.float32),
                }

            def normalize_workspace_s2_response(self, response_variant, valid_mask):
                del response_variant, valid_mask
                self.variant_counter += 1
                return np.full((20, 20), float(self.variant_counter), dtype=np.float32)

            def build_workspace_s2_axis_profile(self, normalized_response, mask, axis):
                del mask, axis
                return np.full(20, float(normalized_response[0, 0]), dtype=np.float32)

            def estimate_workspace_s2_period_and_phase(self, profile, min_period=10, max_period=30):
                del min_period, max_period
                marker = float(profile[0])
                return {"period": 10, "phase": 0, "score": marker}

            def build_workspace_s2_bbox(self, workspace_mask):
                del workspace_mask
                return (0, 0, 19, 19)

        result = manual_workspace_s2.prepare_manual_workspace_s2_inputs(DummyProcessor())

        self.assertIsNotNone(result)
        self.assertEqual(float(result["response_crop"][0, 0]), 2.0)
        self.assertEqual(float(result["vertical_estimate"]["score"]), 2.0)
        self.assertEqual(float(result["horizontal_estimate"]["score"]), 2.0)

    def test_manual_workspace_s2_rectification_prefers_map_corner_geometry(self):
        from tie_robot_perception.pointai import manual_workspace_s2

        map_corners = [
            [10.0, 20.0, 30.0],
            [110.0, 20.0, 30.0],
            [110.0, 120.0, 30.0],
            [10.0, 120.0, 30.0],
        ]
        camera_corners = [
            [1.0, 2.0, 3.0],
            [101.0, 2.0, 3.0],
            [101.0, 102.0, 3.0],
            [1.0, 102.0, 3.0],
        ]

        class DummyProcessor:
            def __init__(self):
                self.depth_v = np.ones((20, 20), dtype=np.float32)
                self.geometry_corner_world = None

            def load_manual_workspace_quad(self):
                return {
                    "corner_pixels": [[0, 0], [19, 0], [19, 19], [0, 19]],
                    "corner_world_map_frame": map_corners,
                    "corner_world_camera_frame": camera_corners,
                }

            def get_manual_workspace_pixel_mask(self):
                return np.ones((20, 20), dtype=np.uint8)

            def ensure_raw_world_channels(self):
                return True

            def build_workspace_s2_rectified_geometry(self, corner_pixels, corner_world):
                del corner_pixels
                self.geometry_corner_world = corner_world
                return None

        dummy = DummyProcessor()
        result = manual_workspace_s2.prepare_manual_workspace_s2_inputs(dummy)

        self.assertIsNone(result)
        self.assertEqual(dummy.geometry_corner_world, map_corners)

    def test_manual_workspace_s2_current_chain_rejects_depth_only_fallback(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        pipeline_index = manual_workspace_s2_text.index("def run_manual_workspace_s2_pipeline(self, publish=False):")
        pipeline_body = manual_workspace_s2_text[pipeline_index:manual_workspace_s2_text.index("def run_manual_workspace_s2(self):", pipeline_index)]
        fallback_index = manual_workspace_s2_text.index("def run_manual_workspace_s2_depth_only_pipeline(self, publish=False):")
        fallback_body = manual_workspace_s2_text[fallback_index:pipeline_index]

        self.assertIn("self.run_manual_workspace_surface_dp_pipeline(publish=publish)", pipeline_body)
        self.assertNotIn("self.run_manual_workspace_s2_depth_only_pipeline(publish=publish)", pipeline_body)
        self.assertNotIn("fallback_result", pipeline_body)
        self.assertIn('surface_result["legacy_depth_only_fallback"] = False', pipeline_body)
        self.assertIn("vertical_lines = self.build_workspace_s2_line_positions(", fallback_body)
        self.assertIn("horizontal_lines = self.build_workspace_s2_line_positions(", fallback_body)
        self.assertIn("rectified_intersections = [", fallback_body)
        self.assertIn("self.build_workspace_s2_projective_line_segments(", fallback_body)
        self.assertIn("v_period=%d, h_period=%d, points=%d", fallback_body)
        self.assertNotIn("collect_stable_manual_workspace_s2_inputs", pipeline_body)
        self.assertNotIn("apply_manual_workspace_s2_phase_lock", pipeline_body)
        self.assertNotIn("beam_exclusion_margin_mm", pipeline_body)
        self.assertNotIn("expand_workspace_s2_exclusion_mask_by_metric_margin", pipeline_body)

    def test_manual_workspace_s2_module_omits_later_stability_and_phase_lock_experiments(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        self.assertNotIn("select_stable_manual_workspace_s2_inputs", manual_workspace_s2_text)
        self.assertNotIn("collect_stable_manual_workspace_s2_inputs", manual_workspace_s2_text)
        self.assertNotIn("manual_workspace_s2_phase_lock", manual_workspace_s2_text)
        self.assertNotIn("apply_manual_workspace_s2_phase_lock", manual_workspace_s2_text)

    def test_manual_workspace_s2_scan_path_omits_beam_expansion_filtering(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        self.assertNotIn('"structural_edge_response_crop": structural_edge_response', manual_workspace_s2_text)
        self.assertNotIn('structural_edge_response = best_variant["response"]', manual_workspace_s2_text)
        self.assertNotIn("beam_exclusion_margin_mm = 130.0", manual_workspace_s2_text)
        self.assertNotIn("expand_workspace_s2_exclusion_mask_by_metric_margin", manual_workspace_s2_text)
        self.assertNotIn('s2_inputs.get("structural_edge_response_crop", s2_inputs["response_crop"])', manual_workspace_s2_text)

    def test_stage_ablation_filters_beam_points_with_structural_edge_response(self):
        stage_ablation_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_stage_ablation.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"structural_edge_response": structural_edge_response', stage_ablation_text)
        self.assertIn('prepared.get("structural_edge_response", best_result["response"])', stage_ablation_text)
        self.assertIn("expand_workspace_s2_exclusion_mask_by_metric_margin", stage_ablation_text)
        self.assertIn("margin_mm=130.0", stage_ablation_text)

    def test_stage_ablation_prefers_first_supported_combined_response(self):
        stage_ablation_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_stage_ablation.py"
        ).read_text(encoding="utf-8")

        self.assertIn("combined_depth_ir_darkline", stage_ablation_text)
        self.assertLess(
            stage_ablation_text.index('"name": "combined_depth_ir_darkline"'),
            stage_ablation_text.index('"name": "depth_background_minus_filled"'),
        )
        self.assertIn('candidate["source"] == "depth_ir"', stage_ablation_text)
        self.assertIn("selected_group_result = None", stage_ablation_text)
        self.assertIn("selected_group_result = {", stage_ablation_text)
        self.assertIn("if selected_group_result is not None:\n                break", stage_ablation_text)

    def test_peak_supported_probe_prefers_first_supported_depth_response(self):
        probe_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_peak_supported_probe.py"
        ).read_text(encoding="utf-8")
        selector_body = probe_text[
            probe_text.index("def select_best_response_variant("):
            probe_text.index("def run_peak_supported_pr_fprg(", probe_text.index("def select_best_response_variant("))
        ]

        self.assertIn("return {", selector_body)
        self.assertNotIn("combined_score > best_variant", selector_body)
        self.assertNotIn("best_variant = {", selector_body)

    def test_peak_supported_probe_uses_combined_response_before_depth_for_scheme1_peak_finding(self):
        probe_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_peak_supported_probe.py"
        ).read_text(encoding="utf-8")

        self.assertIn("combined_depth_ir_darkline", probe_text)
        self.assertIn("response_name_filter", probe_text)
        self.assertLess(
            probe_text.index('"name": "combined_depth_ir_darkline"'),
            probe_text.index('"name": "depth_background_minus_filled"'),
        )

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

    def test_pr_fprg_scheme_ablation_report_covers_active_and_curve_schemes_without_scheme2(self):
        report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        schemes_block = report_text[report_text.index("SCHEMES = ["):report_text.index("STAGE_VARIANTS = [")]
        self.assertIn("01_current_theta_rho", schemes_block)
        self.assertNotIn("02_archived_ir_rho", schemes_block)
        for scheme_id in ("03_greedy_depth_curve", "04_dp_depth_curve", "05_dp_ridge_curve", "06_ir_assisted_curve"):
            self.assertIn(scheme_id, schemes_block)

    def test_pr_fprg_scheme_ablation_report_crosses_schemes_with_stage_variants(self):
        report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        self.assertIn("for scheme in SCHEMES:", report_text)
        self.assertIn("for stage_variant in STAGE_VARIANTS:", report_text)
        self.assertIn("scheme_id", report_text)
        self.assertIn("stage_variant_id", report_text)
        self.assertIn("同一组合响应图", report_text)
        self.assertIn("梁筋±13cm范围", report_text)
        self.assertIn("00_peak_supported_lines.png", report_text)
        self.assertIn("00_peak_spacing_pruned_lines.png", report_text)
        self.assertNotIn("full_angle_sweep", report_text)

    def test_pr_fprg_scheme_ablation_report_has_visible_scheme_navigation(self):
        report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        self.assertIn("scheme-nav", report_text)
        self.assertIn("data-scheme-filter", report_text)
        self.assertIn("showScheme", report_text)

    def test_pr_fprg_scheme_ablation_report_does_not_render_failure_placeholder_images(self):
        report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        self.assertNotIn("render_failure_image", report_text)
        self.assertIn("failure-panel", report_text)

    def test_pr_fprg_scheme_comparison_reports_beam_filter_and_curve_metrics(self):
        comparison_text = PR_FPRG_SCHEME_COMPARISON_PATH.read_text(encoding="utf-8")

        self.assertIn("build_workspace_s2_structural_edge_suppression_mask", comparison_text)
        self.assertIn("detect_workspace_s2_structural_edge_bands", comparison_text)
        self.assertIn("expand_workspace_s2_exclusion_mask_by_metric_margin", comparison_text)
        self.assertIn("beam_exclusion_margin_mm = 130.0", comparison_text)
        self.assertIn("filter_workspace_s2_rectified_points_outside_mask", comparison_text)
        self.assertIn("beam_bands", comparison_text)
        self.assertIn("梁筋±13cm图谱排除 mask", comparison_text)
        self.assertIn("beam_filtered_point_count", comparison_text)
        self.assertIn("curve_metrics", comparison_text)
        self.assertIn("abs_offset_p95", comparison_text)
        self.assertIn("wiggle_mean", comparison_text)
        self.assertIn('"01b_fft_axis_peaks"', comparison_text)
        self.assertIn("FFT 行/列峰值正交网格", comparison_text)
        self.assertIn("period_estimator=\"fft\"", comparison_text)
        self.assertIn("00_combined_response_beam_mask_overlay.png", comparison_text)
        self.assertIn("组合响应 + 梁筋±13cm过滤叠加", comparison_text)
        self.assertIn("00_beam_mask_binary.png", comparison_text)
        self.assertIn("梁筋mask二值图", comparison_text)
        self.assertIn("00_fft_frequency_spectrum.png", comparison_text)
        self.assertIn("FFT 频域谱图", comparison_text)
        self.assertIn("直角坐标系驼峰峰值图", comparison_text)
        self.assertIn("period_px", comparison_text)
        self.assertIn("display_gamma", comparison_text)
        self.assertIn("00_source_workspace_gamma.png", comparison_text)
        self.assertIn("00_selected_response_gamma.png", comparison_text)
        self.assertIn("00_combined_response_gamma.png", comparison_text)
        self.assertIn("高伽马组合响应", comparison_text)
        self.assertIn("梁筋 mask 当前状态", comparison_text)
        self.assertIn("曲线为什么没有完全贴钢筋", comparison_text)
        self.assertIn("尺度变化为什么影响精度", comparison_text)
        self.assertIn("全尺度鲁棒方案", comparison_text)
        self.assertIn("历史技术路径", comparison_text)
        self.assertIn("box-sizing: border-box", comparison_text)
        self.assertIn("overflow-x: hidden", comparison_text)
        self.assertIn("minmax(min(100%, 320px), 1fr)", comparison_text)
        self.assertIn("min-width: 0", comparison_text)
        self.assertIn("max-height: min(76vh, 920px)", comparison_text)
        self.assertIn("object-fit: contain", comparison_text)
        self.assertIn('"topology_source": variant.get("topology_source")', comparison_text)

    def test_pr_fprg_previous_schemes_are_archived_before_instance_graph_probe(self):
        archive_text = PR_FPRG_PREVIOUS_SCHEMES_ARCHIVE_PATH.read_text(encoding="utf-8")

        self.assertIn("方案 1", archive_text)
        self.assertIn("方案 2 废案", archive_text)
        self.assertIn("方案 3/4/5/6", archive_text)
        self.assertIn("红外最终 rho 微调", archive_text)
        self.assertIn("梁筋 edge-band mask", archive_text)
        self.assertIn("不是实例分割", archive_text)
        self.assertIn("组合响应", archive_text)
        self.assertIn("深度响应", archive_text)
        self.assertIn("主链不回滚", archive_text)

    def test_rebar_instance_graph_probe_derives_modalities_from_combined_and_depth_response(self):
        probe_text = REBAR_INSTANCE_GRAPH_PROBE_PATH.read_text(encoding="utf-8")

        self.assertIn("combined_response", probe_text)
        self.assertIn("depth_response", probe_text)
        self.assertIn("hessian", probe_text)
        self.assertIn("ridge", probe_text)
        self.assertIn("skeleton", probe_text)
        self.assertIn("instance_graph", probe_text)
        self.assertIn("worldCoord", probe_text)
        self.assertIn("beam_candidate", probe_text)
        self.assertIn("write_report", probe_text)
        self.assertIn("07_skeleton.png", probe_text)
        self.assertIn("08_instance_graph_overlay.png", probe_text)
        comparison_text = PR_FPRG_SCHEME_COMPARISON_PATH.read_text(encoding="utf-8")
        self.assertIn('"main_axis_rowcol"', comparison_text)
        self.assertIn('"fft_axis_peaks"', comparison_text)
        self.assertIn("方案3/4/5/6 当前使用主链行/列峰值线族作为曲线拓扑骨架，不使用 FFT 线族。", comparison_text)
        for scheme_id in (
            "03f_fft_greedy_depth_curve",
            "04f_fft_dp_depth_curve",
            "05f_fft_dp_ridge_curve",
            "06f_fft_ir_assisted_curve",
        ):
            self.assertIn(scheme_id, comparison_text)
        self.assertIn("FFT 线族拓扑骨架", comparison_text)
        self.assertNotIn('"02_archived_ir_rho"', comparison_text)
        self.assertNotIn("02 归档方案", comparison_text)

    def test_rebar_surface_bindpoint_comparison_reports_segmentation_and_variants(self):
        comparison_text = REBAR_SURFACE_BINDPOINT_COMPARISON_PATH.read_text(encoding="utf-8")

        self.assertIn("surface_segmentation", comparison_text)
        self.assertIn("ordinary_rebar_mask", comparison_text)
        self.assertIn("beam_candidate_mask", comparison_text)
        self.assertIn("beam_candidate_mask_13cm", comparison_text)
        self.assertIn("bind_point_variants", comparison_text)
        self.assertIn("pr_fprg_all", comparison_text)
        self.assertIn("legacy_edge_band_13cm", comparison_text)
        self.assertIn("beam_candidate_direct", comparison_text)
        self.assertIn("beam_candidate_13cm", comparison_text)
        self.assertIn("ir_display_gamma", comparison_text)
        self.assertIn("ir_high_gamma", comparison_text)
        self.assertIn("completed_surface_mask", comparison_text)
        self.assertIn("completed_surface_intersections", comparison_text)
        self.assertIn("instance_graph_junctions", comparison_text)
        self.assertIn("curve3456_on_completed_surface", comparison_text)
        for scheme_id in (
            "03_surface_greedy_curve",
            "04_surface_dp_curve",
            "05_surface_ridge_curve",
            "06_surface_ir_assisted_curve",
        ):
            self.assertIn(scheme_id, comparison_text)
        self.assertIn("variant_timings_ms", comparison_text)
        self.assertIn("write_report", comparison_text)
        self.assertIn("surface_segmentation_overlay.png", comparison_text)
        self.assertIn("surface_completion_overlay.png", comparison_text)
        self.assertIn("ir_high_gamma_workspace.png", comparison_text)
        self.assertIn("bindpoint_comparison_matrix.png", comparison_text)

    def test_pr_fprg_topology_recovery_experiment_reports_each_algorithm_step(self):
        report_text = PR_FPRG_TOPOLOGY_RECOVERY_EXPERIMENT_PATH.read_text(encoding="utf-8")

        self.assertIn("build_recovered_frequency_phase_grid", report_text)
        self.assertIn("enable_peak_support=False", report_text)
        self.assertIn("period_estimator=\"fft\"", report_text)
        self.assertIn("recovered_topology_before_filter", report_text)
        self.assertIn("recovered_topology_after_beam_filter", report_text)
        self.assertIn("beam_candidate_mask_13cm", report_text)
        self.assertIn("01_input_workspace.png", report_text)
        self.assertIn("02_rectified_ir.png", report_text)
        self.assertIn("03_combined_response.png", report_text)
        self.assertIn("04_vertical_profile_fft.png", report_text)
        self.assertIn("05_horizontal_profile_fft.png", report_text)
        self.assertIn("06_frequency_phase_lattice_rectified.png", report_text)
        self.assertIn("07_beam_candidate_13cm_mask.png", report_text)
        self.assertIn("08_before_filter_original.png", report_text)
        self.assertIn("09_after_filter_original.png", report_text)
        self.assertIn("10_current_peak_supported_original.png", report_text)
        self.assertIn("build_curve3456_on_recovered_mainline", report_text)
        for scheme_id in (
            "03_recovered_greedy_curve",
            "04_recovered_dp_curve",
            "05_recovered_ridge_curve",
            "06_recovered_ir_assisted_curve",
        ):
            self.assertIn(scheme_id, report_text)
        self.assertIn("12_03_recovered_greedy_curve_original.png", report_text)
        self.assertIn("13_04_recovered_dp_curve_original.png", report_text)
        self.assertIn("14_05_recovered_ridge_curve_original.png", report_text)
        self.assertIn("15_06_recovered_ir_assisted_curve_original.png", report_text)
        self.assertIn("write_report", report_text)

    def test_pr_fprg_multiscale_experiment_fuses_frequency_surface_graph_and_curve_paths(self):
        report_text = PR_FPRG_MULTISCALE_BINDPOINT_EXPERIMENT_PATH.read_text(encoding="utf-8")

        self.assertIn("PR-FPRG-MS", report_text)
        self.assertIn("def compute_multiscale_scale_metrics", report_text)
        self.assertIn("def merge_multiscale_bindpoint_candidates", report_text)
        self.assertIn("def build_multiscale_bindpoint_experiment", report_text)
        self.assertIn("build_recovered_frequency_phase_grid", report_text)
        self.assertIn("build_surface_segmentation", report_text)
        self.assertIn("build_completed_surface_intersections", report_text)
        self.assertIn("cluster_instance_graph_junctions", report_text)
        self.assertIn("build_curve3456_on_recovered_mainline", report_text)
        self.assertIn("beam_candidate_mask_13cm", report_text)
        self.assertIn("scale_mode", report_text)
        self.assertIn("frequency_phase_after_beam", report_text)
        self.assertIn("completed_surface_intersections_13cm", report_text)
        self.assertIn("instance_graph_junctions_13cm", report_text)
        self.assertIn("multiscale_fused", report_text)
        self.assertIn("04_scale_decision_flow.png", report_text)
        self.assertIn("15_multiscale_fused_original.png", report_text)
        self.assertIn("reports/pr_fprg_multiscale_bindpoint_experiment", report_text)

    def test_pr_fprg_reports_label_bottom_response_as_combined_response(self):
        comparison_text = PR_FPRG_SCHEME_COMPARISON_PATH.read_text(encoding="utf-8")
        probe_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_peak_supported_probe.py"
        ).read_text(encoding="utf-8")

        self.assertIn("00_selected_response.png", comparison_text)
        self.assertIn("00_peak_supported_lines.png", comparison_text)
        self.assertIn("00_peak_spacing_pruned_lines.png", comparison_text)
        self.assertIn("峰值图：peak-supported 候选线", comparison_text)
        self.assertIn("峰值图：绿色保留 / 红色剔除", comparison_text)
        self.assertIn("底层组合响应", comparison_text)
        self.assertIn("底层深度响应", comparison_text)
        self.assertIn("--response-name-filter", comparison_text)
        self.assertIn("04_selected_combined_response", probe_text)
        self.assertNotIn("<figcaption>深度响应</figcaption>", comparison_text)

    def test_pr_fprg_stage_tools_default_full_pipeline_uses_axis_peak_mainline(self):
        stage_ablation_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_stage_ablation.py"
        ).read_text(encoding="utf-8")
        stage_report_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "tools"
            / "pr_fprg_stage_ablation_report.py"
        ).read_text(encoding="utf-8")
        scheme_report_text = PR_FPRG_SCHEME_ABLATION_REPORT_PATH.read_text(encoding="utf-8")

        self.assertIn('"enable_local_peak_refine": False', scheme_report_text)
        self.assertIn('"enable_continuous_validation": False', scheme_report_text)
        self.assertIn('variant.get("enable_local_peak_refine", False)', stage_ablation_text)
        self.assertIn('variant.get("enable_continuous_validation", False)', stage_ablation_text)
        self.assertIn("00_peak_supported_lines.png", stage_report_text)
        self.assertIn("00_peak_spacing_pruned_lines.png", stage_report_text)
        self.assertNotIn("full_angle_sweep", stage_ablation_text)
        self.assertNotIn("全角度候选池", scheme_report_text)

    def test_visual_debug_runtime_controls_are_exposed_to_frontend(self):
        ros_interfaces_text = POINTAI_ROS_INTERFACES_PATH.read_text(encoding="utf-8")
        runtime_config_text = POINTAI_RUNTIME_CONFIG_PATH.read_text(encoding="utf-8")

        self.assertIn("from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32", ros_interfaces_text)
        self.assertIn("'/web/pointAI/set_stable_frame_count'", ros_interfaces_text)
        self.assertIn("self.set_stable_frame_count_callback", ros_interfaces_text)
        self.assertIn("def set_stable_frame_count_callback(self, msg):", runtime_config_text)
        self.assertIn("self.stable_frame_count = max(1, int(getattr(msg, \"data\", 1)))", runtime_config_text)
        self.assertIn('rospy.set_param("~stable_frame_count", int(self.stable_frame_count))', runtime_config_text)

    def test_colab_training_package_includes_full_non_rgb_modality_set(self):
        colab_script = WORKSPACE_ROOT.parent / "notebooks" / "pr_fprg_multimodal_segmentation_colab.py"
        script_text = colab_script.read_text(encoding="utf-8")

        expected_input_channels = (
            '"ir"',
            '"depth_z"',
            '"worldcoord_height_response"',
            '"depth_response"',
            '"depth_gradient"',
            '"combined_response"',
            '"hessian_ridge"',
            '"frangi_like"',
            '"fused_instance_response"',
        )
        for channel_name in expected_input_channels:
            self.assertIn(channel_name, script_text)

        self.assertIn('"rectified_ir"', script_text)
        self.assertIn('"rectified_depth"', script_text)
        self.assertIn('"raw_world_z"', script_text)
        self.assertIn("TRAINING_MASK_CHANNELS", script_text)
        self.assertIn('"valid_mask"', script_text)
        self.assertIn('"workspace_mask"', script_text)
        self.assertIn("apply_training_support_masks", script_text)
        self.assertIn("target[ignore_mask] = IGNORE_INDEX", script_text)
        self.assertNotIn("建议第一版先用这 6 个", script_text)

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

    def test_manual_workspace_s2_pipeline_uses_surface_dp_as_main_path(self):
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")

        pipeline_index = manual_workspace_s2_text.index("def run_manual_workspace_s2_pipeline(self, publish=False):")
        pipeline_body = manual_workspace_s2_text[pipeline_index:manual_workspace_s2_text.index("def run_manual_workspace_s2(self):", pipeline_index)]

        self.assertIn("self.run_manual_workspace_surface_dp_pipeline(publish=publish)", pipeline_body)
        self.assertNotIn("self.run_manual_workspace_s2_depth_only_pipeline(publish=publish)", pipeline_body)
        self.assertIn('surface_result["legacy_depth_only_fallback"] = False', pipeline_body)
        self.assertNotIn("vertical_lines = self.build_workspace_s2_line_positions(", pipeline_body)
        self.assertNotIn("horizontal_lines = self.build_workspace_s2_line_positions(", pipeline_body)
        self.assertNotIn("rectified_intersections = [", pipeline_body)

    def test_execution_refine_hough_is_runtime_bound_without_importing_legacy_pre_img(self):
        processor_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "processor.py"
        ).read_text(encoding="utf-8")
        process_image_service_text = PROCESS_IMAGE_SERVICE_PATH.read_text(encoding="utf-8")
        ros_interfaces_text = POINTAI_ROS_INTERFACES_PATH.read_text(encoding="utf-8")
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
        self.assertIn("from . import execution_refine_hough", processor_text)
        self.assertIn(
            "cls.run_execution_refine_hough_pipeline = execution_refine_hough.run_execution_refine_hough_pipeline",
            processor_text,
        )
        self.assertIn("self.depth_binary_image_pub", ros_interfaces_text)
        self.assertIn("self.line_image_pub", ros_interfaces_text)
        self.assertIn("HoughLinesP", active_pointai_sources)
        self.assertIn("self.run_execution_refine_hough_pipeline(", process_image_service_text)
        self.assertNotIn("cls.pre_img", processor_text)
        self.assertNotIn("self.pre_img(", process_image_service_text)
        self.assertNotIn("from .matrix_preprocess import pre_img", active_pointai_sources)

    def test_scan_and_execution_base_images_are_published_for_frontend_image_layer(self):
        ros_interfaces_text = POINTAI_ROS_INTERFACES_PATH.read_text(encoding="utf-8")
        processor_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "processor.py"
        ).read_text(encoding="utf-8")
        manual_workspace_s2_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "manual_workspace_s2.py"
        ).read_text(encoding="utf-8")
        execution_refine_hough_text = EXECUTION_REFINE_HOUGH_PATH.read_text(encoding="utf-8")

        self.assertIn("'/perception/lashing/scan_surface_dp_base_image'", ros_interfaces_text)
        self.assertIn("'/perception/lashing/scan_surface_dp_completed_surface_image'", ros_interfaces_text)
        self.assertIn("'/perception/lashing/execution_refine_base_image'", ros_interfaces_text)
        self.assertIn(
            "cls.publish_scan_surface_dp_base_images = manual_workspace_s2.publish_scan_surface_dp_base_images",
            processor_text,
        )
        self.assertIn("self.publish_scan_surface_dp_base_images(surface_result)", manual_workspace_s2_text)
        self.assertIn("def _publish_execution_refine_base_image", execution_refine_hough_text)
        self.assertIn("cv2.COLOR_GRAY2BGR", execution_refine_hough_text)
        self.assertIn("point_array_msg.PointCoordinatesArray", execution_refine_hough_text)
        self.assertIn("cv2.circle", execution_refine_hough_text)
        self.assertIn("_publish_execution_refine_base_image(", execution_refine_hough_text)
        self.assertIn("self.execution_refine_diagnostic_points", execution_refine_hough_text)

    def test_execution_refine_binary_overlay_marks_non_roi_reject_gates(self):
        execution_refine_hough_text = EXECUTION_REFINE_HOUGH_PATH.read_text(encoding="utf-8")

        self.assertIn("EXECUTION_REFINE_DIAGNOSTIC_STYLES", execution_refine_hough_text)
        self.assertIn("def _build_execution_refine_diagnostic_points", execution_refine_hough_text)
        self.assertIn("self.execution_refine_diagnostic_points", execution_refine_hough_text)
        for status in (
            '"hough_raw"',
            '"zero_world"',
            '"out_of_range"',
            '"duplicate_removed"',
            '"selected"',
        ):
            with self.subTest(status=status):
                self.assertIn(status, execution_refine_hough_text)
        self.assertNotIn('"roi_reject"', execution_refine_hough_text)

        self.assertIn("raw_candidate_records", execution_refine_hough_text)
        self.assertIn("duplicate_removed_records", execution_refine_hough_text)
        self.assertIn("zero_world_records", execution_refine_hough_text)
        self.assertIn("out_of_range_records", execution_refine_hough_text)
        self.assertIn("diagnostic_points=None", execution_refine_hough_text)
        self.assertIn("_publish_execution_refine_base_image(", execution_refine_hough_text)
        self.assertIn("self.execution_refine_diagnostic_points", execution_refine_hough_text)

    def test_pointai_image_layers_do_not_use_static_pixel_roi(self):
        workspace_masks_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "workspace_masks.py"
        ).read_text(encoding="utf-8")
        processor_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "processor.py"
        ).read_text(encoding="utf-8")
        state_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "state.py"
        ).read_text(encoding="utf-8")
        process_image_service_text = PROCESS_IMAGE_SERVICE_PATH.read_text(encoding="utf-8")
        execution_refine_hough_text = EXECUTION_REFINE_HOUGH_PATH.read_text(encoding="utf-8")

        for forbidden in (
            "def get_roi_pixel_mask",
            "def is_point_in_roi",
            "get_roi_pixel_mask",
            "is_point_in_roi",
            "self.point1",
            "self.point2",
            "roi_reject",
            "白框ROI",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(
                    forbidden,
                    "\n".join(
                        (
                            workspace_masks_text,
                            processor_text,
                            state_text,
                            process_image_service_text,
                            execution_refine_hough_text,
                        )
                    ),
                )

        self.assertNotIn("self.Depth_image_Raw[self.y1:self.y2, self.x1:self.x2]", workspace_masks_text)
        self.assertIn("return self.get_scan_workspace_pixel_mask()", workspace_masks_text)
        self.assertIn("return True", workspace_masks_text)

    def test_execution_refine_hough_applies_tcp_occlusion_black_mask(self):
        workspace_masks_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "workspace_masks.py"
        ).read_text(encoding="utf-8")
        processor_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "processor.py"
        ).read_text(encoding="utf-8")
        state_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "state.py"
        ).read_text(encoding="utf-8")

        self.assertIn("self.tcp_occlusion_mask_rect = (160, 0, 523, 80)", state_text)
        self.assertIn("def get_tcp_occlusion_pixel_mask(self):", workspace_masks_text)
        self.assertIn("def should_apply_tcp_occlusion_mask(self, request_mode):", workspace_masks_text)
        self.assertIn("request_mode == PROCESS_IMAGE_MODE_EXECUTION_REFINE", workspace_masks_text)
        self.assertIn("tcp_occlusion_mask = self.get_tcp_occlusion_pixel_mask()", workspace_masks_text)
        self.assertIn("self.Depth_image_Raw[tcp_occlusion_mask > 0] = 0", workspace_masks_text)
        self.assertIn("detection_mask[tcp_occlusion_mask > 0] = 0", workspace_masks_text)
        self.assertIn("cls.get_tcp_occlusion_pixel_mask = workspace_masks.get_tcp_occlusion_pixel_mask", processor_text)
        self.assertIn("cls.should_apply_tcp_occlusion_mask = workspace_masks.should_apply_tcp_occlusion_mask", processor_text)

    def test_execution_refine_hough_uses_tcp_coordinate_box_as_roi(self):
        workspace_masks_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "workspace_masks.py"
        ).read_text(encoding="utf-8")
        processor_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "processor.py"
        ).read_text(encoding="utf-8")
        state_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "state.py"
        ).read_text(encoding="utf-8")
        tcp_display_text = (
            WORKSPACE_ROOT
            / "tie_robot_perception"
            / "src"
            / "tie_robot_perception"
            / "pointai"
            / "tcp_display.py"
        ).read_text(encoding="utf-8")
        execution_refine_hough_text = EXECUTION_REFINE_HOUGH_PATH.read_text(encoding="utf-8")
        pipeline_body = execution_refine_hough_text[
            execution_refine_hough_text.index("def run_execution_refine_hough_pipeline("):
        ]

        for expected in (
            'self.execution_refine_tcp_roi_min_x_mm = float(rospy.get_param("~execution_refine_tcp_roi_min_x_mm", 0.0))',
            'self.execution_refine_tcp_roi_max_x_mm = float(rospy.get_param("~execution_refine_tcp_roi_max_x_mm", 380.0))',
            'self.execution_refine_tcp_roi_max_y_mm = float(rospy.get_param("~execution_refine_tcp_roi_max_y_mm", 3330.0))',
            'self.execution_refine_tcp_roi_max_z_mm = float(rospy.get_param("~execution_refine_tcp_roi_max_z_mm", 3160.0))',
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, state_text)

        self.assertIn("def camera_channels_to_tcp_jaw_channels", tcp_display_text)
        self.assertIn("def get_execution_refine_tcp_range_pixel_mask(self):", workspace_masks_text)
        self.assertIn("camera_channels_to_tcp_jaw_channels(", workspace_masks_text)
        self.assertIn("execution_refine_tcp_roi_min_x_mm", workspace_masks_text)
        self.assertIn("execution_refine_tcp_roi_max_z_mm", workspace_masks_text)
        self.assertIn("cls.get_execution_refine_tcp_range_pixel_mask = workspace_masks.get_execution_refine_tcp_range_pixel_mask", processor_text)
        self.assertIn("cls.is_camera_world_coord_in_execution_refine_tcp_range = workspace_masks.is_camera_world_coord_in_execution_refine_tcp_range", processor_text)
        self.assertIn("tcp_range_mask = self.get_execution_refine_tcp_range_pixel_mask()", execution_refine_hough_text)
        self.assertIn("binary[tcp_range_mask <= 0] = 0", execution_refine_hough_text)
        self.assertIn("execution_refine_pixel_mask = self.execution_refine_tcp_range_pixel_mask", pipeline_body)
        self.assertIn("self.is_camera_world_coord_in_execution_refine_tcp_range(", pipeline_body)
        self.assertNotIn("execution_refine_pixel_mask = self.get_scan_workspace_pixel_mask()", pipeline_body)

    def test_execution_refine_hough_does_not_apply_static_roi_gate(self):
        execution_refine_hough_text = EXECUTION_REFINE_HOUGH_PATH.read_text(encoding="utf-8")
        pipeline_body = execution_refine_hough_text[
            execution_refine_hough_text.index("def run_execution_refine_hough_pipeline("):
        ]

        self.assertNotIn("self.is_point_in_roi", pipeline_body)
        self.assertNotIn("roi_reject", pipeline_body)
        self.assertNotIn("get_travel_range_pixel_mask()", pipeline_body)
        self.assertIn("execution_refine_pixel_mask = self.execution_refine_tcp_range_pixel_mask", pipeline_body)
        self.assertIn("execution_refine_pixel_mask is None", pipeline_body)

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
