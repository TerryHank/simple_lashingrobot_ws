#!/usr/bin/env python3

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from tie_robot_perception.pointai.bind_point_classification import (
    ClassificationConfig,
    EvidenceBundle,
    append_classification_event,
    build_deep_learning_patch,
    classify_pre_bind_point,
    classify_pre_bind_rule,
    classify_pre_bind_deep_learning,
    filter_unbound_points_for_execution,
    extract_evidence_bundle,
    normalize_classification_method,
    should_mark_point_as_bound,
    verify_post_bind_rule,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
PROCESS_IMAGE_SERVICE_PATH = (
    REPO_ROOT
    / "src"
    / "tie_robot_perception"
    / "src"
    / "tie_robot_perception"
    / "pointai"
    / "process_image_service.py"
)
POINTAI_PROCESSOR_PATH = PROCESS_IMAGE_SERVICE_PATH.parent / "processor.py"


def make_config(root):
    return ClassificationConfig(
        mode="shadow",
        event_log_path=Path(root) / "events.jsonl",
        evidence_root=Path(root) / "evidence",
        patch_half_size_px=12,
        height_band_mm=15.0,
        min_valid_depth_ratio=0.35,
        min_ir_stddev=3.0,
        center_radius_px=4,
        ring_inner_radius_px=6,
        ring_outer_radius_px=11,
        bound_height_delta_mm=4.0,
        bound_texture_stddev=14.0,
        ridge_break_ratio_threshold=0.38,
        pre_bind_bound_score_low=0.42,
        pre_bind_bound_score_high=0.68,
        post_bind_success_score_low=0.38,
        post_bind_success_score_high=0.62,
    )


def make_bundle(label, ir_patch, height_patch, valid_mask):
    return EvidenceBundle(
        point_idx=7,
        phase=label,
        pix_coord=(24, 24),
        world_coord=(120.0, 80.0, 55.0),
        ir_patch=np.asarray(ir_patch, dtype=np.uint8),
        depth_patch=np.asarray(height_patch, dtype=np.float32),
        raw_world_patch=np.zeros((*np.asarray(height_patch).shape, 3), dtype=np.float32),
        height_patch=np.asarray(height_patch, dtype=np.float32),
        valid_depth_mask=np.asarray(valid_mask, dtype=bool),
        ridge_patch=np.zeros(np.asarray(height_patch).shape, dtype=np.uint8),
        evidence_quality=1.0,
        metrics={"valid_depth_ratio": float(np.asarray(valid_mask, dtype=bool).mean())},
    )


class BindPointClassificationTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.config = make_config(self.temp_dir.name)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_classify_pre_bind_rule_marks_clean_flat_patch_unbound(self):
        shape = (25, 25)
        bundle = make_bundle(
            "before",
            np.full(shape, 72, dtype=np.uint8),
            np.zeros(shape, dtype=np.float32),
            np.ones(shape, dtype=bool),
        )

        decision = classify_pre_bind_rule(bundle, self.config)

        self.assertEqual("unbound", decision.label)
        self.assertLess(decision.score, self.config.pre_bind_bound_score_low)
        self.assertIn("center_height_delta_mm", decision.metrics)

    def test_classify_pre_bind_rule_marks_center_bump_and_texture_bound(self):
        shape = (25, 25)
        yy, xx = np.indices(shape)
        center_mask = ((xx - 12) ** 2 + (yy - 12) ** 2) <= 4 ** 2
        height_patch = np.zeros(shape, dtype=np.float32)
        height_patch[center_mask] = 7.0
        ir_patch = np.full(shape, 72, dtype=np.uint8)
        ir_patch[center_mask] = np.where((xx[center_mask] + yy[center_mask]) % 2 == 0, 35, 145)
        ridge_patch = np.zeros(shape, dtype=np.uint8)
        ridge_patch[12, :] = 255
        ridge_patch[:, 12] = 255
        ridge_patch[9:16, 9:16] = 0
        bundle = make_bundle("before", ir_patch, height_patch, np.ones(shape, dtype=bool))
        bundle.ridge_patch = ridge_patch

        decision = classify_pre_bind_rule(bundle, self.config)

        self.assertEqual("bound", decision.label)
        self.assertGreaterEqual(decision.score, self.config.pre_bind_bound_score_high)
        self.assertGreater(decision.metrics["center_height_delta_mm"], 4.0)
        self.assertGreater(decision.metrics["center_ir_stddev"], 14.0)

    def test_classify_pre_bind_rule_marks_low_quality_patch_uncertain(self):
        shape = (25, 25)
        valid_mask = np.zeros(shape, dtype=bool)
        valid_mask[:4, :4] = True
        bundle = make_bundle(
            "before",
            np.full(shape, 72, dtype=np.uint8),
            np.zeros(shape, dtype=np.float32),
            valid_mask,
        )

        decision = classify_pre_bind_rule(bundle, self.config)

        self.assertEqual("uncertain", decision.label)
        self.assertIn("valid_depth_ratio", decision.reason)

    def test_extract_evidence_bundle_uses_depth_ring_as_local_height_reference(self):
        ir_image = np.full((60, 60), 80, dtype=np.uint8)
        depth_image = np.full((60, 60), 1000.0, dtype=np.float32)
        depth_image[27:34, 27:34] = 993.0
        raw_world_image = np.zeros((60, 60, 3), dtype=np.float32)
        raw_world_image[:, :, 2] = depth_image

        bundle = extract_evidence_bundle(
            ir_image=ir_image,
            depth_image=depth_image,
            raw_world_image=raw_world_image,
            point_idx=3,
            pix_coord=(30, 30),
            world_coord=(1.0, 2.0, 993.0),
            phase="before",
            config=self.config,
        )

        self.assertGreater(bundle.metrics["valid_depth_ratio"], 0.95)
        self.assertGreater(bundle.metrics["center_height_delta_mm"], 4.0)
        self.assertEqual((25, 25), bundle.ir_patch.shape)

    def test_verify_post_bind_rule_marks_after_bump_success(self):
        shape = (25, 25)
        before_bundle = make_bundle(
            "before",
            np.full(shape, 72, dtype=np.uint8),
            np.zeros(shape, dtype=np.float32),
            np.ones(shape, dtype=bool),
        )
        after_height = np.zeros(shape, dtype=np.float32)
        after_height[10:15, 10:15] = 7.0
        after_ir = np.full(shape, 72, dtype=np.uint8)
        after_ir[10:15, 10:15] = 145
        after_bundle = make_bundle("after", after_ir, after_height, np.ones(shape, dtype=bool))

        decision = verify_post_bind_rule(before_bundle, after_bundle, self.config)

        self.assertEqual("success", decision.label)
        self.assertGreaterEqual(decision.score, self.config.post_bind_success_score_high)

    def test_append_classification_event_writes_jsonl(self):
        append_classification_event(
            self.config.event_log_path,
            {
                "phase": "before",
                "point_idx": 7,
                "bind_state": "bound",
                "bind_score": 0.91,
            },
        )

        lines = self.config.event_log_path.read_text(encoding="utf-8").strip().splitlines()
        self.assertEqual(1, len(lines))
        payload = json.loads(lines[0])
        self.assertEqual("before", payload["phase"])
        self.assertEqual("bound", payload["bind_state"])
        self.assertEqual(0.91, payload["bind_score"])

    def test_shadow_mode_never_marks_point_as_bound_for_wire_format(self):
        decision = type("Decision", (), {"label": "bound", "score": 0.95})()

        self.assertFalse(should_mark_point_as_bound(decision, self.config))

        advisory_config = make_config(self.temp_dir.name)
        advisory_config.mode = "advisory"
        self.assertTrue(should_mark_point_as_bound(decision, advisory_config))

    def test_classification_config_defaults_to_disabled_deep_learning(self):
        self.assertEqual("off", ClassificationConfig().mode)
        self.assertEqual("deep_learning", ClassificationConfig().method)
        self.assertEqual("deep_learning", normalize_classification_method("bad-value"))
        self.assertEqual("white_box", normalize_classification_method("white_box"))

    def test_deep_learning_patch_matches_legacy_depth_mask_resize_and_clahe_shape(self):
        ir_image = np.arange(80 * 80, dtype=np.uint16).reshape(80, 80)
        depth_image = np.full((80, 80), 1000.0, dtype=np.float32)
        depth_image[:20, :] = 1400.0

        patch = build_deep_learning_patch(
            ir_image=ir_image,
            depth_image=depth_image,
            pix_coord=(40, 40),
            raw_z=1000.0,
            half_size=10,
            tolerance_mm=75.0,
        )

        self.assertEqual((128, 128), patch.shape)
        self.assertEqual(np.uint8, patch.dtype)
        self.assertGreater(float(patch.std()), 0.0)

    def test_deep_learning_classifier_maps_class_zero_to_bound_for_execution_skip(self):
        class FakeProb:
            top1 = 0
            top1conf = 0.91

        class FakeResult:
            probs = FakeProb()

        class FakeModel:
            def __call__(self, image, conf=0.2, verbose=False):
                self.last_shape = image.shape
                return [FakeResult()]

        bundle = make_bundle(
            "before",
            np.full((25, 25), 80, dtype=np.uint8),
            np.zeros((25, 25), dtype=np.float32),
            np.ones((25, 25), dtype=bool),
        )

        decision = classify_pre_bind_deep_learning(bundle, self.config, model=FakeModel())

        self.assertEqual("bound", decision.label)
        self.assertGreater(decision.score, 0.9)

    def test_white_box_method_uses_raw_world_depth_height_response(self):
        config = make_config(self.temp_dir.name)
        config.method = "white_box"
        ir_image = np.full((96, 96), 80, dtype=np.uint8)
        plane_depth_image = np.full((96, 96), 25.0, dtype=np.float32)
        raw_world_image = np.zeros((96, 96, 3), dtype=np.float32)
        raw_world_image[:, :, 2] = 1000.0
        raw_world_image[42:55, 42:55, 2] = 988.0
        cv = np.arange(39, 58)
        ir_image[cv, cv] = 26
        ir_image[cv, cv + 1] = 32

        bundle = extract_evidence_bundle(
            ir_image=ir_image,
            depth_image=plane_depth_image,
            raw_world_image=raw_world_image,
            point_idx=5,
            pix_coord=(48, 48),
            world_coord=(1.0, 2.0, 988.0),
            phase="execution_refine",
            config=config,
        )
        decision = classify_pre_bind_point(bundle, config, model_cache={"model": object()})

        self.assertEqual("bound", decision.label)
        self.assertEqual("white_box_multimodal_tie_residual", decision.reason)
        self.assertGreater(decision.metrics["white_box_diagonal_score"], 0.4)
        self.assertEqual("multimodal_cross_residual", decision.metrics["white_box_response_source"])

    def test_white_box_keeps_strong_clean_rebar_crossing_unbound(self):
        config = make_config(self.temp_dir.name)
        config.method = "white_box"
        ir_image = np.full((96, 96), 112, dtype=np.uint8)
        raw_world_image = np.zeros((96, 96, 3), dtype=np.float32)
        raw_world_image[:, :, 2] = 1000.0
        raw_world_image[45:52, :, 2] = 988.0
        raw_world_image[:, 45:52, 2] = 988.0
        ir_image[45:52, :] = 35
        ir_image[:, 45:52] = 35

        bundle = extract_evidence_bundle(
            ir_image=ir_image,
            depth_image=raw_world_image[:, :, 2],
            raw_world_image=raw_world_image,
            point_idx=6,
            pix_coord=(48, 48),
            world_coord=(1.0, 2.0, 988.0),
            phase="execution_refine",
            config=config,
        )
        decision = classify_pre_bind_point(bundle, config)

        self.assertEqual("unbound", decision.label)
        self.assertEqual("white_box_clean_rebar_crossing", decision.reason)
        self.assertGreater(decision.metrics["white_box_axis_explained_ratio"], 1.3)
        self.assertTrue(decision.metrics["white_box_clean_crossing"])
        self.assertFalse(decision.metrics["white_box_has_diagonal_wire"])
        self.assertLess(decision.metrics["white_box_diagonal_score"], 0.2)

    def test_white_box_marks_off_axis_diagonal_wire_on_crossing_bound(self):
        config = make_config(self.temp_dir.name)
        config.method = "white_box"
        ir_image = np.full((96, 96), 112, dtype=np.uint8)
        raw_world_image = np.zeros((96, 96, 3), dtype=np.float32)
        raw_world_image[:, :, 2] = 1000.0
        raw_world_image[45:52, :, 2] = 988.0
        raw_world_image[:, 45:52, 2] = 988.0
        ir_image[45:52, :] = 35
        ir_image[:, 45:52] = 35
        for offset in range(-13, 14):
            y = 48 + offset
            x = 48 + offset
            ir_image[max(0, y - 1) : min(96, y + 2), max(0, x - 1) : min(96, x + 2)] = 12
            raw_world_image[max(0, y - 1) : min(96, y + 2), max(0, x - 1) : min(96, x + 2), 2] = 982.0

        bundle = extract_evidence_bundle(
            ir_image=ir_image,
            depth_image=raw_world_image[:, :, 2],
            raw_world_image=raw_world_image,
            point_idx=7,
            pix_coord=(48, 48),
            world_coord=(1.0, 2.0, 982.0),
            phase="execution_refine",
            config=config,
        )
        decision = classify_pre_bind_point(bundle, config)

        self.assertEqual("bound", decision.label)
        self.assertEqual("white_box_multimodal_tie_residual", decision.reason)
        self.assertGreater(decision.metrics["white_box_diagonal_score"], 0.4)
        self.assertGreaterEqual(decision.metrics["white_box_positive_votes"], 2)

    def test_execution_refine_blocking_white_box_marks_and_filters_bound_points(self):
        config = make_config(self.temp_dir.name)
        config.mode = "blocking"
        config.method = "white_box"
        ir_image = np.full((160, 160), 80, dtype=np.uint8)
        plane_depth_image = np.full((160, 160), 25.0, dtype=np.float32)
        raw_world_image = np.zeros((160, 160, 3), dtype=np.float32)
        raw_world_image[:, :, 2] = 1000.0
        raw_world_image[42:55, 42:55, 2] = 988.0

        bound_point = SimpleNamespace(
            idx=1,
            Pix_coord=[48, 48],
            World_coord=[1.0, 2.0, 988.0],
            is_shuiguan=False,
        )
        unbound_point = SimpleNamespace(
            idx=2,
            Pix_coord=[120, 120],
            World_coord=[3.0, 4.0, 1000.0],
            is_shuiguan=False,
        )
        for point in (bound_point, unbound_point):
            bundle = extract_evidence_bundle(
                ir_image=ir_image,
                depth_image=plane_depth_image,
                raw_world_image=raw_world_image,
                point_idx=int(point.idx),
                pix_coord=point.Pix_coord,
                world_coord=point.World_coord,
                phase="execution_refine",
                config=config,
            )
            decision = classify_pre_bind_point(bundle, config)
            point.is_shuiguan = should_mark_point_as_bound(decision, config)
        filtered_points = filter_unbound_points_for_execution([bound_point, unbound_point], config)

        self.assertTrue(bound_point.is_shuiguan)
        self.assertFalse(unbound_point.is_shuiguan)
        self.assertEqual([2], [point.idx for point in filtered_points])

    def test_white_box_raw_ir_normalized_prefers_right_two_bound_in_four_point_roi(self):
        config = make_config(self.temp_dir.name)
        config.method = "white_box"
        ir_image = np.full((220, 220), 110, dtype=np.uint8)
        raw_world_image = np.zeros((220, 220, 3), dtype=np.float32)
        raw_world_image[:, :, 2] = 1000.0

        point_specs = [
            ((60, 100), (1.0, 1.0, 988.0), "unbound"),
            ((92, 100), (2.0, 1.0, 988.0), "unbound"),
            ((150, 100), (3.0, 1.0, 982.0), "bound"),
            ((182, 100), (4.0, 1.0, 982.0), "bound"),
        ]
        for pix, world, label in point_specs:
            x, y = pix
            ir_image[40:180, x - 1 : x + 2] = 38
            ir_image[y - 1 : y + 2, 40:210] = 38
            raw_world_image[40:180, x - 1 : x + 2, 2] = 988.0
            raw_world_image[y - 1 : y + 2, 40:210, 2] = 988.0
            if label == "bound":
                for yy in range(y - 8, y + 9):
                    for xx in range(x - 8, x + 9):
                        if 0 <= xx < 220 and 0 <= yy < 220 and (xx - x) ** 2 + (yy - y) ** 2 <= 36:
                            ir_image[yy, xx] = 18
                            raw_world_image[yy, xx, 2] = 982.0
                for offset in range(-8, 9):
                    xx = x + offset
                    yy = y + (offset // 2)
                    if 0 <= xx < 220 and 0 <= yy < 220:
                        ir_image[max(0, yy - 1) : min(220, yy + 2), max(0, xx - 1) : min(220, xx + 2)] = 12
                        raw_world_image[max(0, yy - 1) : min(220, yy + 2), max(0, xx - 1) : min(220, xx + 2), 2] = 980.0

        expected_labels = ["unbound", "unbound", "bound", "bound"]
        actual_labels = []
        for idx, (pix, world, _) in enumerate(point_specs, start=1):
            bundle = extract_evidence_bundle(
                ir_image=ir_image,
                depth_image=raw_world_image[:, :, 2],
                raw_world_image=raw_world_image,
                point_idx=idx,
                pix_coord=pix,
                world_coord=world,
                phase="execution_refine",
                config=config,
            )
            decision = classify_pre_bind_point(bundle, config)
            actual_labels.append(decision.label)

        self.assertEqual(expected_labels, actual_labels)

    def test_surface_dp_execution_refine_still_uses_execution_refine_classification_phase(self):
        process_service_text = PROCESS_IMAGE_SERVICE_PATH.read_text(encoding="utf-8")

        self.assertIn("run_execution_refine_visual_pipeline", process_service_text)
        self.assertIn("classify_execution_refine_points", process_service_text)
        self.assertIn('"execution_refine"', process_service_text)

    def test_execution_refine_classification_result_is_republished_to_base_image(self):
        process_service_text = PROCESS_IMAGE_SERVICE_PATH.read_text(encoding="utf-8")
        processor_text = POINTAI_PROCESSOR_PATH.read_text(encoding="utf-8")

        self.assertIn("build_execution_refine_classification_diagnostic_points", process_service_text)
        self.assertRegex(
            processor_text,
            r"cls\.build_execution_refine_classification_diagnostic_points\s*=\s*\(\s*"
            r"process_image_service\.build_execution_refine_classification_diagnostic_points\s*\)",
        )
        self.assertIn("publish_execution_refine_classified_base_image", process_service_text)
        self.assertIn("classification_bound", process_service_text)
        self.assertIn("classification_unbound", process_service_text)
        self.assertIn("classification_uncertain", process_service_text)
        self.assertIn("self.execution_refine_classification_diagnostic_points = []", process_service_text)
        self.assertRegex(
            process_service_text,
            r"classify_execution_refine_points[\s\S]+execution_refine_classification_diagnostic_points",
        )
        self.assertRegex(
            process_service_text,
            r"evaluate_point_coords_for_mode[\s\S]+publish_execution_refine_classified_base_image",
        )

    def test_filter_unbound_points_keeps_default_off_chain_unchanged(self):
        points = [
            type("Point", (), {"is_shuiguan": False})(),
            type("Point", (), {"is_shuiguan": True})(),
        ]

        self.assertEqual(points, filter_unbound_points_for_execution(points, ClassificationConfig(mode="off")))
        self.assertEqual([points[0]], filter_unbound_points_for_execution(points, ClassificationConfig(mode="blocking")))


if __name__ == "__main__":
    unittest.main()
