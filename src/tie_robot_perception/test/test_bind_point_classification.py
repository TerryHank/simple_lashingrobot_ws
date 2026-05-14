#!/usr/bin/env python3

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tie_robot_perception.pointai.bind_point_classification import (
    ClassificationConfig,
    EvidenceBundle,
    append_classification_event,
    classify_pre_bind_rule,
    extract_evidence_bundle,
    should_mark_point_as_bound,
    verify_post_bind_rule,
)


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


if __name__ == "__main__":
    unittest.main()
