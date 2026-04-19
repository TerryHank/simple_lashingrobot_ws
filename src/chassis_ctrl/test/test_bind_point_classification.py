import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from bind_point_classification import (  # noqa: E402
    ClassificationConfig,
    EvidenceBundle,
    append_classification_event,
    classify_pre_bind_rule,
    verify_post_bind_rule,
)


def make_bundle(label, ir_patch, height_patch, valid_mask):
    return EvidenceBundle(
        point_idx=1,
        phase=label,
        pix_coord=(10, 10),
        world_coord=(120.0, 80.0, 55.0),
        ir_patch=np.asarray(ir_patch, dtype=np.uint8),
        depth_patch=np.asarray(height_patch, dtype=np.float32),
        raw_world_patch=np.zeros((8, 8, 3), dtype=np.float32),
        height_patch=np.asarray(height_patch, dtype=np.float32),
        valid_depth_mask=np.asarray(valid_mask, dtype=bool),
        evidence_quality=1.0,
    )


class BindPointClassificationTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        root = Path(self.temp_dir.name)
        self.config = ClassificationConfig(
            mode="shadow",
            event_log_path=root / "events.jsonl",
            evidence_root=root / "evidence",
            patch_half_size_px=4,
            height_band_mm=12.0,
            alignment_search_px=2,
            min_valid_depth_ratio=0.30,
            min_ir_stddev=2.0,
            pre_bind_bound_score_low=0.35,
            pre_bind_bound_score_high=0.65,
            post_bind_success_score_low=0.35,
            post_bind_success_score_high=0.65,
        )

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_classify_pre_bind_rule_marks_clean_flat_patch_unbound(self):
        bundle = make_bundle(
            "before",
            np.full((8, 8), 70, dtype=np.uint8),
            np.zeros((8, 8), dtype=np.float32),
            np.ones((8, 8), dtype=bool),
        )
        decision = classify_pre_bind_rule(bundle, self.config)
        self.assertEqual("unbound", decision.label)
        self.assertLess(decision.score, self.config.pre_bind_bound_score_low)

    def test_classify_pre_bind_rule_marks_center_bump_bound(self):
        height_patch = np.zeros((8, 8), dtype=np.float32)
        height_patch[3:5, 3:5] = 6.0
        ir_patch = np.full((8, 8), 70, dtype=np.uint8)
        ir_patch[2:6, 2:6] = np.array(
            [[40, 120, 45, 110], [125, 60, 130, 65], [50, 125, 55, 120], [130, 70, 128, 75]],
            dtype=np.uint8,
        )
        bundle = make_bundle("before", ir_patch, height_patch, np.ones((8, 8), dtype=bool))
        decision = classify_pre_bind_rule(bundle, self.config)
        self.assertEqual("bound", decision.label)
        self.assertGreater(decision.score, self.config.pre_bind_bound_score_high)

    def test_classify_pre_bind_rule_marks_low_quality_patch_uncertain(self):
        bundle = make_bundle(
            "before",
            np.full((8, 8), 70, dtype=np.uint8),
            np.zeros((8, 8), dtype=np.float32),
            np.pad(np.ones((2, 2), dtype=bool), ((0, 6), (0, 6))),
        )
        decision = classify_pre_bind_rule(bundle, self.config)
        self.assertEqual("uncertain", decision.label)
        self.assertIn("valid_depth_ratio", decision.reason)

    def test_verify_post_bind_rule_marks_after_bump_success(self):
        before_bundle = make_bundle(
            "before",
            np.full((8, 8), 70, dtype=np.uint8),
            np.zeros((8, 8), dtype=np.float32),
            np.ones((8, 8), dtype=bool),
        )
        after_height = np.zeros((8, 8), dtype=np.float32)
        after_height[3:5, 3:5] = 7.0
        after_ir = np.full((8, 8), 70, dtype=np.uint8)
        after_ir[2:6, 2:6] = 140
        after_bundle = make_bundle("after", after_ir, after_height, np.ones((8, 8), dtype=bool))
        decision = verify_post_bind_rule(before_bundle, after_bundle, self.config)
        self.assertEqual("success", decision.label)
        self.assertGreater(decision.score, self.config.post_bind_success_score_high)

    def test_append_classification_event_writes_one_json_object_per_line(self):
        append_classification_event(
            self.config.event_log_path,
            {
                "phase": "before",
                "point_idx": 1,
                "bind_state": "bound",
                "result": "pending",
                "bind_score": 0.91,
            },
        )
        lines = self.config.event_log_path.read_text(encoding="utf-8").strip().splitlines()
        self.assertEqual(1, len(lines))
        payload = json.loads(lines[0])
        self.assertEqual("before", payload["phase"])
        self.assertEqual("bound", payload["bind_state"])
        self.assertEqual("pending", payload["result"])
