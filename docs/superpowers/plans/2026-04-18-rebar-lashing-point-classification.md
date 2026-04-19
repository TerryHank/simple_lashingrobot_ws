# Rebar Lashing Point Classification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a two-stage rebar lashing point classification path that uses current-frame pre-bind rules to skip already-bound points, caches `before` evidence, and performs post-bind `before/after` verification without breaking the existing `PointCoords.is_shuiguan` wire format.

**Architecture:** Put the reusable evidence extraction, rule scoring, JSONL logging, and evidence persistence into a focused Python helper module under `src/chassis_ctrl/scripts`, then integrate that helper into `pointAI.py` for bind-check capture plus a new `/pointAI/verify_bind_points` service. Keep current downstream compatibility by still publishing `bool is_shuiguan`, and let `moduanNode.cpp` consume that boolean to skip pre-classified points while calling the new verification service after actual execution.

**Tech Stack:** ROS Noetic, rospy, roscpp, OpenCV, NumPy, Python `unittest`, catkin messages/services

---

## File Structure

- Create: `src/chassis_ctrl/config/bind_point_classification.yaml`
  - Centralize runtime mode, thresholds, evidence paths, and patch sizing so rule tuning stops living inside `pointAI.py`.

- Create: `src/chassis_ctrl/scripts/bind_point_classification.py`
  - Own `ClassificationConfig`, `EvidenceBundle`, `RuleDecision`, patch extraction, rule scoring, JSONL event writing, and evidence persistence.

- Create: `src/chassis_ctrl/srv/VerifyBindPoints.srv`
  - Define the post-bind verification request/summary contract between `moduanNode.cpp` and `pointAI.py`.

- Create: `src/chassis_ctrl/test/test_bind_point_classification.py`
  - Hold fast unit tests for rule scoring and JSONL writing without needing live ROS nodes.

- Modify: `src/chassis_ctrl/scripts/pointAI.py`
  - Load the new config, classify bind-check points before publishing, cache `before` evidence by point index, and expose `/pointAI/verify_bind_points`.

- Modify: `src/chassis_ctrl/src/moduanNode.cpp`
  - Skip `is_shuiguan=true` points before execution, keep jump-bind behavior, call post-bind verification after execution, and log the verify summary.

- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
  - Add regression checks for new ROS wiring, config loading, and service registration surfaces.

- Modify: `src/chassis_ctrl/CMakeLists.txt`
  - Register `VerifyBindPoints.srv`.

- Modify: `src/chassis_ctrl/launch/run.launch`
  - Load the classification YAML into `pointAINode` and expose the `post_bind_verify_enabled` switch for `moduanNode`.

- Runtime output only: `src/chassis_ctrl/data/bind_classification_events.jsonl`
  - Append one JSON object per pre-bind or post-bind decision for replay and tuning.

- Runtime output only: `src/chassis_ctrl/data/bind_evidence/`
  - Store `before_*` and `after_*` patches plus `manifest.json` for each point/session.

---

### Task 1: Lock The Rule Engine With Failing Tests

**Files:**
- Create: `src/chassis_ctrl/test/test_bind_point_classification.py`
- Test: `src/chassis_ctrl/test/test_bind_point_classification.py`

- [ ] **Step 1: Write the failing unit tests for pre-bind, post-bind, and JSONL logging**

```python
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from bind_point_classification import (
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
```

- [ ] **Step 2: Run the new unit tests to verify they fail**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_bind_point_classification -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'bind_point_classification'`.

- [ ] **Step 3: Commit the red tests**

```bash
git add src/chassis_ctrl/test/test_bind_point_classification.py
git commit -m "test: lock bind point classification rule behavior"
```

### Task 2: Implement The Shared Classification Helper And Config

**Files:**
- Create: `src/chassis_ctrl/config/bind_point_classification.yaml`
- Create: `src/chassis_ctrl/scripts/bind_point_classification.py`
- Test: `src/chassis_ctrl/test/test_bind_point_classification.py`

- [ ] **Step 1: Add the YAML config file with explicit thresholds and output paths**

```yaml
mode: shadow
event_log_path: /home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_classification_events.jsonl
evidence_root: /home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_evidence
patch_half_size_px: 32
height_band_mm: 15.0
alignment_search_px: 6
min_valid_depth_ratio: 0.35
min_ir_stddev: 5.0
pre_bind_bound_score_low: 0.42
pre_bind_bound_score_high: 0.68
post_bind_success_score_low: 0.38
post_bind_success_score_high: 0.62
```

- [ ] **Step 2: Implement the helper module with config loading, evidence extraction, rule scoring, and JSONL writing**

```python
#!/usr/bin/env python3

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class ClassificationConfig:
    mode: str
    event_log_path: Path
    evidence_root: Path
    patch_half_size_px: int
    height_band_mm: float
    alignment_search_px: int
    min_valid_depth_ratio: float
    min_ir_stddev: float
    pre_bind_bound_score_low: float
    pre_bind_bound_score_high: float
    post_bind_success_score_low: float
    post_bind_success_score_high: float


@dataclass
class EvidenceBundle:
    point_idx: int
    phase: str
    pix_coord: Tuple[int, int]
    world_coord: Tuple[float, float, float]
    ir_patch: np.ndarray
    depth_patch: np.ndarray
    raw_world_patch: np.ndarray
    height_patch: np.ndarray
    valid_depth_mask: np.ndarray
    evidence_quality: float
    metrics: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class RuleDecision:
    label: str
    score: float
    quality: float
    reason: str
    metrics: Dict[str, float]


def load_classification_config(config_path):
    data = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    return ClassificationConfig(
        mode=str(data.get("mode", "shadow")),
        event_log_path=Path(data["event_log_path"]),
        evidence_root=Path(data["evidence_root"]),
        patch_half_size_px=int(data.get("patch_half_size_px", 32)),
        height_band_mm=float(data.get("height_band_mm", 15.0)),
        alignment_search_px=int(data.get("alignment_search_px", 6)),
        min_valid_depth_ratio=float(data.get("min_valid_depth_ratio", 0.35)),
        min_ir_stddev=float(data.get("min_ir_stddev", 5.0)),
        pre_bind_bound_score_low=float(data.get("pre_bind_bound_score_low", 0.42)),
        pre_bind_bound_score_high=float(data.get("pre_bind_bound_score_high", 0.68)),
        post_bind_success_score_low=float(data.get("post_bind_success_score_low", 0.38)),
        post_bind_success_score_high=float(data.get("post_bind_success_score_high", 0.62)),
    )


def extract_evidence_bundle(ir_image, depth_image, raw_world_image, point_idx, pix_coord, world_coord, phase, config):
    x_value, y_value = int(pix_coord[0]), int(pix_coord[1])
    half = int(config.patch_half_size_px)
    x0 = max(0, x_value - half)
    x1 = min(ir_image.shape[1], x_value + half)
    y0 = max(0, y_value - half)
    y1 = min(ir_image.shape[0], y_value + half)

    ir_patch = ir_image[y0:y1, x0:x1].copy()
    depth_patch = depth_image[y0:y1, x0:x1].astype(np.float32).copy()
    raw_world_patch = raw_world_image[y0:y1, x0:x1].astype(np.float32).copy()
    z0 = float(world_coord[2])
    valid_depth_mask = np.isfinite(depth_patch) & (depth_patch > 0)
    height_patch = np.where(valid_depth_mask, np.clip(z0 - depth_patch, 0.0, config.height_band_mm), 0.0)
    valid_depth_ratio = float(valid_depth_mask.mean()) if valid_depth_mask.size else 0.0
    ir_stddev = float(np.std(ir_patch)) if ir_patch.size else 0.0
    evidence_quality = min(1.0, 0.7 * valid_depth_ratio + 0.3 * min(ir_stddev / max(config.min_ir_stddev, 1.0), 1.0))

    return EvidenceBundle(
        point_idx=int(point_idx),
        phase=str(phase),
        pix_coord=(x_value, y_value),
        world_coord=(float(world_coord[0]), float(world_coord[1]), z0),
        ir_patch=ir_patch,
        depth_patch=depth_patch,
        raw_world_patch=raw_world_patch,
        height_patch=height_patch,
        valid_depth_mask=valid_depth_mask,
        evidence_quality=evidence_quality,
        metrics={"valid_depth_ratio": valid_depth_ratio, "ir_stddev": ir_stddev},
    )


def classify_pre_bind_rule(bundle, config):
    valid_depth_ratio = float(bundle.metrics.get("valid_depth_ratio", 0.0))
    if valid_depth_ratio < config.min_valid_depth_ratio:
        return RuleDecision("uncertain", 0.0, bundle.evidence_quality, "valid_depth_ratio_too_low", bundle.metrics)

    center = bundle.height_patch[bundle.height_patch.shape[0] // 4 : bundle.height_patch.shape[0] * 3 // 4,
                                 bundle.height_patch.shape[1] // 4 : bundle.height_patch.shape[1] * 3 // 4]
    center_bump_mm = float(center.mean()) if center.size else 0.0
    texture_score = min(float(bundle.metrics.get("ir_stddev", 0.0)) / 32.0, 1.0)
    bump_score = min(center_bump_mm / 6.0, 1.0)
    bound_score = 0.6 * bump_score + 0.4 * texture_score
    metrics = dict(bundle.metrics)
    metrics.update({"center_bump_mm": center_bump_mm})

    if bound_score >= config.pre_bind_bound_score_high:
        return RuleDecision("bound", bound_score, bundle.evidence_quality, "center_bump_and_texture", metrics)
    if bound_score <= config.pre_bind_bound_score_low:
        return RuleDecision("unbound", bound_score, bundle.evidence_quality, "flat_center_patch", metrics)
    return RuleDecision("uncertain", bound_score, bundle.evidence_quality, "score_between_thresholds", metrics)


def verify_post_bind_rule(before_bundle, after_bundle, config):
    if min(before_bundle.evidence_quality, after_bundle.evidence_quality) < 0.5:
        return RuleDecision("uncertain", 0.0, min(before_bundle.evidence_quality, after_bundle.evidence_quality), "quality_too_low", {})

    best_delta = -1.0
    best_offset = (0, 0)
    for dy in range(-config.alignment_search_px, config.alignment_search_px + 1):
        for dx in range(-config.alignment_search_px, config.alignment_search_px + 1):
            shifted = np.roll(np.roll(after_bundle.height_patch, dy, axis=0), dx, axis=1)
            delta = float((shifted - before_bundle.height_patch).mean())
            if delta > best_delta:
                best_delta = delta
                best_offset = (dx, dy)

    shifted_height = np.roll(np.roll(after_bundle.height_patch, best_offset[1], axis=0), best_offset[0], axis=1)
    shifted_ir = np.roll(np.roll(after_bundle.ir_patch, best_offset[1], axis=0), best_offset[0], axis=1)
    height_gain_mm = float(np.clip((shifted_height - before_bundle.height_patch).mean(), 0.0, None))
    texture_gain = float(np.clip(np.std(shifted_ir) - np.std(before_bundle.ir_patch), 0.0, None))
    success_score = 0.7 * min(height_gain_mm / 5.0, 1.0) + 0.3 * min(texture_gain / 20.0, 1.0)
    metrics = {"height_gain_mm": height_gain_mm, "texture_gain": texture_gain, "align_dx": best_offset[0], "align_dy": best_offset[1]}

    if success_score >= config.post_bind_success_score_high:
        return RuleDecision("success", success_score, min(before_bundle.evidence_quality, after_bundle.evidence_quality), "localized_after_bump", metrics)
    if success_score <= config.post_bind_success_score_low:
        return RuleDecision("failed", success_score, min(before_bundle.evidence_quality, after_bundle.evidence_quality), "insufficient_after_change", metrics)
    return RuleDecision("uncertain", success_score, min(before_bundle.evidence_quality, after_bundle.evidence_quality), "verify_score_between_thresholds", metrics)


def append_classification_event(event_log_path, payload):
    event_log_path = Path(event_log_path)
    event_log_path.parent.mkdir(parents=True, exist_ok=True)
    with event_log_path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=False) + "\n")


def persist_evidence_bundle(bundle, config, session_id):
    point_dir = Path(config.evidence_root) / session_id / f"point_{bundle.point_idx:02d}"
    point_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(point_dir / f"{bundle.phase}_ir.png"), bundle.ir_patch)
    np.save(point_dir / f"{bundle.phase}_depth.npy", bundle.depth_patch)
    np.save(point_dir / f"{bundle.phase}_height.npy", bundle.height_patch)
    np.save(point_dir / f"{bundle.phase}_raw_world.npy", bundle.raw_world_patch)
    manifest_path = point_dir / "manifest.json"
    manifest = {
        "session_id": session_id,
        "phase": bundle.phase,
        "mode": config.mode,
        "point_idx": bundle.point_idx,
        "pix_coord": list(bundle.pix_coord),
        "world_coord": list(bundle.world_coord),
        "evidence_quality": bundle.evidence_quality,
        "metrics": bundle.metrics,
    }
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return point_dir
```

- [ ] **Step 3: Run the unit tests and a syntax check**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_bind_point_classification -v
python3 -m py_compile src/chassis_ctrl/scripts/bind_point_classification.py
```

Expected: PASS.

- [ ] **Step 4: Commit the helper module and config**

```bash
git add src/chassis_ctrl/config/bind_point_classification.yaml src/chassis_ctrl/scripts/bind_point_classification.py src/chassis_ctrl/test/test_bind_point_classification.py
git commit -m "feat: add bind point classification helper"
```

### Task 3: Integrate Pre-Bind Classification Into `pointAI.py`

**Files:**
- Modify: `src/chassis_ctrl/scripts/pointAI.py`
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Add failing regression checks for the pre-bind integration points**

```python
def test_pointai_loads_bind_classification_config_and_marks_bound_points(self):
    pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
    self.assertIn("bind_point_classification.yaml", pointai_text)
    self.assertIn("load_classification_config", pointai_text)
    self.assertIn("self.bind_classification_config_path", pointai_text)
    self.assertIn("self.bind_classification_config.mode", pointai_text)
    self.assertIn("self.pre_bind_evidence_cache = {}", pointai_text)
    self.assertIn("classify_pre_bind_rule(", pointai_text)
    self.assertIn("persist_evidence_bundle(", pointai_text)
    self.assertIn("append_classification_event(", pointai_text)
    self.assertIn("should_block_pre_bind_decision", pointai_text)
    self.assertIn("self.PointCoordinates.is_shuiguan = self.should_block_pre_bind_decision(decision)", pointai_text)
```

- [ ] **Step 2: Run the focused regression test to verify it fails**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_pointai_loads_bind_classification_config_and_marks_bound_points -v
```

Expected: FAIL because `pointAI.py` does not yet load the YAML or classify bind-check points.

- [ ] **Step 3: Modify `pointAI.py` to load the config, start a bind session, classify each bind-check point, and publish the legacy boolean**

```python
from bind_point_classification import (
    append_classification_event,
    classify_pre_bind_rule,
    extract_evidence_bundle,
    load_classification_config,
    persist_evidence_bundle,
)
...
self.bind_classification_config_path = rospy.get_param(
    "~bind_classification_config_path",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "bind_point_classification.yaml")),
)
self.bind_classification_config = load_classification_config(self.bind_classification_config_path)
self.pre_bind_evidence_cache = {}
self.current_bind_session_id = ""
...
def begin_bind_classification_session(self):
    self.current_bind_session_id = str(rospy.Time.now().to_nsec())
    self.pre_bind_evidence_cache = {}

def should_block_pre_bind_decision(self, decision):
    if decision is None:
        return False
    if self.bind_classification_config.mode != "blocking":
        return False
    return decision.label == "bound"

def classify_pre_bind_candidate(self, point_idx, pixel_xy, world_xyz):
    bundle = extract_evidence_bundle(
        self.image_infrared,
        self.depth_v,
        self.image_raw_world,
        point_idx,
        pixel_xy,
        world_xyz,
        "before",
        self.bind_classification_config,
    )
    decision = classify_pre_bind_rule(bundle, self.bind_classification_config)
    self.pre_bind_evidence_cache[int(point_idx)] = bundle
    persist_evidence_bundle(bundle, self.bind_classification_config, self.current_bind_session_id)
    append_classification_event(
        self.bind_classification_config.event_log_path,
        {
            "session_id": self.current_bind_session_id,
            "phase": "before",
            "mode": self.bind_classification_config.mode,
            "point_idx": int(point_idx),
            "pix_coord": [int(pixel_xy[0]), int(pixel_xy[1])],
            "world_coord": [float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2])],
            "bind_state": decision.label,
            "result": "pending",
            "bind_score": decision.score,
            "evidence_quality": decision.quality,
            "decision_reason": decision.reason,
        },
    )
    return decision
...
if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK:
    self.begin_bind_classification_session()
...
decision = None
if request_mode == PROCESS_IMAGE_MODE_BIND_CHECK and self.bind_classification_config.mode != "off":
    decision = self.classify_pre_bind_candidate(idx, (x, y), (self.x_value, self.y_value, self.z_value))

self.PointCoordinates = PointCoords()
self.PointCoordinates.is_shuiguan = self.should_block_pre_bind_decision(decision)
```

- [ ] **Step 4: Run the focused regression test plus the helper/unit suite**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_pointai_loads_bind_classification_config_and_marks_bound_points -v
python3 -m unittest src.chassis_ctrl.test.test_bind_point_classification src/chassis_ctrl/test/test_pointai_order.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit the `pointAI.py` pre-bind integration**

```bash
git add src/chassis_ctrl/scripts/pointAI.py src/chassis_ctrl/test/test_pointai_order.py
git commit -m "feat: classify bind-check points before execution"
```

### Task 4: Add The Post-Bind Verification Service And `pointAI` Handler

**Files:**
- Create: `src/chassis_ctrl/srv/VerifyBindPoints.srv`
- Modify: `src/chassis_ctrl/CMakeLists.txt`
- Modify: `src/chassis_ctrl/scripts/pointAI.py`
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Add failing regression checks for the new service contract and pointAI handler**

```python
def test_verify_bind_points_service_is_generated_and_registered(self):
    service_text = (CHASSIS_CTRL_DIR / "srv" / "VerifyBindPoints.srv").read_text(encoding="utf-8")
    self.assertIn("fast_image_solve/PointCoords[] points", service_text)
    self.assertIn("int32 verified_count", service_text)
    self.assertIn("int32 success_count", service_text)
    self.assertIn("int32 failed_count", service_text)
    self.assertIn("int32 uncertain_count", service_text)

    cmake_text = (CHASSIS_CTRL_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    self.assertIn("VerifyBindPoints.srv", cmake_text)

    pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
    self.assertIn("/pointAI/verify_bind_points", pointai_text)
    self.assertIn('self.bind_classification_config.mode == "off"', pointai_text)
    self.assertIn("verify_post_bind_rule(", pointai_text)
    self.assertIn("self.pre_bind_evidence_cache.get(int(point.idx))", pointai_text)
```

- [ ] **Step 2: Run the focused regression test to verify it fails**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_verify_bind_points_service_is_generated_and_registered -v
```

Expected: FAIL because the service file and handler do not exist yet.

- [ ] **Step 3: Add the service definition, register it with catkin, and implement the handler in `pointAI.py`**

```text
# src/chassis_ctrl/srv/VerifyBindPoints.srv
fast_image_solve/PointCoords[] points
---
bool success
string message
int32 verified_count
int32 success_count
int32 failed_count
int32 uncertain_count
```

```cmake
add_service_files(
  FILES
  SingleMove.srv
  Pathguihua.srv
  linear_module_move.srv
  ExecuteBindPoints.srv
  VerifyBindPoints.srv
  ResetPLCWarning.srv
  Resetzero.srv
  MotionControl.srv
)
```

```python
from chassis_ctrl.srv import VerifyBindPoints, VerifyBindPointsResponse
from bind_point_classification import verify_post_bind_rule
...
self.verify_bind_service = rospy.Service(
    "/pointAI/verify_bind_points",
    VerifyBindPoints,
    self.handle_verify_bind_points,
)
...
def handle_verify_bind_points(self, req):
    if self.bind_classification_config.mode == "off":
        response = VerifyBindPointsResponse()
        response.success = True
        response.message = "post_bind verify disabled by mode=off"
        return response

    success_count = 0
    failed_count = 0
    uncertain_count = 0

    for point in req.points:
        before_bundle = self.pre_bind_evidence_cache.get(int(point.idx))
        if before_bundle is None:
            uncertain_count += 1
            append_classification_event(
                self.bind_classification_config.event_log_path,
                {
                    "session_id": self.current_bind_session_id,
                    "phase": "after",
                    "mode": self.bind_classification_config.mode,
                    "point_idx": int(point.idx),
                    "result": "uncertain",
                    "decision_reason": "missing_before_bundle",
                },
            )
            continue

        after_bundle = extract_evidence_bundle(
            self.image_infrared,
            self.depth_v,
            self.image_raw_world,
            int(point.idx),
            point.Pix_coord,
            point.World_coord,
            "after",
            self.bind_classification_config,
        )
        decision = verify_post_bind_rule(before_bundle, after_bundle, self.bind_classification_config)
        persist_evidence_bundle(after_bundle, self.bind_classification_config, self.current_bind_session_id)
        append_classification_event(
            self.bind_classification_config.event_log_path,
            {
                "session_id": self.current_bind_session_id,
                "phase": "after",
                "mode": self.bind_classification_config.mode,
                "point_idx": int(point.idx),
                "result": decision.label,
                "bind_score": decision.score,
                "evidence_quality": decision.quality,
                "decision_reason": decision.reason,
                "metrics": decision.metrics,
            },
        )

        if decision.label == "success":
            success_count += 1
        elif decision.label == "failed":
            failed_count += 1
        else:
            uncertain_count += 1

    response = VerifyBindPointsResponse()
    response.verified_count = len(req.points)
    response.success_count = success_count
    response.failed_count = failed_count
    response.uncertain_count = uncertain_count
    response.success = failed_count == 0 and uncertain_count == 0
    response.message = (
        f"post_bind verify complete: success={success_count} "
        f"failed={failed_count} uncertain={uncertain_count}"
    )
    return response
```

- [ ] **Step 4: Run the new regression test, rebuild generated code, and re-run the Python suites**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_verify_bind_points_service_is_generated_and_registered -v
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1
python3 -m unittest src.chassis_ctrl.test.test_bind_point_classification src/chassis_ctrl/test/test_pointai_order.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit the service wiring**

```bash
git add src/chassis_ctrl/srv/VerifyBindPoints.srv src/chassis_ctrl/CMakeLists.txt src/chassis_ctrl/scripts/pointAI.py src/chassis_ctrl/test/test_pointai_order.py
git commit -m "feat: add post bind verification service"
```

### Task 5: Wire `moduanNode.cpp` To Skip Bound Points And Call Post-Bind Verification

**Files:**
- Modify: `src/chassis_ctrl/src/moduanNode.cpp`
- Modify: `src/chassis_ctrl/launch/run.launch`
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Add failing regression checks for the moduan/service integration and launch config**

```python
def test_moduan_skips_bound_points_and_calls_post_bind_verifier(self):
    moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")
    self.assertIn("VerifyBindPoints", moduan_text)
    self.assertIn("/pointAI/verify_bind_points", moduan_text)
    self.assertIn("post_bind_verify_enabled", moduan_text)
    self.assertIn("point.is_shuiguan", moduan_text)
    self.assertIn("已绑扎点跳过", moduan_text)

    launch_text = (CHASSIS_CTRL_DIR / "launch" / "run.launch").read_text(encoding="utf-8")
    self.assertIn("bind_point_classification.yaml", launch_text)
    self.assertIn("bind_classification_config_path", launch_text)
    self.assertIn("post_bind_verify_enabled", launch_text)
```

- [ ] **Step 2: Run the focused regression test to verify it fails**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_moduan_skips_bound_points_and_calls_post_bind_verifier -v
```

Expected: FAIL because `moduanNode.cpp` does not yet inspect `point.is_shuiguan` or call the verification service.

- [ ] **Step 3: Implement the bound-point filter, post-bind verify call, and launch params**

```cpp
#include "chassis_ctrl/VerifyBindPoints.h"
...
ros::ServiceClient verify_bind_points_client;
bool post_bind_verify_enabled = true;
...
std::vector<fast_image_solve::PointCoords> filter_points_ready_for_execution(
    const std::vector<fast_image_solve::PointCoords>& points,
    size_t* skipped_bound_count,
    size_t* skipped_jump_count)
{
    std::vector<fast_image_solve::PointCoords> executable;
    for (const auto& point : points) {
        if (point.is_shuiguan) {
            (*skipped_bound_count)++;
            continue;
        }
        if (!should_keep_jump_bind_point(point)) {
            (*skipped_jump_count)++;
            continue;
        }
        executable.push_back(point);
    }
    return executable;
}
...
private_nh.param("post_bind_verify_enabled", post_bind_verify_enabled, true);
verify_bind_points_client = nh_.serviceClient<chassis_ctrl::VerifyBindPoints>("/pointAI/verify_bind_points");
...
size_t skipped_bound_count = 0;
size_t skipped_jump_count = 0;
const auto executablePoints = filter_points_ready_for_execution(
    filteredPoints,
    &skipped_bound_count,
    &skipped_jump_count
);
printf("Moduan_log: 已绑扎点跳过:%zu, 跳绑点跳过:%zu, 待执行点:%zu\n",
       skipped_bound_count,
       skipped_jump_count,
       executablePoints.size());
if (executablePoints.empty()) {
    res.success = false;
    res.message = "分类结果显示当前区域无待绑扎点，跳过当前区域";
    pub_moduan_work_state(false);
    return true;
}
const bool executed = execute_bind_points(executablePoints, res.message, false);
if (executed && post_bind_verify_enabled) {
    chassis_ctrl::VerifyBindPoints verify_srv;
    verify_srv.request.points = executablePoints;
    if (verify_bind_points_client.call(verify_srv)) {
        printf("Moduan_log: 绑后复检结果 verified=%d success=%d failed=%d uncertain=%d\n",
               verify_srv.response.verified_count,
               verify_srv.response.success_count,
               verify_srv.response.failed_count,
               verify_srv.response.uncertain_count);
    }
}
```

```xml
<node name="moduanNode" pkg="chassis_ctrl" type="moduanNode" output="log">
    <param name="post_bind_verify_enabled" value="true" />
</node>
<node name="pointAINode" pkg="chassis_ctrl" type="pointAI.py" output="log">
    <env name="ROSCONSOLE_FORMAT" value="[${severity}] ${message}" />
    <rosparam command="load" file="$(find chassis_ctrl)/config/bind_point_classification.yaml" />
    <param name="bind_classification_config_path" value="$(find chassis_ctrl)/config/bind_point_classification.yaml" />
</node>
```

- [ ] **Step 4: Run the integration regression test, unit suites, build, and launch expansion**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_moduan_skips_bound_points_and_calls_post_bind_verifier -v
python3 -m unittest src.chassis_ctrl.test.test_bind_point_classification src/chassis_ctrl/test/test_pointai_order.py -v
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1
roslaunch --nodes chassis_ctrl run.launch
```

Expected: PASS, and `roslaunch --nodes` should list `pointAINode`, `moduanNode`, `suoquNode`, `gripper_tf_broadcaster`, and `stable_point_tf_broadcaster`.

- [ ] **Step 5: Commit the ROS wiring**

```bash
git add src/chassis_ctrl/src/moduanNode.cpp src/chassis_ctrl/launch/run.launch src/chassis_ctrl/test/test_pointai_order.py
git commit -m "feat: wire bind classification into moduan execution"
```

### Task 6: Final Verification Of The Full Classification Path

**Files:**
- Modify: any touched files above as needed
- Test: integrated verification only

- [ ] **Step 1: Run Python syntax checks for the touched scripts**

Run:

```bash
python3 -m py_compile src/chassis_ctrl/scripts/bind_point_classification.py src/chassis_ctrl/scripts/pointAI.py src/chassis_ctrl/scripts/stable_point_tf_broadcaster.py
```

Expected: PASS.

- [ ] **Step 2: Run the Python regression suite**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_bind_point_classification src/chassis_ctrl/test/test_pointai_order.py -v
```

Expected: PASS.

- [ ] **Step 3: Rebuild the catkin packages that own the messages and runtime nodes**

Run:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1
```

Expected: PASS.

- [ ] **Step 4: Run launch expansion checks for the end-to-end bringup**

Run:

```bash
roslaunch --nodes chassis_ctrl run.launch
roslaunch --nodes chassis_ctrl api.launch
```

Expected:
- `run.launch` expands with the pointAI/moduan/suoqu/TF nodes.
- `api.launch` still expands cleanly without trying to start the new verification service itself.

- [ ] **Step 5: Smoke-test the new evidence outputs after one manual bind-check cycle**

Run:

```bash
ls -R src/chassis_ctrl/data/bind_evidence | head
tail -n 5 src/chassis_ctrl/data/bind_classification_events.jsonl
```

Expected:
- At least one `session_id/point_<idx>` directory exists with `before_*` or `after_*` files.
- The JSONL tail includes `phase`, `mode`, `bind_state` or `result`, `bind_score`, `evidence_quality`, and `decision_reason`.
