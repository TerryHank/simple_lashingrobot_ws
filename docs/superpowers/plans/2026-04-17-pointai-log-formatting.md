# PointAI Log Formatting Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `pointAI.py` print readable multi-line summary logs for point filtering without changing other nodes.

**Architecture:** Add a focused formatter/helper inside `pointAI.py` that assembles one block-style debug message from the current detection summary and emits one throttled warning instead of several long single-line warnings. Cover the formatter with a unit test in the existing Python test module.

**Tech Stack:** Python 3, rospy, unittest

---

### Task 1: Add a failing test for block-style summary formatting

**Files:**
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
- Modify: `src/chassis_ctrl/scripts/pointAI.py`

- [ ] **Step 1: Write the failing test**

```python
def test_build_detection_summary_log_uses_multiline_block(self):
    processor = pointAI.ImageProcessor.__new__(pointAI.ImageProcessor)
    processor.travel_range_max_x_mm = 320.0
    processor.travel_range_max_y_mm = 360.0
    message = processor.build_detection_summary_log(
        request_mode=pointAI.PROCESS_IMAGE_MODE_ADAPTIVE_HEIGHT,
        raw_candidate_count=24,
        duplicate_removed_count=0,
        in_range_candidate_count=0,
        out_of_range_point_count=24,
        selected_count=0,
        output_count=0,
        out_of_range_reason_counts={"X小于0": 24, "Y小于0": 24},
        out_of_range_samples=["idx=8,pix=(23,434),coord=(-301.0,-67.0,255.0),原因=X小于0+Y小于0"],
    )
    self.assertIn("pointAI调试:\n", message)
    self.assertIn("  模式: adaptive_height", message)
    self.assertIn("  样例:\n", message)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_build_detection_summary_log_uses_multiline_block`
Expected: FAIL because `build_detection_summary_log` does not exist yet.

- [ ] **Step 3: Write minimal implementation**

```python
def build_detection_summary_log(...):
    ...
```

Add a helper that returns one multi-line summary string and replace the scattered warn logs in `pre_img()` with a single throttled block log plus a shorter waiting message in `wait_for_stable_point_coords()`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_build_detection_summary_log_uses_multiline_block`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/chassis_ctrl/scripts/pointAI.py src/chassis_ctrl/test/test_pointai_order.py docs/superpowers/plans/2026-04-17-pointai-log-formatting.md
git commit -m "refactor: improve pointAI debug log formatting"
```

### Task 2: Verify the focused change stays safe

**Files:**
- Modify: `src/chassis_ctrl/scripts/pointAI.py`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Run the focused unit test file**

Run: `python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py`
Expected: PASS

- [ ] **Step 2: Run syntax verification**

Run: `python3 -m py_compile src/chassis_ctrl/scripts/pointAI.py`
Expected: PASS with no output

- [ ] **Step 3: Report verified behavior**

Summarize that `pointAI.py` now emits one multi-line detection summary block and a shorter retry message, with evidence from the commands above.
