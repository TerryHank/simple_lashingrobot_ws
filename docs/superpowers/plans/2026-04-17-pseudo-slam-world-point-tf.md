# Pseudo-SLAM World Point TF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add persistent `cabin_frame -> pseudo_slam_point_<global_idx>` TF broadcasting that mirrors scan-time world-point markers in RViz.

**Architecture:** Keep all scan-time world-point visualization inside `suoquNode.cpp`. Reuse the existing TF broadcaster for both cabin pose TF and scan-point TF, while maintaining a thread-safe cache of the current scan point set so TFs stay visible after scanning finishes.

**Tech Stack:** ROS Noetic, C++, `tf2_ros`, `visualization_msgs/MarkerArray`, Python unittest text assertions, `catkin_make`

---

### Task 1: Lock the expected TF publishing behavior in tests

**Files:**
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Write the failing test**

```python
def test_suoqu_publishes_pseudo_slam_world_point_tfs(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

    self.assertIn("pseudo_slam_point_", suoqu_text)
    self.assertIn("sendTransform", suoqu_text)
    self.assertIn("pseudo_slam_tf_points", suoqu_text)
    self.assertIn("cabin_frame", suoqu_text)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_publishes_pseudo_slam_world_point_tfs -v`
Expected: FAIL because `suoquNode.cpp` does not yet contain the new TF broadcasting logic.

- [ ] **Step 3: Write minimal implementation**

Update `suoquNode.cpp` to:
- add a thread-safe cache for current pseudo-slam world points
- refresh that cache at scan start and after each merge update
- publish `cabin_frame -> pseudo_slam_point_<global_idx>` transforms

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_publishes_pseudo_slam_world_point_tfs -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/chassis_ctrl/test/test_pointai_order.py src/chassis_ctrl/src/suoquNode.cpp
git commit -m "feat: publish pseudo slam point tf frames"
```

### Task 2: Keep pseudo-slam point TFs alive after scan completion

**Files:**
- Modify: `src/chassis_ctrl/src/suoquNode.cpp`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Write the failing test**

```python
def test_suoqu_keeps_pseudo_slam_tf_points_for_continuous_broadcast(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")

    self.assertIn("publish_pseudo_slam_point_transforms()", suoqu_text)
    self.assertIn("pseudo_slam_tf_points_mutex", suoqu_text)
    self.assertIn("read_cabin_state", suoqu_text)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_keeps_pseudo_slam_tf_points_for_continuous_broadcast -v`
Expected: FAIL because the continuous TF broadcast helper is not yet present.

- [ ] **Step 3: Write minimal implementation**

Add:
- a global cache plus mutex
- a helper that builds and broadcasts a `std::vector<geometry_msgs::TransformStamped>`
- a call from the existing cabin-state loop so TFs remain visible after scan completion

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_keeps_pseudo_slam_tf_points_for_continuous_broadcast -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/chassis_ctrl/src/suoquNode.cpp src/chassis_ctrl/test/test_pointai_order.py
git commit -m "feat: keep pseudo slam point tfs alive during rviz display"
```

### Task 3: Full verification

**Files:**
- Verify only

- [ ] **Step 1: Run targeted Python test suite**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order src.chassis_ctrl.test.test_gripper_tf_broadcaster`
Expected: PASS with all tests green.

- [ ] **Step 2: Run Python syntax verification**

Run: `python3 -m py_compile src/chassis_ctrl/scripts/pointAI.py`
Expected: PASS with no output.

- [ ] **Step 3: Run workspace build**

Run: `catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1`
Expected: PASS with `Built target suoquNode`.

- [ ] **Step 4: Verify launch node set remains intact**

Run: `roslaunch --nodes chassis_ctrl run.launch`
Expected:
- `/suoquNode`
- `/moduanNode`
- `/gripper_tf_broadcaster`
- `/pointAINode`
- `/stable_point_tf_broadcaster`

- [ ] **Step 5: Commit**

```bash
git add docs/superpowers/specs/2026-04-17-pseudo-slam-world-point-tf-design.md \
        docs/superpowers/plans/2026-04-17-pseudo-slam-world-point-tf.md \
        src/chassis_ctrl/src/suoquNode.cpp \
        src/chassis_ctrl/test/test_pointai_order.py
git commit -m "feat: mirror pseudo slam world points as tf frames"
```
