# Pseudo-SLAM Bind Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a scan-first pseudo-SLAM workflow that builds bind paths in `map`, then executes global work from the saved scan path without re-running vision or adaptive height.

**Architecture:** Extend `suoquNode.cpp` into two explicit modes: scan-only and bind-from-scan. Keep `pointAI.py` as the point producer, add a scan mode that exports whole-ROI world points in `map`, publish `map -> Scepter_depth_frame` from `suoquNode`, and add a dedicated `stable_point_tf_broadcaster.py` node for final bind-point TF lifecycle and area archiving.

**Tech Stack:** ROS Noetic, roscpp, rospy, tf2_ros, geometry_msgs, Python unittest, catkin

---

### Task 1: Add regression tests for the new workflow surfaces

**Files:**
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
- Modify: `src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`
- Modify: `src/chassis_ctrl/scripts/pointAI.py`
- Modify: `src/chassis_ctrl/src/suoquNode.cpp`
- Modify: `src/chassis_ctrl/src/topics_transfer.cpp`
- Modify: `src/chassis_ctrl/scripts/debug_button_node.py`
- Create: `src/chassis_ctrl/scripts/stable_point_tf_broadcaster.py`

- [ ] **Step 1: Write failing tests for pseudo-SLAM entry points and TF publishers**

Add tests that assert:
- `pointAI.py` exposes a scan mode constant / branch distinct from adaptive and bind-check.
- `suoquNode.cpp` contains `map` and publishes `map -> Scepter_depth_frame`.
- `topics_transfer.cpp` and `debug_button_node.py` expose a new scan-build entry.
- `stable_point_tf_broadcaster.py` exists and references `/coordinate_point`, `/cabin/area_progress`, `bind_point_`, and `area_` frames.
- global work execution path in `suoquNode.cpp` references pseudo-SLAM bind path and no adaptive-height retry in scan-based execution.

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest src.chassis_ctrl.test.test_gripper_tf_broadcaster.GripperTFBroadcasterTest`
Expected: FAIL on the new assertions because the scan workflow and new broadcaster do not exist yet.

- [ ] **Step 3: Commit the failing tests only if needed for checkpointing**

```bash
git add src/chassis_ctrl/test/test_pointai_order.py src/chassis_ctrl/test/test_gripper_tf_broadcaster.py
git commit -m "test: add pseudo slam workflow regression coverage"
```

### Task 2: Add scan-mode point production in pointAI

**Files:**
- Modify: `src/chassis_ctrl/scripts/pointAI.py`
- Modify: `src/fast_image_solve/srv/ProcessImage.srv` only if a new request mode constant is required
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Extend tests to describe scan-mode behavior**

Add assertions that the source contains:
- a scan mode constant such as `PROCESS_IMAGE_MODE_SCAN_ONLY`
- scan-mode selection logic
- world points filtered by planning workspace and full-ROI behavior
- no 2x2 restriction in scan mode

- [ ] **Step 2: Run the focused test to verify it fails**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_pointai_supports_scan_mode_for_pseudo_slam`
Expected: FAIL because scan mode does not exist yet.

- [ ] **Step 3: Implement minimal scan-mode support**

In `pointAI.py`:
- add a scan-only request mode constant
- reuse full-ROI candidate extraction
- transform points into `map` world coordinates using the completed TF chain
- filter only by scan planning workspace, not by bind 2x2 rules
- write scan points to a distinct output structure consumable by `suoquNode`

- [ ] **Step 4: Run the focused test and then the whole pointAI test file**

Run:
- `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_pointai_supports_scan_mode_for_pseudo_slam`
- `python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py`
Expected: PASS

### Task 3: Publish `map -> Scepter_depth_frame` from suoqu and split scan/bind execution

**Files:**
- Modify: `src/chassis_ctrl/src/suoquNode.cpp`
- Modify: `src/chassis_ctrl/CMakeLists.txt` if new link deps are needed
- Modify: `src/chassis_ctrl/package.xml` if new deps are needed
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Write failing tests for TF publication and pseudo-SLAM services**

Add assertions that `suoquNode.cpp` contains:
- `map`
- `Scepter_depth_frame`
- TF broadcaster include/use
- pseudo-SLAM scan service / entry
- pseudo-SLAM bind-path file usage
- scan-based global work skips adaptive height and skips failed points

- [ ] **Step 2: Run the focused test to verify it fails**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_supports_map_and_pseudo_slam_execution`
Expected: FAIL

- [ ] **Step 3: Implement the minimal C++ changes**

In `suoquNode.cpp`:
- add a TF broadcaster for `map -> Scepter_depth_frame`
- publish from the existing cabin-state read loop or a timer-safe path
- add a scan-only service/flow that walks planned path and requests scan mode from `pointAI`
- write merged scan results to `pseudo_slam_points.json`
- generate `pseudo_slam_bind_path.json`
- add bind-from-scan execution path that reads saved path, disables adaptive height, and skips failed points instead of aborting

- [ ] **Step 4: Run the focused test and compile-target verification**

Run:
- `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_supports_map_and_pseudo_slam_execution`
- `catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1`
Expected: PASS

### Task 4: Add stable bind-point TF broadcaster

**Files:**
- Create: `src/chassis_ctrl/scripts/stable_point_tf_broadcaster.py`
- Modify: `src/chassis_ctrl/launch/*.launch`
- Test: `src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`

- [ ] **Step 1: Write failing tests for stable-point broadcaster behavior**

Add assertions for:
- file existence
- subscriptions to `/coordinate_point` and `/cabin/area_progress`
- 2-frame / ±4mm stability rules
- `bind_point_` and `area_<N>_point_` naming

- [ ] **Step 2: Run the focused test to verify it fails**

Run: `python3 -m unittest src.chassis_ctrl.test.test_gripper_tf_broadcaster.GripperTFBroadcasterTest.test_stable_point_tf_broadcaster_exists_and_tracks_bind_points`
Expected: FAIL

- [ ] **Step 3: Implement the Python node and launch wiring**

Implement a dedicated node that:
- subscribes to `/coordinate_point`
- subscribes to `/cabin/area_progress`
- evaluates final bind-point stability with 2 frames / ±4mm in z
- publishes `map -> bind_point_<i>`
- archives previous area points as `map -> area_<N>_point_<i>`
- keeps archived frames after area switches

- [ ] **Step 4: Run tests and syntax checks**

Run:
- `python3 -m unittest src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`
- `python3 -m py_compile src/chassis_ctrl/scripts/stable_point_tf_broadcaster.py`
Expected: PASS

### Task 5: Add front-end/control entry points for pseudo-SLAM scan

**Files:**
- Modify: `src/chassis_ctrl/src/topics_transfer.cpp`
- Modify: `src/chassis_ctrl/scripts/debug_button_node.py`
- Possibly modify: launch/start scripts if this workflow needs a new runtime node list
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Write failing tests for the new scan-build button plumbing**

Add assertions that:
- a new `/web/cabin/...` scan topic or service mapping exists
- debug button metadata includes the scan-build action

- [ ] **Step 2: Run the focused test to verify it fails**

Run: `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_frontend_exposes_pseudo_slam_scan_entry`
Expected: FAIL

- [ ] **Step 3: Implement minimal routing changes**

Wire the new front-end entry through `topics_transfer.cpp` and `debug_button_node.py` to the scan-only workflow in `suoquNode`.

- [ ] **Step 4: Run the focused test and whole Python/C++ regression suite**

Run:
- `python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_frontend_exposes_pseudo_slam_scan_entry`
- `python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`
Expected: PASS

### Task 6: Final verification of the integrated workflow

**Files:**
- Modify: any touched files above as needed
- Test: integrated commands only

- [ ] **Step 1: Run syntax verification for Python nodes**

Run: `python3 -m py_compile src/chassis_ctrl/scripts/pointAI.py src/chassis_ctrl/scripts/gripper_tf_broadcaster.py src/chassis_ctrl/scripts/stable_point_tf_broadcaster.py src/fast_image_solve/scripts/vision.py`
Expected: PASS

- [ ] **Step 2: Run package tests**

Run: `python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`
Expected: PASS

- [ ] **Step 3: Run build verification**

Run: `catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1`
Expected: PASS

- [ ] **Step 4: Run launch expansion checks**

Run:
- `roslaunch --nodes chassis_ctrl pointai_tf_verify.launch`
- `roslaunch --nodes chassis_ctrl run.launch`
Expected: both commands succeed and show the expected pointAI / broadcaster nodes without unexpected includes.
