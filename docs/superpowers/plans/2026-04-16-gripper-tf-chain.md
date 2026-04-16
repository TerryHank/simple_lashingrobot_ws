# Gripper TF Chain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a dedicated background TF broadcaster that publishes `Scepter_depth_frame -> gripper_frame` from YAML config, wire it into `api.launch`, and remove old workspace-owned business TF publishers so the new TF chain is the only non-SDK business transform source.

**Architecture:** Keep the Scepter SDK TF tree unchanged and introduce one independent ROS Python node in `chassis_ctrl` that owns the business-space `gripper_frame` link. Visual and fast-image code stop publishing `aruco_*` and `gripper_frame` transforms and remain TF consumers only.

**Tech Stack:** ROS Noetic, `rospy`, `tf2_ros`, `geometry_msgs/TransformStamped`, YAML config, roslaunch, Python unittest text assertions, catkin.

---

## File Structure

### New files

- Create: `src/chassis_ctrl/config/gripper_tf.yaml`
  - Stores `parent_frame`, `child_frame`, `translation`, `rotation_rpy`.
- Create: `src/chassis_ctrl/scripts/gripper_tf_broadcaster.py`
  - Loads YAML config and continuously publishes `Scepter_depth_frame -> gripper_frame`.
- Create: `src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`
  - Focused unit tests for config loading and transform creation logic if the broadcaster is implemented with small helpers.

### Modified files

- Modify: `src/chassis_ctrl/launch/api.launch`
  - Add the new broadcaster node to startup.
- Modify: `src/chassis_ctrl/scripts/pointAI.py`
  - Remove workspace-owned `aruco_*` / `gripper_frame` TF publication while keeping TF lookup/consumer behavior.
- Modify: `src/fast_image_solve/scripts/vision.py`
  - Same cleanup as `pointAI.py`.
- Modify: `src/fast_image_solve/src/5.18auto.cpp`
  - Remove workspace-owned `gripper_frame` / helper TF publication.
- Modify: `src/fast_image_solve/include/fast_image_solve/5.18auto.hpp`
  - Remove declarations related to the old business TF broadcasters.
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
  - Add assertions for the new config file, new launch integration, and removal of old business TF publication in workspace code.

### Verification files/commands

- Test: `src/chassis_ctrl/test/test_pointai_order.py`
- Test: `src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`
- Build: `catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1`
- Syntax: `python3 -m py_compile src/chassis_ctrl/scripts/gripper_tf_broadcaster.py src/chassis_ctrl/scripts/pointAI.py src/fast_image_solve/scripts/vision.py`

---

### Task 1: Lock the New TF Ownership Rules in Tests

**Files:**
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
- Create: `src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`

- [ ] **Step 1: Add failing assertions for the new config file and `api.launch` integration**

Add tests like:

```python
def test_gripper_tf_yaml_exists_with_pose_structure(self):
    config_path = CHASSIS_CTRL_DIR / "config" / "gripper_tf.yaml"
    self.assertTrue(config_path.exists())
    config_text = config_path.read_text(encoding="utf-8")
    self.assertIn("parent_frame: Scepter_depth_frame", config_text)
    self.assertIn("child_frame: gripper_frame", config_text)
    self.assertIn("translation_mm:", config_text)
    self.assertIn("rotation_rpy:", config_text)


def test_api_launch_starts_gripper_tf_broadcaster(self):
    launch_text = (CHASSIS_CTRL_DIR / "launch" / "api.launch").read_text(encoding="utf-8")
    self.assertIn('name="gripper_tf_broadcaster"', launch_text)
    self.assertIn('pkg="chassis_ctrl"', launch_text)
    self.assertIn('type="gripper_tf_broadcaster.py"', launch_text)
```

- [ ] **Step 2: Add failing assertions that old workspace business TF publishers are gone**

Extend the existing test file with checks like:

```python
def test_visual_scripts_remove_old_business_tf_publishers(self):
    pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
    vision_text = (FAST_IMAGE_SOLVE_DIR / "scripts" / "vision.py").read_text(encoding="utf-8")

    self.assertNotIn('child_frame_id = "aruco_raw_frame"', pointai_text)
    self.assertNotIn('child_frame_id = "aruco_frame"', pointai_text)
    self.assertNotIn('child_frame_id = "gripper_frame"', pointai_text)
    self.assertNotIn('child_frame_id = "aruco_raw_frame"', vision_text)
    self.assertNotIn('child_frame_id = "aruco_frame"', vision_text)
    self.assertNotIn('child_frame_id = "gripper_frame"', vision_text)


def test_fast_image_code_removes_old_gripper_tf_publishers(self):
    fast_cpp = (FAST_IMAGE_SOLVE_DIR / "src" / "5.18auto.cpp").read_text(encoding="utf-8")
    self.assertNotIn('child_frame_id = "gripper_frame"', fast_cpp)
    self.assertNotIn('child_frame_id = "aruco_frame"', fast_cpp)
```

- [ ] **Step 3: Add focused unit tests for broadcaster helpers**

Create `src/chassis_ctrl/test/test_gripper_tf_broadcaster.py` with tests similar to:

```python
import math
import tempfile
import unittest
from pathlib import Path

from gripper_tf_broadcaster import load_gripper_tf_config, build_transform


class GripperTFBroadcasterTest(unittest.TestCase):
    def test_load_gripper_tf_config_reads_pose_fields(self):
        config_text = """
parent_frame: Scepter_depth_frame
child_frame: gripper_frame
translation_mm:
  x: 100.0
  y: -200.0
  z: 300.0
rotation_rpy:
  roll: 0.0
  pitch: 0.0
  yaw: 1.57
"""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "gripper_tf.yaml"
            path.write_text(config_text, encoding="utf-8")
            config = load_gripper_tf_config(str(path))
        self.assertEqual(config["parent_frame"], "Scepter_depth_frame")
        self.assertEqual(config["child_frame"], "gripper_frame")
        self.assertAlmostEqual(config["translation"]["y"], -0.2)
        self.assertAlmostEqual(config["rotation_rpy"]["yaw"], 1.57)

    def test_build_transform_uses_parent_child_and_translation(self):
        config = {
            "parent_frame": "Scepter_depth_frame",
            "child_frame": "gripper_frame",
            "translation": {"x": 0.1, "y": 0.2, "z": 0.3},
            "rotation_rpy": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }
        transform = build_transform(config)
        self.assertEqual(transform.header.frame_id, "Scepter_depth_frame")
        self.assertEqual(transform.child_frame_id, "gripper_frame")
        self.assertAlmostEqual(transform.transform.translation.z, 0.3)
        self.assertAlmostEqual(transform.transform.rotation.w, 1.0)
```

- [ ] **Step 4: Run the tests to verify they fail before implementation**

Run:

```bash
python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py src/chassis_ctrl/test/test_gripper_tf_broadcaster.py
```

Expected:

- failures because `gripper_tf.yaml` does not exist yet
- failures because `api.launch` does not start `gripper_tf_broadcaster`
- failures because old business TF publishers still exist
- import failure for `gripper_tf_broadcaster` until the script is created

- [ ] **Step 5: Commit the red tests**

```bash
git add src/chassis_ctrl/test/test_pointai_order.py src/chassis_ctrl/test/test_gripper_tf_broadcaster.py
git commit -m "test: lock new gripper tf chain behavior"
```

---

### Task 2: Add the New Dedicated Gripper TF Broadcaster

**Files:**
- Create: `src/chassis_ctrl/config/gripper_tf.yaml`
- Create: `src/chassis_ctrl/scripts/gripper_tf_broadcaster.py`
- Modify: `src/chassis_ctrl/launch/api.launch`

- [ ] **Step 1: Create the YAML pose config**

Create `src/chassis_ctrl/config/gripper_tf.yaml`:

```yaml
parent_frame: Scepter_depth_frame
child_frame: gripper_frame

translation_mm:
  x: 0.000
  y: 0.000
  z: 0.000

rotation_rpy:
  roll: 0.000
  pitch: 0.000
  yaw: 0.000
```

- [ ] **Step 2: Implement the broadcaster helper functions first**

Start `src/chassis_ctrl/scripts/gripper_tf_broadcaster.py` with small testable helpers:

```python
#!/usr/bin/env python3

import math
import os
import yaml
import rospy
import tf_conversions
import tf2_ros

from geometry_msgs.msg import TransformStamped


def load_gripper_tf_config(config_path):
    with open(config_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    required_top = ("parent_frame", "child_frame", "translation", "rotation_rpy")
    for key in required_top:
        if key not in data:
            raise ValueError(f"missing required key: {key}")

    for key in ("x", "y", "z"):
        if key not in data["translation"]:
            raise ValueError(f"missing translation.{key}")

    for key in ("roll", "pitch", "yaw"):
        if key not in data["rotation_rpy"]:
            raise ValueError(f"missing rotation_rpy.{key}")

    return data


def build_transform(config):
    transform = TransformStamped()
    transform.header.frame_id = config["parent_frame"]
    transform.child_frame_id = config["child_frame"]
    transform.transform.translation.x = float(config["translation"]["x"])
    transform.transform.translation.y = float(config["translation"]["y"])
    transform.transform.translation.z = float(config["translation"]["z"])

    quat = tf_conversions.transformations.quaternion_from_euler(
        float(config["rotation_rpy"]["roll"]),
        float(config["rotation_rpy"]["pitch"]),
        float(config["rotation_rpy"]["yaw"]),
    )
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]
    return transform
```

- [ ] **Step 3: Implement the runtime ROS node**

Complete the same file with runtime publishing:

```python
class GripperTFBroadcaster:
    def __init__(self):
        default_config = os.path.join(
            os.path.dirname(__file__), "..", "config", "gripper_tf.yaml"
        )
        config_path = rospy.get_param("~config_path", os.path.abspath(default_config))
        self.config = load_gripper_tf_config(config_path)
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo(
            "gripper_tf_broadcaster loaded: %s -> %s, xyz=(%.4f, %.4f, %.4f), rpy=(%.4f, %.4f, %.4f)",
            self.config["parent_frame"],
            self.config["child_frame"],
            float(self.config["translation"]["x"]),
            float(self.config["translation"]["y"]),
            float(self.config["translation"]["z"]),
            float(self.config["rotation_rpy"]["roll"]),
            float(self.config["rotation_rpy"]["pitch"]),
            float(self.config["rotation_rpy"]["yaw"]),
        )

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            transform = build_transform(self.config)
            transform.header.stamp = rospy.Time.now()
            self.broadcaster.sendTransform(transform)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("gripper_tf_broadcaster")
    GripperTFBroadcaster().spin()
```

- [ ] **Step 4: Make the new script executable**

Run:

```bash
chmod +x src/chassis_ctrl/scripts/gripper_tf_broadcaster.py
```

Expected:

- no output
- executable bit set in git diff

- [ ] **Step 5: Wire the node into `api.launch`**

Update `src/chassis_ctrl/launch/api.launch` by adding:

```xml
    <node name="gripper_tf_broadcaster"
          pkg="chassis_ctrl"
          type="gripper_tf_broadcaster.py"
          output="log">
        <param name="config_path"
               value="$(find chassis_ctrl)/config/gripper_tf.yaml" />
        <param name="publish_rate_hz" value="10.0" />
    </node>
```

Place it after the Scepter camera include and before `topicTransNode`.

- [ ] **Step 6: Run the targeted tests**

Run:

```bash
python3 -m unittest src/chassis_ctrl/test/test_gripper_tf_broadcaster.py src/chassis_ctrl/test/test_pointai_order.py -k gripper
```

Expected:

- config and launch assertions pass
- broadcaster helper tests pass

- [ ] **Step 7: Commit the new broadcaster**

```bash
git add src/chassis_ctrl/config/gripper_tf.yaml src/chassis_ctrl/scripts/gripper_tf_broadcaster.py src/chassis_ctrl/launch/api.launch src/chassis_ctrl/test/test_gripper_tf_broadcaster.py src/chassis_ctrl/test/test_pointai_order.py
git commit -m "feat: add dedicated gripper tf broadcaster"
```

---

### Task 3: Remove Old Workspace Business TF Publishers and Keep TF Consumption Only

**Files:**
- Modify: `src/chassis_ctrl/scripts/pointAI.py`
- Modify: `src/fast_image_solve/scripts/vision.py`
- Modify: `src/fast_image_solve/src/5.18auto.cpp`
- Modify: `src/fast_image_solve/include/fast_image_solve/5.18auto.hpp`
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Remove old publisher methods from `pointAI.py`**

Delete the methods that create and publish business frames, including blocks like:

```python
def publish_aruco_frame_transform(self):
    ...

def publish_tf_transform(self):
    ...

def publish_gripper_tf_transform(self):
    ...
```

Also remove the initialization call:

```python
self.publish_gripper_tf_transform()
```

And drop the unused broadcaster member if nothing else in the file needs it:

```python
self.tf_broadcaster = tf2_ros.TransformBroadcaster()
```

- [ ] **Step 2: Keep TF lookup behavior in `pointAI.py` but never recreate the chain**

If `transform_to_gripper_frame()` remains, keep only the consumer path:

```python
transform = self.tf_buffer.lookup_transform(
    "gripper_frame",
    "Scepter_depth_frame",
    rospy.Time(0),
    rospy.Duration(1.0),
)
```

Remove any fallback publication logic and remove per-point transform broadcasts like:

```python
transform_stamped.header.frame_id = "gripper_frame"
self.tf_broadcaster.sendTransform(transform_stamped)
```

- [ ] **Step 3: Apply the same cleanup to `vision.py`**

Mirror the `pointAI.py` cleanup:

```python
# remove publishers
def publish_aruco_frame_transform(...): ...
def publish_tf_transform(...): ...
def publish_gripper_tf_transform(...): ...

# keep only lookup_transform consumer behavior
transform = self.tf_buffer.lookup_transform(
    "gripper_frame",
    "Scepter_depth_frame",
    rospy.Time(0),
    rospy.Duration(1.0),
)
```

- [ ] **Step 4: Remove workspace-owned `gripper_frame` publishing from fast image C++**

In `src/fast_image_solve/src/5.18auto.cpp`, remove code blocks like:

```cpp
transform.header.frame_id = "Scepter_color_frame";
transform.child_frame_id = "gripper_frame";
tf_broadcaster_.sendTransform(transform);
```

Also remove per-point helper TF publication such as:

```cpp
point_transform.header.frame_id = "gripper_frame";
tf_broadcaster_.sendTransform(point_transform);
```

In the header, remove declarations that only exist for those publishers.

- [ ] **Step 5: Run the tests for old-publisher removal**

Run:

```bash
python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py
```

Expected:

- the new “old business TF publishers removed” assertions pass
- no remaining test expects `aruco_*` or workspace-owned `gripper_frame` broadcasters

- [ ] **Step 6: Commit the cleanup**

```bash
git add src/chassis_ctrl/scripts/pointAI.py src/fast_image_solve/scripts/vision.py src/fast_image_solve/src/5.18auto.cpp src/fast_image_solve/include/fast_image_solve/5.18auto.hpp src/chassis_ctrl/test/test_pointai_order.py
git commit -m "refactor: remove old workspace tf publishers"
```

---

### Task 4: Verify End-to-End Behavior Through Build and Runtime Checks

**Files:**
- No new code files
- Verify: `src/chassis_ctrl/launch/api.launch`
- Verify: `src/chassis_ctrl/scripts/gripper_tf_broadcaster.py`
- Verify: `src/chassis_ctrl/scripts/pointAI.py`
- Verify: `src/fast_image_solve/scripts/vision.py`

- [ ] **Step 1: Run Python syntax checks**

Run:

```bash
python3 -m py_compile \
  src/chassis_ctrl/scripts/gripper_tf_broadcaster.py \
  src/chassis_ctrl/scripts/pointAI.py \
  src/fast_image_solve/scripts/vision.py
```

Expected:

- no output
- exit code 0

- [ ] **Step 2: Run the full targeted test suite**

Run:

```bash
python3 -m unittest \
  src/chassis_ctrl/test/test_pointai_order.py \
  src/chassis_ctrl/test/test_gripper_tf_broadcaster.py
```

Expected:

- all tests pass

- [ ] **Step 3: Run catkin build**

Run:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1
```

Expected:

- build succeeds
- `chassis_ctrl` and `fast_image_solve` targets finish without compile errors

- [ ] **Step 4: Runtime-check the transform from `api.launch`**

Run:

```bash
source devel/setup.bash
roslaunch chassis_ctrl api.launch
```

In another terminal:

```bash
source devel/setup.bash
rosrun tf tf_echo Scepter_depth_frame gripper_frame
```

Expected:

- transform exists
- translation/rotation values match `config/gripper_tf.yaml`
- no duplicate `gripper_frame` publisher warnings

- [ ] **Step 5: Check launch and diff hygiene**

Run:

```bash
git diff --check -- \
  src/chassis_ctrl/config/gripper_tf.yaml \
  src/chassis_ctrl/scripts/gripper_tf_broadcaster.py \
  src/chassis_ctrl/launch/api.launch \
  src/chassis_ctrl/scripts/pointAI.py \
  src/fast_image_solve/scripts/vision.py \
  src/fast_image_solve/src/5.18auto.cpp \
  src/fast_image_solve/include/fast_image_solve/5.18auto.hpp \
  src/chassis_ctrl/test/test_pointai_order.py \
  src/chassis_ctrl/test/test_gripper_tf_broadcaster.py
```

Expected:

- no whitespace or patch formatting issues

- [ ] **Step 6: Commit the verified end state**

```bash
git add src/chassis_ctrl/config/gripper_tf.yaml src/chassis_ctrl/scripts/gripper_tf_broadcaster.py src/chassis_ctrl/launch/api.launch src/chassis_ctrl/scripts/pointAI.py src/fast_image_solve/scripts/vision.py src/fast_image_solve/src/5.18auto.cpp src/fast_image_solve/include/fast_image_solve/5.18auto.hpp src/chassis_ctrl/test/test_pointai_order.py src/chassis_ctrl/test/test_gripper_tf_broadcaster.py
git commit -m "feat: switch to dedicated gripper tf chain"
```

---

## Self-Review

### Spec coverage

- Dedicated YAML pose config: covered in Task 2
- Independent broadcaster node: covered in Task 2
- `api.launch` autostart: covered in Task 2
- Keep SDK TF chain unchanged: respected across Tasks 2 and 3
- Remove old workspace-owned `aruco_*` / `gripper_frame` publishers: covered in Task 3
- Keep visual code as TF consumers only: covered in Task 3
- Verification with build/runtime TF echo: covered in Task 4

No spec gaps found.

### Placeholder scan

- No `TODO`/`TBD`
- All tasks include concrete files, code snippets, and commands
- No “similar to previous task” shorthand

### Type consistency

- Config keys consistently use `parent_frame`, `child_frame`, `translation`, `rotation_rpy`
- Runtime TF frames consistently use `Scepter_depth_frame` and `gripper_frame`
- Test file names and commands match the planned file paths

Plan complete and saved to `docs/superpowers/plans/2026-04-16-gripper-tf-chain.md`. Two execution options:

1. Subagent-Driven (recommended) - I dispatch a fresh subagent per task, review between tasks, fast iteration
2. Inline Execution - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
