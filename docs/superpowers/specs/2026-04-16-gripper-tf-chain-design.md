# Gripper TF Chain Design

## Context

The current workspace already receives camera-internal TF frames from the Scepter SDK in `/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/src/ScepterROS/src/scepter_manager.cpp`. Those SDK frames should remain untouched.

The project also contains additional business-space TF publishing inside:

- `src/chassis_ctrl/scripts/pointAI.py`
- `src/fast_image_solve/scripts/vision.py`
- `src/fast_image_solve/src/5.18auto.cpp`

Those publishers currently mix visual processing with TF management and may publish `aruco_*` / `gripper_frame` chains that can pollute the runtime TF tree.

The goal is to replace that mixed logic with a single clean business TF link:

`Scepter_depth_frame -> gripper_frame`

This new link should be published by an independent background node that is started from `src/chassis_ctrl/launch/api.launch`.

## Goals

- Keep all camera SDK TF definitions unchanged.
- Publish exactly one business-space TF link:
  - parent: `Scepter_depth_frame`
  - child: `gripper_frame`
- Move TF publishing responsibility out of visual algorithm nodes.
- Load gripper pose from a dedicated configuration file in a pose-oriented format.
- Start the new TF broadcaster automatically from `api.launch`.
- Remove old non-SDK `gripper_frame` / `aruco_*` business TF publishers in this workspace so only the new business TF chain remains.
- Keep visual nodes as TF consumers only.

## Non-Goals

- Do not modify the Scepter SDK package or its internal TF chain.
- Do not redesign the full SLAM/robot frame tree.
- Do not change camera topics, point detection, or binding selection behavior in this task.
- Do not add dynamic calibration UI in this task.

## Recommended Approach

### Option A: Independent ROS node with YAML pose config

Create a dedicated node in `chassis_ctrl` that:

- reads `config/gripper_tf.yaml`
- publishes `Scepter_depth_frame -> gripper_frame`
- republishes continuously in the background
- is started by `api.launch`

Why this option is recommended:

- clean separation between TF infrastructure and visual algorithms
- easy to inspect and debug with `tf_echo` / RViz
- no coupling to detection lifecycle
- easy to tune by editing one config file

### Option B: `static_transform_publisher` directly in launch

This is simpler but not recommended because:

- pose values are less maintainable in XML
- future adjustment is awkward
- there is no dedicated code path for validation/logging

### Option C: Reuse existing visual nodes to publish TF

This is explicitly rejected because:

- visual nodes should not own business TF infrastructure
- startup order becomes fragile
- duplicate TF publication is easy to reintroduce

## Final Design

### 1. New Config File

Add a dedicated config file:

`src/chassis_ctrl/config/gripper_tf.yaml`

Format:

```yaml
parent_frame: Scepter_depth_frame
child_frame: gripper_frame

translation:
  x: 0.000
  y: 0.000
  z: 0.000

rotation_rpy:
  roll: 0.000
  pitch: 0.000
  yaw: 0.000
```

Conventions:

- translation unit: meters
- rotation unit: radians
- pose meaning: transform from `Scepter_depth_frame` to `gripper_frame`

### 2. New Independent Broadcaster Node

Add a new node under `chassis_ctrl`, for example:

`src/chassis_ctrl/scripts/gripper_tf_broadcaster.py`

Responsibilities:

- load YAML config at startup
- validate required fields
- build `geometry_msgs/TransformStamped`
- publish `Scepter_depth_frame -> gripper_frame`
- keep publishing in the background at a small fixed rate so runtime consumers always see the same chain
- log loaded pose parameters once at startup

The node must not:

- publish `aruco_raw_frame`
- publish `aruco_frame`
- publish any point-by-point helper frames
- depend on image callbacks

### 3. Launch Integration

Update:

`src/chassis_ctrl/launch/api.launch`

Add the new TF broadcaster node after the camera include so the business TF chain is present whenever the camera stack is brought up from the API entrypoint.

Expected runtime chain after startup:

- SDK-owned chain:
  - `Scepter_frame -> ... -> Scepter_depth_frame`
- workspace-owned business chain:
  - `Scepter_depth_frame -> gripper_frame`

### 4. Remove Old Business TF Publishers

Remove workspace-owned business TF publication from:

- `src/chassis_ctrl/scripts/pointAI.py`
- `src/fast_image_solve/scripts/vision.py`
- `src/fast_image_solve/src/5.18auto.cpp`
- `src/fast_image_solve/include/fast_image_solve/5.18auto.hpp`

Specifically remove publication of:

- `aruco_raw_frame`
- `aruco_frame`
- `gripper_frame`
- per-point helper TFs that are only used as visual debug artifacts in the same business space

If any transform lookup remains for point conversion, it should only consume:

- source: `Scepter_depth_frame`
- target: `gripper_frame`

### 5. Visual Node Behavior After Cleanup

After cleanup:

- visual nodes no longer broadcast business TF
- visual nodes may keep a TF listener/buffer only if needed for coordinate conversion
- coordinate conversion should rely on the new standalone `gripper_tf_broadcaster`

This keeps TF ownership single-source and avoids duplicate frame definitions.

## Data Flow

1. `api.launch` starts rosbridge, TF web republisher, camera SDK launch, topic transfer node, and the new gripper TF broadcaster.
2. The SDK publishes camera-internal static frames.
3. The new broadcaster publishes `Scepter_depth_frame -> gripper_frame`.
4. Visual nodes query TF when they need to convert depth-frame coordinates into gripper-frame coordinates.
5. Downstream modules consume transformed coordinates without needing the old mixed TF publishers.

## Error Handling

The new TF broadcaster should:

- fail loudly if the YAML file is missing
- fail loudly if `translation` or `rotation_rpy` fields are incomplete
- default to the configured frame names only, not hardcoded alternates
- log the exact loaded pose and frame names on startup

Visual nodes should:

- log a clear warning if `Scepter_depth_frame -> gripper_frame` is temporarily unavailable
- avoid silently creating replacement TF chains

## Verification Plan

Implementation will be considered complete only after all of the following pass:

- YAML config exists and is loaded successfully
- `api.launch` starts the new broadcaster automatically
- old workspace `gripper_frame` / `aruco_*` publishers are removed
- `rosrun tf tf_echo Scepter_depth_frame gripper_frame` returns the configured transform
- Python syntax checks pass
- affected catkin packages build successfully
- tests are updated to verify:
  - `api.launch` includes the new broadcaster
  - old business TF publishers are removed from visual scripts / fast image code
  - new config file exists with the expected pose format

## Risks

- If any old TF publisher remains active, duplicate frame ownership may still exist.
- If visual code still assumes `aruco_frame` exists, cleanup may break those consumers until all lookups are updated.
- If the configured transform is physically wrong, the TF chain will still be clean but coordinates will still be wrong. This task fixes TF ownership, not the physical calibration itself.

## Decision Summary

The project will keep the SDK TF chain intact and add exactly one new independent business TF link, `Scepter_depth_frame -> gripper_frame`, loaded from `config/gripper_tf.yaml` and started from `api.launch`. All old workspace-owned business TF publishers in visual processing code will be removed so the new TF chain is the only non-SDK business transform source.
