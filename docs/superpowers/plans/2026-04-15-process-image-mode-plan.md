# ProcessImage Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add request-mode-aware behavior to `/pointAI/process_image` so adaptive-height and bind-check calls use different Z-gating and return actionable bind-check failures.

**Architecture:** Extend `ProcessImage.srv` with a request mode and richer response, then route the two driver call sites through different request modes. Keep the stable-Z polling loop in the vision node and add bind-mode height-limit evaluation before releasing points.

**Tech Stack:** ROS Noetic services/messages, Python ROS service nodes, C++ ROS clients, Python `unittest`, catkin build

---

### Task 1: Add failing tests for the new service schema and mode logic

**Files:**
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] Add failing tests asserting `ProcessImage.srv` contains mode constants, `request_mode`, and failure-report response fields.
- [ ] Add failing tests for helper logic:
  - adaptive mode accepts stable points without 94mm filtering
  - bind mode rejects stable points above 94mm and reports violating indices/Z values
- [ ] Run `source /home/hyq-/simple_lashingrobot_ws/devel/setup.bash && python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py` and confirm failure.

### Task 2: Extend the service schema

**Files:**
- Modify: `src/fast_image_solve/srv/ProcessImage.srv`

- [ ] Add mode constants and `request_mode` to the request section.
- [ ] Add `success`, `message`, `out_of_height_count`, `out_of_height_point_indices`, and `out_of_height_z_values` to the response section.

### Task 3: Implement mode-aware vision behavior

**Files:**
- Modify: `src/chassis_ctrl/scripts/pointAI.py`
- Modify: `src/fast_image_solve/scripts/vision.py`

- [ ] Add mode constants/helper accessors with adaptive-height as the default fallback.
- [ ] Add helpers that detect points above 94mm and format a detailed response message.
- [ ] Update the stable polling loop to:
  - keep polling until a stable 3-frame window is found
  - return success for adaptive mode once stable
  - return success for bind mode only if all stable points are within 94mm
  - return failure with detailed out-of-height data for bind mode otherwise
- [ ] Update all `ProcessImageResponse(...)` return paths to populate the new fields.

### Task 4: Update driver call sites

**Files:**
- Modify: `src/chassis_ctrl/src/suoquNode.cpp`
- Modify: `src/chassis_ctrl/src/moduanNode.cpp`

- [ ] Set adaptive-height calls to `MODE_ADAPTIVE_HEIGHT`.
- [ ] Set bind calls to `MODE_BIND_CHECK`.
- [ ] In `suoquNode.cpp`, log adaptive-height response failures.
- [ ] In `moduanNode.cpp`, log bind-check height failures and propagate `response.message`.

### Task 5: Verify generated code and compilation

**Files:**
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] Run `python3 -m py_compile src/chassis_ctrl/scripts/pointAI.py src/fast_image_solve/scripts/vision.py src/chassis_ctrl/test/test_pointai_order.py`
- [ ] Run `source /home/hyq-/simple_lashingrobot_ws/devel/setup.bash && python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py`
- [ ] Run `catkin_make --pkg fast_image_solve chassis_ctrl -j1`
