# Area Progress Broadcast Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Broadcast area-level progress from the global cabin workflow so external listeners can know which area is active, which area just finished, and whether the system can move to the next area.

**Architecture:** Add a lightweight `chassis_ctrl/AreaProgress` message, generate it in `chassis_ctrl`, and publish it from the global area loop in `suoquNode.cpp`. The loop will emit progress at area entry, after a successful area bind, and after all areas are finished.

**Tech Stack:** ROS Noetic, catkin message generation, C++, Python `unittest` source checks

---

### Task 1: Add test coverage for the new progress interface

**Files:**
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Write the failing tests**

Add tests that assert:
- `src/chassis_ctrl/msg/AreaProgress.msg` exists with the five required fields
- `src/chassis_ctrl/CMakeLists.txt` lists `AreaProgress.msg`
- `src/chassis_ctrl/src/suoquNode.cpp` advertises `/cabin/area_progress`
- `src/chassis_ctrl/src/suoquNode.cpp` publishes progress updates

- [ ] **Step 2: Run test to verify it fails**

Run: `source /home/hyq-/simple_lashingrobot_ws/devel/setup.bash && python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py`
Expected: FAIL because the new message and publisher do not exist yet.

### Task 2: Add the ROS message and build wiring

**Files:**
- Create: `src/chassis_ctrl/msg/AreaProgress.msg`
- Modify: `src/chassis_ctrl/CMakeLists.txt`

- [ ] **Step 1: Write minimal message definition**

Create a message with:
```text
int32 current_area_index
int32 total_area_count
int32 just_finished_area_index
bool ready_for_next_area
bool all_done
```

- [ ] **Step 2: Register the message**

Add `AreaProgress.msg` to `add_message_files(...)` in `src/chassis_ctrl/CMakeLists.txt`.

### Task 3: Publish progress from the global area loop

**Files:**
- Modify: `src/chassis_ctrl/src/suoquNode.cpp`

- [ ] **Step 1: Add publisher and helper**

Include the generated header, declare a `ros::Publisher pub_area_progress`, and add a helper like:
```cpp
void publish_area_progress(
    int current_area_index,
    int total_area_count,
    int just_finished_area_index,
    bool ready_for_next_area,
    bool all_done)
```

- [ ] **Step 2: Broadcast on area entry**

At the start of each area iteration, publish:
- `current_area_index = current area number (1-based)`
- `total_area_count = initial number of areas`
- `just_finished_area_index = 0`
- `ready_for_next_area = false`
- `all_done = false`

- [ ] **Step 3: Broadcast after a successful bind**

After `/moduan/sg` succeeds, publish:
- `current_area_index = next area number if one exists, otherwise current`
- `total_area_count = initial number of areas`
- `just_finished_area_index = finished area number`
- `ready_for_next_area = true`
- `all_done = false`

- [ ] **Step 4: Broadcast all-done state**

After the loop finishes, publish:
- `current_area_index = total_area_count`
- `total_area_count = total_area_count`
- `just_finished_area_index = total_area_count`
- `ready_for_next_area = false`
- `all_done = true`

### Task 4: Verify end to end

**Files:**
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Run unit tests**

Run: `source /home/hyq-/simple_lashingrobot_ws/devel/setup.bash && python3 -m unittest src/chassis_ctrl/test/test_pointai_order.py`
Expected: PASS

- [ ] **Step 2: Run Python syntax verification**

Run: `python3 -m py_compile src/chassis_ctrl/test/test_pointai_order.py`
Expected: PASS

- [ ] **Step 3: Build chassis_ctrl**

Run: `catkin_make --pkg chassis_ctrl -j1`
Expected: build succeeds and generates `AreaProgress` message artifacts.
