# Edge Pair And Global Execution Memory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让边界区域的两点钢筋组也能进入绑扎路径，并新增一份跨重启持久化、但在每次新扫描成功后整体清空的全局已执行记忆，使 `slam_precomputed` 和 `live_visual` 都服从同一套全局棋盘格判断。

**Architecture:** 继续以 [suoquNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp) 为伪 `slam` 扫描、全局网格和执行主控中心，不新增新的执行节点。扫描阶段生成全局网格与两类组（`matrix_2x2`、`edge_pair`），执行阶段统一通过新的 `bind_execution_memory.json` 做点级过滤与记账；`live_visual` 只负责重新观测点，不绕过全局网格和记忆。

**Tech Stack:** ROS Noetic, C++, `nlohmann::json`, existing `std_srvs::Trigger`/`MotionControl` services, Python `unittest`, `catkin_make`

---

## File Structure

- Modify: `src/chassis_ctrl/src/suoquNode.cpp`
  - 扩展棋盘格成员判定
  - 新增 `edge_pair` 组生成
  - 新增执行记忆的读写、过滤、成功记账
  - 让 `slam_precomputed` 与 `live_visual` 共用全局网格与执行账本

- Modify: `src/chassis_ctrl/src/moduanNode.cpp`
  - 保持 `/moduan/sg`、`/moduan/sg_precomputed(_fast)` 接口不变
  - 如需补强日志，明确哪些点被执行成功，以便上游记账

- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
  - 新增边界两点组、记忆文件、跨重启续作、重新扫描清空记忆、`live_visual` 共用账本的红绿测试

- Modify: `SLAM_V11_QUICKSTART.md`
  - 补充新记忆文件、边界两点组、扫描后记忆重置的运维说明

- Runtime output only: `src/chassis_ctrl/data/bind_execution_memory.json`
  - 不需要作为固定仓库模板提交
  - 由运行时原子创建和重写

---

### Task 1: Lock The New Behavior With Failing Tests

**Files:**
- Modify: `src/chassis_ctrl/test/test_pointai_order.py`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Write the failing tests for edge pairs and execution memory**

Add tests like:

```python
def test_pseudo_slam_bind_path_supports_edge_pair_groups(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    self.assertIn("edge_pair", suoqu_text)
    self.assertIn("group_type", suoqu_text)
    self.assertIn("selected_indices.size() == 2", suoqu_text)

def test_pseudo_slam_checkerboard_membership_no_longer_requires_both_axes(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    self.assertIn("can_form_edge_pair", suoqu_text)
    self.assertNotIn("has_horizontal_neighbor && has_vertical_neighbor", suoqu_text)

def test_suoqu_persists_bind_execution_memory_to_disk(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    self.assertIn("bind_execution_memory.json", suoqu_text)
    self.assertIn("write_bind_execution_memory_json", suoqu_text)
    self.assertIn("load_bind_execution_memory_json", suoqu_text)

def test_rescan_rebuilds_execution_memory_after_scan_outputs_are_written(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    self.assertIn("reset_bind_execution_memory_for_scan_session", suoqu_text)
    self.assertIn("write_pseudo_slam_points_json", suoqu_text)
    self.assertIn("write_pseudo_slam_bind_path_json", suoqu_text)
```

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_pseudo_slam_bind_path_supports_edge_pair_groups src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_pseudo_slam_checkerboard_membership_no_longer_requires_both_axes src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_persists_bind_execution_memory_to_disk src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_rescan_rebuilds_execution_memory_after_scan_outputs_are_written -v
```

Expected: FAIL with missing `edge_pair`, missing memory helpers, and old checkerboard condition still present.

- [ ] **Step 3: Add the live-visual/accounting red tests in the same file**

Append tests like:

```python
def test_slam_precomputed_skips_points_already_recorded_in_execution_memory(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    self.assertIn("is_point_already_executed", suoqu_text)
    self.assertIn("record_successful_execution_point", suoqu_text)
    self.assertIn("point_json.value(\"global_row\"", suoqu_text)

def test_live_visual_reprojects_points_into_global_checkerboard_before_execution(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    self.assertIn("classify_live_visual_point_into_checkerboard", suoqu_text)
    self.assertIn("未能归入全局棋盘格", suoqu_text)
    self.assertIn("source_mode", suoqu_text)
```

- [ ] **Step 4: Run the second red-test batch**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_slam_precomputed_skips_points_already_recorded_in_execution_memory src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_live_visual_reprojects_points_into_global_checkerboard_before_execution -v
```

Expected: FAIL because the new helper names and log strings do not exist yet.

- [ ] **Step 5: Commit the red tests**

```bash
git add src/chassis_ctrl/test/test_pointai_order.py
git commit -m "test: lock edge pair and execution memory behavior"
```

### Task 2: Implement Scan-Time Edge Pair Groups And Memory Reset

**Files:**
- Modify: `src/chassis_ctrl/src/suoquNode.cpp`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Add the new data structures and file paths**

Insert constants/structs near the existing pseudo-slam definitions:

```cpp
const std::string kBindExecutionMemoryJsonPath =
    "/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_execution_memory.json";

struct BindExecutionPointRecord
{
    int global_row = -1;
    int global_col = -1;
    int checkerboard_parity = -1;
    float world_x = 0.0f;
    float world_y = 0.0f;
    float world_z = 0.0f;
    std::string source_mode;
};

struct BindExecutionMemory
{
    std::string scan_session_id;
    Cabin_Point path_origin{};
    std::vector<BindExecutionPointRecord> executed_points;
};
```

- [ ] **Step 2: Implement the minimal JSON read/write/reset helpers**

Add helpers in `suoquNode.cpp`:

```cpp
BindExecutionMemory load_bind_execution_memory_json();
bool write_bind_execution_memory_json(const BindExecutionMemory& memory, std::string* error_message);
BindExecutionMemory reset_bind_execution_memory_for_scan_session(
    const std::string& scan_session_id,
    const Cabin_Point& path_origin
);
```

Minimal write logic should build:

```cpp
json memory_json = {
    {"scan_session_id", memory.scan_session_id},
    {"path_origin", {{"x", memory.path_origin.x}, {"y", memory.path_origin.y}, {"z", memory.path_origin.z}}},
    {"executed_points", json::array()},
};
```

- [ ] **Step 3: Relax checkerboard membership and add edge-pair selection**

Change checkerboard logic from:

```cpp
info.is_checkerboard_member = has_horizontal_neighbor && has_vertical_neighbor;
```

to a new explicit form:

```cpp
const bool can_form_matrix = has_horizontal_neighbor && has_vertical_neighbor;
const bool can_form_edge_pair = has_horizontal_neighbor || has_vertical_neighbor;
info.is_checkerboard_member = can_form_matrix || can_form_edge_pair;
```

Then extend group building with a second branch:

```cpp
if (selected_indices.size() == 4) {
    group.group_type = "matrix_2x2";
} else if (selected_indices.size() == 2) {
    group.group_type = "edge_pair";
} else {
    break;
}
```

- [ ] **Step 4: Reset execution memory only after scan outputs are written**

In the scan-complete section, keep this order:

```cpp
write_pseudo_slam_points_json(merged_world_points, merged_checkerboard_info_by_idx);
write_pseudo_slam_bind_path_json(bind_area_entries, checkerboard_info_by_idx, path_origin, cabin_height, cabin_speed);

const std::string scan_session_id = std::to_string(ros::Time::now().toNSec());
BindExecutionMemory memory = reset_bind_execution_memory_for_scan_session(scan_session_id, path_origin);
std::string memory_error;
write_bind_execution_memory_json(memory, &memory_error);
```

- [ ] **Step 5: Run the targeted tests and commit**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_pseudo_slam_bind_path_supports_edge_pair_groups src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_pseudo_slam_checkerboard_membership_no_longer_requires_both_axes src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_persists_bind_execution_memory_to_disk src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_rescan_rebuilds_execution_memory_after_scan_outputs_are_written -v
```

Expected: PASS

Commit:

```bash
git add src/chassis_ctrl/src/suoquNode.cpp src/chassis_ctrl/test/test_pointai_order.py
git commit -m "feat: add edge pair planning and scan-session memory reset"
```

### Task 3: Filter Precomputed Execution Through The Memory Ledger

**Files:**
- Modify: `src/chassis_ctrl/src/suoquNode.cpp`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Add the red test for point-level skip/record semantics**

Add:

```python
def test_precomputed_execution_filters_by_checkerboard_and_execution_memory(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    self.assertIn("filter_precomputed_group_points_for_execution", suoqu_text)
    self.assertIn("is_point_already_executed", suoqu_text)
    self.assertIn("record_successful_execution_point", suoqu_text)
    self.assertIn("group_json.value(\"group_type\"", suoqu_text)
```

- [ ] **Step 2: Run the new test to verify it fails**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_precomputed_execution_filters_by_checkerboard_and_execution_memory -v
```

Expected: FAIL because the new helper names do not exist yet.

- [ ] **Step 3: Implement memory-aware precomputed filtering**

Add helpers in `suoquNode.cpp`:

```cpp
bool is_point_already_executed(
    const BindExecutionMemory& memory,
    int global_row,
    int global_col
);

std::vector<json> filter_precomputed_group_points_for_execution(
    const json& group_json,
    const BindExecutionMemory& memory,
    bool only_checkerboard_parity_zero
);
```

Filter logic should resemble:

```cpp
if (only_checkerboard_parity_zero && point_json.value("checkerboard_parity", 0) != 0) {
    continue;
}
if (is_point_already_executed(memory, point_json.value("global_row", -1), point_json.value("global_col", -1))) {
    continue;
}
selected_points.push_back(point_json);
```

- [ ] **Step 4: Record only successful points after downstream success**

After a successful `/moduan/sg_precomputed(_fast)` call, update memory with:

```cpp
record_successful_execution_point(
    memory,
    point_json.value("global_row", -1),
    point_json.value("global_col", -1),
    point_json.value("checkerboard_parity", -1),
    point_json.value("world_x", 0.0f),
    point_json.value("world_y", 0.0f),
    point_json.value("world_z", 0.0f),
    "slam_precomputed"
);
```

Then flush the file:

```cpp
std::string memory_error;
write_bind_execution_memory_json(memory, &memory_error);
```

- [ ] **Step 5: Run tests and commit**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_precomputed_execution_filters_by_checkerboard_and_execution_memory src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_slam_precomputed_skips_points_already_recorded_in_execution_memory -v
```

Expected: PASS

Commit:

```bash
git add src/chassis_ctrl/src/suoquNode.cpp src/chassis_ctrl/test/test_pointai_order.py
git commit -m "feat: enforce execution memory in precomputed bind flow"
```

### Task 4: Route Live Visual Through The Same Global Grid And Memory

**Files:**
- Modify: `src/chassis_ctrl/src/suoquNode.cpp`
- Modify: `src/chassis_ctrl/src/moduanNode.cpp`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Add the live-visual red tests**

Add:

```python
def test_live_visual_uses_scan_grid_to_classify_points_before_execution(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    self.assertIn("classify_live_visual_point_into_checkerboard", suoqu_text)
    self.assertIn("load_bind_execution_memory_json", suoqu_text)

def test_live_visual_skips_points_that_cannot_be_projected_into_global_grid(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    self.assertIn("未能归入全局棋盘格", suoqu_text)
    self.assertIn("continue;", suoqu_text)
```

- [ ] **Step 2: Run the live-visual tests to verify they fail**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_live_visual_uses_scan_grid_to_classify_points_before_execution src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_live_visual_skips_points_that_cannot_be_projected_into_global_grid -v
```

Expected: FAIL because there is no live-visual grid classifier yet.

- [ ] **Step 3: Implement grid classification for newly observed points**

Add a helper that projects new world points into the stored grid:

```cpp
bool classify_live_visual_point_into_checkerboard(
    const fast_image_solve::PointCoords& world_point,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx,
    PseudoSlamCheckerboardInfo* classified_info
);
```

Minimal logic:

```cpp
classified_info->global_row = nearest_row_index;
classified_info->global_col = nearest_col_index;
classified_info->checkerboard_parity =
    (classified_info->global_row + classified_info->global_col + phase_reference) % 2;
```

If no suitable row/col exists, emit:

```cpp
printf("Cabin_Warn: live_visual点idx=%d未能归入全局棋盘格，跳过该点。\n", world_point.idx);
```

- [ ] **Step 4: Apply the same memory filter and success recording in live_visual**

In the `live_visual` path, only pass points that satisfy:

```cpp
if (classified_info.checkerboard_parity != 0) {
    continue;
}
if (is_point_already_executed(memory, classified_info.global_row, classified_info.global_col)) {
    continue;
}
```

After the downstream success returns, record:

```cpp
record_successful_execution_point(
    memory,
    classified_info.global_row,
    classified_info.global_col,
    classified_info.checkerboard_parity,
    world_point.World_coord[0],
    world_point.World_coord[1],
    world_point.World_coord[2],
    "live_visual"
);
```

- [ ] **Step 5: Run tests and commit**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_live_visual_uses_scan_grid_to_classify_points_before_execution src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_live_visual_skips_points_that_cannot_be_projected_into_global_grid src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_live_visual_reprojects_points_into_global_checkerboard_before_execution -v
```

Expected: PASS

Commit:

```bash
git add src/chassis_ctrl/src/suoquNode.cpp src/chassis_ctrl/src/moduanNode.cpp src/chassis_ctrl/test/test_pointai_order.py
git commit -m "feat: reuse checkerboard ledger in live visual execution"
```

### Task 5: Update Operator Docs And Run The Full Verification Set

**Files:**
- Modify: `SLAM_V11_QUICKSTART.md`
- Test: `src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **Step 1: Update the quickstart doc for the new operator-visible behavior**

Add sections like:

```md
### 边界两点组

- 现在边界区域允许生成 `edge_pair`
- `edge_pair` 也参与全局棋盘格跳绑

### 已执行记忆

- 文件：`src/chassis_ctrl/data/bind_execution_memory.json`
- 节点或机器重启后仍会继续跳过已绑点
- 重新扫描成功后会整体清空重建
```

- [ ] **Step 2: Run the full Python unit suite**

Run:

```bash
python3 -m unittest src.chassis_ctrl.test.test_gripper_tf_broadcaster src.chassis_ctrl.test.test_pointai_order
```

Expected: `Ran ... tests` and `OK`

- [ ] **Step 3: Run the package build**

Run:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1
```

Expected: build completes with `Built target suoquNode` and `Built target moduanNode`

- [ ] **Step 4: Review the working tree**

Run:

```bash
git status --short
```

Expected: only the intended source, test, and doc files are modified; no `.debug_frames/` or runtime JSON noise is staged accidentally.

- [ ] **Step 5: Commit the final doc/test polish**

```bash
git add SLAM_V11_QUICKSTART.md src/chassis_ctrl/test/test_pointai_order.py src/chassis_ctrl/src/suoquNode.cpp src/chassis_ctrl/src/moduanNode.cpp
git commit -m "docs: describe edge pair memory workflow"
```
