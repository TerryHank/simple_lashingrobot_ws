# Dynamic Scan Bind Planning 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 保留当前“固定扫描位 -> S2 识别全局点 -> 执行层继续视觉精校”的链路，只把扫描完成后的区域规划来源从 `path_points.json` 切换为基于全局点和 TCP 行程范围的动态规划。

**架构：** 在 `suoquNode.cpp` 中新增动态区域规划器，用全局点反推出每个区域的索驱 `x/y/z` 和 4 点模板组，然后继续复用现有的 `run_bind_from_scan` 和 `run_live_visual_global_work`。同时统一规划层与执行层的局部范围到 `x/y 0..300, z 0..140`。

**技术栈：** ROS Noetic, roscpp, rospy, tf2_ros, nlohmann::json, Python unittest, catkin

---

### 任务 1：为动态区域规划补失败测试

**文件：**
- 修改：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/test/test_pointai_order.py`
- 修改：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp`
- 修改：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/moduanNode_show.cpp`

- [ ] **步骤 1：编写失败的测试**

在 `test_pointai_order.py` 新增断言，覆盖：
- `run_pseudo_slam_scan()` 不再直接按 `for (const auto& cabin_point : con_path)` 生成最终 `bind_area_entries`
- `pseudo_slam_bind_path.json` 的区域位姿来自新的动态规划 helper
- 动态规划 helper 明确使用 `gripper_frame` 下的局部工作范围常量
- 局部范围统一为 `x/y 0..300, z 0..140`
- 最后一组允许补齐成 `4` 点模板

- [ ] **步骤 2：运行测试验证失败**

运行：
```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_dynamic_scan_bind_planning_replaces_static_path_area_generation src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_dynamic_scan_bind_planning_uses_tcp_travel_range_300_300_140 src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_dynamic_scan_bind_planning_pads_last_group_to_four_template_points
```

预期：FAIL，因为动态区域规划器和统一范围还不存在。

- [ ] **步骤 3：不改实现，只确认失败原因正确**

预期失败应该是：
- 在 `suoquNode.cpp` 中找不到新的动态规划函数名
- 或者仍能找到基于静态 `con_path` 直接构造 `bind_area_entries` 的旧逻辑

- [ ] **步骤 4：Commit 测试检查点（可选）**

```bash
git add /home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/test/test_pointai_order.py
git commit -m "test: add dynamic scan bind planning regression coverage"
```

### 任务 2：新增动态区域规划器并替换扫描后区域生成

**文件：**
- 修改：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp`
- 测试：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：实现动态规划所需的数据结构和范围常量**

在 `suoquNode.cpp` 中新增或替换成如下语义：

```cpp
constexpr float kDynamicBindTravelMaxXMm = 300.0f;
constexpr float kDynamicBindTravelMaxYMm = 300.0f;
constexpr float kDynamicBindTravelMaxZMm = 140.0f;

struct DynamicBindPlanningCandidatePose
{
    float cabin_x = 0.0f;
    float cabin_y = 0.0f;
    float cabin_z = 0.0f;
};
```

- [ ] **步骤 2：运行最小编译前检查**

运行：
```bash
python3 -m py_compile /home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/test/test_pointai_order.py
```

预期：PASS

- [ ] **步骤 3：实现“世界点 -> 候选索驱位姿”反解 helper**

在 `suoquNode.cpp` 里新增 helper，语义是：

```cpp
DynamicBindPlanningCandidatePose build_dynamic_bind_candidate_pose_from_world_point(
    const fast_image_solve::PointCoords& world_point)
```

实现要点：
- 目标局部模板中心固定取 `(150, 150, 70)`
- 由世界点反推出索驱 `x/y/z`
- 返回值用于后续覆盖集评估

- [ ] **步骤 4：实现“位姿下可覆盖点”筛选 helper**

在 `suoquNode.cpp` 里新增 helper，语义是：

```cpp
std::vector<fast_image_solve::PointCoords> collect_world_points_coverable_by_dynamic_pose(
    const std::vector<fast_image_solve::PointCoords>& world_points,
    const DynamicBindPlanningCandidatePose& pose)
```

要求：
- 每个点先转换到 `gripper_frame`
- 必须同时满足 `x/y/z` 在 `300/300/140` 范围内

- [ ] **步骤 5：实现“4 点模板组装” helper**

新增 helper，语义是：

```cpp
PseudoSlamBindGroup build_dynamic_four_point_template_group(
    const std::vector<fast_image_solve::PointCoords>& coverable_points,
    const std::unordered_set<int>& unfinished_global_indices)
```

要求：
- 优先使用未完成点
- 未完成点不足 `4` 个时允许补齐
- 输出始终为 `4` 个模板点

- [ ] **步骤 6：实现动态区域规划主函数**

新增主函数：

```cpp
std::vector<PseudoSlamGroupedAreaEntry> build_dynamic_bind_area_entries_from_scan_world(
    const std::vector<fast_image_solve::PointCoords>& planning_world_points,
    float cabin_height)
```

要求：
- 循环直到所有未规划点都被覆盖
- 每次产出一个动态区域
- 每个区域只包含一个 `4` 点模板组
- `cabin_pose` 来自候选索驱位姿

- [ ] **步骤 7：在 `run_pseudo_slam_scan()` 中替换旧区域生成**

把这段：

```cpp
for (const auto& cabin_point : con_path) {
    ...
    bind_area_entries.push_back(...)
}
```

替换为：

```cpp
std::vector<PseudoSlamGroupedAreaEntry> bind_area_entries =
    build_dynamic_bind_area_entries_from_scan_world(planning_world_points, cabin_height);
```

- [ ] **步骤 8：运行聚焦测试验证通过**

运行：
```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_dynamic_scan_bind_planning_replaces_static_path_area_generation src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_dynamic_scan_bind_planning_pads_last_group_to_four_template_points
```

预期：PASS

### 任务 3：把执行入口切到动态区域位姿

**文件：**
- 修改：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp`
- 测试：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

增加断言覆盖：
- `run_bind_from_scan()` 直接使用 `bind_path_json["areas"][i]["cabin_pose"]`
- `run_live_visual_global_work()` 不再依赖 `con_path[area_index]` 去定义区域位姿
- 两条入口都继续调用现有 `prepare_precomputed_bind_group_for_execution()`

- [ ] **步骤 2：运行测试验证失败**

运行：
```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_dynamic_scan_bind_execution_uses_planned_dynamic_area_pose src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_live_visual_execution_uses_dynamic_area_pose_but_keeps_refine_strategy
```

预期：FAIL

- [ ] **步骤 3：修改执行入口实现**

在 `suoquNode.cpp` 中：
- 保留 `run_bind_from_scan()` 的执行风格
- 保留 `run_live_visual_global_work()` 的视觉精校风格
- 只把区域位姿来源切成动态 `bind_path_json["areas"]`

- [ ] **步骤 4：运行测试验证通过**

运行：
```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_dynamic_scan_bind_execution_uses_planned_dynamic_area_pose src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_live_visual_execution_uses_dynamic_area_pose_but_keeps_refine_strategy
```

预期：PASS

### 任务 4：统一规划层与执行层的虎口范围

**文件：**
- 修改：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp`
- 修改：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/moduanNode_show.cpp`
- 测试：`/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

新增断言：
- `is_local_bind_point_in_range()` 使用 `300/300/140`
- `moduanNode_show.cpp` 执行前过滤也使用 `300/300/140`

- [ ] **步骤 2：运行测试验证失败**

运行：
```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_planning_and_execution_bind_ranges_are_unified_to_300_300_140
```

预期：FAIL

- [ ] **步骤 3：实现统一常量**

在 `suoquNode.cpp`：

```cpp
constexpr float kTravelMaxXMm = 300.0f;
constexpr float kTravelMaxYMm = 300.0f;
constexpr float kTravelMaxZMm = 140.0f;
```

在 `is_local_bind_point_in_range()` 中增加 `z` 判定。  
在 `moduanNode_show.cpp` 里把硬编码过滤改成同样的 `300/300/140`。

- [ ] **步骤 4：运行测试验证通过**

运行：
```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_planning_and_execution_bind_ranges_are_unified_to_300_300_140
```

预期：PASS

### 任务 5：完成集成验证

**文件：**
- 修改：所有实际变更文件

- [ ] **步骤 1：运行 Python 语法检查**

运行：
```bash
python3 -m py_compile /home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/scripts/pointAI.py /home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/test/test_pointai_order.py
```

预期：PASS

- [ ] **步骤 2：运行回归测试**

运行：
```bash
python3 -m unittest src.chassis_ctrl.test.test_pointai_order
```

预期：PASS

- [ ] **步骤 3：运行编译验证**

运行：
```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1
```

预期：PASS

- [ ] **步骤 4：检查新旧入口仍在线**

运行：
```bash
roslaunch --nodes chassis_ctrl run.launch
```

预期：能看到现有 `suoquNode / pointAINode / topictransNode` 等节点入口，没有因为动态规划改动丢失执行链。
