# Moduan Action/State Architecture 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:executing-plans 或在当前会话中按 TDD 逐任务实现。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将线性模组/绑扎执行机构收口成标准 ROS 风格：驱动层发布状态，控制层提供长任务 Action，上游只调用抽象接口，不直接读 PLC 完成位。

**架构：** `moduan_driver_node` 周期读 PLC 并发布 `/moduan/state`；`moduanNode` 提供 `/moduan/execute_bind_points` Action，内部写点位、发 PLC 执行信号、等待 `FINISHALL`、清完成位；旧服务 `/moduan/sg`、`/moduan/sg_precomputed`、`/moduan/single_move` 保留为兼容 wrapper。

**技术栈：** ROS1 Noetic、catkin、actionlib、C++、`tie_robot_msgs` 自定义 msg/action。

---

## 文件结构

- 创建：`src/tie_robot_msgs/msg/ModuanState.msg`
  - 线性模组标准状态 topic，用于隐藏 PLC 寄存器细节。
- 创建：`src/tie_robot_msgs/action/ExecuteBindPointsTask.action`
  - 线性模组执行绑扎点位的长任务 Action。
- 修改：`src/tie_robot_msgs/CMakeLists.txt`
  - 注册新 msg/action。
- 修改：`src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
  - 发布 `/moduan/state`，增加 `/moduan/execute_bind_points` Action server，把旧服务改为调用统一执行函数。
- 修改：`src/tie_robot_control/src/moduan/linear_module_executor.cpp`
  - 将 `finish_all(150)` 改名为明确的 `wait_for_plc_finish_all(...)`，并在执行段发布 Action feedback。
- 修改：`src/tie_robot_control/include/tie_robot_control/moduan/linear_module_executor.hpp`
  - 暴露新的等待函数名和执行选项。
- 测试：`src/tie_robot_process/test/test_motion_chain_signal_guard.py`
  - 用文本结构测试锁定接口、命名和调用顺序。

## 任务 1：消息和 Action 接口

- [x] **步骤 1：编写失败测试**

在 `src/tie_robot_process/test/test_motion_chain_signal_guard.py` 增加测试，断言：

```python
def test_moduan_state_topic_and_action_interfaces_are_declared(self):
    cmake = (MSGS_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    state_msg = (MSGS_DIR / "msg" / "ModuanState.msg").read_text(encoding="utf-8")
    action = (MSGS_DIR / "action" / "ExecuteBindPointsTask.action").read_text(encoding="utf-8")
    self.assertIn("ModuanState.msg", cmake)
    self.assertIn("ExecuteBindPointsTask.action", cmake)
    self.assertIn("bool executing", state_msg)
    self.assertIn("bool finish_all", state_msg)
    self.assertIn("PointCoords[] points", action)
    self.assertIn("string phase", action)
    self.assertIn("bool success", action)
```

- [x] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard`

预期：FAIL，缺少 `ModuanState.msg` 或 `ExecuteBindPointsTask.action`。

- [x] **步骤 3：实现接口**

创建 `ModuanState.msg`：

```text
bool connected
bool ready
bool executing
bool finish_all
bool error
float64 x
float64 y
float64 z
float64 motor_angle
string phase
string last_error
```

创建 `ExecuteBindPointsTask.action`：

```text
PointCoords[] points
bool apply_jump_bind_filter
---
bool success
string message
string error_code
---
string phase
uint32 selected_count
float64 current_x
float64 current_y
float64 current_z
float64 elapsed_sec
```

- [x] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard`

预期：PASS。

## 任务 2：规范 PLC 完成等待命名

- [x] **步骤 1：编写失败测试**

在 `test_motion_chain_signal_guard.py` 中断言：

```python
executor = (CONTROL_DIR / "src" / "moduan" / "linear_module_executor.cpp").read_text(encoding="utf-8")
self.assertIn("wait_for_plc_finish_all", executor)
self.assertNotIn("finish_all(150)", executor)
self.assertIn("kFinishAllPollInterval", executor)
```

- [x] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard`

预期：FAIL，当前仍是 `finish_all(150)`。

- [x] **步骤 3：最小实现**

把 `finish_all(int inter_time)` 改名为：

```cpp
bool wait_for_plc_finish_all(
    std::chrono::milliseconds poll_interval,
    std::chrono::seconds timeout)
```

调用改为：

```cpp
if (!wait_for_plc_finish_all(kFinishAllPollInterval, kFinishAllTimeout)) {
    response_message = "等待FINISHALL标志超时，当前子区域绑扎未确认完成";
    return false;
}
```

- [x] **步骤 4：运行测试和 catkin 构建**

运行：

```bash
python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j2
```

预期：测试和构建均通过。

## 任务 3：发布 `/moduan/state`

- [x] **步骤 1：编写失败测试**

断言 `moduan_ros_callbacks.cpp`：

```python
self.assertIn('nh_.advertise<tie_robot_msgs::ModuanState>("/moduan/state"', callbacks)
self.assertIn("publish_moduan_state_topic", callbacks)
self.assertIn("FINISH_ALL_FLAG", callbacks)
self.assertIn("linear_module_data_upload_msg", callbacks)
```

- [x] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard`

预期：FAIL，尚未发布标准状态 topic。

- [x] **步骤 3：最小实现**

在 `read_module_motor_state()` 每轮读完 PLC 后，构造 `tie_robot_msgs::ModuanState` 并发布：

```cpp
state_msg.connected = plc != nullptr;
state_msg.ready = plc != nullptr && !error_detected.load();
state_msg.executing = moduan_plc_execution_state.load();
state_msg.finish_all = state->FINISH_ALL_FLAG != 0;
state_msg.error = state->ERROR_FLAG_X || state->ERROR_FLAG_Y || state->ERROR_FLAG_Z ||
                  state->ERROR_FLAG_LASHING || mot_state->ERROR_FLAG_MOTOR;
```

- [x] **步骤 4：运行测试和构建**

运行：

```bash
python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j2
```

预期：通过。

## 任务 4：增加 `/moduan/execute_bind_points` Action

- [x] **步骤 1：编写失败测试**

断言：

```python
self.assertIn("actionlib/server/simple_action_server.h", callbacks)
self.assertIn('"/moduan/execute_bind_points"', callbacks)
self.assertIn("ExecuteBindPointsTaskAction", callbacks)
self.assertIn("execute_bind_points_action_callback", callbacks)
```

- [x] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard`

预期：FAIL，Action server 尚未存在。

- [x] **步骤 3：最小实现**

在 `RunModuanNodeWithDefaultRole()` 创建 Action server；Action callback 拿到 goal 后加 `lashing_mutex`，调用 `execute_bind_points(goal->points, message, goal->apply_jump_bind_filter)`，并 `setSucceeded` 或 `setAborted`。

- [x] **步骤 4：运行测试和构建**

运行：

```bash
python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j2
```

预期：通过。

## 任务 5：旧服务兼容 wrapper

- [x] **步骤 1：编写失败测试**

断言旧服务仍存在，且服务内部只做上游流程，最终落到统一执行函数或 Action：

```python
self.assertIn('advertiseService("/moduan/sg"', callbacks)
self.assertIn('advertiseService("/moduan/sg_precomputed"', callbacks)
self.assertIn("execute_bind_points(", callbacks)
self.assertLess(single_bind_body.index("AI_client.call(srv)"), single_bind_body.index("execute_bind_points("))
```

- [x] **步骤 2：运行测试验证失败或通过**

运行：`python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard`

预期：当前兼容服务已基本满足；若失败，按统一执行入口修正。

- [x] **步骤 3：最终验证**

运行：

```bash
python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_scan_artifact_write_guard src.tie_robot_bringup.test.test_ros_interface_names
python3 scripts/check_ros_interface_names.py
git diff --check -- src/tie_robot_msgs src/tie_robot_control src/tie_robot_process/test/test_motion_chain_signal_guard.py
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j2
```

预期：全部通过。

## 任务 6：流程层只调用 Action

- [x] **步骤 1：编写失败测试**

在 `test_motion_chain_signal_guard.py` 中约束 `suoquNode.cpp` 预计算绑扎点执行链：

```python
self.assertIn("actionlib/client/simple_action_client.h", process_node)
self.assertIn("ExecuteBindPointsTaskAction", process_node)
self.assertIn("execute_moduan_bind_points_via_action", process_node)
self.assertIn('"/moduan/execute_bind_points"', process_node)
self.assertNotIn("sg_precomputed_client.call(", process_node)
self.assertNotIn("sg_precomputed_fast_client.call(", process_node)
```

- [x] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard`

预期：FAIL，流程层仍调用 `/moduan/sg_precomputed*`。

- [x] **步骤 3：最小实现**

`tie_robot_process` 增加 `actionlib` 依赖；`suoquNode.cpp` 初始化 `/moduan/execute_bind_points` Action client，并用 `execute_moduan_bind_points_via_action(...)` 替换四处预计算绑扎点服务调用。

- [x] **步骤 4：最终验证**

运行：

```bash
python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard
python3 -m unittest src.tie_robot_bringup.test.test_ros_interface_names src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_scan_artifact_write_guard src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_single_point_bind_button_calls_atomic_backend_service
python3 scripts/check_ros_interface_names.py
git diff --check -- docs/superpowers/plans/2026-04-29-moduan-action-state-architecture.md docs/architecture/ros_interface_migration_map.yaml src/tie_robot_msgs src/tie_robot_control src/tie_robot_process
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j2
```

预期：全部通过。
