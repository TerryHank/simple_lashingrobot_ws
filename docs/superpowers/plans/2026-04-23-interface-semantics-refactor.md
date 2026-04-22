# 接口语义重构实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将新前端主链上的耗时任务改为 `Action`，将执行模式切换改为 `Service`，保留持续状态与图像流为 `Topic`。

**架构：** 在 `robot_interface_hub` 中新增任务型 `Action` 和短请求 `Service`，由 `topics_transfer` 充当 `ActionServer`/服务桥；`chassis_ctrl` 继续保留内部稳定服务接口，新前端改为使用 `ActionClient` 和 `Service`。

**技术栈：** ROS Noetic、`actionlib`、`roscpp`、`roslibjs`、catkin

---

### 任务 1：补接口语义结构测试

**文件：**
- 修改：`src/chassis_ctrl/test/test_pointai_order.py`
- 修改：`src/chassis_ctrl/test/test_workspace_picker_web_launch.py`

- [ ] **步骤 1：编写会先失败的结构测试**

补以下断言：

- `robot_interface_hub/CMakeLists.txt` 生成 `SetExecutionMode.srv` 和 3 个新 action
- `topics_transfer.cpp` 包含 `SimpleActionServer` 和 3 个 action server 名称
- `suoquNode.cpp` 提供 `/cabin/set_execution_mode`
- 新前端存在 `ROSLIB.ActionClient` 和 `ROSLIB.Service`
- 新前端不再发布 `/web/cabin/start_pseudo_slam_scan`、`/web/cabin/start_global_work`、`/web/cabin/run_bind_path_direct_test`

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_interface_hub_task_interfaces_use_actions_and_short_requests_use_services \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_topics_transfer_hosts_action_servers_for_long_running_web_tasks \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_exposes_set_execution_mode_service \
  src.chassis_ctrl.test.test_workspace_picker_web_launch.WorkspacePickerWebLaunchTest.test_workspace_picker_uses_action_clients_for_long_running_tasks
```

预期：FAIL，提示缺少新 action / service / 前端 ActionClient。

### 任务 2：新增接口定义

**文件：**
- 修改：`src/robot_interface_hub/CMakeLists.txt`
- 修改：`src/robot_interface_hub/package.xml`
- 创建：`src/robot_interface_hub/srv/SetExecutionMode.srv`
- 创建：`src/robot_interface_hub/action/StartPseudoSlamScanTask.action`
- 创建：`src/robot_interface_hub/action/StartGlobalWorkTask.action`
- 创建：`src/robot_interface_hub/action/RunBindPathDirectTestTask.action`

- [ ] **步骤 1：新增接口定义并接入消息生成**

- [ ] **步骤 2：为 actionlib 补构建依赖**

需要在 `find_package(catkin REQUIRED COMPONENTS ...)` 与 `package.xml` 中补齐 `actionlib`。

### 任务 3：后端补短请求 Service

**文件：**
- 修改：`src/chassis_ctrl/include/common.hpp`
- 修改：`src/chassis_ctrl/src/suoquNode.cpp`

- [ ] **步骤 1：新增 `/cabin/set_execution_mode` 服务定义和处理函数**

处理规则：

- `0 -> slam_precomputed`
- `1 -> live_visual`
- 返回明确的 `success/message`

- [ ] **步骤 2：用服务替代新链路中的 mode 切换入口**

保留内部 `set_global_execution_mode(...)`，只调整对外接口语义。

### 任务 4：实现 `topics_transfer` 动作桥

**文件：**
- 修改：`src/robot_interface_hub/src/topics_transfer.cpp`

- [ ] **步骤 1：引入 3 个 `SimpleActionServer`**

- [ ] **步骤 2：扫描建图 Action 桥接 `/cabin/start_pseudo_slam_scan_with_options`**

- [ ] **步骤 3：开始执行层 Action 桥接 `/cabin/set_execution_mode` + `/cabin/start_work_with_options`**

- [ ] **步骤 4：账本直执行 Action 桥接 `/cabin/run_bind_path_direct_test`**

- [ ] **步骤 5：删除这 3 个长任务的旧 topic 桥**

应移除：

- `/web/cabin/start_pseudo_slam_scan`
- `/web/cabin/start_global_work`
- `/web/cabin/run_bind_path_direct_test`

### 任务 5：前端改为 ActionClient / Service

**文件：**
- 修改：`src/robot_interface_hub/web/ir_workspace_picker.mjs`
- 修改：`src/robot_interface_hub/web/index.html`

- [ ] **步骤 1：扫描按钮改为 ActionClient**

- [ ] **步骤 2：开始执行层按钮改为 ActionClient，并通过 Service 切模式**

- [ ] **步骤 3：账本测试按钮改为 ActionClient**

- [ ] **步骤 4：调整按钮可用性判断和状态文案**

### 任务 6：跑定向验证

**文件：**
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`
- 测试：`src/chassis_ctrl/test/test_workspace_picker_web_launch.py`

- [ ] **步骤 1：运行结构测试**

```bash
source /opt/ros/noetic/setup.bash && source /home/hyq-/simple_lashingrobot_ws/devel/setup.bash && \
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_interface_hub_task_interfaces_use_actions_and_short_requests_use_services \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_topics_transfer_hosts_action_servers_for_long_running_web_tasks \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_exposes_set_execution_mode_service \
  src.chassis_ctrl.test.test_workspace_picker_web_launch.WorkspacePickerWebLaunchTest.test_workspace_picker_uses_action_clients_for_long_running_tasks
```

预期：PASS

- [ ] **步骤 2：运行构建**

```bash
source /opt/ros/noetic/setup.bash && \
catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_algorithm_layer;robot_hw_drivers;robot_interface_hub;chassis_ctrl" -j2
```

预期：`exit 0`

- [ ] **步骤 3：运行前端脚本基础语法检查**

```bash
node --check src/robot_interface_hub/web/ir_workspace_picker.mjs
```

预期：无语法错误输出
