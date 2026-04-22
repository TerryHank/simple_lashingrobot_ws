# 全工程接口语义重构设计

**日期：** 2026-04-23  
**范围：** `robot_interface_hub`、`chassis_ctrl`、新前端 `robot_interface_hub/web`

## 1. 背景

当前工程已经完成了接口包、硬件驱动包、算法层包的拆分，但上层 ROS 通信语义仍然混杂：

- 多个耗时任务仍由前端先发 `Topic`，再由 `topics_transfer` 同步转 `Service`
- 某些短平快状态切换仍通过 `Topic` 实现，不利于显式确认结果
- 新前端的主要任务入口缺少任务态反馈通道，调用方只能靠日志和结果图猜当前阶段

这不符合 ROS 常见分工：

- `Topic`：持续数据流、状态广播、图像和进度广播
- `Service`：短平快请求，要求立即有明确返回
- `Action`：耗时任务，需要任务反馈，调用方不应被同步阻塞

## 2. 目标

将接口语义按实际任务特征收口，而不是机械把所有东西都改成 `Action`。

本轮目标只覆盖新前端主链和当前动作链上的关键入口：

- 新前端长任务入口改为 `Action`
- 短请求保持或改成 `Service`
- 持续状态、图像、结果覆盖层保持 `Topic`
- `chassis_ctrl` 内部既有执行链优先保持稳定，不在这一轮强行全改成 `Action`

## 3. 分层边界

### 3.1 `robot_interface_hub`

职责：

- 定义对外 `msg/srv/action`
- 承载新前端
- 承载前端 API bridge（`topics_transfer`）
- 面向前端提供更规范的 `Action` / `Service` 入口

不负责：

- 具体硬件通信
- 动态规划、视觉计算
- 绑扎执行编排细节

### 3.2 `chassis_ctrl`

职责：

- 继续承载应用编排和执行状态机
- 对外保留内部稳定服务接口
- 在必要处补短请求 `Service`

不负责：

- 前端话题协议适配
- Web 侧 Action 客户端逻辑

## 4. 接口语义决策

### 4.1 保持为 Topic

以下接口是持续数据流或广播状态，继续保留 `Topic`：

- `/Scepter/ir/image_raw`
- `/pointAI/manual_workspace_quad_pixels`
- `/pointAI/manual_workspace_s2_result_raw`
- `/pointAI/result_image_raw`
- `/cabin/area_progress`
- `/cabin/pseudo_slam_markers`
- 设备状态与报警广播类话题

### 4.2 保持或改为 Service

以下接口属于短平快请求，应使用 `Service`：

- `/pointAI/process_image`
- `/cabin/plan_path`
- `/cabin/set_execution_mode`（新增，替代新前端对 `/web/cabin/set_execution_mode` 的直接 `Topic` 写入）
- `/cabin/single_move`
- `/moduan/single_move`
- `/moduan/sg_precomputed`
- `/moduan/sg_precomputed_fast`

本轮只强制新增 `/cabin/set_execution_mode`，其余接口继续沿用现有 `Service`。

### 4.3 改为 Action

以下接口属于前端直连的耗时任务，应改为 `Action`：

1. 固定/多位姿/单中心扫描建图  
   新接口：`/web/cabin/start_pseudo_slam_scan`

2. 开始执行层  
   新接口：`/web/cabin/start_global_work`

3. 直接执行账本测试  
   新接口：`/web/cabin/run_bind_path_direct_test`

这些 `Action` 由 `robot_interface_hub/topics_transfer` 提供 `ActionServer`，内部继续桥接到 `chassis_ctrl` 现有服务：

- `/cabin/start_pseudo_slam_scan_with_options`
- `/cabin/start_work_with_options`
- `/cabin/run_bind_path_direct_test`

## 5. 新增接口定义

### 5.1 `SetExecutionMode.srv`

用途：短请求切换执行模式。

请求：

- `uint8 execution_mode`

响应：

- `bool success`
- `string message`

模式枚举：

- `0 = slam_precomputed`
- `1 = live_visual`

### 5.2 `StartPseudoSlamScanTask.action`

Goal：

- `bool enable_capture_gate`
- `uint8 scan_strategy`

Result：

- `bool success`
- `string message`

Feedback：

- `string stage`
- `string detail`
- `float32 progress`

### 5.3 `StartGlobalWorkTask.action`

Goal：

- `bool clear_execution_memory`
- `uint8 execution_mode`

Result：

- `bool success`
- `string message`

Feedback：

- `string stage`
- `string detail`
- `float32 progress`

### 5.4 `RunBindPathDirectTestTask.action`

Goal：

- 空 goal

Result：

- `bool success`
- `string message`

Feedback：

- `string stage`
- `string detail`
- `float32 progress`

## 6. ActionServer 设计

`topics_transfer` 新增 3 个 `SimpleActionServer`：

- `start_pseudo_slam_scan_action_server`
- `start_global_work_action_server`
- `run_bind_path_direct_test_action_server`

执行策略：

1. 收到 goal 后立即回一条 `accepted/dispatching` feedback
2. 对需要短请求准备动作的任务，先同步调用所需 `Service`
3. 再调用现有 `chassis_ctrl` 长任务 `Service`
4. 根据返回结果设置 `Succeeded` 或 `Aborted`

取消策略：

- 本轮是“接口层动作化”，不是“底层执行链全动作化”
- 因内部仍桥接阻塞式 `Service`，所以当前 `cancel` 为软取消：
  - 若服务尚未真正发出，则允许 `Preempted`
  - 若底层服务已发出，则记录日志并在当前轮结果返回后结束

这符合本轮“先规范上层接口，不一次震碎内部执行链”的目标。

## 7. 前端改造

新前端只调整和当前页面直接相关的 3 个长任务按钮：

- “移动到固定扫描位姿并扫描规划” -> `ActionClient`
- “开始执行层 / 保留记忆直接开始执行层” -> `ActionClient`
- “直接执行账本测试” -> `ActionClient`

同时：

- 不再通过 `Topic` 发 `/web/cabin/start_pseudo_slam_scan`
- 不再通过 `Topic` 发 `/web/cabin/start_global_work`
- 不再通过 `Topic` 发 `/web/cabin/run_bind_path_direct_test`
- 执行模式切换改为 `Service` 调 `/cabin/set_execution_mode`

图像订阅和结果覆盖层逻辑不变。

## 8. 不在本轮范围

为控制风险，以下内容不在本轮硬改：

- `pointAI` 的 `/web/pointAI/run_workspace_s2` / `/web/pointAI/set_workspace_quad`
- `moduan` 老调试话题的全面清理
- `suoquNode` / `moduanNode` 内部所有耗时服务改为 Action
- 底层驱动层的 cancel 传播

这些会在后续阶段继续清。

## 9. 验证标准

满足以下条件即视为本轮完成：

1. 新接口定义已落在 `robot_interface_hub`
2. `topics_transfer` 已提供 3 个可工作的 `ActionServer`
3. `suoquNode` 已提供 `/cabin/set_execution_mode` 服务
4. 新前端按钮已改为 `ActionClient` / `Service` 调用
5. 旧的这 3 个长任务前端 topic 发布逻辑已删除
6. `catkin_make` 通过
7. 接口语义定向测试通过
