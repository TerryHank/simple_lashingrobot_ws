# 绑扎机器人 ROS 工程分层重构设计

**日期：** 2026-04-23  
**范围：** `src/robot_interface_hub`、`src/robot_hw_drivers`、`src/chassis_ctrl`、`src/robot_algorithm_layer`

## 1. 背景

当前工程已经完成了一轮拆包，但总体结构仍然偏“中间态”：

- `robot_interface_hub` 同时承载 `msg/srv/action`、`launch`、Web 前端和 API bridge
- `robot_hw_drivers` 同时承载相机驱动和索驱 TCP / 线模 Modbus 驱动
- `chassis_ctrl` 同时承载流程编排、模组控制、视觉节点、URDF、RViz、配置和数据账本
- `robot_algorithm_layer` 已经独立，但依赖的接口包名称仍然是中间态的 `robot_interface_hub`

这与更规范的 ROS 1 机器人工程组织方式不一致。当前最突出的问题不是“功能不能跑”，而是：

- 包职责边界仍然过宽，后续继续扩展会越来越难维护
- 前端、接口、硬件、感知、控制、流程、描述还没有形成稳定边界
- 运行时路径和包名仍残留大量历史命名，增加现场排障成本

## 2. 目标

本轮不机械照搬模板，而是在当前已经存在的 4 个包基础上，收口成更符合机器人工程实践的结构：

```text
src/
├── tie_robot_msgs
├── tie_robot_description
├── tie_robot_hw
├── tie_robot_perception
├── tie_robot_control
├── tie_robot_process
├── tie_robot_bringup
├── tie_robot_web
└── robot_algorithm_layer
```

其中：

- `tie_robot_msgs`：全局 `msg/srv/action`
- `tie_robot_description`：URDF、RViz 等模型/可视化资源
- `tie_robot_hw`：索驱 TCP、线模 Modbus 等底层驱动
- `tie_robot_perception`：相机驱动、视觉节点、视觉配置
- `tie_robot_control`：末端执行控制
- `tie_robot_process`：扫描、规划、执行流程编排和账本
- `tie_robot_bringup`：launch 装配入口
- `tie_robot_web`：前端页面、浏览器脚本、topic/service/action bridge
- `robot_algorithm_layer`：共享算法层，暂不改名，先保持稳定

## 3. 设计原则

### 3.1 按职责拆包，而不是按历史文件来源拆包

拆包后的判断标准是“这个包对外提供什么能力”，而不是“这个文件原来在哪个目录”。

### 3.2 优先切主动作链，非动作链代码不强行迁移

本轮优先保证以下主链完整迁移：

- 相机启动
- `pointAI`
- `suoquNode`
- `moduanNode`
- Web 前端与 `topics_transfer`
- 主 launch

不在主动作链上的调试节点、展示节点、临时脚本，不作为本轮必须项。

### 3.3 允许保留 `robot_algorithm_layer`

`robot_algorithm_layer` 已经是比较干净的共享算法包。本轮不为了名字统一而再引入一次大范围改名；先把外围职责边界切稳。

### 3.4 旧包退役采用“停用而不是立刻清空目录”

当前工作区有大量未提交改动和现场数据。为了避免误删现场资产，本轮对旧包采用：

- 新包接管构建与启动入口
- 旧包添加 `CATKIN_IGNORE`
- 旧目录保留为迁移参考，不再参与 catkin

## 4. 包边界

### 4.1 `tie_robot_msgs`

职责：

- 承载全局 `msg`
- 承载全局 `srv`
- 承载全局 `action`

包含内容：

- 从 `robot_interface_hub` 迁出全部 `msg/`
- 从 `robot_interface_hub` 迁出全部 `srv/`
- 从 `robot_interface_hub` 迁出全部 `action/`

不包含：

- `launch`
- `topics_transfer`
- Web 页面
- 任何业务 `common.hpp`

### 4.2 `tie_robot_web`

职责：

- 承载新前端
- 承载 Web 辅助脚本
- 承载 `topics_transfer` API bridge

包含内容：

- `web/`
- `scripts/workspace_picker_web_server.py`
- `scripts/workspace_picker_web_open.py`
- `scripts/system_log_mux.py`
- `src/topics_transfer.cpp`

依赖：

- `tie_robot_msgs`
- `roscpp`
- `actionlib`
- `rosbridge_server`

### 4.3 `tie_robot_bringup`

职责：

- 只做 launch 装配
- 聚合 Web、相机、感知、控制、流程节点

包含内容：

- 从 `robot_interface_hub/launch` 迁出的主 launch

关键变化：

- `run.launch` 中节点包名统一切到 `tie_robot_process`、`tie_robot_control`、`tie_robot_perception`、`tie_robot_web`
- `api.launch` 相机入口切到 `tie_robot_perception`

### 4.4 `tie_robot_description`

职责：

- 承载结构模型与可视化资源

包含内容：

- `URDF/model.urdf`
- `rviz/chassis_visual.rviz`
- `rviz/sijuyanjiuyuan.rviz`

本轮不扩展 meshes 和 xacro，只先把现有资源归位。

### 4.5 `tie_robot_hw`

职责：

- 承载索驱 TCP 驱动
- 承载线模 Modbus 驱动
- 只暴露驱动级库，不承载相机驱动

包含内容：

- `include/.../driver/*`
- `src/driver/*`

导出库名：

- `tie_robot_hw_driver_core`

### 4.6 `tie_robot_perception`

职责：

- 承载相机驱动
- 承载视觉节点
- 承载感知侧 TF broadcaster 和视觉配置

包含内容：

- 从 `robot_hw_drivers` 迁入相机 `src/camera/`、`cfg/`、`dependencies/`、`srv/ConvertDepthToPointCloud.srv`、相机 launch
- 从 `chassis_ctrl/scripts` 迁入 `pointAI.py`
- 从 `chassis_ctrl/scripts` 迁入 `gripper_tf_broadcaster.py`
- 从 `chassis_ctrl/scripts` 迁入 `stable_point_tf_broadcaster.py`
- 从 `chassis_ctrl/config` 迁入 `gripper_tf.yaml`
- 从 `chassis_ctrl/config` 迁入 `bind_point_classification.yaml`

### 4.7 `tie_robot_control`

职责：

- 承载末端执行控制节点

包含内容：

- `moduanNode.cpp`

本轮刻意不迁移 `moduanNode_show.cpp`，因为它不在主动作链。

### 4.8 `tie_robot_process`

职责：

- 承载扫描、规划、执行流程编排
- 持有账本数据

包含内容：

- `suoquNode.cpp`
- `data/` 下现有 JSON 和文本账本

关键变化：

- 所有硬编码的 `src/chassis_ctrl/data/...` 路径切到 `src/tie_robot_process/data/...`
- `topics_transfer` 读取 fatal error 和 path_points 的路径同步切换

## 5. 与模板建议的差异

用户给出的参考结构里有 `tie_robot_process`、`tie_robot_control`、`tie_robot_perception`、`tie_robot_bringup` 等标准边界，本轮总体遵循；但结合当前工程现状，有两处刻意偏离：

### 5.1 保留 `robot_algorithm_layer`

原因：

- 已完成独立
- 已被 `pointAI` 和 `suoquNode` 接入
- 再次改名收益小、波及面大

### 5.2 额外保留 `tie_robot_web`

原因：

- 当前工程存在独立 Web 前端和浏览器脚本
- `topics_transfer` 既不是接口定义，也不是 bringup 装配
- 将 Web/API bridge 单独成包，边界更清晰

## 6. 迁移策略

### 6.1 本轮完成项

本轮必须完成：

- 新 8 个职责包落地
- 主动作链代码迁入新包
- 新包之间的构建依赖和头文件引用切换完成
- 启动入口切到 `tie_robot_bringup`
- 旧 3 个大包添加 `CATKIN_IGNORE`

### 6.2 本轮刻意不做的事

为控制风险，本轮不额外做这些事：

- 不继续重命名 `robot_algorithm_layer`
- 不清掉所有历史测试和调试脚本
- 不一次性清掉所有旧源码目录
- 不把 `_show` 类节点纳入新主链

## 7. 验证标准

满足以下条件视为本轮完成：

1. `src/` 下存在目标 8 个新包和 `robot_algorithm_layer`
2. `tie_robot_msgs` 负责全部 ROS 接口定义
3. `tie_robot_web` 不再承载 `msg/srv/action`
4. `tie_robot_bringup/run.launch` 和 `api.launch` 已切到新包名
5. `tie_robot_perception` 负责相机驱动与视觉节点
6. `tie_robot_hw` 只负责索驱 / 线模驱动库
7. `tie_robot_control` 和 `tie_robot_process` 分别承载 `moduanNode` / `suoquNode`
8. 旧 `robot_interface_hub`、`robot_hw_drivers`、`chassis_ctrl` 被 `CATKIN_IGNORE`
9. 定向结构测试通过
10. `catkin_make` 对新包集合构建通过
