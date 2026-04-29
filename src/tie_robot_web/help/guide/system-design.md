# 工程设计与架构图

这页记录当前工程的设计边界和主架构图。它不是旧包名的历史图，而是按当前 `tie_robot_*` 八个主包、独立驱动守护、常驻 `rosbridge_stack.launch` 和算法后端重新整理的版本。

![simple_lashingrobot_ws 工程设计总图](/images/architecture/tie-robot-system-architecture.svg)

[打开 PNG 版本](/images/architecture/tie-robot-system-architecture.png)

## 核心设计口径

- `tie_robot_msgs` 只放全局 `msg / srv / action`，避免接口散落在业务包里。
- `tie_robot_hw` 是硬件与协议层，只提供驱动层原子动作：索驱 TCP 帧、线性模组 Modbus 寄存器动作、相机 SDK 取帧。
- `tie_robot_perception` 负责相机世界点处理、PR-FPRG 视觉识别和 TF 相关感知节点。
- `tie_robot_control` 负责编排线性模组运动控制、到位等待、FINISHALL 等末端动作链。
- `tie_robot_process` 负责编排扫描、规划、账本、索驱区域位姿和整条绑扎执行链。
- `tie_robot_web` 负责浏览器前端、ROS bridge 接入、系统控制 HTTP 接口和帮助站。
- `tie_robot_bringup` 只做 launch 装配和 systemd 安装脚本。

## 驱动层原子动作

索驱和线性模组都遵循同一个边界：驱动层只表达硬件能直接理解的一步动作，执行链调度放在控制/算法层。

- 索驱驱动：`moveToPose()`、`sendStop()` 等 TCP 原子指令。
- 线性模组驱动：`writeQueuedPoints()`、`pulseExecutionEnable()`、`setZeroRequest()` 等寄存器原子动作。
- 控制/算法层：决定何时清 FINISHALL、何时写点位、何时触发 `EN_DISABLE 0->1`、何时等待到位或处理暂停。

这样做的目标是让硬件协议不掺业务状态机，后续排查“按钮下发了但没动”“暂停回报慢”“触发位没起边沿”时，可以快速定位是在驱动动作、控制编排还是硬件反馈。

## 运行分层

### 常驻接入层

`tie-robot-rosbridge.service` 拉起 `rosbridge_stack.launch`，里面常驻：

- `rosbridge_websocket` 和 `rosapi`
- `tf2_web_republisher`
- `robot_tf_broadcaster`
- `gripper_tf_broadcaster`
- `web_action_bridge_node`
- `system_log_mux`

它们不跟随后端算法栈重启，浏览器页面、TF 回传、Web Action 桥接和系统日志面板可以保持在线。

### 独立驱动守护

三个驱动由 systemd 分别守护：

- `tie-robot-driver-suoqu.service` -> `driver_suoqu.launch`
- `tie-robot-driver-moduan.service` -> `driver_moduan.launch`
- `tie-robot-driver-camera.service` -> `driver_camera.launch`

驱动之间互不拖拽；只测线性模组、只测相机或只测索驱时，不需要启动整套算法后端。

### 算法后端

`run.launch` 只包含 `algorithm_stack.launch`，主节点包括：

- `pointAINode`
- `bind_map_builder`
- `global_bind_planner`
- `cabin_motion_controller`
- `moduan_motion_controller`
- `bind_task_executor`

后端可以由前端按钮通过受限 systemd 接口启动、停止或重启。后端未启动时，前端 Action 会明确失败，不再伪装成“有 rosbridge 发布但没有服务端订阅”。

## 主数据闭环

```text
浏览器前端
-> rosbridge_websocket
-> web_action_bridge_node / 前端直连话题
-> bind_task_executor / pointAINode
-> cabin_motion_controller / moduan_motion_controller
-> suoqu_driver_node / moduan_driver_node
-> 真实硬件
```

视觉侧的数据闭环：

```text
Scepter 相机
-> scepter_manager
-> scepter_world_coord_processor
-> pointAINode(PR-FPRG)
-> /coordinate_point + /pointAI/* result
-> bind_task_executor + 前端 3D Scene
```

执行侧的数据闭环：

```text
bind_task_executor
-> pseudo_slam_points.json
-> pseudo_slam_bind_path.json
-> 索驱 area.cabin_pose
-> 线性模组 group.points[].x/y/z
-> bind_execution_memory.json
```

## 图源

图源文件随帮助站一起维护：

- `/images/architecture/tie-robot-system-architecture.dot`
- `/images/architecture/tie-robot-system-architecture.svg`
- `/images/architecture/tie-robot-system-architecture.png`

修改架构边界或 launch 拓扑后，请同步更新本页和图资产。
