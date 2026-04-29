# Tie Robot Workspace

这是当前 `simple_lashingrobot_ws` 的主工作空间，用于钢筋绑扎机器人整机联调。  
仓库已经按新的 ROS1 分包结构收口，打开 GitHub 首页时，先看这份 `README` 就能快速搞清：

- 这个工程现在由哪些主包组成
- 扫描、规划、执行、前端分别落在哪一层
- 应该从哪里启动、从哪里读代码
- 详细帮助站和相机 SDK 文档在哪

## 项目当前目标

当前主链围绕 4 件事展开：

1. 相机出图、出世界点、出识别绑扎点
2. 扫描后生成 `pseudo_slam_points.json` 和 `pseudo_slam_bind_path.json`
3. 执行层按账本驱动索驱和线性模组完成绑扎
4. 在新前端里同时查看 IR、3D 场景、Topic Layers 和运行状态

## 当前主包

当前 `src/` 下只保留这 8 个主包：

- `tie_robot_msgs`
  全局 `msg / srv / action`
- `tie_robot_hw`
  硬件与 SDK 接入层，含相机、索驱 TCP、线性模组 Modbus
- `tie_robot_perception`
  相机节点、世界点处理、PointAI 识别算法、TF 广播
- `tie_robot_control`
  线性模组控制与执行
- `tie_robot_process`
  扫描、规划、账本、执行流程编排
- `tie_robot_web`
  新前端、ROS bridge、帮助站、静态服务
- `tie_robot_bringup`
  launch 装配和系统启动入口
- `tie_robot_description`
  当前简化机器人模型与描述资源

## 一张图看主链

```text
相机/硬件
-> tie_robot_hw
-> tie_robot_perception
-> tie_robot_process
-> tie_robot_control
-> 真实绑扎执行

新前端
-> tie_robot_web
-> tie_robot_process / tie_robot_control / tie_robot_perception
-> ROS Action / Service / Topic
```

## 当前动作链

### 1. 扫描建图

```text
前端 /web/cabin/start_pseudo_slam_scan
-> tie_robot_web action bridge
-> tie_robot_process::run_pseudo_slam_scan
-> tie_robot_perception::pointai_node
-> pseudo_slam_points.json
-> pseudo_slam_bind_path.json
-> bind_execution_memory.json(重置)
```

### 2. 执行层

```text
前端 /web/cabin/start_global_work
-> tie_robot_web action bridge
-> /cabin/set_execution_mode
-> tie_robot_process::startGlobalWorkWithOptions
-> 优先按 pseudo_slam_bind_path.json
-> 索驱移动到 area.cabin_pose
-> 线模执行 group.points[].x/y/z
-> bind_execution_memory.json(成功点回写)
```

### 3. 直接执行账本测试

```text
前端 /web/cabin/run_bind_path_direct_test
-> tie_robot_web action bridge
-> tie_robot_process::run_bind_path_direct_test
-> 只读 pseudo_slam_bind_path.json
-> 不走 live 视觉 / 不清记忆 / 不写记忆
```

## 新前端

新前端已经不是旧的单页脚本堆叠，而是按 `robot_viewer` 风格重构成源码工程，入口在：

- 前端源码：[src/tie_robot_web/frontend](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/frontend)
- 构建产物：[src/tie_robot_web/web](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/web)
- 静态服务：[workspace_picker_web_server.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/scripts/workspace_picker_web_server.py:1)

当前前端关键能力：

- IR 底图与工作区四边形选点
- `3D Scene` 三维窗口
- `Topic Layers` 图层窗口
- 相机视角 / 全局视角切换
- 机器、点云、绑扎点、规划点联合显示
- 老前端常用命令迁移面板

三维场景当前直接消费这些话题：

- `/tf`
- `/tf_static`
- `/coordinate_point`，pointAI 相机坐标系原始绑扎点
- `/cabin/pseudo_slam_markers`
- `/Scepter/worldCoord/world_coord`
- `/Scepter/worldCoord/raw_world_coord`

## 启动方式

### ROS 后端启动

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_bringup run.launch
```

后端 launch 在：[run.launch](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_bringup/launch/run.launch:1)

`run.launch` 只负责 ROS 后端算法：视觉算法、全局绑扎点构图、路径规划和执行编排。前端静态服务、rosbridge WebSocket、TF 层、Web Action/Service 桥接和硬件驱动守护不再跟随后端 launch 生命周期。

生产运行时 ROS 后端也可以交给 systemd 管理，前端的“启动 ROS / 重启 ROS / 停止 ROS”按钮会调用这个 unit：

```bash
sudo src/tie_robot_bringup/scripts/install_backend_service.sh
systemctl status tie-robot-backend.service
```

该安装脚本只安装 `tie-robot-backend.service` 和受限 sudoers 白名单，不会默认设置 ROS 后端开机自启；后端由前端或人工通过 systemd 拉起。

### 前端与 rosbridge 守护

```text
http://127.0.0.1:8080/index.html
```

前端静态服务、本机 rosbridge、TF 层和三个硬件驱动都由独立 systemd 守护负责，安装后会随系统启动，并由 systemd 自动拉起：

```bash
sudo src/tie_robot_bringup/scripts/install_frontend_autostart.sh
systemctl status tie-robot-frontend.service
systemctl status tie-robot-rosbridge.service
systemctl status tie-robot-driver-suoqu.service
systemctl status tie-robot-driver-moduan.service
systemctl status tie-robot-driver-camera.service
```

前端守护安装脚本会同时确保 `tie-robot-rosbridge.service`、三个驱动 service、ROS 后端 systemd unit 与前端受限控制权限存在。前端 service、rosbridge service、驱动 service、ROS 后端 service 彼此独立，重启后端不会杀掉浏览器页面、WebSocket 守护、TF 层或驱动守护。

`tie-robot-rosbridge.service` 通过 `rosbridge_stack.launch` 同时拉起 `tf_stack.launch` 和 `api.launch`，TF 层与 Web Action/Service 桥接常驻守护：

- `tf2_web_republisher`：向前端转发 `/tf`。
- `robot_tf_broadcaster`：订阅 `/cabin/cabin_data_upload`，连续发布 `map -> base_link -> Scepter_depth_frame`；索驱状态断流时保留最后位姿并继续等待重连。
- `gripper_tf_broadcaster`：按 `gripper_tf.yaml` 和 `/web/tf/*` 设置连续发布 `Scepter_depth_frame -> gripper_frame`。
- `web_action_bridge_node`：把前端 Action 转成后端 `/cabin/*` 服务调用；后端未启动时会明确失败，不再出现 Action 无订阅者。
- `system_log_mux`：汇总 ROS 标准日志到前端日志话题。

驱动和 ROS 后端 systemd unit 启动前会等待 `tie-robot-rosbridge.service` 拥有的本机 ROS master 可用，避免单个驱动抢先启动自己的 roscore。

三个驱动 service 分别对应：

- `tie-robot-driver-suoqu.service`：只拉起索驱驱动 launch。
- `tie-robot-driver-moduan.service`：只拉起线性模组驱动 launch。
- `tie-robot-driver-camera.service`：只拉起相机驱动 launch。

这样定点绑扎测试可以只启线性模组，视觉识别可以只启相机，某个驱动断链或重启不会把整套后端一起带掉。前端 header 的索驱、末端、视觉状态胶囊提供对应子系统的启动和关闭入口。

如果需要手动在 ROS 环境下启动前端服务，可使用：

```bash
roslaunch tie_robot_bringup frontend.launch
```

systemd 守护默认直接运行静态服务脚本，不会启动 `run.launch`，因此重启后端不会杀掉前端页面。

如果只想单独安装或重启 rosbridge 守护，可使用：

```bash
sudo src/tie_robot_bringup/scripts/install_rosbridge_service.sh
systemctl status tie-robot-rosbridge.service
```

如果只想单独安装驱动守护，可使用：

```bash
sudo src/tie_robot_bringup/scripts/install_driver_services.sh
systemctl status tie-robot-driver-suoqu.service tie-robot-driver-moduan.service tie-robot-driver-camera.service
```

如果端口 `8080` 被占用，静态服务会自动顺延到更高端口。

```bash
src/tie_robot_web/scripts/workspace_picker_web_server.py --no-ros --host 0.0.0.0 --port 8080
```

### 单独构建前端

```bash
cd src/tie_robot_web/frontend
npm install
npm run build
```

### 单独构建帮助站

```bash
cd src/tie_robot_web/help
npm install
npm run build
```

## 目录结构

```text
simple_lashingrobot_ws/
├── src/
│   ├── tie_robot_msgs/
│   ├── tie_robot_hw/
│   ├── tie_robot_perception/
│   ├── tie_robot_control/
│   ├── tie_robot_process/
│   ├── tie_robot_web/
│   ├── tie_robot_bringup/
│   └── tie_robot_description/
├── docs/
├── ScepterSDK/
├── robot_viewer/
└── vzense_wiki/
```

## 从哪里开始读代码

如果你第一次进这个仓库，建议按下面顺序看：

1. 启动装配：[src/tie_robot_bringup/launch/run.launch](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_bringup/launch/run.launch:1)
2. 前端入口：[src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js:1)
3. Web Action/Service bridge：[src/tie_robot_web/src/web_action_bridge.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/src/web_action_bridge.cpp:1)
4. 流程主入口：[src/tie_robot_process/src/suoquNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/src/suoquNode.cpp:1)
5. 线模主入口：[src/tie_robot_control/src/moduanNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_control/src/moduanNode.cpp:1)
6. 视觉主入口：[src/tie_robot_perception/scripts/pointai_node.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/scripts/pointai_node.py:1)，节点实现：[src/tie_robot_perception/src/tie_robot_perception/pointai/node.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/tie_robot_perception/pointai/node.py:1)

## 关键运行数据

当前流程最关键的 3 份账本/运行数据在：

- [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/pseudo_slam_points.json)
- [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/pseudo_slam_bind_path.json)
- [bind_execution_memory.json](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/bind_execution_memory.json)

## Agent 共享记忆

进入本仓库的新旧 Codex 会话或其他 AI agent，先按根目录 [AGENTS.md](/home/hyq-/simple_lashingrobot_ws/AGENTS.md:1) 的顺序读取项目入口和共享记忆。

- 记忆协议：[docs/agent_memory/README.md](/home/hyq-/simple_lashingrobot_ws/docs/agent_memory/README.md:1)
- 当前快照：[docs/agent_memory/current.md](/home/hyq-/simple_lashingrobot_ws/docs/agent_memory/current.md:1)
- 有机体协议：[docs/agent_memory/organism.md](/home/hyq-/simple_lashingrobot_ws/docs/agent_memory/organism.md:1)
- Codex 本地启动说明：[docs/agent_memory/codex_local_setup.md](/home/hyq-/simple_lashingrobot_ws/docs/agent_memory/codex_local_setup.md:1)
- 断电恢复层：[docs/agent_memory/power_loss_recovery.md](/home/hyq-/simple_lashingrobot_ws/docs/agent_memory/power_loss_recovery.md:1)
- 最近恢复点：[docs/agent_memory/checkpoint.md](/home/hyq-/simple_lashingrobot_ws/docs/agent_memory/checkpoint.md:1)
- 追加账本：[docs/agent_memory/session_log.md](/home/hyq-/simple_lashingrobot_ws/docs/agent_memory/session_log.md:1)
- 维护脚本：[scripts/agent_memory.py](/home/hyq-/simple_lashingrobot_ws/scripts/agent_memory.py:1)

关键工程知识、架构决策、避坑经验和跨会话必须继承的修改，需要写入这套共享记忆，避免只停留在单个会话上下文里。
长任务或高风险修改前后，使用 `python3 scripts/agent_memory.py checkpoint ...` 写入断电恢复点；断电或会话中断后，使用 `python3 scripts/agent_memory.py recover` 恢复现场。

## 更详细的文档

如果需要比 GitHub 首页更细的说明，继续看这些：

- 工程帮助站入口：[src/tie_robot_web/help/index.md](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/help/index.md:1)
- 工程总览：[overview.md](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/help/guide/overview.md:1)
- 运行主链：[runtime-flows.md](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/help/guide/runtime-flows.md:1)
- 开发入口：[dev-entrypoints.md](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/help/guide/dev-entrypoints.md:1)
- 相机 SDK 文档入口：[camera-sdk/index.md](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/help/camera-sdk/index.md:1)
- 当前系统交接文档：[2026-04-22_current_system_handoff.md](/home/hyq-/simple_lashingrobot_ws/docs/handoff/2026-04-22_current_system_handoff.md:1)

## 现阶段边界

- 机器三维模型目前仍是简化模型，不是完整现场外形
- 3D 点云当前走 `world_coord/raw_world_coord` 图像采样，不是全量 `PointCloud2`
- `ScepterSDK/` 和 `vzense_wiki/` 仍保留在仓库里，主要用于原厂 SDK 参考与文档镜像
