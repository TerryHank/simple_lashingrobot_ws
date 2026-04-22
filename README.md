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
-> tie_robot_perception::pointAI
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
- `/coordinate_point`
- `/cabin/pseudo_slam_markers`
- `/Scepter/worldCoord/world_coord`
- `/Scepter/worldCoord/raw_world_coord`

## 启动方式

### 主启动

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_bringup run.launch
```

主 launch 在：[run.launch](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_bringup/launch/run.launch:1)

默认前端地址是：

```text
http://127.0.0.1:8080/index.html
```

如果端口 `8080` 被占用，静态服务会自动顺延到更高端口；此时可通过下面命令查看实际地址：

```bash
rosparam get /workspace_picker_web/url
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
3. Web bridge：[src/tie_robot_web/src/topics_transfer.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/src/topics_transfer.cpp:1)
4. 流程主入口：[src/tie_robot_process/src/suoquNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/src/suoquNode.cpp:1)
5. 线模主入口：[src/tie_robot_control/src/moduanNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_control/src/moduanNode.cpp:1)
6. 视觉主入口：[src/tie_robot_perception/scripts/pointAI.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/scripts/pointAI.py:1)

## 关键运行数据

当前流程最关键的 3 份账本/运行数据在：

- [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/pseudo_slam_points.json)
- [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/pseudo_slam_bind_path.json)
- [bind_execution_memory.json](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/data/bind_execution_memory.json)

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
