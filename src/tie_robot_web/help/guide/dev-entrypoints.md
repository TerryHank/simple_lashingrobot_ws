# 开发入口

## 常用启动

- ROS 后端启动：`roslaunch tie_robot_bringup run.launch`
- 前端独立守护：`tie_robot_web/scripts/workspace_picker_web_server.py --no-ros`
- 前端 ROS 手动启动：`roslaunch tie_robot_bringup frontend.launch`
- 新前端源码目录：`src/tie_robot_web/frontend`
- 新前端构建命令：`cd src/tie_robot_web/frontend && npm run build`
- 帮助站目录：`src/tie_robot_web/web/help`

## 主要业务入口

- 前端源码入口：`src/tie_robot_web/frontend/src/main.js`
- 前端应用装配：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- 三维场景视图：`src/tie_robot_web/frontend/src/views/Scene3DView.js`
- 话题图层控制：`src/tie_robot_web/frontend/src/controllers/TopicLayerController.js`
- 三维显示模式配置：`src/tie_robot_web/frontend/src/config/topicLayerCatalog.js`
- 顶部工具条与整页布局：`src/tie_robot_web/frontend/src/ui/UIController.js`
- 浮动面板拖拽：`src/tie_robot_web/frontend/src/ui/PanelManager.js`
- 前端构建产物入口：`src/tie_robot_web/web/index.html`
- 桥接入口：`src/tie_robot_web/src/web_action_bridge.cpp`
- 前端话题注册表：`src/tie_robot_web/frontend/src/config/topicRegistry.js`
- 动态 API 网关入口：`src/tie_robot_web/scripts/ros_api_gateway.py`
- GB28181 视频接入包：`src/tie_robot_gb28181`
- 视觉 ROS 节点入口：`src/tie_robot_perception/scripts/pointai_node.py`
- 视觉算法包入口：`src/tie_robot_perception/src/tie_robot_perception/pointai/`
- 线模入口：`src/tie_robot_control/src/moduanNode.cpp`
- 流程入口：`src/tie_robot_process/src/suoquNode.cpp`

## 结构化后的模块入口

- 前端控制器：`src/tie_robot_web/frontend/src/controllers/`
- 前端 UI：`src/tie_robot_web/frontend/src/ui/`
- 前端视图：`src/tie_robot_web/frontend/src/views/`
- 前端配置：`src/tie_robot_web/frontend/src/config/`
- 前端工具：`src/tie_robot_web/frontend/src/utils/`
- Web bridge：`src/tie_robot_web/src/web_bridge/`
- 线模模块：`src/tie_robot_control/src/moduan/`
- PointAI 模块：`src/tie_robot_perception/src/tie_robot_perception/pointai/`
- 相机模块：`src/tie_robot_perception/src/camera/`
- 规划模块：`src/tie_robot_process/src/planning/`
- 索驱 transport：`src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- Marker 状态模块：`src/tie_robot_process/src/suoqu/pseudo_slam_markers.cpp`
- 扫描算法模块：`src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp`
- service 编排模块：`src/tie_robot_process/src/suoqu/service_orchestration.cpp`
- 区域执行模块：`src/tie_robot_process/src/suoqu/area_execution.cpp`
- 执行记忆模块：`src/tie_robot_process/src/suoqu/execution_memory_store.cpp`
- 账本读写模块：`src/tie_robot_process/src/suoqu/bind_path_store.cpp`
- 过渡占位片段：`src/tie_robot_process/src/suoqu/planned_bind_and_memory.inc`

## 三维窗口数据源

- 机器与相机运动：`/tf`、`/tf_static`
- 当前识别绑扎点：`/coordinate_point`，点值为 `Scepter_depth_frame` 相机原始坐标
- 规划点与执行高亮：`/cabin/pseudo_slam_markers`
- 过滤后世界点云：`/Scepter/worldCoord/world_coord`
- 原始世界点云：`/Scepter/worldCoord/raw_world_coord`

## 当前页面布局

- 整页背景：Three.js 三维场景
- 顶部：工具条
- 左侧：Workspace / Tasks
- 右侧：Topic Layers / Status
- 下方：Logs / Commands

## Topic Layers 显示模式

- `只显示点云`
- `只显示绑扎点`
- `点云 + 绑扎点`
- `规划点/执行点`
- `只显示机器`
- `全开`

## 账本与运行数据

- `src/tie_robot_process/data/pseudo_slam_points.json`
- `src/tie_robot_process/data/pseudo_slam_bind_path.json`
- `src/tie_robot_process/data/bind_execution_memory.json`
