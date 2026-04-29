# ROS Graph

这页放当前工程的 rqt_graph 风格 ROS graph。图片由 `scripts/generate_rqt_rosgraph.py` 调用 `rqt_graph.dotcode.RosGraphDotcodeGenerator` 从当前 ROS master 采样生成，模式等价于 rqt_graph 的 `Nodes/Topics (active)`，不是手工绘制的架构拓扑图。

![Tie Robot ROS graph 摘要](/images/architecture/tie-robot-ros-graph.svg)

[打开 PNG 版本](/images/architecture/tie-robot-ros-graph.png)

## 当前采样依据

本图结合两类信息整理：

- `rqt_graph.dotcode.RosGraphDotcodeGenerator` 读取当前 ROS master 的节点、topic 和连接关系。
- `rosnode list`、`rostopic list`、`rosservice list` 用于文字摘要核对。
- `src/tie_robot_bringup/launch/*.launch` 中的长期拓扑。

当前运行中可见的主节点包括：

```text
/rosbridge_websocket
/rosapi
/web_action_bridge_node
/system_log_mux
/tf2_web_republisher
/robot_tf_broadcaster
/gripper_tf_broadcaster
/suoqu_driver_node
/moduan_driver_node
/scepter_manager
/scepter_world_coord_processor
/pointAINode
/bind_map_builder
/global_bind_planner
/cabin_motion_controller
/moduan_motion_controller
/bind_task_executor
```

## rosbridge_stack.launch

`rosbridge_stack.launch` 是浏览器能看到 ROS 世界的常驻入口：

- `rosbridge_websocket`：浏览器 WebSocket 通道。
- `rosapi`：前端运行时查询 topic、service、message 结构。
- `tf2_web_republisher`：把 `/tf` 转成浏览器可消费的 TF 流。
- `robot_tf_broadcaster`：从 `/cabin/cabin_data_upload` 发布 `map -> base_link -> Scepter_depth_frame`。
- `gripper_tf_broadcaster`：发布 `Scepter_depth_frame -> gripper_frame`。
- `web_action_bridge_node`：把 `/web/cabin/*` Action 转成后端服务调用。
- `system_log_mux`：聚合 `/rosout_agg` 到 `/system_log/all`。

## 核心 topic

前端直接关心的高频 topic：

```text
/tf
/tf_static
/coordinate_point
/cabin/pseudo_slam_markers
/cabin/area_progress
/cabin/cabin_data_upload
/moduan/moduan_gesture_data
/Scepter/ir/image_raw
/Scepter/worldCoord/raw_world_coord
/Scepter/worldCoord/world_coord
/pointAI/manual_workspace_s2_result_raw
/system_log/all
```

前端发出的控制类 topic：

```text
/web/pointAI/set_workspace_quad
/web/pointAI/run_workspace_s2
/web/pointAI/set_height_threshold
/web/pointAI/set_stable_frame_count
/web/cabin/set_cabin_speed
/web/moduan/set_moduan_speed
/web/moduan/interrupt_stop
```

## 核心 service / action

浏览器侧 Action 入口：

```text
/web/cabin/start_pseudo_slam_scan
/web/cabin/start_global_work
/web/cabin/run_bind_path_direct_test
```

后端主服务：

```text
/cabin/start_pseudo_slam_scan_with_options
/cabin/start_work_with_options
/cabin/run_bind_path_direct_test
/cabin/set_execution_mode
/cabin/motion/stop
/moduan/driver/raw_execute_points
/moduan/single_move
/pointAI/process_image
```

系统控制服务：

```text
/web/system/start_algorithm_stack
/web/system/restart_algorithm_stack
/web/system/start_driver_stack
/web/system/restart_driver_stack
/web/system/restart_ros_stack
```

## 图源

图源文件随帮助站一起维护：

- `/images/architecture/tie-robot-ros-graph.dot`
- `/images/architecture/tie-robot-ros-graph.svg`
- `/images/architecture/tie-robot-ros-graph.png`

如果现场运行节点发生变化，先确认 ROS master 正在运行，然后重新生成：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source /opt/ros/noetic/setup.bash
python3 src/tie_robot_web/help/scripts/generate_rqt_rosgraph.py
cd src/tie_robot_web/help
npm run build
```
