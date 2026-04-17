# SLAM v11 快速入门

本文档对应当前工程的 `slam/v11` 版本，目标是让你用最少步骤完成：

- 启动整套系统
- 触发扫描建图
- 触发全局执行
- 做当前区域快测
- 快速定位常见问题

工程根目录：

```bash
/home/hyq-/simple_lashingrobot_ws
```

## 1. 先编译

在工程根目录执行：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1
source devel/setup.bash
```

## 2. 启动方式

当前版本建议分两个终端启动。

终端 1：启动相机、rosbridge、话题转发、日志汇聚

```bash
cd /home/hyq-/simple_lashingrobot_ws
source devel/setup.bash
roslaunch chassis_ctrl api.launch
```

终端 2：启动索驱、末端、视觉、TF、稳定点 TF

```bash
cd /home/hyq-/simple_lashingrobot_ws
source devel/setup.bash
roslaunch chassis_ctrl run.launch
```

也可以直接用脚本：

终端 1：

```bash
bash /home/hyq-/simple_lashingrobot_ws/start.sh
```

终端 2：

```bash
bash /home/hyq-/simple_lashingrobot_ws/start_.sh
```

## 3. 只验证 TF + pointAI

如果你只是想调相机到虎口的 TF，或者只看视觉结果，不想带上整套 api，可单独启动：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source devel/setup.bash
roslaunch chassis_ctrl pointai_tf_verify.launch
```

这个 launch 只会启动：

- `gripper_tf_broadcaster`
- `pointAINode`

适合改完 [gripper_tf.yaml](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/config/gripper_tf.yaml) 后快速验证。

## 4. 扫描建图怎么触发

### 方法 A：直接调服务

```bash
cd /home/hyq-/simple_lashingrobot_ws
source devel/setup.bash
rosservice call /cabin/start_pseudo_slam_scan
```

这是最直接、最稳定的方式。

### 方法 B：走前端同名入口

```bash
rostopic pub -1 /web/cabin/start_pseudo_slam_scan std_msgs/Float32 "data: 1.0"
```

话题转发节点会把它转成上面的扫描服务。

### 扫描阶段当前行为

- 到每个区域后，会先过“最终采集门”
- 最终采集门同时看：
  - 索驱是否真正静止
  - `/Scepter/ir/image_raw` 白框 ROI 的灰度均差是否足够小
- 通过后才会请求 `scan_only`
- 当前扫描最小点数门槛是 `5`
- 少于 `5` 个点会继续轮询视觉

### 扫描完成后会生成三个文件

- [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json)
  - 扫到的全部世界点
  - 同时写入全局棋盘格账本：`global_row`、`global_col`、`checkerboard_parity`、`is_checkerboard_member`
  - `live_visual` 后续会复用这份扫描账本来判断现场新点能不能执行
- [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json)
  - 真正参与执行的区域分组路径
  - 组类型现在可能是标准 `matrix_2x2`，也可能是边界区域的 `edge_pair`
- [bind_execution_memory.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_execution_memory.json)
  - 已成功执行点位的全局记忆账本
  - `slam_precomputed` 和 `live_visual` 共用这份记忆，避免重复绑扎

### 边界区域现在也会生成 `edge_pair`

如果某个区域已经不能组成完整 `2x2`，但还能在同一根钢筋上稳定识别出 `2` 个合法点，扫描结果会把它写成 `edge_pair` 组，而不是整组丢掉。

对现场操作的影响是：

- `pseudo_slam_bind_path.json` 不再只包含 `2x2` 四点组
- 边界区域现在也有机会进入后续绑扎路径
- `edge_pair` 仍然受全局棋盘格跳绑和已执行记忆约束，不是强制必绑

## 5. 全局执行怎么触发

先确认全局执行模式。

`slam_precomputed`：

```bash
rostopic pub -1 /web/cabin/set_execution_mode std_msgs/Float32 "data: 0.0"
```

`live_visual`：

```bash
rostopic pub -1 /web/cabin/set_execution_mode std_msgs/Float32 "data: 1.0"
```

含义：

- `slam_precomputed`：使用扫描后的 `pseudo_slam_bind_path.json` 执行；执行前还会读取 `bind_execution_memory.json`，把已经做过的点过滤掉
- `live_visual`：按 `path_points.json` 逐区域移动，到区域后现场请求视觉；但新点不会直接盲绑，而是先归入扫描时写下的全局棋盘格，再经过 `bind_execution_memory.json` 过滤后才执行

再触发全局作业：

扫描成功后，再执行全局作业：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source devel/setup.bash
rosservice call /cabin/start_work "command: '全局运动请求'"
```

也可以走前端入口：

```bash
rostopic pub -1 /web/cabin/start_global_work std_msgs/Float32 "data: 1.0"
```

### 注意

`slam_precomputed` 模式下：

- 必须先完成一次扫描建图
- 再执行全局作业

`live_visual` 模式下：

- 仍然需要先完成一次扫描建图，因为它要复用 [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json) 里的全局棋盘格账本
- 它不依赖 `pseudo_slam_bind_path.json` 里的预计算分组，但不再是“现场看到什么就直接绑什么”
- 现场新点会先尝试归入扫描得到的棋盘格；归不进去的点会直接跳过

### 重启与重扫行为

- 节点重启或整机重启后，已经成功执行过的点仍然会从 [bind_execution_memory.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_execution_memory.json) 里读回来
- 这意味着重启后继续执行时，会自动跳过已经记过账的点，不需要人工清空
- 只有当一次新的扫描成功，并且新的扫描产物已经写盘后，系统才会清空并重建 `bind_execution_memory.json`
- 如果扫描失败，旧的执行记忆会保留，不会被提前删除
- 如果 `slam_precomputed`、`live_visual` 或“当前区域预计算直执行”里某次绑扎已经实际执行成功，但随后 `bind_execution_memory.json` 写盘失败，系统会立刻按失败处理，不会继续后续流程，也不会返回整体成功
- 发生这类失败时，当前 `pseudo_slam_points.json` / `pseudo_slam_bind_path.json` 的 `scan_session_id` 会被主动失效化；后续无论是否重启，都会因为 session 对齐失败而被拦住，必须重新扫描/重新建图后再执行

如果你看到：

```text
pseudo_slam_bind_path.json没有可执行区域，请先确认扫描分组成功
```

说明不是服务坏了，而是：

- 扫描点有了
- 但没有成功生成可执行分组（包括 `matrix_2x2` 和 `edge_pair`）
- 这时需要先看 `pseudo_slam_bind_path.json` 里的 `areas` 是否为空

## 6. 当前区域快测怎么触发

### 方式 1：当前区域原逻辑识别 + 执行

```bash
rostopic pub -1 /web/moduan/single_bind std_msgs/Float32 "data: 0.0"
```

含义：

- 本区域重新请求视觉
- 本区域识别后直接执行

### 方式 2：当前区域使用扫描后的预计算点直执行

```bash
rostopic pub -1 /web/moduan/single_bind std_msgs/Float32 "data: 1.0"
```

含义：

- 不再重新识别
- 直接从 `pseudo_slam_bind_path.json` 里取当前区域的预计算点执行

也可以直接调服务：

```bash
rosservice call /cabin/bind_current_area_from_scan
```

## 7. 跳绑怎么开关

当前版本全局预计算执行使用的是“全局棋盘格跳绑”。

开启：

```bash
rostopic pub -1 /web/moduan/send_odd_points std_msgs/Bool "data: true"
```

关闭：

```bash
rostopic pub -1 /web/moduan/send_odd_points std_msgs/Bool "data: false"
```

当前语义：

- 开启后，只执行 `checkerboard_parity == 0` 的点
- 这是全局棋盘格跳绑，不是单个 `2x2` 内固定只绑 `1/4`

## 8. TF 偏移如何热修改

当前版本前端“修正视觉偏差”改的是：

- [gripper_tf.yaml](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/config/gripper_tf.yaml) 里的 `translation_mm`

特点：

- 修改后立即生效
- 同时写回 `gripper_tf.yaml`
- 不需要重启 `pointAI` 或 `run.launch`

手工检查可以用：

```bash
rosrun tf tf_echo Scepter_depth_frame gripper_frame
```

## 9. 最常用的调试入口

### 9.1 看整体日志

标准 ROS 日志 + `printf/std::cout` 汇总：

```bash
rostopic echo /system_log/all
```

按节点拆分的标准 ROS 日志：

```bash
rostopic echo /system_log/suoquNode
rostopic echo /system_log/moduanNode
rostopic echo /system_log/pointAINode
rostopic echo /system_log/topicTransNode
```

说明：

- `printf/std::cout` 主要进 `/system_log/all`
- `/system_log/<node>` 更适合看标准 `ROS_INFO/WARN/ERROR`

### 9.2 看扫描进度

```bash
rostopic echo /cabin/area_progress
```

字段含义：

- `current_area_index`：当前区域编号
- `total_area_count`：总区域数
- `just_finished_area_index`：刚完成的区域
- `ready_for_next_area`：当前区域已完成，可进入下一区域
- `all_done`：是否全部完成

### 9.3 看 RViz 扫描点

RViz 中固定坐标系建议先用：

```text
cabin_frame
```

显示项建议加：

- `TF`
- `MarkerArray`

Marker 话题：

```text
/cabin/pseudo_slam_markers
```

这会实时显示扫描累计的世界点和编号。

### 9.4 看 pointAI 图像输出

```bash
rostopic list | grep pointAI
```

常用图像话题：

- `/pointAI/line_image`
- `/pointAI/result_image_raw`
- `/pointAI/depth_binary_image`
- `/pointAI/cropped_ir_image`
- `/pointAI/cropped_depth_image`
- `/pointAI/cropped_color_image`

### 9.5 看 TF 是否正常

检查相机到虎口：

```bash
rosrun tf tf_echo Scepter_depth_frame gripper_frame
```

检查扫描点 TF 是否在发：

```bash
rosrun tf tf_echo cabin_frame pseudo_slam_point_1
```

如果第一号点不存在，可以把 `1` 改成实际存在的编号。

### 9.6 热调扫描稳态阈值

当前这些参数可以运行时直接改，不用重启 `suoquNode`：

```bash
rosparam set /suoquNode/pseudo_slam_capture_gate_image_mean_diff_threshold 2.5
rosparam set /suoquNode/pseudo_slam_capture_gate_stable_sample_count 3
rosparam set /suoquNode/pseudo_slam_capture_gate_poll_interval_sec 0.1
rosparam set /suoquNode/pseudo_slam_capture_gate_log_interval_sec 1.0
rosparam set /suoquNode/pseudo_slam_scan_min_point_count 5
rosparam set /suoquNode/pseudo_slam_scan_retry_interval_sec 0.2
```

如果你只是想看当前值：

```bash
rosparam get /suoquNode/pseudo_slam_capture_gate_image_mean_diff_threshold
rosparam get /suoquNode/pseudo_slam_scan_min_point_count
```

## 10. 当前版本最常见问题

### 10.1 扫描成功但执行时报“没有可执行区域”

排查顺序：

1. 先看 [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json)
   - 如果这里为空，问题在扫描/视觉
2. 再看 [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json)
   - 如果 `areas` 为空，问题在执行层分组
3. 再看 `/system_log/all`
   - 重点搜 `pseudo_slam`、`checkerboard`、`跳过`

### 10.2 RViz 看不到点

先确认：

```bash
rostopic echo -n 1 /cabin/pseudo_slam_markers
```

如果有消息但 RViz 没显示，一般是：

- Fixed Frame 不是 `cabin_frame`
- 没加 `MarkerArray`
- 没加 `TF`

### 10.3 pointAI 没点或结果全错位

先检查：

```bash
rosrun tf tf_echo Scepter_depth_frame gripper_frame
```

如果 TF 数值不对，改：

- [gripper_tf.yaml](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/config/gripper_tf.yaml)

前端修正视觉偏差后会实时生效；如果只是手工改了 YAML，再重启：

```bash
roslaunch chassis_ctrl pointai_tf_verify.launch
```

### 10.4 前端点了按钮没动作

先看：

```bash
rostopic echo /system_log/topicTransNode
```

如果这里没有“收到扫描建图命令”或“收到开始全局作业命令”，说明前端消息没进 ROS。

再看：

```bash
rostopic list | grep rosbridge
```

以及确认 `api.launch` 是否还在运行。

### 10.5 扫描卡在“持续采集图像”

先看：

```bash
rostopic echo /system_log/all
```

重点搜：

- `等待稳定中`
- `图像均差`
- `白色矩形内点数`

当前逻辑里，扫描会卡住的常见原因只有两类：

- 最终采集门还没通过
- 当前区域点数仍然少于 `5`

## 11. 一套最短操作流程

### 全局扫描 + 全局执行

1. 编译

```bash
cd /home/hyq-/simple_lashingrobot_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1
source devel/setup.bash
```

2. 启动

```bash
roslaunch chassis_ctrl api.launch
```

新终端：

```bash
roslaunch chassis_ctrl run.launch
```

3. 选择执行模式

预生成路径执行：

```bash
rostopic pub -1 /web/cabin/set_execution_mode std_msgs/Float32 "data: 0.0"
```

边执行边识别：

```bash
rostopic pub -1 /web/cabin/set_execution_mode std_msgs/Float32 "data: 1.0"
```

4. 扫描

```bash
rosservice call /cabin/start_pseudo_slam_scan
```

5. 确认文件已生成

```bash
ls -l src/chassis_ctrl/data/pseudo_slam_points.json
ls -l src/chassis_ctrl/data/pseudo_slam_bind_path.json
```

6. 执行

```bash
rosservice call /cabin/start_work "command: '全局运动请求'"
```

### 当前区域快测

扫描完成后直接：

```bash
rosservice call /cabin/bind_current_area_from_scan
```

或者：

```bash
rostopic pub -1 /web/moduan/single_bind std_msgs/Float32 "data: 1.0"
```

## 12. 当前版本涉及的关键文件

- [run.launch](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/launch/run.launch)
- [api.launch](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/launch/api.launch)
- [pointai_tf_verify.launch](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/launch/pointai_tf_verify.launch)
- [suoquNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp)
- [topics_transfer.cpp](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/topics_transfer.cpp)
- [pointAI.py](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/scripts/pointAI.py)
- [gripper_tf.yaml](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/config/gripper_tf.yaml)
- [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json)
- [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json)
