# SLAM 快速入门

本文档对应当前工程的 `slam` 分支现状。
它覆盖了 `slam/v11` 以及后续所有和伪 `slam` 扫描、全局棋盘格、执行记忆、`live_visual`、fail-closed 保护相关的新增内容，目标是让你用最少步骤完成：

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

### 方法 A：直接调老服务

```bash
cd /home/hyq-/simple_lashingrobot_ws
source devel/setup.bash
rosservice call /cabin/start_pseudo_slam_scan
```

这是兼容老脚本的方式，等价于“开启最终采集门”。

### 方法 B：直接调带参数的新服务

开启“停稳才拍照”：

```bash
rosservice call /cabin/start_pseudo_slam_scan_with_options "enable_capture_gate: true"
```

关闭“停稳才拍照”，按老版本的快节奏直接扫：

```bash
rosservice call /cabin/start_pseudo_slam_scan_with_options "enable_capture_gate: false"
```

### 方法 C：走前端同名入口

```bash
rostopic pub -1 /web/cabin/start_pseudo_slam_scan std_msgs/Float32 "data: 1.0"
```

话题转发节点会把它转成上面的扫描服务。

补充：

- `data: 1.0` 表示开启最终采集门
- `data: 0.0` 表示关闭最终采集门

### 扫描阶段当前行为

- 如果扫描请求里 `enable_capture_gate=false`，会跳过最终采集门，直接请求 `scan_only`
- 到每个区域后，会先过“最终采集门”
- 最终采集门同时看：
  - 索驱是否真正静止
  - `/Scepter/ir/image_raw` 白框 ROI 的灰度均差是否足够小
- 通过后才会请求 `scan_only`
- 当前扫描最小点数门槛是 `5`
- 少于 `5` 个点会继续轮询视觉
- 每次 `scan_only` 都会新建一份视觉服务请求对象，不会把上一轮成功响应误带到下一轮区域
- `pseudo_slam` 的扫描和执行主链现在共用同一把串行化锁
  - 扫描、全局执行、当前区域预计算直执行、`live_visual` 不会再并发改同一份扫描产物和执行账本
  - 如果你同时触发两条主链，后来的那条会等待前一条结束，而不是并发踩状态

### 扫描完成后会生成三个文件

- [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json)
  - 扫到的全部世界点
  - 同时写入全局棋盘格账本：`global_row`、`global_col`、`checkerboard_parity`、`is_checkerboard_member`
  - 同时写入规划层字段：`planning_global_row`、`planning_global_col`、`planning_checkerboard_parity`、`is_planning_checkerboard_member`
  - `live_visual` 后续只认这套规划层棋盘格字段，不会拿纯显示点直接执行
  - 同时写入 `scan_session_id`
  - 同时写入 `path_signature`，用于绑定“这批扫描点是按哪条路径和关键运动参数扫出来的”
  - `live_visual` 后续会复用这份扫描账本来判断现场新点能不能执行
- [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json)
  - 真正参与执行的区域分组路径
  - 组类型现在可能是标准 `matrix_2x2`，也可能是边界区域的 `edge_pair`
  - 同时写入 `scan_session_id`
  - 同样会持久化 `path_signature`
- [bind_execution_memory.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_execution_memory.json)
  - 已成功执行点位的全局记忆账本
  - `slam_precomputed` 和 `live_visual` 共用这份记忆，避免重复绑扎
  - 同时写入 `scan_session_id`
  - 账本也会带上 `path_signature`，执行时会和当前路径配置一起做一致性校验

### 边界区域现在也会生成 `edge_pair`

如果某个区域已经不能组成完整 `2x2`，但还能在同一根钢筋上稳定识别出 `2` 个合法点，扫描结果会把它写成 `edge_pair` 组，而不是整组丢掉。

对现场操作的影响是：

- `pseudo_slam_bind_path.json` 不再只包含 `2x2` 四点组
- 边界区域现在也有机会进入后续绑扎路径
- `edge_pair` 仍然受全局棋盘格跳绑和已执行记忆约束，不是强制必绑

## 5. 全局执行怎么触发

先确认全局执行模式。

默认值：

- 如果你没有主动发送 `/web/cabin/set_execution_mode`
- `/cabin/start_work` 会默认按 `live_visual` 模式执行

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
- `live_visual`：按 `path_points.json` 逐区域移动，到区域后现场请求视觉；但执行主依据仍然是扫描时写下的 `pseudo_slam_bind_path.json` 当前区域计划点，视觉只用于对这些扫描点做微调
- `live_visual` 当前区域如果在扫描账本里没有计划绑扎点，会直接跳过，不会因为现场杂点或边框点重新下探
- `live_visual` 现场新点如果不在当前区域扫描参考点中，会直接忽略
- `live_visual` 现场新点只有在 `x/y/z` 三轴都落进扫描参考点 `±5mm` 微调范围内，才会替换扫描点坐标；只要任一轴超过 `±5mm`，就保留扫描点，不再因为这类离群点跳过当前区域
- 三个执行入口现在都会联合校验 `pseudo_slam_points.json`、`pseudo_slam_bind_path.json` 和 `bind_execution_memory.json` 的 `scan_session_id`
- 三个执行入口也都会联合校验三份文件里的 `path_signature` 是否仍然匹配当前 `con_path + 关键运动参数`
- 不是只校验各自直接消费的那一份扫描产物；任意一份扫描产物已失效或 session 不对齐，都会按 fail-closed 拒绝继续执行
- 只要你改了路径点、`cabin_height`、`cabin_speed` 这类关键运动参数，就必须重新扫描，不能继续吃旧扫描网格
- 三个执行入口在真正下发到底层前，还会做“批内同格点去重”
  - 如果当前这一批里有两个点最终落到了同一个 `global_row/global_col`
  - 系统只会保留其中一个，不会对同一绑扎位在同一轮里重复动作

再触发全局作业：

扫描成功后，再执行全局作业：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source devel/setup.bash
rosservice call /cabin/start_work "command: '全局运动请求'"
```

如果你要在本次全局作业开始前清空已执行记忆，再用这个新入口：

```bash
rosservice call /cabin/start_work_with_options "command: '全局运动请求'
clear_execution_memory: true"
```

语义：

- `clear_execution_memory=false`
  - 保留 [bind_execution_memory.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_execution_memory.json)
  - 继续在当前扫描账本上续跑
- `clear_execution_memory=true`
  - 执行前先按当前这套扫描产物重建空账本
  - 让本轮全局作业从头重新判断哪些点该绑
  - 不会删除 [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json) 和 [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json)

也可以走前端入口：

```bash
rostopic pub -1 /web/cabin/start_global_work std_msgs/Float32 "data: 1.0"
```

前端话题值语义现在是：

- `data: 1.0`
  - 保留执行记忆，直接开始全局作业
- `data: 2.0`
  - 先清空执行记忆，再开始全局作业

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
- 如果你改路径后必须重新扫描；因为扫描产物和账本都绑定了 `path_signature`，执行入口发现当前路径和扫描时路径不一致会直接拒绝继续执行
- 如果 `slam_precomputed`、`live_visual` 或“当前区域预计算直执行”里某次绑扎已经实际执行成功，但随后 `bind_execution_memory.json` 写盘失败，系统会立刻按失败处理，不会继续后续流程，也不会返回整体成功
- 发生这类失败时，系统会尽力把当前 `pseudo_slam_points.json` / `pseudo_slam_bind_path.json` 的 `scan_session_id` 主动失效化；后续三个执行入口都会联合校验两份扫描产物与账本的 session/path 是否对齐，任意一份失效都会被拦住
- 但这里要如实说清：扫描产物失效化本身也是一次磁盘写操作，如果连这一步也失败，系统只能返回失败并提示人工确认，不会假装已经自动闭锁
- 所以现场一旦看到“账本写盘失败 + 扫描产物失效化失败”这类日志，正确动作不是重试执行，而是先确认磁盘/权限问题，再重新扫描/重新建图

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
- 虽然这个入口直接消费的是 `pseudo_slam_bind_path.json`，但执行前也会联合校验 `pseudo_slam_points.json`、`pseudo_slam_bind_path.json` 和 `bind_execution_memory.json` 的 session/path 元数据

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

当前颜色语义：

- 青绿色：普通扫描点
- 黄色：当前正在执行的区域内点
- 红色：当前这一批已经下发到底层执行的点

补充说明：

- `slam_precomputed`
- `live_visual`
- `当前区域预计算直执行`

这三条执行链都会刷新同一套 RViz 高亮。

因为末端当前还是按一批点下发，所以如果某一批里同时有多个点在执行，这一批点会一起显示成红色，而不是只亮一个点。

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

### 9.7 热调伪 SLAM 离群阈值并刷新 RViz

当前 `pseudo_slam` 的离群点阈值也支持运行时热修改，不用重启 `suoquNode`。

参数名：

```bash
/suoquNode/pseudo_slam_planning_z_outlier_mm
```

它的语义是：

- 扫描点会先拟合出一个工作平面
- 然后看每个点相对这个平面的 `z` 残差
- 超出这个阈值的点，就视为离群点

例如：

```bash
rosparam set /suoquNode/pseudo_slam_planning_z_outlier_mm 8
rosparam set /suoquNode/pseudo_slam_planning_z_outlier_mm 12
rosparam set /suoquNode/pseudo_slam_planning_z_outlier_mm 3
```

当前默认值是 `8mm`。

热更新行为：

- `suoquNode` 会在运行中持续检查这个参数
- 一旦参数变化，会立刻对当前内存里的扫描点重新做一次离群后处理
- 然后立即重发 `/cabin/pseudo_slam_markers`
- 所以 RViz 画面会直接刷新，不需要重新扫描，也不需要重新启动节点

你可以这样看当前值：

```bash
rosparam get /suoquNode/pseudo_slam_planning_z_outlier_mm
```

如果热更新成功，终端里会出现类似：

```text
Cabin_log: pseudo_slam离群阈值热更新：8.00mm -> 12.00mm，开始刷新RViz离群点显示。
Cabin_log: pseudo_slam离群阈值热更新后已重算Marker离群状态，当前离群点X个，列邻域屏蔽点Y个。
```

注意：

- 这次热更新影响的是“当前这份扫描点在 RViz 里的离群判定和着色”
- 如果当前进程里还没有扫描点，或者没有从 [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json) 恢复出点，就不会看到刷新效果
- 它不会自动改写磁盘上的旧扫描产物
- 如果你要把新的离群判定真正固化进 [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json) 和 [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json)，还是需要重新扫描一次

## 10. 当前版本最常见问题

### 10.1 改了路径后为什么直接被拒绝执行

这是当前版本的保护行为，不是故障。

只要下面任一项变了：

- [path_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/path_points.json) 里的 `con_path`
- 扫描/执行使用的 `cabin_height`
- 扫描/执行使用的 `cabin_speed`

三条执行入口都会发现 `path_signature` 不一致，然后直接 fail-closed。

正确做法是：

1. 改完路径或关键运动参数
2. 重新执行一次扫描建图
3. 再做 `slam_precomputed` 或 `live_visual`

### 10.2 扫描成功但执行时报“没有可执行区域”

排查顺序：

1. 先看 [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json)
   - 如果这里为空，问题在扫描/视觉
2. 再看 [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json)
   - 如果 `areas` 为空，问题在执行层分组
3. 再看 `/system_log/all`
   - 重点搜 `pseudo_slam`、`checkerboard`、`跳过`

### 10.3 RViz 看不到点

先确认：

```bash
rostopic echo -n 1 /cabin/pseudo_slam_markers
```

如果有消息但 RViz 没显示，一般是：

- Fixed Frame 不是 `cabin_frame`
- 没加 `MarkerArray`
- 没加 `TF`

### 10.4 pointAI 没点或结果全错位

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

### 10.5 前端点了按钮没动作

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

### 10.6 扫描卡在“持续采集图像”

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

### 10.7 扫描走着走着停很久

先看：

```bash
rostopic echo /system_log/all
```

如果看到这种日志：

```text
等待轴0到位中...
Cabin_Error: 等待轴0到位超时...
```

说明不是视觉或执行记忆卡住，而是索驱没有真正到达目标区域。

当前版本的行为是：

- 扫描阶段如果 `X/Y/Z` 任一轴到位超时，会直接结束本次扫描
- 不会再像旧行为那样超时后继续往下扫
- 如果 TCP 短回包里带非零状态字，还会额外打印：
  - `TCP短回包状态字异常`

这时优先排查：

- 索驱本体是否真的在动
- 目标点是否超出当前索驱可达范围
- 底层回包状态字是否异常

### 10.7 同一个点为什么现在不会在一轮里重复绑

当前版本在真正下发之前会做两层保护：

- 历史账本去重：已经写进 [bind_execution_memory.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_execution_memory.json) 的点直接跳过
- 当前批次去重：如果这一批点里有多个点最终落到同一个 `global_row/global_col`，只保留一个

所以现在“同一绑扎位在一轮执行里被重复下发”的风险已经专门收掉了。

### 10.8 为什么现在不要同时触发扫描和执行

当前版本为了避免并发读写同一份扫描产物和执行账本，已经把这几条主链串行化了：

- `start_pseudo_slam_scan`
- `start_work`
- `bind_current_area_from_scan`
- `live_visual` 全局执行

这意味着：

- 同时触发时不会并发乱跑
- 后触发的那条会等待前一条结束
- 这是故意的安全保护，不是卡死

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

稳态扫描：

```bash
rosservice call /cabin/start_pseudo_slam_scan
```

快速扫描：

```bash
rosservice call /cabin/start_pseudo_slam_scan_with_options "enable_capture_gate: false"
```

5. 确认文件已生成

```bash
ls -l src/chassis_ctrl/data/pseudo_slam_points.json
ls -l src/chassis_ctrl/data/pseudo_slam_bind_path.json
ls -l src/chassis_ctrl/data/bind_execution_memory.json
```

6. 执行

```bash
rosservice call /cabin/start_work "command: '全局运动请求'"
```

如果这轮要从头重新判断已绑点，而不是续跑旧账本：

```bash
rosservice call /cabin/start_work_with_options "command: '全局运动请求'
clear_execution_memory: true"
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
- [StartPseudoSlamScan.srv](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/srv/StartPseudoSlamScan.srv)
- [pointAI.py](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/scripts/pointAI.py)
- [gripper_tf.yaml](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/config/gripper_tf.yaml)
- [pseudo_slam_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_points.json)
- [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json)
- [bind_execution_memory.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/bind_execution_memory.json)
