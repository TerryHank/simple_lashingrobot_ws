# slam/v30 离线复现交接文档

## 版本定位

- 发布名：`slam/v30`
- 发布日期：2026-04-29
- 当前分支：`slam`
- 标签形式：Git tag `slam/v30`
- 主要目标：打包当前前后端、ROS 包、帮助站、Codex 工程记忆、视觉模态样例和本地实验输出，支撑无机器条件下的仿真回放与视觉离线实验。

## 本次发布新增内容

- 新增 `src/tie_robot_perception/tools/export_visual_modalities_snapshot.py`，从当前 ROS 运行态导出一帧视觉/状态快照：
  - 写入 `slam_v30_visual_modalities.bag`
  - 额外保存 `PNG` 预览图、`NPY` 原始数组和文本化 ROS 消息
  - 生成 `metadata.json`，记录每个话题是否捕获成功
- 新增 `src/tie_robot_bringup/launch/slam_v30_offline_visual_replay.launch`，用于 `rosbag play --clock --loop` 离线回放。
- 新增 `docs/releases/slam_v30/` 发布目录，包含交接文档、校验文件和 `.debug_frames` 实验清单。
- 修复 `pointAINode` 运行态方法绑定缺口：`PR-FPRG` 主链实际运行需要绑定线族评分、结构边缘抑制和结构边缘过滤函数，否则 `/perception/lashing/recognize_once` 会在现场报 `ImageProcessor object has no attribute ...`。

## 离线视觉模态

离线样例目录：`docs/releases/slam_v30/visual_modalities/`

已捕获 19/23 个话题，bag 大小约 3.6 MB，包含：

| 类别 | 话题 | 说明 |
| --- | --- | --- |
| 彩色图 | `/Scepter/color/image_raw` | `bgr8`，480 x 640 |
| 深度图 | `/Scepter/depth/image_raw` | `16UC1`，480 x 640 |
| 红外图 | `/Scepter/ir/image_raw` | `8UC1`，480 x 640 |
| 对齐图 | `/Scepter/transformedColor/image_raw` | 对齐彩色图 |
| 对齐深度 | `/Scepter/transformedDepth/image_raw` | 对齐深度图 |
| 世界坐标 | `/Scepter/worldCoord/raw_world_coord` | `32FC3`，相机原始世界坐标 |
| 世界坐标 | `/Scepter/worldCoord/world_coord` | `32FC3`，处理后世界坐标 |
| pointAI 原始结果 | `/pointAI/result_image_raw` | PR-FPRG 中间结果图 |
| 识别结果图 | `/perception/lashing/result_image` | 带结果可视化的 `bgr8` 图 |
| 识别点 | `/perception/lashing/points_camera` | `PointsArray`，本次识别 46 个相机坐标点 |
| 工作区四边形 | `/perception/lashing/workspace/quad_pixels` | `[161,63,489,66,485,441,156,430]` |
| 运动状态 | `/cabin/cabin_data_upload`、`/moduan/moduan_gesture_data` | 前端 3D 和状态面板可用 |
| TF | `/tf`、`/tf_static` | 回放场景坐标关系 |

未捕获的话题：

- `/pointAI/line_image`：当前运行链路没有发布者。
- `/coordinate_point`：旧兼容话题在本次录制窗口未吐帧，当前前端主链已使用 `/perception/lashing/points_camera`。
- `/robot/binding_gun_status`、`/robot/moduan_status`：本次现场运行态没有新消息。

## 完全仿真复现流程

### 1. 只回放场景

```bash
cd /home/hyq-/simple_lashingrobot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
```

另开终端：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_bringup slam_v30_offline_visual_replay.launch
```

验证：

```bash
rostopic list | grep -E 'Scepter|perception/lashing|pointAI|tf'
rostopic echo -n 1 /perception/lashing/points_camera
```

### 2. 带前端复现

另开一个终端启动 rosbridge 和 Web 静态服务：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_bringup rosbridge_stack.launch
```

再开一个终端：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_bringup frontend.launch
```

浏览器打开 `http://127.0.0.1:8080/`。此时前端看到的是 bag 回放的话题，不会启动真实索驱、线性模组或相机驱动。

### 3. 重新跑 pointAI，而不是只看 bag 内结果

先启动上面的 bag 回放，再另开终端：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun tie_robot_perception pointai_node.py
```

等待 `/perception/lashing/recognize_once` 服务出现后触发：

```bash
rosservice call /perception/lashing/recognize_once "{}"
```

预期返回：

```text
success: True
message: "manual workspace S2 finished"
```

如果需要重新导出模态：

```bash
python3 src/tie_robot_perception/tools/export_visual_modalities_snapshot.py --timeout 10
```

为了捕获 pointAI 的非 latched 结果话题，导出脚本运行期间再触发一次 `recognize_once`。

## 下一次 Codex 的启动顺序

新 Codex 会话按根目录 `AGENTS.md` 轻启动：

1. 读 `README.md`
2. 读 `CHANGELOG.md`
3. 读 `docs/agent_memory/current.md`
4. 针对本任务继续读 `docs/releases/slam_v30/SLAM_V30_HANDOFF.md`
5. 如需追溯实验，再看 `docs/releases/slam_v30/debug_frames_manifest.tsv` 和 `.debug_frames/`

本发布包内的跨会话先验知识主要在：

- `docs/agent_memory/`
- `docs/handoff/`
- `docs/superpowers/plans/`
- `docs/superpowers/specs/`
- `docs/releases/slam_v30/`

## 实验与压缩包说明

`.debug_frames/` 当前约 728 MB，包含 PR-FPRG 各阶段、消融、现场快照和可视化过程。为了避免 Git 历史被大体量实验图像污染，Git tag 中保留发布清单与可复现的小型视觉样例；完整 `.debug_frames/` 会随整工程 zip 一起打包。

实验目录索引见：

```text
docs/releases/slam_v30/debug_frames_manifest.tsv
```

校验文件见：

```text
docs/releases/slam_v30/checksums.sha256
```

## 已知边界

- `slam_v30_visual_modalities.bag` 是单帧级场景快照，用于前端/视觉离线复现实验，不模拟真实电机运动闭环。
- 运动控制节点可以启动，但不要在无硬件环境下执行真实 `/cabin/driver/*` 或 `/moduan/driver/*` 控制服务。
- 本发布包的仿真复现重点是相机模态、TF、pointAI PR-FPRG 输出和前端展示链路。
