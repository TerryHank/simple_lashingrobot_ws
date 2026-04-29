# slam/v30 发布包说明

`slam/v30` 是 2026-04-29 当前工作区的离线复现发布包。它面向两个目标：

1. 在没有真实机器时，用 ROS bag 复现当前相机、TF、前端 3D 和 pointAI 识别结果场景。
2. 让下一次 Codex 会话从工程记忆、交接文档和实验输出中恢复上下文。

核心入口：

- 交接文档：`docs/releases/slam_v30/SLAM_V30_HANDOFF.md`
- 离线模态：`docs/releases/slam_v30/visual_modalities/`
- 复现 launch：`src/tie_robot_bringup/launch/slam_v30_offline_visual_replay.launch`
- 导出脚本：`src/tie_robot_perception/tools/export_visual_modalities_snapshot.py`
- 校验文件：`docs/releases/slam_v30/checksums.sha256`
- 实验目录清单：`docs/releases/slam_v30/debug_frames_manifest.tsv`

## 快速复现

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

如需前端联调，再另开终端：

```bash
cd /home/hyq-/simple_lashingrobot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_bringup rosbridge_stack.launch
roslaunch tie_robot_bringup frontend.launch
```

打开 `http://127.0.0.1:8080/` 后，前端可通过 rosbridge 看到 bag 回放的话题、TF 和视觉结果。
