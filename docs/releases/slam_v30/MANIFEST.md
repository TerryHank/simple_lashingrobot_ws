# slam/v30 发布清单

## Git 与构建范围

- 分支：`slam`
- Tag：`slam/v30`
- 发布目录：`docs/releases/slam_v30/`
- 视觉复现 launch：`src/tie_robot_bringup/launch/slam_v30_offline_visual_replay.launch`
- 视觉模态导出工具：`src/tie_robot_perception/tools/export_visual_modalities_snapshot.py`

## 离线视觉包

| 文件 | 说明 |
| --- | --- |
| `visual_modalities/slam_v30_visual_modalities.bag` | 19 条 ROS 消息，覆盖相机、TF、前端状态和 PR-FPRG 输出 |
| `visual_modalities/metadata.json` | 捕获时间、话题类型、成功/缺失清单、图像统计 |
| `visual_modalities/images/*.png` | 人眼可读预览图 |
| `visual_modalities/arrays/*.npy` | 原始图像/深度/世界坐标数组 |
| `visual_modalities/messages/*.txt` | CameraInfo、TF、PointsArray、运动状态文本快照 |

## 关键校验

```bash
sha256sum -c docs/releases/slam_v30/checksums.sha256
```

## 本地完整 zip

整工程 zip 会放在仓库外：

```text
/home/hyq-/simple_lashingrobot_ws_slam_v30.zip
```

zip 包包含 Git tag 中的源码与文档，也包含 Git 未纳入历史的 `.debug_frames/` 实验输出和本地离线样例。
