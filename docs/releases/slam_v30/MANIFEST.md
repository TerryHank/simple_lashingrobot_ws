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
| `visual_modalities/slam_v30_visual_modalities.bag` | 20 条 ROS 消息，覆盖相机、TF、前端状态、PR-FPRG 输出和 `/release/slam_v30/experiment_summary` 实验总结 |
| `visual_modalities/metadata.json` | 捕获时间、话题类型、成功/缺失清单、图像统计 |
| `visual_modalities/images/*.png` | 人眼可读预览图 |
| `visual_modalities/arrays/*.npy` | 原始图像/深度/世界坐标数组 |
| `visual_modalities/messages/*.txt` | CameraInfo、TF、PointsArray、运动状态文本快照和本轮 PR-FPRG 实验总结 |

## 本轮 PR-FPRG 实验结论

- 当前运行主链为方案 1：组合响应 `combined_depth_ir_darkline` + rectified 行/列 profile 峰值识别。
- 方案 1 固定输出 `0° / 90°` 正交网格，不再估计 `theta`，不再把方向自适应 `theta/rho` 斜线族作为主链。
- 方案 2 废弃；方案 3/4/5/6 仅作为曲线收束、地板缝牵引和梁筋处理的后续对照。
- 梁筋只做最终绑扎点级排除，按真实尺度扩张约 `±100 mm`；钢筋线允许穿过梁筋。
- PR-FPRG 报告必须包含峰值图：`00_peak_supported_lines.png` 和 `00_peak_spacing_pruned_lines.png`。
- 最终正交行/列报告已发布到 `/reports/pr_fprg_axis_rowcol_scheme_comparison/index.html`，本地目录为 `.debug_frames/pr_fprg_axis_rowcol_scheme_comparison_20260429_163344/`。
- bag 中追加 `/release/slam_v30/experiment_summary`，同内容文本快照位于 `visual_modalities/messages/release_slam_v30_experiment_summary.txt`。

## 本会话视觉实验归档

| 类别 | 路径 / URL | 说明 |
| --- | --- | --- |
| 最终主链报告 | `.debug_frames/pr_fprg_axis_rowcol_scheme_comparison_20260429_163344/` | 固定行/列 profile 峰值，方案 1 输出 `0° / 90°` 正交网格；本帧 7 x 8 线族，梁筋点级过滤后 49 个点。 |
| 已发布页面 | `/reports/pr_fprg_axis_rowcol_scheme_comparison/index.html` | 前端静态目录中的最终正交行/列峰值报告，包含响应图、峰值图、方案 1 和方案 3/4/5/6 对照。 |
| 峰值图要求 | `images/00_peak_supported_lines.png`、`images/00_peak_spacing_pruned_lines.png` | 后续报告必须保留这两张图，用于追踪候选峰和 spacing 收束。 |
| 组合响应基线 | `.debug_frames/pr_fprg_scheme1_combined_peak_20260429_142818/` | 用户认可的组合响应 + 方案 1 峰值效果，后续底层图像表达以组合响应为准。 |
| 梁筋点级排除 | `.debug_frames/pr_fprg_combo_beam_10cm_exclusion_20260429_151818/` | 梁筋 mask 按真实尺度扩张 `±100 mm` 后只过滤最终绑扎点，钢筋线允许穿过梁筋。 |
| 1-6 方案消融 | `.debug_frames/pr_fprg_dense_42_all_scheme_ablation_peak_20260429_161054/` | 方案 1/3/4/5/6 对照与阶段消融，方案 2 不再进入报告。 |
| 早期正交参考 | `.debug_frames/pr_fprg_peak_supported_probe_20260429_053639/` | 早上 05 点左右的行/列峰值基线，用于解释“不要斜”的用户要求。 |

## 当前算法口径

- 当前默认主链：方案 1，`combined_depth_ir_darkline` 组合响应优先，透视展开后固定行/列 profile 峰值，输出正交网格。
- 方案 2：废案，不打包为默认入口，不进入后续主报告。
- 方案 3/4/5/6：保留为曲线收束和鲁棒性对照，不能直接替代方案 1 主链。
- 梁筋：只在最终点级 / 图谱阶段排除 `±100 mm` 范围内绑扎点，不删除整条钢筋线。
- 地板缝：是当前主要误识别风险，后续应在峰值 / 结构验证阶段提前抑制；不要只依赖最后距离或点级过滤。

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

外层校验文件：

```text
/home/hyq-/simple_lashingrobot_ws_slam_v30.zip.sha256
```
