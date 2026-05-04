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
- 新增 `/reports/pr_fprg_curve_3456` 静态报告，用于离线比较 PR-FPRG 方案 3/4/5/6 的梁筋点级过滤和地板缝曲线牵引指标。该报告不改变现场默认方案 1 主链。
- 修复 `pointAINode` 运行态方法绑定缺口：`PR-FPRG` 主链实际运行需要绑定线族评分、结构边缘抑制和结构边缘过滤函数，否则 `/perception/lashing/recognize_once` 会在现场报 `ImageProcessor object has no attribute ...`。
- `suoquNode` 到线性模组绑扎执行的调用链改为 `/moduan/execute_bind_points` action 客户端，旧 `/moduan/sg_precomputed*` service 只作为历史路径看待；如果复现执行链，必须同时启动 action server 所在的 `moduan_motion_controller_node`。

## 本会话视觉实验总结

本节汇总 2026-04-29 从早上 05:00 到 16:40 左右围绕 pointAI / PR-FPRG 的全部现场实验结论，供后续离线复现和继续开发时继承。

### 当前主链

- 当前有效主链是方案 1：`combined_depth_ir_darkline` 组合响应优先，透视展开工作区后只对 rectified 图的行、列 profile 做峰值识别，输出固定 `0° / 90°` 正交网格。
- 不再把方向自适应 `theta/rho` 估计作为主链。之前“始终是斜的”的根因是运行入口仍在调用方向估计线族，即使底层响应图和峰值已经正确，最终线也会带角度。
- 方案 2 已彻底废案，不进入当前报告、主链或默认调参讨论。
- 方案 3/4/5/6 只作为曲线收束和后续改进对照：先用方案 1 找稳定拓扑，再考虑用曲线方案局部贴合；不能让曲线方案直接替代主链。
- 报告必须同时给出峰值图：`00_peak_supported_lines.png` 和 `00_peak_spacing_pruned_lines.png`。只看最终 overlay 不够，必须能追溯峰值候选和 spacing 收束前后的变化。

### 组合响应与底层模态

- 后续底层图像统一优先看组合响应，而不是单独深度响应或单独红外响应。
- 当前组合方式沿用本轮现场认可的 `0.68 * normalized_depth_dark_line + 0.32 * normalized_infrared_dark_line`，再做归一化；它比纯深度更能抑制地板纹理，比纯红外更稳。
- 深度响应报告保留为对照：纯深度可以解释钢筋/地板缝的结构响应，但不作为默认最终判定图。
- 红外辅助方案保留归档，尤其用于梁筋/宽强结构识别，但红外最终 rho 微调不是当前主链。

### 梁筋、地板缝与 TCP 侵入

- 钢筋可以穿过梁筋，不能因为梁筋区域直接删整条钢筋线或整条 rho。
- 梁筋处理只在最终绑扎点 / 图谱阶段做点级排除，当前规则是梁筋 mask 按真实尺度扩张 `±100 mm` 后过滤落在范围内的绑扎点。
- 本轮现场判断：梁筋只有纵向梁筋；横向“梁筋”多半是 TCP / 末端结构侵入相机造成的强响应，不能当真实梁筋。
- 地板缝会在组合响应里形成强线，尤其在关闭 spacing 或过度放宽峰值时会爆出大量假线。不要把“地板缝被最后过滤掉”当成合格，后续应继续在峰值 / 连续结构验证阶段提前抑制。
- 现场钢筋本身可能摆放不直，不完全是相机或透视造成；该问题已归档为后续改进方向，但当前主链仍按用户要求固定正交行/列峰值。

### 消融与性能结论

- “连续钢筋条验证 / ridge 验证”能帮助排除假线，但在当前行/列峰值主链中不再作为默认入口。历史消融显示，粗暴关闭或打开某个阶段都可能造成点数大幅波动。
- spacing prune 是必要的收束阶段；完全关闭时会产生大量重复线和假点。它不能依赖现场钢筋间距严格一致，后续改进方向是自适应间距簇，而不是固定阈值硬删。
- 性能目标仍是单帧视觉检测低于 `100 ms`。本轮已经把方向估计主链切回行/列 profile 峰值，避免全角度扫描；若后续真实场景仍超过目标，优先优化 spacing / lattice 搜索，再考虑 C++/Rust/Go 或二进制加速。
- 本轮曾验证过多个帧和尺度：同一算法在梁筋、TCP 遮挡和不同工作区尺度下点数会不同，因此报告必须绑定具体帧、响应图和峰值图，不要只引用一个最终点数。

### 关键报告与结果

| 目录 / URL | 结论 |
| --- | --- |
| `.debug_frames/pr_fprg_peak_supported_probe_20260429_053639/` | 早上 05 点左右的行/列峰值方案参考，近似正交，作为“不要斜”的历史基线。 |
| `.debug_frames/pr_fprg_scheme_comparison_20260429_072517/` | 首轮 1/3/4/5/6 独立方案对比，曲线方案可视化归档。 |
| `.debug_frames/pr_fprg_scheme_ablation_report_20260429_082133/` | 1-6 方案消融起点，确认方案 2 不再继续。 |
| `.debug_frames/pr_fprg_scheme1_combined_peak_20260429_142818/` | 用户认可的组合响应 + 方案 1 峰值效果，要求后续保持这种底层表达。 |
| `.debug_frames/pr_fprg_combo_beam_10cm_exclusion_20260429_151818/` | 梁筋 `±10 cm` 点级排除实验，确认钢筋线允许穿过梁筋。 |
| `.debug_frames/pr_fprg_dense_42_scheme_comparison_peak_20260429_161030/` | 追求至少 42 个点时的密集方案对比，并补齐峰值图。 |
| `.debug_frames/pr_fprg_axis_rowcol_scheme_comparison_20260429_163344/` | 最终回到固定行/列峰值正交主链的报告；方案 1 本帧为 7 条横线、8 条纵线，梁筋点级过滤后 49 个绑扎点。 |
| `http://192.168.6.99:8080/reports/pr_fprg_axis_rowcol_scheme_comparison/index.html` | 已发布的最终正交行/列峰值报告页面。 |

### 后续禁区

- 不要把方案 2 恢复成默认方案。
- 不要为了补齐理论交点数放宽工作区 mask，用户已经明确“不纠结理论交点 20 个”，优先保留视觉效果。
- 不要在主链里偷偷恢复 `theta/rho` 方向估计；如果要研究“钢筋本身斜 / 弯”的问题，必须作为单独报告和后续改进方案呈现。
- 不要只交最终 overlay，所有视觉报告都要附峰值图和响应图。

## 离线视觉模态

离线样例目录：`docs/releases/slam_v30/visual_modalities/`

已捕获 20/24 个话题，bag 大小约 3.6 MB，包含：

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
| 发布说明 | `/release/slam_v30/experiment_summary` | `std_msgs/String`，记录本轮视觉实验总结和最终 PR-FPRG 决策 |

注意：bag 内原始相机、TF、`/pointAI/result_image_raw`、`/perception/lashing/result_image` 和 `/perception/lashing/points_camera` 仍保持 2026-04-29 13:37 录制时的现场快照；`/release/slam_v30/experiment_summary` 是后续追加的发布说明话题，用来把本会话后续实验和最终主链决策带入离线包。

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

`.debug_frames/` 当前约 894 MB，包含 PR-FPRG 各阶段、消融、现场快照和可视化过程。为了避免 Git 历史被大体量实验图像污染，Git tag 中保留发布清单与可复现的小型视觉样例；完整 `.debug_frames/` 会随整工程 zip 一起打包。

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
- `/moduan/execute_bind_points` action 属于算法层到线性模组控制层的执行调度接口；驱动层仍只保留 Modbus/PLC 原子动作。
- 本发布包的仿真复现重点是相机模态、TF、pointAI PR-FPRG 输出和前端展示链路。
