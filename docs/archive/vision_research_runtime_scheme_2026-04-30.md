# 视觉研究归档与当前运行方案（2026-04-30）

本文档冻结 2026-04-30 视觉研究的结论，并记录当前运行链路的选择。核心口径：

- 移动到识别位姿后触发扫描：走 2026-04-22 视觉方案，也就是 `PR-FPRG` 的完整拓扑恢复路线。
- 开始执行后，每到执行区域：走执行层局部视觉，方法为平面分割 + Hough。

## 研究归档

本轮已经沉淀的视觉研究不直接等同于现场运行主链，后续引用时按以下边界处理：

- `docs/archive/pr_fprg_previous_schemes_2026-04-30.md`：归档行/列峰值、曲线收束、红外 rho 微调、梁筋过滤和实例图探索的阶段结论。
- `/reports/pr_fprg_topology_recovery_flow/`：归档 2026-04-22 频相主拓扑恢复流程图，说明用 FFT / 自相关周期和相位恢复完整候选网格，再做梁筋过滤、曲线评分和局部收束。
- `/reports/pr_fprg_topology_recovery_experiment/`：归档当前帧的逐步实验结果，用于证明 4 月 22 日频相主拓扑可以恢复高召回候选。
- `/reports/pr_fprg_multiscale_bindpoint_experiment/`：归档多尺度锚点式融合研究。该路线保留为后续研究素材，不直接替换现场运行链。
- `docs/archive/legacy_ransac_hough_pointai/`：保留旧 `RANSAC + Hough + pre_img` 代码快照，仅作追溯和局部执行层实现参考。

## 当前运行链路

### 1. 固定识别位姿扫描

入口：

- `/web/cabin/start_pseudo_slam_scan`
- 后端 `/cabin/start_pseudo_slam_scan`
- 扫描策略：`kFixedManualWorkspace`

流程：

1. 索驱移动到固定识别位姿。
2. 调用 `/pointAI/process_image`，`request_mode = MODE_SCAN_ONLY`。
3. 视觉层运行 2026-04-22 `PR-FPRG` 拓扑恢复方案。
4. 生成并写入 `pseudo_slam_points.json` 和 `pseudo_slam_bind_path.json`。

这条链负责“建图 / 规划账本”，不走平面分割 + Hough。

### 2. 全局执行

入口：

- `/web/cabin/start_global_work`
- 执行模式：`live_visual`

流程：

1. 读取 `pseudo_slam_bind_path.json`。
2. 先回 `path_origin`。
3. 按 `areas[].cabin_pose` 逐区移动索驱。
4. 每到一个执行区域，调用 `/pointAI/process_image`，`request_mode = MODE_EXECUTION_REFINE`。
5. 执行层局部视觉走平面分割 + Hough，输出当前执行框内的相机原始坐标点。
6. 流程层将点匹配回全局棋盘格和扫描账本，再转为 TCP / 虎口局部坐标发送给线性模组。

执行层视觉只负责“到位后的局部修正”，不重新跑 4 月 22 日的扫描拓扑恢复。

## 实现边界

- `MODE_SCAN_ONLY` 是固定识别位姿扫描触发，主视觉为 2026-04-22 `PR-FPRG`。
- `MODE_EXECUTION_REFINE` 是执行层局部视觉，主视觉为平面分割 + Hough。
- 平面分割来自 `/Scepter/worldCoord/world_coord`：世界坐标处理层已经用 PCL `SAC_RANSAC` 去掉主平面，视觉层在非平面深度像素上做二值化、骨架化和 `HoughLinesP`。
- 执行层输出坐标仍从 `/Scepter/worldCoord/raw_world_coord` 取原始相机坐标，避免把平面过滤后的空洞坐标传给下游。
- 旧 `matrix_preprocess.py` 不直接重新绑定为 `pre_img()`；当前运行代码使用独立的 `execution_refine_hough.py`，只在 `MODE_EXECUTION_REFINE` 生效。

## 交接提醒

- 后续如果说“恢复 4 月 22 日视觉方案”，默认只指固定识别位姿扫描触发和高召回拓扑恢复，不代表执行层也改回 `PR-FPRG`。
- 后续如果说“执行层走平面分割 + Hough”，默认只指 `MODE_EXECUTION_REFINE`，不代表扫描建图也回到旧 Hough。
- 这两个模式必须继续拆开验证；如果看到执行层日志出现 `PR-FPRG`，优先检查 `/pointAI/process_image` 的 `request_mode` 分流。
