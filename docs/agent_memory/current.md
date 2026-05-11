# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-12 01:48:59，当前 HEAD：`4ca8724`。

## Bootstrap Files

轻启动必读：

- `README.md`：项目总览、包边界、启动方式和主链路。
- `CHANGELOG.md`：近期项目级约定和关键调整。
- `docs/agent_memory/current.md`：当前共享记忆快照。

按需扩展读取：

- `docs/agent_memory/README.md`：维护共享记忆、写入账本或校验契约时读取。
- `docs/agent_memory/organism.md`：调整 agent 行为、工程协作协议或有机体口径时读取。
- `docs/agent_memory/codex_local_setup.md`：排查 Codex 启动目录或模型输入注入时读取。
- `docs/agent_memory/power_loss_recovery.md`：突然断电后的恢复协议。
- `docs/agent_memory/checkpoint.md`：最近一次可恢复现场。
- `docs/handoff`：只在当前任务命中对应专题时读取。

## Current High-Signal Memory

- 当前工程已经收口到 `tie_robot_msgs`、`tie_robot_hw`、`tie_robot_perception`、`tie_robot_control`、`tie_robot_process`、`tie_robot_web`、`tie_robot_bringup`、`tie_robot_description` 这 8 个主包。
- 新前端源码在 `src/tie_robot_web/frontend`，静态产物在 `src/tie_robot_web/web`。如果修改前端且影响静态页面，需要重新构建产物。
- 不要恢复控制面板按钮上方的任务提示框，除非用户明确要求。
- 本工程里的 Codex 按 `docs/agent_memory/organism.md`（Codex 工程有机体协议）运行：先感知、再行动、持续记忆、完成前自检。
- 轻启动原则：新会话默认先读 `README.md`、`CHANGELOG.md`、`docs/agent_memory/current.md`；其他记忆文档按需扩展读取，避免长上下文卡顿。
- 示例泛化原则：用户说“比如”“例如”“类似”时，示例不是需求边界；先识别真实目标和工程约束，再举一反三，避免机械照搬。
- 长上下文瘦身原则：优先读摘要和相关片段，不把整仓状态、大段日志、构建产物、`.debug_frames` 或无关历史全文塞入上下文；卡顿时写 checkpoint 并开新会话续接。
- Codex 会话超过 100MB 时使用 `python3 scripts/codex_session_guard.py summarize --threshold-mb 100 --skip-open --apply` 压缩活跃会话：原始完整 JSONL 内容复制到 `~/.codex/archived_sessions/oversized/`，`~/.codex/sessions` 里的会话文件不移动出原路径，只把原路径内容改写为摘要替身 JSONL；本机启用 `tie-codex-session-summary.timer` 自动执行该流程。
- Codex 摘要替身必须保留原始 `session_meta` 关键字段和首条真实用户请求作为标题锚点；已有摘要替身可用 `python3 scripts/codex_session_guard.py repair-summaries --apply` 从 archive 原文修复标题锚点。
- 长任务或高风险修改前后运行 `python3 scripts/agent_memory.py checkpoint ...`，断电后用 `python3 scripts/agent_memory.py recover` 读取恢复报告。
- 当前认可的手动工作区 S2 是 `PR-FPRG 透视展开频相回归网格方案`；不要退回原图 bbox 估周期、像素尺度 rectified 或 `pre_img()` 前置门控。
- 涉及 TF、点云或 3D Scene 时，先读 `CHANGELOG.md` 中关于相机、TCP 和前端显示坐标系的约定。

## Latest CHANGELOG Signals

- 用户最新口径：设置页不再展示独立的“工作区选点”和“扫描动作”卡片；“确认工作区域”跟在“移动到选中位姿”右侧，仍在设置 / 工作区的扫描位姿卡片内。
- “触发扫描视觉”重新放回控制面板“扫描区”，且扫描区只保留这个主动视觉触发入口；移动到位姿、记录识别位姿和确认工作区域不再作为控制面板任务按钮。
- 工作区点选列表与“撤销最后一点 / 清空重选”保留在扫描位姿卡片内，避免现场误点后无法修正。
- 当前工程状态已先保存并推送远端 tag `slam/v44`，作为本轮扫描线族语义修改前的恢复点。
- Surface-DP 扫描线族不再使用横纵线数接近 `1:1` 的强平衡门槛；小视野、正方形和长方形钢筋面统一走同一套物理网格评分。
- 新评分用候选横纵线数比例与当前 rectified 有效视野物理长宽比的一致性、线距物理先验、弱规则线召回和响应支持共同判断网格可信度；`34x21` 这类长方形全局网格可通过，正方形视野中的 `16x2` 线族假阳仍会被拒绝。
- 运行态诊断新增 `physical_lattice_score`、`physical_lattice_count_aspect`、`physical_lattice_visible_aspect`、`physical_lattice_count_aspect_error` 和 tolerance 字段；旧 `full_workspace / visible_local` 分档标签收口为 `unified_physical_lattice`，避免后续按视野大小重新分支。

## Recent Session Memory

- `2026-05-12 01:48 - 前端热调扫描线性补偿系数`：2026-05-12：视觉调试页新增扫描线性补偿控件，现场可直接增大/减小 X补偿(%/m)、Y补偿(%/m)、基准Z、生效Z和最大比例限幅。前端通过 /web/pointAI/set_scan_linear_compensation 发布 Float32MultiArray：[enabled, reference_z_mm, x_per_mm, y_per_mm, min_z_mm, max_abs_scale_delta]，其中 %/m 会除以 100000 转成后端 per-mm 系数；pointAI 订阅后热更新 scan_linear_compensation_* ROS 参数并影响后续扫描账本点。
- `2026-05-12 01:46 - Surface-DP 弱线补齐物理网格`：2026-05-12：用户反馈扫描调试图像网格效果好但绑扎点只覆盖一小片，怀疑门限导致。根因定位为 Surface-DP 物理线族选择先按强候选峰接受 9x16/10x16 等局部网格，导致后续交点只覆盖强响应区域。现在 _select_physical_lattice_positions 在候选峰确定可信物理间距后，会沿同一物理网格向两侧补齐有局部凸起证据的弱线，避免强局部网格提前收敛；纯缺线场景仍保留 9/10/11x16 可见网格兼容，不凭空铺满。
- `2026-05-12 01:00 - 视觉算法异常只让视觉按钮变黄`：2026-05-12：用户纠正前端告警口径：/diagnostics 中 tie_robot/visual_algorithm 的 DiagnosticStatus.ERROR（例如 Surface-DP失败：所选扫描底图横纵线族不足）表示视觉算法告警，但不要进入顶部连接报警横幅，也不要把视觉按钮文案改成‘视觉报警/重启’。前端应把原视觉状态按钮置为黄色 warn，保留原按钮文案/启动动作；详细错误只写入前端日志。
- `2026-05-12 00:54 - 扫描账本按识别位姿分组续传`：2026-05-12：扫描 action/service 新增 recognition_pose_index，前端从设置/工作区的当前扫描位姿下拉框传入序号。pseudo_slam_points.json 与 pseudo_slam_bind_path.json 写入 scan_pose_groups 和 scan_pose_groups_by_pose_index，重扫同一识别位姿只替换该位姿大组，其他位姿组保留；顶层 pseudo_slam_points/areas 仍保留为汇总扁平结构供执行层和前端兼容。每个点/区域带 pose_index、recognition_pose_index 和 pose_local_* 字段，执行记忆去重也包含 recognition_pose_index，避免不同位姿相同行列互相吞掉。
- `2026-05-12 00:48 - Surface-DP 继续放宽到 10x16 现场线族`：2026-05-12：用户要求继续放宽扫描底图线族门限。基于前一轮确认 11x16 通过后，新增 10x16 回归并验证当前实现仍失败，随后将 PHYSICAL_LATTICE_ASPECT_BASE_TOLERANCE 再从 0.40 提到 0.55。修改后 11x16、10x16 都通过，9x16 仍被 count_aspect_mismatch 拦住，2x16 仍失败。失败消息继续保留中文原因映射。
- `2026-05-12 00:47 - 视觉算法异常显示为报警态`：2026-05-12：前端状态胶囊语义修正。/diagnostics 中 tie_robot/visual_algorithm 的 DiagnosticStatus.ERROR（例如 Surface-DP失败：所选扫描底图横纵线族不足）表示视觉算法报警，不表示视觉节点关闭；前端应把 detail 纳入报警汇总，视觉胶囊显示‘视觉报警/重启’，只有 OK 运行态才显示‘关闭’动作，WARN 未上报/超时仍按启动语义处理。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
