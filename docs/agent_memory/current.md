# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-11 22:53:17，当前 HEAD：`9eb95dd`。

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

- 用户最新口径：新前端 header 的“演示模式”点击后只关闭当前 `simple_lashingrobot_ws` 工程相关进程和后台服务，然后按钮状态变绿；不再启动、停止或清理 `/home/hyq-/lashingrobotROS` 或 `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws` 里的任何内容。
- 演示模式进入动作收口为停止本工程 `tie-robot-backend.service`、三个 driver service 与 `tie-robot-rosbridge.service`，并且只按当前工作区路径清理残留 ROS 进程；不再启动旧前端、旧 `chassis_ctrl api.launch`、`tie-robot-demo-rosbridge.service` 或 `tie-robot-demo-show-full.service`。
- 演示模式退出动作仍按当前工程依赖顺序恢复 `tie-robot-rosbridge.service`、三个 driver service 和 `tie-robot-backend.service`。

## Recent Session Memory

- `2026-05-11 22:53 - 扫描梁筋过滤默认改为15cm`：2026-05-11：用户现场确认同列缺点符合扫描梁筋过滤路径，当前扫描 Surface-DP 在启用 scan_beam_exclusion 时从梁筋候选 band 扩张物理 mask 后过滤最终交点。运行态默认 scan_beam_exclusion_margin_mm 已从 130.0 mm 改为 150.0 mm；诊断键同步为 beam_candidate_15cm_mask / beam_candidate_15cm_pixels，前端视觉调试开关和 ROS 日志文案显示「梁筋 ±15 cm 过滤」。旧 PR-FPRG 研究工具中的 13cm 命名属于历史实验口径，未作为当前运行态入口修改。
- `2026-05-11 22:51 - Surface-DP统一物理网格评分`：2026-05-11：扫描层 Surface-DP 不再用横纵线数接近 1:1 的 full_workspace 平衡硬门槛过滤线族；改为统一物理网格评分，用候选线数比例与当前 rectified 有效视野物理长宽比的一致性、弱规则线召回和支持度共同接纳/拒绝。长方形全局网格如 34x21 可通过，正方形视野里的 16x2 假阳仍拒绝；小视野、正方形、长方形共用同一判据，不按视野大小分档。当前工程状态已先保存并推送 tag slam/v44。
- `2026-05-11 22:50 - 扫描线族弱响应门限放宽`：2026-05-11：Surface-DP 扫描层在所选单一底图上放宽物理线族弱峰兜底门限：单轴选峰 fallback 从 0.08 放到 0.06，整图物理线族重试增加 0.08/0.06 档；线族评分略降低平均响应权重，提高线数完整度和画幅覆盖权重，便于现场检测弱但规律的横纵线族。仍保留必须同时存在横向线族和纵向线族、且数量比例符合画幅物理宽高的约束，不把单轴结果硬凑成交点。
- `2026-05-11 21:51 - 扫描模式失败当前帧立即返回`：2026-05-11：PointAI 扫描模式 MODE_SCAN_ONLY 在单次 /pointAI/process_image 请求内仍尊重用户设置的 stable_frame_count 释放帧数；但若当前帧 Surface-DP 没有返回有效点，不再循环等待下一帧，而是立即返回当前帧失败原因。Surface-DP 横纵线族不足错误文案改为中文“所选扫描底图横纵线族不足”，避免 completed surface 旧术语误导现场。
- `2026-05-11 21:35 - 相机SDK调试页独立中文化`：2026-05-11：设置页把“相机底层 SDK 调试”从视觉调试页拆成独立设置页选项 cameraSdkDebug；面板内参数显示全部中文化，dynamic_reconfigure 仍使用 Scepter ROS cfg 原始英文参数名向 /scepter_manager/set_parameters 下发，避免破坏相机 SDK 接口。
- `2026-05-11 21:19 - 3D显示未入组扫描点`：2026-05-11：3D Scene 规划点图层新增未入组扫描点显示。/api/planning/bind-path 返回的全量 grid_points 仍保留；黄色 bindPathPoints 只表示进入 pseudo_slam_bind_path.json 有效分组的可执行点，新增玫红色 unplannedBindPathPoints 表示扫描检测到但未进入任何规划组的点，悬停标签为“未入组扫描点”。行/列线、2x2成组线、跳绑高亮仍只使用已分组点，避免误导执行语义。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
