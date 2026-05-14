# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-14 16:56:11，当前 HEAD：`8922c91`。

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

- 用户最新口径：保留“梁筋候选 / 梁筋过滤 / 过滤半径”功能，保留扫描底图红色梁筋候选竖带和前端开关；只取消“某个梁筋 mask 命中点后连带移除同 X 位置其它交点”的逻辑。
- Surface-DP 当前运行链路继续输出 `beam_candidate_*` 诊断，启用梁筋过滤时仍按前端半径扩张 `beam_candidate_margin_mask`，但最终过滤改为点级 `point_mask`：只删除自身落入梁筋 mask 的交点，不会连带移除同一 X 位置的其它普通钢筋交点。
- pointAI ROS 继续订阅 `/web/pointAI/set_scan_beam_exclusion` 与 `/web/pointAI/set_scan_beam_exclusion_margin_mm`；前端视觉调试页继续提供梁筋过滤开关、半径输入、topic registry 和 localStorage 字段。
- 运行态诊断保留 `beam_filtered_point_count`，`beam_filter_mode=point_mask` 表示点级过滤；当前入口不再使用列级过滤 helper，也不再输出列级过滤计数字段。

## Recent Session Memory

- `2026-05-14 16:56 - FINISHALL需观察末端真实运动后才放行`：2026-05-14：修复首区被秒判完成的问题。线性模组 execute_bind_points 的 wait_for_plc_finish_all 现在记录 EN_DISABLE 启动等待时的位置/速度，只有观察到 X/Y/Z 速度超过阈值或位置相对启动点变化后才接受 FINISHALL=1；若 FINISHALL 在未运动前置位，会清零并继续等待，最终按未确认完成失败阻止上层索驱进入下一区域。
- `2026-05-14 05:40 - 扫描完成后只刷新规划显示`：2026-05-14：去掉前端阈值热重规划后，扫描 action 成功时仍必须刷新前端规划组显示。TieRobotFrontApp 的 onSurfaceDpRecognitionFinished 现在只调用 requestPlanningAreaRefresh() 重新读取 /api/planning/bind-path，不调用 /cabin/replan_pseudo_slam_bind_path，也不做第二次规划；这样 pseudo_slam_bind_path.json 写完后 3D Scene 能看到新规划组。
- `2026-05-14 05:27 - 扫描阈值只随一次扫描规划生效`：2026-05-14：用户撤回前端阈值热重规划口径。成行/成列阈值仍由 StartPseudoSlamScan action goal 传入后端，并在 run_pseudo_slam_scan 的一次扫描规划中生成 pseudo_slam_points.json 和 pseudo_slam_bind_path.json；前端不再在阈值变化、扫描完成回调或确认工作区自动视觉完成后调用 /cabin/replan_pseudo_slam_bind_path。replan service 先保留为手动/诊断能力，不作为正常扫描链路第二次规划。
- `2026-05-14 05:03 - 扫描规划优先 2x2 四点分组`：2026-05-14：用户明确扫描账本点规划要优先考虑 2x2 分组。动态规划自适应分组候选形状现在先尝试 2x2，再考虑显式 requested_group_point_count 对应的大矩形和更小补组；默认 4 点与阈值重规划继续保持 slam/v35 固定 2x2 切块口径。只有显式非自适应请求 6/9 等点数时才优先按对应大组规划。
- `2026-05-14 04:52 - 阈值重规划保持 v35 默认 2x2 固定切块`：2026-05-14：修正扫描账本阈值重规划的默认 4 点成组口径。/cabin/replan_pseudo_slam_bind_path 为了让行/列阈值生效会设置 force_axis_threshold_grouping=true、忽略旧 global_row/global_col 并按世界 XY 阈值重聚类；但默认 bind_group_point_count=4 时仍必须和 slam/v35 一样从起点固定 2x2 切块，形成 (1,1)(1,2)(2,2)(2,1) 这类四点组，不走邻接匹配大量生成 matrix_2x2_edge_pair 短线。当前 planner 在 force_axis_threshold_grouping && requested_group_point_count==4 时改走 select_grid_group_candidates_by_fixed_two_by_two_tiling；Z 可达性仍只看 XY，真实 Z 保留。
- `2026-05-14 04:34 - 扫描账本成组可达性改为只看 XY`：2026-05-14：按现场口径，pseudo_slam_bind_path 动态成组的可达性不再用局部 Z 拦截；is_group_reachable_from_centered_dynamic_pose 只检查规划姿态下局部 X/Y 是否落在线模工作盒内，真实点位 Z 仍保留在 bind_points_world 和后续执行 JSON 中。这样扫描层散点只要 XY 可覆盖就能排成组，Z 不再导致规划阶段丢组。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
