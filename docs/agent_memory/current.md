# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-04-29 14:01:54，当前 HEAD：`368c31e`。

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
- Codex 历史会话打不开时，运行 `python3 scripts/codex_session_guard.py scan --threshold-mb 50`；确认后用 `archive --apply` 将超大 JSONL 移到 `~/.codex/archived_sessions/oversized`。本机可用 `scripts/install_codex_session_guard_timer.sh` 安装自动守卫。
- 长任务或高风险修改前后运行 `python3 scripts/agent_memory.py checkpoint ...`，断电后用 `python3 scripts/agent_memory.py recover` 读取恢复报告。
- 当前认可的手动工作区 S2 是 `PR-FPRG 透视展开频相回归网格方案`；不要退回原图 bbox 估周期、像素尺度 rectified 或 `pre_img()` 前置门控。
- 涉及 TF、点云或 3D Scene 时，先读 `CHANGELOG.md` 中关于相机、TCP 和前端显示坐标系的约定。

## Latest CHANGELOG Signals

- 新增 `docs/releases/slam_v30/`，作为 `slam/v30` 的交接、发布清单、校验文件和视觉模态样例目录。
- 新增 `src/tie_robot_perception/tools/export_visual_modalities_snapshot.py`，可从当前 ROS 运行态导出相机、世界坐标、TF、前端状态和 `pointAI/PR-FPRG` 结果到小型 bag、PNG、NPY 与 metadata。
- 新增 `src/tie_robot_bringup/launch/slam_v30_offline_visual_replay.launch`，用于在无机器环境下通过 `rosbag play --clock --loop` 复现当前视觉测试场景。
- `slam/v30` 样例 bag 已捕获 `/Scepter/*` 核心图像、`/Scepter/worldCoord/*`、`/pointAI/result_image_raw`、`/perception/lashing/result_image`、`/perception/lashing/points_camera`、`/perception/lashing/workspace/quad_pixels`、`/tf`、`/tf_static` 和吊篮/末端状态。
- 修复 `pointAINode` 运行态 `PR-FPRG` 方法绑定缺口，确保 `/perception/lashing/recognize_once` 可以真实跑通 `manual workspace S2`，而不是在服务调用时才暴露缺失属性。
- `suoquNode` 调用末端绑扎执行改为 `/moduan/execute_bind_points` action 客户端，取代旧 `/moduan/sg_precomputed*` service 调用路径，并补齐 `tie_robot_process` 的 `actionlib` 依赖。
- `pointAI` 的主视觉链路统一切到方向自适应 `PR-FPRG`：手动工作区透视展开后，不再假设钢筋一定与画面水平/垂直，而是在 rectified 平面内扫描两组 `theta/rho` 周期线族，再做峰值支撑、连续钢筋条验证、主间距兜底和线族求交。
- `PROCESS_IMAGE_MODE_SCAN_ONLY`、执行微调和绑扎检查等 `process_image` 入口都先运行 `run_manual_workspace_s2_pipeline(publish=True)`；旧 `pre_img()` 不再作为运行门控或回退路径。

## Recent Session Memory

- `2026-04-29 14:01 - PR-FPRG 3-6曲线避梁筋/地板缝探究报告`：按用户要求独立探究方案3/4/5/6避开梁筋和地板缝，不改运行时方案1主链。新增/增强 pr_fprg_scheme_comparison.py 的独立报告诊断：梁筋只做最终点级过滤，报告 beam_filtered_point_count；曲线抗地板缝以 curve_metrics 量化，包括 coverage_mean、score_mean、abs_offset_mean、abs_offset_p95、wiggle_mean。当前尺度报告 .debug_frames/pr_fprg_curve_3456_beam_floor_probe_20260429_135902，已发布到 /reports/pr_fprg_curve_3456。当前结果：3/4/5/6 均为 6x6 线族、31-32 点、beam_filtered=0；03 greedy 覆盖高但 wiggle_mean=1.148，容易追地板纹理；04 DP wiggle_mean=0.061 但偏移均值较大；05 DP+ridge wiggle_mean=0.067 且 abs_offset_mean=3.699，作为下一步主候选；06 IR-assisted wiggle_mean=0.067、score_mean较高但可能被红外地板缝牵引，作为辅助验证候选。
- `2026-04-29 13:59 - Xpra 主窗口无边框全屏从代理层 patch`：2026-04-29 验证：rviz/rqt 等图形卡片应通过 workspace_picker_web_server.py 的 _patch_xpra_html5_resource 修补 Xpra HTML5 /js/Window.js 与 /css/client.css，使主窗口标记 tie-robot-embedded-main-window、隐藏内部 windowhead、初始 maximized 铺满 iframe；不要在 UIController 中直接改 frame.contentWindow.client/id_to_window/canvas，否则容易导致蓝屏或刷新。
- `2026-04-29 13:55 - 图形应用嵌入窗口去边框补丁`：workspace_picker_web_server.py 会在代理 xpra HTML/JS 资源时 patch /js/Window.js 和 /css/client.css：普通 NORMAL 主窗口在前端图形应用卡片内强制最大化、隐藏 windowhead、去边框，DIALOG/UTILITY/TOOLTIP 等窗口不套用。
- `2026-04-29 13:55 - 线性模组执行抽象为状态Topic+Action`：用户已确认当前工程采用 ROS 风格分层但不强行引入 ros_control/MoveIt：moduan_driver_node 负责读 PLC 并发布 /moduan/state；moduan_motion_controller_node 提供 /moduan/execute_bind_points Action，内部写点位、发执行信号并等待 FINISHALL；/moduan/sg、/moduan/sg_precomputed、/moduan/single_move 继续作为兼容 wrapper；tie_robot_process 预计算绑扎点执行链现在通过 /moduan/execute_bind_points Action 调度，不再直接调用 /moduan/sg_precomputed* 或感知 PLC 完成位。FINISHALL 仍是 PLC 完成的权威信号，但只属于控制层实现细节。
- `2026-04-29 13:55 - suoquNode 改走 execute_bind_points action`：slam/v30 最终提交中，suoquNode 到线性模组绑扎执行的调度路径改为 actionlib SimpleActionClient 调用 /moduan/execute_bind_points，旧 /moduan/sg_precomputed 与 /moduan/sg_precomputed_fast service 调用不再作为流程编排主路径。
- `2026-04-29 13:47 - PR-FPRG 方案1恢复连续/ridge主链`：用户指出当前尺度效果奇差，根因定位为此前为追求方案1 only 和点数，把运行主链改成 enable_continuous_validation=False，退化为只靠一维 peak/spacing，导致地板缝和边缘细峰在当前尺度下爆出密集假线。已恢复方案1主链口径：depth_background_minus_filled 优先，use_orientation_prior_angle_pool=True，enable_local_peak_refine=True，enable_continuous_validation=True，enable_spacing_prune=True；梁筋仍只做最终点级排除，不删除整条 rho/线族。新增规则 lattice 收束用于 dense floor-seam 候选，避免 peak 候选过密。最新当前尺度报告 .debug_frames/pr_fprg_scheme1_restored_mainline_current_scale_20260429_134514，已发布到 /reports/pr_fprg_scheme1_current_scale、/reports/pr_fprg_scheme1、/reports/pr_fprg_live_full。注意：效果从 54 点密集假线收敛到 36 点，但耗时约 1.28s，仍需后续继续优化质量与速度，不能宣称最终完成。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
