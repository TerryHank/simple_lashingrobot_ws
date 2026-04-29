# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-04-29 13:47:42，当前 HEAD：`44b693b`。

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
- `pointAI` 的主视觉链路统一切到方向自适应 `PR-FPRG`：手动工作区透视展开后，不再假设钢筋一定与画面水平/垂直，而是在 rectified 平面内扫描两组 `theta/rho` 周期线族，再做峰值支撑、连续钢筋条验证、主间距兜底和线族求交。
- `PROCESS_IMAGE_MODE_SCAN_ONLY`、执行微调和绑扎检查等 `process_image` 入口都先运行 `run_manual_workspace_s2_pipeline(publish=True)`；旧 `pre_img()` 不再作为运行门控或回退路径。
- 旧 `RANSAC + Hough + pre_img` 绑扎点识别代码已归档到 `docs/archive/legacy_ransac_hough_pointai/`，active `pointai` package 中删除 `matrix_preprocess.py`，`processor.py` 不再绑定 `cls.pre_img`。

## Recent Session Memory

- `2026-04-29 13:47 - PR-FPRG 方案1恢复连续/ridge主链`：用户指出当前尺度效果奇差，根因定位为此前为追求方案1 only 和点数，把运行主链改成 enable_continuous_validation=False，退化为只靠一维 peak/spacing，导致地板缝和边缘细峰在当前尺度下爆出密集假线。已恢复方案1主链口径：depth_background_minus_filled 优先，use_orientation_prior_angle_pool=True，enable_local_peak_refine=True，enable_continuous_validation=True，enable_spacing_prune=True；梁筋仍只做最终点级排除，不删除整条 rho/线族。新增规则 lattice 收束用于 dense floor-seam 候选，避免 peak 候选过密。最新当前尺度报告 .debug_frames/pr_fprg_scheme1_restored_mainline_current_scale_20260429_134514，已发布到 /reports/pr_fprg_scheme1_current_scale、/reports/pr_fprg_scheme1、/reports/pr_fprg_live_full。注意：效果从 54 点密集假线收敛到 36 点，但耗时约 1.28s，仍需后续继续优化质量与速度，不能宣称最终完成。
- `2026-04-29 13:44 - Xpra 图形卡片不要触碰内部 client`：rqt/rviz 前端蓝屏和 Xpra 断连页的根因不是 Xpra 技术栈本身：同一 session 在最小 iframe 中可正常显示；完整前端失败来自 UIController 在 iframe 加载/resize 时进入 Xpra HTML5 内部触发 resize、redraw、request_refresh 并清内部 DOM 样式，容易打断握手或遮断绘制。后续图形卡片只管理外层卡片和 iframe 宽高，不再调用 frame.contentWindow.client、redraw_windows、request_refresh，也不再修改 Xpra 内部 window/canvas 样式。
- `2026-04-29 13:40 - slam/v30 离线复现发布包`：2026-04-29 当前 slam 分支发布为 slam/v30：新增 docs/releases/slam_v30 交接目录、slam_v30_visual_modalities.bag 小型视觉样例、PNG/NPY/modal metadata、debug_frames_manifest.tsv，以及 slam_v30_offline_visual_replay.launch。完整 .debug_frames 实验输出不进入 Git 历史，随整工程 zip 打包。
- `2026-04-29 12:33 - PR-FPRG 当前只启用方案1直线族`：用户已改为先只做方案1：当前视觉主链采用方案1 theta/rho 直线族，在当前尺度下进行钢筋绑扎点识别；方案2不要，曲线方案3-6暂存档只作为后续收束/改进参考，不进入当前主链或主报告。梁筋处理口径：钢筋可以穿过梁筋，不删除整条 rho/线族；梁筋只用于最终绑扎点的点级排除，IR 只辅助结构梁筋掩膜，主线响应锁定 depth_background_minus_filled。最新方案1报告为 .debug_frames/pr_fprg_scheme1_beam_excluded_20260429_1315/index.html，并已发布到 /reports/pr_fprg_scheme1、/reports/pr_fprg_robustness、/reports/pr_fprg_live_full。
- `2026-04-29 12:24 - PR-FPRG 梁筋只做最终点级过滤`：用户纠正：普通钢筋可以穿过梁筋，不能因为梁筋 mask 删除整条 rho 线或阻断线族；梁筋只用于最终绑扎点排除。当前 3-6 方案报告废弃 1/2，主线回到 08:21 风格：深度响应 depth_background_minus_filled 优先，红外只用于纵向梁筋结构 mask，profile/spacing 拓扑 + 曲线追踪；连续验证在当前现场图会漏真实线，只作为对照。最新报告 .debug_frames/pr_fprg_scheme_ablation_report_curve_depth_irbeam_cleanfull_20260429_1300，已发布到 /reports/pr_fprg_live_full 和 /reports/pr_fprg_robustness。
- `2026-04-29 12:10 - FINISHALL是线性模组执行完成权威信号`：用户澄清：发执行信号 -> 等 FINISHALL 这段由 PLC 执行，PLC 结束后给 FINISHALL，因此它本身可替代服务入口处人为置 /moduan_work 忙。当前实现已去掉 moduan 服务入口的 ScopedModuanWorkState；/moduan_work 只在 execute_bind_points 的实际 PLC 执行段镜像状态：点位写入完成后进入 ScopedPlcExecutionState，随后 pulseExecutionEnable 发执行信号并等待 finish_all(150)，返回/超时后自动发布 false。视觉识别阶段不再发布 moduan_work=true。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
