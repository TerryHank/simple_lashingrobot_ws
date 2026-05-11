# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-11 21:51:43，当前 HEAD：`3ba3e6d`。

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

- `2026-05-11 21:51 - 扫描模式失败当前帧立即返回`：2026-05-11：PointAI 扫描模式 MODE_SCAN_ONLY 在单次 /pointAI/process_image 请求内仍尊重用户设置的 stable_frame_count 释放帧数；但若当前帧 Surface-DP 没有返回有效点，不再循环等待下一帧，而是立即返回当前帧失败原因。Surface-DP 横纵线族不足错误文案改为中文“所选扫描底图横纵线族不足”，避免 completed surface 旧术语误导现场。
- `2026-05-11 21:35 - 相机SDK调试页独立中文化`：2026-05-11：设置页把“相机底层 SDK 调试”从视觉调试页拆成独立设置页选项 cameraSdkDebug；面板内参数显示全部中文化，dynamic_reconfigure 仍使用 Scepter ROS cfg 原始英文参数名向 /scepter_manager/set_parameters 下发，避免破坏相机 SDK 接口。
- `2026-05-11 21:19 - 3D显示未入组扫描点`：2026-05-11：3D Scene 规划点图层新增未入组扫描点显示。/api/planning/bind-path 返回的全量 grid_points 仍保留；黄色 bindPathPoints 只表示进入 pseudo_slam_bind_path.json 有效分组的可执行点，新增玫红色 unplannedBindPathPoints 表示扫描检测到但未进入任何规划组的点，悬停标签为“未入组扫描点”。行/列线、2x2成组线、跳绑高亮仍只使用已分组点，避免误导执行语义。
- `2026-05-11 21:17 - 扫描层单源底图与相机SDK热调`：2026-05-11：扫描层 Surface-DP 从固定 depth_gradient_only 改为 single_selected_response，可在前端 设置/视觉调试 的 扫描底图 下拉栏选择 fused_instance_response、frangi_like、hessian_ridge、depth_gradient、infrared_response、combined_response、depth_response；运行时只生成被选中的底图并做一次物理线族检测，不再使用 completed_candidates 第二轮。PointAI 订阅 /web/pointAI/set_scan_response_source 并持久化 ~scan_response_source。设置页新增相机底层 SDK 调试面板，按 Sceptertof_roscpp.cfg 参数通过 /scepter_manager/set_parameters dynamic_reconfigure/Reconfigure 热修改，并把参数保存到前端 localStorage 重连回放。
- `2026-05-11 21:07 - 扫描图像层收口为单图层`：2026-05-11：扫描视觉图像层只保留 /perception/lashing/scan_surface_dp_base_image，并在前端显示为“扫描识别底图”；移除 scan_surface_dp_completed_surface_image ROS publisher、前端 topic registry/image catalog 入口和当前静态页面选项。后端 publish_scan_surface_dp_base_images 只发布 runtime_response/depth_gradient 单图，继续叠加 DP 交点和梁筋候选诊断。
- `2026-05-11 20:58 - 扫描层接入可配置线性误差补偿`：2026-05-11：PointAI 扫描建图链路在 manual_workspace_s2 输出 PointCoords.World_coord 前新增可配置线性相机坐标补偿。默认 scan_linear_compensation_enabled=false 且 x/y 系数为 0，不改变现有扫描输出；启用后按 z 与 reference_z 的差值对相机坐标 x/y 做比例修正：x*=1-kx*(z-z0)，y*=1+ky*(z-z0)，并通过 min_z 与 max_abs_scale_delta 做门控/限幅。补偿只先接入固定识别位姿扫描账本点，不改底层 raw_world_coord 点云和执行微调 Hough 原始取点。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
