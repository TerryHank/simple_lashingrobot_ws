# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-15 07:50:37，当前 HEAD：`c99cd8d`。

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

- “视觉调试”新增“执行层视觉”选择，默认 `Hough微调`，可切到 `扫描同款Surface-DP`；设置会持久化，并热发布到 `/web/pointAI/set_execution_refine_algorithm`。
- 后端保持 `/pointAI/process_image request_mode=4` 与控制层调用口径不变，pointAI 内部按运行态参数分发到 Hough 或 Surface-DP 执行分支，非法值和旧缓存均回退 Hough。
- Surface-DP 执行分支复用扫描层 `run_manual_workspace_surface_dp_pipeline()` 的算法结果，但只发布执行层过滤后的点；输出继续套 TCP 执行盒、全局工作区过滤、TCP 蛇形排序和执行层分类过滤。
- 前端图像层把原“执行底图 Hough二值”改为“执行视觉底图”，用于显示当前执行层视觉算法的诊断底图，支持 Hough / Surface-DP 两套输出；执行 Surface-DP 使用扫描同款 `runtime_response` 作为原始响应底图，并投回原始相机画幅后发布，方便继续叠加执行点和分类标记。
- 开启分类后，执行层分类结果也会重新叠加到“执行视觉底图”：`BND` 表示已绑扎会被 blocking 过滤，`UNB` 表示未绑扎会继续下发，`UNC` 表示不确定；Hough 与 Surface-DP 执行分支共用这套分类诊断覆盖。
- 视觉调试页新增“账本微调 XY 阈值 (mm)”输入框，默认 80 mm；输入变化会保存到前端共享设置，并热发布到 `/web/cabin/set_ledger_refine_axis_threshold_mm`。
- `ledger_with_refine` / `live_visual` 的微调接纳门槛从写死的 `kPseudoSlamCheckerboardAxisThresholdMm=80mm` 改为运行时热更新值，用于视觉点按全局棋盘行 / 列中心归格时的 XY 轴向阈值。
- 该阈值只控制账本+微调的 XY 归格接纳；Z 不参与该接纳阈值。视觉点的 `world_z` 仍会随微调结果进入执行点，TCP 执行盒的 `X/Y/Z` 范围仍由“绑扎 X/Y/Z min/max”控制。

## Recent Session Memory

- `2026-05-15 07:50 - 区域切换进度语义修正`：前端区域切换的 AreaProgress 语义需要区分 ready_for_next_area: true 场景；当 ready_for_next_area 为真时，current_area_index 表示下一步要去的区域，AreaNavigationController 不能再按“当前所在区域+方向”二次偏移，否则会点击下个区域时跳过目标区。
- `2026-05-15 07:35 - 前端绑扎默认关闭并与 is_lashing 对齐`：前端控制开关的 lashingEnabled 初始值从 true 改为 false，默认显示为开启绑扎，和后端 PLC 的 is_lashing 关闭语义保持一致；补了前端单测锁定默认快照，避免后续回退。
- `2026-05-15 07:27 - 白盒分类接入中心块与凸包诊断`：将执行层白盒分类从仅靠斜向残差扩展为包含中心紧致块/凸包形状统计、离轴斜丝残差和轴向保护的联合判定。修复了普通十字交叉误判为已绑扎、中心块未被识别为已绑扎的问题，并把白盒融合结果接入模态扫图报告，新增 white_box_multimodal_rule 结果与中心块指标展示。
- `2026-05-15 07:10 - 执行层Surface-DP ROI对齐Hough`：2026-05-15：执行层扫描同款Surface-DP不再仅使用手动工作区rectified mask作为识别输入；run_execution_refine_surface_dp_pipeline 现在以 execution_refine_roi_mode=True 调用共用Surface-DP管线，prepare_manual_workspace_s2_inputs 会生成 Hough 同款 get_execution_refine_tcp_range_pixel_mask()，用 forward_h 投到 rectified 工作区后裁剪 workspace_mask_crop，从图像域入口对齐线性模组工具坐标系下的绑扎范围。扫描层默认不传该开关，仍使用手动工作区；legacy depth-only 才要求旧 period/phase 预估，Surface-DP 主链不会再被旧 S2 周期门提前拒绝。
- `2026-05-15 06:46 - 前端header单击清除单层报警`：2026-05-15：前端 header 的索驱、末端、视觉状态胶囊在对应层出现报警/异常 warn 时，短按改为只清除该层前端报警缓存并写日志，长按仍重启对应子系统；未上报/超时等普通 warn 仍保持启动语义。底层若继续通过 diagnostics/telemetry 上报异常，下一帧会重新点亮该层报警。
- `2026-05-15 06:43 - 帮助站前端全功能手册扩充配图并发布`：2026-05-15: 扩充 operator-manual 为全量图文手册, 新增顶部状态、控制面板、快速控制、账本流程、执行流程、图像、视觉调试、设置细分、遥控、3D 图层、日志终端等 15 张 SVG 配图, 并补齐相机 SDK 缺失的 ScepterSDK 入口页以通过构建; GitHub Pages 已发布到 gh-pages 分支。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
