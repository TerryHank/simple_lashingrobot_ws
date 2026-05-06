# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-07 04:00:20，当前 HEAD：`cbd5b3a`。

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

- `/moduan/sg` 单点绑扎在执行微调 Hough 返回一个区域的多个点后，会先把相机点转换到 `gripper_frame`，再按 TCP 局部 `x` 分行、`y` 交替方向蛇形排序后下发线性模组，确保绑扎枪按区域内蛇形点序移动。
- `MODE_EXECUTION_REFINE` 返回给服务响应和执行底图编号的点序同步改为同一 TCP 蛇形口径：从 TCP 局部 `x` 最小行开始，偶数行 `y` 小到大，奇数行 `y` 大到小；扫描建图点序不随本次修改改变。
- 现场口径固定：绑扎点从识别到下发只走一层执行范围校验，即 pointAI 的视觉 / TCP ROI 校验；控制层和预生成组加载层不再使用旧 `X[0,360] / Y[0,320] / Z[0,160]` 或类似硬行程二次拒绝点位。
- `tie_robot_control` 已删除 `execute_bind_points(...)` 内的 `is_valid_precomputed_tcp_travel_point(...)` 过滤和相关 `kTravelMax*` 常量，视觉已经选中的点不会再因控制层旧边界被丢弃；手动 `/moduan/move` 入口也不再复用这组旧硬范围拦截。
- `tie_robot_process` 读取 `pseudo_slam_bind_path.json` 的预生成局部点时只校验字段存在，不再按虎口范围二次过滤；路径生成阶段仍可使用当前 `380 / 330 / 160 mm` 的规划参数形成可执行分组。
- pointAI 和前端 TCP 线模遥控的默认范围显示统一到 `X[0,380] / Y[0,330] / Z[0,160] mm`，避免 UI 或视觉显示继续暴露旧 `360 / 320 / 140` 口径。
- “显示与视角 / 图层设置”新增“绑扎范围”开关，单独控制三维场景中 gripper_frame 下的线性模组绑扎范围实体显示/隐藏；该开关随 topic layer state 持久化，不改变视觉调试页的范围数值，也不影响 pointAI ROI 下发。
- 绑扎范围实体不再被“机器”图层硬性联动隐藏：只要该图层开关打开且 `gripper_frame` TF 可用，即使关闭机器模型，也可以单独查看绑扎范围长方体。

## Recent Session Memory

- `2026-05-07 04:00 - 连接徽标长按重启中动画`：前端顶部“连接成功”徽标长按触发 restartRosStack 后，现在会像索驱、末端、视觉状态胶囊一样进入 pending：禁用按钮、显示“重启中”、保留连接成功主标签并启用旋转/滑入动画；连接状态刷新不会打断该 pending 反馈，完成后恢复“长按重启”。
- `2026-05-07 03:56 - 当前视野边界改为IR底层SVG`：用户要求当前可视区域边界不要通过 overlay canvas 重绘，避免高频 clearRect/stroke 消耗。前端图像层现在在 irCanvas 和 overlayCanvas 之间新增 irBaseBoundaryLayer SVG，实时视野边界只更新 SVG polygon/circle DOM 属性；overlayCanvas 继续只承载算法结果、线模范围、识别点和悬停读数。
- `2026-05-07 03:53 - 长按暂停恢复回起点跨进程中止`：修复长按暂停/恢复作业未可靠清空当前末端任务并回执行起点的问题：末端 driver 与 motion_controller 已拆成两个进程，长按 /web/moduan/hand_sovle_warn data=2 必须同时让 driver 先写 IS_STOP=1 停止当前线性模组队列，并让 motion_controller 收到同一长按信号后置 moduan_return_zero_ordered_requested，使 ExecuteBindPoints Action/FINISHALL 等待中止；随后 /moduan/return_zero_ordered 才能接管 Z 优先回零和索驱回执行起点。
- `2026-05-07 03:46 - 当前视野边界不是扫描工作区`：纠正图像绿色边界语义：用户要看的不是保存给扫描层识别位姿的手动工作区，而是索驱当前位置下相机当前帧实际可见物理区域边界。前端绿色边界应由 /Scepter/worldCoord/raw_world_coord 当前帧有效 3D 像素边界生成，经当前 TF 固定到 map 后再用 IR camera_info 投回图像；/perception/lashing/workspace/quad_camera_points 只代表保存的扫描工作区角点，不应用作当前视野边界。
- `2026-05-07 03:35 - 线模范围与工作区投影区分`：修正 2026-05-07 03:14 的误写口径：图像层不要关闭线性模组/TCP 工作范围蓝色投影，用户要关闭的是旧的工作区蓝色像素框。动态图像工作区投影应使用 /perception/lashing/workspace/quad_camera_points 的相机角点，前端按 TF 固定到 map 后再投回 IR 图像，并用非蓝色样式与线模范围区分；若看不到投影，优先检查 quad_camera_points、IR camera_info 和 TF 三者是否同时到达。
- `2026-05-07 03:14 - 图像工作区投影口径收口`：前端工作区显示口径收口：不要再把实时工作区作为 3D Scene 蓝色框显示，也不要在图像层恢复旧的蓝色已保存工作区框或 TCP 范围投影框。实时工作区应先按 TF 固定到 map 世界坐标，再用当前 IR 相机内参和 TF 动态投回图像；手工选区折线只在工作区选点模式显示。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
