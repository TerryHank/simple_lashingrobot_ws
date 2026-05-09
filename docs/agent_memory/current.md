# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-09 17:25:37，当前 HEAD：`da94e23`。

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

- `2026-05-09 17:25 - 3D执行点只显示已分组点`：2026-05-09：前端 /api/planning/bind-path 会把 pseudo_slam_points.json 的全量 grid_points 附到 bind_path 上，但 3D Scene 的执行/规划点、行列线、跳绑高亮应只使用 pseudo_slam_bind_path.json 中有效分组（至少2点）引用到的 global_idx；未进入分组的扫描网格点不能显示成黄色单点，避免误以为产生了单点执行组。当前账本验证：176个grid_points中112个进入2/4点分组，64个未分组点会被隐藏。
- `2026-05-09 16:40 - 扫描物理先验取消钢筋数量限制`：2026-05-09：扫描层 Surface-DP 物理先验只保留钢筋间距约束（FULL_SCAN_REBAR_SPACING_MM_RANGE=120-160mm），不再使用 full workspace 15-18 条或固定 16 条偏好作为钢筋数量限制。线族数量上限改为当前视野按最小合法间距可容纳的数量，并按实际峰值与间距一致性选择；13x13 等合法间距网格应通过。梁筋候选、梁筋过滤开关和 lattice gate 语义本次不改。
- `2026-05-09 15:56 - PointAI扫描编号按map最小坐标起排`：2026-05-09：PointAI manual_workspace_s2 扫描点返回和结果图显示编号不再按右上角/TCP图像口径排序；每个点优先用 Scepter_depth_frame->map 转换后的坐标排序，按世界 Y 行、世界 X 正向编号，取不到 TF 时回退到原相机坐标。Surface-DP global_row/global_col 元数据继续保留原拓扑语义。
- `2026-05-09 15:52 - 扫描点按世界最小点重新编号`：2026-05-09：扫描代表点在写入 pseudo_slam_points.json、发布 pseudo_slam markers 和进入动态绑扎规划前，会先按 map/world 坐标排序并重新赋 idx/global_idx。排序先按世界 Y 聚行，再在每行按世界 X 正向排列；因此世界坐标最小角点成为 1 号点，不再沿用 PointAI 图像行列或视觉返回顺序。
- `2026-05-09 15:46 - 3D点悬停高亮反馈`：2026-05-09：新前端 3D Scene 的点悬停反馈已改为“点自身高亮”，不再叠加额外覆盖点。鼠标悬停绑扎点、路径规划点、跳绑覆盖点或索驱规划区域中心时，Scene3DView 通过同一 raycaster 命中管线设置该 THREE.Points 几何里的 pointHoverScale / pointHoverColorMix 顶点属性，由 PointsMaterial shader 让被命中的原始点自己变大变亮；移出或未命中时把该点属性恢复。
- `2026-05-09 15:26 - 演示模式只管本工程服务`：2026-05-09：用户明确要求新前端 header 的“演示模式”只关闭当前 /home/hyq-/simple_lashingrobot_ws 工程相关进程和后台服务，进入后按钮变绿；不得再启动、停止、清理或跳转 /home/hyq-/lashingrobotROS 或 /home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws 的任何内容。当前实现把演示模式收口为停止本工程 tie-robot-backend.service、tie-robot-rosbridge.service 与三个 driver service，只按当前工作区路径清理残留 ROS 进程；退出时按当前工程依赖顺序恢复 rosbridge、三个 driver 和 backend。旧 demo rosbridge/show_full unit 与 launch 模板已删除，frontend autostart 不再安装旧前端或 demo 模式服务。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
