# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-07 11:09:56，当前 HEAD：`a861d93`。

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

- 前端控制面板下线“清除识别结果”“固定扫描规划”和“账本测试”三个旧按钮及其入口逻辑，任务按钮按“扫描区”“执行层”“区域切换”重新分组展示。
- “开始执行层”改名为“执行全局绑扎”，“执行层视觉单侧”改名为“单点视觉测试”；扫描区保留“移动到位姿”“设为识别位姿”“确认工作区域”“触发扫描视觉”，执行层保留“执行全局绑扎”“触发单点绑扎”“单点视觉测试”“记忆续跑开始”。
- 新增“上一个区域 / 下一个区域”人工切区：前端先发布 `/web/cabin/manual_area_takeover` 终止当前自动执行链并让线性模组归零，再按当前区域进度或当前位置邻近区域移动索驱到相邻 `cabin_pose`；暂停后未恢复时，后续区域交由人工操作。
- 清理执行视觉链路遗留的“近点排斥/去重”算法：`MODE_EXECUTION_REFINE` 的 Hough 候选点不再因为世界 XY 距离小于旧 `100mm` 阈值而成对丢弃；多根钢筋靠得近时，只要有有效 3D 坐标且落在 TCP 执行范围内，就继续进入排序和下发。
- 执行底图诊断同步移除 `DUP` 标记，日志也不再输出“去重移除”；现场漏点只剩 `H` 原始交点、`ZERO` 无有效 3D 坐标、`OUT` 超出 TCP 范围和 `SEL/编号` 最终输出这几类有效门控。
- Surface-DP 运行态新增 `beam_candidate` 梁筋候选诊断：基于收束底图中的宽、连续、高响应竖向 band 识别梁筋候选，并输出 `beam_candidate_bands`、`beam_candidate_count` 和像素统计。
- `/perception/lashing/scan_surface_dp_base_image` 与 `/perception/lashing/scan_surface_dp_completed_surface_image` 会用红色半透明竖带叠加梁筋候选，同时保留黄色 DP 交点；视觉调试设置里可选择启用「梁筋 ±13 cm 过滤」，默认关闭，启用后只过滤落入梁筋候选扩张范围的最终绑扎点，不删除普通钢筋线族。
- 梁筋候选识别补充「黑色竖沟 + 双侧窄亮边」形态：现场截图中梁筋中间常表现为贯穿全高的暗沟，而不是整条宽亮带；检测逻辑会把两侧连续亮边与中间低覆盖暗沟合并成一条 beam_candidate 竖带，避免漏掉这种梁筋。

## Recent Session Memory

- `2026-05-07 11:09 - 非4点分组统一为2x3物理方向`：动态绑扎规划的非4点模式已按现场口径收口：默认只把3个点跨度放在线模长轴方向，短轴最多2点；不再在2x3不可用时自动切到3x2，也不再让9点请求生成3x3。请求9时按世界坐标蛇形优先使用2x3，再用2x2、1x3、1x2等更小可达矩形补剩余点，所有候选仍按规划cabin_z校验线模工作范围。
- `2026-05-07 11:07 - Surface-DP 梁筋禁入前移到曲线追踪`：用户明确要求不要改默认开关，只把 beam mask 前移到 tracing 阶段。当前 scan_surface_dp 在启用梁筋±13cm过滤时，会先生成 beam_candidate_13cm_mask 作为 curve_trace_mask 禁区，再跑 curved_families；workspace_s2 曲线追踪只保留 support_mask 内的 polyline_points，并新增 polyline_segments，交点只在连续有效段之间求，避免曲线穿过梁筋后再靠最终删点兜底。
- `2026-05-07 10:58 - 动态分组起点兜底优先于后续完整组`：动态绑扎规划在请求 6/9 等多点分组时，不能先执行后续可达完整组再回头补世界最小起点附近的小组；完整候选和可达兜底候选需要进入同一条世界坐标蛇形队列，同一起点优先点数更多的组。这样 9 点组因规划高度或线模范围不可达时，会先在世界最小角附近落到最大可达小组，再继续蛇形填充后续区域。
- `2026-05-07 10:55 - 旧idx跳绑过滤已移除`：末端控制层旧跳绑链路已删除：不再使用 send_odd_points、/web/moduan/send_odd_points、ExecuteBindPointsTask.apply_jump_bind_filter 或 should_keep_jump_bind_point(idx==1/4) 托底过滤。execute_bind_points 现在只执行上游传入的点；跳绑选择保留在流程层，按 scan/bind path 中的 jump_bind、checkerboard_color、checkerboard_parity 元数据和 /web/moduan/jump_bind_enabled、/web/moduan/jump_bind_parity 决定。以后不要把旧 idx==1||4 过滤恢复到 moduan 层。
- `2026-05-07 10:22 - 控制面板任务区收口与人工切区接管`：前端控制面板已下线“清除识别结果”“固定扫描规划”“账本测试”旧入口，改为扫描区、执行层、区域切换三组；上一个/下一个区域会发布 /web/cabin/manual_area_takeover，先让当前自动执行链放弃后续区域并让线性模组归零，再按当前区域进度或当前位置邻近区域移动索驱到相邻 cabin_pose。
- `2026-05-07 10:14 - 索驱设备运动中不再触发自动跳区`：自动执行下发索驱位姿时，设备运动中/status_word=0x00000004 属于索驱忙的暂态，应等待并重试当前目标；Z超正限位、速度错误等真实拒绝仍保持硬失败，不能被通信恢复逻辑吞掉。等待轴到位时缓存索驱协议异常需记录 command_word，只有 TCP 运动类指令(0x0010/0x0011/0x0012)返回纯 bit2 设备运动中才按暂态处理。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
