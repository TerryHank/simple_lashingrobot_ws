# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-05 04:45:10，当前 HEAD：`9c04a06`。

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

- 前端 3D 橙色 TCP 方块不再只贴在静态 `gripper_frame` 外参原点；现在把 `/moduan/moduan_gesture_data` 的线性模组当前位置叠加到 `gripper_frame` 后显示实际移动 TCP 位置，底部线模全局坐标与 3D 方块共用同一换算。
- 执行微调结果图中的 `tcp=(...)` 改为当前运动 TCP/虎口坐标：先按 `Scepter_depth_frame -> gripper_frame` 得到线模绝对目标坐标，再减去当前线性模组 X/Y/Z，使越靠近当前虎口的点数值越接近 0；执行层写 PLC 的点位仍使用绝对线模目标坐标，不受显示相对坐标影响。
- `pointAI` 新增订阅 `/moduan/moduan_gesture_data` 缓存当前线模位置；若线模状态暂未到达，显示转换会自然退回只基于静态相机-TCP外参的旧口径。
- 前端图像层“执行底图 Hough二值”对应的 `/perception/lashing/execution_refine_base_image` 在 Hough 输出点生成后会重新发布带点位叠加的 `bgr8` 调试图：白/黑二值底图不变，识别出的执行点以黄色圆圈、红色中心和编号标出。
- 视觉图像层不再使用固定像素矩形 ROI：移除 `point1/point2` 白框过滤、执行 Hough 的 `roi_reject` 门和执行范围 mask 对静态 ROI 的叠加；候选点只受有效 3D 坐标、近点去重、手动/规划工作区和执行范围约束。
- 执行层视觉微调恢复独立的 TCP 遮挡黑色 mask：仅 `MODE_EXECUTION_REFINE` 会在 Hough 二值化前把已知 TCP 遮挡矩形 `(160,0)-(523,80)` 置黑，不作为点位 ROI 过滤，也不产生 `ROI` 拒绝诊断。
- 执行层视觉微调的 ROI 改为 TCP 坐标执行盒，而不是像素矩形：Hough 二值化前会把 raw world 像素按 `Scepter_depth_frame -> gripper_frame` 外参批量转换，只保留 `x[0,380] / y[0,3330] / z[0,3160]mm` 内的像素和候选点，范围外视图不再参与 Hough。
- “执行底图 Hough二值”进一步叠加诊断标记：`H` 为 Hough 原始交点，`ZERO` 为取不到有效 3D 坐标，`OUT` 为 TCP 执行范围外，`DUP` 为近点去重移除，`SEL/编号` 为最终输出点；现场漏点时可直接从同一图层判断掉在哪道门。

## Recent Session Memory

- `2026-05-05 04:44 - 执行微调改用TCP坐标执行盒ROI`：执行层视觉微调不再用像素矩形或扫描工作区作为Hough ROI；MODE_EXECUTION_REFINE会把raw_world像素按Scepter_depth_frame->gripper_frame外参批量转换，只保留TCP执行盒x[0,380]/y[0,3330]/z[0,3160]mm内的像素和候选点，范围外视图不参与Hough。TCP遮挡黑色mask仍只负责遮挡置黑，不是ROI。
- `2026-05-05 04:43 - 当前画面视觉先于索驱状态账本守卫`：2026-05-05 修复确认工作区/触发视觉识别不启动扫描层视觉的问题：前端runSavedS2走/web/cabin/start_pseudo_slam_scan scan_strategy=3，后端kCurrentFrameNoMotion分支必须先调用/pointAI/process_image request_mode=3触发Surface-DP扫描视觉，再检查索驱状态是否新鲜/静止/有效来决定能否覆盖pseudo_slam_points.json和pseudo_slam_bind_path.json；索驱状态异常时返回'已触发扫描层视觉，但未覆盖本地绑扎点文件'。
- `2026-05-05 04:33 - 扫描分组修复重复格点相邻空格漏组`：2026-05-05 现场图中可分成2x2的一组散点被漏掉，根因是扫描棋盘格身份在边缘出现重复格点和相邻空格：同一global_row/global_col下有两个物理相邻点，旁边真实格为空，动态规划按格占用选择后会漏掉可成组点。已在pseudo_slam扫描棋盘格身份构建阶段把重复格点按物理坐标迁移到有邻居支撑的相邻空格，并在dynamic_bind_planning里增加同类兜底修复旧产物；新增C++回归RecoversPhysicallyAdjacentLeftoversFromDuplicateGridCellGap覆盖该模式。
- `2026-05-05 04:28 - 执行层恢复TCP遮挡黑色mask`：在保持 pointAI 全局无固定像素 ROI 的前提下，执行层视觉微调恢复独立 TCP 遮挡黑色 mask：仅 MODE_EXECUTION_REFINE 在 Hough 二值化前把 tcp_occlusion_mask_rect=(160,0,523,80) 区域置零；它不参与候选点 ROI 过滤，也不恢复 roi_reject/ROI 诊断。扫描层不应用该遮挡。
- `2026-05-05 04:21 - 视觉图像层移除固定像素ROI`：pointAI 图像层不再使用 point1/point2 静态像素矩形 ROI：删除 get_roi_pixel_mask/is_point_in_roi 绑定、执行 Hough 的 roi_reject 过滤和诊断、执行范围 mask 对静态 ROI 的叠加，并禁用执行层顶部固定像素遮挡。候选点仍会经过有效 3D 坐标、近点去重、手动/规划工作区和执行范围过滤。执行底图 Hough二值诊断标记现在为 H/ZERO/OUT/DUP/SEL。
- `2026-05-05 03:52 - 执行 Hough 二值图诊断标记`：执行底图 Hough二值图现在会在最终输出点之外叠加 Hough 原始交点和 ROI/ZERO/执行框外/近点去重移除标记：H=Hough raw，ROI=ROI拒绝，ZERO=无有效3D坐标，OUT=执行微调框或工作区外，DUP=近点去重移除，SEL/编号=最终输出点。现场漏点时优先看此图层判断候选点掉在哪一道过滤门。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
