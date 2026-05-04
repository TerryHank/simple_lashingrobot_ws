# Agent Memory Current Snapshot

> 由 `scripts/agent_memory.py refresh` 生成。刷新时间：2026-05-05 00:07:09，当前 HEAD：`04a5dc1`。

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

- 用户明确现场钢筋间距为 12-16 cm，钢筋网规格约为 `(15-18) * (15-18)`；扫描层全场识别不再以旧 8x8/64 点作为默认目标。
- `scan_surface_dp` 的运行态主链新增物理先验选线：根据 `rectified_geometry.resolution_mm_per_px` 将 12-16 cm 换算成像素间距，当前 5 mm/px 下为 24-32 px；当 rectified 视野足以容纳全场网格时，每轴优先选择 15-18 根线，当前现场帧输出 16x16=256 个候选绑扎点。
- 小视野不会被强行套全场线数：当画面尺度或工作区只容纳少量可见钢筋时，主链切换到 `visible_local`，按当前可见 2-18 根线输出局部交点；它只能识别当前画面里的可见交点，不能从 2-3 根钢筋直接推断完整全场网格。
- 修复全场服务触发仍返回 64 点的问题：运行态不再把旧 8x8 线族作为全场最终输出兜底；当 `fused_instance_response` 单轴响应不足时，会在物理先验下从 `Frangi / Hessian / depth_gradient / IR / combined` 等底图中选择能恢复 15-18 根线的线族。
- 清理旧版本残留：扫描主链 Surface-DP 失败时不再自动回退到 2026-04-22 depth-only S2；`workspace_s2` 的 8 根线/64 点偏置改为显式 `LEGACY_*` 命名，Surface-DP 不再调用 legacy axis-aligned 线族作为补全面支撑；前端视觉触发、内部 overlay 命名和项目关系图文案改为 Surface-DP 物理先验，相机原始绑扎点 TF child 前缀改为 `surface_dp_bind_point_*`，并清理旧 hash 静态资源。
- 新增 `test_scan_surface_dp_runtime.py` 覆盖全场物理先验 16x16 和小视野 2x3 两种行为；现场运行态报告挂载到 `/reports/live_surface_dp_physical_runtime_20260504_232729/`。
- 用户最新口径：扫描层继续推进 `combined / fused_instance_response -> Hessian + Frangi 脊线增强 -> binary candidate + skeleton -> completed_surface_mask -> DP 曲线沿局部 ridge 收束 -> 曲线交点输出 -> instance_graph junction 只做验证 / 补召回`。
- `MODE_SCAN_ONLY` 的触发链路仍保持 `/pointAI/process_image request_mode=3`、现有发布话题和 `PointsArray` 输出结构；算法本体从 2026-05-03 的 depth-only S2 主链切到 Surface-DP 主链。

## Recent Session Memory

- `2026-05-05 00:05 - 补清扫描层旧命名与TF残留`：2026-05-05：继续清理扫描层旧版本残留。运行态 TF child prefix 从 pr_fprg_bind_point 改为 surface_dp_bind_point；前端源码内部 prFprgOverlay/triggerPrFprg 命名改为 surfaceDpOverlay/triggerSurfaceDp；重新 npm run build 并删除未引用旧 hash app 资产。残留扫描限定 active pointai、workspace_s2、frontend src/test 和 web app 产物，未再命中 PR-FPRG/prFprg/pr_fprg_bind_point。重启 pointAINode 后 /pointAI/process_image request_mode=3 返回 count=256，tf_echo 可查 surface_dp_bind_point_1，旧 pr_fprg_bind_point_1 不存在。
- `2026-05-04 23:59 - 清理扫描层旧版本残留`：2026-05-04：用户确认清理旧版本残留。扫描主链 run_manual_workspace_s2_pipeline 现在只调用 Surface-DP，失败时直接返回失败并标记 legacy_depth_only_fallback=False，不再自动回退 run_manual_workspace_s2_depth_only_pipeline；scan_surface_dp 删除 legacy axis-aligned 线族兜底，物理先验无法解析时不再用旧 8x8 线族补 completed_surface；workspace_s2 的 8 根线/64 点评分偏置改为 LEGACY_* 常量，只保留给旧工具/测试；前端视觉触发和 project graph 文案改为 Surface-DP 物理先验，并清理未引用的旧 hash 静态资源。重启 pointAINode 后 /pointAI/process_image request_mode=3 返回 count=256，日志 lines=[16,16], points=256。
- `2026-05-04 23:47 - 前端 header 长按重启动画`：2026-05-04：新前端 header 中索驱、末端、视觉三个状态胶囊长按重启时会先进入 is-long-press-charging 充能态，持续约 0.8 秒；按钮内部使用当前状态色做横向填充和扫光动画，计时满后移除充能态并触发对应 restart*Subsystem。松手、滑出或取消 pointer 会清除充能态并保留短按动作。
- `2026-05-04 23:45 - 修复视觉触发回退 64 点`：2026-05-04 23:45：用户反馈点击前端触发视觉识别仍只有 64 点。系统化排查确认前端按钮调用 /pointAI/process_image request_mode=3，服务稳定返回 count=64，pointAINode 日志为 Surface-DP lines=[8,8]；独立复刻同一工作区和实时帧可输出 256。根因是运行态 Surface-DP 只用 fused/completed 响应做最终物理选线，当 fused 纵向响应不足时物理选线失败，代码又把旧 workspace_s2 8x8 线族作为最终兜底。已改为多底图物理选线：completed/fused/Frangi/Hessian/depth_gradient/IR/combined/depth 中选择能满足 12-16 cm、15-18 根线的物理线族；全场模式不再允许旧 8x8 作为最终输出。重启 pointAINode 后 /pointAI/process_image request_mode=3 返回 count=256，日志 lines=[16,16], points=256, mean_surface=0.974。
- `2026-05-04 23:41 - 前端 header 子系统短按/长按口径`：2026-05-04：新前端 header 中索驱、末端、视觉三个状态胶囊采用短按/长按双语义。状态只决定在线/离线颜色和短按动作：在线 success 短按关闭对应子系统，离线或非 success 短按启动对应子系统；长按约 0.8 秒统一重启对应子系统。索驱/末端的 start/stop/restart 只控制各自驱动守护，不联动视觉算法；视觉 start 为 startCameraDriver + startAlgorithmStack，stop 为 stopAlgorithmStack + stopCameraDriver，restart 为 restartCameraDriver + restartAlgorithmStack。
- `2026-05-04 23:28 - 扫描层主链接入物理间距先验`：2026-05-04：按用户要求将 12-16 cm 钢筋间距、(15-18)*(15-18) 规格接入运行态 scan_surface_dp 主链，替代旧 8x8 目标偏置。scan_surface_dp 现在按 rectified_geometry.resolution_mm_per_px 将 120-160 mm 转为像素间距；全场视野足够时使用 full_workspace 模式选择 15-18 根线，当前现场帧输出 16x16=256 点；视野只容纳少量钢筋时切到 visible_local，支持 2-18 根可见线并只输出当前可见局部交点，不从 2-3 根钢筋推断完整全场。效果页：http://192.168.6.99:8080/reports/live_surface_dp_physical_runtime_20260504_232729/index.html。

## Handoff Documents

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
- `docs/handoff/2026-04-22_current_system_handoff.md`

## Update Protocol

1. 关键修改完成后，运行 `python3 scripts/agent_memory.py add --title ... --summary ... --files ... --validation ...`。
2. 如果手工编辑了 `session_log.md`，运行 `python3 scripts/agent_memory.py refresh`。
3. 结束前运行 `python3 scripts/agent_memory.py check`，确认入口和快照仍可被新会话读取。
