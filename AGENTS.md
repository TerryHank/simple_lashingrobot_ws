# 项目执行说明

在这个仓库里开始任何修改前，先执行轻启动读取：

1. `README.md`
2. `CHANGELOG.md`
3. `docs/agent_memory/current.md`

## 按需扩展读取

轻启动后不要默认展开所有历史文档。根据任务主题再读取：

- 维护共享记忆、写入会话账本或校验记忆系统时，读 `docs/agent_memory/README.md`。
- 调整 agent 行为、工程协作协议或“有机体”口径时，读 `docs/agent_memory/organism.md`。
- 排查 Codex 启动目录、模型输入注入或本地配置时，读 `docs/agent_memory/codex_local_setup.md`。
- 处理断电恢复、长任务续接或 checkpoint/recover 时，读 `docs/agent_memory/power_loss_recovery.md` 和 `docs/agent_memory/checkpoint.md`。
- 如果 `docs/agent_memory/current.md` 指向新的 handoff、专题设计、运行禁区或现场结果目录，只继续读取被当前任务命中的文件。

## Codex 会话启动协议

- Codex 会自动把本仓库根目录 `AGENTS.md` 注入到新会话的模型可见上下文中；可用 `codex debug prompt-input "ping"` 验证。
- 每次开启 Codex 会话，必须先读取轻启动三件套，尤其是 `docs/agent_memory/current.md`，再回答用户或修改代码。
- 如果 `docs/agent_memory/current.md` 指向新的 handoff、专题设计或运行禁区，先按“按需扩展读取”继续读取相关文件，再动手。
- 会话结束前产生跨会话必须继承的信息时，使用 `scripts/agent_memory.py add ...` 写入账本，并运行 `scripts/agent_memory.py refresh`。

## Codex 工程有机体协议

- 本工程里的 Codex 按 `docs/agent_memory/organism.md` 运行：先感知、再行动、持续记忆、完成前自检。
- “有机体”是工程协作协议，不表示 Codex 具备后台自主运行、真实生命或越权行动能力；所有修改仍由当前会话和用户意图驱动。
- 如果会话暴露新的长期知识，先沉淀到共享记忆，再让下一次会话继承。
- 长任务、危险修改或离开机器前，运行 `python3 scripts/agent_memory.py checkpoint ...` 写入断电恢复点；突然断电后运行 `python3 scripts/agent_memory.py recover` 读取恢复报告。

## 示例泛化原则

- 当用户说“比如”“例如”“类似”时，把后面的内容视为意图线索和启发样例，不把它当成唯一需求边界。
- 先根据当前工程状态、代码事实和用户目标举一反三，选择实际合适的实现、检查或文档修改。
- 只有用户明确要求“严格按这个例子”“逐字照做”或“只做这个”时，才把示例收窄为硬约束。
- 如果示例和工程记忆、代码现状或安全边界冲突，先说明差异，再按实际情况处理，避免机械照搬。

## 长上下文瘦身原则

- 默认读取摘要和相关片段，优先用 `docs/agent_memory/current.md`、`rg`、定向 `sed -n`、限定路径的状态检查，不把整仓状态、大段日志、构建产物或无关历史全文塞入上下文。
- 查看 Git 状态时优先限定相关路径；确需全仓状态时只提炼关键摘要，避免把超长 `git status --short` 输出带进会话。
- 读取文档、日志或代码前先定位目标；超过当前任务需要的内容，先摘要再按需继续展开。
- 当 Codex 会话 JSONL 超过 100MB 时，使用 `python3 scripts/codex_session_guard.py summarize --threshold-mb 100 --skip-open --apply` 压缩活跃会话：把原始完整 JSONL 内容复制到 `~/.codex/archived_sessions/oversized/`，不要把 `~/.codex/sessions` 里的会话文件移动出原路径；原路径内容改写为可打开的摘要替身 JSONL，并同步写 `~/.codex/session_summaries/oversized/` Markdown 摘要。
- 摘要替身必须保留原始 `session_meta` 关键字段和首条真实用户请求作为标题锚点；不要让历史会话标题变成“summary/compacted”之类的摘要说明。
- 不要重新启用旧的 `tie-codex-session-guard.timer` 做纯归档清空活跃入口；自动化只使用 `tie-codex-session-summary.timer` 这种“原文归档 + 活跃摘要替身”的压缩流程。
- 会话开始明显变慢、工具输出过大或需要长期暂停时，先写 `python3 scripts/agent_memory.py checkpoint ...`，再建议用新 Codex 会话从轻启动三件套和 checkpoint 续接。

额外约定：

- 修改 `src/tie_robot_web/frontend` 后，如影响静态页面，请重新构建 `src/tie_robot_web/web`。
- 未经用户明确要求，不要恢复控制面板按钮上方的任务提示框。
- 会话结束前如果产生了关键工程知识、架构决策、避坑经验或跨会话必须继承的修改，请使用 `scripts/agent_memory.py add ...` 追加到共享记忆，并运行 `scripts/agent_memory.py refresh` 更新当前快照。
