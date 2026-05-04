# Agent 共享记忆

这套目录用于给进入本仓库的 Codex、其他 AI 代理和人工工程师提供同一份先验知识。它不是运行时业务数据，而是跨会话的工程记忆层。

## 轻启动读取顺序

任何 agent 在开始修改前按根目录 `AGENTS.md` 执行轻启动，默认只读取：

1. `README.md`
2. `CHANGELOG.md`
3. `docs/agent_memory/current.md`

读完后再进入具体代码和包内文档。这样新会话能先拿到项目总览、近期变更和上一批会话留下的关键判断，同时避免把不相关历史全文带入上下文。

## 按需扩展读取

轻启动后，根据任务主题再读取更多记忆文件：

- 维护共享记忆、写入账本或校验契约：读 `docs/agent_memory/README.md` 和 `scripts/agent_memory.py`。
- 调整 agent 行为、有机体协议或协作口径：读 `docs/agent_memory/organism.md`。
- 排查 Codex 启动目录、模型输入或本地配置：读 `docs/agent_memory/codex_local_setup.md`。
- 断电恢复、长任务续接或高风险修改：读 `docs/agent_memory/power_loss_recovery.md` 和 `docs/agent_memory/checkpoint.md`。
- `current.md` 指向的 handoff、专题设计或结果目录只在任务相关时继续展开。

## Codex 接入方式

Codex 新会话会把仓库根目录 `AGENTS.md` 放入模型可见上下文，因此本项目通过 `AGENTS.md` 驱动启动读取记忆系统。

可用下面命令确认当前 Codex 输入里包含记忆入口：

```bash
codex debug prompt-input "ping" | rg "docs/agent_memory/current.md|Codex 会话启动协议"
```

如果这条命令看不到对应内容，说明启动目录不在本仓库根目录，或者 Codex 没有加载本仓库的 `AGENTS.md`。此时应使用 `codex -C /home/hyq-/simple_lashingrobot_ws` 或在仓库根目录重新开启会话。

## 文件职责

- `current.md`：当前共享记忆快照，面向新会话快速启动。
- `session_log.md`：追加式会话记忆账本，记录关键修改、决策、验证证据和后续风险。
- `session_template.md`：手工补写记忆时使用的模板。
- `organism.md`：Codex 工程有机体协议，定义感知、记忆、免疫和生长闭环。
- `codex_local_setup.md`：本地启动说明，确保 Codex 从正确工程目录加载 `AGENTS.md`。
- `power_loss_recovery.md`：断电恢复层协议，定义 checkpoint / recover 的使用口径。
- `checkpoint.md`：最近一次可恢复现场，供突然断电后的下一次会话读取。
- `scripts/agent_memory.py`：追加、刷新和校验记忆系统的 CLI。

## 写入规则

需要记录的内容：

- 会影响后续 agent 判断的架构决策、接口语义、坐标系约定和启动链路。
- 容易被误恢复的历史方案、废弃逻辑和现场避坑经验。
- 跨包修改后的新入口、新命令、新测试和验证证据。
- 用户明确指定的长期偏好或禁区。
- 用户表达习惯中的长期协作偏好；例如“比如”“例如”通常是启发样例，不是要求 agent 机械照搬。

不需要记录的内容：

- 大段日志、完整构建产物、临时调试输出。
- 只在当前会话有效的推测。
- 已经能从代码本身直接读出的普通实现细节。

## 常用命令

追加一条记忆并刷新快照：

```bash
python3 scripts/agent_memory.py add \
  --title "简短标题" \
  --summary "这次会话留下的关键先验" \
  --files "src/example.cpp,docs/example.md" \
  --validation "python3 -m unittest ..."
```

只刷新 `current.md`：

```bash
python3 scripts/agent_memory.py refresh
```

校验记忆系统契约：

```bash
python3 scripts/agent_memory.py check
```

写入断电恢复点：

```bash
python3 scripts/agent_memory.py checkpoint \
  --task "当前任务" \
  --next "下一步动作" \
  --validation "刚运行过的验证命令"
```

断电或会话中断后恢复：

```bash
python3 scripts/agent_memory.py recover
```

## 维护口径

`session_log.md` 保持新条目在最上方，便于 `current.md` 摘取最近上下文。`current.md` 可以由脚本覆盖生成；如果要长期保留某条知识，请先写入 `session_log.md`，再刷新快照。

## 长上下文瘦身

- `current.md` 是新会话的快速路径；不要默认全文读取 `session_log.md`、大型日志、构建产物或 `.debug_frames`。
- 优先使用 `rg`、限定路径的 `git status --short -- <path>` 和定向 `sed -n` 查找相关片段。
- 如果当前会话已经因为上下文过长变慢，先写 checkpoint，再开新会话用轻启动三件套续接。
- 共享记忆只记录高信号结论，不记录大段原始输出。

## Codex 会话压缩

用户最新要求：超大 Codex 会话的原始内容要归档，但 `~/.codex/sessions` 里的会话文件不要移动；存活会话列表下保留自动总结后的上下文。
当活跃会话超过 100MB 时，运行：

```bash
python3 scripts/codex_session_guard.py summarize --threshold-mb 100 --skip-open --apply
```

该命令会先把原始完整 JSONL 内容复制到 `~/.codex/archived_sessions/oversized/`，不把 `~/.codex/sessions` 里的会话文件移动出原路径；随后在原路径写入小型摘要 JSONL，让历史会话列表仍能快速打开并看到上下文重点；同时写一份便于人读的 Markdown 到 `~/.codex/session_summaries/oversized/`。

摘要替身必须保留原始 `session_meta` 关键字段和首条真实用户请求作为标题锚点，避免历史会话标题变成摘要说明。若已有摘要替身缺少标题锚点，可从 archive 原文修复：

```bash
python3 scripts/codex_session_guard.py repair-summaries --apply
```

本机可安装自动总结定时器：

```bash
scripts/install_codex_session_summary_timer.sh
```

它会启用用户级 `tie-codex-session-summary.timer`，开机后 5 分钟运行一次，之后每 15 分钟运行一次；默认压缩超过 100MB、闲置至少 10 分钟且未被打开的 JSONL。

`archive` 子命令只保留为手工救急工具。不要安装或启用旧的纯归档 timer，因为它只移走活跃入口、不留下摘要替身。旧安装脚本默认拒绝启用：

```bash
scripts/install_codex_session_guard_timer.sh
```
