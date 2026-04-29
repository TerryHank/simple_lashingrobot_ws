# 断电恢复层

本文件定义突然断电、终端被杀、远程连接中断或 Codex 会话丢失后的恢复口径。

## 能保证什么

断电恢复层能恢复的是已经落盘的工程现场：

- `AGENTS.md` 和 `docs/agent_memory/current.md` 中的启动先验。
- `docs/agent_memory/session_log.md` 中已写入的长期记忆。
- `docs/agent_memory/checkpoint.md` 中最近一次任务 checkpoint。
- Git 工作区中已经写入磁盘的文件改动。

## 不能保证什么

它不能保证恢复：

- 未保存的 IDE buffer。
- 断电瞬间正在写入且未完成落盘的文件。
- 运行中的 ROS 节点、终端进程、硬件连接状态。
- 没有写进 checkpoint 或 session log 的临时思考。

所以本层目标不是绝对无损，而是把“下一次会话最需要知道什么”提前落盘。

## checkpoint

在长任务、硬件联调、大范围编辑、离开机器或准备运行高风险命令前，写入恢复点：

```bash
python3 scripts/agent_memory.py checkpoint \
  --task "当前任务" \
  --next "下一步动作" \
  --files "关键文件或目录" \
  --validation "刚运行过的验证命令" \
  --note "额外注意"
```

该命令会写入 `docs/agent_memory/checkpoint.md`，包含当前任务、下一步、影响范围、最近验证和 Git 状态摘要。写入使用临时文件替换，降低半写入风险。

## recover

断电后或新会话接手时运行：

```bash
python3 scripts/agent_memory.py recover
```

输出会合并：

- 轻启动入口文件。
- 最近的 `checkpoint.md`。
- 当前共享记忆快照。
- 建议恢复步骤。

恢复后先跑：

```bash
python3 scripts/agent_memory.py check
git status --short
```

再根据 checkpoint 的“下一步”继续。

## 使用节奏

- 每完成一个小阶段，写一次 checkpoint。
- 每次验证后，把验证命令加入 checkpoint。
- 任务完成时，用 `scripts/agent_memory.py add ...` 写入长期记忆，再刷新 `current.md`。
