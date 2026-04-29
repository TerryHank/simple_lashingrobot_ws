# Codex 本地启动说明

本说明用于确保从本工程目录开启的 Codex 会话能加载 `AGENTS.md`，并因此按轻启动协议读取共享记忆。

## 推荐启动方式

在任意目录都可以显式指定工程根目录：

```bash
codex -C /home/hyq-/simple_lashingrobot_ws
```

非交互执行同理：

```bash
codex exec -C /home/hyq-/simple_lashingrobot_ws "你的任务"
```

如果已经在工程根目录，也可以直接运行：

```bash
cd /home/hyq-/simple_lashingrobot_ws
codex
```

## 可选 alias

可以在 shell 配置中添加一个本地别名：

```bash
alias tie-codex='codex -C /home/hyq-/simple_lashingrobot_ws'
```

这不是项目必须项，只是减少从错误目录启动 Codex 的概率。

## 验证启动上下文

用下面命令检查 Codex 新会话的模型可见输入：

```bash
codex debug prompt-input "ping" | rg "Codex 会话启动协议|Codex 工程有机体协议|docs/agent_memory/current.md"
```

看到匹配内容，说明 `AGENTS.md` 已被加载。若没有匹配，请确认当前目录或 `-C` 参数指向 `/home/hyq-/simple_lashingrobot_ws`。

新会话默认只需要从轻启动三件套进入；只有在排查启动注入、维护记忆系统或处理断电恢复时，才继续展开对应专题文档。

## 本地配置现状

当前用户级 Codex 配置中，本工程已被标记为 trusted 项目。这个配置位于用户目录，不作为仓库内容入库；仓库内的长期规则仍以 `AGENTS.md` 和 `docs/agent_memory/` 为准。
