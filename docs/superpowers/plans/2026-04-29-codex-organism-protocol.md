# Codex 工程有机体协议实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让本工程目录下的 Codex 通过项目级规则表现为具备感知、记忆、免疫和生长闭环的工程有机体。

**架构：** 以 `AGENTS.md` 作为 Codex 自动注入入口，`docs/agent_memory/organism.md` 定义行为协议，`docs/agent_memory/codex_local_setup.md` 说明本地启动方式，`scripts/agent_memory.py check` 和契约测试负责防止入口断链。

**技术栈：** Markdown、Python `unittest`、现有 `scripts/agent_memory.py`。

---

### 任务 1：契约测试

**文件：**
- 修改：`src/tie_robot_bringup/test/test_agent_memory_contract.py`

- [x] **步骤 1：编写失败的测试**

检查 `AGENTS.md` 包含 `Codex 工程有机体协议`，并检查 `docs/agent_memory/organism.md`、`docs/agent_memory/codex_local_setup.md` 存在且包含关键章节。

- [x] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py -v`

预期：FAIL，缺少有机体协议文档和 AGENTS 入口。

### 任务 2：协议文档和入口

**文件：**
- 创建：`docs/agent_memory/organism.md`
- 创建：`docs/agent_memory/codex_local_setup.md`
- 修改：`AGENTS.md`
- 修改：`docs/agent_memory/README.md`
- 修改：`README.md`
- 修改：`CHANGELOG.md`

- [x] **步骤 1：补有机体协议**

写清感知层、记忆层、免疫层、生长层和行动边界。

- [x] **步骤 2：补本地启动说明**

写清 `codex -C /home/hyq-/simple_lashingrobot_ws` 和 `codex debug prompt-input` 验证方式。

### 任务 3：校验和快照

**文件：**
- 修改：`scripts/agent_memory.py`
- 修改：`docs/agent_memory/current.md`
- 修改：`docs/agent_memory/session_log.md`

- [x] **步骤 1：扩展 `check`**

校验 AGENTS、README、有机体协议和本地启动说明。

- [x] **步骤 2：刷新并验证**

运行：

```bash
python3 scripts/agent_memory.py refresh
python3 scripts/agent_memory.py check
python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py -v
```
