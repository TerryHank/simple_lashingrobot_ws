# 绑扎点已绑/未绑分类 v55 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 在当前 pointAI 稳定绑扎点输出链路上新增高精度、保守三态分类，区分已绑扎、未绑扎和不确定。

**架构：** 新增 ROS-free 规则分类模块，负责 patch 证据提取、深度差/脊线/IR 纹理评分、JSONL 事件记录。pointAI 在 `MODE_BIND_CHECK` 稳定点释放后调用分类器，默认 `shadow` 只记录；`advisory/blocking` 才把高置信已绑扎写入 `PointCoords.is_shuiguan` 兼容下游。

**技术栈：** ROS Noetic、Python 3、OpenCV、NumPy、YAML、unittest、现有 `tie_robot_perception.pointai` 模块。

---

## 文件结构

- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/bind_point_classification.py`
  - 定义 `ClassificationConfig`、`EvidenceBundle`、`RuleDecision`
  - 加载 YAML 配置
  - 从 IR、depth/raw_world 和候选点提取局部证据
  - 计算规则分类和绑后差分复检评分
  - 追加 JSONL 事件

- 修改：`src/tie_robot_perception/config/bind_point_classification.yaml`
  - 承载默认 `shadow` 模式、阈值、patch 尺寸和证据路径

- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
  - 初始化分类配置路径、配置对象和最近分类结果缓存

- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
  - 在 `load_runtime_config` 中重新加载分类 YAML，允许 ROS 参数覆盖配置路径

- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`
  - 在 `MODE_BIND_CHECK` 稳定点返回前运行预分类
  - 追加事件日志
  - 默认不阻断，只有 `advisory/blocking` 写入 `is_shuiguan`

- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
  - 绑定分类相关方法到 `ImageProcessor`

- 修改：`src/tie_robot_bringup/launch/algorithm_stack.launch`
  - 为 `pointAINode` 暴露 `bind_classification_config_path`

- 创建：`src/tie_robot_perception/test/test_bind_point_classification.py`
  - 单测规则分类器、事件日志、证据提取

- 修改：`src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
  - 增加文本级回归，确保 `MODE_BIND_CHECK` 接入分类且默认影子模式不误阻断

## 任务 1：锁定规则分类器红灯测试

**文件：**
- 创建：`src/tie_robot_perception/test/test_bind_point_classification.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/bind_point_classification.py`

- [ ] **步骤 1：编写失败的规则单测**

测试覆盖：
- 平滑、低纹理、无中心凸起 patch 判为 `unbound`
- 中心有深度凸起和高频 IR 结节判为 `bound`
- 有效深度比例不足判为 `uncertain`
- JSONL 每次追加一个合法 JSON 对象

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_bind_point_classification -v
```

预期：失败，原因是 `bind_point_classification` 模块或函数尚未实现。

- [ ] **步骤 3：提交红灯测试**

```bash
git add src/tie_robot_perception/test/test_bind_point_classification.py
git commit -m "test(感知): 锁定绑扎点分类规则行为"
```

## 任务 2：实现 ROS-free 规则分类器

**文件：**
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/bind_point_classification.py`
- 修改：`src/tie_robot_perception/config/bind_point_classification.yaml`
- 测试：`src/tie_robot_perception/test/test_bind_point_classification.py`

- [ ] **步骤 1：实现配置、证据和评分类型**

实现 `ClassificationConfig`、`EvidenceBundle`、`RuleDecision`，配置包含模式、路径、patch 半径、有效深度比例、深度凸起阈值、脊线断裂阈值和双阈值。

- [ ] **步骤 2：实现证据提取**

从 `image_infrared`、`Depth_image_Raw` 或 `image_raw_world` 裁局部 patch，生成：
- `ir_patch`
- `depth_patch`
- `height_patch`
- `valid_depth_mask`
- `ridge_patch`
- `metrics`

- [ ] **步骤 3：实现规则评分**

组合：
- 中心深度凸起
- 中心/环形深度差
- IR 高频纹理
- skeleton 脊线中心破坏
- 证据质量

- [ ] **步骤 4：运行规则单测验证通过**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_bind_point_classification -v
```

预期：全部通过。

- [ ] **步骤 5：提交规则分类器**

```bash
git add src/tie_robot_perception/src/tie_robot_perception/pointai/bind_point_classification.py src/tie_robot_perception/config/bind_point_classification.yaml src/tie_robot_perception/test/test_bind_point_classification.py
git commit -m "feat(感知): 添加绑扎点规则分类器"
```

## 任务 3：接入 pointAI 的 MODE_BIND_CHECK

**文件：**
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- 修改：`src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

- [ ] **步骤 1：编写失败的接入测试**

测试要求：
- `process_image_service.py` 在 `PROCESS_IMAGE_MODE_BIND_CHECK` 成功释放前调用 `classify_bind_check_points`
- `state.py` 初始化 `bind_classification_config`
- 默认 `shadow` 不把已绑扎写入 `is_shuiguan`
- `advisory/blocking` 才允许写入 `is_shuiguan`

- [ ] **步骤 2：运行接入测试验证失败**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest -v
```

预期：新增测试失败。

- [ ] **步骤 3：实现 pointAI 接入**

新增方法：
- `classify_bind_check_points(self, point_coords)`
- `should_mark_point_as_bound(self, decision)`

行为：
- `mode=off` 直接返回原点集
- `mode=shadow` 记录完整结果但 `is_shuiguan=False`
- `mode=advisory` 和 `mode=blocking` 中，高置信 `bound` 写 `is_shuiguan=True`
- `unbound/uncertain` 永远不写 `True`

- [ ] **步骤 4：运行接入测试验证通过**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest -v
```

预期：全部通过。

- [ ] **步骤 5：提交 pointAI 接入**

```bash
git add src/tie_robot_perception/src/tie_robot_perception/pointai src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py
git commit -m "feat(感知): 接入绑扎点分类到绑定检查"
```

## 任务 4：运行配置、验证和文档记忆

**文件：**
- 修改：`src/tie_robot_bringup/launch/algorithm_stack.launch`
- 修改：`docs/agent_memory/session_log.md`（通过脚本）
- 修改：`docs/agent_memory/current.md`（通过脚本）

- [ ] **步骤 1：给 launch 暴露配置路径**

在 `pointAINode` 增加：

```xml
<param name="bind_classification_config_path" value="$(find tie_robot_perception)/config/bind_point_classification.yaml" />
```

- [ ] **步骤 2：运行目标验证**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_bind_point_classification -v
PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest -v
PYTHONPATH=src/tie_robot_perception/src python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/pointai/bind_point_classification.py src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py
```

预期：全部退出码为 0。

- [ ] **步骤 3：写入共享记忆**

运行：

```bash
python3 scripts/agent_memory.py add --title "绑扎点分类 shadow 接入" --summary "新增 pointAI 绑扎点已绑/未绑三态规则分类，默认 shadow 只记录 JSONL，不阻断执行；advisory/blocking 才把高置信 bound 写入 is_shuiguan。" --files "src/tie_robot_perception/src/tie_robot_perception/pointai/bind_point_classification.py;src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py;src/tie_robot_perception/config/bind_point_classification.yaml" --validation "目标 unittest 与 py_compile 通过"
python3 scripts/agent_memory.py refresh
```

- [ ] **步骤 4：提交最终验证和记忆**

```bash
git add src/tie_robot_bringup/launch/algorithm_stack.launch docs/agent_memory/session_log.md docs/agent_memory/current.md docs/superpowers/plans/2026-05-15-bind-point-classification-v55.md
git commit -m "docs(感知): 记录绑扎点分类实现计划"
```

