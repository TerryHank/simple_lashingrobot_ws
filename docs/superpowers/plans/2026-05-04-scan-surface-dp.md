# 扫描层 Surface-DP 绑扎点识别实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将用户指定的 combined/fused + Hessian/Frangi + completed surface + DP 曲线交点方案接入扫描层。

**架构：** 新增 runtime 纯算法模块 `scan_surface_dp.py`，从现有报告脚本迁移最小必要算法。`manual_workspace_s2.py` 在扫描入口优先调用新模块，失败时回退旧 depth-only S2。

**技术栈：** Python、NumPy、OpenCV、现有 `workspace_s2.py` 线族与曲线追踪工具、ROS1 pointAI 消息链。

---

### 任务 1：纯算法模块测试

**文件：**
- 创建：`src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`

- [ ] **步骤 1：编写失败测试**

```python
def test_surface_dp_outputs_curve_intersections_on_synthetic_grid():
    result = build_synthetic_rectified_grid()
    output = scan_surface_dp.build_scan_surface_dp_result(result)
    assert output["success"] is True
    assert output["variant_id"] == "surface_dp_curve"
    assert len(output["rectified_intersections"]) == 16
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m pytest src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -q`
预期：FAIL，报错缺少 `scan_surface_dp` 或缺少 `build_scan_surface_dp_result`。

- [ ] **步骤 3：实现最少纯算法**

迁移并收敛这些能力：响应归一化、combined/fused 响应、Hessian/Frangi、binary/skeleton、completed surface、DP 曲线族、曲线交点。

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m pytest src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -q`
预期：PASS。

### 任务 2：扫描入口接入

**文件：**
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- 修改：`src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

- [ ] **步骤 1：编写失败测试**

```python
def test_manual_workspace_s2_prefers_surface_dp_before_depth_only_fallback():
    text = MANUAL_WORKSPACE_S2_PATH.read_text(encoding="utf-8")
    assert "run_manual_workspace_surface_dp_pipeline" in text
    assert text.index("run_manual_workspace_surface_dp_pipeline") < text.index("run_manual_workspace_s2_depth_only_pipeline")
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m pytest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py::PointAIScanOnlyPrFrpgTest::test_manual_workspace_s2_prefers_surface_dp_before_depth_only_fallback -q`
预期：FAIL。

- [ ] **步骤 3：接入运行入口**

保留旧 depth-only 实现并重命名为 fallback；新增 Surface-DP 包装函数，复用原有 `PointsArray` 和结果图发布。

- [ ] **步骤 4：运行相关测试验证通过**

运行：`python3 -m pytest src/tie_robot_perception/test/test_scan_surface_dp_runtime.py src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -q`
预期：PASS 或只暴露与本次用户显式算法切换相关的旧口径测试，需要同步更新。

### 任务 3：离线实验回归与记忆

**文件：**
- 修改：`docs/reports/scan_response_full_evaluation_2026-05-04.md`
- 修改：`docs/agent_memory/session_log.md`
- 修改：`docs/agent_memory/current.md`

- [ ] **步骤 1：运行固定 snapshot 实验**

运行：
`python3 src/tie_robot_perception/tools/scan_response_full_evaluation.py --snapshot-dir .debug_frames/rebar_instance_segmentation_modalities_20260430_112028 --output-dir .debug_frames/scan_response_full_evaluation_20260504 --threshold-percentile 83 --ir-display-gamma 1.95`

- [ ] **步骤 2：运行语法和目标测试**

运行：
`python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`

- [ ] **步骤 3：写入共享记忆**

运行：
`python3 scripts/agent_memory.py add --title "扫描层 Surface-DP 主链接入" --summary "..." --files "..." --validation "..."`
`python3 scripts/agent_memory.py refresh`
`python3 scripts/agent_memory.py check`
