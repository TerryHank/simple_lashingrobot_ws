# 扫描层响应底图全量实验报告

生成时间：2026-05-04

## 实验输入

固定离线快照：

```text
.debug_frames/rebar_instance_segmentation_modalities_20260430_112028
```

输出报告：

```text
.debug_frames/scan_response_full_evaluation_20260504/index.html
.debug_frames/scan_response_full_evaluation_20260504/summary.json
```

本轮实验强制读取离线 snapshot，不消费 ROS 实时流。

## 结论

底图换成组合响应、Hessian ridge 或补全钢筋面后，确实能形成比当前扫描层 depth-only 更稳的检测路线。

本轮离线代理指标推荐：

```text
04_surface_dp_curve
```

更适合进入扫描层的新主路线是：

```text
组合响应 / fused_instance_response
-> Hessian / Frangi 脊线增强
-> binary candidate + skeleton
-> completed_surface_mask 补全钢筋面
-> 8x8 线族拓扑
-> DP 曲线沿局部 ridge 收束
-> 曲线交点输出
-> instance_graph junction 只做验证 / 补召回，不直接全量输出
```

## 运行链接入状态

2026-05-04 已将该路线接入扫描层运行代码：

- 新增 `tie_robot_perception.pointai.scan_surface_dp` 纯算法模块。
- `run_manual_workspace_s2_pipeline()` 现在优先执行 `Surface-DP`。
- 原 2026-04-22 depth-only S2 保留为 `run_manual_workspace_s2_depth_only_pipeline()`，仅在新链失败或无有效相机坐标时回退。
- `/pointAI/process_image request_mode=3`、发布话题、`PointsArray` 输出结构不变。

固定 snapshot 的 runtime 模块验证：

```text
snapshot: rebar_instance_segmentation_modalities_20260430_112028
base_response: combined_depth_ir_darkline
base_line_counts: [8, 8]
surface_dp_curve line_counts: [8, 8]
surface_dp_curve points: 64
mean_completed_surface_score: 0.984
instance_graph_junction_count: 664
```

## 关键证据

| 方案 | 点数 | 线数 | 融合响应 | Hessian | 补全面 | 二值支撑 | 近重复 | 代理分 |
|---|---:|---|---:|---:|---:|---:|---:|---:|
| `04_surface_dp_curve` | 64 | `[8, 8]` | 0.944 | 0.805 | 0.993 | 0.97 | 0 | 0.928 |
| `06_surface_ir_assisted_curve` | 64 | `[8, 8]` | 0.925 | 0.795 | 0.981 | 0.95 | 0 | 0.917 |
| `05_surface_ridge_curve` | 64 | `[8, 8]` | 0.922 | 0.797 | 0.983 | 0.94 | 0 | 0.915 |
| `03_surface_greedy_curve` | 64 | `[8, 8]` | 0.887 | 0.779 | 0.966 | 0.92 | 0 | 0.886 |
| `hessian_axis_family` | 64 | `[8, 8]` | 0.860 | 0.715 | 0.870 | 0.83 | 0 | 0.834 |
| `frangi_axis_family` | 64 | `[8, 8]` | 0.848 | 0.716 | 0.860 | 0.78 | 0 | 0.825 |
| `fused_instance_axis_family` | 64 | `[8, 8]` | 0.828 | 0.699 | 0.852 | 0.86 | 0 | 0.821 |
| `completed_surface_intersections` | 64 | `[8, 8]` | 0.805 | 0.693 | 0.915 | 0.81 | 0 | 0.812 |
| `instance_graph_junctions_raw` | 128 | `[]` | 0.977 | 0.628 | 0.981 | 1.00 | 0 | 0.792 |
| `depth_pr_fprg_peak_supported` | 64 | `[8, 8]` | 0.673 | 0.438 | 0.710 | 0.58 | 0 | 0.631 |
| `combined_pr_fprg_peak_supported` | 64 | `[8, 8]` | 0.553 | 0.355 | 0.698 | 0.42 | 0 | 0.545 |
| `current_depth_only_runtime` | 867 | `[17, 51]` | 0.288 | 0.098 | 0.275 | 0.09 | 0 | 0.062 |

## 现有算法问题

当前扫描层 depth-only 运行逻辑在同一 snapshot 上生成：

```text
点数：867
线数：[17, 51]
周期：vertical=29 px, horizontal=10 px
```

这再次验证了此前判断：当前扫描层的单帧 depth-only 周期 / 相位铺网格会被伪周期带偏，产生乘法级过密点。

## 底图对比判断

### 组合响应

组合响应不是简单替换底图就结束。它适合做候选响应的主底图，因为它把深度暗线与 IR 暗线融合，钢筋线连续性比单独 depth-only 更好。

### Hessian ridge

Hessian ridge 单独作为主底图可以稳定得到 `[8, 8]` 线族和 64 个点，代理分高于 depth-only。它适合做钢筋线中心支撑和局部评分，但不建议单独替代整条主链。

### Frangi-like

Frangi-like 与 Hessian ridge 表现接近，也稳定得到 64 个点。它适合做多尺度脊线增强，特别是钢筋宽度变化或局部响应断裂时。

### instance graph

实例骨架 junction 原始输出为 128 个点，响应支撑很强，但过检明显。它不适合作为直接输出主链，应作为候选验证或补召回来源。

### completed surface + DP curve

补全钢筋面后再做 DP 曲线收束，是本轮最稳的路线：

- 点数不膨胀。
- 融合响应、Hessian、补全面支撑都高。
- 没有近重复。
- 能把线贴回局部真实 ridge，而不是只输出理想直线交点。

## 推荐实现方案

### 阶段 1：扫描质量门

先加运行保护：

- `point_count` 超过期望范围直接拒绝写账本。
- `line_counts` 明显过密时拒绝写账本。
- 周期低于物理钢筋间距阈值时拒绝写账本。
- 记录 `current_depth_only_runtime` 这类过密风险指标。

### 阶段 2：新扫描候选生成

新增扫描候选函数：

```text
build_scan_candidates_from_completed_surface(...)
```

输入：

- `combined_response`
- `hessian_ridge`
- `frangi_like`
- `fused_instance_response`
- `rectified_valid`
- 当前 PR-FPRG 线族先验

输出：

- `completed_surface_mask`
- `completed_surface_response`
- `completed_line_families`
- `surface_dp_curve` 交点

### 阶段 3：局部质量评分

每个候选点保存：

- `score_combined`
- `score_hessian`
- `score_frangi`
- `score_completed_surface`
- `binary_support`
- `nearest_duplicate_distance`
- `source`

低分候选拒绝输出。

### 阶段 4：instance graph 只做辅助

`instance_graph_junctions` 不直接输出全部点。建议用途：

1. 验证 DP 曲线交点附近是否存在 skeleton junction。
2. 在某些格点缺失时做补召回。
3. 生成调试图，帮助判断钢筋实例分割质量。

### 阶段 5：现场真值验证

当前 snapshot 没有人工标注真值，因此本报告的代理分不等价于真实精度。

下一步必须补：

- 在 1 到 3 个现场帧上标注真实交点。
- 计算 Precision / Recall / F1。
- 以真实 238 个点为全局扫描质量门。

## 文献依据

- RGB-D + Hough multi-segment fitting 已在钢筋绑扎机器人交点检测中报告高精度。
- LSD / Steger / Frangi 都支持「线状结构检测 + 交点」路线。
- YOLO-FAS 说明长期可用轻量模型做交点验证，但当前项目短期可先走几何 + 脊线增强。
