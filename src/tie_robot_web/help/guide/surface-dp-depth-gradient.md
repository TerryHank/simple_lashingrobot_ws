# Surface-DP 当前视觉方案

扫描层视觉现在采用**设置页选择的一种底图**作为主链响应。每次运行只生成被激活的底图，并在这张底图上完成物理线族识别、统一物理网格校对和 Surface-DP 曲线追踪；其它底图保留在代码和离线工具中，只作为解释、对照和回溯材料。

## 当前口径

- 中文名：Surface-DP 单源底图方案。
- 默认运行源：`depth_gradient`（深度梯度边缘）。
- 运行策略：`single_selected_response`。
- 运行次数：只做一次响应图构建和一次物理线族识别。
- 物理线距先验：120-160 mm。
- 网格校对：`unified_physical_lattice`，不按小视野、大视野、长方形或正方形分档。
- 可选底图：融合实例响应、Frangi-like 脊线、Hessian ridge 脊线、深度梯度边缘、红外响应、组合响应、深度响应。

这版口径的含义是：主链只相信当前选中的一张底图，不再先跑一轮多模态候选、再跑一轮旧补全面候选。默认的深度梯度边缘更关注深度图上的局部变化边缘；如果现场光照、反光或钢筋形态变化，可以在前端「设置 / 视觉调试」里的「扫描底图」下拉栏切换其它底图。前端会把上一次选择保存到本地，并在 ROS 重连后自动下发给 PointAI。

## 主链流程

1. 读取手动工作区四边形和当前 `raw_world_coord` / 深度数据。
2. 将工作区透视展开成 rectified 画幅。
3. 根据「扫描底图」选择，只生成当前被激活的一张响应图。
4. 对响应图做阈值和骨架诊断；这些诊断不直接产出绑扎点。
5. 在当前响应图上按 120-160 mm 物理钢筋间距寻找横纵线族。
6. 用统一物理网格评分校对线族，拒绝线数比例和可见物理视野长宽不一致的结果。
7. 用 Surface-DP 沿当前响应图做曲线追踪，得到两组曲线线族。
8. 求曲线交点，通过 inverse H 投回原图，再从 `raw_world_coord` 反查相机坐标。
9. 发布扫描点、结果图、扫描底图诊断图和 `surface_dp_bind_point_*` TF。

梁筋候选仍作为当前运行链路的一层诊断和可选过滤：`beam_candidate_bands` 会在扫描底图上显示为红色竖带；前端启用梁筋过滤后，PointAI 按设置半径扩张候选 mask，只删除落入 mask 的最终交点。它不会因为一个交点命中梁筋 mask，就连带移除同一 X 位置的其它交点。

默认 `depth_gradient` 的构建步骤是：对填补后的深度图做轻量高斯平滑，用 Sobel 计算 X / Y 方向梯度，再取梯度幅值并归一化。

## 前端切换

- 入口：`设置 / 视觉调试 / 扫描底图`。
- 前端话题：`/web/pointAI/set_scan_response_source`。
- 消息类型：`std_msgs/String`。
- 后端参数：`~scan_response_source`。
- 默认值：`depth_gradient`。

可选值如下：

| 中文名 | 参数值 |
| --- | --- |
| 融合实例响应 | `fused_instance_response` |
| Frangi-like 脊线 | `frangi_like` |
| Hessian ridge 脊线 | `hessian_ridge` |
| 深度梯度边缘 | `depth_gradient` |
| 红外响应 | `infrared_response` |
| 组合响应 | `combined_response` |
| 深度响应 | `depth_response` |

## 主链效果图

每个流程节点的完整效果图见：[当前视觉流程效果图](./current-visual-flow)。这里保留 Surface-DP 主链的核心图。

当前输入工作区：

![当前输入工作区](/images/visual/surface-dp-depth-gradient/input-workspace.png)

透视展开后的红外参考图：

![透视展开红外参考](/images/visual/surface-dp-depth-gradient/rectified-ir.png)

透视展开后的深度参考图：

![透视展开深度参考](/images/visual/surface-dp-depth-gradient/rectified-depth.png)

深度梯度边缘上的横纵线族：

![深度梯度边缘线族](/images/visual/surface-dp-depth-gradient/depth-gradient-lines.png)

深度梯度结果投回原图：

![深度梯度原图投影](/images/visual/surface-dp-depth-gradient/depth-gradient-original.png)

## 统一物理网格校对

旧版曾经用「横纵线族数量接近 `1:1`」来拒绝极端假阳，这对正方形网格有用，但会误伤长方形钢筋面。当前版本改为统一物理网格评分：

- 横纵线数可以不相等，允许 `3 m x 5 m` 这类长方形版面自然出现更多长边方向钢筋。
- 校对使用线数间隔比例，而不是线条数量本身。
- 线数间隔比例要匹配 rectified 有效 mask 的物理长宽比。
- 小视野因为可见间隔少，会自动获得更宽的 tolerance，但仍要求两轴都成立。
- `16 x 2` 这类只在一个方向大量出线、另一个方向近乎缺失的结果，如果和视野物理长宽不匹配，会被 `count_aspect_mismatch` 拒绝。

诊断字段：

```text
physical_prior_modes = [unified_physical_lattice, unified_physical_lattice]
physical_lattice_score
physical_lattice_count_aspect
physical_lattice_visible_aspect
physical_lattice_count_aspect_error
physical_lattice_count_aspect_tolerance
```

判断规则：

```text
physical_lattice_count_aspect_error <= physical_lattice_count_aspect_tolerance
```

成立时，候选网格的线数比例和当前可见物理视野一致；不成立时，说明线族更像局部杂线、梁筋、边缘反光或单轴假阳。

## 隐藏模态说明

未选中的模态不进入扫描层运行主链，但保留在帮助文档和离线诊断工具中，便于对比哪一种底图更适合现场画面。

| 底图 | 原理 | 在当前口径中的位置 |
| --- | --- | --- |
| 深度响应 | 用局部背景深度减去当前深度，突出高于背景的结构。 | 可选主链底图。 |
| 红外响应 | 用局部红外背景减去当前红外强度，突出红外变暗的线状区域。 | 可选主链底图。 |
| 组合响应 | 将深度响应和红外响应按权重融合。 | 可选主链底图。 |
| Hessian ridge | 在组合响应上用二阶导数提取亮脊线。 | 可选主链底图。 |
| Frangi-like | 多尺度提取线状脊结构，适合连续细线。 | 可选主链底图。 |
| 融合实例响应 | 混合组合响应、深度、Frangi、梯度和 Hessian。 | 可选主链底图。 |
| 旧补全钢筋面响应 | 第一轮找线后把线族画回响应图，再做第二轮找线。 | 仅离线对照；主链不再使用第二轮。 |

深度暗线响应离线效果：

![深度暗线响应离线效果](/images/visual/surface-dp-depth-gradient/hidden-depth-response.png)

深度 + 红外组合响应离线效果：

![深度 + 红外组合响应离线效果](/images/visual/surface-dp-depth-gradient/hidden-combined-response.png)

Frangi-like 离线效果：

![Frangi-like 离线效果](/images/visual/surface-dp-depth-gradient/hidden-frangi-like.png)

Hessian ridge 离线效果：

![Hessian ridge 离线效果](/images/visual/surface-dp-depth-gradient/hidden-hessian-ridge.png)

红外响应离线效果：

![红外响应离线效果](/images/visual/surface-dp-depth-gradient/hidden-infrared-response.png)

融合实例响应离线效果：

![融合实例响应离线效果](/images/visual/surface-dp-depth-gradient/hidden-fused-instance-response.png)

旧补全钢筋面离线效果：

![旧补全钢筋面离线效果](/images/visual/surface-dp-depth-gradient/legacy-completed-surface.png)

## 运行诊断字段

`build_scan_surface_dp_result()` 的诊断信息会给出当前选择：

```text
scan_runtime_response_policy = single_selected_response
scan_runtime_response_source = depth_gradient       # 或当前下拉栏选择的其它源
scan_runtime_response_source_requested = depth_gradient
base_physical_source = depth_gradient               # 与当前选中源一致
completed_physical_source = depth_gradient          # 兼容字段，与当前选中源一致
beam_candidate_count
beam_candidate_margin_pixels
beam_filter_mode = point_mask                       # 启用梁筋过滤时只做点级删除
beam_filtered_point_count
```

这里保留 `base_physical_source` 和 `completed_physical_source` 字段，是为了兼容已有日志和前端显示；在当前口径下，两者都应指向当前选中的同一个源。实际运行不再有第一轮多模态候选和第二轮补全面候选。

## 相机 SDK 热修改

「设置 / 相机底层 SDK 调试」面板用来热修改 `scepter_manager` 的 `dynamic_reconfigure` 参数。它走的是 ROS 官方动态参数方案，等价于手工运行：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

前端调用的服务是 `/scepter_manager/set_parameters`，服务类型是 `dynamic_reconfigure/Reconfigure`；同时监听 `/scepter_manager/parameter_updates` 和 `/scepter_manager/parameter_descriptions`。参数名沿用相机驱动的 `Sceptertof_roscpp.cfg`，例如 `FrameRate`、`XDRMode`、`ToFManual`、`ToFExposureTime`、`ColorResloution`、`DepthCloudPoint` 等。前端会把面板里的值保存到浏览器本地设置，ROS 重连后自动回放，避免每次重启都要重新手动调一遍。
