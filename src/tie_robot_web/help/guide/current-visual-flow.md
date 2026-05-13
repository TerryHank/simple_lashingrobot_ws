# 当前视觉流程效果图

本页把当前视觉方案按现场可观察流程拆开说明：扫描建图走 Surface-DP 单源底图与统一物理网格校对；执行微调走平面分割 + Hough。每个流程节点都配有对应效果图，便于对照前端图层、ROS topic 和诊断日志。

## 总览

```text
扫描建图
-> MODE_SCAN_ONLY / request_mode=3
-> Surface-DP 单源底图
-> 统一物理网格评分
-> 物理线族交点
-> pseudo_slam_points.json / pseudo_slam_bind_path.json

执行微调
-> MODE_EXECUTION_REFINE / request_mode=4
-> 平面分割 world_coord
-> Hough 线段和交点
-> TCP 执行范围内 2x2 点
```

当前扫描校对不是强制横纵线数相等，而是比较「候选线数比例」和「当前可见物理视野长宽比」。因此长方形钢筋面、小视野局部、正方形视野和大视野全局都走同一套判据。

## 扫描建图流程

### 1. 工作区输入

![工作区输入](/images/visual/current-visual-flow/00_surface_dp_input_workspace.png)

前端保存 4 个工作区角点后，PointAI 用这块四边形限制扫描范围。扫描只在工作区内部做透视展开、响应图构建和点位反查。

### 2. 透视展开红外参考

![透视展开红外参考](/images/visual/current-visual-flow/01_surface_dp_rectified_ir.png)

四边形工作区被展开成 rectified 平面。红外图只作为人工检查和结果叠加参考，不是默认主响应源。

### 3. 透视展开深度参考

![透视展开深度参考](/images/visual/current-visual-flow/02_surface_dp_rectified_depth.png)

`raw_world_coord` 的 Z 通道被同步展开，并对无效深度做填补。默认 `depth_gradient` 响应就来自这张深度参考图。

### 4. 单源底图线族识别

![深度梯度线族识别](/images/visual/current-visual-flow/03_surface_dp_depth_gradient_lines.png)

当前运行策略是 `single_selected_response`：只生成并使用前端选择的一个扫描底图。默认 `depth_gradient` 会突出深度局部变化边缘，再按 120-160 mm 物理间距寻找横纵线族。

### 5. 统一物理网格校对

![统一物理网格校对](/images/visual/current-visual-flow/04_surface_dp_runtime_rectified.png)

校对关注物理一致性：线距是否合理、两轴是否都存在、线数间隔比例是否匹配有效视野的物理长宽比。`34 x 21` 这类长方形网格可以通过；正方形视野中出现 `16 x 2` 这类单轴假阳会被拒绝。

关键字段：

```text
physical_prior_modes = [unified_physical_lattice, unified_physical_lattice]
physical_lattice_count_aspect_error <= physical_lattice_count_aspect_tolerance
```

### 6. 物理线族交点投回原图

![Surface-DP 原图输出](/images/visual/current-visual-flow/05_surface_dp_runtime_original.png)

通过校对的线族会进入 Surface-DP 曲线追踪。最终物理线族交点通过 inverse H 投回原图，再从 `raw_world_coord` 查找相机系三维坐标，发布到 `/perception/lashing/points_camera`、`/coordinate_point` 和扫描账本。扫描底图会继续用红色竖带叠加 `beam_candidate` 梁筋候选；视觉调试启用梁筋过滤时，只删除落入梁筋扩张 mask 的点，不会连带移除同一 X 位置的其它交点。

## 执行微调流程

### 1. 平面分割世界坐标输入

![平面分割世界坐标输入](/images/visual/current-visual-flow/11_execution_world_coord_z.png)

执行微调读取 `/Scepter/worldCoord/world_coord`。这一路已经经过上游主平面分割，适合在逐区到位后提取局部钢筋线。

### 2. Hough 二值底图

![Hough 二值底图](/images/visual/current-visual-flow/12_execution_binary.png)

执行分支把世界坐标 Z 通道转成二值候选图，并做连通域清理。这个底图会发布到 `/perception/lashing/execution_refine_base_image`，便于现场排查。

### 3. 轴向 Hough 线段

![轴向 Hough 线段](/images/visual/current-visual-flow/13_execution_hough_lines.png)

二值图细化后运行 `HoughLinesP`，只保留接近水平和垂直的线段。扫描分支不走这条 Hough 链，避免把局部执行逻辑误接回全局建图。

### 4. 交点与执行范围筛选

![交点与执行范围筛选](/images/visual/current-visual-flow/14_execution_intersections.png)

Hough 交点会聚类，再从 `raw_world_coord` 查相机坐标，并转换到 TCP / 虎口局部坐标。只有落在当前执行范围内、能组成完整局部矩阵的点才下发给执行层。

### 5. 运行话题结果参考

![运行话题结果参考](/images/visual/current-visual-flow/15_runtime_result_topics.png)

扫描和执行都会发布可视化结果图。现场调试时，优先看 `/perception/lashing/scan_surface_dp_base_image`、`/perception/lashing/execution_refine_base_image` 和 `/perception/lashing/result_image`。

## 运行 topic 对照

| 目标 | Topic / Service | 说明 |
| --- | --- | --- |
| 扫描视觉触发 | `/web/cabin/start_pseudo_slam_scan` | 前端「触发扫描视觉」使用的 action。 |
| 直接视觉服务 | `/pointAI/process_image` | `request_mode=3` 为扫描，`request_mode=4` 为执行微调。 |
| 扫描底图设置 | `/web/pointAI/set_scan_response_source` | 设置当前单源底图，默认 `depth_gradient`。 |
| 梁筋过滤开关 | `/web/pointAI/set_scan_beam_exclusion` | 启用后按梁筋候选扩张 mask 做点级过滤。 |
| 梁筋过滤半径 | `/web/pointAI/set_scan_beam_exclusion_margin_mm` | 设置梁筋候选左右扩张半径，默认 150 mm。 |
| 扫描诊断图 | `/perception/lashing/scan_surface_dp_base_image` | 当前底图、物理线族交点和梁筋候选红带。 |
| 执行诊断图 | `/perception/lashing/execution_refine_base_image` | Hough 二值和执行候选诊断。 |
| 结果图 | `/perception/lashing/result_image` | 当前视觉最终结果图。 |
| 相机点 | `/perception/lashing/points_camera` | 原始相机坐标语义，下游再做 TF 转换。 |

## 复现与更新图片

本页图片来自仓库内现有现场快照和诊断输出。重新采集现场样例后，可以先生成报告，再把新的关键图片同步到帮助站资源目录：

```bash
python3 src/tie_robot_perception/tools/current_scan_all_sources_report.py --output-dir .debug_frames/current_scan_all_sources_live
python3 src/tie_robot_perception/tools/build_current_visual_recognition_flow_page.py
```

帮助站图片目录：

```text
src/tie_robot_web/help/public/images/visual/current-visual-flow/
```
