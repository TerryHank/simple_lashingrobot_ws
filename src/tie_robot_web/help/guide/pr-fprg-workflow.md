# PR-FPRG 识别流程详解

本页说明当前绑扎点识别链路如何从相机图像一步步变成 12 个绑扎点。核心口径是：

- 不再按整张工作区铺满网格。
- 不再假设钢筋一定与画面水平/垂直；钢筋在地面上本身斜摆时，按钢筋真实方向估计线族。
- 候选线必须同时满足「一维峰值」和「二维连续钢筋条」。
- 13、14、15 这类底部垫高件或起子造成的伪点，优先在「二维连续钢筋条验证」删除。
- 「主间距一致性裁剪」保留为最后兜底，不作为起子伪线的主要过滤层。
- 红外图像用于显示和人工核对；正式运行优先使用 `raw_world_coord` 深度响应，响应图梯度只作为 `theta` 候选方向先验，最终线仍要通过 PR-FPRG 峰值与二维连续 ridge 验证。
- 旧 `RANSAC + Hough + pre_img` pointAI 识别链路已经归档，不参与当前运行主链。

目标版本流水线是：

```text
透视展开工作区
-> 深度响应图
-> 自动估计两组钢筋真实方向 theta1 / theta2
-> 沿各自法线方向生成 profile
-> 在 rho 轴上找周期峰值
-> 每条钢筋线表示为 (theta, rho)
-> 连续钢筋条验证沿 theta 方向采样
-> ridge 判断沿法线方向检查「中间强、两侧弱」
-> 两组斜线求交点
-> 逆透视投回原图
```

## 总流程图

![PR-FPRG 总流程](/images/visual/pr-fprg-workflow.png)

这张图对应当前工程链路：

```text
/web/pointAI/run_workspace_s2
-> pointAI.manual_workspace_s2_callback()
-> run_manual_workspace_s2_pipeline(publish=true)
-> /pointAI/manual_workspace_s2_result_raw
-> /pointAI/manual_workspace_s2_points
-> /coordinate_point
-> /tf pr_fprg_bind_point_*
```

## 当前帧最终结果

![当前帧 PR-FPRG 最终结果](/images/visual/pr-fprg-result.png)

这张图由当前相机现场帧重新生成，来源是正式同步的 `/Scepter/worldCoord/raw_world_coord`、`/Scepter/worldCoord/world_coord` 和 `/Scepter/ir/image_raw`，不是旧轴向截图。当前帧识别结果为：

- 响应来源：`depth_background_minus_filled`
- 第一组线族角度：`88°`，`rho = [-262, -172, -84]`
- 第二组线族角度：`178°`，`rho = [-297, -206, -121, -14]`
- 绑扎点数量：`12`

当前帧里钢筋方向接近 rectified 平面的横/竖方向，所以结果看起来接近水平/垂直；如果钢筋在地面上本身斜摆，`line_angle_deg` 会随钢筋真实方向变化，逆透视回原图后也会沿着钢筋走向画线。

## 每一步图像处理

### 1. 同步当前帧与工作区

![1. 同步当前帧与工作区](/images/visual/pr-fprg-steps/pr-fprg-step-01_input_workspace.png)

输入来自 3 个地方：

- `/Scepter/ir/image_raw`：用于显示、叠加和人工检查。
- `/Scepter/worldCoord/raw_world_coord`：保存每个像素对应的相机系 `x/y/z`。
- `manual_workspace_quad.json`：保存手动工作区四边形像素点。

这一步只确定「在哪块区域里识别」，不做绑扎点判断。

### 2. 透视展开工作区

![2. 透视展开后的 IR 工作区](/images/visual/pr-fprg-steps/pr-fprg-step-02_rectified_ir.png)

把原图中的四边形工作区映射成规则矩形：

```text
source_points = 原图四边形角点
destination_points = rectified 矩形角点
forward_h = cv2.getPerspectiveTransform(source_points, destination_points)
inverse_h = cv2.getPerspectiveTransform(destination_points, source_points)
```

后续周期、线和交点都先在 rectified 平面里处理。透视展开负责消除相机视角造成的投影变形；如果钢筋在地面上本身就是斜摆，后续的方向自适应线族会按真实钢筋方向继续识别。

### 3. 展开深度并填补无效值

![3. 展开并填补后的深度](/images/visual/pr-fprg-steps/pr-fprg-step-03_rectified_depth.png)

从 `raw_world_coord[:, :, 2]` 取原始深度 `z`，同时生成有效深度 mask。透视展开后，如果某些像素无效，会用当前有效区域深度中位数填补。

这一步的目的不是直接找线，而是把深度图整理成可做背景差分的规则矩形图。

### 4. 生成钢筋凸起响应图

![4. 深度背景差响应图](/images/visual/pr-fprg-steps/pr-fprg-step-04_depth_response.png)

当前主要响应来自深度背景差：

```text
background_depth = GaussianBlur(filled_depth)
response = background_depth - filled_depth
response = normalize(response, valid_mask)
```

直观理解：钢筋条相对板面有高度变化，经过背景平滑后会在响应图中形成线状亮带。后面的峰值、连续线和间距裁剪都基于这张响应图。

### 5. 生成方向 `theta/rho` profile

![5. 第一组 theta/rho profile](/images/visual/pr-fprg-steps/pr-fprg-step-05_vertical_profile.png)

![6. 第二组 theta/rho profile](/images/visual/pr-fprg-steps/pr-fprg-step-06_horizontal_profile.png)

当前主链会扫描候选钢筋方向 `theta`，把二维响应图沿该方向的法向 `rho` 压缩成一维 profile：

```text
normal(theta) = [-sin(theta), cos(theta)]
rho = normal_x * x + normal_y * y
profile_theta[rho] = response 在同一 rho bin 内的平均值
```

profile 的峰值表示「这条 `rho` 线整体更像钢筋条」。当前版本还会从响应图梯度里提取方向先验，把真实钢筋方向及其正交方向放进候选池；这不是用 Hough 出点，最终线仍由 PR-FPRG 的峰值和连续 ridge 决定。

这样钢筋网只要在 rectified 平面内仍是两组近似平行线，就可以识别；它不要求线条刚好平行于图像 X/Y 轴。但单独看 profile 不够，因为局部杂物也可能把某条 `rho` 线抬成峰值，所以后面还要做二维连续性验证。

### 6. 频相回归生成候选网格

![7. 初始频相候选线](/images/visual/pr-fprg-steps/pr-fprg-step-07_initial_frequency_phase_grid.png)

对每个方向的 profile 做平滑和中心化后，遍历候选周期并计算自相关分数，得到主周期；再遍历相位，取平均响应最高的相位。候选方向先经过响应图方向先验和正交补全，再从候选池里选择两组近似正交、连续性更好的线族：

```text
theta_family = argmax score(profile_theta)
period = argmax corr(profile_theta[t], profile_theta[t + period])
phase = argmax mean(profile_theta[phase::period])
line_rhos = rho_min + phase + k * period
```

这一步只生成「可能的」周期候选线，还不是最终绑扎点。

### 7. 局部峰值修正与峰值支撑筛选

![8. 峰值支撑后的候选线](/images/visual/pr-fprg-steps/pr-fprg-step-08_peak_supported_lines.png)

候选 `rho` 线会在邻域内吸附到局部响应峰，然后做峰值支撑筛选：

- 候选线附近必须有足够强的一维 profile 峰值。
- 太弱的周期线会被删掉。
- 同一根钢筋附近重复候选会做非极大值抑制。

这一步解决「不要整张图按周期铺满」的问题，但还不能完全删除底部伪线，因为底部垫高件也可能有强峰值。

### 8. 二维连续钢筋条验证

![9. 二维连续钢筋条验证](/images/visual/pr-fprg-steps/pr-fprg-step-09_continuous_line_check.png)

对每条候选线，在二维响应图上沿线方向采样，并把线长切成多个段。同时检查每个采样点的横截面是不是「中间强、两侧弱」且贴近候选线中心的 ridge，而不是贴边的一整片宽亮带：

```text
for segment in line_segments:
    ridge_like = 强响应 and 中心 ridge 两侧有落差 and ridge 偏移不能离候选线太远
    strong_fraction = ridge_like 采样数 / 该段有效采样数
line_supported = 支撑段比例 >= min_segment_coverage
```

真实钢筋条应该在多数段上都有线状响应，并且横截面像一根窄钢筋。只有局部亮块、局部阴影、起子、垫高件或斜穿过交叉点的伪线会在这里被删除。

当前帧中，峰值阶段第一组曾多出一条靠边候选线：

```text
peak_supported.family_0 = [-262, -172, -84, 12]
continuous.family_0 = [-262.7, -172.0, -83.7]
```

也就是说，靠边伪线没有等到距离筛选，而是在连续钢筋条验证阶段已经被删掉。

### 9. 主间距一致性裁剪

![10. 主间距一致性裁剪](/images/visual/pr-fprg-steps/pr-fprg-step-10_spacing_pruned_lines.png)

绿色线是保留线。如果连续验证之后仍有靠边伪线，距离筛选会把不符合主网格间距的线标红删除；当前帧里的靠边伪线已经在上一步删除，所以这里没有额外红线。

当前裁剪逻辑是：

```text
diffs = 相邻线间距
reference_spacing = 较大半区间距的中位数
if gap < reference_spacing * 0.65:
    删除更像边缘伪线的一条
```

因此，距离筛选现在作为兜底保险：正常情况下起子伪线先由二维连续 ridge 验证删除；若有残留，再由主间距一致性删除。最终只保留真实钢筋线族。

### 10. 两组线族求交与逆透视映射

![11. 两组线族求交后的最终结果](/images/visual/pr-fprg-steps/pr-fprg-step-11_final_result.png)

最终保留的两组线族使用线方程求交：

```text
normal_a . point = rho_a
normal_b . point = rho_b
rectified_intersection = solve([normal_a; normal_b], [rho_a, rho_b])
```

再用 `inverse_h` 投回原始 IR 图像像素：

```text
image_points = cv2.perspectiveTransform(rectified_intersections, inverse_h)
```

这就是结果图上看到的黄色编号点。

### 11. 从原始世界坐标取点

每个像素交点会去 `/Scepter/worldCoord/raw_world_coord` 中取相机系三维坐标：

```text
camera_xyz = raw_world_coord[pixel_y, pixel_x, :3]
```

如果该像素无效，会在小邻域内找最近有效值。当前 pointAI 发布的点保持相机坐标系语义，前端 3D Scene 再通过 TF 投到全局显示层。

## 代码落点

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`：运行时主链，负责准备输入、调用算法、发布结果。
- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`：方向线族估计、周期估计、峰值筛选、连续线验证、间距裁剪和线族求交。
- `src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py`：独立探针，可抓取当前帧并导出本页使用的逐步图像。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`：把最终线和点渲染到结果图。

## 独立复现命令

启动相机链后运行：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py \
  --timeout 30 \
  --docs-assets-dir src/tie_robot_web/help/public/images/visual/pr-fprg-steps
```

输出内容：

- `.debug_frames/pr_fprg_peak_supported_probe_*/summary.json`
- `.debug_frames/pr_fprg_peak_supported_probe_*/steps/*.png`
- `src/tie_robot_web/help/public/images/visual/pr-fprg-steps/*.png`
