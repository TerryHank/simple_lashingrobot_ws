# 2026-04-23 PR-FPRG 透视展开频相回归网格方案知识条目

## 1. 方案名称

- 中文名：`PR-FPRG 透视展开频相回归网格方案`
- 英文代号：`PR-FPRG`
- 英文全称：`Projective-Rectified Frequency-Phase Regression Grid`

后续如果再提“当前认可的 S2”，统一指这一个方案，不再使用模糊说法。

---

## 2. 方案本质

这版方案的核心不是“在原图里画横竖线”，而是：

1. 先把手动工作区四边形透视展开到规则平面
2. 在展开平面里做频域周期检测与相位回归
3. 结合工作区内的谷底/线响应，回归整张钢筋网格
4. 最后把线和交点逆投影回原图

也就是说，这版方案的主骨架是：

- `FFT / 自相关` 估主间距
- `相位回归` 反推整张网格
- `透视展开` 保证周期估计发生在规则工作平面，而不是发生在畸变原图里

---

## 3. 为什么需要这版方案

之前运行时那条 `manual workspace S2` 存在两个根因问题：

1. 没有先按工作区四边形做规则平面展开
2. 展开尺度错误地使用了图像像素边长，而不是优先使用世界坐标四角的真实尺度

这会导致：

- 周期漂移
- 线和真实钢筋位置偏差很大
- 结果更像“图像里的规则格子”，不像“工作区平面里的真实钢筋网格”

这版 `PR-FPRG` 的目标，就是把运行时 S2 拉回到“工作区规则平面里做频域周期检测与相位回归，再投回原图”的正确几何口径。

---

## 4. 当前方案主链

### 4.1 输入

- IR 图像
- `raw_world_coord`
- 手动工作区四边形：
  - `corner_pixels`
  - `corner_world_map`

### 4.2 处理链

1. 对工作区四边形按顺时针排序
2. 构建工作区的 projective rectified geometry
3. 展开为规则平面工作区
4. 在 rectified 工作区里计算纵横两个方向的 profile
5. 用 `FFT / 自相关` 做主周期估计
6. 做相位回归，求出规则网格线位置
7. 结合工作区内的谷底/线响应稳住网格
8. 将线段和交点通过逆透视映射回原图
9. 输出：
   - 显示层 result
   - 世界点结果

### 4.3 当前关键 helper

当前关键 helper 位于：

- [workspace_s2.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py)

当前关键函数包括：

- `build_workspace_s2_rectified_geometry(...)`
- `estimate_workspace_s2_period_and_phase(...)`
- `build_workspace_s2_projective_line_segments(...)`
- `map_workspace_s2_rectified_points_to_image(...)`

运行时主入口位于：

- [pointai_node.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/scripts/pointai_node.py)
- [pointai/node.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/tie_robot_perception/pointai/node.py)

---

## 5. 当前方案的几何约束

### 5.1 先展开，后估计

当前 S2 必须先在工作区四边形对应的规则平面里做周期估计和网格生成。

不允许：

- 直接拿工作区像素 bbox 做 profile
- 直接在原图里按横竖线生网格

### 5.2 rectified 尺度优先用世界角点

如果已知：

- `corner_world_map`

那么 rectified 工作区的宽高必须优先按世界尺度计算，而不是按像素边长计算。

只有在世界角点不存在时，才允许退回到图像像素边长。

### 5.3 线和点必须投回原图

显示层看见的线和点，不是在 rectified 图里直接画完就结束，而是必须通过 inverse projective mapping 投回原图。

这保证：

- 显示出来的线会跟着四边形几何走
- 不再是“死板的图像横线/竖线”

---

## 6. 当前方案和旧方案的边界

### 6.1 当前认可方案

当前认可的是：

- `PR-FPRG`
- 即 `PR-FPRG 透视展开频相回归网格方案`
- 2026-04-29 起，运行主链为方向自适应版本：透视展开后在 `theta/rho` 空间估计两组钢筋线族，不再默认钢筋与画面水平/垂直。

### 6.2 明确不再认可的旧方案

以下做法视为退化，不应再恢复：

1. 在原图工作区 bbox 里直接估周期
2. 用像素边长直接当 rectified 尺度
3. 在原图里直接铺横竖线
4. 只调参数、不修几何主链
5. 把 `pre_img()`、旧 RANSAC+Hough 线段检测或 `matrix_preprocess.py` 重新接回 pointAI 运行主链

旧 `RANSAC + Hough + pre_img` 绑扎点识别代码已经归档到：

- `docs/archive/legacy_ransac_hough_pointai/`

该归档目录只用于追溯，不参与 ROS 节点运行。

---

## 7. 这版方案的识别结果基准

当前这版方案对应的现场结果目录：

- [manual_workspace_s2_runtime_projective_fix](/home/hyq-/simple_lashingrobot_ws/.debug_frames/manual_workspace_s2_runtime_projective_fix)

关键产物：

- [result.png](/home/hyq-/simple_lashingrobot_ws/.debug_frames/manual_workspace_s2_runtime_projective_fix/result.png)
- [summary.json](/home/hyq-/simple_lashingrobot_ws/.debug_frames/manual_workspace_s2_runtime_projective_fix/summary.json)

这次现场结果记录为：

- `point_count = 272`

这份目录可作为后续回归比对基准。

---

## 8. 回归验证要求

后续如果有人修改这版 S2，至少要通过以下几类验证：

1. rectified geometry 优先使用世界尺度
2. 周期估计不被近邻伪周期轻易带偏
3. projective 线段映射后应跟随四边形几何
4. 运行时 `manual workspace S2` 仍能正常发布结果图和点结果

建议重点关注：

- [test_pointai_order.py](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/test/test_pointai_order.py)

以及现场结果目录：

- [manual_workspace_s2_runtime_projective_fix](/home/hyq-/simple_lashingrobot_ws/.debug_frames/manual_workspace_s2_runtime_projective_fix)

---

## 9. 下一位工程师最容易踩的坑

1. 只改参数，不改几何主链
2. 把 `corner_world_map` 和 `corner_pixels` 排序解绑
3. 把运行时输入退回到未经 rectified 的原图 profile
4. 修改显示结果时忘记 inverse mapping
5. 在 `process_image` 入口重新接回 `pre_img()` 或旧 RANSAC+Hough 回退路径
6. 把线族估计退回固定 X/Y 方向，导致地面上本身斜摆的钢筋网识别漂移

如果后续结果再次出现“线像图像格子、不像真实钢筋网格”，优先检查：

1. 是否仍然先做了 rectified
2. rectified 尺度是否仍优先来自世界角点
3. 周期估计是否仍基于 `theta/rho` 线族扫描、频域/自相关与相位回归
4. 候选线是否仍经过二维连续钢筋条 ridge 验证，而不是只靠一维峰值
5. 线和点是否经过 inverse mapping 投回原图
6. `process_image` 是否仍然直接走方向自适应 `PR-FPRG`，而没有被旧 `pre_img()` 链路重新卡住
