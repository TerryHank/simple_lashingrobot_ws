# 视觉原理

## 当前工作区 S2 方案

当前帮助站、前端选点页、运行时扫描识别里提到的手动工作区 `S2`，统一指：

- `PR-FPRG 透视展开频相回归网格方案`
- `PR-FPRG = Projective-Rectified Frequency-Phase Regression Grid`

这是当前认可的工作区识别主方案。

## 当前方案效果图

下图是当前 `PR-FPRG` 在现场运行时的一次识别结果。图中的网格线和交点，不是在原图里直接硬画出来的，而是：

- 先对工作区做透视展开
- 在 rectified 工作区里做频域周期检测与相位回归
- 再通过 inverse projective mapping 投回原图

![PR-FPRG 现场结果图](/images/visual/pr-fprg-result.png)

这张图对应的现场结果目录是：

- `/.debug_frames/manual_workspace_s2_runtime_projective_fix/result.png`

对应结果摘要里，这次现场识别结果为：

- `point_count = 272`

## 当前方案效果图

下图是当前 `PR-FPRG` 在现场运行时的一次识别结果。图中的网格线和交点，不是在原图里直接硬画出来的，而是：

- 先对工作区做透视展开
- 在 rectified 工作区里做频域周期检测与相位回归
- 再通过 inverse projective mapping 投回原图

![PR-FPRG 现场结果图](/images/visual/pr-fprg-result.png)

这张图对应的现场结果目录是：

- `/.debug_frames/manual_workspace_s2_runtime_projective_fix/result.png`

对应结果摘要里，这次现场识别结果为：

- `point_count = 272`

## 为什么不是原图直接画网格

钢筋工作区在原始 IR 图像里往往是梯形或任意四边形，如果直接在原图里：

- 取工作区 bbox
- 做周期估计
- 直接画横线和竖线

那么透视畸变会把真实钢筋间距拉坏，结果通常会表现为：

- 主周期漂移
- 线与真实钢筋位置偏差大
- 网格更像“图像格子”，不像“工作平面里的真实钢筋网格”

所以当前方案先做几何校正，再做网格估计。

## 当前方案主链

### 1. 手动选四边形工作区

前端用户先在 IR 图像上点出 4 个角点，形成手动工作区四边形。

后端保存：

- `corner_pixels`
- `corner_world_cabin_frame`

其中：

- `corner_pixels` 用来描述图像上的四边形
- `corner_world_cabin_frame` 用来描述该四边形在全局工作坐标里的真实几何尺度

### 2. 透视展开

识别时，先把这个四边形工作区展开成规则平面：

- 优先使用 `corner_world_cabin_frame` 来确定 rectified 工作区的真实宽高
- 只有在世界角点不存在时，才退回到图像像素边长

这样 rectified 图里的钢筋间距才更接近真实工作面上的周期。

### 3. 频域周期检测 + 相位回归

在 rectified 工作区里：

- 分别计算纵向和横向的 profile
- 用 `FFT / 自相关` 估主周期
- 再做相位回归，反推出整张规则网格

这一步是当前方案的核心，所以名称里会强调：

- `Frequency`
- `Phase Regression`

### 4. 结合谷底/线响应稳住网格

钢筋在高度图或响应图里，通常表现为更稳定的线状谷底或线响应。  
当前方案在规则平面里估完周期后，会结合这些响应来稳定最终网格，而不是只靠纯几何周期硬铺满全图。

### 5. 投回原图

最终显示在前端和调试图上的线与交点，不是直接拿 rectified 图结果完事，而是会：

- 通过 inverse projective mapping
- 再投回到原始 IR 图像

所以你在原图上看到的线，会跟着工作区四边形几何走，而不是死板的图像横竖线。

## 从相机 SDK 平面拟合到 `pre_img()` 门控的原理链

当前系统不是“相机一出图就直接走同一条识别链”，而是先经过一层上游几何清理，再分流到不同视觉入口。

### 1. 相机 SDK 上游先做平面拟合

当前上游世界点处理在：

- `src/tie_robot_perception/src/perception/scepter_world_coord_processor.cpp`

这里会先把原始深度图转成三维点，再做一次基于 `RANSAC` 的平面拟合：

- `SACMODEL_PLANE`
- `SAC_RANSAC`

拟合出的主平面内点会被剔掉，剩下的非平面点再重投影回二维图，发布成：

- `world_coord`

同时，未经平面剔除的原始世界点会保留并发布成：

- `raw_world_coord`

可以把它理解成：

- `raw_world_coord`：原始世界点
- `world_coord`：去掉主平面后的“残差世界点”

### 2. `pointAI` 同时持有两套世界点

当前 `pointAI` 会同时使用：

- 原始图像
- `raw_world_coord`
- `world_coord`

其中：

- `raw_world_coord` 更适合做手动工作区 `PR-FPRG`
- `world_coord` / 深度残差链更接近旧的局部检测门控

### 3. `pre_img()` 是旧局部视觉链的门控入口

当前旧的局部视觉识别主入口是：

- `pre_img()`

它做的事情包括：

- 取图像和世界点通道
- 应用检测遮挡
- 对深度残差做二值化、细化、霍夫线提取
- 交点聚类
- 回查世界坐标
- 最终筛成可下游使用的点集

这条链本质上还是“局部矩阵 / 局部绑扎点门控”思路。

### 4. 为什么扫描层不再先走 `pre_img()`

当前扫描层的 `scan_only` 已经被改成：

- 先直接尝试 `PR-FPRG`
- 不再要求先由 `pre_img()` 出点成功后，才允许继续

原因是：

- `PR-FPRG` 的目标是做整块手动工作区的规则网格恢复
- 它依赖的是“工作区透视展开 + 频域周期检测 + 相位回归”
- 不是旧的局部矩阵门控逻辑

所以当前系统里：

- 扫描层：优先走 `PR-FPRG`
- 执行层局部微调：仍然可以继续走原来的局部门控视觉链

### 5. 这条链路的核心分工

可以把当前视觉原理简单理解成：

1. 相机 SDK 上游先做主平面剔除，提供两套世界点
2. 旧链 `pre_img()` 负责局部矩阵 / 局部门控
3. 新链 `PR-FPRG` 负责手动工作区整块规则网格恢复
4. 扫描层和执行层不要再混用同一条识别语义

这也是为什么当前帮助站要把 `PR-FPRG` 单独命名，而不是继续把它混在旧 `pre_img()` 逻辑里讲。

## 从相机 SDK 平面拟合到 `pre_img()` 门控的原理链

当前系统不是“相机一出图就直接走同一条识别链”，而是先经过一层上游几何清理，再分流到不同视觉入口。

### 1. 相机 SDK 上游先做平面拟合

当前上游世界点处理在：

- `src/tie_robot_perception/src/perception/scepter_world_coord_processor.cpp`

这里会先把原始深度图转成三维点，再做一次基于 `RANSAC` 的平面拟合：

- `SACMODEL_PLANE`
- `SAC_RANSAC`

拟合出的主平面内点会被剔掉，剩下的非平面点再重投影回二维图，发布成：

- `world_coord`

同时，未经平面剔除的原始世界点会保留并发布成：

- `raw_world_coord`

可以把它理解成：

- `raw_world_coord`：原始世界点
- `world_coord`：去掉主平面后的“残差世界点”

### 2. `pointAI` 同时持有两套世界点

当前 `pointAI` 会同时使用：

- 原始图像
- `raw_world_coord`
- `world_coord`

其中：

- `raw_world_coord` 更适合做手动工作区 `PR-FPRG`
- `world_coord` / 深度残差链更接近旧的局部检测门控

### 3. `pre_img()` 是旧局部视觉链的门控入口

当前旧的局部视觉识别主入口是：

- `pre_img()`

它做的事情包括：

- 取图像和世界点通道
- 应用检测遮挡
- 对深度残差做二值化、细化、霍夫线提取
- 交点聚类
- 回查世界坐标
- 最终筛成可下游使用的点集

这条链本质上还是“局部矩阵 / 局部绑扎点门控”思路。

### 4. 为什么扫描层不再先走 `pre_img()`

当前扫描层的 `scan_only` 已经被改成：

- 先直接尝试 `PR-FPRG`
- 不再要求先由 `pre_img()` 出点成功后，才允许继续

原因是：

- `PR-FPRG` 的目标是做整块手动工作区的规则网格恢复
- 它依赖的是“工作区透视展开 + 频域周期检测 + 相位回归”
- 不是旧的局部矩阵门控逻辑

所以当前系统里：

- 扫描层：优先走 `PR-FPRG`
- 执行层局部微调：仍然可以继续走原来的局部门控视觉链

### 5. 这条链路的核心分工

可以把当前视觉原理简单理解成：

1. 相机 SDK 上游先做主平面剔除，提供两套世界点
2. 旧链 `pre_img()` 负责局部矩阵 / 局部门控
3. 新链 `PR-FPRG` 负责手动工作区整块规则网格恢复
4. 扫描层和执行层不要再混用同一条识别语义

这也是为什么当前帮助站要把 `PR-FPRG` 单独命名，而不是继续把它混在旧 `pre_img()` 逻辑里讲。

## 当前方案的关键优势

- 先消除工作区透视畸变，再做网格估计
- 周期估计更稳，不容易被原图形变带偏
- 结果更接近真实钢筋面，而不是图像格子
- 能利用世界角点，把工作区展开尺度拉回真实物理尺寸

## 当前方案明确不允许退回的旧做法

以下做法都视为退化，不应再恢复：

1. 直接在工作区像素 bbox 里估周期
2. 用图像像素边长直接决定 rectified 尺度
3. 在原图里直接铺横竖线
4. 只调参数，不修几何主链

## 方案落点

当前关键实现位于：

- `src/tie_robot_perception/scripts/pointAI.py`
- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`

更详细的交接知识条目见：

- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
