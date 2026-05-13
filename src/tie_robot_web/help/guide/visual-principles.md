# 视觉原理

## 当前认可方案

当前视觉分成两条运行分支：

- 扫描建图：Surface-DP 单源底图方案，默认使用 `depth_gradient`，由物理钢筋间距和统一物理网格评分约束整张钢筋面。
- 执行微调：平面分割 + Hough 局部视觉，只在逐区到位后为线性模组提供当前区域的可执行点。

两条分支共用相机输入，但语义不同：扫描分支负责生成全局 `pseudo_slam_points.json` 和 `pseudo_slam_bind_path.json`；执行微调分支负责到位后复核局部 `2x2` 执行点。逐步效果图见：[当前视觉流程效果图](./current-visual-flow)。扫描主链细节见：[Surface-DP 当前视觉方案](./surface-dp-depth-gradient)。

历史 `PR-FPRG 透视展开频相回归网格方案` 已保留为对照文档：[PR-FPRG 流程详解](./pr-fprg-workflow)。它不再代表扫描层当前主链。

## 前端如何触发

控制面板里的「触发扫描视觉」会走扫描建图 action，而不是直接调用旧 `/web/pointAI/run_workspace_s2` 话题：

```text
前端按钮 runSavedS2
-> TaskActionController.triggerSavedWorkspaceS2()
-> /web/cabin/start_pseudo_slam_scan action
-> tie_robot_process::run_pseudo_slam_scan
-> /pointAI/process_image request_mode=3
-> MODE_SCAN_ONLY
-> run_manual_workspace_surface_dp_pipeline(publish=true)
-> Surface-DP 单源底图 + 统一物理网格评分
-> /perception/lashing/points_camera
-> /coordinate_point
-> /perception/lashing/result_image
-> /tf surface_dp_bind_point_*
-> pseudo_slam_points.json
-> pseudo_slam_bind_path.json
```

执行层视觉入口仍复用 `/pointAI/process_image`，但使用 `request_mode=4`：

```text
执行到位或单点视觉测试
-> /pointAI/process_image request_mode=4
-> MODE_EXECUTION_REFINE
-> execution_refine_hough.py
-> /perception/lashing/execution_refine_base_image
-> /perception/lashing/points_camera
```

`process_image` 服务里的扫描模式进入 Surface-DP；执行微调进入 Hough 分支。旧 `pre_img()` 不再作为扫描主链前置门控。

## 输入数据

扫描分支依赖 4 类输入：

- IR 图像：用于工作区确认、结果叠加和人工检查。
- `/Scepter/worldCoord/raw_world_coord`：用于 rectified 深度、物理尺度估计和最终点位反查。
- 手动工作区四边形：后端保存为 `corner_pixels`，用于确定透视展开范围。
- 前端视觉调试设置：扫描底图和稳定放行帧数。
- 梁筋过滤设置：默认关闭；启用后使用梁筋候选扩张 mask 做点级过滤。

执行微调分支依赖：

- `/Scepter/worldCoord/world_coord`：上游平面分割后的世界点图，用于 Hough 二值化。
- `/Scepter/worldCoord/raw_world_coord`：候选交点反查原始相机坐标。
- 当前 TCP / 线性模组范围：用于保留可执行范围内的点。

## 扫描算法主链

1. 读取已保存工作区四边形、IR 图和 `raw_world_coord`。
2. 根据四边形和角点世界坐标构建透视展开几何。
3. 在 rectified 平面里生成当前选中的单一响应图，默认 `depth_gradient`。
4. 阈值化和骨架化只作为诊断，不直接把 skeleton junction 当绑扎点。
5. 在响应图上按 120-160 mm 钢筋间距寻找横纵线族。
6. 用统一物理网格评分校对线族：线数比例要匹配当前有效视野的物理长宽比，而不是强行要求横纵线数接近 `1:1`。
7. 使用 Surface-DP 沿响应图追踪曲线线族，处理轻微弯曲和局部响应弱化。
8. 求两组物理线族交点，通过 inverse H 投回原图。
9. 从 `raw_world_coord` 查找相机系三维坐标并发布点、结果图和 TF。

梁筋候选会继续作为扫描诊断显示：`beam_candidate_bands` 在扫描底图上画成红色竖带。前端启用梁筋过滤后，PointAI 按设置半径扩张候选 mask，只删除落入该 mask 的交点；不会因为同一 X 位置有一个点命中，就连带移除其它普通钢筋交点。

## 校对方案

当前校对不是「纵向钢筋数量必须等于横向钢筋数量」。统一评分看的是候选网格和当前可见钢筋面的物理一致性：

- 线距必须落在 120-160 mm 对应的像素范围内。
- 横纵线族都至少有 2 根线，且弱但规律的线允许通过低阈值召回。
- 候选线数比例使用 `(纵向间隔数 / 横向间隔数)` 计算。
- 该比例会和 rectified 有效 mask 的物理宽高比比较。
- 误差小于动态 tolerance 才通过；小视野线数少时 tolerance 会自然放宽。

所以 `3 m x 5 m` 这类长方形钢筋面不会因为不是正方形而被杀掉。真正会被拒绝的是「物理视野看起来接近正方形，但线族结果却像 `16 x 2`」这类明显假阳。

关键诊断字段：

```text
physical_lattice_score
physical_lattice_count_aspect
physical_lattice_visible_aspect
physical_lattice_count_aspect_error
physical_lattice_count_aspect_tolerance
physical_prior_modes = [unified_physical_lattice, unified_physical_lattice]
```

现场判断时优先看 `physical_lattice_count_aspect_error <= physical_lattice_count_aspect_tolerance`。如果错误超过 tolerance，说明线数比例和当前可见物理视野不一致。

## 代码落点

前端触发代码：

- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`：控制面板按钮文案。
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`：提交四边形、触发扫描视觉和执行层视觉入口。
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`：下发视觉调试设置和调用 `process_image`。
- `src/tie_robot_web/frontend/src/config/visualRecognitionMode.js`：扫描底图选项与 `request_mode` 常量。
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`：IR 图像选点、拖拽四边形、显示结果覆盖层。

后端实现代码：

- `src/tie_robot_perception/scripts/pointai_node.py`：pointAI ROS 节点可执行入口。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/node.py`：pointAI 节点实现。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`：视觉 topic、service 和 publisher 装配。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`：扫描入口，当前优先调用 Surface-DP。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`：Surface-DP 单源底图、物理网格评分和物理线族交点。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`：执行微调 Hough 分支。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`：结果图渲染与发布。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`：`process_image` 主入口，按 `request_mode` 分流扫描和执行微调。

## 不要退回的旧做法

以下做法都视为退化，不应再恢复：

1. 扫描层回到旧 `pre_img()` 或 RANSAC + Hough 直接出点。
2. 把 skeleton junction 原始交点直接写入扫描账本。
3. 只凭一维 profile 或单轴强响应铺满整张网格。
4. 用横纵线数接近 `1:1` 的硬门槛判断钢筋面。
5. 按小视野、大视野、长方形、正方形拆分多套校对逻辑。
6. 修改显示层时忘记把 rectified 线和点逆投影回原图。
