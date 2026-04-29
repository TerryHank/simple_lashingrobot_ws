# 视觉原理

## 当前认可方案

当前前端按钮、扫描层和帮助站里提到的「当前认可 S2」，统一指：

- 中文名：`PR-FPRG 透视展开频相回归网格方案`
- 英文全称：`Projective-Rectified Frequency-Phase Regression Grid`

它不是在原始 IR 图像里直接铺满横竖线，也不再假设钢筋一定与画面水平/垂直。当前版本会先把手动工作区透视展开成规则平面，再在 `theta/rho` 空间估计两组钢筋真实摆放方向的周期线族。候选线必须经过一维峰值、二维连续钢筋条和主间距一致性检查，最后才把线交点投回原图。

完整逐步图像流程见：[PR-FPRG 流程详解](./pr-fprg-workflow)。

## 前端如何触发

新前端的控制面板按钮显示为「触发 PR-FPRG」，内部仍复用历史动作 ID `runSavedS2`，这样可以少动已有 ROS 接口。

触发链如下：

```text
前端按钮 runSavedS2
-> TaskActionController.triggerSavedWorkspaceS2()
-> /web/pointAI/run_workspace_s2 std_msgs/Bool(data=true)
-> pointAI.manual_workspace_s2_callback()
-> run_manual_workspace_s2_pipeline(publish=true)
-> /pointAI/manual_workspace_s2_result_raw
-> /pointAI/manual_workspace_s2_points
-> /coordinate_point
-> /pointAI/result_image_raw
-> /tf pr_fprg_bind_point_*
```

这些点话题表达相机坐标系下的视觉结果；前端 3D 显示层再通过 TF 把它们放到全局场景里。

`process_image` 服务里的扫描、执行微调、绑扎检查等模式现在都以方向自适应 PR-FPRG 作为主视觉入口，不再要求旧 `pre_img()` 出点成功后才放行。

## 当前方案效果图状态

![当前 PR-FPRG 结果](/images/visual/pr-fprg-result.png)

当前帮助站效果图已经用新版探针从现场相机帧重新生成，来源是同步的 `raw_world_coord + world_coord + IR`，不是旧轴向截图。当前帧里钢筋本身接近 rectified 横/竖方向，所以画出来接近水平/垂直；如果钢筋在地面上斜摆，线族角度会随钢筋真实方向变化。

本次截图摘要：

- 响应来源：`depth_background_minus_filled`
- 线族角度：`88° / 178°`
- 线族数量：`3 x 4`
- 绑扎点数量：`12`

方向选择现在会先用响应图梯度做 `theta` 候选先验，再做 PR-FPRG 频相回归、峰值支撑、二维连续钢筋条验证和主间距一致性裁剪。底部原来容易出现的 13、14、15 号伪点应优先在二维连续钢筋条验证阶段删除；主间距一致性裁剪继续作为最后兜底。

## 输入数据

PR-FPRG 依赖 3 类输入：

- IR 图像：用于显示、叠加和人工检查。
- `/Scepter/worldCoord/raw_world_coord`：用于从像素交点反查原始相机系世界坐标。
- 手动工作区四边形：后端保存为 `corner_pixels`，用于确定透视展开范围。

## 算法主链

1. 保存手动工作区四边形，并发布 `/pointAI/manual_workspace_quad_pixels` 给前端确认。
2. 根据四边形构建正向透视矩阵 `forward_h` 和逆向透视矩阵 `inverse_h`。
3. 从 `raw_world_coord[:, :, 2]` 取深度，透视展开并填补无效值。
4. 用背景深度差生成钢筋凸起响应图。
5. 从响应图梯度提取主方向先验，补齐正交方向候选。
6. 扫描候选角度 `theta`，把二维响应图投影到法向 `rho`，得到每个方向的一维 profile。
7. 对每个 profile 做频域周期检测与相位回归，选出两组近似正交且连续性更好的钢筋线族。
8. 把候选 `rho` 线吸附到局部峰值，并删除峰值支撑不足的线。
9. 在二维响应图上沿每条斜线采样，验证它是否是一整条连续钢筋条，并要求横截面呈现贴近候选线中心的 ridge。
10. 按主网格间距兜底删除靠边或由垫高件造成的残留额外伪线。
11. 两组线族用线方程求交点，通过 `inverse_h` 投回原图。
12. 从 `raw_world_coord` 反查每个绑扎点的相机系三维坐标并发布。

## 代码落点

前端触发代码：

- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`：控制面板按钮文案。
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`：提交四边形、触发 PR-FPRG、固定扫描规划入口。
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`：发布 `/web/pointAI/run_workspace_s2`。
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`：触发后切换覆盖层显示、等待 result 图。
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`：IR 图像选点、拖拽四边形、显示结果覆盖层。

后端实现代码：

- `src/tie_robot_perception/scripts/pointai_node.py`：pointAI ROS 节点可执行入口。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/node.py`：pointAI 节点实现，订阅 `/web/pointAI/run_workspace_s2`。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`：保存与发布手动工作区四边形。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`：PR-FPRG 运行时主链。
- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`：方向线族估计、周期估计、峰值筛选、连续线验证、间距裁剪和线族求交。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`：结果图渲染与发布。
- `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`：`process_image` 主视觉入口，统一调用 PR-FPRG。
- `src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py`：独立探针，可导出逐步处理图片。

## 旧 RANSAC + Hough 已归档

相机上游世界点处理在：

- `src/tie_robot_perception/src/perception/scepter_world_coord_processor.cpp`

这里会把原始深度图转成三维点，再用 `SACMODEL_PLANE` 和 `SAC_RANSAC` 拟合主平面。系统同时保留两套世界点：

- `raw_world_coord`：未经过主平面剔除的原始世界点。
- `world_coord`：去掉主平面后的残差世界点。

PR-FPRG 更依赖 `raw_world_coord`，因为它需要从规则网格交点反查真实相机系坐标。

旧 `pre_img()` 绑扎点识别链路曾经负责深度二值化、细化、霍夫线提取、交点聚类和可执行范围过滤。现在这套 `RANSAC + Hough + pre_img` pointAI 链路已经从 active package 中移除，归档在：

- `docs/archive/legacy_ransac_hough_pointai/`

当前分工是：

1. 相机上游继续提供 `raw_world_coord` 和 `world_coord`。
2. 方向自适应 PR-FPRG 负责 pointAI 绑扎点主视觉识别。
3. 旧 `pre_img()` 归档保留用于追溯，不参与 ROS 节点运行。

注意：上游 `scepter_world_coord_processor.cpp` 中的 PCL `SAC_RANSAC` 平面处理属于世界坐标生成链路，不等同于已经归档的旧 pointAI RANSAC+Hough 绑扎点识别。

## 不要退回的旧做法

以下做法都视为退化，不应再恢复：

1. 直接在工作区像素 bbox 里估周期。
2. 在原图里直接铺满横竖线。
3. 只凭一维 profile 峰值生成整张网格。
4. 把方向自适应线族退回固定 X/Y 方向。
5. 把 PR-FPRG 重新挂到旧 `pre_img()` 或 Hough 结果之后。
6. 修改显示层时忘记把 rectified 线和点逆投影回原图。
