# Agent Session Memory Log

本文件按时间倒序记录跨会话共享记忆。新条目写在最上方，并保留 `AGENT-MEMORY:` 标记，方便脚本识别。

## 2026-05-11 22:53 - 扫描梁筋过滤默认改为15cm

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：用户现场确认同列缺点符合扫描梁筋过滤路径，当前扫描 Surface-DP 在启用 scan_beam_exclusion 时从梁筋候选 band 扩张物理 mask 后过滤最终交点。运行态默认 scan_beam_exclusion_margin_mm 已从 130.0 mm 改为 150.0 mm；诊断键同步为 beam_candidate_15cm_mask / beam_candidate_15cm_pixels，前端视觉调试开关和 ROS 日志文案显示「梁筋 ±15 cm 过滤」。旧 PR-FPRG 研究工具中的 13cm 命名属于历史实验口径，未作为当前运行态入口修改。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_scan_surface_dp_runtime.ScanSurfaceDpRuntimeTest; PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_workspace_s2_expands_beam_mask_by_fifteen_centimeters_for_graph_exclusion src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_manual_workspace_s2_current_chain_rejects_depth_only_fallback src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_manual_workspace_s2_module_omits_later_stability_and_phase_lock_experiments src.tie_robot_perception.test.test_current_visual_recognition_flow_report; node test/visualDebugSettings.test.mjs; npm run build`

### 后续注意

- 暂无。

## 2026-05-11 22:51 - Surface-DP统一物理网格评分

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：扫描层 Surface-DP 不再用横纵线数接近 1:1 的 full_workspace 平衡硬门槛过滤线族；改为统一物理网格评分，用候选线数比例与当前 rectified 有效视野物理长宽比的一致性、弱规则线召回和支持度共同接纳/拒绝。长方形全局网格如 34x21 可通过，正方形视野里的 16x2 假阳仍拒绝；小视野、正方形、长方形共用同一判据，不按视野大小分档。当前工程状态已先保存并推送 tag slam/v44。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -> Ran 34 tests in 39.269s OK; git diff --check -- src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 后续注意

- 暂无。

## 2026-05-11 22:50 - 扫描线族弱响应门限放宽

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：Surface-DP 扫描层在所选单一底图上放宽物理线族弱峰兜底门限：单轴选峰 fallback 从 0.08 放到 0.06，整图物理线族重试增加 0.08/0.06 档；线族评分略降低平均响应权重，提高线数完整度和画幅覆盖权重，便于现场检测弱但规律的横纵线族。仍保留必须同时存在横向线族和纵向线族、且数量比例符合画幅物理宽高的约束，不把单轴结果硬凑成交点。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py; src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -v；python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py；git diff --check -- src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 后续注意

- 暂无。

## 2026-05-11 21:51 - 扫描模式失败当前帧立即返回

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：PointAI 扫描模式 MODE_SCAN_ONLY 在单次 /pointAI/process_image 请求内仍尊重用户设置的 stable_frame_count 释放帧数；但若当前帧 Surface-DP 没有返回有效点，不再循环等待下一帧，而是立即返回当前帧失败原因。Surface-DP 横纵线族不足错误文案改为中文“所选扫描底图横纵线族不足”，避免 completed surface 旧术语误导现场。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py; src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py; src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_all_visual_trigger_modes_wait_for_release_frame_count src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_scan_only_no_points_returns_current_frame_failure_without_waiting_next_frame src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_scan_surface_dp_insufficient_line_family_message_is_chinese_selected_source src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_process_image_wait_loop_routes_scan_to_surface_dp_and_execution_to_hough -v；PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -v；python3 -m py_compile updated files；git diff --check`

### 后续注意

- 暂无。

## 2026-05-11 21:35 - 相机SDK调试页独立中文化

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：设置页把“相机底层 SDK 调试”从视觉调试页拆成独立设置页选项 cameraSdkDebug；面板内参数显示全部中文化，dynamic_reconfigure 仍使用 Scepter ROS cfg 原始英文参数名向 /scepter_manager/set_parameters 下发，避免破坏相机 SDK 接口。

### 影响范围

- `src/tie_robot_web/frontend/src/config/cameraSdkDynamicReconfigure.js; src/tie_robot_web/frontend/src/ui/UIController.js; src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs; src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs；for test_file in src/tie_robot_web/frontend/test/*.mjs; do node "" || exit 1; done；cd src/tie_robot_web/frontend && npm run build；git diff --check`

### 后续注意

- 暂无。

## 2026-05-11 21:19 - 3D显示未入组扫描点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：3D Scene 规划点图层新增未入组扫描点显示。/api/planning/bind-path 返回的全量 grid_points 仍保留；黄色 bindPathPoints 只表示进入 pseudo_slam_bind_path.json 有效分组的可执行点，新增玫红色 unplannedBindPathPoints 表示扫描检测到但未进入任何规划组的点，悬停标签为“未入组扫描点”。行/列线、2x2成组线、跳绑高亮仍只使用已分组点，避免误导执行语义。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/bindPathGeometry.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/test/bindPathGeometry.test.mjs`
- `src/tie_robot_web/frontend/test/scenePointHoverTooltip.test.mjs`
- `src/tie_robot_web/frontend/test/bindPathLayerControls.test.mjs`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/bindPathGeometry.test.mjs && node src/tie_robot_web/frontend/test/scenePointHoverTooltip.test.mjs && node src/tie_robot_web/frontend/test/bindPathLayerControls.test.mjs`
- `for test in src/tie_robot_web/frontend/test/*.mjs; do node "$test" || exit 1; done`
- `npm run build`（在 `src/tie_robot_web/frontend`）

### 后续注意

- 暂无。

## 2026-05-11 21:17 - 扫描层单源底图与相机SDK热调

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：扫描层 Surface-DP 从固定 depth_gradient_only 改为 single_selected_response，可在前端 设置/视觉调试 的 扫描底图 下拉栏选择 fused_instance_response、frangi_like、hessian_ridge、depth_gradient、infrared_response、combined_response、depth_response；运行时只生成被选中的底图并做一次物理线族检测，不再使用 completed_candidates 第二轮。PointAI 订阅 /web/pointAI/set_scan_response_source 并持久化 ~scan_response_source。设置页新增相机底层 SDK 调试面板，按 Sceptertof_roscpp.cfg 参数通过 /scepter_manager/set_parameters dynamic_reconfigure/Reconfigure 热修改，并把参数保存到前端 localStorage 重连回放。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py; src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py; src/tie_robot_web/frontend/src/ui/UIController.js; src/tie_robot_web/frontend/src/controllers/RosConnectionController.js; src/tie_robot_web/frontend/src/config/cameraSdkDynamicReconfigure.js; src/tie_robot_web/help/guide/surface-dp-depth-gradient.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -v; for test_file in src/tie_robot_web/frontend/test/*.mjs; do node "" || exit 1; done; python3 -m py_compile pointai files and current_scan_all_sources_report.py; npm run build in frontend; npm run build in help`

### 后续注意

- 暂无。

## 2026-05-11 21:07 - 扫描图像层收口为单图层

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：扫描视觉图像层只保留 /perception/lashing/scan_surface_dp_base_image，并在前端显示为“扫描识别底图”；移除 scan_surface_dp_completed_surface_image ROS publisher、前端 topic registry/image catalog 入口和当前静态页面选项。后端 publish_scan_surface_dp_base_images 只发布 runtime_response/depth_gradient 单图，继续叠加 DP 交点和梁筋候选诊断。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/config/imageTopicCatalog.js`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_visual_recognition_uses_scan_action_and_result_image_topic src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_image_panel_supports_project_image_topics_and_overlay_switching src.tie_robot_perception.test.test_scan_surface_dp_runtime.ScanSurfaceDpRuntimeTest.test_surface_dp_debug_base_images_overlay_rectified_intersections src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_scan_and_execution_base_images_are_published_for_frontend_image_layer; npm run build`

### 后续注意

- 暂无。

## 2026-05-11 20:58 - 扫描层接入可配置线性误差补偿

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：PointAI 扫描建图链路在 manual_workspace_s2 输出 PointCoords.World_coord 前新增可配置线性相机坐标补偿。默认 scan_linear_compensation_enabled=false 且 x/y 系数为 0，不改变现有扫描输出；启用后按 z 与 reference_z 的差值对相机坐标 x/y 做比例修正：x*=1-kx*(z-z0)，y*=1+ky*(z-z0)，并通过 min_z 与 max_abs_scale_delta 做门控/限幅。补偿只先接入固定识别位姿扫描账本点，不改底层 raw_world_coord 点云和执行微调 Hough 原始取点。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_bringup/launch/algorithm_stack.launch`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:src:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg; py_compile pointai modules passed. test_scan_surface_dp_runtime 现有未完成 response_source 热更新期望仍失败，非本次线性补偿引入。`

### 后续注意

- 暂无。

## 2026-05-11 20:22 - 扫描层视觉收口为深度梯度单源

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-11：扫描层 Surface-DP 运行态统一为 depth_gradient_only，只生成一次 depth_gradient 并在该响应图上找物理线族和做 DP 曲线追踪；深度暗线、红外、组合、Hessian、Frangi、融合实例和旧补全面模态保留为隐藏离线对照源，不再由 build_scan_surface_dp_result 主链生成或调用。current_scan_all_sources_report 改为真实主链跑一次，隐藏源由工具离线生成。帮助站新增 Surface-DP 深度梯度主链中文页面和各源效果图。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py;src/tie_robot_perception/tools/current_scan_all_sources_report.py;src/tie_robot_web/help/guide/surface-dp-depth-gradient.md;src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 运行态只认 depth_gradient，其他响应源不删除但隐藏，文档和离线报告可继续解释和对照。

### 标签

- `vision`
- `surface-dp`
- `depth-gradient`
- `help`

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_scan_surface_dp_runtime -v => Ran 28 tests OK; python3 -m py_compile src/tie_robot_perception/tools/current_scan_all_sources_report.py => OK; git diff --check => OK; npm run build in src/tie_robot_web/help => build complete`

### 后续注意

- 暂无。

## 2026-05-09 17:25 - 3D执行点只显示已分组点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-09：前端 /api/planning/bind-path 会把 pseudo_slam_points.json 的全量 grid_points 附到 bind_path 上，但 3D Scene 的执行/规划点、行列线、跳绑高亮应只使用 pseudo_slam_bind_path.json 中有效分组（至少2点）引用到的 global_idx；未进入分组的扫描网格点不能显示成黄色单点，避免误以为产生了单点执行组。当前账本验证：176个grid_points中112个进入2/4点分组，64个未分组点会被隐藏。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/bindPathGeometry.js`
- `src/tie_robot_web/frontend/test/bindPathGeometry.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `for test_file in src/tie_robot_web/frontend/test/*.mjs; do node "$test_file" || exit 1; done`
- `npm run build`（工作目录：`src/tie_robot_web/frontend`）

### 后续注意

- 暂无。

## 2026-05-09 16:40 - 扫描物理先验取消钢筋数量限制

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-09：扫描层 Surface-DP 物理先验只保留钢筋间距约束（FULL_SCAN_REBAR_SPACING_MM_RANGE=120-160mm），不再使用 full workspace 15-18 条或固定 16 条偏好作为钢筋数量限制。线族数量上限改为当前视野按最小合法间距可容纳的数量，并按实际峰值与间距一致性选择；13x13 等合法间距网格应通过。梁筋候选、梁筋过滤开关和 lattice gate 语义本次不改。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py;src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 标签

- `scan`
- `pointai`
- `surface-dp`
- `spacing`

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; git diff --check -- src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 后续注意

- 暂无。

## 2026-05-09 15:56 - PointAI扫描编号按map最小坐标起排

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-09：PointAI manual_workspace_s2 扫描点返回和结果图显示编号不再按右上角/TCP图像口径排序；每个点优先用 Scepter_depth_frame->map 转换后的坐标排序，按世界 Y 行、世界 X 正向编号，取不到 TF 时回退到原相机坐标。Surface-DP global_row/global_col 元数据继续保留原拓扑语义。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py;src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 标签

- `pointai`
- `scan`
- `world-coordinates`
- `numbering`

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 后续注意

- 暂无。

## 2026-05-09 15:52 - 扫描点按世界最小点重新编号

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-09：扫描代表点在写入 pseudo_slam_points.json、发布 pseudo_slam markers 和进入动态绑扎规划前，会先按 map/world 坐标排序并重新赋 idx/global_idx。排序先按世界 Y 聚行，再在每行按世界 X 正向排列；因此世界坐标最小角点成为 1 号点，不再沿用 PointAI 图像行列或视觉返回顺序。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp;src/tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 标签

- `scan`
- `world-coordinates`
- `pseudo-slam`
- `planning`

### 验证证据

- `catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning; catkin_make tie_robot_process_planning suoquNode; python3 -m unittest src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 后续注意

- 暂无。

## 2026-05-09 15:46 - 3D点悬停高亮反馈

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-09：新前端 3D Scene 的点悬停反馈已改为“点自身高亮”，不再叠加额外覆盖点。鼠标悬停绑扎点、路径规划点、跳绑覆盖点或索驱规划区域中心时，Scene3DView 通过同一 raycaster 命中管线设置该 THREE.Points 几何里的 pointHoverScale / pointHoverColorMix 顶点属性，由 PointsMaterial shader 让被命中的原始点自己变大变亮；移出或未命中时把该点属性恢复。

### 影响范围

- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/test/scenePointHoverTooltip.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-D9nig9ED.js`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && for f in test/*.test.mjs; do node "$f" || exit 1; done; npm run build; git diff --check -- src/tie_robot_web/frontend/src/views/Scene3DView.js src/tie_robot_web/frontend/test/scenePointHoverTooltip.test.mjs docs/agent_memory/current.md docs/agent_memory/session_log.md`

### 后续注意

- 暂无。

## 2026-05-09 15:26 - 演示模式只管本工程服务

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-09：用户明确要求新前端 header 的“演示模式”只关闭当前 /home/hyq-/simple_lashingrobot_ws 工程相关进程和后台服务，进入后按钮变绿；不得再启动、停止、清理或跳转 /home/hyq-/lashingrobotROS 或 /home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws 的任何内容。当前实现把演示模式收口为停止本工程 tie-robot-backend.service、tie-robot-rosbridge.service 与三个 driver service，只按当前工作区路径清理残留 ROS 进程；退出时按当前工程依赖顺序恢复 rosbridge、三个 driver 和 backend。旧 demo rosbridge/show_full unit 与 launch 模板已删除，frontend autostart 不再安装旧前端或 demo 模式服务。

### 影响范围

- `README.md`
- `CHANGELOG.md`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/config/systemControlCatalog.js`
- `src/tie_robot_web/frontend/src/controllers/SystemControlController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_bringup/scripts/install_demo_mode_service.sh`
- `src/tie_robot_bringup/scripts/install_frontend_autostart.sh`
- `src/tie_robot_bringup/systemd/tie-robot-backend-control.sudoers.in`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_demo_mode_only_quiets_current_workspace_services src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_demo_mode_toggle_stops_current_services_and_restores_without_bridge src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_frontend_assets_exist; node test/systemControlCatalog.test.mjs; node test/statusChipPressBehavior.test.mjs; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; npm run build`

### 后续注意

- 暂无。

## 2026-05-08 16:43 - 跳绑微调撤回当前区域2x2轮询

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-08：用户明确撤回 planned_path_refine_only 跳绑开启后每到一区域必须原地轮询直到视觉返回最靠近虎口原点2x2的语义。当前代码在跳绑微调准备点阶段只请求一次 /pointAI/process_image；若视觉未返回可执行点、map/TCP转换失败、最近账本点匹配失败或跳绑过滤后无点，则记录原因、清空当前区域执行标记并跳过当前区域，不停留当前区域反复请求。保留当前区域最近账本点匹配、黑白棋过滤和实时视觉坐标执行逻辑。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_scan_artifact_write_guard; python3 -m unittest discover -s src/tie_robot_process/test -p 'test_*.py'; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process; git diff --check -- src/tie_robot_process/src/suoquNode.cpp src/tie_robot_process/test/test_scan_artifact_write_guard.py CHANGELOG.md`

### 后续注意

- 暂无。

## 2026-05-08 16:34 - 跳绑微调回退为当前区域最近账本点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-08：用户发现开启跳绑后稳定2x2视觉仍反复卡住，根因是 planned_path_refine_only 跳绑分支还用当前区域四宫格/边界完整性对账拒绝视觉结果。当前口径已覆盖四宫格方案：视觉点转换到map与gripper_frame后，只在当前pseudo_slam_bind_path区域groups[].points[]内按三维欧式距离匹配最近账本点，继承该账本点jump_bind、checkerboard_color、全局行列等元数据，再按黑/白棋过滤；执行坐标继续使用实时视觉世界坐标和TCP局部坐标。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; source /opt/ros/noetic/setup.bash && source devel/setup.bash && cmake --build build --target suoquNode -- -j2; git diff --check`

### 后续注意

- 暂无。

## 2026-05-08 16:17 - 视觉调试页移除触发按钮与2x2重试诊断

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-08：视觉调试设置页删除“触发视觉服务”按钮，设置页只保留参数热更新；释放帧数、索驱规划Z下限、自适应分组、梁筋过滤、线模绑扎范围输入变化统一走 applyVisualDebugRuntimeSettings，立即写 tie_robot_frontend_visual_debug_settings、刷新3D/IR范围，并在ROS连接时发布 stable_frame_count、execution_refine_tcp_roi、scan_beam_exclusion。注意：本条里 planned_path_refine_only 跳绑微调轮询诊断已被 2026-05-08 16:43“跳绑微调撤回当前区域2x2轮询”覆盖，当前失败后跳过当前区域，不再原地轮询。修改前端后已重建 src/tie_robot_web/web，并清理旧未引用 index 哈希产物。

### 影响范围

- `CHANGELOG.md;src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js;src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs;src/tie_robot_web/web/index.html;src/tie_robot_process/src/suoquNode.cpp`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `visual-debug`
- `execution-refine`

### 验证证据

- `node test/visualDebugSettings.test.mjs; for test_file in test/*.test.mjs; do node "" || exit 1; done; node --check src/app/TieRobotFrontApp.js && node --check src/ui/UIController.js; source /opt/ros/noetic/setup.bash && source devel/setup.bash && cmake --build build --target suoquNode -- -j2; npm run build; rg confirms no visualDebugTrigger/触发视觉服务 in web runtime; git diff --check; python3 scripts/agent_memory.py check`

### 后续注意

- 暂无。

## 2026-05-08 15:59 - 账本最小2点组与线模工作中心对齐落点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 动态绑扎账本生成已收口：规划层归一化请求点数时最小为2，非自适应大组若会留下1个尾点则退回较小可达组，例如5点请求遇到6个连续点输出4+2而不是5+1；2点请求遇到单个尾点时保留可执行2点组但不输出孤点。索驱cabin_pose仍通过gripper/base_link TF反算，使组内世界点中心对齐线性模组工作范围中心tcp_max_x/y/z的一半，而不是base_link中心，并新增2点边缘组中心对齐测试。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 账本生成禁止1点组成为代码级约束；外部传入每组1点会归一化为2点。
- 索驱路径规划参考点继续使用线性模组工作范围中心，由TF把该工作中心从gripper_frame换算到base_link/map关系后反算cabin_pose。

### 标签

- `memory`
- `ledger`
- `planning`
- `tf`

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && devel/lib/tie_robot_process/test_dynamic_bind_planning：35/35 passed`
- `source /opt/ros/noetic/setup.bash && cmake --build build --target suoquNode -- -j2：Built target suoquNode`

### 后续注意

- 暂无。

## 2026-05-08 01:58 - 账本生成禁止单点组

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确新口径：生成或重生成绑扎账本、路径区域和执行分组时，不允许出现孤立单点；最小有效成组规模是 2 个点。边缘剩余点也应合并、补组、等待或提示处理，不能落成 1 点组。

### 影响范围

- `docs/agent_memory/session_log.md`
- `docs/agent_memory/current.md`

### 关键决策

- 账本生成规则新增硬约束：禁止输出 1 点组，最小成组为 2 点。

### 标签

- `memory`
- `ledger`
- `planning`

### 验证证据

- `python3 scripts/agent_memory.py refresh`：已刷新 `docs/agent_memory/current.md`。
- `python3 scripts/agent_memory.py check`：输出 `agent memory contract ok`。

### 后续注意

- 后续修改 pseudo_slam 规划、bind_path 账本、执行记忆或动态分组逻辑时，优先检查是否会产生单点区域。

## 2026-05-08 01:36 - 跳绑微调锁定当前区域并轮询2x2

<!-- AGENT-MEMORY: entry -->

### 摘要

- planned_path_refine_only 跳绑开启后，后端不再用视觉点自己的包围盒划分四宫格；现在视觉点必须先落入当前账本区域扩展边界，再按账本区域中心线归入四宫格，防止二区域点混入三区域执行。注意：本条里的“每个区域原地轮询直到完整 2x2”已被 2026-05-08 16:43“跳绑微调撤回当前区域2x2轮询”覆盖，当前代码失败后跳过当前区域，不再停留当前区域继续轮询。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_process/test -p 'test_*.py'; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-08 01:14 - 跳绑微调改为区域四宫格对账

<!-- AGENT-MEMORY: entry -->

### 摘要

- planned_path_refine_only 跳绑开启后不再让执行微调识别点按欧式最近账本点抢配；现在在同一 bind_path 区域内分别用账本 world x/y 与视觉 world x/y 划分 2x2 四宫格，按同宫格继承账本 global_row/global_col/checkerboard_color 等元数据，并使用视觉 TCP 局部坐标执行。账本或视觉不足完整 4 宫格时不下发局部漏绑点，避免只绑一半区域。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_process/test -p 'test_*.py'; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 23:56 - 前端索驱诊断超时窗口对齐

<!-- AGENT-MEMORY: entry -->

### 摘要

- 排查 [前端] 状态变化 chassis -> 索驱状态超时：该日志来自 tie_robot_web/frontend StatusMonitorController 对 /diagnostics 中 tie_robot/chassis_driver 的缓存新鲜度判断，不是索驱 TCP 请求本身的错误。旧 slam/v25 前端只订阅 /robot/chassis_status，没有 3s 诊断 stale 判定；后端索驱 TCP_TIMEOUT_SEC=5，末端 Modbus response timeout=30。当前将索驱/末端 STATUS_MONITORS 增加 diagnosticStaleMs=12000，视觉仍用默认 3000ms，避免一次底层5s等待或短暂重连期间被前端误报超时，同时超过长窗口仍提示状态超时。

### 影响范围

- `src/tie_robot_web/frontend/src/config/statusMonitorCatalog.js;src/tie_robot_web/frontend/src/controllers/StatusMonitorController.js;src/tie_robot_web/frontend/test/statusMonitorController.test.mjs;src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `suoqu`
- `diagnostics`
- `timeout`

### 验证证据

- `node src/tie_robot_web/frontend/test/statusMonitorController.test.mjs; for test_file in src/tie_robot_web/frontend/test/*.test.mjs; do node  || exit 1; done; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_status_capsule_tracks_only_connection_and_hardware; npm --prefix src/tie_robot_web/frontend run build`

### 后续注意

- 暂无。

## 2026-05-07 23:43 - 索驱断链后的动作续接收口

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场日志确认 23:27 后只有一个 suoqu_driver_node TCP 连接到 192.168.6.62:2001，/cabin/cabin_data_upload 也只有一个发布者；22:54 的高频 Broken pipe 是旧假重连放大，23:31 仍可见单次 0x0001 状态查询 connection closed by peer，说明底层确有上位机/链路主动关连接。当前将索驱单点移动、pseudo_slam 固定工作区/全局中心/多扫描位移动全部切到 move_cabin_pose_for_automatic_execution + wait_cabin_axis_stable_arrival：断链/driver raw_move 暂时失败时等待驱动恢复，状态恢复后重新下发当前 TCP_Move 目标并续接当前动作；底层 CabinTcpTransport 和 legacy Frame_Generate 发送均改用 MSG_NOSIGNAL，避免 peer close 时 SIGPIPE 杀死节点。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_cabin_protocol_contract; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_hw tie_robot_process; systemctl status tie-robot-driver-suoqu.service; ss -tnp`

### 后续注意

- 暂无。

## 2026-05-07 23:25 - 长按停止并回起点轮询释放

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 调整 /moduan/return_zero_ordered：服务入口主动置 moduan_return_zero_ordered_requested=true 并写 IS_STOP=1，然后每100ms轮询等待 lashing_mutex 释放（30s超时）和线性模组真实速度释放（|X/Y/Z_SPEED|<=10mm/s，8s超时），之后才清停止位并按Z优先回(0,0,0)。wait_linear_module_axis_arrival 收到长按回起点请求会立即返回 false 释放执行链。索驱‘线性模组正在运动’硬拦截不要再用 /moduan_work 或PLC任务锁存作为依据。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp;src/tie_robot_control/src/moduan/linear_module_executor.cpp;src/tie_robot_control/test/test_single_point_bind_chain.py`

### 关键决策

- 见摘要。

### 标签

- `moduan`
- `long-press`
- `return-zero`
- `motion-guard`

### 验证证据

- `python3 src/tie_robot_control/test/test_single_point_bind_chain.py; python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_control tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 23:18 - 索驱busy联锁只看真实末端运动

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确指出任务/接管/回零流程状态与线性模组真实运动无关。当前已把索驱侧moduan_work_flag收口为只跟随/moduan/state.executing，不再受/moduan_work任务锁影响；/moduan/state.executing也改为只由X/Y/Z/旋转电机速度绝对值超过10判定，不再把moduan_plc_execution_state纳入。Z联锁仍要求末端状态新鲜、connected=true且|z|<=10mm。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_control tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 23:08 - 末端速度噪声busy阈值调到10

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场确认线性模组未动时PLC速度反馈可能有X=-0.04/Y=-0.01等小噪声，旧kModuanStateMovingSpeedEpsilon=0.01会让/moduan/state.executing=true并触发索驱busy guard。已按用户要求把末端状态速度阈值调为10.0，只有X/Y/Z/旋转电机速度绝对值超过10才因速度判定executing；PLC执行状态moduan_plc_execution_state仍会直接置executing。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_control`

### 后续注意

- 暂无。

## 2026-05-07 23:05 - 视觉调试可配置索驱路径规划Z下限

<!-- AGENT-MEMORY: entry -->

### 摘要

- 设置/视觉调试新增索驱规划 Z 下限，默认 485mm，随 tie_robot_frontend_visual_debug_settings 持久化并在摘要显示；扫描建图 action/service 透传 bind_execution_cabin_min_z_mm 到动态绑扎规划配置，规划时用该值夹紧 cabin_pose.z。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_msgs/srv/StartPseudoSlamScan.srv`
- `src/tie_robot_msgs/action/StartPseudoSlamScanTask.action`
- `src/tie_robot_process/src/suoquNode.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/*.mjs 全量逐个执行通过；python3 -m unittest src.tie_robot_process.test.test_scan_artifact_write_guard -q 通过；npm run build 通过；catkin_make --pkg tie_robot_msgs tie_robot_process tie_robot_web 通过。`

### 后续注意

- 暂无。

## 2026-05-07 22:54 - 索驱Z归零联锁需要新鲜末端状态

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱Z归零联锁不接受陈旧缓存：/moduan/state 回调会缓存 connected、z 和接收时间；索驱绝对位姿与TCP相对运动入口只有在已收到新鲜状态（1秒内）、connected=true、z为有限值且|z|<=10mm、并且末端不busy时才允许下发。状态未知、过期、未连接、Z无效或Z超出10mm都会拒绝。

### 影响范围

- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 22:51 - 索驱移动必须等待末端Z归零

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱统一移动入口 move_cabin_pose_via_driver / move_cabin_incremental_via_driver 现在除了末端busy外，还必须收到 /moduan/state 且线性模组末端 |z|<=10mm 才允许下发；状态未知、NaN或Z超出10mm都会拒绝索驱绝对/相对运动，/cabin/driver/raw_move 与人工/自动索驱移动共同受该guard保护。

### 影响范围

- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 22:50 - 索驱状态查询断管假重连修复

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场 22:44 日志中的未知索驱指令(0x0001) 实为 TCP_Normal_Connection 状态查询帧；旧 Frame_Generate 通过 CabinDriver 暴露的 raw fd 发送，发送/接收失败后没有让 CabinDriver 断开 transport，connectToServer() 可能只看到旧 state=ready 而假重连成功，导致同一个坏 fd 反复 Broken pipe。当前已给 0x0001 补充‘索驱状态查询’调试名，并在 Frame_Generate_With_Retry 发送失败进入重连前 stop CabinDriver、同步 sockfd=-1，再真正重建 TCP 连接；重连等待使用 kCabinDriverRecoveryRetrySleepMs。

### 影响范围

- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard.MotionChainSignalGuardTest.test_cabin_protocol_retry_refreshes_socket_after_reconnect src.tie_robot_process.test.test_motion_chain_signal_guard.MotionChainSignalGuardTest.test_cabin_state_command_has_explicit_debug_name src.tie_robot_process.test.test_motion_chain_signal_guard.MotionChainSignalGuardTest.test_cabin_protocol_retry_drops_stale_transport_before_reconnect src.tie_robot_process.test.test_motion_chain_signal_guard.MotionChainSignalGuardTest.test_cabin_protocol_retry_keeps_requesting_instead_of_emergency_exit src.tie_robot_process.test.test_motion_chain_signal_guard.MotionChainSignalGuardTest.test_legacy_frame_retry_only_waits_on_pure_motion_busy_status src.tie_robot_process.test.test_motion_chain_signal_guard.MotionChainSignalGuardTest.test_cabin_state_poll_uses_legacy_100ms_socket_loop src.tie_robot_process.test.test_motion_chain_signal_guard.MotionChainSignalGuardTest.test_cabin_tcp_legacy_frame_reader_consumes_exact_protocol_response_length; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 22:44 - 索驱底层raw_move也必须吃末端busy

<!-- AGENT-MEMORY: entry -->

### 摘要

- 进一步确认索驱驱动层/cabin/driver/raw_move同样会进入move_cabin_pose_via_driver；若只在bind_task_executor角色订阅/moduan_work，driver角色会默认moduan_work_flag=false并绕过末端运动guard。已把/moduan_work和/moduan/state订阅提升到RunSuoquNodeWithDefaultRole的所有索驱角色通用初始化，确保driver、cabin_motion_controller、bind_task_executor都会用同一末端busy合成信号拦截索驱移动。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp; src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 索驱任何移动入口（含driver/raw_move）都必须订阅并服从末端busy/state，不允许只在执行层做guard。

### 标签

- `safety`
- `suoqu`
- `moduan`
- `raw-move`

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; catkin_make -DCATKIN_WHITELIST_PACKAGES=""`

### 后续注意

- 暂无。

## 2026-05-07 22:43 - 末端运动期间硬禁止索驱动作

<!-- AGENT-MEMORY: entry -->

### 摘要

- 确认现场事故链路：planned_path_refine_only/动作Action在等待FINISHALL失败或末端未确认完成时曾按跳过当前区域继续，且/moduan_work会在失败析构路径过早false，导致后续索驱可被重新下发。已改为末端FINISHALL/线性模组未确认完成即停止全局执行链，ScopedPlcExecutionState仅在成功到位/FINISHALL确认后清busy，失败保持/moduan_work=true；索驱guard同时订阅周期/moduan/state.executing，ModuanState.executing也纳入轴速度，防止/moduan_work非周期消息或进程重启空窗。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp; src/tie_robot_control/src/moduan/linear_module_executor.cpp; src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp; src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 线性模组/末端运动中，索驱任何自动、重发、恢复、跳区都不得动作；末端执行未确认安全完成时宁可阻断全局链并保持busy，不允许跳过继续。

### 标签

- `safety`
- `moduan`
- `suoqu`
- `motion-chain`

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_control/test/test_single_point_bind_chain.py; catkin_make -DCATKIN_WHITELIST_PACKAGES=""`

### 后续注意

- 暂无。

## 2026-05-07 22:33 - 索驱末端视觉子系统隔离

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确现场口径：索驱归索驱、线性模组/末端归线性模组/末端、视觉归视觉，任何一个掉线/超时/重启不要影响其他层；索驱断链只应在索驱层持续重连。当前已移除 tie-robot-backend.service、tie-robot-driver-suoqu.service、tie-robot-driver-moduan.service、tie-robot-driver-camera.service 对 tie-robot-rosbridge.service 的 PartOf 生命周期绑定，保留 Wants/After 和 wait_for_ros_master.py 作为启动前等待；已安装更新后的 unit 到 /etc/systemd/system 并 daemon-reload，服务未重启且均 active。

### 影响范围

- `src/tie_robot_bringup/systemd/tie-robot-backend.service.in;src/tie_robot_bringup/systemd/tie-robot-driver-suoqu.service.in;src/tie_robot_bringup/systemd/tie-robot-driver-moduan.service.in;src/tie_robot_bringup/systemd/tie-robot-driver-camera.service.in;src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py;README.md;CHANGELOG.md`

### 关键决策

- 见摘要。

### 标签

- `systemd`
- `isolation`
- `suoqu`
- `moduan`
- `vision`

### 验证证据

- `python3 src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py; python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; systemctl show -p PartOf -p Wants -p After tie-robot-driver-suoqu.service tie-robot-driver-moduan.service tie-robot-driver-camera.service tie-robot-backend.service; systemctl is-active tie-robot-driver-suoqu.service tie-robot-driver-moduan.service tie-robot-driver-camera.service tie-robot-backend.service`

### 后续注意

- 暂无。

## 2026-05-07 22:25 - 索驱底层持续保活重连

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求索驱底层在重新发送命令失败超过5次时不要紧急退出程序。当前 Frame_Generate_With_Retry 改为 ros::ok() 生命周期内持续请求索驱：通信发送/读取失败后保持节点运行、循环重连，重连成功后继续重发原指令；只有协议明确非瞬态拒绝仍返回失败，不再因5次重发或5次重连调用 emergency_exit_with_flush(4)。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 标签

- `suoqu`
- `driver`
- `keepalive`
- `tcp`

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process'; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=''`

### 后续注意

- 暂无。

## 2026-05-07 22:06 - Surface-DP 结构连续梁筋兜底

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场另一根梁筋在 rectified x≈331..342 附近高度响应不够高，旧高度门控漏检并被 line-family 当普通竖筋，导致梁筋过滤后仍保留异常钢筋列。当前 scan_surface_dp 增加 wide_continuous_column 结构连续兜底，并在 lattice gate 中允许这类候选结合相邻间距畸变恢复；最终现场 /pointAI/process_image MODE_SCAN_ONLY 红带为 x=153..159 与 x=331..342，服务 count=192。现场抓图目录：.debug_frames/beam_current_live_final_20260507_220525。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- `CHANGELOG.md`
- `.debug_frames/beam_current_live_final_20260507_220525/summary.json`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_scan_surface_dp_runtime; PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg -k beam; compileall + git diff --check; live ROS final count=192 red_bands x=153..159`
- `x=331..342`

### 后续注意

- 暂无。

## 2026-05-07 21:41 - 执行微调2x2改为TCP零点最近组

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求MODE_EXECUTION_REFINE的2x2选择改为取离TCP零点最近的一组2x2，顺序形如(1,1)->(1,2)->(2,2)->(2,1)。当前matrix_selection.py会把候选点转换到TCP坐标，按TCP x/y行列匹配完整2x2，在完整组中按整体到(0,0)的距离评分选择最近组；缺行或缺列仍不下发零散点。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest (115 tests OK); python3 -m py_compile matrix_selection.py OK; git diff --check OK; pointAINode restarted and /pointAI/process_image service reachable`

### 后续注意

- 暂无。

## 2026-05-07 21:23 - planned_path_refine_only视觉等待缩短到300ms

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求将planned_path_refine_only首区大Z下降后的视觉稳定等待和EXECUTION_REFINE_NO_POINTS首次无点重试等待从1200ms缩短为300ms。当前suoquNode.cpp中kPlannedPathLargeZPreBindSettleMs=300、kPlannedPathNoPointsRetrySettleMs=300；普通区域到位短等待仍为250ms。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard (42 tests OK); catkin_make --pkg tie_robot_process OK`

### 后续注意

- 暂无。

## 2026-05-07 21:20 - 执行微调2x2候选选择首区跳过修复

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场首区起点图像显示线性模组工作区内有点，但planned_path_refine_only仍跳过。排查确认MODE_EXECUTION_REFINE不是范围内无点：日志显示原始候选5/6、范围内5/6、范围外0，但旧选择器先取离TCP工作盒中心最近四点，若这四点因多余候选落入重复象限就输出0。现改为在四个中心象限内各选最近候选，四象限完整即输出2x2；缺象限仍不下发零散点。重启pointAINode后同一位置12次/pointAI/process_image MODE_EXECUTION_REFINE纯视觉调用全部success=True,count=4。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest (115 tests OK); after rosnode kill /pointAINode respawn`
- `12 repeated rosservice call /pointAI/process_image 4 all returned success=True`
- `count=4`

### 后续注意

- 暂无。

## 2026-05-07 20:49 - planned_path_refine_only首区单点绑扎稳定重试

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场 2026-05-07 20:35 证据显示 planned_path_refine_only 第一区域不是索驱到位失败，而是 /moduan/sg 已调用后 MODE_EXECUTION_REFINE 返回 EXECUTION_REFINE_NO_POINTS，因此未进入 execute_bind_points/FINISHALL 等待。已在索驱到位后增加视觉稳定等待：普通区域250ms，从高位大Z落差到首区时1200ms；纯单点绑扎首次无点时再等待1200ms并重试一次。真正的FINISHALL/PLC失败不被吞掉，仍按失败处理。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; python3 -m unittest src.tie_robot_control.test.test_single_point_bind_chain; bash -lc 'source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process'; git diff --check -- src/tie_robot_process/src/suoquNode.cpp src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 后续注意

- 暂无。

## 2026-05-07 20:46 - planned_path_refine_only首区视觉无点重试

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场从3m扫描识别位姿下降到首个执行区域后，planned_path_refine_only并非索驱到位失败，而是/moduan/sg已调用但MODE_EXECUTION_REFINE返回EXECUTION_REFINE_NO_POINTS，导致第一区域未进入末端执行和FINISHALL等待。当前执行链在每个区域索驱到位后增加视觉稳定等待：普通移动后等250ms，大幅Z移动(>=500mm)后等1200ms；纯单点绑扎分支首次EXECUTION_REFINE_NO_POINTS时再等1200ms并重试一次，避免刚下降后的首帧不稳直接跳区。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_scan_artifact_write_guard.ScanArtifactWriteGuardTest.test_global_execution_mode_selects_ledger_refine_or_pure_refine_without_bind_path_short_circuit; source /opt/ros/noetic/setup.bash && source devel/setup.bash && catkin_make --pkg tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 20:22 - Surface-DP 成对梁筋线族吞噬恢复

<!-- AGENT-MEMORY: entry -->

### 摘要

- 扫描 Surface-DP 梁筋候选发现一类现场漏检：强响应梁筋会把物理竖向 line family 吸到梁筋中心，导致 lattice gate 误判为普通竖筋线上的加粗列并丢弃。当前 scan_surface_dp 的 lattice gate 增加成对 raised beam 恢复：若贴线候选本身是连续高响应/高差 raised_column，且同帧存在相隔多个普通钢筋间距、未被线族吞掉或具有 dark_gutter 签名的梁筋伙伴，则以 paired_raised_beam_recovered_from_line_family 接受；单根 raised regular line 仍拒绝。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k beam; git diff --check -- src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 后续注意

- 暂无。

## 2026-05-07 20:15 - 路径规划微调支持跳绑与最近账本纠正

<!-- AGENT-MEMORY: entry -->

### 摘要

- planned_path_refine_only 路径规划+微调模式已支持跳绑热开关：跳绑关闭保持 /moduan/sg 纯单点微调；跳绑开启时每个区域到位后调用 MODE_EXECUTION_REFINE，把识别点按世界坐标欧式距离匹配本区域 pseudo_slam_bind_path.json 最近账本点，继承该账本点黑白棋/全局索引元数据后按当前 parity 过滤执行。执行局部坐标仍使用当前识别点 Scepter_depth_frame->gripper_frame 转换结果，不把纠正写回扫描账本。checkerboard_jump_bind_enabled 已改为 atomic<bool>，区域执行前读取快照，运行中开关对后续区域生效。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_scan_artifact_write_guard.ScanArtifactWriteGuardTest.test_global_execution_mode_selects_ledger_refine_or_pure_refine_without_bind_path_short_circuit src.tie_robot_process.test.test_scan_artifact_write_guard.ScanArtifactWriteGuardTest.test_planned_path_refine_only_jump_bind_uses_nearest_ledger_correction src.tie_robot_process.test.test_scan_artifact_write_guard.ScanArtifactWriteGuardTest.test_jump_bind_frontend_long_press_toggles_and_click_selects_checkerboard_color; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; python3 -m unittest src.tie_robot_process.test.test_scan_artifact_write_guard.ScanArtifactWriteGuardTest.test_bind_path_direct_test_uses_bind_path_only_without_outlier_blocking src.tie_robot_process.test.test_scan_artifact_write_guard.ScanArtifactWriteGuardTest.test_scan_artifacts_and_execution_memory_carry_jump_bind_color_metadata src.tie_robot_process.test.test_scan_artifact_write_guard.ScanArtifactWriteGuardTest.test_start_execution_defaults_to_execution_memory_disabled; catkin_make`

### 后续注意

- 暂无。

## 2026-05-07 19:26 - 执行微调最近四点2x2与无接纳门限

<!-- AGENT-MEMORY: entry -->

### 摘要

- MODE_EXECUTION_REFINE 只按 TCP 工作盒中心欧式距离取最近四个候选点；这四点必须分别位于工作盒中心四象限，形成完整 2x2，才按 TCP 局部蛇形编号下发。若最近四点不成 2x2，不再为了凑矩阵选择更远点。live_visual 账本+微调链路已移除 kLiveVisualMicroAdjustXYToleranceMm 与‘超出xy微调范围’接纳门，视觉微调点不再因相对扫描参考点固定偏差被拒绝。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; git diff --check --相关文件`

### 后续注意

- 暂无。

## 2026-05-07 19:17 - 索驱状态心跳严格回退到100ms旧路径

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场前端索驱已连接/通信异常闪烁的直接证据：/cabin/cabin_data_upload 在 v40 driver pollState 路径下约 40Hz 发布，采样出现 cabin_connect_flag 1/1/0/1；driver 日志同时有 0x0001 状态查询 connection closed by peer。已移除 CabinDriver::pollState 运行入口，read_cabin_state 回退为旧 100ms TCP_Normal_Connection + Frame_Generate_With_Retry 状态查询路径，保留运动指令纯 status_word=0x00000004 的暂态 busy 托底。重启后状态话题约 9Hz，连续采样 cabin_connect_flag 全为 1，diagnostics 为 索驱驱动已连接/ready。

### 影响范围

- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_driver.hpp; src/tie_robot_hw/src/driver/cabin_driver.cpp; src/tie_robot_process/src/suoquNode.cpp; src/tie_robot_process/test/test_motion_chain_signal_guard.py; src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`

### 关键决策

- 索驱状态心跳不再走 driver-layer pollState 高频路径；当前只保留用户要求的运动 busy 托底策略。

### 标签

- `suoqu`
- `cabin-tcp`
- `heartbeat`
- `rollback`

### 验证证据

- `TDD红灯: test_cabin_state_poll_uses_legacy_100ms_socket_loop 先失败于 bool pollState 残留；绿灯: python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract src.tie_robot_process.test.test_cabin_tcp_transport_contract => Ran 52 tests OK；catkin_make -DCATKIN_WHITELIST_PACKAGES= 通过；重启 tie-robot-backend.service/tie-robot-driver-suoqu.service 后 rostopic hz /cabin/cabin_data_upload 约 9Hz 且 12 个样本 connect_flag 全为 1。`

### 后续注意

- 暂无。

## 2026-05-07 19:09 - 索驱驱动层严格回退确认

<!-- AGENT-MEMORY: entry -->

### 摘要

- 排查确认 src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp 曾残留 v35/非阻塞建连实验改动；已用 git restore 回退，当前 git diff -- src/tie_robot_hw 为空。运行侧只保留 tie_robot_process/suoquNode.cpp 中的托底策略：仅 0x0010/0x0011/0x0012 运动指令返回纯 status_word=0x00000004 视为暂态忙并等待重试，Z 超正限位、速度错误等硬失败不被吞掉。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp; src/tie_robot_process/src/suoquNode.cpp; src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 索驱驱动层不保留本轮通信优化/建连模型实验；通信驱动层严格回退，只保留自动执行层的运动 busy 托底。

### 标签

- `suoqu`
- `cabin-tcp`
- `rollback`
- `fallback`

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract src.tie_robot_process.test.test_cabin_tcp_transport_contract; catkin_make -DCATKIN_WHITELIST_PACKAGES=; systemctl restart tie-robot-backend.service tie-robot-driver-suoqu.service 后确认 active 且运行进程加载 devel/lib/libtie_robot_hw_driver_core.so`

### 后续注意

- 暂无。

## 2026-05-07 19:03 - 执行微调只输出中心最近2x2

<!-- AGENT-MEMORY: entry -->

### 摘要

- MODE_EXECUTION_REFINE 的 Hough 候选进入 TCP 执行范围后，不再全量下发；选择层只输出离线性模组工作盒中心最近的一组完整 2x2 四点矩阵，并继续按 TCP 局部蛇形顺序编号。若候选无法组成完整 2x2，则不向执行层下发零散点；旧近点排斥/去重仍未恢复。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg`

### 后续注意

- 暂无。

## 2026-05-07 19:02 - 执行微调 X/Y 接纳门限放宽到 100mm

<!-- AGENT-MEMORY: entry -->

### 摘要

- live_visual 账本+微调执行时，视觉微调点与扫描参考点的接纳门限已按用户最新要求放宽为 X/Y 各 ±100mm；仍只检查 XY，不恢复 Z 微调门限，微调点的 world_z 继续使用视觉返回值。

### 影响范围

- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_process'`

### 后续注意

- 暂无。

## 2026-05-07 18:47 - 执行微调 X/Y 接纳门限改为 40mm

<!-- AGENT-MEMORY: entry -->

### 摘要

- live_visual 账本+微调执行时，视觉微调点与扫描参考点的接纳门限已从 X/Y 各 120mm 收紧到 X/Y 各 ±40mm；仍只检查 XY，不恢复 Z 微调门限，微调点的 world_z 继续使用视觉返回值。

### 影响范围

- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard`

### 后续注意

- 暂无。

## 2026-05-07 18:39 - 修复长按停止后 FINISHALL 等待竞争

<!-- AGENT-MEMORY: entry -->

### 摘要

- split-node 模式下，/moduan/driver/raw_execute_points 不能和 /moduan/return_zero_ordered 并发执行；raw execute 服务已改为进入 execute_bind_points 前先持有 lashing_mutex，确保长按停止置位后旧执行链先从 wait_for_plc_finish_all 返回 false，再由有序回零服务清标志并回零，避免继续卡在 FINISH_ALL_FLAG=0。现场服务若仍 executing，不要擅自重启驱动节点，需等空闲或明确授权后切换新二进制。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_control/test/test_single_point_bind_chain.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_control.test.test_single_point_bind_chain; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; catkin_make --pkg tie_robot_control`

### 后续注意

- 暂无。

## 2026-05-07 18:28 - Surface-DP 梁筋高度门改为邻近上下文

<!-- AGENT-MEMORY: entry -->

### 摘要

- 梁筋候选漏识别根因：_refine_beam_band_by_height 的左右上下文过宽，会把远处更高普通竖筋当作‘周围钢筋’，导致局部位于相邻竖筋中间且更高的梁筋 x≈331..339 被拒绝。已将高度门 guard/context 收紧到邻近钢筋尺度，并保留 lattice 中点门控；同时增加黑色竖沟梁筋贴在线族上时的豁免，但没有放开无 dark_gutter 签名的普通加粗竖筋。现场服务重启后 /pointAI/process_image 真实运行态红带从 1 条恢复为 2 条：x=149..156 与 x=331..339；启用梁筋±13cm过滤时该帧输出点从 256 降到 224，随后已关闭过滤参数。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_scan_surface_dp_runtime; source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg; runtime /pointAI/process_image red_band_count=2 at x=149..156 and x=331..339`

### 后续注意

- 暂无。

## 2026-05-07 17:59 - 按用户要求回退 Surface-DP 梁筋约束后续改动

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求将代码回退到‘这里有些点被带偏到梁筋上去了，梁筋上是不能有点的，如何约束’之前。当前已把 scan_surface_dp.py 与 test_scan_surface_dp_runtime.py 回退到提交 f619019 对应状态，撤销 da94e23 中新增的 raw beam 默认禁入 tracing 与 line_rho overlap 过滤；manual_workspace_s2.py 同步撤销后续 beam_candidate_debug_bands / rejected overlay 诊断渲染，恢复到 beam_candidate_bands 口径。保留 f619019 之前已有的 beam_candidate 检测与可选 ±13cm 最终点过滤。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -k beam && python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -k debug_base_images_overlay_rectified_intersections && git diff --check -- src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 后续注意

- 暂无。

## 2026-05-07 17:45 - 索驱忙状态托底收口为运动指令纯 bit2

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确只保留一层托底：索驱运动指令 0x0010/0x0011/0x0012 返回纯 status_word=0x00000004/设备运动中时，按‘索驱忙、可等待重试’处理，不自动跳区；其它状态如 Z超正限位、速度错误必须硬失败。已收口 wait_cabin_axis_arrival/wait_cabin_axis_stable_arrival 使用已缓存 command_word+status_word 判断；同时修复 legacy Frame_Generate_With_Retry 之前对任意 0x0012 运动回包异常都持续重试的问题，现在只在 is_transient_cabin_motion_status(command,status)==true 时继续等待。自动执行重试也不再仅凭‘设备运动中’或 status_word=0x00000004 字符串重试，必须同时匹配运动指令文本和纯忙状态。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `红灯测试先确认旧 Frame_Generate_With_Retry 会吞掉任意运动状态异常；修改后 python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract src.tie_robot_process.test.test_cabin_tcp_transport_contract -> Ran 53 tests OK；source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES= -> build OK；sudo -n systemctl restart tie-robot-backend.service tie-robot-driver-suoqu.service 后 backend/driver active，运行 PID: suoqu_driver_node=307105`
- `cabin_motion_controller=307211`
- `bind_task_executor=307213；未下发索驱移动命令。`

### 后续注意

- 暂无。

## 2026-05-07 17:29 - 索驱 TCP 工业化方案归档并回退到 v35 建连模型

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求将刚才的索驱 TCP 工业化连接/短事务/链路指标方案存档并回退，运行方案改为参考 /home/hyq-/lashingrobotROS/src 的 slam/v35 索驱驱动层建连口径：保留持久 socket，不再暴露 CabinTcpConnectionMode/CabinTcpTransportStats/cabin_tcp_connection_mode/auto_detect/短事务收口；CabinTcpTransport::connectLocked 采用 v35 的非阻塞 connect + select 5s + SO_ERROR 判定 + 成功后恢复阻塞模式，同时保留当前工程已有的完整协议长度接收。当前方案已归档到 docs/archive/cabin_tcp_industrialized_connection_scheme_2026-05-07.md。已重启 tie-robot-driver-suoqu.service，仅建连/状态轮询，未下发运动命令；运行态 PID 299150，socket ESTAB 到 192.168.6.62:2001。回退后 diagnostics 会直接暴露对端 heartbeat connection closed by peer，不再使用刚才的前端托底显示。

### 影响范围

- `docs/archive/cabin_tcp_industrialized_connection_scheme_2026-05-07.md`
- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract -> Ran 52 tests OK; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES= -> build OK; sudo -n systemctl restart tie-robot-driver-suoqu.service -> active PID 299150; readlink /proc/299150/exe -> current devel binary; ss shows one ESTAB to 192.168.6.62:2001`

### 后续注意

- 暂无。

## 2026-05-07 17:17 - 索驱 TCP 驱动工业化连接语义与链路指标

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱 TCP 驱动已按工业驱动口径拆分 TCP 握手、应用层 ready、连接灯显示和链路统计：TCP connect 成功仅进入 connecting，协议回包成功才进入 ready；默认连接模式保守使用 persistent，另提供 short_transaction 与 auto_detect 模式，并通过 ROS 参数 ~cabin_tcp_connection_mode 可切换。新增 last_connect_ms/last_send_ms/last_first_byte_ms/last_full_frame_ms/peer_close_count/reconnect_count/auto_short_transaction_detected 诊断字段。现场试验显示 auto_detect 短事务会让状态采样出现空窗，因此默认不启用；persistent 下 30 秒诊断 level 0=30/30、visible ready=30/30、socket 为单个 ESTAB，send p50≈0.008ms，首字节/整帧 p50≈74.8ms。

### 影响范围

- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_tcp_transport.hpp`
- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_driver.hpp`
- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=; /diagnostics 30s: levels {0:30}`
- `visible_states {'ready':30}`
- `mode persistent`

### 后续注意

- 暂无。

## 2026-05-07 16:56 - 索驱连接灯闪烁根因与诊断托底

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱心跳/状态查询的 0 字节读超时保留连接策略只能匹配 tcp_read_wait_failed；peer 主动关闭 connection closed by peer 必须断开并重连，不能当 ready 保留。诊断层对 connect_flag=1 且状态新鲜的瞬时失败做显示托底：前端可见 level/message/transport_state 保持 OK/索驱驱动已连接/ready，原始 raw_transport_state 与 transient_* 字段保留给排查，避免上位机连接灯和日志在 ready/reconnecting/通信异常之间闪烁。现场 30 秒验证可见状态 30/30 ready、bad_visible_frames=0；raw 状态仍可见 reconnecting，说明对端仍会主动关连接但不再打闪 UI。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=; /diagnostics 现场 30s: visible_states {'ready': 30}`
- `bad_visible_frames 0`

### 后续注意

- 暂无。

## 2026-05-07 13:54 - 索驱工业级收发补强：heartbeat迟到包隔离与非阻塞建连

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱 transport 新增两项关键补强：1) sendAndReceive 在 heartbeat(0x0001) 与运动状态请求发包前统一 drainPendingInputLocked，避免上一拍 heartbeat 超时后同一 socket 上迟到的 144 字节状态包被下一拍误当成功回包；新增契约测试已复现并锁定该错包场景。2) connectLocked 改为非阻塞 connect + select + SO_ERROR，建连等待上限收口到 300ms，并开启 SO_KEEPALIVE，避免局域网目标离线时内核 SYN 重传把 poll 线程卡到 130s 级。现场 2026-05-07 13:53 通过杀掉 respawn 子进程让 suoqu_driver_node 吃到新二进制后，/proc/260023/exe 已不再是 (deleted)。无位移现场验证显示 last_poll_duration_ms 从此前 130072ms 级长阻塞消失，健康片段回到约 0.05-1.31ms、last_success_poll_duration_ms 约 1.3-2.1ms；当前剩余主问题已收敛为对端经常 connection closed by peer / heartbeat 0字节不回包，而不再是本机 connect 卡死或迟到 heartbeat 污染下一拍。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract && source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=`

### 后续注意

- 暂无。

## 2026-05-07 13:19 - 索驱状态闪烁根因与到位等待链粒度下调

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端出现‘索驱驱动已连接 -> 索驱驱动通信异常 -> 索驱驱动已连接’的闪烁，本质上是某一拍 0x0001 heartbeat 没收到 144 字节状态包，read_cabin_state 将 failure_detail 写成‘索驱状态查询失败…received=0/144…request_command=0x0001’，下一拍 pollState 又成功，clear_last_cabin_transport_error_detail() 被调用后 diagnostics 恢复为 ready。前端 StatusMonitorController 会把 diagnostic message + failure_detail/transport_error/transport_state 直接拼成 detail，并按 rawValue 变化逐条记日志，所以同一秒内能看到连上-异常-连上的跳变。另已将旧 wait_cabin_axis_arrival 和 wait_cabin_axis_stable_arrival 的固定 100/200ms 休眠统一收口到 kCabinAxisArrivalWaitSleepMs=20ms，对齐当前索驱状态轮询节拍，减少‘索驱已执行但上位机晚一拍知道’的完成判定拖尾。仅完成编译与测试验证，未重启现场节点、未移动索驱。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_web/frontend/src/controllers/StatusMonitorController.js`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_cabin_protocol_contract && source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=`

### 后续注意

- 暂无。

## 2026-05-07 13:09 - 索驱 stop 的 900ms 主要是 rosservice CLI 开销，不是局域网通讯

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场对同一个 /cabin/motion/stop 做了两组无位移实测：1) 使用命令行 rosservice call 时，总耗时约 914-985ms；2) 使用常驻 rospy.ServiceProxy(persistent=True) 时，同一 stop 服务调用仅 1-5ms，并连续 success=true。为排除硬件因素，又对纯本机 ROS 内部服务 /suoqu_driver_node/get_loggers 做了同样对照：rosservice call 约 972ms，而 persistent ServiceProxy 仅 0.16-3.9ms。结论：此前看到的 900ms 级数字主要是 rosservice 命令行客户端的 Python 启动、master 查询和 XMLRPC/TCPROS 建连开销，不代表索驱局域网通讯时延。当前 chassis diagnostics 还抓到 transport_state=ready、connect_flag=1、last_poll_duration_ms≈1.396ms、poll_duration_ema_ms≈1.423ms、consecutive_poll_failures=0，说明驱动常驻链路本身已处于毫秒级。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `python3 - <<'PY'
import time`
- `rospy
from std_srvs.srv import Trigger
rospy.init_node('memory_stop_probe'`
- `anonymous=True`
- `disable_signals=True)
rospy.wait_for_service('/cabin/motion/stop'`
- `timeout=5.0)
proxy = rospy.ServiceProxy('/cabin/motion/stop'`
- `Trigger`
- `persistent=True)
start=time.time(); resp=proxy(); print((time.time()-start)*1000.0`
- `resp.success)
PY
&& python3 - <<'PY'
import time`
- `rospy
from roscpp.srv import GetLoggers
rospy.init_node('memory_logger_probe'`
- `anonymous=True`
- `disable_signals=True)
rospy.wait_for_service('/suoqu_driver_node/get_loggers'`
- `timeout=5.0)
proxy = rospy.ServiceProxy('/suoqu_driver_node/get_loggers'`
- `GetLoggers`
- `persistent=True)
start=time.time(); resp=proxy(); print((time.time()-start)*1000.0`
- `len(resp.loggers))
PY`

### 后续注意

- 暂无。

## 2026-05-07 13:05 - 索驱通讯效率继续落地：软超时快重试与命令静默窗生效

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱状态轮询新增软超时失败分类：heartbeat 零字节超时不再走 100-1000ms 指数回退，而是按 20ms 快重试；同时 stop/raw move 前后引入 command inflight + quiet window，让状态轮询在命令窗口内让行。CabinTcpTransport 的 heartbeat 和 0x0011/0x0012/0x0013 首字节/总窗进一步收口到 200ms/260ms。现场重启 suoqu_driver_node 到新二进制后复测，/cabin/motion/stop 从此前约 1.44s 且 success=false，改善为连续 success=true、约 918-985ms；chassis diagnostics 抓到 transport_state=ready、connect_flag=1、last_poll_duration_ms≈1.396ms、poll_duration_ema_ms≈1.423ms、consecutive_poll_failures=0、poll_sleep_ms=20、soft_timeout_retry_ms=20。停止后再回读 diagnostics 仍保持 ready，command_quiet_remaining_ms≈86.755，说明命令静默窗已在线生效。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/include/tie_robot_process/suoqu/cabin_transport.hpp`
- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_cabin_protocol_contract && source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES= && rosservice call /cabin/motion/stop`

### 后续注意

- 暂无。

## 2026-05-07 12:51 - 自适应绑扎分组避免大账本递归搜索

<!-- AGENT-MEMORY: entry -->

### 摘要

- DynamicBindPlanning 的 adaptive 分组已从全局递归搜索收口为按账本/网格顺序选择当前最大可达矩形组，并在候选会留下单点尾巴时降档；adaptive 的最小边缘组允许横向和竖向2点组，避免右/下边缘单列或单行点没人规划。新增16x16大账本回归测试锁定：不会指数级卡死、不会生成1点组、256点可全部规划，未勾选仍走固定2x2。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `make -C build/tie_robot_process tie_robot_process_planning/fast test_dynamic_bind_planning/fast && ./devel/lib/tie_robot_process/test_dynamic_bind_planning && make -C build/tie_robot_process suoquNode/fast && python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 后续注意

- 暂无。

## 2026-05-07 12:42 - 自适应绑扎分组按TF工作域接入扫描入口

<!-- AGENT-MEMORY: entry -->

### 摘要

- 视觉调试里的自适应每组绑扎点数现在通过 bind_group_point_count<=0 进入后端，并在 suoquNode 构建 DynamicBindPlannerConfig 时显式设置 adaptive_grouping_enabled=true。自适应模式口径是：按当前 TF 与索驱规划高度选择单区域最大可达矩形组，目标是尽量减少区域，最小组为2点；未勾选仍保持固定2x2。若现场勾选后行为像固定分组，先检查 suoquNode 是否把 adaptive_bind_grouping 传入 build_dynamic_bind_planner_config。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`

### 关键决策

- 见摘要。

### 验证证据

- `make -C build/tie_robot_process suoquNode/fast && ./devel/lib/tie_robot_process/test_dynamic_bind_planning && python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 后续注意

- 暂无。

## 2026-05-07 12:41 - 索驱 heartbeat 零字节超时改为保连接，现场失败窗降到1.2秒

<!-- AGENT-MEMORY: entry -->

### 摘要

- CabinTcpTransport 现在对 heartbeat 的零字节超时采用‘保留现有 TCP 连接、不立即重连’策略，并把 0x0001 心跳和 0x0013 停止回包的首字节/总窗统一收口到 1200ms/1500ms。新增契约测试覆盖：同一连接上第一次 heartbeat 零字节超时后，第二次 heartbeat 仍可复用原 socket 成功返回状态。现场重启 /suoqu_driver_node 后复测，索驱 diagnostics 连续 3 帧都稳定表现为 transport_state=ready、socket_fd 持续有效、consecutive_poll_failures 递增但不再立刻拒连；heartbeat 零字节超时窗从之前的 5000ms 降到约 1200ms。/cabin/motion/stop 失败返回也从此前约 8s 进一步降到约 2.27s，当前失败原因为 0x0013 本身在 1200ms 窗口内 received=0/8。结论：软件层等待长尾已进一步压缩，剩余瓶颈主要是控制器对 0x0001 / 0x0013 的零字节不回包。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract && source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES= && rosnode kill /suoqu_driver_node && rosservice call /cabin/motion/stop`

### 后续注意

- 暂无。

## 2026-05-07 12:31 - 索驱现场复测：stop 失败时延降到约1.9秒，heartbeat 仍间歇零字节超时

<!-- AGENT-MEMORY: entry -->

### 摘要

- 在 receiveExact 分层超时模型之外，CabinTcpTransport 现已给 0x0011/0x0012/0x0013 这类 8 字节运动状态回包单独配置 1200ms 首字节 / 80ms inter-byte / 1500ms 总窗，避免安全 stop 指令继续卡在默认 5s 等待。现场复测中，/cabin/motion/stop 从之前实测约 8.0s 的失败返回降到约 1.9s，失败原因为索驱 TCP 连接拒绝。Heartbeat 现场状态仍不稳定：曾抓到 transport_state=ready、connect_flag=1、state_age_sec=0.234、last_success_poll_duration_ms=6.77ms 的成功样本，也抓到 last_success_poll_duration_ms=32.43ms 的成功样本；但多数样本仍表现为 request_command=0x0001 在 warm-up 5s 窗口内 received=0/144 零字节超时，随后 diagnostics 的 poll_duration_ema_ms 被 5s 级失败拉高。结论：软件侧的固定超时拖尾已明显收短，现场剩余主瓶颈是索驱控制器对状态心跳/停止指令的零字节不回包与间歇拒连，而不是 receiveExactLocked 本身的等待模型。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract && source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES= && rosservice call /cabin/motion/stop`

### 后续注意

- 暂无。

## 2026-05-07 12:28 - 索驱心跳超时模型分层并暴露现场零字节回包

<!-- AGENT-MEMORY: entry -->

### 摘要

- CabinTcpTransport 的 heartbeat 接收改成分层模型：冷启动未拿到首个成功回包前，首字节和总窗沿用 timeout_sec 的 warm-up 窗口；一旦该连接拿到过成功 roundtrip，就切到 1200ms 首字节 / 80ms inter-byte / 1500ms 总窗，并保留 heartbeat 事务级重试。新增契约测试锁定‘部分状态包超时后快速重试’行为，同时 warm-up 阶段仅对已收到部分数据的失败重试，0 字节超时不在同一次事务里重试。现场复测显示真正瓶颈已暴露：索驱 TCP 可建立连接，但 0x0001 状态心跳在 5s warm-up 窗口内仍返回 0/144 字节，随后 diagnostics 报 transport_state=ready 或 disconnected、last_poll_duration_ms≈5000ms、last_success_poll_duration_ms=0、consecutive_poll_failures 递增；此前 /cabin/cabin_data_upload 的 20ms 级刷新只能证明发布节拍，不能代表索驱实际有回包。

### 影响范围

- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_tcp_transport.hpp`
- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract && source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=`

### 后续注意

- 暂无。

## 2026-05-07 12:06 - Surface-DP rejected beam 只画轮廓不再整列染色

<!-- AGENT-MEMORY: entry -->

### 摘要

- Surface-DP 调试图切到 beam_candidate_debug_bands 后，rejected 梁筋候选也被按整列半透明填充，现场会把底图整体染成棕色。manual_workspace_s2 现改为只有 accepted beam 才填充 overlay，rejected 候选只保留轮廓和 A/R+原因+score 标签，既保留诊断信息又不污染底图。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -k beam && PYTHONPATH=src/tie_robot_perception/src python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k curve_trace && PYTHONPATH=src/tie_robot_perception/src python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k workspace_s2_intersects_curved_line_families_by_polyline_geometry`

### 后续注意

- 暂无。

## 2026-05-07 11:54 - 索驱 connect_flag 改为滑窗判定并补充 poll 耗时诊断

<!-- AGENT-MEMORY: entry -->

### 摘要

- suoquNode 的 read_cabin_state 不再因单次 pollState 失败立刻把 cabin_driver_last_state_stamp_sec 清零；新增 0.5s connect_flag hold window，/cabin/cabin_data_upload 的 cabin_connect_flag 改为按最近一次成功状态时间戳判定。同步新增索驱状态轮询耗时诊断：last_poll_duration_ms、last_success_poll_duration_ms、poll_duration_ema_ms、consecutive_poll_failures，并写入 /diagnostics。现场重启 /suoqu_driver_node 后复采 12s，connect_flag=0 占比从之前实测的 30/122 降到 1/52，抖动明显收敛。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard && python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract && python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract && source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES= && 现场 rostopic 复采 connect_flag / diagnostics`

### 后续注意

- 暂无。

## 2026-05-07 11:48 - Surface-DP 梁筋候选新增 score 与失败原因诊断

<!-- AGENT-MEMORY: entry -->

### 摘要

- scan_surface_dp 现已输出 beam_candidate_debug_bands：每条梁筋候选带 candidate_id、status(accepted/rejected)、candidate_score、failed_checks、reject_reason 以及各类 gate 指标。manual_workspace_s2 的 Surface-DP debug 图改为叠加这份完整候选列表，并用 A/R + reason + score 标注。当前梁筋候选的 reject_reason_counts 也写入 diagnostics，现场可直接判断是 width_gate、dark_gutter_gate、lattice_* 还是其他门控导致候选消失。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -k beam && PYTHONPATH=src/tie_robot_perception/src python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k curve_trace && PYTHONPATH=src/tie_robot_perception/src python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k workspace_s2_intersects_curved_line_families_by_polyline_geometry`

### 后续注意

- 暂无。

## 2026-05-07 11:44 - 索驱增量点动改为绝对位姿托底

<!-- AGENT-MEMORY: entry -->

### 摘要

- move_cabin_incremental_via_driver 不再直接走 /cabin/driver/incremental_move 或 g_cabin_driver->moveByOffset，而是基于最新 /cabin/cabin_data_upload 当前位姿换算绝对目标后复用 move_cabin_pose_via_driver。现场已重启 /suoqu_driver_node 并实测 /cabin/driver/incremental_move 的 z=+10mm 成功把 Z 从 3187 恢复到 3197；中途可见 motion_status=1、connect_flag 短暂抖动，最终恢复为 motion_status=0、connect_flag=1。

### 影响范围

- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard && python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract && python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract && source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES= && rosservice call /cabin/driver/incremental_move z:+10 fallback smoke test`

### 后续注意

- 暂无。

## 2026-05-07 11:34 - 索驱现场验证：绝对 raw_move 可下移 10mm

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场实机验证中，/cabin/driver/incremental_move 的 TCP 相对位置运动仍不稳定：旧逻辑先暴露出状态包前缀被误判为运动拒绝，修复后再试，相对运动指令会被对端直接关闭连接（read response: connection closed by peer，request_command=0x0012，control_word=0x0002）。在同一现场、同一进程重启后，改用 /cabin/driver/raw_move 绝对位姿方式，将索驱从 (390,1700,3197) 以 50mm/s 下移到 (390,1700,3187) 成功，连续三次状态回读均稳定为 Z=3187、motion_status=0、device_alarm=0、cabin_connect_flag=1。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard ; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES= ; rosnode kill /suoqu_driver_node and respawn ; rosservice call /cabin/driver/raw_move to Z=3187 success ; rostopic echo /cabin/cabin_data_upload shows Z=3187 three times`

### 后续注意

- 暂无。

## 2026-05-07 11:22 - Surface-DP raw beam 默认禁入 tracing 与 line_rho

<!-- AGENT-MEMORY: entry -->

### 摘要

- 在用户反馈点仍落到梁筋上后，scan_surface_dp 进一步收口：不再把 raw beam 候选禁入只绑定到 ±13cm 开关。当前会先用 beam_candidate_mask 过滤 line_families 中与梁筋重叠的 line_rhos，并让 curved_families 默认避开 raw beam mask；只有 beam exclusion 开启时，才把 tracing 禁区和最终点过滤扩展到 13cm margin。这样即使不开 ±13cm，梁筋本体也不会继续参与 tracing 和基准线选取。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -k beam && PYTHONPATH=src/tie_robot_perception/src python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k curve_trace && PYTHONPATH=src/tie_robot_perception/src python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k workspace_s2_intersects_curved_line_families_by_polyline_geometry`

### 后续注意

- 暂无。

## 2026-05-07 11:15 - 6点分组统一物理方向而非写死rowcol

<!-- AGENT-MEMORY: entry -->

### 摘要

- 动态绑扎规划的6/9点模式不再简单写死成row<=2、col<=3，而是先根据提供的全局行列与世界X/Y的映射判断哪一维对应线模长轴。3点跨度始终落在物理长轴上，因此同一现场可能表现为2x3或3x2，但不会把3个点压到短轴上；若长轴方向的完整组不可达，再退到同一物理朝向下的更小矩形。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && ./devel/lib/tie_robot_process/test_dynamic_bind_planning -> 27/27 tests passed; git diff --check -> clean`

### 后续注意

- 暂无。

## 2026-05-07 11:14 - 索驱状态轮询改走驱动层 pollState

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱状态读取不再在 suoquNode 里直接用旧 Frame_Generate_With_Retry 抢占全局 sockfd。新增 CabinProtocol.decodeHeartbeatState 和 CabinDriver.pollState，read_cabin_state 改为通过 g_cabin_driver->pollState 走同一条驱动 transport/io_mutex 链路，避免状态包与运动回包互串。状态轮询默认节拍由 100ms 提升到 20ms；connectToServer 不再把仅连接成功误记为 fresh state，而是等待真实状态包刷新 cabin_driver_last_state_stamp_sec。

### 影响范围

- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_protocol.hpp`
- `src/tie_robot_hw/src/driver/cabin_protocol.cpp`
- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_driver.hpp`
- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_cabin_protocol_contract.py`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard ; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=`

### 后续注意

- 暂无。

## 2026-05-07 11:09 - 非4点分组统一为2x3物理方向

<!-- AGENT-MEMORY: entry -->

### 摘要

- 动态绑扎规划的非4点模式已按现场口径收口：默认只把3个点跨度放在线模长轴方向，短轴最多2点；不再在2x3不可用时自动切到3x2，也不再让9点请求生成3x3。请求9时按世界坐标蛇形优先使用2x3，再用2x2、1x3、1x2等更小可达矩形补剩余点，所有候选仍按规划cabin_z校验线模工作范围。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && ./devel/lib/tie_robot_process/test_dynamic_bind_planning -> 26/26 tests passed; git diff --check -> clean`

### 后续注意

- 暂无。

## 2026-05-07 11:07 - Surface-DP 梁筋禁入前移到曲线追踪

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求不要改默认开关，只把 beam mask 前移到 tracing 阶段。当前 scan_surface_dp 在启用梁筋±13cm过滤时，会先生成 beam_candidate_13cm_mask 作为 curve_trace_mask 禁区，再跑 curved_families；workspace_s2 曲线追踪只保留 support_mask 内的 polyline_points，并新增 polyline_segments，交点只在连续有效段之间求，避免曲线穿过梁筋后再靠最终删点兜底。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py -k beam && PYTHONPATH=src/tie_robot_perception/src python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k curve_trace && PYTHONPATH=src/tie_robot_perception/src python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k workspace_s2_intersects_curved_line_families_by_polyline_geometry`

### 后续注意

- 暂无。

## 2026-05-07 10:58 - 动态分组起点兜底优先于后续完整组

<!-- AGENT-MEMORY: entry -->

### 摘要

- 动态绑扎规划在请求 6/9 等多点分组时，不能先执行后续可达完整组再回头补世界最小起点附近的小组；完整候选和可达兜底候选需要进入同一条世界坐标蛇形队列，同一起点优先点数更多的组。这样 9 点组因规划高度或线模范围不可达时，会先在世界最小角附近落到最大可达小组，再继续蛇形填充后续区域。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && ./devel/lib/tie_robot_process/test_dynamic_bind_planning -> 26/26 tests passed`

### 后续注意

- 暂无。

## 2026-05-07 10:55 - 旧idx跳绑过滤已移除

<!-- AGENT-MEMORY: entry -->

### 摘要

- 末端控制层旧跳绑链路已删除：不再使用 send_odd_points、/web/moduan/send_odd_points、ExecuteBindPointsTask.apply_jump_bind_filter 或 should_keep_jump_bind_point(idx==1/4) 托底过滤。execute_bind_points 现在只执行上游传入的点；跳绑选择保留在流程层，按 scan/bind path 中的 jump_bind、checkerboard_color、checkerboard_parity 元数据和 /web/moduan/jump_bind_enabled、/web/moduan/jump_bind_parity 决定。以后不要把旧 idx==1||4 过滤恢复到 moduan 层。

### 影响范围

- `src/tie_robot_msgs/action/ExecuteBindPointsTask.action`
- `src/tie_robot_control/include/tie_robot_control/moduan/linear_module_executor.hpp`
- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/area_execution.cpp`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/config/legacyCommandCatalog.js`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES= ; npm run build ; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_scan_artifact_write_guard src.tie_robot_control.test.test_moduan_light_status ; node src/tie_robot_web/frontend/test/jumpBindToggleController.test.mjs ; rg old jump-bind strings in production/web => no matches`

### 后续注意

- 暂无。

## 2026-05-07 10:22 - 控制面板任务区收口与人工切区接管

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端控制面板已下线“清除识别结果”“固定扫描规划”“账本测试”旧入口，改为扫描区、执行层、区域切换三组；上一个/下一个区域会发布 /web/cabin/manual_area_takeover，先让当前自动执行链放弃后续区域并让线性模组归零，再按当前区域进度或当前位置邻近区域移动索驱到相邻 cabin_pose。

### 影响范围

- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`
- `src/tie_robot_web/frontend/src/controllers/AreaNavigationController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `README.md`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `npm run build；for f in src/tie_robot_web/frontend/test/*.mjs; do node "$f" || exit $?; done；python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard；python3 -m unittest 5 条控制面板相关 WorkspacePickerWebTest；source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_control tie_robot_process tie_robot_web`

### 后续注意

- 暂无。

## 2026-05-07 10:14 - 索驱设备运动中不再触发自动跳区

<!-- AGENT-MEMORY: entry -->

### 摘要

- 自动执行下发索驱位姿时，设备运动中/status_word=0x00000004 属于索驱忙的暂态，应等待并重试当前目标；Z超正限位、速度错误等真实拒绝仍保持硬失败，不能被通信恢复逻辑吞掉。等待轴到位时缓存索驱协议异常需记录 command_word，只有 TCP 运动类指令(0x0010/0x0011/0x0012)返回纯 bit2 设备运动中才按暂态处理。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/include/tie_robot_process/suoqu/cabin_transport.hpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=""; python3 scripts/agent_memory.py check`

### 后续注意

- 暂无。

## 2026-05-07 10:06 - 跳绑执行不再二次套老idx过滤

<!-- AGENT-MEMORY: entry -->

### 摘要

- 跳绑黑/白棋选择的权威口径在流程层 filter_precomputed_group_points_for_execution，按 jump_bind/checkerboard_color/checkerboard_parity 过滤后再发给 /moduan/execute_bind_points；末端 driver raw_execute_points 不能再套旧 send_odd_points 的 idx==1||4 过滤，否则会出现 3D 加粗黑棋点被流程层选中后又在末端全丢、区域日志显示执行失败但消息为区域绑扎作业完成。已将 moduan_driver_raw_execute_points_service 改为 execute_bind_points(..., false)，让末端严格执行流程层选好的点。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; python3 -m unittest src.tie_robot_process.test.test_scan_artifact_write_guard; bash -lc 'source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=""'`

### 后续注意

- 暂无。

## 2026-05-07 09:52 - 索驱断链恢复不中断自动执行

<!-- AGENT-MEMORY: entry -->

### 摘要

- 自动执行链遇到索驱驱动短暂断链、raw_move服务暂不可用或/cabin/cabin_data_upload状态断流时，不再把当前区域立即判失败；执行层会等待驱动恢复，状态恢复后重新下发当前TCP_Move目标并继续当前任务。长按停止并回起点仍可打断等待。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_control;tie_robot_process'`

### 后续注意

- 暂无。

## 2026-05-07 09:48 - 普通末端回零收口为共享旧PLC入口

<!-- AGENT-MEMORY: entry -->

### 摘要

- 普通末端回零进一步完善：request_moduan_zero、moduan_move_zero_forthread 和 /web/moduan/moduan_move_zero 回调统一调用 request_legacy_moduan_zero。该入口先取消有序回起点请求、释放 handle_pause_interrupt、清 PLC IS_STOP 和 FINISHALL，再按旧 chassis_ctrl active moduanNode.cpp 顺序写 EN_DISABLE=1、IS_ZERO=1；控制层删除 request_linear_module_zero_via_driver，避免普通回零再次走 driver wrapper、坐标回零或 wait_linear_module_axis_arrival 导致卡住。/moduan/return_zero_ordered 继续只服务长按停止并回起点的 Z 优先语义。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp;src/tie_robot_control/src/moduan/linear_module_executor.cpp;src/tie_robot_control/include/tie_robot_control/moduan/linear_module_executor.hpp;src/tie_robot_control/test/test_single_point_bind_chain.py`

### 关键决策

- 见摘要。

### 标签

- `moduan`
- `zero`
- `plc`
- `pause`

### 验证证据

- `python3 -m unittest src/tie_robot_control/test/test_moduan_light_status.py src/tie_robot_control/test/test_single_point_bind_chain.py; git diff --check -- src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp src/tie_robot_control/src/moduan/linear_module_executor.cpp src/tie_robot_control/include/tie_robot_control/moduan/linear_module_executor.hpp src/tie_robot_control/test/test_single_point_bind_chain.py; rg -n request_linear_module_zero_via_driver src/tie_robot_control/src src/tie_robot_control/include exited 1 with no matches; catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_hw;tie_robot_control"; python3 scripts/agent_memory.py check`

### 后续注意

- 暂无。

## 2026-05-07 09:43 - 遥控全局速度控件统一在控制面板底部

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求索驱遥控全局移动速度和 TCP 线性模组遥控速度不再分别放在两个设置页内，而是统一提到左侧控制面板最下面的“全局遥控速度”区；索驱遥控页中“开启键盘遥控”和“单次点击步距”保持同一行。保留原输入 id cabinRemoteSpeed/tcpLinearRemoteSpeed，原持久化和 ROS 全局速度发布链路继续复用。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-AFGHJ_yr.js`
- `src/tie_robot_web/web/assets/app/index-Bj8WXRbo.css`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k global_remote_speed_controls; python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k cabin_remote_step_and_speed; python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k tcp_linear_module_execution_speed; python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k settings_panel_renders_cabin_remote_page; npm run build (src/tie_robot_web/frontend); git diff --check -- relevant files`

### 后续注意

- 暂无。

## 2026-05-07 09:38 - 普通末端回零恢复旧PLC链

<!-- AGENT-MEMORY: entry -->

### 摘要

- 旧工程 chassis_ctrl active moduanNode.cpp 的 /web/moduan/moduan_move_zero 只写 EN_DISABLE=1 后 IS_ZERO=1，不走坐标回零或等待到位。当前普通末端回零已恢复该PLC链，避免 driver wrapper 或严格 wait_linear_module_axis_arrival 让回零卡住；/moduan/return_zero_ordered 的有序Z优先服务仍保留给停止并回起点语义。构建还修正 tie_robot_control CMake 删除重复裸 tie_robot_hw_driver_core 链接项，避免 -ltie_robot_hw_driver_core 找不到。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp;src/tie_robot_control/test/test_single_point_bind_chain.py;src/tie_robot_control/CMakeLists.txt`

### 关键决策

- 见摘要。

### 标签

- `moduan`
- `zero`
- `plc`
- `build`

### 验证证据

- `python3 -m unittest src/tie_robot_control/test/test_moduan_light_status.py src/tie_robot_control/test/test_single_point_bind_chain.py; catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_hw;tie_robot_control"; git diff --check -- src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp src/tie_robot_control/test/test_single_point_bind_chain.py src/tie_robot_control/CMakeLists.txt`

### 后续注意

- 暂无。

## 2026-05-07 09:37 - TCP线模遥控全局线模速度持久化

<!-- AGENT-MEMORY: entry -->

### 摘要

- TCP 线性模组遥控页新增线模执行速度输入，随步距参数一起持久化到 tie_robot_frontend_tcp_linear_remote_settings；ROS 连接就绪和输入变化时发布 /web/moduan/set_moduan_speed。控制端移除预计算当前区域直执行的 ScopedModuleSpeedOverride/kPrecomputedFastModuleSpeedMmPerSec 临时提速，fast service 与普通执行链统一使用全局 module_speed。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-BHWdKruy.js`
- `src/tie_robot_web/web/assets/app/index-CxP-Cu2G.css`
- `src/tie_robot_control/include/tie_robot_control/moduan/linear_module_executor.hpp`
- `src/tie_robot_control/include/tie_robot_control/moduan/runtime_state.hpp`
- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_control/test/test_single_point_bind_chain.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k tcp_linear_module_execution_speed; python3 -m unittest src/tie_robot_control/test/test_single_point_bind_chain.py src/tie_robot_control/test/test_moduan_light_status.py; npm run build (src/tie_robot_web/frontend); source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_control'; git diff --check -- relevant files`

### 后续注意

- 暂无。

## 2026-05-07 09:19 - 执行层账本单组 TCP 蛇形点序

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 用户要求执行层执行时绑扎单组顺序也必须是蛇形。tie_robot_process 的预生成账本组点现在在 filter_precomputed_group_points_for_execution 过滤执行记忆、blocked 点和跳绑 parity 后，会按 TCP 局部 x 分行、y 交替方向做蛇形排序，再装载为线性模组点位；该口径覆盖全局执行、直接账本测试、当前区域测试和 live 视觉微调后的账本点。

### 影响范围

- `src/tie_robot_process/src/suoqu/area_execution.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 -m unittest src/tie_robot_process/test/test_scan_artifact_write_guard.py; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process'`

### 后续注意

- 暂无。

## 2026-05-07 09:16 - 账本+微调Z轴以视觉微调为准

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 用户明确账本+微调执行时不再用账本点Z轴约束视觉微调点。run_live_visual_global_work 的 build_live_visual_execution_points_from_planned_area 现在只用 X/Y 微调容差匹配同一 global_idx，移除 kLiveVisualMicroAdjustZToleranceMm、refine_dz_mm 和 z阈值日志；匹配成功后 world_z/x/y 都写入微调视觉返回值，再按规划索驱位姿转换为线性模组局部点。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 -m unittest src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 后续注意

- 暂无。

## 2026-05-07 09:15 - 执行微调移除近点排斥

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 用户要求清理视觉执行阶段遗留的多根钢筋靠太近就排斥/去重旧算法。MODE_EXECUTION_REFINE 现在不再按世界 XY 100mm 近点阈值成对丢弃 Hough 候选点；只保留有效 3D 坐标、TCP 执行范围和排序下发。执行底图诊断同步移除 DUP，日志移除去重移除计数。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg`

### 后续注意

- 暂无。

## 2026-05-07 09:09 - 线模状态同步灯光按钮

<!-- AGENT-MEMORY: entry -->

### 摘要

- 线性模组状态消息 `/moduan/moduan_gesture_data` 现在追加 `light_state`，驱动线程从 PLC `LIGHT` 寄存器读取灯状态并发布；前端收到该状态后同步控制面板 `lightEnabled`（开启/关闭灯光）按钮的显示和 LegacyCommandController 内部开关值，避免下次点击按旧本地状态发错方向。

### 影响范围

- `src/tie_robot_msgs/msg/linear_module_upload.msg; src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp; src/tie_robot_web/frontend/src/controllers/StatusMonitorController.js; src/tie_robot_web/frontend/src/controllers/LegacyCommandController.js; src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js; src/tie_robot_web/frontend/test/statusMonitorController.test.mjs; src/tie_robot_web/frontend/test/lightToggleTelemetry.test.mjs; src/tie_robot_control/test/test_moduan_light_status.py; src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `moduan`
- `light`
- `status`
- `msg`

### 验证证据

- `node src/tie_robot_web/frontend/test/statusMonitorController.test.mjs && node src/tie_robot_web/frontend/test/lightToggleTelemetry.test.mjs; python3 -m unittest src/tie_robot_control/test/test_moduan_light_status.py src/tie_robot_control/test/test_single_point_bind_chain.py; catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_hw;tie_robot_control"; npm run build (src/tie_robot_web/frontend)`

### 后续注意

- 暂无。

## 2026-05-07 09:02 - 梁筋误识别增加网格中点门控

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 用户指出梁筋红带仍有误识别。根因是上一版为识别细梁筋放开到 5-8px 单列高度候选后，普通竖向钢筋若局部更高/更粗也可能进入 beam_candidate。scan_surface_dp 现在在高度门控后追加 line-family 上下文门控：候选中心必须位于相邻普通竖筋列之间并接近两列中点；落在正常竖筋 line rho 附近的候选按普通钢筋剔除。现场 /pointAI/process_image 连续 5 次仅保留 x≈149..155 与 x≈333..340 两条 6-8px 窄红带；诊断显示 lattice_gate=between_vertical_rebar_columns，左右普通列为 136/171 与 320/352。启用梁筋过滤后 count=192，随后恢复 false。

### 影响范围

- `CHANGELOG.md;src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py;src/tie_robot_perception/test/test_scan_surface_dp_runtime.py;.debug_frames/beam_lattice_gate_live_20260507_090053/summary.json`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_scan_surface_dp_runtime; PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:... python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg; python3 -m compileall -q src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; git diff --check; live ROS: 5/5 service runs red bands 6-8px only`
- `beam exclusion count=192 then restored false`

### 后续注意

- 暂无。

## 2026-05-07 09:02 - 线模位置胶囊长按态不显示滑动条

<!-- AGENT-MEMORY: entry -->

### 摘要

- 底部“线模本地/线模全局”胶囊长按触发回零时，充能态和完成态必须隐藏溢出，避免扫光伪元素配合默认 overflow-x:auto 弹出底部横向滑动条；短按/普通显示仍保留原有布局。

### 影响范围

- `src/tie_robot_web/frontend/src/styles/app.css; src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs; src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `linear-module`
- `long-press`
- `css`

### 验证证据

- `node src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs; node src/tie_robot_web/frontend/test/coordinateDisplayPrecision.test.mjs; npm run build (src/tie_robot_web/frontend)`

### 后续注意

- 暂无。

## 2026-05-07 08:58 - 底部线模位置胶囊长按回零

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端底部“线模本地/线模全局”位置胶囊现在是长按入口：长按 0.5 秒触发现有 15 号“末端回零”命令，发布 /web/moduan/moduan_move_zero Float32(1)，后端继续走 IS_ZERO 线性模组回零链；短按不触发命令并会被拦截。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js; src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js; src/tie_robot_web/frontend/src/styles/app.css; src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs; src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `linear-module`
- `iszero`
- `long-press`

### 验证证据

- `node src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs; node src/tie_robot_web/frontend/test/coordinateDisplayPrecision.test.mjs; npm run build (src/tie_robot_web/frontend)`

### 后续注意

- 暂无。

## 2026-05-07 08:48 - 梁筋候选改为高度门控窄红带

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 梁筋候选不再只看扫描 DP 收束底图里的黑沟/亮边宽带；scan_surface_dp 现在把 depth_response 作为高度响应传入检测，候选竖列必须比邻近普通钢筋更高才标为 beam_candidate，并按高度峰值把红色半透明竖带收窄到约 6-8px。现场重启 /pointAINode 后连续 5 次 /pointAI/process_image request_mode=3 均在 base/completed 图检出窄红带，列约 x=149..155 与 x=333..340；启用 /web/pointAI/set_scan_beam_exclusion=true 后服务 count 从 256 降到 224，随后已恢复 false。

### 影响范围

- `CHANGELOG.md;src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py;src/tie_robot_perception/test/test_scan_surface_dp_runtime.py;.debug_frames/beam_height_gate_live_final_20260507_084647/summary.json`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_scan_surface_dp_runtime; PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:... python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg; python3 -m compileall -q src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; git diff --check; live ROS: 5/5 service runs red bands 6-8px and beam exclusion count 224 then restored false`

### 后续注意

- 暂无。

## 2026-05-07 08:44 - 6/9点分组不漏点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 用户要求执行点数为6或9时不允许有漏点，漏点允许用更小的组填充。dynamic_bind_planning 现在仅在 requested_group_point_count 为6或9时启用小组兜底：先尝试请求的大矩形组，未覆盖格点再按更小可达矩形组降级，最后允许 matrix_1x1 兜底；默认4点模式仍保持固定2x2和边缘2点补组口径。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning -DCATKIN_WHITELIST_PACKAGES=tie_robot_process: 25/25 PASS; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py: 18 tests OK; git diff --check: exit 0`

### 后续注意

- 暂无。

## 2026-05-07 08:42 - 长按停止并回起点闸门补齐

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 现场反馈恢复作业长按后机器人继续到处移动。根因是默认账本+微调执行模式 run_live_visual_global_work 在末端 Action 因长按停止并回起点中止后，会把失败当普通区域失败继续下一区域；已给 live_visual 补齐和执行账本/账本测试一致的 wait_while_execution_paused 与 fail_if_execution_return_to_start_requested 闸门，并在末端执行失败分支遇到 return_to_start 时直接终止自动链。日志口径统一为长按停止并回起点。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`
- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 -m unittest src/tie_robot_control/test/test_single_point_bind_chain.py; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_process;tie_robot_control'`

### 后续注意

- 暂无。

## 2026-05-07 08:30 - 视觉调试设置填完即用

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 用户要求视觉调试设置页的“视觉调试”卡所有需要输入的设置填完即用，不再需要应用/确认按钮。前端已移除“应用帧数”，视觉调试设置 input/change 会立即保存设置、发布 stable_frame_count、同步执行微调 TCP ROI、同步梁筋过滤开关，并保留“触发视觉服务”只作为主动触发识别动作。修改前端后已重新构建 src/tie_robot_web/web。

### 影响范围

- `CHANGELOG.md;src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js;src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs;src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node test/visualDebugSettings.test.mjs; for test_file in test/*.test.mjs; do node "" || exit 1; done; npm run build; git diff --check -- CHANGELOG.md src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js src/tie_robot_web/frontend/src/ui/UIController.js src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs src/tie_robot_web/web/index.html`

### 后续注意

- 暂无。

## 2026-05-07 08:23 - 梁筋红带现场实测通过

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 08:20 后重启 pointAINode 加载黑沟双亮边梁筋检测修复，真实调用 /pointAI/process_image request_mode=3 后，/perception/lashing/scan_surface_dp_base_image 和 completed_surface_image 均出现红色半透明梁筋竖带。首次抓图 .debug_frames/beam_live_verification_20260507_082057 检出红带列 x=290..339；连续 3 次抓图 .debug_frames/beam_live_repeat_20260507_082249 均有红带，其中第 2 次检出 x=290..339 与 x=407..442 两条。用 rostopic pub 确认 /web/pointAI/set_scan_beam_exclusion=true 后服务 count 从 256 降到 160，随后已恢复 false。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- `.debug_frames/beam_live_verification_20260507_082057/summary.json`
- `.debug_frames/beam_live_repeat_20260507_082249/summary.json`

### 关键决策

- 见摘要。

### 验证证据

- `rosnode kill /pointAINode 后 respawn；rosservice call /pointAI/process_image request_mode=3；抓取 /perception/lashing/scan_surface_dp_base_image 与 completed_surface_image；红色主导像素 base=21520 completed=21577，红带列 x=290..339；重复 3 次均检出红带；rostopic pub /web/pointAI/set_scan_beam_exclusion true 后 rosservice count=160，再发布 false 恢复`

### 后续注意

- 暂无。

## 2026-05-07 08:18 - 默认 4 点分组回到 v35 保组口径

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 用户要求绑扎分组规则回到 slam/v35：默认 4 点仍按固定 2x2 切块、边缘 2 点补组和蛇形排序保组；保留视觉调试自定义每组点数；保留以线性模组工作盒中心反算索驱位姿。分组阶段不再因为中心反算高度低于最小绑扎高度直接丢组，而是按 v35 口径保留组并将最终 cabin_pose.z 夹到最小安全高度，避免 256 点网格掉成缺洞路径。

### 影响范围

- `CHANGELOG.md;src/tie_robot_process/src/planning/dynamic_bind_planning.cpp;src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process && catkin_make run_tests_tie_robot_process && catkin_test_results build/test_results/tie_robot_process; git diff --check -- src/tie_robot_process/src/planning/dynamic_bind_planning.cpp src/tie_robot_process/test/test_dynamic_bind_planning.cpp CHANGELOG.md`

### 后续注意

- 暂无。

## 2026-05-07 08:16 - 梁筋候选识别补充黑沟双亮边形态

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场最新扫描 DP 底图没有出现红色半透明梁筋竖带，根因是梁筋在底图中表现为贯穿全高的黑色竖沟加两侧窄亮边，而旧 beam_candidate 检测偏向整条宽亮带。scan_surface_dp.detect_beam_candidate_bands 现在会把两侧连续亮边和中间低覆盖暗沟合并为一条 beam_candidate 竖带，仍只提供候选可视化和可选最终点级 ±13 cm 过滤，不删除普通钢筋线族。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest src.tie_robot_perception.test.test_scan_surface_dp_runtime; bash -lc 'source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; source devel/setup.bash >/dev/null 2>&1 || true; PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg'; python3 -m compileall -q src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; git diff --check -- CHANGELOG.md src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 后续注意

- 暂无。

## 2026-05-07 08:09 - 视觉调试可启用梁筋±13cm过滤

<!-- AGENT-MEMORY: entry -->

### 摘要

- Surface-DP 扫描底图继续用红色半透明竖带标出 beam_candidate 梁筋候选；视觉调试设置新增梁筋 ±13 cm 过滤开关，默认关闭，开启后通过 /web/pointAI/set_scan_beam_exclusion 下发到 pointAI，只在最终绑扎点级排除梁筋候选扩张范围内的点，不删除普通钢筋线族。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_scan_surface_dp_runtime; node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs; bash -lc 'source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; source devel/setup.bash >/dev/null 2>&1 || true; PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg'; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 08:05 - 原始账本口径回退

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 用户撤回账本杂点过滤方案：扫描账本按扫描原始点写出，pseudo_slam_bind_path.json 优先使用 Surface-DP 原始全量行列点，缺行列时直接回落 merged_world_points；pseudo_slam_points.json 和 bind path 不再写 planning/outlier 过滤字段；执行层 collect_blocked_execution_global_indices_from_points_json 固定返回空集合，run_bind_path_direct_test 只读取 pseudo_slam_bind_path.json，不再用 pseudo_slam_points.json 的 blocked/outlier 标记跳点。跳绑黑白棋开关保留，因为它是用户主动选择，不属于杂点过滤。

### 影响范围

- `README.md`
- `CHANGELOG.md`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/area_execution.cpp`
- `src/tie_robot_process/src/suoqu/bind_path_store.cpp`
- `src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_process/test/test_scan_artifact_write_guard.py; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process; source /opt/ros/noetic/setup.bash && catkin_make run_tests_tie_robot_process; source /opt/ros/noetic/setup.bash && catkin_test_results build/test_results/tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 07:52 - 扫描 DP 底图梁筋候选可视化

<!-- AGENT-MEMORY: entry -->

### 摘要

- Surface-DP 运行态新增 beam_candidate 梁筋候选诊断：按收束底图中宽、连续、高响应的竖向 band 输出 beam_candidate_bands/count/pixels，并在 scan_surface_dp_base_image 与 completed_surface_image 上用红色半透明竖带叠加；当前只做识别和可视化，不启用梁筋 ±13 cm 点级过滤，也不删除普通钢筋线族。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- `src/tie_robot_perception/tools/build_current_visual_recognition_flow_page.py`
- `docs/reports/current_visual_recognition_flow/index.html`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_scan_surface_dp_runtime; python3 -m unittest src.tie_robot_perception.test.test_current_visual_recognition_flow_report; source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_workspace_s2_beam_mask_does_not_delete_line_rhos src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_workspace_s2_filters_bind_points_inside_vertical_beam_mask src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_workspace_s2_expands_beam_mask_by_thirteen_centimeters_for_graph_exclusion; python3 -m compileall -q affected Python files`

### 后续注意

- 暂无。

## 2026-05-07 07:49 - 撤回扫描账本参考面吸附方案

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 用户要求撤回此前讨论的扫描账本参考面吸附与原始备份方案；当前代码恢复为扫描账本直接保留识别得到的 world_z，不再做账本 Z 投影、额外原始备份产物或 live visual 面外点过滤。保留跳绑、账本杂点过滤收口和其他无关改动。

### 影响范围

- `CHANGELOG.md;docs/agent_memory/session_log.md;src/tie_robot_process/src/suoqu/bind_path_store.cpp;src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp;src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/test/test_scan_artifact_write_guard.py;src/tie_robot_process/test/test_motion_chain_signal_guard.py;src/tie_robot_process/data/pseudo_slam_points.json;src/tie_robot_process/data/pseudo_slam_bind_path.json`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_process/test -p 'test_*.py'; git diff --check -- CHANGELOG.md docs/agent_memory/current.md docs/agent_memory/session_log.md src/tie_robot_process/src/suoqu/bind_path_store.cpp src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp src/tie_robot_process/src/suoquNode.cpp src/tie_robot_process/test/test_scan_artifact_write_guard.py src/tie_robot_process/test/test_motion_chain_signal_guard.py src/tie_robot_process/data/pseudo_slam_points.json src/tie_robot_process/data/pseudo_slam_bind_path.json; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process; source /opt/ros/noetic/setup.bash && catkin_make run_tests_tie_robot_process; source /opt/ros/noetic/setup.bash && catkin_test_results build/test_results/tie_robot_process; python3 scripts/agent_memory.py check`

### 后续注意

- 暂无。

## 2026-05-07 07:35 - 账本杂点过滤收口

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 为避免账本+微调/直接账本测试继续执行扫描杂点，扫描生成 pseudo_slam_bind_path.json 时只接收已通过规划离群过滤并能归入棋盘格的点；run_bind_path_direct_test 也改为读取 pseudo_slam_points.json 的 blocked 点标记并复用 filter_precomputed_group_points_for_execution，跳绑 parity 过滤同步生效。README/CHANGELOG 已更新此语义。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/bind_path_store.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`
- `README.md`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_process/test/test_scan_artifact_write_guard.py; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process; source /opt/ros/noetic/setup.bash && catkin_make run_tests_tie_robot_process; source /opt/ros/noetic/setup.bash && catkin_test_results build/test_results/tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 07:32 - 跳绑长按启停与黑白棋选择

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端跳绑按钮改为长按启停，单击只切换只绑黑棋/白棋；新增 /web/moduan/jump_bind_parity(std_msgs/Int32, 0=black,1=white) 同步后端，执行过滤按当前 parity 匹配账本点；3D 在跳绑开启时叠加高亮当前要跳绑的点。

### 影响范围

- `CHANGELOG.md;src/tie_robot_web/frontend/src/config/controlPanelCatalog.js;src/tie_robot_web/frontend/src/controllers/LegacyCommandController.js;src/tie_robot_web/frontend/src/views/Scene3DView.js;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/src/suoqu/area_execution.cpp`

### 关键决策

- 见摘要。

### 标签

- `jump-bind`
- `frontend`
- `execution`
- `3d`

### 验证证据

- `node src/tie_robot_web/frontend/test/jumpBindToggleController.test.mjs; node src/tie_robot_web/frontend/test/bindPathGeometry.test.mjs; for test in src/tie_robot_web/frontend/test/*.mjs; do node "" || exit 1; done; python3 -m unittest discover -s src/tie_robot_process/test; devel/lib/tie_robot_process/test_dynamic_bind_planning; catkin_make --pkg tie_robot_process -DCATKIN_ENABLE_TESTING=ON; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 07:13 - 跳绑账本点颜色属性

<!-- AGENT-MEMORY: entry -->

### 摘要

- 扫描账本和执行记忆中的棋盘格点新增 jump_bind 与 checkerboard_color 字段；约定 parity 0 为 black/jump_bind=true，parity 1 为 white/jump_bind=false。跳绑执行优先按 jump_bind=true 过滤，旧账本缺字段时回退 checkerboard_parity==0。前端规划路径 API 和 bindPathGeometry 保留这两个元数据。

### 影响范围

- `src/tie_robot_process/src/suoqu/bind_path_store.cpp;src/tie_robot_process/src/suoqu/area_execution.cpp;src/tie_robot_process/src/suoqu/execution_memory_store.cpp;src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp;src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_web/scripts/workspace_picker_web_server.py;src/tie_robot_web/frontend/src/utils/bindPathGeometry.js`

### 关键决策

- 见摘要。

### 标签

- `jump-bind`
- `ledger`
- `execution`

### 验证证据

- `python3 -m unittest src/tie_robot_process/test/test_scan_artifact_write_guard.py src/tie_robot_process/test/test_motion_chain_signal_guard.py src/tie_robot_process/test/test_tcp_travel_range_config.py; for f in src/tie_robot_web/frontend/test/*.mjs; do node "" || exit 1; done; devel/lib/tie_robot_process/test_dynamic_bind_planning; catkin_make --pkg tie_robot_process -DCATKIN_ENABLE_TESTING=ON; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 07:10 - 暂停恢复长按统一为停止并回起点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 暂停作业/恢复作业按钮的长按语义统一为停止当前工作：前端仍发送 signal_id=11,data=2，但文案改为停止并回起点；后端停止当前自动链，线性模组按Z优先回到(0,0,0)，索驱机器回到记录的执行起点。run_bind_path_direct_test 也要记录回起点姿态并检查暂停/停止回起点请求，避免长按后继续跑后续分组。

### 影响范围

- `src/tie_robot_web/frontend/src/config/legacyCommandCatalog.js`
- `src/tie_robot_web/frontend/src/controllers/LegacyCommandController.js`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 长按按钮不再表达从暂停状态恢复，而是硬语义：停止当前工作、模组回零、索驱回执行起点。

### 标签

- `frontend`
- `ros`
- `control`
- `long-press`
- `memory`

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_control/test/test_single_point_bind_chain.py; npm run build (src/tie_robot_web/frontend); source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process;tie_robot_control;tie_robot_web'; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=''。一次增量构建曾在 pseudo_slam checkerboard helper 符号处链接失败，确认对象内符号存在后重跑通过，归因于脏增量构建状态。`

### 后续注意

- 暂无。

## 2026-05-07 06:54 - 视觉掉线多由前端全ROS重启触发

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 06:51 现场追查‘视觉总是自己掉线’：当前 pointAINode、scepter_manager、scepter_world_coord_processor 只有一套，无重复节点；/Scepter/ir/image_raw、world_coord、raw_world_coord 均约5Hz，/diagnostics 中 tie_robot/visual_algorithm 连续 level=0。掉线窗口对应前端从 192.168.6.192 连续 POST /api/system/start_camera_driver 与 /api/system/start_algorithm_stack 后，又触发 /api/system/restart_ros_stack；systemd 在 06:51:39 停止 backend、三个 driver 和 rosbridge，06:51:45-47 重启全栈。结论：这类视觉‘自己掉线’优先按前端/人工触发全栈重启或状态胶囊操作排查，不要先当作 pointAI 或相机驱动崩溃。

### 影响范围

- `src/tie_robot_web/frontend/src/config/systemControlCatalog.js`
- `src/tie_robot_web/frontend/src/controllers/StatusMonitorController.js`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `docs/agent_memory/current.md`

### 关键决策

- 见摘要。

### 验证证据

- `只读诊断：rosnode list 无重复；rostopic hz /Scepter/ir/image_raw、/Scepter/worldCoord/world_coord、/Scepter/worldCoord/raw_world_coord 约5Hz；rostopic echo /diagnostics 连续 visual_algorithm level=0；journalctl 显示 /api/system/restart_ros_stack 停止并重启全栈`

### 后续注意

- 暂无。

## 2026-05-07 06:49 - 开始执行卡住定位到索驱raw_move窗口掉线

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 06:45 现场点击开始执行后前端看似卡住。ROS action /web/cabin/start_global_work/status 实际为 ABORTED(4)，文本：live_visual模式下未成功执行任何区域，跳过64个区域。日志显示同一秒 tie-robot-driver-suoqu 的旧 suoqu_driver_node 因索驱 TCP 对端关闭状态查询帧（0x0001，0/144字节）连续失败5次紧急退出，随后 backend 在 raw_move service 不可用窗口内遍历64个区域并全部报“无法调用索驱驱动层服务 /cabin/driver/raw_move”。当前恢复时需确认 /cabin/driver/raw_move 已重新注册、/cabin/cabin_data_upload 有新鲜数据，再让操作员确认现场安全后重试；若前端仍显示执行中，多半是浏览器错过了失败 result，可刷新页面查看状态日志。

### 影响范围

- `docs/agent_memory/current.md`
- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`

### 关键决策

- 见摘要。

### 验证证据

- `只读诊断：systemctl status; rosservice info /cabin/driver/raw_move; rostopic echo /web/cabin/start_global_work/status 显示 ABORTED(4); rostopic hz /cabin/cabin_data_upload 约9.5Hz`

### 后续注意

- 暂无。

## 2026-05-07 06:43 - 算法栈启动收口到systemd后端

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 06:35-06:43 现场发现前端视觉/算法启动曾拉起孤儿 roslaunch tie_robot_bringup algorithm_stack.launch（PID 5654），随后 tie-robot-backend.service 又启动 run.launch，形成两套同名 /pointAINode、/bind_map_builder、/global_bind_planner、/cabin_motion_controller、/moduan_motion_controller、/bind_task_executor。ROS master 日志表现为 new node registered with same name，节点不是视觉算法崩溃，而是同名重复注册互踢。start_algorithm_stack.sh 和 restart_algorithm_stack.sh 已收口为调用 tie-robot-backend.service，检测到孤儿 algorithm_stack.launch 时拒绝继续启动；stop_algorithm_stack.sh 会停止 backend 并清理旧孤儿 launcher。若现场仍有旧 PID 5654，因其可能拥有 rosmaster，恢复应确认安全后走全 ROS 栈重启，而不是只杀该 PID。

### 影响范围

- `start_algorithm_stack.sh`
- `restart_algorithm_stack.sh`
- `stop_algorithm_stack.sh`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_system_control_http_endpoints_cover_start_and_restart_actions -v; bash -n start_algorithm_stack.sh restart_algorithm_stack.sh stop_algorithm_stack.sh; git diff --check -- start_algorithm_stack.sh restart_algorithm_stack.sh stop_algorithm_stack.sh src/tie_robot_web/test/test_workspace_picker_web.py`

### 后续注意

- 暂无。

## 2026-05-07 06:20 - 线模短距实测确认PLC未放行运动

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-07 06:11-06:20 现场按用户要求做线性模组短距闭环测试：运行中的 moduan_driver_node 二进制已包含 trigger_linear_module_motion_execution/pulseExecutionEnable，且进程启动时间晚于构建时间；raw_single_move Y=10mm、raw_single_move X=10mm、raw_execute_points 单点Y=10mm、直接Modbus慢脉冲5076低电平保持1s后Y=10mm均未产生位移，反馈X/Y保持0。PLC寄存器确认目标已写入：5066=10mm或slot0 5512=10mm，速度5050/5054/5058=250mm/s，5076=1，5176远程模式=1，5175错误查询=0，5177急停=0，FINISHALL=0；但6456读回在6/7/15之间，写0会被PLC侧重算/置回，疑似PLC/伺服停止或未就绪条件未放行。此前“只差EN_DISABLE执行触发脉冲”的记忆是阶段性误判，当前不能再把该补丁视为已解决现场不动问题。

### 影响范围

- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_control/test/test_single_point_bind_chain.py`
- `docs/agent_memory/session_log.md`
- `docs/agent_memory/current.md`

### 关键决策

- 见摘要。

### 验证证据

- `nm -C devel/lib/tie_robot_control/moduan_driver_node | rg trigger_linear_module_motion_execution; raw_single_move Y=10/X=10 and raw_execute_points Y=10 monitored /moduan/moduan_gesture_data with no X/Y movement; /tmp/moduan_probe_regs confirmed targets and status regs; /tmp/moduan_slow_pulse_y10 confirmed 1s 5076 pulse still no movement`

### 后续注意

- 暂无。

## 2026-05-07 06:09 - 扫描层识别点前端样式纠偏

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户现场明确：图层设置里的‘扫描层识别点’指扫描结果图中的黄色编号点阵（伴随绿色扫描线），不是执行层/旧前端那种红色圆点加白十字。前端 WorkspaceCanvasView 的扫描点覆盖层已改为黄点+绿色描边+编号，并删除红十字绘制。

### 影响范围

- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/test/irImageLayerOverlayControls.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-C88Tx4rQ.js`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/irImageLayerOverlayControls.test.mjs; node src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_image_panel_supports_project_image_topics_and_overlay_switching; for test_file in src/tie_robot_web/frontend/test/*.mjs; do node "$test_file" || exit 1; done; cd src/tie_robot_web/frontend && npm run build`

### 后续注意

- 暂无。

## 2026-05-07 06:00 - 线模遥控补执行触发脉冲

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场线性模组 TCP 线模遥控已能收到 Y=50mm 目标但反馈位置一直为 0，根因是遥控原子移动只写 WX/WY/WZ 目标坐标，没有像预计算绑扎执行链一样触发 EN_DISABLE=5076 的 0->1 执行脉冲。move_linear_module_to_target() 现在在写 X/Y 后和写 Z 后分别调用 pulseExecutionEnable()，再等待对应轴到位；重启 tie-robot-driver-moduan.service 前需确认现场安全，因为节点启动可能触发自动回零，且修复后遥控命令会真正启动运动。

### 影响范围

- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_control/test/test_single_point_bind_chain.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_control/test/test_single_point_bind_chain.py; python3 src/tie_robot_bringup/test/test_architecture_cleanup.py -k linear_module_execution_chain_uses_driver_atomic_functions; catkin_make --only-pkg-with-deps tie_robot_control; catkin_make -DCATKIN_WHITELIST_PACKAGES=""`

### 后续注意

- 暂无。

## 2026-05-07 05:54 - 后端IR工作区灰线改为map固定投影

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户现场指出机器X/Y移动时IR工作区灰线跟着像素走。根因是 /Scepter/worldCoord/raw_world_coord 在pointAI中是Scepter_depth_frame相机坐标，之前用corner_world_camera_frame套当前相机XY通道，仍是相机锚定。现在确认工作区时额外保存corner_world_map_frame；绘制/result_image_raw灰线时把当前像素的相机点通过当前Scepter_depth_frame->map TF批量变到map，再用保存的map四边形判定。旧manual_workspace_quad.json若缺corner_world_map_frame无法恢复确认当时的全局位置，不再画误导性相机框，需要重新确认工作区。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/world_coord.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `红灯: 新增test_manual_workspace_quad_callback_persists_map_frame_corners缺corner_world_map_frame失败，test_confirmed_workspace_display_mask_uses_map_frame_not_camera_frame相机框命中失败；绿灯: 9个pointAI工作区/IR灰线相关unittest通过；py_compile world_coord/workspace_masks/processor/ros_interfaces/image_buffers通过`

### 后续注意

- 暂无。

## 2026-05-07 05:44 - 执行微调无点返回可跳过语义

<!-- AGENT-MEMORY: entry -->

### 摘要

- pointAI MODE_EXECUTION_REFINE 若连续短窗口内没有可执行点，不再无限等待；默认3秒后返回 success=false 且 message 前缀为 EXECUTION_REFINE_NO_POINTS，live_visual 现有失败分支会跳过当前区域继续后续区域。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`

### 后续注意

- 暂无。

## 2026-05-07 05:23 - 前端悬停坐标改为按需订阅 raw_world_coord

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场拖帧排查显示 /Scepter/worldCoord/raw_world_coord 单帧约 3.69MB、rosbridge 当前仍订阅该大图；不要为了图像悬停坐标常驻拉取 raw_world_coord。前端已改为只有鼠标在图像上悬停移动时临时订阅，1.8s 空闲后退订，并把 rosbridge throttle 从 180ms 放到 1000ms；IR 原图和 pointAI 结果图继续保留。

### 影响范围

- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/test/imageHoverCoordinateReadout.test.mjs`
- `src/tie_robot_web/frontend/test/imageHoverCoordinateModeControls.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-DF9c5dCN.js`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/imageHoverCoordinateReadout.test.mjs; node src/tie_robot_web/frontend/test/imageHoverCoordinateModeControls.test.mjs; for test_file in src/tie_robot_web/frontend/test/*.mjs; do node "$test_file" || exit 1; done; npm run build; rostopic bw /Scepter/worldCoord/raw_world_coord /pointAI/result_image_raw /Scepter/ir/image_raw; rosnode info /rosbridge_websocket; ps CPU snapshot`

### 后续注意

- 暂无。

## 2026-05-07 05:21 - 动态绑扎规划强制工作盒中心对齐分组中心

<!-- AGENT-MEMORY: entry -->

### 摘要

- 动态绑扎路径规划不再使用旧 template_center_* 作为索驱位姿反算目标；每个候选分组的世界坐标中心必须落在线性模组工作盒中心 tcp_max_x/2、tcp_max_y/2、tcp_max_z/2。若中心重合所需的索驱高度低于 bind_execution_cabin_min_z_mm，则该组判为不可规划，避免高度夹紧后破坏中心重合。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_geometry.cpp`
- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `test_dynamic_bind_planning 24/24 passed; test_scan_artifact_write_guard.py 13 passed; test_motion_chain_signal_guard.py 30 passed; test_tcp_travel_range_config.py 2 passed; catkin_make tie_robot_hw/tie_robot_msgs/tie_robot_process passed`

### 后续注意

- 暂无。

## 2026-05-07 05:21 - 账本+微调接收门限放宽到120/100

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户现场要求继续放宽账本+微调 live visual 接收门限；当前 kLiveVisualMicroAdjustXYToleranceMm=120mm、kLiveVisualMicroAdjustZToleranceMm=100mm。此门限只控制视觉点能否覆盖扫描账本参考点，棋盘格归类门限 80mm 不变。

### 影响范围

- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; source /opt/ros/noetic/setup.bash && LIBRARY_PATH=/home/hyq-/simple_lashingrobot_ws/devel/lib: catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_msgs;tie_robot_process;tie_robot_web' bind_task_executor_node cabin_motion_controller_node suoquNode`

### 后续注意

- 暂无。

## 2026-05-07 05:14 - 账本+微调接收门限放宽到100/50

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户现场要求继续放宽账本+微调 live visual 接收门限；当前 kLiveVisualMicroAdjustXYToleranceMm=100mm、kLiveVisualMicroAdjustZToleranceMm=50mm。此门限只控制视觉点能否覆盖扫描账本参考点，棋盘格归类门限 80mm 不变。

### 影响范围

- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; source /opt/ros/noetic/setup.bash && catkin_make bind_task_executor_node cabin_motion_controller_node suoquNode`

### 后续注意

- 暂无。

## 2026-05-07 05:14 - IR灰线改为已确认手动工作区边界

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确纠正：后端 IR 灰线不是当前可见深度边界，也不是 TCP/线性模组工作盒边界；应显示最开始确认的手动工作区边界，语义等同旧前端 canvas 绿框，只是绘制位置改到后端 IR 结果图。image_callback 现在读取 get_manual_workspace_cabin_polygon_pixel_mask() 并画灰线；该 mask 优先用 manual_workspace_quad 的 corner_world_camera_frame 与当前 raw/world_coord XY 通道生成，缺世界通道时才回退 corner_pixels。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/image_buffers.py; src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py; src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 当前工作区灰线=已确认手动工作区世界边界，不是 TCP 工作范围，不是整幅有效深度外轮廓。

### 标签

- `vision`
- `pointai`
- `manual-workspace-boundary`

### 验证证据

- `7 targeted pointAI tests OK with ROS/devel env; py_compile OK; grep confirms image_buffers no TCP workspace helper call`

### 后续注意

- 暂无。

## 2026-05-07 05:08 - 账本+微调接收门限放宽到60/10

<!-- AGENT-MEMORY: entry -->

### 摘要

- 视觉调试的账本+微调模式会进入 run_live_visual_global_work 并请求 MODE_EXECUTION_REFINE=4；若现场看起来更像纯账本，优先检查 live_visual 日志中的“忽略本次视觉修正，保留扫描参考点”。本次将 live visual 微调接收门限从 XY 30mm / Z 6mm 放宽为 XY 60mm / Z 10mm，棋盘格归类门限 80mm 不变。

### 影响范围

- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; source /opt/ros/noetic/setup.bash && catkin_make bind_task_executor_node cabin_motion_controller_node suoquNode; source /opt/ros/noetic/setup.bash && catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning`

### 后续注意

- 暂无。

## 2026-05-07 05:04 - 当前工作区灰线改按TCP三维工作盒筛选

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户现场截图显示旧后端灰线把整幅 raw/world_coord 有效画幅框住；根因是直接用有效深度外轮廓当工作区边界。现在 /pointAI/result_image_raw 的灰线先把 raw_world_coord/world_coord 像素转换到 TCP/gripper 工作坐标，并按 execution_refine_tcp_roi 三维盒筛出工作区域 mask，再只绘制该 mask 外轮廓；整幅图有效深度不再导致贴边画框。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/live_visible_area_overlay.py; src/tie_robot_perception/src/tie_robot_perception/pointai/image_buffers.py; src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 当前工作区边界语义为落在 TCP/工作范围三维盒内的像素边界，不是整幅有效深度边界。

### 标签

- `vision`
- `pointai`
- `current-workspace-boundary`

### 验证证据

- `targeted pointAI unittest OK; py_compile OK`

### 后续注意

- 暂无。

## 2026-05-07 04:51 - 当前视野边界改为后端IR灰线

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确否定前端 SVG/Canvas/三维工作区范围方案；索驱降下后相机当前可见区域边界由 pointAI 后端基于 raw_world_coord/world_coord 有效 Z 区域计算外轮廓，并直接绘制到 /pointAI/result_image_raw 的 IR 结果图上，使用灰色边界线。前端不再订阅 workspace/quad_camera_points，不再创建 workspaceRangeGroup，也不再把实时工作区范围投影回图像；仅保留 TCP/线性模组范围投影覆盖层。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/live_visible_area_overlay.py; src/tie_robot_perception/src/tie_robot_perception/pointai/image_buffers.py; src/tie_robot_web/frontend/src/views/Scene3DView.js; src/tie_robot_web/frontend/src/controllers/RosConnectionController.js; src/tie_robot_web/frontend/src/config/topicRegistry.js; src/tie_robot_web/web/index.html`

### 关键决策

- 当前可见区域边界不再是扫描工作区语义，也不在前端工作区图层绘制；后端图像流直接给出灰色边界。

### 标签

- `vision`
- `frontend`
- `pointai`
- `current-visible-area`

### 验证证据

- `pointAI targeted unittest OK; frontend all mjs tests OK; workspace_picker_web targeted unittest OK with ROS env; npm run build OK`

### 后续注意

- 暂无。

## 2026-05-07 04:43 - 视觉调试每组绑扎点数可配置

<!-- AGENT-MEMORY: entry -->

### 摘要

- 视觉调试页新增每组点数，StartPseudoSlamScan 服务/action 透传 bind_group_point_count；动态绑扎规划按 requested_group_point_count 生成矩形候选，默认 4 保持 2x2，6 点可按可达性选择 2x3 或 3x2，9 点等在当前路径高度和线性模组可达盒内无法覆盖全组时返回无法规划提示。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_msgs/srv/StartPseudoSlamScan.srv`
- `src/tie_robot_msgs/action/StartPseudoSlamScanTask.action`
- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/*.test.mjs 全部通过；python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py；python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py；./devel/lib/tie_robot_process/test_dynamic_bind_planning；catkin_make -DCATKIN_WHITELIST_PACKAGES=tie_robot_hw\;tie_robot_msgs\;tie_robot_process\;tie_robot_web`

### 后续注意

- 暂无。

## 2026-05-07 04:36 - live_visual 微调匹配轴向按扫描行列推断

<!-- AGENT-MEMORY: entry -->

### 摘要

- 账本+微调执行链中，MODE_EXECUTION_REFINE 返回相机点后会先转 map 再归入扫描账本棋盘格。当前 Surface-DP 行列在现场数据中 row 稳定对应 world_x、col 稳定对应 world_y，不能再硬编码 row=world_y/col=world_x；live_visual 现在从 pseudo_slam_points 的规划行列 span 推断 row/col 对应世界轴，再用该轴向做 80mm 棋盘格归类和 30mm/6mm 微调门限。

### 影响范围

- `src/tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp`
- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; source /opt/ros/noetic/setup.bash && catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning; source /opt/ros/noetic/setup.bash && catkin_make bind_task_executor_node cabin_motion_controller_node suoquNode`

### 后续注意

- 暂无。

## 2026-05-07 04:17 - 当前视野边界贴边可视化

<!-- AGENT-MEMORY: entry -->

### 摘要

- 实时相机可视边界来自 /Scepter/worldCoord/raw_world_coord 的有效 3D 像素边缘，并在 IR 底层 SVG 绘制；raw 边界抽样需过滤 z<=0 的无效深度，投影时少量点失败不隐藏整条边界。若边界几何上贴着整幅图像边缘，SVG 仅在显示层向内收 8px 画出，避免绿线被图像边框吃掉；线性模组/TCP 青蓝范围仍由 overlay canvas 保留。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/irImageUtils.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/workspaceRealtimeRangeFrame.test.mjs`
- `src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs`

### 关键决策

- 见摘要。

### 验证证据

- `node test/workspaceCanvasViewOverlay.test.mjs && node test/irImageLayerOverlayControls.test.mjs && node test/workspaceRealtimeRangeFrame.test.mjs && node test/workspaceCanvasInteractionMode.test.mjs && node test/imageHoverCoordinateReadout.test.mjs && node test/tcpWorkspaceOverlay.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_image_panel_supports_project_image_topics_and_overlay_switching src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_ir_image_draws_live_tcp_workspace_boundary_overlay; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 04:00 - 连接徽标长按重启中动画

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端顶部“连接成功”徽标长按触发 restartRosStack 后，现在会像索驱、末端、视觉状态胶囊一样进入 pending：禁用按钮、显示“重启中”、保留连接成功主标签并启用旋转/滑入动画；连接状态刷新不会打断该 pending 反馈，完成后恢复“长按重启”。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/connectionBadgeAlarmBehavior.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-DMeuumwo.js`
- `src/tie_robot_web/web/assets/app/index-g9fs5hgq.css`

### 关键决策

- 见摘要。

### 验证证据

- `node --test test/*.test.mjs; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 03:56 - 当前视野边界改为IR底层SVG

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求当前可视区域边界不要通过 overlay canvas 重绘，避免高频 clearRect/stroke 消耗。前端图像层现在在 irCanvas 和 overlayCanvas 之间新增 irBaseBoundaryLayer SVG，实时视野边界只更新 SVG polygon/circle DOM 属性；overlayCanvas 继续只承载算法结果、线模范围、识别点和悬停读数。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `node test/workspaceCanvasViewOverlay.test.mjs && node test/irImageLayerOverlayControls.test.mjs && node test/workspaceRealtimeRangeFrame.test.mjs && node test/workspaceCanvasInteractionMode.test.mjs; node test/imageHoverCoordinateReadout.test.mjs && node test/tcpWorkspaceOverlay.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_image_panel_supports_project_image_topics_and_overlay_switching src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_ir_image_draws_live_tcp_workspace_boundary_overlay; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 03:53 - 长按暂停恢复回起点跨进程中止

<!-- AGENT-MEMORY: entry -->

### 摘要

- 修复长按暂停/恢复作业未可靠清空当前末端任务并回执行起点的问题：末端 driver 与 motion_controller 已拆成两个进程，长按 /web/moduan/hand_sovle_warn data=2 必须同时让 driver 先写 IS_STOP=1 停止当前线性模组队列，并让 motion_controller 收到同一长按信号后置 moduan_return_zero_ordered_requested，使 ExecuteBindPoints Action/FINISHALL 等待中止；随后 /moduan/return_zero_ordered 才能接管 Z 优先回零和索驱回执行起点。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_control/test/test_single_point_bind_chain.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_control/test/test_single_point_bind_chain.py; python3 src/tie_robot_control/test/test_moduan_error_reset.py; python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_control`

### 后续注意

- 暂无。

## 2026-05-07 03:46 - 当前视野边界不是扫描工作区

<!-- AGENT-MEMORY: entry -->

### 摘要

- 纠正图像绿色边界语义：用户要看的不是保存给扫描层识别位姿的手动工作区，而是索驱当前位置下相机当前帧实际可见物理区域边界。前端绿色边界应由 /Scepter/worldCoord/raw_world_coord 当前帧有效 3D 像素边界生成，经当前 TF 固定到 map 后再用 IR camera_info 投回图像；/perception/lashing/workspace/quad_camera_points 只代表保存的扫描工作区角点，不应用作当前视野边界。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/irImageUtils.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/workspaceRealtimeRangeFrame.test.mjs`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `node test/workspaceRealtimeRangeFrame.test.mjs; node test/workspaceCanvasViewOverlay.test.mjs; node test/irImageLayerOverlayControls.test.mjs; node test/imageHoverCoordinateReadout.test.mjs; node test/tcpWorkspaceOverlay.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_image_panel_supports_project_image_topics_and_overlay_switching src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_ir_image_draws_live_tcp_workspace_boundary_overlay; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 03:35 - 线模范围与工作区投影区分

<!-- AGENT-MEMORY: entry -->

### 摘要

- 修正 2026-05-07 03:14 的误写口径：图像层不要关闭线性模组/TCP 工作范围蓝色投影，用户要关闭的是旧的工作区蓝色像素框。动态图像工作区投影应使用 /perception/lashing/workspace/quad_camera_points 的相机角点，前端按 TF 固定到 map 后再投回 IR 图像，并用非蓝色样式与线模范围区分；若看不到投影，优先检查 quad_camera_points、IR camera_info 和 TF 三者是否同时到达。

### 影响范围

- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs`
- `src/tie_robot_web/frontend/test/workspaceRealtimeRangeFrame.test.mjs`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `node test/workspaceCanvasViewOverlay.test.mjs; node test/irImageLayerOverlayControls.test.mjs; node test/workspaceRealtimeRangeFrame.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_image_panel_supports_project_image_topics_and_overlay_switching src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_ir_image_draws_live_tcp_workspace_boundary_overlay; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 03:14 - 图像工作区投影口径收口

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端工作区显示口径收口：不要再把实时工作区作为 3D Scene 蓝色框显示，也不要在图像层恢复旧的蓝色已保存工作区框或 TCP 范围投影框。实时工作区应先按 TF 固定到 map 世界坐标，再用当前 IR 相机内参和 TF 动态投回图像；手工选区折线只在工作区选点模式显示。

### 影响范围

- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`

### 关键决策

- 见摘要。

### 验证证据

- `node test/workspaceRealtimeRangeFrame.test.mjs; node test/workspaceCanvasViewOverlay.test.mjs; node test/irImageLayerOverlayControls.test.mjs; node test/workspaceCanvasInteractionMode.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_image_panel_supports_project_image_topics_and_overlay_switching src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_ir_image_draws_live_tcp_workspace_boundary_overlay; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 03:08 - 连接成功长按重启充能动画

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端顶部“连接成功”连接徽标的长按重启 ROS 现在复用 0.5 秒充能填充和扫光动画；renderConnectionBadgeState 会保留 is-long-press-charging / is-long-press-complete，避免连接状态刷新打断长按反馈。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/connectionBadgeAlarmBehavior.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-BZ3CPdKG.css`
- `src/tie_robot_web/web/assets/app/index-CHpVrKeG.js`

### 关键决策

- 见摘要。

### 验证证据

- `node test/connectionBadgeAlarmBehavior.test.mjs; node test/statusChipPressBehavior.test.mjs; for test_file in test/*.test.mjs; do node "$test_file" || exit 1; done; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 03:08 - TCP线模移动遇软件错误不再死等

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场点击TCP线性模组移动后反复打印 waiting on current target pt，是因为 move_linear_module_to_target 在 error_detected 软件全局错误标志为真时进入 while(is_error) 死等，服务不返回；前端后续看到的暂停/恢复回起点日志来自另外的暂停/长按控制，不是TCP移动主动发起。现在 raw_single_move 在下发任何轴坐标前检测 error_detected，若存在软件错误立即返回失败消息，提示先复位报警或排查PLC错误，不再刷 waiting on current target pt。

### 影响范围

- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_control/test/test_single_point_bind_chain.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_control.test.test_single_point_bind_chain src.tie_robot_control.test.test_moduan_error_reset; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; node --test src/tie_robot_web/frontend/test/*.test.mjs; source /opt/ros/noetic/setup.bash && catkin_make --only-pkg-with-deps tie_robot_control`

### 后续注意

- 暂无。

## 2026-05-07 03:00 - 暂停/恢复长按回执行起点顺序收紧

<!-- AGENT-MEMORY: entry -->

### 摘要

- 暂停/恢复作业按钮长按不是回Home或索驱原点，而是回本轮执行起点：前端长按仍发命令25 (/web/moduan/hand_sovle_warn data=2)，process层收到data=2后先标记当前自动链终止回起点，再显式停止索驱当前运动；停止失败则不继续回起点。停止成功后调用/moduan/return_zero_ordered，control层在当前末端执行链让出后按线性模组Z轴先归零、再X/Y归零，最后process层移动索驱回记录或从pseudo_slam_bind_path恢复出的执行起点。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`
- `src/tie_robot_web/frontend/src/controllers/LegacyCommandController.js`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-CN7T3Hck.js`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; node --test src/tie_robot_web/frontend/test/*.test.mjs; npm run build; source /opt/ros/noetic/setup.bash && catkin_make --only-pkg-with-deps tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 02:55 - 暂停作业长按也直接回执行起点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端 pauseResume 控制现在短按仍按原语义：未暂停时短按‘暂停作业’发 data=1 暂停，已暂停时短按‘恢复作业’发 data=1 恢复当前流程；长按无论在‘暂停作业’还是‘恢复作业’显示态，都发命令25，即 /web/moduan/hand_sovle_warn data=2，进入末端Z优先回零后索驱回执行起点链路。

### 影响范围

- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`
- `src/tie_robot_web/frontend/src/controllers/LegacyCommandController.js`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-Cy-jzLzT.js`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; node --test src/tie_robot_web/frontend/test/*.test.mjs; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 02:41 - 长按恢复起点缺失时从绑扎路径兜底

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场长按恢复作业出现‘没有可用的执行起点记录’后没有后续，是因为 process 层 recover_paused_execution_to_start 在 execution_pause_return_pose 无效时提前返回，未调用 /moduan/return_zero_ordered；control 层收到 data=2 只设置 moduan_return_zero_ordered_requested 并等待执行链让出，不会自行回零。已新增从当前 pseudo_slam_bind_path.json 的 path_origin 或首个 area.cabin_pose 兜底恢复回起点目标，成功后继续执行末端 Z 优先回零和索驱回起点。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; source /opt/ros/noetic/setup.bash && catkin_make --only-pkg-with-deps tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 02:35 - 连接徽标长按重启与报警短按复位

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端顶部“连接成功”连接徽标语义收口：ROS 已连接且无报警时短按不再触发报警复位，只保留长按 0.5 秒重启 ROS；状态监控上报报警后，徽标切为报警文案，短按发布 resetAllAlarms 报警复位，长按仍重启 ROS。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/test/connectionBadgeAlarmBehavior.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-oqQalhgV.js`

### 关键决策

- 见摘要。

### 验证证据

- `node --test src/tie_robot_web/frontend/test/*.test.mjs; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 02:34 - live_visual长按恢复记录规划原点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 修复默认账本+微调/live_visual执行分支长按恢复不回起点的问题：run_live_visual_global_work 开始时清理暂停状态，并在回规划原点前把 path_origin_x/y 与 clamp 后的 Z 记录为 execution_pause_return_pose。暂停 stop 遇到索驱 0x0013/status_word=0x00000004 设备未运动时视为已停止，避免空停污染错误日志。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_hw/src/driver/cabin_driver.cpp;src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract; python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; catkin_make --pkg tie_robot_hw tie_robot_process`

### 后续注意

- 暂无。

## 2026-05-07 02:27 - TCP线模遥控改走驱动原子移动

<!-- AGENT-MEMORY: entry -->

### 摘要

- TCP线性模组遥控不再走 /moduan/single_move -> execute_bind_points -> FINISHALL 等待链。驱动节点新增 /moduan/driver/raw_single_move，直接调用 move_linear_module_to_target 写三轴目标并等待各轴到位；前端 SERVICES.moduan.singleMove 指向该 raw 服务。兼容入口 /moduan/single_move 仅转发 raw 服务或在 compat_all 中调用同一原子函数，不再包装 PointCoords。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_control/test/test_single_point_bind_chain.py; python3 src/tie_robot_web/test/test_tcp_linear_remote_raw_move.py; npm run build (src/tie_robot_web/frontend); source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_control`

### 后续注意

- 暂无。

## 2026-05-07 02:24 - 线性模组报警复位同步清软件全局错误

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场出现Z轴目标0且当前位置0仍报等待到位收到全局错误标志，根因是PLC报警复位后C++进程内error_detected/last_error_msg未清；新增clear_system_error()并在/web/moduan/hand_sovle_warn手动报警复位成功后调用，避免旧软件错误继续中断wait_linear_module_axis_arrival。

### 影响范围

- `src/tie_robot_control/include/tie_robot_control/moduan/error_handling.hpp;src/tie_robot_control/src/moduan/error_handling.cpp;src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp;src/tie_robot_control/test/test_moduan_error_reset.py`

### 关键决策

- 见摘要。

### 标签

- `moduan`
- `error-reset`
- `field-debug`

### 验证证据

- `python3 -m unittest src/tie_robot_control/test/test_moduan_error_reset.py; python3 -m unittest src/tie_robot_control/test/test_single_point_bind_chain.py; bash -lc 'source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_control'`

### 后续注意

- 暂无。

## 2026-05-07 02:09 - 全局执行层增加账本微调模式选择

<!-- AGENT-MEMORY: entry -->

### 摘要

- 视觉调试选项卡新增全局执行模式选择，默认账本+微调；开始执行层按模式派发，不再因为 pseudo_slam_bind_path.json 存在就短路到纯账本执行。

### 影响范围

- `src/tie_robot_msgs/srv/SetExecutionMode.srv`
- `src/tie_robot_msgs/action/StartGlobalWorkTask.action`
- `src/tie_robot_process/src/suoqu/service_orchestration.cpp`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_web/src/web_bridge/action_bridge.cpp`
- `src/tie_robot_web/frontend/src/config/visualRecognitionMode.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`

### 关键决策

- 全局执行模式 0=执行账本/run_bind_from_scan，1=账本+微调/run_live_visual_global_work，2=规划路径+纯微调/run_planned_path_refine_only_global_work。
- 规划路径+纯微调只沿 pseudo_slam_bind_path.json 的 area.cabin_pose 移动，到位后每个区域调用 /moduan/sg；该模式不读取、不写入 bind_execution_memory.json。
- SetExecutionMode.srv 与 StartGlobalWorkTask.action 保留 MODE_LIVE_VISUAL=1，同时新增 MODE_LEDGER_WITH_REFINE=1 作为更准确语义，默认值为账本+微调。

### 标签

- `global-execution`
- `visual-debug`
- `ledger-refine`

### 验证证据

- `python3 -m unittest src/tie_robot_process/test/test_scan_artifact_write_guard.py src/tie_robot_process/test/test_motion_chain_signal_guard.py`
- `for f in test/*.test.mjs; do printf 'RUN %s\n' "$f"; node "$f" || exit 1; done (src/tie_robot_web/frontend)`
- `npm run build (src/tie_robot_web/frontend)`
- `source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=""`

### 后续注意

- 全局执行误差排查时先确认视觉调试页当前 executionMode；默认应走 ledger_with_refine，而不是历史的纯账本短路路径。

## 2026-05-07 01:57 - 撤销前端动态拆包恢复同步3D

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户反馈动态拆包后三维刷新变慢，已撤销‘去做1’方案：TieRobotFrontApp 恢复静态导入 Scene3DView/TerminalController，Scene3DView 恢复静态 OrbitControls，main.js 恢复同步 app.init()；删除 frontendCodeSplitting.test.mjs 和动态拆包产物。当前构建回到单个主 chunk，Vite 大 chunk 警告会重新出现，这是按用户要求换回先前三维刷新路径。

### 影响范围

- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js; src/tie_robot_web/frontend/src/views/Scene3DView.js; src/tie_robot_web/frontend/src/main.js; src/tie_robot_web/web/index.html`

### 关键决策

- 不继续追求 dynamic import 拆包；优先恢复同步 3D/终端加载以避免三维刷新变慢。

### 标签

- `frontend`
- `vite`
- `rollback`
- `3d`

### 验证证据

- `cd src/tie_robot_web/frontend && for test_file in test/*.test.mjs; do node "$test_file" || exit 1; done; npm run build（通过，预期出现大 chunk 警告）`

### 后续注意

- 暂无。

## 2026-05-07 01:42 - 前端重模块动态拆包（已撤销）

<!-- AGENT-MEMORY: entry -->

### 摘要

- 此条为历史方案，已在 `2026-05-07 01:57 - 撤销前端动态拆包恢复同步3D` 中撤销；不要继承动态加载 3D/终端的实现。原方案曾将 Scene3DView、TerminalController、three/OrbitControls 改为 dynamic import 以消除 Vite 大 chunk 警告，但用户反馈三维刷新变慢后已回退。

### 影响范围

- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js; src/tie_robot_web/frontend/src/views/Scene3DView.js; src/tie_robot_web/frontend/src/main.js; src/tie_robot_web/frontend/test/frontendCodeSplitting.test.mjs; src/tie_robot_web/web/index.html`

### 关键决策

- 已撤销，不再作为当前工程决策。

### 标签

- `frontend`
- `vite`
- `code-splitting`

### 验证证据

- `cd src/tie_robot_web/frontend && for test_file in test/*.test.mjs; do node "$test_file" || exit 1; done; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 01:26 - 相机视角隐藏机器人本体且俯视可拖动

<!-- AGENT-MEMORY: entry -->

### 摘要

- 显示与视角中切到相机视角时，3D Scene 会隐藏 base_link 对应的机器人本体 robotGroup，避免相机视角被自身模型遮挡；切回自由视角或俯视视角时按机器图层恢复。本页俯视视角保持严格沿世界 z- 方向看，但 OrbitControls 保持启用并允许平移拖动，关闭旋转以避免视角倾斜。

### 影响范围

- `src/tie_robot_web/frontend/src/views/Scene3DView.js; src/tie_robot_web/frontend/test/sceneViewModeBehavior.test.mjs`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `scene3d`
- `view-mode`

### 验证证据

- `node test/sceneViewModeBehavior.test.mjs && node test/gripperTfCalibration.test.mjs && node test/topicLayerStatePersistence.test.mjs && node test/visualDebugSettings.test.mjs; npm run build; git diff --check`

### 后续注意

- 暂无。

## 2026-05-07 01:18 - 工作区范围固定为map实时投影

<!-- AGENT-MEMORY: entry -->

### 摘要

- 确认工作区后，前端不再把 /perception/lashing/workspace/quad_pixels 保存像素四边形画成黄色静态框；/perception/lashing/workspace/quad_camera_points 到达时会按当时 TF 固定成 map 世界坐标，并在图像 overlay 中按当前相机 TF/camera_info 实时投影，所以相机移动到工作区边界时边界会在画面中移动显现。

### 影响范围

- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/workspaceRealtimeRangeFrame.test.mjs`
- `src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node test/workspaceCanvasViewOverlay.test.mjs; node test/workspaceRealtimeRangeFrame.test.mjs; for test_file in test/*.mjs; do node "" || exit 1; done; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 01:18 - 单点绑扎区域内蛇形点序

<!-- AGENT-MEMORY: entry -->

### 摘要

- /moduan/sg 单点绑扎执行微调返回多个点时，控制侧在 Scepter_depth_frame->gripper_frame TF 转换后按 TCP 局部 x 分行、y 交替方向蛇形排序再下发 execute_bind_points；pointAI MODE_EXECUTION_REFINE 服务响应和执行底图编号也同步改为同一 TCP 蛇形点序。扫描建图点序不随本次修改改变。

### 影响范围

- `CHANGELOG.md;src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp;src/tie_robot_control/test/test_single_point_bind_chain.py;src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py;src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 标签

- `single-bind`
- `execution-order`
- `snake-sort`

### 验证证据

- `python3 src/tie_robot_control/test/test_single_point_bind_chain.py; source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py PointAIScanOnlyPrFrpgTest.test_execution_refine_outputs_are_ordered_as_tcp_snake_rows_from_upper_right_origin; source /opt/ros/noetic/setup.bash && catkin_make --only-pkg-with-deps tie_robot_control`

### 后续注意

- 暂无。

## 2026-05-07 01:15 - 视觉触发统一按释放帧数放行

<!-- AGENT-MEMORY: entry -->

### 摘要

- pointAI 所有视觉检测触发入口统一按视觉调试页 stable_frame_count/释放帧数放行：/pointAI/process_image 的 scan-only 和 execution-refine 不再首帧返回，而是累计到释放帧数；/web/pointAI/run_workspace_s2 与 /perception/lashing/recognize_once 也改走同一 release-frame 入口。前端 runSavedS2、executionVisionOnly、triggerSingleBind、scanPlan、startExecution、startExecutionKeepMemory 触发前会同步当前释放帧数。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py; src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py; src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py; src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js; src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs`

### 关键决策

- 见摘要。

### 标签

- `vision`
- `frontend`
- `pointai`

### 验证证据

- `PYTHONPATH=/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src:/home/hyq-/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; node test/taskActionController.test.mjs && node test/taskActionSurfaceDpRecognition.test.mjs && node test/visualDebugSettings.test.mjs; npm run build; py_compile; git diff --check`

### 后续注意

- 暂无。

## 2026-05-07 01:00 - 执行链只保留视觉范围校验

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场口径固定：绑扎点从视觉识别到下发只走 pointAI 视觉/TCP ROI 一层范围校验；控制层 execute_bind_points、手动 /moduan/move 和 pseudo_slam_bind_path 预生成组加载不再用旧 X[0,360]/Y[0,320]/Z[0,160] 或类似硬范围二次拒绝点位。pointAI 与前端 TCP 线模遥控默认显示统一到 X[0,380]/Y[0,330]/Z[0,160]mm。

### 影响范围

- `CHANGELOG.md;src/tie_robot_control/src/moduan/linear_module_executor.cpp;src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_perception/src/tie_robot_perception/pointai/state.py;src/tie_robot_web/frontend/src/controllers/TcpLinearRemoteController.js`

### 关键决策

- 见摘要。

### 标签

- `视觉校验`
- `TCP ROI`
- `执行链`

### 验证证据

- `python3 -m unittest src/tie_robot_control/test/test_single_point_bind_chain.py -k trusts_visual; python3 -m unittest src/tie_robot_process/test/test_tcp_travel_range_config.py; python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_execution_refine_hough_uses_tcp_coordinate_box_as_roi; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_tcp_linear_module_remote_page_and_ros_flow_exist; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 00:56 - 红外叠加不再强制切换图像话题

<!-- AGENT-MEMORY: entry -->

### 摘要

- 修正前端红外叠加口径：/pointAI/result_image_raw 和视觉识别结果只缓存/叠加到红外原图，结果话题到达时不再调用 setSelectedImageTopic(TOPICS.camera.irImage)，避免用户切换到扫描底图、执行底图、彩色/深度等图像层后被后台结果帧自动拉回红外原图。

### 影响范围

- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/irImageLayerOverlayControls.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node test/irImageLayerOverlayControls.test.mjs; for test_file in test/*.test.mjs; do node "" || exit 1; done; npm run build (src/tie_robot_web/frontend)`

### 后续注意

- 暂无。

## 2026-05-07 00:51 - 前端执行结果统一叠加到红外原图

<!-- AGENT-MEMORY: entry -->

### 摘要

- 图像卡片不再把 /pointAI/result_image_raw 暴露为独立执行结果图层；执行/视觉结果图继续后台订阅并统一叠加到红外原图。显示与视角/图层设置新增红外识别结果、红外扫描点控制，线性模组范围开关也同步控制 IR 上的 TCP 范围投影。

### 影响范围

- `src/tie_robot_web/frontend/src/config/imageTopicCatalog.js`
- `src/tie_robot_web/frontend/src/config/topicLayerCatalog.js`
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`

### 关键决策

- 见摘要。

### 验证证据

- `for test_file in src/tie_robot_web/frontend/test/*.test.mjs; do node "" || exit 1; done; npm run build (src/tie_robot_web/frontend)`

### 后续注意

- 暂无。

## 2026-05-07 00:50 - 前端连接胶囊聚合报警与复位

<!-- AGENT-MEMORY: entry -->

### 摘要

- 右上角主连接胶囊现在聚合索驱/末端报警：ROS 连接成功且存在报警时变为黄色，并直接显示首个报警摘要（如“X轴异常等2项报警”），title 展示完整报警列表；短按发布 /web/moduan/hand_sovle_warn=1 复位报警并清空前端报警状态，长按 0.5 秒改为重启 ROS。末端报警来源同时兼容 /moduan/moduan_gesture_data 错误标志和 /diagnostics 中 tie_robot/moduan_driver，索驱报警读取 /diagnostics 的 device_alarm/internal_calc_error。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/controllers/StatusMonitorController.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/connectionBadgeAlarmBehavior.test.mjs`
- `src/tie_robot_web/frontend/test/statusMonitorController.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-DKL8vGqn.js`
- `src/tie_robot_web/web/assets/app/index-Q578f7rt.css`

### 关键决策

- 见摘要。

### 验证证据

- `node test/connectionBadgeAlarmBehavior.test.mjs; node test/statusMonitorController.test.mjs; node test/statusChipPressBehavior.test.mjs; node test/rosConnectionController.test.mjs; node test/systemControlCatalog.test.mjs; for f in test/*.test.mjs; do node "$f" || exit 1; done; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 00:38 - 图像卡片选点与悬停坐标分离

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端图像卡片现在只有在设置页切到“工作区选点”时才响应点击/拖拽选点；其他设置页下点击不再新增工作区角点，而是用鼠标悬停像素采样 /Scepter/worldCoord/raw_world_coord，并按“显示与视角”里的图像悬停坐标设置显示 map 世界坐标、gripper_frame 工具坐标或 Scepter_depth_frame 相机坐标。悬停显示只做前端 TF 换算，不改变 pointAI 原始相机点语义。

### 影响范围

- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/utils/irImageUtils.js`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `tf`
- `workspace`
- `coordinate-display`

### 验证证据

- `node src/tie_robot_web/frontend/test/*.mjs 全部通过；git diff --check；npm run build`

### 后续注意

- 暂无。

## 2026-05-07 00:29 - 实时工作区范围框

<!-- AGENT-MEMORY: entry -->

### 摘要

- 确认工作区后，pointAI 继续发布 /perception/lashing/workspace/quad_pixels 像素四边形，并新增 /perception/lashing/workspace/quad_camera_points（tie_robot_msgs/PointsArray，World_coord 为 Scepter_depth_frame 下的相机坐标 mm）。前端订阅该实时话题，在 Scene3DView 中通过 TF 转到 map 世界坐标绘制工作区范围框；该框属于实时图层，不走静态 bind-path/API 图层。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `perception`
- `tf`
- `workspace`

### 验证证据

- `node src/tie_robot_web/frontend/test/workspaceRealtimeRangeFrame.test.mjs; node src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs; frontend/test/*.mjs 全部通过; python3 -m py_compile pointAI 改动文件; npm run build`

### 后续注意

- 暂无。

## 2026-05-07 00:10 - 三维绑扎范围增加独立图层开关

<!-- AGENT-MEMORY: entry -->

### 摘要

- 显示与视角/图层设置新增showLinearModuleBindRange状态和“绑扎范围”开关，用于单独控制gripper_frame下线性模组绑扎范围长方体显示。该状态随topic layer preference持久化；范围实体不再被showRobot硬联动隐藏，只要gripper_frame TF可用且开关开启即可单独查看。

### 影响范围

- `src/tie_robot_web/frontend/src/config/topicLayerCatalog.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/test/bindPathLayerControls.test.mjs`
- `src/tie_robot_web/frontend/test/topicLayerStatePersistence.test.mjs`
- `src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs`
- `CHANGELOG.md`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `for test_file in src/tie_robot_web/frontend/test/*.test.mjs; do node ""; done；npm run build (src/tie_robot_web/frontend)`

### 后续注意

- 暂无。

## 2026-05-07 00:02 - 动态绑扎路径改按世界X蛇形

<!-- AGENT-MEMORY: entry -->

### 摘要

- pseudo_slam预生成绑扎路径的2x2候选组排序已改为直接依据绑扎点World_coord：先按世界Y聚成蛇形行带，再在每带沿世界X+ / X-交替遍历。视觉S2的global_row/global_col只用于棋盘相邻关系，不再作为世界Y/X轴排序依据，避免当前相机安装和图像轴语义导致长边走成世界Y。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `devel/lib/tie_robot_process/test_dynamic_bind_planning；source /opt/ros/noetic/setup.bash && catkin_make`

### 后续注意

- 暂无。

## 2026-05-06 23:52 - TCP执行Z行程上限统一为160mm

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场日志显示预生成点 z=141~147mm 被 moduan_driver_node 按 Z[0,140] 拒绝。根因是执行层/动态规划/pointAI travel range 仍保留旧 140mm 上限，已统一到 Z[0,160]mm；视觉 ROI、3D/IR 显示、执行下发和规划过滤的 Z 上限不应再漂移。

### 影响范围

- `src/tie_robot_control/include/tie_robot_control/moduan/runtime_state.hpp`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_control/test/test_single_point_bind_chain.py src/tie_robot_process/test/test_tcp_travel_range_config.py; python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_visual_debug_runtime_controls_are_exposed_to_frontend src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_execution_refine_hough_uses_tcp_coordinate_box_as_roi; source /opt/ros/noetic/setup.bash && catkin_make`

### 后续注意

- 暂无。

## 2026-05-06 23:34 - 线性模组绑扎范围可调并驱动3D/IR/pointAI ROI

<!-- AGENT-MEMORY: entry -->

### 摘要

- 视觉调试设置新增线性模组绑扎范围 linearModuleBindRangeMm，默认 x[0,380]/y[0,330]/z[0,160]mm；该范围持久化到 tie_robot_frontend_visual_debug_settings，并由 Scene3DView 用 gripper_frame 下的透明长方体显示，IR TCP 工作范围投影复用同一范围，同时前端通过 `/web/pointAI/set_execution_refine_tcp_roi` 下发给 pointAI 更新执行微调 TCP ROI 和 ROS 参数。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/tcpWorkspaceOverlay.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && for test_file in test/*.mjs; do node "$test_file" || exit 1; done`
- `python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_visual_debug_runtime_controls_are_exposed_to_frontend src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_execution_refine_hough_uses_tcp_coordinate_box_as_roi`
- `cd src/tie_robot_web/frontend && npm run build`

### 后续注意

- 暂无。

## 2026-05-06 23:16 - 相机机械安装姿态更新为base yaw负90

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场确认 Scepter_depth_frame 在 base_link 下的机械安装姿态应为：相机 -X 与 base_link +Y 重合、相机 +Y 与 base_link -X 重合、相机 +Z 朝地面（base_link -Z）。已将 robot_home_tf.yaml 和 robot_tf_broadcaster 默认 base_to_camera_rpy 改为 roll=pi,pitch=0,yaw=-pi/2。

### 影响范围

- `src/tie_robot_perception/config/robot_home_tf.yaml`
- `src/tie_robot_perception/scripts/robot_tf_broadcaster.py`
- `src/tie_robot_perception/test/test_robot_tf_broadcaster.py`

### 关键决策

- base_link->Scepter_depth_frame 机械相机外参使用 yaw=-pi/2，不再使用 yaw=0。

### 标签

- `tf`
- `camera`
- `robot-home`
- `scepter`

### 验证证据

- `2026-05-06: python3 -m unittest src/tie_robot_perception/test/test_robot_tf_broadcaster.py -> Ran 7 tests OK; rosnode kill /robot_tf_broadcaster 后 respawn；tf_echo base_link Scepter_depth_frame 显示 RPY degree [180`
- `0`
- `-90]。`

### 后续注意

- 暂无。

## 2026-05-06 22:41 - 执行图tcp标签显示绝对gripper坐标

<!-- AGENT-MEMORY: entry -->

### 摘要

- 执行微调结果图的 tcp=(x,y,z) 显示必须是相机坐标经 Scepter_depth_frame -> gripper_frame 后的绝对虎口/TCP坐标；不得再减 current_linear_module_position_mm 或生成当前虎口相对坐标。camera_coord_to_tcp_jaw_coord / camera_channels_to_tcp_jaw_channels 只暴露相机到 gripper_frame 的 TF 转换，不接受 current_tcp_mm。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/tcp_display.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- TCP/工具坐标系只有 gripper_frame 一个语义；结果图标签、ROI 和输出排序使用同一 camera->gripper TF 结果。

### 标签

- `tf`
- `pointai`
- `tcp-display`
- `execution-refine`

### 验证证据

- `2026-05-06: python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> Ran 103 tests OK; rg current_tcp_mm src/tie_robot_perception/src/tie_robot_perception -> no matches.`

### 后续注意

- 暂无。

## 2026-05-06 22:19 - TCP坐标系收口到gripper_frame

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：排查执行层识别结果坐标后确认，TCP工具坐标系就是gripper_frame，感知侧、前端红外投影和执行层都不得再做tcp.x=380-gripper_y、tcp.y=gripper_x这类额外显示换轴。Scepter_depth_frame->gripper_frame TF已改为右上角原点口径：x+对应图像从上到下，y+对应图像从右到左；pointAI tcp_display只做与TF等价的相机点到gripper_frame变换，前端TCP范围直接投gripper_frame边界点。

### 影响范围

- `src/tie_robot_perception/config/gripper_tf.yaml`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/tcp_display.py`
- `src/tie_robot_web/frontend/src/utils/tcpWorkspaceOverlay.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`

### 关键决策

- 见摘要。

### 验证证据

- `pointAI 103 tests OK; gripper_tf_broadcaster 14 tests OK; frontend *.test.mjs OK; npm run build OK`

### 后续注意

- 暂无。

## 2026-05-06 22:06 - 执行层tcp标签改用右上角工具坐标

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：进一步修正执行层识别结果图 tcp=(...) 标签。此前只修了点序排序，但标签仍直接显示 gripper 投影轴值，导致横向变化落在 tcp.x、纵向变化落在 tcp.y，现场出现点1在点3上方但 x 不小于点3。现在 pointAI 的 tcp_display 与前端 TCP 范围投影保持同一工具坐标口径：tcp.x=380-gripper_y、tcp.y=gripper_x、tcp.z=gripper_z，再减去当前线性模组位置；执行层 Hough 输出排序也按该工具 tcp 坐标小到大。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/tcp_display.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash; PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; python3 src/tie_robot_control/test/test_single_point_bind_chain.py; py_compile tcp_display/matrix/rendering/workspace_masks`

### 后续注意

- 暂无。

## 2026-05-06 21:49 - 绑扎点识别按右上角TCP原点排序

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：修正 pointAI 识别结果排序口径。扫描 Surface-DP 生成 PointsArray/结果图编号时，global_row 按画面上到下，global_col 按画面右到左（右上角为TCP工具原点）；执行层 Hough 输出也按像素轴 x+=向下、y+=向左排序。同步把执行层 TCP ROI 默认范围修正为 x[0,380]/y[0,330]/z[0,160]mm，避免旧错误默认值扩大范围门。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash; PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; python3 src/tie_robot_control/test/test_single_point_bind_chain.py; py_compile pointai modules`

### 后续注意

- 暂无。

## 2026-05-06 21:40 - 三维TCP坐标轴跟随运动TCP

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：前端三维橙色 TCP 工具方块叠加了线性模组当前位置后显示实际运动 TCP 位姿；gripper_frame/TCP 坐标轴的显示位置也应跟随这个实际运动 TCP 方块移动。注意这只是三维显示层的坐标轴位置，底层 TF 计算、投影和静态 Scepter_depth_frame->gripper_frame 外参仍保持原始 transformMap 语义。

### 影响范围

- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs; frontend *.mjs tests; npm run build`

### 后续注意

- 暂无。

## 2026-05-06 21:20 - 控制面板执行层视觉单侧

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：控制面板新增“执行层视觉单侧”按钮，语义是只调用 /pointAI/process_image request_mode=4（平面去除 + Hough 执行层视觉），不调用 /moduan/sg 或线性模组单点绑扎链。TCP工具坐标范围投影只作为红外图像覆盖层显示，不放入设置页视觉调试配置项。

### 影响范围

- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/test/taskActionController.test.mjs`
- `src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs`

### 关键决策

- 见摘要。

### 验证证据

- `frontend *.mjs tests; npm run build`

### 后续注意

- 暂无。

## 2026-05-06 21:04 - TCP红外范围投影增加z160顶面

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：红外原图 TCP 工作范围覆盖层不再绘制 TCP z=0 x0-380 y0-330 这类说明文字；投影数据默认包含 TCP 工具坐标系 x[0,380]、y[0,330] 的 z=0 底面和 z=160 顶面，前端画两层矩形并用虚线连接对应角点。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/tcpWorkspaceOverlay.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/test/tcpWorkspaceOverlay.test.mjs`
- `src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/tcpWorkspaceOverlay.test.mjs; node src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs; node src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs; frontend *.mjs tests; npm run build`

### 后续注意

- 暂无。

## 2026-05-06 20:57 - 坐标显示统一到1mm精度

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：用户明确所有坐标系与显示点只需要 1mm 显示精度。后续前端 UI、IR 叠字、TF/Home/TCP/线模等坐标文本应四舍五入为整数 mm；底层 TF、规划、JSON、控制命令仍保留原始浮点用于计算和运动。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/coordinateDisplayPrecision.test.mjs`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/coordinateDisplayPrecision.test.mjs; node src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs; python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; npm run build`

### 后续注意

- 暂无。

## 2026-05-06 20:44 - TCP红外覆盖层只做工具到gripper轴映射

<!-- AGENT-MEMORY: entry -->

### 摘要

- 更正上一条关于 gripper_tf yaw 的判断：TF 到前端三维口径保持正确，Scepter_depth_frame->gripper_frame 继续使用 yaw=pi；不要为了红外原图 TCP 覆盖层改 gripper_tf.yaml 的旋转。红外 overlay 的根因是把 TCP 工具坐标 x/y 直接当成 gripper_frame x/y 投影。现场像素级轴向为工具 x+ 从上到下、工具 y+ 从右到左，因此 overlay 投影前应做工具坐标到 gripper 投影点映射：tool_x 沿 -gripper_y，tool_y 沿 +gripper_x；前端实现为 buildTcpWorkspaceBoundaryGripperPointsMm，Scene3DView.projectTcpWorkspaceBoundaryToImage 使用该映射后再走 TF/CameraInfo 投影。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/tcpWorkspaceOverlay.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/test/tcpWorkspaceOverlay.test.mjs`
- `src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs`
- `src/tie_robot_perception/config/gripper_tf.yaml`

### 关键决策

- 见摘要。

### 验证证据

- `frontend .mjs tests; gripper_tf_broadcaster + single_point_bind_chain unittest; pointai_scan_only_pr_fprg unittest; npm frontend build; tf_echo confirms yaw=180deg; git diff --check`

### 后续注意

- 暂无。

## 2026-05-06 20:37 - TCP gripper轴向修正为图像x下y左

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：现场确认 IR 画面像素轴向为 gripper x+ 从上到下、gripper y+ 从右到左，z+ 朝地面；Scepter_depth_frame->gripper_frame 不应继续用 yaw=pi。已将 src/tie_robot_perception/config/gripper_tf.yaml 的 rotation_rpy.yaw 从 pi 改为 pi/2，并重启 tie-robot-rosbridge.service 使运行时 TF 生效。tf_echo Scepter_depth_frame gripper_frame 显示 quaternion z/w=0.707/0.707、yaw=90deg。前端 TCP 工作区投影继续直接按 TF 投 gripper 坐标，不做 330-y 之类显示层翻转。

### 影响范围

- `src/tie_robot_perception/config/gripper_tf.yaml`
- `src/tie_robot_perception/test/test_gripper_tf_broadcaster.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`

### 关键决策

- 见摘要。

### 验证证据

- `frontend .mjs tests; gripper_tf_broadcaster + single_point_bind_chain unittest; pointai_scan_only_pr_fprg unittest with ROS PYTHONPATH appended; npm frontend build; tf_echo yaw=90deg; git diff --check`

### 后续注意

- 暂无。

## 2026-05-06 19:35 - 暂停恢复短按继续当前流程长按回起点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：暂停作业的最终语义修正为：短按恢复作业恢复当前工作流程，长按恢复作业才执行线性模组Z优先回零后索驱回起点。process层将 execution_pause_requested 与 execution_return_to_start_requested 拆开：暂停时停止索驱并挂起等待；短按 data=1 清暂停，wait_cabin_axis_stable_arrival 会重发 TCP_Move 当前目标并继续到位门控；长按 data=2 设置 return_to_start，当前自动链退出后 recover_paused_execution_to_start 调用 /moduan/return_zero_ordered 再让索驱回起点。control层新增 moduan_return_zero_ordered_requested：wait_for_plc_finish_all 对短暂停留等待恢复，对长按回起点请求返回失败释放执行锁，随后 /moduan/return_zero_ordered 可有序回零。前端 pauseResume 仍保留 longPressCommandId=25；短按走 deactivateCommandId=13(data=1)，长按走 command 25(data=2)。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_control/include/tie_robot_control/moduan/runtime_state.hpp`
- `src/tie_robot_control/src/moduan/runtime_state.cpp`
- `src/tie_robot_web/frontend/src/controllers/LegacyCommandController.js`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`
- `src/tie_robot_control/test/test_single_point_bind_chain.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_control/test/test_single_point_bind_chain.py; for test_file in src/tie_robot_web/frontend/test/*.mjs; do node "$test_file"; done; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_msgs;tie_robot_process;tie_robot_control'; npm --prefix src/tie_robot_web/frontend run build; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; devel/lib/tie_robot_process/test_dynamic_bind_planning; git diff --check; systemctl is-active tie-robot-driver-suoqu.service tie-robot-driver-moduan.service tie-robot-backend.service tie-robot-frontend.service; rosservice type /moduan/return_zero_ordered`

### 后续注意

- 暂无。

## 2026-05-06 19:20 - 暂停作业改为执行层硬暂停与长按回起点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：前端暂停作业现在同时闩锁执行层 execution_pause_abort_requested 并立即下发索驱停止；即使末端 moduan_work 正忙，bind_from_scan 也不会在末端返回后继续移动下一组。前端恢复作业短按只提示需要长按，不再发送旧 data=1 恢复；长按发送 /web/moduan/hand_sovle_warn data=2。process 层收到 data=2 后先调用 /moduan/return_zero_ordered，让线性模组按 Z 轴先回 0、再 X/Y 回 0，然后索驱回到本轮执行起点，且旧自动任务保持终止。control 层新增 /moduan/return_zero_ordered Trigger 服务，wait_for_plc_finish_all 会在人工暂停时返回失败而不是等 FINISHALL 后误报成功。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_web/frontend/src/controllers/LegacyCommandController.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`
- `src/tie_robot_web/frontend/src/config/legacyCommandCatalog.js`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_control/test/test_single_point_bind_chain.py; for test_file in src/tie_robot_web/frontend/test/*.mjs; do node "$test_file"; done; source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_msgs;tie_robot_process;tie_robot_control'; npm --prefix src/tie_robot_web/frontend run build; systemctl is-active tie-robot-driver-suoqu.service tie-robot-driver-moduan.service tie-robot-backend.service tie-robot-frontend.service; rosservice type /moduan/return_zero_ordered`

### 后续注意

- 暂无。

## 2026-05-06 18:52 - bind_from_scan末端完成后索驱切换保护

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：现场日志显示区域1末端FINISHALL完成并清零后，约1ms内立刻向区域2下发/cabin/driver/raw_move，索驱上位机关闭TCP，导致0x0012 raw_move失败，后续0x0001心跳断开只是同一TCP会话的连带现象。bind_from_scan现在在每组/moduan/execute_bind_points成功后，等待/moduan_work连续空闲3个100ms采样再允许下一次索驱移动；索驱驱动层CabinDriver::moveToPose对可重试TCP发送/等待/接收失败会断开重连并仅对0x0012绝对位姿重发一次，moveByOffset仍不自动重发以避免相对运动重复执行。前端web静态产物已重建并清理旧hash chunk，当前构建不再包含电压状态变化日志或旧执行记忆goal结构。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; devel/lib/tie_robot_process/test_dynamic_bind_planning; python3 src/tie_robot_control/test/test_single_point_bind_chain.py; node src/tie_robot_web/frontend/test/*.mjs; npm --prefix src/tie_robot_web/frontend run build; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_msgs;tie_robot_process'; git diff --check; systemctl is-active tie-robot-driver-suoqu.service tie-robot-backend.service tie-robot-frontend.service`

### 后续注意

- 暂无。

## 2026-05-06 18:39 - 执行层首区直接移动与raw_move错误透传

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：bind_from_scan不再先移动到pseudo_slam_bind_path.json的path_origin.z作为执行前置高度；开始执行层会直接进入区域循环，区域1第一条索驱raw_move就是首个area.cabin_pose，例如现场(-823.75,433.17,503.25)，避免先到同XY的487.0再补Z。远程/cabin/driver/raw_move调用新增8秒服务返回超时保护，并在失败时把目标点、速度和驱动层返回message（含connection closed by peer、request_command、request_frame等detail）写入last_cabin_transport_error_detail，StartGlobalWorkAction会通过compose_cabin_failure_message把最近底层错误透传到前端。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; devel/lib/tie_robot_process/test_dynamic_bind_planning; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_msgs;tie_robot_process'; git diff --check`

### 后续注意

- 暂无。

## 2026-05-06 18:11 - 绑扎路径改为x+长边蛇形

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：用户要求索驱蛇形规划以x+为长路径正方向推进。动态绑扎固定2x2分组的traversal_order已从按两列带蛇形改为按两行带蛇形：每两行形成一个row band，偶数row band沿x+遍历各2x2块，奇数row band反向沿x-返回；这样首条长路径从世界最小角点出发沿x+展开。同步更新了4x4、4x6和3x3边缘pair的回归顺序断言，以及pseudo_slam扫描完成日志口径。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `make -C build test_dynamic_bind_planning -j8; devel/lib/tie_robot_process/test_dynamic_bind_planning; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_hw;tie_robot_msgs;tie_robot_process"; git diff --check; sudo -n systemctl restart tie-robot-backend.service`

### 后续注意

- 暂无。

## 2026-05-06 17:56 - 执行层等待改为到位门控

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：用户明确要求执行层索驱等待不要按固定总时长硬失败，而是严格按到位门控推进。当前bind_from_scan等执行链中的wait_cabin_axis_stable_arrival保留30秒软超时日志，但软超时后若索驱仍在运动、坐标仍变化，或已进入容差等待稳定样本，就持续等待到位；不再设置180秒硬超时。真正失败条件收口为索驱状态字异常，或索驱已停止但仍未到位。这样任务链只有在当前步骤真实完成后才进入下一步单点/分组绑扎。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; devel/lib/tie_robot_process/test_dynamic_bind_planning; catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_hw;tie_robot_msgs;tie_robot_process"; git diff --check; sudo -n systemctl restart tie-robot-backend.service; rostopic echo -n1 /cabin/cabin_data_upload`

### 后续注意

- 暂无。

## 2026-05-06 17:48 - 执行层等待索驱运动软超时

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：现场开始执行层在bind_from_scan回规划原点时，X轴30秒等待超时但motion_status=1，底层索驱随后继续到达原点，导致任务线程已失败退出、不会进入后续/moduan/execute_bind_points单点绑扎。修复为执行层stable轴等待区分30秒软超时和180秒硬超时：软超时时若索驱仍在运动，或已进入容差但还在凑稳定样本，继续等待并记录Cabin_Warn；只有停止仍不到位或硬超时时才失败。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; devel/lib/tie_robot_process/test_dynamic_bind_planning; catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_hw;tie_robot_msgs;tie_robot_process"; git diff --check; sudo -n systemctl restart tie-robot-backend.service`

### 后续注意

- 暂无。

## 2026-05-06 17:43 - 前端不再记录机器人电压状态变化日志

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：前端 StatusMonitorController 仍会更新 header 电压徽标，但不再把线性模组 telemetry 中的 robot_battery_voltage 变化写入前端日志；像“[前端] 状态变化 电压 -> 机器人电压 52.9V”这类提示不再显示。新增 statusMonitorController.test.mjs 锁定该行为。

### 影响范围

- `src/tie_robot_web/frontend/src/controllers/StatusMonitorController.js`
- `src/tie_robot_web/frontend/test/statusMonitorController.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && node test/statusMonitorController.test.mjs && for test_file in test/*.mjs; do node "$test_file" || exit 1; done && npm run build; sudo systemctl restart tie-robot-frontend.service && systemctl show tie-robot-frontend.service --property=ActiveState`
- `SubState --no-pager`

### 后续注意

- 暂无。

## 2026-05-06 17:28 - 2x2组中心对齐TCP工作区中心

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：动态绑扎索驱路径候选位姿调整为让每个2x2组在规划高度下的TCP局部点中心对齐虎口工作区中心。TCP执行范围从X[0,360]/Y[0,320]收口到用户现场口径X[0,380]/Y[0,330]，对应模板目标中心为(190,165,70)mm。为避免中心变更影响2x2几何评分，新增nominal_grid_spacing_mm=150mm，square/pair评分不再复用template_center_x_mm。旧pseudo_slam_bind_path.json不会自动变化，需重新扫描规划生成新cabin_pose。

### 影响范围

- `src/tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp`
- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `devel/lib/tie_robot_process/test_dynamic_bind_planning 17/17 passed; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py OK; catkin_make -DCATKIN_WHITELIST_PACKAGES=tie_robot_hw`
- `tie_robot_msgs`
- `tie_robot_process passed; git diff --check clean; tie-robot-backend.service active after restart`

### 后续注意

- 暂无。

## 2026-05-06 14:37 - 执行层默认关闭执行记忆

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：开始执行层默认不再清空、不读取、不校验、不写入bind_execution_memory.json。前端普通开始发送use_execution_memory=false、clear_execution_memory=false；“记忆续跑开始”才发送use_execution_memory=true。StartGlobalWork.srv和StartGlobalWorkTask.action新增use_execution_memory，web_action_bridge透传到/cabin/start_work_with_options，后端仅在clear_execution_memory && use_execution_memory时重置记忆；run_bind_from_scan/run_live_visual_global_work在记忆关闭时只校验pseudo_slam_points.json与pseudo_slam_bind_path.json彼此scan_session_id/path_signature一致，不再和当前路径配置或执行记忆校验。

### 影响范围

- `src/tie_robot_msgs/action/StartGlobalWorkTask.action`
- `src/tie_robot_msgs/srv/StartGlobalWork.srv`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/src/web_bridge/action_bridge.cpp`
- `src/tie_robot_process/src/suoqu/service_orchestration.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; npm --prefix src/tie_robot_web/frontend run build; catkin_make -DCATKIN_WHITELIST_PACKAGES=tie_robot_hw`
- `tie_robot_msgs`
- `tie_robot_process`
- `tie_robot_web; rosapi验证StartGlobalWorkTaskGoal含use_execution_memory且/cabin/start_work_with_options Args含use_execution_memory`

### 后续注意

- 暂无。

## 2026-05-06 14:19 - 执行层点击无反应与 rosbridge 消息缓存

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06 排查开始执行层无反应：PointCoords.msg 变更后只重启 backend/run.launch 不够，运行中的 rosbridge/rosapi 会继续持有旧 PointsArray md5，导致 /perception/lashing/points_camera Dropping connection，并可能让前端 Action/话题桥接表现异常。处理方式：sudo -n systemctl restart tie-robot-rosbridge.service 后 sudo -n systemctl start tie-robot-backend.service；验证 /rosapi/message_details tie_robot_msgs/PointsArray 包含 has_grid_index/global_row/global_col，且 /web/cabin/start_global_work/goal 上 rosbridge 发布、web_action_bridge_node 订阅。

### 影响范围

- `src/tie_robot_msgs/msg/PointCoords.msg src/tie_robot_web/src/web_bridge/action_bridge.cpp src/tie_robot_process/src/suoqu/service_orchestration.cpp`

### 关键决策

- 见摘要。

### 标签

- `rosbridge`
- `execution-layer`
- `md5`
- `runtime`

### 验证证据

- `systemctl is-active tie-robot-rosbridge.service tie-robot-backend.service tie-robot-frontend.service; rosapi message_details PointsArray includes has_grid_index/global_row/global_col; rostopic info /web/cabin/start_global_work/goal`

### 后续注意

- 暂无。

## 2026-05-06 14:02 - 绑扎执行路径改为列带蛇形

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：用户指出索驱规划路径应为蛇形而非Z字形。动态绑扎规划固定2x2分组的traversal_order改为按2列带蛇形排序：偶数列带从世界最小角起点端向末端走，奇数列带反向回来，避免每列都从同一端开始造成跨列长斜跳。新增TraversesProvidedGridAsSnakeColumnsWithoutZJump回归测试。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `catkin_make; devel/lib/tie_robot_process/test_dynamic_bind_planning; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; git diff --check`

### 后续注意

- 暂无。

## 2026-05-06 13:56 - 显示与视角图层开关持久化

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：设置页“显示与视角”的图层/视角状态开始持久化到 localStorage key tie_robot_frontend_topic_layer_state。TieRobotFrontApp 启动时通过 loadTopicLayerStatePreference 恢复 TopicLayerController 初始状态；用户修改显示组件开关或视角控制后调用 saveTopicLayerStatePreference 保存 controller.getState()。保存内容包含机器、坐标轴、点云、规划点/绑扎点、行/列连线、2x2成组、索驱路径、TF轴显示、点云参数、视角模式和跟随原点。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/TopicLayerController.js`
- `src/tie_robot_web/frontend/test/topicLayerStatePersistence.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && node test/topicLayerStatePersistence.test.mjs && for test_file in test/*.mjs; do node "$test_file" || exit 1; done && npm run build; sudo systemctl restart tie-robot-frontend.service && systemctl show tie-robot-frontend.service --property=ActiveState`
- `SubState --no-pager`

### 后续注意

- 暂无。

## 2026-05-06 13:50 - 绑扎2x2成组改为严格DP棋盘块

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：后端动态绑扎规划在有Surface-DP行列编号时，不再用相邻匹配凑组，而是按固定2x2棋盘块生成matrix_2x2；仅奇数行/列最外侧剩余点生成matrix_2x2_edge_pair。扫描写bind_path时优先使用merged_world_points中的完整显式global_row/global_col DP网格点，避免只用planning checkerboard成员导致16x16网格被滤成232点后产生大量2点组。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `catkin_make; devel/lib/tie_robot_process/test_dynamic_bind_planning; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; git diff --check`

### 后续注意

- 暂无。

## 2026-05-06 13:34 - 前端恢复2x2成组可视化

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06 用户要求恢复2x2成组可视化。前端新增独立showBindGroups图层和显示与视角中的“2x2成组”开关；成组层只消费pseudo_slam_bind_path.json的areas/groups执行分组，matrix_2x2画闭合四边框，matrix_2x2_edge_pair画成组边，并用独立颜色/z偏移区别于DP行列线。行列网格仍使用完整DP grid_points。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/bindPathGeometry.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/config/topicLayerCatalog.js`
- `src/tie_robot_web/frontend/test/bindPathGeometry.test.mjs`
- `src/tie_robot_web/frontend/test/bindPathLayerControls.test.mjs`

### 关键决策

- 见摘要。

### 验证证据

- `node test/*.test.mjs通过；相关Python unittest 4项通过；npm run build通过；服务重启后/index.html加载index-BPuBT63s.js；API确认grid_point_count=256、areas=98、groups=98；本地账本成组线段数146。完整test_workspace_picker_web.py仍有5个既有无关失败。`

### 后续注意

- 暂无。

## 2026-05-06 13:27 - 前端行列线严格使用DP行列编号

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06 用户指出3D行/列线仍缺很多并询问是否严格按后端行列。复查发现前端此前只用236个planning checkerboard点，且行线/列线排序仍按世界X/Y；实际pseudo_slam_points.json中256个识别点都有Surface-DP global_row/global_col。已改为/api/planning/bind-path暴露全部DP行列grid_points(当前256点/16行/16列，含20个非planning点并携带outlier标记)，前端行线按同row内global_col升序、列线按同col内global_row升序连接，不再用世界坐标排序。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/utils/bindPathGeometry.js`
- `src/tie_robot_web/frontend/test/bindPathGeometry.test.mjs`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `node test/*.test.mjs通过；相关Python unittest 4项通过；npm run build通过；curl /api/planning/bind-path确认grid_point_count=256 rows=16 cols=16 non_planning=20；完整test_workspace_picker_web.py仍保留5个既有无关失败。`

### 后续注意

- 暂无。

## 2026-05-06 13:12 - 前端行列线改用完整可规划grid点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06 修复3D中部分绑扎点未被行/列线穿起来的问题：根因是前端点层显示/cabin/pseudo_slam_markers原始点，而行/列线只从pseudo_slam_bind_path.json执行分组点生成；当前现场账本中执行分组228点，可规划棋盘格236点，导致8个可规划但未匹配执行分组的点不在线上。/api/planning/bind-path 现在从pseudo_slam_points.json按scan_session_id/path_signature对齐后暴露grid_points完整可规划点集；前端行/列线和绑扎点优先使用grid_points，raw marker点仅作为无bind path时的回退。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/utils/bindPathGeometry.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/test/bindPathGeometry.test.mjs`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `node test/*.test.mjs 全部通过；npm run build 通过；test_bind_path_payload_exposes_full_planning_grid_points_for_row_column_lines 和相关Scene3D静态测试通过；完整test_workspace_picker_web.py仍保留5个既有无关失败。`

### 后续注意

- 暂无。

## 2026-05-06 12:59 - 前端3D显示绑扎行列和独立图层

<!-- AGENT-MEMORY: entry -->

### 摘要

- 三维Scene的显示与视角面板新增绑扎点、行/列连线、索驱路径三个独立图层开关；bind path JSON中的global_row/global_col用于生成每行/每列穿过绑扎点的独立连线。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/bindPathGeometry.js;src/tie_robot_web/frontend/src/views/Scene3DView.js;src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/frontend/src/config/topicLayerCatalog.js`

### 关键决策

- 新增frontend/src/utils/bindPathGeometry.js纯几何工具，从pseudo_slam_bind_path.json的areas/groups/points生成绑扎点坐标和row/column线段；Scene3DView新增bindPathPoints、bindRowLines、bindColumnLines，索驱路径由showCabinPath控制，绑扎点由showBindPoints控制，行列线由showBindGridLines控制。

### 标签

- `frontend`
- `threejs`
- `bind_path`
- `visualization`

### 验证证据

- `frontend全部node test/*.test.mjs通过；npm run build通过并重建src/tie_robot_web/web；git diff --check通过；相关Python静态用例scene_view/tf_axes/topic_layers筛选通过。全量test_workspace_picker_web.py仍有5个既有失败，分别在IR TCP边界、状态胶囊、TF guard、toolbar、旧panel守护。`

### 后续注意

- 暂无。

## 2026-05-06 12:52 - 绑扎路径按世界坐标最小角定向

<!-- AGENT-MEMORY: entry -->

### 摘要

- 动态绑扎规划使用DP显式行列时，行列数字方向可能与世界坐标X/Y方向相反，排序必须从世界坐标最小绑扎点开始。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp;src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- build_dynamic_bind_area_entries_from_scan_world根据规划点在行/列key上的世界Y/X均值判断是否反转row_keys/column_keys，使dense row/col的0,0对应世界坐标最小角；组内点继续保持棋盘行列顺序。

### 标签

- `pseudo_slam`
- `dynamic_bind_planning`
- `grid_order`

### 验证证据

- `新增并通过DynamicBindPlanningTest.StartsProvidedGridTraversalFromMinimumWorldCoordinatePoint；完整test_dynamic_bind_planning 13/13通过；test_scan_artifact_write_guard 10/10通过；git diff --check通过；catkin_make -DCATKIN_WHITELIST_PACKAGES=""通过。`

### 后续注意

- 暂无。

## 2026-05-06 12:48 - 前端 header 长按0.5秒重启

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：header 中索驱、末端、视觉三个状态胶囊长按重启阈值从 1 秒改为 0.5 秒，充能填充/扫光动画同步改为 0.5s；tooltip 文案改为“长按0.5秒重启”。构建后已重启 tie-robot-frontend.service 并确认 active/running。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && node test/statusChipPressBehavior.test.mjs && for test_file in test/*.mjs; do node "$test_file" || exit 1; done && npm run build; sudo systemctl restart tie-robot-frontend.service && systemctl show tie-robot-frontend.service --property=ActiveState`
- `SubState --no-pager`

### 后续注意

- 暂无。

## 2026-05-06 12:43 - Surface-DP行列直通绑扎规划

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06 将扫描绑扎点行列来源从 process 侧 80mm 世界坐标聚类改为优先使用 Surface-DP 收束底图行列：PointCoords 新增 has_grid_index/global_row/global_col；manual_workspace_s2 在 DP 交点输出时按最近 vertical_lines/horizontal_lines 填充行列；suoquNode 在 Scepter_depth_frame->map 变换后保留这些字段；build_checkerboard_info_by_global_index 若检测到显式 DP 行列则直接用它构建 checkerboard_info，只有没有显式行列时才回退旧 cluster_checkerboard_axis_centers 80mm 聚类。

### 影响范围

- `src/tie_robot_msgs/msg/PointCoords.msg;src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp;src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py;src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 扫描规划行列以 Surface-DP 收束底图行列为第一来源，不再在 process 侧优先靠 80mm 世界坐标聚类重猜。

### 标签

- `surface-dp`
- `checkerboard`
- `planning`
- `ros-msg`

### 验证证据

- `python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py: 10/10 pass; PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py: 98/98 pass; python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py: 10/10 pass; catkin_make test_dynamic_bind_planning && devel/lib/tie_robot_process/test_dynamic_bind_planning: 12/12 pass; git diff --check relevant files: pass; catkin_make whitelist tie_robot_msgs/tie_robot_hw/tie_robot_perception/tie_robot_process/tie_robot_web: pass; catkin_make -DCATKIN_WHITELIST_PACKAGES="": pass`

### 后续注意

- 暂无。

## 2026-05-06 12:22 - 绑扎点成组改为棋盘格邻接最大匹配

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06 将动态绑扎点成组从固定 2x2 分块/补边逻辑改为基于 planning_global_row/planning_global_col 的行列邻接最大匹配：同一棋盘格内曼哈顿相邻点可成 2 点组，两个相邻匹配边可合并为合法 2x2；不再因旧固定块边界漏掉可相邻成组的点。无棋盘格索引、仅世界坐标聚类时仍要求存在完整 2x2 棋盘结构作为防噪声门槛。当前旧 pseudo_slam 数据未重生成：文件内仍为旧规划 160 点；用同一批 planning 棋盘格估算，新规则可覆盖 226/228 个唯一格点，仅 2 个无相邻匹配。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp;src/tie_robot_process/test/test_dynamic_bind_planning.cpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 绑扎点组生成以棋盘格行列邻接覆盖优先，禁止再用固定 2x2 分块作为能否成组的硬边界。

### 标签

- `planning`
- `bind-path`
- `checkerboard`

### 验证证据

- `catkin_make test_dynamic_bind_planning && devel/lib/tie_robot_process/test_dynamic_bind_planning: 12/12 pass; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py: 9/9 pass; git diff --check relevant files: pass; catkin_make whitelist tie_robot_msgs/tie_robot_hw/tie_robot_process/tie_robot_web: pass; catkin_make -DCATKIN_WHITELIST_PACKAGES="": pass`

### 后续注意

- 暂无。

## 2026-05-06 11:57 - 固定棋盘块内允许相邻2点组

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06 用户指出严格行列后漏点更多。根因是 dynamic_bind_planning 只输出完整固定2x2，导致同一固定棋盘块内缺1-2格时，明明相邻的点也被跳过。已改为：固定行列块仍不滑动、不跨块；完整2x2输出4点组；不完整固定块内若存在相邻两点，则输出一个matrix_2x2_edge_pair；单点或对角点仍不规划。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `catkin_make test_dynamic_bind_planning && devel/lib/tie_robot_process/test_dynamic_bind_planning; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process;tie_robot_web'; catkin_make -DCATKIN_WHITELIST_PACKAGES=''`

### 后续注意

- 暂无。

## 2026-05-06 11:53 - 前端 header 长按1秒并保持充能动画

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06：header 中索驱、末端、视觉三个状态胶囊长按重启阈值改为 1 秒，充能填充/扫光动画也改为 1s。修复现场看不到充能动画的问题：setStatusChipState 状态刷新会重写 className，之前会擦掉 is-long-press-charging/is-long-press-complete；现在刷新时保留这两个长按动画态，同时提高填充可见度。构建后重启 tie-robot-frontend.service 并确认 active/running。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && node test/statusChipPressBehavior.test.mjs && for test_file in test/*.mjs; do node "$test_file" || exit 1; done && npm run build; sudo systemctl restart tie-robot-frontend.service && systemctl show tie-robot-frontend.service --property=ActiveState`
- `SubState --no-pager`

### 后续注意

- 暂无。

## 2026-05-06 11:45 - 绑扎路径使用planning棋盘格行列

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06 修正 pseudo_slam_bind_path 生成链路：bind_path_checkerboard_info_by_idx 改用规划过滤后的 checkerboard_info_by_idx，而不是 raw merged_checkerboard_info_by_idx；路径规划和 pseudo_slam_points.json 的 planning_global_row/planning_global_col 统一同源，避免严格行列分组时可视化行列与实际路径行列不一致。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; catkin_make test_dynamic_bind_planning && devel/lib/tie_robot_process/test_dynamic_bind_planning; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process;tie_robot_web'; catkin_make -DCATKIN_WHITELIST_PACKAGES=''`

### 后续注意

- 暂无。

## 2026-05-06 11:23 - 扫描绑扎点严格行列分组

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-06 将 dynamic_bind_planning 的绑扎点成组从覆盖优先滑动 DP 改为严格行列固定分块：提供 global_row/global_col 时按 0..max 真实网格展开，不压缩缺行缺列；两列一带、两行一块，只输出完整固定 2x2，只有最后一行/最后一列允许真实边缘 2 点组；内部残点不再滑窗或补偿成组。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `catkin_make test_dynamic_bind_planning && devel/lib/tie_robot_process/test_dynamic_bind_planning; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process;tie_robot_web'; catkin_make -DCATKIN_WHITELIST_PACKAGES=''`

### 后续注意

- 暂无。

## 2026-05-05 04:44 - 执行微调改用TCP坐标执行盒ROI

<!-- AGENT-MEMORY: entry -->

### 摘要

- 执行层视觉微调不再用像素矩形或扫描工作区作为Hough ROI；MODE_EXECUTION_REFINE会把raw_world像素按Scepter_depth_frame->gripper_frame外参批量转换，只保留TCP执行盒x[0,380]/y[0,3330]/z[0,3160]mm内的像素和候选点，范围外视图不参与Hough。TCP遮挡黑色mask仍只负责遮挡置黑，不是ROI。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/tcp_display.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile pointai modules; git diff --check relevant files`

### 后续注意

- 暂无。

## 2026-05-05 04:43 - 当前画面视觉先于索驱状态账本守卫

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05 修复确认工作区/触发视觉识别不启动扫描层视觉的问题：前端runSavedS2走/web/cabin/start_pseudo_slam_scan scan_strategy=3，后端kCurrentFrameNoMotion分支必须先调用/pointAI/process_image request_mode=3触发Surface-DP扫描视觉，再检查索驱状态是否新鲜/静止/有效来决定能否覆盖pseudo_slam_points.json和pseudo_slam_bind_path.json；索驱状态异常时返回'已触发扫描层视觉，但未覆盖本地绑扎点文件'。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; node src/tie_robot_web/frontend/test/taskActionController.test.mjs; node src/tie_robot_web/frontend/test/taskActionSurfaceDpRecognition.test.mjs; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process;tie_robot_web'; git diff --check -- src/tie_robot_process/src/suoquNode.cpp src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 后续注意

- 暂无。

## 2026-05-05 04:33 - 扫描分组修复重复格点相邻空格漏组

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05 现场图中可分成2x2的一组散点被漏掉，根因是扫描棋盘格身份在边缘出现重复格点和相邻空格：同一global_row/global_col下有两个物理相邻点，旁边真实格为空，动态规划按格占用选择后会漏掉可成组点。已在pseudo_slam扫描棋盘格身份构建阶段把重复格点按物理坐标迁移到有邻居支撑的相邻空格，并在dynamic_bind_planning里增加同类兜底修复旧产物；新增C++回归RecoversPhysicallyAdjacentLeftoversFromDuplicateGridCellGap覆盖该模式。

### 影响范围

- `src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp`
- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- `src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process;tie_robot_web'; catkin_make test_dynamic_bind_planning && devel/lib/tie_robot_process/test_dynamic_bind_planning; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; node src/tie_robot_web/frontend/test/taskActionController.test.mjs`

### 后续注意

- 暂无。

## 2026-05-05 04:28 - 执行层恢复TCP遮挡黑色mask

<!-- AGENT-MEMORY: entry -->

### 摘要

- 在保持 pointAI 全局无固定像素 ROI 的前提下，执行层视觉微调恢复独立 TCP 遮挡黑色 mask：仅 MODE_EXECUTION_REFINE 在 Hough 二值化前把 tcp_occlusion_mask_rect=(160,0,523,80) 区域置零；它不参与候选点 ROI 过滤，也不恢复 roi_reject/ROI 诊断。扫描层不应用该遮挡。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 后续注意

- 暂无。

## 2026-05-05 04:21 - 视觉图像层移除固定像素ROI

<!-- AGENT-MEMORY: entry -->

### 摘要

- pointAI 图像层不再使用 point1/point2 静态像素矩形 ROI：删除 get_roi_pixel_mask/is_point_in_roi 绑定、执行 Hough 的 roi_reject 过滤和诊断、执行范围 mask 对静态 ROI 的叠加，并禁用执行层顶部固定像素遮挡。候选点仍会经过有效 3D 坐标、近点去重、手动/规划工作区和执行范围过滤。执行底图 Hough二值诊断标记现在为 H/ZERO/OUT/DUP/SEL。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 后续注意

- 暂无。

## 2026-05-05 03:52 - 执行 Hough 二值图诊断标记

<!-- AGENT-MEMORY: entry -->

### 摘要

- 执行底图 Hough二值图现在会在最终输出点之外叠加 Hough 原始交点和 ROI/ZERO/执行框外/近点去重移除标记：H=Hough raw，ROI=ROI拒绝，ZERO=无有效3D坐标，OUT=执行微调框或工作区外，DUP=近点去重移除，SEL/编号=最终输出点。现场漏点时优先看此图层判断候选点掉在哪一道过滤门。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 后续注意

- 暂无。

## 2026-05-05 03:48 - 固定识别位姿视觉测试结果

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05 用户确认上一轮没有移动到识别位姿后，改用 /cabin/start_pseudo_slam_scan_with_options scan_strategy=2 固定工作区扫描重测：服务返回 success，pseudo_slam_points.json=256 点，pseudo_slam_bind_path.json=64 区域/64 组/246 绑扎点；分组为 59 个 matrix_2x2 + 5 个相邻 edge_pair，无重复点、无非法组尺寸、无非相邻2点组。此前 scan_strategy=3 当前画面无运动帧只出 237 点/22 路径点，不作为固定识别位姿效果判断。

### 影响范围

- `src/tie_robot_process/data/pseudo_slam_points.json; src/tie_robot_process/data/pseudo_slam_bind_path.json`

### 关键决策

- 见摘要。

### 标签

- `vision-test`
- `fixed-scan-pose`
- `pseudo-slam`

### 验证证据

- `rosservice call /cabin/start_pseudo_slam_scan_with_options enable_capture_gate:false scan_strategy:2; 统计 areas=64 groups=64 points_in_groups=246 unique=246 group_sizes={4:59`
- `2:5} bad_adjacent_pairs=0`

### 后续注意

- 暂无。

## 2026-05-05 03:36 - 扫描绑扎路径分组改为最大覆盖

<!-- AGENT-MEMORY: entry -->

### 摘要

- 修复三维规划漏点：dynamic_bind_planning 的网格分组从先选2x2再固定补边改为全局最大覆盖选择，覆盖数优先、同等覆盖下保留2x2优先，只输出4点matrix_2x2或相邻2点edge_pair；suoquNode 写 bind path 时使用扫描原始棋盘格身份，不再用旧规划过滤结果删点，并在棋盘格成员为空时回退到扫描代表点直接聚类，避免写出 areas: []。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp; src/tie_robot_process/src/suoquNode.cpp; src/tie_robot_process/test/test_dynamic_bind_planning.cpp; src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 见摘要。

### 标签

- `planning`
- `pseudo-slam`
- `bind-path`

### 验证证据

- `python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning && catkin_test_results build/test_results/tie_robot_process; catkin_make; git diff --check -- related files`

### 后续注意

- 暂无。

## 2026-05-05 03:26 - 执行底图Hough二值叠加识别点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05 前端图像层“执行底图 Hough二值”对应 /perception/lashing/execution_refine_base_image 现在在执行微调 Hough 输出点生成后重新发布 bgr8 调试图：底图仍是 Hough 二值图，输出点用黄色圆圈、红色中心和编号标出；无点/失败时仍保留二值底图。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_scan_and_execution_base_images_are_published_for_frontend_image_layer; python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m unittest src/tie_robot_control/test/test_single_point_bind_chain.py src/tie_robot_perception/test/test_gripper_tf_broadcaster.py`

### 后续注意

- 暂无。

## 2026-05-05 03:06 - DP debug base images overlay detected points

<!-- AGENT-MEMORY: entry -->

### 摘要

- 扫描 Surface-DP 底图发布链路现在会把 surface_result.rectified_intersections 画到 fused_instance_response 和 completed_surface_response 调试图上；有有效点时以 bgr8 发布黄色点+黑色描边，无有效点时保持 mono8 灰度。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py; src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 关键决策

- 见摘要。

### 标签

- `perception`
- `surface-dp`
- `debug-image`

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py; git diff --check -- src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`

### 后续注意

- 暂无。

## 2026-05-05 03:05 - 实际移动TCP显示与当前虎口相对坐标

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05 修复 TCP 移动时前端 3D 橙色方块不动和执行覆盖图 tcp 数值走向反的问题：Scene3DView 现在把 /moduan/moduan_gesture_data 的线模当前位置叠加到 gripper_frame 后显示实际移动 TCP；pointAI 订阅同一线模状态，执行微调覆盖图 tcp=(...) 先将相机点转到 gripper_frame 绝对线模目标坐标，再减去当前线模 X/Y/Z，显示为以当前运动虎口为原点的相对坐标。执行层写 PLC 仍使用绝对线模目标坐标。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/tcp_display.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `node test/gripperTfCalibration.test.mjs; for f in test/*.test.mjs; do node "$f" || exit 1; done; python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m unittest src/tie_robot_perception/test/test_gripper_tf_broadcaster.py src/tie_robot_control/test/test_single_point_bind_chain.py; npm run build`

### 后续注意

- 暂无。

## 2026-05-05 02:57 - 绑扎分组改为最大2x2匹配

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：为解决视觉三维界面中零散点明明可组成2x2却被边缘二点提前消耗的问题，动态绑扎规划从固定偶数块/局部贪心改为全局网格2x2候选DP：先最大化完整2x2数量，再用几何规整度和两列带遍历顺序打破平局；重复row/col cell保留为候选池，在2x2候选内选择几何更规整的点。bind path改用过滤后同步的checkerboard membership，避免pseudo_slam_points与pseudo_slam_bind_path行列身份不一致。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp; src/tie_robot_process/src/suoquNode.cpp; src/tie_robot_process/test/test_dynamic_bind_planning.cpp; src/tie_robot_process/test/test_scan_artifact_write_guard.py; src/tie_robot_process/data/pseudo_slam_bind_path.json`

### 关键决策

- 见摘要。

### 标签

- `planning`
- `bind-path`
- `grouping`
- `2x2`
- `dp`

### 验证证据

- `catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning -DCATKIN_WHITELIST_PACKAGES=tie_robot_process：9/9通过；python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py：6/6通过；catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_process'：通过；/api/system/restart_ros_stack：返回所有服务running；/api/planning/bind-path：57组，46个4点2x2、11个边缘二点，bad_count=0。`

### 后续注意

- 暂无。

## 2026-05-05 02:42 - 前端相机-TCP外参输入防旧TF回刷

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05 修复设置页相机-TCP外参点击应用后数字弹回旧值：UIController 现在记录上一次由 TF/服务端写入输入框的基准值，普通 TF 回流只有在输入未被人工改动时才刷新输入；应用成功的 forceInputs 仍会接收服务端确认值。回归测试覆盖：人工把 X 从 301 改到 305 后，即使旧 TF 在点击前回流，应用按钮仍读取 305。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `node test/gripperTfCalibration.test.mjs; for f in test/*.test.mjs; do node "$f" || exit 1; done; npm run build`

### 后续注意

- 暂无。

## 2026-05-05 02:35 - 执行结果覆盖图显示 TCP 虎口坐标

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05 用户要求执行结果覆盖原图上的标签显示 TCP/虎口坐标。pointAI 的 PointsArray 仍保持 Scepter_depth_frame raw camera 坐标；新增显示专用 camera->tcp jaw 换算，读取 gripper_tf.yaml 的 translation_mm/rotation_rpy 并按 mtime 缓存，执行微调结果图标签显示 tcp=(x,y,z)。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/tcp_display.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 标签

- `pointai`
- `tf`
- `tcp`
- `overlay`

### 验证证据

- `source devel/setup.bash; python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m unittest src/tie_robot_process/test/test_tf_coordinate_contract.py; python3 -m unittest src/tie_robot_control/test/test_single_point_bind_chain.py`

### 后续注意

- 暂无。

## 2026-05-05 02:25 - 工作区四角世界坐标字段改为 map 口径

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05 清理全仓 TF 命名契约残留：工作区四角世界坐标当前字段统一为 corner_world_map_frame；运行代码仍通过拼接出的旧 key 兼容历史 manual_workspace_quad.json，但仓库文本不再出现旧世界坐标系名。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/tools/scan_response_full_evaluation.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`
- `docs/reports/scan_vision_obsidian_vault/00_Inbox/现场排查问题清单.md`

### 关键决策

- 见摘要。

### 标签

- `tf`
- `map`
- `workspace`
- `pointai`

### 验证证据

- `python3 -m unittest src/tie_robot_process/test/test_tf_coordinate_contract.py; source devel/setup.bash; python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 后续注意

- 暂无。

## 2026-05-05 02:19 - 单点绑扎相机点转 TCP 局部坐标

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05 排查现场截图发现 pointAI 执行微调结果图中的 tcp=(x,y,z) 实际来自 /perception/lashing/points_camera 的 Scepter_depth_frame 原始相机坐标，已改为 cam= 标签；/moduan/sg 现在在执行前用 TF 将 Scepter_depth_frame 点转换到 gripper_frame/TCP 局部坐标，线性模组执行层完整校验 X/Y/Z 行程。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`

### 关键决策

- 见摘要。

### 标签

- `single-bind`
- `tf`
- `moduan`
- `pointai`

### 验证证据

- `python3 -m unittest src/tie_robot_control/test/test_single_point_bind_chain.py; source devel/setup.bash; python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; source devel/setup.bash; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_control'`

### 后续注意

- 暂无。

## 2026-05-05 02:07 - 绑扎分组禁止斜二点与重复补点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：动态绑扎规划分组收紧为只输出完整2x2或同一行/列相邻边缘二点；3点残块优先选择真实边缘二点，孤点只与pending里的相邻孤点补组，禁止与已输出点重复配对。本地pseudo_slam_bind_path.json已按新规则重排为64组：62个2x2、2个边缘二点，斜二点/重复点/非矩形2x2检查为0。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp; src/tie_robot_process/test/test_dynamic_bind_planning.cpp; src/tie_robot_process/data/pseudo_slam_bind_path.json`

### 关键决策

- 见摘要。

### 标签

- `planning`
- `bind-path`
- `grouping`
- `pseudo-slam`

### 验证证据

- `catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning -DCATKIN_WHITELIST_PACKAGES=tie_robot_process：7/7通过；python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py：5/5通过；catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_process'：通过；/api/planning/bind-path返回64组且bad_count=0；ROS全栈已通过/api/system/restart_ros_stack重启成功。`

### 后续注意

- 暂无。

## 2026-05-05 02:06 - 单点绑扎恢复执行层 Hough 链路

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：复查旧 20260403 chassis_ctrl 后确认前端 /web/moduan/single_bind 触发 /moduan/sg，后端调用 /pointAI/process_image 的旧 pre_img 平面/深度二值化 + HoughLinesP + 交点聚类链路，过滤可执行范围后将全部返回点写入线性模组队列并等待 FINISHALL；不是从返回点里挑最近 1 个点。当前工程 /moduan/sg 已从 MODE_BIND_CHECK=2 改为 MODE_EXECUTION_REFINE=4，继续由前端只调用后端原子服务。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_control/include/tie_robot_control/moduan/runtime_state.hpp`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_control/test/test_single_point_bind_chain.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_control/test/test_single_point_bind_chain.py src/tie_robot_process/test/test_scan_artifact_write_guard.py; bash -lc 'source devel/setup.bash && python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py'; bash -lc 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_hw;tie_robot_control"'`

### 后续注意

- 暂无。

## 2026-05-05 01:50 - 前端图像层加入扫描和执行底图

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：将扫描层 Surface-DP 使用的 fused_instance_response 底图和 completed_surface_response/补全面 DP 收束底图分别发布为 /perception/lashing/scan_surface_dp_base_image 与 /perception/lashing/scan_surface_dp_completed_surface_image；将执行微调 Hough 使用的二值底图发布为 /perception/lashing/execution_refine_base_image。三个 topic 都是 sensor_msgs/Image，由 pointAINode latch 发布，并已加入新前端图像下拉。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/config/imageTopicCatalog.js`

### 关键决策

- 见摘要。

### 验证证据

- `npm run build；Node 前端测试通过；pointAI/Surface-DP 98项测试通过；rostopic info 确认三个新 Image topic 由 /pointAINode 发布；ROS 全栈和前端服务已重启且 active/running。`

### 后续注意

- 暂无。

## 2026-05-05 01:42 - 前端 header 长按需满格后重启

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：修复 header 中索驱、末端、视觉状态胶囊长按重启时动画未填满就触发的问题。根因有两点：JS 在 2s 定时器里立即移除 is-long-press-charging 并触发重启，可能撤掉最终满格帧；扫光伪元素原本按自身小宽度移动，宽胶囊无法扫完整。现在长按满 2s 后先进入 is-long-press-complete，完整填充保持约 240ms 再清理；扫光伪元素宽度为 100%，从 translateX(-100%) 扫到 translateX(100%)，确保覆盖整个胶囊。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && node test/statusChipPressBehavior.test.mjs && for test_file in test/*.mjs; do node "$test_file" || exit 1; done && npm run build; sudo systemctl restart tie-robot-frontend.service && systemctl show tie-robot-frontend.service --property=ActiveState`
- `SubState --no-pager`

### 后续注意

- 暂无。

## 2026-05-05 01:40 - 绑扎分组改用global_row_col避免倾斜世界坐标拆行

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：用户截图指出三维界面仍大量二点组且混乱。复查当前 pseudo_slam_bind_path.json 发现实际文件为111区，其中105个matrix_2x2_edge_pair、只有6个matrix_2x2，问题在后端规划而非前端渲染。根因是 dynamic_bind_planning 上一版仍按世界X/Y重新聚类；现场钢筋行列有倾斜/曲率，同一global_row在世界Y上被拆成多行，导致2x2块变成横向二点组。已新增 DynamicBindGridIndex 并让 build_dynamic_bind_area_entries_from_scan_world 优先使用后端已有 global_row/global_col 分组；suoquNode 为绑扎路径传入完整merged checkerboard表格点与grid索引，保留 pseudo_slam_points 的planning/outlier诊断但不再让过滤后的孔洞打散路径分组。当前无运动扫描重生成后 pseudo_slam_bind_path.json=64区域/64组/252点，其中62个四点组、2个二点边缘组。

### 影响范围

- `src/tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp;src/tie_robot_process/src/planning/dynamic_bind_planning.cpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/test/test_dynamic_bind_planning.cpp;src/tie_robot_process/data/pseudo_slam_bind_path.json;src/tie_robot_process/data/pseudo_slam_points.json`

### 关键决策

- 见摘要。

### 验证证据

- `红灯：新增UsesProvidedGridIndicesInsteadOfReclusteringTiltedWorldRows测试时编译失败，证明接口不能接收global_row/global_col；绿灯：catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning 5/5 PASS；test_scan_artifact_write_guard.py 5/5 PASS；git diff --check PASS；catkin_make whitelist PASS；restart_ros_stack成功；rosservice call /cabin/start_pseudo_slam_scan_with_options {enable_capture_gate:false`
- `scan_strategy:3} 成功，生成64区域/252点；/api/planning/bind-path 返回200且区域大小统计{2:2`
- `4:62}。`

### 后续注意

- 暂无。

## 2026-05-05 01:37 - 前端 header 长按重启改为2秒

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：新前端 header 中索驱、末端、视觉三个状态胶囊的长按重启阈值从 0.8 秒改为 2 秒；充能横向填充和扫光动画也同步改为 2s 完成。短按语义不变：在线 success 短按关闭，离线或非 success 短按启动；长按满 2 秒触发对应 restart*Subsystem。前端构建后已重启 tie-robot-frontend.service 并确认 ActiveState=active、SubState=running。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && node test/statusChipPressBehavior.test.mjs && for test_file in test/*.mjs; do node "$test_file" || exit 1; done && npm run build; sudo systemctl restart tie-robot-frontend.service && systemctl show tie-robot-frontend.service --property=ActiveState`
- `SubState --no-pager`

### 后续注意

- 暂无。

## 2026-05-05 01:29 - 扫描绑扎分组只允许4点组和边缘2点组

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：用户纠正：绑扎点分组应以4个点为一组，只有到边缘无法组成2x2时才允许2个点为一组，不能出现1点组或3点组。已调整 dynamic_bind_planning：2x2完整块输出 matrix_2x2；不足4点时拆为2点组 matrix_2x2_edge_pair；单独角点会与最近已规划邻点组成2点组以避免1点组，同时执行记忆仍按global row/col防重复绑扎。对应 gtest 改为验证3x3奇数边缘只出现4/2点组且唯一点集合覆盖完整表格。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp;src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `红灯：KeepsOddGridEdgesAsTwoPointGroupsWithoutOneOrThreePointGroups 在旧实现下失败，暴露1点角落组；绿灯：catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning 4/4 PASS；python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py 5/5 PASS；git diff --check PASS；catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process;tie_robot_web' PASS；/api/system/restart_ros_stack 成功，五个服务 active/running。`

### 后续注意

- 暂无。

## 2026-05-05 01:19 - 扫描绑扎路径改为表格2x2带状遍历

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：用户确认方案B后，将扫描层 pseudo_slam 绑扎路径主链从动态 seed/TCP 覆盖凑4点改为按当前绑扎点世界坐标表格直接规划。dynamic_bind_planning 现在按世界X/Y聚类出列/行中心，从最小X且最小Y角点开始，每两列为一带、每两行为一组，列带内按上下蛇形遍历；4点组标记 matrix_2x2，奇数边缘或缺点组标记 matrix_2x2_partial，不重复同一表格cell。suoquNode 不再对结果做旧的按行蛇形重排，避免破坏两列带顺序。当前本地256点样本估算会由旧44区/176点变为约64区/233唯一cell点。

### 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/test/test_dynamic_bind_planning.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `红灯：新增/改写 gtest 后旧主链 4x4 只产出3区、3x3只产出1区；绿灯：catkin_make run_tests_tie_robot_process_gtest_test_dynamic_bind_planning 4/4 PASS；python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py 5/5 PASS；git diff --check PASS；catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process;tie_robot_web' PASS；/api/system/restart_ros_stack 成功，rosbridge、三驱动、backend active/running。`

### 后续注意

- 暂无。

## 2026-05-05 00:48 - 编译后自动重启运行服务

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：用户明确要求：以后 Codex 每次编译/构建完，如果改动影响运行态，就顺手重启对应服务。ROS/C++ 后端或 web action bridge 编译后优先走 /api/system/restart_ros_stack 或等价 systemd 重启，确保 rosbridge、驱动、后端换新进程；前端 npm run build 后重启 tie-robot-frontend.service 或确认静态服务可读取新产物。重启后检查 systemd ActiveState/SubState。

### 影响范围

- `docs/agent_memory/session_log.md`
- `docs/agent_memory/current.md`

### 关键决策

- 见摘要。

### 验证证据

- `本次已调用 /api/system/restart_ros_stack 成功；sudo -n systemctl restart tie-robot-frontend.service 成功；systemctl show 显示 frontend/rosbridge/backend active running。`

### 后续注意

- 暂无。

## 2026-05-05 00:39 - 纠正视觉触发为当前画面无运动记录

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：用户反馈点击前端‘触发视觉识别’后机器运动。根因是上一版把 runSavedS2 接到 start_pseudo_slam_scan action 的 scan_strategy=2，即 kFixedManualWorkspace，后端会移动索驱到固定识别位姿。已纠正：‘触发视觉识别’只发 scan_strategy=3（kCurrentFrameNoMotion），后端只用当前画面和当前索驱/TF状态请求 /pointAI/process_image mode=3、生成并覆盖 pseudo_slam_points.json/pseudo_slam_bind_path.json、重置 bind_execution_memory.json，不调用 move_cabin_pose_via_driver、不等待轴到位、不写 TCP_Move。‘固定扫描规划’按钮仍保留 scan_strategy=2，会移动机器。

### 影响范围

- `src/tie_robot_msgs/action/StartPseudoSlamScanTask.action`
- `src/tie_robot_msgs/srv/StartPseudoSlamScan.srv`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_web/src/web_bridge/action_bridge.cpp`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`
- `src/tie_robot_web/frontend/test/taskActionSurfaceDpRecognition.test.mjs`
- `src/tie_robot_web/frontend/test/taskActionController.test.mjs`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `node frontend .mjs tests; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; targeted WorkspacePickerWebTest visual-trigger tests; npm run build; catkin_make -DCATKIN_WHITELIST_PACKAGES=tie_robot_msgs;tie_robot_hw;tie_robot_process;tie_robot_web`

### 后续注意

- 暂无。

## 2026-05-05 00:26 - 前端视觉触发改走扫描action覆盖绑扎点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：用户要求每次点击前端‘触发视觉识别’都覆盖旧绑扎点并重新生成本地文件。已将 runSavedS2/triggerSurfaceDpRecognition 从直接调用 /pointAI/process_image request_mode=3 改为发送 /web/cabin/start_pseudo_slam_scan action，goal 为 enable_capture_gate=false、scan_strategy=2；后端固定识别位姿扫描链路会原子写 pseudo_slam_points.json、pseudo_slam_bind_path.json 并重置 bind_execution_memory.json。工作区保存确认后的自动视觉触发也改走同一 action；按钮启用条件改为 startPseudoSlamScanActionClient。保留 /pointAI/process_image 给视觉调试和底层识别服务，不再作为前端主按钮覆盖文件入口。

### 影响范围

- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/taskActionSurfaceDpRecognition.test.mjs`
- `src/tie_robot_web/frontend/test/taskActionController.test.mjs`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node frontend/test/*.mjs targeted via find; python3 src/tie_robot_process/test/test_scan_artifact_write_guard.py; targeted WorkspacePickerWebTest visual-trigger tests; npm run build. Full WorkspacePickerWebTest still has 5 unrelated stale static failures: tcp workspace boundary`
- `recognition pose`
- `status capsule`
- `tf layer guard`
- `toolbar theme toggle.`

### 后续注意

- 暂无。

## 2026-05-05 00:05 - 补清扫描层旧命名与TF残留

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-05：继续清理扫描层旧版本残留。运行态 TF child prefix 从 pr_fprg_bind_point 改为 surface_dp_bind_point；前端源码内部 prFprgOverlay/triggerPrFprg 命名改为 surfaceDpOverlay/triggerSurfaceDp；重新 npm run build 并删除未引用旧 hash app 资产。残留扫描限定 active pointai、workspace_s2、frontend src/test 和 web app 产物，未再命中 PR-FPRG/prFprg/pr_fprg_bind_point。重启 pointAINode 后 /pointAI/process_image request_mode=3 返回 count=256，tf_echo 可查 surface_dp_bind_point_1，旧 pr_fprg_bind_point_1 不存在。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/bind_point_tf.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/web/assets/app`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; source devel/setup.bash && python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; source devel/setup.bash && python3 -m py_compile ...; npm run build; /pointAI/process_image request_mode=3 -> count=256; tf_echo surface_dp_bind_point_1 ok`
- `pr_fprg_bind_point_1 missing`

### 后续注意

- 暂无。

## 2026-05-04 23:59 - 清理扫描层旧版本残留

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04：用户确认清理旧版本残留。扫描主链 run_manual_workspace_s2_pipeline 现在只调用 Surface-DP，失败时直接返回失败并标记 legacy_depth_only_fallback=False，不再自动回退 run_manual_workspace_s2_depth_only_pipeline；scan_surface_dp 删除 legacy axis-aligned 线族兜底，物理先验无法解析时不再用旧 8x8 线族补 completed_surface；workspace_s2 的 8 根线/64 点评分偏置改为 LEGACY_* 常量，只保留给旧工具/测试；前端视觉触发和 project graph 文案改为 Surface-DP 物理先验，并清理未引用的旧 hash 静态资源。重启 pointAINode 后 /pointAI/process_image request_mode=3 返回 count=256，日志 lines=[16,16], points=256。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py; src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py; src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py; src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py; src/tie_robot_web/frontend/src/config/visualRecognitionMode.js; src/tie_robot_web/frontend/src/projectGraph/graphData.js; src/tie_robot_web/web; CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; source devel/setup.bash && python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; npm run build; source devel/setup.bash && rosservice call /pointAI/process_image request_mode:=3 -> count=256`

### 后续注意

- 暂无。

## 2026-05-04 23:47 - 前端 header 长按重启动画

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04：新前端 header 中索驱、末端、视觉三个状态胶囊长按重启时会先进入 is-long-press-charging 充能态，持续约 0.8 秒；按钮内部使用当前状态色做横向填充和扫光动画，计时满后移除充能态并触发对应 restart*Subsystem。松手、滑出或取消 pointer 会清除充能态并保留短按动作。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && for test_file in test/*.mjs; do node "$test_file" || exit 1; done && npm run build`

### 后续注意

- 暂无。

## 2026-05-04 23:45 - 修复视觉触发回退 64 点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04 23:45：用户反馈点击前端触发视觉识别仍只有 64 点。系统化排查确认前端按钮调用 /pointAI/process_image request_mode=3，服务稳定返回 count=64，pointAINode 日志为 Surface-DP lines=[8,8]；独立复刻同一工作区和实时帧可输出 256。根因是运行态 Surface-DP 只用 fused/completed 响应做最终物理选线，当 fused 纵向响应不足时物理选线失败，代码又把旧 workspace_s2 8x8 线族作为最终兜底。已改为多底图物理选线：completed/fused/Frangi/Hessian/depth_gradient/IR/combined/depth 中选择能满足 12-16 cm、15-18 根线的物理线族；全场模式不再允许旧 8x8 作为最终输出。重启 pointAINode 后 /pointAI/process_image request_mode=3 返回 count=256，日志 lines=[16,16], points=256, mean_surface=0.974。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `rosservice call /pointAI/process_image request_mode=3 -> count=256；rostopic /perception/lashing/points_camera/count -> 256；pointAINode log lines=[16`
- `16]`
- `points=256；test_scan_surface_dp_runtime.py Ran 6 OK；test_pointai_scan_only_pr_fprg.py Ran 88 OK；py_compile OK`

### 后续注意

- 暂无。

## 2026-05-04 23:41 - 前端 header 子系统短按/长按口径

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04：新前端 header 中索驱、末端、视觉三个状态胶囊采用短按/长按双语义。状态只决定在线/离线颜色和短按动作：在线 success 短按关闭对应子系统，离线或非 success 短按启动对应子系统；长按约 0.8 秒统一重启对应子系统。索驱/末端的 start/stop/restart 只控制各自驱动守护，不联动视觉算法；视觉 start 为 startCameraDriver + startAlgorithmStack，stop 为 stopAlgorithmStack + stopCameraDriver，restart 为 restartCameraDriver + restartAlgorithmStack。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/config/systemControlCatalog.js`
- `src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs`
- `src/tie_robot_web/frontend/test/systemControlCatalog.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && for test_file in test/*.mjs; do node "$test_file" || exit 1; done && npm run build`

### 后续注意

- 暂无。

## 2026-05-04 23:28 - 扫描层主链接入物理间距先验

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04：按用户要求将 12-16 cm 钢筋间距、(15-18)*(15-18) 规格接入运行态 scan_surface_dp 主链，替代旧 8x8 目标偏置。scan_surface_dp 现在按 rectified_geometry.resolution_mm_per_px 将 120-160 mm 转为像素间距；全场视野足够时使用 full_workspace 模式选择 15-18 根线，当前现场帧输出 16x16=256 点；视野只容纳少量钢筋时切到 visible_local，支持 2-18 根可见线并只输出当前可见局部交点，不从 2-3 根钢筋推断完整全场。效果页：http://192.168.6.99:8080/reports/live_surface_dp_physical_runtime_20260504_232729/index.html。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- `CHANGELOG.md`
- `src/tie_robot_web/web/reports/live_surface_dp_physical_runtime_20260504_232729`

### 关键决策

- 见摘要。

### 验证证据

- `python3 test_scan_surface_dp_runtime.py: Ran 5 tests OK；python3 test_pointai_scan_only_pr_fprg.py: Ran 88 tests OK；py_compile scan_surface_dp/manual_workspace_s2 OK；现场运行态检查 line_counts=[16`
- `16] point_count=256 mean_completed_surface_score≈0.952；报告 HTTP 200，12 张 PNG bad_count=0`

### 后续注意

- 暂无。

## 2026-05-04 23:18 - 物理间距先验重评扫描底图

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04 23:16：用户明确现场钢筋间距 12-16 cm、规格约 (15-18)*(15-18)。按当前 rectified 5 mm/px 等价 24-32 px，旧报告 8x8/64 点来自历史评分目标偏置，不符合现场点数先验。已对现场 live_raw_world_current_capture 跑 selected/combined/Hessian/Frangi/fused/depth_gradient+Hessian+Frangi 全流程：底图 -> Hessian/Frangi -> binary/skeleton -> completed_surface -> DP 曲线交点。所有候选按物理先验收敛到 16x16=256 点；推荐 combined_response_full_pipeline，代理分 0.917，selected 0.917，fused_instance_response 0.906，depth_gradient_hessian_frangi 0.841。报告 URL：http://192.168.6.99:8080/reports/live_physical_spacing_base_map_full_pipeline_20260504_231602/index.html。

### 影响范围

- `.debug_frames/live_physical_spacing_base_map_full_pipeline_20260504_231602`
- `src/tie_robot_web/web/reports/live_physical_spacing_base_map_full_pipeline_20260504_231602`

### 关键决策

- 见摘要。

### 验证证据

- `HTTP 200: curl report -> 200 5254；图片校验：29 张 PNG，bad_count=0；关键图 28_best_dp_original.png HTTP 200 354217 bytes`

### 后续注意

- 暂无。

## 2026-05-04 23:04 - 按 130933 底图模态重跑现场全量测试

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04 23:03：用户澄清要求按 .debug_frames/rebar_instance_graph_probe_20260430_130933 的底图模态/形态顺序，对现场实时图像测试 combined/fused -> Hessian/Frangi -> binary/skeleton -> completed_surface -> DP 曲线交点，并给 方案/点数/线数/代理分 表格和每步效果图。已抓取 live_raw_world_current_capture，输出 http://192.168.6.99:8080/reports/live_rebar_current_scheme_20260504_230352/index.html。结果：06_surface_ir_assisted_curve 64 点/[8,8]/0.933；05_surface_ridge_curve 64/[8,8]/0.923；04_surface_dp_curve 64/[8,8]/0.922；depth_gradient_hessian_frangi_dp 64/[8,8]/0.919；instance_graph_junctions_raw 217 点/无线族/0.722；旧 current_depth_only_runtime 493 点/[29,17]/0.472。17+37+12 张 PNG 全部非空，4 个网页 HTTP 200。

### 影响范围

- `src/tie_robot_web/web/reports/live_rebar_current_scheme_20260504_230352/index.html`
- `src/tie_robot_web/web/reports/live_rebar_instance_graph_modalities_20260504_230352/index.html`
- `src/tie_robot_web/web/reports/live_rebar_scheme_full_table_20260504_230352/index.html`
- `src/tie_robot_web/web/reports/live_rebar_gradient_hessian_frangi_20260504_230352/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `HTTP 200 for 4 report URLs; image nonblank checks: modalities 17/17`
- `full 37/37`
- `gradient 12/12`

### 后续注意

- 暂无。

## 2026-05-04 22:59 - 现场实时帧全量视觉实验

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04 22:58：按用户要求抓取此时此刻现场 ROS 话题 live_raw_world_current_capture（/Scepter/worldCoord/raw_world_coord + /Scepter/ir/image_raw）做全量实验，不使用旧 snapshot。报告挂载到 http://192.168.6.99:8080/reports/live_scan_reports_20260504_225857/index.html。结果：旧 depth-only 对照 current_depth_background_minus_filled 输出 493 点、线族 [29,17]、周期 17/30 px；全量代理指标推荐 06_surface_ir_assisted_curve，64 点、quality_score≈0.919；专项 depth_gradient+Hessian+Frangi DP 输出 64 点、[8,8]、mean_completed_surface_score≈0.931。两份报告分别生成 37 张和 12 张 PNG，HTTP 200 验证通过。

### 影响范围

- `src/tie_robot_web/web/reports/live_scan_reports_20260504_225857/index.html`
- `src/tie_robot_web/web/reports/live_scan_response_full_evaluation_20260504_225857/index.html`
- `src/tie_robot_web/web/reports/live_depth_gradient_hessian_frangi_scheme_20260504_225857/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `HTTP 200 for all three report URLs; image nonblank checks: 37/37 and 12/12 PNG nonblank`

### 后续注意

- 暂无。

## 2026-05-04 22:53 - 视觉报告挂载优先使用 8080 常驻前端服务

<!-- AGENT-MEMORY: entry -->

### 摘要

- 修正上一条视觉报告网页口径：临时 http.server 端口可能退出或被外部访问拒绝。以后视觉效果图报告优先复制/挂载到常驻 0.0.0.0:8080 前端静态服务的 /reports/<report_name>/ 下，并给用户 http://192.168.6.99:8080/reports/<report_name>/index.html 这种可访问 URL。本次报告已挂到 src/tie_robot_web/web/reports/depth_gradient_hessian_frangi_scheme_20260504/，本机验证 http://192.168.6.99:8080/reports/depth_gradient_hessian_frangi_scheme_20260504/index.html 返回 HTTP 200。

### 影响范围

- `src/tie_robot_web/web/reports/depth_gradient_hessian_frangi_scheme_20260504/index.html`
- `docs/agent_memory/current.md`

### 关键决策

- 见摘要。

### 验证证据

- `HTTP 200 from http://192.168.6.99:8080/reports/depth_gradient_hessian_frangi_scheme_20260504/index.html`

### 后续注意

- 暂无。

## 2026-05-04 22:52 - 视觉报告默认启动 0.0.0.0 网页

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求：以后每次完成视觉算法方案、效果图或流程图报告后，都要启动一个绑定 0.0.0.0 的本地静态网页服务，并给出可访问 URL；报告页应包含视觉整个流程每个步骤的效果图，而不是只给文件路径或文字结论。本次 depth_gradient + Hessian + Frangi 效果页已通过 python3 -m http.server 18080 --bind 0.0.0.0 挂载。

### 影响范围

- `.debug_frames/depth_gradient_hessian_frangi_scheme_20260504/index.html`
- `src/tie_robot_perception/tools/depth_gradient_ridge_scheme_report.py`

### 关键决策

- 见摘要。

### 验证证据

- `local health check: http://127.0.0.1:18080/index.html returned HTTP 200`

### 后续注意

- 暂无。

## 2026-05-04 22:31 - 扫描层 Surface-DP 主链接入

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04：根据用户最新明确链路，扫描层 MODE_SCAN_ONLY 算法本体从 2026-05-03 depth-only S2 主链切到 Surface-DP 主链。新增 src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py，运行态实现 combined/fused response、Hessian/Frangi、binary/skeleton 诊断、completed_surface_mask、DP 曲线族与曲线交点；manual_workspace_s2.run_manual_workspace_s2_pipeline() 优先 Surface-DP，旧 depth-only 保留为 run_manual_workspace_s2_depth_only_pipeline() fallback。固定 snapshot rebar_instance_segmentation_modalities_20260430_112028 验证 runtime Surface-DP 输出 [8,8] 线族、64 点、mean_completed_surface_score=0.984；全量实验仍推荐 04_surface_dp_curve，旧 depth-only 复刻为 867 点/[17,51]。instance_graph junction 只做诊断/补召回，不直接全量输出。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `docs/reports/scan_response_full_evaluation_2026-05-04.md`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:$PYTHONPATH python3 src/tie_robot_perception/test/test_scan_surface_dp_runtime.py; source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:$PYTHONPATH python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 fixed snapshot runtime probe -> surface_dp_curve [8`
- `8] 64 points; scan_response_full_evaluation -> recommended 04_surface_dp_curve`

### 后续注意

- 暂无。

## 2026-05-04 22:11 - 扫描响应底图全量实验与推荐方案

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04：新增离线全量实验脚本 src/tie_robot_perception/tools/scan_response_full_evaluation.py，强制读取 .debug_frames/rebar_instance_segmentation_modalities_20260430_112028，不消费 ROS 实时流。实验输出 .debug_frames/scan_response_full_evaluation_20260504/ 和 docs/reports/scan_response_full_evaluation_2026-05-04.md。结果：当前 depth-only 扫描复刻在该 snapshot 上输出 867 点、线数 [17,51]、周期 29/10 px；组合响应/Hessian/Frangi/补全面路线稳定为 64 点、[8,8]。代理指标推荐 04_surface_dp_curve，即 combined/fused_instance_response + Hessian/Frangi 脊线增强 + completed_surface_mask + DP 曲线收束；instance_graph junction 原始 128 点，适合作验证/补召回，不宜直接全量输出。

### 影响范围

- `src/tie_robot_perception/tools/scan_response_full_evaluation.py`
- `docs/reports/scan_response_full_evaluation_2026-05-04.md`
- `.debug_frames/scan_response_full_evaluation_20260504/summary.json`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m py_compile src/tie_robot_perception/tools/scan_response_full_evaluation.py; python3 src/tie_robot_perception/tools/scan_response_full_evaluation.py --snapshot-dir .debug_frames/rebar_instance_segmentation_modalities_20260430_112028 --output-dir .debug_frames/scan_response_full_evaluation_20260504 --threshold-percentile 83 --ir-display-gamma 1.95`

### 后续注意

- 暂无。

## 2026-05-04 21:59 - 扫描层视觉算法资料包与底图结论

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-04：已整理 docs/reports/scan_vision_obsidian_vault/ Obsidian 知识库并压缩为 docs/reports/scan_vision_obsidian_vault.zip。当前扫描层 MODE_SCAN_ONLY 的 PR-FPRG/S2 主底图是 self.depth_v 深度图；算法在手动工作区透视展开后的 rectified depth 上构造 depth-only 背景差分响应，IR 只作为显示和历史实验资料，不是当前扫描主链底图。当前本地 pseudo_slam_points.json 有 816 个点、规划成员 540 个、规划唯一 cell 194 个，强烈指向周期选到半周期/生成式网格过密和重复 cell 候选未抑制。

### 影响范围

- `docs/reports/scan_vision_obsidian_vault`
- `docs/reports/scan_vision_obsidian_vault.zip`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_process/data/pseudo_slam_points.json`

### 关键决策

- 见摘要。

### 验证证据

- `zip -T docs/reports/scan_vision_obsidian_vault.zip`

### 后续注意

- 暂无。

## 2026-05-03 19:31 - 索驱遥控默认绝对点动并保留相对选项

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：用户要求浏览器索驱遥控默认改回绝对移动模式。当前前端方向按钮和键盘遥控默认用当前 /cabin/cabin_data_upload 原始坐标加步距后调用 /cabin/driver/raw_move，对应 TCP 0x0012 控制字 bit0=绝对位置运动触发；索驱遥控页新增“绝对点动 / 相对点动”模式切换并持久化，相对点动仍调用 /cabin/driver/incremental_move，对应 TCP 0x0012 bit1=相对位置运动触发。

### 影响范围

- `src/tie_robot_web/frontend/src/controllers/CabinRemoteController.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/web/index.html`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `node cabinRemote*.test.mjs related tests; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_settings_panel_renders_cabin_remote_page ...; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract src.tie_robot_process.test.test_cabin_tcp_transport_contract; npm run build; git diff --check`

### 后续注意

- 暂无。

## 2026-05-03 19:31 - 扫描 S2 算法同步 38baa98 变体评分

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：用户澄清只同步算法改动，与请求/触发/发布/TF/结果图样式无关。当前扫描 S2 的 prepare_manual_workspace_s2_inputs 已按 38baa98 行为完整评分 background_depth-filled_depth 与 filled_depth-background_depth 两个 depth-only 响应变体，选择纵横周期估计总分最高者；透视展开几何优先使用 corner_world_map_frame，缺失时回退 corner_world_camera_frame 保持现有工作区文件兼容。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py;src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py;CHANGELOG.md`

### 关键决策

- 见摘要。

### 标签

- `perception`
- `pr-fprg`
- `scan-s2`
- `38baa98`

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> Ran 88 tests OK`

### 后续注意

- 暂无。

## 2026-05-03 18:15 - 当前视觉识别流程效果页

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：新增 docs/reports/current_visual_recognition_flow/index.html，基于 docs/releases/slam_v30/visual_modalities 离线样例生成当前视觉识别每个流程效果图。页面覆盖扫描识别 2026-04-22 PR-FPRG（depth-only 背景差分、纵横 profile 周期相位、透视网格反投影）和执行微调平面分割 + Hough 两个分支；生成脚本为 src/tie_robot_perception/tools/build_current_visual_recognition_flow_page.py，产出 15 张本地图片和 manifest.json。

### 影响范围

- `docs/reports/current_visual_recognition_flow/index.html; docs/reports/current_visual_recognition_flow/images/*.png; docs/reports/current_visual_recognition_flow/manifest.json; src/tie_robot_perception/tools/build_current_visual_recognition_flow_page.py; src/tie_robot_perception/test/test_current_visual_recognition_flow_report.py; CHANGELOG.md`

### 关键决策

- 视觉流程效果网页作为离线报告放在 docs/reports/current_visual_recognition_flow，不改前端请求链路、视觉算法或 src/tie_robot_web/web 构建产物。

### 标签

- `vision`
- `report`
- `pointai`
- `pr-fprg`
- `hough`

### 验证证据

- `python3 src/tie_robot_perception/tools/build_current_visual_recognition_flow_page.py => wrote 15 images; python3 src/tie_robot_perception/test/test_current_visual_recognition_flow_report.py => OK; python3 -m py_compile report generator/test => exit 0; source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:$PYTHONPATH python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py => Ran 86 tests OK; git diff --check relevant files => exit 0`

### 后续注意

- 暂无。

## 2026-05-03 18:12 - 索驱协议拒绝回包也带发送帧

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：现场 X- 点动回包 EB 90 00 00 F5 43 B3 02 是 0x0012 合法8字节运动拒绝回包，状态字按字节2~5 big-endian 为 0x0000F543，对应逆解未激活、电机未全部使能及多轴限位等原因。为避免前端显示“发送报文：未记录”，CabinDriver 在 moveToPose/moveByOffset/sendStop 的协议状态字错误路径中追加 request_command 与 request_frame；TCP 收发错误路径仍由 CabinTcpTransport 追加请求上下文。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract; python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; node test/cabinRemoteProtocolFeedback.test.mjs; git diff --check -- src/tie_robot_hw/src/driver/cabin_driver.cpp src/tie_robot_process/test/test_cabin_tcp_transport_contract.py; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process'`

### 后续注意

- 暂无。

## 2026-05-03 17:50 - 视觉扫描 S2 完全清回 2026-04-22 主链

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：用户要求不动 /pointAI/process_image request_mode=3、前端按钮、Web action 或执行层 Hough 分流，只把视觉扫描算法复刻回 2026-04-22 manual workspace S2。当前 run_manual_workspace_s2_pipeline 只走单帧 prepare_manual_workspace_s2_inputs：depth-only 背景差分响应、rectified 纵横 profile 周期/相位估计、build_workspace_s2_line_positions、projective line segments 与 inverse mapping；扫描运行路径和 pointAINode 绑定层已清除 line-family、depth+IR 组合响应、梁筋扩张过滤、稳定采样择优和 phase lock 残留。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `CHANGELOG.md`
- `docs/handoff/2026-04-23_pr_fprg_knowledge.md`

### 关键决策

- 见摘要。

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile manual_workspace_s2.py processor.py state.py`

### 后续注意

- 暂无。

## 2026-05-03 17:46 - 索驱0x0012合法拒绝回包不再误判状态包

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：现场 0x0012 相对位置运动回包 EB 90 00 00 F5 43 B3 02 是校验正确的8字节运动状态回包，状态字0x0000F543表示上位机拒绝运动并给出限位/未使能等原因。此前 CabinTcpTransport 的状态包污染过滤会把其中 00 00 F5 43 按 float 小端识别为 490.0，误当144字节状态包前缀并继续等待136字节导致 received=0/136。现增加合法运动状态回包优先判别：若8字节回包头、校验和、0x0012字节2~5状态字掩码均合法，则直接交给协议解码，不再丢弃为状态包前缀；保留真实144字节状态包前缀跳过逻辑。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract; python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; git diff --check -- src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp src/tie_robot_process/test/test_cabin_tcp_transport_contract.py; node test/cabinRemoteProtocolFeedback.test.mjs; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process'`

### 后续注意

- 暂无。

## 2026-05-03 17:30 - 视觉扫描算法复刻 2026-04-22 S2

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：用户明确要求不动视觉请求/触发链路，只把扫描视觉算法本体恢复到 2026-04-22 manual workspace S2。当前 /pointAI/process_image request_mode=3 仍是入口；run_manual_workspace_s2_pipeline 使用 depth-only 背景差分、纵横 profile 周期相位、projective line segments 和 inverse mapping；不再在扫描运行路径绑定或使用 axis row-column line-family、depth+IR 组合响应、axis_peak_families 或梁筋 ±13cm 扩张过滤。执行层 MODE_EXECUTION_REFINE/Hough 分流不变。

### 影响范围

- `CHANGELOG.md; docs/handoff/2026-04-23_pr_fprg_knowledge.md; src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py; src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py; src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 扫描视觉算法完全复刻 2026-04-22 depth-only profile period/phase + 透视网格恢复主链，视觉请求/触发链路不动。

### 标签

- `vision`
- `pointai`
- `pr-fprg`
- `scan-only`
- `2026-04-22`

### 验证证据

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:$PYTHONPATH python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py => Ran 89 tests OK; python3 -m py_compile manual_workspace_s2.py processor.py test_pointai_scan_only_pr_fprg.py => exit 0; rg old scan-runtime experimental symbols in manual_workspace_s2.py processor.py => no matches`

### 后续注意

- 暂无。

## 2026-05-03 17:17 - 索驱0x0012合法拒绝回包不再误判状态包

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：现场回包 EB 90 00 00 F5 43 B3 02 是 0x0012 合法8字节运动回包，校验0x02B3正确，状态字0x0000F543表示逆解未激活、电机未全部使能以及多轴限位等拒绝原因。此前 CabinTcpTransport 的状态包污染过滤把其字节2~5按float小端看成490.0，误判为144字节状态包前缀并继续等待136字节尾部，导致前端显示 received=0/136。现在 transport 会先检查8字节是否为合法运动状态回包（包头、校验、状态字位宽均符合），合法则直接返回给协议解码；只有不合法且像状态包前缀时才丢弃136字节尾部。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract; python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; git diff --check -- src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp src/tie_robot_process/test/test_cabin_tcp_transport_contract.py; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process'`

### 后续注意

- 暂无。

## 2026-05-03 15:47 - 索驱0x0012状态字按字典收口

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：索驱 TCP位置运动启动 0x0012 的回包状态字统一按用户给定字典解释：返回包字节2~5作为32位状态字，bit0~16映射为逆解未激活、电机未全部使能、设备运动中、报警、速度错误以及 X/Y/Z/A/B/C 正负限位，其中 bit16=C超负限位。tie_robot_hw 的 0x0012 decodeStatus 现在优先按字节2~5的32位字典口径解码，旧 suoqu 兼容层的 pending/status decode 也从 uint16_t 改为 uint32_t，避免把字节4~5误解成16位状态并漏掉 C负限位。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_protocol.cpp`
- `src/tie_robot_process/include/tie_robot_process/suoqu/cabin_transport.hpp`
- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_cabin_protocol_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract; git diff --check -- relevant files; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process'; node test/cabinRemoteProtocolFeedback.test.mjs`

### 后续注意

- 暂无。

## 2026-05-03 15:18 - 手动视觉识别完全独立于工作区提交

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：前端“触发视觉识别”按钮现在完全独立于“确认工作区域”：按钮启用只依赖 ROS ready 和 /pointAI/process_image 可用；点击时不要求 saved workspace，不检查 pending workspace submission，也不会 clearPendingWorkspaceQuadSubmission，因此不会吞掉正在等待 pointAI 保存确认的工作区提交。保存确认后的自动 PR-FPRG 触发仍照常执行。

### 影响范围

- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/taskActionPrFprgRecognition.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node test/taskActionPrFprgRecognition.test.mjs; find test -name '*.mjs' -print0 | sort -z | xargs -0 -n 1 node; npm run build`

### 后续注意

- 暂无。

## 2026-05-03 15:03 - 触发视觉识别按钮不依赖工作区

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：前端控制面板“触发视觉识别”按钮现在只依赖 ROS 已连接且 /pointAI/process_image 可用，不再要求已经提交/保存工作区，也不再因工作区正在保存而禁止点击。点击后仍走 MODE_SCAN_ONLY=3 的 4月22日 PR-FPRG 扫描触发链；“确认工作区域”保存确认后的自动触发逻辑保持存在。单点绑扎等需要工作区的动作仍保留 saved workspace 限制。

### 影响范围

- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/test/taskActionPrFprgRecognition.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node test/taskActionPrFprgRecognition.test.mjs; find test -name '*.mjs' -print0 | sort -z | xargs -0 -n 1 node; npm run build`

### 后续注意

- 暂无。

## 2026-05-03 14:41 - 前端视觉触发固定走4月22日PR-FPRG

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：前端用户主动触发视觉识别已彻底收口到 /pointAI/process_image request_mode=3，即 MODE_SCAN_ONLY 的 2026-04-22 PR-FPRG 扫描触发链；控制面板“触发视觉识别”和设置页“触发视觉服务”不再走 /perception/lashing/recognize_once，也不再允许从 UI 选择自适应高度、绑扎检查或执行微调模式。执行层 start_global_work 的 MODE_EXECUTION_REFINE/Hough 分流保持由后端流程控制，不在前端直接识别按钮里暴露。

### 影响范围

- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/config/visualRecognitionMode.js`
- `src/tie_robot_web/frontend/test/taskActionPrFprgRecognition.test.mjs`
- `src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node test/taskActionPrFprgRecognition.test.mjs; node test/visualDebugSettings.test.mjs; for test_file in test/*.test.mjs; do node "$test_file" || exit 1; done; npm run build`

### 后续注意

- 暂无。

## 2026-05-03 14:41 - 本机 nvm Node 25 已安装并支持 codex-provider-sync

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：按用户要求通过 nvm 安装链路更新 nvm 到 0.40.3，并安装 Node v25.9.0 / npm 11.12.1，nvm default 指向 25。交互式 bash 会自动使用 ~/.nvm/versions/node/v25.9.0/bin/node；非交互脚本若需要 Node 25，应先 source ~/.nvm/nvm.sh。Node 25 已验证支持 node:sqlite，~/codex-provider-sync/src/cli.js 可直接运行。安装后又用真实 codex-provider-sync 执行 sync，修复了 state_5.sqlite 中剩余 4 个 has_user_event 标志；新备份为 ~/.codex/backups_state/provider-sync/20260503T063914973Z。

### 影响范围

- `~/.nvm; ~/.bashrc; ~/codex-provider-sync; ~/.codex/state_5.sqlite*; ~/.codex/backups_state/provider-sync/20260503T063914973Z`

### 关键决策

- 见摘要。

### 验证证据

- `source ~/.nvm/nvm.sh 后 nvm --version=0.40.3、node -v=v25.9.0、npm -v=11.12.1；bash -ic node -v 输出 v25.9.0；node -e import('node:sqlite') 输出 node:sqlite ok；codex-provider-sync status 显示 rollout sessions=chaomeng-api 61、archived_sessions=chaomeng-api 45，SQLite sessions=chaomeng-api 61、archived_sessions=chaomeng-api 41，且不再提示 user-event flags needing repair。`

### 后续注意

- 暂无。

## 2026-05-03 14:40 - 前端确认工作区域自动触发PR-FPRG

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：前端控制面板按钮 submitQuad 文案从“提交四边形”改为“确认工作区域”；工作区四边形被 pointAI 保存确认后，TaskActionController 会自动触发前端视觉识别链路。该链路使用 /pointAI/process_image 的 MODE_SCAN_ONLY=3（4月22日 PR-FPRG扫描输出），不是执行微调 Hough；“触发视觉识别”按钮仍保留，用作已保存工作区的手动重跑入口。

### 影响范围

- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/config/visualRecognitionMode.js`
- `src/tie_robot_web/frontend/test/taskActionController.test.mjs`
- `src/tie_robot_web/frontend/test/taskActionPrFprgRecognition.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node test/taskActionController.test.mjs; node test/taskActionPrFprgRecognition.test.mjs; find test -name '*.mjs' -print0 | sort -z | xargs -0 -n 1 node; npm run build`

### 后续注意

- 暂无。

## 2026-05-03 14:28 - 索驱遥控报文三行反馈

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：前端索驱遥控失败反馈不再原样显示驱动层长 detail；新增 cabinRemoteProtocolFeedback 格式化，把 request_frame 显示为发送报文，把 response_frame 或 received 显示为返回报文，并将原因/status_word/socket 接收失败转成返回含义三行。绝对位姿移动失败也复用同一格式化。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/cabinRemoteProtocolFeedback.js`
- `src/tie_robot_web/frontend/src/controllers/CabinRemoteController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/test/cabinRemoteProtocolFeedback.test.mjs`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node test/cabinRemoteProtocolFeedback.test.mjs; node test/cabinRemoteController.test.mjs; node test/cabinRemoteButtonSingleFire.test.mjs; node test/cabinRemoteKeyboard.test.mjs; node test/cabinRemoteOperationState.test.mjs; npm run build`

### 后续注意

- 暂无。

## 2026-05-03 14:19 - Header索驱胶囊不再联动视觉层

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：前端 header 的索驱状态胶囊异常/未运行态由‘启动’改为‘重启’，点击只执行 restartCabinDriver；start/stopCabinSubsystem 与 start/stopModuanSubsystem 只操作各自驱动，不再夹带 start/stopAlgorithmStack。此前关闭索驱会先 stopAlgorithmStack，导致视觉层诊断随算法层一起消失。后续维护三层胶囊时保持索驱、末端、视觉控制互不联动；视觉子系统仍独立管理相机驱动和算法层。

### 影响范围

- `src/tie_robot_web/frontend/src/config/systemControlCatalog.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/test/systemControlCatalog.test.mjs`
- `src/tie_robot_bringup/systemd/tie-robot-backend-control.sudoers.in`

### 关键决策

- 见摘要。

### 验证证据

- `node test/systemControlCatalog.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_status_capsule_tracks_only_connection_and_hardware src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_system_control_http_endpoints_cover_start_and_restart_actions src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_ros_backend_is_systemd_managed_from_frontend src.tie_robot_bringup.test.test_architecture_cleanup.TieRobotArchitectureCleanupTest.test_driver_nodes_publish_standard_diagnostics_and_support_independent_start_stop; npm run build`

### 后续注意

- 暂无。

## 2026-05-03 14:17 - 本机 Codex provider metadata 已同步到 chaomeng-api

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：按 https://github.com/Dailin521/codex-provider-sync 的源码逻辑恢复本机 Codex 会话可见性。仓库已克隆到 ~/codex-provider-sync；本机 Node v20.20.2 缺 node:sqlite，无法直接运行该 CLI（需 Node >=24），因此用等价 Python 流程执行 status/backup/sync。当前 ~/.codex/config.toml 根级 model_provider=chaomeng-api；已将 ~/.codex/sessions 与 ~/.codex/archived_sessions 中 95 个 rollout 首行 model_provider 从 openai 同步为 chaomeng-api，并将 state_5.sqlite 中 91 行 provider 与 98 个 has_user_event 标志修复。备份在 ~/.codex/backups_state/provider-sync/20260503T061305243+0000。

### 影响范围

- `~/codex-provider-sync; ~/.codex/sessions; ~/.codex/archived_sessions; ~/.codex/state_5.sqlite*; ~/.codex/backups_state/provider-sync/20260503T061305243+0000`

### 关键决策

- 见摘要。

### 验证证据

- `只读复扫：rollout sessions=chaomeng-api 61，archived_sessions=chaomeng-api 45；SQLite sessions=chaomeng-api 61，archived_sessions=chaomeng-api 41；provider mismatch=0；first_user_message 但 has_user_event=0 为 0；python3 scripts/codex_session_guard.py scan --threshold-mb 100 --skip-open：no oversized active Codex sessions found；python3 scripts/agent_memory.py check：agent memory contract ok；codex debug prompt-input ping 命中 AGENTS/superpowers 注入。`

### 后续注意

- 暂无。

## 2026-05-03 14:08 - 旧20260403索驱动作链口径

<!-- AGENT-MEMORY: entry -->

### 摘要

- 20260403旧工作区的 api.launch 实际包含 api.launch + run.launch，run.launch 拉起的是 chassis_ctrl/suoquNode、moduanNode、pointAINode；suoquNode_show.cpp 虽有 0x0012 bit1 相对姿态控制样例，但 CMake 未 add_executable、devel/lib/chassis_ctrl 也无 suoquNode_show，不能当成旧展示运行主链。旧前端/调试按钮发 /web/cabin/cabin_move_debug，topictransNode 转 /cabin/single_move，suoquNode::cabin_single_move 组 0x0012 control_word=0x01 绝对位姿帧。旧状态轮询与运动共用同一 sockfd/socket_mutex，但 Frame_Generate 单次 recv(Rlen)，状态包 144 字节读边界不稳，理论上也会残留污染后续 8 字节运动回包。

### 影响范围

- `/home/hyq-/lashingrobotROS/src/chassis_ctrl/launch/api.launch`
- `/home/hyq-/lashingrobotROS/src/chassis_ctrl/launch/run.launch`
- `/home/hyq-/lashingrobotROS/src/chassis_ctrl/src/topics_transfer.cpp`
- `/home/hyq-/lashingrobotROS/src/chassis_ctrl/src/suoquNode.cpp`
- `/home/hyq-/lashingrobotROS/src/chassis_ctrl/src/suoquNode_show.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `rg/sed/nl 定向核对旧 launch、CMake、topictransNode、suoquNode 帧生成与读写路径`

### 后续注意

- 暂无。

## 2026-05-03 13:57 - cockpit-tools 本机项目已清理

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：按用户要求，已停止 ~/cockpit-tools 的 Vite preview 后台进程，删除 /home/hyq-/cockpit-tools 目录以及 /tmp/cockpit-tools-preview.log、/tmp/cockpit-tools-setsid.log。端口 1420 已不再监听。Rust 工具链和 npm/cargo 全局缓存未删除，避免影响其他项目。

### 影响范围

- `~/cockpit-tools`
- `/tmp/cockpit-tools-preview.log`
- `/tmp/cockpit-tools-setsid.log`

### 关键决策

- 见摘要。

### 验证证据

- `test ! -e /home/hyq-/cockpit-tools; ss -ltnp | rg ':1420' returned no listener`

### 后续注意

- 暂无。

## 2026-05-03 13:47 - 索驱运动回包跳过状态包残留

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：现场确认索驱TCP相对位置运动错帧 raw_status_le32=0x43213333 的 33 33 21 43 与 /cabin/cabin_data_upload 的 cabin_state_X=161.2 完全一致，说明运动命令读到了状态查询包前缀。CabinTcpTransport 现在会在 0x0011/0x0012/0x0013 运动/停止命令发送前排空 socket 旧输入；若发送后先读到 144 字节状态包前缀，会读掉剩余 136 字节并继续等待真正 8 字节运动状态回包，避免把状态坐标误判为运动状态。

### 影响范围

- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_tcp_transport.hpp`
- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_cabin_tcp_transport_contract.py; python3 src/tie_robot_process/test/test_cabin_protocol_contract.py; python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_process'; sudo systemctl restart tie-robot-driver-suoqu.service; rostopic echo -n 1 /cabin/cabin_data_upload`

### 后续注意

- 暂无。

## 2026-05-03 13:34 - 本机 Codex 启用 superpowers-zh 使用纪律

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：已在用户级 ~/.codex/config.toml 顶层写入 developer_instructions，要求会话开始或新任务前先使用 superpowers-zh:using-superpowers，按任务匹配读取 ~/.codex/superpowers/skills/<skill>/SKILL.md；中文沟通默认中文回复，中文代码审查/文档/提交/国内 Git 平台优先对应中文 skill；用户明确指令和项目级 AGENTS.md 高于 superpowers-zh；同时显式设置 [features].multi_agent=true 以支持子 agent/并行工作相关 skill。

### 影响范围

- `~/.codex/config.toml`
- `~/.agents/skills/superpowers`

### 关键决策

- 见摘要。

### 标签

- `codex`
- `superpowers`
- `local-config`

### 验证证据

- `codex debug prompt-input "ping" | rg "本机 Codex 使用 superpowers-zh|superpowers-zh:using-superpowers"; codex features list | rg "multi_agent\\s+"`

### 后续注意

- 暂无。

## 2026-05-03 12:56 - cockpit-tools 本机启动限制

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：用户要求在 ~ 拉取并编译启动 https://gh-proxy.com/github.com/jlcodes99/cockpit-tools.git。已克隆到 ~/cockpit-tools，npm ci 与 npm run build 通过，并用 npm run preview -- --host 127.0.0.1 --port 1420 启动前端预览。完整 Tauri 2 桌面构建在本机 Ubuntu 20.04.6 被系统库阻断：glib-sys 要求 glib-2.0 >= 2.70，而本机 apt 候选/已安装均为 2.64.6；Tauri 依赖还硬要求 webkit2gtk-4.1/libsoup-3.0，focal 源只提供 webkit2gtk-4.0/libsoup2.4。当前机器 DISPLAY 为空，无法直接启动可交互桌面窗口。后续若要正常使用桌面功能，优先换 Ubuntu 22.04+/24.04 或 macOS/Windows 官方支持环境，不建议在机器人主机上硬升级 GLib/WebKit 核心库。

### 影响范围

- `~/cockpit-tools`
- `README.md`
- `package.json`
- `src-tauri/Cargo.toml`

### 关键决策

- 见摘要。

### 验证证据

- `npm run build passed; cargo check -p cockpit-tools failed at glib-2.0 >= 2.70 on Ubuntu 20.04; curl -I http://127.0.0.1:1420 returned HTTP 200`

### 后续注意

- 暂无。

## 2026-05-03 12:48 - 索驱0x0012浮点形态错帧识别

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-05-03：索驱TCP相对位置运动遇到 response_frame=[EB 90 33 33 21 43 45 02] / raw_status_le32=0x43213333 时，字节33 33 21 43按float小端是161.2，包含协议0x0012状态字未定义高位，不应继续解成逆解未激活/限位原因。驱动层现将这种运动回包标记为 protocol_response_desynchronized，断开当前TCP连接清理残留回包；为避免相对运动重复执行，不自动重发，需确认设备状态后重试。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_protocol.cpp`
- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_process/test/test_cabin_protocol_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_process/test/test_cabin_protocol_contract.py; python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; g++ -std=c++14 -I src/tie_robot_hw/include -c src/tie_robot_hw/src/driver/cabin_driver.cpp -o /tmp/cabin_driver.o; g++ -std=c++14 -I src/tie_robot_hw/include -c src/tie_robot_hw/src/driver/cabin_protocol.cpp -o /tmp/cabin_protocol.o; g++ -std=c++14 -I src/tie_robot_hw/include -c src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp -o /tmp/cabin_tcp_transport.o`

### 后续注意

- 暂无。

## 2026-04-30 23:14 - 相机-TCP外参按钮热更新 fallback

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-30：前端相机-TCP外参应用不应只依赖 /web/tf/set_gripper_tf_calibration service。现场 service 未注册、旧 gripper_tf_broadcaster 未重启或 rosbridge 调用超时时，前端会自动改发新的 /web/tf/set_camera_tcp_extrinsic Pose topic，并先按本次 applied translation_mm 更新 3D gripper_frame/TCP 显示和工作区投影，等待后续 /tf 覆盖。后续修改不要把前端外参按钮改回 /web/tf/set_offset；旧 /web/tf/set_offset 只在后端保留兼容订阅。

### 影响范围

- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/config/legacyCommandCatalog.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs`
- `src/tie_robot_perception/scripts/gripper_tf_broadcaster.py`
- `src/tie_robot_perception/test/test_gripper_tf_broadcaster.py`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs; bash -lc 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src/tie_robot_perception/test/test_gripper_tf_broadcaster.py'; for test_file in src/tie_robot_web/frontend/test/*.mjs; do node "$test_file" || exit 1; done; npm run build; rosnode kill /gripper_tf_broadcaster 后 rostopic info /web/tf/set_camera_tcp_extrinsic 显示 /gripper_tf_broadcaster subscriber`

### 后续注意

- 暂无。

## 2026-04-30 22:41 - 前端终端改回默认 PTY

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-30：按用户要求，前端“终端”卡片不再用 tmux 管理。workspace_picker_web_server.py 的 TerminalSession 直接创建 bash PTY，会话标签来自默认 shell（bash、bash 2...），页面刷新可重连仍在服务进程内存中的会话；关闭/exit 会结束对应 shell 并从配置列表清理。不要恢复 TERMINAL_TMUX_*、tmux attach/new/kill 逻辑。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py;src/tie_robot_web/frontend/src/controllers/TerminalController.js;src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `terminal`
- `pty`

### 验证证据

- `py_compile；终端相关 unittest；npm run build；systemd 重启；HTTP 创建 + websocket echo/exit 实测通过`

### 后续注意

- 暂无。

## 2026-04-30 22:11 - 视觉识别位姿与执行层方案分流

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确当前运行口径：移动到固定识别位姿后触发 2026-04-22 PR-FPRG 拓扑恢复方案生成 pseudo_slam_points/bind_path；开始执行后 live_visual 每到区域调用 MODE_EXECUTION_REFINE，视觉层走独立 execution_refine_hough.py，即基于 /Scepter/worldCoord/world_coord 平面分割结果做 Hough，输出坐标仍取 /Scepter/worldCoord/raw_world_coord。旧 legacy_ransac_hough_pointai 只作追溯/参考，不直接导入 matrix_preprocess.pre_img，也不能回接扫描建图主链。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `docs/archive/vision_research_runtime_scheme_2026-04-30.md`
- `docs/archive/pr_fprg_previous_schemes_2026-04-30.md`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src:/home/hyq-/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/lashingrobotROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; PYTHONPATH=... python3 -m py_compile execution_refine_hough.py process_image_service.py processor.py ros_interfaces.py`

### 后续注意

- 暂无。

## 2026-04-30 21:59 - 虎口TCP坐标轴方向

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场确认虎口/TCP gripper_frame 坐标系方向：z+ 朝地面，y+ 与 map.y+ 同向，x+ 与 map.x- 同向。当前通过 Scepter_depth_frame->gripper_frame 的 rotation_rpy yaw=pi 实现；base_link->Scepter_depth_frame 已是 roll=pi，因此组合后 gripper_frame 相对 map/base_link 是合法 180 度旋转，不是 TF 镜像置反。

### 影响范围

- `src/tie_robot_perception/config/gripper_tf.yaml`
- `src/tie_robot_perception/test/test_gripper_tf_broadcaster.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_gripper_tf_broadcaster.py && python3 src/tie_robot_perception/test/test_robot_tf_broadcaster.py; rosnode kill /gripper_tf_broadcaster respawn; tf_echo Scepter_depth_frame gripper_frame shows yaw=180deg; tf_echo base_link gripper_frame shows RPY -180`
- `0`
- `-180`

### 后续注意

- 暂无。

## 2026-04-30 21:51 - TCP工具底面中心锚点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场确认虎口 TCP 工具模型原点是 TCP 长方体最底面的中心点。前端橙色 TCP 长方体应作为 gripper_frame 的子几何显示，并让最终显示出来的最底面中心对齐 gripper_frame；不要把橙色 TCP 方框几何中心对齐 gripper_frame。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_viewer_app_no_longer_renders_topics_tf_problems_panels src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_base_link_splits_machine_pose_from_downward_camera_frame; npm run build; git diff --check`

### 后续注意

- 暂无。

## 2026-04-30 21:37 - base_link底面中心锚点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场确认 base_link 原点是整个机器立方体最底面的中心点，不是质心/几何中心。前端 3D 机器盒体和 URDF 视觉盒体应作为 base_link 的子几何向 z+ 抬半个高度；map->base_link 仍由索驱上位机当前坐标发布。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_description/URDF/model.urdf`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_viewer_app_no_longer_renders_topics_tf_problems_panels src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_base_link_splits_machine_pose_from_downward_camera_frame; python3 src/tie_robot_perception/test/test_robot_tf_broadcaster.py; npm run build`

### 后续注意

- 暂无。

## 2026-04-30 21:27 - 相机相对base_link高度

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场确认相机坐标系原点位于 base_link 的 z+ 方向 460mm；当前 base_link -> Scepter_depth_frame 使用 x=0,y=0,z=460mm，roll=pi,pitch=0,yaw=0 的相机朝地近似。后续如测得横向偏置，再补入 robot_home_tf.yaml 的 base_to_camera_mm。

### 影响范围

- `src/tie_robot_perception/config/robot_home_tf.yaml`
- `src/tie_robot_perception/scripts/robot_tf_broadcaster.py`
- `src/tie_robot_perception/test/test_robot_tf_broadcaster.py`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_robot_tf_broadcaster.py; git diff --check`

### 后续注意

- 暂无。

## 2026-04-30 21:22 - 索驱遥控相对点动改用TCP位置相对触发

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求遥控里的相对位置点动使用 TCP位置运动启动 0x0012 的 bit1=1（控制字 0x0002，TCP相对位置运动触发）。实现后上层服务名 /cabin/driver/incremental_move 暂保持兼容，但 CabinDriver::moveByOffset() 下发 CabinProtocol::buildRelativeMoveFrame(command)，并按 0x0012 解码回包；0x0011 TCP增量运动帧只保留为协议 helper/诊断对象，不再作为遥控相对点动实现。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_driver.cpp; src/tie_robot_process/src/suoqu/cabin_transport.cpp; src/tie_robot_process/src/suoquNode.cpp; src/tie_robot_process/test/test_motion_chain_signal_guard.py; CHANGELOG.md`

### 关键决策

- 见摘要。

### 标签

- `suoqu`
- `cabin_protocol`
- `remote_jog`

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard.MotionChainSignalGuardTest.test_cabin_driver_uses_tcp_position_relative_frame_for_remote_step_buttons src.tie_robot_process.test.test_cabin_protocol_contract.CabinProtocolContractTest.test_relative_move_frame_uses_tcp_position_relative_trigger -> OK; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract -> 16 tests OK; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;tie_robot_hw;tie_robot_process' -> OK; restored whitelist show_legacy_driver_bridge;tie_robot_bringup -> OK; sudo -n systemctl restart tie-robot-driver-suoqu.service -> active; rosnode ping /suoqu_driver_node and rosservice info /cabin/driver/incremental_move -> OK; git diff --check relevant files -> OK.`

### 后续注意

- 暂无。

## 2026-04-30 21:10 - Codex摘要替身保留历史标题锚点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求摘要替身不要改变之前会话标题。codex_session_guard.py 的 summarize 现在保留原始 session_meta 关键字段，并在摘要 assistant 消息前插入首条真实用户请求作为标题锚点；会跳过 AGENTS/环境注入类启动消息。新增 repair-summaries 子命令，可从 ~/.codex/archived_sessions/oversized 原文修复已有 summary_replacement；已对 2026-04-11、04-22、04-25、04-29 四个已有摘要替身执行 repair-summaries --apply。

### 影响范围

- `scripts/codex_session_guard.py`
- `src/tie_robot_bringup/test/test_codex_session_guard.py`
- `AGENTS.md`
- `CHANGELOG.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/organism.md`
- `scripts/agent_memory.py`

### 关键决策

- 摘要替身只瘦身正文体量，不应让历史列表标题变成 compacted/summary 文案；标题锚点来自原文第一条真实用户请求。

### 标签

- `codex-session`
- `agent-memory`
- `long-context`
- `title-anchor`

### 验证证据

- `python3 -m unittest src.tie_robot_bringup.test.test_codex_session_guard src.tie_robot_bringup.test.test_agent_memory_contract -v; python3 -m py_compile scripts/codex_session_guard.py scripts/agent_memory.py; bash -n scripts/install_codex_session_summary_timer.sh scripts/install_codex_session_guard_timer.sh; python3 scripts/codex_session_guard.py repair-summaries --apply`

### 后续注意

- repair-summaries dry-run 仍会列出可重写的摘要替身；这是可重复刷新，不表示标题锚点缺失。

## 2026-04-30 21:04 - 演示模式改为轻量 rosbridge

<!-- AGENT-MEMORY: entry -->

### 摘要

- 旧 20260403 演示链不再复用当前 tie-robot-rosbridge.service；进入演示时停止当前完整 rosbridge、backend、三个 driver 和旧转义层，启动 tie-robot-demo-rosbridge.service（仅 rosbridge_websocket + rosapi，无当前 tf_stack/api.launch），再启动旧工作目录 roslaunch chassis_ctrl api.launch。demo rosbridge 用 topics_glob 白名单保留 /pointAI/result_image，并只放行 Scepter compressed 图像，避免旧前端继续订阅 raw 大流量图像。

### 影响范围

- `src/tie_robot_bringup/launch/demo_rosbridge_light.launch`
- `src/tie_robot_bringup/systemd/tie-robot-demo-rosbridge.service.in`
- `src/tie_robot_bringup/systemd/tie-robot-demo-show-full.service.in`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `README.md`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_demo_mode_uses_old_show_full_without_translation_layer src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_demo_mode_toggle_stops_current_services_and_restores_without_bridge; source devel/setup.bash && roslaunch --nodes tie_robot_bringup demo_rosbridge_light.launch; live systemctl verified demo rosbridge/show_full active and current rosbridge/backend/drivers inactive`

### 后续注意

- 暂无。

## 2026-04-30 20:18 - 旧 show_full 路径点 JSON 收回旧工作目录

<!-- AGENT-MEMORY: entry -->

### 摘要

- 旧 20260403 演示链的 chassis_ctrl/suoquNode 和 topictransNode 曾硬编码读取/清理 /home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/path_points.json；当前主仓已无 src/chassis_ctrl，导致 /cabin/start_work 抛出‘无法打开路径点JSON文件’。已在旧工作目录源码改为使用 /home/hyq-/lashingrobotROS/src/chassis_ctrl/data，并重编旧工作目录。

### 影响范围

- `/home/hyq-/lashingrobotROS/src/chassis_ctrl/src/suoquNode.cpp`
- `/home/hyq-/lashingrobotROS/src/chassis_ctrl/src/topics_transfer.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `bad-path grep red/green; old workspace catkin_make exit 0; tie-robot-demo-show-full.service active; rosservice info /cabin/start_work shows chassis_ctrl/MotionControl from /suoquNode`

### 后续注意

- 暂无。

## 2026-04-30 20:03 - 新前端演示模式直跑旧 show_full

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户最新口径是不再使用 show_legacy_driver_bridge/旧展示转义层。新前端 header 增加演示模式按钮：红色表示普通模式，点击后保留当前 tie-robot-rosbridge.service 和 5173 旧前端，停止当前 backend、三个 driver service 和旧 shared-driver-stack 守护，启动按需 unit tie-robot-demo-show-full.service（旧 20260403 工作目录 roslaunch chassis_ctrl api.launch）；再次点击停止 show_full，清理旧 ROS 残留节点/进程，再恢复当前 rosbridge、driver、backend。旧 show_legacy_driver_bridge 包与 shared-driver-stack 未再保留为工作区文件，机器上已禁用/停止旧 tie-robot-show-legacy-shared-driver-stack.service。

### 影响范围

- `src/tie_robot_web/frontend/src/config/systemControlCatalog.js`
- `src/tie_robot_web/frontend/src/controllers/SystemControlController.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_bringup/systemd/tie-robot-demo-show-full.service.in`
- `src/tie_robot_bringup/scripts/install_demo_mode_service.sh`
- `src/tie_robot_bringup/systemd/tie-robot-backend-control.sudoers.in`
- `README.md`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `npm run build; selected unittest demo/system-control tests OK; roslaunch --nodes chassis_ctrl api.launch OK with explicit legacy+Scepter env; actual POST /api/system/toggle_demo_mode enter/exit OK; final systemd status: demo inactive`
- `shared-driver-stack inactive`
- `rosbridge/backend/driver services active`

### 后续注意

- 暂无。

## 2026-04-30 19:35 - 旧展示适配层只共享当前底层

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确口径：新旧工作目录运行时不各起一套底层；统一使用当前工作目录的 ROS master、tie-robot-rosbridge.service 和三个 driver service。创建 show_legacy_driver_bridge 包的目的，是让旧 20260403 工作区算法和旧前端适配当前新工作目录驱动层。旧前端继续可用，新前端继续可用；旧逻辑走 /show_legacy/* 和旧算法 runner，新逻辑走当前 /cabin/*、/moduan/*、/pointAI/*，底层共享但命名空间隔离。已新增并安装 tie-robot-show-legacy-shared-driver-stack.service，正式启动 show_legacy_shared_driver_stack.launch；show-legacy-rosbridge.service 必须保持 disabled/inactive，避免抢 9090 与 /rosbridge_websocket。

### 影响范围

- `README.md`
- `CHANGELOG.md`
- `src/tie_robot_bringup/systemd/tie-robot-show-legacy-shared-driver-stack.service.in`
- `src/tie_robot_bringup/scripts/install_show_legacy_shared_driver_stack_service.sh`
- `src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py; git diff --check; systemctl is-active ...; systemctl is-enabled ...; rosnode info /rosbridge_websocket; rosservice list | rg 'show_legacy|driver'`

### 后续注意

- 暂无。

## 2026-04-30 19:31 - 旧展示链改走 shared driver stack 守护

<!-- AGENT-MEMORY: entry -->

### 摘要

- 合并新旧工作区运行时，不再启动旧工作区直连 show-legacy-rosbridge.service，因为它会和当前 tie-robot-rosbridge.service 抢 9090 与 /rosbridge_websocket 节点名，导致 rosbridge XML-RPC 不通和高 CPU。旧展示前端继续连接当前 9090；旧链路转接只通过 tie_robot_bringup show_legacy_shared_driver_stack.launch 拉起 /show_legacy/* 服务、show_legacy_pointAI 代理和旧工作区 pointAI runner。本机会话已禁用 show-legacy-rosbridge.service，并以 transient systemd unit tie-robot-show-legacy-shared-driver-stack.service 启动 shared stack。

### 影响范围

- `src/tie_robot_bringup/launch/show_legacy_shared_driver_stack.launch`
- `/etc/systemd/system/show-legacy-rosbridge.service`

### 关键决策

- 见摘要。

### 验证证据

- `systemctl --failed; systemctl list-units 'tie-robot-*' 'show-legacy-*' 'foxglove-bridge.service'; rosnode info /rosbridge_websocket; rosservice list | rg 'show_legacy|driver|pointAI'`

### 后续注意

- 暂无。

## 2026-04-30 19:26 - Codex会话压缩改为复制原文不移动文件

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户进一步校正：不要把大会话文件移动到归档；正确语义是原始内容归档但文件不要移动。codex_session_guard.py 的 summarize 已改为先用 copy2 把原始完整 JSONL 内容复制到 ~/.codex/archived_sessions/oversized，再把 ~/.codex/sessions 原路径内容改写为摘要替身 JSONL；不再通过 shutil.move 搬走会话文件。timer 已重装，描述为 copy original content and leave summary JSONL，ExecStart 仍使用 summarize --threshold-mb 100 --min-age-minutes 10 --skip-open --apply。

### 影响范围

- `scripts/codex_session_guard.py`
- `scripts/install_codex_session_summary_timer.sh`
- `scripts/install_codex_session_guard_timer.sh`
- `src/tie_robot_bringup/test/test_codex_session_guard.py`
- `AGENTS.md`
- `CHANGELOG.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/organism.md`
- `scripts/agent_memory.py`

### 关键决策

- summarize 子命令不得移动 ~/.codex/sessions 里的会话文件；只复制原始内容到归档，再在原路径写摘要替身。archive 子命令仍是手工救急移动工具，自动化不启用。

### 标签

- `codex-session`
- `agent-memory`
- `long-context`

### 验证证据

- `python3 -m unittest src.tie_robot_bringup.test.test_codex_session_guard src.tie_robot_bringup.test.test_agent_memory_contract -v; python3 -m py_compile scripts/codex_session_guard.py scripts/agent_memory.py; bash -n scripts/install_codex_session_summary_timer.sh scripts/install_codex_session_guard_timer.sh; systemctl --user cat tie-codex-session-summary.service tie-codex-session-summary.timer; python3 scripts/codex_session_guard.py scan --threshold-mb 100`

### 后续注意

- 已有一次早先按 move 处理的 201MB 旧会话仍保持现状：原路径是摘要替身，原始内容在 archived_sessions；后续新压缩不再 move 文件。

## 2026-04-30 19:10 - 系统级 show-legacy-rosbridge 服务

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户执行 sudo systemctl restart show-legacy-rosbridge.service 时找不到 unit，因为先前只创建了用户级服务。已改为系统级 /etc/systemd/system/show-legacy-rosbridge.service，User=hyq- 直接运行旧 20260403 工作区 rosbridge_server rosbridge_websocket.launch address:=0.0.0.0 port:=9090，Restart=always，并停止/禁用用户级同名服务避免重复。验证 sudo/systemctl 状态 active，9090 和 roscore 11311 监听。

### 影响范围

- 未指定。

### 关键决策

- 见摘要。

### 标签

- `legacy`
- `rosbridge`
- `systemd`

### 验证证据

- 未记录验证命令。

### 后续注意

- 暂无。

## 2026-04-30 19:06 - 旧展示链 rosbridge 独立守护

<!-- AGENT-MEMORY: entry -->

### 摘要

- 新增并启用用户级 systemd 服务 ~/.config/systemd/user/show-legacy-rosbridge.service，显式加载旧 20260403 工作区与 ScepterSDK 环境，启动 rosbridge_server rosbridge_websocket.launch address:=0.0.0.0 port:=9090，Restart=always。验证 show-legacy-rosbridge.service active，9090 监听，rosapi/rosbridge_websocket 已运行。

### 影响范围

- 未指定。

### 关键决策

- 见摘要。

### 标签

- `legacy`
- `rosbridge`
- `systemd`

### 验证证据

- 未记录验证命令。

### 后续注意

- 暂无。

## 2026-04-30 19:00 - 旧 20260403 展示工作区接管 ROS 环境

<!-- AGENT-MEMORY: entry -->

### 摘要

- 为完全使用旧展示工作区 /home/hyq-/lashingrobotROS，已停止当前工程 systemd 驱动/后端/rosbridge/8080 前端服务；重建旧工作区 build/devel 并修复其 CMake 依赖顺序和 libsimulated_annealing 路径；~/.bashrc 末尾加入 legacy lashing robot workspace 块，使 chassis_ctrl/fast_image_solve 优先解析到 20260403 旧工作区，并显式加入 /home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS 以支持 api.launch 的 ScepterROS。旧工作区 start.sh/start_.sh/restart.sh/topic_tranfer.sh 同步使用该环境。验证 rospack find chassis_ctrl/fast_image_solve 指向旧工作区，rospack find ScepterROS 指向 ScepterSDK，roslaunch --nodes chassis_ctrl run.launch/api.launch/suoquAndmoduan.launch 可解析节点。

### 影响范围

- 未指定。

### 关键决策

- 见摘要。

### 标签

- `legacy`
- `ros`
- `driver`

### 验证证据

- 未记录验证命令。

### 后续注意

- 暂无。

## 2026-04-30 18:45 - 旧展示线模动作链恢复9槽位语义

<!-- AGENT-MEMORY: entry -->

### 摘要

- 排查旧 chassis_ctrl/src/moduanNode.cpp 后确认：旧 moduan_bind_service 与 moduan_move_service 都是 inputAllPoints 写 PLC 预置点槽位后，PLC_Order_Write(EN_DISABLE,1) 触发执行，再 finish_all(150) 等 FINISHALL 并清零。旧 inputAllPoints 只支持 0..8 共 9 个点槽位，超过 9 个点旧代码不会写入 PLC。show_legacy_driver_bridge 之前把旧视觉返回的全部点一次性转发到 /moduan/driver/raw_execute_points，可能写到旧 PLC 槽位表以外地址，导致线模到位后不按旧链路执行。已新增 LEGACY_MODUAN_POINT_SLOT_COUNT=9，对旧 single_bind 和 legacy raw_execute 都截断到 9 点，保持旧包动作链语义；旧角度协议修复仍保留。

### 影响范围

- `src/show_legacy_driver_bridge/scripts/show_legacy_driver_bridge_node.py;src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m py_compile show_legacy_driver_bridge_node.py; python3 -m unittest test_show_legacy_driver_bridge.py test_linear_module_protocol_angle_scaling.py test_driver_algorithm_node_boundaries.py test_systemd_ros_master_ownership.py => 19 tests OK; catkin_make -DCATKIN_WHITELIST_PACKAGES='show_legacy_driver_bridge;tie_robot_bringup'; restarted show_legacy_shared_driver_stack; /show_legacy/moduan/* and /show_legacy/pointAI/* services registered; /moduan/state ready True/error False/idle`

### 后续注意

- 暂无。

## 2026-04-30 18:42 - 视觉图层灰框排查口径

<!-- AGENT-MEMORY: entry -->

### 摘要

- 视觉图层灰框不只可能来自前端 overlay：PR-FPRG/S2 后端结果图曾在 render_manual_workspace_s2_result_image 中用 cv2.polylines(..., (220,220,220), 2) 把手动工作区四边形直接画进 /perception/lashing/result_image 和 /pointAI/result_image_raw；实时 image_callback 也曾画旧 ROI rectangle 和 draw_scan_workspace_overlay。排查灰框时必须同时检查前端 canvas overlay 和后端已烙进像素的 result_image 发布路径。当前已移除这些结果图框线绘制，只保留识别点、线段和标签。

### 影响范围

- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/image_buffers.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `PYTHONPATH=src/tie_robot_perception/src python3 -m unittest ...test_realtime_result_image_omits_legacy_roi_rectangle ...test_realtime_result_image_does_not_draw_workspace_or_legacy_roi_frames ...test_manual_workspace_s2_result_image_does_not_draw_manual_quad_frame; python3 -m py_compile image_buffers.py rendering.py; node test/workspaceCanvasViewOverlay.test.mjs`

### 后续注意

- 暂无。

## 2026-04-30 18:35 - 线模旋转角协议恢复旧尺度

<!-- AGENT-MEMORY: entry -->

### 摘要

- 排查旧 chassis_ctrl 与当前驱动层后确认：旧代码 Set_Module_Coordinate 对 x/y/z 乘 100 写寄存器，但 Set_Motor_Angle 对旋转角只 static_cast<int32_t>(angle)，不乘 100。当前 tie_robot_hw::LinearModuleProtocol 曾把 angle_deg 和 x/y/z 一起乘 100，导致旧视觉/旧前端的 -45 角度按 -4500 类尺度写入 PLC。已改为 x/y/z 使用 scaled word、angle_deg 使用 raw word，并增加 test_linear_module_protocol_angle_scaling.py 锁定该差异。

### 影响范围

- `src/tie_robot_hw/src/driver/linear_module_protocol.cpp;src/tie_robot_bringup/test/test_linear_module_protocol_angle_scaling.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_bringup/test/test_linear_module_protocol_angle_scaling.py; python3 -m unittest src/tie_robot_bringup/test/test_linear_module_protocol_angle_scaling.py src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py src/tie_robot_bringup/test/test_driver_algorithm_node_boundaries.py src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py; catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_hw;tie_robot_control;tie_robot_bringup'; sudo systemctl restart tie-robot-driver-moduan.service; rostopic echo -n 1 /moduan/state => ready True/error False/idle`

### 后续注意

- 暂无。

## 2026-04-30 18:28 - 旧展示视觉切回旧 pointAI 并拦截线模旋转角

<!-- AGENT-MEMORY: entry -->

### 摘要

- show_legacy_driver_bridge 已把旧前端视觉入口改为 /show_legacy/pointAI/process_image：通过 show_legacy_pointai_runner.py 运行旧 chassis_ctrl/scripts/pointAI.py 旧算法 raw 服务 /show_legacy/pointAI/process_image_raw，再由 show_legacy_pointai_proxy_node.py 转换为当前 tie_robot_msgs/ProcessImage。旧前端定点绑扎不再调用当前 /pointAI/process_image；旧算法返回点的 Angle 验证为 -45。桥接层对旧前端末端运动调试/legacy raw execute 增加旋转角安全范围 [-180, 180]，-500 会被拒绝在桥接层，不进入 /moduan/driver/raw_execute_points；旧算法已套旧 lashing_config 偏移，桥接层 apply_legacy_lashing_offset 默认改为 false，避免二次偏移。

### 影响范围

- `src/show_legacy_driver_bridge/scripts/show_legacy_driver_bridge_node.py; src/show_legacy_driver_bridge/scripts/show_legacy_pointai_runner.py; src/show_legacy_driver_bridge/scripts/show_legacy_pointai_proxy_node.py; src/show_legacy_driver_bridge/launch/bridge.launch; src/tie_robot_bringup/launch/show_legacy_shared_driver_stack.launch; src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py src/tie_robot_bringup/test/test_driver_algorithm_node_boundaries.py src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py；catkin_make -DCATKIN_WHITELIST_PACKAGES='show_legacy_driver_bridge;tie_robot_bringup'；rosservice call /show_legacy/pointAI/process_image request_mode:0 返回 28 点且 Angle=-45；rosservice call /show_legacy/moduan/single_move angle:-500 返回 legacy moduan angle out of safe range`

### 后续注意

- 暂无。

## 2026-04-30 18:02 - 旧展示链恢复本地坐标语义

<!-- AGENT-MEMORY: entry -->

### 摘要

- show_legacy_driver_bridge 的旧前端定点绑扎不再转到当前 /moduan/sg BindCheck 主链；改为旧语义：/web/moduan/single_bind 调 /pointAI/process_image 默认模式，叠加旧展示链 chassis_ctrl/data/lashing_config.json 的 cal_x/cal_y/cal_z，把相机/世界点转换回旧末端本地坐标范围后，再调用当前真实线模驱动 /moduan/driver/raw_execute_points。旧路径规划/全局作业也改为读写旧展示链自己的 chassis_ctrl/data/path_points.json，本地路径账本不再写当前 tie_robot_process 路径文件。

### 影响范围

- `src/show_legacy_driver_bridge/scripts/show_legacy_driver_bridge_node.py`
- `src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m py_compile src/show_legacy_driver_bridge/scripts/show_legacy_driver_bridge_node.py`
- `python3 -m unittest src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py src/tie_robot_bringup/test/test_driver_algorithm_node_boundaries.py src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py`
- `catkin_make -DCATKIN_WHITELIST_PACKAGES='tie_robot_msgs;show_legacy_driver_bridge;tie_robot_bringup'`
- `rosservice call /pointAI/process_image request_mode:=0: success true`
- `raw_count=16; applying legacy lashing_config offset produced 9 local executable points in old moduan range`

### 后续注意

- 暂无。

## 2026-04-30 17:48 - 旧展示前端24按钮动作链补全

<!-- AGENT-MEMORY: entry -->

### 摘要

- show_legacy_driver_bridge 已按旧 APP/dist 的 24 个控制按钮逐项对账：补齐 /web/cabin/start、plan_path、clear_path、start_global_work、restart、shutdown、/web/fast_image_solve/process_image、set_height_threshold、set_pointAI_offset、save_path 等旧 topics_transfer 入口；可映射的入口转到当前 /web/system/*、/cabin/plan_path、/cabin/start_work、/pointAI/process_image、/web/pointAI/set_height_threshold、/web/tf/set_offset。当前控制层已直接处理的末端控制 topic 只做观察订阅，不二次下发，避免同一次点击重复执行。

### 影响范围

- `src/show_legacy_driver_bridge/scripts/show_legacy_driver_bridge_node.py;src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py`

### 关键决策

- 见摘要。

### 标签

- `show_legacy`
- `driver_bridge`
- `legacy_frontend`
- `topics_transfer`

### 验证证据

- `python3 -m unittest src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py src/tie_robot_bringup/test/test_driver_algorithm_node_boundaries.py src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py; roslaunch --files tie_robot_bringup show_legacy_shared_driver_stack.launch; roslaunch --files show_legacy_driver_bridge bridge.launch; catkin_make -DCATKIN_WHITELIST_PACKAGES=''`

### 后续注意

- 暂无。

## 2026-04-30 17:38 - 旧展示链定点绑扎调试桥接

<!-- AGENT-MEMORY: entry -->

### 摘要

- 旧展示前端按钮‘定点绑扎调试’发布 std_msgs/Float32 到 /web/moduan/single_bind；show_legacy_driver_bridge 必须订阅该 topic，将非 0 触发按旧 topics_transfer 语义转发到当前 std_srvs/Trigger 服务 /moduan/sg，并暴露兼容服务 /show_legacy/moduan/single_bind。旧代码仍不直接接触 TCP/Modbus。

### 影响范围

- `src/show_legacy_driver_bridge/scripts/show_legacy_driver_bridge_node.py;src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py`

### 关键决策

- 见摘要。

### 标签

- `show_legacy`
- `driver_bridge`
- `moduan`
- `single_bind`

### 验证证据

- `python3 -m unittest src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py src/tie_robot_bringup/test/test_driver_algorithm_node_boundaries.py src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py; catkin_make -DCATKIN_WHITELIST_PACKAGES=''`

### 后续注意

- 暂无。

## 2026-04-30 17:29 - 旧展示前端5173已交给systemd自启

<!-- AGENT-MEMORY: entry -->

### 摘要

- 已安装并启动 tie-robot-show-legacy-frontend.service，服务以 python3 -m http.server 5173 --bind 0.0.0.0 --directory /home/hyq-/lashingrobotROS/src/APP/dist 运行；原手动占用5173的 python3 -m http.server 进程已停止，5173 现在由 systemd 管理。

### 影响范围

- `src/tie_robot_bringup/systemd/tie-robot-show-legacy-frontend.service.in; src/tie_robot_bringup/scripts/install_show_legacy_frontend_service.sh`

### 关键决策

- 旧展示前端5173采用独立systemd service自启，不跟当前8080前端共生命周期。

### 标签

- `show-legacy`
- `frontend`
- `systemd`
- `5173`

### 验证证据

- `systemctl is-enabled tie-robot-show-legacy-frontend.service -> enabled; systemctl is-active -> active; curl -I http://127.0.0.1:5100/ -> HTTP/1.0 200 OK`

### 后续注意

- 暂无。

## 2026-04-30 17:27 - 新增旧展示链共享驱动桥接层

<!-- AGENT-MEMORY: entry -->

### 摘要

- 当前工作区新增 show_legacy_driver_bridge ROS 包，只做旧展示链到当前驱动层的 ROS 转义：/show_legacy/cabin/raw_move -> /cabin/driver/raw_move，/show_legacy/cabin/incremental_move -> /cabin/driver/incremental_move，/show_legacy/moduan/single_move 和 /show_legacy/moduan/raw_execute_points -> /moduan/driver/raw_execute_points；同时转发 /cabin/cabin_data_upload 到 /show_legacy/cabin/cabin_data_upload 并发布 /robot/chassis_status。该桥接层不直接接触 TCP/Modbus/tie_robot_hw，避免旧代码抢占硬件。新增 show_legacy_shared_driver_stack.launch 和 tie-robot-show-legacy-frontend.service.in 用于独立启动旧展示前端 5173。

### 影响范围

- `src/show_legacy_driver_bridge; src/tie_robot_bringup/launch/show_legacy_shared_driver_stack.launch; src/tie_robot_bringup/systemd/tie-robot-show-legacy-frontend.service.in; src/tie_robot_bringup/scripts/install_show_legacy_frontend_service.sh; src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py`

### 关键决策

- 两套前端/执行链共享当前唯一驱动层；旧展示链通过新 bridge/独立 launch/service 接入，不启动旧 suoquNode/moduanNode 硬件拥有者。

### 标签

- `show-legacy`
- `driver-bridge`
- `ros-launch`
- `systemd`
- `shared-driver`

### 验证证据

- `python3 -m unittest src/tie_robot_bringup/test/test_show_legacy_driver_bridge.py src/tie_robot_bringup/test/test_driver_algorithm_node_boundaries.py src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py; roslaunch --files tie_robot_bringup show_legacy_shared_driver_stack.launch; roslaunch --files show_legacy_driver_bridge bridge.launch; catkin_make -DCATKIN_WHITELIST_PACKAGES=""`

### 后续注意

- 暂无。

## 2026-04-30 17:09 - PR-FPRG 分割训练包补齐 9 输入模态

<!-- AGENT-MEMORY: entry -->

### 摘要

- Colab 训练包 notebooks/pr_fprg_multimodal_segmentation_colab.py 和 .ipynb 已从旧 6 通道扩展为 9 个非 RGB 输入通道：ir、depth_z、worldcoord_height_response、depth_response、depth_gradient、combined_response、hessian_ridge、frangi_like、fused_instance_response；valid_mask/workspace_mask 作为训练 ignore/support mask。保留 rectified_ir、rectified_depth、raw_world_z 等别名兼容数据集。

### 影响范围

- `notebooks/pr_fprg_multimodal_segmentation_colab.py; notebooks/pr_fprg_multimodal_segmentation_colab.ipynb; src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- valid_mask/workspace_mask 默认要求存在，作为训练 mask/ignore 区约束，不重复作为网络输入通道。

### 标签

- `pr-fprg`
- `training-package`
- `modalities`
- `colab`
- `segmentation`

### 验证证据

- `python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k full_non_rgb_modality; compile() syntax check; ipynb JSON/modality token check`

### 后续注意

- 暂无。

## 2026-04-30 16:34 - 索驱0x0011 Z-1mm实机测试对端关连接

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户授权只发送 Z- 1mm 作为实机连通性测试包。2026-04-30 16:33 调用 /cabin/driver/incremental_move x=0 y=0 z=-1 speed=100，驱动下发 request_command=0x0011 request_frame=[EB 90 00 11 20 00 00 00 C8 42 00 00 80 3F 75 03]，结果 tcp_recv_failed: connection closed by peer。随后状态心跳读包报对端关闭并自动重连成功，/cabin/cabin_data_upload 仍在线，Z 保持 3078.6001mm，未发生 1mm 位移。结论：当前索驱上位机对协议第8节 0x0011 增量运动不返回协议应答且关闭连接；这不是 TF/前端坐标问题。

### 影响范围

- `docs/agent_memory/session_log.md`
- `docs/agent_memory/current.md`

### 关键决策

- 见摘要。

### 验证证据

- `rosservice call /cabin/driver/incremental_move z=-1 speed=100; journalctl confirmed request_command=0x0011 peer close; rostopic echo /cabin/cabin_data_upload confirmed cabin_connect_flag=1 and unchanged Z`

### 后续注意

- 暂无。

## 2026-04-30 16:18 - 索驱增量服务恢复为0x0011协议原子

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户确认相对位置运动不应混同通信协议第8节‘TCP坐标系增量运动’。在状态心跳完整读包修复后，/cabin/driver/incremental_move 已恢复为 CabinProtocol::buildIncrementalMoveFrame(command) 并按 0x0011 解码回包；0x0012 bit1=TCP相对位置运动触发只保留为独立协议原子函数，不再作为增量服务实现。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; catkin_make --pkg tie_robot_hw tie_robot_process; sudo -n systemctl restart tie-robot-driver-suoqu.service; rostopic echo -n1 /cabin/cabin_data_upload; invalid two-axis /cabin/driver/incremental_move rejected before TCP motion`

### 后续注意

- 暂无。

## 2026-04-30 16:16 - 回退PR-FPRG调试底图前端图像面板接入

<!-- AGENT-MEMORY: entry -->

### 摘要

- 已按用户要求回退‘在前端图像下拉新增 rectified_ir/depth_response/combined_response/Hessian/Frangi/fused_instance/mask 等 PR-FPRG 调试底图’这一需求。当前前端图像面板不再注册 TOPICS.algorithm.visualModalities，也不再显示 /perception/lashing/debug/* 调试图层；pointAI 不再新增 debug_modalities.py 或对应调试图像发布器。

### 影响范围

- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/config/imageTopicCatalog.js`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`

### 关键决策

- 见摘要。

### 验证证据

- `rg -n 'visualModalities|/perception/lashing/debug|debug_modalities|publish_manual_workspace_s2_debug_modalities' src/tie_robot_web/frontend/src/config src/tie_robot_perception/src/tie_robot_perception/pointai; npm run build`

### 后续注意

- 暂无。

## 2026-04-30 16:09 - 索驱状态心跳完整读包避免污染运动回包

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场上位机状态看起来一切正常但增量运动不能动，根因很可能是旧 Frame_Generate 对状态心跳只调用一次 recv，且 read_cabin_state 传入 sizeof(cabin_state_buffer) 而不是协议144字节；TCP 分片时状态包尾字节可能残留在同一 socket，被下一条 /cabin/driver/incremental_move 误读成8字节运动回包，出现类似 status_word=0xC3820000 的异常状态。已改为 Frame_Generate 按 Rlen 循环接收完整回包，read_cabin_state 固定读取 CABIN_STATE_RESPONSE_BYTES=144。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; catkin_make --pkg tie_robot_hw tie_robot_process; sudo -n systemctl restart tie-robot-driver-suoqu.service; rostopic echo -n1 /cabin/cabin_data_upload`

### 后续注意

- 暂无。

## 2026-04-30 16:07 - PR-FPRG调试底图接入前端图像面板

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端图像下拉新增 PR-FPRG 调试底图：rectified_ir、rectified_depth/raw_world z、worldCoord_height_response、depth_response、depth_gradient、combined_response、Hessian ridge、Frangi-like、fused_instance_response、valid_mask、workspace_mask；raw IR/depth 继续复用已有相机图层。pointAI 在 run_manual_workspace_s2_pipeline(publish=True) 拿到透视展开输入后发布 /perception/lashing/debug/* 图像，便于现场直接切换底图查看。

### 影响范围

- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/config/imageTopicCatalog.js`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/debug_modalities.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k image_panel; python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; npm run build`

### 后续注意

- 暂无。

## 2026-04-30 16:02 - 索驱0x0012状态字高位回包归一化

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场点击索驱遥控后收到 motion_command_rejected detail=status_word=0xC3820000。该值按驱动原先 little-endian 32 位读取时有效位落在协议未定义高位，无法给出原因；按回包字节 00 00 82 C3 的协议位宽归一化后为 status_word=0x000082C3，对应逆解未激活、电机未全部使能、X超负限位、Y超正限位、Z超正限位、C超正限位。CabinProtocol::decodeStatus 现在保留 raw_status_le32，同时输出归一化 status_word、status_source、原因列表和 response_frame。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_hw/src/driver/cabin_protocol.cpp`
- `src/tie_robot_process/test/test_cabin_protocol_contract.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; catkin_make --pkg tie_robot_hw tie_robot_process; sudo -n systemctl restart tie-robot-driver-suoqu.service; rostopic echo -n1 /cabin/cabin_data_upload`

### 后续注意

- 暂无。

## 2026-04-30 15:48 - 多模态钢筋面分割Colab训练脚本

<!-- AGENT-MEMORY: entry -->

### 摘要

- 新增 notebooks/pr_fprg_multimodal_segmentation_colab.py 与同名 .ipynb。Colab 脚本可从 Google Drive 导入 dataset zip，读取非 RGB 多模态通道 ir/depth_z/worldcoord_height/depth_gradient/combined_response/frangi_like，训练 U-Net++/U-Net/DeepLabV3+ 分割 ordinary_rebar/beam_rebar/floor_seam/occluder/background，使用 CE+Dice、ignore=255、mIoU/precision/recall 指标，导出 best_model.pt、ONNX、预测可视化和 training_summary，并将结果 zip 回写 Google Drive。

### 影响范围

- `notebooks/pr_fprg_multimodal_segmentation_colab.py;notebooks/pr_fprg_multimodal_segmentation_colab.ipynb`

### 关键决策

- 第一版监督分割训练默认走 U-Net++ + efficientnet-b0 + 6通道非RGB输入；DeepLabV3+ 可作为对照实验取消注释后运行。

### 标签

- `vision`
- `segmentation`
- `colab`
- `training`
- `unetpp`
- `multimodal`

### 验证证据

- `python3 -m py_compile notebooks/pr_fprg_multimodal_segmentation_colab.py; json loads .ipynb nbformat=4 cells=14; git diff --check notebooks/pr_fprg_multimodal_segmentation_colab.py notebooks/pr_fprg_multimodal_segmentation_colab.ipynb`

### 后续注意

- 暂无。

## 2026-04-30 15:45 - 索驱TCP错误detail追加发帧上下文后的停止兼容

<!-- AGENT-MEMORY: entry -->

### 摘要

- CabinTcpTransport 现在会在 tcp_send_failed/tcp_read_wait_failed/tcp_recv_failed 的 detail 追加 request_command 和 request_frame。任何依赖 detail 精确等于 connection closed by peer 的逻辑都应改为片段匹配；CabinDriver::sendStop 已改为 detail.find('connection closed by peer')，保持停止帧被对端关连接时仍按已投递处理。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; catkin_make --pkg tie_robot_hw tie_robot_process; sudo -n systemctl restart tie-robot-driver-suoqu.service; rostopic echo -n1 /cabin/cabin_data_upload`

### 后续注意

- 暂无。

## 2026-04-30 15:43 - 索驱遥控步距改用0x0012相对位置触发

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-30 现场点击索驱遥控时，/cabin/driver/incremental_move 通过0x0011短帧下发会出现 tcp_recv_failed: connection closed by peer；状态心跳随后可恢复，说明不是IP/网线断链，而是对端上位机未返回协议状态字就关闭本次运动命令socket。驱动保持服务名不变，moveByOffset 改用通信协议0x0012的bit1=TCP相对位置运动触发来表达单轴步距；0x0011 buildIncrementalMoveFrame 只作为协议原子函数保留。CabinTcpTransport 的发送/读等待/接收失败 detail 现在带 request_command 和 request_frame，便于后续定位实际发帧。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_protocol.hpp`
- `src/tie_robot_hw/src/driver/cabin_protocol.cpp`
- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_process/test/test_cabin_protocol_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; catkin_make --pkg tie_robot_hw tie_robot_process; sudo -n systemctl restart tie-robot-driver-suoqu.service; rostopic echo -n1 /cabin/cabin_data_upload; rosservice call /cabin/driver/incremental_move with two nonzero axes returned invalid-axis without TCP motion`

### 后续注意

- 暂无。

## 2026-04-30 15:29 - Home点位人工输入边界

<!-- AGENT-MEMORY: entry -->

### 摘要

- 设置页和 RobotHomeCalibration 服务请求只允许人工写入索驱 Home 点位；base_link 到相机不再从前端手填，服务仅返回当前 TF/相机投射状态供显示。map 仍是索驱全局坐标系，map.z=0 是索驱绝对零点而非地面。

### 影响范围

- `src/tie_robot_msgs/srv/RobotHomeCalibration.srv`
- `src/tie_robot_perception/scripts/robot_tf_broadcaster.py`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`

### 关键决策

- 见摘要。

### 验证证据

- `catkin_make; python3 src/tie_robot_perception/test/test_robot_tf_broadcaster.py && python3 src/tie_robot_perception/test/test_gripper_tf_broadcaster.py && python3 src/tie_robot_process/test/test_tf_coordinate_contract.py && python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py; node frontend Home/ROS tests; npm run build`

### 后续注意

- 暂无。

## 2026-04-30 15:21 - 索驱状态轮询成功后清理transport超时态

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-30 15:15 一次 /cabin/driver/incremental_move 出现 索驱 TCP 等待回包失败 detail=read timeout 后，/cabin/cabin_data_upload 已继续 fresh 且 cabin_connect_flag=1，但 CabinTcpTransport 仍停在 kReconnecting 且 lastErrorText 保留 read timeout，导致 diagnostics/front-end 误报‘索驱驱动通信异常/断开’。已新增 markExternalIoSuccess：状态轮询 Frame_Generate_With_Retry 成功读回心跳/状态包后，将 transport_state 标回 ready 并清空旧 transport_error。

### 影响范围

- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_tcp_transport.hpp;src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp;src/tie_robot_hw/include/tie_robot_hw/driver/cabin_driver.hpp;src/tie_robot_hw/src/driver/cabin_driver.cpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/test/test_motion_chain_signal_guard.py;CHANGELOG.md`

### 关键决策

- 见摘要。

### 标签

- `suoqu`
- `diagnostics`
- `transport-state`

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract; catkin_make --pkg tie_robot_hw tie_robot_process; sudo -n systemctl restart tie-robot-driver-suoqu.service; diagnostics shows suoqu_driver_node level=0 transport_state=ready transport_error='' and /cabin/cabin_data_upload cabin_connect_flag=1`

### 后续注意

- 暂无。

## 2026-04-30 15:11 - 索驱0x0011增量运动协议修正

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱驱动层 0x0011 TCP 增量运动帧已按通信协议修正为 16 字节：单轴方向控制字 + speed float + 正的增量位移 float + checksum；/cabin/driver/incremental_move 在驱动层拒绝多轴或零轴请求。0x0011/0x0012/0x0013 回包状态字现在按字节 2~5 的 UINT32 解码，并把协议 bit 原因写入 detail，例如逆解未激活、电机未全部使能、设备运动中、限位、速度错误。此前 connection closed by peer 的根因是 0x0011 被误按 0x0012 六自由度目标帧发送，上位机直接断开连接。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_protocol.cpp;src/tie_robot_hw/src/driver/cabin_driver.cpp;src/tie_robot_process/test/test_cabin_protocol_contract.py;CHANGELOG.md`

### 关键决策

- 见摘要。

### 标签

- `suoqu`
- `cabin-driver`
- `tcp-protocol`

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_cabin_protocol_contract; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; catkin_make --pkg tie_robot_hw tie_robot_process; sudo -n systemctl restart tie-robot-driver-suoqu.service; rostopic echo -n1 /cabin/cabin_data_upload; rosservice call /cabin/driver/incremental_move with two nonzero axes returned invalid-axis protocol detail`

### 后续注意

- 暂无。

## 2026-04-30 14:47 - PR-FPRG-MS多尺度绑扎点实验路线

<!-- AGENT-MEMORY: entry -->

### 摘要

- 新增 PR-FPRG-MS 多尺度实验脚本并发布 /reports/pr_fprg_multiscale_bindpoint_experiment。路线为组合响应 -> 尺度判别 -> 2026-04-22 频相拓扑 / 钢筋面补全 / 实例骨架 -> beam_candidate +/-13cm 点级排除 -> 3/4/5/6 局部收束 -> 锚点式多源融合。当前 raw_world 实验判为 frequency_primary：当前运行主链 [8,8]/64 点，频相恢复 [17,13]，梁筋13cm过滤后 221 点；最终融合使用频相锚点 221 点，曲线/补全面/实例图谱只作为锚点置信度支持，避免无锚点候选并集膨胀。当前帧未检测到 beam_candidate 梁筋 band，报告中记录 mask 像素为 0。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_multiscale_bindpoint_experiment.py;src/tie_robot_web/web/reports/pr_fprg_multiscale_bindpoint_experiment/index.html;src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 后续跨尺度绑扎点识别优先按 PR-FPRG-MS 锚点式融合推进；不要再把频相、实例骨架和曲线候选简单取并集。

### 标签

- `vision`
- `pr-fprg-ms`
- `multiscale`
- `bindpoint`
- `beam-mask`

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_perception/test -p 'test_pointai_scan_only_pr_fprg.py' => 86 tests OK; py_compile multiscale/topology tools OK; diff --check OK; curl 127.0.0.1 and 192.168.6.99 report URL => HTTP 200; Firefox mobile screenshot generated for layout check.`

### 后续注意

- 暂无。

## 2026-04-30 14:40 - ROS全栈重启快速清理旧进程

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-30：/api/system/restart_ros_stack 改为快速重启路径：先 systemctl stop backend、三个 driver 和 rosbridge，stop 等待 8s；随后扫描本工作空间 devel/lib/tie_robot_* 节点、roslaunch tie_robot_bringup、rosmaster :11311、rosout、rosbridge_websocket、rosapi_node、tf2_web_republisher，先 SIGTERM，短等待后对残留进程 SIGKILL，再按 rosbridge -> driver -> backend 启动。backend、rosbridge 和三个 driver unit 的 TimeoutStopSec 均收短到 5s；_run_systemctl 捕获 TimeoutExpired 并返回结构化失败，避免 HTTP 请求崩掉。已重新安装 backend/driver/rosbridge unit 并重启 frontend 服务让新逻辑生效。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_bringup/systemd/tie-robot-backend.service.in`
- `src/tie_robot_bringup/systemd/tie-robot-rosbridge.service.in`
- `src/tie_robot_bringup/systemd/tie-robot-driver-suoqu.service.in`
- `src/tie_robot_bringup/systemd/tie-robot-driver-moduan.service.in`
- `src/tie_robot_bringup/systemd/tie-robot-driver-camera.service.in`
- `CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest targeted workspace_picker_web systemd/restart tests plus src.tie_robot_bringup.test.test_systemd_ros_master_ownership; py_compile workspace_picker_web_server.py; systemd-analyze verify updated unit files; curl -X POST /api/system/restart_ros_stack returned success with stop/cleanup/start steps all 0; rostopic info /Scepter/ir/image_raw and /coordinate_point show publishers; rosnode ping /scepter_manager and /pointAINode succeed; /Scepter/ir/image_raw approx 5Hz`

### 后续注意

- 暂无。

## 2026-04-30 14:37 - 视觉调试与相机TCP外参合并

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端设置页已移除独立“相机-TCP外参”入口，外参卡片并入“视觉调试”页；旧本地偏好 pageId=calibration 会通过别名自动归一到 visualDebug。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js; src/tie_robot_web/test/test_workspace_picker_web.py; src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `settings`
- `vision`
- `calibration`

### 验证证据

- `python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k test_frontend_shell_uses_task_only_left_panel_top_status_and_bottom_quick_controls; python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k test_settings_page_dropdown_supports_drag_order_persistence; npm run build; sudo -n systemctl restart tie-robot-frontend.service && systemctl is-active tie-robot-frontend.service`

### 后续注意

- 暂无。

## 2026-04-30 14:32 - 设置页显示与图层合并

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端设置页已移除独立的“图层与数据”入口，三维视角控制和图层/点云/坐标轴数据显示控件统一放在“显示与视角”页；旧本地偏好 pageId=layers 会通过别名自动归一到 scene。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js; src/tie_robot_web/test/test_workspace_picker_web.py; src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `settings`
- `scene3d`

### 验证证据

- `python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k test_frontend_shell_uses_task_only_left_panel_top_status_and_bottom_quick_controls; python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k test_settings_page_dropdown_supports_drag_order_persistence; npm run build; sudo -n systemctl restart tie-robot-frontend.service && systemctl is-active tie-robot-frontend.service`

### 后续注意

- 暂无。

## 2026-04-30 14:30 - rosbridge重启后视觉链路断链根因

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-30：现场视觉层断链根因不是 PR-FPRG 算法，而是 tie-robot-rosbridge.service 在 13:42 重启并重新拥有 ROS master 后，先前启动的 camera/backend/moduan 进程仍然活着但未重新注册到当前 master，导致 /Scepter/ir/image_raw、/Scepter/depth/image_raw 无发布者，/pointAINode 等 XML-RPC 地址拒绝连接。已给 backend 与三个 driver systemd unit 增加 PartOf=tie-robot-rosbridge.service，并重新安装 unit；随后重启 backend/camera/moduan 让节点重新注册。后续改 ROS master 守护关系时要保持依赖当前 master 的服务跟随 rosbridge restart。

### 影响范围

- `src/tie_robot_bringup/systemd/tie-robot-backend.service.in`
- `src/tie_robot_bringup/systemd/tie-robot-driver-camera.service.in`
- `src/tie_robot_bringup/systemd/tie-robot-driver-moduan.service.in`
- `src/tie_robot_bringup/systemd/tie-robot-driver-suoqu.service.in`
- `src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_bringup.test.test_systemd_ros_master_ownership src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_ros_backend_is_systemd_managed_from_frontend src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_drivers_have_independent_systemd_guard_and_frontend_controls; systemd-analyze verify /etc/systemd/system/tie-robot-backend.service /etc/systemd/system/tie-robot-driver-camera.service /etc/systemd/system/tie-robot-driver-moduan.service /etc/systemd/system/tie-robot-driver-suoqu.service; rostopic info /Scepter/ir/image_raw and /coordinate_point show publishers; rosnode ping /scepter_manager and /pointAINode succeed`

### 后续注意

- 暂无。

## 2026-04-30 14:23 - 前端显示与视角三模式

<!-- AGENT-MEMORY: entry -->

### 摘要

- 显示与视角页改为紧凑视角控制；默认自由视角并保留 Orbit 拖拽；相机视角锁定 Scepter_depth_frame 原点且沿相机 z+ 看；俯视视角锁定世界原点上方且沿全局 z- 看；跟随开关语义统一为跟随当前视角原点。

### 影响范围

- `src/tie_robot_web/frontend/src/config/topicLayerCatalog.js;src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/frontend/src/views/Scene3DView.js;src/tie_robot_web/frontend/src/controllers/TopicLayerController.js;src/tie_robot_web/frontend/src/styles/app.css;src/tie_robot_web/test/test_workspace_picker_web.py;CHANGELOG.md`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `scene-view`
- `threejs`

### 验证证据

- `npm run build; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_frontend_assets_exist src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_scene_view_modes_are_free_camera_top_and_follow_origin src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_base_link_splits_machine_pose_from_downward_camera_frame`

### 后续注意

- 暂无。

## 2026-04-30 14:22 - 3/4/5/6以频相恢复主线为基底完成实验

<!-- AGENT-MEMORY: entry -->

### 摘要

- PR-FPRG topology recovery experiment 已将 3/4/5/6 曲线算法接到当前推荐主线：组合响应 + S2 正交频相/FFT 峰值恢复 [17,16] 全拓扑，再套 beam_candidate +/-13cm 排除梁筋区。2026-04-30 最新报告使用 snapshot:rebar_instance_segmentation_modalities_20260430_112028：当前 peak-supported 主链 72 点；频相恢复滤前 272 点；梁筋过滤后 170 点；3/4/5/6 均在恢复主线基底上输出 170 点且 fallback=False，其中 DP 曲线均分最高约 0.803。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_topology_recovery_experiment.py;src/tie_robot_web/web/reports/pr_fprg_topology_recovery_experiment/index.html;src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 后续讨论 3/4/5/6 时默认以频相恢复主线作为基底对比，不再回到废弃的 1/2 方案或单独低召回曲线底图。

### 标签

- `vision`
- `pr-fprg`
- `topology-recovery`
- `curve3456`
- `beam-mask`

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_perception/test -p 'test_pointai_scan_only_pr_fprg.py' => 85 tests OK; py_compile topology experiment OK; diff --check OK; HTTP 200 at /reports/pr_fprg_topology_recovery_experiment/index.html`

### 后续注意

- 暂无。

## 2026-04-30 14:13 - 前端网络配置卡片红绿按钮反馈

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-30：设置页原‘网络测试’改为‘网络配置’，索驱/线性模组上位机 IP 输入框由同一个‘保存并测试’按钮保存并触发 /api/network/ping；前端 getNetworkPingSettings 保留输入框当前值，空输入不再静默回退默认 IP。按钮 data-state=pending/success/error 分别显示测试中/绿色/红色，结果区只显示连接状态，不再把 stdout/stderr 当主界面输出。默认 IP 仍为索驱 192.168.6.62、线性模组 192.168.6.167，并通过 localStorage 保存现场输入。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `npm run build; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_network_ping_api_uses_safe_ping_command src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_settings_page_has_manual_network_ping_panel`

### 后续注意

- 暂无。

## 2026-04-30 14:06 - PR-FPRG频相主拓扑恢复逐步实验报告

<!-- AGENT-MEMORY: entry -->

### 摘要

- 新增 src/tie_robot_perception/tools/pr_fprg_topology_recovery_experiment.py，并发布 /reports/pr_fprg_topology_recovery_experiment。该报告真实跑当前帧/可用快照，逐步输出：输入工作区、rectified IR、组合响应、纵横 profile+FFT 驼峰、频相完整网格、beam_candidate ±13cm mask、过滤前原图点位、过滤后原图点位、当前 peak-supported 对照。当前运行因 ROS 同步帧超时使用 snapshot:rebar_instance_segmentation_modalities_20260430_112028；结果：当前 peak-supported [9,8]/72 点，恢复频相网格 [17,16]/272 点，beam_candidate ±13cm 过滤后 170 点，验证了 4月22日频相主拓扑在该尺度下能恢复高召回候选。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_topology_recovery_experiment.py`
- `src/tie_robot_web/web/reports/pr_fprg_topology_recovery_experiment`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_perception/test -p 'test_pointai_scan_only_pr_fprg.py' -> 85 tests OK; py_compile report tool -> OK; git diff --check relevant paths -> OK; curl -I /reports/pr_fprg_topology_recovery_experiment/index.html -> HTTP 200; Firefox mobile screenshot OK`

### 后续注意

- 暂无。

## 2026-04-30 13:54 - 前端TF坐标轴姿态跟随TF

<!-- AGENT-MEMORY: entry -->

### 摘要

- 3D视图中 base_link、Scepter_depth_frame、gripper_frame 坐标轴图标现在复制 /tf 合成后的 position/quaternion；map 轴仍为世界参考轴。机器人机身和 TCP 工具模型继续使用原有 display pose，不随本次改动改变显示约定。

### 影响范围

- `src/tie_robot_web/frontend/src/views/Scene3DView.js; src/tie_robot_web/test/test_workspace_picker_web.py; src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `tf`
- `scene3d`

### 验证证据

- `python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k test_base_link_splits_machine_pose_from_downward_camera_frame; python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k test_viewer_app_no_longer_renders_topics_tf_problems_panels; python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -k test_tf_axes_can_be_toggled_per_frame; npm run build; sudo -n systemctl restart tie-robot-frontend.service && systemctl is-active tie-robot-frontend.service`

### 后续注意

- 暂无。

## 2026-04-30 13:53 - PR-FPRG高召回主拓扑恢复流程报告

<!-- AGENT-MEMORY: entry -->

### 摘要

- 新增 /reports/pr_fprg_topology_recovery_flow 静态报告，使用流程图明确下一版视觉算法口径：恢复 2026-04-22 的 FFT/自相关 period + phase 完整网格生成作为主拓扑；组合响应、钢筋面、beam_candidate ±13cm 和 3/4/5/6 曲线方案只作为底图增强、最终点级过滤、评分和局部收束。报告包含 SVG/PNG 流程图，移动端截图已检查。

### 影响范围

- `src/tie_robot_web/web/reports/pr_fprg_topology_recovery_flow/index.html`
- `src/tie_robot_web/web/reports/pr_fprg_topology_recovery_flow/images/pr_fprg_recovered_algorithm_flow.svg`
- `src/tie_robot_web/web/reports/pr_fprg_topology_recovery_flow/images/pr_fprg_recovered_algorithm_flow.png`

### 关键决策

- 见摘要。

### 验证证据

- `SVG XML parse OK; cairosvg 导出 PNG OK; HTML parse OK; local image refs OK; curl -I /reports/pr_fprg_topology_recovery_flow/index.html -> HTTP 200; git diff --check report path -> OK; Firefox mobile screenshot OK`

### 后续注意

- 暂无。

## 2026-04-30 13:50 - 索驱遥控绝对位姿默认填当前坐标

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱遥控页删除重复的当前索驱坐标块，当前坐标继续由页面中心/底部机器位置条显示；绝对目标位姿 X/Y/Z 输入框随 /cabin/cabin_data_upload 的原始 cabin_state_X/Y/Z 自动填入默认值，用户正在编辑任一绝对位姿输入时不被状态刷新覆盖。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js;src/tie_robot_web/test/test_workspace_picker_web.py;CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_settings_panel_renders_cabin_remote_page src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_cabin_remote_keyboard_and_tf_flow_exist; node test/cabinRemoteController.test.mjs && node test/cabinRemoteButtonSingleFire.test.mjs && node test/cabinRemoteOperationState.test.mjs && node test/cabinRemoteKeyboard.test.mjs; npm run build`

### 后续注意

- 暂无。

## 2026-04-30 13:50 - 设置页主页与排序联动

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-30：前端设置下拉的主页和排序统一为一套规则：设置为主页的页面自动移动到下拉第一项并保存排序；拖拽或键盘移动到第一项的页面自动成为主页并保存主页偏好。实现集中在 UIController 的 getSettingsPageHomeFirstOrder、notifySettingsPageOrderCommitted、setSettingsHomePage，App 初始化会把保存的主页规范化到第一位并回写 settings page order。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js;src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `settings`
- `persistence`

### 验证证据

- `node --check UIController.js/App；python3 -m unittest ...test_settings_page_dropdown_supports_drag_order_persistence；npm run build；curl 确认服务 index.html 指向 index-CV8ElP5N.js。`

### 后续注意

- 暂无。

## 2026-04-30 13:45 - TF全局X取反已回退

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求回退此前‘物理 X+ 对应数字 map X-’修改。当前 robot_home_tf.yaml 和 robot_tf_broadcaster 默认 cabin_to_map_sign 均为 x=+1、y=+1、z=+1；TF 层恢复将 /cabin/cabin_data_upload 的 cabin_state_X 同号发布为 map->base_link.translation.x。现场验证 cabin_state_X=-260.0mm 时 tf_echo map base_link translation.x=-0.260m。

### 影响范围

- `CHANGELOG.md;src/tie_robot_perception/config/robot_home_tf.yaml;src/tie_robot_perception/scripts/robot_tf_broadcaster.py;src/tie_robot_perception/test/test_robot_tf_broadcaster.py`

### 关键决策

- 见摘要。

### 标签

- `tf`
- `frontend`
- `perception`
- `cabin`

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_robot_tf_broadcaster src.tie_robot_perception.test.test_gripper_tf_broadcaster; python3 -m py_compile src/tie_robot_perception/scripts/robot_tf_broadcaster.py; sudo -n systemctl restart tie-robot-rosbridge.service; sudo -n systemctl restart tie-robot-driver-suoqu.service; rostopic /cabin/cabin_data_upload 与 tf_echo map base_link 验证 X 同号。`

### 后续注意

- 暂无。

## 2026-04-30 13:40 - 索驱遥控按钮改用 TCP 增量运动

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端索驱遥控方向按钮和键盘遥控现在调用 /cabin/driver/incremental_move，对应索驱 TCP 0x0011 增量运动帧，SingleMove.x/y/z 表示本次增量步距；绝对目标位姿移动作为遥控页独立功能保留，继续调用 /cabin/driver/raw_move，对应 TCP 0x0012 绝对位置运动。后续不要把按钮点动重新接到 raw_move，也不要用 TF/map 坐标推导按钮目标。

### 影响范围

- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_protocol.hpp;src/tie_robot_hw/src/driver/cabin_protocol.cpp;src/tie_robot_hw/include/tie_robot_hw/driver/cabin_driver.hpp;src/tie_robot_hw/src/driver/cabin_driver.cpp;src/tie_robot_process/include/tie_robot_process/suoqu/cabin_transport.hpp;src/tie_robot_process/src/suoqu/cabin_transport.cpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_web/frontend/src/controllers/CabinRemoteController.js;src/tie_robot_web/frontend/src/controllers/RosConnectionController.js;src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js;src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/frontend/src/config/topicRegistry.js;CHANGELOG.md`

### 关键决策

- 见摘要。

### 验证证据

- `node test/cabinRemoteController.test.mjs && node test/cabinRemoteButtonSingleFire.test.mjs && node test/cabinRemoteOperationState.test.mjs && node test/cabinRemoteKeyboard.test.mjs; python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_settings_panel_renders_cabin_remote_page src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_cabin_remote_keyboard_and_tf_flow_exist; npm run build; source /opt/ros/noetic/setup.bash && catkin_make`

### 后续注意

- 暂无。

## 2026-04-30 13:36 - 索驱停止服务按现场TCP断连语义处理

<!-- AGENT-MEMORY: entry -->

### 摘要

- 修复前端停止索驱运行无效的问题：/cabin/motion/stop 由 /suoqu_driver_node 提供，手动调用失败的现场返回是停止帧发出后 TCP recv 得到 connection closed by peer。CabinDriver::sendStop 现在把停止帧发出后的 peer close 视为停止指令已投递并清理本地连接；Frame_Generate_With_Retry 在 connectToServer 成功后刷新本轮 socket=sockfd，避免状态读取线程继续用 -1/旧 fd 并触发紧急退出。

### 影响范围

- `src/tie_robot_hw/src/driver/cabin_driver.cpp;src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 标签

- `ros`
- `cabin`
- `driver`

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; catkin_make --pkg tie_robot_hw tie_robot_process; catkin_make --pkg tie_robot_process; sudo -n systemctl restart tie-robot-driver-suoqu.service; rosservice call /cabin/motion/stop '{}' => success: True; 服务保持 active 且 /suoqu_driver_node PID 稳定。`

### 后续注意

- 暂无。

## 2026-04-30 13:27 - 索驱遥控 raw move 使用原始坐标

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱驱动层 /cabin/driver/raw_move 对应 TCP 0x0012 位置运动，参数 x/y/z 是索驱上位机原始绝对坐标，不是 TF/map 坐标也不是增量步距。前端遥控步进必须从 /cabin/cabin_data_upload 的 cabin_state_X/Y/Z 计算目标位姿；TF 中 map->base_link 可能按现场方向映射，例如 map.x=-cabin_state_X，不能直接回灌 raw_move，否则一次点击会运动到错误远端目标。

### 影响范围

- `src/tie_robot_web/frontend/src/controllers/CabinRemoteController.js;src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js;src/tie_robot_web/frontend/test/cabinRemoteController.test.mjs`

### 关键决策

- 见摘要。

### 验证证据

- `node test/cabinRemoteController.test.mjs && node test/cabinRemoteButtonSingleFire.test.mjs && node test/cabinRemoteOperationState.test.mjs && node test/cabinRemoteKeyboard.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_cabin_remote_keyboard_and_tf_flow_exist; npm run build`

### 后续注意

- 暂无。

## 2026-04-30 13:12 - 梁筋图谱排除范围改为13cm

<!-- AGENT-MEMORY: entry -->

### 摘要

- PR-FPRG 主链和相关实验报告的梁筋图谱排除外扩距离从 ±10cm 改为 ±13cm：manual_workspace_s2.py 使用 beam_exclusion_margin_mm=130.0；stage/scheme/report 工具同步 130mm 文案；rebar_instance_graph_probe 与 rebar_surface_bindpoint_comparison 的报告对比项、summary key 和图片名从 *_10cm 更新为 *_13cm。已重新发布 /reports/rebar_instance_graph_probe 和 /reports/rebar_surface_bindpoint_comparison；当前重新抓取帧未检测到梁筋候选，因此 13cm mask pixels 为 0。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/tools/rebar_instance_graph_probe.py`
- `src/tie_robot_perception/tools/rebar_surface_bindpoint_comparison.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/rebar_instance_graph_probe`
- `src/tie_robot_web/web/reports/rebar_surface_bindpoint_comparison`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_perception/test -p 'test_pointai_scan_only_pr_fprg.py' -> 84 tests OK; python3 -m py_compile ... -> OK; git diff --check relevant paths -> OK; curl -I /reports/rebar_surface_bindpoint_comparison/index.html -> HTTP 200`

### 后续注意

- 暂无。

## 2026-04-30 13:09 - 前端索驱遥控服务分层

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端索驱遥控单点移动应调用驱动守护服务 /cabin/driver/raw_move（tie_robot_msgs/SingleMove）。/cabin/single_move 由 suoquNode 的 cabin_motion_controller 角色注册，属于高层单点服务，后端算法/控制角色未启动时不会存在；自动任务仍走 /web/cabin/* action 与后端执行链。

### 影响范围

- `src/tie_robot_web/frontend/src/config/topicRegistry.js;src/tie_robot_process/src/suoquNode.cpp`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `ros`
- `cabin`

### 验证证据

- `rosservice list 当前有 /cabin/driver/raw_move 和 /cabin/motion/stop；前端 cabin_remote 测试通过；npm run build 通过。`

### 后续注意

- 暂无。

## 2026-04-30 12:58 - TF全局X按现场物理方向取反

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户现场确认：物理索驱 X+ 在数字全局 map 中应表现为 X-。robot_tf_broadcaster 新增 cabin_to_map_sign 映射，默认 x=-1、y/z=+1；发布 map->base_link 和 base_to_camera 偏移时应用该符号。前端三维不单独镜像，只显示 TF。现场验证 cabin_state_X=601.290mm 时 /tf map->base_link.translation.x=-0.601290m。

### 影响范围

- `CHANGELOG.md`
- `src/tie_robot_perception/config/robot_home_tf.yaml`
- `src/tie_robot_perception/scripts/robot_tf_broadcaster.py`
- `src/tie_robot_perception/test/test_robot_tf_broadcaster.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_robot_tf_broadcaster src.tie_robot_perception.test.test_gripper_tf_broadcaster; targeted workspace_picker_web TF tests; python3 -m py_compile robot_tf_broadcaster.py; restarted tie-robot-rosbridge and driver services; rostopic /tf shows X negative`

### 后续注意

- 暂无。

## 2026-04-30 12:49 - 相机图层禁止消费 CameraInfo.D 畸变参数

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求：相机底层已经完成畸变处理，TF 层、视觉层和前端任何图层都不要使用 /Scepter/ir/camera_info 里的 D 畸变参数做二次畸变/去畸变。前端 TCP 工作区投影只读取 K/P、width/height 和 frame_id；pointAI camera_info_callback 不读取 msg.D，只保留零畸变系数；ScepterWorldCoordProcessor 只用 K 计算点云；TF 脚本不消费 CameraInfo。新增静态测试防止 cameraInfo?.D、msg.D、applyDistortion/undistort 回归。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/tcpWorkspaceOverlay.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/test/tcpWorkspaceOverlay.test.mjs`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/image_buffers.py`
- `src/tie_robot_perception/test/test_scepter_sdk_split.py`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/tcpWorkspaceOverlay.test.mjs; targeted workspace_picker_web tests; python3 -m unittest src.tie_robot_perception.test.test_scepter_sdk_split src.tie_robot_perception.test.test_robot_tf_broadcaster src.tie_robot_perception.test.test_gripper_tf_broadcaster; rg no D/distortion consumers; npm run build; frontend service active and index references index-D1TJyjcI.js`

### 后续注意

- 暂无。

## 2026-04-30 12:37 - 钢筋面补全后直接检测交点并保留曲线对照

<!-- AGENT-MEMORY: entry -->

### 摘要

- 根据用户反馈更新 rebar_surface_bindpoint_comparison：所有原图展示改用高伽马 IR（本轮报告 ir_display_gamma=1.95），避免原图钢筋看不清；新增 completed_surface_mask 和 completed_surface_response，用当前分割面 + line support 补全缺失钢筋面，然后以 completed_surface_intersections 作为优先技术路径，从补全面直接重建行/列线族并求交点。当前 raw_world 帧：补全面 line_counts [9,8]，completed_surface_intersections 72 点；3/4/5/6 改到补全面/多模态底图上做对照，分别约 64/65/65/65 点，适合作曲线收束候选而非当前主推荐。报告发布到 /reports/rebar_surface_bindpoint_comparison。

### 影响范围

- `src/tie_robot_perception/tools/rebar_surface_bindpoint_comparison.py`
- `src/tie_robot_web/web/reports/rebar_surface_bindpoint_comparison/index.html`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_perception/test -p 'test_pointai_scan_only_pr_fprg.py' (84 tests OK); python3 -m py_compile rebar_instance_graph_probe.py rebar_surface_bindpoint_comparison.py; git diff --check; curl -I http://192.168.6.99:8080/reports/rebar_surface_bindpoint_comparison/index.html; firefox headless mobile screenshot`

### 后续注意

- 暂无。

## 2026-04-30 12:34 - 红外虎口工作区投影不重复套畸变

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端红外图像上的 TCP/虎口工作区框仍使用 gripper_frame 下四角 (0,0,0)、(380,0,0)、(380,330,0)、(0,330,0) 投影，但现场 /Scepter/ir/image_raw 显示不应再把 /Scepter/ir/camera_info 的 D 畸变重复套到叠加层，否则四角会被径向畸变拽成漏斗形；projectCameraPointMetersToImagePixel 默认针孔投影，只有显式 applyDistortion:true 时才应用 plumb_bob 畸变。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/tcpWorkspaceOverlay.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/test/tcpWorkspaceOverlay.test.mjs`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `node src/tie_robot_web/frontend/test/tcpWorkspaceOverlay.test.mjs; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_ir_image_draws_live_tcp_workspace_boundary_overlay; npm run build; curl index references index-WLjxgA5U.js`

### 后续注意

- 暂无。

## 2026-04-30 12:22 - beam_candidate用于钢筋面分割与绑扎点对比

<!-- AGENT-MEMORY: entry -->

### 摘要

- 新增 src/tie_robot_perception/tools/rebar_surface_bindpoint_comparison.py，复用组合响应/深度响应/instance_graph 派生模态和新 beam_candidate，生成钢筋面分割与绑扎点检测对比报告。当前 live raw_world 帧：PR-FPRG 线族 [9,8]，原始 72 点；旧 legacy edge-band 本帧未检出梁筋所以仍 72 点；新 beam_candidate 检出左右两条宽纵梁筋 x=46..57 和 x=303..313。beam_candidate_direct 去掉梁筋本体上的点后剩 63 点，最符合“梁筋上的绑扎点不要识别，但梁筋附近普通钢筋交点尽量保留”的当前口径；beam_candidate_10cm 剩 45 点，更保守但会丢较多梁筋附近交点；instance_graph_junctions 当前 59 点，仅作探索对照。报告发布到 /reports/rebar_surface_bindpoint_comparison。

### 影响范围

- `src/tie_robot_perception/tools/rebar_surface_bindpoint_comparison.py`
- `src/tie_robot_web/web/reports/rebar_surface_bindpoint_comparison/index.html`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_perception/test -p 'test_pointai_scan_only_pr_fprg.py' (84 tests OK); python3 -m py_compile rebar_instance_graph_probe.py rebar_surface_bindpoint_comparison.py; git diff --check; curl -I http://192.168.6.99:8080/reports/rebar_surface_bindpoint_comparison/index.html; firefox headless mobile screenshot`

### 后续注意

- 暂无。

## 2026-04-30 12:11 - 红外图像叠加 TCP 虎口工作范围

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端红外图像现在订阅 /Scepter/ir/camera_info，并把 gripper_frame 下 TCP 虎口工作范围 x=0..380mm、y=0..330mm、z=0 的四角通过实时 TF 投影到红外画布，作为独立叠加层显示；该范围框与手动工作区选点互不影响。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/tcpWorkspaceOverlay.js`
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `ir-overlay`
- `tcp-workspace`

### 验证证据

- `node src/tie_robot_web/frontend/test/tcpWorkspaceOverlay.test.mjs; targeted unittest image/topic/overlay tests; node rosConnectionController/robotHomeCalibration tests; npm run build; git diff --check`

### 后续注意

- 暂无。

## 2026-04-30 12:03 - 归档PR-FPRG旧方案并新增响应衍生实例图探针

<!-- AGENT-MEMORY: entry -->

### 摘要

- 旧方案冻结到 docs/archive/pr_fprg_previous_schemes_2026-04-30.md：方案1行/列峰值组合响应为当前主链，方案2废案，方案3/4/5/6和FFT/曲线方案保留为对照，梁筋edge-band mask明确不是实例分割。新增 src/tie_robot_perception/tools/rebar_instance_graph_probe.py，独立抓取当前raw_world帧并从组合响应/深度响应衍生worldCoord高度、深度梯度、Hessian ridge、Frangi-like、多模态融合、骨架、instance_graph、新beam_candidate和旧edge-band对照；当前报告发布到 /reports/rebar_instance_graph_probe，当前帧72点，新beam_candidate抓到左右两条宽纵梁筋候选，旧edge-band只抓左侧。

### 影响范围

- `docs/archive/pr_fprg_previous_schemes_2026-04-30.md`
- `src/tie_robot_perception/tools/rebar_instance_graph_probe.py`
- `src/tie_robot_web/web/reports/rebar_instance_graph_probe/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_perception/test -p 'test_pointai_scan_only_pr_fprg.py' (83 tests OK); python3 -m py_compile src/tie_robot_perception/tools/rebar_instance_graph_probe.py; git diff --check; curl -I http://192.168.6.99:8080/reports/rebar_instance_graph_probe/index.html; firefox headless mobile screenshot`

### 后续注意

- 暂无。

## 2026-04-30 11:52 - 前端设置页新增网络 Ping 测试

<!-- AGENT-MEMORY: entry -->

### 摘要

- 设置卡片新增“网络测试”页，可手动输入索驱 IP（默认 192.168.6.62）和线性模组 IP（默认 192.168.6.167）并通过 /api/network/ping 调用本机 ping -c 3 -W 1 测试。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/styles/app.css`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `ping`
- `network`

### 验证证据

- `python3 -m unittest targeted tests; python3 -m py_compile workspace_picker_web_server.py; npm run build; curl /api/network/ping for 192.168.6.62 and 192.168.6.167`

### 后续注意

- 后端 workspace_picker_web_server.py 使用 normalize_ping_host 做安全校验并以参数数组执行 ping，避免 shell 注入；修改 frontend 后已重新构建 src/tie_robot_web/web 并重启 tie-robot-frontend.service。

## 2026-04-30 11:37 - 日志显示索驱和末端断链原因

<!-- AGENT-MEMORY: entry -->

### 摘要

- 现场确认索驱/末端并非 systemd service 未运行，而是状态 topic 无发布者且 /suoqu_driver_node、/moduan_driver_node 未注册到当前 ROS master；索驱最近错误为 XmlRpc/TCP 拒绝连接，末端最近错误为 PLC 未连接。system_log_mux 现在周期检查 /cabin/cabin_data_upload 与 /moduan/moduan_gesture_data 的 publisher、driver node 注册状态，并结合 stdout/journal 最近错误，向 /system_log/all 及 /system_log/suoqu_driver_node、/system_log/moduan_driver_node 发布“断链/恢复”原因日志；前端索驱/线性模组日志分类也改为真实 driver node topic。

### 影响范围

- `src/tie_robot_web/scripts/system_log_mux.py`
- `src/tie_robot_web/frontend/src/config/logTopicCatalog.js`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/test/test_system_log_mux.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_system_log_mux src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_toolbar_uses_theme_toggle_without_system_control_panel; node test/logText.test.mjs; python3 -m py_compile src/tie_robot_web/scripts/system_log_mux.py; npm run build; rostopic echo /system_log/suoqu_driver_node and /system_log/moduan_driver_node showed断链原因`

### 后续注意

- 暂无。

## 2026-04-30 11:28 - 钢筋实例分割方向：多模态BEV+脊线图优化优先

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户确认画面中有梁筋，并希望从梁筋mask阈值升级到钢筋/梁筋实例分割。已盘点本机视觉模态：color、transformedColor、IR、depth、transformedDepth、worldCoord/raw_world_coord、world_coord、depthCloudPoint 等；当前梁筋检测只是 workspace edge-band 宽强响应，不是实例分割，会漏内部/弱梁筋。调研文献与本机环境后推荐下一代方案：固定物理尺度BEV重采样 + IR/depth/worldCoord/点云多模态钢筋概率图 + Steger/Frangi/Hessian多尺度脊线中心线 + 3D高度/层级图优化实例分割；YOLO-seg/Mask R-CNN/U-Net作为有标注后的增强，SAM仅作标注辅助。本机无CUDA但有torch/ultralytics/onnxruntime/OpenCV ximgproc，深度模型CPU速度需实测。

### 影响范围

- `src/tie_robot_web/web/reports/rebar_instance_segmentation_study/index.html`
- `.debug_frames/rebar_instance_segmentation_modalities_20260430_112028`
- `.debug_frames/rebar_instance_segmentation_study_20260430_112742`

### 关键决策

- 见摘要。

### 验证证据

- `rostopic list/modalities inspected; export_visual_modalities_snapshot.py captured 16/25 topics; curl -I http://192.168.6.99:8080/reports/rebar_instance_segmentation_study/index.html returned 200; firefox headless mobile screenshot generated`

### 后续注意

- 暂无。

## 2026-04-30 11:23 - 重启 ROS 改为全栈停止再启动

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端“重启 ROS”按钮的 HTTP 入口 /api/system/restart_ros_stack 不再只是 restart tie-robot-backend.service；workspace_picker_web_server.py 会先用一次 systemctl stop 停 tie-robot-backend、三项驱动服务和 tie-robot-rosbridge，再按 rosbridge -> 驱动层 -> backend 顺序 start，保证驱动层和后端重新连接到上游 ROS master/rosbridge。backend-control sudoers 模板同步授权该受限全栈重启命令。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_bringup/systemd/tie-robot-backend-control.sudoers.in`
- `src/tie_robot_web/frontend/src/config/systemControlCatalog.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_restart_ros_stack_stops_everything_before_restarting_dependencies src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_ros_backend_is_systemd_managed_from_frontend src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_system_control_http_endpoints_cover_start_and_restart_actions; visudo -cf <substituted tie-robot-backend-control.sudoers.in>; npm run build`

### 后续注意

- 暂无。

## 2026-04-30 11:09 - PR-FPRG高伽马诊断报告与相机断流处理

<!-- AGENT-MEMORY: entry -->

### 摘要

- PR-FPRG方案对比报告新增 --display-gamma，仅改变报告可视化，不改变识别算法；报告会输出高伽马红外/底层响应/组合响应图，并加入梁筋mask当前状态、曲线跑偏原因、尺度影响原因、全尺度鲁棒方案和历史技术路径章节。本次生成高伽马报告前发现 /Scepter/ir/image_raw 与 /Scepter/depth/image_raw 无 publisher，tie-robot-driver-camera.service 进程仍在但 /scepter_manager 脱离 ROS master；已只重启相机驱动 service，图像流恢复约5Hz后生成报告。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_all_scheme_comparison/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_pr_fprg_scheme_comparison_reports_beam_filter_and_curve_metrics; python3 -m py_compile src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py; git diff --check -- src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; rostopic hz /Scepter/ir/image_raw; curl -I http://192.168.6.99:8080/reports/pr_fprg_all_scheme_comparison/index.html; firefox headless mobile screenshot`

### 后续注意

- 暂无。

## 2026-04-30 11:05 - 索驱底部位置条显示遥控可操作状态

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端底部中间的机器位置条现在订阅 /cabin/cabin_data_upload 判断索驱遥控是否可继续点击：cabin_connect_flag=1、无 device_alarm/internal_calc_error、motion_status=0、ROS 服务在线且有实时位置时为绿色可操作；运动中、连接断开、状态未上报或缺实时位置为红色不可操作；连接存在但 device_alarm/internal_calc_error 非零时为黄色报警。索驱方向按钮与同一可操作状态绑定，停止按钮在索驱已连接且 stop service 可用时仍可用。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/cabinRemoteOperationState.js`
- `src/tie_robot_web/frontend/test/cabinRemoteOperationState.test.mjs`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-DmQhO3Kx.js`
- `src/tie_robot_web/web/assets/app/index-CBNt9JQv.css`

### 关键决策

- 见摘要。

### 验证证据

- `node test/cabinRemoteOperationState.test.mjs; node test/cabinRemoteButtonSingleFire.test.mjs; node test/cabinRemoteKeyboard.test.mjs; node test/rosConnectionController.test.mjs; node test/logText.test.mjs; node test/visualDebugSettings.test.mjs; node test/robotHomeCalibration.test.mjs; git diff --check targeted frontend files; npm run build`

### 后续注意

- 暂无。

## 2026-04-30 10:53 - PR-FPRG报告图片视口内自适应

<!-- AGENT-MEMORY: entry -->

### 摘要

- PR-FPRG方案对比报告HTML已修复图片溢出：全局box-sizing、页面overflow-x hidden、grid使用minmax(min(100%,320px),1fr)，figure/img/pre/code设置min-width:0/max-width:100%，图片设置max-height:min(76vh,920px)和object-fit:contain。后续报告截图应先检查窄屏与桌面视口，避免大图撑出窗口。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_all_scheme_comparison/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_pr_fprg_scheme_comparison_reports_beam_filter_and_curve_metrics; python3 -m py_compile src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py; git diff --check -- src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; firefox --headless --window-size 390`
- `900 --screenshot ...; firefox --headless --window-size 1440`
- `1000 --screenshot ...`

### 后续注意

- 暂无。

## 2026-04-30 10:50 - 索驱遥控禁止连发

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求前端索驱遥控不要做连发：点一次只发送一次移动指令，否则索驱连接可能卡死。已将索驱遥控按钮从 pointerdown 长按连发改为 click 单发，并移除前端 App 内的连发 timer/状态文案。后续不要恢复 pressstart/pressend 或 CABIN_REMOTE_REPEAT 这类索驱连发机制。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/cabinRemoteButtonSingleFire.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-Db6pI8jb.js`

### 关键决策

- 见摘要。

### 验证证据

- `node test/cabinRemoteButtonSingleFire.test.mjs; node test/cabinRemoteKeyboard.test.mjs; node test/logText.test.mjs; node test/rosConnectionController.test.mjs; node test/visualDebugSettings.test.mjs; node test/robotHomeCalibration.test.mjs; npm run build; rg pressstart/pressend/连发/CABIN_REMOTE_REPEAT/cabinRemoteRepeat returned no matches`

### 后续注意

- 暂无。

## 2026-04-30 10:48 - Home点位与索驱map坐标系TF口径

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户确认 map 是索驱上位机/索驱全局坐标系，map.z=0 是索驱绝对零点而不是地面；实体只维护自身坐标系，跨坐标系投射统一走 TF。已新增 robot_home_tf.yaml 和 /web/tf/robot_home_calibration，robot_tf_broadcaster 现在发布 map->base_link 为索驱原始全局坐标，base_link->Scepter_depth_frame 为持久化机械外参，并将深度最远点作为相机坐标点经 TF 投射到 map 仅用于地面/距离估计显示。前端设置页新增 Home点位，可读取/保存/当前位置设为Home/一键回Home。

### 影响范围

- `src/tie_robot_perception/scripts/robot_tf_broadcaster.py`
- `src/tie_robot_perception/config/robot_home_tf.yaml`
- `src/tie_robot_msgs/srv/RobotHomeCalibration.srv`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_robot_tf_broadcaster.py; python3 src/tie_robot_perception/test/test_gripper_tf_broadcaster.py; python3 src/tie_robot_process/test/test_tf_coordinate_contract.py; node frontend robotHome/rosConnection/visualDebug/cabinRemoteKeyboard/logText tests; npm run build; catkin_make`

### 后续注意

- 暂无。

## 2026-04-30 10:36 - PR-FPRG报告固定展示FFT拓扑对照与梁筋mask状态

<!-- AGENT-MEMORY: entry -->

### 摘要

- PR-FPRG方案对比报告现在保留方案3/4/5/6主链行列峰值拓扑，同时新增03F/04F/05F/06F作为同算法换FFT线族拓扑骨架的对照；每次报告必须包含FFT直角坐标驼峰峰值图(period_px->FFT power)、FFT候选线/剔除线、组合响应梁筋±10cm叠加图和梁筋mask二值图。若summary beam_bands为空，表示本帧未触发梁筋过滤，二值图会显示NO BEAM MASK DETECTED；不要误判为mask被隐藏。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_all_scheme_comparison/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg.PointAIScanOnlyPrFrpgTest.test_pr_fprg_scheme_comparison_reports_beam_filter_and_curve_metrics; python3 -m py_compile src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py; git diff --check -- src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; curl -I http://192.168.6.99:8080/reports/pr_fprg_all_scheme_comparison/index.html`

### 后续注意

- 暂无。

## 2026-04-30 09:37 - PR-FPRG报告固定展示梁筋过滤和FFT频域图

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求后续每次方案对比报告都把梁筋过滤标在组合响应图中，并给出频域图。pr_fprg_scheme_comparison 现在输出 00_combined_response_beam_mask_overlay.png（组合响应+梁筋±10cm mask红色叠加并带白色轮廓）和 00_fft_frequency_spectrum.png（两组轴向 profile 的 FFT 频域谱图）。报告明确标注方案3/4/5/6 使用主链行/列峰值线族作为曲线拓扑骨架，不使用 FFT 线族；01B 才是 FFT 轴向峰值对照。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_all_scheme_comparison`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest targeted PR-FPRG report tests; python3 -m py_compile pr_fprg_scheme_comparison.py; curl -I http://192.168.6.99:8080/reports/pr_fprg_all_scheme_comparison/index.html`

### 后续注意

- 暂无。

## 2026-04-30 09:26 - PR-FPRG 全方案对比加入FFT行列峰值分支

<!-- AGENT-MEMORY: entry -->

### 摘要

- 已在方案对比报告中加入 01B FFT 行/列峰值正交网格，作为方案1的频域对照而非复活方案2；报告固定输出普通峰值图和 FFT 峰值图，方案2仍保持废案不进入对比。2026-04-30 09:24 live 帧报告发布到 /reports/pr_fprg_all_scheme_comparison，主链单帧约80ms，本帧各方案均36点。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_web/web/reports/pr_fprg_all_scheme_comparison`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg; python3 -m py_compile workspace_s2.py manual_workspace_s2.py processor.py pr_fprg_scheme_comparison.py; curl -I http://192.168.6.99:8080/reports/pr_fprg_all_scheme_comparison/index.html`

### 后续注意

- 暂无。

## 2026-04-30 09:19 - Codex超大会话改为原文归档加活跃摘要替身

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户修正最新口径：超过 100MB 的 Codex 会话不再只写旁路摘要，而是把原始完整 JSONL 移入 ~/.codex/archived_sessions/oversized，并在 ~/.codex/sessions 原路径写入小型摘要 JSONL，让存活会话列表可快速打开并看到上下文重点；同时保留 ~/.codex/session_summaries/oversized Markdown 摘要。已重装 tie-codex-session-summary.timer，ExecStart 使用 summarize --threshold-mb 100 --min-age-minutes 10 --skip-open --apply；旧 tie-codex-session-guard.timer 仍保持 archive-only 禁用。已手动压缩 2026-04-29T01-41 约 201MB 旧会话，活跃原路径变为约 30KB 摘要替身，原文在 archived_sessions。

### 影响范围

- `scripts/codex_session_guard.py`
- `scripts/install_codex_session_summary_timer.sh`
- `scripts/install_codex_session_guard_timer.sh`
- `src/tie_robot_bringup/test/test_codex_session_guard.py`
- `AGENTS.md`
- `CHANGELOG.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/organism.md`
- `docs/agent_memory/checkpoint.md`
- `scripts/agent_memory.py`

### 关键决策

- summarize 子命令语义改为原文归档 + 活跃摘要替身；定时器必须带 --skip-open，避免处理当前打开会话。

### 标签

- `codex-session`
- `agent-memory`
- `long-context`

### 验证证据

- `python3 -m unittest src.tie_robot_bringup.test.test_codex_session_guard src.tie_robot_bringup.test.test_agent_memory_contract -v; python3 -m py_compile scripts/codex_session_guard.py scripts/agent_memory.py; bash -n scripts/install_codex_session_summary_timer.sh scripts/install_codex_session_guard_timer.sh; systemctl --user cat tie-codex-session-summary.service tie-codex-session-summary.timer; python3 scripts/codex_session_guard.py scan --threshold-mb 100`

### 后续注意

- 50MB 到 100MB 之间的旧会话暂不自动压缩；当前还有一个约 75MB 活跃 JSONL，符合 100MB 阈值规则所以保留。

## 2026-04-30 09:03 - Codex超大会话改为自动总结不归档

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求当 Codex 会话超过 100MB 时自动总结上下文重点，但不要删除、移动或自动归档会话。codex_session_guard.py 新增 summarize 子命令，摘要写入 ~/.codex/session_summaries/oversized，源 JSONL 保留在 ~/.codex/sessions；已安装并启用 tie-codex-session-summary.timer（每 15 分钟，阈值 100MB，min-age 10 分钟）。旧 install_codex_session_guard_timer.sh 默认拒绝启用自动归档，除非用户明确设置 ALLOW_CODEX_SESSION_ARCHIVE_TIMER=1。

### 影响范围

- `scripts/codex_session_guard.py`
- `scripts/install_codex_session_summary_timer.sh`
- `scripts/install_codex_session_guard_timer.sh`
- `src/tie_robot_bringup/test/test_codex_session_guard.py`
- `AGENTS.md`
- `CHANGELOG.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/organism.md`
- `scripts/agent_memory.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_bringup.test.test_codex_session_guard src.tie_robot_bringup.test.test_agent_memory_contract -v; python3 -m py_compile scripts/codex_session_guard.py scripts/agent_memory.py; bash -n scripts/install_codex_session_summary_timer.sh scripts/install_codex_session_guard_timer.sh; systemctl --user is-active tie-codex-session-summary.timer`

### 后续注意

- 暂无。

## 2026-04-30 08:50 - 用户禁用Codex超大会话自动归档

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求不要自动存档超大 Codex 会话，并要求恢复已归档会话。已停止、disable 并移除用户级 tie-codex-session-guard.timer/service 文件，daemon-reload 后 timer/service 均 inactive；已将 ~/.codex/archived_sessions/oversized 下 5 个 JSONL 恢复到 ~/.codex/sessions 原路径。后续 agent 不要重新安装或启用 tie-codex-session-guard.timer，除非用户明确改口。

### 影响范围

- `docs/agent_memory/current.md`
- `docs/agent_memory/session_log.md`

### 关键决策

- 见摘要。

### 验证证据

- `systemctl --user is-active tie-codex-session-guard.timer tie-codex-session-guard.service; find ~/.codex/archived_sessions/oversized -type f -name '*.jsonl'; find ~/.codex/sessions -type f -name '*.jsonl' -size +50M`

### 后续注意

- 暂无。

## 2026-04-29 16:57 - slam/v30 发布包融入 PR-FPRG 实验总结

<!-- AGENT-MEMORY: entry -->

### 摘要

- 已把 2026-04-29 本会话 PR-FPRG 实验总结写入 docs/releases/slam_v30/SLAM_V30_HANDOFF.md 与 MANIFEST.md；slam_v30_visual_modalities.bag 追加 /release/slam_v30/experiment_summary（std_msgs/String）用于离线回放时查看最终视觉决策；metadata 改为 20/24 个话题；checksums.sha256、debug_frames_manifest.tsv、/home/hyq-/simple_lashingrobot_ws_slam_v30.zip 与 .zip.sha256 均已刷新。最终主链结论：方案1=组合响应 + rectified 行/列峰值正交网格；方案2废弃；方案3-6仅后续曲线收束对照；梁筋只点级排除 ±100mm；报告必须包含 peak-supported 与 spacing-pruned 峰值图。

### 影响范围

- `docs/releases/slam_v30/SLAM_V30_HANDOFF.md`
- `docs/releases/slam_v30/MANIFEST.md`
- `docs/releases/slam_v30/visual_modalities/slam_v30_visual_modalities.bag`
- `docs/releases/slam_v30/visual_modalities/messages/release_slam_v30_experiment_summary.txt`
- `/home/hyq-/simple_lashingrobot_ws_slam_v30.zip`
- `/home/hyq-/simple_lashingrobot_ws_slam_v30.zip.sha256`

### 关键决策

- 见摘要。

### 验证证据

- `sha256sum -c docs/releases/slam_v30/checksums.sha256; sha256sum -c /home/hyq-/simple_lashingrobot_ws_slam_v30.zip.sha256; zip -T /home/hyq-/simple_lashingrobot_ws_slam_v30.zip; rosbag info shows /release/slam_v30/experiment_summary; git diff --check`

### 后续注意

- 暂无。

## 2026-04-29 16:41 - PR-FPRG 方案1回到行列峰值正交主链

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求方案1不要再输出斜线：当前 pointAI 手动工作区 S2 主链改为 combined_depth_ir_darkline 优先，透视展开后固定在 rectified 图行/列 profile 上找峰值，输出 0/90 度正交网格；不再估计 theta，不再把方向自适应 theta/rho 作为主链。方案2继续废案，曲线方案3-6仅作为后续收束/改进对照。报告必须包含 00_peak_supported_lines.png 和 00_peak_spacing_pruned_lines.png 峰值图。梁筋仍只做最终点级/图谱排除，钢筋线允许穿过梁筋。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile ...; git diff --check; curl -I http://192.168.6.99:8080/reports/pr_fprg_axis_rowcol_scheme_comparison/index.html`

### 后续注意

- 暂无。

## 2026-04-29 16:13 - PR-FPRG 报告必须包含峰值图

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求以后每次生成 PR-FPRG 相关报告都要给峰值图。已将方案对比报告、阶段消融报告、全方案消融报告固定输出两张峰值诊断图：00_peak_supported_lines.png 表示 peak-supported 候选线，00_peak_spacing_pruned_lines.png 表示绿色保留/红色剔除。当前三个发布入口 /reports/pr_fprg_dense_42_scheme_comparison、/reports/pr_fprg_dense_42_all_scheme_ablation、/reports/pr_fprg_stage_ablation 都已刷新并包含峰值图。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation_report.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_dense_42_scheme_comparison`
- `src/tie_robot_web/web/reports/pr_fprg_dense_42_all_scheme_ablation`
- `src/tie_robot_web/web/reports/pr_fprg_stage_ablation`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m py_compile scheme_comparison/stage_ablation_report/scheme_ablation_report -> exit 0; python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> 78 tests OK; git diff --check -> exit 0; three report URLs -> HTTP 200; verified peak image files exist and were viewable`

### 后续注意

- 暂无。

## 2026-04-29 15:58 - PR-FPRG 当前尺度密集线族至少42点

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户要求当前尺度至少识别42个绑扎点，并明确方案1不要加入全角度候选。已将方案1规则晶格目标从旧4x4稀疏链调整为当前尺度密集链：PREFERRED_WORKSPACE_S2_LATTICE_LINE_COUNT=8，WORKSPACE_S2_SCORE_TARGET_MIN_POINTS=42，regular lattice max_line_count=10；方案1仍用组合响应 combined_depth_ir_darkline 和方向先验候选，不使用 full_angle_sweep。方案2继续废案不参与。方案3/4/5/6重新进入全方案报告作为曲线收束对照。最新当前帧验证：scheme comparison 方案1=47点、方案4/5/6=48点；all-scheme ablation 全流程各方案=51点，profile_only_spacing=42-43点，profile_only_raw=352+点爆点。当前推荐仍以方案1全流程为主链，方案5 ridge曲线作为后续收束候选。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation_report.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_dense_42_scheme_comparison`
- `src/tie_robot_web/web/reports/pr_fprg_dense_42_all_scheme_ablation`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> 78 tests OK; py_compile workspace_s2/stage_ablation/stage_ablation_report/scheme_ablation_report/scheme_comparison -> exit 0; git diff --check -> exit 0; curl reports/pr_fprg_dense_42_scheme_comparison and reports/pr_fprg_dense_42_all_scheme_ablation -> HTTP 200`

### 后续注意

- 暂无。

## 2026-04-29 15:29 - PR-FPRG 方案1组合响应消融与梁筋图谱排除

<!-- AGENT-MEMORY: entry -->

### 摘要

- 当前主链固定为方案1：组合响应 combined_depth_ir_darkline + peak-supported theta/rho 直线线族。方案2已作为废案从当前方案对比与消融报告移除。梁筋出现时只在最终点位/图谱阶段做排除，排除范围为梁筋 mask 按真实尺度扩张 +-100mm；钢筋线检测仍允许穿过梁筋。最新消融报告发布到 /reports/pr_fprg_stage_ablation/index.html，锁定组合响应，结果显示 raw profile 峰值很多但会爆出大量误线，连续/ridge/间距等收敛步骤不能直接删除；full_angle_sweep 与 full 点数一致，可作为可跳过项。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `.debug_frames/pr_fprg_scheme1_stage_ablation_combo_20260429_152439`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> Ran 78 tests OK; python3 -m py_compile workspace_s2/manual_workspace_s2/processor/scheme_comparison/peak_supported_probe/stage_ablation/scheme_ablation_report -> exit 0; git diff --check -> exit 0; curl -I http://192.168.6.99:8080/reports/pr_fprg_stage_ablation/index.html -> HTTP 200`

### 后续注意

- 暂无。

## 2026-04-29 15:07 - PR-FPRG 组合响应峰值图与深度对照报告

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户确认主方案继续使用 combined_depth_ir_darkline 组合响应，而不是纯深度响应。已给 pr_fprg_scheme_comparison 增加 --response-name-filter，便于单独生成 depth_background_minus_filled 深度对照报告；默认不带该参数仍优先组合响应。梁筋 mask 判据调整为 high_response_ratio=0.40、abnormal_peak_ratio=1.0、abnormal_width_ratio=1.1，并改为基于当前选中的响应图做梁筋检测。新增 /reports/pr_fprg_combined_peaks 峰值可视化：细竖线/细线是 peak-supported 候选，大点/粗线是最终 spacing-pruned 保留峰值。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_combined_peaks`
- `src/tie_robot_web/web/reports/pr_fprg_depth_response`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> 77 tests OK; py_compile workspace_s2/manual_workspace_s2/pr_fprg_scheme_comparison/pr_fprg_peak_supported_probe OK; curl -I http://192.168.6.99:8080/reports/pr_fprg_combined_peaks/index.html -> 200`

### 后续注意

- 暂无。

## 2026-04-29 14:36 - PR-FPRG 底层响应图统一为组合响应

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求以后底层图像都用当前 depth+IR 组合响应表示。已将方案对比报告的底层响应图输出改为 00_selected_response.png，页面 caption 为“底层组合响应”；pr_fprg_peak_supported_probe 的步骤图从 04_depth_response 改为 04_selected_combined_response。scheme_comparison 在 response_source=depth_ir 时不再对组合响应二次混合，06 方案说明改成沿当前底层组合响应做同拓扑曲线收束。当前已发布报告保持用户认可的 19 点效果，只更新图像命名与说明。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_curve_3456`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py; curl -I http://127.0.0.1:8080/reports/pr_fprg_curve_3456/index.html`

### 后续注意

- 暂无。

## 2026-04-29 14:33 - PR-FPRG 方案1改为组合响应找峰值

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户认可 2026-04-29 14:28 的组合响应方案1效果，并明确要求先不要纠结理论交点 20 个，保持这版 19 点视觉效果。已让 manual_workspace_s2 运行主链先尝试 combined_depth_ir_darkline（depth_background_minus_filled 与红外暗线 0.68/0.32 组合）来做方案1 theta/rho 与 peak 支撑找线，失败才回落纯深度/红外；pr_fprg_peak_supported_probe 同步使用组合响应优先，报告 /reports/pr_fprg_curve_3456 已更新。注意：不要再为了补齐第20个边界理论交点而放宽最终 workspace mask；用户当前要保留这版效果。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_curve_3456`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py; curl -I http://127.0.0.1:8080/reports/pr_fprg_curve_3456/index.html`

### 后续注意

- 暂无。

## 2026-04-29 14:19 - PR-FPRG 六点基准溯源与评分恢复

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户指出当前 PR-FPRG 识别效果被破坏并要求必要时回滚到 2026-04-29 06:00。溯源发现没有 06:00 git commit；可用基准来自 .debug_frames/pr_fprg_peak_supported_probe_20260429_060428 与 061217，特点是 depth_background_minus_filled 响应、3x4 稀疏主筋格。当前坏图根因是 workspace_s2 评分把 target_min_points=40 和密度奖励压过真实主筋，选成 6x6 地板缝/密集候选；另一个问题是 pr_fprg_peak_supported_probe 报告工具仍按分数挑深度响应，会回到 06:30 的 depth_filled_minus_background 反相响应。已将晶格/线族评分恢复为 06:00 风格的稀疏优先，同时保留连续支撑分数以免真实强网格被误删；报告探针改为第一个可用 depth_background_minus_filled 优先。新报告 .debug_frames/pr_fprg_curve_3456_0600_response_recovery_20260429_141702 已发布到 /reports/pr_fprg_curve_3456，当前结果 depth_background_minus_filled，方案1 18点，方案5 17点。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `src/tie_robot_web/web/reports/pr_fprg_curve_3456`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py; curl -I http://127.0.0.1:8080/reports/pr_fprg_curve_3456/index.html`

### 后续注意

- 暂无。

## 2026-04-29 14:01 - PR-FPRG 3-6曲线避梁筋/地板缝探究报告

<!-- AGENT-MEMORY: entry -->

### 摘要

- 按用户要求独立探究方案3/4/5/6避开梁筋和地板缝，不改运行时方案1主链。新增/增强 pr_fprg_scheme_comparison.py 的独立报告诊断：梁筋只做最终点级过滤，报告 beam_filtered_point_count；曲线抗地板缝以 curve_metrics 量化，包括 coverage_mean、score_mean、abs_offset_mean、abs_offset_p95、wiggle_mean。当前尺度报告 .debug_frames/pr_fprg_curve_3456_beam_floor_probe_20260429_135902，已发布到 /reports/pr_fprg_curve_3456。当前结果：3/4/5/6 均为 6x6 线族、31-32 点、beam_filtered=0；03 greedy 覆盖高但 wiggle_mean=1.148，容易追地板纹理；04 DP wiggle_mean=0.061 但偏移均值较大；05 DP+ridge wiggle_mean=0.067 且 abs_offset_mean=3.699，作为下一步主候选；06 IR-assisted wiggle_mean=0.067、score_mean较高但可能被红外地板缝牵引，作为辅助验证候选。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `.debug_frames/pr_fprg_curve_3456_beam_floor_probe_20260429_135902/index.html`
- `src/tie_robot_web/web/reports/pr_fprg_curve_3456/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py => Ran 71 tests OK; python3 -m py_compile pr_fprg_scheme_comparison.py OK; git diff --check touched files OK; curl /reports/pr_fprg_curve_3456/index.html and 05 image => HTTP 200`

### 后续注意

- 暂无。

## 2026-04-29 13:59 - Xpra 主窗口无边框全屏从代理层 patch

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-29 验证：rviz/rqt 等图形卡片应通过 workspace_picker_web_server.py 的 _patch_xpra_html5_resource 修补 Xpra HTML5 /js/Window.js 与 /css/client.css，使主窗口标记 tie-robot-embedded-main-window、隐藏内部 windowhead、初始 maximized 铺满 iframe；不要在 UIController 中直接改 frame.contentWindow.client/id_to_window/canvas，否则容易导致蓝屏或刷新。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py;src/tie_robot_web/frontend/src/ui/UIController.js`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `xpra`
- `gui`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards；服务重启后通过真实 Firefox 截图验证 rqt 无内部 Default-rqt 标题栏且内容铺满卡片。`

### 后续注意

- 暂无。

## 2026-04-29 13:55 - 图形应用嵌入窗口去边框补丁

<!-- AGENT-MEMORY: entry -->

### 摘要

- workspace_picker_web_server.py 会在代理 xpra HTML/JS 资源时 patch /js/Window.js 和 /css/client.css：普通 NORMAL 主窗口在前端图形应用卡片内强制最大化、隐藏 windowhead、去边框，DIALOG/UTILITY/TOOLTIP 等窗口不套用。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py;src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 图形应用嵌入样式由 Web server 动态 patch 第三方 xpra 前端资源，不直接改 vendor 文件。

### 标签

- `frontend`
- `graphical-app`
- `xpra`
- `slam-v30`

### 验证证据

- `python3 src/tie_robot_web/test/test_workspace_picker_web.py OK；git diff --check 对相关文件无输出。`

### 后续注意

- 暂无。

## 2026-04-29 13:55 - 线性模组执行抽象为状态Topic+Action

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户已确认当前工程采用 ROS 风格分层但不强行引入 ros_control/MoveIt：moduan_driver_node 负责读 PLC 并发布 /moduan/state；moduan_motion_controller_node 提供 /moduan/execute_bind_points Action，内部写点位、发执行信号并等待 FINISHALL；/moduan/sg、/moduan/sg_precomputed、/moduan/single_move 继续作为兼容 wrapper；tie_robot_process 预计算绑扎点执行链现在通过 /moduan/execute_bind_points Action 调度，不再直接调用 /moduan/sg_precomputed* 或感知 PLC 完成位。FINISHALL 仍是 PLC 完成的权威信号，但只属于控制层实现细节。

### 影响范围

- `docs/superpowers/plans/2026-04-29-moduan-action-state-architecture.md`
- `src/tie_robot_msgs/msg/ModuanState.msg`
- `src/tie_robot_msgs/action/ExecuteBindPointsTask.action`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard; python3 -m unittest src.tie_robot_bringup.test.test_ros_interface_names src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_scan_artifact_write_guard src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_single_point_bind_button_calls_atomic_backend_service; python3 scripts/check_ros_interface_names.py; git diff --check -- docs/superpowers/plans/2026-04-29-moduan-action-state-architecture.md docs/architecture/ros_interface_migration_map.yaml src/tie_robot_msgs src/tie_robot_control src/tie_robot_process; catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j2`

### 后续注意

- 暂无。

## 2026-04-29 13:55 - suoquNode 改走 execute_bind_points action

<!-- AGENT-MEMORY: entry -->

### 摘要

- slam/v30 最终提交中，suoquNode 到线性模组绑扎执行的调度路径改为 actionlib SimpleActionClient 调用 /moduan/execute_bind_points，旧 /moduan/sg_precomputed 与 /moduan/sg_precomputed_fast service 调用不再作为流程编排主路径。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp;src/tie_robot_process/CMakeLists.txt;src/tie_robot_process/package.xml;CHANGELOG.md;docs/releases/slam_v30/SLAM_V30_HANDOFF.md`

### 关键决策

- 算法/流程层通过 action 调度完整绑扎执行，驱动层继续保留原子 Modbus/PLC 操作。

### 标签

- `motion-chain`
- `moduan`
- `actionlib`
- `slam-v30`

### 验证证据

- `python3 src/tie_robot_process/test/test_motion_chain_signal_guard.py OK；python3 src/tie_robot_bringup/test/test_ros_interface_names.py OK；catkin_make exit 0。`

### 后续注意

- 暂无。

## 2026-04-29 13:47 - PR-FPRG 方案1恢复连续/ridge主链

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户指出当前尺度效果奇差，根因定位为此前为追求方案1 only 和点数，把运行主链改成 enable_continuous_validation=False，退化为只靠一维 peak/spacing，导致地板缝和边缘细峰在当前尺度下爆出密集假线。已恢复方案1主链口径：depth_background_minus_filled 优先，use_orientation_prior_angle_pool=True，enable_local_peak_refine=True，enable_continuous_validation=True，enable_spacing_prune=True；梁筋仍只做最终点级排除，不删除整条 rho/线族。新增规则 lattice 收束用于 dense floor-seam 候选，避免 peak 候选过密。最新当前尺度报告 .debug_frames/pr_fprg_scheme1_restored_mainline_current_scale_20260429_134514，已发布到 /reports/pr_fprg_scheme1_current_scale、/reports/pr_fprg_scheme1、/reports/pr_fprg_live_full。注意：效果从 54 点密集假线收敛到 36 点，但耗时约 1.28s，仍需后续继续优化质量与速度，不能宣称最终完成。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `.debug_frames/pr_fprg_scheme1_restored_mainline_current_scale_20260429_134514/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py => Ran 70 tests OK; python3 -m py_compile touched scripts OK; git diff --check touched files OK; current-scale report generated at 2026-04-29 13:45:36 raw_world depth_background_minus_filled full points=36 mean=1280.94ms median=1228.98ms; curl report/image => HTTP 200`

### 后续注意

- 暂无。

## 2026-04-29 13:44 - Xpra 图形卡片不要触碰内部 client

<!-- AGENT-MEMORY: entry -->

### 摘要

- rqt/rviz 前端蓝屏和 Xpra 断连页的根因不是 Xpra 技术栈本身：同一 session 在最小 iframe 中可正常显示；完整前端失败来自 UIController 在 iframe 加载/resize 时进入 Xpra HTML5 内部触发 resize、redraw、request_refresh 并清内部 DOM 样式，容易打断握手或遮断绘制。后续图形卡片只管理外层卡片和 iframe 宽高，不再调用 frame.contentWindow.client、redraw_windows、request_refresh，也不再修改 Xpra 内部 window/canvas 样式。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-BKRosfJs.js`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build; systemctl restart tie-robot-frontend.service; real Firefox/Xpra screenshot verified rqt Topic Monitor and Image View render inside frontend card without blue screen`

### 后续注意

- 暂无。

## 2026-04-29 13:40 - slam/v30 离线复现发布包

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-29 当前 slam 分支发布为 slam/v30：新增 docs/releases/slam_v30 交接目录、slam_v30_visual_modalities.bag 小型视觉样例、PNG/NPY/modal metadata、debug_frames_manifest.tsv，以及 slam_v30_offline_visual_replay.launch。完整 .debug_frames 实验输出不进入 Git 历史，随整工程 zip 打包。

### 影响范围

- `docs/releases/slam_v30/SLAM_V30_HANDOFF.md;docs/releases/slam_v30/visual_modalities/metadata.json;src/tie_robot_bringup/launch/slam_v30_offline_visual_replay.launch;src/tie_robot_perception/tools/export_visual_modalities_snapshot.py`

### 关键决策

- Git tag 保留源码、文档和小型离线视觉包；722MB .debug_frames 实验目录通过 zip 打包，并在 Git 中保留 debug_frames_manifest.tsv 索引。

### 标签

- `release`
- `slam-v30`
- `offline-replay`
- `vision`
- `agent-memory`

### 验证证据

- `已通过 export_visual_modalities_snapshot.py 导出 19/23 个话题；rosbag info 显示 bag 含 Scepter 图像、worldCoord、pointAI/result_image_raw、perception/lashing 结果、TF 与状态。`

### 后续注意

- 暂无。

## 2026-04-29 12:33 - PR-FPRG 当前只启用方案1直线族

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户已改为先只做方案1：当前视觉主链采用方案1 theta/rho 直线族，在当前尺度下进行钢筋绑扎点识别；方案2不要，曲线方案3-6暂存档只作为后续收束/改进参考，不进入当前主链或主报告。梁筋处理口径：钢筋可以穿过梁筋，不删除整条 rho/线族；梁筋只用于最终绑扎点的点级排除，IR 只辅助结构梁筋掩膜，主线响应锁定 depth_background_minus_filled。最新方案1报告为 .debug_frames/pr_fprg_scheme1_beam_excluded_20260429_1315/index.html，并已发布到 /reports/pr_fprg_scheme1、/reports/pr_fprg_robustness、/reports/pr_fprg_live_full。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `.debug_frames/pr_fprg_scheme1_beam_excluded_20260429_1315/index.html`
- `.debug_frames/pr_fprg_robustness_temp_site/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py => Ran 69 tests OK; python3 -m py_compile for touched perception/report scripts OK; git diff --check for touched files OK; curl -I http://127.0.0.1:8080/reports/pr_fprg_scheme1/index.html and robustness/live_full reports => HTTP 200`

### 后续注意

- 暂无。

## 2026-04-29 12:24 - PR-FPRG 梁筋只做最终点级过滤

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户纠正：普通钢筋可以穿过梁筋，不能因为梁筋 mask 删除整条 rho 线或阻断线族；梁筋只用于最终绑扎点排除。当前 3-6 方案报告废弃 1/2，主线回到 08:21 风格：深度响应 depth_background_minus_filled 优先，红外只用于纵向梁筋结构 mask，profile/spacing 拓扑 + 曲线追踪；连续验证在当前现场图会漏真实线，只作为对照。最新报告 .debug_frames/pr_fprg_scheme_ablation_report_curve_depth_irbeam_cleanfull_20260429_1300，已发布到 /reports/pr_fprg_live_full 和 /reports/pr_fprg_robustness。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile workspace_s2/manual_workspace_s2/pr_fprg tools; curl -I /reports/pr_fprg_live_full/index.html`

### 后续注意

- 暂无。

## 2026-04-29 12:10 - FINISHALL是线性模组执行完成权威信号

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户澄清：发执行信号 -> 等 FINISHALL 这段由 PLC 执行，PLC 结束后给 FINISHALL，因此它本身可替代服务入口处人为置 /moduan_work 忙。当前实现已去掉 moduan 服务入口的 ScopedModuanWorkState；/moduan_work 只在 execute_bind_points 的实际 PLC 执行段镜像状态：点位写入完成后进入 ScopedPlcExecutionState，随后 pulseExecutionEnable 发执行信号并等待 finish_all(150)，返回/超时后自动发布 false。视觉识别阶段不再发布 moduan_work=true。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_scan_artifact_write_guard src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_single_point_bind_button_calls_atomic_backend_service src.tie_robot_bringup.test.test_ros_interface_names; python3 scripts/check_ros_interface_names.py; git diff --check -- changed motion-chain files; catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j2`

### 后续注意

- 暂无。

## 2026-04-29 12:04 - Xpra 图形卡片蓝屏修复

<!-- AGENT-MEMORY: entry -->

### 摘要

- rqt/rviz 原生 X11 窗口内容正常；前端蓝屏来自 UIController 对 Xpra 内部 window div/canvas 做 CSS 缩放、位移和 #071b33 背景，遮断了 Xpra HTML5 客户端的实际绘制。后续图形卡片只管理外层卡片大小、置顶和 redraw，不再 transform Xpra 内部窗口。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js; src/tie_robot_web/test/test_workspace_picker_web.py; src/tie_robot_web/web/index.html; src/tie_robot_web/web/assets/app/index-DFx4VCva.js`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `xpra`
- `gui`
- `rqt`
- `rviz`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build; systemctl restart tie-robot-frontend.service; POST /api/gui/sessions rqt -> ready; DELETE session -> sessions empty/no xpra rqt rviz processes`

### 后续注意

- 暂无。

## 2026-04-29 12:03 - 单点绑扎收口为后端原子链

<!-- AGENT-MEMORY: entry -->

### 摘要

- 单点绑扎相对全局绑扎按原子处理：前端按钮只调用 singlePointBindService（后端 /moduan/sg），不再先在前端拆出一次视觉识别；/moduan/sg 内部按置 /moduan_work 忙 -> 调视觉 PR-FPRG/process_image -> execute_bind_points 触发线性模组并等待 FINISHALL 的顺序执行。索驱移动入口 move_cabin_pose_via_driver 在 /moduan_work 为 true 时拒绝下发位姿，避免线性模组/末端绑扎中索驱并发移动。

### 影响范围

- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- `src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_bringup.test.test_ros_interface_names src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_scan_artifact_write_guard src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_single_point_bind_button_calls_atomic_backend_service; python3 scripts/check_ros_interface_names.py; node --check frontend TaskActionController/TieRobotFrontApp; npm run build; catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j2`

### 后续注意

- 暂无。

## 2026-04-29 11:53 - 绑扎/索驱互锁沿用FINISHALL信号链

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-29 对照只读旧工程 /home/hyq-/lashingrobotROS：旧版 /moduan/sg 会写入视觉点位、拉起 EN_DISABLE 执行信号、在服务内部 finish_all(150) 等 FINISHALL 置位，随后清 FINISHALL；索驱执行层同步 call 该服务时不会在末端执行未完成前继续移动。当前工程应保留这个模型，用 /moduan_work 表示线性模组/绑扎忙，索驱移动入口检查该信号拒绝并发移动，避免新增不必要的单点动作链。

### 影响范围

- `src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp;src/tie_robot_process/src/suoqu/cabin_transport.cpp;src/tie_robot_process/src/suoquNode.cpp`

### 关键决策

- 见摘要。

### 标签

- `lashing`
- `moduan`
- `cabin`
- `safety`
- `finall`

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_scan_artifact_write_guard src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_single_point_bind_button_runs_pr_fprg_before_bind_service src.tie_robot_bringup.test.test_ros_interface_names; python3 scripts/check_ros_interface_names.py; catkin_make -DCATKIN_WHITELIST_PACKAGES="" -j2`

### 后续注意

- 暂无。

## 2026-04-29 11:48 - RViz 最大化改为完整窗口 contain 缩放

<!-- AGENT-MEMORY: entry -->

### 摘要

- RViz 原生 Xpra 窗口正常时，如果前端用 CSS cover 缩放（scale=max(scaleX,scaleY)）会裁掉菜单栏、Displays/Tool Properties 等，只剩中间 3D 蓝色视图区，看起来像蓝屏。图形卡片最大化应使用 contain 缩放（scale=min(scaleX,scaleY)）并居中，优先保证完整 Ubuntu/RViz 窗口可见，不再裁剪 UI。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-CXjz3iV1.js`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build; systemctl status tie-robot-frontend.service; /api/gui/sessions [] after cleanup; Xpra native xwininfo showed RViz 1200x846 with full child panels`

### 后续注意

- 暂无。

## 2026-04-29 11:43 - PR-FPRG 梁筋只按纵向边缘结构识别

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户确认梁筋只有纵向，顶部横向强响应是 TCP/末端侵入，不应标为梁筋。当前 detect_workspace_s2_structural_edge_bands 默认 allowed_axes=('x',)，报告 overlay 只标左右纵向梁筋；manual workspace S2 关闭 spacing prune，保留 continuous validation，并在生成 rectified_intersections 后用纵向梁筋 mask 过滤梁筋内绑扎点。现场 11:40 报告 .debug_frames/pr_fprg_scheme_ablation_report_beam_excluded_lines_20260429_114038 显示方案4 full 为 71 点，梁筋 overlay 无横向红框；仍需继续观察无 spacing 下的近邻线/中心重复点。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py;src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py;src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py;.debug_frames/pr_fprg_scheme_ablation_report_beam_excluded_lines_20260429_114038/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> 65 tests OK; py_compile workspace_s2/manual_workspace_s2/stage/report/probe OK; curl 127.0.0.1:8080 reports -> 200`

### 后续注意

- 暂无。

## 2026-04-29 11:42 - 图形命令前台化并支持 Ctrl-C 关闭

<!-- AGENT-MEMORY: entry -->

### 摘要

- 网页终端里的 rviz/rqt 等命令是 build_terminal_rcfile 注入的 shell wrapper，不是真正直接执行原生命令。现在 wrapper 创建 GUI session 后会保持前台等待，轮询 /api/gui/sessions；Ctrl-C、SIGTERM 或 SIGHUP 会 DELETE /api/gui/sessions/<sessionId>，关闭对应 Xpra/RViz 卡片后再回到 shell prompt。已打开的旧 shell 函数不会自动替换，需要新建终端或重新 source 新 rcfile。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; generated rcfile here-doc compile check; live tmux rviz Ctrl-C check: pane python3 -> Ctrl-C -> /api/gui/sessions [] -> pane bash`

### 后续注意

- 暂无。

## 2026-04-29 11:32 - RViz 最大化等比缩放与性能 debounce

<!-- AGENT-MEMORY: entry -->

### 摘要

- RViz 图形卡片最大化改为 CSS 等比 cover 缩放：用单一 scale=max(scaleX,scaleY) 和 translate 居中/贴顶，避免 scaleX/scaleY 非等比造成画面变形。resizeGraphicalAppFrame 对 Xpra redraw 做 panel.__graphicalAppRedrawTimer debounce，最大化瞬间先应用 GPU transform，再延迟一次高质量 refresh，降低蓝屏等待和卡顿。仍保持后端 rviz geometry no-op，避免回到 OpenGL 黑屏链路。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/test/test_workspace_picker_web.py;src/tie_robot_web/web`

### 关键决策

- 铺满窗口优先使用等比 cover CSS 表面缩放，真实高分辨率原生 resize 暂不启用以避免 RViz 黑屏

### 标签

- `frontend`
- `rviz`
- `xpra`
- `maximize`
- `performance`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build; systemctl active; index-B-FLnXAs.js loaded; geometry POST before/after kept RViz at 60`
- `90`
- `1200`
- `846`

### 后续注意

- 暂无。

## 2026-04-29 11:26 - PR-FPRG 关闭 spacing prune 后必须继续做梁筋 mask 主过滤

<!-- AGENT-MEMORY: entry -->

### 摘要

- 按用户最新要求，当前 manual workspace S2 包装层已关闭 enable_spacing_prune，并重新启用 continuous validation 作为非间距验证；新增 detect_workspace_s2_structural_edge_bands / beam mask overlay，报告可标出左右边缘梁筋/斜撑宽强带。现场 11:22 无 spacing 报告显示梁筋 mask 已可视化，但普通点位仍会被强线带偏，不能把无 spacing 点位图视为最终可用版本；下一步应让 beam mask 直接参与候选线族剔除/降权，而不是恢复 spacing prune。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py;src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py;src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py;.debug_frames/pr_fprg_scheme_ablation_report_no_spacing_beam_tuned_20260429_112245/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m py_compile workspace_s2.py manual_workspace_s2.py pr_fprg_stage_ablation.py pr_fprg_scheme_ablation_report.py pr_fprg_peak_supported_probe.py; python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> 62 tests OK; curl -I 127.0.0.1:8080/reports/pr_fprg_live_full/index.html -> 200`

### 后续注意

- 暂无。

## 2026-04-29 11:08 - RViz 最大化采用前端表面缩放

<!-- AGENT-MEMORY: entry -->

### 摘要

- RViz 最大化时不能恢复后端 xdotool/wmctrl geometry 拉伸，也不能修改 Xpra windowRef.w/h 或调用 handle_resized；改为 UIController.scaleGraphicalAppWindowSurface 用 CSS transform 把 Xpra 已渲染窗口表面缩放到 iframe 大小，外层卡片填满且 RViz 原生 X11 窗口尺寸保持不变。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/test/test_workspace_picker_web.py;src/tie_robot_web/web`

### 关键决策

- 最大化填充使用浏览器 CSS 表面缩放，不改变 RViz/Xpra 原生窗口几何

### 标签

- `frontend`
- `rviz`
- `xpra`
- `maximize`
- `css-scale`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build; curl index-Dj7CRhAJ.js includes scaleGraphicalAppWindowSurface and no old geometry/fit helpers; geometry POST before/after kept RViz at 60`
- `90`
- `1200`
- `846`

### 后续注意

- 暂无。

## 2026-04-29 10:53 - RViz 最大化黑屏规避

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端图形窗口最大化不要再从 UIController 强制调用 Xpra 内部窗口 fit/去边框，也不要自动 POST /api/gui/sessions/<id>/geometry 去 xdotool/wmctrl 拉伸 RViz；RViz/OpenGL 在这种强制缩放链路下会黑屏。当前策略是外层卡片最大化只触发 iframe resize/redraw，后端对 rviz/rviz2 的 geometry 请求返回成功但跳过后端强制缩放。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/scripts/workspace_picker_web_server.py;src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 保持 RViz 原生 Xpra 窗口尺寸，只最大化外层前端卡片

### 标签

- `frontend`
- `rviz`
- `xpra`
- `graphical-window`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build; curl geometry 验证 RViz 窗口尺寸 before/after 不变`

### 后续注意

- 暂无。

## 2026-04-29 10:27 - RViz 黑屏由 ROS Noetic EOL 模态弹窗卡住

<!-- AGENT-MEMORY: entry -->

### 摘要

- RViz 图形卡片出现黑底且只显示 ROS Noetic end-of-life 对话框时，根因是 RViz 启动模态弹窗阻塞主界面初始化，不是 iframe 刷新。GraphicalAppSession 已在 Xpra 启动命令和进程环境中设置 DISABLE_ROS1_EOL_WARNINGS=1，后续 rviz/rqt 图形会话不会再弹这个 EOL 警告。验证方式：DISPLAY=:120 xdotool search --name end-of-life 应为 0，RViz 窗口仍存在。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; systemctl is-active tie-robot-frontend.service; /api/gui/sessions shows rviz ready; DISPLAY=:120 xdotool search --name end-of-life count 0 and RViz count 2`

### 后续注意

- 暂无。

## 2026-04-29 10:12 - PR-FPRG 距离筛选修复必须配质量闸门

<!-- AGENT-MEMORY: entry -->

### 摘要

- 09:50 的全保留近邻修复在真实图上产生右侧/边缘假线爆炸，已废弃。当前改为连续验证返回分数、spacing conflict 按分数保留强 ridge，并要求每组线族至少 3 条才算有效；10:08 raw_world 报告中方案1为30点约90.7ms，方案4/5/6同拓扑但更慢。后续出效果图前必须先人工检查，不得只凭测试通过交付。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py;src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py;.debug_frames/pr_fprg_scheme_ablation_report_scored060_20260429_100849/index.html;.debug_frames/pr_fprg_robustness_temp_site/index.html`

### 关键决策

- 见摘要。

### 标签

- `pointai`
- `pr-fprg`
- `visual-quality`

### 验证证据

- `python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py; python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; curl -I http://127.0.0.1:8096/index.html; curl -I http://127.0.0.1:8097/index.html`

### 后续注意

- 暂无。

## 2026-04-29 10:06 - 图形 iframe 204 拦截需保留新页面首次加载

<!-- AGENT-MEMORY: entry -->

### 摘要

- 为兼容旧浏览器 bundle，服务端会对无 embed_instance 的重复 Xpra /index.html 导航返回 204；新前端 resolveGraphicalAppUrl 会在 iframe URL 上加入 embed_instance，服务端检测到该参数时不做 204 拦截，避免用户刷新主页面后新 bundle 的首次 iframe 加载被旧页面同 IP 重复请求误伤成空白。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `old legacy URL duplicate returned 200/204; new URL with embed_instance returned 200/200; journalctl showed 192.168.6.192 legacy repeated index requests returning 204`

### 后续注意

- 暂无。

## 2026-04-29 10:05 - 前端视觉触发切 canonical recognize_once

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端“触发视觉识别”和“触发单点绑扎”的视觉阶段已从旧 /web/pointAI/run_workspace_s2 Topic 切到 /perception/lashing/recognize_once Trigger 服务；视觉覆盖图订阅切到 /perception/lashing/result_image，点位订阅切到 /perception/lashing/points_camera，清除识别结果仍回到当前图层原图。src/tie_robot_web/web 已通过 npm run build 重建。完整 web 结构测试仍有 3 个既有无关断言失败：run.launch/api.launch 关系、q 键盘映射位置、resultImageRaw 图层旧断言。

### 影响范围

- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/projectGraph/graphData.js`
- `src/tie_robot_web/web`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `docs/superpowers/plans/2026-04-29-ros-interface-naming-migration.md`

### 关键决策

- 见摘要。

### 验证证据

- `5 targeted web tests OK; node --check modified frontend files OK; python3 scripts/check_ros_interface_names.py OK found=89 known=167 missing=0; npm run build OK; scoped git diff --check OK`

### 后续注意

- 暂无。

## 2026-04-29 10:02 - 图形化窗口刷新与置顶修复

<!-- AGENT-MEMORY: entry -->

### 摘要

- 图形化 rviz/rqt 卡片不能只靠前端增量 upsert：旧浏览器 bundle 仍可能每次 /api/gui/sessions 轮询重设 iframe src，导致 Xpra index.html 每 2.5 秒重载。TieRobotFrontApp 现在用 graphicalAppSessionSignature 跳过未变化 session 的 render；NoCacheStaticHandler 对同一客户端同一 GUI session 的短时间重复 /index.html 导航返回 204，保留 iframe 当前文档，兼容已经加载的旧页面。图形窗口置顶通过非激活 graphical-app-focus-catcher 捕获首次点击后 bringGraphicalAppPanelToFront，并在激活后关闭捕获层以恢复 iframe 交互。

### 影响范围

- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build; sudo -n systemctl restart tie-robot-frontend.service; rviz GUI session ready; duplicate /api/gui/proxy/<session>/index.html returned 200 then 204/204; journalctl showed 192.168.6.192 repeated index requests returning 204`

### 后续注意

- 暂无。

## 2026-04-29 09:59 - 视觉 PR-FPRG canonical 接口 alias

<!-- AGENT-MEMORY: entry -->

### 摘要

- pointAI 视觉层新增 /perception/lashing/recognize_once Trigger 服务，复用当前 PR-FPRG 单帧触发链；结果图同步发布 /perception/lashing/result_image 和 result_image_compressed，点同步发布 /perception/lashing/points_camera，工作区同步发布 /perception/lashing/workspace/quad_pixels。旧 /pointAI/* 与 /coordinate_point 继续保留兼容 alias，迁移表对应状态更新为 alias。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- `src/tie_robot_perception/CMakeLists.txt`
- `src/tie_robot_perception/package.xml`
- `docs/architecture/ros_interface_migration_map.yaml`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg src.tie_robot_bringup.test.test_ros_interface_names; python3 scripts/check_ros_interface_names.py; python3 -m py_compile modified python files; catkin_make -DCATKIN_WHITELIST_PACKAGES='' -j2`

### 后续注意

- 暂无。

## 2026-04-29 09:54 - ROS 接口命名静态检查器

<!-- AGENT-MEMORY: entry -->

### 摘要

- 新增 scripts/check_ros_interface_names.py 和 src/tie_robot_bringup/test/test_ros_interface_names.py。检查器读取 docs/architecture/ros_interface_migration_map.yaml，扫描感知、控制、流程、Web 前端源码、Web 桥和 bringup launch 中的 ROS 接口字符串；当前 found=89 known=167 missing=0，并会拒绝未登记的新旧接口名。后续接口迁移阶段应持续运行该检查。

### 影响范围

- `scripts/check_ros_interface_names.py`
- `src/tie_robot_bringup/test/test_ros_interface_names.py`
- `docs/superpowers/plans/2026-04-29-ros-interface-naming-migration.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_bringup.test.test_ros_interface_names; python3 scripts/check_ros_interface_names.py; git diff --check; yaml-ok`

### 后续注意

- 暂无。

## 2026-04-29 09:52 - 帮助站 ROS Graph 必须用 rqt_graph 同源生成

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求帮助站里的 rosgraph 是 rqt_graph 那种 ROS 计算图，不是手工绘制的架构拓扑。当前已新增 src/tie_robot_web/help/scripts/generate_rqt_rosgraph.py，调用 rqt_graph.dotcode.RosGraphDotcodeGenerator 从当前 ROS master 采样生成 tie-robot-ros-graph.dot/svg/png；guide/ros-graph.md 明确说明 rqt_graph 来源。后续更新 rosgraph 时先 source ROS 环境并运行该脚本，再构建帮助站。

### 影响范围

- `src/tie_robot_web/help/scripts/generate_rqt_rosgraph.py; src/tie_robot_web/help/guide/ros-graph.md; src/tie_robot_web/help/public/images/architecture/tie-robot-ros-graph.dot; src/tie_robot_web/help/public/images/architecture/tie-robot-ros-graph.svg; src/tie_robot_web/help/public/images/architecture/tie-robot-ros-graph.png; src/tie_robot_bringup/test/test_giant_business_structuring.py`

### 关键决策

- 架构图可以人工维护，但帮助站 ROS Graph 必须由 rqt_graph 同源生成器或真实 rqt_graph 导出生成，不能用手工 Graphviz 拓扑图替代。

### 标签

- `help`
- `rosgraph`
- `rqt_graph`
- `docs`

### 验证证据

- `source /opt/ros/noetic/setup.bash && python3 src/tie_robot_web/help/scripts/generate_rqt_rosgraph.py; python3 src/tie_robot_bringup/test/test_giant_business_structuring.py; cd src/tie_robot_web/help && npm run build`

### 后续注意

- 暂无。

## 2026-04-29 09:52 - ROS 接口分层命名规划落地

<!-- AGENT-MEMORY: entry -->

### 摘要

- 全工程 ROS 接口命名目标固定为 /hw、/perception、/control、/process、/safety、/calibration、/system 分层；视觉拆为相机驱动原子层和 PR-FPRG 算法层。新增 docs/architecture/ros_interface_naming_plan.md、docs/architecture/ros_interface_migration_map.yaml 和执行计划，后续迁移以 mapping 为权威源，旧 /pointAI、/web/pointAI、/web/moduan、/coordinate_point、/moduan_work 只允许作为兼容 alias。

### 影响范围

- `docs/architecture/ros_interface_naming_plan.md`
- `docs/architecture/ros_interface_migration_map.yaml`
- `docs/superpowers/plans/2026-04-29-ros-interface-naming-migration.md`

### 关键决策

- 见摘要。

### 验证证据

- `git diff --check; yaml.safe_load ros_interface_migration_map.yaml; static inventory check found=89 known=167 missing=0`

### 后续注意

- 暂无。

## 2026-04-29 09:39 - 终端标签同步 tmux window name

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端终端 tab 的 label 只同步 tmux window name，不能由前端自造，也不能任由 tmux 默认把所有新窗口命名为 bash。workspace_picker_web_server 新建 tmux session 时用 -n terminal-N 设置独立 window name，关闭 automatic-rename；发现旧 tie_robot_web_* session 的 window_name 是 bash/sh/zsh/fish/shell 时迁移为 terminal-N。TerminalController 每 2.5 秒刷新 /api/terminal/config，只更新已有 tab label/会话列表，从而 tmux rename-window 后前端能同步。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/controllers/TerminalController.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_backend_uses_tmux_sessions_and_restores_existing_sessions; npm run build in src/tie_robot_web/frontend; tie-robot-frontend.service active; API-created terminal labels were terminal-1/terminal-2 and tmux rename-window sync-check immediately appeared in /api/terminal/config; test sessions then deleted`

### 后续注意

- 暂无。

## 2026-04-29 09:35 - 帮助站新增工程设计、架构图和 ROS Graph

<!-- AGENT-MEMORY: entry -->

### 摘要

- 帮助站新增 guide/system-design.md 与 guide/ros-graph.md，导航和首页入口已接入。图资产放在 src/tie_robot_web/help/public/images/architecture/，包含 DOT 源、SVG 和 PNG；system-design 固化当前八包分层、驱动层原子动作与控制/算法层编排边界，ros-graph 基于 rosnode/rostopic/rosservice 当前采样和 launch 拓扑整理，过滤 RViz/rqt/probe 临时节点。

### 影响范围

- `src/tie_robot_web/help/guide/system-design.md; src/tie_robot_web/help/guide/ros-graph.md; src/tie_robot_web/help/public/images/architecture/; src/tie_robot_web/help/.vitepress/config.mjs; src/tie_robot_web/help/index.md; src/tie_robot_bringup/test/test_giant_business_structuring.py`

### 关键决策

- 帮助站架构图使用 Graphviz DOT 作为可维护源，构建前生成 SVG/PNG 后由 VitePress public 静态托管。

### 标签

- `help`
- `architecture`
- `ros-graph`
- `docs`

### 验证证据

- `python3 src/tie_robot_bringup/test/test_giant_business_structuring.py; cd src/tie_robot_web/help && npm run build; test -f src/tie_robot_web/web/help/guide/system-design.html && test -f src/tie_robot_web/web/help/guide/ros-graph.html`

### 后续注意

- 暂无。

## 2026-04-29 09:32 - 图形卡片轮询不能重建 iframe

<!-- AGENT-MEMORY: entry -->

### 摘要

- 图形化 rviz/rqt 卡片持续刷新的根因是前端每次 /api/gui/sessions 轮询都用 graphicalAppPanelDock.innerHTML 整块重绘，导致同一 session 的 Xpra iframe 被销毁重建。UIController 现在用 upsertGraphicalAppPanel 增量增删/更新卡片，已有 iframe 保持原 DOM，只更新标题、状态和按钮；事件绑定改为幂等，避免轮询重复绑定。另将 Xpra 改为 --start-child + --exit-with-children=yes，图形程序退出时 Xpra 不再孤儿占用 6080。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `TDD red-green: python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards failed before fix then passed; npm run build in src/tie_robot_web/frontend passed; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py passed; tie-robot-frontend.service active; fresh rviz session 871e095a25 stayed ready after 10s and proxy returned HTTP 200; ps showed xpra --exit-with-children=yes --start-child=rviz and live rviz process`

### 后续注意

- 暂无。

## 2026-04-29 09:29 - PR-FPRG peak/continuous 距离去重收窄

<!-- AGENT-MEMORY: entry -->

### 摘要

- 针对现场近间距钢筋被距离过滤误杀的问题，peak support、连续钢筋条验证后的候选去重、最终 spacing prune 都已改为只合并 near-duplicate 峰；主流程不再按全局估计间距删除连续验证通过的近邻线。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py;src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 标签

- `pointai`
- `pr-fprg`
- `robustness`

### 验证证据

- `python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py; python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; git diff --check -- src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 后续注意

- 暂无。

## 2026-04-29 09:27 - PR-FPRG 距离筛选改为近重复合并

<!-- AGENT-MEMORY: entry -->

### 摘要

- 当前主流程已将最终距离筛选从全局间距硬删除改为 near-duplicate 合并，连续验证通过的近间距真实钢筋不应再因 period/median spacing 被误删。地板强响应与真实钢筋 close-pair 的后续改进方向保留为纹理/节点一致性抑制，而不是恢复硬间距过滤。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py;src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 标签

- `pointai`
- `pr-fprg`
- `robustness`

### 验证证据

- `python3 -m py_compile src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py; python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 后续注意

- 暂无。

## 2026-04-29 09:23 - 驱动层只放原子动作，控制/算法层编排执行链

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱和线性模组统一按同一层级边界：tie_robot_hw 驱动层提供原子设备动作/帧/寄存器写入；运动控制、执行链调度、等待到位、FINISHALL 等流程编排放在 tie_robot_control 或 tie_robot_process 的控制/算法层。线性模组已去掉驱动层 executeQueuedPoints()/requestZero() 组合接口，控制层改为调用 clearFinishAll()/writeQueuedPoints()/pulseExecutionEnable()/setZeroRequest()。

### 影响范围

- `src/tie_robot_hw/include/tie_robot_hw/driver/linear_module_driver.hpp; src/tie_robot_hw/src/driver/linear_module_driver.cpp; src/tie_robot_control/src/moduan/linear_module_executor.cpp; src/tie_robot_bringup/test/test_architecture_cleanup.py`

### 关键决策

- 以后新增索驱或线性模组能力时，先在驱动层拆原子函数，再由控制/算法层组织顺序、状态机、暂停/停止、等待反馈和错误恢复。

### 标签

- `architecture`
- `driver`
- `control-layer`
- `moduan`
- `cabin`

### 验证证据

- `python3 src/tie_robot_bringup/test/test_architecture_cleanup.py; source /opt/ros/noetic/setup.bash && catkin_make`

### 后续注意

- 暂无。

## 2026-04-29 09:23 - PR-FPRG 后端结果图覆盖前端显示

<!-- AGENT-MEMORY: entry -->

### 摘要

- 视觉识别显示链路改为后端直接渲染 PR-FPRG 结果图：pointAI 在原始 IR 底图上画绿色线族、黄色绑扎点和编号后以 bgr8 发布 /pointAI/manual_workspace_s2_result_raw；前端点击“触发视觉识别”只等待并叠加该后端结果图，不再根据点位话题自行找线/画线。“清除识别结果”会清空覆盖层并锁存用户想看原图的状态，忽略后续被动 /pointAI/result_image_raw，直到下一次主动视觉触发或其他任务重置。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest <6 targeted PR-FPRG frontend/backend tests>; npm run build; Playwright clicked 触发视觉识别/清除识别结果 and observed bgr8 green/yellow overlay then transparent overlay`

### 后续注意

- 暂无。

## 2026-04-29 09:20 - Xpra 代理超时与 stale session 清理

<!-- AGENT-MEMORY: entry -->

### 摘要

- Xpra ready 不能只检查 TCP connect；旧 xpra 可占着 6080 但 HTTP 不响应，导致前端 iframe 502 timed out。workspace_picker_web_server 现在用 HTTP GET / 验证 ready/wait；代理失败会 close/forget session 并返回带 tie-robot-gui-session-closed postMessage 的关闭页；前端轮询 /api/gui/sessions 并监听该消息清理 stale 图形卡片。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build in src/tie_robot_web/frontend; systemctl is-active tie-robot-frontend.service; fresh rviz session proxy returned HTTP 200 and stale proxy returned 410 cleanup page`

### 后续注意

- 暂无。

## 2026-04-29 09:18 - 线性模组执行触发改为 EN_DISABLE 0->1 脉冲

<!-- AGENT-MEMORY: entry -->

### 摘要

- 线性模组点位下发后不能只持续写 EN_DISABLE=1；驱动层现在先写点位寄存器，再通过 pulseExecutionEnable() 对 5076 写 0 后写 1，形成 PLC 可识别的执行边沿。requestZero() 也复用同一触发函数。

### 影响范围

- `src/tie_robot_hw/include/tie_robot_hw/driver/linear_module_driver.hpp; src/tie_robot_hw/src/driver/linear_module_driver.cpp; src/tie_robot_bringup/test/test_architecture_cleanup.py`

### 关键决策

- 驱动层拆出 writeQueuedPoints() 和 pulseExecutionEnable()，executeQueuedPoints() 只编排写点位和执行触发，避免把 PLC 边沿触发规则埋在高层流程里。

### 标签

- `moduan`
- `linear-module`
- `plc`
- `driver`

### 验证证据

- `python3 src/tie_robot_bringup/test/test_architecture_cleanup.py; source /opt/ros/noetic/setup.bash && catkin_make`

### 后续注意

- 暂无。

## 2026-04-29 09:15 - 前端视觉调试页与视觉计时日志

<!-- AGENT-MEMORY: entry -->

### 摘要

- 设置面板新增视觉调试页：可设置 process_image 最终放行稳定帧数，并调用 /pointAI/process_image 触发完整视觉服务请求。pointAI 新增 /web/pointAI/set_stable_frame_count(Int32) 热更新 stable_frame_count；PR-FPRG 单帧 run_manual_workspace_s2_pipeline 日志增加 elapsed_ms，process_image 返回消息与 ROS 日志同时带单帧视觉耗时和整个视觉服务请求耗时。前端普通日志和视觉调试页日志都会显示单帧/服务耗时，静态产物已通过 npm run build 同步到 src/tie_robot_web/web。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m py_compile pointai modules; python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; node frontend tests including visualDebugSettings; npm run build`

### 后续注意

- 暂无。

## 2026-04-29 09:09 - Xpra 图形卡片无边框嵌入

<!-- AGENT-MEMORY: entry -->

### 摘要

- rviz/rqt 等图形卡片最大化时，不能让 Xpra HTML5 继续以远程桌面窗口 chrome 方式显示；前端在同源 iframe 内注入 embedded 样式，把最大普通 XpraWindow 设为 _set_decorated(false)、0/0、宽高等于卡片 frame，并把 windowicon 同步到外层卡片标题图标。后端 geometry 接口仍作为真实 X11 尺寸兜底，不要恢复全局 patch Xpra Window.js 或 set_maximized(true)。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build; curl -fsS http://127.0.0.1:8080/index.html; systemctl is-active tie-robot-frontend.service`

### 后续注意

- 暂无。

## 2026-04-29 08:59 - PR-FPRG 方案4实时目标与局部 refine 删除

<!-- AGENT-MEMORY: entry -->

### 摘要

- 按用户决策，整体流程删除局部峰值 refine：主流程和对比/消融工具的 full variant 均默认 enable_local_peak_refine=False，报告里不再把 skip_local_refine 作为消融项。4-6 曲线方案中选定 04_dp_depth_curve 作为单帧实时优化目标，05_dp_ridge_curve 与 06_ir_assisted_curve 存档保留为后续方案。为压缩方案4全流程耗时，连续钢筋条验证增加预计算 support map/support mask，默认沿线稀疏采样 continuous_sample_step_px=5.0；方案4曲线追踪 sample_step_px=8.0；曲线 polyline 生成改为向量化。最新报告 .debug_frames/pr_fprg_scheme_ablation_report_20260429_085832/index.html 中，方案4全流程 repeat=3：mean 81.724ms、median 81.439ms、max 87.380ms、25 points；方案5 mean 92.221ms 但作为存档保留；方案6 mean 109.038ms。当前 8770 服务指向该报告。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `.debug_frames/pr_fprg_scheme_ablation_report_20260429_085832/index.html`

### 关键决策

- 方案4作为当前4-6中的实时优化目标；方案5/6存档保留，不作为当前主优化对象。

### 标签

- `pointai`
- `pr-fprg`
- `scheme4`
- `performance`
- `local-refine-removed`

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> 48 tests OK; python3 -m py_compile workspace_s2.py pr_fprg_stage_ablation.py pr_fprg_scheme_ablation_report.py; scheme4 single-frame bench 20 repeats -> mean 81.553ms median 80.924ms max 99.514ms on one run; generated report repeat=3 -> scheme4 mean 81.724ms median 81.439ms max 87.380ms; curl -I http://127.0.0.1:8770/index.html -> HTTP/1.0 200 OK`

### 后续注意

- 暂无。

## 2026-04-29 08:57 - Xpra 图形卡片尺寸同步

<!-- AGENT-MEMORY: entry -->

### 摘要

- 图形卡片最大化不能只改外层面板尺寸：前端需要在 iframe/frame-shell 尺寸变化时调用 Xpra HTML5 client 的窗口几何更新，把最大普通窗口的 x/y/w/h 设到 frame 尺寸；后端新增 /api/gui/sessions/<id>/geometry 兜底，在 session DISPLAY 中用 xdotool windowsize/windowmove + wmctrl 调整真实 X11 主窗口。xdotool search 不要加 --onlyvisible，部分 Xpra 会话会找不到 RViz。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/scripts/workspace_picker_web_server.py;src/tie_robot_web/test/test_workspace_picker_web.py;src/tie_robot_web/web`

### 关键决策

- 后端几何接口是兜底，前端 same-origin Xpra client 几何同步是主路径；避免再次只做 redraw 导致外层最大化但内部 RViz 留边。

### 标签

- `frontend`
- `gui`
- `xpra`
- `rviz`
- `geometry`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; npm run build; systemctl is-active tie-robot-frontend.service; /api/gui/sessions shows rviz ready`

### 后续注意

- 暂无。

## 2026-04-29 08:44 - PR-FPRG 间距筛选风险后续改进存档

<!-- AGENT-MEMORY: entry -->

### 摘要

- 当前继续沿用现有 PR-FPRG 方案，不立即修改 spacing prune。已确认现有间距筛选依赖 rho 候选线相邻间距估计：取较大半部分间距中位数作为 reference_spacing，并删除小于 reference_spacing * 0.65 的近邻候选。该规则能去假线，但在现场钢筋间距参差不齐、真实局部间距小于阈值时有漏检风险。后续改进方向：spacing prune 不应硬删除真实候选，只做疑似重复线合并；近邻冲突需结合 ridge 强度、连续长度、中心线支持率和是否为同一根钢筋重复响应判断；若两条近线都连续且 ridge 支持强，应保留；报告中记录被 spacing 判冲突的 rho 以便现场复核。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`

### 关键决策

- 先按当前方案继续，spacing prune 风险作为后续改进方案存档。

### 标签

- `pointai`
- `pr-fprg`
- `spacing-prune`
- `future-improvement`

### 验证证据

- `memory-only decision; no code change`

### 后续注意

- 暂无。

## 2026-04-29 08:44 - 索驱空格暂停阻断方向按钮二次触发

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱键盘遥控空格暂停使用 document capture 监听；若焦点仍在方向按钮上，事件继续传播会触发方向按钮自身 keydown，把同一次空格当成方向点击，进而因 cabinRemoteMoveInFlight 显示“上一条移动指令仍在执行”。已新增 consumeCabinRemoteKeyboardEvent，在全局遥控键被处理后 preventDefault 并 stopImmediatePropagation，避免暂停键继续冒泡到按钮级快捷键。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/cabinRemoteKeyboard.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/cabinRemoteKeyboard.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-DryErGrl.js`

### 关键决策

- 见摘要。

### 验证证据

- `node test/cabinRemoteKeyboard.test.mjs; node test/logText.test.mjs; node test/rosConnectionController.test.mjs; npm run build`

### 后续注意

- 暂无。

## 2026-04-29 08:40 - 前端视觉识别覆盖层缓存锁存点位

<!-- AGENT-MEMORY: entry -->

### 摘要

- 点击“触发视觉识别”后图像卡片无红点时，先确认 /pointAI/manual_workspace_s2_points 是否已有锁存点位；本次根因是前端只在请求窗口内接收点位，页面订阅时先到的锁存点被丢弃。TieRobotFrontApp 现在缓存最近一次视觉点位，点击后立即叠加到当前图像图层，后续新点位仍会刷新，不切换图像话题。

### 影响范围

- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_s2_trigger_reuses_latched_visual_points_for_overlay; 6 个视觉相关前端测试通过; npm run build; Playwright 真机点击 runSavedS2 后 shownVisualPoints=25 redish=1700`

### 后续注意

- 暂无。

## 2026-04-29 08:37 - 前端图形窗口点击置顶

<!-- AGENT-MEMORY: entry -->

### 摘要

- rviz/rqt 等 Xpra 图形卡片需要和普通浮动卡片共用窗口层级：UIController 为每个图形会话保存 zIndex，新会话生成时直接写入 inline z-index；点击图形卡片、focus catcher 或 same-origin iframe 内部 pointerdown/mousedown/focus 都调用 bringGraphicalAppPanelToFront。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js;src/tie_robot_web/test/test_workspace_picker_web.py;src/tie_robot_web/web`

### 关键决策

- 不要只靠 .graphical-app-panel-dock 的 CSS 层级；iframe 内点击不会自然冒泡到父页面，需要显式桥接。

### 标签

- `frontend`
- `gui`
- `xpra`
- `z-index`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; npm run build; curl http://127.0.0.1:8080/ shows index-3jXd_vTC.js`

### 后续注意

- 暂无。

## 2026-04-29 08:32 - PR-FPRG 1-6 消融报告改为可筛选导航

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户指出 1-6 方案消融报告中其他方案不明显，且 skip_peak_support 失败时出现 OpenCV 中文问号占位图。已更新 src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py：顶部新增 scheme-nav 方案筛选入口，可按方案 1-6 显示各自 8 个消融项；失败项不再生成原图/透视图占位 PNG，改用 HTML failure-panel 说明，避免问号图误导。新版报告目录为 .debug_frames/pr_fprg_scheme_ablation_report_20260429_083038/index.html，当前 8770 服务已指向该目录。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `.debug_frames/pr_fprg_scheme_ablation_report_20260429_083038/index.html`

### 关键决策

- 见摘要。

### 标签

- `pointai`
- `pr-fprg`
- `scheme-ablation`
- `frontend-report`

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k 'scheme_ablation_report' -> 4 tests OK; python3 -m py_compile src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py; python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> 45 tests OK; curl -I http://127.0.0.1:8770/index.html -> HTTP/1.0 200 OK`

### 后续注意

- 暂无。

## 2026-04-29 08:32 - 索驱键盘遥控空格暂停

<!-- AGENT-MEMORY: entry -->

### 摘要

- 索驱遥控页开启键盘遥控后，空格键被定义为及时暂停/停止索驱运动：前端 keydown 解析为 stop action，并直接调用现有 handleCabinRemoteStopAction('keyboard')，不走普通步进移动链路；Q/W/E/A/S/D 继续映射 Z+/X+/Z-/Y+/X-/Y-。页面状态提示已同步显示“空格 = 暂停”。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/cabinRemoteKeyboard.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/test/cabinRemoteKeyboard.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-DhtXYuZT.js`

### 关键决策

- 见摘要。

### 验证证据

- `node test/cabinRemoteKeyboard.test.mjs; node test/logText.test.mjs; node test/rosConnectionController.test.mjs; npm run build`

### 后续注意

- 暂无。

## 2026-04-29 08:26 - Xpra 图形卡片最大化黑屏避坑

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端 rviz/rqt 图形卡片黑屏时，先区分 X11 应用渲染和 Xpra HTML5 重绘层。本次验证 DISPLAY=:120 下 RViz 被拉到 1440x820 后 X11 截图正常，说明黑屏根因在 HTML5 客户端/iframe 尺寸变化后的重绘，不应再通过修改 Xpra Window.js 强制 set_maximized(true) 或隐藏原生窗口装饰来修。当前做法是在前端卡片最大化/还原后对同源 Xpra client 调用 resume/redraw_windows/request_refresh，并在 URL 上使用 sharing=1、offscreen=0；Xpra 服务端启动加 --sharing=yes。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `xpra`
- `rviz`
- `gui`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; npm run build; systemctl is-active tie-robot-frontend.service; curl /api/gui/sessions shows rviz ready`

### 后续注意

- 暂无。

## 2026-04-29 08:25 - 固定识别位姿才写扫描绑扎点JSON

<!-- AGENT-MEMORY: entry -->

### 摘要

- 本地扫描产物 pseudo_slam_points.json、pseudo_slam_bind_path.json 以及 bind_execution_memory.json 只允许由固定识别位姿扫描策略 kFixedManualWorkspace 更新。普通 /web/pointAI/run_workspace_s2 视觉刷新和触发单点绑扎前置视觉只发布/消费点位，不启动扫描产物写入；旧 /cabin/start_pseudo_slam_scan 已默认切到固定识别位姿扫描。

### 影响范围

- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/src/suoqu/service_orchestration.cpp`
- `src/tie_robot_process/test/test_scan_artifact_write_guard.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_process.test.test_scan_artifact_write_guard; python3 -m unittest src.tie_robot_process.test.test_tf_coordinate_contract src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_single_point_bind_button_runs_pr_fprg_before_bind_service src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_s2_trigger_auto_prepares_overlay_view_and_timeout_feedback; source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_process`

### 后续注意

- 暂无。

## 2026-04-29 08:23 - PR-FPRG 1-6 方案工序消融网页报告

<!-- AGENT-MEMORY: entry -->

### 摘要

- 新增 1-6 方案工序消融总报告工具 src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py。报告按 6 个方案分别交叉运行 8 个工序消融项，生成 48 个结果，每项包含原图/透视图叠加效果、核心检测耗时、相对本方案全流程漂移和相对方案1全流程漂移。当前生成报告为 .debug_frames/pr_fprg_scheme_ablation_report_20260429_082133/index.html，服务在 http://127.0.0.1:8770/index.html。全流程基准均值：方案1 71.12ms、方案2 84.04ms、方案3 100.04ms、方案4 93.93ms、方案5 112.23ms、方案6 125.18ms，均为 12 点；skip_peak_support 在 6 个方案中均无法产出两组支持线族。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `.debug_frames/pr_fprg_scheme_ablation_report_20260429_082133/index.html`

### 关键决策

- 见摘要。

### 标签

- `pointai`
- `pr-fprg`
- `scheme-ablation`
- `performance`
- `report`

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -k 'scheme_ablation_report' -> 2 tests OK; python3 -m py_compile src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py; python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> 43 tests OK; curl -I http://127.0.0.1:8770/index.html -> HTTP/1.0 200 OK`

### 后续注意

- 暂无。

## 2026-04-29 08:17 - 索驱键盘遥控焦点过滤修复

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端索驱键盘遥控 Q/W/E/A/S/D 的全局 keydown 会忽略输入控件；开启键盘遥控后焦点停在 cabinKeyboardRemoteToggle 复选框，导致按键被 input 过滤。已将键位映射和过滤抽到 src/utils/cabinRemoteKeyboard.js，只豁免该遥控开关本身，步距/速度输入框、终端和自定义下拉仍继续屏蔽遥控键。

### 影响范围

- `src/tie_robot_web/frontend/src/utils/cabinRemoteKeyboard.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/test/cabinRemoteKeyboard.test.mjs`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-DDtn3lC0.js`

### 关键决策

- 见摘要。

### 验证证据

- `node test/cabinRemoteKeyboard.test.mjs; node test/logText.test.mjs; node test/rosConnectionController.test.mjs; npm run build`

### 后续注意

- 暂无。

## 2026-04-29 08:15 - 视觉识别覆盖等待点位而非结果图

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端触发视觉识别后不再把 /pointAI/result_image_raw 或 /pointAI/manual_workspace_s2_result_raw 当作完成信号；结果图只提供覆盖源尺寸。真正绘制和完成以 PointsArray 点位为准，并新增订阅 pointAI 的 latch 话题 /pointAI/manual_workspace_s2_points，点位到达后用 Pix_coord 贴到当前图像图层，底图继续按当前相机/世界坐标图像流刷新。

### 影响范围

- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-BpL6yvwZ.js`
- `src/tie_robot_web/web/assets/app/index-DzYdBNHJ.css`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `visual-recognition`
- `points-overlay`
- `rosbridge`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_workspace_overlay_chain_uses_dedicated_pr_fprg_overlay src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_image_panel_supports_project_image_topics_and_overlay_switching src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_s2_trigger_auto_prepares_overlay_view_and_timeout_feedback src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_single_point_bind_button_runs_pr_fprg_before_bind_service src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_frontend_assets_exist`
- `npm run build (src/tie_robot_web/frontend)`

### 后续注意

- 暂无。

## 2026-04-29 08:14 - TCP 线模遥控九宫格与暂停按钮

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端设置页的 TCP 线性模组遥控已改为固定九宫格：第一行 Z+/X+/Z-，第二行 Y+/运动暂停/Y-，第三行 角度+/X-/角度-。中心的“运动暂停”不走 single_move，而是向 /web/moduan/interrupt_stop 发布 std_msgs/Float32(data=1)，对应后端暂停/中断线性模组运动。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_tcp_linear_module_remote_page_and_ros_flow_exist src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_settings_page_dropdown_supports_drag_order_persistence src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_panel_manager_supports_header_drag_and_native_resize -v; npm run build`

### 后续注意

- 暂无。

## 2026-04-29 08:11 - PR-FPRG 工序消融网页报告

<!-- AGENT-MEMORY: entry -->

### 摘要

- 已生成工序消融可视化网页 .debug_frames/pr_fprg_stage_ablation_report_20260429_080855/index.html，并用 127.0.0.1:8769 服务。报告锁定 depth_background_minus_filled 响应图，展示全流程与逐项去掉连续钢筋条验证、spacing prune、peak 支撑过滤、局部峰值 refine、profile-only、全角度扫描等变体的原图/透视图叠加效果、单帧耗时和点位漂移。当前帧全流程均值约 64.09ms/12点；去掉连续验证此帧 34.90ms/12点且 0 漂移，但此前其他帧出现假点，不能据单帧全局删除；去掉 spacing 或 peak 支撑会明显退化或失败。

### 影响范围

- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation_report.py`
- `.debug_frames/pr_fprg_stage_ablation_report_20260429_080855/index.html`

### 关键决策

- 见摘要。

### 标签

- `pointai`
- `pr-fprg`
- `ablation`
- `performance`
- `report`

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile src/tie_robot_perception/tools/pr_fprg_stage_ablation.py src/tie_robot_perception/tools/pr_fprg_stage_ablation_report.py src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py; curl -I http://127.0.0.1:8769/index.html -> HTTP/1.0 200 OK`

### 后续注意

- 暂无。

## 2026-04-29 08:08 - 前端视觉识别按当前图层点位覆盖

<!-- AGENT-MEMORY: entry -->

### 摘要

- 图像卡片不再把 /pointAI/result_image_raw 当成独立可选图层，也不再在触发视觉识别时强制切回红外图像。视觉识别触发后前端保留当前图像话题，用 /coordinate_point 的 PointsArray.Pix_coord 在当前图层上绘制点位覆盖；/pointAI/manual_workspace_s2_result_raw 只作为识别源尺寸参考。工作区选点显示增强默认改为原图 gamma=1.0。

### 影响范围

- `src/tie_robot_web/frontend/src/config/imageTopicCatalog.js`
- `src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app/index-CYv5Pfwj.js`
- `src/tie_robot_web/web/assets/app/index-DvqWtXsk.css`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `visual-recognition`
- `image-panel`
- `overlay`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_workspace_overlay_chain_uses_dedicated_pr_fprg_overlay src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_image_panel_supports_project_image_topics_and_overlay_switching src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_s2_trigger_auto_prepares_overlay_view_and_timeout_feedback src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_single_point_bind_button_runs_pr_fprg_before_bind_service src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_frontend_assets_exist`
- `npm run build (src/tie_robot_web/frontend)`

### 后续注意

- 暂无。

## 2026-04-29 08:04 - 设置页拖拽排序性能与卡死兜底

<!-- AGENT-MEMORY: entry -->

### 摘要

- 设置页当前页下拉排序继续使用自定义 pointer 拖拽，但拖动过程改为 requestAnimationFrame 按帧处理最新坐标，排序变化拖动中不再逐次触发 localStorage 持久化，而是在松手收尾时统一通知；同时增加 window pointerup/pointercancel/blur 和 document visibilitychange 的全局收尾，避免 pointer capture 丢失或页面失焦后选项保持虚化卡住。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_settings_page_dropdown_supports_drag_order_persistence src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_panel_manager_supports_header_drag_and_native_resize -v; npm run build`

### 后续注意

- 暂无。

## 2026-04-29 08:02 - PR-FPRG 曲线方案端到端耗时校正

<!-- AGENT-MEMORY: entry -->

### 摘要

- 上一条记忆中的 1-6 方案增量需按完整端到端理解：最终完整 PR-FPRG 直线链路 mean 85.892ms / max 94.936ms；方案2 红外 rho 微调是在此基础上额外约 7.2ms，端到端约 93ms；方案3/4/5/6 曲线是在完整线族初始化后额外约 18.6/19.2/23.1/25.0ms，因此端到端约 104.5/105.1/109.0/110.9ms，仍略超 100ms，不应宣称曲线方案已满足完整端到端 100ms。当前可满足 100ms 的是方案1，以及大概率满足的方案2；曲线方案若作为主链还需要 C++/二进制或继续优化基础线族阶段。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`

### 关键决策

- 见摘要。

### 验证证据

- `final e2e timing mean 85.892ms max 94.936ms; curve incremental timing scheme3/4/5/6 mean 18.643/19.184/23.121/24.988ms`

### 后续注意

- 暂无。

## 2026-04-29 08:01 - PR-FPRG 单帧耗时消融与 100ms 优化

<!-- AGENT-MEMORY: entry -->

### 摘要

- 按用户要求对连续钢筋条验证等耗时阶段做消融测试。结论：连续钢筋条验证不能删除，删除后虽约 47ms 但点数从 9 变 12 且最大漂移约 106px；profile-only 会产生 132 点，不能用；peak support 删除会导致线族失败；spacing prune 在本帧漂移小但不省时，保留；local peak refine 多帧 0 漂移且节省少量时间，已默认关闭但保留开关。性能优化包括：theta 候选池、连续验证矩阵化、ridge contrast 去 percentile 热点、响应候选早停、曲线采样/曲线交点向量化、曲线响应归一化缓存。最终完整 PR-FPRG 单帧 15 次重复 mean 85.892ms / median 84.993ms / max 94.936ms；1-6 方案增量：方案1 约 78.7ms，方案2 额外 7.2ms，方案3 额外 18.6ms，方案4 额外 19.2ms，方案5 额外 23.1ms，方案6 额外 25.0ms。若选方案4，端到端约 98ms；方案5/6 仍略超 100ms，需要 C++ 或继续优化才可作为实时主链。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py`
- `src/tie_robot_perception/tools/pr_fprg_stage_ablation.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile workspace_s2/manual_workspace_s2/probe/ablation; pr_fprg_stage_ablation --repeat 5; final e2e timing mean 85.892ms max 94.936ms`

### 后续注意

- 暂无。

## 2026-04-29 07:56 - 设置页下拉排序拖拽手感优化

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端设置页的当前页下拉排序从原生 HTML5 drag 改为自定义 pointer 拖拽：拖动时原选项虚化、生成跟手 ghost 浮层，并用 FLIP 位置记录为排序变化添加滑动动画；排序仍写入 settings page order localStorage。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_settings_page_dropdown_supports_drag_order_persistence src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_settings_panel_supports_home_page_preference_without_control_panel_customization src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_panel_manager_supports_header_drag_and_native_resize -v; npm run build`

### 后续注意

- 暂无。

## 2026-04-29 07:49 - 前端视觉识别按钮和图像卡片兜底显示

<!-- AGENT-MEMORY: entry -->

### 摘要

- 控制面板 runSavedS2 文案改为“触发视觉识别”。触发后图像卡片仍优先消费 /pointAI/manual_workspace_s2_result_raw；若专用结果话题未及时送达，则 /pointAI/result_image_raw 在 prFprgOverlayRequested=true 时也会走同一覆盖层显示，避免点击后没有可见效果。

### 影响范围

- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/config/topicRegistry.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `visual-recognition`
- `overlay`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_workspace_overlay_chain_uses_dedicated_pr_fprg_overlay src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_s2_trigger_auto_prepares_overlay_view_and_timeout_feedback src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_single_point_bind_button_runs_pr_fprg_before_bind_service src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_frontend_assets_exist`
- `npm run build (src/tie_robot_web/frontend)`

### 后续注意

- 暂无。

## 2026-04-29 07:49 - 前端图形化卡片最大化与铺满

<!-- AGENT-MEMORY: entry -->

### 摘要

- GUI 卡片 header 增加独立最大化/还原按钮，状态由 UIController 的 graphicalAppPanelStates 管理；最大化时按前端 toolbar 下方工作区铺满，并向 iframe 发送 resize。Xpra 同源代理继续补丁旧协议，同时对 index.html 注入 tie-robot-xpra-fill-style、隐藏 Xpra 浮动菜单、触发 resize，并对 Window.js 注入普通可调整窗口自动 set_maximized(true)，用于让 rviz/rqt 主窗口填充卡片，减少 Xpra 桌面黑边。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `xpra`
- `gui`
- `rviz`
- `rqt`

### 验证证据

- `相关 unittest 2/2 OK；npm run build exit 0；py_compile 和 git diff --check exit 0；tie-robot-frontend.service active；/api/gui/sessions 中 rviz 1 ready；代理返回 tie-robot-xpra-fill-style 和 Window.js set_maximized(true) 补丁`

### 后续注意

- 暂无。

## 2026-04-29 07:38 - 前端拆分 PR-FPRG 与单点绑扎入口

<!-- AGENT-MEMORY: entry -->

### 摘要

- 控制面板保留“触发 PR-FPRG”作为纯视觉触发，只发布 /web/pointAI/run_workspace_s2；新增“触发单点绑扎”按钮，前端先复用 PR-FPRG 触发，再调用 /moduan/sg 的 std_srvs/Trigger 服务执行定点绑扎。

### 影响范围

- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js;src/tie_robot_web/frontend/src/controllers/TaskActionController.js;src/tie_robot_web/frontend/src/controllers/RosConnectionController.js;src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js;src/tie_robot_web/frontend/src/config/topicRegistry.js`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `pr-fprg`
- `bind`

### 验证证据

- `npm run build；相关 WorkspacePickerWebTest 静态测试通过；全量 test_workspace_picker_web 仍受既有 run.launch 缺 api.launch 断言影响。`

### 后续注意

- 暂无。

## 2026-04-29 07:35 - 前端图形化卡片 Xpra 兼容性

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端图形化程序卡片使用 Xpra 时，Ubuntu 仓库版 Xpra 3.0.6 与捆绑 HTML5 客户端存在协议字段不兼容：需要在同源 /api/gui/proxy 中修补 Protocol.js 的 rencode flag 与 Client.js 的 encodings/hello 字段；前端不要向 Xpra index 传 server/port 参数，否则 192.168.x.x 会被清洗成 invalid address。启动 rviz/rqt 等 GUI 程序用 xpra --start 而不是 --start-child + --exit-with-children，避免 Xpra 会话退出留下孤儿 GUI 进程。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/ui/PanelManager.js`
- `src/tie_robot_web/frontend/src/styles/app.css`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `xpra`
- `gui`
- `rviz`

### 验证证据

- `service active; /api/gui/sessions shows rviz ready; Firefox screenshot via /api/gui/proxy shows Xsession`
- `rviz; related unittest and py_compile passed`

### 后续注意

- 暂无。

## 2026-04-29 07:27 - PR-FPRG 多方案独立对比看板

<!-- AGENT-MEMORY: entry -->

### 摘要

- 为用户定夺钢筋交点贴合方案，新增独立实验工具 src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py。它不改主发布链，基于同一帧分别输出当前 theta/rho 直线族、归档红外最终 rho 微调、局部 ridge 贪心曲线、动态规划曲线、ridge 约束曲线、红外辅助曲线，并生成浏览器报告。本轮现场报告位于 .debug_frames/pr_fprg_scheme_comparison_20260429_072517/index.html，服务端口临时为 http://127.0.0.1:8767/index.html。为曲线方案在 workspace_s2.py 增加 trace_workspace_s2_curved_line_centerline、build_workspace_s2_curved_line_families、intersect_workspace_s2_curved_line_families；主链仍保持 spacing_pruned 直线族。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py`
- `src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`
- `.debug_frames/pr_fprg_scheme_comparison_20260429_072517/index.html`

### 关键决策

- 见摘要。

### 验证证据

- `python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py; curl -I http://127.0.0.1:8767/index.html`

### 后续注意

- 暂无。

## 2026-04-29 06:59 - PR-FPRG 帮助站流程图改为 theta/rho 斜线族版本

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户明确要求帮助站效果图表达当前目标版本：透视展开工作区 -> 深度响应图 -> 自动估计 theta1/theta2 -> 沿各自法线生成 rho profile -> rho 轴周期峰值 -> 每条线表示为 (theta,rho) -> 沿 theta 连续钢筋条验证 -> 沿法线 ridge 中强两弱判断 -> 两组斜线求交 -> 逆透视投回原图。已替换 pr-fprg-workflow.svg/png，并重建 web/help；不再展示 X/Y 轴 profile、保留 1/4/7/10 或红外最终回正等旧口径。

### 影响范围

- `src/tie_robot_web/help/public/images/visual/pr-fprg-workflow.svg; src/tie_robot_web/help/public/images/visual/pr-fprg-workflow.png; src/tie_robot_web/help/guide/pr-fprg-workflow.md; src/tie_robot_web/web/help`

### 关键决策

- 见摘要。

### 标签

- `pr-fprg`
- `help`
- `theta-rho`
- `documentation`

### 验证证据

- `cairosvg 生成 pr-fprg-workflow.png；npm run build OK；rg 确认 help 和 web/help 中无 X/Y 轴 profile、保留 1/4/7/10、红外连续线最终回正、rho=-263 旧口径`

### 后续注意

- 暂无。

## 2026-04-29 06:50 - 图形代理修复启动竞态与会话恢复

<!-- AGENT-MEMORY: entry -->

### 摘要

- 图形卡片曾在 xpra 端口尚未监听时立即加载 iframe，导致 /api/gui/proxy/<session>/index.html 502 Connection refused。已改为前端只有 session.state === 'ready' 才渲染 iframe，后端代理 wait_for_graphical_app_port 最多等待 12 秒；GraphicalAppSession 不再把端口未就绪的超时误标 ready，退出后会从 manager 移除；页面初始化会主动 fetch /api/gui/sessions 恢复已存在图形卡片。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_panel_manager_supports_header_drag_and_native_resize; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; npm run build; systemctl is-active tie-robot-frontend.service; immediate proxy request after session create returned 200 instead of 502; active rviz proxy index/client returned 200 and xpra info confirmed rviz child alive`

### 后续注意

- 暂无。

## 2026-04-29 06:50 - rosbridge最后归档会话运行态复核

<!-- AGENT-MEMORY: entry -->

### 摘要

- 归档索引最后一条 oversized Codex 会话是 2026/04/25 ...019dc0d7...，其最后任务为 rosbridge_stack 常驻底座收口，而不是 PR-FPRG S2。复核时注意：Action 基名 /web/cabin/start_* 用 rostopic info 会显示 Unknown 是正常的，实际订阅在 /goal 子话题；/web/cabin/start_pseudo_slam_scan/goal、/web/cabin/start_global_work/goal、/web/cabin/run_bind_path_direct_test/goal 均由 /web_action_bridge_node 订阅。

### 影响范围

- `docs/agent_memory/current.md`
- `docs/agent_memory/session_log.md`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_bringup/test -v; systemctl is-active tie-robot-rosbridge.service tie-robot-driver-suoqu.service tie-robot-driver-moduan.service tie-robot-driver-camera.service tie-robot-backend.service; rostopic info /web/cabin/*/goal; rostopic info /system_log/all`

### 后续注意

- 暂无。

## 2026-04-29 06:46 - PR-FPRG 红外最终 rho 微调方案归档

<!-- AGENT-MEMORY: entry -->

### 摘要

- 已将被撤回的红外最终 rho 微调版本整理到 docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md。该档案记录当时 workspace_s2/manual_workspace_s2/probe/test 的改动清单、现场结果、撤回原因和后续改进方向；当前 active PR-FPRG 仍保持无 visual_aligned 的 spacing_pruned 版本。

### 影响范围

- `docs/handoff/2026-04-29_pr_fprg_infrared_rho_alignment_archive.md; docs/agent_memory/current.md; docs/agent_memory/session_log.md`

### 关键决策

- 见摘要。

### 标签

- `pr-fprg`
- `archive`
- `infrared-rho-alignment`
- `handoff`

### 验证证据

- `stat 确认 pr-fprg-result.png 源图与 web/help 构建图已在 2026-04-29 06:33 刷新；新增 handoff 文档；agent_memory refresh/check`

### 后续注意

- 暂无。

## 2026-04-29 06:39 - 图形窗口走前端同源代理并用覆盖层置顶

<!-- AGENT-MEMORY: entry -->

### 摘要

- 为避免远端浏览器直接连 xpra 6080 WebSocket 时停在 Xpra HTML5 Client 连接表单，图形卡片 iframe 改为 /api/gui/proxy/<sessionId>/index.html，同源 HTTP 与 WebSocket 由 workspace_picker_web_server 转发到本机 xpra 端口。图形 iframe 增加 graphical-app-focus-catcher，非激活时首击只负责置顶，激活后才把事件交给 rviz/rqt；普通 floating-panel 与 graphical-app-panel 都会维护 is-window-active 和共享 z-index 池。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/ui/PanelManager.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_panel_manager_supports_header_drag_and_native_resize; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; npm run build; tie-robot-frontend.service active; proxy index/js HTTP 200 and proxy WebSocket 101`

### 后续注意

- 暂无。

## 2026-04-29 06:34 - PR-FPRG 回退到无红外最终微调

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户确认要回到红外最终 rho 微调之前的 PR-FPRG 现场版本：保留方向候选稳定排序、峰值支撑、连续钢筋条验证和 spacing_pruned 距离/间距兜底，撤掉 alignment_response_map、infrared_response_crop 和 visual_aligned 最终贴线阶段。上一条“有界红外剖面微调”记忆已被本条取代。另需后续专题解决：当前钢筋偏斜不是相机或画面坐标造成，而是钢筋实物摆在地上本身就是斜的；本轮只存档，不继续改算法。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py; src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py; src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py; src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; src/tie_robot_web/help/public/images/visual/pr-fprg-steps; src/tie_robot_web/web/help`

### 关键决策

- 见摘要。

### 标签

- `pr-fprg`
- `rollback`
- `perception`
- `agent-memory`

### 验证证据

- `python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py: 36 tests OK; py_compile workspace_s2/manual_workspace_s2/probe OK; live pr_fprg_peak_supported_probe: depth_background_minus_filled`
- `12 points`
- `angles 88/178`
- `rhos [-262`
- `-172`
- `-84] and [-297`
- `-206`
- `-121`
- `-14]; npm run build OK`

### 后续注意

- 暂无。

## 2026-04-29 06:23 - rosbridge 常驻 Web/TF/API 底座

<!-- AGENT-MEMORY: entry -->

### 摘要

- rosbridge_stack.launch 现在同时守护 rosbridge、tf_stack、web_action_bridge_node 和 system_log_mux；run.launch 收口为纯算法后端。驱动和 backend systemd unit 启动前通过 wait_for_ros_master.py 等待 rosbridge 提供的 ROS master，避免驱动抢 roscore 或 robot_tf_broadcaster 重名互踢。

### 影响范围

- `src/tie_robot_bringup/launch/rosbridge_stack.launch`
- `src/tie_robot_bringup/launch/run.launch`
- `src/tie_robot_bringup/systemd/*.service.in`
- `src/tie_robot_bringup/tools/wait_for_ros_master.py`
- `src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest discover -s src/tie_robot_bringup/test; rostopic info /web/cabin/start_pseudo_slam_scan/goal; ss -ltnp 'sport = :11311'`

### 后续注意

- 暂无。

## 2026-04-29 06:23 - Codex 超大会话自动归档守卫

<!-- AGENT-MEMORY: entry -->

### 摘要

- 为永久缓解 Codex/VSCode 历史会话打不开问题，codex_session_guard.py 新增 min-age 与 skip-open 保护；安装 scripts/install_codex_session_guard_timer.sh 后启用用户级 tie-codex-session-guard.timer，开机后和每 15 分钟自动归档超过 50MB、闲置至少 60 分钟且未被打开的 Codex JSONL。当前 timer 已 enabled/active，~/.codex/sessions 已无超过 50MB 的活跃 JSONL。

### 影响范围

- `scripts/codex_session_guard.py`
- `scripts/install_codex_session_guard_timer.sh`
- `src/tie_robot_bringup/test/test_codex_session_guard.py`
- `AGENTS.md`
- `CHANGELOG.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/current.md`
- `docs/agent_memory/session_log.md`
- `scripts/agent_memory.py`

### 关键决策

- 永久方案不是修改 Codex 本体，而是在本机用安全定时守卫把已关闭/闲置的超大会话移出活跃历史目录；仍保留归档 JSONL 以便手工追溯。

### 标签

- `codex-session`
- `systemd`
- `context-slimming`
- `local-maintenance`

### 验证证据

- `python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py src/tie_robot_bringup/test/test_codex_session_guard.py -v`
- `python3 scripts/agent_memory.py check`
- `python3 -m py_compile scripts/codex_session_guard.py`
- `bash -n scripts/install_codex_session_guard_timer.sh`
- `systemctl --user is-enabled/is-active tie-codex-session-guard.timer：enabled/active`
- `python3 scripts/codex_session_guard.py scan --threshold-mb 50 --min-age-minutes 60 --skip-open：no oversized active Codex sessions found`

### 后续注意

- 暂无。

## 2026-04-29 06:15 - Codex 超大会话归档守卫

<!-- AGENT-MEMORY: entry -->

### 摘要

- 截图中打不开的 Codex 历史会话命中 ~/.codex/sessions/2026/04/22/...019db29f...jsonl，大小约 182MB；根因是百 MB 级 JSONL 留在活跃 sessions 目录导致恢复器直接加载时卡死。新增 scripts/codex_session_guard.py 扫描/归档超大活跃会话，并已将 369.3MB、182.1MB、163.8MB 三个活跃超大会话移到 ~/.codex/archived_sessions/oversized，当前 ~/.codex/sessions 已无超过 50MB 的活跃 JSONL。

### 影响范围

- `scripts/codex_session_guard.py`
- `src/tie_robot_bringup/test/test_codex_session_guard.py`
- `AGENTS.md`
- `CHANGELOG.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/organism.md`
- `docs/agent_memory/current.md`
- `docs/agent_memory/session_log.md`
- `scripts/agent_memory.py`
- `src/tie_robot_bringup/test/test_agent_memory_contract.py`

### 关键决策

- 打不开的历史 Codex 会话不强行恢复；保留原始 JSONL 但移出 ~/.codex/sessions 活跃目录，避免 Codex/VSCode 历史列表继续加载超大文件。

### 标签

- `codex-session`
- `context-slimming`
- `local-maintenance`

### 验证证据

- `python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py src/tie_robot_bringup/test/test_codex_session_guard.py -v`
- `python3 scripts/agent_memory.py check`
- `python3 -m py_compile scripts/codex_session_guard.py`
- `python3 scripts/codex_session_guard.py scan --threshold-mb 50：no oversized active Codex sessions found`

### 后续注意

- 暂无。

## 2026-04-29 06:14 - PR-FPRG 有界红外剖面微调

<!-- AGENT-MEMORY: entry -->

### 摘要

- 为解决最终网格线与红外可见钢筋中心差 1-3px 的问题，PR-FPRG 保持深度响应、连续性和间距剪枝作为主筛线链路，只在最终 oriented line rho 上使用 infrared dark-line response 的局部 rho 剖面峰值做有界微调；不恢复旧 pre_img/RANSAC/Hough 或独立 IR 连续线方案。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py; src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py; src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py; src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

### 关键决策

- 见摘要。

### 标签

- `pr-fprg`
- `perception`
- `infrared-alignment`

### 验证证据

- `python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile workspace_s2/manual_workspace_s2/probe; live pr_fprg_peak_supported_probe: 12 points`
- `angles 88/178`
- `profile residuals around <=1px`

### 后续注意

- 暂无。

## 2026-04-29 06:12 - 前端图形窗口改用 xpra WebSocket 自动连接

<!-- AGENT-MEMORY: entry -->

### 摘要

- rviz/rqt 等终端图形命令的前端卡片不应停在 xpra HTML5 Client 连接表单。GraphicalAppSession 已从 --bind-tcp 切到 --bind-ws=0.0.0.0:<port>，前端 iframe 生成 /index.html?server=<当前host>&port=<webPort>&ssl=0&path=/&submit=true，图形窗口与普通 floating-panel 共享 .floating-panel, .graphical-app-panel 的 z-index 置顶池。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/ui/PanelManager.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_panel_manager_supports_header_drag_and_native_resize; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; npm run build; systemctl is-active tie-robot-frontend.service; xpra temp session HTTP 200 and WebSocket 101`

### 后续注意

- 暂无。

## 2026-04-29 06:09 - 设置下拉支持拖拽排序

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端设置面板的“当前页”自定义下拉现在支持拖拽排序：每个设置页行带拖拽手柄，拖放后重排原生 select 与自定义菜单，并通过 tie_robot_frontend_settings_page_order 写入 localStorage；刷新后保持用户排序，新增页面会自动补回默认列表。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -v -k settings_page_dropdown_supports_drag_order_persistence; npm run build via nvm node v20.20.2; full src/tie_robot_web/test/test_workspace_picker_web.py currently has one unrelated failure in test_terminal_graphical_commands_open_frontend_cards because the test expects xpra --bind-ws while workspace_picker_web_server.py contains --bind-tcp.`

### 后续注意

- 暂无。

## 2026-04-29 06:00 - 视觉按钮启动失败的两段根因

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端 header 点击视觉只启动相机、不启动算法层有两段原因：1) systemControlCatalog 中 startAlgorithmStack/restartAlgorithmStack 仍配置 serviceKey，rosbridge 常驻在线时前端优先走 ROS service 而不是 HTTP /api/system/start_algorithm_stack；后端算法桥未在线时该 service 不存在，导致复合动作卡在第二步。已改为 serviceKey:null，强制走前端 HTTP/systemd 脚本。2) start_algorithm_stack.sh 用 rosnode ping -c 1 判断节点存活，但 Noetic 下 unknown node 也可能 exit 0，脚本误判 /pointAINode 已在线并退出。已改为 rosnode list + grep -Fx 精确判断。

### 影响范围

- `src/tie_robot_web/frontend/src/config/systemControlCatalog.js`
- `start_algorithm_stack.sh`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `visual`
- `algorithm-stack`
- `rosnode`

### 验证证据

- `npm run build; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web -v; bash -n start_algorithm_stack.sh restart_algorithm_stack.sh stop_algorithm_stack.sh; ./start_algorithm_stack.sh 后 rosnode list 确认 /pointAINode、/bind_map_builder、/global_bind_planner、/cabin_motion_controller、/moduan_motion_controller、/bind_task_executor 在线；/diagnostics 报 tie_robot/visual_algorithm OK`

### 后续注意

- 暂无。

## 2026-04-29 05:48 - robot_tf_broadcaster 首帧前不再发布零位姿

<!-- AGENT-MEMORY: entry -->

### 摘要

- 排查 base_link 在前端抽搐/弹回 map 原点：原因不是 Scene3DView 把缺失 TF 画成原点，而是 robot_tf_broadcaster 在尚未收到 /cabin/cabin_data_upload 首帧前用 (0,0,0) 初始化并发布 map->base_link。已改为首帧有效索驱位姿到来前不发布机器人位姿 TF；收到非法/NaN 位姿时忽略并保持最后有效 TF；首帧后数据陈旧时继续保持最后有效位姿。

### 影响范围

- `src/tie_robot_perception/scripts/robot_tf_broadcaster.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 标签

- `tf`
- `base_link`
- `frontend`
- `jitter`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web -v; python3 -m unittest src.tie_robot_bringup.test.test_architecture_cleanup src.tie_robot_bringup.test.test_navigation_stack_split -v; python3 -m py_compile src/tie_robot_perception/scripts/robot_tf_broadcaster.py; rostopic echo /tf 确认 base_link=(-0.26`
- `1.7`
- `0.5078)m 与 /cabin/cabin_data_upload=(-260`
- `1700`
- `507.8)mm 对齐`

### 后续注意

- 暂无。

## 2026-04-29 05:42 - 设置页移除驱动守护入口

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端设置卡片不再提供“驱动守护”页面、下拉选项或 data-system-action 按钮；索驱、末端、视觉启停统一走 header 状态胶囊的子系统动作。底层 systemControlCatalog 与 HTTP/systemd 控制能力保留给 header 使用。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web -v; npm run build; systemctl show tie-robot-frontend.service; rg driverGuard/driver-guard in current web assets`

### 后续注意

- 暂无。

## 2026-04-29 05:39 - PR-FPRG 帮助站图重采与方向候选修正

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-29 帮助站 PR-FPRG 流程图已用当前相机正式 raw_world 同步帧重采，结果为 depth_background_minus_filled、线族角度 88/178、3x4 共 12 点；scepter_world_coord_processor 曾因 ROS master 早期连接异常未注册，重启 tie-robot-driver-camera.service 后 raw_world_coord 恢复。方向自适应 PR-FPRG 新增响应图梯度方向先验与候选线中心 ridge 偏移约束，避免对角交叉点伪方向抢占真实钢筋方向。

### 影响范围

- `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- `src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py`
- `src/tie_robot_web/help/guide/pr-fprg-workflow.md`
- `src/tie_robot_web/help/guide/visual-principles.md`
- `src/tie_robot_web/help/public/images/visual/pr-fprg-result.png`
- `src/tie_robot_web/help/public/images/visual/pr-fprg-steps`

### 关键决策

- 见摘要。

### 验证证据

- `npm run build (src/tie_robot_web/help); python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; probe .debug_frames/pr_fprg_peak_supported_probe_20260429_053639 summary: raw_world/depth/12 points`

### 后续注意

- 暂无。

## 2026-04-29 05:37 - 算法栈脚本改用当前节点判断在线

<!-- AGENT-MEMORY: entry -->

### 摘要

- start_algorithm_stack.sh 与 restart_algorithm_stack.sh 不再引用已移除的 /stable_point_tf_broadcaster，统一按当前 algorithm_stack 的 pointAINode、bind_map_builder、global_bind_planner、cabin_motion_controller、moduan_motion_controller、bind_task_executor 判断/重启，避免 header 子系统启动时误拉起重复算法栈。

### 影响范围

- `start_algorithm_stack.sh`
- `restart_algorithm_stack.sh`
- `stop_algorithm_stack.sh`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_bringup/test/test_architecture_cleanup.py`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web -v; python3 -m unittest src.tie_robot_bringup.test.test_architecture_cleanup -v; bash -n stop_algorithm_stack.sh start_algorithm_stack.sh restart_algorithm_stack.sh`

### 后续注意

- 暂无。

## 2026-04-29 05:36 - 图形界面 xpra 局域网访问与浮动层级

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端 rviz/rqt 图形卡片由 xpra HTML 服务承载，xpra 必须 bind 到 0.0.0.0:<webPort>，否则远端浏览器访问 http://<机器人IP>:608x 会被拒绝；图形卡片 dock 层级应低于普通 floating-panel，且图形卡片 header 自身可拖动，避免挡住图像/终端等窗口拖拽。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `graphical-app`
- `xpra`
- `drag`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_graphical_commands_open_frontend_cards src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_panel_manager_supports_header_drag_and_native_resize src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_terminal_tool_uses_backend_session_api_and_multi_session_panel; npm run build; live smoke: temporary GUI session listened on 0.0.0.0:6080 and http://192.168.6.99:6080 returned 200`

### 后续注意

- 暂无。

## 2026-04-29 05:34 - GB28181 设置页按钮式写本机配置

<!-- AGENT-MEMORY: entry -->

### 摘要

- 设置->国标接入继续收敛为操作员操作台：不展示 YAML、配置路径或 roslaunch/source 命令，只保留上级平台参数、本机 IP/设备 ID、刷新/清空、帮助文档和“写入本机配置”按钮；前端按钮调用本机 /api/gb28181/config，由 workspace_picker_web_server.py 校验字段并写入 GB28181 网关配置的 sip 块，若 src/tie_robot_gb28181/config 未部署则返回面向操作员的错误。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
- `src/tie_robot_web/web/index.html`
- `src/tie_robot_web/web/assets/app`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py -v; npm run build via nvm node v20.20.2; rg confirms no GB28181 YAML/path/source/roslaunch text remains in frontend source or active built assets`

### 后续注意

- 暂无。

## 2026-04-29 05:34 - Header 状态胶囊接入子系统启停

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端 header 的索驱、末端、视觉状态胶囊不再只做单驱动重启：在线时执行关闭子系统，离线时执行启动子系统。子系统动作按兼容方式组合单驱动 systemd 动作与 algorithm_stack 启停：start*Subsystem 先启动对应驱动再启动算法层，stop*Subsystem 先停止算法层再停止对应驱动。新增 /api/system/stop_algorithm_stack 与 stop_algorithm_stack.sh。

### 影响范围

- `src/tie_robot_web/frontend/src/config/systemControlCatalog.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/controllers/SystemControlController.js`
- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `stop_algorithm_stack.sh`

### 关键决策

- 见摘要。

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web -v; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py; bash -n stop_algorithm_stack.sh start_algorithm_stack.sh restart_algorithm_stack.sh; npm run build; systemctl show tie-robot-frontend.service`

### 后续注意

- 暂无。

## 2026-04-29 05:33 - Agent 轻启动与长上下文瘦身

<!-- AGENT-MEMORY: entry -->

### 摘要

- 为避免长上下文会话卡顿，Codex/agent 新会话改为轻启动：默认只读 README.md、CHANGELOG.md、docs/agent_memory/current.md；其他记忆文档按任务主题扩展读取。新增长上下文瘦身原则：优先摘要、rg、定向 sed 和限定路径状态检查，不默认塞入整仓状态、大日志、构建产物或无关历史全文。

### 影响范围

- `AGENTS.md`
- `CHANGELOG.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/organism.md`
- `docs/agent_memory/codex_local_setup.md`
- `docs/agent_memory/power_loss_recovery.md`
- `docs/agent_memory/current.md`
- `docs/agent_memory/session_log.md`
- `scripts/agent_memory.py`
- `src/tie_robot_bringup/test/test_agent_memory_contract.py`

### 关键决策

- 共享记忆保留，但默认入口从全量必读改为轻启动三件套；其余文档按需扩展读取，以减少新会话和长会话的上下文负担。

### 标签

- `agent-memory`
- `context-slimming`
- `light-bootstrap`

### 验证证据

- `python3 scripts/agent_memory.py check`
- `python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py -v`
- `codex debug prompt-input "ping" | rg -n "轻启动|按需扩展|长上下文瘦身原则|docs/agent_memory/current.md"`

### 后续注意

- 暂无。

## 2026-04-29 05:30 - 前端构建使用 Node 20 LTS

<!-- AGENT-MEMORY: entry -->

### 摘要

- 本机已安装独立 Node.js v20.20.2 到 /opt/nodejs-lts，并将 /usr/local/bin/node、npm、npx 链到该目录；用于构建 src/tie_robot_web/frontend 的 Vite 5 前端。终端前端兜底 label 也不再生成“终端 n”，缺省退到 tmuxSessionName 或 sessionId。

### 影响范围

- `src/tie_robot_web/frontend/src/controllers/TerminalController.js`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `node`
- `terminal`
- `tmux`

### 验证证据

- `npm run build; node --version; npm --version`

### 后续注意

- 暂无。

## 2026-04-29 05:25 - 前端终端显示 tmux window_name

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端终端卡片由 tmux 后端管理：显示名从对应 tie_robot_web_ tmux 会话的当前 window_name 读取，不使用“终端 1/2/3”等前端编号；通过前端关闭终端会调用 DELETE /api/terminal/sessions/<id> 并 kill 对应 tmux session。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- 见摘要。

### 标签

- `frontend`
- `terminal`
- `tmux`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web; python3 -m py_compile src/tie_robot_web/scripts/workspace_picker_web_server.py`

### 后续注意

- 暂无。

## 2026-04-29 05:23 - GB28181 设置页再次瘦身为操作台

<!-- AGENT-MEMORY: entry -->

### 摘要

- 按用户反馈，设置->国标接入从现场说明页进一步收敛为操作台表单：只保留上级平台参数输入、本机 IP/设备 ID 自动显示、YAML sip 配置预览、启动命令和帮助文档入口；移除设置页中的默认通道表、验收/排障说明和本机检查指导。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `source ~/.nvm/nvm.sh; nvm use 20.20.2; cd src/tie_robot_web/frontend && npm run build`

### 后续注意

- 暂无。

## 2026-04-29 05:18 - 前端终端与图形卡片运行环境已补齐

<!-- AGENT-MEMORY: entry -->

### 摘要

- 本机已安装 tmux、xpra、xpra HTML5 web root，并补充 python3-pyinotify/python3-uinput；rviz/rqt 已由 ROS Noetic 提供。验证 tmux 会话创建成功，xpra --html=on 可在 127.0.0.1:6099 返回 HTML，当前 tie-robot-frontend 与 tie-robot-rosbridge systemd 服务均 enabled/active。安装时临时添加 default route via 192.168.6.1 dev enp1s0 用于 apt 出网。

### 影响范围

- `src/tie_robot_web/scripts/workspace_picker_web_server.py`
- `/usr/share/xpra/www`
- `/usr/bin/tmux`
- `/usr/bin/xpra`

### 关键决策

- 见摘要。

### 验证证据

- `tmux -V; xpra --version; curl http://127.0.0.1:6099/ under xpra test returned HTTP/1.0 200 OK; curl http://127.0.0.1:8080/api/terminal/config returned success`

### 后续注意

- 暂无。

## 2026-04-29 04:49 - GB28181 设置页改为现场参数表单

<!-- AGENT-MEMORY: entry -->

### 摘要

- 按用户纠正，前端设置->国标接入不再搬运帮助文档，改为现场接入表单：上级平台 SIP IP/端口/平台 ID/域/密码由现场输入，本机 SIP IP 按当前浏览器访问地址自动推断并显示，实时生成 gb28181_device.yaml 的 sip 配置片段和交给上级平台的本机信息；帮助站 GB28181 文档也移除写死的远端平台 IP 示例，改用平台提供/占位符。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/help/guide/gb28181-video-gateway.md`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && npm run build; cd src/tie_robot_web/help && npm run build`

### 后续注意

- 暂无。

## 2026-04-29 04:48 - Agent 示例泛化原则

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户说“比如”“例如”“类似”时，后续内容默认是意图线索和启发样例；agent 应结合当前工程事实举一反三，避免机械照搬，除非用户明确要求严格按例子执行。

### 影响范围

- `AGENTS.md`
- `docs/agent_memory/organism.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/current.md`
- `CHANGELOG.md`
- `scripts/agent_memory.py`
- `src/tie_robot_bringup/test/test_agent_memory_contract.py`

### 关键决策

- 示例不是需求边界；先抽取真实目标，再根据代码、记忆和风险边界选择合适做法。

### 标签

- `agent-memory`
- `codex-behavior`
- `example-generalization`

### 验证证据

- `python3 scripts/agent_memory.py check`
- `python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py -v`
- `codex debug prompt-input "ping" | rg -n "示例泛化原则|比如|例如|机械照搬"`

### 后续注意

- 暂无。

## 2026-04-29 04:37 - 设置卡片新增 GB28181 本机接入页

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端设置卡片新增“国标接入”页面，把本机部署国标包、确认 ROS 图像话题与 ffmpeg、配置 gb28181_device.yaml、启动节点、提供给上级平台的信息、默认 7 路通道、验收与排障抓包步骤集中到设置面板内；已执行前端 npm run build 同步 src/tie_robot_web/web 主页面资产。

### 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/web`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/frontend && npm run build`

### 后续注意

- 暂无。

## 2026-04-29 04:28 - Agent 断电恢复层建立

<!-- AGENT-MEMORY: entry -->

### 摘要

- 共享记忆系统新增断电恢复层：power_loss_recovery.md 定义能力边界，checkpoint.md 保存最近可恢复现场，agent_memory.py 新增 checkpoint/recover 命令；checkpoint 写入使用临时文件替换以降低半写入风险。

### 影响范围

- `AGENTS.md`
- `README.md`
- `CHANGELOG.md`
- `docs/agent_memory/power_loss_recovery.md`
- `docs/agent_memory/checkpoint.md`
- `docs/agent_memory/current.md`
- `docs/agent_memory/session_log.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/organism.md`
- `scripts/agent_memory.py`
- `src/tie_robot_bringup/test/test_agent_memory_contract.py`

### 关键决策

- 断电恢复层只承诺恢复已落盘现场，不承诺恢复未保存 IDE buffer、运行中 ROS 进程或未写入 checkpoint/session_log 的临时思路。

### 标签

- `power-loss-recovery`
- `agent-memory`

### 验证证据

- `python3 scripts/agent_memory.py check：agent memory contract ok`
- `python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py -v：4 tests OK`
- `python3 scripts/agent_memory.py recover：输出 Agent Recovery Report 并包含 checkpoint.md`

### 后续注意

- 暂无。

## 2026-04-29 04:28 - GB28181 帮助页补充平台接入 SOP

<!-- AGENT-MEMORY: entry -->

### 摘要

- GB28181 视频接入帮助页新增面向上级平台工作人员的逐步接入步骤：网络互通、平台建设备、本机配置、启动、目录确认、点播和验收标准；已重新构建 src/tie_robot_web/web/help。

### 影响范围

- `src/tie_robot_web/help/guide/gb28181-video-gateway.md`
- `src/tie_robot_web/web/help/guide/gb28181-video-gateway.html`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/help && npm run build`

### 后续注意

- 暂无。

## 2026-04-29 04:21 - Driver stack remains independent from run.launch

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-29 verification pass: run.launch intentionally starts API + algorithm stack only. Driver hardware nodes remain in driver_stack.launch and the independent driver systemd services. Tests now expand nested driver launch includes when validating suoqu_driver_node, moduan_driver_node, camera, and gripper_tf_broadcaster, instead of requiring driver_stack.launch to inline all node tags.

### 影响范围

- `src/tie_robot_bringup/launch/run.launch`
- `src/tie_robot_bringup/test/test_architecture_cleanup.py`
- `src/tie_robot_bringup/test/test_driver_algorithm_node_boundaries.py`
- `src/tie_robot_bringup/test/test_navigation_stack_split.py`
- `src/tie_robot_perception/test/test_gripper_tf_broadcaster.py`
- `src/tie_robot_process/test/test_tf_coordinate_contract.py`
- `CHANGELOG.md`

### 关键决策

- Do not add driver_stack.launch back into run.launch unless explicitly changing the driver lifecycle model; keep hardware drivers independently guarded.

### 标签

- `ros`
- `launch`
- `driver-stack`
- `verification`

### 验证证据

- `python3 -m unittest all 12 project test modules -> 152 tests OK; roslaunch run/api/driver/algorithm --nodes OK; catkin_make -DCATKIN_WHITELIST_PACKAGES= -j2 OK`

### 后续注意

- 暂无。

## 2026-04-29 04:10 - Web bridge ROS executable naming

<!-- AGENT-MEMORY: entry -->

### 摘要

- 2026-04-29: tie_robot_web launch now starts web_action_bridge_node with lower_snake_case name/type. CMake builds both web_action_bridge_node and legacy webActionBridgeNode targets from the same sources; keep webActionBridgeNode only as a compatibility executable, not as the launch entry.

### 影响范围

- `src/tie_robot_bringup/launch/api.launch`
- `src/tie_robot_web/CMakeLists.txt`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 关键决策

- Use lower_snake_case for active ROS executable names; retain legacy camelCase targets only when needed for backward compatibility.

### 标签

- `ros`
- `catkin`
- `naming`

### 验证证据

- `python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_run_launch_keeps_frontend_and_core_nodes_guarded src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_web_bridge_no_longer_runs_legacy_topic_conversion_node; catkin_make -DCATKIN_WHITELIST_PACKAGES= -j2`

### 后续注意

- 暂无。

## 2026-04-29 04:04 - ROS package layout normalization

<!-- AGENT-MEMORY: entry -->

### 摘要

- ROS package cleanup moved non-node helpers from catkin scripts/ to tools/: tie_robot_perception/tools/pr_fprg_peak_supported_probe.py and tie_robot_web/tools/run_gitnexus_local_webui.py. C++ public headers in tie_robot_control, tie_robot_process, and tie_robot_web now live under include/<package_name>/ and includes should use <tie_robot_xxx/...> to avoid common.hpp/json.hpp collisions. tie_robot_control now exports INCLUDE_DIRS include and installs moduanNode/moduan_driver_node/moduan_motion_controller_node; tie_robot_web exports and installs its public headers.

### 影响范围

- `src/tie_robot_bringup/test/test_architecture_cleanup.py`
- `src/tie_robot_control/CMakeLists.txt`
- `src/tie_robot_web/CMakeLists.txt`
- `src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py`
- `src/tie_robot_web/tools/run_gitnexus_local_webui.py`

### 关键决策

- Keep scripts/ for ROS executable entrypoints only; put offline probes/local developer helpers under tools/.

### 标签

- `ros`
- `catkin`
- `package-layout`

### 验证证据

- `python3 -m unittest src.tie_robot_bringup.test.test_architecture_cleanup; python3 -m unittest src.tie_robot_perception.test.test_pointai_scan_only_pr_fprg; python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web; catkin_make whitelist tie_robot_msgs/tie_robot_hw/tie_robot_control/tie_robot_process/tie_robot_perception/tie_robot_web -j2`

### 后续注意

- 暂无。

## 2026-04-29 04:04 - Codex 工程有机体协议建立

<!-- AGENT-MEMORY: entry -->

### 摘要

- 本工程目录下的 Codex 已被定义为工程有机体协议：通过 AGENTS.md 启动读取记忆系统，按 organism.md 执行感知、记忆、免疫、生长闭环，并提供 codex_local_setup.md 说明用 codex -C 从正确目录启动。

### 影响范围

- `AGENTS.md`
- `README.md`
- `CHANGELOG.md`
- `docs/agent_memory/organism.md`
- `docs/agent_memory/codex_local_setup.md`
- `docs/agent_memory/current.md`
- `docs/agent_memory/session_log.md`
- `scripts/agent_memory.py`
- `src/tie_robot_bringup/test/test_agent_memory_contract.py`
- `docs/superpowers/plans/2026-04-29-codex-organism-protocol.md`

### 关键决策

- 有机体是工程协作协议，不是后台自主进程；所有行动仍由当前会话和用户意图驱动。

### 标签

- `codex-organism`
- `agent-memory`

### 验证证据

- `python3 scripts/agent_memory.py check：agent memory contract ok`
- `python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py -v：OK`
- `codex debug prompt-input 'ping' | rg -n 'Codex 工程有机体协议|docs/agent_memory/organism.md|docs/agent_memory/codex_local_setup.md'：命中 AGENTS.md 注入内容`

### 后续注意

- 暂无。

## 2026-04-29 03:52 - Codex 会话启动协议加固

<!-- AGENT-MEMORY: entry -->

### 摘要

- 基于 Codex 会把根 AGENTS.md 注入新会话上下文的特性，AGENTS.md 已新增 Codex 会话启动协议，要求每次开启 Codex 会话先读取 README、CHANGELOG 和 docs/agent_memory 快照；已用 codex debug prompt-input 验证模型可见输入包含该协议。

### 影响范围

- `AGENTS.md`
- `docs/agent_memory/README.md`
- `docs/agent_memory/current.md`
- `CHANGELOG.md`
- `scripts/agent_memory.py`
- `src/tie_robot_bringup/test/test_agent_memory_contract.py`

### 关键决策

- 使用 AGENTS.md 作为 Codex 项目级自动入口，而不是依赖手动提示或单次会话记忆。

### 标签

- `codex-bootstrap`
- `agent-memory`

### 验证证据

- `codex debug prompt-input 'ping' | rg -n 'Codex 会话启动协议|docs/agent_memory/current.md|每次开启 Codex 会话|codex debug prompt-input'：命中 AGENTS.md 注入内容`
- `python3 scripts/agent_memory.py check：agent memory contract ok`
- `python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py -v：OK`

### 后续注意

- 暂无。

## 2026-04-29 03:52 - 帮助站新增对外接入文档

<!-- AGENT-MEMORY: entry -->

### 摘要

- 前端帮助站新增 ROS 动态 API 网关与 GB28181 视频接入两篇 guide，并在 VitePress nav/sidebar 与首页入口挂载；已执行 npm run build 同步 src/tie_robot_web/web/help 静态产物。

### 影响范围

- `src/tie_robot_web/help/guide/dynamic-api-gateway.md`
- `src/tie_robot_web/help/guide/gb28181-video-gateway.md`
- `src/tie_robot_web/help/.vitepress/config.mjs`
- `src/tie_robot_web/help/index.md`
- `src/tie_robot_web/web/help`

### 关键决策

- 见摘要。

### 验证证据

- `cd src/tie_robot_web/help && npm run build`

### 后续注意

- 暂无。

## 2026-04-29 03:42 - Agent 共享记忆系统完成

<!-- AGENT-MEMORY: entry -->

### 摘要

- 已建立仓库内共享记忆入口：AGENTS 启动必读、docs/agent_memory 快照与账本、scripts/agent_memory.py 追加/刷新/校验 CLI，并用 unittest 固化契约。

### 影响范围

- `AGENTS.md`
- `README.md`
- `CHANGELOG.md`
- `docs/agent_memory`
- `scripts/agent_memory.py`
- `src/tie_robot_bringup/test/test_agent_memory_contract.py`

### 关键决策

- 记忆放在仓库内，避免依赖单个 agent 的私有上下文；current.md 作快照，session_log.md 作长期账本。

### 标签

- `agent-memory`
- `codex-handoff`

### 验证证据

- `python3 scripts/agent_memory.py check && python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py -v：OK`

### 后续注意

- 暂无。

## 2026-04-29 记忆系统入口建立

<!-- AGENT-MEMORY: entry -->

### 摘要

- 本仓库新增面向 agent 的共享记忆机制：根 `AGENTS.md` 负责启动必读入口，`docs/agent_memory/current.md` 提供当前快照，`session_log.md` 作为追加式账本。
- 后续会话如果产生关键工程知识，应通过 `scripts/agent_memory.py add ...` 写入账本，再运行 `scripts/agent_memory.py refresh` 更新快照。

### 影响范围

- `AGENTS.md`
- `docs/agent_memory/`
- `scripts/agent_memory.py`
- `src/tie_robot_bringup/test/test_agent_memory_contract.py`

### 关键决策

- 记忆系统放在仓库内，而不是依赖某个 agent 的私有会话存储；这样 Codex、其他 AI 代理和人工工程师都能读取同一份先验。
- `current.md` 是快照，`session_log.md` 是长期账本。重要知识先进入账本，再由脚本刷新快照。

### 验证证据

- `python3 -m unittest src/tie_robot_bringup/test/test_agent_memory_contract.py -v`：用于验证记忆入口、文档互链和 CLI 契约。

### 后续注意

- 不要把 `.debug_frames` 大目录、构建产物或长日志塞进记忆账本，只记录路径、结论和验证命令。
