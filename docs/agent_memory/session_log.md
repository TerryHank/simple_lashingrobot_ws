# Agent Session Memory Log

本文件按时间倒序记录跨会话共享记忆。新条目写在最上方，并保留 `AGENT-MEMORY:` 标记，方便脚本识别。

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

- `PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile pointai modules; git diff --check relevant files`

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

- `PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

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

- `PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

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

- `PYTHONPATH=src/tie_robot_perception/src:devel/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

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

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py -> Ran 88 tests OK`

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

- `source /opt/ros/noetic/setup.bash && source devel/setup.bash && PYTHONPATH=src/tie_robot_perception/src:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; python3 -m py_compile manual_workspace_s2.py processor.py state.py`

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

- 20260403旧工作区的 show_full.launch 实际包含 api.launch + run.launch，run.launch 拉起的是 chassis_ctrl/suoquNode、moduanNode、pointAINode；suoquNode_show.cpp 虽有 0x0012 bit1 相对姿态控制样例，但 CMake 未 add_executable、devel/lib/chassis_ctrl 也无 suoquNode_show，不能当成旧展示运行主链。旧前端/调试按钮发 /web/cabin/cabin_move_debug，topictransNode 转 /cabin/single_move，suoquNode::cabin_single_move 组 0x0012 control_word=0x01 绝对位姿帧。旧状态轮询与运动共用同一 sockfd/socket_mutex，但 Frame_Generate 单次 recv(Rlen)，状态包 144 字节读边界不稳，理论上也会残留污染后续 8 字节运动回包。

### 影响范围

- `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/chassis_ctrl/launch/show_full.launch`
- `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/chassis_ctrl/launch/run.launch`
- `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/chassis_ctrl/src/topics_transfer.cpp`
- `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp`
- `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode_show.cpp`

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

- `PYTHONPATH=/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src:/home/hyq-/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages python3 -m unittest src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py; PYTHONPATH=... python3 -m py_compile execution_refine_hough.py process_image_service.py processor.py ros_interfaces.py`

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

- 旧 20260403 演示链不再复用当前 tie-robot-rosbridge.service；进入演示时停止当前完整 rosbridge、backend、三个 driver 和旧转义层，启动 tie-robot-demo-rosbridge.service（仅 rosbridge_websocket + rosapi，无当前 tf_stack/api.launch），再启动旧工作目录 roslaunch chassis_ctrl show_full.launch。demo rosbridge 用 topics_glob 白名单保留 /pointAI/result_image，并只放行 Scepter compressed 图像，避免旧前端继续订阅 raw 大流量图像。

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

- 旧 20260403 演示链的 chassis_ctrl/suoquNode 和 topictransNode 曾硬编码读取/清理 /home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/path_points.json；当前主仓已无 src/chassis_ctrl，导致 /cabin/start_work 抛出‘无法打开路径点JSON文件’。已在旧工作目录源码改为使用 /home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/chassis_ctrl/data，并重编旧工作目录。

### 影响范围

- `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp`
- `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/chassis_ctrl/src/topics_transfer.cpp`

### 关键决策

- 见摘要。

### 验证证据

- `bad-path grep red/green; old workspace catkin_make exit 0; tie-robot-demo-show-full.service active; rosservice info /cabin/start_work shows chassis_ctrl/MotionControl from /suoquNode`

### 后续注意

- 暂无。

## 2026-04-30 20:03 - 新前端演示模式直跑旧 show_full

<!-- AGENT-MEMORY: entry -->

### 摘要

- 用户最新口径是不再使用 show_legacy_driver_bridge/旧展示转义层。新前端 header 增加演示模式按钮：红色表示普通模式，点击后保留当前 tie-robot-rosbridge.service 和 5173 旧前端，停止当前 backend、三个 driver service 和旧 shared-driver-stack 守护，启动按需 unit tie-robot-demo-show-full.service（旧 20260403 工作目录 roslaunch chassis_ctrl show_full.launch）；再次点击停止 show_full，清理旧 ROS 残留节点/进程，再恢复当前 rosbridge、driver、backend。旧 show_legacy_driver_bridge 包与 shared-driver-stack 未再保留为工作区文件，机器上已禁用/停止旧 tie-robot-show-legacy-shared-driver-stack.service。

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

- `npm run build; selected unittest demo/system-control tests OK; roslaunch --nodes chassis_ctrl show_full.launch OK with explicit legacy+Scepter env; actual POST /api/system/toggle_demo_mode enter/exit OK; final systemd status: demo inactive`
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

- 为完全使用旧展示工作区 /home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws，已停止当前工程 systemd 驱动/后端/rosbridge/8080 前端服务；重建旧工作区 build/devel 并修复其 CMake 依赖顺序和 libsimulated_annealing 路径；~/.bashrc 末尾加入 legacy lashing robot workspace 块，使 chassis_ctrl/fast_image_solve 优先解析到 20260403 旧工作区，并显式加入 /home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS 以支持 api.launch 的 ScepterROS。旧工作区 start.sh/start_.sh/restart.sh/topic_tranfer.sh 同步使用该环境。验证 rospack find chassis_ctrl/fast_image_solve 指向旧工作区，rospack find ScepterROS 指向 ScepterSDK，roslaunch --nodes chassis_ctrl run.launch/api.launch/suoquAndmoduan.launch 可解析节点。

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

- 已安装并启动 tie-robot-show-legacy-frontend.service，服务以 python3 -m http.server 5173 --bind 0.0.0.0 --directory /home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/APP/dist 运行；原手动占用5173的 python3 -m http.server 进程已停止，5173 现在由 systemd 管理。

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

- `systemctl is-enabled tie-robot-show-legacy-frontend.service -> enabled; systemctl is-active -> active; curl -I http://127.0.0.1:5173/ -> HTTP/1.0 200 OK`

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

- 2026-04-29 对照只读旧工程 /home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws：旧版 /moduan/sg 会写入视觉点位、拉起 EN_DISABLE 执行信号、在服务内部 finish_all(150) 等 FINISHALL 置位，随后清 FINISHALL；索驱执行层同步 call 该服务时不会在末端执行未完成前继续移动。当前工程应保留这个模型，用 /moduan_work 表示线性模组/绑扎忙，索驱移动入口检查该信号拒绝并发移动，避免新增不必要的单点动作链。

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
