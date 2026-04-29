# Agent Session Memory Log

本文件按时间倒序记录跨会话共享记忆。新条目写在最上方，并保留 `AGENT-MEMORY:` 标记，方便脚本识别。

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
