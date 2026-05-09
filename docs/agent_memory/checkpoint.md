# Agent Power-Loss Checkpoint

- 更新时间：2026-05-09 17:41:01
- HEAD：`da94e23`
- 分支：`slam`

## 当前任务

- 保存当前工程到 slam/v40 后用 slam/v35 强制覆盖本地工程

## 下一步

- 先提交当前项目文件并强制更新远端 tag slam/v40，再 checkout/reset 到 tag slam/v35 并清理未跟踪项目文件

## 影响范围

- `全仓当前项目文件；不纳入本地 .superpowers 临时目录`

## 最近验证

- `将用 git tag/ls-remote/status 验证`

## 注意事项

- 用户明确要求强制覆盖本工程，执行前先把当前状态上传到 slam/v40 tag 作为恢复点

## Git 状态摘要

```text
M CHANGELOG.md
 M README.md
 M docs/agent_memory/current.md
 M docs/agent_memory/session_log.md
 D src/tie_robot_bringup/launch/demo_rosbridge_light.launch
 M src/tie_robot_bringup/scripts/install_demo_mode_service.sh
 M src/tie_robot_bringup/scripts/install_frontend_autostart.sh
 M src/tie_robot_bringup/scripts/install_show_legacy_frontend_service.sh
 M src/tie_robot_bringup/systemd/tie-robot-backend-control.sudoers.in
 M src/tie_robot_bringup/systemd/tie-robot-backend.service.in
 D src/tie_robot_bringup/systemd/tie-robot-demo-rosbridge.service.in
 D src/tie_robot_bringup/systemd/tie-robot-demo-show-full.service.in
 M src/tie_robot_bringup/systemd/tie-robot-driver-camera.service.in
 M src/tie_robot_bringup/systemd/tie-robot-driver-moduan.service.in
 M src/tie_robot_bringup/systemd/tie-robot-driver-suoqu.service.in
 M src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py
 M src/tie_robot_control/src/moduan/linear_module_executor.cpp
 M src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp
 M src/tie_robot_control/test/test_single_point_bind_chain.py
 M src/tie_robot_hw/include/tie_robot_hw/driver/cabin_driver.hpp
 M src/tie_robot_hw/src/driver/cabin_driver.cpp
 M src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp
 M src/tie_robot_msgs/action/StartPseudoSlamScanTask.action
 M src/tie_robot_msgs/srv/StartPseudoSlamScan.srv
 M src/tie_robot_perception/config/gripper_tf.yaml
 M src/tie_robot_perception/data/manual_workspace_quad.json
 M src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py
 M src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py
 M src/tie_robot_perception/test/test_scan_surface_dp_runtime.py
 M src/tie_robot_process/data/bind_execution_memory.json
 M src/tie_robot_process/data/cabin_state.json
 M src/tie_robot_process/data/pseudo_slam_bind_path.json
 M src/tie_robot_process/data/pseudo_slam_points.json
 M src/tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp
 M src/tie_robot_process/src/planning/dynamic_bind_planning.cpp
 M src/tie_robot_process/src/suoqu/cabin_transport.cpp
 M src/tie_robot_process/src/suoqu/service_orchestration.cpp
 M src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp
 M src/tie_robot_process/src/suoquNode.cpp
 M src/tie_robot_process/test/test_cabin_tcp_transport_contract.py
 M src/tie_robot_process/test/test_dynamic_bind_planning.cpp
 M src/tie_robot_process/test/test_motion_chain_signal_guard.py
 M src/tie_robot_process/test/test_scan_artifact_write_guard.py
 M src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js
 M src/tie_robot_web/frontend/src/config/statusMonitorCatalog.js
 M src/tie_robot_web/frontend/src/config/systemControlCatalog.js
 M src/tie_robot_web/frontend/src/controllers/StatusMonitorController.js
 M src/tie_robot_web/frontend/src/controllers/SystemControlController.js
 M src/tie_robot_web/frontend/src/controllers/TaskActionController.js
 M src/tie_robot_web/frontend/src/styles/app.css
 M src/tie_robot_web/frontend/src/ui/UIController.js
 M src/tie_robot_web/frontend/src/utils/bindPathGeometry.js
 M src/tie_robot_web/frontend/src/utils/irImageUtils.js
 M src/tie_robot_web/frontend/src/utils/storage.js
 M src/tie_robot_web/frontend/src/views/Scene3DView.js
 M src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js
 M src/tie_robot_web/frontend/test/bindPathGeometry.test.mjs
 M src/tie_robot_web/frontend/test/statusMonitorController.test.mjs
 M src/tie_robot_web/frontend/test/taskActionController.test.mjs
 M src/tie_robot_web/frontend/test/taskActionSurfaceDpRecognition.test.mjs
 M src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
 M src/tie_robot_web/frontend/test/workspaceCanvasInteractionMode.test.mjs
 M src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs
 M src/tie_robot_web/help/reference/gitnexus-graph.md
 M src/tie_robot_web/scripts/workspace_picker_web_server.py
 M src/tie_robot_web/src/web_bridge/action_bridge.cpp
 M src/tie_robot_web/test/test_workspace_picker_web.py
 D src/tie_robot_web/web/assets/app/index-0Xq9uxzh.js
 D src/tie_robot_web/web/assets/app/index-Da7E5jgi.css
 M src/tie_robot_web/web/help/assets/reference_gitnexus-graph.md.CD70ENMC.js
 M src/tie_robot_web/web/help/assets/reference_gitnexus-graph.md.CD70ENMC.lean.js
 M src/tie_robot_web/web/index.html
?? .superpowers/
?? docs/archive/cabin_tcp_industrialized_connection_scheme_2026-05-07.md
?? src/tie_robot_web/frontend/src/utils/areaProgress.js
?? src/tie_robot_web/frontend/test/areaProgressDisplay.test.mjs
?? src/tie_robot_web/frontend/test/scenePointHoverTooltip.test.mjs
... omitted 4 more lines ...
```

## 恢复命令

```bash
python3 scripts/agent_memory.py recover
python3 scripts/agent_memory.py check
```
