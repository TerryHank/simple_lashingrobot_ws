# Agent Power-Loss Checkpoint

- 更新时间：2026-05-07 04:13:45
- HEAD：`cbd5b3a`
- 分支：`slam`

## 当前任务

- 配置化绑扎分组前现场快照

## 下一步

- 存档当前未提交现场态，强制推送 slam/v35，再按 TDD 增加视觉调试每组点数设置和后端可达性规划校验

## 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp; src/tie_robot_web/frontend/src/ui/UIController.js; src/tie_robot_web/frontend/src/utils/storage.js; src/tie_robot_msgs/action/StartPseudoSlamScanTask.action; src/tie_robot_msgs/srv/StartPseudoSlamScan.srv`

## 最近验证

- `尚未验证，刚开始任务`

## 注意事项

- 当前工作树已有大量现场未提交改动；不要回滚。用户明确要求强制覆盖上传 git slam/v35。

## Git 状态摘要

```text
M CHANGELOG.md
 M docs/agent_memory/current.md
 M docs/agent_memory/session_log.md
 M src/tie_robot_control/include/tie_robot_control/moduan/error_handling.hpp
 M src/tie_robot_control/include/tie_robot_control/moduan/linear_module_executor.hpp
 M src/tie_robot_control/include/tie_robot_control/moduan/moduan_ros_callbacks.hpp
 M src/tie_robot_control/include/tie_robot_control/moduan/runtime_state.hpp
 M src/tie_robot_control/src/moduan/error_handling.cpp
 M src/tie_robot_control/src/moduan/linear_module_executor.cpp
 M src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp
 M src/tie_robot_control/src/moduan/runtime_state.cpp
 M src/tie_robot_control/test/test_single_point_bind_chain.py
 M src/tie_robot_hw/src/driver/cabin_driver.cpp
 M src/tie_robot_msgs/action/StartGlobalWorkTask.action
 M src/tie_robot_msgs/msg/PointCoords.msg
 M src/tie_robot_msgs/srv/SetExecutionMode.srv
 M src/tie_robot_msgs/srv/StartGlobalWork.srv
 M src/tie_robot_perception/config/gripper_tf.yaml
 M src/tie_robot_perception/config/robot_home_tf.yaml
 M src/tie_robot_perception/data/manual_workspace_quad.json
 M src/tie_robot_perception/scripts/robot_tf_broadcaster.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/state.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/tcp_display.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py
 M src/tie_robot_perception/test/test_gripper_tf_broadcaster.py
 M src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py
 M src/tie_robot_perception/test/test_robot_tf_broadcaster.py
 M src/tie_robot_process/data/bind_execution_memory.json
 M src/tie_robot_process/data/cabin_state.json
 M src/tie_robot_process/data/pseudo_slam_bind_path.json
 M src/tie_robot_process/data/pseudo_slam_points.json
 M src/tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp
 M src/tie_robot_process/src/planning/dynamic_bind_planning.cpp
 M src/tie_robot_process/src/suoqu/cabin_transport.cpp
 M src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp
 M src/tie_robot_process/src/suoqu/service_orchestration.cpp
 M src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp
 M src/tie_robot_process/src/suoquNode.cpp
 M src/tie_robot_process/test/test_dynamic_bind_planning.cpp
 M src/tie_robot_process/test/test_motion_chain_signal_guard.py
 M src/tie_robot_process/test/test_scan_artifact_write_guard.py
 M src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js
 M src/tie_robot_web/frontend/src/config/controlPanelCatalog.js
 M src/tie_robot_web/frontend/src/config/imageTopicCatalog.js
 M src/tie_robot_web/frontend/src/config/legacyCommandCatalog.js
 M src/tie_robot_web/frontend/src/config/topicLayerCatalog.js
 M src/tie_robot_web/frontend/src/config/topicRegistry.js
 M src/tie_robot_web/frontend/src/config/visualRecognitionMode.js
 M src/tie_robot_web/frontend/src/controllers/LegacyCommandController.js
 M src/tie_robot_web/frontend/src/controllers/RosConnectionController.js
 M src/tie_robot_web/frontend/src/controllers/StatusMonitorController.js
 M src/tie_robot_web/frontend/src/controllers/TaskActionController.js
 M src/tie_robot_web/frontend/src/controllers/TcpLinearRemoteController.js
 M src/tie_robot_web/frontend/src/controllers/TopicLayerController.js
 M src/tie_robot_web/frontend/src/styles/app.css
 M src/tie_robot_web/frontend/src/ui/UIController.js
 M src/tie_robot_web/frontend/src/utils/irImageUtils.js
 M src/tie_robot_web/frontend/src/utils/storage.js
 M src/tie_robot_web/frontend/src/utils/tcpWorkspaceOverlay.js
 M src/tie_robot_web/frontend/src/views/Scene3DView.js
 M src/tie_robot_web/frontend/src/views/WorkspaceCanvasView.js
 M src/tie_robot_web/frontend/test/gripperTfCalibration.test.mjs
 M src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs
 M src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs
 M src/tie_robot_web/frontend/test/taskActionController.test.mjs
 M src/tie_robot_web/frontend/test/tcpWorkspaceOverlay.test.mjs
 M src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
 M src/tie_robot_web/frontend/test/workspaceCanvasViewOverlay.test.mjs
 M src/tie_robot_web/scripts/workspace_picker_web_server.py
 M src/tie_robot_web/src/web_bridge/action_bridge.cpp
 M src/tie_robot_web/test/test_workspace_picker_web.py
 D src/tie_robot_web/web/assets/app/index-C13nXWGU.js
 D src/tie_robot_web/web/assets/app/index-nTJbOtOb.css
... omitted 68 more lines ...
```

## 恢复命令

```bash
python3 scripts/agent_memory.py recover
python3 scripts/agent_memory.py check
```
