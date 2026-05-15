# Agent Power-Loss Checkpoint

- 更新时间：2026-05-15 08:04:43
- HEAD：`c99cd8d`
- 分支：`slam`

## 当前任务

- 收紧动态绑扎路径规划，使每组中心与线性模组工作区中心对齐

## 下一步

- 先补动态绑扎规划测试，再改 candidate pose / 起点对齐逻辑并验证

## 影响范围

- `src/tie_robot_process/src/planning/dynamic_bind_geometry.cpp src/tie_robot_process/src/planning/dynamic_bind_planning.cpp src/tie_robot_process/src/suoqu/area_execution.cpp src/tie_robot_process/test/test_dynamic_bind_planning.cpp docs/agent_memory/checkpoint.md`

## 最近验证

- `尚未开始`

## 注意事项

- 用户要求保留断点以便回退

## Git 状态摘要

```text
M CHANGELOG.md
 M docs/agent_memory/checkpoint.md
 M docs/agent_memory/current.md
 M docs/agent_memory/session_log.md
 M docs/architecture/ros_interface_migration_map.yaml
 M docs/architecture/ros_interface_naming_plan.md
 M src/tie_robot_bringup/launch/algorithm_stack.launch
 M src/tie_robot_control/src/moduan/linear_module_executor.cpp
 M src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp
 M src/tie_robot_control/test/test_single_point_bind_chain.py
 M src/tie_robot_perception/config/bind_point_classification.yaml
 M src/tie_robot_perception/config/gripper_tf.yaml
 M src/tie_robot_perception/data/bind_classification_events.jsonl
 M src/tie_robot_perception/data/manual_workspace_quad.json
 M src/tie_robot_perception/src/tie_robot_perception/pointai/bind_point_classification.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/execution_refine_hough.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/node.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/scan_surface_dp.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/state.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py
 M src/tie_robot_perception/test/test_bind_point_classification.py
 M src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py
 M src/tie_robot_perception/test/test_scan_surface_dp_runtime.py
 M src/tie_robot_process/data/bind_execution_memory.json
 M src/tie_robot_process/data/cabin_state.json
 M src/tie_robot_process/data/pseudo_slam_bind_path.json
 M src/tie_robot_process/data/pseudo_slam_points.json
 M src/tie_robot_process/src/suoqu/pseudo_slam_scan_processing.cpp
 M src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp
 M src/tie_robot_process/src/suoquNode.cpp
 M src/tie_robot_process/test/test_motion_chain_signal_guard.py
 M src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js
 M src/tie_robot_web/frontend/src/config/controlPanelCatalog.js
 M src/tie_robot_web/frontend/src/config/imageTopicCatalog.js
 M src/tie_robot_web/frontend/src/config/legacyCommandCatalog.js
 M src/tie_robot_web/frontend/src/config/topicRegistry.js
 M src/tie_robot_web/frontend/src/config/visualRecognitionMode.js
 M src/tie_robot_web/frontend/src/controllers/AreaNavigationController.js
 M src/tie_robot_web/frontend/src/controllers/RosConnectionController.js
 M src/tie_robot_web/frontend/src/controllers/StatusMonitorController.js
 M src/tie_robot_web/frontend/src/ui/UIController.js
 M src/tie_robot_web/frontend/src/utils/storage.js
 M src/tie_robot_web/frontend/test/areaNavigationController.test.mjs
 M src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs
 M src/tie_robot_web/frontend/test/jumpBindToggleController.test.mjs
 M src/tie_robot_web/frontend/test/statusChipPressBehavior.test.mjs
 M src/tie_robot_web/frontend/test/statusMonitorController.test.mjs
 M src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
 M src/tie_robot_web/help/guide/operator-manual.md
 M src/tie_robot_web/help/guide/ros-graph.md
 D src/tie_robot_web/web/assets/app/index-B8n4JqXk.js
 D src/tie_robot_web/web/assets/app/index-BUxTumIZ.js
 D src/tie_robot_web/web/assets/app/index-Bouq5NRI.js
 D src/tie_robot_web/web/assets/app/index-Bvps4a7l.js
 D src/tie_robot_web/web/assets/app/index-C_YEfQb3.js
 D src/tie_robot_web/web/assets/app/index-Cl3ffcRD.js
 D src/tie_robot_web/web/assets/app/index-Cn1XYqK-.js
 D src/tie_robot_web/web/assets/app/index-CqSg_nJc.js
 D src/tie_robot_web/web/assets/app/index-DV33-xB5.js
 D src/tie_robot_web/web/assets/app/index-Dp6_eas6.js
 D src/tie_robot_web/web/assets/app/index-MxVDk7lp.js
 D src/tie_robot_web/web/assets/app/index-YLNGTL91.js
 M src/tie_robot_web/web/help/404.html
 D src/tie_robot_web/web/help/assets/guide_ros-graph.md.BSpIFn8T.js
 D src/tie_robot_web/web/help/assets/guide_ros-graph.md.BSpIFn8T.lean.js
 D src/tie_robot_web/web/help/assets/index.md.CCLAcXmS.js
 D src/tie_robot_web/web/help/assets/index.md.CCLAcXmS.lean.js
 M src/tie_robot_web/web/help/camera-sdk/index.html
 M src/tie_robot_web/web/help/camera-sdk/vendor-vzense/README.html
 M src/tie_robot_web/web/help/camera-sdk/vendor-vzense/en/ApplicationNote/DS86&87/DS86-application-note.html
 M src/tie_robot_web/web/help/camera-sdk/vendor-vzense/en/ApplicationNote/NYX650&660/NYX650-application-note.html
 M src/tie_robot_web/web/help/camera-sdk/vendor-vzense/en/ProductIntroduction/DS86,87.html
 M src/tie_robot_web/web/help/camera-sdk/vendor-vzense/en/ProductIntroduction/NYX650,660.html
 M src/tie_robot_web/web/help/camera-sdk/vendor-vzense/en/Quickstart/Quickstart.html
... omitted 84 more lines ...
```

## 恢复命令

```bash
python3 scripts/agent_memory.py recover
python3 scripts/agent_memory.py check
```
