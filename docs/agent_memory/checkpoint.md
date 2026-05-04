# Agent Power-Loss Checkpoint

- 更新时间：2026-05-03 14:10:52
- HEAD：`04a5dc1`
- 分支：`slam`

## 当前任务

- 利用 codex-provider-sync 恢复本机 Codex 会话可见性

## 下一步

- 创建 provider-sync 备份后，将 ~/.codex/sessions、~/.codex/archived_sessions 和 state_5.sqlite 的 model_provider 从 openai 同步到当前 chaomeng-api，并修复 has_user_event/cwd metadata

## 影响范围

- `~/.codex/config.toml; ~/.codex/state_5.sqlite*; ~/.codex/sessions; ~/.codex/archived_sessions; ~/codex-provider-sync`

## 最近验证

- `已只读确认当前 provider=chaomeng-api；rollout sessions: chaomeng-api 10/openai 50；rollout archived_sessions: openai 45；SQLite sessions: chaomeng-api 10/openai 50；SQLite archived_sessions: openai 41`

## 注意事项

- codex-provider-sync CLI 因本机 Node v20 缺 node:sqlite 无法直接运行；按该仓库源码逻辑执行等价的备份与同步。

## Git 状态摘要

```text
M AGENTS.md
 M CHANGELOG.md
 M README.md
 M docs/agent_memory/README.md
 M docs/agent_memory/checkpoint.md
 M docs/agent_memory/current.md
 M docs/agent_memory/organism.md
 M docs/agent_memory/session_log.md
 M docs/handoff/2026-04-22_current_system_handoff.md
 M docs/handoff/2026-04-23_pr_fprg_knowledge.md
 M docs/releases/slam_v30/MANIFEST.md
 M docs/releases/slam_v30/SLAM_V30_HANDOFF.md
 M docs/releases/slam_v30/checksums.sha256
 M docs/releases/slam_v30/debug_frames_manifest.tsv
 M docs/releases/slam_v30/visual_modalities/metadata.json
 M docs/releases/slam_v30/visual_modalities/slam_v30_visual_modalities.bag
 M scripts/agent_memory.py
 M scripts/codex_session_guard.py
 M scripts/install_codex_session_guard_timer.sh
 M src/tie_robot_bringup/CMakeLists.txt
 M src/tie_robot_bringup/launch/tf_stack.launch
 M src/tie_robot_bringup/scripts/install_backend_service.sh
 M src/tie_robot_bringup/scripts/install_frontend_autostart.sh
 M src/tie_robot_bringup/systemd/tie-robot-backend-control.sudoers.in
 M src/tie_robot_bringup/systemd/tie-robot-backend.service.in
 M src/tie_robot_bringup/systemd/tie-robot-driver-camera.service.in
 M src/tie_robot_bringup/systemd/tie-robot-driver-moduan.service.in
 M src/tie_robot_bringup/systemd/tie-robot-driver-suoqu.service.in
 M src/tie_robot_bringup/systemd/tie-robot-rosbridge.service.in
 M src/tie_robot_bringup/test/test_architecture_cleanup.py
 M src/tie_robot_bringup/test/test_codex_session_guard.py
 M src/tie_robot_bringup/test/test_systemd_ros_master_ownership.py
 M src/tie_robot_description/URDF/model.urdf
 M src/tie_robot_hw/include/tie_robot_hw/driver/cabin_driver.hpp
 M src/tie_robot_hw/include/tie_robot_hw/driver/cabin_protocol.hpp
 M src/tie_robot_hw/include/tie_robot_hw/driver/cabin_tcp_transport.hpp
 M src/tie_robot_hw/src/driver/cabin_driver.cpp
 M src/tie_robot_hw/src/driver/cabin_protocol.cpp
 M src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp
 M src/tie_robot_hw/src/driver/linear_module_protocol.cpp
 M src/tie_robot_msgs/CMakeLists.txt
 M src/tie_robot_perception/config/gripper_tf.yaml
 M src/tie_robot_perception/data/manual_workspace_quad.json
 M src/tie_robot_perception/scripts/gripper_tf_broadcaster.py
 M src/tie_robot_perception/scripts/robot_tf_broadcaster.py
 M src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/image_buffers.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py
 M src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py
 M src/tie_robot_perception/test/test_gripper_tf_broadcaster.py
 M src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py
 M src/tie_robot_perception/test/test_scepter_sdk_split.py
 M src/tie_robot_perception/tools/pr_fprg_peak_supported_probe.py
 M src/tie_robot_perception/tools/pr_fprg_scheme_ablation_report.py
 M src/tie_robot_perception/tools/pr_fprg_scheme_comparison.py
 M src/tie_robot_perception/tools/pr_fprg_stage_ablation.py
 M src/tie_robot_perception/tools/pr_fprg_stage_ablation_report.py
 M src/tie_robot_process/data/cabin_state.json
 M src/tie_robot_process/include/tie_robot_process/suoqu/cabin_transport.hpp
 M src/tie_robot_process/src/suoqu/cabin_transport.cpp
 M src/tie_robot_process/src/suoquNode.cpp
 M src/tie_robot_process/test/test_motion_chain_signal_guard.py
 M src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js
 M src/tie_robot_web/frontend/src/config/legacyCommandCatalog.js
 M src/tie_robot_web/frontend/src/config/logTopicCatalog.js
 M src/tie_robot_web/frontend/src/config/systemControlCatalog.js
 M src/tie_robot_web/frontend/src/config/topicLayerCatalog.js
 M src/tie_robot_web/frontend/src/config/topicRegistry.js
 M src/tie_robot_web/frontend/src/controllers/CabinRemoteController.js
 M src/tie_robot_web/frontend/src/controllers/RosConnectionController.js
 M src/tie_robot_web/frontend/src/controllers/SystemControlController.js
 M src/tie_robot_web/frontend/src/controllers/TerminalController.js
 M src/tie_robot_web/frontend/src/controllers/TopicLayerController.js
 M src/tie_robot_web/frontend/src/styles/app.css
 M src/tie_robot_web/frontend/src/ui/UIController.js
 M src/tie_robot_web/frontend/src/utils/storage.js
 M src/tie_robot_web/frontend/src/views/Scene3DView.js
... omitted 79 more lines ...
```

## 恢复命令

```bash
python3 scripts/agent_memory.py recover
python3 scripts/agent_memory.py check
```
