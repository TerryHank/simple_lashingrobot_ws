# 重构映射

## 已拆的巨型业务

- `tie_robot_control/src/moduanNode.cpp`
  现为薄入口，实现在 `tie_robot_control/src/moduan/`
- `tie_robot_perception/scripts/pointAI.py`
  现为薄入口，实现在 `tie_robot_perception/src/tie_robot_perception/pointai/`
- `tie_robot_perception/src/camera/scepter_manager.cpp`
  已拆到 `src/camera/device_session.cpp`、`frame_publish.cpp`、`intrinsics.cpp`
- `tie_robot_web/web/ir_workspace_picker.mjs`
  已退役为旧静态入口，现由 `frontend/src/app/TieRobotFrontApp.js` 及其控制器/视图层替代
- `tie_robot_web/frontend`
  现已按 `robot_viewer` 风格改成“整页三维背景 + 顶部工具条 + 浮动业务面板”
- `tie_robot_web/frontend/src/views/Scene3DView.js`
  负责整页背景三维场景、机器、TF、点云、绑扎点和规划点展示
- `tie_robot_web/frontend/src/controllers/TopicLayerController.js`
  负责 `只显示点云 / 只显示绑扎点 / 点云+绑扎点 / 规划点 / 只显示机器 / 全开` 等显示模式
- `tie_robot_web/frontend/src/ui/UIController.js`
  负责顶部工具条、背景层容器和浮动业务面板布局
- `tie_robot_web/frontend/src/ui/PanelManager.js`
  负责新布局下的浮动面板拖拽
- `tie_robot_web/src/topics_transfer.cpp`
  已拆到 `src/web_bridge/`
- `tie_robot_process/src/planning/dynamic_bind_planning.cpp`
  已拆到 `dynamic_bind_geometry.cpp`、`dynamic_bind_grouping.cpp`、`dynamic_bind_builder.cpp`
- `tie_robot_process/src/suoquNode.cpp`
  现已拆出 `src/suoqu/cabin_transport.cpp`
  现已拆出 `src/suoqu/pseudo_slam_markers.cpp`
  现已拆出 `src/suoqu/pseudo_slam_scan_processing.cpp`
  现已拆出 `src/suoqu/service_orchestration.cpp`
  现已拆出 `src/suoqu/area_execution.cpp`
  现已拆出 `src/suoqu/execution_memory_store.cpp`
  现已拆出 `src/suoqu/bind_path_store.cpp`
  `src/suoqu/planned_bind_and_memory.inc` 仅保留过渡占位

## 当前策略

- 先做纯等价结构化，不主动改动作链行为
- 难拆的 `suoquNode.cpp` 先用“外提实现片段 + 专项模块”的方式降体积
- Marker 状态、扫描算法和 service 编排已拆离主文件
- 账本读写、区域执行和执行记忆已拆离主文件
- 新前端已经不再在 `web/` 下直接堆业务脚本，而是先在 `frontend/` 源码层结构化，再构建到 `web/`
- 当前剩余的大热点转向 `pseudo_slam_scan_processing.cpp` 和 `suoquNode.cpp` 里的流程主线
