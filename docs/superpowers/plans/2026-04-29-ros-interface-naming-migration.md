# ROS 接口命名迁移实现计划

> **面向 AI 代理的工作者：** 本计划承接 `docs/architecture/ros_interface_naming_plan.md` 和 `docs/architecture/ros_interface_migration_map.yaml`。实施时必须小步迁移，旧接口先做 alias，确认前端和现场链路稳定后再删除。

**目标：** 把全工程 ROS Topic、Service、Action 从历史混合命名迁移到 `/hw`、`/perception`、`/control`、`/process`、`/safety`、`/calibration`、`/system` 分层命名，并把视觉识别拆成驱动原子层和算法层。

**架构：** `tie_robot_hw` 暴露设备原子能力；`tie_robot_perception` 暴露 PR-FPRG 与相机数据处理；`tie_robot_control` 暴露短控制动作；`tie_robot_process` 暴露流程 Action；`tie_robot_web` 只做前端桥接和系统守护。

**权威映射表：** `docs/architecture/ros_interface_migration_map.yaml`

---

## 任务 1：增加接口命名静态检查

**文件：**

- 已创建：`scripts/check_ros_interface_names.py`
- 已创建：`src/tie_robot_bringup/test/test_ros_interface_names.py`

- [x] 读取 `docs/architecture/ros_interface_migration_map.yaml`
- [x] 扫描 `src/tie_robot_perception`、`src/tie_robot_control`、`src/tie_robot_process`、`src/tie_robot_web/frontend/src`、`src/tie_robot_web/src`、`src/tie_robot_bringup/launch`
- [x] 对迁移表未登记的 ROS 接口字符串直接失败
- [x] 增加回归测试：当前工程覆盖完整、新增未登记 `/web/moduan/*` 会失败
- [ ] 后续 alias 落地后，收紧为旧名只允许出现在 alias 文件、测试白名单、迁移表和历史注释中
- [ ] 在完成每个迁移阶段后运行该检查

## 任务 2：视觉接口先做 canonical alias

**文件：**

- 已修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- 已修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- 已修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- 已修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`
- 已修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`
- 已修改：`src/tie_robot_perception/test/test_pointai_scan_only_pr_fprg.py`

- [x] 新增 `/perception/lashing/recognize_once`，内部复用当前 PR-FPRG 触发链
- [x] 新增 `/perception/lashing/result_image`，后端直接发布已画线、已画点的结果图
- [x] 新增 `/perception/lashing/points_camera`，替代 `/coordinate_point` 和 `/pointAI/manual_workspace_s2_points`
- [x] 新增 `/perception/lashing/workspace/quad_pixels`
- [x] 保留旧 `/pointAI/*` 和 `/coordinate_point` alias
- [ ] 删除视觉层对 `/web/moduan/send_odd_points` 的直接监听，改由流程层调用清晰接口
- [ ] 确认普通视觉触发不写本地绑扎点 JSON，只有扫描/识别位姿流程允许写

## 任务 3：前端视觉按钮切到 canonical 接口

**文件：**

- 已修改：`src/tie_robot_web/frontend/src/config/topicRegistry.js`
- 已修改：`src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- 已修改：`src/tie_robot_web/frontend/src/controllers/TaskActionController.js`
- 已修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- 已修改：`src/tie_robot_web/frontend/src/projectGraph/graphData.js`
- 已构建：`src/tie_robot_web/web`

- [x] “触发视觉识别”调用 `/perception/lashing/recognize_once`
- [x] 当前图层是 IR 时，识别结果覆盖当前 IR 图层；当前图层是 RGB 时，识别结果覆盖当前 RGB 图层
- [x] “清除识别结果”回到当前图层原始流
- [x] 前端不再订阅线条话题自行叠线，只显示后端 `/perception/lashing/result_image`
- [x] 构建前端静态产物

## 任务 4：索驱和线性模组驱动原子接口 alias

**文件：**

- 修改：`src/tie_robot_process/src/suoquNode.cpp`
- 修改：`src/tie_robot_process/src/suoqu/cabin_transport.cpp`
- 修改：`src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- 修改：`src/tie_robot_control/src/moduan/linear_module_executor.cpp`

- [ ] `/cabin/driver/*` 同时暴露 `/hw/cabin/driver/*`
- [ ] `/cabin/driver/raw_move` 同时暴露 `/hw/cabin/move_raw`
- [ ] `/cabin/motion/stop` 同时暴露 `/hw/cabin/stop`
- [ ] `/moduan/driver/*` 同时暴露 `/hw/linear_module/driver/*`
- [ ] `/moduan/driver/raw_execute_points` 同时暴露 `/hw/linear_module/execute_points_raw`
- [ ] 驱动层文件不新增扫描、PR-FPRG、全局作业、绑扎流程语义

## 任务 5：控制层和安全层接口 alias

**文件：**

- 修改：`src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- 修改：`src/tie_robot_process/src/suoquNode.cpp`
- 修改：`src/tie_robot_web/frontend/src/config/topicRegistry.js`

- [ ] `/moduan/single_move` alias 到 `/control/linear_module/move_once`
- [ ] `/moduan/sg_precomputed` alias 到 `/control/lashing/execute_points`
- [ ] `/moduan/sg_precomputed_fast` alias 到 `/control/lashing/execute_points_fast`
- [ ] `/web/moduan/forced_stop` 迁移到 `/safety/forced_stop/set`
- [ ] `/web/moduan/interrupt_stop` 迁移到 `/safety/pause/set`
- [ ] `/web/moduan/hand_sovle_warn` 迁移到 `/safety/manual_intervention/set`
- [ ] 修正 `hand_sovle_warn` 拼写历史问题，只保留兼容 alias

## 任务 6：流程 Action 收口

**文件：**

- 修改：`src/tie_robot_process/src/suoquNode.cpp`
- 修改：`src/tie_robot_web/src/web_bridge/action_bridge.cpp`
- 修改：`src/tie_robot_web/src/web_bridge/node_app.cpp`
- 修改：`src/tie_robot_msgs/action/*` 或新增 action

- [ ] `/process/scan/start` 统一承载扫描建图
- [ ] `/process/lashing/start_global_work` 统一承载全局作业
- [ ] `/process/lashing/run_bind_path_direct_test` 统一承载账本直执行测试
- [ ] `/process/lashing/bind_single_point` 统一承载“先视觉、再绑扎”的单点绑扎
- [ ] 前端 `/web/cabin/*` action 只保留兼容桥，不作为新代码入口

## 任务 7：系统、日志和标定命名迁移

**文件：**

- 修改：`src/tie_robot_web/src/web_bridge/process_control.cpp`
- 修改：`src/tie_robot_web/src/web_bridge/node_app.cpp`
- 修改：`src/tie_robot_web/scripts/system_log_mux.py`
- 修改：`src/tie_robot_perception/scripts/gripper_tf_broadcaster.py`

- [ ] `/web/system/*` alias 到 `/system/stack/*`
- [ ] `/system_log/*` alias 到 `/system/log/*`
- [ ] `/web/tf/*` alias 到 `/calibration/gripper_tf/*`

## 任务 8：调用方切 canonical，旧名退役

**文件：**

- 修改：`src/tie_robot_web/frontend/src/config/topicRegistry.js`
- 修改：各 ROS 节点硬编码调用点
- 修改：launch 文件

- [ ] 前端 registry 默认使用 canonical 名
- [ ] 流程层和控制层互调使用 canonical 名
- [ ] `rg` 确认旧名只存在于 alias 和迁移表
- [ ] 现场确认后删除 alias

## 验证命令

```bash
git diff --check
python3 -c 'import yaml, pathlib; yaml.safe_load(pathlib.Path("docs/architecture/ros_interface_migration_map.yaml").read_text())'
python3 scripts/check_ros_interface_names.py
source /opt/ros/noetic/setup.bash && catkin_make -j2
cd src/tie_robot_web/frontend && npm run build
```
