# 绑扎机器人 ROS 工程分层重构实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将当前工程从 `robot_interface_hub + robot_hw_drivers + chassis_ctrl + robot_algorithm_layer` 的中间态，收口为 `tie_robot_msgs + tie_robot_description + tie_robot_hw + tie_robot_perception + tie_robot_control + tie_robot_process + tie_robot_bringup + tie_robot_web + robot_algorithm_layer`。

**架构：** 通过新增职责包、迁移主动作链文件、更新构建依赖和启动入口来完成分层；旧包通过 `CATKIN_IGNORE` 退役，不再参与 catkin。`robot_algorithm_layer` 暂时保持原名，避免无收益的大范围二次改名。

**技术栈：** ROS Noetic、catkin、C++14、Python 3、`actionlib`、`dynamic_reconfigure`

---

### 任务 1：用结构测试钉住目标包边界

**文件：**
- 创建：`src/chassis_ctrl/test/test_tie_robot_architecture_relayout.py`

- [x] **步骤 1：编写会先失败的结构测试**

新增断言覆盖：

- 新 8 个目标包存在且 `package.xml` 名称正确
- `tie_robot_msgs` 负责 `msg/srv/action`
- `tie_robot_web` 负责前端和 `topics_transfer`
- `tie_robot_bringup` 的 launch 切到新包名
- `tie_robot_perception` 负责相机驱动和视觉节点
- `tie_robot_hw` 只负责底层驱动库
- `tie_robot_control` / `tie_robot_process` 分别承载 `moduanNode` / `suoquNode`
- 旧包通过 `CATKIN_IGNORE` 退役

- [x] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest src.chassis_ctrl.test.test_tie_robot_architecture_relayout
```

预期：FAIL，提示新包和新边界尚不存在。

### 任务 2：创建 `tie_robot_msgs` 并迁出全局接口

**文件：**
- 创建：`src/tie_robot_msgs/package.xml`
- 创建：`src/tie_robot_msgs/CMakeLists.txt`
- 创建：`src/tie_robot_msgs/msg/*`
- 创建：`src/tie_robot_msgs/srv/*`
- 创建：`src/tie_robot_msgs/action/*`

- [ ] **步骤 1：创建新接口包骨架**

要求：

- `package.xml` 只声明接口生成所需依赖
- `CMakeLists.txt` 只保留 `add_message_files`、`add_service_files`、`add_action_files` 和 `generate_messages`

- [ ] **步骤 2：将 `robot_interface_hub` 里的 `msg/srv/action` 迁到 `tie_robot_msgs`**

- [ ] **步骤 3：把引用包名从 `robot_interface_hub` 改成 `tie_robot_msgs`**

覆盖文件：

- `src/robot_algorithm_layer/**/*`
- `src/chassis_ctrl/test/**/*`
- 本轮新增包中的 C++ / Python / JS 代码

### 任务 3：创建 `tie_robot_web` 和 `tie_robot_bringup`

**文件：**
- 创建：`src/tie_robot_web/package.xml`
- 创建：`src/tie_robot_web/CMakeLists.txt`
- 创建：`src/tie_robot_web/src/topics_transfer.cpp`
- 创建：`src/tie_robot_web/scripts/*`
- 创建：`src/tie_robot_web/web/*`
- 创建：`src/tie_robot_bringup/package.xml`
- 创建：`src/tie_robot_bringup/CMakeLists.txt`
- 创建：`src/tie_robot_bringup/launch/*`
- 修改：`src/APP/dist/index.html`

- [ ] **步骤 1：迁出 Web 前端和 API bridge**

从 `robot_interface_hub` 迁出：

- `src/topics_transfer.cpp`
- `scripts/system_log_mux.py`
- `scripts/workspace_picker_web_server.py`
- `scripts/workspace_picker_web_open.py`
- `web/`

- [ ] **步骤 2：迁出 bringup launch**

从 `robot_interface_hub/launch` 迁出：

- `run.launch`
- `api.launch`
- `pointai_tf_verify.launch`
- `suoquAndmoduan.launch`

- [ ] **步骤 3：更新 launch 的包引用**

要求：

- `run.launch` 用 `tie_robot_bringup` include `api.launch`
- `suoquNode` 使用 `tie_robot_process`
- `moduanNode` 使用 `tie_robot_control`
- `pointAI.py` 与 TF broadcaster 使用 `tie_robot_perception`
- Web 服务脚本使用 `tie_robot_web`
- 相机 launch include `tie_robot_perception`

- [ ] **步骤 4：更新前端 action/service 类型名**

将 `robot_interface_hub/*` 改为 `tie_robot_msgs/*`。

### 任务 4：创建 `tie_robot_hw` 和 `tie_robot_perception`

**文件：**
- 创建：`src/tie_robot_hw/package.xml`
- 创建：`src/tie_robot_hw/CMakeLists.txt`
- 创建：`src/tie_robot_hw/include/tie_robot_hw/driver/*`
- 创建：`src/tie_robot_hw/src/driver/*`
- 创建：`src/tie_robot_perception/package.xml`
- 创建：`src/tie_robot_perception/CMakeLists.txt`
- 创建：`src/tie_robot_perception/include/tie_robot_perception/camera/*`
- 创建：`src/tie_robot_perception/src/camera/*`
- 创建：`src/tie_robot_perception/scripts/*`
- 创建：`src/tie_robot_perception/config/*`
- 创建：`src/tie_robot_perception/launch/*`
- 创建：`src/tie_robot_perception/cfg/*`
- 创建：`src/tie_robot_perception/dependencies/*`
- 创建：`src/tie_robot_perception/srv/ConvertDepthToPointCloud.srv`

- [ ] **步骤 1：把索驱 / 线模驱动库迁到 `tie_robot_hw`**

要求：

- 只迁 `include/.../driver` 与 `src/driver`
- 导出库名改为 `tie_robot_hw_driver_core`
- 不再在 `tie_robot_hw` 中编译 `scepter_camera`

- [ ] **步骤 2：把相机驱动和视觉节点迁到 `tie_robot_perception`**

要求：

- 相机相关的 `src/camera`、`cfg`、`dependencies`、`srv`、launch 都归 `tie_robot_perception`
- `pointAI.py`、`gripper_tf_broadcaster.py`、`stable_point_tf_broadcaster.py` 迁入
- `gripper_tf.yaml`、`bind_point_classification.yaml` 迁入

- [ ] **步骤 3：更新数据与依赖路径**

要求：

- 相机 launch 的 `$(find robot_hw_drivers)` 改为 `$(find tie_robot_perception)`
- `pointAI.py` 中消息导入改为 `tie_robot_msgs`
- `pointAI.py` 中硬编码数据路径从 `src/chassis_ctrl/...` 改为新包路径

### 任务 5：创建 `tie_robot_control`、`tie_robot_process` 和 `tie_robot_description`

**文件：**
- 创建：`src/tie_robot_control/package.xml`
- 创建：`src/tie_robot_control/CMakeLists.txt`
- 创建：`src/tie_robot_control/include/common.hpp`
- 创建：`src/tie_robot_control/include/json.hpp`
- 创建：`src/tie_robot_control/src/moduanNode.cpp`
- 创建：`src/tie_robot_control/data/bindData.txt`
- 创建：`src/tie_robot_process/package.xml`
- 创建：`src/tie_robot_process/CMakeLists.txt`
- 创建：`src/tie_robot_process/include/common.hpp`
- 创建：`src/tie_robot_process/include/json.hpp`
- 创建：`src/tie_robot_process/src/suoquNode.cpp`
- 创建：`src/tie_robot_process/data/*`
- 创建：`src/tie_robot_description/package.xml`
- 创建：`src/tie_robot_description/CMakeLists.txt`
- 创建：`src/tie_robot_description/URDF/model.urdf`
- 创建：`src/tie_robot_description/rviz/*`

- [ ] **步骤 1：迁移 `moduanNode` 到 `tie_robot_control`**

要求：

- 更新消息头文件为 `tie_robot_msgs/*`
- 更新驱动头文件为 `tie_robot_hw/*`
- `bindData.txt` 路径切到 `src/tie_robot_control/data`

- [ ] **步骤 2：迁移 `suoquNode` 到 `tie_robot_process`**

要求：

- 更新消息头文件为 `tie_robot_msgs/*`
- 更新驱动头文件为 `tie_robot_hw/*`
- 更新所有账本和 fatal error 的绝对路径到 `src/tie_robot_process/data`

- [ ] **步骤 3：迁移 URDF 和 RViz 到 `tie_robot_description`**

### 任务 6：调整 `robot_algorithm_layer` 与新包依赖

**文件：**
- 修改：`src/robot_algorithm_layer/package.xml`
- 修改：`src/robot_algorithm_layer/CMakeLists.txt`
- 修改：`src/robot_algorithm_layer/include/robot_algorithm_layer/planning/dynamic_bind_planning.hpp`
- 修改：`src/robot_algorithm_layer/src/planning/dynamic_bind_planning.cpp`

- [ ] **步骤 1：将 `robot_algorithm_layer` 依赖从 `robot_interface_hub` 改为 `tie_robot_msgs`**

- [ ] **步骤 2：将 `PointCoords` 类型引用从 `robot_interface_hub::PointCoords` 改为 `tie_robot_msgs::PointCoords`**

### 任务 7：退役旧包

**文件：**
- 创建：`src/robot_interface_hub/CATKIN_IGNORE`
- 创建：`src/robot_hw_drivers/CATKIN_IGNORE`
- 创建：`src/chassis_ctrl/CATKIN_IGNORE`

- [ ] **步骤 1：给 3 个旧包添加 `CATKIN_IGNORE`**

- [ ] **步骤 2：保留旧目录作为迁移参考，不再参与 catkin**

### 任务 8：运行定向验证和构建

**文件：**
- 测试：`src/chassis_ctrl/test/test_tie_robot_architecture_relayout.py`

- [ ] **步骤 1：运行结构测试**

```bash
python3 -m unittest src.chassis_ctrl.test.test_tie_robot_architecture_relayout
```

预期：PASS

- [ ] **步骤 2：运行前端语法检查**

```bash
node --check src/tie_robot_web/web/ir_workspace_picker.mjs
```

预期：无语法错误输出

- [ ] **步骤 3：运行构建**

```bash
source /opt/ros/noetic/setup.bash && \
catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_description;tie_robot_hw;tie_robot_perception;tie_robot_control;tie_robot_process;tie_robot_bringup;tie_robot_web;robot_algorithm_layer" -j2
```

预期：`exit 0`
