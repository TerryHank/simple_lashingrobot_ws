# 大工程全局接口包重构实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 新建 `robot_interface_hub` 包，统一承接大工程全局 `msg/srv/action/launch`、前端、前端 TCP API 桥接和系统控制面，并让 `chassis_ctrl` 及其他包改为依赖这个新包的接口类型与启动入口。

**架构：** `robot_interface_hub` 作为控制面包，负责接口定义、launch 编排、前端静态资源、web 脚本和 `topics_transfer` 网关；`chassis_ctrl` 收口为执行实现包，只保留驱动层、算法层、执行节点、配置与数据。迁移过程中保持 ROS topic/service/action 名称不变，只切换包归属和消息类型命名空间。

**技术栈：** ROS Noetic、catkin、C++、Python、静态前端资源、roslaunch、message_generation、actionlib_msgs

---

## 文件结构

### 新建文件/目录

- `src/robot_interface_hub/package.xml`
  - 新控制面包元数据与依赖声明
- `src/robot_interface_hub/CMakeLists.txt`
  - 消息、服务、动作生成；`topictransNode` 构建；Python 脚本安装
- `src/robot_interface_hub/msg/`
  - 迁入所有全局消息定义
- `src/robot_interface_hub/srv/`
  - 迁入所有全局服务定义
- `src/robot_interface_hub/action/`
  - 迁入所有全局动作定义
- `src/robot_interface_hub/launch/`
  - 迁入全局 launch
- `src/robot_interface_hub/src/topics_transfer.cpp`
  - 前端到后端的 ROS API 网关
- `src/robot_interface_hub/scripts/workspace_picker_web_server.py`
  - 前端静态资源服务脚本
- `src/robot_interface_hub/scripts/workspace_picker_web_open.py`
  - 前端自动打开脚本
- `src/robot_interface_hub/scripts/system_log_mux.py`
  - 系统日志聚合脚本
- `src/robot_interface_hub/web/`
  - 迁入新前端资源

### 修改文件

- `src/chassis_ctrl/CMakeLists.txt`
  - 删除自身消息生成；改为依赖 `robot_interface_hub`
- `src/chassis_ctrl/package.xml`
  - 切换消息依赖到 `robot_interface_hub`
- `src/chassis_ctrl/src/suoquNode.cpp`
  - 服务/消息类型 include 与命名空间改为 `robot_interface_hub`
- `src/chassis_ctrl/src/moduanNode.cpp`
  - 服务/消息类型 include 与命名空间改为 `robot_interface_hub`
- `src/chassis_ctrl/src/client.cpp`
  - 服务类型改为 `robot_interface_hub`
- `src/chassis_ctrl/scripts/pointAI.py`
  - Python 消息/服务 import 改为 `robot_interface_hub`
- `src/chassis_ctrl/scripts/stable_point_tf_broadcaster.py`
  - Python 消息 import 改为 `robot_interface_hub`
- `src/chassis_ctrl/scripts/workspace_picker_web_server.py`
  - 删除或改为迁移后 stub（本计划优先删除）
- `src/chassis_ctrl/scripts/workspace_picker_web_open.py`
  - 删除或改为迁移后 stub（本计划优先删除）
- `src/chassis_ctrl/scripts/system_log_mux.py`
  - 删除或改为迁移后 stub（本计划优先删除）
- `src/APP/dist/index.html`
  - 新前端入口链接改到新包路径或新统一 URL
- `src/chassis_ctrl/test/test_pointai_order.py`
  - 断言新包存在，接口命名空间迁移
- `src/chassis_ctrl/test/test_workspace_picker_web_launch.py`
  - launch 与 web 脚本路径改到新包
- `src/chassis_ctrl/test/test_workspace_picker_web_runtime.py`
  - web 脚本路径改到新包
- `src/chassis_ctrl/test/test_system_log_mux.py`
  - `system_log_mux.py` 路径改到新包
- `src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`
  - `run.launch/api.launch` 所在包改到新包

### 删除或迁出后不再保留的目录

- `src/chassis_ctrl/msg/`
- `src/chassis_ctrl/srv/`
- `src/chassis_ctrl/action/`
- `src/chassis_ctrl/launch/`
- `src/ir_workspace_picker_web/`

---

## 任务 1：新增 `robot_interface_hub` 包骨架

**文件：**
- 创建：`src/robot_interface_hub/package.xml`
- 创建：`src/robot_interface_hub/CMakeLists.txt`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

```python
def test_robot_interface_hub_package_exists_and_owns_global_interfaces(self):
    hub_dir = REPO_ROOT / "src" / "robot_interface_hub"
    self.assertTrue((hub_dir / "package.xml").exists())
    self.assertTrue((hub_dir / "CMakeLists.txt").exists())
    self.assertTrue((hub_dir / "msg").exists())
    self.assertTrue((hub_dir / "srv").exists())
    self.assertTrue((hub_dir / "action").exists())
    self.assertTrue((hub_dir / "launch").exists())
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_interface_hub_package_exists_and_owns_global_interfaces`

预期：`FAIL`，提示 `src/robot_interface_hub/package.xml` 不存在。

- [ ] **步骤 3：创建最小包骨架**

```xml
<!-- src/robot_interface_hub/package.xml -->
<package format="2">
  <name>robot_interface_hub</name>
  <version>0.0.0</version>
  <description>Global interface hub package</description>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>rosgraph_msgs</build_depend>
  <exec_depend>message_runtime</exec_depend>
</package>
```

```cmake
# src/robot_interface_hub/CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(robot_interface_hub)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  rosgraph_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
```

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_interface_hub_package_exists_and_owns_global_interfaces`

预期：`OK`

---

## 任务 2：迁移 `msg/srv/action` 到新包并接入生成链

**文件：**
- 迁移：`src/chassis_ctrl/msg/*`
- 迁移：`src/chassis_ctrl/srv/*`
- 迁移：`src/chassis_ctrl/action/*`
- 修改：`src/robot_interface_hub/CMakeLists.txt`
- 修改：`src/robot_interface_hub/package.xml`
- 修改：`src/chassis_ctrl/CMakeLists.txt`
- 修改：`src/chassis_ctrl/package.xml`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

```python
def test_robot_interface_hub_cmake_generates_messages_services_and_actions(self):
    hub_cmake = (REPO_ROOT / "src/robot_interface_hub/CMakeLists.txt").read_text(encoding="utf-8")
    self.assertIn("add_service_files(", hub_cmake)
    self.assertIn("add_action_files(", hub_cmake)
    self.assertIn("add_message_files(", hub_cmake)
    self.assertIn("generate_messages(", hub_cmake)
    self.assertIn("PointCoords.msg", hub_cmake)
    self.assertIn("ProcessImage.srv", hub_cmake)
    self.assertIn("Move.action", hub_cmake)

def test_chassis_ctrl_no_longer_generates_its_own_messages(self):
    chassis_cmake = (REPO_ROOT / "src/chassis_ctrl/CMakeLists.txt").read_text(encoding="utf-8")
    self.assertNotIn("add_message_files(", chassis_cmake)
    self.assertNotIn("add_service_files(", chassis_cmake)
    self.assertNotIn("add_action_files(", chassis_cmake)
    self.assertIn("robot_interface_hub", chassis_cmake)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_interface_hub_cmake_generates_messages_services_and_actions src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_chassis_ctrl_no_longer_generates_its_own_messages`

预期：`FAIL`

- [ ] **步骤 3：迁移接口目录并改构建**

```cmake
# src/robot_interface_hub/CMakeLists.txt
add_service_files(FILES
  SingleMove.srv
  Pathguihua.srv
  linear_module_move.srv
  ProcessImage.srv
  PlaneDetection.srv
  ExecuteBindPoints.srv
  ResetPLCWarning.srv
  Resetzero.srv
  MotionControl.srv
  StartGlobalWork.srv
  StartPseudoSlamScan.srv
)

add_action_files(DIRECTORY action FILES Move.action)

add_message_files(FILES
  motion.msg
  linear_module_move_single.msg
  linear_module_move_all.msg
  PointCoords.msg
  PointsArray.msg
  cabin_move_single.msg
  cabin_move_all.msg
  cabin_calibration.msg
  area_choose.msg
  linear_module_upload.msg
  cabin_upload.msg
  LashingArray.msg
  LashingCoords.msg
  Area_info.msg
  AreaProgress.msg
)

generate_messages(DEPENDENCIES geometry_msgs rosgraph_msgs sensor_msgs std_msgs actionlib_msgs)
```

```cmake
# src/chassis_ctrl/CMakeLists.txt
find_package(catkin REQUIRED COMPONENTS
  robot_interface_hub
  ...
)

catkin_package(
  CATKIN_DEPENDS robot_interface_hub ...
)
```

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 3：把 `chassis_ctrl` C++ 节点的接口类型切到 `robot_interface_hub`

**文件：**
- 修改：`src/chassis_ctrl/src/suoquNode.cpp`
- 修改：`src/chassis_ctrl/src/moduanNode.cpp`
- 修改：`src/chassis_ctrl/src/client.cpp`
- 修改：`src/chassis_ctrl/CMakeLists.txt`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

```python
def test_suoqu_and_moduan_use_robot_interface_hub_message_types(self):
    suoqu_text = (CHASSIS_CTRL_DIR / "src" / "suoquNode.cpp").read_text(encoding="utf-8")
    moduan_text = (CHASSIS_CTRL_DIR / "src" / "moduanNode.cpp").read_text(encoding="utf-8")
    self.assertIn("robot_interface_hub::ProcessImage", suoqu_text)
    self.assertIn("robot_interface_hub::ExecuteBindPoints", suoqu_text)
    self.assertIn("robot_interface_hub::PointCoords", moduan_text)
    self.assertNotIn("chassis_ctrl::ProcessImage", suoqu_text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_and_moduan_use_robot_interface_hub_message_types`

预期：`FAIL`

- [ ] **步骤 3：批量替换类型命名空间**

```cpp
// before
chassis_ctrl::ProcessImage scan_srv;
ros::ServiceClient client = nh.serviceClient<chassis_ctrl::ExecuteBindPoints>("/moduan/sg_precomputed");

// after
robot_interface_hub::ProcessImage scan_srv;
ros::ServiceClient client = nh.serviceClient<robot_interface_hub::ExecuteBindPoints>("/moduan/sg_precomputed");
```

```cpp
// before
chassis_ctrl::PointCoords point;

// after
robot_interface_hub::PointCoords point;
```

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 4：把 Python 节点的接口 import 切到 `robot_interface_hub`

**文件：**
- 修改：`src/chassis_ctrl/scripts/pointAI.py`
- 修改：`src/chassis_ctrl/scripts/stable_point_tf_broadcaster.py`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

```python
def test_python_nodes_import_interfaces_from_robot_interface_hub(self):
    pointai_text = (CHASSIS_CTRL_DIR / "scripts" / "pointAI.py").read_text(encoding="utf-8")
    stable_text = (CHASSIS_CTRL_DIR / "scripts" / "stable_point_tf_broadcaster.py").read_text(encoding="utf-8")
    self.assertIn("from robot_interface_hub.srv import", pointai_text)
    self.assertIn("from robot_interface_hub.msg import", pointai_text)
    self.assertIn("from robot_interface_hub.msg import AreaProgress", stable_text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_python_nodes_import_interfaces_from_robot_interface_hub`

预期：`FAIL`

- [ ] **步骤 3：切换 Python import**

```python
# before
from chassis_ctrl.srv import ProcessImage, ProcessImageResponse
from chassis_ctrl.msg import PointsArray, PointCoords

# after
from robot_interface_hub.srv import ProcessImage, ProcessImageResponse
from robot_interface_hub.msg import PointsArray, PointCoords
```

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 5：迁移 `topics_transfer` 到新包并让新包构建 `topictransNode`

**文件：**
- 迁移：`src/chassis_ctrl/src/topics_transfer.cpp` -> `src/robot_interface_hub/src/topics_transfer.cpp`
- 修改：`src/robot_interface_hub/CMakeLists.txt`
- 修改：`src/chassis_ctrl/CMakeLists.txt`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

```python
def test_topic_transfer_node_moves_to_robot_interface_hub(self):
    hub_src = REPO_ROOT / "src/robot_interface_hub/src/topics_transfer.cpp"
    self.assertTrue(hub_src.exists())
    chassis_cmake = (REPO_ROOT / "src/chassis_ctrl/CMakeLists.txt").read_text(encoding="utf-8")
    hub_cmake = (REPO_ROOT / "src/robot_interface_hub/CMakeLists.txt").read_text(encoding="utf-8")
    self.assertNotIn("add_executable(topictransNode", chassis_cmake)
    self.assertIn("add_executable(topictransNode", hub_cmake)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_topic_transfer_node_moves_to_robot_interface_hub`

预期：`FAIL`

- [ ] **步骤 3：迁移并改构建**

```cmake
# src/robot_interface_hub/CMakeLists.txt
add_executable(topictransNode src/topics_transfer.cpp)
add_dependencies(topictransNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topictransNode ${catkin_LIBRARIES})
```

```cmake
# src/chassis_ctrl/CMakeLists.txt
# 删除
add_executable(topictransNode src/topics_transfer.cpp)
```

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 6：迁移 launch、前端资源和控制面脚本到新包

**文件：**
- 迁移：`src/chassis_ctrl/launch/*` -> `src/robot_interface_hub/launch/*`
- 迁移：`src/chassis_ctrl/scripts/workspace_picker_web_server.py`
- 迁移：`src/chassis_ctrl/scripts/workspace_picker_web_open.py`
- 迁移：`src/chassis_ctrl/scripts/system_log_mux.py`
- 迁移：`src/ir_workspace_picker_web/*` -> `src/robot_interface_hub/web/*`
- 修改：`src/APP/dist/index.html`
- 测试：`src/chassis_ctrl/test/test_workspace_picker_web_launch.py`
- 测试：`src/chassis_ctrl/test/test_workspace_picker_web_runtime.py`
- 测试：`src/chassis_ctrl/test/test_system_log_mux.py`
- 测试：`src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`

- [ ] **步骤 1：编写失败的测试**

```python
def test_robot_interface_hub_owns_launch_and_web_assets(self):
    hub_dir = REPO_ROOT / "src/robot_interface_hub"
    self.assertTrue((hub_dir / "launch" / "run.launch").exists())
    self.assertTrue((hub_dir / "launch" / "api.launch").exists())
    self.assertTrue((hub_dir / "scripts" / "workspace_picker_web_server.py").exists())
    self.assertTrue((hub_dir / "scripts" / "workspace_picker_web_open.py").exists())
    self.assertTrue((hub_dir / "scripts" / "system_log_mux.py").exists())
    self.assertTrue((hub_dir / "web" / "index.html").exists())
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_interface_hub_owns_launch_and_web_assets`

预期：`FAIL`

- [ ] **步骤 3：迁移文件并改路径**

```python
# before
return Path(__file__).resolve().parents[2] / "ir_workspace_picker_web"

# after
return Path(__file__).resolve().parents[1] / "web"
```

```xml
<!-- before -->
<include file="$(find chassis_ctrl)/launch/api.launch" />

<!-- after -->
<include file="$(find robot_interface_hub)/launch/api.launch" />
```

```html
<!-- before -->
href="../../ir_workspace_picker_web/index.html"

<!-- after -->
href="../../robot_interface_hub/web/index.html"
```

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_workspace_picker_web_launch \
  src.chassis_ctrl.test.test_workspace_picker_web_runtime \
  src.chassis_ctrl.test.test_system_log_mux \
  src.chassis_ctrl.test.test_gripper_tf_broadcaster
```

预期：`OK`

---

## 任务 7：全量编译联调与回归验证

**文件：**
- 修改：所有编译错误对应文件
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`
- 测试：`src/chassis_ctrl/test/test_workspace_picker_web_launch.py`
- 测试：`src/chassis_ctrl/test/test_workspace_picker_web_runtime.py`
- 测试：`src/chassis_ctrl/test/test_system_log_mux.py`
- 测试：`src/chassis_ctrl/test/test_gripper_tf_broadcaster.py`

- [ ] **步骤 1：运行接口迁移相关测试子集**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order \
  src.chassis_ctrl.test.test_workspace_picker_web_launch \
  src.chassis_ctrl.test.test_workspace_picker_web_runtime \
  src.chassis_ctrl.test.test_system_log_mux \
  src.chassis_ctrl.test.test_gripper_tf_broadcaster
```

预期：如果失败，失败项应只剩真实迁移遗漏，而不是找不到包骨架。

- [ ] **步骤 2：编译整个工作区**

运行：

```bash
source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="ScepterROS;robot_interface_hub;chassis_ctrl" -j2
```

预期：`exit 0`

- [ ] **步骤 3：运行关键回归测试**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_start_work_supports_precomputed_and_live_visual_execution_modes \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_execution_chain_routes_pose_moves_through_cabin_driver \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_moduan_precomputed_execution_routes_points_through_linear_module_driver
```

预期：`OK`

- [ ] **步骤 4：整理遗留兼容项**

需要确认并处理：

- `src/chassis_ctrl/msg|srv|action|launch` 已被迁空或删除
- `src/ir_workspace_picker_web` 已迁空或删除
- 测试中不再硬编码 `chassis_ctrl` 作为控制面 launch/web 所在包

---

## 自检

- 规格覆盖度：设计文档中的控制面包职责、迁移边界、依赖方向、launch 归属、前端资源归属、消息命名空间切换都已有对应任务。
- 占位符扫描：计划中没有 `TODO`、`后续实现`、`类似任务` 这类占位符。
- 类型一致性：统一使用 `robot_interface_hub::*` 作为迁移后的消息/服务/动作命名空间，统一使用 `topictransNode` 作为网关可执行名。
