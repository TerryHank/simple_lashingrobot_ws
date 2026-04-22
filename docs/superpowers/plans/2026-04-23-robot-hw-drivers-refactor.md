# 统一硬件驱动包重构实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 新建 `robot_hw_drivers` 包，把外部磁盘上的 Scepter 相机驱动和 `chassis_ctrl` 内的索驱 TCP / 线性模组 Modbus 驱动统一迁入这个底层硬件包，并让 `robot_interface_hub` 与 `chassis_ctrl` 切换到新的依赖边界。

**架构：** `robot_hw_drivers` 作为统一底层硬件包，承接相机驱动源码、vendor SDK 动态库、相机本地 service、驱动级 launch，以及索驱与线模的 transport / protocol / driver 库；`robot_interface_hub` 只负责控制面与顶层 launch；`chassis_ctrl` 只负责算法与应用执行节点，并通过 `robot_hw_drivers::driver::*` 使用底层驱动。

**技术栈：** ROS Noetic、catkin、C++、Python、dynamic_reconfigure、OpenCV、PCL、libmodbus、vendor 相机 SDK 动态库

---

## 文件结构

### 新建文件与目录

- `src/robot_hw_drivers/package.xml`
  - 统一底层硬件包元数据与依赖
- `src/robot_hw_drivers/CMakeLists.txt`
  - 相机 service / dynamic reconfigure / 底层驱动库 / 可执行构建
- `src/robot_hw_drivers/include/robot_hw_drivers/camera/scepter_manager.hpp`
  - 相机管理器头文件
- `src/robot_hw_drivers/include/robot_hw_drivers/driver/driver_types.hpp`
- `src/robot_hw_drivers/include/robot_hw_drivers/driver/cabin_tcp_transport.hpp`
- `src/robot_hw_drivers/include/robot_hw_drivers/driver/cabin_protocol.hpp`
- `src/robot_hw_drivers/include/robot_hw_drivers/driver/cabin_driver.hpp`
- `src/robot_hw_drivers/include/robot_hw_drivers/driver/modbus_transport.hpp`
- `src/robot_hw_drivers/include/robot_hw_drivers/driver/linear_module_protocol.hpp`
- `src/robot_hw_drivers/include/robot_hw_drivers/driver/linear_module_driver.hpp`
- `src/robot_hw_drivers/src/camera/scepter_driver.cpp`
- `src/robot_hw_drivers/src/camera/scepter_manager.cpp`
- `src/robot_hw_drivers/src/driver/cabin_tcp_transport.cpp`
- `src/robot_hw_drivers/src/driver/cabin_protocol.cpp`
- `src/robot_hw_drivers/src/driver/cabin_driver.cpp`
- `src/robot_hw_drivers/src/driver/modbus_transport.cpp`
- `src/robot_hw_drivers/src/driver/linear_module_protocol.cpp`
- `src/robot_hw_drivers/src/driver/linear_module_driver.cpp`
- `src/robot_hw_drivers/launch/scepter_camera.launch`
- `src/robot_hw_drivers/cfg/Sceptertof_roscpp.cfg`
- `src/robot_hw_drivers/srv/ConvertDepthToPointCloud.srv`
- `src/robot_hw_drivers/scripts/try.py`
- `src/robot_hw_drivers/setup.py`
- `src/robot_hw_drivers/dependencies/`
  - vendor 头文件、配置文件和动态库

### 修改文件

- `src/robot_interface_hub/package.xml`
  - 运行依赖从 `ScepterROS` 切到 `robot_hw_drivers`
- `src/robot_interface_hub/launch/api.launch`
  - include 新相机 launch
- `src/chassis_ctrl/CMakeLists.txt`
  - 删除本包驱动库构建，改为依赖 `robot_hw_drivers`
- `src/chassis_ctrl/package.xml`
  - 增加 `robot_hw_drivers` 依赖
- `src/chassis_ctrl/src/suoquNode.cpp`
  - include 与命名空间改到 `robot_hw_drivers::driver`
- `src/chassis_ctrl/src/moduanNode.cpp`
  - include 与命名空间改到 `robot_hw_drivers::driver`
- `src/chassis_ctrl/test/test_pointai_order.py`
  - 驱动包存在性、launch 归属、命名空间与构建依赖测试切换到新包

### 迁出后删除

- `src/ScepterROS`（当前为外部磁盘符号链接）
- `src/chassis_ctrl/include/chassis_ctrl/driver/`
- `src/chassis_ctrl/src/driver/`

---

## 任务 1：先用测试钉住新包边界与相机回收要求

**文件：**
- 修改：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

```python
def test_robot_hw_drivers_package_exists_and_owns_camera_and_driver_sources(self):
    hw_dir = REPO_ROOT / "src" / "robot_hw_drivers"
    self.assertTrue((hw_dir / "package.xml").exists())
    self.assertTrue((hw_dir / "CMakeLists.txt").exists())
    self.assertTrue((hw_dir / "launch" / "scepter_camera.launch").exists())
    self.assertTrue((hw_dir / "cfg" / "Sceptertof_roscpp.cfg").exists())
    self.assertTrue((hw_dir / "srv" / "ConvertDepthToPointCloud.srv").exists())
    self.assertTrue((hw_dir / "src" / "camera" / "scepter_driver.cpp").exists())
    self.assertTrue((hw_dir / "src" / "driver" / "cabin_driver.cpp").exists())

def test_scepterros_is_no_longer_an_external_symlink(self):
    legacy_path = REPO_ROOT / "src" / "ScepterROS"
    self.assertFalse(legacy_path.exists() and legacy_path.is_symlink())

def test_robot_interface_hub_api_launch_uses_robot_hw_drivers_camera_launch(self):
    api_launch = (REPO_ROOT / "src/robot_interface_hub/launch/api.launch").read_text(encoding="utf-8")
    self.assertIn('$(find robot_hw_drivers)/launch/scepter_camera.launch', api_launch)
    self.assertNotIn('$(find ScepterROS)/launch/scepter_camera.launch', api_launch)

def test_chassis_ctrl_cmakelists_uses_robot_hw_driver_core_instead_of_local_driver_sources(self):
    cmake_text = (REPO_ROOT / "src/chassis_ctrl/CMakeLists.txt").read_text(encoding="utf-8")
    self.assertIn("robot_hw_drivers", cmake_text)
    self.assertNotIn("src/driver/cabin_tcp_transport.cpp", cmake_text)
    self.assertNotIn("add_library(chassis_ctrl_driver", cmake_text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_hw_drivers_package_exists_and_owns_camera_and_driver_sources \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_scepterros_is_no_longer_an_external_symlink \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_interface_hub_api_launch_uses_robot_hw_drivers_camera_launch \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_chassis_ctrl_cmakelists_uses_robot_hw_driver_core_instead_of_local_driver_sources
```

预期：`FAIL`，提示 `src/robot_hw_drivers` 不存在且 `api.launch` 仍引用 `ScepterROS`。

- [ ] **步骤 3：只创建最小骨架与最小断言满足结构**

实现内容：

```text
mkdir -p src/robot_hw_drivers/{cfg,dependencies,include/robot_hw_drivers/{camera,driver},launch,scripts,src/{camera,driver},srv}
```

并补最小 `package.xml` / `CMakeLists.txt` / `launch/scepter_camera.launch`。

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 2：把外部磁盘中的 Scepter 相机包拉回工作区并重命名为 `robot_hw_drivers`

**文件：**
- 创建：`src/robot_hw_drivers/cfg/*`
- 创建：`src/robot_hw_drivers/srv/ConvertDepthToPointCloud.srv`
- 创建：`src/robot_hw_drivers/launch/scepter_camera.launch`
- 创建：`src/robot_hw_drivers/scripts/try.py`
- 创建：`src/robot_hw_drivers/setup.py`
- 创建：`src/robot_hw_drivers/dependencies/*`
- 创建：`src/robot_hw_drivers/include/robot_hw_drivers/camera/scepter_manager.hpp`
- 创建：`src/robot_hw_drivers/src/camera/scepter_driver.cpp`
- 创建：`src/robot_hw_drivers/src/camera/scepter_manager.cpp`
- 删除：`src/ScepterROS`

- [ ] **步骤 1：编写失败的测试**

```python
def test_robot_hw_drivers_camera_launch_uses_local_vendor_sdk_paths(self):
    launch_text = (REPO_ROOT / "src/robot_hw_drivers/launch/scepter_camera.launch").read_text(encoding="utf-8")
    self.assertIn('pkg="robot_hw_drivers"', launch_text)
    self.assertIn('$(find robot_hw_drivers)/dependencies/Lib', launch_text)
    self.assertIn('$(find robot_hw_drivers)/dependencies/Lib/Drivers', launch_text)
    self.assertNotIn('$(find ScepterROS)', launch_text)

def test_robot_hw_drivers_camera_sources_stop_referencing_scepterros_type_names(self):
    header_text = (REPO_ROOT / "src/robot_hw_drivers/include/robot_hw_drivers/camera/scepter_manager.hpp").read_text(encoding="utf-8")
    source_text = (REPO_ROOT / "src/robot_hw_drivers/src/camera/scepter_manager.cpp").read_text(encoding="utf-8")
    self.assertIn("robot_hw_drivers::ConvertDepthToPointCloud", source_text)
    self.assertIn('#include "robot_hw_drivers/ConvertDepthToPointCloud.h"', header_text)
    self.assertIn('#include "robot_hw_drivers/Sceptertof_roscppConfig.h"', header_text)
    self.assertNotIn("ScepterROS::ConvertDepthToPointCloud", source_text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_hw_drivers_camera_launch_uses_local_vendor_sdk_paths \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_hw_drivers_camera_sources_stop_referencing_scepterros_type_names
```

预期：`FAIL`

- [ ] **步骤 3：拷贝相机包内容并调整包名、include 和 launch**

迁移动作：

```bash
cp -a src/ScepterROS/cfg src/robot_hw_drivers/
cp -a src/ScepterROS/srv src/robot_hw_drivers/
cp -a src/ScepterROS/launch src/robot_hw_drivers/
cp -a src/ScepterROS/scripts src/robot_hw_drivers/
cp -a src/ScepterROS/dependencies src/robot_hw_drivers/
cp -a src/ScepterROS/setup.py src/robot_hw_drivers/
```

再用代码修改完成这些切换：

```cpp
// src/robot_hw_drivers/include/robot_hw_drivers/camera/scepter_manager.hpp
#include "robot_hw_drivers/Sceptertof_roscppConfig.h"
#include "robot_hw_drivers/ConvertDepthToPointCloud.h"
```

```cpp
// src/robot_hw_drivers/src/camera/scepter_manager.cpp
bool ScepterManager::convertDepthToPointCloud(
    robot_hw_drivers::ConvertDepthToPointCloud::Request &req,
    robot_hw_drivers::ConvertDepthToPointCloud::Response &res)
```

```xml
<!-- src/robot_hw_drivers/launch/scepter_camera.launch -->
<node pkg="robot_hw_drivers" type="scepter_camera" name="scepter_manager" ...>
  <env name="LD_LIBRARY_PATH"
       value="$(optenv LD_LIBRARY_PATH):$(find robot_hw_drivers)/dependencies/Lib:$(find robot_hw_drivers)/dependencies/Lib/Drivers" />
</node>
```

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 3：把索驱和线模驱动层迁入 `robot_hw_drivers`

**文件：**
- 创建：`src/robot_hw_drivers/include/robot_hw_drivers/driver/*`
- 创建：`src/robot_hw_drivers/src/driver/*`
- 删除：`src/chassis_ctrl/include/chassis_ctrl/driver/*`
- 删除：`src/chassis_ctrl/src/driver/*`
- 修改：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

```python
def test_robot_hw_drivers_package_owns_driver_layer_files(self):
    expected_files = [
        "src/robot_hw_drivers/include/robot_hw_drivers/driver/driver_types.hpp",
        "src/robot_hw_drivers/include/robot_hw_drivers/driver/cabin_tcp_transport.hpp",
        "src/robot_hw_drivers/include/robot_hw_drivers/driver/cabin_driver.hpp",
        "src/robot_hw_drivers/include/robot_hw_drivers/driver/modbus_transport.hpp",
        "src/robot_hw_drivers/include/robot_hw_drivers/driver/linear_module_driver.hpp",
    ]
    for file_path in expected_files:
        self.assertTrue((REPO_ROOT / file_path).exists(), file_path)

def test_chassis_ctrl_no_longer_owns_driver_layer_directories(self):
    self.assertFalse((REPO_ROOT / "src/chassis_ctrl/include/chassis_ctrl/driver").exists())
    self.assertFalse((REPO_ROOT / "src/chassis_ctrl/src/driver").exists())
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_hw_drivers_package_owns_driver_layer_files \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_chassis_ctrl_no_longer_owns_driver_layer_directories
```

预期：`FAIL`

- [ ] **步骤 3：拷贝驱动层文件并切命名空间**

迁移动作：

```bash
cp -a src/chassis_ctrl/include/chassis_ctrl/driver/. src/robot_hw_drivers/include/robot_hw_drivers/driver/
cp -a src/chassis_ctrl/src/driver/. src/robot_hw_drivers/src/driver/
```

批量替换：

```cpp
namespace robot_hw_drivers::driver {
```

```cpp
#include "robot_hw_drivers/driver/cabin_driver.hpp"
```

同时把所有头文件里的：

```cpp
#include "chassis_ctrl/driver/..."
```

改成：

```cpp
#include "robot_hw_drivers/driver/..."
```

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 4：让 `robot_hw_drivers` 真正构建相机可执行和底层驱动库

**文件：**
- 修改：`src/robot_hw_drivers/package.xml`
- 修改：`src/robot_hw_drivers/CMakeLists.txt`

- [ ] **步骤 1：编写失败的测试**

```python
def test_robot_hw_drivers_cmake_builds_scepter_camera_and_driver_core(self):
    cmake_text = (REPO_ROOT / "src/robot_hw_drivers/CMakeLists.txt").read_text(encoding="utf-8")
    self.assertIn("project(robot_hw_drivers)", cmake_text)
    self.assertIn("add_service_files(", cmake_text)
    self.assertIn("ConvertDepthToPointCloud.srv", cmake_text)
    self.assertIn("generate_dynamic_reconfigure_options(", cmake_text)
    self.assertIn("add_library(robot_hw_driver_core", cmake_text)
    self.assertIn("add_executable(scepter_camera", cmake_text)
    self.assertIn("target_link_libraries(scepter_camera", cmake_text)
    self.assertIn("libScepter_api", cmake_text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_hw_drivers_cmake_builds_scepter_camera_and_driver_core
```

预期：`FAIL`

- [ ] **步骤 3：补全 `package.xml` 与 `CMakeLists.txt`**

关键结构：

```cmake
project(robot_hw_drivers)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy sensor_msgs std_msgs geometry_msgs
  dynamic_reconfigure camera_info_manager image_transport cv_bridge
  tf2 tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions
  message_generation
)

add_service_files(FILES ConvertDepthToPointCloud.srv)
generate_messages(DEPENDENCIES std_msgs)
generate_dynamic_reconfigure_options(cfg/Sceptertof_roscpp.cfg)

add_library(robot_hw_driver_core
  src/driver/cabin_tcp_transport.cpp
  src/driver/cabin_protocol.cpp
  src/driver/cabin_driver.cpp
  src/driver/modbus_transport.cpp
  src/driver/linear_module_protocol.cpp
  src/driver/linear_module_driver.cpp
)

add_executable(scepter_camera
  src/camera/scepter_driver.cpp
  src/camera/scepter_manager.cpp
)
target_link_libraries(scepter_camera ${catkin_LIBRARIES} ${libScepter_api} ${OpenCV_LIBRARIES})
target_link_libraries(robot_hw_driver_core ${catkin_LIBRARIES} ${LIBMODBUS} modbus)
```

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 5：让 `robot_interface_hub` 切到新相机包

**文件：**
- 修改：`src/robot_interface_hub/package.xml`
- 修改：`src/robot_interface_hub/launch/api.launch`

- [ ] **步骤 1：编写失败的测试**

```python
def test_robot_interface_hub_depends_on_robot_hw_drivers_not_scepterros(self):
    package_text = (REPO_ROOT / "src/robot_interface_hub/package.xml").read_text(encoding="utf-8")
    self.assertIn("<exec_depend>robot_hw_drivers</exec_depend>", package_text)
    self.assertNotIn("<exec_depend>ScepterROS</exec_depend>", package_text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_robot_interface_hub_depends_on_robot_hw_drivers_not_scepterros
```

预期：`FAIL`

- [ ] **步骤 3：修改 `api.launch` 与 `package.xml`**

```xml
<!-- src/robot_interface_hub/launch/api.launch -->
<include file="$(find robot_hw_drivers)/launch/scepter_camera.launch" />
```

```xml
<!-- src/robot_interface_hub/package.xml -->
<exec_depend>robot_hw_drivers</exec_depend>
```

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 6：让 `chassis_ctrl` 链接新驱动包并切命名空间

**文件：**
- 修改：`src/chassis_ctrl/CMakeLists.txt`
- 修改：`src/chassis_ctrl/package.xml`
- 修改：`src/chassis_ctrl/src/suoquNode.cpp`
- 修改：`src/chassis_ctrl/src/moduanNode.cpp`
- 修改：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**

```python
def test_suoqu_and_moduan_use_robot_hw_drivers_namespace(self):
    suoqu_text = (REPO_ROOT / "src/chassis_ctrl/src/suoquNode.cpp").read_text(encoding="utf-8")
    moduan_text = (REPO_ROOT / "src/chassis_ctrl/src/moduanNode.cpp").read_text(encoding="utf-8")
    self.assertIn('#include "robot_hw_drivers/driver/cabin_driver.hpp"', suoqu_text)
    self.assertIn('#include "robot_hw_drivers/driver/linear_module_driver.hpp"', moduan_text)
    self.assertIn("std::make_unique<robot_hw_drivers::driver::CabinDriver>()", suoqu_text)
    self.assertIn("std::make_unique<robot_hw_drivers::driver::LinearModuleDriver>()", moduan_text)

def test_chassis_ctrl_cmake_links_robot_hw_driver_core(self):
    cmake_text = (REPO_ROOT / "src/chassis_ctrl/CMakeLists.txt").read_text(encoding="utf-8")
    self.assertIn("robot_hw_drivers", cmake_text)
    self.assertIn("robot_hw_driver_core", cmake_text)
    self.assertNotIn("src/driver/modbus_transport.cpp", cmake_text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_and_moduan_use_robot_hw_drivers_namespace \
  src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_chassis_ctrl_cmake_links_robot_hw_driver_core
```

预期：`FAIL`

- [ ] **步骤 3：切 include、命名空间和链接库**

核心修改：

```cpp
// src/chassis_ctrl/src/suoquNode.cpp
#include "robot_hw_drivers/driver/cabin_driver.hpp"
std::unique_ptr<robot_hw_drivers::driver::CabinDriver> g_cabin_driver;
```

```cpp
// src/chassis_ctrl/src/moduanNode.cpp
#include "robot_hw_drivers/driver/linear_module_driver.hpp"
std::unique_ptr<robot_hw_drivers::driver::LinearModuleDriver> g_linear_module_driver;
```

```cmake
find_package(catkin REQUIRED COMPONENTS
  robot_hw_drivers
  robot_interface_hub
  ...
)

target_link_libraries(moduanNode ${COMMON_LIBS} robot_hw_driver_core)
target_link_libraries(suoquNode ${COMMON_LIBS} robot_hw_driver_core -fopenmp)
```

- [ ] **步骤 4：运行测试验证通过**

运行同步骤 2。

预期：`OK`

---

## 任务 7：删除旧相机 symlink 与旧驱动目录，并完成编译验证

**文件：**
- 删除：`src/ScepterROS`
- 删除：`src/chassis_ctrl/include/chassis_ctrl/driver/`
- 删除：`src/chassis_ctrl/src/driver/`

- [ ] **步骤 1：运行删除前的定向构建，确认迁移链闭合**

运行：

```bash
source /opt/ros/noetic/setup.bash && \
catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_hw_drivers;robot_interface_hub;chassis_ctrl" -j2
```

预期：`exit 0`

- [ ] **步骤 2：删除旧目录**

运行：

```bash
rm -rf src/ScepterROS
rm -rf src/chassis_ctrl/include/chassis_ctrl/driver
rm -rf src/chassis_ctrl/src/driver
```

- [ ] **步骤 3：运行全套定向验证**

运行：

```bash
source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
python3 -m unittest \
  src.chassis_ctrl.test.test_pointai_order \
  src.chassis_ctrl.test.test_workspace_picker_web_launch \
  src.chassis_ctrl.test.test_workspace_picker_web_runtime \
  src.chassis_ctrl.test.test_system_log_mux \
  src.chassis_ctrl.test.test_gripper_tf_broadcaster
```

预期：与本次迁移相关的断言全部通过。

- [ ] **步骤 4：重新运行白名单编译**

运行：

```bash
source /opt/ros/noetic/setup.bash && \
catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_hw_drivers;robot_interface_hub;chassis_ctrl" -j2
```

预期：`exit 0`

- [ ] **步骤 5：Commit**

```bash
git add src/robot_hw_drivers src/robot_interface_hub src/chassis_ctrl docs/superpowers/plans/2026-04-23-robot-hw-drivers-refactor.md
git commit -m "refactor: 收口统一硬件驱动包"
```

---

## 自检

- 规格覆盖度：
  - 已覆盖相机源码回收、vendor SDK 一起迁移、launch 改向、索驱和线模驱动迁出、命名空间切换、旧目录删除、编译与定向测试。
- 占位符扫描：
  - 无 “TODO / 待定 / 后续实现 / 补充细节” 之类占位词。
- 类型一致性：
  - 统一使用 `robot_hw_drivers::driver::*` 命名空间。
  - 顶层编排包统一引用 `robot_hw_drivers/launch/scepter_camera.launch`。
  - 白名单构建统一使用 `robot_hw_drivers;robot_interface_hub;chassis_ctrl`。
