# 合并 `robot_algorithm_layer` 到运行时主包实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans` 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将独立的 `robot_algorithm_layer` 包并回现有新主包框架中，使规划算法归属 `tie_robot_process`，视觉 S2 算法归属 `tie_robot_perception`，并删除独立算法包。

**架构：** `C++` 的动态规划库迁移到 `tie_robot_process/include|src/planning/`，由 `suoquNode` 直接链接本包库；`Python` 的 `workspace_s2` 模块迁移到 `tie_robot_perception/src/tie_robot_perception/perception/`，由 `pointAI.py` 直接从本包导入。迁移完成后移除 `robot_algorithm_layer` 包及其所有构建依赖。

**技术栈：** ROS1 catkin、C++14、Python 3、unittest、catkin_make

---

### 任务 1：锁定并包后的结构验收

**文件：**
- 修改：`src/tie_robot_bringup/test/test_architecture_cleanup.py`
- 测试：`src/tie_robot_bringup/test/test_architecture_cleanup.py`

- [x] **步骤 1：编写失败的结构测试**

  已将结构测试改为断言：
  - `robot_algorithm_layer` 目录应不存在
  - `tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp` 应存在
  - `tie_robot_process/src/planning/dynamic_bind_planning.cpp` 应存在
  - `tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py` 应存在

- [x] **步骤 2：运行测试验证失败**

  运行：`source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.tie_robot_bringup.test.test_architecture_cleanup`

  预期：`FAIL`，因为当前 `robot_algorithm_layer` 仍存在，目标文件尚未迁入运行时包。

### 任务 2：迁移规划算法到 `tie_robot_process`

**文件：**
- 创建：`src/tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp`
- 创建：`src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- 修改：`src/tie_robot_process/src/suoquNode.cpp`
- 修改：`src/tie_robot_process/CMakeLists.txt`
- 修改：`src/tie_robot_process/package.xml`

- [ ] **步骤 1：复制规划头文件与实现文件**

  将 `robot_algorithm_layer` 中的规划文件迁移到：
  - `src/tie_robot_process/include/tie_robot_process/planning/dynamic_bind_planning.hpp`
  - `src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`

  并将命名空间从 `robot_algorithm_layer::planning` 调整为 `tie_robot_process::planning`。

- [ ] **步骤 2：更新 `suoquNode.cpp` 头文件和命名空间引用**

  将：

  ```cpp
  #include "robot_algorithm_layer/planning/dynamic_bind_planning.hpp"
  ```

  替换为：

  ```cpp
  #include "tie_robot_process/planning/dynamic_bind_planning.hpp"
  ```

  并将所有 `robot_algorithm_layer::planning::...` 调整为 `tie_robot_process::planning::...`。

- [ ] **步骤 3：更新 `tie_robot_process` 构建文件**

  在 `src/tie_robot_process/CMakeLists.txt` 中：
  - 去掉 `find_package(catkin REQUIRED COMPONENTS ... robot_algorithm_layer ...)`
  - 去掉 `catkin_package(CATKIN_DEPENDS ... robot_algorithm_layer ...)`
  - 新增本包库：

  ```cmake
  add_library(tie_robot_process_planning
    src/planning/dynamic_bind_planning.cpp
  )
  add_dependencies(tie_robot_process_planning ${catkin_EXPORTED_TARGETS})
  target_link_libraries(tie_robot_process_planning ${catkin_LIBRARIES})
  ```

  并让 `suoquNode` 链接 `tie_robot_process_planning`。

- [ ] **步骤 4：更新 `package.xml`**

  删除 `robot_algorithm_layer` 的 `build_depend` / `build_export_depend` / `exec_depend`。

- [ ] **步骤 5：运行结构测试验证过程包迁移生效**

  运行：`source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.tie_robot_bringup.test.test_architecture_cleanup`

  预期：仍可能失败，但失败点应只剩视觉算法文件和 `robot_algorithm_layer` 未删除。

### 任务 3：迁移视觉 S2 算法到 `tie_robot_perception`

**文件：**
- 创建：`src/tie_robot_perception/setup.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/__init__.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/perception/__init__.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`
- 修改：`src/tie_robot_perception/scripts/pointAI.py`
- 修改：`src/tie_robot_perception/CMakeLists.txt`
- 修改：`src/tie_robot_perception/package.xml`

- [ ] **步骤 1：复制 `workspace_s2.py` 并建立 Python 包结构**

  将 `robot_algorithm_layer/src/robot_algorithm_layer/perception/workspace_s2.py` 迁移到：
  `src/tie_robot_perception/src/tie_robot_perception/perception/workspace_s2.py`

  同时补齐两个 `__init__.py` 和 `setup.py`。

- [ ] **步骤 2：更新 `pointAI.py` 的本地导入路径**

  删除：

  ```python
  ALGORITHM_LAYER_PYTHON_DIR = ...
  from robot_algorithm_layer.perception.workspace_s2 import ...
  ```

  改为：

  ```python
  PERCEPTION_PACKAGE_DIR = os.path.abspath(
      os.path.join(os.path.dirname(__file__), "..", "src")
  )
  if os.path.isdir(PERCEPTION_PACKAGE_DIR) and PERCEPTION_PACKAGE_DIR not in sys.path:
      sys.path.insert(0, PERCEPTION_PACKAGE_DIR)

  from tie_robot_perception.perception.workspace_s2 import ...
  ```

- [ ] **步骤 3：启用 `catkin_python_setup()` 并删除算法包依赖**

  在 `src/tie_robot_perception/CMakeLists.txt` 中新增：

  ```cmake
  catkin_python_setup()
  ```

  并去掉 `robot_algorithm_layer` 依赖。

- [ ] **步骤 4：运行结构测试与脚本语法检查**

  运行：
  - `source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.tie_robot_bringup.test.test_architecture_cleanup`
  - `python3 -m py_compile src/tie_robot_perception/scripts/pointAI.py`

  预期：结构测试只剩 `robot_algorithm_layer` 目录存在这一类失败，语法检查通过。

### 任务 4：删除 `robot_algorithm_layer` 包并清理引用

**文件：**
- 删除：`src/robot_algorithm_layer/`
- 修改：`src/tie_robot_bringup/test/test_architecture_cleanup.py`

- [ ] **步骤 1：确认全局不再引用 `robot_algorithm_layer`**

  运行：

  ```bash
  rg -n "robot_algorithm_layer|robot_algorithm_planning" src/tie_robot_* build/CMakeCache.txt || true
  ```

  预期：只允许历史测试或缓存残留；源码层不应再有运行时引用。

- [ ] **步骤 2：删除独立算法包目录**

  运行：

  ```bash
  rm -rf src/robot_algorithm_layer
  ```

- [ ] **步骤 3：重新运行结构测试确认转绿**

  运行：`source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -m unittest src.tie_robot_bringup.test.test_architecture_cleanup`

  预期：`OK`

### 任务 5：重建并验证新工作空间

**文件：**
- 验证：`src/tie_robot_process/...`
- 验证：`src/tie_robot_perception/...`
- 验证：`src/tie_robot_web/...`

- [ ] **步骤 1：冷启动重建工作空间**

  运行：

  ```bash
  rm -rf build devel
  source /opt/ros/noetic/setup.bash
  catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_description;tie_robot_hw;tie_robot_perception;tie_robot_control;tie_robot_process;tie_robot_bringup;tie_robot_web" -j2
  ```

  预期：`exit 0`

- [ ] **步骤 2：运行迁移后的关键测试**

  运行：

  ```bash
  source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
  python3 -m unittest \
    src.tie_robot_bringup.test.test_architecture_cleanup \
    src.tie_robot_web.test.test_workspace_picker_web \
    src.tie_robot_web.test.test_system_log_mux \
    src.tie_robot_perception.test.test_gripper_tf_broadcaster
  ```

  预期：全部 `OK`

- [ ] **步骤 3：运行脚本静态验证**

  运行：

  ```bash
  node --check src/tie_robot_web/web/ir_workspace_picker.mjs
  python3 -m py_compile \
    src/tie_robot_perception/scripts/pointAI.py \
    src/tie_robot_perception/scripts/gripper_tf_broadcaster.py \
    src/tie_robot_perception/scripts/stable_point_tf_broadcaster.py
  ```

  预期：全部通过
