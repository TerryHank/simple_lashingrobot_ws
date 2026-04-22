# `robot_algorithm_layer` 实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 新建 `robot_algorithm_layer` 独立算法包，把视觉侧工作区 S2 纯算法和底盘侧动态分组/蛇形排序规划算法从应用节点中抽离出来。

**架构：** `robot_algorithm_layer` 作为混合语言算法包，提供 Python 感知算法模块和 C++ 规划算法库；`pointAI.py` 与 `suoquNode.cpp` 保留流程入口，但内部改为调用算法层；JSON 账本、TF 查询、ROS 回调与硬件连接继续留在应用层。

**技术栈：** ROS Noetic、catkin、Python、C++14、NumPy、OpenCV、tf2

---

## 文件结构

### 新建

- `src/robot_algorithm_layer/package.xml`
- `src/robot_algorithm_layer/CMakeLists.txt`
- `src/robot_algorithm_layer/setup.py`
- `src/robot_algorithm_layer/include/robot_algorithm_layer/planning/dynamic_bind_planning.hpp`
- `src/robot_algorithm_layer/src/planning/dynamic_bind_planning.cpp`
- `src/robot_algorithm_layer/src/robot_algorithm_layer/__init__.py`
- `src/robot_algorithm_layer/src/robot_algorithm_layer/perception/__init__.py`
- `src/robot_algorithm_layer/src/robot_algorithm_layer/perception/workspace_s2.py`

### 修改

- `src/chassis_ctrl/scripts/pointAI.py`
- `src/chassis_ctrl/src/suoquNode.cpp`
- `src/chassis_ctrl/CMakeLists.txt`
- `src/chassis_ctrl/package.xml`
- `src/chassis_ctrl/test/test_pointai_order.py`

---

## 任务 1：先用测试钉住算法层边界

**文件：**
- 修改：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**
- [ ] **步骤 2：运行测试验证失败**
- [ ] **步骤 3：创建最小 `robot_algorithm_layer` 包骨架**
- [ ] **步骤 4：运行测试验证通过**

## 任务 2：迁移 `pointAI` 的工作区 S2 纯算法

**文件：**
- 创建：`src/robot_algorithm_layer/src/robot_algorithm_layer/perception/workspace_s2.py`
- 修改：`src/chassis_ctrl/scripts/pointAI.py`
- 修改：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**
- [ ] **步骤 2：运行测试验证失败**
- [ ] **步骤 3：迁移顺时针排序、周期估计、透视整形等纯函数**
- [ ] **步骤 4：让 `pointAI.py` 改为导入并调用算法层**
- [ ] **步骤 5：运行测试验证通过**

## 任务 3：迁移 `suoquNode` 的动态规划算法

**文件：**
- 创建：`src/robot_algorithm_layer/include/robot_algorithm_layer/planning/dynamic_bind_planning.hpp`
- 创建：`src/robot_algorithm_layer/src/planning/dynamic_bind_planning.cpp`
- 修改：`src/chassis_ctrl/src/suoquNode.cpp`
- 修改：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的测试**
- [ ] **步骤 2：运行测试验证失败**
- [ ] **步骤 3：迁移动态分组、执行起点和蛇形排序算法到 C++ 库**
- [ ] **步骤 4：让 `suoquNode.cpp` 改为包含并调用算法层**
- [ ] **步骤 5：运行测试验证通过**

## 任务 4：切换构建依赖并完成验证

**文件：**
- 修改：`src/robot_algorithm_layer/CMakeLists.txt`
- 修改：`src/robot_algorithm_layer/package.xml`
- 修改：`src/chassis_ctrl/CMakeLists.txt`
- 修改：`src/chassis_ctrl/package.xml`

- [ ] **步骤 1：补齐 `robot_algorithm_layer` 的 Python/C++ 构建配置**
- [ ] **步骤 2：让 `chassis_ctrl` 依赖并链接 `robot_algorithm_layer`**
- [ ] **步骤 3：运行定向单测**
- [ ] **步骤 4：运行定向 `catkin_make`**

