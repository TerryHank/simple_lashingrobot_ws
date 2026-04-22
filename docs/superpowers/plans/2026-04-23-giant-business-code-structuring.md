# 巨型业务代码结构化与帮助站建设实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将当前工作空间内所有主链巨型业务文件拆成职责清晰的模块，同时在 `tie_robot_web` 中补齐可静态访问的 VitePress 帮助站。

**架构：** 保持现有包边界不变，只在包内做纯等价结构化重构：入口文件变薄，常量/执行器/渲染/桥接/账本/流程模块下沉。帮助站源码放入 `src/tie_robot_web/help`，构建产物输出到 `src/tie_robot_web/web/help` 并由现有静态服务直接托管。

**技术栈：** ROS1 catkin、C++11、Python3、ES Modules、VitePress、unittest、Node.js

---

### 任务 1：建立结构化回归测试与帮助站骨架

**文件：**
- 创建：`src/tie_robot_bringup/test/test_giant_business_structuring.py`
- 创建：`src/tie_robot_web/help/package.json`
- 创建：`src/tie_robot_web/help/.vitepress/config.mjs`
- 创建：`src/tie_robot_web/help/index.md`
- 创建：`src/tie_robot_web/help/guide/overview.md`
- 创建：`src/tie_robot_web/help/scripts/generate_structure_docs.py`
- 修改：`src/tie_robot_web/web/index.html`

- [ ] **步骤 1：编写失败的结构测试**

```python
import unittest
from pathlib import Path


class GiantBusinessStructuringTest(unittest.TestCase):
    def test_structured_modules_exist(self):
        expected = [
            Path("src/tie_robot_control/include/tie_robot_control/moduan/register_map.hpp"),
            Path("src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py"),
            Path("src/tie_robot_process/include/tie_robot_process/suoqu/execution_mode.hpp"),
            Path("src/tie_robot_web/web/modules/ros_connection.mjs"),
            Path("src/tie_robot_web/help/.vitepress/config.mjs"),
        ]
        for path in expected:
            self.assertTrue(path.exists(), str(path))
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：FAIL，提示结构化模块或帮助站文件不存在。

- [ ] **步骤 3：编写最少帮助站骨架与测试文件**

```json
{
  "name": "tie-robot-help",
  "private": true,
  "scripts": {
    "build": "vitepress build . --outDir ../web/help"
  }
}
```

```js
export default {
  title: "Tie Robot Help",
  description: "工程结构与主链说明"
};
```

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：PASS。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_bringup/test/test_giant_business_structuring.py src/tie_robot_web/help src/tie_robot_web/web/index.html
git commit -m "test: add giant business structuring guardrails"
```

### 任务 2：拆分 `tie_robot_control` 的 `moduanNode.cpp`

**文件：**
- 创建：`src/tie_robot_control/include/tie_robot_control/moduan/register_map.hpp`
- 创建：`src/tie_robot_control/include/tie_robot_control/moduan/runtime_state.hpp`
- 创建：`src/tie_robot_control/include/tie_robot_control/moduan/numeric_codec.hpp`
- 创建：`src/tie_robot_control/include/tie_robot_control/moduan/error_handling.hpp`
- 创建：`src/tie_robot_control/include/tie_robot_control/moduan/linear_module_executor.hpp`
- 创建：`src/tie_robot_control/include/tie_robot_control/moduan/moduan_ros_callbacks.hpp`
- 创建：`src/tie_robot_control/src/moduan/numeric_codec.cpp`
- 创建：`src/tie_robot_control/src/moduan/error_handling.cpp`
- 创建：`src/tie_robot_control/src/moduan/linear_module_executor.cpp`
- 创建：`src/tie_robot_control/src/moduan/moduan_ros_callbacks.cpp`
- 创建：`src/tie_robot_control/src/moduan/moduan_node_main.cpp`
- 修改：`src/tie_robot_control/CMakeLists.txt`
- 修改：`src/tie_robot_control/src/moduanNode.cpp`
- 测试：`src/tie_robot_bringup/test/test_giant_business_structuring.py`

- [ ] **步骤 1：扩展失败的结构测试**

```python
def test_moduan_entry_is_thin(self):
    line_count = sum(1 for _ in Path("src/tie_robot_control/src/moduanNode.cpp").open())
    self.assertLess(line_count, 900)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：FAIL，`moduanNode.cpp` 仍大于阈值。

- [ ] **步骤 3：提取常量、状态、执行器与回调**

```cpp
// register_map.hpp
namespace tie_robot_control::moduan {
constexpr int kFinishAllFlagRegister = 5210;
}
```

```cpp
// moduan_node_main.cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "moduanNode");
  return tie_robot_control::moduan::RunModuanNode(argc, argv);
}
```

- [ ] **步骤 4：运行测试与定向构建**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：PASS

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_hw;tie_robot_control" -j2`
预期：构建通过。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_control/include/tie_robot_control/moduan src/tie_robot_control/src/moduan src/tie_robot_control/src/moduanNode.cpp src/tie_robot_control/CMakeLists.txt src/tie_robot_bringup/test/test_giant_business_structuring.py
git commit -m "refactor: split tie_robot_control moduan node"
```

### 任务 3：拆分 `tie_robot_perception` 的 `pointAI.py` 与 `scepter_manager.cpp`

**文件：**
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/image_buffers.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/tf_transform.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/world_coord.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/workspace_masks.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/matrix_selection.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/rendering.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`
- 创建：`src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- 创建：`src/tie_robot_perception/include/tie_robot_perception/camera/device_session.hpp`
- 创建：`src/tie_robot_perception/include/tie_robot_perception/camera/intrinsics.hpp`
- 创建：`src/tie_robot_perception/include/tie_robot_perception/camera/frame_publish.hpp`
- 创建：`src/tie_robot_perception/src/camera/device_session.cpp`
- 创建：`src/tie_robot_perception/src/camera/intrinsics.cpp`
- 创建：`src/tie_robot_perception/src/camera/frame_publish.cpp`
- 修改：`src/tie_robot_perception/scripts/pointAI.py`
- 修改：`src/tie_robot_perception/src/camera/scepter_manager.cpp`
- 修改：`src/tie_robot_perception/include/tie_robot_perception/camera/scepter_manager.hpp`
- 修改：`src/tie_robot_perception/CMakeLists.txt`
- 测试：`src/tie_robot_bringup/test/test_giant_business_structuring.py`
- 测试：`src/tie_robot_perception/test/test_scepter_sdk_split.py`

- [ ] **步骤 1：扩展失败的结构测试**

```python
def test_pointai_entry_is_thin(self):
    line_count = sum(1 for _ in Path("src/tie_robot_perception/scripts/pointAI.py").open())
    self.assertLess(line_count, 800)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：FAIL，`pointAI.py` 仍大于阈值。

- [ ] **步骤 3：提取 Python 模块与相机管理子模块**

```python
# processor.py
class PointAIProcessor:
    def __init__(self, runtime_config, image_buffers, tf_transform):
        self.runtime_config = runtime_config
```

```cpp
// device_session.hpp
class ScepterDeviceSession {
 public:
  bool Connect();
  void Disconnect();
};
```

- [ ] **步骤 4：运行语法检查、单测与定向构建**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring src.tie_robot_perception.test.test_scepter_sdk_split -v`
预期：PASS

运行：`python3 -m py_compile src/tie_robot_perception/scripts/pointAI.py src/tie_robot_perception/src/tie_robot_perception/pointai/*.py`
预期：通过

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_perception" -j2`
预期：构建通过。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_perception src/tie_robot_bringup/test/test_giant_business_structuring.py
git commit -m "refactor: split tie_robot_perception giant modules"
```

### 任务 4：拆分 `tie_robot_process` 的 `suoquNode.cpp` 与 `dynamic_bind_planning.cpp`

**文件：**
- 创建：`src/tie_robot_process/include/tie_robot_process/suoqu/execution_mode.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/suoqu/scan_session_store.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/suoqu/bind_path_store.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/suoqu/current_area_bind_runner.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/suoqu/global_bind_runner.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/suoqu/live_visual_runner.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/suoqu/direct_bind_test_runner.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/suoqu/start_work_services.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/suoqu/cabin_motion_helpers.hpp`
- 创建：`src/tie_robot_process/src/suoqu/scan_session_store.cpp`
- 创建：`src/tie_robot_process/src/suoqu/bind_path_store.cpp`
- 创建：`src/tie_robot_process/src/suoqu/current_area_bind_runner.cpp`
- 创建：`src/tie_robot_process/src/suoqu/global_bind_runner.cpp`
- 创建：`src/tie_robot_process/src/suoqu/live_visual_runner.cpp`
- 创建：`src/tie_robot_process/src/suoqu/direct_bind_test_runner.cpp`
- 创建：`src/tie_robot_process/src/suoqu/start_work_services.cpp`
- 创建：`src/tie_robot_process/src/suoqu/cabin_motion_helpers.cpp`
- 创建：`src/tie_robot_process/src/suoqu/node_main.cpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/planning/coverability.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/planning/grouping.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/planning/ordering.hpp`
- 创建：`src/tie_robot_process/include/tie_robot_process/planning/path_origin.hpp`
- 创建：`src/tie_robot_process/src/planning/coverability.cpp`
- 创建：`src/tie_robot_process/src/planning/grouping.cpp`
- 创建：`src/tie_robot_process/src/planning/ordering.cpp`
- 创建：`src/tie_robot_process/src/planning/path_origin.cpp`
- 修改：`src/tie_robot_process/src/suoquNode.cpp`
- 修改：`src/tie_robot_process/src/planning/dynamic_bind_planning.cpp`
- 修改：`src/tie_robot_process/CMakeLists.txt`
- 测试：`src/tie_robot_bringup/test/test_giant_business_structuring.py`

- [ ] **步骤 1：扩展失败的结构测试**

```python
def test_suoqu_entry_is_thin(self):
    line_count = sum(1 for _ in Path("src/tie_robot_process/src/suoquNode.cpp").open())
    self.assertLess(line_count, 1200)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：FAIL，`suoquNode.cpp` 仍大于阈值。

- [ ] **步骤 3：提取账本、执行支线、运动辅助和规划子模块**

```cpp
// execution_mode.hpp
enum class ExecutionMode {
  kSlamPrecomputed = 0,
  kLiveVisual = 1,
};
```

```cpp
// path_origin.hpp
geometry_msgs::Point ComputePathOrigin(const std::vector<BindAreaEntry>& areas);
```

- [ ] **步骤 4：运行定向测试与构建**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：PASS

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_hw;tie_robot_process" -j2`
预期：构建通过。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_process src/tie_robot_bringup/test/test_giant_business_structuring.py
git commit -m "refactor: split tie_robot_process giant workflow files"
```

### 任务 5：拆分 `tie_robot_web` 的桥接层和前端脚本

**文件：**
- 创建：`src/tie_robot_web/include/tie_robot_web/web_bridge/service_clients.hpp`
- 创建：`src/tie_robot_web/include/tie_robot_web/web_bridge/action_bridge.hpp`
- 创建：`src/tie_robot_web/include/tie_robot_web/web_bridge/topic_callbacks.hpp`
- 创建：`src/tie_robot_web/include/tie_robot_web/web_bridge/system_control.hpp`
- 创建：`src/tie_robot_web/src/web_bridge/service_clients.cpp`
- 创建：`src/tie_robot_web/src/web_bridge/action_bridge.cpp`
- 创建：`src/tie_robot_web/src/web_bridge/topic_callbacks.cpp`
- 创建：`src/tie_robot_web/src/web_bridge/system_control.cpp`
- 创建：`src/tie_robot_web/src/web_bridge/main.cpp`
- 创建：`src/tie_robot_web/web/modules/ros_connection.mjs`
- 创建：`src/tie_robot_web/web/modules/action_clients.mjs`
- 创建：`src/tie_robot_web/web/modules/canvas_renderer.mjs`
- 创建：`src/tie_robot_web/web/modules/overlay_renderer.mjs`
- 创建：`src/tie_robot_web/web/modules/workspace_selection.mjs`
- 创建：`src/tie_robot_web/web/modules/execution_actions.mjs`
- 创建：`src/tie_robot_web/web/modules/ui_state.mjs`
- 创建：`src/tie_robot_web/web/modules/dom_refs.mjs`
- 修改：`src/tie_robot_web/src/topics_transfer.cpp`
- 修改：`src/tie_robot_web/web/ir_workspace_picker.mjs`
- 修改：`src/tie_robot_web/CMakeLists.txt`
- 测试：`src/tie_robot_bringup/test/test_giant_business_structuring.py`
- 测试：`src/tie_robot_web/test/test_workspace_picker_web.py`

- [ ] **步骤 1：扩展失败的结构测试**

```python
def test_web_entries_are_thin(self):
    topics = sum(1 for _ in Path("src/tie_robot_web/src/topics_transfer.cpp").open())
    frontend = sum(1 for _ in Path("src/tie_robot_web/web/ir_workspace_picker.mjs").open())
    self.assertLess(topics, 350)
    self.assertLess(frontend, 300)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：FAIL，入口文件仍超阈值。

- [ ] **步骤 3：提取桥接器和前端模块**

```js
// ros_connection.mjs
export function createRosConnection(ROSLIB, url) {
  return new ROSLIB.Ros({ url });
}
```

```cpp
// service_clients.hpp
namespace tie_robot_web::bridge {
bool CallStartGlobalWork(...);
}
```

- [ ] **步骤 4：运行结构测试、前端测试和语法检查**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring src.tie_robot_web.test.test_workspace_picker_web -v`
预期：PASS

运行：`node --check src/tie_robot_web/web/ir_workspace_picker.mjs`
预期：通过

运行：`node --test src/tie_robot_web/web/ir_workspace_picker_helpers.test.mjs`
预期：通过。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_web src/tie_robot_bringup/test/test_giant_business_structuring.py
git commit -m "refactor: split tie_robot_web giant bridge and frontend files"
```

### 任务 6：生成帮助站内容、自动结构页并接入帮助入口

**文件：**
- 创建：`src/tie_robot_web/help/guide/packages.md`
- 创建：`src/tie_robot_web/help/guide/action-chain.md`
- 创建：`src/tie_robot_web/help/guide/data-chain.md`
- 创建：`src/tie_robot_web/help/guide/dev-entrypoints.md`
- 创建：`src/tie_robot_web/help/reference/file-tree.md`
- 创建：`src/tie_robot_web/help/reference/refactor-map.md`
- 修改：`src/tie_robot_web/help/.vitepress/config.mjs`
- 修改：`src/tie_robot_web/help/scripts/generate_structure_docs.py`
- 修改：`src/tie_robot_web/web/index.html`
- 测试：`src/tie_robot_bringup/test/test_giant_business_structuring.py`

- [ ] **步骤 1：扩展失败的帮助站测试**

```python
def test_help_output_exists(self):
    self.assertTrue(Path("src/tie_robot_web/web/help/index.html").exists())
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：FAIL，帮助站尚未构建。

- [ ] **步骤 3：编写帮助站页面和结构生成脚本**

```python
# generate_structure_docs.py
from pathlib import Path
ROOT = Path(__file__).resolve().parents[4] / "src"
```

```md
# 工程总览

- `tie_robot_hw`：硬件与 SDK
- `tie_robot_control`：线模控制
- `tie_robot_process`：执行流程
```

- [ ] **步骤 4：运行结构生成与帮助站构建**

运行：`python3 src/tie_robot_web/help/scripts/generate_structure_docs.py`
预期：更新 `help/reference/*.md`

运行：`cd src/tie_robot_web/help && npm install && npm run build`
预期：生成 `src/tie_robot_web/web/help/index.html`

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v`
预期：PASS。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_web/help src/tie_robot_web/web/index.html src/tie_robot_web/web/help src/tie_robot_bringup/test/test_giant_business_structuring.py
git commit -m "docs: add tie robot help site"
```

### 任务 7：全量验证与收尾

**文件：**
- 修改：`docs/superpowers/plans/2026-04-23-giant-business-code-structuring.md`
- 测试：`src/tie_robot_bringup/test/test_giant_business_structuring.py`
- 测试：`src/tie_robot_perception/test/test_scepter_sdk_split.py`
- 测试：`src/tie_robot_web/test/test_workspace_picker_web.py`

- [ ] **步骤 1：运行结构与定向测试**

运行：`python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring src.tie_robot_perception.test.test_scepter_sdk_split src.tie_robot_web.test.test_workspace_picker_web -v`
预期：全部通过。

- [ ] **步骤 2：运行语法与构建检查**

运行：`python3 -m py_compile src/tie_robot_perception/scripts/pointAI.py src/tie_robot_perception/src/tie_robot_perception/pointai/*.py src/tie_robot_web/scripts/*.py`
预期：通过

运行：`node --check src/tie_robot_web/web/ir_workspace_picker.mjs`
预期：通过

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_description;tie_robot_hw;tie_robot_perception;tie_robot_control;tie_robot_process;tie_robot_bringup;tie_robot_web" -j2`
预期：构建通过。

- [ ] **步骤 3：检查帮助站产物与入口**

运行：`test -f src/tie_robot_web/web/help/index.html && echo ok`
预期：输出 `ok`

运行：`rg -n "help" src/tie_robot_web/web/index.html`
预期：能看到帮助入口链接。

- [ ] **步骤 4：记录偏差并更新计划状态**

```md
- 已完成：结构化拆分、帮助站上线、主链构建验证
- 未做：第二阶段接口与行为优化
```

- [ ] **步骤 5：Commit**

```bash
git add docs/superpowers/plans/2026-04-23-giant-business-code-structuring.md
git commit -m "chore: record giant business structuring verification"
```
