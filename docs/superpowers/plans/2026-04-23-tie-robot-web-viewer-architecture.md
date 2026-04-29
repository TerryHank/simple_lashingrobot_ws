# `tie_robot_web` Viewer 化重构实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans` 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将当前 `tie_robot_web` 前端从“页面式拼装”重构为更接近 Foxglove / Webviz 的工具型 viewer 架构，同时保持现有功能和新前端视觉方向不变。

**架构：** 引入 `ViewerStore + SceneAdapter + PanelRegistry + LayoutManager` 四个骨架层，先把数据、布局、面板注册、3D 固定坐标系收口，再逐步把现有控制面板、工作区、日志、图层面板迁入。整个过程优先做等价重构，不先改后端接口。

**技术栈：** Vite、原生 ES Modules、Three.js、ROSLIB、ROS 1 rosbridge、本地 `unittest` + `node --check` + `vite build`

---

## 文件结构

### 新增

- `src/tie_robot_web/frontend/src/state/ViewerStore.js`
  统一保存连接状态、场景对象、布局状态、问题列表。
- `src/tie_robot_web/frontend/src/data/SceneAdapter.js`
  统一处理点云、识别点、规划点、TF 到 `map` 的归一化。
- `src/tie_robot_web/frontend/src/layout/defaultLayouts.js`
  默认布局模板。
- `src/tie_robot_web/frontend/src/layout/LayoutManager.js`
  布局保存、恢复、切换。
- `src/tie_robot_web/frontend/src/panels/panelRegistry.js`
  面板注册表。

### 修改

- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/ui/PanelManager.js`
- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/test/test_workspace_picker_web.py`

### 二期候选新增

- `src/tie_robot_web/frontend/src/views/TopicsPanelView.js`
- `src/tie_robot_web/frontend/src/views/TfInspectorView.js`
- `src/tie_robot_web/frontend/src/views/ProblemsPanelView.js`

---

### 任务 1：建立 Viewer 骨架层

**文件：**
- 创建：`src/tie_robot_web/frontend/src/state/ViewerStore.js`
- 创建：`src/tie_robot_web/frontend/src/data/SceneAdapter.js`
- 测试：`src/tie_robot_web/test/test_workspace_picker_web.py`

- [ ] **步骤 1：编写失败的结构测试**

在 `test_workspace_picker_web.py` 中新增断言，检查以下文件和关键符号存在：

```python
self.assertTrue((FRONTEND_SRC_DIR / "state" / "ViewerStore.js").exists())
self.assertTrue((FRONTEND_SRC_DIR / "data" / "SceneAdapter.js").exists())
self.assertIn("export class ViewerStore", viewer_store)
self.assertIn("export class SceneAdapter", scene_adapter)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_viewer_architecture_scaffold_exists -v
```

预期：失败，提示文件或类不存在。

- [ ] **步骤 3：编写最小骨架实现**

在 `ViewerStore.js` 中提供最小 API：

```js
export class ViewerStore {
  constructor(initialState = {}) {
    this.state = { ...initialState };
    this.listeners = new Set();
  }

  getState() {
    return this.state;
  }

  patch(nextState) {
    this.state = { ...this.state, ...nextState };
    this.listeners.forEach((listener) => listener(this.state));
  }

  subscribe(listener) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }
}
```

在 `SceneAdapter.js` 中先定义统一入口：

```js
export class SceneAdapter {
  normalizePointCloud(message, context) {
    return { positions: new Float32Array(), count: 0, context };
  }

  normalizeTiePoints(message) {
    return { positions: new Float32Array(), count: 0 };
  }

  normalizePlanningMarkers(message) {
    return { positions: new Float32Array(), count: 0 };
  }
}
```

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_viewer_architecture_scaffold_exists -v
```

预期：通过。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_web/frontend/src/state/ViewerStore.js \
        src/tie_robot_web/frontend/src/data/SceneAdapter.js \
        src/tie_robot_web/test/test_workspace_picker_web.py
git commit -m "feat: add viewer store and scene adapter scaffolds"
```

---

### 任务 2：引入 Panel Registry 和默认布局

**文件：**
- 创建：`src/tie_robot_web/frontend/src/panels/panelRegistry.js`
- 创建：`src/tie_robot_web/frontend/src/layout/defaultLayouts.js`
- 创建：`src/tie_robot_web/frontend/src/layout/LayoutManager.js`
- 修改：`src/tie_robot_web/frontend/src/utils/storage.js`
- 测试：`src/tie_robot_web/test/test_workspace_picker_web.py`

- [ ] **步骤 1：编写失败的结构测试**

新增断言：

```python
self.assertTrue((FRONTEND_SRC_DIR / "panels" / "panelRegistry.js").exists())
self.assertTrue((FRONTEND_SRC_DIR / "layout" / "defaultLayouts.js").exists())
self.assertTrue((FRONTEND_SRC_DIR / "layout" / "LayoutManager.js").exists())
self.assertIn("executionDebug", default_layouts)
self.assertIn("visionDebug", default_layouts)
self.assertIn("export const PANEL_REGISTRY", panel_registry)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_panel_registry_and_default_layouts_exist -v
```

预期：失败。

- [ ] **步骤 3：编写最小实现**

`panelRegistry.js` 先只注册现有面板：

```js
export const PANEL_REGISTRY = [
  { id: "controlPanel", title: "控制面板", defaultVisible: true, group: "operations" },
  { id: "workspacePanel", title: "工作区", defaultVisible: true, group: "vision" },
  { id: "topicLayersPanel", title: "话题图层", defaultVisible: true, group: "scene" },
  { id: "logPanel", title: "日志", defaultVisible: true, group: "diagnostics" },
];
```

`defaultLayouts.js` 提供 2 套起步模板：

```js
export const DEFAULT_LAYOUTS = {
  executionDebug: { ... },
  visionDebug: { ... },
};
```

`LayoutManager.js` 提供：

```js
export class LayoutManager {
  constructor({ storage, defaults }) { ... }
  getLayout(name) { ... }
  saveLayout(name, layout) { ... }
  listLayouts() { ... }
}
```

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_panel_registry_and_default_layouts_exist -v
```

预期：通过。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_web/frontend/src/panels/panelRegistry.js \
        src/tie_robot_web/frontend/src/layout/defaultLayouts.js \
        src/tie_robot_web/frontend/src/layout/LayoutManager.js \
        src/tie_robot_web/frontend/src/utils/storage.js \
        src/tie_robot_web/test/test_workspace_picker_web.py
git commit -m "feat: add panel registry and default viewer layouts"
```

---

### 任务 3：让应用从 Registry 和 Layout 生成面板

**文件：**
- 修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- 修改：`src/tie_robot_web/frontend/src/ui/UIController.js`
- 修改：`src/tie_robot_web/frontend/src/ui/PanelManager.js`
- 测试：`src/tie_robot_web/test/test_workspace_picker_web.py`

- [ ] **步骤 1：编写失败的结构测试**

新增断言：

```python
self.assertIn("PANEL_REGISTRY", app_logic)
self.assertIn("DEFAULT_LAYOUTS", app_logic)
self.assertIn("LayoutManager", app_logic)
self.assertIn("renderPanelsFromLayout", ui_controller)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_app_uses_panel_registry_and_layout_manager -v
```

预期：失败。

- [ ] **步骤 3：编写最小实现**

`TieRobotFrontApp` 初始化：

```js
this.layoutManager = new LayoutManager({ storage, defaults: DEFAULT_LAYOUTS });
this.panelRegistry = PANEL_REGISTRY;
this.activeLayout = this.layoutManager.getLayout("executionDebug");
```

`UIController` 增加：

```js
renderPanelsFromLayout(layout, panelRegistry) { ... }
```

`PanelManager` 保持拖动缩放逻辑不变，只接收新生成的 panel DOM。

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_app_uses_panel_registry_and_layout_manager -v
```

预期：通过。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js \
        src/tie_robot_web/frontend/src/ui/UIController.js \
        src/tie_robot_web/frontend/src/ui/PanelManager.js \
        src/tie_robot_web/test/test_workspace_picker_web.py
git commit -m "refactor: drive panels from registry and layouts"
```

---

### 任务 4：把 3D 场景的数据统一接到 SceneAdapter

**文件：**
- 修改：`src/tie_robot_web/frontend/src/views/Scene3DView.js`
- 修改：`src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- 修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- 测试：`src/tie_robot_web/test/test_workspace_picker_web.py`

- [ ] **步骤 1：编写失败的结构测试**

新增断言：

```python
self.assertIn("SceneAdapter", scene_view)
self.assertIn("normalizePointCloud", app_logic)
self.assertNotIn("point.applyQuaternion(scepterTransform.quaternion).add(scepterTransform.position);", scene_view)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_scene_uses_scene_adapter_pipeline -v
```

预期：失败。

- [ ] **步骤 3：编写实现**

要求：

- `Scene3DView` 不再直接理解 `/Scepter/worldCoord/*`
- 点云、识别点、规划点全部通过 `SceneAdapter` 统一后再渲染
- `SceneAdapter` 负责把点云统一为 `map`

起步 API：

```js
const normalized = this.sceneAdapter.normalizePointCloud(message, {
  source,
  transforms: this.scene3d.getTransformsSnapshot(),
});
this.scene3d.setNormalizedPointCloud(source, normalized);
```

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_scene_uses_scene_adapter_pipeline -v
node --check src/tie_robot_web/frontend/src/views/Scene3DView.js
```

预期：通过。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_web/frontend/src/views/Scene3DView.js \
        src/tie_robot_web/frontend/src/controllers/RosConnectionController.js \
        src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js \
        src/tie_robot_web/test/test_workspace_picker_web.py
git commit -m "refactor: route scene data through scene adapter"
```

---

### 任务 5：新增 Topics / TF / Problems 三类排查面板

**文件：**
- 创建：`src/tie_robot_web/frontend/src/views/TopicsPanelView.js`
- 创建：`src/tie_robot_web/frontend/src/views/TfInspectorView.js`
- 创建：`src/tie_robot_web/frontend/src/views/ProblemsPanelView.js`
- 修改：`src/tie_robot_web/frontend/src/panels/panelRegistry.js`
- 修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- 测试：`src/tie_robot_web/test/test_workspace_picker_web.py`

- [ ] **步骤 1：编写失败的结构测试**

新增断言：

```python
self.assertTrue((FRONTEND_SRC_DIR / "views" / "TopicsPanelView.js").exists())
self.assertTrue((FRONTEND_SRC_DIR / "views" / "TfInspectorView.js").exists())
self.assertTrue((FRONTEND_SRC_DIR / "views" / "ProblemsPanelView.js").exists())
self.assertIn('id: "topicsPanel"', panel_registry)
self.assertIn('id: "tfPanel"', panel_registry)
self.assertIn('id: "problemsPanel"', panel_registry)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_diagnostic_panels_exist_in_registry -v
```

预期：失败。

- [ ] **步骤 3：编写最小实现**

面板先做最小只读版本：

- `TopicsPanelView`：显示关键 topic 在线状态
- `TfInspectorView`：显示 `map / Scepter_depth_frame / gripper_frame`
- `ProblemsPanelView`：显示当前问题列表

不要求一期就做复杂交互。

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_diagnostic_panels_exist_in_registry -v
node --check src/tie_robot_web/frontend/src/views/TopicsPanelView.js
node --check src/tie_robot_web/frontend/src/views/TfInspectorView.js
node --check src/tie_robot_web/frontend/src/views/ProblemsPanelView.js
```

预期：通过。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_web/frontend/src/views/TopicsPanelView.js \
        src/tie_robot_web/frontend/src/views/TfInspectorView.js \
        src/tie_robot_web/frontend/src/views/ProblemsPanelView.js \
        src/tie_robot_web/frontend/src/panels/panelRegistry.js \
        src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js \
        src/tie_robot_web/test/test_workspace_picker_web.py
git commit -m "feat: add diagnostic viewer panels"
```

---

### 任务 6：补文档和最终验证

**文件：**
- 修改：`README.md`
- 修改：`src/tie_robot_web/help/guide/overview.md`
- 修改：`src/tie_robot_web/help/guide/dev-entrypoints.md`
- 修改：`src/tie_robot_web/help/reference/refactor-map.md`

- [ ] **步骤 1：更新帮助站**

说明以下内容：

- viewer 架构分层
- panel registry
- layout manager
- scene adapter
- 新增的排查型 panel

- [ ] **步骤 2：运行最终验证**

运行：

```bash
python3 -m unittest src.tie_robot_web.test.test_workspace_picker_web -v
node --check src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js
node --check src/tie_robot_web/frontend/src/ui/UIController.js
node --check src/tie_robot_web/frontend/src/views/Scene3DView.js
cd src/tie_robot_web/frontend && npm run build
cd src/tie_robot_web/help && npm run build
source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_description;tie_robot_hw;tie_robot_perception;tie_robot_control;tie_robot_process;tie_robot_bringup;tie_robot_web" -j2
```

预期：

- `unittest` 全部通过
- `node --check` 全部通过
- 前端构建成功
- 帮助站构建成功
- `catkin_make` 通过

- [ ] **步骤 3：Commit**

```bash
git add README.md \
        src/tie_robot_web/help/guide/overview.md \
        src/tie_robot_web/help/guide/dev-entrypoints.md \
        src/tie_robot_web/help/reference/refactor-map.md
git commit -m "docs: document viewer architecture"
```

---

计划已完成并保存到 `docs/superpowers/plans/2026-04-23-tie-robot-web-viewer-architecture.md`。

两种执行方式：

1. 子代理驱动（推荐）  
2. 内联执行

当前会话默认继续走内联执行，并先从任务 1 的最小骨架开始。
