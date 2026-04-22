# 新前端整页三维背景布局实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 把 `tie_robot_web` 新前端改成 `robot_viewer` 风格的整页 3D 背景布局，同时保留现有新前端和旧 `APP` 的业务功能入口。

**架构：** 继续复用现有 `Scene3DView / TopicLayerController / WorkspaceCanvasView / TaskActionController`，但重新组织页面：以整页三维背景为底，顶部工具条和玻璃态浮动面板为交互层。数据流与 ROS 接口不变，只改布局、UI 结构和少量控制接缝。

**技术栈：** Vite、原生 ES Modules、Three.js、ROSLIB、VitePress、unittest

---

## 文件结构

### 修改

- `src/tie_robot_web/frontend/src/ui/UIController.js`
  - 重新生成页面结构
  - 增加顶部工具条、面板显隐控制和整页背景容器
- `src/tie_robot_web/frontend/src/styles/app.css`
  - 重写为 `robot_viewer` 风格的全屏 3D 背景和玻璃态浮层样式
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
  - 场景容器改为背景层
  - 接入工具条的快捷显隐与视角切换
- `src/tie_robot_web/frontend/src/ui/PanelManager.js`
  - 适配新的默认面板布局与隐藏状态
- `src/tie_robot_web/frontend/src/controllers/TopicLayerController.js`
  - 适配顶部快捷切换
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
  - 确保全屏背景场景尺寸与交互稳定
- `src/tie_robot_web/help/guide/overview.md`
- `src/tie_robot_web/help/guide/dev-entrypoints.md`
- `src/tie_robot_web/help/reference/refactor-map.md`
  - 更新为新的布局口径
- `src/tie_robot_bringup/test/test_giant_business_structuring.py`
  - 补布局结构测试

### 测试与构建

- `src/tie_robot_web/frontend/package.json`
  - 如无额外依赖则不改
- `src/tie_robot_web/help/package.json`
  - 只用于帮助站构建验证

## 任务 1：先补失败的布局结构测试

**文件：**
- 修改：`src/tie_robot_bringup/test/test_giant_business_structuring.py`

- [ ] **步骤 1：增加布局断言**

在 `test_frontend_3d_scene_and_topic_layers_scaffold_exists` 附近新增断言，要求：

- `UIController.js` 包含顶部工具条相关标识
- `UIController.js` 包含全屏场景背景容器标识
- `app.css` 包含 `top-toolbar`、`scene-background` 等布局类

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring.GiantBusinessStructuringTest.test_frontend_3d_scene_and_topic_layers_scaffold_exists -v
```

预期：FAIL，因为当前布局还是旧结构。

## 任务 2：重构 UIController 的页面结构

**文件：**
- 修改：`src/tie_robot_web/frontend/src/ui/UIController.js`

- [ ] **步骤 1：将页面拆成背景层、工具条层、浮动面板层**

把 `renderShell()` 重写成：

- `scene-background`
- `scene-overlay`
- `top-toolbar`
- `floating-panels`

并把原来的 `sceneViewport` 改为背景容器，不再放在单独面板里。

- [ ] **步骤 2：增加工具条按钮和面板显隐控制**

在 `UIController.js` 中新增：

- 顶部面板切换按钮
- 场景快捷按钮
- 帮助站入口按钮

并提供对应方法，例如：

```js
onToolbarAction(callback) {
  this.refs.toolbarButtons.forEach((button) => {
    button.addEventListener("click", () => callback(button.dataset.toolbarAction));
  });
}
```

- [ ] **步骤 3：为各面板增加统一控制按钮**

每个浮动面板标题栏加入关闭按钮，允许通过工具条重新打开。

## 任务 3：把样式改成 robot_viewer 风格

**文件：**
- 修改：`src/tie_robot_web/frontend/src/styles/app.css`

- [ ] **步骤 1：改成全屏三维背景**

新增或改造：

```css
.scene-background {
  position: fixed;
  inset: 0;
  z-index: 0;
}

.scene-background canvas {
  width: 100%;
  height: 100%;
  display: block;
}
```

- [ ] **步骤 2：加入顶部工具条和玻璃态浮层样式**

新增：

- `.top-toolbar`
- `.toolbar-pill`
- `.floating-panel`
- `.scene-overlay`

视觉上向 `robot_viewer` 靠近：

- 深色背景
- 毛玻璃卡片
- 顶部圆角 pill 按钮
- 大面积留给 3D 背景

- [ ] **步骤 3：把 IR 画布面板收成浮动卡片**

保留 IR 画布和叠加层，但让其出现在左上浮动卡片里，而不是页面中心大舞台。

## 任务 4：把三维视图改成背景容器

**文件：**
- 修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- 修改：`src/tie_robot_web/frontend/src/views/Scene3DView.js`

- [ ] **步骤 1：让 Scene3DView 挂在整页背景节点**

在 `TieRobotFrontApp.js` 中继续创建 `Scene3DView`，但容器改为新的背景层节点。

- [ ] **步骤 2：接上工具条快捷控制**

新增对 `UIController.onToolbarAction(...)` 的绑定，支持：

- 视角切换
- 跟随相机开关
- 点云 / 绑扎点快捷显隐
- 面板开关

- [ ] **步骤 3：确认背景层尺寸稳定**

运行前端后，确保窗口 resize 时 `Scene3DView.resize()` 可以正确覆盖整页。

## 任务 5：适配面板拖拽和显隐

**文件：**
- 修改：`src/tie_robot_web/frontend/src/ui/PanelManager.js`
- 修改：`src/tie_robot_web/frontend/src/ui/UIController.js`

- [ ] **步骤 1：适配新的默认定位**

确保浮动面板在新布局下仍能：

- 记录初始位置
- 被拖动
- 被带到最前

- [ ] **步骤 2：加入显隐切换**

采用最小实现：

```js
setPanelVisible(panelId, visible) {
  const panel = this.rootElement.querySelector(`#${panelId}`);
  if (!panel) return;
  panel.style.display = visible ? "" : "none";
}
```

并让工具条按钮驱动它。

## 任务 6：帮助站更新

**文件：**
- 修改：`src/tie_robot_web/help/guide/overview.md`
- 修改：`src/tie_robot_web/help/guide/dev-entrypoints.md`
- 修改：`src/tie_robot_web/help/reference/refactor-map.md`

- [ ] **步骤 1：更新总览**

把“3D 只是一个独立面板”的描述改成“3D 是整页背景层”。

- [ ] **步骤 2：更新开发入口**

补充：

- 顶部工具条入口
- 面板显隐逻辑
- 背景层容器

- [ ] **步骤 3：更新重构映射**

说明新前端现在是：

- 背景三维层
- 工具条层
- 浮动业务面板层

## 任务 7：验证

**文件：**
- 测试：`src/tie_robot_bringup/test/test_giant_business_structuring.py`

- [ ] **步骤 1：跑结构测试**

运行：

```bash
python3 -m unittest src.tie_robot_bringup.test.test_giant_business_structuring -v
```

预期：PASS

- [ ] **步骤 2：跑前端语法检查**

运行：

```bash
node --check src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js
node --check src/tie_robot_web/frontend/src/ui/UIController.js
node --check src/tie_robot_web/frontend/src/ui/PanelManager.js
node --check src/tie_robot_web/frontend/src/controllers/TopicLayerController.js
node --check src/tie_robot_web/frontend/src/views/Scene3DView.js
```

预期：全部无输出

- [ ] **步骤 3：构建前端**

运行：

```bash
cd src/tie_robot_web/frontend && npm run build
```

预期：PASS，新的 `web/index.html` 与资源文件生成

- [ ] **步骤 4：构建帮助站**

运行：

```bash
cd src/tie_robot_web/help && npm run build
```

预期：PASS

- [ ] **步骤 5：跑 catkin 构建**

运行：

```bash
source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_description;tie_robot_hw;tie_robot_perception;tie_robot_control;tie_robot_process;tie_robot_bringup;tie_robot_web" -j2
```

预期：PASS

