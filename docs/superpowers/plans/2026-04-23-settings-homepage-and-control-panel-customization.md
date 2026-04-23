# 设置主页与控制面板自定义实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让用户可以通过设置页下拉菜单中的小房子指定设置卡片默认主页，并通过控制面板标题栏的加号新增自定义 `std_srvs/Trigger` service 按钮；新增按钮与内建按钮混排显示，支持长按拖拽到垃圾桶删除。

**架构：** 前端本地存储继续保存设置主页，同时新增“自定义控制面板按钮”数组；`UIController` 负责主页小房子、自定义按钮表单、按钮网格、长按拖拽删除和垃圾桶 UI；`TieRobotFrontApp` 负责初始化加载、保存按钮、调用通用 Trigger service；`RosConnectionController` 提供动态 service client 封装。内建任务按钮保持原链路，不改业务实现。

**技术栈：** 原生 JavaScript、Vite、Python `unittest` 结构测试、本地存储 `localStorage`、`ROSLIB.Service`。

---

### 任务 1：先补失败测试

**文件：**
- 修改：`src/tie_robot_web/test/test_workspace_picker_web.py`

- [x] **步骤 1：为设置主页与自定义 service 按钮补结构测试**

覆盖点：
- 设置页小房子主页入口仍然存在
- 控制面板存在 `＋`、自定义按钮表单、服务地址输入框和垃圾桶
- `UIController` 暴露创建、点击、删除、长按拖拽相关接口
- `TieRobotFrontApp` 会加载和保存自定义按钮，并接上点击回调
- `storage.js` 有 `CUSTOM_CONTROL_PANEL_BUTTONS_KEY`
- `controlPanelCatalog.js` 有 `normalizeCustomControlPanelButtons()`
- `RosConnectionController.js` 有 `callTriggerService()`

- [x] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest -v src.tie_robot_web.test.test_workspace_picker_web.WorkspacePickerWebTest.test_settings_panel_supports_home_page_preference_and_control_panel_customization`

预期：新增断言失败，因为自定义按钮表单和动态 service 调用尚未实现完。

- [x] **步骤 3：确认失败原因只来自新能力缺失**

确认最初失败点来自控制面板旧的可见任务自定义断言残留，随后把测试口径更新为当前“自定义 service 按钮”方案。

### 任务 2：补存储、目录规范化与通用 service 调用

**文件：**
- 修改：`src/tie_robot_web/frontend/src/utils/storage.js`
- 修改：`src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`
- 修改：`src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`

- [x] **步骤 1：在 `storage.js` 中新增自定义按钮存储**

新增：
- `CUSTOM_CONTROL_PANEL_BUTTONS_KEY`
- `loadCustomControlPanelButtons()`
- `saveCustomControlPanelButtons(buttons)`

- [x] **步骤 2：在 `controlPanelCatalog.js` 中补自定义按钮规范化工具**

新增 `normalizeCustomControlPanelButtons(buttons)`：
- 校验 `id / label / servicePath`
- 自动补齐前导 `/`
- 过滤非法项和重复 `id`
- 默认附带 `serviceType: "std_srvs/Trigger"`

- [x] **步骤 3：在 `RosConnectionController.js` 中补通用 Trigger service 调用**

新增：
- 动态 `ROSLIB.Service` 缓存
- `callTriggerService(serviceName, serviceType = TRIGGER_SERVICE_TYPE)`

### 任务 3：实现控制面板自定义按钮 UI 与应用层接线

**文件：**
- 修改：`src/tie_robot_web/frontend/src/ui/UIController.js`
- 修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- 修改：`src/tie_robot_web/frontend/src/styles/app.css`

- [x] **步骤 1：在控制面板标题栏保留 `＋` 并渲染自定义按钮表单**

包括：
- `controlPanelCustomizeButton`
- `controlPanelCustomizeMenu`
- `controlPanelCustomButtonForm`
- `controlPanelCustomButtonName`
- `controlPanelCustomButtonService`

- [x] **步骤 2：让控制面板支持“内建任务 + 自定义按钮”混排渲染**

`renderControlPanelTasks(taskIds, customButtons)`：
- 内建按钮继续走 `data-task-action`
- 自定义按钮走 `data-custom-service-action`
- 自定义按钮显示主标题和 service 地址说明

- [x] **步骤 3：补创建、点击和 pending 态接线**

`TieRobotFrontApp`：
- 初始化加载 `loadCustomControlPanelButtons()`
- 新增按钮后持久化并重绘
- 点击自定义按钮时调用 `callTriggerService()`
- 执行中维护 pending 状态，避免重复触发

- [x] **步骤 4：补长按拖拽删除**

`UIController`：
- 长按约 `520ms` 进入编辑态
- 编辑态显示垃圾桶
- 拖入垃圾桶后触发删除回调
- `Esc` 或空白点击退出编辑态

- [x] **步骤 5：补样式**

包括：
- 自定义按钮卡片样式
- 自定义按钮表单样式
- 垃圾桶样式与 hover 态
- 拖拽代理层
- 编辑态抖动动画

### 任务 4：更新文档并验证

**文件：**
- 修改：`docs/superpowers/specs/2026-04-23-settings-homepage-and-control-panel-customization-design.md`
- 修改：`docs/superpowers/plans/2026-04-23-settings-homepage-and-control-panel-customization.md`

- [x] **步骤 1：把 superpower 文档从“勾选现有按钮”更新为“新增自定义 Trigger service 按钮”**

- [x] **步骤 2：运行完整验证**

运行：
- `python3 -m unittest /home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/test/test_workspace_picker_web.py`
- `npm --prefix /home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/frontend run build`

- [x] **步骤 3：确认静态产物同步**

前端构建通过后，`src/tie_robot_web/web` 已同步到最新产物。
