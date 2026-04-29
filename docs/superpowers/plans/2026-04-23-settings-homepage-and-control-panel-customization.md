# 设置主页与控制面板回退实现计划

**目标：** 保留设置页小房子主页能力，回退控制面板添加按钮和删除按钮功能，使控制面板重新只显示固定任务按钮。

**范围：** 删除控制面板 `＋`、自定义 service 按钮表单、长按拖拽删除、垃圾桶、动态 Trigger service 调用和相关本地存储。保留内建任务按钮、按钮点击反馈、设置主页小房子和控制面板任务提示框的移除状态。

## 任务 1：先补失败测试

- [x] 更新 `test_workspace_picker_web.py` 中设置主页与控制面板相关断言。
- [x] 断言控制面板不再包含 `controlPanelCustomizeButton`、自定义按钮表单、垃圾桶和拖拽接口。
- [x] 断言应用层不再加载、保存或调用自定义按钮。
- [x] 运行目标测试，确认当前实现会失败。

## 任务 2：删除前端实现

- [x] 从 `UIController.js` 删除控制面板 `＋`、自定义菜单、垃圾桶和拖拽删除逻辑。
- [x] 从 `TieRobotFrontApp.js` 删除自定义按钮状态、事件接线、持久化和 service 调用。
- [x] 从 `storage.js` 删除控制面板按钮自定义相关本地存储。
- [x] 从 `controlPanelCatalog.js` 删除自定义按钮规范化工具。
- [x] 从 `RosConnectionController.js` 删除通用自定义 Trigger service 调用。
- [x] 从 `app.css` 删除自定义按钮、垃圾桶、拖拽代理和编辑态样式。

## 任务 3：同步文档和验证

- [x] 更新 superpower 设计文档，明确控制面板添加/删除按钮功能已回退。
- [x] 更新 superpower 实现计划，记录本次回退范围。
- [x] 运行完整 `tie_robot_web` 回归测试。
- [x] 重新构建前端并同步 `src/tie_robot_web/web`。
