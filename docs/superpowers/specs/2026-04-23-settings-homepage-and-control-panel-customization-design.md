# 设置主页与控制面板自定义设计

## 目标

- 允许用户在 `设置` 卡片中指定一个“主页”，即设置卡片下次打开或页面刷新后默认进入的设置页。
- 在 `控制面板` 卡片标题栏新增一个加号按钮。
- 用户点击加号后，可以自己输入按钮名称和 ROS service 地址，把一个新的自定义按钮加入控制面板。
- 自定义按钮与内建任务按钮混排显示，点击时按 `std_srvs/Trigger` 空请求 service 调用。
- 自定义按钮支持长按进入编辑态，并像手机桌面一样拖到垃圾桶删除。

## 范围收口

- “主页”只指 `设置` 卡片的默认页面，不改变整个前端的默认布局、默认面板可见性或场景首页。
- 控制面板的内建任务按钮保持现状，不改原来的业务链路、禁用态、点击回调或按钮文案。
- 本轮“自定义按钮”只支持 `ROS Service`，并且只发送空请求，适用于 `std_srvs/Trigger` 这类服务。
- 本轮不支持用户自定义 topic、action、JSON 请求体，也不支持拖拽排序。
- 只有自定义按钮支持长按拖拽删除；内建按钮不能删除。

## 交互方案

### 1. 设置主页

- 保持 `设置` 卡片顶部原有的单个“当前页”下拉，不新增第二个“主页”下拉。
- 在“当前页”下拉的自定义菜单里，为每个页面项增加一个小房子按钮：
  - `话题总览`
  - `工作区选点`
  - `显示与视角`
  - `图层与数据`
  - `索驱遥控`
  - `相机-TCP外参`
- 用户点击小房子后立即写入本地存储。
- 前端初始化时优先读取这个主页配置；若无配置或配置无效，则回退到当前默认值 `topics`。
- 设置页切换本身仍然独立工作；“当前页”不自动覆盖“主页”，只有用户显式点击小房子时才更新。
- 小房子有激活态，用户打开下拉即可看出当前哪一页被设为主页。

### 2. 控制面板加号

- 在 `控制面板` 标题栏新增 `＋` 按钮。
- 点击后打开一个轻量表单，字段只有：
  - `按钮名称`
  - `服务地址`
- 表单下方显示当前已创建的自定义按钮摘要列表，便于确认已经添加过哪些项。
- 添加成功后，自定义按钮立即出现在控制面板里，并写入本地存储。
- 如果控制面板没有任何固定按钮和自定义按钮，主体显示空状态提示，例如“还没有固定按钮，点击右上角加号添加”。

### 3. 自定义按钮行为

- 自定义按钮与内建按钮混排在同一个控制面板网格中。
- 按钮点击时调用用户填写的 service 地址，并按 `std_srvs/Trigger` 空请求发起调用。
- 按钮需要复用现有前端的交互反馈风格：
  - 可点击时有激活闪烁
  - ROS 未就绪时进入禁用态
  - 请求执行中显示 pending 态，避免重复点击
- 调试信息只写日志，不恢复控制面板顶部的文字提示框。

### 4. 长按拖拽删除

- 自定义按钮按住约 `520ms` 进入编辑态。
- 编辑态下所有自定义按钮进入抖动/可拖拽状态，底部出现垃圾桶。
- 用户拖动某个自定义按钮到垃圾桶区域并松开后，立即删除该按钮并写回本地存储。
- 若未拖入垃圾桶，则保留编辑态，用户可以继续拖其他按钮；点击空白区域或按 `Esc` 退出编辑态。
- 内建按钮不参与编辑态，不可删除。

## 数据设计

### 1. 本地存储

- `SETTINGS_HOME_PAGE_KEY`
  - 值为设置页 id，例如 `topics`、`workspace`、`cabinRemote`
- `CONTROL_PANEL_VISIBLE_TASKS_KEY`
  - 继续保留，用于内建控制面板任务按钮的可见性数据兼容
- `CUSTOM_CONTROL_PANEL_BUTTONS_KEY`
  - 值为自定义按钮数组，例如：

```json
[
  {
    "id": "custom-service-123",
    "label": "停止索驱",
    "servicePath": "/cabin/motion/stop"
  }
]
```

### 2. 默认值

- 设置主页默认值：`topics`
- 内建控制面板任务默认显示：沿用当前 `CONTROL_PANEL_TASKS` 全量显示
- 自定义按钮默认值：空数组

### 3. 容错

- 如果本地存储里的主页值不在允许列表中，则回退到 `topics`
- 如果自定义按钮数组为空、不是数组、字段缺失或 service 地址非法，则过滤无效项
- service 地址若未以 `/` 开头，前端自动补齐前导 `/`
- 当前实现会去重 `id`，避免重复记录导致渲染冲突

## 代码职责划分

### 1. `storage.js`

- 负责新增“自定义控制面板按钮”读写函数
- 继续负责“设置主页”和“控制面板可见任务”读写
- 统一做本地存储容错和默认值回退

### 2. `controlPanelCatalog.js`

- 继续维护内建任务定义
- 提供 `normalizeCustomControlPanelButtons()`，统一整理自定义按钮的 `id / label / servicePath / serviceType`

### 3. `RosConnectionController.js`

- 提供通用的 `callTriggerService(serviceName, serviceType)` 封装
- 对动态创建的 `ROSLIB.Service` 做轻量缓存，避免重复实例化

### 4. `UIController.js`

- 渲染设置页下拉中的主页小房子动作
- 渲染控制面板加号按钮、自定义按钮表单、自定义按钮摘要列表
- 根据“内建任务 + 自定义按钮”重绘控制面板按钮
- 提供自定义按钮创建、点击调用、长按拖拽删除回调
- 管理编辑态、拖拽代理层和垃圾桶 hover 态

### 5. `TieRobotFrontApp.js`

- 初始化时加载主页配置和自定义按钮列表
- 把主页同步给 `UIController`
- 在用户新增/删除自定义按钮时写回存储
- 在用户点击自定义按钮时调用 `RosConnectionController.callTriggerService()`
- 继续通过现有 `onTaskAction` 驱动内建任务逻辑，不改已有任务处理链路

## 测试范围

- 结构测试覆盖：
  - 设置卡片不再存在单独“主页”选择器
  - 设置页下拉菜单包含主页小房子动作
  - 控制面板标题栏存在加号按钮
  - 控制面板存在自定义按钮表单、服务地址输入框和垃圾桶
  - `storage.js` 中存在自定义按钮持久化读写函数
  - `controlPanelCatalog.js` 中存在自定义按钮规范化函数
  - `RosConnectionController.js` 中存在通用 Trigger service 调用
  - `TieRobotFrontApp.js` 会加载、保存和调用自定义按钮
- 构建验证：
  - `python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py`
  - `npm --prefix src/tie_robot_web/frontend run build`

## 影响范围

- `src/tie_robot_web/frontend/src/utils/storage.js`
- `src/tie_robot_web/frontend/src/config/controlPanelCatalog.js`
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
