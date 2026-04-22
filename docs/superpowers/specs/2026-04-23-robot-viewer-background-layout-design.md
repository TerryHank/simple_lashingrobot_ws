# 新前端整页三维背景布局设计

## 背景

当前 `tie_robot_web/frontend` 已经完成了功能迁移：新前端具备 IR 选点、Action/Service 主链、3D 场景、Topic Layers、状态监控和老前端命令面板。但布局仍然更像“二维主舞台 + 多个业务浮窗”，与用户期望的 `robot_viewer` 风格不一致。

用户当前明确要求：

- 布局参考 `robot_viewer`
- 整个页面以三维视图作为背景主画布
- 顶部和四周采用玻璃态、可拖动的浮动窗口
- 保留当前新前端的功能
- 参考旧 `src/APP` 的功能面，不参考它的布局

## 目标

将 `tie_robot_web/frontend` 重构为如下布局：

- 全页面 `Three.js` 视图作为背景层
- 顶部工具条承担品牌、面板显隐、视角快捷切换和帮助入口
- IR 工作区、任务主链、Topic Layers、状态日志、老前端命令等改为覆盖在三维背景上的浮动面板
- 继续保留现有 ROS 数据流、3D 话题订阅、IR 选点和执行入口

本轮只改布局和前端组织，不主动改变后端接口和业务行为。

## 参考边界

### `robot_viewer` 负责提供的参考

- 页面整体构图
  - 整页三维背景
  - 顶部工具条
  - 左右两侧玻璃态浮动面板
- 面板交互
  - 可拖动
  - 工具条控制显隐
- 视觉语言
  - 深色背景
  - 半透明玻璃态卡片
  - 顶部圆角 pill 工具按钮

### `src/APP` 负责提供的参考

- 当前业务功能集合
- 老前端常用操作语义
- 状态信息和控制入口的覆盖面

### 明确不做的事

- 不把 `robot_viewer` 的模型编辑器、文件树、代码编辑器整套搬入
- 不引入 `src/APP` 的 React 源码重建
- 不修改 ROS 接口语义
- 不把 3D 视图切回单独的面板

## 新布局方案

### 页面分层

页面改为 4 层：

1. `scene-background`
   - 整页 `Three.js` 视图
   - 始终铺满窗口
2. `scene-overlay`
   - 轻度渐变与 vignette
   - 保证面板可读性
3. `top-toolbar`
   - 品牌标题
   - 面板显隐按钮
   - 视角快捷按钮
   - 帮助入口
4. `floating-panels`
   - 业务面板层
   - IR 工作区、任务、Topic Layers、状态、日志、命令

### 面板布局

- 左上：`Workspace`
  - IR 图像
  - 四边形选点
  - 显示增强
- 左下：`Tasks`
  - 扫描
  - 识别
  - 开始执行层
  - 保留记忆开始
  - 直接执行账本测试
- 右上：`Topic Layers`
  - 显示模式
  - 点云源
  - 图层开关
  - 点大小 / 透明度
- 右中：`Status`
  - 状态灯
  - 结果消息
- 右下：`Logs`
  - 运行日志
- 下方宽面板：`Legacy Commands`
  - 保留旧前端常用命令

### 顶部工具条

工具条只做两件事：

- 面板显隐
  - Workspace
  - Tasks
  - Topic Layers
  - Status
  - Logs
  - Commands
- 场景快捷切换
  - 相机视角
  - 全局视角
  - 跟随相机
  - 点云快捷显隐
  - 绑扎点快捷显隐

## 与现有模块的对应关系

- `Scene3DView`
  - 继续负责三维场景
  - 但容器从卡片内部改为整页背景节点
- `TopicLayerController`
  - 继续负责图层模式与显示状态
  - 额外接收顶部工具条的快捷操作
- `UIController`
  - 负责重新输出整个页面结构
  - 引入工具条、面板显隐状态和面板控制按钮
- `PanelManager`
  - 继续负责拖动
  - 需要兼容新的默认定位和隐藏状态
- `WorkspaceCanvasView`
  - 保留现有 IR 画布逻辑
  - 只调整容器呈现

## 数据流

布局改造后，数据流不变：

- `RosConnectionController`
  - 继续订阅 `/tf`、`/tf_static`、`/coordinate_point`、`/cabin/pseudo_slam_markers`、`/Scepter/worldCoord/world_coord`、`/Scepter/worldCoord/raw_world_coord`
- `Scene3DView`
  - 继续作为三维数据消费端
- `WorkspaceCanvasView`
  - 继续作为 IR 图和结果叠加消费端
- `TaskActionController`
  - 继续桥接扫描、执行、账本测试动作
- `LegacyCommandController`
  - 继续保留旧控制入口

## 测试与验证

本轮至少补 3 类验证：

1. 结构测试
   - 前端存在新的布局关键文件和帮助站说明
2. 语法与构建
   - `node --check`
   - `npm run build`
   - `help npm run build`
3. 回归
   - `tie_robot_bringup` 的结构测试继续通过

## 风险

### 风险 1：三维背景全屏后遮挡业务面板

通过分层 DOM 和显式 `z-index` 解决，保证背景画布永远在底层。

### 风险 2：浮动面板过多导致小屏拥挤

保持当前桌面优先口径；移动端只做基本可访问，不追求等价体验。

### 风险 3：功能迁移时把现有动作链打散

本轮不改 `TaskActionController`、`RosConnectionController` 的行为语义，只调整 UI 组织与少量连接点。

