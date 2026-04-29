# `tie_robot_web` Viewer 化重构设计

## 背景

当前新前端已经具备以下能力：

- 整页 `3D` 背景与浮动卡片
- ROS 连接、执行主链、工作区、话题图层、日志、帮助站
- 点云、识别点、规划点、TF 的浏览器内可视化

但整体仍偏“页面式拼装”：

- 各个面板直接各自订阅、各自维护状态
- `3D` 与工作区、日志、控制面板之间缺少统一的数据适配层
- 显示布局和业务布局写死在页面装配中
- 缺少类似 Foxglove / Webviz 的 `panel-first`、`layout-driven`、`fixed-frame-first` 架构

这会带来两个直接问题：

1. 前端持续加功能时，状态与数据流会越来越乱。
2. 像“点云悬空”“TF 口径不一致”“某个话题该在哪个面板排查”这类问题，很难有统一落点。

## 参考结论

基于以下资料：

- Foxglove Visualization：<https://docs.foxglove.dev/docs/visualization>
- Foxglove Panels：<https://docs.foxglove.dev/docs/visualization/panels/introduction>
- Foxglove Layouts：<https://docs.foxglove.dev/docs/visualization/layouts>
- Foxglove 3D Panel：<https://docs.foxglove.dev/docs/visualization/panels/3d>
- Foxglove Variables：<https://docs.foxglove.dev/docs/visualization/variables>
- Foxglove Topic Graph：<https://docs.foxglove.dev/docs/visualization/panels/topic-graph>
- Foxglove Embed：<https://docs.foxglove.dev/docs/embed/foxglove-embed>
- Webviz：<https://webviz.io/>、<https://github.com/cruise-automation/webviz>

可以收出 4 个适合本工程的架构原则：

### 1. `Panel-first`

界面不是“一个页面 + 一堆模块”，而是“一个 viewer + 多个可编排 panel”。

### 2. `Layout-driven`

布局本身是可保存、可恢复、可切换的资产，而不是写死在 `UIController` 里。

### 3. `Fixed-frame-first`

所有 `3D` 数据必须先统一到固定显示坐标系（当前工程就是 `map`），再进入渲染层。

### 4. `Data-adapter-first`

原始 ROS 话题不直接喂 UI。要先进入一个前端数据适配/状态层，再分发给面板。

## 目标

将 `tie_robot_web/frontend` 收成一个更像 Foxglove / Webviz 的工具型 viewer，但不盲目照搬它们的交互和品牌样式。

保留本项目已有特色：

- 整页 `3D` 背景
- 左侧控制面板
- 中文控制台风格
- 与 ROS 1 / rosbridge / 现有动作链兼容

## 非目标

本轮不做以下事项：

- 不改后端 ROS 接口语义
- 不引入 Foxglove 本体或 Webviz 本体作为依赖
- 不切到新的前端框架
- 不直接实现“任意拖拽布局编辑器”
- 不做 bag 回放系统

## 目标架构

### 一、顶层分层

```text
frontend/src/
├── app/                 # 应用装配与启动
├── state/               # ViewerStore：统一运行时状态
├── data/                # ROS 话题适配、固定坐标系转换、日志归一化
├── layout/              # 默认布局、布局持久化、布局切换
├── panels/              # Panel 注册表与 panel 元数据
├── controllers/         # 用户动作与 ROS 命令编排
├── views/               # 具体视图（3D / 工作区 / 日志等）
├── ui/                  # 容器 UI、浮窗管理、工具栏
├── config/              # 面板目录、按钮目录、默认显示模式
└── utils/               # 纯工具函数
```

### 二、核心运行对象

#### `ViewerStore`

前端唯一可信运行时状态入口，负责：

- ROS 连接状态
- 当前固定显示坐标系
- 点云、识别点、规划点、TF 的统一缓存
- 当前布局状态
- 当前面板显隐与顺序
- 状态栏指标

要求：

- UI 不直接持有原始 ROS 数据
- panel 只订阅 `ViewerStore` 已规整的数据

#### `SceneAdapter`

负责把以下原始输入统一成 `map` 语义：

- `/tf` 与 `/tf_static`
- `/Scepter/worldCoord/world_coord`
- `/Scepter/worldCoord/raw_world_coord`
- `/coordinate_point`
- `/cabin/pseudo_slam_markers`

它是“点为什么悬空”的唯一修复落点。

#### `PanelRegistry`

统一声明 panel 元数据：

- `id`
- `title`
- `defaultVisible`
- `defaultRect`
- `group`
- `viewFactory`
- `supportsLayout`

#### `LayoutManager`

统一维护：

- 默认布局
- 本地保存布局
- 恢复布局
- 切换布局模板

本工程建议内置 4 套默认布局：

- `执行调试`
- `视觉识别`
- `相机标定`
- `故障排查`

## 面板体系

### 保留并标准化

- `3D Scene`
- `控制面板`
- `工作区`
- `话题图层`
- `日志`

### 新增排查型 panel

- `Topics`
  - 浏览当前关键话题
  - 显示 topic 是否在线、消息类型、最近更新时间

- `TF`
  - 浏览 `map / Scepter_depth_frame / gripper_frame`
  - 显示最近变换与关键平移值
  - 快速暴露“TF 断了 / 高度不合理”

- `Problems`
  - 展示当前 viewer 发现的问题
  - 如：无相机 `camera_info`、TF 不闭合、点云源缺失、日志复用失败

## 布局策略

### 默认布局 A：执行调试

- 全屏 `3D`
- 左侧：控制面板
- 右上：话题图层
- 右中：日志
- 右下：TF / Problems 标签页

### 默认布局 B：视觉识别

- 全屏 `3D`
- 左侧：工作区
- 右上：话题图层
- 右下：Topics / 日志

### 默认布局 C：故障排查

- 全屏 `3D`
- 左侧：Topics
- 右侧：TF
- 底部：Problems + 日志

## 数据流原则

### 当前不再允许的方式

- 某个视图自己直接解释 ROS 消息语义
- 某个面板自己实现一套 `TF` 变换缓存
- 不同面板对同一话题做不同口径的坐标转换

### 统一数据链

```text
ROS / rosbridge
  -> RosConnectionController
  -> SceneAdapter / TopicAdapter / LogAdapter
  -> ViewerStore
  -> Panel Views
```

## 渲染与性能原则

参考 Foxglove 3D panel 的思路，本工程也采用以下约束：

- `3D` 只渲染已经归一化的固定坐标系对象
- 点云保留采样与最大点数限制
- 尽量减少对象数量，优先复用 `BufferGeometry`
- 大坐标位移问题优先在适配层解决，不把误差留给 shader 或视角层

## 分阶段实施

### 阶段 1：骨架化

- 引入 `ViewerStore / PanelRegistry / LayoutManager / SceneAdapter`
- 不改主要功能，只改结构

### 阶段 2：面板化

- 将现有卡片都收进 panel registry
- 引入默认布局与本地保存

### 阶段 3：排查能力

- 新增 `Topics / TF / Problems`
- 让问题定位有明确入口

### 阶段 4：交互优化

- 布局切换
- 更细的图层控制
- 更清晰的工具型操作体验

## 验收标准

满足以下条件时，认为 viewer 化一期完成：

1. 所有现有功能仍可用。
2. 前端存在统一的 `ViewerStore`。
3. 点云、识别点、规划点全部先统一到 `map` 再渲染。
4. 所有主要卡片都来自 `PanelRegistry`，而不是在 `UIController` 里硬编码。
5. 至少支持 2 套以上默认布局切换。
6. 至少具备 `Topics / TF / Problems` 中的 2 个排查型 panel。
