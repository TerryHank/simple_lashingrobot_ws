# 新前端三维视图与话题图层设计

## 背景

当前 `tie_robot_web/frontend` 已经完成了 `robot_viewer` 风格的二维 IR 选点、任务动作和旧前端命令迁移，但还缺少 `robot_viewer` 风格里最关键的三维场景窗口。用户希望新前端不仅有可拖动窗口，还能在网页里直接看到：

- 机器相对相机坐标系的运动
- 一次性识别到的绑扎点在全局空间下的位置
- 根据当前相机模态、TF 和可显示话题切换图层显示

## 目标

在不改现有后端消息口径的前提下，为 `tie_robot_web/frontend` 增加一个 `Three.js` 三维场景面板和一个独立的 `Topic Layers` 图层面板。

三维视图一期必须满足：

- 保留当前 draggable floating panel 风格
- 默认提供相机视角和全局视角切换
- 显示机器简化模型、坐标轴、点云、识别绑扎点
- 复用当前 ROSBridge 直接订阅能力，不引入新的后端接口

## 现有数据源

### TF

- `map -> Scepter_depth_frame`
  - 来源：`tie_robot_process/src/suoquNode.cpp`
  - 语义：索驱带动相机在全局坐标系中的运动
- `Scepter_depth_frame -> gripper_frame`
  - 来源：`tie_robot_perception/scripts/gripper_tf_broadcaster.py`
  - 语义：相机到虎口的静态外参
- `pseudo_slam_point_*`
  - 来源：`tie_robot_process/src/suoquNode.cpp`
  - 语义：规划点的全局 TF
- `bind_point_* / area_*_point_*`
  - 来源：`tie_robot_perception/scripts/stable_point_tf_broadcaster.py`
  - 语义：稳定绑扎点 TF

### 点与图层

- `/coordinate_point` (`tie_robot_msgs/PointsArray`)
  - 当前识别到的绑扎点
- `/cabin/pseudo_slam_markers` (`visualization_msgs/MarkerArray`)
  - pseudo_slam 全局点、高亮点、执行点
- `/Scepter/worldCoord/world_coord` (`sensor_msgs/Image`, `32FC3`)
  - 经过 RANSAC 过滤后的世界点云图
- `/Scepter/worldCoord/raw_world_coord` (`sensor_msgs/Image`, `32FC3`)
  - 原始世界点云图

### 机器模型

- `tie_robot_description/URDF/model.urdf`
  - 当前只有极简 box 模型
  - 一期三维视图采用简化模型渲染，不在本轮引入完整 URDF loader 链

## 三维前端方案

采用 `Three.js + OrbitControls` 落地新的三维视图模块，保留当前 `Vite + app/controllers/ui/views` 结构，不把 `robot_viewer` 整套代码直接嵌入。

### 新增前端模块

- `src/controllers/TopicLayerController.js`
  - 管理图层显示模式、点云源和开关状态
- `src/views/Scene3DView.js`
  - 管理 Three.js 场景、相机、控件、网格、坐标轴、点集、模型
- `src/config/topicLayerCatalog.js`
  - 定义显示模式、可见图层、默认源

### UI 布局

新增两个 floating panel：

- `3D Scene`
  - 内部承载 Three.js renderer
  - 提供 `相机视角 / 全局视角 / 跟随相机` 控件
- `Topic Layers`
  - 提供图层模式与显隐控制

## Topic Layers 设计

### 模式

- `onlyPointCloud`
  - 只显示点云
- `onlyTiePoints`
  - 只显示识别绑扎点
- `pointCloudAndTiePoints`
  - 点云 + 绑扎点
- `planningFocus`
  - 规划点与执行高亮
- `machineOnly`
  - 只显示机器和坐标轴
- `all`
  - 机器 + 点云 + 绑扎点 + 规划点

### 点云源

- `filteredWorldCoord`
  - 订阅 `/Scepter/worldCoord/world_coord`
- `rawWorldCoord`
  - 订阅 `/Scepter/worldCoord/raw_world_coord`

### 控件

- 显示模式下拉框
- 点云源切换
- 开关：
  - 机器
  - 坐标轴
  - 点云
  - 绑扎点
  - 规划点
- 视觉参数：
  - 点大小
  - 点透明度

## 数据处理策略

### 点云

不在一期解析 `PointCloud2`，而是直接复用当前前端更容易消费的 `32FC3 world_coord` 图像：

- 将 `sensor_msgs/Image.data` 解码为 `Float32Array`
- 每个像素按 `x/y/z(mm)` 采样成点
- 默认 stride 采样降密，避免浏览器过载
- `0/0/0` 和非有限值丢弃
- 坐标统一转换为米，挂到 `map` 场景

### 绑扎点

- 订阅 `/coordinate_point`
- 从 `PointCoords.World_coord` 中提取全局点
- 用高亮 sprite / sphere 显示

### 规划点

- 订阅 `/cabin/pseudo_slam_markers`
- 只消费 `SPHERE_LIST` 及其颜色
- 保持与后端 marker 颜色语义一致

### 机器与视角

- 机器简化模型先用 box geometry 表示
- 模型绑定到 `Scepter_depth_frame` 的动态 TF
- `相机视角`：虚拟相机跟随 `Scepter_depth_frame`
- `全局视角`：虚拟相机固定观察 `map`

## 不在本轮范围

- 完整 URDF/mesh 可视化链
- 浏览器端通用 TF 树编辑器
- RViz 级别的 Marker 全类型支持
- PointCloud2 全量解码与颜色映射优化

## 验证

- 结构测试确认以下文件存在：
  - `Scene3DView.js`
  - `TopicLayerController.js`
  - `topicLayerCatalog.js`
- 前端源码包含 `3D Scene` 和 `Topic Layers`
- `npm run build` 通过
- 帮助站重新构建通过
- `catkin_make` 不受前端改动影响
