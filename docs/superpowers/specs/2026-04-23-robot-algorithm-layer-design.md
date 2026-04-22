# `robot_algorithm_layer` 设计算法层独立包设计

## 背景

当前工程已经拆出了：

- `robot_interface_hub`：前端、接口、顶层 launch
- `robot_hw_drivers`：相机、索驱 TCP、线性模组 Modbus 等底层驱动
- `chassis_ctrl`：执行流程、状态机、节点编排

但 `chassis_ctrl` 和 `pointAI.py` 里仍混着大量纯算法代码：

- 视觉侧：工作区几何、S2 周期估计、透视整形、线段映射、交点生成
- 底盘侧：`pseudo_slam` 动态分组、局部可达性筛选、蛇形排序、执行起点推导

这些能力并不属于接口层、驱动层，也不应继续夹在应用节点里。

## 目标

新建独立算法包 `robot_algorithm_layer`，承接视觉与底盘中的纯算法逻辑，形成清晰的四层结构：

```text
robot_interface_hub   -> 接口层
robot_algorithm_layer -> 算法层
robot_hw_drivers      -> 驱动层
chassis_ctrl          -> 应用层
```

## 设计原则

### 1. 算法层只做“怎么算”

`robot_algorithm_layer` 只放：

- 纯计算
- 数据变换
- 几何/排序/分组/过滤规则

不放：

- ROS topic / service / action 回调
- JSON 账本读写
- 固定文件路径
- 硬件连接
- 执行状态机

### 2. 第一阶段先抽最纯、最稳定的算法核

本轮不追求把所有逻辑一次性搬空，而是先抽两块最清晰、复用价值最高的算法：

- 视觉算法核：`pointAI.py` 的工作区 S2 纯函数
- 规划算法核：`suoquNode.cpp` 的动态分组与蛇形路径算法

### 3. 保持应用层入口不变

第一阶段不改前端接口，不改 ROS service 名称，不改动作链。  
`pointAI.py` 和 `suoquNode.cpp` 仍是入口，但内部调用改为转向 `robot_algorithm_layer`。

## 包结构

```text
src/robot_algorithm_layer/
  package.xml
  CMakeLists.txt
  setup.py
  include/robot_algorithm_layer/planning/
    dynamic_bind_planning.hpp
  src/planning/
    dynamic_bind_planning.cpp
  src/robot_algorithm_layer/
    __init__.py
    perception/
      __init__.py
      workspace_s2.py
```

## 第一阶段迁移范围

### 视觉算法

迁入 `robot_algorithm_layer/perception/workspace_s2.py`：

- 多边形顺时针排序
- S2 轴向 profile 平滑
- 周期与相位估计
- 工作区 bbox 推导
- 响应图归一化
- 透视整形矩阵推导
- 整形点反投影
- 投影线段生成

这些函数保持纯函数风格，只依赖 `numpy` / `cv2`。

仍保留在 `pointAI.py` 的内容：

- 图像/深度/世界坐标通道获取
- ROS publisher / subscriber
- 参数读取
- 结果消息组装
- 显示绘制与运行流程

### 底盘规划算法

迁入 `robot_algorithm_layer::planning`：

- 动态候选点构造
- 矩阵四点模板选择
- 边对选择
- 局部原点距离评分
- 种子点选择
- 动态区域生成
- 执行起点推导
- 蛇形行排序

这些函数保持“给定输入点集、变换和参数，输出分组结果”的纯规划模式。

仍保留在 `suoquNode.cpp` 的内容：

- TF 查询
- 扫描流程控制
- JSON 账本读写
- 执行记忆管理
- service 回调
- 索驱/模组调用

## C++ 公共接口

公开头文件：`dynamic_bind_planning.hpp`

对外暴露：

- `CabinPoint`
- `PseudoSlamBindGroup`
- `PseudoSlamGroupedAreaEntry`
- `BindExecutionPathOriginPose`
- `DynamicBindPlannerConfig`
- `build_dynamic_bind_area_entries_from_scan_world(...)`
- `build_dynamic_bind_execution_path_origin(...)`
- `sort_bind_area_entries_by_snake_rows(...)`

其中 `tf2::Transform` 由应用层先查询好后传入算法层，避免算法层直接依赖 TF buffer。

## Python 公共接口

公开模块：`robot_algorithm_layer.perception.workspace_s2`

对外暴露：

- `sort_polygon_points_clockwise`
- `sort_polygon_indices_clockwise`
- `smooth_workspace_s2_profile`
- `estimate_workspace_s2_period_and_phase`
- `build_workspace_s2_line_positions`
- `build_workspace_s2_bbox`
- `build_workspace_s2_axis_profile`
- `normalize_workspace_s2_response`
- `build_workspace_s2_rectified_geometry`
- `map_workspace_s2_rectified_points_to_image`
- `build_workspace_s2_projective_line_segments`

## 依赖关系

### `robot_algorithm_layer`

依赖：

- `robot_interface_hub`：复用 `PointCoords`
- `tf2`：规划算法输入变换
- Python 运行时：`numpy`、`cv2`

不依赖：

- `robot_hw_drivers`
- `robot_interface_hub` 的前端/API 代码
- `chassis_ctrl` 的状态机和账本逻辑

### `chassis_ctrl`

新增依赖：

- `robot_algorithm_layer`

并改为：

- `pointAI.py` 导入 Python 感知算法模块
- `suoquNode.cpp` 链接并调用 C++ 规划算法库

## 测试策略

第一阶段测试重点是边界正确和调用切换正确：

1. `robot_algorithm_layer` 包存在，且同时拥有 perception 与 planning 源码。
2. `pointAI.py` 改为从 `robot_algorithm_layer.perception.workspace_s2` 导入纯算法函数。
3. `suoquNode.cpp` 改为包含 `robot_algorithm_layer/planning/dynamic_bind_planning.hpp`。
4. `chassis_ctrl` 的 `CMakeLists.txt` / `package.xml` 改为依赖 `robot_algorithm_layer`。
5. `catkin_make` 能同时编译：
   - `robot_algorithm_layer`
   - `robot_interface_hub`
   - `robot_hw_drivers`
   - `chassis_ctrl`

## 非目标

本轮明确不做：

- 把 `pseudo_slam_points.json` / `pseudo_slam_bind_path.json` 读写迁入算法层
- 把 live_visual 执行流程迁入算法层
- 把所有 `pointAI.py` 算法一次性全部拆空
- 把 `moduanNode` 的执行策略或 PLC 业务逻辑迁入算法层

## 完成标准

完成后应满足：

1. 新增 `robot_algorithm_layer` 独立包。
2. 视觉侧已有一批纯 S2 算法从 `pointAI.py` 抽离。
3. 底盘侧动态分组/蛇形排序算法从 `suoquNode.cpp` 抽离。
4. `chassis_ctrl` 已切到新算法包调用。
5. 定向测试通过，定向构建通过。
