# Pseudo-SLAM Bind Workflow Design

## Goal

把当前“走到一个区域后立即自适应高度 + 识别 + 绑扎”的流程，改成两阶段：

1. 先按已规划好的索驱区域路径完整走一遍，每个区域只做识别，不做自适应高度，不做绑扎。
2. 识别出的点统一落到索驱世界坐标系 `cabin_frame` 下，过滤出索驱规划工作区范围内的点，做去重和归档，生成后续绑扎路径。
3. 点击全局作业时，直接按之前伪“slam”得到的绑扎路径执行，不再二次识别，也不再做自适应高度。
4. 如果执行阶段某个预生成绑扎点不适合绑扎，则跳过该点继续下一个，不中断整条全局作业流程。

同时，把索驱反馈坐标正式发布成 TF：

- `cabin_frame -> Scepter_depth_frame`
- `Scepter_depth_frame -> gripper_frame`（现有静态外参保留）

并新增独立节点把最终绑扎点发布成世界系 TF：

- `cabin_frame -> bind_point_<i>`
- `cabin_frame -> area_<N>_point_<i>`

## Current Problems

当前工程存在这些结构性问题：

- `suoquNode.cpp` 的全局作业流程是“到一个区域就立刻请求视觉，再做自适应高度，再触发绑扎”，识别和执行强耦合。
- `pointAI.py` 当前输出重点是“当前执行点”，不适合做完整工作区扫描建图。
- 现在没有正式的索驱世界 TF；`pointAI.py` 只消费 `Scepter_depth_frame -> gripper_frame`，无法把跨区域识别点统一落到索驱世界系里。
- 重新点击全局作业时，还需要再次识别，无法复用上一轮扫出来的绑扎点路径。

## Required Behavior

### 1. 新增“扫描建图”按钮流程

前端新增一个按钮，触发新的扫描流程（下文称“伪 slam 扫描”）：

- 索驱按已经规划好的区域路径完整走一遍。
- 每个区域只做一次识别，不做自适应高度，不做绑扎。
- 识别时使用整个矩形画幅内的点，不只取当前 2x2 执行点。
- 点统一转换到 `cabin_frame` 世界坐标系。
- 只保留落在索驱规划工作区范围内的点。
- 扫描完成后，自动生成后续绑扎路径文件。

### 2. 全局作业改成复用扫描结果

点击“全局作业”时：

- 不再重新请求视觉识别。
- 不再做自适应高度。
- 直接读取伪 slam 扫描阶段生成的绑扎路径。
- 索驱和末端按该路径完成绑扎。
- 如果某个预生成点执行时失败或被判定为不适合绑扎，则跳过该点继续执行后续点。

### 3. 索驱世界坐标 TF

`suoquNode.cpp` 需要持续把索驱反馈位置发布成动态 TF：

- 父坐标系：`cabin_frame`
- 子坐标系：`Scepter_depth_frame`

定义：

- `cabin_frame` 的原点和轴方向，直接等同于 `suoquNode` 当前索驱工作坐标系的零点和 `X/Y/Z` 正方向。
- `Scepter_depth_frame` 在 `cabin_frame` 下的位置，由索驱实时反馈的 `cabin_state.X/Y/Z` 决定。

现有静态外参继续保留：

- `Scepter_depth_frame -> gripper_frame`

### 4. 最终绑扎点 TF

新增独立节点 `stable_point_tf_broadcaster.py`：

- 不参与点检测。
- 只旁路消费 `pointAI.py` 产出的最终绑扎点结果。
- 不广播自适应高度阶段的临时点。
- 只广播最终用于绑扎的点：
  - 当前区域：`bind_point_<i>`
  - 历史归档：`area_<N>_point_<i>`

稳定规则：

- 只对最终绑扎点做稳定判断。
- 以点编号 `idx` 为单位判断。
- 连续 `2` 帧都检测到同一编号点。
- 两帧内该点 `z` 波动满足 `max(z)-min(z) <= 8mm`（即 `+-4mm`）。
- 满足后才刷新 `bind_point_<i>`。

区域归档规则：

- 节点订阅 `/cabin/area_progress`。
- 当区域更新时，把上一块区域已经稳定的当前点固化成：
  - `area_<old_index>_point_<i>`
- 这些历史点保留在 `cabin_frame` 下，不删除。

## Architecture

### A. 索驱扫描与执行模式拆分

`suoquNode.cpp` 需要拆出两类流程：

1. `scan_only` / `pseudo_slam_scan`
   - 走完整个规划路径。
   - 每个区域只请求一次“扫描识别”视觉结果。
   - 收集世界坐标点，不执行绑扎。

2. `bind_from_scan`
   - 读取扫描阶段生成的绑扎路径。
   - 直接执行绑扎。
   - 不再调用视觉识别。
   - 不再做自适应高度。
   - 某个点执行失败时跳过该点，继续下一个。

### B. 视觉输出拆分

`pointAI.py` 需要支持两类视觉输出：

1. 现有执行视觉输出
   - 用于当前绑扎链路中的最终执行点。

2. 新增扫描视觉输出
   - 使用整个矩形画幅内的点。
   - 转到 `cabin_frame` 世界坐标系。
   - 过滤掉不在索驱规划工作区范围内的点。
   - 输出给扫描阶段存储与后续路径生成使用。

### C. 伪 slam 数据落盘

新增扫描结果文件，例如：

- `data/pseudo_slam_points.json`
- `data/pseudo_slam_bind_path.json`

`pseudo_slam_points.json` 存原始世界坐标点云式结果（已按工作区过滤和去重）。
`pseudo_slam_bind_path.json` 存最终要绑扎的路径点。

### D. 前端按钮/入口

在现有前端命令体系下新增一个“扫描建图”按钮入口，推荐走 `topics_transfer.cpp` 做 topic/service 转换，和现有 `/web/cabin/plan_path`、`/web/cabin/start_global_work` 保持同一风格。

## Data Flow

### 扫描阶段

1. 前端点击“扫描建图”。
2. `topics_transfer.cpp` 转发到新的索驱扫描服务。
3. `suoquNode.cpp` 读取已规划路径，逐区域移动。
4. 到达每个区域后，请求 `pointAI.py` 的扫描模式。
5. `pointAI.py` 返回当前矩形画幅内、已过滤工作区的世界坐标点。
6. `suoquNode.cpp` 或专用扫描路径生成逻辑将这些点合并、去重、落盘。
7. 扫描完成后生成 `pseudo_slam_bind_path.json`。

### 执行阶段

1. 前端点击“开始全局作业”。
2. `suoquNode.cpp` 直接读取 `pseudo_slam_bind_path.json`。
3. 索驱和末端按此路径执行绑扎。
4. 不再请求视觉识别。
5. 不再做自适应高度。
6. 若某个点执行失败，则记录日志并跳过到下一个点。

## File Changes

### New Files

- `src/chassis_ctrl/scripts/stable_point_tf_broadcaster.py`
  - 消费最终绑扎点，做稳定判定与 TF 广播/归档。

- `src/chassis_ctrl/data/pseudo_slam_points.json`
  - 保存扫描阶段的世界坐标点集合。

- `src/chassis_ctrl/data/pseudo_slam_bind_path.json`
  - 保存扫描后自动生成的绑扎路径。

### Modified Files

- `src/chassis_ctrl/src/suoquNode.cpp`
  - 发布 `cabin_frame -> Scepter_depth_frame`
  - 新增扫描模式和按扫描路径执行模式
  - 读取/写入伪 slam 路径文件

- `src/chassis_ctrl/src/topics_transfer.cpp`
  - 新增“扫描建图”按钮入口映射

- `src/chassis_ctrl/scripts/debug_button_node.py`
  - 新增按钮调试入口

- `src/chassis_ctrl/scripts/pointAI.py`
  - 新增扫描模式
  - 以整个矩形画幅内的点为输入
  - 输出 `cabin_frame` 下的工作区内点

- `src/chassis_ctrl/launch/*.launch`
  - 把 `stable_point_tf_broadcaster.py` 接入真实运行入口

## Open Decisions Locked Here

为避免实现阶段继续漂移，这里锁定以下决策：

- 世界坐标系固定命名为 `cabin_frame`。
- `cabin_frame` 原点/轴方向直接等同于 `suoquNode` 当前索驱工作坐标系。
- 只广播最终绑扎点 TF，不广播自适应高度阶段临时点。
- 稳定判定固定为：`2` 帧内同一点 `z` 在 `+-4mm`。
- 区域切换后，上一块区域的点保留为 `area_<N>_point_<i>`。
- 全局作业执行时直接复用伪 slam 路径，不再二次识别。
- 全局作业执行时完全取消自适应高度。
- 预生成绑扎点执行失败时跳过，不中断整条作业流程。

## Risks and Mitigations

### 风险 1：扫描点跨区域重复

缓解：

- 在 `cabin_frame` 世界坐标里做去重。
- 复用现有欧氏距离去重思路，但阈值切到世界坐标去判断。

### 风险 2：扫描点和当前绑扎点混淆

缓解：

- 扫描模式和绑扎执行模式严格拆开。
- `stable_point_tf_broadcaster.py` 只处理最终绑扎点，不消费扫描点。

### 风险 3：世界 TF 链不完整

缓解：

- 先由 `suoquNode.cpp` 补齐 `cabin_frame -> Scepter_depth_frame`
- 再接入扫描与稳定点广播逻辑。

## Success Criteria

实现完成后，应满足：

1. 可以先按规划路径完整扫描一次，不绑扎、不自适应。
2. 扫描得到的点以 `cabin_frame` 世界坐标保存。
3. 只保留索驱规划工作区内的点。
4. 扫描完成后自动生成绑扎路径。
5. 点击全局作业时，不再识别，不再自适应，直接按扫描路径执行。
6. `cabin_frame -> Scepter_depth_frame -> gripper_frame` TF 链完整可查。
7. 最终绑扎点可通过 `bind_point_<i>` 与 `area_<N>_point_<i>` 在 TF 中查询。
8. 某个预生成点执行失败时，系统会跳过该点并继续后续点，而不是整条流程报错退出。
