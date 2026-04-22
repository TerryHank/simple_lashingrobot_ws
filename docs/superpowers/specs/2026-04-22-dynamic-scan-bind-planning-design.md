# Dynamic Scan Bind Planning Design

## Goal

在保留当前“扫描 -> 直线 S2 识别全局点 -> 执行层继续做视觉精校”主链的前提下，把扫描完成后的区域规划来源，从静态 [path_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/path_points.json) 切换成“基于已识别全局点、TCP 行程范围和当前工具位姿约束的动态规划”。

新行为要求：

1. 抬升到固定扫描位姿后完成一次全局识别。
2. 识别到全局点后，系统自行计算需要移动的每个索驱 `x/y/z` 位姿。
3. 每个区域固定规划为 `4` 点模板。
4. 当某个局部窗口内剩余未绑扎点不足 `4` 个时，允许用已绑定/占位邻点补齐模板，但真实执行仍只执行未完成点。
5. 执行层继续保持现有“预计算直执行 + 到位后视觉精校”的策略。

## Current Problems

当前工程中，扫描后的规划和执行有两个核心耦合：

1. `run_pseudo_slam_scan()` 仍然按 `con_path` 遍历每个静态区域，并用该区域中心去筛选 `2x2` / `edge_pair` 组，输出 [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json)。
2. `run_bind_from_scan()` 与 `run_live_visual_global_work()` 都默认 `areas[].cabin_pose` 是来自静态规划路径的区域位姿。

这会带来两个问题：

- 规划结果高度依赖 [path_points.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/path_points.json) 的区域步距，而不是识别后的真实全局点分布。
- 真正决定一组点是否可执行的是 `gripper_frame` 下的局部坐标范围，但现有规划层和执行层使用的范围口径并不一致。

## Locked Requirements

### 1. 扫描链保持不变

以下行为不改：

- 固定扫描位姿触发扫描。
- S2 直线链负责输出全局点。
- 执行层仍然是先移动索驱，再把局部点送给末端执行服务，区域内继续保留视觉精校。

### 2. 规划来源改成动态

扫描结束后，不再使用静态 `con_path` 作为区域划分来源，而是直接根据全局点反推一系列动态区域。

每个动态区域必须包含：

- `area_index`
- `cabin_pose.x/y/z`
- `groups`
- 每组恰好 `4` 个模板点

### 3. TCP 行程范围以用户给定为准

动态规划与执行前过滤统一使用：

- `x: 0..300`
- `y: 0..300`
- `z: 0..140`

这里的坐标系固定为 `gripper_frame` 下的局部工具坐标。

### 4. 4 点模板补齐规则

当某个局部窗口里剩余未绑扎点不足 `4` 个时：

- 仍然必须规划出 `4` 点模板。
- 允许把邻近的已绑扎/占位点纳入模板补齐。
- 但执行时只对未完成点下发。

## Recommended Architecture

### A. 新增“动态区域规划器”

在 [suoquNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp) 中新增一层动态规划器，输入为：

- 扫描后的全局世界点
- 当前扫描位姿
- `gripper_frame` 工作范围

输出为：

- 动态区域列表
- 每个区域的索驱 `x/y/z`
- 每个区域下的 `4` 点模板组

### B. 动态区域位姿求解

对每个候选世界点，反推出一个“若要让该点落到虎口模板参考位”的索驱位姿。推荐使用固定模板中心：

- `local x = 150`
- `local y = 150`
- `local z = 70`

基于该候选位姿，把所有全局点转换到 `gripper_frame` 下，保留落入 `0..300 / 0..300 / 0..140` 的点，形成该区域候选覆盖集。

### C. 每区域只保留一个 4 点模板组

为避免重复改执行层语义，每个区域仍维持“一次下发一个组”的现有模式。动态规划器会：

1. 从剩余未规划点中选择一个种子点。
2. 求出一个候选索驱位姿。
3. 在该位姿下收集可覆盖点。
4. 选出最符合模板的 `4` 个点，必要时按补齐规则凑满。
5. 把该组从“剩余未规划点”里标记为已覆盖。

### D. 执行入口复用现有两条主链

以下执行策略不改：

- [run_bind_from_scan](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp)
- [run_live_visual_global_work](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp)

它们只需要改成消费新的动态 `areas[].cabin_pose` 与 `groups[].points`。

## Data Model Changes

### `pseudo_slam_bind_path.json`

仍沿用现有文件，但语义变化为：

- `areas[].cabin_pose`：动态规划求得的索驱位姿，不再等于静态路径点。
- `groups[].points`：每组固定 `4` 个模板点。
- `path_origin`：保留扫描起点语义，用于棋盘格排序和执行账本兼容。
- `path_signature`：继续保留，但仅用于“本次扫描/执行会话一致性”，不再表示静态区域步距本身。

## Range Unification

本次必须同步收口这几个不一致的范围：

- [suoquNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/suoquNode.cpp) 当前规划过滤只卡 `x/y <= 320`
- [moduanNode_show.cpp](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/src/moduanNode_show.cpp) 当前真实执行放行 `x < 380, y < 316, z < 120`

统一后应为：

- 规划过滤：`x/y/z` 全部按 `300/300/140`
- 真执行过滤：同样按 `300/300/140`

## Testing Strategy

重点测试分三层：

1. 规划层
   - 动态区域位姿是否由世界点反解而来
   - 每组是否稳定生成 `4` 点模板
   - 最后一组不足 `4` 个未完成点时是否正确补齐

2. 工件层
   - [pseudo_slam_bind_path.json](/home/hyq-/simple_lashingrobot_ws/src/chassis_ctrl/data/pseudo_slam_bind_path.json) 是否不再依赖静态 `path_points` 区域位姿
   - 动态区域是否带完整 `cabin_pose`

3. 执行层
   - 预计算执行与 `live_visual` 是否读取动态区域位姿
   - 执行前过滤范围是否统一为 `300/300/140`

## Risks

### 风险 1：动态区域顺序与历史账本对不上

缓解：

- 保留 `path_origin` 和全局点棋盘格索引
- 执行账本继续以 `global_idx / row / col / parity` 对齐，而不是以区域序号对齐

### 风险 2：最后一组模板补齐导致重复绑扎

缓解：

- 模板可补齐，但真正下发前仍要过执行记忆过滤
- 已绑定点仅作为模板占位，不作为实际执行目标

### 风险 3：规划层与真实末端范围不一致

缓解：

- 本次统一规划层和执行层的局部坐标范围常量
- 用一组回归测试同时卡住 `suoquNode` 和 `moduanNode_show` 的口径
