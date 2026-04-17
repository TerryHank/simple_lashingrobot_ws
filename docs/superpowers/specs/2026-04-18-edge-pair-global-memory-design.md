# Edge Pair And Global Execution Memory Design

## Goal

在当前 `pseudo_slam` 绑定系统上补齐两类能力：

1. 边界区域如果无法组成完整 `2x2`，但还能形成同一根钢筋上的 `2` 个点，也允许参与后续绑扎。
2. 系统需要有一份写盘保存的“已绑扎记忆”，在节点或机器重启后仍能继续判断哪些点已经做过；但每次重新扫描成功后，这份记忆必须整体清空重建。

同时，这份能力不能只作用于 `slam_precomputed`。在 `live_visual` 模式下，到区域后重新识别出来的点，也必须建立在同一套全局棋盘格与执行记忆之上，保证“哪些该绑、哪些已经绑过”是全局一致的。

## Current Problems

当前流程存在这几个结构性缺口：

- 规划层只接受完整 `2x2` 小组，边界区域只有一根钢筋、只能落出 `2` 个点时，会整组丢失。
- 棋盘格成员判定依赖“同时有横向邻居和纵向邻居”，导致边界合法点即使已归到全局网格，也可能被当成非棋盘格成员。
- 系统目前没有正式的“已绑扎点记忆”文件，执行成功后无法跨重启续上。
- `live_visual` 只是到区域后现场识别再执行，还没有服从全局棋盘格与全局已执行记忆。

## Required Behavior

### 1. 扫描后允许两类组并存

扫描完成后，执行路径文件里允许同时存在：

- `matrix_2x2`
- `edge_pair`

其中：

- `matrix_2x2` 仍然代表标准四点组。
- `edge_pair` 代表边界区域只剩一根钢筋时形成的两点组。

### 2. 棋盘格资格不再由“是否完整 2x2”决定

只要一个点能够稳定归入全局网格，就应当具有：

- `global_row`
- `global_col`
- `checkerboard_parity`

是否允许执行，改由以下三条共同决定：

1. 是否是合法全局网格点。
2. 是否满足当前全局跳绑颜色规则。
3. 是否已经在执行记忆里被标记为已绑扎。

### 3. 执行记忆写盘保存

新增执行记忆文件：

- `src/chassis_ctrl/data/bind_execution_memory.json`

要求：

- 节点重启后仍能读取。
- 机器重启后仍能读取。
- 每次新扫描成功并生成新的扫描产物后，原子清空并重建。

### 4. live_visual 服从全局棋盘格和记忆

`live_visual` 模式下，新识别出的点不需要去匹配旧点 ID，但必须：

1. 转为全局世界坐标。
2. 重新归入扫描阶段建立的全局网格。
3. 根据全局 `checkerboard_parity` 判断它是不是该绑颜色。
4. 根据执行记忆判断它是否已经做过。

只有满足上述条件，才允许执行；成功执行后，需要把它写入执行记忆。

### 5. 重新扫描时记忆整体换代

行为锁定如下：

- 扫描开始时不立刻删除旧记忆。
- 只有当新的
  - `pseudo_slam_points.json`
  - `pseudo_slam_bind_path.json`
  生成成功后，才原子重建新的 `bind_execution_memory.json`。
- 如果扫描失败，旧记忆保持不动。

## Architecture

### A. 全局网格成为唯一执行判断来源

扫描成功后，系统固化一套全局网格轴：

- `row` 轴集合
- `col` 轴集合
- `checkerboard_parity` 相位规则

后续不论是：

- `slam_precomputed`
- `live_visual`

都只允许基于这套全局网格做执行判断。

### B. 组类型扩展

当前“只接受 4 点组”的分组逻辑，扩成两类：

1. `matrix_2x2`
   - 优先保留现有主流程。
   - 正常区域继续以四点组为主。

2. `edge_pair`
   - 当某区域无法再组成 `2x2`，但还能在同一根钢筋上稳定形成 `2` 个合法全局网格点时，允许落成两点组。
   - 这类组不享受特殊豁免，仍然受全局棋盘格和执行记忆约束。

### C. 执行记忆作为独立账本

执行记忆不再记录“哪个组做过”，而是记录“哪个全局格点做过”。

推荐字段：

- `scan_session_id`
- `path_origin`
- `executed_points`
- `last_updated_at`

其中 `executed_points` 中每条记录至少包含：

- `global_row`
- `global_col`
- `checkerboard_parity`
- `world_x`
- `world_y`
- `world_z`
- `source_mode`

`source_mode` 取值：

- `slam_precomputed`
- `live_visual`

### D. 记账时机

只在下游执行成功返回后记账。

禁止这些时机写入记忆：

- 点刚被选中
- 点刚准备下发
- 下发中途失败
- 因范围限制、跳绑规则、重复执行而被过滤

## Data Files

### 1. 扫描点文件

文件：

- `src/chassis_ctrl/data/pseudo_slam_points.json`

职责：

- 保存扫描得到的全量世界点。
- 保存全局 `row/col/parity` 信息。
- 保存每个点是否可归入全局棋盘格。

建议继续保留并补强这些字段：

- `global_idx`
- `global_row`
- `global_col`
- `checkerboard_parity`
- `is_checkerboard_member`

### 2. 执行路径文件

文件：

- `src/chassis_ctrl/data/pseudo_slam_bind_path.json`

职责：

- 保存真正可执行的区域与分组。
- 组类型显式写成：
  - `matrix_2x2`
  - `edge_pair`

每个组内仍保存点级别的全局网格字段，以便执行层不需要二次猜测。

### 3. 执行记忆文件

文件：

- `src/chassis_ctrl/data/bind_execution_memory.json`

职责：

- 作为当前扫描会话的执行账本。
- 只记录成功执行过的全局格点。
- 被 `slam_precomputed` 和 `live_visual` 共用。

## Execution Semantics

### 1. slam_precomputed

`slam_precomputed` 模式执行时：

1. 读取 `pseudo_slam_bind_path.json`
2. 逐区域、逐组读取候选点
3. 对每个点检查：
   - 是否在应绑棋盘格颜色中
   - 是否已经在 `bind_execution_memory.json`
4. 只把“该绑且未绑过”的点下发
5. 执行成功后，按点级别写入记忆

如果某个组筛完后没有剩余点：

- 该组直接跳过，不报致命错误。

### 2. live_visual

`live_visual` 模式执行时：

1. 到区域后，现场识别得到新的点。
2. 新点先转为全局世界坐标。
3. 使用扫描阶段建立的全局网格，把新点重新归到最近的：
   - `global_row`
   - `global_col`
4. 计算该点的 `checkerboard_parity`
5. 再检查：
   - 是否是合法全局网格点
   - 是否是应绑颜色
   - 是否已经执行过
6. 只有满足条件的点才允许执行
7. 执行成功后写入 `bind_execution_memory.json`

锁定规则：

- `live_visual` 不去匹配旧点编号。
- `live_visual` 也不能绕开全局棋盘格和执行记忆。

### 3. 无法归网的实时点

如果 `live_visual` 新识别出的点距离任何全局 `row/col` 都过远，无法稳定归入扫描阶段的全局网格：

- 该点不执行
- 该点不记账
- 只打日志说明“未能归入全局棋盘格”

## Lifecycle

### 1. 节点启动

节点启动时：

- 读取当前扫描文件
- 读取当前执行记忆文件

如果存在旧扫描和旧记忆，则继续沿用。

### 2. 重新扫描

重新扫描时：

1. 先保留旧扫描产物和旧记忆。
2. 新扫描成功后，落盘新的：
   - `pseudo_slam_points.json`
   - `pseudo_slam_bind_path.json`
3. 然后原子重建新的 `bind_execution_memory.json`

### 3. 执行成功

每次执行成功后：

- 立即更新内存中的执行记忆
- 立即原子写盘

保证中途断电或节点崩溃时，最多只丢最后一次尚未落盘的成功点。

## File Changes

### Modified Files

- `src/chassis_ctrl/src/suoquNode.cpp`
  - 放宽全局棋盘格成员判定
  - 新增 `edge_pair` 生成逻辑
  - 读取/写入执行记忆
  - 让 `slam_precomputed` 和 `live_visual` 共用同一套全局账本判断

- `src/chassis_ctrl/src/moduanNode.cpp`
  - 保持执行接口不变
  - 如需要，补充点级执行成功回传与记账协同

- `src/chassis_ctrl/test/test_pointai_order.py`
  - 新增边界两点组、执行记忆、重启续作、重新扫描清空记忆等测试

### New Files

- `src/chassis_ctrl/data/bind_execution_memory.json`
  - 当前扫描会话的已执行格点账本

## Risks And Mitigations

### 风险 1：边界点被放宽后，杂点也混进执行路径

缓解：

- 不是所有两点都放行，只允许能稳定归入全局网格的两点组成为 `edge_pair`
- 仍然保留全局网格合法性检查

### 风险 2：live_visual 新点漂移，污染执行记忆

缓解：

- 新点必须先归入扫描期全局网格
- 归不进去的点禁止执行，也禁止记账

### 风险 3：扫描失败导致记忆被误清空

缓解：

- 只在新扫描产物完整落盘成功后重建记忆
- 扫描开始时不删旧记忆

### 风险 4：组级跳过与点级记忆冲突

缓解：

- 记忆只记点，不记组
- 组只是执行组织单位
- 某个组内部分点已做过时，允许只执行剩余点

## Success Criteria

实现完成后，应满足：

1. 扫描生成的执行路径里，边界区域可以出现 `edge_pair` 两点组。
2. `edge_pair` 同样参与全局棋盘格 `checkerboard_parity`。
3. `slam_precomputed` 执行成功后，会把成功点写入 `bind_execution_memory.json`。
4. 节点或机器重启后，系统仍会继续跳过已经做过的全局格点。
5. `live_visual` 到区域后现场识别的新点，也会服从全局棋盘格和执行记忆。
6. 重新扫描成功后，旧执行记忆整体清空，按新点阵重建。
