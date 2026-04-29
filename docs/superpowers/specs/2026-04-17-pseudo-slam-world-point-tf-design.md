# Pseudo-SLAM World Point TF Design

## Goal
在现有 `pseudo_slam_points` 世界坐标可视化基础上，为每个扫描得到的世界点额外发布一个 TF 子坐标系，使 RViz 中的 `TF` 显示与 `MarkerArray` 世界点完全重叠。

## Scope
- 不改变扫描识别逻辑、分组逻辑和执行逻辑。
- 继续保留 `/cabin/pseudo_slam_markers` 的 `MarkerArray` 可视化。
- 新增 `map -> pseudo_slam_point_<global_idx>` 的动态 TF。

## Coordinate System
- 父坐标系固定为 `map`。
- 子坐标系命名固定为 `pseudo_slam_point_<global_idx>`。
- 位置直接使用 `pseudo_slam_points` 的世界坐标，单位从毫米转换为米。
- 旋转统一使用单位四元数。

## Lifecycle
- 开始新的扫描建图时：
  - 清空旧的 `MarkerArray`；
  - 清空节点内部维护的当前扫描点 TF 集合；
  - 后续只持续广播新扫描得到的世界点集合。
- 扫描过程中：
  - `merged_world_points` 每次更新后，同步刷新 `MarkerArray` 和当前扫描点 TF 集合。
- 扫描完成后：
  - 保持最后一版 `MarkerArray`；
  - 持续广播最后一版扫描点 TF，使 RViz 中世界点坐标系保持可见。

## Implementation Approach
- 在 `suoquNode.cpp` 内直接实现，不新增独立节点。
- 复用现有 `cabin_tf_broadcaster` 发布：
  - `map -> Scepter_depth_frame`
  - `map -> pseudo_slam_point_<global_idx>`
- 增加一个线程安全的当前扫描点缓存，供持续广播使用。
- 在索驱状态线程中持续广播当前扫描点 TF，保证扫描完成后 TF 不会自然超时消失。

## Caveat
- TF 没有 `DELETEALL` 语义。实现会在新扫描开始时清空内部缓存并停止旧点广播，新点集合会立即开始广播。
- 旧的扫描点 TF 在 TF 缓存窗口内可能会短暂残留，随后自动消失；`MarkerArray` 则会立即清空。

## Verification
- 静态测试断言：
  - `pseudo_slam_point_`
  - `sendTransform`
  - `/cabin/pseudo_slam_markers`
  - `map`
- 构建验证：`catkin_make -DCATKIN_WHITELIST_PACKAGES="fast_image_solve;chassis_ctrl" -j1`
- 运行验证：
  - `roslaunch chassis_ctrl run.launch`
  - RViz Fixed Frame = `map`
  - 同时打开 `TF` 与 `MarkerArray`
  - 触发扫描后观察点与 TF 同步累积并保持
