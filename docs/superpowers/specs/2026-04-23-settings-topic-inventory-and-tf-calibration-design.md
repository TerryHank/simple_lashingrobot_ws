# 设置卡片话题总览与相机-TCP外参页设计

## 目标

- 将 `设置` 卡片首页改成 `话题总览`
- 在 `设置` 中新增 `相机-TCP外参` 页面
- 话题总览展示当前 ROS 中存活的话题及类型
- 外参页读取当前 `Scepter_depth_frame -> gripper_frame` 的相对位姿，并允许前端热修改

## 方案

### 1. 话题总览

- 前端通过 `roslib` 的 `getTopicsAndRawTypes()` 周期轮询当前活跃话题
- 仅展示：
  - 话题名
  - 类型
- 默认进入该页，作为 `设置` 的首页

### 2. 相机-TCP外参

- 直接复用现有 `gripper_tf_broadcaster.py` 已支持的热更新入口：
  - `/web/pointAI/set_offset`
  - `geometry_msgs/Pose`
- 页面读取当前 TF：
  - `Scepter_depth_frame -> gripper_frame`
- 前端显示为 `translation_mm`
  - `x = -tf.x * 1000`
  - `y = -tf.y * 1000`
  - `z = -tf.z * 1000`
- 用户修改后点击应用，前端发布到 `/web/pointAI/set_offset`

## 边界

- 本轮只做平移 `translation_mm` 热修改，不做旋转 `rotation_rpy`
- 只在前端做话题发现，不新增后端 topic inventory 服务
- 不改变现有 `gripper_tf_broadcaster` 的热更新协议

## 影响范围

- `tie_robot_web` 前端 UI
- `RosConnectionController`
- `Scene3DView`
- 结构测试
