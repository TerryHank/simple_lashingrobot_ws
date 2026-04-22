# 新前端三维视图与话题图层实现计划

1. 先补结构测试，要求新前端具备：
   - `Scene3DView.js`
   - `TopicLayerController.js`
   - `topicLayerCatalog.js`
   - `package.json` 引入 `three`
   - `UIController.js` 出现 `3D Scene` 和 `Topic Layers`

2. 在 `frontend` 中新增三维视图与图层配置模块：
   - `src/views/Scene3DView.js`
   - `src/controllers/TopicLayerController.js`
   - `src/config/topicLayerCatalog.js`

3. 扩展 `UIController`：
   - 新增 `3D Scene` floating panel
   - 新增 `Topic Layers` floating panel
   - 提供模式切换、图层开关、点大小与透明度控件

4. 扩展 `RosConnectionController`：
   - 订阅 `/tf`
   - 订阅 `/tf_static`
   - 订阅 `/coordinate_point`
   - 订阅 `/cabin/pseudo_slam_markers`
   - 订阅 `/Scepter/worldCoord/world_coord`
   - 订阅 `/Scepter/worldCoord/raw_world_coord`

5. 扩展 `TieRobotFrontApp`：
   - 装配 `Scene3DView`
   - 装配 `TopicLayerController`
   - 将 TF、点云、绑扎点、规划点桥接到三维场景

6. 点云解码策略：
   - 解码 `32FC3 sensor_msgs/Image`
   - 采样为前端点集
   - 转换为米并写入 Three.js `Points`

7. 视角与模型：
   - 建立简化机器模型
   - 实现 `相机视角 / 全局视角 / 跟随相机`

8. 更新帮助站中的前端入口说明和 3D 面板说明。

9. 跑验证：
   - 结构测试
   - `node --check`
   - `npm run build`
   - `npm run build` for help
   - `catkin_make`
