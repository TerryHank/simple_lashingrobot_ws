# robot_viewer 风格新前端迁移计划

1. 先补结构测试，要求新的 `tie_robot_web/frontend` Vite 脚手架、控制器、视图和样式文件存在。

2. 创建新前端源目录：
   - `package.json`
   - `vite.config.js`
   - `index.html`
   - `src/main.js`

3. 落应用装配层：
   - `TieRobotFrontApp`
   - `UIController`
   - `PanelManager`

4. 落 ROS 与业务控制器：
   - `RosConnectionController`
   - `TaskActionController`
   - `LegacyCommandController`
   - `StatusMonitorController`

5. 迁移当前 `tie_robot_web` 的 IR 与执行主链功能：
   - IR 图像增强与绘制
   - 工作区点选与拖拽
   - S2/执行层结果图层
   - Action / Service 触发

6. 把老 `APP` 可识别的命令目录做成配置化面板：
   - 操作目录
   - 参数输入区
   - 状态灯
   - 日志面板

7. 用 Vite 构建到 `src/tie_robot_web/web`
   - 保留 `help/`
   - 让静态服务继续指向 `web/index.html`

8. 更新帮助站开发入口说明，并跑验证：
   - 结构测试
   - `npm run build`
   - `node --check`
   - 工作区全量 `catkin_make`
