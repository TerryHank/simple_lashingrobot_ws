# robot_viewer 风格新前端设计

## 背景

当前工程存在两套前端能力来源：

- `src/tie_robot_web/web/`：现行 IR 工作区选点与执行入口
- `src/APP/dist/`：老前端打包产物，仍可观察到一批操作按钮、状态灯和日志面板口径

用户希望参考 `robot_viewer/` 的工程化前端结构，把这两套能力迁成一个新的前端，但目标不是 URDF 预览，而是新的机器人操作控制台。

## 设计目标

1. 前端结构采用 `robot_viewer` 风格的分层：
   - `app/`：应用装配入口
   - `controllers/`：ROS、任务动作、老操作口径控制器
   - `ui/`：全局 UI 与浮动面板管理
   - `views/`：IR 画布与业务面板视图
   - `config/`：旧前端命令目录、状态目录
   - `utils/`：IR 图像增强、存储与通用工具
2. 新前端继续服务于 `tie_robot_web/web/index.html`
3. 当前 `tie_robot_web` 的功能必须保留：
   - IR 图像显示与增强
   - 4 点工作区选取/拖拽
   - S2 触发与结果覆盖
   - 扫描规划 Action
   - 执行层 Action
   - 账本直执行 Action
4. 参考老 `APP` 打包产物中仍能识别的能力，补入新前端：
   - 操作按钮目录
   - 参数输入区
   - 设备状态灯
   - 日志面板
5. 本轮不做的事：
   - 不复刻老 `APP` 的地图/SLAM 画布细节
   - 不试图逆向恢复 `APP` 内部 React 组件结构
   - 不改 ROS 后端协议口径

## 迁移边界

### 当前前端保留并迁移

- ROS 连接逻辑
- IR 图像与 result 图层订阅
- 工作区点选四边形
- Action / Service 触发链
- 帮助站入口

### 老 APP 迁移方式

由于 `src/APP` 当前只有打包产物，没有源码，因此本轮按“可观察行为”迁移：

- 解析得到的命令目录直接整理为配置表
- 参数区按旧行为兼容：
  - 全局路径规划参数
  - 固定点/调试参数
  - 速度/高度参数
- 状态灯按旧订阅话题兼容：
  - `/robot/chassis_status`
  - `/robot/moduan_status`
  - `/robot/binding_gun_status`

## 新前端结构

```text
src/tie_robot_web/frontend/
├── index.html
├── package.json
├── vite.config.js
└── src/
    ├── main.js
    ├── app/
    │   └── TieRobotFrontApp.js
    ├── controllers/
    │   ├── RosConnectionController.js
    │   ├── TaskActionController.js
    │   ├── LegacyCommandController.js
    │   └── StatusMonitorController.js
    ├── ui/
    │   ├── UIController.js
    │   └── PanelManager.js
    ├── views/
    │   └── WorkspaceCanvasView.js
    ├── config/
    │   ├── legacyCommandCatalog.js
    │   └── statusMonitorCatalog.js
    ├── utils/
    │   ├── irImageUtils.js
    │   ├── rosbridge.js
    │   └── storage.js
    ├── vendor/
    │   └── roslib.js
    └── styles/
        └── app.css
```

## 页面布局

采用 `robot_viewer` 风格的浮动面板而不是旧的左右硬切栏：

- 中央主视图：IR 画布 + S2/执行层覆盖
- 左上：连接与任务面板
- 左下：工作区点位与显示增强
- 右上：状态灯与日志
- 右下：老前端操作与参数调试面板

## 构建与发布

- 新前端使用 Vite 构建
- 输出目录仍为 `src/tie_robot_web/web`
- `emptyOutDir=false`，保留既有 `help/`
- `workspace_picker_web_server.py` 继续服务 `web/index.html`

## 验证标准

1. 新前端源目录与控制器/视图骨架存在
2. `npm run build` 能生成新的 `web/index.html`
3. 结构测试能识别新脚手架
4. 旧帮助站 `help/` 不被覆盖
5. 新前端首页可见：
   - IR 画布
   - 任务按钮
   - 老操作按钮
   - 状态灯
   - 日志面板
