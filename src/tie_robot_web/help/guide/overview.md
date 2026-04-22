# 工程总览

当前工作空间主包如下：

- `tie_robot_msgs`：全局消息、服务与动作接口
- `tie_robot_hw`：相机、索驱、线性模组驱动与 SDK 接入
- `tie_robot_perception`：视觉节点与识别算法
- `tie_robot_control`：线性模组控制
- `tie_robot_process`：扫描、账本、执行流程
- `tie_robot_web`：前端、桥接和帮助站
- `tie_robot_bringup`：launch 与系统装配

## 分层口径

- `tie_robot_msgs` 只放全局 `msg / srv / action`
- `tie_robot_hw` 只负责硬件连接、协议和 SDK
- `tie_robot_perception` 负责相机节点、视觉算法和世界点处理
- `tie_robot_control` 负责线性模组执行
- `tie_robot_process` 负责扫描、账本、执行层流程编排
- `tie_robot_web` 负责新前端、桥接和帮助站
- `tie_robot_bringup` 负责启动装配

## 这轮结构化重点

- 巨型入口文件被拆成“薄入口 + 模块实现”
- 前端主入口改成 `frontend/` 下的 `app / controllers / ui / views / config / utils` 分层
- `tie_robot_web/web/index.html` 现在由 Vite 从 `frontend/` 构建生成，并继续保留 `web/help`
- 新前端布局已经改成 `robot_viewer` 风格：整页三维背景 + 顶部工具条 + 四周浮动面板
- `3D Scene` 不再只是一个独立面板，而是整个页面的背景层
- 顶部工具条负责面板显隐、视角快捷切换和帮助入口
- `Topic Layers` 会按当前相机模态、TF 和可视话题切换 `只显示点云 / 只显示绑扎点 / 点云+绑扎点 / 规划点 / 只显示机器 / 全开`
- 识别到的绑扎点直接显示 `/coordinate_point` 中的全局世界坐标
- 点云图层复用 `/Scepter/worldCoord/world_coord` 与 `/Scepter/worldCoord/raw_world_coord`，并按 `Scepter_depth_frame` 当前 TF 投到全局场景
- `moduanNode.cpp` 收成 `moduan/` 目录
- `pointAI.py` 收成 `pointai/` Python 包
- `topics_transfer.cpp` 收成 `web_bridge/` 目录
- `dynamic_bind_planning.cpp` 收成 `planning/` 多文件
- `suoquNode.cpp` 已拆出 `cabin_transport.cpp`、`pseudo_slam_markers.cpp`、`pseudo_slam_scan_processing.cpp`、`service_orchestration.cpp`
- 预计算账本与执行主体已继续拆成 `area_execution.cpp`、`execution_memory_store.cpp`、`bind_path_store.cpp`
