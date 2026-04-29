# 巨型业务代码结构化与帮助站建设设计

## 背景

当前工程已经完成了按职责分包的第一轮整理，主包结构基本稳定：

- `tie_robot_hw`
- `tie_robot_control`
- `tie_robot_perception`
- `tie_robot_process`
- `tie_robot_web`
- `tie_robot_msgs`
- `tie_robot_bringup`
- `tie_robot_description`

但包结构稳定之后，包内仍然保留了多处巨型业务文件。这些文件虽然已经落进了正确的包边界里，但在文件级别仍然存在“巨型类、巨型节点、巨型脚本、巨型回调汇总”的问题，导致：

- 单文件职责过多，阅读和维护成本很高。
- 头文件、常量、协议地址、工具函数与业务编排混杂在一起。
- 测试难以精确覆盖，只能做粗粒度回归。
- 新成员很难从文件结构上理解工程。

同时，当前新前端 `tie_robot_web` 还没有一套能直接面向现场和开发者的帮助站，用于说明工程结构、主包职责、动作链和主要文件职责。

## 本次范围

本次范围明确包含：

- 对 `src/` 下所有巨型**业务**文件进行结构化拆分。
- 将新前端帮助文档站点放入 `tie_robot_web`，并作为当前工程的帮助入口。

本次范围明确排除：

- `/home/hyq-/simple_lashingrobot_ws/src/APP`
- 第三方库、厂商 SDK 头文件、`json.hpp`
- 生成产物、构建目录、依赖缓存

## 当前巨型业务文件盘点

按当前工作区统计，`src/` 下需要纳入本次的一期巨型业务文件包括：

- [tie_robot_process/src/suoquNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/src/suoquNode.cpp:1)，约 `8981` 行
- [tie_robot_perception/scripts/pointAI.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/scripts/pointAI.py:1)，约 `3100` 行
- [tie_robot_control/src/moduanNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_control/src/moduanNode.cpp:1)，约 `1882` 行
- [tie_robot_perception/src/camera/scepter_manager.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/camera/scepter_manager.cpp:1)，约 `922` 行
- [tie_robot_process/src/planning/dynamic_bind_planning.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/src/planning/dynamic_bind_planning.cpp:1)，约 `836` 行
- [tie_robot_web/web/ir_workspace_picker.mjs](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/web/ir_workspace_picker.mjs:1)，约 `835` 行
- [tie_robot_web/src/topics_transfer.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/src/topics_transfer.cpp:1)，约 `779` 行

这些文件都属于当前动作链或新前端主链上的业务代码，因此应当进入一期纯等价结构化范围。

## 问题定义

### 1. 文件职责过载

以 [moduanNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_control/src/moduanNode.cpp:1) 为例，单文件同时承载：

- PLC 地址常量
- 模组状态与全局变量
- Modbus 读写
- 点位执行
- 错误处理
- ROS topic / service 回调
- `main(...)`

以 [pointAI.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/scripts/pointAI.py:1) 为例，单类同时承载：

- 相机订阅与缓存
- TF 查询
- 工作区几何
- `worldCoord` 访问
- 候选点筛选
- 矩阵选点
- 绘图与标注
- service 入口
- 运行时配置读写

这种组织方式不利于继续工程化。

### 2. 结构虽分包，但文件层级还不工程化

现在的问题已经不再是“包边界错了”，而是“包内部仍像历史大仓脚本”。  
也就是说，本次核心目标不是再改包名，而是把包内文件结构真正做成工程化目录。

### 3. 缺少系统化帮助入口

当前工程缺少一套能说明以下内容的帮助站：

- 包结构与职责
- 主动作链与数据链
- 关键节点与关键文件
- 开发调试入口
- 重构后的模块边界

## 目标

本次一期目标如下：

1. 将所有巨型业务文件按当前包结构拆分为职责清晰的模块。
2. 保持现有动作链、话题、服务、动作和算法口径尽量不变。
3. 让入口文件变成“薄入口”，把具体职责沉到包内模块目录。
4. 为每个主包建立可读的内部目录层次。
5. 在 `tie_robot_web` 中新增 VitePress 帮助站，并挂到新前端帮助入口。

## 非目标

本次明确不做：

- 不进行接口语义大改。
- 不把现有 service / action / topic 再重新发明一遍。
- 不顺手做行为优化、算法改口径、状态机重写。
- 不触碰 `src/APP`。
- 不大规模替换 ROS 节点名、topic 名和服务名。

这些统一留到二期“行为与接口优化”阶段处理。

## 方案对比

### 方案 A：一次性大换血，边拆边改行为

优点：

- 一次可以清掉更多历史包袱。

缺点：

- 风险最高。
- 很难区分“结构变化导致的问题”和“行为变化导致的问题”。
- 对现场系统不友好。

### 方案 B：纯等价结构化重构，帮助站同步建设

做法：

- 先只做文件级职责拆分与目录结构整理。
- 保持对外行为、接口和算法口径基本不变。
- 同时补齐帮助站和工程地图。

优点：

- 风险最低。
- 最适合现场工程。
- 为二期行为优化提供稳定结构基础。

缺点：

- 历史接口包袱会先保留。
- 第一阶段看起来更像“整理地基”，而不是“换新系统”。

### 方案 C：只补帮助站，不拆巨型文件

优点：

- 成本低。

缺点：

- 只是给混乱结构加说明书，不能真正降低维护成本。

## 选型

采用方案 B。

原因：

- 它最符合用户提出的“先把大文件拆干净、帮助站补好、结构理顺；结构稳定后再做第二阶段”的节奏。
- 当前工程已经完成包级整理，现在最缺的是文件级工程化。

## 总体设计

本次按包内结构化来推进，而不是再新增新的总包。每个主包内部都遵循相同原则：

- 入口文件保留，但变薄。
- 常量、协议、回调、执行器、工具函数、渲染、配置分拆到独立模块。
- 新模块名按职责命名，而不是按“misc / utils / temp”命名。
- 任何单文件只承担一个清晰职责。

## 各包拆分设计

### `tie_robot_control`

#### 当前问题

[moduanNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_control/src/moduanNode.cpp:1) 是典型的“巨型硬件控制节点”。

#### 目标结构

建议新增以下目录：

- `include/tie_robot_control/moduan/`
- `src/moduan/`

建议拆分为：

- `register_map.hpp`
  - PLC 地址、寄存器定义、固定常量
- `runtime_state.hpp`
  - 全局状态结构体和运行时缓存
- `numeric_codec.cpp/.hpp`
  - 浮点与寄存器转换
- `error_handling.cpp/.hpp`
  - 错误消息、报警上报、停止策略
- `linear_module_executor.cpp/.hpp`
  - 绑扎点执行、单次点位发包、完成等待
- `moduan_ros_callbacks.cpp/.hpp`
  - ROS topic / service 回调
- `moduan_node_main.cpp`
  - `main(...)`，只负责初始化与装配

#### 设计原则

- `#define` 和地址映射不能继续堆在入口 `.cpp` 顶部，必须进入头文件。
- 和 Modbus/执行流程有关的逻辑要分层，而不是全靠全局变量串起来。

### `tie_robot_perception`

#### 当前问题

[pointAI.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/scripts/pointAI.py:1) 是一个典型巨型类，单类包含多种责任。

#### 目标结构

保留 `scripts/pointAI.py` 作为 ROS 入口，但新增包内模块目录：

- `src/tie_robot_perception/src/tie_robot_perception/pointai/`

建议拆分为：

- `runtime_config.py`
  - 运行时配置读写
- `image_buffers.py`
  - 图像缓存、订阅数据存取
- `tf_transform.py`
  - `Scepter_depth_frame`、`gripper_frame`、`map` 的坐标变换
- `world_coord.py`
  - `worldCoord` 采样和邻域补点
- `workspace_masks.py`
  - ROI、扫描区、手工工作区、travel range mask
- `matrix_selection.py`
  - 矩阵候选构造、排序、选点与过滤
- `rendering.py`
  - 结果图、文字标注、overlay
- `process_image_service.py`
  - `ProcessImage` 请求处理编排
- `processor.py`
  - 组合以上模块的主协调类

同时，[scepter_manager.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/camera/scepter_manager.cpp:1) 继续按“纯底层驱动”原则再做内部拆分，建议新增：

- `src/camera/frame_publish.cpp/.hpp`
- `src/camera/intrinsics.cpp/.hpp`
- `src/camera/device_session.cpp/.hpp`

#### 设计原则

- `pointAI.py` 最终应成为一个薄入口，内部只做节点初始化和对象装配。
- 视觉算法模块要按“输入/变换/筛选/渲染/服务”拆分，而不是按函数数量拆分。

### `tie_robot_process`

#### 当前问题

[suoquNode.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/src/suoquNode.cpp:1) 是当前工程里最大的业务入口，混合了：

- 路径账本
- 动态规划结果组织
- 索驱移动编排
- 执行模式分流
- 扫描与执行调度
- 恢复与状态持久化
- 各类 service 入口

同时 [dynamic_bind_planning.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/src/planning/dynamic_bind_planning.cpp:1) 本身也已接近新的巨型算法文件。

#### 目标结构

建议新增：

- `include/tie_robot_process/suoqu/`
- `src/suoqu/`

建议拆分为：

- `execution_mode.hpp`
  - 执行模式与公共枚举
- `scan_session_store.cpp/.hpp`
  - 扫描产物、会话信息、执行记忆读写
- `bind_path_store.cpp/.hpp`
  - `pseudo_slam_bind_path.json` 与相关账本读写
- `current_area_bind_runner.cpp/.hpp`
  - 当前区域执行逻辑
- `global_bind_runner.cpp/.hpp`
  - 全局执行主链
- `live_visual_runner.cpp/.hpp`
  - `live_visual` 分支
- `direct_bind_test_runner.cpp/.hpp`
  - “直接执行账本测试”支线
- `start_work_services.cpp/.hpp`
  - service 入口桥接
- `cabin_motion_helpers.cpp/.hpp`
  - 当前仍留在流程层的索驱动作辅助
- `node_main.cpp`
  - 入口装配

对 [dynamic_bind_planning.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_process/src/planning/dynamic_bind_planning.cpp:1) 再细分为：

- `coverability.cpp/.hpp`
- `grouping.cpp/.hpp`
- `ordering.cpp/.hpp`
- `path_origin.cpp/.hpp`

#### 设计原则

- 扫描、执行、账本、service 入口不能继续都堆在一个 `.cpp` 中。
- “主动作链编排”和“纯算法”必须分文件。

### `tie_robot_web`

#### 当前问题

[topics_transfer.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/src/topics_transfer.cpp:1) 把所有前端桥接回调堆在一起；[ir_workspace_picker.mjs](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/web/ir_workspace_picker.mjs:1) 把状态、ROS、Canvas、业务按钮逻辑也揉在一个文件里。

#### 目标结构

对于 C++ 桥接层：

- `src/web_bridge/`

建议拆分为：

- `service_clients.cpp/.hpp`
- `action_bridge.cpp/.hpp`
- `topic_callbacks.cpp/.hpp`
- `system_control.cpp/.hpp`
- `main.cpp`

对于前端：

- `web/modules/`

建议拆分为：

- `ros_connection.mjs`
- `action_clients.mjs`
- `canvas_renderer.mjs`
- `overlay_renderer.mjs`
- `workspace_selection.mjs`
- `execution_actions.mjs`
- `ui_state.mjs`
- `dom_refs.mjs`

保留 [ir_workspace_picker.mjs](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/web/ir_workspace_picker.mjs:1) 为入口装配文件。

## 帮助站设计

### 目标

使用 VitePress 生成一套静态帮助站，作为新前端 `tie_robot_web` 的帮助页入口。

### 目录设计

建议新增：

- `src/tie_robot_web/help/`
  - VitePress 源码目录
- `src/tie_robot_web/web/help/`
  - VitePress 构建输出目录，供现有静态服务器直接服务

### 内容设计

帮助站至少包含这些页面：

- 工程总览
- 主包职责说明
- 动作链 / 数据链
- 关键节点说明
- 文件结构树
- 重构后模块边界
- 开发调试入口

### 生成策略

帮助站不是纯手写文档，而是“手写说明 + 自动生成工程结构清单”结合：

- 手写：包职责、动作链、开发说明
- 自动生成：`src/` 文件树、模块索引、巨型文件拆分映射

建议增加一个生成脚本，根据当前 `src/` 目录扫描生成 Markdown 索引页，避免帮助站与代码结构持续漂移。

### 前端接入

在新前端首页增加明显的“帮助文档”入口，直接打开 `/help/`。

由于当前 [workspace_picker_web_server.py](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_web/scripts/workspace_picker_web_server.py:1) 已经直接服务 `tie_robot_web/web` 目录，因此 VitePress 构建产物放入 `web/help` 后即可被现有前端静态服务直接访问。

## 分阶段实施

### 一期：纯等价结构化与帮助站建设

目标：

- 拆巨型文件
- 建立模块目录
- 收头文件与常量
- 帮助站上线

约束：

- 不改现有对外行为口径
- 不主动重写状态机和接口语义

### 二期：接口和行为优化

在一期稳定之后，再做：

- topic / service / action 语义优化
- 状态机进一步清晰化
- 历史兼容逻辑清退
- 算法与流程进一步重构

## 成功标准

一期完成后，应满足：

1. 当前识别和执行主链可继续构建运行。
2. 所有巨型业务入口文件都被降到“薄入口”。
3. 常量、地址映射、工具函数、执行器、渲染器、桥接器都有独立文件归属。
4. 新增帮助站可通过 `tie_robot_web` 的静态服务访问。
5. 开发者不需要翻一个几千行文件，才能理解单个功能点。

## 测试与验证策略

本次需要增加的验证包括：

- 结构测试
  - 巨型文件是否已按预期拆分
  - 入口文件是否仅保留装配职责
- 构建测试
  - `catkin_make` 白名单主包构建通过
- 前端检查
  - JS 模块语法检查通过
  - 帮助站静态构建通过
- 文档一致性检查
  - 自动生成的文件结构页与当前 `src/` 树一致

## 风险与缓解

### 风险 1：一次重构面过大

缓解：

- 虽然范围覆盖所有巨型业务文件，但实现计划按包和阶段拆开执行。
- 优先做纯等价拆分，不改行为。

### 风险 2：拆分后链接和导入路径变乱

缓解：

- 每个包先建立模块目录和公共头，再做迁移。
- 入口文件始终保留，避免外部调用路径一次性大变。

### 风险 3：帮助站与代码结构脱节

缓解：

- 文件树和模块索引采用脚本自动生成。
- 手写说明只负责解释，不手工维护整个树。

## 落地原则

本次重构遵循 4 条原则：

1. 先结构化，再行为优化。
2. 先按职责拆分，再谈进一步抽象。
3. 入口保留，模块下沉。
4. 文档与工程结构同步生成，而不是事后补文档。
