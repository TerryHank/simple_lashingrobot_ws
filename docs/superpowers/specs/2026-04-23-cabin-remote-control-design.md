# 设置卡片索驱遥控页设计

## 目标

- 在 `设置` 卡片中新增 `索驱遥控` 页。
- 允许用户通过前端方向按钮和全局键盘快捷键遥控索驱。
- 提供两个可调参数：
  - 单次点击移动步距（mm）
  - 全局移动速度
- 实时显示当前索驱 `x / y / z` 坐标。
- 支持方向按钮按住后按固定节拍连续步进，松开立即停止。
- 方向移动统一复用现有 `/cabin/single_move` 服务。
- 中心停止按钮调用真实 `/cabin/motion/stop`，只停止当前运动，不关闭索驱驱动。
- `索驱遥控` 页里的速度输入框成为全局索驱速度源；遥控、固定点移动、固定扫描和执行层索驱运动都统一使用它。

## 交互方案

### 1. 页面入口

- 在 `设置` 页签下新增一个 `索驱遥控` 选项。
- 页面内展示：
  - `开启键盘遥控` 勾选框
  - `单次点击步距（mm）` 输入框
  - `全局移动速度` 输入框
  - `当前索驱坐标` 信息块
  - 6 个方向按钮和 1 个停止按钮：
    - 顶行：`Z+ / X+ / Z-`
    - 中行：`Y+ / 索驱停止移动 / Y-`
    - 底行：`X-`
  - 一块状态说明区域，用于显示快捷键映射、启用状态和最近一次动作结果

### 2. 键盘映射

- 勾选 `开启键盘遥控` 后，在整个页面全局生效。
- 映射关系固定为：
  - `Q` → `Z+`
  - `W` → `X+`
  - `E` → `Z-`
  - `A` → `Y+`
  - `S` → `X-`
  - `D` → `Y-`
- 键盘和页面按钮走同一条前端逻辑，参数完全一致。
- 键盘仍保持单次步进，不启用按住连续连发，避免浏览器重复按键导致节拍失控。

### 3. 防误触

- 当焦点位于 `input / textarea / select / contenteditable` 区域时，键盘遥控不触发。
- 忽略带修饰键的组合输入（`Ctrl / Meta / Alt`）。
- 忽略浏览器长按产生的 `repeat` 事件，保持单次按下只触发一次步进。

### 4. 按钮长按连续步进

- 方向按钮支持 `pointerdown` 长按。
- 按下后立即执行 1 次当前方向步进。
- 随后以前端固定间隔 `250 ms` 持续连发。
- 松开、移出按钮、窗口失焦、页面切后台时立即停止连发。
- 连发过程中只有上一条 `/cabin/single_move` 返回后，下一条节拍才允许继续发送；若上一条仍在执行，则跳过该节拍，不堆积请求。

## 坐标与服务调用

### 1. 当前位置来源

- `/cabin/single_move` 服务收的是绝对坐标，而不是增量。
- 当前工程已经通过 TF 发布 `map -> Scepter_depth_frame`，其平移量就是索驱当前 `x / y / z`。
- 前端通过 `Scene3DView` 读取当前 `Scepter_depth_frame` 在 `map` 下的世界坐标，并转换为毫米。
- 该坐标既作为遥控前的计算基准，也作为遥控页 `当前索驱坐标` 的实时显示来源。

### 2. 单步移动计算

- 每次遥控前先读取当前索驱位置。
- 按方向将对应轴加减 `step`，生成新的绝对目标点。
- 以 `索驱遥控` 页里的 `全局移动速度` 作为本次请求速度。
- 如果当前 TF 尚未就绪，则拒绝下发，并提示“暂未拿到索驱当前位置”。

### 3. 全局速度同步

- 前端在 `索驱遥控` 页速度值变化时，通过 `/web/cabin/set_cabin_speed` 立即同步后端 `global_cabin_speed`。
- ROS 重新连接后，前端会再次把当前设置页速度推送给后端，避免驱动节点重启后回退到默认值。
- 后端任务链中的扫描建图、直接执行账本测试、执行层和当前区域直执行，不再优先读取 `pseudo_slam_bind_path.json` 里的 `cabin_speed`，而是统一使用当前 `global_cabin_speed`。
- “移动到索驱位置”固定点按钮也不再带固定速度常量，而是读取当前 `全局移动速度`。

### 4. 前端兜底

- 每次 `/cabin/single_move` 成功后，前端缓存最近一次目标位姿。
- 若短时间内 TF 尚未刷新，可用最近一次成功目标作为临时兜底，避免 UI 状态与用户操作脱节。
- 当 TF 暂时缺失时，坐标区显示“等待索驱 TF…”，但若有最近一次成功目标，遥控计算仍可用该缓存兜底。

## 前端职责划分

### 1. `UIController`

- 渲染 `索驱遥控` 页。
- 提供遥控参数读取方法。
- 绑定页面方向按钮按压事件。
- 提供遥控状态展示、当前位置展示、按钮禁用状态和按钮激活态更新。

### 2. `TieRobotFrontApp`

- 负责全局 `keydown` 监听。
- 负责按钮长按连发、节拍调度和停止条件。
- 负责把 `索驱遥控` 页速度同步到 `/web/cabin/set_cabin_speed`。
- 读取 UI 参数并调用遥控控制器。
- 在 ROS 连接状态变化时更新遥控按钮可用性。
- 在 TF 和移动结果返回后刷新当前索驱坐标显示。

### 3. `CabinRemoteController`

- 封装方向到坐标增量的映射。
- 负责读取当前位置、计算目标点、调用 `/cabin/single_move`。
- 对外统一返回成功 / 失败结果，供页面日志与状态区复用。

### 4. `Scene3DView`

- 暴露当前索驱位置读取接口，基于 `Scepter_depth_frame` 的 TF 结果返回毫米坐标。

### 5. 后端索驱流程

- `suoquNode` 继续通过 `/web/cabin/set_cabin_speed` 维护 `global_cabin_speed`。
- 新增 `/cabin/motion/stop` 服务，服务内部调用底层停止当前运动的方法，只停止当前索驱运动。
- 所有任务态索驱移动统一使用当前全局速度，避免旧扫描账本或旧配置文件里的速度值覆盖设置页。

## 验证范围

- 结构测试覆盖：
  - `设置` 页签新增 `索驱遥控`
  - 遥控页的勾选框、输入框和方向按钮存在
  - 遥控页存在当前索驱坐标显示区
  - 长按按钮连发相关方法与停止监听存在
  - 全局键盘监听逻辑与按键映射存在
  - 前端通过 TF 读取当前索驱位置
  - 按钮与键盘最终都走 `/cabin/single_move`
  - 中心停止按钮最终调用 `/cabin/motion/stop`
  - `索驱遥控` 页速度会同步到 `/web/cabin/set_cabin_speed`
  - 后端任务链索驱移动统一读取 `global_cabin_speed`
- 构建验证：
  - `python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py`
  - `npm --prefix src/tie_robot_web/frontend run build`

## 影响范围

- `src/tie_robot_web/frontend/src/ui/UIController.js`
- `src/tie_robot_web/frontend/src/styles/app.css`
- `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- `src/tie_robot_web/frontend/src/views/Scene3DView.js`
- `src/tie_robot_web/frontend/src/controllers/CabinRemoteController.js`（新增）
- `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- `src/tie_robot_process/src/suoqu/service_orchestration.cpp`
- `src/tie_robot_process/src/suoqu/suoqu_runtime_internal.hpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_web/test/test_workspace_picker_web.py`
