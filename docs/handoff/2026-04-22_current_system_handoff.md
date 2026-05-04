# 2026-04-22 当前系统交接文档

## 1. 文档目的

这份文档用于把当前工程的核心数据链、状态机、入口按钮、落盘工件、最近做过的结构性改动，以及仍需继续追的风险点，完整交接给下一位工程师。

重点不是“历史上改过什么”，而是：

- 现在系统 **实际上怎么跑**
- 哪些链路 **已经统一**
- 哪些地方 **代码已经改了，但还需要现场重启/复测**
- 下一位工程师从哪里接手最省时间

---

## 2. 当前工程边界

### 2.1 已移除的模块

本轮已经从工程中彻底移除以下模块及其构建依赖：

- `src/fast_image_solve`
- `src/livox_ros_driver2`
- `simulated_annealing` 相关头文件、包含和链接逻辑

当前 `chassis_ctrl` 已经独立承接原本依赖 `fast_image_solve` 的消息和服务定义。

### 2.2 已迁移到 `chassis_ctrl` 的消息/服务

当前由 `chassis_ctrl` 自己提供：

- `msg/PointCoords.msg`
- `msg/PointsArray.msg`
- `srv/ProcessImage.srv`
- `srv/PlaneDetection.srv`

其中 `ProcessImage.srv` 当前模式定义如下：

- `MODE_DEFAULT = 0`
- `MODE_ADAPTIVE_HEIGHT = 1`
- `MODE_BIND_CHECK = 2`
- `MODE_SCAN_ONLY = 3`
- `MODE_EXECUTION_REFINE = 4`

---

## 3. 启动拓扑

主启动入口：

- `roslaunch chassis_ctrl run.launch`

当前 `run.launch` 会启动：

- `suoquNode`
- `moduanNode`
- `gripper_tf_broadcaster.py`
- `pointAI.py`
- `stable_point_tf_broadcaster.py`
- `workspace_picker_web_server.py`
- `workspace_picker_web_open.py`
- 可选包含 `api.launch`

### 3.1 前端入口

独立工作区前端页面：

- `src/ir_workspace_picker_web/index.html`

默认通过本地静态服务暴露，默认端口：

- `http://127.0.0.1:8765/index.html`

如果端口冲突，`workspace_picker_web_server.py` 会自动顺延到可用端口。

---

## 4. 当前前端按钮与动作

前端文件：

- `src/ir_workspace_picker_web/index.html`
- `src/ir_workspace_picker_web/ir_workspace_picker.mjs`

当前页面主要按钮及其含义如下。

### 4.1 `提交四边形并触发 S2`

动作：

1. 将 IR 图上当前选中的 4 个角点发布到：
   - `/web/pointAI/set_workspace_quad`
2. 随后触发：
   - `/web/pointAI/run_workspace_s2`

作用：

- 保存手动工作区四边形
- 立即执行一次手动工作区 `S2`

### 4.2 `直接识别绑扎点`

动作：

- 不重新点 4 个角点
- 直接复用当前已保存工作区
- 触发：
  - `/web/pointAI/run_workspace_s2`

作用：

- 用当前保存的工作区直接重新跑一次 `S2`

### 4.3 `移动到固定扫描位姿并扫描规划`

动作：

- 前端发：
  - `/web/cabin/start_pseudo_slam_scan`
- 当前约定值：
  - `data = 5.0`

后端含义：

- 固定工作区单次扫描
- 先移动到固定扫描位姿
- 再触发扫描识别
- 扫描结束后做动态规划并写盘

### 4.4 `开始执行层`

动作：

1. 先发：
   - `/web/cabin/set_execution_mode`
   - 当前值表示 `live_visual`
2. 再发：
   - `/web/cabin/start_global_work`

作用：

- 读取当前动态规划账本
- 进入执行层

### 4.5 `清记忆并开始执行层`

与上一个按钮相同，但会携带：

- `clear_execution_memory = true`

作用：

- 清理执行记忆后重新开始全局执行

---

## 5. 工作区数据链

### 5.1 用户交互

用户在 IR 图上点 4 个角点。

页面支持：

- 点击选点
- 拖拽修正已选点
- 回显当前已保存工作区

### 5.2 后端保存格式

工作区保存到：

- `src/chassis_ctrl/data/manual_workspace_quad.json`

当前保存的是：

- `corner_pixels`
- `corner_world_map`
- 可选 `corner_sample_pixels`

也就是说：

- 交互发生在 2D 图像上
- 实际保存的是 `map` 下的世界坐标四边形

### 5.3 当前工作区回显话题

回显话题：

- `/pointAI/manual_workspace_quad_pixels`

前端页面打开后会自动订阅并叠加蓝色虚线框。

---

## 6. 扫描层数据链

### 6.1 扫描策略

扫描服务：

- `/cabin/start_pseudo_slam_scan`
- `/cabin/start_pseudo_slam_scan_with_options`

对应服务定义：

- `src/chassis_ctrl/srv/StartPseudoSlamScan.srv`

当前扫描策略枚举：

- `SCAN_STRATEGY_SINGLE_CENTER = 0`
- `SCAN_STRATEGY_MULTI_POSE = 1`
- `SCAN_STRATEGY_FIXED_MANUAL_WORKSPACE = 2`

前端按钮“移动到固定扫描位姿并扫描规划”当前走的是：

- `SCAN_STRATEGY_FIXED_MANUAL_WORKSPACE`

### 6.2 固定扫描位姿

当前固定扫描位姿不是动态算的，写死在后端逻辑中：

- `x = -260`
- `y = 1700`
- `z = 2997`
- `speed = 100`

这个位姿只用于扫描层，不是执行原点。

### 6.3 扫描层视觉算法

扫描层当前使用：

- 手动工作区 `S2`
- 也就是“频域周期检测 + 相位回归”的这条扫描识别链

对应主逻辑集中在：

- `src/chassis_ctrl/scripts/pointAI.py`

关键点：

- `S2` 只应该在扫描层使用
- 当前代码已经明确拆成两种视觉模式：
  - 扫描层使用 `MODE_SCAN_ONLY = 3`
  - 执行层局部视觉使用 `MODE_EXECUTION_REFINE = 4`
- 执行层不再复用 `scan_only` 的兼容语义

### 6.4 扫描产物

扫描后会写两份关键文件：

- `src/chassis_ctrl/data/pseudo_slam_points.json`
- `src/chassis_ctrl/data/pseudo_slam_bind_path.json`

当前文件角色：

- `pseudo_slam_points.json`
  - 保存扫描层全局识别点
  - 当前主数组字段名为：`pseudo_slam_points`
- `pseudo_slam_bind_path.json`
  - 保存基于全局点做出的动态规划结果
  - 包含：
    - `path_origin`
      - 当前口径应与蛇形排序后的首个真实执行区域 `cabin_pose.x/y` 对齐
    - `areas[]`
    - `areas[].cabin_pose`
    - `areas[].groups[]`
    - `groups[].points[]`

另外还有执行记忆：

- `src/chassis_ctrl/data/bind_execution_memory.json`

---

## 7. 动态规划当前实现

### 7.1 当前实现不是“全局窗口覆盖式规划”

这一点必须明确告诉接手工程师：

**当前动态规划实现仍然是种子点驱动的启发式规划，不是用户后来希望的“从全局真实点直接做窗口覆盖规划”。**

核心函数在：

- `collect_dynamic_bind_seed_world_points(...)`
- `build_dynamic_bind_candidate_pose_from_world_point(...)`
- `build_dynamic_bind_area_entries_from_scan_world(...)`

文件位置：

- `src/chassis_ctrl/src/suoquNode.cpp`

### 7.2 当前实现思路

简化描述如下：

1. 从未完成规划点里选种子点
2. 围绕种子点收邻域候选
3. 反推出一个候选 `cabin_pose`
4. 看该位姿下能覆盖多少点
5. 生成区域和组

这套方法的优点：

- 实现快
- 局部成团时能出结果

缺点：

- 强依赖种子点选择
- 不是用户期望的“全局真实点 -> 直接规划窗口”
- 后续如果要做得更稳，建议重构成：
  - 基于 TCP 工作空间窗口的直接覆盖规划

### 7.3 当前模板约束

用户确认过的规则仍然保留：

- 每组固定按 `4` 点模板规划
- 若真实未完成点不足 `4` 个，允许补齐模板
- 但真正执行时只对未完成点执行

---

## 8. 执行层数据链

### 8.1 执行入口

执行层当前主入口：

- `/web/cabin/start_global_work`

执行模式切换：

- `/web/cabin/set_execution_mode`

当前主要模式：

- `live_visual`

### 8.2 执行层读取的账本

执行层不直接吃前端 `S2` 显示点，也不直接吃前端页面状态。

执行层真正读取的是：

- `src/chassis_ctrl/data/pseudo_slam_bind_path.json`

也就是说：

- 扫描层决定全局点
- 动态规划决定区域路径
- 执行层只按账本跑

### 8.3 执行层当前顺序

当前设计意图是：

1. 点击“开始执行层”
2. 读取 `pseudo_slam_bind_path.json`
3. 先回 `path_origin`
   - `path_origin` 现在应理解为“执行起点”，不是随意拼出的包围盒左下角虚拟点
4. 再按蛇形区域顺序逐区执行
5. 每到一个区域：
   - 先移动索驱
   - 再做局部视觉微调
   - 再发给线性模组执行

### 8.4 当前执行层视觉原则

目标原则已经明确：

- 扫描层执行 `S2`
- 执行层 **不执行 `S2`**
- 执行层保留原来的局部视觉微调策略；2026-04-30 起明确为平面分割 + Hough
- 微调参考扫描层给出的全局点

当前代码已经按这个方向收口：

- `MODE_EXECUTION_REFINE = 4`
- `run_live_visual_global_work(...)` 改为使用执行微调模式
- `pointAI` 已按 `request_mode` 直接分流扫描层 `S2 / PR-FPRG` 和执行层平面分割 + Hough 局部视觉
- 已移除执行层借 `scan_only` 显示/参数开关复用的兼容路径

但这里要特别提醒：

**这条链虽然模式已经明确拆开，但仍然必须在现场重启相关节点后再完整复测一次，确认执行层运行时确实不再落回扫描层 `S2`。**

### 8.5 下游线性模组的坐标口径

这一条是关键交接项：

> 发给下游“线性模组”的点，必须是 TCP / 虎口坐标系下的局部坐标。

当前链路里，这条原则是成立的：

- 全局点会先转到 `gripper_frame`
- 执行前还会做 TCP 工作空间过滤
- 当前虎口 / TCP 局部坐标执行限位为：
  - `x: 0~360 mm`
  - `y: 0~320 mm`
  - `z: 0~140 mm`
- 真正发给下游的是局部点

也就是说：

- 下游执行坐标不应是索驱全局坐标
- 也不应是扫描层全局点坐标
- 必须是 TCP 局部点

---

## 9. 当前状态机（推荐理解方式）

这里不按代码变量名讲，而按运行阶段讲。

### 状态 0：系统启动

入口：

- `run.launch`

目标：

- 拉起 ROS 节点
- 拉起工作区前端

### 状态 1：工作区待确认

入口：

- 打开 IR 选点前端

目标：

- 读取当前 IR 图
- 回显已保存工作区
- 允许重新点 4 个角点

### 状态 2：工作区已保存

触发：

- 点击 `提交四边形并触发 S2`

产物：

- `manual_workspace_quad.json`
- `/pointAI/manual_workspace_quad_pixels`

### 状态 3：扫描规划中

触发：

- 点击 `移动到固定扫描位姿并扫描规划`

流程：

1. 索驱移动到固定扫描位姿
2. 扫描层触发 `S2`
3. 生成全局点
4. 生成动态规划账本

产物：

- `pseudo_slam_points.json`
- `pseudo_slam_bind_path.json`

### 状态 4：执行待启动

条件：

- 动态规划账本已生成

动作：

- 可点击 `开始执行层`

### 状态 5：执行中

触发：

- `开始执行层`

流程：

1. 回 `path_origin`
2. 按蛇形区域顺序逐区执行
3. 到区后做局部视觉微调
4. 发下游线性模组执行

### 状态 6：执行记忆控制

触发：

- `清记忆并开始执行层`

作用：

- 清除 `bind_execution_memory.json`
- 重新从第一批区域开始执行

---

## 10. 当前关键工件说明

### 10.1 `manual_workspace_quad.json`

用途：

- 当前手动工作区定义

当前角色：

- 扫描层 `S2` 的唯一工作区约束来源

### 10.2 `pseudo_slam_points.json`

用途：

- 保存扫描后全局识别点

注意：

- 当前数组主键名是 `pseudo_slam_points`
- 不是 `points`

### 10.3 `pseudo_slam_bind_path.json`

用途：

- 保存动态规划后的执行区域路径

包含：

- `path_origin`
- 当前应与 `areas[0].cabin_pose.x/y` 对齐，避免先回虚拟角点后再大跨度跳首区
- `areas[]`
- 每个区域的 `cabin_pose`
- 每组 `4` 点模板
- `groups[].points[].world_x/world_y/world_z`
- 记录索驱 / cabin 绝对坐标系下的全局点
- `groups[].points[].x/y/z`
- 记录该点在当前区域 `cabin_pose` 下反算出的 TCP / 虎口局部坐标

### 10.4 `bind_execution_memory.json`

用途：

- 执行去重 / 记忆

典型字段：

- `executed_points`
- `path_origin`
- `path_signature`
- `scan_session_id`

---

## 11. 当前已经明确的风险点

这一节非常重要，下一位工程师应优先看。

### 11.1 动态规划仍是种子点驱动

这不是最终理想方案。

如果后续发现：

- 区域位姿不稳定
- 第一组区域不合理
- 分组依赖“第一颗种子”太重

优先考虑重构为：

- 基于 TCP 窗口的全局覆盖式规划

### 11.2 执行层与扫描层的视觉模式刚刚拆分

代码已经往正确方向改：

- 扫描层走 `S2`
- 执行层走原局部视觉微调

但这类改动必须现场重启并完整复测：

- `pointAINode`
- `suoquNode`

否则很容易继续看到“执行层还在跑扫描层算法”的旧现象。

### 11.3 执行显示层和真实下游执行层是两条链

目前工程里要严防一种误判：

- 图上显示错了 ≠ 下游执行点真的错了

必须分清：

- `result_image_raw` 的显示层
- 线性模组收到的 TCP 局部执行点

### 11.4 坐标链仍是高风险区域

尤其是：

- 相机坐标
- `map`
- `gripper_frame`
- TCP / 虎口局部坐标

这条链过去已经出现过：

- 全局点 `z` 明显异常
- 动态规划区域 `cabin_pose.z` 不合理
- 执行图显示世界坐标而不是 TCP 局部坐标

这部分建议下一位工程师继续重点盯。

### 11.5 当前账本文件可能是“最新代码 + 旧运行结果”的混合状态

交接时要特别提醒：

- 代码改了，不代表磁盘上的 `pseudo_slam_points.json` / `pseudo_slam_bind_path.json` 自动更新
- 很多现象要重新跑一次扫描规划后才有意义

---

## 12. 推荐的接手检查顺序

下一位工程师接手后，建议不要直接从执行层往下撞，而按这个顺序检查。

### 第一步：确认节点版本生效

建议动作：

1. 重启 `run.launch`
2. 确认：
   - `pointAINode`
   - `suoquNode`
   - `topictransNode`
   - `workspace_picker_web_server`
   已经是当前代码版本

### 第二步：确认工作区链通

检查：

- 前端点 4 个点后能否写入 `manual_workspace_quad.json`
- 前端重开后蓝色虚线是否回显

### 第三步：确认扫描层链通

点击：

- `移动到固定扫描位姿并扫描规划`

然后检查：

- `pseudo_slam_points.json`
- `pseudo_slam_bind_path.json`

重点看：

- `scan_session_id`
- `pseudo_slam_points` 数量
- `areas` 数量
- `groups[].group_type`
- `path_origin`

### 第四步：确认执行层不再跑 `S2`

检查：

- 执行层是否只做局部视觉微调
- 执行图是否显示 TCP 局部点
- 执行时间是否不再被整幅 `S2` 卡住

### 第五步：确认下游仍然吃 TCP 局部坐标

必要时抓：

- 发给线性模组的最终点

不要只看显示图。

---

## 13. 建议下一位工程师优先做的 3 件事

### 优先级 1

把“执行层不跑 `S2`”做现场闭环验证。

也就是：

- 启动系统
- 扫描建图
- 开始执行
- 确认执行时不再跑整幅手动工作区 `S2`

### 优先级 2

重新审视动态规划算法，决定是否从“种子点驱动”重构到“TCP 窗口覆盖驱动”。

这是当前规划质量的最大结构性风险。

### 优先级 3

把坐标链做成一份可视化调试清单：

- 相机原始点
- `map` 全局点
- `gripper_frame` / TCP 局部点
- 最终下游执行点

否则后续一旦再出现“显示对不上、执行范围超界、z 口径异常”，排查成本会很高。

---

## 14. 本次交接时的验证结论

本次文档落盘前，已确认以下事实：

- `fast_image_solve`、`livox_ros_driver2`、`simulated_annealing` 已从源码和构建链中移除
- `chassis_ctrl` 可以独立编译
- 相关前端和后端脚本已切到 `/web/pointAI/...` 命名
- 交接文档所述的文件路径和当前工件路径存在

但也要明确：

**涉及执行层现场行为的那些新改动，仍应在完整重启系统后做一次现场闭环验证。**

这份文档写的是“当前代码口径 + 当前账本角色 + 当前已知风险”，不是替代现场复测。
