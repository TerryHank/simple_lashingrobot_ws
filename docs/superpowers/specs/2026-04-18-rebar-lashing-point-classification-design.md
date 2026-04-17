# Rebar Lashing Point Classification Design

## Goal

在当前绑扎点识别链路上新增“已绑扎 / 未绑扎”分类能力，并把它做成可落地、可回放、可逐步演进的两阶段系统：

1. `绑前判定`
   - 在执行绑扎前，判断当前候选点是否已经绑过。
   - 避免重复绑扎。
   - 主依据是“当前走到该点时现场拍到的单帧局部证据”，而不是依赖历史图片。
   - 对低置信度样本保留 `不确定` 支路，不强行二选一。

2. `绑后复检`
   - 在执行绑扎后，对同一点再次采样。
   - 结合“本次执行前现拍的 `before`”和“执行后现拍的 `after`”做差分，判断这次绑扎是否成功。
   - 对低置信度样本同样保留 `不确定` 支路。

3. `先规则、后模型`
   - 第一阶段先上线：
     - 基于当前单帧 `IR + depth/height` 的绑前规则判定器
     - 基于本次 `before/after` 差分的绑后规则复检器
   - 第二阶段基于现场真实数据，把规则版逐步替换成轻量模型。

这项能力必须优先复用现有模态与现有点位链路，而不是重建一套独立视觉系统。

## Current Context

当前工程已经具备这些基础能力：

- `pointAI.py` 能输出稳定的绑扎候选点，携带：
  - `idx`
  - `Pix_coord`
  - `World_coord`
  - `Angle`
  - `is_shuiguan`
- `pointAI.py` 已经有围绕候选点裁局部小图的半成品入口 `save_image()`，会基于候选点中心裁局部 ROI，并叠加局部深度掩码。
- `ScepterROS` 正在持续发布：
  - `IR`
  - `depth`
  - `world_coord`
  - `raw_world_coord`
  - `CameraInfo`
- 当前 `PointCoords.msg` 里只有一个布尔位 `is_shuiguan` 可承载简单分类结果。

当前工程也存在这些结构性缺口：

- 绑扎点识别和“已绑扎 / 未绑扎”判定还没有真正接通，`is_shuiguan` 现在基本固定为 `false`。
- 没有正式的“点位证据包”落盘机制，无法系统积累单帧样本和 `before / after` 样本。
- 没有统一的绑前拦截、绑后复检状态机。
- 当前系统还没有 `不确定` 分支，容易把低质量样本硬判成二值结果。

## Locked Decisions

本设计锁定以下决策，避免实现阶段继续漂移：

- 分类目标固定为：`已绑扎 / 未绑扎`。
- 分类使用时机固定为：`绑前 + 绑后都要`。
- 绑前主依据固定为：当前走到该点时采到的单帧局部证据。
- 绑后主依据固定为：本次动作前后现拍的 `before / after` 差分。
- 前后采样视角允许有小偏差，但必须支持局部几何对齐。
- 风险偏好固定为：不追求强行二分类，允许 `不确定` 并进入复检或补拍支路。
- 落地路径固定为：`先规则版上线，再逐步让模型接管`。

## Required Behavior

### 1. 新增双阶段分类流程

每个候选绑扎点都需要支持两次分类：

- `pre_bind_classification`
  - 发生在绑扎动作前。
  - 主输入是当前走到该点时现拍的一份单帧证据包。
  - 不要求系统以前见过这个点，也不要求存在历史 `before` 图片。
  - 输出：
    - `unbound`
    - `bound`
    - `uncertain`

- `post_bind_verification`
  - 发生在绑扎动作后。
  - 主输入是同一轮动作中的 `before` 与 `after` 两份证据包。
  - 输出：
    - `success`
    - `failed`
    - `uncertain`

### 2. 低置信度样本不得强判

无论绑前还是绑后，若局部证据质量差、视角偏移过大、局部有效深度不足、或分数落在中间区间，都必须输出 `uncertain`。

实现上允许四种运行模式：

- `off`
  - 完全关闭分类能力。
- `shadow`
  - 只计算结果，不控制动作。
- `advisory`
  - 输出建议结果，但不阻断主流程。
- `blocking`
  - 绑前 `bound` 可直接拦截；绑后 `failed` 可触发失败处理。

### 3. 分类必须复用现有点位链路

分类器不能脱离现有候选点识别系统单独工作。它必须挂在稳定点输出之后，并以每个稳定点为单位采样局部证据。

换句话说，当前链路应保持：

`pointAI stable point -> evidence capture -> classification/verifier -> execution decision`

### 4. 证据来源必须明确区分绑前和绑后

为避免实现误解，这里锁定证据来源：

- `绑前判定`
  - 只要求机器人当前已经走到该点。
  - 系统在这个时刻现拍一张局部图，作为当前单帧证据。
  - 该判定不依赖“这个点以前是否拍过”。

- `绑后复检`
  - `before` 由系统在本次执行绑扎动作前现场采集。
  - `after` 由系统在本次执行绑扎动作后现场采集。
  - 复检比较的是“本次动作前后”的局部变化，不是比较很久以前的历史样本。

### 5. 必须保留与现有消息兼容的输出方式

考虑到当前 `PointCoords.msg` 只有 `bool is_shuiguan`，第一阶段锁定如下兼容策略：

- 对主下游仍继续填充 `PointCoords.is_shuiguan`
- 语义临时约定为：
  - `true`：已绑扎
  - `false`：未绑扎或未确定

同时，新增一条旁路调试/记录输出，用于保留完整状态。第一阶段锁定为新增 JSON Lines 调试日志文件：

- `src/chassis_ctrl/data/bind_classification_events.jsonl`

每条记录至少包含：

- `bind_state`
  - `unbound`
  - `bound`
  - `uncertain`
- `bind_score`
- `evidence_quality`
- `decision_reason`
- `phase`
  - `before`
  - `after`
- `result`
  - `success`
  - `failed`
  - `uncertain`

这样可以先不修改 `PointCoords.msg`，先把能力安全接上；后续如需更丰富联动，再单独升级消息定义。

## Architecture

### A. 新增点位证据采集层

在 `pointAI.py` 现有候选点裁剪入口附近新增标准化证据采集器。它不直接做动作控制，只负责把当前点变成一份可重复采样的证据包。

输入：

- 稳定点的 `idx`
- `Pix_coord`
- `World_coord`
- 当前机器人与 TF 状态
- 原始 `IR / depth / raw_world`

输出：

- `evidence_bundle`

### B. 规则版判定器

第一阶段新增独立的规则版判定器，职责分为：

- `pre_bind_rule_classifier`
- `post_bind_rule_verifier`

这两个模块都只吃证据包，不直接操作运动控制。

### C. 结果协调层

新增一个协调层，把分类结果变成业务可消费的决策：

- 绑前：
  - `bound` -> 跳过该点
  - `unbound` -> 允许执行
  - `uncertain` -> 补拍、影子记录，或交给上层模式决定

- 绑后：
  - `success` -> 正常记账
  - `failed` -> 告警或失败重试
  - `uncertain` -> 不直接记成功，按模式决定是否人工确认或自动补拍

### D. 数据沉淀层

所有点位证据、执行动作和判定结果都必须落盘，后续模型训练和规则调参都基于这一层。

模型版上线后，只替换判定器本身，不改变证据采集、状态机和落盘结构。

## Evidence Bundle

每个绑扎点建议保存一份标准证据包，包含：

### 1. Meta

- `task_id`
- `area_id`
- `point_idx`
- `phase`
  - `before`
  - `after`
  - 对绑前单帧判定来说，`before` 就是“当前到点后、动作执行前”现拍的那一帧
- `timestamp`
- `request_mode`
- 当前流程模式
  - `shadow`
  - `advisory`
  - `blocking`

### 2. Geometry

- `Pix_coord`
- `World_coord`
- `gripper_frame` 坐标
- 当前机器人位姿
- 关键 TF 快照

### 3. Modalities

- `ir_patch`
- `depth_patch`
- `raw_world_patch`
- `height_patch`
  - 以当前点局部深度为参考的相对高度图

### 4. Masks

- `valid_depth_mask`
- `local_depth_band_mask`
  - 例如 `z0 +- 10~20 mm`

### 5. Debug Fields

- 裁剪窗口
- 有效深度比例
- 对齐偏移量
- 对齐失败原因
- 评分分项

## Patch Strategy

### 1. 裁剪

以候选点 `Pix_coord` 为中心裁局部 patch：

- 原始 patch 优先保留 `64x64` 或 `96x96`
- 同时生成统一输入尺寸 `128x128`

规范要求：

- 原始 patch 必须保留，不允许只保存 resize 后结果
- `IR` 保留原灰度
- `depth` 同时保留原值和归一化值
- `raw_world` 至少保留局部 `z`，最好保留局部 `xyz`

### 2. 前后对齐

绑后复检时，必须先做局部对齐再做差分。

建议流程：

1. 用当前点中心做粗对齐
2. 在局部 patch 上做小范围平移或微旋转对齐
3. 优先利用：
   - `IR` 边缘
   - `depth/height` 梯度
4. 若对齐偏移超过阈值，直接判 `uncertain`

## Rule-Based Classifier

### A. 绑前单帧判定

绑前规则版的输入就是“当前到点后现拍的一张局部 patch”，不依赖历史图。它先看四类特征：

1. `quality features`
   - 有效深度比例
   - patch 非零比例
   - ROI 是否越界
   - 是否被机械结构或无效区域遮挡

2. `geometry features`
   - 局部高度图凸起程度
   - 峰值数量
   - 高度粗糙度
   - 相对钢筋平面的局部突起量

3. `IR texture features`
   - 高边缘密度
   - 高频能量
   - 方向混乱度
   - 结扣类局部纹理增强

4. `structural consistency`
   - 当前点附近是否仍像规则钢筋交点
   - 若偏离交点结构太多，则优先判 `uncertain`

输出采用双阈值：

- `score >= T_high` -> `bound`
- `score <= T_low` -> `unbound`
- `T_low < score < T_high` -> `uncertain`

### B. 绑后前后差分复检

绑后复检重点看差分特征：

- `IR difference`
- `height/depth difference`
- `新增局部凸起体积`
- `局部高度变化面积`
- `新增高频与边缘`

期望模式：

- 若绑扎成功，则局部结构和纹理应发生符合绑扎行为的增量变化
- 若前后差异很弱，或变化方向异常，则判为 `failed` 或 `uncertain`

输出采用三态：

- 差分强且方向合理 -> `success`
- 差分弱或反向 -> `failed`
- 证据质量不足或配准失败 -> `uncertain`

## Model Handoff Plan

第二阶段模型接管必须遵守以下顺序：

1. 先让模型接管 `post_bind_verification`
2. 再让模型接管 `pre_bind_classification`

原因：

- 你已经确认现场最稳定的核心信号来自绑后 `before / after` 差分
- 绑后复检更容易从真实执行样本中得到稳定标签
- 绑前单帧分类通常比绑后差分更难，需要更多现场样本

推荐模型演进路径：

### Phase 2A

轻量 `before/after` 双输入模型：

- 输入：
  - `before_ir`
  - `after_ir`
  - `before_height`
  - `after_height`
- 输出：
  - `success`
  - `failed`
  - `confidence`

### Phase 2B

轻量单帧 `IR + height` 双分支模型：

- 输入：
  - `ir_patch`
  - `height_patch`
- 输出：
  - `bound`
  - `unbound`
  - `confidence`

模型初期运行方式锁定为：

- 先在 `shadow` 模式下和规则版并跑
- 先只处理规则版 `uncertain` 的样本
- 稳定后再扩大接管范围

## Data Storage

每个绑扎点建议单独落盘到一个目录中，例如：

- `data/bind_point_evidence/<task_id>/<area_id>/<point_idx>/`

至少包含：

- `before_ir.png`
- `before_depth.npy`
- `before_world.npy`
- `before_height.npy`
- `after_ir.png`
- `after_depth.npy`
- `after_world.npy`
- `after_height.npy`
- `manifest.json`

`manifest.json` 至少包含：

- 点位索引
- 时间戳
- 坐标与 TF 信息
- 绑前判定结果
- 绑后复检结果
- 执行动作结果
- 最终人工或系统确认标签
- 所有评分分项

## Runtime Controls

新增以下运行时控制项：

- `enable_pre_bind_classification`
- `enable_post_bind_verification`
- `classification_mode`
  - `off`
  - `shadow`
  - `advisory`
  - `blocking`
- `pre_bind_bound_high_threshold`
- `pre_bind_unbound_low_threshold`
- `post_bind_success_high_threshold`
- `post_bind_failed_low_threshold`
- `evidence_patch_size`
- `alignment_max_offset_px`
- `minimum_valid_depth_ratio`

这些参数必须走统一配置，不允许继续散落在脚本常量里。

## File Changes

### Modified Files

- `src/chassis_ctrl/scripts/pointAI.py`
  - 新增标准证据采集能力
  - 新增规则版绑前判定
  - 新增规则版绑后复检入口
  - 在现有 `PointCoords.is_shuiguan` 上填充兼容结果

- `src/chassis_ctrl/src/moduanNode.cpp`
  - 在绑前加入“已绑扎拦截”或影子记录
  - 在绑后接收复检结果

- `src/chassis_ctrl/src/suoquNode.cpp`
  - 在需要时消费分类结果，决定是否跳过或复检

- `src/chassis_ctrl/launch/*.launch`
  - 接入新参数和运行模式

### New Files

- `src/chassis_ctrl/data/bind_point_evidence/`
  - 点位证据数据目录

- `src/chassis_ctrl/config/bind_point_classification.yaml`
  - 分类运行参数和阈值配置

- `src/chassis_ctrl/data/bind_classification_events.jsonl`
  - 记录绑前判定、绑后复检和影子模式事件

## Testing Strategy

### 1. 离线样本验证

从真实任务中抽取点位样本，人工标注：

- `bound`
- `unbound`
- `uncertain`
- `post_success`
- `post_failed`

先离线验证规则版命中率和误判类型。

### 2. 在线影子模式

在线上先启用：

- `classification_mode=shadow`

要求：

- 只输出结果
- 不阻断动作
- 记录每次建议结果与真实执行结果

### 3. 在线闭环模式

稳定后逐步进入：

- `classification_mode=advisory`
- 再到 `classification_mode=blocking`

建议顺序：

1. 先启用绑后复检告警
2. 再启用绑前已绑扎拦截
3. 最后再考虑把绑后失败接入自动重试

## Risks And Mitigations

### 风险 1：单帧外观漂移大，绑前误判高

缓解：

- 绑前使用三态，不强行二值
- 先以规则版保守拦截为主
- 低置信度样本不直接阻断

### 风险 2：前后视角有小偏差，差分不稳定

缓解：

- 必须做局部对齐
- 引入有效质量阈值
- 对齐失败直接判 `uncertain`

### 风险 3：现场没有成体系标注数据，模型难训练

缓解：

- 第一阶段强制全量落盘
- 先跑影子模式积累样本
- 模型只在第二阶段接管

### 风险 4：现有消息结构太窄，无法直接表达三态

缓解：

- 第一阶段保留 `is_shuiguan` 兼容输出
- 同时旁路输出完整分类状态
- 后续如确有必要，再单独升级消息定义

## Out Of Scope

本设计当前不包含这些内容：

- 重建整套点位检测算法
- 用整图检测器替换现有稳定点识别
- 立即修改 `PointCoords.msg` 做大范围协议升级
- 一开始就用重型深度学习模型直接替换规则版
