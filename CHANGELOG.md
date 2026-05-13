# 项目全局改动日志

本文档记录 `simple_lashingrobot_ws` 的项目级变更约定和近期关键调整。  
开始修改代码前，先读最新日期的记录，再进入具体包目录。

## 2026-05-13

### 扫描 Surface-DP 梁筋过滤改为点级过滤

- 用户最新口径：保留“梁筋候选 / 梁筋过滤 / 过滤半径”功能，保留扫描底图红色梁筋候选竖带和前端开关；只取消“某个梁筋 mask 命中点后连带移除同 X 位置其它交点”的逻辑。
- Surface-DP 当前运行链路继续输出 `beam_candidate_*` 诊断，启用梁筋过滤时仍按前端半径扩张 `beam_candidate_margin_mask`，但最终过滤改为点级 `point_mask`：只删除自身落入梁筋 mask 的交点，不会连带移除同一 X 位置的其它普通钢筋交点。
- pointAI ROS 继续订阅 `/web/pointAI/set_scan_beam_exclusion` 与 `/web/pointAI/set_scan_beam_exclusion_margin_mm`；前端视觉调试页继续提供梁筋过滤开关、半径输入、topic registry 和 localStorage 字段。
- 运行态诊断保留 `beam_filtered_point_count`，`beam_filter_mode=point_mask` 表示点级过滤；当前入口不再使用列级过滤 helper，也不再输出列级过滤计数字段。

## 2026-05-11

### 前端报警只落到对应层级状态

- 用户最新口径：任何索驱、末端、视觉或诊断报警都不要再占用顶部连接报警横幅；顶部连接胶囊只表达 ROS / rosbridge 连接状态。
- 报警详情统一进入前端日志；哪一层报警，就只让哪一层状态胶囊变黄并显示该层详情，例如索驱报警只让“索驱”胶囊 `warn`，末端报警只让“末端”胶囊 `warn`。
- `/diagnostics` 中各子系统 `ERROR` 级别不再映射为红色错误态，也不再汇总为顶部“等 N 项报警”；按层级转成黄色告警，恢复 OK 后回到对应运行态。

### 前端扫描入口二次收口

- 用户最新口径：设置页不再展示独立的“工作区选点”和“扫描动作”卡片；“确认工作区域”跟在“移动到选中位姿”右侧，仍在设置 / 工作区的扫描位姿卡片内。
- “触发扫描视觉”重新放回控制面板“扫描区”，且扫描区只保留这个主动视觉触发入口；移动到位姿、记录识别位姿和确认工作区域不再作为控制面板任务按钮。
- 工作区点选列表与“撤销最后一点 / 清空重选”保留在扫描位姿卡片内，避免现场误点后无法修正。

### Surface-DP 统一物理网格评分

- 当前工程状态已先保存并推送远端 tag `slam/v44`，作为本轮扫描线族语义修改前的恢复点。
- Surface-DP 扫描线族不再使用横纵线数接近 `1:1` 的强平衡门槛；小视野、正方形和长方形钢筋面统一走同一套物理网格评分。
- 新评分用候选横纵线数比例与当前 rectified 有效视野物理长宽比的一致性、线距物理先验、弱规则线召回和响应支持共同判断网格可信度；`34x21` 这类长方形全局网格可通过，正方形视野中的 `16x2` 线族假阳仍会被拒绝。
- 运行态诊断新增 `physical_lattice_score`、`physical_lattice_count_aspect`、`physical_lattice_visible_aspect`、`physical_lattice_count_aspect_error` 和 tolerance 字段；旧 `full_workspace / visible_local` 分档标签收口为 `unified_physical_lattice`，避免后续按视野大小重新分支。

## 2026-05-09

### 演示模式收回本工程静默开关

- 用户最新口径：新前端 header 的“演示模式”点击后只关闭当前 `simple_lashingrobot_ws` 工程相关进程和后台服务，然后按钮状态变绿；不再启动、停止或清理 `/home/hyq-/lashingrobotROS` 或 `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws` 里的任何内容。
- 演示模式进入动作收口为停止本工程 `tie-robot-backend.service`、三个 driver service 与 `tie-robot-rosbridge.service`，并且只按当前工作区路径清理残留 ROS 进程；不再启动旧前端、旧 `chassis_ctrl api.launch`、`tie-robot-demo-rosbridge.service` 或 `tie-robot-demo-show-full.service`。
- 演示模式退出动作仍按当前工程依赖顺序恢复 `tie-robot-rosbridge.service`、三个 driver service 和 `tie-robot-backend.service`。

## 2026-05-08

### 跳绑微调按当前区域最近账本点

- 用户最新口径覆盖 2026-05-07 的区域四宫格对账方案，并撤回“当前区域 2x2 未就绪就原地轮询”的语义：`planned_path_refine_only` 跳绑开启后，视觉侧若未返回可执行微调点，后端记录原因并跳过当前区域，不停留当前区域反复请求。
- 视觉返回可执行点时，后端不再按账本区域边界、中心线四宫格或 4 宫格完整性拒绝稳定视觉结果。
- 后端现在把每个视觉点转换到 `map` 与 `gripper_frame` 后，只在当前 `pseudo_slam_bind_path.json` 区域的 `groups[].points[]` 里按三维欧式距离找最近账本点；执行点继承该账本点的 `jump_bind`、`checkerboard_color`、全局行列等元数据，坐标使用实时视觉得到的世界坐标和 TCP 局部坐标。
- 跳绑过滤仍在最近账本点匹配之后执行：选择黑/白棋时按最近账本点的元数据过滤下发，因此稳定 2x2 不会再因为四宫格对账失败而反复重试视觉。

## 2026-05-07

### 子系统隔离与索驱持续重连

- 现场最新口径：索驱归索驱、线性模组/末端归线性模组/末端、视觉归视觉；任一子系统掉线、超时或重启时，不应通过 systemd 生命周期把其他层一起停止或重启。
- `tie-robot-backend.service`、`tie-robot-driver-suoqu.service`、`tie-robot-driver-moduan.service`、`tie-robot-driver-camera.service` 启动前仍通过 `Wants/After=tie-robot-rosbridge.service` 和 `wait_for_ros_master.py` 等待本机 ROS master 可用，但不再使用 `PartOf=tie-robot-rosbridge.service` 跟随 rosbridge 停止/重启。
- 索驱底层 `Frame_Generate_With_Retry` 不再因“重新发送命令失败超过5次”或“重新连接失败超过5次”紧急退出；通信发送/读取失败时保持节点运行并持续重连索驱，重连成功后继续重发原指令。协议明确非瞬态拒绝仍返回失败，避免限位、速度错误等硬拒绝被无限重发。

### 路径规划+微调跳绑

- `planned_path_refine_only` 路径规划+微调模式支持跳绑热开关：跳绑关闭时保留原 `/moduan/sg` 纯单点微调链路；跳绑开启时每个区域到位后直接调用 `/pointAI/process_image` 的执行微调。旧版曾要求当前帧没有 2x2 时停留当前区域继续轮询；2026-05-08 该轮询语义已撤回，当前做法是记录原因并跳过当前区域。
- 跳绑开启时的微调纠正不落盘修改 `pseudo_slam_bind_path.json`：实际执行坐标继续使用当前识别点由 `Scepter_depth_frame -> gripper_frame` 转换得到的局部坐标，账本负责当前区域边界、四宫格、黑白棋和全局行列元数据。后端只接收落在当前账本区域扩展边界内的视觉点，并用账本区域中心线划分四宫格；账本或边界内视觉不足完整 4 宫格时不下发局部漏绑点，避免相邻区域视觉点混入当前区域。
- 后端跳绑启停状态改为 `std::atomic<bool>`，和黑/白棋 parity 一样按区域执行前读取快照，保证运行中开关/切换对后续区域热生效。
- `planned_path_refine_only` 首区大 Z 下降后的视觉稳定等待和 `EXECUTION_REFINE_NO_POINTS` 重试等待均从 `1200ms` 缩短为 `300ms`，保留到位后短等待与首次无点重试，但减少首区节拍延迟。

### 长按停止并回起点 FINISHALL 等待修复

- 修复 split-node 模式下 `/moduan/driver/raw_execute_points` 与 `/moduan/return_zero_ordered` 并发竞争：驱动层 raw execute 服务现在和有序回零服务共用 `lashing_mutex`，避免长按停止刚置位后，有序回零先清掉 `moduan_return_zero_ordered_requested`，导致旧执行链继续卡在 `等待FINISHALL标志中，FINISH_ALL_FLAG=0`。
- 新增控制链回归测试，约束 raw execute 必须先取得 `lashing_mutex` 再进入 `execute_bind_points(...)`；同时放宽暂停等待测试的脆弱单行字符串断言，改为检查多行 `while` 中的真实暂停条件。

### 控制面板任务区收口与人工切区

- 前端控制面板下线“清除识别结果”“固定扫描规划”和“账本测试”三个旧按钮及其入口逻辑，任务按钮按“扫描区”“执行层”“区域切换”重新分组展示。
- “开始执行层”改名为“执行全局绑扎”，“执行层视觉单侧”改名为“单点视觉测试”；扫描区保留“移动到位姿”“设为识别位姿”“确认工作区域”“触发扫描视觉”，执行层保留“执行全局绑扎”“触发单点绑扎”“单点视觉测试”“记忆续跑开始”。
- 新增“上一个区域 / 下一个区域”人工切区：前端先发布 `/web/cabin/manual_area_takeover` 终止当前自动执行链并让线性模组归零，再按当前区域进度或当前位置邻近区域移动索驱到相邻 `cabin_pose`；暂停后未恢复时，后续区域交由人工操作。

### 执行微调近点排斥清理

- 清理执行视觉链路遗留的“近点排斥/去重”算法：`MODE_EXECUTION_REFINE` 的 Hough 候选点不再因为世界 XY 距离小于旧 `100mm` 阈值而成对丢弃；多根钢筋靠得近时，只要有有效 3D 坐标且落在 TCP 执行范围内，就继续进入排序和下发。
- 执行底图诊断同步移除 `DUP` 标记，日志也不再输出“去重移除”；现场漏点只剩 `H` 原始交点、`ZERO` 无有效 3D 坐标、`OUT` 超出 TCP 范围和 `SEL/编号` 最终输出这几类有效门控。
- 执行微调输出进一步收口为完整 `2x2`：在 TCP 执行范围内按行列匹配完整矩阵，从能组成 `2x2` 的候选组里选择整体离 TCP 零点最近的一组，并按 `(1,1)->(1,2)->(2,2)->(2,1)` 的 TCP 局部蛇形顺序编号下发；若缺行或缺列则不向执行层下发零散点。
- live_visual 账本+微调链路移除旧 X/Y 接纳门限：视觉微调点不再因为相对扫描参考点超过固定毫米阈值而被丢弃；是否执行收口到“触发单点绑扎”视觉动作和上述执行微调 `2x2` 输出条件。

### 扫描 DP 底图梁筋候选可视化

- Surface-DP 运行态新增 `beam_candidate` 梁筋候选诊断：基于收束底图中的宽、连续、高响应竖向 band 识别梁筋候选，并输出 `beam_candidate_bands`、`beam_candidate_count` 和像素统计。
- `/perception/lashing/scan_surface_dp_base_image` 与 `/perception/lashing/scan_surface_dp_completed_surface_image` 会用红色半透明竖带叠加梁筋候选，同时保留黄色 DP 交点；视觉调试设置里可启用梁筋过滤并填写过滤半径，默认半径 150 mm，启用后按梁筋候选扩张范围过滤最终绑扎点，不删除普通钢筋线族。
- 梁筋候选识别补充「黑色竖沟 + 双侧窄亮边」形态：现场截图中梁筋中间常表现为贯穿全高的暗沟，而不是整条宽亮带；检测逻辑会把两侧连续亮边与中间低覆盖暗沟合并成一条 beam_candidate 竖带，避免漏掉这种梁筋。
- 梁筋候选进一步增加高度门控：竖带这一列必须在 `background_depth - filled_depth` 高度响应上高于邻近普通钢筋才会标为梁筋；红色半透明带按高度峰值列收窄，避免把只是更宽、更亮但不更高的普通钢筋误判为梁筋。
- 梁筋候选再增加网格线族上下文门控：候选竖带必须位于相邻普通竖向钢筋列之间，并接近这两列的中点；如果候选中心落在正常竖筋 line-family 上，会被视为普通钢筋抬高或局部变粗而剔除，降低误识别红带。
- 梁筋高度门控的“周围钢筋”改为邻近上下文比较：不再跨多列取远处更高竖筋压低局部梁筋候选，现场帧中红带由 `x=149..156` 一条恢复为 `x=149..156` 与 `x=331..339` 两条；同时保留普通加粗竖筋拒绝和黑色竖沟梁筋贴在线族上时的豁免。
- 梁筋候选新增“结构连续但高度扁平”兜底：当现场另一根梁筋高度响应只有约 `0.48~0.54`、但竖向响应更宽且连续时，仍按 `wide_continuous_column` 标为梁筋候选；如果 line-family 已把它吞成普通竖筋列，lattice gate 会结合相邻间距轻微畸变恢复该梁筋，避免把梁筋当普通钢筋导致列间距异常。

### 跳绑长按启停与黑白棋选择

- 前端控制面板的跳绑按钮改为长按启停：长按发布 `/web/moduan/send_odd_points` 的 Bool 开关，单击只在“只绑黑棋 / 只绑白棋”之间切换，不再误触启停。
- 新增 `/web/moduan/jump_bind_parity`（`std_msgs/Int32`）同步前端选择，约定 `0=black`、`1=white`；后端全局执行、账本测试和账本+微调执行开启跳绑时按当前 parity 过滤账本点。
- 3D 规划路径图层在跳绑开启后，会额外用更大、更深的点叠加高亮当前要跳绑的黑/白棋点；关闭跳绑时恢复普通全点显示。

### 跳绑账本点颜色属性

- 扫描后生成的 `pseudo_slam_points.json`、`pseudo_slam_bind_path.json` 和执行回写的 `bind_execution_memory.json` 会为每个棋盘格点写入 `jump_bind` 和 `checkerboard_color`；当前约定 `checkerboard_parity==0` 为 `black` / `jump_bind=true`，`checkerboard_parity==1` 为 `white` / `jump_bind=false`。
- 全局执行、账本测试和账本+微调执行在跳绑开关开启时改为优先按 `jump_bind=true` 过滤；旧账本缺少该字段时仍回退到 `checkerboard_parity==0`，避免现场已有产物立即失效。
- 前端静态规划路径 API 和 `bindPathGeometry` 归一化会保留 `jump_bind`、`checkerboard_color` 与 parity 元数据，后续 3D/调试显示可直接区分黑白棋子。

### 原始账本口径回退

- 用户最新口径：撤回账本杂点过滤方案，扫描生成 `pseudo_slam_points.json` 和 `pseudo_slam_bind_path.json` 时保留扫描原始点；执行层不再按 `outlier`、`blocked` 或 planning 棋盘成员标记删点。跳绑仍只作为用户主动选择黑 / 白棋点的执行开关。

### 视觉调试可配置绑扎分组点数

- “视觉调试”设置新增“每组点数”，默认保持 4 点，即原有 2x2 分组；扫描动作会把该值随 `StartPseudoSlamScan` 服务和 action 目标透传到后端规划链。
- 动态绑扎规划的非 4 点模式收口为统一物理方向：短轴最多 2 点、长轴最多 3 点，3 点跨度始终落在线性模组更长、在当前规划高度更安全的那根物理轴上；对应到网格索引上，分组标签会根据行列轴与世界 X/Y 的映射表现为 `2x3` 或 `3x2`，但不会在同一现场口径下混用两种物理朝向，也不再让 9 点请求生成 `3x3`。若用户填 9，规划会按世界坐标蛇形优先填当前物理长轴方向上的 6 点组，再用 `2x2`、`1x3`、`1x2` 等更小可达矩形补剩余点。
- 每个候选组都会在当前路径规划高度下，以该组中心规划索驱位姿，再校验组内所有点是否落在线性模组可达盒内；若用户设置的点数无法形成任何可达分组，`pseudo_slam` 会返回“无法规划”提示，要求调小每组点数或调整路径高度。
- 动态绑扎路径规划进一步收口为以中心重合反算索驱位姿：每个分组的世界坐标中心优先对齐线性模组工作盒中心，即 `tcp_max_x / 2`、`tcp_max_y / 2`、`tcp_max_z / 2`；旧 `template_center_*` 偏移不再参与索驱位姿反算。
- 默认 4 点分组的成组口径回到 `slam/v35`：固定 2x2 切块、边缘 2 点补组和蛇形排序不因中心反算高度触底而丢组；若索驱高度低于最小绑扎高度，最终 `cabin_pose.z` 按 v35 口径夹到安全下限，保持 256 点网格可稳定输出 64 个 2x2 组。
- 规划层新增 `requested_group_point_count` 默认值 4，并补充 6 点、9 点不可达和 Surface-DP 网格轴向推断相关测试，避免现场行列轴与世界 X/Y 交换时误分组。

### 视觉调试设置填完即用

- “视觉调试”卡里的释放帧数、索驱规划 Z 下限、自适应分组、梁筋过滤开关、梁筋过滤半径和绑扎范围输入统一改为填完即用：输入变化会立即保存设置、刷新 3D / IR 绑扎范围，并同步发布 `/web/pointAI/set_stable_frame_count`、`/web/pointAI/set_execution_refine_tcp_roi`、`/web/pointAI/set_scan_beam_exclusion` 与 `/web/pointAI/set_scan_beam_exclusion_margin_mm`，不再需要点击确认。
- 设置页删除“触发视觉服务”按钮；视觉调试页只负责参数热更新和持久化，主动视觉触发入口收口到控制面板扫描/执行按钮。
- `planned_path_refine_only` 跳绑微调失败原因进一步区分：如果 pointAI 已经返回可执行视觉点，但账本最近点匹配或跳绑过滤没通过，后端日志会直接说明失败原因；2026-05-08 已撤回失败后原地轮询语义，当前失败会跳过当前区域。

### 单点绑扎区域内蛇形点序

- `/moduan/sg` 单点绑扎在执行微调 Hough 返回一个区域的多个点后，会先把相机点转换到 `gripper_frame`，再按 TCP 局部 `x` 分行、`y` 交替方向蛇形排序后下发线性模组，确保绑扎枪按区域内蛇形点序移动。
- `MODE_EXECUTION_REFINE` 返回给服务响应和执行底图编号的点序同步改为同一 TCP 蛇形口径：从 TCP 局部 `x` 最小行开始，偶数行 `y` 小到大，奇数行 `y` 大到小；扫描建图点序不随本次修改改变。

### 执行链移除重复 TCP 行程硬校验

- 现场口径固定：绑扎点从识别到下发只走一层执行范围校验，即 pointAI 的视觉 / TCP ROI 校验；控制层和预生成组加载层不再使用旧 `X[0,360] / Y[0,320] / Z[0,160]` 或类似硬行程二次拒绝点位。
- `tie_robot_control` 已删除 `execute_bind_points(...)` 内的 `is_valid_precomputed_tcp_travel_point(...)` 过滤和相关 `kTravelMax*` 常量，视觉已经选中的点不会再因控制层旧边界被丢弃；手动 `/moduan/move` 入口也不再复用这组旧硬范围拦截。
- `tie_robot_process` 读取 `pseudo_slam_bind_path.json` 的预生成局部点时只校验字段存在，不再按虎口范围二次过滤；路径生成阶段仍可使用当前 `380 / 330 / 160 mm` 的规划参数形成可执行分组。
- pointAI 和前端 TCP 线模遥控的默认范围显示统一到 `X[0,380] / Y[0,330] / Z[0,160] mm`，避免 UI 或视觉显示继续暴露旧 `360 / 320 / 140` 口径。

### 三维绑扎范围图层开关

- “显示与视角 / 图层设置”新增“绑扎范围”开关，单独控制三维场景中 gripper_frame 下的线性模组绑扎范围实体显示/隐藏；该开关随 topic layer state 持久化，不改变视觉调试页的范围数值，也不影响 pointAI ROI 下发。
- 绑扎范围实体不再被“机器”图层硬性联动隐藏：只要该图层开关打开且 `gripper_frame` TF 可用，即使关闭机器模型，也可以单独查看绑扎范围长方体。

## 2026-05-05

### 动态绑扎路径按世界 X+ 蛇形遍历

- pseudo_slam 预生成绑扎路径的 2x2 候选组排序不再把视觉 `global_row/global_col` 当成固定世界 Y/X 轴；row/col 只用于判断棋盘相邻关系，最终区域顺序直接按绑扎点 `World_coord` 聚成世界 Y 带，并在每带沿世界 X+ / X- 交替蛇形遍历。
- 现场 S2 点表的 `global_row` 来自图像/工具从上到下轴，`global_col` 来自右到左轴，在当前相机安装下会与索驱世界 X/Y 交换；路径起点因此必须以世界坐标下的最小 X/Y 组为准，而不是以点表 row/col 最小为准。

### 实际移动 TCP 显示与当前虎口相对坐标

- 前端 3D 橙色 TCP 方块不再只贴在静态 `gripper_frame` 外参原点；现在把 `/moduan/moduan_gesture_data` 的线性模组当前位置叠加到 `gripper_frame` 后显示实际移动 TCP 位置，底部线模全局坐标与 3D 方块共用同一换算。
- 执行微调结果图中的 `tcp=(...)` 改为当前运动 TCP/虎口坐标：先按 `Scepter_depth_frame -> gripper_frame` 得到线模绝对目标坐标，再减去当前线性模组 X/Y/Z，使越靠近当前虎口的点数值越接近 0；执行层写 PLC 的点位仍使用绝对线模目标坐标，不受显示相对坐标影响。
- `pointAI` 新增订阅 `/moduan/moduan_gesture_data` 缓存当前线模位置；若线模状态暂未到达，显示转换会自然退回只基于静态相机-TCP外参的旧口径。
- 前端图像层“执行底图 Hough二值”对应的 `/perception/lashing/execution_refine_base_image` 在 Hough 输出点生成后会重新发布带点位叠加的 `bgr8` 调试图：白/黑二值底图不变，识别出的执行点以黄色圆圈、红色中心和编号标出。
- 视觉图像层不再使用固定像素矩形 ROI：移除 `point1/point2` 白框过滤、执行 Hough 的 `roi_reject` 门和执行范围 mask 对静态 ROI 的叠加；候选点只受有效 3D 坐标、手动/规划工作区和执行范围约束。2026-05-07 起执行微调里的近点去重也已移除。
- 执行层视觉微调恢复独立的 TCP 遮挡黑色 mask：仅 `MODE_EXECUTION_REFINE` 会在 Hough 二值化前把已知 TCP 遮挡矩形 `(160,0)-(523,80)` 置黑，不作为点位 ROI 过滤，也不产生 `ROI` 拒绝诊断。
- 执行层视觉微调的 ROI 改为 TCP 坐标执行盒，而不是像素矩形：Hough 二值化前会把 raw world 像素按 `Scepter_depth_frame -> gripper_frame` 外参批量转换，只保留 `x[0,380] / y[0,330] / z[0,160]mm` 内的像素和候选点，范围外视图不再参与 Hough。
- 绑扎点识别结果的编号与行列索引按当前现场像素轴向固定：画面上方为 `x=0`，从上到下为 `x+`；画面右侧为 `y=0`，从右到左为 `y+`，即右上角为 TCP 工具原点。Surface-DP 扫描点和执行层 Hough 输出点都按该口径从小坐标开始排序。
- 执行层结果图 `tcp=(...)` 不再直接显示 gripper 投影轴值；现在与红外 TCP 工作范围覆盖层使用同一工具坐标口径：`tcp.x = 380 - gripper_y`、`tcp.y = gripper_x`、`tcp.z = gripper_z`，再减去当前线性模组位置。这样画面上方点的 `tcp.x` 小于下方点，同列上下点编号不会再出现 1 的坐标大于 3。
- “执行底图 Hough二值”进一步叠加诊断标记：`H` 为 Hough 原始交点，`ZERO` 为取不到有效 3D 坐标，`OUT` 为 TCP 执行范围外，`SEL/编号` 为最终输出点；现场漏点时可直接从同一图层判断掉在哪道门。2026-05-07 起不再存在 `DUP` 近点去重门。

### 单点绑扎相机点到 TCP 局部坐标修正

- 现场截图中 `tcp=(-65,72,854)` 这类数经排查并非 TF 后的 TCP 坐标，而是 `/perception/lashing/points_camera` 保持的 `Scepter_depth_frame` 原始相机坐标；点消息继续保持 raw camera 语义，执行结果覆盖原图的文字标签改为显示 `gripper_frame` 下的 `tcp=(...)` 虎口局部坐标。
- `/moduan/sg` 在调用 `MODE_EXECUTION_REFINE=4` 获得 Hough 结果后，新增 `Scepter_depth_frame -> gripper_frame` 的 TF 转换，再把转换后的 TCP 局部点交给 `execute_bind_points(...)`，恢复“视觉输出 raw camera，下游坐标层负责执行坐标”的工程约定。
- 线性模组执行层的预生成点校验从仅检查局部 `Z[0,140]mm` 扩展为完整 TCP 行程 `X[0,360]mm / Y[0,320]mm / Z[0,160]mm`，防止转换后仍越界的点进入 PLC 点位队列；2026-05-06 已将旧 140mm 执行上限与规划/感知旅行范围统一到 160mm。

### 单点绑扎恢复旧 Hough 执行语义

- 前端“触发单点绑扎”继续只调用后端原子服务 `/moduan/sg`，不在前端拆视觉步骤。
- `/moduan/sg` 的视觉阶段改为 `/pointAI/process_image` 的 `MODE_EXECUTION_REFINE=4`，即现有执行层“平面分割 + Hough”局部视觉分流；不再走 `MODE_BIND_CHECK=2` 的 Surface-DP/稳定 2x2 校验分支。
- 旧 `20260403` 工程链路经复查为：`/web/moduan/single_bind` -> `/moduan/sg` -> `/pointAI/process_image` -> `pre_img()` 深度二值化、骨架化、`HoughLinesP`、交点聚类、ROI 过滤 -> 过滤可执行范围后的全部点写入线性模组点位队列 -> 使能执行并等待 `FINISHALL`。因此当前单点绑扎不应从 Hough 返回点里挑最近 1 个点执行。
- 新增 `src/tie_robot_control/test/test_single_point_bind_chain.py` 锁定上述语义：后端单点绑扎必须请求 `kProcessImageModeExecutionRefine`，并把返回点集合整体交给 `execute_bind_points(...)`。

## 2026-05-04

### 扫描层接入 12-16 cm 物理间距先验

- 用户明确现场钢筋间距为 12-16 cm，钢筋网规格约为 `(15-18) * (15-18)`；扫描层全场识别不再以旧 8x8/64 点作为默认目标。
- `scan_surface_dp` 的运行态主链新增物理先验选线：根据 `rectified_geometry.resolution_mm_per_px` 将 12-16 cm 换算成像素间距，当前 5 mm/px 下为 24-32 px；当 rectified 视野足以容纳全场网格时，每轴优先选择 15-18 根线，当前现场帧输出 16x16=256 个候选绑扎点。
- 小视野不会被强行套全场线数：当画面尺度或工作区只容纳少量可见钢筋时，主链切换到 `visible_local`，按当前可见 2-18 根线输出局部交点；它只能识别当前画面里的可见交点，不能从 2-3 根钢筋直接推断完整全场网格。
- 修复全场服务触发仍返回 64 点的问题：运行态不再把旧 8x8 线族作为全场最终输出兜底；当 `fused_instance_response` 单轴响应不足时，会在物理先验下从 `Frangi / Hessian / depth_gradient / IR / combined` 等底图中选择能恢复 15-18 根线的线族。
- 清理旧版本残留：扫描主链 Surface-DP 失败时不再自动回退到 2026-04-22 depth-only S2；`workspace_s2` 的 8 根线/64 点偏置改为显式 `LEGACY_*` 命名，Surface-DP 不再调用 legacy axis-aligned 线族作为补全面支撑；前端视觉触发、内部 overlay 命名和项目关系图文案改为 Surface-DP 物理先验，相机原始绑扎点 TF child 前缀改为 `surface_dp_bind_point_*`，并清理旧 hash 静态资源。
- 新增 `test_scan_surface_dp_runtime.py` 覆盖全场物理先验 16x16 和小视野 2x3 两种行为；现场运行态报告挂载到 `/reports/live_surface_dp_physical_runtime_20260504_232729/`。

### 扫描层 Surface-DP 新主链接入

- 用户最新口径：扫描层继续推进 `combined / fused_instance_response -> Hessian + Frangi 脊线增强 -> binary candidate + skeleton -> completed_surface_mask -> DP 曲线沿局部 ridge 收束 -> 曲线交点输出 -> instance_graph junction 只做验证 / 补召回`。
- `MODE_SCAN_ONLY` 的触发链路仍保持 `/pointAI/process_image request_mode=3`、现有发布话题和 `PointsArray` 输出结构；算法本体从 2026-05-03 的 depth-only S2 主链切到 Surface-DP 主链。
- 新增运行态纯算法模块 `tie_robot_perception.pointai.scan_surface_dp`：负责组合响应、Hessian/Frangi、候选二值化、骨架诊断、补全面、DP 曲线族和曲线交点；运行态不导入 `tools/` 报告脚本。
- `manual_workspace_s2.run_manual_workspace_s2_pipeline()` 现在优先调用 `run_manual_workspace_surface_dp_pipeline()`；原 2026-04-22 / 2026-05-03 depth-only 网格生成链保留为 `run_manual_workspace_s2_depth_only_pipeline()`，仅在 Surface-DP 失败或无有效相机坐标时回退。
- 固定 snapshot `.debug_frames/rebar_instance_segmentation_modalities_20260430_112028` 验证：runtime Surface-DP 输出 `[8, 8]` 线族、64 个曲线交点，`mean_completed_surface_score=0.984`；对照实验仍显示旧 depth-only 运行复刻会输出 867 点、线数 `[17, 51]`。
- `instance_graph junction` 不作为直接全量输出点源，只进入诊断字段和后续补召回依据；避免 skeleton junction 原始过检进入扫描账本。

## 2026-05-03

### 视觉扫描算法复刻 2026-04-22 PR-FPRG

- 用户明确口径：视觉请求和触发链路保持当前 `/pointAI/process_image request_mode=3`，只把扫描视觉算法本体恢复到 2026-04-22 那版 `manual workspace S2`。
- 扫描 S2 主链回到 depth-only 版本：手动工作区透视展开后，基于深度背景差分构造响应图，分别对 rectified 图的纵向、横向 profile 做周期和相位估计，再用 `build_workspace_s2_projective_line_segments` 与 inverse mapping 投回原图。
- 扫描 S2 与 `38baa98` 的算法差异继续收口：运行态会完整评分 `background_depth - filled_depth` 与 `filled_depth - background_depth` 两个 depth 响应变体，并按纵横周期估计总分选择最佳变体；透视展开几何优先使用当前 `map` 口径的 `corner_world_map_frame`，缺失时才回退兼容当前已有的 `corner_world_camera_frame`。
- 当前扫描算法不恢复旧实验链路中的行/列峰值 line-family 主链、depth+IR 组合响应、`axis_peak_families` 日志口径、稳定采样择优或 phase lock；当前运行态 Surface-DP 梁筋过滤由视觉调试开关和半径输入控制，默认半径 150 mm，启用后只按梁筋候选扩张 mask 过滤最终绑扎点，不删除普通钢筋线族。
- `MODE_EXECUTION_REFINE` 仍按 2026-04-30 口径走平面分割 + Hough 局部视觉；本次不修改前端按钮、Web action、`/pointAI/process_image` 服务入口或执行层 Hough 分流。

### 当前视觉识别流程效果页

- 新增 `docs/reports/current_visual_recognition_flow/index.html`，把当前扫描识别和执行微调两个视觉分支的输入、中间效果和输出整理成静态网页。
- 新增 `src/tie_robot_perception/tools/build_current_visual_recognition_flow_page.py`，可基于 `docs/releases/slam_v30/visual_modalities` 离线快照重新生成 15 张流程效果图和页面清单。
- 新增 `src/tie_robot_perception/test/test_current_visual_recognition_flow_report.py`，校验报告页存在、包含当前算法口径说明，并确保页面引用的本地图片都存在。

## 2026-04-30

### 视觉研究归档与执行层 Hough 分流

- 新增 `docs/archive/vision_research_runtime_scheme_2026-04-30.md`，冻结本轮视觉研究结论：固定识别位姿扫描触发走 2026-04-22 `PR-FPRG` 拓扑恢复方案，执行层逐区到位后走平面分割 + Hough 局部视觉。
- `/pointAI/process_image` 的 `MODE_SCAN_ONLY` 和 `MODE_EXECUTION_REFINE` 已重新拆开：扫描触发继续调用 `run_manual_workspace_s2_pipeline(publish=True)`；执行微调调用新的 `execution_refine_hough.py`。
- 执行层 Hough 链路输入为 `/Scepter/worldCoord/world_coord`，该图已经在世界坐标处理层经 PCL `SAC_RANSAC` 去主平面；输出点坐标仍从 `/Scepter/worldCoord/raw_world_coord` 取原始相机坐标，再由流程层匹配扫描账本并转换为 TCP / 虎口局部执行点。
- 旧 `docs/archive/legacy_ransac_hough_pointai/` 仍只作为追溯和局部执行层参考，不直接导入 `matrix_preprocess.pre_img`，避免把旧 Hough 门控重新接回扫描建图主链。

### 新前端演示模式直跑旧 show_full

- 用户最新口径改为不再使用旧展示转义层：旧 `20260403` 展示链进入演示时直接由旧工作目录 `roslaunch chassis_ctrl api.launch` 接管，`show_legacy_driver_bridge` / `tie-robot-show-legacy-shared-driver-stack.service` 不再作为演示链路的一部分。
- 新前端 header 的“演示模式”状态按钮进入演示时会确保 5173 旧前端在线，停止当前 `tie-robot-rosbridge.service`、`tie-robot-backend.service`、三个 driver service 和旧转义层服务，再启动轻量 `tie-robot-demo-rosbridge.service` 与旧工作目录 `tie-robot-demo-show-full.service`；进入后按钮变绿并打开 `http://<当前主机>:5100/`。
- `tie-robot-demo-rosbridge.service` 只运行 `rosbridge_websocket + rosapi`，不包含当前工作目录 `tf_stack.launch` / `api.launch`，避免演示态继续拉起 `robot_tf_broadcaster`、`web_action_bridge_node` 等当前 TF/API 节点。
- 演示态 rosbridge 增加 topic whitelist：保留 `/pointAI/result_image` 给旧 `pointAI.py` 的绑扎点画面，Scepter 相机图像只放行 `/Scepter/*/image_raw/compressed` 和 camera_info，阻断旧前端继续订阅大流量 raw 图像。
- 再次点击“演示模式”会停止 `tie-robot-demo-show-full.service` 与 `tie-robot-demo-rosbridge.service`，随后启动当前工作目录完整 `tie-robot-rosbridge.service`、三个 driver service 和 `tie-robot-backend.service`，恢复本程序普通运行态。
- `install_demo_mode_service.sh` 现在同时安装 `tie-robot-demo-rosbridge.service` 与 `tie-robot-demo-show-full.service`；两个 unit 都只按需启动，不设置开机自启，安装时会禁用已存在的 `tie-robot-show-legacy-shared-driver-stack.service`，避免转义层继续常驻。

### 旧展示链共享当前驱动与 rosbridge

- 该共享底层/转义层方案已被上方“轻量旧 rosbridge + 旧 show_full”演示模式取代，不再作为当前演示路径。
- 明确新旧工作目录运行时只允许一套底层：当前工作目录的 `tie-robot-rosbridge.service`、三个 driver service 和 ROS master 是唯一底座；旧展示链不再单独启动旧工作区 `rosbridge_server` 抢占 `9090` 或 `/rosbridge_websocket`。
- 新增 `tie-robot-show-legacy-shared-driver-stack.service` 与安装脚本，服务只启动 `roslaunch tie_robot_bringup show_legacy_shared_driver_stack.launch start_legacy_frontend:=false`，由该共享栈暴露 `/show_legacy/*` 旧入口、旧 `pointAI` 代理和旧工作区算法 runner。
- `show_legacy_driver_bridge` 是适配包/节点，不是独立底层守护；它只把旧前端、旧算法的服务和话题映射到当前驱动层，不能拥有索驱、线模、相机或 rosbridge。
- 旧前端继续作为静态页面由 `tie-robot-show-legacy-frontend.service` 服务，连接当前 `9090` rosbridge；新前端继续走 `8080`。新旧前端可同时打开，旧逻辑仍走旧入口，新逻辑仍走当前入口，底层驱动共享但业务命名空间隔离。

### Home 点位与机器人 TF 原点收口

- 新增“设置 / Home点位”页：操作员先在索驱上位机恢复机器人到 Home，再在本程序点击“当前位置设为Home”或手动编辑 Home X/Y/Z 并保存；保存后 Home 点位持久化到 `robot_home_tf.yaml`，并提供一键回 Home。
- `map` 明确定义为索驱全局坐标系，`map.z=0` 是索驱绝对零点，不代表地面；索驱上位机当前 Z 值或 Home Z 值不能直接解释为 `base_link` 离地高度。
- `map -> base_link` 由 `/cabin/cabin_data_upload` 的当前索驱坐标连续发布，`base_link` 原点定义为整个机器立方体最底面的中心点；实体模型只关心自身坐标系，跨坐标系投射统一交给 TF。
- 现场确认相机坐标系原点位于 `base_link` 的 `z+` 方向 `460mm`；当前 `base_link -> Scepter_depth_frame` 以 `x=0,y=0,z=460mm` 和相机朝地旋转发布，后续如测得横向偏置再补入 TF 配置。
- 虎口 TCP 工具模型原点定义为 TCP 长方体最底面的中心点；前端橙色 TCP 长方体作为 `gripper_frame` 的子几何显示，底面中心对齐 `gripper_frame`。
- 虎口/TCP 坐标系方向定义为：`z+` 朝地面，`y+` 与 `map.y+` 同向，`x+` 与 `map.x-` 同向；当前通过 `Scepter_depth_frame -> gripper_frame` 的 `yaw=pi` 实现，这是合法旋转，不是镜像置反。
- `base_link -> Scepter_depth_frame` 不再从前端人工填写；`RobotHomeCalibration` 服务请求只接受 `command + home_x/y/z`，相机 TF 和深度最远点投射状态只作为响应展示。

### 索驱 TCP 运动协议修正

- 修复驱动层 `0x0011` TCP 增量运动帧：协议只允许“单轴方向控制字 + 速度 float + 正的增量位移 float + 校验”和 16 字节总长，不再误用 `0x0012` 绝对位姿的六自由度目标帧。
- `/cabin/driver/incremental_move` 现在会在驱动层拒绝多轴或零轴增量请求，避免把无法表达的请求发给索驱上位机；按钮/键盘遥控仍应一次只传一个非零轴。
- `0x0011`、`0x0012`、`0x0013` 回包状态字按协议字节 2~5 的 `UINT32` 解析，并把 bit 原因写入 `detail`，例如 `逆解未激活`、`电机未全部使能`、`设备运动中`、限位和速度错误等，不再只暴露 socket 层 `connection closed by peer`。
- 修复一次增量运动 `read timeout` 后前端误报索驱断开的状态分叉：状态轮询若已成功读回 `/cabin/cabin_data_upload`，会同步把驱动 transport 标回 `ready` 并清空旧 `transport_error`，避免 fresh state 与 diagnostics 显示互相打架。
- 现场早期看到按钮遥控发送 `0x0011` 短帧后直接关闭 TCP，后续定位到同 socket 状态心跳读包边界不稳可能污染运动回包；读包边界修复后，遥控点动统一收口到协议 `0x0012`，当前默认绝对点动使用 `bit0=TCP绝对位置运动触发`，相对点动作为页面可选项使用 `bit1=TCP相对位置运动触发`，`0x0011` 只保留为协议辅助函数。
- `CabinTcpTransport` 的发送、等待回包和接收失败现在会把 `request_command` 与 `request_frame=[...]` 写入 `detail`，便于区分“设备按协议返回拒绝状态”和“对端未返回状态字直接断开 socket”。
- 停止帧遇到 `connection closed by peer` 仍按“停止指令已投递”处理；该判断改为匹配 `detail` 中的 peer-close 片段，兼容新增的 request frame 诊断字段。
- `0x0012` 运动回包若出现类似 `status_word=0xC3820000` 的高位状态字，驱动会保留 `raw_status_le32`，并按协议位宽自动归一化为可解释的 `status_word`，同时输出 `status_source`、原因列表和 `response_frame`；该现场值归一化后对应逆解未激活、电机未全部使能和若干限位位。
- 修复旧状态心跳读包的协议边界：`Frame_Generate` 现在按 `Rlen` 循环接收完整回包，`read_cabin_state` 按协议固定读取 144 字节状态包，不再单次 `recv()` 后就放行，避免状态包尾字节残留在同一 TCP 流里，被下一条遥控运动命令误当成 8 字节运动回包。

### 前端显示与视角三模式

- “显示与视角”页改为紧凑的单行视角控制，减少设置页大面积留白。
- 三维视图默认切到“自由视角”，保持现有 Orbit 拖拽交互；“相机视角”锁定在 `Scepter_depth_frame` 原点，并沿相机自身 `z+` 方向看；“俯视视角”锁定世界原点上方，并沿全局坐标 `z-` 方向看。
- 原“跟随相机”语义收口为“跟随原点”：自由视角下保留当前拖拽角度并随当前视角原点平移，锁定视角下持续回到对应原点与朝向。

### rosbridge 重启后依赖服务重新注册口径

- 现场视觉断链根因是 `tie-robot-rosbridge.service` 重启并重新拥有 ROS master 后，旧的相机、后端和末端进程仍在运行但没有重新注册到当前 master，导致 `/Scepter/ir/image_raw`、`/Scepter/depth/image_raw` 无发布者，`/pointAINode` 等节点 XML-RPC 地址拒绝连接。
- 旧方案曾让 `tie-robot-backend.service` 和三个 driver service 通过 `PartOf=tie-robot-rosbridge.service` 跟随 rosbridge 重启；该方案已被上方“子系统隔离与索驱持续重连”取代，避免 rosbridge 或视觉链路动作牵连索驱、末端。
- 如果 rosbridge/ROS master 确实重启，按当前口径应由前端或人工只重启需要重新注册的子系统；不要用 systemd `PartOf` 做自动连坐重启。

### ROS 全栈快速重启清理残留进程

- `/api/system/restart_ros_stack` 现在先 `systemctl stop` 全部 ROS 相关 unit，再执行快速残留清理：扫描本工作空间 `devel/lib/tie_robot_*` 节点、`roslaunch tie_robot_bringup`、`rosmaster :11311`、`rosout`、`rosbridge_websocket`、`rosapi_node` 和 `tf2_web_republisher`，先发 `SIGTERM`，短等待后对仍残留的进程发 `SIGKILL`。
- ROS 后端、rosbridge 和三个 driver unit 的 `TimeoutStopSec` 均收短为 `5s`；前端全栈重启的 stop 等待收短为 `8s`，避免旧节点慢退出拖住现场恢复。
- `systemctl` 超时现在会返回结构化失败结果，不再让 HTTP 控制请求抛异常；真实调用 `/api/system/restart_ros_stack` 已验证按 `stop -> cleanup -> start rosbridge -> start drivers -> start backend` 顺序完成。

### 前端网络配置卡片收口为 IP 保存与连接测试

- 设置页原“网络测试”改为“网络配置”，索驱和线性模组卡片分别维护对应上位机 IP；点击“保存并测试”时只按输入框当前 IP 调用 `/api/network/ping`，空输入不再静默回退默认地址。
- ping 结果改为按钮状态反馈：测试中为蓝色，成功为绿色，失败为红色；结果区只显示“连接成功/连接失败/正在测试连接”等现场可读状态，不再把 stdout/stderr 或命令行摘要作为主界面内容。
- 默认地址仍为索驱 `192.168.6.62`、线性模组 `192.168.6.167`，浏览器侧会继续用 localStorage 保存现场输入。

### 索驱遥控默认绝对点动并保留相对模式

- 前端“索驱遥控”方向按钮和键盘遥控默认回到绝对点动：用 `/cabin/cabin_data_upload` 的当前索驱原始坐标加上本次步距，调用 `/cabin/driver/raw_move`，驱动层下发 TCP `0x0012` 位置运动帧，控制字为 `0x0001`（`bit0=TCP绝对位置运动触发`）。
- 遥控页新增“绝对点动 / 相对点动”模式切换并持久化到 localStorage；切到相对点动时继续调用 `/cabin/driver/incremental_move`，驱动层下发 TCP `0x0012` 控制字 `0x0002`（`bit1=TCP相对位置运动触发`）。
- 旧 TCP `0x0011` 增量运动帧保留在协议层作为独立 helper/诊断对象，不作为遥控点动默认实现。
- 遥控页“绝对目标位姿”输入与执行按钮继续保留，直接调用 `/cabin/driver/raw_move`；输入框默认随 `/cabin/cabin_data_upload` 的当前索驱原始坐标填充，用户正在编辑时不被状态刷新覆盖。
- 遥控页内重复的“当前索驱坐标”块已删除，当前坐标只保留在页面中心/底部机器位置条显示。

### 前端索驱遥控改走驱动守护 raw move

- 当前守护驱动节点 `/suoqu_driver_node` 注册的是 `/cabin/driver/raw_move`，不是高层 `/cabin/single_move`；前端“索驱遥控”和一键位置移动改用驱动守护提供的 raw move 服务，避免后端算法层未启动时提示 `Service /cabin/single_move does not exist`。
- `/cabin/single_move` 仍属于 `suoquNode` 的 `cabin_motion_controller` 角色，包含等待到位等高层逻辑；自动任务/扫描仍走 Web action 与后端执行链，不把手动遥控混入任务动作链。
- 修复 `/cabin/motion/stop`：索驱停止帧已发出后，对端关闭 TCP 连接按“停止指令已下发”处理；状态读取线程重连后刷新本轮 socket，避免继续用旧/无效 fd 导致驱动节点紧急退出。

### TF 全局 X 方向取反回退

- 回退此前“物理 X+ 对应数字全局 `map X-`”的现场校正；当前 TF 层恢复把索驱 `cabin_state_X` 同号发布为 `map -> base_link.translation.x`。
- `robot_home_tf.yaml` 的 `cabin_to_map_sign` 当前配置为 `x: 1.0, y: 1.0, z: 1.0`；`robot_tf_broadcaster` 默认值也同步回到 X 不翻转。
- 前端三维仍只显示 TF 结果，不在前端单独镜像全局 X。

### Codex 会话压缩策略

- 用户最新口径：超大 Codex 会话的原始内容要归档，但 `~/.codex/sessions` 里的会话文件不要移动；存活会话列表里保留自动总结后的上下文摘要替身。
- `scripts/codex_session_guard.py summarize` 对超过 100MB 的活跃会话执行压缩：先把原始完整 JSONL 内容复制到 `~/.codex/archived_sessions/oversized/`，再把 `~/.codex/sessions` 原路径内容改写成小型摘要 JSONL，同时在 `~/.codex/session_summaries/oversized/` 写 Markdown 摘要。
- 摘要替身现在保留原始 `session_meta` 关键字段，并把首条真实用户请求放在摘要正文前面作为标题锚点，避免历史会话列表标题被摘要说明覆盖；`repair-summaries --apply` 可从 archive 原文修复已有摘要替身。
- `scripts/install_codex_session_summary_timer.sh` 安装用户级 `tie-codex-session-summary.timer`，每 15 分钟自动压缩超过 100MB、闲置至少 10 分钟且未被打开的会话。
- 旧 `scripts/install_codex_session_guard_timer.sh` 仍默认拒绝安装纯归档 timer，避免只移走活跃入口而不留下摘要替身；只有用户明确设置 `ALLOW_CODEX_SESSION_ARCHIVE_TIMER=1` 时才允许恢复。

## 2026-04-29

### slam/v30 离线复现发布包

- 新增 `docs/releases/slam_v30/`，作为 `slam/v30` 的交接、发布清单、校验文件和视觉模态样例目录。
- 新增 `src/tie_robot_perception/tools/export_visual_modalities_snapshot.py`，可从当前 ROS 运行态导出相机、世界坐标、TF、前端状态和 `pointAI/PR-FPRG` 结果到小型 bag、PNG、NPY 与 metadata。
- 新增 `src/tie_robot_bringup/launch/slam_v30_offline_visual_replay.launch`，用于在无机器环境下通过 `rosbag play --clock --loop` 复现当前视觉测试场景。
- `slam/v30` 样例 bag 已捕获 `/Scepter/*` 核心图像、`/Scepter/worldCoord/*`、`/pointAI/result_image_raw`、`/perception/lashing/result_image`、`/perception/lashing/points_camera`、`/perception/lashing/workspace/quad_pixels`、`/tf`、`/tf_static` 和吊篮/末端状态。
- 修复 `pointAINode` 运行态 `PR-FPRG` 方法绑定缺口，确保 `/perception/lashing/recognize_once` 可以真实跑通 `manual workspace S2`，而不是在服务调用时才暴露缺失属性。
- `suoquNode` 调用末端绑扎执行改为 `/moduan/execute_bind_points` action 客户端，取代旧 `/moduan/sg_precomputed*` service 调用路径，并补齐 `tie_robot_process` 的 `actionlib` 依赖。
- 新增 `/reports/pr_fprg_curve_3456` 静态实验报告，独立比较 PR-FPRG 方案 3/4/5/6 对梁筋点级过滤和地板缝曲线牵引的表现；运行主链仍保持方案 1，不把曲线方案直接切入现场默认链路。

### 视觉主方案切换为行/列峰值 PR-FPRG

- `pointAI` 的主视觉链路统一切到行/列峰值 `PR-FPRG`：手动工作区透视展开后，固定在 rectified 图的行、列 profile 上做峰值识别，输出 0/90 度正交网格；方案2废弃，曲线方案3-6只保留为后续收束/改进报告。
- `PROCESS_IMAGE_MODE_SCAN_ONLY`、执行微调和绑扎检查等 `process_image` 入口都先运行 `run_manual_workspace_s2_pipeline(publish=True)`；旧 `pre_img()` 不再作为运行门控或回退路径。
- 旧 `RANSAC + Hough + pre_img` 绑扎点识别代码已归档到 `docs/archive/legacy_ransac_hough_pointai/`，active `pointai` package 中删除 `matrix_preprocess.py`，`processor.py` 不再绑定 `cls.pre_img`。
- 当前 PR-FPRG 结果继续发布 `/pointAI/manual_workspace_s2_points`、`/coordinate_point`、`/pointAI/result_image_raw`，并以 `pr_fprg_bind_point_*` 作为相机原始坐标 TF child 前缀。
- 上游点云/世界坐标里的 PCL `SAC_RANSAC` 平面处理仍属于相机世界坐标生成链路，不等同于已归档的旧 pointAI RANSAC+Hough 绑扎点识别。

### 驱动守护与后端解耦

- `run.launch` 不再直接拉起硬件驱动；索驱、线性模组、相机分别拆成 `driver_suoqu.launch`、`driver_moduan.launch`、`driver_camera.launch`，`driver_stack.launch` 仅作为兼容聚合入口。
- 新增三个独立 systemd 守护：`tie-robot-driver-suoqu.service`、`tie-robot-driver-moduan.service`、`tie-robot-driver-camera.service`，均使用 `Restart=always`，驱动进程崩溃后由 systemd 自动拉起。
- 前端系统控制改为通过本机 HTTP 接口调用受限 sudoers 的 `systemctl start/stop/restart`，支持总驱动栈和单个驱动分别启停。
- `install_frontend_autostart.sh` 会同时安装前端、rosbridge、驱动和后端控制 unit；其中驱动 unit 设置为开机自启，ROS 后端 unit 仍不默认开机启动。
- `api.launch` 不再包含相机 SDK launch，避免 ROS 后端启动时因为相机驱动重名造成节点互踢；相机属于独立驱动守护。
- 相机驱动运行中连续取帧失败会主动退出，交给 launch respawn 与 `tie-robot-driver-camera.service` 重新拉起并等待设备恢复。
- 新增 `tf_stack.launch` 并由 `rosbridge_stack.launch` 包含，TF 层跟随 `tie-robot-rosbridge.service` 常驻守护，不再绑定索驱或相机驱动生命周期。
- `robot_tf_broadcaster.py` 订阅 `/cabin/cabin_data_upload` 连续发布 `map -> base_link -> Scepter_depth_frame`；索驱状态断流时保留最后位姿继续发布，等待驱动恢复后自动接上。
- `gripper_tf_broadcaster.py` 从相机驱动 launch 移到 TF 栈，继续负责 `Scepter_depth_frame -> gripper_frame`，避免相机驱动重启时打断前端 TF 链。
- `web_action_bridge_node` 和 `system_log_mux` 跟随 `rosbridge_stack.launch` 常驻守护，`run.launch` 收口为纯算法后端，避免后端未启动时前端 Action 话题只有 rosbridge 发布却没有服务端订阅。
- 驱动与后端 systemd unit 启动前等待 `tie-robot-rosbridge.service` 提供的本机 ROS master 可用，避免索驱、线模或相机驱动抢先拉起自己的 roscore，造成 TF/节点重名互踢。

### ROS 包结构规范化

- 非 ROS 节点的辅助工具从 `scripts/` 移到 `tools/`：`pr_fprg_peak_supported_probe.py` 和 `run_gitnexus_local_webui.py` 不再占用 catkin 可执行节点目录。
- C++ 公共头文件统一收进 `include/<package_name>/`，避免多个包的 `common.hpp/json.hpp` 在 include 路径中互相串包。
- `tie_robot_control` 补齐 `INCLUDE_DIRS include`、头文件安装和 `moduan*` C++ 节点安装规则，`tie_robot_web` 补齐头文件导出与安装规则。
- `tie_robot_web` 的 launch 入口切到标准 lower_snake_case：`web_action_bridge_node`；旧 `webActionBridgeNode` 仍保留为兼容可执行目标。
- 新增架构测试约束：`scripts/` 只保留 ROS 可执行入口或安装脚本，C++ 包的公共头文件必须走包名命名空间并可安装。
- `run.launch` 保持纯算法栈入口；索驱、线模、相机由 `driver_stack.launch` 及独立 driver systemd 服务管理，TF 层、Web Action/Service 桥接和日志汇总由 `rosbridge_stack.launch` 守护，避免一键 Web 后端启动时抢占硬件驱动。

### Agent 共享记忆

- 新增 `docs/agent_memory/` 作为跨 Codex 会话、跨 agent 的共享工程记忆目录。
- 根 `AGENTS.md` 的开工必读顺序扩展为：`README.md`、`CHANGELOG.md`、`docs/agent_memory/README.md`、`docs/agent_memory/current.md`。
- 根 `AGENTS.md` 新增 Codex 会话启动协议：利用 Codex 自动注入 `AGENTS.md` 的特性，要求每次新会话先读取共享记忆系统，并可通过 `codex debug prompt-input` 验证。
- 新增 `docs/agent_memory/organism.md`，把“本工程目录下的 Codex 是一个有机体”定义为感知、记忆、免疫、生长的工程协作闭环。
- 新增 `docs/agent_memory/codex_local_setup.md`，说明通过 `codex -C /home/hyq-/simple_lashingrobot_ws` 和 `codex debug prompt-input` 确认 Codex 从正确目录启动。
- 新增 `docs/agent_memory/power_loss_recovery.md` 和 `docs/agent_memory/checkpoint.md`，用于突然断电或会话丢失后的恢复入口。
- 新增 `scripts/agent_memory.py`，支持 `add`、`refresh`、`check`、`checkpoint`、`recover` 命令，用于追加会话记忆、刷新当前快照、校验记忆契约、写入恢复点和读取恢复报告。
- 新增“示例泛化原则”：用户说“比如”“例如”“类似”时，后续内容默认视为意图线索和启发样例；agent 应根据实际工程状态举一反三，避免机械照搬，除非用户明确要求严格按例子执行。
- 新增“轻启动”和“长上下文瘦身原则”：新 Codex 会话默认只读 `README.md`、`CHANGELOG.md`、`docs/agent_memory/current.md`，其他记忆文档按主题扩展读取；遇到长日志、整仓状态或大型调试目录时先摘要和限域查询，避免长上下文卡顿。
- 新增 `scripts/codex_session_guard.py`，用于扫描和归档 `~/.codex/sessions` 中过大的活跃 Codex JSONL 会话，避免历史会话列表或恢复器因直接加载百 MB 级会话而打不开。
- 新增 `scripts/install_codex_session_guard_timer.sh`，用于安装用户级 `tie-codex-session-guard.timer`，自动周期性归档超过 50MB、闲置至少 60 分钟且未被打开的 Codex 会话。
- 后续会话如果产生关键工程知识、架构决策、避坑经验或跨会话必须继承的修改，应写入共享记忆，避免只停留在单个 agent 的私有上下文里。

## 2026-04-23

### Codex 执行约定

- 开工前先阅读根目录 `README.md` 和本文件。
- 涉及 `src/tie_robot_web/frontend` 的改动，优先修改源码，再按需执行 `npm run build` 同步 `src/tie_robot_web/web`。
- 未经用户明确要求，不要恢复控制面板按钮上方的任务提示框。

### 新前端本次调整

- 控制面板顶部旧任务文案已移除，不再显示原先的旧标题提示。
- 控制面板任务按钮上方的提示框已删除，页面只保留任务按钮本体。
- 前端内部仍保留 `setControlFeedback()` 这个接口，但当前实现为空，用来兼容已有控制器回调，避免为了删除提示框而牵连任务流程。
- 清理前端产物时，若发现旧的 hash 静态资源仍包含已经删除的 UI，请在重新构建后删除这些过期产物，避免搜索结果和页面缓存混淆。
- 3D 视图中的蓝色大方块底面中心应对齐 `base_link`，橙色 TCP 方框最底面中心应对齐 `gripper_frame`；不要再把蓝色块额外抬到 TCP 上方。
- 3D 视图中的相机框、TCP 框及其坐标轴采用项目自定义显示坐标系：位置继续跟随 TF，但显示姿态固定为 `x+`、`y+` 与全局地面坐标一致，`z+` 朝向地面；这是前端显示约定，不要再按标准右手系四元数去纠正它。
- 相机-TCP 外参现在按“怎么标就怎么发”的口径处理：`translation_mm.x/y/z` 都不再在 `gripper_tf_broadcaster` 里取反，前端外参面板从 TF 回填时也直接显示发布值；对应地，前后端点云换算统一走 TF 链。
- `map -> Scepter_depth_frame` 现在真实发布为绕 `x` 轴 `180°` 的相机朝下坐标系，这样 `translation_mm.z > 0` 会把 TCP 放到相机 `z+` 方向；前端相机跟随视角不再直接套真实四元数，避免视角跟着翻到地面下方。
- `PROCESS_IMAGE_MODE_SCAN_ONLY` 下的 S2 现在直接轮询 `PR-FPRG` 手动工作区链路，不再先通过 `pre_img()` 出点后才放行，也不再把旧视觉出点结果当成扫描模式的默认回退；2026-04-29 后旧 `pre_img()` 已归档，不要把它恢复为运行门控。
- `manual workspace S2` 和扫描链 `PR-FPRG` 的结果点现在除了继续发布 `/pointAI/manual_workspace_s2_points`，还会同步发布到 `/coordinate_point`，以便新前端 3D Scene 继续沿用原有绑扎点显示链路。
- 视觉节点入口按标准 ROS/Python 包结构收口：ROS executable 从 `pointAI.py` 改名为 `pointai_node.py`，真实节点实现迁入 `tie_robot_perception.pointai.node`，ROS 接口与诊断分别迁入 `ros_interfaces.py` 和 `diagnostics.py`。

### 当前 S2 方案命名

- 当前认可的手动工作区 `S2` 统一命名为：`PR-FPRG 透视展开频相回归网格方案`
- 英文代号：`PR-FPRG`
- 后续涉及 `manual workspace S2` 的修改、回归和交接，统一按这套名称引用
- 详细知识条目见：
  - `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
