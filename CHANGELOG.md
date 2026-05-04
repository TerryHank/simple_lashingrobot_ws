# 项目全局改动日志

本文档记录 `simple_lashingrobot_ws` 的项目级变更约定和近期关键调整。  
开始修改代码前，先读最新日期的记录，再进入具体包目录。

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
- 扫描 S2 与 `38baa98` 的算法差异继续收口：运行态会完整评分 `background_depth - filled_depth` 与 `filled_depth - background_depth` 两个 depth 响应变体，并按纵横周期估计总分选择最佳变体；透视展开几何优先使用 2026-04-22 口径的 `corner_world_cabin_frame`，缺失时才回退兼容当前已有的 `corner_world_camera_frame`。
- 当前扫描算法不再使用行/列峰值 line-family 主链、depth+IR 组合响应、`axis_peak_families` 日志口径、梁筋 ±13 cm 扩张过滤、稳定采样择优或 phase lock；这些实验链路只保留为报告/研究参考，不进入扫描运行路径。
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

- 用户最新口径改为不再使用旧展示转义层：旧 `20260403` 展示链进入演示时直接由旧工作目录 `roslaunch chassis_ctrl show_full.launch` 接管，`show_legacy_driver_bridge` / `tie-robot-show-legacy-shared-driver-stack.service` 不再作为演示链路的一部分。
- 新前端 header 的“演示模式”状态按钮进入演示时会确保 5173 旧前端在线，停止当前 `tie-robot-rosbridge.service`、`tie-robot-backend.service`、三个 driver service 和旧转义层服务，再启动轻量 `tie-robot-demo-rosbridge.service` 与旧工作目录 `tie-robot-demo-show-full.service`；进入后按钮变绿并打开 `http://<当前主机>:5173/`。
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

### rosbridge 重启后依赖服务跟随重启

- 现场视觉断链根因是 `tie-robot-rosbridge.service` 重启并重新拥有 ROS master 后，旧的相机、后端和末端进程仍在运行但没有重新注册到当前 master，导致 `/Scepter/ir/image_raw`、`/Scepter/depth/image_raw` 无发布者，`/pointAINode` 等节点 XML-RPC 地址拒绝连接。
- `tie-robot-backend.service` 和三个 driver service 增加 `PartOf=tie-robot-rosbridge.service`，后续 rosbridge 重启时依赖当前 ROS master 的服务会跟随重启，避免留下“进程活着但 ROS 图断链”的半断状态。
- 本机已重新安装 systemd unit，并重启 backend、camera、moduan 让节点重新注册；恢复后 `/Scepter/ir/image_raw` 约 5Hz，`/coordinate_point` 有 `/pointAINode` 发布者。

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
