# 项目全局改动日志

本文档记录 `simple_lashingrobot_ws` 的项目级变更约定和近期关键调整。  
开始修改代码前，先读最新日期的记录，再进入具体包目录。

## 2026-04-29

### slam/v30 离线复现发布包

- 新增 `docs/releases/slam_v30/`，作为 `slam/v30` 的交接、发布清单、校验文件和视觉模态样例目录。
- 新增 `src/tie_robot_perception/tools/export_visual_modalities_snapshot.py`，可从当前 ROS 运行态导出相机、世界坐标、TF、前端状态和 `pointAI/PR-FPRG` 结果到小型 bag、PNG、NPY 与 metadata。
- 新增 `src/tie_robot_bringup/launch/slam_v30_offline_visual_replay.launch`，用于在无机器环境下通过 `rosbag play --clock --loop` 复现当前视觉测试场景。
- `slam/v30` 样例 bag 已捕获 `/Scepter/*` 核心图像、`/Scepter/worldCoord/*`、`/pointAI/result_image_raw`、`/perception/lashing/result_image`、`/perception/lashing/points_camera`、`/perception/lashing/workspace/quad_pixels`、`/tf`、`/tf_static` 和吊篮/末端状态。
- 修复 `pointAINode` 运行态 `PR-FPRG` 方法绑定缺口，确保 `/perception/lashing/recognize_once` 可以真实跑通 `manual workspace S2`，而不是在服务调用时才暴露缺失属性。
- `suoquNode` 调用末端绑扎执行改为 `/moduan/execute_bind_points` action 客户端，取代旧 `/moduan/sg_precomputed*` service 调用路径，并补齐 `tie_robot_process` 的 `actionlib` 依赖。
- 新增 `/reports/pr_fprg_curve_3456` 静态实验报告，独立比较 PR-FPRG 方案 3/4/5/6 对梁筋点级过滤和地板缝曲线牵引的表现；运行主链仍保持方案 1，不把曲线方案直接切入现场默认链路。

### 视觉主方案切换为方向自适应 PR-FPRG

- `pointAI` 的主视觉链路统一切到方向自适应 `PR-FPRG`：手动工作区透视展开后，不再假设钢筋一定与画面水平/垂直，而是在 rectified 平面内扫描两组 `theta/rho` 周期线族，再做峰值支撑、连续钢筋条验证、主间距兜底和线族求交。
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
- 3D 视图中的蓝色大方块中心应对齐 `Scepter_depth_frame`，橙色 TCP 方框中心应对齐 `gripper_frame`；不要再把蓝色块额外抬到 TCP 上方。
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
