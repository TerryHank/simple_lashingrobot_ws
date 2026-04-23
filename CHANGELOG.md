# 项目全局改动日志

本文档记录 `simple_lashingrobot_ws` 的项目级变更约定和近期关键调整。  
开始修改代码前，先读最新日期的记录，再进入具体包目录。

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
- 相机-TCP 外参现在按“怎么标就怎么发”的口径处理：`translation_mm.z` 不再在 `gripper_tf_broadcaster` 里取反，前端外参面板从 TF 回填时也直接显示正 `z`；对应地，前后端点云换算统一改为 `cabin_z = cabin_tf_z - depth_z + gripper_tf_z`。
- `cabin_frame -> Scepter_depth_frame` 现在真实发布为绕 `x` 轴 `180°` 的相机朝下坐标系，这样 `translation_mm.z > 0` 会把 TCP 放到相机 `z+` 方向；前端相机跟随视角不再直接套真实四元数，避免视角跟着翻到地面下方。
- `PROCESS_IMAGE_MODE_SCAN_ONLY` 下的 S2 现在直接轮询 `PR-FPRG` 手动工作区链路，不再先通过 `pre_img()` 出点后才放行，也不再把旧视觉出点结果当成扫描模式的默认回退；除非用户再次明确要求，否则不要恢复这层门控。
- `manual workspace S2` 和扫描链 `PR-FPRG` 的结果点现在除了继续发布 `/pointAI/manual_workspace_s2_points`，还会同步发布到 `/coordinate_point`，以便新前端 3D Scene 继续沿用原有绑扎点显示链路。

### 当前 S2 方案命名

- 当前认可的手动工作区 `S2` 统一命名为：`PR-FPRG 透视展开频相回归网格方案`
- 英文代号：`PR-FPRG`
- 后续涉及 `manual workspace S2` 的修改、回归和交接，统一按这套名称引用
- 详细知识条目见：
  - `docs/handoff/2026-04-23_pr_fprg_knowledge.md`
