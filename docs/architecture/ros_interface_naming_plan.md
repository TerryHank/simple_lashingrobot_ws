# ROS 接口命名与分层迁移规划

**日期：** 2026-04-29
**状态：** 规划已确认，代码迁移按本文分阶段执行
**范围：** 全工程 ROS Topic、Service、Action、前端 ROS registry、launch 与节点内硬编码接口名

## 1. 背景

当前工程已经拆成 `tie_robot_hw`、`tie_robot_perception`、`tie_robot_control`、`tie_robot_process`、`tie_robot_web` 等主包，但 ROS 接口命名仍保留了历史混合语义：

- `/pointAI/*` 同时表示视觉算法、工作区调参、结果图、前端触发入口。
- `/web/moduan/*` 同时承载末端控制、急停、流程跳点、保存数据、灯光等不同责任层。
- `/cabin/*` 同时包含索驱驱动原子服务、控制服务、流程服务和前端长任务入口。
- 视觉节点会监听 `/web/moduan/send_odd_points`，流程节点也会监听多个 `/web/moduan/*`，导致前端协议直接穿透到算法和流程内部。
- `Topic`、`Service`、`Action` 的使用边界不够清晰，耗时任务、短请求和持续流混在一起。

本轮目标是先把“名字和责任层”固定下来，再按兼容迁移方式逐步改代码，避免一次性大改导致现场联调不可控。

## 2. 命名原则

ROS 名称必须先表达责任层，再表达实体，再表达动作或数据：

```text
/<layer>/<entity>/<capability_or_stream>
```

推荐层级：

| 层级 | 责任 | 示例 |
| --- | --- | --- |
| `/hw` | 硬件驱动层原子能力，只做设备帧、寄存器、TCP/Modbus/SDK 原子读写 | `/hw/cabin/move_raw` |
| `/perception` | 感知与视觉算法层，只做图像处理、世界点转换、PR-FPRG 识别和结果发布 | `/perception/lashing/recognize_once` |
| `/control` | 控制层短动作封装，负责等待到位、速度、单轴/末端控制、预计算点执行 | `/control/linear_module/move_once` |
| `/process` | 流程编排层，负责扫描、识别位姿、全局作业、单点绑扎流程 | `/process/lashing/start_global_work` |
| `/calibration` | 标定入口 | `/calibration/gripper_tf/set` |
| `/system` | 系统守护、日志、栈启停 | `/system/stack/driver/start` |
| `/safety` | 急停、暂停、人工介入等跨设备安全状态 | `/safety/forced_stop/set` |
| `/web` | 只允许保留 Web 桥自身的内部兼容入口，不再表达硬件或算法业务语义 | 兼容期 alias |

保留 ROS 标准名：

- `/tf`
- `/tf_static`
- `/diagnostics`
- `/rosout` / `/rosout_agg`

## 3. Topic、Service、Action 边界

| 类型 | 使用场景 | 禁止场景 |
| --- | --- | --- |
| `Topic` | 图像流、点云、状态、进度、Marker、结果图、持续开关状态广播 | 前端点击后要求明确成功/失败的短请求 |
| `Service` | 单次视觉识别、设置参数、移动一次、启停驱动、设置工作区等短请求 | 扫描建图、全局作业、连续绑扎这类耗时任务 |
| `Action` | 扫描、识别位姿、全局作业、账本测试、单点绑扎等需要进度/可取消的任务 | 高频状态流、图像流 |

## 4. 视觉重新拆分

视觉按“相机原子层”和“算法层”拆：

### 4.1 相机驱动原子层

目标归属：`tie_robot_hw`

只负责：

- Scepter SDK 连接与重连。
- RGB / IR / Depth / CameraInfo 原始流发布。
- 相机驱动状态与诊断。
- 不做 PR-FPRG，不保存绑扎点 JSON，不触发流程动作。

目标接口：

```text
/hw/camera/scepter/rgb/image_raw
/hw/camera/scepter/rgb/camera_info
/hw/camera/scepter/ir/image_raw
/hw/camera/scepter/ir/camera_info
/hw/camera/scepter/depth/image_raw
/hw/camera/scepter/depth/camera_info
/hw/camera/scepter/status
```

### 4.2 相机数据处理层

目标归属：`tie_robot_perception`

负责深度图转相机坐标/世界点图，但不负责绑扎点识别：

```text
/perception/camera/scepter/world_coord/raw_image
/perception/camera/scepter/world_coord/filtered_image
/perception/camera/scepter/world_coord/camera_info
/perception/camera/scepter/convert_depth
```

### 4.3 绑扎点视觉算法层

目标归属：`tie_robot_perception`

只负责 PR-FPRG 识别和结果发布：

```text
/perception/lashing/set_workspace
/perception/lashing/config/set_stable_frame_count
/perception/lashing/config/set_height_threshold
/perception/lashing/recognize_once
/perception/lashing/points_camera
/perception/lashing/points_world
/perception/lashing/workspace/quad_pixels
/perception/lashing/result_image
/perception/lashing/debug/cropped_ir_image
/perception/lashing/debug/cropped_rgb_image
/perception/lashing/debug/cropped_depth_image
```

约束：

- “触发视觉识别”只调用 `/perception/lashing/recognize_once`。
- 普通点位单测触发视觉识别时，不写本地绑扎点 JSON。
- 只有 `/process/scan/start` 或 `/process/pose_recognition/start` 这类识别位姿流程允许写入扫描/绑扎点 JSON。
- 前端显示识别结果只消费 `/perception/lashing/result_image`，清除识别结果后回到当前图层原始流。
- 前端不再查找线话题自行叠线；算法层直接发布已经叠加线条和点的结果图。

## 5. 控制与流程拆分

### 5.1 索驱

驱动原子层：

```text
/hw/cabin/driver/start
/hw/cabin/driver/stop
/hw/cabin/driver/restart
/hw/cabin/state
/hw/cabin/move_raw
/hw/cabin/stop
```

控制层：

```text
/control/cabin/move_once
/control/cabin/set_speed
```

流程层：

```text
/process/scan/start
/process/scan/markers
/process/lashing/global_bind_path
/process/lashing/area_progress
/process/lashing/start_global_work
/process/lashing/run_bind_path_direct_test
/process/lashing/bind_current_area
```

### 5.2 末端线性模组与绑扎工具

驱动原子层：

```text
/hw/linear_module/driver/start
/hw/linear_module/driver/stop
/hw/linear_module/driver/restart
/hw/linear_module/state
/hw/linear_module/execute_points_raw
/hw/binding_tool/status
/hw/binding_tool/light/set
```

控制层：

```text
/control/linear_module/move_once
/control/linear_module/home
/control/linear_module/set_speed
/control/lashing/execute_points
/control/lashing/execute_points_fast
/control/binding_tool/set_enabled
```

流程层：

```text
/process/lashing/bind_single_point
/process/lashing/save_bind_data
/process/lashing/checkerboard_mode/set
```

### 5.3 安全

```text
/safety/forced_stop/set
/safety/forced_stop/state
/safety/pause/set
/safety/pause/state
/safety/manual_intervention/set
/safety/manual_intervention/state
```

## 6. Web 与系统层

前端源码只引用 `src/tie_robot_web/frontend/src/config/topicRegistry.js` 中的注册表。迁移期该注册表允许同时保存 canonical name 和 legacy alias，但业务代码不再散落硬编码。

系统控制：

```text
/system/stack/driver/start
/system/stack/driver/restart
/system/stack/algorithm/start
/system/stack/algorithm/restart
/system/stack/ros/restart
```

日志：

```text
/system/log/all
/system/log/cabin
/system/log/camera
/system/log/linear_module
```

标定：

```text
/calibration/gripper_tf/set_offset
/calibration/gripper_tf/set
```

## 7. 兼容迁移策略

不采用一次性全工程替换。迁移分 5 个阶段：

1. **接口表落地**
   保留 `docs/architecture/ros_interface_migration_map.yaml` 作为旧新映射源。

2. **新增 canonical 接口 alias**
   在现有节点上同时 advertise/subscribe 新接口和旧接口。旧接口只作为 alias，不再新增业务逻辑。

3. **调用方切换**
   先切流程层和控制层，再切前端 registry，最后清理算法层内部旧名依赖。

4. **静态检查**
   增加脚本扫描源码、launch、前端，禁止新代码直接引用旧名。允许旧名只存在于 alias 文件、迁移表和测试白名单。

5. **删除旧接口**
   现场确认稳定后，删除 `/pointAI/*`、`/web/moduan/*`、`/moduan_work`、`/coordinate_point` 等旧名。

## 8. 验收标准

- 前端点击“触发视觉识别”调用 `/perception/lashing/recognize_once`，只更新当前图层结果图，不触发绑扎，不写扫描 JSON。
- 前端点击“触发单点绑扎”调用 `/process/lashing/bind_single_point`，由流程层先触发视觉，再调用控制层绑扎。
- PR-FPRG 结果图由后端直接生成并发布到 `/perception/lashing/result_image`。
- 流程层不再监听 `/web/moduan/*`。
- 视觉算法层不再监听 `/web/moduan/*`。
- 驱动层不出现扫描、PR-FPRG、全局作业、绑扎流程语义。
- 除兼容 alias 外，源码中不再新增 `/pointAI`、`/web/pointAI`、`/web/moduan`、`/moduan_work`、`/coordinate_point`。
- `catkin_make`、前端 `npm run build`、接口静态检查通过。
