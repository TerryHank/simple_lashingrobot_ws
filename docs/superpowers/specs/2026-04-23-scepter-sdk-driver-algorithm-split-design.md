# Scepter 相机 SDK 驱动层与视觉算法层拆分设计

## 背景

当前工程里实际运行的相机代码位于 [tie_robot_perception/src/camera](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/camera)。这套代码虽然已经迁入新包结构，但内部仍然混合了两类职责：

- 相机底层驱动职责：设备发现、建连、拉流、动态参数、基础图像与点云发布。
- 视觉算法职责：世界坐标图生成、原始世界坐标图发布、平面去除、RANSAC、像素深度转世界坐标服务。

用户给出的两份参考代码也证明了这一点：

- 原厂 SDK：`/home/hyq-/simple_lashingrobot_ws/ScepterSDK/3rd-PartyPlugin/ROS/src/ScepterROS`
- 二次开发 SDK：`/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/src/ScepterROS`

对比结果表明，当前工程中的 [scepter_manager.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/camera/scepter_manager.cpp) 更接近“二次开发 SDK”，而不是原厂“纯驱动”版本。额外混入的内容主要包括：

- PCL / RANSAC / 平面剔除
- `/Scepter/worldCoord/world_coord` 与 `/Scepter/worldCoord/raw_world_coord` 生成
- `convert_depth_to_point_cloud` 服务
- 若干面向算法效果的处理逻辑

这与当前工程分层目标不一致：相机 SDK 应当只承担硬件接入与基础数据输出，新增算法应该进入视觉算法层。

## 问题定义

当前混层带来 4 个直接问题：

1. 相机 SDK 不再是“可替换的底层驱动”，而变成“驱动 + 算法混合节点”。
2. 上层视觉算法 `pointAI.py` 对相机节点内部实现细节产生耦合，而不是对稳定的视觉中间产物耦合。
3. 后续若要替换相机厂商 SDK，必须同时迁移算法逻辑，风险偏大。
4. CMake 和依赖被相机驱动层牵入 PCL、分割、点云后处理等非纯驱动能力，层级变脏。

## 目标

本次拆分的目标如下：

- 让 `tie_robot_perception/camera` 回归“纯相机底层驱动”。
- 将二次开发新增的算法能力迁入 `tie_robot_perception` 的视觉算法层。
- 尽量保持 `pointAI.py` 现有输入话题口径不变，减少上层联动修改。
- 保持工程内 SDK 依赖完全本地化，不重新依赖工作区外的 `/home/hyq-/ScepterSDK`。

## 非目标

本次不处理以下范围：

- 不改 `pointAI.py` 的业务识别算法口径。
- 不改全局消息包结构，不顺手重构所有 perception 接口。
- 不替换当前 Scepter 厂商 SDK 头文件和动态库版本。
- 不把所有视觉算法都改成插件式加载，本次先完成明确分层。

## 现状对比

### 原厂 SDK

原厂版本的 `ScepterROS` 更接近纯驱动模型，核心特征是：

- 负责相机设备建连、拉流、发布图像与基础点云。
- 不包含额外的 PCL / RANSAC 处理。
- 不生成 `worldCoord` 图像。
- 不提供像素深度转世界坐标的业务服务。

### 二次开发 SDK

用户自建版本在原厂 SDK 基础上加入了视觉算法能力，典型特征是：

- 引入 PCL、`SACSegmentation`、`ExtractIndices`
- 新增 `worldCoord` 相关 publisher
- 新增 `convert_depth_to_point_cloud` 服务
- 在相机节点内部完成世界坐标图和去平面处理

### 当前工程

当前工程的 [scepter_manager.hpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/include/tie_robot_perception/camera/scepter_manager.hpp) 与 [scepter_manager.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/camera/scepter_manager.cpp) 仍保留了上述二次开发能力，因此还不符合“驱动层只做底层接入”的边界。

## 方案对比

### 方案 A：SDK 纯驱动化，算法迁到独立视觉节点

做法：

- 相机 SDK 仅保留设备连接、拉流、基础图像与基础点云输出。
- 新增独立视觉算法节点，订阅基础点云或深度数据，生成 `worldCoord` 与转换服务。
- 上层 `pointAI.py` 保持对现有 `worldCoord` 话题的依赖，只改变其数据生产者。

优点：

- 分层最清楚。
- 上层接口改动最小。
- 符合当前工程“包分层、职责单一”的目标。

缺点：

- 需要新增一个过渡算法节点。
- 需要重新梳理相机层和算法层之间的数据约定。

### 方案 B：仅移出 PCL / RANSAC，保留 worldCoord 生成在 SDK

优点：

- 代码改动最少。

缺点：

- SDK 里仍然留有明显的算法职责。
- 结构上没有真正解决“驱动层变成感知层”的问题。

### 方案 C：SDK 回到原厂口径，上层直接吃深度图自算

优点：

- 驱动层最纯。

缺点：

- `pointAI.py` 和上层链路改动面最大。
- 当前系统风险最高，不适合先做。

## 选型

采用方案 A。

原因是它在“驱动层纯净化”和“现有上层链路稳定性”之间取得了最好平衡：

- 下层回到纯驱动。
- 算法职责进入视觉层。
- 上层 `pointAI.py` 可以先保持现有订阅口径。

## 目标结构

### 驱动层

保留在：

- [tie_robot_perception/include/tie_robot_perception/camera](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/include/tie_robot_perception/camera)
- [tie_robot_perception/src/camera](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/camera)

职责：

- Scepter 相机设备发现、打开、关闭、异常处理
- 深度 / IR / 彩色 / 对齐图像发布
- 基础点云发布
- 厂商 SDK 原生参数与硬件滤波控制
- 相机内参与标定信息发布

明确不再保留：

- `worldCoord` 图像生成
- 原始世界坐标图生成
- PCL / RANSAC / 平面剔除
- `convert_depth_to_point_cloud` 服务

### 视觉算法层

保留在：

- [tie_robot_perception/src/tie_robot_perception/perception](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/tie_robot_perception/perception)
- [tie_robot_perception/src/perception](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/perception)

新增一个独立视觉中间节点，暂定命名为 `scepter_world_coord_processor`。

职责：

- 订阅相机驱动发布的基础点云
- 构建 `world_coord` 和 `raw_world_coord`
- 承接现有 RANSAC / 去平面 / 点云二次处理
- 对外继续发布：
  - `/Scepter/worldCoord/world_coord`
  - `/Scepter/worldCoord/raw_world_coord`
  - `/Scepter/worldCoord/camera_info`
- 对外继续提供 `convert_depth_to_point_cloud` 服务

## 数据流

拆分后的数据流如下：

```text
Scepter 相机硬件
  -> scepter_camera
     -> /Scepter/depth/image_raw
     -> /Scepter/color/image_raw
     -> /Scepter/ir/image_raw
     -> /Scepter/depthCloudPoint/cloud_points
     -> /Scepter/depth2colorCloudPoint/cloud_points

/Scepter/depthCloudPoint/cloud_points
  -> scepter_world_coord_processor
     -> /Scepter/worldCoord/world_coord
     -> /Scepter/worldCoord/raw_world_coord
     -> /Scepter/worldCoord/camera_info
     -> /Scepter/convert_depth_to_point_cloud

/Scepter/worldCoord/*
  -> pointAI.py
```

## 接口兼容策略

本次采用“上层话题兼容、内部生产者迁移”的策略。

兼容要求如下：

1. `pointAI.py` 当前订阅的 `/Scepter/worldCoord/world_coord` 和 `/Scepter/worldCoord/raw_world_coord` 继续保留。
2. 这些话题的发布者从 `scepter_camera` 切换为新算法节点。
3. `convert_depth_to_point_cloud` 服务名继续保留，调用方不需要同步修改。
4. `depthCloudPoint` 和 `depth2colorCloudPoint` 仍由驱动层发布，但语义收敛为基础点云，不再承诺内部已做算法后处理。

## 代码迁移边界

### 从相机驱动层移出的代码

以下能力必须移出 [scepter_manager.cpp](/home/hyq-/simple_lashingrobot_ws/src/tie_robot_perception/src/camera/scepter_manager.cpp)：

- PCL 相关 `#include`
- `publishCloudPoint(...)` 内的 RANSAC / 去平面处理
- `worldCoord` 相关 publisher 与 camera info publisher
- `convertDepthToPointCloud(...)` 服务实现
- `convertDepth_service_nh_`、`worldCoord_nh_` 等仅算法层需要的 NodeHandle

### 可继续留在相机驱动层的代码

以下内容继续留在驱动层：

- `Scepter_api` 设备控制
- 原始图像帧转换
- 基础点云生成与发布
- 硬件滤波开关和原厂参数回调

这里的判断标准是：只要某项能力直接调用厂商 SDK 完成硬件控制或基础数据产出，就属于驱动层；只要某项能力开始对基础数据做理解、筛选、重建或业务化转换，就属于算法层。

## 构建与依赖调整

驱动层拆净后，`tie_robot_perception` 仍然保留一个包，但内部构建会分成两个可执行单元：

- `scepter_camera`：纯驱动可执行程序
- `scepter_world_coord_processor`：视觉算法中间节点

依赖调整目标：

- `scepter_camera` 不再链接 PCL
- PCL 只由 `scepter_world_coord_processor` 使用
- `ConvertDepthToPointCloud.srv` 先保留在 `tie_robot_perception`，避免本次顺手扩大接口迁移范围

## 启动与运行

`launch` 层需要从“只启动 `scepter_camera`”改成“同时启动驱动节点和算法节点”。新的启动约束为：

- 先启动 `scepter_camera`
- 再启动 `scepter_world_coord_processor`
- `pointAI.py` 不要求修改启动顺序，只要最终 `worldCoord` 话题存在即可

## 测试策略

本次迁移必须覆盖以下验证：

1. 构建验证
   - `scepter_camera` 可独立编译
   - `scepter_world_coord_processor` 可独立编译
2. 分层验证
   - `scepter_camera` 源码不再包含 PCL / RANSAC / worldCoord 服务实现
3. 接口兼容验证
   - `pointAI.py` 仍订阅原有 `worldCoord` 话题名
   - 新算法节点成功发布这些话题
4. 回归验证
   - `convert_depth_to_point_cloud` 服务仍可返回世界坐标
   - 现有基于 `worldCoord` 的识别链不因发布者切换而断裂

## 风险与缓解

### 风险 1：基础点云与原先 worldCoord 图像语义不完全一致

缓解：

- 新算法节点先复用当前 `publishCloudPoint(...)` 的世界坐标构造逻辑，保证结果口径一致，再逐步清理内部实现。

### 风险 2：`pointAI.py` 依赖了某些隐含时序

缓解：

- 保持话题名不变。
- 保持 `world_coord` 和 `raw_world_coord` 同时发布。
- 必要时增加节点就绪日志，便于现场定位。

### 风险 3：本次顺手扩大到接口大迁移

缓解：

- 明确把“消息 / 服务进一步抽到全局接口包”列为后续优化，不并入本次。
- 本次只做“驱动纯化 + 算法迁移 + 现有接口兼容”。

## 落地原则

本次实施遵循 3 条原则：

1. 先保证分层正确，再考虑进一步抽象。
2. 先保持上层话题兼容，再逐步清理历史耦合。
3. 相机 SDK 目录中的代码只保留“没有业务语义的底层相机能力”。
