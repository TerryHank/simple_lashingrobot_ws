# 统一硬件驱动包重构设计

## 1. 背景

当前工程的底层硬件驱动还处于分散状态：

- 相机驱动使用 `src/ScepterROS`，但它不是工作区内真实目录，而是一个指向外部磁盘路径的符号链接：
  - `src/ScepterROS -> /home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/src/ScepterROS`
- 索驱 TCP 驱动和线性模组 Modbus 驱动已经做了第一阶段面向对象抽象，但仍然挂在 `chassis_ctrl` 包内：
  - `src/chassis_ctrl/include/chassis_ctrl/driver/`
  - `src/chassis_ctrl/src/driver/`
- `robot_interface_hub` 已经接管了全局接口、前端和 launch 编排，因此底层硬件包现在有了继续下拆的稳定边界。

这会带来几个长期问题：

- 相机驱动不在当前工作区真实受控，迁移、打包、部署和回滚都不稳定。
- 相机、索驱、线模这三类底层 SDK 驱动不在同一包，职责边界不统一。
- `chassis_ctrl` 既承载算法/应用执行，又承载底层驱动实现，后续继续瘦身会越来越困难。
- `ScepterROS` 包名不符合 ROS 常规命名规范，大写包名在工具链里持续产生告警。

## 2. 目标

本次重构的目标是新建统一底层硬件包 `robot_hw_drivers`，把以下内容全部收口进去：

- Scepter 相机驱动源码
- Scepter 相机运行所需的二进制 SDK、配置文件和 launch
- 索驱 TCP 驱动层
- 线性模组 Modbus 驱动层

重构完成后，工程包边界收口为：

```text
robot_interface_hub
  -> 全局接口、前端、launch 编排、topic/service 网关

robot_hw_drivers
  -> 相机驱动
  -> 索驱 TCP 驱动
  -> 线性模组 Modbus 驱动
  -> 硬件本地 launch / cfg / vendor SDK

chassis_ctrl
  -> 算法层
  -> 应用执行节点
  -> 数据、配置、业务流程
```

## 3. 非目标

本次设计不处理以下事项：

- 不重写 `pointAI.py` 的视觉算法。
- 不改变新前端动作链语义。
- 不把所有硬件本地接口都迁入 `robot_interface_hub`。
- 不在本次顺手清理 `*_show.cpp` 等历史展示节点。
- 不先做“兼容双轨长期保留”；本次目标是直接完成正式迁移。

## 4. 总体方案

采用单一底层硬件包方案：

```text
robot_interface_hub
  ├─ msg/srv/action
  ├─ launch
  ├─ web
  └─ API bridge

robot_hw_drivers
  ├─ include/robot_hw_drivers/driver/
  ├─ include/robot_hw_drivers/camera/
  ├─ src/driver/
  ├─ src/camera/
  ├─ launch/
  ├─ cfg/
  ├─ srv/
  ├─ dependencies/
  └─ scripts/

chassis_ctrl
  ├─ src/suoquNode.cpp
  ├─ src/moduanNode.cpp
  ├─ scripts/pointAI.py
  └─ 算法、运行时数据和业务逻辑
```

## 5. 包职责定义

### 5.1 `robot_hw_drivers`

`robot_hw_drivers` 是统一底层硬件包，负责：

- 相机驱动可执行程序 `scepter_camera`
- 相机动态参数和硬件本地 service
- 相机运行所需 vendor SDK 与驱动动态库
- 索驱 TCP transport / protocol / driver
- 线性模组 Modbus transport / protocol / driver
- 硬件级 launch

它只处理“如何和硬件连通、发包、收包、解释底层状态”。

### 5.2 `chassis_ctrl`

`chassis_ctrl` 在这次迁移后只负责：

- 执行策略
- 动态规划
- 预计算执行
- 视觉处理
- ROS 业务节点

它不再拥有底层 SDK 实现，也不再拥有相机驱动源码。

### 5.3 `robot_interface_hub`

`robot_interface_hub` 继续作为控制面包，只做：

- 全局接口定义
- 新前端
- API bridge
- 顶层 launch 编排

它只通过 launch、topic、service、action 名称来调用 `robot_hw_drivers` 和 `chassis_ctrl`。

## 6. 相机驱动迁移设计

### 6.1 迁移源

当前相机包不是工作区真实目录，而是外部磁盘符号链接：

```text
src/ScepterROS -> /home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS/src/ScepterROS
```

本次迁移要把这份外部目录内容“拉回工作区内”，变成真实源码和依赖文件，不再依赖外部路径。

### 6.2 新包内布局

相机相关内容迁入 `robot_hw_drivers` 后，建议布局如下：

```text
src/robot_hw_drivers/
  cfg/Sceptertof_roscpp.cfg
  srv/ConvertDepthToPointCloud.srv
  launch/scepter_camera.launch
  scripts/try.py
  include/robot_hw_drivers/camera/scepter_manager.hpp
  src/camera/scepter_driver.cpp
  src/camera/scepter_manager.cpp
  dependencies/Include/*
  dependencies/Lib/*
  dependencies/Lib/Drivers/*
```

### 6.3 二进制 SDK 处理原则

当前相机 launch 已经明确依赖包内二进制目录：

```xml
<env name="LD_LIBRARY_PATH"
     value="$(optenv LD_LIBRARY_PATH):$(find ScepterROS)/dependencies/Lib:$(find ScepterROS)/dependencies/Lib/Drivers" />
```

这说明 Scepter 相机不是走系统安装库，而是走包内 vendor SDK。

因此迁移时必须满足以下原则：

- `dependencies/Include/` 和 `dependencies/Lib/` 连同源码一起迁入 `robot_hw_drivers`
- 运行时依然通过 launch 注入 `LD_LIBRARY_PATH`
- 不要求现场额外安装 Scepter SDK
- 不再依赖 `/home/hyq-/ScepterSDK/...` 这样的外部绝对路径

迁移后的 launch 改为：

```xml
<env name="LD_LIBRARY_PATH"
     value="$(optenv LD_LIBRARY_PATH):$(find robot_hw_drivers)/dependencies/Lib:$(find robot_hw_drivers)/dependencies/Lib/Drivers" />
```

### 6.4 命名策略

包名改为符合 ROS 习惯的小写包名：

- 旧：`ScepterROS`
- 新：`robot_hw_drivers`

但为了减小现场冲击，以下运行元素保持稳定：

- 可执行文件名继续保留 `scepter_camera`
- 节点名继续保留 `scepter_manager`
- 相机输出 topic 名称不在本次主动重命名

## 7. 索驱和线模驱动层迁移设计

### 7.1 当前来源

当前驱动层已经位于：

- `src/chassis_ctrl/include/chassis_ctrl/driver/`
- `src/chassis_ctrl/src/driver/`

包括：

- `CabinTcpTransport`
- `CabinProtocol`
- `CabinDriver`
- `ModbusTransport`
- `LinearModuleProtocol`
- `LinearModuleDriver`

### 7.2 迁移后的目录

迁入 `robot_hw_drivers` 后建议整理为：

```text
src/robot_hw_drivers/
  include/robot_hw_drivers/driver/driver_types.hpp
  include/robot_hw_drivers/driver/cabin_tcp_transport.hpp
  include/robot_hw_drivers/driver/cabin_protocol.hpp
  include/robot_hw_drivers/driver/cabin_driver.hpp
  include/robot_hw_drivers/driver/modbus_transport.hpp
  include/robot_hw_drivers/driver/linear_module_protocol.hpp
  include/robot_hw_drivers/driver/linear_module_driver.hpp
  src/driver/cabin_tcp_transport.cpp
  src/driver/cabin_protocol.cpp
  src/driver/cabin_driver.cpp
  src/driver/modbus_transport.cpp
  src/driver/linear_module_protocol.cpp
  src/driver/linear_module_driver.cpp
```

### 7.3 命名空间

驱动层命名空间从：

```cpp
namespace chassis_ctrl::driver
```

收口为：

```cpp
namespace robot_hw_drivers::driver
```

这样驱动层的归属不会再混在业务包命名空间里。

### 7.4 构建产物

`robot_hw_drivers` 提供一个统一驱动库，例如：

```text
librobot_hw_driver_core.so
```

其中包含：

- 索驱 TCP 驱动实现
- 线性模组 Modbus 驱动实现

`chassis_ctrl` 只链接这个库，不再编译自己的 `src/driver/*`。

## 8. 接口与依赖规则

### 8.1 允许的依赖方向

```text
robot_interface_hub  --->  robot_hw_drivers
         |                    |
         v                    v
                    chassis_ctrl
```

更具体地说：

- `robot_interface_hub`
  - 运行时 include `robot_hw_drivers/launch/scepter_camera.launch`
  - 不 include 驱动实现头文件
- `chassis_ctrl`
  - 编译时依赖 `robot_interface_hub` 的全局接口类型
  - 编译时依赖 `robot_hw_drivers` 的驱动头和驱动库
- `robot_hw_drivers`
  - 不依赖 `robot_interface_hub`
  - 不依赖 `chassis_ctrl`

### 8.2 相机本地 service 的处理

`ConvertDepthToPointCloud.srv` 属于相机硬件本地能力，不属于全局控制面。

因此它保留在 `robot_hw_drivers`，而不是迁入 `robot_interface_hub`。

换句话说：

- 全局控制接口：放在 `robot_interface_hub`
- 设备本地接口：放在 `robot_hw_drivers`

## 9. Launch 归属

迁移后，launch 关系为：

- `robot_hw_drivers/launch/scepter_camera.launch`
  - 只负责启动相机驱动
- `robot_interface_hub/launch/api.launch`
  - include `robot_hw_drivers/launch/scepter_camera.launch`
  - 启动 `topictransNode`
  - 启动 `system_log_mux.py`
- `robot_interface_hub/launch/run.launch`
  - include `api.launch`
  - 启动 `suoquNode`
  - 启动 `moduanNode`
  - 启动前端和 TF 节点

这意味着：

- 硬件 launch 归 `robot_hw_drivers`
- 系统编排 launch 归 `robot_interface_hub`

## 10. 构建设计

### 10.1 `robot_hw_drivers` 依赖

新包需要统一声明这些依赖：

- `roscpp`
- `rospy`
- `sensor_msgs`
- `std_msgs`
- `geometry_msgs`
- `dynamic_reconfigure`
- `camera_info_manager`
- `image_transport`
- `cv_bridge`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `pcl_ros`
- `pcl_conversions`
- `message_generation`
- `message_runtime`
- `libmodbus`
- `OpenCV`

### 10.2 CMake 结构

新包 CMake 至少生成两类产物：

- 相机驱动可执行：
  - `scepter_camera`
- 驱动层公共库：
  - `robot_hw_driver_core`

`chassis_ctrl` 只链接：

```cmake
target_link_libraries(suoquNode ${catkin_LIBRARIES} robot_hw_driver_core)
target_link_libraries(moduanNode ${catkin_LIBRARIES} robot_hw_driver_core)
```

## 11. 迁移范围

### 11.1 迁出到 `robot_hw_drivers`

来自外部磁盘的相机目录内容：

- `src/ScepterROS/cfg/*`
- `src/ScepterROS/include/*`
- `src/ScepterROS/src/*`
- `src/ScepterROS/scripts/*`
- `src/ScepterROS/srv/*`
- `src/ScepterROS/launch/*`
- `src/ScepterROS/dependencies/*`

来自 `chassis_ctrl` 的驱动层内容：

- `src/chassis_ctrl/include/chassis_ctrl/driver/*`
- `src/chassis_ctrl/src/driver/*`

### 11.2 迁移后删除

迁移完成并验证后，以下内容应删除：

- 外部磁盘符号链接 `src/ScepterROS`
- `src/chassis_ctrl/include/chassis_ctrl/driver/`
- `src/chassis_ctrl/src/driver/`
- `chassis_ctrl` 中与旧驱动构建直接绑定的库定义

## 12. 验收标准

迁移完成后，应满足以下条件：

1. `src/ScepterROS` 不再是外部符号链接。
2. 新包 `src/robot_hw_drivers/` 为工作区真实目录。
3. `robot_interface_hub/api.launch` 改为 include `$(find robot_hw_drivers)/launch/scepter_camera.launch`。
4. `chassis_ctrl` 不再包含 `src/driver/*` 和 `include/chassis_ctrl/driver/*`。
5. `suoquNode` 和 `moduanNode` 编译时通过 `robot_hw_drivers::driver::*` 使用驱动层。
6. `catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_interface_hub;robot_hw_drivers;chassis_ctrl"` 可以通过。
7. ROS 能解析新的相机节点路径：
   - `devel/lib/robot_hw_drivers/scepter_camera`
8. 相机运行时仍能找到 vendor SDK 动态库。

## 13. 风险与控制

### 13.1 相机动态库丢失风险

如果只迁源码、不迁 `dependencies/Lib*`，相机会在运行时直接失败。

控制策略：

- vendor SDK 目录与源码一起迁移
- launch 明确保留 `LD_LIBRARY_PATH`

### 13.2 包循环依赖风险

如果把全局接口也混进 `robot_hw_drivers`，容易再次形成包职责交叉。

控制策略：

- 全局接口只在 `robot_interface_hub`
- 设备本地接口只在 `robot_hw_drivers`

### 13.3 命名空间混乱风险

如果驱动层继续保留 `chassis_ctrl::driver` 命名空间，代码层级会继续误导维护者。

控制策略：

- 驱动层统一收口到 `robot_hw_drivers::driver`

## 14. 最终结论

本次重构采用单一底层硬件包方案：

- 新建 `robot_hw_drivers`
- 将相机驱动从外部磁盘符号链接正式迁入工作区真实目录
- 将索驱 TCP 驱动和线性模组 Modbus 驱动从 `chassis_ctrl` 迁出
- `robot_interface_hub` 只做控制面
- `chassis_ctrl` 只做算法和应用执行

这样之后，整个工程会形成清晰的三层包结构：

```text
robot_interface_hub  -> 控制面
robot_hw_drivers     -> 底层硬件驱动
chassis_ctrl         -> 算法与执行应用
```
