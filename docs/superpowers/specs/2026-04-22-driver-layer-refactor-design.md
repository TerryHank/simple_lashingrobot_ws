# 驱动层重构设计

## 1. 目标

本次重构只处理底层驱动，不重写现有执行流程，不先动新前端动作链。

目标是把以下两类硬件连接能力从 `suoquNode.cpp` 和 `moduanNode.cpp` 中抽离为可复用、可测试、可持续运行的独立驱动层：

- 索驱 TCP 驱动
- 线性模组 Modbus 驱动

重构完成后，应用层节点仍可保留现有 ROS 对外入口，但节点内部不再直接操作 `socket/select/send/recv` 和 `modbus_*`，而是只调用语义化驱动接口。

## 2. 问题背景

当前两个主节点同时承担了过多职责：

- 硬件连接管理
- 协议组帧和寄存器映射
- 状态解码和错误解释
- 到位/完成判定
- 业务流程编排
- ROS service/topic 对外适配

这导致几个典型问题：

- TCP/Modbus 通信失败会直接影响整个节点生命周期
- 断线重连逻辑散落在业务节点中，无法单独验证
- 协议解析、日志、错误语义和业务逻辑强耦合
- 很难在不触碰业务层的前提下替换或扩展驱动实现
- 现场问题定位时，日志来源和状态边界不清晰

## 3. 设计原则

### 3.1 分层单一职责

按典型机器人工程结构拆为：

- `transport`：连接生命周期、线程安全读写、超时、重连
- `protocol`：组帧、校验、寄存器映射、状态字解释
- `driver`：对上提供语义化硬件能力
- `algorithm`：到位判定、完成判定、重试/超时策略
- `app`：ROS 适配和业务流程

### 3.2 驱动层不主动杀进程

驱动层不允许再出现 `_exit(...)` 或等价行为。

驱动层的职责是：

- 保持连接
- 自动重连
- 提供当前连接状态
- 返回结构化错误
- 记录详细诊断日志

是否中止当前任务、是否向上返回失败、是否切换流程，由应用层决定。

### 3.3 保持第一阶段外部接口稳定

第一阶段不要求改变新前端和现有 ROS 入口。

允许保留：

- `suoquNode`
- `moduanNode`
- `topics_transfer`

但这几个节点内部只能调用驱动接口，不再直接接触底层通信细节。

### 3.4 按 ROS 语义区分能力

按 ROS 规范区分三类接口：

- `topic`：持续状态、连接状态、错误事件
- `service`：短操作，例如停止、重连、清标志、复位
- `action`：长操作，例如移动到位、执行一组点

应用层可以继续保留旧 service/topic，但新的驱动层节点内部要按这个语义组织。

## 4. 目标架构

### 4.1 索驱链

```text
app/suoquNode
  -> driver/CabinDriver
    -> protocol/CabinProtocol
    -> transport/CabinTcpTransport
```

职责：

- `CabinTcpTransport`
  - 建连、断线检测、自动重连、发送接收、线程安全
- `CabinProtocol`
  - `0x0012` 等命令组帧
  - 回包校验
  - 状态字解码
- `CabinDriver`
  - `moveToPose(...)`
  - `stop()`
  - `readState()`
  - `getConnectionState()`

### 4.2 线性模组链

```text
app/moduanNode
  -> driver/LinearModuleDriver
    -> protocol/LinearModuleProtocol
    -> transport/ModbusTransport
```

职责：

- `ModbusTransport`
  - Modbus 连接生命周期
  - 读写封装
  - 重连和错误缓存
- `LinearModuleProtocol`
  - 寄存器地址映射
  - 点位写入
  - 状态读取和报警位解释
- `LinearModuleDriver`
  - `writePointSlot(...)`
  - `executeQueuedPoints(...)`
  - `stop()`
  - `clearFinishAll()`
  - `readState()`

### 4.3 算法层

算法层只保留硬件无关的判定逻辑，例如：

- `CabinMotionGuard`
  - 到位容差
  - 稳定样本数
  - 运动中日志节流
- `LinearModuleExecutionGuard`
  - `FINISHALL` 超时
  - 完成等待日志
  - 错误消息拼装

## 5. 目录结构

新增目录：

- `src/chassis_ctrl/include/chassis_ctrl/driver/`
- `src/chassis_ctrl/include/chassis_ctrl/algorithm/`
- `src/chassis_ctrl/src/driver/`
- `src/chassis_ctrl/src/algorithm/`

第一阶段核心文件：

- `include/chassis_ctrl/driver/cabin_tcp_transport.hpp`
- `include/chassis_ctrl/driver/cabin_protocol.hpp`
- `include/chassis_ctrl/driver/cabin_driver.hpp`
- `include/chassis_ctrl/driver/modbus_transport.hpp`
- `include/chassis_ctrl/driver/linear_module_protocol.hpp`
- `include/chassis_ctrl/driver/linear_module_driver.hpp`
- `include/chassis_ctrl/algorithm/cabin_motion_guard.hpp`
- `include/chassis_ctrl/algorithm/linear_module_execution_guard.hpp`
- `src/driver/cabin_tcp_transport.cpp`
- `src/driver/cabin_protocol.cpp`
- `src/driver/cabin_driver.cpp`
- `src/driver/modbus_transport.cpp`
- `src/driver/linear_module_protocol.cpp`
- `src/driver/linear_module_driver.cpp`

## 6. 驱动层接口设计

### 6.1 索驱驱动接口

建议的语义接口：

```cpp
namespace chassis_ctrl::driver {

enum class ConnectionState {
    kDisconnected,
    kConnecting,
    kReady,
    kReconnecting,
    kFault,
};

struct DriverError {
    std::string code;
    std::string message;
    std::string detail;
    bool retryable = false;
};

struct CabinPoseCommand {
    float speed_mm_per_sec = 0.0f;
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
};

struct CabinStateSnapshot {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    int motion_status = 0;
    int device_alarm = 0;
    int internal_calc_error = 0;
    bool connected = false;
};

class CabinDriver {
public:
    bool start();
    void stop();
    bool moveToPose(const CabinPoseCommand& command, DriverError* error);
    bool sendStop(DriverError* error);
    CabinStateSnapshot readState() const;
    ConnectionState connectionState() const;
    std::string lastErrorText() const;
};

}  // namespace chassis_ctrl::driver
```

### 6.2 线性模组驱动接口

```cpp
namespace chassis_ctrl::driver {

struct LinearModulePoint {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    float angle_deg = 0.0f;
};

struct LinearModuleStateSnapshot {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    float angle_deg = 0.0f;
    int finish_all = 0;
    int error_x = 0;
    int error_y = 0;
    int error_z = 0;
    int error_motor = 0;
    bool connected = false;
};

class LinearModuleDriver {
public:
    bool start();
    void stop();
    bool clearFinishAll(DriverError* error);
    bool executeQueuedPoints(
        const std::vector<LinearModulePoint>& points,
        DriverError* error);
    bool requestZero(DriverError* error);
    LinearModuleStateSnapshot readState() const;
    ConnectionState connectionState() const;
    std::string lastErrorText() const;
};

}  // namespace chassis_ctrl::driver
```

## 7. ROS 结构化能力设计

### 7.1 新增驱动状态 topic

新增驱动状态发布，但先不要求前端直接消费：

- `/cabin_driver/state`
- `/cabin_driver/connection`
- `/linear_module_driver/state`
- `/linear_module_driver/connection`

第一阶段可以继续复用现有 `cabin_upload.msg` 和 `linear_module_upload.msg`，连接状态不足的部分先用日志和扩展字段补齐；后续如果需要再单独抽出标准驱动状态 msg。

### 7.2 新增驱动 action/service

第一阶段新增但不强制前端直连：

- `cabin_driver/move_to_pose`：action
- `cabin_driver/stop`：service
- `cabin_driver/reconnect`：service
- `linear_module_driver/execute_points`：action
- `linear_module_driver/clear_finishall`：service
- `linear_module_driver/stop`：service

应用层节点可以在内部调用这些 action/service，也可以先直接链接 C++ 驱动类。第一阶段优先保证内聚性，不强求全部 ROS 化到位。

## 8. 错误处理与生命周期

### 8.1 连接状态机

两个驱动统一采用：

- `Disconnected`
- `Connecting`
- `Ready`
- `Reconnecting`
- `Fault`

状态机语义：

- `Disconnected`：未开始或已停止
- `Connecting`：首次连接
- `Ready`：连接可用
- `Reconnecting`：出现传输错误后的后台重连
- `Fault`：超过阈值仍无法恢复，但驱动层进程不自杀

### 8.2 错误上报原则

所有底层错误统一结构化输出：

- 错误类别
- 原始系统错误
- 协议上下文
- 是否可重试
- 最近一次 frame 或寄存器信息

ROS 日志继续保留：

- `ROS_ERROR_STREAM`
- `ROS_WARN_STREAM`
- `ROS_INFO_STREAM`

但日志只是副产物，权威信息由驱动层状态和 `DriverError` 返回。

### 8.3 不再通过驱动层退出进程

去除以下模式：

- 通信失败直接 `_exit(4)`
- 读状态报警直接 `_exit(3)`

改为：

- 驱动状态切到 `Fault`
- 返回结构化错误
- 由应用层决定暂停任务、返回失败或请求人工介入

## 9. 第一阶段实施范围

本轮只做第一阶段：

1. 抽离索驱 TCP 驱动基础设施
2. 抽离线性模组 Modbus 驱动基础设施
3. 将 `suoquNode` 和 `moduanNode` 接到新驱动对象
4. 去掉节点里直接底层通信的大块实现
5. 保持新前端动作链和现有对外 ROS 接口先不变

## 10. 非目标

本轮不做这些：

- 不重写扫描/执行算法
- 不重写新前端交互
- 不一次性清理所有旧 service/topic
- 不把所有业务逻辑都迁到新控制器层
- 不更换现有消息和服务定义

## 11. 验收标准

第一阶段完成的验收口径：

- `suoquNode.cpp` 内不再直接操作 TCP socket 细节
- `moduanNode.cpp` 内不再直接操作 Modbus 读写细节
- 驱动层断线后进入重连/故障状态，而不是直接杀进程
- 当前新前端动作链仍能编译并跑通基础入口
- 驱动层能输出结构化错误和连接状态

## 12. 风险与迁移策略

最大风险不是接口定义，而是现场行为一致性。

因此迁移策略采用：

- 先抽底层
- 再保持应用层入口不变
- 最后逐步删旧实现

这样能把“结构改造风险”和“业务行为风险”分开。

对于现场已知敏感点，例如：

- 索驱对特定位姿的 TCP 断链
- 线性模组对边界点位的执行差异

本次只负责让这些问题被更清晰地隔离和观测，不在驱动抽离阶段混入新的业务口径调整。
