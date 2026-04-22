# 驱动层重构实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将索驱 TCP 和线性模组 Modbus 从 `suoquNode/moduanNode` 中抽离为独立驱动层，同时保持第一阶段现有应用层 ROS 接口和新前端动作链不变。

**架构：** 新增 `transport/protocol/driver/algorithm` 四层 C++ 结构，应用层节点保留现有对外入口，只改为调用语义化驱动对象。驱动层不再直接 `_exit(...)`，统一维护连接状态、结构化错误和自动重连。

**技术栈：** ROS Noetic、C++14、catkin、POSIX TCP、libmodbus、现有 `chassis_ctrl` msg/srv/action。

---

## 文件结构

**创建：**

- `src/chassis_ctrl/include/chassis_ctrl/driver/driver_types.hpp`
- `src/chassis_ctrl/include/chassis_ctrl/driver/cabin_tcp_transport.hpp`
- `src/chassis_ctrl/include/chassis_ctrl/driver/cabin_protocol.hpp`
- `src/chassis_ctrl/include/chassis_ctrl/driver/cabin_driver.hpp`
- `src/chassis_ctrl/include/chassis_ctrl/driver/modbus_transport.hpp`
- `src/chassis_ctrl/include/chassis_ctrl/driver/linear_module_protocol.hpp`
- `src/chassis_ctrl/include/chassis_ctrl/driver/linear_module_driver.hpp`
- `src/chassis_ctrl/include/chassis_ctrl/algorithm/cabin_motion_guard.hpp`
- `src/chassis_ctrl/include/chassis_ctrl/algorithm/linear_module_execution_guard.hpp`
- `src/chassis_ctrl/src/driver/cabin_tcp_transport.cpp`
- `src/chassis_ctrl/src/driver/cabin_protocol.cpp`
- `src/chassis_ctrl/src/driver/cabin_driver.cpp`
- `src/chassis_ctrl/src/driver/modbus_transport.cpp`
- `src/chassis_ctrl/src/driver/linear_module_protocol.cpp`
- `src/chassis_ctrl/src/driver/linear_module_driver.cpp`

**修改：**

- `src/chassis_ctrl/CMakeLists.txt`
- `src/chassis_ctrl/src/suoquNode.cpp`
- `src/chassis_ctrl/src/moduanNode.cpp`
- `src/chassis_ctrl/test/test_pointai_order.py`

**职责：**

- `driver_types.hpp`
  - 统一连接状态、错误结构、驱动快照和语义命令
- `cabin_tcp_transport.*`
  - 索驱 TCP 建连、读写、超时、重连、线程安全发送
- `cabin_protocol.*`
  - 索驱命令组帧、回包校验、状态字解码
- `cabin_driver.*`
  - 索驱语义接口，封装 transport + protocol
- `modbus_transport.*`
  - Modbus 连接、读写、重连、错误传播
- `linear_module_protocol.*`
  - 寄存器映射、点位写入、状态读取
- `linear_module_driver.*`
  - 线性模组语义接口
- `cabin_motion_guard.hpp`
  - 索驱到位判定
- `linear_module_execution_guard.hpp`
  - `FINISHALL` 完成判定

## 任务 1：搭建驱动层公共类型与构建骨架

**文件：**
- 创建：`src/chassis_ctrl/include/chassis_ctrl/driver/driver_types.hpp`
- 修改：`src/chassis_ctrl/CMakeLists.txt`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的结构测试**

```python
def test_driver_layer_files_and_names_exist(self):
    expected_files = [
        "src/chassis_ctrl/include/chassis_ctrl/driver/driver_types.hpp",
        "src/chassis_ctrl/include/chassis_ctrl/driver/cabin_tcp_transport.hpp",
        "src/chassis_ctrl/include/chassis_ctrl/driver/cabin_driver.hpp",
        "src/chassis_ctrl/include/chassis_ctrl/driver/modbus_transport.hpp",
        "src/chassis_ctrl/include/chassis_ctrl/driver/linear_module_driver.hpp",
    ]
    for file_path in expected_files:
        self.assertTrue((self.repo_root / file_path).exists(), file_path)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_driver_layer_files_and_names_exist`
预期：FAIL，提示缺少新的驱动层头文件。

- [ ] **步骤 3：创建最小骨架头文件和构建入口**

代码要求：
- 在 `driver_types.hpp` 中定义 `ConnectionState`、`DriverError`
- 在 `CMakeLists.txt` 中新增驱动层库目标，例如 `chassis_ctrl_driver`
- 保持 `suoquNode` 和 `moduanNode` 仍然可链接

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_driver_layer_files_and_names_exist`
预期：PASS

- [ ] **步骤 5：运行基础构建验证**

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="ScepterROS;chassis_ctrl" -j2`
预期：exit 0

## 任务 2：实现索驱 TCP transport 和 protocol 骨架

**文件：**
- 创建：`src/chassis_ctrl/include/chassis_ctrl/driver/cabin_tcp_transport.hpp`
- 创建：`src/chassis_ctrl/include/chassis_ctrl/driver/cabin_protocol.hpp`
- 创建：`src/chassis_ctrl/src/driver/cabin_tcp_transport.cpp`
- 创建：`src/chassis_ctrl/src/driver/cabin_protocol.cpp`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的索驱驱动结构测试**

```python
def test_cabin_transport_and_protocol_declare_standard_interfaces(self):
    text = (self.repo_root / "src/chassis_ctrl/include/chassis_ctrl/driver/cabin_tcp_transport.hpp").read_text()
    self.assertIn("class CabinTcpTransport", text)
    self.assertIn("bool connect", text)
    self.assertIn("bool sendAndReceive", text)

    protocol_text = (self.repo_root / "src/chassis_ctrl/include/chassis_ctrl/driver/cabin_protocol.hpp").read_text()
    self.assertIn("class CabinProtocol", protocol_text)
    self.assertIn("buildMoveToPoseFrame", protocol_text)
    self.assertIn("decodeStatus", protocol_text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_cabin_transport_and_protocol_declare_standard_interfaces`
预期：FAIL

- [ ] **步骤 3：实现索驱 transport/protocol 头文件和最小 cpp**

代码要求：
- `CabinTcpTransport` 持有 socket、连接状态、最近错误
- 提供 `connect()`、`disconnect()`、`ensureConnected()`、`sendAndReceive(...)`
- `CabinProtocol` 提供 `buildMoveToPoseFrame(...)`、`buildStopFrame()`、`decodeStatus(...)`
- 不复制现有业务逻辑，只先做可编译最小骨架

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_cabin_transport_and_protocol_declare_standard_interfaces`
预期：PASS

- [ ] **步骤 5：运行基础构建验证**

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="ScepterROS;chassis_ctrl" -j2`
预期：exit 0

## 任务 3：实现索驱 Driver 和应用层初接线

**文件：**
- 创建：`src/chassis_ctrl/include/chassis_ctrl/driver/cabin_driver.hpp`
- 创建：`src/chassis_ctrl/include/chassis_ctrl/algorithm/cabin_motion_guard.hpp`
- 创建：`src/chassis_ctrl/src/driver/cabin_driver.cpp`
- 修改：`src/chassis_ctrl/src/suoquNode.cpp`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的接线测试**

```python
def test_suoqu_node_uses_cabin_driver_instead_of_direct_tcp_helpers(self):
    text = (self.repo_root / "src/chassis_ctrl/src/suoquNode.cpp").read_text()
    self.assertIn("CabinDriver", text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_node_uses_cabin_driver_instead_of_direct_tcp_helpers`
预期：FAIL

- [ ] **步骤 3：实现 CabinDriver 并在 suoquNode 中最小接线**

代码要求：
- `CabinDriver` 封装 move/stop/read-state 语义接口
- `suoquNode` 先引入驱动头并实例化驱动对象
- 第一阶段允许旧 helper 与新驱动暂时并存，但新的移动入口至少有一条链切换到 `CabinDriver`
- 不得再新增 `_exit(...)` 到驱动代码路径

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_node_uses_cabin_driver_instead_of_direct_tcp_helpers`
预期：PASS

- [ ] **步骤 5：运行定向构建验证**

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="ScepterROS;chassis_ctrl" -j2`
预期：exit 0

## 任务 4：实现线性模组 transport/protocol/driver 骨架

**文件：**
- 创建：`src/chassis_ctrl/include/chassis_ctrl/driver/modbus_transport.hpp`
- 创建：`src/chassis_ctrl/include/chassis_ctrl/driver/linear_module_protocol.hpp`
- 创建：`src/chassis_ctrl/include/chassis_ctrl/driver/linear_module_driver.hpp`
- 创建：`src/chassis_ctrl/include/chassis_ctrl/algorithm/linear_module_execution_guard.hpp`
- 创建：`src/chassis_ctrl/src/driver/modbus_transport.cpp`
- 创建：`src/chassis_ctrl/src/driver/linear_module_protocol.cpp`
- 创建：`src/chassis_ctrl/src/driver/linear_module_driver.cpp`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的线模驱动结构测试**

```python
def test_linear_module_driver_declares_execute_and_state_interfaces(self):
    text = (self.repo_root / "src/chassis_ctrl/include/chassis_ctrl/driver/linear_module_driver.hpp").read_text()
    self.assertIn("class LinearModuleDriver", text)
    self.assertIn("executeQueuedPoints", text)
    self.assertIn("clearFinishAll", text)
    self.assertIn("readState", text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_linear_module_driver_declares_execute_and_state_interfaces`
预期：FAIL

- [ ] **步骤 3：实现线模驱动层骨架**

代码要求：
- `ModbusTransport` 封装读写和连接状态
- `LinearModuleProtocol` 封装寄存器地址和槽位写入
- `LinearModuleDriver` 提供 `executeQueuedPoints(...)`、`clearFinishAll(...)`、`readState()`
- `LinearModuleExecutionGuard` 负责 `FINISHALL` 等待逻辑骨架

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_linear_module_driver_declares_execute_and_state_interfaces`
预期：PASS

- [ ] **步骤 5：运行基础构建验证**

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="ScepterROS;chassis_ctrl" -j2`
预期：exit 0

## 任务 5：在 moduanNode 中接入线模驱动骨架

**文件：**
- 修改：`src/chassis_ctrl/src/moduanNode.cpp`
- 测试：`src/chassis_ctrl/test/test_pointai_order.py`

- [ ] **步骤 1：编写失败的接线测试**

```python
def test_moduan_node_uses_linear_module_driver(self):
    text = (self.repo_root / "src/chassis_ctrl/src/moduanNode.cpp").read_text()
    self.assertIn("LinearModuleDriver", text)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_moduan_node_uses_linear_module_driver`
预期：FAIL

- [ ] **步骤 3：实现最小接线**

代码要求：
- `moduanNode` 引入 `LinearModuleDriver`
- 预计算执行路径至少有一条链改为通过驱动对象写点、启动执行、读状态
- 节点保留现有 ROS service 名称不变

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_moduan_node_uses_linear_module_driver`
预期：PASS

- [ ] **步骤 5：运行定向构建验证**

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="ScepterROS;chassis_ctrl" -j2`
预期：exit 0

## 任务 6：验证现有动作链未被第一阶段破坏

**文件：**
- 修改：`src/chassis_ctrl/test/test_pointai_order.py`
- 修改：`src/chassis_ctrl/test/test_workspace_picker_web_launch.py`

- [ ] **步骤 1：编写或补强回归测试**

```python
def test_existing_direct_bind_entrypoint_still_exists(self):
    text = (self.repo_root / "src/chassis_ctrl/src/suoquNode.cpp").read_text()
    self.assertIn("run_bind_path_direct_test", text)
```

- [ ] **步骤 2：运行定向测试**

运行：`python3 -m unittest src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_existing_direct_bind_entrypoint_still_exists`
预期：PASS

- [ ] **步骤 3：运行驱动层相关测试子集**

运行：
`python3 -m unittest \
src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_driver_layer_files_and_names_exist \
src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_cabin_transport_and_protocol_declare_standard_interfaces \
src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_suoqu_node_uses_cabin_driver_instead_of_direct_tcp_helpers \
src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_linear_module_driver_declares_execute_and_state_interfaces \
src.chassis_ctrl.test.test_pointai_order.PointAIOrderTest.test_moduan_node_uses_linear_module_driver`

预期：全部 PASS

- [ ] **步骤 4：运行最终构建验证**

运行：`source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="ScepterROS;chassis_ctrl" -j2`
预期：exit 0

## 自检

- 规格覆盖度：计划已覆盖索驱驱动、线模驱动、应用层接线、测试和构建验证
- 占位符扫描：已移除“后续补充”“适当处理”等模糊步骤
- 类型一致性：统一使用 `CabinDriver`、`LinearModuleDriver`、`DriverError`、`ConnectionState`
