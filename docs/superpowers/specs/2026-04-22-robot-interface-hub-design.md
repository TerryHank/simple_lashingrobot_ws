# 大工程全局接口包重构设计

## 1. 背景

当前工程只有一个主业务包 `chassis_ctrl`，它同时承担了以下多种职责：

- 全局 `msg/srv/action` 定义
- 全局 `launch` 编排
- 前端静态页面 `ir_workspace_picker_web`
- 前端 HTTP 打开与静态资源服务脚本
- 前端 TCP / ROS API 桥接节点 `topics_transfer.cpp`
- 底层执行节点 `suoquNode`、`moduanNode`
- 驱动层、算法层、数据文件与运行时账本

这会带来几个长期问题：

- 控制面与执行面耦合过深，接口修改容易波及底层实现
- `chassis_ctrl` 既是接口包，又是执行包，边界不清
- 新前端、launch、topic/service/action 类型都散在业务包内部，不利于后续扩展
- 未来如果继续拆驱动层、算法层、应用层，现有包边界会越来越混乱

本次设计目标是把“整个大工程的全局控制面”独立成一个包，作为统一入口和统一接口层。

## 2. 设计目标

新建单一控制面包，暂定名为 `robot_interface_hub`，统一承接以下内容：

- 全局 `msg`
- 全局 `srv`
- 全局 `action`
- 全局 `launch`
- 前端静态资源
- 前端 HTTP 打开/托管脚本
- 前端 TCP / ROS API 桥接节点

同时把 `chassis_ctrl` 收口为执行实现包，只保留：

- 驱动层
- 算法层
- 执行节点
- 运行时数据、配置与内部实现逻辑

## 3. 非目标

本次重构不做这些事情：

- 不重写前端业务逻辑
- 不重命名现有 ROS topic/service/action 名称
- 不改变新前端动作链语义
- 不把底层驱动层再拆成单独包
- 不处理旧展示链 `*_show.cpp` 的彻底清理

也就是说，这次重点是“包边界重构”，不是“业务语义重写”。

## 4. 总体方案

采用单包控制面方案：

```text
robot_interface_hub
  ├─ msg/
  ├─ srv/
  ├─ action/
  ├─ launch/
  ├─ web/
  ├─ scripts/
  └─ src/
       └─ interface_gateway_node(topics_transfer)

chassis_ctrl
  ├─ include/
  ├─ src/
  │   ├─ driver/
  │   ├─ algorithm/
  │   ├─ suoquNode.cpp
  │   └─ moduanNode.cpp
  ├─ scripts/
  │   ├─ pointAI.py
  │   ├─ gripper_tf_broadcaster.py
  │   └─ stable_point_tf_broadcaster.py
  ├─ config/
  └─ data/
```

## 5. 包职责定义

### 5.1 `robot_interface_hub`

这是“对外控制面包”，负责：

- 统一定义跨包通信契约
- 统一承接新前端入口
- 统一承接前端到 ROS 的桥接
- 统一提供整套系统 launch 启动入口

它是大工程的“外壳”和“接口面”。

### 5.2 `chassis_ctrl`

这是“执行实现包”，负责：

- 索驱执行
- 线性模组执行
- 点云/视觉执行逻辑
- 预计算路径执行
- 驱动层、算法层、运行时状态和内部配置

它不再承载全局接口定义和前端控制面资源。

## 6. 迁移范围

### 6.1 从 `chassis_ctrl` 迁出到 `robot_interface_hub`

#### 消息类型

迁出整个目录：

- `src/chassis_ctrl/msg/*`

包括但不限于：

- `PointCoords.msg`
- `PointsArray.msg`
- `AreaProgress.msg`
- `linear_module_upload.msg`
- `motion.msg`

#### 服务类型

迁出整个目录：

- `src/chassis_ctrl/srv/*`

包括但不限于：

- `ProcessImage.srv`
- `ExecuteBindPoints.srv`
- `StartGlobalWork.srv`
- `StartPseudoSlamScan.srv`
- `SingleMove.srv`

#### 动作类型

迁出整个目录：

- `src/chassis_ctrl/action/*`

当前为：

- `Move.action`

#### Launch

迁出整个目录：

- `src/chassis_ctrl/launch/api.launch`
- `src/chassis_ctrl/launch/run.launch`
- `src/chassis_ctrl/launch/suoquAndmoduan.launch`
- `src/chassis_ctrl/launch/pointai_tf_verify.launch`

#### 前端静态资源

把当前独立目录：

- `src/ir_workspace_picker_web/`

迁入新包，例如：

- `src/robot_interface_hub/web/`

包括：

- `index.html`
- `ir_workspace_picker.mjs`
- `ir_workspace_picker_helpers.mjs`
- `vendor/`

#### 前端支撑脚本

迁出以下脚本到新包：

- `src/chassis_ctrl/scripts/workspace_picker_web_server.py`
- `src/chassis_ctrl/scripts/workspace_picker_web_open.py`
- `src/chassis_ctrl/scripts/system_log_mux.py`

这些脚本属于控制面和运行编排，不属于底层执行实现。

#### 前端 TCP / ROS API 桥接节点

迁出：

- `src/chassis_ctrl/src/topics_transfer.cpp`

目标位置建议为：

- `src/robot_interface_hub/src/interface_gateway_node.cpp`

可执行名允许保留 `topictransNode` 以兼容旧 launch 和现场脚本；内部节点名也允许先保持 `/topics_transfer_node` 不变。

### 6.2 继续保留在 `chassis_ctrl`

- `suoquNode.cpp`
- `moduanNode.cpp`
- `client.cpp`
- 驱动层 `src/driver/*`
- 算法层
- `pointAI.py`
- `gripper_tf_broadcaster.py`
- `stable_point_tf_broadcaster.py`
- `config/`
- `data/`
- 与执行链直接相关的测试

## 7. 依赖规则

这是这次设计最关键的约束。

### 7.1 允许的依赖方向

```text
robot_interface_hub  --->  chassis_ctrl
        ^                    |
        |                    |
        +--------------------+
         仅通过 ROS 名称和接口类型通信
```

更具体地说：

- `chassis_ctrl` 编译时依赖 `robot_interface_hub` 的 `msg/srv/action`
- `robot_interface_hub` 运行时通过 launch 和 service client 调 `chassis_ctrl` 节点
- `robot_interface_hub` 不 include `chassis_ctrl` 的内部头文件
- `chassis_ctrl` 不 include `robot_interface_hub` 的前端或 launch 实现代码

### 7.2 禁止的依赖

禁止出现：

- `chassis_ctrl` 依赖 `robot_interface_hub` 的前端资源路径
- `chassis_ctrl` include `robot_interface_hub/src/*.cpp` 或其内部实现头文件
- `robot_interface_hub` 反向 include `chassis_ctrl` 内部 driver/algorithm 头文件

`robot_interface_hub` 对 `chassis_ctrl` 的调用边界，只允许通过：

- topic 名称
- service 名称
- action 名称
- launch 编排

## 8. 目录设计

建议的新包目录：

```text
src/robot_interface_hub/
  CMakeLists.txt
  package.xml
  msg/
  srv/
  action/
  launch/
    api.launch
    run.launch
    suoquAndmoduan.launch
    pointai_tf_verify.launch
  scripts/
    workspace_picker_web_server.py
    workspace_picker_web_open.py
    system_log_mux.py
  src/
    interface_gateway_node.cpp
  web/
    index.html
    ir_workspace_picker.mjs
    ir_workspace_picker_helpers.mjs
    vendor/
  test/
```

## 9. Launch 设计

### 9.1 `robot_interface_hub/launch/run.launch`

成为新的全局启动入口，负责拉起：

- API 栈
- 前端静态资源服务
- 前端自动打开
- `chassis_ctrl` 执行节点
- `pointAI.py`
- TF 广播节点

也就是说，`run.launch` 迁到新包后，语义仍然是“整套系统启动入口”，只是归属切到控制面包。

### 9.2 `robot_interface_hub/launch/api.launch`

继续承接：

- `rosbridge_server`
- `tf2_web_republisher`
- `ScepterROS` 相机 launch
- `topictransNode` / `interface_gateway_node`
- `system_log_mux`

### 9.3 `chassis_ctrl` 不再保留系统级 launch

`chassis_ctrl` 后续如果还保留 launch，只允许保留调试型、局部型或节点自测型 launch，不再作为全局入口。

## 10. 前端资源设计

前端资源统一移动到 `robot_interface_hub/web/`。

配套调整规则如下：

- `workspace_picker_web_server.py` 不再通过 `parents[2] / "ir_workspace_picker_web"` 找目录
- 改为通过当前包路径定位 `web/`
- `workspace_picker_web_open.py` 中的相对提示文案也同步更新
- `APP/dist/index.html` 中跳转到前端的链接也要改成新的相对位置或新的统一 URL

前端本身的页面逻辑、按钮语义、本地 JS 行为本次不改。

## 11. 接口类型设计

所有当前 `chassis_ctrl::...` 类型迁移后，命名空间变为：

- `robot_interface_hub::PointCoords`
- `robot_interface_hub::ProcessImage`
- `robot_interface_hub::ExecuteBindPoints`

这意味着：

- `suoquNode.cpp`
- `moduanNode.cpp`
- `pointAI.py`
- `topics_transfer.cpp`
- 现有测试

都需要把消息类型 include / import 从 `chassis_ctrl` 改成 `robot_interface_hub`。

注意：ROS 名称空间改的是“包级类型前缀”，不是 topic/service 名称本身。  
例如 `/cabin/start_work_with_options` 这个 service 名称可以保持不变，只是服务类型从 `chassis_ctrl/StartGlobalWork` 变成 `robot_interface_hub/StartGlobalWork`。

## 12. 构建设计

### 12.1 新包 `robot_interface_hub`

需要具备：

- `message_generation`
- `actionlib_msgs`
- `roscpp`
- `rospy`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `rosgraph_msgs`
- 以及运行前端脚本所需最小依赖

它会生成：

- 所有 `msg/srv/action`
- `topictransNode`
- Web server/open 脚本安装规则

### 12.2 `chassis_ctrl`

需要调整为：

- 删除自身的 `add_message_files/add_service_files/add_action_files`
- 删除自身的 `generate_messages`
- `find_package(catkin REQUIRED COMPONENTS ...)` 中增加 `robot_interface_hub`
- `catkin_package(CATKIN_DEPENDS ...)` 中增加 `robot_interface_hub`
- 所有节点依赖切到 `${robot_interface_hub_EXPORTED_TARGETS}` 和 `${catkin_EXPORTED_TARGETS}`

## 13. 迁移原则

### 13.1 先迁接口定义，再迁控制面实现

迁移顺序必须是：

1. 建新包骨架
2. 先迁 `msg/srv/action`
3. 调整 `chassis_ctrl` 编译依赖
4. 再迁 `topics_transfer.cpp`
5. 再迁 `launch`
6. 最后迁前端静态资源和 web 脚本

原因是 `msg/srv/action` 是整个工程的编译基座，必须优先稳定。

### 13.2 ROS 名称保持兼容

本次重构期间，以下内容应尽量保持不变：

- topic 名称
- service 名称
- action 名称
- 主要节点名
- 前端按钮行为

这样可以把风险主要限制在“包归属与编译依赖”层面。

### 13.3 不保留双份接口定义

迁移完成后，不允许：

- `chassis_ctrl/msg` 和 `robot_interface_hub/msg` 双份共存
- `chassis_ctrl/srv` 和 `robot_interface_hub/srv` 双份共存
- `chassis_ctrl/action` 和 `robot_interface_hub/action` 双份共存

因为双份定义会导致后续类型漂移和维护混乱。

## 14. 测试与验证要求

这次重构至少需要覆盖三类验证：

### 14.1 构建验证

- 新包 `robot_interface_hub` 能独立生成 `msg/srv/action`
- `chassis_ctrl` 能成功链接新接口包
- 全工作区 `catkin_make` 通过

### 14.2 静态结构验证

需要新增或调整测试，确认：

- 新包存在
- `msg/srv/action/launch/web` 已迁入新包
- `topics_transfer.cpp` 已从 `chassis_ctrl` 迁出
- `run.launch` / `api.launch` 已迁到新包
- `workspace_picker_web_server.py` 指向新包下的 `web/`

### 14.3 运行链验证

至少验证以下链路不变：

- 新前端能正常打开
- `topictransNode` 仍能把前端 topic 转成后端 service
- “开始执行层”
- “直接执行账本测试”
- “扫描建图”

## 15. 风险

### 15.1 消息类型大迁移风险

这是本次最大风险点。  
任何一个 `#include`、`serviceClient`、Python import 没切干净，都会导致编译或运行失败。

### 15.2 Launch 路径失效风险

当前 launch 中大量使用 `$(find chassis_ctrl)`。迁移后如果不统一改，会直接拉不起来。

### 15.3 前端资源路径风险

当前前端目录是工作区独立目录，不是 ROS 包内部目录。迁移进新包后，所有静态资源定位逻辑都要同步收口。

## 16. 结论

本次采用单一控制面包方案：

- 新建 `robot_interface_hub`
- 把全局接口定义、前端、前端 TCP/API、launch 全部迁入新包
- `chassis_ctrl` 收口为执行实现包

这样做虽然不如“纯接口包 + bringup 包”那么教科书化，但它最贴合当前项目诉求：

- 对外入口统一
- 新前端相关内容集中
- 整个大工程的控制面被清晰抽离
- 为后续继续细拆 `chassis_ctrl` 驱动层/算法层/应用层留出边界
