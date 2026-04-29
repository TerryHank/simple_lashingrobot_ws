# ROS 动态 API 网关接入

## 定位

ROS 动态 API 网关用于把运行中的 ROS Topic、Service 和 Action 等价转换为 HTTP 与 MQTT API。它不是前端专用桥，也不替代 `rosbridge_websocket`；它是挂在 `api.launch` 旁路上的通用对外接口层。

这层的目标是：后续 ROS 里新增话题、服务或动作后，只要类型已经由 catkin 正常生成并被当前环境 `source` 到，网关就能在运行时发现并暴露出去，不需要再为每个业务接口手写桥接代码。

当前实现分支为：

```text
feature/ros-api-gateway
```

合入后主要代码落点为：

```text
src/tie_robot_web/scripts/ros_api_gateway.py
src/tie_robot_web/test/test_ros_api_gateway.py
src/tie_robot_bringup/launch/api.launch
```

## 架构

```text
外部 HTTP 客户端
-> ros_api_gateway.py
-> rospy / rosservice / actionlib
-> ROS graph
```

```text
外部 MQTT broker
<-> ros_api_gateway.py MQTT client
<-> rospy / rosservice / actionlib
<-> ROS graph
```

网关只作为 MQTT client 连接外部 broker，默认地址是：

```text
127.0.0.1:1883
```

broker 由 Mosquitto、EMQX 或现场已有 MQTT 服务管理，ROS 网关不内置 broker。

## 启动方式

合入实现分支后，`api.launch` 会增加动态 API 网关开关。典型启动方式：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_bringup api.launch start_ros_api_gateway:=true
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `start_ros_api_gateway` | `true` | 是否启动动态 API 网关 |
| `ros_api_gateway_http_enabled` | `true` | 是否启用 HTTP API |
| `ros_api_gateway_http_host` | `127.0.0.1` | HTTP 监听地址 |
| `ros_api_gateway_http_port` | `18080` | HTTP 监听端口 |
| `ros_api_gateway_mqtt_enabled` | `true` | 是否启用 MQTT client |
| `ros_api_gateway_mqtt_host` | `127.0.0.1` | MQTT broker 地址 |
| `ros_api_gateway_mqtt_port` | `1883` | MQTT broker 端口 |
| `ros_api_gateway_mqtt_prefix` | `tie_robot/ros` | MQTT 主题前缀 |
| `ros_api_gateway_read_only` | `false` | 只读模式，禁止反向发布、服务调用和 Action goal |

现场部署时，如果只想先开放查询能力，可以使用：

```bash
roslaunch tie_robot_bringup api.launch \
  start_ros_api_gateway:=true \
  ros_api_gateway_read_only:=true
```

## HTTP API

HTTP 默认监听：

```text
http://127.0.0.1:18080
```

基础接口：

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| `GET` | `/health` | 网关健康状态 |
| `GET` | `/api/topics` | 当前 Topic 清单 |
| `GET` | `/api/services` | 当前 Service 清单 |
| `GET` | `/api/actions` | 当前 Action 清单 |
| `GET` | `/api/types/<type>` | 消息或服务类型结构 |

Topic 查询与发布：

```text
GET  /api/topic/<ros_topic>/latest
POST /api/topic/<ros_topic>/publish
```

示例：

```bash
curl http://127.0.0.1:18080/api/topic/cabin/area_progress/latest
```

发布已有 Topic：

```json
{
  "message": {
    "data": true
  }
}
```

发布尚未出现在 ROS graph 中的 Topic 时，需要额外提供类型：

```json
{
  "type": "std_msgs/Bool",
  "message": {
    "data": true
  }
}
```

Service 调用：

```text
POST /api/service/<ros_service>/call
```

请求体：

```json
{
  "request": {}
}
```

Action 调用：

```text
POST /api/action/<ros_action>/goal
POST /api/action/<ros_action>/cancel
GET  /api/action/<ros_action>/state/<goal_id>
```

Action 会按 ROS1 actionlib 的标准 5 个 Topic 自动识别：

```text
<action>/goal
<action>/cancel
<action>/status
<action>/feedback
<action>/result
```

## MQTT API

默认 MQTT 前缀：

```text
tie_robot/ros
```

网关状态：

| 方向 | 主题 | 说明 |
| --- | --- | --- |
| 发布 | `tie_robot/ros/gateway/status` | 网关健康状态 |
| 发布 | `tie_robot/ros/gateway/inventory` | Topic、Service、Action 清单摘要 |

Topic 上行：

```text
tie_robot/ros/topic/<ros_topic>
```

示例：

```text
tie_robot/ros/topic/cabin/area_progress
```

消息格式：

```json
{
  "name": "/cabin/area_progress",
  "type": "tie_robot_msgs/AreaProgress",
  "stamp": 1770000000.123,
  "message": {}
}
```

Topic 下行：

```text
tie_robot/ros/publish/<ros_topic>
```

Service 调用：

```text
tie_robot/ros/service_call/<ros_service>
tie_robot/ros/service_response/<request_id>
```

Action 调用：

```text
tie_robot/ros/action_goal/<ros_action>
tie_robot/ros/action_cancel/<ros_action>
tie_robot/ros/action_status/<ros_action>/<goal_id>
tie_robot/ros/action_feedback/<ros_action>/<goal_id>
tie_robot/ros/action_result/<ros_action>/<goal_id>
```

## JSON 规则

- ROS 基础类型映射为 JSON number、boolean、string。
- ROS 数组映射为 JSON array。
- ROS 嵌套消息映射为 JSON object。
- `time` 和 `duration` 映射为 `{ "secs": 0, "nsecs": 0 }`。
- 请求体可以缺省字段，缺省字段使用 ROS 消息类默认值。
- 额外字段默认报错，避免字段拼错后被静默丢弃。

统一成功响应：

```json
{
  "ok": true,
  "data": {}
}
```

统一错误响应：

```json
{
  "ok": false,
  "error": "message",
  "detail": "optional detail"
}
```

## 部署边界

动态 API 网关默认能覆盖当前 ROS graph。进入现场前建议按用途收紧 allow / deny 规则，或开启只读模式。

推荐策略：

- 内网调试：HTTP 监听 `127.0.0.1`，通过 SSH 隧道访问。
- 现场集成：MQTT 连接固定 broker，broker 侧做鉴权和 ACL。
- 只读看板：开启 `read_only`，只允许 Topic 和状态查询。
- 生产控制：只放行明确需要的 Service 与 Action，避免把整套 ROS graph 暴露给外部系统。
