# ROS 动态 API 网关设计

## 背景

当前工程的 `api.launch` 已启动 `rosbridge_websocket`、`rosapi`、`tf2_web_republisher` 和面向前端业务按钮的 `webActionBridgeNode`。这套链路能服务浏览器前端，但 `webActionBridgeNode` 对 Action 和 Service 仍是手写桥接；新增 ROS 话题、服务或动作后，如果要对外提供 HTTP / MQTT API，还需要继续修改桥接层。

本设计新增一个独立的 ROS 动态 API 网关，使工程发布的话题、消息、动作、服务可以在运行时等价映射到 HTTP 和 MQTT。网关不替代现有 `rosbridge_websocket` 和前端业务桥，而是作为旁路通用层挂入 `api.launch`。

## 目标

- 动态发现当前 ROS graph 中的 Topic、Service 和 Action，不为具体业务接口写死类型。
- 将 ROS Topic 映射为 HTTP 查询 / 发布接口和 MQTT 上行 / 下行主题。
- 将 ROS Service 映射为 HTTP POST 调用和 MQTT 请求 / 响应。
- 将 ROS Action 按 ROS1 Action 的标准 5 个 Topic 自动识别，提供 goal、cancel、status、feedback、result 能力。
- 新增 ROS 话题、服务或动作后，只要其类型在当前 ROS 环境可加载，网关不需要改代码。
- 默认连接外部 MQTT broker，地址为 `127.0.0.1:1883`，网关本身不启动 broker。
- 保留 allow / deny 过滤参数，默认可覆盖全 ROS graph，现场部署可收紧。

## 非目标

- 不重写 `rosbridge_server`，也不改变现有前端和 `webActionBridgeNode` 的行为。
- 不新增前端 UI，不修改 `src/tie_robot_web/frontend`，因此无需重建静态页面。
- 不提供鉴权系统的完整实现，只在网关侧预留 bind host、allow / deny、只读模式等部署级保护。
- 不支持无法通过 `roslib.message` 加载的自定义消息类型；这类类型需要先由 catkin 正常生成并 source 到当前环境。

## 推荐架构

新增脚本：

- `src/tie_robot_web/scripts/ros_api_gateway.py`

新增测试：

- `src/tie_robot_web/test/test_ros_api_gateway.py`

修改启动与安装：

- `src/tie_robot_web/CMakeLists.txt`：安装 `ros_api_gateway.py`
- `src/tie_robot_web/package.xml`：声明运行依赖 `python3-paho-mqtt`
- `src/tie_robot_bringup/launch/api.launch`：增加可关闭的 `ros_api_gateway` 节点

运行时结构：

```text
外部 HTTP 客户端
-> ros_api_gateway.py HTTP server
-> rospy / rosservice / actionlib
-> ROS graph

外部 MQTT broker
<-> ros_api_gateway.py MQTT client
<-> rospy / rosservice / actionlib
<-> ROS graph
```

HTTP 使用 Python 标准库 `http.server.ThreadingHTTPServer`，避免引入 Flask / aiohttp。MQTT 使用 `paho.mqtt.client`，通过系统包 `python3-paho-mqtt` 提供。

## 运行参数

全部参数挂在私有命名空间：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `~http_enabled` | `true` | 是否启动 HTTP API |
| `~http_host` | `127.0.0.1` | HTTP 监听地址，默认只允许本机访问 |
| `~http_port` | `18080` | HTTP 监听端口 |
| `~mqtt_enabled` | `true` | 是否启用 MQTT client |
| `~mqtt_host` | `127.0.0.1` | 外部 MQTT broker 地址 |
| `~mqtt_port` | `1883` | 外部 MQTT broker 端口 |
| `~mqtt_client_id` | `tie_robot_ros_api_gateway` | MQTT client ID |
| `~mqtt_username` | `""` | MQTT 用户名 |
| `~mqtt_password` | `""` | MQTT 密码 |
| `~mqtt_prefix` | `tie_robot/ros` | MQTT 主题前缀 |
| `~discovery_hz` | `1.0` | ROS graph 刷新频率 |
| `~allow_topics` | `["*"]` | 允许桥接的话题 glob |
| `~deny_topics` | `[]` | 禁止桥接的话题 glob |
| `~allow_services` | `["*"]` | 允许桥接的服务 glob |
| `~deny_services` | `[]` | 禁止桥接的服务 glob |
| `~allow_actions` | `["*"]` | 允许桥接的动作 glob |
| `~deny_actions` | `[]` | 禁止桥接的动作 glob |
| `~read_only` | `false` | 只读模式，禁止 Topic 发布、Service 调用和 Action goal |
| `~topic_sample_queue_size` | `10` | ROS Topic 订阅队列 |
| `~publish_latched` | `false` | HTTP / MQTT 反向发布到 ROS 时是否 latch |

## JSON 表达

网关使用结构化 JSON 表达 ROS 消息：

- ROS 基础类型映射为 JSON number、boolean、string。
- ROS 数组映射为 JSON array。
- ROS 嵌套消息映射为 JSON object。
- `time` / `duration` 映射为 `{ "secs": 0, "nsecs": 0 }`。
- `uint8[]` 可接受 JSON number array；后续可扩展 base64，但首版不默认启用。
- 请求体允许缺省字段，缺省字段使用 ROS 生成消息类的默认值。
- 额外字段默认报错，避免误拼字段静默丢失。

统一错误响应：

```json
{
  "ok": false,
  "error": "message",
  "detail": "optional detail"
}
```

统一成功响应：

```json
{
  "ok": true,
  "data": {}
}
```

## HTTP API

### 健康检查

- `GET /health`

返回网关状态、ROS master 可用性、MQTT 连接状态和当前发现数量。

### 清单接口

- `GET /api/topics`
- `GET /api/services`
- `GET /api/actions`
- `GET /api/types/<type_name>`

Topic 清单返回名称、类型、当前订阅状态、最近消息时间和是否通过 allow / deny。Service 清单返回名称、类型、请求字段和响应字段。Action 清单返回 action 名称、action 类型，以及 goal / feedback / result 类型。

### Topic 接口

- `GET /api/topic/<encoded_topic_name>/latest`
- `POST /api/topic/<encoded_topic_name>/publish`

`encoded_topic_name` 使用 URL path 形式保存原始 Topic，如 `/cabin/area_progress` 对应 `/api/topic/cabin/area_progress/latest`。发布请求体为 ROS 消息 JSON：

```json
{
  "message": {
    "data": true
  }
}
```

如果 Topic 尚未存在，发布接口必须带 `type`：

```json
{
  "type": "std_msgs/Bool",
  "message": {
    "data": true
  }
}
```

### Service 接口

- `POST /api/service/<encoded_service_name>/call`

请求体：

```json
{
  "request": {}
}
```

响应体：

```json
{
  "ok": true,
  "data": {
    "response": {}
  }
}
```

### Action 接口

- `POST /api/action/<encoded_action_name>/goal`
- `POST /api/action/<encoded_action_name>/cancel`
- `GET /api/action/<encoded_action_name>/state/<goal_id>`

goal 请求体：

```json
{
  "goal": {}
}
```

Action 以 actionlib 客户端方式发送 goal，网关为每个 goal 生成 `goal_id`，缓存状态、feedback 和 result。状态查询返回当前 goal 状态、最近 feedback 和最终 result。

## MQTT API

默认前缀为 `tie_robot/ros`。ROS 名称中的 `/` 保留为 MQTT 层级分隔符；例如 `/cabin/area_progress` 映射为 `tie_robot/ros/topic/cabin/area_progress`。

### 网关状态

- 发布：`tie_robot/ros/gateway/status`
- 发布：`tie_robot/ros/gateway/inventory`

`status` 周期发布网关健康状态。`inventory` 在发现结果变化时发布 Topic / Service / Action 清单摘要。

### Topic 上行

- 发布：`tie_robot/ros/topic/<ros_topic>`

网关订阅 ROS Topic 后，将最新 ROS 消息转成 JSON 并发布到对应 MQTT 主题。

消息格式：

```json
{
  "name": "/cabin/area_progress",
  "type": "tie_robot_msgs/AreaProgress",
  "stamp": 1770000000.123,
  "message": {}
}
```

### Topic 下行

- 订阅：`tie_robot/ros/publish/<ros_topic>`

MQTT 客户端向该主题发布 JSON，即可反向发布到 ROS Topic。Topic 已存在时自动使用发现到的类型；Topic 不存在时请求体必须提供 `type`。

```json
{
  "type": "std_msgs/Bool",
  "message": {
    "data": true
  }
}
```

### Service 调用

- 订阅：`tie_robot/ros/service_call/<ros_service>`
- 发布：`tie_robot/ros/service_response/<request_id>`

请求体：

```json
{
  "request_id": "client-generated-id",
  "request": {}
}
```

响应体：

```json
{
  "request_id": "client-generated-id",
  "ok": true,
  "response": {}
}
```

### Action 调用

- 订阅：`tie_robot/ros/action_goal/<ros_action>`
- 订阅：`tie_robot/ros/action_cancel/<ros_action>`
- 发布：`tie_robot/ros/action_status/<ros_action>/<goal_id>`
- 发布：`tie_robot/ros/action_feedback/<ros_action>/<goal_id>`
- 发布：`tie_robot/ros/action_result/<ros_action>/<goal_id>`

Action goal 请求体：

```json
{
  "request_id": "client-generated-id",
  "goal": {}
}
```

响应先通过 `action_status` 返回 accepted / rejected，再通过 feedback / result 主题持续推送状态。

## 动态发现

Topic 发现通过 ROS master 获取当前 `name -> type` 映射。网关对允许的 Topic 建立订阅，并维护最近一条消息缓存。发现到 Topic 删除或类型变化时，释放旧订阅并重新建立。

Service 发现通过 `rosservice` 获取服务名称和类型。调用前再次解析服务类，避免启动早期服务尚未 ready 导致缓存错误。

Action 发现基于 ROS1 actionlib 约定。若存在以下 Topic 集合，则识别为一个 Action：

- `<name>/goal`
- `<name>/cancel`
- `<name>/status`
- `<name>/feedback`
- `<name>/result`

Action 类型从 `<name>/goal` 的消息类型推导。例如 `tie_robot_msgs/StartGlobalWorkTaskActionGoal` 推导出 `tie_robot_msgs/StartGlobalWorkTaskAction`。

## 错误处理

- ROS master 不可用时，HTTP `/health` 返回 `ros_master_ok=false`，MQTT 状态继续报告网关进程存活。
- MQTT broker 不可用时，节点不退出，按退避策略重连；HTTP API 仍可用。
- 消息类型无法加载时，该接口标记为 `type_load_error`，不会阻塞其他接口。
- Service 调用超时返回 `ok=false`，错误信息包含服务名和超时秒数。
- Action goal 超时或被取消时，状态查询和 MQTT result 都返回 actionlib 状态码与文本。
- `read_only=true` 时，所有写操作返回 `403` 或 MQTT 错误响应。

## 安全边界

默认 HTTP 只绑定 `127.0.0.1`，MQTT 连接本机 broker。部署到现场网络前，应通过参数显式设置 bind host、broker 地址和 allow / deny 规则。

建议现场配置：

```xml
<arg name="ros_api_gateway_http_host" default="127.0.0.1" />
<arg name="ros_api_gateway_http_port" default="18080" />
<arg name="ros_api_gateway_mqtt_host" default="127.0.0.1" />
<arg name="ros_api_gateway_mqtt_port" default="1883" />
```

默认不屏蔽 `/rosout`、`/tf`、`/tf_static`，因为用户要求全 ROS graph 动态兼容。需要降低流量时，可用 deny 参数排除高频 Topic。

## 测试策略

单元测试覆盖纯 Python 逻辑：

- ROS 名称与 HTTP / MQTT 路径互转。
- allow / deny glob 过滤优先级。
- ROS 消息 JSON 序列化和反序列化，包括嵌套消息、数组、time / duration。
- Service 请求 / 响应 JSON 转换。
- Action 名称和类型推导。
- HTTP handler 路由解析。
- MQTT 请求主题解析和响应主题生成。

集成级验证覆盖：

- `python3 -m unittest src/tie_robot_web/test/test_ros_api_gateway.py`
- `catkin_make`
- 启动 `roscore` 后运行网关，确认 `/health` 可返回 ROS master 状态。

MQTT 端到端验证需要外部 broker。若本机没有 broker，测试只验证 MQTT topic 映射和 paho client 封装，不声明 broker 通信通过。

## 兼容性与迁移

该网关是新增旁路能力，不改变现有接口：

- `rosbridge_websocket` 继续服务前端。
- `webActionBridgeNode` 继续承载当前业务 Action / Service 桥。
- 当前前端 `topicRegistry.js` 不需要迁移。
- 后续新增 ROS 功能时，只要按标准 ROS Topic / Service / Action 发布，HTTP / MQTT 网关自动发现。

## 验收标准

- `api.launch` 可以通过参数开启 / 关闭 `ros_api_gateway`。
- HTTP `/health`、`/api/topics`、`/api/services`、`/api/actions` 可运行。
- 已存在 Topic 可以通过 HTTP latest 和 MQTT topic 上行获取最新消息。
- 已存在 Topic 可以通过 HTTP POST 和 MQTT `publish` 主题反向发布。
- 已存在 Service 可以通过 HTTP POST 和 MQTT `service_call` 调用。
- 标准 ROS Action 可以被自动发现，并支持 goal、cancel、feedback、result。
- 新增一个测试用 Topic / Service / Action 后，不修改网关代码即可出现在清单中。
