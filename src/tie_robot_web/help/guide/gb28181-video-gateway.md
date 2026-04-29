# GB28181 视频接入

## 定位

GB28181 视频接入用于把当前工程里的 ROS 视频图层包装成国标虚拟摄像机通道。它不依赖 ZLMediaKit，也不把 ROS 图像裸推给平台，而是在机器人侧模拟一个 GB28181 设备端。

当前实现分支为：

```text
feature/gb28181-video-layers
```

合入后主要代码落点为：

```text
src/tie_robot_gb28181/
src/tie_robot_gb28181/config/gb28181_device.yaml
src/tie_robot_bringup/launch/api.launch
```

## 协议链路

GB28181 首版按「设备注册后等待平台点播」实现：

```text
ROS 图像订阅
-> 图像可视化转换
-> BGR 视频帧
-> H.264 编码
-> MPEG-PS 封装
-> RTP/UDP 打包
-> GB28181 上级平台
```

信令侧：

```text
GB28181 设备节点
-> SIP REGISTER
-> Keepalive
-> Catalog 响应
-> INVITE / SDP 解析
-> ACK 后启动 RTP
-> BYE 后停止 RTP
```

第一版范围：

- SIP UDP
- REGISTER + 401 Digest 鉴权
- Keepalive
- Catalog 返回虚拟通道
- INVITE / SDP 解析
- ACK 后按需推流
- BYE 后停流
- H.264 + MPEG-PS + RTP/UDP

暂不覆盖：

- RTP/TCP
- 音频
- 回放
- 录像查询
- 云台控制
- 语音对讲
- 多级目录级联

## 视频通道映射

默认把当前 7 路视频或可视化图层映射为 7 个国标通道：

| 国标通道名 | ROS Topic | 可视化方式 |
| --- | --- | --- |
| `ROS IR Image` | `/Scepter/ir/image_raw` | 灰度图 |
| `ROS Color Image` | `/Scepter/color/image_raw` | 彩色图 |
| `ROS Depth Image` | `/Scepter/depth/image_raw` | 深度归一化 |
| `ROS Detection Result` | `/pointAI/result_image_raw` | 彩色图 |
| `ROS PR-FPRG Result` | `/pointAI/manual_workspace_s2_result_raw` | 彩色图 |
| `ROS World Coord Filtered` | `/Scepter/worldCoord/world_coord` | 世界坐标幅值可视化 |
| `ROS World Coord Raw` | `/Scepter/worldCoord/raw_world_coord` | 世界坐标幅值可视化 |

默认设备 ID 与通道 ID 口径：

```text
设备 ID：34020000001320000001
通道 1：34020000001320000002
通道 2：34020000001320000003
通道 3：34020000001320000004
通道 4：34020000001320000005
通道 5：34020000001320000006
通道 6：34020000001320000007
通道 7：34020000001320000008
```

## 配置文件

合入实现分支后，默认配置位于：

```text
src/tie_robot_gb28181/config/gb28181_device.yaml
```

核心配置分为两部分。

SIP 设备参数：

```yaml
sip:
  server_ip: "<上级平台SIP_IP>"
  server_port: 5060
  server_id: "<平台国标ID>"
  domain: "<国标域>"
  device_id: "34020000001320000001"
  password: "<接入密码>"
  local_ip: "<本机可达IP>"
  local_port: 5060
```

媒体参数：

```yaml
media:
  ffmpeg_path: "ffmpeg"
  max_payload_size: 1200
  local_rtp_port: 0
  profile: "baseline"
  preset: "ultrafast"
  tune: "zerolatency"
```

通道参数：

```yaml
channels:
  - channel_id: "34020000001320000003"
    name: "ROS Color Image"
    topic: "/Scepter/color/image_raw"
    visualization: "color"
    width: 1280
    height: 720
    fps: 15
    bitrate: 2000000
    gop: 30
```

## 启动方式

独立启动：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_gb28181 gb28181_device.launch
```

通过 API 启动链路挂载：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_bringup api.launch start_gb28181:=true
```

如果使用现场配置：

```bash
roslaunch tie_robot_bringup api.launch \
  start_gb28181:=true \
  gb28181_config_file:=/path/to/gb28181_device.yaml
```

## 上级平台接入步骤

这一节面向上级国标平台、安防平台或集成方工作人员。目标是让平台能看到本机注册上来的虚拟设备，并点播本机 ROS 图像通道。

### 第 1 步：确认双方网络

先确认本机和上级平台在网络上互通。

平台侧需要告诉本机：

| 项目 | 示例 | 说明 |
| --- | --- | --- |
| SIP 服务器 IP | 平台提供 | 上级平台 SIP 监听地址 |
| SIP 服务器端口 | `5060` | 通常是 UDP 5060 |
| 平台国标 ID | 平台提供 | 对应配置里的 `server_id` |
| 国标域 | `3402000000` | 对应配置里的 `domain` |
| 设备接入密码 | 平台提供 | 平台给设备分配的 Digest 密码 |
| 媒体接收方式 | `RTP/AVP` | 第一版只支持 RTP over UDP |
| 媒体格式 | `PS/90000` | 第一版按 PS over RTP 推流 |

本机需要告诉平台：

| 项目 | 默认值 | 说明 |
| --- | --- | --- |
| 设备国标 ID | `34020000001320000001` | 平台添加设备时填写 |
| 设备 SIP IP | 现场本机网卡 IP | 例如 `192.168.6.99` |
| 设备 SIP 端口 | `5060` | 本机监听 SIP UDP 的端口 |
| 通道数量 | `7` | 对应 7 路 ROS 图像或可视化图层 |
| 传输协议 | `UDP` | 第一版不支持 RTP/TCP |
| 编码格式 | `H.264` | 封装为 MPEG-PS 后走 RTP |

网络放行要求：

- 平台到本机：允许 UDP `5060`，用于 SIP REGISTER、MESSAGE、INVITE、BYE。
- 本机到平台：允许 UDP `5060`，用于 SIP 响应、注册和心跳。
- 本机到平台媒体端口：允许 UDP 媒体流。媒体端口由平台 INVITE 的 SDP 决定，常见是平台侧动态端口或平台配置的 RTP 端口池。
- 如果中间有 NAT、防火墙或多网卡，`local_ip` 必须配置成平台能访问到的本机地址。

### 第 2 步：平台侧添加设备

在上级平台中新增一个 GB28181 设备，按平台 UI 填写：

```text
设备 ID：34020000001320000001
所属域：3402000000
传输协议：UDP
认证密码：与本机 sip.password 一致
设备类型：IPC / 编码设备 / 其他虚拟摄像机类型
```

不同平台字段名字可能不同，但本质上都要匹配：

```text
平台侧设备 ID == 本机 sip.device_id
平台侧域 == 本机 sip.domain
平台侧密码 == 本机 sip.password
平台侧传输协议 == UDP
```

### 第 3 步：本机修改国标配置

在本机配置文件中把平台信息写进去：

```yaml
sip:
  server_ip: "<上级平台SIP_IP>"
  server_port: 5060
  server_id: "<平台国标ID>"
  domain: "<国标域>"
  device_id: "34020000001320000001"
  password: "<接入密码>"
  local_ip: "192.168.6.99"
  local_port: 5060
```

`local_ip` 要写本机对平台可达的网卡 IP。如果帮助站地址是：

```text
http://192.168.6.99:8080/help/
```

通常可以先把 `local_ip` 设为 `192.168.6.99`，但最终以现场实际网卡和平台路由为准。

### 第 4 步：启动本机 GB28181 节点

启动前确认 ROS 后端、相机和图像话题已经正常：

```bash
rostopic list | grep -E 'Scepter|pointAI'
```

确认编码器可用：

```bash
ffmpeg -version
```

启动国标接入：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tie_robot_bringup api.launch start_gb28181:=true
```

如果使用单独配置文件：

```bash
roslaunch tie_robot_bringup api.launch \
  start_gb28181:=true \
  gb28181_config_file:=/path/to/gb28181_device.yaml
```

### 第 5 步：平台侧确认注册和目录

节点启动后，平台侧应看到：

```text
设备上线
心跳正常
目录查询成功
通道数量：7
```

平台目录中应出现这些通道：

| 通道 ID | 通道名 |
| --- | --- |
| `34020000001320000002` | `ROS IR Image` |
| `34020000001320000003` | `ROS Color Image` |
| `34020000001320000004` | `ROS Depth Image` |
| `34020000001320000005` | `ROS Detection Result` |
| `34020000001320000006` | `ROS PR-FPRG Result` |
| `34020000001320000007` | `ROS World Coord Filtered` |
| `34020000001320000008` | `ROS World Coord Raw` |

如果设备能上线但没有通道，优先检查平台是否发起 Catalog 查询，以及本机是否回复 Catalog XML。

### 第 6 步：平台侧点播

在平台上选择一个通道点击实时预览。平台应向本机发送 INVITE，SDP 里推荐是：

```text
m=video <平台媒体端口> RTP/AVP 96
a=rtpmap:96 PS/90000
y=<SSRC>
```

本机收到 ACK 后才会开始订阅对应 ROS 图像并推送 RTP。也就是说，正常情况下本机不会开机就持续推流，而是平台点播后才开始发流。

### 第 7 步：验收标准

平台工作人员可以按下面顺序验收：

1. 设备状态为在线。
2. 平台能看到 7 个通道。
3. 点播 `ROS Color Image` 有画面。
4. 点播 `ROS IR Image` 有灰度画面。
5. 点播 `ROS Depth Image` 有归一化深度画面。
6. 点播 `ROS PR-FPRG Result` 能看到当前识别结果画面。
7. 停止预览后，平台发送 BYE，本机停止该路推流。

## 平台点播时序

上级平台看到目录后点播某一路通道，节点按下面时序工作：

```text
平台 -> 设备：INVITE
设备：解析 SDP，得到媒体 IP、端口、payload type、SSRC
设备 -> 平台：100 Trying
设备 -> 平台：200 OK
平台 -> 设备：ACK
设备：订阅对应 ROS 图像，启动编码和 PS-RTP/UDP 发送
平台 -> 设备：BYE
设备：停止订阅、编码和 RTP 发送
设备 -> 平台：200 OK
```

SDP 里最关键的字段：

```text
c=IN IP4 <媒体目标 IP>
m=video <媒体目标端口> RTP/AVP <payload type>
a=rtpmap:<payload type> PS/90000
y=<SSRC>
```

第一版只处理 `RTP/AVP`，也就是 RTP over UDP。

## 编码依赖

媒体编码默认使用外部 `ffmpeg` 命令：

```text
BGR raw stdin
-> libx264
-> yuv420p
-> MPEG-PS stdout
-> RTP packetizer
```

部署前确认：

```bash
ffmpeg -version
```

如果现场没有 `ffmpeg`，需要安装它，或把 `media.ffmpeg_path` 指向等价编码器。节点缺少编码器时会在点播启动流时给出明确错误。

推荐第一版编码参数：

- H.264 Baseline
- 1280x720 或更低
- 10 / 15 fps
- GOP 约 2 秒
- 关闭 B 帧
- 只发视频，不带音频

## 调试建议

先验证三件事：

1. 平台能看到设备和 7 个通道。
2. 平台点播放时，节点收到 INVITE 并能解析出 SDP。
3. ACK 后，平台媒体端口能收到 RTP/UDP 包。

抓包过滤建议：

```text
sip or udp.port == 5060
```

```text
udp.port == <平台 SDP 里的媒体端口>
```

如果目录能出现但没有画面，优先检查：

- `ffmpeg` 是否可执行。
- 设备端是否能访问平台 SDP 里的媒体 IP。
- 防火墙是否放行媒体 UDP 端口。
- 平台 SDP 是否要求 `TCP/RTP/AVP`，第一版不支持 TCP。
- 平台是否要求固定 SSRC 或特定 payload type。

## 与普通视频推流的区别

GB28181 不是固定地址裸推流。标准流程是：

```text
设备先注册
平台查目录
平台点播
设备按 INVITE 的 SDP 发送媒体
平台 BYE 后设备停流
```

所以这层应该被理解为「ROS 图像到 GB28181 虚拟摄像机」，不是「ROS 图像到 RTMP/RTSP 推流」。
