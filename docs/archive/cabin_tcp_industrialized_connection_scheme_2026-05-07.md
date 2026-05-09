# 索驱 TCP 工业化连接方案归档

归档时间：2026-05-07

## 背景

本方案是在排查索驱前端连接状态闪烁、心跳偶发超时、局域网响应时间误判和索驱 TCP 对端主动关连接后形成的临时改造方案。用户随后明确要求：当前方案存档，运行实现回退，并改为参考 `slam/v35` 展示工程的索驱连接驱动层方案：

`/home/hyq-/lashingrobotROS/src`

## 当前方案内容

- 在 `CabinTcpTransport` 增加连接模式：`persistent`、`short_transaction`、`auto_detect`。
- 将 TCP connect 成功和应用层协议 round-trip 成功拆开：connect 后先进入 `connecting`，收到协议回包后才进入 `ready`。
- 增加链路统计：`last_connect_ms`、`last_send_ms`、`last_first_byte_ms`、`last_full_frame_ms`、`peer_close_count`、`reconnect_count`、`auto_short_transaction_detected`。
- 在 diagnostics 中暴露原始 transport 状态、可见状态托底、瞬时错误、连接模式和链路耗时。
- 对 heartbeat/state 查询使用较短的首字节/总窗超时，并尝试在 `auto_detect` 下识别对端短事务行为。

## 涉及文件

- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_tcp_transport.hpp`
- `src/tie_robot_hw/src/driver/cabin_tcp_transport.cpp`
- `src/tie_robot_hw/include/tie_robot_hw/driver/cabin_driver.hpp`
- `src/tie_robot_hw/src/driver/cabin_driver.cpp`
- `src/tie_robot_process/src/suoquNode.cpp`
- `src/tie_robot_process/test/test_cabin_tcp_transport_contract.py`
- `src/tie_robot_process/test/test_motion_chain_signal_guard.py`
- `docs/agent_memory/session_log.md`
- `docs/agent_memory/current.md`

## 已验证数据

- 单元测试：
  `python3 -m unittest src.tie_robot_process.test.test_cabin_tcp_transport_contract src.tie_robot_process.test.test_motion_chain_signal_guard src.tie_robot_process.test.test_cabin_protocol_contract`
  结果：`Ran 71 tests`，`OK`。
- 构建：
  `source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=`
  结果：构建通过。
- 现场只重启索驱驱动节点，未下发运动命令。
- 30 秒 diagnostics：
  `level=0` 为 `30/30`，前端可见状态 `ready=30/30`，socket 维持单个 `ESTAB`。
- 链路耗时采样：
  `last_send_ms p50≈0.008ms`，`last_first_byte_ms/last_full_frame_ms p50≈74.8ms`。

## 回退原因

用户判断该方案偏离 `slam/v35` 展示工程的索驱连接驱动层连接方式，要求存档后回退刚才的连接层方案，并按参考工程一致的连接生命周期实现。后续运行代码不再保留本方案新增的连接模式、短事务/自动识别、链路统计和 diagnostics 托底字段。

