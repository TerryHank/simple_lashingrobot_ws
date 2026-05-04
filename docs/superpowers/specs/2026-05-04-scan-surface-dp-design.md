# 扫描层 Surface-DP 绑扎点识别设计

## 目标

把扫描层从 depth-only 频相生成网格推进到现场实验推荐的 `surface_dp_curve` 路线：以组合响应和实例融合响应为底图，经过 Hessian/Frangi 脊线增强、二值候选、骨架、补全钢筋面和 DP 曲线收束后输出曲线交点。旧 depth-only S2 保留为失败回退和对照链路。

## 范围

- 保持 `/pointAI/process_image request_mode=3`、发布话题、`PointsArray` 消息和前端触发链不变。
- 新增纯 Python 算法模块承载响应融合、补全面和 DP 交点逻辑，避免运行态导入 `tools/` 报告脚本。
- `manual_workspace_s2.py` 继续负责读取手动工作区、透视展开、原图反投影、查 raw_world 相机坐标和发布结果。
- `instance_graph junction` 只作为诊断与补召回候选，不作为主输出全量点源。

## 数据流

```text
depth_v + infrared_v + raw_world
-> manual workspace rectification
-> combined_response / fused_instance_response
-> Hessian + Frangi ridge enhancement
-> binary_candidate + skeleton diagnostics
-> completed_surface_mask / completed_surface_response
-> axis families on completed surface
-> DP curved line families
-> curve intersections
-> image pixels
-> raw camera world coords
```

## 成功标准

- 离线合成栅格测试中，曲线收束输出期望交点数量，并带有 `surface_dp_curve` 诊断字段。
- 运行入口优先调用新 Surface-DP 管线；当新管线失败或有效点不足时，回退旧 depth-only S2。
- 2026-05-04 固定 snapshot 的全量实验报告继续推荐 `04_surface_dp_curve`，作为回归对照。

## 错误处理

新管线缺 IR、缺 raw_world、线族不足、曲线交点不足或坐标查找失败时，不让扫描请求直接空跑；入口记录失败原因并调用旧 S2。旧 S2 仍失败时返回原有失败响应。
