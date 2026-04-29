# Legacy RANSAC + Hough pointAI

这里保存的是旧版 `pointAI` 绑扎点识别链路的运行代码快照。

- `matrix_preprocess.py`：旧的深度二值化、ROI 裁剪、RANSAC/Hough 线段检测与点输出流程。
- `matrix_selection.py`：与旧流程配套的矩阵点筛选工具快照。

2026-04-29 起，工程主视觉方案切换为方向自适应 PR-FPRG 手动工作区 S2：

1. 在手动工作区内做透视展开。
2. 按钢筋真实摆放方向估计两组周期线族。
3. 对峰值线做连续钢筋条验证和间距剔除。
4. 用两组线方程求交并发布绑扎点。

本目录只用于追溯旧实现，不参与 ROS 节点运行，也不应被 `processor.py` 或 `process_image_service.py` 重新绑定。
