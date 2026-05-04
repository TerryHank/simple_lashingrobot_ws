# 扫描层视觉算法知识库

这是一个可直接用 Obsidian 打开的知识库，用于整理当前扫描层视觉算法、现场误识别诊断、相关文献和后续改进路线。

## 快速入口

- [[10_Project/扫描层视觉算法总览]]
- [[20_Code/当前底图与话题]]
- [[40_Experiments/当前现场产物诊断]]
- [[30_Literature/文献卡片]]
- [[10_Project/改进路线]]

## 当前结论

当前扫描层视觉识别的底图主输入是 **深度图**，运行算法在手动工作区透视展开后的 rectified depth 上构造 depth-only 背景差分响应图，再用纵横 profile 做周期和相位估计。IR 图像主要用于显示和历史实验，不是当前扫描主链的底图。

## 资产

`assets/` 目录内复制了当前视觉流程报告中的关键图：

- `03_scan_raw_depth.png`
- `05_scan_rectified_depth.png`
- `06_scan_depth_response.png`
- `07_scan_axis_profiles.png`
- `08_scan_rectified_grid.png`
- `10_scan_final_overlay.png`

