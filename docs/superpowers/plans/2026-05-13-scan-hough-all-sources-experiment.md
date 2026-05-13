# 扫描全底图 Hough 交点实验实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 在不改扫描主链的前提下，新增离线实验，把当前所有扫描底图提取为 Hough 直线图，并从横纵线交点生成候选绑扎点。

**架构：** 新增 ROS-free 纯算法助手负责响应图二值化、骨架化、HoughLinesP、横纵线聚类和交点生成；新增一个 live/offline 报告脚本复用现有 `current_scan_all_sources_report.py` 的抓帧与全底图构建能力，输出 HTML、PNG 和 `summary.json`。运行态 `/pointAI/process_image` 与 `scan_surface_dp.py` 不接入该实验。

**技术栈：** Python、OpenCV、NumPy、unittest、现有 Surface-DP 工具链。

---

### 任务 1：Hough 交点纯算法模块

**文件：**
- 创建：`src/tie_robot_perception/tools/scan_hough_intersection_experiment.py`
- 测试：`src/tie_robot_perception/test/test_scan_hough_intersection_experiment.py`

- [ ] **步骤 1：编写失败的测试**

测试合成响应图里的 3 条竖线和 2 条横线能被聚类为 6 个交点；空图返回 0 条线、0 个点。

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_perception.test.test_scan_hough_intersection_experiment -v`
预期：FAIL，报错缺少 `scan_hough_intersection_experiment` 模块。

- [ ] **步骤 3：实现最少算法**

实现：
- `extract_hough_intersections(response_map, valid_mask=None, ...)`
- `render_hough_overlay(response_map, valid_mask, hough_result, label)`
- 返回 `vertical_lines`、`horizontal_lines`、`intersections`、`raw_line_count`、`binary_pixels`、`threshold`。

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.tie_robot_perception.test.test_scan_hough_intersection_experiment -v`
预期：PASS。

### 任务 2：全底图 Hough 报告脚本

**文件：**
- 创建：`src/tie_robot_perception/tools/current_scan_all_sources_hough_report.py`
- 测试：`src/tie_robot_perception/test/test_scan_hough_intersection_experiment.py`

- [ ] **步骤 1：补充失败测试**

用源码结构测试约束：
- 报告脚本复用 `build_offline_response_maps` 和 `BASE_SOURCE_ORDER`。
- 报告脚本调用 `extract_hough_intersections`。
- 报告脚本默认输出 `summary.json` 和 Hough overlay 图片。

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src.tie_robot_perception.test.test_scan_hough_intersection_experiment -v`
预期：FAIL，报错缺少报告脚本。

- [ ] **步骤 3：实现报告脚本**

实现 CLI：
- `--output-dir`
- `--timeout`
- `--threshold-percentile`
- `--hough-threshold-percentile`
- `--response-source`
- `--no-depth-ir-fallback`

输出：
- `images/*_hough_rectified.png`
- `images/00_input_workspace.png`
- `summary.json`
- `index.html`

- [ ] **步骤 4：运行测试验证通过**

运行：`python3 -m unittest src.tie_robot_perception.test.test_scan_hough_intersection_experiment -v`
预期：PASS。

### 任务 3：验证与试跑

**文件：**
- 修改：无运行态文件。

- [ ] **步骤 1：语法验证**

运行：
`python3 -m py_compile src/tie_robot_perception/tools/scan_hough_intersection_experiment.py src/tie_robot_perception/tools/current_scan_all_sources_hough_report.py`

- [ ] **步骤 2：定向单测**

运行：
`python3 -m unittest src.tie_robot_perception.test.test_scan_hough_intersection_experiment -v`

- [ ] **步骤 3：尝试生成一次报告**

优先使用当前 ROS 帧：
`python3 src/tie_robot_perception/tools/current_scan_all_sources_hough_report.py --output-dir .debug_frames/current_scan_all_sources_hough_$(date +%Y%m%d_%H%M%S)`

若 ROS 帧不可用，报告阻塞原因，不改主链。
