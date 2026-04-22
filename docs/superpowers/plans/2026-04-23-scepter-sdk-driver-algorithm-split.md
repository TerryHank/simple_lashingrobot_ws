# Scepter 相机 SDK 驱动层与视觉算法层拆分实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 让 `tie_robot_perception` 中的 `scepter_camera` 回归纯相机底层驱动，并将 `worldCoord`、RANSAC 和深度转世界坐标服务迁移到独立视觉算法节点，同时保持 `pointAI.py` 现有输入话题口径不变。

**架构：** 保留现有 `tie_robot_perception` 单包结构，但将感知内部拆成两个可执行单元：`scepter_camera` 负责相机建连、拉流、基础图像与基础点云输出；新增 `scepter_world_coord_processor` 负责从深度图和内参构建 `world_coord`、`raw_world_coord`，并提供 `convert_depth_to_point_cloud` 服务。`pointAI.py` 继续消费 `/Scepter/worldCoord/*`。

**技术栈：** ROS 1 Noetic、C++14、OpenCV、PCL、dynamic_reconfigure、sensor_msgs、cv_bridge

---

## 文件结构

- 创建：`src/tie_robot_perception/include/tie_robot_perception/perception/scepter_world_coord_processor.hpp`
  - 独立视觉算法处理器类声明，负责订阅深度图、保存内参、发布 `worldCoord` 结果、提供转换服务。
- 创建：`src/tie_robot_perception/src/perception/scepter_world_coord_processor.cpp`
  - 处理器实现，承接从相机驱动层迁出的世界坐标生成和 RANSAC 去平面逻辑。
- 创建：`src/tie_robot_perception/src/perception/scepter_world_coord_processor_node.cpp`
  - 节点入口，实例化处理器并 `ros::spin()`。
- 创建：`src/tie_robot_perception/test/test_scepter_sdk_split.py`
  - 结构性回归测试，钉住“驱动不再含算法逻辑、launch 启动双节点、CMake 链接边界正确”。
- 修改：`src/tie_robot_perception/include/tie_robot_perception/camera/scepter_manager.hpp`
  - 删除 `worldCoord` 和服务相关成员，只保留驱动层需要的声明。
- 修改：`src/tie_robot_perception/src/camera/scepter_manager.cpp`
  - 删除 PCL / RANSAC / `worldCoord` / 转换服务实现，保留基础点云发布。
- 修改：`src/tie_robot_perception/CMakeLists.txt`
  - 新增 `scepter_world_coord_processor` 目标；让 `scepter_camera` 不再链接 PCL；仅新算法节点链接 PCL。
- 修改：`src/tie_robot_perception/launch/scepter_camera.launch`
  - 从只启动 `scepter_camera` 改为同时启动 `scepter_camera` 和 `scepter_world_coord_processor`。

## 任务 1：先用失败测试钉住拆分边界

**文件：**
- 创建：`src/tie_robot_perception/test/test_scepter_sdk_split.py`

- [ ] **步骤 1：编写失败的结构测试**

```python
#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
PACKAGE_DIR = WORKSPACE_SRC / "tie_robot_perception"


class ScepterSdkSplitTest(unittest.TestCase):
    def test_camera_manager_no_longer_contains_world_coord_or_convert_service(self):
        content = (PACKAGE_DIR / "src" / "camera" / "scepter_manager.cpp").read_text(
            encoding="utf-8"
        )
        for marker in (
            "worldCoord_pub_",
            "raw_worldCoord_pub_",
            "convertDepthToPointCloud(",
            "convert_depth_to_point_cloud",
            "SACSegmentation",
            "ExtractIndices",
            "pcl::",
        ):
            self.assertNotIn(marker, content)

    def test_camera_header_no_longer_exposes_algorithm_members(self):
        content = (
            PACKAGE_DIR / "include" / "tie_robot_perception" / "camera" / "scepter_manager.hpp"
        ).read_text(encoding="utf-8")
        for marker in (
            "ConvertDepthToPointCloud",
            "worldCoord_nh_",
            "worldCoord_pub_",
            "raw_worldCoord_pub_",
            "convertDepthToPointCloud(",
        ):
            self.assertNotIn(marker, content)

    def test_launch_starts_world_coord_processor(self):
        content = (PACKAGE_DIR / "launch" / "scepter_camera.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('type="scepter_world_coord_processor"', content)
        self.assertIn('name="scepter_world_coord_processor"', content)

    def test_cmake_splits_driver_and_algorithm_targets(self):
        content = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("add_executable(scepter_world_coord_processor", content)
        camera_block = re.search(
            r"target_link_libraries\\(scepter_camera(?P<body>.*?)\\n\\)",
            content,
            re.S,
        )
        self.assertIsNotNone(camera_block)
        self.assertNotIn("${PCL_LIBRARIES}", camera_block.group("body"))
        processor_block = re.search(
            r"target_link_libraries\\(scepter_world_coord_processor(?P<body>.*?)\\n\\)",
            content,
            re.S,
        )
        self.assertIsNotNone(processor_block)
        self.assertIn("${PCL_LIBRARIES}", processor_block.group("body"))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.tie_robot_perception.test.test_scepter_sdk_split -v
```

预期：FAIL，报错 `worldCoord_pub_`、`ConvertDepthToPointCloud`、`pcl::` 仍存在，且 `scepter_world_coord_processor` 目标尚未创建。

- [ ] **步骤 3：提交失败测试**

```bash
git add src/tie_robot_perception/test/test_scepter_sdk_split.py
git commit -m "test: add Scepter SDK split regression guards（任务 1/3）"
```

## 任务 2：纯净化 `scepter_camera` 驱动层

**文件：**
- 修改：`src/tie_robot_perception/include/tie_robot_perception/camera/scepter_manager.hpp`
- 修改：`src/tie_robot_perception/src/camera/scepter_manager.cpp`

- [ ] **步骤 1：从头文件删除算法层声明**

目标变更：

```cpp
// 删除
#include "tie_robot_perception/ConvertDepthToPointCloud.h"

// 删除成员
bool convertDepthToPointCloud(...);
ros::ServiceServer service_;
ros::NodeHandle ... convertDepth_service_nh_, worldCoord_nh_;
std::shared_ptr<image_transport::ImageTransport> ... worldCoord_it_;
std::shared_ptr<camera_info_manager::CameraInfoManager> ... worldCoord_info_;
ros::Publisher ... worldCoord_pub_, raw_worldCoord_pub_;
ros::Publisher ... worldCoord_cameraInfoPub_;
```

- [ ] **步骤 2：把 `publishCloudPoint(...)` 收成基础点云发布**

目标约束：

- 移除 `#include <pcl/...>` 相关头文件
- 不再生成 `coord_image`
- 不再运行 RANSAC / `ExtractIndices`
- 不再发布 `world_coord` 与 `raw_world_coord`
- 仅保留将深度帧转换为基础 `sensor_msgs::PointCloud2` 的逻辑

建议实现要点：

```cpp
#include <sensor_msgs/point_cloud2_iterator.h>

void ScepterManager::publishCloudPoint(...) {
    sensor_msgs::PointCloud2 output_msg;
    sensor_msgs::PointCloud2Modifier modifier(output_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(valid_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(output_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(output_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(output_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(output_msg, "b");

    // 写入所有有效点，不做平面剔除
}
```

- [ ] **步骤 3：删除驱动层服务注册和实现**

移除：

```cpp
service_ = convertDepth_service_nh_.advertiseService(...);
bool ScepterManager::convertDepthToPointCloud(...)
```

- [ ] **步骤 4：运行结构测试确认驱动层边界已收紧**

运行：

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.tie_robot_perception.test.test_scepter_sdk_split.ScepterSdkSplitTest.test_camera_manager_no_longer_contains_world_coord_or_convert_service src.tie_robot_perception.test.test_scepter_sdk_split.ScepterSdkSplitTest.test_camera_header_no_longer_exposes_algorithm_members -v
```

预期：PASS

- [ ] **步骤 5：提交驱动层拆分**

```bash
git add src/tie_robot_perception/include/tie_robot_perception/camera/scepter_manager.hpp src/tie_robot_perception/src/camera/scepter_manager.cpp
git commit -m "refactor: slim Scepter camera driver to hardware-only flow（任务 2/3）"
```

## 任务 3：新增视觉算法节点并接回现有话题口径

**文件：**
- 创建：`src/tie_robot_perception/include/tie_robot_perception/perception/scepter_world_coord_processor.hpp`
- 创建：`src/tie_robot_perception/src/perception/scepter_world_coord_processor.cpp`
- 创建：`src/tie_robot_perception/src/perception/scepter_world_coord_processor_node.cpp`
- 修改：`src/tie_robot_perception/CMakeLists.txt`
- 修改：`src/tie_robot_perception/launch/scepter_camera.launch`

- [ ] **步骤 1：创建处理器类声明**

建议骨架：

```cpp
class ScepterWorldCoordProcessor {
public:
    explicit ScepterWorldCoordProcessor(const std::string& camera_name = "Scepter");

private:
    void depthCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool convertDepthToPointCloud(
        tie_robot_perception::ConvertDepthToPointCloud::Request& req,
        tie_robot_perception::ConvertDepthToPointCloud::Response& res);

    bool has_depth_info_;
    sensor_msgs::CameraInfo latest_depth_info_;
    ros::NodeHandle depth_nh_;
    ros::NodeHandle world_coord_nh_;
    image_transport::ImageTransport depth_it_;
    image_transport::ImageTransport world_coord_it_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber depth_info_sub_;
    image_transport::Publisher world_coord_pub_;
    image_transport::Publisher raw_world_coord_pub_;
    ros::Publisher world_coord_camera_info_pub_;
    ros::ServiceServer convert_service_;
};
```

- [ ] **步骤 2：实现从深度图生成 `raw_world_coord`**

目标实现：

```cpp
float z_mm = static_cast<float>(depth_mm);
float x_mm = (u - cx) * z_mm / fx;
float y_mm = (v - cy) * z_mm / fy;
raw_coord_image.at<cv::Vec3f>(v, u) = cv::Vec3f(x_mm, y_mm, z_mm);
```

要求：

- 输入深度图编码按 `16UC1`
- 输出图像编码保持 `32FC3`
- 无效深度保留为 `0`

- [ ] **步骤 3：在算法节点内实现 RANSAC 去平面后再投回图像**

目标实现：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
// 收集有效点
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(0.008);
extract.setNegative(true);
// 将 non_plane_cloud 投影回 filtered_coord_image
```

要求：

- 行为口径尽量沿用旧 `publishCloudPoint(...)`
- `world_coord` 发布过滤后的坐标图
- `raw_world_coord` 发布未过滤的原始坐标图

- [ ] **步骤 4：把 `convert_depth_to_point_cloud` 服务迁到新节点**

使用最新深度相机内参按针孔模型直接换算：

```cpp
res.world_x = (req.x - cx) * req.depth / fx;
res.world_y = (req.y - cy) * req.depth / fy;
res.world_z = req.depth;
```

如果尚未收到 `CameraInfo`，返回 `false` 并打印 `ROS_ERROR`。

- [ ] **步骤 5：更新 `CMakeLists.txt` 和 `launch`**

要求：

- `add_executable(scepter_world_coord_processor ...)`
- `scepter_world_coord_processor` 链接 `${PCL_LIBRARIES}`
- `scepter_camera` 不再链接 `${PCL_LIBRARIES}`
- `scepter_camera.launch` 同时启动：

```xml
<node pkg="tie_robot_perception" type="scepter_camera" ... />
<node pkg="tie_robot_perception" type="scepter_world_coord_processor" name="scepter_world_coord_processor" output="screen" />
```

- [ ] **步骤 6：运行结构测试与编译验证**

运行：

```bash
source /opt/ros/noetic/setup.bash && python3 -m unittest src.tie_robot_perception.test.test_scepter_sdk_split -v
```

运行：

```bash
source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES="tie_robot_msgs;tie_robot_description;tie_robot_hw;tie_robot_perception;tie_robot_control;tie_robot_process;tie_robot_bringup;tie_robot_web" -j2
```

预期：

- 结构测试全部 PASS
- `scepter_camera` 和 `scepter_world_coord_processor` 都编译通过

- [ ] **步骤 7：提交算法节点迁移**

```bash
git add src/tie_robot_perception/include/tie_robot_perception/perception/scepter_world_coord_processor.hpp src/tie_robot_perception/src/perception/scepter_world_coord_processor.cpp src/tie_robot_perception/src/perception/scepter_world_coord_processor_node.cpp src/tie_robot_perception/CMakeLists.txt src/tie_robot_perception/launch/scepter_camera.launch
git commit -m "feat: move Scepter world coordinate processing out of camera SDK（任务 3/3）"
```

## 自检

- 规格覆盖度
  - 驱动纯净化：由任务 2 覆盖
  - `worldCoord` 独立节点：由任务 3 覆盖
  - 接口兼容：由任务 3 的 launch 和服务迁移覆盖
- 占位符扫描
  - 没有 `TODO`、`待定`、`后续实现`
- 类型一致性
  - 新节点统一使用 `scepter_world_coord_processor`
  - 服务统一沿用 `tie_robot_perception/ConvertDepthToPointCloud`
