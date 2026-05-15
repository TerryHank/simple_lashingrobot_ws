# tie_robot_vision 视觉移植包实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 将当前工程完整视觉链路提取为可移植 ROS1 包，并实际适配到 `/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws`。

**架构：** 新增 `src/tie_robot_vision` 自包含包，内置相机驱动、世界坐标处理、pointAI、TF、视觉接口和兼容启动入口；目标旧工作区通过复制该包和启动 `legacy_20260403_vision.launch` 接入。旧控制层仍调用 `/pointAI/process_image`，服务类型兼容 `fast_image_solve/ProcessImage`。

**技术栈：** ROS1 Noetic、catkin、C++14、Python 3、OpenCV、PCL、cv_bridge、image_transport、tf2_ros、Scepter SDK。

---

### 任务 1：结构回归测试

**文件：**
- 创建：`test/test_tie_robot_vision_package.py`

- [ ] **步骤 1：编写失败的测试**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "tie_robot_vision"


def test_portable_vision_package_has_required_surface():
    required = [
        PKG / "package.xml",
        PKG / "CMakeLists.txt",
        PKG / "msg" / "PointCoords.msg",
        PKG / "msg" / "PointsArray.msg",
        PKG / "srv" / "ProcessImage.srv",
        PKG / "srv" / "SetGripperTfCalibration.srv",
        PKG / "srv" / "RobotHomeCalibration.srv",
        PKG / "scripts" / "pointai_node.py",
        PKG / "scripts" / "gripper_tf_broadcaster.py",
        PKG / "scripts" / "robot_tf_broadcaster.py",
        PKG / "launch" / "vision_stack.launch",
        PKG / "launch" / "legacy_20260403_vision.launch",
        PKG / "docs" / "移植交接说明.md",
    ]
    missing = [str(path.relative_to(ROOT)) for path in required if not path.exists()]
    assert not missing


def test_legacy_launch_documents_old_workspace_path_and_compat_service():
    launch_text = (PKG / "launch" / "legacy_20260403_vision.launch").read_text(encoding="utf-8")
    assert "/pointAI/process_image" in launch_text
    assert "/Moduan/process_image" in launch_text
    doc_text = (PKG / "docs" / "移植交接说明.md").read_text(encoding="utf-8")
    assert "/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws" in doc_text
    assert "20260403" in doc_text
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m pytest test/test_tie_robot_vision_package.py -q`
预期：FAIL，缺少 `src/tie_robot_vision`。

### 任务 2：创建自包含视觉包

**文件：**
- 创建：`src/tie_robot_vision/**`
- 来源：`src/tie_robot_perception/**`、`src/tie_robot_msgs/msg/PointCoords.msg`、`src/tie_robot_msgs/msg/PointsArray.msg`、`src/tie_robot_msgs/srv/ProcessImage.srv`、TF 相关 srv/msg。

- [ ] **步骤 1：复制视觉源码与 SDK**

复制 `tie_robot_perception` 中的 `cfg`、`config`、`data`、`dependencies`、`include`、`launch`、`scripts`、`setup.py`、`src/tie_robot_perception` 到 `tie_robot_vision` 对应结构，并把 Python 包目录改名为 `tie_robot_vision`。

- [ ] **步骤 2：内置视觉消息和服务**

在 `tie_robot_vision/msg` 添加 `PointCoords.msg`、`PointsArray.msg`、`linear_module_upload.msg`、`cabin_upload.msg`、`motion.msg`；在 `tie_robot_vision/srv` 添加 `ProcessImage.srv`、`PlaneDetection.srv`、`SingleMove.srv`、`linear_module_move.srv`、`SetGripperTfCalibration.srv`、`RobotHomeCalibration.srv`、`ConvertDepthToPointCloud.srv`。

- [ ] **步骤 3：替换包名引用**

将 `tie_robot_perception` 替换为 `tie_robot_vision`，将 `tie_robot_msgs` 替换为 `tie_robot_vision`，确保 C++ include、Python import、launch `pkg` 都指向新包。

- [ ] **步骤 4：提供兼容入口**

新增或改造 `legacy_20260403_vision.launch`，启动 `pointAINode`、`scepter_camera`、`scepter_world_coord_processor`、`robot_tf_broadcaster`、`gripper_tf_broadcaster`，并让 pointAI 同时提供 `/pointAI/process_image` 和 `/Moduan/process_image` 服务。

### 任务 3：中文交接文档和 zip

**文件：**
- 创建：`src/tie_robot_vision/docs/移植交接说明.md`
- 创建：`artifacts/tie_robot_vision_portable_20260515.zip`

- [ ] **步骤 1：写交接文档**

文档必须包含：原工程视觉链路、旧工作区路径、复制命令、构建命令、启动命令、旧服务兼容说明、话题/服务清单、故障排查。

- [ ] **步骤 2：生成 zip**

运行：`zip -r artifacts/tie_robot_vision_portable_20260515.zip src/tie_robot_vision -x '**/__pycache__/*'`

### 任务 4：旧工作区适配尝试

**文件：**
- 创建：`/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/tie_robot_vision/**`

- [ ] **步骤 1：复制包到旧工作区**

运行：`rsync -a --delete src/tie_robot_vision/ /home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/tie_robot_vision/`

- [ ] **步骤 2：构建旧工作区新包**

运行：`source /opt/ros/noetic/setup.bash && catkin_make --pkg tie_robot_vision`
预期：如果系统依赖齐全，`tie_robot_vision` 构建成功；如果旧工作区已有依赖缺失，记录具体错误到交接文档。

### 任务 5：验证和记忆

**文件：**
- 修改：`docs/agent_memory/current.md`（通过脚本刷新）

- [ ] **步骤 1：运行结构测试**

运行：`python3 -m pytest test/test_tie_robot_vision_package.py -q`
预期：PASS。

- [ ] **步骤 2：运行共享记忆更新**

运行：`python3 scripts/agent_memory.py add --title "视觉链路已提取为 tie_robot_vision 移植包" --summary "..."`
再运行：`python3 scripts/agent_memory.py refresh`。
