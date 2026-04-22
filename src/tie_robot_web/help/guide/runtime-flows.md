# 运行主链

## 数据链

```text
新前端
-> tie_robot_web topics/action/service bridge
-> tie_robot_process / tie_robot_control
-> tie_robot_hw
-> 真实硬件
```

```text
相机
-> tie_robot_hw 相机驱动
-> tie_robot_perception 世界点/视觉算法
-> tie_robot_process 规划与账本
-> tie_robot_control 末端执行
```

```text
新前端 3D Scene / Topic Layers
-> /tf + /tf_static
-> /coordinate_point
-> /cabin/pseudo_slam_markers
-> /Scepter/worldCoord/world_coord
-> /Scepter/worldCoord/raw_world_coord
-> 浏览器内三维场景
```

## 扫描建图

```text
/web/cabin/start_pseudo_slam_scan (Action)
-> tie_robot_web/action bridge
-> /cabin/start_pseudo_slam_scan_with_options
-> tie_robot_process::run_pseudo_slam_scan
-> tie_robot_perception::pointAI
-> pseudo_slam_points.json
-> pseudo_slam_bind_path.json
-> bind_execution_memory.json(重置)
```

## 执行层

```text
/web/cabin/start_global_work (Action)
-> tie_robot_web/action bridge
-> /cabin/set_execution_mode
-> /cabin/start_work_with_options
-> tie_robot_process::startGlobalWorkWithOptions
-> 优先走 pseudo_slam_bind_path.json
-> 索驱 area.cabin_pose
-> 线模 group.points[].x/y/z
-> bind_execution_memory.json(成功点回写)
```

## 直接执行账本测试

```text
/web/cabin/run_bind_path_direct_test (Action)
-> tie_robot_web/action bridge
-> /cabin/run_bind_path_direct_test
-> tie_robot_process::run_bind_path_direct_test
-> 只读 pseudo_slam_bind_path.json
-> 不走相机 live / 不清记忆 / 不写记忆
```
