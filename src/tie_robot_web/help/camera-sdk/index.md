# 相机 SDK 帮助

这里把工作区根目录的 `vzense_wiki/` 融入到了新前端帮助站里，作为相机 SDK 的原厂参考区。

## 当前工程里的相机链

- 相机底层驱动：`src/tie_robot_perception/src/camera/`
- 世界点与算法处理：`src/tie_robot_perception/src/perception/`
- 启动入口：`src/tie_robot_perception/launch/scepter_camera.launch`

## 原厂文档镜像

- [Vzense 原厂文档总入口](./vendor-vzense/README)
- [中文文档入口](./vendor-vzense/zh-cn/README)
- [English Entry](./vendor-vzense/en/README)

## 常用入口

- [中文 Quickstart](./vendor-vzense/zh-cn/Quickstart/Quickstart)
- [中文 ScepterSDK 简介](./vendor-vzense/zh-cn/ScepterSDK/Overview)
- [中文 BaseSDK](./vendor-vzense/zh-cn/ScepterSDK/BaseSDK)
- [中文 ROS 插件说明](./vendor-vzense/zh-cn/ScepterSDK/3rd-Party-Plugin/ROS)
- [English BaseSDK](./vendor-vzense/en/ScepterSDK/BaseSDK)

## 同步方式

当工作区根目录下的 `vzense_wiki/` 更新后，可以重新执行：

```bash
python3 src/tie_robot_web/help/scripts/sync_vzense_wiki.py
cd src/tie_robot_web/help && npm run build
```
