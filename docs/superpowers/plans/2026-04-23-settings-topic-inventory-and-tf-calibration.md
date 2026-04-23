# 设置卡片话题总览与相机-TCP外参页实现计划

1. 扩展 `设置` 页签
   - 新增 `话题总览`
   - 新增 `相机-TCP外参`
   - 默认切到 `话题总览`

2. 扩展 ROS 连接层
   - 周期调用 `ros.getTopicsAndRawTypes()`
   - 发布当前 topic inventory 给前端
   - 增加 `/web/pointAI/set_offset` 发布器

3. 扩展 3D/TF 读取层
   - 提供当前 `Scepter_depth_frame -> gripper_frame` 的平移读取接口
   - 转换为 UI 使用的 `translation_mm`

4. 扩展 UI
   - 渲染话题列表
   - 渲染外参输入框和应用按钮
   - 绑定应用动作

5. 回归验证
   - 结构测试
   - 前端语法检查
   - 前端构建
