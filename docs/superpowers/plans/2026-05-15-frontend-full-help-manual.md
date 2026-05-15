# 浏览器前端全功能图文手册实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 在帮助站中新增一份覆盖整个浏览器前端的图文操作手册，包含系统启停、控制开关、账本生成、全局绑扎、视觉调试、图层、遥控、日志、终端、国标接入和相机 SDK 调试等入口。

**架构：** 只修改帮助站源码与帮助站图片资产，不改前端运行逻辑。新增 `operator-manual.md` 作为全量手册页，并从首页、导航栏、侧边栏挂入口；图片资源放在 `help/public/images/operator-manual/`，构建后进入 `web/help`。

**技术栈：** VitePress Markdown、静态 PNG/SVG 图片、现有 `npm run build` 帮助站构建链。

---

### 任务 1：新增帮助站全量手册入口

**文件：**
- 修改：`src/tie_robot_web/help/.vitepress/config.mjs`
- 修改：`src/tie_robot_web/help/index.md`
- 创建：`src/tie_robot_web/help/guide/operator-manual.md`

- [ ] **步骤 1：新增空手册页并挂导航**

新增 `guide/operator-manual.md`，标题为「浏览器前端全功能图文手册」。在 VitePress 顶栏和「工程说明」侧边栏加入「前端全功能手册」，在首页「常用入口」加入同名链接。

- [ ] **步骤 2：运行帮助站构建验证路由存在**

运行：`cd src/tie_robot_web/help && npm run build`

预期：退出码为 0，`src/tie_robot_web/web/help/guide/operator-manual.html` 存在。

### 任务 2：补齐图文素材

**文件：**
- 创建：`src/tie_robot_web/help/public/images/operator-manual/frontend-shell.svg`
- 创建：`src/tie_robot_web/help/public/images/operator-manual/frontend-shell.png`（如可生成）

- [ ] **步骤 1：创建一张前端区域总览图**

基于真实前端结构绘制总览图，标注顶部工具条、状态胶囊、控制面板、图像面板、设置页、3D 背景、日志/终端、底部快速控制区。

- [ ] **步骤 2：确认图片被帮助站引用**

在手册页引用 `/images/operator-manual/frontend-shell.svg`，若生成 PNG，则作为备用说明。

### 任务 3：撰写全功能手册内容

**文件：**
- 修改：`src/tie_robot_web/help/guide/operator-manual.md`

- [ ] **步骤 1：写系统入口和安全状态**

覆盖浏览器入口、ROS 连接、电压、索驱/末端/视觉状态胶囊、演示模式、主题与帮助入口。

- [ ] **步骤 2：写控制面板和底部快速开关**

覆盖扫描区、执行层、区域切换、全局遥控速度，以及暂停/恢复、开启/关闭绑扎、跳绑黑白棋、分类、灯光等快速开关。

- [ ] **步骤 3：写账本生成到全局绑扎主流程**

覆盖识别位姿、工作区选点、确认工作区域、触发扫描视觉、账本产物、执行全局绑扎、记忆续跑和人工切区。

- [ ] **步骤 4：写设置页全功能**

覆盖话题总览、节点日志、视觉调试、相机 SDK、国标接入、网络配置、工作区选点、显示与视角、Home 点位、索驱遥控、TCP 线模遥控。

- [ ] **步骤 5：写图像/3D/日志/终端和常见问题**

覆盖图像话题、覆盖层、3D 图层、点云源、悬停坐标、日志过滤、终端会话、常见失败提示。

### 任务 4：构建与验证

**文件：**
- 输出：`src/tie_robot_web/web/help/**`

- [ ] **步骤 1：构建前端（如源码变更影响静态页）**

本计划不改前端源码；若实际执行中改了 `src/tie_robot_web/frontend`，运行：`cd src/tie_robot_web/frontend && npm run build`。

- [ ] **步骤 2：构建帮助站**

运行：`cd src/tie_robot_web/help && npm run build`

预期：退出码为 0，新增页面和图片可被构建。

- [ ] **步骤 3：内容覆盖检查**

运行：

```bash
rg -n "开启绑扎|灯光|跳绑|分类|执行全局|记忆续跑|索驱遥控|TCP 线模|国标|相机底层 SDK|演示模式" src/tie_robot_web/help/guide/operator-manual.md
```

预期：每个关键词至少命中 1 次。
