# 设置卡片索驱遥控页实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 在 `设置` 卡片中新增一个索驱遥控页，支持按钮遥控、全局键盘遥控、当前位置实时显示、按钮长按连续步进，以及中间停止按钮调用真实的“停止当前索驱运动”服务。

**架构：** UI 层新增 `索驱遥控` 页签、当前位置展示和按钮激活态；应用层绑定全局键盘输入，并负责按钮长按 `250 ms` 连发节拍、停止条件、全局速度同步与状态反馈；`CabinRemoteController` 负责方向映射、当前位置读取和 `/cabin/single_move` 调用。当前位置优先来自 `Scene3DView` 里的 `map -> Scepter_depth_frame` TF 结果；中心停止按钮单独调用 `/cabin/motion/stop`。`索驱遥控` 页里的速度输入框通过 `/web/cabin/set_cabin_speed` 同步后端 `global_cabin_speed`，并成为遥控、固定点移动、固定扫描和执行层索驱移动的统一速度源。

**技术栈：** 原生 JavaScript、Three.js、ROSLIB、Python `unittest` 结构测试、Vite 构建。

---

### 任务 1：为索驱遥控页补失败测试

**文件：**
- 修改：`src/tie_robot_web/test/test_workspace_picker_web.py`
- 参考：`src/tie_robot_web/frontend/src/ui/UIController.js`
- 参考：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- 参考：`src/tie_robot_web/frontend/src/views/Scene3DView.js`

- [ ] **步骤 1：编写失败的结构测试**

```python
    def test_settings_panel_renders_cabin_remote_page(self):
        ui_controller = (
            FRONTEND_SRC_DIR / "ui" / "UIController.js"
        ).read_text(encoding="utf-8")

        self.assertIn('<option value="cabinRemote">索驱遥控</option>', ui_controller)
        self.assertIn('data-settings-page="cabinRemote"', ui_controller)
        self.assertIn('id="cabinKeyboardRemoteToggle"', ui_controller)
        self.assertIn('id="cabinRemoteStep"', ui_controller)
        self.assertIn('id="cabinRemoteSpeed"', ui_controller)
        self.assertIn('data-cabin-remote-direction="xNegative"', ui_controller)
        self.assertIn('data-cabin-remote-direction="zPositive"', ui_controller)

    def test_cabin_remote_keyboard_and_tf_flow_exist(self):
        app_logic = (
            FRONTEND_SRC_DIR / "app" / "TieRobotFrontApp.js"
        ).read_text(encoding="utf-8")
        scene_view = (
            FRONTEND_SRC_DIR / "views" / "Scene3DView.js"
        ).read_text(encoding="utf-8")
        cabin_remote_controller = (
            FRONTEND_SRC_DIR / "controllers" / "CabinRemoteController.js"
        ).read_text(encoding="utf-8")

        self.assertIn("handleCabinRemoteKeyDown(event)", app_logic)
        self.assertIn("document.addEventListener(\"keydown\", this.handleCabinRemoteKeyDown, true);", app_logic)
        self.assertIn("event.repeat", app_logic)
        self.assertIn("cabinKeyboardRemoteToggle", app_logic)
        self.assertIn("new CabinRemoteController({", app_logic)
        self.assertIn("getCurrentCabinPositionMm()", scene_view)
        self.assertIn("callCabinSingleMoveService(target)", cabin_remote_controller)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py`

预期：新增的 `索驱遥控` 相关断言失败，因为页面、控制器和键盘逻辑尚未实现。

- [ ] **步骤 3：确认失败原因只来自新功能缺失**

检查失败输出，确保不是路径错误、编码错误或已有功能回归。

### 任务 2：实现索驱遥控 UI 和样式

**文件：**
- 修改：`src/tie_robot_web/frontend/src/ui/UIController.js`
- 修改：`src/tie_robot_web/frontend/src/styles/app.css`

- [ ] **步骤 1：在设置页下新增索驱遥控页签与 DOM 结构**

```js
<option value="cabinRemote">索驱遥控</option>

<section class="settings-page" data-settings-page="cabinRemote" hidden>
  <div class="settings-grid">
    <div class="settings-section">
      <div class="section-title">索驱遥控</div>
      <label class="checkbox-field">
        <input id="cabinKeyboardRemoteToggle" type="checkbox" />
        <span>开启键盘遥控（全局生效）</span>
      </label>
      <div class="field-grid compact-grid">
        <div class="field">
          <label for="cabinRemoteStep">单次点击步距（mm）</label>
          <input id="cabinRemoteStep" type="number" min="1" step="1" value="50" />
        </div>
        <div class="field">
          <label for="cabinRemoteSpeed">移动速度</label>
          <input id="cabinRemoteSpeed" type="number" min="1" step="1" value="300" />
        </div>
      </div>
      <div id="cabinRemoteStatus" class="info-block mono">键位：W/S=Y，A/D=X，Q/E=Z。</div>
      <div class="cabin-remote-pad">
        <!-- 6 个 data-cabin-remote-direction 按钮 -->
      </div>
    </div>
  </div>
</section>
```

- [ ] **步骤 2：在 `UIController` 中绑定新 refs 和读取方法**

```js
this.refs.cabinKeyboardRemoteToggle = this.rootElement.querySelector("#cabinKeyboardRemoteToggle");
this.refs.cabinRemoteStep = this.rootElement.querySelector("#cabinRemoteStep");
this.refs.cabinRemoteSpeed = this.rootElement.querySelector("#cabinRemoteSpeed");
this.refs.cabinRemoteStatus = this.rootElement.querySelector("#cabinRemoteStatus");
this.refs.cabinRemoteButtons = [...this.rootElement.querySelectorAll("[data-cabin-remote-direction]")];
```

```js
getCabinRemoteSettings() {
  return {
    keyboardEnabled: Boolean(this.refs.cabinKeyboardRemoteToggle?.checked),
    step: Number.parseFloat(this.refs.cabinRemoteStep?.value || "50"),
    speed: Number.parseFloat(this.refs.cabinRemoteSpeed?.value || "300"),
  };
}
```

- [ ] **步骤 3：补充 UI 回调与状态更新接口**

```js
onCabinRemoteAction(callback) {
  this.refs.cabinRemoteButtons.forEach((button) => {
    button.addEventListener("click", () => callback(button.dataset.cabinRemoteDirection));
  });
}

setCabinRemoteStatus(message) {
  if (this.refs.cabinRemoteStatus) {
    this.refs.cabinRemoteStatus.textContent = message;
  }
}

setCabinRemoteButtonsEnabled(enabled) {
  this.refs.cabinRemoteButtons.forEach((button) => {
    button.disabled = !enabled;
  });
}
```

- [ ] **步骤 4：添加样式，保证按钮区与现有风格一致**

```css
.cabin-remote-pad {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 10px;
}

.cabin-remote-btn {
  min-height: 54px;
  border-radius: 16px;
}
```

- [ ] **步骤 5：运行测试确认 UI 结构断言通过**

运行：`python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py`

预期：原先与 UI 结构相关的新断言通过；键盘与控制器相关断言仍可能失败。

### 任务 3：实现当前位置读取与索驱遥控控制器

**文件：**
- 新增：`src/tie_robot_web/frontend/src/controllers/CabinRemoteController.js`
- 修改：`src/tie_robot_web/frontend/src/views/Scene3DView.js`

- [ ] **步骤 1：在 `Scene3DView` 中暴露当前索驱位置读取接口**

```js
getCurrentCabinPositionMm() {
  const scepterTransform = this.getWorldTransform(SCEPTER_FRAME);
  if (!scepterTransform) {
    return null;
  }

  return {
    x: scepterTransform.position.x * 1000.0,
    y: scepterTransform.position.y * 1000.0,
    z: scepterTransform.position.z * 1000.0,
  };
}
```

- [ ] **步骤 2：新增 `CabinRemoteController`，封装方向映射和单步移动**

```js
const CABIN_REMOTE_DIRECTION_DEFINITIONS = {
  xNegative: { axis: "x", delta: -1, label: "X-" },
  xPositive: { axis: "x", delta: 1, label: "X+" },
  yPositive: { axis: "y", delta: 1, label: "Y+" },
  yNegative: { axis: "y", delta: -1, label: "Y-" },
  zPositive: { axis: "z", delta: 1, label: "Z+" },
  zNegative: { axis: "z", delta: -1, label: "Z-" },
};
```

```js
async move(directionId, { step, speed }) {
  const definition = CABIN_REMOTE_DIRECTION_DEFINITIONS[directionId];
  const currentPosition = this.sceneView.getCurrentCabinPositionMm() || this.lastKnownCabinPositionMm;
  if (!definition || !currentPosition) {
    return { success: false, message: "暂未拿到索驱当前位置，无法执行步进遥控。" };
  }

  const target = { ...currentPosition, speed };
  target[definition.axis] += definition.delta * step;
  const result = await this.rosConnection.callCabinSingleMoveService(target);
  if (result.success) {
    this.lastKnownCabinPositionMm = { x: target.x, y: target.y, z: target.z };
  }
  return { ...result, target, label: definition.label };
}
```

- [ ] **步骤 3：运行测试确认控制器与 TF 读取断言通过**

运行：`python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py`

预期：`Scene3DView` 和新控制器的结构断言通过。

### 任务 4：在应用层接入页面按钮、全局键盘和状态反馈

**文件：**
- 修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`

- [ ] **步骤 1：引入并初始化 `CabinRemoteController`**

```js
import { CabinRemoteController } from "../controllers/CabinRemoteController.js";

this.handleCabinRemoteKeyDown = this.handleCabinRemoteKeyDown.bind(this);
this.cabinRemoteController = new CabinRemoteController({
  rosConnection: this.rosConnectionController,
  sceneView: this.sceneView,
});
```

- [ ] **步骤 2：绑定页面按钮与全局键盘事件**

```js
this.ui.onCabinRemoteAction((directionId) => {
  this.handleCabinRemoteDirection(directionId, "button");
});

document.removeEventListener("keydown", this.handleCabinRemoteKeyDown, true);
document.addEventListener("keydown", this.handleCabinRemoteKeyDown, true);
```

```js
handleCabinRemoteKeyDown(event) {
  if (event.repeat || event.ctrlKey || event.metaKey || event.altKey) {
    return;
  }
  if (!this.ui.getCabinRemoteSettings().keyboardEnabled) {
    return;
  }
  if (this.shouldIgnoreCabinRemoteKeyboard(event.target)) {
    return;
  }
  const directionId = this.resolveCabinRemoteDirectionFromKey(event.key);
  if (!directionId) {
    return;
  }
  event.preventDefault();
  this.handleCabinRemoteDirection(directionId, "keyboard");
}
```

- [ ] **步骤 3：统一处理单步移动结果，并把固定索驱位同步给遥控控制器**

```js
async handleCabinRemoteDirection(directionId, source) {
  const settings = this.ui.getCabinRemoteSettings();
  const result = await this.cabinRemoteController.move(directionId, settings);
  this.ui.setCabinRemoteStatus(result.message);
  this.ui.setResultMessage(result.message);
  this.addLog(result.message, result.success ? "success" : "warn");
}
```

```js
if (result.success) {
  this.cabinRemoteController.setLastKnownCabinPosition(payload);
}
```

- [ ] **步骤 4：在动作可用性刷新里控制遥控按钮禁用态**

```js
this.ui.setCabinRemoteButtonsEnabled(
  ready && Boolean(resources?.cabinSingleMoveService),
);
```

- [ ] **步骤 5：运行测试确认应用层结构断言全部通过**

运行：`python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py`

预期：全部结构测试通过。

### 任务 4A：补齐实时坐标显示与按钮长按连发

**文件：**
- 修改：`src/tie_robot_web/frontend/src/ui/UIController.js`
- 修改：`src/tie_robot_web/frontend/src/styles/app.css`
- 修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
- 修改：`src/tie_robot_web/test/test_workspace_picker_web.py`

- [ ] **步骤 1：先补失败的结构测试**

```python
    self.assertIn('id="cabinRemoteCurrentPosition"', ui_controller)
    self.assertIn("setCabinRemoteCurrentPosition(position)", ui_controller)
    self.assertIn("setCabinRemoteButtonActive(directionId)", ui_controller)
    self.assertIn("clearCabinRemoteButtonActive()", ui_controller)
    self.assertIn("refreshCabinRemoteCurrentPosition()", app_logic)
    self.assertIn("startCabinRemoteRepeat(directionId)", app_logic)
    self.assertIn("stopCabinRemoteRepeat()", app_logic)
    self.assertIn("scheduleCabinRemoteRepeatTick()", app_logic)
```

- [ ] **步骤 2：运行测试验证失败**

运行：`python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py`

预期：`索驱遥控` 新增的实时坐标与长按连发断言失败，且失败原因仅来自新需求缺失。

- [ ] **步骤 3：在 `UIController` 增加当前位置区和按钮按压事件通道**

```js
<div id="cabinRemoteCurrentPosition" class="info-block cabin-remote-position-grid mono"></div>

onCabinRemoteAction(callback) {
  this.refs.cabinRemoteButtons.forEach((button) => {
    button.addEventListener("pointerdown", () => callback(button.dataset.cabinRemoteDirection, { type: "pressstart" }));
    button.addEventListener("pointerup", () => callback(button.dataset.cabinRemoteDirection, { type: "pressend" }));
    button.addEventListener("pointerleave", () => callback(button.dataset.cabinRemoteDirection, { type: "pressend" }));
  });
}
```

- [ ] **步骤 4：在 `TieRobotFrontApp` 增加实时坐标刷新和长按连发调度**

```js
refreshCabinRemoteCurrentPosition() {
  const currentCabinPosition = this.cabinRemoteController.getCurrentCabinPositionMm();
  this.ui.setCabinRemoteCurrentPosition(currentCabinPosition);
}

async startCabinRemoteRepeat(directionId) {
  this.stopCabinRemoteRepeat();
  this.cabinRemoteRepeatDirectionId = directionId;
  this.ui.setCabinRemoteButtonActive(directionId);
  const result = await this.handleCabinRemoteDirection(directionId, "button", { repeating: true });
  if (result?.success && this.cabinRemoteRepeatDirectionId === directionId) {
    this.scheduleCabinRemoteRepeatTick();
  }
}

scheduleCabinRemoteRepeatTick() {
  this.cabinRemoteRepeatTimerId = window.setTimeout(async () => {
    if (this.cabinRemoteMoveInFlight) {
      this.scheduleCabinRemoteRepeatTick();
      return;
    }
    // 下一次固定节拍连发
  }, 250);
}
```

- [ ] **步骤 5：运行测试确认增强断言通过**

运行：`python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py`

预期：实时坐标、按钮激活态、全局停止条件和连发节拍相关断言全部通过。

### 任务 5：构建验证并同步前端产物

**文件：**
- 修改：`src/tie_robot_web/web/index.html`
- 生成：`src/tie_robot_web/web/assets/app/index-*.js`
- 生成：`src/tie_robot_web/web/assets/app/index-*.css`

- [ ] **步骤 1：运行完整测试**

运行：`python3 -m unittest src/tie_robot_web/test/test_workspace_picker_web.py`

预期：`OK`

- [ ] **步骤 2：运行前端构建**

运行：`npm --prefix src/tie_robot_web/frontend run build`

预期：构建成功，输出新的 `web/assets/app/index-*.js` 和 `index-*.css`

- [ ] **步骤 3：人工核对需求覆盖**

核对以下行为是否都已被代码覆盖：

```text
1. 设置卡片出现“索驱遥控”页
2. 勾选后全局键盘 W/A/S/D/Q/E 生效
3. 输入框聚焦时不触发键盘遥控
4. 按钮与键盘共用步距和速度
5. 最终统一走 /cabin/single_move
6. 当前位置来自 TF，而不是写死坐标
```
