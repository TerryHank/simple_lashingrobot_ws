# 执行层视觉算法源选择实现计划

> **面向 AI 代理的工作者：** 必需子技能：使用 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 逐任务实现此计划。步骤使用复选框（`- [ ]`）语法来跟踪进度。

**目标：** 在前端“视觉调试”里新增“执行层视觉”二选一，让执行层视觉可在现有 Hough 方案和扫描层 Surface-DP 同款方案之间切换。

**架构：** 前端保存并热发布一个 `std_msgs/String` 运行态参数 `/web/pointAI/set_execution_refine_algorithm`，默认值为 `hough`，可选 `surface_dp`。后端保持 `/pointAI/process_image` 和 `MODE_EXECUTION_REFINE=4` 不变，只在 pointAI 内部根据该参数选择 `run_execution_refine_hough_pipeline()` 或新增的 `run_execution_refine_surface_dp_pipeline()`；单点绑扎、单点视觉测试、执行全局绑扎和记忆续跑都会自动复用同一选择。

**技术栈：** ROS1 Noetic、Python pointAI 节点、`tie_robot_msgs/ProcessImage`、`std_msgs/String`、Vite 原生前端、Node ESM 测试、现有 Surface-DP 与 Hough 算法模块。

---

## 文件结构

- 修改：`src/tie_robot_web/frontend/src/config/visualRecognitionMode.js`
  - 新增执行层视觉算法枚举、默认值和 normalize 函数。
- 修改：`src/tie_robot_web/frontend/src/config/topicRegistry.js`
  - 新增 `/web/pointAI/set_execution_refine_algorithm` 话题，并登记到前端 topic registry。
  - 将执行底图 label/usage 从固定 Hough 改成兼容 Hough / Surface-DP。
- 修改：`src/tie_robot_web/frontend/src/utils/storage.js`
  - 持久化 `executionRefineAlgorithm`，默认 `hough`，旧 localStorage 自动回退默认。
- 修改：`src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
  - 建立、advertise 并发布执行层视觉算法话题。
- 修改：`src/tie_robot_web/frontend/src/ui/UIController.js`
  - 在“视觉调试”卡片新增 `执行层视觉` 下拉框或 segmented 控件，纳入 `get/set/onVisualDebugSettingsChange`。
- 修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`
  - `applyVisualDebugRuntimeSettings()` 同步发布新参数，并在已有视觉任务触发前一起同步。
- 修改：`src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs`
  - 锁定前端 topic、UI、存储和发布行为。
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
  - 初始化 `execution_refine_algorithm`，默认 `hough`。
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
  - 新增 normalize 和 `/web/pointAI/set_execution_refine_algorithm` callback。
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
  - 订阅新话题。
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
  - 绑定 runtime callback 和新增执行 Surface-DP 方法。
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`
  - 在 `MODE_EXECUTION_REFINE` 分支调用算法分发 helper，并按算法名调整等待/超时日志。
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
  - 新增 `run_execution_refine_surface_dp_pipeline()`，复用扫描 Surface-DP 后按 TCP 执行盒 + 全局工作区过滤，并按 TCP 蛇形输出。
- 修改：`src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
  - 加 Surface-DP 执行微调分支的源码级/轻量行为测试。
- 修改：`src/tie_robot_perception/test/test_bind_point_classification.py`
  - 锁定 Surface-DP 执行微调输出仍进入 `phase="execution_refine"` 分类并过滤已绑扎点。
- 修改：`src/tie_robot_control/test/test_single_point_bind_chain.py`
  - 锁定控制层仍只请求 `MODE_EXECUTION_REFINE=4`，不直接理解算法源，避免把选择扩散到控制层。
- 修改：`src/tie_robot_web/web`
  - 前端构建产物，执行 `npm run build` 后同步更新。
- 可能修改：`CHANGELOG.md`、`docs/agent_memory/current.md`
  - 完成实现和验证后记录跨会话工程口径。

---

## 关键行为约定

1. UI 文案：下拉 label 使用 `执行层视觉`；选项为 `Hough微调` 和 `扫描同款Surface-DP`。
2. 存储字段：`executionRefineAlgorithm`，默认 `hough`。
3. ROS 话题：`/web/pointAI/set_execution_refine_algorithm`，消息类型 `std_msgs/String`，可选值 `hough`、`surface_dp`。
4. 默认兼容：未发布、旧缓存、非法值全部回退 `hough`，保证现场现有执行行为不变。
5. 服务入口不变：控制层、流程层和前端动作仍调用 `/pointAI/process_image` 且 `request_mode=4`。
6. Hough 分支不改算法语义。
7. Surface-DP 分支必须复用扫描层同款 `run_manual_workspace_surface_dp_pipeline()` 或同一底层 Surface-DP 构建逻辑，不能另写一套“像 Surface-DP 的”临时代码。
8. Surface-DP 执行分支必须在 Surface-DP 输出后继续套执行层门控：有效相机坐标、TCP 执行盒、手动确认全局工作区，最后按 TCP 蛇形顺序作为当前区域一组输出。
9. Surface-DP 执行分支需要发布 `/perception/lashing/execution_refine_base_image`，让前端执行底图卡仍能看到当前执行层视觉诊断；扫描底图话题可继续由 Surface-DP helper 发布。
10. 绑扎点分类开关开启时，Surface-DP 执行分支和 Hough 分支一样进入 `classify_execution_refine_points()`，已绑扎点被过滤。

---

### 任务 1：前端配置、topic 和持久化红灯测试

**文件：**
- 修改：`src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs`
- 预期后续修改：`src/tie_robot_web/frontend/src/config/visualRecognitionMode.js`
- 预期后续修改：`src/tie_robot_web/frontend/src/config/topicRegistry.js`
- 预期后续修改：`src/tie_robot_web/frontend/src/utils/storage.js`

- [ ] **步骤 1：编写失败的测试**

在 `visualDebugSettings.test.mjs` 的 imports 中加入新符号：

```js
import {
  DEFAULT_EXECUTION_REFINE_ALGORITHM,
  DEFAULT_SCAN_RESPONSE_SOURCE,
  EXECUTION_REFINE_ALGORITHM_OPTIONS,
  GLOBAL_EXECUTION_MODES,
  SCAN_RESPONSE_SOURCE_OPTIONS,
} from "../src/config/visualRecognitionMode.js";
```

在现有 topic 断言附近追加：

```js
assert.equal(
  TOPICS.algorithm.setExecutionRefineAlgorithm,
  "/web/pointAI/set_execution_refine_algorithm",
);
assert.equal(DEFAULT_EXECUTION_REFINE_ALGORITHM, "hough");
assert.deepEqual(EXECUTION_REFINE_ALGORITHM_OPTIONS.map((option) => option.label), [
  "Hough微调",
  "扫描同款Surface-DP",
]);
```

在 `saveVisualDebugSettings(...)` / `loadVisualDebugSettings(...)` 相关断言附近追加最小存储行为：

```js
localStorage.clear();
assert.equal(loadVisualDebugSettings().executionRefineAlgorithm, "hough");
saveVisualDebugSettings({ executionRefineAlgorithm: "surface_dp" });
assert.equal(loadVisualDebugSettings().executionRefineAlgorithm, "surface_dp");
saveVisualDebugSettings({ executionRefineAlgorithm: "bad-value" });
assert.equal(loadVisualDebugSettings().executionRefineAlgorithm, "hough");
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
```

预期：FAIL，失败原因包含 `does not provide an export named 'DEFAULT_EXECUTION_REFINE_ALGORITHM'` 或 `setExecutionRefineAlgorithm` 为 `undefined`。

- [ ] **步骤 3：实现最少配置和持久化代码**

修改 `src/tie_robot_web/frontend/src/config/visualRecognitionMode.js`，在分类方法常量后加入：

```js
export const EXECUTION_REFINE_ALGORITHM_OPTIONS = Object.freeze([
  { id: "hough", label: "Hough微调" },
  { id: "surface_dp", label: "扫描同款Surface-DP" },
]);

export const DEFAULT_EXECUTION_REFINE_ALGORITHM = "hough";

export function normalizeExecutionRefineAlgorithm(
  value,
  fallback = DEFAULT_EXECUTION_REFINE_ALGORITHM,
) {
  return EXECUTION_REFINE_ALGORITHM_OPTIONS.some((option) => option.id === value)
    ? value
    : fallback;
}
```

修改 `src/tie_robot_web/frontend/src/config/topicRegistry.js`：

```js
setExecutionRefineAlgorithm: "/web/pointAI/set_execution_refine_algorithm",
```

放在 `algorithm` topic 组内 `setExecutionRefineTcpRoi` 附近。

在 `FRONTEND_DIRECT_TOPIC_REGISTRY` 中加入：

```js
{
  key: "algorithm.setExecutionRefineAlgorithm",
  name: TOPICS.algorithm.setExecutionRefineAlgorithm,
  label: "执行层视觉算法选择",
  messageType: MESSAGE_TYPES.string,
  sourceLayer: "frontend",
  sourceLabel: "新前端",
  ownerNode: "pointAINode",
  direction: "publish",
  usage: "视觉调试页选择执行层视觉使用 Hough 微调或扫描层 Surface-DP 同款方案",
},
```

修改 `src/tie_robot_web/frontend/src/utils/storage.js` import：

```js
import {
  DEFAULT_BIND_CLASSIFICATION_METHOD,
  DEFAULT_EXECUTION_REFINE_ALGORITHM,
  DEFAULT_GLOBAL_EXECUTION_MODE,
  FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE,
  GLOBAL_EXECUTION_MODE_OPTIONS,
  DEFAULT_SCAN_RESPONSE_SOURCE,
  normalizeBindClassificationMethod,
  normalizeExecutionRefineAlgorithm,
  normalizeScanResponseSource,
} from "../config/visualRecognitionMode.js";
```

在 `loadVisualDebugSettings()` 的 `defaults` 中加入：

```js
executionRefineAlgorithm: DEFAULT_EXECUTION_REFINE_ALGORITHM,
```

在 parsed return 中加入：

```js
executionRefineAlgorithm: normalizeExecutionRefineAlgorithm(
  parsed?.executionRefineAlgorithm,
  defaults.executionRefineAlgorithm,
),
```

在 `saveVisualDebugSettings()` payload 中加入：

```js
executionRefineAlgorithm: normalizeExecutionRefineAlgorithm(value?.executionRefineAlgorithm),
```

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
```

预期：PASS。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs \
  src/tie_robot_web/frontend/src/config/visualRecognitionMode.js \
  src/tie_robot_web/frontend/src/config/topicRegistry.js \
  src/tie_robot_web/frontend/src/utils/storage.js
git commit -m "test(web): lock execution refine algorithm setting"
```

---

### 任务 2：前端 UI 和发布链路

**文件：**
- 修改：`src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs`
- 修改：`src/tie_robot_web/frontend/src/ui/UIController.js`
- 修改：`src/tie_robot_web/frontend/src/controllers/RosConnectionController.js`
- 修改：`src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js`

- [ ] **步骤 1：编写失败的测试**

在 `visualDebugSettings.test.mjs` 的 UI 静态断言附近追加：

```js
assert.match(uiControllerText, /id="visualDebugExecutionRefineAlgorithm"/);
assert.match(uiControllerText, /执行层视觉/);
assert.match(uiControllerText, /EXECUTION_REFINE_ALGORITHM_OPTIONS\.map/);
assert.match(visualRecognitionModeText, /Hough微调/);
assert.match(visualRecognitionModeText, /扫描同款Surface-DP/);
```

在 `visualDebugRuntimeSettingsBlock` 断言附近追加：

```js
assert.match(
  visualDebugRuntimeSettingsBlock,
  /publishExecutionRefineAlgorithm\(nextSettings\.executionRefineAlgorithm\)/,
);
```

在 resources 发布行为断言附近追加一个运行时测试：

```js
const publishAlgorithmResult = controller.publishExecutionRefineAlgorithm("surface_dp");
assert.equal(publishAlgorithmResult.success, true);
assert.equal(publishAlgorithmResult.algorithm, "surface_dp");
assert.equal(
  controller.resources.executionRefineAlgorithmPublisher.published.at(-1).data,
  "surface_dp",
);
const fallbackAlgorithmResult = controller.publishExecutionRefineAlgorithm("bad-value");
assert.equal(fallbackAlgorithmResult.algorithm, "hough");
assert.equal(
  controller.resources.executionRefineAlgorithmPublisher.published.at(-1).data,
  "hough",
);
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
```

预期：FAIL，失败原因包含 `publishExecutionRefineAlgorithm is not a function` 或找不到 `visualDebugExecutionRefineAlgorithm`。

- [ ] **步骤 3：实现 UI 控件**

修改 `src/tie_robot_web/frontend/src/ui/UIController.js` import：

```js
import {
  DEFAULT_GLOBAL_EXECUTION_MODE,
  BIND_CLASSIFICATION_METHODS,
  DEFAULT_BIND_CLASSIFICATION_METHOD,
  DEFAULT_EXECUTION_REFINE_ALGORITHM,
  EXECUTION_REFINE_ALGORITHM_OPTIONS,
  FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE,
  GLOBAL_EXECUTION_MODES,
  DEFAULT_SCAN_RESPONSE_SOURCE,
  SCAN_RESPONSE_SOURCE_OPTIONS,
  normalizeBindClassificationMethod,
  normalizeExecutionRefineAlgorithm,
  normalizeScanResponseSource,
} from "../config/visualRecognitionMode.js";
```

在视觉调试卡片 `field-grid compact-grid` 内，放在 `扫描底图` 和 `分类方法` 之间：

```html
<div class="field">
  <label for="visualDebugExecutionRefineAlgorithm">执行层视觉</label>
  <select id="visualDebugExecutionRefineAlgorithm">
    ${EXECUTION_REFINE_ALGORITHM_OPTIONS.map((option) => `
      <option value="${option.id}" ${option.id === DEFAULT_EXECUTION_REFINE_ALGORITHM ? "selected" : ""}>${option.label}</option>
    `).join("")}
  </select>
</div>
```

在 `bindRefs()` 中加入：

```js
this.refs.visualDebugExecutionRefineAlgorithm = this.rootElement.querySelector("#visualDebugExecutionRefineAlgorithm");
```

在 `getVisualDebugSettings()` return 中加入：

```js
executionRefineAlgorithm: normalizeExecutionRefineAlgorithm(
  this.refs.visualDebugExecutionRefineAlgorithm?.value,
),
```

在 `setVisualDebugSettings(settings)` 中加入：

```js
if (this.refs.visualDebugExecutionRefineAlgorithm) {
  this.refs.visualDebugExecutionRefineAlgorithm.value = normalizeExecutionRefineAlgorithm(
    settings?.executionRefineAlgorithm,
  );
}
```

在 `onVisualDebugSettingsChange()` 监听数组中加入：

```js
this.refs.visualDebugExecutionRefineAlgorithm,
```

- [ ] **步骤 4：实现 ROS 发布链路**

修改 `src/tie_robot_web/frontend/src/controllers/RosConnectionController.js` import：

```js
import {
  FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE,
  normalizeBindClassificationMethod,
  normalizeExecutionRefineAlgorithm,
  normalizeScanResponseSource,
} from "../config/visualRecognitionMode.js";
```

在连接成功 advertise 区块加入：

```js
this.resources.executionRefineAlgorithmPublisher.advertise();
```

在 `buildResources()` 的 `scanResponseSourcePublisher` 附近加入：

```js
executionRefineAlgorithmPublisher: new ROSLIB.Topic({
  ros,
  name: TOPICS.algorithm.setExecutionRefineAlgorithm,
  messageType: MESSAGE_TYPES.string,
}),
```

在 `publishScanResponseSource()` 后加入：

```js
publishExecutionRefineAlgorithm(algorithm) {
  if (!this.ros?.isConnected || !this.resources?.executionRefineAlgorithmPublisher) {
    return { success: false, message: "ROS 未连接，无法设置执行层视觉算法。" };
  }
  const normalizedAlgorithm = normalizeExecutionRefineAlgorithm(algorithm);
  this.resources.executionRefineAlgorithmPublisher.publish(new ROSLIB.Message({ data: normalizedAlgorithm }));
  return {
    success: true,
    algorithm: normalizedAlgorithm,
    message: `执行层视觉已切换为 ${normalizedAlgorithm}。`,
  };
}
```

修改 `src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js` 的 `applyVisualDebugRuntimeSettings()`，在发布扫描底图后加入：

```js
const executionRefineAlgorithmResult = this.rosConnectionController.publishExecutionRefineAlgorithm(
  nextSettings.executionRefineAlgorithm,
);
```

并在已有失败日志块附近加入：

```js
if (!executionRefineAlgorithmResult?.success && !suppressLog) {
  const message = executionRefineAlgorithmResult?.message || "执行层视觉算法设置同步失败。";
  this.addLog(message, "warn");
  this.addVisualDebugLog(message, "warn");
}
```

- [ ] **步骤 5：运行测试验证通过**

运行：

```bash
node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs \
  src/tie_robot_web/frontend/src/ui/UIController.js \
  src/tie_robot_web/frontend/src/controllers/RosConnectionController.js \
  src/tie_robot_web/frontend/src/app/TieRobotFrontApp.js
git commit -m "feat(web): add execution vision algorithm selector"
```

---

### 任务 3：pointAI 运行态参数和算法分发

**文件：**
- 修改：`src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`

- [ ] **步骤 1：编写失败的测试**

在 `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py` 中新增测试方法：

```python
def test_execution_refine_algorithm_runtime_setting_is_subscribed_and_dispatches(self):
    state_source = _read_source(PERCEPTION_SRC / "tie_robot_perception" / "pointai" / "state.py")
    runtime_source = _read_source(PERCEPTION_SRC / "tie_robot_perception" / "pointai" / "runtime_config.py")
    ros_source = _read_source(PERCEPTION_SRC / "tie_robot_perception" / "pointai" / "ros_interfaces.py")
    processor_source = _read_source(PERCEPTION_SRC / "tie_robot_perception" / "pointai" / "processor.py")
    service_source = _read_source(PERCEPTION_SRC / "tie_robot_perception" / "pointai" / "process_image_service.py")

    self.assertIn("execution_refine_algorithm", state_source)
    self.assertIn("~execution_refine_algorithm", state_source)
    self.assertIn("set_execution_refine_algorithm_callback", runtime_source)
    self.assertIn("/web/pointAI/set_execution_refine_algorithm", ros_source)
    self.assertIn("cls.set_execution_refine_algorithm_callback", processor_source)
    self.assertIn("run_execution_refine_visual_pipeline", service_source)
    self.assertIn("run_execution_refine_surface_dp_pipeline", service_source)
    self.assertIn("run_execution_refine_hough_pipeline", service_source)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m pytest \
  src/tie_robot_perception/test/test_scan_surface_dp_runtime.py::ScanSurfaceDpRuntimeTest::test_execution_refine_algorithm_runtime_setting_is_subscribed_and_dispatches -q
```

预期：FAIL，失败原因包含 `execution_refine_algorithm` 或 `run_execution_refine_visual_pipeline` 不存在。

- [ ] **步骤 3：实现运行态参数**

修改 `src/tie_robot_perception/src/tie_robot_perception/pointai/state.py`，在 `scan_response_source` 后加入：

```python
self.execution_refine_algorithm = str(
    rospy.get_param("~execution_refine_algorithm", "hough") or "hough"
)
if self.execution_refine_algorithm not in {"hough", "surface_dp"}:
    self.execution_refine_algorithm = "hough"
```

修改 `src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py`，在 `set_scan_response_source_callback()` 后加入：

```python
def normalize_execution_refine_algorithm(value):
    requested = str(value or "").strip()
    if requested in {"hough", "surface_dp"}:
        return requested
    return "hough"


def set_execution_refine_algorithm_callback(self, msg):
    requested_algorithm = normalize_execution_refine_algorithm(getattr(msg, "data", "hough"))
    self.execution_refine_algorithm = requested_algorithm
    rospy.set_param("~execution_refine_algorithm", str(self.execution_refine_algorithm))
    rospy.loginfo("pointAI: 执行层视觉算法已切换为: %s", self.execution_refine_algorithm)
```

修改 `src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py`，在 `set_scan_response_source` 订阅附近加入：

```python
rospy.Subscriber('/web/pointAI/set_execution_refine_algorithm', String, self.set_execution_refine_algorithm_callback)
```

修改 `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`，在 runtime_config 绑定附近加入：

```python
cls.set_execution_refine_algorithm_callback = runtime_config.set_execution_refine_algorithm_callback
```

- [ ] **步骤 4：实现算法分发 helper**

修改 `src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`，在 `wait_for_stable_point_coords()` 前加入：

```python
def get_execution_refine_algorithm(self):
    algorithm = str(getattr(self, "execution_refine_algorithm", "hough") or "hough").strip()
    return algorithm if algorithm in {"hough", "surface_dp"} else "hough"


def get_execution_refine_algorithm_label(self):
    algorithm = self.get_execution_refine_algorithm()
    if algorithm == "surface_dp":
        return "扫描同款Surface-DP"
    return "平面分割+Hough"


def run_execution_refine_visual_pipeline(self, publish=True):
    algorithm = self.get_execution_refine_algorithm()
    if algorithm == "surface_dp":
        return self.run_execution_refine_surface_dp_pipeline(publish=publish)
    return self.run_execution_refine_hough_pipeline(publish=publish)
```

在同文件 `wait_for_stable_point_coords()` 的 `PROCESS_IMAGE_MODE_EXECUTION_REFINE` 分支中，把：

```python
execution_refine_result = self.run_execution_refine_hough_pipeline(publish=True)
```

替换为：

```python
execution_refine_result = self.run_execution_refine_visual_pipeline(publish=True)
```

把等待日志中的固定文案：

```python
"pointAI等待执行微调平面分割+Hough有效点: %s（无点等待%.1fs/%.1fs）"
```

替换为：

```python
"pointAI等待执行微调%s有效点: %s（无点等待%.1fs/%.1fs）"
```

并在参数列表中把算法 label 放在视觉消息前：

```python
self.get_execution_refine_algorithm_label(),
execution_refine_result.get("message", "未知错误"),
```

把超时文案：

```python
message = "pointAI视觉服务等待执行微调平面分割+Hough超时"
```

替换为：

```python
message = f"pointAI视觉服务等待执行微调{self.get_execution_refine_algorithm_label()}超时"
```

在 `processor.py` 中绑定新增 helper：

```python
cls.get_execution_refine_algorithm = process_image_service.get_execution_refine_algorithm
cls.get_execution_refine_algorithm_label = process_image_service.get_execution_refine_algorithm_label
cls.run_execution_refine_visual_pipeline = process_image_service.run_execution_refine_visual_pipeline
```

- [ ] **步骤 5：运行测试验证通过**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m pytest \
  src/tie_robot_perception/test/test_scan_surface_dp_runtime.py::ScanSurfaceDpRuntimeTest::test_execution_refine_algorithm_runtime_setting_is_subscribed_and_dispatches -q
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add src/tie_robot_perception/test/test_scan_surface_dp_runtime.py \
  src/tie_robot_perception/src/tie_robot_perception/pointai/state.py \
  src/tie_robot_perception/src/tie_robot_perception/pointai/runtime_config.py \
  src/tie_robot_perception/src/tie_robot_perception/pointai/ros_interfaces.py \
  src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py \
  src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py
git commit -m "feat(perception): route execution refine algorithm by setting"
```

---

### 任务 4：Surface-DP 执行微调分支

**文件：**
- 修改：`src/tie_robot_perception/test/test_scan_surface_dp_runtime.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py`
- 修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`

- [ ] **步骤 1：编写失败的测试**

在 `src/tie_robot_perception/test/test_scan_surface_dp_runtime.py` 中新增源码级行为测试：

```python
def test_execution_refine_surface_dp_reuses_scan_pipeline_and_keeps_execution_gates(self):
    source = _read_source(PERCEPTION_SRC / "tie_robot_perception" / "pointai" / "manual_workspace_s2.py")
    self.assertIn("def run_execution_refine_surface_dp_pipeline", source)
    start = source.index("def run_execution_refine_surface_dp_pipeline")
    end = source.index("\ndef run_manual_workspace_s2_depth_only_pipeline", start)
    body = source[start:end]
    self.assertIn("run_manual_workspace_surface_dp_pipeline", body)
    self.assertIn("is_camera_world_coord_in_execution_refine_tcp_range", body)
    self.assertIn("is_camera_world_coord_in_global_workspace", body)
    self.assertIn("select_output_centers_for_mode", body)
    self.assertIn("PROCESS_IMAGE_MODE_EXECUTION_REFINE", body)
    self.assertIn("execution_refine_base_image_pub", body)
    self.assertIn("execution_surface_dp_bind_point", body)
    self.assertNotIn("run_manual_workspace_s2_depth_only_pipeline", body)
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m pytest \
  src/tie_robot_perception/test/test_scan_surface_dp_runtime.py::ScanSurfaceDpRuntimeTest::test_execution_refine_surface_dp_reuses_scan_pipeline_and_keeps_execution_gates -q
```

预期：FAIL，失败原因包含 `def run_execution_refine_surface_dp_pipeline` 不存在。

- [ ] **步骤 3：实现 Surface-DP 执行分支**

`src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py` 当前已具备 `rospy`、`PointCoords`、`PointsArray` 和 `from .constants import *`，因此直接在 `run_manual_workspace_surface_dp_pipeline()` 后、`run_manual_workspace_s2_depth_only_pipeline()` 前加入：

```python
def run_execution_refine_surface_dp_pipeline(self, publish=True):
    start_time = time.perf_counter()
    surface_result = self.run_manual_workspace_surface_dp_pipeline(publish=publish)
    point_coords = surface_result.get("point_coords")
    if not surface_result.get("success", False) or not self.has_detected_points(point_coords):
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        return {
            "success": False,
            "message": "执行微调扫描同款Surface-DP未返回扫描点：{}".format(
                surface_result.get("message", "未知错误")
            ),
            "point_coords": point_coords,
            "single_frame_elapsed_ms": elapsed_ms,
        }

    in_range_centers = []
    out_of_range_count = 0
    out_of_range_reason_counts = {}
    out_of_range_samples = []
    for source_idx, point in enumerate(point_coords.PointCoordinatesArray):
        camera_coord = [float(value) for value in point.World_coord[:3]]
        pixel_coord = [int(point.Pix_coord[0]), int(point.Pix_coord[1])]
        is_in_tcp_range = self.is_camera_world_coord_in_execution_refine_tcp_range(camera_coord)
        is_in_global_workspace = self.is_camera_world_coord_in_global_workspace(camera_coord)
        center_record = (source_idx, pixel_coord, camera_coord)
        if is_in_tcp_range and is_in_global_workspace:
            in_range_centers.append(center_record)
            continue

        out_of_range_count += 1
        reasons = []
        if not is_in_global_workspace:
            reasons.append("超出全局工作区")
        if not is_in_tcp_range:
            reasons.append("超出TCP执行范围")
        reason = "+".join(reasons) or "超出执行范围"
        out_of_range_reason_counts[reason] = out_of_range_reason_counts.get(reason, 0) + 1
        if len(out_of_range_samples) < 5:
            out_of_range_samples.append(
                "idx={},pix=({},{}),coord=({:.1f},{:.1f},{:.1f}),原因={}".format(
                    source_idx,
                    pixel_coord[0],
                    pixel_coord[1],
                    camera_coord[0],
                    camera_coord[1],
                    camera_coord[2],
                    reason,
                )
            )

    output_centers = self.select_output_centers_for_mode(
        PROCESS_IMAGE_MODE_EXECUTION_REFINE,
        in_range_centers,
        [],
    )
    points_array_msg = PointsArray()
    points_array_msg.header.stamp = rospy.Time.now()
    points_array_msg.header.frame_id = "Scepter_depth_frame"
    for output_idx, (_source_idx, pixel_coord, camera_coord) in enumerate(output_centers, start=1):
        point_msg = PointCoords()
        point_msg.idx = output_idx
        point_msg.Pix_coord = [int(pixel_coord[0]), int(pixel_coord[1])]
        point_msg.World_coord = [float(camera_coord[0]), float(camera_coord[1]), float(camera_coord[2])]
        points_array_msg.PointCoordinatesArray.append(point_msg)
    points_array_msg.count = len(points_array_msg.PointCoordinatesArray)

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    self.last_detection_debug = {
        "algorithm": "surface_dp",
        "candidate_points": int(getattr(point_coords, "count", 0)),
        "in_range_candidates": len(in_range_centers),
        "selected_points": len(output_centers),
        "out_of_range_points": out_of_range_count,
        "output_points": points_array_msg.count,
    }
    rospy.loginfo(
        "执行微调Surface-DP统计：候选点=%d，范围内候选=%d，范围外点=%d，输出点=%d",
        self.last_detection_debug["candidate_points"],
        self.last_detection_debug["in_range_candidates"],
        self.last_detection_debug["out_of_range_points"],
        self.last_detection_debug["output_points"],
    )
    rospy.logwarn_throttle(
        1.0,
        self.build_detection_summary_log(
            request_mode=PROCESS_IMAGE_MODE_EXECUTION_REFINE,
            raw_candidate_count=int(getattr(point_coords, "count", 0)),
            in_range_candidate_count=len(in_range_centers),
            out_of_range_point_count=out_of_range_count,
            selected_count=len(output_centers),
            output_count=points_array_msg.count,
            out_of_range_reason_counts=out_of_range_reason_counts,
            out_of_range_samples=out_of_range_samples,
        ),
    )

    if publish:
        self.coordinate_publisher.publish(points_array_msg)
        result_image = surface_result.get("result_image")
        if result_image is not None and getattr(self, "execution_refine_base_image_pub", None) is not None:
            self.execution_refine_base_image_pub.publish(
                self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
            )
        if getattr(self, "lashing_points_camera_pub", None) is not None:
            self.lashing_points_camera_pub.publish(points_array_msg)
        previous_prefix = getattr(self, "raw_bind_point_tf_child_prefix", "surface_dp_bind_point")
        self.raw_bind_point_tf_child_prefix = "execution_surface_dp_bind_point"
        self.publish_raw_camera_bind_point_transforms(points_array_msg)
        self.raw_bind_point_tf_child_prefix = previous_prefix

    if points_array_msg.count <= 0:
        return {
            "success": False,
            "message": "执行微调扫描同款Surface-DP没有输出执行范围内点",
            "point_coords": points_array_msg,
            "single_frame_elapsed_ms": elapsed_ms,
        }
    return {
        "success": True,
        "message": "执行微调扫描同款Surface-DP输出{}个相机原始坐标点".format(points_array_msg.count),
        "point_coords": points_array_msg,
        "single_frame_elapsed_ms": elapsed_ms,
    }
```

修改 `src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py`，绑定方法：

```python
cls.run_execution_refine_surface_dp_pipeline = manual_workspace_s2.run_execution_refine_surface_dp_pipeline
```

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m pytest \
  src/tie_robot_perception/test/test_scan_surface_dp_runtime.py::ScanSurfaceDpRuntimeTest::test_execution_refine_surface_dp_reuses_scan_pipeline_and_keeps_execution_gates -q
```

预期：PASS。

- [ ] **步骤 5：运行相关感知测试**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m pytest \
  src/tie_robot_perception/test/test_scan_surface_dp_runtime.py \
  src/tie_robot_perception/test/test_bind_point_classification.py -q
```

预期：PASS。

- [ ] **步骤 6：Commit**

```bash
git add src/tie_robot_perception/test/test_scan_surface_dp_runtime.py \
  src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py \
  src/tie_robot_perception/src/tie_robot_perception/pointai/processor.py
git commit -m "feat(perception): support surface dp execution refine"
```

---

### 任务 5：分类和控制层边界回归

**文件：**
- 修改：`src/tie_robot_perception/test/test_bind_point_classification.py`
- 修改：`src/tie_robot_control/test/test_single_point_bind_chain.py`
- 可能修改：`src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py`

- [ ] **步骤 1：编写失败的测试**

在 `src/tie_robot_perception/test/test_bind_point_classification.py` 中追加源码边界测试：

```python
def test_surface_dp_execution_refine_still_uses_execution_refine_classification_phase(self):
    service_source = (PERCEPTION_SRC / "tie_robot_perception" / "pointai" / "process_image_service.py").read_text(encoding="utf-8")
    self.assertIn("run_execution_refine_visual_pipeline", service_source)
    self.assertIn("classify_execution_refine_points", service_source)
    self.assertIn('"execution_refine"', service_source)
```

在 `src/tie_robot_control/test/test_single_point_bind_chain.py` 的现有 `test_single_point_bind_uses_execution_refine_hough_and_dispatches_all_points` 附近补充或改名断言，使文案不再把控制层绑定到 Hough：

```python
def test_single_point_bind_does_not_choose_execution_refine_algorithm_in_control_layer(self):
    source = MODUAN_CALLBACKS_PATH.read_text(encoding="utf-8")
    start = source.index("bool ModuanRosCallbacks::single_point_bind_callback")
    end = source.index("\nbool ModuanRosCallbacks::", start + 1)
    body = source[start:end]
    self.assertIn("srv.request.request_mode = kProcessImageModeExecutionRefine;", body)
    self.assertNotIn("execution_refine_algorithm", body)
    self.assertNotIn("surface_dp", body)
```

- [ ] **步骤 2：运行测试验证失败或确认现状**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m pytest \
  src/tie_robot_perception/test/test_bind_point_classification.py::BindPointClassificationTest::test_surface_dp_execution_refine_still_uses_execution_refine_classification_phase \
  src/tie_robot_control/test/test_single_point_bind_chain.py::SinglePointBindChainTest::test_single_point_bind_does_not_choose_execution_refine_algorithm_in_control_layer -q
```

预期：如果任务 3 已正确实现，第一个可能 PASS；第二个如果类名或函数边界名字不同会 FAIL，需要按实际测试文件中的 helper/类名调整测试位置，但测试语义保持不变。

- [ ] **步骤 3：修正生产代码或测试定位**

如果第一个测试失败，确认 `evaluate_point_coords_for_mode()` 中 `PROCESS_IMAGE_MODE_EXECUTION_REFINE` 分支仍调用：

```python
point_coords = self.classify_execution_refine_points(point_coords)
```

如果第二个测试因旧测试名仍写 Hough 固定语义失败，把旧测试名和断言文案改为“uses execution refine service mode”，保留以下核心断言：

```python
self.assertIn("uint8 MODE_EXECUTION_REFINE=4", process_image_srv)
self.assertIn("srv.request.request_mode = kProcessImageModeExecutionRefine;", body)
self.assertIn("execute_bind_points", body)
self.assertNotIn("surface_dp", body)
self.assertNotIn("execution_refine_algorithm", body)
```

- [ ] **步骤 4：运行测试验证通过**

运行：

```bash
PYTHONPATH=src/tie_robot_perception/src python3 -m pytest \
  src/tie_robot_perception/test/test_bind_point_classification.py \
  src/tie_robot_control/test/test_single_point_bind_chain.py -q
```

预期：PASS。

- [ ] **步骤 5：Commit**

```bash
git add src/tie_robot_perception/test/test_bind_point_classification.py \
  src/tie_robot_control/test/test_single_point_bind_chain.py \
  src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py
git commit -m "test(control): keep execution refine algorithm inside pointai"
```

---

### 任务 6：前端构建产物和显示文案收口

**文件：**
- 修改：`src/tie_robot_web/frontend/src/config/topicRegistry.js`
- 修改：`src/tie_robot_web/frontend/src/config/imageTopicCatalog.js`
- 修改：`src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs`
- 修改：`src/tie_robot_web/web`

- [ ] **步骤 1：编写失败的测试**

修改 `src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs`，把固定 Hough 文案断言改成兼容执行视觉底图。例如追加：

```js
const executionOption = IMAGE_TOPIC_OPTIONS.find((option) => option.id === TOPICS.algorithm.executionRefineBaseImage);
assert.equal(executionOption.label, "执行视觉底图");
```

在 `visualDebugSettings.test.mjs` 或 `imageTopicCatalog.test.mjs` 中追加 topic registry 文案断言：

```js
assert.match(topicRegistryText, /执行视觉底图/);
assert.match(topicRegistryText, /Hough \/ Surface-DP/);
```

- [ ] **步骤 2：运行测试验证失败**

运行：

```bash
node src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs
node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
```

预期：FAIL，仍显示 `执行底图 Hough二值`。

- [ ] **步骤 3：修改显示文案**

修改 `src/tie_robot_web/frontend/src/config/topicRegistry.js` 中 `algorithm.executionRefineBaseImage`：

```js
label: "执行视觉底图",
usage: "执行微调当前算法的诊断底图，支持 Hough / Surface-DP 输出",
```

修改 `src/tie_robot_web/frontend/src/config/imageTopicCatalog.js` fallback：

```js
label: getRegistryLabel(TOPICS.algorithm.executionRefineBaseImage, "执行视觉底图"),
```

- [ ] **步骤 4：运行前端测试**

运行：

```bash
node src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs
node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
```

预期：PASS。

- [ ] **步骤 5：构建静态页面**

运行：

```bash
cd src/tie_robot_web/frontend && npm run build
```

预期：Vite build 成功，`src/tie_robot_web/web` 产物更新。

- [ ] **步骤 6：Commit**

```bash
git add src/tie_robot_web/frontend/src/config/topicRegistry.js \
  src/tie_robot_web/frontend/src/config/imageTopicCatalog.js \
  src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs \
  src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs \
  src/tie_robot_web/web
git commit -m "chore(web): rebuild execution vision selector assets"
```

---

### 任务 7：整体验证、记忆和收尾

**文件：**
- 修改：`CHANGELOG.md`
- 修改：`docs/agent_memory/current.md`，通过脚本生成
- 可能修改：`docs/agent_memory/session_log.md`，通过脚本追加

- [ ] **步骤 1：运行完整相关验证**

运行：

```bash
node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs
node src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs
PYTHONPATH=src/tie_robot_perception/src python3 -m pytest \
  src/tie_robot_perception/test/test_scan_surface_dp_runtime.py \
  src/tie_robot_perception/test/test_bind_point_classification.py \
  src/tie_robot_control/test/test_single_point_bind_chain.py -q
cd src/tie_robot_web/frontend && npm run build
```

预期：全部 PASS / build 成功。

- [ ] **步骤 2：更新 CHANGELOG**

在 `CHANGELOG.md` 的 `2026-05-15` 下新增条目：

```markdown
### 执行层视觉算法可切换

- “视觉调试”新增“执行层视觉”选择，默认保持 `Hough微调`，可切换为 `扫描同款Surface-DP`。
- 前端通过 `/web/pointAI/set_execution_refine_algorithm` 热发布选择；单点绑扎、单点视觉测试、执行全局绑扎和记忆续跑仍统一调用 `/pointAI/process_image request_mode=4`。
- pointAI 内部按运行态参数选择 Hough 或 Surface-DP；Surface-DP 执行分支复用扫描层 Surface-DP，并继续套 TCP 执行盒、全局工作区过滤、TCP 蛇形排序和执行层分类过滤。
```

- [ ] **步骤 3：写入共享记忆**

运行：

```bash
python3 scripts/agent_memory.py add \
  --title "执行层视觉算法源可切换" \
  --summary "2026-05-15：视觉调试新增执行层视觉算法选择，默认Hough微调，可切扫描同款Surface-DP；前端热发布/web/pointAI/set_execution_refine_algorithm，执行链仍统一走/pointAI/process_image request_mode=4，由pointAI内部按参数分发。Surface-DP执行分支复用扫描层Surface-DP后继续执行TCP盒、全局工作区、蛇形排序和分类过滤。" \
  --files "src/tie_robot_web/frontend/src/config/visualRecognitionMode.js,src/tie_robot_web/frontend/src/ui/UIController.js,src/tie_robot_web/frontend/src/controllers/RosConnectionController.js,src/tie_robot_perception/src/tie_robot_perception/pointai/process_image_service.py,src/tie_robot_perception/src/tie_robot_perception/pointai/manual_workspace_s2.py" \
  --validation "node src/tie_robot_web/frontend/test/visualDebugSettings.test.mjs; node src/tie_robot_web/frontend/test/imageTopicCatalog.test.mjs; PYTHONPATH=src/tie_robot_perception/src python3 -m pytest src/tie_robot_perception/test/test_scan_surface_dp_runtime.py src/tie_robot_perception/test/test_bind_point_classification.py src/tie_robot_control/test/test_single_point_bind_chain.py -q; cd src/tie_robot_web/frontend && npm run build"
python3 scripts/agent_memory.py refresh
python3 scripts/agent_memory.py check
```

预期：三条脚本成功，`docs/agent_memory/current.md` 刷新。

- [ ] **步骤 4：最终 git 检查**

运行：

```bash
git status --short
```

预期：只剩本功能相关文件变更；如果前面每个任务都 commit，此处应干净或只剩记忆/CHANGELOG 最后一笔。

- [ ] **步骤 5：Commit**

```bash
git add CHANGELOG.md docs/agent_memory/current.md docs/agent_memory/session_log.md
git commit -m "docs(memory): record execution vision algorithm switch"
```

---

## 自检

- 规格覆盖度：计划覆盖前端配置、UI、localStorage、ROS topic 发布、pointAI 订阅、运行态参数、算法分发、Surface-DP 执行分支、分类过滤、控制层边界、构建产物和共享记忆。
- 占位符扫描：没有“待定 / TODO / 后续实现”等占位步骤；每个代码变更步骤给出具体路径、代码或精确行为。
- 类型一致性：前端字段统一为 `executionRefineAlgorithm`；ROS topic 统一为 `/web/pointAI/set_execution_refine_algorithm`；后端属性统一为 `execution_refine_algorithm`；算法值统一为 `hough` / `surface_dp`。
- 范围控制：不新增服务字段，不改 `ProcessImage.srv`，不让控制层理解算法源，不改变 Hough 默认行为。
