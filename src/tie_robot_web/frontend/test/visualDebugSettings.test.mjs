import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import { ROSLIB } from "../src/vendor/roslib.js";
import { RosConnectionController } from "../src/controllers/RosConnectionController.js";
import { CAMERA_SDK_PARAMETER_DEFINITIONS } from "../src/config/cameraSdkDynamicReconfigure.js";
import { MESSAGE_TYPES, SERVICE_TYPES, SERVICES, TOPICS } from "../src/config/topicRegistry.js";
import {
  CAMERA_SDK_SETTINGS_KEY,
  VISUAL_DEBUG_SETTINGS_KEY,
  loadCameraSdkSettings,
  loadVisualDebugSettings,
  saveCameraSdkSettings,
  saveVisualDebugSettings,
} from "../src/utils/storage.js";
import {
  DEFAULT_SCAN_RESPONSE_SOURCE,
  GLOBAL_EXECUTION_MODES,
  SCAN_RESPONSE_SOURCE_OPTIONS,
} from "../src/config/visualRecognitionMode.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

global.window = {
  location: {
    hostname: "127.0.0.1",
    pathname: "/index.html",
    protocol: "http:",
  },
  setTimeout() {
    return 1;
  },
  clearTimeout() {},
  setInterval() {
    return 1;
  },
  clearInterval() {},
};

let nowMs = 1000;
global.performance = {
  now() {
    const current = nowMs;
    nowMs += 123.4;
    return current;
  },
};

const localStorageState = new Map();
global.localStorage = {
  getItem(key) {
    return localStorageState.has(key) ? localStorageState.get(key) : null;
  },
  setItem(key, value) {
    localStorageState.set(key, String(value));
  },
  removeItem(key) {
    localStorageState.delete(key);
  },
  clear() {
    localStorageState.clear();
  },
};

class FakeRos {
  constructor() {
    this.isConnected = true;
  }

  on() {}
}

class FakeTopic {
  constructor(options) {
    Object.assign(this, options);
    this.published = [];
  }

  advertise() {}

  publish(message) {
    this.published.push(message);
  }

  subscribe() {}

  unsubscribe() {}
}

class FakeService {
  constructor(options) {
    Object.assign(this, options);
    this.calls = [];
  }

  callService(request, success) {
    this.calls.push(request);
    success({
      success: true,
      message: "点位已满足2帧稳定",
      count: 8,
    });
  }
}

class FakeActionClient {
  constructor(options) {
    Object.assign(this, options);
  }
}

ROSLIB.Ros = FakeRos;
ROSLIB.Topic = FakeTopic;
ROSLIB.Service = FakeService;
ROSLIB.ActionClient = FakeActionClient;
ROSLIB.Message = class {
  constructor(payload) {
    Object.assign(this, payload);
  }
};
ROSLIB.ServiceRequest = class {
  constructor(payload) {
    Object.assign(this, payload);
  }
};

assert.equal(MESSAGE_TYPES.int32, "std_msgs/Int32");
assert.equal(MESSAGE_TYPES.string, "std_msgs/String");
assert.equal(MESSAGE_TYPES.float32MultiArray, "std_msgs/Float32MultiArray");
assert.equal(MESSAGE_TYPES.dynamicReconfigureConfig, "dynamic_reconfigure/Config");
assert.equal(SERVICES.algorithm.processImage, "/pointAI/process_image");
assert.equal(SERVICES.camera.scepterSetParameters, "/scepter_manager/set_parameters");
assert.equal(SERVICE_TYPES.algorithm.processImage, "tie_robot_msgs/ProcessImage");
assert.equal(SERVICE_TYPES.camera.dynamicReconfigure, "dynamic_reconfigure/Reconfigure");
assert.equal(TOPICS.algorithm.setStableFrameCount, "/web/pointAI/set_stable_frame_count");
assert.equal(TOPICS.algorithm.setExecutionRefineTcpRoi, "/web/pointAI/set_execution_refine_tcp_roi");
assert.equal(TOPICS.algorithm.setScanBeamExclusion, "/web/pointAI/set_scan_beam_exclusion");
assert.equal(TOPICS.algorithm.setScanBeamExclusionMargin, "/web/pointAI/set_scan_beam_exclusion_margin_mm");
assert.equal(TOPICS.algorithm.setScanResponseSource, "/web/pointAI/set_scan_response_source");
assert.equal(TOPICS.algorithm.setScanLinearCompensation, "/web/pointAI/set_scan_linear_compensation");
assert.equal(TOPICS.camera.scepterParameterUpdates, "/scepter_manager/parameter_updates");
assert.equal(TOPICS.camera.scepterParameterDescriptions, "/scepter_manager/parameter_descriptions");
assert.equal(DEFAULT_SCAN_RESPONSE_SOURCE, "depth_gradient");
assert.deepEqual(SCAN_RESPONSE_SOURCE_OPTIONS.map((option) => option.label), [
  "融合实例响应",
  "Frangi-like 脊线",
  "Hessian ridge 脊线",
  "深度梯度边缘",
  "红外响应",
  "组合响应",
  "深度响应",
]);
assert.deepEqual(
  CAMERA_SDK_PARAMETER_DEFINITIONS
    .filter((definition) => ["FrameRate", "XDRMode", "ColorResloution", "DepthCloudPoint"].includes(definition.name))
    .map((definition) => definition.name),
  ["FrameRate", "ColorResloution", "XDRMode", "DepthCloudPoint"],
);
assert.deepEqual(
  CAMERA_SDK_PARAMETER_DEFINITIONS
    .filter((definition) => ["FrameRate", "XDRMode", "ToFExposureTime", "DepthCloudPoint"].includes(definition.name))
    .map((definition) => definition.label),
  ["帧率", "动态范围模式", "深度曝光时间", "深度点云"],
);
assert.deepEqual(
  CAMERA_SDK_PARAMETER_DEFINITIONS.find((definition) => definition.name === "XDRMode").options.map((option) => option.label),
  ["普通模式", "高动态范围模式", "宽动态范围模式"],
);
assert.deepEqual(
  CAMERA_SDK_PARAMETER_DEFINITIONS.find((definition) => definition.name === "WorkMode").options.map((option) => option.label),
  ["主动模式", "硬件触发模式", "软件触发模式"],
);

const controller = new RosConnectionController();
controller.ros = new FakeRos();
controller.resources = controller.buildResources(controller.ros);

const frameCountResult = controller.publishStableFrameCount(2);
assert.equal(frameCountResult.success, true);
assert.equal(controller.resources.stableFrameCountPublisher.published.at(-1).data, 2);

const tcpRoiResult = controller.publishExecutionRefineTcpRoi({
  x: { min: 150, max: 20 },
  y: { min: 25, max: 225 },
  z: { min: 5, max: 45 },
});
assert.equal(tcpRoiResult.success, true);
assert.deepEqual(
  controller.resources.executionRefineTcpRoiPublisher.published.at(-1).data,
  [20, 150, 25, 225, 5, 45],
);

const beamExclusionResult = controller.publishScanBeamExclusion(true);
assert.equal(beamExclusionResult.success, true);
assert.equal(controller.resources.scanBeamExclusionPublisher.published.at(-1).data, true);
assert.match(beamExclusionResult.message, /扫描梁筋过滤已启用/);
const beamMarginResult = controller.publishScanBeamExclusionMargin(180);
assert.equal(beamMarginResult.success, true);
assert.equal(controller.resources.scanBeamExclusionMarginPublisher.published.at(-1).data, 180);
assert.match(beamMarginResult.message, /扫描梁筋过滤半径已设置为 180 mm/);
const invalidBeamMarginResult = controller.publishScanBeamExclusionMargin(-5);
assert.equal(invalidBeamMarginResult.marginMm, 150);
assert.equal(controller.resources.scanBeamExclusionMarginPublisher.published.at(-1).data, 150);

const scanSourceResult = controller.publishScanResponseSource("frangi_like");
assert.equal(scanSourceResult.success, true);
assert.equal(controller.resources.scanResponseSourcePublisher.published.at(-1).data, "frangi_like");
const invalidScanSourceResult = controller.publishScanResponseSource("bad_source");
assert.equal(invalidScanSourceResult.source, DEFAULT_SCAN_RESPONSE_SOURCE);
assert.equal(controller.resources.scanResponseSourcePublisher.published.at(-1).data, DEFAULT_SCAN_RESPONSE_SOURCE);
const scanLinearCompensationResult = controller.publishScanLinearCompensation({
  enabled: true,
  referenceZMm: 1000,
  xPercentPerMeter: 1.2,
  yPercentPerMeter: 2.4,
  minZMm: 1200,
  maxScaleDelta: 0.25,
});
assert.equal(scanLinearCompensationResult.success, true);
assert.deepEqual(
  controller.resources.scanLinearCompensationPublisher.published.at(-1).data,
  [1, 1000, 0.000012, 0.000024, 1200, 0.25],
);
assert.match(scanLinearCompensationResult.message, /扫描线性补偿已启用/);

const cameraSdkResult = await controller.callScepterCameraReconfigure({
  FrameRate: 8,
  ToFManual: false,
  XDRMode: 2,
  ColorExposureTime: 12000,
});
assert.equal(cameraSdkResult.success, true);
assert.deepEqual(controller.resources.scepterCameraReconfigureService.calls.at(-1).config.ints, [
  { name: "FrameRate", value: 8 },
  { name: "XDRMode", value: 2 },
  { name: "ColorExposureTime", value: 12000 },
]);
assert.deepEqual(controller.resources.scepterCameraReconfigureService.calls.at(-1).config.bools, [
  { name: "ToFManual", value: false },
]);

localStorage.clear();
assert.deepEqual(loadVisualDebugSettings().linearModuleBindRangeMm, {
  x: { min: 0, max: 380 },
  y: { min: 0, max: 330 },
  z: { min: 0, max: 160 },
});
assert.equal(loadVisualDebugSettings().bindExecutionCabinMinZMm, 485);
assert.equal(loadVisualDebugSettings().adaptiveBindGrouping, false);
assert.equal(loadVisualDebugSettings().enableBeamExclusion, false);
assert.equal(loadVisualDebugSettings().beamExclusionMarginMm, 150);
assert.equal(loadVisualDebugSettings().executionMode, GLOBAL_EXECUTION_MODES.LEDGER_WITH_REFINE);
assert.equal(loadVisualDebugSettings().scanResponseSource, DEFAULT_SCAN_RESPONSE_SOURCE);
assert.deepEqual(loadVisualDebugSettings().scanLinearCompensation, {
  enabled: false,
  referenceZMm: 1000,
  xPercentPerMeter: 0,
  yPercentPerMeter: 0,
  minZMm: 1200,
  maxScaleDelta: 0.25,
});
assert.equal(loadCameraSdkSettings().FrameRate, 5);
assert.equal(loadCameraSdkSettings().ColorResloution, 2);
assert.equal(loadCameraSdkSettings().DepthCloudPoint, true);

localStorage.setItem(VISUAL_DEBUG_SETTINGS_KEY, JSON.stringify({
  stableFrameCount: 5,
  executionMode: GLOBAL_EXECUTION_MODES.PLANNED_PATH_REFINE_ONLY,
  scanResponseSource: "hessian_ridge",
  adaptiveBindGrouping: true,
  enableBeamExclusion: true,
  beamExclusionMarginMm: 175,
  scanLinearCompensation: {
    enabled: true,
    referenceZMm: 900,
    xPercentPerMeter: 1.5,
    yPercentPerMeter: 2.5,
    minZMm: 1300,
    maxScaleDelta: 0.3,
  },
  bindExecutionCabinMinZMm: 420,
  linearModuleBindRangeMm: {
    x: { min: 30, max: 180 },
    y: { min: 40, max: 280 },
    z: { min: 10, max: 95 },
  },
}));
assert.deepEqual(loadVisualDebugSettings(), {
  stableFrameCount: 5,
  requestMode: 3,
  executionMode: GLOBAL_EXECUTION_MODES.PLANNED_PATH_REFINE_ONLY,
  scanResponseSource: "hessian_ridge",
  adaptiveBindGrouping: true,
  enableBeamExclusion: true,
  beamExclusionMarginMm: 175,
  scanLinearCompensation: {
    enabled: true,
    referenceZMm: 900,
    xPercentPerMeter: 1.5,
    yPercentPerMeter: 2.5,
    minZMm: 1300,
    maxScaleDelta: 0.3,
  },
  bindExecutionCabinMinZMm: 420,
  linearModuleBindRangeMm: {
    x: { min: 30, max: 180 },
    y: { min: 40, max: 280 },
    z: { min: 10, max: 95 },
  },
});

saveVisualDebugSettings({
  stableFrameCount: 2,
  executionMode: GLOBAL_EXECUTION_MODES.SLAM_PRECOMPUTED,
  scanResponseSource: "infrared_response",
  adaptiveBindGrouping: true,
  enableBeamExclusion: true,
  beamExclusionMarginMm: 180,
  scanLinearCompensation: {
    enabled: true,
    referenceZMm: 1000,
    xPercentPerMeter: 1.2,
    yPercentPerMeter: 2.4,
    minZMm: 1200,
    maxScaleDelta: 0.25,
  },
  bindExecutionCabinMinZMm: 430,
  linearModuleBindRangeMm: {
    x: { min: 150, max: 20 },
    y: { min: 25, max: 225 },
    z: { min: 5, max: 45 },
  },
});
assert.deepEqual(JSON.parse(localStorage.getItem(VISUAL_DEBUG_SETTINGS_KEY)), {
  stableFrameCount: 2,
  requestMode: 3,
  executionMode: GLOBAL_EXECUTION_MODES.SLAM_PRECOMPUTED,
  scanResponseSource: "infrared_response",
  adaptiveBindGrouping: true,
  enableBeamExclusion: true,
  beamExclusionMarginMm: 180,
  scanLinearCompensation: {
    enabled: true,
    referenceZMm: 1000,
    xPercentPerMeter: 1.2,
    yPercentPerMeter: 2.4,
    minZMm: 1200,
    maxScaleDelta: 0.25,
  },
  bindExecutionCabinMinZMm: 430,
  linearModuleBindRangeMm: {
    x: { min: 20, max: 150 },
    y: { min: 25, max: 225 },
    z: { min: 5, max: 45 },
  },
});

saveCameraSdkSettings({
  FrameRate: 40,
  IRGMMGain: -20,
  ColorResloution: 1,
  XDRMode: 2,
  ToFManual: false,
  ToFExposureTime: 7000,
  DepthCloudPoint: false,
});
assert.deepEqual(JSON.parse(localStorage.getItem(CAMERA_SDK_SETTINGS_KEY)), {
  FrameRate: 15,
  IRGMMGain: 0,
  ColorResloution: 1,
  XDRMode: 2,
  ToFManual: false,
  ToFExposureTime: 5000,
  FlyingPixelenable: false,
  FlyingPixelvalue: 10,
  Confidenceenable: false,
  Confidencevalue: 4,
  TimeFilterenable: true,
  TimeFiltervalue: 3,
  IRGMMCorrectionenable: false,
  IRGMMCorrectionvalue: 79,
  SpatialFilterEnabled: false,
  FillHoleFilterEnabled: false,
  ColorManual: false,
  ColorExposureTime: 1000,
  WorkMode: 0,
  SoftwareTrigger: false,
  DepthCloudPoint: false,
  Depth2ColorCloudPoint: false,
});

const serviceResult = await controller.callProcessImageService();
assert.equal(serviceResult.success, true);
assert.equal(serviceResult.count, 8);
assert.equal(serviceResult.serviceElapsedMs, 123.4);
assert.equal(controller.resources.processImageService.calls.at(-1).request_mode, 3);

const uiControllerText = readFileSync(resolve(frontendRoot, "src/ui/UIController.js"), "utf-8");
const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");
assert.match(uiControllerText, /id: "visualDebug", label: "视觉调试"/);
assert.match(uiControllerText, /id: "cameraSdkDebug", label: "相机底层 SDK 调试"/);
assert.doesNotMatch(uiControllerText, /id="visualDebugTrigger"/);
assert.doesNotMatch(uiControllerText, /触发视觉服务/);
assert.match(uiControllerText, /id="visualDebugStableFrameCount"/);
assert.match(uiControllerText, /id="visualDebugScanResponseSource"/);
assert.match(uiControllerText, /扫描底图/);
assert.match(uiControllerText, /SCAN_RESPONSE_SOURCE_OPTIONS\.map/);
assert.match(uiControllerText, /id="visualDebugBindExecutionCabinMinZ"/);
assert.match(uiControllerText, /索驱规划 Z 下限/);
assert.match(uiControllerText, /id="visualDebugAdaptiveBindGrouping"/);
assert.match(uiControllerText, /自适应每组绑扎点数/);
assert.match(uiControllerText, /id="visualDebugBeamExclusionToggle"/);
assert.match(uiControllerText, /启用梁筋过滤/);
assert.match(uiControllerText, /id="visualDebugBeamExclusionMargin"/);
assert.match(uiControllerText, /梁筋过滤半径 \(mm\)/);
assert.match(uiControllerText, /id="visualDebugScanLinearCompensationToggle"/);
assert.match(uiControllerText, /扫描线性补偿/);
assert.match(uiControllerText, /id="visualDebugScanLinearCompensationX"/);
assert.match(uiControllerText, /id="visualDebugScanLinearCompensationY"/);
assert.match(uiControllerText, /id="visualDebugScanLinearCompensationReferenceZ"/);
assert.match(uiControllerText, /id="visualDebugScanLinearCompensationMinZ"/);
assert.match(uiControllerText, /id="visualDebugScanLinearCompensationMaxDelta"/);
assert.match(uiControllerText, /id="visualDebugBindRangeXMin"/);
assert.match(uiControllerText, /id="visualDebugBindRangeXMax"/);
assert.match(uiControllerText, /id="visualDebugBindRangeYMin"/);
assert.match(uiControllerText, /id="visualDebugBindRangeYMax"/);
assert.match(uiControllerText, /id="visualDebugBindRangeZMin"/);
assert.match(uiControllerText, /id="visualDebugBindRangeZMax"/);
assert.match(uiControllerText, /visualDebugExecutionModeLedgerRefine/);
assert.match(uiControllerText, /账本\+微调/);
assert.match(uiControllerText, /规划路径\+纯微调/);
assert.match(uiControllerText, /id="visualDebugTimingSummary"/);
assert.match(uiControllerText, /相机底层 SDK 调试/);
assert.match(uiControllerText, /CAMERA_SDK_PARAMETER_DEFINITIONS\.map\(renderCameraSdkParameterControl\)/);
assert.match(uiControllerText, /data-camera-sdk-param/);
assert.match(uiControllerText, /热修改服务：\/scepter_manager\/set_parameters/);
assert.match(uiControllerText, /Z下限=/);
assert.doesNotMatch(uiControllerText, /id="visualDebugRequestMode"/);
assert.doesNotMatch(uiControllerText, /id="visualDebugBindGroupPointCount"/);
assert.doesNotMatch(uiControllerText, /每组点数/);
assert.doesNotMatch(uiControllerText, /visualDebugApplyStableFrameCount|应用帧数/);
assert.doesNotMatch(uiControllerText, /TCP z=0 x0-380 y0-330/);
assert.doesNotMatch(uiControllerText, /tcp工具坐标系下x[:：]0~380/);

const visualDebugPageStart = uiControllerText.indexOf('<section class="settings-page" data-settings-page="visualDebug" hidden>');
const visualDebugPageEnd = uiControllerText.indexOf('<section class="settings-page" data-settings-page="cameraSdkDebug" hidden>');
assert.notEqual(visualDebugPageStart, -1);
assert.notEqual(visualDebugPageEnd, -1);
const visualDebugPageMarkup = uiControllerText.slice(visualDebugPageStart, visualDebugPageEnd);
assert.equal(
  visualDebugPageMarkup.indexOf("gripper-tf-calibration-card") < visualDebugPageMarkup.indexOf("visual-debug-control-card"),
  true,
);
assert.doesNotMatch(visualDebugPageMarkup, /visual-debug-log-card/);
assert.doesNotMatch(visualDebugPageMarkup, /视觉调试日志|暂无视觉调试记录/);
assert.doesNotMatch(visualDebugPageMarkup, /camera-sdk-debug-card|相机底层 SDK 调试|data-camera-sdk-param/);

const cameraSdkPageStart = uiControllerText.indexOf('<section class="settings-page" data-settings-page="cameraSdkDebug" hidden>');
const cameraSdkPageEnd = uiControllerText.indexOf('<section class="settings-page" data-settings-page="gb28181Local" hidden>');
assert.notEqual(cameraSdkPageStart, -1);
assert.notEqual(cameraSdkPageEnd, -1);
const cameraSdkPageMarkup = uiControllerText.slice(cameraSdkPageStart, cameraSdkPageEnd);
assert.match(cameraSdkPageMarkup, /camera-sdk-debug-card/);
assert.match(cameraSdkPageMarkup, /相机底层 SDK 调试/);
assert.match(cameraSdkPageMarkup, /CAMERA_SDK_PARAMETER_DEFINITIONS\.map\(renderCameraSdkParameterControl\)/);

assert.match(appText, /VISUAL_FRAME_SYNC_TASK_ACTIONS/);
assert.match(appText, /applyVisualDebugBeamExclusionSettings/);
const visualDebugSettingsChangeStart = appText.indexOf("this.ui.onVisualDebugSettingsChange((settings) => {");
const visualDebugSettingsChangeEnd = appText.indexOf("this.ui.onLegacyCommand", visualDebugSettingsChangeStart);
assert.notEqual(visualDebugSettingsChangeStart, -1);
assert.notEqual(visualDebugSettingsChangeEnd, -1);
const visualDebugSettingsChangeBlock = appText.slice(
  visualDebugSettingsChangeStart,
  visualDebugSettingsChangeEnd,
);
assert.match(visualDebugSettingsChangeBlock, /this\.applyVisualDebugRuntimeSettings\(settings, \{ suppressLog: true \}\)/);
const visualDebugRuntimeSettingsStart = appText.indexOf("applyVisualDebugRuntimeSettings(");
const visualDebugRuntimeSettingsEnd = appText.indexOf("applyVisualDebugStableFrameCount", visualDebugRuntimeSettingsStart);
assert.notEqual(visualDebugRuntimeSettingsStart, -1);
assert.notEqual(visualDebugRuntimeSettingsEnd, -1);
const visualDebugRuntimeSettingsBlock = appText.slice(
  visualDebugRuntimeSettingsStart,
  visualDebugRuntimeSettingsEnd,
);
assert.match(visualDebugRuntimeSettingsBlock, /saveVisualDebugSettings\(nextSettings\)/);
assert.match(visualDebugRuntimeSettingsBlock, /this\.applyVisualDebugBindRangeSettings\(nextSettings\)/);
assert.match(visualDebugRuntimeSettingsBlock, /publishStableFrameCount\(nextSettings\.stableFrameCount\)/);
assert.match(visualDebugRuntimeSettingsBlock, /publishScanResponseSource\(nextSettings\.scanResponseSource\)/);
assert.match(visualDebugRuntimeSettingsBlock, /publishScanLinearCompensation\(nextSettings\.scanLinearCompensation\)/);
assert.match(visualDebugRuntimeSettingsBlock, /publishScanBeamExclusionMargin\(\s*nextSettings\.beamExclusionMarginMm,\s*\)/);
assert.match(visualDebugRuntimeSettingsBlock, /this\.applyVisualDebugBeamExclusionSettings\(nextSettings, \{ suppressLog \}\)/);
assert.match(appText, /this\.applyCameraSdkSettings\(this\.cameraSdkSettings, \{ suppressLog: true \}\)/);
assert.match(appText, /this\.ui\.onCameraSdkSettingsChange/);
assert.match(appText, /saveCameraSdkSettings\(nextSettings\)/);
assert.match(appText, /callScepterCameraReconfigure\(nextSettings\)/);
assert.doesNotMatch(appText, /onVisualDebugApplyStableFrameCount/);
assert.doesNotMatch(appText, /onVisualDebugTrigger|handleVisualDebugTrigger|VISUAL_DEBUG_REQUEST_MODE_LABELS/);
for (const actionId of ["runSavedS2", "executionVisionOnly", "triggerSingleBind", "startExecution", "startExecutionKeepMemory"]) {
  assert.match(appText, new RegExp(`"${actionId}"`));
}
assert.doesNotMatch(appText, /"scanPlan"/);
assert.match(appText, /VISUAL_FRAME_SYNC_TASK_ACTIONS\.has\(taskAction\)/);
assert.match(appText, /this\.applyVisualDebugRuntimeSettings\(this\.visualDebugSettings, \{ suppressLog: true \}\)/);
assert.match(appText, /adaptiveBindGrouping/);
assert.match(appText, /getBindExecutionCabinMinZ/);
assert.doesNotMatch(appText, /bindGroupPointCount/);
