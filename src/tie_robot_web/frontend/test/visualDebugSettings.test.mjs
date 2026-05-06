import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import { ROSLIB } from "../src/vendor/roslib.js";
import { RosConnectionController } from "../src/controllers/RosConnectionController.js";
import { MESSAGE_TYPES, SERVICE_TYPES, SERVICES, TOPICS } from "../src/config/topicRegistry.js";
import {
  VISUAL_DEBUG_SETTINGS_KEY,
  loadVisualDebugSettings,
  saveVisualDebugSettings,
} from "../src/utils/storage.js";
import { GLOBAL_EXECUTION_MODES } from "../src/config/visualRecognitionMode.js";

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
assert.equal(MESSAGE_TYPES.float32MultiArray, "std_msgs/Float32MultiArray");
assert.equal(SERVICES.algorithm.processImage, "/pointAI/process_image");
assert.equal(SERVICE_TYPES.algorithm.processImage, "tie_robot_msgs/ProcessImage");
assert.equal(TOPICS.algorithm.setStableFrameCount, "/web/pointAI/set_stable_frame_count");
assert.equal(TOPICS.algorithm.setExecutionRefineTcpRoi, "/web/pointAI/set_execution_refine_tcp_roi");

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

localStorage.clear();
assert.deepEqual(loadVisualDebugSettings().linearModuleBindRangeMm, {
  x: { min: 0, max: 380 },
  y: { min: 0, max: 330 },
  z: { min: 0, max: 160 },
});
assert.equal(loadVisualDebugSettings().executionMode, GLOBAL_EXECUTION_MODES.LEDGER_WITH_REFINE);

localStorage.setItem(VISUAL_DEBUG_SETTINGS_KEY, JSON.stringify({
  stableFrameCount: 5,
  executionMode: GLOBAL_EXECUTION_MODES.PLANNED_PATH_REFINE_ONLY,
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
  linearModuleBindRangeMm: {
    x: { min: 30, max: 180 },
    y: { min: 40, max: 280 },
    z: { min: 10, max: 95 },
  },
});

saveVisualDebugSettings({
  stableFrameCount: 2,
  executionMode: GLOBAL_EXECUTION_MODES.SLAM_PRECOMPUTED,
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
  linearModuleBindRangeMm: {
    x: { min: 20, max: 150 },
    y: { min: 25, max: 225 },
    z: { min: 5, max: 45 },
  },
});

const serviceResult = await controller.callProcessImageService();
assert.equal(serviceResult.success, true);
assert.equal(serviceResult.count, 8);
assert.equal(serviceResult.serviceElapsedMs, 123.4);
assert.equal(controller.resources.processImageService.calls.at(-1).request_mode, 3);

const uiControllerText = readFileSync(resolve(frontendRoot, "src/ui/UIController.js"), "utf-8");
const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");
assert.match(uiControllerText, /id: "visualDebug", label: "视觉调试"/);
assert.match(uiControllerText, /id="visualDebugTrigger"/);
assert.match(uiControllerText, /id="visualDebugStableFrameCount"/);
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
assert.doesNotMatch(uiControllerText, /id="visualDebugRequestMode"/);
assert.doesNotMatch(uiControllerText, /TCP z=0 x0-380 y0-330/);
assert.doesNotMatch(uiControllerText, /tcp工具坐标系下x[:：]0~380/);

const visualDebugPageStart = uiControllerText.indexOf('<section class="settings-page" data-settings-page="visualDebug" hidden>');
const visualDebugPageEnd = uiControllerText.indexOf('<section class="settings-page" data-settings-page="gb28181Local" hidden>');
assert.notEqual(visualDebugPageStart, -1);
assert.notEqual(visualDebugPageEnd, -1);
const visualDebugPageMarkup = uiControllerText.slice(visualDebugPageStart, visualDebugPageEnd);
assert.equal(
  visualDebugPageMarkup.indexOf("gripper-tf-calibration-card") < visualDebugPageMarkup.indexOf("visual-debug-control-card"),
  true,
);
assert.doesNotMatch(visualDebugPageMarkup, /visual-debug-log-card/);
assert.doesNotMatch(visualDebugPageMarkup, /视觉调试日志|暂无视觉调试记录/);

assert.match(appText, /VISUAL_FRAME_SYNC_TASK_ACTIONS/);
for (const actionId of ["runSavedS2", "executionVisionOnly", "triggerSingleBind", "scanPlan", "startExecution", "startExecutionKeepMemory"]) {
  assert.match(appText, new RegExp(`"${actionId}"`));
}
assert.match(appText, /VISUAL_FRAME_SYNC_TASK_ACTIONS\.has\(taskAction\)/);
assert.match(appText, /this\.applyVisualDebugStableFrameCount\(\{ suppressLog: true \}\)/);
