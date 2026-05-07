import assert from "node:assert/strict";

import { CONTROL_PANEL_TASKS, CONTROL_PANEL_TASK_SECTIONS } from "../src/config/controlPanelCatalog.js";
import { GLOBAL_EXECUTION_MODES, PROCESS_IMAGE_REQUEST_MODES } from "../src/config/visualRecognitionMode.js";
import { TaskActionController } from "../src/controllers/TaskActionController.js";
import { ROSLIB } from "../src/vendor/roslib.js";

global.window = {
  setTimeout() {
    return 1;
  },
  clearTimeout() {},
};

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

const actionGoals = [];
ROSLIB.Goal = class {
  constructor({ actionClient, goalMessage }) {
    this.actionClient = actionClient;
    this.goalMessage = goalMessage;
    this.handlers = {};
    this.sent = false;
    actionGoals.push(this);
  }

  on(eventName, handler) {
    this.handlers[eventName] = handler;
  }

  send() {
    this.sent = true;
    this.handlers.feedback?.({ detail: "正在覆盖本地绑扎点JSON" });
    this.handlers.result?.({
      success: true,
      message: "扫描建图完成，pseudo_slam_points.json=256个点，pseudo_slam_bind_path.json=1个区域/16个分组/256个绑扎点",
    });
  }
};

const submitTask = CONTROL_PANEL_TASKS.find((task) => task.id === "submitQuad");
const runSavedS2Task = CONTROL_PANEL_TASKS.find((task) => task.id === "runSavedS2");
const executionVisionOnlyTask = CONTROL_PANEL_TASKS.find((task) => task.id === "executionVisionOnly");
const startExecutionTask = CONTROL_PANEL_TASKS.find((task) => task.id === "startExecution");
const taskIds = CONTROL_PANEL_TASKS.map((task) => task.id);
assert.equal(submitTask?.label, "确认\n工作区域");
assert.equal(runSavedS2Task?.label, "触发扫描\n视觉");
assert.equal(executionVisionOnlyTask?.label, "单点视觉\n测试");
assert.equal(startExecutionTask?.label, "执行全局\n绑扎");
assert.deepEqual(CONTROL_PANEL_TASK_SECTIONS.map((section) => section.title), ["扫描区", "执行层", "区域切换"]);
assert.deepEqual(taskIds, [
  "moveToPosition",
  "setRecognitionPose",
  "submitQuad",
  "runSavedS2",
  "startExecution",
  "triggerSingleBind",
  "executionVisionOnly",
  "startExecutionKeepMemory",
  "previousArea",
  "nextArea",
]);
assert.equal(taskIds.includes("clearVisualRecognition"), false);
assert.equal(taskIds.includes("scanPlan"), false);
assert.equal(taskIds.includes("runBindPathTest"), false);

const payload = [10, 20, 110, 20, 110, 120, 10, 120];
const publishedMessages = [];
const logs = [];
const resultMessages = [];
const processImageCalls = [];
const singlePointBindCalls = [];
const executionModeCalls = [];
const scanActionClient = { name: "start_pseudo_slam_scan" };
const startGlobalWorkActionClient = { name: "start_global_work" };

const workspaceView = {
  savedPoints: [],
  getSelectedPoints() {
    return [
      { x: 10, y: 20 },
      { x: 110, y: 20 },
      { x: 110, y: 120 },
      { x: 10, y: 120 },
    ];
  },
  getSavedWorkspacePoints() {
    return this.savedPoints;
  },
  setExecutionOverlayMessage() {},
};

const rosConnection = {
  getResources() {
    return {
      workspaceQuadPublisher: {
        publish(message) {
          publishedMessages.push(message);
        },
      },
      processImageService: {},
      startPseudoSlamScanActionClient: scanActionClient,
      executionModeService: {
        callService(request, success) {
          executionModeCalls.push(request);
          success({ success: true, message: "全局执行模式已切换" });
        },
      },
      startGlobalWorkActionClient,
    };
  },
  async callProcessImageService(request) {
    processImageCalls.push(request);
    return { success: true, message: "Surface-DP 识别完成。", count: 8 };
  },
  async callSinglePointBindService() {
    singlePointBindCalls.push({});
    return { success: true, message: "单点绑扎完成。" };
  },
};

const controller = new TaskActionController({
  rosConnection,
  workspaceView,
  getBindGroupPointCount() {
    return 6;
  },
  callbacks: {
    onResultMessage: (message) => resultMessages.push(message),
    onLog: (message, level) => logs.push({ message, level }),
  },
});

controller.publishWorkspaceQuad();
assert.deepEqual(publishedMessages.at(-1).data, payload);

workspaceView.savedPoints = workspaceView.getSelectedPoints();
assert.equal(controller.handleSavedWorkspacePayload(payload), true);
await new Promise((resolve) => setTimeout(resolve, 0));

assert.deepEqual(processImageCalls, []);
assert.equal(actionGoals.length, 1);
assert.equal(actionGoals.at(-1)?.actionClient, scanActionClient);
assert.deepEqual(actionGoals.at(-1)?.goalMessage, { enable_capture_gate: false, scan_strategy: 3, bind_group_point_count: 6 });
assert.equal(actionGoals.at(-1)?.sent, true);
assert.match(resultMessages.at(-1), /Surface-DP|视觉识别/);
assert.equal(logs.some((entry) => /自动触发.*视觉识别/.test(entry.message)), true);
assert.equal(logs.some((entry) => entry.message.includes("pseudo_slam_points.json")), true);

const processImageCallCountBeforeVisionOnly = processImageCalls.length;
const singleBindCallCountBeforeVisionOnly = singlePointBindCalls.length;
const executionVisionOnlyResult = await controller.handle("executionVisionOnly");
assert.equal(executionVisionOnlyResult, true);
assert.equal(processImageCalls.length, processImageCallCountBeforeVisionOnly + 1);
assert.deepEqual(processImageCalls.at(-1), { requestMode: PROCESS_IMAGE_REQUEST_MODES.EXECUTION_REFINE });
assert.equal(singlePointBindCalls.length, singleBindCallCountBeforeVisionOnly);
assert.equal(
  resultMessages.some((message) => message.includes("不执行线性模组单点绑扎")),
  true,
);

const actionCountBeforeRemovedActions = actionGoals.length;
controller.handle("scanPlan");
controller.handle("runBindPathTest");
assert.equal(actionGoals.length, actionCountBeforeRemovedActions);
assert.equal(typeof controller.triggerPseudoSlamScan, "undefined");
assert.equal(typeof controller.triggerBindPathDirectTest, "undefined");

const defaultExecutionActionCountBefore = actionGoals.length;
controller.handle("startExecution");
assert.equal(executionModeCalls.at(-1).execution_mode, GLOBAL_EXECUTION_MODES.LEDGER_WITH_REFINE);
assert.equal(actionGoals.length, defaultExecutionActionCountBefore + 1);
assert.equal(actionGoals.at(-1)?.actionClient, startGlobalWorkActionClient);
assert.equal(actionGoals.at(-1)?.goalMessage.execution_mode, GLOBAL_EXECUTION_MODES.LEDGER_WITH_REFINE);

const pureRefineController = new TaskActionController({
  rosConnection,
  workspaceView,
  getExecutionMode() {
    return GLOBAL_EXECUTION_MODES.PLANNED_PATH_REFINE_ONLY;
  },
});
pureRefineController.handle("startExecution");
assert.equal(executionModeCalls.at(-1).execution_mode, GLOBAL_EXECUTION_MODES.PLANNED_PATH_REFINE_ONLY);
assert.equal(actionGoals.at(-1)?.goalMessage.execution_mode, GLOBAL_EXECUTION_MODES.PLANNED_PATH_REFINE_ONLY);
