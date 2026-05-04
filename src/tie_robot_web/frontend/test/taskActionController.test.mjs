import assert from "node:assert/strict";

import { CONTROL_PANEL_TASKS } from "../src/config/controlPanelCatalog.js";
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
assert.equal(submitTask?.label, "确认\n工作区域");
assert.equal(runSavedS2Task?.label, "触发\n视觉识别");

const payload = [10, 20, 110, 20, 110, 120, 10, 120];
const publishedMessages = [];
const logs = [];
const resultMessages = [];
const processImageCalls = [];
const scanActionClient = { name: "start_pseudo_slam_scan" };

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
    };
  },
  async callProcessImageService(request) {
    processImageCalls.push(request);
    return { success: true, message: "Surface-DP 识别完成。", count: 8 };
  },
};

const controller = new TaskActionController({
  rosConnection,
  workspaceView,
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
assert.deepEqual(actionGoals.at(-1)?.goalMessage, { enable_capture_gate: false, scan_strategy: 3 });
assert.equal(actionGoals.at(-1)?.sent, true);
assert.match(resultMessages.at(-1), /Surface-DP|视觉识别/);
assert.equal(logs.some((entry) => /自动触发.*视觉识别/.test(entry.message)), true);
assert.equal(logs.some((entry) => entry.message.includes("pseudo_slam_points.json")), true);

const fixedScanMessages = [];
const fixedScanController = new TaskActionController({
  rosConnection,
  workspaceView,
  getRecognitionPose() {
    return { x: 490, y: 1700, z: 3197 };
  },
  callbacks: {
    onResultMessage: (message) => fixedScanMessages.push(message),
    onLog: (message, level) => logs.push({ message, level }),
  },
});

const fixedScanActionCountBefore = actionGoals.length;
fixedScanController.triggerPseudoSlamScan();
assert.equal(actionGoals.length, fixedScanActionCountBefore + 1);
assert.deepEqual(actionGoals.at(-1)?.goalMessage, {
  enable_capture_gate: false,
  scan_strategy: 2,
  use_fixed_scan_pose_override: true,
  fixed_scan_pose_x_mm: 490,
  fixed_scan_pose_y_mm: 1700,
  fixed_scan_pose_z_mm: 3197,
});
assert.equal(fixedScanMessages.some((message) => message.includes("x=490, y=1700, z=3197")), true);
