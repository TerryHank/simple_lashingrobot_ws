import assert from "node:assert/strict";

import { CONTROL_PANEL_TASKS } from "../src/config/controlPanelCatalog.js";
import { FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE } from "../src/config/visualRecognitionMode.js";
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

const submitTask = CONTROL_PANEL_TASKS.find((task) => task.id === "submitQuad");
const runSavedS2Task = CONTROL_PANEL_TASKS.find((task) => task.id === "runSavedS2");
assert.equal(submitTask?.label, "确认\n工作区域");
assert.equal(runSavedS2Task?.label, "触发\n视觉识别");

const payload = [10, 20, 110, 20, 110, 120, 10, 120];
const publishedMessages = [];
const logs = [];
const resultMessages = [];
const processImageCalls = [];

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

assert.deepEqual(processImageCalls, [{ requestMode: FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE }]);
assert.match(resultMessages.at(-1), /Surface-DP|视觉识别/);
assert.equal(logs.some((entry) => /自动触发.*视觉识别/.test(entry.message)), true);
