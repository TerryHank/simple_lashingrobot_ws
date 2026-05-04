import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { TaskActionController } from "../src/controllers/TaskActionController.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

global.window = {
  setTimeout() {
    return 1;
  },
  clearTimeout() {},
};

const savedWorkspacePoints = [
  { x: 10, y: 20 },
  { x: 110, y: 20 },
  { x: 110, y: 120 },
  { x: 10, y: 120 },
];

const processImageCalls = [];
const callbackEvents = [];

const workspaceView = {
  getSavedWorkspacePoints() {
    return savedWorkspacePoints;
  },
  setExecutionOverlayMessage(message) {
    callbackEvents.push(["executionOverlay", message]);
  },
};

const rosConnection = {
  getResources() {
    return {
      processImageService: {},
    };
  },
  async callProcessImageService(request) {
    processImageCalls.push(request);
    return {
      success: true,
      message: "扫描模式已直接输出整个矩形画幅内、规划工作区内的相机原始坐标点",
      count: 8,
    };
  },
  async callLashingRecognizeOnceService() {
    throw new Error("前端触发视觉识别不应再走 /perception/lashing/recognize_once");
  },
};

const controller = new TaskActionController({
  rosConnection,
  workspaceView,
  callbacks: {
    onWorkspaceS2Triggered() {
      callbackEvents.push(["workspaceS2Triggered"]);
    },
    onResultMessage(message) {
      callbackEvents.push(["result", message]);
    },
    onLog(message, level) {
      callbackEvents.push(["log", message, level]);
    },
  },
});

const result = await controller.triggerSavedWorkspaceS2();

assert.equal(result, true);
assert.deepEqual(processImageCalls, [{ requestMode: 3 }]);
assert.equal(callbackEvents.some(([event]) => event === "workspaceS2Triggered"), true);
assert.equal(
  callbackEvents.some((event) => event[0] === "log" && event[1].includes("Surface-DP物理先验")),
  true,
);

const independentProcessImageCalls = [];
const independentController = new TaskActionController({
  rosConnection: {
    getResources() {
      return {
        processImageService: {},
      };
    },
    async callProcessImageService(request) {
      independentProcessImageCalls.push(request);
      return {
        success: true,
        message: "没有新提交工作区也允许触发扫描视觉识别",
        count: 0,
      };
    },
  },
  workspaceView: {
    getSavedWorkspacePoints() {
      return [];
    },
    setExecutionOverlayMessage(message) {
      callbackEvents.push(["independentExecutionOverlay", message]);
    },
  },
  callbacks: {
    onWorkspaceS2Triggered() {
      callbackEvents.push(["independentWorkspaceS2Triggered"]);
    },
    onResultMessage(message) {
      callbackEvents.push(["independentResult", message]);
    },
    onLog(message, level) {
      callbackEvents.push(["independentLog", message, level]);
    },
  },
});

assert.equal(await independentController.triggerSavedWorkspaceS2(), true);
assert.deepEqual(independentProcessImageCalls, [{ requestMode: 3 }]);

const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");
const runSavedS2Block = appText.match(/runSavedS2:\s*([\s\S]*?),\n\s*triggerSingleBind:/)?.[1] || "";
assert.doesNotMatch(runSavedS2Block, /savedPoints\.length/);

const pendingPayload = [10, 20, 110, 20, 110, 120, 10, 120];
const pendingProcessImageCalls = [];
const pendingController = new TaskActionController({
  rosConnection: {
    getResources() {
      return {
        processImageService: {},
      };
    },
    async callProcessImageService(request) {
      pendingProcessImageCalls.push(request);
      return {
        success: true,
        message: "Surface-DP 识别完成",
        count: 8,
      };
    },
  },
  workspaceView: {
    getSavedWorkspacePoints() {
      return [];
    },
    setExecutionOverlayMessage() {},
  },
  callbacks: {},
});

pendingController.setPendingWorkspaceQuadSubmission(pendingPayload);
assert.equal(await pendingController.triggerSavedWorkspaceS2(), true);
assert.equal(pendingController.handleSavedWorkspacePayload(pendingPayload), true);
await new Promise((resolve) => setTimeout(resolve, 0));
assert.deepEqual(pendingProcessImageCalls, [{ requestMode: 3 }, { requestMode: 3 }]);
