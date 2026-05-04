import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { TaskActionController } from "../src/controllers/TaskActionController.js";
import { ROSLIB } from "../src/vendor/roslib.js";

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

const scanActionClient = { name: "start_pseudo_slam_scan" };
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
      startPseudoSlamScanActionClient: scanActionClient,
    };
  },
  async callProcessImageService(request) {
    processImageCalls.push(request);
    throw new Error("触发视觉识别不应直接调用 /pointAI/process_image");
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
assert.deepEqual(processImageCalls, []);
assert.deepEqual(actionGoals.at(-1)?.goalMessage, { enable_capture_gate: false, scan_strategy: 3 });
assert.equal(actionGoals.at(-1)?.actionClient, scanActionClient);
assert.equal(actionGoals.at(-1)?.sent, true);
assert.equal(callbackEvents.some(([event]) => event === "workspaceS2Triggered"), true);
assert.equal(
  callbackEvents.some((event) => event[0] === "log" && event[1].includes("pseudo_slam_points.json")),
  true,
);

const independentActionClient = { name: "independent_start_pseudo_slam_scan" };
const independentController = new TaskActionController({
  rosConnection: {
    getResources() {
      return {
        startPseudoSlamScanActionClient: independentActionClient,
      };
    },
    async callProcessImageService(request) {
      processImageCalls.push(request);
      throw new Error("没有新提交工作区时也应走扫描 action 覆盖本地绑扎点文件");
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

const independentActionCountBefore = actionGoals.length;
assert.equal(await independentController.triggerSavedWorkspaceS2(), true);
assert.equal(actionGoals.length, independentActionCountBefore + 1);
assert.equal(actionGoals.at(-1)?.actionClient, independentActionClient);
assert.deepEqual(actionGoals.at(-1)?.goalMessage, { enable_capture_gate: false, scan_strategy: 3 });
assert.deepEqual(processImageCalls, []);

const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");
const runSavedS2Block = appText.match(/runSavedS2:\s*([\s\S]*?),\n\s*triggerSingleBind:/)?.[1] || "";
assert.doesNotMatch(runSavedS2Block, /savedPoints\.length/);
assert.match(runSavedS2Block, /startPseudoSlamScanActionClient/);
assert.doesNotMatch(runSavedS2Block, /processImageService/);

const pendingPayload = [10, 20, 110, 20, 110, 120, 10, 120];
const pendingActionClient = { name: "pending_start_pseudo_slam_scan" };
const pendingController = new TaskActionController({
  rosConnection: {
    getResources() {
      return {
        startPseudoSlamScanActionClient: pendingActionClient,
      };
    },
    async callProcessImageService(request) {
      processImageCalls.push(request);
      throw new Error("工作区保存确认后的自动视觉识别也应走扫描 action");
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
const pendingActionCountBefore = actionGoals.length;
assert.equal(await pendingController.triggerSavedWorkspaceS2(), true);
assert.equal(pendingController.handleSavedWorkspacePayload(pendingPayload), true);
await new Promise((resolve) => setTimeout(resolve, 0));
assert.equal(actionGoals.length, pendingActionCountBefore + 2);
assert.deepEqual(
  actionGoals.slice(-2).map((goal) => goal.goalMessage),
  [
    { enable_capture_gate: false, scan_strategy: 3 },
    { enable_capture_gate: false, scan_strategy: 3 },
  ],
);
assert.deepEqual(processImageCalls, []);
