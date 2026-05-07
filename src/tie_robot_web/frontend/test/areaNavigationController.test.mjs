import assert from "node:assert/strict";

import { AreaNavigationController } from "../src/controllers/AreaNavigationController.js";

const bindPath = {
  areas: [
    { area_index: 1, cabin_pose: { x: 0, y: 0, z: 500 } },
    { area_index: 2, cabin_pose: { x: 100, y: 0, z: 510 } },
    { area_index: 3, cabin_pose: { x: 200, y: 0, z: 520 } },
  ],
};

function buildController({ currentPosition = { x: 95, y: 3, z: 500 } } = {}) {
  const calls = [];
  const messages = [];
  const rosConnection = {
    getResources() {
      return {
        cabinSingleMoveService: {},
        manualAreaTakeoverPublisher: {},
        moduanMoveZeroPublisher: {},
      };
    },
    publishManualAreaTakeover() {
      calls.push({ type: "manualTakeover" });
      return { success: true, message: "人工接管信号已发送" };
    },
    publishModuanMoveZero() {
      calls.push({ type: "moduanZero" });
      return { success: true, message: "末端回零已发送" };
    },
    async callCabinSingleMoveService(payload) {
      calls.push({ type: "cabinMove", payload });
      return { success: true, message: "索驱移动已下发" };
    },
  };
  const controller = new AreaNavigationController({
    rosConnection,
    loadBindPath: async () => bindPath,
    waitAfterLinearModuleZero: async () => {
      calls.push({ type: "zeroDelay" });
    },
    getCurrentCabinPosition: () => currentPosition,
    getCabinSpeed: () => 360,
    callbacks: {
      onResultMessage: (message) => messages.push(message),
      onLog: (message, level) => messages.push(`${level}:${message}`),
    },
  });
  return { controller, calls, messages };
}

const progressCase = buildController();
progressCase.controller.handleAreaProgressMessage({ current_area_index: 2, total_area_count: 3 });
const nextResult = await progressCase.controller.moveRelative(1);
assert.equal(nextResult.success, true);
assert.deepEqual(progressCase.calls.map((call) => call.type), [
  "manualTakeover",
  "moduanZero",
  "zeroDelay",
  "cabinMove",
]);
assert.deepEqual(progressCase.calls.at(-1).payload, { x: 200, y: 0, z: 520, speed: 360 });

const nearestCase = buildController({ currentPosition: { x: 98, y: 10, z: 500 } });
const previousResult = await nearestCase.controller.moveRelative(-1);
assert.equal(previousResult.success, true);
assert.deepEqual(nearestCase.calls.at(-1).payload, { x: 0, y: 0, z: 500, speed: 360 });

const boundaryCase = buildController({ currentPosition: { x: 2, y: 1, z: 500 } });
const boundaryResult = await boundaryCase.controller.moveRelative(-1);
assert.equal(boundaryResult.success, false);
assert.equal(boundaryCase.calls.length, 0);
assert.equal(boundaryCase.messages.some((message) => message.includes("已经是第一个区域")), true);
