import assert from "node:assert/strict";

import { CabinRemoteController } from "../src/controllers/CabinRemoteController.js";

const serviceCalls = [];
const relativeServiceCalls = [];
const controller = new CabinRemoteController({
  rosConnection: {
    async callCabinSingleMoveService(target) {
      serviceCalls.push(target);
      return { success: true, message: "ok" };
    },
    async callCabinIncrementalMoveService(target) {
      relativeServiceCalls.push(target);
      return { success: true, message: "ok" };
    },
  },
  sceneView: {
    getCurrentCabinPositionMm() {
      return { x: -600, y: 120, z: 500 };
    },
  },
});

controller.setLastKnownRawCabinPosition({
  x: 600,
  y: 120,
  z: 500,
});

const result = await controller.move("xPositive", { step: 50, speed: 300 });

assert.equal(result.success, true);
assert.deepEqual(serviceCalls, [
  {
    x: 650,
    y: 120,
    z: 500,
    speed: 300,
  },
]);
assert.deepEqual(relativeServiceCalls, []);
assert.equal(result.mode, "absolute");
assert.deepEqual(result.delta, { x: 50, y: 0, z: 0 });

const relativeResult = await controller.move("xNegative", {
  step: 25,
  speed: 200,
  moveMode: "relative",
});

assert.equal(relativeResult.success, true);
assert.deepEqual(relativeServiceCalls, [
  {
    x: -25,
    y: 0,
    z: 0,
    speed: 200,
  },
]);
assert.equal(relativeResult.mode, "relative");

const fallbackCalls = [];
const controllerWithoutRawState = new CabinRemoteController({
  rosConnection: {
    async callCabinSingleMoveService(target) {
      fallbackCalls.push(target);
      return { success: true, message: "unexpected" };
    },
    async callCabinIncrementalMoveService(target) {
      fallbackCalls.push(target);
      return { success: true, message: "unexpected" };
    },
  },
  sceneView: {
    getCurrentCabinPositionMm() {
      return { x: -600, y: 120, z: 500 };
    },
  },
});

const blockedResult = await controllerWithoutRawState.move("xPositive", { step: 50, speed: 300 });

assert.equal(blockedResult.success, false);
assert.equal(blockedResult.message, "绝对点动需要先收到索驱当前原始坐标。");
assert.deepEqual(fallbackCalls, []);
