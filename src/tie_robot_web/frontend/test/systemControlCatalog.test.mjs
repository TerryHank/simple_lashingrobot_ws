import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { getSystemControlAction } from "../src/config/systemControlCatalog.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const uiControllerText = readFileSync(
  resolve(__dirname, "../src/ui/UIController.js"),
  "utf-8",
);

function actionSteps(actionId) {
  const action = getSystemControlAction(actionId);
  assert.ok(action, `missing system control action: ${actionId}`);
  return action.steps || [];
}

assert.match(uiControllerText, /chassis: level === "success" \? "stopCabinSubsystem" : "startCabinSubsystem"/);
assert.match(uiControllerText, /chassis: level === "success" \? "关闭" : "启动"/);

assert.deepEqual(actionSteps("startCabinSubsystem"), ["startCabinDriver"]);
assert.deepEqual(actionSteps("restartCabinSubsystem"), ["restartCabinDriver"]);
assert.deepEqual(actionSteps("stopCabinSubsystem"), ["stopCabinDriver"]);
assert.deepEqual(actionSteps("startModuanSubsystem"), ["startModuanDriver"]);
assert.deepEqual(actionSteps("stopModuanSubsystem"), ["stopModuanDriver"]);
assert.deepEqual(actionSteps("restartModuanSubsystem"), ["restartModuanDriver"]);
assert.deepEqual(actionSteps("startVisualSubsystem"), ["startCameraDriver", "startAlgorithmStack"]);
assert.deepEqual(actionSteps("stopVisualSubsystem"), ["stopAlgorithmStack", "stopCameraDriver"]);
assert.deepEqual(actionSteps("restartVisualSubsystem"), ["restartCameraDriver", "restartAlgorithmStack"]);

for (const actionId of [
  "startCabinSubsystem",
  "restartCabinSubsystem",
  "stopCabinSubsystem",
  "startModuanSubsystem",
  "stopModuanSubsystem",
  "restartModuanSubsystem",
]) {
  assert.ok(
    !actionSteps(actionId).some((stepId) => stepId.includes("AlgorithmStack")),
    `${actionId} must not control the visual algorithm layer`,
  );
}
