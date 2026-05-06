import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

const sceneViewText = readFileSync(new URL("../src/views/Scene3DView.js", import.meta.url), "utf-8");
const appText = readFileSync(new URL("../src/app/TieRobotFrontApp.js", import.meta.url), "utf-8");
const rosControllerText = readFileSync(
  new URL("../src/controllers/RosConnectionController.js", import.meta.url),
  "utf-8",
);
const topicRegistryText = readFileSync(new URL("../src/config/topicRegistry.js", import.meta.url), "utf-8");

assert.equal(sceneViewText.includes("workspaceRangeGroup"), false);
assert.equal(sceneViewText.includes("setRealtimeWorkspaceRangeMessage"), false);
assert.equal(sceneViewText.includes("projectRealtimeWorkspaceRangeToImage"), false);
assert.equal(sceneViewText.includes("getRealtimeWorkspaceRangeMapPositions"), false);
assert.equal(appText.includes("onWorkspaceRangePoints"), false);
assert.equal(appText.includes("setRealtimeWorkspaceRangeMessage"), false);
assert.equal(rosControllerText.includes("algorithm.workspaceQuadCameraPoints"), false);
assert.equal(topicRegistryText.includes("workspaceQuadCameraPoints"), false);
