import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import {
  DEFAULT_TOPIC_LAYER_STATE,
  MODE_PRESETS,
} from "../src/config/topicLayerCatalog.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");
const uiControllerText = readFileSync(resolve(frontendRoot, "src/ui/UIController.js"), "utf-8");
const scene3dViewText = readFileSync(resolve(frontendRoot, "src/views/Scene3DView.js"), "utf-8");

assert.equal(DEFAULT_TOPIC_LAYER_STATE.showBindPoints, true);
assert.equal(DEFAULT_TOPIC_LAYER_STATE.showBindGridLines, true);
assert.equal(DEFAULT_TOPIC_LAYER_STATE.showBindGroups, true);
assert.equal(DEFAULT_TOPIC_LAYER_STATE.showCabinPath, true);
assert.equal(DEFAULT_TOPIC_LAYER_STATE.showLinearModuleBindRange, true);
assert.equal(MODE_PRESETS.onlyPlanningPoints.showBindPoints, true);
assert.equal(MODE_PRESETS.onlyPlanningPoints.showBindGroups, true);
assert.equal(MODE_PRESETS.onlyPlanningPoints.showCabinPath, true);
assert.equal(MODE_PRESETS.onlyPointCloud.showLinearModuleBindRange, false);
assert.equal(MODE_PRESETS.machineOnly.showLinearModuleBindRange, true);

assert.match(uiControllerText, /id="showBindPointsToggle"/);
assert.match(uiControllerText, /id="showBindGridLinesToggle"/);
assert.match(uiControllerText, /id="showBindGroupsToggle"/);
assert.match(uiControllerText, /id="showCabinPathToggle"/);
assert.match(uiControllerText, /id="showLinearModuleBindRangeToggle"/);
assert.match(uiControllerText, /showBindPoints:\s*this\.refs\.showBindPointsToggle\.checked/);
assert.match(uiControllerText, /showBindGroups:\s*this\.refs\.showBindGroupsToggle\.checked/);
assert.match(uiControllerText, /showCabinPath:\s*this\.refs\.showCabinPathToggle\.checked/);
assert.match(uiControllerText, /showLinearModuleBindRange:\s*this\.refs\.showLinearModuleBindRangeToggle\.checked/);

assert.match(scene3dViewText, /this\.bindPathPoints = buildPointsObject/);
assert.match(scene3dViewText, /this\.unplannedBindPathPoints = buildPointsObject/);
assert.match(scene3dViewText, /this\.bindRowLines = new THREE\.LineSegments/);
assert.match(scene3dViewText, /this\.bindColumnLines = new THREE\.LineSegments/);
assert.match(scene3dViewText, /this\.bindGroupLines = new THREE\.LineSegments/);
assert.match(scene3dViewText, /state\.showLinearModuleBindRange/);
assert.match(scene3dViewText, /this\.planningAreaPath\.visible = showCabinPath/);
assert.match(scene3dViewText, /this\.bindGroupLines\.visible = showBindGroups/);
assert.match(scene3dViewText, /this\.planningPoints\.visible = showBindPoints && !hasBindPathPointOverlay/);
assert.match(scene3dViewText, /this\.bindPathPoints\.visible = showBindPoints && hasBindPathPointOverlay/);
assert.match(scene3dViewText, /this\.unplannedBindPathPoints\.visible = showBindPoints && hasUnplannedBindPathPointOverlay/);
