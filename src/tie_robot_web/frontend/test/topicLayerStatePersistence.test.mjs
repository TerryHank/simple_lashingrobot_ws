import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");
const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");
const storageText = readFileSync(resolve(frontendRoot, "src/utils/storage.js"), "utf-8");

assert.match(storageText, /TOPIC_LAYER_STATE_KEY/);
assert.match(storageText, /loadTopicLayerStatePreference/);
assert.match(storageText, /saveTopicLayerStatePreference/);
assert.match(appText, /loadTopicLayerStatePreference/);
assert.match(appText, /saveTopicLayerStatePreference/);
assert.match(appText, /initialState:\s*this\.topicLayerState/);
assert.match(appText, /saveTopicLayerStatePreference\(this\.topicLayerController\.getState\(\)\)/);

const localStore = new Map();
global.localStorage = {
  getItem(key) {
    return localStore.has(key) ? localStore.get(key) : null;
  },
  setItem(key, value) {
    localStore.set(key, String(value));
  },
};

const {
  loadTopicLayerStatePreference,
  saveTopicLayerStatePreference,
  TOPIC_LAYER_STATE_KEY,
} = await import("../src/utils/storage.js");

assert.equal(loadTopicLayerStatePreference().showRobot, true);

localStorage.setItem(TOPIC_LAYER_STATE_KEY, JSON.stringify({
  mode: "onlyPlanningPoints",
  pointCloudSource: "rawWorldCoord",
  showRobot: false,
  showAxes: false,
  showPointCloud: true,
  showBindPoints: false,
  showBindGridLines: false,
  showBindGroups: false,
  showCabinPath: false,
  showLinearModuleBindRange: false,
  showImageRecognitionResult: false,
  showImageScanPoints: false,
  tfAxisFrameVisibility: { map: false, base_link: true },
  pointSize: 0.06,
  pointOpacity: 0.42,
  viewMode: "top",
  followOrigin: true,
}));

const loaded = loadTopicLayerStatePreference();
assert.equal(loaded.mode, "onlyPlanningPoints");
assert.equal(loaded.pointCloudSource, "rawWorldCoord");
assert.equal(loaded.showRobot, false);
assert.equal(loaded.showBindPoints, false);
assert.equal(loaded.showCabinPath, false);
assert.equal(loaded.showLinearModuleBindRange, false);
assert.equal(loaded.showImageRecognitionResult, false);
assert.equal(loaded.showImageScanPoints, false);
assert.equal(loaded.tfAxisFrameVisibility.map, false);
assert.equal(loaded.pointSize, 0.06);
assert.equal(loaded.pointOpacity, 0.42);
assert.equal(loaded.viewMode, "top");
assert.equal(loaded.followOrigin, true);

saveTopicLayerStatePreference({ ...loaded, pointOpacity: 2 });
const saved = JSON.parse(localStorage.getItem(TOPIC_LAYER_STATE_KEY));
assert.equal(saved.showRobot, false);
assert.equal(saved.showBindGridLines, false);
assert.equal(saved.showLinearModuleBindRange, false);
assert.equal(saved.showImageRecognitionResult, false);
assert.equal(saved.showImageScanPoints, false);
assert.equal(saved.pointOpacity, 1);
