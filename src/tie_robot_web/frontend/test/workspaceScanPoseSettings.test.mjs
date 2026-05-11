import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import {
  RECOGNITION_POSE_KEY,
  loadRecognitionPose,
  loadRecognitionPoseLibrary,
  saveRecognitionPose,
  saveRecognitionPoseLibrary,
} from "../src/utils/storage.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

const localStorageState = new Map();
global.localStorage = {
  getItem(key) {
    return localStorageState.has(key) ? localStorageState.get(key) : null;
  },
  setItem(key, value) {
    localStorageState.set(key, String(value));
  },
  clear() {
    localStorageState.clear();
  },
};

localStorage.clear();
saveRecognitionPoseLibrary({
  selectedId: "pose-2",
  poses: [
    { id: "pose-1", label: "识别位姿 1", x: 100, y: 200, z: 300 },
    { id: "pose-2", label: "识别位姿 2", x: 400, y: 500, z: 600 },
  ],
});
assert.deepEqual(loadRecognitionPoseLibrary().poses.map((pose) => pose.label), ["识别位姿 1", "识别位姿 2"]);
assert.equal(loadRecognitionPoseLibrary().selectedId, "pose-2");
assert.deepEqual(loadRecognitionPose(), { x: 400, y: 500, z: 600 });

localStorage.clear();
saveRecognitionPose({ x: 11, y: 22, z: 33 });
assert.deepEqual(loadRecognitionPoseLibrary(), {
  selectedId: "pose-1",
  poses: [{ id: "pose-1", label: "识别位姿 1", x: 11, y: 22, z: 33 }],
});

localStorage.clear();
localStorage.setItem(RECOGNITION_POSE_KEY, JSON.stringify({ x: 7, y: 8, z: 9 }));
assert.deepEqual(loadRecognitionPoseLibrary().poses.at(0), {
  id: "pose-1",
  label: "识别位姿 1",
  x: 7,
  y: 8,
  z: 9,
});

const controlPanelCatalogText = readFileSync(
  resolve(frontendRoot, "src/config/controlPanelCatalog.js"),
  "utf-8",
);
assert.match(controlPanelCatalogText, /title: "扫描区"/);
assert.match(controlPanelCatalogText, /id: "runSavedS2"/);
assert.match(controlPanelCatalogText, /触发扫描\\n视觉/);
for (const scanActionId of ["moveToPosition", "setRecognitionPose", "submitQuad"]) {
  assert.doesNotMatch(controlPanelCatalogText, new RegExp(`id: "${scanActionId}"`));
}

const uiControllerText = readFileSync(resolve(frontendRoot, "src/ui/UIController.js"), "utf-8");
const workspacePageStart = uiControllerText.indexOf('<section class="settings-page is-active" data-settings-page="workspace">');
const workspacePageEnd = uiControllerText.indexOf('<section class="settings-page" data-settings-page="scene" hidden>');
assert.notEqual(workspacePageStart, -1);
assert.notEqual(workspacePageEnd, -1);
const workspacePageMarkup = uiControllerText.slice(workspacePageStart, workspacePageEnd);

assert.match(workspacePageMarkup, /id="recognitionPoseSelect"/);
assert.match(workspacePageMarkup, /data-workspace-scan-action="addRecognitionPose"/);
assert.match(workspacePageMarkup, /data-workspace-scan-action="deleteRecognitionPose"/);
assert.match(workspacePageMarkup, /data-workspace-scan-action="setRecognitionPose"/);
assert.match(workspacePageMarkup, /data-workspace-scan-action="moveToPosition"/);
assert.match(workspacePageMarkup, /data-workspace-scan-action="submitQuad"/);
assert.doesNotMatch(workspacePageMarkup, /data-workspace-scan-action="runSavedS2"/);
assert.match(workspacePageMarkup, /记录当前位姿/);
assert.match(workspacePageMarkup, /删除位姿/);
assert.match(workspacePageMarkup, /移动到选中位姿/);
assert.match(workspacePageMarkup, /确认工作区域/);
assert.doesNotMatch(workspacePageMarkup, /扫描动作/);
assert.doesNotMatch(workspacePageMarkup, /<div class="section-title">工作区选点<\/div>/);
assert.doesNotMatch(workspacePageMarkup, /触发扫描视觉/);
assert.match(workspacePageMarkup, /id="selectedPoints"/);
assert.match(workspacePageMarkup, /data-workspace-action="undo"/);
assert.match(workspacePageMarkup, /data-workspace-action="clear"/);

const moveButtonIndex = workspacePageMarkup.indexOf('data-workspace-scan-action="moveToPosition"');
const submitButtonIndex = workspacePageMarkup.indexOf('data-workspace-scan-action="submitQuad"');
assert.ok(moveButtonIndex >= 0);
assert.ok(submitButtonIndex > moveButtonIndex);

assert.match(uiControllerText, /onWorkspaceScanAction\(callback\)/);
assert.match(uiControllerText, /setRecognitionPoseLibrary/);
assert.match(uiControllerText, /setWorkspaceScanButtonsEnabled/);

const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");
assert.match(appText, /loadRecognitionPoseLibrary/);
assert.match(appText, /saveRecognitionPoseLibrary/);
assert.match(appText, /onWorkspaceScanAction/);
assert.match(appText, /handleAddRecognitionPose/);
assert.match(appText, /handleDeleteRecognitionPose/);
assert.match(appText, /handleSetRecognitionPose/);
assert.match(appText, /handleMoveToPosition/);
assert.match(appText, /this\.taskActionController\.handle\(workspaceAction\)/);
assert.match(appText, /workspaceAction === "runSavedS2"/);
assert.match(appText, /getSelectedRecognitionPose\(\)/);
assert.match(appText, /findIndex\(\(pose\) => pose\.id === selectedPose\?\.id\)/);
assert.match(appText, /selectedId: pose\.id/);
assert.match(appText, /deleteRecognitionPose:\s*this\.recognitionPoseLibrary\.poses\.length > 1/);
