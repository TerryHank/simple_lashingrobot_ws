import assert from "node:assert/strict";

import {
  FRONTEND_SHARED_STATE_ENDPOINT,
  GB28181_SETTINGS_KEY,
  VIEWER_LAYOUT_PREFIX,
  VISUAL_DEBUG_SETTINGS_KEY,
  hydrateFrontendSharedState,
  loadGb28181Settings,
  saveGb28181Settings,
  flushFrontendSharedStateOnUnload,
  saveViewerLayout,
  loadVisualDebugSettings,
  saveVisualDebugSettings,
  syncFrontendSharedState,
} from "../src/utils/storage.js";

const localStorageState = new Map();
const fetchCalls = [];
let timeoutCallback = null;

global.localStorage = {
  getItem(key) {
    return localStorageState.has(key) ? localStorageState.get(key) : null;
  },
  setItem(key, value) {
    localStorageState.set(key, String(value));
  },
  removeItem(key) {
    localStorageState.delete(key);
  },
  clear() {
    localStorageState.clear();
  },
};

global.window = {
  setTimeout(callback) {
    timeoutCallback = callback;
    return 1;
  },
  clearTimeout() {
    timeoutCallback = null;
  },
};

global.fetch = async (url, options = {}) => {
  fetchCalls.push({ url, options });
  return {
    ok: true,
    async json() {
      return { success: true };
    },
  };
};

localStorage.clear();
localStorage.setItem(VISUAL_DEBUG_SETTINGS_KEY, JSON.stringify({ stableFrameCount: 2 }));
hydrateFrontendSharedState({
  [VISUAL_DEBUG_SETTINGS_KEY]: {
    stableFrameCount: 7,
    scanLinearCompensation: {
      enabled: true,
      mode: "translation",
      referenceZMm: 1000,
      xShiftMmPerMeter: 18,
      yShiftMmPerMeter: -6,
    },
  },
});
assert.equal(loadVisualDebugSettings().stableFrameCount, 7);
assert.equal(loadVisualDebugSettings().scanLinearCompensation.mode, "translation");
assert.equal(loadVisualDebugSettings().scanLinearCompensation.xShiftMmPerMeter, 18);

localStorage.clear();
fetchCalls.length = 0;
timeoutCallback = null;
hydrateFrontendSharedState({});
localStorage.setItem(VISUAL_DEBUG_SETTINGS_KEY, JSON.stringify({ stableFrameCount: 6 }));
assert.equal(loadVisualDebugSettings().stableFrameCount, 6);
assert.equal(typeof timeoutCallback, "function");
await timeoutCallback();
assert.equal(fetchCalls.length, 1);
assert.equal(
  JSON.parse(fetchCalls[0].options.body).state[VISUAL_DEBUG_SETTINGS_KEY].stableFrameCount,
  6,
);

localStorage.clear();
fetchCalls.length = 0;
timeoutCallback = null;
hydrateFrontendSharedState({});
saveVisualDebugSettings({
  stableFrameCount: 4,
  scanLinearCompensation: {
    enabled: true,
    mode: "optical_axis",
    referenceZMm: 1000,
    xPercentPerMeter: 1.5,
    yPercentPerMeter: -0.5,
  },
});
assert.equal(
  JSON.parse(localStorage.getItem(VISUAL_DEBUG_SETTINGS_KEY)).stableFrameCount,
  4,
);

await syncFrontendSharedState();
assert.equal(fetchCalls.length, 1);
assert.equal(fetchCalls[0].url, FRONTEND_SHARED_STATE_ENDPOINT);
assert.equal(fetchCalls[0].options.method, "POST");
const postedBody = JSON.parse(fetchCalls[0].options.body);
assert.equal(postedBody.version, 1);
assert.equal(postedBody.state[VISUAL_DEBUG_SETTINGS_KEY].stableFrameCount, 4);
assert.equal(
  postedBody.state[VISUAL_DEBUG_SETTINGS_KEY].scanLinearCompensation.mode,
  "optical_axis",
);

localStorage.clear();
fetchCalls.length = 0;
timeoutCallback = null;
hydrateFrontendSharedState({
  [VISUAL_DEBUG_SETTINGS_KEY]: { stableFrameCount: 8 },
  [`${VIEWER_LAYOUT_PREFIX}executionDebug`]: {
    panels: {
      settingsPanel: {
        rect: { left: 40, top: 88, width: 340, height: 610 },
      },
    },
  },
});
saveViewerLayout("executionDebug", {
  panels: {
    settingsPanel: {
      rect: { left: 520, top: 96, width: 380, height: 640 },
    },
  },
});
await syncFrontendSharedState();
assert.equal(fetchCalls.length, 1);
const layoutPatchBody = JSON.parse(fetchCalls[0].options.body);
assert.equal(layoutPatchBody.patch, true);
assert.deepEqual(Object.keys(layoutPatchBody.state), [`${VIEWER_LAYOUT_PREFIX}executionDebug`]);
assert.equal(
  layoutPatchBody.state[`${VIEWER_LAYOUT_PREFIX}executionDebug`].panels.settingsPanel.rect.left,
  520,
);

localStorage.clear();
fetchCalls.length = 0;
timeoutCallback = null;
hydrateFrontendSharedState({
  [GB28181_SETTINGS_KEY]: {
    remoteIp: "10.0.0.8",
    remotePort: 15060,
    serverId: "34020000002000000001",
    domain: "3402000000",
    password: "secret",
    localIp: "10.0.0.20",
    localPort: 5061,
    deviceId: "34020000001320000002",
  },
});
assert.deepEqual(loadGb28181Settings(), {
  remoteIp: "10.0.0.8",
  remotePort: 15060,
  serverId: "34020000002000000001",
  domain: "3402000000",
  password: "secret",
  localIp: "10.0.0.20",
  localPort: 5061,
  deviceId: "34020000001320000002",
});
saveGb28181Settings({ remoteIp: "10.0.0.9", remotePort: 5060 });
await syncFrontendSharedState();
const gb28181PatchBody = JSON.parse(fetchCalls.at(-1).options.body);
assert.equal(gb28181PatchBody.patch, true);
assert.deepEqual(Object.keys(gb28181PatchBody.state), [GB28181_SETTINGS_KEY]);
assert.equal(gb28181PatchBody.state[GB28181_SETTINGS_KEY].remoteIp, "10.0.0.9");

localStorage.clear();
fetchCalls.length = 0;
timeoutCallback = null;
hydrateFrontendSharedState({
  [VISUAL_DEBUG_SETTINGS_KEY]: { stableFrameCount: 11 },
});
assert.equal(flushFrontendSharedStateOnUnload(), true);
assert.equal(
  fetchCalls.length,
  0,
  "未修改当前页面时，关闭页面不应整包回写旧共享状态",
);
