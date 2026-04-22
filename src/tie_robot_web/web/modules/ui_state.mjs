import { t as ROSLIBFactory } from "../vendor/roslib-CFvqKDwv.js";
import {
  clearBtn,
  connectionEl,
  displayModeEl,
  gammaRangeEl,
  gammaValueEl,
  overlayOpacityRangeEl,
  overlayOpacityValueEl,
  runDirectBindPathTestBtn,
  runSavedS2Btn,
  startExecutionBtn,
  startExecutionClearMemoryBtn,
  statusEl,
  submitBtn,
  undoBtn,
  moveToWorkspaceCenterScanPoseBtn,
  s2OverlayCanvas,
} from "./dom_refs.mjs";

export const CONNECTION_PREFERENCES_KEY = "ros_web_gui_connection_preferences";
export const DISPLAY_PREFERENCES_KEY = "ir_workspace_picker_display_preferences";
export const IR_TOPIC = "/Scepter/ir/image_raw";
export const WORKSPACE_QUAD_TOPIC = "/web/pointAI/set_workspace_quad";
export const RUN_WORKSPACE_S2_TOPIC = "/web/pointAI/run_workspace_s2";
export const START_PSEUDO_SLAM_SCAN_ACTION = "/web/cabin/start_pseudo_slam_scan";
export const START_GLOBAL_WORK_ACTION = "/web/cabin/start_global_work";
export const RUN_DIRECT_BIND_PATH_TEST_ACTION = "/web/cabin/run_bind_path_direct_test";
export const SET_EXECUTION_MODE_SERVICE = "/cabin/set_execution_mode";
export const START_PSEUDO_SLAM_SCAN_ACTION_TYPE = "tie_robot_msgs/StartPseudoSlamScanTaskAction";
export const START_GLOBAL_WORK_ACTION_TYPE = "tie_robot_msgs/StartGlobalWorkTaskAction";
export const RUN_DIRECT_BIND_PATH_TEST_ACTION_TYPE = "tie_robot_msgs/RunBindPathDirectTestTaskAction";
export const SET_EXECUTION_MODE_SERVICE_TYPE = "tie_robot_msgs/SetExecutionMode";
export const CURRENT_WORKSPACE_QUAD_TOPIC = "/pointAI/manual_workspace_quad_pixels";
export const WORKSPACE_S2_RESULT_TOPIC = "/pointAI/manual_workspace_s2_result_raw";
export const EXECUTION_RESULT_TOPIC = "/pointAI/result_image_raw";
export const ROSLIB = typeof ROSLIBFactory === "function" ? ROSLIBFactory() : ROSLIBFactory;

export function loadSavedConnectionPreferences() {
  try {
    const raw = localStorage.getItem(CONNECTION_PREFERENCES_KEY);
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
}

export function loadDisplayPreferences() {
  const defaults = { mode: "auto", gamma: 0.85, overlayOpacity: 0.88 };
  try {
    const raw = localStorage.getItem(DISPLAY_PREFERENCES_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      mode: ["raw", "auto", "strong"].includes(parsed?.mode) ? parsed.mode : defaults.mode,
      gamma: Number.isFinite(parsed?.gamma) ? parsed.gamma : defaults.gamma,
      overlayOpacity: Number.isFinite(parsed?.overlayOpacity) ? parsed.overlayOpacity : defaults.overlayOpacity,
    };
  } catch {
    return defaults;
  }
}

export const state = {
  ros: null,
  irSubscriber: null,
  quadPublisher: null,
  runS2Publisher: null,
  pseudoSlamScanActionClient: null,
  executionModeService: null,
  startGlobalWorkActionClient: null,
  runDirectBindPathTestActionClient: null,
  currentQuadSubscriber: null,
  s2ResultSubscriber: null,
  executionResultSubscriber: null,
  lastImageMessage: null,
  lastResultImageMessage: null,
  lastExecutionResultImageMessage: null,
  selectedPoints: [],
  savedWorkspacePoints: [],
  displaySettings: loadDisplayPreferences(),
  overlaySource: "s2",
  dragState: {
    activeIndex: -1,
    pointerId: null,
    moved: false,
  },
  suppressNextCanvasClick: false,
};

export function saveDisplayPreferences() {
  try {
    localStorage.setItem(DISPLAY_PREFERENCES_KEY, JSON.stringify(state.displaySettings));
  } catch {
    // ignore storage failures
  }
}

export function setStatus(message, level = "info") {
  statusEl.textContent = message;
  statusEl.dataset.level = level;
}

export function updateActionButtons() {
  submitBtn.disabled = state.selectedPoints.length !== 4 || !state.quadPublisher || !state.runS2Publisher;
  runSavedS2Btn.disabled = state.savedWorkspacePoints.length !== 4 || !state.runS2Publisher;
  moveToWorkspaceCenterScanPoseBtn.disabled = !state.pseudoSlamScanActionClient;
  startExecutionBtn.disabled = !state.executionModeService || !state.startGlobalWorkActionClient;
  startExecutionClearMemoryBtn.disabled = !state.executionModeService || !state.startGlobalWorkActionClient;
  runDirectBindPathTestBtn.disabled = !state.runDirectBindPathTestActionClient;
  undoBtn.disabled = state.selectedPoints.length === 0;
  clearBtn.disabled = state.selectedPoints.length === 0;
}

export function syncDisplayControls() {
  if (displayModeEl) {
    displayModeEl.value = state.displaySettings.mode;
  }
  if (gammaRangeEl) {
    gammaRangeEl.value = state.displaySettings.gamma.toFixed(2);
  }
  if (gammaValueEl) {
    gammaValueEl.textContent = state.displaySettings.gamma.toFixed(2);
  }
  if (overlayOpacityRangeEl) {
    overlayOpacityRangeEl.value = state.displaySettings.overlayOpacity.toFixed(2);
  }
  if (overlayOpacityValueEl) {
    overlayOpacityValueEl.textContent = state.displaySettings.overlayOpacity.toFixed(2);
  }
}

export function applyOverlayOpacity() {
  if (!s2OverlayCanvas) {
    return;
  }
  s2OverlayCanvas.style.opacity = String(state.displaySettings.overlayOpacity);
}

export function updateConnectionText(value) {
  connectionEl.textContent = value;
}
