import {
  DEFAULT_GLOBAL_EXECUTION_MODE,
  FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE,
  GLOBAL_EXECUTION_MODE_OPTIONS,
  DEFAULT_SCAN_RESPONSE_SOURCE,
  normalizeScanResponseSource,
} from "../config/visualRecognitionMode.js";
import { normalizeCameraSdkSettings } from "../config/cameraSdkDynamicReconfigure.js";
import { normalizeTcpWorkspaceBoundaryMm } from "./tcpWorkspaceOverlay.js";
import {
  DEFAULT_TOPIC_LAYER_STATE,
  IMAGE_HOVER_COORDINATE_FRAMES,
  POINT_CLOUD_SOURCES,
  SCENE_VIEW_MODES,
  TF_AXIS_FRAMES,
  TOPIC_LAYER_MODES,
} from "../config/topicLayerCatalog.js";

export const DISPLAY_PREFERENCES_KEY = "tie_robot_frontend_display_preferences";
export const VIEWER_LAYOUT_PREFIX = "tie_robot_frontend_layout_";
export const THEME_PREFERENCE_KEY = "tie_robot_frontend_theme";
export const SETTINGS_HOME_PAGE_KEY = "tie_robot_frontend_settings_home_page";
export const SETTINGS_PAGE_ORDER_KEY = "tie_robot_frontend_settings_page_order";
export const CABIN_REMOTE_SETTINGS_KEY = "tie_robot_frontend_cabin_remote_settings";
export const TCP_LINEAR_REMOTE_SETTINGS_KEY = "tie_robot_frontend_tcp_linear_remote_settings";
export const NETWORK_PING_SETTINGS_KEY = "tie_robot_frontend_network_ping_settings";
export const GB28181_SETTINGS_KEY = "tie_robot_frontend_gb28181_settings";
export const RECOGNITION_POSE_KEY = "tie_robot_frontend_recognition_pose";
export const VISUAL_DEBUG_SETTINGS_KEY = "tie_robot_frontend_visual_debug_settings";
export const CAMERA_SDK_SETTINGS_KEY = "tie_robot_frontend_camera_sdk_settings";
export const TOPIC_LAYER_STATE_KEY = "tie_robot_frontend_topic_layer_state";
export const FRONTEND_SHARED_STATE_ENDPOINT = "/api/frontend/state";
export const FRONTEND_SHARED_STATE_VERSION = 1;
export const DEFAULT_BIND_EXECUTION_CABIN_MIN_Z_MM = 485;
export const DEFAULT_BIND_EXECUTION_CABIN_Z_MODE = "fixed";
export const DEFAULT_SCAN_BEAM_EXCLUSION_MARGIN_MM = 150;
export const DEFAULT_BIND_GROUP_ROW_THRESHOLD_MM = 40;
export const DEFAULT_BIND_GROUP_COLUMN_THRESHOLD_MM = 45;
export const DEFAULT_SCAN_LINEAR_COMPENSATION = Object.freeze({
  enabled: false,
  mode: "optical_axis",
  referenceZMm: 1000,
  xPercentPerMeter: 0,
  yPercentPerMeter: 0,
  xShiftMmPerMeter: 0,
  yShiftMmPerMeter: 0,
  minZMm: 1200,
  maxScaleDelta: 0.25,
});
const DEFAULT_RECOGNITION_POSE_ID = "pose-1";
const frontendSharedState = new Map();
const frontendSharedStateDirtyKeys = new Set();
let frontendSharedStateHydrated = false;
let frontendSharedStateSaveTimer = null;
let frontendSharedStateSaveInFlight = false;
let frontendSharedStateSavePending = false;
const FRONTEND_SHARED_STATE_SAVE_DELAY_MS = 180;

function readLocalStorageRaw(key) {
  try {
    return localStorage.getItem(key);
  } catch {
    return null;
  }
}

function writeLocalStorageRaw(key, value) {
  try {
    localStorage.setItem(key, value);
  } catch {
    // ignore storage failures
  }
}

function readStoredRawValue(key) {
  if (frontendSharedStateHydrated && frontendSharedState.has(key)) {
    const sharedValue = frontendSharedState.get(key);
    if (typeof sharedValue === "string") {
      return sharedValue;
    }
    try {
      return JSON.stringify(sharedValue);
    } catch {
      return null;
    }
  }
  const localValue = readLocalStorageRaw(key);
  if (localValue !== null) {
    if (frontendSharedStateHydrated && !frontendSharedState.has(key)) {
      try {
        frontendSharedState.set(key, JSON.parse(localValue));
      } catch {
        frontendSharedState.set(key, localValue);
      }
      frontendSharedStateDirtyKeys.add(key);
      queueFrontendSharedStateSync();
    }
    return localValue;
  }
  if (!frontendSharedState.has(key)) {
    return null;
  }
  try {
    return JSON.stringify(frontendSharedState.get(key));
  } catch {
    return null;
  }
}

function writeStoredJsonValue(key, value) {
  frontendSharedState.set(key, value);
  frontendSharedStateDirtyKeys.add(key);
  writeLocalStorageRaw(key, JSON.stringify(value));
  queueFrontendSharedStateSync();
}

function writeStoredStringValue(key, value) {
  frontendSharedState.set(key, value);
  frontendSharedStateDirtyKeys.add(key);
  writeLocalStorageRaw(key, value);
  queueFrontendSharedStateSync();
}

function normalizeFrontendSharedStateEnvelope(payload = null) {
  const state = payload?.state ?? payload;
  return state && typeof state === "object" && !Array.isArray(state) ? state : {};
}

export function hydrateFrontendSharedState(payload = null) {
  frontendSharedState.clear();
  frontendSharedStateDirtyKeys.clear();
  frontendSharedStateHydrated = true;
  const state = normalizeFrontendSharedStateEnvelope(payload);
  Object.entries(state).forEach(([key, value]) => {
    if (typeof key === "string" && key) {
      frontendSharedState.set(key, value);
    }
  });
  return Object.fromEntries(frontendSharedState);
}

export async function loadFrontendSharedState() {
  try {
    const response = await fetch(FRONTEND_SHARED_STATE_ENDPOINT, { cache: "no-store" });
    const payload = await response.json();
    if (!response.ok || payload?.success === false) {
      throw new Error(payload?.message || response.statusText || "load failed");
    }
    hydrateFrontendSharedState(payload?.state || {});
    return { success: true, state: Object.fromEntries(frontendSharedState) };
  } catch (error) {
    return {
      success: false,
      message: error?.message || String(error),
      state: Object.fromEntries(frontendSharedState),
    };
  }
}

export async function syncFrontendSharedState() {
  const dirtyKeys = [...frontendSharedStateDirtyKeys];
  const state = Object.fromEntries(
    (dirtyKeys.length ? dirtyKeys : [...frontendSharedState.keys()])
      .filter((key) => frontendSharedState.has(key))
      .map((key) => [key, frontendSharedState.get(key)]),
  );
  try {
    const response = await fetch(FRONTEND_SHARED_STATE_ENDPOINT, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Accept: "application/json",
      },
      body: JSON.stringify({
        version: FRONTEND_SHARED_STATE_VERSION,
        patch: dirtyKeys.length > 0,
        state,
      }),
    });
    const payload = await response.json().catch(() => ({}));
    if (!response.ok || payload?.success === false) {
      throw new Error(payload?.message || response.statusText || "save failed");
    }
    dirtyKeys.forEach((key) => {
      if (frontendSharedState.get(key) === state[key]) {
        frontendSharedStateDirtyKeys.delete(key);
      }
    });
    return { success: true, state };
  } catch (error) {
    return {
      success: false,
      message: error?.message || String(error),
      state,
    };
  }
}

export function flushFrontendSharedStateOnUnload() {
  if (frontendSharedStateSaveTimer !== null && typeof window !== "undefined") {
    window.clearTimeout(frontendSharedStateSaveTimer);
    frontendSharedStateSaveTimer = null;
  }
  const dirtyKeys = [...frontendSharedStateDirtyKeys];
  if (!dirtyKeys.length) {
    return true;
  }
  const state = Object.fromEntries(
    dirtyKeys
      .filter((key) => frontendSharedState.has(key))
      .map((key) => [key, frontendSharedState.get(key)]),
  );
  const body = JSON.stringify({
    version: FRONTEND_SHARED_STATE_VERSION,
    patch: dirtyKeys.length > 0,
    state,
  });
  if (
    typeof navigator !== "undefined"
    && typeof navigator.sendBeacon === "function"
  ) {
    try {
      const blob = new Blob([body], { type: "application/json" });
      if (navigator.sendBeacon(FRONTEND_SHARED_STATE_ENDPOINT, blob)) {
        return true;
      }
    } catch {
      // fall back to fetch
    }
  }
  if (typeof fetch === "function") {
    try {
      fetch(FRONTEND_SHARED_STATE_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body,
        keepalive: true,
      });
      return true;
    } catch {
      return false;
    }
  }
  return false;
}

export function queueFrontendSharedStateSync() {
  frontendSharedStateSavePending = true;
  if (frontendSharedStateSaveTimer !== null && typeof window !== "undefined") {
    window.clearTimeout(frontendSharedStateSaveTimer);
  }
  const schedule = typeof window !== "undefined" && typeof window.setTimeout === "function"
    ? window.setTimeout.bind(window)
    : setTimeout;
  frontendSharedStateSaveTimer = schedule(async () => {
    frontendSharedStateSaveTimer = null;
    if (frontendSharedStateSaveInFlight) {
      queueFrontendSharedStateSync();
      return;
    }
    frontendSharedStateSavePending = false;
    frontendSharedStateSaveInFlight = true;
    await syncFrontendSharedState();
    frontendSharedStateSaveInFlight = false;
    if (frontendSharedStateSavePending) {
      queueFrontendSharedStateSync();
    }
  }, FRONTEND_SHARED_STATE_SAVE_DELAY_MS);
}

function normalizePositiveNumber(value, fallback) {
  const numericValue = Number(value);
  return Number.isFinite(numericValue) && numericValue > 0 ? numericValue : fallback;
}

function normalizeNonNegativeNumber(value, fallback) {
  const numericValue = Number(value);
  return Number.isFinite(numericValue) && numericValue >= 0 ? numericValue : fallback;
}

function normalizeFiniteNumber(value, fallback) {
  const numericValue = Number(value);
  return Number.isFinite(numericValue) ? numericValue : fallback;
}

function normalizeScanLinearCompensationMode(value, fallback = DEFAULT_SCAN_LINEAR_COMPENSATION.mode) {
  return value === "translation" || value === "optical_axis" ? value : fallback;
}

function normalizeBindExecutionCabinZMode(value, fallback = DEFAULT_BIND_EXECUTION_CABIN_Z_MODE) {
  return value === "min" || value === "fixed" ? value : fallback;
}

function normalizeScanLinearCompensation(value = null) {
  return {
    enabled: normalizeBoolean(value?.enabled, DEFAULT_SCAN_LINEAR_COMPENSATION.enabled),
    mode: normalizeScanLinearCompensationMode(value?.mode),
    referenceZMm: normalizePositiveNumber(
      value?.referenceZMm,
      DEFAULT_SCAN_LINEAR_COMPENSATION.referenceZMm,
    ),
    xPercentPerMeter: normalizeFiniteNumber(
      value?.xPercentPerMeter,
      DEFAULT_SCAN_LINEAR_COMPENSATION.xPercentPerMeter,
    ),
    yPercentPerMeter: normalizeFiniteNumber(
      value?.yPercentPerMeter,
      DEFAULT_SCAN_LINEAR_COMPENSATION.yPercentPerMeter,
    ),
    xShiftMmPerMeter: normalizeFiniteNumber(
      value?.xShiftMmPerMeter,
      DEFAULT_SCAN_LINEAR_COMPENSATION.xShiftMmPerMeter,
    ),
    yShiftMmPerMeter: normalizeFiniteNumber(
      value?.yShiftMmPerMeter,
      DEFAULT_SCAN_LINEAR_COMPENSATION.yShiftMmPerMeter,
    ),
    minZMm: normalizeNonNegativeNumber(
      value?.minZMm,
      DEFAULT_SCAN_LINEAR_COMPENSATION.minZMm,
    ),
    maxScaleDelta: normalizeNonNegativeNumber(
      value?.maxScaleDelta,
      DEFAULT_SCAN_LINEAR_COMPENSATION.maxScaleDelta,
    ),
  };
}

function normalizeGlobalExecutionMode(value, fallback = DEFAULT_GLOBAL_EXECUTION_MODE) {
  const numericValue = Number(value);
  const roundedValue = Number.isFinite(numericValue) ? Math.round(numericValue) : fallback;
  return GLOBAL_EXECUTION_MODE_OPTIONS.some((option) => option.id === roundedValue)
    ? roundedValue
    : fallback;
}

function normalizeCabinPose(value, fallback = null) {
  const pose = {
    x: Number(value?.x),
    y: Number(value?.y),
    z: Number(value?.z),
  };
  if (Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.z)) {
    return pose;
  }
  return fallback ? normalizeCabinPose(fallback, null) : null;
}

function normalizeWorkspaceQuadPayload(value) {
  if (!Array.isArray(value) || value.length !== 8) {
    return null;
  }
  const payload = value.map((item) => Math.round(Number(item)));
  return payload.every((item) => Number.isFinite(item)) ? payload : null;
}

function normalizeRecognitionPoseWorkspace(value = null) {
  const selectedPayload = normalizeWorkspaceQuadPayload(value?.selectedPayload);
  const savedPayload = normalizeWorkspaceQuadPayload(value?.savedPayload);
  if (!selectedPayload && !savedPayload) {
    return null;
  }
  return {
    selectedPayload: selectedPayload || savedPayload,
    savedPayload: savedPayload || null,
  };
}

function normalizeRecognitionPoseItem(value, index = 0, fallbackPose = null) {
  const pose = normalizeCabinPose(value, fallbackPose);
  if (!pose) {
    return null;
  }
  const fallbackId = `pose-${index + 1}`;
  const id = typeof value?.id === "string" && value.id.trim()
    ? value.id.trim()
    : fallbackId;
  const label = typeof value?.label === "string" && value.label.trim()
    ? value.label.trim()
    : `识别位姿 ${index + 1}`;
  const workspace = normalizeRecognitionPoseWorkspace(value?.workspace);
  return workspace ? { id, label, ...pose, workspace } : { id, label, ...pose };
}

function normalizeRecognitionPoseLibrary(value, defaultPose = null) {
  const sourcePoses = Array.isArray(value?.poses)
    ? value.poses
    : [value].filter(Boolean);
  const poses = [];
  const usedIds = new Set();

  sourcePoses.forEach((sourcePose, index) => {
    const normalized = normalizeRecognitionPoseItem(sourcePose, index);
    if (!normalized) {
      return;
    }
    let id = normalized.id;
    if (usedIds.has(id)) {
      id = `pose-${poses.length + 1}`;
    }
    usedIds.add(id);
    poses.push({ ...normalized, id });
  });

  if (!poses.length) {
    const fallbackPose = normalizeCabinPose(defaultPose);
    const fallbackItem = normalizeRecognitionPoseItem({
      id: DEFAULT_RECOGNITION_POSE_ID,
      label: "识别位姿 1",
      ...fallbackPose,
    });
    if (fallbackItem) {
      poses.push(fallbackItem);
    }
  }

  const selectedId = poses.some((pose) => pose.id === value?.selectedId)
    ? value.selectedId
    : poses[0]?.id || DEFAULT_RECOGNITION_POSE_ID;

  return { selectedId, poses };
}

function normalizeBoolean(value, fallback) {
  return typeof value === "boolean" ? value : fallback;
}

function normalizeOption(value, options, fallback) {
  return options.some((option) => option.id === value) ? value : fallback;
}

function normalizeUnitNumber(value, fallback) {
  const numericValue = Number(value);
  return Number.isFinite(numericValue)
    ? Math.min(1, Math.max(0, numericValue))
    : fallback;
}

function normalizeTopicLayerState(value = null) {
  const defaults = DEFAULT_TOPIC_LAYER_STATE;
  const planningFallback = value?.showPlanningMarkers;
  const tfAxisFrameVisibility = TF_AXIS_FRAMES.reduce((accumulator, frame) => {
    accumulator[frame.id] = normalizeBoolean(
      value?.tfAxisFrameVisibility?.[frame.id],
      defaults.tfAxisFrameVisibility[frame.id],
    );
    return accumulator;
  }, {});
  return {
    mode: normalizeOption(value?.mode, TOPIC_LAYER_MODES, defaults.mode),
    pointCloudSource: normalizeOption(value?.pointCloudSource, POINT_CLOUD_SOURCES, defaults.pointCloudSource),
    showRobot: normalizeBoolean(value?.showRobot, defaults.showRobot),
    showAxes: normalizeBoolean(value?.showAxes, defaults.showAxes),
    showPointCloud: normalizeBoolean(value?.showPointCloud, defaults.showPointCloud),
    showTiePoints: normalizeBoolean(value?.showTiePoints, defaults.showTiePoints),
    showBindPoints: normalizeBoolean(value?.showBindPoints, normalizeBoolean(planningFallback, defaults.showBindPoints)),
    showBindGridLines: normalizeBoolean(value?.showBindGridLines, normalizeBoolean(planningFallback, defaults.showBindGridLines)),
    showBindGroups: normalizeBoolean(value?.showBindGroups, normalizeBoolean(planningFallback, defaults.showBindGroups)),
    showCabinPath: normalizeBoolean(value?.showCabinPath, normalizeBoolean(planningFallback, defaults.showCabinPath)),
    showLinearModuleBindRange: normalizeBoolean(value?.showLinearModuleBindRange, defaults.showLinearModuleBindRange),
    showImageRecognitionResult: normalizeBoolean(value?.showImageRecognitionResult, defaults.showImageRecognitionResult),
    showImageScanPoints: normalizeBoolean(value?.showImageScanPoints, defaults.showImageScanPoints),
    tfAxisFrameVisibility,
    pointSize: normalizePositiveNumber(value?.pointSize, defaults.pointSize),
    pointOpacity: normalizeUnitNumber(value?.pointOpacity, defaults.pointOpacity),
    viewMode: normalizeOption(value?.viewMode, SCENE_VIEW_MODES, defaults.viewMode),
    followOrigin: normalizeBoolean(value?.followOrigin, defaults.followOrigin),
    imageHoverCoordinateFrame: normalizeOption(
      value?.imageHoverCoordinateFrame,
      IMAGE_HOVER_COORDINATE_FRAMES,
      defaults.imageHoverCoordinateFrame,
    ),
  };
}

export function loadDisplayPreferences() {
  const defaults = { mode: "raw", gamma: 1.0, overlayOpacity: 0.88 };
  try {
    const raw = readStoredRawValue(DISPLAY_PREFERENCES_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      mode: ["raw", "auto", "strong"].includes(parsed?.mode) ? parsed.mode : defaults.mode,
      gamma: Number.isFinite(parsed?.gamma) ? parsed.gamma : defaults.gamma,
      overlayOpacity: Number.isFinite(parsed?.overlayOpacity)
        ? parsed.overlayOpacity
        : defaults.overlayOpacity,
    };
  } catch {
    return defaults;
  }
}

export function saveDisplayPreferences(value) {
  writeStoredJsonValue(DISPLAY_PREFERENCES_KEY, value);
}

export function loadThemePreference() {
  const prefersDark = typeof window !== "undefined"
    && window.matchMedia
    && window.matchMedia("(prefers-color-scheme: dark)").matches;
  const fallback = prefersDark ? "dark" : "light";
  try {
    const raw = readStoredRawValue(THEME_PREFERENCE_KEY);
    return raw === "light" || raw === "dark" ? raw : fallback;
  } catch {
    return fallback;
  }
}

export function saveThemePreference(theme) {
  if (theme !== "light" && theme !== "dark") {
    return;
  }
  writeStoredStringValue(THEME_PREFERENCE_KEY, theme);
}

export function loadSettingsHomePagePreference() {
  try {
    const raw = readStoredRawValue(SETTINGS_HOME_PAGE_KEY);
    return typeof raw === "string" && raw ? raw : "topics";
  } catch {
    return "topics";
  }
}

export function saveSettingsHomePagePreference(pageId) {
  if (!pageId) {
    return;
  }
  writeStoredStringValue(SETTINGS_HOME_PAGE_KEY, pageId);
}

export function loadSettingsPageOrderPreference() {
  try {
    const raw = readStoredRawValue(SETTINGS_PAGE_ORDER_KEY);
    if (!raw) {
      return [];
    }
    const parsed = JSON.parse(raw);
    return Array.isArray(parsed)
      ? parsed.filter((item) => typeof item === "string" && item)
      : [];
  } catch {
    return [];
  }
}

export function saveSettingsPageOrderPreference(pageIds) {
  if (!Array.isArray(pageIds)) {
    return;
  }
  writeStoredJsonValue(SETTINGS_PAGE_ORDER_KEY, pageIds);
}

export function loadCabinRemoteSettings() {
  const defaults = { step: 50, speed: 300, moveMode: "absolute" };
  try {
    const raw = readStoredRawValue(CABIN_REMOTE_SETTINGS_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      step: normalizePositiveNumber(parsed?.step, defaults.step),
      speed: normalizePositiveNumber(parsed?.speed, defaults.speed),
      moveMode: parsed?.moveMode === "relative" ? "relative" : defaults.moveMode,
    };
  } catch {
    return defaults;
  }
}

export function saveCabinRemoteSettings(value) {
  const defaults = { step: 50, speed: 300, moveMode: "absolute" };
  const payload = {
    step: normalizePositiveNumber(value?.step, defaults.step),
    speed: normalizePositiveNumber(value?.speed, defaults.speed),
    moveMode: value?.moveMode === "relative" ? "relative" : defaults.moveMode,
  };
  writeStoredJsonValue(CABIN_REMOTE_SETTINGS_KEY, payload);
}

export function loadTcpLinearRemoteSettings() {
  const defaults = { step: 5, angleStep: 5, speed: 250 };
  try {
    const raw = readStoredRawValue(TCP_LINEAR_REMOTE_SETTINGS_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      step: normalizePositiveNumber(parsed?.step, defaults.step),
      angleStep: normalizePositiveNumber(parsed?.angleStep, defaults.angleStep),
      speed: normalizePositiveNumber(parsed?.speed, defaults.speed),
    };
  } catch {
    return defaults;
  }
}

export function saveTcpLinearRemoteSettings(value) {
  const defaults = { step: 5, angleStep: 5, speed: 250 };
  const payload = {
    step: normalizePositiveNumber(value?.step, defaults.step),
    angleStep: normalizePositiveNumber(value?.angleStep, defaults.angleStep),
    speed: normalizePositiveNumber(value?.speed, defaults.speed),
  };
  writeStoredJsonValue(TCP_LINEAR_REMOTE_SETTINGS_KEY, payload);
}

function normalizePingHost(value, fallback) {
  const normalized = String(value ?? "").trim();
  return normalized || fallback;
}

export function loadNetworkPingSettings() {
  const defaults = { cabinHost: "192.168.6.62", moduanHost: "192.168.6.167" };
  try {
    const raw = readStoredRawValue(NETWORK_PING_SETTINGS_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      cabinHost: normalizePingHost(parsed?.cabinHost, defaults.cabinHost),
      moduanHost: normalizePingHost(parsed?.moduanHost, defaults.moduanHost),
    };
  } catch {
    return defaults;
  }
}

export function saveNetworkPingSettings(value) {
  const defaults = { cabinHost: "192.168.6.62", moduanHost: "192.168.6.167" };
  const payload = {
    cabinHost: normalizePingHost(value?.cabinHost, defaults.cabinHost),
    moduanHost: normalizePingHost(value?.moduanHost, defaults.moduanHost),
  };
  writeStoredJsonValue(NETWORK_PING_SETTINGS_KEY, payload);
}

function normalizeGb28181Text(value, fallback = "") {
  const normalized = String(value ?? "").trim();
  return normalized || fallback;
}

function normalizeGb28181PortSetting(value, fallback = 5060) {
  const numericValue = Number(value);
  return Number.isInteger(numericValue) && numericValue >= 1 && numericValue <= 65535
    ? numericValue
    : fallback;
}

function normalizeGb28181Settings(value = null) {
  return {
    remoteIp: normalizeGb28181Text(value?.remoteIp),
    remotePort: normalizeGb28181PortSetting(value?.remotePort, 5060),
    serverId: normalizeGb28181Text(value?.serverId),
    domain: normalizeGb28181Text(value?.domain),
    password: normalizeGb28181Text(value?.password),
    localIp: normalizeGb28181Text(value?.localIp),
    localPort: normalizeGb28181PortSetting(value?.localPort, 5060),
    deviceId: normalizeGb28181Text(value?.deviceId, "34020000001320000001"),
  };
}

export function loadGb28181Settings() {
  try {
    const raw = readStoredRawValue(GB28181_SETTINGS_KEY);
    if (!raw) {
      return normalizeGb28181Settings();
    }
    return normalizeGb28181Settings(JSON.parse(raw));
  } catch {
    return normalizeGb28181Settings();
  }
}

export function saveGb28181Settings(value) {
  writeStoredJsonValue(GB28181_SETTINGS_KEY, normalizeGb28181Settings(value));
}

export function loadRecognitionPose(defaultPose = null) {
  const library = loadRecognitionPoseLibrary(defaultPose);
  const selectedPose = library.poses.find((pose) => pose.id === library.selectedId) || library.poses[0];
  return normalizeCabinPose(selectedPose, defaultPose);
}

export function saveRecognitionPose(value) {
  const pose = normalizeRecognitionPoseItem({
    id: DEFAULT_RECOGNITION_POSE_ID,
    label: "识别位姿 1",
    ...value,
  });
  if (!pose) {
    return;
  }
  saveRecognitionPoseLibrary({ selectedId: pose.id, poses: [pose] });
}

export function loadRecognitionPoseLibrary(defaultPose = null) {
  try {
    const raw = readStoredRawValue(RECOGNITION_POSE_KEY);
    if (!raw) {
      return normalizeRecognitionPoseLibrary(null, defaultPose);
    }
    return normalizeRecognitionPoseLibrary(JSON.parse(raw), defaultPose);
  } catch {
    return normalizeRecognitionPoseLibrary(null, defaultPose);
  }
}

export function saveRecognitionPoseLibrary(value) {
  const payload = normalizeRecognitionPoseLibrary(value);
  if (!payload.poses.length) {
    return;
  }
  writeStoredJsonValue(RECOGNITION_POSE_KEY, payload);
}

export function loadVisualDebugSettings() {
  const defaults = {
    stableFrameCount: 3,
    requestMode: FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE,
    executionMode: DEFAULT_GLOBAL_EXECUTION_MODE,
    scanResponseSource: DEFAULT_SCAN_RESPONSE_SOURCE,
    adaptiveBindGrouping: false,
    enableBeamExclusion: false,
    beamExclusionMarginMm: DEFAULT_SCAN_BEAM_EXCLUSION_MARGIN_MM,
    bindGroupRowThresholdMm: DEFAULT_BIND_GROUP_ROW_THRESHOLD_MM,
    bindGroupColumnThresholdMm: DEFAULT_BIND_GROUP_COLUMN_THRESHOLD_MM,
    scanLinearCompensation: normalizeScanLinearCompensation(),
    bindExecutionCabinMinZMm: DEFAULT_BIND_EXECUTION_CABIN_MIN_Z_MM,
    bindExecutionCabinZMode: DEFAULT_BIND_EXECUTION_CABIN_Z_MODE,
    linearModuleBindRangeMm: normalizeTcpWorkspaceBoundaryMm(),
  };
  try {
    const raw = readStoredRawValue(VISUAL_DEBUG_SETTINGS_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      stableFrameCount: Math.max(1, Math.round(normalizePositiveNumber(parsed?.stableFrameCount, defaults.stableFrameCount))),
      requestMode: defaults.requestMode,
      executionMode: normalizeGlobalExecutionMode(parsed?.executionMode, defaults.executionMode),
      scanResponseSource: normalizeScanResponseSource(parsed?.scanResponseSource, defaults.scanResponseSource),
      adaptiveBindGrouping: normalizeBoolean(parsed?.adaptiveBindGrouping, defaults.adaptiveBindGrouping),
      enableBeamExclusion: normalizeBoolean(parsed?.enableBeamExclusion, defaults.enableBeamExclusion),
      beamExclusionMarginMm: normalizePositiveNumber(
        parsed?.beamExclusionMarginMm,
        defaults.beamExclusionMarginMm,
      ),
      bindGroupRowThresholdMm: normalizePositiveNumber(
        parsed?.bindGroupRowThresholdMm,
        defaults.bindGroupRowThresholdMm,
      ),
      bindGroupColumnThresholdMm: normalizePositiveNumber(
        parsed?.bindGroupColumnThresholdMm,
        defaults.bindGroupColumnThresholdMm,
      ),
      scanLinearCompensation: normalizeScanLinearCompensation(parsed?.scanLinearCompensation),
      bindExecutionCabinMinZMm: normalizeNonNegativeNumber(
        parsed?.bindExecutionCabinMinZMm,
        defaults.bindExecutionCabinMinZMm,
      ),
      bindExecutionCabinZMode: normalizeBindExecutionCabinZMode(
        parsed?.bindExecutionCabinZMode,
        defaults.bindExecutionCabinZMode,
      ),
      linearModuleBindRangeMm: normalizeTcpWorkspaceBoundaryMm(parsed?.linearModuleBindRangeMm, defaults.linearModuleBindRangeMm),
    };
  } catch {
    return defaults;
  }
}

export function saveVisualDebugSettings(value) {
  const payload = {
    stableFrameCount: Math.max(1, Math.round(normalizePositiveNumber(value?.stableFrameCount, 3))),
    requestMode: FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE,
    executionMode: normalizeGlobalExecutionMode(value?.executionMode),
    scanResponseSource: normalizeScanResponseSource(value?.scanResponseSource, DEFAULT_SCAN_RESPONSE_SOURCE),
    adaptiveBindGrouping: normalizeBoolean(value?.adaptiveBindGrouping, false),
    enableBeamExclusion: normalizeBoolean(value?.enableBeamExclusion, false),
    beamExclusionMarginMm: normalizePositiveNumber(
      value?.beamExclusionMarginMm,
      DEFAULT_SCAN_BEAM_EXCLUSION_MARGIN_MM,
    ),
    bindGroupRowThresholdMm: normalizePositiveNumber(
      value?.bindGroupRowThresholdMm,
      DEFAULT_BIND_GROUP_ROW_THRESHOLD_MM,
    ),
    bindGroupColumnThresholdMm: normalizePositiveNumber(
      value?.bindGroupColumnThresholdMm,
      DEFAULT_BIND_GROUP_COLUMN_THRESHOLD_MM,
    ),
    scanLinearCompensation: normalizeScanLinearCompensation(value?.scanLinearCompensation),
    bindExecutionCabinMinZMm: normalizeNonNegativeNumber(
      value?.bindExecutionCabinMinZMm,
      DEFAULT_BIND_EXECUTION_CABIN_MIN_Z_MM,
    ),
    bindExecutionCabinZMode: normalizeBindExecutionCabinZMode(value?.bindExecutionCabinZMode),
    linearModuleBindRangeMm: normalizeTcpWorkspaceBoundaryMm(value?.linearModuleBindRangeMm),
  };
  writeStoredJsonValue(VISUAL_DEBUG_SETTINGS_KEY, payload);
}

export function loadCameraSdkSettings() {
  try {
    const raw = readStoredRawValue(CAMERA_SDK_SETTINGS_KEY);
    if (!raw) {
      return normalizeCameraSdkSettings();
    }
    return normalizeCameraSdkSettings(JSON.parse(raw));
  } catch {
    return normalizeCameraSdkSettings();
  }
}

export function saveCameraSdkSettings(value) {
  const payload = normalizeCameraSdkSettings(value);
  writeStoredJsonValue(CAMERA_SDK_SETTINGS_KEY, payload);
}

export function loadTopicLayerStatePreference() {
  try {
    const raw = readStoredRawValue(TOPIC_LAYER_STATE_KEY);
    if (!raw) {
      return normalizeTopicLayerState();
    }
    return normalizeTopicLayerState(JSON.parse(raw));
  } catch {
    return normalizeTopicLayerState();
  }
}

export function saveTopicLayerStatePreference(value) {
  writeStoredJsonValue(TOPIC_LAYER_STATE_KEY, normalizeTopicLayerState(value));
}

export function loadViewerLayout(layoutId) {
  if (!layoutId) {
    return null;
  }
  try {
    const raw = readStoredRawValue(`${VIEWER_LAYOUT_PREFIX}${layoutId}`);
    if (!raw) {
      return null;
    }
    return JSON.parse(raw);
  } catch {
    return null;
  }
}

export function saveViewerLayout(layoutId, value) {
  if (!layoutId) {
    return;
  }
  writeStoredJsonValue(`${VIEWER_LAYOUT_PREFIX}${layoutId}`, value);
}
