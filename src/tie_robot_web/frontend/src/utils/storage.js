import {
  DEFAULT_GLOBAL_EXECUTION_MODE,
  FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE,
  GLOBAL_EXECUTION_MODE_OPTIONS,
} from "../config/visualRecognitionMode.js";
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
export const RECOGNITION_POSE_KEY = "tie_robot_frontend_recognition_pose";
export const VISUAL_DEBUG_SETTINGS_KEY = "tie_robot_frontend_visual_debug_settings";
export const TOPIC_LAYER_STATE_KEY = "tie_robot_frontend_topic_layer_state";
export const DEFAULT_BIND_EXECUTION_CABIN_MIN_Z_MM = 485;

function normalizePositiveNumber(value, fallback) {
  const numericValue = Number(value);
  return Number.isFinite(numericValue) && numericValue > 0 ? numericValue : fallback;
}

function normalizeNonNegativeNumber(value, fallback) {
  const numericValue = Number(value);
  return Number.isFinite(numericValue) && numericValue >= 0 ? numericValue : fallback;
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
    const raw = localStorage.getItem(DISPLAY_PREFERENCES_KEY);
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
  try {
    localStorage.setItem(DISPLAY_PREFERENCES_KEY, JSON.stringify(value));
  } catch {
    // ignore storage failures
  }
}

export function loadThemePreference() {
  const prefersDark = typeof window !== "undefined"
    && window.matchMedia
    && window.matchMedia("(prefers-color-scheme: dark)").matches;
  const fallback = prefersDark ? "dark" : "light";
  try {
    const raw = localStorage.getItem(THEME_PREFERENCE_KEY);
    return raw === "light" || raw === "dark" ? raw : fallback;
  } catch {
    return fallback;
  }
}

export function saveThemePreference(theme) {
  if (theme !== "light" && theme !== "dark") {
    return;
  }
  try {
    localStorage.setItem(THEME_PREFERENCE_KEY, theme);
  } catch {
    // ignore storage failures
  }
}

export function loadSettingsHomePagePreference() {
  try {
    const raw = localStorage.getItem(SETTINGS_HOME_PAGE_KEY);
    return typeof raw === "string" && raw ? raw : "topics";
  } catch {
    return "topics";
  }
}

export function saveSettingsHomePagePreference(pageId) {
  if (!pageId) {
    return;
  }
  try {
    localStorage.setItem(SETTINGS_HOME_PAGE_KEY, pageId);
  } catch {
    // ignore storage failures
  }
}

export function loadSettingsPageOrderPreference() {
  try {
    const raw = localStorage.getItem(SETTINGS_PAGE_ORDER_KEY);
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
  try {
    localStorage.setItem(SETTINGS_PAGE_ORDER_KEY, JSON.stringify(pageIds));
  } catch {
    // ignore storage failures
  }
}

export function loadCabinRemoteSettings() {
  const defaults = { step: 50, speed: 300, moveMode: "absolute" };
  try {
    const raw = localStorage.getItem(CABIN_REMOTE_SETTINGS_KEY);
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
  try {
    localStorage.setItem(CABIN_REMOTE_SETTINGS_KEY, JSON.stringify(payload));
  } catch {
    // ignore storage failures
  }
}

export function loadTcpLinearRemoteSettings() {
  const defaults = { step: 5, angleStep: 5, speed: 250 };
  try {
    const raw = localStorage.getItem(TCP_LINEAR_REMOTE_SETTINGS_KEY);
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
  try {
    localStorage.setItem(TCP_LINEAR_REMOTE_SETTINGS_KEY, JSON.stringify(payload));
  } catch {
    // ignore storage failures
  }
}

function normalizePingHost(value, fallback) {
  const normalized = String(value ?? "").trim();
  return normalized || fallback;
}

export function loadNetworkPingSettings() {
  const defaults = { cabinHost: "192.168.6.62", moduanHost: "192.168.6.167" };
  try {
    const raw = localStorage.getItem(NETWORK_PING_SETTINGS_KEY);
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
  try {
    localStorage.setItem(NETWORK_PING_SETTINGS_KEY, JSON.stringify(payload));
  } catch {
    // ignore storage failures
  }
}

export function loadRecognitionPose(defaultPose = null) {
  try {
    const raw = localStorage.getItem(RECOGNITION_POSE_KEY);
    if (!raw) {
      return normalizeCabinPose(defaultPose);
    }
    return normalizeCabinPose(JSON.parse(raw), defaultPose);
  } catch {
    return normalizeCabinPose(defaultPose);
  }
}

export function saveRecognitionPose(value) {
  const payload = normalizeCabinPose(value);
  if (!payload) {
    return;
  }
  try {
    localStorage.setItem(RECOGNITION_POSE_KEY, JSON.stringify(payload));
  } catch {
    // ignore storage failures
  }
}

export function loadVisualDebugSettings() {
  const defaults = {
    stableFrameCount: 3,
    requestMode: FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE,
    executionMode: DEFAULT_GLOBAL_EXECUTION_MODE,
    adaptiveBindGrouping: false,
    enableBeamExclusion: false,
    bindExecutionCabinMinZMm: DEFAULT_BIND_EXECUTION_CABIN_MIN_Z_MM,
    linearModuleBindRangeMm: normalizeTcpWorkspaceBoundaryMm(),
  };
  try {
    const raw = localStorage.getItem(VISUAL_DEBUG_SETTINGS_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      stableFrameCount: Math.max(1, Math.round(normalizePositiveNumber(parsed?.stableFrameCount, defaults.stableFrameCount))),
      requestMode: defaults.requestMode,
      executionMode: normalizeGlobalExecutionMode(parsed?.executionMode, defaults.executionMode),
      adaptiveBindGrouping: normalizeBoolean(parsed?.adaptiveBindGrouping, defaults.adaptiveBindGrouping),
      enableBeamExclusion: normalizeBoolean(parsed?.enableBeamExclusion, defaults.enableBeamExclusion),
      bindExecutionCabinMinZMm: normalizeNonNegativeNumber(
        parsed?.bindExecutionCabinMinZMm,
        defaults.bindExecutionCabinMinZMm,
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
    adaptiveBindGrouping: normalizeBoolean(value?.adaptiveBindGrouping, false),
    enableBeamExclusion: normalizeBoolean(value?.enableBeamExclusion, false),
    bindExecutionCabinMinZMm: normalizeNonNegativeNumber(
      value?.bindExecutionCabinMinZMm,
      DEFAULT_BIND_EXECUTION_CABIN_MIN_Z_MM,
    ),
    linearModuleBindRangeMm: normalizeTcpWorkspaceBoundaryMm(value?.linearModuleBindRangeMm),
  };
  try {
    localStorage.setItem(VISUAL_DEBUG_SETTINGS_KEY, JSON.stringify(payload));
  } catch {
    // ignore storage failures
  }
}

export function loadTopicLayerStatePreference() {
  try {
    const raw = localStorage.getItem(TOPIC_LAYER_STATE_KEY);
    if (!raw) {
      return normalizeTopicLayerState();
    }
    return normalizeTopicLayerState(JSON.parse(raw));
  } catch {
    return normalizeTopicLayerState();
  }
}

export function saveTopicLayerStatePreference(value) {
  try {
    localStorage.setItem(TOPIC_LAYER_STATE_KEY, JSON.stringify(normalizeTopicLayerState(value)));
  } catch {
    // ignore storage failures
  }
}

export function loadViewerLayout(layoutId) {
  if (!layoutId) {
    return null;
  }
  try {
    const raw = localStorage.getItem(`${VIEWER_LAYOUT_PREFIX}${layoutId}`);
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
  try {
    localStorage.setItem(`${VIEWER_LAYOUT_PREFIX}${layoutId}`, JSON.stringify(value));
  } catch {
    // ignore storage failures
  }
}
