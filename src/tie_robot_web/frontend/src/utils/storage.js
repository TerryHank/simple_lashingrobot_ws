export const DISPLAY_PREFERENCES_KEY = "tie_robot_frontend_display_preferences";
export const VIEWER_LAYOUT_PREFIX = "tie_robot_frontend_layout_";
export const THEME_PREFERENCE_KEY = "tie_robot_frontend_theme";
export const SETTINGS_HOME_PAGE_KEY = "tie_robot_frontend_settings_home_page";
export const SETTINGS_PAGE_ORDER_KEY = "tie_robot_frontend_settings_page_order";
export const CABIN_REMOTE_SETTINGS_KEY = "tie_robot_frontend_cabin_remote_settings";
export const RECOGNITION_POSE_KEY = "tie_robot_frontend_recognition_pose";
export const VISUAL_DEBUG_SETTINGS_KEY = "tie_robot_frontend_visual_debug_settings";

function normalizePositiveNumber(value, fallback) {
  const numericValue = Number(value);
  return Number.isFinite(numericValue) && numericValue > 0 ? numericValue : fallback;
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
  const defaults = { step: 50, speed: 300 };
  try {
    const raw = localStorage.getItem(CABIN_REMOTE_SETTINGS_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      step: normalizePositiveNumber(parsed?.step, defaults.step),
      speed: normalizePositiveNumber(parsed?.speed, defaults.speed),
    };
  } catch {
    return defaults;
  }
}

export function saveCabinRemoteSettings(value) {
  const defaults = { step: 50, speed: 300 };
  const payload = {
    step: normalizePositiveNumber(value?.step, defaults.step),
    speed: normalizePositiveNumber(value?.speed, defaults.speed),
  };
  try {
    localStorage.setItem(CABIN_REMOTE_SETTINGS_KEY, JSON.stringify(payload));
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
  const defaults = { stableFrameCount: 3, requestMode: 1 };
  try {
    const raw = localStorage.getItem(VISUAL_DEBUG_SETTINGS_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      stableFrameCount: Math.max(1, Math.round(normalizePositiveNumber(parsed?.stableFrameCount, defaults.stableFrameCount))),
      requestMode: [0, 1, 2, 3, 4].includes(Number(parsed?.requestMode))
        ? Number(parsed.requestMode)
        : defaults.requestMode,
    };
  } catch {
    return defaults;
  }
}

export function saveVisualDebugSettings(value) {
  const payload = {
    stableFrameCount: Math.max(1, Math.round(normalizePositiveNumber(value?.stableFrameCount, 3))),
    requestMode: [0, 1, 2, 3, 4].includes(Number(value?.requestMode))
      ? Number(value.requestMode)
      : 1,
  };
  try {
    localStorage.setItem(VISUAL_DEBUG_SETTINGS_KEY, JSON.stringify(payload));
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
