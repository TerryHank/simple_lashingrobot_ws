export const DISPLAY_PREFERENCES_KEY = "tie_robot_frontend_display_preferences";
export const VIEWER_LAYOUT_PREFIX = "tie_robot_frontend_layout_";

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
