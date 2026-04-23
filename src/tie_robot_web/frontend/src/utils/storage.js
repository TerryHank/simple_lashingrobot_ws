export const DISPLAY_PREFERENCES_KEY = "tie_robot_frontend_display_preferences";
export const VIEWER_LAYOUT_PREFIX = "tie_robot_frontend_layout_";
export const THEME_PREFERENCE_KEY = "tie_robot_frontend_theme";
export const SETTINGS_HOME_PAGE_KEY = "tie_robot_frontend_settings_home_page";
export const CONTROL_PANEL_VISIBLE_TASKS_KEY = "tie_robot_frontend_control_panel_visible_tasks";
export const CUSTOM_CONTROL_PANEL_BUTTONS_KEY = "tie_robot_frontend_custom_control_panel_buttons";

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

export function loadControlPanelVisibleTasks() {
  try {
    const raw = localStorage.getItem(CONTROL_PANEL_VISIBLE_TASKS_KEY);
    if (!raw) {
      return null;
    }
    const parsed = JSON.parse(raw);
    return Array.isArray(parsed) ? parsed : null;
  } catch {
    return null;
  }
}

export function saveControlPanelVisibleTasks(taskIds) {
  if (!Array.isArray(taskIds)) {
    return;
  }
  try {
    localStorage.setItem(CONTROL_PANEL_VISIBLE_TASKS_KEY, JSON.stringify(taskIds));
  } catch {
    // ignore storage failures
  }
}

export function loadCustomControlPanelButtons() {
  try {
    const raw = localStorage.getItem(CUSTOM_CONTROL_PANEL_BUTTONS_KEY);
    if (!raw) {
      return [];
    }
    const parsed = JSON.parse(raw);
    return Array.isArray(parsed) ? parsed : [];
  } catch {
    return [];
  }
}

export function saveCustomControlPanelButtons(buttons) {
  if (!Array.isArray(buttons)) {
    return;
  }
  try {
    localStorage.setItem(CUSTOM_CONTROL_PANEL_BUTTONS_KEY, JSON.stringify(buttons));
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
