const CABIN_REMOTE_KEYBOARD_EXEMPT_INPUT_IDS = new Set(["cabinKeyboardRemoteToggle"]);

const CABIN_REMOTE_KEYBOARD_IGNORED_SELECTOR = [
  "input",
  "textarea",
  "select",
  "[contenteditable=\"true\"]",
  ".ui-select-shell",
  ".ui-select-menu",
  ".ui-select-trigger",
  ".xterm",
  ".xterm-helper-textarea",
].join(", ");

export function resolveCabinRemoteDirectionFromKey(key) {
  const normalizedKey = String(key || "").toLowerCase();
  const directionMap = {
    q: "zPositive",
    w: "xPositive",
    e: "zNegative",
    a: "yPositive",
    s: "xNegative",
    d: "yNegative",
  };
  return directionMap[normalizedKey] || null;
}

export function resolveCabinRemoteKeyboardActionFromKey(key) {
  if (key === " " || key === "Spacebar") {
    return { type: "stop" };
  }

  const directionId = resolveCabinRemoteDirectionFromKey(key);
  return directionId ? { type: "move", directionId } : null;
}

export function consumeCabinRemoteKeyboardEvent(event) {
  event?.preventDefault?.();
  event?.stopImmediatePropagation?.();
}

function getKeyboardTargetElement(target, activeElement = globalThis.document?.activeElement) {
  if (target && typeof target.closest === "function") {
    return target;
  }
  if (activeElement && typeof activeElement.closest === "function") {
    return activeElement;
  }
  return null;
}

export function shouldIgnoreCabinRemoteKeyboardTarget(target, activeElement) {
  const element = getKeyboardTargetElement(target, activeElement);
  if (!element) {
    return false;
  }

  const ignoredElement = element.closest(CABIN_REMOTE_KEYBOARD_IGNORED_SELECTOR);
  if (!ignoredElement) {
    return false;
  }

  return !CABIN_REMOTE_KEYBOARD_EXEMPT_INPUT_IDS.has(ignoredElement.id);
}
