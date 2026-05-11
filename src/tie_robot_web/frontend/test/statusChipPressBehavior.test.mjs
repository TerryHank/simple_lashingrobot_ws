import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { UIController } from "../src/ui/UIController.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

function createFakeStatusChip({ statusId = "chassis", statusAction = "", statusLongAction = "", statusLabelText = statusId } = {}) {
  const listeners = new Map();
  const classNames = new Set(["system-status-item", "success", "is-interactive"]);
  const actionLabel = { textContent: "" };
  const statusLabel = { textContent: statusLabelText };
  const chip = {
    dataset: { statusId, statusAction, statusLongAction },
    disabled: false,
    title: "",
    attributes: new Map(),
    get className() {
      return [...classNames].join(" ");
    },
    set className(value) {
      classNames.clear();
      String(value || "")
        .split(/\s+/)
        .filter(Boolean)
        .forEach((name) => classNames.add(name));
    },
    classList: {
      add(name) {
        classNames.add(name);
      },
      remove(name) {
        classNames.delete(name);
      },
      contains(name) {
        return classNames.has(name);
      },
      toggle(name, force) {
        if (force) {
          classNames.add(name);
        } else {
          classNames.delete(name);
        }
      },
    },
    addEventListener(eventName, listener) {
      const eventListeners = listeners.get(eventName) || [];
      eventListeners.push(listener);
      listeners.set(eventName, eventListeners);
    },
    dispatch(eventName, event = {}) {
      const eventListeners = listeners.get(eventName) || [];
      eventListeners.forEach((listener) => listener(event));
    },
    listenerCount(eventName) {
      return (listeners.get(eventName) || []).length;
    },
    setAttribute(name, value) {
      this.attributes.set(name, value);
    },
    querySelector(selector) {
      if (selector === ".system-status-action-label") {
        return actionLabel;
      }
      if (selector === ".system-status-label") {
        return statusLabel;
      }
      return null;
    },
    actionLabel,
    statusLabel,
  };
  return chip;
}

function makeRootForChip(chip) {
  return {
    querySelector(selector) {
      return selector === `[data-status-id="${chip.dataset.statusId}"]` ? chip : null;
    },
    querySelectorAll(selector) {
      return selector === "[data-status-id][data-status-action]" ? [chip] : [];
    },
  };
}

function createFakeBottomLinearModulePosition() {
  const listeners = new Map();
  const classNames = new Set(["bottom-linear-module-position"]);
  return {
    dataset: {},
    disabled: false,
    title: "",
    attributes: new Map(),
    classList: {
      add(name) {
        classNames.add(name);
      },
      remove(name) {
        classNames.delete(name);
      },
      contains(name) {
        return classNames.has(name);
      },
    },
    addEventListener(eventName, listener) {
      const eventListeners = listeners.get(eventName) || [];
      eventListeners.push(listener);
      listeners.set(eventName, eventListeners);
    },
    dispatch(eventName, event = {}) {
      const eventListeners = listeners.get(eventName) || [];
      eventListeners.forEach((listener) => listener(event));
    },
    listenerCount(eventName) {
      return (listeners.get(eventName) || []).length;
    },
    setAttribute(name, value) {
      this.attributes.set(name, value);
    },
  };
}

const expectedActionsByStatus = [
  ["chassis", "success", "stopCabinSubsystem", "restartCabinSubsystem", "关闭", "索驱"],
  ["chassis", "warn", "startCabinSubsystem", "restartCabinSubsystem", "启动", "索驱"],
  ["chassis", "error", "restartCabinSubsystem", "restartCabinSubsystem", "重启", "索驱报警"],
  ["moduan", "success", "stopModuanSubsystem", "restartModuanSubsystem", "关闭", "末端"],
  ["moduan", "warn", "startModuanSubsystem", "restartModuanSubsystem", "启动", "末端"],
  ["moduan", "error", "restartModuanSubsystem", "restartModuanSubsystem", "重启", "末端报警"],
  ["visual", "success", "stopVisualSubsystem", "restartVisualSubsystem", "关闭", "视觉"],
  ["visual", "warn", "startVisualSubsystem", "restartVisualSubsystem", "启动", "视觉"],
  ["visual", "error", "startVisualSubsystem", "restartVisualSubsystem", "启动", "视觉"],
];

for (const [statusId, level, shortAction, longAction, label, visibleStatusLabel] of expectedActionsByStatus) {
  const chip = createFakeStatusChip({ statusId, statusLabelText: visibleStatusLabel.replace(/报警$/, "") });
  UIController.prototype.setStatusChipState.call({ rootElement: makeRootForChip(chip) }, statusId, level, "状态详情");

  assert.equal(chip.dataset.statusAction, shortAction, `${statusId}/${level} short action`);
  assert.equal(chip.dataset.statusLongAction, longAction, `${statusId}/${level} long action`);
  assert.equal(chip.actionLabel.textContent, label, `${statusId}/${level} visible action label`);
  assert.equal(chip.statusLabel.textContent, visibleStatusLabel, `${statusId}/${level} visible status label`);
}

const scheduledTimers = [];
let clearedTimer = false;
global.window = {
  setTimeout(callback, delay) {
    const timerId = scheduledTimers.length + 7;
    scheduledTimers.push({ timerId, callback, delay, cleared: false });
    return timerId;
  },
  clearTimeout(timerId) {
    const timer = scheduledTimers.find((item) => item.timerId === timerId);
    if (timer) {
      timer.cleared = true;
      clearedTimer = true;
    }
  },
};

const chip = createFakeStatusChip({
  statusId: "chassis",
  statusAction: "stopCabinSubsystem",
  statusLongAction: "restartCabinSubsystem",
});
const calls = [];

UIController.prototype.onStatusChipAction.call({ rootElement: makeRootForChip(chip) }, (statusId, actionId) => {
  calls.push({ statusId, actionId });
});

assert.equal(chip.listenerCount("pointerdown"), 1);
assert.equal(chip.listenerCount("pointerup"), 1);
assert.equal(chip.listenerCount("pointerleave"), 1);
assert.equal(chip.listenerCount("pointercancel"), 1);
assert.equal(chip.listenerCount("click"), 1);

chip.dispatch("pointerdown", {
  button: 0,
  preventDefault() {},
});
assert.equal(chip.classList.contains("is-long-press-charging"), true);
assert.equal(scheduledTimers.at(-1).delay, 500);
UIController.prototype.setStatusChipState.call({ rootElement: makeRootForChip(chip) }, "chassis", "success", "状态刷新");
assert.equal(chip.classList.contains("is-long-press-charging"), true);
chip.dispatch("pointerup");
assert.equal(clearedTimer, true);
assert.equal(chip.classList.contains("is-long-press-charging"), false);
assert.equal(chip.classList.contains("is-long-press-complete"), false);
chip.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(calls, [{ statusId: "chassis", actionId: "stopCabinSubsystem" }]);

calls.length = 0;
scheduledTimers.length = 0;
clearedTimer = false;

chip.dispatch("pointerdown", {
  button: 0,
  preventDefault() {},
});
assert.equal(chip.classList.contains("is-long-press-charging"), true);
assert.equal(scheduledTimers.at(-1).delay, 500);
scheduledTimers.at(-1).callback();
assert.equal(chip.classList.contains("is-long-press-charging"), false);
assert.equal(chip.classList.contains("is-long-press-complete"), true);
assert.equal(scheduledTimers.at(-1).delay, 240);
chip.dispatch("pointerup");
chip.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(calls, [{ statusId: "chassis", actionId: "restartCabinSubsystem" }]);

const bottomLinearModulePosition = createFakeBottomLinearModulePosition();
const bottomZeroCalls = [];
scheduledTimers.length = 0;
clearedTimer = false;

UIController.prototype.onBottomLinearModuleZeroAction.call(
  { refs: { bottomLinearModulePosition } },
  () => bottomZeroCalls.push("zero"),
);

assert.equal(bottomLinearModulePosition.listenerCount("pointerdown"), 1);
assert.equal(bottomLinearModulePosition.listenerCount("pointerup"), 1);
assert.equal(bottomLinearModulePosition.listenerCount("pointerleave"), 1);
assert.equal(bottomLinearModulePosition.listenerCount("pointercancel"), 1);
assert.equal(bottomLinearModulePosition.listenerCount("click"), 1);

bottomLinearModulePosition.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(bottomZeroCalls, []);

bottomLinearModulePosition.dispatch("pointerdown", {
  button: 0,
  preventDefault() {},
});
assert.equal(bottomLinearModulePosition.classList.contains("is-long-press-charging"), true);
assert.equal(scheduledTimers.at(-1).delay, 500);
scheduledTimers.at(-1).callback();
assert.equal(bottomLinearModulePosition.classList.contains("is-long-press-charging"), false);
assert.equal(bottomLinearModulePosition.classList.contains("is-long-press-complete"), true);
assert.equal(scheduledTimers.at(-1).delay, 240);
bottomLinearModulePosition.dispatch("pointerup");
bottomLinearModulePosition.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(bottomZeroCalls, ["zero"]);

const stylesheetText = readFileSync(resolve(frontendRoot, "src/styles/app.css"), "utf-8");
assert.match(stylesheetText, /animation:\s*status-charge-fill 0\.5s linear forwards/);
assert.match(stylesheetText, /animation:\s*status-charge-sweep 0\.5s ease-out forwards/);
assert.match(stylesheetText, /\.system-status-item::after[\s\S]*width:\s*100%;/);
assert.match(stylesheetText, /transform:\s*translateX\(-100%\);/);
assert.match(stylesheetText, /\.system-status-item\.is-long-press-complete::before[\s\S]*transform:\s*scaleX\(1\);/);
assert.match(stylesheetText, /@keyframes status-charge-sweep[\s\S]*transform:\s*translateX\(100%\);/);
assert.match(stylesheetText, /\.bottom-linear-module-position\s*{[\s\S]*pointer-events:\s*auto;/);
assert.match(stylesheetText, /\.bottom-linear-module-position\.is-long-press-charging::before[\s\S]*animation:\s*status-charge-fill 0\.5s linear forwards/);
assert.match(stylesheetText, /\.bottom-linear-module-position\.is-long-press-complete::before[\s\S]*transform:\s*scaleX\(1\);/);
assert.match(stylesheetText, /\.bottom-linear-module-position\.is-long-press-charging\s*{[^}]*overflow:\s*hidden;/);
