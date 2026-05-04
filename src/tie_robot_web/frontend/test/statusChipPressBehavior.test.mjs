import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { UIController } from "../src/ui/UIController.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

function createFakeStatusChip({ statusId = "chassis", statusAction = "", statusLongAction = "" } = {}) {
  const listeners = new Map();
  const classNames = new Set(["system-status-item", "success", "is-interactive"]);
  const actionLabel = { textContent: "" };
  const statusLabel = { textContent: statusId };
  const chip = {
    dataset: { statusId, statusAction, statusLongAction },
    disabled: false,
    className: "",
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

const expectedActionsByStatus = [
  ["chassis", "success", "stopCabinSubsystem", "restartCabinSubsystem", "关闭"],
  ["chassis", "warn", "startCabinSubsystem", "restartCabinSubsystem", "启动"],
  ["moduan", "success", "stopModuanSubsystem", "restartModuanSubsystem", "关闭"],
  ["moduan", "warn", "startModuanSubsystem", "restartModuanSubsystem", "启动"],
  ["visual", "success", "stopVisualSubsystem", "restartVisualSubsystem", "关闭"],
  ["visual", "warn", "startVisualSubsystem", "restartVisualSubsystem", "启动"],
];

for (const [statusId, level, shortAction, longAction, label] of expectedActionsByStatus) {
  const chip = createFakeStatusChip({ statusId });
  UIController.prototype.setStatusChipState.call({ rootElement: makeRootForChip(chip) }, statusId, level, "状态详情");

  assert.equal(chip.dataset.statusAction, shortAction, `${statusId}/${level} short action`);
  assert.equal(chip.dataset.statusLongAction, longAction, `${statusId}/${level} long action`);
  assert.equal(chip.actionLabel.textContent, label, `${statusId}/${level} visible action label`);
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
assert.equal(scheduledTimers.at(-1).delay, 2000);
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
assert.equal(scheduledTimers.at(-1).delay, 2000);
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

const stylesheetText = readFileSync(resolve(frontendRoot, "src/styles/app.css"), "utf-8");
assert.match(stylesheetText, /animation:\s*status-charge-fill 2s linear forwards/);
assert.match(stylesheetText, /animation:\s*status-charge-sweep 2s ease-out forwards/);
assert.match(stylesheetText, /\.system-status-item::after[\s\S]*width:\s*100%;/);
assert.match(stylesheetText, /transform:\s*translateX\(-100%\);/);
assert.match(stylesheetText, /\.system-status-item\.is-long-press-complete::before[\s\S]*transform:\s*scaleX\(1\);/);
assert.match(stylesheetText, /@keyframes status-charge-sweep[\s\S]*transform:\s*translateX\(100%\);/);
