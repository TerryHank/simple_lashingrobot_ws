import assert from "node:assert/strict";

import { UIController } from "../src/ui/UIController.js";

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

let scheduledCallback = null;
let scheduledDelay = null;
let clearedTimer = false;
global.window = {
  setTimeout(callback, delay) {
    scheduledCallback = callback;
    scheduledDelay = delay;
    return 7;
  },
  clearTimeout(timerId) {
    if (timerId === 7) {
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
assert.equal(scheduledDelay, 800);
chip.dispatch("pointerup");
assert.equal(clearedTimer, true);
assert.equal(chip.classList.contains("is-long-press-charging"), false);
chip.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(calls, [{ statusId: "chassis", actionId: "stopCabinSubsystem" }]);

calls.length = 0;
scheduledCallback = null;
scheduledDelay = null;
clearedTimer = false;

chip.dispatch("pointerdown", {
  button: 0,
  preventDefault() {},
});
assert.equal(chip.classList.contains("is-long-press-charging"), true);
assert.equal(scheduledDelay, 800);
scheduledCallback();
assert.equal(chip.classList.contains("is-long-press-charging"), false);
chip.dispatch("pointerup");
chip.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(calls, [{ statusId: "chassis", actionId: "restartCabinSubsystem" }]);
