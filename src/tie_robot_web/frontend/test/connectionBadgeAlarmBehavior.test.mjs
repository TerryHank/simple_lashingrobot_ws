import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { UIController } from "../src/ui/UIController.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

function createFakeConnectionBadge() {
  const listeners = new Map();
  const classNames = new Set(["toolbar-connection-badge", "success"]);
  const labelNode = { textContent: "" };
  const actionNode = { textContent: "" };
  return {
    dataset: {},
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
    removeAttribute(name) {
      this.attributes.delete(name);
    },
    querySelector(selector) {
      if (selector === ".toolbar-connection-label") {
        return labelNode;
      }
      if (selector === ".toolbar-connection-action-label") {
        return actionNode;
      }
      return null;
    },
    labelNode,
    actionNode,
  };
}

const scheduledTimers = [];
global.window = {
  setTimeout(callback, delay) {
    const timerId = scheduledTimers.length + 100;
    scheduledTimers.push({ timerId, callback, delay, cleared: false });
    return timerId;
  },
  clearTimeout(timerId) {
    const timer = scheduledTimers.find((item) => item.timerId === timerId);
    if (timer) {
      timer.cleared = true;
    }
  },
};

const connectionBadge = createFakeConnectionBadge();
const context = {
  rootElement: { querySelectorAll: () => [] },
  refs: { connectionBadge },
  connectionAlarmMessages: [],
  connectionInfo: null,
  setStatusChipState() {},
  getPendingActionLabel: UIController.prototype.getPendingActionLabel,
  renderConnectionBadgeState: UIController.prototype.renderConnectionBadgeState,
};

UIController.prototype.setConnectionInfo.call(context, "ws://127.0.0.1:9090", "连接成功", "success");
assert.equal(connectionBadge.labelNode.textContent, "连接成功");
assert.equal(connectionBadge.actionNode.textContent, "长按重启");
assert.equal(connectionBadge.dataset.connectionAction, "");
assert.equal(connectionBadge.dataset.connectionLongAction, "restartRosStack");
assert.equal(connectionBadge.dataset.hasAction, "true");
assert.doesNotMatch(connectionBadge.title, /短按报警复位/);
assert.match(connectionBadge.title, /长按0\.5秒重启ROS/);

const calls = [];
UIController.prototype.onConnectionAction.call(context, (actionId) => calls.push(actionId));
assert.equal(connectionBadge.listenerCount("pointerdown"), 1);
assert.equal(connectionBadge.listenerCount("pointerup"), 1);
assert.equal(connectionBadge.listenerCount("pointerleave"), 1);
assert.equal(connectionBadge.listenerCount("pointercancel"), 1);
assert.equal(connectionBadge.listenerCount("click"), 1);

UIController.prototype.setConnectionInfo.call(context, "ws://127.0.0.1:9090", "开始连接 ROSBridge", "info");
assert.equal(connectionBadge.labelNode.textContent, "连接中");
assert.equal(connectionBadge.actionNode.textContent, "立即重连");
assert.equal(connectionBadge.dataset.connectionAction, "manualRosReconnect");
assert.equal(connectionBadge.dataset.connectionLongAction, "");
assert.equal(connectionBadge.dataset.hasAction, "true");
assert.match(connectionBadge.title, /短按立即重连/);
connectionBadge.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(calls, ["manualRosReconnect"]);
calls.length = 0;

UIController.prototype.setConnectionInfo.call(context, "ws://127.0.0.1:9090", "自动重连中", "reconnecting");
assert.equal(connectionBadge.labelNode.textContent, "重连中");
assert.equal(connectionBadge.actionNode.textContent, "立即重连");
assert.equal(connectionBadge.dataset.connectionAction, "manualRosReconnect");
connectionBadge.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(calls, ["manualRosReconnect"]);
calls.length = 0;

UIController.prototype.setConnectionInfo.call(context, "ws://127.0.0.1:9090", "连接成功", "success");
connectionBadge.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(calls, []);

scheduledTimers.length = 0;
connectionBadge.dispatch("pointerdown", {
  button: 0,
  preventDefault() {},
});
assert.equal(connectionBadge.classList.contains("is-long-press-charging"), true);
assert.equal(scheduledTimers.at(-1).delay, 500);
UIController.prototype.setConnectionInfo.call(context, "ws://127.0.0.1:9090", "连接成功", "success");
assert.equal(connectionBadge.classList.contains("is-long-press-charging"), true);
connectionBadge.dispatch("pointerup");
assert.equal(connectionBadge.classList.contains("is-long-press-charging"), false);

UIController.prototype.setSystemActionPending.call(context, "restartRosStack", true);
assert.equal(connectionBadge.disabled, true);
assert.equal(connectionBadge.classList.contains("is-pending"), true);
assert.equal(connectionBadge.classList.contains("is-booting"), true);
assert.equal(connectionBadge.dataset.pendingActionId, "restartRosStack");
assert.equal(connectionBadge.labelNode.textContent, "连接成功");
assert.equal(connectionBadge.actionNode.textContent, "重启中");
assert.equal(connectionBadge.attributes.get("aria-busy"), "true");
UIController.prototype.setConnectionInfo.call(context, "ws://127.0.0.1:9090", "连接成功", "success");
assert.equal(connectionBadge.classList.contains("is-pending"), true);
assert.equal(connectionBadge.actionNode.textContent, "重启中");
UIController.prototype.setSystemActionPending.call(context, "restartRosStack", false);
assert.equal(connectionBadge.disabled, false);
assert.equal(connectionBadge.classList.contains("is-pending"), false);
assert.equal(connectionBadge.classList.contains("is-booting"), false);
assert.equal(connectionBadge.dataset.pendingActionId, undefined);
assert.equal(connectionBadge.actionNode.textContent, "长按重启");
assert.equal(connectionBadge.attributes.has("aria-busy"), false);

UIController.prototype.setConnectionAlarmState.call(context, ["X轴异常", "Y轴异常"]);
assert.equal(connectionBadge.labelNode.textContent, "连接成功");
assert.equal(connectionBadge.actionNode.textContent, "长按重启");
assert.equal(connectionBadge.classList.contains("success"), true);
assert.equal(connectionBadge.dataset.connectionAction, "");
assert.equal(connectionBadge.dataset.connectionLongAction, "restartRosStack");
assert.doesNotMatch(connectionBadge.title, /X轴异常|Y轴异常|报警复位/);
connectionBadge.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(calls, []);

calls.length = 0;
scheduledTimers.length = 0;
connectionBadge.dispatch("pointerdown", {
  button: 0,
  preventDefault() {},
});
assert.equal(connectionBadge.classList.contains("is-long-press-charging"), true);
assert.equal(scheduledTimers.at(-1).delay, 500);
scheduledTimers.at(-1).callback();
assert.equal(connectionBadge.classList.contains("is-long-press-complete"), true);
connectionBadge.dispatch("pointerup");
connectionBadge.dispatch("click", {
  preventDefault() {},
  stopPropagation() {},
});
assert.deepEqual(calls, ["restartRosStack"]);

const stylesheetText = readFileSync(resolve(frontendRoot, "src/styles/app.css"), "utf-8");
assert.match(
  stylesheetText,
  /\.toolbar-connection-badge\.is-long-press-charging\s+\.toolbar-connection-charge[\s\S]*animation:\s*status-charge-fill 0\.5s linear forwards/,
);
assert.match(
  stylesheetText,
  /\.toolbar-connection-badge\.is-long-press-complete\s+\.toolbar-connection-charge[\s\S]*transform:\s*scaleX\(1\);/,
);
assert.match(
  stylesheetText,
  /\.toolbar-connection-badge\.is-pending::before[\s\S]*animation:\s*status-spin 0\.72s linear infinite/,
);
assert.match(
  stylesheetText,
  /\.toolbar-connection-badge\.is-pending\s+\.toolbar-connection-action-label[\s\S]*transform:\s*translateX\(0\);/,
);
