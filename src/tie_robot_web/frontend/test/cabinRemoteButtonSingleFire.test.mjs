import assert from "node:assert/strict";

import { UIController } from "../src/ui/UIController.js";

function createFakeButton({ direction = "xPositive", disabled = false } = {}) {
  const listeners = new Map();
  return {
    dataset: { cabinRemoteDirection: direction },
    disabled,
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
  };
}

const moveButton = createFakeButton({ direction: "zPositive" });
const stopButton = createFakeButton();
const calls = [];

UIController.prototype.onCabinRemoteAction.call(
  {
    refs: {
      cabinRemoteButtons: [moveButton],
      cabinRemoteStopButton: stopButton,
    },
  },
  (directionId, event) => calls.push({ directionId, event }),
);

assert.equal(moveButton.listenerCount("pointerdown"), 0);
assert.equal(moveButton.listenerCount("pointerup"), 0);
assert.equal(moveButton.listenerCount("pointerleave"), 0);
assert.equal(moveButton.listenerCount("pointercancel"), 0);

moveButton.dispatch("pointerdown", {
  button: 0,
  preventDefault() {
    calls.push({ directionId: "unexpectedPreventDefault", event: { type: "pointerdown" } });
  },
});
assert.deepEqual(calls, []);

moveButton.dispatch("click");
assert.deepEqual(calls, [{ directionId: "zPositive", event: { type: "click" } }]);

moveButton.dispatch("click");
assert.deepEqual(calls, [
  { directionId: "zPositive", event: { type: "click" } },
  { directionId: "zPositive", event: { type: "click" } },
]);
