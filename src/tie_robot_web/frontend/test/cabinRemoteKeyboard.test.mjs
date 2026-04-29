import assert from "node:assert/strict";

import {
  consumeCabinRemoteKeyboardEvent,
  resolveCabinRemoteKeyboardActionFromKey,
  resolveCabinRemoteDirectionFromKey,
  shouldIgnoreCabinRemoteKeyboardTarget,
} from "../src/utils/cabinRemoteKeyboard.js";

function createElement({ id = "", closestMatch = null } = {}) {
  return {
    id,
    closest(selector) {
      return closestMatch && selector.includes(closestMatch) ? this : null;
    },
  };
}

assert.equal(resolveCabinRemoteDirectionFromKey("Q"), "zPositive");
assert.equal(resolveCabinRemoteDirectionFromKey("w"), "xPositive");
assert.equal(resolveCabinRemoteDirectionFromKey(" "), null);
assert.deepEqual(resolveCabinRemoteKeyboardActionFromKey("Q"), { type: "move", directionId: "zPositive" });
assert.deepEqual(resolveCabinRemoteKeyboardActionFromKey(" "), { type: "stop" });
assert.deepEqual(resolveCabinRemoteKeyboardActionFromKey("Spacebar"), { type: "stop" });
assert.equal(resolveCabinRemoteKeyboardActionFromKey("Escape"), null);

{
  const calls = [];
  consumeCabinRemoteKeyboardEvent({
    preventDefault: () => calls.push("preventDefault"),
    stopImmediatePropagation: () => calls.push("stopImmediatePropagation"),
  });
  assert.deepEqual(calls, ["preventDefault", "stopImmediatePropagation"]);
}

assert.equal(
  shouldIgnoreCabinRemoteKeyboardTarget(
    createElement({ id: "cabinKeyboardRemoteToggle", closestMatch: "input" }),
  ),
  false,
);

assert.equal(
  shouldIgnoreCabinRemoteKeyboardTarget(
    createElement({ id: "cabinRemoteStep", closestMatch: "input" }),
  ),
  true,
);

assert.equal(
  shouldIgnoreCabinRemoteKeyboardTarget(createElement({ closestMatch: ".xterm" })),
  true,
);
