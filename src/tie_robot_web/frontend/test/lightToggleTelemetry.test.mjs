import assert from "node:assert/strict";

import { LegacyCommandController } from "../src/controllers/LegacyCommandController.js";
import { TOPICS } from "../src/config/topicRegistry.js";
import { ROSLIB } from "../src/vendor/roslib.js";

class FakeTopic {
  static published = [];

  constructor(options) {
    Object.assign(this, options);
  }

  advertise() {}

  publish(message) {
    FakeTopic.published.push({
      name: this.name,
      messageType: this.messageType,
      message: { ...message },
    });
  }
}

ROSLIB.Topic = FakeTopic;
ROSLIB.Message = class {
  constructor(payload) {
    Object.assign(this, payload);
  }
};

const controller = new LegacyCommandController({
  rosConnection: {
    getResources: () => ({ ros: {} }),
  },
});

const syncedLightOnState = controller.syncToggleState("lightEnabled", true);
assert.equal(syncedLightOnState.value, true);
assert.equal(syncedLightOnState.label, "关闭灯光");

const lightOffState = controller.handleToggle("lightEnabled", {});
assert.equal(lightOffState.value, false);
assert.equal(lightOffState.label, "开启灯光");
assert.deepEqual(FakeTopic.published.at(-1), {
  name: TOPICS.control.light,
  messageType: "std_msgs/Bool",
  message: { data: false },
});

const syncedLightOffState = controller.syncToggleState("lightEnabled", false);
assert.equal(syncedLightOffState.value, false);
assert.equal(syncedLightOffState.label, "开启灯光");

const lightOnState = controller.handleToggle("lightEnabled", {});
assert.equal(lightOnState.value, true);
assert.equal(lightOnState.label, "关闭灯光");
assert.deepEqual(FakeTopic.published.at(-1), {
  name: TOPICS.control.light,
  messageType: "std_msgs/Bool",
  message: { data: true },
});
