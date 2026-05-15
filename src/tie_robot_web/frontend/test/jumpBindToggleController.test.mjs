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

const ros = {};
const resultMessages = [];
const logs = [];
const controller = new LegacyCommandController({
  rosConnection: {
    getResources: () => ({ ros }),
  },
  callbacks: {
    onResultMessage: (message) => resultMessages.push(message),
    onLog: (message, level) => logs.push({ message, level }),
  },
});

const initialState = controller.getToggleStateSnapshot().jumpBindEnabled;
assert.equal(initialState.value, false);
assert.equal(initialState.selectedParity, 0);
assert.equal(initialState.selectedColor, "black");
assert.equal(initialState.label, "长按开启跳绑黑棋");

const initialClassificationState = controller.getToggleStateSnapshot().bindClassificationEnabled;
assert.equal(initialClassificationState.value, false);
assert.equal(initialClassificationState.label, "开启分类");

const classificationEnabledState = controller.handleToggle("bindClassificationEnabled", {});
assert.equal(classificationEnabledState.value, true);
assert.equal(classificationEnabledState.label, "关闭分类");
assert.deepEqual(FakeTopic.published.at(-1), {
  name: TOPICS.algorithm.setBindClassificationEnabled,
  messageType: "std_msgs/Bool",
  message: { data: true },
});

const classificationDisabledState = controller.handleToggle("bindClassificationEnabled", {});
assert.equal(classificationDisabledState.value, false);
assert.equal(classificationDisabledState.label, "开启分类");
assert.deepEqual(FakeTopic.published.at(-1), {
  name: TOPICS.algorithm.setBindClassificationEnabled,
  messageType: "std_msgs/Bool",
  message: { data: false },
});

const whiteState = controller.handleToggle("jumpBindEnabled", {});
assert.equal(whiteState.value, false);
assert.equal(whiteState.selectedParity, 1);
assert.equal(whiteState.selectedColor, "white");
assert.equal(whiteState.label, "长按开启跳绑白棋");
assert.deepEqual(FakeTopic.published.at(-1), {
  name: TOPICS.control.jumpBindParity,
  messageType: "std_msgs/Int32",
  message: { data: 1 },
});

const enabledWhiteState = controller.handleToggleLongPress("jumpBindEnabled", {});
assert.equal(enabledWhiteState.value, true);
assert.equal(enabledWhiteState.selectedParity, 1);
assert.equal(enabledWhiteState.label, "长按关闭跳绑白棋");
assert.deepEqual(FakeTopic.published.slice(-2), [
  {
    name: TOPICS.control.jumpBindParity,
    messageType: "std_msgs/Int32",
    message: { data: 1 },
  },
  {
    name: TOPICS.control.jumpBindEnabled,
    messageType: "std_msgs/Bool",
    message: { data: true },
  },
]);

const enabledBlackState = controller.handleToggle("jumpBindEnabled", {});
assert.equal(enabledBlackState.value, true);
assert.equal(enabledBlackState.selectedParity, 0);
assert.equal(enabledBlackState.selectedColor, "black");
assert.equal(enabledBlackState.label, "长按关闭跳绑黑棋");
assert.deepEqual(FakeTopic.published.at(-1), {
  name: TOPICS.control.jumpBindParity,
  messageType: "std_msgs/Int32",
  message: { data: 0 },
});

const disabledBlackState = controller.handleToggleLongPress("jumpBindEnabled", {});
assert.equal(disabledBlackState.value, false);
assert.equal(disabledBlackState.selectedParity, 0);
assert.equal(disabledBlackState.label, "长按开启跳绑黑棋");
assert.deepEqual(FakeTopic.published.slice(-2), [
  {
    name: TOPICS.control.jumpBindParity,
    messageType: "std_msgs/Int32",
    message: { data: 0 },
  },
  {
    name: TOPICS.control.jumpBindEnabled,
    messageType: "std_msgs/Bool",
    message: { data: false },
  },
]);

assert.match(resultMessages.at(-1), /长按开启跳绑黑棋，已关闭/);
assert.match(logs.at(-1).message, /状态=已关闭/);
