import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import { ROSLIB } from "../src/vendor/roslib.js";
import { RosConnectionController } from "../src/controllers/RosConnectionController.js";
import { MESSAGE_TYPES, SERVICE_TYPES, SERVICES, TOPICS } from "../src/config/topicRegistry.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

global.window = {
  location: {
    hostname: "127.0.0.1",
    pathname: "/index.html",
    protocol: "http:",
  },
  setTimeout() {
    return 1;
  },
  clearTimeout() {},
  setInterval() {
    return 1;
  },
  clearInterval() {},
};

let nowMs = 1000;
global.performance = {
  now() {
    const current = nowMs;
    nowMs += 123.4;
    return current;
  },
};

class FakeRos {
  constructor() {
    this.isConnected = true;
  }

  on() {}
}

class FakeTopic {
  constructor(options) {
    Object.assign(this, options);
    this.published = [];
  }

  advertise() {}

  publish(message) {
    this.published.push(message);
  }

  subscribe() {}

  unsubscribe() {}
}

class FakeService {
  constructor(options) {
    Object.assign(this, options);
    this.calls = [];
  }

  callService(request, success) {
    this.calls.push(request);
    success({
      success: true,
      message: "点位已满足2帧稳定",
      count: 8,
    });
  }
}

class FakeActionClient {
  constructor(options) {
    Object.assign(this, options);
  }
}

ROSLIB.Ros = FakeRos;
ROSLIB.Topic = FakeTopic;
ROSLIB.Service = FakeService;
ROSLIB.ActionClient = FakeActionClient;
ROSLIB.Message = class {
  constructor(payload) {
    Object.assign(this, payload);
  }
};
ROSLIB.ServiceRequest = class {
  constructor(payload) {
    Object.assign(this, payload);
  }
};

assert.equal(MESSAGE_TYPES.int32, "std_msgs/Int32");
assert.equal(SERVICES.algorithm.processImage, "/pointAI/process_image");
assert.equal(SERVICE_TYPES.algorithm.processImage, "tie_robot_msgs/ProcessImage");
assert.equal(TOPICS.algorithm.setStableFrameCount, "/web/pointAI/set_stable_frame_count");

const controller = new RosConnectionController();
controller.ros = new FakeRos();
controller.resources = controller.buildResources(controller.ros);

const frameCountResult = controller.publishStableFrameCount(2);
assert.equal(frameCountResult.success, true);
assert.equal(controller.resources.stableFrameCountPublisher.published.at(-1).data, 2);

const serviceResult = await controller.callProcessImageService({ requestMode: 1 });
assert.equal(serviceResult.success, true);
assert.equal(serviceResult.count, 8);
assert.equal(serviceResult.serviceElapsedMs, 123.4);
assert.equal(controller.resources.processImageService.calls.at(-1).request_mode, 1);

const uiControllerText = readFileSync(resolve(frontendRoot, "src/ui/UIController.js"), "utf-8");
assert.match(uiControllerText, /id: "visualDebug", label: "视觉调试"/);
assert.match(uiControllerText, /id="visualDebugTrigger"/);
assert.match(uiControllerText, /id="visualDebugStableFrameCount"/);
assert.match(uiControllerText, /id="visualDebugTimingSummary"/);
