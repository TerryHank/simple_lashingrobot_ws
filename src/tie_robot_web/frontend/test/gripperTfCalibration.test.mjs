import assert from "node:assert/strict";

import { ROSLIB } from "../src/vendor/roslib.js";
import { RosConnectionController } from "../src/controllers/RosConnectionController.js";
import { MESSAGE_TYPES, SERVICE_TYPES, SERVICES, TOPICS } from "../src/config/topicRegistry.js";
import { Scene3DView } from "../src/views/Scene3DView.js";

global.window = {
  location: {
    hostname: "127.0.0.1",
    pathname: "/index.html",
    protocol: "http:",
  },
  setTimeout(callback) {
    return setTimeout(callback, 0);
  },
  clearTimeout(timerId) {
    clearTimeout(timerId);
  },
  setInterval() {
    return 1;
  },
  clearInterval() {},
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
    this.messages = [];
    this.advertised = false;
  }

  advertise() {
    this.advertised = true;
  }

  publish(message) {
    this.messages.push(message);
  }

  subscribe() {}

  unsubscribe() {}
}

class FakeService {
  static failNextCall = false;

  constructor(options) {
    Object.assign(this, options);
    this.calls = [];
  }

  callService(request, success, failure) {
    this.calls.push(request);
    if (FakeService.failNextCall) {
      failure({ message: "Service /web/tf/set_gripper_tf_calibration does not exist" });
      return;
    }
    success({
      success: true,
      message: "service updated",
      applied_x_mm: request.x_mm,
      applied_y_mm: request.y_mm,
      applied_z_mm: request.z_mm,
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

assert.equal(SERVICES.tf.setGripperTfCalibration, "/web/tf/set_gripper_tf_calibration");
assert.equal(SERVICE_TYPES.tf.setGripperTfCalibration, "tie_robot_msgs/SetGripperTfCalibration");
assert.equal(TOPICS.tf.setCameraTcpExtrinsic, "/web/tf/set_camera_tcp_extrinsic");

const controller = new RosConnectionController();
controller.ros = new FakeRos();
controller.resources = controller.buildResources(controller.ros);

assert.equal(controller.resources.setCameraTcpExtrinsicPublisher.name, TOPICS.tf.setCameraTcpExtrinsic);
assert.equal(controller.resources.setCameraTcpExtrinsicPublisher.messageType, MESSAGE_TYPES.pose);

const serviceResult = await controller.callGripperTfCalibrationService({ x: 301, y: 92, z: 728 });
assert.equal(serviceResult.success, true);
assert.equal(serviceResult.fallback, false);
assert.equal(controller.resources.setGripperTfCalibrationService.calls.at(-1).x_mm, 301);
assert.equal(controller.resources.setCameraTcpExtrinsicPublisher.messages.length, 0);

FakeService.failNextCall = true;
const fallbackResult = await controller.callGripperTfCalibrationService({ x: 302, y: 93, z: 729 });
assert.equal(fallbackResult.success, true);
assert.equal(fallbackResult.fallback, "topic");
assert.deepEqual(fallbackResult.applied, { x: 302, y: 93, z: 729 });
assert.equal(controller.resources.setCameraTcpExtrinsicPublisher.advertised, true);
assert.deepEqual(controller.resources.setCameraTcpExtrinsicPublisher.messages.at(-1).position, {
  x: 302,
  y: 93,
  z: 729,
});
assert.deepEqual(controller.resources.setCameraTcpExtrinsicPublisher.messages.at(-1).orientation, {
  x: 0,
  y: 0,
  z: 0,
  w: 1,
});

let frameTransformsApplied = false;
const sceneView = Object.create(Scene3DView.prototype);
sceneView.transformMap = new Map();
sceneView.cachedWorldTransforms = new Map();
sceneView.applyFrameTransforms = () => {
  frameTransformsApplied = true;
};
const optimisticCalibration = sceneView.applyCameraToTcpCalibration({
  translationMm: { x: 305, y: 96, z: 731 },
});

assert.equal(frameTransformsApplied, true);
assert.deepEqual(optimisticCalibration.translationMm, { x: 305, y: 96, z: 731 });
assert.equal(sceneView.transformMap.get("gripper_frame").parentFrame, "Scepter_depth_frame");
assert.equal(sceneView.transformMap.get("gripper_frame").position.z, 0.731);
