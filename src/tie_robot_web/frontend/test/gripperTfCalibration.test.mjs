import assert from "node:assert/strict";

import * as THREE from "three";
import { ROSLIB } from "../src/vendor/roslib.js";
import { RosConnectionController } from "../src/controllers/RosConnectionController.js";
import { MESSAGE_TYPES, SERVICE_TYPES, SERVICES, TOPICS } from "../src/config/topicRegistry.js";
import { Scene3DView } from "../src/views/Scene3DView.js";
import { UIController } from "../src/ui/UIController.js";

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

global.document = {
  activeElement: null,
};

function makeFakeElement(initialValue = "") {
  const listeners = new Map();
  return {
    value: initialValue,
    textContent: "",
    addEventListener(eventName, listener) {
      const eventListeners = listeners.get(eventName) || [];
      eventListeners.push(listener);
      listeners.set(eventName, eventListeners);
    },
    dispatch(eventName, event = {}) {
      (listeners.get(eventName) || []).forEach((listener) => listener(event));
    },
  };
}

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

const movingSceneView = Object.create(Scene3DView.prototype);
movingSceneView.transformMap = new Map([
  ["Scepter_depth_frame", {
    parentFrame: "map",
    position: new THREE.Vector3(0, 0, 0),
    quaternion: new THREE.Quaternion(),
  }],
  ["gripper_frame", {
    parentFrame: "Scepter_depth_frame",
    position: new THREE.Vector3(0.285, 0.070, 0.740),
    quaternion: new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, Math.PI)),
  }],
]);
movingSceneView.cachedWorldTransforms = new Map();
let movingTcpFrameRefreshRequested = false;
movingSceneView.applyFrameTransforms = () => {
  movingTcpFrameRefreshRequested = true;
};

const movingTcpPosition = movingSceneView.setLinearModuleLocalPosition({ x: 10, y: 20, z: 30 });

assert.equal(movingTcpFrameRefreshRequested, true);
assert.deepEqual(
  {
    x: Number(movingTcpPosition.x.toFixed(1)),
    y: Number(movingTcpPosition.y.toFixed(1)),
    z: Number(movingTcpPosition.z.toFixed(1)),
  },
  { x: 275.0, y: 50.0, z: 770.0 },
);

const gripperTfX = makeFakeElement();
const gripperTfY = makeFakeElement();
const gripperTfZ = makeFakeElement();
const applyButton = makeFakeElement();
const uiController = Object.create(UIController.prototype);
uiController.refs = {
  gripperTfCurrent: makeFakeElement(),
  gripperTfX,
  gripperTfY,
  gripperTfZ,
  applyGripperTfCalibration: applyButton,
};

const oldCalibration = {
  parentFrame: "Scepter_depth_frame",
  childFrame: "gripper_frame",
  translationMm: { x: 301, y: 92, z: 728 },
};
uiController.setGripperTfCalibration(oldCalibration, { forceInputs: true });
gripperTfX.value = "305.0";
global.document.activeElement = applyButton;
uiController.setGripperTfCalibration(oldCalibration);

let appliedPayload = null;
uiController.onCalibrationApply((payload) => {
  appliedPayload = payload;
});
applyButton.dispatch("click");

assert.equal(gripperTfX.value, "305.0");
assert.deepEqual(appliedPayload, { x: 305, y: 92, z: 728 });
