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
    position: new THREE.Vector3(0.285, -0.310, 0.740),
    quaternion: new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, Math.PI / 2)),
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
  { x: 265.0, y: -300.0, z: 770.0 },
);

const movingAxisSceneView = Object.create(Scene3DView.prototype);
movingAxisSceneView.transformMap = new Map([
  ["Scepter_depth_frame", {
    parentFrame: "map",
    position: new THREE.Vector3(0, 0, 0),
    quaternion: new THREE.Quaternion(),
  }],
  ["gripper_frame", {
    parentFrame: "Scepter_depth_frame",
    position: new THREE.Vector3(0.285, -0.310, 0.740),
    quaternion: new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, Math.PI / 2)),
  }],
]);
movingAxisSceneView.cachedWorldTransforms = new Map();
movingAxisSceneView.layerState = { showRobot: true, showAxes: true, tfAxisFrameVisibility: {} };
movingAxisSceneView.baseLinkFrame = new THREE.Group();
movingAxisSceneView.scepterFrame = new THREE.Group();
movingAxisSceneView.gripperFrame = new THREE.Group();
movingAxisSceneView.robotGroup = new THREE.Group();
movingAxisSceneView.tcpToolGroup = new THREE.Group();

movingAxisSceneView.setLinearModuleLocalPosition({ x: 10, y: 20, z: 30 });

assert.deepEqual(
  movingAxisSceneView.gripperFrame.position.toArray().map((value) => Number(value.toFixed(3))),
  movingAxisSceneView.tcpToolGroup.position.toArray().map((value) => Number(value.toFixed(3))),
);

const workspaceProjectionSceneView = Object.create(Scene3DView.prototype);
workspaceProjectionSceneView.transformMap = new Map([
  ["Scepter_depth_frame", {
    parentFrame: "map",
    position: new THREE.Vector3(0, 0, 0),
    quaternion: new THREE.Quaternion(),
  }],
  ["gripper_frame", {
    parentFrame: "Scepter_depth_frame",
    position: new THREE.Vector3(0.285, -0.310, 0.740),
    quaternion: new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, Math.PI / 2)),
  }],
]);
workspaceProjectionSceneView.cachedWorldTransforms = new Map();

const projectedTcpWorkspace = workspaceProjectionSceneView.projectTcpWorkspaceBoundaryToImage({
  header: { frame_id: "Scepter_depth_frame" },
  width: 640,
  height: 480,
  K: [500, 0, 320, 0, 500, 240, 0, 0, 1],
});

assert.equal(projectedTcpWorkspace.points[1].y > projectedTcpWorkspace.points[0].y, true);
assert.equal(projectedTcpWorkspace.points[3].x < projectedTcpWorkspace.points[0].x, true);

workspaceProjectionSceneView.setLinearModuleBindRange({
  x: { min: 20, max: 120 },
  y: { min: 30, max: 230 },
  z: { min: 10, max: 90 },
});
const projectedCustomTcpWorkspace = workspaceProjectionSceneView.projectTcpWorkspaceBoundaryToImage({
  header: { frame_id: "Scepter_depth_frame" },
  width: 640,
  height: 480,
  K: [500, 0, 320, 0, 500, 240, 0, 0, 1],
});
assert.equal(projectedCustomTcpWorkspace.planes[0].z, 10);
assert.equal(projectedCustomTcpWorkspace.planes[1].z, 90);

const bindRangeSceneView = Object.create(Scene3DView.prototype);
bindRangeSceneView.linearModuleBindRangeMesh = new THREE.Mesh(
  new THREE.BoxGeometry(1, 1, 1),
  new THREE.MeshBasicMaterial(),
);
bindRangeSceneView.linearModuleBindRangeEdges = new THREE.LineSegments(
  new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial(),
);
let bindRangeFrameRefreshRequested = false;
bindRangeSceneView.applyFrameTransforms = () => {
  bindRangeFrameRefreshRequested = true;
};
const normalizedBindRange = bindRangeSceneView.setLinearModuleBindRange({
  x: { min: 120, max: 20 },
  y: { min: 30, max: 230 },
  z: { min: 10, max: 90 },
});
bindRangeSceneView.linearModuleBindRangeMesh.geometry.computeBoundingBox();
const bindRangeSize = new THREE.Vector3();
bindRangeSceneView.linearModuleBindRangeMesh.geometry.boundingBox.getSize(bindRangeSize);

assert.equal(bindRangeFrameRefreshRequested, true);
assert.deepEqual(normalizedBindRange, {
  x: { min: 20, max: 120 },
  y: { min: 30, max: 230 },
  z: { min: 10, max: 90 },
});
assert.deepEqual(
  bindRangeSceneView.linearModuleBindRangeMesh.position.toArray().map((value) => Number((value * 1000).toFixed(1))),
  [70, 130, 50],
);
assert.deepEqual(
  bindRangeSize.toArray().map((value) => Number((value * 1000).toFixed(1))),
  [100, 200, 80],
);

const bindRangeVisibilitySceneView = Object.create(Scene3DView.prototype);
bindRangeVisibilitySceneView.linearModuleBindRangeGroup = { visible: true };
bindRangeVisibilitySceneView.layerState = {
  showRobot: false,
  showLinearModuleBindRange: false,
};
let bindRangePoseApplied = false;
bindRangeVisibilitySceneView.applyTfFramePose = () => {
  bindRangePoseApplied = true;
};
bindRangeVisibilitySceneView.applyLinearModuleBindRangeTransform({ position: new THREE.Vector3() });
assert.equal(bindRangeVisibilitySceneView.linearModuleBindRangeGroup.visible, false);
assert.equal(bindRangePoseApplied, false);

bindRangeVisibilitySceneView.layerState.showLinearModuleBindRange = true;
bindRangeVisibilitySceneView.applyLinearModuleBindRangeTransform({ position: new THREE.Vector3() });
assert.equal(bindRangeVisibilitySceneView.linearModuleBindRangeGroup.visible, true);
assert.equal(bindRangePoseApplied, true);

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
assert.equal(
  uiController.refs.gripperTfCurrent.textContent,
  "Scepter_depth_frame -> gripper_frame | translation_mm=(301, 92, 728)",
);
assert.equal(gripperTfX.value, "301");
assert.equal(gripperTfY.value, "92");
assert.equal(gripperTfZ.value, "728");
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
