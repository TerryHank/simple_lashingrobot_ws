import assert from "node:assert/strict";

import * as THREE from "three";
import { RosConnectionController } from "../src/controllers/RosConnectionController.js";
import { sampleFloat32XYZImagePixel } from "../src/utils/irImageUtils.js";
import { Scene3DView } from "../src/views/Scene3DView.js";

function makeFloat32Image(width, height, valuesMm) {
  const array = new Float32Array(width * height * 3);
  valuesMm.forEach((value, index) => {
    array[index] = value;
  });
  return {
    width,
    height,
    step: width * 12,
    encoding: "32FC3",
    data: new Uint8Array(array.buffer),
  };
}

const imageMessage = makeFloat32Image(2, 2, [
  0, 0, 0,
  100, 200, 300,
  400, 500, 600,
  Number.NaN, 1, 2,
]);

assert.deepEqual(sampleFloat32XYZImagePixel(imageMessage, { x: 1, y: 0 }), {
  x: 100,
  y: 200,
  z: 300,
  pixel: { x: 1, y: 0 },
});
assert.equal(sampleFloat32XYZImagePixel(imageMessage, { x: 0, y: 0 }), null);
assert.equal(sampleFloat32XYZImagePixel(imageMessage, { x: 1, y: 1 }), null);

const sceneView = Object.create(Scene3DView.prototype);
sceneView.transformMap = new Map([
  ["Scepter_depth_frame", {
    parentFrame: "map",
    position: new THREE.Vector3(1, 2, 0.5),
    quaternion: new THREE.Quaternion(),
  }],
  ["gripper_frame", {
    parentFrame: "map",
    position: new THREE.Vector3(1.1, 2, 1.5),
    quaternion: new THREE.Quaternion(),
  }],
]);
sceneView.cachedWorldTransforms = new Map();

assert.deepEqual(
  sceneView.convertScepterPointMmToFrameMm({ x: 100, y: 0, z: 1000 }, "map"),
  { x: 1100, y: 2000, z: 1500 },
);
assert.deepEqual(
  sceneView.convertScepterPointMmToFrameMm({ x: 100, y: 0, z: 1000 }, "gripper_frame"),
  { x: 0, y: 0, z: 0 },
);
assert.deepEqual(
  sceneView.convertScepterPointMmToFrameMm({ x: 100, y: 0, z: 1000 }, "Scepter_depth_frame"),
  { x: 100, y: 0, z: 1000 },
);

const hoverRosController = new RosConnectionController();
hoverRosController.ros = { isConnected: true };
let hoverTopicOptions = null;
hoverRosController.buildTopic = (_name, _type, options) => {
  hoverTopicOptions = options;
  return {
    subscribe() {},
    unsubscribe() {},
  };
};
hoverRosController.updateImageHoverCoordinateSubscription({ enabled: true });
assert.equal(hoverTopicOptions.throttle_rate, 1000);
