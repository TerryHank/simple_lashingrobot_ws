import assert from "node:assert/strict";

import * as THREE from "three";
import { Scene3DView } from "../src/views/Scene3DView.js";

function makeSceneView() {
  const sceneView = Object.create(Scene3DView.prototype);
  sceneView.viewMode = "free";
  sceneView.layerState = { showRobot: true, showAxes: true, tfAxisFrameVisibility: {} };
  sceneView.transformMap = new Map([
    ["base_link", {
      parentFrame: "map",
      position: new THREE.Vector3(0.5, 0.25, 0),
      quaternion: new THREE.Quaternion(),
    }],
    ["Scepter_depth_frame", {
      parentFrame: "base_link",
      position: new THREE.Vector3(0, 0, 0.46),
      quaternion: new THREE.Quaternion(),
    }],
  ]);
  sceneView.cachedWorldTransforms = new Map();
  sceneView.camera = new THREE.PerspectiveCamera();
  sceneView.camera.up.set(0, 0, 1);
  sceneView.controls = {
    enabled: true,
    enablePan: true,
    enableRotate: true,
    target: new THREE.Vector3(),
  };
  sceneView.baseLinkFrame = new THREE.Group();
  sceneView.scepterFrame = new THREE.Group();
  sceneView.gripperFrame = new THREE.Group();
  sceneView.robotGroup = new THREE.Group();
  sceneView.tcpToolGroup = new THREE.Group();
  return sceneView;
}

function assertVectorClose(actual, expected) {
  assert.equal(Math.abs(actual.x - expected.x) < 1e-9, true);
  assert.equal(Math.abs(actual.y - expected.y) < 1e-9, true);
  assert.equal(Math.abs(actual.z - expected.z) < 1e-9, true);
}

const cameraSceneView = makeSceneView();
cameraSceneView.setViewMode("camera");

assert.equal(cameraSceneView.robotGroup.visible, false);
assert.equal(cameraSceneView.controls.enabled, false);
assert.equal(cameraSceneView.controls.enablePan, false);
assert.equal(cameraSceneView.controls.enableRotate, false);

cameraSceneView.setViewMode("top");
cameraSceneView.resetView("top");

assert.equal(cameraSceneView.robotGroup.visible, true);
assert.equal(cameraSceneView.controls.enabled, true);
assert.equal(cameraSceneView.controls.enablePan, true);
assert.equal(cameraSceneView.controls.enableRotate, false);
assertVectorClose(
  cameraSceneView.controls.target.clone().sub(cameraSceneView.camera.position).normalize(),
  new THREE.Vector3(0, 0, -1),
);

cameraSceneView.setViewMode("free");

assert.equal(cameraSceneView.robotGroup.visible, true);
assert.equal(cameraSceneView.controls.enabled, true);
assert.equal(cameraSceneView.controls.enablePan, true);
assert.equal(cameraSceneView.controls.enableRotate, true);
