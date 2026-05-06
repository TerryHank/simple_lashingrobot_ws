import assert from "node:assert/strict";

import * as THREE from "three";
import { Scene3DView } from "../src/views/Scene3DView.js";

function makeSceneViewForWorkspaceRange() {
  const view = Object.create(Scene3DView.prototype);
  view.transformMap = new Map([
    ["Scepter_depth_frame", {
      parentFrame: "map",
      position: new THREE.Vector3(1, 2, 0.5),
      quaternion: new THREE.Quaternion(),
    }],
  ]);
  view.cachedWorldTransforms = new Map();
  view.workspaceRangeFrame = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
  view.workspaceRangeCorners = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial());
  view.workspaceRangeGroup = new THREE.Group();
  view.workspaceRangeGroup.add(view.workspaceRangeFrame, view.workspaceRangeCorners);
  return view;
}

function roundImagePoints(points) {
  return points.map((point) => ({
    x: Number(point.x.toFixed(3)),
    y: Number(point.y.toFixed(3)),
    inside: point.inside,
  }));
}

const sceneView = makeSceneViewForWorkspaceRange();
const count = sceneView.setRealtimeWorkspaceRangeMessage({
  count: 4,
  PointCoordinatesArray: [
    { idx: 1, Pix_coord: [10, 20], World_coord: [0, 0, 1000] },
    { idx: 2, Pix_coord: [110, 20], World_coord: [100, 0, 1000] },
    { idx: 3, Pix_coord: [110, 120], World_coord: [100, 100, 1000] },
    { idx: 4, Pix_coord: [10, 120], World_coord: [0, 100, 1000] },
  ],
});

assert.equal(count, 4);
assert.equal(sceneView.workspaceRangeGroup.visible, false);

const sceneFrameAttribute = sceneView.workspaceRangeFrame.geometry.getAttribute("position");
assert.equal(sceneFrameAttribute ? sceneFrameAttribute.array.length : 0, 0);

const mapPositions = Array.from(sceneView.getRealtimeWorkspaceRangeMapPositions())
  .map((value) => Number(value.toFixed(3)));
assert.deepEqual(mapPositions, [
  1, 2, 1.5, 1.1, 2, 1.5,
  1.1, 2.1, 1.5,
  1, 2.1, 1.5,
]);

sceneView.transformMap.set("Scepter_depth_frame", {
  parentFrame: "map",
  position: new THREE.Vector3(3, 4, 0.5),
  quaternion: new THREE.Quaternion(),
});
sceneView.cachedWorldTransforms.clear();
sceneView.refreshRealtimeWorkspaceRangeWorldPositions();
assert.deepEqual(
  Array.from(sceneView.getRealtimeWorkspaceRangeMapPositions())
    .map((value) => Number(value.toFixed(3))),
  mapPositions,
);
assert.equal(sceneView.workspaceRangeGroup.visible, false);

const imageProjectionSceneView = makeSceneViewForWorkspaceRange();
imageProjectionSceneView.setRealtimeWorkspaceRangeMessage({
  count: 4,
  PointCoordinatesArray: [
    { World_coord: [0, 0, 1000] },
    { World_coord: [100, 0, 1000] },
    { World_coord: [100, 100, 1000] },
    { World_coord: [0, 100, 1000] },
  ],
});
const cameraInfo = {
  header: { frame_id: "Scepter_depth_frame" },
  width: 100,
  height: 100,
  K: [100, 0, 50, 0, 100, 50, 0, 0, 1],
};
assert.deepEqual(roundImagePoints(imageProjectionSceneView.projectRealtimeWorkspaceRangeToImage(cameraInfo).points), [
  { x: 50, y: 50, inside: true },
  { x: 60, y: 50, inside: true },
  { x: 60, y: 60, inside: true },
  { x: 50, y: 60, inside: true },
]);

imageProjectionSceneView.transformMap.set("Scepter_depth_frame", {
  parentFrame: "map",
  position: new THREE.Vector3(1.1, 2, 0.5),
  quaternion: new THREE.Quaternion(),
});
imageProjectionSceneView.cachedWorldTransforms.clear();
assert.deepEqual(roundImagePoints(imageProjectionSceneView.projectRealtimeWorkspaceRangeToImage(cameraInfo).points), [
  { x: 40, y: 50, inside: true },
  { x: 50, y: 50, inside: true },
  { x: 50, y: 60, inside: true },
  { x: 40, y: 60, inside: true },
]);
