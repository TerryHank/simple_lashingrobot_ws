import assert from "node:assert/strict";

import {
  TCP_WORKSPACE_BOUNDARY_MM,
  buildTcpWorkspaceBoundaryPlanesMm,
  buildTcpWorkspaceBoundaryPointsMm,
  buildTcpWorkspaceBoundaryGripperPointsMm,
  buildTcpWorkspaceBoundaryGripperPlanesMm,
  normalizeTcpWorkspaceBoundaryMm,
  normalizeCameraProjection,
  projectCameraPointMetersToImagePixel,
} from "../src/utils/tcpWorkspaceOverlay.js";

assert.deepEqual(TCP_WORKSPACE_BOUNDARY_MM, {
  x: { min: 0, max: 380 },
  y: { min: 0, max: 330 },
  z: { min: 0, max: 160 },
});

assert.deepEqual(buildTcpWorkspaceBoundaryPointsMm(), [
  { x: 0, y: 0, z: 0 },
  { x: 380, y: 0, z: 0 },
  { x: 380, y: 330, z: 0 },
  { x: 0, y: 330, z: 0 },
]);

assert.deepEqual(buildTcpWorkspaceBoundaryGripperPointsMm(), [
  { x: 0, y: 0, z: 0 },
  { x: 380, y: 0, z: 0 },
  { x: 380, y: 330, z: 0 },
  { x: 0, y: 330, z: 0 },
]);

assert.deepEqual(buildTcpWorkspaceBoundaryPointsMm(undefined, 160), [
  { x: 0, y: 0, z: 160 },
  { x: 380, y: 0, z: 160 },
  { x: 380, y: 330, z: 160 },
  { x: 0, y: 330, z: 160 },
]);

assert.deepEqual(buildTcpWorkspaceBoundaryPlanesMm().map((plane) => plane.z), [0, 160]);
assert.deepEqual(buildTcpWorkspaceBoundaryPlanesMm()[1].points, [
  { x: 0, y: 0, z: 160 },
  { x: 380, y: 0, z: 160 },
  { x: 380, y: 330, z: 160 },
  { x: 0, y: 330, z: 160 },
]);
assert.deepEqual(buildTcpWorkspaceBoundaryGripperPlanesMm()[1].points, [
  { x: 0, y: 0, z: 160 },
  { x: 380, y: 0, z: 160 },
  { x: 380, y: 330, z: 160 },
  { x: 0, y: 330, z: 160 },
]);

assert.deepEqual(
  normalizeTcpWorkspaceBoundaryMm({
    x: { min: 120, max: 20 },
    y: { min: 15, max: 215 },
    z: { min: 8, max: 88 },
  }),
  {
    x: { min: 20, max: 120 },
    y: { min: 15, max: 215 },
    z: { min: 8, max: 88 },
  },
);

assert.deepEqual(
  buildTcpWorkspaceBoundaryPlanesMm({
    x: { min: 20, max: 120 },
    y: { min: 15, max: 215 },
    z: { min: 8, max: 88 },
  })[1].points,
  [
    { x: 20, y: 15, z: 88 },
    { x: 120, y: 15, z: 88 },
    { x: 120, y: 215, z: 88 },
    { x: 20, y: 215, z: 88 },
  ],
);

const cameraInfo = {
  width: 640,
  height: 480,
  K: [500, 0, 320, 0, 500, 240, 0, 0, 1],
  D: [0, 0, 0, 0, 0],
};

assert.deepEqual(normalizeCameraProjection(cameraInfo), {
  width: 640,
  height: 480,
  fx: 500,
  fy: 500,
  cx: 320,
  cy: 240,
});

assert.deepEqual(
  projectCameraPointMetersToImagePixel({ x: 0, y: 0, z: 1 }, cameraInfo),
  { x: 320, y: 240, inside: true },
);

assert.deepEqual(
  projectCameraPointMetersToImagePixel({ x: 0.1, y: -0.05, z: 1 }, cameraInfo),
  { x: 370, y: 215, inside: true },
);

const distortedCameraInfo = {
  width: 640,
  height: 480,
  K: [100, 0, 0, 0, 100, 0, 0, 0, 1],
  D: [-0.5, 0, 0, 0, 0],
};

assert.deepEqual(
  projectCameraPointMetersToImagePixel({ x: 0.5, y: 0, z: 1 }, distortedCameraInfo),
  { x: 50, y: 0, inside: true },
);

assert.deepEqual(
  normalizeCameraProjection(distortedCameraInfo),
  { width: 640, height: 480, fx: 100, fy: 100, cx: 0, cy: 0 },
);

assert.equal(projectCameraPointMetersToImagePixel({ x: 0, y: 0, z: 0 }, cameraInfo), null);
