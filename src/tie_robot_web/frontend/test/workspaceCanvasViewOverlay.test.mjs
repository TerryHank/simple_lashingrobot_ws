import assert from "node:assert/strict";

import { WorkspaceCanvasView } from "../src/views/WorkspaceCanvasView.js";

function createRecordingContext() {
  const calls = [];
  const context = {
    calls,
    beginPath: () => calls.push("beginPath"),
    clearRect: () => calls.push("clearRect"),
    closePath: () => calls.push("closePath"),
    fill: () => calls.push("fill"),
    lineTo: () => calls.push("lineTo"),
    moveTo: () => calls.push("moveTo"),
    restore: () => calls.push("restore"),
    save: () => calls.push("save"),
    setLineDash: () => calls.push("setLineDash"),
    stroke: () => calls.push("stroke"),
    arc: () => calls.push("arc"),
  };
  return context;
}

function createCanvas(context) {
  return {
    width: 640,
    height: 480,
    style: {},
    addEventListener: () => {},
    getContext: () => context,
  };
}

const imageContext = createRecordingContext();
const overlayContext = createRecordingContext();
const view = new WorkspaceCanvasView({
  canvas: createCanvas(imageContext),
  overlayCanvas: createCanvas(overlayContext),
});

view.setTcpWorkspaceBoundary({
  points: [
    { x: 40, y: 40 },
    { x: 600, y: 40 },
    { x: 600, y: 440 },
    { x: 40, y: 440 },
  ],
  sourceSize: { width: 640, height: 480 },
});

assert.equal(overlayContext.calls.includes("clearRect"), true);
assert.equal(overlayContext.calls.includes("stroke"), false);
assert.equal(overlayContext.calls.includes("fill"), false);
assert.equal(overlayContext.calls.includes("arc"), false);
