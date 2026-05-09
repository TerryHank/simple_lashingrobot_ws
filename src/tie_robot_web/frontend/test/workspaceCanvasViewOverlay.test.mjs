import assert from "node:assert/strict";

import { WorkspaceCanvasView } from "../src/views/WorkspaceCanvasView.js";

function createRecordingContext() {
  const calls = [];
  const context = {
    calls,
    beginPath: () => calls.push(["beginPath"]),
    clearRect: () => calls.push(["clearRect"]),
    closePath: () => calls.push(["closePath"]),
    fill: () => calls.push(["fill"]),
    fillRect: (...args) => calls.push(["fillRect", ...args]),
    fillText: (...args) => calls.push(["fillText", ...args]),
    lineTo: (...args) => calls.push(["lineTo", ...args]),
    measureText: (text) => ({ width: String(text).length * 7 }),
    moveTo: (...args) => calls.push(["moveTo", ...args]),
    restore: () => calls.push(["restore"]),
    save: () => calls.push(["save"]),
    setLineDash: (...args) => calls.push(["setLineDash", ...args]),
    stroke: () => calls.push(["stroke"]),
    arc: (...args) => calls.push(["arc", ...args]),
  };
  Object.defineProperties(context, {
    strokeStyle: {
      set: (value) => calls.push(["strokeStyle", value]),
    },
    fillStyle: {
      set: (value) => calls.push(["fillStyle", value]),
    },
    lineWidth: {
      set: (value) => calls.push(["lineWidth", value]),
    },
  });
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
  planes: [
    {
      z: 0,
      points: [
        { x: 40, y: 40 },
        { x: 600, y: 40 },
        { x: 600, y: 440 },
        { x: 40, y: 440 },
      ],
    },
    {
      z: 160,
      points: [
        { x: 70, y: 70 },
        { x: 570, y: 70 },
        { x: 570, y: 410 },
        { x: 70, y: 410 },
      ],
    },
  ],
  sourceSize: { width: 640, height: 480 },
});

assert.equal(overlayContext.calls.some(([name]) => name === "clearRect"), true);
assert.equal(overlayContext.calls.some(([name]) => name === "stroke"), true);
assert.equal(overlayContext.calls.some(([name]) => name === "arc"), true);
assert.deepEqual(
  overlayContext.calls.filter(([name]) => name === "moveTo" || name === "lineTo").slice(0, 4),
  [
    ["moveTo", 40, 40],
    ["lineTo", 600, 40],
    ["lineTo", 600, 440],
    ["lineTo", 40, 440],
  ],
);
assert.equal(
  overlayContext.calls.some(([name, value]) => (
    name === "strokeStyle" && String(value).includes("126, 220, 255")
  )),
  true,
);

const savedWorkspaceOverlayContext = createRecordingContext();
const savedWorkspaceView = new WorkspaceCanvasView({
  canvas: createCanvas(createRecordingContext()),
  overlayCanvas: createCanvas(savedWorkspaceOverlayContext),
});
savedWorkspaceView.setSavedWorkspacePayload([20, 30, 220, 30, 220, 130, 20, 130]);

assert.equal(
  savedWorkspaceOverlayContext.calls.some(([name, text]) => name === "fillText" && text === "实时工作区范围"),
  false,
);
assert.deepEqual(
  savedWorkspaceOverlayContext.calls.filter(([name]) => name === "moveTo" || name === "lineTo").slice(0, 4),
  [],
);

const selectedWorkspaceContext = createRecordingContext();
const selectedWorkspaceView = new WorkspaceCanvasView({
  canvas: createCanvas(selectedWorkspaceContext),
  overlayCanvas: createCanvas(createRecordingContext()),
});
selectedWorkspaceView.setSelectedWorkspacePayload([20, 30, 220, 30, 220, 130, 20, 130]);
selectedWorkspaceView.drawWorkspacePolylines();
assert.deepEqual(
  selectedWorkspaceContext.calls.filter(([name]) => name === "moveTo" || name === "lineTo").slice(0, 4),
  [],
);

selectedWorkspaceView.setWorkspacePickingEnabled(true);
selectedWorkspaceView.drawWorkspacePolylines();
assert.deepEqual(
  selectedWorkspaceContext.calls.filter(([name]) => name === "moveTo" || name === "lineTo").slice(0, 4),
  [
    ["moveTo", 20, 30],
    ["lineTo", 220, 30],
    ["lineTo", 220, 130],
    ["lineTo", 20, 130],
  ],
);

const incompatibleTopicWorkspaceContext = createRecordingContext();
const incompatibleTopicWorkspaceView = new WorkspaceCanvasView({
  canvas: createCanvas(incompatibleTopicWorkspaceContext),
  overlayCanvas: createCanvas(createRecordingContext()),
});
incompatibleTopicWorkspaceView.setSelectedWorkspacePayload([20, 30, 220, 30, 220, 130, 20, 130]);
incompatibleTopicWorkspaceView.setWorkspacePickingEnabled(true);
incompatibleTopicWorkspaceView.setOverlayEnabled(false);
incompatibleTopicWorkspaceView.drawWorkspacePolylines();
assert.deepEqual(
  incompatibleTopicWorkspaceContext.calls.filter(([name]) => name === "moveTo" || name === "lineTo").slice(0, 4),
  [],
);

const workspaceCanvasViewText = await import("node:fs").then(({ readFileSync }) => (
  readFileSync(new URL("../src/views/WorkspaceCanvasView.js", import.meta.url), "utf-8")
));
assert.equal(
  /drawOverlay\(\)\s*{[\s\S]*drawRealtimeWorkspaceBoundary\(/.test(workspaceCanvasViewText),
  false,
);
assert.equal(
  workspaceCanvasViewText.includes("renderRealtimeWorkspaceBoundaryLayer()"),
  false,
);
assert.equal(workspaceCanvasViewText.includes("setRealtimeWorkspaceBoundary("), false);
