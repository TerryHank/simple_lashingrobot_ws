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

function createBoundaryLayer() {
  return {
    attributes: {},
    children: [],
    style: {},
    setAttribute(name, value) {
      this.attributes[name] = String(value);
    },
    removeAttribute(name) {
      delete this.attributes[name];
    },
    replaceChildren(...children) {
      this.children = children;
    },
    ownerDocument: {
      createElementNS(_namespace, tagName) {
        return {
          tagName,
          attributes: {},
          setAttribute(name, value) {
            this.attributes[name] = String(value);
          },
        };
      },
    },
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

const realtimeWorkspaceOverlayContext = createRecordingContext();
const realtimeWorkspaceBoundaryLayer = createBoundaryLayer();
const realtimeWorkspaceView = new WorkspaceCanvasView({
  canvas: createCanvas(createRecordingContext()),
  overlayCanvas: createCanvas(realtimeWorkspaceOverlayContext),
  baseBoundaryLayer: realtimeWorkspaceBoundaryLayer,
});
realtimeWorkspaceView.setOverlayEnabled(false);
realtimeWorkspaceView.setImageOverlayLayerState({ showLinearModuleBindRange: false });
realtimeWorkspaceOverlayContext.calls.length = 0;
realtimeWorkspaceView.setRealtimeWorkspaceBoundary({
  points: [
    { x: 30, y: 40 },
    { x: 210, y: 45 },
    { x: 205, y: 135 },
    { x: 28, y: 130 },
  ],
  sourceSize: { width: 640, height: 480 },
});

assert.equal(
  realtimeWorkspaceOverlayContext.calls.some(([name, text]) => name === "fillText" && text === "实时工作区范围"),
  false,
);
assert.equal(
  realtimeWorkspaceOverlayContext.calls.some(([name]) => name === "clearRect" || name === "stroke" || name === "arc"),
  false,
);
assert.equal(realtimeWorkspaceBoundaryLayer.children.length, 2);
assert.equal(realtimeWorkspaceBoundaryLayer.attributes.viewBox, "0 0 640 480");
assert.equal(realtimeWorkspaceBoundaryLayer.children[0].attributes.points, "30,40 210,45 205,135 28,130");
assert.equal(
  realtimeWorkspaceBoundaryLayer.children[0].attributes.class,
  "live-visible-area-polygon",
);
assert.equal(
  realtimeWorkspaceBoundaryLayer.children[1].attributes.class,
  "live-visible-area-corners",
);

realtimeWorkspaceView.setRealtimeWorkspaceBoundary({
  points: [
    { x: 0, y: 0 },
    { x: 639, y: 0 },
    { x: 639, y: 479 },
    { x: 0, y: 479 },
  ],
  sourceSize: { width: 640, height: 480 },
});
assert.equal(realtimeWorkspaceBoundaryLayer.children[0].attributes.points, "8,8 632,8 632,472 8,472");

realtimeWorkspaceView.setRealtimeWorkspaceBoundary(null);
assert.equal(realtimeWorkspaceBoundaryLayer.children.length, 0);
assert.equal(
  realtimeWorkspaceBoundaryLayer.attributes["data-visible"],
  "false",
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
  true,
);
