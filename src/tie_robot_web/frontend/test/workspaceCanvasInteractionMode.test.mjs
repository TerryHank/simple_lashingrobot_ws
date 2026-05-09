import assert from "node:assert/strict";

import { WorkspaceCanvasView } from "../src/views/WorkspaceCanvasView.js";

function createContext() {
  return {
    clearRect() {},
    fillRect() {},
    putImageData() {},
    save() {},
    restore() {},
    beginPath() {},
    closePath() {},
    moveTo() {},
    lineTo() {},
    stroke() {},
    fill() {},
    arc() {},
    fillText() {},
    measureText: (text) => ({ width: String(text).length * 7 }),
    setLineDash() {},
  };
}

function createCanvas(rect = { left: 0, top: 0, width: 640, height: 480 }) {
  return {
    width: 640,
    height: 480,
    style: {},
    addEventListener() {},
    getBoundingClientRect: () => rect,
    getContext: () => createContext(),
  };
}

let lastHoverPixel = "unset";
const selectedSnapshots = [];
const view = new WorkspaceCanvasView({
  canvas: createCanvas(),
  overlayCanvas: createCanvas(),
  onSelectionChanged: (points) => selectedSnapshots.push(points),
  onHoverPixelChanged: (pixel) => {
    lastHoverPixel = pixel;
  },
});
view.lastImageMessage = { width: 640, height: 480 };
view.draw = () => {};
view.drawOverlay = () => {};

view.setWorkspacePickingEnabled(false);
view.handleCanvasClick({ clientX: 64, clientY: 48 });
assert.deepEqual(view.getSelectedPoints(), []);
assert.deepEqual(selectedSnapshots, []);

view.setOverlayEnabled(false);
view.setWorkspacePickingEnabled(true);
view.handleCanvasClick({ clientX: 64, clientY: 48 });
assert.deepEqual(view.getSelectedPoints(), []);
assert.deepEqual(selectedSnapshots, []);
view.setOverlayEnabled(true);
view.setWorkspacePickingEnabled(false);

view.handlePointerMove({ clientX: 32, clientY: 24 });
assert.deepEqual(lastHoverPixel, { x: 32, y: 24 });

view.setWorkspacePickingEnabled(true);
view.handlePointerMove({ clientX: 96, clientY: 72 });
assert.deepEqual(lastHoverPixel, null);

view.handleCanvasClick({ clientX: 64, clientY: 48 });
assert.deepEqual(view.getSelectedPoints(), [{ x: 64, y: 48 }]);
assert.deepEqual(selectedSnapshots, [[{ x: 64, y: 48 }]]);

const letterboxedSnapshots = [];
const letterboxedView = new WorkspaceCanvasView({
  canvas: createCanvas({ left: 0, top: 0, width: 1000, height: 480 }),
  overlayCanvas: createCanvas({ left: 0, top: 0, width: 1000, height: 480 }),
  onSelectionChanged: (points) => letterboxedSnapshots.push(points),
});
letterboxedView.lastImageMessage = { width: 640, height: 480 };
letterboxedView.draw = () => {};
letterboxedView.drawOverlay = () => {};
letterboxedView.setWorkspacePickingEnabled(true);

letterboxedView.handleCanvasClick({ clientX: 180, clientY: 0 });
assert.deepEqual(letterboxedView.getSelectedPoints(), [{ x: 0, y: 0 }]);

letterboxedView.handlePointerDown({ clientX: 184, clientY: 4, pointerId: 1 });
letterboxedView.handlePointerMove({ clientX: 280, clientY: 40 });
letterboxedView.handlePointerUp();
assert.deepEqual(letterboxedView.getSelectedPoints(), [{ x: 100, y: 40 }]);
assert.deepEqual(letterboxedSnapshots.at(-1), [{ x: 100, y: 40 }]);
