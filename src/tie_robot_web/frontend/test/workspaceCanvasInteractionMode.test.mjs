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

function createCanvas() {
  return {
    width: 640,
    height: 480,
    style: {},
    addEventListener() {},
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 640, height: 480 }),
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

view.handlePointerMove({ clientX: 32, clientY: 24 });
assert.deepEqual(lastHoverPixel, { x: 32, y: 24 });

view.setWorkspacePickingEnabled(true);
view.handlePointerMove({ clientX: 96, clientY: 72 });
assert.deepEqual(lastHoverPixel, null);

view.handleCanvasClick({ clientX: 64, clientY: 48 });
assert.deepEqual(view.getSelectedPoints(), [{ x: 64, y: 48 }]);
assert.deepEqual(selectedSnapshots, [[{ x: 64, y: 48 }]]);
