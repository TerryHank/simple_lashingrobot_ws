import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { DEFAULT_TOPIC_LAYER_STATE } from "../src/config/topicLayerCatalog.js";
import { WorkspaceCanvasView } from "../src/views/WorkspaceCanvasView.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");
const uiControllerText = readFileSync(resolve(frontendRoot, "src/ui/UIController.js"), "utf-8");
const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");

global.ImageData = class ImageData {
  constructor(data, width, height) {
    this.data = data;
    this.width = width;
    this.height = height;
  }
};

function createRecordingContext() {
  const calls = [];
  const context = {
    calls,
    beginPath: () => calls.push(["beginPath"]),
    clearRect: (...args) => calls.push(["clearRect", ...args]),
    closePath: () => calls.push(["closePath"]),
    fill: () => calls.push(["fill"]),
    fillRect: (...args) => calls.push(["fillRect", ...args]),
    fillText: (...args) => calls.push(["fillText", ...args]),
    lineTo: (...args) => calls.push(["lineTo", ...args]),
    measureText: (text) => ({ width: String(text).length * 7 }),
    moveTo: (...args) => calls.push(["moveTo", ...args]),
    putImageData: (...args) => calls.push(["putImageData", ...args]),
    restore: () => calls.push(["restore"]),
    save: () => calls.push(["save"]),
    setLineDash: (...args) => calls.push(["setLineDash", ...args]),
    stroke: () => calls.push(["stroke"]),
    strokeRect: (...args) => calls.push(["strokeRect", ...args]),
    arc: (...args) => calls.push(["arc", ...args]),
  };
  Object.defineProperties(context, {
    fillStyle: {
      set: (value) => calls.push(["fillStyle", value]),
    },
    strokeStyle: {
      set: (value) => calls.push(["strokeStyle", value]),
    },
    lineWidth: {
      set: (value) => calls.push(["lineWidth", value]),
    },
  });
  return context;
}

function createCanvas(context, { width = 2, height = 2 } = {}) {
  return {
    width,
    height,
    style: {},
    addEventListener: () => {},
    getContext: () => context,
  };
}

const resultImageMessage = {
  width: 2,
  height: 2,
  encoding: "bgr8",
  data: [
    0, 0, 255,
    0, 255, 0,
    255, 0, 0,
    255, 255, 255,
  ],
};

assert.equal(DEFAULT_TOPIC_LAYER_STATE.showImageRecognitionResult, true);
assert.equal(DEFAULT_TOPIC_LAYER_STATE.showImageScanPoints, true);
assert.match(uiControllerText, /id="showImageRecognitionResultToggle"/);
assert.match(uiControllerText, /id="showImageScanPointsToggle"/);
assert.match(appText, /applyImageOverlayLayerState/);
assert.doesNotMatch(appText, /setSelectedImageTopic\(TOPICS\.camera\.irImage\)/);
assert.doesNotMatch(appText, /ensureInfraredImageLayer/);

const overlayContext = createRecordingContext();
const view = new WorkspaceCanvasView({
  canvas: createCanvas(createRecordingContext()),
  overlayCanvas: createCanvas(overlayContext),
});

view.setExecutionOverlayMessage(resultImageMessage);
assert.equal(overlayContext.calls.some(([name]) => name === "putImageData"), true);

overlayContext.calls.length = 0;
view.setImageOverlayLayerState({ showImageRecognitionResult: false });
view.setExecutionOverlayMessage(resultImageMessage);
assert.equal(overlayContext.calls.some(([name]) => name === "putImageData"), false);

overlayContext.calls.length = 0;
view.setVisualRecognitionPointsMessage({
  PointCoordinatesArray: [
    { idx: 7, Pix_coord: [1, 1] },
  ],
}, { sourceSize: { width: 2, height: 2 } });
assert.equal(overlayContext.calls.some(([name]) => name === "arc"), true);
assert.equal(
  overlayContext.calls.some(([name, value]) => (
    name === "fillStyle" && String(value).includes("255, 246, 97")
  )),
  true,
);
assert.equal(
  overlayContext.calls.some(([name, value]) => (
    name === "fillStyle" && String(value).includes("255, 45, 85")
  )),
  false,
);
assert.equal(
  overlayContext.calls.some(([name]) => name === "moveTo" || name === "lineTo"),
  false,
);

overlayContext.calls.length = 0;
view.setImageOverlayLayerState({ showImageScanPoints: false });
view.setVisualRecognitionPointsMessage({
  PointCoordinatesArray: [
    { idx: 7, Pix_coord: [1, 1] },
  ],
}, { sourceSize: { width: 2, height: 2 } });
assert.equal(overlayContext.calls.some(([name]) => name === "arc"), false);

const rangeContext = createRecordingContext();
const rangeView = new WorkspaceCanvasView({
  canvas: createCanvas(createRecordingContext(), { width: 640, height: 480 }),
  overlayCanvas: createCanvas(rangeContext, { width: 640, height: 480 }),
});
rangeView.setImageOverlayLayerState({ showLinearModuleBindRange: false });
rangeView.setTcpWorkspaceBoundary({
  points: [
    { x: 40, y: 40 },
    { x: 600, y: 40 },
    { x: 600, y: 440 },
    { x: 40, y: 440 },
  ],
  sourceSize: { width: 640, height: 480 },
});
assert.equal(rangeContext.calls.some(([name]) => name === "arc"), false);
