import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { createScene3DView } from "../src/views/Scene3DView.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

class FakeElement {
  constructor() {
    this.children = [];
    this.className = "";
    this.textContent = "";
    this.clientWidth = 640;
    this.clientHeight = 360;
    this.hidden = false;
    this.dataset = {};
    this.attributes = new Map();
  }

  appendChild(child) {
    this.children.push(child);
    return child;
  }

  setAttribute(name, value) {
    this.attributes.set(name, value);
  }

  addEventListener() {}
}

global.document = {
  createElement() {
    return new FakeElement();
  },
};

global.window = {
  devicePixelRatio: 1,
  requestAnimationFrame() {
    return 1;
  },
};

global.ResizeObserver = class {
  observe() {}
};

const container = new FakeElement();
const view = createScene3DView({
  container,
  rendererFactory() {
    throw new Error("Error creating WebGL context.");
  },
});

assert.equal(view.webglAvailable, false);
assert.equal(container.children.length, 1);
assert.match(container.children[0].textContent, /3D 场景暂不可用/);

view.setTheme("light");
view.setLayerState({ showRobot: true });
view.setViewMode("camera");
view.setFollowOrigin(true);

view.handleTfMessage({
  transforms: [
    {
      child_frame_id: "base_link",
      header: { frame_id: "map" },
      transform: {
        translation: { x: 1.2, y: 2.3, z: 0.56 },
        rotation: { x: 0, y: 0, z: 0, w: 1 },
      },
    },
    {
      child_frame_id: "Scepter_depth_frame",
      header: { frame_id: "base_link" },
      transform: {
        translation: { x: 0, y: 0, z: 0.4 },
        rotation: { x: 0, y: 0, z: 0, w: 1 },
      },
    },
    {
      child_frame_id: "gripper_frame",
      header: { frame_id: "Scepter_depth_frame" },
      transform: {
        translation: { x: 0.1, y: 0, z: 0.5 },
        rotation: { x: 0, y: 0, z: 0, w: 1 },
      },
    },
  ],
});

assert.equal(view.getKnownTransformCount(), 3);
assert.deepEqual(view.getKnownTransforms(), [
  { childFrame: "base_link", parentFrame: "map" },
  { childFrame: "gripper_frame", parentFrame: "Scepter_depth_frame" },
  { childFrame: "Scepter_depth_frame", parentFrame: "base_link" },
]);
assert.deepEqual(view.getCurrentCabinPositionMm(), { x: 1200, y: 2300, z: 560 });
assert.deepEqual(
  view.convertScepterPointMmToFrameMm({ x: 100, y: 0, z: 1000 }, "map"),
  { x: 1300, y: 2300, z: 1960 },
);
assert.deepEqual(
  view.getCameraToTcpCalibration().translationMm,
  { x: 100, y: 0, z: 500 },
);
assert.equal(view.setPointCloudImageMessage("filteredWorldCoord", { width: 1, height: 1, data: [] }), 0);
assert.equal(view.setTiePointsMessage({ PointCoordinatesArray: [{ World_coord: [1, 2, 3] }] }), 0);
assert.equal(view.setPlanningMarkersMessage({ markers: [{ points: [{ x: 1, y: 2, z: 3 }] }] }), 0);
assert.equal(view.clearPointCloudSource("filteredWorldCoord"), 0);
view.pointCounts.tiePoints = 12;
view.pointCounts.planningPoints = 34;
view.pointCounts.bindPathPoints = 56;
view.pointCounts.unplannedBindPathPoints = 7;
view.pointCounts.jumpBindPoints = 8;
view.pointHoverEntries.tiePoints = [{ label: "绑扎点" }];
view.pointHoverEntries.planningPoints = [{ label: "索驱规划点" }];
view.pointHoverEntries.bindPathPoints = [{ label: "账本绑扎点" }];
view.pointHoverEntries.unplannedBindPathPoints = [{ label: "未规划绑扎点" }];
view.pointHoverEntries.jumpBindPoints = [{ label: "跳绑点" }];
view.planningAreaPayload = { areas: [{}], grid_points: [{}] };
view.clearBindPointVisuals();
assert.equal(view.pointCounts.tiePoints, 0);
assert.equal(view.pointCounts.planningPoints, 0);
assert.equal(view.pointCounts.bindPathPoints, 0);
assert.equal(view.pointCounts.unplannedBindPathPoints, 0);
assert.equal(view.pointCounts.jumpBindPoints, 0);
assert.deepEqual(view.pointHoverEntries.tiePoints, []);
assert.deepEqual(view.pointHoverEntries.planningPoints, []);
assert.deepEqual(view.pointHoverEntries.bindPathPoints, []);
assert.deepEqual(view.pointHoverEntries.unplannedBindPathPoints, []);
assert.deepEqual(view.pointHoverEntries.jumpBindPoints, []);
assert.equal(view.planningAreaPayload, null);

assert.throws(
  () => createScene3DView({
    container: new FakeElement(),
    rendererFactory() {
      throw new Error("Unexpected scene setup failure.");
    },
  }),
  /Unexpected scene setup failure/,
);

const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");
assert.match(appText, /createScene3DView/);
assert.doesNotMatch(appText, /new Scene3DView\(/);

const cssText = readFileSync(resolve(frontendRoot, "src/styles/app.css"), "utf-8");
assert.match(cssText, /\.scene-unavailable-notice/);
