import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
import * as THREE from "three";

import { Scene3DView } from "../src/views/Scene3DView.js";
import {
  buildBindPathPointHoverEntries,
  buildCabinPathPointHoverEntries,
  buildUnplannedBindPathPointHoverEntries,
  formatScenePointWorldCoordinate,
} from "../src/utils/bindPathGeometry.js";

const bindPath = {
  areas: [
    {
      area_index: 7,
      cabin_pose: { x: 1200, y: 2300, z: 560 },
      groups: [
        {
          points: [
            { global_idx: 11, global_row: 2, global_col: 3, world_x: 100, world_y: 200, world_z: 510 },
            { global_idx: 12, global_row: 2, global_col: 4, world_x: 250, world_y: 200, world_z: 512 },
          ],
        },
      ],
    },
  ],
};

assert.deepEqual(
  buildBindPathPointHoverEntries(bindPath.areas).map((entry) => ({
    label: entry.label,
    globalIdx: entry.globalIdx,
    worldMm: entry.worldMm,
  })),
  [
    { label: "绑扎点", globalIdx: 11, worldMm: { x: 100, y: 200, z: 510 } },
    { label: "绑扎点", globalIdx: 12, worldMm: { x: 250, y: 200, z: 512 } },
  ],
);

assert.deepEqual(
  buildCabinPathPointHoverEntries(bindPath.areas).map((entry) => ({
    label: entry.label,
    areaIndex: entry.areaIndex,
    worldMm: entry.worldMm,
  })),
  [
    { label: "索驱规划点", areaIndex: 7, worldMm: { x: 1200, y: 2300, z: 560 } },
  ],
);

assert.equal(
  formatScenePointWorldCoordinate(buildBindPathPointHoverEntries(bindPath.areas)[0]),
  "绑扎点 #11\n世界 X 100.0 mm\n世界 Y 200.0 mm\n世界 Z 510.0 mm",
);

assert.equal(
  formatScenePointWorldCoordinate(buildCabinPathPointHoverEntries(bindPath.areas)[0]),
  "索驱规划点 区域7\n世界 X 1200.0 mm\n世界 Y 2300.0 mm\n世界 Z 560.0 mm",
);

assert.equal(
  formatScenePointWorldCoordinate(buildUnplannedBindPathPointHoverEntries(
    bindPath.areas,
    [
      { global_idx: 11, global_row: 2, global_col: 3, world_x: 100, world_y: 200, world_z: 510 },
      { global_idx: 13, global_row: 2, global_col: 5, world_x: 410, world_y: 205, world_z: 514 },
    ],
  )[0]),
  "未入组扫描点 #13\n世界 X 410.0 mm\n世界 Y 205.0 mm\n世界 Z 514.0 mm",
);

const hoverSceneView = Object.create(Scene3DView.prototype);
hoverSceneView.theme = "dark";
hoverSceneView.layerState = { pointSize: 0.04 };
hoverSceneView.scenePointHoverSelection = null;
const hoverObject = new THREE.Points(
  new THREE.BufferGeometry().setAttribute(
    "position",
    new THREE.Float32BufferAttribute([
      0.0, 0.0, 0.0,
      1.25, -0.5, 0.75,
    ], 3),
  ),
  new THREE.PointsMaterial({ size: 0.01 }),
);

hoverSceneView.setScenePointSelfHover({
  point: new THREE.Vector3(1.25, -0.5, 0.75),
  object: hoverObject,
  index: 1,
});

assert.equal(hoverSceneView.scenePointHoverSelection.object, hoverObject);
assert.equal(hoverSceneView.scenePointHoverSelection.index, 1);
assert.equal(hoverObject.geometry.getAttribute("pointHoverScale").getX(0), 1);
assert.ok(hoverObject.geometry.getAttribute("pointHoverScale").getX(1) > 1);
assert.equal(hoverObject.geometry.getAttribute("pointHoverColorMix").getX(0), 0);
assert.ok(hoverObject.geometry.getAttribute("pointHoverColorMix").getX(1) > 0);

hoverSceneView.clearScenePointSelfHover();
assert.equal(hoverSceneView.scenePointHoverSelection, null);
assert.equal(hoverObject.geometry.getAttribute("pointHoverScale").getX(1), 1);
assert.equal(hoverObject.geometry.getAttribute("pointHoverColorMix").getX(1), 0);

const currentDir = dirname(fileURLToPath(import.meta.url));
const sceneSource = readFileSync(resolve(currentDir, "../src/views/Scene3DView.js"), "utf8");
assert.match(sceneSource, /pointermove/);
assert.match(sceneSource, /pickScenePointAtClientPosition/);
assert.match(sceneSource, /pointHoverEntries/);
assert.match(sceneSource, /pointHoverScale/);
assert.match(sceneSource, /onBeforeCompile/);
assert.doesNotMatch(sceneSource, /scenePointHoverHighlight/);
