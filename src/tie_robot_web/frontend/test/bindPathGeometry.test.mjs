import assert from "node:assert/strict";

import {
  buildBindGridLineSegmentPositions,
  buildBindGroupLineSegmentPositions,
  buildBindPathPointPositions,
  collectBindPathGridPoints,
} from "../src/utils/bindPathGeometry.js";

const bindPath = {
  areas: [
    {
      groups: [
        {
          points: [
            { global_idx: 1, global_row: 0, global_col: 0, world_x: 0, world_y: 0, world_z: 500 },
            { global_idx: 2, global_row: 0, global_col: 1, world_x: 150, world_y: 0, world_z: 500 },
          ],
        },
      ],
    },
    {
      groups: [
        {
          points: [
            { global_idx: 3, global_row: 1, global_col: 0, world_x: 0, world_y: 150, world_z: 510 },
            { global_idx: 4, global_row: 1, global_col: 1, world_x: 150, world_y: 150, world_z: 510 },
            { global_idx: 4, global_row: 1, global_col: 1, world_x: 150, world_y: 150, world_z: 510 },
          ],
        },
      ],
    },
  ],
};

assert.deepEqual(
  collectBindPathGridPoints(bindPath.areas).map((point) => ({
    globalIdx: point.globalIdx,
    row: point.row,
    col: point.col,
    x: point.x,
    y: point.y,
    z: point.z,
  })),
  [
    { globalIdx: 1, row: 0, col: 0, x: 0, y: 0, z: 0.5 },
    { globalIdx: 2, row: 0, col: 1, x: 0.15, y: 0, z: 0.5 },
    { globalIdx: 3, row: 1, col: 0, x: 0, y: 0.15, z: 0.51 },
    { globalIdx: 4, row: 1, col: 1, x: 0.15, y: 0.15, z: 0.51 },
  ],
);

assert.deepEqual(buildBindPathPointPositions(bindPath.areas), [
  0, 0, 0.5,
  0.15, 0, 0.5,
  0, 0.15, 0.51,
  0.15, 0.15, 0.51,
]);

assert.deepEqual(buildBindGridLineSegmentPositions(bindPath.areas, { axis: "row" }), [
  0, 0, 0.5,
  0.15, 0, 0.5,
  0, 0.15, 0.51,
  0.15, 0.15, 0.51,
]);

assert.deepEqual(buildBindGridLineSegmentPositions(bindPath.areas, { axis: "column" }), [
  0, 0, 0.5,
  0, 0.15, 0.51,
  0.15, 0, 0.5,
  0.15, 0.15, 0.51,
]);

const bindPathWithUnmatchedGridPoint = {
  areas: [
    {
      groups: [
        {
          points: [
            { global_idx: 1, global_row: 0, global_col: 0, world_x: 0, world_y: 0, world_z: 500 },
            { global_idx: 2, global_row: 0, global_col: 1, world_x: 150, world_y: 0, world_z: 500 },
          ],
        },
      ],
    },
  ],
  grid_points: [
    { global_idx: 1, global_row: 0, global_col: 0, world_x: 0, world_y: 0, world_z: 500 },
    { global_idx: 2, global_row: 0, global_col: 1, world_x: 150, world_y: 0, world_z: 500 },
    { global_idx: 3, global_row: 0, global_col: 2, world_x: 300, world_y: 0, world_z: 500 },
  ],
};

assert.deepEqual(
  buildBindGridLineSegmentPositions(bindPathWithUnmatchedGridPoint.areas, {
    axis: "row",
    gridPoints: bindPathWithUnmatchedGridPoint.grid_points,
  }),
  [
    0, 0, 0.5,
    0.15, 0, 0.5,
    0.15, 0, 0.5,
    0.3, 0, 0.5,
  ],
);

const bindPathWithWorldOrderDisagreeingWithGridOrder = {
  areas: [],
  grid_points: [
    { global_idx: 1, global_row: 0, global_col: 0, world_x: 0, world_y: 0, world_z: 500 },
    { global_idx: 2, global_row: 0, global_col: 1, world_x: 300, world_y: 10, world_z: 500 },
    { global_idx: 3, global_row: 0, global_col: 2, world_x: 150, world_y: 20, world_z: 500 },
    { global_idx: 4, global_row: 0, global_col: 3, world_x: 450, world_y: 30, world_z: 500 },
  ],
};

assert.deepEqual(
  buildBindGridLineSegmentPositions(bindPathWithWorldOrderDisagreeingWithGridOrder.areas, {
    axis: "row",
    gridPoints: bindPathWithWorldOrderDisagreeingWithGridOrder.grid_points,
  }),
  [
    0, 0, 0.5,
    0.3, 0.01, 0.5,
    0.3, 0.01, 0.5,
    0.15, 0.02, 0.5,
    0.15, 0.02, 0.5,
    0.45, 0.03, 0.5,
  ],
);

const bindPathWithGroupShapes = {
  areas: [
    {
      groups: [
        {
          group_type: "matrix_2x2",
          points: [
            { global_idx: 1, global_row: 0, global_col: 0, world_x: 0, world_y: 0, world_z: 500 },
            { global_idx: 2, global_row: 0, global_col: 1, world_x: 150, world_y: 0, world_z: 500 },
            { global_idx: 3, global_row: 1, global_col: 0, world_x: 0, world_y: 150, world_z: 510 },
            { global_idx: 4, global_row: 1, global_col: 1, world_x: 150, world_y: 150, world_z: 510 },
          ],
        },
        {
          group_type: "matrix_2x2_edge_pair",
          points: [
            { global_idx: 5, global_row: 3, global_col: 2, world_x: 300, world_y: 450, world_z: 520 },
            { global_idx: 6, global_row: 3, global_col: 3, world_x: 450, world_y: 450, world_z: 520 },
          ],
        },
      ],
    },
  ],
};

assert.deepEqual(buildBindGroupLineSegmentPositions(bindPathWithGroupShapes.areas), [
  0, 0, 0.5,
  0.15, 0, 0.5,
  0.15, 0, 0.5,
  0.15, 0.15, 0.51,
  0.15, 0.15, 0.51,
  0, 0.15, 0.51,
  0, 0.15, 0.51,
  0, 0, 0.5,
  0.3, 0.45, 0.52,
  0.45, 0.45, 0.52,
]);
