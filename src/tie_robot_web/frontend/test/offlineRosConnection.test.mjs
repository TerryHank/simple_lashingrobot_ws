import assert from "node:assert/strict";

import { createOfflineRosConnection } from "../src/controllers/offlineRosConnection.js";

const offline = createOfflineRosConnection();

assert.equal(offline.isReady(), false);
assert.deepEqual(offline.getResources(), {});

assert.deepEqual(
  offline.updateDisplayedImageSubscription("/Scepter/ir/image_raw"),
  { success: false, changed: false, topic: "/Scepter/ir/image_raw" },
);
assert.deepEqual(
  offline.updateLogSubscription("all"),
  { success: false, changed: false, topicId: "all" },
);
assert.deepEqual(
  offline.updatePointCloudSubscription({ enabled: true, source: "rawWorldCoord" }),
  { success: false, changed: false, enabled: true, source: "rawWorldCoord" },
);
assert.deepEqual(
  offline.updateImageHoverCoordinateSubscription({ enabled: false }),
  { success: false, changed: false, enabled: false },
);
