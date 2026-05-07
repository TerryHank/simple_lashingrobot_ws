import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import {
  DEFAULT_TOPIC_LAYER_STATE,
  IMAGE_HOVER_COORDINATE_FRAMES,
  getImageHoverCoordinateFrameLabel,
} from "../src/config/topicLayerCatalog.js";
import { loadTopicLayerStatePreference, saveTopicLayerStatePreference } from "../src/utils/storage.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

global.localStorage = {
  value: "",
  getItem() {
    return this.value;
  },
  setItem(_key, value) {
    this.value = value;
  },
};

assert.deepEqual(IMAGE_HOVER_COORDINATE_FRAMES.map((frame) => frame.id), ["map", "gripper_frame", "Scepter_depth_frame"]);
assert.equal(DEFAULT_TOPIC_LAYER_STATE.imageHoverCoordinateFrame, "map");
assert.equal(getImageHoverCoordinateFrameLabel("gripper_frame"), "工具 TCP");

saveTopicLayerStatePreference({ ...DEFAULT_TOPIC_LAYER_STATE, imageHoverCoordinateFrame: "gripper_frame" });
assert.equal(loadTopicLayerStatePreference().imageHoverCoordinateFrame, "gripper_frame");

const uiText = readFileSync(resolve(frontendRoot, "src/ui/UIController.js"), "utf-8");
assert.match(uiText, /id="imageHoverCoordinateFrame"/);
assert.match(uiText, /IMAGE_HOVER_COORDINATE_FRAMES\.map/);

const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");
assert.match(appText, /syncImageHoverCoordinateSubscription/);
assert.match(appText, /handleImageHoverPixelChanged/);
assert.match(appText, /IMAGE_HOVER_COORDINATE_IDLE_UNSUBSCRIBE_MS\s*=\s*1800/);
assert.match(appText, /scheduleImageHoverCoordinateSubscriptionIdle/);
assert.doesNotMatch(appText, /const hoverEnabled = this\.activeSettingsPage !== "workspace";/);
assert.match(appText, /updateImageHoverCoordinateSubscription\(\{ enabled: true \}\)/);
assert.match(appText, /updateImageHoverCoordinateSubscription\(\{ enabled: false \}\)/);

const rosText = readFileSync(resolve(frontendRoot, "src/controllers/RosConnectionController.js"), "utf-8");
assert.match(rosText, /updateImageHoverCoordinateSubscription/);
assert.match(rosText, /onImageHoverWorldCoord/);
assert.match(rosText, /IMAGE_HOVER_WORLD_COORD_THROTTLE_MS\s*=\s*1000/);
assert.match(rosText, /throttle_rate:\s*IMAGE_HOVER_WORLD_COORD_THROTTLE_MS/);
