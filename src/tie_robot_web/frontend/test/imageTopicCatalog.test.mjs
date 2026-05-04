import assert from "node:assert/strict";

import {
  IMAGE_TOPIC_OPTIONS,
  isOverlayCompatibleImageTopic,
} from "../src/config/imageTopicCatalog.js";
import { TOPICS } from "../src/config/topicRegistry.js";

const optionIds = IMAGE_TOPIC_OPTIONS.map((option) => option.id);

assert.ok(optionIds.includes(TOPICS.algorithm.scanSurfaceDpBaseImage));
assert.ok(optionIds.includes(TOPICS.algorithm.scanSurfaceDpCompletedSurfaceImage));
assert.ok(optionIds.includes(TOPICS.algorithm.executionRefineBaseImage));

assert.equal(isOverlayCompatibleImageTopic(TOPICS.algorithm.scanSurfaceDpBaseImage), false);
assert.equal(isOverlayCompatibleImageTopic(TOPICS.algorithm.scanSurfaceDpCompletedSurfaceImage), false);
assert.equal(isOverlayCompatibleImageTopic(TOPICS.algorithm.executionRefineBaseImage), false);
