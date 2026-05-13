import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");
const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");

assert.match(appText, /this\.bindPointVisualsSuppressed = false;/);
assert.match(appText, /this\.bindPointVisualsSuppressed = true;/);
assert.match(appText, /this\.planningAreaRequestToken \+= 1;/);
assert.match(appText, /shouldSuppressBindPointVisualMessage\(message\)/);
assert.match(appText, /countPlanningMarkerVisualPoints\(marker\)/);
assert.match(appText, /markerNamespace\.startsWith\("pseudo_slam_"\)/);
assert.match(appText, /if \(this\.shouldSuppressBindPointVisualMessage\(message\)\) \{\s*return 0;/s);
assert.match(appText, /handleWorkspaceS2Triggered\(\) \{\s*this\.bindPointVisualsSuppressed = false;/s);
assert.match(appText, /this\.latestVisualRecognitionPointsMessage = null;/);
