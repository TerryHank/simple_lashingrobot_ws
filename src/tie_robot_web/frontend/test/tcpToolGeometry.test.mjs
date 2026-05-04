import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const scene3dViewText = readFileSync(resolve(__dirname, "../src/views/Scene3DView.js"), "utf-8");

assert.match(
  scene3dViewText,
  /const TCP_TOOL_SIZE_METERS = \{\s*x: 0\.1,\s*y: 0\.2,\s*z: 0\.2,\s*\};/s,
);
