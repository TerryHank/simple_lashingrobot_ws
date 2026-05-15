import assert from "node:assert/strict";

import { getInitialControlToggleStateMap } from "../src/config/controlPanelCatalog.js";
import { LegacyCommandController } from "../src/controllers/LegacyCommandController.js";

const initialStates = getInitialControlToggleStateMap();
assert.equal(initialStates.lashingEnabled.value, false);
assert.equal(initialStates.lashingEnabled.label, "开启绑扎");
assert.equal(initialStates.lashingEnabled.tone, "green");

const controller = new LegacyCommandController({
  rosConnection: {
    getResources: () => ({ ros: {} }),
  },
});

const snapshot = controller.getToggleStateSnapshot().lashingEnabled;
assert.equal(snapshot.value, false);
assert.equal(snapshot.label, "开启绑扎");
assert.equal(snapshot.tone, "green");
