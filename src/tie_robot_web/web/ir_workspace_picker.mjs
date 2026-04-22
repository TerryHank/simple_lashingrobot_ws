import { clearBtn, displayModeEl, gammaRangeEl, overlayOpacityRangeEl, runDirectBindPathTestBtn, runSavedS2Btn, moveToWorkspaceCenterScanPoseBtn, startExecutionBtn, startExecutionClearMemoryBtn, submitBtn, undoBtn, canvas } from "./modules/dom_refs.mjs";
import { applyOverlayOpacity, syncDisplayControls } from "./modules/ui_state.mjs";
import { updateDisplaySettings } from "./modules/canvas_renderer.mjs";
import { publishWorkspaceQuad, triggerDirectBindPathTest, triggerExecutionLayer, triggerSavedWorkspaceS2, triggerWorkspaceCenterScanPoseMove } from "./modules/execution_actions.mjs";
import { clearPoints, handleCanvasClick, handleCanvasPointerDown, handleCanvasPointerMove, handleCanvasPointerUp, undoPoint } from "./modules/workspace_selection.mjs";
import { connectRosbridge } from "./modules/ros_connection.mjs";
import { refreshPointsList } from "./modules/canvas_renderer.mjs";

canvas.addEventListener("pointerdown", handleCanvasPointerDown);
canvas.addEventListener("pointermove", handleCanvasPointerMove);
canvas.addEventListener("pointerup", handleCanvasPointerUp);
canvas.addEventListener("pointercancel", handleCanvasPointerUp);
canvas.addEventListener("click", handleCanvasClick);
clearBtn.addEventListener("click", clearPoints);
undoBtn.addEventListener("click", undoPoint);
submitBtn.addEventListener("click", publishWorkspaceQuad);
runSavedS2Btn.addEventListener("click", triggerSavedWorkspaceS2);
moveToWorkspaceCenterScanPoseBtn.addEventListener("click", triggerWorkspaceCenterScanPoseMove);
startExecutionBtn.addEventListener("click", () => triggerExecutionLayer(true));
startExecutionClearMemoryBtn.addEventListener("click", () => triggerExecutionLayer(false));
runDirectBindPathTestBtn.addEventListener("click", triggerDirectBindPathTest);
displayModeEl.addEventListener("change", (event) => {
  updateDisplaySettings({ mode: event.target.value });
});
gammaRangeEl.addEventListener("input", (event) => {
  updateDisplaySettings({ gamma: Number(event.target.value) });
});
overlayOpacityRangeEl.addEventListener("input", (event) => {
  updateDisplaySettings({ overlayOpacity: Number(event.target.value) });
});

refreshPointsList();
syncDisplayControls();
applyOverlayOpacity();
connectRosbridge();
