import {
  findDraggablePointIndex,
  mapCanvasClickToImagePixel,
  replaceWorkspacePoint,
} from "../ir_workspace_picker_helpers.mjs";
import { canvas } from "./dom_refs.mjs";
import { drawOverlay, refreshPointsList } from "./canvas_renderer.mjs";
import { setStatus, state } from "./ui_state.mjs";

export function mapPointerEventToImagePixel(event) {
  return mapCanvasClickToImagePixel({
    clientX: event.clientX,
    clientY: event.clientY,
    rect: canvas.getBoundingClientRect(),
    imageWidth: Number(state.lastImageMessage.width),
    imageHeight: Number(state.lastImageMessage.height),
  });
}

export function handleCanvasClick(event) {
  if (state.suppressNextCanvasClick) {
    state.suppressNextCanvasClick = false;
    return;
  }
  if (!state.lastImageMessage) {
    setStatus("IR 图像还没到，先等一帧", "warn");
    return;
  }
  if (state.selectedPoints.length >= 4) {
    setStatus("已经点满 4 个角点了，先清空或撤销再继续", "warn");
    return;
  }
  const point = mapCanvasClickToImagePixel({
    clientX: event.clientX,
    clientY: event.clientY,
    rect: canvas.getBoundingClientRect(),
    imageWidth: Number(state.lastImageMessage.width),
    imageHeight: Number(state.lastImageMessage.height),
  });
  state.selectedPoints = [...state.selectedPoints, point];
  refreshPointsList();
  drawOverlay();
  if (state.selectedPoints.length === 4) {
    setStatus("4 个角点已选完，可直接拖拽绿点微调，点“提交四边形”即可保存工作区", "success");
  } else {
    setStatus(`已记录第 ${state.selectedPoints.length} 个角点`, "info");
  }
}

export function handleCanvasPointerDown(event) {
  if (!state.lastImageMessage || state.selectedPoints.length === 0) {
    return;
  }
  const point = mapPointerEventToImagePixel(event);
  const dragIndex = findDraggablePointIndex(state.selectedPoints, point, 12);
  if (dragIndex < 0) {
    return;
  }
  state.dragState = { activeIndex: dragIndex, pointerId: event.pointerId, moved: false };
  if (typeof canvas.setPointerCapture === "function") {
    canvas.setPointerCapture(event.pointerId);
  }
  setStatus(`正在拖拽第 ${dragIndex + 1} 个角点`, "info");
}

export function handleCanvasPointerMove(event) {
  if (state.dragState.activeIndex < 0 || !state.lastImageMessage) {
    return;
  }
  const point = mapPointerEventToImagePixel(event);
  state.selectedPoints = replaceWorkspacePoint(state.selectedPoints, state.dragState.activeIndex, point);
  state.dragState = { ...state.dragState, moved: true };
  drawOverlay();
}

export function handleCanvasPointerUp() {
  if (state.dragState.activeIndex < 0) {
    return;
  }
  if (typeof canvas.releasePointerCapture === "function" && state.dragState.pointerId !== null) {
    try {
      canvas.releasePointerCapture(state.dragState.pointerId);
    } catch {
      // ignore release failures
    }
  }
  const finishedIndex = state.dragState.activeIndex;
  const moved = state.dragState.moved;
  state.dragState = { activeIndex: -1, pointerId: null, moved: false };
  if (moved) {
    state.suppressNextCanvasClick = true;
    refreshPointsList();
    drawOverlay();
    if (state.selectedPoints.length === 4) {
      setStatus(`已调整第 ${finishedIndex + 1} 个角点，可继续拖拽微调或直接提交四边形`, "success");
    } else {
      setStatus(`已调整第 ${finishedIndex + 1} 个角点`, "info");
    }
  }
}

export function clearPoints() {
  state.selectedPoints = [];
  refreshPointsList();
  drawOverlay();
  setStatus("已清空角点，重新点击 IR 图即可", "info");
}

export function undoPoint() {
  if (!state.selectedPoints.length) {
    return;
  }
  state.selectedPoints = state.selectedPoints.slice(0, -1);
  refreshPointsList();
  drawOverlay();
  setStatus("已撤销最后一个角点", "info");
}
