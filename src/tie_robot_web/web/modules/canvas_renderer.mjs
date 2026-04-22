import { enhanceMono8 } from "../ir_workspace_picker_helpers.mjs";
import { canvas, ctx, pointsEl, s2OverlayCanvas, s2OverlayCtx } from "./dom_refs.mjs";
import { applyOverlayOpacity, saveDisplayPreferences, state, syncDisplayControls, updateActionButtons } from "./ui_state.mjs";

export function decodeImageBytes(data) {
  if (Array.isArray(data)) {
    return Uint8ClampedArray.from(data);
  }
  if (typeof data === "string") {
    const binary = atob(data);
    const bytes = new Uint8ClampedArray(binary.length);
    for (let index = 0; index < binary.length; index += 1) {
      bytes[index] = binary.charCodeAt(index);
    }
    return bytes;
  }
  if (data instanceof Uint8Array || data instanceof Uint8ClampedArray) {
    return new Uint8ClampedArray(data);
  }
  if (data?.buffer instanceof ArrayBuffer) {
    return new Uint8ClampedArray(data.buffer);
  }
  return null;
}

export function sensorImageToImageData(message, enhancementSettings = state.displaySettings) {
  const bytes = decodeImageBytes(message.data);
  if (!bytes) {
    throw new Error("无法解析 IR 图像数据");
  }
  const width = Number(message.width) || 0;
  const height = Number(message.height) || 0;
  const encoding = String(message.encoding || "").toLowerCase();
  if (width <= 0 || height <= 0) {
    throw new Error("IR 图像尺寸无效");
  }
  const rgba = new Uint8ClampedArray(width * height * 4);
  if (encoding.includes("mono8") || encoding.includes("8uc1")) {
    const displayBytes = enhanceMono8(bytes, enhancementSettings);
    for (let pixelIndex = 0; pixelIndex < width * height; pixelIndex += 1) {
      const value = displayBytes[pixelIndex] ?? 0;
      const rgbaIndex = pixelIndex * 4;
      rgba[rgbaIndex] = value;
      rgba[rgbaIndex + 1] = value;
      rgba[rgbaIndex + 2] = value;
      rgba[rgbaIndex + 3] = 255;
    }
    return new ImageData(rgba, width, height);
  }
  if (encoding.includes("rgb8")) {
    for (let pixelIndex = 0; pixelIndex < width * height; pixelIndex += 1) {
      const sourceIndex = pixelIndex * 3;
      const rgbaIndex = pixelIndex * 4;
      rgba[rgbaIndex] = bytes[sourceIndex] ?? 0;
      rgba[rgbaIndex + 1] = bytes[sourceIndex + 1] ?? 0;
      rgba[rgbaIndex + 2] = bytes[sourceIndex + 2] ?? 0;
      rgba[rgbaIndex + 3] = 255;
    }
    return new ImageData(rgba, width, height);
  }
  if (encoding.includes("bgr8")) {
    for (let pixelIndex = 0; pixelIndex < width * height; pixelIndex += 1) {
      const sourceIndex = pixelIndex * 3;
      const rgbaIndex = pixelIndex * 4;
      rgba[rgbaIndex] = bytes[sourceIndex + 2] ?? 0;
      rgba[rgbaIndex + 1] = bytes[sourceIndex + 1] ?? 0;
      rgba[rgbaIndex + 2] = bytes[sourceIndex] ?? 0;
      rgba[rgbaIndex + 3] = 255;
    }
    return new ImageData(rgba, width, height);
  }
  throw new Error(`暂不支持的图像编码: ${message.encoding}`);
}

export function drawImageMessageToCanvas(targetCanvas, targetCtx, message, options = state.displaySettings) {
  if (!targetCanvas || !targetCtx || !message) {
    return;
  }
  const imageData = sensorImageToImageData(message, options);
  targetCanvas.width = imageData.width;
  targetCanvas.height = imageData.height;
  targetCtx.putImageData(imageData, 0, 0);
}

export function clearCanvas(targetCanvas, targetCtx) {
  if (!targetCanvas || !targetCtx) {
    return;
  }
  targetCtx.clearRect(0, 0, targetCanvas.width, targetCanvas.height);
}

export function drawS2Overlay() {
  const overlayMessage =
    state.overlaySource === "execution" ? state.lastExecutionResultImageMessage : state.lastResultImageMessage;
  if (!overlayMessage) {
    clearCanvas(s2OverlayCanvas, s2OverlayCtx);
    return;
  }
  drawImageMessageToCanvas(s2OverlayCanvas, s2OverlayCtx, overlayMessage, { mode: "raw", gamma: 1.0 });
}

export function drawOverlay() {
  if (!state.lastImageMessage) {
    return;
  }
  drawImageMessageToCanvas(canvas, ctx, state.lastImageMessage, state.displaySettings);
  ctx.save();
  ctx.lineWidth = 2;
  ctx.font = "18px monospace";
  if (state.savedWorkspacePoints.length >= 2) {
    ctx.strokeStyle = "#6aa6ff";
    ctx.fillStyle = "#6aa6ff";
    ctx.setLineDash([10, 6]);
    ctx.beginPath();
    ctx.moveTo(state.savedWorkspacePoints[0].x, state.savedWorkspacePoints[0].y);
    for (let index = 1; index < state.savedWorkspacePoints.length; index += 1) {
      ctx.lineTo(state.savedWorkspacePoints[index].x, state.savedWorkspacePoints[index].y);
    }
    if (state.savedWorkspacePoints.length === 4) {
      ctx.closePath();
    }
    ctx.stroke();
    ctx.setLineDash([]);
    state.savedWorkspacePoints.forEach((point, index) => {
      ctx.beginPath();
      ctx.arc(point.x, point.y, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillText(`S${index + 1}`, point.x + 8, point.y + 18);
    });
  }
  if (state.selectedPoints.length) {
    ctx.strokeStyle = "#4de3a5";
    ctx.fillStyle = "#ffae42";
    if (state.selectedPoints.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(state.selectedPoints[0].x, state.selectedPoints[0].y);
      for (let index = 1; index < state.selectedPoints.length; index += 1) {
        ctx.lineTo(state.selectedPoints[index].x, state.selectedPoints[index].y);
      }
      if (state.selectedPoints.length === 4) {
        ctx.closePath();
      }
      ctx.stroke();
    }
    state.selectedPoints.forEach((point, index) => {
      ctx.beginPath();
      ctx.arc(point.x, point.y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.stroke();
      ctx.fillStyle = "#4de3a5";
      ctx.fillText(`${index + 1}`, point.x + 8, point.y - 8);
      ctx.fillStyle = "#ffae42";
    });
  }
  ctx.restore();
  drawS2Overlay();
}

export function refreshPointsList() {
  pointsEl.innerHTML = "";
  state.selectedPoints.forEach((point, index) => {
    const item = document.createElement("li");
    item.textContent = `${index + 1}. 角点 ${index + 1} 已选`;
    pointsEl.appendChild(item);
  });
  if (!state.selectedPoints.length) {
    const item = document.createElement("li");
    item.textContent = "还没有点，直接点击 IR 图";
    pointsEl.appendChild(item);
  }
  updateActionButtons();
}

export function updateDisplaySettings(nextSettings) {
  state.displaySettings = {
    ...state.displaySettings,
    ...nextSettings,
  };
  syncDisplayControls();
  applyOverlayOpacity();
  saveDisplayPreferences();
  drawOverlay();
}
