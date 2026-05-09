import {
  buildWorkspaceQuadPayload,
  findDraggablePointIndex,
  mapCanvasClickToImagePixel,
  parseWorkspaceQuadPayload,
  replaceWorkspacePoint,
  sensorImageToImageData,
} from "../utils/irImageUtils.js";

function normalizeImageSize(source) {
  const width = Number(source?.width);
  const height = Number(source?.height);
  if (!Number.isFinite(width) || !Number.isFinite(height) || width <= 0 || height <= 0) {
    return null;
  }
  return { width, height };
}

function extractPointPixel(point) {
  const pixCoord = Array.isArray(point?.Pix_coord) ? point.Pix_coord : [];
  if (pixCoord.length < 2) {
    return null;
  }
  const x = Number(pixCoord[0]);
  const y = Number(pixCoord[1]);
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return null;
  }
  return { x, y };
}

function normalizeTcpWorkspacePlane(plane) {
  const points = Array.isArray(plane?.points)
    ? plane.points.map((point) => {
      const x = Number(point?.x);
      const y = Number(point?.y);
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        return null;
      }
      return { x, y, inside: Boolean(point?.inside) };
    })
    : [];
  if (points.length !== 4 || points.some((point) => !point)) {
    return null;
  }
  const z = Number(plane?.z);
  return {
    z: Number.isFinite(z) ? z : null,
    points,
  };
}

function normalizeTcpWorkspaceBoundary(boundary) {
  const explicitPlanes = Array.isArray(boundary?.planes)
    ? boundary.planes.map((plane) => normalizeTcpWorkspacePlane(plane)).filter(Boolean)
    : [];
  const fallbackPlane = normalizeTcpWorkspacePlane(boundary);
  const planes = explicitPlanes.length ? explicitPlanes : (fallbackPlane ? [fallbackPlane] : []);
  if (!planes.length) {
    return null;
  }
  return {
    points: planes[0].points,
    planes,
    sourceSize: normalizeImageSize(boundary?.sourceSize),
    frameId: boundary?.frameId || "gripper_frame",
  };
}

const DEFAULT_IMAGE_OVERLAY_LAYER_STATE = Object.freeze({
  showImageRecognitionResult: true,
  showImageScanPoints: true,
  showLinearModuleBindRange: true,
});

export class WorkspaceCanvasView {
  constructor({
    canvas,
    overlayCanvas,
    onSelectionChanged,
    onMessage,
    onHoverPixelChanged,
  }) {
    this.canvas = canvas;
    this.overlayCanvas = overlayCanvas;
    this.ctx = canvas.getContext("2d");
    this.overlayCtx = overlayCanvas.getContext("2d");
    this.onSelectionChanged = onSelectionChanged;
    this.onMessage = onMessage;
    this.onHoverPixelChanged = onHoverPixelChanged;
    this.lastImageMessage = null;
    this.lastExecutionResultMessage = null;
    this.lastVisualRecognitionPointsMessage = null;
    this.visualRecognitionPointSourceSize = null;
    this.tcpWorkspaceBoundary = null;
    this.savedWorkspacePoints = [];
    this.savedWorkspaceSourceSize = null;
    this.selectedPoints = [];
    this.workspacePickingEnabled = false;
    this.hoverCoordinateReadout = null;
    this.displaySettings = { mode: "raw", gamma: 1.0, overlayOpacity: 0.88 };
    this.overlayEnabled = true;
    this.imageOverlayLayerState = { ...DEFAULT_IMAGE_OVERLAY_LAYER_STATE };
    this.savedWorkspaceGuideVisible = false;
    this.dragState = { activeIndex: -1, moved: false };
    this.suppressNextCanvasClick = false;
  }

  bindPointerEvents() {
    this.canvas.addEventListener("click", (event) => this.handleCanvasClick(event));
    this.canvas.addEventListener("pointerdown", (event) => this.handlePointerDown(event));
    this.canvas.addEventListener("pointermove", (event) => this.handlePointerMove(event));
    this.canvas.addEventListener("pointerup", () => this.handlePointerUp());
    this.canvas.addEventListener("pointerleave", () => this.handlePointerLeave());
  }

  setDisplaySettings(settings) {
    this.displaySettings = { ...this.displaySettings, ...settings };
    this.overlayCanvas.style.opacity = String(this.displaySettings.overlayOpacity);
    this.draw();
  }

  setOverlayEnabled(enabled) {
    this.overlayEnabled = Boolean(enabled);
    if (!this.isWorkspacePickingActive()) {
      this.dragState = { activeIndex: -1, moved: false };
      this.suppressNextCanvasClick = false;
    }
    this.draw();
  }

  setImageOverlayLayerState(state = {}) {
    const nextState = { ...this.imageOverlayLayerState };
    Object.keys(DEFAULT_IMAGE_OVERLAY_LAYER_STATE).forEach((key) => {
      if (typeof state?.[key] === "boolean") {
        nextState[key] = state[key];
      }
    });
    this.imageOverlayLayerState = nextState;
    this.drawOverlay();
  }

  setSavedWorkspaceGuideVisible(enabled) {
    this.savedWorkspaceGuideVisible = Boolean(enabled);
    this.draw();
  }

  setWorkspacePickingEnabled(enabled) {
    this.workspacePickingEnabled = Boolean(enabled);
    if (this.isWorkspacePickingActive()) {
      this.setHoverCoordinateReadout(null);
      this.onHoverPixelChanged?.(null);
    }
  }

  isWorkspacePickingActive() {
    return this.workspacePickingEnabled && this.overlayEnabled;
  }

  setHoverCoordinateReadout(readout) {
    this.hoverCoordinateReadout = readout && readout.pixel ? readout : null;
    this.drawOverlay();
  }

  setBaseImageMessage(message) {
    this.lastImageMessage = message;
    this.draw();
  }

  setS2OverlayMessage(message) {
    this.setExecutionOverlayMessage(message);
  }

  setExecutionOverlayMessage(message) {
    this.lastExecutionResultMessage = message;
    this.drawOverlay();
  }

  setVisualRecognitionOverlaySourceSize(source) {
    this.visualRecognitionPointSourceSize = normalizeImageSize(source);
    this.drawOverlay();
  }

  setVisualRecognitionPointsMessage(message, { sourceSize = null } = {}) {
    this.lastVisualRecognitionPointsMessage = Array.isArray(message?.PointCoordinatesArray)
      ? message
      : null;
    this.visualRecognitionPointSourceSize =
      normalizeImageSize(sourceSize)
      || this.visualRecognitionPointSourceSize
      || this.getCurrentImageSize();
    this.drawOverlay();
  }

  setTcpWorkspaceBoundary(boundary) {
    this.tcpWorkspaceBoundary = normalizeTcpWorkspaceBoundary(boundary);
    this.drawOverlay();
  }

  setOverlaySource(source) {
    void source;
    this.drawOverlay();
  }

  setSavedWorkspacePoints(points) {
    this.savedWorkspacePoints = Array.isArray(points) ? points : [];
    this.savedWorkspaceSourceSize = this.getCurrentImageSize();
    this.draw();
    this.drawOverlay();
  }

  setSavedWorkspacePayload(payload, { sourceSize = null } = {}) {
    this.savedWorkspacePoints = parseWorkspaceQuadPayload(payload);
    this.savedWorkspaceSourceSize = normalizeImageSize(sourceSize) || this.getCurrentImageSize();
    this.draw();
    this.drawOverlay();
  }

  setSelectedWorkspacePayload(payload) {
    this.selectedPoints = parseWorkspaceQuadPayload(payload);
    this.notifySelectionChanged();
    this.draw();
  }

  getSelectedPoints() {
    return [...this.selectedPoints];
  }

  getSavedWorkspacePoints() {
    return [...this.savedWorkspacePoints];
  }

  clearSelection() {
    this.selectedPoints = [];
    this.notifySelectionChanged();
    this.draw();
  }

  undoSelection() {
    if (!this.selectedPoints.length) {
      return;
    }
    this.selectedPoints = this.selectedPoints.slice(0, -1);
    this.notifySelectionChanged();
    this.draw();
  }

  buildWorkspacePayload() {
    return buildWorkspaceQuadPayload(this.selectedPoints);
  }

  notifySelectionChanged() {
    this.onSelectionChanged?.(this.getSelectedPoints());
  }

  draw() {
    const imageMessage = this.lastImageMessage;
    if (!imageMessage) {
      return;
    }
    const imageData = sensorImageToImageData(imageMessage, this.displaySettings);
    this.canvas.width = imageData.width;
    this.canvas.height = imageData.height;
    this.overlayCanvas.width = imageData.width;
    this.overlayCanvas.height = imageData.height;
    this.ctx.putImageData(imageData, 0, 0);
    this.drawWorkspacePolylines();
    this.drawOverlay();
  }

  getCurrentImageSize() {
    return normalizeImageSize(this.lastImageMessage)
      || normalizeImageSize({ width: this.canvas.width, height: this.canvas.height });
  }

  drawOverlay() {
    this.overlayCtx.clearRect(0, 0, this.overlayCanvas.width, this.overlayCanvas.height);
    if (
      this.overlayEnabled
      && this.imageOverlayLayerState.showImageRecognitionResult !== false
      && this.lastExecutionResultMessage
    ) {
      const imageData = sensorImageToImageData(this.lastExecutionResultMessage, { mode: "raw", gamma: 1.0, overlayOpacity: 1.0 });
      if (imageData.width === this.overlayCanvas.width && imageData.height === this.overlayCanvas.height) {
        this.overlayCtx.putImageData(imageData, 0, 0);
      }
    }
    if (this.overlayEnabled && this.imageOverlayLayerState.showLinearModuleBindRange !== false) {
      this.drawTcpWorkspaceBoundary();
    }
    if (this.overlayEnabled && this.imageOverlayLayerState.showImageScanPoints !== false) {
      this.drawVisualRecognitionPoints();
    }
    this.drawHoverCoordinateReadout();
    this.overlayCanvas.style.opacity = String(this.displaySettings.overlayOpacity);
  }

  drawHoverCoordinateReadout() {
    const readout = this.hoverCoordinateReadout;
    if (!readout || this.overlayCanvas.width <= 0 || this.overlayCanvas.height <= 0) {
      return;
    }

    const sourceSize = normalizeImageSize(readout.sourceSize) || this.getCurrentImageSize();
    if (!sourceSize) {
      return;
    }
    const xScale = this.overlayCanvas.width / sourceSize.width;
    const yScale = this.overlayCanvas.height / sourceSize.height;
    const x = Number(readout.pixel.x) * xScale;
    const y = Number(readout.pixel.y) * yScale;
    if (![x, y].every(Number.isFinite)) {
      return;
    }

    const lines = Array.isArray(readout.lines) && readout.lines.length
      ? readout.lines.map((line) => String(line))
      : [String(readout.text || "")].filter(Boolean);
    if (!lines.length) {
      return;
    }

    this.overlayCtx.save();
    this.overlayCtx.font = "13px monospace";
    this.overlayCtx.textBaseline = "top";
    this.overlayCtx.lineWidth = 1.5;
    this.overlayCtx.strokeStyle = "rgba(255, 255, 255, 0.92)";
    this.overlayCtx.beginPath();
    this.overlayCtx.moveTo(x - 9, y);
    this.overlayCtx.lineTo(x + 9, y);
    this.overlayCtx.moveTo(x, y - 9);
    this.overlayCtx.lineTo(x, y + 9);
    this.overlayCtx.stroke();

    const paddingX = 7;
    const paddingY = 5;
    const lineHeight = 16;
    const textWidth = Math.max(...lines.map((line) => this.overlayCtx.measureText(line).width));
    const boxWidth = textWidth + paddingX * 2;
    const boxHeight = lines.length * lineHeight + paddingY * 2;
    const placeRight = x < this.overlayCanvas.width * 0.58;
    const placeBelow = y < this.overlayCanvas.height * 0.58;
    const rawLabelX = placeRight ? x + 14 : x - boxWidth - 14;
    const rawLabelY = placeBelow ? y + 14 : y - boxHeight - 14;
    const labelX = Math.min(Math.max(rawLabelX, 4), Math.max(4, this.overlayCanvas.width - boxWidth - 4));
    const labelY = Math.min(Math.max(rawLabelY, 4), Math.max(4, this.overlayCanvas.height - boxHeight - 4));

    this.overlayCtx.fillStyle = "rgba(4, 11, 21, 0.84)";
    this.overlayCtx.fillRect(labelX, labelY, boxWidth, boxHeight);
    this.overlayCtx.strokeStyle = "rgba(255, 210, 92, 0.82)";
    this.overlayCtx.strokeRect?.(labelX, labelY, boxWidth, boxHeight);
    this.overlayCtx.fillStyle = "rgba(255, 243, 211, 0.98)";
    lines.forEach((line, index) => {
      this.overlayCtx.fillText(line, labelX + paddingX, labelY + paddingY + index * lineHeight);
    });
    this.overlayCtx.restore();
  }

  drawTcpWorkspaceBoundary() {
    const boundary = this.tcpWorkspaceBoundary;
    if (!boundary || this.overlayCanvas.width <= 0 || this.overlayCanvas.height <= 0) {
      return;
    }

    const sourceSize = boundary.sourceSize || this.getCurrentImageSize();
    if (!sourceSize) {
      return;
    }

    const xScale = this.overlayCanvas.width / sourceSize.width;
    const yScale = this.overlayCanvas.height / sourceSize.height;
    const planes = Array.isArray(boundary.planes) && boundary.planes.length
      ? boundary.planes
      : [{ points: boundary.points }];
    const scaledPlanes = planes
      .map((plane) => ({
        ...plane,
        points: Array.isArray(plane.points)
          ? plane.points.map((point) => ({
            x: Number(point.x) * xScale,
            y: Number(point.y) * yScale,
          }))
          : [],
      }))
      .filter((plane) => (
        plane.points.length === 4
        && plane.points.every((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
      ));

    if (!scaledPlanes.length) {
      return;
    }

    this.overlayCtx.save();
    this.overlayCtx.lineWidth = 2.1;
    this.overlayCtx.setLineDash([7, 5]);
    scaledPlanes.forEach((plane, index) => {
      const primaryPlane = index === 0;
      this.overlayCtx.strokeStyle = primaryPlane
        ? "rgba(126, 220, 255, 0.96)"
        : "rgba(126, 220, 255, 0.70)";
      this.overlayCtx.fillStyle = primaryPlane
        ? "rgba(126, 220, 255, 0.08)"
        : "rgba(126, 220, 255, 0.04)";
      this.overlayCtx.beginPath();
      this.overlayCtx.moveTo(plane.points[0].x, plane.points[0].y);
      plane.points.slice(1).forEach((point) => this.overlayCtx.lineTo(point.x, point.y));
      this.overlayCtx.closePath();
      if (primaryPlane) {
        this.overlayCtx.fill();
      }
      this.overlayCtx.stroke();
    });

    if (scaledPlanes.length >= 2) {
      const [nearPlane, farPlane] = scaledPlanes;
      this.overlayCtx.strokeStyle = "rgba(126, 220, 255, 0.48)";
      this.overlayCtx.lineWidth = 1.6;
      this.overlayCtx.setLineDash([4, 5]);
      for (let index = 0; index < 4; index += 1) {
        const nearPoint = nearPlane.points[index];
        const farPoint = farPlane.points[index];
        this.overlayCtx.beginPath();
        this.overlayCtx.moveTo(nearPoint.x, nearPoint.y);
        this.overlayCtx.lineTo(farPoint.x, farPoint.y);
        this.overlayCtx.stroke();
      }
    }

    this.overlayCtx.setLineDash([]);
    this.overlayCtx.fillStyle = "rgba(126, 220, 255, 0.92)";
    this.overlayCtx.strokeStyle = "rgba(4, 11, 21, 0.86)";
    this.overlayCtx.lineWidth = 1.2;
    scaledPlanes[0].points.forEach((point) => {
      this.overlayCtx.beginPath();
      this.overlayCtx.arc(point.x, point.y, 4.5, 0, Math.PI * 2);
      this.overlayCtx.fill();
      this.overlayCtx.stroke();
    });
    this.overlayCtx.restore();
  }

  drawVisualRecognitionPoints() {
    const points = Array.isArray(this.lastVisualRecognitionPointsMessage?.PointCoordinatesArray)
      ? this.lastVisualRecognitionPointsMessage.PointCoordinatesArray
      : [];
    if (!points.length || this.overlayCanvas.width <= 0 || this.overlayCanvas.height <= 0) {
      return;
    }

    const sourceSize = this.visualRecognitionPointSourceSize || this.getCurrentImageSize();
    if (!sourceSize) {
      return;
    }

    const xScale = this.overlayCanvas.width / sourceSize.width;
    const yScale = this.overlayCanvas.height / sourceSize.height;
    this.overlayCtx.save();
    this.overlayCtx.font = "14px monospace";
    this.overlayCtx.lineWidth = 2;
    this.overlayCtx.textBaseline = "middle";
    points.forEach((point, index) => {
      const pixel = extractPointPixel(point);
      if (!pixel) {
        return;
      }
      const x = pixel.x * xScale;
      const y = pixel.y * yScale;
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        return;
      }
      const label = String(Number.isFinite(Number(point?.idx)) ? Number(point.idx) : index + 1);
      this.overlayCtx.beginPath();
      this.overlayCtx.arc(x, y, 7, 0, Math.PI * 2);
      this.overlayCtx.fillStyle = "rgba(7, 18, 12, 0.62)";
      this.overlayCtx.fill();
      this.overlayCtx.beginPath();
      this.overlayCtx.arc(x, y, 4.5, 0, Math.PI * 2);
      this.overlayCtx.fillStyle = "rgba(255, 246, 97, 0.96)";
      this.overlayCtx.fill();
      this.overlayCtx.lineWidth = 1.5;
      this.overlayCtx.strokeStyle = "rgba(126, 255, 118, 0.92)";
      this.overlayCtx.stroke();
      const textWidth = this.overlayCtx.measureText(label).width;
      const labelX = Math.min(Math.max(x + 10, 2), Math.max(2, this.overlayCanvas.width - textWidth - 8));
      const labelY = Math.min(Math.max(y - 12, 10), Math.max(10, this.overlayCanvas.height - 10));
      this.overlayCtx.fillStyle = "rgba(8, 17, 34, 0.78)";
      this.overlayCtx.fillRect(labelX - 3, labelY - 8, textWidth + 6, 16);
      this.overlayCtx.fillStyle = "#ffffff";
      this.overlayCtx.fillText(label, labelX, labelY);
    });
    this.overlayCtx.restore();
  }

  drawWorkspacePolylines() {
    this.ctx.save();
    this.ctx.lineWidth = 2;
    this.ctx.font = "18px monospace";

    if (this.isWorkspacePickingActive() && this.selectedPoints.length) {
      this.ctx.strokeStyle = "#4de3a5";
      this.ctx.fillStyle = "#ffae42";
      if (this.selectedPoints.length >= 2) {
        this.ctx.beginPath();
        this.ctx.moveTo(this.selectedPoints[0].x, this.selectedPoints[0].y);
        for (let index = 1; index < this.selectedPoints.length; index += 1) {
          this.ctx.lineTo(this.selectedPoints[index].x, this.selectedPoints[index].y);
        }
        if (this.selectedPoints.length === 4) {
          this.ctx.closePath();
        }
        this.ctx.stroke();
      }
      this.selectedPoints.forEach((point, index) => {
        this.ctx.beginPath();
        this.ctx.arc(point.x, point.y, 5, 0, Math.PI * 2);
        this.ctx.fill();
        this.ctx.strokeStyle = "#ffffff";
        this.ctx.stroke();
        this.ctx.fillStyle = "#4de3a5";
        this.ctx.fillText(`${index + 1}`, point.x + 8, point.y - 8);
        this.ctx.fillStyle = "#ffae42";
      });
    }

    this.ctx.restore();
  }

  handleCanvasClick(event) {
    if (!this.isWorkspacePickingActive()) {
      return;
    }
    if (this.suppressNextCanvasClick) {
      this.suppressNextCanvasClick = false;
      return;
    }
    if (!this.lastImageMessage) {
      this.onMessage?.("IR 图像还没到，先等一帧");
      return;
    }
    if (this.selectedPoints.length >= 4) {
      this.onMessage?.("已经点满 4 个角点了，先清空或撤销再继续");
      return;
    }
    const point = mapCanvasClickToImagePixel({
      clientX: event.clientX,
      clientY: event.clientY,
      rect: this.canvas.getBoundingClientRect(),
      imageWidth: Number(this.lastImageMessage.width),
      imageHeight: Number(this.lastImageMessage.height),
    });
    this.selectedPoints = [...this.selectedPoints, point];
    this.notifySelectionChanged();
    this.draw();
  }

  handlePointerDown(event) {
    if (!this.isWorkspacePickingActive()) {
      return;
    }
    if (!this.lastImageMessage || !this.selectedPoints.length) {
      return;
    }
    const point = mapCanvasClickToImagePixel({
      clientX: event.clientX,
      clientY: event.clientY,
      rect: this.canvas.getBoundingClientRect(),
      imageWidth: Number(this.lastImageMessage.width),
      imageHeight: Number(this.lastImageMessage.height),
    });
    const dragIndex = findDraggablePointIndex(this.selectedPoints, point, 12);
    if (dragIndex < 0) {
      return;
    }
    this.dragState = { activeIndex: dragIndex, moved: false };
    this.canvas.setPointerCapture?.(event.pointerId);
  }

  handlePointerMove(event) {
    if (!this.isWorkspacePickingActive()) {
      this.emitHoverPixelFromPointerEvent(event);
      return;
    }
    if (this.dragState.activeIndex < 0 || !this.lastImageMessage) {
      return;
    }
    const point = mapCanvasClickToImagePixel({
      clientX: event.clientX,
      clientY: event.clientY,
      rect: this.canvas.getBoundingClientRect(),
      imageWidth: Number(this.lastImageMessage.width),
      imageHeight: Number(this.lastImageMessage.height),
    });
    this.selectedPoints = replaceWorkspacePoint(this.selectedPoints, this.dragState.activeIndex, point);
    this.dragState = { ...this.dragState, moved: true };
    this.draw();
  }

  handlePointerUp() {
    if (this.dragState.activeIndex < 0) {
      return;
    }
    const moved = this.dragState.moved;
    this.dragState = { activeIndex: -1, moved: false };
    if (moved) {
      this.suppressNextCanvasClick = true;
      this.notifySelectionChanged();
    }
  }

  handlePointerLeave() {
    this.handlePointerUp();
    if (!this.isWorkspacePickingActive()) {
      this.onHoverPixelChanged?.(null);
      this.setHoverCoordinateReadout(null);
    }
  }

  emitHoverPixelFromPointerEvent(event) {
    if (!this.lastImageMessage) {
      this.onHoverPixelChanged?.(null);
      return;
    }
    const point = mapCanvasClickToImagePixel({
      clientX: event.clientX,
      clientY: event.clientY,
      rect: this.canvas.getBoundingClientRect(),
      imageWidth: Number(this.lastImageMessage.width),
      imageHeight: Number(this.lastImageMessage.height),
    });
    this.onHoverPixelChanged?.(point);
  }
}
