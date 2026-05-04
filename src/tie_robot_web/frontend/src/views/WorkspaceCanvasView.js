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

export class WorkspaceCanvasView {
  constructor({ canvas, overlayCanvas, onSelectionChanged, onMessage }) {
    this.canvas = canvas;
    this.overlayCanvas = overlayCanvas;
    this.ctx = canvas.getContext("2d");
    this.overlayCtx = overlayCanvas.getContext("2d");
    this.onSelectionChanged = onSelectionChanged;
    this.onMessage = onMessage;
    this.lastImageMessage = null;
    this.lastExecutionResultMessage = null;
    this.lastVisualRecognitionPointsMessage = null;
    this.visualRecognitionPointSourceSize = null;
    this.tcpWorkspaceBoundary = null;
    this.savedWorkspacePoints = [];
    this.selectedPoints = [];
    this.displaySettings = { mode: "raw", gamma: 1.0, overlayOpacity: 0.88 };
    this.overlayEnabled = true;
    this.savedWorkspaceGuideVisible = false;
    this.dragState = { activeIndex: -1, moved: false };
    this.suppressNextCanvasClick = false;
  }

  bindPointerEvents() {
    this.canvas.addEventListener("click", (event) => this.handleCanvasClick(event));
    this.canvas.addEventListener("pointerdown", (event) => this.handlePointerDown(event));
    this.canvas.addEventListener("pointermove", (event) => this.handlePointerMove(event));
    this.canvas.addEventListener("pointerup", () => this.handlePointerUp());
    this.canvas.addEventListener("pointerleave", () => this.handlePointerUp());
  }

  setDisplaySettings(settings) {
    this.displaySettings = { ...this.displaySettings, ...settings };
    this.overlayCanvas.style.opacity = String(this.displaySettings.overlayOpacity);
    this.draw();
  }

  setOverlayEnabled(enabled) {
    this.overlayEnabled = Boolean(enabled);
    this.drawOverlay();
  }

  setSavedWorkspaceGuideVisible(enabled) {
    this.savedWorkspaceGuideVisible = Boolean(enabled);
    this.draw();
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
    void boundary;
    this.tcpWorkspaceBoundary = null;
    this.drawOverlay();
  }

  setOverlaySource(source) {
    void source;
    this.drawOverlay();
  }

  setSavedWorkspacePoints(points) {
    this.savedWorkspacePoints = Array.isArray(points) ? points : [];
    this.draw();
  }

  setSavedWorkspacePayload(payload) {
    this.savedWorkspacePoints = parseWorkspaceQuadPayload(payload);
    this.draw();
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
    if (this.overlayEnabled && this.lastExecutionResultMessage) {
      const imageData = sensorImageToImageData(this.lastExecutionResultMessage, { mode: "raw", gamma: 1.0, overlayOpacity: 1.0 });
      if (imageData.width === this.overlayCanvas.width && imageData.height === this.overlayCanvas.height) {
        this.overlayCtx.putImageData(imageData, 0, 0);
      }
    }
    this.drawVisualRecognitionPoints();
    this.overlayCanvas.style.opacity = String(this.displaySettings.overlayOpacity);
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
      this.overlayCtx.fillStyle = "rgba(255, 45, 85, 0.92)";
      this.overlayCtx.fill();
      this.overlayCtx.strokeStyle = "rgba(255, 255, 255, 0.95)";
      this.overlayCtx.stroke();
      this.overlayCtx.beginPath();
      this.overlayCtx.moveTo(x - 11, y);
      this.overlayCtx.lineTo(x + 11, y);
      this.overlayCtx.moveTo(x, y - 11);
      this.overlayCtx.lineTo(x, y + 11);
      this.overlayCtx.strokeStyle = "rgba(255, 255, 255, 0.85)";
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

    if (this.savedWorkspaceGuideVisible && this.savedWorkspacePoints.length >= 2) {
      this.ctx.strokeStyle = "#6aa6ff";
      this.ctx.fillStyle = "#6aa6ff";
      this.ctx.setLineDash([10, 6]);
      this.ctx.beginPath();
      this.ctx.moveTo(this.savedWorkspacePoints[0].x, this.savedWorkspacePoints[0].y);
      for (let index = 1; index < this.savedWorkspacePoints.length; index += 1) {
        this.ctx.lineTo(this.savedWorkspacePoints[index].x, this.savedWorkspacePoints[index].y);
      }
      if (this.savedWorkspacePoints.length === 4) {
        this.ctx.closePath();
      }
      this.ctx.stroke();
      this.ctx.setLineDash([]);
    }

    if (this.selectedPoints.length) {
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
}
