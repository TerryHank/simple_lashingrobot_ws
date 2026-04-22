import { t as ROSLIBFactory } from './vendor/roslib-CFvqKDwv.js';
import {
  enhanceMono8,
  buildWorkspaceQuadPayload,
  findDraggablePointIndex,
  mapCanvasClickToImagePixel,
  parseWorkspaceQuadPayload,
  replaceWorkspacePoint,
  resolveRosbridgeUrl,
} from './ir_workspace_picker_helpers.mjs';

const CONNECTION_PREFERENCES_KEY = 'ros_web_gui_connection_preferences';
const DISPLAY_PREFERENCES_KEY = 'ir_workspace_picker_display_preferences';
const IR_TOPIC = '/Scepter/ir/image_raw';
const WORKSPACE_QUAD_TOPIC = '/web/pointAI/set_workspace_quad';
const RUN_WORKSPACE_S2_TOPIC = '/web/pointAI/run_workspace_s2';
const PSEUDO_SLAM_SCAN_TOPIC = '/web/cabin/start_pseudo_slam_scan';
const EXECUTION_MODE_TOPIC = '/web/cabin/set_execution_mode';
const START_GLOBAL_WORK_TOPIC = '/web/cabin/start_global_work';
const CURRENT_WORKSPACE_QUAD_TOPIC = '/pointAI/manual_workspace_quad_pixels';
const WORKSPACE_S2_RESULT_TOPIC = '/pointAI/manual_workspace_s2_result_raw';
const EXECUTION_RESULT_TOPIC = '/pointAI/result_image_raw';

const statusEl = document.getElementById('status');
const connectionEl = document.getElementById('connection');
const canvas = document.getElementById('irCanvas');
const s2OverlayCanvas = document.getElementById('s2OverlayCanvas');
const pointsEl = document.getElementById('points');
const clearBtn = document.getElementById('clearPoints');
const undoBtn = document.getElementById('undoPoint');
const submitBtn = document.getElementById('submitQuad');
const runSavedS2Btn = document.getElementById('runSavedS2');
const moveToWorkspaceCenterScanPoseBtn = document.getElementById('moveToWorkspaceCenterScanPose');
const startExecutionBtn = document.getElementById('startExecution');
const startExecutionClearMemoryBtn = document.getElementById('startExecutionClearMemory');
const displayModeEl = document.getElementById('displayMode');
const gammaRangeEl = document.getElementById('gammaRange');
const gammaValueEl = document.getElementById('gammaValue');
const overlayOpacityRangeEl = document.getElementById('overlayOpacityRange');
const overlayOpacityValueEl = document.getElementById('overlayOpacityValue');
const resultStatusEl = document.getElementById('resultStatus');
const ctx = canvas.getContext('2d');
const s2OverlayCtx = s2OverlayCanvas.getContext('2d');
const ROSLIB = typeof ROSLIBFactory === 'function' ? ROSLIBFactory() : ROSLIBFactory;

let ros = null;
let irSubscriber = null;
let quadPublisher = null;
let runS2Publisher = null;
let pseudoSlamScanPublisher = null;
let executionModePublisher = null;
let startGlobalWorkPublisher = null;
let currentQuadSubscriber = null;
let s2ResultSubscriber = null;
let executionResultSubscriber = null;
let lastImageMessage = null;
let lastResultImageMessage = null;
let lastExecutionResultImageMessage = null;
let selectedPoints = [];
let savedWorkspacePoints = [];
let displaySettings = loadDisplayPreferences();
let overlaySource = 's2';
let dragState = {
  activeIndex: -1,
  pointerId: null,
  moved: false,
};
let suppressNextCanvasClick = false;

function updateActionButtons() {
  submitBtn.disabled = selectedPoints.length !== 4 || !quadPublisher || !runS2Publisher;
  runSavedS2Btn.disabled = savedWorkspacePoints.length !== 4 || !runS2Publisher;
  moveToWorkspaceCenterScanPoseBtn.disabled = !pseudoSlamScanPublisher;
  startExecutionBtn.disabled = !executionModePublisher || !startGlobalWorkPublisher;
  startExecutionClearMemoryBtn.disabled = !executionModePublisher || !startGlobalWorkPublisher;
  undoBtn.disabled = selectedPoints.length === 0;
  clearBtn.disabled = selectedPoints.length === 0;
}

function loadSavedConnectionPreferences() {
  try {
    const raw = localStorage.getItem(CONNECTION_PREFERENCES_KEY);
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
}

function loadDisplayPreferences() {
  const defaults = { mode: 'auto', gamma: 0.85, overlayOpacity: 0.88 };
  try {
    const raw = localStorage.getItem(DISPLAY_PREFERENCES_KEY);
    if (!raw) {
      return defaults;
    }
    const parsed = JSON.parse(raw);
    return {
      mode: ['raw', 'auto', 'strong'].includes(parsed?.mode) ? parsed.mode : defaults.mode,
      gamma: Number.isFinite(parsed?.gamma) ? parsed.gamma : defaults.gamma,
      overlayOpacity: Number.isFinite(parsed?.overlayOpacity) ? parsed.overlayOpacity : defaults.overlayOpacity,
    };
  } catch {
    return defaults;
  }
}

function saveDisplayPreferences() {
  try {
    localStorage.setItem(DISPLAY_PREFERENCES_KEY, JSON.stringify(displaySettings));
  } catch {
    // ignore storage failures
  }
}

function setStatus(message, level = 'info') {
  statusEl.textContent = message;
  statusEl.dataset.level = level;
}

function syncDisplayControls() {
  if (displayModeEl) {
    displayModeEl.value = displaySettings.mode;
  }
  if (gammaRangeEl) {
    gammaRangeEl.value = displaySettings.gamma.toFixed(2);
  }
  if (gammaValueEl) {
    gammaValueEl.textContent = displaySettings.gamma.toFixed(2);
  }
  if (overlayOpacityRangeEl) {
    overlayOpacityRangeEl.value = displaySettings.overlayOpacity.toFixed(2);
  }
  if (overlayOpacityValueEl) {
    overlayOpacityValueEl.textContent = displaySettings.overlayOpacity.toFixed(2);
  }
}

function applyOverlayOpacity() {
  if (!s2OverlayCanvas) {
    return;
  }
  s2OverlayCanvas.style.opacity = String(displaySettings.overlayOpacity);
}

function decodeImageBytes(data) {
  if (Array.isArray(data)) {
    return Uint8ClampedArray.from(data);
  }

  if (typeof data === 'string') {
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

function sensorImageToImageData(message, enhancementSettings = displaySettings) {
  const bytes = decodeImageBytes(message.data);
  if (!bytes) {
    throw new Error('无法解析 IR 图像数据');
  }

  const width = Number(message.width) || 0;
  const height = Number(message.height) || 0;
  const encoding = String(message.encoding || '').toLowerCase();
  if (width <= 0 || height <= 0) {
    throw new Error('IR 图像尺寸无效');
  }

  const rgba = new Uint8ClampedArray(width * height * 4);

  if (encoding.includes('mono8') || encoding.includes('8uc1')) {
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

  if (encoding.includes('rgb8')) {
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

  if (encoding.includes('bgr8')) {
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

function drawImageMessageToCanvas(targetCanvas, targetCtx, message, options = displaySettings) {
  if (!targetCanvas || !targetCtx || !message) {
    return;
  }

  const imageData = sensorImageToImageData(message, options);

  targetCanvas.width = imageData.width;
  targetCanvas.height = imageData.height;
  targetCtx.putImageData(imageData, 0, 0);
}

function clearCanvas(targetCanvas, targetCtx) {
  if (!targetCanvas || !targetCtx) {
    return;
  }
  targetCtx.clearRect(0, 0, targetCanvas.width, targetCanvas.height);
}

function updateDisplaySettings(nextSettings) {
  displaySettings = {
    ...displaySettings,
    ...nextSettings,
  };
  syncDisplayControls();
  applyOverlayOpacity();
  saveDisplayPreferences();
  drawOverlay();
}

function drawOverlay() {
  if (!lastImageMessage) {
    return;
  }

  drawImageMessageToCanvas(canvas, ctx, lastImageMessage, displaySettings);

  ctx.save();
  ctx.lineWidth = 2;
  ctx.font = '18px monospace';

  if (savedWorkspacePoints.length >= 2) {
    ctx.strokeStyle = '#6aa6ff';
    ctx.fillStyle = '#6aa6ff';
    ctx.setLineDash([10, 6]);
    ctx.beginPath();
    ctx.moveTo(savedWorkspacePoints[0].x, savedWorkspacePoints[0].y);
    for (let index = 1; index < savedWorkspacePoints.length; index += 1) {
      ctx.lineTo(savedWorkspacePoints[index].x, savedWorkspacePoints[index].y);
    }
    if (savedWorkspacePoints.length === 4) {
      ctx.closePath();
    }
    ctx.stroke();
    ctx.setLineDash([]);
    savedWorkspacePoints.forEach((point, index) => {
      ctx.beginPath();
      ctx.arc(point.x, point.y, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillText(`S${index + 1}`, point.x + 8, point.y + 18);
    });
  }

  if (selectedPoints.length) {
    ctx.strokeStyle = '#4de3a5';
    ctx.fillStyle = '#ffae42';
    if (selectedPoints.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(selectedPoints[0].x, selectedPoints[0].y);
      for (let index = 1; index < selectedPoints.length; index += 1) {
        ctx.lineTo(selectedPoints[index].x, selectedPoints[index].y);
      }
      if (selectedPoints.length === 4) {
        ctx.closePath();
      }
      ctx.stroke();
    }

    selectedPoints.forEach((point, index) => {
      ctx.beginPath();
      ctx.arc(point.x, point.y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.stroke();
      ctx.fillStyle = '#4de3a5';
      ctx.fillText(`${index + 1}`, point.x + 8, point.y - 8);
      ctx.fillStyle = '#ffae42';
    });
  }

  ctx.restore();
  drawS2Overlay();
}

function drawS2Overlay() {
  const overlayMessage =
    overlaySource === 'execution' ? lastExecutionResultImageMessage : lastResultImageMessage;

  if (!overlayMessage) {
    clearCanvas(s2OverlayCanvas, s2OverlayCtx);
    return;
  }

  drawImageMessageToCanvas(s2OverlayCanvas, s2OverlayCtx, overlayMessage, { mode: 'raw', gamma: 1.0 });
}

function refreshPointsList() {
  pointsEl.innerHTML = '';
  selectedPoints.forEach((point, index) => {
    const item = document.createElement('li');
    item.textContent = `${index + 1}. 角点 ${index + 1} 已选`;
    pointsEl.appendChild(item);
  });

  if (!selectedPoints.length) {
    const item = document.createElement('li');
    item.textContent = '还没有点，直接点击 IR 图';
    pointsEl.appendChild(item);
  }
  updateActionButtons();
}

function publishWorkspaceQuad() {
  if (!quadPublisher || !runS2Publisher || selectedPoints.length !== 4) {
    return;
  }

  overlaySource = 's2';
  const payload = buildWorkspaceQuadPayload(selectedPoints);
  const message = new ROSLIB.Message({ data: payload });
  quadPublisher.publish(message);
  lastResultImageMessage = null;
  drawS2Overlay();
  setStatus(`工作区四边形已发送，正在触发 S2: [${payload.join(', ')}]`, 'success');
  if (resultStatusEl) {
    resultStatusEl.textContent = '正在等待 pointAI 返回 S2 result...';
  }
  window.setTimeout(() => {
    runS2Publisher.publish(new ROSLIB.Message({ data: true }));
  }, 180);
}

function triggerSavedWorkspaceS2() {
  if (!runS2Publisher || savedWorkspacePoints.length !== 4) {
    setStatus('还没有已保存的工作区四边形，先提交一次工作区', 'warn');
    if (resultStatusEl) {
      resultStatusEl.textContent = '当前没有可复用的已保存工作区，请先点 4 个角点并提交。';
    }
    return;
  }

  overlaySource = 's2';
  lastResultImageMessage = null;
  drawS2Overlay();
  setStatus('已使用当前已保存工作区触发 S2，正在等待结果...', 'success');
  if (resultStatusEl) {
    resultStatusEl.textContent = '正在使用当前已保存工作区识别绑扎点...';
  }
  runS2Publisher.publish(new ROSLIB.Message({ data: true }));
}

function triggerWorkspaceCenterScanPoseMove() {
  if (!pseudoSlamScanPublisher) {
    setStatus('ROS 还没连好，暂时不能开始固定扫描规划', 'warn');
    return;
  }

  overlaySource = 's2';
  lastResultImageMessage = null;
  drawS2Overlay();
  pseudoSlamScanPublisher.publish(new ROSLIB.Message({ data: 5.0 }));
  setStatus('已触发固定扫描建图，后端将自动移动、识别并动态规划', 'success');
  if (resultStatusEl) {
    resultStatusEl.textContent =
      '正在执行固定工作区扫描：移动到 x=-260, y=1700, z=2997, speed=100，然后触发 S2 并动态规划。';
  }
}

function triggerExecutionLayer(clearExecutionMemory = false) {
  if (!executionModePublisher || !startGlobalWorkPublisher) {
    setStatus('ROS 还没连好，暂时不能开始执行层', 'warn');
    return;
  }

  overlaySource = 'execution';
  lastExecutionResultImageMessage = null;
  drawS2Overlay();

  executionModePublisher.publish(new ROSLIB.Message({ data: 1.0 }));
  setStatus(
    clearExecutionMemory
      ? '已切到视觉精校执行，正在清记忆并开始执行层...'
      : '已切到视觉精校执行，正在开始执行层...',
    'success',
  );
  if (resultStatusEl) {
    resultStatusEl.textContent =
      '执行层已切到 result_img 覆盖层，后续显示 /pointAI/result_image_raw。';
  }
  window.setTimeout(() => {
    startGlobalWorkPublisher.publish(
      new ROSLIB.Message({ data: clearExecutionMemory ? 2.0 : 1.0 }),
    );
  }, 120);
}

function handleCanvasClick(event) {
  if (suppressNextCanvasClick) {
    suppressNextCanvasClick = false;
    return;
  }

  if (!lastImageMessage) {
    setStatus('IR 图像还没到，先等一帧', 'warn');
    return;
  }

  if (selectedPoints.length >= 4) {
    setStatus('已经点满 4 个角点了，先清空或撤销再继续', 'warn');
    return;
  }

  const point = mapCanvasClickToImagePixel({
    clientX: event.clientX,
    clientY: event.clientY,
    rect: canvas.getBoundingClientRect(),
    imageWidth: Number(lastImageMessage.width),
    imageHeight: Number(lastImageMessage.height),
  });

  selectedPoints = [...selectedPoints, point];
  refreshPointsList();
  drawOverlay();

  if (selectedPoints.length === 4) {
    setStatus('4 个角点已选完，可直接拖拽绿点微调，点“提交四边形”即可保存工作区', 'success');
  } else {
    setStatus(`已记录第 ${selectedPoints.length} 个角点`, 'info');
  }
}

function mapPointerEventToImagePixel(event) {
  return mapCanvasClickToImagePixel({
    clientX: event.clientX,
    clientY: event.clientY,
    rect: canvas.getBoundingClientRect(),
    imageWidth: Number(lastImageMessage.width),
    imageHeight: Number(lastImageMessage.height),
  });
}

function handleCanvasPointerDown(event) {
  if (!lastImageMessage || selectedPoints.length === 0) {
    return;
  }

  const point = mapPointerEventToImagePixel(event);
  const dragIndex = findDraggablePointIndex(selectedPoints, point, 12);
  if (dragIndex < 0) {
    return;
  }

  dragState = {
    activeIndex: dragIndex,
    pointerId: event.pointerId,
    moved: false,
  };
  if (typeof canvas.setPointerCapture === 'function') {
    canvas.setPointerCapture(event.pointerId);
  }
  setStatus(`正在拖拽第 ${dragIndex + 1} 个角点`, 'info');
}

function handleCanvasPointerMove(event) {
  if (dragState.activeIndex < 0 || !lastImageMessage) {
    return;
  }

  const point = mapPointerEventToImagePixel(event);
  selectedPoints = replaceWorkspacePoint(selectedPoints, dragState.activeIndex, point);
  dragState = {
    ...dragState,
    moved: true,
  };
  drawOverlay();
}

function handleCanvasPointerUp(event) {
  if (dragState.activeIndex < 0) {
    return;
  }

  if (typeof canvas.releasePointerCapture === 'function' && dragState.pointerId !== null) {
    try {
      canvas.releasePointerCapture(dragState.pointerId);
    } catch {
      // ignore release failures
    }
  }

  const finishedIndex = dragState.activeIndex;
  const moved = dragState.moved;
  dragState = {
    activeIndex: -1,
    pointerId: null,
    moved: false,
  };

  if (moved) {
    suppressNextCanvasClick = true;
    refreshPointsList();
    drawOverlay();
    if (selectedPoints.length === 4) {
      setStatus(`已调整第 ${finishedIndex + 1} 个角点，可继续拖拽微调或直接提交四边形`, 'success');
    } else {
      setStatus(`已调整第 ${finishedIndex + 1} 个角点`, 'info');
    }
  }
}

function clearPoints() {
  selectedPoints = [];
  refreshPointsList();
  drawOverlay();
  setStatus('已清空角点，重新点击 IR 图即可', 'info');
}

function undoPoint() {
  if (!selectedPoints.length) {
    return;
  }
  selectedPoints = selectedPoints.slice(0, -1);
  refreshPointsList();
  drawOverlay();
  setStatus('已撤销最后一个角点', 'info');
}

function connectRosbridge() {
  const rosbridgeUrl = resolveRosbridgeUrl({
    pathname: window.location.pathname,
    savedPreferences: loadSavedConnectionPreferences(),
    currentHost: window.location.hostname,
  });
  connectionEl.textContent = rosbridgeUrl;
  setStatus(`正在连接 ${rosbridgeUrl} ...`, 'info');

  ros = new ROSLIB.Ros({ url: rosbridgeUrl });
  irSubscriber = new ROSLIB.Topic({
    ros,
    name: IR_TOPIC,
    messageType: 'sensor_msgs/Image',
  });
  quadPublisher = new ROSLIB.Topic({
    ros,
    name: WORKSPACE_QUAD_TOPIC,
    messageType: 'std_msgs/Float32MultiArray',
  });
  runS2Publisher = new ROSLIB.Topic({
    ros,
    name: RUN_WORKSPACE_S2_TOPIC,
    messageType: 'std_msgs/Bool',
  });
  pseudoSlamScanPublisher = new ROSLIB.Topic({
    ros,
    name: PSEUDO_SLAM_SCAN_TOPIC,
    messageType: 'std_msgs/Float32',
  });
  executionModePublisher = new ROSLIB.Topic({
    ros,
    name: EXECUTION_MODE_TOPIC,
    messageType: 'std_msgs/Float32',
  });
  startGlobalWorkPublisher = new ROSLIB.Topic({
    ros,
    name: START_GLOBAL_WORK_TOPIC,
    messageType: 'std_msgs/Float32',
  });
  currentQuadSubscriber = new ROSLIB.Topic({
    ros,
    name: CURRENT_WORKSPACE_QUAD_TOPIC,
    messageType: 'std_msgs/Float32MultiArray',
  });
  s2ResultSubscriber = new ROSLIB.Topic({
    ros,
    name: WORKSPACE_S2_RESULT_TOPIC,
    messageType: 'sensor_msgs/Image',
  });
  executionResultSubscriber = new ROSLIB.Topic({
    ros,
    name: EXECUTION_RESULT_TOPIC,
    messageType: 'sensor_msgs/Image',
  });

  ros.on('connection', () => {
    setStatus('ROS 连接成功，等待 IR 图像...', 'success');
    quadPublisher.advertise();
    runS2Publisher.advertise();
    pseudoSlamScanPublisher.advertise();
    executionModePublisher.advertise();
    startGlobalWorkPublisher.advertise();
    updateActionButtons();
    currentQuadSubscriber.subscribe((message) => {
      try {
        savedWorkspacePoints = parseWorkspaceQuadPayload(Array.from(message.data || []));
        updateActionButtons();
        drawOverlay();
      } catch (error) {
        console.warn('无法解析当前已保存工作区四边形', error);
      }
    });
    s2ResultSubscriber.subscribe((message) => {
      lastResultImageMessage = message;
      if (overlaySource !== 'execution') {
        overlaySource = 's2';
      }
      drawS2Overlay();
      if (resultStatusEl) {
        resultStatusEl.textContent = '已收到最新 S2 result，当前这层覆盖直接叠在 IR 选点底图上。';
      }
      setStatus('工作区已保存，S2 result 已更新', 'success');
    });
    executionResultSubscriber.subscribe((message) => {
      lastExecutionResultImageMessage = message;
      if (overlaySource === 'execution') {
        drawS2Overlay();
      }
      if (resultStatusEl && overlaySource === 'execution') {
        resultStatusEl.textContent = '执行层 result_img 已更新，当前覆盖层正在显示 /pointAI/result_image_raw。';
      }
      if (overlaySource === 'execution') {
        setStatus('执行层识别图层已更新', 'success');
      }
    });
    irSubscriber.subscribe((message) => {
      lastImageMessage = message;
      drawOverlay();
      if (selectedPoints.length === 0) {
        setStatus('IR 图像已就绪，直接在图上点 4 个角点', 'success');
      }
    });
  });

  ros.on('error', (error) => {
    setStatus(`ROS 连接失败: ${error?.message || error}`, 'error');
  });

  ros.on('close', () => {
    setStatus('ROS 连接已断开', 'warn');
    if (resultStatusEl) {
      resultStatusEl.textContent = 'ROS 已断开，S2 result 停止更新。';
    }
  });
}

canvas.addEventListener('pointerdown', handleCanvasPointerDown);
canvas.addEventListener('pointermove', handleCanvasPointerMove);
canvas.addEventListener('pointerup', handleCanvasPointerUp);
canvas.addEventListener('pointercancel', handleCanvasPointerUp);
canvas.addEventListener('click', handleCanvasClick);
clearBtn.addEventListener('click', clearPoints);
undoBtn.addEventListener('click', undoPoint);
submitBtn.addEventListener('click', publishWorkspaceQuad);
runSavedS2Btn.addEventListener('click', triggerSavedWorkspaceS2);
moveToWorkspaceCenterScanPoseBtn.addEventListener('click', triggerWorkspaceCenterScanPoseMove);
startExecutionBtn.addEventListener('click', () => triggerExecutionLayer(false));
startExecutionClearMemoryBtn.addEventListener('click', () => triggerExecutionLayer(true));
displayModeEl.addEventListener('change', (event) => {
  updateDisplaySettings({ mode: event.target.value });
});
gammaRangeEl.addEventListener('input', (event) => {
  updateDisplaySettings({ gamma: Number(event.target.value) });
});
overlayOpacityRangeEl.addEventListener('input', (event) => {
  updateDisplaySettings({ overlayOpacity: Number(event.target.value) });
});

refreshPointsList();
syncDisplayControls();
applyOverlayOpacity();
connectRosbridge();
