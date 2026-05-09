import { SceneAdapter } from "../data/SceneAdapter.js";
import { DEFAULT_IMAGE_TOPIC, getImageTopicLabel, isOverlayCompatibleImageTopic } from "../config/imageTopicCatalog.js";
import {
  DEFAULT_LOG_TOPIC,
  getLogTopicLabel,
  matchesLogTopicFilter,
} from "../config/logTopicCatalog.js";
import { DEFAULT_LAYOUTS } from "../layout/defaultLayouts.js";
import { LayoutManager } from "../layout/LayoutManager.js";
import { PANEL_REGISTRY } from "../panels/panelRegistry.js";
import { ViewerStore } from "../state/ViewerStore.js";
import { LegacyCommandController } from "../controllers/LegacyCommandController.js";
import { RosConnectionController } from "../controllers/RosConnectionController.js";
import { StatusMonitorController } from "../controllers/StatusMonitorController.js";
import { SystemControlController } from "../controllers/SystemControlController.js";
import { TaskActionController } from "../controllers/TaskActionController.js";
import { AreaNavigationController } from "../controllers/AreaNavigationController.js";
import { CabinRemoteController } from "../controllers/CabinRemoteController.js";
import { TcpLinearRemoteController } from "../controllers/TcpLinearRemoteController.js";
import { TerminalController } from "../controllers/TerminalController.js";
import { TopicLayerController } from "../controllers/TopicLayerController.js";
import { PanelManager } from "../ui/PanelManager.js";
import { UIController } from "../ui/UIController.js";
import {
  loadCabinRemoteSettings,
  loadDisplayPreferences,
  loadNetworkPingSettings,
  loadRecognitionPose,
  loadSettingsHomePagePreference,
  loadSettingsPageOrderPreference,
  loadTcpLinearRemoteSettings,
  loadThemePreference,
  loadTopicLayerStatePreference,
  loadVisualDebugSettings,
  loadViewerLayout,
  saveCabinRemoteSettings,
  saveDisplayPreferences,
  saveNetworkPingSettings,
  saveRecognitionPose,
  saveSettingsHomePagePreference,
  saveSettingsPageOrderPreference,
  saveTcpLinearRemoteSettings,
  saveThemePreference,
  saveTopicLayerStatePreference,
  saveVisualDebugSettings,
  saveViewerLayout,
} from "../utils/storage.js";
import {
  consumeCabinRemoteKeyboardEvent,
  resolveCabinRemoteKeyboardActionFromKey,
  resolveCabinRemoteDirectionFromKey,
  shouldIgnoreCabinRemoteKeyboardTarget,
} from "../utils/cabinRemoteKeyboard.js";
import {
  normalizeCabinTelemetry,
  resolveCabinRemoteOperationState,
} from "../utils/cabinRemoteOperationState.js";
import { formatCabinRemoteProtocolFeedback } from "../utils/cabinRemoteProtocolFeedback.js";
import { sampleFloat32XYZImagePixel } from "../utils/irImageUtils.js";
import { inferLegacyLogLevel, sanitizeRosLogText } from "../utils/logText.js";
import { Scene3DView } from "../views/Scene3DView.js";
import { WorkspaceCanvasView } from "../views/WorkspaceCanvasView.js";
import { TOPICS } from "../config/topicRegistry.js";
import { getImageHoverCoordinateFrameLabel } from "../config/topicLayerCatalog.js";

const DIRECT_CABIN_MOVE_TARGET = Object.freeze({
  x: 490,
  y: 1700,
  z: 3197,
});
const PLANNING_AREA_REFRESH_DELAY_MS = 180;
const S2_RESULT_TIMEOUT_MS = 6000;
const SETTINGS_LAYER_LOG_HISTORY_LIMIT = 50;
const SETTINGS_LAYER_LOG_TOTAL_LIMIT = 500;
const IMAGE_HOVER_COORDINATE_IDLE_UNSUBSCRIBE_MS = 1800;
const LINEAR_MODULE_ZERO_LEGACY_COMMAND_ID = 15;

const DRIVER_LAYER_LOG_NODES = new Map([
  ["suoquNode", "索驱/流程主控"],
  ["moduanNode", "线性模组驱动"],
  ["scepter_manager", "相机驱动"],
  ["scepter_world_coord_processor", "相机点云处理"],
  ["gripper_tf_broadcaster", "相机-TCP TF"],
]);

const ALGORITHM_LAYER_LOG_NODES = new Map([
  ["pointAINode", "PointAI 识别"],
]);

const SETTINGS_LAYER_LOG_GROUPS = [
  { id: "driver", label: "驱动层", nodes: DRIVER_LAYER_LOG_NODES },
  { id: "algorithm", label: "算法层", nodes: ALGORITHM_LAYER_LOG_NODES },
];

const VISUAL_FRAME_SYNC_TASK_ACTIONS = new Set([
  "runSavedS2",
  "executionVisionOnly",
  "triggerSingleBind",
  "startExecution",
  "startExecutionKeepMemory",
]);

export class TieRobotFrontApp {
  constructor(rootElement) {
    this.rootElement = rootElement;
    this.logs = [];
    this.frontendLogs = [];
    this.systemLogs = [];
    this.layerSystemLogs = [];
    this.visualDebugLogs = [];
    this.selectedLogTopicId = DEFAULT_LOG_TOPIC;
    this.s2ResultTimeoutId = null;
    this.planningAreaRefreshTimerId = null;
    this.imageHoverCoordinateIdleTimerId = null;
    this.planningAreaRequestToken = 0;
    this.surfaceDpOverlayActive = false;
    this.surfaceDpOverlayRequested = false;
    this.visualRecognitionOverlayCompleted = false;
    this.visualRecognitionOverlayCleared = false;
    this.latestVisualRecognitionPointsMessage = null;
    this.latestAreaProgress = null;
    this.irCameraInfo = null;
    this.displayedImageTopicName = DEFAULT_IMAGE_TOPIC;
    this.cabinTelemetry = normalizeCabinTelemetry(null);
    this.cabinRemoteMoveInFlight = false;
    this.tcpLinearRemoteMoveInFlight = false;
    this.robotHomeCalibration = null;
    this.graphicalAppSessions = [];
    this.graphicalAppSessionSignature = "";
    this.graphicalAppSessionPollTimer = null;
    this.demoModeStatusPollTimer = null;
    this.handleCabinRemoteKeyDown = this.handleCabinRemoteKeyDown.bind(this);
    this.handleWindowBeforeUnload = this.handleWindowBeforeUnload.bind(this);
    this.handleGraphicalAppFrameMessage = this.handleGraphicalAppFrameMessage.bind(this);
    this.displaySettings = loadDisplayPreferences();
    this.cabinRemoteSettings = loadCabinRemoteSettings();
    this.tcpLinearRemoteSettings = loadTcpLinearRemoteSettings();
    this.networkPingSettings = loadNetworkPingSettings();
    this.recognitionPose = loadRecognitionPose(DIRECT_CABIN_MOVE_TARGET);
    this.visualDebugSettings = loadVisualDebugSettings();
    this.topicLayerState = loadTopicLayerStatePreference();
    this.settingsHomePage = loadSettingsHomePagePreference();
    this.activeSettingsPage = this.settingsHomePage;
    this.settingsPageOrder = loadSettingsPageOrderPreference();
    this.theme = loadThemePreference();
    document.documentElement.setAttribute("data-theme", this.theme);
    this.viewerStore = new ViewerStore();
    this.sceneAdapter = new SceneAdapter();
    this.panelRegistry = PANEL_REGISTRY;
    this.layoutManager = new LayoutManager({
      defaults: DEFAULT_LAYOUTS,
      loadLayout: loadViewerLayout,
      saveLayout: saveViewerLayout,
    });
    this.activeLayout = this.layoutManager.getLayout("executionDebug");

    this.ui = new UIController(rootElement, {
      panelRegistry: this.panelRegistry,
      initialLayout: this.activeLayout,
      settingsPageOrder: this.settingsPageOrder,
    });
    this.ui.renderShell();
    this.renderControlPanelTasks();
    this.settingsPageOrder = this.ui.setSettingsPageOrder(this.settingsPageOrder);
    this.settingsHomePage = this.ui.setSettingsHomePage(this.settingsHomePage);
    const homeFirstPageOrder = this.ui.getSettingsPageOrder();
    if (homeFirstPageOrder.join("\u0000") !== this.settingsPageOrder.join("\u0000")) {
      this.settingsPageOrder = homeFirstPageOrder;
      saveSettingsPageOrderPreference(homeFirstPageOrder);
    }
    this.ui.setSettingsPage(this.settingsHomePage);
    this.ui.renderPanelsFromLayout(this.activeLayout);
    this.ui.setTheme(this.theme);
    this.ui.setCabinRemoteSettings(this.cabinRemoteSettings);
    this.ui.setTcpLinearRemoteSettings(this.tcpLinearRemoteSettings);
    this.ui.setNetworkPingSettings(this.networkPingSettings);

    this.panelManager = new PanelManager({
      onLayoutChange: () => this.persistActiveLayout(),
    });
    this.panelManager.init(rootElement);
    this.panelManager.applyPanelLayout(this.activeLayout);

    const canvasRefs = this.ui.getCanvasRefs();
    this.workspaceView = new WorkspaceCanvasView({
      canvas: canvasRefs.canvas,
      overlayCanvas: canvasRefs.overlayCanvas,
      onSelectionChanged: (points) => {
        this.ui.renderPointList(points);
        this.refreshActionState();
      },
      onMessage: (message) => this.addLog(message, "info"),
      onHoverPixelChanged: (pixel) => this.handleImageHoverPixelChanged(pixel),
    });
    this.workspaceView.bindPointerEvents();
    this.workspaceView.setDisplaySettings(this.displaySettings);
    this.workspaceView.setWorkspacePickingEnabled(this.activeSettingsPage === "workspace");
    this.workspaceView.setSavedWorkspaceGuideVisible(this.activeSettingsPage === "workspace");
    this.ui.setDisplaySettings(this.displaySettings);
    this.ui.setVisualDebugSettings(this.visualDebugSettings);
    this.ui.renderPointList([]);
    this.ui.renderSettingsLayerLogs(this.buildSettingsLayerLogViewModel());
    this.ui.renderVisualDebugLogs(this.visualDebugLogs);

    this.sceneView = new Scene3DView({
      container: this.ui.getSceneContainer(),
    });
    this.sceneView.setTheme(this.theme);
    this.applyVisualDebugBindRangeSettings(this.visualDebugSettings);

    this.topicLayerController = new TopicLayerController({
      ui: this.ui,
      sceneView: this.sceneView,
      initialState: this.topicLayerState,
      callbacks: {
        onLog: (message, level) => this.addLog(message, level),
      },
    });
    this.topicLayerController.init();
    this.applyImageOverlayLayerState();

    this.statusMonitorController = new StatusMonitorController({
      onStatusChip: (statusId, level, detail) => this.ui.setStatusChipState(statusId, level, detail),
      onBatteryVoltage: (voltage) => this.ui.setBatteryVoltage(voltage),
      onLightState: (enabled) => this.syncControlToggleStateFromTelemetry("lightEnabled", enabled),
      onAlarmState: (alarms) => this.ui.setConnectionAlarmState(alarms),
      onLog: (message, level) => this.addLog(message, level),
    });

    this.rosConnectionController = new RosConnectionController({
      onConnectionInfo: (url, message, level) => {
        this.ui.setConnectionInfo(url, message, level);
        this.statusMonitorController.setConnectionState(level, message);
        this.viewerStore.updateIn("connection", {
          url,
          message,
          level,
          ready: level === "success",
        });
      },
      onRosReady: (resources) => {
        this.statusMonitorController.start(resources.ros);
        this.viewerStore.updateIn("connection", { ready: true });
        this.syncDisplayedImageSubscription({ suppressLog: true });
        this.syncPointCloudSubscription({ suppressLog: true });
        this.syncImageHoverCoordinateSubscription({ suppressLog: true });
        this.syncLogSubscription({ suppressLog: true });
        this.syncGlobalCabinMoveSpeed({ suppressLog: true });
        this.syncGlobalLinearModuleSpeed({ suppressLog: true });
        this.applyVisualDebugRuntimeSettings(this.visualDebugSettings, { suppressLog: true });
        this.refreshRobotHomeCalibration({ suppressLog: true });
        this.schedulePlanningAreaRefresh();
        this.refreshActionState();
      },
      onRosUnavailable: () => {
        this.statusMonitorController.stop();
        this.clearS2ResultTimeout();
        this.clearPlanningAreaRefresh();
        this.clearImageHoverCoordinateSubscriptionIdle();
        this.planningAreaRequestToken += 1;
        this.sceneView.setPlanningAreaPayload(null);
        this.surfaceDpOverlayActive = false;
        this.surfaceDpOverlayRequested = false;
        this.visualRecognitionOverlayCleared = false;
        this.irCameraInfo = null;
        this.imageHoverWorldCoordMessage = null;
        this.workspaceView.setTcpWorkspaceBoundary(null);
        this.workspaceView.setHoverCoordinateReadout(null);
        this.legacyCommandController?.reset();
        this.syncControlToggleStatesToUiAndScene();
        this.ui.renderTopicInventory([]);
        this.ui.setCabinRemoteCurrentPosition(null);
        this.cabinTelemetry = normalizeCabinTelemetry(null);
        this.ui.setTcpLinearRemoteState(null);
        this.ui.setBottomLinearModulePosition(null, null);
        this.ui.setTcpLinearRemoteButtonsEnabled(false);
        this.ui.setGripperTfCalibration(null);
        this.ui.setRobotHomeCalibration(null);
        this.robotHomeCalibration = null;
        this.viewerStore.patch({
          connection: {
            ready: false,
            level: "warn",
            message: "连接失败",
            url: "",
          },
          scene: {
            filteredWorldCoordCount: 0,
            rawWorldCoordCount: 0,
            tiePointCount: 0,
            planningPointCount: 0,
            tfFrameCount: 0,
          },
        });
        this.clearInactivePointClouds({ clearSelectedSource: true });
        this.refreshActionState();
      },
      onLog: (message, level) => this.addLog(message, level),
      onTopicInventory: (topics) => {
        this.ui.renderTopicInventory(topics);
      },
      onSystemLog: (message, option) => {
        this.handleSystemLog(message, option);
      },
      onLayerSystemLog: (message) => {
        this.handleLayerSystemLog(message);
      },
      onBaseImage: (message) => {
        this.workspaceView.setBaseImageMessage(message);
      },
      onDisplayedImage: (message, topicName) => {
        this.displayedImageTopicName = topicName || DEFAULT_IMAGE_TOPIC;
        this.workspaceView.setBaseImageMessage(message);
        this.syncTcpWorkspaceBoundaryOverlay();
      },
      onIrCameraInfo: (message) => {
        this.irCameraInfo = message;
        this.syncTcpWorkspaceBoundaryOverlay();
      },
      onImageHoverWorldCoord: (message) => {
        this.imageHoverWorldCoordMessage = message;
      },
      onSavedWorkspacePayload: (payload) => {
        this.workspaceView.setSavedWorkspacePayload(payload);
        const confirmed = this.taskActionController.handleSavedWorkspacePayload(payload);
        if (confirmed) {
          this.workspaceView.setSelectedWorkspacePayload(payload);
        }
        this.refreshActionState();
      },
      onExecutionOverlay: (message) => {
        if (this.surfaceDpOverlayRequested) {
          return;
        }
        if (this.visualRecognitionOverlayCleared) {
          return;
        }
        if (this.surfaceDpOverlayActive) {
          return;
        }
        this.clearS2ResultTimeout();
        this.workspaceView.setExecutionOverlayMessage(message);
        this.ui.setControlFeedback("执行结果已更新；切到红外原图可查看叠加。");
      },
      onWorkspaceS2Overlay: (message) => {
        if (!this.surfaceDpOverlayRequested) {
          return;
        }
        this.showVisualRecognitionOverlayMessage(message);
      },
      onPointCloudImage: (source, message) => {
        this.sceneAdapter.normalizePointCloud(message, { source });
        const count = this.sceneView.setPointCloudImageMessage(source, message);
        this.topicLayerController.updateStats({ [`${source}Count`]: count });
        this.viewerStore.updateIn("scene", { [`${source}Count`]: count });
      },
      onTiePoints: (message) => {
        this.sceneAdapter.normalizeTiePoints(message);
        const count = this.sceneView.setTiePointsMessage(message);
        if (this.surfaceDpOverlayActive || this.surfaceDpOverlayRequested) {
          this.handleVisualRecognitionPointsMessage(message);
        }
        this.topicLayerController.updateStats({ tiePointCount: count });
        this.viewerStore.updateIn("scene", { tiePointCount: count });
      },
      onVisualRecognitionPoints: (message) => {
        this.handleVisualRecognitionPointsMessage(message);
      },
      onPlanningMarkers: (message) => {
        this.sceneAdapter.normalizePlanningMarkers(message);
        const count = this.sceneView.setPlanningMarkersMessage(message);
        this.schedulePlanningAreaRefresh();
        this.topicLayerController.updateStats({ planningPointCount: count });
        this.viewerStore.updateIn("scene", { planningPointCount: count });
      },
      onTfMessage: (message) => {
        this.sceneView.handleTfMessage(message);
        const sceneCabinPosition = this.sceneView.getCurrentCabinPositionMm();
        if (sceneCabinPosition) {
          this.cabinRemoteController?.setLastKnownCabinPosition(sceneCabinPosition);
        }
        const rawCabinPosition = this.cabinRemoteController?.getCurrentRawCabinPositionMm?.() || null;
        this.ui.setCabinRemoteCurrentPosition(rawCabinPosition || sceneCabinPosition);
        this.ui.setTaskButtonEnabled("setRecognitionPose", Boolean(rawCabinPosition));
        const tfFrameCount = this.sceneView.getKnownTransformCount();
        this.topicLayerController.updateStats({ tfFrameCount });
        this.viewerStore.updateIn("scene", { tfFrameCount });
        this.ui.setGripperTfCalibration(this.sceneView.getCameraToTcpCalibration());
        this.syncTcpWorkspaceBoundaryOverlay();
        this.syncCabinRemoteOperationState();
        this.refreshBottomLinearModulePosition();
      },
      onCabinState: (message) => {
        this.handleCabinStateMessage(message);
      },
      onAreaProgress: (message) => {
        this.handleAreaProgressMessage(message);
      },
      onLinearModuleState: (message) => {
        this.tcpLinearRemoteController.setCurrentState(message);
        this.ui.setTcpLinearRemoteState(message);
        this.refreshBottomLinearModulePosition();
      },
    });

    this.cabinRemoteController = new CabinRemoteController({
      rosConnection: this.rosConnectionController,
      sceneView: this.sceneView,
    });

    this.areaNavigationController = new AreaNavigationController({
      rosConnection: this.rosConnectionController,
      getCurrentCabinPosition: () =>
        this.cabinRemoteController.getCurrentRawCabinPositionMm()
        || this.cabinRemoteController.getCurrentCabinPositionMm(),
      getCabinSpeed: () => this.getGlobalCabinMoveSpeed(),
      callbacks: {
        onResultMessage: (message) => this.ui.setControlFeedback(message),
        onLog: (message, level) => this.addLog(message, level),
        onCabinMoved: (payload) => {
          this.cabinRemoteController.setLastKnownRawCabinPosition(payload);
          this.cabinRemoteController.setLastKnownCabinPosition(payload);
          this.refreshCabinRemoteCurrentPosition();
          this.schedulePlanningAreaRefresh();
        },
      },
    });

    this.tcpLinearRemoteController = new TcpLinearRemoteController({
      rosConnection: this.rosConnectionController,
    });

    this.taskActionController = new TaskActionController({
      rosConnection: this.rosConnectionController,
      workspaceView: this.workspaceView,
      getExecutionMode: () => this.visualDebugSettings?.executionMode,
      getAdaptiveBindGrouping: () => this.visualDebugSettings?.adaptiveBindGrouping,
      getBindExecutionCabinMinZ: () => this.visualDebugSettings?.bindExecutionCabinMinZMm,
      callbacks: {
        onResultMessage: (message) => this.ui.setControlFeedback(message),
        onLog: (message, level) => this.addLog(message, level),
        onWorkspaceS2Triggered: () => this.handleWorkspaceS2Triggered(),
      },
    });

    this.legacyCommandController = new LegacyCommandController({
      rosConnection: this.rosConnectionController,
      callbacks: {
        onResultMessage: (message) => this.ui.setControlFeedback(message),
        onLog: (message, level) => this.addLog(message, level),
        onPendingChange: (actionId, pending) => this.ui.setSystemActionPending(actionId, pending),
      },
    });
    this.systemControlController = new SystemControlController({
      rosConnection: this.rosConnectionController,
      callbacks: {
        onResultMessage: (message) => this.ui.setControlFeedback(message),
        onLog: (message, level) => this.addLog(message, level),
        onPendingChange: (actionId, pending) => this.ui.setSystemActionPending(actionId, pending),
        onDemoModeStatus: (payload) => this.handleDemoModeStatus(payload),
      },
    });
    this.terminalController = new TerminalController({
      ui: this.ui,
      callbacks: {
        onLog: (message, level) => this.addLog(message, level),
        onGraphicalSession: (session) => this.handleGraphicalAppSession(session),
        onGraphicalSessions: (sessions) => this.handleGraphicalAppSessions(sessions),
        onGraphicalSessionClosed: (sessionId) => this.handleGraphicalAppSessionClosed(sessionId),
      },
    });
    this.terminalController.init();
    this.syncControlToggleStatesToUiAndScene();
  }

  init() {
    this.bindUIEvents();
    this.refreshCabinRemoteCurrentPosition();
    this.syncCabinRemoteOperationState();
    this.refreshBottomLinearModulePosition();
    this.syncCabinRemoteStatusSummary();
    this.syncDisplayedImageSubscription({ suppressLog: true });
    this.syncPointCloudSubscription({ suppressLog: true });
    this.syncLogSubscription({ suppressLog: true });
    this.refreshActionState();
    this.restoreTerminalPanelIfVisible();
    this.refreshGraphicalAppSessions();
    this.startGraphicalAppSessionPolling();
    this.refreshDemoModeStatus({ suppressLog: true });
    this.startDemoModeStatusPolling();
    this.rosConnectionController.connect();
    window.addEventListener("message", this.handleGraphicalAppFrameMessage);
    return this;
  }

  refreshBottomLinearModulePosition() {
    const localPosition = this.tcpLinearRemoteController?.getCurrentState?.() || null;
    this.sceneView.setLinearModuleLocalPosition(localPosition);
    const globalPosition = this.sceneView.getLinearModuleGlobalPositionMm(localPosition);
    this.ui.setBottomLinearModulePosition(localPosition, globalPosition);
    return { localPosition, globalPosition };
  }

  syncTcpWorkspaceBoundaryOverlay() {
    if (this.displayedImageTopicName !== TOPICS.camera.irImage || !this.irCameraInfo) {
      this.workspaceView.setTcpWorkspaceBoundary(null);
      return null;
    }
    const boundary = this.sceneView.projectTcpWorkspaceBoundaryToImage(this.irCameraInfo);
    this.workspaceView.setTcpWorkspaceBoundary(boundary);
    return boundary;
  }

  applyVisualDebugBindRangeSettings(settings = this.visualDebugSettings) {
    const range = settings?.linearModuleBindRangeMm;
    this.sceneView?.setLinearModuleBindRange(range);
    const boundary = this.syncTcpWorkspaceBoundaryOverlay();
    if (this.rosConnectionController?.isReady?.()) {
      this.rosConnectionController.publishExecutionRefineTcpRoi(range);
    }
    return boundary;
  }

  handleCabinStateMessage(message) {
    this.cabinTelemetry = normalizeCabinTelemetry(message);
    this.cabinRemoteController?.setLastKnownRawCabinPosition?.({
      x: Number(message?.cabin_state_X),
      y: Number(message?.cabin_state_Y),
      z: Number(message?.cabin_state_Z),
    });
    const currentCabinPosition = this.cabinRemoteController?.getCurrentRawCabinPositionMm?.() || null;
    this.ui.setCabinRemoteCurrentPosition(currentCabinPosition);
    this.ui.setCabinRemoteAbsoluteTarget(currentCabinPosition);
    this.ui.setTaskButtonEnabled("setRecognitionPose", Boolean(currentCabinPosition));
    this.syncCabinRemoteOperationState();
  }

  handleAreaProgressMessage(message) {
    this.latestAreaProgress = message || null;
    this.areaNavigationController?.handleAreaProgressMessage(message);
    this.ui.setAreaProgress(message || null);
  }

  handleAreaNavigationTask(taskAction) {
    const direction = taskAction === "previousArea" ? -1 : 1;
    this.areaNavigationController.moveRelative(direction).finally(() => {
      this.refreshActionState();
    });
  }

  getCabinRemoteOperationState() {
    const ready = this.rosConnectionController?.isReady?.() || false;
    const resources = this.rosConnectionController?.getResources?.() || {};
    const currentCabinPosition = this.cabinRemoteController?.getCurrentRawCabinPositionMm?.() || null;
    return resolveCabinRemoteOperationState({
      rosReady: ready,
      hasMoveServices: Boolean(
        resources?.cabinIncrementalMoveService &&
        resources?.cabinSingleMoveService &&
        resources?.stopCabinMotionService,
      ),
      hasPosition: Boolean(currentCabinPosition),
      moveInFlight: this.cabinRemoteMoveInFlight,
      cabinTelemetry: this.cabinTelemetry,
    });
  }

  async handleResetAllAlarms() {
    const result = await this.rosConnectionController.publishAlarmReset();
    if (!result.success) {
      this.addLog(result.message || "报警复位失败。", "warn");
      return;
    }
    this.statusMonitorController.clearAlarmState();
    this.ui.setConnectionAlarmState([]);
    this.addLog(result.message || "报警复位信号已发送。", "success");
  }

  syncCabinRemoteOperationState() {
    const operationState = this.getCabinRemoteOperationState();
    const ready = this.rosConnectionController?.isReady?.() || false;
    const resources = this.rosConnectionController?.getResources?.() || {};
    const stopEnabled = ready
      && Boolean(resources?.stopCabinMotionService)
      && this.cabinTelemetry?.hasTelemetry
      && this.cabinTelemetry?.cabinConnectFlag === 1;
    this.ui.setBottomCabinOperationState(operationState);
    this.ui.setCabinRemoteButtonsEnabled({
      move: operationState.canOperate,
      stop: stopEnabled,
    });
    return operationState;
  }

  bindUIEvents() {
    this.ui.onToolbarAction((toolbarAction) => {
      this.handleToolbarAction(toolbarAction);
    });
    this.ui.onConnectionAction((actionId) => {
      if (actionId === "manualRosReconnect") {
        this.rosConnectionController.connect({ manual: true });
        return;
      }
      if (actionId === "resetAllAlarms") {
        this.handleResetAllAlarms();
        return;
      }
      this.systemControlController.handle(actionId);
    });
    this.ui.onStatusChipAction((_statusId, actionId) => {
      this.systemControlController.handle(actionId);
    });
    this.ui.onSystemAction((actionId) => {
      this.systemControlController.handle(actionId);
    });
    this.ui.onTaskAction((taskAction) => {
      if (taskAction === "setRecognitionPose") {
        this.handleSetRecognitionPose();
        this.refreshActionState();
        return;
      }
      if (taskAction === "previousArea" || taskAction === "nextArea") {
        this.handleAreaNavigationTask(taskAction);
        this.refreshActionState();
        return;
      }
      if (taskAction === "moveToPosition") {
        this.surfaceDpOverlayActive = false;
        this.surfaceDpOverlayRequested = false;
        this.visualRecognitionOverlayCompleted = false;
        this.visualRecognitionOverlayCleared = false;
        this.workspaceView.setS2OverlayMessage(null);
        this.workspaceView.setVisualRecognitionPointsMessage(null);
        this.workspaceView.setVisualRecognitionOverlaySourceSize(null);
        this.handleMoveToPosition();
        return;
      }
      if (!["runSavedS2", "triggerSingleBind"].includes(taskAction)) {
        this.surfaceDpOverlayActive = false;
        this.surfaceDpOverlayRequested = false;
        this.visualRecognitionOverlayCompleted = false;
        this.visualRecognitionOverlayCleared = false;
        this.workspaceView.setS2OverlayMessage(null);
        this.workspaceView.setVisualRecognitionPointsMessage(null);
        this.workspaceView.setVisualRecognitionOverlaySourceSize(null);
      }
      if (VISUAL_FRAME_SYNC_TASK_ACTIONS.has(taskAction)) {
        this.applyVisualDebugRuntimeSettings(this.visualDebugSettings, { suppressLog: true });
      }
      this.taskActionController.handle(taskAction);
      this.refreshActionState();
    });
    this.ui.onWorkspaceAction((workspaceAction) => {
      if (workspaceAction === "undo") {
        this.workspaceView.undoSelection();
      } else if (workspaceAction === "clear") {
        this.workspaceView.clearSelection();
      }
      this.refreshActionState();
    });
    this.ui.onDisplaySettingsChange((settings) => {
      this.displaySettings = {
        mode: settings.mode,
        gamma: Number.isFinite(settings.gamma) ? settings.gamma : 0.85,
        overlayOpacity: Number.isFinite(settings.overlayOpacity)
          ? settings.overlayOpacity
          : 0.88,
      };
      saveDisplayPreferences(this.displaySettings);
      this.workspaceView.setDisplaySettings(this.displaySettings);
      this.ui.setDisplaySettings(this.displaySettings);
    });
    this.ui.onSceneControlsChange((state) => {
      this.topicLayerController.handleSceneControlsChange(state);
      saveTopicLayerStatePreference(this.topicLayerController.getState());
      this.syncImageHoverCoordinateSubscription();
    });
    this.ui.onSettingsPageChange((pageId) => {
      this.activeSettingsPage = pageId;
      const workspacePickingEnabled = pageId === "workspace";
      this.workspaceView.setWorkspacePickingEnabled(workspacePickingEnabled);
      this.workspaceView.setSavedWorkspaceGuideVisible(workspacePickingEnabled);
      this.syncImageHoverCoordinateSubscription();
      if (pageId === "homeCalibration") {
        this.refreshRobotHomeCalibration({ suppressLog: true });
      }
    });
    this.ui.onSettingsHomePageChange((pageId) => {
      this.settingsHomePage = pageId;
      saveSettingsHomePagePreference(pageId);
    });
    this.ui.onSettingsPageOrderChange((pageOrder) => {
      this.settingsPageOrder = pageOrder;
      saveSettingsPageOrderPreference(pageOrder);
    });
    this.ui.onImageTopicChange(() => {
      this.surfaceDpOverlayActive = false;
      this.surfaceDpOverlayRequested = false;
      this.workspaceView.setS2OverlayMessage(null);
      this.syncDisplayedImageSubscription();
    });
    this.ui.onLogTopicChange(() => {
      this.syncLogSubscription();
    });
    this.ui.onTopicLayerControlsChange((state) => {
      this.topicLayerController.handleLayerControlsChange(state);
      this.applyImageOverlayLayerState();
      saveTopicLayerStatePreference(this.topicLayerController.getState());
      this.syncPointCloudSubscription();
    });
    this.ui.onCalibrationApply((payload) => {
      this.applyGripperTfCalibration(payload);
    });
    this.ui.onRobotHomeCalibrationAction((action, payload) => {
      this.handleRobotHomeCalibrationAction(action, payload);
    });
    this.ui.onCabinRemoteAction((directionId, event) => {
      if (event.type === "stop") {
        this.handleCabinRemoteStopAction("button");
        return;
      }
      this.handleCabinRemoteDirection(directionId, "button");
    });
    this.ui.onCabinRemoteAbsoluteMoveAction((target) => {
      this.handleCabinRemoteAbsoluteMove(target);
    });
    this.ui.onCabinRemoteSettingsChange((settings) => {
      this.cabinRemoteSettings = settings;
      saveCabinRemoteSettings(settings);
      this.syncCabinRemoteStatusSummary();
      this.syncGlobalCabinMoveSpeed({ suppressLog: true });
    });
    this.ui.onNetworkPingSettingsChange((settings) => {
      this.networkPingSettings = settings;
      saveNetworkPingSettings(settings);
    });
    this.ui.onNetworkPingTest((targetId) => {
      this.handleNetworkPingTest(targetId);
    });
    this.ui.onTcpLinearRemoteAction((directionId, event = {}) => {
      if (event.type === "stop") {
        this.handleTcpLinearRemoteStopAction();
        return;
      }
      this.handleTcpLinearRemoteDirection(directionId);
    });
    this.ui.onTcpLinearRemoteSettingsChange((settings) => {
      this.tcpLinearRemoteSettings = settings;
      saveTcpLinearRemoteSettings(settings);
      this.syncGlobalLinearModuleSpeed({ suppressLog: true });
      this.ui.setTcpLinearRemoteStatus(
        `TCP 步进参数：线性=${Number.isFinite(settings.step) ? settings.step : 5}mm，角度=${Number.isFinite(settings.angleStep) ? settings.angleStep : 5}deg，线模执行速度=${Number.isFinite(settings.speed) ? settings.speed : 250}mm/s。`,
      );
    });
    this.ui.onVisualDebugSettingsChange((settings) => {
      this.applyVisualDebugRuntimeSettings(settings, { suppressLog: true });
    });
    this.ui.onLegacyCommand((commandId) => {
      this.legacyCommandController.handle(commandId, this.ui.getParameterValues());
    });
    this.ui.onBottomLinearModuleZeroAction(() => {
      this.legacyCommandController.handle(LINEAR_MODULE_ZERO_LEGACY_COMMAND_ID, this.ui.getParameterValues());
    });
    this.ui.onControlToggle((toggleId, options = {}) => {
      const nextState = options.longPress
        ? this.legacyCommandController.handleToggleLongPress(toggleId, this.ui.getParameterValues())
        : this.legacyCommandController.handleToggle(toggleId, this.ui.getParameterValues());
      if (nextState) {
        this.ui.setControlToggleState(toggleId, nextState);
        if (toggleId === "jumpBindEnabled") {
          this.sceneView.setJumpBindVisualizationState(nextState);
        }
      }
    });
    this.ui.onClearLogs(() => {
      this.logs = [];
      this.frontendLogs = [];
      this.systemLogs = [];
      this.layerSystemLogs = [];
      this.visualDebugLogs = [];
      this.ui.renderLogs(this.logs);
      this.ui.renderSettingsLayerLogs(this.buildSettingsLayerLogViewModel());
      this.ui.renderVisualDebugLogs(this.visualDebugLogs);
    });
    this.ui.onTerminalAction((action, sessionId) => {
      this.handleTerminalAction(action, sessionId);
    });
    this.ui.onGraphicalAppAction((action, sessionId) => {
      this.handleGraphicalAppAction(action, sessionId);
    });
    document.removeEventListener("keydown", this.handleCabinRemoteKeyDown, true);
    document.addEventListener("keydown", this.handleCabinRemoteKeyDown, true);
    window.removeEventListener("beforeunload", this.handleWindowBeforeUnload, true);
    window.addEventListener("beforeunload", this.handleWindowBeforeUnload, true);
  }

  renderControlPanelTasks() {
    this.ui.renderControlPanelTasks();
  }

  syncControlToggleStatesToUiAndScene() {
    const snapshot = this.legacyCommandController?.getToggleStateSnapshot?.() || {};
    this.ui.syncControlToggleStates(snapshot);
    this.sceneView?.setJumpBindVisualizationState(snapshot.jumpBindEnabled);
  }

  syncControlToggleStateFromTelemetry(toggleId, active) {
    const nextState = this.legacyCommandController?.syncToggleState?.(toggleId, active);
    if (!nextState) {
      return;
    }
    this.ui.setControlToggleState(toggleId, nextState);
    if (toggleId === "jumpBindEnabled") {
      this.sceneView?.setJumpBindVisualizationState(nextState);
    }
  }

  createLayoutSnapshot() {
    const panelVisibility = this.ui.getPanelVisibilitySnapshot();
    const panelGeometry = this.panelManager.getPanelLayoutSnapshot();
    const panels = {};
    this.panelRegistry.forEach((panel) => {
      panels[panel.id] = {
        ...(panelVisibility[panel.id] || {}),
        ...(panelGeometry[panel.id] || {}),
      };
    });
    return {
      ...this.activeLayout,
      panels,
    };
  }

  persistActiveLayout() {
    if (!this.activeLayout?.id || !this.panelManager) {
      return;
    }
    const snapshot = this.createLayoutSnapshot();
    this.activeLayout = snapshot;
    this.layoutManager.persistLayout(this.activeLayout.id, snapshot);
  }

  handleWindowBeforeUnload() {
    this.persistActiveLayout();
  }

  restoreTerminalPanelIfVisible() {
    if (!this.ui.isPanelVisible("terminalPanel")) {
      return;
    }
    this.terminalController.handle("ensure").catch((error) => {
      this.addLog(`恢复终端失败：${error?.message || String(error)}`, "error");
    });
  }

  handleToolbarAction(action) {
    if (!action) {
      return;
    }

    if (action === "toggle-theme") {
      this.applyTheme(this.theme === "dark" ? "light" : "dark");
      return;
    }

    if (action.startsWith("toggle-panel:")) {
      const panelId = action.split(":")[1];
      const nextVisible = this.ui.togglePanelVisible(panelId);
      if (nextVisible) {
        if (!this.panelManager.hasPanelLayout(panelId)) {
          this.panelManager.applyDefaultPanelRect(panelId);
        }
      }
      if (panelId === "terminalPanel" && nextVisible) {
        this.terminalController.handle("ensure").catch((error) => {
          const message = `创建终端失败：${error?.message || String(error)}`;
          this.ui.setTerminalNotice(message, "error");
          this.addLog(message, "error");
        });
        window.setTimeout(() => {
          this.terminalController.fitActiveSession();
        }, 80);
      }
      this.persistActiveLayout();
      return;
    }

    return;
  }

  handleSetRecognitionPose() {
    const pose = this.cabinRemoteController.getCurrentRawCabinPositionMm();
    if (!pose || !["x", "y", "z"].every((axis) => Number.isFinite(Number(pose[axis])))) {
      const message = "暂未拿到机器当前位置，无法设置识别位姿。";
      this.ui.setControlFeedback(message);
      this.addLog(message, "warn");
      return;
    }
    this.recognitionPose = {
      x: Number(pose.x),
      y: Number(pose.y),
      z: Number(pose.z),
    };
    saveRecognitionPose(pose);
    const message =
      `识别位姿已保存: x=${Math.round(this.recognitionPose.x)}, y=${Math.round(this.recognitionPose.y)}, z=${Math.round(this.recognitionPose.z)}`;
    this.ui.setControlFeedback(message);
    this.addLog(message, "success");
  }

  async handleMoveToPosition() {
    const payload = { ...this.recognitionPose, speed: this.getGlobalCabinMoveSpeed() };
    this.addLog(
      `准备直接移动到位姿: x=${payload.x}, y=${payload.y}, z=${payload.z}, speed=${payload.speed}`,
      "info",
    );
    const result = await this.rosConnectionController.callCabinSingleMoveService(payload);
    if (result.success) {
      this.cabinRemoteController.setLastKnownRawCabinPosition(payload);
      this.cabinRemoteController.setLastKnownCabinPosition(payload);
      this.refreshCabinRemoteCurrentPosition();
      this.ui.setControlFeedback(result.message || "索驱已移动到识别位姿。");
      this.addLog(result.message || "索驱已移动到识别位姿。", "success");
      return;
    }
    this.ui.setControlFeedback(result.message || "索驱移动到识别位姿失败。");
    this.addLog(result.message || "索驱移动到识别位姿失败。", "error");
  }

  getGlobalCabinMoveSpeed() {
    const { speed } = this.ui.getCabinRemoteSettings();
    return Number.isFinite(speed) && speed > 0 ? speed : 300;
  }

  getGlobalLinearModuleSpeed() {
    const { speed } = this.ui.getTcpLinearRemoteSettings();
    return Number.isFinite(speed) && speed > 0 ? speed : 250;
  }

  syncGlobalCabinMoveSpeed({ suppressLog = false } = {}) {
    const speed = this.getGlobalCabinMoveSpeed();
    const result = this.rosConnectionController.publishCabinSpeed(speed);
    if (!result?.success && !suppressLog) {
      this.addLog(result.message || "同步全局索驱速度失败。", "warn");
    }
    return result;
  }

  syncGlobalLinearModuleSpeed({ suppressLog = false } = {}) {
    const speed = this.getGlobalLinearModuleSpeed();
    const result = this.rosConnectionController.publishLinearModuleSpeed(speed);
    if (!result?.success && !suppressLog) {
      this.addLog(result.message || "同步全局线性模组速度失败。", "warn");
    }
    return result;
  }

  addVisualDebugLog(message, level = "info") {
    const entry = {
      timestamp: new Date().toLocaleTimeString("zh-CN", { hour12: false }),
      message,
      level,
    };
    this.visualDebugLogs = [entry, ...this.visualDebugLogs].slice(0, 24);
    this.ui.renderVisualDebugLogs(this.visualDebugLogs);
  }

  applyVisualDebugRuntimeSettings(settings = this.ui.getVisualDebugSettings(), { suppressLog = false } = {}) {
    const nextSettings = settings || this.ui.getVisualDebugSettings();
    this.visualDebugSettings = nextSettings;
    saveVisualDebugSettings(nextSettings);
    const boundary = this.applyVisualDebugBindRangeSettings(nextSettings);
    const frameResult = this.rosConnectionController.publishStableFrameCount(nextSettings.stableFrameCount);
    this.ui.setVisualDebugTimingSummary({
      releaseFrameCount: nextSettings.stableFrameCount,
      bindExecutionCabinMinZMm: nextSettings.bindExecutionCabinMinZMm,
    });
    if (frameResult?.success) {
      const message = frameResult.message || `视觉服务最终放行帧数已设置为 ${nextSettings.stableFrameCount} 帧。`;
      if (!suppressLog) {
        this.addLog(message, "success");
        this.addVisualDebugLog(message, "success");
      }
    } else if (!suppressLog) {
      const message = frameResult?.message || "视觉服务放行帧数设置失败。";
      this.addLog(message, "warn");
      this.addVisualDebugLog(message, "warn");
    }
    const beamResult = this.applyVisualDebugBeamExclusionSettings(nextSettings, { suppressLog });
    return { boundary, frameResult, beamResult };
  }

  applyVisualDebugStableFrameCount({ suppressLog = false } = {}) {
    const settings = this.ui.getVisualDebugSettings();
    this.visualDebugSettings = settings;
    saveVisualDebugSettings(settings);
    this.applyVisualDebugBindRangeSettings(settings);
    const result = this.rosConnectionController.publishStableFrameCount(settings.stableFrameCount);
    this.ui.setVisualDebugTimingSummary({
      releaseFrameCount: settings.stableFrameCount,
      bindExecutionCabinMinZMm: settings.bindExecutionCabinMinZMm,
    });
    if (result?.success) {
      const message = result.message || `视觉服务最终放行帧数已设置为 ${settings.stableFrameCount} 帧。`;
      if (!suppressLog) {
        this.addLog(message, "success");
        this.addVisualDebugLog(message, "success");
      }
      return result;
    }
    if (!suppressLog) {
      const message = result?.message || "视觉服务放行帧数设置失败。";
      this.addLog(message, "warn");
      this.addVisualDebugLog(message, "warn");
    }
    return result;
  }

  applyVisualDebugBeamExclusionSettings(
    settings = this.visualDebugSettings,
    { suppressLog = false } = {},
  ) {
    const nextSettings = settings || this.ui.getVisualDebugSettings();
    this.visualDebugSettings = nextSettings;
    const result = this.rosConnectionController.publishScanBeamExclusion(
      Boolean(nextSettings?.enableBeamExclusion),
    );
    if (result?.success) {
      if (!suppressLog) {
        this.addLog(result.message || "扫描梁筋过滤设置已同步。", "success");
        this.addVisualDebugLog(result.message || "扫描梁筋过滤设置已同步。", "success");
      }
      return result;
    }
    if (!suppressLog) {
      const message = result?.message || "扫描梁筋过滤设置同步失败。";
      this.addLog(message, "warn");
      this.addVisualDebugLog(message, "warn");
    }
    return result;
  }

  async handleNetworkPingTest(targetId) {
    const settings = this.ui.getNetworkPingSettings();
    this.networkPingSettings = settings;
    saveNetworkPingSettings(settings);

    const targetLabels = {
      cabin: "索驱",
      moduan: "线性模组",
    };
    const targetHosts = {
      cabin: settings.cabinHost,
      moduan: settings.moduanHost,
    };
    const label = targetLabels[targetId] || "设备";
    const host = String(targetHosts[targetId] || "").trim();
    if (!host) {
      const message = `${label} IP 不能为空。`;
      this.ui.setNetworkPingResult(targetId, { state: "error", message });
      this.addLog(message, "warn");
      return;
    }

    this.ui.setNetworkPingPending(targetId, true);
    this.ui.setNetworkPingResult(targetId, {
      state: "pending",
      target: host,
      summary: "正在测试连接…",
    });

    try {
      const response = await fetch("/api/network/ping", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ targetId, label, host }),
      });
      const result = await response.json().catch(() => ({}));
      const reachable = Boolean(response.ok && result.success);
      const summary = result.summary || result.message || (reachable ? "连接成功。" : "连接失败。");
      const message = `${label} ${result.target || host} ${reachable ? "可达" : "不可达"}：${summary}`;
      this.ui.setNetworkPingResult(targetId, {
        ...result,
        state: reachable ? "success" : "error",
        target: result.target || host,
        summary,
      });
      this.addLog(message, reachable ? "success" : "warn");
    } catch (error) {
      const message = `${label} ${host} 网络测试请求失败：${error?.message || String(error)}`;
      this.ui.setNetworkPingResult(targetId, {
        state: "error",
        target: host,
        summary: message,
      });
      this.addLog(message, "error");
    } finally {
      this.ui.setNetworkPingPending(targetId, false);
    }
  }

  handleCabinRemoteKeyDown(event) {
    if (event.repeat || event.ctrlKey || event.metaKey || event.altKey) {
      return;
    }

    const { keyboardEnabled } = this.ui.getCabinRemoteSettings();
    if (!keyboardEnabled) {
      return;
    }

    if (this.shouldIgnoreCabinRemoteKeyboard(event.target)) {
      return;
    }

    const action = resolveCabinRemoteKeyboardActionFromKey(event.key);
    if (!action) {
      return;
    }

    consumeCabinRemoteKeyboardEvent(event);
    if (action.type === "stop") {
      this.handleCabinRemoteStopAction("keyboard");
      return;
    }
    this.handleCabinRemoteDirection(action.directionId, "keyboard");
  }

  resolveCabinRemoteDirectionFromKey(key) {
    return resolveCabinRemoteDirectionFromKey(key);
  }

  shouldIgnoreCabinRemoteKeyboard(target) {
    return shouldIgnoreCabinRemoteKeyboardTarget(target, document.activeElement);
  }

  refreshCabinRemoteCurrentPosition() {
    const currentCabinPosition =
      this.cabinRemoteController.getCurrentRawCabinPositionMm()
      || this.cabinRemoteController.getCurrentCabinPositionMm();
    this.ui.setCabinRemoteCurrentPosition(currentCabinPosition);
    return currentCabinPosition;
  }

  async handleCabinRemoteStopAction(source) {
    this.ui.refs.cabinRemoteStopButton?.classList.add("is-active");
    window.setTimeout(() => {
      this.ui.refs.cabinRemoteStopButton?.classList.remove("is-active");
    }, 420);

    const result = await this.rosConnectionController.callCabinMotionStopService();
    const sourceLabel = source === "keyboard" ? "键盘" : "按钮";
    const message = result?.success
      ? `${sourceLabel}已发送索驱停止指令。`
      : result?.message || `${sourceLabel}停止索驱失败。`;
    this.ui.setCabinRemoteStatus(message);
    this.ui.setControlFeedback(message);
    this.addLog(message, result?.success ? "success" : "warn");
    this.syncCabinRemoteOperationState();
    return result;
  }

  async handleCabinRemoteDirection(directionId, source) {
    const operationState = this.getCabinRemoteOperationState();
    if (!operationState.canOperate) {
      const blockedMessage = operationState.detail || "索驱当前不可操作，已忽略本次遥控。";
      this.ui.setCabinRemoteStatus(blockedMessage);
      this.ui.setControlFeedback(blockedMessage);
      this.addLog(blockedMessage, "warn");
      this.syncCabinRemoteOperationState();
      return { success: false, skipped: true, message: blockedMessage };
    }

    const settings = this.ui.getCabinRemoteSettings();
    this.cabinRemoteMoveInFlight = true;
    this.syncCabinRemoteOperationState();
    try {
      const result = await this.cabinRemoteController.move(directionId, settings);
      this.refreshCabinRemoteCurrentPosition();
      const message = this.buildCabinRemoteFeedbackMessage(result, source);
      this.ui.setCabinRemoteStatus(message);
      this.ui.setControlFeedback(message);
      this.addLog(message, result.success ? "success" : "warn");
      return result;
    } finally {
      this.cabinRemoteMoveInFlight = false;
      this.syncCabinRemoteOperationState();
    }
  }

  buildCabinRemoteFeedbackMessage(result, source) {
    const sourceLabel = source === "keyboard" ? "键盘" : "按钮";
    if (!result?.success) {
      return result?.message || `${sourceLabel}遥控执行失败。`;
    }
    const delta = result.delta || result.target || { x: 0, y: 0, z: 0 };
    if (result.mode === "relative") {
      return `${sourceLabel}遥控 ${result.label}：模式=相对点动 步距=${result.step}mm 速度=${result.speed} 增量=(${Math.round(delta.x)}, ${Math.round(delta.y)}, ${Math.round(delta.z)})`;
    }
    const target = result.target || { x: 0, y: 0, z: 0 };
    return `${sourceLabel}遥控 ${result.label}：模式=绝对点动 步距=${result.step}mm 速度=${result.speed} 目标=(${Math.round(target.x)}, ${Math.round(target.y)}, ${Math.round(target.z)}) 增量=(${Math.round(delta.x)}, ${Math.round(delta.y)}, ${Math.round(delta.z)})`;
  }

  async handleCabinRemoteAbsoluteMove(target) {
    const operationState = this.getCabinRemoteOperationState();
    if (!operationState.canOperate) {
      const blockedMessage = operationState.detail || "索驱当前不可操作，已忽略绝对位姿移动。";
      this.ui.setCabinRemoteStatus(blockedMessage);
      this.ui.setControlFeedback(blockedMessage);
      this.addLog(blockedMessage, "warn");
      this.syncCabinRemoteOperationState();
      return { success: false, skipped: true, message: blockedMessage };
    }

    const payload = {
      x: Number(target?.x),
      y: Number(target?.y),
      z: Number(target?.z),
      speed: Number.isFinite(Number(target?.speed)) && Number(target.speed) > 0
        ? Number(target.speed)
        : this.getGlobalCabinMoveSpeed(),
    };
    if (!["x", "y", "z"].every((axis) => Number.isFinite(payload[axis]))) {
      const message = "绝对目标位姿不完整，索驱未移动。";
      this.ui.setCabinRemoteStatus(message);
      this.ui.setControlFeedback(message);
      this.addLog(message, "warn");
      return { success: false, message };
    }

    this.cabinRemoteMoveInFlight = true;
    this.syncCabinRemoteOperationState();
    try {
      const result = await this.rosConnectionController.callCabinSingleMoveService(payload);
      if (result?.success) {
        this.cabinRemoteController.setLastKnownRawCabinPosition(payload);
        this.ui.setCabinRemoteCurrentPosition(payload);
        this.ui.setCabinRemoteAbsoluteTarget(payload);
      }
      const message = result?.success
        ? `绝对位姿移动已下发：X=${Math.round(payload.x)} Y=${Math.round(payload.y)} Z=${Math.round(payload.z)} speed=${Math.round(payload.speed)}`
        : formatCabinRemoteProtocolFeedback(result?.message || "") || "绝对位姿移动失败。";
      this.ui.setCabinRemoteStatus(message);
      this.ui.setControlFeedback(message);
      this.addLog(message, result?.success ? "success" : "warn");
      return result;
    } finally {
      this.cabinRemoteMoveInFlight = false;
      this.syncCabinRemoteOperationState();
    }
  }

  async handleTcpLinearRemoteDirection(directionId) {
    if (this.tcpLinearRemoteMoveInFlight) {
      const busyMessage = "TCP 线性模组上一条移动指令仍在执行，已忽略本次遥控。";
      this.ui.setTcpLinearRemoteStatus(busyMessage);
      this.addLog(busyMessage, "warn");
      return { success: false, skipped: true, message: busyMessage };
    }

    this.tcpLinearRemoteMoveInFlight = true;
    this.ui.flashTcpLinearRemoteButton(directionId);
    try {
      const result = await this.tcpLinearRemoteController.move(directionId, this.ui.getTcpLinearRemoteSettings());
      const message = this.buildTcpLinearRemoteFeedbackMessage(result);
      this.ui.setTcpLinearRemoteStatus(message);
      this.addLog(message, result.success ? "success" : "warn");
      return result;
    } finally {
      this.tcpLinearRemoteMoveInFlight = false;
    }
  }

  async handleTcpLinearRemoteStopAction() {
    this.ui.flashTcpLinearRemoteStopButton();
    const result = await this.rosConnectionController.publishLinearModuleInterruptStop();
    const message = result?.message || "TCP 线性模组运动暂停信号已发送。";
    this.ui.setTcpLinearRemoteStatus(message);
    this.addLog(message, result?.success ? "success" : "warn");
    return result;
  }

  buildTcpLinearRemoteFeedbackMessage(result) {
    if (!result?.success) {
      return result?.message || "TCP 线性模组遥控执行失败。";
    }

    const clampSuffix = result.clamped ? " | 已夹到线模行程边界" : "";
    return `TCP线模遥控 ${result.label}：步距=${result.increment} 目标=(${Math.round(Number(result.target.x))}, ${Math.round(Number(result.target.y))}, ${Math.round(Number(result.target.z))}, 角度=${result.target.angle.toFixed(1)})${clampSuffix}`;
  }

  async refreshRobotHomeCalibration({ suppressLog = false } = {}) {
    const result = await this.rosConnectionController.callRobotHomeCalibrationService({ command: "get" });
    if (!result?.success) {
      if (!suppressLog) {
        this.addLog(result?.message || "读取 Home 点位标定失败。", "warn");
      }
      return result;
    }
    this.robotHomeCalibration = result;
    this.ui.setRobotHomeCalibration(result, { forceInputs: false });
    if (!suppressLog) {
      this.addLog(result.message || "已读取 Home 点位标定。", "info");
    }
    return result;
  }

  async handleRobotHomeCalibrationAction(action, payload) {
    if (action === "refresh") {
      await this.refreshRobotHomeCalibration();
      return;
    }

    if (action === "capture") {
      const result = await this.rosConnectionController.callRobotHomeCalibrationService({
        command: "capture_current",
      });
      if (!result?.success) {
        this.addLog(result?.message || "当前位置设为 Home 失败。", "warn");
        return;
      }
      this.robotHomeCalibration = result;
      this.ui.setRobotHomeCalibration(result, { forceInputs: true });
      this.addLog(result.message || "当前位置已写入 Home 点位。", "success");
      return;
    }

    if (action === "save") {
      const result = await this.rosConnectionController.callRobotHomeCalibrationService({
        command: "set_home",
        home: payload.home,
      });
      if (!result?.success) {
        this.addLog(result?.message || "保存 Home 点位失败。", "warn");
        return;
      }
      this.robotHomeCalibration = result;
      this.ui.setRobotHomeCalibration(result, { forceInputs: true });
      this.addLog(result.message || "Home 点位已保存。", "success");
      return;
    }

    if (action === "moveHome") {
      const home = this.robotHomeCalibration?.home || payload.home;
      const hasHome = ["x", "y", "z"].every((axis) => Number.isFinite(Number(home?.[axis])));
      if (!hasHome) {
        this.addLog("没有可用 Home 点位，无法回 Home。", "warn");
        return;
      }
      const { speed } = this.ui.getCabinRemoteSettings();
      const result = await this.rosConnectionController.callCabinSingleMoveService({
        x: Number(home.x),
        y: Number(home.y),
        z: Number(home.z),
        speed,
      });
      if (!result?.success) {
        this.addLog(result?.message || "一键回 Home 失败。", "warn");
        return;
      }
      this.cabinRemoteController.setLastKnownRawCabinPosition(home);
      this.refreshCabinRemoteCurrentPosition();
      this.addLog(
        `一键回 Home 已下发：X=${Math.round(Number(home.x))}mm Y=${Math.round(Number(home.y))}mm Z=${Math.round(Number(home.z))}mm`,
        "success",
      );
    }
  }

  syncCabinRemoteStatusSummary() {
    const settings = this.ui.getCabinRemoteSettings();
    const keyboardStatus = settings.keyboardEnabled ? "已开启" : "未开启";
    const moveModeLabel = settings.moveMode === "relative" ? "相对点动" : "绝对点动";
    this.ui.setCabinRemoteStatus(
      `键盘遥控${keyboardStatus}：Q/W/E = Z+/X+/Z-，A/S/D = Y+/X-/Y-，空格 = 暂停 | 模式=${moveModeLabel} | 步距=${Number.isFinite(settings.step) ? settings.step : 50}mm | 全局索驱速度=${Number.isFinite(settings.speed) ? settings.speed : 300}`,
    );
  }

  handleWorkspaceS2Triggered() {
    this.surfaceDpOverlayActive = true;
    this.surfaceDpOverlayRequested = true;
    this.visualRecognitionOverlayCompleted = false;
    this.visualRecognitionOverlayCleared = false;
    this.workspaceView.setS2OverlayMessage(null);
    this.workspaceView.setVisualRecognitionPointsMessage(null);
    this.workspaceView.setVisualRecognitionOverlaySourceSize(null);
    this.syncDisplayedImageSubscription({ suppressLog: true });
    this.addLog("视觉识别会叠加在红外原图图层。", "info");
    this.scheduleS2ResultTimeout();
  }

  showVisualRecognitionOverlayMessage(message) {
    this.surfaceDpOverlayActive = true;
    this.surfaceDpOverlayRequested = false;
    this.visualRecognitionOverlayCompleted = true;
    this.visualRecognitionOverlayCleared = false;
    this.clearS2ResultTimeout();
    this.workspaceView.setS2OverlayMessage(message);
    this.workspaceView.setVisualRecognitionOverlaySourceSize(message);
    if (this.latestVisualRecognitionPointsMessage) {
      this.workspaceView.setVisualRecognitionPointsMessage(this.latestVisualRecognitionPointsMessage, { sourceSize: message });
    }
    this.ui.setControlFeedback("后端视觉识别结果已叠加到红外原图。");
    this.addLog("后端视觉识别结果已叠加到红外原图。", "success");
  }

  cacheLatestVisualRecognitionPointsMessage(message) {
    const points = Array.isArray(message?.PointCoordinatesArray)
      ? message.PointCoordinatesArray
      : [];
    if (points.length > 0) {
      this.latestVisualRecognitionPointsMessage = message;
    }
    return points.length;
  }

  handleVisualRecognitionPointsMessage(message) {
    const pointsLength = this.cacheLatestVisualRecognitionPointsMessage(message);
    const requestActive = this.surfaceDpOverlayActive || this.surfaceDpOverlayRequested;
    if (pointsLength > 0 && (requestActive || this.displayedImageTopicName === TOPICS.camera.irImage)) {
      this.workspaceView.setVisualRecognitionPointsMessage(message);
    }
    if (!requestActive) {
      return pointsLength;
    }
    return pointsLength;
  }

  scheduleS2ResultTimeout() {
    this.clearS2ResultTimeout();
    this.s2ResultTimeoutId = window.setTimeout(() => {
      this.s2ResultTimeoutId = null;
      this.surfaceDpOverlayActive = false;
      this.surfaceDpOverlayRequested = false;
      this.visualRecognitionOverlayCompleted = false;
      this.visualRecognitionOverlayCleared = true;
      this.workspaceView.setS2OverlayMessage(null);
      this.workspaceView.setVisualRecognitionPointsMessage(null);
      this.ui.setControlFeedback("视觉识别已触发，但等待后端结果图超时。请检查视觉日志、世界坐标和当前工作区。");
      this.addLog("视觉识别已触发，但等待后端结果图超时。", "warn");
    }, S2_RESULT_TIMEOUT_MS);
  }

  clearS2ResultTimeout() {
    if (!this.s2ResultTimeoutId) {
      return;
    }
    window.clearTimeout(this.s2ResultTimeoutId);
    this.s2ResultTimeoutId = null;
  }

  schedulePlanningAreaRefresh() {
    this.clearPlanningAreaRefresh();
    this.planningAreaRefreshTimerId = window.setTimeout(() => {
      this.planningAreaRefreshTimerId = null;
      this.refreshPlanningAreaOverlay();
    }, PLANNING_AREA_REFRESH_DELAY_MS);
  }

  clearPlanningAreaRefresh() {
    if (!this.planningAreaRefreshTimerId) {
      return;
    }
    window.clearTimeout(this.planningAreaRefreshTimerId);
    this.planningAreaRefreshTimerId = null;
  }

  clearImageHoverCoordinateSubscriptionIdle() {
    if (!this.imageHoverCoordinateIdleTimerId) {
      return;
    }
    window.clearTimeout(this.imageHoverCoordinateIdleTimerId);
    this.imageHoverCoordinateIdleTimerId = null;
  }

  scheduleImageHoverCoordinateSubscriptionIdle() {
    this.clearImageHoverCoordinateSubscriptionIdle();
    this.imageHoverCoordinateIdleTimerId = window.setTimeout(() => {
      this.imageHoverCoordinateIdleTimerId = null;
      this.imageHoverWorldCoordMessage = null;
      this.rosConnectionController.updateImageHoverCoordinateSubscription({ enabled: false });
    }, IMAGE_HOVER_COORDINATE_IDLE_UNSUBSCRIBE_MS);
  }

  async refreshPlanningAreaOverlay() {
    const requestToken = this.planningAreaRequestToken + 1;
    this.planningAreaRequestToken = requestToken;
    try {
      const response = await fetch("/api/planning/bind-path", { cache: "no-store" });
      if (requestToken !== this.planningAreaRequestToken) {
        return;
      }
      if (!response.ok) {
        this.sceneView.setPlanningAreaPayload(null);
        return;
      }
      const payload = await response.json();
      if (requestToken !== this.planningAreaRequestToken) {
        return;
      }
      this.sceneView.setPlanningAreaPayload(payload.bind_path || null);
    } catch (_error) {
      if (requestToken !== this.planningAreaRequestToken) {
        return;
      }
      this.sceneView.setPlanningAreaPayload(null);
    }
  }

  refreshActionState() {
    const ready = this.rosConnectionController.isReady();
    const resources = this.rosConnectionController.getResources();
    const selectedPoints = this.workspaceView.getSelectedPoints();
    const savedPoints = this.workspaceView.getSavedWorkspacePoints();
    const currentCabinPosition = this.cabinRemoteController.getCurrentCabinPositionMm();
    this.ui.setTaskButtonsEnabled({
      submitQuad:
        ready &&
        Boolean(resources?.workspaceQuadPublisher && resources?.processImageService) &&
        selectedPoints.length === 4,
      runSavedS2:
        ready &&
        Boolean(resources?.startPseudoSlamScanActionClient),
      triggerSingleBind:
        ready &&
        Boolean(resources?.singlePointBindService) &&
        savedPoints.length === 4,
      startExecution:
        ready &&
        Boolean(resources?.executionModeService) &&
        Boolean(resources?.startGlobalWorkActionClient),
      startExecutionKeepMemory:
        ready &&
        Boolean(resources?.executionModeService) &&
        Boolean(resources?.startGlobalWorkActionClient),
      setRecognitionPose: Boolean(currentCabinPosition),
      moveToPosition: ready && Boolean(resources?.cabinSingleMoveService),
      previousArea:
        ready &&
        Boolean(resources?.cabinSingleMoveService) &&
        Boolean(resources?.manualAreaTakeoverPublisher) &&
        Boolean(resources?.moduanMoveZeroPublisher),
      nextArea:
        ready &&
        Boolean(resources?.cabinSingleMoveService) &&
        Boolean(resources?.manualAreaTakeoverPublisher) &&
        Boolean(resources?.moduanMoveZeroPublisher),
    });
    this.ui.setWorkspaceButtonsEnabled({
      undo: selectedPoints.length > 0,
      clear: selectedPoints.length > 0,
    });
    this.syncCabinRemoteOperationState();
    this.ui.setTcpLinearRemoteButtonsEnabled(
      ready &&
        Boolean(resources?.linearModuleSingleMoveService) &&
        Boolean(resources?.linearModuleInterruptStopPublisher),
    );
  }

  updateDerivedStatusIndicators() {
    // 状态胶囊当前只保留硬件状态，不再派生任务/地图状态。
  }

  clearInactivePointClouds({ clearSelectedSource = false } = {}) {
    const layerState = this.topicLayerController.getState();
    const sources = ["filteredWorldCoord", "rawWorldCoord"];
    const inactiveSources = sources.filter((source) => clearSelectedSource || source !== layerState.pointCloudSource);

    if (!layerState.showPointCloud || clearSelectedSource) {
      inactiveSources.push(layerState.pointCloudSource);
    }

    [...new Set(inactiveSources)].forEach((source) => {
      this.sceneView.clearPointCloudSource(source);
      this.topicLayerController.updateStats({ [`${source}Count`]: 0 });
      this.viewerStore.updateIn("scene", { [`${source}Count`]: 0 });
    });
  }

  applyImageOverlayLayerState(state = null) {
    const layerState = state || this.topicLayerController?.getState?.() || this.topicLayerState;
    this.workspaceView.setImageOverlayLayerState(layerState);
  }

  syncDisplayedImageSubscription({ suppressLog = false } = {}) {
    const selectedTopic = this.ui.getSelectedImageTopic();
    this.displayedImageTopicName = selectedTopic || DEFAULT_IMAGE_TOPIC;
    this.workspaceView.setOverlayEnabled(isOverlayCompatibleImageTopic(selectedTopic));
    this.syncTcpWorkspaceBoundaryOverlay();
    this.syncImageHoverCoordinateSubscription({ suppressLog: true });
    const subscriptionResult = this.rosConnectionController.updateDisplayedImageSubscription(selectedTopic);
    if (!suppressLog && subscriptionResult?.changed) {
      this.addLog(`图像卡片已切换到 ${getImageTopicLabel(subscriptionResult.topic || DEFAULT_IMAGE_TOPIC)}。`, "info");
    }
  }

  syncLogSubscription({ suppressLog = false } = {}) {
    const selectedTopicId = this.ui.getSelectedLogTopic();
    this.selectedLogTopicId = selectedTopicId;
    const subscriptionResult = this.rosConnectionController.updateLogSubscription(selectedTopicId);
    this.renderLogView();
    if (!suppressLog && subscriptionResult?.changed) {
      this.addLog(`日志卡片已切换到 ${getLogTopicLabel(subscriptionResult.topicId || DEFAULT_LOG_TOPIC)}。`, "info");
    }
  }

  syncPointCloudSubscription({ suppressLog = false } = {}) {
    const layerState = this.topicLayerController.getState();
    const subscriptionResult = this.rosConnectionController.updatePointCloudSubscription({
      enabled: Boolean(layerState.showPointCloud),
      source: layerState.pointCloudSource,
    });
    this.clearInactivePointClouds({ clearSelectedSource: !layerState.showPointCloud });
    if (!layerState.showPointCloud) {
      if (!suppressLog && subscriptionResult?.changed) {
        this.addLog("点云默认按需订阅；当前图层关闭，未订阅世界点云。", "info");
      }
      return;
    }

    if (!suppressLog && subscriptionResult?.changed) {
      const sourceLabel = layerState.pointCloudSource === "rawWorldCoord" ? "原始世界点云" : "滤波世界点云";
      this.addLog(`点云图层已开启：当前仅订阅 ${sourceLabel}。`, "info");
    }
  }

  syncImageHoverCoordinateSubscription({ suppressLog = false } = {}) {
    this.clearImageHoverCoordinateSubscriptionIdle();
    this.imageHoverWorldCoordMessage = null;
    const subscriptionResult = this.rosConnectionController.updateImageHoverCoordinateSubscription({ enabled: false });
    this.workspaceView.setHoverCoordinateReadout(null);
    if (!suppressLog && subscriptionResult?.changed) {
      this.addLog(
        this.activeSettingsPage === "workspace"
          ? "工作区选点模式已开启，图像悬停坐标暂停。"
          : "图像悬停坐标改为按需订阅，当前已释放原始世界点图。",
        "info",
      );
    }
    return subscriptionResult;
  }

  getImageHoverCoordinateTargetFrame() {
    const selectedFrame = this.topicLayerController.getState().imageHoverCoordinateFrame || "map";
    return ["map", "gripper_frame", "Scepter_depth_frame"].includes(selectedFrame)
      ? selectedFrame
      : "map";
  }

  formatHoverCoordinateValue(value) {
    const rounded = Number(value);
    return Number.isFinite(rounded) ? rounded.toFixed(1) : "--";
  }

  handleImageHoverPixelChanged(pixel) {
    if (!pixel || this.activeSettingsPage === "workspace") {
      this.clearImageHoverCoordinateSubscriptionIdle();
      this.imageHoverWorldCoordMessage = null;
      this.rosConnectionController.updateImageHoverCoordinateSubscription({ enabled: false });
      this.workspaceView.setHoverCoordinateReadout(null);
      return;
    }

    this.rosConnectionController.updateImageHoverCoordinateSubscription({ enabled: true });
    this.scheduleImageHoverCoordinateSubscriptionIdle();

    const targetFrame = this.getImageHoverCoordinateTargetFrame();
    const frameLabel = getImageHoverCoordinateFrameLabel(targetFrame);
    const sampledCameraPoint = sampleFloat32XYZImagePixel(this.imageHoverWorldCoordMessage, pixel);
    if (!sampledCameraPoint) {
      this.workspaceView.setHoverCoordinateReadout({
        pixel,
        sourceSize: this.workspaceView.getCurrentImageSize(),
        lines: [
          `px=(${pixel.x},${pixel.y})`,
          "等待raw_world_coord",
        ],
      });
      return;
    }

    const coordinate = this.sceneView.convertScepterPointMmToFrameMm(sampledCameraPoint, targetFrame);
    if (!coordinate) {
      this.workspaceView.setHoverCoordinateReadout({
        pixel,
        sourceSize: this.workspaceView.getCurrentImageSize(),
        lines: [
          `px=(${pixel.x},${pixel.y})`,
          `${frameLabel}: 等待TF`,
        ],
      });
      return;
    }

    this.workspaceView.setHoverCoordinateReadout({
      pixel,
      sourceSize: this.workspaceView.getCurrentImageSize(),
      lines: [
        `px=(${pixel.x},${pixel.y}) ${frameLabel}`,
        `x=${this.formatHoverCoordinateValue(coordinate.x)}mm`,
        `y=${this.formatHoverCoordinateValue(coordinate.y)}mm z=${this.formatHoverCoordinateValue(coordinate.z)}mm`,
      ],
    });
  }

  async applyGripperTfCalibration(payload) {
    const result = await this.rosConnectionController.callGripperTfCalibrationService(payload);
    if (!result?.success) {
      this.addLog(result?.message || "相机-TCP外参热更新失败。", "warn");
      return;
    }

    const appliedCalibration = {
      parentFrame: "Scepter_depth_frame",
      childFrame: "gripper_frame",
      translationMm: {
        x: Number.isFinite(result?.applied?.x) ? result.applied.x : 0,
        y: Number.isFinite(result?.applied?.y) ? result.applied.y : 0,
        z: Number.isFinite(result?.applied?.z) ? result.applied.z : 0,
      },
    };
    const sceneCalibration = this.sceneView.applyCameraToTcpCalibration(appliedCalibration) || appliedCalibration;
    this.ui.setGripperTfCalibration(sceneCalibration, { forceInputs: true });
    this.syncTcpWorkspaceBoundaryOverlay();
    this.addLog(
      `相机-TCP外参已热更新：x=${Math.round(Number(result.applied.x))}mm y=${Math.round(Number(result.applied.y))}mm z=${Math.round(Number(result.applied.z))}mm`,
      "success",
    );
    this.addLog(result.message || "gripper_tf_broadcaster 已写回本地 gripper_tf.yaml。", "info");
    if (result.fallback === "topic") {
      this.addLog("service 未返回时已自动改发 /web/tf/set_camera_tcp_extrinsic，前端已先按本次外参刷新显示。", "warn");
    }

    const savedPoints = this.workspaceView.getSavedWorkspacePoints();
    if (savedPoints.length === 4) {
      this.addLog("外参已热更新；视觉识别不会自动重跑，请手动点击“触发视觉识别”。", "info");
      return;
    }

    this.addLog("外参已热更新，但当前没有已保存工作区；确认工作区域后会自动触发视觉识别。", "info");
  }

  applyTheme(theme) {
    this.theme = theme === "light" ? "light" : "dark";
    document.documentElement.setAttribute("data-theme", this.theme);
    saveThemePreference(this.theme);
    this.ui.setTheme(this.theme);
    this.sceneView.setTheme(this.theme);
    this.terminalController.setTheme(this.theme);
  }

  async handleTerminalAction(action, sessionId = null) {
    try {
      await this.terminalController.handle(action, sessionId);
    } catch (error) {
      this.ui.setTerminalNotice(`终端操作失败：${error?.message || String(error)}`, "error");
      this.addLog(`终端操作失败：${error?.message || String(error)}`, "error");
    }
  }

  handleGraphicalAppSession(session) {
    if (!session?.sessionId) {
      return;
    }
    const nextSessions = [...this.graphicalAppSessions];
    const existingIndex = nextSessions.findIndex((item) => item.sessionId === session.sessionId);
    if (existingIndex >= 0) {
      nextSessions.splice(existingIndex, 1, session);
    } else {
      nextSessions.push(session);
    }
    this.handleGraphicalAppSessions(nextSessions);
    if (session.state === "error") {
      this.addLog(session.message || `图形界面启动失败：${session.label || session.sessionId}`, "error");
    }
  }

  getGraphicalAppSessionSignature(sessions) {
    const items = Array.isArray(sessions) ? sessions : [];
    return JSON.stringify(items.map((session) => ({
      sessionId: String(session?.sessionId || ""),
      label: String(session?.label || ""),
      state: String(session?.state || ""),
      webPort: String(session?.webPort || ""),
      webPath: String(session?.webPath || ""),
      url: String(session?.url || ""),
      command: Array.isArray(session?.command) ? session.command.map((part) => String(part)) : [],
    })));
  }

  handleGraphicalAppSessions(sessions) {
    const nextSessions = Array.isArray(sessions) ? sessions : [];
    const nextSignature = this.getGraphicalAppSessionSignature(nextSessions);
    if (nextSignature === this.graphicalAppSessionSignature) {
      return;
    }
    this.graphicalAppSessionSignature = nextSignature;
    this.graphicalAppSessions = nextSessions;
    this.ui.renderGraphicalAppSessions(this.graphicalAppSessions);
  }

  startGraphicalAppSessionPolling() {
    if (this.graphicalAppSessionPollTimer) {
      return;
    }
    this.graphicalAppSessionPollTimer = window.setInterval(() => {
      this.refreshGraphicalAppSessions({ suppressLog: true });
    }, 2500);
  }

  handleDemoModeStatus(payload) {
    const active = Boolean(payload?.active ?? payload?.demoModeActive);
    const detail = payload?.message || (active ? "演示模式运行中" : "演示模式未启用");
    this.ui.setDemoModeState(active, detail);
  }

  async refreshDemoModeStatus({ suppressLog = false } = {}) {
    try {
      await this.systemControlController.refreshDemoModeStatus();
    } catch (error) {
      this.ui.setDemoModeState(false, `演示模式状态读取失败：${error?.message || String(error)}`);
      if (!suppressLog) {
        this.addLog(`演示模式状态读取失败：${error?.message || String(error)}`, "warn");
      }
    }
  }

  startDemoModeStatusPolling() {
    if (this.demoModeStatusPollTimer) {
      return;
    }
    this.demoModeStatusPollTimer = window.setInterval(() => {
      this.refreshDemoModeStatus({ suppressLog: true });
    }, 5000);
  }

  handleGraphicalAppFrameMessage(event) {
    if (event.origin !== window.location.origin) {
      return;
    }
    if (event.data?.type !== "tie-robot-gui-session-closed") {
      return;
    }
    const sessionId = String(event.data?.sessionId || "");
    if (sessionId) {
      this.handleGraphicalAppSessionClosed(sessionId);
    }
    if (event.data?.message) {
      this.addLog(event.data.message, "warn");
    }
    this.refreshGraphicalAppSessions({ suppressLog: true });
  }

  async refreshGraphicalAppSessions(options = {}) {
    const suppressLog = Boolean(options?.suppressLog);
    try {
      const response = await fetch("/api/gui/sessions");
      const payload = await response.json();
      if (payload?.success) {
        this.handleGraphicalAppSessions(payload.sessions || []);
      }
    } catch (error) {
      if (!suppressLog) {
        this.addLog(`图形界面状态刷新失败：${error?.message || String(error)}`, "warn");
      }
    }
  }

  handleGraphicalAppSessionClosed(sessionId) {
    this.handleGraphicalAppSessions(
      this.graphicalAppSessions.filter((session) => session.sessionId !== sessionId),
    );
  }

  async handleGraphicalAppAction(action, sessionId) {
    if (action !== "close" || !sessionId) {
      return;
    }
    try {
      await fetch(`/api/gui/sessions/${sessionId}`, { method: "DELETE" });
    } catch (error) {
      this.addLog(`关闭图形界面失败：${error?.message || String(error)}`, "error");
    }
    this.handleGraphicalAppSessionClosed(sessionId);
  }

  addLog(message, level = "info") {
    const timestamp = new Date().toLocaleTimeString("zh-CN", { hour12: false });
    const entry = {
      timestamp,
      message: `[前端] ${message}`,
      level,
      nodeName: "frontend",
      source: "frontend",
      sortKey: Date.now(),
    };
    this.frontendLogs = [entry, ...this.frontendLogs].slice(0, 80);
    this.renderLogView();
  }

  handleSystemLog(message) {
    const entry = this.normalizeRosLogEntry(message);
    if (!entry) {
      return;
    }
    this.systemLogs = [entry, ...this.systemLogs].slice(0, 200);
    this.renderLogView();
  }

  handleLayerSystemLog(message) {
    const entry = this.normalizeRosLogEntry(message);
    if (!entry) {
      return;
    }
    const category = this.getLayerLogCategory(entry.nodeName);
    if (!category) {
      return;
    }
    this.layerSystemLogs = [{
      ...entry,
      category,
    }, ...this.layerSystemLogs].slice(0, SETTINGS_LAYER_LOG_TOTAL_LIMIT);
    this.ui.renderSettingsLayerLogs(this.buildSettingsLayerLogViewModel());
  }

  normalizeRosLogEntry(message) {
    const rawText = sanitizeRosLogText(message?.msg);
    if (!rawText.trim()) {
      return null;
    }
    const nodeName = String(message?.name || "").split("/").filter(Boolean).pop() || "unknown";
    const content = rawText;
    const level = inferLegacyLogLevel(message?.msg) || this.mapRosLogLevel(message?.level);
    const stamp = message?.header?.stamp;
    const sortKey = stamp?.secs
      ? (Number(stamp.secs) * 1000) + Math.floor(Number(stamp.nsecs || 0) / 1e6)
      : Date.now();
    const timestamp = new Date(sortKey).toLocaleTimeString("zh-CN", { hour12: false });
    return {
      timestamp,
      message: `[${nodeName}] ${content}`,
      content,
      level,
      nodeName,
      source: "system",
      sortKey,
    };
  }

  getLayerLogCategory(nodeName) {
    if (DRIVER_LAYER_LOG_NODES.has(nodeName)) {
      return "driver";
    }
    if (ALGORITHM_LAYER_LOG_NODES.has(nodeName)) {
      return "algorithm";
    }
    return null;
  }

  buildSettingsLayerLogViewModel() {
    return SETTINGS_LAYER_LOG_GROUPS.map((group) => ({
      id: group.id,
      label: group.label,
      nodes: Array.from(group.nodes.entries()).map(([nodeName, label]) => {
        const history = this.layerSystemLogs
          .filter((entry) => entry.category === group.id && entry.nodeName === nodeName)
          .slice(0, SETTINGS_LAYER_LOG_HISTORY_LIMIT);
        return {
          nodeName,
          label,
          latest: history[0] || null,
          history,
        };
      }),
    }));
  }

  renderLogView() {
    const filteredSystemLogs = this.systemLogs.filter((entry) => matchesLogTopicFilter(entry, this.selectedLogTopicId));
    const visibleLogs = this.selectedLogTopicId === "all"
      ? [...this.frontendLogs, ...filteredSystemLogs]
      : filteredSystemLogs;
    this.logs = visibleLogs
      .sort((left, right) => Number(right.sortKey || 0) - Number(left.sortKey || 0))
      .slice(0, 200);
    this.ui.renderLogs(this.logs);
  }

  mapRosLogLevel(level) {
    if (level >= 16) {
      return "error";
    }
    if (level >= 8) {
      return "warn";
    }
    if (level >= 2) {
      return "info";
    }
    return "success";
  }
}
