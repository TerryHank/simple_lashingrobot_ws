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
import { CabinRemoteController } from "../controllers/CabinRemoteController.js";
import { TcpLinearRemoteController } from "../controllers/TcpLinearRemoteController.js";
import { TerminalController } from "../controllers/TerminalController.js";
import { TopicLayerController } from "../controllers/TopicLayerController.js";
import { PanelManager } from "../ui/PanelManager.js";
import { UIController } from "../ui/UIController.js";
import {
  loadCabinRemoteSettings,
  loadDisplayPreferences,
  loadRecognitionPose,
  loadSettingsHomePagePreference,
  loadSettingsPageOrderPreference,
  loadThemePreference,
  loadVisualDebugSettings,
  loadViewerLayout,
  saveCabinRemoteSettings,
  saveDisplayPreferences,
  saveRecognitionPose,
  saveSettingsHomePagePreference,
  saveSettingsPageOrderPreference,
  saveThemePreference,
  saveVisualDebugSettings,
  saveViewerLayout,
} from "../utils/storage.js";
import {
  consumeCabinRemoteKeyboardEvent,
  resolveCabinRemoteKeyboardActionFromKey,
  resolveCabinRemoteDirectionFromKey,
  shouldIgnoreCabinRemoteKeyboardTarget,
} from "../utils/cabinRemoteKeyboard.js";
import { inferLegacyLogLevel, sanitizeRosLogText } from "../utils/logText.js";
import { Scene3DView } from "../views/Scene3DView.js";
import { WorkspaceCanvasView } from "../views/WorkspaceCanvasView.js";

const DIRECT_CABIN_MOVE_TARGET = Object.freeze({
  x: -260,
  y: 1700,
  z: 3197,
});
const CABIN_REMOTE_REPEAT_INTERVAL_MS = 250;
const PLANNING_AREA_REFRESH_DELAY_MS = 180;
const S2_RESULT_TIMEOUT_MS = 6000;
const SETTINGS_LAYER_LOG_HISTORY_LIMIT = 50;
const SETTINGS_LAYER_LOG_TOTAL_LIMIT = 500;

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

const VISUAL_DEBUG_REQUEST_MODE_LABELS = {
  0: "默认模式",
  1: "自适应高度",
  2: "绑扎检查",
  3: "扫描输出",
  4: "执行微调",
};

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
    this.planningAreaRequestToken = 0;
    this.prFprgOverlayActive = false;
    this.prFprgOverlayRequested = false;
    this.visualRecognitionOverlayCompleted = false;
    this.visualRecognitionOverlayCleared = false;
    this.latestVisualRecognitionPointsMessage = null;
    this.cabinRemoteRepeatTimerId = null;
    this.cabinRemoteRepeatDirectionId = null;
    this.cabinRemoteMoveInFlight = false;
    this.tcpLinearRemoteMoveInFlight = false;
    this.graphicalAppSessions = [];
    this.graphicalAppSessionSignature = "";
    this.graphicalAppSessionPollTimer = null;
    this.handleCabinRemoteKeyDown = this.handleCabinRemoteKeyDown.bind(this);
    this.handleCabinRemoteGlobalPointerUp = this.handleCabinRemoteGlobalPointerUp.bind(this);
    this.handleCabinRemoteWindowBlur = this.handleCabinRemoteWindowBlur.bind(this);
    this.handleCabinRemoteVisibilityChange = this.handleCabinRemoteVisibilityChange.bind(this);
    this.handleWindowBeforeUnload = this.handleWindowBeforeUnload.bind(this);
    this.handleGraphicalAppFrameMessage = this.handleGraphicalAppFrameMessage.bind(this);
    this.displaySettings = loadDisplayPreferences();
    this.cabinRemoteSettings = loadCabinRemoteSettings();
    this.recognitionPose = loadRecognitionPose(DIRECT_CABIN_MOVE_TARGET);
    this.visualDebugSettings = loadVisualDebugSettings();
    this.settingsHomePage = loadSettingsHomePagePreference();
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
    this.ui.setSettingsPage(this.settingsHomePage);
    this.ui.renderPanelsFromLayout(this.activeLayout);
    this.ui.setTheme(this.theme);
    this.ui.setCabinRemoteSettings(this.cabinRemoteSettings);

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
    });
    this.workspaceView.bindPointerEvents();
    this.workspaceView.setDisplaySettings(this.displaySettings);
    this.workspaceView.setSavedWorkspaceGuideVisible(false);
    this.ui.setDisplaySettings(this.displaySettings);
    this.ui.setVisualDebugSettings(this.visualDebugSettings);
    this.ui.renderPointList([]);
    this.ui.renderSettingsLayerLogs(this.buildSettingsLayerLogViewModel());
    this.ui.renderVisualDebugLogs(this.visualDebugLogs);

    this.sceneView = new Scene3DView({
      container: this.ui.getSceneContainer(),
    });
    this.sceneView.setTheme(this.theme);

    this.topicLayerController = new TopicLayerController({
      ui: this.ui,
      sceneView: this.sceneView,
      callbacks: {
        onLog: (message, level) => this.addLog(message, level),
      },
    });
    this.topicLayerController.init();

    this.statusMonitorController = new StatusMonitorController({
      onStatusChip: (statusId, level, detail) => this.ui.setStatusChipState(statusId, level, detail),
      onBatteryVoltage: (voltage) => this.ui.setBatteryVoltage(voltage),
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
        this.syncLogSubscription({ suppressLog: true });
        this.syncGlobalCabinMoveSpeed({ suppressLog: true });
        this.applyVisualDebugStableFrameCount({ suppressLog: true });
        this.schedulePlanningAreaRefresh();
        this.refreshActionState();
      },
      onRosUnavailable: () => {
        this.statusMonitorController.stop();
        this.clearS2ResultTimeout();
        this.clearPlanningAreaRefresh();
        this.planningAreaRequestToken += 1;
        this.sceneView.setPlanningAreaPayload(null);
        this.prFprgOverlayActive = false;
        this.prFprgOverlayRequested = false;
        this.visualRecognitionOverlayCleared = false;
        this.stopCabinRemoteRepeat();
        this.legacyCommandController?.reset();
        this.ui.syncControlToggleStates(this.legacyCommandController?.getToggleStateSnapshot());
        this.ui.renderTopicInventory([]);
        this.ui.setCabinRemoteCurrentPosition(null);
        this.ui.setTcpLinearRemoteState(null);
        this.ui.setBottomLinearModulePosition(null, null);
        this.ui.setTcpLinearRemoteButtonsEnabled(false);
        this.ui.setGripperTfCalibration(null);
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
      onDisplayedImage: (message) => {
        this.workspaceView.setBaseImageMessage(message);
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
        if (this.prFprgOverlayRequested) {
          return;
        }
        if (this.visualRecognitionOverlayCleared) {
          return;
        }
        if (this.prFprgOverlayActive) {
          return;
        }
        this.clearS2ResultTimeout();
        this.workspaceView.setExecutionOverlayMessage(message);
        this.ui.setControlFeedback("执行结果覆盖层已更新。");
      },
      onWorkspaceS2Overlay: (message) => {
        if (!this.prFprgOverlayRequested) {
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
        if (this.prFprgOverlayActive || this.prFprgOverlayRequested) {
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
        const currentCabinPosition = this.sceneView.getCurrentCabinPositionMm();
        if (currentCabinPosition) {
          this.cabinRemoteController?.setLastKnownCabinPosition(currentCabinPosition);
        }
        this.ui.setCabinRemoteCurrentPosition(currentCabinPosition);
        this.ui.setTaskButtonEnabled("setRecognitionPose", Boolean(currentCabinPosition));
        const tfFrameCount = this.sceneView.getKnownTransformCount();
        this.topicLayerController.updateStats({ tfFrameCount });
        this.viewerStore.updateIn("scene", { tfFrameCount });
        this.ui.setGripperTfCalibration(this.sceneView.getCameraToTcpCalibration());
        this.refreshBottomLinearModulePosition();
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

    this.tcpLinearRemoteController = new TcpLinearRemoteController({
      rosConnection: this.rosConnectionController,
    });

    this.taskActionController = new TaskActionController({
      rosConnection: this.rosConnectionController,
      workspaceView: this.workspaceView,
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
    this.ui.syncControlToggleStates(this.legacyCommandController.getToggleStateSnapshot());
  }

  init() {
    this.bindUIEvents();
    this.refreshCabinRemoteCurrentPosition();
    this.refreshBottomLinearModulePosition();
    this.syncCabinRemoteStatusSummary();
    this.syncDisplayedImageSubscription({ suppressLog: true });
    this.syncPointCloudSubscription({ suppressLog: true });
    this.syncLogSubscription({ suppressLog: true });
    this.refreshActionState();
    this.restoreTerminalPanelIfVisible();
    this.refreshGraphicalAppSessions();
    this.startGraphicalAppSessionPolling();
    this.rosConnectionController.connect();
    window.addEventListener("message", this.handleGraphicalAppFrameMessage);
    return this;
  }

  refreshBottomLinearModulePosition() {
    const localPosition = this.tcpLinearRemoteController?.getCurrentState?.() || null;
    const globalPosition = this.sceneView.getLinearModuleGlobalPositionMm(localPosition);
    this.ui.setBottomLinearModulePosition(localPosition, globalPosition);
    return { localPosition, globalPosition };
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
      if (taskAction === "clearVisualRecognition") {
        this.handleClearVisualRecognitionOverlay();
        this.refreshActionState();
        return;
      }
      if (taskAction === "moveToPosition") {
        this.prFprgOverlayActive = false;
        this.prFprgOverlayRequested = false;
        this.visualRecognitionOverlayCompleted = false;
        this.visualRecognitionOverlayCleared = false;
        this.workspaceView.setS2OverlayMessage(null);
        this.workspaceView.setVisualRecognitionPointsMessage(null);
        this.workspaceView.setVisualRecognitionOverlaySourceSize(null);
        this.handleMoveToPosition();
        return;
      }
      if (!["runSavedS2", "triggerSingleBind"].includes(taskAction)) {
        this.prFprgOverlayActive = false;
        this.prFprgOverlayRequested = false;
        this.visualRecognitionOverlayCompleted = false;
        this.visualRecognitionOverlayCleared = false;
        this.workspaceView.setS2OverlayMessage(null);
        this.workspaceView.setVisualRecognitionPointsMessage(null);
        this.workspaceView.setVisualRecognitionOverlaySourceSize(null);
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
    });
    this.ui.onSettingsPageChange((pageId) => {
      this.workspaceView.setSavedWorkspaceGuideVisible(pageId === "workspace");
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
      this.prFprgOverlayActive = false;
      this.prFprgOverlayRequested = false;
      this.workspaceView.setS2OverlayMessage(null);
      this.syncDisplayedImageSubscription();
    });
    this.ui.onLogTopicChange(() => {
      this.syncLogSubscription();
    });
    this.ui.onTopicLayerControlsChange((state) => {
      this.topicLayerController.handleLayerControlsChange(state);
      this.syncPointCloudSubscription();
    });
    this.ui.onCalibrationApply((payload) => {
      this.applyGripperTfCalibration(payload);
    });
    this.ui.onCabinRemoteAction((directionId, event) => {
      if (event.type === "stop") {
        this.handleCabinRemoteStopAction("button");
        return;
      }
      if (event.type === "pressstart") {
        this.startCabinRemoteRepeat(directionId);
        return;
      }
      if (event.type === "pressend") {
        this.stopCabinRemoteRepeat();
        return;
      }
      this.handleCabinRemoteDirection(directionId, "button");
    });
    this.ui.onCabinRemoteSettingsChange((settings) => {
      this.cabinRemoteSettings = settings;
      saveCabinRemoteSettings(settings);
      this.syncCabinRemoteStatusSummary();
      this.syncGlobalCabinMoveSpeed({ suppressLog: true });
    });
    this.ui.onTcpLinearRemoteAction((directionId, event = {}) => {
      if (event.type === "stop") {
        this.handleTcpLinearRemoteStopAction();
        return;
      }
      this.handleTcpLinearRemoteDirection(directionId);
    });
    this.ui.onTcpLinearRemoteSettingsChange(() => {
      const settings = this.ui.getTcpLinearRemoteSettings();
      this.ui.setTcpLinearRemoteStatus(
        `TCP 步进参数：线性=${Number.isFinite(settings.step) ? settings.step : 5}mm，角度=${Number.isFinite(settings.angleStep) ? settings.angleStep : 5}deg。`,
      );
    });
    this.ui.onVisualDebugSettingsChange((settings) => {
      this.visualDebugSettings = settings;
      saveVisualDebugSettings(settings);
      this.ui.setVisualDebugTimingSummary({
        releaseFrameCount: settings.stableFrameCount,
      });
    });
    this.ui.onVisualDebugApplyStableFrameCount((settings) => {
      this.visualDebugSettings = settings;
      saveVisualDebugSettings(settings);
      this.applyVisualDebugStableFrameCount();
    });
    this.ui.onVisualDebugTrigger((settings) => {
      this.handleVisualDebugTrigger(settings);
    });
    this.ui.onLegacyCommand((commandId) => {
      this.legacyCommandController.handle(commandId, this.ui.getParameterValues());
    });
    this.ui.onControlToggle((toggleId) => {
      const nextState = this.legacyCommandController.handleToggle(toggleId, this.ui.getParameterValues());
      if (nextState) {
        this.ui.setControlToggleState(toggleId, nextState);
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
    window.removeEventListener("pointerup", this.handleCabinRemoteGlobalPointerUp, true);
    window.addEventListener("pointerup", this.handleCabinRemoteGlobalPointerUp, true);
    window.removeEventListener("blur", this.handleCabinRemoteWindowBlur, true);
    window.addEventListener("blur", this.handleCabinRemoteWindowBlur, true);
    document.removeEventListener("visibilitychange", this.handleCabinRemoteVisibilityChange, true);
    document.addEventListener("visibilitychange", this.handleCabinRemoteVisibilityChange, true);
    window.removeEventListener("beforeunload", this.handleWindowBeforeUnload, true);
    window.addEventListener("beforeunload", this.handleWindowBeforeUnload, true);
  }

  renderControlPanelTasks() {
    this.ui.renderControlPanelTasks();
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
    const pose = this.sceneView.getCurrentCabinPositionMm();
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

  syncGlobalCabinMoveSpeed({ suppressLog = false } = {}) {
    const speed = this.getGlobalCabinMoveSpeed();
    const result = this.rosConnectionController.publishCabinSpeed(speed);
    if (!result?.success && !suppressLog) {
      this.addLog(result.message || "同步全局索驱速度失败。", "warn");
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

  applyVisualDebugStableFrameCount({ suppressLog = false } = {}) {
    const settings = this.ui.getVisualDebugSettings();
    this.visualDebugSettings = settings;
    saveVisualDebugSettings(settings);
    const result = this.rosConnectionController.publishStableFrameCount(settings.stableFrameCount);
    this.ui.setVisualDebugTimingSummary({
      releaseFrameCount: settings.stableFrameCount,
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

  async handleVisualDebugTrigger(settings = this.ui.getVisualDebugSettings()) {
    this.visualDebugSettings = settings;
    saveVisualDebugSettings(settings);
    if (!this.rosConnectionController.isReady()) {
      const message = "ROS 未连接，无法触发视觉调试服务。";
      this.addLog(message, "error");
      this.addVisualDebugLog(message, "error");
      this.ui.setControlFeedback(message);
      return;
    }
    this.applyVisualDebugStableFrameCount({ suppressLog: true });
    this.handleWorkspaceS2Triggered();

    const modeLabel = VISUAL_DEBUG_REQUEST_MODE_LABELS[settings.requestMode] || `mode=${settings.requestMode}`;
    const startMessage = `视觉调试服务请求开始：模式=${modeLabel}，释放=${settings.stableFrameCount}帧。`;
    this.addLog(startMessage, "info");
    this.addVisualDebugLog(startMessage, "info");

    const result = await this.rosConnectionController.callProcessImageService({
      requestMode: settings.requestMode,
    });
    const singleFrameText = Number.isFinite(Number(result.singleFrameElapsedMs))
      ? `${Number(result.singleFrameElapsedMs).toFixed(1)}ms`
      : "--ms";
    const serviceText = Number.isFinite(Number(result.serviceElapsedMs))
      ? `${Number(result.serviceElapsedMs).toFixed(1)}ms`
      : "--ms";
    const level = result.success ? "success" : "error";
    const message =
      `视觉调试服务返回：单帧=${singleFrameText}，服务请求=${serviceText}，点数=${result.count || 0}，结果=${result.message || "无消息"}`;

    this.ui.setVisualDebugTimingSummary({
      singleFrameElapsedMs: result.singleFrameElapsedMs,
      serviceElapsedMs: result.serviceElapsedMs,
      releaseFrameCount: settings.stableFrameCount,
      pointCount: result.count,
    });
    this.addLog(message, level);
    this.addVisualDebugLog(message, level);
    this.ui.setControlFeedback(result.message || (result.success ? "视觉服务请求完成。" : "视觉服务请求失败。"));
    this.refreshActionState();
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
    const currentCabinPosition = this.cabinRemoteController.getCurrentCabinPositionMm();
    this.ui.setCabinRemoteCurrentPosition(currentCabinPosition);
    return currentCabinPosition;
  }

  handleCabinRemoteGlobalPointerUp() {
    this.stopCabinRemoteRepeat();
  }

  handleCabinRemoteWindowBlur() {
    this.stopCabinRemoteRepeat();
  }

  handleCabinRemoteVisibilityChange() {
    if (document.hidden) {
      this.stopCabinRemoteRepeat();
    }
  }

  async handleCabinRemoteStopAction(source) {
    this.stopCabinRemoteRepeat();
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
    return result;
  }

  async startCabinRemoteRepeat(directionId) {
    if (!directionId) {
      return;
    }
    if (this.cabinRemoteRepeatDirectionId === directionId && (this.cabinRemoteRepeatTimerId || this.cabinRemoteMoveInFlight)) {
      return;
    }
    this.stopCabinRemoteRepeat();
    this.cabinRemoteRepeatDirectionId = directionId;
    this.ui.setCabinRemoteButtonActive(directionId);
    const result = await this.handleCabinRemoteDirection(directionId, "button", { repeating: true });
    if (result?.success && this.cabinRemoteRepeatDirectionId === directionId) {
      this.scheduleCabinRemoteRepeatTick();
      return;
    }
    this.stopCabinRemoteRepeat();
  }

  stopCabinRemoteRepeat() {
    if (this.cabinRemoteRepeatTimerId) {
      window.clearTimeout(this.cabinRemoteRepeatTimerId);
      this.cabinRemoteRepeatTimerId = null;
    }
    this.cabinRemoteRepeatDirectionId = null;
    this.ui.clearCabinRemoteButtonActive();
  }

  scheduleCabinRemoteRepeatTick() {
    if (!this.cabinRemoteRepeatDirectionId) {
      return;
    }
    if (this.cabinRemoteRepeatTimerId) {
      window.clearTimeout(this.cabinRemoteRepeatTimerId);
    }
    this.cabinRemoteRepeatTimerId = window.setTimeout(async () => {
      this.cabinRemoteRepeatTimerId = null;
      const directionId = this.cabinRemoteRepeatDirectionId;
      if (!directionId) {
        return;
      }
      if (this.cabinRemoteMoveInFlight) {
        this.scheduleCabinRemoteRepeatTick();
        return;
      }
      const result = await this.handleCabinRemoteDirection(directionId, "button", { repeating: true });
      if (result?.success && this.cabinRemoteRepeatDirectionId === directionId) {
        this.scheduleCabinRemoteRepeatTick();
        return;
      }
      this.stopCabinRemoteRepeat();
    }, CABIN_REMOTE_REPEAT_INTERVAL_MS);
  }

  async handleCabinRemoteDirection(directionId, source, { repeating = false } = {}) {
    if (this.cabinRemoteMoveInFlight) {
      if (repeating) {
        return { success: false, skipped: true, message: "索驱上一条移动指令仍在执行，已跳过本次连发节拍。" };
      }
      const busyMessage = "索驱上一条移动指令仍在执行，已忽略本次遥控。";
      this.ui.setCabinRemoteStatus(busyMessage);
      this.ui.setControlFeedback(busyMessage);
      this.addLog(busyMessage, "warn");
      return { success: false, skipped: true, message: busyMessage };
    }

    const settings = this.ui.getCabinRemoteSettings();
    this.cabinRemoteMoveInFlight = true;
    try {
      const result = await this.cabinRemoteController.move(directionId, settings);
      this.refreshCabinRemoteCurrentPosition();
      const message = this.buildCabinRemoteFeedbackMessage(result, source, { repeating });
      this.ui.setCabinRemoteStatus(message);
      this.ui.setControlFeedback(message);
      this.addLog(message, result.success ? "success" : "warn");
      if (!result.success && repeating) {
        this.stopCabinRemoteRepeat();
      }
      return result;
    } finally {
      this.cabinRemoteMoveInFlight = false;
    }
  }

  buildCabinRemoteFeedbackMessage(result, source, { repeating = false } = {}) {
    const sourceLabel = source === "keyboard" ? "键盘" : "按钮";
    if (!result?.success) {
      return result?.message || `${sourceLabel}遥控执行失败。`;
    }
    const repeatSuffix = repeating ? " | 连发中" : "";
    return `${sourceLabel}遥控 ${result.label}：步距=${result.step}mm 速度=${result.speed} 目标=(${Math.round(result.target.x)}, ${Math.round(result.target.y)}, ${Math.round(result.target.z)})${repeatSuffix}`;
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
    return `TCP线模遥控 ${result.label}：步距=${result.increment} 目标=(${result.target.x.toFixed(1)}, ${result.target.y.toFixed(1)}, ${result.target.z.toFixed(1)}, 角度=${result.target.angle.toFixed(1)})${clampSuffix}`;
  }

  syncCabinRemoteStatusSummary() {
    const settings = this.ui.getCabinRemoteSettings();
    const keyboardStatus = settings.keyboardEnabled ? "已开启" : "未开启";
    this.ui.setCabinRemoteStatus(
      `键盘遥控${keyboardStatus}：Q/W/E = Z+/X+/Z-，A/S/D = Y+/X-/Y-，空格 = 暂停 | 步距=${Number.isFinite(settings.step) ? settings.step : 50}mm | 全局索驱速度=${Number.isFinite(settings.speed) ? settings.speed : 300}`,
    );
  }

  handleWorkspaceS2Triggered() {
    this.prFprgOverlayActive = true;
    this.prFprgOverlayRequested = true;
    this.visualRecognitionOverlayCompleted = false;
    this.visualRecognitionOverlayCleared = false;
    this.workspaceView.setS2OverlayMessage(null);
    this.workspaceView.setVisualRecognitionPointsMessage(null);
    this.workspaceView.setVisualRecognitionOverlaySourceSize(null);
    this.syncDisplayedImageSubscription({ suppressLog: true });
    this.addLog("视觉识别会叠加在当前图像图层，不切换图像话题。", "info");
    this.scheduleS2ResultTimeout();
  }

  showVisualRecognitionOverlayMessage(message) {
    this.prFprgOverlayActive = true;
    this.prFprgOverlayRequested = false;
    this.visualRecognitionOverlayCompleted = true;
    this.visualRecognitionOverlayCleared = false;
    this.clearS2ResultTimeout();
    this.workspaceView.setS2OverlayMessage(message);
    this.workspaceView.setVisualRecognitionPointsMessage(null);
    this.workspaceView.setVisualRecognitionOverlaySourceSize(message);
    this.ui.setControlFeedback("后端视觉识别结果图已叠加在当前图像图层。");
    this.addLog("后端视觉识别结果图已叠加在当前图像图层。", "success");
  }

  handleClearVisualRecognitionOverlay() {
    this.clearS2ResultTimeout();
    this.prFprgOverlayActive = false;
    this.prFprgOverlayRequested = false;
    this.visualRecognitionOverlayCompleted = false;
    this.visualRecognitionOverlayCleared = true;
    this.workspaceView.setS2OverlayMessage(null);
    this.workspaceView.setVisualRecognitionPointsMessage(null);
    this.workspaceView.setVisualRecognitionOverlaySourceSize(null);
    this.ui.setControlFeedback("识别结果覆盖层已清除，图像卡片显示原图。");
    this.addLog("识别结果覆盖层已清除，图像卡片显示原图。", "info");
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
    const requestActive = this.prFprgOverlayActive || this.prFprgOverlayRequested;
    if (!requestActive) {
      return pointsLength;
    }
    return pointsLength;
  }

  scheduleS2ResultTimeout() {
    this.clearS2ResultTimeout();
    this.s2ResultTimeoutId = window.setTimeout(() => {
      this.s2ResultTimeoutId = null;
      this.prFprgOverlayActive = false;
      this.prFprgOverlayRequested = false;
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
        Boolean(resources?.workspaceQuadPublisher && resources?.lashingRecognizeOnceService) &&
        selectedPoints.length === 4,
      runSavedS2:
        ready &&
        Boolean(resources?.lashingRecognizeOnceService) &&
        savedPoints.length === 4,
      triggerSingleBind:
        ready &&
        Boolean(resources?.singlePointBindService) &&
        savedPoints.length === 4,
      scanPlan: ready && Boolean(resources?.startPseudoSlamScanActionClient),
      startExecution:
        ready &&
        Boolean(resources?.executionModeService) &&
        Boolean(resources?.startGlobalWorkActionClient),
      startExecutionKeepMemory:
        ready &&
        Boolean(resources?.executionModeService) &&
        Boolean(resources?.startGlobalWorkActionClient),
      runBindPathTest: ready && Boolean(resources?.runDirectBindPathTestActionClient),
      setRecognitionPose: Boolean(currentCabinPosition),
      moveToPosition: ready && Boolean(resources?.cabinSingleMoveService),
      clearVisualRecognition: true,
    });
    this.ui.setWorkspaceButtonsEnabled({
      undo: selectedPoints.length > 0,
      clear: selectedPoints.length > 0,
    });
    const cabinRemoteEnabled = ready
      && Boolean(resources?.cabinSingleMoveService)
      && Boolean(resources?.stopCabinMotionService);
    this.ui.setCabinRemoteButtonsEnabled(cabinRemoteEnabled);
    if (!cabinRemoteEnabled) {
      this.stopCabinRemoteRepeat();
    }
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

  syncDisplayedImageSubscription({ suppressLog = false } = {}) {
    const selectedTopic = this.ui.getSelectedImageTopic();
    this.workspaceView.setOverlayEnabled(isOverlayCompatibleImageTopic(selectedTopic));
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

  async applyGripperTfCalibration(payload) {
    const result = await this.rosConnectionController.callGripperTfCalibrationService(payload);
    if (!result?.success) {
      this.addLog(result?.message || "相机-TCP外参热更新失败。", "warn");
      return;
    }

    this.ui.setGripperTfCalibration({
      parentFrame: "Scepter_depth_frame",
      childFrame: "gripper_frame",
      translationMm: {
        x: Number.isFinite(result?.applied?.x) ? result.applied.x : 0,
        y: Number.isFinite(result?.applied?.y) ? result.applied.y : 0,
        z: Number.isFinite(result?.applied?.z) ? result.applied.z : 0,
      },
    }, { forceInputs: true });
    this.addLog(
      `相机-TCP外参已热更新：x=${Number(result.applied.x).toFixed(1)}mm y=${Number(result.applied.y).toFixed(1)}mm z=${Number(result.applied.z).toFixed(1)}mm`,
      "success",
    );
    this.addLog(result.message || "gripper_tf_broadcaster 已写回本地 gripper_tf.yaml。", "info");

    const savedPoints = this.workspaceView.getSavedWorkspacePoints();
    if (savedPoints.length === 4) {
      this.addLog("外参已热更新；视觉识别不会自动重跑，请手动点击“触发视觉识别”。", "info");
      return;
    }

    this.addLog("外参已热更新，但当前没有已保存工作区；提交四边形后再手动触发视觉识别。", "info");
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
