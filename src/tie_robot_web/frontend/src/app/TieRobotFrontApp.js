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
import { TerminalController } from "../controllers/TerminalController.js";
import { TopicLayerController } from "../controllers/TopicLayerController.js";
import { normalizeControlPanelVisibleTaskIds, normalizeCustomControlPanelButtons } from "../config/controlPanelCatalog.js";
import { PanelManager } from "../ui/PanelManager.js";
import { UIController } from "../ui/UIController.js";
import {
  loadControlPanelVisibleTasks,
  loadCustomControlPanelButtons,
  loadDisplayPreferences,
  loadSettingsHomePagePreference,
  loadThemePreference,
  loadViewerLayout,
  saveControlPanelVisibleTasks,
  saveCustomControlPanelButtons,
  saveDisplayPreferences,
  saveSettingsHomePagePreference,
  saveThemePreference,
  saveViewerLayout,
} from "../utils/storage.js";
import { Scene3DView } from "../views/Scene3DView.js";
import { WorkspaceCanvasView } from "../views/WorkspaceCanvasView.js";

const DIRECT_CABIN_MOVE_TARGET = Object.freeze({
  x: -260,
  y: 1700,
  z: 3197,
});
const CABIN_REMOTE_REPEAT_INTERVAL_MS = 250;
const S2_RESULT_TIMEOUT_MS = 6000;

export class TieRobotFrontApp {
  constructor(rootElement) {
    this.rootElement = rootElement;
    this.logs = [];
    this.frontendLogs = [];
    this.systemLogs = [];
    this.selectedLogTopicId = DEFAULT_LOG_TOPIC;
    this.s2ResultTimeoutId = null;
    this.cabinRemoteRepeatTimerId = null;
    this.cabinRemoteRepeatDirectionId = null;
    this.cabinRemoteMoveInFlight = false;
    this.handleCabinRemoteKeyDown = this.handleCabinRemoteKeyDown.bind(this);
    this.handleCabinRemoteGlobalPointerUp = this.handleCabinRemoteGlobalPointerUp.bind(this);
    this.handleCabinRemoteWindowBlur = this.handleCabinRemoteWindowBlur.bind(this);
    this.handleCabinRemoteVisibilityChange = this.handleCabinRemoteVisibilityChange.bind(this);
    this.displaySettings = loadDisplayPreferences();
    this.settingsHomePage = loadSettingsHomePagePreference();
    this.visibleControlPanelTaskIds = normalizeControlPanelVisibleTaskIds(loadControlPanelVisibleTasks());
    this.customControlPanelButtons = normalizeCustomControlPanelButtons(loadCustomControlPanelButtons());
    this.pendingCustomControlPanelButtonIds = new Set();
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
    });
    this.ui.renderShell();
    this.renderControlPanelTasks(this.visibleControlPanelTaskIds, this.customControlPanelButtons);
    this.renderControlPanelCustomizeMenu(this.customControlPanelButtons);
    this.settingsHomePage = this.ui.setSettingsHomePage(this.settingsHomePage);
    this.ui.setSettingsPage(this.settingsHomePage);
    this.ui.renderPanelsFromLayout(this.activeLayout);
    this.ui.setTheme(this.theme);

    this.panelManager = new PanelManager();
    this.panelManager.init(rootElement);

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
    this.ui.renderPointList([]);

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
        this.refreshActionState();
      },
      onRosUnavailable: () => {
        this.statusMonitorController.stop();
        this.clearS2ResultTimeout();
        this.stopCabinRemoteRepeat();
        this.legacyCommandController?.reset();
        this.ui.syncControlToggleStates(this.legacyCommandController?.getToggleStateSnapshot());
        this.ui.renderTopicInventory([]);
        this.ui.setCabinRemoteCurrentPosition(null);
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
        this.clearS2ResultTimeout();
        this.workspaceView.setExecutionOverlayMessage(message);
        this.ui.setControlFeedback("工作区覆盖层已更新，当前统一显示 /pointAI/result_image_raw。");
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
        this.topicLayerController.updateStats({ tiePointCount: count });
        this.viewerStore.updateIn("scene", { tiePointCount: count });
      },
      onPlanningMarkers: (message) => {
        this.sceneAdapter.normalizePlanningMarkers(message);
        const count = this.sceneView.setPlanningMarkersMessage(message);
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
        const tfFrameCount = this.sceneView.getKnownTransformCount();
        this.topicLayerController.updateStats({ tfFrameCount });
        this.viewerStore.updateIn("scene", { tfFrameCount });
        this.ui.setGripperTfCalibration(this.sceneView.getCameraToTcpCalibration());
      },
    });

    this.cabinRemoteController = new CabinRemoteController({
      rosConnection: this.rosConnectionController,
      sceneView: this.sceneView,
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
      },
    });
    this.systemControlController = new SystemControlController({
      rosConnection: this.rosConnectionController,
      callbacks: {
        onResultMessage: (message) => this.ui.setControlFeedback(message),
        onLog: (message, level) => this.addLog(message, level),
      },
    });
    this.terminalController = new TerminalController({
      ui: this.ui,
      callbacks: {
        onLog: (message, level) => this.addLog(message, level),
      },
    });
    this.terminalController.init();
    this.ui.syncControlToggleStates(this.legacyCommandController.getToggleStateSnapshot());
  }

  init() {
    this.bindUIEvents();
    this.refreshCabinRemoteCurrentPosition();
    this.syncCabinRemoteStatusSummary();
    this.syncDisplayedImageSubscription({ suppressLog: true });
    this.syncPointCloudSubscription({ suppressLog: true });
    this.syncLogSubscription({ suppressLog: true });
    this.refreshActionState();
    this.rosConnectionController.connect();
    return this;
  }

  bindUIEvents() {
    this.ui.onToolbarAction((toolbarAction) => {
      this.handleToolbarAction(toolbarAction);
    });
    this.ui.onConnectionAction((actionId) => {
      this.systemControlController.handle(actionId);
    });
    this.ui.onStatusChipAction((_statusId, actionId) => {
      this.systemControlController.handle(actionId);
    });
    this.ui.onTaskAction((taskAction) => {
      if (taskAction === "moveToPosition") {
        this.handleMoveToPosition();
        return;
      }
      this.taskActionController.handle(taskAction);
      this.refreshActionState();
    });
    this.ui.onControlPanelCustomButtonCreate((draft) => {
      this.addCustomControlPanelButton(draft);
    });
    this.ui.onCustomControlPanelAction((buttonId) => {
      this.handleCustomControlPanelButton(buttonId);
    });
    this.ui.onCustomControlPanelDelete((buttonId) => {
      this.deleteCustomControlPanelButton(buttonId);
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
    this.ui.onImageTopicChange(() => {
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
    this.ui.onCabinRemoteSettingsChange(() => {
      this.syncCabinRemoteStatusSummary();
      this.syncGlobalCabinMoveSpeed({ suppressLog: true });
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
      this.ui.renderLogs(this.logs);
    });
    this.ui.onTerminalAction((action, sessionId) => {
      this.handleTerminalAction(action, sessionId);
    });
    document.removeEventListener("keydown", this.handleCabinRemoteKeyDown, true);
    document.addEventListener("keydown", this.handleCabinRemoteKeyDown, true);
    window.removeEventListener("pointerup", this.handleCabinRemoteGlobalPointerUp, true);
    window.addEventListener("pointerup", this.handleCabinRemoteGlobalPointerUp, true);
    window.removeEventListener("blur", this.handleCabinRemoteWindowBlur, true);
    window.addEventListener("blur", this.handleCabinRemoteWindowBlur, true);
    document.removeEventListener("visibilitychange", this.handleCabinRemoteVisibilityChange, true);
    document.addEventListener("visibilitychange", this.handleCabinRemoteVisibilityChange, true);
  }

  renderControlPanelTasks(taskIds = this.visibleControlPanelTaskIds, customButtons = this.customControlPanelButtons) {
    this.ui.renderControlPanelTasks(taskIds, customButtons);
  }

  renderControlPanelCustomizeMenu(customButtons = this.customControlPanelButtons) {
    this.ui.renderControlPanelCustomizeMenu(customButtons);
  }

  addCustomControlPanelButton(draft) {
    const label = String(draft?.label || draft?.name || "").trim();
    const rawServicePath = String(draft?.servicePath || draft?.service || "").trim();
    const servicePath = rawServicePath
      ? (rawServicePath.startsWith("/") ? rawServicePath : `/${rawServicePath}`)
      : "";
    if (!label || !servicePath) {
      return;
    }

    this.customControlPanelButtons = normalizeCustomControlPanelButtons([
      ...this.customControlPanelButtons,
      {
        id: this.buildCustomControlPanelButtonId(),
        label,
        servicePath,
      },
    ]);
    saveCustomControlPanelButtons(this.customControlPanelButtons);
    this.renderControlPanelTasks(this.visibleControlPanelTaskIds, this.customControlPanelButtons);
    this.renderControlPanelCustomizeMenu(this.customControlPanelButtons);
    this.refreshActionState();
  }

  deleteCustomControlPanelButton(buttonId) {
    this.pendingCustomControlPanelButtonIds.delete(buttonId);
    this.customControlPanelButtons = this.customControlPanelButtons.filter((button) => button.id !== buttonId);
    saveCustomControlPanelButtons(this.customControlPanelButtons);
    this.renderControlPanelTasks(this.visibleControlPanelTaskIds, this.customControlPanelButtons);
    this.renderControlPanelCustomizeMenu(this.customControlPanelButtons);
    this.refreshActionState();
  }

  buildCustomControlPanelButtonId() {
    const randomId = globalThis.crypto?.randomUUID?.();
    if (randomId) {
      return `custom-service-${randomId}`;
    }
    return `custom-service-${Date.now()}-${Math.random().toString(16).slice(2, 10)}`;
  }

  async handleCustomControlPanelButton(buttonId) {
    const button = this.customControlPanelButtons.find((item) => item.id === buttonId);
    if (!button || this.pendingCustomControlPanelButtonIds.has(buttonId)) {
      return;
    }

    this.pendingCustomControlPanelButtonIds.add(buttonId);
    this.ui.setCustomControlPanelButtonPending(buttonId, true);
    try {
      const result = await this.rosConnectionController.callTriggerService(button.servicePath, button.serviceType);
      if (result?.success) {
        this.addLog(result.message || `已触发自定义服务：${button.servicePath}`, "success");
        return;
      }
      this.addLog(result?.message || `自定义服务调用失败：${button.servicePath}`, "warn");
    } finally {
      this.pendingCustomControlPanelButtonIds.delete(buttonId);
      this.ui.setCustomControlPanelButtonPending(buttonId, false);
    }
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
      if (panelId === "logPanel" && nextVisible) {
        this.panelManager.applyDefaultPanelRect(panelId);
      }
      if (panelId === "terminalPanel" && nextVisible) {
        this.panelManager.applyDefaultPanelRect(panelId);
        this.terminalController.handle("ensure").catch((error) => {
          this.addLog(`创建终端失败：${error?.message || String(error)}`, "error");
        });
        window.setTimeout(() => {
          this.terminalController.fitActiveSession();
        }, 80);
      }
      return;
    }

    return;
  }

  async handleMoveToPosition() {
    const payload = { ...DIRECT_CABIN_MOVE_TARGET, speed: this.getGlobalCabinMoveSpeed() };
    this.addLog(
      `准备直接移动到索驱位置: x=${payload.x}, y=${payload.y}, z=${payload.z}, speed=${payload.speed}`,
      "info",
    );
    const result = await this.rosConnectionController.callCabinSingleMoveService(payload);
    if (result.success) {
      this.cabinRemoteController.setLastKnownCabinPosition(payload);
      this.refreshCabinRemoteCurrentPosition();
      this.ui.setControlFeedback(result.message || "索驱已移动到指定索驱位置。");
      this.addLog(result.message || "索驱已移动到指定索驱位置。", "success");
      return;
    }
    this.ui.setControlFeedback(result.message || "索驱移动到指定索驱位置失败。");
    this.addLog(result.message || "索驱移动到指定索驱位置失败。", "error");
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

    const directionId = this.resolveCabinRemoteDirectionFromKey(event.key);
    if (!directionId) {
      return;
    }

    event.preventDefault();
    this.handleCabinRemoteDirection(directionId, "keyboard");
  }

  resolveCabinRemoteDirectionFromKey(key) {
    const normalizedKey = String(key || "").toLowerCase();
    const directionMap = {
      q: "zPositive",
      w: "xPositive",
      e: "zNegative",
      a: "yPositive",
      s: "xNegative",
      d: "yNegative",
    };
    return directionMap[normalizedKey] || null;
  }

  shouldIgnoreCabinRemoteKeyboard(target) {
    const element = target instanceof Element ? target : document.activeElement;
    if (!(element instanceof Element)) {
      return false;
    }

    return Boolean(element.closest(
      "input, textarea, select, [contenteditable=\"true\"], .ui-select-shell, .ui-select-menu, .ui-select-trigger, .xterm, .xterm-helper-textarea",
    ));
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

  syncCabinRemoteStatusSummary() {
    const settings = this.ui.getCabinRemoteSettings();
    const keyboardStatus = settings.keyboardEnabled ? "已开启" : "未开启";
    this.ui.setCabinRemoteStatus(
      `键盘遥控${keyboardStatus}：Q/W/E = Z+/X+/Z-，A/S/D = Y+/X-/Y- | 步距=${Number.isFinite(settings.step) ? settings.step : 50}mm | 全局索驱速度=${Number.isFinite(settings.speed) ? settings.speed : 300}`,
    );
  }

  handleWorkspaceS2Triggered() {
    this.ensureS2OverlayDisplayTopic();
    this.scheduleS2ResultTimeout();
  }

  ensureS2OverlayDisplayTopic() {
    const selectedTopic = this.ui.getSelectedImageTopic();
    if (isOverlayCompatibleImageTopic(selectedTopic)) {
      return;
    }

    this.ui.setSelectedImageTopic(DEFAULT_IMAGE_TOPIC);
    this.syncDisplayedImageSubscription({ suppressLog: true });
    this.addLog("图像卡片已自动切回红外图像，以便显示 S2 覆盖层。", "info");
  }

  scheduleS2ResultTimeout() {
    this.clearS2ResultTimeout();
    this.s2ResultTimeoutId = window.setTimeout(() => {
      this.s2ResultTimeoutId = null;
      this.ui.setControlFeedback("S2 已触发，但等待结果图超时。请检查视觉日志、世界坐标和当前工作区。");
      this.addLog("S2 已触发，但等待结果图超时。", "warn");
    }, S2_RESULT_TIMEOUT_MS);
  }

  clearS2ResultTimeout() {
    if (!this.s2ResultTimeoutId) {
      return;
    }
    window.clearTimeout(this.s2ResultTimeoutId);
    this.s2ResultTimeoutId = null;
  }

  refreshActionState() {
    const ready = this.rosConnectionController.isReady();
    const resources = this.rosConnectionController.getResources();
    const selectedPoints = this.workspaceView.getSelectedPoints();
    const savedPoints = this.workspaceView.getSavedWorkspacePoints();
    this.ui.setTaskButtonsEnabled({
      submitQuad:
        ready &&
        Boolean(resources?.workspaceQuadPublisher && resources?.runWorkspaceS2Publisher) &&
        selectedPoints.length === 4,
      runSavedS2:
        ready &&
        Boolean(resources?.runWorkspaceS2Publisher) &&
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
      moveToPosition: ready && Boolean(resources?.cabinSingleMoveService),
    });
    this.ui.setCustomControlPanelButtonsEnabled(ready);
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
      window.setTimeout(() => {
        const refreshResult = this.rosConnectionController.triggerWorkspaceS2Refresh();
        if (!refreshResult?.success) {
          this.addLog(refreshResult?.message || "外参更新后自动刷新识别失败。", "warn");
          return;
        }
        this.addLog("已按新外参自动重跑当前工作区识别，结果图会使用最新参数刷新。", "success");
      }, 220);
      return;
    }

    this.addLog("外参已热更新，但当前没有已保存工作区，未自动重跑识别。", "info");
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
      this.addLog(`终端操作失败：${error?.message || String(error)}`, "error");
    }
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
    const rawText = String(message?.msg || "");
    if (!rawText.startsWith("[stdout]")) {
      return;
    }
    const nodeName = String(message?.name || "").split("/").filter(Boolean).pop() || "unknown";
    const content = rawText.replace(/^\[stdout\]\s*/, "");
    const level = this.mapRosLogLevel(message?.level);
    const stamp = message?.header?.stamp;
    const sortKey = stamp?.secs
      ? (Number(stamp.secs) * 1000) + Math.floor(Number(stamp.nsecs || 0) / 1e6)
      : Date.now();
    const timestamp = new Date(sortKey).toLocaleTimeString("zh-CN", { hour12: false });
    this.systemLogs = [{
      timestamp,
      message: `[${nodeName}] ${content}`,
      level,
      nodeName,
      source: "system",
      sortKey,
    }, ...this.systemLogs].slice(0, 200);
    this.renderLogView();
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
