import { SceneAdapter } from "../data/SceneAdapter.js";
import { DEFAULT_LAYOUTS } from "../layout/defaultLayouts.js";
import { LayoutManager } from "../layout/LayoutManager.js";
import { PANEL_REGISTRY } from "../panels/panelRegistry.js";
import { ViewerStore } from "../state/ViewerStore.js";
import { LegacyCommandController } from "../controllers/LegacyCommandController.js";
import { RosConnectionController } from "../controllers/RosConnectionController.js";
import { StatusMonitorController } from "../controllers/StatusMonitorController.js";
import { TaskActionController } from "../controllers/TaskActionController.js";
import { TopicLayerController } from "../controllers/TopicLayerController.js";
import { PanelManager } from "../ui/PanelManager.js";
import { UIController } from "../ui/UIController.js";
import {
  loadDisplayPreferences,
  loadViewerLayout,
  saveDisplayPreferences,
  saveViewerLayout,
} from "../utils/storage.js";
import { Scene3DView } from "../views/Scene3DView.js";
import { WorkspaceCanvasView } from "../views/WorkspaceCanvasView.js";

export class TieRobotFrontApp {
  constructor(rootElement) {
    this.rootElement = rootElement;
    this.logs = [];
    this.frontendLogs = [];
    this.displaySettings = loadDisplayPreferences();
    this.viewerStore = new ViewerStore();
    this.sceneAdapter = new SceneAdapter();
    this.panelRegistry = PANEL_REGISTRY;
    this.layoutManager = new LayoutManager({
      defaults: DEFAULT_LAYOUTS,
      loadLayout: loadViewerLayout,
      saveLayout: saveViewerLayout,
    });
    this.activeLayout = this.layoutManager.getLayout("executionDebug");

    this.ui = new UIController(rootElement);
    this.ui.renderShell();

    this.panelManager = new PanelManager();
    this.panelManager.init(rootElement);
    rootElement.querySelectorAll(".floating-panel").forEach((panel) => {
      this.panelManager.registerPanel(panel);
    });

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
    this.ui.setDisplaySettings(this.displaySettings);
    this.ui.renderPointList([]);

    this.sceneView = new Scene3DView({
      container: this.ui.getSceneContainer(),
    });

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
        this.refreshActionState();
      },
      onRosUnavailable: () => {
        this.statusMonitorController.stop();
        this.legacyCommandController?.reset();
        this.ui.syncControlToggleStates(this.legacyCommandController?.getToggleStateSnapshot());
        this.viewerStore.updateIn("connection", { ready: false });
        this.refreshActionState();
      },
      onLog: (message, level) => this.addLog(message, level),
      onSystemLog: (message) => this.handleSystemLog(message),
      onBaseImage: (message) => {
        this.workspaceView.setBaseImageMessage(message);
        if (this.workspaceView.getSelectedPoints().length === 0) {
          this.ui.setResultMessage("IR 图像已就绪，直接在图上点 4 个角点。");
        }
      },
      onSavedWorkspacePayload: (payload) => {
        this.workspaceView.setSavedWorkspacePayload(payload);
        this.refreshActionState();
      },
      onExecutionOverlay: (message) => {
        this.workspaceView.setExecutionOverlayMessage(message);
        this.ui.setResultMessage("工作区覆盖层已更新，当前统一显示 /pointAI/result_image_raw。");
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
        const tfFrameCount = this.sceneView.getKnownTransformCount();
        this.topicLayerController.updateStats({ tfFrameCount });
        this.viewerStore.updateIn("scene", { tfFrameCount });
      },
    });

    this.taskActionController = new TaskActionController({
      rosConnection: this.rosConnectionController,
      workspaceView: this.workspaceView,
      callbacks: {
        onResultMessage: (message) => this.ui.setResultMessage(message),
        onLog: (message, level) => this.addLog(message, level),
      },
    });

    this.legacyCommandController = new LegacyCommandController({
      rosConnection: this.rosConnectionController,
      callbacks: {
        onResultMessage: (message) => this.ui.setResultMessage(message),
        onLog: (message, level) => this.addLog(message, level),
      },
    });
    this.ui.syncControlToggleStates(this.legacyCommandController.getToggleStateSnapshot());
  }

  init() {
    this.bindUIEvents();
    this.refreshActionState();
    this.rosConnectionController.connect();
    return this;
  }

  bindUIEvents() {
    this.ui.onToolbarAction((toolbarAction) => {
      this.handleToolbarAction(toolbarAction);
    });
    this.ui.onTaskAction((taskAction) => {
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
    this.ui.onTopicLayerControlsChange((state) => {
      this.topicLayerController.handleLayerControlsChange(state);
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
      this.ui.renderLogs(this.logs);
    });
  }

  handleToolbarAction(action) {
    if (!action) {
      return;
    }

    if (action.startsWith("toggle-panel:")) {
      const panelId = action.split(":")[1];
      this.ui.togglePanelVisible(panelId);
      return;
    }

    return;
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
    });
    this.ui.setWorkspaceButtonsEnabled({
      undo: selectedPoints.length > 0,
      clear: selectedPoints.length > 0,
    });
  }

  addLog(message, level = "info") {
    const timestamp = new Date().toLocaleTimeString("zh-CN", { hour12: false });
    this.frontendLogs = [{ timestamp, message, level }, ...this.frontendLogs].slice(0, 80);
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
    const timestamp = stamp?.secs
      ? new Date((Number(stamp.secs) * 1000) + Math.floor(Number(stamp.nsecs || 0) / 1e6)).toLocaleTimeString("zh-CN", { hour12: false })
      : new Date().toLocaleTimeString("zh-CN", { hour12: false });
    this.logs = [{ timestamp, message: `[${nodeName}] ${content}`, level }, ...this.logs].slice(0, 200);
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
