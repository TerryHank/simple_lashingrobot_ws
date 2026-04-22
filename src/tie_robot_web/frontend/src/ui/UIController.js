import { LEGACY_PARAMETER_DEFAULTS } from "../config/legacyCommandCatalog.js";
import {
  CONTROL_PANEL_TASKS,
  getControlToggleDefinition,
  getControlPanelCommandTone,
  getControlPanelGroups,
  getInitialControlToggleStateMap,
} from "../config/controlPanelCatalog.js";
import {
  getPointCloudSourceLabel,
  POINT_CLOUD_SOURCES,
  SCENE_VIEW_MODES,
  TOPIC_LAYER_MODES,
  getTopicLayerModeLabel,
} from "../config/topicLayerCatalog.js";
import { STATUS_MONITORS } from "../config/statusMonitorCatalog.js";

const PANEL_TOGGLE_BUTTONS = [
  { id: "controlPanel", label: "控制面板" },
  { id: "workspacePanel", label: "工作区" },
  { id: "topicLayersPanel", label: "话题图层" },
  { id: "logPanel", label: "日志" },
];

const DEFAULT_PANEL_VISIBILITY = {
  controlPanel: true,
  workspacePanel: true,
  topicLayersPanel: true,
  logPanel: false,
};

const DISPLAY_MODE_LABELS = {
  auto: "自动增强",
  strong: "强增强",
  raw: "原图",
};

const CONNECTION_LABELS = {
  info: "连接中",
  success: "连接成功",
  warn: "连接失败",
  error: "连接失败",
};

export class UIController {
  constructor(rootElement) {
    this.rootElement = rootElement;
    this.refs = {};
  }

  renderShell() {
    this.rootElement.innerHTML = `
      <div class="app-shell">
        <div id="sceneBackground" class="scene-background"></div>
        <div class="scene-overlay"></div>

        <header class="top-toolbar">
          <div class="toolbar-group toolbar-brand-group">
            <div class="toolbar-brand">绑扎机器人</div>
          </div>

          <div class="toolbar-group">
            ${PANEL_TOGGLE_BUTTONS.map((panel) => `
              <button
                class="toolbar-pill active"
                type="button"
                data-toolbar-action="toggle-panel:${panel.id}"
                data-panel-toggle="${panel.id}"
              >${panel.label}</button>
            `).join("")}
          </div>

          <div class="toolbar-group toolbar-status-group">
            <div id="connectionBadge" class="toolbar-connection-badge info" title="连接中">连接中</div>
            <div id="voltageBadge" class="toolbar-voltage-badge" title="机器人电压暂未获取">电压 --.-V</div>
            <div id="statusChips" class="toolbar-status-chips"></div>
          </div>

          <div class="toolbar-group toolbar-link-group">
            <a class="toolbar-pill toolbar-link" href="/help/" target="_blank" rel="noreferrer">帮助</a>
          </div>
        </header>

        <div class="floating-panels">
          <section id="controlPanel" class="floating-panel panel-control" data-size="wide">
            <div class="panel-header">
              <div>
                <div class="panel-title">控制面板</div>
                <div class="panel-subtitle">快速排查、调试与执行入口</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:controlPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content control-panel-content">
              <div class="section-title control-section-title">主链任务</div>
              <div id="controlPanelTaskGrid" class="control-button-grid"></div>

              <div class="section-title control-section-title">调试与排查</div>
              <div id="controlPanelGroups" class="control-group-stack"></div>
            </div>
          </section>

          <section id="workspacePanel" class="floating-panel panel-workspace">
            <div class="panel-header">
              <div>
                <div class="panel-title">工作区</div>
                <div class="panel-subtitle">IR 工作区选点与结果叠加</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:workspacePanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content workspace-panel-content">
              <div class="canvas-stage">
                <canvas id="irCanvas" width="640" height="480"></canvas>
                <canvas id="overlayCanvas" width="640" height="480"></canvas>
              </div>

              <div class="workspace-metadata">
                <div class="section-title">已选角点</div>
                <ol id="selectedPoints" class="point-list"></ol>
                <div class="button-row">
                  <button class="secondary-btn" data-workspace-action="undo" disabled>撤销最后一点</button>
                  <button class="secondary-btn" data-workspace-action="clear" disabled>清空重选</button>
                </div>
              </div>

              <div class="section-title">显示增强</div>
              <div class="field-grid single-column">
                <div class="field">
                  <label for="displayMode">模式</label>
                  <select id="displayMode">
                    <option value="auto">自动增强</option>
                    <option value="strong">强增强</option>
                    <option value="raw">原图</option>
                  </select>
                </div>
                <div class="field">
                  <label for="gammaRange">伽马</label>
                  <input id="gammaRange" type="range" min="0.40" max="1.60" step="0.05" value="0.85" />
                </div>
                <div class="field">
                  <label for="overlayOpacityRange">覆盖透明度</label>
                  <input id="overlayOpacityRange" type="range" min="0.00" max="1.00" step="0.02" value="0.88" />
                </div>
                <div class="info-block mono" id="displaySettingSummary">模式=自动增强 伽马=0.85 覆盖=0.88</div>
              </div>
            </div>
          </section>

          <section id="topicLayersPanel" class="floating-panel panel-topic-layers">
            <div class="panel-header">
              <div>
                <div class="panel-title">话题图层</div>
                <div class="panel-subtitle">按相机模态、TF 与话题切换图层</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:topicLayersPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content">
              <div class="section-title">场景视角</div>
              <div class="field-grid compact-grid">
                <div class="field">
                  <label for="sceneViewMode">视角</label>
                  <select id="sceneViewMode">
                    ${SCENE_VIEW_MODES.map((mode) => `
                      <option value="${mode.id}">${mode.label}</option>
                    `).join("")}
                  </select>
                </div>
                <label class="checkbox-field">
                  <input id="followCameraToggle" type="checkbox" />
                  <span>跟随相机</span>
                </label>
              </div>

              <div class="section-title">显示模式</div>
              <div class="field-grid compact-grid">
                <div class="field">
                  <label for="topicLayerMode">模式</label>
                  <select id="topicLayerMode">
                    ${TOPIC_LAYER_MODES.map((mode) => `
                      <option value="${mode.id}">${mode.label}</option>
                    `).join("")}
                  </select>
                </div>
                <div class="field">
                  <label for="pointCloudSource">点云源</label>
                  <select id="pointCloudSource">
                    ${POINT_CLOUD_SOURCES.map((source) => `
                      <option value="${source.id}">${source.label}</option>
                    `).join("")}
                  </select>
                </div>
              </div>

              <div class="toggle-grid">
                <label class="checkbox-field"><input id="showRobotToggle" type="checkbox" checked /><span>机器</span></label>
                <label class="checkbox-field"><input id="showAxesToggle" type="checkbox" checked /><span>坐标轴</span></label>
                <label class="checkbox-field"><input id="showPointCloudToggle" type="checkbox" checked /><span>点云</span></label>
                <label class="checkbox-field"><input id="showTiePointsToggle" type="checkbox" checked /><span>绑扎点</span></label>
                <label class="checkbox-field"><input id="showPlanningMarkersToggle" type="checkbox" /><span>规划点</span></label>
              </div>

              <div class="field-grid compact-grid" style="margin-top: 12px;">
                <div class="field">
                  <label for="pointSizeRange">点大小</label>
                  <input id="pointSizeRange" type="range" min="0.01" max="0.10" step="0.005" value="0.035" />
                </div>
                <div class="field">
                  <label for="pointOpacityRange">点透明度</label>
                  <input id="pointOpacityRange" type="range" min="0.15" max="1.00" step="0.05" value="0.78" />
                </div>
              </div>

              <div id="topicLayerSummary" class="info-block mono">模式=点云 + 绑扎点 点云源=滤波世界点云 点大小=0.035 透明度=0.78</div>

              <div class="section-title">当前数据量</div>
              <div id="topicLayerStats" class="stats-grid"></div>
            </div>
          </section>

          <section id="logPanel" class="floating-panel panel-log">
            <div class="panel-header">
              <div>
              <div class="panel-title">日志</div>
                <div class="panel-subtitle">终端日志订阅</div>
              </div>
              <div class="panel-actions">
                <button id="clearLogs" class="panel-action-btn" type="button" title="清空日志">⌫</button>
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:logPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content">
              <ul id="logList" class="log-list"></ul>
            </div>
          </section>
        </div>
      </div>
    `;
    this.bindRefs();
    this.applyDefaultPanelVisibility();
    this.renderStatusChips();
    this.renderControlPanelTasks();
    this.renderLegacyCommands();
    this.renderPointList([]);
    this.renderLogs([]);
    this.renderTopicLayerStats({
      filteredWorldCoordCount: 0,
      rawWorldCoordCount: 0,
      tiePointCount: 0,
      planningPointCount: 0,
      tfFrameCount: 0,
    });
  }

  bindRefs() {
    this.refs.sceneBackground = this.rootElement.querySelector("#sceneBackground");
    this.refs.irCanvas = this.rootElement.querySelector("#irCanvas");
    this.refs.overlayCanvas = this.rootElement.querySelector("#overlayCanvas");
    this.refs.sceneViewMode = this.rootElement.querySelector("#sceneViewMode");
    this.refs.followCameraToggle = this.rootElement.querySelector("#followCameraToggle");
    this.refs.connectionBadge = this.rootElement.querySelector("#connectionBadge");
    this.refs.voltageBadge = this.rootElement.querySelector("#voltageBadge");
    this.refs.controlPanelTaskGrid = this.rootElement.querySelector("#controlPanelTaskGrid");
    this.refs.controlPanelGroups = this.rootElement.querySelector("#controlPanelGroups");
    this.refs.topicLayerMode = this.rootElement.querySelector("#topicLayerMode");
    this.refs.pointCloudSource = this.rootElement.querySelector("#pointCloudSource");
    this.refs.showRobotToggle = this.rootElement.querySelector("#showRobotToggle");
    this.refs.showAxesToggle = this.rootElement.querySelector("#showAxesToggle");
    this.refs.showPointCloudToggle = this.rootElement.querySelector("#showPointCloudToggle");
    this.refs.showTiePointsToggle = this.rootElement.querySelector("#showTiePointsToggle");
    this.refs.showPlanningMarkersToggle = this.rootElement.querySelector("#showPlanningMarkersToggle");
    this.refs.pointSizeRange = this.rootElement.querySelector("#pointSizeRange");
    this.refs.pointOpacityRange = this.rootElement.querySelector("#pointOpacityRange");
    this.refs.topicLayerSummary = this.rootElement.querySelector("#topicLayerSummary");
    this.refs.topicLayerStats = this.rootElement.querySelector("#topicLayerStats");
    this.refs.selectedPoints = this.rootElement.querySelector("#selectedPoints");
    this.refs.displayMode = this.rootElement.querySelector("#displayMode");
    this.refs.gammaRange = this.rootElement.querySelector("#gammaRange");
    this.refs.overlayOpacityRange = this.rootElement.querySelector("#overlayOpacityRange");
    this.refs.displaySettingSummary = this.rootElement.querySelector("#displaySettingSummary");
    this.refs.statusChips = this.rootElement.querySelector("#statusChips");
    this.refs.logList = this.rootElement.querySelector("#logList");
    this.refs.clearLogs = this.rootElement.querySelector("#clearLogs");
    this.refs.taskButtons = [...this.rootElement.querySelectorAll("[data-task-action]")];
    this.refs.workspaceButtons = [...this.rootElement.querySelectorAll("[data-workspace-action]")];
    this.refs.toolbarButtons = [...this.rootElement.querySelectorAll("[data-toolbar-action]")];
    this.refs.panelElements = new Map(
      PANEL_TOGGLE_BUTTONS.map(({ id }) => [id, this.rootElement.querySelector(`#${id}`)]),
    );
    this.refs.panelToggleButtons = [...this.rootElement.querySelectorAll("[data-panel-toggle]")];
    this.refs.topicLayerInputs = [
      this.refs.topicLayerMode,
      this.refs.pointCloudSource,
      this.refs.showRobotToggle,
      this.refs.showAxesToggle,
      this.refs.showPointCloudToggle,
      this.refs.showTiePointsToggle,
      this.refs.showPlanningMarkersToggle,
      this.refs.pointSizeRange,
      this.refs.pointOpacityRange,
    ];
    this.refs.sceneInputs = [this.refs.sceneViewMode, this.refs.followCameraToggle];
  }

  applyDefaultPanelVisibility() {
    Object.entries(DEFAULT_PANEL_VISIBILITY).forEach(([panelId, visible]) => {
      this.setPanelVisible(panelId, visible);
    });
  }

  renderStatusChips() {
    this.refs.statusChips.innerHTML = STATUS_MONITORS.map((item) => `
      <span class="status-chip info" data-status-id="${item.id}" title="${item.label}">
        <span>${item.label}</span>
      </span>
    `).join("");
  }

  renderControlPanelTasks() {
    this.refs.controlPanelTaskGrid.innerHTML = CONTROL_PANEL_TASKS.map((task) => `
      <button
        class="control-action-btn"
        type="button"
        data-task-action="${task.id}"
        data-tone="${task.tone}"
        disabled
      >${task.label.replaceAll("\n", "<br />")}</button>
    `).join("");
    this.refs.taskButtons = [...this.refs.controlPanelTaskGrid.querySelectorAll("[data-task-action]")];
  }

  renderLegacyCommands() {
    this.refs.controlPanelGroups.innerHTML = getControlPanelGroups().map(({ title, controls }) => `
      <div class="control-command-group">
        <div class="control-group-title">${title}</div>
        <div class="control-button-grid">
          ${controls.map((control) => `
            ${control.kind === "toggle" ? `
              <button
                class="control-action-btn ${control.active ? "is-active" : ""}"
                data-control-toggle="${control.id}"
                data-tone="${control.tone}"
                data-active="${control.active ? "true" : "false"}"
                type="button"
              >
                ${control.label}
              </button>
            ` : `
              <button
                class="control-action-btn"
                data-legacy-command="${control.commandId}"
                data-tone="${getControlPanelCommandTone(control.commandId)}"
                type="button"
              >
                ${control.label}
              </button>
            `}
          `).join("")}
        </div>
      </div>
    `).join("");
    this.refs.legacyButtons = [...this.rootElement.querySelectorAll("[data-legacy-command]")];
    this.refs.toggleButtons = [...this.rootElement.querySelectorAll("[data-control-toggle]")];
  }

  getCanvasRefs() {
    return { canvas: this.refs.irCanvas, overlayCanvas: this.refs.overlayCanvas };
  }

  getSceneContainer() {
    return this.refs.sceneBackground;
  }

  getDisplayControls() {
    return {
      displayMode: this.refs.displayMode,
      gammaRange: this.refs.gammaRange,
      overlayOpacityRange: this.refs.overlayOpacityRange,
    };
  }

  getParameterValues() {
    return Object.keys(LEGACY_PARAMETER_DEFAULTS).reduce((accumulator, key) => {
      const input = this.rootElement.querySelector(`#param-${key}`);
      accumulator[key] = Number.parseFloat(input?.value ?? LEGACY_PARAMETER_DEFAULTS[key]);
      return accumulator;
    }, {});
  }

  getTopicLayerState() {
    return {
      mode: this.refs.topicLayerMode.value,
      pointCloudSource: this.refs.pointCloudSource.value,
      showRobot: this.refs.showRobotToggle.checked,
      showAxes: this.refs.showAxesToggle.checked,
      showPointCloud: this.refs.showPointCloudToggle.checked,
      showTiePoints: this.refs.showTiePointsToggle.checked,
      showPlanningMarkers: this.refs.showPlanningMarkersToggle.checked,
      pointSize: Number.parseFloat(this.refs.pointSizeRange.value),
      pointOpacity: Number.parseFloat(this.refs.pointOpacityRange.value),
      viewMode: this.refs.sceneViewMode.value,
      followCamera: this.refs.followCameraToggle.checked,
    };
  }

  getSceneViewState() {
    return {
      viewMode: this.refs.sceneViewMode.value,
      followCamera: this.refs.followCameraToggle.checked,
    };
  }

  isPanelVisible(panelId) {
    const panel = this.refs.panelElements.get(panelId);
    return Boolean(panel && panel.dataset.visible !== "false");
  }

  setPanelVisible(panelId, visible) {
    const panel = this.refs.panelElements.get(panelId);
    if (!panel) {
      return;
    }
    panel.hidden = !visible;
    panel.dataset.visible = visible ? "true" : "false";
    panel.style.display = visible ? "" : "none";
    this.refs.panelToggleButtons
      .filter((button) => button.dataset.panelToggle === panelId)
      .forEach((button) => {
        button.classList.toggle("active", visible);
        button.setAttribute("aria-pressed", visible ? "true" : "false");
      });
  }

  togglePanelVisible(panelId) {
    const nextVisible = !this.isPanelVisible(panelId);
    this.setPanelVisible(panelId, nextVisible);
    return nextVisible;
  }

  onToolbarAction(callback) {
    this.refs.toolbarButtons.forEach((button) => {
      button.addEventListener("click", (event) => {
        event.preventDefault();
        event.stopPropagation();
        callback(button.dataset.toolbarAction);
      });
    });
  }

  onTaskAction(callback) {
    this.refs.taskButtons.forEach((button) => {
      button.addEventListener("click", () => callback(button.dataset.taskAction));
    });
  }

  onWorkspaceAction(callback) {
    this.refs.workspaceButtons.forEach((button) => {
      button.addEventListener("click", () => callback(button.dataset.workspaceAction));
    });
  }

  onLegacyCommand(callback) {
    this.refs.legacyButtons.forEach((button) => {
      button.addEventListener("click", () => callback(Number(button.dataset.legacyCommand)));
    });
  }

  onControlToggle(callback) {
    this.refs.toggleButtons.forEach((button) => {
      button.addEventListener("click", () => callback(button.dataset.controlToggle));
    });
  }

  onDisplaySettingsChange(callback) {
    [this.refs.displayMode, this.refs.gammaRange, this.refs.overlayOpacityRange].forEach((element) => {
      element.addEventListener("input", () => callback(this.getDisplaySettings()));
      element.addEventListener("change", () => callback(this.getDisplaySettings()));
    });
  }

  onSceneControlsChange(callback) {
    this.refs.sceneInputs.forEach((element) => {
      const eventName = element.type === "checkbox" ? "change" : "input";
      element.addEventListener(eventName, () => callback(this.getSceneViewState()));
    });
  }

  onTopicLayerControlsChange(callback) {
    this.refs.topicLayerInputs.forEach((element) => {
      const eventName = element.type === "checkbox" ? "change" : "input";
      element.addEventListener(eventName, () => callback(this.getTopicLayerState()));
      if (eventName !== "change") {
        element.addEventListener("change", () => callback(this.getTopicLayerState()));
      }
    });
  }

  onClearLogs(callback) {
    this.refs.clearLogs.addEventListener("click", callback);
  }

  setConnectionInfo(url, message, level = "info") {
    const normalizedLevel = CONNECTION_LABELS[level] ? level : "info";
    this.refs.connectionBadge.textContent = CONNECTION_LABELS[normalizedLevel];
    this.refs.connectionBadge.className = `toolbar-connection-badge ${normalizedLevel}`;
    this.refs.connectionBadge.title = message || url || CONNECTION_LABELS[normalizedLevel];
  }

  setBatteryVoltage(voltage) {
    if (!Number.isFinite(voltage) || voltage <= 0 || voltage > 100) {
      this.refs.voltageBadge.textContent = "电压 --.-V";
      this.refs.voltageBadge.title = "机器人电压无效或超出显示范围";
      this.refs.voltageBadge.className = "toolbar-voltage-badge warn";
      return;
    }
    const formatted = Number(voltage).toFixed(1);
    this.refs.voltageBadge.textContent = `电压 ${formatted}V`;
    this.refs.voltageBadge.title = `机器人电压 ${formatted}V`;
    this.refs.voltageBadge.className = "toolbar-voltage-badge success";
  }

  setTaskButtonsEnabled(enabledMap) {
    this.refs.taskButtons.forEach((button) => {
      const key = button.dataset.taskAction;
      button.disabled = enabledMap[key] === false;
    });
  }

  setWorkspaceButtonsEnabled(enabledMap) {
    this.refs.workspaceButtons.forEach((button) => {
      const key = button.dataset.workspaceAction;
      button.disabled = enabledMap[key] === false;
    });
  }

  setDisplaySettings(settings) {
    this.refs.displayMode.value = settings.mode;
    this.refs.gammaRange.value = settings.gamma.toFixed(2);
    this.refs.overlayOpacityRange.value = settings.overlayOpacity.toFixed(2);
    this.refs.displaySettingSummary.textContent =
      `模式=${DISPLAY_MODE_LABELS[settings.mode] || settings.mode} 伽马=${settings.gamma.toFixed(2)} 覆盖=${settings.overlayOpacity.toFixed(2)}`;
  }

  setTopicLayerState(state) {
    this.refs.topicLayerMode.value = state.mode;
    this.refs.pointCloudSource.value = state.pointCloudSource;
    this.refs.showRobotToggle.checked = Boolean(state.showRobot);
    this.refs.showAxesToggle.checked = Boolean(state.showAxes);
    this.refs.showPointCloudToggle.checked = Boolean(state.showPointCloud);
    this.refs.showTiePointsToggle.checked = Boolean(state.showTiePoints);
    this.refs.showPlanningMarkersToggle.checked = Boolean(state.showPlanningMarkers);
    this.refs.pointSizeRange.value = Number(state.pointSize).toFixed(3);
    this.refs.pointOpacityRange.value = Number(state.pointOpacity).toFixed(2);
    this.refs.sceneViewMode.value = state.viewMode;
    this.refs.followCameraToggle.checked = Boolean(state.followCamera);
    this.refs.topicLayerSummary.textContent =
      `模式=${getTopicLayerModeLabel(state.mode)} 点云源=${getPointCloudSourceLabel(state.pointCloudSource)} 点大小=${Number(state.pointSize).toFixed(3)} 透明度=${Number(state.pointOpacity).toFixed(2)}`;
  }

  renderTopicLayerStats(stats) {
    this.refs.topicLayerStats.innerHTML = `
      <div class="stats-card"><span>滤波点云</span><strong>${stats.filteredWorldCoordCount}</strong></div>
      <div class="stats-card"><span>原始点云</span><strong>${stats.rawWorldCoordCount}</strong></div>
      <div class="stats-card"><span>绑扎点</span><strong>${stats.tiePointCount}</strong></div>
      <div class="stats-card"><span>规划点</span><strong>${stats.planningPointCount}</strong></div>
      <div class="stats-card"><span>TF 帧</span><strong>${stats.tfFrameCount}</strong></div>
    `;
  }

  getDisplaySettings() {
    return {
      mode: this.refs.displayMode.value,
      gamma: Number.parseFloat(this.refs.gammaRange.value),
      overlayOpacity: Number.parseFloat(this.refs.overlayOpacityRange.value),
    };
  }

  renderPointList(points) {
    if (!points.length) {
      this.refs.selectedPoints.innerHTML = `<li class="point-item">还没有点，直接在 IR 图上点 4 个角点。</li>`;
      return;
    }
    this.refs.selectedPoints.innerHTML = points.map((point, index) => `
      <li class="point-item mono">${index + 1}. x=${point.x}, y=${point.y}</li>
    `).join("");
  }

  setStatusChipState(statusId, level, detail) {
    const chip = this.rootElement.querySelector(`[data-status-id="${statusId}"]`);
    if (!chip) {
      return;
    }
    chip.className = `status-chip ${level}`;
    chip.title = detail || "";
  }

  setResultMessage(message) {
    if (!this.refs.resultMessage) {
      return;
    }
    this.refs.resultMessage.textContent = message;
    this.refs.resultMessage.title = message;
  }

  syncControlToggleStates(stateMap = {}) {
    const initialStates = getInitialControlToggleStateMap();
    Object.entries({ ...initialStates, ...stateMap }).forEach(([toggleId, state]) => {
      this.setControlToggleState(toggleId, state);
    });
  }

  setControlToggleState(toggleId, state) {
    const button = this.rootElement.querySelector(`[data-control-toggle="${toggleId}"]`);
    const definition = getControlToggleDefinition(toggleId);
    if (!button || !definition) {
      return;
    }
    const active = Boolean(state?.value);
    button.dataset.active = active ? "true" : "false";
    button.dataset.tone = state?.tone || (active ? definition.activeTone : definition.inactiveTone);
    button.classList.toggle("is-active", active);
    button.textContent = state?.label || (active ? definition.activeLabel : definition.inactiveLabel);
  }

  renderLogs(logs) {
    if (!logs.length) {
      this.refs.logList.innerHTML = `<li class="log-item info">暂无终端日志，等待节点 stdout 输出。</li>`;
      return;
    }
    this.refs.logList.innerHTML = logs.map((entry) => `
      <li class="log-item ${entry.level}">
        <div class="log-meta mono">${entry.timestamp}</div>
        <div>${entry.message}</div>
      </li>
    `).join("");
  }
}
