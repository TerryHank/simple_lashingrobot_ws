import {
  CONTROL_PANEL_TASKS,
  getControlToggleDefinition,
  getControlPanelGroups,
  getInitialControlToggleStateMap,
  normalizeControlPanelVisibleTaskIds,
  normalizeCustomControlPanelButtons,
} from "../config/controlPanelCatalog.js";
import { LEGACY_PARAMETER_DEFAULTS } from "../config/legacyCommandCatalog.js";
import {
  getPointCloudSourceLabel,
  POINT_CLOUD_SOURCES,
  SCENE_VIEW_MODES,
  TOPIC_LAYER_MODES,
  getTopicLayerModeLabel,
} from "../config/topicLayerCatalog.js";
import { DEFAULT_IMAGE_TOPIC, IMAGE_TOPIC_OPTIONS, getImageTopicLabel } from "../config/imageTopicCatalog.js";
import { DEFAULT_LOG_TOPIC, LOG_TOPIC_OPTIONS, getLogTopicLabel } from "../config/logTopicCatalog.js";
import { STATUS_MONITORS } from "../config/statusMonitorCatalog.js";

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

const CONNECTION_ACTIONS = {
  info: { id: "", label: "" },
  success: { id: "restartRosStack", label: "重启ROS" },
  warn: { id: "startRosStack", label: "启动ROS" },
  error: { id: "startRosStack", label: "启动ROS" },
};

const SETTINGS_PAGE_OPTIONS = [
  { id: "topics", label: "话题总览" },
  { id: "workspace", label: "工作区选点" },
  { id: "scene", label: "显示与视角" },
  { id: "layers", label: "图层与数据" },
  { id: "cabinRemote", label: "索驱遥控" },
  { id: "calibration", label: "相机-TCP外参" },
];

const SETTINGS_PAGE_OPTION_MARKUP = `
  <option value="topics">话题总览</option>
  <option value="workspace">工作区选点</option>
  <option value="scene">显示与视角</option>
  <option value="layers">图层与数据</option>
  <option value="cabinRemote">索驱遥控</option>
  <option value="calibration">相机-TCP外参</option>
`;

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

export class UIController {
  constructor(rootElement, { panelRegistry = [], initialLayout = null } = {}) {
    this.rootElement = rootElement;
    this.refs = {};
    this.panelRegistry = panelRegistry;
    this.initialLayout = initialLayout;
    this.customSelects = new Map();
    this.taskButtonFeedbackTimers = new Map();
    this.topicInventoryExpanded = new Map();
    this.settingsHomePageId = "topics";
    this.handleSettingsHomePageChange = null;
    this.handleCustomControlPanelDelete = null;
    this.customControlPanelEditMode = false;
    this.customControlPanelLongPressTimerId = null;
    this.customControlPanelLongPressContext = null;
    this.customControlPanelDragState = null;
    this.handleDocumentPointerDown = this.handleDocumentPointerDown.bind(this);
    this.handleDocumentKeyDown = this.handleDocumentKeyDown.bind(this);
    this.handleCustomControlPanelPointerMove = this.handleCustomControlPanelPointerMove.bind(this);
    this.handleCustomControlPanelPointerUp = this.handleCustomControlPanelPointerUp.bind(this);
  }

  getPanelToggleButtons() {
    return this.panelRegistry.map((panel) => ({
      id: panel.id,
      label: panel.title,
    }));
  }

  getInitialPanelVisibility() {
    const layoutPanels = this.initialLayout?.panels || {};
    return this.panelRegistry.reduce((accumulator, panel) => {
      const layoutVisible = layoutPanels[panel.id]?.visible;
      accumulator[panel.id] = typeof layoutVisible === "boolean"
        ? layoutVisible
        : panel.defaultVisible !== false;
      return accumulator;
    }, {});
  }

  renderShell() {
    const panelToggleButtons = this.getPanelToggleButtons();
    this.rootElement.innerHTML = `
      <div class="app-shell">
        <div id="sceneBackground" class="scene-background"></div>
        <div class="scene-overlay"></div>

        <header class="top-toolbar">
          <div class="toolbar-group toolbar-brand-group">
            <div class="toolbar-brand">绑扎机器人</div>
          </div>

          <div class="toolbar-group">
            ${panelToggleButtons.map((panel) => `
              <button
                class="toolbar-pill active"
                type="button"
                data-toolbar-action="toggle-panel:${panel.id}"
                data-panel-toggle="${panel.id}"
              >${panel.label}</button>
            `).join("")}
          </div>

          <div class="toolbar-group toolbar-status-group">
            <div class="toolbar-status-strip">
              <button
                id="connectionBadge"
                class="toolbar-connection-badge info"
                type="button"
                data-connection-action=""
                data-has-action="false"
                title="连接中"
              >
                  <span class="toolbar-connection-text">
                    <span class="toolbar-connection-label">连接中</span>
                    <span class="toolbar-connection-action-label"></span>
                  </span>
              </button>
              <div id="voltageBadge" class="toolbar-voltage-badge" title="机器人电压暂未获取">电压 --.-V</div>
              <div id="statusCapsuleGrid" class="toolbar-status-capsules"></div>
            </div>
          </div>

          <div class="toolbar-group toolbar-link-group">
            <button
              id="themeToggle"
              class="toolbar-pill toolbar-theme-toggle"
              type="button"
              data-toolbar-action="toggle-theme"
              title="切换主题"
              aria-label="切换主题"
            ><span class="toolbar-theme-icon" aria-hidden="true">🌙</span></button>
            <a class="toolbar-pill toolbar-link" href="/help/" target="_blank" rel="noreferrer">帮助</a>
          </div>
        </header>

        <div class="floating-panels">
          <section id="controlPanel" class="floating-panel panel-control" data-size="wide">
            <div class="panel-header">
              <div>
                <div class="panel-title">控制面板</div>
              </div>
              <div class="panel-actions">
                <button
                  id="controlPanelCustomizeButton"
                  class="panel-action-btn"
                  type="button"
                  title="自定义按钮"
                  aria-label="自定义控制面板按钮"
                  aria-expanded="false"
                >＋</button>
                <button class="panel-action-btn panel-maximize-btn" type="button" data-panel-maximize="controlPanel" title="最大化">⛶</button>
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:controlPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content control-panel-content">
              <div id="controlPanelTaskGrid" class="control-button-grid"></div>
              <div id="controlPanelCustomizeMenu" class="control-panel-customize-menu" hidden></div>
              <div id="controlPanelTrashBin" class="control-panel-trash-bin" hidden>拖到这里删除</div>
            </div>
          </section>

          <section id="imagePanel" class="floating-panel panel-image">
            <div class="panel-header">
              <div>
                <div class="panel-title">图像</div>
                <div class="panel-subtitle">切换本工程图像话题预览</div>
              </div>
              <div class="panel-header-field panel-image-topic-field">
                <select id="imageTopicSelect" class="ui-select" title="切换图像话题">
                  ${IMAGE_TOPIC_OPTIONS.map((option) => `
                    <option value="${option.id}" ${option.id === DEFAULT_IMAGE_TOPIC ? "selected" : ""}>${option.label}</option>
                  `).join("")}
                </select>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn panel-maximize-btn" type="button" data-panel-maximize="imagePanel" title="最大化">⛶</button>
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:imagePanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content image-panel-content">
              <div class="canvas-stage">
                <canvas id="irCanvas" width="640" height="480"></canvas>
                <canvas id="overlayCanvas" width="640" height="480"></canvas>
              </div>
            </div>
            <button
              class="image-panel-resize-handle"
              type="button"
              data-image-panel-resize="true"
              title="按图像比例缩放"
              aria-label="按图像比例缩放"
            ></button>
          </section>

          <section id="settingsPanel" class="floating-panel panel-settings">
            <div class="panel-header">
              <div>
                <div class="panel-title">设置</div>
                <div class="panel-subtitle">话题、工作区、图层与外参配置</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn panel-maximize-btn" type="button" data-panel-maximize="settingsPanel" title="最大化">⛶</button>
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:settingsPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content settings-panel-content">
              <div class="settings-page-switcher">
                <label class="field settings-page-field">
                  <span>当前页</span>
                  <select id="settingsPageSelect" class="ui-select">
                    ${SETTINGS_PAGE_OPTION_MARKUP}
                  </select>
                </label>
              </div>
              <div class="settings-page-stack">
                <section class="settings-page is-active" data-settings-page="topics">
                  <div class="settings-topic-overview">
                    <ul id="topicInventoryList" class="topic-inventory-list">
                      <li class="topic-inventory-item is-empty">暂无话题数据。</li>
                    </ul>
                  </div>
                </section>

                <section class="settings-page is-active" data-settings-page="workspace">
                  <div class="settings-grid">
                    <div class="settings-section">
                      <div class="section-title">工作区选点</div>
                      <div class="workspace-metadata">
                        <ol id="selectedPoints" class="point-list"></ol>
                        <div class="button-row">
                          <button class="secondary-btn" data-workspace-action="undo" disabled>撤销最后一点</button>
                          <button class="secondary-btn" data-workspace-action="clear" disabled>清空重选</button>
                        </div>
                      </div>
                    </div>

                    <div class="settings-section">
                      <div class="section-title">显示增强</div>
                      <div class="field-grid single-column">
                        <div class="field">
                          <label for="displayMode">模式</label>
                          <select id="displayMode" class="ui-select">
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
                  </div>
                </section>

                <section class="settings-page" data-settings-page="scene" hidden>
                  <div class="settings-grid">
                    <div class="settings-section">
                      <div class="section-title">场景视角</div>
                      <div class="field-grid compact-grid">
                        <div class="field">
                          <label for="sceneViewMode">视角</label>
                          <select id="sceneViewMode" class="ui-select">
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
                    </div>
                  </div>
                </section>

                <section class="settings-page" data-settings-page="layers" hidden>
                  <div class="settings-section settings-section-layers">
                    <div class="section-title">图层设置</div>
                    <div class="field-grid compact-grid">
                      <div class="field">
                        <label for="topicLayerMode">模式</label>
                          <select id="topicLayerMode" class="ui-select">
                          ${TOPIC_LAYER_MODES.map((mode) => `
                            <option value="${mode.id}">${mode.label}</option>
                          `).join("")}
                        </select>
                      </div>
                      <div class="field">
                        <label for="pointCloudSource">点云源</label>
                          <select id="pointCloudSource" class="ui-select">
                          ${POINT_CLOUD_SOURCES.map((source) => `
                            <option value="${source.id}">${source.label}</option>
                          `).join("")}
                        </select>
                      </div>
                    </div>

                    <div class="toggle-grid">
                      <label class="checkbox-field"><input id="showRobotToggle" type="checkbox" checked /><span>机器</span></label>
                      <label class="checkbox-field"><input id="showAxesToggle" type="checkbox" checked /><span>坐标轴</span></label>
                      <label class="checkbox-field"><input id="showPointCloudToggle" type="checkbox" /><span>点云</span></label>
                      <label class="checkbox-field"><input id="showTiePointsToggle" type="checkbox" checked /><span>绑扎点</span></label>
                      <label class="checkbox-field"><input id="showPlanningMarkersToggle" type="checkbox" /><span>规划点</span></label>
                    </div>

                    <div class="field-grid compact-grid settings-range-grid">
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

                <section class="settings-page" data-settings-page="calibration" hidden>
                  <div class="settings-grid">
                    <div class="settings-section">
                      <div class="section-title">相机-TCP外参</div>
                      <div id="gripperTfCurrent" class="info-block mono">等待 TF 链路后读取当前外参。</div>
                      <div class="field-grid compact-grid">
                        <div class="field">
                          <label for="gripperTfX">X (mm)</label>
                          <input id="gripperTfX" type="number" step="1" value="0" />
                        </div>
                        <div class="field">
                          <label for="gripperTfY">Y (mm)</label>
                          <input id="gripperTfY" type="number" step="1" value="0" />
                        </div>
                        <div class="field">
                          <label for="gripperTfZ">Z (mm)</label>
                          <input id="gripperTfZ" type="number" step="1" value="0" />
                        </div>
                      </div>
                      <div class="button-row">
                        <button id="applyGripperTfCalibration" class="primary-btn" type="button">应用外参</button>
                      </div>
                    </div>
                  </div>
                </section>

                <section class="settings-page" data-settings-page="cabinRemote" hidden>
                  <div class="settings-grid">
                    <div class="settings-section">
                      <div class="section-title">索驱遥控</div>
                      <label class="checkbox-field">
                        <input id="cabinKeyboardRemoteToggle" type="checkbox" />
                        <span>开启键盘遥控（全局生效）</span>
                      </label>
                      <div class="cabin-remote-pad">
                        <button class="secondary-btn cabin-remote-btn" type="button" data-cabin-remote-direction="zPositive" disabled>上</button>
                        <button class="secondary-btn cabin-remote-btn" type="button" data-cabin-remote-direction="xPositive" disabled>X+</button>
                        <button class="secondary-btn cabin-remote-btn" type="button" data-cabin-remote-direction="zNegative" disabled>下</button>
                        <button class="secondary-btn cabin-remote-btn" type="button" data-cabin-remote-direction="yPositive" disabled>Y+</button>
                        <button
                          id="cabinRemoteStopButton"
                          class="secondary-btn cabin-remote-btn cabin-remote-stop-btn"
                          type="button"
                          data-cabin-remote-stop="true"
                          disabled
                        >索驱停止移动</button>
                        <button class="secondary-btn cabin-remote-btn" type="button" data-cabin-remote-direction="yNegative" disabled>Y-</button>
                        <div class="cabin-remote-pad-spacer" data-cabin-remote-spacer="emptyLeft" aria-hidden="true"></div>
                        <button class="secondary-btn cabin-remote-btn" type="button" data-cabin-remote-direction="xNegative" disabled>X-</button>
                        <div class="cabin-remote-pad-spacer" data-cabin-remote-spacer="emptyRight" aria-hidden="true"></div>
                      </div>
                      <div id="cabinRemoteStatus" class="info-block mono">键位：Q/W/E = Z+/X+/Z-，A/S/D = Y+/X-/Y-。</div>
                      <div class="field-grid compact-grid">
                        <div class="field">
                          <label for="cabinRemoteStep">单次点击步距（mm）</label>
                          <input id="cabinRemoteStep" type="number" min="1" step="1" value="50" />
                        </div>
                        <div class="field">
                          <label for="cabinRemoteSpeed">全局移动速度</label>
                          <input id="cabinRemoteSpeed" type="number" min="1" step="1" value="300" />
                        </div>
                      </div>
                      <div class="section-title">当前索驱坐标</div>
                      <div id="cabinRemoteCurrentPosition" class="info-block cabin-remote-position-grid mono">
                        <div class="cabin-remote-position-item">
                          <span class="cabin-remote-position-label">X</span>
                          <span class="cabin-remote-position-value">等待索驱 TF…</span>
                        </div>
                        <div class="cabin-remote-position-item">
                          <span class="cabin-remote-position-label">Y</span>
                          <span class="cabin-remote-position-value">等待索驱 TF…</span>
                        </div>
                        <div class="cabin-remote-position-item">
                          <span class="cabin-remote-position-label">Z</span>
                          <span class="cabin-remote-position-value">等待索驱 TF…</span>
                        </div>
                      </div>
                    </div>
                  </div>
                </section>
              </div>
            </div>
          </section>

          <section id="terminalPanel" class="floating-panel panel-terminal">
            <div class="panel-header">
              <div>
                <div class="panel-title">终端</div>
                <div class="panel-subtitle">本机 Ubuntu Shell 与 SSH 会话</div>
              </div>
              <div class="panel-actions">
                <button id="createTerminalSession" class="panel-action-btn" type="button" title="新建终端">＋</button>
                <button class="panel-action-btn panel-maximize-btn" type="button" data-panel-maximize="terminalPanel" title="最大化">⛶</button>
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:terminalPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content terminal-panel-content">
              <div id="terminalTabStrip" class="terminal-tab-strip"></div>
              <div id="terminalViewport" class="terminal-viewport">
                <div id="terminalEmptyState" class="terminal-empty-state">
                  <div class="terminal-empty-title">还没有终端会话</div>
                  <div class="terminal-empty-subtitle">点击右上角加号，或从工具栏打开“终端”后自动创建本机 shell。</div>
                  <button id="createTerminalSessionEmpty" class="primary-btn" type="button">新建终端</button>
                </div>
              </div>
            </div>
          </section>

          <section id="logPanel" class="floating-panel panel-log">
            <div class="panel-header">
              <div>
              <div class="panel-title">日志</div>
                <div class="panel-subtitle">终端日志订阅</div>
              </div>
              <div class="panel-header-field panel-log-topic-field">
                <select id="logTopicSelect" class="ui-select" title="切换日志来源">
                  ${LOG_TOPIC_OPTIONS.map((option) => `
                    <option value="${option.id}" ${option.id === DEFAULT_LOG_TOPIC ? "selected" : ""}>${option.label}</option>
                  `).join("")}
                </select>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn panel-maximize-btn" type="button" data-panel-maximize="logPanel" title="最大化">⛶</button>
                <button id="clearLogs" class="panel-action-btn" type="button" title="清空日志">⌫</button>
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:logPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content">
              <ul id="logList" class="log-list"></ul>
            </div>
          </section>

        </div>

        <div class="quick-control-dock">
          <div id="quickControlGrid" class="quick-control-grid"></div>
        </div>
      </div>
    `;
    this.bindRefs();
    this.applyDefaultPanelVisibility();
    this.renderStatusChips();
    this.renderControlPanelTasks();
    this.renderControlPanelCustomizeMenu();
    this.renderLegacyCommands();
    this.renderPointList([]);
    this.renderTerminalSessions([], null);
    this.renderLogs([]);
    this.renderTopicLayerStats({
      filteredWorldCoordCount: 0,
      rawWorldCoordCount: 0,
      tiePointCount: 0,
      planningPointCount: 0,
      tfFrameCount: 0,
    });
    this.renderTopicInventory([]);
    this.setGripperTfCalibration(null);
    this.setSettingsHomePage("topics");
    this.setSettingsPage("topics");
    this.setSelectedImageTopic(DEFAULT_IMAGE_TOPIC);
    this.setSelectedLogTopic(DEFAULT_LOG_TOPIC);
  }

  bindRefs() {
    this.refs.sceneBackground = this.rootElement.querySelector("#sceneBackground");
    this.refs.irCanvas = this.rootElement.querySelector("#irCanvas");
    this.refs.overlayCanvas = this.rootElement.querySelector("#overlayCanvas");
    this.refs.sceneViewMode = this.rootElement.querySelector("#sceneViewMode");
    this.refs.followCameraToggle = this.rootElement.querySelector("#followCameraToggle");
    this.refs.connectionBadge = this.rootElement.querySelector("#connectionBadge");
    this.refs.voltageBadge = this.rootElement.querySelector("#voltageBadge");
    this.refs.themeToggle = this.rootElement.querySelector("#themeToggle");
    this.refs.imageTopicSelect = this.rootElement.querySelector("#imageTopicSelect");
    this.refs.logTopicSelect = this.rootElement.querySelector("#logTopicSelect");
    this.refs.terminalTabStrip = this.rootElement.querySelector("#terminalTabStrip");
    this.refs.terminalViewport = this.rootElement.querySelector("#terminalViewport");
    this.refs.terminalEmptyState = this.rootElement.querySelector("#terminalEmptyState");
    this.refs.createTerminalSession = this.rootElement.querySelector("#createTerminalSession");
    this.refs.createTerminalSessionEmpty = this.rootElement.querySelector("#createTerminalSessionEmpty");
    this.refs.controlPanelTaskGrid = this.rootElement.querySelector("#controlPanelTaskGrid");
    this.refs.quickControlGrid = this.rootElement.querySelector("#quickControlGrid");
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
    this.refs.topicInventorySummary = this.rootElement.querySelector("#topicInventorySummary");
    this.refs.topicInventoryList = this.rootElement.querySelector("#topicInventoryList");
    this.refs.selectedPoints = this.rootElement.querySelector("#selectedPoints");
    this.refs.displayMode = this.rootElement.querySelector("#displayMode");
    this.refs.gammaRange = this.rootElement.querySelector("#gammaRange");
    this.refs.overlayOpacityRange = this.rootElement.querySelector("#overlayOpacityRange");
    this.refs.displaySettingSummary = this.rootElement.querySelector("#displaySettingSummary");
    this.refs.statusCapsuleGrid = this.rootElement.querySelector("#statusCapsuleGrid");
    this.refs.logList = this.rootElement.querySelector("#logList");
    this.refs.clearLogs = this.rootElement.querySelector("#clearLogs");
    this.refs.settingsPageSelect = this.rootElement.querySelector("#settingsPageSelect");
    this.refs.settingsPages = [...this.rootElement.querySelectorAll("[data-settings-page]")];
    this.refs.gripperTfCurrent = this.rootElement.querySelector("#gripperTfCurrent");
    this.refs.gripperTfX = this.rootElement.querySelector("#gripperTfX");
    this.refs.gripperTfY = this.rootElement.querySelector("#gripperTfY");
    this.refs.gripperTfZ = this.rootElement.querySelector("#gripperTfZ");
    this.refs.applyGripperTfCalibration = this.rootElement.querySelector("#applyGripperTfCalibration");
    this.refs.cabinKeyboardRemoteToggle = this.rootElement.querySelector("#cabinKeyboardRemoteToggle");
    this.refs.cabinRemoteStep = this.rootElement.querySelector("#cabinRemoteStep");
    this.refs.cabinRemoteSpeed = this.rootElement.querySelector("#cabinRemoteSpeed");
    this.refs.cabinRemoteCurrentPosition = this.rootElement.querySelector("#cabinRemoteCurrentPosition");
    this.refs.cabinRemoteStatus = this.rootElement.querySelector("#cabinRemoteStatus");
    this.refs.cabinRemoteButtons = [...this.rootElement.querySelectorAll("[data-cabin-remote-direction]")];
    this.refs.cabinRemoteStopButton = this.rootElement.querySelector("#cabinRemoteStopButton");
    this.refs.controlPanelCustomizeButton = this.rootElement.querySelector("#controlPanelCustomizeButton");
    this.refs.controlPanelCustomizeMenu = this.rootElement.querySelector("#controlPanelCustomizeMenu");
    this.refs.controlPanelTrashBin = this.rootElement.querySelector("#controlPanelTrashBin");
    this.refs.taskButtons = [...this.rootElement.querySelectorAll("[data-task-action]")];
    this.refs.customControlPanelButtons = [...this.rootElement.querySelectorAll("[data-custom-service-action]")];
    this.refs.workspaceButtons = [...this.rootElement.querySelectorAll("[data-workspace-action]")];
    this.refs.toolbarButtons = [...this.rootElement.querySelectorAll("[data-toolbar-action]")];
    this.refs.panelElements = new Map(
      this.getPanelToggleButtons().map(({ id }) => [id, this.rootElement.querySelector(`#${id}`)]),
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
    this.enhanceSelectControls();
    this.refs.settingsPageSelect?.addEventListener("change", () => this.setSettingsPage(this.refs.settingsPageSelect.value));
  }

  enhanceSelectControls() {
    this.customSelects.forEach(({ shell }) => shell.remove());
    this.customSelects.clear();

    const selects = [...this.rootElement.querySelectorAll("select.ui-select")];
    selects.forEach((select) => {
      if (!select.id) {
        return;
      }
      select.classList.add("native-select");

      const shell = document.createElement("div");
      shell.className = "ui-select-shell";
      shell.dataset.selectShellFor = select.id;

      const trigger = document.createElement("button");
      trigger.type = "button";
      trigger.className = "ui-select-trigger";
      trigger.setAttribute("aria-haspopup", "listbox");
      trigger.setAttribute("aria-expanded", "false");

      const label = document.createElement("span");
      label.className = "ui-select-trigger-label";
      trigger.appendChild(label);

      const chevron = document.createElement("span");
      chevron.className = "ui-select-chevron";
      chevron.setAttribute("aria-hidden", "true");
      trigger.appendChild(chevron);

      const menu = document.createElement("div");
      menu.className = "ui-select-menu";
      menu.hidden = true;
      menu.setAttribute("role", "listbox");

      [...select.options].forEach((option) => {
        const optionButton = document.createElement("button");
        optionButton.type = "button";
        optionButton.className = "ui-select-option";
        optionButton.dataset.value = option.value;
        optionButton.setAttribute("role", "option");
        const optionLabel = document.createElement("span");
        optionLabel.className = select.id === "settingsPageSelect"
          ? "settings-page-option-label"
          : "ui-select-option-label";
        optionLabel.textContent = option.textContent;
        optionButton.appendChild(optionLabel);
        optionButton.addEventListener("click", (event) => {
          event.preventDefault();
          event.stopPropagation();
          if (!option.disabled) {
            select.value = option.value;
            select.dispatchEvent(new Event("input", { bubbles: true }));
            select.dispatchEvent(new Event("change", { bubbles: true }));
            this.refreshCustomSelect(select);
          }
          this.closeCustomSelect(select.id);
        });
        if (select.id !== "settingsPageSelect") {
          menu.appendChild(optionButton);
          return;
        }

        const optionRow = document.createElement("div");
        optionRow.className = "settings-page-option-row";

        const homeButton = document.createElement("button");
        homeButton.type = "button";
        homeButton.className = "settings-home-page-button";
        homeButton.dataset.settingsHomePage = option.value;
        homeButton.setAttribute("aria-pressed", "false");
        homeButton.textContent = "⌂";
        homeButton.title = `设为主页：${option.textContent}`;
        homeButton.setAttribute("aria-label", homeButton.title);
        homeButton.addEventListener("click", (event) => {
          event.preventDefault();
          event.stopPropagation();
          if (option.disabled) {
            return;
          }
          const previousHomePageId = this.settingsHomePageId;
          const normalizedPageId = this.setSettingsHomePage(option.value);
          if (previousHomePageId !== normalizedPageId) {
            this.handleSettingsHomePageChange?.(normalizedPageId);
          }
        });

        optionRow.appendChild(optionButton);
        optionRow.appendChild(homeButton);
        menu.appendChild(optionRow);
      });

      trigger.addEventListener("click", (event) => {
        event.preventDefault();
        event.stopPropagation();
        if (select.disabled) {
          return;
        }
        const isOpen = shell.dataset.open === "true";
        if (isOpen) {
          this.closeCustomSelect(select.id);
          return;
        }
        this.openCustomSelect(select.id);
      });

      shell.appendChild(trigger);
      shell.appendChild(menu);
      select.insertAdjacentElement("afterend", shell);

      this.customSelects.set(select.id, {
        select,
        shell,
        trigger,
        label,
        menu,
      });
      this.refreshCustomSelect(select);
    });

    document.removeEventListener("pointerdown", this.handleDocumentPointerDown, true);
    document.removeEventListener("keydown", this.handleDocumentKeyDown, true);
    document.addEventListener("pointerdown", this.handleDocumentPointerDown, true);
    document.addEventListener("keydown", this.handleDocumentKeyDown, true);
  }

  handleDocumentPointerDown(event) {
    if (this.customControlPanelEditMode) {
      if (event.target.closest("[data-custom-service-action]") || event.target.closest("#controlPanelTrashBin")) {
        return;
      }
      this.setCustomControlPanelEditMode(false);
    }
    if (event.target.closest(".ui-select-shell")) {
      return;
    }
    if (event.target.closest("#controlPanelCustomizeButton") || event.target.closest("#controlPanelCustomizeMenu")) {
      return;
    }
    this.closeAllCustomSelects();
    this.toggleControlPanelCustomizeMenu(false);
  }

  handleDocumentKeyDown(event) {
    if (event.key === "Escape") {
      this.finishCustomControlPanelDrag({ cancel: true });
      this.setCustomControlPanelEditMode(false);
      this.closeAllCustomSelects();
      this.toggleControlPanelCustomizeMenu(false);
    }
  }

  refreshCustomSelect(select) {
    const entry = this.customSelects.get(select.id);
    if (!entry) {
      return;
    }
    const activeOption = select.options[select.selectedIndex] || select.options[0];
    entry.label.textContent = activeOption?.textContent || "";
    entry.trigger.title = select.title || activeOption?.textContent || "";
    entry.trigger.disabled = select.disabled;
    entry.shell.dataset.disabled = select.disabled ? "true" : "false";
    [...entry.menu.querySelectorAll(".ui-select-option")].forEach((optionButton) => {
      const selected = optionButton.dataset.value === select.value;
      optionButton.classList.toggle("is-selected", selected);
      optionButton.setAttribute("aria-selected", selected ? "true" : "false");
    });
    [...entry.menu.querySelectorAll(".settings-home-page-button")].forEach((homeButton) => {
      const optionValue = homeButton.dataset.settingsHomePage || "";
      const optionLabel = [...select.options].find((item) => item.value === optionValue)?.textContent || optionValue;
      const active = optionValue === this.settingsHomePageId;
      homeButton.classList.toggle("is-active", active);
      homeButton.setAttribute("aria-pressed", active ? "true" : "false");
      homeButton.title = active ? `当前主页：${optionLabel}` : `设为主页：${optionLabel}`;
      homeButton.setAttribute("aria-label", homeButton.title);
    });
  }

  openCustomSelect(selectId) {
    this.closeAllCustomSelects();
    const entry = this.customSelects.get(selectId);
    if (!entry) {
      return;
    }
    entry.shell.dataset.open = "true";
    entry.menu.hidden = false;
    entry.trigger.setAttribute("aria-expanded", "true");
  }

  closeCustomSelect(selectId) {
    const entry = this.customSelects.get(selectId);
    if (!entry) {
      return;
    }
    entry.shell.dataset.open = "false";
    entry.menu.hidden = true;
    entry.trigger.setAttribute("aria-expanded", "false");
  }

  closeAllCustomSelects() {
    this.customSelects.forEach((_entry, selectId) => {
      this.closeCustomSelect(selectId);
    });
  }

  applyDefaultPanelVisibility() {
    Object.entries(this.getInitialPanelVisibility()).forEach(([panelId, visible]) => {
      this.setPanelVisible(panelId, visible);
    });
  }

  renderPanelsFromLayout(layout) {
    this.initialLayout = layout;
    this.applyDefaultPanelVisibility();
  }

  setSettingsPage(pageId) {
    this.closeAllCustomSelects();
    const normalizedPageId = this.normalizeSettingsPageId(pageId);
    if (this.refs.settingsPageSelect) {
      this.refs.settingsPageSelect.value = normalizedPageId;
      this.refreshCustomSelect(this.refs.settingsPageSelect);
    }
    this.refs.settingsPages?.forEach((page) => {
      const active = page.dataset.settingsPage === normalizedPageId;
      page.classList.toggle("is-active", active);
      page.hidden = !active;
    });
    this.handleSettingsPageChange?.(normalizedPageId);
  }

  normalizeSettingsPageId(pageId) {
    return SETTINGS_PAGE_OPTIONS.some((option) => option.id === pageId) ? pageId : "topics";
  }

  setSettingsHomePage(pageId) {
    const normalizedPageId = this.normalizeSettingsPageId(pageId);
    this.settingsHomePageId = normalizedPageId;
    if (this.refs.settingsPageSelect) {
      this.refreshCustomSelect(this.refs.settingsPageSelect);
    }
    return normalizedPageId;
  }

  renderStatusChips() {
    this.refs.statusCapsuleGrid.innerHTML = STATUS_MONITORS
      .filter((item) => item.id !== "ros")
      .map((item) => `
      <button
        class="system-status-item info is-interactive"
        type="button"
        data-status-id="${item.id}"
        data-status-action=""
        title="${item.label}"
      >
          <span class="system-status-light"></span>
          <span class="system-status-text">
            <span class="system-status-label">${item.label}</span>
            <span class="system-status-action-label">启动</span>
          </span>
      </button>
    `).join("");
  }

  renderControlPanelTasks(taskIds = null, customButtons = []) {
    const visibleTaskIds = normalizeControlPanelVisibleTaskIds(taskIds);
    const visibleTasks = CONTROL_PANEL_TASKS.filter((task) => visibleTaskIds.includes(task.id));
    const normalizedCustomButtons = normalizeCustomControlPanelButtons(customButtons);
    if (!visibleTasks.length && !normalizedCustomButtons.length) {
      this.refs.controlPanelTaskGrid.innerHTML = `
        <div class="control-panel-empty-state">还没有固定按钮，点击右上角加号添加。</div>
      `;
      this.refs.taskButtons = [];
      this.refs.customControlPanelButtons = [];
      this.setCustomControlPanelEditMode(false);
      return;
    }
    this.refs.controlPanelTaskGrid.innerHTML = [
      ...visibleTasks.map((task) => `
      <button
        class="control-action-btn"
        type="button"
        data-task-action="${task.id}"
        data-tone="${task.tone}"
        disabled
      >${task.label.replaceAll("\n", "<br />")}</button>
    `),
      ...normalizedCustomButtons.map((button) => `
      <button
        class="control-action-btn control-action-btn-custom ${this.customControlPanelEditMode ? "is-editing" : ""}"
        type="button"
        data-custom-service-action="${button.id}"
        data-tone="${button.tone}"
        data-enabled="false"
        aria-disabled="true"
      >
        <span class="control-action-btn-main">${escapeHtml(button.label)}</span>
        <span class="control-action-btn-caption mono">${escapeHtml(button.servicePath)}</span>
      </button>
    `),
    ].join("");
    this.refs.taskButtons = [...this.refs.controlPanelTaskGrid.querySelectorAll("[data-task-action]")];
    this.refs.customControlPanelButtons = [...this.refs.controlPanelTaskGrid.querySelectorAll("[data-custom-service-action]")];
    if (!this.refs.customControlPanelButtons.length) {
      this.setCustomControlPanelEditMode(false);
    } else if (this.customControlPanelEditMode) {
      this.setCustomControlPanelEditMode(true);
    }
  }

  renderControlPanelCustomizeMenu(customButtons = []) {
    if (!this.refs.controlPanelCustomizeMenu) {
      return;
    }
    const normalizedCustomButtons = normalizeCustomControlPanelButtons(customButtons);
    this.refs.controlPanelCustomizeMenu.innerHTML = `
      <div class="control-panel-customize-title">新增自定义按钮</div>
      <form id="controlPanelCustomButtonForm" class="control-panel-custom-form">
        <label class="field">
          <span>按钮名称</span>
          <input id="controlPanelCustomButtonName" type="text" maxlength="24" placeholder="例如：启动索驱" />
        </label>
        <label class="field">
          <span>服务地址</span>
          <input id="controlPanelCustomButtonService" type="text" placeholder="/your/service/path" />
        </label>
        <div class="control-panel-customize-hint">当前仅支持 std_srvs/Trigger 空请求服务。</div>
        <button id="controlPanelCustomButtonSubmit" class="primary-btn" type="submit">添加按钮</button>
      </form>
      <div class="control-panel-customize-list">
        ${normalizedCustomButtons.length
          ? normalizedCustomButtons.map((button) => `
            <div class="control-panel-customize-option">
              <strong>${escapeHtml(button.label)}</strong>
              <span class="mono">${escapeHtml(button.servicePath)}</span>
            </div>
          `).join("")
          : '<div class="control-panel-customize-option">还没有自定义按钮。</div>'}
      </div>
    `;
    this.refs.controlPanelCustomButtonForm = this.rootElement.querySelector("#controlPanelCustomButtonForm");
    this.refs.controlPanelCustomButtonName = this.rootElement.querySelector("#controlPanelCustomButtonName");
    this.refs.controlPanelCustomButtonService = this.rootElement.querySelector("#controlPanelCustomButtonService");
  }

  renderLegacyCommands() {
    this.refs.quickControlGrid.innerHTML = getControlPanelGroups()
      .flatMap(({ controls }) => controls)
      .map((control) => `
        <button
          class="quick-control-btn ${control.active ? "is-active" : ""}"
          data-control-toggle="${control.id}"
          data-tone="${control.tone}"
          data-active="${control.active ? "true" : "false"}"
          aria-pressed="${control.active ? "true" : "false"}"
          type="button"
        >
          <span class="quick-control-indicator">
            <span class="quick-control-light ${control.active ? "is-active" : ""}"></span>
            <span class="quick-control-label">${control.label}</span>
          </span>
        </button>
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

  getTerminalViewport() {
    return this.refs.terminalViewport;
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

  getCabinRemoteSettings() {
    return {
      keyboardEnabled: Boolean(this.refs.cabinKeyboardRemoteToggle?.checked),
      step: Number.parseFloat(this.refs.cabinRemoteStep?.value || "50"),
      speed: Number.parseFloat(this.refs.cabinRemoteSpeed?.value || "300"),
    };
  }

  getControlPanelCustomButtonDraft() {
    return {
      label: String(this.refs.controlPanelCustomButtonName?.value || "").trim(),
      servicePath: String(this.refs.controlPanelCustomButtonService?.value || "").trim(),
    };
  }

  resetControlPanelCustomButtonForm() {
    this.refs.controlPanelCustomButtonForm?.reset();
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

  onStatusChipAction(callback) {
    [...this.rootElement.querySelectorAll("[data-status-id][data-status-action]")].forEach((button) => {
      button.addEventListener("click", (event) => {
        event.preventDefault();
        event.stopPropagation();
        if (!button.dataset.statusAction) {
          return;
        }
        callback(button.dataset.statusId, button.dataset.statusAction);
      });
    });
  }

  onConnectionAction(callback) {
    this.refs.connectionBadge?.addEventListener("click", (event) => {
      event.preventDefault();
      event.stopPropagation();
      if (!this.refs.connectionBadge.dataset.connectionAction) {
        return;
      }
      callback(this.refs.connectionBadge.dataset.connectionAction);
    });
  }

  onTaskAction(callback) {
    this.refs.controlPanelTaskGrid?.addEventListener("click", (event) => {
      const button = event.target.closest("[data-task-action]");
      if (!button) {
        return;
      }
      this.flashTaskButton(button.dataset.taskAction);
      callback(button.dataset.taskAction);
    });
  }

  onImageTopicChange(callback) {
    this.refs.imageTopicSelect?.addEventListener("change", () => callback(this.refs.imageTopicSelect.value));
  }

  onLogTopicChange(callback) {
    this.refs.logTopicSelect?.addEventListener("change", () => callback(this.refs.logTopicSelect.value));
  }

  onSettingsPageChange(callback) {
    this.handleSettingsPageChange = callback;
  }

  onSettingsHomePageChange(callback) {
    this.handleSettingsHomePageChange = callback;
  }

  onControlPanelVisibleTasksChange(callback) {
    this.handleControlPanelVisibleTasksChange = callback;
  }

  onControlPanelCustomButtonCreate(callback) {
    this.refs.controlPanelCustomizeButton?.addEventListener("click", (event) => {
      event.preventDefault();
      event.stopPropagation();
      const nextOpen = this.toggleControlPanelCustomizeMenu();
      if (nextOpen) {
        window.setTimeout(() => {
          this.refs.controlPanelCustomButtonName?.focus();
        }, 0);
      }
    });
    this.refs.controlPanelCustomizeMenu?.addEventListener("submit", (event) => {
      if (!(event.target instanceof HTMLFormElement) || event.target.id !== "controlPanelCustomButtonForm") {
        return;
      }
      event.preventDefault();
      const draft = this.getControlPanelCustomButtonDraft();
      if (!draft.label || !draft.servicePath) {
        return;
      }
      callback(draft);
      this.resetControlPanelCustomButtonForm();
      this.toggleControlPanelCustomizeMenu(false);
    });
  }

  onCustomControlPanelAction(callback) {
    this.refs.controlPanelTaskGrid?.addEventListener("click", (event) => {
      const button = event.target.closest("[data-custom-service-action]");
      if (!button || this.customControlPanelEditMode || button.dataset.enabled === "false") {
        return;
      }
      const buttonId = button.dataset.customServiceAction;
      this.flashCustomControlPanelButton(buttonId);
      callback(buttonId);
    });
  }

  onCustomControlPanelDelete(callback) {
    this.handleCustomControlPanelDelete = callback;
    this.refs.controlPanelTaskGrid?.addEventListener("pointerdown", (event) => {
      const button = event.target.closest("[data-custom-service-action]");
      if (!button || event.button !== 0) {
        return;
      }
      window.addEventListener("pointermove", this.handleCustomControlPanelPointerMove, true);
      window.addEventListener("pointerup", this.handleCustomControlPanelPointerUp, true);
      window.addEventListener("pointercancel", this.handleCustomControlPanelPointerUp, true);
      if (this.customControlPanelEditMode) {
        this.beginCustomControlPanelDrag(button, {
          pointerId: event.pointerId,
          clientX: event.clientX,
          clientY: event.clientY,
        });
        return;
      }
      this.startCustomControlPanelLongPress(button, event);
    });
  }

  onWorkspaceAction(callback) {
    this.refs.workspaceButtons.forEach((button) => {
      button.addEventListener("click", () => callback(button.dataset.workspaceAction));
    });
  }

  onCabinRemoteAction(callback) {
    this.refs.cabinRemoteButtons.forEach((button) => {
      button.addEventListener("pointerdown", (event) => {
        if (event.button !== 0 || button.disabled) {
          return;
        }
        event.preventDefault();
        callback(button.dataset.cabinRemoteDirection, { type: "pressstart" });
      });
      ["pointerup", "pointerleave", "pointercancel"].forEach((eventName) => {
        button.addEventListener(eventName, () => callback(button.dataset.cabinRemoteDirection, { type: "pressend" }));
      });
      button.addEventListener("keydown", (event) => {
        if (button.disabled || event.repeat) {
          return;
        }
        if (event.key !== "Enter" && event.key !== " ") {
          return;
        }
        event.preventDefault();
        callback(button.dataset.cabinRemoteDirection, { type: "click" });
      });
    });
    this.refs.cabinRemoteStopButton?.addEventListener("click", () => {
      if (this.refs.cabinRemoteStopButton.disabled) {
        return;
      }
      callback("stopMotion", { type: "stop" });
    });
  }

  onCabinRemoteSettingsChange(callback) {
    [this.refs.cabinKeyboardRemoteToggle, this.refs.cabinRemoteStep, this.refs.cabinRemoteSpeed]
      .filter(Boolean)
      .forEach((element) => {
        const eventName = element.type === "checkbox" ? "change" : "input";
        element.addEventListener(eventName, () => callback(this.getCabinRemoteSettings()));
        if (eventName !== "change") {
          element.addEventListener("change", () => callback(this.getCabinRemoteSettings()));
        }
      });
  }

  onCalibrationApply(callback) {
    this.refs.applyGripperTfCalibration?.addEventListener("click", () => callback(this.getGripperTfCalibrationInputs()));
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

  onTerminalAction(callback) {
    this.refs.createTerminalSession?.addEventListener("click", () => callback("new"));
    this.refs.createTerminalSessionEmpty?.addEventListener("click", () => callback("new"));
    this.refs.terminalTabStrip?.addEventListener("click", (event) => {
      const closeButton = event.target.closest("[data-terminal-close]");
      if (closeButton) {
        callback("close", closeButton.dataset.terminalClose);
        return;
      }
      const tabButton = event.target.closest("[data-terminal-select]");
      if (tabButton) {
        callback("activate", tabButton.dataset.terminalSelect);
      }
    });
  }

  toggleControlPanelCustomizeMenu(forceOpen = null) {
    if (!this.refs.controlPanelCustomizeMenu || !this.refs.controlPanelCustomizeButton) {
      return false;
    }
    const nextOpen = typeof forceOpen === "boolean"
      ? forceOpen
      : this.refs.controlPanelCustomizeMenu.hidden;
    if (nextOpen) {
      this.closeAllCustomSelects();
    }
    this.refs.controlPanelCustomizeMenu.hidden = !nextOpen;
    this.refs.controlPanelCustomizeButton.setAttribute("aria-expanded", nextOpen ? "true" : "false");
    return nextOpen;
  }

  setCustomControlPanelButtonsEnabled(enabled) {
    this.refs.customControlPanelButtons.forEach((button) => {
      button.dataset.enabled = enabled ? "true" : "false";
      button.setAttribute("aria-disabled", enabled ? "false" : "true");
      button.classList.toggle("is-disabled", !enabled);
    });
  }

  setCustomControlPanelButtonPending(buttonId, pending) {
    const button = this.refs.customControlPanelButtons.find((item) => item.dataset.customServiceAction === buttonId);
    if (!button) {
      return;
    }
    button.classList.toggle("is-pending", pending);
    button.dataset.pending = pending ? "true" : "false";
  }

  setConnectionInfo(url, message, level = "info") {
    const normalizedLevel = CONNECTION_LABELS[level] ? level : "info";
    const action = CONNECTION_ACTIONS[normalizedLevel] || CONNECTION_ACTIONS.info;
    const hasAction = Boolean(action.id);
    const labelNode = this.refs.connectionBadge.querySelector(".toolbar-connection-label");
    const actionNode = this.refs.connectionBadge.querySelector(".toolbar-connection-action-label");
    if (labelNode) {
      labelNode.textContent = CONNECTION_LABELS[normalizedLevel];
    }
    if (actionNode) {
      actionNode.textContent = action.label;
    }
    this.refs.connectionBadge.className = `toolbar-connection-badge ${normalizedLevel}`;
    this.refs.connectionBadge.title = message || url || CONNECTION_LABELS[normalizedLevel];
    this.refs.connectionBadge.dataset.connectionAction = action.id;
    this.refs.connectionBadge.dataset.hasAction = hasAction ? "true" : "false";
    this.refs.connectionBadge.setAttribute(
      "aria-label",
      hasAction ? `${CONNECTION_LABELS[normalizedLevel]}，点击${action.label}` : CONNECTION_LABELS[normalizedLevel],
    );
    this.setStatusChipState("ros", normalizedLevel === "success" ? "success" : normalizedLevel === "info" ? "info" : "warn", message || url || CONNECTION_LABELS[normalizedLevel]);
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
      if (button.disabled) {
        this.clearTaskButtonFeedback(key);
      }
    });
  }

  setWorkspaceButtonsEnabled(enabledMap) {
    this.refs.workspaceButtons.forEach((button) => {
      const key = button.dataset.workspaceAction;
      button.disabled = enabledMap[key] === false;
    });
  }

  setCabinRemoteButtonsEnabled(enabled) {
    this.refs.cabinRemoteButtons.forEach((button) => {
      button.disabled = !enabled;
    });
    if (this.refs.cabinRemoteStopButton) {
      this.refs.cabinRemoteStopButton.disabled = !enabled;
    }
    if (!enabled) {
      this.clearCabinRemoteButtonActive();
    }
  }

  setDisplaySettings(settings) {
    this.refs.displayMode.value = settings.mode;
    this.refs.gammaRange.value = settings.gamma.toFixed(2);
    this.refs.overlayOpacityRange.value = settings.overlayOpacity.toFixed(2);
    this.refreshCustomSelect(this.refs.displayMode);
    this.refs.displaySettingSummary.textContent =
      `模式=${DISPLAY_MODE_LABELS[settings.mode] || settings.mode} 伽马=${settings.gamma.toFixed(2)} 覆盖=${settings.overlayOpacity.toFixed(2)}`;
  }

  getSelectedImageTopic() {
    return this.refs.imageTopicSelect?.value || DEFAULT_IMAGE_TOPIC;
  }

  setSelectedImageTopic(topicId) {
    if (!this.refs.imageTopicSelect) {
      return;
    }
    this.refs.imageTopicSelect.value = topicId || DEFAULT_IMAGE_TOPIC;
    this.refs.imageTopicSelect.title = `当前图像话题：${getImageTopicLabel(this.refs.imageTopicSelect.value)}`;
    this.refreshCustomSelect(this.refs.imageTopicSelect);
  }

  getSelectedLogTopic() {
    return this.refs.logTopicSelect?.value || DEFAULT_LOG_TOPIC;
  }

  setSelectedLogTopic(topicId) {
    if (!this.refs.logTopicSelect) {
      return;
    }
    this.refs.logTopicSelect.value = topicId || DEFAULT_LOG_TOPIC;
    this.refs.logTopicSelect.title = `当前日志来源：${getLogTopicLabel(this.refs.logTopicSelect.value)}`;
    this.refreshCustomSelect(this.refs.logTopicSelect);
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
    this.refreshCustomSelect(this.refs.topicLayerMode);
    this.refreshCustomSelect(this.refs.pointCloudSource);
    this.refreshCustomSelect(this.refs.sceneViewMode);
    this.refs.topicLayerSummary.textContent =
      `模式=${getTopicLayerModeLabel(state.mode)} 点云=${state.showPointCloud ? "开启" : "关闭"} 点云源=${getPointCloudSourceLabel(state.pointCloudSource)} 点大小=${Number(state.pointSize).toFixed(3)} 透明度=${Number(state.pointOpacity).toFixed(2)}`;
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

  renderTopicInventory(topics) {
    const entries = Array.isArray(topics) ? topics : [];
    if (!entries.length) {
      this.refs.topicInventoryList.innerHTML = '<li class="topic-inventory-item is-empty">暂无话题数据。</li>';
      return;
    }

    const groupedEntries = entries.reduce((accumulator, entry) => {
      const topicName = String(entry?.name || "");
      const normalizedSegments = topicName.split("/").filter(Boolean);
      const rootNamespace = normalizedSegments[0] ? `/${normalizedSegments[0]}` : "(根级话题)";
      if (!accumulator.has(rootNamespace)) {
        accumulator.set(rootNamespace, []);
      }
      accumulator.get(rootNamespace).push(entry);
      return accumulator;
    }, new Map());

    const orderedGroups = Array.from(groupedEntries.entries()).sort(([leftName], [rightName]) => {
      if (leftName === "(根级话题)") {
        return -1;
      }
      if (rightName === "(根级话题)") {
        return 1;
      }
      return leftName.localeCompare(rightName, "zh-CN");
    });

    this.refs.topicInventoryList.innerHTML = orderedGroups.map(([groupName, groupEntries]) => `
      ${(() => {
        const isExpanded = this.topicInventoryExpanded.has(groupName)
          ? this.topicInventoryExpanded.get(groupName)
          : true;
        const groupKey = encodeURIComponent(groupName);
        return `
      <li class="topic-inventory-group ${isExpanded ? "is-expanded" : "is-collapsed"}" data-topic-group="${groupKey}" data-expanded="${isExpanded ? "true" : "false"}">
        <button
          class="topic-inventory-group-header"
          type="button"
          data-topic-group-toggle="${groupKey}"
          aria-expanded="${isExpanded ? "true" : "false"}"
        >
          <span class="topic-inventory-group-main">
            <span class="topic-inventory-group-name mono">${groupName}</span>
            <span class="topic-inventory-group-count">${groupEntries.length} 条</span>
          </span>
          <span class="topic-inventory-group-chevron" aria-hidden="true">⌄</span>
        </button>
        <ul class="topic-inventory-group-list" ${isExpanded ? "" : "hidden"}>
          ${groupEntries.map((entry) => `
            <li class="topic-inventory-item">
              <div class="topic-inventory-name mono">${entry.name}</div>
              <div class="topic-inventory-type mono">${entry.type}</div>
            </li>
          `).join("")}
        </ul>
      </li>`;
      })()}
    `).join("");
    this.bindTopicInventoryGroupToggles();
  }

  bindTopicInventoryGroupToggles() {
    this.refs.topicInventoryList
      .querySelectorAll("[data-topic-group-toggle]")
      .forEach((button) => {
        button.addEventListener("click", () => {
          const encodedGroupName = button.dataset.topicGroupToggle || "";
          const groupName = decodeURIComponent(encodedGroupName);
          const groupElement = button.closest(".topic-inventory-group");
          const listElement = groupElement?.querySelector(".topic-inventory-group-list");
          const currentExpanded = groupElement?.dataset.expanded !== "false";
          const nextExpanded = !currentExpanded;

          this.topicInventoryExpanded.set(groupName, nextExpanded);
          if (groupElement) {
            groupElement.dataset.expanded = nextExpanded ? "true" : "false";
            groupElement.classList.toggle("is-expanded", nextExpanded);
            groupElement.classList.toggle("is-collapsed", !nextExpanded);
          }
          button.setAttribute("aria-expanded", nextExpanded ? "true" : "false");
          if (listElement) {
            listElement.hidden = !nextExpanded;
          }
        });
      });
  }

  getGripperTfCalibrationInputs() {
    return {
      x: Number.parseFloat(this.refs.gripperTfX?.value || "0"),
      y: Number.parseFloat(this.refs.gripperTfY?.value || "0"),
      z: Number.parseFloat(this.refs.gripperTfZ?.value || "0"),
    };
  }

  setGripperTfCalibration(calibration, { forceInputs = false } = {}) {
    if (!calibration) {
      this.refs.gripperTfCurrent.textContent = "等待 TF 链路后读取当前外参。";
      return;
    }

    this.refs.gripperTfCurrent.textContent =
      `${calibration.parentFrame} -> ${calibration.childFrame} | translation_mm=(${calibration.translationMm.x.toFixed(1)}, ${calibration.translationMm.y.toFixed(1)}, ${calibration.translationMm.z.toFixed(1)})`;
    const activeElement = document.activeElement;
    const isEditing = activeElement === this.refs.gripperTfX
      || activeElement === this.refs.gripperTfY
      || activeElement === this.refs.gripperTfZ;
    if (forceInputs || !isEditing) {
      this.refs.gripperTfX.value = calibration.translationMm.x.toFixed(1);
      this.refs.gripperTfY.value = calibration.translationMm.y.toFixed(1);
      this.refs.gripperTfZ.value = calibration.translationMm.z.toFixed(1);
    }
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
    chip.className = `system-status-item ${level} is-interactive`;
    chip.title = detail || "";
    const actionLabel = chip.querySelector(".system-status-action-label");
    const nextActionMap = {
      ros: level === "success" ? "restartRosStack" : "startRosStack",
      chassis: level === "success" ? "restartCabinDriver" : "startCabinDriver",
      moduan: level === "success" ? "restartModuanDriver" : "startModuanDriver",
      visual: level === "success" ? "restartAlgorithmStack" : "startAlgorithmStack",
    };
    const nextActionLabelMap = {
      ros: level === "success" ? "重启ROS" : "启动ROS",
      chassis: level === "success" ? "重启" : "启动",
      moduan: level === "success" ? "重启" : "启动",
      visual: level === "success" ? "重启" : "启动",
    };
    const nextAction = nextActionMap[statusId] || "";
    const nextActionLabel = nextActionLabelMap[statusId] || "";
    chip.dataset.statusAction = nextAction;
    chip.setAttribute("aria-label", `${chip.querySelector(".system-status-label")?.textContent || statusId}：${nextActionLabel}`);
    if (actionLabel) {
      actionLabel.textContent = nextActionLabel;
    }
  }

  setTheme(theme) {
    if (!this.refs.themeToggle) {
      return;
    }
    const nextTheme = theme === "dark" ? "light" : "dark";
    const icon = this.refs.themeToggle.querySelector(".toolbar-theme-icon");
    if (icon) {
      icon.textContent = theme === "dark" ? "🌙" : "☀️";
    }
    this.refs.themeToggle.title = `切换到${nextTheme === "light" ? "浅色" : "深色"}模式`;
    this.refs.themeToggle.setAttribute("aria-label", this.refs.themeToggle.title);
  }

  setCabinRemoteCurrentPosition(position) {
    if (!this.refs.cabinRemoteCurrentPosition) {
      return;
    }
    const axes = [
      { id: "x", label: "X" },
      { id: "y", label: "Y" },
      { id: "z", label: "Z" },
    ];
    this.refs.cabinRemoteCurrentPosition.innerHTML = axes.map(({ id, label }) => {
      const value = Number.isFinite(position?.[id]) ? `${Math.round(position[id])} mm` : "等待索驱 TF…";
      return `
        <div class="cabin-remote-position-item">
          <span class="cabin-remote-position-label">${label}</span>
          <span class="cabin-remote-position-value">${value}</span>
        </div>
      `;
    }).join("");
  }

  setCabinRemoteButtonActive(directionId) {
    this.refs.cabinRemoteButtons.forEach((button) => {
      const active = button.dataset.cabinRemoteDirection === directionId;
      button.classList.toggle("is-active", active);
      button.setAttribute("aria-pressed", active ? "true" : "false");
    });
  }

  clearCabinRemoteButtonActive() {
    this.refs.cabinRemoteButtons.forEach((button) => {
      button.classList.remove("is-active");
      button.setAttribute("aria-pressed", "false");
    });
    this.refs.cabinRemoteStopButton?.classList.remove("is-active");
    this.refs.cabinRemoteStopButton?.setAttribute("aria-pressed", "false");
  }

  setCabinRemoteStatus(message) {
    if (!this.refs.cabinRemoteStatus) {
      return;
    }
    this.refs.cabinRemoteStatus.textContent = message || "键位：Q/W/E = Z+/X+/Z-，A/S/D = Y+/X-/Y-。";
    this.refs.cabinRemoteStatus.classList.toggle(
      "cabin-remote-status-active",
      Boolean(this.refs.cabinKeyboardRemoteToggle?.checked),
    );
  }

  flashCustomControlPanelButton(buttonId, { durationMs = 700 } = {}) {
    const timerKey = `custom:${buttonId}`;
    const button = this.refs.customControlPanelButtons.find((item) => item.dataset.customServiceAction === buttonId);
    if (!button) {
      return;
    }
    const timeoutId = this.taskButtonFeedbackTimers.get(timerKey);
    if (timeoutId) {
      window.clearTimeout(timeoutId);
      this.taskButtonFeedbackTimers.delete(timerKey);
    }
    button.classList.add("is-active");
    const nextTimeoutId = window.setTimeout(() => {
      button.classList.remove("is-active");
      if (this.taskButtonFeedbackTimers.get(timerKey) === nextTimeoutId) {
        this.taskButtonFeedbackTimers.delete(timerKey);
      }
    }, durationMs);
    this.taskButtonFeedbackTimers.set(timerKey, nextTimeoutId);
  }

  flashTaskButton(taskAction, { durationMs = 700 } = {}) {
    const button = this.refs.taskButtons.find((item) => item.dataset.taskAction === taskAction);
    if (!button) {
      return;
    }
    this.clearTaskButtonFeedback(taskAction);
    button.classList.add("is-active");
    const timeoutId = window.setTimeout(() => {
      button.classList.remove("is-active");
      if (this.taskButtonFeedbackTimers.get(taskAction) === timeoutId) {
        this.taskButtonFeedbackTimers.delete(taskAction);
      }
    }, durationMs);
    this.taskButtonFeedbackTimers.set(taskAction, timeoutId);
  }

  clearTaskButtonFeedback(taskAction) {
    const timeoutId = this.taskButtonFeedbackTimers.get(taskAction);
    if (timeoutId) {
      window.clearTimeout(timeoutId);
      this.taskButtonFeedbackTimers.delete(taskAction);
    }
    const button = this.refs.taskButtons.find((item) => item.dataset.taskAction === taskAction);
    button?.classList.remove("is-active");
  }

  startCustomControlPanelLongPress(button, event) {
    this.finishCustomControlPanelDrag({ cancel: true });
    this.customControlPanelLongPressContext = {
      button,
      pointerId: event.pointerId,
      startX: event.clientX,
      startY: event.clientY,
    };
    this.customControlPanelLongPressTimerId = window.setTimeout(() => {
      this.customControlPanelLongPressTimerId = null;
      this.setCustomControlPanelEditMode(true);
      this.beginCustomControlPanelDrag(button, {
        pointerId: event.pointerId,
        clientX: event.clientX,
        clientY: event.clientY,
      });
    }, 520);
  }

  cancelCustomControlPanelLongPress() {
    if (this.customControlPanelLongPressTimerId) {
      window.clearTimeout(this.customControlPanelLongPressTimerId);
      this.customControlPanelLongPressTimerId = null;
    }
    this.customControlPanelLongPressContext = null;
  }

  beginCustomControlPanelDrag(button, pointerState) {
    const rect = button.getBoundingClientRect();
    const dragProxy = button.cloneNode(true);
    dragProxy.classList.add("control-action-btn-drag-proxy");
    dragProxy.style.width = `${rect.width}px`;
    dragProxy.style.height = `${rect.height}px`;
    document.body.appendChild(dragProxy);
    this.customControlPanelDragState = {
      buttonId: button.dataset.customServiceAction,
      pointerId: pointerState.pointerId,
      sourceButton: button,
      dragProxy,
      offsetX: pointerState.clientX - rect.left,
      offsetY: pointerState.clientY - rect.top,
    };
    button.classList.add("is-drag-origin");
    this.updateCustomControlPanelDragPosition(pointerState.clientX, pointerState.clientY);
  }

  handleCustomControlPanelPointerMove(event) {
    if (this.customControlPanelDragState) {
      if (event.pointerId !== this.customControlPanelDragState.pointerId) {
        return;
      }
      event.preventDefault();
      this.updateCustomControlPanelDragPosition(event.clientX, event.clientY);
      return;
    }

    if (!this.customControlPanelLongPressContext || event.pointerId !== this.customControlPanelLongPressContext.pointerId) {
      return;
    }

    const deltaX = Math.abs(event.clientX - this.customControlPanelLongPressContext.startX);
    const deltaY = Math.abs(event.clientY - this.customControlPanelLongPressContext.startY);
    if (deltaX > 8 || deltaY > 8) {
      this.finishCustomControlPanelDrag({ cancel: true });
    }
  }

  updateCustomControlPanelDragPosition(clientX, clientY) {
    if (!this.customControlPanelDragState) {
      return;
    }
    const { dragProxy, offsetX, offsetY } = this.customControlPanelDragState;
    dragProxy.style.left = `${clientX - offsetX}px`;
    dragProxy.style.top = `${clientY - offsetY}px`;
    const trashRect = this.refs.controlPanelTrashBin?.getBoundingClientRect();
    const hoveringTrash = Boolean(
      trashRect
      && clientX >= trashRect.left
      && clientX <= trashRect.right
      && clientY >= trashRect.top
      && clientY <= trashRect.bottom,
    );
    this.refs.controlPanelTrashBin?.classList.toggle("is-hovered", hoveringTrash);
  }

  handleCustomControlPanelPointerUp(event) {
    if (this.customControlPanelDragState && event.pointerId === this.customControlPanelDragState.pointerId) {
      this.finishCustomControlPanelDrag({
        clientX: event.clientX,
        clientY: event.clientY,
      });
      return;
    }
    if (this.customControlPanelLongPressContext && event.pointerId === this.customControlPanelLongPressContext.pointerId) {
      this.finishCustomControlPanelDrag({ cancel: true });
    }
  }

  finishCustomControlPanelDrag({ clientX = 0, clientY = 0, cancel = false } = {}) {
    this.cancelCustomControlPanelLongPress();
    window.removeEventListener("pointermove", this.handleCustomControlPanelPointerMove, true);
    window.removeEventListener("pointerup", this.handleCustomControlPanelPointerUp, true);
    window.removeEventListener("pointercancel", this.handleCustomControlPanelPointerUp, true);
    const dragState = this.customControlPanelDragState;
    if (!dragState) {
      this.refs.controlPanelTrashBin?.classList.remove("is-hovered");
      return;
    }

    const trashRect = this.refs.controlPanelTrashBin?.getBoundingClientRect();
    const droppedOnTrash = !cancel
      && trashRect
      && clientX >= trashRect.left
      && clientX <= trashRect.right
      && clientY >= trashRect.top
      && clientY <= trashRect.bottom;
    dragState.dragProxy.remove();
    dragState.sourceButton.classList.remove("is-drag-origin");
    this.refs.controlPanelTrashBin?.classList.remove("is-hovered");
    this.customControlPanelDragState = null;

    if (droppedOnTrash) {
      this.handleCustomControlPanelDelete?.(dragState.buttonId);
      this.setCustomControlPanelEditMode(false);
      return;
    }
  }

  setCustomControlPanelEditMode(enabled) {
    this.customControlPanelEditMode = Boolean(enabled) && this.refs.customControlPanelButtons.length > 0;
    this.refs.customControlPanelButtons.forEach((button) => {
      button.classList.toggle("is-editing", this.customControlPanelEditMode);
    });
    if (this.refs.controlPanelTrashBin) {
      this.refs.controlPanelTrashBin.hidden = !this.customControlPanelEditMode;
    }
    if (!this.customControlPanelEditMode) {
      this.refs.controlPanelTrashBin?.classList.remove("is-hovered");
    }
  }

  setControlFeedback(_message) {}

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
    button.setAttribute("aria-pressed", active ? "true" : "false");
    button.classList.toggle("is-active", active);
    const label = state?.label || (active ? definition.activeLabel : definition.inactiveLabel);
    const light = button.querySelector(".quick-control-light");
    const labelNode = button.querySelector(".quick-control-label");
    if (light) {
      light.classList.toggle("is-active", active);
    }
    if (labelNode) {
      labelNode.textContent = label;
    } else {
      button.textContent = label;
    }
  }

  renderTerminalSessions(sessions, activeSessionId) {
    const items = Array.isArray(sessions) ? sessions : [];
    if (this.refs.terminalEmptyState) {
      this.refs.terminalEmptyState.hidden = items.length > 0;
    }
    if (!this.refs.terminalTabStrip) {
      return;
    }
    this.refs.terminalTabStrip.innerHTML = items.map((session) => `
      <div class="terminal-tab ${session.sessionId === activeSessionId ? "is-active" : ""}" data-state="${session.state || "connecting"}">
        <button class="terminal-tab-select" type="button" data-terminal-select="${session.sessionId}">
          <span class="terminal-tab-label">${session.label}</span>
          <span class="terminal-tab-state">${this.getTerminalStateLabel(session.state)}</span>
        </button>
        <button class="terminal-tab-close" type="button" data-terminal-close="${session.sessionId}" title="关闭终端">×</button>
      </div>
    `).join("");
  }

  getTerminalStateLabel(state) {
    switch (state) {
      case "ready":
        return "在线";
      case "closed":
        return "已断开";
      case "error":
        return "错误";
      default:
        return "连接中";
    }
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
