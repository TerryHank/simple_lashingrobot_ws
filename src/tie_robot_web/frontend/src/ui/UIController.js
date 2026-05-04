import {
  CONTROL_PANEL_TASKS,
  getControlToggleDefinition,
  getControlPanelGroups,
  getInitialControlToggleStateMap,
} from "../config/controlPanelCatalog.js";
import { LEGACY_PARAMETER_DEFAULTS } from "../config/legacyCommandCatalog.js";
import {
  TF_AXIS_FRAMES,
  getPointCloudSourceLabel,
  POINT_CLOUD_SOURCES,
  SCENE_VIEW_MODES,
  TOPIC_LAYER_MODES,
  getTopicLayerModeLabel,
} from "../config/topicLayerCatalog.js";
import { DEFAULT_IMAGE_TOPIC, IMAGE_TOPIC_OPTIONS, getImageTopicLabel } from "../config/imageTopicCatalog.js";
import { DEFAULT_LOG_TOPIC, LOG_TOPIC_OPTIONS, getLogTopicLabel } from "../config/logTopicCatalog.js";
import { STATUS_MONITORS } from "../config/statusMonitorCatalog.js";
import { FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE } from "../config/visualRecognitionMode.js";

const DISPLAY_MODE_LABELS = {
  auto: "自动增强",
  strong: "强增强",
  raw: "原图",
};

const CONNECTION_LABELS = {
  info: "连接中",
  reconnecting: "重连中",
  success: "连接成功",
  manual: "手动重连",
  warn: "连接失败",
  error: "连接失败",
};

const CONNECTION_ACTIONS = {
  info: { id: "", label: "" },
  reconnecting: { id: "manualRosReconnect", label: "立即重连" },
  success: { id: "restartRosStack", label: "重启ROS" },
  manual: { id: "manualRosReconnect", label: "手动重连" },
  warn: { id: "startRosStack", label: "启动ROS" },
  error: { id: "startRosStack", label: "启动ROS" },
};

const STATUS_CHIP_LONG_PRESS_RESTART_MS = 800;

const CABIN_POSITION_AXES = [
  { id: "x", label: "X" },
  { id: "y", label: "Y" },
  { id: "z", label: "Z" },
];

const CABIN_REMOTE_MOVE_MODES = [
  { id: "absolute", label: "绝对点动" },
  { id: "relative", label: "相对点动" },
];

const TCP_LINEAR_POSITION_FIELDS = [
  { id: "x", label: "X", key: "linear_module_position_X", unit: "mm" },
  { id: "y", label: "Y", key: "linear_module_position_Y", unit: "mm" },
  { id: "z", label: "Z", key: "linear_module_position_Z", unit: "mm" },
  { id: "angle", label: "角度", key: "motor_angle", unit: "deg" },
];

const NETWORK_PING_TARGETS = [
  {
    id: "cabin",
    label: "索驱",
    settingsKey: "cabinHost",
    defaultHost: "192.168.6.62",
    hostRef: "networkPingCabinHost",
    buttonRef: "networkPingCabinButton",
    resultRef: "networkPingCabinResult",
    buttonLabel: "保存并测试",
  },
  {
    id: "moduan",
    label: "线性模组",
    settingsKey: "moduanHost",
    defaultHost: "192.168.6.167",
    hostRef: "networkPingModuanHost",
    buttonRef: "networkPingModuanButton",
    resultRef: "networkPingModuanResult",
    buttonLabel: "保存并测试",
  },
];

const SETTINGS_PAGE_OPTIONS = [
  { id: "topics", label: "话题总览" },
  { id: "logs", label: "节点日志" },
  { id: "visualDebug", label: "视觉调试" },
  { id: "gb28181Local", label: "国标接入" },
  { id: "networkPing", label: "网络配置" },
  { id: "workspace", label: "工作区选点" },
  { id: "scene", label: "显示与视角" },
  { id: "cabinRemote", label: "索驱遥控" },
  { id: "tcpLinearRemote", label: "TCP线模遥控" },
  { id: "homeCalibration", label: "Home点位" },
];

const SETTINGS_PAGE_ALIASES = {
  layers: "scene",
  calibration: "visualDebug",
};

function escapeHtml(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function normalizeCabinRemoteMoveMode(moveMode) {
  return moveMode === "relative" ? "relative" : "absolute";
}

export class UIController {
  constructor(rootElement, { panelRegistry = [], initialLayout = null, settingsPageOrder = [] } = {}) {
    this.rootElement = rootElement;
    this.refs = {};
    this.panelRegistry = panelRegistry;
    this.initialLayout = initialLayout;
    this.customSelects = new Map();
    this.taskButtonFeedbackTimers = new Map();
    this.topicInventoryExpanded = new Map();
    this.layerLogExpanded = new Set();
    this.rosBackendActionPending = false;
    this.settingsHomePageId = "topics";
    this.settingsPageOrder = this.normalizeSettingsPageOrder(settingsPageOrder);
    this.settingsPageDragValue = "";
    this.settingsPagePointerDrag = null;
    this.settingsPageDragGhost = null;
    this.settingsPageDragFrame = 0;
    this.bottomCabinPositionTitle = "等待索驱 TF 更新机器位置";
    this.bottomCabinOperationDetail = "索驱不可操作：连接断开或状态未上报";
    this.handleSettingsHomePageChange = null;
    this.handleSettingsPageOrderChange = null;
    this.graphicalAppPanelStates = new Map();
    this.graphicalAppPanelZIndexSeed = 40;
    this.graphicalAppEmbedInstanceId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
    this.handleDocumentPointerDown = this.handleDocumentPointerDown.bind(this);
    this.handleDocumentKeyDown = this.handleDocumentKeyDown.bind(this);
    this.handleSettingsPageGlobalPointerMove = this.handleSettingsPageGlobalPointerMove.bind(this);
    this.handleSettingsPageGlobalPointerUp = this.handleSettingsPageGlobalPointerUp.bind(this);
    this.handleSettingsPageGlobalPointerCancel = this.handleSettingsPageGlobalPointerCancel.bind(this);
    this.handleSettingsPageGlobalDragAbort = this.handleSettingsPageGlobalDragAbort.bind(this);
    this.handleGraphicalAppWindowResize = this.handleGraphicalAppWindowResize.bind(this);
    window.addEventListener("resize", this.handleGraphicalAppWindowResize);
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

  normalizeSettingsPageOrder(pageOrder = []) {
    const allowedIds = new Set(SETTINGS_PAGE_OPTIONS.map((option) => option.id));
    const uniqueIds = [];
    (Array.isArray(pageOrder) ? pageOrder : []).forEach((pageId) => {
      const normalizedPageId = this.normalizeSettingsPageId(pageId);
      if (allowedIds.has(normalizedPageId) && !uniqueIds.includes(normalizedPageId)) {
        uniqueIds.push(normalizedPageId);
      }
    });
    SETTINGS_PAGE_OPTIONS.forEach((option) => {
      if (!uniqueIds.includes(option.id)) {
        uniqueIds.push(option.id);
      }
    });
    return uniqueIds;
  }

  getOrderedSettingsPageOptions() {
    const optionById = new Map(SETTINGS_PAGE_OPTIONS.map((option) => [option.id, option]));
    return this.normalizeSettingsPageOrder(this.settingsPageOrder)
      .map((pageId) => optionById.get(pageId))
      .filter(Boolean);
  }

  getSettingsPageHomeFirstOrder(pageId, pageOrder = this.settingsPageOrder) {
    const normalizedPageId = this.normalizeSettingsPageId(pageId);
    const normalizedOrder = this.normalizeSettingsPageOrder(pageOrder);
    return [
      normalizedPageId,
      ...normalizedOrder.filter((candidateId) => candidateId !== normalizedPageId),
    ];
  }

  getSettingsPageOrder() {
    return [...this.settingsPageOrder];
  }

  renderSettingsPageOptionMarkup() {
    return this.getOrderedSettingsPageOptions()
      .map((option) => `<option value="${escapeHtml(option.id)}">${escapeHtml(option.label)}</option>`)
      .join("");
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
              <button
                id="voltageBadge"
                class="toolbar-voltage-badge warn"
                type="button"
                data-voltage-action="startRosStack"
                title="机器人电压暂未获取，点击启动 ROS 后端"
              >电压 --.-V</button>
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
                <button class="panel-action-btn panel-maximize-btn" type="button" data-panel-maximize="controlPanel" title="最大化">⛶</button>
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:controlPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content control-panel-content">
              <div id="controlPanelTaskGrid" class="control-button-grid"></div>
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
                <div class="panel-subtitle">话题、工作区、图层、外参与国标接入</div>
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
                    ${this.renderSettingsPageOptionMarkup()}
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

                <section class="settings-page" data-settings-page="logs" hidden>
                  <div class="settings-section settings-layer-log-section">
                    <div class="section-title">节点日志</div>
                    <div id="settingsLayerLogList" class="settings-layer-log-list"></div>
                  </div>
                </section>

                <section class="settings-page" data-settings-page="visualDebug" hidden>
                  <div class="settings-grid visual-debug-grid">
                    <div class="settings-section visual-debug-control-card">
                      <div class="section-title">视觉调试</div>
                      <div class="field-grid compact-grid">
                        <div class="field">
                          <label for="visualDebugStableFrameCount">释放帧数</label>
                          <input id="visualDebugStableFrameCount" type="number" min="1" max="30" step="1" value="3" />
                        </div>
                      </div>
                      <div class="button-row">
                        <button id="visualDebugApplyStableFrameCount" class="secondary-btn" type="button">应用帧数</button>
                        <button id="visualDebugTrigger" class="primary-btn" type="button">触发视觉服务</button>
                      </div>
                      <div id="visualDebugTimingSummary" class="info-block mono">单帧=--ms 服务=--ms 释放=3帧 点数=--</div>
                    </div>

                    <div class="settings-section gripper-tf-calibration-card">
                      <div class="section-title">相机-TCP外参</div>
                      <div id="gripperTfCurrent" class="info-block mono gripper-tf-current">等待 TF 链路后读取当前外参。</div>
                      <div class="field-grid compact-grid gripper-tf-grid">
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
                      <div class="button-row gripper-tf-actions">
                        <button id="applyGripperTfCalibration" class="primary-btn" type="button">应用外参</button>
                      </div>
                    </div>

                    <div class="settings-section visual-debug-log-card">
                      <div class="section-title">视觉调试日志</div>
                      <ul id="visualDebugLogList" class="visual-debug-log-list"></ul>
                    </div>
                  </div>
                </section>

                <section class="settings-page" data-settings-page="gb28181Local" hidden>
                  <div class="settings-grid gb28181-local-grid">
                    <div class="settings-section gb28181-local-card gb28181-local-card-primary">
                      <div class="section-title">GB28181 接入参数</div>
                      <div class="gb28181-local-form">
                        <div class="field">
                          <label for="gb28181RemoteIp">上级平台 SIP IP</label>
                          <input id="gb28181RemoteIp" type="text" inputmode="decimal" placeholder="由平台工作人员提供" autocomplete="off" />
                        </div>
                        <div class="field">
                          <label for="gb28181RemotePort">上级平台 SIP 端口</label>
                          <input id="gb28181RemotePort" type="number" min="1" max="65535" step="1" value="5060" />
                        </div>
                        <div class="field">
                          <label for="gb28181ServerId">平台国标 ID</label>
                          <input id="gb28181ServerId" type="text" placeholder="由平台提供" autocomplete="off" />
                        </div>
                        <div class="field">
                          <label for="gb28181Domain">国标域</label>
                          <input id="gb28181Domain" type="text" placeholder="由平台提供" autocomplete="off" />
                        </div>
                        <div class="field">
                          <label for="gb28181Password">设备接入密码</label>
                          <input id="gb28181Password" type="text" placeholder="由平台分配" autocomplete="off" />
                        </div>
                        <div class="field">
                          <label for="gb28181LocalIp">本机 SIP IP</label>
                          <input id="gb28181LocalIp" type="text" readonly value="" />
                        </div>
                        <div class="field">
                          <label for="gb28181LocalPort">本机 SIP 端口</label>
                          <input id="gb28181LocalPort" type="number" min="1" max="65535" step="1" value="5060" />
                        </div>
                        <div class="field">
                          <label for="gb28181DeviceId">本机设备 ID</label>
                          <input id="gb28181DeviceId" type="text" readonly value="34020000001320000001" />
                        </div>
                      </div>
                      <div class="button-row">
                        <button id="gb28181RefreshLocalIp" class="secondary-btn" type="button">刷新本机 IP</button>
                        <button id="gb28181ResetRemote" class="secondary-btn" type="button">清空平台输入</button>
                        <a class="help-link" href="/help/guide/gb28181-video-gateway" target="_blank" rel="noreferrer">帮助文档</a>
                      </div>
                    </div>

                    <div class="settings-section gb28181-local-card">
                      <div class="section-title">本机配置</div>
                      <button id="gb28181WriteConfig" class="primary-btn gb28181-local-write-btn" type="button">写入本机配置</button>
                      <div id="gb28181ConfigStatus" class="gb28181-local-status" data-state="idle">填写完成后点击按钮。</div>
                    </div>
                  </div>
                </section>

                <section class="settings-page" data-settings-page="networkPing" hidden>
                  <div class="settings-grid network-ping-grid">
                    <div class="settings-section network-ping-card">
                      <div class="section-title">索驱上位机</div>
                      <div class="field-grid compact-grid network-ping-field-grid">
                        <div class="field">
                          <label for="networkPingCabinHost">上位机 IP</label>
                          <input id="networkPingCabinHost" type="text" inputmode="decimal" autocomplete="off" value="192.168.6.62" />
                        </div>
                        <div class="network-ping-action-cell">
                          <button
                            id="networkPingCabinButton"
                            class="secondary-btn network-ping-button"
                            type="button"
                            data-network-ping-target="cabin"
                            data-state="idle"
                          >保存并测试</button>
                        </div>
                      </div>
                      <div id="networkPingCabinResult" class="network-ping-result" data-state="idle">等待测试。</div>
                    </div>

                    <div class="settings-section network-ping-card">
                      <div class="section-title">线性模组上位机</div>
                      <div class="field-grid compact-grid network-ping-field-grid">
                        <div class="field">
                          <label for="networkPingModuanHost">上位机 IP</label>
                          <input id="networkPingModuanHost" type="text" inputmode="decimal" autocomplete="off" value="192.168.6.167" />
                        </div>
                        <div class="network-ping-action-cell">
                          <button
                            id="networkPingModuanButton"
                            class="secondary-btn network-ping-button"
                            type="button"
                            data-network-ping-target="moduan"
                            data-state="idle"
                          >保存并测试</button>
                        </div>
                      </div>
                      <div id="networkPingModuanResult" class="network-ping-result" data-state="idle">等待测试。</div>
                    </div>
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
                            <option value="raw" selected>原图</option>
                          </select>
                        </div>
                        <div class="field">
                          <label for="gammaRange">伽马</label>
                          <input id="gammaRange" type="range" min="0.40" max="1.60" step="0.05" value="1.00" />
                        </div>
                        <div class="field">
                          <label for="overlayOpacityRange">覆盖透明度</label>
                          <input id="overlayOpacityRange" type="range" min="0.00" max="1.00" step="0.02" value="0.88" />
                        </div>
                        <div class="info-block mono" id="displaySettingSummary">模式=原图 伽马=1.00 覆盖=0.88</div>
                      </div>
                    </div>
                  </div>
                </section>

                <section class="settings-page" data-settings-page="scene" hidden>
                  <div class="settings-grid scene-view-grid">
                    <div class="settings-section scene-view-card">
                      <div class="section-title">视角</div>
                      <input id="sceneViewMode" type="hidden" value="free" />
                      <div class="scene-view-control-row">
                        <div class="scene-view-mode-group" role="group" aria-label="视角">
                          ${SCENE_VIEW_MODES.map((mode) => `
                            <button
                              class="scene-view-mode-button ${mode.id === "free" ? "is-active" : ""}"
                              type="button"
                              data-scene-view-mode="${mode.id}"
                              aria-pressed="${mode.id === "free" ? "true" : "false"}"
                            >${mode.label}</button>
                          `).join("")}
                        </div>
                        <label class="checkbox-field scene-follow-origin-field">
                          <input id="followOriginToggle" type="checkbox" />
                          <span>跟随原点</span>
                        </label>
                      </div>
                    </div>

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
                        <label class="checkbox-field"><input id="showPlanningMarkersToggle" type="checkbox" checked /><span>规划点/绑扎点</span></label>
                      </div>

                      <div class="tf-axis-frame-panel">
                        <div class="tf-axis-frame-header">
                          <span>坐标轴明细</span>
                          <small>总开关关闭时全部隐藏</small>
                        </div>
                        <div class="tf-axis-frame-grid">
                          ${TF_AXIS_FRAMES.map((frame) => `
                            <label class="checkbox-field tf-axis-frame-option">
                              <input type="checkbox" data-tf-axis-frame="${frame.id}" checked />
                              <span>${frame.label}</span>
                            </label>
                          `).join("")}
                        </div>
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

                      <div id="topicLayerSummary" class="info-block mono">模式=点云 + 规划点/绑扎点 点云源=滤波世界点云 点大小=0.035 透明度=0.78</div>

                      <div class="section-title">当前数据量</div>
                      <div id="topicLayerStats" class="stats-grid"></div>
                    </div>
                  </div>
                </section>

                <section class="settings-page" data-settings-page="homeCalibration" hidden>
                  <div class="settings-grid home-calibration-grid">
                    <div class="settings-section">
                      <div class="section-title">Home点位</div>
                      <div id="robotHomeSummary" class="info-block mono">等待 Home 标定服务。</div>
                      <div class="field-grid compact-grid">
                        <div class="field">
                          <label for="robotHomeX">Home X (mm)</label>
                          <input id="robotHomeX" type="number" step="1" value="0" />
                        </div>
                        <div class="field">
                          <label for="robotHomeY">Home Y (mm)</label>
                          <input id="robotHomeY" type="number" step="1" value="0" />
                        </div>
                        <div class="field">
                          <label for="robotHomeZ">Home Z (mm)</label>
                          <input id="robotHomeZ" type="number" step="1" value="0" />
                        </div>
                      </div>
                      <div class="button-row">
                        <button id="refreshRobotHome" class="secondary-btn" type="button">刷新</button>
                        <button id="captureRobotHome" class="secondary-btn" type="button">当前位置设为Home</button>
                        <button id="saveRobotHomeCalibration" class="primary-btn" type="button">保存</button>
                        <button id="moveRobotHome" class="secondary-btn" type="button">回Home</button>
                      </div>
                    </div>

                    <div class="settings-section">
                      <div class="section-title">base_link → 相机</div>
                      <div id="robotHomeTfSummary" class="info-block mono">等待 TF 投射状态。</div>
                      <div id="baseToCameraComputed" class="info-block mono">由 TF 链自动计算，人工只设置 Home 点。</div>
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
                      <input id="cabinRemoteMoveMode" type="hidden" value="absolute" />
                      <div class="cabin-remote-mode-group" role="group" aria-label="索驱点动模式">
                        ${CABIN_REMOTE_MOVE_MODES.map((mode) => `
                          <button
                            class="cabin-remote-mode-button ${mode.id === "absolute" ? "is-active" : ""}"
                            type="button"
                            data-cabin-remote-move-mode="${mode.id}"
                            aria-pressed="${mode.id === "absolute" ? "true" : "false"}"
                          >${mode.label}</button>
                        `).join("")}
                      </div>
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
                      <div id="cabinRemoteStatus" class="info-block mono">键位：Q/W/E = Z+/X+/Z-，A/S/D = Y+/X-/Y-，空格 = 暂停。</div>
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
                      <div class="section-title">绝对目标位姿</div>
                      <div class="field-grid compact-grid">
                        <div class="field">
                          <label for="cabinRemoteAbsoluteX">X (mm)</label>
                          <input id="cabinRemoteAbsoluteX" type="number" step="1" />
                        </div>
                        <div class="field">
                          <label for="cabinRemoteAbsoluteY">Y (mm)</label>
                          <input id="cabinRemoteAbsoluteY" type="number" step="1" />
                        </div>
                        <div class="field">
                          <label for="cabinRemoteAbsoluteZ">Z (mm)</label>
                          <input id="cabinRemoteAbsoluteZ" type="number" step="1" />
                        </div>
                      </div>
                      <div class="cabin-remote-absolute-actions">
                        <button id="cabinRemoteAbsoluteMoveButton" class="primary-btn" type="button" disabled>移动到绝对位姿</button>
                      </div>
                    </div>
                  </div>
                </section>

                <section class="settings-page" data-settings-page="tcpLinearRemote" hidden>
                  <div class="settings-grid">
                    <div class="settings-section tcp-linear-remote-card">
                      <div class="section-title">TCP 线性模组遥控</div>
                      <div class="info-block">
                        这里控制的是末端 TCP 线性模组，调用 <span class="mono">/moduan/single_move</span>。
                        后端按绝对坐标执行，按钮会基于实时上传位置做安全步进。
                      </div>
                      <div id="tcpLinearRemoteCurrentPosition" class="info-block tcp-linear-remote-position-grid mono">
                        ${TCP_LINEAR_POSITION_FIELDS.map((field) => `
                          <div class="tcp-linear-remote-position-item">
                            <span class="tcp-linear-remote-position-label">${field.label}</span>
                            <span class="tcp-linear-remote-position-value">等待线模状态…</span>
                          </div>
                        `).join("")}
                      </div>
                      <div class="tcp-linear-remote-pad">
                        <button class="secondary-btn tcp-linear-remote-btn" type="button" data-tcp-linear-remote-axis="zPositive" disabled>Z+</button>
                        <button class="secondary-btn tcp-linear-remote-btn" type="button" data-tcp-linear-remote-axis="xPositive" disabled>X+</button>
                        <button class="secondary-btn tcp-linear-remote-btn" type="button" data-tcp-linear-remote-axis="zNegative" disabled>Z-</button>
                        <button class="secondary-btn tcp-linear-remote-btn" type="button" data-tcp-linear-remote-axis="yPositive" disabled>Y+</button>
                        <button
                          class="secondary-btn tcp-linear-remote-btn tcp-linear-remote-stop-btn"
                          type="button"
                          data-tcp-linear-remote-stop="true"
                          disabled
                        >运动暂停</button>
                        <button class="secondary-btn tcp-linear-remote-btn" type="button" data-tcp-linear-remote-axis="yNegative" disabled>Y-</button>
                        <button class="secondary-btn tcp-linear-remote-btn tcp-linear-remote-angle-btn" type="button" data-tcp-linear-remote-axis="anglePositive" disabled>角度+</button>
                        <button class="secondary-btn tcp-linear-remote-btn" type="button" data-tcp-linear-remote-axis="xNegative" disabled>X-</button>
                        <button class="secondary-btn tcp-linear-remote-btn tcp-linear-remote-angle-btn" type="button" data-tcp-linear-remote-axis="angleNegative" disabled>角度-</button>
                      </div>
                      <div class="field-grid compact-grid">
                        <div class="field">
                          <label for="tcpLinearRemoteStep">单次线性步距（mm）</label>
                          <input id="tcpLinearRemoteStep" type="number" min="1" step="1" value="5" />
                        </div>
                        <div class="field">
                          <label for="tcpLinearRemoteAngleStep">单次角度步距（deg）</label>
                          <input id="tcpLinearRemoteAngleStep" type="number" min="1" step="1" value="5" />
                        </div>
                      </div>
                      <div id="tcpLinearRemoteStatus" class="info-block mono">等待线性模组状态；行程：X 0~360mm，Y 0~320mm，Z 0~140mm。</div>
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
                <div id="terminalEmptyState" class="terminal-empty-state" data-state="idle">
                  <div class="terminal-empty-title">还没有终端会话</div>
                  <div class="terminal-empty-subtitle">点击右上角加号，或从工具栏打开“终端”后自动创建本机 shell。</div>
                  <button id="createTerminalSessionEmpty" class="primary-btn" type="button">新建终端</button>
                </div>
              </div>
            </div>
          </section>

          <div id="graphicalAppPanelDock" class="graphical-app-panel-dock"></div>

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
          <div
            id="bottomCabinPosition"
            class="bottom-cabin-position mono"
            data-state="waiting"
            data-operation-state="blocked"
            title="等待索驱 TF 更新机器位置"
            aria-label="等待索驱 TF 更新机器位置"
          >
            <span class="bottom-cabin-position-title">机器位置</span>
            <span class="bottom-cabin-position-axis" data-bottom-cabin-axis="x">X -- mm</span>
            <span class="bottom-cabin-position-axis" data-bottom-cabin-axis="y">Y -- mm</span>
            <span class="bottom-cabin-position-axis" data-bottom-cabin-axis="z">Z -- mm</span>
          </div>
          <div
            id="bottomLinearModulePosition"
            class="bottom-linear-module-position mono"
            data-state="waiting"
            title="等待线性模组状态和 gripper_frame TF"
            aria-label="等待线性模组状态和 gripper_frame TF"
          >
            <div class="bottom-linear-module-row" data-bottom-linear-row="local">
              <span class="bottom-linear-module-title">线模本地</span>
              <span class="bottom-linear-module-axis" data-bottom-linear-axis="local:x">X -- mm</span>
              <span class="bottom-linear-module-axis" data-bottom-linear-axis="local:y">Y -- mm</span>
              <span class="bottom-linear-module-axis" data-bottom-linear-axis="local:z">Z -- mm</span>
            </div>
            <div class="bottom-linear-module-row" data-bottom-linear-row="global">
              <span class="bottom-linear-module-title">线模全局</span>
              <span class="bottom-linear-module-axis" data-bottom-linear-axis="global:x">X -- mm</span>
              <span class="bottom-linear-module-axis" data-bottom-linear-axis="global:y">Y -- mm</span>
              <span class="bottom-linear-module-axis" data-bottom-linear-axis="global:z">Z -- mm</span>
            </div>
          </div>
          <div id="quickControlGrid" class="quick-control-grid"></div>
        </div>
      </div>
    `;
    this.bindRefs();
    this.applyDefaultPanelVisibility();
    this.renderStatusChips();
    this.renderControlPanelTasks();
    this.renderLegacyCommands();
    this.renderPointList([]);
    this.renderTerminalSessions([], null);
    this.renderLogs([]);
    this.renderSettingsLayerLogs([]);
    this.renderVisualDebugLogs([]);
    this.renderTopicLayerStats({
      filteredWorldCoordCount: 0,
      rawWorldCoordCount: 0,
      tiePointCount: 0,
      planningPointCount: 0,
      tfFrameCount: 0,
    });
    this.renderTopicInventory([]);
    this.setGripperTfCalibration(null);
    this.setTcpLinearRemoteState(null);
    this.setBottomLinearModulePosition(null, null);
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
    this.refs.sceneViewModeButtons = [...this.rootElement.querySelectorAll("[data-scene-view-mode]")];
    this.refs.followOriginToggle = this.rootElement.querySelector("#followOriginToggle");
    this.refs.connectionBadge = this.rootElement.querySelector("#connectionBadge");
    this.refs.voltageBadge = this.rootElement.querySelector("#voltageBadge");
    this.refs.themeToggle = this.rootElement.querySelector("#themeToggle");
    this.refs.imageTopicSelect = this.rootElement.querySelector("#imageTopicSelect");
    this.refs.logTopicSelect = this.rootElement.querySelector("#logTopicSelect");
    this.refs.terminalTabStrip = this.rootElement.querySelector("#terminalTabStrip");
    this.refs.terminalViewport = this.rootElement.querySelector("#terminalViewport");
    this.refs.terminalEmptyState = this.rootElement.querySelector("#terminalEmptyState");
    this.refs.terminalEmptyTitle = this.rootElement.querySelector("#terminalEmptyState .terminal-empty-title");
    this.refs.terminalEmptySubtitle = this.rootElement.querySelector("#terminalEmptyState .terminal-empty-subtitle");
    this.refs.createTerminalSession = this.rootElement.querySelector("#createTerminalSession");
    this.refs.createTerminalSessionEmpty = this.rootElement.querySelector("#createTerminalSessionEmpty");
    this.refs.graphicalAppPanelDock = this.rootElement.querySelector("#graphicalAppPanelDock");
    this.refs.controlPanelTaskGrid = this.rootElement.querySelector("#controlPanelTaskGrid");
    this.refs.quickControlGrid = this.rootElement.querySelector("#quickControlGrid");
    this.refs.topicLayerMode = this.rootElement.querySelector("#topicLayerMode");
    this.refs.pointCloudSource = this.rootElement.querySelector("#pointCloudSource");
    this.refs.showRobotToggle = this.rootElement.querySelector("#showRobotToggle");
    this.refs.showAxesToggle = this.rootElement.querySelector("#showAxesToggle");
    this.refs.showPointCloudToggle = this.rootElement.querySelector("#showPointCloudToggle");
    this.refs.showPlanningMarkersToggle = this.rootElement.querySelector("#showPlanningMarkersToggle");
    this.refs.tfAxisFrameToggles = [...this.rootElement.querySelectorAll("[data-tf-axis-frame]")];
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
    this.refs.settingsLayerLogList = this.rootElement.querySelector("#settingsLayerLogList");
    this.refs.visualDebugStableFrameCount = this.rootElement.querySelector("#visualDebugStableFrameCount");
    this.refs.visualDebugApplyStableFrameCount = this.rootElement.querySelector("#visualDebugApplyStableFrameCount");
    this.refs.visualDebugTrigger = this.rootElement.querySelector("#visualDebugTrigger");
    this.refs.visualDebugTimingSummary = this.rootElement.querySelector("#visualDebugTimingSummary");
    this.refs.visualDebugLogList = this.rootElement.querySelector("#visualDebugLogList");
    this.refs.clearLogs = this.rootElement.querySelector("#clearLogs");
    this.refs.settingsPageSelect = this.rootElement.querySelector("#settingsPageSelect");
    this.refs.settingsPages = [...this.rootElement.querySelectorAll("[data-settings-page]")];
    this.refs.systemActionButtons = [...this.rootElement.querySelectorAll("[data-system-action]")];
    this.refs.gb28181RemoteIp = this.rootElement.querySelector("#gb28181RemoteIp");
    this.refs.gb28181RemotePort = this.rootElement.querySelector("#gb28181RemotePort");
    this.refs.gb28181ServerId = this.rootElement.querySelector("#gb28181ServerId");
    this.refs.gb28181Domain = this.rootElement.querySelector("#gb28181Domain");
    this.refs.gb28181Password = this.rootElement.querySelector("#gb28181Password");
    this.refs.gb28181LocalIp = this.rootElement.querySelector("#gb28181LocalIp");
    this.refs.gb28181LocalPort = this.rootElement.querySelector("#gb28181LocalPort");
    this.refs.gb28181DeviceId = this.rootElement.querySelector("#gb28181DeviceId");
    this.refs.gb28181RefreshLocalIp = this.rootElement.querySelector("#gb28181RefreshLocalIp");
    this.refs.gb28181ResetRemote = this.rootElement.querySelector("#gb28181ResetRemote");
    this.refs.gb28181WriteConfig = this.rootElement.querySelector("#gb28181WriteConfig");
    this.refs.gb28181ConfigStatus = this.rootElement.querySelector("#gb28181ConfigStatus");
    this.refs.networkPingCabinHost = this.rootElement.querySelector("#networkPingCabinHost");
    this.refs.networkPingModuanHost = this.rootElement.querySelector("#networkPingModuanHost");
    this.refs.networkPingCabinButton = this.rootElement.querySelector("#networkPingCabinButton");
    this.refs.networkPingModuanButton = this.rootElement.querySelector("#networkPingModuanButton");
    this.refs.networkPingCabinResult = this.rootElement.querySelector("#networkPingCabinResult");
    this.refs.networkPingModuanResult = this.rootElement.querySelector("#networkPingModuanResult");
    this.refs.networkPingInputs = [this.refs.networkPingCabinHost, this.refs.networkPingModuanHost].filter(Boolean);
    this.refs.networkPingButtons = [...this.rootElement.querySelectorAll("[data-network-ping-target]")];
    this.refs.gripperTfCurrent = this.rootElement.querySelector("#gripperTfCurrent");
    this.refs.gripperTfX = this.rootElement.querySelector("#gripperTfX");
    this.refs.gripperTfY = this.rootElement.querySelector("#gripperTfY");
    this.refs.gripperTfZ = this.rootElement.querySelector("#gripperTfZ");
    this.refs.applyGripperTfCalibration = this.rootElement.querySelector("#applyGripperTfCalibration");
    this.refs.robotHomeSummary = this.rootElement.querySelector("#robotHomeSummary");
    this.refs.robotHomeTfSummary = this.rootElement.querySelector("#robotHomeTfSummary");
    this.refs.robotHomeX = this.rootElement.querySelector("#robotHomeX");
    this.refs.robotHomeY = this.rootElement.querySelector("#robotHomeY");
    this.refs.robotHomeZ = this.rootElement.querySelector("#robotHomeZ");
    this.refs.baseToCameraComputed = this.rootElement.querySelector("#baseToCameraComputed");
    this.refs.refreshRobotHome = this.rootElement.querySelector("#refreshRobotHome");
    this.refs.captureRobotHome = this.rootElement.querySelector("#captureRobotHome");
    this.refs.saveRobotHomeCalibration = this.rootElement.querySelector("#saveRobotHomeCalibration");
    this.refs.moveRobotHome = this.rootElement.querySelector("#moveRobotHome");
    this.refs.cabinKeyboardRemoteToggle = this.rootElement.querySelector("#cabinKeyboardRemoteToggle");
    this.refs.cabinRemoteMoveMode = this.rootElement.querySelector("#cabinRemoteMoveMode");
    this.refs.cabinRemoteMoveModeButtons = [...this.rootElement.querySelectorAll("[data-cabin-remote-move-mode]")];
    this.refs.cabinRemoteStep = this.rootElement.querySelector("#cabinRemoteStep");
    this.refs.cabinRemoteSpeed = this.rootElement.querySelector("#cabinRemoteSpeed");
    this.refs.cabinRemoteAbsoluteX = this.rootElement.querySelector("#cabinRemoteAbsoluteX");
    this.refs.cabinRemoteAbsoluteY = this.rootElement.querySelector("#cabinRemoteAbsoluteY");
    this.refs.cabinRemoteAbsoluteZ = this.rootElement.querySelector("#cabinRemoteAbsoluteZ");
    this.refs.cabinRemoteAbsoluteMoveButton = this.rootElement.querySelector("#cabinRemoteAbsoluteMoveButton");
    this.refs.bottomCabinPosition = this.rootElement.querySelector("#bottomCabinPosition");
    this.refs.bottomCabinPositionAxes = [...this.rootElement.querySelectorAll("[data-bottom-cabin-axis]")];
    this.refs.bottomLinearModulePosition = this.rootElement.querySelector("#bottomLinearModulePosition");
    this.refs.bottomLinearModuleAxes = [...this.rootElement.querySelectorAll("[data-bottom-linear-axis]")];
    this.refs.cabinRemoteCurrentPosition = this.rootElement.querySelector("#cabinRemoteCurrentPosition");
    this.refs.cabinRemoteStatus = this.rootElement.querySelector("#cabinRemoteStatus");
    this.refs.cabinRemoteButtons = [...this.rootElement.querySelectorAll("[data-cabin-remote-direction]")];
    this.refs.cabinRemoteStopButton = this.rootElement.querySelector("#cabinRemoteStopButton");
    this.refs.tcpLinearRemoteCurrentPosition = this.rootElement.querySelector("#tcpLinearRemoteCurrentPosition");
    this.refs.tcpLinearRemoteStatus = this.rootElement.querySelector("#tcpLinearRemoteStatus");
    this.refs.tcpLinearRemoteStep = this.rootElement.querySelector("#tcpLinearRemoteStep");
    this.refs.tcpLinearRemoteAngleStep = this.rootElement.querySelector("#tcpLinearRemoteAngleStep");
    this.refs.tcpLinearRemoteButtons = [...this.rootElement.querySelectorAll("[data-tcp-linear-remote-axis]")];
    this.refs.tcpLinearRemoteStopButton = this.rootElement.querySelector("[data-tcp-linear-remote-stop]");
    this.refs.taskButtons = [...this.rootElement.querySelectorAll("[data-task-action]")];
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
      this.refs.showPlanningMarkersToggle,
      ...this.refs.tfAxisFrameToggles,
      this.refs.pointSizeRange,
      this.refs.pointOpacityRange,
    ].filter(Boolean);
    this.refs.sceneInputs = [this.refs.followOriginToggle].filter(Boolean);
    this.enhanceSelectControls();
    this.refs.settingsPageSelect?.addEventListener("change", () => this.setSettingsPage(this.refs.settingsPageSelect.value));
    this.bindGb28181LocalControls();
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
        optionButton.draggable = false;
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
        optionRow.dataset.settingsPageOption = option.value;
        optionRow.draggable = false;

        const dragHandle = document.createElement("span");
        dragHandle.className = "settings-page-drag-handle";
        dragHandle.textContent = "⋮⋮";
        dragHandle.title = "拖动排序";
        dragHandle.setAttribute("role", "button");
        dragHandle.setAttribute("tabindex", "0");
        dragHandle.setAttribute("aria-label", `拖动排序：${option.textContent}`);
        dragHandle.addEventListener("pointerdown", (event) => {
          this.handleSettingsPagePointerDown(event, option.value, optionRow, dragHandle);
        });
        dragHandle.addEventListener("keydown", (event) => {
          this.handleSettingsPageHandleKeyDown(event, option.value);
        });

        const homeButton = document.createElement("button");
        homeButton.type = "button";
        homeButton.className = "settings-home-page-button";
        homeButton.dataset.settingsHomePage = option.value;
        homeButton.draggable = false;
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
          const normalizedPageId = this.setSettingsHomePage(option.value, {
            notifyOrder: true,
            animate: true,
          });
          if (previousHomePageId !== normalizedPageId) {
            this.handleSettingsHomePageChange?.(normalizedPageId);
          }
        });

        optionRow.appendChild(dragHandle);
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
    if (event.target.closest(".ui-select-shell")) {
      return;
    }
    this.closeAllCustomSelects();
  }

  handleDocumentKeyDown(event) {
    if (event.key === "Escape") {
      this.closeAllCustomSelects();
    }
  }

  handleSettingsPagePointerDown(event, optionValue, optionRow, dragHandle) {
    if (event.button !== 0 || !optionValue || !optionRow) {
      return;
    }
    const entry = this.customSelects.get("settingsPageSelect");
    if (!entry?.menu?.contains(optionRow)) {
      return;
    }
    this.finishSettingsPagePointerDrag();
    event.preventDefault();
    event.stopPropagation();

    const rect = optionRow.getBoundingClientRect();
    this.settingsPagePointerDrag = {
      entry,
      handle: dragHandle,
      pointerId: event.pointerId,
      sourceValue: optionValue,
      sourceRow: optionRow,
      offsetX: event.clientX - rect.left,
      offsetY: event.clientY - rect.top,
      lastClientX: event.clientX,
      lastClientY: event.clientY,
      lastInsertIndex: null,
      orderChanged: false,
      previousBodyUserSelect: document.body.style.userSelect,
    };

    optionRow.classList.add("is-dragging", "is-drag-source");
    entry.menu.classList.add("is-sorting-menu");
    document.body.style.userSelect = "none";
    dragHandle?.setPointerCapture?.(event.pointerId);
    this.addSettingsPageDragGlobalListeners();
    this.createSettingsPageDragGhost(optionRow);
    this.updateSettingsPageDragGhost(event.clientX, event.clientY);
  }

  handleSettingsPagePointerMove(event) {
    const drag = this.settingsPagePointerDrag;
    if (!drag || drag.pointerId !== event.pointerId) {
      return;
    }
    event.preventDefault();
    event.stopPropagation();
    drag.lastClientX = event.clientX;
    drag.lastClientY = event.clientY;
    if (!this.settingsPageDragFrame) {
      this.settingsPageDragFrame = requestAnimationFrame(() => this.flushSettingsPagePointerDragFrame());
    }
  }

  flushSettingsPagePointerDragFrame() {
    this.settingsPageDragFrame = 0;
    const drag = this.settingsPagePointerDrag;
    if (!drag) {
      return;
    }
    this.updateSettingsPageDragGhost(drag.lastClientX, drag.lastClientY);
    const insertIndex = this.getSettingsPagePointerInsertIndex(drag.lastClientY, drag.sourceValue);
    if (insertIndex === drag.lastInsertIndex) {
      return;
    }
    drag.lastInsertIndex = insertIndex;
    this.highlightSettingsPageInsertionTarget(insertIndex, drag.sourceValue);
    const normalizedOrder = this.moveSettingsPageOptionToIndex(drag.sourceValue, insertIndex, { notify: false });
    if (normalizedOrder) {
      drag.orderChanged = true;
      drag.sourceRow = this.getSettingsPageOptionRow(drag.sourceValue) || drag.sourceRow;
      drag.sourceRow?.classList.add("is-dragging", "is-drag-source");
    }
  }

  handleSettingsPagePointerUp(event) {
    const drag = this.settingsPagePointerDrag;
    if (!drag || drag.pointerId !== event.pointerId) {
      return;
    }
    event.preventDefault();
    event.stopPropagation();
    this.finishSettingsPagePointerDrag();
  }

  handleSettingsPagePointerCancel(event) {
    const drag = this.settingsPagePointerDrag;
    if (drag && drag.pointerId === event.pointerId) {
      this.finishSettingsPagePointerDrag();
    }
  }

  handleSettingsPageGlobalPointerMove(event) {
    this.handleSettingsPagePointerMove(event);
  }

  handleSettingsPageGlobalPointerUp(event) {
    this.handleSettingsPagePointerUp(event);
  }

  handleSettingsPageGlobalPointerCancel(event) {
    this.handleSettingsPagePointerCancel(event);
  }

  handleSettingsPageGlobalDragAbort(event) {
    if (event?.type === "visibilitychange" && document.visibilityState === "visible") {
      return;
    }
    this.finishSettingsPagePointerDrag();
  }

  handleSettingsPageHandleKeyDown(event, optionValue) {
    if (event.key !== "ArrowUp" && event.key !== "ArrowDown") {
      return;
    }
    event.preventDefault();
    event.stopPropagation();
    const currentOrder = this.normalizeSettingsPageOrder(this.settingsPageOrder);
    const currentIndex = currentOrder.indexOf(optionValue);
    if (currentIndex < 0) {
      return;
    }
    const targetIndex = event.key === "ArrowUp"
      ? Math.max(0, currentIndex - 1)
      : Math.min(currentOrder.length - 1, currentIndex + 1);
    const normalizedOrder = this.moveSettingsPageOptionToIndex(optionValue, targetIndex);
    if (normalizedOrder) {
      requestAnimationFrame(() => {
        this.getSettingsPageOptionRow(optionValue)
          ?.querySelector(".settings-page-drag-handle")
          ?.focus();
      });
    }
  }

  addSettingsPageDragGlobalListeners() {
    window.addEventListener("pointermove", this.handleSettingsPageGlobalPointerMove, true);
    window.addEventListener("pointerup", this.handleSettingsPageGlobalPointerUp, true);
    window.addEventListener("pointercancel", this.handleSettingsPageGlobalPointerCancel, true);
    window.addEventListener("blur", this.handleSettingsPageGlobalDragAbort, true);
    document.addEventListener("visibilitychange", this.handleSettingsPageGlobalDragAbort, true);
  }

  removeSettingsPageDragGlobalListeners() {
    window.removeEventListener("pointermove", this.handleSettingsPageGlobalPointerMove, true);
    window.removeEventListener("pointerup", this.handleSettingsPageGlobalPointerUp, true);
    window.removeEventListener("pointercancel", this.handleSettingsPageGlobalPointerCancel, true);
    window.removeEventListener("blur", this.handleSettingsPageGlobalDragAbort, true);
    document.removeEventListener("visibilitychange", this.handleSettingsPageGlobalDragAbort, true);
  }

  finishSettingsPagePointerDrag() {
    const drag = this.settingsPagePointerDrag;
    if (this.settingsPageDragFrame) {
      cancelAnimationFrame(this.settingsPageDragFrame);
      this.settingsPageDragFrame = 0;
      this.flushSettingsPagePointerDragFrame();
    }
    if (drag?.handle?.hasPointerCapture?.(drag.pointerId)) {
      try {
        drag.handle.releasePointerCapture(drag.pointerId);
      } catch (_error) {
        // Pointer capture can already be gone if the browser cancels the gesture.
      }
    }
    const finalOrder = drag?.orderChanged ? [...this.settingsPageOrder] : null;
    document.body.style.userSelect = drag?.previousBodyUserSelect || "";
    this.removeSettingsPageDragGlobalListeners();
    this.customSelects
      .get("settingsPageSelect")
      ?.menu.querySelectorAll(".settings-page-option-row.is-dragging, .settings-page-option-row.is-drag-source, .settings-page-option-row.is-drag-over")
      .forEach((row) => {
        row.classList.remove("is-dragging", "is-drag-source", "is-drag-over");
      });
    this.customSelects.get("settingsPageSelect")?.menu.classList.remove("is-sorting-menu");
    this.cleanupSettingsPageDragGhost();
    this.settingsPagePointerDrag = null;
    if (finalOrder) {
      this.notifySettingsPageOrderCommitted(finalOrder);
    }
  }

  getSettingsPagePointerInsertIndex(clientY, sourceValue) {
    const entry = this.customSelects.get("settingsPageSelect");
    if (!entry?.menu) {
      return -1;
    }
    const rows = [...entry.menu.querySelectorAll("[data-settings-page-option]")]
      .filter((row) => row.dataset.settingsPageOption !== sourceValue);
    const orderWithoutSource = this.normalizeSettingsPageOrder(this.settingsPageOrder)
      .filter((pageId) => pageId !== sourceValue);
    for (const row of rows) {
      const rect = row.getBoundingClientRect();
      if (clientY < rect.top + rect.height / 2) {
        return orderWithoutSource.indexOf(row.dataset.settingsPageOption);
      }
    }
    return orderWithoutSource.length;
  }

  highlightSettingsPageInsertionTarget(insertIndex, sourceValue) {
    const entry = this.customSelects.get("settingsPageSelect");
    if (!entry?.menu) {
      return;
    }
    const orderWithoutSource = this.normalizeSettingsPageOrder(this.settingsPageOrder)
      .filter((pageId) => pageId !== sourceValue);
    const targetValue = orderWithoutSource[insertIndex] || orderWithoutSource[orderWithoutSource.length - 1] || "";
    entry.menu
      .querySelectorAll(".settings-page-option-row.is-drag-over")
      .forEach((row) => row.classList.remove("is-drag-over"));
    const targetRow = targetValue ? this.getSettingsPageOptionRow(targetValue) : null;
    targetRow?.classList.add("is-drag-over");
  }

  moveSettingsPageOption(sourceValue, targetValue) {
    const nextOrder = this.normalizeSettingsPageOrder(this.settingsPageOrder);
    const sourceIndex = nextOrder.indexOf(sourceValue);
    const targetIndex = nextOrder.indexOf(targetValue);
    if (sourceIndex < 0 || targetIndex < 0 || sourceIndex === targetIndex) {
      return;
    }
    const [movedPageId] = nextOrder.splice(sourceIndex, 1);
    nextOrder.splice(targetIndex, 0, movedPageId);
    return this.commitSettingsPageOrder(nextOrder, { animate: true });
  }

  moveSettingsPageOptionToIndex(sourceValue, insertIndex, { notify = true } = {}) {
    const currentOrder = this.normalizeSettingsPageOrder(this.settingsPageOrder);
    if (!currentOrder.includes(sourceValue) || insertIndex < 0) {
      return null;
    }
    const orderWithoutSource = currentOrder.filter((pageId) => pageId !== sourceValue);
    const targetIndex = Math.min(Math.max(insertIndex, 0), orderWithoutSource.length);
    const nextOrder = [...orderWithoutSource];
    nextOrder.splice(targetIndex, 0, sourceValue);
    return this.commitSettingsPageOrder(nextOrder, { animate: true, notify });
  }

  commitSettingsPageOrder(pageOrder, { animate = false, notify = true } = {}) {
    const normalizedOrder = this.normalizeSettingsPageOrder(pageOrder);
    if (normalizedOrder.join("\u0000") === this.settingsPageOrder.join("\u0000")) {
      return null;
    }
    const entry = this.customSelects.get("settingsPageSelect");
    const beforeRects = animate ? this.captureSettingsPageRowRects(entry) : null;
    const committedOrder = this.setSettingsPageOrder(normalizedOrder, { animateFrom: beforeRects });
    if (notify) {
      this.notifySettingsPageOrderCommitted(committedOrder);
    }
    return committedOrder;
  }

  notifySettingsPageOrderCommitted(pageOrder) {
    const committedOrder = this.normalizeSettingsPageOrder(pageOrder);
    this.handleSettingsPageOrderChange?.(committedOrder);
    const firstPageId = committedOrder[0] || "";
    if (firstPageId && firstPageId !== this.settingsHomePageId) {
      const normalizedHomePageId = this.setSettingsHomePage(firstPageId, {
        reorder: false,
      });
      this.handleSettingsHomePageChange?.(normalizedHomePageId);
    }
  }

  setSettingsPageOrder(pageOrder, { animateFrom = null } = {}) {
    this.settingsPageOrder = this.normalizeSettingsPageOrder(pageOrder);
    if (this.refs.settingsPageSelect) {
      const entry = this.customSelects.get("settingsPageSelect");
      this.applySettingsPageOrderToDom(this.refs.settingsPageSelect);
      this.refreshCustomSelect(this.refs.settingsPageSelect);
      if (animateFrom) {
        this.animateSettingsPageOrderChange(entry, animateFrom);
      }
    }
    return [...this.settingsPageOrder];
  }

  applySettingsPageOrderToDom(select) {
    const currentValue = this.normalizeSettingsPageId(select.value);
    const optionByValue = new Map([...select.options].map((option) => [option.value, option]));
    this.settingsPageOrder.forEach((pageId) => {
      const option = optionByValue.get(pageId);
      if (option) {
        select.appendChild(option);
      }
    });
    select.value = currentValue;

    const entry = this.customSelects.get(select.id);
    if (!entry) {
      return;
    }
    const rowByValue = new Map(
      [...entry.menu.querySelectorAll("[data-settings-page-option]")]
        .map((row) => [row.dataset.settingsPageOption, row]),
    );
    this.settingsPageOrder.forEach((pageId) => {
      const row = rowByValue.get(pageId);
      if (row) {
        entry.menu.appendChild(row);
      }
    });
  }

  getSettingsPageOptionRow(pageId) {
    if (!pageId) {
      return null;
    }
    const escapedPageId = typeof CSS !== "undefined" && CSS.escape
      ? CSS.escape(pageId)
      : String(pageId).replace(/"/g, '\\"');
    return this.customSelects
      .get("settingsPageSelect")
      ?.menu.querySelector(`[data-settings-page-option="${escapedPageId}"]`) || null;
  }

  captureSettingsPageRowRects(entry = this.customSelects.get("settingsPageSelect")) {
    if (!entry?.menu) {
      return null;
    }
    return new Map(
      [...entry.menu.querySelectorAll("[data-settings-page-option]")].map((row) => {
        const rect = row.getBoundingClientRect();
        return [row.dataset.settingsPageOption, {
          top: rect.top,
          left: rect.left,
          width: rect.width,
          height: rect.height,
        }];
      }),
    );
  }

  animateSettingsPageOrderChange(entry = this.customSelects.get("settingsPageSelect"), beforeRects = null) {
    if (!entry?.menu || !beforeRects?.size) {
      return;
    }
    const reduceMotion = window.matchMedia?.("(prefers-reduced-motion: reduce)")?.matches;
    if (reduceMotion) {
      return;
    }
    [...entry.menu.querySelectorAll("[data-settings-page-option]")].forEach((row) => {
      const before = beforeRects.get(row.dataset.settingsPageOption);
      if (!before) {
        return;
      }
      const after = row.getBoundingClientRect();
      const deltaY = before.top - after.top;
      if (Math.abs(deltaY) < 0.5) {
        return;
      }
      row.classList.add("is-sorting");
      row.style.transition = "none";
      row.style.transform = `translate3d(0, ${deltaY}px, 0)`;
      row.getBoundingClientRect();
      requestAnimationFrame(() => {
        row.style.transition = "";
        row.style.transform = "";
      });
      const cleanup = () => {
        row.classList.remove("is-sorting");
        row.style.transition = "";
        row.style.transform = "";
        row.removeEventListener("transitionend", cleanup);
      };
      row.addEventListener("transitionend", cleanup, { once: true });
      window.setTimeout(cleanup, 260);
    });
  }

  createSettingsPageDragGhost(optionRow) {
    this.cleanupSettingsPageDragGhost();
    if (!optionRow) {
      return null;
    }
    const rect = optionRow.getBoundingClientRect();
    const ghost = optionRow.cloneNode(true);
    ghost.classList.add("settings-page-drag-ghost");
    ghost.classList.remove("is-dragging", "is-drag-source", "is-drag-over", "is-sorting");
    ghost.removeAttribute("draggable");
    ghost.style.width = `${rect.width}px`;
    ghost.style.height = `${rect.height}px`;
    document.body.appendChild(ghost);
    this.settingsPageDragGhost = ghost;
    return ghost;
  }

  updateSettingsPageDragGhost(clientX, clientY) {
    const drag = this.settingsPagePointerDrag;
    if (!drag || !this.settingsPageDragGhost) {
      return;
    }
    const left = Math.round(clientX - drag.offsetX);
    const top = Math.round(clientY - drag.offsetY);
    this.settingsPageDragGhost.style.transform = `translate3d(${left}px, ${top}px, 0)`;
  }

  cleanupSettingsPageDragGhost() {
    this.settingsPageDragGhost?.remove();
    this.settingsPageDragGhost = null;
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
    const normalizedPageId = SETTINGS_PAGE_ALIASES[pageId] || pageId;
    return SETTINGS_PAGE_OPTIONS.some((option) => option.id === normalizedPageId) ? normalizedPageId : "topics";
  }

  setSettingsHomePage(pageId, { reorder = true, notifyOrder = false, animate = false } = {}) {
    const normalizedPageId = this.normalizeSettingsPageId(pageId);
    this.settingsHomePageId = normalizedPageId;
    if (reorder) {
      const nextOrder = this.getSettingsPageHomeFirstOrder(normalizedPageId);
      const orderChanged = nextOrder.join("\u0000") !== this.settingsPageOrder.join("\u0000");
      if (orderChanged) {
        const entry = this.customSelects.get("settingsPageSelect");
        const beforeRects = animate ? this.captureSettingsPageRowRects(entry) : null;
        const committedOrder = this.setSettingsPageOrder(nextOrder, { animateFrom: beforeRects });
        if (notifyOrder) {
          this.handleSettingsPageOrderChange?.(committedOrder);
        }
        return normalizedPageId;
      }
    }
    if (this.refs.settingsPageSelect) {
      this.refreshCustomSelect(this.refs.settingsPageSelect);
    }
    return normalizedPageId;
  }

  inferGb28181LocalIp() {
    if (typeof window === "undefined") {
      return "";
    }
    const hostname = window.location.hostname || "";
    if (!hostname || hostname === "localhost") {
      return "127.0.0.1";
    }
    return hostname;
  }

  getGb28181InputValue(refName, fallback = "") {
    return this.refs[refName]?.value?.trim() || fallback;
  }

  bindGb28181LocalControls() {
    if (!this.refs.gb28181WriteConfig) {
      return;
    }
    if (this.refs.gb28181LocalIp && !this.refs.gb28181LocalIp.value) {
      this.refs.gb28181LocalIp.value = this.inferGb28181LocalIp();
    }
    const markChanged = () => this.setGb28181ConfigStatus("参数已修改，尚未写入。", "idle");
    [
      this.refs.gb28181RemoteIp,
      this.refs.gb28181RemotePort,
      this.refs.gb28181ServerId,
      this.refs.gb28181Domain,
      this.refs.gb28181Password,
      this.refs.gb28181LocalIp,
      this.refs.gb28181LocalPort,
    ].filter(Boolean).forEach((element) => {
      element.addEventListener("input", markChanged);
      element.addEventListener("change", markChanged);
    });
    this.refs.gb28181RefreshLocalIp?.addEventListener("click", () => {
      if (this.refs.gb28181LocalIp) {
        this.refs.gb28181LocalIp.value = this.inferGb28181LocalIp();
      }
      markChanged();
    });
    this.refs.gb28181ResetRemote?.addEventListener("click", () => {
      [
        this.refs.gb28181RemoteIp,
        this.refs.gb28181ServerId,
        this.refs.gb28181Domain,
        this.refs.gb28181Password,
      ].filter(Boolean).forEach((element) => {
        element.value = "";
      });
      if (this.refs.gb28181RemotePort) {
        this.refs.gb28181RemotePort.value = "5060";
      }
      this.setGb28181ConfigStatus("已清空平台输入。", "idle");
    });
    this.refs.gb28181WriteConfig.addEventListener("click", () => {
      this.writeGb28181Config();
    });
    this.setGb28181ConfigStatus("填写完成后点击按钮。", "idle");
  }

  setGb28181ConfigStatus(message, state = "idle") {
    if (!this.refs.gb28181ConfigStatus) {
      return;
    }
    this.refs.gb28181ConfigStatus.textContent = message;
    this.refs.gb28181ConfigStatus.dataset.state = state;
  }

  getGb28181Port(refName, fallback = 5060) {
    const rawValue = this.getGb28181InputValue(refName, String(fallback));
    const port = Number(rawValue);
    return Number.isInteger(port) ? port : NaN;
  }

  buildGb28181ConfigPayload() {
    const localIp = this.getGb28181InputValue("gb28181LocalIp", this.inferGb28181LocalIp());
    if (this.refs.gb28181LocalIp && !this.refs.gb28181LocalIp.value && localIp) {
      this.refs.gb28181LocalIp.value = localIp;
    }

    const fields = [
      { ref: "gb28181RemoteIp", key: "server_ip", label: "上级平台 SIP IP" },
      { ref: "gb28181ServerId", key: "server_id", label: "平台国标 ID" },
      { ref: "gb28181Domain", key: "domain", label: "国标域" },
      { ref: "gb28181Password", key: "password", label: "设备接入密码" },
      { ref: "gb28181LocalIp", key: "local_ip", label: "本机 SIP IP" },
      { ref: "gb28181DeviceId", key: "device_id", label: "本机设备 ID" },
    ];
    const payload = { sip: {} };
    const missing = [];
    fields.forEach(({ ref, key, label }) => {
      const value = this.getGb28181InputValue(ref);
      payload.sip[key] = value;
      if (!value) {
        missing.push({ ref, label });
      }
    });

    if (missing.length) {
      this.setGb28181ConfigStatus(`请先填写：${missing.map(({ label }) => label).join("、")}。`, "warn");
      this.refs[missing[0].ref]?.focus?.();
      return null;
    }

    const remotePort = this.getGb28181Port("gb28181RemotePort", 5060);
    const localPort = this.getGb28181Port("gb28181LocalPort", 5060);
    if (!Number.isInteger(remotePort) || remotePort < 1 || remotePort > 65535) {
      this.setGb28181ConfigStatus("上级平台 SIP 端口必须是 1-65535。", "warn");
      this.refs.gb28181RemotePort?.focus?.();
      return null;
    }
    if (!Number.isInteger(localPort) || localPort < 1 || localPort > 65535) {
      this.setGb28181ConfigStatus("本机 SIP 端口必须是 1-65535。", "warn");
      this.refs.gb28181LocalPort?.focus?.();
      return null;
    }

    payload.sip.server_port = remotePort;
    payload.sip.local_port = localPort;
    return payload;
  }

  async writeGb28181Config() {
    const payload = this.buildGb28181ConfigPayload();
    if (!payload) {
      return;
    }

    const button = this.refs.gb28181WriteConfig;
    const originalLabel = button?.textContent || "写入本机配置";
    if (button) {
      button.disabled = true;
      button.classList.add("is-pending");
      button.textContent = "正在写入…";
    }
    this.setGb28181ConfigStatus("正在写入本机配置…", "pending");

    try {
      const response = await fetch("/api/gb28181/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      const result = await response.json().catch(() => ({}));
      if (!response.ok || result.success === false) {
        throw new Error(result.message || "写入失败，请联系工程人员。");
      }
      this.setGb28181ConfigStatus(result.message || "已写入本机配置。", "success");
    } catch (error) {
      this.setGb28181ConfigStatus(error?.message || "写入失败，请联系工程人员。", "error");
    } finally {
      if (button) {
        button.disabled = false;
        button.classList.remove("is-pending");
        button.textContent = originalLabel;
      }
    }
  }

  renderStatusChips() {
    const monitorMarkup = STATUS_MONITORS
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
    const demoModeMarkup = `
      <button
        class="system-status-item error is-interactive"
        type="button"
        data-status-id="demoMode"
        data-status-action="toggleDemoMode"
        aria-pressed="false"
        title="演示模式未启用"
      >
          <span class="system-status-light"></span>
          <span class="system-status-text">
            <span class="system-status-label">演示模式</span>
            <span class="system-status-action-label">进入</span>
          </span>
      </button>
    `;
    this.refs.statusCapsuleGrid.innerHTML = `${monitorMarkup}${demoModeMarkup}`;
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
      showTiePoints: false,
      showPlanningMarkers: this.refs.showPlanningMarkersToggle.checked,
      tfAxisFrameVisibility: this.getTfAxisFrameVisibility(),
      pointSize: Number.parseFloat(this.refs.pointSizeRange.value),
      pointOpacity: Number.parseFloat(this.refs.pointOpacityRange.value),
      viewMode: this.getSelectedSceneViewMode(),
      followOrigin: this.refs.followOriginToggle.checked,
    };
  }

  getTfAxisFrameVisibility() {
    return this.refs.tfAxisFrameToggles.reduce((visibility, input) => {
      visibility[input.dataset.tfAxisFrame] = input.checked;
      return visibility;
    }, {});
  }

  getSceneViewState() {
    return {
      viewMode: this.getSelectedSceneViewMode(),
      followOrigin: this.refs.followOriginToggle.checked,
    };
  }

  getSelectedSceneViewMode() {
    const currentValue = this.refs.sceneViewMode?.value || "free";
    return SCENE_VIEW_MODES.some((mode) => mode.id === currentValue) ? currentValue : "free";
  }

  setSceneViewMode(viewMode) {
    const normalizedViewMode = SCENE_VIEW_MODES.some((mode) => mode.id === viewMode)
      ? viewMode
      : "free";
    if (this.refs.sceneViewMode) {
      this.refs.sceneViewMode.value = normalizedViewMode;
    }
    this.refs.sceneViewModeButtons?.forEach((button) => {
      const active = button.dataset.sceneViewMode === normalizedViewMode;
      button.classList.toggle("is-active", active);
      button.setAttribute("aria-pressed", active ? "true" : "false");
    });
    return normalizedViewMode;
  }

  getCabinRemoteSettings() {
    return {
      keyboardEnabled: Boolean(this.refs.cabinKeyboardRemoteToggle?.checked),
      moveMode: normalizeCabinRemoteMoveMode(this.refs.cabinRemoteMoveMode?.value),
      step: Number.parseFloat(this.refs.cabinRemoteStep?.value || "50"),
      speed: Number.parseFloat(this.refs.cabinRemoteSpeed?.value || "300"),
    };
  }

  getCabinRemoteAbsoluteTarget() {
    return {
      x: Number.parseFloat(this.refs.cabinRemoteAbsoluteX?.value || ""),
      y: Number.parseFloat(this.refs.cabinRemoteAbsoluteY?.value || ""),
      z: Number.parseFloat(this.refs.cabinRemoteAbsoluteZ?.value || ""),
      speed: Number.parseFloat(this.refs.cabinRemoteSpeed?.value || "300"),
    };
  }

  setCabinRemoteAbsoluteTarget(position) {
    const activeElement = document.activeElement;
    const isEditing = [
      this.refs.cabinRemoteAbsoluteX,
      this.refs.cabinRemoteAbsoluteY,
      this.refs.cabinRemoteAbsoluteZ,
    ].includes(activeElement);
    if (isEditing) {
      return;
    }
    CABIN_POSITION_AXES.forEach(({ id }) => {
      const refName = `cabinRemoteAbsolute${id.toUpperCase()}`;
      if (!this.refs[refName] || !Number.isFinite(Number(position?.[id]))) {
        return;
      }
      this.refs[refName].value = String(Math.round(Number(position[id])));
    });
  }

  setCabinRemoteSettings(settings) {
    const step = Number.isFinite(Number(settings?.step)) && Number(settings.step) > 0
      ? Number(settings.step)
      : 50;
    const speed = Number.isFinite(Number(settings?.speed)) && Number(settings.speed) > 0
      ? Number(settings.speed)
      : 300;
    if (this.refs.cabinRemoteStep) {
      this.refs.cabinRemoteStep.value = String(step);
    }
    if (this.refs.cabinRemoteSpeed) {
      this.refs.cabinRemoteSpeed.value = String(speed);
    }
    this.setCabinRemoteMoveMode(settings?.moveMode);
  }

  setCabinRemoteMoveMode(moveMode) {
    const normalizedMoveMode = normalizeCabinRemoteMoveMode(moveMode);
    if (this.refs.cabinRemoteMoveMode) {
      this.refs.cabinRemoteMoveMode.value = normalizedMoveMode;
    }
    this.refs.cabinRemoteMoveModeButtons?.forEach((button) => {
      const active = button.dataset.cabinRemoteMoveMode === normalizedMoveMode;
      button.classList.toggle("is-active", active);
      button.setAttribute("aria-pressed", active ? "true" : "false");
    });
    return normalizedMoveMode;
  }

  getNetworkPingTargetMeta(targetId) {
    return NETWORK_PING_TARGETS.find((target) => target.id === targetId) || null;
  }

  getNetworkPingSettings() {
    return NETWORK_PING_TARGETS.reduce((settings, target) => {
      const value = this.refs[target.hostRef]?.value;
      settings[target.settingsKey] = value == null ? target.defaultHost : String(value).trim();
      return settings;
    }, {});
  }

  setNetworkPingSettings(settings = {}) {
    NETWORK_PING_TARGETS.forEach((target) => {
      const input = this.refs[target.hostRef];
      if (!input) {
        return;
      }
      input.value = String(settings?.[target.settingsKey] || target.defaultHost);
    });
  }

  setNetworkPingPending(targetId, pending) {
    const target = this.getNetworkPingTargetMeta(targetId);
    const button = target ? this.refs[target.buttonRef] : null;
    if (!button) {
      return;
    }
    button.disabled = Boolean(pending);
    button.classList.toggle("is-pending", Boolean(pending));
    if (pending) {
      button.dataset.state = "pending";
    }
    button.textContent = pending ? "测试中…" : target.buttonLabel;
  }

  setNetworkPingResult(targetId, result) {
    const target = this.getNetworkPingTargetMeta(targetId);
    const resultNode = target ? this.refs[target.resultRef] : null;
    if (!resultNode) {
      return;
    }
    const button = this.refs[target.buttonRef];
    const state = result?.state
      || (result?.success ? "success" : result?.reachable === false ? "error" : "idle");
    const hasResultTarget = Object.prototype.hasOwnProperty.call(result || {}, "target");
    const inputHost = this.refs[target.hostRef]?.value?.trim();
    const targetHost = hasResultTarget ? String(result.target || "").trim() : inputHost || target.defaultHost;
    const displayHost = targetHost || "未填写";
    const summary = String(result?.summary || result?.message || "").trim();
    const durationText = Number.isFinite(Number(result?.durationMs))
      ? `，耗时 ${Math.round(Number(result.durationMs))} ms`
      : "";
    const directMessage = !result?.target && summary;
    const displayText = directMessage
      ? summary
      : state === "pending"
        ? `${target.label} ${displayHost}：正在测试连接`
        : state === "success"
          ? `${target.label} ${displayHost}：连接成功${durationText}`
          : state === "error"
            ? `${target.label} ${displayHost}：连接失败${durationText}`
            : `${target.label} ${displayHost}：等待测试。`;
    resultNode.dataset.state = state;
    resultNode.textContent = displayText;
    resultNode.title = summary || displayText;
    if (button) {
      button.dataset.state = state;
    }
  }

  getVisualDebugSettings() {
    return {
      stableFrameCount: Math.max(1, Math.round(Number.parseFloat(this.refs.visualDebugStableFrameCount?.value || "3"))),
      requestMode: FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE,
    };
  }

  setVisualDebugSettings(settings) {
    const stableFrameCount = Math.max(1, Math.round(Number(settings?.stableFrameCount) || 3));
    if (this.refs.visualDebugStableFrameCount) {
      this.refs.visualDebugStableFrameCount.value = String(stableFrameCount);
    }
    this.setVisualDebugTimingSummary({
      releaseFrameCount: stableFrameCount,
    });
  }

  getTcpLinearRemoteSettings() {
    return {
      step: Number.parseFloat(this.refs.tcpLinearRemoteStep?.value || "5"),
      angleStep: Number.parseFloat(this.refs.tcpLinearRemoteAngleStep?.value || "5"),
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

  getPanelVisibilitySnapshot() {
    const snapshot = {};
    this.refs.panelElements?.forEach((_panel, panelId) => {
      snapshot[panelId] = { visible: this.isPanelVisible(panelId) };
    });
    return snapshot;
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
      let longPressTimer = null;
      let suppressNextClick = false;
      const clearLongPressTimer = () => {
        button.classList.remove("is-long-press-charging");
        if (!longPressTimer) {
          return;
        }
        window.clearTimeout(longPressTimer);
        longPressTimer = null;
      };
      button.addEventListener("pointerdown", (event) => {
        if (event.button !== undefined && event.button !== 0) {
          return;
        }
        if (button.disabled || button.classList.contains("is-pending") || !button.dataset.statusLongAction) {
          return;
        }
        clearLongPressTimer();
        suppressNextClick = false;
        button.classList.add("is-long-press-charging");
        longPressTimer = window.setTimeout(() => {
          longPressTimer = null;
          suppressNextClick = true;
          button.classList.remove("is-long-press-charging");
          if (button.disabled || button.classList.contains("is-pending")) {
            return;
          }
          callback(button.dataset.statusId, button.dataset.statusLongAction);
        }, STATUS_CHIP_LONG_PRESS_RESTART_MS);
      });
      ["pointerup", "pointerleave", "pointercancel"].forEach((eventName) => {
        button.addEventListener(eventName, () => {
          clearLongPressTimer();
        });
      });
      button.addEventListener("click", (event) => {
        event.preventDefault();
        event.stopPropagation();
        if (suppressNextClick) {
          suppressNextClick = false;
          return;
        }
        if (button.disabled || button.classList.contains("is-pending")) {
          return;
        }
        if (!button.dataset.statusAction) {
          return;
        }
        callback(button.dataset.statusId, button.dataset.statusAction);
      });
    });
  }

  onSystemAction(callback) {
    this.refs.systemActionButtons.forEach((button) => {
      button.addEventListener("click", (event) => {
        event.preventDefault();
        event.stopPropagation();
        if (button.disabled) {
          return;
        }
        callback(button.dataset.systemAction);
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
    this.refs.voltageBadge?.addEventListener("click", (event) => {
      event.preventDefault();
      event.stopPropagation();
      if (!this.refs.voltageBadge.dataset.voltageAction) {
        return;
      }
      callback(this.refs.voltageBadge.dataset.voltageAction);
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

  onSettingsPageOrderChange(callback) {
    this.handleSettingsPageOrderChange = callback;
  }

  onWorkspaceAction(callback) {
    this.refs.workspaceButtons.forEach((button) => {
      button.addEventListener("click", () => callback(button.dataset.workspaceAction));
    });
  }

  onCabinRemoteAction(callback) {
    this.refs.cabinRemoteButtons.forEach((button) => {
      button.addEventListener("click", () => {
        if (button.disabled) {
          return;
        }
        callback(button.dataset.cabinRemoteDirection, { type: "click" });
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

  onCabinRemoteAbsoluteMoveAction(callback) {
    this.refs.cabinRemoteAbsoluteMoveButton?.addEventListener("click", () => {
      if (this.refs.cabinRemoteAbsoluteMoveButton.disabled) {
        return;
      }
      callback(this.getCabinRemoteAbsoluteTarget());
    });
  }

  onCabinRemoteSettingsChange(callback) {
    this.refs.cabinRemoteMoveModeButtons?.forEach((button) => {
      button.addEventListener("click", (event) => {
        event.preventDefault();
        this.setCabinRemoteMoveMode(button.dataset.cabinRemoteMoveMode);
        callback(this.getCabinRemoteSettings());
      });
    });
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

  onNetworkPingSettingsChange(callback) {
    this.refs.networkPingInputs
      ?.filter(Boolean)
      .forEach((element) => {
        const notify = () => {
          callback(this.getNetworkPingSettings());
          const target = NETWORK_PING_TARGETS.find((candidate) => this.refs[candidate.hostRef] === element);
          if (target) {
            this.setNetworkPingResult(target.id, {
              state: "idle",
              target: element.value.trim(),
            });
          }
        };
        element.addEventListener("input", notify);
        element.addEventListener("change", notify);
      });
  }

  onNetworkPingTest(callback) {
    this.refs.networkPingButtons
      ?.filter(Boolean)
      .forEach((button) => {
        button.addEventListener("click", (event) => {
          event.preventDefault();
          event.stopPropagation();
          if (button.disabled) {
            return;
          }
          callback(button.dataset.networkPingTarget);
        });
      });
  }

  onTcpLinearRemoteAction(callback) {
    this.refs.tcpLinearRemoteStopButton?.addEventListener("click", () => {
      if (this.refs.tcpLinearRemoteStopButton.disabled) {
        return;
      }
      callback("stopMotion", { type: "stop" });
    });
    this.refs.tcpLinearRemoteButtons.forEach((button) => {
      button.addEventListener("click", () => {
        if (button.disabled) {
          return;
        }
        callback(button.dataset.tcpLinearRemoteAxis);
      });
    });
  }

  onTcpLinearRemoteSettingsChange(callback) {
    [this.refs.tcpLinearRemoteStep, this.refs.tcpLinearRemoteAngleStep]
      .filter(Boolean)
      .forEach((element) => {
        element.addEventListener("input", () => callback(this.getTcpLinearRemoteSettings()));
        element.addEventListener("change", () => callback(this.getTcpLinearRemoteSettings()));
      });
  }

  onVisualDebugApplyStableFrameCount(callback) {
    this.refs.visualDebugApplyStableFrameCount?.addEventListener("click", () => {
      callback(this.getVisualDebugSettings());
    });
  }

  onVisualDebugTrigger(callback) {
    this.refs.visualDebugTrigger?.addEventListener("click", () => {
      callback(this.getVisualDebugSettings());
    });
  }

  onVisualDebugSettingsChange(callback) {
    [this.refs.visualDebugStableFrameCount]
      .filter(Boolean)
      .forEach((element) => {
        element.addEventListener("input", () => callback(this.getVisualDebugSettings()));
        element.addEventListener("change", () => callback(this.getVisualDebugSettings()));
      });
  }

  onCalibrationApply(callback) {
    this.refs.applyGripperTfCalibration?.addEventListener("click", () => callback(this.getGripperTfCalibrationInputs()));
  }

  onRobotHomeCalibrationAction(callback) {
    [
      [this.refs.refreshRobotHome, "refresh"],
      [this.refs.captureRobotHome, "capture"],
      [this.refs.saveRobotHomeCalibration, "save"],
      [this.refs.moveRobotHome, "moveHome"],
    ].forEach(([button, action]) => {
      button?.addEventListener("click", () => callback(action, this.getRobotHomeCalibrationInputs()));
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
    this.refs.sceneViewModeButtons?.forEach((button) => {
      button.addEventListener("click", (event) => {
        event.preventDefault();
        this.setSceneViewMode(button.dataset.sceneViewMode);
        callback(this.getSceneViewState());
      });
    });
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

  onGraphicalAppAction(callback) {
    this.refs.graphicalAppPanelDock?.addEventListener("click", (event) => {
      const maximizeButton = event.target.closest("[data-gui-session-maximize]");
      if (maximizeButton) {
        event.preventDefault();
        event.stopPropagation();
        this.toggleGraphicalAppMaximized(maximizeButton.dataset.guiSessionMaximize);
        return;
      }
      const closeButton = event.target.closest("[data-gui-session-close]");
      if (closeButton) {
        callback("close", closeButton.dataset.guiSessionClose);
      }
    });
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
    this.setStatusChipState(
      "ros",
      normalizedLevel === "success" ? "success" : ["info", "reconnecting"].includes(normalizedLevel) ? "info" : "warn",
      message || url || CONNECTION_LABELS[normalizedLevel],
    );
  }

  setBatteryVoltage(voltage) {
    if (!Number.isFinite(voltage) || voltage <= 0 || voltage > 100) {
      this.refs.voltageBadge.textContent = "电压 --.-V";
      this.refs.voltageBadge.title = this.rosBackendActionPending
        ? "ROS 后端正在通过 systemd 启动…"
        : "机器人电压无效或超出显示范围，点击启动 ROS 后端";
      this.refs.voltageBadge.dataset.voltageAction = "startRosStack";
      this.refs.voltageBadge.className = this.rosBackendActionPending
        ? "toolbar-voltage-badge is-booting warn"
        : "toolbar-voltage-badge warn";
      return;
    }
    const formatted = Number(voltage).toFixed(1);
    this.refs.voltageBadge.textContent = `电压 ${formatted}V`;
    this.refs.voltageBadge.title = `机器人电压 ${formatted}V`;
    this.refs.voltageBadge.dataset.voltageAction = "";
    this.refs.voltageBadge.className = "toolbar-voltage-badge success";
  }

  setSystemActionPending(actionId, pending) {
    this.refs.systemActionButtons
      ?.filter((button) => button.dataset.systemAction === actionId)
      .forEach((button) => {
        button.disabled = Boolean(pending);
        button.classList.toggle("is-pending", Boolean(pending));
      });
    [...this.rootElement.querySelectorAll("[data-status-action]")]
      .filter((button) => button.dataset.statusAction === actionId
        || button.dataset.statusLongAction === actionId
        || button.dataset.pendingActionId === actionId)
      .forEach((button) => {
        button.disabled = Boolean(pending);
        button.classList.toggle("is-pending", Boolean(pending));
        const actionLabel = button.querySelector(".system-status-action-label");
        const statusLabel = button.querySelector(".system-status-label")?.textContent || "状态";
        if (!actionLabel) {
          return;
        }
        if (pending) {
          const pendingLabel = this.getPendingActionLabel(actionId);
          button.dataset.pendingActionId = actionId;
          button.setAttribute("aria-busy", "true");
          button.setAttribute("aria-label", `${statusLabel}：${pendingLabel}`);
          actionLabel.textContent = pendingLabel;
          return;
        }
        delete button.dataset.pendingActionId;
        button.removeAttribute("aria-busy");
        actionLabel.textContent = this.getStatusActionLabelForAction(button.dataset.statusAction);
        button.setAttribute("aria-label", `${statusLabel}：${actionLabel.textContent}`);
      });
    if (!["startRosStack", "restartRosStack", "stopRosStack"].includes(actionId)) {
      return;
    }
    this.rosBackendActionPending = Boolean(pending);
    this.refs.connectionBadge?.classList.toggle("is-booting", this.rosBackendActionPending);
    if (!this.refs.voltageBadge) {
      return;
    }
    this.refs.voltageBadge.classList.toggle("is-booting", this.rosBackendActionPending);
    if (this.rosBackendActionPending) {
      this.refs.voltageBadge.title = "ROS 后端正在通过 systemd 启动…";
    }
  }

  getPendingActionLabel(actionId) {
    if (actionId === "toggleDemoMode") {
      return "切换中";
    }
    if (actionId.startsWith("stop")) {
      return "关闭中";
    }
    if (actionId.startsWith("restart")) {
      return "重启中";
    }
    return "启动中";
  }

  getStatusActionLabelForAction(actionId) {
    if (actionId === "toggleDemoMode") {
      return "进入";
    }
    if (actionId === "restartRosStack") {
      return "重启ROS";
    }
    if (actionId === "startRosStack") {
      return "启动ROS";
    }
    if (actionId?.startsWith("stop")) {
      return "关闭";
    }
    if (actionId?.startsWith("restart")) {
      return "重启";
    }
    return actionId ? "启动" : "";
  }

  setTaskButtonsEnabled(enabledMap) {
    this.refs.taskButtons.forEach((button) => {
      const key = button.dataset.taskAction;
      this.setTaskButtonEnabled(key, enabledMap[key] !== false);
    });
  }

  setTaskButtonEnabled(taskAction, enabled) {
    const button = this.refs.taskButtons.find((item) => item.dataset.taskAction === taskAction);
    if (!button) {
      return;
    }
    button.disabled = !enabled;
    if (button.disabled) {
      this.clearTaskButtonFeedback(taskAction);
    }
  }

  setWorkspaceButtonsEnabled(enabledMap) {
    this.refs.workspaceButtons.forEach((button) => {
      const key = button.dataset.workspaceAction;
      button.disabled = enabledMap[key] === false;
    });
  }

  setCabinRemoteButtonsEnabled(enabled) {
    const moveEnabled = typeof enabled === "object"
      ? Boolean(enabled?.move)
      : Boolean(enabled);
    const stopEnabled = typeof enabled === "object"
      ? Boolean(enabled?.stop ?? enabled?.move)
      : Boolean(enabled);
    this.refs.cabinRemoteButtons.forEach((button) => {
      button.disabled = !moveEnabled;
    });
    if (this.refs.cabinRemoteStopButton) {
      this.refs.cabinRemoteStopButton.disabled = !stopEnabled;
    }
    if (this.refs.cabinRemoteAbsoluteMoveButton) {
      this.refs.cabinRemoteAbsoluteMoveButton.disabled = !moveEnabled;
    }
    if (!moveEnabled) {
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

  setVisualDebugTimingSummary({
    singleFrameElapsedMs = null,
    serviceElapsedMs = null,
    releaseFrameCount = null,
    pointCount = null,
  } = {}) {
    if (!this.refs.visualDebugTimingSummary) {
      return;
    }
    const frameCount = releaseFrameCount ?? this.getVisualDebugSettings().stableFrameCount;
    const singleFrameText = Number.isFinite(Number(singleFrameElapsedMs))
      ? Number(singleFrameElapsedMs).toFixed(1)
      : "--";
    const serviceText = Number.isFinite(Number(serviceElapsedMs))
      ? Number(serviceElapsedMs).toFixed(1)
      : "--";
    const pointText = Number.isFinite(Number(pointCount))
      ? String(Number(pointCount))
      : "--";
    this.refs.visualDebugTimingSummary.textContent =
      `单帧=${singleFrameText}ms 服务=${serviceText}ms 释放=${frameCount}帧 点数=${pointText}`;
  }

  setTopicLayerState(state) {
    this.refs.topicLayerMode.value = state.mode;
    this.refs.pointCloudSource.value = state.pointCloudSource;
    this.refs.showRobotToggle.checked = Boolean(state.showRobot);
    this.refs.showAxesToggle.checked = Boolean(state.showAxes);
    this.refs.showPointCloudToggle.checked = Boolean(state.showPointCloud);
    this.refs.showPlanningMarkersToggle.checked = Boolean(state.showPlanningMarkers);
    this.setTfAxisFrameVisibility(state.tfAxisFrameVisibility);
    this.refs.pointSizeRange.value = Number(state.pointSize).toFixed(3);
    this.refs.pointOpacityRange.value = Number(state.pointOpacity).toFixed(2);
    this.setSceneViewMode(state.viewMode);
    this.refs.followOriginToggle.checked = Boolean(state.followOrigin);
    this.refreshCustomSelect(this.refs.topicLayerMode);
    this.refreshCustomSelect(this.refs.pointCloudSource);
    const visibleTfAxes = this.getVisibleTfAxisFrameLabels(state.tfAxisFrameVisibility);
    this.refs.topicLayerSummary.textContent =
      `模式=${getTopicLayerModeLabel(state.mode)} 坐标轴=${state.showAxes ? visibleTfAxes : "总开关关闭"} 点云=${state.showPointCloud ? "开启" : "关闭"} 点云源=${getPointCloudSourceLabel(state.pointCloudSource)} 点大小=${Number(state.pointSize).toFixed(3)} 透明度=${Number(state.pointOpacity).toFixed(2)}`;
  }

  setTfAxisFrameVisibility(frameVisibility = {}) {
    const axesEnabled = Boolean(this.refs.showAxesToggle.checked);
    this.refs.tfAxisFrameToggles.forEach((input) => {
      const frameId = input.dataset.tfAxisFrame;
      input.checked = frameVisibility?.[frameId] !== false;
      input.disabled = !axesEnabled;
      input.closest(".checkbox-field")?.classList.toggle("is-disabled", !axesEnabled);
    });
  }

  getVisibleTfAxisFrameLabels(frameVisibility = {}) {
    const visibleLabels = TF_AXIS_FRAMES
      .filter((frame) => frameVisibility?.[frame.id] !== false)
      .map((frame) => frame.label.replace(/\s+/g, ":"));
    return visibleLabels.length ? visibleLabels.join("/") : "全部关闭";
  }

  renderTopicLayerStats(stats) {
    const unifiedPlanningPointCount = stats.planningPointCount || stats.tiePointCount || 0;
    this.refs.topicLayerStats.innerHTML = `
      <div class="stats-card"><span>滤波点云</span><strong>${stats.filteredWorldCoordCount}</strong></div>
      <div class="stats-card"><span>原始点云</span><strong>${stats.rawWorldCoordCount}</strong></div>
      <div class="stats-card"><span>规划点/绑扎点</span><strong>${unifiedPlanningPointCount}</strong></div>
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
          ${groupEntries.map((entry) => {
            const sourceLabel = entry.registry?.sourceLabel || "未登记";
            const ownerLabel = entry.registry?.ownerNode || "runtime";
            const usageLabel = entry.registry?.usage || "ROS 运行时发现";
            return `
            <li class="topic-inventory-item">
              <div class="topic-inventory-name mono">${entry.name}</div>
              <div class="topic-inventory-type mono">${entry.type}</div>
              <div class="topic-inventory-meta">
                <span class="topic-inventory-source">${sourceLabel}</span>
                <span class="topic-inventory-owner">${ownerLabel}</span>
              </div>
              <div class="topic-inventory-usage">${usageLabel}</div>
            </li>
          `;
          }).join("")}
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

  getRobotHomeCalibrationInputs() {
    return {
      home: {
        x: Number.parseFloat(this.refs.robotHomeX?.value || "0"),
        y: Number.parseFloat(this.refs.robotHomeY?.value || "0"),
        z: Number.parseFloat(this.refs.robotHomeZ?.value || "0"),
      },
    };
  }

  formatMmTriplet(position, waitingText = "等待") {
    if (!position || !CABIN_POSITION_AXES.every(({ id }) => Number.isFinite(Number(position[id])))) {
      return waitingText;
    }
    return CABIN_POSITION_AXES
      .map(({ id, label }) => `${label}=${Math.round(Number(position[id]))}mm`)
      .join(" ");
  }

  setRobotHomeCalibration(calibration, { forceInputs = false } = {}) {
    if (!calibration) {
      if (this.refs.robotHomeSummary) {
        this.refs.robotHomeSummary.textContent = "等待 Home 标定服务。";
      }
      if (this.refs.robotHomeTfSummary) {
        this.refs.robotHomeTfSummary.textContent = "等待 TF 投射状态。";
      }
      return;
    }

    const currentText = calibration.hasCurrentPose
      ? this.formatMmTriplet(calibration.current, "无当前坐标")
      : "等待索驱当前坐标";
    const homeText = this.formatMmTriplet(calibration.home, "未保存 Home");
    if (this.refs.robotHomeSummary) {
      this.refs.robotHomeSummary.textContent = `当前=${currentText} | Home=${homeText}`;
    }

    const cameraText = calibration.hasCameraPose
      ? this.formatMmTriplet(calibration.camera, "无相机投射")
      : "等待 map -> Scepter_depth_frame";
    const groundText = calibration.hasGroundProbe
      ? `${Math.round(Number(calibration.groundProbe.distance || 0))}mm | ${this.formatMmTriplet(calibration.groundProbe, "无地面投射")}`
      : "等待深度最远点";
    if (this.refs.robotHomeTfSummary) {
      this.refs.robotHomeTfSummary.textContent = `相机map=${cameraText} | 深度最远点=${groundText}`;
    }
    if (this.refs.baseToCameraComputed) {
      const baseToCamera = calibration.baseToCamera || {};
      const baseToCameraText = [
        `X=${Number(baseToCamera.x || 0).toFixed(1)}mm`,
        `Y=${Number(baseToCamera.y || 0).toFixed(1)}mm`,
        `Z=${Number(baseToCamera.z || 0).toFixed(1)}mm`,
        `R=${Number(baseToCamera.roll ?? Math.PI).toFixed(4)}`,
        `P=${Number(baseToCamera.pitch || 0).toFixed(4)}`,
        `Y=${Number(baseToCamera.yaw || 0).toFixed(4)}`,
      ].join(" ");
      this.refs.baseToCameraComputed.textContent = `TF自动结果：${baseToCameraText}`;
    }

    const activeElement = document.activeElement;
    const editableRefs = [
      this.refs.robotHomeX,
      this.refs.robotHomeY,
      this.refs.robotHomeZ,
    ];
    if (forceInputs || !editableRefs.includes(activeElement)) {
      this.refs.robotHomeX.value = Number(calibration.home?.x || 0).toFixed(1);
      this.refs.robotHomeY.value = Number(calibration.home?.y || 0).toFixed(1);
      this.refs.robotHomeZ.value = Number(calibration.home?.z || 0).toFixed(1);
    }
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

  setDemoModeState(active, detail = "") {
    const chip = this.rootElement.querySelector('[data-status-id="demoMode"]');
    if (!chip) {
      return;
    }
    const pending = chip.classList.contains("is-pending");
    const level = active ? "success" : "error";
    const actionText = active ? "退出" : "进入";
    chip.className = `system-status-item ${level} is-interactive`;
    chip.classList.toggle("is-pending", pending);
    chip.dataset.statusAction = "toggleDemoMode";
    chip.setAttribute("aria-pressed", active ? "true" : "false");
    chip.setAttribute("aria-label", `演示模式：${actionText}`);
    chip.title = detail || (active ? "演示模式运行中" : "演示模式未启用");
    const actionLabel = chip.querySelector(".system-status-action-label");
    if (actionLabel) {
      actionLabel.textContent = chip.dataset.pendingActionId
        ? this.getPendingActionLabel(chip.dataset.pendingActionId)
        : actionText;
    }
  }

  setStatusChipState(statusId, level, detail) {
    const chip = this.rootElement.querySelector(`[data-status-id="${statusId}"]`);
    if (!chip) {
      return;
    }
    const pending = chip.classList.contains("is-pending");
    chip.className = `system-status-item ${level} is-interactive`;
    chip.classList.toggle("is-pending", pending);
    chip.title = detail || "";
    const actionLabel = chip.querySelector(".system-status-action-label");
    const nextActionMap = {
      ros: level === "success" ? "restartRosStack" : "startRosStack",
      chassis: level === "success" ? "stopCabinSubsystem" : "startCabinSubsystem",
      moduan: level === "success" ? "stopModuanSubsystem" : "startModuanSubsystem",
      visual: level === "success" ? "stopVisualSubsystem" : "startVisualSubsystem",
    };
    const longPressActionMap = {
      chassis: "restartCabinSubsystem",
      moduan: "restartModuanSubsystem",
      visual: "restartVisualSubsystem",
    };
    const nextActionLabelMap = {
      ros: level === "success" ? "重启ROS" : "启动ROS",
      chassis: level === "success" ? "关闭" : "启动",
      moduan: level === "success" ? "关闭" : "启动",
      visual: level === "success" ? "关闭" : "启动",
    };
    const nextAction = nextActionMap[statusId] || "";
    const longPressAction = longPressActionMap[statusId] || "";
    const nextActionLabel = nextActionLabelMap[statusId] || "";
    chip.dataset.statusAction = nextAction;
    chip.dataset.statusLongAction = longPressAction;
    const statusLabel = chip.querySelector(".system-status-label")?.textContent || statusId;
    chip.setAttribute("aria-label", longPressAction
      ? `${statusLabel}：短按${nextActionLabel}，长按重启`
      : `${statusLabel}：${nextActionLabel}`);
    chip.title = longPressAction
      ? `${detail || ""}${detail ? "；" : ""}短按${nextActionLabel}，长按0.8秒重启`
      : detail || "";
    if (actionLabel) {
      actionLabel.textContent = chip.dataset.pendingActionId
        ? this.getPendingActionLabel(chip.dataset.pendingActionId)
        : nextActionLabel;
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
    const hasPosition = CABIN_POSITION_AXES.every(({ id }) => Number.isFinite(position?.[id]));
    if (this.refs.cabinRemoteCurrentPosition) {
      this.refs.cabinRemoteCurrentPosition.innerHTML = CABIN_POSITION_AXES.map(({ id, label }) => {
        const value = Number.isFinite(position?.[id]) ? `${Math.round(position[id])} mm` : "等待索驱 TF…";
        return `
          <div class="cabin-remote-position-item">
            <span class="cabin-remote-position-label">${label}</span>
            <span class="cabin-remote-position-value">${value}</span>
          </div>
        `;
      }).join("");
    }

    if (!this.refs.bottomCabinPosition) {
      return;
    }
    this.bottomCabinPositionTitle = hasPosition
      ? `机器位置：${CABIN_POSITION_AXES.map(({ id, label }) => `${label}=${Math.round(position[id])}mm`).join(" ")}`
      : "等待索驱 TF 更新机器位置";
    this.refs.bottomCabinPosition.dataset.state = hasPosition ? "live" : "waiting";
    this.syncBottomCabinPositionA11y();
    this.refs.bottomCabinPositionAxes.forEach((axisNode) => {
      const axisId = axisNode.dataset.bottomCabinAxis;
      const axis = CABIN_POSITION_AXES.find((item) => item.id === axisId);
      if (!axis) {
        return;
      }
      const { label } = axis;
      const value = Number.isFinite(position?.[axisId]) ? `${Math.round(position[axisId])} mm` : "-- mm";
      axisNode.textContent = `${label} ${value}`;
    });
  }

  setBottomCabinOperationState(operationState) {
    if (!this.refs.bottomCabinPosition) {
      return;
    }
    this.refs.bottomCabinPosition.dataset.operationState = operationState?.state || "blocked";
    this.bottomCabinOperationDetail = operationState?.detail || "索驱不可操作";
    this.syncBottomCabinPositionA11y();
  }

  syncBottomCabinPositionA11y() {
    if (!this.refs.bottomCabinPosition) {
      return;
    }
    const title = `${this.bottomCabinPositionTitle} | ${this.bottomCabinOperationDetail}`;
    this.refs.bottomCabinPosition.title = title;
    this.refs.bottomCabinPosition.setAttribute("aria-label", title);
  }

  setBottomLinearModulePosition(localPosition, globalPosition) {
    if (!this.refs.bottomLinearModulePosition) {
      return;
    }

    const hasLocalPosition = CABIN_POSITION_AXES.every(({ id }) => Number.isFinite(Number(localPosition?.[id])));
    const hasGlobalPosition = CABIN_POSITION_AXES.every(({ id }) => Number.isFinite(Number(globalPosition?.[id])));
    const state = hasLocalPosition && hasGlobalPosition
      ? "live"
      : hasLocalPosition
        ? "partial"
        : "waiting";
    const describePosition = (label, position) => (
      `${label}：${CABIN_POSITION_AXES
        .map(({ id, label: axisLabel }) => `${axisLabel}=${Number.isFinite(Number(position?.[id])) ? Math.round(Number(position[id])) : "--"}mm`)
        .join(" ")}`
    );
    const title = hasLocalPosition
      ? [
        describePosition("线模本地", localPosition),
        hasGlobalPosition ? describePosition("线模全局", globalPosition) : "线模全局：等待 gripper_frame TF",
      ].join("；")
      : "等待线性模组状态和 gripper_frame TF";

    this.refs.bottomLinearModulePosition.dataset.state = state;
    this.refs.bottomLinearModulePosition.title = title;
    this.refs.bottomLinearModulePosition.setAttribute("aria-label", title);
    this.refs.bottomLinearModuleAxes.forEach((axisNode) => {
      const [scopeId, axisId] = (axisNode.dataset.bottomLinearAxis || "").split(":");
      const axis = CABIN_POSITION_AXES.find((item) => item.id === axisId);
      if (!axis) {
        return;
      }
      const position = scopeId === "global" ? globalPosition : localPosition;
      const rawValue = Number(position?.[axisId]);
      const value = Number.isFinite(rawValue) ? `${Math.round(rawValue)} mm` : "-- mm";
      axisNode.textContent = `${axis.label} ${value}`;
    });
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
    this.refs.cabinRemoteStatus.textContent = message || "键位：Q/W/E = Z+/X+/Z-，A/S/D = Y+/X-/Y-，空格 = 暂停。";
    this.refs.cabinRemoteStatus.classList.toggle(
      "cabin-remote-status-active",
      Boolean(this.refs.cabinKeyboardRemoteToggle?.checked),
    );
  }

  setTcpLinearRemoteState(message) {
    if (!this.refs.tcpLinearRemoteCurrentPosition) {
      return;
    }

    const hasState = TCP_LINEAR_POSITION_FIELDS.every((field) => Number.isFinite(Number(message?.[field.key])));
    this.refs.tcpLinearRemoteCurrentPosition.innerHTML = TCP_LINEAR_POSITION_FIELDS.map((field) => {
      const rawValue = Number(message?.[field.key]);
      const formatted = Number.isFinite(rawValue)
        ? `${rawValue.toFixed(1)} ${field.unit}`
        : "等待线模状态…";
      return `
        <div class="tcp-linear-remote-position-item">
          <span class="tcp-linear-remote-position-label">${field.label}</span>
          <span class="tcp-linear-remote-position-value">${formatted}</span>
        </div>
      `;
    }).join("");

    if (!hasState) {
      this.setTcpLinearRemoteStatus("等待线性模组状态；行程：X 0~360mm，Y 0~320mm，Z 0~140mm。");
      return;
    }

    const errorFlags = [
      ["X", message.linear_module_error_flag_X],
      ["Y", message.linear_module_error_flag_Y],
      ["Z", message.linear_module_error_flag_Z],
      ["电机", message.motor_error_flag],
    ].filter(([, flag]) => Number(flag) !== 0);
    if (errorFlags.length) {
      this.setTcpLinearRemoteStatus(`线性模组存在错误位：${errorFlags.map(([label, flag]) => `${label}=${flag}`).join(" ")}`);
      return;
    }

    this.setTcpLinearRemoteStatus("线性模组状态实时更新中，可按按钮做 TCP 步进遥控。");
  }

  setTcpLinearRemoteStatus(message) {
    if (!this.refs.tcpLinearRemoteStatus) {
      return;
    }
    this.refs.tcpLinearRemoteStatus.textContent =
      message || "等待线性模组状态；行程：X 0~360mm，Y 0~320mm，Z 0~140mm。";
  }

  setTcpLinearRemoteButtonsEnabled(enabled) {
    this.refs.tcpLinearRemoteButtons.forEach((button) => {
      button.disabled = !enabled;
      if (!enabled) {
        button.classList.remove("is-active");
      }
    });
    if (this.refs.tcpLinearRemoteStopButton) {
      this.refs.tcpLinearRemoteStopButton.disabled = !enabled;
      if (!enabled) {
        this.refs.tcpLinearRemoteStopButton.classList.remove("is-active");
      }
    }
  }

  flashTcpLinearRemoteButton(directionId) {
    const button = this.refs.tcpLinearRemoteButtons.find((item) => item.dataset.tcpLinearRemoteAxis === directionId);
    if (!button) {
      return;
    }
    button.classList.add("is-active");
    window.setTimeout(() => {
      button.classList.remove("is-active");
    }, 320);
  }

  flashTcpLinearRemoteStopButton() {
    const button = this.refs.tcpLinearRemoteStopButton;
    if (!button) {
      return;
    }
    button.classList.add("is-active");
    window.setTimeout(() => {
      button.classList.remove("is-active");
    }, 520);
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

  setTerminalNotice(message, level = "info") {
    if (!this.refs.terminalEmptyState) {
      return;
    }

    const normalizedMessage = typeof message === "string" ? message.trim() : "";
    if (!normalizedMessage) {
      this.refs.terminalEmptyState.dataset.state = "idle";
      if (this.refs.terminalEmptyTitle) {
        this.refs.terminalEmptyTitle.textContent = "还没有终端会话";
      }
      if (this.refs.terminalEmptySubtitle) {
        this.refs.terminalEmptySubtitle.textContent = "点击右上角加号，或从工具栏打开“终端”后自动创建本机 shell。";
      }
      return;
    }

    this.refs.terminalEmptyState.dataset.state = level;
    if (this.refs.terminalEmptyTitle) {
      this.refs.terminalEmptyTitle.textContent = level === "error" ? "终端启动失败" : "终端准备中";
    }
    if (this.refs.terminalEmptySubtitle) {
      this.refs.terminalEmptySubtitle.textContent = normalizedMessage;
    }
  }

  renderTerminalSessions(sessions, activeSessionId) {
    const items = Array.isArray(sessions) ? sessions : [];
    if (items.length > 0) {
      this.setTerminalNotice(null);
    }
    if (this.refs.terminalEmptyState) {
      this.refs.terminalEmptyState.hidden = items.length > 0;
    }
    if (!this.refs.terminalTabStrip) {
      return;
    }
    this.refs.terminalTabStrip.innerHTML = items.map((session) => `
      <div class="terminal-tab ${session.sessionId === activeSessionId ? "is-active" : ""}" data-state="${escapeHtml(session.state || "connecting")}">
        <button class="terminal-tab-select" type="button" data-terminal-select="${escapeHtml(session.sessionId)}">
          <span class="terminal-tab-label">${escapeHtml(session.label)}</span>
          <span class="terminal-tab-state">${escapeHtml(this.getTerminalStateLabel(session.state))}</span>
        </button>
        <button class="terminal-tab-close" type="button" data-terminal-close="${escapeHtml(session.sessionId)}" title="关闭终端">×</button>
      </div>
    `).join("");
  }

  getHighestFloatingWindowZIndex(excludedPanel = null) {
    let maxZIndex = 20;
    document.querySelectorAll(".floating-panel, .graphical-app-panel").forEach((candidate) => {
      if (candidate === excludedPanel) {
        return;
      }
      const zIndex = Number.parseInt(window.getComputedStyle(candidate).zIndex, 10);
      if (Number.isFinite(zIndex) && zIndex > maxZIndex) {
        maxZIndex = zIndex;
      }
    });
    this.graphicalAppPanelZIndexSeed = Math.max(this.graphicalAppPanelZIndexSeed, maxZIndex);
    return maxZIndex;
  }

  getNextFloatingWindowZIndex(excludedPanel = null) {
    const nextZIndex = this.getHighestFloatingWindowZIndex(excludedPanel) + 1;
    this.graphicalAppPanelZIndexSeed = Math.max(this.graphicalAppPanelZIndexSeed + 1, nextZIndex);
    return this.graphicalAppPanelZIndexSeed;
  }

  getGraphicalAppPanelZIndex(sessionId) {
    const normalizedSessionId = String(sessionId || "");
    const state = this.graphicalAppPanelStates.get(normalizedSessionId) || {};
    const currentZIndex = Number.parseInt(state.zIndex, 10);
    if (Number.isFinite(currentZIndex)) {
      this.graphicalAppPanelZIndexSeed = Math.max(this.graphicalAppPanelZIndexSeed, currentZIndex);
      return currentZIndex;
    }
    const zIndex = this.getNextFloatingWindowZIndex();
    this.graphicalAppPanelStates.set(normalizedSessionId, { ...state, zIndex });
    return zIndex;
  }

  bringGraphicalAppPanelToFront(panel) {
    if (!panel) {
      return;
    }
    const currentZIndex = Number.parseInt(window.getComputedStyle(panel).zIndex, 10);
    const maxOtherZIndex = this.getHighestFloatingWindowZIndex(panel);
    const nextZIndex = Number.isFinite(currentZIndex) && currentZIndex > maxOtherZIndex
      ? currentZIndex
      : this.getNextFloatingWindowZIndex(panel);

    document.querySelectorAll(".floating-panel, .graphical-app-panel").forEach((candidate) => {
      candidate.classList.remove("is-window-active");
    });
    panel.classList.add("is-window-active");
    panel.style.zIndex = String(nextZIndex);
    this.graphicalAppPanelZIndexSeed = Math.max(this.graphicalAppPanelZIndexSeed, nextZIndex);

    const sessionId = panel.dataset.guiSessionId;
    if (sessionId) {
      const state = this.graphicalAppPanelStates.get(sessionId) || {};
      this.graphicalAppPanelStates.set(sessionId, { ...state, zIndex: nextZIndex });
    }
  }

  syncActiveFloatingWindowClass() {
    let activePanel = null;
    let maxZIndex = 20;
    document.querySelectorAll(".floating-panel, .graphical-app-panel").forEach((panel) => {
      const zIndex = Number.parseInt(window.getComputedStyle(panel).zIndex, 10);
      if (Number.isFinite(zIndex) && zIndex >= maxZIndex) {
        maxZIndex = zIndex;
        activePanel = panel;
      }
      panel.classList.remove("is-window-active");
    });
    activePanel?.classList.add("is-window-active");
  }

  renderGraphicalAppSessions(sessions) {
    const items = Array.isArray(sessions) ? sessions : [];
    if (!this.refs.graphicalAppPanelDock) {
      return;
    }
    const activeSessionIds = new Set(items.map((session) => String(session.sessionId || "")));
    this.graphicalAppPanelStates.forEach((_state, sessionId) => {
      if (!activeSessionIds.has(sessionId)) {
        this.graphicalAppPanelStates.delete(sessionId);
      }
    });
    this.refs.graphicalAppPanelDock
      .querySelectorAll(".graphical-app-panel")
      .forEach((panel) => {
        const sessionId = String(panel.dataset.guiSessionId || "");
        if (!activeSessionIds.has(sessionId)) {
          panel.__graphicalAppResizeObserver?.disconnect();
          panel.remove();
        }
      });
    items.forEach((session, index) => {
      this.upsertGraphicalAppPanel(session, index);
    });
    if (items.length > 0) {
      this.syncActiveFloatingWindowClass();
    }
  }

  upsertGraphicalAppPanel(session, index) {
    const sessionId = String(session?.sessionId || "");
    if (!sessionId || !this.refs.graphicalAppPanelDock) {
      return null;
    }
    let panel = this.findGraphicalAppPanel(sessionId);
    if (!panel) {
      const template = document.createElement("template");
      template.innerHTML = this.buildGraphicalAppPanelHtml(session, index).trim();
      panel = template.content.firstElementChild;
      this.refs.graphicalAppPanelDock.appendChild(panel);
    } else {
      this.updateGraphicalAppPanel(panel, session, index);
      this.refs.graphicalAppPanelDock.appendChild(panel);
    }
    this.activateGraphicalAppPanelDrag(panel);
    return panel;
  }

  findGraphicalAppPanel(sessionId) {
    return [...(this.refs.graphicalAppPanelDock?.querySelectorAll(".graphical-app-panel") || [])]
      .find((panel) => panel.dataset.guiSessionId === String(sessionId || "")) || null;
  }

  buildGraphicalAppPanelHtml(session, index) {
    const frameUrl = this.resolveGraphicalAppUrl(session);
    const canRenderFrame = Boolean(frameUrl) && session.state === "ready";
    const sessionId = String(session.sessionId || "");
    const state = this.graphicalAppPanelStates.get(sessionId);
    const iconFallback = this.getGraphicalAppIconFallback(session);
    return `
      <section class="graphical-app-panel ${state?.maximized ? "maximized" : ""}" data-gui-session-id="${escapeHtml(sessionId)}" data-state="${escapeHtml(session.state || "starting")}" style="${this.getGraphicalAppPanelStyle(sessionId, index)}">
        <div class="graphical-app-header">
          <div class="graphical-app-title-row">
            <span class="graphical-app-icon" data-gui-session-icon>${escapeHtml(iconFallback)}</span>
            <div class="graphical-app-title-copy">
              <div class="graphical-app-title">${escapeHtml(session.label || session.command?.[0] || "图形界面")}</div>
              <div class="graphical-app-subtitle">${escapeHtml(this.getGraphicalAppStateLabel(session.state))}</div>
            </div>
          </div>
          <div class="panel-actions">
            <button class="panel-action-btn panel-maximize-btn" type="button" data-gui-session-maximize="${escapeHtml(sessionId)}" title="${state?.maximized ? "还原" : "最大化"}">${state?.maximized ? "❐" : "⛶"}</button>
            <button class="panel-action-btn" type="button" data-gui-session-close="${escapeHtml(sessionId)}" title="关闭图形界面">✕</button>
          </div>
        </div>
        <div class="graphical-app-frame-shell">
          ${canRenderFrame
            ? this.getGraphicalAppFrameMarkup(session, frameUrl)
            : `<div class="graphical-app-placeholder">${escapeHtml(session.message || "图形界面暂不可用。")}</div>`}
        </div>
      </section>
    `;
  }

  updateGraphicalAppPanel(panel, session, index) {
    const sessionId = String(session.sessionId || "");
    const frameUrl = this.resolveGraphicalAppUrl(session);
    const canRenderFrame = Boolean(frameUrl) && session.state === "ready";
    const state = this.graphicalAppPanelStates.get(sessionId);
    panel.dataset.state = session.state || "starting";
    panel.classList.toggle("maximized", Boolean(state?.maximized));
    this.applyGraphicalAppPanelStyle(panel, sessionId, index);
    const title = panel.querySelector(".graphical-app-title");
    const subtitle = panel.querySelector(".graphical-app-subtitle");
    const iconSlot = panel.querySelector("[data-gui-session-icon]");
    const maximizeButton = panel.querySelector("[data-gui-session-maximize]");
    const closeButton = panel.querySelector("[data-gui-session-close]");
    if (title) {
      title.textContent = session.label || session.command?.[0] || "图形界面";
    }
    if (subtitle) {
      subtitle.textContent = this.getGraphicalAppStateLabel(session.state);
    }
    if (iconSlot && !iconSlot.classList.contains("has-image")) {
      iconSlot.textContent = this.getGraphicalAppIconFallback(session);
    }
    if (maximizeButton) {
      maximizeButton.dataset.guiSessionMaximize = sessionId;
      maximizeButton.textContent = state?.maximized ? "❐" : "⛶";
      maximizeButton.title = state?.maximized ? "还原" : "最大化";
    }
    if (closeButton) {
      closeButton.dataset.guiSessionClose = sessionId;
    }
    this.syncGraphicalAppFrameShell(panel, session, frameUrl, canRenderFrame);
  }

  getGraphicalAppPanelStyle(sessionId, index) {
    const offset = Math.min(index, 5) * 28;
    const state = this.graphicalAppPanelStates.get(sessionId);
    const zIndex = this.getGraphicalAppPanelZIndex(sessionId);
    const rect = state?.maximized
      ? this.getGraphicalAppMaximizedRect()
      : state?.restoreRect;
    return rect
      ? `left: ${rect.left}px; top: ${rect.top}px; width: ${rect.width}px; height: ${rect.height}px; right: auto; z-index: ${zIndex};`
      : `top: ${148 + offset}px; right: ${20 + offset}px; z-index: ${zIndex};`;
  }

  applyGraphicalAppPanelStyle(panel, sessionId, index) {
    const offset = Math.min(index, 5) * 28;
    const state = this.graphicalAppPanelStates.get(sessionId);
    const rect = state?.maximized
      ? this.getGraphicalAppMaximizedRect()
      : state?.restoreRect;
    const zIndex = this.getGraphicalAppPanelZIndex(sessionId);
    if (rect) {
      this.applyGraphicalAppPanelRect(panel, rect);
    } else {
      panel.style.left = "";
      panel.style.top = `${148 + offset}px`;
      panel.style.right = `${20 + offset}px`;
      panel.style.bottom = "";
      panel.style.width = "";
      panel.style.height = "";
      panel.style.transform = "";
    }
    panel.style.zIndex = String(zIndex);
  }

  syncGraphicalAppFrameShell(panel, session, frameUrl, canRenderFrame) {
    const frameShell = panel.querySelector(".graphical-app-frame-shell");
    if (!frameShell) {
      return;
    }
    const frame = frameShell.querySelector(".graphical-app-frame");
    if (canRenderFrame) {
      if (!frame) {
        frameShell.innerHTML = this.getGraphicalAppFrameMarkup(session, frameUrl);
        return;
      }
      frame.title = session.label || "图形界面";
      if (!frameShell.querySelector(".graphical-app-focus-catcher")) {
        frame.insertAdjacentHTML("beforebegin", `<div class="graphical-app-focus-catcher" data-gui-focus="${escapeHtml(session.sessionId)}"></div>`);
      }
      return;
    }
    const message = session.message || "图形界面暂不可用。";
    const placeholder = frameShell.querySelector(".graphical-app-placeholder");
    if (placeholder && !frame) {
      placeholder.textContent = message;
      return;
    }
    frameShell.innerHTML = `<div class="graphical-app-placeholder">${escapeHtml(message)}</div>`;
  }

  getGraphicalAppFrameMarkup(session, frameUrl) {
    return `<div class="graphical-app-focus-catcher" data-gui-focus="${escapeHtml(session.sessionId)}"></div><iframe class="graphical-app-frame" title="${escapeHtml(session.label || "图形界面")}" src="${escapeHtml(frameUrl)}"></iframe>`;
  }

  getGraphicalAppIconFallback(session) {
    const source = String(session?.command?.[0] || session?.label || "GUI")
      .replace(/[^a-z0-9]/gi, "")
      .slice(0, 2)
      .toUpperCase();
    return source || "UI";
  }

  getGraphicalAppMaximizedRect() {
    const toolbar = document.querySelector(".top-toolbar");
    const toolbarBottom = toolbar ? toolbar.getBoundingClientRect().bottom : 68;
    const top = Math.max(88, Math.round(toolbarBottom + 12));
    return {
      left: 20,
      top,
      width: Math.max(360, window.innerWidth - 40),
      height: Math.max(260, window.innerHeight - top - 20),
    };
  }

  toggleGraphicalAppMaximized(sessionId) {
    if (!sessionId) {
      return;
    }
    const panel = this.refs.graphicalAppPanelDock?.querySelector(`[data-gui-session-id="${CSS.escape(sessionId)}"]`);
    if (!panel) {
      return;
    }
    const currentState = this.graphicalAppPanelStates.get(sessionId) || {};
    const button = panel.querySelector("[data-gui-session-maximize]");
    if (!currentState.maximized) {
      const rect = panel.getBoundingClientRect();
      const restoreRect = {
        left: Math.round(rect.left),
        top: Math.round(rect.top),
        width: Math.round(rect.width),
        height: Math.round(rect.height),
      };
      const target = this.getGraphicalAppMaximizedRect();
      this.graphicalAppPanelStates.set(sessionId, { ...currentState, maximized: true, restoreRect });
      this.applyGraphicalAppPanelRect(panel, target);
      panel.classList.add("maximized");
      if (button) {
        button.textContent = "❐";
        button.title = "还原";
      }
      this.resizeGraphicalAppFrame(panel);
      return;
    }

    const restoreRect = currentState.restoreRect || {
      left: 40,
      top: 148,
      width: Math.min(920, window.innerWidth - 40),
      height: Math.min(640, window.innerHeight - 180),
    };
    this.graphicalAppPanelStates.set(sessionId, { ...currentState, maximized: false, restoreRect });
    this.applyGraphicalAppPanelRect(panel, restoreRect);
    panel.classList.remove("maximized");
    if (button) {
      button.textContent = "⛶";
      button.title = "最大化";
    }
    this.resizeGraphicalAppFrame(panel);
  }

  applyGraphicalAppPanelRect(panel, rect) {
    panel.style.left = `${rect.left}px`;
    panel.style.top = `${rect.top}px`;
    panel.style.width = `${rect.width}px`;
    panel.style.height = `${rect.height}px`;
    panel.style.right = "auto";
    panel.style.bottom = "auto";
    panel.style.transform = "";
  }

  handleGraphicalAppWindowResize() {
    this.graphicalAppPanelStates.forEach((state, sessionId) => {
      if (!state?.maximized) {
        return;
      }
      const panel = this.refs.graphicalAppPanelDock?.querySelector(`[data-gui-session-id="${CSS.escape(sessionId)}"]`);
      if (!panel) {
        return;
      }
      this.applyGraphicalAppPanelRect(panel, this.getGraphicalAppMaximizedRect());
      this.resizeGraphicalAppFrame(panel);
    });
  }

  observeGraphicalAppPanelResize(panel) {
    if (!panel || panel.__graphicalAppResizeObserver || typeof ResizeObserver !== "function") {
      return;
    }
    const frameShell = panel.querySelector(".graphical-app-frame-shell");
    if (!frameShell) {
      return;
    }
    const resizeObserver = new ResizeObserver(() => {
      this.resizeGraphicalAppFrame(panel);
    });
    resizeObserver.observe(frameShell);
    panel.__graphicalAppResizeObserver = resizeObserver;
  }

  resizeGraphicalAppFrame(panel) {
    const frame = panel.querySelector(".graphical-app-frame");
    if (!frame) {
      return;
    }
    frame.style.width = "100%";
    frame.style.height = "100%";
  }

  activateGraphicalAppPanelDrag(panel) {
    const header = panel.querySelector(".graphical-app-header");
    if (!header) {
      return;
    }

    const bringToFront = () => this.bringGraphicalAppPanelToFront(panel);

    const wireFocusCatcher = () => {
      const focusCatcher = panel.querySelector(".graphical-app-focus-catcher");
      if (!focusCatcher || focusCatcher.__graphicalAppFocusCatcherWired) {
        return;
      }
      focusCatcher.__graphicalAppFocusCatcherWired = true;
      focusCatcher.addEventListener("pointerdown", (event) => {
        bringToFront();
        event.preventDefault();
        event.stopPropagation();
      });
    };

    const wireFrameFocus = () => {
      const frame = panel.querySelector(".graphical-app-frame");
      if (!frame) {
        return;
      }
      const wireFrameWindow = () => {
        try {
          const frameWindow = frame.contentWindow;
          if (!frameWindow || frame.__graphicalAppFrameWindow === frameWindow) {
            return;
          }
          frame.__graphicalAppFrameWindow = frameWindow;
          frameWindow.addEventListener("pointerdown", bringToFront, true);
          frameWindow.addEventListener("mousedown", bringToFront, true);
          frameWindow.addEventListener("focus", bringToFront, true);
        } catch (_error) {
          // The Xpra proxy is same-origin; if a browser blocks access, the outer card still activates.
        }
      };
      if (!frame.__graphicalAppFrameWired) {
        frame.__graphicalAppFrameWired = true;
        frame.addEventListener("focus", bringToFront);
        frame.addEventListener("load", () => {
          wireFrameWindow();
          this.resizeGraphicalAppFrame(panel);
        });
        this.resizeGraphicalAppFrame(panel);
      }
      wireFrameWindow();
    };

    if (panel.__graphicalAppPanelDragActivated) {
      const sessionId = panel.dataset.guiSessionId;
      const state = sessionId ? this.graphicalAppPanelStates.get(sessionId) : null;
      panel.classList.toggle("maximized", Boolean(state?.maximized));
      wireFocusCatcher();
      wireFrameFocus();
      this.observeGraphicalAppPanelResize(panel);
      return;
    }
    panel.__graphicalAppPanelDragActivated = true;

    const normalizePanelGeometry = () => {
      const rect = panel.getBoundingClientRect();
      panel.style.left = `${rect.left}px`;
      panel.style.top = `${rect.top}px`;
      panel.style.width = `${rect.width}px`;
      panel.style.height = `${rect.height}px`;
      panel.style.right = "auto";
      panel.style.bottom = "auto";
    };

    const clampToViewport = () => {
      const rect = panel.getBoundingClientRect();
      const maxLeft = Math.max(20, window.innerWidth - rect.width - 20);
      const maxTop = Math.max(88, window.innerHeight - rect.height - 20);
      const nextLeft = Math.min(Math.max(rect.left, 20), maxLeft);
      const nextTop = Math.min(Math.max(rect.top, 88), maxTop);
      panel.style.left = `${nextLeft}px`;
      panel.style.top = `${nextTop}px`;
    };

    header.addEventListener("mousedown", (event) => {
      if (event.button !== 0) {
        return;
      }
      const sessionId = panel.dataset.guiSessionId;
      const state = this.graphicalAppPanelStates.get(sessionId);
      panel.classList.toggle("maximized", Boolean(state?.maximized));
      bringToFront();
      if (state?.maximized) {
        return;
      }
      if (event.target.closest(".panel-action-btn")) {
        return;
      }

      normalizePanelGeometry();
      bringToFront();
      const dragOffsetX = event.clientX - panel.offsetLeft;
      const dragOffsetY = event.clientY - panel.offsetTop;
      document.body.style.userSelect = "none";

      const handleMove = (moveEvent) => {
        const maxLeft = Math.max(20, window.innerWidth - panel.offsetWidth - 20);
        const maxTop = Math.max(88, window.innerHeight - panel.offsetHeight - 20);
        const nextLeft = Math.min(Math.max(moveEvent.clientX - dragOffsetX, 20), maxLeft);
        const nextTop = Math.min(Math.max(moveEvent.clientY - dragOffsetY, 88), maxTop);
        panel.style.left = `${nextLeft}px`;
        panel.style.top = `${nextTop}px`;
      };

      const handleUp = () => {
        document.body.style.userSelect = "";
        window.removeEventListener("mousemove", handleMove);
        window.removeEventListener("mouseup", handleUp);
        clampToViewport();
        const rect = panel.getBoundingClientRect();
        const sessionId = panel.dataset.guiSessionId;
        if (sessionId) {
          const currentState = this.graphicalAppPanelStates.get(sessionId) || {};
          this.graphicalAppPanelStates.set(sessionId, {
            ...currentState,
            maximized: false,
            restoreRect: {
              left: Math.round(rect.left),
              top: Math.round(rect.top),
              width: Math.round(rect.width),
              height: Math.round(rect.height),
            },
          });
        }
      };

      window.addEventListener("mousemove", handleMove);
      window.addEventListener("mouseup", handleUp);
      event.preventDefault();
    });

    panel.addEventListener("pointerdown", bringToFront);
    panel.addEventListener("mousedown", bringToFront);
    wireFocusCatcher();
    wireFrameFocus();
    const sessionId = panel.dataset.guiSessionId;
    const state = sessionId ? this.graphicalAppPanelStates.get(sessionId) : null;
    panel.classList.toggle("maximized", Boolean(state?.maximized));
    this.observeGraphicalAppPanelResize(panel);
  }

  resolveGraphicalAppUrl(session) {
    if (session?.url) {
      return session.url;
    }
    if (!session?.webPort) {
      return "";
    }
    const proxyPath = `/api/gui/proxy/${encodeURIComponent(session.sessionId)}/`;
    const params = new URLSearchParams({
      path: proxyPath,
      submit: "true",
      embed_instance: this.graphicalAppEmbedInstanceId,
      floating_menu: "0",
      sharing: "1",
      offscreen: "0",
    });
    return `${proxyPath}index.html?${params.toString()}`;
  }

  getGraphicalAppStateLabel(state) {
    switch (state) {
      case "ready":
        return "在线";
      case "error":
        return "启动失败";
      case "closed":
        return "已关闭";
      default:
        return "启动中";
    }
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
        <div class="log-meta mono">${this.escapeHtml(entry.timestamp)}</div>
        <div>${this.escapeHtml(entry.message)}</div>
      </li>
    `).join("");
  }

  renderVisualDebugLogs(logs) {
    if (!this.refs.visualDebugLogList) {
      return;
    }
    if (!logs.length) {
      this.refs.visualDebugLogList.innerHTML = `<li class="visual-debug-log-item info">暂无视觉调试记录。</li>`;
      return;
    }
    this.refs.visualDebugLogList.innerHTML = logs.map((entry) => `
      <li class="visual-debug-log-item ${entry.level}">
        <div class="log-meta mono">${this.escapeHtml(entry.timestamp)}</div>
        <div>${this.escapeHtml(entry.message)}</div>
      </li>
    `).join("");
  }

  renderSettingsLayerLogs(groups) {
    if (!this.refs.settingsLayerLogList) {
      return;
    }

    const layerGroups = Array.isArray(groups) ? groups : [];
    if (!layerGroups.length) {
      this.refs.settingsLayerLogList.innerHTML = `
        <div class="settings-layer-log-empty">暂无日志。</div>
      `;
      return;
    }

    this.refs.settingsLayerLogList.innerHTML = layerGroups.map((group) => `
      <section class="settings-layer-log-group" data-layer-log-group="${this.escapeHtml(group.id)}">
        <div class="settings-layer-log-group-header">
          <span>${this.escapeHtml(group.label)}</span>
          <small>${group.nodes.filter((node) => node.latest).length}/${group.nodes.length} 个节点有日志</small>
        </div>
        <div class="settings-layer-log-node-list">
          ${group.nodes.map((node) => {
            const expanded = this.layerLogExpanded.has(node.nodeName);
            const latest = node.latest;
            const level = latest?.level || "info";
            return `
              <article class="settings-layer-log-card ${latest ? "" : "is-waiting"} ${expanded ? "is-expanded" : ""} ${level}">
                <button
                  class="settings-layer-log-card-header"
                  type="button"
                  data-layer-log-node="${node.nodeName}"
                  aria-expanded="${expanded ? "true" : "false"}"
                >
                  <span class="settings-layer-log-node-name mono">${this.escapeHtml(node.nodeName)}</span>
                  <span class="settings-layer-log-node-label">${this.escapeHtml(node.label)}</span>
                  <span class="settings-layer-log-node-time mono">${latest ? this.escapeHtml(latest.timestamp) : "等待日志"}</span>
                </button>
                <div class="settings-layer-log-latest">
                  ${latest ? this.escapeHtml(latest.content) : "该节点还没有收到日志。"}
                </div>
                <ul class="settings-layer-log-history" ${expanded ? "" : "hidden"}>
                  ${(node.history || []).map((entry) => `
                    <li class="settings-layer-log-history-item ${entry.level}">
                      <span class="settings-layer-log-history-time mono">${this.escapeHtml(entry.timestamp)}</span>
                      <span class="settings-layer-log-history-message">${this.escapeHtml(entry.content)}</span>
                    </li>
                  `).join("")}
                </ul>
              </article>
            `;
          }).join("")}
        </div>
      </section>
    `).join("");
    this.bindSettingsLayerLogToggles();
  }

  bindSettingsLayerLogToggles() {
    this.refs.settingsLayerLogList
      ?.querySelectorAll("[data-layer-log-node]")
      .forEach((button) => {
        button.addEventListener("click", () => {
          this.toggleSettingsLayerLogNode(button.dataset.layerLogNode || "");
        });
      });
  }

  toggleSettingsLayerLogNode(nodeName) {
    if (!nodeName) {
      return;
    }
    if (this.layerLogExpanded.has(nodeName)) {
      this.layerLogExpanded.delete(nodeName);
    } else {
      this.layerLogExpanded.add(nodeName);
    }

    const trigger = [...(this.refs.settingsLayerLogList?.querySelectorAll("[data-layer-log-node]") || [])]
      .find((button) => button.dataset.layerLogNode === nodeName);
    const card = trigger?.closest(".settings-layer-log-card");
    const history = card?.querySelector(".settings-layer-log-history");
    const expanded = this.layerLogExpanded.has(nodeName);
    card?.classList.toggle("is-expanded", expanded);
    trigger?.setAttribute("aria-expanded", expanded ? "true" : "false");
    if (history) {
      history.hidden = !expanded;
    }
  }

  escapeHtml(value) {
    return String(value ?? "")
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;")
      .replace(/'/g, "&#39;");
  }

}
