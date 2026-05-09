import { STATUS_MONITORS } from "../config/statusMonitorCatalog.js";
import { MESSAGE_TYPES, TOPICS } from "../config/topicRegistry.js";
import { ROSLIB } from "../vendor/roslib.js";

const DEFAULT_DIAGNOSTIC_STALE_MS = 3000;

const MODUAN_ALARM_VALUE_LABELS = [
  ["error_x", "X轴异常"],
  ["error_y", "Y轴异常"],
  ["error_z", "Z轴异常"],
  ["error_lashing", "绑扎枪报警"],
  ["error_motor", "旋转电机异常"],
];

const LINEAR_MODULE_ALARM_VALUE_LABELS = [
  ["linear_module_error_flag_X", "X轴异常"],
  ["linear_module_error_flag_Y", "Y轴异常"],
  ["linear_module_error_flag_Z", "Z轴异常"],
  ["motor_error_flag", "旋转电机异常"],
];

const CABIN_ALARM_VALUE_LABELS = [
  ["device_alarm", "索驱设备报警"],
  ["internal_calc_error", "索驱内部计算异常"],
];

function uniqueLabels(labels) {
  const seen = new Set();
  return labels.filter((label) => {
    const normalized = String(label || "").trim();
    if (!normalized || seen.has(normalized)) {
      return false;
    }
    seen.add(normalized);
    return true;
  });
}

function isAlarmValue(value) {
  if (typeof value === "boolean") {
    return value;
  }
  if (typeof value === "number") {
    return Number.isFinite(value) && value !== 0;
  }
  const normalized = String(value ?? "").trim().toLowerCase();
  if (!normalized) {
    return false;
  }
  if (["true", "yes", "on", "alarm", "error", "fault"].includes(normalized)) {
    return true;
  }
  const numeric = Number(normalized);
  return Number.isFinite(numeric) && numeric !== 0;
}

function normalizeOptionalBoolean(value) {
  if (value === undefined || value === null) {
    return null;
  }
  if (typeof value === "boolean") {
    return value;
  }
  if (typeof value === "number") {
    return Number.isFinite(value) ? value !== 0 : null;
  }
  const normalized = String(value).trim().toLowerCase();
  if (!normalized) {
    return null;
  }
  if (["true", "yes", "on", "1"].includes(normalized)) {
    return true;
  }
  if (["false", "no", "off", "0"].includes(normalized)) {
    return false;
  }
  return null;
}

function collectAlarmLabelsFromObject(source, valueLabels) {
  return uniqueLabels(valueLabels
    .filter(([key]) => isAlarmValue(source?.[key]))
    .map(([, label]) => label));
}

function diagnosticValuesToObject(status) {
  return (Array.isArray(status?.values) ? status.values : []).reduce((accumulator, item) => {
    if (item?.key) {
      accumulator[item.key] = item.value;
    }
    return accumulator;
  }, {});
}

function collectDiagnosticAlarmLabels(status, monitorId) {
  const values = diagnosticValuesToObject(status);
  if (monitorId === "moduan") {
    return collectAlarmLabelsFromObject(values, MODUAN_ALARM_VALUE_LABELS);
  }
  if (monitorId === "chassis") {
    return collectAlarmLabelsFromObject(values, CABIN_ALARM_VALUE_LABELS);
  }
  return [];
}

function collectLinearModuleAlarmLabels(message) {
  return collectAlarmLabelsFromObject(message, LINEAR_MODULE_ALARM_VALUE_LABELS);
}

function diagnosticLevelToUiLevel(level) {
  switch (Number(level)) {
    case 0:
      return "success";
    case 1:
      return "warn";
    case 2:
      return "error";
    case 3:
      return "warn";
    default:
      return "warn";
  }
}

function formatDiagnosticDetail(status) {
  if (!status) {
    return "状态未上报";
  }
  const base = status.message || "状态未上报";
  const values = Array.isArray(status.values) ? status.values : [];
  const connectionState = values.find((item) => item?.key === "transport_state")?.value;
  const failureDetail = values.find((item) => item?.key === "failure_detail")?.value;
  const transportError = values.find((item) => item?.key === "transport_error")?.value;
  const detailSuffix = failureDetail || transportError || connectionState;
  return detailSuffix ? `${base}：${detailSuffix}` : base;
}

export class StatusMonitorController {
  constructor(callbacks = {}) {
    this.callbacks = callbacks;
    this.subscriptions = [];
    this.lastValues = new Map();
    this.diagnosticCache = new Map();
    this.alarmSources = new Map();
  }

  setConnectionState(level, detail) {
    this.callbacks.onStatusChip?.("ros", level, detail);
  }

  emitStatus(statusId, level, detail, rawValue) {
    this.callbacks.onStatusChip?.(statusId, level, detail);
    if (this.lastValues.get(statusId) !== rawValue) {
      this.callbacks.onLog?.(`状态变化 ${statusId} -> ${detail}`, level);
      this.lastValues.set(statusId, rawValue);
    }
  }

  setAlarmLabels(sourceId, labels = []) {
    const normalizedLabels = uniqueLabels(labels);
    if (normalizedLabels.length > 0) {
      this.alarmSources.set(sourceId, normalizedLabels);
    } else {
      this.alarmSources.delete(sourceId);
    }
    this.emitAlarmState();
  }

  emitAlarmState() {
    const labels = uniqueLabels([...this.alarmSources.values()].flat());
    this.callbacks.onAlarmState?.(labels);
  }

  clearAlarmState() {
    this.alarmSources.clear();
    this.emitAlarmState();
  }

  start(ros) {
    this.stop();

    const diagnosticsTopic = new ROSLIB.Topic({
      ros,
      name: TOPICS.process.diagnostics,
      messageType: MESSAGE_TYPES.diagnosticArray,
    });
    diagnosticsTopic.subscribe((message) => {
      const statuses = Array.isArray(message?.status) ? message.status : [];
      const now = Date.now();
      statuses.forEach((status) => {
        const hardwareId = status?.hardware_id;
        if (!hardwareId) {
          return;
        }
        this.diagnosticCache.set(hardwareId, {
          status,
          receivedAt: now,
        });
      });
      STATUS_MONITORS.filter((item) => item.diagnosticHardwareId).forEach((monitor) => {
        const cached = this.diagnosticCache.get(monitor.diagnosticHardwareId);
        if (!cached) {
          const detail = `${monitor.label}状态未上报`;
          this.emitStatus(monitor.id, "warn", detail, `missing:${detail}`);
          this.setAlarmLabels(`diagnostic:${monitor.id}`, []);
          return;
        }
        const staleMs = monitor.diagnosticStaleMs ?? DEFAULT_DIAGNOSTIC_STALE_MS;
        const stale = now - cached.receivedAt > staleMs;
        const detail = stale
          ? `${monitor.label}状态超时`
          : formatDiagnosticDetail(cached.status);
        const level = stale ? "warn" : diagnosticLevelToUiLevel(cached.status.level);
        this.emitStatus(monitor.id, level, detail, `${cached.status.level}:${detail}`);
        this.setAlarmLabels(
          `diagnostic:${monitor.id}`,
          stale ? [] : collectDiagnosticAlarmLabels(cached.status, monitor.id),
        );
      });
    });
    this.subscriptions.push(diagnosticsTopic);

    const telemetryTopic = new ROSLIB.Topic({
      ros,
      name: TOPICS.control.linearModuleState,
      messageType: MESSAGE_TYPES.linearModuleState,
    });
    telemetryTopic.subscribe((message) => {
      const voltage = Number(message?.robot_battery_voltage);
      this.callbacks.onBatteryVoltage?.(voltage);
      if (this.lastValues.get("robot_battery_voltage") !== voltage) {
        this.lastValues.set("robot_battery_voltage", voltage);
      }
      const lightState = normalizeOptionalBoolean(message?.light_state);
      if (lightState !== null) {
        this.callbacks.onLightState?.(lightState);
      }
      this.setAlarmLabels("telemetry:moduan", collectLinearModuleAlarmLabels(message));
    });
    this.subscriptions.push(telemetryTopic);
  }

  stop() {
    this.subscriptions.forEach((topic) => {
      try {
        topic.unsubscribe();
      } catch {
        // ignore close race
      }
    });
    this.subscriptions = [];
    this.lastValues.clear();
    this.diagnosticCache.clear();
    this.clearAlarmState();
    this.callbacks.onBatteryVoltage?.(Number.NaN);
    this.callbacks.onStatusChip?.("chassis", "warn", "索驱状态未上报");
    this.callbacks.onStatusChip?.("moduan", "warn", "末端状态未上报");
    this.callbacks.onStatusChip?.("visual", "warn", "视觉状态未上报");
  }
}
