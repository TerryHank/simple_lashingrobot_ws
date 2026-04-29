import { STATUS_MONITORS } from "../config/statusMonitorCatalog.js";
import { MESSAGE_TYPES, TOPICS } from "../config/topicRegistry.js";
import { ROSLIB } from "../vendor/roslib.js";

const DIAGNOSTIC_STALE_MS = 3000;

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
          return;
        }
        const stale = now - cached.receivedAt > DIAGNOSTIC_STALE_MS;
        const detail = stale
          ? `${monitor.label}状态超时`
          : formatDiagnosticDetail(cached.status);
        const level = stale ? "warn" : diagnosticLevelToUiLevel(cached.status.level);
        this.emitStatus(monitor.id, level, detail, `${cached.status.level}:${detail}`);
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
        const detail = Number.isFinite(voltage) && voltage > 0
          ? `机器人电压 ${voltage.toFixed(1)}V`
          : "机器人电压无效";
        this.callbacks.onLog?.(
          `状态变化 电压 -> ${detail}`,
          Number.isFinite(voltage) && voltage > 0 ? "info" : "warn",
        );
        this.lastValues.set("robot_battery_voltage", voltage);
      }
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
    this.callbacks.onBatteryVoltage?.(Number.NaN);
    this.callbacks.onStatusChip?.("chassis", "warn", "索驱状态未上报");
    this.callbacks.onStatusChip?.("moduan", "warn", "末端状态未上报");
    this.callbacks.onStatusChip?.("visual", "warn", "视觉状态未上报");
  }
}
