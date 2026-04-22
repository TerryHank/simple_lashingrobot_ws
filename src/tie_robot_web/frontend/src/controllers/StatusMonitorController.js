import { STATUS_MONITORS } from "../config/statusMonitorCatalog.js";
import { ROSLIB } from "../vendor/roslib.js";

const MODUAN_TELEMETRY_TOPIC = "/moduan/moduan_gesture_data";
const MODUAN_TELEMETRY_TYPE = "tie_robot_msgs/linear_module_upload";

function classifyNumericStatus(value) {
  if (!Number.isFinite(value)) {
    return "warn";
  }
  if (value < 0) {
    return "error";
  }
  if (value > 0) {
    return "success";
  }
  return "info";
}

export class StatusMonitorController {
  constructor(callbacks = {}) {
    this.callbacks = callbacks;
    this.subscriptions = [];
    this.lastValues = new Map();
  }

  setConnectionState(level, detail) {
    this.callbacks.onStatusChip?.("ros", level, detail);
  }

  start(ros) {
    this.stop();
    STATUS_MONITORS.filter((item) => item.kind !== "connection").forEach((monitor) => {
      const topic = new ROSLIB.Topic({
        ros,
        name: monitor.topic,
        messageType: monitor.messageType,
      });
      topic.subscribe((message) => {
        const rawValue = Number(message?.data);
        const level = classifyNumericStatus(rawValue);
        const detail = `${monitor.label}: ${Number.isFinite(rawValue) ? rawValue : "无效值"}`;
        this.callbacks.onStatusChip?.(monitor.id, level, detail);
        if (this.lastValues.get(monitor.id) !== rawValue) {
          this.callbacks.onLog?.(`状态变化 ${monitor.label} -> ${detail}`, level);
          this.lastValues.set(monitor.id, rawValue);
        }
      });
      this.subscriptions.push(topic);
    });

    const telemetryTopic = new ROSLIB.Topic({
      ros,
      name: MODUAN_TELEMETRY_TOPIC,
      messageType: MODUAN_TELEMETRY_TYPE,
    });
    telemetryTopic.subscribe((message) => {
      const voltage = Number(message?.robot_battery_voltage);
      this.callbacks.onBatteryVoltage?.(voltage);
      if (this.lastValues.get("robot_battery_voltage") !== voltage) {
        const detail = Number.isFinite(voltage) && voltage > 0
          ? `机器人电压 ${voltage.toFixed(1)}V`
          : "机器人电压无效";
        this.callbacks.onLog?.(`状态变化 电压 -> ${detail}`, Number.isFinite(voltage) && voltage > 0 ? "info" : "warn");
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
    this.lastValues.delete("robot_battery_voltage");
    this.callbacks.onBatteryVoltage?.(Number.NaN);
  }
}
