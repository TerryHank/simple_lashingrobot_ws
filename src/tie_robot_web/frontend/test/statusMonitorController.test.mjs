import assert from "node:assert/strict";

import { ROSLIB } from "../src/vendor/roslib.js";
import { StatusMonitorController } from "../src/controllers/StatusMonitorController.js";
import { TOPICS } from "../src/config/topicRegistry.js";

const topicInstances = [];

class FakeTopic {
  constructor(options) {
    Object.assign(this, options);
    this.listener = null;
    this.unsubscribed = false;
    topicInstances.push(this);
  }

  subscribe(listener) {
    this.listener = listener;
  }

  unsubscribe() {
    this.unsubscribed = true;
  }

  emit(message) {
    this.listener?.(message);
  }
}

ROSLIB.Topic = FakeTopic;

const batteryVoltages = [];
const logs = [];
const statusChanges = [];
const alarmStates = [];
const lightStates = [];

const controller = new StatusMonitorController({
  onBatteryVoltage: (voltage) => batteryVoltages.push(voltage),
  onLightState: (enabled) => lightStates.push(enabled),
  onLog: (message, level) => logs.push({ message, level }),
  onStatusChip: (statusId, level, detail) => statusChanges.push({ statusId, level, detail }),
  onAlarmState: (alarms) => alarmStates.push(alarms),
});

controller.start({ isConnected: true });
logs.length = 0;
statusChanges.length = 0;
batteryVoltages.length = 0;
alarmStates.length = 0;
lightStates.length = 0;

const telemetryTopic = topicInstances.find((topic) => topic.name === TOPICS.control.linearModuleState);
assert.ok(telemetryTopic, "telemetry topic should be subscribed");

telemetryTopic.emit({
  robot_battery_voltage: 52.9,
});

assert.deepEqual(batteryVoltages, [52.9]);
assert.deepEqual(logs, []);
assert.deepEqual(statusChanges, []);
assert.deepEqual(lightStates, []);

telemetryTopic.emit({
  light_state: true,
});

telemetryTopic.emit({
  light_state: false,
});

assert.deepEqual(lightStates, [true, false]);

telemetryTopic.emit({
  robot_battery_voltage: 52.9,
  linear_module_error_flag_X: 1,
  linear_module_error_flag_Y: 1,
  linear_module_error_flag_Z: 0,
  motor_error_flag: 1,
});

assert.deepEqual(alarmStates.at(-1), ["X轴异常", "Y轴异常", "旋转电机异常"]);

const diagnosticsTopic = topicInstances.find((topic) => topic.name === TOPICS.process.diagnostics);
assert.ok(diagnosticsTopic, "diagnostics topic should be subscribed");

diagnosticsTopic.emit({
  status: [
    {
      hardware_id: "tie_robot/chassis_driver",
      level: 0,
      message: "索驱驱动已连接",
      values: [
        { key: "device_alarm", value: "1" },
        { key: "internal_calc_error", value: "0" },
      ],
    },
    {
      hardware_id: "tie_robot/moduan_driver",
      level: 0,
      message: "末端驱动已连接",
      values: [
        { key: "error_x", value: "1" },
        { key: "error_y", value: "0" },
        { key: "error_z", value: "1" },
        { key: "error_lashing", value: "1" },
        { key: "error_motor", value: "0" },
      ],
    },
  ],
});

assert.deepEqual(alarmStates.at(-1), [
  "X轴异常",
  "Y轴异常",
  "旋转电机异常",
  "索驱设备报警",
  "Z轴异常",
  "绑扎枪报警",
]);
