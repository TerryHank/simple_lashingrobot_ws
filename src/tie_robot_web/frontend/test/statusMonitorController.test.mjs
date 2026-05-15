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

assert.deepEqual(alarmStates, [], "线性模组报警不应进入顶部连接报警汇总");
assert.deepEqual(
  statusChanges.filter((change) => change.statusId === "moduan").at(-1),
  {
    statusId: "moduan",
    level: "warn",
    detail: "末端报警：X轴异常、Y轴异常、旋转电机异常",
  },
  "线性模组报警应让末端状态胶囊变黄",
);
assert.deepEqual(
  logs.filter((entry) => entry.message.includes("状态变化 moduan")).at(-1),
  {
    message: "状态变化 moduan -> 末端报警：X轴异常、Y轴异常、旋转电机异常",
    level: "warn",
  },
  "线性模组报警详情应写入前端日志",
);

logs.length = 0;
statusChanges.length = 0;
alarmStates.length = 0;

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

assert.deepEqual(alarmStates, [], "诊断报警不应进入顶部连接报警汇总");
assert.deepEqual(
  statusChanges.filter((change) => change.statusId === "chassis").at(-1),
  {
    statusId: "chassis",
    level: "warn",
    detail: "索驱报警：索驱设备报警",
  },
  "索驱诊断报警应让索驱状态胶囊变黄",
);
assert.equal(
  statusChanges.filter((change) => change.statusId === "moduan").at(-1)?.level,
  "warn",
  "末端诊断报警应让末端状态胶囊保持黄色",
);
assert.match(
  logs.filter((entry) => entry.message.includes("状态变化 chassis")).at(-1)?.message || "",
  /索驱报警：索驱设备报警/,
  "索驱诊断报警详情应写入前端日志",
);

statusChanges.length = 0;
logs.length = 0;

assert.equal(
  controller.clearLayerAlarmState("moduan"),
  true,
  "应支持只清除末端层报警",
);
assert.deepEqual(
  statusChanges.filter((change) => change.statusId === "moduan").at(-1),
  {
    statusId: "moduan",
    level: "success",
    detail: "末端报警已清除",
  },
  "清除末端层报警后，末端胶囊应回到清除态",
);
assert.deepEqual(
  statusChanges.filter((change) => change.statusId === "chassis").at(-1),
  undefined,
  "清除末端层报警不应改动索驱胶囊",
);
assert.deepEqual(
  logs.filter((entry) => entry.message.includes("状态变化 moduan")).at(-1),
  {
    message: "状态变化 moduan -> 末端报警已清除",
    level: "success",
  },
  "清除末端层报警应写入前端日志",
);

statusChanges.length = 0;
logs.length = 0;

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
        { key: "error_z", value: "0" },
        { key: "error_lashing", value: "0" },
        { key: "error_motor", value: "0" },
      ],
    },
  ],
});
assert.deepEqual(
  statusChanges.filter((change) => change.statusId === "moduan").at(-1),
  {
    statusId: "moduan",
    level: "warn",
    detail: "末端报警：X轴异常",
  },
  "底层仍在上报报警时，下一帧诊断应重新点亮该层报警",
);

const originalDateNow = Date.now;
try {
  topicInstances.length = 0;
  logs.length = 0;
  statusChanges.length = 0;
  batteryVoltages.length = 0;
  alarmStates.length = 0;
  lightStates.length = 0;

  let fakeNow = 1000;
  Date.now = () => fakeNow;
  const staleController = new StatusMonitorController({
    onBatteryVoltage: (voltage) => batteryVoltages.push(voltage),
    onLightState: (enabled) => lightStates.push(enabled),
    onLog: (message, level) => logs.push({ message, level }),
    onStatusChip: (statusId, level, detail) => statusChanges.push({ statusId, level, detail }),
    onAlarmState: (alarms) => alarmStates.push(alarms),
  });
  staleController.start({ isConnected: true });
  logs.length = 0;
  statusChanges.length = 0;
  alarmStates.length = 0;

  const staleDiagnosticsTopic = topicInstances.find((topic) => topic.name === TOPICS.process.diagnostics);
  assert.ok(staleDiagnosticsTopic, "diagnostics topic should be subscribed for stale timing checks");

  staleDiagnosticsTopic.emit({
    status: [
      {
        hardware_id: "tie_robot/chassis_driver",
        level: 0,
        message: "索驱驱动已连接",
        values: [],
      },
    ],
  });
  statusChanges.length = 0;

  fakeNow += 5000;
  staleDiagnosticsTopic.emit({
    status: [
      {
        hardware_id: "tie_robot/visual_algorithm",
        level: 0,
        message: "视觉算法运行中",
        values: [],
      },
    ],
  });

  assert.deepEqual(
    statusChanges.filter((change) => change.statusId === "chassis").at(-1),
    { statusId: "chassis", level: "success", detail: "索驱驱动已连接" },
    "索驱诊断允许跨过一次5秒底层请求窗口，避免任务恢复期误报超时",
  );

  fakeNow = 14001;
  staleDiagnosticsTopic.emit({
    status: [
      {
        hardware_id: "tie_robot/visual_algorithm",
        level: 0,
        message: "视觉算法运行中",
        values: [],
      },
    ],
  });

  assert.deepEqual(
    statusChanges.filter((change) => change.statusId === "chassis").at(-1),
    { statusId: "chassis", level: "warn", detail: "索驱状态超时" },
    "索驱诊断长时间不上报时仍然要提示超时",
  );
  staleController.stop();
} finally {
  Date.now = originalDateNow;
}

topicInstances.length = 0;
logs.length = 0;
statusChanges.length = 0;
batteryVoltages.length = 0;
alarmStates.length = 0;
lightStates.length = 0;

const visualAlarmController = new StatusMonitorController({
  onBatteryVoltage: (voltage) => batteryVoltages.push(voltage),
  onLightState: (enabled) => lightStates.push(enabled),
  onLog: (message, level) => logs.push({ message, level }),
  onStatusChip: (statusId, level, detail) => statusChanges.push({ statusId, level, detail }),
  onAlarmState: (alarms) => alarmStates.push(alarms),
});
visualAlarmController.start({ isConnected: true });
logs.length = 0;
statusChanges.length = 0;
alarmStates.length = 0;

const visualAlarmDiagnosticsTopic = topicInstances.find((topic) => topic.name === TOPICS.process.diagnostics);
assert.ok(visualAlarmDiagnosticsTopic, "diagnostics topic should be subscribed for visual alarm checks");

visualAlarmDiagnosticsTopic.emit({
  status: [
    {
      hardware_id: "tie_robot/visual_algorithm",
      level: 2,
      message: "视觉算法异常",
      values: [
        { key: "transport_state", value: "algorithm_error" },
        { key: "failure_detail", value: "Surface-DP失败：所选扫描底图横纵线族不足" },
      ],
    },
  ],
});

assert.deepEqual(
  statusChanges.filter((change) => change.statusId === "visual").at(-1),
  {
    statusId: "visual",
    level: "warn",
    detail: "视觉算法异常：Surface-DP失败：所选扫描底图横纵线族不足",
  },
  "视觉算法诊断 ERROR 应让原视觉按钮变黄，详情只保留给日志",
);
assert.deepEqual(
  alarmStates,
  [],
  "视觉算法异常不应进入顶部连接报警汇总",
);
assert.deepEqual(
  logs.filter((entry) => entry.message.includes("状态变化 visual")).at(-1),
  {
    message: "状态变化 visual -> 视觉算法异常：Surface-DP失败：所选扫描底图横纵线族不足",
    level: "warn",
  },
  "视觉算法异常详情应写入前端日志",
);

visualAlarmDiagnosticsTopic.emit({
  status: [
    {
      hardware_id: "tie_robot/visual_algorithm",
      level: 0,
      message: "视觉算法运行中",
      values: [
        { key: "transport_state", value: "running" },
        { key: "failure_detail", value: "" },
      ],
    },
  ],
});

assert.deepEqual(
  alarmStates,
  [],
  "视觉算法恢复 OK 后应继续保持顶部报警汇总为空",
);
visualAlarmController.stop();
