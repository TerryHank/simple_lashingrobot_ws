import { MESSAGE_TYPES, TOPICS } from "./topicRegistry.js";

export const LEGACY_PARAMETER_DEFAULTS = {
  globalX: 0,
  globalY: 0,
  globalZ: 10,
  speed: 300,
  stepX: 370,
  stepY: 320,
  startX: 0,
  startY: 0,
  startZ: 0,
  fixedX: 0,
  fixedY: 0,
  fixedZ: 0,
  fixedTheta: 0,
  fixedSpeed: 30,
};

export const LEGACY_PARAMETER_LABELS = {
  globalX: "全局 X",
  globalY: "全局 Y",
  globalZ: "全局 Z",
  speed: "速度",
  stepX: "步长 X",
  stepY: "步长 Y",
  startX: "起点 X",
  startY: "起点 Y",
  startZ: "起点 Z",
  fixedX: "固定 X",
  fixedY: "固定 Y",
  fixedZ: "固定 Z",
  fixedTheta: "固定角度",
  fixedSpeed: "固定速度",
};

export const LEGACY_COMMANDS = [
  { id: 10, name: "暂停作业", topic: TOPICS.control.interruptStop, type: MESSAGE_TYPES.float32, group: "末端控制" },
  { id: 11, name: "开启绑扎", topic: TOPICS.control.enableLashing, type: MESSAGE_TYPES.float32, group: "末端控制" },
  { id: 12, name: "开启(关闭)跳绑", topic: TOPICS.control.sendOddPoints, type: MESSAGE_TYPES.bool, group: "末端控制" },
  { id: 13, name: "恢复作业", topic: TOPICS.control.handSolveWarn, type: MESSAGE_TYPES.float32, group: "末端控制" },
  { id: 14, name: "开启(关闭)灯光", topic: TOPICS.control.light, type: MESSAGE_TYPES.bool, group: "末端控制" },
  { id: 15, name: "末端回零", topic: TOPICS.control.moduanMoveZero, type: MESSAGE_TYPES.float32, group: "末端控制" },
  { id: 16, name: "关闭绑扎", topic: TOPICS.control.enableLashing, type: MESSAGE_TYPES.float32, group: "末端控制" },
  { id: 18, name: "急停作业", topic: TOPICS.control.forcedStop, type: MESSAGE_TYPES.float32, group: "末端控制" },
  { id: 19, name: "设置Z固定高度", topic: TOPICS.algorithm.setHeightThreshold, type: MESSAGE_TYPES.float32, group: "视觉调试" },
  { id: 21, name: "保存绑扎数据", topic: TOPICS.control.saveBindingData, type: MESSAGE_TYPES.float32, group: "末端控制" },
  { id: 22, name: "修正TF外参", topic: TOPICS.tf.setCameraTcpExtrinsic, type: MESSAGE_TYPES.pose, group: "TF标定" },
  { id: 23, name: "设置索驱速度", topic: TOPICS.process.setCabinSpeed, type: MESSAGE_TYPES.float32, group: "流程控制" },
  { id: 24, name: "设置末端速度", topic: TOPICS.control.setModuanSpeed, type: MESSAGE_TYPES.float32, group: "末端控制" },
];
