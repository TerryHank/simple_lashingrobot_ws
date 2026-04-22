import { LEGACY_COMMANDS } from "../config/legacyCommandCatalog.js";
import {
  getControlToggleDefinition,
  getInitialControlToggleStateMap,
} from "../config/controlPanelCatalog.js";
import { ROSLIB } from "../vendor/roslib.js";

function buildFloatMessage(command, parameters) {
  if (command.id === 17 || command.id === 16) {
    return { data: 0 };
  }
  if (command.id === 5) {
    return { data: 2 };
  }
  if (command.id === 18) {
    return { data: 3 };
  }
  if (command.id === 19) {
    return { data: Number(parameters.globalZ) || 0 };
  }
  if (command.id === 23) {
    return { data: Number(parameters.speed) || 0 };
  }
  if (command.id === 24) {
    return { data: Number(parameters.fixedSpeed) || 0 };
  }
  return { data: 1 };
}

function buildPoseMessage(command, parameters) {
  if (command.id === 2) {
    return {
      position: {
        x: Number(parameters.startX) || 0,
        y: Number(parameters.startY) || 0,
        z: Number(parameters.startZ) || 0,
      },
      orientation: {
        x: Number(parameters.globalX) || 0,
        y: Number(parameters.globalY) || 0,
        z: Number(parameters.stepX) || 0,
        w: Number(parameters.stepY) || 0,
      },
    };
  }
  if (command.id === 6) {
    return {
      position: {
        x: Number(parameters.fixedX) || 0,
        y: Number(parameters.fixedY) || 0,
        z: Number(parameters.fixedZ) || 0,
      },
      orientation: {
        x: Number(parameters.fixedTheta) || 0,
        y: 0,
        z: 0,
        w: 1,
      },
    };
  }
  if (command.id === 7) {
    return {
      position: {
        x: Number(parameters.startX) || 0,
        y: Number(parameters.startY) || 0,
        z: Number(parameters.startZ) || 0,
      },
      orientation: {
        x: Number(parameters.speed) || 0,
        y: 0,
        z: 0,
        w: 1,
      },
    };
  }
  if (command.id === 22) {
    return {
      position: {
        x: Number(parameters.fixedX) || 0,
        y: Number(parameters.fixedY) || 0,
        z: Number(parameters.fixedZ) || 0,
      },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    };
  }
  return {
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 },
  };
}

export class LegacyCommandController {
  constructor({ rosConnection, callbacks = {} }) {
    this.rosConnection = rosConnection;
    this.callbacks = callbacks;
    this.publisherCache = new Map();
    this.toggleStates = new Map(
      Object.entries(getInitialControlToggleStateMap()).map(([toggleId, state]) => [toggleId, state.value]),
    );
  }

  reset() {
    this.publisherCache.clear();
    this.toggleStates = new Map(
      Object.entries(getInitialControlToggleStateMap()).map(([toggleId, state]) => [toggleId, state.value]),
    );
  }

  getToggleStateSnapshot() {
    return Object.entries(getInitialControlToggleStateMap()).reduce((accumulator, [toggleId, initialState]) => {
      const active = this.toggleStates.has(toggleId)
        ? this.toggleStates.get(toggleId)
        : initialState.value;
      accumulator[toggleId] = {
        value: active,
        label: active ? getControlToggleDefinition(toggleId)?.activeLabel : getControlToggleDefinition(toggleId)?.inactiveLabel,
        tone: active ? getControlToggleDefinition(toggleId)?.activeTone : getControlToggleDefinition(toggleId)?.inactiveTone,
      };
      return accumulator;
    }, {});
  }

  handle(commandId, parameters) {
    const command = LEGACY_COMMANDS.find((item) => item.id === commandId);
    if (!command) {
      this.callbacks.onResultMessage?.(`未识别的老前端命令: ${commandId}`);
      this.callbacks.onLog?.(`未识别的老前端命令: ${commandId}`, "warn");
      return;
    }
    const resources = this.rosConnection.getResources();
    if (!resources?.ros) {
      this.callbacks.onResultMessage?.(`ROS 未连接，无法执行: ${command.name}`);
      this.callbacks.onLog?.(`ROS 未连接，无法执行: ${command.name}`, "warn");
      return;
    }

    const publisher = this.getOrCreatePublisher(resources.ros, command.topic, command.type);
    const payload = this.buildMessagePayload(command, parameters);
    publisher.publish(new ROSLIB.Message(payload));
    this.callbacks.onResultMessage?.(`已发送 ${command.name} -> ${command.topic}`);
    this.callbacks.onLog?.(`已发送 ${command.name} -> ${command.topic}`, "success");
  }

  handleToggle(toggleId, parameters) {
    const definition = getControlToggleDefinition(toggleId);
    if (!definition) {
      this.callbacks.onResultMessage?.(`未识别的开关动作: ${toggleId}`);
      this.callbacks.onLog?.(`未识别的开关动作: ${toggleId}`, "warn");
      return null;
    }

    const resources = this.rosConnection.getResources();
    if (!resources?.ros) {
      this.callbacks.onResultMessage?.(`ROS 未连接，无法执行: ${definition.inactiveLabel}/${definition.activeLabel}`);
      this.callbacks.onLog?.(`ROS 未连接，无法执行开关: ${toggleId}`, "warn");
      return null;
    }

    const currentValue = this.toggleStates.has(toggleId)
      ? this.toggleStates.get(toggleId)
      : Boolean(definition.initialValue);
    const nextValue = !currentValue;
    const command = this.resolveToggleCommand(definition, nextValue);
    if (!command) {
      this.callbacks.onResultMessage?.(`开关 ${toggleId} 缺少对应命令`);
      this.callbacks.onLog?.(`开关 ${toggleId} 缺少对应命令`, "error");
      return null;
    }

    const publisher = this.getOrCreatePublisher(resources.ros, command.topic, command.type);
    const payload = this.buildToggleMessagePayload(definition, command, nextValue, parameters);
    publisher.publish(new ROSLIB.Message(payload));
    this.toggleStates.set(toggleId, nextValue);

    const label = nextValue ? definition.activeLabel : definition.inactiveLabel;
    const tone = nextValue ? definition.activeTone : definition.inactiveTone;
    const humanState = nextValue ? "已开启" : "已关闭";
    this.callbacks.onResultMessage?.(`${label}，${humanState}`);
    this.callbacks.onLog?.(`已发送 ${command.name} -> ${command.topic}，状态=${humanState}`, "success");
    return { value: nextValue, label, tone };
  }

  getOrCreatePublisher(ros, topicName, messageType) {
    const key = `${topicName}|${messageType}`;
    if (this.publisherCache.has(key)) {
      return this.publisherCache.get(key);
    }
    const publisher = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType,
    });
    publisher.advertise();
    this.publisherCache.set(key, publisher);
    return publisher;
  }

  buildMessagePayload(command, parameters) {
    if (command.type === "std_msgs/Float32") {
      return buildFloatMessage(command, parameters);
    }
    if (command.type === "geometry_msgs/Pose") {
      return buildPoseMessage(command, parameters);
    }
    return {};
  }

  resolveToggleCommand(definition, nextValue) {
    const commandId = nextValue
      ? (definition.activateCommandId ?? definition.commandId)
      : (definition.deactivateCommandId ?? definition.commandId);
    return LEGACY_COMMANDS.find((item) => item.id === commandId) || null;
  }

  buildToggleMessagePayload(definition, command, nextValue, parameters) {
    if (definition.messageType === "std_msgs/Bool" || command.type === "std_msgs/Bool") {
      return { data: nextValue };
    }
    return this.buildMessagePayload(command, parameters);
  }
}
