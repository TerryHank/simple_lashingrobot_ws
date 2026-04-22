import { LEGACY_COMMANDS } from "./legacyCommandCatalog.js";

export const CONTROL_PANEL_TASKS = [
  { id: "submitQuad", label: "提交四边形\n触发 S2", tone: "green" },
  { id: "runSavedS2", label: "直接识别\n绑扎点", tone: "green" },
  { id: "scanPlan", label: "固定扫描\n规划", tone: "green" },
  { id: "startExecution", label: "开始\n执行层", tone: "green" },
  { id: "startExecutionKeepMemory", label: "保留记忆\n开始", tone: "amber" },
  { id: "runBindPathTest", label: "账本\n测试", tone: "amber" },
];

export const CONTROL_TOGGLE_DEFINITIONS = {
  pauseResume: {
    id: "pauseResume",
    group: "末端控制",
    stateKey: "paused",
    initialValue: false,
    inactiveLabel: "暂停作业",
    activeLabel: "恢复作业",
    inactiveTone: "amber",
    activeTone: "green",
    activateCommandId: 10,
    deactivateCommandId: 13,
  },
  lashingEnabled: {
    id: "lashingEnabled",
    group: "末端控制",
    stateKey: "enabled",
    initialValue: true,
    inactiveLabel: "开启绑扎",
    activeLabel: "关闭绑扎",
    inactiveTone: "green",
    activeTone: "red",
    activateCommandId: 11,
    deactivateCommandId: 16,
  },
  jumpBindEnabled: {
    id: "jumpBindEnabled",
    group: "末端控制",
    stateKey: "enabled",
    initialValue: false,
    inactiveLabel: "开启跳绑",
    activeLabel: "关闭跳绑",
    inactiveTone: "blue",
    activeTone: "amber",
    commandId: 12,
    messageType: "std_msgs/Bool",
  },
  lightEnabled: {
    id: "lightEnabled",
    group: "末端控制",
    stateKey: "enabled",
    initialValue: false,
    inactiveLabel: "开启灯光",
    activeLabel: "关闭灯光",
    inactiveTone: "blue",
    activeTone: "amber",
    commandId: 14,
    messageType: "std_msgs/Bool",
  },
};

const CONTROL_PANEL_GROUPS = [
  { title: "流程控制", items: [1, 2, 3, 4, 5, 17, 20, 23] },
  { title: "调试控制", items: [7, 6, 8, 9] },
  { title: "末端控制", items: ["pauseResume", "lashingEnabled", "jumpBindEnabled", "lightEnabled", 15, 18, 21, 24] },
  { title: "视觉调试", items: [19, 22] },
];

const COMMAND_TONES = {
  1: "green",
  2: "green",
  3: "red",
  4: "green",
  5: "amber",
  6: "amber",
  7: "amber",
  8: "amber",
  9: "amber",
  15: "green",
  17: "red",
  18: "red",
  19: "blue",
  20: "blue",
  21: "blue",
  22: "blue",
  23: "blue",
  24: "blue",
};

export function getControlPanelCommandTone(commandId) {
  return COMMAND_TONES[commandId] || "amber";
}

export function getControlToggleDefinition(toggleId) {
  return CONTROL_TOGGLE_DEFINITIONS[toggleId] || null;
}

export function getInitialControlToggleState(toggleId) {
  const definition = getControlToggleDefinition(toggleId);
  if (!definition) {
    return null;
  }
  const active = Boolean(definition.initialValue);
  return {
    value: active,
    label: active ? definition.activeLabel : definition.inactiveLabel,
    tone: active ? definition.activeTone : definition.inactiveTone,
  };
}

export function getInitialControlToggleStateMap() {
  return Object.keys(CONTROL_TOGGLE_DEFINITIONS).reduce((accumulator, toggleId) => {
    accumulator[toggleId] = getInitialControlToggleState(toggleId);
    return accumulator;
  }, {});
}

export function getControlPanelGroups() {
  return CONTROL_PANEL_GROUPS.map((group) => ({
    title: group.title,
    controls: group.items
      .map((item) => {
        if (typeof item === "string") {
          const definition = getControlToggleDefinition(item);
          if (!definition) {
            return null;
          }
          const initialState = getInitialControlToggleState(item);
          return {
            kind: "toggle",
            id: definition.id,
            label: initialState?.label || definition.inactiveLabel,
            tone: initialState?.tone || definition.inactiveTone,
            active: Boolean(initialState?.value),
          };
        }
        const command = LEGACY_COMMANDS.find((entry) => entry.id === item);
        if (!command) {
          return null;
        }
        return {
          kind: "command",
          id: String(command.id),
          commandId: command.id,
          label: command.name,
          tone: getControlPanelCommandTone(command.id),
        };
      })
      .filter(Boolean),
  }));
}
