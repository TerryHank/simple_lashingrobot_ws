export const CONTROL_PANEL_TASK_SECTIONS = [
  {
    id: "scan",
    title: "扫描区",
    tasks: [
      { id: "runSavedS2", label: "触发扫描\n视觉", tone: "blue" },
    ],
  },
  {
    id: "execution",
    title: "执行层",
    tasks: [
      { id: "startExecution", label: "执行全局\n绑扎", tone: "green" },
      { id: "triggerSingleBind", label: "触发单点\n绑扎", tone: "red" },
      { id: "executionVisionOnly", label: "单点视觉\n测试", tone: "blue" },
      { id: "startExecutionKeepMemory", label: "记忆续跑\n开始", tone: "amber" },
    ],
  },
  {
    id: "areaNavigation",
    title: "区域切换",
    tasks: [
      { id: "previousArea", label: "上一个\n区域", tone: "blue" },
      { id: "nextArea", label: "下一个\n区域", tone: "blue" },
    ],
  },
];

export const CONTROL_PANEL_TASKS = CONTROL_PANEL_TASK_SECTIONS.flatMap((section) => section.tasks);

export const CHECKERBOARD_PARITY_LABELS = Object.freeze({
  0: "黑棋",
  1: "白棋",
});

export const CHECKERBOARD_PARITY_COLORS = Object.freeze({
  0: "black",
  1: "white",
});

export function normalizeCheckerboardParity(value) {
  return Number(value) === 1 ? 1 : 0;
}

export function getCheckerboardParityLabel(value) {
  return CHECKERBOARD_PARITY_LABELS[normalizeCheckerboardParity(value)];
}

export function getCheckerboardParityColor(value) {
  return CHECKERBOARD_PARITY_COLORS[normalizeCheckerboardParity(value)];
}

export function buildControlToggleState(definition, active, { selectedParity } = {}) {
  const isActive = Boolean(active);
  const state = {
    value: isActive,
    label: isActive ? definition.activeLabel : definition.inactiveLabel,
    tone: isActive ? definition.activeTone : definition.inactiveTone,
  };

  if (definition.selectedParityCommandId) {
    const parity = normalizeCheckerboardParity(
      selectedParity ?? definition.selectedParityInitialValue,
    );
    state.selectedParity = parity;
    state.selectedColor = getCheckerboardParityColor(parity);
    state.label = `${isActive ? "长按关闭" : "长按开启"}跳绑${getCheckerboardParityLabel(parity)}`;
  }

  return state;
}

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
    inactiveRequiresLongPress: true,
    inactiveLongPressCommandId: 25,
    activeRequiresLongPress: true,
    longPressCommandId: 25,
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
    inactiveLabel: "长按开启跳绑黑棋",
    activeLabel: "长按关闭跳绑黑棋",
    inactiveTone: "blue",
    activeTone: "amber",
    commandId: 12,
    messageType: "std_msgs/Bool",
    inactiveRequiresLongPress: true,
    inactiveLongPressCommandId: 12,
    activeRequiresLongPress: true,
    longPressCommandId: 12,
    longPressTogglesState: true,
    singleClickAction: "cycleSelectedParity",
    selectedParityCommandId: 26,
    selectedParityInitialValue: 0,
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
  { title: "控制开关", items: ["pauseResume", "lashingEnabled", "jumpBindEnabled", "lightEnabled"] },
];

export function getControlToggleDefinition(toggleId) {
  return CONTROL_TOGGLE_DEFINITIONS[toggleId] || null;
}

export function getInitialControlToggleState(toggleId) {
  const definition = getControlToggleDefinition(toggleId);
  if (!definition) {
    return null;
  }
  const active = Boolean(definition.initialValue);
  return buildControlToggleState(definition, active);
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
        return null;
      })
      .filter(Boolean),
  }));
}
