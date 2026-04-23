export const CONTROL_PANEL_TASKS = [
  { id: "submitQuad", label: "提交\n四边形", tone: "green" },
  { id: "runSavedS2", label: "触发\nS2", tone: "green" },
  { id: "scanPlan", label: "固定扫描\n规划", tone: "green" },
  { id: "startExecution", label: "开始\n执行层", tone: "green" },
  { id: "startExecutionKeepMemory", label: "保留记忆\n开始", tone: "amber" },
  { id: "runBindPathTest", label: "账本\n测试", tone: "amber" },
  { id: "moveToPosition", label: "移动到\n索驱位置", tone: "blue" },
];

export function getControlPanelTaskIds() {
  return CONTROL_PANEL_TASKS.map((task) => task.id);
}

export function normalizeControlPanelVisibleTaskIds(taskIds) {
  const allowedIds = new Set(getControlPanelTaskIds());
  if (!Array.isArray(taskIds)) {
    return getControlPanelTaskIds();
  }
  return taskIds.filter((taskId, index) => allowedIds.has(taskId) && taskIds.indexOf(taskId) === index);
}

export function normalizeCustomControlPanelButtons(buttons) {
  if (!Array.isArray(buttons)) {
    return [];
  }

  const seenIds = new Set();
  return buttons
    .map((button) => {
      const rawId = String(button?.id || "").trim();
      const rawLabel = String(button?.label || button?.name || "").trim();
      const rawServicePath = String(button?.servicePath || button?.service || "").trim();
      const normalizedServicePath = rawServicePath
        ? (rawServicePath.startsWith("/") ? rawServicePath : `/${rawServicePath}`)
        : "";
      if (!rawId || !rawLabel || !normalizedServicePath || seenIds.has(rawId)) {
        return null;
      }
      seenIds.add(rawId);
      return {
        id: rawId,
        label: rawLabel,
        servicePath: normalizedServicePath,
        serviceType: "std_srvs/Trigger",
        tone: "blue",
      };
    })
    .filter(Boolean);
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
        return null;
      })
      .filter(Boolean),
  }));
}
