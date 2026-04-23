export const DEFAULT_LAYOUTS = {
  executionDebug: {
    id: "executionDebug",
    title: "执行调试",
    panels: {
      controlPanel: { visible: true },
      imagePanel: { visible: true },
      settingsPanel: { visible: true },
      terminalPanel: { visible: false },
      logPanel: { visible: false },
    },
  },
  visionDebug: {
    id: "visionDebug",
    title: "视觉识别",
    panels: {
      controlPanel: { visible: true },
      imagePanel: { visible: true },
      settingsPanel: { visible: true },
      terminalPanel: { visible: true },
      logPanel: { visible: true },
    },
  },
};
