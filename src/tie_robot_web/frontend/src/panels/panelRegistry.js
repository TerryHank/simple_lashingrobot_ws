export const PANEL_REGISTRY = [
  {
    id: "controlPanel",
    title: "控制面板",
    group: "operations",
    defaultVisible: true,
  },
  {
    id: "imagePanel",
    title: "图像",
    group: "vision",
    defaultVisible: true,
  },
  {
    id: "settingsPanel",
    title: "设置",
    group: "settings",
    defaultVisible: true,
  },
  {
    id: "terminalPanel",
    title: "终端",
    group: "tools",
    defaultVisible: false,
  },
  {
    id: "logPanel",
    title: "日志",
    group: "diagnostics",
    defaultVisible: false,
  },
];

export function getPanelRegistryMap() {
  return new Map(PANEL_REGISTRY.map((panel) => [panel.id, panel]));
}
