export const PANEL_REGISTRY = [
  {
    id: "controlPanel",
    title: "控制面板",
    group: "operations",
    defaultVisible: true,
  },
  {
    id: "workspacePanel",
    title: "工作区",
    group: "vision",
    defaultVisible: true,
  },
  {
    id: "topicLayersPanel",
    title: "话题图层",
    group: "scene",
    defaultVisible: true,
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
