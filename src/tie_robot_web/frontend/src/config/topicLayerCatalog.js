export const TOPIC_LAYER_MODES = [
  { id: "onlyPointCloud", label: "只显示点云" },
  { id: "onlyTiePoints", label: "只显示绑扎点" },
  { id: "pointCloudAndTiePoints", label: "点云 + 绑扎点" },
  { id: "planningFocus", label: "规划点/执行点" },
  { id: "machineOnly", label: "只显示机器" },
  { id: "all", label: "全开" },
];

export const POINT_CLOUD_SOURCES = [
  { id: "filteredWorldCoord", label: "滤波世界点云" },
  { id: "rawWorldCoord", label: "原始世界点云" },
];

export const SCENE_VIEW_MODES = [
  { id: "camera", label: "相机视角" },
  { id: "global", label: "全局视角" },
];

function getLabel(options, id, fallback = id) {
  return options.find((option) => option.id === id)?.label || fallback;
}

export const MODE_PRESETS = {
  onlyPointCloud: {
    showRobot: false,
    showAxes: false,
    showPointCloud: true,
    showTiePoints: false,
    showPlanningMarkers: false,
  },
  onlyTiePoints: {
    showRobot: false,
    showAxes: true,
    showPointCloud: false,
    showTiePoints: true,
    showPlanningMarkers: false,
  },
  pointCloudAndTiePoints: {
    showRobot: true,
    showAxes: true,
    showPointCloud: true,
    showTiePoints: true,
    showPlanningMarkers: false,
  },
  planningFocus: {
    showRobot: true,
    showAxes: true,
    showPointCloud: false,
    showTiePoints: false,
    showPlanningMarkers: true,
  },
  machineOnly: {
    showRobot: true,
    showAxes: true,
    showPointCloud: false,
    showTiePoints: false,
    showPlanningMarkers: false,
  },
  all: {
    showRobot: true,
    showAxes: true,
    showPointCloud: true,
    showTiePoints: true,
    showPlanningMarkers: true,
  },
};

export const DEFAULT_TOPIC_LAYER_STATE = {
  mode: "all",
  pointCloudSource: "filteredWorldCoord",
  showRobot: true,
  showAxes: true,
  showPointCloud: false,
  showTiePoints: true,
  showPlanningMarkers: true,
  pointSize: 0.035,
  pointOpacity: 0.78,
  viewMode: "camera",
  followCamera: false,
};

export function applyModePreset(mode, currentState) {
  const preset = MODE_PRESETS[mode] || MODE_PRESETS.pointCloudAndTiePoints;
  return {
    ...currentState,
    mode,
    ...preset,
  };
}

export function getTopicLayerModeLabel(mode) {
  return getLabel(TOPIC_LAYER_MODES, mode);
}

export function getPointCloudSourceLabel(source) {
  return getLabel(POINT_CLOUD_SOURCES, source);
}

export function getSceneViewModeLabel(viewMode) {
  return getLabel(SCENE_VIEW_MODES, viewMode);
}
