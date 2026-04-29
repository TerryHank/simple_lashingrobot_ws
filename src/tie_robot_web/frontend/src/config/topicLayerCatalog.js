import { getTopicLayerSourceTopic } from "./topicRegistry.js";

export const TOPIC_LAYER_MODES = [
  { id: "onlyPointCloud", label: "只显示点云" },
  { id: "onlyPlanningPoints", label: "只显示规划点/绑扎点" },
  { id: "pointCloudAndPlanningPoints", label: "点云 + 规划点/绑扎点" },
  { id: "machineOnly", label: "只显示机器" },
  { id: "all", label: "全开" },
];

export const POINT_CLOUD_SOURCES = [
  { id: "filteredWorldCoord", label: "滤波世界点云", topic: getTopicLayerSourceTopic("filteredWorldCoord") },
  { id: "rawWorldCoord", label: "原始世界点云", topic: getTopicLayerSourceTopic("rawWorldCoord") },
];

export const SCENE_VIEW_MODES = [
  { id: "camera", label: "相机视角" },
  { id: "global", label: "全局视角" },
];

export const TF_AXIS_FRAMES = [
  { id: "map", label: "索驱世界 map" },
  { id: "base_link", label: "机器 base_link" },
  { id: "Scepter_depth_frame", label: "相机 Scepter_depth_frame" },
  { id: "gripper_frame", label: "TCP gripper_frame" },
];

export const DEFAULT_TF_AXIS_FRAME_VISIBILITY = TF_AXIS_FRAMES.reduce((accumulator, frame) => {
  accumulator[frame.id] = true;
  return accumulator;
}, {});

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
  onlyPlanningPoints: {
    showRobot: false,
    showAxes: true,
    showPointCloud: false,
    showTiePoints: false,
    showPlanningMarkers: true,
  },
  pointCloudAndPlanningPoints: {
    showRobot: true,
    showAxes: true,
    showPointCloud: true,
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
    showTiePoints: false,
    showPlanningMarkers: true,
  },
};

export const DEFAULT_TOPIC_LAYER_STATE = {
  mode: "all",
  pointCloudSource: "filteredWorldCoord",
  showRobot: true,
  showAxes: true,
  showPointCloud: false,
  showTiePoints: false,
  showPlanningMarkers: true,
  tfAxisFrameVisibility: DEFAULT_TF_AXIS_FRAME_VISIBILITY,
  pointSize: 0.035,
  pointOpacity: 0.78,
  viewMode: "camera",
  followCamera: false,
};

export function applyModePreset(mode, currentState) {
  const normalizedMode = MODE_PRESETS[mode] ? mode : "pointCloudAndPlanningPoints";
  const preset = MODE_PRESETS[normalizedMode];
  return {
    ...currentState,
    mode: normalizedMode,
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
