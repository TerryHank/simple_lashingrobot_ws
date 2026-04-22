import {
  applyModePreset,
  DEFAULT_TOPIC_LAYER_STATE,
  getSceneViewModeLabel,
  getTopicLayerModeLabel,
} from "../config/topicLayerCatalog.js";

export class TopicLayerController {
  constructor({ ui, sceneView, callbacks = {} }) {
    this.ui = ui;
    this.sceneView = sceneView;
    this.callbacks = callbacks;
    this.state = { ...DEFAULT_TOPIC_LAYER_STATE };
    this.stats = {
      filteredWorldCoordCount: 0,
      rawWorldCoordCount: 0,
      tiePointCount: 0,
      planningPointCount: 0,
      tfFrameCount: 0,
    };
  }

  init() {
    this.ui.setTopicLayerState(this.state);
    this.ui.renderTopicLayerStats(this.stats);
    this.sceneView.setLayerState(this.state);
    this.sceneView.setViewMode(this.state.viewMode);
    this.sceneView.setFollowCamera(this.state.followCamera);
  }

  handleLayerControlsChange(partialState) {
    const nextMode = partialState.mode || this.state.mode;
    let nextState = { ...this.state, ...partialState };
    if (partialState.mode && partialState.mode !== this.state.mode) {
      nextState = applyModePreset(nextMode, nextState);
    }
    this.state = nextState;
    this.ui.setTopicLayerState(this.state);
    this.sceneView.setLayerState(this.state);
    this.callbacks.onLog?.(`三维图层模式已切换为 ${getTopicLayerModeLabel(this.state.mode)}`, "info");
  }

  handleSceneControlsChange(partialState) {
    this.state = { ...this.state, ...partialState };
    this.ui.setTopicLayerState(this.state);
    this.sceneView.setViewMode(this.state.viewMode);
    this.sceneView.setFollowCamera(this.state.followCamera);
    this.callbacks.onLog?.(
      `三维视角已切换为 ${getSceneViewModeLabel(this.state.viewMode)}${this.state.followCamera ? "（跟随相机）" : ""}`,
      "info",
    );
  }

  updateStats(partialStats) {
    this.stats = { ...this.stats, ...partialStats };
    this.ui.renderTopicLayerStats(this.stats);
  }

  getState() {
    return { ...this.state };
  }
}
