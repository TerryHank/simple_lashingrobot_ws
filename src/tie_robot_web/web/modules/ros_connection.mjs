import {
  parseWorkspaceQuadPayload,
  resolveRosbridgeUrl,
} from "../ir_workspace_picker_helpers.mjs";
import { resultStatusEl } from "./dom_refs.mjs";
import { drawOverlay, drawS2Overlay } from "./canvas_renderer.mjs";
import {
  CURRENT_WORKSPACE_QUAD_TOPIC,
  EXECUTION_RESULT_TOPIC,
  IR_TOPIC,
  loadSavedConnectionPreferences,
  ROSLIB,
  RUN_DIRECT_BIND_PATH_TEST_ACTION,
  RUN_DIRECT_BIND_PATH_TEST_ACTION_TYPE,
  RUN_WORKSPACE_S2_TOPIC,
  setStatus,
  SET_EXECUTION_MODE_SERVICE,
  SET_EXECUTION_MODE_SERVICE_TYPE,
  START_GLOBAL_WORK_ACTION,
  START_GLOBAL_WORK_ACTION_TYPE,
  START_PSEUDO_SLAM_SCAN_ACTION,
  START_PSEUDO_SLAM_SCAN_ACTION_TYPE,
  state,
  updateActionButtons,
  updateConnectionText,
  WORKSPACE_QUAD_TOPIC,
  WORKSPACE_S2_RESULT_TOPIC,
} from "./ui_state.mjs";

export function connectRosbridge() {
  const rosbridgeUrl = resolveRosbridgeUrl({
    pathname: window.location.pathname,
    savedPreferences: loadSavedConnectionPreferences(),
    currentHost: window.location.hostname,
  });
  updateConnectionText(rosbridgeUrl);
  setStatus(`正在连接 ${rosbridgeUrl} ...`, "info");

  state.ros = new ROSLIB.Ros({ url: rosbridgeUrl });
  state.irSubscriber = new ROSLIB.Topic({ ros: state.ros, name: IR_TOPIC, messageType: "sensor_msgs/Image" });
  state.quadPublisher = new ROSLIB.Topic({ ros: state.ros, name: WORKSPACE_QUAD_TOPIC, messageType: "std_msgs/Float32MultiArray" });
  state.runS2Publisher = new ROSLIB.Topic({ ros: state.ros, name: RUN_WORKSPACE_S2_TOPIC, messageType: "std_msgs/Bool" });
  state.pseudoSlamScanActionClient = new ROSLIB.ActionClient({
    ros: state.ros,
    serverName: START_PSEUDO_SLAM_SCAN_ACTION,
    actionName: START_PSEUDO_SLAM_SCAN_ACTION_TYPE,
  });
  state.executionModeService = new ROSLIB.Service({
    ros: state.ros,
    name: SET_EXECUTION_MODE_SERVICE,
    serviceType: SET_EXECUTION_MODE_SERVICE_TYPE,
  });
  state.startGlobalWorkActionClient = new ROSLIB.ActionClient({
    ros: state.ros,
    serverName: START_GLOBAL_WORK_ACTION,
    actionName: START_GLOBAL_WORK_ACTION_TYPE,
  });
  state.runDirectBindPathTestActionClient = new ROSLIB.ActionClient({
    ros: state.ros,
    serverName: RUN_DIRECT_BIND_PATH_TEST_ACTION,
    actionName: RUN_DIRECT_BIND_PATH_TEST_ACTION_TYPE,
  });
  state.currentQuadSubscriber = new ROSLIB.Topic({
    ros: state.ros,
    name: CURRENT_WORKSPACE_QUAD_TOPIC,
    messageType: "std_msgs/Float32MultiArray",
  });
  state.s2ResultSubscriber = new ROSLIB.Topic({
    ros: state.ros,
    name: WORKSPACE_S2_RESULT_TOPIC,
    messageType: "sensor_msgs/Image",
  });
  state.executionResultSubscriber = new ROSLIB.Topic({
    ros: state.ros,
    name: EXECUTION_RESULT_TOPIC,
    messageType: "sensor_msgs/Image",
  });

  state.ros.on("connection", () => {
    setStatus("ROS 连接成功，等待 IR 图像...", "success");
    state.quadPublisher.advertise();
    state.runS2Publisher.advertise();
    updateActionButtons();
    state.currentQuadSubscriber.subscribe((message) => {
      try {
        state.savedWorkspacePoints = parseWorkspaceQuadPayload(Array.from(message.data || []));
        updateActionButtons();
        drawOverlay();
      } catch (error) {
        console.warn("无法解析当前已保存工作区四边形", error);
      }
    });
    state.s2ResultSubscriber.subscribe((message) => {
      state.lastResultImageMessage = message;
      if (state.overlaySource !== "execution") {
        state.overlaySource = "s2";
      }
      drawS2Overlay();
      if (resultStatusEl) {
        resultStatusEl.textContent = "已收到最新 S2 result，当前这层覆盖直接叠在 IR 选点底图上。";
      }
      setStatus("工作区已保存，S2 result 已更新", "success");
    });
    state.executionResultSubscriber.subscribe((message) => {
      state.lastExecutionResultImageMessage = message;
      if (state.overlaySource === "execution") {
        drawS2Overlay();
      }
      if (resultStatusEl && state.overlaySource === "execution") {
        resultStatusEl.textContent = "执行层 result_img 已更新，当前覆盖层正在显示 /pointAI/result_image_raw。";
      }
      if (state.overlaySource === "execution") {
        setStatus("执行层识别图层已更新", "success");
      }
    });
    state.irSubscriber.subscribe((message) => {
      state.lastImageMessage = message;
      drawOverlay();
      if (state.selectedPoints.length === 0) {
        setStatus("IR 图像已就绪，直接在图上点 4 个角点", "success");
      }
    });
  });

  state.ros.on("error", (error) => {
    setStatus(`ROS 连接失败: ${error?.message || error}`, "error");
  });

  state.ros.on("close", () => {
    setStatus("ROS 连接已断开", "warn");
    if (resultStatusEl) {
      resultStatusEl.textContent = "ROS 已断开，S2 result 停止更新。";
    }
  });
}
