import { ROSLIB } from "../vendor/roslib.js";
import { resolveRosbridgeUrl } from "../utils/rosbridge.js";

const IR_TOPIC = "/Scepter/ir/image_raw";
const WORKSPACE_QUAD_TOPIC = "/web/pointAI/set_workspace_quad";
const RUN_WORKSPACE_S2_TOPIC = "/web/pointAI/run_workspace_s2";
const CURRENT_WORKSPACE_QUAD_TOPIC = "/pointAI/manual_workspace_quad_pixels";
const EXECUTION_RESULT_TOPIC = "/pointAI/result_image_raw";
const FILTERED_WORLD_COORD_TOPIC = "/Scepter/worldCoord/world_coord";
const RAW_WORLD_COORD_TOPIC = "/Scepter/worldCoord/raw_world_coord";
const COORDINATE_POINT_TOPIC = "/coordinate_point";
const PSEUDO_SLAM_MARKERS_TOPIC = "/cabin/pseudo_slam_markers";
const SYSTEM_LOG_TOPIC = "/system_log/all";
const TF_TOPIC = "/tf";
const TF_STATIC_TOPIC = "/tf_static";

const START_PSEUDO_SLAM_SCAN_ACTION = "/web/cabin/start_pseudo_slam_scan";
const START_GLOBAL_WORK_ACTION = "/web/cabin/start_global_work";
const RUN_DIRECT_BIND_PATH_TEST_ACTION = "/web/cabin/run_bind_path_direct_test";
const SET_EXECUTION_MODE_SERVICE = "/cabin/set_execution_mode";

const START_PSEUDO_SLAM_SCAN_ACTION_TYPE = "tie_robot_msgs/StartPseudoSlamScanTaskAction";
const START_GLOBAL_WORK_ACTION_TYPE = "tie_robot_msgs/StartGlobalWorkTaskAction";
const RUN_DIRECT_BIND_PATH_TEST_ACTION_TYPE = "tie_robot_msgs/RunBindPathDirectTestTaskAction";
const SET_EXECUTION_MODE_SERVICE_TYPE = "tie_robot_msgs/SetExecutionMode";

export class RosConnectionController {
  constructor(callbacks = {}) {
    this.callbacks = callbacks;
    this.ros = null;
    this.resources = null;
    this.topicSubscribers = [];
  }

  connect() {
    const rosbridgeUrl = resolveRosbridgeUrl({
      pathname: window.location.pathname,
      currentHost: window.location.hostname,
    });
    this.callbacks.onConnectionInfo?.(rosbridgeUrl, "连接中", "info");
    this.callbacks.onLog?.(`开始连接 ROSBridge: ${rosbridgeUrl}`, "info");

    this.ros = new ROSLIB.Ros({ url: rosbridgeUrl });
    this.resources = this.buildResources(this.ros);

    this.ros.on("connection", () => {
      this.resources.workspaceQuadPublisher.advertise();
      this.resources.runWorkspaceS2Publisher.advertise();
      this.bindSubscriptions();
      this.callbacks.onConnectionInfo?.(rosbridgeUrl, "连接成功", "success");
      this.callbacks.onRosReady?.(this.resources);
      this.callbacks.onLog?.("ROS 连接成功，动作链与图像订阅已就绪", "success");
    });

    this.ros.on("error", (error) => {
      const detail = error?.message || String(error);
      this.callbacks.onConnectionInfo?.(rosbridgeUrl, "连接失败", "error");
      this.callbacks.onRosUnavailable?.();
      this.callbacks.onLog?.(`ROS 连接失败: ${detail}`, "error");
    });

    this.ros.on("close", () => {
      this.unbindSubscriptions();
      this.callbacks.onConnectionInfo?.(rosbridgeUrl, "连接失败", "warn");
      this.callbacks.onRosUnavailable?.();
      this.callbacks.onLog?.("ROS 连接已断开", "warn");
    });
  }

  buildResources(ros) {
    return {
      ros,
      workspaceQuadPublisher: new ROSLIB.Topic({
        ros,
        name: WORKSPACE_QUAD_TOPIC,
        messageType: "std_msgs/Float32MultiArray",
      }),
      runWorkspaceS2Publisher: new ROSLIB.Topic({
        ros,
        name: RUN_WORKSPACE_S2_TOPIC,
        messageType: "std_msgs/Bool",
      }),
      startPseudoSlamScanActionClient: new ROSLIB.ActionClient({
        ros,
        serverName: START_PSEUDO_SLAM_SCAN_ACTION,
        actionName: START_PSEUDO_SLAM_SCAN_ACTION_TYPE,
      }),
      executionModeService: new ROSLIB.Service({
        ros,
        name: SET_EXECUTION_MODE_SERVICE,
        serviceType: SET_EXECUTION_MODE_SERVICE_TYPE,
      }),
      startGlobalWorkActionClient: new ROSLIB.ActionClient({
        ros,
        serverName: START_GLOBAL_WORK_ACTION,
        actionName: START_GLOBAL_WORK_ACTION_TYPE,
      }),
      runDirectBindPathTestActionClient: new ROSLIB.ActionClient({
        ros,
        serverName: RUN_DIRECT_BIND_PATH_TEST_ACTION,
        actionName: RUN_DIRECT_BIND_PATH_TEST_ACTION_TYPE,
      }),
    };
  }

  bindSubscriptions() {
    this.unbindSubscriptions();
    const subscriptions = [
      new ROSLIB.Topic({
        ros: this.ros,
        name: IR_TOPIC,
        messageType: "sensor_msgs/Image",
      }),
      new ROSLIB.Topic({
        ros: this.ros,
        name: CURRENT_WORKSPACE_QUAD_TOPIC,
        messageType: "std_msgs/Float32MultiArray",
      }),
      new ROSLIB.Topic({
        ros: this.ros,
        name: EXECUTION_RESULT_TOPIC,
        messageType: "sensor_msgs/Image",
      }),
      new ROSLIB.Topic({
        ros: this.ros,
        name: FILTERED_WORLD_COORD_TOPIC,
        messageType: "sensor_msgs/Image",
      }),
      new ROSLIB.Topic({
        ros: this.ros,
        name: RAW_WORLD_COORD_TOPIC,
        messageType: "sensor_msgs/Image",
      }),
      new ROSLIB.Topic({
        ros: this.ros,
        name: COORDINATE_POINT_TOPIC,
        messageType: "tie_robot_msgs/PointsArray",
      }),
      new ROSLIB.Topic({
        ros: this.ros,
        name: PSEUDO_SLAM_MARKERS_TOPIC,
        messageType: "visualization_msgs/MarkerArray",
      }),
      new ROSLIB.Topic({
        ros: this.ros,
        name: SYSTEM_LOG_TOPIC,
        messageType: "rosgraph_msgs/Log",
      }),
      new ROSLIB.Topic({
        ros: this.ros,
        name: TF_TOPIC,
        messageType: "tf2_msgs/TFMessage",
      }),
      new ROSLIB.Topic({
        ros: this.ros,
        name: TF_STATIC_TOPIC,
        messageType: "tf2_msgs/TFMessage",
      }),
    ];

    subscriptions[0].subscribe((message) => this.callbacks.onBaseImage?.(message));
    subscriptions[1].subscribe((message) => this.callbacks.onSavedWorkspacePayload?.(Array.from(message.data || [])));
    subscriptions[2].subscribe((message) => this.callbacks.onExecutionOverlay?.(message));
    subscriptions[3].subscribe((message) => this.callbacks.onPointCloudImage?.("filteredWorldCoord", message));
    subscriptions[4].subscribe((message) => this.callbacks.onPointCloudImage?.("rawWorldCoord", message));
    subscriptions[5].subscribe((message) => this.callbacks.onTiePoints?.(message));
    subscriptions[6].subscribe((message) => this.callbacks.onPlanningMarkers?.(message));
    subscriptions[7].subscribe((message) => this.callbacks.onSystemLog?.(message));
    subscriptions[8].subscribe((message) => this.callbacks.onTfMessage?.(message));
    subscriptions[9].subscribe((message) => this.callbacks.onTfMessage?.(message));
    this.topicSubscribers = subscriptions;
  }

  unbindSubscriptions() {
    this.topicSubscribers.forEach((topic) => {
      try {
        topic.unsubscribe();
      } catch {
        // ignore shutdown race
      }
    });
    this.topicSubscribers = [];
  }

  getResources() {
    return this.resources;
  }

  isReady() {
    return Boolean(this.resources && this.ros && this.ros.isConnected);
  }
}
