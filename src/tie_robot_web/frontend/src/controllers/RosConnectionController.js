import { ROSLIB } from "../vendor/roslib.js";
import { resolveRosbridgeUrl } from "../utils/rosbridge.js";
import { DEFAULT_IMAGE_TOPIC } from "../config/imageTopicCatalog.js";
import { DEFAULT_LOG_TOPIC, getLogTopicOption } from "../config/logTopicCatalog.js";

const IR_TOPIC = "/Scepter/ir/image_raw";
const COLOR_TOPIC = "/Scepter/color/image_raw";
const DEPTH_TOPIC = "/Scepter/depth/image_raw";
const WORKSPACE_QUAD_TOPIC = "/web/pointAI/set_workspace_quad";
const POINTAI_OFFSET_TOPIC = "/web/pointAI/set_offset";
const SET_CABIN_SPEED_TOPIC = "/web/cabin/set_cabin_speed";
const SET_GRIPPER_TF_CALIBRATION_SERVICE = "/web/pointAI/set_gripper_tf_calibration";
const RUN_WORKSPACE_S2_TOPIC = "/web/pointAI/run_workspace_s2";
const CURRENT_WORKSPACE_QUAD_TOPIC = "/pointAI/manual_workspace_quad_pixels";
const EXECUTION_RESULT_TOPIC = "/pointAI/result_image_raw";
const FILTERED_WORLD_COORD_TOPIC = "/Scepter/worldCoord/world_coord";
const RAW_WORLD_COORD_TOPIC = "/Scepter/worldCoord/raw_world_coord";
const COORDINATE_POINT_TOPIC = "/coordinate_point";
const PSEUDO_SLAM_MARKERS_TOPIC = "/cabin/pseudo_slam_markers";
const DIAGNOSTICS_TOPIC = "/diagnostics";
const TF_TOPIC = "/tf";
const TF_STATIC_TOPIC = "/tf_static";

const START_PSEUDO_SLAM_SCAN_ACTION = "/web/cabin/start_pseudo_slam_scan";
const START_GLOBAL_WORK_ACTION = "/web/cabin/start_global_work";
const RUN_DIRECT_BIND_PATH_TEST_ACTION = "/web/cabin/run_bind_path_direct_test";
const SET_EXECUTION_MODE_SERVICE = "/cabin/set_execution_mode";
const START_DRIVER_STACK_SERVICE = "/web/system/start_driver_stack";
const RESTART_DRIVER_STACK_SERVICE = "/web/system/restart_driver_stack";
const START_ALGORITHM_STACK_SERVICE = "/web/system/start_algorithm_stack";
const RESTART_ALGORITHM_STACK_SERVICE = "/web/system/restart_algorithm_stack";
const RESTART_ROS_STACK_SERVICE = "/web/system/restart_ros_stack";
const START_CABIN_DRIVER_SERVICE = "/cabin/driver/start";
const STOP_CABIN_DRIVER_SERVICE = "/cabin/driver/stop";
const RESTART_CABIN_DRIVER_SERVICE = "/cabin/driver/restart";
const STOP_CABIN_MOTION_SERVICE = "/cabin/motion/stop";
const START_MODUAN_DRIVER_SERVICE = "/moduan/driver/start";
const STOP_MODUAN_DRIVER_SERVICE = "/moduan/driver/stop";
const RESTART_MODUAN_DRIVER_SERVICE = "/moduan/driver/restart";
const CABIN_SINGLE_MOVE_SERVICE = "/cabin/single_move";

const START_PSEUDO_SLAM_SCAN_ACTION_TYPE = "tie_robot_msgs/StartPseudoSlamScanTaskAction";
const START_GLOBAL_WORK_ACTION_TYPE = "tie_robot_msgs/StartGlobalWorkTaskAction";
const RUN_DIRECT_BIND_PATH_TEST_ACTION_TYPE = "tie_robot_msgs/RunBindPathDirectTestTaskAction";
const SET_EXECUTION_MODE_SERVICE_TYPE = "tie_robot_msgs/SetExecutionMode";
const SET_GRIPPER_TF_CALIBRATION_SERVICE_TYPE = "tie_robot_msgs/SetGripperTfCalibration";
const CABIN_SINGLE_MOVE_SERVICE_TYPE = "tie_robot_msgs/SingleMove";
const TRIGGER_SERVICE_TYPE = "std_srvs/Trigger";

export class RosConnectionController {
  constructor(callbacks = {}) {
    this.callbacks = callbacks;
    this.ros = null;
    this.resources = null;
    this.fixedTopicSubscribers = [];
    this.pointCloudTopicSubscriber = null;
    this.displayedImageTopicSubscriber = null;
    this.logTopicSubscriber = null;
    this.topicInventoryTimer = null;
    this.desiredPointCloudSubscription = {
      enabled: false,
      source: "filteredWorldCoord",
    };
    this.dynamicServiceCache = new Map();
    this.desiredDisplayedImageTopic = DEFAULT_IMAGE_TOPIC;
    this.desiredLogTopicId = DEFAULT_LOG_TOPIC;
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
      this.resources.gripperTfOffsetPublisher.advertise();
      this.resources.cabinSpeedPublisher.advertise();
      this.resources.runWorkspaceS2Publisher.advertise();
      this.bindSubscriptions();
      this.applyDisplayedImageSubscription({ suppressLog: true });
      this.applyPointCloudSubscription({ suppressLog: true });
      this.applyLogSubscription({ suppressLog: true });
      this.startTopicInventoryPolling();
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
      this.stopTopicInventoryPolling();
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
      gripperTfOffsetPublisher: new ROSLIB.Topic({
        ros,
        name: POINTAI_OFFSET_TOPIC,
        messageType: "geometry_msgs/Pose",
      }),
      cabinSpeedPublisher: new ROSLIB.Topic({
        ros,
        name: SET_CABIN_SPEED_TOPIC,
        messageType: "std_msgs/Float32",
      }),
      setGripperTfCalibrationService: new ROSLIB.Service({
        ros,
        name: SET_GRIPPER_TF_CALIBRATION_SERVICE,
        serviceType: SET_GRIPPER_TF_CALIBRATION_SERVICE_TYPE,
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
      startDriverStackService: new ROSLIB.Service({
        ros,
        name: START_DRIVER_STACK_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      restartDriverStackService: new ROSLIB.Service({
        ros,
        name: RESTART_DRIVER_STACK_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      startAlgorithmStackService: new ROSLIB.Service({
        ros,
        name: START_ALGORITHM_STACK_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      restartAlgorithmStackService: new ROSLIB.Service({
        ros,
        name: RESTART_ALGORITHM_STACK_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      restartRosStackService: new ROSLIB.Service({
        ros,
        name: RESTART_ROS_STACK_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      startCabinDriverService: new ROSLIB.Service({
        ros,
        name: START_CABIN_DRIVER_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      stopCabinDriverService: new ROSLIB.Service({
        ros,
        name: STOP_CABIN_DRIVER_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      restartCabinDriverService: new ROSLIB.Service({
        ros,
        name: RESTART_CABIN_DRIVER_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      stopCabinMotionService: new ROSLIB.Service({
        ros,
        name: STOP_CABIN_MOTION_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      startModuanDriverService: new ROSLIB.Service({
        ros,
        name: START_MODUAN_DRIVER_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      stopModuanDriverService: new ROSLIB.Service({
        ros,
        name: STOP_MODUAN_DRIVER_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      restartModuanDriverService: new ROSLIB.Service({
        ros,
        name: RESTART_MODUAN_DRIVER_SERVICE,
        serviceType: TRIGGER_SERVICE_TYPE,
      }),
      cabinSingleMoveService: new ROSLIB.Service({
        ros,
        name: CABIN_SINGLE_MOVE_SERVICE,
        serviceType: CABIN_SINGLE_MOVE_SERVICE_TYPE,
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

  startTopicInventoryPolling() {
    this.stopTopicInventoryPolling();
    this.fetchTopicInventory({ suppressLog: true });
    this.topicInventoryTimer = window.setInterval(() => {
      this.fetchTopicInventory({ suppressLog: true });
    }, 3000);
  }

  stopTopicInventoryPolling() {
    if (this.topicInventoryTimer) {
      window.clearInterval(this.topicInventoryTimer);
      this.topicInventoryTimer = null;
    }
  }

  fetchTopicInventory({ suppressLog = false } = {}) {
    if (!this.ros?.isConnected) {
      return;
    }
    this.ros.getTopicsAndRawTypes((result) => {
      const topics = Array.isArray(result?.topics) ? result.topics : [];
      const types = Array.isArray(result?.types) ? result.types : [];
      const inventory = topics.map((name, index) => ({
        name,
        type: types[index] || "unknown",
      })).sort((left, right) => left.name.localeCompare(right.name));
      this.callbacks.onTopicInventory?.(inventory);
    }, (error) => {
      if (!suppressLog) {
        this.callbacks.onLog?.(`读取当前话题列表失败: ${error}`, "warn");
      }
    });
  }

  publishGripperTfOffsetMm({ x, y, z }) {
    if (!this.ros?.isConnected || !this.resources?.gripperTfOffsetPublisher) {
      return { success: false, message: "ROS 未连接，无法发布外参热更新。" };
    }
    const message = new ROSLIB.Message({
      position: {
        x: Number.isFinite(x) ? x : 0,
        y: Number.isFinite(y) ? y : 0,
        z: Number.isFinite(z) ? z : 0,
      },
      orientation: {
        x: 0,
        y: 0,
        z: 0,
        w: 1,
      },
    });
    this.resources.gripperTfOffsetPublisher.publish(message);
    return { success: true, message: "已发布相机-TCP外参热更新请求。" };
  }

  publishCabinSpeed(speed) {
    if (!this.ros?.isConnected || !this.resources?.cabinSpeedPublisher) {
      return { success: false, message: "ROS 未连接，无法同步全局索驱速度。" };
    }

    const sanitizedSpeed = Number.isFinite(speed) && speed > 0 ? speed : 300;
    this.resources.cabinSpeedPublisher.publish(new ROSLIB.Message({ data: sanitizedSpeed }));
    return {
      success: true,
      message: `已同步全局索驱速度：${sanitizedSpeed}`,
      speed: sanitizedSpeed,
    };
  }

  callGripperTfCalibrationService({ x, y, z }) {
    if (!this.ros?.isConnected || !this.resources?.setGripperTfCalibrationService) {
      return Promise.resolve({ success: false, message: "ROS 未连接，无法调用相机-TCP外参热更新服务。" });
    }

    return new Promise((resolve) => {
      this.resources.setGripperTfCalibrationService.callService(
        new ROSLIB.ServiceRequest({
          x_mm: Number.isFinite(x) ? x : 0,
          y_mm: Number.isFinite(y) ? y : 0,
          z_mm: Number.isFinite(z) ? z : 0,
        }),
        (response) => {
          resolve({
            success: Boolean(response?.success),
            message: response?.message || "",
            applied: {
              x: Number(response?.applied_x_mm || 0),
              y: Number(response?.applied_y_mm || 0),
              z: Number(response?.applied_z_mm || 0),
            },
          });
        },
        (error) => {
          resolve({
            success: false,
            message: error?.message || String(error),
          });
        },
      );
    });
  }

  callCabinSingleMoveService({ x, y, z, speed }) {
    if (!this.ros?.isConnected || !this.resources?.cabinSingleMoveService) {
      return Promise.resolve({ success: false, message: "ROS 未连接，无法执行索驱单点运动。" });
    }

    return new Promise((resolve) => {
      this.resources.cabinSingleMoveService.callService(
        new ROSLIB.ServiceRequest({
          command: "单点运动请求",
          x: Number.isFinite(x) ? x : 0,
          y: Number.isFinite(y) ? y : 0,
          z: Number.isFinite(z) ? z : 0,
          speed: Number.isFinite(speed) ? speed : 0,
        }),
        (response) => {
          resolve({
            success: Boolean(response?.success),
            message: response?.message || "",
          });
        },
        (error) => {
          resolve({
            success: false,
            message: error?.message || String(error) || "索驱单点运动服务调用失败。",
          });
        },
      );
    });
  }

  callCabinMotionStopService() {
    if (!this.ros?.isConnected || !this.resources?.stopCabinMotionService) {
      return Promise.resolve({ success: false, message: "ROS 未连接，无法执行索驱停止运动。" });
    }

    return new Promise((resolve) => {
      this.resources.stopCabinMotionService.callService(
        new ROSLIB.ServiceRequest({}),
        (response) => {
          resolve({
            success: Boolean(response?.success),
            message: response?.message || "",
          });
        },
        (error) => {
          resolve({
            success: false,
            message: error?.message || String(error) || "索驱停止运动服务调用失败。",
          });
        },
      );
    });
  }

  callTriggerService(serviceName, serviceType = TRIGGER_SERVICE_TYPE) {
    const normalizedServiceName = String(serviceName || "").trim();
    if (!this.ros?.isConnected || !normalizedServiceName) {
      return Promise.resolve({ success: false, message: "ROS 未连接或服务地址为空，无法调用自定义服务。" });
    }

    const cacheKey = `${normalizedServiceName}|${serviceType}`;
    if (!this.dynamicServiceCache.has(cacheKey)) {
      this.dynamicServiceCache.set(cacheKey, new ROSLIB.Service({
        ros: this.ros,
        name: normalizedServiceName,
        serviceType,
      }));
    }

    const service = this.dynamicServiceCache.get(cacheKey);
    return new Promise((resolve) => {
      service.callService(
        new ROSLIB.ServiceRequest({}),
        (response) => {
          resolve({
            success: Boolean(response?.success),
            message: response?.message || "",
          });
        },
        (error) => {
          resolve({
            success: false,
            message: error?.message || String(error) || "自定义空请求服务调用失败。",
          });
        },
      );
    });
  }

  triggerWorkspaceS2Refresh() {
    if (!this.ros?.isConnected || !this.resources?.runWorkspaceS2Publisher) {
      return { success: false, message: "ROS 未连接，无法触发工作区识别刷新。" };
    }
    this.resources.runWorkspaceS2Publisher.publish(new ROSLIB.Message({ data: true }));
    return { success: true, message: "已触发基于当前工作区的识别刷新。" };
  }

  buildTopic(name, messageType, options = {}) {
    return new ROSLIB.Topic({
      ros: this.ros,
      name,
      messageType,
      ...options,
    });
  }

  bindSubscriptions() {
    this.unbindSubscriptions();
    const subscriptions = [
      this.buildTopic(CURRENT_WORKSPACE_QUAD_TOPIC, "std_msgs/Float32MultiArray"),
      this.buildTopic(EXECUTION_RESULT_TOPIC, "sensor_msgs/Image"),
      this.buildTopic(COORDINATE_POINT_TOPIC, "tie_robot_msgs/PointsArray"),
      this.buildTopic(PSEUDO_SLAM_MARKERS_TOPIC, "visualization_msgs/MarkerArray"),
      this.buildTopic(DIAGNOSTICS_TOPIC, "diagnostic_msgs/DiagnosticArray"),
      this.buildTopic(TF_TOPIC, "tf2_msgs/TFMessage"),
      this.buildTopic(TF_STATIC_TOPIC, "tf2_msgs/TFMessage"),
    ];

    subscriptions[0].subscribe((message) => this.callbacks.onSavedWorkspacePayload?.(Array.from(message.data || [])));
    subscriptions[1].subscribe((message) => this.callbacks.onExecutionOverlay?.(message));
    subscriptions[2].subscribe((message) => this.callbacks.onTiePoints?.(message));
    subscriptions[3].subscribe((message) => this.callbacks.onPlanningMarkers?.(message));
    subscriptions[4].subscribe((message) => this.callbacks.onDiagnostics?.(message));
    subscriptions[5].subscribe((message) => this.callbacks.onTfMessage?.(message));
    subscriptions[6].subscribe((message) => this.callbacks.onTfMessage?.(message));
    this.fixedTopicSubscribers = subscriptions;
  }

  getDisplayedImageMessageType(topicName) {
    if (
      topicName === IR_TOPIC ||
      topicName === COLOR_TOPIC ||
      topicName === DEPTH_TOPIC ||
      topicName === EXECUTION_RESULT_TOPIC ||
      topicName === FILTERED_WORLD_COORD_TOPIC ||
      topicName === RAW_WORLD_COORD_TOPIC
    ) {
      return "sensor_msgs/Image";
    }
    return "sensor_msgs/Image";
  }

  applyDisplayedImageSubscription({ suppressLog = false } = {}) {
    if (!this.ros?.isConnected) {
      return { changed: false, topic: this.desiredDisplayedImageTopic };
    }

    const nextTopicName = this.desiredDisplayedImageTopic || DEFAULT_IMAGE_TOPIC;
    const currentTopicName = this.displayedImageTopicSubscriber?.name || null;
    if (this.displayedImageTopicSubscriber && currentTopicName === nextTopicName) {
      return { changed: false, topic: nextTopicName };
    }

    if (this.displayedImageTopicSubscriber) {
      this.displayedImageTopicSubscriber.unsubscribe();
      this.displayedImageTopicSubscriber = null;
    }

    const topic = this.buildTopic(nextTopicName, this.getDisplayedImageMessageType(nextTopicName), {
      queue_length: 1,
    });
    topic.subscribe((message) => this.callbacks.onDisplayedImage?.(message, nextTopicName));
    this.displayedImageTopicSubscriber = topic;

    if (!suppressLog) {
      this.callbacks.onLog?.(`图像卡片已切换订阅：${nextTopicName}`, "info");
    }
    return { changed: true, topic: nextTopicName };
  }

  updateDisplayedImageSubscription(topicName) {
    this.desiredDisplayedImageTopic = topicName || DEFAULT_IMAGE_TOPIC;
    return this.applyDisplayedImageSubscription();
  }

  applyLogSubscription({ suppressLog = false } = {}) {
    if (!this.ros?.isConnected) {
      return { changed: false, topicId: this.desiredLogTopicId, topic: getLogTopicOption(this.desiredLogTopicId).topic };
    }

    const option = getLogTopicOption(this.desiredLogTopicId);
    const nextTopicName = option.topic;
    const currentTopicName = this.logTopicSubscriber?.name || null;
    if (this.logTopicSubscriber && currentTopicName === nextTopicName) {
      return { changed: false, topicId: option.id, topic: nextTopicName };
    }

    if (this.logTopicSubscriber) {
      this.logTopicSubscriber.unsubscribe();
      this.logTopicSubscriber = null;
    }

    const topic = this.buildTopic(nextTopicName, "rosgraph_msgs/Log", {
      queue_length: 30,
    });
    topic.subscribe((message) => this.callbacks.onSystemLog?.(message, option));
    this.logTopicSubscriber = topic;

    if (!suppressLog) {
      this.callbacks.onLog?.(`日志卡片已切换订阅：${option.label} (${nextTopicName})`, "info");
    }
    return { changed: true, topicId: option.id, topic: nextTopicName };
  }

  updateLogSubscription(topicId) {
    this.desiredLogTopicId = topicId || DEFAULT_LOG_TOPIC;
    return this.applyLogSubscription();
  }

  getPointCloudTopicName(source) {
    return source === "rawWorldCoord" ? RAW_WORLD_COORD_TOPIC : FILTERED_WORLD_COORD_TOPIC;
  }

  applyPointCloudSubscription({ suppressLog = false } = {}) {
    if (!this.ros?.isConnected) {
      return { changed: false, enabled: this.desiredPointCloudSubscription.enabled, source: this.desiredPointCloudSubscription.source };
    }

    const nextState = this.desiredPointCloudSubscription;
    const currentSource = this.pointCloudTopicSubscriber?.name
      ? (this.pointCloudTopicSubscriber.name === RAW_WORLD_COORD_TOPIC ? "rawWorldCoord" : "filteredWorldCoord")
      : null;

    if (!nextState.enabled) {
      if (this.pointCloudTopicSubscriber) {
        this.pointCloudTopicSubscriber.unsubscribe();
        this.pointCloudTopicSubscriber = null;
        if (!suppressLog) {
          this.callbacks.onLog?.("点云图层已关闭，已退订世界点云流", "info");
        }
        return { changed: true, enabled: false, source: nextState.source };
      }
      return { changed: false, enabled: false, source: nextState.source };
    }

    if (this.pointCloudTopicSubscriber && currentSource === nextState.source) {
      return { changed: false, enabled: true, source: nextState.source };
    }

    if (this.pointCloudTopicSubscriber) {
      this.pointCloudTopicSubscriber.unsubscribe();
      this.pointCloudTopicSubscriber = null;
    }

    const nextTopicName = this.getPointCloudTopicName(nextState.source);
    const topic = this.buildTopic(nextTopicName, "sensor_msgs/Image", {
      throttle_rate: 350,
      queue_length: 1,
    });
    topic.subscribe((message) => this.callbacks.onPointCloudImage?.(nextState.source, message));
    this.pointCloudTopicSubscriber = topic;

    if (!suppressLog) {
      const sourceLabel = nextState.source === "rawWorldCoord" ? "原始世界点云" : "滤波世界点云";
      this.callbacks.onLog?.(`点云图层已切换为按需订阅：${sourceLabel}（350ms 节流）`, "info");
    }
    return { changed: true, enabled: true, source: nextState.source };
  }

  updatePointCloudSubscription({ enabled, source }) {
    this.desiredPointCloudSubscription = {
      enabled: Boolean(enabled),
      source: source === "rawWorldCoord" ? "rawWorldCoord" : "filteredWorldCoord",
    };
    return this.applyPointCloudSubscription();
  }

  unbindSubscriptions() {
    this.fixedTopicSubscribers.forEach((topic) => {
      try {
        topic.unsubscribe();
      } catch {
        // ignore shutdown race
      }
    });
    this.fixedTopicSubscribers = [];
    if (this.pointCloudTopicSubscriber) {
      try {
        this.pointCloudTopicSubscriber.unsubscribe();
      } catch {
        // ignore shutdown race
      }
      this.pointCloudTopicSubscriber = null;
    }
    if (this.displayedImageTopicSubscriber) {
      try {
        this.displayedImageTopicSubscriber.unsubscribe();
      } catch {
        // ignore shutdown race
      }
      this.displayedImageTopicSubscriber = null;
    }
    if (this.logTopicSubscriber) {
      try {
        this.logTopicSubscriber.unsubscribe();
      } catch {
        // ignore shutdown race
      }
      this.logTopicSubscriber = null;
    }
    this.stopTopicInventoryPolling();
  }

  getResources() {
    return this.resources;
  }

  isReady() {
    return Boolean(this.resources && this.ros && this.ros.isConnected);
  }
}
