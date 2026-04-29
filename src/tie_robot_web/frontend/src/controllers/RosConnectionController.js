import { ROSLIB } from "../vendor/roslib.js";
import { resolveRosbridgeUrl } from "../utils/rosbridge.js";
import { DEFAULT_IMAGE_TOPIC } from "../config/imageTopicCatalog.js";
import { DEFAULT_LOG_TOPIC, getLogTopicOption } from "../config/logTopicCatalog.js";
import {
  ACTION_TYPES,
  ACTIONS,
  MESSAGE_TYPES,
  SERVICE_TYPES,
  SERVICES,
  TOPICS,
  getPointCloudTopicName,
  getTopicRegistryEntry,
  getTopicRegistryEntryByKey,
} from "../config/topicRegistry.js";

const AUTO_RECONNECT_MAX_ATTEMPTS = 3;
const AUTO_RECONNECT_DELAY_MS = 1500;

export class RosConnectionController {
  constructor(callbacks = {}) {
    this.callbacks = callbacks;
    this.ros = null;
    this.resources = null;
    this.connectionGeneration = 0;
    this.connectionLossGeneration = null;
    this.autoReconnectAttempts = 0;
    this.reconnectTimer = null;
    this.manualReconnectRequired = false;
    this.fixedTopicSubscribers = [];
    this.pointCloudTopicSubscriber = null;
    this.displayedImageTopicSubscriber = null;
    this.logTopicSubscriber = null;
    this.settingsLogTopicSubscriber = null;
    this.topicInventoryTimer = null;
    this.desiredPointCloudSubscription = {
      enabled: false,
      source: "filteredWorldCoord",
    };
    this.desiredDisplayedImageTopic = DEFAULT_IMAGE_TOPIC;
    this.desiredLogTopicId = DEFAULT_LOG_TOPIC;
  }

  connect({ manual = false, autoReconnect = false } = {}) {
    const rosbridgeUrl = resolveRosbridgeUrl({
      pathname: window.location.pathname,
      currentHost: window.location.hostname,
    });
    this.cancelReconnectTimer();
    if (!autoReconnect) {
      this.autoReconnectAttempts = 0;
    }
    this.manualReconnectRequired = false;

    const connectionMessage = manual
      ? `手动重新连接 ROSBridge: ${rosbridgeUrl}`
      : autoReconnect
        ? `正在自动重连 ROSBridge(${this.autoReconnectAttempts}/${AUTO_RECONNECT_MAX_ATTEMPTS}): ${rosbridgeUrl}`
        : `开始连接 ROSBridge: ${rosbridgeUrl}`;
    this.callbacks.onConnectionInfo?.(rosbridgeUrl, connectionMessage, autoReconnect ? "reconnecting" : "info");
    this.callbacks.onLog?.(connectionMessage, "info");

    const previousRos = this.ros;
    const generation = this.connectionGeneration + 1;
    this.connectionGeneration = generation;
    this.connectionLossGeneration = null;
    this.stopTopicInventoryPolling();
    this.unbindSubscriptions();
    if (previousRos?.close) {
      try {
        previousRos.close();
      } catch {
        // ignore stale socket cleanup races
      }
    }

    this.ros = new ROSLIB.Ros({ url: rosbridgeUrl });
    this.resources = this.buildResources(this.ros);

    this.ros.on("connection", () => {
      if (!this.isActiveGeneration(generation)) {
        return;
      }
      this.cancelReconnectTimer();
      this.autoReconnectAttempts = 0;
      this.manualReconnectRequired = false;
      this.resources.workspaceQuadPublisher.advertise();
      this.resources.cabinSpeedPublisher.advertise();
      this.resources.stableFrameCountPublisher.advertise();
      this.resources.linearModuleInterruptStopPublisher.advertise();
      this.bindSubscriptions();
      this.applySettingsLogSubscription();
      this.applyDisplayedImageSubscription({ suppressLog: true });
      this.applyPointCloudSubscription({ suppressLog: true });
      this.applyLogSubscription({ suppressLog: true });
      this.startTopicInventoryPolling();
      this.callbacks.onConnectionInfo?.(rosbridgeUrl, "连接成功", "success");
      this.callbacks.onRosReady?.(this.resources);
      this.callbacks.onLog?.("ROS 连接成功，动作链与图像订阅已就绪", "success");
    });

    this.ros.on("error", (error) => {
      if (!this.isActiveGeneration(generation)) {
        return;
      }
      const detail = error?.message || String(error);
      this.callbacks.onLog?.(`ROS 连接失败: ${detail}`, "error");
      this.handleConnectionLoss(generation, rosbridgeUrl);
    });

    this.ros.on("close", () => {
      if (!this.isActiveGeneration(generation)) {
        return;
      }
      this.callbacks.onLog?.("ROS 连接已断开", "warn");
      this.handleConnectionLoss(generation, rosbridgeUrl);
    });
  }

  isActiveGeneration(generation) {
    return generation === this.connectionGeneration;
  }

  handleConnectionLoss(generation, rosbridgeUrl) {
    if (!this.isActiveGeneration(generation) || this.connectionLossGeneration === generation) {
      return;
    }
    this.connectionLossGeneration = generation;
    this.stopTopicInventoryPolling();
    this.unbindSubscriptions();
    this.callbacks.onRosUnavailable?.();
    if (this.autoReconnectAttempts >= AUTO_RECONNECT_MAX_ATTEMPTS) {
      this.showManualReconnectRequired(rosbridgeUrl);
      return;
    }
    this.scheduleReconnect(rosbridgeUrl);
  }

  scheduleReconnect(rosbridgeUrl) {
    this.cancelReconnectTimer();
    this.autoReconnectAttempts += 1;
    const message = `ROS 连接中断，${AUTO_RECONNECT_DELAY_MS / 1000}秒后自动重连(${this.autoReconnectAttempts}/${AUTO_RECONNECT_MAX_ATTEMPTS})。`;
    this.callbacks.onConnectionInfo?.(rosbridgeUrl, message, "reconnecting");
    this.callbacks.onLog?.(message, "warn");
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect({ autoReconnect: true });
    }, AUTO_RECONNECT_DELAY_MS);
  }

  showManualReconnectRequired(rosbridgeUrl) {
    this.manualReconnectRequired = true;
    const message = `ROS 自动重连${AUTO_RECONNECT_MAX_ATTEMPTS}次失败，请点击手动重连。`;
    this.callbacks.onConnectionInfo?.(rosbridgeUrl, message, "manual");
    this.callbacks.onLog?.(message, "error");
  }

  cancelReconnectTimer() {
    if (!this.reconnectTimer) {
      return;
    }
    window.clearTimeout(this.reconnectTimer);
    this.reconnectTimer = null;
  }

  buildResources(ros) {
    return {
      ros,
      workspaceQuadPublisher: new ROSLIB.Topic({
        ros,
        name: TOPICS.algorithm.setWorkspaceQuad,
        messageType: MESSAGE_TYPES.float32MultiArray,
      }),
      cabinSpeedPublisher: new ROSLIB.Topic({
        ros,
        name: TOPICS.process.setCabinSpeed,
        messageType: MESSAGE_TYPES.float32,
      }),
      setGripperTfCalibrationService: new ROSLIB.Service({
        ros,
        name: SERVICES.tf.setGripperTfCalibration,
        serviceType: SERVICE_TYPES.tf.setGripperTfCalibration,
      }),
      stableFrameCountPublisher: new ROSLIB.Topic({
        ros,
        name: TOPICS.algorithm.setStableFrameCount,
        messageType: MESSAGE_TYPES.int32,
      }),
      lashingRecognizeOnceService: new ROSLIB.Service({
        ros,
        name: SERVICES.algorithm.recognizeOnce,
        serviceType: SERVICE_TYPES.trigger,
      }),
      processImageService: new ROSLIB.Service({
        ros,
        name: SERVICES.algorithm.processImage,
        serviceType: SERVICE_TYPES.algorithm.processImage,
      }),
      startPseudoSlamScanActionClient: new ROSLIB.ActionClient({
        ros,
        serverName: ACTIONS.cabin.startPseudoSlamScan,
        actionName: ACTION_TYPES.cabin.startPseudoSlamScan,
      }),
      executionModeService: new ROSLIB.Service({
        ros,
        name: SERVICES.cabin.setExecutionMode,
        serviceType: SERVICE_TYPES.cabin.setExecutionMode,
      }),
      startDriverStackService: new ROSLIB.Service({
        ros,
        name: SERVICES.system.startDriverStack,
        serviceType: SERVICE_TYPES.trigger,
      }),
      restartDriverStackService: new ROSLIB.Service({
        ros,
        name: SERVICES.system.restartDriverStack,
        serviceType: SERVICE_TYPES.trigger,
      }),
      startAlgorithmStackService: new ROSLIB.Service({
        ros,
        name: SERVICES.system.startAlgorithmStack,
        serviceType: SERVICE_TYPES.trigger,
      }),
      restartAlgorithmStackService: new ROSLIB.Service({
        ros,
        name: SERVICES.system.restartAlgorithmStack,
        serviceType: SERVICE_TYPES.trigger,
      }),
      restartRosStackService: new ROSLIB.Service({
        ros,
        name: SERVICES.system.restartRosStack,
        serviceType: SERVICE_TYPES.trigger,
      }),
      startCabinDriverService: new ROSLIB.Service({
        ros,
        name: SERVICES.cabin.driverStart,
        serviceType: SERVICE_TYPES.trigger,
      }),
      stopCabinDriverService: new ROSLIB.Service({
        ros,
        name: SERVICES.cabin.driverStop,
        serviceType: SERVICE_TYPES.trigger,
      }),
      restartCabinDriverService: new ROSLIB.Service({
        ros,
        name: SERVICES.cabin.driverRestart,
        serviceType: SERVICE_TYPES.trigger,
      }),
      stopCabinMotionService: new ROSLIB.Service({
        ros,
        name: SERVICES.cabin.motionStop,
        serviceType: SERVICE_TYPES.trigger,
      }),
      startModuanDriverService: new ROSLIB.Service({
        ros,
        name: SERVICES.moduan.driverStart,
        serviceType: SERVICE_TYPES.trigger,
      }),
      stopModuanDriverService: new ROSLIB.Service({
        ros,
        name: SERVICES.moduan.driverStop,
        serviceType: SERVICE_TYPES.trigger,
      }),
      restartModuanDriverService: new ROSLIB.Service({
        ros,
        name: SERVICES.moduan.driverRestart,
        serviceType: SERVICE_TYPES.trigger,
      }),
      cabinSingleMoveService: new ROSLIB.Service({
        ros,
        name: SERVICES.cabin.singleMove,
        serviceType: SERVICE_TYPES.cabin.singleMove,
      }),
      linearModuleSingleMoveService: new ROSLIB.Service({
        ros,
        name: SERVICES.moduan.singleMove,
        serviceType: SERVICE_TYPES.moduan.linearModuleMove,
      }),
      linearModuleInterruptStopPublisher: new ROSLIB.Topic({
        ros,
        name: TOPICS.control.interruptStop,
        messageType: MESSAGE_TYPES.float32,
      }),
      singlePointBindService: new ROSLIB.Service({
        ros,
        name: SERVICES.moduan.singleBind,
        serviceType: SERVICE_TYPES.trigger,
      }),
      startGlobalWorkActionClient: new ROSLIB.ActionClient({
        ros,
        serverName: ACTIONS.cabin.startGlobalWork,
        actionName: ACTION_TYPES.cabin.startGlobalWork,
      }),
      runDirectBindPathTestActionClient: new ROSLIB.ActionClient({
        ros,
        serverName: ACTIONS.cabin.runBindPathDirectTest,
        actionName: ACTION_TYPES.cabin.runBindPathDirectTest,
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
        registry: getTopicRegistryEntry(name),
      })).sort((left, right) => left.name.localeCompare(right.name));
      this.callbacks.onTopicInventory?.(inventory);
    }, (error) => {
      if (!suppressLog) {
        this.callbacks.onLog?.(`读取当前话题列表失败: ${error}`, "warn");
      }
    });
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

  callLinearModuleSingleMoveService({ x, y, z, angle }) {
    if (!this.ros?.isConnected || !this.resources?.linearModuleSingleMoveService) {
      return Promise.resolve({ success: false, message: "ROS 未连接，无法执行 TCP 线性模组移动。" });
    }

    return new Promise((resolve) => {
      this.resources.linearModuleSingleMoveService.callService(
        new ROSLIB.ServiceRequest({
          pos_x: Number.isFinite(x) ? x : 0,
          pos_y: Number.isFinite(y) ? y : 0,
          pos_z: Number.isFinite(z) ? z : 0,
          angle: Number.isFinite(angle) ? angle : 0,
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
            message: error?.message || String(error) || "TCP 线性模组单点移动服务调用失败。",
          });
        },
      );
    });
  }

  publishLinearModuleInterruptStop() {
    if (!this.ros?.isConnected || !this.resources?.linearModuleInterruptStopPublisher) {
      return Promise.resolve({ success: false, message: "ROS 未连接，无法暂停 TCP 线性模组运动。" });
    }
    this.resources.linearModuleInterruptStopPublisher.publish(new ROSLIB.Message({ data: 1 }));
    return Promise.resolve({ success: true, message: "TCP 线性模组运动暂停信号已发送。" });
  }

  callSinglePointBindService() {
    if (!this.ros?.isConnected || !this.resources?.singlePointBindService) {
      return Promise.resolve({ success: false, message: "ROS 未连接，无法触发单点绑扎服务。" });
    }

    return new Promise((resolve) => {
      this.resources.singlePointBindService.callService(
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
            message: error?.message || String(error) || "单点绑扎服务调用失败。",
          });
        },
      );
    });
  }

  triggerWorkspaceS2Refresh() {
    return this.callLashingRecognizeOnceService();
  }

  callLashingRecognizeOnceService() {
    if (!this.ros?.isConnected || !this.resources?.lashingRecognizeOnceService) {
      return Promise.resolve({ success: false, message: "ROS 未连接，无法触发视觉识别刷新。" });
    }

    return new Promise((resolve) => {
      this.resources.lashingRecognizeOnceService.callService(
        new ROSLIB.ServiceRequest({}),
        (response) => {
          resolve({
            success: Boolean(response?.success),
            message: response?.message || (response?.success ? "视觉识别完成。" : "视觉识别失败。"),
          });
        },
        (error) => {
          resolve({
            success: false,
            message: error?.message || String(error) || "视觉识别服务调用失败。",
          });
        },
      );
    });
  }

  publishStableFrameCount(frameCount) {
    if (!this.ros?.isConnected || !this.resources?.stableFrameCountPublisher) {
      return { success: false, message: "ROS 未连接，无法设置视觉服务放行帧数。" };
    }
    const sanitizedFrameCount = Math.max(1, Math.round(Number(frameCount) || 1));
    this.resources.stableFrameCountPublisher.publish(new ROSLIB.Message({ data: sanitizedFrameCount }));
    return {
      success: true,
      frameCount: sanitizedFrameCount,
      message: `视觉服务最终放行帧数已设置为 ${sanitizedFrameCount} 帧。`,
    };
  }

  callProcessImageService({ requestMode = 1 } = {}) {
    if (!this.ros?.isConnected || !this.resources?.processImageService) {
      return Promise.resolve({ success: false, message: "ROS 未连接，无法调用视觉服务。", serviceElapsedMs: null });
    }

    const startedAt = performance.now();
    const sanitizedRequestMode = Math.max(0, Math.round(Number(requestMode) || 0));
    return new Promise((resolve) => {
      this.resources.processImageService.callService(
        new ROSLIB.ServiceRequest({ request_mode: sanitizedRequestMode }),
        (response) => {
          const serviceElapsedMs = Number((performance.now() - startedAt).toFixed(1));
          const responseMessage = response?.message || "";
          const singleFrameMatch = responseMessage.match(/单帧视觉耗时=([0-9.]+)ms/);
          resolve({
            success: Boolean(response?.success),
            message: responseMessage,
            count: Number(response?.count || 0),
            requestMode: sanitizedRequestMode,
            serviceElapsedMs,
            singleFrameElapsedMs: singleFrameMatch ? Number(singleFrameMatch[1]) : null,
          });
        },
        (error) => {
          const serviceElapsedMs = Number((performance.now() - startedAt).toFixed(1));
          resolve({
            success: false,
            message: error?.message || String(error) || "视觉服务调用失败。",
            count: 0,
            requestMode: sanitizedRequestMode,
            serviceElapsedMs,
            singleFrameElapsedMs: null,
          });
        },
      );
    });
  }

  buildTopic(name, messageType, options = {}) {
    return new ROSLIB.Topic({
      ros: this.ros,
      name,
      messageType,
      ...options,
    });
  }

  buildTopicFromRegistry(registryKey, options = {}) {
    const entry = getTopicRegistryEntryByKey(registryKey);
    if (!entry) {
      throw new Error(`未知前端话题注册项：${registryKey}`);
    }
    return this.buildTopic(entry.name, entry.messageType, options);
  }

  bindSubscriptions() {
    this.unbindSubscriptions();
    const subscriptions = [
      this.buildTopicFromRegistry("algorithm.currentWorkspaceQuadPixels"),
      this.buildTopicFromRegistry("algorithm.resultImageRaw"),
      this.buildTopicFromRegistry("algorithm.manualWorkspaceS2ResultRaw"),
      this.buildTopicFromRegistry("algorithm.manualWorkspaceS2Points"),
      this.buildTopicFromRegistry("algorithm.coordinatePoint"),
      this.buildTopicFromRegistry("process.pseudoSlamMarkers"),
      this.buildTopicFromRegistry("process.diagnostics"),
      this.buildTopicFromRegistry("tf.live"),
      this.buildTopicFromRegistry("tf.static"),
      this.buildTopicFromRegistry("control.linearModuleState"),
    ];

    subscriptions[0].subscribe((message) => this.callbacks.onSavedWorkspacePayload?.(Array.from(message.data || [])));
    subscriptions[1].subscribe((message) => this.callbacks.onExecutionOverlay?.(message));
    subscriptions[2].subscribe((message) => this.callbacks.onWorkspaceS2Overlay?.(message));
    subscriptions[3].subscribe((message) => this.callbacks.onVisualRecognitionPoints?.(message));
    subscriptions[4].subscribe((message) => this.callbacks.onTiePoints?.(message));
    subscriptions[5].subscribe((message) => this.callbacks.onPlanningMarkers?.(message));
    subscriptions[6].subscribe((message) => this.callbacks.onDiagnostics?.(message));
    subscriptions[7].subscribe((message) => this.callbacks.onTfMessage?.(message));
    subscriptions[8].subscribe((message) => this.callbacks.onTfMessage?.(message));
    subscriptions[9].subscribe((message) => this.callbacks.onLinearModuleState?.(message));
    this.fixedTopicSubscribers = subscriptions;
  }

  getDisplayedImageMessageType(topicName) {
    return getTopicRegistryEntry(topicName)?.messageType || MESSAGE_TYPES.image;
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

    const topic = this.buildTopic(nextTopicName, MESSAGE_TYPES.log, {
      queue_length: 30,
    });
    topic.subscribe((message) => this.callbacks.onSystemLog?.(message, option));
    this.logTopicSubscriber = topic;

    if (!suppressLog) {
      this.callbacks.onLog?.(`日志卡片已切换订阅：${option.label} (${nextTopicName})`, "info");
    }
    return { changed: true, topicId: option.id, topic: nextTopicName };
  }

  applySettingsLogSubscription() {
    if (!this.ros?.isConnected || this.settingsLogTopicSubscriber) {
      return;
    }

    const topic = this.buildTopic(TOPICS.logs.all, MESSAGE_TYPES.log, {
      queue_length: 80,
    });
    topic.subscribe((message) => this.callbacks.onLayerSystemLog?.(message));
    this.settingsLogTopicSubscriber = topic;
  }

  updateLogSubscription(topicId) {
    this.desiredLogTopicId = topicId || DEFAULT_LOG_TOPIC;
    return this.applyLogSubscription();
  }

  applyPointCloudSubscription({ suppressLog = false } = {}) {
    if (!this.ros?.isConnected) {
      return { changed: false, enabled: this.desiredPointCloudSubscription.enabled, source: this.desiredPointCloudSubscription.source };
    }

    const nextState = this.desiredPointCloudSubscription;
    const currentSource = this.pointCloudTopicSubscriber?.name
      ? (this.pointCloudTopicSubscriber.name === getPointCloudTopicName("rawWorldCoord") ? "rawWorldCoord" : "filteredWorldCoord")
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

    const nextTopicName = getPointCloudTopicName(nextState.source);
    const topic = this.buildTopic(nextTopicName, MESSAGE_TYPES.image, {
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
    if (this.settingsLogTopicSubscriber) {
      try {
        this.settingsLogTopicSubscriber.unsubscribe();
      } catch {
        // ignore shutdown race
      }
      this.settingsLogTopicSubscriber = null;
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
