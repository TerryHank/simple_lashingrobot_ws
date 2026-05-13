import { ROSLIB } from "../vendor/roslib.js";
import {
  DEFAULT_GLOBAL_EXECUTION_MODE,
  FRONTEND_VISUAL_RECOGNITION_FULL_LABEL,
  FRONTEND_VISUAL_RECOGNITION_MODE_LABEL,
  GLOBAL_EXECUTION_MODE_OPTIONS,
  PROCESS_IMAGE_REQUEST_MODES,
} from "../config/visualRecognitionMode.js";
import { buildWorkspaceQuadPayload } from "../utils/irImageUtils.js";

const WORKSPACE_QUAD_ACK_TIMEOUT_MS = 4000;
const DEFAULT_BIND_EXECUTION_CABIN_MIN_Z_MM = 485;
const DEFAULT_BIND_GROUP_ROW_THRESHOLD_MM = 40;
const DEFAULT_BIND_GROUP_COLUMN_THRESHOLD_MM = 45;
const BIND_EXECUTION_CABIN_Z_MODE_FIXED = 0;
const BIND_EXECUTION_CABIN_Z_MODE_MIN = 1;

function buildWorkspaceQuadPayloadKey(payload) {
  if (!Array.isArray(payload) || payload.length !== 8) {
    return "";
  }

  const pairs = [];
  for (let index = 0; index < payload.length; index += 2) {
    pairs.push(`${Math.round(Number(payload[index]) || 0)},${Math.round(Number(payload[index + 1]) || 0)}`);
  }
  return pairs.sort().join("|");
}

function normalizeAdaptiveBindGrouping(value) {
  return Boolean(value);
}

function normalizeBindExecutionCabinMinZ(value) {
  const numericValue = Number(value);
  return Number.isFinite(numericValue) && numericValue >= 0
    ? numericValue
    : DEFAULT_BIND_EXECUTION_CABIN_MIN_Z_MM;
}

function normalizeBindGroupAxisThreshold(value, fallback) {
  const numericValue = Number(value);
  return Number.isFinite(numericValue) && numericValue > 0
    ? numericValue
    : fallback;
}

function normalizeBindExecutionCabinZMode(value) {
  return value === "min" || Number(value) === BIND_EXECUTION_CABIN_Z_MODE_MIN
    ? BIND_EXECUTION_CABIN_Z_MODE_MIN
    : BIND_EXECUTION_CABIN_Z_MODE_FIXED;
}

function normalizeRecognitionPoseIndex(value) {
  const numericValue = Number(value);
  const roundedValue = Number.isFinite(numericValue) ? Math.round(numericValue) : 1;
  return roundedValue >= 1 ? roundedValue : 1;
}

function normalizeGlobalExecutionMode(value) {
  const numericValue = Number(value);
  const roundedValue = Number.isFinite(numericValue) ? Math.round(numericValue) : DEFAULT_GLOBAL_EXECUTION_MODE;
  return GLOBAL_EXECUTION_MODE_OPTIONS.some((option) => option.id === roundedValue)
    ? roundedValue
    : DEFAULT_GLOBAL_EXECUTION_MODE;
}

export class TaskActionController {
  constructor({
    rosConnection,
    workspaceView,
    getExecutionMode = null,
    getAdaptiveBindGrouping = null,
    getBindGroupRowThreshold = null,
    getBindGroupColumnThreshold = null,
    getBindExecutionCabinMinZ = null,
    getBindExecutionCabinZMode = null,
    getRecognitionPoseIndex = null,
    callbacks = {},
  }) {
    this.rosConnection = rosConnection;
    this.workspaceView = workspaceView;
    this.getExecutionMode = getExecutionMode;
    this.getAdaptiveBindGrouping = getAdaptiveBindGrouping;
    this.getBindGroupRowThreshold = getBindGroupRowThreshold;
    this.getBindGroupColumnThreshold = getBindGroupColumnThreshold;
    this.getBindExecutionCabinMinZ = getBindExecutionCabinMinZ;
    this.getBindExecutionCabinZMode = getBindExecutionCabinZMode;
    this.getRecognitionPoseIndex = getRecognitionPoseIndex;
    this.callbacks = callbacks;
    this.pendingWorkspaceQuadSubmission = null;
  }

  handle(taskAction) {
    switch (taskAction) {
      case "submitQuad":
        return this.publishWorkspaceQuad();
      case "runSavedS2":
        return this.triggerSavedWorkspaceS2();
      case "executionVisionOnly":
        return this.triggerExecutionRefineVisionOnly();
      case "triggerSingleBind":
        return this.triggerSinglePointBind();
      case "startExecution":
        return this.triggerExecutionLayer({ useExecutionMemory: false, clearExecutionMemory: false });
      case "startExecutionKeepMemory":
        return this.triggerExecutionLayer({ useExecutionMemory: true, clearExecutionMemory: false });
      default:
        this.report(`未识别的任务动作: ${taskAction}`, "warn");
    }
  }

  publishWorkspaceQuad() {
    const resources = this.rosConnection.getResources();
    const selectedPoints = this.workspaceView.getSelectedPoints();
    if (!resources?.workspaceQuadPublisher || selectedPoints.length !== 4) {
      this.report("当前还不能确认工作区域，请先连上 ROS 并点满 4 个角点", "warn");
      return;
    }
    const payload = buildWorkspaceQuadPayload(selectedPoints);
    this.workspaceView.setExecutionOverlayMessage(null);
    this.setPendingWorkspaceQuadSubmission(payload, {
      recognitionPoseIndex: normalizeRecognitionPoseIndex(this.getRecognitionPoseIndex?.()),
    });
    resources.workspaceQuadPublisher.publish(new ROSLIB.Message({ data: payload }));
    this.callbacks.onResultMessage?.(`工作区域已发送，等待 pointAI 保存确认: [${payload.join(", ")}]`);
    this.callbacks.onLog?.(`已发送工作区域: [${payload.join(", ")}]`, "success");
  }

  triggerSavedWorkspaceS2() {
    return this.triggerSurfaceDpRecognition();
  }

  async triggerSurfaceDpRecognition({
    resultMessage = null,
    logMessage = null,
    recognitionPoseIndex: explicitRecognitionPoseIndex = null,
  } = {}) {
    const resources = this.rosConnection.getResources();
    const recognitionPoseIndex = normalizeRecognitionPoseIndex(
      explicitRecognitionPoseIndex ?? this.getRecognitionPoseIndex?.(),
    );
    const recognitionPoseLabel = `识别位姿 ${recognitionPoseIndex}`;
    const effectiveResultMessage =
      resultMessage ||
      `正在触发当前画面无运动视觉记录的${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别；` +
        `完成后会覆盖${recognitionPoseLabel} 的大组，并保留其他识别位姿数据。`;
    const effectiveLogMessage =
      logMessage ||
      `已触发${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别，将更新${recognitionPoseLabel}的大组并保留其他识别位姿数据`;
    if (!resources?.startPseudoSlamScanActionClient) {
      this.report("ROS 还没连好，暂时不能触发视觉识别并更新当前识别位姿大组。", "warn");
      return false;
    }
    this.workspaceView.setExecutionOverlayMessage(null);
    this.callbacks.onWorkspaceS2Triggered?.();
    this.callbacks.onResultMessage?.(effectiveResultMessage);
    this.callbacks.onLog?.(effectiveLogMessage, "success");

    const result = await this.sendActionGoal(resources.startPseudoSlamScanActionClient, {
      goalMessage: {
        enable_capture_gate: false,
        scan_strategy: 3,
        recognition_pose_index: recognitionPoseIndex,
        bind_group_point_count:
          normalizeAdaptiveBindGrouping(this.getAdaptiveBindGrouping?.()) ? 0 : 4,
        bind_group_row_threshold_mm:
          normalizeBindGroupAxisThreshold(
            this.getBindGroupRowThreshold?.(),
            DEFAULT_BIND_GROUP_ROW_THRESHOLD_MM,
          ),
        bind_group_column_threshold_mm:
          normalizeBindGroupAxisThreshold(
            this.getBindGroupColumnThreshold?.(),
            DEFAULT_BIND_GROUP_COLUMN_THRESHOLD_MM,
          ),
        bind_execution_cabin_min_z_mm:
          normalizeBindExecutionCabinMinZ(this.getBindExecutionCabinMinZ?.()),
        bind_execution_cabin_z_mode:
          normalizeBindExecutionCabinZMode(this.getBindExecutionCabinZMode?.()),
      },
      feedbackPrefix: "视觉识别建图进行中",
      successPrefix: "视觉识别建图完成",
      failurePrefix: `${FRONTEND_VISUAL_RECOGNITION_FULL_LABEL}失败`,
    });
    const success = Boolean(result?.success);
    if (success) {
      this.callbacks.onSurfaceDpRecognitionFinished?.({ recognitionPoseIndex, result });
    }
    return success;
  }

  async triggerSinglePointBind() {
    const resources = this.rosConnection.getResources();
    const savedPoints = this.workspaceView.getSavedWorkspacePoints();
    if (!resources?.singlePointBindService || savedPoints.length !== 4) {
      if (this.pendingWorkspaceQuadSubmission) {
        this.report("工作区正在保存，请等保存完成后再触发单点绑扎。", "warn");
        return;
      }
      this.report("当前没有可复用的已保存工作区，请先点 4 个角点并确认工作区域。", "warn");
      return;
    }

    this.clearPendingWorkspaceQuadSubmission();
    this.workspaceView.setExecutionOverlayMessage(null);
    this.callbacks.onWorkspaceS2Triggered?.();
    this.callbacks.onResultMessage?.("正在触发单点绑扎：后端会在同一个原子链路内完成视觉识别、线性模组执行和完成信号等待。");
    this.callbacks.onLog?.("已触发单点绑扎原子服务", "success");
    const result = await this.rosConnection.callSinglePointBindService();
    if (!result?.success) {
      this.report(`单点绑扎失败: ${result?.message || "未知错误"}`, "error");
      return;
    }
    this.report(`单点绑扎完成: ${result.message || "末端已完成当前视觉点位绑扎"}`, "success");
  }

  async triggerExecutionRefineVisionOnly() {
    const resources = this.rosConnection.getResources();
    if (!resources?.processImageService) {
      this.report("ROS 还没连好，暂时不能触发执行层视觉单侧。", "warn");
      return false;
    }

    this.workspaceView.setExecutionOverlayMessage(null);
    this.callbacks.onResultMessage?.(
      "正在触发执行层视觉单侧：只请求一次平面去除 + Hough 视觉，不执行线性模组单点绑扎。",
    );
    this.callbacks.onLog?.("已触发执行层视觉单侧 process_image mode=4", "success");
    const result = await this.rosConnection.callProcessImageService({
      requestMode: PROCESS_IMAGE_REQUEST_MODES.EXECUTION_REFINE,
    });
    const level = result?.success ? "success" : "error";
    const pointCount = Number(result?.count || 0);
    this.report(
      `执行层视觉单侧${result?.success ? "完成" : "失败"}: 点数=${pointCount}，${result?.message || "无消息"}`,
      level,
    );
    return Boolean(result?.success);
  }

  handleSavedWorkspacePayload(payload) {
    return this.confirmPendingWorkspaceQuadSubmission(payload);
  }

  isPendingWorkspacePayload(payload) {
    if (!this.pendingWorkspaceQuadSubmission) {
      return false;
    }
    const payloadKey = buildWorkspaceQuadPayloadKey(payload);
    return Boolean(payloadKey && payloadKey === this.pendingWorkspaceQuadSubmission.payloadKey);
  }

  setPendingWorkspaceQuadSubmission(payload, { recognitionPoseIndex = null } = {}) {
    this.clearPendingWorkspaceQuadSubmission();
    const payloadKey = buildWorkspaceQuadPayloadKey(payload);
    if (!payloadKey) {
      return;
    }

    const timeoutId = window.setTimeout(() => {
      if (!this.pendingWorkspaceQuadSubmission || this.pendingWorkspaceQuadSubmission.payloadKey !== payloadKey) {
        return;
      }
      this.pendingWorkspaceQuadSubmission = null;
      this.report("工作区域已发送，但等待 pointAI 保存确认超时。请检查视觉日志后重试。", "warn");
    }, WORKSPACE_QUAD_ACK_TIMEOUT_MS);

    this.pendingWorkspaceQuadSubmission = {
      payloadKey,
      recognitionPoseIndex: normalizeRecognitionPoseIndex(recognitionPoseIndex),
      timeoutId,
    };
  }

  clearPendingWorkspaceQuadSubmission() {
    if (this.pendingWorkspaceQuadSubmission?.timeoutId) {
      window.clearTimeout(this.pendingWorkspaceQuadSubmission.timeoutId);
    }
    this.pendingWorkspaceQuadSubmission = null;
  }

  confirmPendingWorkspaceQuadSubmission(payload) {
    const pendingSubmission = this.pendingWorkspaceQuadSubmission;
    if (!pendingSubmission) {
      return false;
    }

    const payloadKey = buildWorkspaceQuadPayloadKey(payload);
    if (!payloadKey || payloadKey !== pendingSubmission.payloadKey) {
      return false;
    }

    const resources = this.rosConnection.getResources();
    if (!resources?.startPseudoSlamScanActionClient) {
      this.clearPendingWorkspaceQuadSubmission();
      this.report("工作区已保存，但 ROS 未就绪，当前不能触发视觉识别并更新当前识别位姿大组。", "warn");
      return false;
    }

    this.clearPendingWorkspaceQuadSubmission();
    void this.triggerSurfaceDpRecognition({
      recognitionPoseIndex: pendingSubmission.recognitionPoseIndex,
      resultMessage:
        `工作区已保存，正在自动触发当前画面无运动视觉记录的${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别；` +
        "完成后会覆盖当前识别位姿的大组，并保留其他识别位姿数据。",
      logMessage:
        `已收到工作区保存确认，自动触发${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别并更新当前识别位姿大组`,
    }).catch((error) => {
      this.report(`自动触发视觉识别失败: ${error?.message || String(error)}`, "error");
    });
    return true;
  }

  triggerExecutionLayer({ useExecutionMemory = false, clearExecutionMemory = false } = {}) {
    const resources = this.rosConnection.getResources();
    if (!resources?.executionModeService || !resources?.startGlobalWorkActionClient) {
      this.report("ROS 还没连好，暂时不能开始执行层", "warn");
      return;
    }
    this.workspaceView.setExecutionOverlayMessage(null);
    this.callbacks.onResultMessage?.("执行结果会叠加到红外原图。");
    this.callbacks.onLog?.(
      !useExecutionMemory
        ? "准备开始执行层（执行记忆关闭）"
        : clearExecutionMemory
          ? "准备清记忆并开始执行层"
          : "准备按执行记忆续跑",
      "success",
    );
    const executionMode = normalizeGlobalExecutionMode(this.getExecutionMode?.());
    const request = new ROSLIB.ServiceRequest({ execution_mode: executionMode });
    resources.executionModeService.callService(
      request,
      (response) => {
        if (!response?.success) {
          this.report(`执行模式切换失败: ${response?.message || "未知错误"}`, "error");
          return;
        }
        this.sendActionGoal(resources.startGlobalWorkActionClient, {
          goalMessage: {
            clear_execution_memory: clearExecutionMemory,
            use_execution_memory: useExecutionMemory,
            execution_mode: executionMode,
          },
          feedbackPrefix: "执行层进行中",
          successPrefix: "执行层任务完成",
          failurePrefix: "执行层任务失败",
        });
      },
      (error) => this.report(`执行模式切换失败: ${error?.message || String(error)}`, "error"),
    );
  }

  sendActionGoal(actionClient, { goalMessage, feedbackPrefix, successPrefix, failurePrefix }) {
    return new Promise((resolve) => {
      let settled = false;
      const settle = (result) => {
        if (settled) {
          return;
        }
        settled = true;
        resolve(result);
      };

      let goal;
      try {
        goal = new ROSLIB.Goal({ actionClient, goalMessage });
      } catch (error) {
        const message = error?.message || String(error);
        this.report(`${failurePrefix}: ${message}`, "error");
        settle({ success: false, message });
        return;
      }

      goal.on("feedback", (feedback) => {
        const detail = feedback?.detail || feedback?.stage || "处理中";
        this.callbacks.onResultMessage?.(`${feedbackPrefix}: ${detail}`);
      });
      goal.on("result", (result) => {
        const success = Boolean(result?.success);
        const message = result?.message || "未知结果";
        if (success) {
          this.callbacks.onResultMessage?.(`${successPrefix}: ${message}`);
          this.callbacks.onLog?.(`${successPrefix}: ${message}`, "success");
          settle({ success: true, message, result });
          return;
        }
        this.callbacks.onResultMessage?.(`${failurePrefix}: ${message}`);
        this.callbacks.onLog?.(`${failurePrefix}: ${message}`, "error");
        settle({ success: false, message, result });
      });

      try {
        goal.send();
      } catch (error) {
        const message = error?.message || String(error);
        this.report(`${failurePrefix}: ${message}`, "error");
        settle({ success: false, message });
      }
    });
  }

  report(message, level = "info") {
    this.callbacks.onResultMessage?.(message);
    this.callbacks.onLog?.(message, level);
  }
}
