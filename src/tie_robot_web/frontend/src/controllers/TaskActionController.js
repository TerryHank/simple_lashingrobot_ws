import { ROSLIB } from "../vendor/roslib.js";
import {
  FRONTEND_VISUAL_RECOGNITION_FULL_LABEL,
  FRONTEND_VISUAL_RECOGNITION_MODE_LABEL,
} from "../config/visualRecognitionMode.js";
import { buildWorkspaceQuadPayload } from "../utils/irImageUtils.js";

const WORKSPACE_QUAD_ACK_TIMEOUT_MS = 4000;
const DEFAULT_FIXED_SCAN_POSE_MM = Object.freeze({ x: 490, y: 1700, z: 3197 });

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

function normalizeFixedScanPoseMm(pose) {
  const source = pose || DEFAULT_FIXED_SCAN_POSE_MM;
  const normalized = {
    x: Number(source.x),
    y: Number(source.y),
    z: Number(source.z),
  };
  if (!["x", "y", "z"].every((axis) => Number.isFinite(normalized[axis]))) {
    return DEFAULT_FIXED_SCAN_POSE_MM;
  }
  return normalized;
}

function buildFixedScanGoalMessage(pose) {
  const fixedPose = normalizeFixedScanPoseMm(pose);
  return {
    enable_capture_gate: false,
    scan_strategy: 2,
    use_fixed_scan_pose_override: true,
    fixed_scan_pose_x_mm: fixedPose.x,
    fixed_scan_pose_y_mm: fixedPose.y,
    fixed_scan_pose_z_mm: fixedPose.z,
  };
}

export class TaskActionController {
  constructor({ rosConnection, workspaceView, getRecognitionPose = null, callbacks = {} }) {
    this.rosConnection = rosConnection;
    this.workspaceView = workspaceView;
    this.getRecognitionPose = getRecognitionPose;
    this.callbacks = callbacks;
    this.pendingWorkspaceQuadSubmission = null;
  }

  handle(taskAction) {
    switch (taskAction) {
      case "submitQuad":
        return this.publishWorkspaceQuad();
      case "runSavedS2":
        return this.triggerSavedWorkspaceS2();
      case "triggerSingleBind":
        return this.triggerSinglePointBind();
      case "scanPlan":
        return this.triggerPseudoSlamScan();
      case "startExecution":
        return this.triggerExecutionLayer(true);
      case "startExecutionKeepMemory":
        return this.triggerExecutionLayer(false);
      case "runBindPathTest":
        return this.triggerBindPathDirectTest();
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
    this.setPendingWorkspaceQuadSubmission(payload);
    resources.workspaceQuadPublisher.publish(new ROSLIB.Message({ data: payload }));
    this.callbacks.onResultMessage?.(`工作区域已发送，等待 pointAI 保存确认: [${payload.join(", ")}]`);
    this.callbacks.onLog?.(`已发送工作区域: [${payload.join(", ")}]`, "success");
  }

  triggerSavedWorkspaceS2() {
    return this.triggerSurfaceDpRecognition({
      resultMessage:
        `正在触发当前画面无运动视觉记录的${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别；` +
        "完成后会覆盖 pseudo_slam_points.json 和 pseudo_slam_bind_path.json。",
      logMessage: `已触发${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别并准备覆盖本地绑扎点文件`,
    });
  }

  async triggerSurfaceDpRecognition({
    resultMessage =
      `正在触发当前画面无运动视觉记录的${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别；` +
      "完成后会覆盖 pseudo_slam_points.json 和 pseudo_slam_bind_path.json。",
    logMessage = `已触发${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别并准备覆盖本地绑扎点文件`,
  } = {}) {
    const resources = this.rosConnection.getResources();
    if (!resources?.startPseudoSlamScanActionClient) {
      this.report("ROS 还没连好，暂时不能触发视觉识别并覆盖本地绑扎点文件。", "warn");
      return false;
    }
    this.workspaceView.setExecutionOverlayMessage(null);
    this.callbacks.onWorkspaceS2Triggered?.();
    this.callbacks.onResultMessage?.(resultMessage);
    this.callbacks.onLog?.(logMessage, "success");

    const result = await this.sendActionGoal(resources.startPseudoSlamScanActionClient, {
      goalMessage: { enable_capture_gate: false, scan_strategy: 3 },
      feedbackPrefix: "视觉识别建图进行中",
      successPrefix: "视觉识别建图完成",
      failurePrefix: `${FRONTEND_VISUAL_RECOGNITION_FULL_LABEL}失败`,
    });
    return Boolean(result?.success);
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

  handleSavedWorkspacePayload(payload) {
    return this.confirmPendingWorkspaceQuadSubmission(payload);
  }

  setPendingWorkspaceQuadSubmission(payload) {
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
    if (!this.pendingWorkspaceQuadSubmission) {
      return false;
    }

    const payloadKey = buildWorkspaceQuadPayloadKey(payload);
    if (!payloadKey || payloadKey !== this.pendingWorkspaceQuadSubmission.payloadKey) {
      return false;
    }

    const resources = this.rosConnection.getResources();
    if (!resources?.startPseudoSlamScanActionClient) {
      this.clearPendingWorkspaceQuadSubmission();
      this.report("工作区已保存，但 ROS 未就绪，当前不能触发视觉识别并覆盖本地绑扎点文件。", "warn");
      return false;
    }

    this.clearPendingWorkspaceQuadSubmission();
    void this.triggerSurfaceDpRecognition({
      resultMessage:
        `工作区已保存，正在自动触发当前画面无运动视觉记录的${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别；` +
        "完成后会覆盖 pseudo_slam_points.json 和 pseudo_slam_bind_path.json。",
      logMessage:
        `已收到工作区保存确认，自动触发${FRONTEND_VISUAL_RECOGNITION_MODE_LABEL}视觉识别并覆盖本地绑扎点文件`,
    }).catch((error) => {
      this.report(`自动触发视觉识别失败: ${error?.message || String(error)}`, "error");
    });
    return true;
  }

  triggerPseudoSlamScan() {
    const resources = this.rosConnection.getResources();
    if (!resources?.startPseudoSlamScanActionClient) {
      this.report("ROS 还没连好，暂时不能开始固定扫描规划", "warn");
      return;
    }
    this.workspaceView.setExecutionOverlayMessage(null);
    const fixedScanPose = normalizeFixedScanPoseMm(this.getRecognitionPose?.());
    this.callbacks.onResultMessage?.(
      `正在执行固定工作区扫描：移动到 x=${Math.round(fixedScanPose.x)}, y=${Math.round(fixedScanPose.y)}, z=${Math.round(fixedScanPose.z)}，` +
        "索驱速度使用“索驱遥控”页里的全局索驱速度，然后触发视觉识别并动态规划，点位覆盖层会显示在当前图像图层。",
    );
    this.callbacks.onLog?.("已触发固定扫描建图任务", "success");
    return this.sendActionGoal(resources.startPseudoSlamScanActionClient, {
      goalMessage: buildFixedScanGoalMessage(fixedScanPose),
      feedbackPrefix: "扫描建图进行中",
      successPrefix: "扫描建图完成",
      failurePrefix: "扫描建图失败",
    });
  }

  triggerExecutionLayer(clearExecutionMemory) {
    const resources = this.rosConnection.getResources();
    if (!resources?.executionModeService || !resources?.startGlobalWorkActionClient) {
      this.report("ROS 还没连好，暂时不能开始执行层", "warn");
      return;
    }
    this.workspaceView.setExecutionOverlayMessage(null);
    this.callbacks.onResultMessage?.(
      "执行结果覆盖层会叠加到可兼容图像底图。",
    );
    this.callbacks.onLog?.(
      clearExecutionMemory ? "准备清记忆并开始执行层" : "准备保留记忆直接开始执行层",
      "success",
    );
    const request = new ROSLIB.ServiceRequest({ execution_mode: 1 });
    resources.executionModeService.callService(
      request,
      (response) => {
        if (!response?.success) {
          this.report(`执行模式切换失败: ${response?.message || "未知错误"}`, "error");
          return;
        }
        this.sendActionGoal(resources.startGlobalWorkActionClient, {
          goalMessage: { clear_execution_memory: clearExecutionMemory, execution_mode: 1 },
          feedbackPrefix: "执行层进行中",
          successPrefix: "执行层任务完成",
          failurePrefix: "执行层任务失败",
        });
      },
      (error) => this.report(`执行模式切换失败: ${error?.message || String(error)}`, "error"),
    );
  }

  triggerBindPathDirectTest() {
    const resources = this.rosConnection.getResources();
    if (!resources?.runDirectBindPathTestActionClient) {
      this.report("ROS 还没连好，暂时不能直接执行账本测试", "warn");
      return;
    }
    this.callbacks.onResultMessage?.(
      "直接执行账本测试已触发：后端将只按 pseudo_slam_bind_path.json 的 path_origin、cabin_pose 和 x/y/z 执行。",
    );
    this.callbacks.onLog?.("已触发直接执行账本测试", "success");
    return this.sendActionGoal(resources.runDirectBindPathTestActionClient, {
      goalMessage: {},
      feedbackPrefix: "账本测试进行中",
      successPrefix: "账本测试完成",
      failurePrefix: "账本测试失败",
    });
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
