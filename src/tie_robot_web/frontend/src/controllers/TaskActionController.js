import { ROSLIB } from "../vendor/roslib.js";
import { buildWorkspaceQuadPayload } from "../utils/irImageUtils.js";

export class TaskActionController {
  constructor({ rosConnection, workspaceView, callbacks = {} }) {
    this.rosConnection = rosConnection;
    this.workspaceView = workspaceView;
    this.callbacks = callbacks;
  }

  handle(taskAction) {
    switch (taskAction) {
      case "submitQuad":
        return this.publishWorkspaceQuad();
      case "runSavedS2":
        return this.triggerSavedWorkspaceS2();
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
    if (!resources?.workspaceQuadPublisher || !resources?.runWorkspaceS2Publisher || selectedPoints.length !== 4) {
      this.report("当前还不能提交工作区四边形，请先连上 ROS 并点满 4 个角点", "warn");
      return;
    }
    const payload = buildWorkspaceQuadPayload(selectedPoints);
    this.workspaceView.setExecutionOverlayMessage(null);
    resources.workspaceQuadPublisher.publish(new ROSLIB.Message({ data: payload }));
    this.callbacks.onResultMessage?.(`工作区四边形已发送，正在触发 S2，覆盖层统一等待 result_img: [${payload.join(", ")}]`);
    this.callbacks.onLog?.(`已发送工作区四边形: [${payload.join(", ")}]`, "success");
    window.setTimeout(() => {
      resources.runWorkspaceS2Publisher.publish(new ROSLIB.Message({ data: true }));
    }, 180);
  }

  triggerSavedWorkspaceS2() {
    const resources = this.rosConnection.getResources();
    const savedPoints = this.workspaceView.getSavedWorkspacePoints();
    if (!resources?.runWorkspaceS2Publisher || savedPoints.length !== 4) {
      this.report("当前没有可复用的已保存工作区，请先点 4 个角点并提交", "warn");
      return;
    }
    this.workspaceView.setExecutionOverlayMessage(null);
    resources.runWorkspaceS2Publisher.publish(new ROSLIB.Message({ data: true }));
    this.callbacks.onResultMessage?.("正在使用当前已保存工作区识别绑扎点，覆盖层统一等待 result_img...");
    this.callbacks.onLog?.("已触发基于已保存工作区的 S2 识别", "success");
  }

  triggerPseudoSlamScan() {
    const resources = this.rosConnection.getResources();
    if (!resources?.startPseudoSlamScanActionClient) {
      this.report("ROS 还没连好，暂时不能开始固定扫描规划", "warn");
      return;
    }
    this.workspaceView.setExecutionOverlayMessage(null);
    this.callbacks.onResultMessage?.(
      "正在执行固定工作区扫描：移动到 x=-260, y=1700, z=2997, speed=100，然后触发 S2 并动态规划，覆盖层统一显示 result_img。",
    );
    this.callbacks.onLog?.("已触发固定扫描建图任务", "success");
    this.sendActionGoal(resources.startPseudoSlamScanActionClient, {
      goalMessage: { enable_capture_gate: false, scan_strategy: 2 },
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
      "工作区覆盖层统一使用 result_img，后续显示 /pointAI/result_image_raw。",
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
    this.sendActionGoal(resources.runDirectBindPathTestActionClient, {
      goalMessage: {},
      feedbackPrefix: "账本测试进行中",
      successPrefix: "账本测试完成",
      failurePrefix: "账本测试失败",
    });
  }

  sendActionGoal(actionClient, { goalMessage, feedbackPrefix, successPrefix, failurePrefix }) {
    const goal = new ROSLIB.Goal({ actionClient, goalMessage });
    goal.on("feedback", (feedback) => {
      const detail = feedback?.detail || feedback?.stage || "处理中";
      this.callbacks.onResultMessage?.(`${feedbackPrefix}: ${detail}`);
    });
    goal.on("result", (result) => {
      const message = result?.message || "未知结果";
      if (result?.success) {
        this.callbacks.onResultMessage?.(`${successPrefix}: ${message}`);
        this.callbacks.onLog?.(`${successPrefix}: ${message}`, "success");
        return;
      }
      this.callbacks.onResultMessage?.(`${failurePrefix}: ${message}`);
      this.callbacks.onLog?.(`${failurePrefix}: ${message}`, "error");
    });
    goal.send();
  }

  report(message, level = "info") {
    this.callbacks.onResultMessage?.(message);
    this.callbacks.onLog?.(message, level);
  }
}
