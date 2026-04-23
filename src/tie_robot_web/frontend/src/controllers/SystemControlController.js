import { ROSLIB } from "../vendor/roslib.js";
import {
  getSystemControlAction,
  SYSTEM_CONTROL_ACTIONS,
} from "../config/systemControlCatalog.js";

export class SystemControlController {
  constructor({ rosConnection, callbacks = {} }) {
    this.rosConnection = rosConnection;
    this.callbacks = callbacks;
  }

  listActions() {
    return SYSTEM_CONTROL_ACTIONS;
  }

  handle(actionId) {
    const action = getSystemControlAction(actionId);
    if (!action) {
      this.report(`未识别的系统动作: ${actionId}`, "warn");
      return;
    }

    const resources = this.rosConnection.getResources();
    const service = resources?.[action.serviceKey];
    if (!this.rosConnection.isReady() || !service) {
      if (!action.httpEndpoint) {
        this.report(`ROS 未连接，暂时不能执行：${action.label}`, "warn");
        return;
      }
      this.handleViaHttp(action);
      return;
    }

    this.callbacks.onPendingChange?.(actionId, true);
    this.report(`已发送系统动作：${action.label}`, "info");
    service.callService(
      new ROSLIB.ServiceRequest({}),
      (response) => {
        this.callbacks.onPendingChange?.(actionId, false);
        if (!response?.success) {
          this.report(`${action.label}失败：${response?.message || "未知错误"}`, "error");
          return;
        }
        this.report(response?.message || `${action.label}已受理`, "success");
      },
      (error) => {
        this.callbacks.onPendingChange?.(actionId, false);
        this.report(`${action.label}失败：${error?.message || String(error)}`, "error");
      },
    );
  }

  report(message, level = "info") {
    this.callbacks.onResultMessage?.(message);
    this.callbacks.onLog?.(message, level);
  }

  async handleViaHttp(action) {
    this.callbacks.onPendingChange?.(action.id, true);
    this.report(`已发送系统动作：${action.label}`, "info");
    try {
      const response = await fetch(action.httpEndpoint, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ action: action.id }),
      });
      const payload = await response.json().catch(() => ({}));
      this.callbacks.onPendingChange?.(action.id, false);
      if (!response.ok || payload?.success === false) {
        this.report(`${action.label}失败：${payload?.message || response.statusText || "未知错误"}`, "error");
        return;
      }
      this.report(payload?.message || `${action.label}已受理`, "success");
    } catch (error) {
      this.callbacks.onPendingChange?.(action.id, false);
      this.report(`${action.label}失败：${error?.message || String(error)}`, "error");
    }
  }
}
