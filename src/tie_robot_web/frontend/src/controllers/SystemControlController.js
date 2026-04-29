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

    if (Array.isArray(action.steps) && action.steps.length > 0) {
      this.executeActionSequence(action);
      return;
    }

    const resources = this.rosConnection.getResources();
    const service = action.serviceKey ? resources?.[action.serviceKey] : null;
    if (!this.rosConnection.isReady() || !service) {
      if (!action.httpEndpoint) {
        this.report(`ROS 未连接，暂时不能执行：${action.label}`, "warn");
        return;
      }
      this.handleViaHttp(action);
      return;
    }

    this.executeSingleActionViaService(action, service);
  }

  executeSingleActionViaService(action, service) {
    this.callbacks.onPendingChange?.(action.id, true);
    this.report(`已发送系统动作：${action.label}`, "info");
    this.callRosService(action, service)
      .then((message) => {
        this.callbacks.onPendingChange?.(action.id, false);
        this.report(message || `${action.label}已受理`, "success");
      })
      .catch((error) => {
        this.callbacks.onPendingChange?.(action.id, false);
        this.report(`${action.label}失败：${error?.message || String(error)}`, "error");
      });
  }

  async executeActionSequence(action) {
    this.callbacks.onPendingChange?.(action.id, true);
    this.report(`已发送系统动作：${action.label}`, "info");
    try {
      for (const stepId of action.steps) {
        const stepAction = getSystemControlAction(stepId);
        if (!stepAction) {
          throw new Error(`未识别的系统动作: ${stepId}`);
        }
        await this.executeAction(stepAction);
      }
      this.callbacks.onPendingChange?.(action.id, false);
      this.report(`${action.label}已完成`, "success");
    } catch (error) {
      this.callbacks.onPendingChange?.(action.id, false);
      this.report(`${action.label}失败：${error?.message || String(error)}`, "error");
    }
  }

  executeAction(action) {
    const resources = this.rosConnection.getResources();
    const service = action.serviceKey ? resources?.[action.serviceKey] : null;
    if (this.rosConnection.isReady() && service) {
      return this.callRosService(action, service);
    }
    if (!action.httpEndpoint) {
      return Promise.reject(new Error(`ROS 未连接，暂时不能执行：${action.label}`));
    }
    return this.executeHttpAction(action);
  }

  callRosService(action, service) {
    return new Promise((resolve, reject) => {
      service.callService(
        new ROSLIB.ServiceRequest({}),
        (response) => {
          if (!response?.success) {
            reject(new Error(response?.message || "未知错误"));
            return;
          }
          resolve(response?.message || `${action.label}已受理`);
        },
        (error) => {
          reject(error);
        },
      );
    });
  }

  report(message, level = "info") {
    this.callbacks.onResultMessage?.(message);
    this.callbacks.onLog?.(message, level);
  }

  async handleViaHttp(action) {
    this.callbacks.onPendingChange?.(action.id, true);
    this.report(`已发送系统动作：${action.label}`, "info");
    try {
      const message = await this.executeHttpAction(action);
      this.callbacks.onPendingChange?.(action.id, false);
      this.report(message || `${action.label}已受理`, "success");
    } catch (error) {
      this.callbacks.onPendingChange?.(action.id, false);
      this.report(`${action.label}失败：${error?.message || String(error)}`, "error");
    }
  }

  async executeHttpAction(action) {
    const response = await fetch(action.httpEndpoint, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ action: action.id }),
    });
    const payload = await response.json().catch(() => ({}));
    if (!response.ok || payload?.success === false) {
      throw new Error(payload?.message || response.statusText || "未知错误");
    }
    return payload?.message || `${action.label}已受理`;
  }
}
