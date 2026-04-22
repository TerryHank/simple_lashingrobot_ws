import { ROSLIB, state } from "./ui_state.mjs";


export function startRosActionGoal(goal, { onFeedback, onResult }) {
  if (typeof onFeedback === "function") {
    goal.on("feedback", onFeedback);
  }
  if (typeof onResult === "function") {
    goal.on("result", onResult);
  }
  goal.send();
}

export function callExecutionModeService(executionMode, onSuccess, onFailure) {
  if (!state.executionModeService) {
    onFailure?.("执行模式服务未就绪");
    return;
  }
  const request = new ROSLIB.ServiceRequest({ execution_mode: executionMode });
  state.executionModeService.callService(
    request,
    (response) => {
      if (!response?.success) {
        onFailure?.(response?.message || "执行模式切换失败");
        return;
      }
      onSuccess?.(response);
    },
    (error) => {
      onFailure?.(error?.message || String(error));
    },
  );
}
