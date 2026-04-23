import { buildWorkspaceQuadPayload } from "../ir_workspace_picker_helpers.mjs";
import { resultStatusEl } from "./dom_refs.mjs";
import { drawS2Overlay } from "./canvas_renderer.mjs";
import { callExecutionModeService, startRosActionGoal } from "./action_clients.mjs";
import { ROSLIB, state, setStatus } from "./ui_state.mjs";

export function publishWorkspaceQuad() {
  if (!state.quadPublisher || !state.runS2Publisher || state.selectedPoints.length !== 4) {
    return;
  }
  state.overlaySource = "s2";
  const payload = buildWorkspaceQuadPayload(state.selectedPoints);
  const message = new ROSLIB.Message({ data: payload });
  state.quadPublisher.publish(message);
  state.lastResultImageMessage = null;
  drawS2Overlay();
  setStatus(`工作区四边形已发送，正在触发 S2: [${payload.join(", ")}]`, "success");
  if (resultStatusEl) {
    resultStatusEl.textContent = "正在等待 pointAI 返回 S2 result...";
  }
  window.setTimeout(() => {
    state.runS2Publisher.publish(new ROSLIB.Message({ data: true }));
  }, 180);
}

export function triggerSavedWorkspaceS2() {
  if (!state.runS2Publisher || state.savedWorkspacePoints.length !== 4) {
    setStatus("还没有已保存的工作区四边形，先提交一次工作区", "warn");
    if (resultStatusEl) {
      resultStatusEl.textContent = "当前没有可复用的已保存工作区，请先点 4 个角点并提交。";
    }
    return;
  }
  state.overlaySource = "s2";
  state.lastResultImageMessage = null;
  drawS2Overlay();
  setStatus("已使用当前已保存工作区触发 S2，正在等待结果...", "success");
  if (resultStatusEl) {
    resultStatusEl.textContent = "正在使用当前已保存工作区识别绑扎点...";
  }
  state.runS2Publisher.publish(new ROSLIB.Message({ data: true }));
}

export function triggerWorkspaceCenterScanPoseMove() {
  if (!state.pseudoSlamScanActionClient) {
    setStatus("ROS 还没连好，暂时不能开始固定扫描规划", "warn");
    return;
  }
  state.overlaySource = "s2";
  state.lastResultImageMessage = null;
  drawS2Overlay();
  setStatus("已触发固定扫描建图，后端将自动移动、识别并动态规划", "success");
  if (resultStatusEl) {
    resultStatusEl.textContent =
      "正在执行固定工作区扫描：移动到 x=-260, y=1700, z=3197，索驱速度使用“索驱遥控”页里的全局索驱速度，然后触发 S2 并动态规划。";
  }
  const goal = new ROSLIB.Goal({
    actionClient: state.pseudoSlamScanActionClient,
    goalMessage: { enable_capture_gate: false, scan_strategy: 2 },
  });
  startRosActionGoal(goal, {
    onFeedback: (feedback) => {
      setStatus(`扫描建图进行中: ${feedback?.detail || feedback?.stage || "处理中"}`, "info");
    },
    onResult: (result) => {
      if (result?.success) {
        setStatus(`扫描建图完成: ${result.message || "已完成"}`, "success");
        if (resultStatusEl) {
          resultStatusEl.textContent = `扫描建图完成：${result.message || "已完成动态规划。"}`;
        }
        return;
      }
      setStatus(`扫描建图失败: ${result?.message || "未知错误"}`, "error");
      if (resultStatusEl) {
        resultStatusEl.textContent = `扫描建图失败：${result?.message || "未知错误"}`;
      }
    },
  });
}

export function triggerExecutionLayer(clearExecutionMemory = true) {
  if (!state.executionModeService || !state.startGlobalWorkActionClient) {
    setStatus("ROS 还没连好，暂时不能开始执行层", "warn");
    return;
  }
  state.overlaySource = "execution";
  state.lastExecutionResultImageMessage = null;
  drawS2Overlay();
  setStatus(
    clearExecutionMemory
      ? "已切到视觉精校执行，正在清记忆并开始执行层..."
      : "已切到视觉精校执行，正在保留记忆直接开始执行层...",
    "success",
  );
  if (resultStatusEl) {
    resultStatusEl.textContent =
      "执行层已切到 result_img 覆盖层，后续显示 /pointAI/result_image_raw。";
  }
  callExecutionModeService(
    1,
    () => {
      const goal = new ROSLIB.Goal({
        actionClient: state.startGlobalWorkActionClient,
        goalMessage: {
          clear_execution_memory: clearExecutionMemory,
          execution_mode: 1,
        },
      });
      startRosActionGoal(goal, {
        onFeedback: (feedback) => {
          setStatus(`执行层进行中: ${feedback?.detail || feedback?.stage || "处理中"}`, "info");
        },
        onResult: (result) => {
          if (result?.success) {
            setStatus(`执行层任务完成: ${result.message || "已完成"}`, "success");
            if (resultStatusEl) {
              resultStatusEl.textContent = `执行层任务完成：${result.message || "已完成"}`;
            }
            return;
          }
          setStatus(`执行层任务失败: ${result?.message || "未知错误"}`, "error");
          if (resultStatusEl) {
            resultStatusEl.textContent = `执行层任务失败：${result?.message || "未知错误"}`;
          }
        },
      });
    },
    (message) => {
      setStatus(`执行模式切换失败: ${message}`, "error");
      if (resultStatusEl) {
        resultStatusEl.textContent = `执行模式切换失败：${message}`;
      }
    },
  );
}

export function triggerDirectBindPathTest() {
  if (!state.runDirectBindPathTestActionClient) {
    setStatus("ROS 还没连好，暂时不能直接执行账本测试", "warn");
    return;
  }
  setStatus("已触发直接执行账本测试：只按 pseudo_slam_bind_path.json 执行，不走相机 live", "success");
  if (resultStatusEl) {
    resultStatusEl.textContent =
      "直接执行账本测试已触发：后端将只按 pseudo_slam_bind_path.json 的 path_origin、cabin_pose 和 x/y/z 执行。";
  }
  const goal = new ROSLIB.Goal({ actionClient: state.runDirectBindPathTestActionClient, goalMessage: {} });
  startRosActionGoal(goal, {
    onFeedback: (feedback) => {
      setStatus(`账本测试进行中: ${feedback?.detail || feedback?.stage || "处理中"}`, "info");
    },
    onResult: (result) => {
      if (result?.success) {
        setStatus(`账本测试完成: ${result.message || "已完成"}`, "success");
        if (resultStatusEl) {
          resultStatusEl.textContent = `账本测试完成：${result.message || "已完成"}`;
        }
        return;
      }
      setStatus(`账本测试失败: ${result?.message || "未知错误"}`, "error");
      if (resultStatusEl) {
        resultStatusEl.textContent = `账本测试失败：${result?.message || "未知错误"}`;
      }
    },
  });
}
