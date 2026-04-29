const TCP_LINEAR_REMOTE_DIRECTION_DEFINITIONS = {
  xNegative: { axis: "x", delta: -1, label: "X-" },
  xPositive: { axis: "x", delta: 1, label: "X+" },
  yNegative: { axis: "y", delta: -1, label: "Y-" },
  yPositive: { axis: "y", delta: 1, label: "Y+" },
  zNegative: { axis: "z", delta: -1, label: "Z-" },
  zPositive: { axis: "z", delta: 1, label: "Z+" },
  angleNegative: { axis: "angle", delta: -1, label: "角度-" },
  anglePositive: { axis: "angle", delta: 1, label: "角度+" },
};

const TCP_LINEAR_TRAVEL_LIMITS_MM = Object.freeze({
  x: { min: 0, max: 360 },
  y: { min: 0, max: 320 },
  z: { min: 0, max: 140 },
});

function sanitizeNumber(value, fallback = 0) {
  const parsed = Number.parseFloat(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function sanitizePositiveNumber(value, fallback) {
  const parsed = Number.parseFloat(value);
  if (!Number.isFinite(parsed) || parsed <= 0) {
    return fallback;
  }
  return parsed;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function normalizeLinearModuleState(message) {
  if (!message) {
    return null;
  }

  return {
    x: sanitizeNumber(message.linear_module_position_X),
    y: sanitizeNumber(message.linear_module_position_Y),
    z: sanitizeNumber(message.linear_module_position_Z),
    angle: sanitizeNumber(message.motor_angle),
  };
}

function clampTcpLinearTarget(target) {
  return {
    x: clamp(target.x, TCP_LINEAR_TRAVEL_LIMITS_MM.x.min, TCP_LINEAR_TRAVEL_LIMITS_MM.x.max),
    y: clamp(target.y, TCP_LINEAR_TRAVEL_LIMITS_MM.y.min, TCP_LINEAR_TRAVEL_LIMITS_MM.y.max),
    z: clamp(target.z, TCP_LINEAR_TRAVEL_LIMITS_MM.z.min, TCP_LINEAR_TRAVEL_LIMITS_MM.z.max),
    angle: sanitizeNumber(target.angle),
  };
}

export class TcpLinearRemoteController {
  constructor({ rosConnection }) {
    this.rosConnection = rosConnection;
    this.currentState = null;
  }

  setCurrentState(message) {
    const nextState = normalizeLinearModuleState(message);
    if (!nextState) {
      return;
    }
    this.currentState = nextState;
  }

  getCurrentState() {
    return this.currentState ? { ...this.currentState } : null;
  }

  async move(directionId, { step, angleStep }) {
    const definition = TCP_LINEAR_REMOTE_DIRECTION_DEFINITIONS[directionId];
    if (!definition) {
      return {
        success: false,
        message: `未识别的 TCP 线性模组遥控方向：${directionId}`,
      };
    }

    if (!this.currentState) {
      return {
        success: false,
        message: "暂未拿到线性模组当前位置，无法执行 TCP 步进遥控。",
        label: definition.label,
      };
    }

    const sanitizedStep = sanitizePositiveNumber(step, 5);
    const sanitizedAngleStep = sanitizePositiveNumber(angleStep, 5);
    const increment = definition.axis === "angle" ? sanitizedAngleStep : sanitizedStep;
    const rawTarget = {
      ...this.currentState,
    };
    rawTarget[definition.axis] += definition.delta * increment;
    const target = clampTcpLinearTarget(rawTarget);

    const result = await this.rosConnection.callLinearModuleSingleMoveService(target);
    if (result?.success) {
      this.currentState = target;
    }

    return {
      success: Boolean(result?.success),
      message: result?.message || "",
      label: definition.label,
      target,
      step: sanitizedStep,
      angleStep: sanitizedAngleStep,
      increment,
      clamped: target.x !== rawTarget.x || target.y !== rawTarget.y || target.z !== rawTarget.z,
    };
  }
}
