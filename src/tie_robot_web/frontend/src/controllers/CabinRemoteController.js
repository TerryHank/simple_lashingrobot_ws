const CABIN_REMOTE_DIRECTION_DEFINITIONS = {
  xNegative: { axis: "x", delta: -1, label: "X-" },
  xPositive: { axis: "x", delta: 1, label: "X+" },
  yPositive: { axis: "y", delta: 1, label: "Y+" },
  yNegative: { axis: "y", delta: -1, label: "Y-" },
  zPositive: { axis: "z", delta: 1, label: "Z+" },
  zNegative: { axis: "z", delta: -1, label: "Z-" },
};

function sanitizePositiveNumber(value, fallback) {
  const parsed = Number.parseFloat(value);
  if (!Number.isFinite(parsed) || parsed <= 0) {
    return fallback;
  }
  return parsed;
}

function sanitizeCabinPosition(position) {
  if (!position) {
    return null;
  }

  return {
    x: Number.isFinite(position.x) ? position.x : 0,
    y: Number.isFinite(position.y) ? position.y : 0,
    z: Number.isFinite(position.z) ? position.z : 0,
  };
}

export class CabinRemoteController {
  constructor({ rosConnection, sceneView }) {
    this.rosConnection = rosConnection;
    this.sceneView = sceneView;
    this.lastKnownCabinPositionMm = null;
  }

  setLastKnownCabinPosition(position) {
    const sanitized = sanitizeCabinPosition(position);
    if (!sanitized) {
      return;
    }
    this.lastKnownCabinPositionMm = sanitized;
  }

  getCurrentCabinPositionMm() {
    const currentPosition = sanitizeCabinPosition(this.sceneView.getCurrentCabinPositionMm());
    if (currentPosition) {
      this.lastKnownCabinPositionMm = currentPosition;
      return currentPosition;
    }
    return this.lastKnownCabinPositionMm;
  }

  async move(directionId, { step, speed }) {
    const definition = CABIN_REMOTE_DIRECTION_DEFINITIONS[directionId];
    if (!definition) {
      return {
        success: false,
        message: `未识别的索驱遥控方向：${directionId}`,
      };
    }

    const currentPosition = this.getCurrentCabinPositionMm();
    if (!currentPosition) {
      return {
        success: false,
        message: "暂未拿到索驱当前位置，无法执行步进遥控。",
        label: definition.label,
      };
    }

    const sanitizedStep = sanitizePositiveNumber(step, 50);
    const sanitizedSpeed = sanitizePositiveNumber(speed, 300);
    const target = {
      x: currentPosition.x,
      y: currentPosition.y,
      z: currentPosition.z,
      speed: sanitizedSpeed,
    };
    target[definition.axis] += definition.delta * sanitizedStep;

    const result = await this.rosConnection.callCabinSingleMoveService(target);
    if (result?.success) {
      this.lastKnownCabinPositionMm = {
        x: target.x,
        y: target.y,
        z: target.z,
      };
    }

    return {
      success: Boolean(result?.success),
      message: result?.message || "",
      label: definition.label,
      target,
      step: sanitizedStep,
      speed: sanitizedSpeed,
    };
  }
}
