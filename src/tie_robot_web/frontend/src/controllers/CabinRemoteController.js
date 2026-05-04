import { formatCabinRemoteProtocolFeedback } from "../utils/cabinRemoteProtocolFeedback.js";

const CABIN_REMOTE_DIRECTION_DEFINITIONS = {
  xNegative: { axis: "x", delta: -1, label: "X-" },
  xPositive: { axis: "x", delta: 1, label: "X+" },
  yPositive: { axis: "y", delta: 1, label: "Y+" },
  yNegative: { axis: "y", delta: -1, label: "Y-" },
  zPositive: { axis: "z", delta: 1, label: "Z+" },
  zNegative: { axis: "z", delta: -1, label: "Z-" },
};

const CABIN_REMOTE_MOVE_MODE_RELATIVE = "relative";

function sanitizePositiveNumber(value, fallback) {
  const parsed = Number.parseFloat(value);
  if (!Number.isFinite(parsed) || parsed <= 0) {
    return fallback;
  }
  return parsed;
}

function normalizeMoveMode(value) {
  return value === CABIN_REMOTE_MOVE_MODE_RELATIVE ? "relative" : "absolute";
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
    this.lastKnownRawCabinPositionMm = null;
  }

  setLastKnownCabinPosition(position) {
    const sanitized = sanitizeCabinPosition(position);
    if (!sanitized) {
      return;
    }
    this.lastKnownCabinPositionMm = sanitized;
  }

  setLastKnownRawCabinPosition(position) {
    const sanitized = sanitizeCabinPosition(position);
    if (!sanitized) {
      return;
    }
    this.lastKnownRawCabinPositionMm = sanitized;
  }

  getCurrentRawCabinPositionMm() {
    return this.lastKnownRawCabinPositionMm;
  }

  getCurrentCabinPositionMm() {
    const currentPosition = sanitizeCabinPosition(this.sceneView.getCurrentCabinPositionMm());
    if (currentPosition) {
      this.lastKnownCabinPositionMm = currentPosition;
      return currentPosition;
    }
    return this.lastKnownCabinPositionMm;
  }

  async move(directionId, { step, speed, moveMode } = {}) {
    const definition = CABIN_REMOTE_DIRECTION_DEFINITIONS[directionId];
    if (!definition) {
      return {
        success: false,
        message: `未识别的索驱遥控方向：${directionId}`,
      };
    }

    const sanitizedStep = sanitizePositiveNumber(step, 50);
    const sanitizedSpeed = sanitizePositiveNumber(speed, 300);
    const mode = normalizeMoveMode(moveMode);
    const delta = {
      x: 0,
      y: 0,
      z: 0,
    };
    delta[definition.axis] = definition.delta * sanitizedStep;

    if (mode === "relative") {
      const target = {
        ...delta,
        speed: sanitizedSpeed,
      };
      const result = await this.rosConnection.callCabinIncrementalMoveService(target);
      if (result?.success) {
        const rawPosition = this.getCurrentRawCabinPositionMm();
        if (rawPosition) {
          this.lastKnownRawCabinPositionMm = {
            x: rawPosition.x + delta.x,
            y: rawPosition.y + delta.y,
            z: rawPosition.z + delta.z,
          };
        }
      }

      return {
        success: Boolean(result?.success),
        message: result?.success
          ? result?.message || ""
          : formatCabinRemoteProtocolFeedback(result?.message || ""),
        label: definition.label,
        mode,
        target,
        delta,
        step: sanitizedStep,
        speed: sanitizedSpeed,
      };
    }

    const rawPosition = this.getCurrentRawCabinPositionMm();
    if (!rawPosition) {
      return {
        success: false,
        message: "绝对点动需要先收到索驱当前原始坐标。",
        label: definition.label,
        mode,
        target: null,
        delta,
        step: sanitizedStep,
        speed: sanitizedSpeed,
      };
    }

    const target = {
      x: rawPosition.x + delta.x,
      y: rawPosition.y + delta.y,
      z: rawPosition.z + delta.z,
      speed: sanitizedSpeed,
    };
    const result = await this.rosConnection.callCabinSingleMoveService(target);
    if (result?.success) {
      this.lastKnownRawCabinPositionMm = {
        x: target.x,
        y: target.y,
        z: target.z,
      };
    }

    return {
      success: Boolean(result?.success),
      message: result?.success
        ? result?.message || ""
        : formatCabinRemoteProtocolFeedback(result?.message || ""),
      label: definition.label,
      mode,
      target,
      delta,
      step: sanitizedStep,
      speed: sanitizedSpeed,
    };
  }
}
