function toNumber(value, fallback = 0) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : fallback;
}

export function normalizeCabinTelemetry(message) {
  if (!message) {
    return {
      hasTelemetry: false,
      motionStatus: 0,
      deviceAlarm: 0,
      internalCalcError: 0,
      cabinConnectFlag: 0,
    };
  }

  return {
    hasTelemetry: true,
    motionStatus: toNumber(message.motion_status),
    deviceAlarm: toNumber(message.device_alarm),
    internalCalcError: toNumber(message.internal_calc_error),
    cabinConnectFlag: toNumber(message.cabin_connect_flag),
  };
}

export function resolveCabinRemoteOperationState({
  rosReady = false,
  hasMoveServices = false,
  hasPosition = false,
  moveInFlight = false,
  cabinTelemetry = null,
} = {}) {
  const telemetry = cabinTelemetry || normalizeCabinTelemetry(null);
  const connected = rosReady
    && hasMoveServices
    && telemetry.hasTelemetry
    && telemetry.cabinConnectFlag === 1;

  if (!connected) {
    return {
      state: "blocked",
      canOperate: false,
      detail: "索驱不可操作：连接断开或状态未上报",
    };
  }

  if (telemetry.deviceAlarm !== 0 || telemetry.internalCalcError !== 0) {
    return {
      state: "alarm",
      canOperate: false,
      detail: "索驱报警：请先处理报警后再遥控",
    };
  }

  if (!hasPosition) {
    return {
      state: "blocked",
      canOperate: false,
      detail: "索驱不可操作：等待实时位置",
    };
  }

  if (moveInFlight || telemetry.motionStatus !== 0) {
    return {
      state: "blocked",
      canOperate: false,
      detail: "索驱不可操作：运动中",
    };
  }

  return {
    state: "ready",
    canOperate: true,
    detail: "索驱可操作",
  };
}
