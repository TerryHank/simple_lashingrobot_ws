import assert from "node:assert/strict";

import {
  normalizeCabinTelemetry,
  resolveCabinRemoteOperationState,
} from "../src/utils/cabinRemoteOperationState.js";

assert.deepEqual(
  normalizeCabinTelemetry({
    motion_status: 0,
    device_alarm: 0,
    internal_calc_error: 0,
    cabin_connect_flag: 1,
  }),
  {
    hasTelemetry: true,
    motionStatus: 0,
    deviceAlarm: 0,
    internalCalcError: 0,
    cabinConnectFlag: 1,
  },
);

assert.deepEqual(
  resolveCabinRemoteOperationState({
    rosReady: true,
    hasMoveServices: true,
    hasPosition: true,
    moveInFlight: false,
    cabinTelemetry: normalizeCabinTelemetry({
      motion_status: 0,
      device_alarm: 0,
      internal_calc_error: 0,
      cabin_connect_flag: 1,
    }),
  }),
  {
    state: "ready",
    canOperate: true,
    detail: "索驱可操作",
  },
);

assert.equal(
  resolveCabinRemoteOperationState({
    rosReady: true,
    hasMoveServices: true,
    hasPosition: true,
    moveInFlight: false,
    cabinTelemetry: normalizeCabinTelemetry({
      motion_status: 1,
      device_alarm: 0,
      internal_calc_error: 0,
      cabin_connect_flag: 1,
    }),
  }).state,
  "blocked",
);

assert.equal(
  resolveCabinRemoteOperationState({
    rosReady: true,
    hasMoveServices: true,
    hasPosition: true,
    moveInFlight: false,
    cabinTelemetry: normalizeCabinTelemetry({
      motion_status: 0,
      device_alarm: 1,
      internal_calc_error: 0,
      cabin_connect_flag: 1,
    }),
  }).state,
  "alarm",
);

assert.equal(
  resolveCabinRemoteOperationState({
    rosReady: true,
    hasMoveServices: true,
    hasPosition: true,
    moveInFlight: false,
    cabinTelemetry: normalizeCabinTelemetry({
      motion_status: 0,
      device_alarm: 1,
      internal_calc_error: 0,
      cabin_connect_flag: 0,
    }),
  }).state,
  "blocked",
);
