import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import { UIController } from "../src/ui/UIController.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

global.window = {
  setTimeout(callback) {
    return setTimeout(callback, 0);
  },
  clearTimeout(timerId) {
    clearTimeout(timerId);
  },
};

global.document = {
  activeElement: null,
};

function makeFakeElement(initialValue = "") {
  return {
    value: initialValue,
    textContent: "",
    innerHTML: "",
    dataset: {},
    setAttribute(name, value) {
      this[name] = value;
    },
  };
}

function makeBottomLinearAxis(scope, axis) {
  const element = makeFakeElement();
  element.dataset.bottomLinearAxis = `${scope}:${axis}`;
  return element;
}

const uiController = Object.create(UIController.prototype);
uiController.refs = {
  selectedPoints: makeFakeElement(),
  robotHomeSummary: makeFakeElement(),
  robotHomeTfSummary: makeFakeElement(),
  baseToCameraComputed: makeFakeElement(),
  robotHomeX: makeFakeElement(),
  robotHomeY: makeFakeElement(),
  robotHomeZ: makeFakeElement(),
  tcpLinearRemoteCurrentPosition: makeFakeElement(),
  tcpLinearRemoteStatus: makeFakeElement(),
  bottomLinearModulePosition: makeFakeElement(),
  bottomLinearModuleAxes: [
    makeBottomLinearAxis("local", "x"),
    makeBottomLinearAxis("local", "y"),
    makeBottomLinearAxis("local", "z"),
    makeBottomLinearAxis("local", "angle"),
    makeBottomLinearAxis("global", "x"),
    makeBottomLinearAxis("global", "y"),
    makeBottomLinearAxis("global", "z"),
  ],
};

uiController.renderPointList([
  { x: 12.6, y: 77.2 },
]);
assert.match(uiController.refs.selectedPoints.innerHTML, /x=13, y=77/);
assert.doesNotMatch(uiController.refs.selectedPoints.innerHTML, /\d+\.\d/);

uiController.setRobotHomeCalibration({
  hasCurrentPose: true,
  current: { x: 1000.6, y: 1999.4, z: 523.5 },
  home: { x: 1000.6, y: 1999.4, z: 523.5 },
  hasCameraPose: true,
  camera: { x: 1010.4, y: 2020.5, z: 553.49 },
  hasGroundProbe: true,
  groundProbe: { distance: 894.4, x: 1010.4, y: 2020.5, z: -341.5 },
  baseToCamera: { x: 10.4, y: 20.5, z: 30.49, roll: Math.PI, pitch: 0, yaw: 0 },
}, { forceInputs: true });

assert.equal(uiController.refs.robotHomeX.value, "1001");
assert.equal(uiController.refs.robotHomeY.value, "1999");
assert.equal(uiController.refs.robotHomeZ.value, "524");
assert.match(uiController.refs.robotHomeSummary.textContent, /X=1001mm Y=1999mm Z=524mm/);
assert.match(uiController.refs.robotHomeTfSummary.textContent, /相机map=X=1010mm Y=2021mm Z=553mm/);
assert.match(uiController.refs.baseToCameraComputed.textContent, /X=10mm Y=21mm Z=30mm/);
assert.doesNotMatch(uiController.refs.baseToCameraComputed.textContent, /[XYZ]=[-\d]+\.\dmm/);

uiController.setTcpLinearRemoteState({
  linear_module_position_X: 10.4,
  linear_module_position_Y: 20.5,
  linear_module_position_Z: 30.49,
  motor_angle: 9.5,
  linear_module_error_flag_X: 0,
  linear_module_error_flag_Y: 0,
  linear_module_error_flag_Z: 0,
  motor_error_flag: 0,
});

assert.match(uiController.refs.tcpLinearRemoteCurrentPosition.innerHTML, />10 mm</);
assert.match(uiController.refs.tcpLinearRemoteCurrentPosition.innerHTML, />21 mm</);
assert.match(uiController.refs.tcpLinearRemoteCurrentPosition.innerHTML, />30 mm</);
assert.match(uiController.refs.tcpLinearRemoteCurrentPosition.innerHTML, />9.5 deg</);
assert.doesNotMatch(uiController.refs.tcpLinearRemoteCurrentPosition.innerHTML, /\d+\.\d mm/);

uiController.setBottomLinearModulePosition(
  { x: 10.4, y: 20.5, z: 30.49, angle: 9.5 },
  { x: 100.4, y: 200.5, z: 300.49 },
);

assert.equal(uiController.refs.bottomLinearModuleAxes[0].textContent, "X 10 mm");
assert.equal(uiController.refs.bottomLinearModuleAxes[1].textContent, "Y 21 mm");
assert.equal(uiController.refs.bottomLinearModuleAxes[2].textContent, "Z 30 mm");
assert.equal(uiController.refs.bottomLinearModuleAxes[3].textContent, "angle 9.5 deg");
assert.match(uiController.refs.bottomLinearModulePosition.title, /线模本地：X=10mm Y=21mm Z=30mm angle=9\.5deg/);

const appText = readFileSync(resolve(frontendRoot, "src/app/TieRobotFrontApp.js"), "utf-8");
assert.doesNotMatch(appText, /target\.x\.toFixed\(1\)/);
assert.doesNotMatch(appText, /target\.y\.toFixed\(1\)/);
assert.doesNotMatch(appText, /target\.z\.toFixed\(1\)/);
assert.doesNotMatch(appText, /applied\.x\)\.toFixed\(1\)/);
