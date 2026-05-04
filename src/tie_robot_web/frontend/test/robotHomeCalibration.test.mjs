import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import { ROSLIB } from "../src/vendor/roslib.js";
import { RosConnectionController } from "../src/controllers/RosConnectionController.js";
import { SERVICE_TYPES, SERVICES } from "../src/config/topicRegistry.js";

const __dirname = dirname(fileURLToPath(import.meta.url));
const frontendRoot = resolve(__dirname, "..");

global.window = {
  location: {
    hostname: "127.0.0.1",
    pathname: "/index.html",
    protocol: "http:",
  },
  setTimeout() {
    return 1;
  },
  clearTimeout() {},
  setInterval() {
    return 1;
  },
  clearInterval() {},
};

class FakeRos {
  constructor() {
    this.isConnected = true;
  }

  on() {}
}

class FakeTopic {
  constructor(options) {
    Object.assign(this, options);
  }

  advertise() {}

  publish() {}

  subscribe() {}

  unsubscribe() {}
}

class FakeService {
  constructor(options) {
    Object.assign(this, options);
    this.calls = [];
  }

  callService(request, success) {
    this.calls.push(request);
    success({
      success: true,
      message: "home calibration updated",
      has_current_pose: true,
      current_x_mm: 1000,
      current_y_mm: 2000,
      current_z_mm: 523,
      home_x_mm: 1000,
      home_y_mm: 2000,
      home_z_mm: 523,
      base_to_camera_x_mm: 10,
      base_to_camera_y_mm: 20,
      base_to_camera_z_mm: 30,
      base_to_camera_roll_rad: 3.141592653589793,
      base_to_camera_pitch_rad: 0,
      base_to_camera_yaw_rad: 0,
      has_camera_pose: true,
      camera_x_mm: 1010,
      camera_y_mm: 2020,
      camera_z_mm: 553,
      has_ground_probe: true,
      camera_ground_distance_mm: 894,
      ground_x_mm: 1010,
      ground_y_mm: 2020,
      ground_z_mm: -341,
    });
  }
}

class FakeActionClient {
  constructor(options) {
    Object.assign(this, options);
  }
}

ROSLIB.Ros = FakeRos;
ROSLIB.Topic = FakeTopic;
ROSLIB.Service = FakeService;
ROSLIB.ActionClient = FakeActionClient;
ROSLIB.Message = class {
  constructor(payload) {
    Object.assign(this, payload);
  }
};
ROSLIB.ServiceRequest = class {
  constructor(payload) {
    Object.assign(this, payload);
  }
};

assert.equal(SERVICES.tf.robotHomeCalibration, "/web/tf/robot_home_calibration");
assert.equal(SERVICE_TYPES.tf.robotHomeCalibration, "tie_robot_msgs/RobotHomeCalibration");

const controller = new RosConnectionController();
controller.ros = new FakeRos();
controller.resources = controller.buildResources(controller.ros);

const result = await controller.callRobotHomeCalibrationService({
  command: "set_home",
  home: { x: 1000, y: 2000, z: 523 },
});

assert.equal(result.success, true);
assert.equal(result.current.x, 1000);
assert.equal(result.home.z, 523);
assert.equal(result.baseToCamera.roll, Math.PI);
assert.equal(result.groundProbe.distance, 894);
assert.equal(controller.resources.robotHomeCalibrationService.calls.at(-1).command, "set_home");
assert.equal(controller.resources.robotHomeCalibrationService.calls.at(-1).home_z_mm, 523);
assert.equal(
  Object.hasOwn(controller.resources.robotHomeCalibrationService.calls.at(-1), "base_to_camera_x_mm"),
  false,
);

const uiControllerText = readFileSync(resolve(frontendRoot, "src/ui/UIController.js"), "utf-8");
assert.match(uiControllerText, /id: "homeCalibration", label: "Home点位"/);
assert.match(uiControllerText, /id="captureRobotHome"/);
assert.match(uiControllerText, /id="saveRobotHomeCalibration"/);
assert.match(uiControllerText, /id="moveRobotHome"/);
assert.match(uiControllerText, /id="baseToCameraComputed"/);
assert.doesNotMatch(uiControllerText, /id="baseToCameraX"/);
assert.doesNotMatch(uiControllerText, /id="baseToCameraRoll"/);
