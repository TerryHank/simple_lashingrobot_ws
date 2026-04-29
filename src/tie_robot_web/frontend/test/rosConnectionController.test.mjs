import assert from "node:assert/strict";

import { ROSLIB } from "../src/vendor/roslib.js";
import { RosConnectionController } from "../src/controllers/RosConnectionController.js";

const timers = [];
global.window = {
  location: {
    hostname: "127.0.0.1",
    pathname: "/index.html",
    protocol: "http:",
  },
  setTimeout(callback, delay) {
    timers.push({ callback, delay });
    return timers.length;
  },
  clearTimeout() {},
  setInterval() {
    return 1;
  },
  clearInterval() {},
};

class FakeRos {
  static instances = [];

  constructor({ url }) {
    this.url = url;
    this.isConnected = false;
    this.handlers = new Map();
    FakeRos.instances.push(this);
  }

  on(eventName, callback) {
    this.handlers.set(eventName, callback);
  }

  emit(eventName, payload) {
    this.handlers.get(eventName)?.(payload);
  }

  close() {
    this.emit("close");
  }

  getTopicsAndRawTypes(success) {
    success({ topics: [], types: [] });
  }
}

class FakeTopic {
  constructor(options) {
    Object.assign(this, options);
  }

  advertise() {}

  subscribe() {}

  unsubscribe() {}

  publish() {}
}

class FakeService {
  constructor(options) {
    Object.assign(this, options);
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

const connectionEvents = [];
const logEvents = [];
let unavailableCount = 0;
const controller = new RosConnectionController({
  onConnectionInfo: (url, message, level) => connectionEvents.push({ url, message, level }),
  onLog: (message, level) => logEvents.push({ message, level }),
  onRosUnavailable: () => {
    unavailableCount += 1;
  },
});

controller.connect();
FakeRos.instances.at(-1).emit("close");
assert.equal(timers.length, 1);
assert.equal(timers.at(-1).delay, 1500);
assert.equal(connectionEvents.at(-1).level, "reconnecting");
assert.match(connectionEvents.at(-1).message, /自动重连\(1\/3\)/);

timers.shift().callback();
FakeRos.instances.at(-1).emit("close");
assert.equal(timers.length, 1);
assert.match(connectionEvents.at(-1).message, /自动重连\(2\/3\)/);

timers.shift().callback();
FakeRos.instances.at(-1).emit("close");
assert.equal(timers.length, 1);
assert.match(connectionEvents.at(-1).message, /自动重连\(3\/3\)/);

timers.shift().callback();
FakeRos.instances.at(-1).emit("close");
assert.equal(timers.length, 0);
assert.equal(connectionEvents.at(-1).level, "manual");
assert.match(connectionEvents.at(-1).message, /请点击手动重连/);
assert.equal(unavailableCount, 4);

controller.connect({ manual: true });
assert.equal(connectionEvents.at(-1).level, "info");
assert.match(logEvents.at(-1).message, /手动重新连接 ROSBridge/);
