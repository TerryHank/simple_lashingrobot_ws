import assert from "node:assert/strict";

import { PanelManager } from "../src/ui/PanelManager.js";

function createClassList() {
  const classes = new Set();
  return {
    add(name) {
      classes.add(name);
    },
    remove(name) {
      classes.delete(name);
    },
    toggle(name, force) {
      if (force === undefined) {
        if (classes.has(name)) {
          classes.delete(name);
          return false;
        }
        classes.add(name);
        return true;
      }
      if (force) {
        classes.add(name);
      } else {
        classes.delete(name);
      }
      return Boolean(force);
    },
    contains(name) {
      return classes.has(name);
    },
  };
}

function px(value, fallback = 0) {
  const parsed = Number.parseFloat(String(value || ""));
  return Number.isFinite(parsed) ? parsed : fallback;
}

function createPanel(id, rect) {
  const currentRect = { ...rect };
  const style = {};
  const panel = {
    id,
    style,
    classList: createClassList(),
    dataset: {},
    addEventListener() {},
    querySelector(selector) {
      if (selector === ".panel-header") {
        return {
          offsetHeight: 52,
          addEventListener() {},
        };
      }
      if (selector === "[data-panel-maximize]") {
        return {
          textContent: "",
          title: "",
          addEventListener() {},
        };
      }
      return null;
    },
    getBoundingClientRect() {
      return {
        left: px(style.left, currentRect.left),
        top: px(style.top, currentRect.top),
        width: px(style.width, currentRect.width),
        height: px(style.height, currentRect.height),
      };
    },
    setRect(nextRect) {
      Object.assign(currentRect, nextRect);
      Object.entries(nextRect).forEach(([key, value]) => {
        style[key] = `${value}px`;
      });
    },
  };
  Object.defineProperties(panel, {
    offsetLeft: {
      get() {
        return panel.getBoundingClientRect().left;
      },
    },
    offsetTop: {
      get() {
        return panel.getBoundingClientRect().top;
      },
    },
    offsetWidth: {
      get() {
        return panel.getBoundingClientRect().width;
      },
    },
    offsetHeight: {
      get() {
        return panel.getBoundingClientRect().height;
      },
    },
  });
  return panel;
}

const animationFrameQueue = [];
const resizeObserverCallbacks = [];

global.window = {
  innerWidth: 900,
  innerHeight: 700,
  addEventListener() {},
  requestAnimationFrame(callback) {
    animationFrameQueue.push(callback);
    return animationFrameQueue.length;
  },
};

global.document = {
  body: { style: {} },
  querySelector() {
    return null;
  },
  querySelectorAll() {
    return [];
  },
};

global.requestAnimationFrame = global.window.requestAnimationFrame;
global.ResizeObserver = class {
  constructor(callback) {
    resizeObserverCallbacks.push(callback);
  }

  observe() {}
};

function flushAnimationFrames() {
  while (animationFrameQueue.length) {
    const callback = animationFrameQueue.shift();
    callback();
  }
}

const layoutChanges = [];
const manager = new PanelManager({
  onLayoutChange: (snapshot) => layoutChanges.push(snapshot),
});
const settingsPanel = createPanel("settingsPanel", {
  left: 1488,
  top: 88,
  width: 340,
  height: 610,
});
manager.registerPanel(settingsPanel);

assert.equal(resizeObserverCallbacks.length, 1);
resizeObserverCallbacks[0]();
assert.equal(
  layoutChanges.length,
  0,
  "初始 ResizeObserver 回调不应把程序化布局保存成共享状态",
);

flushAnimationFrames();
manager.enableResizeObserverLayoutPersistence();
settingsPanel.setRect({ left: 140, top: 120, width: 430, height: 620 });
resizeObserverCallbacks[0]();
assert.equal(layoutChanges.length, 1, "用户调整面板尺寸后仍应保存布局");

layoutChanges.length = 0;
global.window.innerWidth = 760;
global.window.innerHeight = 620;
manager.handleWindowResize();
resizeObserverCallbacks[0]();
assert.equal(
  layoutChanges.length,
  0,
  "浏览器窗口 resize / 视口夹取及其后续 ResizeObserver 不应覆盖跨浏览器共享布局",
);
