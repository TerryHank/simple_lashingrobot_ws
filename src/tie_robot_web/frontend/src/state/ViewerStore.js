export class ViewerStore {
  constructor(initialState = {}) {
    this.state = {
      connection: {
        ready: false,
        level: "info",
        message: "连接中",
        url: "",
      },
      scene: {
        fixedFrame: "map",
        filteredWorldCoordCount: 0,
        rawWorldCoordCount: 0,
        tiePointCount: 0,
        planningPointCount: 0,
        tfFrameCount: 0,
      },
      topics: {},
      tf: {
        frames: [],
      },
      layout: {
        activeLayoutId: "executionDebug",
      },
      panels: {},
      problems: [],
      ...initialState,
    };
    this.listeners = new Set();
  }

  getState() {
    return this.state;
  }

  patch(nextState) {
    this.state = {
      ...this.state,
      ...nextState,
    };
    this.listeners.forEach((listener) => listener(this.state));
  }

  updateIn(key, nextValue) {
    this.state = {
      ...this.state,
      [key]: {
        ...(this.state[key] || {}),
        ...(nextValue || {}),
      },
    };
    this.listeners.forEach((listener) => listener(this.state));
  }

  subscribe(listener) {
    if (typeof listener !== "function") {
      return () => {};
    }
    this.listeners.add(listener);
    return () => {
      this.listeners.delete(listener);
    };
  }
}
