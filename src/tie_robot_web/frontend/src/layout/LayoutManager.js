export class LayoutManager {
  constructor({ defaults = {}, loadLayout, saveLayout } = {}) {
    this.defaults = defaults;
    this.loadLayout = typeof loadLayout === "function" ? loadLayout : () => null;
    this.saveLayout = typeof saveLayout === "function" ? saveLayout : () => {};
  }

  getLayout(layoutId) {
    const savedLayout = this.loadLayout(layoutId);
    if (savedLayout) {
      return savedLayout;
    }
    return this.defaults[layoutId] || null;
  }

  listLayouts() {
    return Object.values(this.defaults);
  }

  persistLayout(layoutId, layout) {
    this.saveLayout(layoutId, layout);
  }
}
