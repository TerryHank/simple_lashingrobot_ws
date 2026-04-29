export class PanelManager {
  constructor({ onLayoutChange = null } = {}) {
    this.panels = new Map();
    this.resizeObservers = new Map();
    this.onLayoutChange = typeof onLayoutChange === "function" ? onLayoutChange : null;
    this.suppressLayoutChange = false;
    this.handleWindowResize = this.handleWindowResize.bind(this);
  }

  notifyLayoutChange() {
    if (this.suppressLayoutChange) {
      return;
    }
    this.onLayoutChange?.(this.getPanelLayoutSnapshot());
  }

  getCenteredPanelRect(width, height, verticalOffset = 0) {
    const viewportWidth = Math.max(320, window.innerWidth);
    const viewportHeight = Math.max(320, window.innerHeight);
    const boundedWidth = Math.min(
      Math.max(280, Math.round(width)),
      Math.max(280, viewportWidth - 40),
    );
    const boundedHeight = Math.min(
      Math.max(100, Math.round(height)),
      Math.max(100, viewportHeight - 120),
    );
    const left = Math.max(20, Math.round((viewportWidth - boundedWidth) / 2));
    const rawTop = Math.round((viewportHeight - boundedHeight) / 2) + verticalOffset;
    const maxTop = Math.max(88, viewportHeight - boundedHeight - 20);
    const top = Math.min(Math.max(88, rawTop), maxTop);
    return { left, top, width: boundedWidth, height: boundedHeight };
  }

  getDefaultPanelRect(panelId) {
    if (panelId === "controlPanel") {
      const width = Math.min(360, Math.max(320, window.innerWidth - 720));
      const height = Math.min(640, Math.max(360, window.innerHeight - 140));
      return this.getCenteredPanelRect(width, height);
    }

    if (panelId === "imagePanel") {
      const width = Math.min(760, Math.max(460, window.innerWidth - 560));
      const height = Math.min(560, Math.max(320, window.innerHeight - 220));
      return this.getCenteredPanelRect(width, height);
    }

    if (panelId === "settingsPanel") {
      const width = Math.min(380, Math.max(320, window.innerWidth - 720));
      const height = Math.min(640, Math.max(360, window.innerHeight - 140));
      return this.getCenteredPanelRect(width, height);
    }

    if (panelId === "logPanel") {
      const width = Math.min(700, Math.max(520, window.innerWidth - 720));
      const height = Math.min(320, Math.max(240, window.innerHeight - 860));
      return this.getCenteredPanelRect(width, height);
    }

    if (panelId === "terminalPanel") {
      const width = Math.min(920, Math.max(620, window.innerWidth - 320));
      const height = Math.min(460, Math.max(320, window.innerHeight - 220));
      return this.getCenteredPanelRect(width, height);
    }

    return null;
  }

  isUsablePanelRect(rect) {
    return Number.isFinite(Number(rect?.left))
      && Number.isFinite(Number(rect?.top))
      && Number.isFinite(Number(rect?.width))
      && Number.isFinite(Number(rect?.height))
      && Number(rect.width) >= 160
      && Number(rect.height) >= 100;
  }

  applyDefaultPanelRect(panelId) {
    const state = this.panels.get(panelId);
    if (!state) {
      return;
    }

    const defaultRect = this.getDefaultPanelRect(panelId);
    if (!defaultRect) {
      return;
    }

    const { panel } = state;
    panel.classList.remove("maximized");
    panel.style.left = `${defaultRect.left}px`;
    panel.style.top = `${defaultRect.top}px`;
    panel.style.width = `${defaultRect.width}px`;
    panel.style.height = `${defaultRect.height}px`;
    panel.style.right = "auto";
    panel.style.bottom = "auto";
    panel.style.transform = "";
    state.maximized = false;
    state.restoreRect = null;
    state.hasSavedLayout = true;
    state.lastVisibleRect = { ...defaultRect };

    const button = panel.querySelector("[data-panel-maximize]");
    if (button) {
      button.textContent = "⛶";
      button.title = "最大化";
    }

    state.bringToFront?.();
    this.notifyLayoutChange();
  }

  init(root) {
    root.querySelectorAll(".floating-panel").forEach((panel) => this.registerPanel(panel));
    window.addEventListener("resize", this.handleWindowResize);
  }

  registerPanel(panel) {
    if (!panel || this.panels.has(panel.id)) {
      return;
    }

    const panelId = panel.id;
    const maximizeButton = panel.querySelector("[data-panel-maximize]");
    const panelHeader = panel.querySelector(".panel-header");
    const imageResizeHandle = panel.querySelector("[data-image-panel-resize]");
    const state = {
      panel,
      maximized: false,
      restoreRect: null,
      hasSavedLayout: false,
      lastVisibleRect: null,
    };

    const bringToFront = () => {
      const allPanels = document.querySelectorAll(".floating-panel, .graphical-app-panel");
      let maxZIndex = 20;

      allPanels.forEach((candidate) => {
        const zIndex = parseInt(window.getComputedStyle(candidate).zIndex || "20", 10);
        if (candidate !== panel && zIndex > maxZIndex) {
          maxZIndex = zIndex;
        }
        candidate.classList.remove("is-window-active");
      });

      panel.classList.add("is-window-active");
      panel.style.zIndex = String(maxZIndex + 1);
    };

    const normalizePanelGeometry = () => {
      const rect = panel.getBoundingClientRect();
      panel.style.left = `${rect.left}px`;
      panel.style.top = `${rect.top}px`;
      panel.style.width = `${rect.width}px`;
      panel.style.height = `${rect.height}px`;
      panel.style.right = "auto";
      panel.style.bottom = "auto";
      panel.style.transform = "";
    };

    const getImageAspectRatio = () => {
      const imageCanvas = panel.querySelector("#irCanvas");
      const width = Number(imageCanvas?.width) || 640;
      const height = Number(imageCanvas?.height) || 480;
      return height > 0 ? width / height : 4 / 3;
    };

    const applyImagePanelAspectRatio = () => {
      if (panelId !== "imagePanel") {
        return;
      }
      const rect = panel.getBoundingClientRect();
      const aspect = getImageAspectRatio();
      const headerHeight = panelHeader?.offsetHeight || 52;
      const viewportMaxWidth = Math.max(320, window.innerWidth - 40);
      const viewportMaxHeight = Math.max(220, window.innerHeight - 108);

      let nextWidth = Math.min(rect.width, viewportMaxWidth);
      let nextContentHeight = nextWidth / aspect;
      let nextHeight = headerHeight + nextContentHeight;

      if (nextHeight > viewportMaxHeight) {
        nextHeight = viewportMaxHeight;
        nextContentHeight = Math.max(120, nextHeight - headerHeight);
        nextWidth = nextContentHeight * aspect;
      }

      panel.style.width = `${nextWidth}px`;
      panel.style.height = `${headerHeight + nextContentHeight}px`;
    };

    normalizePanelGeometry();
    applyImagePanelAspectRatio();

    panel.addEventListener("mousedown", () => {
      bringToFront();
    });

    if (panelHeader) {
      let isDragging = false;
      let dragOffsetX = 0;
      let dragOffsetY = 0;

      const clampToViewport = () => {
        const rect = panel.getBoundingClientRect();
        const maxLeft = Math.max(20, window.innerWidth - rect.width - 20);
        const maxTop = Math.max(88, window.innerHeight - rect.height - 20);
        const nextLeft = Math.min(Math.max(rect.left, 20), maxLeft);
        const nextTop = Math.min(Math.max(rect.top, 88), maxTop);
        panel.style.left = `${nextLeft}px`;
        panel.style.top = `${nextTop}px`;
      };

      panelHeader.addEventListener("mousedown", (event) => {
        if (state.maximized) {
          return;
        }
        if (event.button !== 0) {
          return;
        }
        if (event.target.closest(".panel-action-btn") || event.target.closest(".panel-header-field")) {
          return;
        }
        normalizePanelGeometry();
        bringToFront();
        isDragging = true;
        dragOffsetX = event.clientX - panel.offsetLeft;
        dragOffsetY = event.clientY - panel.offsetTop;
        document.body.style.userSelect = "none";
        event.preventDefault();
      });

      window.addEventListener("mousemove", (event) => {
        if (!isDragging) {
          return;
        }
        const maxLeft = Math.max(20, window.innerWidth - panel.offsetWidth - 20);
        const maxTop = Math.max(88, window.innerHeight - panel.offsetHeight - 20);
        const nextLeft = Math.min(Math.max(event.clientX - dragOffsetX, 20), maxLeft);
        const nextTop = Math.min(Math.max(event.clientY - dragOffsetY, 88), maxTop);
        panel.style.left = `${nextLeft}px`;
        panel.style.top = `${nextTop}px`;
      });

      window.addEventListener("mouseup", () => {
        if (!isDragging) {
          return;
        }
        isDragging = false;
        document.body.style.userSelect = "";
        clampToViewport();
        this.notifyLayoutChange();
      });
    }

    if (panelId === "imagePanel") {
      let syncingImagePanel = false;
      const observer = new ResizeObserver(() => {
        if (syncingImagePanel || state.maximized) {
          return;
        }
        syncingImagePanel = true;
        applyImagePanelAspectRatio();
        requestAnimationFrame(() => {
          syncingImagePanel = false;
        });
      });
      observer.observe(panel);
      this.resizeObservers.set(panelId, observer);

      if (panelId === "imagePanel" && imageResizeHandle) {
        let isResizing = false;
        let resizeStartX = 0;
        let resizeStartWidth = 0;
        let resizeStartLeft = 0;

        const stopResize = () => {
          if (!isResizing) {
            return;
          }
          isResizing = false;
          document.body.style.userSelect = "";
          panel.classList.remove("is-resizing");
          window.removeEventListener("mousemove", handleResizeMove);
          window.removeEventListener("mouseup", stopResize);
          this.notifyLayoutChange();
        };

        const handleResizeMove = (event) => {
          if (!isResizing || state.maximized) {
            return;
          }
          const aspect = getImageAspectRatio();
          const headerHeight = panelHeader?.offsetHeight || 52;
          const viewportMaxWidth = Math.max(320, window.innerWidth - 40);
          const viewportMaxHeight = Math.max(220, window.innerHeight - 108);
          const minWidth = 320;
          const deltaX = event.clientX - resizeStartX;

          let nextWidth = Math.min(
            Math.max(minWidth, resizeStartWidth + deltaX),
            viewportMaxWidth,
          );
          let nextContentHeight = nextWidth / aspect;
          let nextHeight = headerHeight + nextContentHeight;

          if (nextHeight > viewportMaxHeight) {
            nextHeight = viewportMaxHeight;
            nextContentHeight = Math.max(120, nextHeight - headerHeight);
            nextWidth = nextContentHeight * aspect;
          }

          const maxLeft = Math.max(20, window.innerWidth - nextWidth - 20);
          const currentTop = Math.min(
            Math.max(panel.offsetTop, 88),
            Math.max(88, window.innerHeight - nextHeight - 20),
          );

          panel.style.left = `${Math.min(Math.max(resizeStartLeft, 20), maxLeft)}px`;
          panel.style.top = `${currentTop}px`;
          panel.style.width = `${nextWidth}px`;
          panel.style.height = `${headerHeight + nextContentHeight}px`;
          panel.style.right = "auto";
          panel.style.bottom = "auto";
        };

        imageResizeHandle.addEventListener("mousedown", (event) => {
          if (state.maximized || event.button !== 0) {
            return;
          }
          normalizePanelGeometry();
          bringToFront();
          isResizing = true;
          resizeStartX = event.clientX;
          resizeStartWidth = panel.offsetWidth;
          resizeStartLeft = panel.offsetLeft;
          document.body.style.userSelect = "none";
          panel.classList.add("is-resizing");
          window.addEventListener("mousemove", handleResizeMove);
          window.addEventListener("mouseup", stopResize);
          event.preventDefault();
          event.stopPropagation();
        });
      }
    }

    if (maximizeButton) {
      maximizeButton.addEventListener("click", (event) => {
        event.preventDefault();
        event.stopPropagation();
        this.toggleMaximize(panelId);
      });
    }

    if (panelId !== "imagePanel") {
      const observer = new ResizeObserver(() => {
        this.notifyLayoutChange();
      });
      observer.observe(panel);
      this.resizeObservers.set(panelId, observer);
    }

    this.panels.set(panelId, { ...state, bringToFront });
  }

  getMaximizedRect() {
    const toolbar = document.querySelector(".top-toolbar");
    const toolbarBottom = toolbar ? toolbar.getBoundingClientRect().bottom : 68;
    const top = Math.max(88, Math.round(toolbarBottom + 12));
    return {
      left: 20,
      top,
      width: Math.max(320, window.innerWidth - 40),
      height: Math.max(220, window.innerHeight - top - 20),
    };
  }

  setPanelMaximized(panelId, maximized) {
    const state = this.panels.get(panelId);
    if (!state) {
      return;
    }

    const { panel } = state;
    const button = panel.querySelector("[data-panel-maximize]");

    if (maximized) {
      const rect = panel.getBoundingClientRect();
      state.restoreRect = {
        left: rect.left,
        top: rect.top,
        width: rect.width,
        height: rect.height,
      };
      state.lastVisibleRect = { ...state.restoreRect };

      const target = this.getMaximizedRect();
      panel.classList.add("maximized");
      panel.style.left = `${target.left}px`;
      panel.style.top = `${target.top}px`;
      panel.style.width = `${target.width}px`;
      panel.style.height = `${target.height}px`;
      panel.style.right = "auto";
      panel.style.bottom = "auto";
      panel.style.transform = "";
      state.maximized = true;
      if (button) {
        button.textContent = "❐";
        button.title = "还原";
      }
      state.bringToFront?.();
      this.notifyLayoutChange();
      return;
    }

    panel.classList.remove("maximized");
    if (state.restoreRect) {
      panel.style.left = `${state.restoreRect.left}px`;
      panel.style.top = `${state.restoreRect.top}px`;
      panel.style.width = `${state.restoreRect.width}px`;
      panel.style.height = `${state.restoreRect.height}px`;
      panel.style.right = "auto";
      panel.style.bottom = "auto";
      panel.style.transform = "";
    }
    state.maximized = false;
    if (button) {
      button.textContent = "⛶";
      button.title = "最大化";
    }
    this.notifyLayoutChange();
  }

  toggleMaximize(panelId) {
    const state = this.panels.get(panelId);
    if (!state) {
      return;
    }
    this.setPanelMaximized(panelId, !state.maximized);
  }

  handleWindowResize() {
    this.panels.forEach((state) => {
      if (!state.maximized) {
        const rect = state.panel.getBoundingClientRect();
        const maxLeft = Math.max(20, window.innerWidth - rect.width - 20);
        const maxTop = Math.max(88, window.innerHeight - rect.height - 20);
        state.panel.style.left = `${Math.min(Math.max(rect.left, 20), maxLeft)}px`;
        state.panel.style.top = `${Math.min(Math.max(rect.top, 88), maxTop)}px`;
        if (state.panel.id === "imagePanel") {
          const aspect = (() => {
            const imageCanvas = state.panel.querySelector("#irCanvas");
            const width = Number(imageCanvas?.width) || 640;
            const height = Number(imageCanvas?.height) || 480;
            return height > 0 ? width / height : 4 / 3;
          })();
          const headerHeight = state.panel.querySelector(".panel-header")?.offsetHeight || 52;
          const viewportMaxHeight = Math.max(220, window.innerHeight - 108);
          let width = rect.width;
          let contentHeight = width / aspect;
          if (headerHeight + contentHeight > viewportMaxHeight) {
            contentHeight = Math.max(120, viewportMaxHeight - headerHeight);
            width = contentHeight * aspect;
          }
          state.panel.style.width = `${width}px`;
          state.panel.style.height = `${headerHeight + contentHeight}px`;
        }
        return;
      }
      const target = this.getMaximizedRect();
      state.panel.style.left = `${target.left}px`;
      state.panel.style.top = `${target.top}px`;
      state.panel.style.width = `${target.width}px`;
      state.panel.style.height = `${target.height}px`;
      state.panel.style.right = "auto";
      state.panel.style.bottom = "auto";
      state.panel.style.transform = "";
    });
    this.notifyLayoutChange();
  }

  hasPanelLayout(panelId) {
    const state = this.panels.get(panelId);
    return Boolean(state?.hasSavedLayout);
  }

  applyPanelLayout(layout) {
    const layoutPanels = layout?.panels || {};
    this.suppressLayoutChange = true;
    Object.entries(layoutPanels).forEach(([panelId, panelLayout]) => {
      const state = this.panels.get(panelId);
      if (!state || !panelLayout?.rect) {
        return;
      }
      const { panel } = state;
      const rect = panelLayout.rect;
      if (!this.isUsablePanelRect(rect)) {
        return;
      }
      panel.classList.toggle("maximized", Boolean(panelLayout.maximized));
      panel.style.left = `${Number(rect.left) || 20}px`;
      panel.style.top = `${Number(rect.top) || 88}px`;
      panel.style.width = `${Math.max(280, Number(rect.width) || panel.offsetWidth || 320)}px`;
      panel.style.height = `${Math.max(100, Number(rect.height) || panel.offsetHeight || 240)}px`;
      panel.style.right = "auto";
      panel.style.bottom = "auto";
      panel.style.transform = "";
      state.maximized = Boolean(panelLayout.maximized);
      state.restoreRect = panelLayout.restoreRect || null;
      state.hasSavedLayout = true;
      state.lastVisibleRect = {
        left: Math.round(Number(rect.left) || 20),
        top: Math.round(Number(rect.top) || 88),
        width: Math.round(Math.max(280, Number(rect.width) || panel.offsetWidth || 320)),
        height: Math.round(Math.max(100, Number(rect.height) || panel.offsetHeight || 240)),
      };
      const button = panel.querySelector("[data-panel-maximize]");
      if (button) {
        button.textContent = state.maximized ? "❐" : "⛶";
        button.title = state.maximized ? "还原" : "最大化";
      }
    });
    this.suppressLayoutChange = false;
  }

  getPanelLayoutSnapshot() {
    const panels = {};
    this.panels.forEach((state, panelId) => {
      const rect = state.panel.getBoundingClientRect();
      const currentRect = {
        left: Math.round(rect.left),
        top: Math.round(rect.top),
        width: Math.round(rect.width),
        height: Math.round(rect.height),
      };
      const storedRect = this.isUsablePanelRect(currentRect)
        ? currentRect
        : state.lastVisibleRect;
      if (storedRect) {
        state.lastVisibleRect = { ...storedRect };
      }
      panels[panelId] = {
        ...(storedRect ? { rect: storedRect } : {}),
        maximized: Boolean(state.maximized),
        restoreRect: state.restoreRect || null,
      };
    });
    return panels;
  }
}
