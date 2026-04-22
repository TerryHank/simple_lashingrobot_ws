export class PanelManager {
  constructor() {
    this.panels = new Map();
  }

  registerPanel(panel) {
    if (!panel) {
      return;
    }
    const header = panel.querySelector(".panel-header");
    if (!header) {
      return;
    }

    let dragging = false;
    let originX = 0;
    let originY = 0;
    let startX = 0;
    let startY = 0;

    const bringToFront = () => {
      let maxZ = 20;
      this.panels.forEach((candidate) => {
        maxZ = Math.max(maxZ, Number(candidate.style.zIndex || 20));
      });
      panel.style.zIndex = String(maxZ + 1);
    };

    const onPointerMove = (event) => {
      if (!dragging) {
        return;
      }
      const dx = event.clientX - startX;
      const dy = event.clientY - startY;
      panel.style.left = `${originX + dx}px`;
      panel.style.top = `${originY + dy}px`;
      panel.style.right = "auto";
    };

    const onPointerUp = () => {
      dragging = false;
      document.removeEventListener("pointermove", onPointerMove);
      document.removeEventListener("pointerup", onPointerUp);
    };

    header.addEventListener("pointerdown", (event) => {
      if (window.innerWidth <= 1100) {
        return;
      }
      const interactiveTarget = event.target.closest("button, a, input, select, label");
      if (interactiveTarget) {
        return;
      }
      dragging = true;
      bringToFront();
      const rect = panel.getBoundingClientRect();
      originX = rect.left;
      originY = rect.top;
      startX = event.clientX;
      startY = event.clientY;
      panel.style.left = `${rect.left}px`;
      panel.style.top = `${rect.top}px`;
      panel.style.right = "auto";
      document.addEventListener("pointermove", onPointerMove);
      document.addEventListener("pointerup", onPointerUp);
    });

    this.panels.set(panel.id, panel);
  }

  init(root) {
    root.querySelectorAll(".floating-panel").forEach((panel) => this.registerPanel(panel));
  }
}
