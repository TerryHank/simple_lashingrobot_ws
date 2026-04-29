import { Terminal } from "@xterm/xterm";
import { FitAddon } from "@xterm/addon-fit";
import "@xterm/xterm/css/xterm.css";

const DEFAULT_TERMINAL_THEME = {
  dark: {
    background: "#11161d",
    foreground: "#edf4ff",
    cursor: "#7ab9ff",
    cursorAccent: "#11161d",
    selectionBackground: "rgba(122, 185, 255, 0.28)",
    black: "#0c1015",
    red: "#ff7a7a",
    green: "#8bf0a8",
    yellow: "#f6d06f",
    blue: "#5ba8ff",
    magenta: "#c68cff",
    cyan: "#7fe8ff",
    white: "#f5fbff",
    brightBlack: "#556272",
    brightRed: "#ff9b9b",
    brightGreen: "#aff7c0",
    brightYellow: "#ffe29a",
    brightBlue: "#89c2ff",
    brightMagenta: "#dbabff",
    brightCyan: "#a5f2ff",
    brightWhite: "#ffffff",
  },
  light: {
    background: "#f7f9fc",
    foreground: "#1e2734",
    cursor: "#2563eb",
    cursorAccent: "#f7f9fc",
    selectionBackground: "rgba(37, 99, 235, 0.18)",
    black: "#283345",
    red: "#c74444",
    green: "#287d4a",
    yellow: "#9c6b16",
    blue: "#2563eb",
    magenta: "#9446d8",
    cyan: "#0f7490",
    white: "#f7f9fc",
    brightBlack: "#536177",
    brightRed: "#da5c5c",
    brightGreen: "#39955e",
    brightYellow: "#b88521",
    brightBlue: "#4680f0",
    brightMagenta: "#ab66e6",
    brightCyan: "#1794b2",
    brightWhite: "#ffffff",
  },
};

function getTerminalTheme(theme) {
  return DEFAULT_TERMINAL_THEME[theme] || DEFAULT_TERMINAL_THEME.dark;
}

function buildTerminalWsUrl(session) {
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  const host = window.location.hostname;
  const port = Number(session.terminalPort || 8081);
  return `${protocol}//${host}:${port}${session.wsPath}`;
}

export class TerminalController {
  constructor({ ui, callbacks = {} }) {
    this.ui = ui;
    this.callbacks = callbacks;
    this.sessions = new Map();
    this.activeSessionId = null;
    this.terminalViewport = null;
    this.resizeObserver = null;
    this.theme = document.documentElement.getAttribute("data-theme") || "dark";
    this.terminalConfig = null;
    this.sessionLabelSyncTimer = null;
  }

  init() {
    this.terminalViewport = this.ui.getTerminalViewport();
    this.resizeObserver = new ResizeObserver(() => {
      this.fitActiveSession();
    });
    if (this.terminalViewport) {
      this.resizeObserver.observe(this.terminalViewport);
    }
    this.ui.renderTerminalSessions([], null);
    this.startSessionLabelSync();
    return this;
  }

  async ensureConfig() {
    return this.fetchConfig();
  }

  async fetchConfig() {
    if (this.terminalConfig) {
      return this.terminalConfig;
    }
    const response = await fetch("/api/terminal/config", { cache: "no-store" });
    const payload = await response.json();
    if (!response.ok || payload?.success === false) {
      throw new Error(payload?.message || "读取终端配置失败");
    }
    this.terminalConfig = payload;
    return this.terminalConfig;
  }

  async hydrateExistingSessions() {
    this.terminalConfig = null;
    const config = await this.ensureConfig();
    const sessions = Array.isArray(config?.sessions) ? config.sessions : [];
    sessions
      .map((session) => this.normalizeSessionPayload(session, config))
      .filter((session) => session?.sessionId && !this.sessions.has(session.sessionId))
      .forEach((session) => {
        this.mountSession(session);
        this.connectSessionSocket(session.sessionId);
      });
    if (!this.activeSessionId && this.sessions.size > 0) {
      const [firstSessionId] = this.sessions.keys();
      this.activateSession(firstSessionId);
    }
    return this.listSessionSummaries();
  }

  startSessionLabelSync() {
    if (this.sessionLabelSyncTimer) {
      return;
    }
    this.sessionLabelSyncTimer = window.setInterval(() => {
      this.refreshSessionSummaries({ suppressLog: true });
    }, 2500);
  }

  async refreshSessionSummaries(options = {}) {
    const suppressLog = Boolean(options?.suppressLog);
    try {
      this.terminalConfig = null;
      const config = await this.ensureConfig();
      const remoteSessions = Array.isArray(config?.sessions) ? config.sessions : [];
      const remoteSessionIds = new Set();
      let changed = false;
      remoteSessions
        .map((session) => this.normalizeSessionPayload(session, config))
        .filter((session) => session?.sessionId)
        .forEach((session) => {
          remoteSessionIds.add(session.sessionId);
          const existing = this.sessions.get(session.sessionId);
          if (!existing) {
            this.mountSession(session);
            this.connectSessionSocket(session.sessionId);
            changed = true;
            return;
          }
          if (existing.label !== session.label) {
            existing.label = session.label;
            changed = true;
          }
          existing.terminalPort = session.terminalPort;
          existing.wsPath = session.wsPath;
        });

      [...this.sessions.entries()].forEach(([sessionId, session]) => {
        if (remoteSessionIds.has(sessionId)) {
          return;
        }
        if (session.socket && session.socket.readyState <= WebSocket.OPEN) {
          session.socket.close();
        }
        session.terminal.dispose();
        session.element.remove();
        this.sessions.delete(sessionId);
        changed = true;
      });

      if (this.activeSessionId && !this.sessions.has(this.activeSessionId)) {
        this.activeSessionId = this.sessions.keys().next().value || null;
      }
      if (changed) {
        this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
      }
    } catch (error) {
      if (!suppressLog) {
        this.report(`同步 tmux 窗口名称失败：${error?.message || String(error)}`, "warn");
      }
    }
  }

  normalizeSessionPayload(session, config = null) {
    if (!session) {
      return null;
    }
    const sessionId = session.sessionId || session.session_id;
    if (!sessionId) {
      return null;
    }
    return {
      ...session,
      sessionId,
      label: session.label || session.tmuxSessionName || sessionId,
      terminalPort: session.terminalPort || config?.terminal_port || this.terminalConfig?.terminal_port || 8081,
      wsPath: session.wsPath || `/ws/terminal/${sessionId}`,
      state: session.state || "connecting",
    };
  }

  async ensureSession() {
    await this.hydrateExistingSessions();
    if (this.sessions.size > 0) {
      if (!this.activeSessionId) {
        const [firstSessionId] = this.sessions.keys();
        this.activateSession(firstSessionId);
      }
      return this.activeSessionId;
    }
    const session = await this.createSession();
    return session?.sessionId || null;
  }

  async createSession() {
    this.ui.setTerminalNotice("正在创建终端会话…", "info");
    const viewportRect = this.terminalViewport?.getBoundingClientRect();
    const config = await this.ensureConfig();
    const response = await fetch("/api/terminal/sessions", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        cols: 120,
        rows: Math.max(18, Math.floor((viewportRect?.height || 560) / 18)),
      }),
    });
    const payload = await response.json();
    if (!response.ok || payload?.success === false || !payload?.session) {
      throw new Error(payload?.message || "创建终端会话失败");
    }

    const session = {
      ...this.normalizeSessionPayload(payload.session, config),
      state: "connecting",
    };
    this.mountSession(session);
    this.connectSessionSocket(session.sessionId);
    this.activateSession(session.sessionId);
    this.ui.setTerminalNotice(null);
    this.report(`已创建终端：${session.label}`, "success");
    return session;
  }

  mountSession(session) {
    if (!this.terminalViewport) {
      return;
    }
    const sessionElement = document.createElement("div");
    sessionElement.className = "terminal-session";
    sessionElement.dataset.sessionId = session.sessionId;
    sessionElement.hidden = true;
    this.terminalViewport.appendChild(sessionElement);

    const terminal = new Terminal({
      cursorBlink: true,
      scrollback: 5000,
      fontFamily: '"Cascadia Code", "JetBrains Mono", "SFMono-Regular", "Consolas", monospace',
      fontSize: 13,
      lineHeight: 1.26,
      allowTransparency: true,
      theme: getTerminalTheme(this.theme),
    });
    const fitAddon = new FitAddon();
    terminal.loadAddon(fitAddon);
    terminal.open(sessionElement);

    const sessionState = {
      ...session,
      terminal,
      fitAddon,
      socket: null,
      element: sessionElement,
      state: "connecting",
    };

    terminal.onData((data) => {
      if (sessionState.socket?.readyState === WebSocket.OPEN) {
        sessionState.socket.send(JSON.stringify({ type: "input", data }));
      }
    });

    this.sessions.set(session.sessionId, sessionState);
    this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
  }

  connectSessionSocket(sessionId) {
    const session = this.sessions.get(sessionId);
    if (!session) {
      return;
    }
    const socket = new WebSocket(buildTerminalWsUrl(session));
    session.socket = socket;
    socket.addEventListener("open", () => {
      session.state = "ready";
      this.fitSession(sessionId, true);
      this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
    });
    socket.addEventListener("message", (event) => {
      this.handleSocketMessage(sessionId, event.data);
    });
    socket.addEventListener("close", () => {
      session.state = "closed";
      this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
      session.terminal.writeln("\r\n[终端连接已关闭]");
    });
    socket.addEventListener("error", () => {
      session.state = "error";
      this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
      session.terminal.writeln("\r\n[终端连接失败]");
    });
  }

  handleSocketMessage(sessionId, rawMessage) {
    const session = this.sessions.get(sessionId);
    if (!session) {
      return;
    }
    let payload = null;
    try {
      payload = JSON.parse(rawMessage);
    } catch {
      return;
    }
    switch (payload?.type) {
      case "ready":
        session.label = payload.label || session.label;
        session.state = "ready";
        this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
        this.fitSession(sessionId, true);
        break;
      case "history":
      case "output":
        session.terminal.write(payload.data || "");
        break;
      case "exit":
        session.state = "closed";
        session.terminal.writeln(`\r\n[终端已退出，退出码 ${payload.code ?? 0}]`);
        this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
        break;
      case "error":
        session.state = "error";
        session.terminal.writeln(`\r\n[错误] ${payload.message || "未知错误"}`);
        this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
        break;
      case "gui_session":
        this.callbacks.onGraphicalSession?.(payload.session);
        break;
      case "gui_sessions":
        this.callbacks.onGraphicalSessions?.(payload.sessions || []);
        break;
      case "gui_session_closed":
        this.callbacks.onGraphicalSessionClosed?.(payload.sessionId);
        break;
      default:
        break;
    }
  }

  listSessionSummaries() {
    return [...this.sessions.values()].map((session) => ({
      sessionId: session.sessionId,
      label: session.label,
      state: session.state,
      active: session.sessionId === this.activeSessionId,
    }));
  }

  activateSession(sessionId) {
    if (!this.sessions.has(sessionId)) {
      return;
    }
    this.activeSessionId = sessionId;
    this.sessions.forEach((session, id) => {
      session.element.hidden = id !== sessionId;
    });
    this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
    this.fitSession(sessionId, true);
  }

  async closeSession(sessionId) {
    const session = this.sessions.get(sessionId);
    if (!session) {
      return;
    }
    try {
      await fetch(`/api/terminal/sessions/${sessionId}`, {
        method: "DELETE",
      });
    } catch {
      // ignore close transport errors and clean up locally
    }
    if (session.socket && session.socket.readyState <= WebSocket.OPEN) {
      session.socket.close();
    }
    session.terminal.dispose();
    session.element.remove();
    this.sessions.delete(sessionId);
    if (this.activeSessionId === sessionId) {
      const nextActive = this.sessions.keys().next().value || null;
      this.activeSessionId = nextActive;
      if (nextActive) {
        this.activateSession(nextActive);
      }
    }
    this.ui.renderTerminalSessions(this.listSessionSummaries(), this.activeSessionId);
  }

  fitSession(sessionId, sendResize = false) {
    const session = this.sessions.get(sessionId);
    if (!session || session.element.hidden) {
      return;
    }
    requestAnimationFrame(() => {
      try {
        session.fitAddon.fit();
      } catch {
        return;
      }
      if (sendResize && session.socket?.readyState === WebSocket.OPEN) {
        session.socket.send(JSON.stringify({
          type: "resize",
          cols: session.terminal.cols,
          rows: session.terminal.rows,
        }));
      }
    });
  }

  fitActiveSession() {
    if (!this.activeSessionId) {
      return;
    }
    this.fitSession(this.activeSessionId, true);
  }

  setTheme(theme) {
    this.theme = theme === "light" ? "light" : "dark";
    const terminalTheme = getTerminalTheme(this.theme);
    this.sessions.forEach((session) => {
      session.terminal.options.theme = terminalTheme;
    });
  }

  async handle(action, sessionId = null) {
    if (action === "new") {
      await this.createSession();
      return;
    }
    if (action === "ensure") {
      await this.ensureSession();
      return;
    }
    if (action === "activate" && sessionId) {
      this.activateSession(sessionId);
      return;
    }
    if (action === "close" && sessionId) {
      await this.closeSession(sessionId);
    }
  }

  report(message, level = "info") {
    this.callbacks.onLog?.(message, level);
  }
}
