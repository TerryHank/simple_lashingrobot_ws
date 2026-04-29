import { gitnexusSummary, packageEdges, packageNodes, systemFlows } from "./graphData.js";
import "./style.css";

const app = document.getElementById("projectGraphApp");
const nodeById = new Map(packageNodes.map((node) => [node.id, node]));

function formatNumber(value) {
  return new Intl.NumberFormat("zh-CN").format(value);
}

function edgeClass(edge, activeNodeId) {
  const isActive = !activeNodeId || edge.from === activeNodeId || edge.to === activeNodeId;
  return [
    "graph-edge",
    `is-${edge.strength}`,
    isActive ? "is-active" : "is-muted",
  ].join(" ");
}

function renderSummaryCards() {
  const cards = [
    ["Files", gitnexusSummary.files],
    ["Symbols", gitnexusSummary.symbols],
    ["Edges", gitnexusSummary.edges],
    ["Processes", gitnexusSummary.processes],
  ];
  return cards.map(([label, value], index) => `
    <article class="summary-card" style="--delay: ${index * 70}ms">
      <span>${label}</span>
      <strong>${formatNumber(value)}</strong>
    </article>
  `).join("");
}

function renderPackageGraph(activeNodeId = null) {
  const svgEdges = packageEdges.map((edge) => {
    const fromNode = nodeById.get(edge.from);
    const toNode = nodeById.get(edge.to);
    if (!fromNode || !toNode) {
      return "";
    }
    const midX = (fromNode.x + toNode.x) / 2;
    const midY = (fromNode.y + toNode.y) / 2;
    const bend = edge.strength === "support" ? -7 : 0;
    return `
      <path
        class="${edgeClass(edge, activeNodeId)}"
        data-edge-from="${edge.from}"
        data-edge-to="${edge.to}"
        d="M ${fromNode.x} ${fromNode.y} Q ${midX} ${midY + bend} ${toNode.x} ${toNode.y}"
      />
    `;
  }).join("");

  const nodeButtons = packageNodes.map((node) => {
    const isActive = !activeNodeId || activeNodeId === node.id;
    return `
      <button
        class="graph-node is-${node.tone} ${isActive ? "is-active" : "is-muted"}"
        data-node-id="${node.id}"
        style="left: ${node.x}%; top: ${node.y}%"
        type="button"
        aria-label="查看 ${node.label} 关系"
      >
        <span class="node-orbit"></span>
        <strong>${node.label}</strong>
        <small>${node.subtitle}</small>
        <em>${node.symbols} symbols</em>
      </button>
    `;
  }).join("");

  return `
    <div class="graph-board">
      <svg class="graph-lines" viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true">
        ${svgEdges}
      </svg>
      ${nodeButtons}
    </div>
  `;
}

function renderNodeDetails(activeNodeId = "tie_robot_process") {
  const node = nodeById.get(activeNodeId) || nodeById.get("tie_robot_process");
  const connectedEdges = packageEdges.filter((edge) => edge.from === node.id || edge.to === node.id);
  return `
    <section class="detail-card">
      <p class="eyebrow">Selected Package</p>
      <h2>${node.label}</h2>
      <p>${node.summary}</p>
      <div class="entry-list">
        ${node.entrypoints.map((entry) => `<span>${entry}</span>`).join("")}
      </div>
    </section>
    <section class="detail-card detail-card-edges">
      <p class="eyebrow">Relations</p>
      <h2>关系强度</h2>
      ${connectedEdges.map((edge) => `
        <article>
          <strong>${nodeById.get(edge.from)?.label || edge.from} -> ${nodeById.get(edge.to)?.label || edge.to}</strong>
          <span>${edge.label}</span>
          <p>${edge.description}</p>
        </article>
      `).join("")}
    </section>
  `;
}

function renderFlowCards() {
  return systemFlows.map((flow, index) => `
    <article class="flow-card" style="--delay: ${160 + index * 90}ms">
      <span class="flow-index">0${index + 1}</span>
      <h3>${flow.name}</h3>
      <div class="flow-route">
        ${flow.route.map((nodeId) => `<span>${nodeById.get(nodeId)?.subtitle || nodeId}</span>`).join("<b></b>")}
      </div>
      <p>${flow.detail}</p>
    </article>
  `).join("");
}

function render(activeNodeId = "tie_robot_process") {
  app.innerHTML = `
    <main class="project-graph-page">
      <section class="hero">
        <div>
          <p class="eyebrow">GitNexus Knowledge Graph</p>
          <h1>绑扎机器人工程关系图</h1>
          <p class="hero-copy">
            由 GitNexus 索引当前仓库后整理：${formatNumber(gitnexusSummary.files)} 个文件、
            ${formatNumber(gitnexusSummary.symbols)} 个符号、${formatNumber(gitnexusSummary.edges)} 条关系边。
            页面展示 ROS 包之间的主依赖、运行链路和调试入口。
          </p>
        </div>
        <aside class="index-stamp">
          <span>Indexed</span>
          <strong>${gitnexusSummary.indexedAt}</strong>
          <small>${gitnexusSummary.indexedCommit} / ${gitnexusSummary.source}</small>
        </aside>
      </section>

      <section class="summary-grid">
        ${renderSummaryCards()}
      </section>

      <section class="graph-shell">
        <div class="graph-header">
          <div>
            <p class="eyebrow">Package Topology</p>
            <h2>从启动到视觉、规划、执行、前端的主链</h2>
          </div>
          <button class="reset-button" type="button" data-reset-graph>显示全图</button>
        </div>
        ${renderPackageGraph(activeNodeId)}
      </section>

      <section class="detail-grid">
        ${renderNodeDetails(activeNodeId)}
      </section>

      <section class="flow-grid">
        ${renderFlowCards()}
      </section>
    </main>
  `;
  bindInteractions();
}

function bindInteractions() {
  app.querySelectorAll("[data-node-id]").forEach((button) => {
    button.addEventListener("click", () => {
      render(button.getAttribute("data-node-id"));
    });
  });
  app.querySelector("[data-reset-graph]")?.addEventListener("click", () => render(null));
}

render();
