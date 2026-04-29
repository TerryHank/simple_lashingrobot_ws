import { v as onMounted, C as resolveComponent, o as openBlock, c as createElementBlock, ae as createStaticVNode, E as createVNode, w as withCtx, j as createBaseVNode, t as toDisplayString, p as ref, h as computed } from "./chunks/framework.BxzTKQzU.js";
const _hoisted_1 = { class: "gitnexus-actions" };
const _hoisted_2 = ["href"];
const _hoisted_3 = ["src"];
const __pageData = JSON.parse('{"title":"GitNexus 本地 WebUI 图谱","description":"","frontmatter":{},"headers":[],"relativePath":"reference/gitnexus-graph.md","filePath":"reference/gitnexus-graph.md"}');
const __default__ = { name: "reference/gitnexus-graph.md" };
const _sfc_main = /* @__PURE__ */ Object.assign(__default__, {
  setup(__props) {
    const browserHost = ref("");
    const projectName = "simple_lashingrobot_ws";
    onMounted(() => {
      browserHost.value = window.location.hostname || "127.0.0.1";
    });
    const gitnexusHost = computed(() => browserHost.value || "127.0.0.1");
    const webuiOrigin = computed(() => `http://${gitnexusHost.value}:5173`);
    const bridgeOrigin = computed(() => `http://${gitnexusHost.value}:4747`);
    const bridgeReposUrl = computed(() => `${bridgeOrigin.value}/api/repos`);
    const gitnexusWebuiUrl = computed(() => {
      const query = new URLSearchParams({
        server: bridgeOrigin.value,
        project: projectName
      });
      return `${webuiOrigin.value}/?${query.toString()}`;
    });
    return (_ctx, _cache) => {
      const _component_ClientOnly = resolveComponent("ClientOnly");
      return openBlock(), createElementBlock("div", null, [
        _cache[0] || (_cache[0] = createStaticVNode('<h1 id="gitnexus-本地-webui-图谱" tabindex="-1">GitNexus 本地 WebUI 图谱 <a class="header-anchor" href="#gitnexus-本地-webui-图谱" aria-label="Permalink to &quot;GitNexus 本地 WebUI 图谱&quot;">​</a></h1><p>本页使用 GitNexus 仓库里的 <code>gitnexus-web</code> 本地 WebUI 显示当前 <code>simple_lashingrobot_ws</code> 的知识图谱，不再跳转到外部托管版页面。</p><p>当前本地 GitNexus 索引已经解析出：</p><ul><li><code>3713 files</code></li><li><code>29351 symbols</code></li><li><code>46164 edges</code></li><li><code>404 clusters</code></li><li><code>300 processes</code></li></ul><h2 id="使用方式" tabindex="-1">使用方式 <a class="header-anchor" href="#使用方式" aria-label="Permalink to &quot;使用方式&quot;">​</a></h2><p>在工作区根目录启动本地 WebUI。脚本会在本机缓存 GitNexus 源码，启动 <code>gitnexus serve</code>，再启动 GitNexus WebUI 的 Vite 开发服务：</p><div class="language-bash vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">src/tie_robot_web/tools/run_gitnexus_local_webui.py</span></span></code></pre></div><p>如果要手动启动，分别运行：</p><div class="language-bash vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">gitnexus</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> serve</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> --host</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0.0.0.0</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> --port</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 4747</span></span>\n<span class="line"><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">cd</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> ~/.cache/tie_robot_web/GitNexus/gitnexus-web</span></span>\n<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">npm</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> install</span></span>\n<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">npm</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> run</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> dev</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> --</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> --host</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0.0.0.0</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> --port</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5173</span></span></code></pre></div><p>下面的 iframe 会使用当前浏览器访问帮助站时的主机名生成 WebUI 地址。也就是说，在本机打开时走本机地址，在局域网里通过 <code>192.168.x.x:8080</code> 打开时，会自动连接同一台机器的 <code>5173</code> 和 <code>4747</code> 端口。</p>', 10)),
        createVNode(_component_ClientOnly, null, {
          default: withCtx(() => [
            createBaseVNode("div", _hoisted_1, [
              createBaseVNode("a", {
                class: "gitnexus-open-link",
                href: gitnexusWebuiUrl.value,
                target: "_blank",
                rel: "noreferrer"
              }, " 在新标签打开 GitNexus WebUI ", 8, _hoisted_2),
              createBaseVNode("code", null, toDisplayString(webuiOrigin.value), 1),
              createBaseVNode("code", null, toDisplayString(bridgeReposUrl.value), 1)
            ]),
            createBaseVNode("iframe", {
              class: "gitnexus-webui-frame",
              title: "GitNexus WebUI - simple_lashingrobot_ws",
              src: gitnexusWebuiUrl.value,
              loading: "lazy",
              referrerpolicy: "no-referrer"
            }, null, 8, _hoisted_3)
          ]),
          _: 1
        }),
        _cache[1] || (_cache[1] = createStaticVNode('<h2 id="说明" tabindex="-1">说明 <a class="header-anchor" href="#说明" aria-label="Permalink to &quot;说明&quot;">​</a></h2><ul><li>WebUI 页面本身来自本机运行的 GitNexus <code>gitnexus-web</code>，关系图数据来自本机 <code>gitnexus serve</code> 暴露的本地索引。</li><li>如果 iframe 里显示无法连接，先确认脚本正在运行，并确认局域网客户端可以访问当前主机的 <code>5173</code> 和 <code>4747</code> 端口。</li><li>如果有防火墙，放行 <code>8080</code>、<code>5173</code>、<code>4747</code> 三个端口；<code>8080</code> 是帮助站，<code>5173</code> 是 GitNexus WebUI，<code>4747</code> 是 GitNexus API。</li><li>如果仓库结构有较大改动，先执行 <code>gitnexus analyze</code> 更新索引，再刷新本页。</li></ul>', 2))
      ]);
    };
  }
});
export {
  __pageData,
  _sfc_main as default
};
