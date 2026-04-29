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
        _cache[0] || (_cache[0] = createStaticVNode("", 10)),
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
        _cache[1] || (_cache[1] = createStaticVNode("", 2))
      ]);
    };
  }
});
export {
  __pageData,
  _sfc_main as default
};
