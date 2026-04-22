import { _ as _imports_0 } from "./chunks/01.BegxfoJl.js";
import { _ as _export_sfc, o as openBlock, c as createElementBlock, ae as createStaticVNode } from "./chunks/framework.BxzTKQzU.js";
const __pageData = JSON.parse('{"title":"5. FAQ","description":"","frontmatter":{},"headers":[],"relativePath":"camera-sdk/vendor-vzense/zh-cn/ScepterSDK/FAQ.md","filePath":"camera-sdk/vendor-vzense/zh-cn/ScepterSDK/FAQ.md"}');
const _sfc_main = { name: "camera-sdk/vendor-vzense/zh-cn/ScepterSDK/FAQ.md" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, [..._cache[0] || (_cache[0] = [
    createStaticVNode('<h1 id="_5-faq" tabindex="-1">5. FAQ <a class="header-anchor" href="#_5-faq" aria-label="Permalink to &quot;5. FAQ&quot;">​</a></h1><h2 id="_5-1-sdk-日志存放位置" tabindex="-1">5.1. SDK 日志存放位置 <a class="header-anchor" href="#_5-1-sdk-日志存放位置" aria-label="Permalink to &quot;5.1. SDK 日志存放位置&quot;">​</a></h2><p>Windows 上默认日志路径：C:\\Users\\&lt;user name&gt;\\AppData\\Local\\Scepter\\Log</p><p>Linux 上默认日志路径： /home/&lt;user name&gt;/.config/Scepter/Log</p><h2 id="_5-2-sdk-输出的点云格式" tabindex="-1">5.2. SDK 输出的点云格式 <a class="header-anchor" href="#_5-2-sdk-输出的点云格式" aria-label="Permalink to &quot;5.2. SDK 输出的点云格式&quot;">​</a></h2><p>SDK 输出的点云格式为从当前帧的第一个 pixel 至最后一个 pixel 的 X,Y,Z 值，以 float 型输出，单位是 mm，说明如下：</p><p><img src="' + _imports_0 + '" alt="SDK output pointcloud"></p><h2 id="_5-3-无法打开相机的排查步骤" tabindex="-1">5.3. 无法打开相机的排查步骤 <a class="header-anchor" href="#_5-3-无法打开相机的排查步骤" aria-label="Permalink to &quot;5.3. 无法打开相机的排查步骤&quot;">​</a></h2><p>搜索不到相机通常有以下几种情况：</p><ol><li>相机与主机端的接线是否良好，主机端的网卡是否可用</li><li>相机与主机不在同一网段。如果相机设置为非 DHCP 模式，请确保相机的固定 IP 于主机在同一网段，如 192.168.1.X。若相机设置为 DHCP 模式，请确保相机与主机处于同 一局域网下，并且路由器/交换机具有 DHCP sever 功能</li><li>软件的网络访问权限是否被限制</li><li>局域网的 9007， 9008， 9009 端口是否被禁止</li><li>相机的供电是否足够。如果使用非 POE 方式，请确保适配器打开并插入</li></ol><p>如果以上检查项都确认无问题，但仍无法打开相机，请联系 FAE 处理。</p>', 11)
  ])]);
}
const FAQ = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  FAQ as default
};
