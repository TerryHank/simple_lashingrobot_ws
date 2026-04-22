import { _ as _imports_4 } from "./chunks/05.D_qNBoot.js";
import { _ as _export_sfc, o as openBlock, c as createElementBlock, ae as createStaticVNode } from "./chunks/framework.BxzTKQzU.js";
const _imports_0 = "/help/assets/01.BJyNrZw-.png";
const _imports_1 = "/help/assets/02.BDYjfQEj.png";
const _imports_2 = "/help/assets/03.K6JZUk62.png";
const _imports_3 = "/help/assets/04.Dt863xGF.png";
const _imports_5 = "/help/assets/06.DAWmvZM9.png";
const __pageData = JSON.parse('{"title":"4. FAQ","description":"","frontmatter":{},"headers":[],"relativePath":"camera-sdk/vendor-vzense/zh-cn/ScepterGUITool/FAQ.md","filePath":"camera-sdk/vendor-vzense/zh-cn/ScepterGUITool/FAQ.md"}');
const _sfc_main = { name: "camera-sdk/vendor-vzense/zh-cn/ScepterGUITool/FAQ.md" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, [..._cache[0] || (_cache[0] = [
    createStaticVNode('<h1 id="_4-faq" tabindex="-1">4. FAQ <a class="header-anchor" href="#_4-faq" aria-label="Permalink to &quot;4. FAQ&quot;">​</a></h1><p><strong>Q1：为什么可以搜索到网口类设备但却打不开相机？</strong></p><p><strong>A1</strong>: 1.确认相机的连接和供电没有问题，相机的蓝色指示灯闪烁</p><p>2.查看相机 IP（默认 192.168.1.101）是否能 ping 通：</p><h4 id="windows" tabindex="-1"><strong>Windows</strong> <a class="header-anchor" href="#windows" aria-label="Permalink to &quot;**Windows**&quot;">​</a></h4><p>①Win+R 打开运行命令，输入&#39;cmd&#39;点击确定，打开命令行程序；</p><p><img src="' + _imports_0 + '" alt="打开cmd"></p><p>② 输入&#39;ping 192.168.1.101&#39;回车等待，即可查看相机是否 ping 通(相机默认 ip 地址为 192.168.1.101)；</p><p><img src="' + _imports_1 + '" alt="ping通相机"></p><p>③ 若无法 ping 通，打开 cmd，输入&#39;ipconfig&#39;查看主机端的 IP 是否和相机默认 IP 处于同一网段；</p><p><img src="' + _imports_2 + '" alt="ipconfig"></p><p>④ 若可以 ping 通，查看防火墙是否关闭，或允许工具使用公用网络和专用网络</p><p><img src="' + _imports_3 + '" alt="设置防火墙"></p><p>如以上措施均不能解决问题，请用 ipconfig 指令查看 PC 端网络状态，将除与相机同一网段之外的网络禁用再次搜索设备。</p><h4 id="ubuntu" tabindex="-1"><strong>Ubuntu</strong> <a class="header-anchor" href="#ubuntu" aria-label="Permalink to &quot;**Ubuntu**&quot;">​</a></h4><p>① 打开 Ubuntu 终端，输入&#39;ping -c 5 192.168.1.101&#39;回车等待，即可查看相机是否 ping 通(相机默认 ip 地址为 192.168.1.101)；</p><p><img src="' + _imports_4 + '" alt="ping通相机"></p><p>② 若无法 ping 通，打开终端，输入&#39;ifconfig&#39;查看主机端的 IP 是否和相机默认 IP 处于同一网段；</p><p><img src="' + _imports_5 + '" alt="Ubuntu ifconfig"></p><p>如以上措施均不能解决问题，请用 ifconfig 指令查看 PC 端网络状态，将除与相机同一网段之外的网络禁用再次搜索设备。</p><p><strong>Q2:  通过 GUI 保存的 IR 图和深度图为什么打不开？点云图如何查看？</strong></p><p><strong>A2</strong>: ScepterGUITool 保存的 IR 和 Depth 图像是 16bit 图片数据，可以使用 ImageJ 打开查看；GUITool 保存的点云图是.txt 格式，可使用 CloudCompare 进行查看。</p><p><strong>Q3:  如何改善相机对黑色物体的检测</strong></p><p><strong>A3</strong>:  可以尝试下述方法：1.将产品帧率降低（例如 5 帧），调整曝光时间到当前帧率下的最大值；2.将 Confidence 滤波阈值更改为 2 或 5</p>', 24)
  ])]);
}
const FAQ = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  FAQ as default
};
