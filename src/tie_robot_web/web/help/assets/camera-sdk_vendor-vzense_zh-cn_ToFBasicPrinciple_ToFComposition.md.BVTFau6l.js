import { _ as _imports_0 } from "./chunks/01.BgaCP8kg.js";
import { _ as _export_sfc, o as openBlock, c as createElementBlock, ae as createStaticVNode } from "./chunks/framework.BxzTKQzU.js";
const __pageData = JSON.parse('{"title":"3. ToF 相机的组成","description":"","frontmatter":{},"headers":[],"relativePath":"camera-sdk/vendor-vzense/zh-cn/ToFBasicPrinciple/ToFComposition.md","filePath":"camera-sdk/vendor-vzense/zh-cn/ToFBasicPrinciple/ToFComposition.md"}');
const _sfc_main = { name: "camera-sdk/vendor-vzense/zh-cn/ToFBasicPrinciple/ToFComposition.md" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, [..._cache[0] || (_cache[0] = [
    createStaticVNode('<h1 id="_3-tof-相机的组成" tabindex="-1">3. ToF 相机的组成 <a class="header-anchor" href="#_3-tof-相机的组成" aria-label="Permalink to &quot;3. ToF 相机的组成&quot;">​</a></h1><p>ToF 相机是指以光学系统为接受路径的面阵非扫描式 3D 成像深度信息捕捉技术，主要由照射单元，光学透镜，成像传感器，控制单元，计算单元等器件组成。</p><p><img src="' + _imports_0 + '" alt="ToF 相机的组成"></p><p><strong>照射单元：</strong></p><p>照射单元需要对光源进行脉冲调制之后再进行发射，调制的光脉冲频率可以高达 100MHz。因此，在图像拍摄过程中，光源会打开和关闭几千次，各个光脉冲只有几纳秒的时长。相机的曝光时间参数决定了每次成像的脉冲数。要实现精确测量，必须精确地控制光脉冲，使其具有完全相同的持续时间、上升时间和下降时间。因为即使很小的只是 1ns 的偏差即可产生高达 15cm 的距离测量误差。如此高的调制频率和精度只有采用精良的 LED 或激光二极管才能实现。一般照射光源都是采用人眼不可见的红外光源。</p><p><strong>光学透镜：</strong></p><p>用于汇聚反射光线，在光学传感器上成像，与普通光学镜头不同的是这里需要加一个带通滤光片来保证只有与照明光源波长相同的光才能进入，这样做的目的是抑制非相干光源减少噪声，同时防止感光传感器因外部光线干扰而过度曝光</p><p><strong>成像传感器：</strong></p><p>成像传感器是 TOF 相机的核心。该传感器结构与普通图像传感器类似，但比图像传感器更复杂，它包含 2 个或者更多快门，用来在不同时间采样反射光线。</p><p><strong>控制单元：</strong></p><p>相机的电子控制单元触发的光脉冲序列与芯片电子快门的开/闭精确同步。它对传感器电荷执行读出和转换，并将它们引导至分析单元和数据接口。</p><p><strong>计算单元：</strong></p><p>计算单元可以记录精确的深度图。深度图通常是灰度图，其中的每个值代表光反射表面和相机之间的距离。为了得到更好的效果，通常会进行数据校准。</p>', 13)
  ])]);
}
const ToFComposition = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  ToFComposition as default
};
