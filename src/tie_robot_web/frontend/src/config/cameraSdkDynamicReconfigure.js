export const CAMERA_SDK_PARAMETER_DEFINITIONS = Object.freeze([
  { name: "FrameRate", label: "帧率", type: "int", min: 1, max: 15, defaultValue: 5 },
  { name: "IRGMMGain", label: "红外增益", type: "int", min: 0, max: 100, defaultValue: 50 },
  {
    name: "ColorResloution",
    label: "彩色分辨率",
    type: "enum",
    defaultValue: 2,
    options: Object.freeze([
      { value: 0, label: "1600 x 1200" },
      { value: 1, label: "800 x 600" },
      { value: 2, label: "640 x 480" },
    ]),
  },
  {
    name: "XDRMode",
    label: "动态范围模式",
    type: "enum",
    defaultValue: 1,
    options: Object.freeze([
      { value: 0, label: "普通模式" },
      { value: 1, label: "高动态范围模式" },
      { value: 2, label: "宽动态范围模式" },
    ]),
  },
  { name: "ToFManual", label: "深度手动曝光", type: "bool", defaultValue: true },
  { name: "ToFExposureTime", label: "深度曝光时间", type: "int", min: 0, max: 5000, defaultValue: 3000 },
  { name: "FlyingPixelenable", label: "飞点过滤", type: "bool", defaultValue: false },
  { name: "FlyingPixelvalue", label: "飞点过滤强度", type: "int", min: 0, max: 16, defaultValue: 10 },
  { name: "Confidenceenable", label: "置信度过滤", type: "bool", defaultValue: false },
  { name: "Confidencevalue", label: "置信度阈值", type: "int", min: 0, max: 100, defaultValue: 4 },
  { name: "TimeFilterenable", label: "时域滤波", type: "bool", defaultValue: true },
  { name: "TimeFiltervalue", label: "时域滤波强度", type: "int", min: 0, max: 3, defaultValue: 3 },
  { name: "IRGMMCorrectionenable", label: "红外增益校正", type: "bool", defaultValue: false },
  { name: "IRGMMCorrectionvalue", label: "红外增益校正值", type: "int", min: 0, max: 100, defaultValue: 79 },
  { name: "SpatialFilterEnabled", label: "空间滤波", type: "bool", defaultValue: false },
  { name: "FillHoleFilterEnabled", label: "空洞填充", type: "bool", defaultValue: false },
  { name: "ColorManual", label: "彩色手动曝光", type: "bool", defaultValue: false },
  { name: "ColorExposureTime", label: "彩色曝光时间", type: "int", min: 100, max: 30000, defaultValue: 1000 },
  {
    name: "WorkMode",
    label: "工作模式",
    type: "enum",
    defaultValue: 0,
    options: Object.freeze([
      { value: 0, label: "主动模式" },
      { value: 1, label: "硬件触发模式" },
      { value: 2, label: "软件触发模式" },
    ]),
  },
  { name: "SoftwareTrigger", label: "软件触发", type: "bool", defaultValue: false },
  { name: "DepthCloudPoint", label: "深度点云", type: "bool", defaultValue: true },
  { name: "Depth2ColorCloudPoint", label: "彩色对齐点云", type: "bool", defaultValue: false },
]);

export const CAMERA_SDK_PARAMETER_MAP = new Map(
  CAMERA_SDK_PARAMETER_DEFINITIONS.map((definition) => [definition.name, definition]),
);

export function getDefaultCameraSdkSettings() {
  return CAMERA_SDK_PARAMETER_DEFINITIONS.reduce((settings, definition) => {
    settings[definition.name] = definition.defaultValue;
    return settings;
  }, {});
}

export function normalizeCameraSdkSettingValue(definition, value) {
  if (definition.type === "bool") {
    return typeof value === "boolean" ? value : Boolean(definition.defaultValue);
  }
  const numericValue = Number(value);
  const fallback = Number(definition.defaultValue);
  const roundedValue = Number.isFinite(numericValue) ? Math.round(numericValue) : fallback;
  if (definition.type === "enum") {
    return definition.options.some((option) => Number(option.value) === roundedValue)
      ? roundedValue
      : fallback;
  }
  const min = Number.isFinite(definition.min) ? definition.min : roundedValue;
  const max = Number.isFinite(definition.max) ? definition.max : roundedValue;
  return Math.min(max, Math.max(min, roundedValue));
}

export function normalizeCameraSdkSettings(value = {}) {
  return CAMERA_SDK_PARAMETER_DEFINITIONS.reduce((settings, definition) => {
    settings[definition.name] = normalizeCameraSdkSettingValue(definition, value?.[definition.name]);
    return settings;
  }, {});
}
