export const PROCESS_IMAGE_REQUEST_MODES = Object.freeze({
  DEFAULT: 0,
  ADAPTIVE_HEIGHT: 1,
  BIND_CHECK: 2,
  SCAN_ONLY: 3,
  EXECUTION_REFINE: 4,
});

export const GLOBAL_EXECUTION_MODES = Object.freeze({
  SLAM_PRECOMPUTED: 0,
  LEDGER_WITH_REFINE: 1,
  PLANNED_PATH_REFINE_ONLY: 2,
});

export const DEFAULT_GLOBAL_EXECUTION_MODE = GLOBAL_EXECUTION_MODES.LEDGER_WITH_REFINE;

export const GLOBAL_EXECUTION_MODE_OPTIONS = Object.freeze([
  {
    id: GLOBAL_EXECUTION_MODES.SLAM_PRECOMPUTED,
    label: "执行账本",
  },
  {
    id: GLOBAL_EXECUTION_MODES.LEDGER_WITH_REFINE,
    label: "账本+微调",
  },
  {
    id: GLOBAL_EXECUTION_MODES.PLANNED_PATH_REFINE_ONLY,
    label: "规划路径+纯微调",
  },
]);

export const FRONTEND_VISUAL_RECOGNITION_REQUEST_MODE = PROCESS_IMAGE_REQUEST_MODES.SCAN_ONLY;
export const FRONTEND_VISUAL_RECOGNITION_MODE_LABEL = "Surface-DP物理先验";
export const FRONTEND_VISUAL_RECOGNITION_FULL_LABEL = "Surface-DP物理先验扫描输出";

export const SCAN_RESPONSE_SOURCE_OPTIONS = Object.freeze([
  { id: "fused_instance_response", label: "融合实例响应" },
  { id: "frangi_like", label: "Frangi-like 脊线" },
  { id: "hessian_ridge", label: "Hessian ridge 脊线" },
  { id: "depth_gradient", label: "深度梯度边缘" },
  { id: "infrared_response", label: "红外响应" },
  { id: "combined_response", label: "组合响应" },
  { id: "depth_response", label: "深度响应" },
]);

export const DEFAULT_SCAN_RESPONSE_SOURCE = "depth_gradient";

export function normalizeScanResponseSource(value, fallback = DEFAULT_SCAN_RESPONSE_SOURCE) {
  return SCAN_RESPONSE_SOURCE_OPTIONS.some((option) => option.id === value)
    ? value
    : fallback;
}
