# 2026-04-29 PR-FPRG 红外最终 rho 微调归档

## 1. 状态

这份文档归档的是一次已经从运行链路中撤回的改动版本。

当前 active PR-FPRG 不启用红外最终 rho 微调；运行链路停在：

1. 方向候选稳定排序
2. 峰值支撑
3. 连续钢筋条验证
4. `spacing_pruned` 距离 / 间距兜底
5. 线族求交出点

本归档只作为后续改进方案，不代表当前运行代码。

## 2. 当时要解决的问题

用户反馈：在深度响应筛出的线族基础上，最终绿线和红外图里的可见钢筋中心仍有约 1-3 px 偏差。

当时的判断是：

- 深度响应适合做主筛线与连续性过滤。
- 红外图更贴近肉眼看到的钢筋中心。
- 可以保留深度主链，只在最后的 `rho` 上做小范围红外视觉微调。

## 3. 已撤回的改动清单

### 3.1 `workspace_s2.py`

曾新增以下逻辑：

- `refine_workspace_s2_oriented_line_rhos_to_auxiliary_ridges(...)`
- `_workspace_s2_subpixel_profile_peak_offset(...)`
- `build_workspace_s2_oriented_line_families(...)` 增加 `alignment_response_map=None`
- 在 `spacing_rhos = prune_workspace_s2_line_rhos_by_spacing(continuous_rhos)` 之后调用红外微调
- 保存 `family["pre_alignment_line_rhos"] = spacing_rhos`
- 将 `family["line_rhos"]` 设置为红外微调后的 `aligned_rhos`

当时核心流程如下：

```python
spacing_rhos = prune_workspace_s2_line_rhos_by_spacing(continuous_rhos)
aligned_rhos = refine_workspace_s2_oriented_line_rhos_to_auxiliary_ridges(
    response_map,
    alignment_response_map,
    workspace_mask,
    spacing_rhos,
    family["line_angle_deg"],
    normal=family["normal"],
    search_radius_px=max(2, min(8, int(round(period * 0.35)))),
    min_response_ratio=0.25,
    min_ridge_contrast_ratio=0.12,
    max_visual_offset_px=max(2.0, min(4.0, float(period) * 0.18)),
)
family["pre_alignment_line_rhos"] = spacing_rhos
family["line_rhos"] = aligned_rhos
```

红外微调函数后来从逐点中位偏移改成了「红外 rho 剖面局部峰值」优先：

```python
profile_data = build_workspace_s2_oriented_axis_profile(
    auxiliary_response_map,
    support_mask,
    line_angle_deg,
)
auxiliary_profile = normalize_workspace_s2_profile_for_support(profile_data.get("profile", []))
profile_rho_min = float(profile_data.get("rho_min", 0.0))

profile_position = float(line_rho) - profile_rho_min
center_position = int(round(profile_position))
window_start = max(0, center_position - profile_search_radius_px)
window_end = min(auxiliary_profile.size - 1, center_position + profile_search_radius_px)
peak_index = int(window_start + np.argmax(auxiliary_profile[window_start:window_end + 1]))
peak_rho = profile_rho_min + float(peak_index) + subpixel_offset
```

### 3.2 `manual_workspace_s2.py`

曾把红外响应作为最终对齐辅助输入传入 oriented line family 构建：

```python
line_families = self.build_workspace_s2_oriented_line_families(
    normalized_response,
    rectified_mask_uint8,
    min_period=10,
    max_period=30,
    alignment_response_map=infrared_response,
)
```

并在二次运行主链时使用：

```python
alignment_response_map=s2_inputs.get("infrared_response_crop")
```

### 3.3 `pr_fprg_peak_supported_probe.py`

曾在探针中生成红外暗线响应：

```python
infrared_background = cv2.GaussianBlur(rectified_ir, (0, 0), sigmaX=7.0, sigmaY=7.0)
infrared_dark_line_response = normalize_workspace_s2_response(
    infrared_background - rectified_ir,
    rectified_valid,
)
```

并传给候选选择：

```python
best_variant = select_best_response_variant(
    response_candidates,
    rectified_valid,
    alignment_response_map=infrared_dark_line_response,
)
```

探针摘要中也曾拆分：

- `spacing_pruned`
- `visual_aligned`

### 3.4 回归测试

曾增加两类测试：

- 红外辅助线整体偏移时，最终 `rho` 应贴到红外视觉 ridge。
- 红外图里同时有深度旧位置弱残影和真实视觉峰值时，应优先选真实视觉峰值。

测试名称曾为：

- `test_workspace_s2_refines_oriented_rhos_to_auxiliary_visual_ridges`
- `test_workspace_s2_auxiliary_alignment_prefers_visual_profile_peak_over_weak_residue`

## 4. 当时现场验证结果

红外最终 rho 微调版本曾在现场探针中得到：

- 输出目录：`.debug_frames/pr_fprg_peak_supported_probe_20260429_061217`
- response：`depth_background_minus_filled`
- 点数：12
- 角度：`[88.0, 178.0]`
- 微调前：
  - family 0：`[-263.0, -172.0, -84.0]`
  - family 1：`[-297.0, -206.0, -121.0, -14.0]`
- 微调后：
  - family 0：`[-259.41, -172.32, -83.74]`
  - family 1：`[-295.34, -206.23, -120.83, -13.88]`

诊断脚本显示红外剖面残差大多能压到约 1 px 内。

## 5. 撤回原因

用户现场判断这版「又有一些对不齐，精度还没之前的方案高」。

实际风险是：

- 红外可见峰值和真实绑扎几何中心不一定一致。
- 局部红外峰值可能被边缘、阴影、表面纹理或钢筋宽度响应带偏。
- 如果把红外峰值作为最终 `rho`，会改变深度主链已经稳定下来的几何相位。

因此当前 active 版本回到无红外最终微调：

- `line_rhos = spacing_rhos`
- 保留连续钢筋条验证
- 保留距离 / 间距兜底
- 不启用 `visual_aligned`

## 6. 后续改进方向

后续如果继续解决「钢筋实物摆放偏斜」问题，不应简单恢复本归档的最终红外贴线。

更合适的方向是：

1. 明确区分「相机 / 画面斜」和「钢筋实物在地上本身斜」。
2. 在线族模型层处理物理偏斜，而不是在最后逐条线做红外吸附。
3. 如果使用红外信息，优先作为候选评分或置信度辅助，而不是直接覆盖最终 `rho`。
4. 对每条钢筋线可以研究低自由度形变模型，例如同一线族内允许统一小角度、统一相位或缓慢漂移，但不能让每条线被局部纹理单独牵引。
5. 验证标准必须以现场最终点位和绑扎几何为准，不能只看红外图中绿线是否贴视觉中心。

## 7. 当前结论

这版改动作为后续方案归档。当前运行代码不应恢复：

- `alignment_response_map`
- `infrared_response_crop`
- `visual_aligned`
- 红外最终 `rho` 覆盖

除非后续重新定义验证标准，并能证明它在现场精度上优于当前 `spacing_pruned` 版本。
