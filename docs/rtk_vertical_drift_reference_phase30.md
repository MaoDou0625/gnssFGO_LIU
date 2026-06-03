# Phase30 RTK 垂向漂移参数化与中心参考去漂实现说明

本文档整理 phase30 中 `drift_corrected_rtk` 的估计和接入方案。当前实现的目标不是重写 RTK 约束，也不是把 RTK 变成图内可动潜变量，而是在主图外估计一个固定的 RTK 垂向漂移 profile，并仅用于修正垂向 `center-pull` 的参考中心。`raw RTK envelope gate` 继续使用原始 RTK 高程。

## 目标

当前 IRI 偏大的重要怀疑来源是 RTK 垂向观测中存在中高频波动和低频漂移。如果直接把 raw RTK 作为 gate 内 center-pull 的中心，RTK 的慢漂和部分波动会持续牵引 `pose.z`，进而影响导航高程和 IRI。

phase30 的目标是：

- 保留 raw RTK 的 envelope gate 作为安全边界。
- 不新增 RTK 潜变量，不把 `pose.z` 和可动 RTK reference 直接耦合到同一个主图。
- 根据上一轮优化轨迹和静态 100 s 常高先验，估计 RTK 垂向漂移分量。
- 用 `raw_rtk_up - drift_estimate` 作为 gate 内 `center-pull` 的中心。
- 保持 IMU、NHC、PV consistency、adaptive dvz、jump-bias 等现有约束结构不变。

## 参数来源

phase30 默认参数来自静态 100 s RTK Allan 方差分析结果：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `rtk_vertical_drift_correlation_time_s` | `5.3` | 一阶 GM/OU 漂移相关时间 |
| `rtk_vertical_drift_sigma_m` | `0.010` | 相关漂移稳态标准差，约 10 mm |
| `rtk_vertical_white_noise_sigma_m` | `0.002` | 白噪声标准差，约 2 mm |
| `rtk_vertical_drift_huber_sigma_m` | `0.030` | 漂移估计观测更新的 Huber 截断尺度 |
| `rtk_vertical_drift_max_abs_correction_m` | `0.050` | 单点最大漂移修正幅度 |
| `rtk_vertical_drift_convergence_threshold_m` | `0.001` | 外循环 drift profile 收敛阈值 |

phase30 配置入口：

```ini
enable_rtk_vertical_drift_reference = true
rtk_vertical_drift_correlation_time_s = 5.3
rtk_vertical_drift_sigma_m = 0.010
rtk_vertical_white_noise_sigma_m = 0.002
rtk_vertical_drift_huber_sigma_m = 0.030
rtk_vertical_drift_max_abs_correction_m = 0.050
rtk_vertical_drift_convergence_threshold_m = 0.001
rtk_vertical_drift_outer_iterations = 20
rtk_vertical_drift_use_for_center_pull = true
rtk_vertical_drift_use_for_envelope_gate = false
```

对应实验配置：

```text
config/transformed1cut1_vertical_envelope_phase30_rtk_drift_reference.cfg
```

## 代码结构

主要实现文件：

| 文件 | 作用 |
| --- | --- |
| `include/offline_lc_minimal/core/RtkVerticalDriftReferenceEstimator.h` | drift estimator 的请求、结果和接口定义 |
| `src/core/RtkVerticalDriftReferenceEstimator.cpp` | RTK drift profile 估计、OU 滤波、RTS 平滑和 summary 统计 |
| `src/core/OfflineBatchRunner.cpp` | 外循环中调用 estimator，并把 profile 传给 GNSS builder |
| `include/offline_lc_minimal/core/GnssFactorBuilder.h` | 给 GNSS builder 增加可选 drift profile 输入 |
| `src/core/GnssFactorBuilder.cpp` | 把 drift profile 转交给 vertical constraint policy |
| `include/offline_lc_minimal/core/VerticalConstraintPolicy.h` | policy context 增加 drift profile 指针 |
| `src/core/VerticalConstraintPolicy.cpp` | 选择 raw RTK 或 drift-corrected RTK center reference |
| `src/core/RunDiagnosticsBuilder.cpp` | post-fit 诊断中 center-pull residual 使用实际 center reference |
| `src/common/ResultOutputWriters.cpp` | 输出 drift diagnostics 和扩展 envelope diagnostics |
| `scripts/plot_alignment_navigation_continuity.py` | 绘制 raw RTK / drift-corrected RTK / drift estimate / white residual |

## 数学模型

对每个有效 RTK/GNSS 垂向样本，构造观测残差：

```text
residual_i = raw_rtk_up_i - nav_reference_up_i
```

其中 `nav_reference_up_i` 的来源分两类：

- 静态 100 s 内：使用静态 RTK 高程锚点得到的“真实高程常值”参考。
- 动态段：使用上一轮主图优化结果中的导航高程，按 GNSS 时间同步或 GP 插值到测量时刻。

残差建模为：

```text
raw_rtk_up - nav_up = constant_bias + drift_gm + white_noise
```

这里三部分含义不同：

- `constant_bias`：RTK 与导航参考之间的常值基准差。当前实现只估计它，用于漂移分离，不从 center 参考里扣除。
- `drift_gm`：一阶 GM/OU 漂移，是 phase30 真正用于修正 RTK center 的量。
- `white_noise`：剩余短时白噪声，用于诊断 RTK 垂向高频水平。

`drift_gm` 使用一阶 OU 模型：

```text
d_k = phi_k * d_{k-1} + q_k
phi_k = exp(-dt / tau_c)
Q_k = sigma_drift^2 * (1 - phi_k^2)
z_k = residual_k - constant_bias = d_k + white_noise_k
```

测量更新中对 innovation 做 Huber 风格截断：

```text
innovation = clamp(z_k - d_pred, -huber_sigma, +huber_sigma)
```

然后执行前向 Kalman filter 和后向 RTS smoothing，得到每个 RTK 样本的平滑漂移估计 `drift_estimate_m`。

最后进行幅度保护：

```text
drift_estimate_m = clamp(drift_estimate_m,
                         -rtk_vertical_drift_max_abs_correction_m,
                         +rtk_vertical_drift_max_abs_correction_m)
```

并生成中心参考：

```text
corrected_center_up_m = raw_rtk_up_m - drift_estimate_m
```

注意：当前没有扣除 `constant_bias_m`。原因是常值偏差对应高程基准，不应该被漂移模型自动删除。漂移模型只负责去除随时间变化的低频牵引。

## 外循环流程

phase30 与 phase27 的 adaptive motion reweighting 共用外循环。RTK drift reference 的额外迭代次数由 `rtk_vertical_drift_outer_iterations` 控制；phase30 当前设置为 `20`，默认配置仍保留 `2` 以兼容早期实验。当前逻辑为：

1. `pass0`
   - 不存在 drift profile。
   - GNSS 垂向 center-pull 使用 raw RTK。
   - 主图按 phase27 原始方案优化。

2. `pass1`
   - 使用 `pass0` 的优化结果估计 RTK drift profile。
   - 重建主图。
   - envelope gate 仍使用 raw RTK。
   - center-pull 使用 `corrected_center_up_m`。

3. `pass2`
   - 使用 `pass1` 的优化结果重新估计 drift profile。
   - 如果 profile 相比上一轮最大变化小于 `rtk_vertical_drift_convergence_threshold_m`，认为已收敛。
   - 否则继续用新的 fixed profile 重建图。

当前 phase30 默认最多做两次 drift profile 更新。若 adaptive motion reweighting 也开启，则总 pass 数取两类外循环需求的较大者。

## 与 GNSS 垂向约束的关系

phase30 保持 RTK 垂向 envelope gate 的安全边界不变：

```text
VerticalEnvelopeFactor:
  residual = VerticalEnvelopeResidual(pose.z - raw_rtk_up, half_width)
```

也就是说：

- gate 中心仍然是 raw RTK。
- gate half width 仍由原配置决定。
- gate 外 violation 仍然按 raw RTK 判断。
- `rtk_vertical_drift_use_for_envelope_gate = true` 当前会被配置校验拒绝。

只有 center-pull 的参考中心会被替换：

```text
VerticalEnvelopeCenterPullFactor:
  residual = VerticalEnvelopeCenterResidual(pose.z - center_reference_up, half_width, deadband)

center_reference_up =
  corrected_center_up_m, if drift profile valid
  raw_rtk_up_m, otherwise
```

这使得 RTK 的 gate 仍能防止导航长期跑出安全范围，但 gate 内的持续牵引不再直接跟随 raw RTK 慢漂。

## 静态 100 s 的作用

静态 100 s 不是被单独拿出来后处理，而是参与 drift 分离的参考构造：

- 静态窗口内真实高度应为常值。
- estimator 中静态样本的 `nav_reference_up_m` 使用 `initial_static_rtk_height_reference_up_m`。
- 静态样本用于估计 `constant_bias_m` 的中位数基准。
- 静态段的残差时间变化主要被解释为 `drift_gm + white_noise`。

这样做的作用是：

- 把 RTK 的常值基准差和时间相关漂移分离。
- 避免把静态段中缓慢变化的 RTK 高程当成真实路面高程。
- 为动态段 drift profile 提供更稳定的初始统计尺度。

## 动态段的作用

动态段不能假设真实高程常值，因此 estimator 使用上一轮优化出的导航高程作为短时连续参考：

- 若 GNSS 样本与状态同步，直接读取对应 `X_i.translation().z()`。
- 若 GNSS 样本在相邻状态之间，使用已有 `GPWNOJInterpolator` 插值导航 pose。
- 若状态不存在、插值 key 缺失或超出 IMU 覆盖，样本标记 invalid，不参与 drift 估计。

这种设计让 drift profile 不依赖 RTK 自己的高频趋势，而是依赖上一轮已经受 IMU、dvz、PV、NHC、jump-bias 共同约束的导航轨迹。

## 诊断输出

### `rtk_vertical_drift_reference_diagnostics.csv`

每个 GNSS 样本输出一行：

| 字段 | 含义 |
| --- | --- |
| `sample_index` | GNSS 样本序号 |
| `time_s` | corrected GNSS 时间 |
| `raw_rtk_up_m` | 原始 RTK ENU up |
| `nav_reference_up_m` | drift 估计所用导航/静态参考高程 |
| `residual_m` | `raw_rtk_up_m - nav_reference_up_m` |
| `constant_bias_m` | residual 的常值基准 |
| `drift_estimate_m` | OU 平滑后的 RTK 漂移估计 |
| `corrected_center_up_m` | `raw_rtk_up_m - drift_estimate_m` |
| `white_residual_m` | `residual_m - constant_bias_m - drift_estimate_m` |
| `drift_sigma_m` | drift GM sigma |
| `white_sigma_m` | white noise sigma |
| `tau_s` | drift correlation time |
| `static_window_flag` | 是否属于静态 100 s |
| `valid` | 是否参与 drift profile |
| `skip_reason` | 无效原因或 `OK` |

### `vertical_envelope_diagnostics.csv`

新增字段：

| 字段 | 含义 |
| --- | --- |
| `center_pull_reference_type` | `raw_rtk` 或 `rtk_drift_corrected` |
| `center_pull_reference_up_m` | center-pull 实际使用的参考高程 |
| `rtk_drift_estimate_m` | 对应 RTK 样本的 drift estimate |

其中：

- `raw_residual_m` 始终是 `predicted_up_m - raw_rtk_up_m`，用于 raw gate。
- `center_pull_residual_m` 使用 `predicted_up_m - center_pull_reference_up_m`，用于 center-pull 诊断。

### `summary.txt`

新增 summary 字段：

| 字段 | 含义 |
| --- | --- |
| `rtk_vertical_drift_reference_enabled` | 是否启用 drift reference |
| `rtk_vertical_drift_reference_pass_count` | 外循环中 drift reference pass 数 |
| `rtk_vertical_drift_reference_valid_count` | 有效 drift 样本数 |
| `rtk_vertical_drift_static_range_m` | 静态段 drift range |
| `rtk_vertical_drift_static_std_m` | 静态段 drift std |
| `rtk_vertical_drift_white_residual_std_m` | white residual std |
| `rtk_vertical_drift_max_abs_correction_m` | 最大绝对 drift 修正 |
| `rtk_vertical_drift_first20_mean_correction_m` | 动态前 20 s mean correction |
| `rtk_vertical_drift_first20_max_abs_correction_m` | 动态前 20 s max abs correction |

## 当前 phase30 smoke 结果

运行命令：

```bash
./run_offline.sh \
  --config config/transformed1cut1_vertical_envelope_phase30_rtk_drift_reference.cfg \
  --output-dir build/rtk_gate_eval/phase30_rtk_drift_reference
```

关键 summary：

| 指标 | 数值 |
| --- | ---: |
| `rtk_vertical_drift_reference_valid_count` | `1377` |
| `rtk_vertical_drift_static_range_m` | `0.0423738` |
| `rtk_vertical_drift_static_std_m` | `0.00850783` |
| `rtk_vertical_drift_white_residual_std_m` | `0.00211643` |
| `rtk_vertical_drift_max_abs_correction_m` | `0.0388871` |
| `rtk_vertical_drift_first20_mean_correction_m` | `0.0100009` |
| `rtk_vertical_drift_first20_max_abs_correction_m` | `0.0188962` |

这些结果与 Allan 参数基本一致：

- white residual 约 `2.1 mm`，接近 white sigma `2 mm`。
- 静态 drift std 约 `8.5 mm`，接近 drift sigma `10 mm`。
- 最大 correction 小于 `5 cm` 裁剪阈值。

## 当前 IRI 对比

phase30 的 50 m IRI 计算结果如下，单位为 `mm/m`，等价于 `m/km`：

| source | segment count | mean IRI | median IRI | max IRI |
| --- | ---: | ---: | ---: | ---: |
| `optimized_nav` | `3` | `5.120689544` | `5.195261436` | `5.302900638` |
| `raw_rtk` | `3` | `17.365089481` | `15.618313270` | `22.494216900` |
| `drift_corrected_rtk` | `3` | `5.894089343` | `5.915899601` | `6.048313605` |
| `optimized_nav_on_rtk_station` | `3` | `4.910472894` | `4.992968510` | `5.152478377` |

输出路径：

```text
build/rtk_gate_eval/phase30_rtk_drift_reference/iri_50m/
```

该结果说明：

- raw RTK 的 IRI 明显偏大。
- drift-corrected RTK 的 IRI 降到与 optimized nav 接近的量级。
- 当前 phase30 的 drift reference 有效降低了 RTK 垂向波动对 IRI 参考的影响。

## 与 phase28 / phase29 的区别

phase28 是图外固定低通参考：

- 先对 RTK 低通，再把低通结果作为 center-pull center。
- 低通结果不依赖主图优化轨迹。
- 对 RTK 漂移和白噪声没有显式参数化。

phase29 是图内潜变量参考：

- 为 RTK 垂向参考新增图内 scalar key。
- 导航高程和可动 RTK reference 在同一主图中耦合。
- 实际测试暴露了框架耦合问题，导航速度和高程约束效果异常。

phase30 当前方案：

- 不新增主图 RTK latent key。
- drift profile 固定在主图外估计。
- raw RTK gate 不变。
- 只修正 center-pull reference。
- 使用明确的 `tau / drift_sigma / white_sigma` 参数来描述 RTK 垂向误差结构。

## 重要实现边界

1. `drift_corrected_rtk` 不是新的导航状态，也不是图内变量。
2. `drift_corrected_rtk` 只影响 gate 内 center-pull，默认不影响 envelope gate。
3. `constant_bias_m` 只用于漂移分离，不从 RTK center 中扣除。
4. 动态段 drift 估计依赖上一轮优化轨迹，因此 phase30 至少需要 pass0 之后才能生成 corrected reference。
5. 如果某个 RTK 样本 drift profile 无效，该样本 center-pull 自动回退 raw RTK。
6. 如果 `rtk_vertical_drift_use_for_envelope_gate = true`，当前配置校验会报错，防止误把 raw gate 改成漂移修正 gate。

## 后续可验证项

建议继续关注以下诊断：

- `rtk_vertical_drift_reference_diagnostics.csv` 中 `white_residual_m` 的 std 是否稳定在 `2 mm` 量级。
- `drift_estimate_m` 是否出现贴近 `max_abs_correction` 的长时间饱和。
- `vertical_envelope_diagnostics.csv` 中 `raw_residual_m` 与 `center_pull_residual_m` 的差异。
- `optimized_nav` 与 `drift_corrected_rtk` 的 IRI 差异是否随参数变化保持合理。
- 动态前 20 s 的 `rtk_vertical_drift_first20_*` 是否与高程慢漂方向一致。
