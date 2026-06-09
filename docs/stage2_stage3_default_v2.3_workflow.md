# Stage2/Stage3 Default Workflow v2.3

本文档说明 v2.3 默认配置下 Stage2 和 Stage3 的规范流程、关键参数、数据流和验证方法。

## 目标

v2.3 的目标是把当前验证过的两阶段垂向处理设为系统默认：

1. Stage2 保留本组数据的姿态、水平位置、水平速度、IMU 垂向约束、vertical jump/bias 框架和 road-state bias reestimate。
2. Stage3 不再重新解释水平、姿态或速度，只继承 Stage2 的姿态、水平位置、水平速度和 bias。
3. Stage3 只做高程低频一致性修正，目标为：

```text
z_stage3_target = z_stage2 + lowfreq(z_shared - z_stage2)
```

这意味着 Stage3 与 Stage2 的差值应主要是低频绝对高程校正，不应包含会显著改变 IRI 的短波毛刺。

## 默认配置来源

正式运行默认配置：

```text
config/default_offline.cfg
```

代码兜底默认值：

```text
include/offline_lc_minimal/common/Config.h
```

v2.3 已将 Stage2/Stage3 的关键默认值在上述两个位置保持一致。命令行未显式加载 cfg 时，不会回退到旧 Stage3 策略。

## Stage2 默认策略

Stage2 默认启用：

```text
enable_stage2_velocity_optimization=true
enable_stage2_vehicle_nhc_constraint=true
stage2_attitude_hold_sigma_rad=1e-05
stage2_horizontal_position_hold_sigma_m=0.0001
stage2_horizontal_velocity_hold_sigma_mps=0.0001
stage2_vehicle_y_nhc_velocity_sigma_mps=0.05
stage2_vehicle_y_nhc_displacement_sigma_m=0.05
```

Stage2 的职责：

- 生成后续 Stage3 继承的姿态、水平位置、水平速度和 bias。
- 保留垂向 IMU / DVZ 约束。
- 保留 vertical jump/bias 框架，用于处理 IMU 垂向异常。
- 使用 road-noise state 触发 `ba_z` 局部重估，避免把路面高噪声段的局部偏置强行拉回全局零偏。

推荐的 Stage2-only 调用：

```bash
./build/offline_lc_runner \
  --config config/member_a.cfg \
  --output-dir runs/member_a_stage2 \
  --set enable_stage3_vertical_reference_optimization=false
```

输出中后续必须保留：

```text
runs/member_a_stage2/trajectory.csv
runs/member_a_stage2/body_z_bias_reestimate_segments.csv
```

## Shared Height Reference

两组或多组数据应先分别跑完 Stage2，然后生成共同距离域参考：

```bash
./build/offline_lc_shared_vertical_reference_builder \
  --manifest runs/shared_manifest.csv \
  --output-dir runs/shared_reference \
  --grid-spacing-m 1 \
  --sigma-m 0.015
```

共享参考输出：

```text
shared_vertical_reference.csv
shared_reference_line.csv
shared_vertical_reference_projection_diagnostics.csv
shared_vertical_reference_summary.txt
```

`z_shared(s)` 是组级低频绝对高程参考。它不是 raw RTK 高程散点，也不应该携带 RTK 高频噪声。

## Stage3 默认策略

Stage3 默认启用：

```text
enable_stage3_vertical_reference_optimization=true
stage3_vertical_reference_smoothing_method=spline_baseline
stage3_vertical_reference_lowpass_cutoff_hz=0.01
stage3_vertical_reference_spline_knot_spacing_m=1
stage3_vertical_reference_spline_smooth_lambda=10000
stage3_vertical_reference_spline_anchor_weight=100000
stage3_vertical_reference_spline_slope_weight=1000
stage3_vertical_reference_constraint_mode=gaussian
stage3_vertical_anchor_sigma_m=0.001
```

Stage3-only 入口会通过 `MakeStage3HeightOptimizationConfig()` 强制最终优化遵守以下策略：

```text
enable_stage3_stage2_vertical_increment_hold=true
stage3_stage2_vertical_increment_sigma_m=0.0002
stage3_stage2_vertical_increment_jump_sigma_m=0.0005
enable_stage3_stage2_jump_shape_hold=true
stage3_stage2_jump_shape_sigma_m=0.0005
enable_stage3_jump_velocity_smoothness_regularizer=false
enable_stage3_jump_height_highfreq_deadband=false
enable_stage3_jump_adaptive_context_envelope=false
```

Stage3 final 会关闭竞争项：

- raw GNSS 垂向因子；
- RTK drift / outage 垂向参考；
- Stage3 内部重新分段的 RTK outage batch；
- body-z NHC / vehicle NHC / 姿态和水平重优化；
- 旧 Stage3 jump velocity / high-frequency height regularizer。

这些关闭项的原因是 Stage3 的职责已经收窄为高程低频校正。姿态、水平位置和水平速度应继承 Stage2，不应在 Stage3 被重新解释。

推荐的 Stage3-only 调用：

```bash
./build/offline_lc_stage3_runner \
  --config runs/member_a_stage2/config_snapshot.cfg \
  --stage2-trajectory runs/member_a_stage2/trajectory.csv \
  --shared-reference runs/shared_reference/shared_vertical_reference.csv \
  --shared-reference-line runs/shared_reference/shared_reference_line.csv \
  --output-dir runs/member_a_stage3
```

## 为什么 Stage3 要约束 `Stage3 - Stage2`

当前验证发现：

- `z_shared - z_stage2` 平滑参考本身是低频的；
- 旧 Stage3 final 允许在 envelope / center-pull 的毫米级自由度内产生高频残差；
- 这些残差虽然幅值不大，但其 IRI 可以达到 `1 mm/m` 量级，会破坏 Stage2 两组数据原本较高的短波相关性。

v2.3 采用两层保护：

1. 参考目标只使用 `z_stage2 + lowfreq(z_shared - z_stage2)`。
2. 优化中加入 Stage2 相邻垂向增量继承和 jump shape 继承。

因此 Stage3 仍能校正绝对高程，但 `Stage3 - Stage2` 不应再携带大量短波毛刺。

## 诊断文件

Stage3 输出中应重点检查：

```text
trajectory.csv
stage3_vertical_reference_diagnostics.csv
stage3_stage2_vertical_increment_hold_diagnostics.csv
stage3_stage2_jump_shape_hold_diagnostics.csv
vertical_velocity_delta_diagnostics.csv
vertical_jump_bias_diagnostics.csv
body_z_bias_reestimate_segments.csv
summary.txt
```

关键 summary 字段：

```text
stage3_vertical_reference_mean_abs_residual_m
stage3_vertical_reference_max_abs_residual_m
stage3_stage2_vertical_increment_mean_abs_residual_m
stage3_stage2_vertical_increment_max_abs_residual_m
stage3_stage2_jump_shape_mean_abs_residual_m
stage3_stage2_jump_shape_max_abs_residual_m
```

## IRI 验证口径

验证 `Stage3 - Stage2` 是否低频时，不应只比较 `IRI(Stage3) - IRI(Stage2)`，而应直接计算：

```text
IRI(z_stage2 - z_stage3)
```

推荐口径：

- 使用 Stage2 水平距离轴；
- Stage3 高程按时间插值到 Stage2 时间/距离轴；
- 去掉首尾水平速度低于 `0.5 m/s` 的静止段；
- 按 `0.25 m` 距离间隔重采样；
- 使用 Sroubek/Sorel IRI 实现，非重叠 `20 m` 和 `50 m` 窗口。

本次验证中，v2.3 默认策略得到：

| 数据 | 20 m `IRI(stage2-stage3)` | 50 m `IRI(stage2-stage3)` |
| --- | ---: | ---: |
| 132613 | 0.2123 mm/m | 0.2026 mm/m |
| 181436 | 0.2025 mm/m | 0.1967 mm/m |

旧策略约为：

| 数据 | 20 m `IRI(stage2-stage3)` | 50 m `IRI(stage2-stage3)` |
| --- | ---: | ---: |
| 132613 | 1.8756 mm/m | 1.9397 mm/m |
| 181436 | 1.2108 mm/m | 1.2533 mm/m |

## Release Validation Commands

推荐发布前验证：

```bash
cmake --build build -j4
export LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib:$LD_LIBRARY_PATH
ctest --test-dir build --output-on-failure
```

对 rtk_err_11 数据的验证输出位于：

```text
runs/road_noise_state_verify_20260609/132613_stage3_lowfreq_delta_policy_default
runs/road_noise_state_verify_20260609/181436_stage3_lowfreq_delta_policy_default
runs/road_noise_state_verify_20260609/stage2_stage3_iri_lowfreq_delta_policy_default
runs/road_noise_state_verify_20260609/stage2_stage3_height_lowfreq_delta_policy_default
```
