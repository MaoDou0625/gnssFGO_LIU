# offline_lc_minimal 的 IMU+GNSS 精度增强说明

## 1. 这次增强解决了什么

这一轮只围绕离线 `IMU + GNSS solution` 精度做增强，重点补上了两件事：

1. `GNSS robust loss`
2. 更细的时间对齐 / 插值机制

同时保持当前“纯 C++、不依赖 ROS、只保留 GNSS+IMU 必要模块”的工程边界不变。

后续又补上了一项初始化增强：

3. 可选的“IMU 优先”静止双矢量对准，用高等级 IMU 的长静止窗优先估计初始姿态、航向和零偏，失败再 fallback。

## 2. 这次尽量复用了原始代码的哪些思路

这次不是重新发明一套新逻辑，而是把原始 `gnssFGO` 里对离线 `LC` 精度真正有帮助的几层思路迁回到了最小版：

- `assignNoiseModel` 风格的 GNSS noise model 分派
- `findStateForMeasurement` 风格的 GNSS 测量与状态同步判定
- `GPInterpolatedGPSFactor` 风格的区间内 GNSS 插值因子
- 用 `omega` 作为 GP 插值支撑态

没有迁回来的部分主要是：

- ROS / plugin / dataset adapter 体系
- GNSS velocity / attitude 因子
- 完整 GP motion prior 和 `acc` 支撑态

也就是说，当前实现是“尽量复用原始算法层，避免把原始框架复杂度一并搬回来”。

## 3. 当前离线 GNSS+IMU 解算主链路

当前主链路在 [src/core/OfflineBatchRunner.cpp](D:/Code/offline_lc_minimal/src/core/OfflineBatchRunner.cpp)：

1. 读取 `imu_gnss_fgo.txt` 和 `gnss_solution_gnss_fgo.txt`
2. 先做 GNSS 质量过滤和 IMU 覆盖检查，再选初始化样本
3. 用首个合格 GNSS 点建立局部 ENU 原点
4. 用起始 IMU 窗口做重力对准，得到初始 `roll/pitch`
5. 如果开启 `prefer_imu_initial_yaw`，先尝试用静止 IMU 的双矢量对准估计初始 `yaw` 和零偏
6. 如果 IMU 双矢量不可用，再用早期合格 GNSS 位移估计初始 `yaw`
7. 按 `state_frequency_hz` 构建固定频率状态网格
8. 相邻状态之间继续用 `CombinedImuFactor`
9. 额外保留 `omega` 状态，并用 [AngularRateFactor.h](D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/AngularRateFactor.h) 约束 `omega`
10. GNSS 测量先做时间修正，再做同步状态判定
11. 对同步测量加普通 `GPSFactor`
12. 对落在状态区间内的测量加 [GPInterpolatedGPSFactor.h](D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/GPInterpolatedGPSFactor.h)
13. 对超出状态/IMU 支撑范围的测量直接丢弃，并记入统计
14. GNSS 噪声由 `sigma_lat/lon/h`、fix-type 缩放和 robust loss 共同决定
15. 最后做整段 batch 优化并落盘结果

## 4. GNSS robust loss 现在是怎么做的

当前配置和解析在：

- [config/default_offline.cfg](D:/Code/offline_lc_minimal/config/default_offline.cfg)
- [Config.h](D:/Code/offline_lc_minimal/include/offline_lc_minimal/common/Config.h)
- [Config.cpp](D:/Code/offline_lc_minimal/src/common/Config.cpp)

新增配置项：

- `gnss_position_noise_model`
- `gnss_position_robust_param`

当前支持：

- `gaussian`
- `cauchy`
- `huber`
- `dcs`
- `tukey`
- `geman_mcclure`
- `welsch`

默认值是：

- `gnss_position_noise_model = cauchy`
- `gnss_position_robust_param = 0.5`

实现上，`sigma_lat/lon/h` 仍然是基础协方差来源；`RTKFIX / RTKFLOAT / SINGLE` 的分级权重仍然保留；`NO_SOLUTION` 默认丢弃，不会再以异常协方差的形式混入图中。

## 5. 更细的时间对齐 / 插值现在是怎么做的

新增配置项：

- `state_frequency_hz`
- `enable_gp_interpolated_gnss`
- `gnss_time_offset_s`
- `state_meas_sync_lower_bound_s`
- `state_meas_sync_upper_bound_s`

当前同步判定分成 5 类：

- `SYNCHRONIZED_I`
- `SYNCHRONIZED_J`
- `INTERPOLATED`
- `DROPPED`
- `CACHED`

当前策略是：

- 先看测量时间是否落在 IMU 支撑范围内
- 再看是否与状态 `I/J` 同步
- 若不严格同步但落在区间内部，则走 GP 插值
- 若超出当前状态或 IMU 可支撑范围，则拒绝并记统计

这里的关键点是：不再把 GNSS 测量粗暴挂到最近状态，也不做静默外推。

## 6. IMU 姿态现在是否被保留

是，而且保留了两层：

1. 主姿态仍保存在 `Pose3.rotation()` 中
2. 额外新增了 `omega` 状态，用来支撑 GNSS GP 插值

所以这轮并不是“重新加一套姿态状态”，而是在保留原有 IMU 姿态状态的基础上，补了一个更适合时间插值的角速度支撑态。

当前没有新增绝对姿态因子，原因是文本输入里没有可靠的 GNSS heading / attitude 真值。姿态仍主要由 IMU 预积分与 `CombinedImuFactor` 传播和约束。

补充一点：当前已经支持“高等级 IMU 静止窗双矢量对准”作为初始化选项。它不是新的图因子，而是初始化阶段的先验估计增强。

## 7. 新增了哪些输出，方便验收

当前结果目录除了原来的 `summary.txt` 和 `trajectory.csv`，还新增或增强了这些内容：

- `trajectory.csv`
  - 新增 `omega_x/y/z_radps`
- `gnss_residuals.csv`
  - 继续保留按状态的 GNSS 摘要，兼容旧的读取方式
- `gnss_alignment.csv`
  - 新增逐测量诊断，明确记录每个 GNSS 测量是同步、插值还是丢弃
  - 仅在 `write_debug_csv = true` 时输出
- `summary.txt`
  - 新增 `gnss_synced_factor_count`
  - 新增 `gnss_interpolated_factor_count`
  - 新增 `gnss_dropped_count`
  - 新增 `gnss_cached_count`
  - 新增 `dropped_out_of_imu_coverage_count`

这些输出主要是为了让时间对齐和精度问题可以直接定位，而不是只能看最终轨迹。

## 8. 这轮已经验证过的结果

### 8.1 默认 Cauchy 配置

运行目录：

- [runs/cauchy_gp_smoke_postpatch](D:/Code/offline_lc_minimal/runs/cauchy_gp_smoke_postpatch)

结果：

- `state_count = 5825`
- `gnss_factor_count = 1377`
- `gnss_synced_factor_count = 1377`
- `gnss_interpolated_factor_count = 0`
- `gnss_dropped_count = 80`
- `final_error = 0.372051`

这里没有出现 `INTERPOLATED`，是因为默认 `20 Hz` 状态频率和当前 GNSS 时间关系比较整齐，绝大多数测量自然落在同步窗口内。这是正常现象，不代表插值路径没实现。

### 8.2 强制触发插值路径

把 `state_frequency_hz` 改成 `17.0` 后，运行目录：

- [runs/interp_gp_postpatch](D:/Code/offline_lc_minimal/runs/interp_gp_postpatch)

结果：

- `state_count = 4952`
- `gnss_factor_count = 1377`
- `gnss_synced_factor_count = 985`
- `gnss_interpolated_factor_count = 392`
- `gnss_dropped_count = 80`
- `final_error = 0.60106`

这说明同步分支和插值分支都已经实际跑通，而且 `gnss_alignment.csv` 里可以直接看到 `INTERPOLATED` 记录。

### 8.3 Gaussian 对照

运行目录：

- [runs/gaussian_gp_postpatch](D:/Code/offline_lc_minimal/runs/gaussian_gp_postpatch)

结果：

- `initial_error = 7.38025e+07`
- `final_error = 0.373752`

对照默认 Cauchy：

- `initial_error = 1742.04`
- `final_error = 0.372051`

从当前数据看，默认 `cauchy + 0.5` 在初始阶段明显更稳，最终误差也略好。

### 8.4 IMU 优先初始化对照

打开 `prefer_imu_initial_yaw = true` 后，运行目录：

- [runs/imu_priority_init](D:/Code/offline_lc_minimal/runs/imu_priority_init)

结果：

- `yaw_source = imu_dual_vector`
- `initial_error = 1181.58`
- `final_error = 0.367681`

对照默认 displacement 初始化：

- `yaw_source = displacement`
- `initial_error = 1740.28`
- `final_error = 0.372051`

在当前这条数据上，IMU 优先初始化确实生效了，而且初始误差和最终误差都更好。

## 9. 和这轮改造前相比，精度上最重要的变化

只看 `GNSS + IMU`，这轮真正对精度最有意义的变化是：

- GNSS 已经支持 robust loss，不再只能用纯 Gaussian
- GNSS 已经支持更细时间对齐和区间内 GP 插值
- 初始化已经支持高等级 IMU 长静止窗的双矢量对准，可优先估计初始航向和零偏
- 初始化已经先做 GNSS 质量过滤和 IMU 覆盖检查
- IMU 支撑范围外的 GNSS 不会再被静默外推
- GNSS 测量和因子之间现在有显式 bookkeeping，诊断更可靠

## 10. 还没有做的部分

当前还没有做：

- GNSS velocity factor
- GNSS attitude factor
- 完整 GP motion prior
- TC / per-satellite 观测级融合

所以当前更准确的定位是：

- 它已经是一个比初版 minimal baseline 更强、也更稳的离线 `IMU + GNSS` 精度基线
- 但它还不是原始 `gnssFGO` 全功能 LC/TC 体系的完整复刻
