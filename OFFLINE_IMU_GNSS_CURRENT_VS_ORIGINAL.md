# 当前 offline_lc_minimal 的离线 IMU+GNSS 解算与原始 gnssFGO 的差异

## 1. 当前框架如何做离线 IMU+GNSS

当前 [offline_lc_minimal](D:/Code/offline_lc_minimal) 已经不是最初那版“每个 GNSS 点一个状态”的纯简化实现了，现在的主链路是：

1. 从 `imu_gnss_fgo.txt` 和 `gnss_solution_gnss_fgo.txt` 读取 IMU 和 GNSS solution。
2. 先按 GNSS 质量和 IMU 时间覆盖范围筛出可用初始化点。
3. 用首个合格 GNSS 点建立局部 ENU 原点。
4. 用起始 IMU 窗口做重力对准，得到初始 `roll/pitch`。
5. 如果开启 `prefer_imu_initial_yaw`，优先用静止 IMU 双矢量对准估计初始姿态、航向和零偏。
6. 如果 IMU 双矢量不可用，再用早期合格 GNSS 位移估计初始 `yaw`。
7. 按 `state_frequency_hz` 构建固定状态时间网格，而不是直接按 GNSS 建状态。
8. 相邻状态之间用 `CombinedImuFactor` 做 IMU 预积分约束。
9. 额外保留 `omega` 状态，并用角速度因子把 `omega` 和 IMU gyro 测量连起来。
10. GNSS 进入图之前先做时间修正和状态同步判定：
   - 同步到状态 `I/J` 时，加普通 GNSS 位置因子
   - 落在状态区间内时，加 GP-interpolated GNSS 位置因子
   - 超出状态或 IMU 支撑范围时，直接 drop/cache，不做静默外推
11. GNSS 噪声先由 `sigma_lat/lon/h` 映射，再叠加 fix-type 缩放和 robust loss。
12. 最后用 batch `LevenbergMarquardtOptimizer` 做整段离线优化。

当前核心代码文件是：

- [src/core/OfflineBatchRunner.cpp](D:/Code/offline_lc_minimal/src/core/OfflineBatchRunner.cpp)
- [src/core/TrajectoryInitializer.cpp](D:/Code/offline_lc_minimal/src/core/TrajectoryInitializer.cpp)
- [src/io/TextDatasetLoader.cpp](D:/Code/offline_lc_minimal/src/io/TextDatasetLoader.cpp)
- [include/offline_lc_minimal/factor/GPInterpolatedGPSFactor.h](D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/GPInterpolatedGPSFactor.h)
- [include/offline_lc_minimal/gp/GPWNOJInterpolator.h](D:/Code/offline_lc_minimal/include/offline_lc_minimal/gp/GPWNOJInterpolator.h)

## 2. 当前版本相对原始 gnssFGO 的主要变化

只看 `GNSS + IMU`，当前版本和原始 `online_fgo` / `gnssFGO` 的关键区别是：

### 2.1 为了保精度保留下来的部分

- 继续使用 `CombinedImuFactor` 作为 IMU 主约束。
- 继续把 IMU 姿态保存在 `Pose3.rotation()` 中，而不是额外造一套姿态状态。
- 把原始 LC 路径里的这几层思路迁回来了：
  - robust GNSS noise model
  - `findStateForMeasurement` 式同步判定
  - `GPInterpolatedGPSFactor`
  - `omega` 支撑态

### 2.2 为了简化工程而没有迁回来的部分

- 没有迁 ROS、pluginlib、dataset plugin、rosbag2。
- 没有迁 GNSS 速度因子和 GNSS 姿态因子。
- 没有迁完整 GP motion prior，也没有引入 `acc` 支撑态。
- 没有把 offline 和 online 再揉成同一套大框架。

这意味着当前版本是“尽量复用原始算法层，但不把原框架复杂度一起搬回来”。

## 3. 这些差异对精度意味着什么

### 3.1 当前版本比最初的 minimal baseline 更强的地方

- GNSS 已经支持 robust loss，默认是 `cauchy + 0.5`，对异常点更稳。
- GNSS 已经支持更细的时间对齐，不再只会“测量来了就硬挂到最近状态”。
- 初始化已经支持“高等级 IMU 长静止窗双矢量对准”，可以优先估计初始航向和零偏。
- IMU 姿态不仅保留在状态里，还增加了 `omega` 来支撑插值。
- 初始化不再用“第一个有限 GNSS”直接定原点，而是先做质量过滤和 IMU 覆盖检查。
- `IntegrateImuWindow` 现在会拒绝超出 IMU 支撑范围的时间窗，不做边界外推。
- 默认继续保留 `Use2ndOrderCoriolis(true)`，这点比原始 offline 默认更完整。

### 3.2 当前版本相对原始 gnssFGO 仍然可能吃亏的地方

- 目前只接 GNSS 位置，不接 GNSS 速度和姿态，所以信息量还是更少。
- GP 插值现在是最小可用迁移版，重点解决时间对齐，不是完整复刻原始 motion-prior 体系。
- 当前还是 batch offline runner，不是原始 time-centric + integrator/plugin 的完整形态。

所以如果问题是“现在这版是不是已经比原始工程更适合做离线 GNSS+IMU 精度基线”，答案是：

- 是，更适合做干净、可复现、可定位问题的基线。

如果问题是“它是不是已经在功能上完全追平原始 LC 路径的精度上限”，答案是：

- 还没有，主要差在 GNSS velocity / attitude 和完整 GP motion prior。

## 4. 这次改动后可以明确下来的结论

- 当前框架已经具备 robust GNSS 和 finer time alignment 两个关键增强点。
- 当前状态里已经保留 IMU 姿态，不需要再额外加一套“姿态状态”；这次新增的是 `omega` 支撑态。
- 当前结果目录除了 `trajectory.csv` 和 `gnss_residuals.csv`，还会输出 `gnss_alignment.csv`，可以直接看每个 GNSS 测量是同步、插值、还是被丢弃。
- 默认数据上，如果状态频率刚好和 GNSS 形成整齐对齐，绝大多数测量会走 `sync`；这是正常现象，不代表插值路径没实现。
- 在非整除状态频率配置下，`sync + interpolated` 两条路径都已经能跑通。

## 5. 当前建议

如果后面继续只围绕精度推进，优先级建议是：

1. 保持当前 robust + synchronization + interpolation 这套主干稳定。
2. 在此基础上补 GNSS velocity factor。
3. 只有在 velocity 稳定后，再考虑是否补 GNSS attitude 或更完整的 GP motion prior。
4. 不要为了“和原始工程更像”把 ROS/plugin/data adapter 复杂度重新引回来。
