# offline_lc_minimal v1.1

## 概要

`v1.1` 相对 `v1.0` 的重点不是单一算法替换，而是把项目扩展成了一个更完整的离线 IMU + GNSS 融合实验平台。

本版本的主要价值在于：

- 增强了静态初始化与静态一致性约束
- 增强了 GNSS 一致性建模与垂向观测控制
- 增强了 bias 建模实验能力
- 增加了更完整的诊断输出、绘图脚本和测试

## 主要变化

### 1. 初始化与静态约束增强

- 新增初始静态子图
- 支持静态 `ZUPT / ZARU`
- 支持静态零比力和姿态漂移约束
- 初始化链路不再只依赖单一启发式方式

### 2. GNSS 建模增强

- 增加 GNSS consistency / NIS 风格统计
- 支持水平与垂向 sigma 分离
- 支持垂向固定 sigma 与垂向低频参考实验
- 增加更细粒度的 GNSS 调试 CSV

### 3. bias 与过程模型实验增强

- 增加全局 accelerometer / gyro bias tie
- 增加水平 accelerometer bias tie
- 增加 bias GM 过程模型实验能力
- 增加垂向 accelerometer bias 过程模型实验能力

### 4. 诊断与输出增强

- 新增 `initial_static_trajectory.csv`
- 新增 `reference_node_trajectory.csv`
- 新增 `error_state_trajectory.csv`
- 新增 `segment_error_diagnostics.csv`
- 新增 `gnss_consistency.csv`
- 新增 `initial_dynamic_consistency.csv`
- `summary.txt` 增加静态窗口稳定性、GNSS 一致性、early dynamic bias/attitude 一致性等统计

### 5. 绘图与测试增强

- 新增姿态、速度、航向、高程、静态一致性等绘图脚本
- 导航主图与姿态图分离
- 增加 native test 与更完整的 Python 绘图测试

## 当前状态说明

`v1.1` 包含了稳定主线增强，也包含多条实验路线。

已经归档/取消的路线：

- `VerticalSpecificForceContinuityFactor` 连续性因子路线

取消原因：

- 它会与原有 `CombinedImuFactor` 对同一段 IMU 加速度证据产生重复使用
- 不适合作为干净的主图建模方式继续推进

## 参考文档

- 当前工作树相对 `v1.0` 的详细变化见：
  - [current_vs_release_v1.0.md](/D:/Code/offline_lc_minimal/docs/current_vs_release_v1.0.md)

