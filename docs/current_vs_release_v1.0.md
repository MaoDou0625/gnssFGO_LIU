# 当前工作树相对 `v1.0` Release 的变化

## 背景

- Release 基线：`v1.0`
- Release 提交：`1ae807d`，提交信息为 `Release offline_lc_minimal v1.0`
- 本文档描述的是：**当前工作树** 相对该 release 的变化
- 需要特别说明的是：当前状态**还不是新的正式版本**
- 当前工作树本质上是一个“工程主线 + 多条实验分支”的混合状态，里面同时包含：
  - 已经比较稳定的主路径增强
  - 新增的诊断、绘图和测试能力
  - 多条还没有完全收敛的实验路线

## 总体变化概览

相对 `v1.0`，当前工作树已经不再只是一个相对精简的离线 GNSS/IMU batch 处理程序，而是逐步演变成了一个更偏“实验平台”的版本，主要增强体现在：

1. 初始化和静态对准约束更丰富
2. GNSS 观测建模更灵活，尤其是垂向和一致性诊断
3. bias 建模和 IMU 权重实验显著增多
4. 诊断输出、CSV 和绘图脚本大幅增加
5. 原生测试和 Python 绘图测试更完整

同时也要注意：

- 当前工作树里探索过多条实验路线
- 不是所有路线都适合继续作为默认主线推进
- 其中有些路线已经被明确归档或取消

## 1. 主图与求解链路的变化

### 1.1 静态初始化与静态子图

当前工作树相对 `v1.0` 的最大结构性变化之一，是围绕起始静态窗口增加了一整套静态初始化能力，包括：

- `InitialStaticConstraintBuilder`
- 初始静态子图
- 可选的静态 `ZUPT / ZARU`
- 可选的零比力约束
- 可选的静态姿态漂移约束

这样做的目标是：

- 让第一个导航状态不再只依赖单次启发式初始化
- 而是尽可能由显式静态约束共同决定

相关文件：

- [D:/Code/offline_lc_minimal/src/core/OfflineBatchRunner.cpp](/D:/Code/offline_lc_minimal/src/core/OfflineBatchRunner.cpp)
- [D:/Code/offline_lc_minimal/src/core/InitialStaticConstraintBuilder.cpp](/D:/Code/offline_lc_minimal/src/core/InitialStaticConstraintBuilder.cpp)
- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/core/InitialStaticConstraintBuilder.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/core/InitialStaticConstraintBuilder.h)

### 1.2 bias 建模实验明显增多

相对 release，当前工作树增加了多种 bias 因子和 bias 过程模型实验，包括：

- 全局 accelerometer bias tie
- 全局 gyro bias tie
- 水平 accelerometer bias tie
- bias GM transition
- 垂向 accelerometer bias GM transition

这些改动使当前 runner 可以测试多种建模假设，例如：

- bias 近似全局常值
- bias 缓慢变化
- 水平与垂向 accelerometer bias 分开处理

相关文件：

- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/GlobalAccelBiasFactor.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/GlobalAccelBiasFactor.h)
- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/GlobalGyroBiasFactor.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/GlobalGyroBiasFactor.h)
- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/GlobalPlanarAccelBiasFactor.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/GlobalPlanarAccelBiasFactor.h)
- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/BiasGmTransitionFactor.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/BiasGmTransitionFactor.h)
- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/VerticalAccelBiasGmTransitionFactor.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/VerticalAccelBiasGmTransitionFactor.h)

### 1.3 IMU 主边重加权与平移可信度实验

当前工作树新增了一个自定义的 IMU 包装因子：

- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/ReweightedCombinedImuFactor.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/ReweightedCombinedImuFactor.h)

它主要被用于探索：

- 更强的姿态可信度
- 轴向 `specific_force` 驱动的平移可信度实验

当前状态是：

- 姿态重加权仍然属于主路径的一部分
- 平移侧的 `specific_force`/continuity 类实验目前都还没有收敛
- 其中失败的 continuity 因子路线已经从主路径里摘除

### 1.4 显式误差状态与 segment feedback 实验

相对 `v1.0`，当前工作树还新增了一整组“慢变误差状态/分段反馈”实验模块，包括：

- 显式误差状态模型
- 误差状态校正因子
- 误差状态传播因子
- segment bias feedback 因子

这些模块主要用于验证：

- 低频误差状态建模
- 分段反馈
- 慢变 bias 与姿态误差的解释能力

相关文件：

- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/core/ErrorStateModel.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/core/ErrorStateModel.h)
- [D:/Code/offline_lc_minimal/src/core/ErrorStateModel.cpp](/D:/Code/offline_lc_minimal/src/core/ErrorStateModel.cpp)
- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/ErrorStateCorrectionFactor.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/ErrorStateCorrectionFactor.h)
- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/ErrorStateTransitionFactor.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/ErrorStateTransitionFactor.h)
- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/SegmentBiasFeedbackFactor.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/factor/SegmentBiasFeedbackFactor.h)

这些模块目前更适合被视为**实验能力**，还不能简单视为 release 级稳定主线。

## 2. GNSS 观测建模的变化

相对 `v1.0`，当前工作树对 GNSS 的处理已经明显更复杂，尤其是在“一致性”和“垂向”两个方向。

### 2.1 GNSS 一致性门限与统计

当前工作树新增了与 GNSS 一致性相关的一整套逻辑和输出，包括：

- GNSS consistency gate mode
- NIS 风格的一致性统计
- 按轴 `2σ` 通过率统计
- GNSS 因子协方差缩放

相关文件：

- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/common/Types.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/common/Types.h)
- [D:/Code/offline_lc_minimal/src/core/OfflineBatchRunner.cpp](/D:/Code/offline_lc_minimal/src/core/OfflineBatchRunner.cpp)
- [D:/Code/offline_lc_minimal/src/common/ResultWriter.cpp](/D:/Code/offline_lc_minimal/src/common/ResultWriter.cpp)

### 2.2 水平与垂向 sigma 分离

当前工作树新增了对水平和垂向 GNSS sigma 的独立处理，包括：

- 水平 sigma floor
- 垂向 sigma floor
- 垂向 sigma 使用文件值还是固定值
- 垂向低频参考 / 慢飘参考

这和 `v1.0` 相比是一个很重要的变化，因为垂向现在被明确当作和水平不同的问题来处理。

## 3. 配置面的扩张

`OfflineRunnerConfig` 相比 release 明显变大，新增的配置大致可分为几组：

- 静态对准与静态子图控制
- GNSS 一致性门限控制
- GNSS 垂向 sigma 与慢飘控制
- 全局 bias / 水平 bias / 垂向 bias 过程控制
- 误差状态 / segment feedback 实验控制
- IMU 重加权控制

相关文件：

- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/common/Config.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/common/Config.h)
- [D:/Code/offline_lc_minimal/src/common/Config.cpp](/D:/Code/offline_lc_minimal/src/common/Config.cpp)
- [D:/Code/offline_lc_minimal/config/default_offline.cfg](/D:/Code/offline_lc_minimal/config/default_offline.cfg)

整体上看，项目已经从 release 时较为紧凑的配置形态，演变成了一个实验参数很多的研究型配置面。

## 4. 新增输出与诊断

这是相对 `v1.0` 最明显的实用增强之一。

当前工作树新增了许多 CSV 和摘要输出，例如：

- `initial_static_trajectory.csv`
- `reference_node_trajectory.csv`
- `error_state_trajectory.csv`
- `segment_error_diagnostics.csv`
- `segment_error_summary.txt`
- `gnss_consistency.csv`
- `initial_dynamic_consistency.csv`

`summary.txt` 也新增了大量统计项，包括：

- 静态窗口稳定性指标
- 静态 specific-force 窗口统计
- GNSS 一致性统计
- feedback-forward 斜率诊断
- 早期动态 bias / 姿态一致性诊断

相关文件：

- [D:/Code/offline_lc_minimal/include/offline_lc_minimal/common/Types.h](/D:/Code/offline_lc_minimal/include/offline_lc_minimal/common/Types.h)
- [D:/Code/offline_lc_minimal/src/common/ResultWriter.cpp](/D:/Code/offline_lc_minimal/src/common/ResultWriter.cpp)
- [D:/Code/offline_lc_minimal/src/core/OfflineBatchRunner.cpp](/D:/Code/offline_lc_minimal/src/core/OfflineBatchRunner.cpp)

## 5. 绘图与分析工具的大幅扩展

release 版本的分析脚本明显更少。当前工作树新增了一套比较完整的绘图工具：

- [D:/Code/offline_lc_minimal/scripts/plot_attitude_over_time.py](/D:/Code/offline_lc_minimal/scripts/plot_attitude_over_time.py)
- [D:/Code/offline_lc_minimal/scripts/plot_speed_vs_rtk.py](/D:/Code/offline_lc_minimal/scripts/plot_speed_vs_rtk.py)
- [D:/Code/offline_lc_minimal/scripts/plot_heading_vs_rtk.py](/D:/Code/offline_lc_minimal/scripts/plot_heading_vs_rtk.py)
- [D:/Code/offline_lc_minimal/scripts/plot_forward_heading_diagnostic.py](/D:/Code/offline_lc_minimal/scripts/plot_forward_heading_diagnostic.py)
- [D:/Code/offline_lc_minimal/scripts/plot_forward_altitude_diagnostic.py](/D:/Code/offline_lc_minimal/scripts/plot_forward_altitude_diagnostic.py)
- [D:/Code/offline_lc_minimal/scripts/plot_initial_static_consistency.py](/D:/Code/offline_lc_minimal/scripts/plot_initial_static_consistency.py)
- [D:/Code/offline_lc_minimal/scripts/plot_error_state_trajectory.py](/D:/Code/offline_lc_minimal/scripts/plot_error_state_trajectory.py)
- [D:/Code/offline_lc_minimal/scripts/plot_segment_error_diagnostics.py](/D:/Code/offline_lc_minimal/scripts/plot_segment_error_diagnostics.py)
- [D:/Code/offline_lc_minimal/scripts/plot_optimized_vertical_profile.py](/D:/Code/offline_lc_minimal/scripts/plot_optimized_vertical_profile.py)

同时：

- [D:/Code/offline_lc_minimal/scripts/plot_nav_vs_rtk.py](/D:/Code/offline_lc_minimal/scripts/plot_nav_vs_rtk.py) 也被调整过
- 导航主图重新聚焦在导航/位置对比
- 姿态被单独拆成独立图来显示

## 6. 测试与构建的变化

相对 `v1.0`，当前工作树在测试层面也更完整。

### 6.1 原生测试

新增了：

- `OFFLINE_LC_MINIMAL_BUILD_TESTS` CMake 选项
- 原生测试目标
  - [D:/Code/offline_lc_minimal/tests/reweighted_combined_imu_factor_test.cpp](/D:/Code/offline_lc_minimal/tests/reweighted_combined_imu_factor_test.cpp)

### 6.2 Python 绘图测试

新增的绘图测试包括：

- [D:/Code/offline_lc_minimal/tests/test_plot_attitude_over_time.py](/D:/Code/offline_lc_minimal/tests/test_plot_attitude_over_time.py)
- [D:/Code/offline_lc_minimal/tests/test_plot_speed_vs_rtk.py](/D:/Code/offline_lc_minimal/tests/test_plot_speed_vs_rtk.py)
- [D:/Code/offline_lc_minimal/tests/test_plot_heading_vs_rtk.py](/D:/Code/offline_lc_minimal/tests/test_plot_heading_vs_rtk.py)
- [D:/Code/offline_lc_minimal/tests/test_plot_forward_heading_diagnostic.py](/D:/Code/offline_lc_minimal/tests/test_plot_forward_heading_diagnostic.py)
- [D:/Code/offline_lc_minimal/tests/test_plot_forward_altitude_diagnostic.py](/D:/Code/offline_lc_minimal/tests/test_plot_forward_altitude_diagnostic.py)
- [D:/Code/offline_lc_minimal/tests/test_plot_nav_vs_rtk.py](/D:/Code/offline_lc_minimal/tests/test_plot_nav_vs_rtk.py)
- [D:/Code/offline_lc_minimal/tests/test_plot_segment_error_diagnostics.py](/D:/Code/offline_lc_minimal/tests/test_plot_segment_error_diagnostics.py)
- [D:/Code/offline_lc_minimal/tests/test_plot_optimized_vertical_profile.py](/D:/Code/offline_lc_minimal/tests/test_plot_optimized_vertical_profile.py)

## 7. Release 之后新增的实验配置

当前工作树里新增了大量实验配置文件，这些配置并不都代表“当前推荐路线”，但它们记录了 release 之后的主要探索方向。

主要包括：

- `static100_*` 系列实验
- `segment_feedback_stage1/stage2/stage3` 系列实验
- `error_state_stage1/error_state_feedback` 系列实验
- `specificforce_z*` 平移可信度实验
- 垂向 GNSS 实验，例如：
  - 固定垂向 sigma
  - 垂向慢飘参考
  - 垂向 accelerometer bias 过程实验

示例配置：

- [D:/Code/offline_lc_minimal/config/transformed1cut1_segment_feedback_stage1.cfg](/D:/Code/offline_lc_minimal/config/transformed1cut1_segment_feedback_stage1.cfg)
- [D:/Code/offline_lc_minimal/config/transformed1cut1_segment_feedback_stage2_vertical_drift.cfg](/D:/Code/offline_lc_minimal/config/transformed1cut1_segment_feedback_stage2_vertical_drift.cfg)
- [D:/Code/offline_lc_minimal/config/transformed1cut1_segment_feedback_stage2_vertical_acc_bias.cfg](/D:/Code/offline_lc_minimal/config/transformed1cut1_segment_feedback_stage2_vertical_acc_bias.cfg)
- [D:/Code/offline_lc_minimal/config/transformed1cut1_segment_feedback_stage1_specificforce_z0100.cfg](/D:/Code/offline_lc_minimal/config/transformed1cut1_segment_feedback_stage1_specificforce_z0100.cfg)

这些配置更适合作为实验历史，而不是直接视为当前“正式配置”。

## 8. 当前稳定性判断

如果从“当前工作树是否已经整体优于 release”这个角度看，结论不是简单的“是”或“否”，而是：

### 已经比较稳定、值得保留的能力

- 静态子图与静态约束工具链
- GNSS 一致性诊断
- 更丰富的 summary 和 CSV 输出
- 绘图和测试工具链
- bias tie / bias process 的实验基础设施

### 仍属于实验状态的部分

- 显式 error-state 路线
- segment feedback 路线
- `specific_force` 平移可信度路线
- 垂向 bias 过程模型调参与收敛

### 已明确取消的路线

`VerticalSpecificForceContinuityFactor` 这条 continuity 路线已经被从主路径中摘掉。原因是它会对同一段 IMU 加速度证据产生重复使用，不适合作为干净的主图建模方式继续推进。

## 9. 实际理解上的一句话总结

如果用一句话概括当前工作树相对 `v1.0` 的变化：

> 项目已经从一个较为紧凑的离线 batch 求解器，逐步演变为一个围绕初始化、bias、GNSS 一致性和垂向诊断进行快速迭代的实验平台。

这意味着当前工作树的主要提升，不只是“代码变多”，而是：

- 更强调诊断驱动
- 更强调实验可比性
- 更强调主线与实验分支并存

## 10. 最近的 Git 里程碑

围绕当前 release 基线，最近几次关键提交是：

1. `1ae807d` — `Release offline_lc_minimal v1.0`
2. `33eac1c` — `Move offline_lc_minimal to repository root`
3. `6fbd8b9` — `Trim branch to offline_lc_minimal only`
4. `e134457` — `Add standalone offline LC minimal project`

