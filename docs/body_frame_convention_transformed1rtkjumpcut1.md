# transformed1rtkjumpcut1 载体系坐标约定记录

本文档记录 `transformed1rtkjumpcut1` 数据集在当前 `offline_lc_minimal`
解算链路中的载体系坐标方向，用于后续 RTK 失效窗口、RTK 速度约束和水平
NHC 设计时避免误用。

## 数据集范围

- 数据集目录：
  `D:\Code\dataset\BeiJingGongLuTuiChe\gnssFGO_use\20260323_124742\transformed1rtkjumpcut1`
- IMU 文件：
  `imu_gnss_fgo.txt`
- GNSS/RTK 文件：
  `gnss_solution_gnss_fgo.txt`
- 验证运行输出：
  `runs\transformed1rtkjumpcut1_phase33_rtk_velocity_pre_outage`
- 验证诊断文件：
  `runs\transformed1rtkjumpcut1_phase33_rtk_velocity_pre_outage\rtk_velocity_diagnostics.csv`
- 验证图：
  `runs\transformed1rtkjumpcut1_phase33_rtk_velocity_pre_outage\plots\rtk_velocity_body_x_direction_check.png`

此结论是针对上述数据集和当前输入文件格式的记录。若后续更换 IMU 安装、
更换数据导出脚本、或在加载阶段增加 IMU/车辆外参重映射，需要重新验证。

## 当前程序中的体轴定义

当前代码把 GTSAM `Pose3` 的旋转矩阵列向量作为体轴在导航系中的方向：

- `rotation.matrix().col(0)`：`body_x_axis_nav`
- `rotation.matrix().col(1)`：`body_y_axis_nav`
- `rotation.matrix().col(2)`：`body_z_axis_nav`

对应代码位置：

- `include/offline_lc_minimal/factor/BodyZVelocityZeroFactor.h`
- `src/core/RtkVelocityConstraintBuilder.cpp`

当前程序没有在 RTK 速度诊断中额外做车辆坐标外参变换，因此这里的
`body_x/body_y/body_z` 应理解为当前解算使用的 IMU/body 坐标。

## transformed1rtkjumpcut1 的实测方向

基于 RTKFIX 差分速度投影验证，`transformed1rtkjumpcut1` 在当前解算中：

- `+body_x`：车辆前进方向。
- `-body_x`：车辆后退方向。
- `body_y`：水平侧向轴。
- `body_z`：竖向/安装相关轴，当前 Body-Z NHC 使用该轴约束体轴竖向速度。

不要把 `body_y` 当作前进轴；也不要把 `body_x` 取反作为前进轴。

## 数值证据

验证使用的是 RTK 失效前截断运行：

```powershell
./build/offline_lc_runner `
  --config config/transformed1cut1_vertical_envelope_phase32_rtk_outage_smoother.cfg `
  --imu /mnt/d/Code/dataset/BeiJingGongLuTuiChe/gnssFGO_use/20260323_124742/transformed1rtkjumpcut1/imu_gnss_fgo.txt `
  --gnss /mnt/d/Code/dataset/BeiJingGongLuTuiChe/gnssFGO_use/20260323_124742/transformed1rtkjumpcut1/gnss_solution_gnss_fgo.txt `
  --output-dir ./runs/transformed1rtkjumpcut1_phase33_rtk_velocity_pre_outage `
  --set processing_end_time_s=6006.988 `
  --set enable_rtk_velocity_constraint=true `
  --set rtk_velocity_window_s=1.0 `
  --set rtk_velocity_horizontal_sigma_mps=0.25
```

对 `rtk_velocity_diagnostics.csv` 中 `factor_added=1` 且 RTK 水平速度
`> 0.5 m/s` 的样本统计：

- 样本数：`867`
- `rtk_body_x_mps > 0`：`867`
- `rtk_body_x_mps < 0`：`0`
- `rtk_body_x_mps` 最小值：`+0.503689 m/s`
- `rtk_body_x_mps / rtk_horizontal_speed_mps` 平均值：`0.990051`
- `rtk_body_x_mps / rtk_horizontal_speed_mps` 最小值：`0.911881`

如果 `+body_x` 是后退方向，上述投影会整体接近 `-rtk_horizontal_speed_mps`。
实际结果全部为正，并且接近 RTK 水平速度，因此当前数据集中 `+body_x`
就是前进方向。

## 后续使用约束

在本数据集和当前坐标约定下，水平车辆约束应按以下方式理解：

- 前进速度在 `body_x` 上，不能强制为零。
- 侧向速度在 `body_y` 上，水平 NHC 应主要约束 `body_y` 接近零。
- `body_z` 继续用于竖向/安装相关 NHC，不应与前进轴混用。

同时需要注意，`body_y` 的正方向目前只记录为“水平侧向”。本文档没有证明
`+body_y` 对应车辆左侧还是右侧；如后续算法需要左右符号，需要单独通过
车辆安装信息或带明确转向方向的运动数据验证。
