#pragma once

#include <string>
#include <string_view>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct OfflineRunnerConfig {
  std::string imu_path;
  std::string gnss_path;
  std::string output_dir = "./runs/default_offline";

  bool enable_gnss = true;
  bool enable_gp_interpolated_gnss = true;
  bool verbose = false;
  bool write_debug_csv = true;
  bool write_imu_rate_avp = false;
  double state_frequency_hz = 20.0;
  double gnss_time_offset_s = 0.0;
  double state_meas_sync_lower_bound_s = -0.02;
  double state_meas_sync_upper_bound_s = 0.02;

  double gravity_mps2 = 9.81;
  double imu_sigma_acc = 0.08;
  double imu_sigma_gyro = 0.002;
  double integration_sigma = 1e-5;
  double bias_acc_sigma = 1e-4;
  double bias_gyro_sigma = 1e-5;
  double bias_acc_prior_sigma = 0.10;
  double bias_gyro_prior_sigma = 0.01;

  double stationary_window_s = 1.0;
  double stationary_acc_tolerance_mps2 = 0.8;
  double stationary_gyro_threshold_radps = 0.02;
  bool prefer_imu_initial_yaw = false;
  double imu_dual_vector_window_s = 100.0;
  int imu_dual_vector_min_sample_count = 1000;
  double imu_dual_vector_min_cross_norm = 1e-3;
  double yaw_min_distance_m = 1.0;
  double fallback_initial_yaw_rad = 0.0;

  double early_gnss_relaxation_duration_s = 5.0;
  double early_gnss_relaxation_scale = 3.0;

  double position_sigma_floor_m = 0.5;
  double position_sigma_ceiling_m = 50.0;
  GnssNoiseModel gnss_position_noise_model = GnssNoiseModel::kCauchy;
  double gnss_position_robust_param = 0.5;
  double rtkfix_scale = 1.0;
  double rtkfloat_scale = 2.0;
  double single_scale = 5.0;
  int required_best_sol_status_code = 1;
  bool drop_no_solution = true;
  bool drop_nonfinite_sigma = true;

  double initial_position_sigma_m = 3.0;
  double initial_roll_pitch_sigma_rad = 0.35;
  double initial_yaw_sigma_rad = 0.70;
  double initial_velocity_sigma_mps = 1.0;

  double lm_lambda_initial = 1e-3;
  int lm_max_iterations = 80;
};

OfflineRunnerConfig DefaultConfig();
OfflineRunnerConfig LoadConfigFile(std::string_view config_path, const OfflineRunnerConfig &base_config);
std::string ConfigToString(const OfflineRunnerConfig &config);
void OverrideConfigField(OfflineRunnerConfig &config, std::string_view key, std::string_view value);

}  // namespace offline_lc_minimal
