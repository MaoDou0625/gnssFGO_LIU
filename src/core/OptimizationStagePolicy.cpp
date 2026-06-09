#include "offline_lc_minimal/core/OptimizationStagePolicy.h"

namespace offline_lc_minimal {

OfflineRunnerConfig MakeStage1YawRefinementConfig(const OfflineRunnerConfig &config) {
  OfflineRunnerConfig stage_config = config;

  stage_config.enable_stage1_yaw_refinement = false;

  stage_config.enable_rtk_velocity_constraint = false;
  stage_config.enable_vertical_velocity_delta_constraint = false;
  stage_config.enable_vertical_motion_adaptive_reweighting = false;
  stage_config.enable_vertical_position_velocity_consistency_all_states = false;
  stage_config.enable_vertical_position_velocity_window_consistency = false;

  stage_config.enable_body_z_nhc_constraint = false;
  stage_config.enable_body_z_nhc_global_weak_constraint = false;
  stage_config.enable_body_z_nhc_strict_effective_weighting = false;
  stage_config.enable_body_z_nhc_horizontal_leakage_correction = false;

  stage_config.enable_vertical_jump_masked_imu = false;
  stage_config.enable_vertical_jump_impulse = false;
  stage_config.enable_vertical_jump_bias = false;
  stage_config.enable_vertical_jump_segmented_bias = false;
  stage_config.enable_vertical_jump_spectral_bias_relaxation = false;
  stage_config.enable_vertical_jump_velocity_ramp_smoothing = false;
  stage_config.enable_vertical_jump_position_ramp_smoothing = false;
  stage_config.enable_vertical_jump_velocity_continuity = false;
  stage_config.enable_vertical_jump_velocity_context_mean = false;
  stage_config.enable_vertical_jump_context_mean_continuity = false;
  stage_config.enable_vertical_jump_position_velocity_consistency = false;
  stage_config.enable_vertical_jump_velocity_height_slope_constraint = false;

  return stage_config;
}

OfflineRunnerConfig MakeStage2VelocityOptimizationConfig(const OfflineRunnerConfig &config) {
  OfflineRunnerConfig stage_config = config;

  stage_config.enable_stage1_yaw_refinement = false;
  stage_config.enable_stage2_velocity_optimization = true;

  stage_config.enable_attitude_reference_constraint = false;
  stage_config.enable_rtk_velocity_constraint = false;

  stage_config.enable_body_z_nhc_constraint = false;
  stage_config.enable_body_z_nhc_horizontal_leakage_correction = false;
  stage_config.enable_stage2_vehicle_nhc_constraint =
    config.enable_stage2_vehicle_nhc_constraint;

  return stage_config;
}

}  // namespace offline_lc_minimal
