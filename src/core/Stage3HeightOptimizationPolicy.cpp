#include "offline_lc_minimal/core/Stage3HeightOptimizationPolicy.h"

#include <utility>

#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/RtkOutageSegmentationPolicy.h"

namespace offline_lc_minimal {
namespace {

void DisableStage3LegacyJumpRegularizers(OfflineRunnerConfig &config) {
  config.enable_stage3_jump_velocity_smoothness_regularizer = false;
  config.enable_stage3_jump_height_highfreq_deadband = false;
  config.enable_stage3_jump_adaptive_context_envelope = false;
}

void DisableRtkDriftAndOutageVerticalReferences(OfflineRunnerConfig &config) {
  config = DisableRtkOutageSegmentedBatchRecursion(std::move(config));
  config.stage3_disable_rtk_outage_segmented_batch = false;
  config.enable_rtk_outage_smoothing = false;
  config.enable_rtk_outage_velocity_delta_3d = false;
  config.enable_rtk_vertical_drift_reference = false;
  config.enable_rtk_vertical_lowpass_reference = false;
  config.enable_rtk_outage_causal_drift_reference = false;
  config.enable_rtk_outage_preoutage_vertical_fence = false;
  config.enable_late_static_detection = false;
  config.enable_initial_static_rtk_height_reference = false;
  config.gnss_vertical_reference_source = GnssVerticalReferenceSource::kRawRtk;
}

void DisableCompetingAttitudeAndHorizontalSolvers(OfflineRunnerConfig &config) {
  config.enable_attitude_reference_constraint = false;
  config.enable_base_graph_tilt_reference_constraint = false;
  config.enable_body_z_nhc_constraint = false;
  config.enable_body_z_nhc_global_weak_constraint = false;
  config.enable_body_z_nhc_strict_effective_weighting = false;
  config.enable_body_z_nhc_horizontal_leakage_correction = false;
  config.enable_stage2_vehicle_nhc_constraint = false;
  config.stage3_disable_stage2_vehicle_nhc_constraint = false;
}

void DisableRawRtkStaticVerticalPulls(OfflineRunnerConfig &config) {
  config.enable_initial_dynamic_static_detection = false;
  config.enable_initial_dynamic_static_lowpass_protection = false;
  config.enable_initial_dynamic_static_vz_constraint = false;
}

void ApplyLowFrequencyStage2DeltaPolicy(OfflineRunnerConfig &config) {
  config.stage3_vertical_reference_constraint_mode =
    Stage3VerticalReferenceConstraintMode::kGaussian;
  config.stage3_vertical_anchor_sigma_m = 0.001;

  config.enable_stage3_stage2_vertical_increment_hold = true;
  config.stage3_stage2_vertical_increment_sigma_m = 0.0002;
  config.stage3_stage2_vertical_increment_jump_sigma_m = 0.0005;

  config.enable_stage3_stage2_jump_shape_hold = true;
  config.stage3_stage2_jump_shape_sigma_m = 0.0005;
}

}  // namespace

OfflineRunnerConfig MakeStage3HeightReferenceSourceConfig(
  OfflineRunnerConfig config) {
  config.enable_stage3_vertical_reference_optimization = false;
  config.stage3_disable_stage2_vehicle_nhc_constraint = false;
  DisableStage3LegacyJumpRegularizers(config);
  return config;
}

OfflineRunnerConfig MakeStage3HeightOptimizationConfig(
  OfflineRunnerConfig config) {
  config = MakeStage2VelocityOptimizationConfig(config);
  config.enable_stage3_vertical_reference_optimization = false;

  DisableRtkDriftAndOutageVerticalReferences(config);
  DisableCompetingAttitudeAndHorizontalSolvers(config);
  DisableRawRtkStaticVerticalPulls(config);
  DisableStage3LegacyJumpRegularizers(config);
  ApplyLowFrequencyStage2DeltaPolicy(config);

  return config;
}

}  // namespace offline_lc_minimal
