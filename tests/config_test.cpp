#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Units.h"

namespace {

template <typename Function>
void RunTest(const std::string &name, Function &&function) {
  try {
    function();
  } catch (const std::exception &exception) {
    throw std::runtime_error(name + ": " + exception.what());
  }
}

void ExpectTrue(const bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void ExpectUnknownKey(const std::string &key) {
  auto config = offline_lc_minimal::DefaultConfig();
  bool threw = false;
  try {
    offline_lc_minimal::OverrideConfigField(config, key, "true");
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("unknown config key") != std::string::npos;
  }
  ExpectTrue(threw, key + " should be rejected as an unknown config key");
}

void TestDirectZSmokeConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_direct_z_phase1.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(config.enable_body_z_jump_detection, "direct-z smoke config should enable body-z detection");
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kDirectZ,
    "direct-z smoke config should select direct_z vertical constraints");
}

void TestEnvelopeSmokeConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase2.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(config.enable_body_z_jump_detection, "envelope smoke config should enable body-z detection");
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "envelope smoke config should select envelope vertical constraints");
  ExpectTrue(
    std::abs(config.vertical_envelope_gate_sigma_multiple - 2.0) < 1e-12,
    "envelope sigma gate should load");
  ExpectTrue(
    std::abs(config.vertical_envelope_min_half_width_m - 0.10) < 1e-12,
    "envelope min half-width should load");
  ExpectTrue(
    std::abs(config.vertical_envelope_factor_sigma_m - 0.20) < 1e-12,
    "envelope factor sigma should load");
}

void TestPhase3SmokeConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase3.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(config.enable_body_z_jump_detection, "phase3 config should enable body-z detection");
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase3 config should use envelope constraints");
  ExpectTrue(config.enable_vertical_velocity_delta_constraint, "phase3 config should enable velocity delta constraints");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_acc_sigma_mps2 - 0.50) < 1e-12,
    "velocity delta acc sigma should load");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_min_sigma_mps - 0.02) < 1e-12,
    "velocity delta min sigma should load");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_jump_padding_s - 0.25) < 1e-12,
    "velocity delta jump padding should load");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_target_acc_limit_mps2 - 0.85) < 1e-12,
    "velocity delta target acceleration limit should load");
}

void TestPhase4SmokeConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase4.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(config.enable_body_z_jump_detection, "phase4 config should enable body-z detection");
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase4 config should use envelope constraints");
  ExpectTrue(config.enable_vertical_velocity_delta_constraint, "phase4 config should keep velocity delta constraints");
  ExpectTrue(config.enable_vertical_jump_masked_imu, "phase4 config should enable vertical masked IMU");
  ExpectTrue(
    std::abs(config.vertical_jump_masked_imu_padding_s - 0.25) < 1e-12,
    "vertical jump masked IMU padding should load");
  ExpectTrue(
    config.enable_vertical_jump_velocity_ramp_smoothing,
    "phase4 config should enable jump velocity ramp smoothing");
  ExpectTrue(
    std::abs(config.vertical_jump_velocity_ramp_sigma_mps - 0.08) < 1e-12,
    "vertical jump velocity ramp sigma should load");
  ExpectTrue(
    config.enable_vertical_jump_position_ramp_smoothing,
    "phase4 config should enable jump position ramp smoothing");
  ExpectTrue(
    std::abs(config.vertical_jump_position_ramp_sigma_m - 0.10) < 1e-12,
    "vertical jump position ramp sigma should load");
  ExpectTrue(!config.enable_vertical_jump_velocity_continuity, "phase4 should keep velocity continuity disabled");
  ExpectTrue(
    !config.enable_vertical_jump_position_velocity_consistency,
    "phase4 should keep position-velocity consistency disabled");
  ExpectTrue(
    config.enable_vertical_jump_velocity_height_slope_constraint,
    "phase4 should preserve velocity height slope constraint");
  ExpectTrue(
    std::abs(config.vertical_jump_velocity_height_slope_sigma_mps - 0.50) < 1e-12,
    "vertical jump velocity height slope sigma should load");
}

void TestPhase5SmokeConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase5_continuous_vz.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(config.enable_body_z_jump_detection, "phase5 config should enable body-z detection");
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase5 config should use envelope constraints");
  ExpectTrue(config.enable_vertical_velocity_delta_constraint, "phase5 should keep velocity delta constraints");
  ExpectTrue(config.enable_vertical_jump_masked_imu, "phase5 should keep vertical masked IMU");
  ExpectTrue(config.enable_vertical_jump_velocity_ramp_smoothing, "phase5 should enable velocity ramp");
  ExpectTrue(config.enable_vertical_jump_position_ramp_smoothing, "phase5 should enable position ramp");
  ExpectTrue(config.enable_vertical_jump_velocity_continuity, "phase5 should enable velocity continuity");
  ExpectTrue(
    std::abs(config.vertical_jump_velocity_continuity_sigma_mps - 0.02) < 1e-12,
    "phase5 velocity continuity sigma should load");
  ExpectTrue(config.enable_vertical_jump_velocity_context_mean, "phase5 should enable velocity context mean");
  ExpectTrue(
    std::abs(config.vertical_jump_velocity_context_window_s - 1.0) < 1e-12,
    "phase5 velocity context window should load");
  ExpectTrue(
    std::abs(config.vertical_jump_velocity_context_mean_sigma_mps - 0.03) < 1e-12,
    "phase5 velocity context sigma should load");
  ExpectTrue(
    config.enable_vertical_jump_position_velocity_consistency,
    "phase5 should enable position-velocity consistency");
  ExpectTrue(
    std::abs(config.vertical_jump_position_velocity_consistency_sigma_m - 0.08) < 1e-12,
    "phase5 position-velocity consistency sigma should load");
  ExpectTrue(
    std::abs(config.vertical_jump_boundary_position_velocity_consistency_sigma_m - 0.01) < 1e-12,
    "phase5 boundary position-velocity consistency sigma should load");
  ExpectTrue(
    !config.enable_vertical_jump_velocity_height_slope_constraint,
    "phase5 should disable velocity height slope constraint");
}

void TestPhase6SmokeConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase6_context_mean_continuity.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(config.enable_body_z_jump_detection, "phase6 config should enable body-z detection");
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase6 config should use envelope constraints");
  ExpectTrue(config.enable_vertical_velocity_delta_constraint, "phase6 should keep velocity delta constraints");
  ExpectTrue(config.enable_vertical_jump_masked_imu, "phase6 should keep vertical masked IMU");
  ExpectTrue(config.enable_vertical_jump_velocity_context_mean, "phase6 should keep velocity context mean");
  ExpectTrue(
    config.enable_vertical_jump_context_mean_continuity,
    "phase6 should enable context mean continuity");
  ExpectTrue(
    std::abs(config.vertical_jump_context_mean_continuity_sigma_mps - 0.01) < 1e-12,
    "phase6 context mean continuity sigma should load");
  ExpectTrue(!config.enable_vertical_envelope_center_pull, "phase6 should leave center pull disabled");
}

void TestPhase7SmokeConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase7_center_pull.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(config.enable_body_z_jump_detection, "phase7 config should enable body-z detection");
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase7 config should use envelope constraints");
  ExpectTrue(config.enable_vertical_velocity_delta_constraint, "phase7 should keep velocity delta constraints");
  ExpectTrue(config.enable_vertical_jump_masked_imu, "phase7 should keep vertical masked IMU");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase7 should enable center pull");
  ExpectTrue(
    std::abs(config.vertical_envelope_center_sigma_m - 0.60) < 1e-12,
    "phase7 center pull sigma should load");
  ExpectTrue(
    std::abs(config.vertical_envelope_center_deadband_m - 0.01) < 1e-12,
    "phase7 center pull deadband should load");
  ExpectTrue(
    std::abs(config.vertical_acc_bias_sigma_mps2 - 0.01) < 1e-12,
    "phase7 should tighten vertical accelerometer bias GM sigma");
  ExpectTrue(
    config.enable_initial_static_vertical_bias_soft_prior,
    "phase7 should enable initial static vertical bias soft prior");
  ExpectTrue(
    std::abs(config.initial_static_vertical_bias_global_tie_sigma_mps2 - 5e-5) < 1e-12,
    "phase7 static vertical bias sigma should load");
}

void TestPhase7TightDvzConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase7_tight_dvz.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase7 tight-dvz config should use envelope constraints");
  ExpectTrue(config.enable_vertical_velocity_delta_constraint, "phase7 tight-dvz should keep velocity delta enabled");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase7 tight-dvz should keep center pull enabled");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_acc_sigma_mps2 - 0.10) < 1e-12,
    "phase7 tight-dvz should tighten velocity delta acceleration sigma");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_min_sigma_mps - 0.003) < 1e-12,
    "phase7 tight-dvz should tighten velocity delta minimum sigma");
}

void TestPhase8ImpulseConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase8_impulse.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase8 impulse config should use envelope constraints");
  ExpectTrue(config.enable_vertical_velocity_delta_constraint, "phase8 should keep jump-outside dvz constraints");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase8 should keep center pull enabled");
  ExpectTrue(!config.enable_vertical_jump_masked_imu, "phase8 should not enable legacy masked IMU mode");
  ExpectTrue(config.enable_vertical_jump_impulse, "phase8 should enable jump impulse constraints");
  ExpectTrue(
    std::abs(config.vertical_jump_impulse_prior_sigma_mps - 0.30) < 1e-12,
    "phase8 impulse prior sigma should load");
  ExpectTrue(
    std::abs(config.vertical_jump_impulse_velocity_sigma_mps - 0.03) < 1e-12,
    "phase8 impulse velocity sigma should load");
  ExpectTrue(
    std::abs(config.vertical_jump_impulse_position_velocity_sigma_m - 0.02) < 1e-12,
    "phase8 impulse position-velocity sigma should load");
  ExpectTrue(config.enable_vertical_jump_velocity_continuity, "phase8 should keep boundary velocity continuity");
  ExpectTrue(
    config.enable_vertical_jump_position_velocity_consistency,
    "phase8 should keep jump position-velocity consistency");
  ExpectTrue(config.enable_vertical_jump_velocity_context_mean, "phase8 should keep weak context mean smoothing");
  ExpectTrue(config.enable_vertical_jump_context_mean_continuity, "phase8 should keep weak context mean continuity");
  ExpectTrue(config.enable_vertical_jump_velocity_ramp_smoothing, "phase8 should keep weak velocity ramp smoothing");
  ExpectTrue(config.enable_vertical_jump_position_ramp_smoothing, "phase8 should keep weak position ramp smoothing");
}

void TestPhase9JumpBiasConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase9_jump_bias.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase9 jump-bias config should use envelope constraints");
  ExpectTrue(config.enable_vertical_velocity_delta_constraint, "phase9 should keep jump-outside dvz constraints");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase9 should keep center pull enabled");
  ExpectTrue(!config.enable_vertical_jump_impulse, "phase9 should disable jump impulse");
  ExpectTrue(!config.enable_vertical_jump_masked_imu, "phase9 should disable standalone masked IMU");
  ExpectTrue(config.enable_vertical_jump_bias, "phase9 should enable jump-local bias constraints");
  ExpectTrue(
    std::abs(config.vertical_jump_bias_padding_s - 0.0) < 1e-12,
    "phase9 jump-bias padding should load");
  ExpectTrue(
    std::abs(config.vertical_jump_bias_prior_sigma_mps2 - 0.05) < 1e-12,
    "phase9 jump-bias prior sigma should load");
  ExpectTrue(
    std::abs(config.vertical_jump_bias_velocity_sigma_mps - 0.01) < 1e-12,
    "phase9 jump-bias velocity sigma should load");
  ExpectTrue(
    std::abs(config.vertical_jump_bias_position_velocity_sigma_m - 0.02) < 1e-12,
    "phase9 jump-bias position-velocity sigma should load");
  ExpectTrue(!config.enable_vertical_jump_velocity_ramp_smoothing, "phase9 should disable velocity ramp smoothing");
  ExpectTrue(!config.enable_vertical_jump_velocity_context_mean, "phase9 should disable context mean smoothing");
}

void TestPhase10SegmentedJumpBiasConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase10_segmented_jump_bias.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase10 segmented jump-bias config should use envelope constraints");
  ExpectTrue(config.enable_vertical_jump_bias, "phase10 should enable jump-local bias constraints");
  ExpectTrue(config.enable_vertical_jump_segmented_bias, "phase10 should enable segmented jump bias");
  ExpectTrue(
    std::abs(config.vertical_jump_segmented_bias_min_segment_s - 0.30) < 1e-12,
    "phase10 segmented min duration should load");
  ExpectTrue(
    config.vertical_jump_segmented_bias_max_segments == 5,
    "phase10 segmented max segment count should load");
  ExpectTrue(
    std::abs(config.vertical_jump_segmented_bias_slope_merge_threshold_mps2 - 0.015) < 1e-12,
    "phase10 segmented slope merge threshold should load");
  ExpectTrue(
    std::abs(config.vertical_jump_bias_highfreq_sigma_scale - 0.02) < 1e-12,
    "phase10 high-frequency sigma scale should load");
  ExpectTrue(
    std::abs(config.vertical_jump_bias_highfreq_sigma_max_mps - 0.08) < 1e-12,
    "phase10 high-frequency sigma cap should load");
}

void TestPhase12RtkGateOnlyFullNavConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase12_rtk_gate_only_full_nav.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase12 gate-only config should use envelope constraints");
  ExpectTrue(!config.enable_vertical_envelope_center_pull, "phase12 should disable center pull");
  ExpectTrue(
    std::abs(config.vertical_envelope_gate_sigma_multiple - 2.0) < 1e-12,
    "phase12 should keep the 2-sigma vertical envelope gate");
  ExpectTrue(
    std::abs(config.vertical_envelope_min_half_width_m - 0.10) < 1e-12,
    "phase12 should keep the minimum vertical envelope half-width");
  ExpectTrue(
    std::abs(config.vertical_envelope_factor_sigma_m - 0.20) < 1e-12,
    "phase12 should keep the gate-outside vertical sigma");
  ExpectTrue(config.enable_gp_interpolated_gnss, "phase12 should keep full GNSS interpolation enabled");
  ExpectTrue(!config.drop_non_rtkfix, "phase12 should keep all valid non-RTKFIX samples available");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase12 should keep fixed-axis body-z NHC enabled");
  ExpectTrue(
    !config.enable_body_z_nhc_global_weak_constraint,
    "phase12 should keep global weak body-z NHC disabled");
  ExpectTrue(
    std::abs(config.body_z_nhc_jump_velocity_sigma_mps - 0.005) < 1e-12,
    "phase12 should keep strong jump-window NHC velocity sigma");
  ExpectTrue(
      std::abs(config.body_z_nhc_jump_displacement_sigma_m - 0.005) < 1e-12,
      "phase12 should keep strong jump-window NHC displacement sigma");
}

void TestPhase15StaticBazGmTightenedConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase15_static_baz_gm_tightened.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase15 config should use envelope constraints");
  ExpectTrue(!config.enable_vertical_envelope_center_pull, "phase15 should keep center pull disabled");
  ExpectTrue(config.enable_initial_static_subgraph, "phase15 should keep static subgraph enabled");
  ExpectTrue(config.enable_global_acc_bias, "phase15 should keep global accelerometer bias enabled");
  ExpectTrue(config.enable_vertical_acc_bias_gm_process, "phase15 should keep vertical GM bias process enabled");
  ExpectTrue(
    config.enable_initial_static_vertical_specific_force,
    "phase15 should keep static vertical specific force enabled");
  ExpectTrue(
    config.enable_initial_static_vertical_bias_soft_prior,
    "phase15 should keep static vertical bias soft prior enabled");
  ExpectTrue(
    config.enable_initial_static_vertical_bias_gm_tightening,
    "phase15 should tighten initial static vertical bias GM");
  ExpectTrue(
    std::abs(config.initial_static_vertical_bias_gm_sigma_mps2 -
             offline_lc_minimal::MicroGToMps2(0.02)) < 1e-15,
    "phase15 static vertical GM sigma should load from ug");
  ExpectTrue(
    std::abs(config.initial_static_vertical_bias_global_tie_sigma_mps2 -
             offline_lc_minimal::MicroGToMps2(0.05)) < 1e-15,
    "phase15 static vertical global tie sigma should load from ug");
  ExpectTrue(
    config.enable_initial_static_vertical_position_hold,
    "phase15 should enable static vertical position hold");
  ExpectTrue(
    std::abs(config.initial_static_vertical_position_hold_sigma_m - 0.005) < 1e-12,
    "phase15 static vertical position hold sigma should load");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase15 should keep fixed-axis body-z NHC enabled");
  ExpectTrue(
    !config.enable_body_z_nhc_global_weak_constraint,
    "phase15 should keep global weak body-z NHC disabled");
  ExpectTrue(
    std::abs(config.early_gnss_relaxation_duration_s) < 1e-12,
    "phase15 should disable early GNSS/RTK relaxation duration");
  ExpectTrue(
    std::abs(config.early_gnss_relaxation_scale - 1.0) < 1e-12,
    "phase15 should use neutral early GNSS/RTK relaxation scale");
}

void TestPhase16StaticRtkHeightAnchorConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase16_static_rtk_height_anchor.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase16 config should use envelope constraints");
  ExpectTrue(!config.enable_vertical_envelope_center_pull, "phase16 should keep center pull disabled");
  ExpectTrue(config.enable_initial_static_subgraph, "phase16 should keep static subgraph enabled");
  ExpectTrue(
    config.enable_initial_static_vertical_position_hold,
    "phase16 should keep static vertical position hold");
  ExpectTrue(
    config.enable_initial_static_rtk_height_reference,
    "phase16 should enable static RTK height reference");
  ExpectTrue(
    std::abs(config.initial_static_rtk_height_reference_sigma_m - 0.02) < 1e-12,
    "phase16 static RTK height reference sigma should load");
  ExpectTrue(
    config.initial_static_rtk_height_reference_min_sample_count == 20,
    "phase16 static RTK height minimum sample count should load");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase16 should keep fixed-axis body-z NHC enabled");
  ExpectTrue(
    !config.enable_body_z_nhc_global_weak_constraint,
    "phase16 should keep global weak body-z NHC disabled");
}

void TestPhase17BiasConsistentDvzConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase17_bias_consistent_dvz.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase17 config should use envelope constraints");
  ExpectTrue(!config.enable_vertical_envelope_center_pull, "phase17 should keep center pull disabled");
  ExpectTrue(config.enable_initial_static_rtk_height_reference, "phase17 should keep static RTK anchor");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase17 should keep fixed-axis body-z NHC enabled");
  ExpectTrue(
    config.enable_vertical_velocity_delta_bias_consistent_sigma,
    "phase17 should enable bias-consistent velocity delta sigma");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_bias_sigma_mps2 -
             offline_lc_minimal::MicroGToMps2(10.0)) < 1e-15,
    "phase17 velocity delta bias sigma should parse from ug");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_attitude_sigma_rad - 1.0e-4) < 1e-15,
    "phase17 attitude sigma should load");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_sigma_floor_mps - 1.0e-5) < 1e-15,
    "phase17 velocity delta sigma floor should load");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_sigma_ceiling_mps - 5.0e-4) < 1e-15,
    "phase17 velocity delta sigma ceiling should load");
}

void TestPhase18AttitudeRefBiasAwareDvzConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase18_attitude_ref_bias_aware_dvz.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase18 config should use envelope constraints");
  ExpectTrue(!config.enable_vertical_envelope_center_pull, "phase18 should keep gate-only center pull disabled");
  ExpectTrue(
    config.vertical_envelope_center_sigma_mode == offline_lc_minimal::VerticalEnvelopeCenterSigmaMode::kFixed,
    "phase18 should keep legacy fixed center sigma mode");
  ExpectTrue(config.enable_initial_static_rtk_height_reference, "phase18 should keep static RTK anchor");
  ExpectTrue(config.enable_vertical_velocity_delta_bias_consistent_sigma, "phase18 should keep bias-consistent sigma");
  ExpectTrue(config.enable_vertical_velocity_delta_bias_aware_target, "phase18 should enable bias-aware dvz target");
  ExpectTrue(
    config.enable_vertical_velocity_delta_initial_static_constraint,
    "phase18 should apply dvz constraints inside the static alignment window");
  ExpectTrue(
    std::abs(config.vertical_acc_bias_sigma_mps2 - offline_lc_minimal::MicroGToMps2(10.0)) < 1e-15,
    "phase18 dynamic vertical ba_z GM sigma should load from 10 ug");
  ExpectTrue(config.enable_attitude_reference_constraint, "phase18 should enable attitude reference constraints");
  ExpectTrue(
    std::abs(config.attitude_reference_sigma_rad - 0.01) < 1e-15,
    "phase18 attitude reference sigma should load");
  ExpectTrue(
    std::abs(config.attitude_reference_relative_yaw_sigma_rad - 0.01) < 1e-15,
    "phase18 relative yaw reference sigma should use default");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase18 should keep fixed-axis body-z NHC enabled");
}

void TestPhase19GateInsideRtkWeightConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase19_gate_inside_rtk_weight.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase19 config should use envelope constraints");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase19 should restore gate-inside RTK center pull");
  ExpectTrue(
    config.vertical_envelope_center_sigma_mode == offline_lc_minimal::VerticalEnvelopeCenterSigmaMode::kGateSigma,
    "phase19 center pull sigma should derive from gate sigma");
  ExpectTrue(
    std::abs(config.vertical_envelope_center_deadband_m) < 1e-15,
    "phase19 center pull should not use a deadband");
  ExpectTrue(config.enable_vertical_velocity_delta_initial_static_constraint, "phase19 should keep static dvz enabled");
  ExpectTrue(config.enable_attitude_reference_constraint, "phase19 should keep attitude reference constraints");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase19 should keep fixed-axis body-z NHC enabled");
  ExpectTrue(
    !config.enable_vertical_position_velocity_consistency_all_states,
    "phase19 should not enable all-state position-velocity consistency");
}

void TestPhase20PositionVelocityConsistentHeightConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase20_pv_consistent_height.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase20 config should use envelope constraints");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase20 should keep gate-inside RTK center pull");
  ExpectTrue(
    config.vertical_envelope_center_sigma_mode == offline_lc_minimal::VerticalEnvelopeCenterSigmaMode::kGateSigma,
    "phase20 center pull sigma should derive from gate sigma");
  ExpectTrue(
    config.enable_vertical_position_velocity_consistency_all_states,
    "phase20 should enable all-state position-velocity consistency");
  ExpectTrue(
    std::abs(config.vertical_position_velocity_consistency_sigma_m - 0.001) < 1e-15,
    "phase20 position-velocity consistency sigma should be 1 mm");
  ExpectTrue(
    !config.enable_vertical_position_velocity_window_consistency,
    "phase20 should not enable window position-velocity consistency");
  ExpectTrue(config.enable_vertical_velocity_delta_initial_static_constraint, "phase20 should keep static dvz enabled");
  ExpectTrue(config.enable_attitude_reference_constraint, "phase20 should keep attitude reference constraints");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase20 should keep fixed-axis body-z NHC enabled");
}

void TestPhase21PositionVelocityWindowConsistencyConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase21_pv_window_consistency.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase21 config should use envelope constraints");
  ExpectTrue(
    config.enable_vertical_position_velocity_consistency_all_states,
    "phase21 should keep adjacent all-state position-velocity consistency");
  ExpectTrue(
    std::abs(config.vertical_position_velocity_consistency_sigma_m - 0.0001) < 1e-15,
    "phase21 adjacent position-velocity consistency sigma should be 0.1 mm");
  ExpectTrue(
    config.enable_vertical_position_velocity_window_consistency,
    "phase21 should enable window position-velocity consistency");
  ExpectTrue(
    std::abs(config.vertical_position_velocity_window_s - 1.0) < 1e-15,
    "phase21 window duration should load");
  ExpectTrue(
    std::abs(config.vertical_position_velocity_window_stride_s - 0.5) < 1e-15,
    "phase21 window stride should load");
  ExpectTrue(
    std::abs(config.vertical_position_velocity_window_sigma_m - 0.0005) < 1e-15,
    "phase21 window sigma should load");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase21 should keep gate-inside RTK center pull");
  ExpectTrue(config.enable_attitude_reference_constraint, "phase21 should keep attitude reference constraints");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase21 should keep fixed-axis body-z NHC enabled");
}

void TestPhase22OneUgBiasStrengthConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase22_1ug_bias_strength.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase22 config should use envelope constraints");
  ExpectTrue(
    config.enable_vertical_position_velocity_consistency_all_states,
    "phase22 should keep adjacent all-state position-velocity consistency");
  ExpectTrue(
    config.enable_vertical_position_velocity_window_consistency,
    "phase22 should keep window position-velocity consistency");
  ExpectTrue(
    std::abs(config.vertical_acc_bias_sigma_mps2 - offline_lc_minimal::MicroGToMps2(1.0)) < 1e-15,
    "phase22 dynamic vertical ba_z GM sigma should load from 1 ug");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_bias_sigma_mps2 -
             offline_lc_minimal::MicroGToMps2(1.0)) < 1e-15,
    "phase22 velocity delta bias sigma should load from 1 ug");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase22 should keep gate-inside RTK center pull");
  ExpectTrue(config.enable_attitude_reference_constraint, "phase22 should keep attitude reference constraints");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase22 should keep fixed-axis body-z NHC enabled");
}

void TestPhase23PointOneUgBiasStrengthConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase23_0p1ug_bias_strength.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase23 config should use envelope constraints");
  ExpectTrue(
    config.enable_vertical_position_velocity_consistency_all_states,
    "phase23 should keep adjacent all-state position-velocity consistency");
  ExpectTrue(
    config.enable_vertical_position_velocity_window_consistency,
    "phase23 should keep window position-velocity consistency");
  ExpectTrue(
    std::abs(config.vertical_acc_bias_sigma_mps2 - offline_lc_minimal::MicroGToMps2(0.1)) < 1e-15,
    "phase23 dynamic vertical ba_z GM sigma should load from 0.1 ug");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_bias_sigma_mps2 -
             offline_lc_minimal::MicroGToMps2(0.1)) < 1e-15,
    "phase23 velocity delta bias sigma should load from 0.1 ug");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase23 should keep gate-inside RTK center pull");
  ExpectTrue(config.enable_attitude_reference_constraint, "phase23 should keep attitude reference constraints");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase23 should keep fixed-axis body-z NHC enabled");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_jump_padding_s - config.body_z_nhc_jump_padding_s) < 1e-12,
    "phase23 dvz jump padding should match body-z NHC jump padding");
}

void TestPhase24ThreeCentimeterRtkGateConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase24_3cm_rtk_gate.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase24 config should use envelope constraints");
  ExpectTrue(
    std::abs(config.gnss_vertical_fixed_sigma_m - 0.015) < 1e-15,
    "phase24 fixed RTK vertical sigma should be 1.5cm");
  ExpectTrue(
    std::abs(config.vertical_envelope_gate_sigma_multiple - 2.0) < 1e-15,
    "phase24 should keep a 2-sigma RTK gate");
  ExpectTrue(
    std::abs(config.vertical_envelope_min_half_width_m - 0.03) < 1e-15,
    "phase24 RTK gate minimum half-width should be 3cm");
  ExpectTrue(
    config.vertical_envelope_center_sigma_mode ==
      offline_lc_minimal::VerticalEnvelopeCenterSigmaMode::kGateSigma,
    "phase24 center pull sigma should derive from the 3cm gate");
  ExpectTrue(
    std::abs(config.vertical_envelope_center_deadband_m) < 1e-15,
    "phase24 center pull should not use a deadband");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase24 should keep gate-inside RTK center pull");
  ExpectTrue(config.enable_attitude_reference_constraint, "phase24 should keep attitude reference constraints");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase24 should keep fixed-axis body-z NHC enabled");
}

void TestPhase25StaticHoldTightenedConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase25_static_hold_tightened.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase25 config should use envelope constraints");
  ExpectTrue(
    std::abs(config.gnss_vertical_fixed_sigma_m - 0.015) < 1e-15,
    "phase25 should keep the 3cm RTK gate sigma");
  ExpectTrue(
    std::abs(config.vertical_envelope_min_half_width_m - 0.03) < 1e-15,
    "phase25 should keep the 3cm RTK gate minimum half-width");
  ExpectTrue(
    std::abs(config.initial_static_zupt_velocity_sigma_mps - 0.0005) < 1e-15,
    "phase25 should tighten static ZUPT velocity sigma");
  ExpectTrue(
    !config.enable_initial_static_vertical_position_hold,
    "phase25 should replace vertical-only static position hold");
  ExpectTrue(
    config.enable_initial_static_position_hold,
    "phase25 should enable full 3D static position hold");
  ExpectTrue(
    std::abs(config.initial_static_position_hold_sigma_m - 0.001) < 1e-15,
    "phase25 full static position hold sigma should be 1mm");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase25 should keep gate-inside RTK center pull");
  ExpectTrue(config.enable_attitude_reference_constraint, "phase25 should keep attitude reference constraints");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase25 should keep fixed-axis body-z NHC enabled");
}

void TestPhase26LeakageCorrectedNHCConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase26_leakage_corrected_nhc.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase26 config should use envelope constraints");
  ExpectTrue(config.enable_body_z_nhc_constraint, "phase26 should keep body-z NHC enabled");
  ExpectTrue(
    config.enable_body_z_nhc_global_weak_constraint,
    "phase26 should enable outside-window body-z NHC");
  ExpectTrue(
    config.enable_body_z_nhc_horizontal_leakage_correction,
    "phase26 should enable horizontal leakage correction");
  ExpectTrue(
    std::abs(config.body_z_nhc_global_velocity_sigma_mps - 0.02) < 1e-15,
    "phase26 outside-window velocity sigma should be 0.02m/s");
  ExpectTrue(
    std::abs(config.body_z_nhc_global_displacement_sigma_m - 0.02) < 1e-15,
    "phase26 outside-window displacement sigma should be 0.02m");
  ExpectTrue(
    std::abs(config.body_z_nhc_horizontal_leakage_min_speed_mps - 0.5) < 1e-15,
    "phase26 leakage min speed should load");
  ExpectTrue(
    config.body_z_nhc_horizontal_leakage_min_sample_count == 30,
    "phase26 leakage min sample count should load");
}

void TestPhase27AdaptiveMotionReweightConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase27_adaptive_motion_reweight.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase27 config should use envelope constraints");
  ExpectTrue(
    config.enable_vertical_motion_adaptive_reweighting,
    "phase27 should enable adaptive vertical motion reweighting");
  ExpectTrue(
    config.vertical_motion_adaptive_outer_iterations == 2,
    "phase27 should run two adaptive outer iterations");
  ExpectTrue(
    std::abs(
      config.vertical_motion_adaptive_static_dvz_bias_sigma_mps2 -
      offline_lc_minimal::MicroGToMps2(0.02)) < 1e-15,
    "phase27 static dvz bias sigma should parse in ug");
  ExpectTrue(
    std::abs(config.vertical_motion_adaptive_static_attitude_sigma_rad - 1e-5) < 1e-15,
    "phase27 static attitude sigma should load");
  ExpectTrue(
    std::abs(config.vertical_motion_adaptive_static_sigma_floor_mps - 2e-6) < 1e-15,
    "phase27 adaptive floor should load");
  ExpectTrue(
    std::abs(config.vertical_motion_adaptive_static_sigma_ceiling_mps - 5e-5) < 1e-15,
    "phase27 adaptive ceiling should load");
  ExpectTrue(
    std::abs(
      config.vertical_motion_adaptive_static_baz_gm_sigma_mps2 -
      offline_lc_minimal::MicroGToMps2(0.02)) < 1e-15,
    "phase27 static ba_z GM sigma should parse in ug");
  ExpectTrue(config.enable_body_z_nhc_horizontal_leakage_correction, "phase27 should keep leakage correction");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase27 should keep gate-inside RTK center pull");
}

void TestPhase30RtkDriftReferenceConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase30_rtk_drift_reference.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase30 config should use envelope constraints");
  ExpectTrue(config.enable_vertical_envelope_center_pull, "phase30 should keep center pull enabled");
  ExpectTrue(config.enable_rtk_vertical_drift_reference, "phase30 should enable RTK drift reference");
  ExpectTrue(
    std::abs(config.rtk_vertical_drift_correlation_time_s - 5.3) < 1e-15,
    "phase30 drift correlation time should load");
  ExpectTrue(
    std::abs(config.rtk_vertical_drift_sigma_m - 0.010) < 1e-15,
    "phase30 drift sigma should load");
  ExpectTrue(
    std::abs(config.rtk_vertical_white_noise_sigma_m - 0.002) < 1e-15,
    "phase30 white noise sigma should load");
  ExpectTrue(config.rtk_vertical_drift_outer_iterations == 20, "phase30 drift iterations should load");
  ExpectTrue(
    config.enable_rtk_vertical_drift_outage_segmentation,
    "phase30 should enable outage-aware drift segmentation");
  ExpectTrue(config.rtk_vertical_drift_use_for_center_pull, "phase30 should use drift for center pull");
  ExpectTrue(
    !config.rtk_vertical_drift_use_for_envelope_gate,
    "phase30 should preserve the raw RTK envelope gate");
}

void TestPhase31StrictNHCWeightConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase31_strict_nhc_weight.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase31 config should use envelope constraints");
  ExpectTrue(
    config.enable_body_z_nhc_strict_effective_weighting,
    "phase31 should enable strict Body-Z NHC effective weighting");
  ExpectTrue(
    std::abs(config.body_z_nhc_global_stride_s - config.body_z_nhc_global_window_s) < 1e-15,
    "phase31 global NHC stride should match window length");
  ExpectTrue(
    std::abs(config.body_z_nhc_global_velocity_sigma_mps - 0.005) < 1e-15,
    "phase31 global NHC velocity sigma should match jump strength");
  ExpectTrue(
    std::abs(config.body_z_nhc_global_displacement_sigma_m - 0.005) < 1e-15,
    "phase31 global NHC displacement sigma should match jump strength");
  ExpectTrue(config.enable_rtk_vertical_drift_reference, "phase31 should keep RTK drift reference");
  ExpectTrue(
    config.enable_body_z_nhc_horizontal_leakage_correction,
    "phase31 should keep leakage-corrected NHC");
}

void TestPhase32RtkOutageSmootherConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1cut1_vertical_envelope_phase32_rtk_outage_smoother.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.vertical_constraint_mode == offline_lc_minimal::VerticalConstraintMode::kEnvelope,
    "phase32 config should keep envelope constraints");
  ExpectTrue(config.drop_non_rtkfix, "phase32 should remove non-fixed GNSS factors");
  ExpectTrue(
    config.enable_rtk_outage_smoothing,
    "phase32 should enable RTK outage smoothing");
  ExpectTrue(
    config.enable_rtk_outage_baz_reestimate,
    "phase32 should enable RTK outage ba_z reestimate");
  ExpectTrue(
    config.enable_rtk_outage_boundary_constraints,
    "phase32 should enable RTK outage boundary constraints");
  ExpectTrue(
    config.rtk_outage_recovery_reference_min_fix_samples == 5,
    "phase32 recovery reference sample count should load");
  ExpectTrue(
    std::abs(config.rtk_outage_recovery_reference_max_duration_s - 2.0) < 1e-15,
    "phase32 recovery reference duration should load");
  ExpectTrue(
    std::abs(config.rtk_outage_boundary_up_sigma_m - 0.005) < 1e-15,
    "phase32 boundary up sigma should load");
  ExpectTrue(
    std::abs(config.rtk_outage_boundary_vz_sigma_mps - 0.02) < 1e-15,
    "phase32 boundary vz sigma should load");
  ExpectTrue(
    std::abs(offline_lc_minimal::Mps2ToMicroG(config.rtk_outage_boundary_baz_sigma_mps2) -
             50.0) < 1e-12,
    "phase32 boundary ba_z sigma should load in ug");
  ExpectTrue(
    std::abs(offline_lc_minimal::Mps2ToMicroG(
               config.rtk_outage_baz_continuity_break_delta_threshold_mps2) -
             1000.0) < 1e-9,
    "phase32 ba_z reset threshold should load in ug");
  ExpectTrue(
    std::abs(config.rtk_outage_min_gap_s - 2.0) < 1e-15,
    "phase32 RTK outage gap threshold should load");
  ExpectTrue(
    std::abs(config.rtk_outage_position_ramp_sigma_m - 0.05) < 1e-15,
    "phase32 RTK outage position ramp sigma should load");
  ExpectTrue(
    std::abs(config.rtk_outage_velocity_delta_sigma_mps - 0.02) < 1e-15,
    "phase32 RTK outage dvz sigma should load");
  ExpectTrue(config.enable_rtk_outage_attitude_hold, "phase32 should enable outage attitude hold");
  ExpectTrue(
    std::abs(config.rtk_outage_attitude_guard_duration_s - 1.0) < 1e-15,
    "phase32 outage attitude guard duration should load");
  ExpectTrue(
    std::abs(config.rtk_outage_absolute_attitude_sigma_rad - 1.0e-4) < 1e-15,
    "phase32 outage absolute attitude sigma should load");
  ExpectTrue(
    std::abs(config.rtk_outage_relative_attitude_sigma_rad - 1.0e-4) < 1e-15,
    "phase32 outage relative attitude sigma should load");
  ExpectTrue(config.enable_rtk_outage_velocity_delta_3d, "phase32 should enable outage 3D velocity delta");
  ExpectTrue(
    std::abs(config.rtk_outage_velocity_delta_3d_sigma_mps - 0.20) < 1e-15,
    "phase32 outage 3D velocity delta sigma should load");
  ExpectTrue(
    config.enable_rtk_vertical_drift_reference,
    "phase32 should keep RTK drift reference");
  ExpectTrue(
    config.enable_rtk_vertical_drift_outage_segmentation,
    "phase32 should keep outage-aware drift segmentation");
  ExpectTrue(
    config.enable_rtk_outage_causal_drift_reference,
    "phase32 should enable outage causal drift reference");
  ExpectTrue(
    config.enable_rtk_outage_preoutage_vertical_fence,
    "phase32 should enable pre-outage vertical fence");
  ExpectTrue(
    config.enable_rtk_outage_segmented_batch,
    "phase32 should enable outage segmented batch");
  ExpectTrue(
    config.rtk_outage_segmented_batch_max_outages == 1,
    "phase32 should segment the first outage by default");
  ExpectTrue(
    config.rtk_outage_segmented_batch_allow_vertical_boundary_jump,
    "phase32 should allow vertical boundary jumps between outage segments");
  ExpectTrue(
    config.rtk_outage_causal_reference_max_prefix_runs == 1,
    "phase32 should use one causal prefix run");
  ExpectTrue(
    std::abs(config.rtk_outage_preoutage_fence_stride_s - 0.05) < 1e-15,
    "phase32 pre-outage fence stride should load");
  ExpectTrue(
    std::abs(config.rtk_outage_preoutage_fence_up_sigma_m - 0.0005) < 1e-15,
    "phase32 pre-outage fence up sigma should load");
  ExpectTrue(
    std::abs(config.rtk_outage_preoutage_fence_vz_sigma_mps - 0.0005) < 1e-15,
    "phase32 pre-outage fence vz sigma should load");
  ExpectTrue(
    config.enable_body_z_nhc_strict_effective_weighting,
    "phase32 should keep strict Body-Z NHC weighting");
  ExpectTrue(
    std::abs(config.attitude_reference_relative_yaw_sigma_rad - 0.001) < 1e-15,
    "phase32 relative yaw reference sigma should load");
}

void TestDefaultOfflineConfigUsesV14SegmentedStage2() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) + "/config/default_offline.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(config.enable_stage1_yaw_refinement, "default config should refine Stage1 yaw");
  ExpectTrue(config.enable_stage2_velocity_optimization, "default config should enable segmented Stage2");
  ExpectTrue(
    config.enable_stage2_vehicle_nhc_constraint,
    "default config should enable Stage2 vehicle NHC constraints");
  ExpectTrue(
    !config.enable_stage3_vertical_reference_optimization,
    "default config should keep Stage3 vertical reference optimization disabled");
  ExpectTrue(
    !config.enable_stage2_lowfreq_vertical_reference_optimization,
    "default config should keep Stage2 lowfreq vertical reference optimization disabled");
  ExpectTrue(
    config.stage2_lowfreq_vertical_reference_source ==
      offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass,
    "default Stage2 lowfreq final source should be Stage2 lowpass");
  ExpectTrue(
    config.gnss_vertical_reference_source ==
      offline_lc_minimal::GnssVerticalReferenceSource::kRawRtk,
    "default GNSS vertical reference should use raw RTK");
  ExpectTrue(config.enable_rtk_vertical_drift_reference, "default config should use RTK drift reference");
  ExpectTrue(
    config.enable_rtk_vertical_drift_gate_weighting,
    "default config should use gate-aware RTK drift weighting");
  ExpectTrue(
    !config.enable_rtk_vertical_lowpass_reference,
    "default config should keep lowpass RTK reference disabled");
  ExpectTrue(
    config.enable_rtk_vertical_drift_outage_segmentation,
    "default config should use outage-aware RTK drift segmentation");
  ExpectTrue(
    config.enable_rtk_outage_causal_drift_reference,
    "default config should use causal RTK outage drift reference");
  ExpectTrue(
    config.enable_rtk_outage_preoutage_vertical_fence,
    "default config should use pre-outage vertical fence");
  ExpectTrue(
    config.enable_rtk_outage_segmented_batch,
    "default config should use RTK outage segmented batch");
  ExpectTrue(
    config.enable_body_z_nhc_strict_effective_weighting,
    "default config should use strict Body-Z NHC weighting");
  ExpectTrue(
    std::abs(config.body_z_nhc_global_velocity_sigma_mps - 0.005) < 1e-15,
    "default global NHC velocity sigma should match phase31");
  ExpectTrue(
    std::abs(config.body_z_nhc_global_stride_s - config.body_z_nhc_global_window_s) < 1e-15,
    "default global NHC windows should be non-overlapping");
  ExpectTrue(config.drop_non_rtkfix, "default config should remove non-fixed GNSS factors");
  ExpectTrue(config.enable_rtk_outage_smoothing, "default config should enable RTK outage smoothing");
  ExpectTrue(
    config.enable_rtk_outage_baz_reestimate,
    "default config should enable RTK outage ba_z reestimate");
  ExpectTrue(
    config.enable_rtk_outage_boundary_constraints,
    "default config should enable RTK outage boundary constraints");
  ExpectTrue(
    config.rtk_outage_recovery_reference_min_fix_samples == 5,
    "default recovery reference sample count should match phase32");
  ExpectTrue(
    std::abs(config.rtk_outage_recovery_reference_max_duration_s - 2.0) < 1e-15,
    "default recovery reference duration should match phase32");
  ExpectTrue(
    std::abs(config.rtk_outage_boundary_up_sigma_m - 0.005) < 1e-15,
    "default boundary up sigma should match phase32");
  ExpectTrue(
    std::abs(config.rtk_outage_boundary_vz_sigma_mps - 0.02) < 1e-15,
    "default boundary vz sigma should match phase32");
  ExpectTrue(
    std::abs(offline_lc_minimal::Mps2ToMicroG(config.rtk_outage_boundary_baz_sigma_mps2) -
             50.0) < 1e-12,
    "default boundary ba_z sigma should match phase32");
  ExpectTrue(
    std::abs(offline_lc_minimal::Mps2ToMicroG(
               config.rtk_outage_baz_continuity_break_delta_threshold_mps2) -
             1000.0) < 1e-9,
    "default ba_z continuity reset threshold should match phase32");
  ExpectTrue(config.enable_rtk_outage_attitude_hold, "default config should enable outage attitude hold");
  ExpectTrue(
    std::abs(config.rtk_outage_attitude_guard_duration_s - 1.0) < 1e-15,
    "default outage attitude guard duration should match phase32");
  ExpectTrue(
    std::abs(config.rtk_outage_absolute_attitude_sigma_rad - 1.0e-4) < 1e-15,
    "default outage absolute attitude sigma should match phase32");
  ExpectTrue(config.enable_rtk_outage_velocity_delta_3d, "default config should enable outage 3D velocity delta");
  ExpectTrue(
    std::abs(config.rtk_outage_velocity_delta_3d_sigma_mps - 0.20) < 1e-15,
    "default outage 3D velocity sigma should match v1.4 default");
}

void TestOldCompatibilityKeysAreRejected() {
  ExpectUnknownKey("enable_vertical_rtk_preintegration_feedback");
  ExpectUnknownKey("vertical_local_recovery_enabled");
  ExpectUnknownKey("vertical_inside_bias_window_s");
  ExpectUnknownKey("enable_nhc_jump_reference");
  ExpectUnknownKey("enable_reweighted_combined_imu_factor");
  ExpectUnknownKey("enable_body_z_seed_jump_windows");
  ExpectUnknownKey("enable_vertical_rtk_seed_pass");
  ExpectUnknownKey("enable_static_vertical_bias_carryover");
  ExpectUnknownKey("static_vertical_bias_carryover_sigma_mps2");
  ExpectUnknownKey("static_vertical_bias_carryover_vertical_gm_sigma_mps2");
}

void TestBodyZJumpDetectionFlagLoads() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "enable_body_z_jump_detection", "true");
  offline_lc_minimal::OverrideConfigField(config, "attitude_reference_relative_yaw_sigma_rad", "0.02");
  offline_lc_minimal::OverrideConfigField(config, "body_z_long_bias_min_duration_s", "12.5");
  ExpectTrue(config.enable_body_z_jump_detection, "new body-z detection flag should load");
  ExpectTrue(
    std::abs(config.attitude_reference_relative_yaw_sigma_rad - 0.02) < 1e-15,
    "relative yaw reference sigma should parse");
  ExpectTrue(
    std::abs(config.body_z_long_bias_min_duration_s - 12.5) < 1e-15,
    "long body-z bias window threshold should parse");
}

void TestBodyZRequiresGnssAfterOverrides() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_gnss = false;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("enable_body_z_jump_detection requires enable_gnss") !=
            std::string::npos;
  }
  ExpectTrue(threw, "body-z detection should be rejected when GNSS is disabled after overrides");
}

void TestVerticalVelocityDeltaConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical velocity delta settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity delta sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_min_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical velocity delta settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity delta min sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_jump_padding_s = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical velocity delta settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity delta jump padding should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_target_acc_limit_mps2 = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical velocity delta settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity delta target limit should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_bias_sigma_mps2 = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical velocity delta settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity delta bias sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_attitude_sigma_rad = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical velocity delta settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity delta attitude sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_sigma_floor_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical velocity delta settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity delta sigma floor should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_sigma_ceiling_mps = config.vertical_velocity_delta_sigma_floor_mps * 0.5;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("sigma ceiling") != std::string::npos;
  }
  ExpectTrue(threw, "velocity delta sigma ceiling below floor should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.vertical_position_velocity_consistency_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical position-velocity consistency settings") !=
            std::string::npos;
  }
  ExpectTrue(threw, "non-positive position-velocity consistency sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_position_velocity_window_consistency = true;
  config.vertical_position_velocity_window_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical position-velocity window consistency settings") !=
            std::string::npos;
  }
  ExpectTrue(threw, "non-positive position-velocity window sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_velocity_delta_constraint = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("requires enable_body_z_jump_detection") != std::string::npos;
  }
  ExpectTrue(threw, "velocity delta constraints should require body-z jump detection");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.attitude_reference_sigma_rad = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("attitude reference settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive attitude reference sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.attitude_reference_relative_yaw_sigma_rad = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("attitude reference settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive relative yaw reference sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_attitude_reference_constraint = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("attitude reference constraint") != std::string::npos;
  }
  ExpectTrue(threw, "attitude reference constraints should require body-z seed optimization");
}

void TestVerticalEnvelopeCenterPullConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_envelope_center_sigma_m = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical envelope settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive center pull sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.vertical_envelope_center_deadband_m = -1.0e-3;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical envelope settings") != std::string::npos;
  }
  ExpectTrue(threw, "negative center pull deadband should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_center_deadband_m = 0.10;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("center deadband") != std::string::npos;
  }
  ExpectTrue(threw, "center pull deadband should be smaller than the envelope minimum half-width");

  config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "vertical_envelope_center_sigma_mode", "gate_sigma");
  ExpectTrue(
    config.vertical_envelope_center_sigma_mode == offline_lc_minimal::VerticalEnvelopeCenterSigmaMode::kGateSigma,
    "gate-sigma center pull mode should parse");

  config = offline_lc_minimal::DefaultConfig();
  threw = false;
  try {
    offline_lc_minimal::OverrideConfigField(config, "vertical_envelope_center_sigma_mode", "bogus");
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical envelope center sigma mode") != std::string::npos;
  }
  ExpectTrue(threw, "unknown center pull sigma mode should be rejected");
}

void TestInitialStaticVerticalBiasConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_global_acc_bias = true;
  config.enable_vertical_acc_bias_gm_process = true;
  config.enable_initial_static_subgraph = true;
  config.enable_initial_static_vertical_bias_soft_prior = true;
  config.static_alignment_duration_s = 100.0;
  config.initial_static_vertical_bias_global_tie_sigma_mps2 = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("initial static vertical bias and position settings") !=
            std::string::npos;
  }
  ExpectTrue(threw, "non-positive static vertical bias sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_initial_static_subgraph = true;
  config.enable_initial_static_vertical_bias_soft_prior = true;
  config.static_alignment_duration_s = 100.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("global accelerometer bias") != std::string::npos;
  }
  ExpectTrue(threw, "static vertical bias soft prior should require global vertical GM bias mode");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_global_acc_bias = true;
  config.enable_vertical_acc_bias_gm_process = true;
  config.enable_initial_static_vertical_bias_soft_prior = true;
  config.static_alignment_duration_s = 100.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("initial static subgraph") != std::string::npos;
  }
  ExpectTrue(threw, "static vertical bias soft prior should require the static subgraph");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_initial_static_vertical_bias_gm_tightening = true;
  config.static_alignment_duration_s = 100.0;
  config.enable_initial_static_subgraph = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical GM bias process") != std::string::npos;
  }
  ExpectTrue(threw, "static GM tightening should require vertical GM mode");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_initial_static_vertical_position_hold = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("static_alignment_duration_s") != std::string::npos;
  }
  ExpectTrue(threw, "static position hold should require a static alignment duration");

  config = offline_lc_minimal::DefaultConfig();
  config.static_alignment_duration_s = 100.0;
  config.enable_initial_static_vertical_position_hold = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("initial static subgraph") != std::string::npos;
  }
  ExpectTrue(threw, "static position hold should require the static subgraph");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_initial_static_position_hold = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("static_alignment_duration_s") != std::string::npos;
  }
  ExpectTrue(threw, "full static position hold should require a static alignment duration");

  config = offline_lc_minimal::DefaultConfig();
  config.static_alignment_duration_s = 100.0;
  config.enable_initial_static_position_hold = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("initial static subgraph") != std::string::npos;
  }
  ExpectTrue(threw, "full static position hold should require the static subgraph");
}

void TestInitialStaticRtkHeightReferenceConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.static_alignment_duration_s = 100.0;
  config.enable_initial_static_rtk_height_reference = true;
  config.enable_gnss = true;
  config.enable_initial_static_subgraph = false;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("static RTK height reference") != std::string::npos;
  }
  ExpectTrue(threw, "static RTK height reference should require the static subgraph");

  config = offline_lc_minimal::DefaultConfig();
  config.static_alignment_duration_s = 100.0;
  config.enable_initial_static_subgraph = true;
  config.enable_initial_static_rtk_height_reference = true;
  config.initial_static_rtk_height_reference_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("initial static constraint settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive static RTK height reference sigma should be rejected");
}

void TestRtkVerticalLowpassReferenceConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.static_alignment_duration_s = 100.0;
  config.enable_gnss = true;
  config.enable_initial_static_subgraph = true;
  config.enable_initial_static_rtk_height_reference = true;
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.enable_vertical_envelope_center_pull = true;
  config.enable_rtk_vertical_drift_reference = true;
  config.rtk_vertical_drift_use_for_center_pull = true;
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_vertical_lowpass_reference", "true");
  offline_lc_minimal::OverrideConfigField(config, "rtk_vertical_lowpass_reference_cutoff_hz", "0.1");
  ExpectTrue(config.enable_rtk_vertical_lowpass_reference, "RTK vertical lowpass flag should parse");
  ExpectTrue(
    std::abs(config.rtk_vertical_lowpass_reference_cutoff_hz - 0.1) < 1e-15,
    "RTK vertical lowpass cutoff should parse");
  offline_lc_minimal::ValidateConfig(config);

  config.rtk_vertical_lowpass_reference_cutoff_hz = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK vertical drift reference settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive RTK vertical lowpass cutoff should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_vertical_lowpass_reference = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("lowpass reference") != std::string::npos;
  }
  ExpectTrue(threw, "lowpass reference should require drift center-pull reference");
}

void TestRtkVerticalDriftGateWeightingConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_vertical_drift_outage_segmentation", "false");
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_vertical_drift_gate_weighting", "false");
  offline_lc_minimal::OverrideConfigField(config, "rtk_vertical_drift_gate_weight_floor", "0.25");
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_outage_causal_drift_reference", "false");
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_outage_preoutage_vertical_fence", "false");
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_outage_segmented_batch", "false");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_segmented_batch_max_outages", "0");
  offline_lc_minimal::OverrideConfigField(
    config,
    "rtk_outage_segmented_batch_allow_vertical_boundary_jump",
    "false");
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_outage_boundary_constraints", "false");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_recovery_reference_min_fix_samples", "7");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_recovery_reference_max_duration_s", "3.5");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_boundary_up_sigma_m", "0.006");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_boundary_vz_sigma_mps", "0.03");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_boundary_baz_sigma_ug", "75");
  offline_lc_minimal::OverrideConfigField(
    config,
    "rtk_outage_baz_continuity_break_delta_threshold_ug",
    "1200");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_causal_reference_max_prefix_runs", "0");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_preoutage_fence_stride_s", "0.4");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_preoutage_fence_up_sigma_m", "0.004");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_preoutage_fence_vz_sigma_mps", "0.005");
  ExpectTrue(
    !config.enable_rtk_vertical_drift_outage_segmentation,
    "RTK vertical drift outage segmentation flag should parse");
  ExpectTrue(
    !config.enable_rtk_vertical_drift_gate_weighting,
    "RTK vertical drift gate weighting flag should parse");
  ExpectTrue(
    std::abs(config.rtk_vertical_drift_gate_weight_floor - 0.25) < 1e-15,
    "RTK vertical drift gate weight floor should parse");
  ExpectTrue(
    !config.enable_rtk_outage_causal_drift_reference,
    "causal RTK outage drift reference flag should parse");
  ExpectTrue(
    !config.enable_rtk_outage_preoutage_vertical_fence,
    "pre-outage vertical fence flag should parse");
  ExpectTrue(
    !config.enable_rtk_outage_segmented_batch,
    "outage segmented batch flag should parse");
  ExpectTrue(
    config.rtk_outage_segmented_batch_max_outages == 0,
    "outage segmented batch max outage count should parse");
  ExpectTrue(
    !config.rtk_outage_segmented_batch_allow_vertical_boundary_jump,
    "outage segmented batch boundary jump flag should parse");
  ExpectTrue(
    !config.enable_rtk_outage_boundary_constraints,
    "outage boundary constraint flag should parse");
  ExpectTrue(
    config.rtk_outage_recovery_reference_min_fix_samples == 7,
    "recovery reference min sample count should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_recovery_reference_max_duration_s - 3.5) < 1e-15,
    "recovery reference max duration should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_boundary_up_sigma_m - 0.006) < 1e-15,
    "boundary up sigma should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_boundary_vz_sigma_mps - 0.03) < 1e-15,
    "boundary vz sigma should parse");
  ExpectTrue(
    std::abs(offline_lc_minimal::Mps2ToMicroG(config.rtk_outage_boundary_baz_sigma_mps2) -
             75.0) < 1e-12,
    "boundary ba_z sigma should parse as ug");
  ExpectTrue(
    std::abs(offline_lc_minimal::Mps2ToMicroG(
               config.rtk_outage_baz_continuity_break_delta_threshold_mps2) -
             1200.0) < 1e-9,
    "boundary ba_z reset threshold should parse as ug");
  ExpectTrue(
    config.rtk_outage_causal_reference_max_prefix_runs == 0,
    "causal prefix run count should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_preoutage_fence_stride_s - 0.4) < 1e-15,
    "pre-outage fence stride should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_preoutage_fence_up_sigma_m - 0.004) < 1e-15,
    "pre-outage fence up sigma should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_preoutage_fence_vz_sigma_mps - 0.005) < 1e-15,
    "pre-outage fence vz sigma should parse");
  const std::string serialized = offline_lc_minimal::ConfigToString(config);
  ExpectTrue(
    serialized.find("enable_rtk_vertical_drift_outage_segmentation=false") != std::string::npos,
    "RTK vertical drift outage segmentation flag should be serialized");
  ExpectTrue(
    serialized.find("enable_rtk_vertical_drift_gate_weighting=false") != std::string::npos,
    "RTK vertical drift gate weighting flag should be serialized");
  ExpectTrue(
    serialized.find("rtk_vertical_drift_gate_weight_floor=0.25") != std::string::npos,
    "RTK vertical drift gate weight floor should be serialized");
  ExpectTrue(
    serialized.find("enable_rtk_outage_causal_drift_reference=false") != std::string::npos,
    "causal RTK outage drift reference flag should be serialized");
  ExpectTrue(
    serialized.find("enable_rtk_outage_preoutage_vertical_fence=false") != std::string::npos,
    "pre-outage vertical fence flag should be serialized");
  ExpectTrue(
    serialized.find("enable_rtk_outage_segmented_batch=false") != std::string::npos,
    "outage segmented batch flag should be serialized");
  ExpectTrue(
    serialized.find("rtk_outage_segmented_batch_max_outages=0") != std::string::npos,
    "outage segmented batch max outage count should be serialized");
  ExpectTrue(
    serialized.find("rtk_outage_segmented_batch_allow_vertical_boundary_jump=false") !=
      std::string::npos,
    "outage segmented batch boundary jump flag should be serialized");
  ExpectTrue(
    serialized.find("enable_rtk_outage_boundary_constraints=false") != std::string::npos,
    "outage boundary constraint flag should be serialized");
  ExpectTrue(
    serialized.find("rtk_outage_recovery_reference_min_fix_samples=7") != std::string::npos,
    "recovery reference min sample count should be serialized");
  ExpectTrue(
    serialized.find("rtk_outage_recovery_reference_max_duration_s=3.5") != std::string::npos,
    "recovery reference max duration should be serialized");
  ExpectTrue(
    serialized.find("rtk_outage_boundary_up_sigma_m=0.006") != std::string::npos,
    "boundary up sigma should be serialized");
  ExpectTrue(
    serialized.find("rtk_outage_boundary_vz_sigma_mps=0.03") != std::string::npos,
    "boundary vz sigma should be serialized");
  ExpectTrue(
    serialized.find("rtk_outage_boundary_baz_sigma_ug=75") != std::string::npos,
    "boundary ba_z sigma should be serialized in ug");
  ExpectTrue(
    serialized.find("rtk_outage_baz_continuity_break_delta_threshold_ug=1200") !=
      std::string::npos,
    "boundary ba_z reset threshold should be serialized in ug");
  ExpectTrue(
    serialized.find("rtk_outage_causal_reference_max_prefix_runs=0") != std::string::npos,
    "causal prefix run count should be serialized");
  ExpectTrue(
    serialized.find("rtk_outage_preoutage_fence_stride_s=0.4") != std::string::npos,
    "pre-outage fence stride should be serialized");
  offline_lc_minimal::ValidateConfig(config);

  config = offline_lc_minimal::DefaultConfig();
  config.rtk_vertical_drift_gate_weight_floor = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK vertical drift reference settings") != std::string::npos;
  }
  ExpectTrue(threw, "zero RTK vertical drift gate weight floor should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.rtk_vertical_drift_gate_weight_floor = 1.01;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK vertical drift reference settings") != std::string::npos;
  }
  ExpectTrue(threw, "RTK vertical drift gate weight floor above one should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_preoutage_fence_stride_s = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK vertical drift reference settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive pre-outage fence stride should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_causal_reference_max_prefix_runs = -1;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK vertical drift reference settings") != std::string::npos;
  }
  ExpectTrue(threw, "negative causal prefix run count should be rejected");
}

void TestStage3VerticalReferenceConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  ExpectTrue(
    !config.enable_stage3_vertical_reference_optimization,
    "Stage3 should be disabled by default");
  ExpectTrue(
    std::abs(config.stage3_vertical_reference_lowpass_cutoff_hz - 0.05) < 1e-15,
    "Stage3 default cutoff should be 0.05 Hz");
  ExpectTrue(
    std::abs(config.stage3_vertical_anchor_sigma_m - 0.015) < 1e-15,
    "Stage3 default anchor sigma should be 0.015 m");
  ExpectTrue(
    config.stage3_vertical_reference_constraint_mode ==
      offline_lc_minimal::Stage3VerticalReferenceConstraintMode::kGaussian,
    "Stage3 default constraint mode should be gaussian");
  ExpectTrue(
    std::abs(config.stage3_vertical_envelope_half_width_m - 0.008) < 1e-15,
    "Stage3 default envelope half-width should be 0.008 m");
  ExpectTrue(
    std::abs(config.stage3_vertical_envelope_sigma_m - 0.003) < 1e-15,
    "Stage3 default envelope sigma should be 0.003 m");
  ExpectTrue(
    config.enable_stage3_vertical_envelope_center_pull,
    "Stage3 envelope center pull should default on for experiment configs");
  ExpectTrue(
    std::abs(config.stage3_vertical_envelope_center_sigma_m - 0.006) < 1e-15,
    "Stage3 default envelope center sigma should be 0.006 m");
  ExpectTrue(
    std::abs(config.stage3_vertical_envelope_center_deadband_m - 0.002) < 1e-15,
    "Stage3 default envelope center deadband should be 0.002 m");
  ExpectTrue(
    config.stage3_disable_rtk_outage_segmented_batch,
    "Stage3 segmented-batch compatibility flag should default to true");

  offline_lc_minimal::OverrideConfigField(
    config,
    "enable_stage3_vertical_reference_optimization",
    "true");
  offline_lc_minimal::OverrideConfigField(
    config,
    "enable_stage2_velocity_optimization",
    "true");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage3_vertical_reference_lowpass_cutoff_hz",
    "0.03");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage3_vertical_anchor_sigma_m",
    "0.02");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage3_vertical_reference_constraint_mode",
    "envelope");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage3_vertical_envelope_half_width_m",
    "0.012");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage3_vertical_envelope_sigma_m",
    "0.004");
  offline_lc_minimal::OverrideConfigField(
    config,
    "enable_stage3_vertical_envelope_center_pull",
    "false");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage3_vertical_envelope_center_sigma_m",
    "0.007");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage3_vertical_envelope_center_deadband_m",
    "0.003");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage3_disable_rtk_outage_segmented_batch",
    "false");
  ExpectTrue(config.enable_stage3_vertical_reference_optimization, "Stage3 enable flag should parse");
  ExpectTrue(
    std::abs(config.stage3_vertical_reference_lowpass_cutoff_hz - 0.03) < 1e-15,
    "Stage3 cutoff should parse");
  ExpectTrue(
    std::abs(config.stage3_vertical_anchor_sigma_m - 0.02) < 1e-15,
    "Stage3 anchor sigma should parse");
  ExpectTrue(
    config.stage3_vertical_reference_constraint_mode ==
      offline_lc_minimal::Stage3VerticalReferenceConstraintMode::kEnvelope,
    "Stage3 envelope mode should parse");
  ExpectTrue(
    std::abs(config.stage3_vertical_envelope_half_width_m - 0.012) < 1e-15,
    "Stage3 envelope half-width should parse");
  ExpectTrue(
    std::abs(config.stage3_vertical_envelope_sigma_m - 0.004) < 1e-15,
    "Stage3 envelope sigma should parse");
  ExpectTrue(
    !config.enable_stage3_vertical_envelope_center_pull,
    "Stage3 envelope center-pull flag should parse");
  ExpectTrue(
    std::abs(config.stage3_vertical_envelope_center_sigma_m - 0.007) < 1e-15,
    "Stage3 envelope center sigma should parse");
  ExpectTrue(
    std::abs(config.stage3_vertical_envelope_center_deadband_m - 0.003) < 1e-15,
    "Stage3 envelope center deadband should parse");
  ExpectTrue(!config.stage3_disable_rtk_outage_segmented_batch, "Stage3 segmented flag should parse");
  const std::string serialized = offline_lc_minimal::ConfigToString(config);
  ExpectTrue(
    serialized.find("enable_stage3_vertical_reference_optimization=true") != std::string::npos,
    "Stage3 enable flag should be serialized");
  ExpectTrue(
    serialized.find("stage3_vertical_reference_lowpass_cutoff_hz=0.03") != std::string::npos,
    "Stage3 cutoff should be serialized");
  ExpectTrue(
    serialized.find("stage3_vertical_anchor_sigma_m=0.02") != std::string::npos,
    "Stage3 anchor sigma should be serialized");
  ExpectTrue(
    serialized.find("stage3_vertical_reference_constraint_mode=envelope") != std::string::npos,
    "Stage3 envelope mode should be serialized");
  ExpectTrue(
    serialized.find("stage3_vertical_envelope_half_width_m=0.012") != std::string::npos,
    "Stage3 envelope half-width should be serialized");
  ExpectTrue(
    serialized.find("stage3_vertical_envelope_sigma_m=0.004") != std::string::npos,
    "Stage3 envelope sigma should be serialized");
  ExpectTrue(
    serialized.find("enable_stage3_vertical_envelope_center_pull=false") != std::string::npos,
    "Stage3 envelope center-pull flag should be serialized");
  ExpectTrue(
    serialized.find("stage3_vertical_envelope_center_sigma_m=0.007") != std::string::npos,
    "Stage3 envelope center sigma should be serialized");
  ExpectTrue(
    serialized.find("stage3_vertical_envelope_center_deadband_m=0.003") != std::string::npos,
    "Stage3 envelope center deadband should be serialized");
  ExpectTrue(
    serialized.find("stage3_disable_rtk_outage_segmented_batch=false") != std::string::npos,
    "Stage3 segmented flag should be serialized");
  offline_lc_minimal::ValidateConfig(config);

  config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_lowpass_cutoff_hz = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("stage3 vertical reference settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive Stage3 cutoff should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_anchor_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("stage3 vertical reference settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive Stage3 anchor sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  threw = false;
  try {
    offline_lc_minimal::OverrideConfigField(
      config,
      "stage3_vertical_reference_constraint_mode",
      "invalid_mode");
  } catch (const std::runtime_error &exception) {
    threw =
      std::string(exception.what()).find("Stage3 vertical reference constraint mode") !=
      std::string::npos;
  }
  ExpectTrue(threw, "invalid Stage3 constraint mode should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_constraint_mode =
    offline_lc_minimal::Stage3VerticalReferenceConstraintMode::kEnvelope;
  config.stage3_vertical_envelope_half_width_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("stage3 vertical envelope settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive Stage3 envelope half-width should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_constraint_mode =
    offline_lc_minimal::Stage3VerticalReferenceConstraintMode::kEnvelope;
  config.stage3_vertical_envelope_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("stage3 vertical envelope settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive Stage3 envelope sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_constraint_mode =
    offline_lc_minimal::Stage3VerticalReferenceConstraintMode::kEnvelope;
  config.stage3_vertical_envelope_center_deadband_m =
    config.stage3_vertical_envelope_half_width_m;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("center deadband") != std::string::npos;
  }
  ExpectTrue(threw, "Stage3 center deadband should be smaller than the half-width");

  config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_envelope_half_width_m = 0.0;
  config.stage3_vertical_envelope_sigma_m = 0.0;
  config.stage3_vertical_envelope_center_deadband_m = 1.0;
  offline_lc_minimal::ValidateConfig(config);

  config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_constraint_mode =
    offline_lc_minimal::Stage3VerticalReferenceConstraintMode::kEnvelope;
  config.enable_stage3_vertical_envelope_center_pull = false;
  config.stage3_vertical_envelope_center_sigma_m = 0.0;
  config.stage3_vertical_envelope_center_deadband_m =
    config.stage3_vertical_envelope_half_width_m;
  offline_lc_minimal::ValidateConfig(config);

  config = offline_lc_minimal::DefaultConfig();
  config.enable_stage3_vertical_reference_optimization = true;
  config.enable_stage2_velocity_optimization = false;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw =
      std::string(exception.what()).find("requires enable_stage2_velocity_optimization") !=
      std::string::npos;
  }
  ExpectTrue(threw, "Stage3 should require Stage2 to be enabled");
}

void TestStage2LowfreqVerticalReferenceConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  ExpectTrue(
    !config.enable_stage2_lowfreq_vertical_reference_optimization,
    "Stage2 lowfreq vertical reference optimization should default off");
  ExpectTrue(
    std::abs(config.stage2_lowfreq_vertical_reference_cutoff_hz - 0.05) < 1e-15,
    "Stage2 lowfreq default cutoff should be 0.05 Hz");
  ExpectTrue(
    config.stage2_lowfreq_vertical_reference_source ==
      offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass,
    "Stage2 lowfreq default final source should be Stage2 lowpass");
  ExpectTrue(
    config.gnss_vertical_reference_source ==
      offline_lc_minimal::GnssVerticalReferenceSource::kRawRtk,
    "GNSS vertical reference should default to raw RTK");

  offline_lc_minimal::OverrideConfigField(
    config,
    "enable_stage2_lowfreq_vertical_reference_optimization",
    "true");
  offline_lc_minimal::OverrideConfigField(
    config,
    "enable_stage2_velocity_optimization",
    "true");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage2_lowfreq_vertical_reference_cutoff_hz",
    "0.03");
  offline_lc_minimal::OverrideConfigField(
    config,
    "stage2_lowfreq_vertical_reference_source",
    "stage2_lowpass");
  offline_lc_minimal::OverrideConfigField(
    config,
    "gnss_vertical_reference_source",
    "stage2_lowpass");
  ExpectTrue(
    config.enable_stage2_lowfreq_vertical_reference_optimization,
    "Stage2 lowfreq enable flag should parse");
  ExpectTrue(
    std::abs(config.stage2_lowfreq_vertical_reference_cutoff_hz - 0.03) < 1e-15,
    "Stage2 lowfreq cutoff should parse");
  ExpectTrue(
    config.stage2_lowfreq_vertical_reference_source ==
      offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass,
    "Stage2 lowfreq source should parse");
  ExpectTrue(
    config.gnss_vertical_reference_source ==
      offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass,
    "GNSS vertical reference source should parse");
  const std::string serialized = offline_lc_minimal::ConfigToString(config);
  ExpectTrue(
    serialized.find("enable_stage2_lowfreq_vertical_reference_optimization=true") != std::string::npos,
    "Stage2 lowfreq enable flag should serialize");
  ExpectTrue(
    serialized.find("stage2_lowfreq_vertical_reference_cutoff_hz=0.03") != std::string::npos,
    "Stage2 lowfreq cutoff should serialize");
  ExpectTrue(
    serialized.find("stage2_lowfreq_vertical_reference_source=stage2_lowpass") != std::string::npos,
    "Stage2 lowfreq source should serialize");
  ExpectTrue(
    serialized.find("gnss_vertical_reference_source=stage2_lowpass") != std::string::npos,
    "GNSS vertical reference source should serialize");
  offline_lc_minimal::ValidateConfig(config);

  config = offline_lc_minimal::DefaultConfig();
  config.stage2_lowfreq_vertical_reference_cutoff_hz = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw =
      std::string(exception.what()).find("stage2 lowfreq vertical reference settings") !=
      std::string::npos;
  }
  ExpectTrue(threw, "non-positive Stage2 lowfreq cutoff should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_stage2_velocity_optimization = true;
  config.enable_stage2_lowfreq_vertical_reference_optimization = true;
  config.stage2_lowfreq_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kRawRtk;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw =
      std::string(exception.what()).find("requires a non-raw final reference source") !=
      std::string::npos;
  }
  ExpectTrue(threw, "Stage2 lowfreq optimization should reject raw final source");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_stage2_velocity_optimization = true;
  config.enable_stage2_lowfreq_vertical_reference_optimization = true;
  config.enable_stage3_vertical_reference_optimization = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw =
      std::string(exception.what()).find("incompatible with Stage3") != std::string::npos;
  }
  ExpectTrue(threw, "Stage2 lowfreq optimization should reject simultaneous Stage3");

  config = offline_lc_minimal::DefaultConfig();
  config.gnss_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw =
      std::string(exception.what()).find("requires stage2 lowfreq vertical reference optimization") !=
      std::string::npos;
  }
  ExpectTrue(threw, "Stage2 lowpass GNSS source should require the wrapper flow");

  config = offline_lc_minimal::DefaultConfig();
  threw = false;
  try {
    offline_lc_minimal::OverrideConfigField(
      config,
      "stage2_lowfreq_vertical_reference_source",
      "bad_source");
  } catch (const std::runtime_error &exception) {
    threw =
      std::string(exception.what()).find("GNSS vertical reference source") !=
      std::string::npos;
  }
  ExpectTrue(threw, "invalid Stage2 lowfreq source should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.stage2_lowfreq_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kRtkDriftLowpass;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw =
      std::string(exception.what()).find("RTK drift lowpass GNSS vertical reference") !=
      std::string::npos;
  }
  ExpectTrue(threw, "RTK drift lowpass source should require lowpass reference settings");
}

void TestStage3ExperimentConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1rtkjumpcut1_stage3_stage2_lowpass_vertical_anchor.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.enable_stage3_vertical_reference_optimization,
    "Stage3 experiment config should enable Stage3");
  ExpectTrue(config.enable_stage2_velocity_optimization, "Stage3 experiment should keep Stage2 enabled");
  ExpectTrue(
    config.stage3_disable_rtk_outage_segmented_batch,
    "Stage3 experiment should disable segmented batch inside Stage3 pass");
  ExpectTrue(
    std::abs(config.stage3_vertical_reference_lowpass_cutoff_hz - 0.05) < 1e-15,
    "Stage3 experiment cutoff should be 0.05 Hz");
  ExpectTrue(
    std::abs(config.stage3_vertical_anchor_sigma_m - 0.015) < 1e-15,
    "Stage3 experiment anchor sigma should be 0.015 m");

  const auto envelope_config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1rtkjumpcut1_stage3_lowpass_vertical_envelope_gate.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    envelope_config.enable_stage3_vertical_reference_optimization,
    "Stage3 envelope-gate config should enable Stage3");
  ExpectTrue(
    envelope_config.stage3_vertical_reference_constraint_mode ==
      offline_lc_minimal::Stage3VerticalReferenceConstraintMode::kEnvelope,
    "Stage3 envelope-gate config should select envelope mode");
  ExpectTrue(
    std::abs(envelope_config.stage3_vertical_envelope_half_width_m - 0.008) < 1e-15,
    "Stage3 envelope-gate config should use 8 mm half-width");
  ExpectTrue(
    std::abs(envelope_config.stage3_vertical_envelope_sigma_m - 0.003) < 1e-15,
    "Stage3 envelope-gate config should use 3 mm envelope sigma");
  ExpectTrue(
    envelope_config.enable_stage3_vertical_envelope_center_pull,
    "Stage3 envelope-gate config should enable bounded center-pull");
}

void TestStage2LowfreqExperimentConfigLoads() {
  const auto config = offline_lc_minimal::LoadConfigFile(
    std::string(OFFLINE_LC_MINIMAL_SOURCE_DIR) +
      "/config/transformed1rtkjumpcut1_stage2_lowfreq_vertical_reference.cfg",
    offline_lc_minimal::DefaultConfig());
  ExpectTrue(
    config.enable_stage2_lowfreq_vertical_reference_optimization,
    "Stage2 lowfreq experiment should enable the wrapper");
  ExpectTrue(config.enable_stage2_velocity_optimization, "Stage2 lowfreq experiment should keep Stage2 enabled");
  ExpectTrue(
    config.enable_rtk_outage_segmented_batch,
    "Stage2 lowfreq experiment should keep segmented batch enabled");
  ExpectTrue(
    !config.enable_stage3_vertical_reference_optimization,
    "Stage2 lowfreq experiment should keep Stage3 disabled");
  ExpectTrue(
    config.stage2_lowfreq_vertical_reference_source ==
      offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass,
    "Stage2 lowfreq experiment should use Stage2 lowpass as final vertical reference");
  ExpectTrue(
    config.gnss_vertical_reference_source ==
      offline_lc_minimal::GnssVerticalReferenceSource::kRawRtk,
    "outer experiment config should start from raw RTK GNSS vertical reference");
  ExpectTrue(
    std::abs(config.stage2_lowfreq_vertical_reference_cutoff_hz - 0.05) < 1e-15,
    "Stage2 lowfreq experiment cutoff should be 0.05 Hz");
}

void TestAccelerometerBiasUgConfigParsing() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "vertical_acc_bias_sigma_ug", "10.0");
  offline_lc_minimal::OverrideConfigField(config, "vertical_velocity_delta_bias_sigma_ug", "12.0");
  offline_lc_minimal::OverrideConfigField(config, "global_acc_bias_tie_sigma_ug", "0.5");
  offline_lc_minimal::OverrideConfigField(config, "global_acc_bias_tie_sigma_xy_ug", "0.25");
  offline_lc_minimal::OverrideConfigField(config, "initial_static_vertical_bias_global_tie_sigma_ug", "0.05");
  offline_lc_minimal::OverrideConfigField(config, "initial_static_vertical_bias_gm_sigma_ug", "0.02");
  offline_lc_minimal::OverrideConfigField(config, "enable_vertical_velocity_delta_bias_aware_target", "true");
  offline_lc_minimal::OverrideConfigField(config, "enable_vertical_velocity_delta_initial_static_constraint", "true");

  ExpectTrue(
    std::abs(config.vertical_acc_bias_sigma_mps2 - offline_lc_minimal::MicroGToMps2(10.0)) < 1e-15,
    "vertical acc bias sigma should parse from ug");
  ExpectTrue(
    std::abs(config.vertical_velocity_delta_bias_sigma_mps2 -
             offline_lc_minimal::MicroGToMps2(12.0)) < 1e-15,
    "vertical velocity delta bias sigma should parse from ug");
  ExpectTrue(
    std::abs(config.global_acc_bias_tie_sigma_mps2 - offline_lc_minimal::MicroGToMps2(0.5)) < 1e-15,
    "global acc bias tie sigma should parse from ug");
  ExpectTrue(
    std::abs(config.global_acc_bias_tie_sigma_xy_mps2 - offline_lc_minimal::MicroGToMps2(0.25)) < 1e-15,
    "global planar acc bias tie sigma should parse from ug");
  ExpectTrue(
    std::abs(config.initial_static_vertical_bias_global_tie_sigma_mps2 -
             offline_lc_minimal::MicroGToMps2(0.05)) < 1e-15,
    "static vertical bias global tie sigma should parse from ug");
  ExpectTrue(
    std::abs(config.initial_static_vertical_bias_gm_sigma_mps2 -
             offline_lc_minimal::MicroGToMps2(0.02)) < 1e-15,
    "static vertical bias GM sigma should parse from ug");
  ExpectTrue(config.enable_vertical_velocity_delta_bias_aware_target, "bias-aware dvz target flag should parse");
  ExpectTrue(
    config.enable_vertical_velocity_delta_initial_static_constraint,
    "initial static dvz constraint flag should parse");
}

void TestVerticalJumpConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_jump_masked_imu = true;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump constraints require enable_body_z_jump_detection") !=
            std::string::npos;
  }
  ExpectTrue(threw, "vertical jump masked IMU should require body-z detection");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_jump_position_ramp_smoothing = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump constraints require enable_body_z_jump_detection") !=
            std::string::npos;
  }
  ExpectTrue(threw, "vertical jump position ramp should require body-z detection");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_jump_velocity_continuity = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump constraints require enable_body_z_jump_detection") !=
            std::string::npos;
  }
  ExpectTrue(threw, "vertical jump velocity continuity should require body-z detection");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_jump_velocity_context_mean = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump constraints require enable_body_z_jump_detection") !=
            std::string::npos;
  }
  ExpectTrue(threw, "vertical jump velocity context mean should require body-z detection");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_jump_context_mean_continuity = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump constraints require enable_body_z_jump_detection") !=
            std::string::npos;
  }
  ExpectTrue(threw, "vertical jump context mean continuity should require body-z detection");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_masked_imu = true;
  config.enable_segment_error_feedback = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("requires CombinedImuFactor mode") != std::string::npos;
  }
  ExpectTrue(threw, "vertical jump masked IMU should reject ImuFactor mode");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_impulse = true;
  config.enable_segment_error_feedback = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("requires CombinedImuFactor mode") != std::string::npos;
  }
  ExpectTrue(threw, "vertical jump impulse should reject ImuFactor mode");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_impulse = true;
  config.enable_vertical_jump_masked_imu = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("incompatible") != std::string::npos;
  }
  ExpectTrue(threw, "vertical jump impulse should be incompatible with legacy masked IMU mode");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_jump_impulse = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump constraints require enable_body_z_jump_detection") !=
            std::string::npos;
  }
  ExpectTrue(threw, "vertical jump impulse should require body-z detection");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_jump_bias = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump constraints require enable_body_z_jump_detection") !=
            std::string::npos;
  }
  ExpectTrue(threw, "vertical jump bias should require body-z detection");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.enable_segment_error_feedback = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("requires CombinedImuFactor mode") != std::string::npos;
  }
  ExpectTrue(threw, "vertical jump bias should reject ImuFactor mode");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.enable_vertical_jump_impulse = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("incompatible") != std::string::npos;
  }
  ExpectTrue(threw, "vertical jump bias should be incompatible with impulse mode");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.vertical_jump_bias_prior_sigma_mps2 = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive jump-bias prior sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.vertical_jump_bias_velocity_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive jump-bias velocity sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.vertical_jump_bias_position_velocity_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive jump-bias position-velocity sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.vertical_jump_bias_padding_s = -1.0e-3;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "negative jump-bias padding should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_segmented_bias = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("requires enable_vertical_jump_bias") != std::string::npos;
  }
  ExpectTrue(threw, "segmented jump bias should require jump-bias mode");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.enable_vertical_jump_segmented_bias = true;
  config.vertical_jump_segmented_bias_min_segment_s = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive segmented jump-bias minimum duration should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.enable_vertical_jump_segmented_bias = true;
  config.vertical_jump_segmented_bias_max_segments = 0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive segmented jump-bias max segments should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.enable_vertical_jump_segmented_bias = true;
  config.vertical_jump_bias_highfreq_sigma_scale = -1.0e-3;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "negative high-frequency jump-bias sigma scale should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.enable_vertical_jump_segmented_bias = true;
  config.vertical_jump_bias_highfreq_sigma_max_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive high-frequency jump-bias sigma cap should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_masked_imu = true;
  config.vertical_jump_masked_imu_padding_s = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive masked IMU padding should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_impulse = true;
  config.vertical_jump_impulse_prior_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive impulse prior sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_impulse = true;
  config.vertical_jump_impulse_velocity_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive impulse velocity sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_impulse = true;
  config.vertical_jump_impulse_position_velocity_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive impulse position-velocity sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_ramp_smoothing = true;
  config.vertical_jump_velocity_ramp_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive jump ramp sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_position_ramp_smoothing = true;
  config.vertical_jump_position_ramp_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive jump position ramp sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_ramp_smoothing = true;
  config.vertical_jump_velocity_height_slope_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity height slope sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_continuity = true;
  config.vertical_jump_velocity_continuity_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity continuity sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.vertical_jump_velocity_context_window_s = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity context window should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.vertical_jump_velocity_context_mean_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive velocity context sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.vertical_jump_velocity_context_window_s = std::numeric_limits<double>::quiet_NaN();
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "NaN velocity context window should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.vertical_jump_velocity_context_mean_sigma_mps = std::numeric_limits<double>::infinity();
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "infinite velocity context sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_context_mean_continuity = true;
  config.vertical_jump_context_mean_continuity_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive context mean continuity sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_context_mean_continuity = true;
  config.vertical_jump_context_mean_continuity_sigma_mps = std::numeric_limits<double>::infinity();
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "infinite context mean continuity sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_position_velocity_consistency = true;
  config.vertical_jump_position_velocity_consistency_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive position-velocity consistency sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_position_velocity_consistency = true;
  config.vertical_jump_boundary_position_velocity_consistency_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  ExpectTrue(threw, "non-positive boundary position-velocity consistency sigma should be rejected");
}

void TestVerticalJumpSpectralBiasRelaxationConfig() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "enable_body_z_jump_detection", "true");
  offline_lc_minimal::OverrideConfigField(config, "enable_vertical_jump_bias", "true");
  offline_lc_minimal::OverrideConfigField(config, "enable_vertical_jump_spectral_bias_relaxation", "true");
  offline_lc_minimal::OverrideConfigField(config, "vertical_jump_spectral_window_s", "2.5");
  offline_lc_minimal::OverrideConfigField(config, "vertical_jump_spectral_stride_s", "0.25");
  offline_lc_minimal::OverrideConfigField(config, "vertical_jump_spectral_reference_margin_s", "9.0");
  offline_lc_minimal::OverrideConfigField(config, "vertical_jump_spectral_min_reference_window_count", "4");
  offline_lc_minimal::OverrideConfigField(config, "vertical_jump_spectral_response_trigger_ratio", "1.3");
  offline_lc_minimal::OverrideConfigField(config, "vertical_jump_spectral_response_full_ratio", "2.8");
  offline_lc_minimal::OverrideConfigField(config, "vertical_jump_spectral_bias_prior_max_sigma_mps2", "0.35");

  ExpectTrue(config.enable_vertical_jump_spectral_bias_relaxation, "spectral relaxation flag should parse");
  ExpectTrue(
    std::abs(config.vertical_jump_spectral_window_s - 2.5) < 1e-12,
    "spectral window should parse");
  ExpectTrue(
    std::abs(config.vertical_jump_spectral_stride_s - 0.25) < 1e-12,
    "spectral stride should parse");
  ExpectTrue(
    config.vertical_jump_spectral_min_reference_window_count == 4,
    "spectral minimum reference count should parse");
  offline_lc_minimal::ValidateConfig(config);

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_spectral_bias_relaxation = true;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("requires enable_vertical_jump_bias") != std::string::npos;
  }
  ExpectTrue(threw, "spectral relaxation should require jump-local bias mode");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.enable_vertical_jump_spectral_bias_relaxation = true;
  config.vertical_jump_spectral_response_full_ratio = 1.1;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical jump settings") != std::string::npos;
  }
  ExpectTrue(threw, "spectral full-response ratio must exceed trigger ratio");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_bias = true;
  config.enable_vertical_jump_spectral_bias_relaxation = true;
  config.vertical_jump_bias_prior_sigma_mps2 = 0.40;
  config.vertical_jump_spectral_bias_prior_max_sigma_mps2 = 0.30;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("spectral bias prior max sigma") != std::string::npos;
  }
  ExpectTrue(threw, "spectral max prior sigma should not be below the base prior sigma");
}

void TestBodyZNHCConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "enable_body_z_nhc_constraint", "true");
  offline_lc_minimal::OverrideConfigField(config, "enable_body_z_nhc_global_weak_constraint", "true");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_jump_padding_s", "0.30");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_merge_gap_s", "0.20");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_min_window_s", "0.50");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_jump_velocity_sigma_mps", "0.02");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_jump_displacement_sigma_m", "0.02");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_global_window_s", "3.0");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_global_stride_s", "1.0");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_global_velocity_sigma_mps", "0.05");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_global_displacement_sigma_m", "0.05");
  offline_lc_minimal::OverrideConfigField(config, "enable_body_z_nhc_horizontal_leakage_correction", "true");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_horizontal_leakage_min_speed_mps", "0.5");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_horizontal_leakage_min_sample_count", "30");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_horizontal_leakage_huber_sigma_mps", "0.02");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_horizontal_leakage_max_abs_coeff_rad", "0.02");
  offline_lc_minimal::OverrideConfigField(config, "body_z_nhc_horizontal_leakage_guard_s", "0.30");
  ExpectTrue(config.enable_body_z_nhc_constraint, "body-z NHC flag should load");
  ExpectTrue(config.enable_body_z_nhc_global_weak_constraint, "body-z NHC global flag should load");
  ExpectTrue(
    config.enable_body_z_nhc_horizontal_leakage_correction,
    "body-z horizontal leakage flag should load");
  ExpectTrue(std::abs(config.body_z_nhc_global_window_s - 3.0) < 1e-12, "global NHC window should load");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_constraint = true;
  config.enable_body_z_jump_detection = false;
  config.enable_body_z_nhc_global_weak_constraint = false;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("jump-only body-z NHC") != std::string::npos;
  }
  ExpectTrue(threw, "jump-only body-z NHC should require jump detection");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_global_weak_constraint = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("requires enable_body_z_nhc_constraint") != std::string::npos;
  }
  ExpectTrue(threw, "global body-z NHC should require the main NHC flag");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_horizontal_leakage_correction = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("requires enable_body_z_nhc_constraint") != std::string::npos;
  }
  ExpectTrue(threw, "body-z horizontal leakage should require the main NHC flag");

  config = offline_lc_minimal::DefaultConfig();
  config.body_z_nhc_horizontal_leakage_huber_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("horizontal leakage settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive horizontal leakage sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.body_z_nhc_jump_velocity_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("body-z NHC settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive body-z NHC velocity sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_constraint = true;
  config.enable_body_z_nhc_global_weak_constraint = true;
  config.enable_body_z_nhc_strict_effective_weighting = true;
  config.body_z_nhc_global_window_s = 3.0;
  config.body_z_nhc_global_stride_s = 1.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("strict body-z NHC effective weighting") != std::string::npos;
  }
  ExpectTrue(threw, "strict body-z NHC should reject overlapping global windows");
}

void TestRtkVelocityConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "processing_start_time_s", "5996.988");
  offline_lc_minimal::OverrideConfigField(config, "processing_end_time_s", "6006.988");
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_velocity_constraint", "true");
  offline_lc_minimal::OverrideConfigField(config, "rtk_velocity_window_s", "1.5");
  offline_lc_minimal::OverrideConfigField(config, "rtk_velocity_horizontal_sigma_mps", "0.25");
  ExpectTrue(
    std::abs(config.processing_start_time_s - 5996.988) < 1e-12,
    "processing start override should parse");
  ExpectTrue(
    std::abs(config.processing_end_time_s - 6006.988) < 1e-12,
    "processing end override should parse");
  ExpectTrue(config.enable_rtk_velocity_constraint, "RTK velocity constraint flag should parse");
  ExpectTrue(std::abs(config.rtk_velocity_window_s - 1.5) < 1e-12, "RTK velocity window should parse");
  ExpectTrue(
    std::abs(config.rtk_velocity_horizontal_sigma_mps - 0.25) < 1e-12,
    "RTK velocity sigma should parse");

  config = offline_lc_minimal::DefaultConfig();
  config.processing_start_time_s = -1.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("processing_start_time_s") != std::string::npos;
  }
  ExpectTrue(threw, "negative processing start time should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.processing_end_time_s = -1.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("processing_end_time_s") != std::string::npos;
  }
  ExpectTrue(threw, "negative processing end time should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.processing_start_time_s = 5.0;
  config.processing_end_time_s = 5.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("processing_start_time_s") != std::string::npos;
  }
  ExpectTrue(threw, "processing end should be after processing start");

  config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_velocity_constraint = true;
  config.rtk_velocity_window_s = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK velocity constraint settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive RTK velocity window should be rejected");
}

void TestRtkOutageRecoveryConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_outage_attitude_hold", "true");
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_outage_baz_reestimate", "true");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_attitude_guard_duration_s", "1.5");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_absolute_attitude_sigma_rad", "2e-4");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_relative_attitude_sigma_rad", "3e-4");
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_outage_velocity_delta_3d", "true");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_velocity_delta_3d_sigma_mps", "0.35");
  offline_lc_minimal::OverrideConfigField(config, "enable_rtk_outage_boundary_constraints", "true");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_recovery_reference_min_fix_samples", "6");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_recovery_reference_max_duration_s", "1.5");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_boundary_up_sigma_m", "0.007");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_boundary_vz_sigma_mps", "0.025");
  offline_lc_minimal::OverrideConfigField(config, "rtk_outage_boundary_baz_sigma_ug", "60");
  offline_lc_minimal::OverrideConfigField(
    config,
    "rtk_outage_baz_continuity_break_delta_threshold_ug",
    "1100");
  offline_lc_minimal::ValidateConfig(config);
  ExpectTrue(config.enable_rtk_outage_attitude_hold, "outage attitude hold flag should parse");
  ExpectTrue(config.enable_rtk_outage_baz_reestimate, "outage ba_z reestimate flag should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_attitude_guard_duration_s - 1.5) < 1e-12,
    "outage attitude guard duration should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_absolute_attitude_sigma_rad - 2e-4) < 1e-12,
    "outage absolute attitude sigma should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_relative_attitude_sigma_rad - 3e-4) < 1e-12,
    "outage relative attitude sigma should parse");
  ExpectTrue(config.enable_rtk_outage_velocity_delta_3d, "outage 3D velocity flag should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_velocity_delta_3d_sigma_mps - 0.35) < 1e-12,
    "outage 3D velocity sigma should parse");
  ExpectTrue(config.enable_rtk_outage_boundary_constraints,
             "outage boundary constraint flag should parse");
  ExpectTrue(config.rtk_outage_recovery_reference_min_fix_samples == 6,
             "recovery reference min samples should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_recovery_reference_max_duration_s - 1.5) < 1e-12,
    "recovery reference duration should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_boundary_up_sigma_m - 0.007) < 1e-12,
    "boundary up sigma should parse");
  ExpectTrue(
    std::abs(config.rtk_outage_boundary_vz_sigma_mps - 0.025) < 1e-12,
    "boundary vz sigma should parse");
  ExpectTrue(
    std::abs(offline_lc_minimal::Mps2ToMicroG(config.rtk_outage_boundary_baz_sigma_mps2) -
             60.0) < 1e-12,
    "boundary ba_z sigma should parse");
  ExpectTrue(
    std::abs(offline_lc_minimal::Mps2ToMicroG(
               config.rtk_outage_baz_continuity_break_delta_threshold_mps2) -
             1100.0) < 1e-9,
    "boundary ba_z continuity threshold should parse");

  config.rtk_outage_absolute_attitude_sigma_rad = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK outage smoothing settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive outage attitude sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_attitude_guard_duration_s = -1.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK outage smoothing settings") != std::string::npos;
  }
  ExpectTrue(threw, "negative outage attitude guard duration should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_velocity_delta_3d_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK outage smoothing settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive outage 3D velocity sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_segmented_batch_max_outages = -1;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK outage segmented batch") != std::string::npos;
  }
  ExpectTrue(threw, "negative segmented batch outage count should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_recovery_reference_min_fix_samples = 0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK outage boundary constraint settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive recovery min sample count should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_boundary_baz_sigma_mps2 = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("RTK outage boundary constraint settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive boundary ba_z sigma should be rejected");
}

void TestInitialYawOverrideConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "enable_initial_yaw_override", "true");
  offline_lc_minimal::OverrideConfigField(config, "initial_yaw_override_rad", "1.25");
  ExpectTrue(config.enable_initial_yaw_override, "initial yaw override flag should parse");
  ExpectTrue(
    std::abs(config.initial_yaw_override_rad - 1.25) < 1e-12,
    "initial yaw override angle should parse");

  config.initial_yaw_override_rad = std::numeric_limits<double>::quiet_NaN();
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("initial yaw override") != std::string::npos;
  }
  ExpectTrue(threw, "enabled initial yaw override should reject non-finite yaw");
}

void TestStage1YawRefinementConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "enable_stage1_yaw_refinement", "true");
  offline_lc_minimal::OverrideConfigField(config, "stage1_yaw_refinement_max_iterations", "8");
  offline_lc_minimal::OverrideConfigField(config, "stage1_heading_window_s", "1.5");
  offline_lc_minimal::OverrideConfigField(config, "stage1_heading_time_tolerance_s", "0.15");
  offline_lc_minimal::OverrideConfigField(config, "stage1_heading_min_displacement_m", "0.30");
  offline_lc_minimal::OverrideConfigField(config, "stage1_heading_noise_floor_rad", "0.01");
  offline_lc_minimal::OverrideConfigField(config, "stage1_yaw_update_max_rad", "1.0");
  offline_lc_minimal::OverrideConfigField(config, "enable_stage1_outage_body_y_envelope", "true");
  offline_lc_minimal::OverrideConfigField(config, "stage1_outage_body_y_pre_window_s", "45");
  offline_lc_minimal::OverrideConfigField(config, "stage1_outage_body_y_deadband_rmse_multiplier", "2.5");
  offline_lc_minimal::OverrideConfigField(config, "stage1_outage_body_y_min_sample_count", "42");
  offline_lc_minimal::OverrideConfigField(config, "stage1_outage_body_y_min_speed_mps", "0.6");
  offline_lc_minimal::OverrideConfigField(config, "stage1_outage_body_y_min_sigma_mps", "0.02");
  offline_lc_minimal::OverrideConfigField(config, "stage1_outage_body_y_max_sigma_mps", "0.09");
  offline_lc_minimal::OverrideConfigField(config, "stage1_outage_body_y_huber_k", "1.6");
  offline_lc_minimal::OverrideConfigField(config, "enable_late_static_detection", "true");
  offline_lc_minimal::OverrideConfigField(config, "late_static_window_s", "6.0");
  offline_lc_minimal::OverrideConfigField(config, "late_static_stride_s", "0.75");
  offline_lc_minimal::OverrideConfigField(config, "late_static_min_duration_s", "9.0");
  offline_lc_minimal::OverrideConfigField(config, "late_static_threshold_method", "log_otsu");
  offline_lc_minimal::OverrideConfigField(config, "late_static_min_rtkfix_samples", "12");
  offline_lc_minimal::OverrideConfigField(config, "late_static_merge_gap_s", "1.5");
  offline_lc_minimal::OverrideConfigField(config, "late_static_exclude_initial_static", "false");
  offline_lc_minimal::OverrideConfigField(config, "late_static_exclude_rtk_outage", "true");
  offline_lc_minimal::OverrideConfigField(config, "late_static_vz_sigma_mps", "0.003");
  offline_lc_minimal::OverrideConfigField(config, "late_static_up_sigma_m", "0.025");
  offline_lc_minimal::OverrideConfigField(config, "late_static_height_hold_sigma_m", "0.004");
  offline_lc_minimal::ValidateConfig(config);
  ExpectTrue(config.enable_stage1_yaw_refinement, "stage1 yaw refinement flag should parse");
  ExpectTrue(config.stage1_yaw_refinement_max_iterations == 8, "stage1 max iterations should parse");
  ExpectTrue(std::abs(config.stage1_heading_window_s - 1.5) < 1e-12, "stage1 heading window should parse");
  ExpectTrue(
    std::abs(config.stage1_heading_time_tolerance_s - 0.15) < 1e-12,
    "stage1 heading time tolerance should parse");
  ExpectTrue(
    std::abs(config.stage1_heading_min_displacement_m - 0.30) < 1e-12,
    "stage1 heading displacement threshold should parse");
  ExpectTrue(
    std::abs(config.stage1_heading_noise_floor_rad - 0.01) < 1e-12,
    "stage1 heading noise floor should parse");
  ExpectTrue(
    std::abs(config.stage1_yaw_update_max_rad - 1.0) < 1e-12,
    "stage1 yaw update cap should parse");
  ExpectTrue(config.enable_stage1_outage_body_y_envelope,
             "stage1 outage body-y envelope flag should parse");
  ExpectTrue(std::abs(config.stage1_outage_body_y_pre_window_s - 45.0) < 1e-12,
             "stage1 body-y PRE window should parse");
  ExpectTrue(
    std::abs(config.stage1_outage_body_y_deadband_rmse_multiplier - 2.5) < 1e-12,
    "stage1 body-y deadband multiplier should parse");
  ExpectTrue(config.stage1_outage_body_y_min_sample_count == 42,
             "stage1 body-y min sample count should parse");
  ExpectTrue(std::abs(config.stage1_outage_body_y_min_speed_mps - 0.6) < 1e-12,
             "stage1 body-y min speed should parse");
  ExpectTrue(std::abs(config.stage1_outage_body_y_min_sigma_mps - 0.02) < 1e-12,
             "stage1 body-y min sigma should parse");
  ExpectTrue(std::abs(config.stage1_outage_body_y_max_sigma_mps - 0.09) < 1e-12,
             "stage1 body-y max sigma should parse");
  ExpectTrue(std::abs(config.stage1_outage_body_y_huber_k - 1.6) < 1e-12,
             "stage1 body-y Huber k should parse");
  ExpectTrue(config.enable_late_static_detection,
             "late-static detector flag should parse");
  ExpectTrue(std::abs(config.late_static_window_s - 6.0) < 1e-12,
             "late-static window should parse");
  ExpectTrue(std::abs(config.late_static_stride_s - 0.75) < 1e-12,
             "late-static stride should parse");
  ExpectTrue(std::abs(config.late_static_min_duration_s - 9.0) < 1e-12,
             "late-static min duration should parse");
  ExpectTrue(config.late_static_threshold_method == "log_otsu",
             "late-static threshold method should parse");
  ExpectTrue(config.late_static_min_rtkfix_samples == 12,
             "late-static min RTKFIX sample count should parse");
  ExpectTrue(std::abs(config.late_static_merge_gap_s - 1.5) < 1e-12,
             "late-static merge gap should parse");
  ExpectTrue(!config.late_static_exclude_initial_static,
             "late-static initial-static exclusion flag should parse");
  ExpectTrue(config.late_static_exclude_rtk_outage,
             "late-static RTK outage exclusion flag should parse");
  ExpectTrue(std::abs(config.late_static_vz_sigma_mps - 0.003) < 1e-12,
             "late-static vz sigma should parse");
  ExpectTrue(std::abs(config.late_static_up_sigma_m - 0.025) < 1e-12,
             "late-static up sigma should parse");
  ExpectTrue(std::abs(config.late_static_height_hold_sigma_m - 0.004) < 1e-12,
             "late-static height hold sigma should parse");

  config = offline_lc_minimal::DefaultConfig();
  config.stage1_yaw_refinement_max_iterations = 0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("stage1 yaw refinement settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive stage1 iteration count should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.stage1_heading_window_s = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("stage1 yaw refinement settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive stage1 heading window should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.stage1_heading_noise_floor_rad = std::numeric_limits<double>::infinity();
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("stage1 yaw refinement settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-finite stage1 noise floor should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.stage1_outage_body_y_max_sigma_mps =
    config.stage1_outage_body_y_min_sigma_mps * 0.5;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("stage1 outage body-y envelope") != std::string::npos;
  }
  ExpectTrue(threw, "stage1 body-y max sigma below min sigma should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.late_static_threshold_method = "fixed";
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("late_static_threshold_method") != std::string::npos;
  }
  ExpectTrue(threw, "unsupported late-static threshold method should be rejected");

  config = offline_lc_minimal::DefaultConfig();
  config.late_static_min_rtkfix_samples = 1;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("late-static detection settings") != std::string::npos;
  }
  ExpectTrue(threw, "late-static min RTKFIX sample count should be rejected below two");

  config = offline_lc_minimal::DefaultConfig();
  config.late_static_height_hold_sigma_m = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("late-static detection settings") != std::string::npos;
  }
  ExpectTrue(threw, "late-static height hold sigma should be positive");
}

}  // namespace

int main() {
  try {
    RunTest("TestDirectZSmokeConfigLoads", TestDirectZSmokeConfigLoads);
    RunTest("TestEnvelopeSmokeConfigLoads", TestEnvelopeSmokeConfigLoads);
    RunTest("TestPhase3SmokeConfigLoads", TestPhase3SmokeConfigLoads);
    RunTest("TestPhase4SmokeConfigLoads", TestPhase4SmokeConfigLoads);
    RunTest("TestPhase5SmokeConfigLoads", TestPhase5SmokeConfigLoads);
    RunTest("TestPhase6SmokeConfigLoads", TestPhase6SmokeConfigLoads);
    RunTest("TestPhase7SmokeConfigLoads", TestPhase7SmokeConfigLoads);
    RunTest("TestPhase7TightDvzConfigLoads", TestPhase7TightDvzConfigLoads);
    RunTest("TestPhase8ImpulseConfigLoads", TestPhase8ImpulseConfigLoads);
    RunTest("TestPhase9JumpBiasConfigLoads", TestPhase9JumpBiasConfigLoads);
    RunTest("TestPhase10SegmentedJumpBiasConfigLoads", TestPhase10SegmentedJumpBiasConfigLoads);
    RunTest("TestPhase12RtkGateOnlyFullNavConfigLoads", TestPhase12RtkGateOnlyFullNavConfigLoads);
    RunTest("TestPhase15StaticBazGmTightenedConfigLoads", TestPhase15StaticBazGmTightenedConfigLoads);
    RunTest("TestPhase16StaticRtkHeightAnchorConfigLoads", TestPhase16StaticRtkHeightAnchorConfigLoads);
    RunTest("TestPhase17BiasConsistentDvzConfigLoads", TestPhase17BiasConsistentDvzConfigLoads);
    RunTest("TestPhase18AttitudeRefBiasAwareDvzConfigLoads", TestPhase18AttitudeRefBiasAwareDvzConfigLoads);
    RunTest("TestPhase19GateInsideRtkWeightConfigLoads", TestPhase19GateInsideRtkWeightConfigLoads);
    RunTest(
      "TestPhase20PositionVelocityConsistentHeightConfigLoads",
      TestPhase20PositionVelocityConsistentHeightConfigLoads);
    RunTest(
      "TestPhase21PositionVelocityWindowConsistencyConfigLoads",
      TestPhase21PositionVelocityWindowConsistencyConfigLoads);
    RunTest(
      "TestPhase22OneUgBiasStrengthConfigLoads",
      TestPhase22OneUgBiasStrengthConfigLoads);
    RunTest(
      "TestPhase23PointOneUgBiasStrengthConfigLoads",
      TestPhase23PointOneUgBiasStrengthConfigLoads);
    RunTest(
      "TestPhase24ThreeCentimeterRtkGateConfigLoads",
      TestPhase24ThreeCentimeterRtkGateConfigLoads);
    RunTest(
      "TestPhase25StaticHoldTightenedConfigLoads",
      TestPhase25StaticHoldTightenedConfigLoads);
    RunTest(
      "TestPhase26LeakageCorrectedNHCConfigLoads",
      TestPhase26LeakageCorrectedNHCConfigLoads);
    RunTest(
      "TestPhase27AdaptiveMotionReweightConfigLoads",
      TestPhase27AdaptiveMotionReweightConfigLoads);
    RunTest(
      "TestPhase30RtkDriftReferenceConfigLoads",
      TestPhase30RtkDriftReferenceConfigLoads);
    RunTest(
      "TestPhase31StrictNHCWeightConfigLoads",
      TestPhase31StrictNHCWeightConfigLoads);
    RunTest(
      "TestPhase32RtkOutageSmootherConfigLoads",
      TestPhase32RtkOutageSmootherConfigLoads);
    RunTest(
      "TestDefaultOfflineConfigUsesV14SegmentedStage2",
      TestDefaultOfflineConfigUsesV14SegmentedStage2);
    RunTest(
      "TestStage3VerticalReferenceConfigValidation",
      TestStage3VerticalReferenceConfigValidation);
    RunTest(
      "TestStage2LowfreqVerticalReferenceConfigValidation",
      TestStage2LowfreqVerticalReferenceConfigValidation);
    RunTest(
      "TestStage3ExperimentConfigLoads",
      TestStage3ExperimentConfigLoads);
    RunTest(
      "TestStage2LowfreqExperimentConfigLoads",
      TestStage2LowfreqExperimentConfigLoads);
    RunTest("TestOldCompatibilityKeysAreRejected", TestOldCompatibilityKeysAreRejected);
    RunTest("TestBodyZJumpDetectionFlagLoads", TestBodyZJumpDetectionFlagLoads);
    RunTest("TestBodyZRequiresGnssAfterOverrides", TestBodyZRequiresGnssAfterOverrides);
    RunTest("TestVerticalEnvelopeCenterPullConfigValidation", TestVerticalEnvelopeCenterPullConfigValidation);
    RunTest("TestInitialStaticVerticalBiasConfigValidation", TestInitialStaticVerticalBiasConfigValidation);
    RunTest(
      "TestInitialStaticRtkHeightReferenceConfigValidation",
      TestInitialStaticRtkHeightReferenceConfigValidation);
    RunTest(
      "TestRtkVerticalLowpassReferenceConfigValidation",
      TestRtkVerticalLowpassReferenceConfigValidation);
    RunTest(
      "TestRtkVerticalDriftGateWeightingConfigValidation",
      TestRtkVerticalDriftGateWeightingConfigValidation);
    RunTest("TestAccelerometerBiasUgConfigParsing", TestAccelerometerBiasUgConfigParsing);
    RunTest("TestVerticalVelocityDeltaConfigValidation", TestVerticalVelocityDeltaConfigValidation);
    RunTest("TestVerticalJumpConfigValidation", TestVerticalJumpConfigValidation);
    RunTest(
      "TestVerticalJumpSpectralBiasRelaxationConfig",
      TestVerticalJumpSpectralBiasRelaxationConfig);
    RunTest("TestBodyZNHCConfigValidation", TestBodyZNHCConfigValidation);
    RunTest("TestRtkVelocityConfigValidation", TestRtkVelocityConfigValidation);
    RunTest("TestRtkOutageRecoveryConfigValidation", TestRtkOutageRecoveryConfigValidation);
    RunTest("TestInitialYawOverrideConfigValidation", TestInitialYawOverrideConfigValidation);
    RunTest("TestStage1YawRefinementConfigValidation", TestStage1YawRefinementConfigValidation);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
