#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "offline_lc_minimal/common/Config.h"

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
    std::abs(config.initial_static_vertical_bias_sigma_mps2 - 5e-5) < 1e-12,
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

void TestOldCompatibilityKeysAreRejected() {
  ExpectUnknownKey("enable_vertical_rtk_preintegration_feedback");
  ExpectUnknownKey("vertical_local_recovery_enabled");
  ExpectUnknownKey("vertical_inside_bias_window_s");
  ExpectUnknownKey("enable_nhc_jump_reference");
  ExpectUnknownKey("enable_reweighted_combined_imu_factor");
  ExpectUnknownKey("enable_body_z_seed_jump_windows");
  ExpectUnknownKey("enable_vertical_rtk_seed_pass");
}

void TestBodyZJumpDetectionFlagLoads() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "enable_body_z_jump_detection", "true");
  ExpectTrue(config.enable_body_z_jump_detection, "new body-z detection flag should load");
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
  config.enable_vertical_velocity_delta_constraint = true;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("requires enable_body_z_jump_detection") != std::string::npos;
  }
  ExpectTrue(threw, "velocity delta constraints should require body-z jump detection");
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
}

void TestInitialStaticVerticalBiasConfigValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_global_acc_bias = true;
  config.enable_vertical_acc_bias_gm_process = true;
  config.enable_initial_static_subgraph = true;
  config.enable_initial_static_vertical_bias_soft_prior = true;
  config.static_alignment_duration_s = 100.0;
  config.initial_static_vertical_bias_sigma_mps2 = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("initial static constraint settings") != std::string::npos;
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
  ExpectTrue(config.enable_body_z_nhc_constraint, "body-z NHC flag should load");
  ExpectTrue(config.enable_body_z_nhc_global_weak_constraint, "body-z NHC global flag should load");
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
  config.body_z_nhc_jump_velocity_sigma_mps = 0.0;
  threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("body-z NHC settings") != std::string::npos;
  }
  ExpectTrue(threw, "non-positive body-z NHC velocity sigma should be rejected");
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
    RunTest("TestOldCompatibilityKeysAreRejected", TestOldCompatibilityKeysAreRejected);
    RunTest("TestBodyZJumpDetectionFlagLoads", TestBodyZJumpDetectionFlagLoads);
    RunTest("TestBodyZRequiresGnssAfterOverrides", TestBodyZRequiresGnssAfterOverrides);
    RunTest("TestVerticalEnvelopeCenterPullConfigValidation", TestVerticalEnvelopeCenterPullConfigValidation);
    RunTest("TestInitialStaticVerticalBiasConfigValidation", TestInitialStaticVerticalBiasConfigValidation);
    RunTest("TestVerticalVelocityDeltaConfigValidation", TestVerticalVelocityDeltaConfigValidation);
    RunTest("TestVerticalJumpConfigValidation", TestVerticalJumpConfigValidation);
    RunTest("TestBodyZNHCConfigValidation", TestBodyZNHCConfigValidation);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
