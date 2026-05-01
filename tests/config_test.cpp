#include <cmath>
#include <iostream>
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

}  // namespace

int main() {
  try {
    RunTest("TestDirectZSmokeConfigLoads", TestDirectZSmokeConfigLoads);
    RunTest("TestEnvelopeSmokeConfigLoads", TestEnvelopeSmokeConfigLoads);
    RunTest("TestPhase3SmokeConfigLoads", TestPhase3SmokeConfigLoads);
    RunTest("TestPhase4SmokeConfigLoads", TestPhase4SmokeConfigLoads);
    RunTest("TestPhase5SmokeConfigLoads", TestPhase5SmokeConfigLoads);
    RunTest("TestOldCompatibilityKeysAreRejected", TestOldCompatibilityKeysAreRejected);
    RunTest("TestBodyZJumpDetectionFlagLoads", TestBodyZJumpDetectionFlagLoads);
    RunTest("TestBodyZRequiresGnssAfterOverrides", TestBodyZRequiresGnssAfterOverrides);
    RunTest("TestVerticalVelocityDeltaConfigValidation", TestVerticalVelocityDeltaConfigValidation);
    RunTest("TestVerticalJumpConfigValidation", TestVerticalJumpConfigValidation);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
