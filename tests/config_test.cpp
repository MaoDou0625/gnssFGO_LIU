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

}  // namespace

int main() {
  try {
    RunTest("TestDirectZSmokeConfigLoads", TestDirectZSmokeConfigLoads);
    RunTest("TestEnvelopeSmokeConfigLoads", TestEnvelopeSmokeConfigLoads);
    RunTest("TestOldCompatibilityKeysAreRejected", TestOldCompatibilityKeysAreRejected);
    RunTest("TestBodyZJumpDetectionFlagLoads", TestBodyZJumpDetectionFlagLoads);
    RunTest("TestBodyZRequiresGnssAfterOverrides", TestBodyZRequiresGnssAfterOverrides);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
