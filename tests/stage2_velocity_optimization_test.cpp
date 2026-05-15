#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/Stage2AttitudeHoldBuilder.h"
#include "offline_lc_minimal/core/Stage2HorizontalHoldBuilder.h"
#include "offline_lc_minimal/core/Stage2VehicleNHCConstraintBuilder.h"
#include "offline_lc_minimal/factor/AttitudeHoldFactor.h"
#include "offline_lc_minimal/factor/HorizontalHoldFactor.h"
#include "offline_lc_minimal/factor/VehicleVelocityNHCFactor.h"
#include "offline_lc_minimal/factor/VehicleZFixedForwardNHCFactor.h"

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

void ExpectNear(
  const double actual,
  const double expected,
  const double tolerance,
  const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
      message + ": actual=" + std::to_string(actual) +
      " expected=" + std::to_string(expected));
  }
}

offline_lc_minimal::factor::BodyFrameAxesNav IdentityAxes() {
  offline_lc_minimal::factor::BodyFrameAxesNav axes;
  axes.body_x_axis_nav = gtsam::Vector3::UnitX();
  axes.body_y_axis_nav = gtsam::Vector3::UnitY();
  axes.body_z_axis_nav = gtsam::Vector3::UnitZ();
  return axes;
}

std::vector<offline_lc_minimal::ReferenceNodeState> MakeReferenceStates(
  const std::size_t count) {
  std::vector<offline_lc_minimal::ReferenceNodeState> states(count);
  for (std::size_t index = 0; index < count; ++index) {
    states[index].time_s = static_cast<double>(index);
    states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(0.1 * static_cast<double>(index), 0.01, -0.02),
      gtsam::Point3(
        10.0 + static_cast<double>(index),
        -3.0 - 0.5 * static_cast<double>(index),
        2.0));
    states[index].velocity = gtsam::Vector3(
      1.0 + static_cast<double>(index),
      -0.1 * static_cast<double>(index),
      0.2);
  }
  return states;
}

void TestVehicleVelocityFactorsUseVelocityAndMountOnly() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const auto axes = IdentityAxes();
  const gtsam::Key velocity_key = gtsam::symbol_shorthand::V(3);
  const gtsam::Key mount_key = gtsam::Symbol('m', 0);
  const gtsam::Vector3 velocity(2.0, 3.0, 1.0);
  const gtsam::Vector3 mount(0.1, -0.2, 0.3);

  offline_lc_minimal::factor::VehicleYVelocityZeroFactor y_factor(
    velocity_key,
    mount_key,
    axes,
    noise);
  gtsam::Matrix h_y_velocity;
  gtsam::Matrix h_y_mount;
  const auto y_error = y_factor.evaluateError(
    velocity,
    mount,
    h_y_velocity,
    h_y_mount);
  ExpectNear(y_error[0], 2.4, 1e-12, "vehicle-y residual is wrong");
  ExpectNear(h_y_velocity(0, 0), -0.3, 1e-12, "vehicle-y dres/dvx is wrong");
  ExpectNear(h_y_velocity(0, 1), 1.0, 1e-12, "vehicle-y dres/dvy is wrong");
  ExpectNear(h_y_velocity(0, 2), 0.0, 1e-12, "vehicle-y dres/dvz is wrong");
  ExpectNear(h_y_mount(0, 0), 0.0, 1e-12, "vehicle-y dres/dk_zx is wrong");
  ExpectNear(h_y_mount(0, 1), 0.0, 1e-12, "vehicle-y dres/dk_zy is wrong");
  ExpectNear(h_y_mount(0, 2), -2.0, 1e-12, "vehicle-y dres/dk_yx is wrong");

  offline_lc_minimal::factor::VehicleZVelocityZeroFactor z_factor(
    velocity_key,
    mount_key,
    axes,
    noise);
  gtsam::Matrix h_z_velocity;
  gtsam::Matrix h_z_mount;
  const auto z_error = z_factor.evaluateError(
    velocity,
    mount,
    h_z_velocity,
    h_z_mount);
  ExpectNear(z_error[0], 0.8, 1e-12, "vehicle-z residual is wrong");
  ExpectNear(h_z_velocity(0, 0), -0.1, 1e-12, "vehicle-z dres/dvx is wrong");
  ExpectNear(h_z_velocity(0, 1), 0.0, 1e-12, "vehicle-z dres/dvy is wrong");
  ExpectNear(h_z_velocity(0, 2), 1.0, 1e-12, "vehicle-z dres/dvz is wrong");
  ExpectNear(h_z_mount(0, 0), -2.0, 1e-12, "vehicle-z dres/dk_zx is wrong");
  ExpectNear(h_z_mount(0, 1), 0.0, 1e-12, "vehicle-z should ignore fixed k_zy");
  ExpectNear(h_z_mount(0, 2), 0.0, 1e-12, "vehicle-z dres/dk_yx is wrong");

  ExpectTrue(y_factor.keys().size() == 2U, "vehicle-y factor should have two keys");
  ExpectTrue(z_factor.keys().size() == 2U, "vehicle-z factor should have two keys");
  ExpectTrue(y_factor.keys()[0] == velocity_key && y_factor.keys()[1] == mount_key,
             "vehicle-y factor should not connect pose");
  ExpectTrue(z_factor.keys()[0] == velocity_key && z_factor.keys()[1] == mount_key,
             "vehicle-z factor should not connect pose");
}

void TestVehicleZFixedForwardDoesNotPullForwardVelocity() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const auto axes = IdentityAxes();
  const gtsam::Key velocity_key = gtsam::symbol_shorthand::V(3);
  const gtsam::Key mount_key = gtsam::Symbol('m', 0);
  const gtsam::Vector3 velocity(2.0, 3.0, 1.0);
  const gtsam::Vector3 mount(0.1, 0.0, 0.0);

  offline_lc_minimal::factor::VehicleZVelocityZeroFixedForwardFactor factor(
    velocity_key,
    mount_key,
    axes,
    5.0,
    gtsam::Vector3(20.0, -30.0, -0.5),
    noise);
  gtsam::Matrix h_velocity;
  gtsam::Matrix h_mount;
  const auto error = factor.evaluateError(
    velocity,
    mount,
    h_velocity,
    h_mount);
  ExpectNear(error[0], 0.5, 1e-12, "fixed-forward vehicle-z residual is wrong");
  ExpectNear(h_velocity(0, 0), 0.0, 1e-12,
             "fixed-forward vehicle-z should not pull body-x velocity");
  ExpectNear(h_velocity(0, 1), 0.0, 1e-12,
             "fixed-forward vehicle-z should not pull body-y velocity");
  ExpectNear(h_velocity(0, 2), 1.0, 1e-12,
             "fixed-forward vehicle-z should constrain body-z velocity");
  ExpectNear(h_mount(0, 0), -5.0, 1e-12,
             "fixed-forward vehicle-z dres/dk_zx is wrong");
  ExpectNear(h_mount(0, 1), 0.0, 1e-12,
             "fixed-forward vehicle-z should ignore k_zy");
  ExpectNear(h_mount(0, 2), 0.0, 1e-12,
             "fixed-forward vehicle-z should ignore k_yx");

  gtsam::Values values;
  values.insert(gtsam::symbol_shorthand::V(0), gtsam::Vector3(2.0, 0.0, 1.0));
  values.insert(gtsam::symbol_shorthand::V(1), gtsam::Vector3(9.0, 0.0, 2.0));
  values.insert(mount_key, mount);
  const std::vector<gtsam::Key> velocity_keys{
    gtsam::symbol_shorthand::V(0),
    gtsam::symbol_shorthand::V(1)};
  const std::vector<offline_lc_minimal::factor::BodyFrameAxesNav> axes_by_state{
    axes,
    axes};
  offline_lc_minimal::factor::VehicleZWindowDisplacementZeroFixedForwardFactor window_factor(
    velocity_keys,
    mount_key,
    axes_by_state,
    std::vector<double>{5.0, 7.0},
    std::vector<gtsam::Vector3>{
      gtsam::Vector3(20.0, -30.0, 0.0),
      gtsam::Vector3(-10.0, 40.0, 0.0)},
    std::vector<double>{0.0, 2.0},
    noise);
  std::vector<gtsam::Matrix> jacobians;
  const auto displacement_error = window_factor.unwhitenedError(values, jacobians);
  ExpectNear(displacement_error[0], 1.8, 1e-12,
             "fixed-forward vehicle-z displacement residual is wrong");
  ExpectNear(jacobians[0](0, 0), 0.0, 1e-12,
             "fixed-forward displacement should not pull first body-x velocity");
  ExpectNear(jacobians[1](0, 0), 0.0, 1e-12,
             "fixed-forward displacement should not pull second body-x velocity");
  ExpectNear(jacobians[0](0, 2), 1.0, 1e-12,
             "fixed-forward displacement first dres/dvz is wrong");
  ExpectNear(jacobians[1](0, 2), 1.0, 1e-12,
             "fixed-forward displacement second dres/dvz is wrong");
  ExpectNear(jacobians[2](0, 0), -12.0, 1e-12,
             "fixed-forward displacement dres/dk_zx is wrong");
}

void TestStage2AttitudeHoldBuilderAddsOneFactorPerState() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage2_velocity_optimization = true;
  config.stage2_attitude_hold_sigma_rad = 1e-5;
  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  const auto reference_states = MakeReferenceStates(timestamps.size());
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;

  offline_lc_minimal::Stage2AttitudeHoldBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.reference_states = &reference_states;
  request.graph = &graph;
  request.run_summary = &summary;
  offline_lc_minimal::Stage2AttitudeHoldBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == timestamps.size(), "attitude hold factor count is wrong");
  ExpectTrue(summary.stage2_attitude_hold_factor_count == timestamps.size(),
             "summary attitude hold count is wrong");
  const auto first_factor =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::AttitudeHoldFactor>(graph.at(0));
  ExpectTrue(first_factor.get() != nullptr, "first factor should be an attitude hold factor");
  ExpectTrue(first_factor->keys().front() == gtsam::symbol_shorthand::X(0),
             "attitude hold should connect the first pose key");
}

void TestStage2HorizontalHoldFactorsIgnoreVerticalAndAttitude() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  offline_lc_minimal::factor::HorizontalPositionHoldFactor position_factor(
    gtsam::symbol_shorthand::X(0),
    (gtsam::Vector2() << 1.0, 2.0).finished(),
    noise);
  const gtsam::Pose3 pose(
    gtsam::Rot3::Ypr(0.5, -0.4, 0.3),
    gtsam::Point3(1.2, 1.7, 99.0));
  gtsam::Matrix h_pose;
  const auto position_error = position_factor.evaluateError(pose, h_pose);
  ExpectNear(position_error[0], 0.2, 1e-12, "horizontal position east residual is wrong");
  ExpectNear(position_error[1], -0.3, 1e-12, "horizontal position north residual is wrong");

  offline_lc_minimal::factor::HorizontalVelocityHoldFactor velocity_factor(
    gtsam::symbol_shorthand::V(0),
    (gtsam::Vector2() << 3.0, 4.0).finished(),
    noise);
  gtsam::Matrix h_velocity;
  const auto velocity_error = velocity_factor.evaluateError(
    gtsam::Vector3(2.5, 4.25, 10.0),
    h_velocity);
  ExpectNear(velocity_error[0], -0.5, 1e-12, "horizontal velocity east residual is wrong");
  ExpectNear(velocity_error[1], 0.25, 1e-12, "horizontal velocity north residual is wrong");
  ExpectNear(h_velocity(0, 0), 1.0, 1e-12, "horizontal velocity dres/dvx is wrong");
  ExpectNear(h_velocity(1, 1), 1.0, 1e-12, "horizontal velocity dres/dvy is wrong");
  ExpectNear(h_velocity(0, 2), 0.0, 1e-12, "horizontal velocity should ignore vz");
  ExpectNear(h_velocity(1, 2), 0.0, 1e-12, "horizontal velocity should ignore vz");
}

void TestStage2HorizontalHoldBuilderAddsPositionAndVelocityFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage2_velocity_optimization = true;
  config.stage2_horizontal_position_hold_sigma_m = 1e-4;
  config.stage2_horizontal_velocity_hold_sigma_mps = 1e-4;
  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  const auto reference_states = MakeReferenceStates(timestamps.size());
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;

  offline_lc_minimal::Stage2HorizontalHoldBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.reference_states = &reference_states;
  request.graph = &graph;
  request.run_summary = &summary;
  offline_lc_minimal::Stage2HorizontalHoldBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == timestamps.size() * 2U,
             "horizontal hold should add one position and one velocity factor per state");
  ExpectTrue(summary.stage2_horizontal_position_hold_factor_count == timestamps.size(),
             "summary horizontal position hold count is wrong");
  ExpectTrue(summary.stage2_horizontal_velocity_hold_factor_count == timestamps.size(),
             "summary horizontal velocity hold count is wrong");
}

void TestStage2VehicleNHCLabelsGlobalWindowsAndUsesGlobalSigma() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage2_velocity_optimization = true;
  config.enable_stage2_vehicle_nhc_constraint = true;
  config.enable_body_z_nhc_global_weak_constraint = true;
  config.enable_body_z_nhc_strict_effective_weighting = false;
  config.body_z_nhc_min_window_s = 0.5;
  config.body_z_nhc_global_window_s = 2.0;
  config.body_z_nhc_global_stride_s = 2.0;
  config.body_z_nhc_jump_velocity_sigma_mps = 0.005;
  config.body_z_nhc_jump_displacement_sigma_m = 0.005;
  config.body_z_nhc_global_velocity_sigma_mps = 0.123;
  config.body_z_nhc_global_displacement_sigma_m = 0.234;
  config.body_z_nhc_horizontal_leakage_min_speed_mps = 0.0;
  config.body_z_nhc_horizontal_leakage_min_sample_count = 1;
  const std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  const auto reference_states = MakeReferenceStates(timestamps.size());
  std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::Values initial_values;
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    initial_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(1.0 + static_cast<double>(index), 0.0, 0.1));
  }
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::Stage2MountLeakageDiagnosticRow> mount_diagnostics;
  std::vector<offline_lc_minimal::Stage2VehicleNHCStateDiagnosticRow> state_diagnostics;

  offline_lc_minimal::Stage2VehicleNHCConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &initial_values;
  request.reference_states = &reference_states;
  request.dynamic_start_index = 0U;
  request.mount_leakage_key = gtsam::Symbol('m', 0);
  request.graph = &graph;
  request.run_summary = &summary;
  request.mount_diagnostics = &mount_diagnostics;
  request.state_diagnostics = &state_diagnostics;
  offline_lc_minimal::Stage2VehicleNHCConstraintBuilder(std::move(request)).Build();

  ExpectTrue(!state_diagnostics.empty(), "stage2 global NHC diagnostics should not be empty");
  for (const auto &row : state_diagnostics) {
    ExpectTrue(row.nhc_region_type == "GLOBAL",
               "stage2 global windows should be labeled GLOBAL");
    if (row.velocity_factor_used) {
      ExpectNear(
        row.effective_vehicle_z_sigma_mps,
        config.body_z_nhc_global_velocity_sigma_mps,
        1e-12,
        "stage2 global window should use global velocity sigma");
    }
  }
  ExpectTrue(summary.stage2_vehicle_nhc_window_count > 0U,
             "stage2 global NHC windows should be added");
  ExpectTrue(summary.stage2_vehicle_z_nhc_velocity_factor_count > 0U,
             "stage2 global velocity factors should be added");
}

void TestStage2PolicyDisablesRtkVelocityAndAttitudeReference() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage1_yaw_refinement = true;
  config.enable_stage2_velocity_optimization = true;
  config.enable_attitude_reference_constraint = true;
  config.enable_rtk_velocity_constraint = false;
  config.enable_rtk_outage_velocity_delta_3d = true;
  config.enable_body_z_nhc_constraint = true;
  config.enable_body_z_nhc_horizontal_leakage_correction = true;
  config.enable_stage2_vehicle_nhc_constraint = false;

  const auto stage2_config =
    offline_lc_minimal::MakeStage2VelocityOptimizationConfig(config);
  ExpectTrue(!stage2_config.enable_stage1_yaw_refinement,
             "stage2 should not recurse into stage1");
  ExpectTrue(stage2_config.enable_stage2_velocity_optimization,
             "stage2 flag should remain enabled for inner run");
  ExpectTrue(!stage2_config.enable_attitude_reference_constraint,
             "stage2 should disable attitude reference");
  ExpectTrue(!stage2_config.enable_rtk_velocity_constraint,
             "stage2 should not enable RTK horizontal velocity");
  ExpectTrue(!stage2_config.enable_rtk_outage_velocity_delta_3d,
             "stage2 should disable outage 3D velocity delta");
  ExpectTrue(!stage2_config.enable_body_z_nhc_constraint,
             "stage2 should disable old body-z NHC");
  ExpectTrue(!stage2_config.enable_body_z_nhc_horizontal_leakage_correction,
             "stage2 should disable fixed body-z leakage correction");
  ExpectTrue(stage2_config.enable_stage2_vehicle_nhc_constraint,
             "stage2 should enable graph-internal vehicle NHC");
}

void TestStage2ConfigParsingAndValidation() {
  auto config = offline_lc_minimal::DefaultConfig();
  offline_lc_minimal::OverrideConfigField(config, "enable_stage2_velocity_optimization", "true");
  offline_lc_minimal::OverrideConfigField(config, "enable_stage2_vehicle_nhc_constraint", "true");
  offline_lc_minimal::OverrideConfigField(config, "stage2_attitude_hold_sigma_rad", "2e-5");
  offline_lc_minimal::OverrideConfigField(config, "stage2_horizontal_position_hold_sigma_m", "3e-4");
  offline_lc_minimal::OverrideConfigField(config, "stage2_horizontal_velocity_hold_sigma_mps", "4e-4");
  offline_lc_minimal::OverrideConfigField(config, "stage2_mount_leakage_prior_sigma_rad", "0.03");
  offline_lc_minimal::OverrideConfigField(config, "stage2_vehicle_y_nhc_velocity_sigma_mps", "0.07");
  offline_lc_minimal::OverrideConfigField(config, "stage2_vehicle_y_nhc_displacement_sigma_m", "0.09");
  offline_lc_minimal::ValidateConfig(config);
  ExpectTrue(config.enable_stage2_velocity_optimization, "stage2 enable flag should parse");
  ExpectNear(config.stage2_attitude_hold_sigma_rad, 2e-5, 1e-12,
             "stage2 attitude hold sigma should parse");
  ExpectNear(config.stage2_horizontal_position_hold_sigma_m, 3e-4, 1e-12,
             "stage2 horizontal position hold sigma should parse");
  ExpectNear(config.stage2_horizontal_velocity_hold_sigma_mps, 4e-4, 1e-12,
             "stage2 horizontal velocity hold sigma should parse");
  ExpectNear(config.stage2_mount_leakage_prior_sigma_rad, 0.03, 1e-12,
             "stage2 mount prior sigma should parse");
  ExpectNear(config.stage2_vehicle_y_nhc_velocity_sigma_mps, 0.07, 1e-12,
             "stage2 vehicle-y velocity sigma should parse");
  ExpectNear(config.stage2_vehicle_y_nhc_displacement_sigma_m, 0.09, 1e-12,
             "stage2 vehicle-y displacement sigma should parse");

  config.stage2_mount_leakage_prior_sigma_rad = 0.0;
  bool threw = false;
  try {
    offline_lc_minimal::ValidateConfig(config);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  ExpectTrue(threw, "non-positive stage2 mount prior sigma should be rejected");
}

}  // namespace

int main() {
  try {
    RunTest("TestVehicleVelocityFactorsUseVelocityAndMountOnly", TestVehicleVelocityFactorsUseVelocityAndMountOnly);
    RunTest("TestVehicleZFixedForwardDoesNotPullForwardVelocity", TestVehicleZFixedForwardDoesNotPullForwardVelocity);
    RunTest("TestStage2AttitudeHoldBuilderAddsOneFactorPerState", TestStage2AttitudeHoldBuilderAddsOneFactorPerState);
    RunTest("TestStage2HorizontalHoldFactorsIgnoreVerticalAndAttitude", TestStage2HorizontalHoldFactorsIgnoreVerticalAndAttitude);
    RunTest("TestStage2HorizontalHoldBuilderAddsPositionAndVelocityFactors", TestStage2HorizontalHoldBuilderAddsPositionAndVelocityFactors);
    RunTest("TestStage2VehicleNHCLabelsGlobalWindowsAndUsesGlobalSigma", TestStage2VehicleNHCLabelsGlobalWindowsAndUsesGlobalSigma);
    RunTest("TestStage2PolicyDisablesRtkVelocityAndAttitudeReference", TestStage2PolicyDisablesRtkVelocityAndAttitudeReference);
    RunTest("TestStage2ConfigParsingAndValidation", TestStage2ConfigParsingAndValidation);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
