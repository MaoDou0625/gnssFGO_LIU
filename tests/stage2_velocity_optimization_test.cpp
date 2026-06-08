#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/RtkOutageSegmentedBatchRunner.h"
#include "offline_lc_minimal/core/Stage2AttitudeHoldBuilder.h"
#include "offline_lc_minimal/core/Stage2HorizontalHoldBuilder.h"
#include "offline_lc_minimal/core/Stage1OutageBodyYEnvelopeConstraintBuilder.h"
#include "offline_lc_minimal/core/Stage1OutageLateralVelocityEnvelopeEstimator.h"
#include "offline_lc_minimal/core/Stage1SourceReferencePolicy.h"
#include "offline_lc_minimal/core/Stage2VehicleNHCConstraintBuilder.h"
#include "offline_lc_minimal/core/Stage2VelocityOptimizationRunner.h"
#include "offline_lc_minimal/factor/AttitudeHoldFactor.h"
#include "offline_lc_minimal/factor/HorizontalHoldFactor.h"
#include "offline_lc_minimal/factor/FixedAxisBodyYVelocityEnvelopeFactor.h"
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

std::vector<offline_lc_minimal::TrajectoryRow> MakeTrajectoryRows(
  const double start_time_s,
  const double end_time_s) {
  std::vector<offline_lc_minimal::TrajectoryRow> rows;
  const int start = static_cast<int>(std::ceil(start_time_s - 1e-9));
  const int end = static_cast<int>(std::floor(end_time_s + 1e-9));
  for (int time_s = start; time_s <= end; ++time_s) {
    offline_lc_minimal::TrajectoryRow row;
    row.time_s = static_cast<double>(time_s);
    row.enu_position_m = Eigen::Vector3d(
      static_cast<double>(time_s),
      0.0,
      0.01 * static_cast<double>(time_s));
    row.enu_velocity_mps = Eigen::Vector3d::Zero();
    rows.push_back(row);
  }
  return rows;
}

std::vector<offline_lc_minimal::TrajectoryRow> MakeMovingTrajectoryRows(
  const double start_time_s,
  const double end_time_s) {
  auto rows = MakeTrajectoryRows(start_time_s, end_time_s);
  for (std::size_t index = 0; index < rows.size(); ++index) {
    rows[index].enu_velocity_mps = Eigen::Vector3d(
      1.0,
      index % 2U == 0U ? 0.0 : 0.02,
      0.0);
  }
  return rows;
}

offline_lc_minimal::RtkOutageWindowRow MakePlannedOutageWindow() {
  offline_lc_minimal::RtkOutageWindowRow row;
  row.window_index = 0U;
  row.pre_anchor_state_index = 10U;
  row.post_anchor_state_index = 20U;
  row.pre_anchor_time_s = 10.0;
  row.post_anchor_time_s = 20.0;
  row.start_time_s = 10.0;
  row.end_time_s = 20.0;
  row.duration_s = 10.0;
  row.skip_reason = "ADDED";
  return row;
}

offline_lc_minimal::GnssSolutionSample MakeRecoveryGnssSample(
  const double time_s,
  const double up_m) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 1.0;
  sample.lon_rad = 1.0;
  sample.h_m = 10.0;
  sample.sigma_lat_m = 0.01;
  sample.sigma_lon_m = 0.01;
  sample.sigma_h_m = 0.01;
  sample.best_sol_status_code = 1;
  sample.gnssfgo_type_code = 1;
  sample.enu_position_m = Eigen::Vector3d(time_s, 0.0, up_m);
  sample.has_enu_position = true;
  return sample;
}

std::vector<offline_lc_minimal::GnssSolutionSample> MakeRecoveryGnssSamples(
  const double start_time_s,
  const double end_time_s,
  const double step_s) {
  std::vector<offline_lc_minimal::GnssSolutionSample> samples;
  for (double time_s = start_time_s; time_s <= end_time_s + 1.0e-9; time_s += step_s) {
    samples.push_back(MakeRecoveryGnssSample(time_s, 0.10 + 0.01 * time_s));
  }
  return samples;
}

void TestStage2ReferenceAttitudeHorizontalApplicationPreservesVerticalComponents() {
  offline_lc_minimal::Stage2VelocityReference reference;
  offline_lc_minimal::TrajectoryRow row;
  row.time_s = 0.0;
  row.enu_position_m = Eigen::Vector3d(1.0, 2.0, 30.0);
  row.enu_velocity_mps = Eigen::Vector3d(4.0, 5.0, 60.0);
  row.ypr_rad = Eigen::Vector3d(2.0, 0.0, 0.0);
  row.bias_acc = Eigen::Vector3d(0.01, 0.02, 0.03);
  row.bias_gyro = Eigen::Vector3d(0.04, 0.05, 0.06);
  row.omega_radps = Eigen::Vector3d(0.07, 0.08, 0.09);
  reference.trajectory.push_back(row);
  offline_lc_minimal::ReferenceNodeState reference_state;
  reference_state.time_s = 0.0;
  reference_state.pose = gtsam::Pose3(
    gtsam::Rot3::Ypr(0.3, -0.2, 0.1),
    gtsam::Point3(1.0, 2.0, 30.0));
  reference_state.velocity = gtsam::Vector3(4.0, 5.0, 60.0);
  reference_state.bias = gtsam::imuBias::ConstantBias(
    gtsam::Vector3(0.01, 0.02, 0.03),
    gtsam::Vector3(0.04, 0.05, 0.06));
  reference_state.omega = gtsam::Vector3(0.07, 0.08, 0.09);
  reference.reference_states.push_back(reference_state);

  gtsam::Values values;
  values.insert(
    gtsam::symbol_shorthand::X(0),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(10.0, 20.0, 300.0)));
  values.insert(gtsam::symbol_shorthand::V(0), gtsam::Vector3(40.0, 50.0, 600.0));
  values.insert(
    gtsam::symbol_shorthand::B(0),
    gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.4, 0.5, 0.6),
      gtsam::Vector3(0.7, 0.8, 0.9)));
  values.insert(gtsam::symbol_shorthand::W(0), gtsam::Vector3(1.0, 2.0, 3.0));

  offline_lc_minimal::ApplyStage2ReferenceTrajectoryToInitialValues(
    reference,
    std::vector<double>{0.0},
    values,
    offline_lc_minimal::Stage2AttitudeHorizontalReferenceApplicationOptions());

  const auto pose = values.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(0));
  const auto ypr = pose.rotation().ypr();
  ExpectNear(ypr.x(), 0.3, 1.0e-12, "stage2 reference should apply yaw");
  ExpectNear(ypr.y(), -0.2, 1.0e-12, "stage2 reference should apply pitch");
  ExpectNear(ypr.z(), 0.1, 1.0e-12, "stage2 reference should apply roll");
  ExpectNear(pose.translation().x(), 1.0, 1.0e-12,
             "stage2 reference should apply horizontal x");
  ExpectNear(pose.translation().y(), 2.0, 1.0e-12,
             "stage2 reference should apply horizontal y");
  ExpectNear(pose.translation().z(), 300.0, 1.0e-12,
             "stage2 reference should preserve local vertical position");

  const auto velocity = values.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(0));
  ExpectNear(velocity.x(), 4.0, 1.0e-12,
             "stage2 reference should apply horizontal vx");
  ExpectNear(velocity.y(), 5.0, 1.0e-12,
             "stage2 reference should apply horizontal vy");
  ExpectNear(velocity.z(), 600.0, 1.0e-12,
             "stage2 reference should preserve local vertical velocity");

  const auto bias =
    values.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(0));
  ExpectNear(bias.accelerometer().x(), 0.01, 1.0e-12,
             "stage2 reference should apply ba_x");
  ExpectNear(bias.accelerometer().y(), 0.02, 1.0e-12,
             "stage2 reference should apply ba_y");
  ExpectNear(bias.accelerometer().z(), 0.6, 1.0e-12,
             "stage2 reference should preserve ba_z");
  ExpectNear(bias.gyroscope().x(), 0.04, 1.0e-12,
             "stage2 reference should apply gyro bias x");
  ExpectNear(bias.gyroscope().y(), 0.05, 1.0e-12,
             "stage2 reference should apply gyro bias y");
  ExpectNear(bias.gyroscope().z(), 0.06, 1.0e-12,
             "stage2 reference should apply gyro bias z");
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
  ExpectTrue(!stage2_config.enable_stage2_vehicle_nhc_constraint,
             "stage2 should respect the vehicle NHC experiment flag");
}

void TestFixedAxisBodyYVelocityEnvelopeFactorDeadband() {
  const gtsam::Vector3 body_y_axis = gtsam::Vector3::UnitY();
  const double mean_body_y_mps = 0.01;
  const double deadband_mps = 0.04;
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.02);
  offline_lc_minimal::factor::FixedAxisBodyYVelocityEnvelopeFactor factor(
    gtsam::symbol_shorthand::V(0),
    body_y_axis,
    mean_body_y_mps,
    deadband_mps,
    noise);

  gtsam::Matrix jacobian;
  const gtsam::Vector inside =
    factor.evaluateError(gtsam::Vector3(1.0, 0.03, 0.0), jacobian);
  ExpectNear(inside[0], 0.0, 1e-12, "inside deadband residual should be zero");
  ExpectNear(jacobian.norm(), 0.0, 1e-12, "inside deadband jacobian should be zero");

  const gtsam::Vector outside =
    factor.evaluateError(gtsam::Vector3(1.0, 0.08, 0.0), jacobian);
  ExpectNear(outside[0], 0.03, 1e-12,
             "outside deadband residual should subtract deadband");
  ExpectNear(jacobian(0, 0), 0.0, 1e-12, "body-y factor should not observe nav-x");
  ExpectNear(jacobian(0, 1), 1.0, 1e-12, "body-y factor should observe fixed y-axis");
  ExpectNear(jacobian(0, 2), 0.0, 1e-12, "body-y factor should not observe nav-z");
}

void TestStage1OutageBodyYEnvelopeEstimatorStatsAndSkips() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage1_outage_body_y_envelope = true;
  config.stage1_outage_body_y_pre_window_s = 5.0;
  config.stage1_outage_body_y_min_sample_count = 4;
  config.stage1_outage_body_y_min_speed_mps = 0.5;
  config.stage1_outage_body_y_min_sigma_mps = 0.005;
  config.stage1_outage_body_y_max_sigma_mps = 0.08;

  const std::vector<offline_lc_minimal::TrajectoryRow> trajectory =
    MakeMovingTrajectoryRows(0.0, 20.0);
  std::vector<offline_lc_minimal::RtkOutageWindowRow> outages{
    MakePlannedOutageWindow()};

  offline_lc_minimal::Stage1OutageLateralVelocityEnvelopeEstimateRequest request;
  request.config = &config;
  request.outage_windows = &outages;
  request.trajectory = &trajectory;
  const auto estimate =
    offline_lc_minimal::Stage1OutageLateralVelocityEnvelopeEstimator(request).Estimate();
  ExpectTrue(estimate.envelopes.size() == 1U, "estimator should return one outage row");
  const auto &row = estimate.envelopes.front();
  ExpectTrue(row.valid, "valid PRE samples should produce a body-y envelope");
  ExpectTrue(row.used_sample_count == 5U, "pre window should sample five 1 Hz states");
  ExpectNear(row.mean_body_y_mps, 0.012, 1e-12,
             "mean body-y should come from PRE velocity");
  ExpectNear(row.rmse_body_y_mps, 0.009797958971132712, 1e-12,
             "RMSE should be centered around the PRE mean");
  ExpectNear(row.deadband_mps, 2.0 * row.rmse_body_y_mps, 1e-12,
             "deadband should be the configured RMSE multiplier");

  config.stage1_outage_body_y_min_speed_mps = 2.0;
  const auto skipped =
    offline_lc_minimal::Stage1OutageLateralVelocityEnvelopeEstimator(request).Estimate();
  ExpectTrue(!skipped.envelopes.front().valid,
             "low-speed PRE samples should skip the envelope");
  ExpectTrue(skipped.envelopes.front().skip_reason == "INSUFFICIENT_SAMPLES",
             "low-speed rejection should fall back to insufficient samples");
  ExpectTrue(skipped.envelopes.front().skipped_low_speed_count == 5U,
             "low-speed skipped count should be recorded");

  std::vector<offline_lc_minimal::RtkOutageWindowRow> no_outages;
  request.outage_windows = &no_outages;
  const auto empty =
    offline_lc_minimal::Stage1OutageLateralVelocityEnvelopeEstimator(request).Estimate();
  ExpectTrue(empty.envelopes.empty(), "no outage windows should produce no envelope rows");
}

void TestStage1OutageBodyYEnvelopeBuilderAddsOnlyOutageFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage1_outage_body_y_envelope = true;

  offline_lc_minimal::Stage1OutageBodyYEnvelopeReference reference;
  reference.reference_states.resize(5U);
  for (std::size_t index = 0; index < reference.reference_states.size(); ++index) {
    reference.reference_states[index].time_s = static_cast<double>(index);
    reference.reference_states[index].pose = gtsam::Pose3();
  }
  offline_lc_minimal::Stage1OutageBodyYEnvelopeRow envelope;
  envelope.window_index = 7U;
  envelope.outage_start_time_s = 1.5;
  envelope.outage_end_time_s = 3.0;
  envelope.valid = true;
  envelope.mean_body_y_mps = 0.01;
  envelope.rmse_body_y_mps = 0.02;
  envelope.deadband_mps = 0.04;
  envelope.sigma_mps = 0.02;
  envelope.huber_k = 1.345;
  envelope.skip_reason = "OK";
  reference.envelopes.push_back(envelope);

  std::vector<double> state_timestamps{0.0, 1.0, 2.0, 3.0, 4.0};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::Stage1OutageBodyYEnvelopeRow> envelopes;
  std::vector<offline_lc_minimal::Stage1OutageBodyYStateDiagnosticRow> diagnostics;
  offline_lc_minimal::Stage1OutageBodyYEnvelopeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.reference = &reference;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.envelopes = &envelopes;
  request.state_diagnostics = &diagnostics;
  offline_lc_minimal::Stage1OutageBodyYEnvelopeConstraintBuilder(request).Build();

  ExpectTrue(graph.size() == 2U, "builder should add factors only for outage states");
  ExpectTrue(diagnostics.size() == 2U, "builder should emit one diagnostic per factor");
  ExpectTrue(diagnostics[0].state_index == 2U && diagnostics[1].state_index == 3U,
             "builder should skip PRE and POST states");
  ExpectTrue(envelopes.front().factor_count == 2U,
             "envelope CSV row should record the added factor count");
  ExpectTrue(summary.stage1_outage_body_y_velocity_factor_count == 2U,
             "summary should count body-y envelope factors");
}

void TestStage2SkipsStrongReferenceWhenStage1ReferenceInvalid() {
  struct CallRecord {
    bool has_stage2_reference = false;
    bool enable_stage2_velocity_optimization = false;
  };

  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage1_yaw_refinement = true;
  config.stage1_yaw_refinement_max_iterations = 1;
  config.enable_stage2_velocity_optimization = true;
  config.enable_rtk_outage_segmented_batch = false;

  std::vector<CallRecord> calls;
  offline_lc_minimal::Stage2VelocityOptimizationRequest request;
  request.config = config;
  request.dataset = offline_lc_minimal::DataSet{};
  request.run_once = [&](
                       const offline_lc_minimal::OfflineRunnerConfig &run_config,
                       std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> stage2_reference,
                       std::shared_ptr<const offline_lc_minimal::Stage1OutageBodyYEnvelopeReference>,
                       std::shared_ptr<const offline_lc_minimal::RoadNoiseStateReference>,
                       offline_lc_minimal::DataSet) {
    calls.push_back(CallRecord{
      stage2_reference != nullptr,
      run_config.enable_stage2_velocity_optimization});

    offline_lc_minimal::OfflineRunResult result;
    result.run_summary.origin_lat_rad = 0.1;
    result.run_summary.origin_lon_rad = 0.2;
    result.run_summary.origin_h_m = 10.0;
    result.run_summary.dynamic_start_time_s = 0.0;
    result.run_summary.processing_start_time_s = 0.0;
    result.run_summary.processing_end_time_s = 4.0;
    result.trajectory = MakeTrajectoryRows(0.0, 4.0);
    return result;
  };

  const auto result =
    offline_lc_minimal::Stage2VelocityOptimizationRunner(std::move(request)).Run();
  ExpectTrue(calls.size() == 1U,
             "invalid Stage1 reference should stop before the Stage2 hold pass");
  ExpectTrue(!calls.front().has_stage2_reference,
             "the Stage1 pass must not receive a Stage2 reference");
  ExpectTrue(!calls.front().enable_stage2_velocity_optimization,
             "the Stage1 pass should run with Stage2 disabled");
  ExpectTrue(result.run_summary.stage1_yaw_refinement_enabled,
             "result should expose the Stage1 diagnostic state");
  ExpectTrue(!result.run_summary.stage1_yaw_refinement_reference_valid,
             "invalid Stage1 branch should be marked unusable for strong hold");
  ExpectTrue(!result.run_summary.stage2_velocity_optimization_enabled,
             "Stage2 should be degraded instead of locking an invalid attitude reference");
}

void TestStage2RunsConstrainedStage1BeforeSegmentedStage2() {
  struct CallRecord {
    bool has_stage2_reference = false;
    bool has_body_y_reference = false;
    bool enable_stage2_velocity_optimization = false;
    bool enable_rtk_outage_segmented_batch = false;
  };

  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage1_yaw_refinement = true;
  config.stage1_yaw_refinement_max_iterations = 1;
  config.enable_stage2_velocity_optimization = true;
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_segmented_batch = true;
  config.enable_stage1_outage_body_y_envelope = true;
  config.stage1_heading_min_displacement_m = 0.1;
  config.stage1_outage_body_y_pre_window_s = 5.0;
  config.stage1_outage_body_y_min_sample_count = 4;
  config.stage1_outage_body_y_min_speed_mps = 0.5;
  config.rtk_outage_segmented_batch_max_outages = 1;

  std::vector<CallRecord> calls;
  offline_lc_minimal::Stage2VelocityOptimizationRequest request;
  request.config = config;
  request.dataset = offline_lc_minimal::DataSet{};
  request.dataset.gnss_samples = MakeRecoveryGnssSamples(0.0, 30.0, 0.5);
  request.run_once = [&](
                       const offline_lc_minimal::OfflineRunnerConfig &run_config,
                       std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> stage2_reference,
                       std::shared_ptr<const offline_lc_minimal::Stage1OutageBodyYEnvelopeReference> body_y_reference,
                       std::shared_ptr<const offline_lc_minimal::RoadNoiseStateReference>,
                       offline_lc_minimal::DataSet) {
    calls.push_back(CallRecord{
      stage2_reference != nullptr,
      body_y_reference != nullptr,
      run_config.enable_stage2_velocity_optimization,
      run_config.enable_rtk_outage_segmented_batch});

    offline_lc_minimal::OfflineRunResult result;
    const double start_time_s = run_config.processing_start_time_s;
    const double end_time_s =
      run_config.processing_end_time_s > 0.0 ? run_config.processing_end_time_s : 30.0;
    result.run_summary.dynamic_start_time_s = start_time_s;
    result.run_summary.processing_start_time_s = start_time_s;
    result.run_summary.processing_end_time_s = end_time_s;
    result.trajectory = MakeMovingTrajectoryRows(start_time_s, end_time_s);
    if (calls.size() <= 2U) {
      result.run_summary.dynamic_start_time_s = 0.0;
      result.run_summary.processing_start_time_s = 0.0;
      result.run_summary.processing_end_time_s = 30.0;
      result.trajectory = MakeMovingTrajectoryRows(0.0, 30.0);
      result.rtk_outage_windows.push_back(MakePlannedOutageWindow());
      if (run_config.enable_rtk_outage_segmented_batch) {
        result.run_summary.rtk_outage_segmented_batch_enabled = true;
        result.run_summary.rtk_outage_batch_segment_count = 3U;
        result.run_summary.rtk_outage_segmented_batch_run_count = 3U;
      }
    }
    if (body_y_reference != nullptr) {
      result.stage1_outage_body_y_envelopes = body_y_reference->envelopes;
      result.run_summary.stage1_outage_body_y_envelope_enabled = true;
      result.run_summary.stage1_outage_body_y_envelope_count =
        body_y_reference->envelopes.size();
      result.run_summary.stage1_outage_body_y_envelope_valid_count =
        static_cast<std::size_t>(
          std::count_if(
            body_y_reference->envelopes.begin(),
            body_y_reference->envelopes.end(),
            [](const offline_lc_minimal::Stage1OutageBodyYEnvelopeRow &row) {
              return row.valid;
            }));
    }
    return result;
  };

  const auto result =
    offline_lc_minimal::Stage2VelocityOptimizationRunner(std::move(request)).Run();
  ExpectTrue(calls.size() == 6U,
             "body-y envelope flow should run baseline Stage1, constrained Stage1, then four segmented child solves; actual=" +
               std::to_string(calls.size()) +
               " stop=" + result.run_summary.stage1_yaw_refinement_stop_reason +
               " reason=" + result.run_summary.stage1_yaw_refinement_selection_reason +
               " final_median=" +
               std::to_string(result.run_summary.stage1_yaw_refinement_final_median_error_rad) +
               " valid=" +
               (result.run_summary.stage1_yaw_refinement_reference_valid ? "true" : "false"));
  ExpectTrue(!calls[0].has_body_y_reference && !calls[0].has_stage2_reference,
             "baseline Stage1 should not receive body-y or stage2 references");
  ExpectTrue(calls[0].enable_rtk_outage_segmented_batch,
             "baseline Stage1 source should preserve top-level segmented batch");
  ExpectTrue(calls[1].has_body_y_reference && !calls[1].has_stage2_reference,
             "constrained Stage1 should receive the estimated body-y envelope");
  ExpectTrue(calls[1].enable_rtk_outage_segmented_batch,
             "constrained Stage1 source should preserve top-level segmented batch");
  for (std::size_t index = 2U; index < calls.size(); ++index) {
    ExpectTrue(!calls[index].has_body_y_reference,
               "segmented Stage2 children should not add Stage1 body-y factors");
    ExpectTrue(!calls[index].enable_rtk_outage_segmented_batch,
               "segmented Stage2 children should disable segmented recursion");
  }
  ExpectTrue(result.stage1_outage_body_y_envelopes.size() == 1U,
             "assembled Stage2 result should keep Stage1 body-y envelope diagnostics");
  ExpectTrue(result.run_summary.stage2_velocity_optimization_enabled,
             "assembled result should still mark Stage2 enabled");
}

void TestSegmentedStage2RunsStandalonePreAndGlobalReferenceChildren() {
  struct CallRecord {
    bool enable_stage1_yaw_refinement = false;
    bool enable_stage2_velocity_optimization = false;
    bool enable_rtk_outage_segmented_batch = false;
    bool enable_attitude_reference_constraint = false;
    bool has_stage2_reference = false;
    double processing_start_time_s = 0.0;
    double processing_end_time_s = 0.0;
  };

  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage1_yaw_refinement = true;
  config.enable_stage2_velocity_optimization = true;
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_segmented_batch = true;
  config.enable_attitude_reference_constraint = true;
  config.enable_stage1_outage_body_y_envelope = false;
  config.rtk_outage_segmented_batch_max_outages = 1;

  std::vector<CallRecord> calls;
  offline_lc_minimal::Stage2VelocityOptimizationRequest request;
  request.config = config;
  request.dataset = offline_lc_minimal::DataSet{};
  request.dataset.gnss_samples = MakeRecoveryGnssSamples(0.0, 30.0, 0.5);
  request.run_once = [&](const offline_lc_minimal::OfflineRunnerConfig &run_config,
                         std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> reference,
                         std::shared_ptr<const offline_lc_minimal::Stage1OutageBodyYEnvelopeReference>,
                         std::shared_ptr<const offline_lc_minimal::RoadNoiseStateReference>,
                         offline_lc_minimal::DataSet) {
    calls.push_back(CallRecord{
      run_config.enable_stage1_yaw_refinement,
      run_config.enable_stage2_velocity_optimization,
      run_config.enable_rtk_outage_segmented_batch,
      run_config.enable_attitude_reference_constraint,
      reference != nullptr,
      run_config.processing_start_time_s,
      run_config.processing_end_time_s});

    offline_lc_minimal::OfflineRunResult result;
    const double start_time_s = run_config.processing_start_time_s;
    const double end_time_s =
      run_config.processing_end_time_s > 0.0 ? run_config.processing_end_time_s : 30.0;
    result.run_summary.dynamic_start_time_s = start_time_s;
    result.run_summary.processing_start_time_s = start_time_s;
    result.run_summary.processing_end_time_s = end_time_s;
    result.trajectory = MakeTrajectoryRows(start_time_s, end_time_s);

    if (calls.size() == 1U) {
      result.run_summary.dynamic_start_time_s = 0.0;
      result.run_summary.processing_start_time_s = 0.0;
      result.run_summary.processing_end_time_s = 30.0;
      result.trajectory = MakeTrajectoryRows(0.0, 30.0);
      result.rtk_outage_windows.push_back(MakePlannedOutageWindow());
      if (run_config.enable_rtk_outage_segmented_batch) {
        result.run_summary.rtk_outage_segmented_batch_enabled = true;
        result.run_summary.rtk_outage_batch_segment_count = 3U;
        result.run_summary.rtk_outage_segmented_batch_run_count = 3U;
      }
    }
    return result;
  };

  const offline_lc_minimal::OfflineRunResult result =
    offline_lc_minimal::Stage2VelocityOptimizationRunner(std::move(request)).Run();

  ExpectTrue(calls.size() == 5U,
             "segmented stage2 should make one global stage1 pass plus four child solves");
  ExpectTrue(!calls[0].enable_stage1_yaw_refinement,
             "global stage1 run_once should already be inside the yaw refinement runner");
  ExpectTrue(!calls[0].enable_stage2_velocity_optimization,
             "global stage1 pass should disable stage2 optimization");
  ExpectTrue(calls[0].enable_rtk_outage_segmented_batch,
             "Stage1 source pass should preserve top-level segmented batch");
  ExpectTrue(!calls[0].has_stage2_reference,
             "global stage1 pass should not receive a stage2 reference");

  ExpectTrue(calls[1].enable_stage1_yaw_refinement,
             "pre segment should run as a standalone cutoff solve");
  ExpectTrue(calls[1].enable_stage2_velocity_optimization,
             "pre segment should keep the base stage2 route enabled");
  ExpectTrue(!calls[1].enable_rtk_outage_segmented_batch,
             "pre segment should disable segmented recursion");
  ExpectTrue(!calls[1].has_stage2_reference,
             "pre segment should not receive the global stage1 reference");

  for (std::size_t index = 2U; index < calls.size(); ++index) {
    ExpectTrue(!calls[index].enable_stage1_yaw_refinement,
               "non-pre segment child should reuse the global stage1 reference");
    ExpectTrue(calls[index].enable_stage2_velocity_optimization,
               "non-pre segment child should run a stage2 solve");
    ExpectTrue(!calls[index].enable_rtk_outage_segmented_batch,
               "non-pre segment child should disable segmented recursion");
    ExpectTrue(calls[index].has_stage2_reference,
               "non-pre segment child should use the sliced global stage1 reference");
  }
  ExpectNear(calls[1].processing_start_time_s, 0.0, 1e-12,
             "pre child start should match the prefix run");
  ExpectNear(calls[1].processing_end_time_s, 10.0, 1e-12,
             "pre child end should stop at outage start");
  ExpectNear(calls[2].processing_start_time_s, 20.0, 1e-12,
             "post probe should run before outage to produce position/velocity closure");
  ExpectNear(calls[2].processing_end_time_s, 30.0, 1e-12,
             "post probe should cover the post segment");
  ExpectNear(calls[3].processing_start_time_s, 0.0, 1e-12,
             "outage child should still run as the causal prefix for attitude");
  ExpectNear(calls[3].processing_end_time_s, 20.0, 1e-12,
              "outage child end should match outage end");
  ExpectNear(calls[4].processing_start_time_s, 20.0, 1e-12,
             "post child should run after outage and start at outage end");

  ExpectTrue(result.run_summary.rtk_outage_segmented_batch_enabled,
             "assembled result should mark segmented batch enabled");
  ExpectTrue(result.run_summary.rtk_outage_batch_segment_count == 3U,
             "assembled result should contain pre/outage/post segments");
  ExpectTrue(result.run_summary.stage1_source_segmented_batch_requested,
             "result should record that Stage1 source was allowed to segment");
  ExpectTrue(result.run_summary.stage1_source_segmentation_context == "stage1_source",
             "result should label the Stage1 source segmentation context");
}

void TestStage1SourcePolicyRejectsForcedGlobalOutageReference() {
  auto requested_config = offline_lc_minimal::DefaultConfig();
  requested_config.enable_rtk_outage_smoothing = true;
  requested_config.enable_rtk_outage_segmented_batch = true;
  requested_config.rtk_outage_segmented_batch_max_outages = 1;

  auto source_config = requested_config;
  source_config.enable_rtk_outage_segmented_batch = false;

  offline_lc_minimal::OfflineRunResult source_result;
  source_result.run_summary.stage1_yaw_refinement_enabled = true;
  source_result.run_summary.stage1_yaw_refinement_reference_valid = true;
  source_result.rtk_outage_windows.push_back(MakePlannedOutageWindow());

  offline_lc_minimal::ApplyStage1SourceReferencePolicy(
    offline_lc_minimal::Stage1SourceReferencePolicyRequest{
      &requested_config,
      &source_config},
    source_result);

  ExpectTrue(!source_result.run_summary.stage1_source_reference_valid,
             "forced-global Stage1 source should be rejected when outage segmentation was requested");
  ExpectTrue(source_result.run_summary.stage1_source_reference_evaluated,
             "Stage1 source policy should mark the reference decision as evaluated");
  ExpectTrue(!source_result.run_summary.stage1_yaw_refinement_reference_valid,
             "forced-global rejection should disable strong Stage1 yaw reference handoff");
  ExpectTrue(
    source_result.run_summary.stage1_source_reference_reject_reason ==
      "segmented_batch_disabled_with_outage",
    "forced-global rejection reason should identify the disabled outage segmentation");
}

void TestStage1SourcePolicyRejectsSegmentedPassthroughOutageReference() {
  auto requested_config = offline_lc_minimal::DefaultConfig();
  requested_config.enable_rtk_outage_smoothing = true;
  requested_config.enable_rtk_outage_segmented_batch = true;
  requested_config.rtk_outage_segmented_batch_max_outages = 1;

  auto source_config = requested_config;

  offline_lc_minimal::OfflineRunResult source_result;
  source_result.run_summary.stage1_yaw_refinement_enabled = true;
  source_result.run_summary.stage1_yaw_refinement_reference_valid = true;
  source_result.run_summary.rtk_outage_segmented_batch_enabled = false;
  source_result.rtk_outage_windows.push_back(MakePlannedOutageWindow());

  offline_lc_minimal::ApplyStage1SourceReferencePolicy(
    offline_lc_minimal::Stage1SourceReferencePolicyRequest{
      &requested_config,
      &source_config},
    source_result);

  ExpectTrue(!source_result.run_summary.stage1_source_reference_valid,
             "Stage1 source should reject outage references when requested segmentation did not run");
  ExpectTrue(source_result.run_summary.stage1_source_reference_evaluated,
             "Stage1 source policy should mark passthrough decisions as evaluated");
  ExpectTrue(
    source_result.run_summary.stage1_source_reference_reject_reason ==
      "segmented_batch_requested_but_not_run",
    "passthrough rejection reason should identify the missing segmented solve");
}

void TestSegmentedStage2SlicesRotationNativeReferenceWithoutTrajectoryRows() {
  struct CallRecord {
    bool has_reference = false;
    bool reference_has_states = false;
    bool reference_has_trajectory = false;
  };

  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage2_velocity_optimization = true;
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_segmented_batch = true;
  config.enable_rtk_outage_boundary_constraints = false;
  config.enable_initial_static_subgraph = true;
  config.static_alignment_duration_s = 1.0;
  config.initial_static_state_frequency_hz = 1.0;
  config.state_frequency_hz = 1.0;
  config.rtk_outage_segmented_batch_max_outages = 1;
  config.processing_start_time_s = 0.0;
  config.processing_end_time_s = 30.0;

  auto reference = std::make_shared<offline_lc_minimal::Stage2VelocityReference>();
  reference->reference_states = MakeReferenceStates(31U);

  std::vector<double> state_timestamps;
  for (int time_s = 0; time_s <= 30; ++time_s) {
    state_timestamps.push_back(static_cast<double>(time_s));
  }

  std::vector<CallRecord> calls;
  offline_lc_minimal::RtkOutageSegmentedBatchRunRequest request;
  request.base_config = config;
  request.config = config;
  request.stage2_reference = reference;
  request.outage_windows.push_back(MakePlannedOutageWindow());
  request.state_timestamps = state_timestamps;
  request.dynamic_start_time_s = 0.0;
  request.processing_end_time_s = 30.0;
  request.run_once = [&](
                       offline_lc_minimal::OfflineRunnerConfig run_config,
                       std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> child_reference,
                       std::shared_ptr<const offline_lc_minimal::Stage1OutageBodyYEnvelopeReference>,
                       std::shared_ptr<const offline_lc_minimal::RoadNoiseStateReference>,
                       offline_lc_minimal::DataSet) {
    calls.push_back(CallRecord{
      child_reference != nullptr,
      child_reference != nullptr && !child_reference->reference_states.empty(),
      child_reference != nullptr && !child_reference->trajectory.empty()});

    offline_lc_minimal::OfflineRunResult result;
    const double start_time_s = run_config.processing_start_time_s;
    const double end_time_s =
      run_config.processing_end_time_s > 0.0 ? run_config.processing_end_time_s : 30.0;
    result.run_summary.dynamic_start_time_s = start_time_s;
    result.run_summary.processing_start_time_s = start_time_s;
    result.run_summary.processing_end_time_s = end_time_s;
    result.trajectory = MakeTrajectoryRows(start_time_s, end_time_s);
    result.optimized_reference_states =
      offline_lc_minimal::BuildStage2ReferenceStatesFromTrajectory(result.trajectory);
    return result;
  };

  const auto result =
    offline_lc_minimal::RtkOutageSegmentedBatchRunner(std::move(request)).Run();
  ExpectTrue(!calls.empty(), "segmented runner should invoke child solves");
  const bool saw_sliced_reference =
    std::any_of(
      calls.begin(),
      calls.end(),
      [](const CallRecord &call) {
        return call.has_reference &&
               call.reference_has_states &&
               call.reference_has_trajectory;
      });
  ExpectTrue(
    saw_sliced_reference,
    "segmented runner should slice a reference that has only native states");
  ExpectTrue(result.run_summary.rtk_outage_segmented_batch_enabled,
             "assembled result should mark segmented batch enabled");
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
    RunTest(
      "TestStage2ReferenceAttitudeHorizontalApplicationPreservesVerticalComponents",
      TestStage2ReferenceAttitudeHorizontalApplicationPreservesVerticalComponents);
    RunTest("TestVehicleVelocityFactorsUseVelocityAndMountOnly", TestVehicleVelocityFactorsUseVelocityAndMountOnly);
    RunTest("TestVehicleZFixedForwardDoesNotPullForwardVelocity", TestVehicleZFixedForwardDoesNotPullForwardVelocity);
    RunTest("TestStage2AttitudeHoldBuilderAddsOneFactorPerState", TestStage2AttitudeHoldBuilderAddsOneFactorPerState);
    RunTest("TestStage2HorizontalHoldFactorsIgnoreVerticalAndAttitude", TestStage2HorizontalHoldFactorsIgnoreVerticalAndAttitude);
    RunTest("TestStage2HorizontalHoldBuilderAddsPositionAndVelocityFactors", TestStage2HorizontalHoldBuilderAddsPositionAndVelocityFactors);
    RunTest("TestStage2VehicleNHCLabelsGlobalWindowsAndUsesGlobalSigma", TestStage2VehicleNHCLabelsGlobalWindowsAndUsesGlobalSigma);
    RunTest("TestStage2PolicyDisablesRtkVelocityAndAttitudeReference", TestStage2PolicyDisablesRtkVelocityAndAttitudeReference);
    RunTest("TestFixedAxisBodyYVelocityEnvelopeFactorDeadband", TestFixedAxisBodyYVelocityEnvelopeFactorDeadband);
    RunTest(
      "TestStage1OutageBodyYEnvelopeEstimatorStatsAndSkips",
      TestStage1OutageBodyYEnvelopeEstimatorStatsAndSkips);
    RunTest(
      "TestStage1OutageBodyYEnvelopeBuilderAddsOnlyOutageFactors",
      TestStage1OutageBodyYEnvelopeBuilderAddsOnlyOutageFactors);
    RunTest(
      "TestStage2SkipsStrongReferenceWhenStage1ReferenceInvalid",
      TestStage2SkipsStrongReferenceWhenStage1ReferenceInvalid);
    RunTest(
      "TestStage2RunsConstrainedStage1BeforeSegmentedStage2",
      TestStage2RunsConstrainedStage1BeforeSegmentedStage2);
    RunTest(
      "TestSegmentedStage2RunsStandalonePreAndGlobalReferenceChildren",
      TestSegmentedStage2RunsStandalonePreAndGlobalReferenceChildren);
    RunTest(
      "TestStage1SourcePolicyRejectsForcedGlobalOutageReference",
      TestStage1SourcePolicyRejectsForcedGlobalOutageReference);
    RunTest(
      "TestStage1SourcePolicyRejectsSegmentedPassthroughOutageReference",
      TestStage1SourcePolicyRejectsSegmentedPassthroughOutageReference);
    RunTest(
      "TestSegmentedStage2SlicesRotationNativeReferenceWithoutTrajectoryRows",
      TestSegmentedStage2SlicesRotationNativeReferenceWithoutTrajectoryRows);
    RunTest("TestStage2ConfigParsingAndValidation", TestStage2ConfigParsingAndValidation);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
