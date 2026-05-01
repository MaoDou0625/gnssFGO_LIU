#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <array>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/core/VerticalJumpImuMasker.h"
#include "offline_lc_minimal/core/VerticalJumpShapeConstraintBuilder.h"
#include "offline_lc_minimal/factor/VerticalMaskedCombinedImuFactor.h"
#include "offline_lc_minimal/factor/VerticalPositionRampFactor.h"
#include "offline_lc_minimal/factor/VerticalPositionVelocityConsistencyFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityContextMeanContinuityFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityDeltaFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityHeightSlopeFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityMeanFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityRampFactor.h"

namespace {

namespace symbol = gtsam::symbol_shorthand;

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

void ExpectNear(const double actual, const double expected, const double tolerance, const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
      message + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
  }
}

gtsam::PreintegratedCombinedMeasurements MakePreintegratedMeasurements() {
  auto params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
  params->setAccelerometerCovariance(gtsam::Matrix33::Identity() * 1e-4);
  params->setGyroscopeCovariance(gtsam::Matrix33::Identity() * 1e-4);
  params->setIntegrationCovariance(gtsam::Matrix33::Identity() * 1e-6);
  params->setBiasAccCovariance(gtsam::Matrix33::Identity() * 1e-6);
  params->setBiasOmegaCovariance(gtsam::Matrix33::Identity() * 1e-6);
  gtsam::PreintegratedCombinedMeasurements pim(params, gtsam::imuBias::ConstantBias());
  pim.integrateMeasurement(gtsam::Vector3::Zero(), gtsam::Vector3::Zero(), 0.1);
  return pim;
}

offline_lc_minimal::BodyZSeedJumpWindowRow MakeJumpWindow(const double start_time_s, const double end_time_s) {
  offline_lc_minimal::BodyZSeedJumpWindowRow window;
  window.start_time_s = start_time_s;
  window.end_time_s = end_time_s;
  return window;
}

void TestVerticalMaskedCombinedImuDropsZAndVz() {
  const auto pim = MakePreintegratedMeasurements();
  const gtsam::imuBias::ConstantBias bias;
  const gtsam::NavState state_i(gtsam::Pose3(), gtsam::Vector3::Zero());
  const gtsam::NavState predicted_j = pim.predict(state_i, bias);
  const offline_lc_minimal::factor::VerticalMaskedCombinedImuFactor factor(
    symbol::X(0), symbol::V(0), symbol::X(1), symbol::V(1), symbol::B(0), symbol::B(1), pim);

  const gtsam::Vector base_error = factor.evaluateError(
    state_i.pose(),
    state_i.v(),
    predicted_j.pose(),
    predicted_j.v(),
    bias,
    bias);
  ExpectNear(base_error.norm(), 0.0, 1e-8, "predicted state should satisfy masked factor");

  const auto translation = predicted_j.pose().translation();
  const gtsam::Pose3 z_shifted_pose(
    predicted_j.pose().rotation(),
    gtsam::Point3(translation.x(), translation.y(), translation.z() + 10.0));
  const gtsam::Vector3 z_shifted_velocity = predicted_j.v() + gtsam::Vector3(0.0, 0.0, 5.0);
  const gtsam::Vector z_error = factor.evaluateError(
    state_i.pose(),
    state_i.v(),
    z_shifted_pose,
    z_shifted_velocity,
    bias,
    bias);
  ExpectNear(z_error.norm(), 0.0, 1e-8, "z position and vz should be masked");

  const gtsam::Pose3 x_shifted_pose(
    predicted_j.pose().rotation(),
    gtsam::Point3(translation.x() + 1.0, translation.y(), translation.z()));
  const gtsam::Vector x_error = factor.evaluateError(
    state_i.pose(),
    state_i.v(),
    x_shifted_pose,
    predicted_j.v(),
    bias,
    bias);
  ExpectTrue(x_error.norm() > 0.1, "horizontal position residual should remain active");

  const gtsam::Vector3 x_shifted_velocity = predicted_j.v() + gtsam::Vector3(1.0, 0.0, 0.0);
  const gtsam::Vector vx_error = factor.evaluateError(
    state_i.pose(),
    state_i.v(),
    predicted_j.pose(),
    x_shifted_velocity,
    bias,
    bias);
  ExpectTrue(vx_error.norm() > 0.1, "horizontal velocity residual should remain active");
}

void TestVerticalMaskedCombinedImuMatchesCombinedKeptRowsForBias() {
  constexpr std::array<int, offline_lc_minimal::factor::VerticalMaskedCombinedImuFactor::kMaskedErrorDim> kept_rows{
    0, 1, 2, 3, 4, 6, 7, 9, 10, 11, 12, 13, 14};
  const auto pim = MakePreintegratedMeasurements();
  const gtsam::imuBias::ConstantBias bias_i(
    gtsam::Vector3(0.01, -0.02, 0.03),
    gtsam::Vector3(0.001, -0.002, 0.003));
  const gtsam::imuBias::ConstantBias bias_j(
    gtsam::Vector3(-0.04, 0.05, -0.06),
    gtsam::Vector3(-0.004, 0.005, -0.006));
  const gtsam::NavState state_i(
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.01, -0.02, 0.03), gtsam::Point3(1.0, 2.0, 3.0)),
    gtsam::Vector3(0.4, -0.5, 0.6));
  const gtsam::NavState state_j(
    gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.03, 0.02, -0.01), gtsam::Point3(1.2, 2.3, 3.4)),
    gtsam::Vector3(-0.2, 0.1, -0.3));

  const gtsam::CombinedImuFactor combined(
    symbol::X(0), symbol::V(0), symbol::X(1), symbol::V(1), symbol::B(0), symbol::B(1), pim);
  const offline_lc_minimal::factor::VerticalMaskedCombinedImuFactor masked(
    symbol::X(0), symbol::V(0), symbol::X(1), symbol::V(1), symbol::B(0), symbol::B(1), pim);

  gtsam::Matrix combined_h_bias_i;
  gtsam::Matrix combined_h_bias_j;
  const gtsam::Vector combined_error = combined.evaluateError(
    state_i.pose(),
    state_i.v(),
    state_j.pose(),
    state_j.v(),
    bias_i,
    bias_j,
    boost::none,
    boost::none,
    boost::none,
    boost::none,
    combined_h_bias_i,
    combined_h_bias_j);

  gtsam::Matrix masked_h_bias_i;
  gtsam::Matrix masked_h_bias_j;
  const gtsam::Vector masked_error = masked.evaluateError(
    state_i.pose(),
    state_i.v(),
    state_j.pose(),
    state_j.v(),
    bias_i,
    bias_j,
    boost::none,
    boost::none,
    boost::none,
    boost::none,
    masked_h_bias_i,
    masked_h_bias_j);

  for (std::size_t row = 0; row < kept_rows.size(); ++row) {
    ExpectNear(
      masked_error(static_cast<Eigen::Index>(row)),
      combined_error(kept_rows[row]),
      1e-12,
      "masked error row should match CombinedImuFactor");
    for (Eigen::Index col = 0; col < masked_h_bias_i.cols(); ++col) {
      ExpectNear(
        masked_h_bias_i(static_cast<Eigen::Index>(row), col),
        combined_h_bias_i(kept_rows[row], col),
        1e-12,
        "masked bias_i Jacobian row should match CombinedImuFactor");
      ExpectNear(
        masked_h_bias_j(static_cast<Eigen::Index>(row), col),
        combined_h_bias_j(kept_rows[row], col),
        1e-12,
        "masked bias_j Jacobian row should match CombinedImuFactor");
    }
  }
}

void TestVerticalJumpImuMaskerReplacesOverlappingIntervals() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_masked_imu = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  const auto pim = MakePreintegratedMeasurements();
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::CombinedImuFactor(symbol::X(0), symbol::V(0), symbol::X(1), symbol::V(1), symbol::B(0), symbol::B(1), pim));
  graph.add(gtsam::CombinedImuFactor(symbol::X(1), symbol::V(1), symbol::X(2), symbol::V(2), symbol::B(1), symbol::B(2), pim));

  const std::vector<offline_lc_minimal::VerticalJumpImuIntervalRecord> intervals{
    {0, 1, 0.0, 0.5, 0, pim},
    {1, 2, 2.0, 2.5, 1, pim},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(0.2, 0.3)};
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpMaskedImuDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalJumpImuMaskRequest request;
  request.config = &config;
  request.intervals = &intervals;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalJumpImuMasker(std::move(request)).Apply();

  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalMaskedCombinedImuFactor>(graph.at(0)) != nullptr,
    "overlapping interval should be replaced with masked IMU factor");
  ExpectTrue(
    boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(graph.at(1)) != nullptr,
    "non-overlapping interval should keep normal CombinedImuFactor");
  ExpectNear(static_cast<double>(summary.vertical_jump_masked_imu_factor_count), 1.0, 0.0, "masked count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_jump_combined_imu_factor_count), 1.0, 0.0, "combined count is wrong");
  ExpectTrue(diagnostics.front().masked_z_position && diagnostics.front().masked_vz, "mask diagnostics are wrong");
}

void TestVerticalVelocityRampFactor() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalVelocityRampFactor factor(1, 2, 3, 0.5, noise);

  ExpectNear(
    factor.evaluateError(gtsam::Vector3(5.0, 2.0, 0.0), gtsam::Vector3(-3.0, 4.0, 1.0), gtsam::Vector3(8.0, -7.0, 2.0))(0),
    0.0,
    1e-12,
    "linear ramp should have zero residual");
  ExpectNear(
    factor.evaluateError(gtsam::Vector3(5.0, 2.0, 0.0), gtsam::Vector3(-3.0, 4.0, 1.5), gtsam::Vector3(8.0, -7.0, 2.0))(0),
    0.5,
    1e-12,
    "zigzag vertical speed should produce residual");
}

void TestVerticalPositionRampFactor() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalPositionRampFactor factor(1, 2, 3, 0.5, noise);
  const gtsam::Pose3 start_pose(gtsam::Rot3(), gtsam::Point3(5.0, 2.0, 0.0));
  const gtsam::Pose3 end_pose(gtsam::Rot3(), gtsam::Point3(8.0, -7.0, 2.0));

  ExpectNear(
    factor.evaluateError(start_pose, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-3.0, 4.0, 1.0)), end_pose)(0),
    0.0,
    1e-12,
    "linear vertical position ramp should have zero residual");
  ExpectNear(
    factor.evaluateError(start_pose, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-3.0, 4.0, 1.5)), end_pose)(0),
    0.5,
    1e-12,
    "zigzag vertical position should produce residual");
}

void TestVerticalVelocityHeightSlopeFactor() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalVelocityHeightSlopeFactor factor(1, 2, 3, 2.0, noise);
  const gtsam::Pose3 start_pose(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 1.0));
  const gtsam::Pose3 end_pose(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 3.0));

  ExpectNear(
    factor.evaluateError(start_pose, gtsam::Vector3(5.0, -2.0, 1.0), end_pose)(0),
    0.0,
    1e-12,
    "velocity should match height slope");
  ExpectNear(
    factor.evaluateError(start_pose, gtsam::Vector3(5.0, -2.0, 1.5), end_pose)(0),
    0.5,
    1e-12,
    "vertical velocity slope residual is wrong");
}

void TestVerticalPositionVelocityConsistencyFactor() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalPositionVelocityConsistencyFactor factor(
    1,
    2,
    3,
    4,
    2.0,
    noise);
  const gtsam::Pose3 pose_i(gtsam::Rot3(), gtsam::Point3(3.0, -5.0, 1.0));
  const gtsam::Pose3 pose_j(gtsam::Rot3(), gtsam::Point3(-7.0, 9.0, 5.0));

  ExpectNear(
    factor.evaluateError(
      pose_i,
      gtsam::Vector3(100.0, -50.0, 1.5),
      pose_j,
      gtsam::Vector3(-60.0, 70.0, 2.5))(0),
    0.0,
    1e-12,
    "trapezoid vertical motion should have zero residual");
  ExpectNear(
    factor.evaluateError(
      pose_i,
      gtsam::Vector3(100.0, -50.0, 1.5),
      pose_j,
      gtsam::Vector3(-60.0, 70.0, 3.5))(0),
    -1.0,
    1e-12,
    "vertical position-velocity residual is wrong");
}

void TestVerticalVelocityMeanFactor() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalVelocityMeanFactor factor(
    symbol::V(0),
    std::vector<gtsam::Key>{symbol::V(1), symbol::V(2), symbol::V(3)},
    noise);

  gtsam::Values values;
  values.insert(symbol::V(0), gtsam::Vector3(100.0, -20.0, 2.0));
  values.insert(symbol::V(1), gtsam::Vector3(-10.0, 30.0, 1.0));
  values.insert(symbol::V(2), gtsam::Vector3(50.0, -60.0, 2.0));
  values.insert(symbol::V(3), gtsam::Vector3(70.0, 80.0, 3.0));
  ExpectNear(factor.unwhitenedError(values)(0), 0.0, 1e-12, "boundary should match context mean");

  values.update(symbol::V(0), gtsam::Vector3(-200.0, 400.0, 2.5));
  ExpectNear(factor.unwhitenedError(values)(0), 0.5, 1e-12, "vertical mean residual is wrong");
}

void TestVerticalVelocityContextMeanContinuityFactor() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalVelocityContextMeanContinuityFactor factor(
    std::vector<gtsam::Key>{symbol::V(0), symbol::V(1)},
    std::vector<gtsam::Key>{symbol::V(2), symbol::V(3), symbol::V(4)},
    noise);

  gtsam::Values values;
  values.insert(symbol::V(0), gtsam::Vector3(10.0, -20.0, 1.0));
  values.insert(symbol::V(1), gtsam::Vector3(30.0, 40.0, 3.0));
  values.insert(symbol::V(2), gtsam::Vector3(-50.0, 60.0, 0.0));
  values.insert(symbol::V(3), gtsam::Vector3(70.0, -80.0, 2.0));
  values.insert(symbol::V(4), gtsam::Vector3(90.0, 100.0, 4.0));
  ExpectNear(factor.unwhitenedError(values)(0), 0.0, 1e-12, "context means should match");

  values.update(symbol::V(4), gtsam::Vector3(-90.0, -100.0, 5.5));
  ExpectNear(factor.unwhitenedError(values)(0), 0.5, 1e-12, "context mean continuity residual is wrong");

  std::vector<gtsam::Matrix> jacobians;
  (void)factor.unwhitenedError(values, jacobians);
  ExpectNear(static_cast<double>(jacobians.size()), 5.0, 0.0, "context mean Jacobian count is wrong");
  for (const auto &jacobian : jacobians) {
    ExpectNear(jacobian(0, 0), 0.0, 1e-12, "context mean Jacobian should not touch vx");
    ExpectNear(jacobian(0, 1), 0.0, 1e-12, "context mean Jacobian should not touch vy");
  }
  ExpectNear(jacobians[0](0, 2), -0.5, 1e-12, "pre context Jacobian is wrong");
  ExpectNear(jacobians[1](0, 2), -0.5, 1e-12, "pre context Jacobian is wrong");
  ExpectNear(jacobians[2](0, 2), 1.0 / 3.0, 1e-12, "post context Jacobian is wrong");
  ExpectNear(jacobians[3](0, 2), 1.0 / 3.0, 1e-12, "post context Jacobian is wrong");
  ExpectNear(jacobians[4](0, 2), 1.0 / 3.0, 1e-12, "post context Jacobian is wrong");
}

void TestVerticalJumpShapeConstraintBuilder() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_ramp_smoothing = true;
  config.enable_vertical_jump_position_ramp_smoothing = true;
  config.enable_vertical_jump_velocity_height_slope_constraint = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  config.vertical_jump_velocity_ramp_sigma_mps = 0.08;
  config.vertical_jump_position_ramp_sigma_m = 0.10;
  config.vertical_jump_velocity_height_slope_sigma_mps = 0.20;
  const std::vector<double> state_timestamps{0.0, 0.5, 1.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(0.25, 0.75)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(
    static_cast<double>(graph.size()),
    3.0,
    0.0,
    "velocity ramp, position ramp, and velocity-height slope factors should be added");
  ExpectNear(static_cast<double>(summary.vertical_jump_velocity_ramp_factor_count), 1.0, 0.0, "ramp count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_position_ramp_factor_count),
    1.0,
    0.0,
    "position ramp count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_height_slope_factor_count),
    1.0,
    0.0,
    "velocity-height slope count is wrong");
  ExpectNear(static_cast<double>(diagnostics.front().factor_count), 3.0, 0.0, "diagnostic factor count is wrong");

  gtsam::Values values;
  values.insert(symbol::X(0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)));
  values.insert(symbol::X(1), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 1.5)));
  values.insert(symbol::X(2), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 2.0)));
  values.insert(symbol::V(0), gtsam::Vector3(0.0, 0.0, 0.0));
  values.insert(symbol::V(1), gtsam::Vector3(0.0, 0.0, 1.5));
  values.insert(symbol::V(2), gtsam::Vector3(0.0, 0.0, 2.0));
  offline_lc_minimal::PopulateVerticalJumpVelocityRampDiagnostics(values, diagnostics);
  ExpectNear(diagnostics.front().inside_vz_range_mps, 2.0, 1e-12, "inside vz range is wrong");
  ExpectNear(diagnostics.front().ramp_residual_max_mps, 0.5, 1e-12, "ramp residual max is wrong");
  ExpectNear(diagnostics.front().inside_up_range_m, 2.0, 1e-12, "inside up range is wrong");
  ExpectNear(
    diagnostics.front().position_ramp_residual_max_m,
    0.5,
    1e-12,
    "position ramp residual max is wrong");
}

void TestVerticalJumpShapeConstraintBuilderPositionOnly() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_ramp_smoothing = false;
  config.enable_vertical_jump_position_ramp_smoothing = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  config.vertical_jump_position_ramp_sigma_m = 0.10;
  const std::vector<double> state_timestamps{0.0, 0.5, 1.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(0.25, 0.75)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "one position ramp factor should be added");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_position_ramp_factor_count),
    1.0,
    0.0,
    "position-only ramp count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_ramp_factor_count),
    0.0,
    0.0,
    "position-only builder should not add velocity ramp factors");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_height_slope_factor_count),
    0.0,
    0.0,
    "position-only builder should not add velocity-height slope factors");
}

void TestVerticalJumpShapeConstraintBuilderContinuityAndConsistency() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_ramp_smoothing = true;
  config.enable_vertical_jump_position_ramp_smoothing = true;
  config.enable_vertical_jump_velocity_continuity = true;
  config.enable_vertical_jump_position_velocity_consistency = true;
  config.enable_vertical_jump_velocity_height_slope_constraint = false;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  const std::vector<double> state_timestamps{0.0, 0.5, 1.0, 1.5, 2.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(0.75, 1.25)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(summary.vertical_jump_velocity_continuity_factor_count), 2.0, 0.0, "continuity count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_position_velocity_consistency_factor_count),
    4.0,
    0.0,
    "position-velocity consistency count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_jump_velocity_ramp_factor_count), 1.0, 0.0, "velocity ramp count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_jump_position_ramp_factor_count), 1.0, 0.0, "position ramp count is wrong");
  ExpectNear(static_cast<double>(graph.size()), 8.0, 0.0, "phase5 builder factor count is wrong");
  ExpectNear(static_cast<double>(continuity_diagnostics.size()), 1.0, 0.0, "continuity diagnostics count is wrong");
  ExpectNear(static_cast<double>(continuity_diagnostics.front().pre_anchor_state_index), 0.0, 0.0, "pre anchor is wrong");
  ExpectNear(static_cast<double>(continuity_diagnostics.front().post_anchor_state_index), 4.0, 0.0, "post anchor is wrong");
  ExpectTrue(
    continuity_diagnostics.front().entry_position_velocity_factor_added,
    "entry boundary position-velocity factor should be added");
  ExpectTrue(
    continuity_diagnostics.front().exit_position_velocity_factor_added,
    "exit boundary position-velocity factor should be added");

  gtsam::Values values;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    values.insert(symbol::X(index), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, static_cast<double>(index))));
    values.insert(symbol::V(index), gtsam::Vector3(0.0, 0.0, 2.0));
  }
  offline_lc_minimal::PopulateVerticalJumpContinuityDiagnostics(
    values,
    state_timestamps,
    continuity_diagnostics);
  ExpectNear(continuity_diagnostics.front().entry_delta_vz_mps, 0.0, 1e-12, "entry continuity residual is wrong");
  ExpectNear(continuity_diagnostics.front().exit_delta_vz_mps, 0.0, 1e-12, "exit continuity residual is wrong");
  ExpectNear(
    continuity_diagnostics.front().max_position_velocity_residual_m,
    0.0,
    1e-12,
    "position-velocity diagnostic residual is wrong");
  ExpectNear(
    continuity_diagnostics.front().max_boundary_zv_mismatch_m,
    0.0,
    1e-12,
    "boundary position-velocity diagnostic residual is wrong");

  values.update(symbol::X(0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.25)));
  offline_lc_minimal::PopulateVerticalJumpContinuityDiagnostics(
    values,
    state_timestamps,
    continuity_diagnostics);
  ExpectNear(
    continuity_diagnostics.front().entry_zv_mismatch_m,
    -0.25,
    1e-12,
    "non-zero entry boundary position-velocity diagnostic residual is wrong");
  ExpectNear(
    continuity_diagnostics.front().max_boundary_zv_mismatch_m,
    0.25,
    1e-12,
    "non-zero boundary position-velocity max diagnostic residual is wrong");
}

void TestVerticalJumpShapeConstraintBuilderVelocityContextMean() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  config.vertical_jump_velocity_context_window_s = 1.0;
  config.vertical_jump_velocity_context_mean_sigma_mps = 0.03;
  const std::vector<double> state_timestamps{0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(1.25, 1.75)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 4.0, 0.0, "context mean factor count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_context_factor_count),
    4.0,
    0.0,
    "summary context factor count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_context_skipped_count),
    0.0,
    0.0,
    "context constraints should not skip when both sides have samples");
  ExpectNear(static_cast<double>(continuity_diagnostics.size()), 1.0, 0.0, "context diagnostic missing");
  ExpectNear(
    static_cast<double>(continuity_diagnostics.front().pre_context_state_count),
    2.0,
    0.0,
    "pre context state count is wrong");
  ExpectNear(
    static_cast<double>(continuity_diagnostics.front().post_context_state_count),
    2.0,
    0.0,
    "post context state count is wrong");
  ExpectNear(
    static_cast<double>(continuity_diagnostics.front().velocity_context_factor_count),
    4.0,
    0.0,
    "diagnostic context factor count is wrong");

  gtsam::Values values;
  const std::array<double, 7> vz_values{1.0, 3.0, 2.1, 0.0, -1.1, -2.0, 0.0};
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    values.insert(symbol::X(index), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.5 * static_cast<double>(index))));
    values.insert(symbol::V(index), gtsam::Vector3(0.0, 0.0, vz_values[index]));
  }
  offline_lc_minimal::PopulateVerticalJumpContinuityDiagnostics(
    values,
    state_timestamps,
    continuity_diagnostics);
  ExpectNear(
    continuity_diagnostics.front().pre_context_mean_vz_mps,
    2.0,
    1e-12,
    "pre context mean is wrong");
  ExpectNear(
    continuity_diagnostics.front().post_context_mean_vz_mps,
    -1.0,
    1e-12,
    "post context mean is wrong");
  ExpectNear(
    continuity_diagnostics.front().max_pre_context_residual_mps,
    1.0,
    1e-12,
    "pre context residual is wrong");
  ExpectNear(
    continuity_diagnostics.front().max_post_context_residual_mps,
    1.0,
    1e-12,
    "post context residual is wrong");
}

void TestVerticalJumpShapeConstraintBuilderContextMeanContinuity() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_context_mean_continuity = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  config.vertical_jump_velocity_context_window_s = 1.0;
  config.vertical_jump_context_mean_continuity_sigma_mps = 0.01;
  const std::vector<double> state_timestamps{0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(1.25, 1.75)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "one context mean continuity factor should be added");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_context_mean_continuity_factor_count),
    1.0,
    0.0,
    "context mean continuity count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_context_mean_continuity_skipped_count),
    0.0,
    0.0,
    "context mean continuity should not skip when both contexts exist");
  ExpectNear(static_cast<double>(continuity_diagnostics.size()), 1.0, 0.0, "context mean diagnostic missing");
  ExpectTrue(
    continuity_diagnostics.front().context_mean_continuity_factor_added,
    "diagnostic should mark context mean continuity factor");

  gtsam::Values values;
  const std::array<double, 7> vz_values{1.0, 3.0, 2.1, 0.0, -1.1, -2.0, 0.0};
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    values.insert(symbol::X(index), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)));
    values.insert(symbol::V(index), gtsam::Vector3(0.0, 0.0, vz_values[index]));
  }
  offline_lc_minimal::PopulateVerticalJumpContinuityDiagnostics(
    values,
    state_timestamps,
    continuity_diagnostics);
  ExpectNear(
    continuity_diagnostics.front().context_mean_delta_vz_mps,
    -3.0,
    1e-12,
    "context mean delta diagnostic is wrong");
  ExpectNear(
    continuity_diagnostics.front().context_mean_continuity_residual_mps,
    -3.0,
    1e-12,
    "context mean residual diagnostic is wrong");
}

void TestVerticalJumpShapeConstraintBuilderContextMeanContinuityMissingSideSkips() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_context_mean_continuity = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  config.vertical_jump_velocity_context_window_s = 1.0;
  const std::vector<double> state_timestamps{1.0, 1.5, 2.0, 2.5, 3.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(1.25, 1.75)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 0.0, 0.0, "missing pre context should add no mean factor");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_context_mean_continuity_factor_count),
    0.0,
    0.0,
    "missing pre context should not count as added");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_context_mean_continuity_skipped_count),
    1.0,
    0.0,
    "missing pre context should count as skipped");
  ExpectNear(static_cast<double>(continuity_diagnostics.size()), 1.0, 0.0, "missing side diagnostic missing");
  ExpectTrue(
    !continuity_diagnostics.front().context_mean_continuity_factor_added,
    "missing pre context should not mark factor added");
}

void TestVerticalJumpShapeConstraintBuilderVelocityContextUsesPaddedSpanBoundary() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  config.vertical_jump_velocity_context_window_s = 1.0;
  const std::vector<double> state_timestamps{0.1, 0.45, 1.0, 1.5, 1.9, 2.3, 2.55, 3.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(1.65, 1.85)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(continuity_diagnostics.size()), 1.0, 0.0, "off-grid span diagnostic missing");
  ExpectNear(
    static_cast<double>(continuity_diagnostics.front().pre_context_start_state_index),
    1.0,
    0.0,
    "pre context should start from padded span start minus window");
  ExpectNear(
    static_cast<double>(continuity_diagnostics.front().pre_context_state_count),
    2.0,
    0.0,
    "pre context should use the padded span start boundary");
  ExpectNear(
    static_cast<double>(continuity_diagnostics.front().post_context_state_count),
    3.0,
    0.0,
    "post context should use the padded span end boundary");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_context_factor_count),
    4.0,
    0.0,
    "off-grid context factor count is wrong");
}

void TestVerticalJumpShapeConstraintBuilderVelocityContextStopsAtNeighborSpan() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  config.vertical_jump_velocity_context_window_s = 1.0;
  const std::vector<double> state_timestamps{0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{
    MakeJumpWindow(1.25, 1.75),
    MakeJumpWindow(2.75, 3.0),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(continuity_diagnostics.size()), 2.0, 0.0, "two spans should be diagnosed");
  ExpectNear(
    static_cast<double>(continuity_diagnostics.front().post_context_state_count),
    0.0,
    0.0,
    "post context should stop before a neighboring jump span");
  ExpectNear(
    static_cast<double>(continuity_diagnostics.back().pre_context_state_count),
    0.0,
    0.0,
    "pre context should stop after a neighboring jump span");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_context_factor_count),
    4.0,
    0.0,
    "neighboring spans should only use immediate clean context samples");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_context_skipped_count),
    2.0,
    0.0,
    "missing clean context sides should be counted as skipped");
}

void TestVerticalJumpShapeConstraintBuilderContextOnlyEmptySpanSkipsContext() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  config.vertical_jump_velocity_context_window_s = 1.0;
  const std::vector<double> state_timestamps{0.0, 1.0, 2.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(10.0, 10.5)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 0.0, 0.0, "empty context-only span should add no factors");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_ramp_skipped_count),
    0.0,
    0.0,
    "context-only empty span should not count as a ramp skip");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_context_skipped_count),
    1.0,
    0.0,
    "context-only empty span should count as a context skip");
}

void TestVerticalJumpShapeConstraintBuilderMergesOverlappingSpans() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_continuity = true;
  config.enable_vertical_jump_position_velocity_consistency = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  const std::vector<double> state_timestamps{0.0, 0.5, 1.0, 1.5, 2.0, 2.5};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{
    MakeJumpWindow(0.75, 1.25),
    MakeJumpWindow(1.30, 1.60),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(summary.vertical_jump_velocity_continuity_factor_count), 2.0, 0.0, "merged continuity count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_position_velocity_consistency_factor_count),
    4.0,
    0.0,
    "merged position-velocity consistency count is wrong");
  ExpectNear(static_cast<double>(graph.size()), 6.0, 0.0, "merged span should not duplicate boundary factors");
  ExpectNear(static_cast<double>(continuity_diagnostics.size()), 1.0, 0.0, "overlapping windows should share one continuity span");
}

void TestVerticalJumpShapeConstraintBuilderMissingAnchorSkipsBoundary() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_continuity = true;
  config.enable_vertical_jump_position_velocity_consistency = true;
  config.vertical_jump_masked_imu_padding_s = 0.25;
  const std::vector<double> state_timestamps{0.0, 0.5, 1.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(0.25, 0.50)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(summary.vertical_jump_velocity_continuity_factor_count), 1.0, 0.0, "one boundary should be constrained");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_position_velocity_consistency_factor_count),
    2.0,
    0.0,
    "one boundary and one internal position-velocity factor should be added");
  ExpectNear(static_cast<double>(graph.size()), 3.0, 0.0, "missing-anchor graph factor count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_jump_continuity_skipped_count), 1.0, 0.0, "one missing anchor should be skipped");
  ExpectTrue(!continuity_diagnostics.front().entry_factor_added, "entry anchor should be missing");
  ExpectTrue(continuity_diagnostics.front().exit_factor_added, "exit anchor should be constrained");
  ExpectTrue(
    !continuity_diagnostics.front().entry_position_velocity_factor_added,
    "entry position-velocity anchor should be missing");
  ExpectTrue(
    continuity_diagnostics.front().exit_position_velocity_factor_added,
    "exit position-velocity boundary should be constrained");
}

void TestVerticalJumpShapeConstraintBuilderOneStateSpanStillAddsContinuity() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_jump_velocity_continuity = true;
  config.enable_vertical_jump_position_velocity_consistency = true;
  config.vertical_jump_masked_imu_padding_s = 0.10;
  const std::vector<double> state_timestamps{0.0, 1.0, 2.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows{MakeJumpWindow(1.0, 1.0)};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalJumpVelocityRampDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::VerticalJumpContinuityDiagnosticRow> continuity_diagnostics;

  offline_lc_minimal::VerticalJumpShapeConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &windows;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.continuity_diagnostics = &continuity_diagnostics;
  offline_lc_minimal::VerticalJumpShapeConstraintBuilder(std::move(request)).Build();

  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_continuity_factor_count),
    2.0,
    0.0,
    "single-state span should constrain both continuity boundaries");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_position_velocity_consistency_factor_count),
    2.0,
    0.0,
    "single-state span should constrain both position-velocity boundaries");
  ExpectNear(static_cast<double>(graph.size()), 4.0, 0.0, "single-state span continuity factor count is wrong");
  ExpectNear(static_cast<double>(continuity_diagnostics.size()), 1.0, 0.0, "single-state continuity diagnostic missing");
  ExpectTrue(continuity_diagnostics.front().entry_factor_added, "single-state entry continuity should be constrained");
  ExpectTrue(continuity_diagnostics.front().exit_factor_added, "single-state exit continuity should be constrained");
  ExpectTrue(
    continuity_diagnostics.front().entry_position_velocity_factor_added,
    "single-state entry position-velocity should be constrained");
  ExpectTrue(
    continuity_diagnostics.front().exit_position_velocity_factor_added,
    "single-state exit position-velocity should be constrained");
  ExpectNear(
    static_cast<double>(summary.vertical_jump_velocity_ramp_skipped_count),
    0.0,
    0.0,
    "continuity-only single-state span should not count as a ramp skip");

  gtsam::Values values;
  values.insert(symbol::X(0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)));
  values.insert(symbol::X(1), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(10.0, -5.0, 1.0)));
  values.insert(symbol::X(2), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-8.0, 3.0, 2.0)));
  values.insert(symbol::V(0), gtsam::Vector3(30.0, 40.0, 1.0));
  values.insert(symbol::V(1), gtsam::Vector3(-20.0, 50.0, 1.0));
  values.insert(symbol::V(2), gtsam::Vector3(10.0, -30.0, 1.0));
  offline_lc_minimal::PopulateVerticalJumpContinuityDiagnostics(
    values,
    state_timestamps,
    continuity_diagnostics);
  ExpectNear(
    continuity_diagnostics.front().entry_zv_mismatch_m,
    0.0,
    1e-12,
    "single-state entry z-v mismatch should be zero");
  ExpectNear(
    continuity_diagnostics.front().exit_zv_mismatch_m,
    0.0,
    1e-12,
    "single-state exit z-v mismatch should be zero");
}

}  // namespace

int main() {
  try {
    RunTest("TestVerticalMaskedCombinedImuDropsZAndVz", TestVerticalMaskedCombinedImuDropsZAndVz);
    RunTest(
      "TestVerticalMaskedCombinedImuMatchesCombinedKeptRowsForBias",
      TestVerticalMaskedCombinedImuMatchesCombinedKeptRowsForBias);
    RunTest("TestVerticalJumpImuMaskerReplacesOverlappingIntervals", TestVerticalJumpImuMaskerReplacesOverlappingIntervals);
    RunTest("TestVerticalVelocityRampFactor", TestVerticalVelocityRampFactor);
    RunTest("TestVerticalPositionRampFactor", TestVerticalPositionRampFactor);
    RunTest("TestVerticalVelocityHeightSlopeFactor", TestVerticalVelocityHeightSlopeFactor);
    RunTest("TestVerticalPositionVelocityConsistencyFactor", TestVerticalPositionVelocityConsistencyFactor);
    RunTest("TestVerticalVelocityMeanFactor", TestVerticalVelocityMeanFactor);
    RunTest("TestVerticalVelocityContextMeanContinuityFactor", TestVerticalVelocityContextMeanContinuityFactor);
    RunTest("TestVerticalJumpShapeConstraintBuilder", TestVerticalJumpShapeConstraintBuilder);
    RunTest("TestVerticalJumpShapeConstraintBuilderPositionOnly", TestVerticalJumpShapeConstraintBuilderPositionOnly);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderContinuityAndConsistency",
      TestVerticalJumpShapeConstraintBuilderContinuityAndConsistency);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderVelocityContextMean",
      TestVerticalJumpShapeConstraintBuilderVelocityContextMean);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderContextMeanContinuity",
      TestVerticalJumpShapeConstraintBuilderContextMeanContinuity);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderContextMeanContinuityMissingSideSkips",
      TestVerticalJumpShapeConstraintBuilderContextMeanContinuityMissingSideSkips);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderVelocityContextUsesPaddedSpanBoundary",
      TestVerticalJumpShapeConstraintBuilderVelocityContextUsesPaddedSpanBoundary);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderVelocityContextStopsAtNeighborSpan",
      TestVerticalJumpShapeConstraintBuilderVelocityContextStopsAtNeighborSpan);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderContextOnlyEmptySpanSkipsContext",
      TestVerticalJumpShapeConstraintBuilderContextOnlyEmptySpanSkipsContext);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderMergesOverlappingSpans",
      TestVerticalJumpShapeConstraintBuilderMergesOverlappingSpans);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderMissingAnchorSkipsBoundary",
      TestVerticalJumpShapeConstraintBuilderMissingAnchorSkipsBoundary);
    RunTest(
      "TestVerticalJumpShapeConstraintBuilderOneStateSpanStillAddsContinuity",
      TestVerticalJumpShapeConstraintBuilderOneStateSpanStillAddsContinuity);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
