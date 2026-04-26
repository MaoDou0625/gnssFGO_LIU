#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <functional>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/BodyZBidirectionalJumpDetector.h"
#include "offline_lc_minimal/core/SequentialNhcJumpDetector.h"
#include "offline_lc_minimal/core/SparseVerticalJumpPlanner.h"
#include "offline_lc_minimal/core/VerticalInsideBiasAdapter.h"
#include "offline_lc_minimal/factor/HorizontalPositionFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalSpecificForceFactor.h"
#include "offline_lc_minimal/factor/VerticalInsideKinematicFactor.h"
#include "offline_lc_minimal/factor/VerticalRtkPreintegrationFeedbackFactor.h"

namespace {

using offline_lc_minimal::DefaultConfig;
using offline_lc_minimal::BodyZBidirectionalJumpDetector;
using offline_lc_minimal::ImuSample;
using offline_lc_minimal::LoadConfigFile;
using offline_lc_minimal::ReferenceNodeState;
using offline_lc_minimal::SequentialNhcJumpDetector;
using offline_lc_minimal::SparseVerticalJumpPlanner;
using offline_lc_minimal::VerticalInsideBiasAdapter;
using offline_lc_minimal::VerticalVzReferenceSample;
using offline_lc_minimal::factor::HorizontalPositionFactor;
using offline_lc_minimal::factor::StaticVerticalSpecificForceFactor;
using offline_lc_minimal::factor::VerticalInsideKinematicFactor;
using offline_lc_minimal::factor::VerticalRtkPreintegrationFeedbackFactor;

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

void ExpectMatrixNear(
  const gtsam::Matrix &actual,
  const gtsam::Matrix &expected,
  const double tolerance,
  const std::string &message) {
  if (actual.rows() != expected.rows() || actual.cols() != expected.cols()) {
    throw std::runtime_error(message + ": matrix shape mismatch");
  }
  const double max_error = (actual - expected).cwiseAbs().maxCoeff();
  if (max_error > tolerance) {
    throw std::runtime_error(message + ": max_error=" + std::to_string(max_error));
  }
}

std::filesystem::path WriteTempConfig(const std::string &contents, const std::string &filename) {
  const std::filesystem::path path = std::filesystem::temp_directory_path() / filename;
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to create temp config");
  }
  stream << contents;
  return path;
}

std::string MakeVerticalFeedbackConfigPrefix() {
  return std::string()
         + "enable_gnss=true\n"
         + "enable_global_acc_bias=true\n"
         + "enable_vertical_acc_bias_gm_process=true\n"
         + "enable_vertical_rtk_preintegration_feedback=true\n";
}

gtsam::PreintegratedImuMeasurements MakeZeroPreintegratedMeasurements() {
  auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
  params->accelerometerCovariance = 1e-4 * gtsam::I_3x3;
  params->gyroscopeCovariance = 1e-6 * gtsam::I_3x3;
  params->integrationCovariance = 1e-8 * gtsam::I_3x3;
  params->biasAccCovariance = 1e-10 * gtsam::I_3x3;
  params->biasOmegaCovariance = 1e-12 * gtsam::I_3x3;
  params->biasAccOmegaInt = gtsam::Matrix66::Zero();

  gtsam::imuBias::ConstantBias bias;
  gtsam::PreintegratedImuMeasurements measurements(params, bias);
  for (int sample_index = 0; sample_index < 5; ++sample_index) {
    measurements.integrateMeasurement(gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3::Zero(), 0.01);
  }
  return measurements;
}

VerticalRtkPreintegrationFeedbackFactor MakeFactor(
  const gtsam::PreintegratedImuMeasurements &measurements,
  const double gain_scale,
  const double vertical_residual_m) {
  return VerticalRtkPreintegrationFeedbackFactor(
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    measurements,
    0.8,
    0.1,
    vertical_residual_m,
    gain_scale,
    2.0,
    3.0,
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(1e-3, 1e-3, 1e-2, 1e-4)));
}

ReferenceNodeState MakeReferenceNodeState(
  const double time_s,
  const gtsam::Pose3 &pose,
  const gtsam::Vector3 &velocity,
  const gtsam::imuBias::ConstantBias &bias = gtsam::imuBias::ConstantBias()) {
  ReferenceNodeState state;
  state.time_s = time_s;
  state.pose = pose;
  state.velocity = velocity;
  state.bias = bias;
  return state;
}

void TestVerticalFeedbackRequiresGnss() {
  const auto config_path = WriteTempConfig(
    MakeVerticalFeedbackConfigPrefix() + "enable_gnss=false\n",
    "vertical_rtk_feedback_requires_gnss.cfg");
  bool threw = false;
  try {
    [[maybe_unused]] const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  } catch (const std::exception &) {
    threw = true;
  }
  std::filesystem::remove(config_path);
  ExpectTrue(threw, "vertical RTK feedback should require GNSS");
}

void TestVerticalFeedbackRequiresVerticalGmBiasProcess() {
  const auto config_path = WriteTempConfig(
    "enable_gnss=true\n"
    "enable_global_acc_bias=true\n"
    "enable_vertical_rtk_preintegration_feedback=true\n"
    "enable_vertical_acc_bias_gm_process=false\n",
    "vertical_rtk_feedback_requires_vertical_gm.cfg");
  bool threw = false;
  try {
    [[maybe_unused]] const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  } catch (const std::exception &) {
    threw = true;
  }
  std::filesystem::remove(config_path);
  ExpectTrue(threw, "vertical RTK feedback should require the vertical GM bias process");
}

void TestVerticalFeedbackRejectsSegmentFeedbackCompatibility() {
  const auto config_path = WriteTempConfig(
    MakeVerticalFeedbackConfigPrefix() + "enable_segment_error_feedback=true\n",
    "vertical_rtk_feedback_segment_conflict.cfg");
  bool threw = false;
  try {
    [[maybe_unused]] const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  } catch (const std::exception &) {
    threw = true;
  }
  std::filesystem::remove(config_path);
  ExpectTrue(threw, "vertical RTK feedback should reject segment feedback mode");
}

void TestInitialStaticVerticalSpecificForceLoads() {
  const auto config_path = WriteTempConfig(
    "static_alignment_duration_s=100.0\n"
    "enable_initial_static_vertical_specific_force=true\n"
    "initial_static_vertical_specific_force_sigma_mps2=0.0125\n",
    "initial_static_vertical_specific_force_loads.cfg");
  const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  std::filesystem::remove(config_path);
  ExpectTrue(
    config.enable_initial_static_vertical_specific_force,
    "vertical static specific-force flag should load");
  ExpectNear(
    config.initial_static_vertical_specific_force_sigma_mps2,
    0.0125,
    1e-12,
    "vertical static specific-force sigma should load");
}

void TestInitialStaticVerticalSpecificForceRequiresStaticAlignment() {
  const auto config_path = WriteTempConfig(
    "enable_initial_static_vertical_specific_force=true\n",
    "initial_static_vertical_specific_force_requires_alignment.cfg");
  bool threw = false;
  try {
    [[maybe_unused]] const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  } catch (const std::exception &) {
    threw = true;
  }
  std::filesystem::remove(config_path);
  ExpectTrue(threw, "vertical static specific-force should require a positive static alignment duration");
}

void TestReserveVerticalVelocityInterfaceIsAccepted() {
  const auto config_path = WriteTempConfig(
    MakeVerticalFeedbackConfigPrefix() + "reserve_vertical_velocity_feedback_interface=true\n",
    "vertical_rtk_feedback_reserved_velocity.cfg");
  const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  std::filesystem::remove(config_path);
  ExpectTrue(config.reserve_vertical_velocity_feedback_interface, "reserved velocity interface flag should load");
  ExpectTrue(config.enable_vertical_rtk_preintegration_feedback, "vertical RTK feedback should remain enabled");
}

void TestVerticalFeedbackMinIntervalLoads() {
  const auto config_path = WriteTempConfig(
    MakeVerticalFeedbackConfigPrefix() + "vertical_rtk_feedback_min_interval_s=2.5\n",
    "vertical_rtk_feedback_min_interval.cfg");
  const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  std::filesystem::remove(config_path);
  ExpectNear(
    config.vertical_rtk_feedback_min_interval_s,
    2.5,
    1e-12,
    "vertical RTK feedback minimum interval should load");
}

void TestVerticalFeedbackRejectsNegativeMinInterval() {
  const auto config_path = WriteTempConfig(
    MakeVerticalFeedbackConfigPrefix() + "vertical_rtk_feedback_min_interval_s=-0.1\n",
    "vertical_rtk_feedback_negative_min_interval.cfg");
  bool threw = false;
  try {
    [[maybe_unused]] const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  } catch (const std::exception &) {
    threw = true;
  }
  std::filesystem::remove(config_path);
  ExpectTrue(threw, "vertical RTK feedback should reject a negative minimum interval");
}

void TestSequentialRecoveryConfigLoads() {
  const auto config_path = WriteTempConfig(
    MakeVerticalFeedbackConfigPrefix()
    + "vertical_local_recovery_max_iterations=6\n"
      "vertical_global_vz_window_s=1.5\n"
      "vertical_global_vz_smooth_window_s=1.2\n"
      "vertical_jump_candidate_min_separation_s=1.1\n"
      "vertical_jump_max_candidates_per_segment=7\n"
      "vertical_jump_max_selected_points_per_segment=2\n"
      "vertical_jump_hold_window_s=2.5\n"
      "vertical_jump_step_min_threshold_mps=0.07\n"
      "vertical_jump_vz_prior_sigma_mps=0.025\n"
      "vertical_jump_window_default_padding_states=2\n"
      "vertical_jump_window_support_ratio=0.45\n"
      "vertical_jump_window_max_duration_s=0.4\n"
      "vertical_jump_window_max_points=9\n"
      "vertical_jump_window_tail_target_s=1.4\n"
      "vertical_jump_window_velocity_smoothness_weight=12.0\n"
      "vertical_jump_window_height_integral_weight=4.0\n"
      "vertical_jump_window_ref_weight=1.5\n"
      "vertical_jump_future_trend_window_s=18.0\n"
      "vertical_jump_future_trend_min_fix_count=6\n"
      "vertical_jump_future_trend_mean_weight=0.8\n"
      "vertical_jump_future_trend_slope_weight=120.0\n"
      "enable_nhc_jump_reference=true\n"
      "nhc_history_half_life_s=12.0\n"
      "nhc_history_max_age_s=45.0\n"
      "nhc_body_vy_min_threshold_mps=0.04\n"
      "nhc_body_vz_min_threshold_mps=0.004\n"
      "nhc_body_vz_max_threshold_mps=0.06\n"
      "nhc_body_vy_percentile_scale=1.6\n"
      "nhc_body_vz_percentile_scale=3.5\n"
      "nhc_jump_min_separation_s=1.25\n",
    "vertical_rtk_sequential_recovery_loads.cfg");
  const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  std::filesystem::remove(config_path);
  ExpectNear(config.vertical_local_recovery_max_iterations, 6, 0.0, "local recovery iteration count should load");
  ExpectNear(config.vertical_global_vz_window_s, 1.5, 1e-12, "global vz diff window should load");
  ExpectNear(config.vertical_global_vz_smooth_window_s, 1.2, 1e-12, "global vz smooth window should load");
  ExpectNear(
    config.vertical_jump_candidate_min_separation_s,
    1.1,
    1e-12,
    "jump candidate minimum separation should load");
  ExpectNear(
    config.vertical_jump_max_candidates_per_segment,
    7,
    0.0,
    "jump candidate cap should load");
  ExpectNear(
    config.vertical_jump_max_selected_points_per_segment,
    2,
    0.0,
    "jump selected-point cap should load");
  ExpectNear(config.vertical_jump_hold_window_s, 2.5, 1e-12, "jump hold window should load");
  ExpectNear(config.vertical_jump_step_min_threshold_mps, 0.07, 1e-12, "jump threshold floor should load");
  ExpectNear(config.vertical_jump_vz_prior_sigma_mps, 0.025, 1e-12, "jump vz prior sigma should load");
  ExpectNear(
    config.vertical_jump_window_default_padding_states,
    2,
    0.0,
    "jump window default padding should load");
  ExpectNear(config.vertical_jump_window_support_ratio, 0.45, 1e-12, "jump window support ratio should load");
  ExpectNear(config.vertical_jump_window_max_duration_s, 0.4, 1e-12, "jump window max duration should load");
  ExpectNear(config.vertical_jump_window_max_points, 9, 0.0, "jump window max points should load");
  ExpectNear(config.vertical_jump_window_tail_target_s, 1.4, 1e-12, "jump window tail target should load");
  ExpectNear(
    config.vertical_jump_window_velocity_smoothness_weight,
    12.0,
    1e-12,
    "jump window velocity smoothness weight should load");
  ExpectNear(
    config.vertical_jump_window_height_integral_weight,
    4.0,
    1e-12,
    "jump window height integral weight should load");
  ExpectNear(config.vertical_jump_window_ref_weight, 1.5, 1e-12, "jump window ref weight should load");
  ExpectNear(config.vertical_jump_future_trend_window_s, 18.0, 1e-12, "future trend window should load");
  ExpectNear(config.vertical_jump_future_trend_min_fix_count, 6, 0.0, "future trend minimum fix count should load");
  ExpectNear(config.vertical_jump_future_trend_mean_weight, 0.8, 1e-12, "future trend mean weight should load");
  ExpectNear(config.vertical_jump_future_trend_slope_weight, 120.0, 1e-12, "future trend slope weight should load");
  ExpectTrue(config.enable_nhc_jump_reference, "NHC jump reference flag should load");
  ExpectNear(config.nhc_history_half_life_s, 12.0, 1e-12, "NHC half-life should load");
  ExpectNear(config.nhc_history_max_age_s, 45.0, 1e-12, "NHC max age should load");
  ExpectNear(config.nhc_body_vy_min_threshold_mps, 0.04, 1e-12, "NHC vy threshold floor should load");
  ExpectNear(config.nhc_body_vz_min_threshold_mps, 0.004, 1e-12, "NHC vz threshold floor should load");
  ExpectNear(config.nhc_body_vz_max_threshold_mps, 0.06, 1e-12, "NHC vz threshold cap should load");
  ExpectNear(config.nhc_body_vy_percentile_scale, 1.6, 1e-12, "NHC vy percentile scale should load");
  ExpectNear(config.nhc_body_vz_percentile_scale, 3.5, 1e-12, "NHC vz percentile scale should load");
  ExpectNear(config.nhc_jump_min_separation_s, 1.25, 1e-12, "NHC jump separation should load");
}

void TestSequentialRecoveryRejectsNonPositiveIterationCount() {
  const auto config_path = WriteTempConfig(
    MakeVerticalFeedbackConfigPrefix() + "vertical_local_recovery_max_iterations=0\n",
    "vertical_rtk_sequential_recovery_iterations_invalid.cfg");
  bool threw = false;
  try {
    [[maybe_unused]] const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  } catch (const std::exception &) {
    threw = true;
  }
  std::filesystem::remove(config_path);
  ExpectTrue(threw, "sequential recovery should reject non-positive local iteration count");
}

void TestSequentialRecoveryRejectsInvalidNhcHistoryWindow() {
  const auto config_path = WriteTempConfig(
    MakeVerticalFeedbackConfigPrefix()
    + "nhc_history_half_life_s=20.0\n"
      "nhc_history_max_age_s=10.0\n",
    "vertical_rtk_invalid_nhc_history.cfg");
  bool threw = false;
  try {
    [[maybe_unused]] const auto config = LoadConfigFile(config_path.string(), DefaultConfig());
  } catch (const std::exception &) {
    threw = true;
  }
  std::filesystem::remove(config_path);
  ExpectTrue(threw, "sequential recovery should reject NHC max age shorter than its half-life");
}

void TestHorizontalPositionFactorIgnoresVerticalTranslation() {
  const HorizontalPositionFactor factor(
    1,
    gtsam::Point2(1.0, 2.0),
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.2)));
  const gtsam::Pose3 pose(
    gtsam::Rot3::RzRyRx(0.2, -0.1, 0.05),
    gtsam::Point3(1.1, 1.8, 9.5));
  gtsam::Matrix jacobian;
  const gtsam::Vector residual = factor.evaluateError(pose, jacobian);
  ExpectNear(residual[0], 0.1, 1e-12, "horizontal factor should use east translation");
  ExpectNear(residual[1], -0.2, 1e-12, "horizontal factor should use north translation");
  ExpectTrue(jacobian.rows() == 2 && jacobian.cols() == 6, "horizontal factor Jacobian shape should be 2x6");
  const auto numerical_jacobian =
    gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
      [&](const gtsam::Pose3 &value) { return factor.evaluateError(value); },
      pose,
      1e-6);
  ExpectMatrixNear(jacobian, numerical_jacobian, 5e-5, "horizontal factor Jacobian should match numerical derivative");
}

StaticVerticalSpecificForceFactor MakeStaticVerticalSpecificForceFactor(
  const double measured_acc_z_mps2,
  const Eigen::Vector3d &gravity_enu = Eigen::Vector3d(0.0, 0.0, 9.81)) {
  return StaticVerticalSpecificForceFactor(
    1,
    2,
    measured_acc_z_mps2,
    gravity_enu,
    gtsam::noiseModel::Isotropic::Sigma(1, 1e-2));
}

void TestStaticVerticalSpecificForceResidualNearZeroWhenBiasMatches() {
  const auto factor = MakeStaticVerticalSpecificForceFactor(9.81 + 0.02);
  const gtsam::Pose3 pose;
  const gtsam::imuBias::ConstantBias bias(gtsam::Vector3(0.0, 0.0, 0.02), gtsam::Vector3::Zero());
  const gtsam::Vector residual = factor.evaluateError(pose, bias);
  ExpectNear(residual[0], 0.0, 1e-9, "matching ba_z should zero the static vertical residual");
}

void TestStaticVerticalSpecificForceRespondsLinearlyToBaz() {
  const auto factor = MakeStaticVerticalSpecificForceFactor(9.81);
  const gtsam::Pose3 pose;
  const gtsam::imuBias::ConstantBias low_bias(gtsam::Vector3(0.0, 0.0, -0.01), gtsam::Vector3::Zero());
  const gtsam::imuBias::ConstantBias high_bias(gtsam::Vector3(0.0, 0.0, 0.03), gtsam::Vector3::Zero());
  const gtsam::Vector residual_low = factor.evaluateError(pose, low_bias);
  const gtsam::Vector residual_high = factor.evaluateError(pose, high_bias);
  ExpectNear(
    residual_high[0] - residual_low[0],
    0.04,
    1e-9,
    "static vertical residual should vary linearly with ba_z");
}

void TestStaticVerticalSpecificForceIgnoresYawButRespondsToRollPitch() {
  const auto factor = MakeStaticVerticalSpecificForceFactor(9.81);
  const gtsam::imuBias::ConstantBias bias;
  const gtsam::Pose3 yaw_only(gtsam::Rot3::Yaw(0.3), gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Pose3 roll_pitch(gtsam::Rot3::RzRyRx(0.0, 0.03, -0.02), gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Vector yaw_residual = factor.evaluateError(yaw_only, bias);
  const gtsam::Vector roll_pitch_residual = factor.evaluateError(roll_pitch, bias);
  ExpectNear(yaw_residual[0], 0.0, 1e-9, "pure yaw should not change static vertical residual");
  ExpectTrue(
    std::abs(roll_pitch_residual[0]) > 1e-3,
    "roll or pitch should change the static vertical residual");
}

void TestStaticVerticalSpecificForceJacobianMatchesNumericalDerivative() {
  const auto factor = MakeStaticVerticalSpecificForceFactor(9.81 + 0.015);
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.1, -0.05, 0.02), gtsam::Point3(0.1, -0.2, 0.3));
  const gtsam::imuBias::ConstantBias bias(
    gtsam::Vector3(0.01, -0.02, 0.015),
    gtsam::Vector3(0.001, -0.002, 0.003));

  gtsam::Matrix h_pose;
  gtsam::Matrix h_bias;
  [[maybe_unused]] const gtsam::Vector residual = factor.evaluateError(pose, bias, h_pose, h_bias);

  const auto numerical_pose =
    gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
      [&](const gtsam::Pose3 &value) { return factor.evaluateError(value, bias); },
      pose,
      1e-6);
  const auto numerical_bias =
    gtsam::numericalDerivative11<gtsam::Vector, gtsam::imuBias::ConstantBias>(
      [&](const gtsam::imuBias::ConstantBias &value) { return factor.evaluateError(pose, value); },
      bias,
      1e-6);

  ExpectMatrixNear(h_pose, numerical_pose, 5e-5, "static vertical pose Jacobian should match numerical derivative");
  ExpectMatrixNear(h_bias, numerical_bias, 5e-5, "static vertical bias Jacobian should match numerical derivative");
}

void TestPureYawLeavesRollPitchFeedbackResidualZero() {
  const auto measurements = MakeZeroPreintegratedMeasurements();
  const auto factor = MakeFactor(measurements, 1.0, 0.0);
  const gtsam::Pose3 pose_i;
  const gtsam::Pose3 pose_j(
    gtsam::Rot3::Yaw(0.15),
    gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Vector3 velocity = gtsam::Vector3::Zero();
  const gtsam::imuBias::ConstantBias bias;
  const gtsam::Vector residual =
    factor.evaluateError(pose_i, velocity, bias, pose_j, velocity, bias, gtsam::Vector3::Zero());
  ExpectNear(residual[0], 0.0, 1e-9, "pure yaw should not affect roll feedback residual");
  ExpectNear(residual[1], 0.0, 1e-9, "pure yaw should not affect pitch feedback residual");
}

void TestGainScaleAmplifiesAttitudeAndBiasTargets() {
  const auto measurements = MakeZeroPreintegratedMeasurements();
  const auto low_gain_factor = MakeFactor(measurements, 1.0, 0.3);
  const auto high_gain_factor = MakeFactor(measurements, 2.0, 0.3);
  const gtsam::Pose3 pose_i;
  const gtsam::Pose3 pose_j(
    gtsam::Rot3::RzRyRx(0.0, -0.03, 0.02),
    gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Vector3 velocity = gtsam::Vector3::Zero();
  const gtsam::imuBias::ConstantBias bias;

  const gtsam::Vector low_gain_residual =
    low_gain_factor.evaluateError(pose_i, velocity, bias, pose_j, velocity, bias, gtsam::Vector3::Zero());
  const gtsam::Vector high_gain_residual =
    high_gain_factor.evaluateError(pose_i, velocity, bias, pose_j, velocity, bias, gtsam::Vector3::Zero());

  ExpectNear(high_gain_residual[0], 2.0 * low_gain_residual[0], 1e-9, "gain scale should double roll residual");
  ExpectNear(high_gain_residual[1], 2.0 * low_gain_residual[1], 1e-9, "gain scale should double pitch residual");
  ExpectNear(high_gain_residual[2], 2.0 * low_gain_residual[2], 1e-9, "gain scale should double dp_z residual");
  ExpectNear(high_gain_residual[3], 2.0 * low_gain_residual[3], 1e-9, "gain scale should double baz target");
}

void TestVerticalResidualFeedsDpZRow() {
  const auto measurements = MakeZeroPreintegratedMeasurements();
  const auto factor = MakeFactor(measurements, 1.5, 0.2);
  const gtsam::Vector residual = factor.evaluateError(
    gtsam::Pose3(),
    gtsam::Vector3::Zero(),
    gtsam::imuBias::ConstantBias(),
    gtsam::Pose3(),
    gtsam::Vector3::Zero(),
    gtsam::imuBias::ConstantBias(),
    gtsam::Vector3::Zero());
  ExpectNear(residual[2], 0.3, 1e-9, "vertical residual should drive the dp_z row");
}

void TestJacobianMatchesNumericalDerivative() {
  const auto measurements = MakeZeroPreintegratedMeasurements();
  const auto factor = MakeFactor(measurements, 1.5, -0.2);
  const gtsam::Pose3 pose_i(
    gtsam::Rot3::RzRyRx(0.01, -0.02, 0.03),
    gtsam::Point3(0.2, -0.1, 0.05));
  const gtsam::Vector3 vel_i(0.1, -0.2, 0.3);
  const gtsam::imuBias::ConstantBias bias_i(
    gtsam::Vector3(0.01, -0.02, 0.03),
    gtsam::Vector3(0.001, -0.002, 0.003));
  const gtsam::Pose3 pose_j(
    gtsam::Rot3::RzRyRx(-0.01, 0.015, -0.02),
    gtsam::Point3(0.25, -0.12, 0.06));
  const gtsam::Vector3 vel_j(-0.05, 0.04, -0.03);
  const gtsam::imuBias::ConstantBias bias_j(
    gtsam::Vector3(0.005, -0.015, 0.025),
    gtsam::Vector3(0.0, 0.0, 0.0));
  const gtsam::Vector3 global_acc_bias(0.002, -0.003, 0.004);

  gtsam::Matrix h1;
  gtsam::Matrix h2;
  gtsam::Matrix h3;
  gtsam::Matrix h4;
  gtsam::Matrix h5;
  gtsam::Matrix h6;
  gtsam::Matrix h7;
  [[maybe_unused]] const gtsam::Vector residual =
    factor.evaluateError(pose_i, vel_i, bias_i, pose_j, vel_j, bias_j, global_acc_bias, h1, h2, h3, h4, h5, h6, h7);

  const std::function<gtsam::Vector(const gtsam::Pose3 &)> error_pose_i =
    [&](const gtsam::Pose3 &value) {
      return factor.evaluateError(value, vel_i, bias_i, pose_j, vel_j, bias_j, global_acc_bias);
    };
  const std::function<gtsam::Vector(const gtsam::Vector3 &)> error_vel_i =
    [&](const gtsam::Vector3 &value) {
      return factor.evaluateError(pose_i, value, bias_i, pose_j, vel_j, bias_j, global_acc_bias);
    };
  const std::function<gtsam::Vector(const gtsam::imuBias::ConstantBias &)> error_bias_i =
    [&](const gtsam::imuBias::ConstantBias &value) {
      return factor.evaluateError(pose_i, vel_i, value, pose_j, vel_j, bias_j, global_acc_bias);
    };
  const std::function<gtsam::Vector(const gtsam::Pose3 &)> error_pose_j =
    [&](const gtsam::Pose3 &value) {
      return factor.evaluateError(pose_i, vel_i, bias_i, value, vel_j, bias_j, global_acc_bias);
    };
  const std::function<gtsam::Vector(const gtsam::Vector3 &)> error_vel_j =
    [&](const gtsam::Vector3 &value) {
      return factor.evaluateError(pose_i, vel_i, bias_i, pose_j, value, bias_j, global_acc_bias);
    };
  const std::function<gtsam::Vector(const gtsam::imuBias::ConstantBias &)> error_bias_j =
    [&](const gtsam::imuBias::ConstantBias &value) {
      return factor.evaluateError(pose_i, vel_i, bias_i, pose_j, vel_j, value, global_acc_bias);
    };
  const std::function<gtsam::Vector(const gtsam::Vector3 &)> error_global_acc_bias =
    [&](const gtsam::Vector3 &value) {
      return factor.evaluateError(pose_i, vel_i, bias_i, pose_j, vel_j, bias_j, value);
    };

  const auto num_h1 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    error_pose_i,
    pose_i,
    1e-6);
  const auto num_h2 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
    error_vel_i,
    vel_i,
    1e-6);
  const auto num_h3 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::imuBias::ConstantBias>(
    error_bias_i,
    bias_i,
    1e-6);
  const auto num_h4 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    error_pose_j,
    pose_j,
    1e-6);
  const auto num_h5 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
    error_vel_j,
    vel_j,
    1e-6);
  const auto num_h6 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::imuBias::ConstantBias>(
    error_bias_j,
    bias_j,
    1e-6);
  const auto num_h7 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(
    error_global_acc_bias,
    global_acc_bias,
    1e-6);

  ExpectMatrixNear(h1, num_h1, 5e-5, "pose_i Jacobian should match numerical derivative");
  ExpectMatrixNear(h2, num_h2, 5e-5, "vel_i Jacobian should match numerical derivative");
  ExpectMatrixNear(h3, num_h3, 5e-5, "bias_i Jacobian should match numerical derivative");
  ExpectMatrixNear(h4, num_h4, 5e-5, "pose_j Jacobian should match numerical derivative");
  ExpectMatrixNear(h5, num_h5, 5e-5, "vel_j Jacobian should match numerical derivative");
  ExpectMatrixNear(h6, num_h6, 5e-5, "bias_j Jacobian should match numerical derivative");
  ExpectMatrixNear(h7, num_h7, 5e-5, "global_acc_bias Jacobian should match numerical derivative");
}

void TestVerticalInsideKinematicFactorTracksPitchRollVzAndBaz() {
  const VerticalInsideKinematicFactor factor(
    1,
    2,
    3,
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, -0.02, 0.01), gtsam::Point3(1.0, 2.0, 3.0)),
    gtsam::Vector3(0.1, -0.2, 0.3),
    gtsam::imuBias::ConstantBias(gtsam::Vector3(0.0, 0.0, 0.02), gtsam::Vector3::Zero()),
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(1e-3, 1e-3, 1e-2, 1e-3)));
  const gtsam::Pose3 pose(
    gtsam::Rot3::RzRyRx(0.15, -0.015, 0.03),
    gtsam::Point3(4.0, 5.0, 30.0));
  const gtsam::Vector3 velocity(-1.0, 2.0, 0.55);
  const gtsam::imuBias::ConstantBias bias(gtsam::Vector3(0.3, -0.4, 0.07), gtsam::Vector3::Zero());
  const gtsam::Vector residual = factor.evaluateError(pose, velocity, bias);
  ExpectTrue(std::abs(residual[0]) > 1e-6, "roll residual should be active");
  ExpectTrue(std::abs(residual[1]) > 1e-6, "pitch residual should be active");
  ExpectNear(residual[2], 0.25, 1e-9, "vertical inside kinematic factor should track vz");
  ExpectNear(residual[3], 0.05, 1e-9, "vertical inside kinematic factor should track ba_z");
}

void TestVerticalInsideKinematicFactorIgnoresYawHorizontalTranslationAndXyBias() {
  const gtsam::Pose3 reference_pose(
    gtsam::Rot3::Yaw(0.0).compose(gtsam::Rot3::Pitch(0.01)).compose(gtsam::Rot3::Roll(-0.02)),
    gtsam::Point3(1.0, 2.0, 3.0));
  const VerticalInsideKinematicFactor factor(
    1,
    2,
    3,
    reference_pose,
    gtsam::Vector3(0.2, -0.1, 0.3),
    gtsam::imuBias::ConstantBias(gtsam::Vector3(0.01, -0.02, 0.03), gtsam::Vector3::Zero()),
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(1e-3, 1e-3, 1e-2, 1e-3)));
  const gtsam::Pose3 pose(
    gtsam::Rot3::Yaw(0.25).compose(gtsam::Rot3::Pitch(0.01)).compose(gtsam::Rot3::Roll(-0.02)),
    gtsam::Point3(10.0, -4.0, 999.0));
  const gtsam::Vector3 velocity(-3.0, 4.0, 0.3);
  const gtsam::imuBias::ConstantBias bias(gtsam::Vector3(0.4, -0.5, 0.03), gtsam::Vector3::Zero());
  const gtsam::Vector residual = factor.evaluateError(pose, velocity, bias);
  ExpectNear(residual[0], 0.0, 1e-9, "pure yaw should not affect roll residual");
  ExpectNear(residual[1], 0.0, 1e-9, "pure yaw should not affect pitch residual");
  ExpectNear(residual[2], 0.0, 1e-9, "matching vz should zero the vz residual");
  ExpectNear(residual[3], 0.0, 1e-9, "xy bias changes should not affect ba_z residual");
}

void TestVerticalInsideKinematicFactorJacobianMatchesNumericalDerivative() {
  const VerticalInsideKinematicFactor factor(
    1,
    2,
    3,
    gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.03, 0.02, -0.01), gtsam::Point3(0.5, -0.2, 1.0)),
    gtsam::Vector3(0.1, 0.2, -0.3),
    gtsam::imuBias::ConstantBias(gtsam::Vector3(0.01, -0.02, 0.03), gtsam::Vector3::Zero()),
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(1e-3, 1e-3, 1e-2, 1e-3)));
  const gtsam::Pose3 pose(
    gtsam::Rot3::RzRyRx(0.04, -0.015, 0.02),
    gtsam::Point3(0.3, 0.7, 1.25));
  const gtsam::Vector3 velocity(0.2, -0.1, 0.4);
  const gtsam::imuBias::ConstantBias bias(gtsam::Vector3(0.02, -0.03, 0.05), gtsam::Vector3(0.0, 0.0, 0.0));
  gtsam::Matrix h1;
  gtsam::Matrix h2;
  gtsam::Matrix h3;
  [[maybe_unused]] const gtsam::Vector residual = factor.evaluateError(pose, velocity, bias, h1, h2, h3);
  const auto numerical_h1 =
    gtsam::numericalDerivative31<gtsam::Vector4, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
      [&](const gtsam::Pose3 &value, const gtsam::Vector3 &v, const gtsam::imuBias::ConstantBias &b) {
        return factor.evaluateError(value, v, b);
      },
      pose,
      velocity,
      bias,
      1e-6);
  const auto numerical_h2 =
    gtsam::numericalDerivative32<gtsam::Vector4, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
      [&](const gtsam::Pose3 &p, const gtsam::Vector3 &value, const gtsam::imuBias::ConstantBias &b) {
        return factor.evaluateError(p, value, b);
      },
      pose,
      velocity,
      bias,
      1e-6);
  const auto numerical_h3 =
    gtsam::numericalDerivative33<gtsam::Vector4, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
      [&](const gtsam::Pose3 &p, const gtsam::Vector3 &v, const gtsam::imuBias::ConstantBias &value) {
        return factor.evaluateError(p, v, value);
      },
      pose,
      velocity,
      bias,
      1e-6);
  ExpectMatrixNear(h1, numerical_h1, 5e-5, "vertical inside kinematic pose Jacobian should match numerical derivative");
  ExpectMatrixNear(h2, numerical_h2, 5e-5, "vertical inside kinematic velocity Jacobian should match numerical derivative");
  ExpectMatrixNear(h3, numerical_h3, 5e-5, "vertical inside kinematic bias Jacobian should match numerical derivative");
}

void TestNhcThresholdBootstrapRespectsMinimumFloors() {
  auto config = DefaultConfig();
  config.enable_nhc_jump_reference = true;
  SequentialNhcJumpDetector detector(config);
  std::vector<ReferenceNodeState> states{
    MakeReferenceNodeState(0.0, gtsam::Pose3(), gtsam::Vector3(0.001, 0.002, 0.0005)),
    MakeReferenceNodeState(1.0, gtsam::Pose3(), gtsam::Vector3(-0.001, -0.002, 0.0002)),
    MakeReferenceNodeState(2.0, gtsam::Pose3(), gtsam::Vector3(0.002, 0.001, -0.0003)),
  };
  detector.SeedWithConfirmedStates(states, 0U, states.size() - 1U);
  const auto thresholds = detector.CurrentThresholds(2.0);
  ExpectNear(
    thresholds.body_vy_threshold_mps,
    config.nhc_body_vy_min_threshold_mps,
    1e-12,
    "NHC vy threshold should respect the configured floor");
  ExpectNear(
    thresholds.body_vz_threshold_mps,
    config.nhc_body_vz_min_threshold_mps,
    1e-12,
    "NHC vz threshold should respect the configured floor");
}

void TestNhcJumpAnchorFindsFirstThresholdCrossingFromBodyVz() {
  auto config = DefaultConfig();
  config.enable_nhc_jump_reference = true;
  config.nhc_jump_min_separation_s = 0.5;
  SequentialNhcJumpDetector detector(config);
  std::vector<ReferenceNodeState> states{
    MakeReferenceNodeState(0.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.0)),
    MakeReferenceNodeState(1.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.0)),
    MakeReferenceNodeState(2.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.0)),
    MakeReferenceNodeState(3.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.005)),
    MakeReferenceNodeState(4.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.006)),
    MakeReferenceNodeState(5.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.006)),
  };
  detector.SeedWithConfirmedStates(states, 0U, 2U);
  const auto jump_anchor = detector.FindJumpAnchor(states, 2U, 5U);
  ExpectTrue(
    jump_anchor.has_value(),
    "NHC jump detector should find an anchor when body vz exceeds the threshold");
  ExpectNear(
    static_cast<double>(*jump_anchor),
    3.0,
    0.0,
    "NHC jump detector should return the earliest threshold crossing");
}

void TestNhcJumpAnchorCachesConfirmedWindowCrossing() {
  auto config = DefaultConfig();
  config.enable_nhc_jump_reference = true;
  config.nhc_jump_min_separation_s = 0.5;
  SequentialNhcJumpDetector detector(config);
  std::vector<ReferenceNodeState> states{
    MakeReferenceNodeState(0.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.0)),
    MakeReferenceNodeState(1.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.0)),
    MakeReferenceNodeState(2.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.0)),
    MakeReferenceNodeState(3.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, -0.20)),
    MakeReferenceNodeState(4.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, -0.21)),
  };
  detector.SeedWithConfirmedStates(states, 0U, 2U);
  detector.ObserveConfirmedWindow(states, 3U, 4U);
  const auto jump_anchor = detector.FindRecentJumpAnchor(0U, 4U);
  ExpectTrue(
    jump_anchor.has_value(),
    "NHC detector should cache jump anchors while confirmed windows are observed");
  ExpectNear(
    static_cast<double>(*jump_anchor),
    3.0,
    0.0,
    "cached NHC jump anchor should point to the body vz jump state");
}

void TestNhcJumpAnchorIgnoresBodyVyOnlyCrossing() {
  auto config = DefaultConfig();
  config.enable_nhc_jump_reference = true;
  config.nhc_jump_min_separation_s = 0.5;
  SequentialNhcJumpDetector detector(config);
  std::vector<ReferenceNodeState> states{
    MakeReferenceNodeState(0.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.0)),
    MakeReferenceNodeState(1.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.0)),
    MakeReferenceNodeState(2.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.0)),
    MakeReferenceNodeState(3.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.05, 0.0)),
    MakeReferenceNodeState(4.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.06, 0.0)),
    MakeReferenceNodeState(5.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.06, 0.0)),
  };
  detector.SeedWithConfirmedStates(states, 0U, 2U);
  const auto jump_anchor = detector.FindJumpAnchor(states, 2U, 5U);
  ExpectTrue(
    !jump_anchor.has_value(),
    "NHC jump detector should ignore body vy-only threshold crossings");
}

void TestSparseVerticalJumpPlannerBuildsSinglePeakCandidate() {
  auto config = DefaultConfig();
  config.vertical_jump_step_min_threshold_mps = 0.05;
  config.vertical_jump_candidate_min_separation_s = 1.0;
  config.vertical_jump_max_candidates_per_segment = 5;
  SparseVerticalJumpPlanner planner(config);

  std::vector<ReferenceNodeState> states{
    MakeReferenceNodeState(0.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.00)),
    MakeReferenceNodeState(1.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.00)),
    MakeReferenceNodeState(2.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.12)),
    MakeReferenceNodeState(3.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.13)),
    MakeReferenceNodeState(4.0, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.131)),
  };
  std::vector<VerticalVzReferenceSample> reference(states.size());
  for (std::size_t state_index = 0; state_index < states.size(); ++state_index) {
    reference[state_index].time_s = states[state_index].time_s;
    reference[state_index].valid = true;
    reference[state_index].vz_ref_global_mps = 0.0;
    reference[state_index].vz_ref_global_smoothed_mps = 0.0;
  }

  planner.SeedWithConfirmedStates(states, reference, 0U, 1U);
  const auto candidates = planner.BuildCandidates(states, reference, 0U, 4U, [](const std::size_t state_index) {
    return state_index == 2U;
  });
  ExpectNear(candidates.size(), 1.0, 0.0, "planner should emit one sparse jump candidate");
  ExpectNear(static_cast<double>(candidates.front().state_index), 2.0, 0.0, "candidate should point to jump peak");
  ExpectNear(
    candidates.front().delta_vz_init_mps,
    -0.12,
    1e-12,
    "candidate delta vz should oppose the mismatch jump");
  ExpectTrue(candidates.front().nhc_supported, "candidate should preserve NHC support bonus");
}

void TestSparseVerticalJumpPlannerBuildsWindowAroundPeakCandidate() {
  auto config = DefaultConfig();
  config.vertical_jump_step_min_threshold_mps = 0.05;
  config.vertical_jump_candidate_min_separation_s = 1.0;
  config.vertical_jump_max_candidates_per_segment = 5;
  config.vertical_jump_window_default_padding_states = 1;
  config.vertical_jump_window_max_duration_s = 0.35;
  config.vertical_jump_window_max_points = 7;
  SparseVerticalJumpPlanner planner(config);

  std::vector<ReferenceNodeState> states{
    MakeReferenceNodeState(0.00, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.00)),
    MakeReferenceNodeState(0.05, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, 0.00)),
    MakeReferenceNodeState(0.10, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, -0.18)),
    MakeReferenceNodeState(0.15, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, -0.19)),
    MakeReferenceNodeState(0.20, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, -0.18)),
    MakeReferenceNodeState(0.25, gtsam::Pose3(), gtsam::Vector3(0.0, 0.0, -0.18)),
  };
  std::vector<VerticalVzReferenceSample> reference(states.size());
  for (std::size_t state_index = 0; state_index < states.size(); ++state_index) {
    reference[state_index].time_s = states[state_index].time_s;
    reference[state_index].valid = true;
    reference[state_index].vz_ref_global_mps = 0.0;
    reference[state_index].vz_ref_global_smoothed_mps = 0.0;
  }

  planner.SeedWithConfirmedStates(states, reference, 0U, 1U);
  const auto windows = planner.BuildWindowCandidates(states, reference, 0U, 5U, [](const std::size_t state_index) {
    return state_index == 2U;
  });
  ExpectNear(windows.size(), 1.0, 0.0, "planner should emit one sparse jump window");
  ExpectNear(
    static_cast<double>(windows.front().center_state_index),
    2.0,
    0.0,
    "window center should point to jump peak");
  ExpectNear(
    static_cast<double>(windows.front().start_state_index),
    1.0,
    0.0,
    "window should include the state before the peak");
  ExpectNear(
    static_cast<double>(windows.front().end_state_index),
    4.0,
    0.0,
    "window should include redundant safety padding after the peak");
  ExpectNear(
    static_cast<double>(windows.front().point_count),
    4.0,
    0.0,
    "window point count should reflect the redundant correction span");
}

void TestBodyZSeedJumpDetectorFindsDownwardWindow() {
  auto config = DefaultConfig();
  config.body_z_jump_min_score_mps = 0.003;
  config.body_z_jump_threshold_ratio = 0.35;
  config.body_z_jump_support_ratio = 0.25;
  config.body_z_jump_max_window_duration_s = 0.75;
  config.body_z_jump_min_separation_s = 0.1;

  std::vector<ImuSample> imu_samples;
  for (int sample_index = 0; sample_index <= 600; ++sample_index) {
    const double time_s = 0.005 * static_cast<double>(sample_index);
    ImuSample sample;
    sample.time_s = time_s;
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);
    if (time_s >= 1.0 && time_s <= 1.10) {
      sample.accel_mps2.z() -= 1.0;
    }
    imu_samples.push_back(sample);
  }

  std::vector<double> state_timestamps;
  std::vector<ReferenceNodeState> seed_states;
  for (int state_index = 0; state_index <= 60; ++state_index) {
    const double time_s = 0.05 * static_cast<double>(state_index);
    state_timestamps.push_back(time_s);
    seed_states.push_back(MakeReferenceNodeState(time_s, gtsam::Pose3(), gtsam::Vector3::Zero()));
  }

  BodyZBidirectionalJumpDetector detector(config);
  const auto detection = detector.Detect(imu_samples, seed_states, state_timestamps, 0.0, 3.0);
  const auto down_window_it = std::find_if(
    detection.windows.begin(),
    detection.windows.end(),
    [](const auto &window) { return window.direction == "DOWN"; });
  ExpectTrue(down_window_it != detection.windows.end(), "body-z detector should find the downward velocity step");
  ExpectTrue(
    down_window_it->center_time_s > 0.8 && down_window_it->center_time_s < 1.3,
    "body-z detector center should be near the injected pulse");
  ExpectTrue(
    down_window_it->signed_delta_velocity_mps < 0.0,
    "DOWN body-z window should have a negative signed velocity delta");
  ExpectTrue(
    down_window_it->delta_vz_init_mps > 0.0,
    "DOWN body-z window should initialize an upward nav-z velocity correction");
}

void TestBodyZSeedJumpDetectorUsesSeedAttitudeGravityProjection() {
  auto config = DefaultConfig();
  std::vector<ImuSample> imu_samples;
  for (int sample_index = 0; sample_index <= 10; ++sample_index) {
    ImuSample sample;
    sample.time_s = 0.01 * static_cast<double>(sample_index);
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);
    imu_samples.push_back(sample);
  }
  const gtsam::Pose3 tilted_pose(gtsam::Rot3::RzRyRx(0.0, 0.2, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
  std::vector<double> state_timestamps{0.0, 0.1};
  std::vector<ReferenceNodeState> seed_states{
    MakeReferenceNodeState(0.0, tilted_pose, gtsam::Vector3::Zero()),
    MakeReferenceNodeState(0.1, tilted_pose, gtsam::Vector3::Zero()),
  };

  BodyZBidirectionalJumpDetector detector(config);
  const auto detection = detector.Detect(imu_samples, seed_states, state_timestamps, 0.0, 0.1);
  ExpectTrue(!detection.signal.empty(), "body-z detector should emit a signal");
  const double expected_gravity_projection = 9.81 * std::cos(0.2);
  ExpectNear(
    detection.signal.front().gravity_projection_z_mps2,
    expected_gravity_projection,
    1e-9,
    "body-z detector should use seed pitch when projecting gravity");
}

void TestVerticalInsideBiasAdapterUsesInsideResidualTrend() {
  auto config = DefaultConfig();
  config.enable_vertical_inside_bias_adaptation = true;
  config.vertical_inside_bias_window_s = 2.0;
  config.vertical_inside_bias_min_window_s = 1.0;
  config.vertical_inside_bias_min_observations = 3;
  config.vertical_inside_bias_update_interval_s = 0.1;
  config.vertical_inside_bias_gain = 0.5;
  config.vertical_inside_bias_max_delta_mps2 = 1e-3;
  config.vertical_inside_bias_min_abs_residual_m = 0.0;
  config.vertical_inside_bias_min_residual_delta_m = 0.0;
  config.vertical_inside_bias_gate_fraction = 1.0;

  VerticalInsideBiasAdapter adapter(config);
  ExpectTrue(
    !adapter.ObserveInsideResidual(1U, 0.0, 0.0, 0.05, 0.1).has_value(),
    "first inside residual should only seed the bias adapter");
  ExpectTrue(
    !adapter.ObserveInsideResidual(2U, 0.5, -0.01, 0.05, 0.1).has_value(),
    "adapter should wait for the configured time window");
  const auto update = adapter.ObserveInsideResidual(3U, 1.0, -0.04, 0.05, 0.1);
  ExpectTrue(update.has_value(), "inside residual trend should produce a bias update");
  ExpectNear(update->delta_baz_mps2, -1e-3, 1e-12, "bias update should be bounded and follow residual trend sign");
  ExpectNear(update->equivalent_acc_mps2, -0.08, 1e-12, "equivalent acceleration should be 2*dr/T^2");
  ExpectTrue(update->anchor_state_index == 1U, "bias update should anchor at the start of the observed trend");
  ExpectTrue(update->current_state_index == 3U, "bias update should report the latest observed state");
}

void TestVerticalInsideBiasAdapterIgnoresOutsideGateResidual() {
  auto config = DefaultConfig();
  config.enable_vertical_inside_bias_adaptation = true;
  config.vertical_inside_bias_window_s = 2.0;
  config.vertical_inside_bias_min_window_s = 1.0;
  config.vertical_inside_bias_min_observations = 2;
  config.vertical_inside_bias_update_interval_s = 0.1;
  config.vertical_inside_bias_gain = 1.0;
  config.vertical_inside_bias_max_delta_mps2 = 1e-3;
  config.vertical_inside_bias_min_abs_residual_m = 0.0;
  config.vertical_inside_bias_min_residual_delta_m = 0.0;
  config.vertical_inside_bias_gate_fraction = 1.0;

  VerticalInsideBiasAdapter adapter(config);
  ExpectTrue(
    !adapter.ObserveInsideResidual(1U, 0.0, 0.0, 0.05, 0.1).has_value(),
    "first inside residual should only seed the bias adapter");
  ExpectTrue(
    !adapter.ObserveInsideResidual(2U, 1.0, -0.12, 0.05, 0.1).has_value(),
    "outside residual should not be used for inside bias adaptation");
}

}  // namespace

int main() {
  try {
    RunTest("TestVerticalFeedbackRequiresGnss", TestVerticalFeedbackRequiresGnss);
    RunTest("TestVerticalFeedbackRequiresVerticalGmBiasProcess", TestVerticalFeedbackRequiresVerticalGmBiasProcess);
    RunTest("TestVerticalFeedbackRejectsSegmentFeedbackCompatibility", TestVerticalFeedbackRejectsSegmentFeedbackCompatibility);
    RunTest("TestInitialStaticVerticalSpecificForceLoads", TestInitialStaticVerticalSpecificForceLoads);
    RunTest(
      "TestInitialStaticVerticalSpecificForceRequiresStaticAlignment",
      TestInitialStaticVerticalSpecificForceRequiresStaticAlignment);
    RunTest("TestReserveVerticalVelocityInterfaceIsAccepted", TestReserveVerticalVelocityInterfaceIsAccepted);
    RunTest("TestVerticalFeedbackMinIntervalLoads", TestVerticalFeedbackMinIntervalLoads);
    RunTest("TestVerticalFeedbackRejectsNegativeMinInterval", TestVerticalFeedbackRejectsNegativeMinInterval);
    RunTest("TestSequentialRecoveryConfigLoads", TestSequentialRecoveryConfigLoads);
    RunTest(
      "TestSequentialRecoveryRejectsNonPositiveIterationCount",
      TestSequentialRecoveryRejectsNonPositiveIterationCount);
    RunTest(
      "TestSequentialRecoveryRejectsInvalidNhcHistoryWindow",
      TestSequentialRecoveryRejectsInvalidNhcHistoryWindow);
    RunTest("TestHorizontalPositionFactorIgnoresVerticalTranslation", TestHorizontalPositionFactorIgnoresVerticalTranslation);
    RunTest(
      "TestStaticVerticalSpecificForceResidualNearZeroWhenBiasMatches",
      TestStaticVerticalSpecificForceResidualNearZeroWhenBiasMatches);
    RunTest(
      "TestStaticVerticalSpecificForceRespondsLinearlyToBaz",
      TestStaticVerticalSpecificForceRespondsLinearlyToBaz);
    RunTest(
      "TestStaticVerticalSpecificForceIgnoresYawButRespondsToRollPitch",
      TestStaticVerticalSpecificForceIgnoresYawButRespondsToRollPitch);
    RunTest(
      "TestStaticVerticalSpecificForceJacobianMatchesNumericalDerivative",
      TestStaticVerticalSpecificForceJacobianMatchesNumericalDerivative);
    RunTest("TestPureYawLeavesRollPitchFeedbackResidualZero", TestPureYawLeavesRollPitchFeedbackResidualZero);
    RunTest("TestGainScaleAmplifiesAttitudeAndBiasTargets", TestGainScaleAmplifiesAttitudeAndBiasTargets);
    RunTest("TestVerticalResidualFeedsDpZRow", TestVerticalResidualFeedsDpZRow);
    RunTest("TestJacobianMatchesNumericalDerivative", TestJacobianMatchesNumericalDerivative);
    RunTest(
      "TestVerticalInsideKinematicFactorTracksPitchRollVzAndBaz",
      TestVerticalInsideKinematicFactorTracksPitchRollVzAndBaz);
    RunTest(
      "TestVerticalInsideKinematicFactorIgnoresYawHorizontalTranslationAndXyBias",
      TestVerticalInsideKinematicFactorIgnoresYawHorizontalTranslationAndXyBias);
    RunTest(
      "TestVerticalInsideKinematicFactorJacobianMatchesNumericalDerivative",
      TestVerticalInsideKinematicFactorJacobianMatchesNumericalDerivative);
    RunTest(
      "TestNhcThresholdBootstrapRespectsMinimumFloors",
      TestNhcThresholdBootstrapRespectsMinimumFloors);
    RunTest(
      "TestNhcJumpAnchorFindsFirstThresholdCrossingFromBodyVz",
      TestNhcJumpAnchorFindsFirstThresholdCrossingFromBodyVz);
    RunTest(
      "TestNhcJumpAnchorCachesConfirmedWindowCrossing",
      TestNhcJumpAnchorCachesConfirmedWindowCrossing);
    RunTest(
      "TestNhcJumpAnchorIgnoresBodyVyOnlyCrossing",
      TestNhcJumpAnchorIgnoresBodyVyOnlyCrossing);
    RunTest(
      "TestSparseVerticalJumpPlannerBuildsSinglePeakCandidate",
      TestSparseVerticalJumpPlannerBuildsSinglePeakCandidate);
    RunTest(
      "TestSparseVerticalJumpPlannerBuildsWindowAroundPeakCandidate",
      TestSparseVerticalJumpPlannerBuildsWindowAroundPeakCandidate);
    RunTest("TestBodyZSeedJumpDetectorFindsDownwardWindow", TestBodyZSeedJumpDetectorFindsDownwardWindow);
    RunTest(
      "TestBodyZSeedJumpDetectorUsesSeedAttitudeGravityProjection",
      TestBodyZSeedJumpDetectorUsesSeedAttitudeGravityProjection);
    RunTest(
      "TestVerticalInsideBiasAdapterUsesInsideResidualTrend",
      TestVerticalInsideBiasAdapterUsesInsideResidualTrend);
    RunTest(
      "TestVerticalInsideBiasAdapterIgnoresOutsideGateResidual",
      TestVerticalInsideBiasAdapterIgnoresOutsideGateResidual);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
