#include <iostream>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/core/GnssFactorBuilder.h"
#include "offline_lc_minimal/core/RunDiagnosticsBuilder.h"
#include "offline_lc_minimal/factor/GPInterpolatedHorizontalPositionFactor.h"
#include "offline_lc_minimal/factor/HorizontalPositionFactor.h"
#include "offline_lc_minimal/factor/VerticalRtkFactors.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

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

void ExpectNear(const double actual, const double expected, const double tolerance, const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
      message + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
  }
}

offline_lc_minimal::GnssSolutionSample MakeGnssSample(const double time_s) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 0.1;
  sample.lon_rad = 0.2;
  sample.h_m = 3.0;
  sample.sigma_lat_m = 0.2;
  sample.sigma_lon_m = 0.1;
  sample.sigma_h_m = 0.3;
  sample.gnssfgo_type_code = 1;
  sample.best_sol_status_code = 1;
  sample.has_enu_position = true;
  sample.enu_position_m = Eigen::Vector3d(4.0, 5.0, 6.0);
  return sample;
}

void TestBuilderAddsRobustHorizontalAndGaussianVerticalFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.gnss_position_noise_model = offline_lc_minimal::GnssNoiseModel::kHuber;
  config.gnss_position_robust_param = 0.5;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  gtsam::NonlinearFactorGraph graph;
  std::vector<offline_lc_minimal::TrajectoryRow> trajectory(2);
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::GnssFactorRecord> factor_records;
  std::vector<offline_lc_minimal::GnssConsistencyRecord> consistency_records;
  std::vector<offline_lc_minimal::VerticalEnvelopeDiagnosticRow> envelope_diagnostics;

  offline_lc_minimal::GnssFactorBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.navigation_start_index = 0;
  request.graph = &graph;
  request.trajectory = &trajectory;
  request.run_summary = &summary;
  request.factor_records = &factor_records;
  request.consistency_records = &consistency_records;
  request.vertical_envelope_diagnostics = &envelope_diagnostics;
  request.collect_consistency_records = true;
  request.dynamic_start_time_s = 1.0;
  request.should_use_sample = [](const auto &) { return true; };
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.1, 0.2, 0.3); };
  request.find_state_for_time_s = [](double) {
    offline_lc_minimal::StateMeasSyncResult result;
    result.status = offline_lc_minimal::StateMeasSyncStatus::kSynchronizedI;
    result.key_index_i = 1;
    result.key_index_j = 1;
    result.timestamp_i_s = 1.0;
    result.timestamp_j_s = 1.0;
    return result;
  };
  request.trajectory_row_index_for_state = [](std::size_t state_index) {
    return static_cast<long long>(state_index);
  };

  offline_lc_minimal::GnssFactorBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "builder should add horizontal and vertical factors");
  ExpectNear(static_cast<double>(summary.gnss_factor_count), 1.0, 0.0, "one GNSS sample should be used");
  ExpectNear(static_cast<double>(summary.gnss_synced_factor_count), 1.0, 0.0, "sample should be synchronized");
  ExpectTrue(trajectory[1].gnss_factor_used, "synchronized trajectory row should be marked");
  ExpectNear(factor_records.front().measurement_enu_m.z(), 6.0, 0.0, "record should retain measured up");
  ExpectTrue(consistency_records.front().vertical_direct_position_factor_used, "vertical factor should be recorded");

  const auto horizontal_factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[0]);
  const auto vertical_factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[1]);
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::HorizontalPositionFactor>(graph[0])),
    "first factor should remain the horizontal factor");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalPositionFactor>(graph[1])),
    "direct_z should add a direct vertical factor");
  ExpectTrue(static_cast<bool>(horizontal_factor), "horizontal factor should expose a noise model");
  ExpectTrue(static_cast<bool>(vertical_factor), "vertical factor should expose a noise model");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(horizontal_factor->noiseModel())),
    "horizontal factor should keep robust GNSS noise");
  ExpectTrue(
    !static_cast<bool>(boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(vertical_factor->noiseModel())),
    "vertical Phase 1 factor should use plain Gaussian noise");
  ExpectTrue(envelope_diagnostics.empty(), "direct_z should not emit envelope diagnostics");
}

void TestEnvelopeModeAddsHorizontalAndEnvelopeFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  gtsam::NonlinearFactorGraph graph;
  std::vector<offline_lc_minimal::TrajectoryRow> trajectory(2);
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::GnssFactorRecord> factor_records;
  std::vector<offline_lc_minimal::GnssConsistencyRecord> consistency_records;
  std::vector<offline_lc_minimal::VerticalEnvelopeDiagnosticRow> envelope_diagnostics;

  offline_lc_minimal::GnssFactorBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.navigation_start_index = 0;
  request.graph = &graph;
  request.trajectory = &trajectory;
  request.run_summary = &summary;
  request.factor_records = &factor_records;
  request.consistency_records = &consistency_records;
  request.vertical_envelope_diagnostics = &envelope_diagnostics;
  request.collect_consistency_records = true;
  request.dynamic_start_time_s = 1.0;
  request.should_use_sample = [](const auto &) { return true; };
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.1, 0.2, 0.3); };
  request.find_state_for_time_s = [](double) {
    offline_lc_minimal::StateMeasSyncResult result;
    result.status = offline_lc_minimal::StateMeasSyncStatus::kSynchronizedI;
    result.key_index_i = 1;
    result.key_index_j = 1;
    result.timestamp_i_s = 1.0;
    result.timestamp_j_s = 1.0;
    return result;
  };
  request.trajectory_row_index_for_state = [](std::size_t state_index) {
    return static_cast<long long>(state_index);
  };

  offline_lc_minimal::GnssFactorBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "envelope should keep one horizontal and one vertical factor");
  ExpectNear(static_cast<double>(summary.gnss_factor_count), 1.0, 0.0, "one GNSS sample should be used");
  ExpectNear(static_cast<double>(summary.gnss_synced_factor_count), 1.0, 0.0, "sample should be synchronized");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::HorizontalPositionFactor>(graph[0])),
    "envelope should keep the horizontal factor first");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalEnvelopeFactor>(graph[1])),
    "envelope should add a vertical envelope factor");
  ExpectTrue(!consistency_records.front().vertical_direct_position_factor_used, "envelope is not a direct z factor");
  ExpectNear(consistency_records.front().vertical_sigma_u_used_m, 0.20, 1e-12, "envelope factor sigma should be recorded");
  ExpectNear(static_cast<double>(envelope_diagnostics.size()), 1.0, 0.0, "envelope diagnostics should be emitted");
  ExpectNear(envelope_diagnostics.front().rtk_up_m, 6.0, 0.0, "envelope diagnostic should retain measured up");
  ExpectNear(envelope_diagnostics.front().sigma_u_m, 0.3, 1e-12, "envelope diagnostic should retain RTK sigma");
  ExpectNear(envelope_diagnostics.front().half_width_m, 0.6, 1e-12, "envelope half-width should use sigma gate");
  ExpectTrue(!envelope_diagnostics.front().center_pull_factor_used, "center pull should default off");
}

void TestEnvelopeModeAddsCenterPullFactorWhenEnabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.enable_vertical_envelope_center_pull = true;
  config.vertical_envelope_center_sigma_m = 0.60;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  gtsam::NonlinearFactorGraph graph;
  std::vector<offline_lc_minimal::TrajectoryRow> trajectory(2);
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::GnssFactorRecord> factor_records;
  std::vector<offline_lc_minimal::GnssConsistencyRecord> consistency_records;
  std::vector<offline_lc_minimal::VerticalEnvelopeDiagnosticRow> envelope_diagnostics;

  offline_lc_minimal::GnssFactorBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.navigation_start_index = 0;
  request.graph = &graph;
  request.trajectory = &trajectory;
  request.run_summary = &summary;
  request.factor_records = &factor_records;
  request.consistency_records = &consistency_records;
  request.vertical_envelope_diagnostics = &envelope_diagnostics;
  request.collect_consistency_records = true;
  request.dynamic_start_time_s = 1.0;
  request.should_use_sample = [](const auto &) { return true; };
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.1, 0.2, 0.3); };
  request.find_state_for_time_s = [](double) {
    offline_lc_minimal::StateMeasSyncResult result;
    result.status = offline_lc_minimal::StateMeasSyncStatus::kSynchronizedI;
    result.key_index_i = 1;
    result.key_index_j = 1;
    result.timestamp_i_s = 1.0;
    result.timestamp_j_s = 1.0;
    return result;
  };
  request.trajectory_row_index_for_state = [](std::size_t state_index) {
    return static_cast<long long>(state_index);
  };

  offline_lc_minimal::GnssFactorBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 3.0, 0.0, "center pull should add one extra vertical factor");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::HorizontalPositionFactor>(graph[0])),
    "center pull should keep the horizontal factor first");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalEnvelopeFactor>(graph[1])),
    "center pull should keep the envelope factor");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalEnvelopeCenterPullFactor>(graph[2])),
    "center pull should add the weak center factor");
  ExpectNear(static_cast<double>(summary.gnss_factor_count), 1.0, 0.0, "GNSS sample count should not change");
  ExpectNear(static_cast<double>(envelope_diagnostics.size()), 1.0, 0.0, "envelope diagnostics should stay one row per sample");
  ExpectTrue(envelope_diagnostics.front().center_pull_factor_used, "diagnostic should mark center pull");
  ExpectNear(envelope_diagnostics.front().center_pull_sigma_m, 0.60, 1e-12, "center pull sigma should be recorded");
  ExpectNear(envelope_diagnostics.front().center_pull_deadband_m, 0.01, 1e-12, "center pull deadband should be recorded");

  gtsam::Values optimized_values;
  optimized_values.insert(
    gtsam::symbol_shorthand::X(1),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(4.0, 5.0, 6.25)));
  const auto qc_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(10000.0));
  const offline_lc_minimal::gp::GPWNOJInterpolator interpolator(qc_model, 1.0, 0.0);
  offline_lc_minimal::PopulateVerticalEnvelopeDiagnostics(
    optimized_values,
    interpolator,
    envelope_diagnostics);
  ExpectNear(envelope_diagnostics.front().raw_residual_m, 0.25, 1e-12, "raw residual should be populated");
  ExpectNear(
    envelope_diagnostics.front().center_pull_residual_m,
    0.24,
    1e-12,
    "center pull diagnostic should subtract the deadband");
}

void TestEnvelopeModeAddsInterpolatedCenterPullFactorWhenEnabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.enable_vertical_envelope_center_pull = true;
  config.vertical_envelope_center_sigma_m = 0.60;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  gtsam::NonlinearFactorGraph graph;
  std::vector<offline_lc_minimal::TrajectoryRow> trajectory(2);
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::GnssFactorRecord> factor_records;
  std::vector<offline_lc_minimal::GnssConsistencyRecord> consistency_records;
  std::vector<offline_lc_minimal::VerticalEnvelopeDiagnosticRow> envelope_diagnostics;

  offline_lc_minimal::GnssFactorBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.navigation_start_index = 0;
  request.graph = &graph;
  request.trajectory = &trajectory;
  request.run_summary = &summary;
  request.factor_records = &factor_records;
  request.consistency_records = &consistency_records;
  request.vertical_envelope_diagnostics = &envelope_diagnostics;
  request.collect_consistency_records = true;
  request.dynamic_start_time_s = 1.0;
  request.should_use_sample = [](const auto &) { return true; };
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.1, 0.2, 0.3); };
  request.find_state_for_time_s = [](double) {
    offline_lc_minimal::StateMeasSyncResult result;
    result.status = offline_lc_minimal::StateMeasSyncStatus::kInterpolated;
    result.key_index_i = 0;
    result.key_index_j = 1;
    result.timestamp_i_s = 0.0;
    result.timestamp_j_s = 1.0;
    result.duration_from_state_i_s = 0.4;
    return result;
  };
  request.trajectory_row_index_for_state = [](std::size_t state_index) {
    return static_cast<long long>(state_index);
  };

  offline_lc_minimal::GnssFactorBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 3.0, 0.0, "interpolated center pull should add three GNSS factors");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::GPInterpolatedHorizontalPositionFactor>(graph[0])),
    "interpolated center pull should keep the GP horizontal factor first");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeFactor>(graph[1])),
    "interpolated center pull should keep the GP envelope factor");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeCenterPullFactor>(graph[2])),
    "interpolated center pull should add the GP center factor");
  ExpectNear(static_cast<double>(summary.gnss_interpolated_factor_count), 1.0, 0.0, "sample should be interpolated");
  ExpectNear(static_cast<double>(summary.gnss_factor_count), 1.0, 0.0, "GNSS sample count should not change");
  ExpectNear(static_cast<double>(envelope_diagnostics.size()), 1.0, 0.0, "interpolated diagnostics should stay one row per sample");
  ExpectTrue(envelope_diagnostics.front().center_pull_factor_used, "interpolated diagnostic should mark center pull");
}

void TestEnvelopeResidualStaysHorizontalWithoutConsistencyRecords() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  gtsam::NonlinearFactorGraph graph;
  std::vector<offline_lc_minimal::TrajectoryRow> trajectory(2);
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::GnssFactorRecord> factor_records;
  std::vector<offline_lc_minimal::VerticalEnvelopeDiagnosticRow> envelope_diagnostics;

  offline_lc_minimal::GnssFactorBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.navigation_start_index = 0;
  request.graph = &graph;
  request.trajectory = &trajectory;
  request.run_summary = &summary;
  request.factor_records = &factor_records;
  request.vertical_envelope_diagnostics = &envelope_diagnostics;
  request.collect_consistency_records = false;
  request.dynamic_start_time_s = 1.0;
  request.should_use_sample = [](const auto &) { return true; };
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.1, 0.2, 0.3); };
  request.find_state_for_time_s = [](double) {
    offline_lc_minimal::StateMeasSyncResult result;
    result.status = offline_lc_minimal::StateMeasSyncStatus::kSynchronizedI;
    result.key_index_i = 1;
    result.key_index_j = 1;
    result.timestamp_i_s = 1.0;
    result.timestamp_j_s = 1.0;
    return result;
  };
  request.trajectory_row_index_for_state = [](std::size_t state_index) {
    return static_cast<long long>(state_index);
  };

  offline_lc_minimal::GnssFactorBuilder(std::move(request)).Build();

  gtsam::Values optimized_values;
  optimized_values.insert(
    gtsam::symbol_shorthand::X(1),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(4.0, 5.0, 100.0)));
  const auto qc_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(10000.0));
  const offline_lc_minimal::gp::GPWNOJInterpolator interpolator(qc_model, 1.0, 0.0);
  std::vector<std::optional<std::size_t>> trajectory_row_index_by_state(2);
  trajectory_row_index_by_state[1] = 1U;

  offline_lc_minimal::PopulateGnssPostfitResiduals(
    optimized_values,
    interpolator,
    trajectory_row_index_by_state,
    factor_records,
    nullptr,
    &trajectory);

  ExpectTrue(!factor_records.front().vertical_direct_position_factor_used, "envelope record should mark non-direct z");
  ExpectNear(
    factor_records.front().residual_m,
    0.0,
    1e-12,
    "envelope GNSS residual should stay horizontal when consistency records are disabled");
  ExpectNear(
    trajectory[1].gnss_residual_m,
    0.0,
    1e-12,
    "trajectory GNSS residual should stay horizontal when consistency records are disabled");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestBuilderAddsRobustHorizontalAndGaussianVerticalFactors",
      TestBuilderAddsRobustHorizontalAndGaussianVerticalFactors);
    RunTest(
      "TestEnvelopeModeAddsHorizontalAndEnvelopeFactors",
      TestEnvelopeModeAddsHorizontalAndEnvelopeFactors);
    RunTest(
      "TestEnvelopeModeAddsCenterPullFactorWhenEnabled",
      TestEnvelopeModeAddsCenterPullFactorWhenEnabled);
    RunTest(
      "TestEnvelopeModeAddsInterpolatedCenterPullFactorWhenEnabled",
      TestEnvelopeModeAddsInterpolatedCenterPullFactorWhenEnabled);
    RunTest(
      "TestEnvelopeResidualStaysHorizontalWithoutConsistencyRecords",
      TestEnvelopeResidualStaysHorizontalWithoutConsistencyRecords);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
