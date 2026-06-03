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

offline_lc_minimal::Stage3VerticalReference MakeStage2LowpassReference(
  const double selected_up_m) {
  offline_lc_minimal::Stage3VerticalReference reference;
  for (std::size_t index = 0; index < 2U; ++index) {
    offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = static_cast<double>(index);
    row.stage2_up_m = 6.0;
    row.stage2_lowpass_up_m = index == 0U ? selected_up_m - 0.1 : selected_up_m;
    row.lowpass_delta_m = row.stage2_lowpass_up_m - row.stage2_up_m;
    row.skip_reason = "PLANNED";
    reference.rows.push_back(row);
  }
  return reference;
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

void TestDirectZUsesStage2LowpassVerticalReference() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kDirectZ;
  config.gnss_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  const offline_lc_minimal::Stage3VerticalReference lowpass_reference =
    MakeStage2LowpassReference(4.5);
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
  request.stage2_lowpass_vertical_reference = &lowpass_reference;
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

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "direct_z should add horizontal and vertical factors");
  ExpectTrue(summary.gnss_vertical_reference_source == "stage2_lowpass", "summary should record lowpass source");
  ExpectNear(
    static_cast<double>(summary.gnss_vertical_reference_selected_count),
    1.0,
    0.0,
    "lowpass reference should be selected");
  ExpectNear(
    factor_records.front().raw_rtk_up_m,
    6.0,
    1e-12,
    "diagnostic should retain raw RTK up");
  ExpectNear(
    factor_records.front().vertical_reference_up_m,
    4.5,
    1e-12,
    "diagnostic should record selected lowpass up");
  ExpectNear(
    factor_records.front().vertical_reference_highfreq_residual_m,
    1.5,
    1e-12,
    "diagnostic should record raw-minus-selected high-frequency residual");
  ExpectNear(
    factor_records.front().measurement_enu_m.z(),
    4.5,
    1e-12,
    "GNSS measurement up should be replaced by Stage2 lowpass");

  gtsam::Values values;
  values.insert(
    gtsam::symbol_shorthand::X(1),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.0, 5.0, 4.5)));
  ExpectNear(graph[1]->error(values), 0.0, 1e-12, "vertical factor should use selected lowpass height");
}

void TestEnvelopeUsesStage2LowpassVerticalReferenceAsGateCenter() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.enable_vertical_envelope_center_pull = false;
  config.gnss_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  const offline_lc_minimal::Stage3VerticalReference lowpass_reference =
    MakeStage2LowpassReference(4.5);
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
  request.stage2_lowpass_vertical_reference = &lowpass_reference;
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

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "envelope should add horizontal and envelope factors");
  ExpectNear(static_cast<double>(envelope_diagnostics.size()), 1.0, 0.0, "one envelope diagnostic should be emitted");
  ExpectNear(
    envelope_diagnostics.front().rtk_up_m,
    4.5,
    1e-12,
    "envelope center should use selected Stage2 lowpass up");
  ExpectTrue(
    envelope_diagnostics.front().center_pull_reference_type == "stage2_lowpass",
    "diagnostic should name the selected vertical reference");
  ExpectNear(
    factor_records.front().measurement_enu_m.z(),
    4.5,
    1e-12,
    "recorded GNSS measurement should use selected Stage2 lowpass up");
}

void TestMissingStage2LowpassVerticalReferenceKeepsHorizontalOnly() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kDirectZ;
  config.gnss_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass;
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

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "missing lowpass should keep horizontal GNSS only");
  ExpectNear(
    static_cast<double>(summary.gnss_vertical_reference_selected_count),
    0.0,
    0.0,
    "missing lowpass should not count as selected");
  ExpectNear(
    static_cast<double>(summary.gnss_vertical_reference_skipped_count),
    1.0,
    0.0,
    "missing lowpass should count as skipped");
  ExpectTrue(
    factor_records.front().vertical_reference_skip_reason ==
      "STAGE2_LOWPASS_REFERENCE_UNAVAILABLE",
    "missing lowpass should be diagnosed");
  ExpectTrue(
    !factor_records.front().vertical_direct_position_factor_used,
    "missing lowpass should not report a vertical direct factor");
  ExpectTrue(
    !consistency_records.front().vertical_direct_position_factor_used,
    "missing lowpass should not report a vertical consistency factor");
}

void TestBuilderCanDisableVerticalFactorsWhileKeepingHorizontal() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kDirectZ;
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
  request.disable_vertical_factors = true;
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

  ExpectTrue(graph.size() == 1U, "vertical-disabled GNSS should add only the horizontal factor");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::HorizontalPositionFactor>(graph[0])),
    "remaining GNSS factor should be horizontal");
  ExpectTrue(factor_records.size() == 1U, "used sample should still be recorded");
  ExpectTrue(factor_records.front().factor_used, "horizontal-only sample should be marked used");
  ExpectTrue(
    !factor_records.front().vertical_direct_position_factor_used,
    "vertical-disabled sample should not report a direct vertical factor");
  ExpectTrue(
    !consistency_records.front().vertical_direct_position_factor_used,
    "vertical-disabled consistency record should not report a direct vertical factor");
  ExpectTrue(trajectory[1].gnss_factor_used, "horizontal-only sample should still mark trajectory GNSS support");
}

void TestRejectedNonFixedSampleAddsNoGnssFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.drop_non_rtkfix = true;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  samples[1].gnssfgo_type_code = 2;

  gtsam::NonlinearFactorGraph graph;
  std::vector<offline_lc_minimal::TrajectoryRow> trajectory(2);
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::GnssFactorRecord> factor_records;
  std::vector<offline_lc_minimal::GnssConsistencyRecord> consistency_records;
  std::vector<offline_lc_minimal::VerticalEnvelopeDiagnosticRow> envelope_diagnostics;
  bool find_state_called = false;

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
  request.should_use_sample = [](const auto &sample) {
    return sample.fix_type() == offline_lc_minimal::GnssFixType::kRtkFix;
  };
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.1, 0.2, 0.3); };
  request.find_state_for_time_s = [&find_state_called](double) {
    find_state_called = true;
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

  ExpectNear(static_cast<double>(graph.size()), 0.0, 0.0, "non-RTKFIX samples should add no GNSS factors");
  ExpectNear(static_cast<double>(summary.gnss_factor_count), 0.0, 0.0, "non-RTKFIX samples should not count as used");
  ExpectNear(
    static_cast<double>(summary.gnss_synced_factor_count),
    0.0,
    0.0,
    "non-RTKFIX samples should not be synchronized");
  ExpectNear(
    static_cast<double>(summary.gnss_interpolated_factor_count),
    0.0,
    0.0,
    "non-RTKFIX samples should not be interpolated");
  ExpectNear(static_cast<double>(factor_records.size()), 1.0, 0.0, "dropped sample should still be recorded");
  ExpectNear(
    static_cast<double>(consistency_records.size()),
    1.0,
    0.0,
    "dropped sample should still keep a consistency record");
  ExpectTrue(!find_state_called, "non-RTKFIX samples should be rejected before state synchronization");
  ExpectTrue(
    factor_records.front().sync_status == offline_lc_minimal::StateMeasSyncStatus::kDropped,
    "dropped sample should have dropped sync status");
  ExpectTrue(!factor_records.front().factor_used, "dropped sample should not mark factor_used");
  ExpectTrue(
    consistency_records.front().sync_status == offline_lc_minimal::StateMeasSyncStatus::kDropped,
    "dropped consistency record should have dropped sync status");
  ExpectTrue(!consistency_records.front().factor_used, "dropped consistency record should not mark factor_used");
  ExpectTrue(!trajectory[1].gnss_factor_used, "dropped sample should not mark trajectory GNSS usage");
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

void TestEnvelopeModeUsesGateSigmaCenterPullWhenConfigured() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.enable_vertical_envelope_center_pull = true;
  config.vertical_envelope_center_sigma_m = 99.0;
  config.vertical_envelope_center_sigma_mode = offline_lc_minimal::VerticalEnvelopeCenterSigmaMode::kGateSigma;
  config.vertical_envelope_center_deadband_m = 0.0;
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

  ExpectNear(static_cast<double>(graph.size()), 3.0, 0.0, "gate-sigma center pull should add one extra vertical factor");
  ExpectTrue(envelope_diagnostics.front().center_pull_factor_used, "diagnostic should mark center pull");
  ExpectNear(envelope_diagnostics.front().half_width_m, 0.60, 1e-12, "half-width should use the gate multiple");
  ExpectNear(envelope_diagnostics.front().center_pull_sigma_m, 0.30, 1e-12, "center sigma should derive from gate width");
  ExpectNear(envelope_diagnostics.front().center_pull_deadband_m, 0.0, 1e-12, "center pull deadband should be disabled");

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
  ExpectNear(envelope_diagnostics.front().center_pull_residual_m, 0.25, 1e-12, "zero-deadband residual should keep raw residual inside gate");
}

void TestEnvelopeCenterPullUsesDriftCorrectedReferenceWhenAvailable() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.enable_vertical_envelope_center_pull = true;
  config.vertical_envelope_center_sigma_mode =
    offline_lc_minimal::VerticalEnvelopeCenterSigmaMode::kGateSigma;
  config.vertical_envelope_center_deadband_m = 0.0;
  config.enable_rtk_vertical_drift_reference = true;
  config.rtk_vertical_drift_use_for_center_pull = true;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  std::vector<offline_lc_minimal::RtkVerticalDriftReferenceDiagnosticRow> drift_profile(samples.size());
  drift_profile[1].sample_index = 1;
  drift_profile[1].valid = true;
  drift_profile[1].drift_estimate_m = 0.10;
  drift_profile[1].corrected_center_up_m = 5.90;

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
  request.rtk_vertical_drift_reference_profile = &drift_profile;
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

  ExpectNear(static_cast<double>(graph.size()), 3.0, 0.0, "drift center should not change factor count");
  ExpectNear(static_cast<double>(envelope_diagnostics.size()), 1.0, 0.0, "one envelope diagnostic should be emitted");
  ExpectTrue(
    envelope_diagnostics.front().center_pull_reference_type == "rtk_drift_corrected",
    "diagnostic should record drift-corrected center reference");
  ExpectNear(
    envelope_diagnostics.front().center_pull_reference_up_m,
    5.90,
    1e-12,
    "center-pull reference should use corrected RTK height");
  ExpectNear(
    envelope_diagnostics.front().rtk_drift_estimate_m,
    0.10,
    1e-12,
    "diagnostic should record drift estimate");

  gtsam::Values optimized_values;
  optimized_values.insert(
    gtsam::symbol_shorthand::X(1),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(4.0, 5.0, 6.0)));
  const auto qc_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(10000.0));
  const offline_lc_minimal::gp::GPWNOJInterpolator interpolator(qc_model, 1.0, 0.0);
  offline_lc_minimal::PopulateVerticalEnvelopeDiagnostics(
    optimized_values,
    interpolator,
    envelope_diagnostics);
  ExpectNear(envelope_diagnostics.front().raw_residual_m, 0.0, 1e-12, "raw gate residual should remain raw RTK");
  ExpectNear(
    envelope_diagnostics.front().center_pull_residual_m,
    0.10,
    1e-12,
    "center pull residual should use drift-corrected reference");
}

void TestEnvelopeCenterPullUsesLowpassReferenceWhenEnabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.enable_vertical_envelope_center_pull = true;
  config.vertical_envelope_center_sigma_mode =
    offline_lc_minimal::VerticalEnvelopeCenterSigmaMode::kGateSigma;
  config.vertical_envelope_center_deadband_m = 0.0;
  config.enable_rtk_vertical_drift_reference = true;
  config.rtk_vertical_drift_use_for_center_pull = true;
  config.enable_rtk_vertical_lowpass_reference = true;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  std::vector<offline_lc_minimal::RtkVerticalDriftReferenceDiagnosticRow> drift_profile(samples.size());
  drift_profile[1].sample_index = 1;
  drift_profile[1].valid = true;
  drift_profile[1].drift_estimate_m = 0.10;
  drift_profile[1].corrected_center_up_m = 5.90;
  drift_profile[1].lowpass_center_up_m = 5.80;
  drift_profile[1].lowpass_delta_m = -0.10;
  drift_profile[1].lowpass_applied = true;

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
  request.rtk_vertical_drift_reference_profile = &drift_profile;
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

  ExpectNear(static_cast<double>(envelope_diagnostics.size()), 1.0, 0.0, "one envelope diagnostic should be emitted");
  ExpectTrue(
    envelope_diagnostics.front().center_pull_reference_type == "rtk_drift_lowpass",
    "diagnostic should record lowpass center reference");
  ExpectNear(
    envelope_diagnostics.front().center_pull_reference_up_m,
    5.80,
    1e-12,
    "center-pull reference should use lowpass RTK height");

  gtsam::Values optimized_values;
  optimized_values.insert(
    gtsam::symbol_shorthand::X(1),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(4.0, 5.0, 6.0)));
  const auto qc_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(10000.0));
  const offline_lc_minimal::gp::GPWNOJInterpolator interpolator(qc_model, 1.0, 0.0);
  offline_lc_minimal::PopulateVerticalEnvelopeDiagnostics(
    optimized_values,
    interpolator,
    envelope_diagnostics);
  ExpectNear(envelope_diagnostics.front().raw_residual_m, 0.0, 1e-12, "raw gate residual should remain raw RTK");
  ExpectNear(
    envelope_diagnostics.front().center_pull_residual_m,
    0.20,
    1e-12,
    "center pull residual should use lowpass reference");
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

void TestEnvelopeModeUsesGateSigmaForInterpolatedCenterPull() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.enable_vertical_envelope_center_pull = true;
  config.vertical_envelope_center_sigma_m = 99.0;
  config.vertical_envelope_center_sigma_mode = offline_lc_minimal::VerticalEnvelopeCenterSigmaMode::kGateSigma;
  config.vertical_envelope_center_deadband_m = 0.0;
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

  ExpectNear(static_cast<double>(graph.size()), 3.0, 0.0, "interpolated gate-sigma center pull should add three factors");
  ExpectTrue(envelope_diagnostics.front().center_pull_factor_used, "interpolated diagnostic should mark center pull");
  ExpectNear(envelope_diagnostics.front().half_width_m, 0.60, 1e-12, "interpolated half-width should use the gate multiple");
  ExpectNear(envelope_diagnostics.front().center_pull_sigma_m, 0.30, 1e-12, "interpolated center sigma should derive from gate width");
  ExpectNear(envelope_diagnostics.front().center_pull_deadband_m, 0.0, 1e-12, "interpolated center pull deadband should be disabled");
}

void TestEnvelopeModeAddsOnlyInterpolatedEnvelopeWhenCenterPullDisabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.10;
  config.vertical_envelope_factor_sigma_m = 0.20;
  config.enable_vertical_envelope_center_pull = false;
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

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "interpolated gate-only RTK should add two GNSS factors");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::GPInterpolatedHorizontalPositionFactor>(graph[0])),
    "gate-only RTK should keep the GP horizontal factor first");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeFactor>(graph[1])),
    "gate-only RTK should add one GP vertical envelope factor");
  ExpectTrue(
    !static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeCenterPullFactor>(graph[1])),
    "gate-only RTK should not add an interpolated center-pull factor");
  ExpectNear(static_cast<double>(summary.gnss_interpolated_factor_count), 1.0, 0.0, "sample should be interpolated");
  ExpectNear(static_cast<double>(summary.gnss_factor_count), 1.0, 0.0, "GNSS sample count should not change");
  ExpectNear(static_cast<double>(envelope_diagnostics.size()), 1.0, 0.0, "envelope diagnostics should stay one row per sample");
  ExpectTrue(!envelope_diagnostics.front().center_pull_factor_used, "gate-only diagnostic should not mark center pull");
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
      "TestDirectZUsesStage2LowpassVerticalReference",
      TestDirectZUsesStage2LowpassVerticalReference);
    RunTest(
      "TestEnvelopeUsesStage2LowpassVerticalReferenceAsGateCenter",
      TestEnvelopeUsesStage2LowpassVerticalReferenceAsGateCenter);
    RunTest(
      "TestMissingStage2LowpassVerticalReferenceKeepsHorizontalOnly",
      TestMissingStage2LowpassVerticalReferenceKeepsHorizontalOnly);
    RunTest(
      "TestBuilderCanDisableVerticalFactorsWhileKeepingHorizontal",
      TestBuilderCanDisableVerticalFactorsWhileKeepingHorizontal);
    RunTest(
      "TestRejectedNonFixedSampleAddsNoGnssFactors",
      TestRejectedNonFixedSampleAddsNoGnssFactors);
    RunTest(
      "TestEnvelopeModeAddsHorizontalAndEnvelopeFactors",
      TestEnvelopeModeAddsHorizontalAndEnvelopeFactors);
    RunTest(
      "TestEnvelopeModeAddsCenterPullFactorWhenEnabled",
      TestEnvelopeModeAddsCenterPullFactorWhenEnabled);
    RunTest(
      "TestEnvelopeModeUsesGateSigmaCenterPullWhenConfigured",
      TestEnvelopeModeUsesGateSigmaCenterPullWhenConfigured);
    RunTest(
      "TestEnvelopeCenterPullUsesDriftCorrectedReferenceWhenAvailable",
      TestEnvelopeCenterPullUsesDriftCorrectedReferenceWhenAvailable);
    RunTest(
      "TestEnvelopeCenterPullUsesLowpassReferenceWhenEnabled",
      TestEnvelopeCenterPullUsesLowpassReferenceWhenEnabled);
    RunTest(
      "TestEnvelopeModeAddsInterpolatedCenterPullFactorWhenEnabled",
      TestEnvelopeModeAddsInterpolatedCenterPullFactorWhenEnabled);
    RunTest(
      "TestEnvelopeModeUsesGateSigmaForInterpolatedCenterPull",
      TestEnvelopeModeUsesGateSigmaForInterpolatedCenterPull);
    RunTest(
      "TestEnvelopeModeAddsOnlyInterpolatedEnvelopeWhenCenterPullDisabled",
      TestEnvelopeModeAddsOnlyInterpolatedEnvelopeWhenCenterPullDisabled);
    RunTest(
      "TestEnvelopeResidualStaysHorizontalWithoutConsistencyRecords",
      TestEnvelopeResidualStaysHorizontalWithoutConsistencyRecords);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
