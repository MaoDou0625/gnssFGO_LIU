#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "offline_lc_minimal/core/RtkOutageInitialValueSmoother.h"
#include "offline_lc_minimal/core/RtkOutageBiasContinuityPolicy.h"
#include "offline_lc_minimal/core/RtkOutageBatchSegmentPlanner.h"
#include "offline_lc_minimal/core/RtkOutageRecoveryReferenceBuilder.h"
#include "offline_lc_minimal/core/RtkOutageSegmentedBatchRunner.h"
#include "offline_lc_minimal/core/RtkOutageWindowPlanner.h"
#include "offline_lc_minimal/core/SegmentedBatchResultAssembler.h"

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

offline_lc_minimal::GnssSolutionSample MakeGnssSample(
  const double time_s,
  const int gnssfgo_type_code,
  const int best_sol_status_code = 1,
  const bool has_position = true) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 1.0;
  sample.lon_rad = 1.0;
  sample.h_m = 10.0;
  sample.sigma_lat_m = 0.01;
  sample.sigma_lon_m = 0.01;
  sample.sigma_h_m = 0.01;
  sample.best_sol_status_code = best_sol_status_code;
  sample.gnssfgo_type_code = gnssfgo_type_code;
  sample.enu_position_m = Eigen::Vector3d(time_s, 0.0, 0.0);
  sample.has_enu_position = has_position;
  return sample;
}

bool PassesQualityFilters(
  const offline_lc_minimal::OfflineRunnerConfig &config,
  const offline_lc_minimal::GnssSolutionSample &sample) {
  if (!sample.has_valid_position()) {
    return false;
  }
  if (config.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config.required_best_sol_status_code) {
    return false;
  }
  if (config.drop_no_solution &&
      sample.fix_type() == offline_lc_minimal::GnssFixType::kNoSolution) {
    return false;
  }
  if (config.drop_non_rtkfix &&
      sample.fix_type() != offline_lc_minimal::GnssFixType::kRtkFix) {
    return false;
  }
  if (config.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    return false;
  }
  return true;
}

void TestPlannerUsesOnlyFixedAnchorsAndCountsRejectedInteriorSamples() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.enable_rtk_outage_smoothing = true;
  config.rtk_outage_min_gap_s = 2.0;
  config.drop_non_rtkfix = true;
  config.drop_no_solution = true;
  config.drop_nonfinite_sigma = true;
  config.required_best_sol_status_code = 1;

  const std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0, 1),
    MakeGnssSample(1.0, 1),
    MakeGnssSample(2.0, 2),
    MakeGnssSample(3.0, 3),
    MakeGnssSample(4.0, 4, 0, false),
    MakeGnssSample(10.0, 1),
    MakeGnssSample(11.0, 1)};
  const std::vector<double> state_timestamps{
    0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
    6.0, 7.0, 8.0, 9.0, 10.0, 11.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> body_z_windows{
    [] {
      offline_lc_minimal::BodyZSeedJumpWindowRow row;
      row.start_time_s = 2.5;
      row.end_time_s = 3.5;
      return row;
    }()};

  offline_lc_minimal::RtkOutageWindowPlanRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.state_timestamps = &state_timestamps;
  request.body_z_jump_windows = &body_z_windows;
  request.passes_gnss_quality_filters =
    [&](const offline_lc_minimal::GnssSolutionSample &sample) {
      return PassesQualityFilters(config, sample);
    };
  request.corrected_time_s =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.time_s;
    };

  const std::vector<offline_lc_minimal::RtkOutageWindowRow> windows =
    offline_lc_minimal::RtkOutageWindowPlanner(std::move(request)).Plan();
  ExpectTrue(windows.size() == 1U, "exactly one fixed-solution gap should be planned");
  ExpectTrue(windows.front().pre_sample_index == 1U, "pre-anchor must be the last RTKFIX sample");
  ExpectTrue(windows.front().post_sample_index == 5U, "post-anchor must be the next RTKFIX sample");
  ExpectTrue(windows.front().pre_anchor_state_index == 1U, "pre-anchor state index is wrong");
  ExpectTrue(windows.front().post_anchor_state_index == 10U, "post-anchor state index is wrong");
  ExpectTrue(windows.front().interior_state_count == 8U, "interior state count is wrong");
  ExpectTrue(windows.front().rejected_sample_count == 3U, "non-fixed interior samples should be counted");
  ExpectTrue(windows.front().body_z_jump_overlap_count == 1U, "Body-Z overlap should be recorded");
  ExpectTrue(windows.front().skip_reason == "PLANNED", "window should remain planned");
}

void TestOutageWindowsCanBeAddedToNHCWindowsWithoutDroppingFixedAnchors() {
  offline_lc_minimal::BodyZSeedJumpWindowRow body_z_row;
  body_z_row.start_time_s = 20.0;
  body_z_row.end_time_s = 21.0;

  offline_lc_minimal::RtkOutageWindowRow outage;
  outage.pre_anchor_state_index = 1U;
  outage.post_anchor_state_index = 10U;
  outage.start_time_s = 1.0;
  outage.end_time_s = 10.0;
  outage.duration_s = 9.0;
  outage.skip_reason = "PLANNED";

  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> windows =
    offline_lc_minimal::BuildRtkOutageNHCWindows({body_z_row}, {outage});
  ExpectTrue(windows.size() == 2U, "RTK outage should be appended to NHC windows");
  ExpectTrue(windows.back().direction == "RTK_OUTAGE", "RTK outage NHC window should be labeled");
  ExpectTrue(windows.back().start_state_index == 1, "RTK outage should keep pre-anchor state");
  ExpectTrue(windows.back().end_state_index == 10, "RTK outage should keep post-anchor state");
}

void TestInitialValueSmootherResetsOutageHeightWithoutRemovingFixedAnchors() {
  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0, 1),
    MakeGnssSample(3.0, 1)};
  samples[0].enu_position_m.z() = 1.0;
  samples[1].enu_position_m.z() = 4.0;

  const std::vector<double> state_timestamps{0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> propagation_records{
    {0U, 1U, 0.0, 1.0, 0.10, 0.0},
    {1U, 2U, 1.0, 2.0, 0.20, 0.0},
    {2U, 3U, 2.0, 3.0, -0.10, 0.0}};

  gtsam::Values values;
  for (std::size_t index = 0U; index < state_timestamps.size(); ++index) {
    values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        gtsam::Rot3::Identity(),
        gtsam::Point3(0.0, 0.0, -100.0 - static_cast<double>(index))));
    values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(0.0, 0.0, -10.0));
  }

  std::vector<offline_lc_minimal::RtkOutageWindowRow> windows(1U);
  windows.front().pre_sample_index = 0U;
  windows.front().post_sample_index = 1U;
  windows.front().pre_anchor_state_index = 0U;
  windows.front().post_anchor_state_index = 3U;
  windows.front().start_time_s = 0.0;
  windows.front().end_time_s = 3.0;
  windows.front().skip_reason = "PLANNED";

  offline_lc_minimal::RtkOutageInitialValueSmoothRequest request;
  request.state_timestamps = &state_timestamps;
  request.gnss_samples = &samples;
  request.propagation_records = &propagation_records;
  request.initial_values = &values;
  request.outage_windows = &windows;
  offline_lc_minimal::RtkOutageInitialValueSmoother(std::move(request)).Apply();

  const auto pre_pose = values.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(0));
  const auto mid_pose = values.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(1));
  const auto post_pose = values.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(3));
  const auto after_pose = values.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(4));
  ExpectTrue(std::abs(pre_pose.translation().z() - 1.0) < 1e-12, "pre-anchor height should be reset");
  ExpectTrue(std::abs(mid_pose.translation().z() - 2.0) < 1e-12, "interior height should be interpolated");
  ExpectTrue(std::abs(post_pose.translation().z() - 4.0) < 1e-12, "post-anchor height should be reset");
  ExpectTrue(std::abs(after_pose.translation().z() - 3.0) < 1e-12, "post-outage height should be shifted");
  ExpectTrue(windows.front().initial_value_smoothing_applied, "smoothing diagnostic should be set");
  ExpectTrue(windows.front().initial_value_smoothed_state_count == 4U, "smoothed state count is wrong");
}

offline_lc_minimal::GnssSolutionSample MakeRecoverySample(
  const double time_s,
  const int gnssfgo_type_code,
  const double up_m,
  const int best_sol_status_code = 1,
  const bool has_position = true) {
  offline_lc_minimal::GnssSolutionSample sample =
    MakeGnssSample(time_s, gnssfgo_type_code, best_sol_status_code, has_position);
  sample.enu_position_m = Eigen::Vector3d(0.0, 0.0, up_m);
  sample.has_enu_position = has_position;
  return sample;
}

void TestRecoveryReferenceUsesOnlyValidRtkFixSamples() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_recovery_reference_min_fix_samples = 3;
  config.rtk_outage_recovery_reference_max_duration_s = 2.0;

  std::vector<offline_lc_minimal::RtkOutageWindowRow> outages(1U);
  outages.front().window_index = 3U;
  outages.front().end_time_s = 10.0;
  outages.front().skip_reason = "PLANNED";

  const std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeRecoverySample(10.0, 1, 100.0),
    MakeRecoverySample(10.5, 2, 500.0),
    MakeRecoverySample(10.5, 1, 100.05),
    MakeRecoverySample(11.0, 1, 100.10),
    MakeRecoverySample(11.5, 1, 900.0, 0),
    MakeRecoverySample(13.0, 1, 1000.0)};

  offline_lc_minimal::RtkOutageRecoveryReferenceBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.outage_windows = &outages;
  request.passes_gnss_quality_filters =
    [&](const offline_lc_minimal::GnssSolutionSample &sample) {
      return PassesQualityFilters(config, sample);
    };
  request.corrected_time_s =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.time_s;
    };

  const std::vector<offline_lc_minimal::RtkOutageRecoveryReferenceRow> rows =
    offline_lc_minimal::RtkOutageRecoveryReferenceBuilder(std::move(request)).Build();
  ExpectTrue(rows.size() == 1U, "one recovery reference row should be emitted");
  ExpectTrue(rows.front().valid, "three RTKFIX samples should produce a valid recovery reference");
  ExpectTrue(rows.front().valid_fix_sample_count == 3U, "only RTKFIX samples should be counted");
  ExpectTrue(std::abs(rows.front().reference_up_m - 100.0) < 1e-12,
             "recovery fit should extrapolate up to outage end");
  ExpectTrue(std::abs(rows.front().reference_vz_mps - 0.10) < 1e-12,
             "recovery fit should estimate vertical velocity");
}

void TestRecoveryReferenceSkipsWhenSamplesAreInsufficient() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_recovery_reference_min_fix_samples = 3;
  config.rtk_outage_recovery_reference_max_duration_s = 2.0;

  std::vector<offline_lc_minimal::RtkOutageWindowRow> outages(1U);
  outages.front().window_index = 4U;
  outages.front().end_time_s = 10.0;
  outages.front().skip_reason = "PLANNED";

  const std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeRecoverySample(10.0, 1, 100.0),
    MakeRecoverySample(10.5, 2, 100.1),
    MakeRecoverySample(11.0, 1, 100.2, 0)};

  offline_lc_minimal::RtkOutageRecoveryReferenceBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.outage_windows = &outages;
  request.passes_gnss_quality_filters =
    [&](const offline_lc_minimal::GnssSolutionSample &sample) {
      return PassesQualityFilters(config, sample);
    };
  request.corrected_time_s =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.time_s;
    };

  const std::vector<offline_lc_minimal::RtkOutageRecoveryReferenceRow> rows =
    offline_lc_minimal::RtkOutageRecoveryReferenceBuilder(std::move(request)).Build();
  ExpectTrue(rows.size() == 1U, "one recovery reference row should be emitted");
  ExpectTrue(!rows.front().valid, "insufficient RTKFIX samples should be skipped");
  ExpectTrue(rows.front().skip_reason == "insufficient_rtkfix_samples",
             "skip reason should explain insufficient recovery support");
}

void TestBiasContinuityPolicyBreaksForLargeReestimateWindow() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_baz_continuity_break_delta_threshold_mps2 =
    offline_lc_minimal::MicroGToMps2(1000.0);

  std::vector<offline_lc_minimal::RtkOutageWindowRow> outages(1U);
  outages.front().window_index = 5U;
  outages.front().start_time_s = 10.0;
  outages.front().end_time_s = 20.0;
  outages.front().skip_reason = "PLANNED";

  std::vector<offline_lc_minimal::BodyZBiasReestimateSegmentRow> segments(1U);
  segments.front().source_outage_window_index = 5;
  segments.front().start_time_s = 11.0;
  segments.front().end_time_s = 19.0;
  segments.front().detected_bias_delta_mps2 =
    offline_lc_minimal::MicroGToMps2(1500.0);

  offline_lc_minimal::RtkOutageBiasContinuityPolicyRequest request;
  request.config = &config;
  request.outage_windows = &outages;
  request.bias_reestimate_segments = &segments;
  const std::vector<offline_lc_minimal::RtkOutageBiasContinuityPolicyRow> rows =
    offline_lc_minimal::RtkOutageBiasContinuityPolicy(std::move(request)).Build();
  ExpectTrue(rows.size() == 2U, "policy should emit start and end boundary rows");
  for (const auto &row : rows) {
    ExpectTrue(!row.ba_z_continuity_allowed, "large reestimate should break ba_z continuity");
    ExpectTrue(row.reset_reason == "delta_threshold_exceeded",
               "policy should report the threshold reset reason");
  }
}

void TestBatchSegmentPlannerSplitsFirstPlannedOutage() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_segmented_batch = true;
  config.rtk_outage_segmented_batch_max_outages = 1;
  config.rtk_outage_segmented_batch_allow_vertical_boundary_jump = true;

  std::vector<offline_lc_minimal::RtkOutageWindowRow> outages(1U);
  outages.front().window_index = 7U;
  outages.front().pre_anchor_state_index = 3U;
  outages.front().post_anchor_state_index = 5U;
  outages.front().start_time_s = 10.0;
  outages.front().end_time_s = 20.0;
  outages.front().skip_reason = "PLANNED";
  const std::vector<double> state_timestamps{0.0, 2.0, 8.0, 10.1, 15.0, 20.2, 30.0};

  offline_lc_minimal::RtkOutageBatchSegmentPlanRequest request;
  request.config = &config;
  request.outage_windows = &outages;
  request.state_timestamps = &state_timestamps;
  request.dynamic_start_time_s = 2.0;
  request.final_end_time_s = 30.0;

  const std::vector<offline_lc_minimal::RtkOutageBatchSegmentRow> segments =
    offline_lc_minimal::RtkOutageBatchSegmentPlanner(std::move(request)).Plan();
  ExpectTrue(segments.size() == 3U, "first outage should produce pre/outage/post segments");
  ExpectTrue(segments[0].segment_role == "PRE_RTK_VALID", "first segment role is wrong");
  ExpectTrue(segments[1].segment_role == "RTK_OUTAGE", "second segment role is wrong");
  ExpectTrue(segments[2].segment_role == "POST_RTK_VALID", "third segment role is wrong");
  ExpectTrue(std::abs(segments[0].start_time_s - 2.0) < 1e-12, "pre start is wrong");
  ExpectTrue(std::abs(segments[0].end_time_s - 10.1) < 1e-12, "pre end should use anchor state time");
  ExpectTrue(std::abs(segments[1].start_time_s - 10.1) < 1e-12, "outage start should use anchor state time");
  ExpectTrue(std::abs(segments[1].end_time_s - 20.2) < 1e-12, "outage end should use anchor state time");
  ExpectTrue(std::abs(segments[2].start_time_s - 20.2) < 1e-12, "post start should use anchor state time");
  ExpectTrue(std::abs(segments[2].end_time_s - 30.0) < 1e-12, "post end is wrong");
  ExpectTrue(segments[1].source_outage_window_index == 7, "source outage index should be retained");
  ExpectTrue(segments[1].vertical_boundary_jump_allowed, "boundary jump flag should be retained");
}

offline_lc_minimal::TrajectoryRow MakeTrajectoryRow(
  const double time_s,
  const double up_m,
  const double yaw_rad = 0.0) {
  offline_lc_minimal::TrajectoryRow row;
  row.time_s = time_s;
  row.enu_position_m = Eigen::Vector3d(0.0, 0.0, up_m);
  row.ypr_rad = Eigen::Vector3d(yaw_rad, 0.0, 0.0);
  return row;
}

offline_lc_minimal::ReferenceNodeState MakeReferenceNodeState(
  const double time_s,
  const double up_m,
  const double yaw_rad) {
  offline_lc_minimal::ReferenceNodeState state;
  state.time_s = time_s;
  state.pose = gtsam::Pose3(
    gtsam::Rot3::Ypr(yaw_rad, 0.0, 0.0),
    gtsam::Point3(0.0, 0.0, up_m));
  return state;
}

offline_lc_minimal::GnssFactorRecord MakeGnssFactorRecord(
  const double corrected_time_s,
  const std::string &source,
  const std::string &skip_reason) {
  offline_lc_minimal::GnssFactorRecord row;
  row.corrected_time_s = corrected_time_s;
  row.factor_used = true;
  row.vertical_reference_source = source;
  row.vertical_reference_skip_reason = skip_reason;
  return row;
}

void TestSegmentedBatchRunnerPreservesAppliedStandalonePrefixFinalScales() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage2_velocity_optimization = true;
  config.enable_stage2_lowfreq_vertical_reference_optimization = true;
  config.stage2_lowfreq_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass;
  config.gnss_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass;
  config.enable_stage2_lowfreq_final_dvz_relaxation = true;
  config.stage2_lowfreq_final_dvz_sigma_scale = 10.0;
  config.enable_stage2_lowfreq_final_hold_relaxation = true;
  config.stage2_lowfreq_final_attitude_hold_sigma_scale = 20.0;
  config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale = 50.0;
  config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale = 80.0;
  config.vertical_velocity_delta_sigma_scale = 10.0;
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_segmented_batch = true;
  config.enable_rtk_outage_boundary_constraints = false;
  config.enable_initial_dynamic_static_detection = true;
  config.enable_initial_dynamic_static_lowpass_protection = true;
  config.enable_initial_dynamic_static_vz_constraint = true;

  auto stage_config = config;
  stage_config.stage2_attitude_hold_sigma_rad *=
    stage_config.stage2_lowfreq_final_attitude_hold_sigma_scale;
  stage_config.stage2_horizontal_position_hold_sigma_m *=
    stage_config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale;
  stage_config.stage2_horizontal_velocity_hold_sigma_mps *=
    stage_config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale;

  auto base_config = stage_config;
  base_config.enable_stage2_lowfreq_vertical_reference_optimization = false;
  base_config.enable_stage2_lowfreq_final_dvz_relaxation = true;
  base_config.enable_stage2_lowfreq_final_hold_relaxation = true;
  base_config.vertical_velocity_delta_sigma_scale = 10.0;
  base_config.gnss_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kRawRtk;

  offline_lc_minimal::RtkOutageWindowRow outage;
  outage.window_index = 2U;
  outage.pre_anchor_state_index = 3U;
  outage.post_anchor_state_index = 5U;
  outage.start_time_s = 10.0;
  outage.end_time_s = 20.0;
  outage.skip_reason = "PLANNED";

  struct Call {
    bool stage2_lowfreq_enabled = false;
    bool final_dvz_relaxation_enabled = false;
    bool final_hold_relaxation_enabled = false;
    double dvz_sigma_scale = 0.0;
    double attitude_hold_sigma_rad = 0.0;
    double horizontal_position_hold_sigma_m = 0.0;
    double horizontal_velocity_hold_sigma_mps = 0.0;
    double processing_start_time_s = 0.0;
    double processing_end_time_s = 0.0;
    bool initial_dynamic_static_detection_enabled = false;
    bool initial_dynamic_static_lowpass_protection_enabled = false;
    bool initial_dynamic_static_vz_constraint_enabled = false;
  };
  std::vector<Call> calls;

  offline_lc_minimal::RtkOutageSegmentedBatchRunRequest request;
  request.base_config = base_config;
  request.config = stage_config;
  request.dataset = offline_lc_minimal::DataSet{};
  request.outage_windows = {outage};
  request.state_timestamps = {0.0, 2.0, 8.0, 10.1, 15.0, 20.2, 30.0};
  request.dynamic_start_time_s = 2.0;
  request.processing_end_time_s = 30.0;
  request.run_once = [&](offline_lc_minimal::OfflineRunnerConfig child_config,
                         std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference>,
                         std::shared_ptr<const offline_lc_minimal::Stage1OutageBodyYEnvelopeReference>,
                         offline_lc_minimal::DataSet) {
    offline_lc_minimal::ValidateConfig(child_config);
    calls.push_back(Call{
      child_config.enable_stage2_lowfreq_vertical_reference_optimization,
      child_config.enable_stage2_lowfreq_final_dvz_relaxation,
      child_config.enable_stage2_lowfreq_final_hold_relaxation,
      child_config.vertical_velocity_delta_sigma_scale,
      child_config.stage2_attitude_hold_sigma_rad,
      child_config.stage2_horizontal_position_hold_sigma_m,
      child_config.stage2_horizontal_velocity_hold_sigma_mps,
      child_config.processing_start_time_s,
      child_config.processing_end_time_s,
      child_config.enable_initial_dynamic_static_detection,
      child_config.enable_initial_dynamic_static_lowpass_protection,
      child_config.enable_initial_dynamic_static_vz_constraint});
    offline_lc_minimal::OfflineRunResult result;
    result.trajectory = {
      MakeTrajectoryRow(child_config.processing_start_time_s, 0.0),
      MakeTrajectoryRow(child_config.processing_end_time_s, 1.0)};
    return result;
  };

  (void)offline_lc_minimal::RtkOutageSegmentedBatchRunner(std::move(request)).Run();
  ExpectTrue(calls.size() == 3U, "segmented batch should run pre/outage/post children");
  ExpectTrue(
    !calls[0].stage2_lowfreq_enabled,
    "standalone prefix child should disable Stage2 lowfreq wrapper");
  ExpectTrue(
    !calls[0].final_dvz_relaxation_enabled,
    "standalone prefix child should clear final DVZ relaxation");
  ExpectTrue(
    !calls[0].final_hold_relaxation_enabled,
    "standalone prefix child should clear final hold relaxation");
  ExpectTrue(
    calls[0].initial_dynamic_static_detection_enabled &&
      calls[0].initial_dynamic_static_lowpass_protection_enabled &&
      calls[0].initial_dynamic_static_vz_constraint_enabled,
    "standalone prefix child should preserve initial dynamic static settings");
  ExpectTrue(
    std::abs(calls[0].dvz_sigma_scale - 10.0) < 1e-15,
    "standalone prefix child should preserve already-applied DVZ output sigma scale");
  ExpectTrue(
    std::abs(calls[0].attitude_hold_sigma_rad -
             config.stage2_attitude_hold_sigma_rad * 20.0) < 1e-15,
    "standalone prefix child should preserve already-applied attitude hold sigma");
  ExpectTrue(
    std::abs(calls[0].horizontal_position_hold_sigma_m -
             config.stage2_horizontal_position_hold_sigma_m * 50.0) < 1e-15,
    "standalone prefix child should preserve already-applied horizontal position hold sigma");
  ExpectTrue(
    std::abs(calls[0].horizontal_velocity_hold_sigma_mps -
             config.stage2_horizontal_velocity_hold_sigma_mps * 80.0) < 1e-15,
    "standalone prefix child should preserve already-applied horizontal velocity hold sigma");
  ExpectTrue(
    calls[1].stage2_lowfreq_enabled && calls[1].final_dvz_relaxation_enabled &&
      calls[1].final_hold_relaxation_enabled,
    "outage child should keep Stage2 lowfreq final relaxations");
  ExpectTrue(
    !calls[1].initial_dynamic_static_detection_enabled &&
      !calls[1].initial_dynamic_static_lowpass_protection_enabled &&
      !calls[1].initial_dynamic_static_vz_constraint_enabled,
    "outage child should disable initial dynamic static settings");
  ExpectTrue(
    std::abs(calls[1].dvz_sigma_scale - 10.0) < 1e-15,
    "outage child should keep relaxed DVZ output sigma scale");
  ExpectTrue(
    std::abs(calls[1].attitude_hold_sigma_rad -
             config.stage2_attitude_hold_sigma_rad * 20.0) < 1e-15,
    "outage child should use relaxed attitude hold sigma");
  ExpectTrue(
    std::abs(calls[1].horizontal_position_hold_sigma_m -
             config.stage2_horizontal_position_hold_sigma_m * 50.0) < 1e-15,
    "outage child should use relaxed horizontal position hold sigma");
  ExpectTrue(
    std::abs(calls[1].horizontal_velocity_hold_sigma_mps -
             config.stage2_horizontal_velocity_hold_sigma_mps * 80.0) < 1e-15,
    "outage child should use relaxed horizontal velocity hold sigma");
  ExpectTrue(
    calls[2].stage2_lowfreq_enabled && calls[2].final_dvz_relaxation_enabled &&
      calls[2].final_hold_relaxation_enabled,
    "post child should keep Stage2 lowfreq final relaxations");
  ExpectTrue(
    !calls[2].initial_dynamic_static_detection_enabled &&
      !calls[2].initial_dynamic_static_lowpass_protection_enabled &&
      !calls[2].initial_dynamic_static_vz_constraint_enabled,
    "post child should disable initial dynamic static settings");
  ExpectTrue(
    std::abs(calls[2].dvz_sigma_scale - 10.0) < 1e-15,
    "post child should keep relaxed DVZ output sigma scale");
}

void TestSegmentedBatchRunnerPassesBoundaryAttitudeReferenceWithoutStage2Timeline() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_segmented_batch = true;
  config.enable_rtk_outage_boundary_constraints = true;
  config.rtk_outage_segmented_batch_max_outages = 1;
  config.rtk_outage_segmented_batch_allow_vertical_boundary_jump = true;
  config.rtk_outage_recovery_reference_min_fix_samples = 3;
  config.rtk_outage_recovery_reference_max_duration_s = 2.0;
  config.required_best_sol_status_code = 1;

  offline_lc_minimal::RtkOutageWindowRow outage;
  outage.window_index = 11U;
  outage.pre_anchor_state_index = 3U;
  outage.post_anchor_state_index = 5U;
  outage.start_time_s = 10.0;
  outage.end_time_s = 20.0;
  outage.skip_reason = "PLANNED";

  struct Call {
    double processing_start_time_s = 0.0;
    double processing_end_time_s = 0.0;
    std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> reference;
  };
  std::vector<Call> calls;

  offline_lc_minimal::RtkOutageSegmentedBatchRunRequest request;
  request.base_config = config;
  request.config = config;
  request.dataset.gnss_samples = {
    MakeRecoverySample(20.0, 1, 100.0),
    MakeRecoverySample(20.5, 1, 100.05),
    MakeRecoverySample(21.0, 1, 100.10)};
  request.outage_windows = {outage};
  request.state_timestamps = {0.0, 2.0, 8.0, 10.1, 15.0, 20.2, 30.0};
  request.dynamic_start_time_s = 2.0;
  request.processing_end_time_s = 30.0;
  request.run_once =
    [&](offline_lc_minimal::OfflineRunnerConfig child_config,
        std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> stage2_reference,
        std::shared_ptr<const offline_lc_minimal::Stage1OutageBodyYEnvelopeReference>,
        offline_lc_minimal::DataSet) {
      calls.push_back(Call{
        child_config.processing_start_time_s,
        child_config.processing_end_time_s,
        stage2_reference});

      offline_lc_minimal::OfflineRunResult result;
      if (child_config.processing_end_time_s <= outage.start_time_s + 1.0e-9) {
        result.trajectory = {
          MakeTrajectoryRow(0.0, 0.0, 2.0),
          MakeTrajectoryRow(outage.start_time_s, 10.0, 2.0)};
        result.optimized_reference_states = {
          MakeReferenceNodeState(0.0, 0.0, 0.1),
          MakeReferenceNodeState(outage.start_time_s, 10.0, 0.4)};
      } else if (child_config.processing_start_time_s >= outage.end_time_s - 1.0e-9) {
        result.trajectory = {
          MakeTrajectoryRow(outage.end_time_s, 20.0, 2.5),
          MakeTrajectoryRow(child_config.processing_end_time_s, 30.0, 2.5)};
        result.optimized_reference_states = {
          MakeReferenceNodeState(outage.end_time_s, 20.0, -0.3),
          MakeReferenceNodeState(child_config.processing_end_time_s, 30.0, -0.1)};
      } else {
        result.trajectory = {
          MakeTrajectoryRow(0.0, 0.0),
          MakeTrajectoryRow(child_config.processing_end_time_s, 1.0)};
        result.optimized_reference_states = {
          MakeReferenceNodeState(0.0, 0.0, 0.2),
          MakeReferenceNodeState(outage.end_time_s, 20.0, 0.7)};
      }
      return result;
    };

  (void)offline_lc_minimal::RtkOutageSegmentedBatchRunner(std::move(request)).Run();

  ExpectTrue(calls.size() == 3U, "boundary handoff should run pre, outage, and post children");
  ExpectTrue(calls[0].reference == nullptr, "pre child should not receive a boundary carrier");

  ExpectTrue(calls[1].reference != nullptr, "outage child should receive boundary references");
  const auto &refs = calls[1].reference->boundary_references;
  ExpectTrue(refs.size() == 2U, "outage child should receive start and end boundaries");
  ExpectTrue(refs[0].boundary_role == "OUTAGE_START", "first outage boundary should be start");
  ExpectTrue(refs[1].boundary_role == "OUTAGE_END", "second outage boundary should be end");
  ExpectTrue(refs[0].has_attitude && refs[0].add_attitude_constraint,
              "outage start should carry an attitude constraint");
  ExpectTrue(!refs[1].has_attitude && !refs[1].add_attitude_constraint,
              "outage end recovery boundary should not carry a post-derived attitude");
  ExpectTrue(refs[1].has_up && refs[1].has_vz,
              "outage end should carry the recovery vertical boundary");
  ExpectTrue(std::abs(refs[0].reference_rotation.ypr().x() - 0.4) < 1e-12,
              "outage start attitude must use the pre optimized Rot3 branch");

  ExpectTrue(calls[2].reference != nullptr, "post child should receive recovery boundary reference");
  ExpectTrue(calls[2].reference->boundary_references.size() == 1U,
             "post child should receive one post-start boundary");
  const auto &post_ref = calls[2].reference->boundary_references.front();
  ExpectTrue(post_ref.boundary_role == "POST_START",
             "post child boundary should target post start");
  ExpectTrue(post_ref.has_up && post_ref.has_vz,
             "post child should keep the recovery vertical boundary");
  ExpectTrue(post_ref.has_attitude && post_ref.add_attitude_constraint,
             "post child should receive outage-end attitude");
  ExpectTrue(std::abs(post_ref.reference_rotation.ypr().x() - 0.7) < 1e-12,
             "post start attitude must use the outage optimized Rot3 branch");
}

void TestSegmentedBatchRunnerKeepsAttitudeHandoffWithoutRecoveryReference() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_segmented_batch = true;
  config.enable_rtk_outage_boundary_constraints = true;
  config.rtk_outage_segmented_batch_max_outages = 1;
  config.rtk_outage_segmented_batch_allow_vertical_boundary_jump = true;
  config.rtk_outage_recovery_reference_min_fix_samples = 3;
  config.required_best_sol_status_code = 1;

  offline_lc_minimal::RtkOutageWindowRow outage;
  outage.window_index = 12U;
  outage.pre_anchor_state_index = 3U;
  outage.post_anchor_state_index = 5U;
  outage.start_time_s = 10.0;
  outage.end_time_s = 20.0;
  outage.skip_reason = "PLANNED";

  struct Call {
    double processing_start_time_s = 0.0;
    double processing_end_time_s = 0.0;
    std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> reference;
  };
  std::vector<Call> calls;

  offline_lc_minimal::RtkOutageSegmentedBatchRunRequest request;
  request.base_config = config;
  request.config = config;
  request.dataset.gnss_samples = {
    MakeRecoverySample(20.0, 3, 100.0),
    MakeRecoverySample(20.5, 3, 100.05)};
  request.outage_windows = {outage};
  request.state_timestamps = {0.0, 2.0, 8.0, 10.1, 15.0, 20.2, 30.0};
  request.dynamic_start_time_s = 2.0;
  request.processing_end_time_s = 30.0;
  request.run_once =
    [&](offline_lc_minimal::OfflineRunnerConfig child_config,
        std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> stage2_reference,
        std::shared_ptr<const offline_lc_minimal::Stage1OutageBodyYEnvelopeReference>,
        offline_lc_minimal::DataSet) {
      calls.push_back(Call{
        child_config.processing_start_time_s,
        child_config.processing_end_time_s,
        stage2_reference});

      offline_lc_minimal::OfflineRunResult result;
      if (child_config.processing_end_time_s <= outage.start_time_s + 1.0e-9) {
        result.trajectory = {
          MakeTrajectoryRow(0.0, 0.0, 2.0),
          MakeTrajectoryRow(outage.start_time_s, 10.0, 2.0)};
        result.optimized_reference_states = {
          MakeReferenceNodeState(0.0, 0.0, 0.1),
          MakeReferenceNodeState(outage.start_time_s, 10.0, 0.4)};
      } else if (child_config.processing_start_time_s < outage.end_time_s - 1.0e-9) {
        result.trajectory = {
          MakeTrajectoryRow(0.0, 0.0),
          MakeTrajectoryRow(outage.end_time_s, 1.0)};
        result.optimized_reference_states = {
          MakeReferenceNodeState(0.0, 0.0, 0.2),
          MakeReferenceNodeState(outage.end_time_s, 20.0, 0.7)};
      } else {
        result.trajectory = {
          MakeTrajectoryRow(outage.end_time_s, 20.0, 2.5),
          MakeTrajectoryRow(child_config.processing_end_time_s, 30.0, 2.5)};
      }
      return result;
    };

  (void)offline_lc_minimal::RtkOutageSegmentedBatchRunner(std::move(request)).Run();

  ExpectTrue(calls.size() == 3U,
             "invalid recovery reference should still run causal pre/outage/post handoff");
  ExpectTrue(calls[1].reference != nullptr, "outage child should receive boundary references");
  const auto &outage_refs = calls[1].reference->boundary_references;
  ExpectTrue(outage_refs.size() == 2U, "outage child should still receive start and end markers");
  ExpectTrue(outage_refs[0].has_attitude && outage_refs[0].add_attitude_constraint,
             "outage start should still use pre attitude");
  ExpectTrue(outage_refs[1].boundary_role == "OUTAGE_END",
             "outage child should receive an outage-end marker");
  ExpectTrue(!outage_refs[1].has_up && !outage_refs[1].has_vz,
             "invalid recovery should not add vertical recovery constraints");
  ExpectTrue(!outage_refs[1].has_attitude,
             "outage end marker should not invent an attitude reference");

  ExpectTrue(calls[2].reference != nullptr, "post child should receive a post-start boundary");
  const auto &post_ref = calls[2].reference->boundary_references.front();
  ExpectTrue(post_ref.boundary_role == "POST_START",
             "post child boundary should target post start");
  ExpectTrue(!post_ref.has_up && !post_ref.has_vz,
             "invalid recovery should not add post vertical constraints");
  ExpectTrue(post_ref.has_attitude && post_ref.add_attitude_constraint,
             "post child should still receive outage-end attitude");
  ExpectTrue(std::abs(post_ref.reference_rotation.ypr().x() - 0.7) < 1e-12,
             "post start attitude must use the outage optimized Rot3 branch");
}

void TestSegmentedBatchAssemblerSplicesTrajectoryWithoutBoundaryDuplicates() {
  offline_lc_minimal::RtkOutageBatchSegmentRow pre;
  pre.segment_index = 0U;
  pre.segment_role = "PRE_RTK_VALID";
  pre.source_outage_window_index = 0;
  pre.start_time_s = 0.0;
  pre.end_time_s = 10.0;
  pre.planned = true;
  offline_lc_minimal::RtkOutageBatchSegmentRow outage;
  outage.segment_index = 1U;
  outage.segment_role = "RTK_OUTAGE";
  outage.source_outage_window_index = 0;
  outage.start_time_s = 10.0;
  outage.end_time_s = 20.0;
  outage.planned = true;
  offline_lc_minimal::RtkOutageBatchSegmentRow post;
  post.segment_index = 2U;
  post.segment_role = "POST_RTK_VALID";
  post.source_outage_window_index = 0;
  post.start_time_s = 20.0;
  post.end_time_s = 30.0;
  post.planned = true;

  offline_lc_minimal::OfflineRunResult pre_result;
  pre_result.trajectory = {
    MakeTrajectoryRow(0.0, 0.0),
    MakeTrajectoryRow(10.0, 10.0)};
  pre_result.gnss_factor_records = {
    MakeGnssFactorRecord(5.0, "raw_rtk", "OK")};
  pre_result.run_summary.gnss_vertical_reference_source = "raw_rtk";
  offline_lc_minimal::LateStaticWindowRow pre_initial_dynamic_window;
  pre_initial_dynamic_window.window_index = 0U;
  pre_initial_dynamic_window.start_time_s = 1.0;
  pre_initial_dynamic_window.end_time_s = 3.0;
  pre_initial_dynamic_window.valid = true;
  pre_result.initial_dynamic_static_windows = {pre_initial_dynamic_window};
  pre_result.run_summary.initial_dynamic_static_detection_enabled = true;
  pre_result.run_summary.initial_dynamic_static_window_count = 1U;
  pre_result.run_summary.initial_dynamic_static_vz_factor_count = 3U;
  offline_lc_minimal::OfflineRunResult outage_result;
  outage_result.trajectory = {
    MakeTrajectoryRow(10.0, 100.0),
    MakeTrajectoryRow(15.0, 15.0),
    MakeTrajectoryRow(19.95, 20.0)};
  outage_result.gnss_factor_records = {
    MakeGnssFactorRecord(15.0, "stage2_lowpass", "OK")};
  outage_result.run_summary.gnss_vertical_reference_source = "stage2_lowpass";
  outage_result.run_summary.rtk_outage_attitude_hold_factor_count = 2U;
  outage_result.run_summary.rtk_outage_relative_attitude_factor_count = 1U;
  outage_result.run_summary.rtk_outage_attitude_hold_max_abs_residual_rad = 0.01;
  outage_result.run_summary.rtk_outage_relative_attitude_max_abs_residual_rad = 0.02;
  offline_lc_minimal::OfflineRunResult post_result;
  post_result.trajectory = {
    MakeTrajectoryRow(19.95, 200.0),
    MakeTrajectoryRow(30.0, 30.0)};
  post_result.gnss_factor_records = {
    MakeGnssFactorRecord(25.0, "stage2_lowpass", "STAGE2_LOWPASS_REFERENCE_UNAVAILABLE")};
  post_result.run_summary.gnss_vertical_reference_source = "stage2_lowpass";
  offline_lc_minimal::LateStaticWindowRow post_initial_dynamic_window;
  post_initial_dynamic_window.window_index = 1U;
  post_initial_dynamic_window.start_time_s = 21.0;
  post_initial_dynamic_window.end_time_s = 23.0;
  post_initial_dynamic_window.valid = true;
  post_result.initial_dynamic_static_windows = {post_initial_dynamic_window};
  post_result.run_summary.initial_dynamic_static_detection_enabled = true;
  post_result.run_summary.initial_dynamic_static_window_count = 1U;
  post_result.run_summary.initial_dynamic_static_vz_factor_count = 4U;

  offline_lc_minimal::SegmentedBatchResultAssemblerRequest request;
  request.pieces = {
    {pre, pre_result},
    {outage, outage_result},
    {post, post_result}};
  offline_lc_minimal::RtkOutageWindowRow outage_window;
  outage_window.window_index = 0U;
  outage_window.start_time_s = 10.0;
  outage_window.end_time_s = 19.95;
  request.outage_windows = {outage_window};
  request.processing_end_time_s = 30.0;
  request.vertical_boundary_jump_allowed = true;

  const offline_lc_minimal::OfflineRunResult assembled =
    offline_lc_minimal::SegmentedBatchResultAssembler(std::move(request)).Assemble();
  ExpectTrue(assembled.trajectory.size() == 5U, "assembled trajectory should not duplicate boundaries");
  ExpectTrue(std::abs(assembled.trajectory[1].enu_position_m.z() - 10.0) < 1e-12,
             "pre boundary value should be retained at outage start");
  ExpectTrue(std::abs(assembled.trajectory[3].enu_position_m.z() - 200.0) < 1e-12,
             "post boundary value should be retained at outage recovery");
  ExpectTrue(assembled.run_summary.rtk_outage_segmented_batch_enabled,
             "assembled summary should mark segmented batch enabled");
  ExpectTrue(assembled.run_summary.rtk_outage_batch_segment_count == 3U,
             "assembled summary should count segments");
  ExpectTrue(assembled.run_summary.gnss_vertical_reference_source == "stage2_lowpass",
             "assembled summary should keep final segment vertical reference source");
  ExpectTrue(assembled.run_summary.gnss_vertical_reference_selected_count == 2U,
             "assembled summary should count selected vertical references");
  ExpectTrue(assembled.run_summary.gnss_vertical_reference_skipped_count == 1U,
             "assembled summary should count skipped vertical references");
  ExpectTrue(assembled.run_summary.rtk_outage_attitude_hold_factor_count == 2U,
             "assembled summary should count outage attitude hold factors");
  ExpectTrue(assembled.run_summary.rtk_outage_relative_attitude_factor_count == 1U,
             "assembled summary should count outage relative attitude factors");
  ExpectTrue(
    std::abs(assembled.run_summary.rtk_outage_attitude_hold_max_abs_residual_rad - 0.01) < 1e-12,
    "assembled summary should keep outage attitude hold residual summary");
  ExpectTrue(
    std::abs(assembled.run_summary.rtk_outage_relative_attitude_max_abs_residual_rad - 0.02) < 1e-12,
    "assembled summary should keep outage relative attitude residual summary");
  ExpectTrue(assembled.initial_dynamic_static_windows.size() == 1U,
             "assembler should keep only prefix initial dynamic static windows");
  ExpectTrue(assembled.initial_dynamic_static_windows.front().start_time_s == 1.0,
             "assembler should not keep post-segment initial dynamic static windows");
  ExpectTrue(assembled.run_summary.initial_dynamic_static_window_count == 1U,
             "assembled summary should count only prefix initial dynamic static windows");
  ExpectTrue(assembled.run_summary.initial_dynamic_static_vz_factor_count == 3U,
             "assembled summary should count only prefix initial dynamic static factors");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestPlannerUsesOnlyFixedAnchorsAndCountsRejectedInteriorSamples",
      TestPlannerUsesOnlyFixedAnchorsAndCountsRejectedInteriorSamples);
    RunTest(
      "TestOutageWindowsCanBeAddedToNHCWindowsWithoutDroppingFixedAnchors",
      TestOutageWindowsCanBeAddedToNHCWindowsWithoutDroppingFixedAnchors);
    RunTest(
      "TestInitialValueSmootherResetsOutageHeightWithoutRemovingFixedAnchors",
      TestInitialValueSmootherResetsOutageHeightWithoutRemovingFixedAnchors);
    RunTest(
      "TestRecoveryReferenceUsesOnlyValidRtkFixSamples",
      TestRecoveryReferenceUsesOnlyValidRtkFixSamples);
    RunTest(
      "TestRecoveryReferenceSkipsWhenSamplesAreInsufficient",
      TestRecoveryReferenceSkipsWhenSamplesAreInsufficient);
    RunTest(
      "TestBiasContinuityPolicyBreaksForLargeReestimateWindow",
      TestBiasContinuityPolicyBreaksForLargeReestimateWindow);
    RunTest(
      "TestBatchSegmentPlannerSplitsFirstPlannedOutage",
      TestBatchSegmentPlannerSplitsFirstPlannedOutage);
    RunTest(
      "TestSegmentedBatchRunnerPreservesAppliedStandalonePrefixFinalScales",
      TestSegmentedBatchRunnerPreservesAppliedStandalonePrefixFinalScales);
    RunTest(
      "TestSegmentedBatchRunnerPassesBoundaryAttitudeReferenceWithoutStage2Timeline",
      TestSegmentedBatchRunnerPassesBoundaryAttitudeReferenceWithoutStage2Timeline);
    RunTest(
      "TestSegmentedBatchRunnerKeepsAttitudeHandoffWithoutRecoveryReference",
      TestSegmentedBatchRunnerKeepsAttitudeHandoffWithoutRecoveryReference);
    RunTest(
      "TestSegmentedBatchAssemblerSplicesTrajectoryWithoutBoundaryDuplicates",
      TestSegmentedBatchAssemblerSplicesTrajectoryWithoutBoundaryDuplicates);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
