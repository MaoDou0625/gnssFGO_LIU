#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "offline_lc_minimal/core/RtkOutageInitialValueSmoother.h"
#include "offline_lc_minimal/core/RtkOutageBatchSegmentPlanner.h"
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

offline_lc_minimal::TrajectoryRow MakeTrajectoryRow(const double time_s, const double up_m) {
  offline_lc_minimal::TrajectoryRow row;
  row.time_s = time_s;
  row.enu_position_m = Eigen::Vector3d(0.0, 0.0, up_m);
  return row;
}

void TestSegmentedBatchAssemblerSplicesTrajectoryWithoutBoundaryDuplicates() {
  offline_lc_minimal::RtkOutageBatchSegmentRow pre;
  pre.segment_index = 0U;
  pre.segment_role = "PRE_RTK_VALID";
  pre.start_time_s = 0.0;
  pre.end_time_s = 10.0;
  pre.planned = true;
  offline_lc_minimal::RtkOutageBatchSegmentRow outage;
  outage.segment_index = 1U;
  outage.segment_role = "RTK_OUTAGE";
  outage.start_time_s = 10.0;
  outage.end_time_s = 20.0;
  outage.planned = true;
  offline_lc_minimal::RtkOutageBatchSegmentRow post;
  post.segment_index = 2U;
  post.segment_role = "POST_RTK_VALID";
  post.start_time_s = 20.0;
  post.end_time_s = 30.0;
  post.planned = true;

  offline_lc_minimal::OfflineRunResult pre_result;
  pre_result.trajectory = {
    MakeTrajectoryRow(0.0, 0.0),
    MakeTrajectoryRow(10.0, 10.0)};
  offline_lc_minimal::OfflineRunResult outage_result;
  outage_result.trajectory = {
    MakeTrajectoryRow(10.0, 100.0),
    MakeTrajectoryRow(15.0, 15.0),
    MakeTrajectoryRow(20.0, 20.0)};
  offline_lc_minimal::OfflineRunResult post_result;
  post_result.trajectory = {
    MakeTrajectoryRow(20.0, 200.0),
    MakeTrajectoryRow(30.0, 30.0)};

  offline_lc_minimal::SegmentedBatchResultAssemblerRequest request;
  request.pieces = {
    {pre, pre_result},
    {outage, outage_result},
    {post, post_result}};
  request.processing_end_time_s = 30.0;
  request.vertical_boundary_jump_allowed = true;

  const offline_lc_minimal::OfflineRunResult assembled =
    offline_lc_minimal::SegmentedBatchResultAssembler(std::move(request)).Assemble();
  ExpectTrue(assembled.trajectory.size() == 5U, "assembled trajectory should not duplicate boundaries");
  ExpectTrue(std::abs(assembled.trajectory[1].enu_position_m.z() - 10.0) < 1e-12,
             "pre boundary value should be retained at outage start");
  ExpectTrue(std::abs(assembled.trajectory[3].enu_position_m.z() - 20.0) < 1e-12,
             "outage boundary value should be retained at post start");
  ExpectTrue(assembled.run_summary.rtk_outage_segmented_batch_enabled,
             "assembled summary should mark segmented batch enabled");
  ExpectTrue(assembled.run_summary.rtk_outage_batch_segment_count == 3U,
             "assembled summary should count segments");
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
      "TestBatchSegmentPlannerSplitsFirstPlannedOutage",
      TestBatchSegmentPlannerSplitsFirstPlannedOutage);
    RunTest(
      "TestSegmentedBatchAssemblerSplicesTrajectoryWithoutBoundaryDuplicates",
      TestSegmentedBatchAssemblerSplicesTrajectoryWithoutBoundaryDuplicates);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
