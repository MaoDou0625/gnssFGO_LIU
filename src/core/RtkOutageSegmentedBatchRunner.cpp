#include "offline_lc_minimal/core/RtkOutageSegmentedBatchRunner.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "offline_lc_minimal/core/GraphTimelineBuilder.h"
#include "offline_lc_minimal/core/RtkOutageBatchSegmentPlanner.h"
#include "offline_lc_minimal/core/SegmentedBatchResultAssembler.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

OfflineRunnerConfig MakeChildConfig(
  OfflineRunnerConfig config,
  const RtkOutageBatchSegmentRow &segment,
  const RtkOutageWindowRow *source_outage) {
  config.enable_rtk_outage_segmented_batch = false;
  config.enable_rtk_outage_causal_drift_reference = false;
  config.enable_rtk_outage_preoutage_vertical_fence = false;
  config.enable_rtk_vertical_lowpass_reference = false;
  config.enable_attitude_reference_constraint = false;
  config.processing_start_time_s = segment.start_time_s;
  config.processing_end_time_s = segment.end_time_s;

  if (segment.segment_role == "PRE_RTK_VALID") {
    config.processing_start_time_s = 0.0;
    if (source_outage != nullptr) {
      config.processing_end_time_s = source_outage->start_time_s;
    }
  } else if (segment.segment_role == "RTK_OUTAGE") {
    if (source_outage != nullptr) {
      config.processing_start_time_s = source_outage->start_time_s;
      config.processing_end_time_s = source_outage->end_time_s;
    }
    config.enable_rtk_outage_smoothing = false;
    config.enable_rtk_vertical_drift_reference = false;
    config.enable_vertical_motion_adaptive_reweighting = false;
  } else if (segment.segment_role == "POST_RTK_VALID") {
    if (source_outage != nullptr) {
      config.processing_start_time_s = source_outage->end_time_s;
    }
    config.enable_rtk_outage_smoothing = false;
  }
  return config;
}

const RtkOutageWindowRow *FindSourceOutage(
  const std::vector<RtkOutageWindowRow> &outage_windows,
  const RtkOutageBatchSegmentRow &segment) {
  if (segment.source_outage_window_index < 0) {
    return nullptr;
  }
  const auto it = std::find_if(
    outage_windows.begin(),
    outage_windows.end(),
    [&](const RtkOutageWindowRow &row) {
      return static_cast<long long>(row.window_index) ==
             segment.source_outage_window_index;
    });
  return it == outage_windows.end() ? nullptr : &(*it);
}

std::shared_ptr<const Stage2VelocityReference> SliceStage2ReferenceForSegment(
  const std::shared_ptr<const Stage2VelocityReference> &reference,
  const OfflineRunnerConfig &child_config,
  const RtkOutageBatchSegmentRow &segment) {
  if (reference == nullptr) {
    return nullptr;
  }
  if (reference->trajectory.empty()) {
    throw std::runtime_error("cannot slice an empty stage2 reference trajectory");
  }
  auto sliced = std::make_shared<Stage2VelocityReference>();
  sliced->source_config = reference->source_config;

  const auto interpolate_row = [&](const double time_s) {
    const auto &trajectory = reference->trajectory;
    const auto upper = std::lower_bound(
      trajectory.begin(),
      trajectory.end(),
      time_s,
      [](const TrajectoryRow &row, const double target_time_s) {
        return row.time_s < target_time_s;
      });
    if (upper == trajectory.begin()) {
      TrajectoryRow row = trajectory.front();
      row.time_s = time_s;
      return row;
    }
    if (upper == trajectory.end()) {
      TrajectoryRow row = trajectory.back();
      row.time_s = time_s;
      return row;
    }
    const auto &right = *upper;
    const auto &left = *std::prev(upper);
    if (std::abs(right.time_s - time_s) <= kTimeEpsilonS) {
      return right;
    }
    if (std::abs(left.time_s - time_s) <= kTimeEpsilonS ||
        right.time_s <= left.time_s + kTimeEpsilonS) {
      return left;
    }
    const double alpha = (time_s - left.time_s) / (right.time_s - left.time_s);
    TrajectoryRow row;
    row.time_s = time_s;
    row.enu_position_m =
      (1.0 - alpha) * left.enu_position_m + alpha * right.enu_position_m;
    row.enu_velocity_mps =
      (1.0 - alpha) * left.enu_velocity_mps + alpha * right.enu_velocity_mps;
    row.ypr_rad = (1.0 - alpha) * left.ypr_rad + alpha * right.ypr_rad;
    row.omega_radps = (1.0 - alpha) * left.omega_radps + alpha * right.omega_radps;
    row.bias_acc = (1.0 - alpha) * left.bias_acc + alpha * right.bias_acc;
    row.bias_gyro = (1.0 - alpha) * left.bias_gyro + alpha * right.bias_gyro;
    row.gnss_factor_used = left.gnss_factor_used || right.gnss_factor_used;
    row.gnss_fix_type = right.gnss_fix_type;
    row.gnss_residual_m =
      (1.0 - alpha) * left.gnss_residual_m + alpha * right.gnss_residual_m;
    return row;
  };

  std::vector<double> target_timestamps;
  if (child_config.enable_initial_static_subgraph &&
      child_config.static_alignment_duration_s > 0.0) {
    const double alignment_start_time_s = reference->trajectory.front().time_s;
    const double alignment_end_time_s =
      alignment_start_time_s + child_config.static_alignment_duration_s;
    target_timestamps = BuildStateTimestamps(
      alignment_start_time_s,
      alignment_end_time_s,
      child_config.initial_static_state_frequency_hz);
  }
  const double dynamic_start_time_s = child_config.processing_start_time_s > 0.0
    ? child_config.processing_start_time_s
    : segment.start_time_s;
  const double dynamic_end_time_s = child_config.processing_end_time_s > 0.0
    ? child_config.processing_end_time_s
    : segment.end_time_s;
  std::vector<double> dynamic_timestamps = BuildStateTimestamps(
    dynamic_start_time_s,
    dynamic_end_time_s,
    child_config.state_frequency_hz);
  if (!target_timestamps.empty() &&
      std::abs(target_timestamps.back() - dynamic_timestamps.front()) <= kTimeEpsilonS) {
    dynamic_timestamps.erase(dynamic_timestamps.begin());
  }
  target_timestamps.insert(
    target_timestamps.end(),
    dynamic_timestamps.begin(),
    dynamic_timestamps.end());

  sliced->trajectory.reserve(target_timestamps.size());
  for (const double time_s : target_timestamps) {
    sliced->trajectory.push_back(interpolate_row(time_s));
  }
  return sliced;
}

}  // namespace

RtkOutageSegmentedBatchRunner::RtkOutageSegmentedBatchRunner(
  RtkOutageSegmentedBatchRunRequest request)
    : request_(std::move(request)) {}

OfflineRunResult RtkOutageSegmentedBatchRunner::Run() const {
  if (!request_.run_once) {
    throw std::runtime_error("RtkOutageSegmentedBatchRunner requires a run_once callback");
  }
  if (!std::isfinite(request_.processing_end_time_s) ||
      request_.processing_end_time_s <= request_.dynamic_start_time_s + kTimeEpsilonS) {
    throw std::runtime_error("RtkOutageSegmentedBatchRunner received an invalid processing window");
  }

  RtkOutageBatchSegmentPlanRequest plan_request;
  plan_request.config = &request_.config;
  plan_request.outage_windows = &request_.outage_windows;
  plan_request.state_timestamps = &request_.state_timestamps;
  plan_request.dynamic_start_time_s = request_.dynamic_start_time_s;
  plan_request.final_end_time_s = request_.processing_end_time_s;
  std::vector<RtkOutageBatchSegmentRow> segments =
    RtkOutageBatchSegmentPlanner(std::move(plan_request)).Plan();
  if (segments.empty()) {
    OfflineRunnerConfig passthrough_config = request_.config;
    passthrough_config.enable_rtk_outage_segmented_batch = false;
    return request_.run_once(
      std::move(passthrough_config),
      request_.stage2_reference,
      request_.dataset);
  }

  std::vector<SegmentedBatchResultPiece> pieces;
  pieces.reserve(segments.size());
  for (const auto &segment : segments) {
    const RtkOutageWindowRow *source_outage =
      FindSourceOutage(request_.outage_windows, segment);
    OfflineRunnerConfig child_config =
      MakeChildConfig(request_.config, segment, source_outage);
    std::shared_ptr<const Stage2VelocityReference> child_stage2_reference =
      SliceStage2ReferenceForSegment(request_.stage2_reference, child_config, segment);
    OfflineRunResult child_result =
      request_.run_once(
        std::move(child_config),
        std::move(child_stage2_reference),
        request_.dataset);
    pieces.push_back(SegmentedBatchResultPiece{segment, std::move(child_result)});
  }

  SegmentedBatchResultAssemblerRequest assemble_request;
  assemble_request.pieces = std::move(pieces);
  assemble_request.outage_windows = request_.outage_windows;
  assemble_request.processing_start_time_s = 0.0;
  assemble_request.processing_end_time_s = request_.processing_end_time_s;
  assemble_request.vertical_boundary_jump_allowed =
    request_.config.rtk_outage_segmented_batch_allow_vertical_boundary_jump;
  return SegmentedBatchResultAssembler(std::move(assemble_request)).Assemble();
}

}  // namespace offline_lc_minimal
