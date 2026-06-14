#include "offline_lc_minimal/core/RtkOutageSegmentedBatchRunner.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <stdexcept>
#include <utility>

#include "offline_lc_minimal/core/GraphTimelineBuilder.h"
#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/RtkOutageBoundaryBiasHandoff.h"
#include "offline_lc_minimal/core/RtkOutageBoundaryVerticalHandoff.h"
#include "offline_lc_minimal/core/RtkOutageBiasContinuityPolicy.h"
#include "offline_lc_minimal/core/RtkOutageBatchSegmentPlanner.h"
#include "offline_lc_minimal/core/RtkOutageRecoveryReferenceBuilder.h"
#include "offline_lc_minimal/core/RtkOutageSegmentationPolicy.h"
#include "offline_lc_minimal/core/RtkOutageTerminalVelocityReference.h"
#include "offline_lc_minimal/core/SegmentedBatchResultAssembler.h"
#include "offline_lc_minimal/core/StageAttitudeReference.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IntervalsOverlap(
  const double lhs_start_s,
  const double lhs_end_s,
  const double rhs_start_s,
  const double rhs_end_s) {
  if (!std::isfinite(lhs_start_s) || !std::isfinite(lhs_end_s) ||
      !std::isfinite(rhs_start_s) || !std::isfinite(rhs_end_s)) {
    return false;
  }
  return lhs_start_s <= rhs_end_s + kTimeEpsilonS &&
         rhs_start_s <= lhs_end_s + kTimeEpsilonS;
}

std::vector<BodyZBiasReestimateSegmentRow> ClipBodyZBiasSegmentsToTimeline(
  const std::vector<BodyZBiasReestimateSegmentRow> &segments,
  const double start_time_s,
  const double end_time_s) {
  std::vector<BodyZBiasReestimateSegmentRow> clipped;
  for (const auto &segment : segments) {
    if (IntervalsOverlap(
          segment.start_time_s,
          segment.end_time_s,
          start_time_s,
          end_time_s)) {
      clipped.push_back(segment);
    }
  }
  return clipped;
}

bool IsStandalonePrefixSegment(const RtkOutageBatchSegmentRow &segment) {
  return segment.segment_role == "PRE_RTK_VALID";
}

OfflineRunnerConfig MakeStandalonePrefixChildConfig(
  OfflineRunnerConfig config,
  const RtkOutageWindowRow *source_outage) {
  config = DisableRtkOutageSegmentedBatchRecursion(std::move(config));
  config.enable_stage2_lowfreq_vertical_reference_optimization = false;
  config.enable_stage2_lowfreq_final_dvz_relaxation = false;
  config.enable_stage2_lowfreq_final_hold_relaxation = false;
  config = DisableStage3VerticalReferenceOptimization(std::move(config));
  if (config.gnss_vertical_reference_source !=
      GnssVerticalReferenceSource::kRtkDriftLowpass) {
    config.enable_rtk_vertical_lowpass_reference = false;
  }
  config.processing_start_time_s = 0.0;
  if (source_outage != nullptr) {
    config.processing_end_time_s = source_outage->start_time_s;
  }
  return config;
}

OfflineRunnerConfig MakeChildConfig(
  OfflineRunnerConfig stage_config,
  const OfflineRunnerConfig &base_config,
  const RtkOutageBatchSegmentRow &segment,
  const RtkOutageWindowRow *source_outage) {
  if (IsStandalonePrefixSegment(segment)) {
    return MakeStandalonePrefixChildConfig(base_config, source_outage);
  }

  OfflineRunnerConfig config = stage_config;
  config = DisableRtkOutageSegmentedBatchRecursion(std::move(config));
  config.enable_rtk_outage_causal_drift_reference = false;
  config.enable_rtk_outage_preoutage_vertical_fence = false;
  config.enable_stage1_yaw_refinement = false;
  config.enable_initial_dynamic_static_detection = false;
  config.enable_initial_dynamic_static_lowpass_protection = false;
  config.enable_initial_dynamic_static_vz_constraint = false;
  if (config.gnss_vertical_reference_source !=
      GnssVerticalReferenceSource::kRtkDriftLowpass) {
    config.enable_rtk_vertical_lowpass_reference = false;
  }
  config.processing_start_time_s = segment.start_time_s;
  config.processing_end_time_s = segment.end_time_s;

  if (segment.segment_role == "RTK_OUTAGE") {
    config.processing_start_time_s = 0.0;
    config.processing_end_time_s = segment.end_time_s;
    config.enable_late_static_detection = false;
  } else if (segment.segment_role == "POST_RTK_VALID") {
    if (source_outage != nullptr && std::isfinite(source_outage->end_time_s)) {
      config.processing_start_time_s = source_outage->end_time_s;
    } else {
      config.processing_start_time_s = segment.start_time_s;
    }
    config.enable_rtk_outage_smoothing = false;
    config.enable_initial_static_subgraph = false;
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

const RtkOutageBatchSegmentRow *FindSegmentByRole(
  const std::vector<RtkOutageBatchSegmentRow> &segments,
  const std::string &role) {
  const auto it = std::find_if(
    segments.begin(),
    segments.end(),
    [&](const RtkOutageBatchSegmentRow &segment) {
      return segment.segment_role == role;
    });
  return it == segments.end() ? nullptr : &(*it);
}

bool PassesRecoveryRtkFixQuality(
  const OfflineRunnerConfig &config,
  const GnssSolutionSample &sample) {
  if (!sample.has_valid_position() || !sample.has_enu_position ||
      !sample.enu_position_m.allFinite()) {
    return false;
  }
  if (sample.fix_type() != GnssFixType::kRtkFix) {
    return false;
  }
  if (config.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config.required_best_sol_status_code) {
    return false;
  }
  if (config.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    return false;
  }
  return true;
}

double CorrectedGnssTime(
  const OfflineRunnerConfig &config,
  const GnssSolutionSample &sample) {
  return sample.time_s - config.gnss_time_offset_s;
}

std::shared_ptr<const Stage2VelocityReference> SliceStage2ReferenceForSegment(
  const std::shared_ptr<const Stage2VelocityReference> &reference,
  const OfflineRunnerConfig &child_config,
  const RtkOutageBatchSegmentRow &segment,
  const double dynamic_start_fallback_s) {
  if (reference == nullptr) {
    return nullptr;
  }
  const std::vector<ReferenceNodeState> source_reference_states =
    SortedFiniteReferenceStates(BuildStage2ReferenceStates(*reference));
  if (source_reference_states.empty()) {
    throw std::runtime_error("cannot slice an empty stage2 reference state sequence");
  }
  auto sliced = std::make_shared<Stage2VelocityReference>();
  sliced->source_config = reference->source_config;

  const auto interpolate_state = [&](const double time_s) {
    return InterpolateStageReferenceState(source_reference_states, time_s);
  };

  std::vector<double> target_timestamps;
  if (child_config.enable_initial_static_subgraph &&
      child_config.static_alignment_duration_s > 0.0) {
    const double alignment_start_time_s = source_reference_states.front().time_s;
    const double alignment_end_time_s =
      alignment_start_time_s + child_config.static_alignment_duration_s;
    target_timestamps = BuildStateTimestamps(
      alignment_start_time_s,
      alignment_end_time_s,
      child_config.initial_static_state_frequency_hz);
  }
  const double dynamic_start_time_s = child_config.processing_start_time_s > 0.0
    ? child_config.processing_start_time_s
    : dynamic_start_fallback_s;
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
  if (!target_timestamps.empty()) {
    sliced->body_z_bias_reestimate_segments =
      ClipBodyZBiasSegmentsToTimeline(
        reference->body_z_bias_reestimate_segments,
        target_timestamps.front(),
        target_timestamps.back());
  }

  sliced->trajectory.reserve(target_timestamps.size());
  sliced->reference_states.reserve(target_timestamps.size());
  for (const double time_s : target_timestamps) {
    const ReferenceNodeState state = interpolate_state(time_s);
    sliced->reference_states.push_back(state);
    sliced->trajectory.push_back(TrajectoryRowFromReferenceState(state));
  }
  return sliced;
}

std::shared_ptr<const Stage2VelocityReference> WithBoundaryReferences(
  std::shared_ptr<const Stage2VelocityReference> reference,
  std::vector<RtkOutageBoundaryReferenceRow> boundary_references) {
  auto mutable_reference = reference != nullptr
    ? std::make_shared<Stage2VelocityReference>(*reference)
    : std::make_shared<Stage2VelocityReference>();
  mutable_reference->boundary_references = std::move(boundary_references);
  return mutable_reference;
}

const TrajectoryRow *NearestTrajectoryRow(
  const std::vector<TrajectoryRow> &trajectory,
  const double time_s) {
  if (trajectory.empty() || !std::isfinite(time_s)) {
    return nullptr;
  }
  const auto upper = std::lower_bound(
    trajectory.begin(),
    trajectory.end(),
    time_s,
    [](const TrajectoryRow &row, const double target_time_s) {
      return row.time_s < target_time_s;
    });
  if (upper == trajectory.begin()) {
    return &trajectory.front();
  }
  if (upper == trajectory.end()) {
    return &trajectory.back();
  }
  const auto &right = *upper;
  const auto &left = *std::prev(upper);
  return std::abs(left.time_s - time_s) <= std::abs(right.time_s - time_s)
    ? &left
    : &right;
}

const ReferenceNodeState *NearestReferenceState(
  const std::vector<ReferenceNodeState> &states,
  const double time_s) {
  if (states.empty() || !std::isfinite(time_s)) {
    return nullptr;
  }
  const auto upper = std::lower_bound(
    states.begin(),
    states.end(),
    time_s,
    [](const ReferenceNodeState &state, const double target_time_s) {
      return state.time_s < target_time_s;
    });
  if (upper == states.begin()) {
    return &states.front();
  }
  if (upper == states.end()) {
    return &states.back();
  }
  const auto &right = *upper;
  const auto &left = *std::prev(upper);
  return std::abs(left.time_s - time_s) <= std::abs(right.time_s - time_s)
    ? &left
    : &right;
}

RtkOutageBoundaryReferenceRow MakeBoundaryReferenceFromTrajectory(
  const OfflineRunnerConfig &config,
  const OfflineRunResult &result,
  const std::size_t window_index,
  const double target_time_s,
  const std::string &boundary_role,
  const std::string &source_type,
  const bool allow_ba_z) {
  RtkOutageBoundaryReferenceRow reference;
  reference.window_index = window_index;
  reference.boundary_role = boundary_role;
  reference.source_type = source_type;
  reference.target_time_s = target_time_s;
  reference.up_sigma_m = config.rtk_outage_boundary_up_sigma_m;
  reference.vz_sigma_mps = config.rtk_outage_boundary_vz_sigma_mps;
  reference.ba_z_sigma_mps2 = config.rtk_outage_boundary_baz_sigma_mps2;
  reference.horizontal_position_sigma_m = config.stage2_horizontal_position_hold_sigma_m;
  reference.horizontal_velocity_sigma_mps = config.stage2_horizontal_velocity_hold_sigma_mps;
  reference.attitude_sigma_rad = config.rtk_outage_absolute_attitude_sigma_rad;

  const TrajectoryRow *row = NearestTrajectoryRow(result.trajectory, target_time_s);
  const ReferenceNodeState *state =
    NearestReferenceState(result.optimized_reference_states, target_time_s);
  if (row == nullptr && state == nullptr) {
    reference.skip_reason = "missing_boundary_reference";
    return reference;
  }
  if (row != nullptr) {
    reference.reference_up_m = row->enu_position_m.z();
    reference.reference_vz_mps = row->enu_velocity_mps.z();
    reference.reference_ba_z_mps2 = row->bias_acc.z();
    reference.reference_horizontal_position_m =
      Eigen::Vector2d(row->enu_position_m.x(), row->enu_position_m.y());
    reference.reference_horizontal_velocity_mps =
      Eigen::Vector2d(row->enu_velocity_mps.x(), row->enu_velocity_mps.y());
  }
  if (state != nullptr && state->pose.rotation().matrix().allFinite()) {
    reference.reference_rotation = state->pose.rotation();
    reference.has_attitude = true;
  }
  reference.has_up = std::isfinite(reference.reference_up_m);
  reference.has_vz = std::isfinite(reference.reference_vz_mps);
  reference.has_ba_z = allow_ba_z && std::isfinite(reference.reference_ba_z_mps2);
  reference.has_horizontal_position = reference.reference_horizontal_position_m.allFinite();
  reference.has_horizontal_velocity = reference.reference_horizontal_velocity_mps.allFinite();
  reference.add_up_constraint = reference.has_up;
  reference.add_vz_constraint = reference.has_vz;
  reference.add_ba_z_constraint = reference.has_ba_z;
  if (reference.has_ba_z) {
    reference.ba_z_sigma_mps2 =
      ResolveRtkOutageBoundaryBazHandoffSigmaMps2(
        config,
        reference.ba_z_sigma_mps2);
  }
  reference.add_horizontal_position_constraint = reference.has_horizontal_position;
  reference.add_horizontal_velocity_constraint = reference.has_horizontal_velocity;
  reference.add_attitude_constraint = reference.has_attitude;
  reference.valid =
    reference.has_up || reference.has_vz || reference.has_ba_z ||
    reference.has_horizontal_position || reference.has_horizontal_velocity ||
    reference.has_attitude;
  reference.skip_reason = reference.valid ? "OK" : "nonfinite_trajectory_reference";
  return reference;
}

void RefreshBoundaryReferenceValidity(RtkOutageBoundaryReferenceRow &reference) {
  reference.valid =
    reference.has_up || reference.has_vz || reference.has_ba_z ||
    reference.has_horizontal_position || reference.has_horizontal_velocity ||
    reference.has_horizontal_position_velocity_handoff ||
    reference.has_vertical_position_velocity_handoff || reference.has_attitude;
  if (!reference.valid) {
    reference.skip_reason = "nonfinite_trajectory_reference";
  } else if (reference.skip_reason == "nonfinite_trajectory_reference" ||
             reference.skip_reason == "UNSET") {
    reference.skip_reason = "OK";
  }
}

void RemoveAttitudeReference(RtkOutageBoundaryReferenceRow &reference) {
  reference.has_attitude = false;
  reference.add_attitude_constraint = false;
  reference.attitude_sigma_rad = std::numeric_limits<double>::quiet_NaN();
  RefreshBoundaryReferenceValidity(reference);
}

RtkOutageBoundaryReferenceRow MakeOutageEndPositionVelocityReferenceFromPostResult(
  const OfflineRunnerConfig &config,
  const OfflineRunResult &post_result,
  const RtkOutageWindowRow &outage,
  const bool allow_ba_z) {
  RtkOutageBoundaryReferenceRow reference = MakeBoundaryReferenceFromTrajectory(
    config,
    post_result,
    outage.window_index,
    outage.end_time_s,
    "OUTAGE_END",
    "POST_RECOVERY_OPTIMIZED_POSITION_VELOCITY",
    allow_ba_z);
  RemoveAttitudeReference(reference);
  return reference;
}

std::vector<RtkOutageBiasContinuityPolicyRow> BuildBiasContinuityPolicyRows(
  const OfflineRunnerConfig &config,
  const std::vector<RtkOutageWindowRow> &outage_windows,
  const std::vector<BodyZBiasReestimateSegmentRow> &bias_reestimate_segments) {
  RtkOutageBiasContinuityPolicyRequest request;
  request.config = &config;
  request.outage_windows = &outage_windows;
  request.bias_reestimate_segments = &bias_reestimate_segments;
  return RtkOutageBiasContinuityPolicy(std::move(request)).Build();
}

std::vector<RtkOutageRecoveryReferenceRow> BuildRecoveryReferences(
  const OfflineRunnerConfig &config,
  const DataSet &dataset,
  const std::vector<GnssFactorRecord> &gnss_factor_records,
  const std::vector<RtkOutageWindowRow> &outage_windows) {
  RtkOutageRecoveryReferenceBuildRequest request;
  request.config = &config;
  request.gnss_samples = &dataset.gnss_samples;
  request.gnss_factor_records = &gnss_factor_records;
  request.outage_windows = &outage_windows;
  request.passes_gnss_quality_filters =
    [&](const GnssSolutionSample &sample) {
      return PassesRecoveryRtkFixQuality(config, sample);
    };
  request.corrected_time_s =
    [&](const GnssSolutionSample &sample) {
      return CorrectedGnssTime(config, sample);
    };
  return RtkOutageRecoveryReferenceBuilder(std::move(request)).Build();
}

const RtkOutageRecoveryReferenceRow *FindRecoveryReference(
  const std::vector<RtkOutageRecoveryReferenceRow> &rows,
  const std::size_t window_index) {
  const auto it = std::find_if(
    rows.begin(),
    rows.end(),
    [&](const RtkOutageRecoveryReferenceRow &row) {
      return row.window_index == window_index;
    });
  return it == rows.end() ? nullptr : &(*it);
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

  std::vector<RtkOutageRecoveryReferenceRow> recovery_references;
  if (request_.config.enable_rtk_outage_smoothing) {
    recovery_references = BuildRecoveryReferences(
      request_.config,
      request_.dataset,
      request_.gnss_factor_records,
      request_.outage_windows);
  }

  RtkOutageBatchSegmentPlanRequest plan_request;
  plan_request.config = &request_.config;
  plan_request.outage_windows = &request_.outage_windows;
  plan_request.gnss_factor_records = &request_.gnss_factor_records;
  plan_request.recovery_references = &recovery_references;
  plan_request.state_timestamps = &request_.state_timestamps;
  plan_request.dynamic_start_time_s = request_.dynamic_start_time_s;
  plan_request.final_end_time_s = request_.processing_end_time_s;
  std::vector<RtkOutageBatchSegmentRow> segments =
    RtkOutageBatchSegmentPlanner(std::move(plan_request)).Plan();
  if (segments.empty()) {
    OfflineRunnerConfig passthrough_config = request_.config;
    passthrough_config =
      DisableRtkOutageSegmentedBatchRecursion(std::move(passthrough_config));
    return request_.run_once(
      std::move(passthrough_config),
      request_.stage2_reference,
      nullptr,
      request_.road_noise_state_reference,
      request_.dataset);
  }

  const auto assemble_pieces =
    [&](std::vector<SegmentedBatchResultPiece> pieces,
        const std::size_t additional_child_run_count = 0U) {
      SegmentedBatchResultAssemblerRequest assemble_request;
      assemble_request.pieces = std::move(pieces);
      assemble_request.outage_windows = request_.outage_windows;
      assemble_request.processing_start_time_s = 0.0;
      assemble_request.processing_end_time_s = request_.processing_end_time_s;
      assemble_request.vertical_boundary_jump_allowed =
        request_.config.rtk_outage_segmented_batch_allow_vertical_boundary_jump;
      assemble_request.additional_child_run_count = additional_child_run_count;
      OfflineRunResult assembled =
        SegmentedBatchResultAssembler(std::move(assemble_request)).Assemble();
      assembled.rtk_outage_recovery_references = recovery_references;
      if (request_.road_noise_state_reference != nullptr &&
          !request_.road_noise_state_reference->empty()) {
        assembled.road_noise_state_segments =
          request_.road_noise_state_reference->Clip(
            request_.dynamic_start_time_s,
            request_.processing_end_time_s);
        assembled.run_summary.road_noise_state_segment_count =
          assembled.road_noise_state_segments.size();
        assembled.run_summary.road_noise_state_high_segment_count =
          static_cast<std::size_t>(std::count_if(
            assembled.road_noise_state_segments.begin(),
            assembled.road_noise_state_segments.end(),
            [](const RoadNoiseStateSegmentRow &segment) {
              return segment.state == "HIGH_NOISE";
            }));
        assembled.run_summary.road_noise_state_baz_reestimate_enabled =
          assembled.run_summary.road_noise_state_high_segment_count > 0U;
      }
      return assembled;
    };

  const auto run_segment =
    [&](const RtkOutageBatchSegmentRow &segment,
        std::shared_ptr<const Stage2VelocityReference> child_stage2_reference) {
      const RtkOutageWindowRow *source_outage =
        FindSourceOutage(request_.outage_windows, segment);
      OfflineRunnerConfig child_config =
        MakeChildConfig(request_.config, request_.base_config, segment, source_outage);
      return request_.run_once(
        std::move(child_config),
        std::move(child_stage2_reference),
        nullptr,
        request_.road_noise_state_reference,
        request_.dataset);
    };

  const RtkOutageBatchSegmentRow *pre_segment =
    FindSegmentByRole(segments, "PRE_RTK_VALID");
  const RtkOutageBatchSegmentRow *outage_segment =
    FindSegmentByRole(segments, "RTK_OUTAGE");
  const RtkOutageBatchSegmentRow *post_segment =
    FindSegmentByRole(segments, "POST_RTK_VALID");
  const RtkOutageWindowRow *source_outage =
    outage_segment != nullptr
      ? FindSourceOutage(request_.outage_windows, *outage_segment)
      : nullptr;
  const bool can_run_boundary_handoff =
    request_.config.enable_rtk_outage_boundary_constraints &&
    pre_segment != nullptr && outage_segment != nullptr &&
    post_segment != nullptr && source_outage != nullptr;

  if (can_run_boundary_handoff) {
    const RtkOutageWindowRow &outage = *source_outage;
    RtkOutageWindowRow handoff_outage = outage;
    handoff_outage.end_time_s = post_segment->start_time_s;
    const RtkOutageRecoveryReferenceRow *recovery_reference =
      FindRecoveryReference(recovery_references, outage.window_index);

    OfflineRunResult pre_result = run_segment(*pre_segment, nullptr);

    std::vector<RtkOutageBiasContinuityPolicyRow> bias_policy =
      BuildBiasContinuityPolicyRows(
        request_.config,
        request_.outage_windows,
        request_.bias_reestimate_segments);
    const bool allow_start_ba_z = AllowsRtkOutageBazContinuity(
      bias_policy,
      outage.window_index,
      "OUTAGE_START");
    const RtkOutageWindowRow *post_source_outage =
      FindSourceOutage(request_.outage_windows, *post_segment);
    OfflineRunnerConfig post_config =
      MakeChildConfig(request_.config, request_.base_config, *post_segment, post_source_outage);
    std::shared_ptr<const Stage2VelocityReference> post_reference =
      SliceStage2ReferenceForSegment(
        request_.stage2_reference,
        post_config,
        *post_segment,
        request_.dynamic_start_time_s);
    std::vector<RtkOutageBoundaryReferenceRow> post_boundary_refs;
    if (recovery_reference != nullptr) {
      RtkOutageBoundaryReferenceRow post_start_reference =
        MakePostStartBoundaryReferenceFromRecovery(
          request_.config,
          *recovery_reference,
          post_segment->start_time_s);
      if (post_start_reference.valid) {
        post_boundary_refs.push_back(std::move(post_start_reference));
      }
    }
    OfflineRunResult post_result = request_.run_once(
      std::move(post_config),
      WithBoundaryReferences(
        std::move(post_reference),
        std::move(post_boundary_refs)),
      nullptr,
      request_.road_noise_state_reference,
      request_.dataset);

    std::vector<RtkOutageBoundaryReferenceRow> outage_boundary_refs;
    RtkOutageBoundaryReferenceRow outage_start_reference =
      MakeBoundaryReferenceFromTrajectory(
        request_.config,
        pre_result,
        outage.window_index,
        outage.start_time_s,
        "OUTAGE_START",
        "PRE_TERMINAL",
        allow_start_ba_z);
    DisableDirectRtkOutageBoundaryUpConstraint(outage_start_reference);
    outage_boundary_refs.push_back(std::move(outage_start_reference));
    const std::optional<double> outage_first_kept_time_s =
      FirstKeptOutageStateTime(request_.state_timestamps, outage);
    if (outage_first_kept_time_s.has_value()) {
      RtkOutageBoundaryReferenceRow start_vertical_source =
        MakeBoundaryReferenceFromTrajectory(
          request_.config,
          pre_result,
          outage.window_index,
          outage.start_time_s,
          "OUTAGE_START_VERTICAL_HANDOFF",
          "PRE_TERMINAL_VERTICAL_POSITION_VELOCITY_HANDOFF",
          false);
      RtkOutageBoundaryReferenceRow start_vertical_handoff_reference =
        MakeVerticalPositionVelocityHandoffReference(
          request_.config,
          std::move(start_vertical_source),
          "OUTAGE_START_VERTICAL_HANDOFF",
          "PRE_TERMINAL_VERTICAL_POSITION_VELOCITY_HANDOFF",
          *outage_first_kept_time_s,
          outage.start_time_s,
          false,
          "missing_start_vertical_handoff_reference");
      if (start_vertical_handoff_reference.valid) {
        outage_boundary_refs.push_back(std::move(start_vertical_handoff_reference));
      }
    }
    const bool allow_end_ba_z = AllowsRtkOutageBazContinuity(
      bias_policy,
      outage.window_index,
      "OUTAGE_END");
    const RtkOutageWindowRow *outage_source_outage =
      FindSourceOutage(request_.outage_windows, *outage_segment);
    OfflineRunnerConfig outage_config =
      MakeChildConfig(request_.config, request_.base_config, *outage_segment, outage_source_outage);
    std::shared_ptr<const Stage2VelocityReference> outage_reference =
      SliceStage2ReferenceForSegment(
        request_.stage2_reference,
        outage_config,
        *outage_segment,
        request_.dynamic_start_time_s);
    RtkOutageBoundaryReferenceRow outage_end_reference =
      MakeOutageEndPositionVelocityReferenceFromPostResult(
        request_.config,
        post_result,
        handoff_outage,
        allow_end_ba_z);
    if (!outage_end_reference.valid && recovery_reference != nullptr) {
      outage_end_reference = MakeOutageEndBoundaryReferenceFromRecovery(
        request_.config,
        *recovery_reference);
    }
    outage_boundary_refs.push_back(std::move(outage_end_reference));
    if (outage_reference != nullptr) {
      const RtkOutageBoundaryReferenceRow post_first_boundary_reference =
        outage_boundary_refs.back();
      const std::optional<RtkOutageBoundaryReferenceRow> terminal_velocity_reference =
        BuildRtkOutageTerminalVelocityReference(
          RtkOutageTerminalVelocityReferenceRequest{
            &request_.config,
            &request_.state_timestamps,
            &outage_reference->reference_states,
            &request_.dataset.imu_samples,
            request_.imu_params,
            &handoff_outage,
            &post_first_boundary_reference});
      if (terminal_velocity_reference.has_value()) {
        outage_boundary_refs.push_back(*terminal_velocity_reference);
      }
      const std::optional<RtkOutageBoundaryReferenceRow> terminal_vertical_handoff_reference =
        BuildRtkOutageTerminalVerticalHandoffReference(
          RtkOutageTerminalVelocityReferenceRequest{
            &request_.config,
            &request_.state_timestamps,
            &outage_reference->reference_states,
            &request_.dataset.imu_samples,
            request_.imu_params,
            &handoff_outage,
            &post_first_boundary_reference});
      if (terminal_vertical_handoff_reference.has_value()) {
        outage_boundary_refs.push_back(*terminal_vertical_handoff_reference);
      }
    }
    OfflineRunResult outage_result = request_.run_once(
      std::move(outage_config),
      WithBoundaryReferences(std::move(outage_reference), std::move(outage_boundary_refs)),
      nullptr,
      request_.road_noise_state_reference,
      request_.dataset);

    std::vector<SegmentedBatchResultPiece> pieces;
    pieces.reserve(3U);
    pieces.push_back(SegmentedBatchResultPiece{*pre_segment, std::move(pre_result)});
    pieces.push_back(SegmentedBatchResultPiece{*outage_segment, std::move(outage_result)});
    pieces.push_back(SegmentedBatchResultPiece{*post_segment, std::move(post_result)});
    OfflineRunResult assembled =
      assemble_pieces(std::move(pieces), 0U);
    assembled.rtk_outage_bias_continuity_policy = std::move(bias_policy);
    return assembled;
  }

  std::vector<SegmentedBatchResultPiece> pieces;
  pieces.reserve(segments.size());
  for (const auto &segment : segments) {
    const RtkOutageWindowRow *source_outage =
      FindSourceOutage(request_.outage_windows, segment);
    OfflineRunnerConfig child_config =
      MakeChildConfig(request_.config, request_.base_config, segment, source_outage);
    std::shared_ptr<const Stage2VelocityReference> child_stage2_reference =
      IsStandalonePrefixSegment(segment)
        ? nullptr
        : SliceStage2ReferenceForSegment(
            request_.stage2_reference,
            child_config,
            segment,
            request_.dynamic_start_time_s);
    OfflineRunResult child_result =
      request_.run_once(
        std::move(child_config),
        std::move(child_stage2_reference),
        nullptr,
        request_.road_noise_state_reference,
        request_.dataset);
    pieces.push_back(SegmentedBatchResultPiece{segment, std::move(child_result)});
  }

  return assemble_pieces(std::move(pieces));
}

}  // namespace offline_lc_minimal
