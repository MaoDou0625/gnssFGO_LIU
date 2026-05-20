#include "offline_lc_minimal/core/RtkOutageSegmentedBatchRunner.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>
#include <utility>

#include "offline_lc_minimal/core/GraphTimelineBuilder.h"
#include "offline_lc_minimal/core/RtkOutageBiasContinuityPolicy.h"
#include "offline_lc_minimal/core/RtkOutageBatchSegmentPlanner.h"
#include "offline_lc_minimal/core/RtkOutageRecoveryReferenceBuilder.h"
#include "offline_lc_minimal/core/SegmentedBatchResultAssembler.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsStandalonePrefixSegment(const RtkOutageBatchSegmentRow &segment) {
  return segment.segment_role == "PRE_RTK_VALID";
}

OfflineRunnerConfig MakeStandalonePrefixChildConfig(
  OfflineRunnerConfig config,
  const RtkOutageWindowRow *source_outage) {
  config.enable_rtk_outage_segmented_batch = false;
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
  config.enable_rtk_outage_segmented_batch = false;
  config.enable_rtk_outage_causal_drift_reference = false;
  config.enable_rtk_outage_preoutage_vertical_fence = false;
  config.enable_rtk_vertical_lowpass_reference = false;
  config.processing_start_time_s = segment.start_time_s;
  config.processing_end_time_s = segment.end_time_s;

  if (segment.segment_role == "RTK_OUTAGE") {
    if (source_outage != nullptr) {
      config.processing_start_time_s = 0.0;
      config.processing_end_time_s = source_outage->end_time_s;
    }
  } else if (segment.segment_role == "POST_RTK_VALID") {
    if (source_outage != nullptr) {
      config.processing_start_time_s = source_outage->end_time_s;
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

  sliced->trajectory.reserve(target_timestamps.size());
  for (const double time_s : target_timestamps) {
    sliced->trajectory.push_back(interpolate_row(time_s));
  }
  return sliced;
}

std::shared_ptr<const Stage2VelocityReference> WithBoundaryReferences(
  std::shared_ptr<const Stage2VelocityReference> reference,
  std::vector<RtkOutageBoundaryReferenceRow> boundary_references) {
  if (reference == nullptr) {
    return nullptr;
  }
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

  const TrajectoryRow *row = NearestTrajectoryRow(result.trajectory, target_time_s);
  if (row == nullptr) {
    reference.skip_reason = "missing_trajectory_reference";
    return reference;
  }
  reference.reference_up_m = row->enu_position_m.z();
  reference.reference_vz_mps = row->enu_velocity_mps.z();
  reference.reference_ba_z_mps2 = row->bias_acc.z();
  reference.has_up = std::isfinite(reference.reference_up_m);
  reference.has_vz = std::isfinite(reference.reference_vz_mps);
  reference.has_ba_z = allow_ba_z && std::isfinite(reference.reference_ba_z_mps2);
  reference.add_up_constraint = reference.has_up;
  reference.add_vz_constraint = reference.has_vz;
  reference.add_ba_z_constraint = reference.has_ba_z;
  reference.valid = reference.has_up || reference.has_vz || reference.has_ba_z;
  reference.skip_reason = reference.valid ? "OK" : "nonfinite_trajectory_reference";
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

bool HasValidRecoveryReference(
  const std::vector<RtkOutageRecoveryReferenceRow> &rows,
  const std::size_t window_index) {
  return std::any_of(
    rows.begin(),
    rows.end(),
    [&](const RtkOutageRecoveryReferenceRow &row) {
      return row.window_index == window_index && row.valid;
    });
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
      nullptr,
      request_.dataset);
  }

  std::vector<RtkOutageRecoveryReferenceRow> recovery_references;
  if (request_.config.enable_rtk_outage_boundary_constraints) {
    recovery_references = BuildRecoveryReferences(
      request_.config,
      request_.dataset,
      request_.gnss_factor_records,
      request_.outage_windows);
  }

  const auto assemble_pieces =
    [&](std::vector<SegmentedBatchResultPiece> pieces) {
      SegmentedBatchResultAssemblerRequest assemble_request;
      assemble_request.pieces = std::move(pieces);
      assemble_request.outage_windows = request_.outage_windows;
      assemble_request.processing_start_time_s = 0.0;
      assemble_request.processing_end_time_s = request_.processing_end_time_s;
      assemble_request.vertical_boundary_jump_allowed =
        request_.config.rtk_outage_segmented_batch_allow_vertical_boundary_jump;
      OfflineRunResult assembled =
        SegmentedBatchResultAssembler(std::move(assemble_request)).Assemble();
      assembled.rtk_outage_recovery_references = recovery_references;
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
    post_segment != nullptr && source_outage != nullptr &&
    HasValidRecoveryReference(recovery_references, source_outage->window_index);

  if (can_run_boundary_handoff) {
    const RtkOutageWindowRow &outage = *source_outage;
    const RtkOutageRecoveryReferenceRow *recovery_reference =
      FindRecoveryReference(recovery_references, outage.window_index);

    OfflineRunResult pre_result = run_segment(*pre_segment, nullptr);

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
      post_boundary_refs.push_back(
        MakePostStartBoundaryReferenceFromRecovery(
          request_.config,
          *recovery_reference));
    }
    OfflineRunResult post_result = request_.run_once(
      std::move(post_config),
      WithBoundaryReferences(std::move(post_reference), std::move(post_boundary_refs)),
      nullptr,
      request_.dataset);

    std::vector<RtkOutageBiasContinuityPolicyRow> bias_policy =
      BuildBiasContinuityPolicyRows(
        request_.config,
        request_.outage_windows,
        request_.bias_reestimate_segments);
    const bool allow_start_ba_z = AllowsRtkOutageBazContinuity(
      bias_policy,
      outage.window_index,
      "OUTAGE_START");
    const bool allow_end_ba_z = AllowsRtkOutageBazContinuity(
      bias_policy,
      outage.window_index,
      "OUTAGE_END");
    std::vector<RtkOutageBoundaryReferenceRow> outage_boundary_refs;
    outage_boundary_refs.push_back(MakeBoundaryReferenceFromTrajectory(
      request_.config,
      pre_result,
      outage.window_index,
      outage.start_time_s,
      "OUTAGE_START",
      "PRE_TERMINAL",
      allow_start_ba_z));
    outage_boundary_refs.push_back(MakeBoundaryReferenceFromTrajectory(
      request_.config,
      post_result,
      outage.window_index,
      outage.end_time_s,
      "OUTAGE_END",
      "POST_RECOVERY_OPTIMIZED",
      allow_end_ba_z));

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
    OfflineRunResult outage_result = request_.run_once(
      std::move(outage_config),
      WithBoundaryReferences(std::move(outage_reference), std::move(outage_boundary_refs)),
      nullptr,
      request_.dataset);

    std::vector<SegmentedBatchResultPiece> pieces;
    pieces.reserve(3U);
    pieces.push_back(SegmentedBatchResultPiece{*pre_segment, std::move(pre_result)});
    pieces.push_back(SegmentedBatchResultPiece{*outage_segment, std::move(outage_result)});
    pieces.push_back(SegmentedBatchResultPiece{*post_segment, std::move(post_result)});
    OfflineRunResult assembled = assemble_pieces(std::move(pieces));
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
        request_.dataset);
    pieces.push_back(SegmentedBatchResultPiece{segment, std::move(child_result)});
  }

  return assemble_pieces(std::move(pieces));
}

}  // namespace offline_lc_minimal
