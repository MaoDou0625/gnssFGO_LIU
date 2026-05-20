#include "offline_lc_minimal/core/Stage1YawRefinementRunner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/RtkHeadingAlignmentEstimator.h"

namespace offline_lc_minimal {
namespace {

double Clamp(const double value, const double limit) {
  return std::clamp(value, -limit, limit);
}

double InitialYawForNextUpdate(
  const OfflineRunnerConfig &config,
  const OfflineRunResult &result) {
  if (config.enable_initial_yaw_override) {
    return NormalizeHeadingAngleRad(config.initial_yaw_override_rad);
  }

  const auto it = std::lower_bound(
    result.trajectory.begin(),
    result.trajectory.end(),
    result.run_summary.dynamic_start_time_s,
    [](const TrajectoryRow &row, const double time_s) {
      return row.time_s < time_s;
    });
  if (it != result.trajectory.end() && std::isfinite(it->ypr_rad.x())) {
    return NormalizeHeadingAngleRad(it->ypr_rad.x());
  }
  for (const auto &row : result.trajectory) {
    if (std::isfinite(row.ypr_rad.x())) {
      return NormalizeHeadingAngleRad(row.ypr_rad.x());
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

void ApplyStage1Summary(
  OfflineRunResult &result,
  const std::vector<Stage1YawRefinementDiagnosticRow> &diagnostics,
  const bool converged,
  const std::string &stop_reason,
  const double final_yaw_rad) {
  result.stage1_yaw_refinement_diagnostics = diagnostics;
  result.run_summary.stage1_yaw_refinement_enabled = true;
  result.run_summary.stage1_yaw_refinement_iteration_count =
    static_cast<int>(diagnostics.size());
  result.run_summary.stage1_yaw_refinement_converged = converged;
  result.run_summary.stage1_yaw_refinement_stop_reason = stop_reason;
  result.run_summary.stage1_yaw_refinement_final_yaw_rad = final_yaw_rad;
  if (!diagnostics.empty()) {
    const auto &last = diagnostics.back();
    result.run_summary.stage1_yaw_refinement_final_median_error_rad =
      last.median_error_rad;
    result.run_summary.stage1_yaw_refinement_final_noise_rad =
      last.heading_noise_rad;
    result.run_summary.stage1_yaw_refinement_final_update_rad =
      last.yaw_update_rad;
  }
}

Stage1YawRefinementDiagnosticRow MakeDiagnosticRow(
  const int iteration,
  const double input_yaw_rad,
  const RtkHeadingAlignmentEstimate &estimate,
  const OfflineRunResult &result) {
  Stage1YawRefinementDiagnosticRow row;
  row.iteration = iteration;
  row.input_yaw_rad = input_yaw_rad;
  row.median_error_rad = estimate.median_error_rad;
  row.heading_noise_rad = estimate.heading_noise_rad;
  row.next_yaw_rad = input_yaw_rad;
  row.valid_pair_count = estimate.valid_pair_count;
  row.mean_abs_error_rad = estimate.mean_abs_error_rad;
  row.rms_error_rad = estimate.rms_error_rad;
  row.max_abs_error_rad = estimate.max_abs_error_rad;
  row.final_error = result.run_summary.final_error;
  row.gnss_nis_mean = result.run_summary.gnss_nis_mean;
  row.stop_reason = estimate.stop_reason;
  return row;
}

}  // namespace

Stage1YawRefinementRunner::Stage1YawRefinementRunner(Stage1YawRefinementRequest request)
    : request_(std::move(request)) {}

OfflineRunResult Stage1YawRefinementRunner::Run() const {
  if (!request_.run_once) {
    throw std::runtime_error("stage1 yaw refinement requires a run_once callback");
  }

  OfflineRunnerConfig stage_config = MakeStage1YawRefinementConfig(request_.config);
  std::vector<Stage1YawRefinementDiagnosticRow> diagnostics;
  diagnostics.reserve(
    static_cast<std::size_t>(stage_config.stage1_yaw_refinement_max_iterations));

  OfflineRunResult last_result;
  double final_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  std::string stop_reason = "max_iterations";
  bool converged = false;

  for (int iteration = 1; iteration <= stage_config.stage1_yaw_refinement_max_iterations; ++iteration) {
    last_result = request_.run_once(
      stage_config,
      request_.body_y_envelope_reference,
      request_.dataset);
    const double input_yaw_rad = InitialYawForNextUpdate(stage_config, last_result);
    final_yaw_rad = input_yaw_rad;

    const GeoReference geo_reference(
      last_result.run_summary.origin_lat_rad,
      last_result.run_summary.origin_lon_rad,
      last_result.run_summary.origin_h_m);
    RtkHeadingAlignmentRequest estimate_request;
    estimate_request.gnss_samples = &request_.dataset.gnss_samples;
    estimate_request.trajectory = &last_result.trajectory;
    estimate_request.geo_reference = &geo_reference;
    estimate_request.options.heading_window_s = stage_config.stage1_heading_window_s;
    estimate_request.options.time_tolerance_s = stage_config.stage1_heading_time_tolerance_s;
    estimate_request.options.min_displacement_m = stage_config.stage1_heading_min_displacement_m;
    estimate_request.options.gnss_time_offset_s = stage_config.gnss_time_offset_s;
    estimate_request.options.dynamic_start_time_s = last_result.run_summary.dynamic_start_time_s;
    estimate_request.options.end_time_s = last_result.run_summary.processing_end_time_s;

    const RtkHeadingAlignmentEstimate estimate =
      RtkHeadingAlignmentEstimator(estimate_request).Estimate();
    Stage1YawRefinementDiagnosticRow row =
      MakeDiagnosticRow(iteration, input_yaw_rad, estimate, last_result);

    if (!estimate.valid || !std::isfinite(input_yaw_rad)) {
      stop_reason = estimate.valid ? "invalid_input_yaw" : estimate.stop_reason;
      row.stop_reason = stop_reason;
      diagnostics.push_back(row);
      ApplyStage1Summary(last_result, diagnostics, false, stop_reason, final_yaw_rad);
      return last_result;
    }

    const double heading_noise_rad = std::isfinite(estimate.heading_noise_rad)
                                       ? estimate.heading_noise_rad
                                       : 0.0;
    const double stop_threshold_rad =
      std::max(stage_config.stage1_heading_noise_floor_rad, heading_noise_rad);
    if (std::abs(estimate.median_error_rad) <= stop_threshold_rad) {
      stop_reason = "converged";
      converged = true;
      row.stop_reason = stop_reason;
      diagnostics.push_back(row);
      ApplyStage1Summary(last_result, diagnostics, converged, stop_reason, final_yaw_rad);
      return last_result;
    }

    if (iteration == stage_config.stage1_yaw_refinement_max_iterations) {
      stop_reason = "max_iterations";
      row.stop_reason = stop_reason;
      diagnostics.push_back(row);
      ApplyStage1Summary(last_result, diagnostics, false, stop_reason, final_yaw_rad);
      return last_result;
    }

    const double yaw_update_rad =
      -Clamp(estimate.median_error_rad, stage_config.stage1_yaw_update_max_rad);
    const double next_yaw_rad = NormalizeHeadingAngleRad(input_yaw_rad + yaw_update_rad);
    row.yaw_update_rad = yaw_update_rad;
    row.next_yaw_rad = next_yaw_rad;
    row.stop_reason = "updated";
    diagnostics.push_back(row);

    stage_config.enable_initial_yaw_override = true;
    stage_config.initial_yaw_override_rad = next_yaw_rad;
    final_yaw_rad = next_yaw_rad;
  }

  ApplyStage1Summary(last_result, diagnostics, converged, stop_reason, final_yaw_rad);
  return last_result;
}

}  // namespace offline_lc_minimal
