#pragma once

#include <limits>
#include <optional>
#include <vector>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/GraphTimelineBuilder.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {

struct ForwardDriftSummary {
  double up_slope_10s = std::numeric_limits<double>::quiet_NaN();
  double up_slope_30s = std::numeric_limits<double>::quiet_NaN();
  double horizontal_slope_10s = std::numeric_limits<double>::quiet_NaN();
  double horizontal_slope_30s = std::numeric_limits<double>::quiet_NaN();
};

void PopulateGnssPostfitResiduals(
  const gtsam::Values &optimized_values,
  const gp::GPWNOJInterpolator &base_interpolator,
  const std::vector<std::optional<std::size_t>> &trajectory_row_index_by_state,
  std::vector<GnssFactorRecord> &gnss_factor_records,
  std::vector<GnssConsistencyRecord> *gnss_consistency_records,
  std::vector<TrajectoryRow> *trajectory_rows);

void PopulateVerticalEnvelopeDiagnostics(
  const gtsam::Values &optimized_values,
  const gp::GPWNOJInterpolator &base_interpolator,
  std::vector<VerticalEnvelopeDiagnosticRow> &vertical_envelope_diagnostics);

void PopulateRtkVerticalLatentReferenceDiagnostics(
  const gtsam::Values &optimized_values,
  const std::vector<GnssSolutionSample> &gnss_samples,
  std::vector<RtkVerticalLatentReferenceDiagnosticRow> &diagnostics,
  RunSummary &run_summary);

[[nodiscard]] ForwardDriftSummary ComputeFeedbackForwardDriftSummary(
  const std::vector<ImuSample> &imu_samples,
  const gtsam::Pose3 &start_pose,
  const gtsam::Vector3 &start_velocity,
  const gtsam::imuBias::ConstantBias &bias,
  double start_time_s,
  double max_time_s,
  double gravity_mps2);

void AccumulateForwardTrajectoryVariationSummary(
  const std::vector<TrajectoryRow> &rows,
  double &up_total_variation_m,
  double &vz_total_variation_mps);

void AccumulateInitialDynamicConsistencyMetrics(
  const std::vector<TrajectoryRow> &trajectory_rows,
  const gtsam::imuBias::ConstantBias &initial_bias,
  const std::optional<TrajectoryRow> &optimized_last_static_row,
  const std::vector<TrajectoryRow> &optimized_static_terminal_forward_rows,
  RunSummary &run_summary);

[[nodiscard]] std::vector<SegmentErrorDiagnostic> BuildSegmentErrorDiagnostics(
  const std::vector<double> &state_timestamps,
  const std::vector<ImuSample> &imu_samples,
  const boost::shared_ptr<gtsam::PreintegrationCombinedParams> &imu_params,
  const gtsam::Values &optimized_values,
  const std::vector<GnssFactorRecord> &gnss_factor_records,
  const std::vector<GnssConsistencyRecord> &gnss_consistency_records,
  const OfflineRunnerConfig &config);

void AccumulateStaticConsistencyMetrics(
  const gtsam::Values &optimized_values,
  const GraphTimeline &graph_timeline,
  RunSummary &run_summary);

void AccumulateStaticSpecificForceWindowMetrics(
  const std::vector<ImuSample> &imu_samples,
  double start_time_s,
  double end_time_s,
  double window_duration_s,
  RunSummary &run_summary);

[[nodiscard]] std::vector<StaticAlignmentValidationRow> BuildStaticAlignmentValidation(
  const gtsam::Values &optimized_values,
  const GraphTimeline &graph_timeline,
  gtsam::Key global_acc_bias_key,
  double vertical_acc_bias_tau_s,
  double static_rtk_reference_up_m,
  RunSummary &run_summary);

void AccumulateGnssConsistencySummary(
  const std::vector<GnssConsistencyRecord> &gnss_consistency_records,
  const OfflineRunnerConfig &config,
  RunSummary &run_summary);

[[nodiscard]] std::vector<VerticalStateCorrectionRow> BuildVerticalStateCorrections(
  const std::vector<double> &state_timestamps,
  const gtsam::Values &optimized_values,
  const std::vector<GnssFactorRecord> &gnss_factor_records,
  const std::vector<GnssConsistencyRecord> &gnss_consistency_records,
  bool collect_gnss_consistency);

}  // namespace offline_lc_minimal
