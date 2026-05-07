#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct StaticVerticalBiasCalibrationRequest {
  const std::vector<ImuSample> *imu_samples = nullptr;
  double alignment_start_time_s = 0.0;
  double alignment_end_time_s = 0.0;
  Eigen::Vector3d earth_rate_enu = Eigen::Vector3d::Zero();
  OfflineRunnerConfig config;
};

struct StaticVerticalBiasCalibrationResult {
  double static_window_start_s = 0.0;
  double static_window_end_s = 0.0;
  std::size_t static_sample_count = 0;
  std::size_t static_state_count = 0;
  double initial_baz_mps2 = 0.0;
  double static_baz_ref_mps2 = 0.0;
  double initial_error = 0.0;
  double final_error = 0.0;
};

class StaticVerticalBiasCalibrator {
 public:
  [[nodiscard]] static StaticVerticalBiasCalibrationResult Calibrate(
    const StaticVerticalBiasCalibrationRequest &request);
};

[[nodiscard]] StaticVerticalBiasCarryoverDiagnosticRow BuildStaticVerticalBiasCarryoverDiagnostic(
  const StaticVerticalBiasCalibrationResult &calibration,
  const gtsam::Values &optimized_values,
  const std::vector<double> &state_timestamps_s,
  std::size_t dynamic_start_index,
  gtsam::Key global_acc_bias_key);

}  // namespace offline_lc_minimal
