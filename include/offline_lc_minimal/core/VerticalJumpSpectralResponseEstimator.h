#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/core/VerticalJumpBiasSegmenter.h"

namespace offline_lc_minimal {

struct VerticalJumpSpectralResponseEstimate {
  bool enabled = false;
  bool valid = false;
  std::string skip_reason = "DISABLED";
  std::size_t target_window_count = 0;
  std::size_t reference_window_count = 0;
  double target_total_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  double reference_total_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  double total_rms_ratio = std::numeric_limits<double>::quiet_NaN();
  double band_30_60_rms_ratio = std::numeric_limits<double>::quiet_NaN();
  double band_60_120_rms_ratio = std::numeric_limits<double>::quiet_NaN();
  double band_120_250_rms_ratio = std::numeric_limits<double>::quiet_NaN();
  double response_ratio = std::numeric_limits<double>::quiet_NaN();
  double score = 0.0;
};

struct VerticalJumpSpectralResponseRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<BodyZSeedImuDiagnosticRow> *body_z_diagnostics = nullptr;
  const std::vector<VerticalJumpBiasSpanInput> *excluded_spans = nullptr;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
};

[[nodiscard]] VerticalJumpSpectralResponseEstimate EstimateVerticalJumpSpectralResponse(
  const VerticalJumpSpectralResponseRequest &request);

[[nodiscard]] double ComputeVerticalJumpSpectralEffectivePriorSigma(
  const OfflineRunnerConfig &config,
  double base_prior_sigma_mps2,
  const VerticalJumpSpectralResponseEstimate &estimate);

}  // namespace offline_lc_minimal
