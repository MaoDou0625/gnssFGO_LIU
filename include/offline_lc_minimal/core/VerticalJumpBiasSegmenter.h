#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct VerticalJumpBiasSpanInput {
  std::size_t span_index = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  std::vector<std::size_t> source_window_indices;
};

struct VerticalJumpBiasSegmentEstimate {
  std::size_t span_index = 0;
  std::size_t segment_index = 0;
  std::size_t segment_count = 1;
  std::size_t bias_key_index = 0;
  std::size_t source_window_index = 0;
  std::size_t source_window_count = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  double source_window_duration_s = 0.0;
  double detected_signed_delta_velocity_mps = 0.0;
  double detected_bias_mps2 = 0.0;
  bool used_segmented_estimate = false;
  double highfreq_rms_mps2 = 0.0;
  double highfreq_p95_abs_mps2 = 0.0;
};

struct VerticalJumpBiasSegmenterRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  const std::vector<BodyZSeedImuDiagnosticRow> *body_z_diagnostics = nullptr;
  const std::vector<VerticalJumpBiasSpanInput> *spans = nullptr;
};

std::vector<VerticalJumpBiasSegmentEstimate> EstimateVerticalJumpBiasSegments(
  const VerticalJumpBiasSegmenterRequest &request);

}  // namespace offline_lc_minimal
