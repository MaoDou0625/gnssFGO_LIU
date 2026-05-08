#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include <Eigen/Core>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/BodyZBidirectionalJumpDetector.h"

namespace offline_lc_minimal {

struct BodyZWindowPipelineRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<ImuSample> *imu_samples = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const gtsam::NonlinearFactorGraph *base_graph = nullptr;
  const gtsam::Values *base_initial_values = nullptr;
  gtsam::LevenbergMarquardtParams optimizer_params;
  std::size_t navigation_start_index = 0;
  double dynamic_start_time_s = 0.0;
  double end_time_s = 0.0;

  std::function<bool(const GnssSolutionSample &sample)> passes_gnss_quality_filters;
  std::function<bool(double corrected_time_s)> is_within_imu_coverage;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
  std::function<Eigen::Vector3d(const GnssSolutionSample &sample)> clamped_sigma_m;
  std::function<StateMeasSyncResult(double corrected_time_s)> find_state_for_time_s;
};

struct BodyZWindowPipelineResult {
  BodyZJumpDetectionResult detection;
  std::vector<ReferenceNodeState> seed_reference_states;
  std::vector<BodyZSeedImuDiagnosticRow> imu_diagnostics;
  std::vector<BodyZSeedJumpWindowRow> jump_windows;
};

class BodyZWindowPipeline {
 public:
  explicit BodyZWindowPipeline(BodyZWindowPipelineRequest request);

  [[nodiscard]] BodyZWindowPipelineResult Run() const;

 private:
  BodyZWindowPipelineRequest request_;
};

}  // namespace offline_lc_minimal
