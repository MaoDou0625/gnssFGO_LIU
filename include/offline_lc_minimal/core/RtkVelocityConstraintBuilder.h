#pragma once

#include <functional>
#include <map>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkVelocityConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<RtkVelocityDiagnosticRow> *diagnostics = nullptr;
  std::size_t dynamic_start_index = 0;

  std::function<bool(const GnssSolutionSample &sample)> should_use_sample;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
  std::function<StateMeasSyncResult(double corrected_time_s)> find_state_for_time_s;
};

class RtkVelocityConstraintBuilder {
 public:
  explicit RtkVelocityConstraintBuilder(RtkVelocityConstraintBuildRequest request);

  void Build() const;

  static void PopulateDiagnostics(
    const gtsam::Values &optimized_values,
    std::vector<RtkVelocityDiagnosticRow> &diagnostics,
    RunSummary &run_summary);

 private:
  void Validate() const;

  RtkVelocityConstraintBuildRequest request_;
};

}  // namespace offline_lc_minimal
