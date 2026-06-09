#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"

namespace offline_lc_minimal {

struct VelocityDeltaPropagationRecord {
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  gtsam::Vector3 target_delta_v_mps = gtsam::Vector3::Zero();
};

struct RtkOutageRecoveryConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<RtkOutageWindowRow> *outage_windows = nullptr;
  const std::vector<ReferenceNodeState> *reference_states = nullptr;
  std::string attitude_reference_source = "reference_states";
  const std::vector<ReferenceNodeState> *tilt_reference_states = nullptr;
  std::string tilt_reference_source = "reference_states";
  const std::vector<VelocityDeltaPropagationRecord> *velocity_delta_records = nullptr;
  const std::vector<VerticalVelocityDeltaPropagationRecord> *vertical_velocity_delta_records = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<RtkOutageAttitudeHoldDiagnosticRow> *attitude_diagnostics = nullptr;
  std::vector<RtkOutageVelocityDelta3dDiagnosticRow> *velocity_diagnostics = nullptr;
};

class RtkOutageRecoveryConstraintBuilder {
 public:
  explicit RtkOutageRecoveryConstraintBuilder(RtkOutageRecoveryConstraintBuildRequest request);

  void Build() const;

 private:
  [[nodiscard]] bool HasCompleteAttitudeRequest() const;
  [[nodiscard]] bool HasCompleteVelocityRequest() const;

  RtkOutageRecoveryConstraintBuildRequest request_;
};

void PopulateRtkOutageRecoveryDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<RtkOutageAttitudeHoldDiagnosticRow> &attitude_diagnostics,
  std::vector<RtkOutageVelocityDelta3dDiagnosticRow> &velocity_diagnostics,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
