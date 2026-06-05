#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct AttitudeReferenceConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<ReferenceNodeState> *reference_states = nullptr;
  const std::vector<ReferenceNodeState> *tilt_reference_states = nullptr;
  const std::vector<ReferenceNodeState> *yaw_reference_states = nullptr;
  const std::vector<ReferenceNodeState> *relative_yaw_reference_states = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<AttitudeReferenceDiagnosticRow> *diagnostics = nullptr;
  std::vector<RelativeYawReferenceDiagnosticRow> *relative_yaw_diagnostics = nullptr;
};

class AttitudeReferenceConstraintBuilder {
 public:
  explicit AttitudeReferenceConstraintBuilder(AttitudeReferenceConstraintBuildRequest request);

  void Build() const;

 private:
  AttitudeReferenceConstraintBuildRequest request_;
};

void PopulateAttitudeReferenceDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<AttitudeReferenceDiagnosticRow> &diagnostics);

void PopulateRelativeYawReferenceDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<RelativeYawReferenceDiagnosticRow> &diagnostics);

}  // namespace offline_lc_minimal
