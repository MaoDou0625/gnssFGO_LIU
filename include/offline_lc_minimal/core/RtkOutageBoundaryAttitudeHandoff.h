#pragma once

#include <string>

#include <boost/shared_ptr.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

inline constexpr const char *kPostStartImuRelativeHandoffSource =
  "POST_START_IMU_RELATIVE_HANDOFF";

struct RtkOutageBoundaryAttitudeHandoffRequest {
  const OfflineRunnerConfig *config = nullptr;
  const DataSet *dataset = nullptr;
  const OfflineRunResult *outage_result = nullptr;
  const RtkOutageWindowRow *outage_window = nullptr;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> imu_params;
  double post_first_time_s = 0.0;
};

struct RtkOutageBoundaryAttitudeHandoffResult {
  bool valid = false;
  std::string skip_reason = "UNSET";
  RtkOutageBoundaryReferenceRow boundary_reference;
  RtkOutageAttitudeHoldDiagnosticRow diagnostic;
};

[[nodiscard]] RtkOutageBoundaryAttitudeHandoffResult
BuildRtkOutageBoundaryAttitudeHandoff(
  const RtkOutageBoundaryAttitudeHandoffRequest &request);

void AttachRtkOutageBoundaryAttitudeHandoff(
  const RtkOutageBoundaryAttitudeHandoffResult &handoff,
  RtkOutageBoundaryReferenceRow &reference);

void PopulateRtkOutageBoundaryAttitudeHandoffDiagnostic(
  const OfflineRunResult &post_result,
  RtkOutageAttitudeHoldDiagnosticRow &diagnostic);

}  // namespace offline_lc_minimal
