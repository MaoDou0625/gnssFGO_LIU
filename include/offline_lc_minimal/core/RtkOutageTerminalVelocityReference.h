#pragma once

#include <optional>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/SensorTypes.h"

namespace offline_lc_minimal {

struct RtkOutageTerminalVelocityReferenceRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<ReferenceNodeState> *reference_states = nullptr;
  const std::vector<ImuSample> *imu_samples = nullptr;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> imu_params;
  const RtkOutageWindowRow *outage = nullptr;
  const RtkOutageBoundaryReferenceRow *post_boundary_reference = nullptr;
};

[[nodiscard]] std::optional<RtkOutageBoundaryReferenceRow>
BuildRtkOutageTerminalVelocityReference(
  const RtkOutageTerminalVelocityReferenceRequest &request);

[[nodiscard]] std::optional<RtkOutageBoundaryReferenceRow>
BuildRtkOutageTerminalVerticalHandoffReference(
  const RtkOutageTerminalVelocityReferenceRequest &request);

[[nodiscard]] std::optional<RtkOutageBoundaryReferenceRow>
BuildRtkOutageTerminalHorizontalHandoffReference(
  const RtkOutageTerminalVelocityReferenceRequest &request);

}  // namespace offline_lc_minimal
