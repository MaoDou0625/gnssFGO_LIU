#pragma once

#include <optional>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/RunResultTypes.h"

namespace offline_lc_minimal {

[[nodiscard]] std::optional<double> FirstKeptOutageStateTime(
  const std::vector<double> &state_timestamps,
  const RtkOutageWindowRow &outage);

void DisableDirectRtkOutageBoundaryUpConstraint(
  RtkOutageBoundaryReferenceRow &reference);

void ConfigureVerticalPositionVelocityHandoff(
  RtkOutageBoundaryReferenceRow &reference,
  const OfflineRunnerConfig &config,
  double target_time_s,
  double reference_time_s,
  bool add_direct_vz_constraint,
  const std::string &missing_reference_reason);

[[nodiscard]] RtkOutageBoundaryReferenceRow MakeVerticalPositionVelocityHandoffReference(
  const OfflineRunnerConfig &config,
  RtkOutageBoundaryReferenceRow reference,
  const std::string &boundary_role,
  const std::string &source_type,
  double target_time_s,
  double reference_time_s,
  bool add_direct_vz_constraint,
  const std::string &missing_reference_reason);

void AttachPostStartVerticalPositionVelocityHandoff(
  const OfflineRunnerConfig &config,
  const OfflineRunResult &outage_result,
  const RtkOutageWindowRow &outage,
  double post_first_time_s,
  RtkOutageBoundaryReferenceRow &reference);

}  // namespace offline_lc_minimal
