#pragma once

#include <string>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

inline constexpr const char *kPostStartBazHandoffSource =
  "POST_START_BAZ_HANDOFF";

struct RtkOutageBoundaryBiasHandoffRequest {
  const OfflineRunnerConfig *config = nullptr;
  const OfflineRunResult *outage_result = nullptr;
  const RtkOutageWindowRow *outage_window = nullptr;
  double post_first_time_s = 0.0;
};

struct RtkOutageBoundaryBiasHandoffResult {
  bool valid = false;
  std::string skip_reason = "UNSET";
  RtkOutageBoundaryReferenceRow boundary_reference;
};

[[nodiscard]] RtkOutageBoundaryBiasHandoffResult
BuildRtkOutageBoundaryBiasHandoff(
  const RtkOutageBoundaryBiasHandoffRequest &request);

void AttachRtkOutageBoundaryBiasHandoff(
  const RtkOutageBoundaryBiasHandoffResult &handoff,
  RtkOutageBoundaryReferenceRow &reference);

}  // namespace offline_lc_minimal
