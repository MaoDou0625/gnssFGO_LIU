#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/RunResultTypes.h"

namespace offline_lc_minimal {

struct Stage1YawBranchCandidate {
  OfflineRunResult result;
  Stage1YawRefinementDiagnosticRow diagnostic;
};

struct Stage1YawBranchResolution {
  bool has_selection = false;
  bool cycle_detected = false;
  bool reference_valid_for_strong_hold = false;
  std::size_t selected_index = 0;
  int selected_iteration = 0;
  double selected_branch_score = 0.0;
  std::string stop_reason = "max_iterations";
  std::string selection_reason = "UNSET";
  std::vector<Stage1YawRefinementDiagnosticRow> diagnostics;
};

class Stage1YawBranchResolver {
 public:
  [[nodiscard]] Stage1YawBranchResolution Resolve(
    std::vector<Stage1YawBranchCandidate> candidates) const;
};

}  // namespace offline_lc_minimal
