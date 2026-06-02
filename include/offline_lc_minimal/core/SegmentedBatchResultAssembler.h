#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/RunResultTypes.h"

namespace offline_lc_minimal {

struct SegmentedBatchResultPiece {
  RtkOutageBatchSegmentRow segment;
  OfflineRunResult result;
};

struct SegmentedBatchResultAssemblerRequest {
  std::vector<SegmentedBatchResultPiece> pieces;
  std::vector<RtkOutageWindowRow> outage_windows;
  double processing_start_time_s = 0.0;
  double processing_end_time_s = 0.0;
  bool vertical_boundary_jump_allowed = false;
  std::size_t additional_child_run_count = 0U;
};

class SegmentedBatchResultAssembler {
 public:
  explicit SegmentedBatchResultAssembler(SegmentedBatchResultAssemblerRequest request);

  [[nodiscard]] OfflineRunResult Assemble() const;

 private:
  SegmentedBatchResultAssemblerRequest request_;
};

}  // namespace offline_lc_minimal
