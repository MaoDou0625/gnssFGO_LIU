#pragma once

#include <vector>

#include "offline_lc_minimal/common/RunResultTypes.h"

namespace offline_lc_minimal {

class RoadNoiseStateReference {
 public:
  explicit RoadNoiseStateReference(std::vector<RoadNoiseStateSegmentRow> segments);

  [[nodiscard]] const std::vector<RoadNoiseStateSegmentRow> &segments() const {
    return segments_;
  }
  [[nodiscard]] bool empty() const { return segments_.empty(); }
  [[nodiscard]] std::vector<RoadNoiseStateSegmentRow> Clip(
    double start_time_s,
    double end_time_s) const;

 private:
  std::vector<RoadNoiseStateSegmentRow> segments_;
};

}  // namespace offline_lc_minimal
