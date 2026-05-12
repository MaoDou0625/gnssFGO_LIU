#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkOutageWindowPlanRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *body_z_jump_windows = nullptr;
  std::size_t navigation_start_index = 0;

  std::function<bool(const GnssSolutionSample &sample)> passes_gnss_quality_filters;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
};

class RtkOutageWindowPlanner {
 public:
  explicit RtkOutageWindowPlanner(RtkOutageWindowPlanRequest request);

  [[nodiscard]] std::vector<RtkOutageWindowRow> Plan() const;

 private:
  [[nodiscard]] std::size_t NearestStateIndex(double time_s) const;
  [[nodiscard]] std::size_t CountRejectedSamples(std::size_t begin_index, std::size_t end_index) const;
  [[nodiscard]] std::size_t CountBodyZOverlaps(double start_time_s, double end_time_s) const;

  RtkOutageWindowPlanRequest request_;
};

[[nodiscard]] std::vector<BodyZSeedJumpWindowRow> BuildRtkOutageNHCWindows(
  const std::vector<BodyZSeedJumpWindowRow> &body_z_jump_windows,
  const std::vector<RtkOutageWindowRow> &rtk_outage_windows);

}  // namespace offline_lc_minimal
