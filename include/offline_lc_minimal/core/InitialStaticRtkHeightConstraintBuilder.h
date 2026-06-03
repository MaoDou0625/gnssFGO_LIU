#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/SensorTypes.h"

namespace offline_lc_minimal {

struct InitialStaticRtkHeightReference {
  bool valid = false;
  std::size_t sample_count = 0;
  double reference_up_m = 0.0;
};

class InitialStaticRtkHeightConstraintBuilder {
 public:
  [[nodiscard]] static bool Enabled(const OfflineRunnerConfig &config);

  [[nodiscard]] static InitialStaticRtkHeightReference BuildReference(
    const std::vector<GnssSolutionSample> &gnss_samples,
    double alignment_start_time_s,
    double alignment_end_time_s,
    const OfflineRunnerConfig &config);

  [[nodiscard]] static bool AddVerticalReference(
    const InitialStaticRtkHeightReference &reference,
    const OfflineRunnerConfig &config,
    gtsam::NonlinearFactorGraph &graph,
    gtsam::Key pose_key);
};

}  // namespace offline_lc_minimal
