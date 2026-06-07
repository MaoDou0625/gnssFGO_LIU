#pragma once

#include <vector>

namespace offline_lc_minimal {

struct Stage3SharedReferenceDeltaSample {
  double s_m = 0.0;
  double raw_delta_m = 0.0;
};

struct Stage3SharedReferenceDeltaSmoothingOptions {
  double radius_m = 12.0;
};

[[nodiscard]] std::vector<double> SmoothStage3SharedReferenceDelta(
  const std::vector<Stage3SharedReferenceDeltaSample> &samples,
  Stage3SharedReferenceDeltaSmoothingOptions options = {});

}  // namespace offline_lc_minimal
