#pragma once

#include <vector>

#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"

namespace offline_lc_minimal {

struct RtkOutageInitialValueSmoothRequest {
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  const std::vector<VerticalVelocityDeltaPropagationRecord> *propagation_records = nullptr;
  gtsam::Values *initial_values = nullptr;
  std::vector<RtkOutageWindowRow> *outage_windows = nullptr;
};

class RtkOutageInitialValueSmoother {
 public:
  explicit RtkOutageInitialValueSmoother(RtkOutageInitialValueSmoothRequest request);

  void Apply() const;

 private:
  [[nodiscard]] std::vector<double> BuildVelocityProfile(
    const RtkOutageWindowRow &window,
    double pre_up_m,
    double post_up_m) const;

  RtkOutageInitialValueSmoothRequest request_;
};

}  // namespace offline_lc_minimal
