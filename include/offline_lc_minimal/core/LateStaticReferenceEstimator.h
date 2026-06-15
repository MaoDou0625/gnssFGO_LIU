#pragma once

#include <functional>
#include <vector>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct LateStaticReferenceEstimationRequest {
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  std::vector<LateStaticWindowRow> *windows = nullptr;

  std::function<bool(const GnssSolutionSample &sample)> should_use_rtkfix_sample;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
};

class LateStaticReferenceEstimator {
 public:
  explicit LateStaticReferenceEstimator(LateStaticReferenceEstimationRequest request);

  void Estimate() const;

 private:
  LateStaticReferenceEstimationRequest request_;
};

}  // namespace offline_lc_minimal
