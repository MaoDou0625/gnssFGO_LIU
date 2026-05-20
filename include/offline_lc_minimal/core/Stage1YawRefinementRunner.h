#pragma once

#include <functional>
#include <memory>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage1OutageLateralVelocityEnvelopeEstimator.h"

namespace offline_lc_minimal {

using Stage1RunOnce = std::function<OfflineRunResult(
  const OfflineRunnerConfig &,
  std::shared_ptr<const Stage1OutageBodyYEnvelopeReference>,
  DataSet)>;

struct Stage1YawRefinementRequest {
  OfflineRunnerConfig config;
  DataSet dataset;
  std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> body_y_envelope_reference;
  Stage1RunOnce run_once;
};

class Stage1YawRefinementRunner {
 public:
  explicit Stage1YawRefinementRunner(Stage1YawRefinementRequest request);

  [[nodiscard]] OfflineRunResult Run() const;

 private:
  Stage1YawRefinementRequest request_;
};

}  // namespace offline_lc_minimal
