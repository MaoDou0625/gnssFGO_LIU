#pragma once

#include <functional>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"
#include "offline_lc_minimal/common/SensorTypes.h"

namespace offline_lc_minimal {

struct RtkOutageRecoveryReferenceBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  const std::vector<GnssFactorRecord> *gnss_factor_records = nullptr;
  const std::vector<RtkOutageWindowRow> *outage_windows = nullptr;
  std::function<bool(const GnssSolutionSample &)> passes_gnss_quality_filters;
  std::function<double(const GnssSolutionSample &)> corrected_time_s;
};

class RtkOutageRecoveryReferenceBuilder {
 public:
  explicit RtkOutageRecoveryReferenceBuilder(
    RtkOutageRecoveryReferenceBuildRequest request);

  [[nodiscard]] std::vector<RtkOutageRecoveryReferenceRow> Build() const;

 private:
  RtkOutageRecoveryReferenceBuildRequest request_;
};

[[nodiscard]] RtkOutageBoundaryReferenceRow
MakePostStartBoundaryReferenceFromRecovery(
  const OfflineRunnerConfig &config,
  const RtkOutageRecoveryReferenceRow &reference);

[[nodiscard]] RtkOutageBoundaryReferenceRow
MakePostStartBoundaryReferenceFromRecovery(
  const OfflineRunnerConfig &config,
  const RtkOutageRecoveryReferenceRow &reference,
  double post_start_time_s);

[[nodiscard]] RtkOutageBoundaryReferenceRow
MakeOutageEndBoundaryReferenceFromRecovery(
  const OfflineRunnerConfig &config,
  const RtkOutageRecoveryReferenceRow &reference);

}  // namespace offline_lc_minimal
