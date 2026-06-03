#pragma once

#include <functional>
#include <limits>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkOutageCausalReferenceBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const OfflineRunnerConfig *prefix_base_config = nullptr;
  const DataSet *dataset = nullptr;
  const std::vector<RtkOutageWindowRow> *outage_windows = nullptr;
  double dynamic_start_time_s = 0.0;

  std::function<bool(const GnssSolutionSample &sample)> should_use_sample;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
  std::function<bool(double corrected_time_s)> is_within_imu_coverage;
  std::function<OfflineRunResult(
    OfflineRunnerConfig config,
    DataSet dataset)> run_prefix;
};

struct RtkOutageCausalReferenceResult {
  bool valid = false;
  std::size_t prefix_run_count = 0;
  double boundary_time_s = std::numeric_limits<double>::quiet_NaN();
  std::vector<RtkOutageCausalNavReferenceRow> nav_reference_rows;
  std::vector<RtkOutageCausalStateReferenceRow> state_reference_rows;
};

class RtkOutageCausalReferenceBuilder {
 public:
  explicit RtkOutageCausalReferenceBuilder(RtkOutageCausalReferenceBuildRequest request);

  [[nodiscard]] RtkOutageCausalReferenceResult Build() const;

 private:
  RtkOutageCausalReferenceBuildRequest request_;
};

}  // namespace offline_lc_minimal
