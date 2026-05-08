#pragma once

#include <string>

#include "offline_lc_minimal/common/Config.h"

namespace offline_lc_minimal {

struct VerticalVelocityDeltaSigmaResult {
  std::string model = "legacy";
  double sigma_mps = 0.0;
  double legacy_sigma_mps = 0.0;
  double bias_sigma_mps = 0.0;
  double attitude_sigma_mps = 0.0;
  double sigma_floor_mps = 0.0;
  double sigma_ceiling_mps = 0.0;
  bool clamped_floor = false;
  bool clamped_ceiling = false;
};

class VerticalVelocityDeltaSigmaModel {
 public:
  explicit VerticalVelocityDeltaSigmaModel(const OfflineRunnerConfig &config);

  [[nodiscard]] VerticalVelocityDeltaSigmaResult Compute(double dt_s) const;

 private:
  const OfflineRunnerConfig &config_;
};

}  // namespace offline_lc_minimal
