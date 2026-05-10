#pragma once

#include <string>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"

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
  [[nodiscard]] VerticalVelocityDeltaSigmaResult Compute(
    double dt_s,
    const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry) const;

 private:
  [[nodiscard]] VerticalVelocityDeltaSigmaResult ComputeBiasConsistent(
    double dt_s,
    double bias_sigma_mps2,
    double attitude_sigma_rad,
    double floor_mps,
    double ceiling_mps,
    std::string model) const;

  const OfflineRunnerConfig &config_;
};

}  // namespace offline_lc_minimal
