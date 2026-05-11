#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {

struct VerticalConstraintPolicyContext {
  gtsam::NonlinearFactorGraph *graph = nullptr;
  std::vector<VerticalEnvelopeDiagnosticRow> *envelope_diagnostics = nullptr;
  const std::vector<RtkVerticalDriftReferenceDiagnosticRow> *rtk_vertical_drift_reference_profile =
    nullptr;
};

class VerticalConstraintPolicy {
 public:
  virtual ~VerticalConstraintPolicy() = default;

  [[nodiscard]] virtual bool UsesDirectPositionFactor() const = 0;
  [[nodiscard]] virtual double VerticalSigmaUsedM(const Eigen::Vector3d &sigma_m) const = 0;

  virtual void AddSynchronized(
    const GnssSolutionSample &sample,
    std::size_t sample_index,
    double corrected_time_s,
    const StateMeasSyncResult &sync_result,
    const Eigen::Vector3d &sigma_m,
    const VerticalConstraintPolicyContext &context) const = 0;

  virtual void AddInterpolated(
    const GnssSolutionSample &sample,
    std::size_t sample_index,
    double corrected_time_s,
    const StateMeasSyncResult &sync_result,
    const Eigen::Vector3d &sigma_m,
    const gp::GPWNOJInterpolator &interpolator,
    const VerticalConstraintPolicyContext &context) const = 0;
};

[[nodiscard]] std::unique_ptr<VerticalConstraintPolicy> CreateVerticalConstraintPolicy(
  const OfflineRunnerConfig &config);

}  // namespace offline_lc_minimal
