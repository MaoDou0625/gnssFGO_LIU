#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"

namespace offline_lc_minimal {

struct GnssVerticalReferenceSelection {
  bool valid = false;
  std::string source = "raw_rtk";
  std::string skip_reason = "UNSET";
  double raw_up_m = std::numeric_limits<double>::quiet_NaN();
  double selected_up_m = std::numeric_limits<double>::quiet_NaN();
  double highfreq_residual_m = std::numeric_limits<double>::quiet_NaN();
  double rtk_drift_estimate_m = std::numeric_limits<double>::quiet_NaN();
};

struct GnssVerticalReferenceSelectionRequest {
  GnssVerticalReferenceSource source = GnssVerticalReferenceSource::kRawRtk;
  const GnssSolutionSample *sample = nullptr;
  std::size_t sample_index = 0;
  double corrected_time_s = std::numeric_limits<double>::quiet_NaN();
  const Stage3VerticalReference *stage2_lowpass_reference = nullptr;
  const std::vector<RtkVerticalDriftReferenceDiagnosticRow> *rtk_vertical_drift_reference_profile =
    nullptr;
};

class GnssVerticalReferenceSelector {
 public:
  explicit GnssVerticalReferenceSelector(
    GnssVerticalReferenceSelectionRequest request);

  [[nodiscard]] GnssVerticalReferenceSelection Select() const;

 private:
  GnssVerticalReferenceSelectionRequest request_;
};

}  // namespace offline_lc_minimal
