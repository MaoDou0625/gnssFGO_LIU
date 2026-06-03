#include "offline_lc_minimal/core/GnssPreOutageQualityOverride.h"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void ExpectTrue(const bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

offline_lc_minimal::GnssSolutionSample MakeSample(
  const double time_s,
  const int gnssfgo_type_code) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 0.1;
  sample.lon_rad = 0.2;
  sample.h_m = 3.0;
  sample.sigma_lat_m = 0.01;
  sample.sigma_lon_m = 0.01;
  sample.sigma_h_m = 0.02;
  sample.best_sol_status_code = 1;
  sample.best_pos_type_code = gnssfgo_type_code == 1 ? 7 : 6;
  sample.gnssfgo_type_code = gnssfgo_type_code;
  return sample;
}

void TestDisabledOverrideLeavesSamplesUnchanged() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_gnss_preoutage_quality_override = false;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(0.0, 1),
    MakeSample(1.0, 2),
    MakeSample(5.0, 1),
  };

  const auto summary =
    offline_lc_minimal::ApplyGnssPreOutageQualityOverride(config, samples);
  ExpectTrue(summary.planned_outage_count == 0, "disabled override should not plan outages");
  ExpectTrue(samples[0].gnssfgo_type_code == 1, "first sample should remain RTKFIX");
  ExpectTrue(samples[1].gnssfgo_type_code == 2, "middle sample should remain RTKFLOAT");
  ExpectTrue(samples[2].gnssfgo_type_code == 1, "last sample should remain RTKFIX");
}

void TestPreOutageFixesBecomeFloatAndOutageNonFixCanBecomeNoSolution() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_gnss_preoutage_quality_override = true;
  config.gnss_preoutage_quality_override_duration_s = 2.0;
  config.gnss_preoutage_quality_override_min_gap_s = 2.0;
  config.gnss_preoutage_quality_override_mark_outage_nonfix_no_solution = true;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(8.0, 1),
    MakeSample(9.0, 1),
    MakeSample(10.0, 1),
    MakeSample(11.0, 1),
    MakeSample(11.2, 2),
    MakeSample(11.4, 3),
    MakeSample(15.0, 1),
  };

  const auto summary =
    offline_lc_minimal::ApplyGnssPreOutageQualityOverride(config, samples);
  ExpectTrue(summary.planned_outage_count == 1, "one long RTK outage should be planned");
  ExpectTrue(summary.rtkfix_to_float_count == 2, "two pre-outage RTKFIX samples should become float");
  ExpectTrue(
    summary.nonfix_to_no_solution_count == 2,
    "two outage non-fix samples should become no-solution");
  ExpectTrue(samples[0].gnssfgo_type_code == 1, "outside pre-outage fix should remain RTKFIX");
  ExpectTrue(samples[1].gnssfgo_type_code == 1, "fix before pre-window should remain RTKFIX");
  ExpectTrue(samples[2].gnssfgo_type_code == 2, "pre-outage fix should become RTKFLOAT");
  ExpectTrue(samples[3].gnssfgo_type_code == 2, "last pre-outage fix should become RTKFLOAT");
  ExpectTrue(samples[4].gnssfgo_type_code == 4, "outage RTKFLOAT should become no-solution");
  ExpectTrue(samples[5].gnssfgo_type_code == 4, "outage single should become no-solution");
  ExpectTrue(samples[6].gnssfgo_type_code == 1, "recovery fix should remain RTKFIX");
}

void TestShortNonFixGapIsIgnored() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_gnss_preoutage_quality_override = true;
  config.gnss_preoutage_quality_override_duration_s = 2.0;
  config.gnss_preoutage_quality_override_min_gap_s = 2.0;
  config.gnss_preoutage_quality_override_mark_outage_nonfix_no_solution = true;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(0.0, 1),
    MakeSample(0.2, 1),
    MakeSample(0.4, 2),
    MakeSample(0.6, 1),
  };

  const auto summary =
    offline_lc_minimal::ApplyGnssPreOutageQualityOverride(config, samples);
  ExpectTrue(summary.planned_outage_count == 0, "short non-fix gap should not be treated as outage");
  ExpectTrue(samples[0].gnssfgo_type_code == 1, "first fix should remain RTKFIX");
  ExpectTrue(samples[1].gnssfgo_type_code == 1, "second fix should remain RTKFIX");
  ExpectTrue(samples[2].gnssfgo_type_code == 2, "short-gap float should remain RTKFLOAT");
  ExpectTrue(samples[3].gnssfgo_type_code == 1, "final fix should remain RTKFIX");
}

void TestTimestampGapWithoutIntermediateRowsStillMarksPreOutageFixes() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_gnss_preoutage_quality_override = true;
  config.gnss_preoutage_quality_override_duration_s = 2.0;
  config.gnss_preoutage_quality_override_min_gap_s = 2.0;
  config.gnss_preoutage_quality_override_mark_outage_nonfix_no_solution = true;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(7.0, 1),
    MakeSample(8.0, 1),
    MakeSample(9.0, 1),
    MakeSample(13.0, 1),
  };

  const auto summary =
    offline_lc_minimal::ApplyGnssPreOutageQualityOverride(config, samples);
  ExpectTrue(summary.planned_outage_count == 1, "pure timestamp gap should be planned");
  ExpectTrue(
    summary.rtkfix_to_float_count == 3,
    "pre-outage RTKFIX samples up to the gap boundary should become float");
  ExpectTrue(
    summary.nonfix_to_no_solution_count == 0,
    "pure timestamp gap has no non-fix samples to convert");
  ExpectTrue(samples[0].gnssfgo_type_code == 2, "pre-gap fix inside window should become float");
  ExpectTrue(samples[1].gnssfgo_type_code == 2, "second pre-gap fix should become float");
  ExpectTrue(samples[2].gnssfgo_type_code == 2, "last pre-gap fix should become float");
  ExpectTrue(samples[3].gnssfgo_type_code == 1, "post-gap recovery fix should remain RTKFIX");
}

void TestExistingNoSolutionRowsAreNotCountedAsConversions() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_gnss_preoutage_quality_override = true;
  config.gnss_preoutage_quality_override_duration_s = 2.0;
  config.gnss_preoutage_quality_override_min_gap_s = 2.0;
  config.gnss_preoutage_quality_override_mark_outage_nonfix_no_solution = true;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(0.0, 1),
    MakeSample(1.0, 1),
    MakeSample(1.2, 4),
    MakeSample(1.4, 2),
    MakeSample(4.0, 1),
  };

  const auto summary =
    offline_lc_minimal::ApplyGnssPreOutageQualityOverride(config, samples);
  ExpectTrue(summary.planned_outage_count == 1, "one outage should be planned");
  ExpectTrue(
    summary.nonfix_to_no_solution_count == 1,
    "only the RTKFLOAT row should count as a no-solution conversion");
  ExpectTrue(samples[2].gnssfgo_type_code == 4, "existing no-solution row should remain no-solution");
  ExpectTrue(samples[3].gnssfgo_type_code == 4, "float row should become no-solution");
}

}  // namespace

int main() {
  try {
    TestDisabledOverrideLeavesSamplesUnchanged();
    TestPreOutageFixesBecomeFloatAndOutageNonFixCanBecomeNoSolution();
    TestShortNonFixGapIsIgnored();
    TestTimestampGapWithoutIntermediateRowsStillMarksPreOutageFixes();
    TestExistingNoSolutionRowsAreNotCountedAsConversions();
  } catch (const std::exception &exception) {
    std::cerr << "gnss_preoutage_quality_override_test failed: "
              << exception.what() << '\n';
    return 1;
  }
  std::cout << "gnss_preoutage_quality_override_test passed\n";
  return 0;
}
