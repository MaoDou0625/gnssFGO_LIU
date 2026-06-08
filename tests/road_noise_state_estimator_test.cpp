#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "offline_lc_minimal/core/RoadNoiseBiasReestimatePlanner.h"
#include "offline_lc_minimal/core/RoadNoiseStateEstimator.h"

namespace {

template <typename Function>
void RunTest(const std::string &name, Function &&function) {
  try {
    function();
  } catch (const std::exception &exception) {
    throw std::runtime_error(name + ": " + exception.what());
  }
}

void ExpectTrue(const bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void ExpectNear(
  const double actual,
  const double expected,
  const double tolerance,
  const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(message);
  }
}

offline_lc_minimal::BodyZSeedJumpWindowRow MakeJumpWindow(
  const double start_time_s,
  const double end_time_s) {
  offline_lc_minimal::BodyZSeedJumpWindowRow row;
  row.start_time_s = start_time_s;
  row.end_time_s = end_time_s;
  row.duration_s = end_time_s - start_time_s;
  return row;
}

std::vector<offline_lc_minimal::BodyZJumpSignalSample> MakeLowHighLowSignal() {
  std::vector<offline_lc_minimal::BodyZJumpSignalSample> signal;
  for (int index = 0; index <= 320; ++index) {
    const double time_s = static_cast<double>(index) * 0.1;
    const bool high_noise = time_s >= 10.0 && time_s <= 22.0;
    const double amplitude = high_noise ? 0.12 : 0.015;
    const double sign = index % 2 == 0 ? 1.0 : -1.0;

    offline_lc_minimal::BodyZJumpSignalSample sample;
    sample.time_s = time_s;
    sample.relative_time_s = time_s;
    sample.body_z_acc_1s_smooth_mps2 = 0.0;
    sample.body_z_acc_mps2 = amplitude * sign;
    if (time_s >= 5.0 && time_s <= 6.0) {
      sample.body_z_acc_mps2 = 1.5 * sign;
    }
    signal.push_back(sample);
  }
  return signal;
}

void TestRoadNoiseStateEstimatorInfersTwoStatesWithoutForcingSegmentCount() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.road_noise_state_window_s = 2.0;
  config.road_noise_state_stride_s = 1.0;
  config.road_noise_state_min_sample_count = 5;
  config.road_noise_state_min_segment_s = 2.0;
  config.road_noise_state_hysteresis_ratio = 0.05;
  config.vertical_velocity_delta_jump_padding_s = 0.4;

  const std::vector<offline_lc_minimal::BodyZJumpSignalSample> signal =
    MakeLowHighLowSignal();
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jumps{
    MakeJumpWindow(5.0, 6.0),
  };

  offline_lc_minimal::RoadNoiseStateEstimatorRequest request;
  request.config = &config;
  request.signal = &signal;
  request.jump_windows = &jumps;
  const std::vector<offline_lc_minimal::RoadNoiseStateSegmentRow> segments =
    offline_lc_minimal::RoadNoiseStateEstimator(std::move(request)).Estimate();

  ExpectNear(static_cast<double>(segments.size()), 3.0, 0.0, "road state segment count is wrong");
  ExpectTrue(segments[0].state == "LOW_NOISE", "first road state should be low noise");
  ExpectTrue(segments[1].state == "HIGH_NOISE", "middle road state should be high noise");
  ExpectTrue(segments[2].state == "LOW_NOISE", "last road state should be low noise");
  ExpectTrue(segments[1].start_time_s > 8.0 && segments[1].start_time_s < 12.0,
             "high-noise segment should start near the actual road transition");
  ExpectTrue(segments[1].end_time_s > 20.0 && segments[1].end_time_s < 24.0,
             "high-noise segment should end near the actual road transition");
}

void TestRoadNoiseBiasReestimatePlannerUsesOnlyHighNoiseSegments() {
  std::vector<offline_lc_minimal::RoadNoiseStateSegmentRow> road_segments(3U);
  road_segments[0].segment_index = 0U;
  road_segments[0].state = "LOW_NOISE";
  road_segments[0].start_time_s = 0.0;
  road_segments[0].end_time_s = 10.0;
  road_segments[1].segment_index = 1U;
  road_segments[1].state = "HIGH_NOISE";
  road_segments[1].start_time_s = 10.0;
  road_segments[1].end_time_s = 22.0;
  road_segments[2].segment_index = 2U;
  road_segments[2].state = "LOW_NOISE";
  road_segments[2].start_time_s = 22.0;
  road_segments[2].end_time_s = 32.0;

  const std::vector<offline_lc_minimal::BodyZBiasReestimateSegmentRow> segments =
    offline_lc_minimal::PlanRoadNoiseBiasReestimateSegments(
      road_segments,
      offline_lc_minimal::RoadNoiseBiasReestimatePlannerOptions{2.0});

  ExpectNear(static_cast<double>(segments.size()), 1.0, 0.0, "only high-noise road should create a ba_z segment");
  ExpectTrue(segments.front().source_type == "ROAD_HIGH_NOISE", "source type is wrong");
  ExpectNear(segments.front().start_time_s, 10.0, 1e-12, "segment start is wrong");
  ExpectNear(segments.front().end_time_s, 22.0, 1e-12, "segment end is wrong");
  ExpectNear(segments.front().detected_bias_delta_mps2, 0.0, 0.0, "road-state planner should not inject body-z delta");
}

}  // namespace

int main() {
  RunTest(
    "TestRoadNoiseStateEstimatorInfersTwoStatesWithoutForcingSegmentCount",
    TestRoadNoiseStateEstimatorInfersTwoStatesWithoutForcingSegmentCount);
  RunTest(
    "TestRoadNoiseBiasReestimatePlannerUsesOnlyHighNoiseSegments",
    TestRoadNoiseBiasReestimatePlannerUsesOnlyHighNoiseSegments);
  return 0;
}
