#include "offline_lc_minimal/io/TextDatasetLoader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "offline_lc_minimal/core/GnssPreOutageQualityOverride.h"

namespace offline_lc_minimal {

namespace {

std::string Trim(std::string value) {
  auto not_space = [](const unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

bool LooksLikeComment(const std::string &line) {
  return line.empty() || line.front() == '#';
}

std::vector<std::string> Tokenize(const std::string &line) {
  std::vector<std::string> tokens;
  std::istringstream line_stream(line);
  std::string token;
  while (line_stream >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

bool IsLikelyNumeric(const std::string &value) {
  if (value.empty()) {
    return false;
  }
  const char first = value.front();
  return std::isdigit(static_cast<unsigned char>(first)) || first == '-' || first == '+' || first == '.';
}

std::vector<ImuSample> LoadImuSamples(const std::string &path) {
  std::ifstream input_stream(path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("failed to open IMU file: " + path);
  }

  std::vector<ImuSample> imu_samples;
  std::string line;
  while (std::getline(input_stream, line)) {
    line = Trim(std::move(line));
    if (LooksLikeComment(line)) {
      continue;
    }
    const auto tokens = Tokenize(line);
    if (tokens.size() != 7 || !IsLikelyNumeric(tokens.front())) {
      continue;
    }

    ImuSample sample;
    sample.time_s = std::stod(tokens[0]);
    sample.gyro_radps = Eigen::Vector3d(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
    sample.accel_mps2 = Eigen::Vector3d(std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]));
    imu_samples.push_back(sample);
  }

  if (imu_samples.empty()) {
    throw std::runtime_error("no IMU samples parsed from: " + path);
  }

  std::sort(imu_samples.begin(), imu_samples.end(), [](const auto &lhs, const auto &rhs) {
    return lhs.time_s < rhs.time_s;
  });
  return imu_samples;
}

std::vector<GnssSolutionSample> LoadGnssSamples(const std::string &path) {
  std::ifstream input_stream(path);
  if (!input_stream.is_open()) {
    throw std::runtime_error("failed to open GNSS solution file: " + path);
  }

  std::vector<GnssSolutionSample> gnss_samples;
  std::string line;
  while (std::getline(input_stream, line)) {
    line = Trim(std::move(line));
    if (LooksLikeComment(line)) {
      continue;
    }
    const auto tokens = Tokenize(line);
    if (tokens.size() < 18 || !IsLikelyNumeric(tokens.front())) {
      continue;
    }

    GnssSolutionSample sample;
    sample.time_s = std::stod(tokens[0]);
    sample.lat_rad = std::stod(tokens[1]);
    sample.lon_rad = std::stod(tokens[2]);
    sample.h_m = std::stod(tokens[3]);
    sample.sigma_lat_m = std::stod(tokens[4]);
    sample.sigma_lon_m = std::stod(tokens[5]);
    sample.sigma_h_m = std::stod(tokens[6]);
    sample.diff_age_s = std::stod(tokens[7]);
    sample.sol_age_s = std::stod(tokens[8]);
    sample.num_svs = static_cast<int>(std::lround(std::stod(tokens[9])));
    sample.best_sol_status_code = static_cast<int>(std::lround(std::stod(tokens[10])));
    sample.best_pos_type_code = static_cast<int>(std::lround(std::stod(tokens[11])));
    sample.gnssfgo_type_code = static_cast<int>(std::lround(std::stod(tokens[12])));
    sample.dop_pdop = std::stod(tokens[13]);
    sample.dop_hdop = std::stod(tokens[14]);
    sample.dop_vdop = std::stod(tokens[15]);
    sample.pos_source_code = static_cast<int>(std::lround(std::stod(tokens[16])));
    sample.hpd_status_code = static_cast<int>(std::lround(std::stod(tokens[17])));
    gnss_samples.push_back(sample);
  }

  if (gnss_samples.empty()) {
    throw std::runtime_error("no GNSS samples parsed from: " + path);
  }

  std::sort(gnss_samples.begin(), gnss_samples.end(), [](const auto &lhs, const auto &rhs) {
    return lhs.time_s < rhs.time_s;
  });
  return gnss_samples;
}

template <typename SampleT>
void FillTimingStats(
  const std::vector<SampleT> &samples,
  double &start_s,
  double &end_s,
  double &mean_dt_s,
  double &min_dt_s,
  double &max_dt_s) {
  start_s = samples.front().time_s;
  end_s = samples.back().time_s;
  if (samples.size() < 2) {
    mean_dt_s = 0.0;
    min_dt_s = 0.0;
    max_dt_s = 0.0;
    return;
  }

  double sum_dt = 0.0;
  min_dt_s = std::numeric_limits<double>::infinity();
  max_dt_s = 0.0;

  for (std::size_t index = 1; index < samples.size(); ++index) {
    const double dt = samples[index].time_s - samples[index - 1].time_s;
    sum_dt += dt;
    min_dt_s = std::min(min_dt_s, dt);
    max_dt_s = std::max(max_dt_s, dt);
  }
  mean_dt_s = sum_dt / static_cast<double>(samples.size() - 1);
}

DataSummary BuildSummary(const std::vector<ImuSample> &imu_samples, const std::vector<GnssSolutionSample> &gnss_samples) {
  DataSummary summary;
  summary.imu_count = imu_samples.size();
  summary.gnss_count = gnss_samples.size();

  FillTimingStats(
    imu_samples,
    summary.imu_start_s,
    summary.imu_end_s,
    summary.imu_mean_dt_s,
    summary.imu_min_dt_s,
    summary.imu_max_dt_s);

  FillTimingStats(
    gnss_samples,
    summary.gnss_start_s,
    summary.gnss_end_s,
    summary.gnss_mean_dt_s,
    summary.gnss_min_dt_s,
    summary.gnss_max_dt_s);

  for (const auto &sample : gnss_samples) {
    if (sample.has_valid_position()) {
      ++summary.gnss_valid_position_count;
    }
    if (sample.has_finite_sigma()) {
      ++summary.gnss_finite_sigma_count;
    }
    if (sample.fix_type() == GnssFixType::kNoSolution) {
      ++summary.gnss_no_solution_count;
    }
  }

  return summary;
}

}  // namespace

DataSet TextDatasetLoader::Load(const OfflineRunnerConfig &config) {
  DataSet dataset;
  dataset.imu_samples = LoadImuSamples(config.imu_path);
  dataset.gnss_samples = LoadGnssSamples(config.gnss_path);
  ApplyGnssPreOutageQualityOverride(config, dataset.gnss_samples);
  dataset.summary = BuildSummary(dataset.imu_samples, dataset.gnss_samples);
  return dataset;
}

}  // namespace offline_lc_minimal
