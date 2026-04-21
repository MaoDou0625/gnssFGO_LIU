#include <algorithm>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/GeoUtils.h"
#include "offline_lc_minimal/common/ResultWriter.h"
#include "offline_lc_minimal/core/OfflineBatchRunner.h"
#include "offline_lc_minimal/io/TextDatasetLoader.h"

namespace offline_lc_minimal {

namespace {

void PrintUsage() {
  std::cout
    << "offline_lc_runner --config <file> [--imu <path>] [--gnss <path>] [--output-dir <dir>] [--imu-only] [--verbose]\n";
}

}  // namespace

}  // namespace offline_lc_minimal

int main(int argc, char **argv) {
  using namespace offline_lc_minimal;

  try {
    std::string config_path;
    std::string imu_override;
    std::string gnss_override;
    std::string output_dir_override;
    bool imu_only = false;
    bool verbose = false;

    for (int index = 1; index < argc; ++index) {
      const std::string arg = argv[index];
      if (arg == "--help" || arg == "-h") {
        PrintUsage();
        return 0;
      }
      if ((arg == "--config" || arg == "-c") && index + 1 < argc) {
        config_path = argv[++index];
      } else if (arg == "--imu" && index + 1 < argc) {
        imu_override = argv[++index];
      } else if (arg == "--gnss" && index + 1 < argc) {
        gnss_override = argv[++index];
      } else if (arg == "--output-dir" && index + 1 < argc) {
        output_dir_override = argv[++index];
      } else if (arg == "--imu-only") {
        imu_only = true;
      } else if (arg == "--verbose") {
        verbose = true;
      } else {
        throw std::runtime_error("unknown or incomplete argument: " + arg);
      }
    }

    OfflineRunnerConfig config = DefaultConfig();
    if (!config_path.empty()) {
      config = LoadConfigFile(config_path, config);
    }
    if (!imu_override.empty()) {
      config.imu_path = imu_override;
    }
    if (!gnss_override.empty()) {
      config.gnss_path = gnss_override;
    }
    if (!output_dir_override.empty()) {
      config.output_dir = output_dir_override;
    }
    if (imu_only) {
      config.enable_gnss = false;
    }
    if (verbose) {
      config.verbose = true;
    }

    if (config.state_meas_sync_lower_bound_s > config.state_meas_sync_upper_bound_s) {
      throw std::runtime_error("state_meas_sync_lower_bound_s must be <= state_meas_sync_upper_bound_s");
    }
    if (config.state_meas_sync_lower_bound_s > 0.0 || config.state_meas_sync_upper_bound_s < 0.0) {
      throw std::runtime_error("state_meas_sync bounds must satisfy lower <= 0 <= upper");
    }
    if (config.state_frequency_hz <= 0.0) {
      throw std::runtime_error("state_frequency_hz must be positive");
    }
    if (config.gnss_position_robust_param <= 0.0) {
      throw std::runtime_error("gnss_position_robust_param must be positive");
    }
    if (config.stationary_window_s <= 0.0) {
      throw std::runtime_error("stationary_window_s must be positive");
    }
    if (config.imu_dual_vector_window_s <= 0.0) {
      throw std::runtime_error("imu_dual_vector_window_s must be positive");
    }
    if (config.imu_dual_vector_min_sample_count <= 0) {
      throw std::runtime_error("imu_dual_vector_min_sample_count must be positive");
    }
    if (config.imu_dual_vector_min_cross_norm <= 0.0) {
      throw std::runtime_error("imu_dual_vector_min_cross_norm must be positive");
    }

    if (config.imu_path.empty() || config.gnss_path.empty()) {
      throw std::runtime_error("both imu_path and gnss_path must be provided");
    }

    DataSet dataset = TextDatasetLoader::Load(config);
    const OfflineBatchRunner runner(config);
    const OfflineRunResult result = runner.Run(std::move(dataset));

    const GeoReference geo_reference(
      result.run_summary.origin_lat_rad,
      result.run_summary.origin_lon_rad,
      result.run_summary.origin_h_m);
    ResultWriter::WriteOutputs(config.output_dir, config, result, geo_reference);

    std::cout << "offline_lc_runner completed.\n"
              << "output_dir=" << std::filesystem::absolute(config.output_dir).string() << '\n'
              << result.run_summary.ToMultilineString()
              << result.data_summary.ToMultilineString();
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "offline_lc_runner failed: " << exception.what() << '\n';
    return 1;
  }
}
