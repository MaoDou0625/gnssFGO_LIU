#include <exception>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/GeoUtils.h"
#include "offline_lc_minimal/common/ResultWriter.h"
#include "offline_lc_minimal/core/OfflineBatchRunner.h"
#include "offline_lc_minimal/core/SharedVerticalReferenceBuilder.h"
#include "offline_lc_minimal/core/Stage3HeightOptimizationPolicy.h"
#include "offline_lc_minimal/core/Stage3SharedReferenceMapper.h"
#include "offline_lc_minimal/io/BodyZBiasReestimateCsvReader.h"
#include "offline_lc_minimal/io/TextDatasetLoader.h"
#include "offline_lc_minimal/io/TrajectoryCsvReader.h"

namespace offline_lc_minimal {
namespace {

struct Stage3RunnerArguments {
  std::string config_path;
  std::string imu_override;
  std::string gnss_override;
  std::string output_dir_override;
  std::filesystem::path stage2_trajectory_path;
  std::filesystem::path shared_reference_path;
  std::filesystem::path shared_reference_line_path;
  std::vector<std::string> config_overrides;
  bool verbose = false;
};

void PrintUsage() {
  std::cout
    << "offline_lc_stage3_runner --config <file> "
       "--stage2-trajectory <trajectory.csv> "
       "--shared-reference <shared_vertical_reference.csv> "
       "--shared-reference-line <shared_reference_line.csv> "
       "--output-dir <dir> [--imu <path>] [--gnss <path>] "
       "[--set key=value] [--verbose]\n";
}

Stage3RunnerArguments ParseArguments(const int argc, char **argv) {
  Stage3RunnerArguments args;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--help" || arg == "-h") {
      PrintUsage();
      std::exit(0);
    }
    if ((arg == "--config" || arg == "-c") && index + 1 < argc) {
      args.config_path = argv[++index];
    } else if (arg == "--imu" && index + 1 < argc) {
      args.imu_override = argv[++index];
    } else if (arg == "--gnss" && index + 1 < argc) {
      args.gnss_override = argv[++index];
    } else if (arg == "--output-dir" && index + 1 < argc) {
      args.output_dir_override = argv[++index];
    } else if (arg == "--stage2-trajectory" && index + 1 < argc) {
      args.stage2_trajectory_path = argv[++index];
    } else if (arg == "--shared-reference" && index + 1 < argc) {
      args.shared_reference_path = argv[++index];
    } else if (arg == "--shared-reference-line" && index + 1 < argc) {
      args.shared_reference_line_path = argv[++index];
    } else if (arg == "--set" && index + 1 < argc) {
      args.config_overrides.push_back(argv[++index]);
    } else if (arg == "--verbose") {
      args.verbose = true;
    } else {
      throw std::runtime_error("unknown or incomplete argument: " + arg);
    }
  }

  if (args.config_path.empty() ||
      args.stage2_trajectory_path.empty() ||
      args.shared_reference_path.empty() ||
      args.shared_reference_line_path.empty() ||
      args.output_dir_override.empty()) {
    PrintUsage();
    throw std::runtime_error(
      "--config, --stage2-trajectory, --shared-reference, "
      "--shared-reference-line, and --output-dir are required");
  }
  return args;
}

void ApplyOverrides(
  const Stage3RunnerArguments &args,
  OfflineRunnerConfig &config) {
  if (!args.imu_override.empty()) {
    config.imu_path = args.imu_override;
  }
  if (!args.gnss_override.empty()) {
    config.gnss_path = args.gnss_override;
  }
  config.output_dir = args.output_dir_override;
  for (const auto &override_entry : args.config_overrides) {
    const std::size_t delimiter_pos = override_entry.find('=');
    if (delimiter_pos == std::string::npos) {
      throw std::runtime_error("--set expects key=value, got: " + override_entry);
    }
    OverrideConfigField(
      config,
      override_entry.substr(0, delimiter_pos),
      override_entry.substr(delimiter_pos + 1U));
  }
  if (args.verbose) {
    config.verbose = true;
  }
}

std::shared_ptr<Stage2VelocityReference> LoadStage2Reference(
  const std::filesystem::path &path,
  const OfflineRunnerConfig &config) {
  auto reference =
    std::make_shared<Stage2VelocityReference>(
      ReadStage2VelocityReferenceCsv(
        path,
        std::make_shared<OfflineRunnerConfig>(config)));
  if (reference->trajectory.empty() && reference->reference_states.empty()) {
    throw std::runtime_error("Stage2 trajectory did not produce any reference states");
  }
  const std::filesystem::path stage2_dir = path.parent_path();
  const std::filesystem::path bias_reestimate_path =
    stage2_dir / "body_z_bias_reestimate_segments.csv";
  if (std::filesystem::exists(bias_reestimate_path)) {
    reference->body_z_bias_reestimate_segments =
      ReadBodyZBiasReestimateSegmentCsv(bias_reestimate_path);
  }
  return reference;
}

std::shared_ptr<Stage3VerticalReference> BuildSharedStage3Reference(
  const Stage3RunnerArguments &args,
  const OfflineRunnerConfig &config,
  const std::vector<TrajectoryCsvRow> &stage2_rows) {
  const std::vector<SharedVerticalReferenceRow> shared_reference =
    ReadSharedVerticalReferenceCsv(args.shared_reference_path);
  const std::vector<SharedReferenceLinePoint> shared_reference_line =
    ReadSharedReferenceLineCsv(args.shared_reference_line_path);

  Stage3SharedReferenceMapRequest map_request;
  map_request.config = &config;
  map_request.stage2_trajectory = &stage2_rows;
  map_request.shared_reference = &shared_reference;
  map_request.shared_reference_line = &shared_reference_line;

  return std::make_shared<Stage3VerticalReference>(
    BuildStage3ReferenceFromSharedVerticalReference(map_request));
}

}  // namespace
}  // namespace offline_lc_minimal

int main(int argc, char **argv) {
  using namespace offline_lc_minimal;

  OfflineRunnerConfig config = DefaultConfig();
  try {
    const Stage3RunnerArguments args = ParseArguments(argc, argv);
    config = LoadConfigFile(args.config_path, config);
    ApplyOverrides(args, config);

    OfflineRunnerConfig stage3_config =
      MakeStage3HeightOptimizationConfig(config);
    ValidateConfig(stage3_config);
    if (stage3_config.imu_path.empty() || stage3_config.gnss_path.empty()) {
      throw std::runtime_error("both imu_path and gnss_path must be provided");
    }
    config = stage3_config;

    std::vector<TrajectoryCsvRow> stage2_rows =
      ReadTrajectoryCsvRows(args.stage2_trajectory_path);
    auto stage2_reference =
      LoadStage2Reference(args.stage2_trajectory_path, stage3_config);
    auto stage3_reference =
      BuildSharedStage3Reference(args, stage3_config, stage2_rows);

    DataSet dataset = TextDatasetLoader::Load(stage3_config);
    const OfflineBatchRunner runner(
      stage3_config,
      std::move(stage2_reference),
      nullptr,
      std::move(stage3_reference));
    const OfflineRunResult result = runner.Run(std::move(dataset));

    const GeoReference geo_reference(
      result.run_summary.origin_lat_rad,
      result.run_summary.origin_lon_rad,
      result.run_summary.origin_h_m);
    ResultWriter::WriteOutputs(
      stage3_config.output_dir,
      stage3_config,
      result,
      geo_reference);

    std::cout << "offline_lc_stage3_runner completed.\n"
              << "output_dir="
              << std::filesystem::absolute(stage3_config.output_dir).string()
              << '\n'
              << "stage2_trajectory="
              << std::filesystem::absolute(args.stage2_trajectory_path).string()
              << '\n'
              << "shared_reference="
              << std::filesystem::absolute(args.shared_reference_path).string()
              << '\n'
              << result.run_summary.ToMultilineString()
              << result.data_summary.ToMultilineString();
    return 0;
  } catch (const OfflineRunFailure &failure) {
    try {
      const auto &partial_result = failure.partial_result();
      const GeoReference geo_reference(
        partial_result.run_summary.origin_lat_rad,
        partial_result.run_summary.origin_lon_rad,
        partial_result.run_summary.origin_h_m);
      ResultWriter::WriteOutputs(
        config.output_dir,
        config,
        partial_result,
        geo_reference);
    } catch (const std::exception &write_exception) {
      std::cerr << "failed to write partial outputs: "
                << write_exception.what() << '\n';
    }
    std::cerr << "offline_lc_stage3_runner failed: "
              << failure.what() << '\n';
    return 1;
  } catch (const std::exception &exception) {
    std::cerr << "offline_lc_stage3_runner failed: "
              << exception.what() << '\n';
    return 1;
  }
}
