#include "offline_lc_minimal/common/ResultWriter.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace offline_lc_minimal {

namespace {

void WriteTextFile(const std::filesystem::path &path, const std::string &content) {
  std::ofstream output_stream(path);
  if (!output_stream.is_open()) {
    throw std::runtime_error("failed to write file: " + path.string());
  }
  output_stream << content;
}

}  // namespace

void ResultWriter::WriteOutputs(
  const std::string &output_dir,
  const OfflineRunnerConfig &config,
  const OfflineRunResult &result,
  const GeoReference &geo_reference) {
  const std::filesystem::path output_path(output_dir);
  std::filesystem::create_directories(output_path);

  WriteTextFile(output_path / "summary.txt",
                result.run_summary.ToMultilineString() + "\n" + result.data_summary.ToMultilineString());
  WriteTextFile(output_path / "data_summary.txt", result.data_summary.ToMultilineString());
  WriteTextFile(output_path / "config_snapshot.cfg", ConfigToString(config));

  {
    std::ofstream trajectory_stream(output_path / "trajectory.csv");
    if (!trajectory_stream.is_open()) {
      throw std::runtime_error("failed to write trajectory.csv");
    }
    trajectory_stream << std::setprecision(17);
    trajectory_stream
      << "time_s,east_m,north_m,up_m,vx_mps,vy_mps,vz_mps,yaw_rad,pitch_rad,roll_rad,bax,bay,baz,bgx,bgy,bgz,"
         "gnss_factor_used,gnss_fix_type,gnss_residual_m,lat_rad,lon_rad,h_m\n";

    for (const auto &row : result.trajectory) {
      const auto llh = geo_reference.Reverse(row.enu_position_m);
      trajectory_stream << row.time_s << ','
                        << row.enu_position_m.x() << ','
                        << row.enu_position_m.y() << ','
                        << row.enu_position_m.z() << ','
                        << row.enu_velocity_mps.x() << ','
                        << row.enu_velocity_mps.y() << ','
                        << row.enu_velocity_mps.z() << ','
                        << row.ypr_rad.x() << ','
                        << row.ypr_rad.y() << ','
                        << row.ypr_rad.z() << ','
                        << row.bias_acc.x() << ','
                        << row.bias_acc.y() << ','
                        << row.bias_acc.z() << ','
                        << row.bias_gyro.x() << ','
                        << row.bias_gyro.y() << ','
                        << row.bias_gyro.z() << ','
                        << (row.gnss_factor_used ? 1 : 0) << ','
                        << ToString(row.gnss_fix_type) << ','
                        << row.gnss_residual_m << ','
                        << llh[0] << ','
                        << llh[1] << ','
                        << llh[2] << '\n';
    }
  }

  if (config.write_debug_csv) {
    std::ofstream residual_stream(output_path / "gnss_residuals.csv");
    if (!residual_stream.is_open()) {
      throw std::runtime_error("failed to write gnss_residuals.csv");
    }
    residual_stream << std::setprecision(17);
    residual_stream << "time_s,gnss_factor_used,gnss_fix_type,gnss_residual_m\n";
    for (const auto &row : result.trajectory) {
      residual_stream << row.time_s << ','
                      << (row.gnss_factor_used ? 1 : 0) << ','
                      << ToString(row.gnss_fix_type) << ','
                      << row.gnss_residual_m << '\n';
    }
  }
}

}  // namespace offline_lc_minimal
