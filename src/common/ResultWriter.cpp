#include "offline_lc_minimal/common/ResultWriter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace offline_lc_minimal {

namespace {

struct ScalarSeriesStats {
  double range = std::numeric_limits<double>::quiet_NaN();
  double mean = std::numeric_limits<double>::quiet_NaN();
  double stddev = std::numeric_limits<double>::quiet_NaN();
  double end_minus_start = std::numeric_limits<double>::quiet_NaN();
  double total_variation = std::numeric_limits<double>::quiet_NaN();
};

ScalarSeriesStats ComputeScalarSeriesStats(const std::vector<double> &values) {
  ScalarSeriesStats stats;
  std::vector<double> finite_values;
  finite_values.reserve(values.size());
  for (const double value : values) {
    if (std::isfinite(value)) {
      finite_values.push_back(value);
    }
  }
  if (finite_values.empty()) {
    return stats;
  }

  const auto [min_it, max_it] = std::minmax_element(finite_values.begin(), finite_values.end());
  stats.range = *max_it - *min_it;
  stats.mean = std::accumulate(finite_values.begin(), finite_values.end(), 0.0) / static_cast<double>(finite_values.size());
  if (finite_values.size() > 1U) {
    double variance = 0.0;
    double total_variation = 0.0;
    for (std::size_t index = 0; index < finite_values.size(); ++index) {
      const double centered = finite_values[index] - stats.mean;
      variance += centered * centered;
      if (index > 0U) {
        total_variation += std::abs(finite_values[index] - finite_values[index - 1U]);
      }
    }
    stats.stddev = std::sqrt(variance / static_cast<double>(finite_values.size()));
    stats.total_variation = total_variation;
  } else {
    stats.stddev = 0.0;
    stats.total_variation = 0.0;
  }
  stats.end_minus_start = finite_values.back() - finite_values.front();
  return stats;
}

void WriteTextFile(const std::filesystem::path &path, const std::string &content) {
  std::ofstream output_stream(path);
  if (!output_stream.is_open()) {
    throw std::runtime_error("failed to write file: " + path.string());
  }
  output_stream << content;
}

void WriteTrajectoryCsv(
  const std::filesystem::path &path,
  const std::vector<TrajectoryRow> &rows,
  const GeoReference &geo_reference) {
  std::ofstream trajectory_stream(path);
  if (!trajectory_stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  trajectory_stream << std::setprecision(17);
  trajectory_stream
    << "time_s,east_m,north_m,up_m,vx_mps,vy_mps,vz_mps,yaw_rad,pitch_rad,roll_rad,bax,bay,baz,bgx,bgy,bgz,"
       "gnss_factor_used,gnss_fix_type,gnss_residual_m,lat_rad,lon_rad,h_m\n";

  for (const auto &row : rows) {
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

void WriteReferenceNodeCsv(
  const std::filesystem::path &path,
  const std::vector<ReferenceNodeRow> &rows,
  const GeoReference &geo_reference) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "time_s,east_m,north_m,up_m,vx_mps,vy_mps,vz_mps,yaw_rad,pitch_rad,roll_rad,bax,bay,baz,bgx,bgy,bgz,"
       "lat_rad,lon_rad,h_m\n";

  for (const auto &row : rows) {
    const auto llh = geo_reference.Reverse(row.enu_position_m);
    stream << row.time_s << ','
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
           << llh[0] << ','
           << llh[1] << ','
           << llh[2] << '\n';
  }
}

void WriteErrorStateCsv(const std::filesystem::path &path, const std::vector<ErrorStateRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "time_s,dtheta_x_rad,dtheta_y_rad,dtheta_z_rad,dv_x_mps,dv_y_mps,dv_z_mps,dp_x_m,dp_y_m,dp_z_m,"
       "dbg_x_radps,dbg_y_radps,dbg_z_radps,dba_x_mps2,dba_y_mps2,dba_z_mps2\n";
  for (const auto &row : rows) {
    stream << row.time_s;
    for (Eigen::Index index = 0; index < row.state.size(); ++index) {
      stream << ',' << row.state[index];
    }
    stream << '\n';
  }
}

void WriteSegmentErrorCsv(const std::filesystem::path &path, const std::vector<SegmentErrorDiagnostic> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "segment_index,start_time_s,end_time_s,dtheta_x_rad,dtheta_y_rad,dtheta_z_rad,dv_x_mps,dv_y_mps,dv_z_mps,"
       "dp_x_m,dp_y_m,dp_z_m,dbg_x_radps,dbg_y_radps,dbg_z_radps,dba_x_mps2,dba_y_mps2,dba_z_mps2,"
       "gnss_factor_count,mean_prefit_nis,mean_postfit_nis,mean_covariance_scale,"
       "segment_vertical_rtk_residual_m,segment_vertical_gate_inside,segment_target_baz_mps2,"
       "segment_feedback_attitude_scale\n";
  for (const auto &row : rows) {
    stream << row.segment_index << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << row.dtheta_rad.x() << ','
           << row.dtheta_rad.y() << ','
           << row.dtheta_rad.z() << ','
           << row.dv_mps.x() << ','
           << row.dv_mps.y() << ','
           << row.dv_mps.z() << ','
           << row.dp_m.x() << ','
           << row.dp_m.y() << ','
           << row.dp_m.z() << ','
           << row.dbg_radps.x() << ','
           << row.dbg_radps.y() << ','
           << row.dbg_radps.z() << ','
           << row.dba_mps2.x() << ','
           << row.dba_mps2.y() << ','
           << row.dba_mps2.z() << ','
           << row.gnss_factor_count << ','
           << row.mean_prefit_nis << ','
           << row.mean_postfit_nis << ','
           << row.mean_covariance_scale << ','
           << row.segment_vertical_rtk_residual_m << ','
           << row.segment_vertical_gate_inside << ','
           << row.segment_target_baz_mps2 << ','
           << row.segment_feedback_attitude_scale << '\n';
  }
}

std::string BuildSegmentErrorSummaryText(const std::vector<SegmentErrorDiagnostic> &rows) {
  std::ostringstream stream;
  stream << std::setprecision(17);
  if (rows.empty()) {
    stream << "segment_error_count=0\n";
    return stream.str();
  }

  const std::vector<std::pair<std::string, std::function<double(const SegmentErrorDiagnostic &)>>> series = {
    {"dtheta_x_rad", [](const SegmentErrorDiagnostic &row) { return row.dtheta_rad.x(); }},
    {"dtheta_y_rad", [](const SegmentErrorDiagnostic &row) { return row.dtheta_rad.y(); }},
    {"dtheta_z_rad", [](const SegmentErrorDiagnostic &row) { return row.dtheta_rad.z(); }},
    {"dv_x_mps", [](const SegmentErrorDiagnostic &row) { return row.dv_mps.x(); }},
    {"dv_y_mps", [](const SegmentErrorDiagnostic &row) { return row.dv_mps.y(); }},
    {"dv_z_mps", [](const SegmentErrorDiagnostic &row) { return row.dv_mps.z(); }},
    {"dp_x_m", [](const SegmentErrorDiagnostic &row) { return row.dp_m.x(); }},
    {"dp_y_m", [](const SegmentErrorDiagnostic &row) { return row.dp_m.y(); }},
    {"dp_z_m", [](const SegmentErrorDiagnostic &row) { return row.dp_m.z(); }},
    {"dbg_x_radps", [](const SegmentErrorDiagnostic &row) { return row.dbg_radps.x(); }},
    {"dbg_y_radps", [](const SegmentErrorDiagnostic &row) { return row.dbg_radps.y(); }},
    {"dbg_z_radps", [](const SegmentErrorDiagnostic &row) { return row.dbg_radps.z(); }},
    {"dba_x_mps2", [](const SegmentErrorDiagnostic &row) { return row.dba_mps2.x(); }},
    {"dba_y_mps2", [](const SegmentErrorDiagnostic &row) { return row.dba_mps2.y(); }},
    {"dba_z_mps2", [](const SegmentErrorDiagnostic &row) { return row.dba_mps2.z(); }},
    {"segment_vertical_rtk_residual_m", [](const SegmentErrorDiagnostic &row) {
      return row.segment_vertical_rtk_residual_m;
    }},
    {"segment_vertical_gate_inside", [](const SegmentErrorDiagnostic &row) {
      return row.segment_vertical_gate_inside;
    }},
    {"segment_target_baz_mps2", [](const SegmentErrorDiagnostic &row) {
      return row.segment_target_baz_mps2;
    }},
    {"segment_feedback_attitude_scale", [](const SegmentErrorDiagnostic &row) {
      return row.segment_feedback_attitude_scale;
    }},
  };

  stream << "segment_error_count=" << rows.size() << '\n';
  for (const auto &[label, accessor] : series) {
    std::vector<double> values;
    values.reserve(rows.size());
    for (const auto &row : rows) {
      values.push_back(accessor(row));
    }
    const ScalarSeriesStats stats = ComputeScalarSeriesStats(values);
    stream << label << ".range=" << stats.range << '\n'
           << label << ".mean=" << stats.mean << '\n'
           << label << ".std=" << stats.stddev << '\n'
           << label << ".end_minus_start=" << stats.end_minus_start << '\n'
           << label << ".total_variation=" << stats.total_variation << '\n';
  }
  return stream.str();
}

void WriteGnssConsistencyCsv(
  const std::filesystem::path &path,
  const std::vector<GnssConsistencyRecord> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "sample_index,raw_time_s,corrected_time_s,factor_used,fix_type,sync_status,raw_sigma_h_m,"
       "sigma_e_m,sigma_n_m,sigma_u_m,effective_sigma_u_m,vertical_gate_threshold_m,vertical_gate_inside,"
       "vertical_sigma_u_used_m,vertical_direct_position_factor_used,"
       "vertical_feedback_target_baz_mps2,vertical_feedback_attitude_scale,"
       "vertical_reference_up_m,vertical_reference_used,"
       "covariance_scale,covariance_scale_e,covariance_scale_n,covariance_scale_u,"
       "prefit_residual_u_before_local_recovery_m,prefit_residual_u_after_local_recovery_m,"
       "prefit_residual_e_m,prefit_residual_n_m,prefit_residual_u_m,postfit_residual_e_m,"
       "postfit_residual_n_m,postfit_residual_u_m,prefit_nis,postfit_nis\n";
  for (const auto &row : rows) {
    stream << row.sample_index << ','
           << row.raw_time_s << ','
           << row.corrected_time_s << ','
           << (row.factor_used ? 1 : 0) << ','
           << ToString(row.gnss_fix_type) << ','
           << ToString(row.sync_status) << ','
           << row.raw_sigma_h_m << ','
           << row.sigma_e_m << ','
           << row.sigma_n_m << ','
           << row.sigma_u_m << ','
           << row.effective_sigma_u_m << ','
           << row.vertical_gate_threshold_m << ','
           << row.vertical_gate_inside << ','
           << row.vertical_sigma_u_used_m << ','
           << (row.vertical_direct_position_factor_used ? 1 : 0) << ','
           << row.vertical_feedback_target_baz_mps2 << ','
           << row.vertical_feedback_attitude_scale << ','
           << row.vertical_reference_up_m << ','
           << (row.vertical_reference_used ? 1 : 0) << ','
           << row.covariance_scale << ','
           << row.covariance_scale_e << ','
           << row.covariance_scale_n << ','
           << row.covariance_scale_u << ','
           << row.prefit_residual_u_before_local_recovery_m << ','
           << row.prefit_residual_u_after_local_recovery_m << ','
           << row.prefit_residual_enu_m.x() << ','
           << row.prefit_residual_enu_m.y() << ','
           << row.prefit_residual_enu_m.z() << ','
           << row.postfit_residual_enu_m.x() << ','
           << row.postfit_residual_enu_m.y() << ','
           << row.postfit_residual_enu_m.z() << ','
           << row.prefit_nis << ','
           << row.postfit_nis << '\n';
  }
}

void WriteVerticalStateCorrectionCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalStateCorrectionRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "sample_index,raw_time_s,corrected_time_s,sync_status,state_index,state_time_s,factor_used,"
       "reference_available,vertical_gate_inside,vertical_direct_position_factor_used,measurement_up_m,"
       "reference_up_m,optimized_up_m,delta_up_m,reference_vz_mps,optimized_vz_mps,delta_vz_mps,"
       "reference_pitch_rad,optimized_pitch_rad,delta_pitch_rad,reference_roll_rad,optimized_roll_rad,"
       "delta_roll_rad,reference_baz_mps2,optimized_baz_mps2,delta_baz_mps2,prefit_residual_u_m,"
       "postfit_residual_u_m\n";
  for (const auto &row : rows) {
    stream << row.sample_index << ','
           << row.raw_time_s << ','
           << row.corrected_time_s << ','
           << ToString(row.sync_status) << ','
           << row.state_index << ','
           << row.state_time_s << ','
           << (row.factor_used ? 1 : 0) << ','
           << (row.reference_available ? 1 : 0) << ','
           << row.vertical_gate_inside << ','
           << (row.vertical_direct_position_factor_used ? 1 : 0) << ','
           << row.measurement_up_m << ','
           << row.reference_up_m << ','
           << row.optimized_up_m << ','
           << row.delta_up_m << ','
           << row.reference_vz_mps << ','
           << row.optimized_vz_mps << ','
           << row.delta_vz_mps << ','
           << row.reference_pitch_rad << ','
           << row.optimized_pitch_rad << ','
           << row.delta_pitch_rad << ','
           << row.reference_roll_rad << ','
           << row.optimized_roll_rad << ','
           << row.delta_roll_rad << ','
           << row.reference_baz_mps2 << ','
           << row.optimized_baz_mps2 << ','
           << row.delta_baz_mps2 << ','
           << row.prefit_residual_u_m << ','
           << row.postfit_residual_u_m << '\n';
  }
}

void WriteInitialDynamicConsistencyCsv(
  const std::filesystem::path &path,
  const std::vector<TrajectoryRow> &rows,
  const RunSummary &run_summary) {
  if (rows.empty()) {
    return;
  }

  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "time_s,relative_time_s,up_m,vz_mps,yaw_rad,pitch_rad,roll_rad,baz_mps2,bgz_radps,initial_baz_mps2,"
       "initial_bgz_radps,static_baz_mps2,static_bgz_radps,optimized_last_static_baz_mps2,"
       "optimized_last_static_bgz_radps,optimized_first_dynamic_baz_mps2,optimized_first_dynamic_bgz_radps\n";

  const double start_time_s = rows.front().time_s;
  for (const auto &row : rows) {
    if (row.time_s > start_time_s + 30.0) {
      break;
    }
    stream << row.time_s << ','
           << (row.time_s - start_time_s) << ','
           << row.enu_position_m.z() << ','
           << row.enu_velocity_mps.z() << ','
           << row.ypr_rad.x() << ','
           << row.ypr_rad.y() << ','
           << row.ypr_rad.z() << ','
           << row.bias_acc.z() << ','
           << row.bias_gyro.z() << ','
           << run_summary.initial_baz_mps2 << ','
           << run_summary.initial_bgz_radps << ','
           << run_summary.static_baz_mps2 << ','
           << run_summary.static_bgz_radps << ','
           << run_summary.optimized_last_static_baz_mps2 << ','
           << run_summary.optimized_last_static_bgz_radps << ','
           << run_summary.optimized_first_dynamic_baz_mps2 << ','
           << run_summary.optimized_first_dynamic_bgz_radps << '\n';
  }
}

std::vector<TrajectoryRow> FilterDynamicTrajectoryRows(
  const std::vector<TrajectoryRow> &rows,
  const double dynamic_start_time_s) {
  if (!std::isfinite(dynamic_start_time_s)) {
    return rows;
  }

  auto it = std::find_if(
    rows.begin(),
    rows.end(),
    [dynamic_start_time_s](const TrajectoryRow &row) {
      return row.time_s + 1e-9 >= dynamic_start_time_s;
    });
  if (it == rows.end()) {
    return rows;
  }
  return std::vector<TrajectoryRow>(it, rows.end());
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

  WriteTrajectoryCsv(output_path / "trajectory.csv", result.trajectory, geo_reference);
  const std::vector<TrajectoryRow> dynamic_trajectory_rows =
    FilterDynamicTrajectoryRows(result.trajectory, result.run_summary.dynamic_start_time_s);
  WriteInitialDynamicConsistencyCsv(
    output_path / "initial_dynamic_consistency.csv",
    dynamic_trajectory_rows,
    result.run_summary);
  if (!result.initial_static_trajectory.empty()) {
    WriteTrajectoryCsv(output_path / "initial_static_trajectory.csv", result.initial_static_trajectory, geo_reference);
  }
  if (!result.optimized_static_terminal_forward_trajectory.empty()) {
    WriteTrajectoryCsv(
      output_path / "optimized_static_terminal_forward_trajectory.csv",
      result.optimized_static_terminal_forward_trajectory,
      geo_reference);
  }
  if (!result.reference_node_trajectory.empty()) {
    WriteReferenceNodeCsv(output_path / "reference_node_trajectory.csv", result.reference_node_trajectory, geo_reference);
  }
  if (!result.error_state_trajectory.empty()) {
    WriteErrorStateCsv(output_path / "error_state_trajectory.csv", result.error_state_trajectory);
  }
  if (!result.segment_error_diagnostics.empty()) {
    WriteSegmentErrorCsv(output_path / "segment_error_diagnostics.csv", result.segment_error_diagnostics);
    WriteTextFile(
      output_path / "segment_error_summary.txt",
      BuildSegmentErrorSummaryText(result.segment_error_diagnostics));
  }
  if (!result.gnss_consistency_records.empty()) {
    WriteGnssConsistencyCsv(output_path / "gnss_consistency.csv", result.gnss_consistency_records);
  }
  if (!result.vertical_state_corrections.empty()) {
    WriteVerticalStateCorrectionCsv(
      output_path / "vertical_state_corrections.csv",
      result.vertical_state_corrections);
  }

  if (!result.imu_rate_avp.empty()) {
    std::ofstream imu_rate_stream(output_path / "trajectory_imu_rate.csv");
    if (!imu_rate_stream.is_open()) {
      throw std::runtime_error("failed to write trajectory_imu_rate.csv");
    }
    imu_rate_stream << std::setprecision(17);
    imu_rate_stream
      << "time_s,east_m,north_m,up_m,vx_mps,vy_mps,vz_mps,yaw_rad,pitch_rad,roll_rad,bax,bay,baz,bgx,bgy,bgz,"
         "lat_rad,lon_rad,h_m\n";

    for (const auto &row : result.imu_rate_avp) {
      const auto llh = geo_reference.Reverse(row.enu_position_m);
      imu_rate_stream << row.time_s << ','
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

    std::ofstream alignment_stream(output_path / "gnss_alignment.csv");
    if (!alignment_stream.is_open()) {
      throw std::runtime_error("failed to write gnss_alignment.csv");
    }
    alignment_stream << std::setprecision(17);
    alignment_stream
      << "sample_index,raw_time_s,corrected_time_s,sync_status,factor_used,fix_type,trajectory_row_index_i,state_time_i_s,"
         "trajectory_row_index_j,state_time_j_s,synchronized_trajectory_row_index,graph_state_index_i,graph_state_index_j,"
         "graph_synchronized_state_index,duration_from_state_i_s,meas_east_m,meas_north_m,meas_up_m,residual_m\n";
    for (const auto &record : result.gnss_factor_records) {
      const long long graph_state_index_i = std::isfinite(record.state_time_i_s)
                                              ? static_cast<long long>(record.state_index_i)
                                              : -1;
      const long long graph_state_index_j = std::isfinite(record.state_time_j_s)
                                              ? static_cast<long long>(record.state_index_j)
                                              : -1;
      const long long graph_synchronized_state_index =
        record.factor_used &&
            (record.sync_status == StateMeasSyncStatus::kSynchronizedI ||
             record.sync_status == StateMeasSyncStatus::kSynchronizedJ)
          ? static_cast<long long>(record.synchronized_state_index)
          : -1;
      alignment_stream << record.sample_index << ','
                       << record.raw_time_s << ','
                       << record.corrected_time_s << ','
                       << ToString(record.sync_status) << ','
                       << (record.factor_used ? 1 : 0) << ','
                       << ToString(record.gnss_fix_type) << ','
                       << record.trajectory_row_index_i << ','
                       << record.state_time_i_s << ','
                       << record.trajectory_row_index_j << ','
                       << record.state_time_j_s << ','
                       << record.synchronized_trajectory_row_index << ','
                       << graph_state_index_i << ','
                       << graph_state_index_j << ','
                       << graph_synchronized_state_index << ','
                       << record.duration_from_state_i_s << ','
                       << record.measurement_enu_m.x() << ','
                       << record.measurement_enu_m.y() << ','
                       << record.measurement_enu_m.z() << ','
                       << record.residual_m << '\n';
    }

    if (!result.imu_rate_interval_diagnostics.empty()) {
      std::ofstream imu_rate_diag_stream(output_path / "trajectory_imu_rate_diagnostics.csv");
      if (!imu_rate_diag_stream.is_open()) {
        throw std::runtime_error("failed to write trajectory_imu_rate_diagnostics.csv");
      }
      imu_rate_diag_stream << std::setprecision(17);
      imu_rate_diag_stream
        << "interval_index,start_time_s,end_time_s,imu_sample_count,emitted_sample_count,used_interval,status\n";
      for (const auto &diagnostic : result.imu_rate_interval_diagnostics) {
        imu_rate_diag_stream << diagnostic.interval_index << ','
                             << diagnostic.start_time_s << ','
                             << diagnostic.end_time_s << ','
                             << diagnostic.imu_sample_count << ','
                             << diagnostic.emitted_sample_count << ','
                             << (diagnostic.used_interval ? 1 : 0) << ','
                             << diagnostic.status << '\n';
      }
    }
  }
}

}  // namespace offline_lc_minimal
