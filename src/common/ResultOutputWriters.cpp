#include "offline_lc_minimal/common/ResultOutputWriters.h"

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

#include "offline_lc_minimal/common/Units.h"

namespace offline_lc_minimal {

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

void WriteSeedBodyZAccDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZSeedImuDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "time_s,relative_time_s,body_z_specific_force_mps2,gravity_projection_z_mps2,body_z_acc_mps2,"
       "body_z_acc_1s_smooth_mps2,integrated_body_z_velocity_mps,"
       "integrated_body_z_velocity_0p2s_smooth_mps,integrated_body_z_velocity_1s_smooth_mps,"
       "signed_step_metric_mps,downward_score_mps,upward_score_mps,body_z_axis_nav_z\n";
  for (const auto &row : rows) {
    stream << row.time_s << ','
           << row.relative_time_s << ','
           << row.body_z_specific_force_mps2 << ','
           << row.gravity_projection_z_mps2 << ','
           << row.body_z_acc_mps2 << ','
           << row.body_z_acc_1s_smooth_mps2 << ','
           << row.integrated_body_z_velocity_mps << ','
           << row.integrated_body_z_velocity_0p2s_smooth_mps << ','
           << row.integrated_body_z_velocity_1s_smooth_mps << ','
           << row.signed_step_metric_mps << ','
           << row.downward_score_mps << ','
           << row.upward_score_mps << ','
           << row.body_z_axis_nav_z << '\n';
  }
}

void WriteBodyZSeedJumpWindowCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZSeedJumpWindowRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "direction,selection_level,start_state_index,center_state_index,end_state_index,"
       "start_time_s,center_time_s,end_time_s,start_relative_time_s,center_relative_time_s,end_relative_time_s,"
       "duration_s,pre_velocity_mps,post_velocity_mps,signed_delta_velocity_mps,direction_score_mps,"
       "signed_step_metric_mps,level_threshold_mps,level_max_peak_mps,level_noise_floor_mps,"
       "min_acc_mps2,max_acc_mps2,mean_acc_mps2,body_z_axis_nav_z,delta_vz_init_mps\n";
  for (const auto &row : rows) {
    stream << row.direction << ','
           << row.selection_level << ','
           << row.start_state_index << ','
           << row.center_state_index << ','
           << row.end_state_index << ','
           << row.start_time_s << ','
           << row.center_time_s << ','
           << row.end_time_s << ','
           << row.start_relative_time_s << ','
           << row.center_relative_time_s << ','
           << row.end_relative_time_s << ','
           << row.duration_s << ','
           << row.pre_velocity_mps << ','
           << row.post_velocity_mps << ','
           << row.signed_delta_velocity_mps << ','
           << row.direction_score_mps << ','
           << row.signed_step_metric_mps << ','
           << row.level_threshold_mps << ','
           << row.level_max_peak_mps << ','
           << row.level_noise_floor_mps << ','
           << row.min_acc_mps2 << ','
           << row.max_acc_mps2 << ','
           << row.mean_acc_mps2 << ','
           << row.body_z_axis_nav_z << ','
           << row.delta_vz_init_mps << '\n';
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
       "gnss_factor_count,mean_postfit_nis\n";
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
           << row.mean_postfit_nis << '\n';
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
       "sigma_e_m,sigma_n_m,sigma_u_m,effective_sigma_u_m,vertical_sigma_u_used_m,"
       "vertical_direct_position_factor_used,postfit_residual_e_m,postfit_residual_n_m,"
       "postfit_residual_u_m,postfit_nis\n";
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
           << row.vertical_sigma_u_used_m << ','
           << (row.vertical_direct_position_factor_used ? 1 : 0) << ','
           << row.postfit_residual_enu_m.x() << ','
           << row.postfit_residual_enu_m.y() << ','
           << row.postfit_residual_enu_m.z() << ','
           << row.postfit_nis << '\n';
  }
}

void WriteVerticalEnvelopeDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalEnvelopeDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "sample_index,raw_time_s,corrected_time_s,sync_status,state_index_i,state_index_j,"
       "synchronized_state_index,state_time_i_s,state_time_j_s,duration_from_state_i_s,"
       "factor_used,rtk_up_m,sigma_u_m,half_width_m,predicted_up_m,raw_residual_m,"
       "violation_m,inside_envelope,center_pull_factor_used,center_pull_reference_type,"
       "center_pull_reference_up_m,rtk_drift_estimate_m,center_pull_sigma_m,"
       "center_pull_deadband_m,center_pull_residual_m\n";
  for (const auto &row : rows) {
    stream << row.sample_index << ','
           << row.raw_time_s << ','
           << row.corrected_time_s << ','
           << ToString(row.sync_status) << ','
           << row.state_index_i << ','
           << row.state_index_j << ','
           << row.synchronized_state_index << ','
           << row.state_time_i_s << ','
           << row.state_time_j_s << ','
           << row.duration_from_state_i_s << ','
           << (row.factor_used ? 1 : 0) << ','
           << row.rtk_up_m << ','
           << row.sigma_u_m << ','
           << row.half_width_m << ','
           << row.predicted_up_m << ','
           << row.raw_residual_m << ','
           << row.violation_m << ','
           << (row.inside_envelope ? 1 : 0) << ','
           << (row.center_pull_factor_used ? 1 : 0) << ','
           << row.center_pull_reference_type << ','
           << row.center_pull_reference_up_m << ','
           << row.rtk_drift_estimate_m << ','
           << row.center_pull_sigma_m << ','
           << row.center_pull_deadband_m << ','
           << row.center_pull_residual_m << '\n';
  }
}

void WriteRtkVerticalDriftReferenceDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<RtkVerticalDriftReferenceDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "sample_index,time_s,raw_rtk_up_m,nav_reference_up_m,residual_m,constant_bias_m,"
       "drift_estimate_m,corrected_center_up_m,white_residual_m,drift_sigma_m,"
       "white_sigma_m,tau_s,static_window_flag,valid,skip_reason\n";
  for (const auto &row : rows) {
    stream << row.sample_index << ','
           << row.time_s << ','
           << row.raw_rtk_up_m << ','
           << row.nav_reference_up_m << ','
           << row.residual_m << ','
           << row.constant_bias_m << ','
           << row.drift_estimate_m << ','
           << row.corrected_center_up_m << ','
           << row.white_residual_m << ','
           << row.drift_sigma_m << ','
           << row.white_sigma_m << ','
           << row.tau_s << ','
           << (row.static_window_flag ? 1 : 0) << ','
           << (row.valid ? 1 : 0) << ','
           << row.skip_reason << '\n';
  }
}

void WriteRtkOutageWindowsCsv(
  const std::filesystem::path &path,
  const std::vector<RtkOutageWindowRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "window_index,pre_sample_index,post_sample_index,pre_anchor_state_index,"
       "post_anchor_state_index,pre_anchor_time_s,post_anchor_time_s,start_time_s,"
       "end_time_s,duration_s,interior_state_count,rejected_sample_count,"
       "body_z_jump_overlap_count,initial_value_smoothing_applied,"
       "initial_value_smoothed_state_count,pre_anchor_up_m,post_anchor_up_m,"
       "post_anchor_up_offset_m,factor_added,position_ramp_factor_count,"
       "velocity_delta_factor_count,velocity_delta_skipped_body_z_jump_count,"
       "skip_reason\n";
  for (const auto &row : rows) {
    stream << row.window_index << ','
           << row.pre_sample_index << ','
           << row.post_sample_index << ','
           << row.pre_anchor_state_index << ','
           << row.post_anchor_state_index << ','
           << row.pre_anchor_time_s << ','
           << row.post_anchor_time_s << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << row.duration_s << ','
           << row.interior_state_count << ','
           << row.rejected_sample_count << ','
           << row.body_z_jump_overlap_count << ','
           << (row.initial_value_smoothing_applied ? 1 : 0) << ','
           << row.initial_value_smoothed_state_count << ','
           << row.pre_anchor_up_m << ','
           << row.post_anchor_up_m << ','
           << row.post_anchor_up_offset_m << ','
           << (row.factor_added ? 1 : 0) << ','
           << row.position_ramp_factor_count << ','
           << row.velocity_delta_factor_count << ','
           << row.velocity_delta_skipped_body_z_jump_count << ','
           << row.skip_reason << '\n';
  }
}

void WriteRtkVelocityDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<RtkVelocityDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "sample_index,state_index,raw_time_s,corrected_time_s,state_time_s,window_dt_s,"
       "factor_added,skip_reason,sync_status,sigma_mps,"
       "rtk_vx_mps,rtk_vy_mps,rtk_vz_mps,opt_vx_mps,opt_vy_mps,opt_vz_mps,"
       "residual_vx_mps,residual_vy_mps,residual_vz_mps,horizontal_residual_mps,"
       "rtk_body_x_mps,rtk_body_y_mps,rtk_body_z_mps,"
       "opt_body_x_mps,opt_body_y_mps,opt_body_z_mps,body_y_residual_mps\n";
  for (const auto &row : rows) {
    stream << row.sample_index << ','
           << row.state_index << ','
           << row.raw_time_s << ','
           << row.corrected_time_s << ','
           << row.state_time_s << ','
           << row.window_dt_s << ','
           << (row.factor_added ? 1 : 0) << ','
           << row.skip_reason << ','
           << ToString(row.sync_status) << ','
           << row.sigma_mps << ','
           << row.rtk_velocity_mps.x() << ','
           << row.rtk_velocity_mps.y() << ','
           << row.rtk_velocity_mps.z() << ','
           << row.optimized_velocity_mps.x() << ','
           << row.optimized_velocity_mps.y() << ','
           << row.optimized_velocity_mps.z() << ','
           << row.velocity_residual_mps.x() << ','
           << row.velocity_residual_mps.y() << ','
           << row.velocity_residual_mps.z() << ','
           << row.horizontal_residual_mps << ','
           << row.rtk_body_x_mps << ','
           << row.rtk_body_y_mps << ','
           << row.rtk_body_z_mps << ','
           << row.optimized_body_x_mps << ','
           << row.optimized_body_y_mps << ','
           << row.optimized_body_z_mps << ','
           << row.body_y_residual_mps << '\n';
  }
}

void WriteStage1YawRefinementDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<Stage1YawRefinementDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "iteration,input_yaw_rad,median_error_rad,heading_noise_rad,yaw_update_rad,next_yaw_rad,"
       "valid_pair_count,mean_abs_error_rad,rms_error_rad,max_abs_error_rad,final_error,gnss_nis_mean,"
       "stop_reason\n";
  for (const auto &row : rows) {
    stream << row.iteration << ','
           << row.input_yaw_rad << ','
           << row.median_error_rad << ','
           << row.heading_noise_rad << ','
           << row.yaw_update_rad << ','
           << row.next_yaw_rad << ','
           << row.valid_pair_count << ','
           << row.mean_abs_error_rad << ','
           << row.rms_error_rad << ','
           << row.max_abs_error_rad << ','
           << row.final_error << ','
           << row.gnss_nis_mean << ','
           << row.stop_reason << '\n';
  }
}

void WriteStaticAlignmentValidationCsv(
  const std::filesystem::path &path,
  const std::vector<StaticAlignmentValidationRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "time_s,relative_time_s,up_delta_m,vz_mps,ba_z_ug,global_ba_z_ug,"
       "ba_z_minus_global_ug,static_bias_gm_residual_ug,static_height_residual_m,"
       "rtk_reference_up_m,rtk_reference_residual_m\n";
  for (const auto &row : rows) {
    stream << row.time_s << ','
           << row.relative_time_s << ','
           << row.up_delta_m << ','
           << row.vz_mps << ','
           << row.ba_z_ug << ','
           << row.global_ba_z_ug << ','
           << row.ba_z_minus_global_ug << ','
           << row.static_bias_gm_residual_ug << ','
           << row.static_height_residual_m << ','
           << row.rtk_reference_up_m << ','
           << row.rtk_reference_residual_m << '\n';
  }
}

void WriteVerticalVelocityDeltaDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalVelocityDeltaDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "state_i,state_j,outer_pass,start_time_s,end_time_s,dt_s,factor_added,skip_reason,in_jump_padding,"
       "target_clamped,raw_target_delta_vz_mps,"
       "target_delta_vz_mps,optimized_delta_vz_mps,residual_mps,sigma_mps,sigma_model,"
       "legacy_sigma_mps,bias_sigma_mps,attitude_sigma_mps,sigma_floor_mps,sigma_ceiling_mps,"
       "adaptive_motion_score,adaptive_sigma_mps,adaptive_sigma_ratio,"
       "local_horizontal_speed_rms_mps,local_vz_rms_mps,local_vz_range_mps,"
       "local_target_acc_rms_mps2,"
       "bias_aware_factor,reference_ba_z_ug,optimized_ba_z_ug,bias_delta_ug,"
       "bias_delta_velocity_correction_mps\n";
  for (const auto &row : rows) {
    stream << row.state_index_i << ','
           << row.state_index_j << ','
           << row.outer_pass << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << row.dt_s << ','
           << (row.factor_added ? 1 : 0) << ','
           << row.skip_reason << ','
           << (row.in_jump_padding ? 1 : 0) << ','
           << (row.target_clamped ? 1 : 0) << ','
           << row.raw_target_delta_vz_mps << ','
           << row.target_delta_vz_mps << ','
           << row.optimized_delta_vz_mps << ','
           << row.residual_mps << ','
           << row.sigma_mps << ','
           << row.sigma_model << ','
           << row.legacy_sigma_mps << ','
           << row.bias_sigma_mps << ','
           << row.attitude_sigma_mps << ','
           << row.sigma_floor_mps << ','
           << row.sigma_ceiling_mps << ','
           << row.adaptive_motion_score << ','
           << row.adaptive_sigma_mps << ','
           << row.adaptive_sigma_ratio << ','
           << row.local_horizontal_speed_rms_mps << ','
           << row.local_vz_rms_mps << ','
           << row.local_vz_range_mps << ','
           << row.local_target_acc_rms_mps2 << ','
           << (row.bias_aware_factor ? 1 : 0) << ','
           << row.reference_ba_z_ug << ','
           << row.optimized_ba_z_ug << ','
           << row.bias_delta_ug << ','
           << row.bias_delta_velocity_correction_mps << '\n';
  }
}

void WriteVerticalMotionAdaptiveReweightingDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalMotionAdaptiveReweightingDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "outer_pass,state_i,state_j,start_time_s,end_time_s,dt_s,motion_score,stability_class,"
       "horizontal_speed_rms_mps,vz_rms_mps,vz_range_mps,target_vertical_acc_rms_mps2,"
       "dvz_sigma_before_mps,dvz_sigma_after_mps,baz_gm_sigma_before_ug,baz_gm_sigma_after_ug,"
       "in_jump_padding,skip_reason\n";
  for (const auto &row : rows) {
    stream << row.outer_pass << ','
           << row.state_index_i << ','
           << row.state_index_j << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << row.dt_s << ','
           << row.motion_score << ','
           << row.stability_class << ','
           << row.horizontal_speed_rms_mps << ','
           << row.vz_rms_mps << ','
           << row.vz_range_mps << ','
           << row.target_vertical_acc_rms_mps2 << ','
           << row.dvz_sigma_before_mps << ','
           << row.dvz_sigma_after_mps << ','
           << row.baz_gm_sigma_before_ug << ','
           << row.baz_gm_sigma_after_ug << ','
           << (row.in_jump_padding ? 1 : 0) << ','
           << row.skip_reason << '\n';
  }
}

void WriteVerticalPositionVelocityConsistencyDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalPositionVelocityConsistencyDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "constraint_type,state_i,state_j,state_count,start_time_s,end_time_s,dt_s,interval_type,"
       "factor_added,skip_reason,"
       "sigma_m,initial_delta_z_m,initial_trapezoid_vz_integral_m,initial_mismatch_m,"
       "optimized_delta_z_m,optimized_trapezoid_vz_integral_m,optimized_mismatch_m,"
       "normalized_residual\n";
  for (const auto &row : rows) {
    stream << row.constraint_type << ','
           << row.state_index_i << ','
           << row.state_index_j << ','
           << row.state_count << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << row.dt_s << ','
           << row.interval_type << ','
           << (row.factor_added ? 1 : 0) << ','
           << row.skip_reason << ','
           << row.sigma_m << ','
           << row.initial_delta_z_m << ','
           << row.initial_trapezoid_vz_integral_m << ','
           << row.initial_mismatch_m << ','
           << row.optimized_delta_z_m << ','
           << row.optimized_trapezoid_vz_integral_m << ','
           << row.optimized_mismatch_m << ','
           << row.normalized_residual << '\n';
  }
}

void WriteAttitudeReferenceDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<AttitudeReferenceDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "state_index,time_s,factor_added,skip_reason,"
       "reference_yaw_rad,reference_pitch_rad,reference_roll_rad,"
       "optimized_yaw_rad,optimized_pitch_rad,optimized_roll_rad,"
       "residual_x_rad,residual_y_rad,residual_z_rad,residual_norm_rad\n";
  for (const auto &row : rows) {
    stream << row.state_index << ','
           << row.time_s << ','
           << (row.factor_added ? 1 : 0) << ','
           << row.skip_reason << ','
           << row.reference_ypr_rad.x() << ','
           << row.reference_ypr_rad.y() << ','
           << row.reference_ypr_rad.z() << ','
           << row.optimized_ypr_rad.x() << ','
           << row.optimized_ypr_rad.y() << ','
           << row.optimized_ypr_rad.z() << ','
           << row.residual_x_rad << ','
           << row.residual_y_rad << ','
           << row.residual_z_rad << ','
           << row.residual_norm_rad << '\n';
  }
}

void WriteRelativeYawReferenceDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<RelativeYawReferenceDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "edge_index,state_index_i,state_index_j,time_i_s,time_j_s,"
       "factor_added,skip_reason,sigma_rad,"
       "reference_delta_yaw_rad,optimized_delta_yaw_rad,residual_yaw_rad\n";
  for (const auto &row : rows) {
    stream << row.edge_index << ','
           << row.state_index_i << ','
           << row.state_index_j << ','
           << row.time_i_s << ','
           << row.time_j_s << ','
           << (row.factor_added ? 1 : 0) << ','
           << row.skip_reason << ','
           << row.sigma_rad << ','
           << row.reference_delta_yaw_rad << ','
           << row.optimized_delta_yaw_rad << ','
           << row.residual_yaw_rad << '\n';
  }
}

void WriteBodyZNHCDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZNHCDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "window_index,window_type,from_jump_window,source_window_index,source_window_count,"
       "start_state_index,end_state_index,state_count,start_time_s,end_time_s,duration_s,"
       "actual_state_span_s,factor_added,skip_reason,velocity_factor_count,displacement_factor_count,"
        "velocity_sigma_mps,displacement_sigma_m,strict_weighting_enabled,"
        "target_velocity_sigma_mps,applied_velocity_sigma_mps,target_displacement_sigma_m,"
        "applied_displacement_sigma_m,velocity_state_duplicate_count,interval_overlap_count,"
        "horizontal_leakage_correction_enabled,"
        "horizontal_leakage_x_rad,horizontal_leakage_y_rad,initial_mean_abs_body_z_velocity_mps,"
        "initial_max_abs_body_z_velocity_mps,initial_body_z_displacement_m,"
        "initial_mean_abs_corrected_body_z_velocity_mps,"
        "initial_max_abs_corrected_body_z_velocity_mps,initial_corrected_body_z_displacement_m,"
        "optimized_mean_abs_body_z_velocity_mps,optimized_max_abs_body_z_velocity_mps,"
        "optimized_body_z_displacement_m,optimized_pose_mean_abs_body_z_velocity_mps,"
        "optimized_mean_abs_corrected_body_z_velocity_mps,"
        "optimized_max_abs_corrected_body_z_velocity_mps,optimized_corrected_body_z_displacement_m,"
        "optimized_pose_max_abs_body_z_velocity_mps,optimized_pose_body_z_displacement_m,"
        "optimized_pitch_range_rad,optimized_roll_range_rad,max_velocity_residual_mps,"
        "displacement_residual_m\n";
  for (const auto &row : rows) {
    stream << row.window_index << ','
           << row.window_type << ','
           << (row.from_jump_window ? 1 : 0) << ','
           << row.source_window_index << ','
           << row.source_window_count << ','
           << row.start_state_index << ','
           << row.end_state_index << ','
           << row.state_count << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << row.duration_s << ','
           << row.actual_state_span_s << ','
           << (row.factor_added ? 1 : 0) << ','
           << row.skip_reason << ','
           << row.velocity_factor_count << ','
           << row.displacement_factor_count << ','
           << row.velocity_sigma_mps << ','
           << row.displacement_sigma_m << ','
           << (row.strict_weighting_enabled ? 1 : 0) << ','
           << row.target_velocity_sigma_mps << ','
           << row.applied_velocity_sigma_mps << ','
           << row.target_displacement_sigma_m << ','
           << row.applied_displacement_sigma_m << ','
           << row.velocity_state_duplicate_count << ','
           << row.interval_overlap_count << ','
           << (row.horizontal_leakage_correction_enabled ? 1 : 0) << ','
           << row.horizontal_leakage_x_rad << ','
           << row.horizontal_leakage_y_rad << ','
           << row.initial_mean_abs_body_z_velocity_mps << ','
           << row.initial_max_abs_body_z_velocity_mps << ','
           << row.initial_body_z_displacement_m << ','
           << row.initial_mean_abs_corrected_body_z_velocity_mps << ','
           << row.initial_max_abs_corrected_body_z_velocity_mps << ','
           << row.initial_corrected_body_z_displacement_m << ','
            << row.optimized_mean_abs_body_z_velocity_mps << ','
            << row.optimized_max_abs_body_z_velocity_mps << ','
            << row.optimized_body_z_displacement_m << ','
            << row.optimized_pose_mean_abs_body_z_velocity_mps << ','
            << row.optimized_mean_abs_corrected_body_z_velocity_mps << ','
            << row.optimized_max_abs_corrected_body_z_velocity_mps << ','
            << row.optimized_corrected_body_z_displacement_m << ','
            << row.optimized_pose_max_abs_body_z_velocity_mps << ','
            << row.optimized_pose_body_z_displacement_m << ','
            << row.optimized_pitch_range_rad << ','
            << row.optimized_roll_range_rad << ','
            << row.max_velocity_residual_mps << ','
            << row.displacement_residual_m << '\n';
  }
}

void WriteBodyZHorizontalLeakageDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZHorizontalLeakageDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "enabled,estimate_valid,velocity_source,skip_reason,candidate_sample_count,"
       "used_sample_count,skipped_window_count,skipped_low_speed_count,skipped_invalid_count,"
       "min_speed_mps,huber_sigma_mps,max_abs_coeff_rad,leak_x_rad,leak_y_rad,"
       "raw_rms_body_z_mps,raw_max_abs_body_z_mps,corrected_rms_body_z_mps,"
       "corrected_max_abs_body_z_mps\n";
  for (const auto &row : rows) {
    stream << (row.enabled ? 1 : 0) << ','
           << (row.estimate_valid ? 1 : 0) << ','
           << row.velocity_source << ','
           << row.skip_reason << ','
           << row.candidate_sample_count << ','
           << row.used_sample_count << ','
           << row.skipped_window_count << ','
           << row.skipped_low_speed_count << ','
           << row.skipped_invalid_count << ','
           << row.min_speed_mps << ','
           << row.huber_sigma_mps << ','
           << row.max_abs_coeff_rad << ','
           << row.leak_x_rad << ','
           << row.leak_y_rad << ','
           << row.raw_rms_body_z_mps << ','
           << row.raw_max_abs_body_z_mps << ','
           << row.corrected_rms_body_z_mps << ','
           << row.corrected_max_abs_body_z_mps << '\n';
  }
}

void WriteBodyZNHCStateDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZNHCStateDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "window_index,state_index,time_s,nhc_region_type,velocity_factor_used,"
       "effective_velocity_sigma_mps,fixed_axis_x,fixed_axis_y,fixed_axis_z,"
       "vx_mps,vy_mps,vz_mps,horizontal_speed_mps,v_body_x_mps,v_body_y_mps,"
       "raw_v_body_z_mps,horizontal_leakage_x_rad,horizontal_leakage_y_rad,"
       "leakage_correction_mps,corrected_v_body_z_mps,fixed_horizontal_projection_mps,"
       "fixed_vertical_projection_mps,fixed_body_z_velocity_mps,optimized_pose_axis_x,"
       "optimized_pose_axis_y,optimized_pose_axis_z,optimized_pose_horizontal_projection_mps,"
       "optimized_pose_vertical_projection_mps,optimized_pose_body_z_velocity_mps\n";
  for (const auto &row : rows) {
    stream << row.window_index << ','
           << row.state_index << ','
           << row.time_s << ','
           << row.nhc_region_type << ','
           << (row.velocity_factor_used ? 1 : 0) << ','
           << row.effective_velocity_sigma_mps << ','
           << row.fixed_axis_x << ','
           << row.fixed_axis_y << ','
           << row.fixed_axis_z << ','
           << row.vx_mps << ','
           << row.vy_mps << ','
           << row.vz_mps << ','
           << row.horizontal_speed_mps << ','
           << row.v_body_x_mps << ','
           << row.v_body_y_mps << ','
           << row.raw_v_body_z_mps << ','
           << row.horizontal_leakage_x_rad << ','
           << row.horizontal_leakage_y_rad << ','
           << row.leakage_correction_mps << ','
           << row.corrected_v_body_z_mps << ','
           << row.fixed_horizontal_projection_mps << ','
           << row.fixed_vertical_projection_mps << ','
           << row.fixed_body_z_velocity_mps << ','
           << row.optimized_pose_axis_x << ','
           << row.optimized_pose_axis_y << ','
           << row.optimized_pose_axis_z << ','
           << row.optimized_pose_horizontal_projection_mps << ','
           << row.optimized_pose_vertical_projection_mps << ','
           << row.optimized_pose_body_z_velocity_mps << '\n';
  }
}

void WriteStage2MountLeakageDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<Stage2MountLeakageDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "enabled,estimate_valid,skip_reason,used_sample_count,prior_sigma_rad,"
       "initial_k_zx_rad,initial_k_zy_rad,initial_k_yx_rad,"
       "optimized_k_zx_rad,optimized_k_zy_rad,optimized_k_yx_rad,"
       "prior_residual_norm,initial_raw_y_rms_mps,initial_raw_y_max_abs_mps,"
       "initial_vehicle_y_rms_mps,initial_vehicle_y_max_abs_mps,"
       "optimized_raw_y_rms_mps,optimized_raw_y_max_abs_mps,"
       "optimized_vehicle_y_rms_mps,optimized_vehicle_y_max_abs_mps,"
       "initial_raw_z_rms_mps,initial_raw_z_max_abs_mps,"
       "initial_vehicle_z_rms_mps,initial_vehicle_z_max_abs_mps,"
       "optimized_raw_z_rms_mps,optimized_raw_z_max_abs_mps,"
       "optimized_vehicle_z_rms_mps,optimized_vehicle_z_max_abs_mps\n";
  for (const auto &row : rows) {
    stream << (row.enabled ? 1 : 0) << ','
           << (row.estimate_valid ? 1 : 0) << ','
           << row.skip_reason << ','
           << row.used_sample_count << ','
           << row.prior_sigma_rad << ','
           << row.initial_k_zx_rad << ','
           << row.initial_k_zy_rad << ','
           << row.initial_k_yx_rad << ','
           << row.optimized_k_zx_rad << ','
           << row.optimized_k_zy_rad << ','
           << row.optimized_k_yx_rad << ','
           << row.prior_residual_norm << ','
           << row.initial_raw_y_rms_mps << ','
           << row.initial_raw_y_max_abs_mps << ','
           << row.initial_vehicle_y_rms_mps << ','
           << row.initial_vehicle_y_max_abs_mps << ','
           << row.optimized_raw_y_rms_mps << ','
           << row.optimized_raw_y_max_abs_mps << ','
           << row.optimized_vehicle_y_rms_mps << ','
           << row.optimized_vehicle_y_max_abs_mps << ','
           << row.initial_raw_z_rms_mps << ','
           << row.initial_raw_z_max_abs_mps << ','
           << row.initial_vehicle_z_rms_mps << ','
           << row.initial_vehicle_z_max_abs_mps << ','
           << row.optimized_raw_z_rms_mps << ','
           << row.optimized_raw_z_max_abs_mps << ','
           << row.optimized_vehicle_z_rms_mps << ','
           << row.optimized_vehicle_z_max_abs_mps << '\n';
  }
}

void WriteStage2VehicleNHCStateDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<Stage2VehicleNHCStateDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "window_index,state_index,time_s,nhc_region_type,velocity_factor_used,"
       "effective_vehicle_y_sigma_mps,effective_vehicle_z_sigma_mps,"
       "vx_mps,vy_mps,vz_mps,v_body_x_mps,v_body_y_mps,v_body_z_mps,"
       "k_zx_rad,k_zy_rad,k_yx_rad,vehicle_y_correction_mps,"
       "vehicle_z_correction_from_x_mps,vehicle_z_correction_from_y_mps,"
       "v_vehicle_y_mps,v_vehicle_z_mps\n";
  for (const auto &row : rows) {
    stream << row.window_index << ','
           << row.state_index << ','
           << row.time_s << ','
           << row.nhc_region_type << ','
           << (row.velocity_factor_used ? 1 : 0) << ','
           << row.effective_vehicle_y_sigma_mps << ','
           << row.effective_vehicle_z_sigma_mps << ','
           << row.vx_mps << ','
           << row.vy_mps << ','
           << row.vz_mps << ','
           << row.v_body_x_mps << ','
           << row.v_body_y_mps << ','
           << row.v_body_z_mps << ','
           << row.k_zx_rad << ','
           << row.k_zy_rad << ','
           << row.k_yx_rad << ','
           << row.vehicle_y_correction_mps << ','
           << row.vehicle_z_correction_from_x_mps << ','
           << row.vehicle_z_correction_from_y_mps << ','
           << row.v_vehicle_y_mps << ','
           << row.v_vehicle_z_mps << '\n';
  }
}

void WriteVerticalJumpMaskedImuDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpMaskedImuDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "state_i,state_j,start_time_s,end_time_s,factor_type,overlap_jump_padding,masked_z_position,masked_vz\n";
  for (const auto &row : rows) {
    stream << row.state_index_i << ','
           << row.state_index_j << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << row.factor_type << ','
           << (row.overlap_jump_padding ? 1 : 0) << ','
           << (row.masked_z_position ? 1 : 0) << ','
           << (row.masked_vz ? 1 : 0) << '\n';
  }
}

void WriteVerticalJumpImpulseDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpImpulseDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "span_index,source_window_index,source_window_count,start_state,end_state,pre_anchor,post_anchor,"
       "start_time_s,end_time_s,factor_added,skip_reason,replaced_imu_factor_count,"
       "imu_delta_vz_mps,detected_delta_vz_init_mps,detected_signed_delta_velocity_mps,"
       "prior_sigma_mps,velocity_sigma_mps,estimated_jump_impulse_mps,corrected_delta_vz_mps,"
       "optimized_delta_vz_mps,residual_mps,pre_anchor_vz_mps,post_anchor_vz_mps\n";
  for (const auto &row : rows) {
    stream << row.span_index << ','
           << row.source_window_index << ','
           << row.source_window_count << ','
           << row.start_state_index << ','
           << row.end_state_index << ','
           << row.pre_anchor_state_index << ','
           << row.post_anchor_state_index << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << (row.factor_added ? 1 : 0) << ','
           << row.skip_reason << ','
           << row.replaced_imu_factor_count << ','
           << row.imu_delta_vz_mps << ','
           << row.detected_delta_vz_init_mps << ','
           << row.detected_signed_delta_velocity_mps << ','
           << row.prior_sigma_mps << ','
           << row.velocity_sigma_mps << ','
           << row.estimated_jump_impulse_mps << ','
           << row.corrected_delta_vz_mps << ','
           << row.optimized_delta_vz_mps << ','
           << row.residual_mps << ','
           << row.pre_anchor_vz_mps << ','
           << row.post_anchor_vz_mps << '\n';
  }
}

void WriteVerticalJumpBiasDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpBiasDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "span_index,segment_index,segment_count,bias_key_index,"
       "source_window_index,source_window_count,start_state,end_state,"
       "start_time_s,end_time_s,factor_added,skip_reason,replaced_imu_factor_count,"
       "velocity_factor_count,position_velocity_factor_count,source_window_duration_s,"
       "factor_duration_s,imu_delta_vz_mps,detected_signed_delta_velocity_mps,"
       "detected_bias_mps2,used_segmented_estimate,prior_sigma_mps2,"
       "base_velocity_sigma_mps,highfreq_rms_mps2,highfreq_p95_abs_mps2,"
       "highfreq_sigma_inflation_mps,velocity_sigma_mps,position_velocity_sigma_m,"
       "estimated_bias_mps2,corrected_delta_vz_mps,optimized_delta_vz_mps,residual_mps\n";
  for (const auto &row : rows) {
    stream << row.span_index << ','
           << row.segment_index << ','
           << row.segment_count << ','
           << row.bias_key_index << ','
           << row.source_window_index << ','
           << row.source_window_count << ','
           << row.start_state_index << ','
           << row.end_state_index << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << (row.factor_added ? 1 : 0) << ','
           << row.skip_reason << ','
           << row.replaced_imu_factor_count << ','
           << row.velocity_factor_count << ','
           << row.position_velocity_factor_count << ','
           << row.source_window_duration_s << ','
           << row.factor_duration_s << ','
           << row.imu_delta_vz_mps << ','
           << row.detected_signed_delta_velocity_mps << ','
           << row.detected_bias_mps2 << ','
           << (row.used_segmented_estimate ? 1 : 0) << ','
           << row.prior_sigma_mps2 << ','
           << row.base_velocity_sigma_mps << ','
           << row.highfreq_rms_mps2 << ','
           << row.highfreq_p95_abs_mps2 << ','
           << row.highfreq_sigma_inflation_mps << ','
           << row.velocity_sigma_mps << ','
           << row.position_velocity_sigma_m << ','
           << row.estimated_bias_mps2 << ','
           << row.corrected_delta_vz_mps << ','
           << row.optimized_delta_vz_mps << ','
           << row.residual_mps << '\n';
  }
}

void WriteVerticalJumpVelocityRampDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpVelocityRampDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "window_index,start_state_index,end_state_index,start_time_s,end_time_s,factor_count,skip_reason,"
       "pre_vz_mean_mps,post_vz_mean_mps,jump_delta_vz_mps,inside_vz_min_mps,inside_vz_max_mps,"
       "inside_vz_range_mps,ramp_residual_max_mps,ramp_residual_p95_mps,inside_up_min_m,inside_up_max_m,"
       "inside_up_range_m,position_ramp_residual_max_m,position_ramp_residual_p95_m\n";
  for (const auto &row : rows) {
    stream << row.window_index << ','
           << row.start_state_index << ','
           << row.end_state_index << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << row.factor_count << ','
           << row.skip_reason << ','
           << row.pre_vz_mean_mps << ','
           << row.post_vz_mean_mps << ','
           << row.jump_delta_vz_mps << ','
           << row.inside_vz_min_mps << ','
           << row.inside_vz_max_mps << ','
           << row.inside_vz_range_mps << ','
           << row.ramp_residual_max_mps << ','
           << row.ramp_residual_p95_mps << ','
           << row.inside_up_min_m << ','
           << row.inside_up_max_m << ','
           << row.inside_up_range_m << ','
           << row.position_ramp_residual_max_m << ','
           << row.position_ramp_residual_p95_m << '\n';
  }
}

void WriteVerticalJumpContinuityDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpContinuityDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write " + path.filename().string());
  }
  stream << std::setprecision(17);
  stream
    << "window_index,start_state,end_state,pre_anchor,post_anchor,start_time_s,end_time_s,"
       "pre_context_start_state,pre_context_end_state,post_context_start_state,post_context_end_state,"
       "pre_context_state_count,post_context_state_count,velocity_context_factor_count,"
       "context_mean_continuity_factor_added,"
       "entry_factor_added,exit_factor_added,skip_reason,entry_delta_vz_mps,exit_delta_vz_mps,"
       "entry_residual_mps,exit_residual_mps,"
       "entry_position_velocity_factor_added,exit_position_velocity_factor_added,"
       "entry_delta_z_m,entry_velocity_integral_m,entry_zv_mismatch_m,"
       "exit_delta_z_m,exit_velocity_integral_m,exit_zv_mismatch_m,"
       "pre_context_mean_vz_mps,post_context_mean_vz_mps,"
       "context_mean_delta_vz_mps,context_mean_continuity_residual_mps,"
       "max_pre_context_residual_mps,max_post_context_residual_mps,"
       "max_inside_vz_range_mps,max_boundary_step_mps,max_boundary_zv_mismatch_m,"
       "max_position_velocity_residual_m\n";
  for (const auto &row : rows) {
    stream << row.window_index << ','
           << row.start_state_index << ','
           << row.end_state_index << ','
           << row.pre_anchor_state_index << ','
           << row.post_anchor_state_index << ','
           << row.start_time_s << ','
           << row.end_time_s << ','
           << row.pre_context_start_state_index << ','
           << row.pre_context_end_state_index << ','
           << row.post_context_start_state_index << ','
           << row.post_context_end_state_index << ','
           << row.pre_context_state_count << ','
           << row.post_context_state_count << ','
           << row.velocity_context_factor_count << ','
           << (row.context_mean_continuity_factor_added ? 1 : 0) << ','
           << (row.entry_factor_added ? 1 : 0) << ','
           << (row.exit_factor_added ? 1 : 0) << ','
           << row.skip_reason << ','
           << row.entry_delta_vz_mps << ','
           << row.exit_delta_vz_mps << ','
           << row.entry_residual_mps << ','
           << row.exit_residual_mps << ','
           << (row.entry_position_velocity_factor_added ? 1 : 0) << ','
           << (row.exit_position_velocity_factor_added ? 1 : 0) << ','
           << row.entry_delta_z_m << ','
           << row.entry_velocity_integral_m << ','
           << row.entry_zv_mismatch_m << ','
           << row.exit_delta_z_m << ','
           << row.exit_velocity_integral_m << ','
           << row.exit_zv_mismatch_m << ','
           << row.pre_context_mean_vz_mps << ','
           << row.post_context_mean_vz_mps << ','
           << row.context_mean_delta_vz_mps << ','
           << row.context_mean_continuity_residual_mps << ','
           << row.max_pre_context_residual_mps << ','
           << row.max_post_context_residual_mps << ','
           << row.max_inside_vz_range_mps << ','
           << row.max_boundary_step_mps << ','
           << row.max_boundary_zv_mismatch_m << ','
           << row.max_position_velocity_residual_m << '\n';
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
       "vertical_direct_position_factor_used,measurement_up_m,optimized_up_m,optimized_vz_mps,"
       "optimized_pitch_rad,optimized_roll_rad,optimized_baz_ug,postfit_residual_u_m\n";
  for (const auto &row : rows) {
    stream << row.sample_index << ','
           << row.raw_time_s << ','
           << row.corrected_time_s << ','
           << ToString(row.sync_status) << ','
           << row.state_index << ','
           << row.state_time_s << ','
           << (row.factor_used ? 1 : 0) << ','
           << (row.vertical_direct_position_factor_used ? 1 : 0) << ','
           << row.measurement_up_m << ','
           << row.optimized_up_m << ','
           << row.optimized_vz_mps << ','
           << row.optimized_pitch_rad << ','
           << row.optimized_roll_rad << ','
           << Mps2ToMicroG(row.optimized_baz_mps2) << ','
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
    << "time_s,relative_time_s,up_m,vz_mps,yaw_rad,pitch_rad,roll_rad,baz_ug,bgz_radps,initial_baz_ug,"
       "initial_bgz_radps,static_baz_ug,static_bgz_radps,optimized_last_static_baz_ug,"
       "optimized_last_static_bgz_radps,optimized_first_static_baz_ug,optimized_first_static_bgz_radps,"
       "optimized_first_dynamic_baz_ug,optimized_first_dynamic_bgz_radps,"
       "bootstrap_to_optimized_first_dynamic_baz_delta_ug,static_to_dynamic_baz_delta_ug\n";

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
           << Mps2ToMicroG(row.bias_acc.z()) << ','
           << row.bias_gyro.z() << ','
           << Mps2ToMicroG(run_summary.initial_baz_mps2) << ','
           << run_summary.initial_bgz_radps << ','
           << Mps2ToMicroG(run_summary.static_baz_mps2) << ','
           << run_summary.static_bgz_radps << ','
           << Mps2ToMicroG(run_summary.optimized_last_static_baz_mps2) << ','
           << run_summary.optimized_last_static_bgz_radps << ','
           << Mps2ToMicroG(run_summary.optimized_first_static_baz_mps2) << ','
           << run_summary.optimized_first_static_bgz_radps << ','
           << Mps2ToMicroG(run_summary.optimized_first_dynamic_baz_mps2) << ','
           << run_summary.optimized_first_dynamic_bgz_radps << ','
           << Mps2ToMicroG(run_summary.bootstrap_to_optimized_first_dynamic_baz_delta_mps2) << ','
           << Mps2ToMicroG(run_summary.static_to_dynamic_baz_delta_mps2) << '\n';
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

}  // namespace offline_lc_minimal
