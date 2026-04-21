#include "offline_lc_minimal/common/Config.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace offline_lc_minimal {

namespace {

std::string Trim(std::string value) {
  auto not_space = [](const unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string StripQuotes(std::string value) {
  value = Trim(std::move(value));
  if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') ||
                            (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

bool ParseBool(const std::string &value) {
  const auto lowered = [&]() {
    std::string result = value;
    std::transform(result.begin(), result.end(), result.begin(), [](const unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return result;
  }();

  if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on") {
    return true;
  }
  if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off") {
    return false;
  }
  throw std::runtime_error("invalid boolean value: " + value);
}

double ParseDouble(const std::string &value) {
  return std::stod(value);
}

int ParseInt(const std::string &value) {
  return std::stoi(value);
}

GnssNoiseModel ParseNoiseModel(const std::string &value) {
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (lowered == "gaussian") {
    return GnssNoiseModel::kGaussian;
  }
  if (lowered == "cauchy") {
    return GnssNoiseModel::kCauchy;
  }
  if (lowered == "huber") {
    return GnssNoiseModel::kHuber;
  }
  if (lowered == "dcs") {
    return GnssNoiseModel::kDcs;
  }
  if (lowered == "tukey") {
    return GnssNoiseModel::kTukey;
  }
  if (lowered == "geman_mcclure") {
    return GnssNoiseModel::kGemanMcClure;
  }
  if (lowered == "welsch") {
    return GnssNoiseModel::kWelsch;
  }
  throw std::runtime_error("invalid GNSS noise model: " + value);
}

}  // namespace

OfflineRunnerConfig DefaultConfig() {
  return OfflineRunnerConfig{};
}

void OverrideConfigField(OfflineRunnerConfig &config, const std::string_view key, const std::string_view value) {
  const std::string normalized_key = Trim(std::string(key));
  const std::string normalized_value = StripQuotes(std::string(value));

  if (normalized_key == "imu_path") {
    config.imu_path = normalized_value;
  } else if (normalized_key == "gnss_path") {
    config.gnss_path = normalized_value;
  } else if (normalized_key == "output_dir") {
    config.output_dir = normalized_value;
  } else if (normalized_key == "enable_gnss") {
    config.enable_gnss = ParseBool(normalized_value);
  } else if (normalized_key == "enable_gp_interpolated_gnss") {
    config.enable_gp_interpolated_gnss = ParseBool(normalized_value);
  } else if (normalized_key == "verbose") {
    config.verbose = ParseBool(normalized_value);
  } else if (normalized_key == "write_debug_csv") {
    config.write_debug_csv = ParseBool(normalized_value);
  } else if (normalized_key == "write_imu_rate_avp") {
    config.write_imu_rate_avp = ParseBool(normalized_value);
  } else if (normalized_key == "state_frequency_hz") {
    config.state_frequency_hz = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_time_offset_s") {
    config.gnss_time_offset_s = ParseDouble(normalized_value);
  } else if (normalized_key == "state_meas_sync_lower_bound_s") {
    config.state_meas_sync_lower_bound_s = ParseDouble(normalized_value);
  } else if (normalized_key == "state_meas_sync_upper_bound_s") {
    config.state_meas_sync_upper_bound_s = ParseDouble(normalized_value);
  } else if (normalized_key == "gravity_mps2") {
    config.gravity_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "imu_sigma_acc") {
    config.imu_sigma_acc = ParseDouble(normalized_value);
  } else if (normalized_key == "imu_sigma_gyro") {
    config.imu_sigma_gyro = ParseDouble(normalized_value);
  } else if (normalized_key == "integration_sigma") {
    config.integration_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_acc_sigma") {
    config.bias_acc_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_gyro_sigma") {
    config.bias_gyro_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_acc_prior_sigma") {
    config.bias_acc_prior_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_gyro_prior_sigma") {
    config.bias_gyro_prior_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "stationary_window_s") {
    config.stationary_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "stationary_acc_tolerance_mps2") {
    config.stationary_acc_tolerance_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "stationary_gyro_threshold_radps") {
    config.stationary_gyro_threshold_radps = ParseDouble(normalized_value);
  } else if (normalized_key == "prefer_imu_initial_yaw") {
    config.prefer_imu_initial_yaw = ParseBool(normalized_value);
  } else if (normalized_key == "imu_dual_vector_window_s") {
    config.imu_dual_vector_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "imu_dual_vector_min_sample_count") {
    config.imu_dual_vector_min_sample_count = ParseInt(normalized_value);
  } else if (normalized_key == "imu_dual_vector_min_cross_norm") {
    config.imu_dual_vector_min_cross_norm = ParseDouble(normalized_value);
  } else if (normalized_key == "yaw_min_distance_m") {
    config.yaw_min_distance_m = ParseDouble(normalized_value);
  } else if (normalized_key == "fallback_initial_yaw_rad") {
    config.fallback_initial_yaw_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "early_gnss_relaxation_duration_s") {
    config.early_gnss_relaxation_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "early_gnss_relaxation_scale") {
    config.early_gnss_relaxation_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "position_sigma_floor_m") {
    config.position_sigma_floor_m = ParseDouble(normalized_value);
  } else if (normalized_key == "position_sigma_ceiling_m") {
    config.position_sigma_ceiling_m = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_position_noise_model") {
    config.gnss_position_noise_model = ParseNoiseModel(normalized_value);
  } else if (normalized_key == "gnss_position_robust_param") {
    config.gnss_position_robust_param = ParseDouble(normalized_value);
  } else if (normalized_key == "rtkfix_scale") {
    config.rtkfix_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "rtkfloat_scale") {
    config.rtkfloat_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "single_scale") {
    config.single_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "required_best_sol_status_code") {
    config.required_best_sol_status_code = ParseInt(normalized_value);
  } else if (normalized_key == "drop_no_solution") {
    config.drop_no_solution = ParseBool(normalized_value);
  } else if (normalized_key == "drop_nonfinite_sigma") {
    config.drop_nonfinite_sigma = ParseBool(normalized_value);
  } else if (normalized_key == "initial_position_sigma_m") {
    config.initial_position_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_roll_pitch_sigma_rad") {
    config.initial_roll_pitch_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_yaw_sigma_rad") {
    config.initial_yaw_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_velocity_sigma_mps") {
    config.initial_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "lm_lambda_initial") {
    config.lm_lambda_initial = ParseDouble(normalized_value);
  } else if (normalized_key == "lm_max_iterations") {
    config.lm_max_iterations = ParseInt(normalized_value);
  } else {
    throw std::runtime_error("unknown config key: " + normalized_key);
  }
}

OfflineRunnerConfig LoadConfigFile(const std::string_view config_path, const OfflineRunnerConfig &base_config) {
  OfflineRunnerConfig config = base_config;
  if (config_path.empty()) {
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
    return config;
  }

  std::ifstream input_stream{std::string(config_path)};
  if (!input_stream.is_open()) {
    throw std::runtime_error("failed to open config file: " + std::string(config_path));
  }

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input_stream, line)) {
    ++line_number;
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }
    line = Trim(std::move(line));
    if (line.empty()) {
      continue;
    }

    const auto delimiter_pos = line.find('=');
    if (delimiter_pos == std::string::npos) {
      throw std::runtime_error("invalid config line " + std::to_string(line_number) + ": missing '='");
    }

    const auto key = Trim(line.substr(0, delimiter_pos));
    const auto value = Trim(line.substr(delimiter_pos + 1));
    OverrideConfigField(config, key, value);
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

  return config;
}

std::string ConfigToString(const OfflineRunnerConfig &config) {
  std::ostringstream oss;
  oss << "imu_path=" << config.imu_path << '\n'
      << "gnss_path=" << config.gnss_path << '\n'
      << "output_dir=" << config.output_dir << '\n'
      << "enable_gnss=" << (config.enable_gnss ? "true" : "false") << '\n'
      << "enable_gp_interpolated_gnss=" << (config.enable_gp_interpolated_gnss ? "true" : "false") << '\n'
      << "verbose=" << (config.verbose ? "true" : "false") << '\n'
      << "write_debug_csv=" << (config.write_debug_csv ? "true" : "false") << '\n'
      << "write_imu_rate_avp=" << (config.write_imu_rate_avp ? "true" : "false") << '\n'
      << "state_frequency_hz=" << config.state_frequency_hz << '\n'
      << "gnss_time_offset_s=" << config.gnss_time_offset_s << '\n'
      << "state_meas_sync_lower_bound_s=" << config.state_meas_sync_lower_bound_s << '\n'
      << "state_meas_sync_upper_bound_s=" << config.state_meas_sync_upper_bound_s << '\n'
      << "gravity_mps2=" << config.gravity_mps2 << '\n'
      << "imu_sigma_acc=" << config.imu_sigma_acc << '\n'
      << "imu_sigma_gyro=" << config.imu_sigma_gyro << '\n'
      << "integration_sigma=" << config.integration_sigma << '\n'
      << "bias_acc_sigma=" << config.bias_acc_sigma << '\n'
      << "bias_gyro_sigma=" << config.bias_gyro_sigma << '\n'
      << "bias_acc_prior_sigma=" << config.bias_acc_prior_sigma << '\n'
      << "bias_gyro_prior_sigma=" << config.bias_gyro_prior_sigma << '\n'
      << "stationary_window_s=" << config.stationary_window_s << '\n'
      << "stationary_acc_tolerance_mps2=" << config.stationary_acc_tolerance_mps2 << '\n'
      << "stationary_gyro_threshold_radps=" << config.stationary_gyro_threshold_radps << '\n'
      << "prefer_imu_initial_yaw=" << (config.prefer_imu_initial_yaw ? "true" : "false") << '\n'
      << "imu_dual_vector_window_s=" << config.imu_dual_vector_window_s << '\n'
      << "imu_dual_vector_min_sample_count=" << config.imu_dual_vector_min_sample_count << '\n'
      << "imu_dual_vector_min_cross_norm=" << config.imu_dual_vector_min_cross_norm << '\n'
      << "yaw_min_distance_m=" << config.yaw_min_distance_m << '\n'
      << "fallback_initial_yaw_rad=" << config.fallback_initial_yaw_rad << '\n'
      << "early_gnss_relaxation_duration_s=" << config.early_gnss_relaxation_duration_s << '\n'
      << "early_gnss_relaxation_scale=" << config.early_gnss_relaxation_scale << '\n'
      << "position_sigma_floor_m=" << config.position_sigma_floor_m << '\n'
      << "position_sigma_ceiling_m=" << config.position_sigma_ceiling_m << '\n'
      << "gnss_position_noise_model=" << ToString(config.gnss_position_noise_model) << '\n'
      << "gnss_position_robust_param=" << config.gnss_position_robust_param << '\n'
      << "rtkfix_scale=" << config.rtkfix_scale << '\n'
      << "rtkfloat_scale=" << config.rtkfloat_scale << '\n'
      << "single_scale=" << config.single_scale << '\n'
      << "required_best_sol_status_code=" << config.required_best_sol_status_code << '\n'
      << "drop_no_solution=" << (config.drop_no_solution ? "true" : "false") << '\n'
      << "drop_nonfinite_sigma=" << (config.drop_nonfinite_sigma ? "true" : "false") << '\n'
      << "initial_position_sigma_m=" << config.initial_position_sigma_m << '\n'
      << "initial_roll_pitch_sigma_rad=" << config.initial_roll_pitch_sigma_rad << '\n'
      << "initial_yaw_sigma_rad=" << config.initial_yaw_sigma_rad << '\n'
      << "initial_velocity_sigma_mps=" << config.initial_velocity_sigma_mps << '\n'
      << "lm_lambda_initial=" << config.lm_lambda_initial << '\n'
      << "lm_max_iterations=" << config.lm_max_iterations << '\n';
  return oss.str();
}

}  // namespace offline_lc_minimal
