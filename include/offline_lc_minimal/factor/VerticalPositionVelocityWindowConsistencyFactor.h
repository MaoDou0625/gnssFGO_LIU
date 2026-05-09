#pragma once

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

namespace offline_lc_minimal::factor {

class VerticalPositionVelocityWindowConsistencyFactor final : public gtsam::NoiseModelFactor {
 public:
  VerticalPositionVelocityWindowConsistencyFactor(
    gtsam::Key pose_start_key,
    gtsam::Key pose_end_key,
    std::vector<gtsam::Key> velocity_keys,
    std::vector<double> state_times_s,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor(model, BuildKeys(pose_start_key, pose_end_key, velocity_keys)),
        velocity_key_count_(velocity_keys.size()),
        state_times_s_(std::move(state_times_s)) {
    if (velocity_key_count_ < 2U) {
      throw std::invalid_argument("VerticalPositionVelocityWindowConsistencyFactor requires at least two velocities");
    }
    if (state_times_s_.size() != velocity_key_count_) {
      throw std::invalid_argument(
        "VerticalPositionVelocityWindowConsistencyFactor time and velocity key counts must match");
    }
    for (std::size_t index = 1U; index < state_times_s_.size(); ++index) {
      if (!std::isfinite(state_times_s_[index - 1U]) ||
          !std::isfinite(state_times_s_[index]) ||
          state_times_s_[index] <= state_times_s_[index - 1U]) {
        throw std::invalid_argument(
          "VerticalPositionVelocityWindowConsistencyFactor requires increasing finite times");
      }
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalPositionVelocityWindowConsistencyFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector unwhitenedError(
    const gtsam::Values &values,
    boost::optional<std::vector<gtsam::Matrix> &> h = boost::none) const override {
    if (h) {
      h->assign(keys().size(), gtsam::Matrix());
      (*h)[0] = PoseZJacobian(values.at<gtsam::Pose3>(keys()[0]), -1.0);
      (*h)[1] = PoseZJacobian(values.at<gtsam::Pose3>(keys()[1]), 1.0);
      for (std::size_t velocity_offset = 0U; velocity_offset < velocity_key_count_; ++velocity_offset) {
        gtsam::Matrix velocity_jacobian = gtsam::Matrix::Zero(1, 3);
        velocity_jacobian(0, 2) = -IntegrationWeightS(velocity_offset);
        (*h)[velocity_offset + 2U] = velocity_jacobian;
      }
    }
    return Evaluate(values);
  }

 private:
  static gtsam::KeyVector BuildKeys(
    const gtsam::Key pose_start_key,
    const gtsam::Key pose_end_key,
    const std::vector<gtsam::Key> &velocity_keys) {
    gtsam::KeyVector keys;
    keys.reserve(velocity_keys.size() + 2U);
    keys.push_back(pose_start_key);
    keys.push_back(pose_end_key);
    keys.insert(keys.end(), velocity_keys.begin(), velocity_keys.end());
    return keys;
  }

  static gtsam::Matrix PoseZJacobian(const gtsam::Pose3 &pose, const double sign) {
    const auto z_function = [sign](const gtsam::Pose3 &candidate_pose) {
      return gtsam::Vector1(sign * candidate_pose.translation().z());
    };
    return gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(z_function, pose);
  }

  [[nodiscard]] gtsam::Vector1 Evaluate(const gtsam::Values &values) const {
    const auto &pose_start = values.at<gtsam::Pose3>(keys()[0]);
    const auto &pose_end = values.at<gtsam::Pose3>(keys()[1]);
    double trapezoid_delta_z_m = 0.0;
    for (std::size_t index = 1U; index < velocity_key_count_; ++index) {
      const double dt_s = state_times_s_[index] - state_times_s_[index - 1U];
      const double prev_vz_mps = values.at<gtsam::Vector3>(keys()[index + 1U]).z();
      const double current_vz_mps = values.at<gtsam::Vector3>(keys()[index + 2U]).z();
      trapezoid_delta_z_m += 0.5 * dt_s * (prev_vz_mps + current_vz_mps);
    }
    const double delta_z_m = pose_end.translation().z() - pose_start.translation().z();
    return gtsam::Vector1(delta_z_m - trapezoid_delta_z_m);
  }

  [[nodiscard]] double IntegrationWeightS(const std::size_t velocity_offset) const {
    double weight_s = 0.0;
    if (velocity_offset > 0U) {
      weight_s += 0.5 * (state_times_s_[velocity_offset] - state_times_s_[velocity_offset - 1U]);
    }
    if (velocity_offset + 1U < velocity_key_count_) {
      weight_s += 0.5 * (state_times_s_[velocity_offset + 1U] - state_times_s_[velocity_offset]);
    }
    return weight_s;
  }

  std::size_t velocity_key_count_ = 0U;
  std::vector<double> state_times_s_;
};

}  // namespace offline_lc_minimal::factor
