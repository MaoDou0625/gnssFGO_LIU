#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/factor/BodyZVelocityZeroFactor.h"

namespace offline_lc_minimal::factor {

class BodyZWindowDisplacementZeroFactor final : public gtsam::NoiseModelFactor {
 public:
  BodyZWindowDisplacementZeroFactor(
    const std::vector<gtsam::Key> &pose_keys,
    const std::vector<gtsam::Key> &velocity_keys,
    std::vector<double> state_times_s,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor(model, BuildKeys(pose_keys, velocity_keys)),
        state_count_(pose_keys.size()),
        state_times_s_(state_times_s) {
    if (state_count_ < 2U) {
      throw std::invalid_argument("BodyZWindowDisplacementZeroFactor requires at least two states");
    }
    for (std::size_t index = 1U; index < state_times_s_.size(); ++index) {
      if (!std::isfinite(state_times_s_[index - 1U]) ||
          !std::isfinite(state_times_s_[index]) ||
          state_times_s_[index] <= state_times_s_[index - 1U]) {
        throw std::invalid_argument("BodyZWindowDisplacementZeroFactor requires increasing finite times");
      }
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new BodyZWindowDisplacementZeroFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector unwhitenedError(
    const gtsam::Values &values,
    boost::optional<std::vector<gtsam::Matrix> &> h = boost::none) const override {
    if (h) {
      h->resize(keys().size());
      for (std::size_t state_offset = 0U; state_offset < state_count_; ++state_offset) {
        const auto &pose = values.at<gtsam::Pose3>(keys()[2U * state_offset]);
        const auto &velocity = values.at<gtsam::Vector3>(keys()[2U * state_offset + 1U]);
        const double weight_s = IntegrationWeightS(state_offset);
        (*h)[2U * state_offset] = weight_s * BodyZVelocityPoseJacobian(pose, velocity);
        (*h)[2U * state_offset + 1U] = weight_s * BodyZVelocityVelocityJacobian(pose);
      }
    }
    return Evaluate(values);
  }

 private:
  static gtsam::KeyVector BuildKeys(
    const std::vector<gtsam::Key> &pose_keys,
    const std::vector<gtsam::Key> &velocity_keys) {
    if (pose_keys.size() != velocity_keys.size()) {
      throw std::invalid_argument("BodyZWindowDisplacementZeroFactor pose and velocity key counts must match");
    }
    gtsam::KeyVector keys;
    keys.reserve(pose_keys.size() + velocity_keys.size());
    for (std::size_t index = 0U; index < pose_keys.size(); ++index) {
      keys.push_back(pose_keys[index]);
      keys.push_back(velocity_keys[index]);
    }
    return keys;
  }

  [[nodiscard]] gtsam::Vector1 Evaluate(const gtsam::Values &values) const {
    double displacement_m = 0.0;
    for (std::size_t index = 1U; index < state_count_; ++index) {
      const double dt_s = state_times_s_[index] - state_times_s_[index - 1U];
      const double prev_vz_mps = BodyZVelocityAt(values, index - 1U);
      const double current_vz_mps = BodyZVelocityAt(values, index);
      displacement_m += 0.5 * dt_s * (prev_vz_mps + current_vz_mps);
    }
    return gtsam::Vector1(displacement_m);
  }

  [[nodiscard]] double BodyZVelocityAt(
    const gtsam::Values &values,
    const std::size_t state_offset) const {
    const auto &pose = values.at<gtsam::Pose3>(keys()[2U * state_offset]);
    const auto &velocity = values.at<gtsam::Vector3>(keys()[2U * state_offset + 1U]);
    return BodyZVelocityMps(pose, velocity);
  }

  [[nodiscard]] double IntegrationWeightS(const std::size_t state_offset) const {
    double weight_s = 0.0;
    if (state_offset > 0U) {
      weight_s += 0.5 * (state_times_s_[state_offset] - state_times_s_[state_offset - 1U]);
    }
    if (state_offset + 1U < state_count_) {
      weight_s += 0.5 * (state_times_s_[state_offset + 1U] - state_times_s_[state_offset]);
    }
    return weight_s;
  }

  std::size_t state_count_ = 0U;
  std::vector<double> state_times_s_;
};

}  // namespace offline_lc_minimal::factor
