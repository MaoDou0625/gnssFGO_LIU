#pragma once

#include <stdexcept>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

namespace offline_lc_minimal::factor {

class VerticalVelocityMeanFactor final : public gtsam::NoiseModelFactor {
 public:
  VerticalVelocityMeanFactor(
    gtsam::Key boundary_velocity_key,
    std::vector<gtsam::Key> context_velocity_keys,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor(model, BuildKeys(boundary_velocity_key, context_velocity_keys)),
        context_key_count_(context_velocity_keys.size()) {
    if (context_key_count_ == 0U) {
      throw std::invalid_argument("VerticalVelocityMeanFactor requires at least one context key");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalVelocityMeanFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector unwhitenedError(
    const gtsam::Values &values,
    boost::optional<std::vector<gtsam::Matrix> &> h = boost::none) const override {
    const auto &boundary_velocity = values.at<gtsam::Vector3>(keys()[0]);
    double context_vz_sum_mps = 0.0;
    for (std::size_t key_index = 1; key_index < keys().size(); ++key_index) {
      context_vz_sum_mps += values.at<gtsam::Vector3>(keys()[key_index]).z();
    }
    const double inv_context_count = 1.0 / static_cast<double>(context_key_count_);
    if (h) {
      h->assign(keys().size(), gtsam::Matrix::Zero(1, 3));
      (*h)[0](0, 2) = 1.0;
      for (std::size_t key_index = 1; key_index < keys().size(); ++key_index) {
        (*h)[key_index](0, 2) = -inv_context_count;
      }
    }
    return gtsam::Vector1(boundary_velocity.z() - context_vz_sum_mps * inv_context_count);
  }

 private:
  static gtsam::KeyVector BuildKeys(
    gtsam::Key boundary_velocity_key,
    const std::vector<gtsam::Key> &context_velocity_keys) {
    gtsam::KeyVector keys;
    keys.reserve(context_velocity_keys.size() + 1U);
    keys.push_back(boundary_velocity_key);
    keys.insert(keys.end(), context_velocity_keys.begin(), context_velocity_keys.end());
    return keys;
  }

  std::size_t context_key_count_ = 0;
};

}  // namespace offline_lc_minimal::factor
