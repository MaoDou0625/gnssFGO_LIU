#pragma once

#include <stdexcept>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

namespace offline_lc_minimal::factor {

class VerticalVelocityContextMeanContinuityFactor final : public gtsam::NoiseModelFactor {
 public:
  VerticalVelocityContextMeanContinuityFactor(
    std::vector<gtsam::Key> pre_context_velocity_keys,
    std::vector<gtsam::Key> post_context_velocity_keys,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor(model, BuildKeys(pre_context_velocity_keys, post_context_velocity_keys)),
        pre_context_key_count_(pre_context_velocity_keys.size()),
        post_context_key_count_(post_context_velocity_keys.size()) {
    if (pre_context_key_count_ == 0U || post_context_key_count_ == 0U) {
      throw std::invalid_argument(
        "VerticalVelocityContextMeanContinuityFactor requires both pre and post context keys");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalVelocityContextMeanContinuityFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector unwhitenedError(
    const gtsam::Values &values,
    boost::optional<std::vector<gtsam::Matrix> &> h = boost::none) const override {
    double pre_vz_sum_mps = 0.0;
    for (std::size_t key_index = 0; key_index < pre_context_key_count_; ++key_index) {
      pre_vz_sum_mps += values.at<gtsam::Vector3>(keys()[key_index]).z();
    }
    double post_vz_sum_mps = 0.0;
    for (std::size_t key_index = pre_context_key_count_; key_index < keys().size(); ++key_index) {
      post_vz_sum_mps += values.at<gtsam::Vector3>(keys()[key_index]).z();
    }

    const double inv_pre_count = 1.0 / static_cast<double>(pre_context_key_count_);
    const double inv_post_count = 1.0 / static_cast<double>(post_context_key_count_);
    if (h) {
      h->assign(keys().size(), gtsam::Matrix::Zero(1, 3));
      for (std::size_t key_index = 0; key_index < pre_context_key_count_; ++key_index) {
        (*h)[key_index](0, 2) = -inv_pre_count;
      }
      for (std::size_t key_index = pre_context_key_count_; key_index < keys().size(); ++key_index) {
        (*h)[key_index](0, 2) = inv_post_count;
      }
    }
    return gtsam::Vector1(post_vz_sum_mps * inv_post_count - pre_vz_sum_mps * inv_pre_count);
  }

 private:
  static gtsam::KeyVector BuildKeys(
    const std::vector<gtsam::Key> &pre_context_velocity_keys,
    const std::vector<gtsam::Key> &post_context_velocity_keys) {
    gtsam::KeyVector keys;
    keys.reserve(pre_context_velocity_keys.size() + post_context_velocity_keys.size());
    keys.insert(keys.end(), pre_context_velocity_keys.begin(), pre_context_velocity_keys.end());
    keys.insert(keys.end(), post_context_velocity_keys.begin(), post_context_velocity_keys.end());
    return keys;
  }

  std::size_t pre_context_key_count_ = 0;
  std::size_t post_context_key_count_ = 0;
};

}  // namespace offline_lc_minimal::factor
