#pragma once

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/factor/VehicleVelocityNHCFactor.h"

namespace offline_lc_minimal::factor {

[[nodiscard]] inline double VehicleZVelocityMpsWithFixedForward(
  const BodyFrameAxesNav &axes,
  const VehicleMountLeakageModel &mount,
  const gtsam::Vector3 &nav_velocity,
  const double reference_body_x_mps,
  const gtsam::Vector3 &reference_nav_velocity) {
  if (!std::isfinite(reference_body_x_mps)) {
    throw std::invalid_argument("reference body-x velocity must be finite");
  }
  if (!reference_nav_velocity.allFinite()) {
    throw std::invalid_argument("reference navigation velocity must be finite");
  }
  const double reference_body_z_mps =
    BodyZRawVelocityMps(axes, reference_nav_velocity);
  const double optimized_up_delta_mps =
    nav_velocity.z() - reference_nav_velocity.z();
  return reference_body_z_mps +
         axes.body_z_axis_nav.z() * optimized_up_delta_mps -
         mount.k_zx_rad * reference_body_x_mps;
}

[[nodiscard]] inline gtsam::Matrix VehicleZFixedForwardJacobianWrtVelocity(
  const BodyFrameAxesNav &axes) {
  gtsam::Matrix jacobian = gtsam::Matrix::Zero(1, 3);
  jacobian(0, 2) = axes.body_z_axis_nav.z();
  return jacobian;
}

[[nodiscard]] inline gtsam::Matrix VehicleZFixedForwardJacobianWrtMount(
  const double reference_body_x_mps) {
  if (!std::isfinite(reference_body_x_mps)) {
    throw std::invalid_argument("reference body-x velocity must be finite");
  }
  gtsam::Matrix jacobian = gtsam::Matrix::Zero(1, 3);
  jacobian(0, 0) = -reference_body_x_mps;
  return jacobian;
}

class VehicleZVelocityZeroFixedForwardFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  VehicleZVelocityZeroFixedForwardFactor(
    gtsam::Key velocity_key,
    gtsam::Key mount_leakage_key,
    const BodyFrameAxesNav &body_axes_nav,
    double reference_body_x_mps,
    const gtsam::Vector3 &reference_nav_velocity,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3>(
          model,
          velocity_key,
          mount_leakage_key),
        body_axes_nav_(NormalizeBodyFrameAxesNav(body_axes_nav)),
        reference_body_x_mps_(reference_body_x_mps),
        reference_nav_velocity_(reference_nav_velocity) {
    if (!std::isfinite(reference_body_x_mps_)) {
      throw std::invalid_argument("reference body-x velocity must be finite");
    }
    if (!reference_nav_velocity_.allFinite()) {
      throw std::invalid_argument("reference navigation velocity must be finite");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VehicleZVelocityZeroFixedForwardFactor(*this)));
  }

  [[nodiscard]] const BodyFrameAxesNav &bodyAxesNav() const {
    return body_axes_nav_;
  }

  [[nodiscard]] double referenceBodyXMps() const {
    return reference_body_x_mps_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &nav_velocity,
    const gtsam::Vector3 &mount_leakage,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none,
    boost::optional<gtsam::Matrix &> h_mount = boost::none) const override {
    const VehicleMountLeakageModel mount =
      VehicleMountLeakageModelFromVector(mount_leakage);
    if (h_velocity) {
      *h_velocity = VehicleZFixedForwardJacobianWrtVelocity(body_axes_nav_);
    }
    if (h_mount) {
      *h_mount = VehicleZFixedForwardJacobianWrtMount(reference_body_x_mps_);
    }
    return gtsam::Vector1(
      VehicleZVelocityMpsWithFixedForward(
        body_axes_nav_,
        mount,
        nav_velocity,
        reference_body_x_mps_,
        reference_nav_velocity_));
  }

 private:
  BodyFrameAxesNav body_axes_nav_;
  double reference_body_x_mps_ = 0.0;
  gtsam::Vector3 reference_nav_velocity_ = gtsam::Vector3::Zero();
};

class VehicleZWindowDisplacementZeroFixedForwardFactor final
    : public gtsam::NoiseModelFactor {
 public:
  VehicleZWindowDisplacementZeroFixedForwardFactor(
    const std::vector<gtsam::Key> &velocity_keys,
    gtsam::Key mount_leakage_key,
    const std::vector<BodyFrameAxesNav> &body_axes_nav,
    std::vector<double> reference_body_x_mps,
    std::vector<gtsam::Vector3> reference_nav_velocities,
    std::vector<double> state_times_s,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor(model, BuildKeys(velocity_keys, mount_leakage_key)),
        state_count_(velocity_keys.size()),
        body_axes_nav_(NormalizeAxes(body_axes_nav, velocity_keys.size())),
        reference_body_x_mps_(std::move(reference_body_x_mps)),
        reference_nav_velocities_(std::move(reference_nav_velocities)),
        state_times_s_(std::move(state_times_s)) {
    if (state_count_ < 2U) {
      throw std::invalid_argument(
        "VehicleZWindowDisplacementZeroFixedForwardFactor requires at least two states");
    }
    if (reference_body_x_mps_.size() != state_count_ ||
        reference_nav_velocities_.size() != state_count_ ||
        state_times_s_.size() != state_count_) {
      throw std::invalid_argument(
        "VehicleZWindowDisplacementZeroFixedForwardFactor input counts must match");
    }
    for (const double reference_body_x_mps : reference_body_x_mps_) {
      if (!std::isfinite(reference_body_x_mps)) {
        throw std::invalid_argument("reference body-x velocities must be finite");
      }
    }
    for (const auto &reference_nav_velocity : reference_nav_velocities_) {
      if (!reference_nav_velocity.allFinite()) {
        throw std::invalid_argument("reference navigation velocities must be finite");
      }
    }
    for (std::size_t index = 1U; index < state_times_s_.size(); ++index) {
      if (!std::isfinite(state_times_s_[index - 1U]) ||
          !std::isfinite(state_times_s_[index]) ||
          state_times_s_[index] <= state_times_s_[index - 1U]) {
        throw std::invalid_argument(
          "VehicleZWindowDisplacementZeroFixedForwardFactor requires increasing finite times");
      }
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(
        new VehicleZWindowDisplacementZeroFixedForwardFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector unwhitenedError(
    const gtsam::Values &values,
    boost::optional<std::vector<gtsam::Matrix> &> h = boost::none) const override {
    const auto mount_leakage =
      values.at<gtsam::Vector3>(keys()[state_count_]);
    const VehicleMountLeakageModel mount =
      VehicleMountLeakageModelFromVector(mount_leakage);
    if (h) {
      h->resize(keys().size());
      gtsam::Matrix mount_jacobian = gtsam::Matrix::Zero(1, 3);
      for (std::size_t state_offset = 0U; state_offset < state_count_; ++state_offset) {
        const double weight_s = IntegrationWeightS(state_offset);
        (*h)[state_offset] =
          weight_s * VehicleZFixedForwardJacobianWrtVelocity(
                       body_axes_nav_[state_offset]);
        mount_jacobian +=
          weight_s * VehicleZFixedForwardJacobianWrtMount(
                       reference_body_x_mps_[state_offset]);
      }
      (*h)[state_count_] = mount_jacobian;
    }
    return Evaluate(values, mount);
  }

 private:
  static gtsam::KeyVector BuildKeys(
    const std::vector<gtsam::Key> &velocity_keys,
    gtsam::Key mount_leakage_key) {
    gtsam::KeyVector keys;
    keys.reserve(velocity_keys.size() + 1U);
    for (const gtsam::Key key : velocity_keys) {
      keys.push_back(key);
    }
    keys.push_back(mount_leakage_key);
    return keys;
  }

  static std::vector<BodyFrameAxesNav> NormalizeAxes(
    const std::vector<BodyFrameAxesNav> &body_axes_nav,
    const std::size_t expected_count) {
    if (body_axes_nav.size() != expected_count) {
      throw std::invalid_argument(
        "VehicleZWindowDisplacementZeroFixedForwardFactor axis and velocity key counts must match");
    }
    std::vector<BodyFrameAxesNav> normalized_axes;
    normalized_axes.reserve(body_axes_nav.size());
    for (const auto &axes : body_axes_nav) {
      normalized_axes.push_back(NormalizeBodyFrameAxesNav(axes));
    }
    return normalized_axes;
  }

  [[nodiscard]] gtsam::Vector1 Evaluate(
    const gtsam::Values &values,
    const VehicleMountLeakageModel &mount) const {
    double displacement_m = 0.0;
    for (std::size_t index = 1U; index < state_count_; ++index) {
      const double dt_s = state_times_s_[index] - state_times_s_[index - 1U];
      const double prev_velocity_mps = VehicleZVelocityAt(values, mount, index - 1U);
      const double current_velocity_mps = VehicleZVelocityAt(values, mount, index);
      displacement_m += 0.5 * dt_s * (prev_velocity_mps + current_velocity_mps);
    }
    return gtsam::Vector1(displacement_m);
  }

  [[nodiscard]] double VehicleZVelocityAt(
    const gtsam::Values &values,
    const VehicleMountLeakageModel &mount,
    const std::size_t state_offset) const {
    const auto &velocity = values.at<gtsam::Vector3>(keys()[state_offset]);
    return VehicleZVelocityMpsWithFixedForward(
      body_axes_nav_[state_offset],
      mount,
      velocity,
      reference_body_x_mps_[state_offset],
      reference_nav_velocities_[state_offset]);
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
  std::vector<BodyFrameAxesNav> body_axes_nav_;
  std::vector<double> reference_body_x_mps_;
  std::vector<gtsam::Vector3> reference_nav_velocities_;
  std::vector<double> state_times_s_;
};

}  // namespace offline_lc_minimal::factor
