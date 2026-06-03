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

#include "offline_lc_minimal/factor/BodyZLeakageCorrectedVelocityFactor.h"

namespace offline_lc_minimal::factor {

struct VehicleMountLeakageModel {
  double k_zx_rad = 0.0;
  double k_zy_rad = 0.0;
  double k_yx_rad = 0.0;
};

[[nodiscard]] inline VehicleMountLeakageModel VehicleMountLeakageModelFromVector(
  const gtsam::Vector3 &mount_leakage) {
  if (!mount_leakage.allFinite()) {
    throw std::invalid_argument("vehicle mount leakage vector must be finite");
  }
  return VehicleMountLeakageModel{
    mount_leakage.x(),
    mount_leakage.y(),
    mount_leakage.z()};
}

[[nodiscard]] inline gtsam::Vector3 VehicleMountLeakageVector(
  const VehicleMountLeakageModel &model) {
  if (!std::isfinite(model.k_zx_rad) ||
      !std::isfinite(model.k_zy_rad) ||
      !std::isfinite(model.k_yx_rad)) {
    throw std::invalid_argument("vehicle mount leakage model must be finite");
  }
  return (gtsam::Vector3() << model.k_zx_rad, model.k_zy_rad, model.k_yx_rad).finished();
}

[[nodiscard]] inline double VehicleYVelocityMps(
  const BodyFrameAxesNav &axes,
  const VehicleMountLeakageModel &mount,
  const gtsam::Vector3 &nav_velocity) {
  return BodyYVelocityMps(axes, nav_velocity) -
         mount.k_yx_rad * BodyXVelocityMps(axes, nav_velocity);
}

[[nodiscard]] inline double VehicleZVelocityMps(
  const BodyFrameAxesNav &axes,
  const VehicleMountLeakageModel &mount,
  const gtsam::Vector3 &nav_velocity) {
  return BodyZRawVelocityMps(axes, nav_velocity) -
         mount.k_zx_rad * BodyXVelocityMps(axes, nav_velocity);
}

[[nodiscard]] inline gtsam::Matrix VehicleYVelocityJacobianWrtVelocity(
  const BodyFrameAxesNav &axes,
  const VehicleMountLeakageModel &mount) {
  gtsam::Matrix jacobian(1, 3);
  jacobian.row(0) =
    axes.body_y_axis_nav.transpose() -
    mount.k_yx_rad * axes.body_x_axis_nav.transpose();
  return jacobian;
}

[[nodiscard]] inline gtsam::Matrix VehicleZVelocityJacobianWrtVelocity(
  const BodyFrameAxesNav &axes,
  const VehicleMountLeakageModel &mount) {
  gtsam::Matrix jacobian(1, 3);
  jacobian.row(0) =
    axes.body_z_axis_nav.transpose() -
    mount.k_zx_rad * axes.body_x_axis_nav.transpose();
  return jacobian;
}

[[nodiscard]] inline gtsam::Matrix VehicleYVelocityJacobianWrtMount(
  const BodyFrameAxesNav &axes,
  const gtsam::Vector3 &nav_velocity) {
  gtsam::Matrix jacobian = gtsam::Matrix::Zero(1, 3);
  jacobian(0, 2) = -BodyXVelocityMps(axes, nav_velocity);
  return jacobian;
}

[[nodiscard]] inline gtsam::Matrix VehicleZVelocityJacobianWrtMount(
  const BodyFrameAxesNav &axes,
  const gtsam::Vector3 &nav_velocity) {
  gtsam::Matrix jacobian = gtsam::Matrix::Zero(1, 3);
  jacobian(0, 0) = -BodyXVelocityMps(axes, nav_velocity);
  return jacobian;
}

class VehicleYVelocityZeroFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  VehicleYVelocityZeroFactor(
    gtsam::Key velocity_key,
    gtsam::Key mount_leakage_key,
    const BodyFrameAxesNav &body_axes_nav,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3>(
          model,
          velocity_key,
          mount_leakage_key),
        body_axes_nav_(NormalizeBodyFrameAxesNav(body_axes_nav)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VehicleYVelocityZeroFactor(*this)));
  }

  [[nodiscard]] const BodyFrameAxesNav &bodyAxesNav() const {
    return body_axes_nav_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &nav_velocity,
    const gtsam::Vector3 &mount_leakage,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none,
    boost::optional<gtsam::Matrix &> h_mount = boost::none) const override {
    const VehicleMountLeakageModel mount =
      VehicleMountLeakageModelFromVector(mount_leakage);
    if (h_velocity) {
      *h_velocity = VehicleYVelocityJacobianWrtVelocity(body_axes_nav_, mount);
    }
    if (h_mount) {
      *h_mount = VehicleYVelocityJacobianWrtMount(body_axes_nav_, nav_velocity);
    }
    return gtsam::Vector1(VehicleYVelocityMps(body_axes_nav_, mount, nav_velocity));
  }

 private:
  BodyFrameAxesNav body_axes_nav_;
};

class VehicleZVelocityZeroFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  VehicleZVelocityZeroFactor(
    gtsam::Key velocity_key,
    gtsam::Key mount_leakage_key,
    const BodyFrameAxesNav &body_axes_nav,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3>(
          model,
          velocity_key,
          mount_leakage_key),
        body_axes_nav_(NormalizeBodyFrameAxesNav(body_axes_nav)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VehicleZVelocityZeroFactor(*this)));
  }

  [[nodiscard]] const BodyFrameAxesNav &bodyAxesNav() const {
    return body_axes_nav_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &nav_velocity,
    const gtsam::Vector3 &mount_leakage,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none,
    boost::optional<gtsam::Matrix &> h_mount = boost::none) const override {
    const VehicleMountLeakageModel mount =
      VehicleMountLeakageModelFromVector(mount_leakage);
    if (h_velocity) {
      *h_velocity = VehicleZVelocityJacobianWrtVelocity(body_axes_nav_, mount);
    }
    if (h_mount) {
      *h_mount = VehicleZVelocityJacobianWrtMount(body_axes_nav_, nav_velocity);
    }
    return gtsam::Vector1(VehicleZVelocityMps(body_axes_nav_, mount, nav_velocity));
  }

 private:
  BodyFrameAxesNav body_axes_nav_;
};

enum class VehicleNHCComponent {
  kY,
  kZ
};

class VehicleWindowDisplacementZeroFactor final : public gtsam::NoiseModelFactor {
 public:
  VehicleWindowDisplacementZeroFactor(
    const std::vector<gtsam::Key> &velocity_keys,
    gtsam::Key mount_leakage_key,
    const std::vector<BodyFrameAxesNav> &body_axes_nav,
    std::vector<double> state_times_s,
    VehicleNHCComponent component,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor(model, BuildKeys(velocity_keys, mount_leakage_key)),
        state_count_(velocity_keys.size()),
        body_axes_nav_(NormalizeAxes(body_axes_nav, velocity_keys.size())),
        state_times_s_(std::move(state_times_s)),
        component_(component) {
    if (state_count_ < 2U) {
      throw std::invalid_argument("VehicleWindowDisplacementZeroFactor requires at least two states");
    }
    if (state_times_s_.size() != state_count_) {
      throw std::invalid_argument(
        "VehicleWindowDisplacementZeroFactor time and velocity key counts must match");
    }
    for (std::size_t index = 1U; index < state_times_s_.size(); ++index) {
      if (!std::isfinite(state_times_s_[index - 1U]) ||
          !std::isfinite(state_times_s_[index]) ||
          state_times_s_[index] <= state_times_s_[index - 1U]) {
        throw std::invalid_argument(
          "VehicleWindowDisplacementZeroFactor requires increasing finite times");
      }
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VehicleWindowDisplacementZeroFactor(*this)));
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
        const auto velocity = values.at<gtsam::Vector3>(keys()[state_offset]);
        if (component_ == VehicleNHCComponent::kY) {
          (*h)[state_offset] =
            weight_s * VehicleYVelocityJacobianWrtVelocity(
                         body_axes_nav_[state_offset],
                         mount);
          mount_jacobian +=
            weight_s * VehicleYVelocityJacobianWrtMount(
                         body_axes_nav_[state_offset],
                         velocity);
        } else {
          (*h)[state_offset] =
            weight_s * VehicleZVelocityJacobianWrtVelocity(
                         body_axes_nav_[state_offset],
                         mount);
          mount_jacobian +=
            weight_s * VehicleZVelocityJacobianWrtMount(
                         body_axes_nav_[state_offset],
                         velocity);
        }
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
        "VehicleWindowDisplacementZeroFactor axis and velocity key counts must match");
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
      const double prev_velocity_mps = VehicleVelocityAt(values, mount, index - 1U);
      const double current_velocity_mps = VehicleVelocityAt(values, mount, index);
      displacement_m += 0.5 * dt_s * (prev_velocity_mps + current_velocity_mps);
    }
    return gtsam::Vector1(displacement_m);
  }

  [[nodiscard]] double VehicleVelocityAt(
    const gtsam::Values &values,
    const VehicleMountLeakageModel &mount,
    const std::size_t state_offset) const {
    const auto &velocity = values.at<gtsam::Vector3>(keys()[state_offset]);
    if (component_ == VehicleNHCComponent::kY) {
      return VehicleYVelocityMps(body_axes_nav_[state_offset], mount, velocity);
    }
    return VehicleZVelocityMps(body_axes_nav_[state_offset], mount, velocity);
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
  std::vector<double> state_times_s_;
  VehicleNHCComponent component_ = VehicleNHCComponent::kZ;
};

}  // namespace offline_lc_minimal::factor
