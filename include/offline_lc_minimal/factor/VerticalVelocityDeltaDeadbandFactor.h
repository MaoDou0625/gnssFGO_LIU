#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

inline double SignedDeadbandResidual(const double value, const double deadband) {
  if (value > deadband) {
    return value - deadband;
  }
  if (value < -deadband) {
    return value + deadband;
  }
  return 0.0;
}

class VerticalVelocityDeltaDeadbandFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  VerticalVelocityDeltaDeadbandFactor(
    gtsam::Key velocity_i_key,
    gtsam::Key velocity_j_key,
    double deadband_mps,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3>(
          model,
          velocity_i_key,
          velocity_j_key),
        deadband_mps_(deadband_mps) {
    if (!std::isfinite(deadband_mps_) || deadband_mps_ < 0.0) {
      throw std::invalid_argument(
        "VerticalVelocityDeltaDeadbandFactor requires a finite nonnegative deadband");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(
        new VerticalVelocityDeltaDeadbandFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity_i,
    const gtsam::Vector3 &velocity_j,
    boost::optional<gtsam::Matrix &> h_i = boost::none,
    boost::optional<gtsam::Matrix &> h_j = boost::none) const override {
    const double delta_vz_mps = velocity_j.z() - velocity_i.z();
    const bool outside_deadband = std::abs(delta_vz_mps) > deadband_mps_;
    if (h_i) {
      *h_i = gtsam::Matrix::Zero(1, 3);
      if (outside_deadband) {
        (*h_i)(0, 2) = -1.0;
      }
    }
    if (h_j) {
      *h_j = gtsam::Matrix::Zero(1, 3);
      if (outside_deadband) {
        (*h_j)(0, 2) = 1.0;
      }
    }
    return gtsam::Vector1(SignedDeadbandResidual(delta_vz_mps, deadband_mps_));
  }

 private:
  double deadband_mps_ = 0.0;
};

class VerticalVelocityDeadbandFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  VerticalVelocityDeadbandFactor(
    gtsam::Key velocity_key,
    double target_vz_mps,
    double deadband_mps,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(model, velocity_key),
        target_vz_mps_(target_vz_mps),
        deadband_mps_(deadband_mps) {
    if (!std::isfinite(target_vz_mps_) ||
        !std::isfinite(deadband_mps_) ||
        deadband_mps_ < 0.0) {
      throw std::invalid_argument(
        "VerticalVelocityDeadbandFactor requires finite target and nonnegative deadband");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(
        new VerticalVelocityDeadbandFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity,
    boost::optional<gtsam::Matrix &> h = boost::none) const override {
    const double centered_vz_mps = velocity.z() - target_vz_mps_;
    const bool outside_deadband = std::abs(centered_vz_mps) > deadband_mps_;
    if (h) {
      *h = gtsam::Matrix::Zero(1, 3);
      if (outside_deadband) {
        (*h)(0, 2) = 1.0;
      }
    }
    return gtsam::Vector1(SignedDeadbandResidual(centered_vz_mps, deadband_mps_));
  }

 private:
  double target_vz_mps_ = 0.0;
  double deadband_mps_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
