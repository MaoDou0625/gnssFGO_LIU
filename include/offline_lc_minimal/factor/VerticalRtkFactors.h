#pragma once

#include <algorithm>
#include <cmath>

#include <boost/bind/bind.hpp>
#include <boost/pointer_cast.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal::factor {

inline double VerticalEnvelopeResidual(const double raw_residual_m, const double half_width_m) {
  if (raw_residual_m > half_width_m) {
    return raw_residual_m - half_width_m;
  }
  if (raw_residual_m < -half_width_m) {
    return raw_residual_m + half_width_m;
  }
  return 0.0;
}

inline double VerticalEnvelopeResidualSlope(const double raw_residual_m, const double half_width_m) {
  return std::abs(raw_residual_m) > half_width_m ? 1.0 : 0.0;
}

inline double VerticalEnvelopeCenterResidual(
  const double raw_residual_m,
  const double half_width_m,
  const double deadband_m) {
  const double signed_clamped_m = std::clamp(raw_residual_m, -half_width_m, half_width_m);
  const double magnitude_m = std::max(std::abs(signed_clamped_m) - deadband_m, 0.0);
  if (magnitude_m == 0.0) {
    return 0.0;
  }
  return std::copysign(magnitude_m, signed_clamped_m);
}

inline double VerticalEnvelopeCenterResidualSlope(
  const double raw_residual_m,
  const double half_width_m,
  const double deadband_m) {
  if (std::abs(raw_residual_m) >= half_width_m) {
    return 0.0;
  }
  return std::abs(raw_residual_m) > deadband_m ? 1.0 : 0.0;
}

class VerticalPositionFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  VerticalPositionFactor(gtsam::Key pose_key, double measured_up,
                         const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        measured_up_(measured_up) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalPositionFactor(*this)));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              boost::optional<gtsam::Matrix&> H = boost::none) const override {
    if (H) {
      using boost::placeholders::_1;
      *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
        boost::bind(&VerticalPositionFactor::EvaluateErrorNoJacobians, this, _1),
        pose,
        1e-5);
    }
    return EvaluateErrorNoJacobians(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector1 EvaluateErrorNoJacobians(const gtsam::Pose3& pose) const {
    return gtsam::Vector1(pose.translation().z() - measured_up_);
  }

  double measured_up_ = 0.0;
};

class VerticalEnvelopeFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  VerticalEnvelopeFactor(
    gtsam::Key pose_key,
    double measured_up,
    double half_width_m,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        measured_up_(measured_up),
        half_width_m_(half_width_m) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalEnvelopeFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override {
    if (H) {
      using boost::placeholders::_1;
      *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
        boost::bind(&VerticalEnvelopeFactor::EvaluateErrorNoJacobians, this, _1),
        pose,
        1e-5);
    }
    return EvaluateErrorNoJacobians(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector1 EvaluateErrorNoJacobians(const gtsam::Pose3 &pose) const {
    const double raw_residual_m = pose.translation().z() - measured_up_;
    return gtsam::Vector1(VerticalEnvelopeResidual(raw_residual_m, half_width_m_));
  }

  double measured_up_ = 0.0;
  double half_width_m_ = 0.0;
};

class VerticalEnvelopeCenterPullFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  VerticalEnvelopeCenterPullFactor(
    gtsam::Key pose_key,
    double measured_up,
    double half_width_m,
    double deadband_m,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        measured_up_(measured_up),
        half_width_m_(half_width_m),
        deadband_m_(deadband_m) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalEnvelopeCenterPullFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override {
    if (H) {
      using boost::placeholders::_1;
      *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
        boost::bind(&VerticalEnvelopeCenterPullFactor::EvaluateErrorNoJacobians, this, _1),
        pose,
        1e-5);
    }
    return EvaluateErrorNoJacobians(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector1 EvaluateErrorNoJacobians(const gtsam::Pose3 &pose) const {
    const double raw_residual_m = pose.translation().z() - measured_up_;
    return gtsam::Vector1(VerticalEnvelopeCenterResidual(raw_residual_m, half_width_m_, deadband_m_));
  }

  double measured_up_ = 0.0;
  double half_width_m_ = 0.0;
  double deadband_m_ = 0.0;
};

class RtkVerticalReferenceMeasurementFactor final : public gtsam::NoiseModelFactor1<double> {
 public:
  RtkVerticalReferenceMeasurementFactor(
    gtsam::Key reference_key,
    double measured_up,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<double>(model, reference_key),
        measured_up_(measured_up) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new RtkVerticalReferenceMeasurementFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const double &reference_up_m,
    boost::optional<gtsam::Matrix &> H = boost::none) const override {
    if (H) {
      *H = gtsam::Matrix::Identity(1, 1);
    }
    return gtsam::Vector1(reference_up_m - measured_up_);
  }

 private:
  double measured_up_ = 0.0;
};

class RtkVerticalReferenceSmoothnessFactor final : public gtsam::NoiseModelFactor2<double, double> {
 public:
  RtkVerticalReferenceSmoothnessFactor(
    gtsam::Key reference_i_key,
    gtsam::Key reference_j_key,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<double, double>(model, reference_i_key, reference_j_key) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new RtkVerticalReferenceSmoothnessFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const double &reference_i_up_m,
    const double &reference_j_up_m,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
    if (H1) {
      *H1 = -gtsam::Matrix::Identity(1, 1);
    }
    if (H2) {
      *H2 = gtsam::Matrix::Identity(1, 1);
    }
    return gtsam::Vector1(reference_j_up_m - reference_i_up_m);
  }
};

class VerticalEnvelopeLatentReferenceFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, double> {
 public:
  VerticalEnvelopeLatentReferenceFactor(
    gtsam::Key pose_key,
    gtsam::Key reference_key,
    double half_width_m,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, double>(model, pose_key, reference_key),
        half_width_m_(half_width_m) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalEnvelopeLatentReferenceFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    const double &reference_up_m,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
    const double raw_residual_m = pose.translation().z() - reference_up_m;
    const double slope = VerticalEnvelopeResidualSlope(raw_residual_m, half_width_m_);
    if (H1) {
      const auto z_function = [slope](const gtsam::Pose3 &candidate_pose) {
        return gtsam::Vector1(slope * candidate_pose.translation().z());
      };
      *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
        z_function,
        pose,
        1e-5);
    }
    if (H2) {
      *H2 = -slope * gtsam::Matrix::Identity(1, 1);
    }
    return gtsam::Vector1(VerticalEnvelopeResidual(raw_residual_m, half_width_m_));
  }

 private:
  double half_width_m_ = 0.0;
};

class VerticalEnvelopeLatentCenterPullFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, double> {
 public:
  VerticalEnvelopeLatentCenterPullFactor(
    gtsam::Key pose_key,
    gtsam::Key reference_key,
    double half_width_m,
    double deadband_m,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, double>(model, pose_key, reference_key),
        half_width_m_(half_width_m),
        deadband_m_(deadband_m) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalEnvelopeLatentCenterPullFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    const double &reference_up_m,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override {
    const double raw_residual_m = pose.translation().z() - reference_up_m;
    const double slope =
      VerticalEnvelopeCenterResidualSlope(raw_residual_m, half_width_m_, deadband_m_);
    if (H1) {
      const auto z_function = [slope](const gtsam::Pose3 &candidate_pose) {
        return gtsam::Vector1(slope * candidate_pose.translation().z());
      };
      *H1 = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
        z_function,
        pose,
        1e-5);
    }
    if (H2) {
      *H2 = -slope * gtsam::Matrix::Identity(1, 1);
    }
    return gtsam::Vector1(
      VerticalEnvelopeCenterResidual(raw_residual_m, half_width_m_, deadband_m_));
  }

 private:
  double half_width_m_ = 0.0;
  double deadband_m_ = 0.0;
};

class GPInterpolatedVerticalPositionFactor final
    : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
 public:
  GPInterpolatedVerticalPositionFactor(gtsam::Key pose_i_key, gtsam::Key vel_i_key,
                                       gtsam::Key omega_i_key, gtsam::Key pose_j_key,
                                       gtsam::Key vel_j_key, gtsam::Key omega_j_key,
                                       double measured_up,
                                       const offline_lc_minimal::gp::GPWNOJInterpolator& interpolator,
                                       const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                 gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            model, pose_i_key, vel_i_key, omega_i_key, pose_j_key, vel_j_key, omega_j_key),
        measured_up_(measured_up),
        interpolator_(interpolator) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GPInterpolatedVerticalPositionFactor(*this)));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, const gtsam::Vector3& vel_i,
                              const gtsam::Vector3& omega_i, const gtsam::Pose3& pose_j,
                              const gtsam::Vector3& vel_j, const gtsam::Vector3& omega_j,
                              boost::optional<gtsam::Matrix&> H1 = boost::none,
                              boost::optional<gtsam::Matrix&> H2 = boost::none,
                              boost::optional<gtsam::Matrix&> H3 = boost::none,
                              boost::optional<gtsam::Matrix&> H4 = boost::none,
                              boost::optional<gtsam::Matrix&> H5 = boost::none,
                              boost::optional<gtsam::Matrix&> H6 = boost::none) const override {
    using boost::placeholders::_1;
    using boost::placeholders::_2;
    using boost::placeholders::_3;
    using boost::placeholders::_4;
    using boost::placeholders::_5;
    using boost::placeholders::_6;

    if (H1) {
      *H1 = gtsam::numericalDerivative61<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H2) {
      *H2 = gtsam::numericalDerivative62<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H3) {
      *H3 = gtsam::numericalDerivative63<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H4) {
      *H4 = gtsam::numericalDerivative64<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H5) {
      *H5 = gtsam::numericalDerivative65<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H6) {
      *H6 = gtsam::numericalDerivative66<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }

    return EvaluateErrorNoJacobians(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
  }

 private:
  [[nodiscard]] gtsam::Vector1 EvaluateErrorNoJacobians(
      const gtsam::Pose3& pose_i, const gtsam::Vector3& vel_i,
      const gtsam::Vector3& omega_i, const gtsam::Pose3& pose_j,
      const gtsam::Vector3& vel_j, const gtsam::Vector3& omega_j) const {
    const gtsam::Pose3 interpolated_pose =
      interpolator_.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
    return gtsam::Vector1(interpolated_pose.translation().z() - measured_up_);
  }

  double measured_up_ = 0.0;
  offline_lc_minimal::gp::GPWNOJInterpolator interpolator_;
};

class GPInterpolatedVerticalEnvelopeFactor final
    : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
 public:
  GPInterpolatedVerticalEnvelopeFactor(
    gtsam::Key pose_i_key,
    gtsam::Key vel_i_key,
    gtsam::Key omega_i_key,
    gtsam::Key pose_j_key,
    gtsam::Key vel_j_key,
    gtsam::Key omega_j_key,
    double measured_up,
    double half_width_m,
    const offline_lc_minimal::gp::GPWNOJInterpolator &interpolator,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                 gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
          model, pose_i_key, vel_i_key, omega_i_key, pose_j_key, vel_j_key, omega_j_key),
        measured_up_(measured_up),
        half_width_m_(half_width_m),
        interpolator_(interpolator) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GPInterpolatedVerticalEnvelopeFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none,
    boost::optional<gtsam::Matrix &> H4 = boost::none,
    boost::optional<gtsam::Matrix &> H5 = boost::none,
    boost::optional<gtsam::Matrix &> H6 = boost::none) const override {
    using boost::placeholders::_1;
    using boost::placeholders::_2;
    using boost::placeholders::_3;
    using boost::placeholders::_4;
    using boost::placeholders::_5;
    using boost::placeholders::_6;

    if (H1) {
      *H1 = gtsam::numericalDerivative61<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H2) {
      *H2 = gtsam::numericalDerivative62<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H3) {
      *H3 = gtsam::numericalDerivative63<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H4) {
      *H4 = gtsam::numericalDerivative64<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H5) {
      *H5 = gtsam::numericalDerivative65<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H6) {
      *H6 = gtsam::numericalDerivative66<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }

    return EvaluateErrorNoJacobians(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
  }

 private:
  [[nodiscard]] gtsam::Vector1 EvaluateErrorNoJacobians(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j) const {
    const gtsam::Pose3 interpolated_pose =
      interpolator_.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
    const double raw_residual_m = interpolated_pose.translation().z() - measured_up_;
    return gtsam::Vector1(VerticalEnvelopeResidual(raw_residual_m, half_width_m_));
  }

  double measured_up_ = 0.0;
  double half_width_m_ = 0.0;
  offline_lc_minimal::gp::GPWNOJInterpolator interpolator_;
};

class GPInterpolatedVerticalEnvelopeCenterPullFactor final
    : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
 public:
  GPInterpolatedVerticalEnvelopeCenterPullFactor(
    gtsam::Key pose_i_key,
    gtsam::Key vel_i_key,
    gtsam::Key omega_i_key,
    gtsam::Key pose_j_key,
    gtsam::Key vel_j_key,
    gtsam::Key omega_j_key,
    double measured_up,
    double half_width_m,
    double deadband_m,
    const offline_lc_minimal::gp::GPWNOJInterpolator &interpolator,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                 gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
          model, pose_i_key, vel_i_key, omega_i_key, pose_j_key, vel_j_key, omega_j_key),
        measured_up_(measured_up),
        half_width_m_(half_width_m),
        deadband_m_(deadband_m),
        interpolator_(interpolator) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GPInterpolatedVerticalEnvelopeCenterPullFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none,
    boost::optional<gtsam::Matrix &> H4 = boost::none,
    boost::optional<gtsam::Matrix &> H5 = boost::none,
    boost::optional<gtsam::Matrix &> H6 = boost::none) const override {
    using boost::placeholders::_1;
    using boost::placeholders::_2;
    using boost::placeholders::_3;
    using boost::placeholders::_4;
    using boost::placeholders::_5;
    using boost::placeholders::_6;

    if (H1) {
      *H1 = gtsam::numericalDerivative61<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeCenterPullFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H2) {
      *H2 = gtsam::numericalDerivative62<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeCenterPullFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H3) {
      *H3 = gtsam::numericalDerivative63<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeCenterPullFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H4) {
      *H4 = gtsam::numericalDerivative64<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeCenterPullFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H5) {
      *H5 = gtsam::numericalDerivative65<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeCenterPullFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H6) {
      *H6 = gtsam::numericalDerivative66<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalEnvelopeCenterPullFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }

    return EvaluateErrorNoJacobians(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
  }

 private:
  [[nodiscard]] gtsam::Vector1 EvaluateErrorNoJacobians(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j) const {
    const gtsam::Pose3 interpolated_pose =
      interpolator_.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
    const double raw_residual_m = interpolated_pose.translation().z() - measured_up_;
    return gtsam::Vector1(VerticalEnvelopeCenterResidual(raw_residual_m, half_width_m_, deadband_m_));
  }

  double measured_up_ = 0.0;
  double half_width_m_ = 0.0;
  double deadband_m_ = 0.0;
  offline_lc_minimal::gp::GPWNOJInterpolator interpolator_;
};

class GPInterpolatedVerticalEnvelopeLatentReferenceFactor final
    : public gtsam::NoiseModelFactor {
 public:
  GPInterpolatedVerticalEnvelopeLatentReferenceFactor(
    gtsam::Key pose_i_key,
    gtsam::Key vel_i_key,
    gtsam::Key omega_i_key,
    gtsam::Key pose_j_key,
    gtsam::Key vel_j_key,
    gtsam::Key omega_j_key,
    gtsam::Key reference_key,
    double half_width_m,
    const offline_lc_minimal::gp::GPWNOJInterpolator &interpolator,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor(
          model,
          BuildKeys(
            pose_i_key,
            vel_i_key,
            omega_i_key,
            pose_j_key,
            vel_j_key,
            omega_j_key,
            reference_key)),
        half_width_m_(half_width_m),
        interpolator_(interpolator) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(
        new GPInterpolatedVerticalEnvelopeLatentReferenceFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector unwhitenedError(
    const gtsam::Values &values,
    boost::optional<std::vector<gtsam::Matrix> &> h = boost::none) const override {
    const auto &pose_i = values.at<gtsam::Pose3>(keys()[0]);
    const auto &vel_i = values.at<gtsam::Vector3>(keys()[1]);
    const auto &omega_i = values.at<gtsam::Vector3>(keys()[2]);
    const auto &pose_j = values.at<gtsam::Pose3>(keys()[3]);
    const auto &vel_j = values.at<gtsam::Vector3>(keys()[4]);
    const auto &omega_j = values.at<gtsam::Vector3>(keys()[5]);
    const double reference_up_m = values.at<double>(keys()[6]);
    if (h) {
      FillNumericalJacobians(
        pose_i,
        vel_i,
        omega_i,
        pose_j,
        vel_j,
        omega_j,
        reference_up_m,
        *h);
    }
    return Evaluate(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, reference_up_m);
  }

 private:
  static gtsam::KeyVector BuildKeys(
    const gtsam::Key pose_i_key,
    const gtsam::Key vel_i_key,
    const gtsam::Key omega_i_key,
    const gtsam::Key pose_j_key,
    const gtsam::Key vel_j_key,
    const gtsam::Key omega_j_key,
    const gtsam::Key reference_key) {
    return gtsam::KeyVector{
      pose_i_key,
      vel_i_key,
      omega_i_key,
      pose_j_key,
      vel_j_key,
      omega_j_key,
      reference_key};
  }

  [[nodiscard]] gtsam::Vector1 Evaluate(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j,
    const double reference_up_m) const {
    const gtsam::Pose3 interpolated_pose =
      interpolator_.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
    const double raw_residual_m = interpolated_pose.translation().z() - reference_up_m;
    return gtsam::Vector1(VerticalEnvelopeResidual(raw_residual_m, half_width_m_));
  }

  void FillNumericalJacobians(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j,
    const double reference_up_m,
    std::vector<gtsam::Matrix> &h) const {
    h.resize(keys().size());
    h[0] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
      [&](const gtsam::Pose3 &candidate) {
        return Evaluate(candidate, vel_i, omega_i, pose_j, vel_j, omega_j, reference_up_m);
      },
      pose_i,
      1e-5);
    h[1] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
      [&](const gtsam::Vector3 &candidate) {
        return Evaluate(pose_i, candidate, omega_i, pose_j, vel_j, omega_j, reference_up_m);
      },
      vel_i,
      1e-5);
    h[2] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
      [&](const gtsam::Vector3 &candidate) {
        return Evaluate(pose_i, vel_i, candidate, pose_j, vel_j, omega_j, reference_up_m);
      },
      omega_i,
      1e-5);
    h[3] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
      [&](const gtsam::Pose3 &candidate) {
        return Evaluate(pose_i, vel_i, omega_i, candidate, vel_j, omega_j, reference_up_m);
      },
      pose_j,
      1e-5);
    h[4] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
      [&](const gtsam::Vector3 &candidate) {
        return Evaluate(pose_i, vel_i, omega_i, pose_j, candidate, omega_j, reference_up_m);
      },
      vel_j,
      1e-5);
    h[5] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
      [&](const gtsam::Vector3 &candidate) {
        return Evaluate(pose_i, vel_i, omega_i, pose_j, vel_j, candidate, reference_up_m);
      },
      omega_j,
      1e-5);
    const gtsam::Pose3 interpolated_pose =
      interpolator_.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
    const double raw_residual_m = interpolated_pose.translation().z() - reference_up_m;
    h[6] = -VerticalEnvelopeResidualSlope(raw_residual_m, half_width_m_) *
           gtsam::Matrix::Identity(1, 1);
  }

  double half_width_m_ = 0.0;
  offline_lc_minimal::gp::GPWNOJInterpolator interpolator_;
};

class GPInterpolatedVerticalEnvelopeLatentCenterPullFactor final
    : public gtsam::NoiseModelFactor {
 public:
  GPInterpolatedVerticalEnvelopeLatentCenterPullFactor(
    gtsam::Key pose_i_key,
    gtsam::Key vel_i_key,
    gtsam::Key omega_i_key,
    gtsam::Key pose_j_key,
    gtsam::Key vel_j_key,
    gtsam::Key omega_j_key,
    gtsam::Key reference_key,
    double half_width_m,
    double deadband_m,
    const offline_lc_minimal::gp::GPWNOJInterpolator &interpolator,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor(
          model,
          BuildKeys(
            pose_i_key,
            vel_i_key,
            omega_i_key,
            pose_j_key,
            vel_j_key,
            omega_j_key,
            reference_key)),
        half_width_m_(half_width_m),
        deadband_m_(deadband_m),
        interpolator_(interpolator) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(
        new GPInterpolatedVerticalEnvelopeLatentCenterPullFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector unwhitenedError(
    const gtsam::Values &values,
    boost::optional<std::vector<gtsam::Matrix> &> h = boost::none) const override {
    const auto &pose_i = values.at<gtsam::Pose3>(keys()[0]);
    const auto &vel_i = values.at<gtsam::Vector3>(keys()[1]);
    const auto &omega_i = values.at<gtsam::Vector3>(keys()[2]);
    const auto &pose_j = values.at<gtsam::Pose3>(keys()[3]);
    const auto &vel_j = values.at<gtsam::Vector3>(keys()[4]);
    const auto &omega_j = values.at<gtsam::Vector3>(keys()[5]);
    const double reference_up_m = values.at<double>(keys()[6]);
    if (h) {
      FillNumericalJacobians(
        pose_i,
        vel_i,
        omega_i,
        pose_j,
        vel_j,
        omega_j,
        reference_up_m,
        *h);
    }
    return Evaluate(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, reference_up_m);
  }

 private:
  static gtsam::KeyVector BuildKeys(
    const gtsam::Key pose_i_key,
    const gtsam::Key vel_i_key,
    const gtsam::Key omega_i_key,
    const gtsam::Key pose_j_key,
    const gtsam::Key vel_j_key,
    const gtsam::Key omega_j_key,
    const gtsam::Key reference_key) {
    return gtsam::KeyVector{
      pose_i_key,
      vel_i_key,
      omega_i_key,
      pose_j_key,
      vel_j_key,
      omega_j_key,
      reference_key};
  }

  [[nodiscard]] gtsam::Vector1 Evaluate(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j,
    const double reference_up_m) const {
    const gtsam::Pose3 interpolated_pose =
      interpolator_.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
    const double raw_residual_m = interpolated_pose.translation().z() - reference_up_m;
    return gtsam::Vector1(
      VerticalEnvelopeCenterResidual(raw_residual_m, half_width_m_, deadband_m_));
  }

  void FillNumericalJacobians(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j,
    const double reference_up_m,
    std::vector<gtsam::Matrix> &h) const {
    h.resize(keys().size());
    h[0] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
      [&](const gtsam::Pose3 &candidate) {
        return Evaluate(candidate, vel_i, omega_i, pose_j, vel_j, omega_j, reference_up_m);
      },
      pose_i,
      1e-5);
    h[1] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
      [&](const gtsam::Vector3 &candidate) {
        return Evaluate(pose_i, candidate, omega_i, pose_j, vel_j, omega_j, reference_up_m);
      },
      vel_i,
      1e-5);
    h[2] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
      [&](const gtsam::Vector3 &candidate) {
        return Evaluate(pose_i, vel_i, candidate, pose_j, vel_j, omega_j, reference_up_m);
      },
      omega_i,
      1e-5);
    h[3] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
      [&](const gtsam::Pose3 &candidate) {
        return Evaluate(pose_i, vel_i, omega_i, candidate, vel_j, omega_j, reference_up_m);
      },
      pose_j,
      1e-5);
    h[4] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
      [&](const gtsam::Vector3 &candidate) {
        return Evaluate(pose_i, vel_i, omega_i, pose_j, candidate, omega_j, reference_up_m);
      },
      vel_j,
      1e-5);
    h[5] = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Vector3>(
      [&](const gtsam::Vector3 &candidate) {
        return Evaluate(pose_i, vel_i, omega_i, pose_j, vel_j, candidate, reference_up_m);
      },
      omega_j,
      1e-5);
    const gtsam::Pose3 interpolated_pose =
      interpolator_.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
    const double raw_residual_m = interpolated_pose.translation().z() - reference_up_m;
    h[6] = -VerticalEnvelopeCenterResidualSlope(raw_residual_m, half_width_m_, deadband_m_) *
           gtsam::Matrix::Identity(1, 1);
  }

  double half_width_m_ = 0.0;
  double deadband_m_ = 0.0;
  offline_lc_minimal::gp::GPWNOJInterpolator interpolator_;
};

}  // namespace offline_lc_minimal::factor
