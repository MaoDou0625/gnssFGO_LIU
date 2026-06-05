#pragma once

#include <cmath>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

[[nodiscard]] inline double NormalizeAngleRad(const double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

[[nodiscard]] inline double RelativeYawRad(
  const gtsam::Rot3 &rotation_i,
  const gtsam::Rot3 &rotation_j) {
  return NormalizeAngleRad(rotation_i.between(rotation_j).ypr().x());
}

class RollPitchReferenceFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  RollPitchReferenceFactor(
    gtsam::Key pose_key,
    const gtsam::Rot3 &reference_rotation,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        reference_pitch_rad_(reference_rotation.ypr().y()),
        reference_roll_rad_(reference_rotation.ypr().z()) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new RollPitchReferenceFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h_pose = boost::none) const override {
    const auto error_function = [this](const gtsam::Pose3 &candidate_pose) {
      return Evaluate(candidate_pose);
    };
    if (h_pose) {
      *h_pose = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(error_function, pose);
    }
    return Evaluate(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector2 Evaluate(const gtsam::Pose3 &pose) const {
    const auto ypr = pose.rotation().ypr();
    return (gtsam::Vector2() <<
      NormalizeAngleRad(ypr.z() - reference_roll_rad_),
      NormalizeAngleRad(ypr.y() - reference_pitch_rad_)).finished();
  }

  double reference_pitch_rad_ = 0.0;
  double reference_roll_rad_ = 0.0;
};

class TiltReferenceFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  TiltReferenceFactor(
    gtsam::Key pose_key,
    const gtsam::Rot3 &reference_rotation,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        reference_nav_z_body_(NavZBody(reference_rotation)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new TiltReferenceFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h_pose = boost::none) const override {
    const auto error_function = [this](const gtsam::Pose3 &candidate_pose) {
      return Evaluate(candidate_pose);
    };
    if (h_pose) {
      *h_pose = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(error_function, pose);
    }
    return Evaluate(pose);
  }

 private:
  [[nodiscard]] static gtsam::Vector3 NavZBody(const gtsam::Rot3 &rotation) {
    return rotation.unrotate(gtsam::Vector3::UnitZ());
  }

  [[nodiscard]] gtsam::Vector2 Evaluate(const gtsam::Pose3 &pose) const {
    const gtsam::Vector3 nav_z_body = NavZBody(pose.rotation());
    return (gtsam::Vector2() <<
      nav_z_body.x() - reference_nav_z_body_.x(),
      nav_z_body.y() - reference_nav_z_body_.y()).finished();
  }

  gtsam::Vector3 reference_nav_z_body_ = gtsam::Vector3::UnitZ();
};

class YawReferenceFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  YawReferenceFactor(
    gtsam::Key pose_key,
    const gtsam::Rot3 &reference_rotation,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        reference_yaw_rad_(reference_rotation.ypr().x()) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new YawReferenceFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h_pose = boost::none) const override {
    const auto error_function = [this](const gtsam::Pose3 &candidate_pose) {
      return Evaluate(candidate_pose);
    };
    if (h_pose) {
      *h_pose = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(error_function, pose);
    }
    return Evaluate(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector1 Evaluate(const gtsam::Pose3 &pose) const {
    return gtsam::Vector1(NormalizeAngleRad(pose.rotation().ypr().x() - reference_yaw_rad_));
  }

  double reference_yaw_rad_ = 0.0;
};

class RelativeYawReferenceFactor final : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  RelativeYawReferenceFactor(
    gtsam::Key pose_i_key,
    gtsam::Key pose_j_key,
    const gtsam::Rot3 &reference_rotation_i,
    const gtsam::Rot3 &reference_rotation_j,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, pose_i_key, pose_j_key),
        reference_delta_yaw_rad_(RelativeYawRad(reference_rotation_i, reference_rotation_j)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new RelativeYawReferenceFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Pose3 &pose_j,
    boost::optional<gtsam::Matrix &> h_pose_i = boost::none,
    boost::optional<gtsam::Matrix &> h_pose_j = boost::none) const override {
    const auto error_function = [this](
                                  const gtsam::Pose3 &candidate_pose_i,
                                  const gtsam::Pose3 &candidate_pose_j) {
      return Evaluate(candidate_pose_i, candidate_pose_j);
    };
    if (h_pose_i) {
      *h_pose_i = gtsam::numericalDerivative21<gtsam::Vector1, gtsam::Pose3, gtsam::Pose3>(
        error_function,
        pose_i,
        pose_j);
    }
    if (h_pose_j) {
      *h_pose_j = gtsam::numericalDerivative22<gtsam::Vector1, gtsam::Pose3, gtsam::Pose3>(
        error_function,
        pose_i,
        pose_j);
    }
    return Evaluate(pose_i, pose_j);
  }

 private:
  [[nodiscard]] gtsam::Vector1 Evaluate(
    const gtsam::Pose3 &pose_i,
    const gtsam::Pose3 &pose_j) const {
    return gtsam::Vector1(
      NormalizeAngleRad(RelativeYawRad(pose_i.rotation(), pose_j.rotation()) - reference_delta_yaw_rad_));
  }

  double reference_delta_yaw_rad_ = 0.0;
};

class RelativeRotationReferenceFactor final : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  RelativeRotationReferenceFactor(
    gtsam::Key pose_i_key,
    gtsam::Key pose_j_key,
    const gtsam::Rot3 &reference_rotation_i,
    const gtsam::Rot3 &reference_rotation_j,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, pose_i_key, pose_j_key),
        reference_delta_rotation_(reference_rotation_i.between(reference_rotation_j)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new RelativeRotationReferenceFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Pose3 &pose_j,
    boost::optional<gtsam::Matrix &> h_pose_i = boost::none,
    boost::optional<gtsam::Matrix &> h_pose_j = boost::none) const override {
    const auto error_function = [this](
                                  const gtsam::Pose3 &candidate_pose_i,
                                  const gtsam::Pose3 &candidate_pose_j) {
      return Evaluate(candidate_pose_i, candidate_pose_j);
    };
    if (h_pose_i) {
      *h_pose_i = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
        error_function,
        pose_i,
        pose_j);
    }
    if (h_pose_j) {
      *h_pose_j = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
        error_function,
        pose_i,
        pose_j);
    }
    return Evaluate(pose_i, pose_j);
  }

 private:
  [[nodiscard]] gtsam::Vector3 Evaluate(
    const gtsam::Pose3 &pose_i,
    const gtsam::Pose3 &pose_j) const {
    const gtsam::Rot3 optimized_delta_rotation =
      pose_i.rotation().between(pose_j.rotation());
    return gtsam::Rot3::Logmap(
      reference_delta_rotation_.between(optimized_delta_rotation));
  }

  gtsam::Rot3 reference_delta_rotation_;
};

}  // namespace offline_lc_minimal::factor
