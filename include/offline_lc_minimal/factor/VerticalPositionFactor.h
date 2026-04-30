#pragma once

#include <boost/bind/bind.hpp>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalPositionFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  VerticalPositionFactor(
    gtsam::Key pose_key,
    const double measured_up_m,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
        measured_up_m_(measured_up_m) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalPositionFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h = boost::none) const override {
    if (h) {
      using boost::placeholders::_1;
      *h = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
        boost::bind(&VerticalPositionFactor::EvaluateErrorNoJacobians, this, _1),
        pose,
        1e-6);
    }

    return EvaluateErrorNoJacobians(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector1 EvaluateErrorNoJacobians(const gtsam::Pose3 &pose) const {
    return (gtsam::Vector1() << pose.translation().z() - measured_up_m_).finished();
  }

  double measured_up_m_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
