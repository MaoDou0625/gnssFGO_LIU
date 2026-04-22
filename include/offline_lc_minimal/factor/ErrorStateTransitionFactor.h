#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal::factor {

class ErrorStateTransitionFactor : public gtsam::NoiseModelFactor2<ErrorStateVector, ErrorStateVector> {
 public:
  ErrorStateTransitionFactor(
    gtsam::Key error_i_key,
    gtsam::Key error_j_key,
    ErrorStateMatrix phi,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<ErrorStateVector, ErrorStateVector>(noise_model, error_i_key, error_j_key),
        phi_(std::move(phi)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new ErrorStateTransitionFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const ErrorStateVector &error_i,
    const ErrorStateVector &error_j,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    if (h1) {
      *h1 = -phi_;
    }
    if (h2) {
      *h2 = ErrorStateMatrix::Identity();
    }
    return error_j - phi_ * error_i;
  }

 private:
  ErrorStateMatrix phi_;
};

}  // namespace offline_lc_minimal::factor
