#pragma once

#include <cmath>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

namespace offline_lc_minimal::gp {

inline gtsam::Matrix GetQc(const gtsam::SharedNoiseModel &qc_model) {
  auto *gaussian_model = dynamic_cast<gtsam::noiseModel::Gaussian *>(qc_model.get());
  return (gaussian_model->R().transpose() * gaussian_model->R()).inverse();
}

template <int Dim>
Eigen::Matrix<double, 3 * Dim, 3 * Dim> CalcQ3(const Eigen::Matrix<double, Dim, Dim> &qc, double tau) {
  return (Eigen::Matrix<double, 3 * Dim, 3 * Dim>() <<
            1.0 / 20.0 * std::pow(tau, 5.0) * qc, 1.0 / 8.0 * std::pow(tau, 4.0) * qc,
            1.0 / 6.0 * std::pow(tau, 3.0) * qc,
            1.0 / 8.0 * std::pow(tau, 4.0) * qc, 1.0 / 3.0 * std::pow(tau, 3.0) * qc,
            1.0 / 2.0 * std::pow(tau, 2.0) * qc,
            1.0 / 6.0 * std::pow(tau, 3.0) * qc, 1.0 / 2.0 * std::pow(tau, 2.0) * qc, tau * qc)
    .finished();
}

template <int Dim>
Eigen::Matrix<double, 3 * Dim, 3 * Dim> CalcQInv3(const Eigen::Matrix<double, Dim, Dim> &qc, double tau) {
  const Eigen::Matrix<double, Dim, Dim> qc_inv = qc.inverse();
  return (Eigen::Matrix<double, 3 * Dim, 3 * Dim>() <<
            720.0 * std::pow(tau, -5.0) * qc_inv, -360.0 * std::pow(tau, -4.0) * qc_inv,
            60.0 * std::pow(tau, -3.0) * qc_inv,
            -360.0 * std::pow(tau, -4.0) * qc_inv, 192.0 * std::pow(tau, -3.0) * qc_inv,
            -36.0 * std::pow(tau, -2.0) * qc_inv,
            60.0 * std::pow(tau, -3.0) * qc_inv, -36.0 * std::pow(tau, -2.0) * qc_inv,
            9.0 * std::pow(tau, -1.0) * qc_inv)
    .finished();
}

template <int Dim>
Eigen::Matrix<double, 3 * Dim, 3 * Dim> CalcPhi3(double tau) {
  return (Eigen::Matrix<double, 3 * Dim, 3 * Dim>() <<
            Eigen::Matrix<double, Dim, Dim>::Identity(), tau * Eigen::Matrix<double, Dim, Dim>::Identity(),
            0.5 * std::pow(tau, 2.0) * Eigen::Matrix<double, Dim, Dim>::Identity(),
            Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Identity(),
            tau * Eigen::Matrix<double, Dim, Dim>::Identity(),
            Eigen::Matrix<double, Dim, Dim>::Zero(), Eigen::Matrix<double, Dim, Dim>::Zero(),
            Eigen::Matrix<double, Dim, Dim>::Identity())
    .finished();
}

template <int Dim>
Eigen::Matrix<double, 3 * Dim, 3 * Dim> CalcLambda3(
  const Eigen::Matrix<double, Dim, Dim> &qc,
  double delta_t,
  double tau) {
  return CalcPhi3<Dim>(tau) -
         CalcQ3(qc, tau) * CalcPhi3<Dim>(delta_t - tau).transpose() * CalcQInv3(qc, delta_t) *
           CalcPhi3<Dim>(delta_t);
}

template <int Dim>
Eigen::Matrix<double, 3 * Dim, 3 * Dim> CalcPsi3(
  const Eigen::Matrix<double, Dim, Dim> &qc,
  double delta_t,
  double tau) {
  return CalcQ3(qc, tau) * CalcPhi3<Dim>(delta_t - tau).transpose() * CalcQInv3(qc, delta_t);
}

}  // namespace offline_lc_minimal::gp
