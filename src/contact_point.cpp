/**
 * @file
 * @copyright Copyright (c) 2022, LAAS-CNRS, Toward, PalRobotics
 * @brief
 */

#include "dynacom/contact_point.hpp"

#include "pinocchio/algorithm/geometry.hpp"

namespace dynacom {

ContactPoint::ContactPoint() {}
ContactPoint::ContactPoint(const ContactPointSettings& settings) {
  initialize(settings);
}
void ContactPoint::initialize(const ContactPointSettings& settings) {
  settings_ = settings;
  double mu = settings_.mu;

  // SIZES
  contactForce_.resize(3);
  regularization_A_.resize(3);
  regularization_b_.resize(3);
  unilaterality_A_.resize(1, 3);
  unilaterality_b_.resize(1);
  friction_A_.resize(4, 3);
  friction_b_.resize(4);
  newton_euler_A_.resize(6, 3);

  // Make matrices
  regularization_A_ << settings_.weights;
  regularization_b_.setZero();

  unilaterality_A_ << 0, 0, -1;
  unilaterality_b_.setZero();

  friction_A_.setZero();
  friction_A_.block<4, 1>(0, 2).setConstant(-mu);
  friction_A_.block<2, 2>(0, 0) << Eigen::Matrix2d::Identity();
  friction_A_.block<2, 2>(2, 0) << -Eigen::Matrix2d::Identity();
  friction_b_.setZero();

  newton_euler_A_.setZero();
  newton_euler_A_.block<3, 3>(0, 0).setIdentity();
  // Note: newton_euler must be updated before using it

  contactForce_.setZero();
}

void ContactPoint::setMu(const double& mu) {
  settings_.mu = mu;
  friction_A_.block<4, 1>(0, 2).setConstant(-mu);
}

void ContactPoint::setForceWeights(const Eigen::Vector3d& force_weights) {
  settings_.weights.head<3>() = force_weights;
  regularization_A_.head<3>() = force_weights;
}

void ContactPoint::updateNewtonEuler(const Eigen::Vector3d& CoM,
                                     const pinocchio::SE3& oMs) {
  /**
   * @brief Assuming that the orientation of the world frame is the identity.
   *
   */

  oMs_ = oMs;
  cMo_ = pinocchio::SE3(Eigen::Matrix3d::Identity(), -CoM);

  newton_euler_A_
      << (cMo_.act(oMs_)).toActionMatrixInverse().transpose().block<6, 3>(0, 0);
}

}  // namespace dynacom
